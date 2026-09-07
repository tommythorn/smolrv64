`default_nettype none

// Shared FP execution unit: a thin wrapper around CVFPU (fpnew RV64D). One op in flight,
// result carries its writeback routing in the tag.
//
// WHY THIS TALKS TO fpnew_top DIRECTLY. It used to go through smolrv64_cvfpu, which is a
// TWO-CLOCK-DOMAIN wrapper: a toggle request/response protocol with an ASYNC_REG two-flop
// synchroniser in each direction, so a core-clock op costs
//
//    1 (req toggle) + 2 (sync) + 1 (FPU_IDLE->ISSUE) + 5 (accept + PipeRegs)
//                   + 1 (resp toggle) + 2 (sync) + 1 (latch)  =  ~13 cycles
//
// and hardware measured exactly 14.02 stall cycles per FP op. But BOTH instantiations of
// this module tied fpu_clock to clk, so every one of those ~9 synchroniser cycles paid for a
// clock crossing that does not exist. The FPU was 45% of all GB5 cycles at the time, and two
// thirds of that was this handshake. (smolrv64_cvfpu itself left with the scalar core.)
module fp_unit #(parameter TAGW = 24,
                 parameter int unsigned PIPE_REGS = 4,
                 // OPS IN FLIGHT. fpnew is a PIPELINED unit (PipeRegs = PIPE_REGS), able to
                 // accept an op every cycle, but this wrapper held exactly one and so
                 // delivered one op per round trip. Measured on workloads/fpbench with eight
                 // INDEPENDENT chains -- as much ILP as the shape allows -- that was 4.00
                 // cycles/op against 8.00 for a serial chain: an overlap of 1.99x where the
                 // unit's own latency (PIPE_REGS) should have allowed far more.
                 //
                 // The default stays 1 (the retired sharded core's value; ooo2_core passes 4):
                 // at NFLIGHT=1 every expression below reduces to the single-slot wrapper it
                 // replaced. Only a caller that can route results by tag
                 // may raise it -- with more than one in flight, results come back TAGGED and
                 // not necessarily in issue order, because fpnew's op groups (ADDMUL,
                 // DIVSQRT, NONCOMP, CONV) have different latencies.
                 parameter int unsigned NFLIGHT = 1)
   (input  wire             clk,
    input  wire             reset,
    // issue (1 op/cycle when iss_ready)
    input  wire             iss_valid,
    output wire             iss_ready,
    input  wire [3:0]       iss_op,
    input  wire             iss_op_mod,
    input  wire [2:0]       iss_src_fmt,
    input  wire [2:0]       iss_dst_fmt,
    input  wire [1:0]       iss_int_fmt,
    input  wire [2:0]       iss_rnd,
    input  wire [191:0]     iss_operands,   // {op2, op1, op0}
    input  wire [TAGW-1:0]  iss_tag,
    // result
    output wire             res_valid,
    input  wire             res_ready,
    output wire [63:0]      res_data,
    output wire [4:0]       res_fflags,
    output wire [TAGW-1:0]  res_tag,
    input  wire             flush,          // abort in-flight op(s) (squash)
    output wire             busy);

   localparam fpnew_pkg::fpu_features_t Features = fpnew_pkg::RV64D;
   localparam fpnew_pkg::fpu_implementation_t Implementation = '{
      PipeRegs:   '{default: PIPE_REGS},
      UnitTypes:  '{'{default: fpnew_pkg::MERGED},   // ADDMUL
                    '{default: fpnew_pkg::MERGED},   // DIVSQRT
                    '{default: fpnew_pkg::PARALLEL}, // NONCOMP
                    '{default: fpnew_pkg::MERGED}},  // CONV
      PipeConfig: fpnew_pkg::DISTRIBUTED
   };

   // The held request, or the one arriving right now -- see fpn_valid below. Mutually
   // exclusive: iss_ready is low while req_v_q is set, so `fire` cannot occur then.
   wire [191:0]    sel_ops = req_v_q ? req_ops_q : iss_operands;
   wire [3:0]      sel_op  = req_v_q ? req_op_q  : iss_op;
   wire            sel_mod = req_v_q ? req_mod_q : iss_op_mod;
   wire [2:0]      sel_sf  = req_v_q ? req_sf_q  : iss_src_fmt;
   wire [2:0]      sel_df  = req_v_q ? req_df_q  : iss_dst_fmt;
   wire [1:0]      sel_if  = req_v_q ? req_if_q  : iss_int_fmt;
   wire [2:0]      sel_rnd = req_v_q ? req_rnd_q : iss_rnd;
   wire [TAGW-1:0] sel_tag = req_v_q ? req_tag_q : iss_tag;

   logic [2:0][63:0] ops;
   assign ops[0] = sel_ops[63:0];
   assign ops[1] = sel_ops[127:64];
   assign ops[2] = sel_ops[191:128];

   logic                 fpn_in_ready, fpn_out_valid, fpn_busy, fpn_early;
   logic [63:0]          fpn_result;
   fpnew_pkg::status_t   fpn_status;
   logic [TAGW-1:0]      fpn_tag;

   // ONE OP IN FLIGHT, which is the contract both callers already rely on (ooo2_core asserts
   // it, exec_shard gates on busy). The result is REGISTERED rather than passed straight
   // out: res_valid feeds m_done in ooo2_core, and fpnew's combinational output there would
   // drag the whole FP datapath into the completion cone.
   //
   // EVERY OUTPUT OF THIS MODULE IS A REGISTER, and iss_ready especially. fpnew's
   // in_ready_o is combinational in in_valid_i, and exec_shard.v:273 feeds iss_ready
   // straight back into the shard's issue decision
   //     munit_busy = mbusy | dbusy | fpu_inflight | fpu_busyo | ~fp_iss_ready
   // which produces fp_start -- so exposing fpnew's ready closes a loop through the
   // scheduler. run-vl-tests.sh builds with -Wno-UNOPTFLAT, so that loop does not error;
   // the loop is simply settled wrong, and the damage lands on whatever else that shard
   // was issuing: 11 atomics tests and rv64uc-v-rvc failed, none of which touch FP.
   // The request is therefore HELD here until fpnew takes it.
   logic                 req_v_q;
   logic [191:0]         req_ops_q;
   logic [3:0]           req_op_q;
   logic                 req_mod_q;
   logic [2:0]           req_sf_q, req_df_q, req_rnd_q;
   logic [1:0]           req_if_q;
   logic [TAGW-1:0]      req_tag_q;
   localparam int unsigned FW = (NFLIGHT <= 1) ? 1 : $clog2(NFLIGHT + 1);
   logic [FW-1:0]        nflight_q;          // ops accepted by fpnew, not yet returned
   logic                 out_valid_q;
   logic [63:0]          result_q;
   logic [4:0]           fflags_q;
   logic [TAGW-1:0]      tag_q;
   initial begin nflight_q = '0; out_valid_q = 1'b0; req_v_q = 1'b0; end

   // Still REGISTERS ONLY. fpnew's in_ready_o is combinational in in_valid_i and
   // exec_shard.v feeds iss_ready into its issue decision, so exposing it closes a loop --
   // that cost 11 atomics and rv64uc-v-rvc once, hidden behind -Wno-UNOPTFLAT.
   // At NFLIGHT=1 out_blk is `out_valid_q`, i.e. the original expression unchanged. Above
   // 1, a result being delivered no longer blocks issue when the consumer takes it in the
   // same cycle, which is the whole point.
   wire out_blk = out_valid_q & ((NFLIGHT <= 1) ? 1'b1 : ~res_ready);
   assign iss_ready  = ~req_v_q & (nflight_q != FW'(NFLIGHT)) & ~out_blk;
   wire   fire       = iss_valid & iss_ready;                   // core hands the op over
   // STRAIGHT THROUGH IN THE COMMON CASE. The request register is a FALLBACK for the cycle
   // fpnew declines, not a stage every op pays -- routing every op through it cost a cycle
   // on all of them to cover a case that needs one op held. iss_ready stays registers-only,
   // so the loop through exec_shard's issue decision stays broken; fpn_valid may depend on
   // iss_valid without closing anything, because fpn_take feeds only registers.
   wire   fpn_valid  = req_v_q | fire;
   wire   fpn_take   = fpn_valid & fpn_in_ready;
   assign res_valid  = out_valid_q;
   assign res_data   = result_q;
   assign res_fflags = fflags_q;
   assign res_tag    = tag_q;
   assign busy       = req_v_q | (nflight_q != '0) | out_valid_q;

   always_ff @(posedge clk) begin
      if (reset | flush) begin
         req_v_q <= 1'b0; nflight_q <= '0; out_valid_q <= 1'b0;
      end else begin
         if (out_valid_q & res_ready) out_valid_q <= 1'b0;
         if (fpn_take) req_v_q <= 1'b0;
         if (fire & ~fpn_in_ready) begin        // fpnew declined: hold it here
            req_v_q   <= 1'b1;    req_ops_q <= iss_operands; req_op_q  <= iss_op;
            req_mod_q <= iss_op_mod; req_sf_q <= iss_src_fmt; req_df_q <= iss_dst_fmt;
            req_if_q  <= iss_int_fmt; req_rnd_q <= iss_rnd;  req_tag_q <= iss_tag;
         end
         // NOT ALL OPS ARE PIPELINED. NONCOMP (MINMAX/SGNJ/CMP/CLASSIFY) and parts of CONV
         // come back COMBINATIONALLY -- fpnew asserts out_valid in the very cycle it accepts
         // the op, PipeRegs notwithstanding. The old FSM had an explicit branch for this in
         // FPU_ISSUE; dropping it failed exactly the 16 fcvt/fmin/recoding tests and nothing
         // else. `else if` is the whole fix: a same-cycle result never sets inflight_q.
         // Counter, not a flag. Both same-cycle cases net to zero and need no special
         // arm: one op returning while another is accepted, and a NONCOMP/CONV op that
         // returns in the very cycle it is accepted (the `else if` this replaces).
         if (fpn_out_valid & ~fpn_take)      nflight_q <= nflight_q - 1'b1;
         else if (fpn_take & ~fpn_out_valid) nflight_q <= nflight_q + 1'b1;
         if (fpn_out_valid) begin
            out_valid_q <= 1'b1;
            result_q    <= fpn_result;
            fflags_q    <= {fpn_status.NV, fpn_status.DZ, fpn_status.OF,
                            fpn_status.UF, fpn_status.NX};
            tag_q       <= fpn_tag;
         end
      end
   end

   // Invariants (docs/rtl-rules.md A1). Both say the same thing from opposite ends: the
   // one-op-at-a-time contract holds, so no result can arrive unowned or be overwritten.
   always_ff @(posedge clk) if (!reset) begin
      if (fpn_out_valid & (nflight_q == '0) & ~fpn_take)
         $fatal(1, "fp_unit: fpnew produced a result with no op in flight");
      if (fpn_out_valid & out_valid_q & ~res_ready)
         $fatal(1, "fp_unit: result arrived while the previous one was unconsumed");
   end

   fpnew_top #(
      .Features       ( Features       ),
      .Implementation ( Implementation ),
      .DivSqrtSel     ( fpnew_pkg::THMULTI ),
      .TagType        ( logic [TAGW-1:0] )
   ) u_fpnew (
      .clk_i         ( clk ),
      .rst_ni        ( ~reset ),
      .operands_i    ( ops ),
      .rnd_mode_i    ( fpnew_pkg::roundmode_e'(sel_rnd) ),
      .op_i          ( fpnew_pkg::operation_e'(sel_op) ),
      .op_mod_i      ( sel_mod ),
      .src_fmt_i     ( fpnew_pkg::fp_format_e'(sel_sf) ),
      .dst_fmt_i     ( fpnew_pkg::fp_format_e'(sel_df) ),
      .int_fmt_i     ( fpnew_pkg::int_format_e'(sel_if) ),
      .vectorial_op_i( 1'b0 ),
      .tag_i         ( sel_tag ),
      .simd_mask_i   ( '1 ),
      .in_valid_i    ( fpn_valid ),
      .in_ready_o    ( fpn_in_ready ),
      .flush_i       ( flush ),
      .result_o      ( fpn_result ),
      .status_o      ( fpn_status ),
      .tag_o         ( fpn_tag ),
      .out_valid_o   ( fpn_out_valid ),
      .out_ready_i   ( 1'b1 ),
      .busy_o        ( fpn_busy ),
      .early_valid_o ( fpn_early )
   );
endmodule

`default_nettype wire
