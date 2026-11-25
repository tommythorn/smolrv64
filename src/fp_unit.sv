`default_nettype none

// Shared FP execution unit: CVFPU (fpnew RV64D) for every op but division and square root, which
// fp_divsqrt does. Results carry their writeback routing in the tag; fpnew's take the result
// register first, and fp_divsqrt holds its answer while one does.
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
                 parameter RDV_BIT = TAGW - 2,      // the tag's rd_v
                 parameter FSHARD_BIT = 0,          // the tag's shard MSB: set for an f-register
                 parameter int unsigned PIPE_REGS = 4,
                 // OPS IN FLIGHT. fpnew is a PIPELINED unit (PipeRegs = PIPE_REGS), able to
                 // accept an op every cycle, but this wrapper held exactly one and so
                 // delivered one op per round trip. Measured on workloads/fpbench with eight
                 // INDEPENDENT chains -- as much ILP as the shape allows -- that was 4.00
                 // cycles/op against 8.00 for a serial chain: an overlap of 1.99x where the
                 // unit's own latency (PIPE_REGS) should have allowed far more.
                 //
                 // The default stays 1 (the retired sharded core's value; smolrv64_core passes 4):
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
    output wire             busy,
    output logic            err,            // integrity log (registered): an orphan or overwritten result
    // the result's destination kind, flops beside res_valid: an f-register's (res_fp) or an
    // integer register's (res_int, the tag's rd_v set and its register in an integer shard).
    // The core's landing selects on these directly; a decode of the tag there would sit one
    // level in front of the FE wake bus, which fans out to every scheduler entry.
    output wire             res_fp,
    output wire             res_int,
    output wire  [15:0]     dbg);           // wait state for the board's wedge ILA (core_dbg)

   localparam fpnew_pkg::fpu_features_t Features = fpnew_pkg::RV64D;
   localparam fpnew_pkg::fpu_implementation_t Implementation = '{
      // Every format of every op group: a pattern per group, because a group's entry is a PACKED
      // array of formats and '{default: N} at the outer level assigns N to that packed value whole
      // (only its last element). Verilator does exactly that and simulated every group without its
      // pipeline registers, 5 cycles short of the board's FMA; Vivado gave every element N.
      PipeRegs:   '{'{default: PIPE_REGS}, '{default: PIPE_REGS}, '{default: PIPE_REGS}, '{default: PIPE_REGS}},
      UnitTypes:  '{'{default: fpnew_pkg::MERGED},   // ADDMUL
                    '{default: fpnew_pkg::DISABLED}, // DIVSQRT: fp_divsqrt
                    '{default: fpnew_pkg::DISABLED}, // NONCOMP: compares, sign injection, classify
                                                     // and FMIN/FMAX run in the core
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
   // division and square root go to fp_divsqrt
   wire                  sel_ds = (sel_op == 4'(fpnew_pkg::DIV)) | (sel_op == 4'(fpnew_pkg::SQRT));
   wire                  ds_in_ready, ds_out_valid, ds_busy;
   wire [63:0]           ds_result;
   wire [4:0]            ds_flags;
   wire [TAGW-1:0]       ds_tag;
   wire                  ds_take_out = ds_out_valid & ~fpn_out_valid;   // fpnew's result goes first
   logic [63:0]          fpn_result;
   fpnew_pkg::status_t   fpn_status;
   logic [TAGW-1:0]      fpn_tag;

   // ONE OP IN FLIGHT, which is the contract both callers already rely on (smolrv64_core asserts
   // it, exec_shard gates on busy). The result is REGISTERED rather than passed straight
   // out: res_valid feeds m_done in smolrv64_core, and fpnew's combinational output there would
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
   logic                 out_valid_q, out_fp_q, out_int_q;
   logic [63:0]          result_q;
   logic [4:0]           fflags_q;
   logic [TAGW-1:0]      tag_q;
   wire  [TAGW-1:0]      r_tag = fpn_out_valid ? fpn_tag : ds_tag;   // the arriving result's
   initial begin nflight_q = '0; out_valid_q = 1'b0; out_fp_q = 1'b0; out_int_q = 1'b0; req_v_q = 1'b0; end

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
   wire   any_valid  = req_v_q | fire;
   wire   fpn_valid  = any_valid & ~sel_ds;
   wire   ds_valid   = any_valid & sel_ds;
   wire   fpn_take   = (fpn_valid & fpn_in_ready) | (ds_valid & ds_in_ready);   // either unit took it
   wire   res_in     = fpn_out_valid | ds_take_out;                            // a result arrives
   assign res_valid  = out_valid_q;
   assign res_fp     = out_fp_q;
   assign res_int    = out_int_q;
   assign res_data   = result_q;
   assign res_fflags = fflags_q;
   assign res_tag    = tag_q;
   assign busy       = req_v_q | (nflight_q != '0) | out_valid_q;

   // The held request's fields load on every hand-over (none can come while one is held), so
   // their enable is `fire` alone: neither the flush nor fpnew's ready is in it.
   always_ff @(posedge clk) if (fire) begin
      req_ops_q <= iss_operands; req_op_q  <= iss_op;
      req_mod_q <= iss_op_mod; req_sf_q <= iss_src_fmt; req_df_q <= iss_dst_fmt;
      req_if_q  <= iss_int_fmt; req_rnd_q <= iss_rnd;  req_tag_q <= iss_tag;
   end
   always_ff @(posedge clk) begin
      if (reset | flush) begin
         req_v_q <= 1'b0; nflight_q <= '0; out_valid_q <= 1'b0; out_fp_q <= 1'b0; out_int_q <= 1'b0;
      end else begin
         if (out_valid_q & res_ready) begin out_valid_q <= 1'b0; out_fp_q <= 1'b0; out_int_q <= 1'b0; end
         if (fpn_take) req_v_q <= 1'b0;
         if (fire & ~fpn_take) req_v_q <= 1'b1;       // the unit declined: hold it here
         // Ops in flight: a counter, not a flag. One op returning while another is accepted nets
         // to zero, and so would a result in the very cycle its op is accepted (a group with no
         // pipeline registers); the `else if` keeps both cases out of the count.
         if (res_in & ~fpn_take)      nflight_q <= nflight_q - 1'b1;
         else if (fpn_take & ~res_in) nflight_q <= nflight_q + 1'b1;
         if (res_in) begin
            out_valid_q <= 1'b1;
            out_int_q   <= r_tag[RDV_BIT] & ~r_tag[FSHARD_BIT];
            out_fp_q    <= ~(r_tag[RDV_BIT] & ~r_tag[FSHARD_BIT]);
            result_q    <= fpn_out_valid ? fpn_result : ds_result;
            fflags_q    <= fpn_out_valid ? {fpn_status.NV, fpn_status.DZ, fpn_status.OF,
                                            fpn_status.UF, fpn_status.NX} : ds_flags;
            tag_q       <= r_tag;
         end
      end
   end

   // Invariants (docs/rtl-rules.md A1). Both say the same thing from opposite ends: the
   // one-op-at-a-time contract holds, so no result can arrive unowned or be overwritten.
   // {request held, result held, fpnew ready / busy / valid / early, issue ready, 0,
   //  ops in flight, the held request's op}
   assign dbg = {req_v_q, out_valid_q, fpn_in_ready, fpn_busy, fpn_out_valid, fpn_early, iss_ready,
                 1'b0, 4'(nflight_q), req_op_q};
   wire e_orphan = res_in & (nflight_q == '0) & ~fpn_take;
   wire e_over   = res_in & out_valid_q & ~res_ready;
   always_ff @(posedge clk) err <= ~reset & (e_orphan | e_over);
   always_ff @(posedge clk) if (!reset) begin
      if (e_orphan)
         $fatal(1, "fp_unit: a result (%s) with no op in flight", fpn_out_valid ? "fpnew" : "fp_divsqrt");
      if (e_over)
         $fatal(1, "fp_unit: result arrived while the previous one was unconsumed");
      if (fpn_valid & (sel_op == 4'(fpnew_pkg::DIV) | sel_op == 4'(fpnew_pkg::SQRT)))
         $fatal(1, "fp_unit: a division or square root offered to fpnew");
   end

   fp_divsqrt #(.TAGW(TAGW)) u_ds (
      .clk(clk), .reset(reset), .flush(flush),
      .in_valid(ds_valid), .in_ready(ds_in_ready), .in_sqrt(sel_op == 4'(fpnew_pkg::SQRT)),
      .in_f64(sel_df == 3'(fpnew_pkg::FP64)), .in_rm(sel_rnd), .in_a(ops[0]), .in_b(ops[1]), .in_tag(sel_tag),
      .out_valid(ds_out_valid), .out_ready(~fpn_out_valid), .out_result(ds_result), .out_flags(ds_flags),
      .out_tag(ds_tag), .busy(ds_busy));

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
