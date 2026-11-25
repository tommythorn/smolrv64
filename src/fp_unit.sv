`default_nettype none

// fp_unit -- the FPU: fp_fma for the FMA group (f[n]madd/f[n]msub, fadd/fsub, fmul) and the
// conversions, fp_divsqrt for division and square root. Sign injection, the compares, FMIN/FMAX,
// the moves and FCLASS are the core's own (smolrv64_fp_ops.vh). An op is taken in the cycle it
// arrives, or held here while fp_divsqrt is busy; results carry their writeback routing in the tag,
// not in issue order, and take the result register in a fixed order: fp_fma's (its pipeline never
// waits), then fp_divsqrt's (it holds its answer meanwhile).
//
// iss_op is the decoder's op code (decode_fp.v): FMADD 0, FNMSUB 1, ADD 2, MUL 3, DIV 4, SQRT 5,
// F2F 10, F2I 11, I2F 12; iss_op_mod negates the addend (FMSUB, FNMADD, FSUB) or makes a conversion's
// integer unsigned. Formats: FP32 0, FP64 1; int_fmt INT32 2, INT64 3.
//
// NFLIGHT: ops taken and not yet returned. More than one only for a caller that routes results by tag.
module fp_unit #(parameter TAGW = 24,
                 parameter RDV_BIT = TAGW - 2,      // the tag's rd_v
                 parameter FSHARD_BIT = 0,          // the tag's shard MSB: set for an f-register
                 parameter NFLIGHT = 1)
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
    output reg              err,            // integrity log (registered): an orphan or overwritten result
    // the result's destination kind, flops beside res_valid: an f-register's (res_fp) or an
    // integer register's (res_int, the tag's rd_v set and its register in an integer shard).
    // The core's landing selects on these directly; a decode of the tag there would sit one
    // level in front of the FE wake bus, which fans out to every scheduler entry.
    output wire             res_fp,
    output wire             res_int,
    output wire [15:0]      dbg);           // wait state for the board's wedge ILA (core_dbg)

   localparam [3:0] FMADD = 4'd0, FNMSUB = 4'd1, ADD = 4'd2, MUL = 4'd3, DIV = 4'd4, SQRT = 4'd5,
                    F2F = 4'd10, F2I = 4'd11, I2F = 4'd12;

   // EVERY OUTPUT OF THIS MODULE IS A REGISTER, iss_ready especially: the issue decision reads it,
   // so a combinational ready would close a loop through the scheduler. An op fp_divsqrt cannot take
   // yet is HELD here (req_*) and iss_ready drops; the common case goes straight through.
   reg             req_v_q;
   reg  [191:0]    req_ops_q;
   reg  [3:0]      req_op_q;
   reg             req_mod_q;
   reg  [2:0]      req_sf_q, req_df_q, req_rnd_q;
   reg  [1:0]      req_if_q;
   reg  [TAGW-1:0] req_tag_q;
   localparam integer FW = (NFLIGHT <= 1) ? 1 : $clog2(NFLIGHT + 1);
   reg  [FW-1:0]   nflight_q;               // ops taken by a unit, not yet returned
   reg             out_valid_q, out_fp_q, out_int_q;
   reg  [63:0]     result_q;
   reg  [4:0]      fflags_q;
   reg  [TAGW-1:0] tag_q;
   initial begin nflight_q = {FW{1'b0}}; out_valid_q = 1'b0; out_fp_q = 1'b0; out_int_q = 1'b0; req_v_q = 1'b0; end

   // the held request, or the one arriving now (exclusive: iss_ready is low while one is held)
   wire [191:0]    sel_ops = req_v_q ? req_ops_q : iss_operands;
   wire [3:0]      sel_op  = req_v_q ? req_op_q  : iss_op;
   wire            sel_mod = req_v_q ? req_mod_q : iss_op_mod;
   wire [2:0]      sel_sf  = req_v_q ? req_sf_q  : iss_src_fmt;
   wire [2:0]      sel_df  = req_v_q ? req_df_q  : iss_dst_fmt;
   wire [1:0]      sel_if  = req_v_q ? req_if_q  : iss_int_fmt;
   wire [2:0]      sel_rnd = req_v_q ? req_rnd_q : iss_rnd;
   wire [TAGW-1:0] sel_tag = req_v_q ? req_tag_q : iss_tag;
   wire [63:0]     op0 = sel_ops[63:0], op1 = sel_ops[127:64], op2 = sel_ops[191:128];

   // At NFLIGHT=1 out_blk is `out_valid_q`; above 1, a result being delivered does not block issue
   // when the consumer takes it in the same cycle.
   wire out_blk = out_valid_q & ((NFLIGHT <= 1) ? 1'b1 : ~res_ready);
   assign iss_ready = ~req_v_q & (nflight_q != NFLIGHT[FW-1:0]) & ~out_blk;
   wire   fire      = iss_valid & iss_ready;
   wire   any_valid = req_v_q | fire;

   wire   sel_ds  = (sel_op == DIV) | (sel_op == SQRT);
   wire   sel_fma = ~sel_ds;
   // fp_fma's variant from the op and its modifier
   wire [3:0] fma_v = (sel_op == FMADD)  ? {3'b000, sel_mod} :
                      (sel_op == FNMSUB) ? {3'b001, sel_mod} :
                      (sel_op == ADD)    ? {3'b010, sel_mod} :
                      (sel_op == MUL)    ? 4'd6 :
                      (sel_op == F2F)    ? 4'd7 :
                      (sel_op == I2F)    ? 4'd8 : 4'd9;
   wire        ds_in_ready, ds_out_valid, ds_busy;
   wire [63:0] ds_result;
   wire [4:0]  ds_flags;
   wire [TAGW-1:0] ds_tag;
   wire        fma_out_valid;
   wire [63:0] fma_result;
   wire [4:0]  fma_flags;
   wire [TAGW-1:0] fma_tag;
   wire   ds_valid   = any_valid & sel_ds;
   wire   fma_valid  = any_valid & sel_fma;                     // fp_fma takes every op
   wire   take       = fma_valid | (ds_valid & ds_in_ready);    // a unit took it
   wire   ds_take_out = ds_out_valid & ~fma_out_valid;          // after fp_fma's
   wire   res_in     = fma_out_valid | ds_take_out;             // a result arrives
   wire [TAGW-1:0] r_tag = fma_out_valid ? fma_tag : ds_tag;
   assign res_valid  = out_valid_q;
   assign res_fp     = out_fp_q;
   assign res_int    = out_int_q;
   assign res_data   = result_q;
   assign res_fflags = fflags_q;
   assign res_tag    = tag_q;
   assign busy       = req_v_q | (nflight_q != {FW{1'b0}}) | out_valid_q;

   // the held request's fields load on every hand-over (none can come while one is held)
   always @(posedge clk) if (fire) begin
      req_ops_q <= iss_operands; req_op_q <= iss_op;
      req_mod_q <= iss_op_mod; req_sf_q <= iss_src_fmt; req_df_q <= iss_dst_fmt;
      req_if_q  <= iss_int_fmt; req_rnd_q <= iss_rnd;  req_tag_q <= iss_tag;
   end
   always @(posedge clk) begin
      if (reset | flush) begin
         req_v_q <= 1'b0; nflight_q <= {FW{1'b0}}; out_valid_q <= 1'b0; out_fp_q <= 1'b0; out_int_q <= 1'b0;
      end else begin
         if (out_valid_q & res_ready) begin out_valid_q <= 1'b0; out_fp_q <= 1'b0; out_int_q <= 1'b0; end
         if (take) req_v_q <= 1'b0;
         if (fire & ~take) req_v_q <= 1'b1;                    // fp_divsqrt is busy: hold it here
         // ops in flight: one returning while another is taken nets to zero
         if (res_in & ~take)      nflight_q <= nflight_q - 1'b1;
         else if (take & ~res_in) nflight_q <= nflight_q + 1'b1;
         if (res_in) begin
            out_valid_q <= 1'b1;
            out_int_q   <= r_tag[RDV_BIT] & ~r_tag[FSHARD_BIT];
            out_fp_q    <= ~(r_tag[RDV_BIT] & ~r_tag[FSHARD_BIT]);
            result_q    <= fma_out_valid ? fma_result : ds_result;
            fflags_q    <= fma_out_valid ? fma_flags : ds_flags;
            tag_q       <= r_tag;
         end
      end
   end

   // Invariants (docs/rtl-rules.md A1): no result arrives unowned or overwrites an unconsumed one.
   // {request held, result held, fp_divsqrt ready / busy / answering, fp_fma answering, issue ready,
   //  0, ops in flight, the held request's op}
   assign dbg = {req_v_q, out_valid_q, ds_in_ready, ds_busy, ds_out_valid, fma_out_valid, iss_ready,
                 1'b0, {{(4-FW){1'b0}}, nflight_q}, req_op_q};
   wire e_orphan = res_in & (nflight_q == {FW{1'b0}}) & ~take;
   wire e_over   = res_in & out_valid_q & ~res_ready;
   always @(posedge clk) err <= ~reset & (e_orphan | e_over);
   always @(posedge clk) if (!reset) begin
      if (e_orphan)
         $fatal(1, "fp_unit: a result (%s) with no op in flight", fma_out_valid ? "fp_fma" : "fp_divsqrt");
      if (e_over)
         $fatal(1, "fp_unit: result arrived while the previous one was unconsumed");
      if (any_valid & ~sel_ds & ~((sel_op <= MUL) | (sel_op == F2F) | (sel_op == F2I) | (sel_op == I2F)))
         $fatal(1, "fp_unit: op %0d is no FPU op (the core does it)", sel_op);
   end

   fp_divsqrt #(.TAGW(TAGW)) u_ds (
      .clk(clk), .reset(reset), .flush(flush),
      .in_valid(ds_valid), .in_ready(ds_in_ready), .in_sqrt(sel_op == SQRT),
      .in_f64(sel_df == 3'd1), .in_rm(sel_rnd), .in_a(op0), .in_b(op1), .in_tag(sel_tag),
      .out_valid(ds_out_valid), .out_ready(~fma_out_valid), .out_result(ds_result), .out_flags(ds_flags),
      .out_tag(ds_tag), .busy(ds_busy));

   // operands in the decoder's convention: ADD is op1 + op2, the others start at op0
   fp_fma #(.TAGW(TAGW)) u_fma (
      .clk(clk), .reset(reset), .flush(flush),
      .in_valid(fma_valid), .in_op(fma_v), .in_sf(sel_sf == 3'd1), .in_df(sel_df == 3'd1),
      .in_il(sel_if[0]), .in_iu(sel_mod), .in_rm(sel_rnd),
      .in_a((sel_op == ADD) ? op1 : op0), .in_b(op1), .in_c(op2), .in_tag(sel_tag),
      .out_valid(fma_out_valid), .out_result(fma_result), .out_flags(fma_flags), .out_tag(fma_tag));
endmodule

`default_nettype wire
