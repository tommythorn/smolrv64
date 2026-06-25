`default_nettype none

// Shared FP execution unit for the sharded-OoO probe: a thin wrapper around
// SmolRV64's CVFPU (fpnew RV64D). One unit, fed at most 1 FP op/cycle from the
// scheduler; result returns out-of-order (variable latency) carrying its writeback
// routing in the FPU tag.
//
// FUNCTIONAL build: fpu_clock = clk (no divider, no CDC) -- the divided clock + CDC
// are an FPGA-timing concern added later; results are identical. The flat operand
// bus {op2,op1,op0} reshapes to CVFPU's [2:0][63:0]. TAGW carries the dest physreg +
// owner + seqno + ckpt so the writeback can route + wake + commit-count.
module fp_unit #(parameter TAGW = 24)
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
    // result (out-of-order; res_valid with the matching tag)
    output wire             res_valid,
    input  wire             res_ready,
    output wire [63:0]      res_data,
    output wire [4:0]       res_fflags,
    output wire [TAGW-1:0]  res_tag,
    output wire             busy);

   logic [2:0][63:0] ops;
   assign ops[0] = iss_operands[63:0];
   assign ops[1] = iss_operands[127:64];
   assign ops[2] = iss_operands[191:128];

   smolrv64_cvfpu #(.TAG_WIDTH(TAGW), .PIPE_REGS(4)) u_cvfpu
     (.clock(clk), .fpu_clock(clk), .reset(reset),
      .in_valid(iss_valid), .in_ready(iss_ready),
      .operands(ops), .rnd_mode(iss_rnd),
      .op(iss_op), .op_mod(iss_op_mod),
      .src_fmt(iss_src_fmt), .dst_fmt(iss_dst_fmt), .int_fmt(iss_int_fmt),
      .tag_in(iss_tag),
      .result(res_data), .fflags(res_fflags), .tag_out(res_tag),
      .out_valid(res_valid), .out_ready(res_ready),
      .flush(1'b0), .busy(busy));
endmodule

`default_nettype wire
