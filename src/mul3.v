`default_nettype none

// 3-cycle pipelined RV64 multiply, 1 outstanding (start/busy/done/abort), integrated
// as a deferred-completion unit like the divider so it is OFF the single-cycle execute
// path. One 65x65 signed multiply covers every op (low or high half), instead of the
// combinational version's three full 64x64 products -> far fewer DSPs.
//
//   f3: 000 MUL (low)  001 MULH  010 MULHSU  011 MULHU ;  is_w: MULW (low 32, sext)
//   sign-extend rs1 for MULH/MULHSU, rs2 for MULH; zero-extend otherwise.
// Latency 3: start at T (operands sampled) -> done pulse at T+3 with the result.
module mul3
  (input  wire        clk,
   input  wire        reset,
   input  wire        start,        // begin (ignored while busy)
   input  wire        abort,        // squash: flush the pipe, no done
   input  wire [63:0] rs1,
   input  wire [63:0] rs2,
   input  wire [2:0]  f3,
   input  wire        is_w,
   output wire        busy,
   output wire        done,         // 1-cycle pulse: result valid
   output wire [63:0] result);

   wire a_sgn = (f3 == 3'b001) | (f3 == 3'b010);   // MULH, MULHSU
   wire b_sgn = (f3 == 3'b001);                     // MULH
   wire hi    = (f3 == 3'b001) | (f3 == 3'b010) | (f3 == 3'b011);  // high half
   wire [64:0] a_ext = is_w ? {33'd0, rs1[31:0]} : {a_sgn & rs1[63], rs1};
   wire [64:0] b_ext = is_w ? {33'd0, rs2[31:0]} : {b_sgn & rs2[63], rs2};

   // 3 pipeline stages: A = operands, B = product, C = selected result.
   reg               v1, v2, v3;
   reg signed [64:0] a1, b1;
   reg               w1, hi1, w2, hi2;
   reg signed [129:0] p2;
   reg        [63:0] res3;

   initial begin v1 = 0; v2 = 0; v3 = 0; end
   always @(posedge clk) begin
      if (reset || abort) begin
         v1 <= 1'b0; v2 <= 1'b0; v3 <= 1'b0;
      end else begin
         v1 <= start;  a1 <= $signed(a_ext); b1 <= $signed(b_ext); w1 <= is_w; hi1 <= hi;
         v2 <= v1;     p2 <= a1 * b1;        w2 <= w1; hi2 <= hi1;
         v3 <= v2;     res3 <= w2 ? {{32{p2[31]}}, p2[31:0]} : (hi2 ? p2[127:64] : p2[63:0]);
      end
   end

   // busy = the registered stages only (NOT start: start is gated by ~busy upstream,
   // so including it would form a combinational loop). The start-cycle stall is covered
   // by q_iss_is_mul in the scheduler's busy gate.
   assign busy   = v1 | v2 | v3;

   assign done   = v3;
   assign result = res3;
endmodule

`default_nettype wire
