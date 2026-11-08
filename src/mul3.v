`default_nettype none

// 3-cycle RV64 multiply, fully pipelined: a start every cycle, each result 3 cycles later
// (done), a squash flushing every stage (abort). A deferred-completion unit, OFF the
// single-cycle execute path. One 65x65 signed multiply covers every op (low or high half), instead of the
// combinational version's three full 64x64 products -> far fewer DSPs.
//
//   f3: 000 MUL (low)  001 MULH  010 MULHSU  011 MULHU ;  is_w: MULW (low 32, sext)
//   sign-extend rs1 for MULH/MULHSU, rs2 for MULH; zero-extend otherwise.
// Latency 2+PB: start at T (operands sampled) -> done pulse at T+2+PB with the result. PB is the
// product's register stages: 1 (the 65x65 product in one cycle), or 2 (a register inside it, which
// the DSP cascade absorbs -- the lanes' clk2x).
module mul3 #(parameter PB = 1)
  (input  wire        clk,
   input  wire        reset,
   input  wire        start,        // begin a multiply (every cycle if wanted)
   input  wire        abort,        // squash: flush the pipe, no done
   input  wire [63:0] rs1,
   input  wire [63:0] rs2,
   input  wire [2:0]  f3,
   input  wire        is_w,
   output wire        busy,
   output wire        done,         // 1-cycle pulse: result valid
   output wire [63:0] result,
   output wire        pre_done,     // the result a cycle before `done`, for a consumer that
   output wire [63:0] pre_result);  // registers it itself (the lanes' write register)

   wire a_sgn = (f3 == 3'b001) | (f3 == 3'b010);   // MULH, MULHSU
   wire b_sgn = (f3 == 3'b001);                     // MULH
   wire hi    = (f3 == 3'b001) | (f3 == 3'b010) | (f3 == 3'b011);  // high half
   // MULW: the product's low 32 bits depend on the operands' low 32 bits alone (a_sgn = b_sgn = 0)
   wire [64:0] a_ext = {a_sgn & rs1[63], rs1};
   wire [64:0] b_ext = {b_sgn & rs2[63], rs2};

   // pipeline stages: A = operands, (M = the product's inner register,) B = product, C = selected result.
   reg               v1, vm, v2, v3;
   reg signed [64:0] a1, b1;
   reg               w1, hi1, wm, him, w2, hi2;
   reg signed [129:0] pm, p2;
   reg        [63:0] res3;

   initial begin v1 = 0; vm = 0; v2 = 0; v3 = 0; end
   wire [63:0] res_d = w2 ? {{32{p2[31]}}, p2[31:0]} : (hi2 ? p2[127:64] : p2[63:0]);
   always @(posedge clk) begin
      if (reset || abort) begin
         v1 <= 1'b0; vm <= 1'b0; v2 <= 1'b0; v3 <= 1'b0;
      end else begin
         v1 <= start;  a1 <= $signed(a_ext); b1 <= $signed(b_ext); w1 <= is_w; hi1 <= hi;
         if (PB == 2) begin
            vm <= v1;  pm <= a1 * b1;  wm <= w1;  him <= hi1;
            v2 <= vm;  p2 <= pm;       w2 <= wm;  hi2 <= him;
         end else begin
            v2 <= v1;  p2 <= a1 * b1;  w2 <= w1;  hi2 <= hi1;
         end
         v3 <= v2;     res3 <= res_d;
      end
   end

   // busy: a multiply in some stage (the registered stages only, not start)
   assign busy   = v1 | vm | v2 | v3;

   assign done   = v3;
   assign pre_done   = v2;
   assign pre_result = res_d;
   assign result = res3;
endmodule

`default_nettype wire
