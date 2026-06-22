`default_nettype none

// RV64 multiply (combinational; DSP-friendly). The divide/remainder ops are NOT here
// -- they run on the iterative divider.v (combinational divide is a timing bomb).
// Selected by {is_w, f3} like the rest of the M ops:
//   f3 (is_w=0): 000 MUL  001 MULH  010 MULHSU  011 MULHU
//   is_w=1:      000 MULW
// (div f3 100..111 never reach here; exec_shard routes those to the divider.)
module mul
  (input  wire [63:0] rs1,
   input  wire [63:0] rs2,
   input  wire [2:0]  f3,
   input  wire        is_w,
   output reg  [63:0] result);

   wire signed [127:0] p_ss = $signed(rs1) * $signed(rs2);                        // signed × signed
   wire        [127:0] p_uu = rs1 * rs2;                                          // unsigned × unsigned
   wire signed [127:0] p_su = $signed({{64{rs1[63]}}, rs1}) * $signed({64'd0, rs2}); // signed × unsigned
   wire [31:0] mulw = rs1[31:0] * rs2[31:0];

   always @* begin
      if (is_w) result = {{32{mulw[31]}}, mulw};   // MULW
      else case (f3)
         3'b000:  result = p_ss[63:0];     // MUL (low half; sign-agnostic)
         3'b001:  result = p_ss[127:64];   // MULH
         3'b010:  result = p_su[127:64];   // MULHSU
         default: result = p_uu[127:64];   // 011 MULHU
      endcase
   end
endmodule

`default_nettype wire
