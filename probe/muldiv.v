`default_nettype none

// RV64 M-extension datapath (combinational arithmetic core) for the sharded-OoO
// backend. Selected by {is_w, f3} exactly as the instruction encodes it:
//   f3: 000 MUL 001 MULH 010 MULHSU 011 MULHU 100 DIV 101 DIVU 110 REM 111 REMU
//   is_w (OP-32): 000 MULW  100 DIVW  101 DIVUW  110 REMW  111 REMUW
// (br_func in the payload already equals this funct3; alu_w is is_w -- so wiring
// this in needs only an `is_mul` payload flag, no new operand fields.)
//
// Results follow the RISC-V spec exactly (truncate toward zero; REM takes the sign
// of the dividend; div-by-zero and signed overflow have defined results). This is
// pure combinational arithmetic — a latency wrapper (pipelined multiply / iterative
// divide) is a separate integration concern, like exec_alu vs exec_shard.
module muldiv
  (input  wire [63:0] rs1,
   input  wire [63:0] rs2,
   input  wire [2:0]  f3,        // funct3
   input  wire        is_w,      // *W (32-bit) variant
   output reg  [63:0] result);

   // ---- full-width products (high halves are the MULH* results) ----
   wire signed [127:0] p_ss = $signed(rs1) * $signed(rs2);                       // signed × signed
   wire        [127:0] p_uu = rs1 * rs2;                                         // unsigned × unsigned
   wire signed [127:0] p_su = $signed({{64{rs1[63]}}, rs1}) * $signed({64'd0, rs2}); // signed × unsigned

   // ---- 64-bit divide/remainder with RISC-V special cases ----
   wire               d0   = (rs2 == 64'd0);
   wire               ovf  = (rs1 == 64'h8000000000000000) && (rs2 == ~64'd0);
   wire signed [63:0] s1   = $signed(rs1);
   wire signed [63:0] s2   = $signed(rs2);
   // Keep the signed div/rem in their own signed assignments: a mixed-sign ternary
   // is evaluated in UNSIGNED context, which would silently coerce s1/s2 to unsigned.
   wire signed [63:0] q_s  = s1 / s2;
   wire signed [63:0] m_s  = s1 % s2;
   wire [63:0] divs = d0 ? ~64'd0 : ovf ? rs1   : q_s;
   wire [63:0] rems = d0 ? rs1    : ovf ? 64'd0 : m_s;
   wire [63:0] divu = d0 ? ~64'd0 : (rs1 / rs2);
   wire [63:0] remu = d0 ? rs1    : (rs1 % rs2);

   // ---- 32-bit (W) operands ----
   wire [31:0] a = rs1[31:0];
   wire [31:0] b = rs2[31:0];
   wire               d0w  = (b == 32'd0);
   wire               ovfw = (a == 32'h80000000) && (b == 32'hffffffff);
   wire signed [31:0] as   = $signed(a);
   wire signed [31:0] bs   = $signed(b);
   wire [31:0] mulw  = a * b;
   wire signed [31:0] qsw = as / bs;
   wire signed [31:0] msw = as % bs;
   wire [31:0] divsw = d0w ? ~32'd0 : ovfw ? a     : qsw;
   wire [31:0] remsw = d0w ? a      : ovfw ? 32'd0 : msw;
   wire [31:0] divuw = d0w ? ~32'd0 : (a / b);
   wire [31:0] remuw = d0w ? a      : (a % b);

   always @* begin
      if (is_w) case (f3)
         3'b000:  result = {{32{mulw[31]}},  mulw};
         3'b100:  result = {{32{divsw[31]}}, divsw};
         3'b101:  result = {{32{divuw[31]}}, divuw};
         3'b110:  result = {{32{remsw[31]}}, remsw};
         default: result = {{32{remuw[31]}}, remuw};   // 111 REMUW
      endcase
      else case (f3)
         3'b000:  result = p_ss[63:0];     // MUL (low half; sign-agnostic)
         3'b001:  result = p_ss[127:64];   // MULH
         3'b010:  result = p_su[127:64];   // MULHSU
         3'b011:  result = p_uu[127:64];   // MULHU
         3'b100:  result = divs;
         3'b101:  result = divu;
         3'b110:  result = rems;
         default: result = remu;           // 111 REMU
      endcase
   end
endmodule

`default_nettype wire
