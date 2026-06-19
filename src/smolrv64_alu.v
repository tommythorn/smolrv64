`include "smolrv64_defs.vh"

// Combinational integer ALU. Pure function of the pre-decoded operation code
// (op, decoded in S_RF) and its operands. The core registers `result` into
// exe_add at the S_EXECUTE -> S_EXECUTE2 boundary, so this is the first of the
// two pipeline stages that keep the arithmetic critical path to ~7 LUT levels.
//
//   sxt = 1  -> W-type: operate on the low 32 bits and let the core
//               sign-extend bit 31 of the result (via exe_sext32).
module smolrv64_alu
  (input  wire [ 3:0] op,    // EXOP_* operation code
   input  wire [63:0] a,     // rs1 value
   input  wire [63:0] b,     // second operand (rs2 / immediate)
   input  wire        sxt,   // 1 = W-type 32-bit operation
   output reg  [63:0] result);

   always @* begin
      case (op)
        `EXOP_ADD: result = sxt
                       ? {32'd0, a[31:0] + b[31:0]}
                       : a + b;
        `EXOP_SUB: result = sxt
                       ? {32'd0, a[31:0] - b[31:0]}
                       : a - b;
        `EXOP_SHL: result = sxt
                       ? {32'd0, a[31:0] << b[4:0]}
                       : a << b[5:0];
        `EXOP_SHR: result = sxt
                       ? {32'd0, a[31:0] >> b[4:0]}
                       : a >> b[5:0];
        // EXOP_SAR: use if/else to avoid ternary mixing signed/unsigned arms
        // (Verilog coerces $signed(a)>>>n to unsigned/logical when the other
        //  ternary arm is unsigned, breaking arithmetic right shift)
        `EXOP_SAR: if (sxt)
                      result = {32'd0, $signed(a[31:0]) >>> b[4:0]};
                   else
                      result = $signed(a) >>> b[5:0];
        `EXOP_XOR: result = a ^ b;
        `EXOP_OR:  result = a | b;
        `EXOP_AND: result = a & b;
        `EXOP_LTS: result = $signed(a) < $signed(b) ? 1 : 0;
        `EXOP_LTU: result = a < b ? 1 : 0;
        `EXOP_OPB: result = b;
        `EXOP_ONE: result = 1;
        default:   result = 0;
      endcase
   end
endmodule
