`include "smolrv64_defs.vh"
`include "alu_ops.vh"

// smolrv64 integer ALU: thin wrapper around the RVA22+Zicond ALU (alu.v).
// The core's pre-decoded execute_req_alu_op already carries ALU_* op codes, so
// this just forwards them, plus the smolrv64-only result-routing pseudo-ops:
//   OPB (pass b)  -> LUI / AUIPC / JAL-link / MV / LI
//   ONE (=1)      -> SC.W/D fail
//
//   sxt = 1 -> W-type. alu.v sign-extends the 32-bit result; the core also
//   sign-extends via exe_sext32 (idempotent), so write_back_value is identical.
module smolrv64_alu
  (input  wire [ 5:0] op,    // ALU_* op code (plus EXOP_OPB / EXOP_ONE)
   input  wire [63:0] a,     // rs1 value
   input  wire [63:0] b,     // second operand (rs2 / immediate)
   input  wire        sxt,   // 1 = W-type 32-bit operation
   input  wire        uw,    // 1 = zero-extend op1 from 32 bits (Zba .uw / slli.uw)
   output reg  [63:0] result);

   wire [63:0] alu_r;
   alu #(.XLEN(64)) core
     (.op(op), .w(sxt), .uw(uw), .op1(a), .op2(b),
      .result(alu_r), .sum(), .eq(), .lt(), .ltu());

   always @* case (op)
      `EXOP_OPB: result = b;
      `EXOP_ONE: result = 64'd1;
      default:   result = alu_r;
   endcase
endmodule
