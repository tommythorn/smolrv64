`include "smolrv64_defs.vh"
`include "alu_ops.vh"

// smolrv64 integer ALU: a thin wrapper that maps the core's pre-decoded EXOP
// control onto the RVA22+Zicond ALU (alu.v). The op/a/b/sxt interface is
// unchanged, so the core (S_RF decode, S_EXECUTE instantiation) is untouched.
//
//   sxt = 1 -> W-type. alu.v sign-extends the 32-bit result; the core also
//   sign-extends via exe_sext32 (idempotent), so write_back_value is identical
//   to the old behaviour.
//
// OPB (pass b) and ONE (=1) are smolrv64 result-routing pseudo-ops (LUI/AUIPC/
// JAL-link/MV and SC.W/D-fail); they have no RVA22 equivalent and are selected
// here. The bitmanip/Zicond ops alu.v supports are reachable once the core's
// S_RF decoder emits the wider op codes (future work).
module smolrv64_alu
  (input  wire [ 3:0] op,    // EXOP_* operation code
   input  wire [63:0] a,     // rs1 value
   input  wire [63:0] b,     // second operand (rs2 / immediate)
   input  wire        sxt,   // 1 = W-type 32-bit operation
   output reg  [63:0] result);

   // EXOP_* -> ALU_* (alu.v) operation map.
   reg [5:0] aop;
   always @* case (op)
      `EXOP_ADD: aop = `ALU_ADD;
      `EXOP_SUB: aop = `ALU_SUB;
      `EXOP_SHL: aop = `ALU_SLL;
      `EXOP_SHR: aop = `ALU_SRL;
      `EXOP_SAR: aop = `ALU_SRA;
      `EXOP_XOR: aop = `ALU_XOR;
      `EXOP_OR:  aop = `ALU_OR;
      `EXOP_AND: aop = `ALU_AND;
      `EXOP_LTS: aop = `ALU_SLT;
      `EXOP_LTU: aop = `ALU_SLTU;
      default:   aop = `ALU_ADD;   // OPB/ONE handled below; ADD is a harmless default
   endcase

   wire [63:0] alu_r;
   alu #(.XLEN(64)) core
     (.op(aop), .w(sxt), .uw(1'b0), .op1(a), .op2(b),
      .result(alu_r), .sum(), .eq(), .lt(), .ltu());

   always @* case (op)
      `EXOP_OPB: result = b;
      `EXOP_ONE: result = 64'd1;
      default:   result = alu_r;
   endcase
endmodule
