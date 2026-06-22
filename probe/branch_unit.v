`default_nettype none

// Branch/jump resolution for one shard. Combinational, fed by exec_alu's compare
// outputs (eq/lt/ltu over rs1,rs2) and AGU sum (rs1+imm, used as the JALR target).
// The frontend predicts not-taken (fall-through), so a redirect is needed on a
// taken conditional branch or on any jump (JAL/JALR):
//
//   taken (per funct3): BEQ eq / BNE !eq / BLT lt / BGE !lt / BLTU ltu / BGEU !ltu
//   target: JALR -> (rs1+imm)&~1 (= agu_addr) ; branch/JAL -> pc + imm
//   redirect = (is_branch & taken) | is_jump
module branch_unit
   (input  wire        is_branch,
    input  wire        is_jump,
    input  wire        is_jalr,      // JALR (target from rs1+imm) vs JAL/branch (pc+imm)
    input  wire [2:0]  br_func,      // BRANCH funct3
    input  wire        cmp_eq,
    input  wire        cmp_lt,
    input  wire        cmp_ltu,
    input  wire [63:0] pc,
    input  wire [63:0] imm,
    input  wire [63:0] agu_addr,     // rs1 + imm (from exec_alu)
    output wire        redirect,
    output wire [63:0] target);

   reg taken;
   always @* case (br_func)
      3'b000:  taken = cmp_eq;     // BEQ
      3'b001:  taken = ~cmp_eq;    // BNE
      3'b100:  taken = cmp_lt;     // BLT
      3'b101:  taken = ~cmp_lt;    // BGE
      3'b110:  taken = cmp_ltu;    // BLTU
      default: taken = ~cmp_ltu;   // BGEU (3'b111)
   endcase

   assign redirect = (is_branch & taken) | is_jump;
   assign target   = is_jalr ? (agu_addr & ~64'd1) : (pc + imm);
endmodule

`default_nettype wire
