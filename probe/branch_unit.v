`default_nettype none

// Branch/jump resolution for one shard. Combinational, fed by exec_alu's compare
// outputs (eq/lt/ltu over rs1,rs2) and AGU sum (rs1+imm, used as the JALR target).
// The frontend records the next PC it actually fetched after this op's bundle
// (pred_npc); a redirect is needed exactly when reality disagrees:
//
//   taken (per funct3): BEQ eq / BNE !eq / BLT lt / BGE !lt / BLTU ltu / BGEU !ltu
//   taken_tgt:  JALR -> (rs1+imm)&~1 (= agu_addr) ; branch/JAL -> pc + imm
//   actual_npc: taken ? taken_tgt : pc + ilen
//   redirect  = (is_branch | is_jump) & (actual_npc != pred_npc)
//   target    = actual_npc
//
// TIMING SHAPE: for branches and JAL both candidate next-PCs are payload-static,
// so the two 64-bit inequalities are PRECOMPUTED AT RR and arrive here as flops
// (mis_taken = taken-target != pred_npc, mis_nt = fall-through != pred_npc);
// the redirect bit at EX is then just a mux on `taken`. Only JALR -- whose
// target exists only after the AGU add -- pays an EX-time compare (pred_npc).
// The 64-bit adders below feed only the redirect TARGET value, same depth the
// pre-predictor target path had. A never-predicting frontend (pred_npc =
// fall-through) degenerates to the old (is_branch & taken) | is_jump.
module branch_unit
   (input  wire        is_branch,
    input  wire        is_jump,
    input  wire        is_jalr,      // JALR (target from rs1+imm) vs JAL/branch (pc+imm)
    input  wire        is_rvc,       // compressed -> fall-through = pc+2
    input  wire [2:0]  br_func,      // BRANCH funct3
    input  wire        cmp_eq,
    input  wire        cmp_lt,
    input  wire        cmp_ltu,
    input  wire [63:0] pc,
    input  wire [63:0] imm,
    input  wire [63:0] agu_addr,     // rs1 + imm (from exec_alu)
    input  wire        mis_taken,    // RR-precomputed: (pc+imm)   != pred_npc
    input  wire        mis_nt,       // RR-precomputed: (pc+ilen)  != pred_npc
    input  wire [63:0] pred_npc,     // for the JALR-only EX-time compare
    output wire        redirect,
    output wire [63:0] target,
    output wire        taken_o,      // resolved direction (jumps: 1) -- training/GHR repair
    output wire [63:0] taken_tgt);   // resolved taken-target -- BTB training

   reg taken;
   always @* case (br_func)
      3'b000:  taken = cmp_eq;     // BEQ
      3'b001:  taken = ~cmp_eq;    // BNE
      3'b100:  taken = cmp_lt;     // BLT
      3'b101:  taken = ~cmp_lt;    // BGE
      3'b110:  taken = cmp_ltu;    // BLTU
      default: taken = ~cmp_ltu;   // BGEU (3'b111)
   endcase

   wire        tk         = is_jump | (is_branch & taken);
   wire [63:0] seq_npc    = pc + (is_rvc ? 64'd2 : 64'd4);
   wire [63:0] actual_npc = tk ? taken_tgt : seq_npc;

   assign taken_tgt = is_jalr ? (agu_addr & ~64'd1) : (pc + imm);
   assign taken_o   = tk;
   assign redirect  = (is_branch | is_jump)
                    & (is_jalr ? ((agu_addr & ~64'd1) != pred_npc)
                               : (tk ? mis_taken : mis_nt));
   assign target    = actual_npc;
endmodule

`default_nettype wire
