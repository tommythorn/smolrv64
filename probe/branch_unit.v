`default_nettype none

// Branch/jump resolution for one shard. Combinational, fed by exec_alu's compare
// outputs (eq/lt/ltu over rs1,rs2) and AGU sum (rs1+imm, used as the JALR target).
// The frontend records the next PC it actually fetched after this op's bundle
// (pred_npc: predicted target if it followed a prediction, else fall-through);
// a redirect is needed exactly when reality disagrees:
//
//   taken (per funct3): BEQ eq / BNE !eq / BLT lt / BGE !lt / BLTU ltu / BGEU !ltu
//   taken_tgt:  JALR -> (rs1+imm)&~1 (= agu_addr) ; branch/JAL -> pc + imm
//   actual_npc: taken ? taken_tgt : pc + ilen
//   redirect  = (is_branch | is_jump) & (actual_npc != pred_npc)
//   target    = actual_npc
//
// One comparator folds cond-branch / JAL / JALR / return: a predicted-taken
// branch that resolves not-taken redirects to its fall-through, a wrong BTB/RAS
// target redirects to the real target, and a never-predicting frontend
// (pred_npc = fall-through) degenerates to the old (is_branch & taken) | is_jump.
// taken_o / taken_tgt are exported for predictor training.
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
    input  wire [63:0] pred_npc,     // the frontend's chosen next PC for this bundle
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
   assign redirect  = (is_branch | is_jump) & (actual_npc != pred_npc);
   assign target    = actual_npc;
endmodule

`default_nettype wire
