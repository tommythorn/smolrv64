// One instruction's decoded record: what the frontend's decoupling queue holds and dispatch
// takes, one per slot. IR_REC(P) is the field list, the single source of truth for the pack
// and unpack order; IR_RECW is its width (the including module has PCW, PDW and SEQW).
`ifndef SMOLRV64_IREC_VH
`define SMOLRV64_IREC_VH
`include "smolrv64_areg.vh"
`define SMOLRV64_UX_W  5               // a uop's index in its instruction (smolrv64_vdec.vh)
`define SMOLRV64_VST_W 15              // the vector decode state (smolrv64_vdec.vh)
`define IR_REC(P) {P``pdet, P``pc, P``fault_hi, P``is_fp, P``seq, P``pred_npc, P``fault, P``fault_cause, P``insn, P``rvc, P``ux, P``ul, P``vst, P``is_vec, P``rd, P``rd_v, P``rs1, P``rs1_v, P``rs2, P``rs2_v, P``rs3, P``rs3_v, P``imm32, P``alu_op, P``alu_w, P``alu_uw, P``op1_sel, P``op2_imm, P``res_link, P``is_mem, P``is_store, P``mem_size, P``mem_signed, P``is_branch, P``br_func, P``is_jump, P``is_jalr, P``is_mul, P``is_csr, P``csr_func, P``is_serialize, P``is_amo, P``amo_func, P``is_fencei, P``is_cbo, P``cbo_zero, P``cbo_keep, P``illegal, P``mis_taken, P``mis_nt, P``gc}
`define IR_RECW (2*PCW + PDW + SEQW + 4*`AREG_W + `SMOLRV64_UX_W + `SMOLRV64_VST_W + 136)
// IR_DECL(P) declares the fields under prefix P (the including module has PCW, PDW and SEQW). A cracked instruction's uop carries its index (ux), whether it is
// the last (ul: the one that retires the instruction) and the vector decode state after its
// instruction (vst); is_vec marks a vector instruction's uop (it sets mstatus.VS Dirty as it commits). A
// fetch fault's address is its pc, or pc+2 for a straddler (fault_hi), and an immediate is 32 bits
// sign-extended: the record stores those, and fault_tval and imm are rebuilt from them.
`define IR_DECL(P) wire [PDW-1:0] P``pdet; wire [PCW-1:0] P``pc, P``pred_npc; wire P``fault_hi; \
   wire [`SMOLRV64_UX_W-1:0] P``ux; wire [`SMOLRV64_VST_W-1:0] P``vst; wire P``ul, P``is_vec; \
   wire P``is_fp, P``fault, P``rvc, P``rd_v, P``rs1_v, P``rs2_v, P``rs3_v; wire [SEQW-1:0] P``seq; \
   wire [3:0] P``fault_cause; wire [31:0] P``insn; wire [`AREG_W-1:0] P``rd, P``rs1, P``rs2, P``rs3; wire [5:0] P``alu_op; \
   wire [31:0] P``imm32; wire [63:0] P``imm = {{32{P``imm32[31]}}, P``imm32}; \
   wire [PCW-1:0] P``fault_tval = P``pc + {{(PCW-2){1'b0}}, P``fault_hi, 1'b0}; wire P``alu_w, P``alu_uw, P``op2_imm, P``res_link, P``is_mem, P``is_store; \
   wire [1:0] P``op1_sel, P``mem_size; wire P``mem_signed, P``is_branch, P``is_jump, P``is_jalr; \
   wire [2:0] P``br_func, P``csr_func; wire P``is_mul, P``is_csr, P``is_serialize, P``is_amo; \
   wire [4:0] P``amo_func; wire P``is_fencei, P``is_cbo, P``cbo_zero, P``cbo_keep, P``illegal; \
   wire P``mis_taken, P``mis_nt; wire [15:0] P``gc;
// What the decoupling queue holds of an instruction (QR_REC): its expanded bits and RVC flag, and what decode computed
// from the fetch state beside them -- the predicted next PC (from fetch's choice and target), the
// illegal bit (mstatus.FS at decode), the branch compares and the dispatch class, the class last
// (the queue head reads its low 16 bits), and a uop's index, last bit and the vector decode state
// before its instruction (vsb). smolrv64_dslot rebuilds the rest from the bits at the head.
`define QR_REC(P) {P``pdet, P``pc, P``fault_hi, P``seq, P``pred_npc, P``fault, P``fault_cause, P``inst, P``rvc, P``ux, P``ul, P``vsb, P``illegal, P``mis_taken, P``mis_nt, P``gc}
`define QR_RECW (2*PCW + PDW + SEQW + 1 + 1 + 4 + 32 + 1 + `SMOLRV64_UX_W + 1 + `SMOLRV64_VST_W + 1 + 1 + 1 + 16)
`define QR_DECL(P) wire [PDW-1:0] P``pdet; wire [PCW-1:0] P``pc, P``pred_npc; wire P``fault_hi, P``fault, P``illegal; \
   wire P``mis_taken, P``mis_nt, P``rvc, P``ul; wire [`SMOLRV64_UX_W-1:0] P``ux; wire [`SMOLRV64_VST_W-1:0] P``vsb; wire [SEQW-1:0] P``seq; wire [3:0] P``fault_cause; wire [31:0] P``inst; wire [15:0] P``gc;
`endif
