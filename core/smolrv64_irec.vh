// One instruction's decoded record: what the frontend's decoupling queue holds and dispatch
// takes, one per slot. IR_REC(P) is the field list, the single source of truth for the pack
// and unpack order; IR_RECW is its width (the including module has PCW, PDW and SEQW).
`ifndef SMOLRV64_IREC_VH
`define SMOLRV64_IREC_VH
`define IR_REC(P) {P``pdet, P``pc, P``fault_tval, P``is_fp, P``seq, P``pred_npc, P``fault, P``fault_cause, P``insn, P``rvc, P``rd, P``rd_v, P``rs1, P``rs1_v, P``rs2, P``rs2_v, P``rs3, P``rs3_v, P``imm, P``alu_op, P``alu_w, P``alu_uw, P``op1_sel, P``op2_imm, P``res_link, P``is_mem, P``is_store, P``mem_size, P``mem_signed, P``is_branch, P``br_func, P``is_jump, P``is_jalr, P``is_mul, P``is_csr, P``csr_func, P``is_serialize, P``is_amo, P``amo_func, P``is_fencei, P``is_cbo, P``cbo_zero, P``cbo_keep, P``illegal, P``mis_taken, P``mis_nt, P``gc}
`define IR_RECW (3*PCW + PDW + SEQW + 189)
// IR_DECL(P) declares the fields under prefix P (the including module has PCW, PDW and SEQW)
`define IR_DECL(P) wire [PDW-1:0] P``pdet; wire [PCW-1:0] P``pc, P``fault_tval, P``pred_npc; \
   wire P``is_fp, P``fault, P``rvc, P``rd_v, P``rs1_v, P``rs2_v, P``rs3_v; wire [SEQW-1:0] P``seq; \
   wire [3:0] P``fault_cause; wire [31:0] P``insn; wire [5:0] P``rd, P``rs1, P``rs2, P``rs3, P``alu_op; \
   wire [63:0] P``imm; wire P``alu_w, P``alu_uw, P``op2_imm, P``res_link, P``is_mem, P``is_store; \
   wire [1:0] P``op1_sel, P``mem_size; wire P``mem_signed, P``is_branch, P``is_jump, P``is_jalr; \
   wire [2:0] P``br_func, P``csr_func; wire P``is_mul, P``is_csr, P``is_serialize, P``is_amo; \
   wire [4:0] P``amo_func; wire P``is_fencei, P``is_cbo, P``cbo_zero, P``cbo_keep, P``illegal; \
   wire P``mis_taken, P``mis_nt; wire [15:0] P``gc;
`endif
