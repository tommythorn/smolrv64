`default_nettype none
`include "smolrv64_irec.vh"

// One slot of the frontend's bundle register, decoded into the record the decoupling queue
// holds (smolrv64_irec.vh). A fetch-fault pseudo-op decodes to a serialising op that carries
// the fault and nothing else. The branch compares are payload-static (pc, imm and the
// prediction are known here), so they are computed once, at decode, and ride as two bits.
module smolrv64_dslot
  #(parameter PCW  = 64,
    parameter SEQW = 8,
    parameter PDW  = 31,
    parameter integer DCR_HIT  = 2,
    parameter integer DCR_BACK = 1)
   (input  wire [PDW-1:0]     pdet,       // the bundle's predict details
    input  wire [PCW-1:0]     pc,
    input  wire [31:0]        inst,
    input  wire [SEQW-1:0]    seq_in,
    input  wire [1:0]         pk,         // fetch's next-PC choice: fall through, target, hold
    input  wire [PCW-1:0]     tgt,        // ...the predicted target
    input  wire               fault_op,   // the fetch-fault pseudo-op
    input  wire [3:0]         cause,
    input  wire [PCW-1:0]     tval,       // the faulting VA
    input  wire               fs_off,     // mstatus.FS is Off: an FP op decodes illegal
    output wire [`IR_RECW-1:0] rec);

   wire        s_rvc, s_rd_v, s_rs1_v, s_rs2_v, s_rs3_v, s_legal, s_has_imm;
   wire [31:0] s_exp;
   wire [5:0]  s_rd, s_rs1, s_rs2, s_rs3, s_alu_op;
   wire [63:0] s_imm;
   wire        s_alu_w, s_alu_uw, s_op2_imm, s_res_link;
   wire [1:0]  s_op1_sel, s_mem_size;
   wire        s_is_mem, s_is_store, s_mem_signed, s_is_branch, s_is_jump;
   wire [2:0]  s_br_func, s_csr_func;
   wire        s_is_mul, s_is_csr, s_is_serialize, s_is_amo, s_is_fencei;
   wire [4:0]  s_amo_func;
   wire        s_is_cbo, s_cbo_zero, s_cbo_keep, s_illegal;
   decode_slot #(.SEQW(SEQW)) u_dec
     (.inst(inst), .in_valid(1'b1), .seq_in(seq_in),
      .valid(), .seq(), .is_rvc(s_rvc), .expanded(s_exp),
      .rd(s_rd), .rd_v(s_rd_v), .rs1(s_rs1), .rs1_v(s_rs1_v),
      .rs2(s_rs2), .rs2_v(s_rs2_v), .rs3(s_rs3), .rs3_v(s_rs3_v),
      .imm(s_imm), .has_imm(s_has_imm), .legal(s_legal),
      .alu_op(s_alu_op), .alu_w(s_alu_w), .alu_uw(s_alu_uw), .op1_sel(s_op1_sel),
      .op2_imm(s_op2_imm), .res_link(s_res_link), .is_mem(s_is_mem),
      .is_store(s_is_store), .mem_size(s_mem_size), .mem_signed(s_mem_signed),
      .is_branch(s_is_branch), .br_func(s_br_func), .is_jump(s_is_jump),
      .is_mul(s_is_mul), .is_csr(s_is_csr), .csr_func(s_csr_func),
      .is_serialize(s_is_serialize), .is_amo(s_is_amo), .amo_func(s_amo_func),
      .is_fencei(s_is_fencei), .is_cbo(s_is_cbo), .cbo_zero(s_cbo_zero),
      .cbo_keep(s_cbo_keep), .illegal(s_illegal));

   // JALR is told from JAL here: its target is rs1+imm, not pc+imm.
   wire           s_is_jalr  = (s_exp[6:2] == 5'b11001);
   wire [PCW-1:0] s_taken_pc = pc + s_imm;
   wire [PCW-1:0] s_ft_pc    = pc + (s_rvc ? 64'd2 : 64'd4);
   // fetch's pred_npc, rebuilt from the queued choice and the length decoded here
   wire [PCW-1:0] pnpc       = (pk == 2'd2) ? pc : (pk == 2'd1) ? tgt : s_ft_pc;

   // the record's fields, fault-masked
   wire [PDW-1:0]  m_pdet = pdet;
   wire [PCW-1:0]  m_pc = pc;
   wire [PCW-1:0]  m_fault_tval = tval;     // faulting VA (straddle: pc+2)
   wire            m_is_fp = fault_op ? 1'b0 : ((s_exp[6:2] == 5'b10100) | (s_exp[6:2] == 5'b10000) | (s_exp[6:2] == 5'b10001) | (s_exp[6:2] == 5'b10010) | (s_exp[6:2] == 5'b10011) | (s_exp[6:2] == 5'b00001) | (s_exp[6:2] == 5'b01001));
   wire [SEQW-1:0] m_seq = seq_in;
   wire [PCW-1:0]  m_pred_npc = pnpc;
   wire            m_fault = fault_op;
   wire [3:0]      m_fault_cause = cause;
   wire [31:0]     m_insn = fault_op ? 32'd0 : s_exp;
   wire            m_rvc = fault_op ? 1'b0 : s_rvc;
   wire [5:0]      m_rd = fault_op ? 6'd0 : s_rd;
   wire            m_rd_v = fault_op ? 1'b0 : s_rd_v;
   wire [5:0]      m_rs1 = fault_op ? 6'd0 : s_rs1;
   wire            m_rs1_v = fault_op ? 1'b0 : s_rs1_v;
   wire [5:0]      m_rs2 = fault_op ? 6'd0 : s_rs2;
   wire            m_rs2_v = fault_op ? 1'b0 : s_rs2_v;
   wire [5:0]      m_rs3 = fault_op ? 6'd0 : s_rs3;
   wire            m_rs3_v = fault_op ? 1'b0 : s_rs3_v;
   wire [63:0]     m_imm = fault_op ? 64'd0 : s_imm;
   wire [5:0]      m_alu_op = s_alu_op;
   wire            m_alu_w = fault_op ? 1'b0 : s_alu_w;
   wire            m_alu_uw = fault_op ? 1'b0 : s_alu_uw;
   wire [1:0]      m_op1_sel = fault_op ? 2'd0 : s_op1_sel;
   wire            m_op2_imm = fault_op ? 1'b0 : s_op2_imm;
   wire            m_res_link = fault_op ? 1'b0 : s_res_link;
   wire            m_is_mem = fault_op ? 1'b0 : s_is_mem;
   wire            m_is_store = fault_op ? 1'b0 : s_is_store;
   wire [1:0]      m_mem_size = s_mem_size;
   wire            m_mem_signed = s_mem_signed;
   wire            m_is_branch = fault_op ? 1'b0 : s_is_branch;
   wire [2:0]      m_br_func = s_br_func;
   wire            m_is_jump = fault_op ? 1'b0 : s_is_jump;
   wire            m_is_jalr = fault_op ? 1'b0 : s_is_jalr;
   wire            m_is_mul = fault_op ? 1'b0 : s_is_mul;
   wire            m_is_csr = fault_op ? 1'b0 : s_is_csr;
   wire [2:0]      m_csr_func = s_csr_func;
   wire            m_is_serialize = fault_op ? 1'b1 : s_is_serialize;
   wire            m_is_amo = fault_op ? 1'b0 : s_is_amo;
   wire [4:0]      m_amo_func = s_amo_func;
   wire            m_is_fencei = fault_op ? 1'b0 : s_is_fencei;
   wire            m_is_cbo = fault_op ? 1'b0 : s_is_cbo;
   wire            m_cbo_zero = s_cbo_zero;
   wire            m_cbo_keep = s_cbo_keep;
   wire            m_illegal = fault_op ? 1'b0 : s_illegal | (m_is_fp & fs_off);
   wire            m_mis_taken = (s_taken_pc != pnpc);
   wire            m_mis_nt = (s_ft_pc != pnpc);
   // the dispatch class, once, at decode (the queue head and dispatch both read it)
   wire [15:0]     m_gc;
   smolrv64_gclass #(.DCR_BACK(DCR_BACK)) u_gc
     (.insn(m_insn), .is_mem(m_is_mem), .is_store(m_is_store), .is_amo(m_is_amo), .is_mul(m_is_mul), .is_fp(m_is_fp),
      .is_csr(m_is_csr), .is_serialize(m_is_serialize), .is_fencei(m_is_fencei), .is_cbo(m_is_cbo),
      .is_branch(m_is_branch), .is_jump(m_is_jump), .is_jalr(m_is_jalr), .illegal(m_illegal),
      .fault(m_fault), .mis_taken(m_mis_taken), .imm_neg(m_imm[63]), .btb_hit(m_pdet[DCR_HIT]), .gc(m_gc));
   assign rec = `IR_REC(m_);
endmodule

`default_nettype wire
