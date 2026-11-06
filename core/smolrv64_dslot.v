`default_nettype none
`include "smolrv64_irec.vh"

// One slot decoded into an instruction's record (smolrv64_irec.vh). A fetch-fault pseudo-op decodes
// to a serialising op that carries the fault and nothing else. The branch compares are
// payload-static (pc, imm and the prediction are known here), so they are computed once, at
// decode, and ride as two bits.
//
// Twice: before the decoupling queue, a slot of the bundle register decodes into what the queue
// holds (qrec: the expanded instruction, its RVC flag and what needs the fetch state -- fetch's choice
// and target, mstatus.FS); at the queue head (QHEAD), a queued record decodes into the whole record
// dispatch takes (rec), the fetch-state fields taken from the queue. The rest is decoded from the same
// expanded bits by the same logic, so the head's decoders expand nothing.
//
// REG=1 is the decoder before the queue on clk2x (smolrv64_frontend): decode_slot's fields and the
// slot's inputs are registered at clk, so the expansion and decode take one cycle and the branch
// compares, the dispatch class and the record the next.
module smolrv64_dslot
  #(parameter PCW  = 64,
    parameter SEQW = 8,
    parameter PDW  = 32,
    parameter integer DCR_HIT  = 2,
    parameter integer DCR_BACK = 1,
    parameter integer QHEAD    = 0,   // 1: at the queue head, inst is the queued expanded instruction
    parameter integer REG      = 0)   // 1: a register after decode_slot (two cycles of clk)
   (input  wire               clk,        // REG's
    input  wire [PDW-1:0]     pdet,       // the bundle's predict details
    input  wire [PCW-1:0]     pc,
    input  wire [31:0]        inst,
    input  wire [SEQW-1:0]    seq_in,
    input  wire [1:0]         pk,         // fetch's next-PC choice: fall through, target, hold
    input  wire [PCW-1:0]     tgt,        // ...the predicted target
    input  wire               fault_op,   // the fetch-fault pseudo-op
    input  wire [3:0]         cause,
    input  wire               fault_hi,   // the fault is the straddler's high halfword's (its VA is pc+2)
    input  wire               fs_off,     // mstatus.FS is Off: an FP op decodes illegal
    input  wire               q_rvc,      // a queued record (QHEAD): its RVC flag and the four below
    input  wire [PCW-1:0]     q_pnpc,
    input  wire               q_illegal, q_mis_taken, q_mis_nt,
    input  wire [15:0]        q_gc,
    output wire [`QR_RECW-1:0] qrec,
    output wire [`IR_RECW-1:0] rec,
    output wire               imm_wide);  // the immediate is not its low 32 bits sign-extended (smolrv64_frontend asserts)
`include "smolrv64_pc.vh"

   wire        d_rvc, s_rd_v, s_rs1_v, s_rs2_v, s_rs3_v, s_legal, s_has_imm;
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
   wire q_v = (QHEAD != 0);
   decode_slot #(.SEQW(SEQW), .EXPANDED(QHEAD)) u_dec
     (.inst(inst), .in_valid(1'b1), .seq_in(seq_in),
      .valid(), .seq(), .is_rvc(d_rvc), .expanded(s_exp),
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

   // ---- the stage boundary (REG): decode_slot's fields and the inputs the rest reads ----
   localparam integer SAW = 32 + 1 + 1 + 1 + 1 + 1 + 6 + 6 + 6 + 6 + 6 + 64 + 1 + 1 + 1 + 1 + 2 + 2 + 1 + 1 + 1 + 1 + 3 + 1 + 1 + 1 + 3 + 1 + 1 + 5 + 1 + 1 + 1 + 1 + 1 + PDW + PCW + SEQW + 2 + PCW + 1 + 4 + 1 + 1;
   wire [SAW-1:0] sa = {s_exp, d_rvc, s_rd_v, s_rs1_v, s_rs2_v, s_rs3_v, s_rd, s_rs1, s_rs2, s_rs3, s_alu_op, s_imm, s_alu_w, s_alu_uw, s_op2_imm, s_res_link, s_op1_sel, s_mem_size, s_is_mem, s_is_store, s_mem_signed, s_is_branch, s_br_func, s_is_jump, s_is_mul, s_is_csr, s_csr_func, s_is_serialize, s_is_amo, s_amo_func, s_is_fencei, s_is_cbo, s_cbo_zero, s_cbo_keep, s_illegal, pdet, pc, seq_in, pk, tgt, fault_op, cause, fault_hi, fs_off};
   wire [SAW-1:0] sb;
   generate if (REG != 0) begin: rg
      reg [SAW-1:0] q;
      always @(posedge clk) q <= sa;
      assign sb = q;
   end else begin: cb
      assign sb = sa;
   end endgenerate
   wire [31:0] t_exp;
   wire t_rvc_d;
   wire t_rd_v;
   wire t_rs1_v;
   wire t_rs2_v;
   wire t_rs3_v;
   wire [5:0] t_rd;
   wire [5:0] t_rs1;
   wire [5:0] t_rs2;
   wire [5:0] t_rs3;
   wire [5:0] t_alu_op;
   wire [63:0] t_imm;
   wire t_alu_w;
   wire t_alu_uw;
   wire t_op2_imm;
   wire t_res_link;
   wire [1:0] t_op1_sel;
   wire [1:0] t_mem_size;
   wire t_is_mem;
   wire t_is_store;
   wire t_mem_signed;
   wire t_is_branch;
   wire [2:0] t_br_func;
   wire t_is_jump;
   wire t_is_mul;
   wire t_is_csr;
   wire [2:0] t_csr_func;
   wire t_is_serialize;
   wire t_is_amo;
   wire [4:0] t_amo_func;
   wire t_is_fencei;
   wire t_is_cbo;
   wire t_cbo_zero;
   wire t_cbo_keep;
   wire t_illegal;
   wire [PDW-1:0] t_pdet;
   wire [PCW-1:0] t_pc;
   wire [SEQW-1:0] t_seq_in;
   wire [1:0] t_pk;
   wire [PCW-1:0] t_tgt;
   wire t_fault_op;
   wire [3:0] t_cause;
   wire t_fault_hi;
   wire t_fs_off;
   assign {t_exp, t_rvc_d, t_rd_v, t_rs1_v, t_rs2_v, t_rs3_v, t_rd, t_rs1, t_rs2, t_rs3, t_alu_op, t_imm, t_alu_w, t_alu_uw, t_op2_imm, t_res_link, t_op1_sel, t_mem_size, t_is_mem, t_is_store, t_mem_signed, t_is_branch, t_br_func, t_is_jump, t_is_mul, t_is_csr, t_csr_func, t_is_serialize, t_is_amo, t_amo_func, t_is_fencei, t_is_cbo, t_cbo_zero, t_cbo_keep, t_illegal, t_pdet, t_pc, t_seq_in, t_pk, t_tgt, t_fault_op, t_cause, t_fault_hi, t_fs_off} = sb;

   // JALR is told from JAL here: its target is rs1+imm, not pc+imm.
   wire           s_is_jalr  = (t_exp[6:2] == 5'b11001);
   wire [PCW-1:0] s_taken_pc = t_pc + t_imm[PCW-1:0];   // pc-relative: exact at PCW (smolrv64_pc.vh)
   wire           s_rvc      = q_v ? q_rvc : t_rvc_d;
   wire [PCW-1:0] s_ft_pc    = t_pc + (s_rvc ? PC2 : PC4);
   // fetch's pred_npc, rebuilt from the queued choice and the length decoded here
   wire [PCW-1:0] pnpc       = (t_pk == 2'd2) ? t_pc : (t_pk == 2'd1) ? t_tgt : s_ft_pc;

   // the record's fields, fault-masked
   wire [PDW-1:0]  m_pdet = t_pdet;
   wire [PCW-1:0]  m_pc = t_pc;
   wire            m_fault_hi = t_fault_hi;
   wire            m_is_fp = t_fault_op ? 1'b0 : ((t_exp[6:2] == 5'b10100) | (t_exp[6:2] == 5'b10000) | (t_exp[6:2] == 5'b10001) | (t_exp[6:2] == 5'b10010) | (t_exp[6:2] == 5'b10011) | (t_exp[6:2] == 5'b00001) | (t_exp[6:2] == 5'b01001));
   wire [SEQW-1:0] m_seq = t_seq_in;
   wire [PCW-1:0]  m_pred_npc = q_v ? q_pnpc : pnpc;
   wire            m_fault = t_fault_op;
   wire [3:0]      m_fault_cause = t_cause;
   wire [31:0]     m_insn = t_fault_op ? 32'd0 : t_exp;
   wire            m_rvc = t_fault_op ? 1'b0 : s_rvc;
   wire [5:0]      m_rd = t_fault_op ? 6'd0 : t_rd;
   wire            m_rd_v = t_fault_op ? 1'b0 : t_rd_v;
   wire [5:0]      m_rs1 = t_fault_op ? 6'd0 : t_rs1;
   wire            m_rs1_v = t_fault_op ? 1'b0 : t_rs1_v;
   wire [5:0]      m_rs2 = t_fault_op ? 6'd0 : t_rs2;
   wire            m_rs2_v = t_fault_op ? 1'b0 : t_rs2_v;
   wire [5:0]      m_rs3 = t_fault_op ? 6'd0 : t_rs3;
   wire            m_rs3_v = t_fault_op ? 1'b0 : t_rs3_v;
   wire [63:0]     m_imm = t_fault_op ? 64'd0 : t_imm;
   wire [31:0]     m_imm32 = m_imm[31:0];
   assign imm_wide = m_imm[63:32] != {32{m_imm[31]}};
   wire [5:0]      m_alu_op = t_alu_op;
   wire            m_alu_w = t_fault_op ? 1'b0 : t_alu_w;
   wire            m_alu_uw = t_fault_op ? 1'b0 : t_alu_uw;
   wire [1:0]      m_op1_sel = t_fault_op ? 2'd0 : t_op1_sel;
   wire            m_op2_imm = t_fault_op ? 1'b0 : t_op2_imm;
   wire            m_res_link = t_fault_op ? 1'b0 : t_res_link;
   wire            m_is_mem = t_fault_op ? 1'b0 : t_is_mem;
   wire            m_is_store = t_fault_op ? 1'b0 : t_is_store;
   wire [1:0]      m_mem_size = t_mem_size;
   wire            m_mem_signed = t_mem_signed;
   wire            m_is_branch = t_fault_op ? 1'b0 : t_is_branch;
   wire [2:0]      m_br_func = t_br_func;
   wire            m_is_jump = t_fault_op ? 1'b0 : t_is_jump;
   wire            m_is_jalr = t_fault_op ? 1'b0 : s_is_jalr;
   wire            m_is_mul = t_fault_op ? 1'b0 : t_is_mul;
   wire            m_is_csr = t_fault_op ? 1'b0 : t_is_csr;
   wire [2:0]      m_csr_func = t_csr_func;
   wire            m_is_serialize = t_fault_op ? 1'b1 : t_is_serialize;
   wire            m_is_amo = t_fault_op ? 1'b0 : t_is_amo;
   wire [4:0]      m_amo_func = t_amo_func;
   wire            m_is_fencei = t_fault_op ? 1'b0 : t_is_fencei;
   wire            m_is_cbo = t_fault_op ? 1'b0 : t_is_cbo;
   wire            m_cbo_zero = t_cbo_zero;
   wire            m_cbo_keep = t_cbo_keep;
   wire            m_illegal = q_v ? q_illegal : t_fault_op ? 1'b0 : t_illegal | (m_is_fp & t_fs_off);
   wire            m_mis_taken = q_v ? q_mis_taken : (s_taken_pc != pnpc);
   wire            m_mis_nt = q_v ? q_mis_nt : (s_ft_pc != pnpc);
   // the dispatch class, once, at decode (the queue head and dispatch both read it)
   wire [15:0]     d_gc;
   wire [15:0]     m_gc = q_v ? q_gc : d_gc;
   smolrv64_gclass #(.DCR_BACK(DCR_BACK)) u_gc
     (.insn(m_insn), .is_mem(m_is_mem), .is_store(m_is_store), .is_amo(m_is_amo), .is_mul(m_is_mul), .is_fp(m_is_fp),
      .is_csr(m_is_csr), .is_serialize(m_is_serialize), .is_fencei(m_is_fencei), .is_cbo(m_is_cbo),
      .is_branch(m_is_branch), .is_jump(m_is_jump), .is_jalr(m_is_jalr), .illegal(m_illegal),
      .fault(m_fault), .mis_taken(m_mis_taken), .imm_neg(m_imm[63]), .btb_hit(m_pdet[DCR_HIT]), .gc(d_gc));
   wire [31:0]     m_inst = t_exp;   // the queue holds it expanded (see the header)
   assign qrec = `QR_REC(m_);
   assign rec = `IR_REC(m_);
endmodule

`default_nettype wire
