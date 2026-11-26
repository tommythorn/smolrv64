`default_nettype none
`include "smolrv64_irec.vh"
`include "alu_ops.vh"

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
// A cracked instruction's uop is decoded from the instruction, its index (ux) and the decode state
// the predecode gave it (vst, smolrv64_vdec.vh), so the head rebuilds a uop as it rebuilds any
// record; the record carries the state after it. A vset that writes vl is two uops: the VL uop (its
// internal word: the F stage writes min(AVL, VLMAX) into the VL file's entry its rd names) and then
// the rd uop, a plain ALU op that writes rd = vl the same way and retires the instruction. A move
// (smolrv64_vdec.vh) is a uop a register it writes, each its instruction's word with its place in
// the immediate, after the move-in of an x-register scalar into VH0.
module smolrv64_dslot
  #(parameter PCW  = 64,
    parameter SEQW = 8,
    parameter PDW  = 32,
    parameter integer DCR_HIT  = 2,
    parameter integer DCR_BACK = 1,
    parameter integer QHEAD    = 0)   // 1: at the queue head, inst is the queued expanded instruction
   (input  wire [PDW-1:0]     pdet,       // the bundle's predict details
    input  wire [PCW-1:0]     pc,
    input  wire [31:0]        inst,
    input  wire [SEQW-1:0]    seq_in,
    input  wire [1:0]         pk,         // fetch's next-PC choice: fall through, target, hold
    input  wire [PCW-1:0]     tgt,        // ...the predicted target
    input  wire               fault_op,   // the fetch-fault pseudo-op
    input  wire [3:0]         cause,
    input  wire               fault_hi,   // the fault is the straddler's high halfword's (its VA is pc+2)
    input  wire               fs_off,     // mstatus.FS is Off: an FP op decodes illegal
    input  wire               vs_off,     // mstatus.VS is Off: a vector instruction decodes illegal
    input  wire [`SMOLRV64_UX_W-1:0] ux,       // the uop's index in its instruction
    input  wire               ul,         // ...and it is the last
    input  wire [`SMOLRV64_VST_W-1:0] vst,       // the vector decode state before the instruction
    input  wire               q_rvc,      // a queued record (QHEAD): its RVC flag and the four below
    input  wire [PCW-1:0]     q_pnpc,
    input  wire               q_illegal, q_mis_taken, q_mis_nt,
    input  wire [15:0]        q_gc,
    output wire [`QR_RECW-1:0] qrec,
    output wire [`IR_RECW-1:0] rec,
    output wire               imm_wide);  // the immediate is not its low 32 bits sign-extended (smolrv64_frontend asserts)
`include "smolrv64_pc.vh"
`include "smolrv64_vdec.vh"

   wire        d_rvc, s_rd_v, s_rs1_v, s_rs2_v, s_rs3_v, s_legal, s_has_imm;
   wire [31:0] s_exp;
   wire [`AREG_W-1:0] s_rd, s_rs1, s_rs2, s_rs3;
   wire [5:0]  s_alu_op;
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

   // JALR is told from JAL here: its target is rs1+imm, not pc+imm.
   wire           s_is_jalr  = (s_exp[6:2] == 5'b11001);
   wire [PCW-1:0] s_taken_pc = pc + s_imm[PCW-1:0];   // pc-relative: exact at PCW (smolrv64_pc.vh)
   wire           s_rvc      = q_v ? q_rvc : d_rvc;
   wire [PCW-1:0] s_ft_pc    = pc + (s_rvc ? PC2 : PC4);
   // fetch's pred_npc, rebuilt from the queued choice and the length decoded here
   wire [PCW-1:0] pnpc       = (pk == 2'd2) ? pc : (pk == 2'd1) ? tgt : s_ft_pc;

   // the record's fields, fault-masked
   wire [PDW-1:0]  m_pdet = pdet;
   wire [PCW-1:0]  m_pc = pc;
   wire            m_fault_hi = fault_hi;
   // the vector instructions, the vset uops, and vsetvl (a serialising SYSQ op with rs1 and rs2)
   wire            m_is_vec = ~fault_op & is_vecins(s_exp);
   wire            v_set = ~fault_op & is_vset(s_exp) & ~is_vsetvl(s_exp);
   wire            v_vl  = v_set & vset_alloc(s_exp) & (ux == 5'd0);    // the VL uop
   wire            v_rd  = v_set & ul;                                  // the rd uop
   wire            v_svl = ~fault_op & is_vsetvl(s_exp);
   wire [8:0]      v_vt  = vset_vtype(s_exp);
   wire [7:0]      v_max = vlmax_of(v_vt[7:0]);
   wire            v_reg = is_vsetx(s_exp) & (s_exp[19:15] != 5'd0) & ~v_vt[8];   // vl = min(rs1, VLMAX)
   wire [7:0]      v_imm = v_vt[8] ? 8'd0 : is_vseti(s_exp) ? (({3'd0, s_exp[19:15]} < v_max) ? {3'd0, s_exp[19:15]} : v_max)
                         : v_max;
   // a move: its uop 0 moves an x-register scalar in (v_mi), the others are the vector unit's (v_mu),
   // each writing register r of the destination group
   wire            v_mv  = ~fault_op & is_vmv(s_exp);
   wire            v_mi  = v_mv & vmv_x(s_exp) & (ux == {UX_W{1'b0}});
   wire            v_mu  = v_mv & ~v_mi;
   wire [VP_W-1:0] v_pos = vpos_of(s_exp, ux, vst);
   wire [4:0]      v_r   = {2'b00, v_pos[2:0]};
   wire            v_xs  = is_vmvxs(s_exp);                             // vmv.x.s: x rd
   wire            v_vv  = is_vmvv(s_exp) & (s_exp[14:12] == 3'b000);  // vmv.v.v: vs1
   wire            v_vr  = is_vmvr(s_exp);                             // vmv<nr>r.v: vs2
   wire [`AREG_W-1:0] v_vd  = {2'b10, s_exp[11:7] + v_r},  v_vs1 = {2'b10, s_exp[19:15] + v_r},
                      v_vs2 = {2'b10, s_exp[24:20] + v_r};
   wire            m_is_fp = fault_op ? 1'b0 : ((s_exp[6:2] == 5'b10100) | (s_exp[6:2] == 5'b10000) | (s_exp[6:2] == 5'b10001) | (s_exp[6:2] == 5'b10010) | (s_exp[6:2] == 5'b10011) | (s_exp[6:2] == 5'b00001) | (s_exp[6:2] == 5'b01001));
   wire [SEQW-1:0] m_seq = seq_in;
   wire [PCW-1:0]  m_pred_npc = q_v ? q_pnpc : pnpc;
   wire            m_fault = fault_op;
   wire [3:0]      m_fault_cause = cause;
   // an illegal uop traps as its instruction, so it keeps the instruction's word
   wire [31:0]     m_insn = fault_op ? 32'd0 : m_illegal ? s_exp : v_vl ? vl_word(s_exp) : v_mi ? mvin_word(s_exp) : s_exp;
   wire [UX_W-1:0] m_ux = ux;
   wire            m_ul = ul;
   // the state after: an instruction that traps changes none (the trap refetches)
   wire [VST_W-1:0] m_vst = (fault_op | m_illegal) ? vst : vst_after(s_exp, vst);
   wire [VST_W-1:0] m_vsb = vst;            // ...and before it, which the queue holds (the head rebuilds from it)
   wire            m_rvc = fault_op ? 1'b0 : s_rvc;
   wire [`AREG_W-1:0] v_xrd = {2'b00, s_exp[11:7]}, v_xrs1 = {2'b00, s_exp[19:15]};
   // the VL uop's version is the one it allocates: the state after it
   wire [VST_W-1:0] v_aft = vst_after(s_exp, vst);
   wire [`AREG_W-1:0] m_rd = fault_op ? {`AREG_W{1'b0}} : v_vl ? (VL_ID | {3'd0, v_aft[3:0]}) : (v_rd | v_svl) ? v_xrd
                           : v_mi ? VH0 : v_mu ? (v_xs ? v_xrd : v_vd) : s_rd;
   wire            m_rd_v = fault_op ? 1'b0 : v_vl ? 1'b0 : (v_rd | v_svl) ? (s_exp[11:7] != 5'd0)
                           : v_mi ? 1'b1 : v_mu ? (~v_xs | (s_exp[11:7] != 5'd0)) : s_rd_v;
   wire [`AREG_W-1:0] m_rs1 = fault_op ? {`AREG_W{1'b0}} : (v_vl | v_rd) ? (v_reg ? v_xrs1 : {`AREG_W{1'b0}}) : (v_svl | v_mi) ? v_xrs1
                            : v_mu ? (v_vv ? v_vs1 : vmv_x(s_exp) ? VH0 : {`AREG_W{1'b0}}) : s_rs1;
   wire            m_rs1_v = fault_op ? 1'b0 : (v_vl | v_rd) ? v_reg : (v_svl | v_mi) ? 1'b1 : v_mu ? (v_vv | vmv_x(s_exp)) : s_rs1_v;
   wire [`AREG_W-1:0] m_rs2 = (fault_op | v_set | v_mi) ? {`AREG_W{1'b0}} : v_svl ? {2'b00, s_exp[24:20]}
                            : v_mu ? ((v_vr | v_xs) ? v_vs2 : {`AREG_W{1'b0}}) : s_rs2;
   wire            m_rs2_v = (fault_op | v_set | v_mi) ? 1'b0 : v_svl ? 1'b1 : v_mu ? (v_vr | v_xs) : s_rs2_v;
   wire [`AREG_W-1:0] m_rs3 = (fault_op | v_set | v_mv) ? {`AREG_W{1'b0}} : s_rs3;
   wire            m_rs3_v = (fault_op | v_set | v_mv) ? 1'b0 : s_rs3_v;
   wire [63:0]     m_imm = fault_op ? 64'd0 : v_rd ? (v_reg ? {56'd0, v_max} : {56'd0, v_imm})
                         : v_mv ? {{(64-VP_W){1'b0}}, v_pos} : s_imm;
   wire [31:0]     m_imm32 = m_imm[31:0];
   assign imm_wide = m_imm[63:32] != {32{m_imm[31]}};
   // the rd uop: minu rd, rs1, VLMAX, or rd = the constant vl
   wire [5:0]      m_alu_op = v_rd ? (v_reg ? `ALU_MINU : `ALU_ADD) : s_alu_op;
   wire            m_alu_w = (fault_op | v_rd) ? 1'b0 : s_alu_w;
   wire            m_alu_uw = (fault_op | v_rd) ? 1'b0 : s_alu_uw;
   wire [1:0]      m_op1_sel = fault_op ? 2'd0 : v_rd ? (v_reg ? 2'd0 : 2'd2) : s_op1_sel;   // OP1_RS1 / OP1_ZERO
   wire            m_op2_imm = fault_op ? 1'b0 : v_rd ? 1'b1 : s_op2_imm;
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
   wire [2:0]      m_csr_func = v_svl ? 3'd0 : s_csr_func;   // vsetvl is no CSR op: the SYSQ takes rs1, not an immediate
   wire            m_is_serialize = fault_op ? 1'b1 : v_svl ? 1'b1 : s_is_serialize;
   wire            m_is_amo = fault_op ? 1'b0 : s_is_amo;
   wire [4:0]      m_amo_func = s_amo_func;
   wire            m_is_fencei = fault_op ? 1'b0 : s_is_fencei;
   wire            m_is_cbo = fault_op ? 1'b0 : s_is_cbo;
   wire            m_cbo_zero = s_cbo_zero;
   wire            m_cbo_keep = s_cbo_keep;
   wire            m_illegal = q_v ? q_illegal : fault_op ? 1'b0 : (v_set | v_svl) ? vs_off : v_mv ? (vs_off | vst[13])
                             : s_illegal | (m_is_fp & fs_off);
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
   wire [31:0]     m_inst = s_exp;   // the queue holds it expanded (see the header)
   assign qrec = `QR_REC(m_);
   assign rec = `IR_REC(m_);
endmodule

`default_nettype wire
