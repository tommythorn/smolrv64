`include "va_codec.vh"
`default_nettype none

// In-order pipelined RVA22S64 core: F | X | M.
//
//   F : PC -> iMMU -> I$ window -> aligner -> RVC expand -> decode   (ooo2_frontend)
//   X : regfile read + M-bypass -> exec_alu (ALU/AGU/compare) -> branch_unit
//   M : LSU (dMMU + D$) | csr_file | trap | redirect | regfile write
//   (mul/div: the MD stage, the F/CTF port's third drain, since C1 2026-09-17)
//
// M IS THE ONLY COMMIT POINT. Every architectural side effect happens there and
// nowhere else, which is what makes traps precise for free: when an instruction is
// in M nothing older can still fault (older ones have retired) and nothing younger
// has changed anything (X and F hold no state). All the OoO core's recovery
// machinery -- checkpoints, replay-to-solo fault delivery, the illegal/data-fault
// latches, the AMO dispatch gap, rollback priority -- collapses into one mux here.
//
// STALLS. Anything multi-cycle (D$ access or miss, page-table walk)
// freezes the whole pipe: M holds its instruction until `m_done`, X holds
// because it cannot hand off, F holds because `accept` is low. One bypass level
// (M -> X) covers every RAW hazard, because an instruction two ahead has already
// written the regfile.
//
// SERIALIZATION. A CSR/system/fence op lets nothing follow it into X until it has
// left M, so CSR values, privilege, satp and mstatus.FS are never read stale.
`ifndef OOO2_HW
 `define OOO2_HW 8                 // fetch window halfwords: 16 bytes, the shipping build since 2026-09-05 (4 before)
`endif
`ifndef OOO2_IW
 `define OOO2_IW 3                 // pipeline width (instructions/cycle); 3 = shipping. Stage 3 width knob.
`endif
module ooo2_core
  #(parameter PCW  = 64,
    parameter SEQW = 8,
    parameter HW   = `OOO2_HW,
    parameter IW   = `OOO2_IW,
    parameter AW   = 64,
    // ooo2_predictor's predict-detail width: {rsp, ghr, yhit, yctr, yidx, hit, ctr}, 3+11+14+3 bits.
    parameter PDW   = 31,
    parameter [PCW-1:0] RESET_PC = 0,
    parameter [63:0] LBASE    = 64'h7000_0000,   // the local SRAM, for the LSU's alignment rule
    parameter        LRAM_LG2 = 18,
    parameter        PABITS   = 36)              // the architectural physical-address width
   (input  wire                    clk,
    input  wire                    reset,
    // ---- instruction memory: the fetch ring's stream into the I$ (rv_icache) ----
    output wire [PCW-1:0]          imem_addr,       // the fetch PC's PA (diagnostics)
    output wire                    imem_ctx_chg,    // a mapping change: the I$ advances its epoch
    input  wire                    ic_busy,         // fence.i or an I$ invalidation in progress
    output wire                    ic_req,
    output wire [63:0]             ic_va,           // the pair's first byte, 8-byte aligned
    output wire [63:0]             ic_pa,
    output wire [9:0]              ic_tag,
    input  wire                    ic_ack,
    input  wire                    ic_valid,
    input  wire [127:0]            ic_data,
    input  wire [9:0]              ic_rtag,
    // Diagnostic only (FBDIAG_BASE readout).  These are the REGISTERED copies the VA tag
    // already maintains, so exporting them adds a fanout and nothing else.
    output wire [63:0]             imem_satp_q,
    output wire [1:0]              imem_priv_q,
    // Fetch-buffer events (computed in rv_soc_top, where the buffer lives) and the redirect
    // it needs to qualify them.  Same route as hpm_dc_access/hpm_ic_access below.
    output wire                    fe_redirect,
    input  wire                    hpm_fb_hit,
    input  wire                    hpm_fb_rhit,
    // ---- platform interrupt lines + time ----
    input  wire [11:0]             hw_ip,
    input  wire [63:0]             mtime,
    input  wire                    hpm_dc_access, hpm_dc_miss, hpm_ic_access, hpm_ic_miss,
    // ---- data memory port ----
    output wire [AW-1:0]           dmem_raddr,
    output wire                    dmem_ren,
    output wire                    dmem_runcached,
    input  wire [63:0]             dmem_rdata,
    input  wire                    dmem_rvalid,
    output wire                    dmem_rfast,      // a fast (queued, tagged) read (C4a)
    output wire [LQ_IB-1:0]        dmem_rtag,       // its tag: the load-queue index
    input  wire                    dmem_rvalid_c,   // a fast-tagged response, one cycle
    input  wire [LQ_IB-1:0]        dmem_rtag_resp,
    input  wire [63:0]             dmem_rdata_c,
    input  wire                    dmem_rbusy,      // the last read awaits the cache's accept
    output wire                    dmem_wen,
    output wire [AW-1:0]           dmem_waddr,
    output wire [AW-1:0]           dmem_wabase,  // access base PA (device decode)
    output wire [63:0]             dmem_wdata,
    output wire [7:0]              dmem_wmask,
    output wire                    dmem_wuncached,
    output wire                    dmem_cbo,
    output wire                    dmem_cbo_zero,
    output wire                    dmem_cbo_keep,
    input  wire                    dmem_wready,
    input  wire                    dmem_waccept,
    output wire                    dmem_idle,          // no memory op in flight (fence.i drain)
    output wire                    ifence,             // FENCE.I this cycle -> flush D$/I$
    // ---- page-table-walker ports (instruction side, data side) ----
    output wire [55:0]             ptw_addr,
    output wire                    ptw_read,
    input  wire [63:0]             ptw_rdata,
    input  wire                    ptw_rvalid,
    output wire [55:0]             dptw_addr,
    output wire                    dptw_read,
    input  wire [63:0]             dptw_rdata,
    input  wire                    dptw_rvalid,
    // ---- observation ----
    output wire                    retire,             // an instruction retired this cycle
    output wire [PCW-1:0]          retire_pc,
    output wire [31:0]             retire_insn,
    output wire                    retire2,            // ...and a second one behind it (item 10c)
    output wire                    retire3,            // ...and a third (IW>=3; hard 0 below): a testbench that
                                                       // sums retire+retire2 undercounts a 3-wide core (2026-09-17)
    output wire [PCW-1:0]          retire2_pc,
    output wire [31:0]             retire2_insn,
    output wire                    redirect,
    output wire [PCW-1:0]          redirect_target,
    // The LSU's invariants, on their way to the SoC's integrity log (rv_errlog). Pure
    // pass-through: registered in the LSU, read in rv_soc_top, nothing in between.
    output wire [15:0]             lsu_err,
    output wire [15:0]             fe_err);        // the frontend's invariants (ooo2_frontend)

   // =========================================================== stage F
   wire                     d_valid, d_rvc, d_rd_v, d_rs1_v, d_rs2_v, d_rs3_v;
   wire [PCW-1:0]           d_pc, d_pred_npc, d_fault_tval;
   wire [31:0]              d_insn;
   wire [SEQW-1:0]          d_seq;
   wire [PDW-1:0]           d_pdet;
   wire [5:0]               d_rd, d_rs1, d_rs2, d_rs3;
   wire [63:0]              d_imm;
   wire [5:0]               d_alu_op;
   wire                     d_alu_w, d_alu_uw, d_op2_imm, d_res_link;
   wire [1:0]               d_op1_sel, d_mem_size;
   wire                     d_is_mem, d_is_store, d_mem_signed;
   wire                     d_is_branch, d_is_jump, d_is_jalr;
   wire [2:0]               d_br_func, d_csr_func;
   wire                     d_is_mul, d_is_csr, d_is_serialize, d_is_amo;
   wire [4:0]               d_amo_func;
   wire                     d_is_fp, d_is_fencei, d_is_cbo, d_cbo_zero, d_cbo_keep;
   wire                     d_illegal, d_mis_taken, d_mis_nt, d_fault;
   wire [3:0]               d_fault_cause;
   wire [SEQW-1:0]          fe_cur_seq;

   wire                     accept;
   wire                     m_done, m_advance;   // M completed / M can take a new op
   wire                     irq_inject;
   wire                     fe_dq_valid;   // fetch assembled an instruction (bubble sub-attribution)
   wire [PCW-1:0]           imem_va;
   wire [55:0]              immu_pa;
   wire                     immu_ready, immu_fault;
   wire [3:0]               immu_cause;
   wire [1:0]               immu_lvl;    // iMMU leaf level (0=4K,1=2M,2=1G) -- VHPR I$ page cap (Stage 2)
   // The fetch ring (ooo2_fring, in the frontend) holds bytes read under a real translation of
   // their VA, and a mapping change empties it (imem_ctx_chg is part of its freeze), so the iMMU's
   // verdict is not part of consuming them: the ring's window is served while the iMMU looks at
   // the PC, and a fault is delivered only when the ring has nothing for the PC (imem_fault, in
   // the frontend). A window served on a faulting translation is asserted impossible, except in
   // a mapping change's own cycle, when the ring still holds the old mapping's bytes, the iMMU
   // already answers for the new one, and the ring's freeze withholds the window.
   wire [$clog2(HW+2)-1:0]  imem_avail;
   wire                     imem_ok;
   always @(posedge clk)
      if (!reset && imem_ok && ~imem_ctx_chg && immu_ready && immu_fault)
         $fatal(1, "ooo2_core: the fetch ring serves a window on a faulting translation (va=%h cause=%0d)", imem_va, immu_cause);
   // The count, for the counters only (FE_QUE's "no bytes" attribution below).
   wire [$clog2(HW+2)-1:0]  imem_avail_g = imem_ok ? imem_avail : {$clog2(HW+2){1'b0}};
   // resolve/training port (driven from M, below)
   wire                     res_v, res_cbr, res_call, res_ret, res_taken;
   wire [PCW-1:0]           res_tgt;
   wire                     redirect_is_trap;
   wire [SEQW-1:0]          redirect_seq;
   wire                     fe_red_pulse, fr_set;
   // The predict details' fields the core reads: the RAS-top snapshot is the top field, the
   // history snapshot the next (ooo2_predictor's pd_mk/pd_no).
   localparam integer RASB   = 3;
   localparam integer GHL    = 11;
   localparam integer PD_RSP = PDW - RASB;                   // the RAS snapshot's LSB
   localparam integer PD_GHR = PD_RSP - GHL;                 // the history snapshot's LSB
   reg  [RASB-1:0]          fe_red_rsp_q;                    // the RAS top the redirect restores
   reg  [GHL-1:0]           fe_red_ghr_q;                    // ...and the history
   wire [PCW-1:0]           fe_red_tgt;
   wire [SEQW-1:0]          fe_red_seq;
   // decode-stage direct-CTI redirect (static JAL / backward-branch resteer): assigned
   // after the dispatch steering; detection wires dcr0/1/2 are just after the frontend.
   wire                     dec_red;
   wire [PCW-1:0]           dec_red_tgt;
   wire [SEQW-1:0]          dec_red_seq;

   // ---- FMAX: the predictor's training bundle lands one cycle later --------------
   // res_v is gated by m_done, which depends on lsu_done -- so the D$/dTLB hit path
   // reached the BTB/ycorr arrays combinationally. Updates are hints, so the extra
   // cycle costs no correctness and no bubble; kept in step with redirect_q so u_bp
   // sees resolve and rollback in their original relative order.
   reg                      res_v_q, res_cbr_q, res_call_q, res_ret_q;
   reg                      res_taken_q;
   reg [PDW-1:0]            res_pdet_q;
   reg [PCW-1:0]            res_tgt_q;
   reg [PCW-1:0]            res_pc_q;     // the resolving CTI's own PC and length: u_bp
   reg                      res_rvc_q;    // recomputes its BTB key and its PC-only tags from
                                          // them instead of carrying them.
   initial res_v_q = 1'b0;
   always @(posedge clk) begin
      res_v_q     <= ~reset & res_v;
      res_cbr_q   <= res_cbr;
      res_call_q  <= res_call;
      res_ret_q   <= res_ret;
      res_taken_q <= res_taken;
      res_pdet_q  <= cf_pdet;    // control flow resolves on the CTF pipe now, not M
      res_tgt_q   <= res_tgt;
      res_pc_q    <= cf_pc;
      res_rvc_q   <= cf_rvc;
   end

   // ---- FMAX: the frontend sees the redirect one cycle late ----------------------
   // Cuts the redirect -> iMMU-translate -> predictor-update cone, which was the whole
   // critical path. redirect_q doubles as the shadow flag: the cycle it is high is
   // exactly the cycle in which the frontend is squashing the extra wrong-path bundle
   // it fetched, and in which M must refuse that bundle.
   reg                      redirect_q, redirect_is_trap_q;
   reg [PCW-1:0]            redirect_target_q;
   reg [SEQW-1:0]           redirect_seq_q;
   initial redirect_q = 1'b0;
   always @(posedge clk) begin
      if (reset) redirect_q <= 1'b0;
      else       redirect_q <= redirect;
      redirect_target_q  <= redirect_target;
      redirect_is_trap_q <= redirect_is_trap;
      redirect_seq_q     <= redirect_seq;
   end

   // The FRONTEND is driven by fe_red_* below, NOT by the squash. The two came apart when
   // the mispredict restart moved to execute; see the EARLY FRONTEND RESTART block.
   reg                      fe_red_q;
   reg [PCW-1:0]            fe_red_tgt_q;
   reg [SEQW-1:0]           fe_red_seq_q;
   // dec_red_q shadows the decode-redirect exactly as redirect_q shadows the backend
   // redirect: the frontend flush + fetch resteer land one cycle late (registered fe_red_q),
   // so the cycle AFTER dec_red the frontend still holds the stale wrong-path slots. dec_red
   // sets no fr_v freeze (there is no backend flush pending), so without this the wrong path
   // would dispatch in that one-cycle window. dec_red_q holds dispatch for that cycle.
   reg                      dec_red_q;
   initial begin fe_red_q = 1'b0; dec_red_q = 1'b0; end
   always @(posedge clk) begin
      if (reset) fe_red_q <= 1'b0;
      else       fe_red_q <= fe_red_pulse;
      fe_red_tgt_q <= fe_red_tgt;
      fe_red_seq_q <= fe_red_seq;
      dec_red_q    <= reset ? 1'b0 : dec_red;
   end

   // ---- slot B, the second IR (item 10b) ----
   wire d2_valid;
   wire [PCW-1:0] d2_pc;
   wire [31:0] d2_insn;
   wire d2_rvc;
   wire [SEQW-1:0] d2_seq;
   wire [PDW-1:0] d2_pdet;
   wire [PCW-1:0] d2_pred_npc;
   wire [5:0] d2_rd;
   wire [5:0] d2_rs1;
   wire [5:0] d2_rs2;
   wire [5:0] d2_rs3;
   wire d2_rd_v;
   wire d2_rs1_v;
   wire d2_rs2_v;
   wire d2_rs3_v;
   wire [63:0] d2_imm;
   wire [5:0] d2_alu_op;
   wire d2_alu_w;
   wire d2_alu_uw;
   wire [1:0] d2_op1_sel;
   wire d2_op2_imm;
   wire d2_res_link;
   wire d2_is_mem;
   wire d2_is_store;
   wire [1:0] d2_mem_size;
   wire d2_mem_signed;
   wire d2_is_branch;
   wire [2:0] d2_br_func;
   wire d2_is_jump;
   wire d2_is_jalr;
   wire d2_is_mul;
   wire d2_is_csr;
   wire [2:0] d2_csr_func;
   wire d2_is_serialize;
   wire d2_is_amo;
   wire [4:0] d2_amo_func;
   wire d2_is_fp;
   wire d2_is_fencei;
   wire d2_is_cbo;
   wire d2_cbo_zero;
   wire d2_cbo_keep;
   wire d2_illegal;
   wire d2_mis_taken;
   wire d2_mis_nt;
   wire d2_fault;
   wire [3:0] d2_fault_cause;
   wire [PCW-1:0] d2_fault_tval;
   wire rn_valid_b;
   wire rn_valid_c;   // slot C dispatched (Stage 3); driven by d3_take in the steering below
   // ---- slot C IR (IW>=3): the frontend's third decoded slot. Dead at IW=2 ----
   wire d3_valid;
   wire [PCW-1:0] d3_pc;
   wire [31:0] d3_insn;
   wire d3_rvc;
   wire [SEQW-1:0] d3_seq;
   wire [PDW-1:0] d3_pdet;
   wire [PCW-1:0] d3_pred_npc;
   wire [5:0] d3_rd, d3_rs1, d3_rs2, d3_rs3;
   wire d3_rd_v, d3_rs1_v, d3_rs2_v, d3_rs3_v;
   wire [63:0] d3_imm;
   wire [5:0] d3_alu_op;
   wire d3_alu_w, d3_alu_uw;
   wire [1:0] d3_op1_sel;
   wire d3_op2_imm, d3_res_link;
   wire d3_is_mem, d3_is_store;
   wire [1:0] d3_mem_size;
   wire d3_mem_signed;
   wire d3_is_branch;
   wire [2:0] d3_br_func;
   wire d3_is_jump, d3_is_jalr;
   wire d3_is_mul, d3_is_csr;
   wire [2:0] d3_csr_func;
   wire d3_is_serialize;
   wire d3_is_amo;
   wire [4:0] d3_amo_func;
   wire d3_is_fp, d3_is_fencei;
   wire d3_is_cbo, d3_cbo_zero, d3_cbo_keep;
   wire d3_illegal;
   wire d3_mis_taken, d3_mis_nt;
   wire d3_fault;
   wire [3:0] d3_fault_cause;
   wire [PCW-1:0] d3_fault_tval;
   ooo2_frontend #(.PCW(PCW), .SEQW(SEQW), .HW(HW), .IW(IW), .PDW(PDW), .RASB(RASB),
                  .RESET_PC(RESET_PC)) fe
     (.clk(clk), .reset(reset), .accept(accept), .consume(rn_valid),
      // slot B (item 10b): not filled yet -- two_wide low keeps the one-IR timing exactly
   // slot B (item 10b): dispatched beside A when the rules below allow
      .consume_b(rn_valid_b), .two_wide(1'b1),
      // slot C (IW>=3): consume_c is the third rename valid; three_wide=(IW>=3) is the master
      // enable. At IW=2 three_wide=0, so the frontend never presents slot C.
      .consume_c(rn_valid_c), .three_wide(three_wide),
      .d2_valid(d2_valid), .d2_pc(d2_pc), .d2_insn(d2_insn), .d2_rvc(d2_rvc), .d2_seq(d2_seq), .d2_pdet(d2_pdet), .d2_pred_npc(d2_pred_npc), .d2_rd(d2_rd), .d2_rs1(d2_rs1), .d2_rs2(d2_rs2), .d2_rs3(d2_rs3), .d2_rd_v(d2_rd_v), .d2_rs1_v(d2_rs1_v), .d2_rs2_v(d2_rs2_v), .d2_rs3_v(d2_rs3_v), .d2_imm(d2_imm), .d2_alu_op(d2_alu_op), .d2_alu_w(d2_alu_w), .d2_alu_uw(d2_alu_uw), .d2_op1_sel(d2_op1_sel), .d2_op2_imm(d2_op2_imm), .d2_res_link(d2_res_link), .d2_is_mem(d2_is_mem), .d2_is_store(d2_is_store), .d2_mem_size(d2_mem_size), .d2_mem_signed(d2_mem_signed), .d2_is_branch(d2_is_branch), .d2_br_func(d2_br_func), .d2_is_jump(d2_is_jump), .d2_is_jalr(d2_is_jalr), .d2_is_mul(d2_is_mul), .d2_is_csr(d2_is_csr), .d2_csr_func(d2_csr_func), .d2_is_serialize(d2_is_serialize), .d2_is_amo(d2_is_amo), .d2_amo_func(d2_amo_func), .d2_is_fp(d2_is_fp), .d2_is_fencei(d2_is_fencei), .d2_is_cbo(d2_is_cbo), .d2_cbo_zero(d2_cbo_zero), .d2_cbo_keep(d2_cbo_keep), .d2_illegal(d2_illegal), .d2_mis_taken(d2_mis_taken), .d2_mis_nt(d2_mis_nt), .d2_fault(d2_fault), .d2_fault_cause(d2_fault_cause), .d2_fault_tval(d2_fault_tval),
      .d3_valid(d3_valid), .d3_pc(d3_pc), .d3_insn(d3_insn), .d3_rvc(d3_rvc), .d3_seq(d3_seq), .d3_pdet(d3_pdet), .d3_pred_npc(d3_pred_npc), .d3_rd(d3_rd), .d3_rs1(d3_rs1), .d3_rs2(d3_rs2), .d3_rs3(d3_rs3), .d3_rd_v(d3_rd_v), .d3_rs1_v(d3_rs1_v), .d3_rs2_v(d3_rs2_v), .d3_rs3_v(d3_rs3_v), .d3_imm(d3_imm), .d3_alu_op(d3_alu_op), .d3_alu_w(d3_alu_w), .d3_alu_uw(d3_alu_uw), .d3_op1_sel(d3_op1_sel), .d3_op2_imm(d3_op2_imm), .d3_res_link(d3_res_link), .d3_is_mem(d3_is_mem), .d3_is_store(d3_is_store), .d3_mem_size(d3_mem_size), .d3_mem_signed(d3_mem_signed), .d3_is_branch(d3_is_branch), .d3_br_func(d3_br_func), .d3_is_jump(d3_is_jump), .d3_is_jalr(d3_is_jalr), .d3_is_mul(d3_is_mul), .d3_is_csr(d3_is_csr), .d3_csr_func(d3_csr_func), .d3_is_serialize(d3_is_serialize), .d3_is_amo(d3_is_amo), .d3_amo_func(d3_amo_func), .d3_is_fp(d3_is_fp), .d3_is_fencei(d3_is_fencei), .d3_is_cbo(d3_is_cbo), .d3_cbo_zero(d3_cbo_zero), .d3_cbo_keep(d3_cbo_keep), .d3_illegal(d3_illegal), .d3_mis_taken(d3_mis_taken), .d3_mis_nt(d3_mis_nt), .d3_fault(d3_fault), .d3_fault_cause(d3_fault_cause), .d3_fault_tval(d3_fault_tval),
      .redirect(fe_red_q), .redirect_pc(fe_red_tgt_q), .redirect_seq(fe_red_seq_q), .redirect_rsp(fe_red_rsp_q), .redirect_ghr(fe_red_ghr_q),
      .irq_inject(irq_inject), .irq_taken(irq_taken), .fe_dq_valid(fe_dq_valid),
      .imem_addr(imem_va), .imem_ipc(), .imem_pa(imem_addr), .imem_xlvl(immu_lvl),
      .imem_xlate_ok(immu_ready & ~immu_fault), .imem_freeze(ic_busy | imem_ctx_chg),
      .fe_avail(imem_avail), .fe_ok(imem_ok),
      .ic_req(ic_req), .ic_va(ic_va), .ic_pa(ic_pa), .ic_tag(ic_tag),
      .ic_ack(ic_ack), .ic_valid(ic_valid), .ic_data(ic_data), .ic_rtag(ic_rtag),
      .imem_fault(immu_ready & immu_fault), .imem_cause(immu_cause),
      .res_v(res_v_q), .res_cbr(res_cbr_q), .res_call(res_call_q), .res_ret(res_ret_q),
      .res_taken(res_taken_q), .res_pdet(res_pdet_q), .res_tgt(res_tgt_q),
      .res_pc(res_pc_q), .res_rvc(res_rvc_q),
      .d_valid(d_valid), .d_pc(d_pc), .d_insn(d_insn), .d_rvc(d_rvc), .d_seq(d_seq),
      .d_pdet(d_pdet), .d_pred_npc(d_pred_npc),
      .d_rd(d_rd), .d_rs1(d_rs1), .d_rs2(d_rs2), .d_rs3(d_rs3),
      .d_rd_v(d_rd_v), .d_rs1_v(d_rs1_v), .d_rs2_v(d_rs2_v), .d_rs3_v(d_rs3_v),
      .d_imm(d_imm),
      .d_alu_op(d_alu_op), .d_alu_w(d_alu_w), .d_alu_uw(d_alu_uw), .d_op1_sel(d_op1_sel),
      .d_op2_imm(d_op2_imm), .d_res_link(d_res_link),
      .d_is_mem(d_is_mem), .d_is_store(d_is_store), .d_mem_size(d_mem_size),
      .d_mem_signed(d_mem_signed), .d_is_branch(d_is_branch), .d_br_func(d_br_func),
      .d_is_jump(d_is_jump), .d_is_jalr(d_is_jalr), .d_is_mul(d_is_mul), .d_is_csr(d_is_csr),
      .d_csr_func(d_csr_func), .d_is_serialize(d_is_serialize), .d_is_amo(d_is_amo),
      .d_amo_func(d_amo_func), .d_is_fp(d_is_fp), .d_is_fencei(d_is_fencei),
      .d_is_cbo(d_is_cbo), .d_cbo_zero(d_cbo_zero), .d_cbo_keep(d_cbo_keep),
      .d_illegal(d_illegal), .d_mis_taken(d_mis_taken), .d_mis_nt(d_mis_nt),
      .d_fault(d_fault), .d_fault_cause(d_fault_cause), .d_fault_tval(d_fault_tval),
      .cur_seq(fe_cur_seq), .fe_err(fe_err));

   // ---- decode-stage direct-CTI redirect (static) -------------------------------------
   // Take a control transfer the predictor called fall-through AT DISPATCH, instead of
   // eating a full exec-resolved mispredict: a JAL (unconditionally taken) or a BACKWARD
   // conditional branch (the loop-back bet). d_mis_taken is the frontend's precomputed
   // (taken_target != pred_npc), so it is 1 exactly when the BTB did not already steer to
   // the taken target. d_pdet[DCR_HIT] is the BTB hit bit ({hit,ctr} = the low BIMW bits of
   // the carried details): gate BACKWARD BRANCHES on a MISS so a TRAINED not-taken bimodal
   // is never overridden by the static-taken bet; a JAL fires on any miss/wrong-target.
   // The target (dec_red_tgt below) is d_pc+d_imm -- a registered decode-stage value, off
   // the guarded fetch array cone (docs/rtl-rules.md, ooo2_predictor's timing note).
   localparam DCR_HIT = 2;   // BIMW-1: the BTB-hit bit inside the carried predict details
   localparam DCR_BACK = 1'b1;   // static-taken for a BACKWARD conditional branch on a BTB miss (loop back-edge bet)
   // Suppress the decode-redirect while an interrupt pseudo-op is being presented or is in
   // flight: fe_red would flush it out of the frontend without the irq FSM's redirect_q/fr_v
   // ever seeing it, wedging inject_inflight forever (the rv64mi-p-illegal vectored-interrupt
   // spin). The interrupt must make progress; a JAL that coincides falls back to the exec
   // redirect -- rare, and cheaper than a livelock.
   wire dcr_arm = ~irq_inject & ~inject_inflight;
   wire dcr0 = dcr_arm & ((d_is_jump   & ~d_is_jalr  & d_mis_taken)
             | (DCR_BACK & d_is_branch  & d_imm[63]   & d_mis_taken & ~d_pdet[DCR_HIT]));
   wire dcr1 = dcr_arm & ((d2_is_jump  & ~d2_is_jalr & d2_mis_taken)
             | (DCR_BACK & d2_is_branch & d2_imm[63]  & d2_mis_taken & ~d2_pdet[DCR_HIT]));
   wire dcr2 = dcr_arm & ((d3_is_jump  & ~d3_is_jalr & d3_mis_taken)
             | (DCR_BACK & d3_is_branch & d3_imm[63]  & d3_mis_taken & ~d3_pdet[DCR_HIT]));


   // THE PHYSICAL-ADDRESS CAP. DRAM starts at 0x8000_0000 and is 2^DRAM_LG2 bytes; a PA at or
   // above its top does not exist and takes an access fault in both MMUs (src/mmu.v), so no
   // access to it reaches a cache or the bus. DRAM_LG2 is the instance's configuration: the
   // board's 2 GiB (its DTS memory node, checked by src/lint.sh), or the cosim's modeled DDR,
   // which Simmerv faults beyond in the same way. PABITS bounds every instance.
`ifdef COSIM_MEM_SIZE_LG2
   localparam integer DRAM_LG2 = `COSIM_MEM_SIZE_LG2;
`else
   localparam integer DRAM_LG2 = 31;
`endif
   localparam [63:0] DRAM_TOP = 64'h8000_0000 + (64'd1 << DRAM_LG2);
   initial if (DRAM_TOP > (64'd1 << PABITS))
      $fatal(1, "ooo2_core: DRAM_TOP=%h lies beyond the %0d-bit physical address space", DRAM_TOP, PABITS);

   // instruction-side translation. M-mode fetches are physical (satp forced Bare).
   wire [63:0] mmu_satp;
   wire [1:0]  mmu_priv, mmu_dpriv;
   wire        mmu_sum, mmu_mxr, mmu_flush;
   wire [63:0] satp_fetch = (mmu_priv  == 2'd3) ? 64'd0 : mmu_satp;
   wire [63:0] satp_data  = (mmu_dpriv == 2'd3) ? 64'd0 : mmu_satp;

   mmu #(.AW(56), .DRAM_TOP(DRAM_TOP)) u_immu
     (.clk(clk), .reset(reset),
      .req_valid(1'b1), .req_vaddr(imem_va), .req_access(2'd0),
      .priv(mmu_priv), .sum(mmu_sum), .mxr(mmu_mxr), .satp(satp_fetch), .flush(mmu_flush),
      .ptw_addr(ptw_addr), .ptw_read(ptw_read),
      .ptw_rdata(ptw_rdata), .ptw_rvalid(ptw_rvalid),
      .walking(), .t_ready(immu_ready), .t_paddr(immu_pa), .t_fault(immu_fault),
      .t_cause(immu_cause), .t_lvl(immu_lvl), .t_uncached(), .t_ok(), .t_fault_raw());
   assign imem_addr = {8'd0, immu_pa};

   // ---- a mapping change ------------------------------------------------------------
   // What a VA maps to changes with a satp write, an sfence.vma, or a switch to or from M-mode's
   // bare translation (all folded into satp_fetch). It empties the fetch ring and advances the
   // I$ epoch. A privilege change alone does not: the iMMU checks permissions on every fetch.
   // mstatus.SUM/MXR gate data accesses, not fetch.
   reg  [1:0]  ipriv_q;
   reg  [63:0] isatp_q;
   always @(posedge clk) begin
      ipriv_q <= mmu_priv;
      isatp_q <= satp_fetch;
   end
   assign imem_ctx_chg  = mmu_flush | (isatp_q != satp_fetch);
   assign fe_redirect   = fe_red_pulse;
   assign imem_satp_q   = isatp_q;
   assign imem_priv_q   = ipriv_q;

   // =========================================================== stage X
   wire [63:0] rf_rs1, rf_rs2, rf_rs3;
   wire        rf_we, rf_we2;
   wire [5:0]  rf_wa2;  wire [63:0] rf_wd2;
   wire [5:0]  rf_wa;
   wire [63:0] rf_wd;

   // rv_regfile is now a SIMULATION-ONLY REFERENCE, not the operand source.  It costs
   // nothing in hardware (`ifndef SYNTHESIS`) and keeps the every-cycle cross-check that
   // found four rename bugs and the missing a1 seed.  Delete it only when the cosim has
   // run the switched design as long as the shadow one did -- a checker that has already
   // caught five defects is worth more than the lines it occupies.
`ifndef SYNTHESIS
   rv_regfile u_rf
     (.clk(clk), .rs1(d_rs1), .rs1_val(rf_rs1), .rs2(d_rs2), .rs2_val(rf_rs2),
      .rs3(d_rs3), .rs3_val(rf_rs3), .we(rf_we), .wa(rf_wa), .wd(rf_wd),
      .we2(rf_we2), .wa2(rf_wa2), .wd2(rf_wd2));
`else
   assign rf_rs1 = 64'd0;  assign rf_rs2 = 64'd0;  assign rf_rs3 = 64'd0;
`endif

   // ---- renaming and the sharded PRF, running as a SHADOW ---------------------------
   // Issue and commit are still in order and rv_regfile is still the operand source, so
   // this changes no architectural behaviour.  The point is that rename and ooo2_prf are
   // driven by the real instruction stream and CHECKED against the known-good register
   // file every cycle (see the assertion below), so the 240-test suite and the 13.5e9
   // retirement cosim validate them before anything depends on them.  Switching the
   // operand source and deleting rv_regfile is then a one-line change against a proven
   // structure rather than a big-bang swap of the core's most load-bearing datapath.
   localparam integer RN_IDXB  = 7;
   localparam integer RN_PBITS = RN_IDXB + 3;   // 3 shard bits: room for a 5th shard (the 3rd ALU)
   localparam [2:0]   SH_IE = 3'd0, SH_LD = 3'd1, SH_FE = 3'd2, SH_IE2 = 3'd3, SH_IE3 = 3'd4;

   wire d_ord   = d_is_mem | d_is_amo | d_is_mul | d_is_fp | d_is_csr | d_is_serialize
                | d_is_fencei | d_is_cbo | d_is_branch | d_is_jump | d_is_jalr
                | d_illegal | d_fault | d_is_irqop;
   wire d2_is_irqop = (d2_insn[6:2] == 5'b11100) & (d2_insn[14:12] == 3'b000)
                    & (d2_insn[31:20] == 12'h7F0) & ~d2_illegal & ~d2_fault;
   wire d2_ord  = d2_is_mem | d2_is_amo | d2_is_mul | d2_is_fp | d2_is_csr | d2_is_serialize
                | d2_is_fencei | d2_is_cbo | d2_is_branch | d2_is_jump | d2_is_jalr
                | d2_illegal | d2_fault | d2_is_irqop;
   wire d3_is_irqop = (d3_insn[6:2] == 5'b11100) & (d3_insn[14:12] == 3'b000)
                    & (d3_insn[31:20] == 12'h7F0) & ~d3_illegal & ~d3_fault;
   wire d3_ord  = d3_is_mem | d3_is_amo | d3_is_mul | d3_is_fp | d3_is_csr | d3_is_serialize
                | d3_is_fencei | d3_is_cbo | d3_is_branch | d3_is_jump | d3_is_jalr
                | d3_illegal | d3_fault | d3_is_irqop;

   // Destination shard = where the result will be written.  Loads and AMOs take SH_LD;
   // mul/div take SH_FE with the FPU (C1, 2026-09-17: the MD stage lands them by tag).
   //
   // An FP instruction goes to SH_FE only if the F STAGE EXECUTES IT (d_cls_f: the FPU's
   // arithmetic and conversions, integer destination or not -- the FPU writes integer regs
   // too, which is why N_FE > 64). The in-core FP ops execute in M and take SH_LD (below).
   // THE RULE IS: shard = the UNIT that writes it, and nothing else. An op routed to M
   // takes SH_LD even when its result is an ordinary integer -- a CSR read and a jump's
   // link register are M's results, not the ALU's. Leaving those two in SH_IE gave that
   // shard a second writer, and the only way to keep one write port was to hold the ALU
   // off whenever M was writing it (`unit_busy = m_wb_ie`). That put the whole LSU
   // completion cone into the INTEGER scheduler's ready bits: the post-route critical path
   // was m_addr -> lsu -> m_done -> m_wb_ie -> u_iq_i/e_r[9][1], 24 levels, -0.383 ns.
   // With this line the ALU is SH_IE's only writer, m_wb_ie is identically 0 (asserted
   // below, not assumed), and the integer scheduler has no unit_busy term at all.
   // SH_LD absorbs it free: 128 registers against a 16-entry ROB.
   // SH_FE IS THE FP/CTF PIPE'S SLICE AND NOTHING ELSE'S. An FP instruction goes to SH_FE
   // only when the F stage executes it (d_cls_f); the in-core FP ops (FSGNJ, FMIN/FMAX, the
   // compares, FMV, FCLASS) execute in M and take SH_LD like every other M result. Routing
   // them by d_is_fp gave SH_FE a second writer, and the only way to keep one write port was
   // to hold the CTF link off while M was writing it (cf_link_wb = ... & ~m_wb_fe) -- the same
   // shape as the m_wb_ie mistake above, with the same consequence: M's whole completion
   // cone (the SQ's commit, the MMU's state) in front of the CTF pipe's wakeup broadcast,
   // u_sq/kcc -> m_wb_fe -> cf_link_wb -> we_fe -> e_r, 22 levels. Assigned after the class
   // wires below (d_cls_f is declared there).
   wire [2:0] d_shard, d2_shard, d3_shard;

   wire [RN_PBITS-1:0] rn_prs1, rn_prs2, rn_prs3, rn_prd;
   wire [RN_PBITS-1:0] rn_prs1_b, rn_prs2_b, rn_prs3_b, rn_prd_b;
   wire [RN_PBITS-1:0] rn_sprs1_b, rn_sprs2_b, rn_sprs3_b, rn_mprs1_b, rn_mprs2_b, rn_mprs3_b;
   wire                rn_lv1_b, rn_lv2_b, rn_lv3_b, rn_byp1_b, rn_byp2_b, rn_byp3_b;
   // ---- 3rd rename port outputs (IW>=3): dead at IW=2, consumed by the slot-C invariant ----
   wire [RN_PBITS-1:0] rn_prs1_c, rn_prs2_c, rn_prs3_c, rn_prd_c;
   wire [RN_PBITS-1:0] rn_sprs1_c, rn_sprs2_c, rn_sprs3_c, rn_mprs1_c, rn_mprs2_c, rn_mprs3_c;
   wire                rn_lv1_c, rn_lv2_c, rn_lv3_c, rn_byp1_c, rn_byp2_c, rn_byp3_c;
   wire [RN_PBITS-1:0] rn_sprs1, rn_sprs2, rn_sprs3;   // the two map candidates, and
   wire [RN_PBITS-1:0] rn_mprs1, rn_mprs2, rn_mprs3;   // the late bit that chooses
   wire                rn_lv1, rn_lv2, rn_lv3;
   wire                rn_stall;
   wire [4:0]          rn_shard_low;   // 5 shards (SH_IE3 added, Stage 3)
   // Rename exactly when the instruction is dispatched (d_take: structural room, no fault
   // replay, not the cycle after a redirect). It MAY be renamed in the redirect cycle
   // itself: that instruction is younger than the redirecting op and the rename's flush arm
   // rolls the pointers back over it (rule I11) -- gating on the same-cycle redirect put M's
   // completion in front of every dispatch write (gate V3, 2026-09-05).
   wire rn_valid = d_take;      // dispatch is no longer gated on M being free

   ooo2_rename #(.IDXB(RN_IDXB), .N_FE(128), .IW(IW)) u_rename
     (.clk(clk), .reset(reset),
      .r_valid(rn_valid), .r_rs1(d_rs1), .r_rs2(d_rs2), .r_rs3(d_rs3),
      .r_rd(d_rd), .r_rd_v(d_rd_v), .r_shard(d_shard),
      .r_prs1(rn_prs1), .r_prs2(rn_prs2), .r_prs3(rn_prs3),
      .r_sprs1(rn_sprs1), .r_sprs2(rn_sprs2), .r_sprs3(rn_sprs3),
      .r_mprs1(rn_mprs1), .r_mprs2(rn_mprs2), .r_mprs3(rn_mprs3),
      .r_lv1(rn_lv1), .r_lv2(rn_lv2), .r_lv3(rn_lv3),
      .r_prd(rn_prd),
      // port B (item 10b): slot B, younger than A in the same cycle
      .r_valid_b(rn_valid_b), .r_rs1_b(d2_rs1), .r_rs2_b(d2_rs2), .r_rs3_b(d2_rs3), .r_rd_b(d2_rd), .r_rd_v_b(d2_rd_v),
      .r_shard_b(d2_shard), .r_prs1_b(rn_prs1_b), .r_prs2_b(rn_prs2_b), .r_prs3_b(rn_prs3_b),
      .r_sprs1_b(rn_sprs1_b), .r_sprs2_b(rn_sprs2_b), .r_sprs3_b(rn_sprs3_b),
      .r_mprs1_b(rn_mprs1_b), .r_mprs2_b(rn_mprs2_b), .r_mprs3_b(rn_mprs3_b),
      .r_lv1_b(rn_lv1_b), .r_lv2_b(rn_lv2_b), .r_lv3_b(rn_lv3_b),
      .r_byp1_b(rn_byp1_b), .r_byp2_b(rn_byp2_b), .r_byp3_b(rn_byp3_b), .r_prd_b(rn_prd_b),
      // port C (IW>=3): dead at IW=2 (r_valid_c tied 0); the dispatch-widening step connects
      // it to the third dispatched uop. Outputs go to the slot-C invariant below until then.
      .r_valid_c(rn_valid_c), .r_rs1_c(d3_rs1), .r_rs2_c(d3_rs2), .r_rs3_c(d3_rs3), .r_rd_c(d3_rd), .r_rd_v_c(d3_rd_v),
      .r_shard_c(d3_shard), .r_prs1_c(rn_prs1_c), .r_prs2_c(rn_prs2_c), .r_prs3_c(rn_prs3_c),
      .r_sprs1_c(rn_sprs1_c), .r_sprs2_c(rn_sprs2_c), .r_sprs3_c(rn_sprs3_c),
      .r_mprs1_c(rn_mprs1_c), .r_mprs2_c(rn_mprs2_c), .r_mprs3_c(rn_mprs3_c),
      .r_lv1_c(rn_lv1_c), .r_lv2_c(rn_lv2_c), .r_lv3_c(rn_lv3_c),
      .r_byp1_c(rn_byp1_c), .r_byp2_c(rn_byp2_c), .r_byp3_c(rn_byp3_c), .r_prd_c(rn_prd_c),
      // COMMIT NOW COMES FROM THE ROB HEAD, not from the M stage. One line, against a
      // structure the previous commit proved bit-identical over 9.17e6 commits -- the same
      // way rename itself was switched over once its shadow had earned it.
      .c_valid(rob_c_valid), .c_rd(rob_c_rd), .c_rd_v(rob_c_rd_v), .c_prd(rob_c_prd),
      .c2_valid(rob_c2_valid), .c2_rd(rob_c2_rd), .c2_rd_v(rob_c2_rd_v), .c2_prd(rob_c2_prd),
      .c3_valid(rob_c3_valid), .c3_rd(rob_c3_rd), .c3_rd_v(rob_c3_rd_v), .c3_prd(rob_c3_prd),
      .flush(redirect),
      .stall(rn_stall), .shard_low(rn_shard_low));

   wire [63:0] prf_rs1, prf_rs2, prf_rs3;
   wire [63:0] prf_f1, prf_f2, prf_f3;                 // the independent F/CTF port's reads
   wire [63:0] prf_a1, prf_a2;                         // the ALU port's operands
   wire [63:0] prf_a21, prf_a22;                       // the second ALU port's (10d-ii)
   wire [63:0] prf_a31, prf_a32;                       // the third ALU port's (Stage 3)
   ooo2_prf #(.IDXB(RN_IDXB), .N_FE(128)) u_prf
     (.clk(clk),
      .we_ie(alu_q_v), .we_ld(we_ld), .we_fe(we_fe),       // int-exec: from the writeback register
      .wa_ie(alu_q_prd), .wa_ld(wa_ld), .wa_fe(wa_fe),
      .wd_ie(alu_q_val), .wd_ld(wb_ld), .wd_fe(wb_fe),
      .we_ie2(alu2_q_v), .wa_ie2(alu2_q_prd), .wd_ie2(alu2_q_val),   // the second ALU (10d-ii)
      .we_ie3(1'b0), .wa_ie3({RN_PBITS{1'b0}}), .wd_ie3(64'd0),   // the third ALU is dead since the swizzle: SH_IE3 has no writer (asserted below)
      // Operands are read AT ISSUE, addressed by the entry the scheduler selected --
      // doc 1's "values live in one place". Reading them at dispatch and carrying them into
      // M is the second copy that property exists to avoid.
      // ra3 is tied off: M reads two operands. Its only would-be user, an FMA routed to M
      // under mstatus.FS=Off, traps before it needs a third (op3f is 0 for it).
      .ra1(i_ps1), .ra2(i_ps2), .ra3({RN_PBITS{1'b0}}),
      .rd1(prf_rs1), .rd2(prf_rs2), .rd3(prf_rs3),
      .ra4(a_ps1), .ra5(a_ps2), .rd4(prf_a1), .rd5(prf_a2),
      .ra6(a2_ps1), .ra7(a2_ps2), .rd6(prf_a21), .rd7(prf_a22),
      .ra8({RN_PBITS{1'b0}}), .ra9({RN_PBITS{1'b0}}), .rd8(prf_a31), .rd9(prf_a32),   // ...and no reader
      .ra10(j_ps1), .ra11(j_ps2), .ra12(j_ps3), .rd10(prf_f1), .rd11(prf_f2), .rd12(prf_f3));

   // ---- reorder buffer, running as a SHADOW ------------------------------------------
   // The step from "M is the commit point" to "the ROB head is the commit point" -- the
   // substrate out-of-order issue needs. Brought up exactly the way rename was (rule I3):
   // it is driven by the real instruction stream and its commit decision is CHECKED against
   // the live one every cycle, while still driving nothing. Switching ooo2_rename's commit
   // port over is then one line against a proven structure.
   //
   // It is small because ooo2_rename already did the hard part: SMAP/RMAP/lv and a per-shard
   // free list with separate speculative and committed heads, where rollback is a pointer
   // restore. That already supports N uncommitted instructions; N is only ever 1 today
   // because M blocks. So the ROB holds the commit RECORD and re-orders it, nothing else.
   localparam integer ROB_DEPTH = 32, ROB_IDXB = 5;
   wire [ROB_IDXB-1:0] rob_d_idx, rob_d_idx2, rob_d_idx3;
   wire                rob_ready, rob_ready2, rob_ready3, rob_empty;
   // Whether the M instruction is the OLDEST in flight. Once M stops blocking, a trap or a
   // redirect may only fire when it is: the trapping instruction is YOUNGER than an
   // outstanding load, and `flush` would otherwise kill that older entry and lose its
   // register write. Not consumed yet -- see the note above lsu_started.
   wire [ROB_IDXB-1:0] rob_head_idx;
   wire                m_at_head = (rob_head_idx == m_rob_idx);
   wire                rob_c_valid, rob_c_rd_v, rob_c_noret;
   wire                rob_c2_valid, rob_c2_rd_v, rob_c2_noret;
   wire [5:0]          rob_c2_rd;
   wire [RN_PBITS-1:0] rob_c2_prd;
   // 3rd commit (IW>=3): dead at IW=2 (ROB gates c3_valid on GE3). Fed to rename's c3 port.
   wire                rob_c3_valid, rob_c3_rd_v, rob_c3_noret;
   // retire3 is a port now (see the port list): every IW=3 retire count taken before 2026-09-17
   // summed two of the three commit ports.
   wire [5:0]          rob_c3_rd;
   wire [RN_PBITS-1:0] rob_c3_prd;
   // Mirrors m_is_irqop one stage earlier. That signal is
   //   m_is_sys & funct3==0 & imm==0x7F0, with m_is_sys carrying ~m_ill_eff & ~m_fault,
   // and m_ill_eff reduces to m_illegal here because a SYSTEM op is never m_is_fp -- so
   // this decode is exact, not approximate.
   wire d_is_irqop = (d_insn[6:2] == 5'b11100) & (d_insn[14:12] == 3'b000)
                   & (d_insn[31:20] == 12'h7F0) & ~d_illegal & ~d_fault;
   wire [5:0]          rob_c_rd;
   wire [RN_PBITS-1:0] rob_c_prd;
   reg  [ROB_IDXB-1:0] m_rob_idx;          // rides with the op, names its slot at completion
   initial m_rob_idx = {ROB_IDXB{1'b0}};
   // i_rob, not iq_iss_rob: M is loaded from the ISSUE REGISTER now, a cycle after
   // selection. Naming the current selection here let M's slot drift from the instruction
   // M actually holds, and both completion ports then marked the same entry done.
   always @(posedge clk) if (m_advance) m_rob_idx <= i_rob;

   // Completion. While M blocks this is just "M finished", so the head is always the M
   // instruction; when the blocking is cut, this becomes one input per unit.
   // A load's done bit is set when its DATA lands, not when M released it -- otherwise the
   // ROB would commit it before its register write exists. A FAULTING load never sets done
   // at all: it traps, and the redirect's flush retires the entry.
   // An ALU op completing at issue is the SECOND completion port: it never enters M, so
   // its ROB entry has to be marked done from here. This is why ooo2_rob's port was widened.
   // FP has its OWN completion port now. It used to share this one, which is why a landing
   // FP result had to be held whenever a load landed in the same cycle -- with several FP
   // ops in flight that collision stops being rare, and holding stops being cheap.
   // A plain store leaves M with no result AND no memory effect yet -- the buffer owns both.
   // Its ROB slot is completed by sq_c_take above, in the cycle memory is actually written.
   wire m_st_nb   = m_is_store & ~m_is_amo & ~m_is_cbo;
   wire m_sq_fill = m_valid & m_st_nb & lsu_xo_v;
   wire m_lq_fill = m_valid & m_ld_nb & lsu_xo_v;
   wire rob_w_valid = (m_valid & m_done & ~m_ld_nb & ~fp_arith & ~m_st_nb) | ld_land | sy_done;
   wire [ROB_IDXB-1:0] rob_w_idx = ld_land ? lq_l_rob : sy_done ? sy_rob : m_rob_idx;   // sy_done: M is empty (asserted)

   // ---- per-physreg readiness (SHADOW: read and checked, not yet acted on) -----------
   // docs/Area-Efficient-Scalar-OoO.md 5. The scheduler needs readiness as STATE per
   // register, because dynamic issue makes the number of outstanding results unbounded;
   // today's interlock is the degenerate case of that with one load tag and one FP tag.
   // CONSUMED, not a shadow: pnd_r1/2/3 are d_srdy below, which is every queue's d_r. It was
   // brought up under rule I3 and the comment here still said "nothing consumes pnd_r*"
   // long after it did -- which sent a later reader looking for work that was already done
   // (D9: a stale statement stops the next person from checking). The I3 assertion below
   // still cross-checks it every cycle. NWB=3, one per PRF shard, matching the write ports.
   // READINESS QUERIES BOTH MAPS AND SELECTS AFTERWARDS.
   //
   // It used to query the already-muxed tag: lv[rs] picked smap[rs] or rmap[rs], and THAT
   // 9-bit result addressed the 512-deep pending array. `lv` is a late signal -- it is
   // written every rename and cleared wholesale on a flush -- so it sat in front of a
   // register-file-sized lookup whose output then had to reach every issue-queue entry's
   // ready bit. That was the post-floorplan critical path: u_rename/lv_reg[15] ->
   // u_iq_l/e_r_reg[9][2], 82% route (see docs/rtl-rules.md I2).
   //
   // Readiness is a pure function of `pend`, so pnd(lv ? s : m) == (lv ? pnd(s) : pnd(m)).
   // Looking BOTH candidates up in parallel and letting lv pick the 1-bit RESULT turns a
   // late 9-bit address mux into a late 2:1 on one wire. The map reads do not depend on lv
   // and start immediately. Cost is three more read ports on a 1-bit-wide array.
   //
   // rn_prs* keeps the muxed tag: the queue payload and psmem still need the actual number,
   // but that is a write into flops/LUTRAM, not a lookup feeding readiness.
   wire pnd_s1, pnd_s2, pnd_s3, pnd_m1, pnd_m2, pnd_m3;
   wire pnd_r1 = rn_lv1 ? pnd_s1 : pnd_m1;
   wire pnd_r2 = rn_lv2 ? pnd_s2 : pnd_m2;
   wire pnd_r3 = rn_lv3 ? pnd_s3 : pnd_m3;
   wire pnd_i1, pnd_i2, pnd_i3;
   wire pnd_a1, pnd_a2;                                // the ALU port's operands (10d-i)
   wire pnd_b1, pnd_b2;                                // the second ALU port's (10d-ii)
   // slot B: the same two-candidate query; a source that IS A's destination is not ready
   wire pnd_s1_b, pnd_s2_b, pnd_s3_b, pnd_m1_b, pnd_m2_b, pnd_m3_b;
   wire pnd_r1_b = ~rn_byp1_b & (rn_lv1_b ? pnd_s1_b : pnd_m1_b);
   wire pnd_r2_b = ~rn_byp2_b & (rn_lv2_b ? pnd_s2_b : pnd_m2_b);
   wire pnd_r3_b = ~rn_byp3_b & (rn_lv3_b ? pnd_s3_b : pnd_m3_b);
   ooo2_pending #(.PBITS(RN_PBITS), .NWB(NWB_C)) u_pend
     (.clk(clk), .reset(reset),
      .a_v(rn_valid & d_rd_v), .a_preg(rn_prd),
      .a_v2(rn_valid_b & d2_rd_v), .a_preg2(rn_prd_b),
      .a_v3(rn_valid_c & d3_rd_v), .a_preg3(rn_prd_c),
      .q10(rn_sprs1_b), .q11(rn_sprs2_b), .q12(rn_sprs3_b), .r10(pnd_s1_b), .r11(pnd_s2_b), .r12(pnd_s3_b),
      .q13(rn_mprs1_b), .q14(rn_mprs2_b), .q15(rn_mprs3_b), .r13(pnd_m1_b), .r14(pnd_m2_b), .r15(pnd_m3_b),
      .q20(rn_sprs1_c), .q21(rn_sprs2_c), .q22(rn_sprs3_c), .r20(pnd_s1_c), .r21(pnd_s2_c), .r22(pnd_s3_c),
      .q23(rn_mprs1_c), .q24(rn_mprs2_c), .q25(rn_mprs3_c), .r23(pnd_m1_c), .r24(pnd_m2_c), .r25(pnd_m3_c),
      .q16(a_ps1), .q17(a_ps2), .r16(pnd_a1), .r17(pnd_a2),
      .q18(a2_ps1), .q19(a2_ps2), .r18(pnd_b1), .r19(pnd_b2),
      .w_v({we_ie3, we_ie2, we_fe, we_ld, we_ie}), .w_preg({wa_ie3, wa_ie2, wa_fe, wa_ld, wa_ie}),
      .q1(rn_sprs1), .q2(rn_sprs2), .q3(rn_sprs3),
      .r1(pnd_s1), .r2(pnd_s2), .r3(pnd_s3),
      .q7(rn_mprs1), .q8(rn_mprs2), .q9(rn_mprs3),
      .r7(pnd_m1), .r8(pnd_m2), .r9(pnd_m3),
      .q4(i_ps1), .q5(i_ps2), .q6(i_ps3),
      .r4(pnd_i1), .r5(pnd_i2), .r6(pnd_i3),
      .flush(redirect));

   // THE SHADOW CHECK. At the cycle an instruction is actually consumed out of X, every
   // source it reads must be either ready by the pending bits or supplied by the M->X
   // bypass. If the bits ever claim "not ready" for a source this machine went ahead and
   // read, they are wrong -- and they would be wrong in the direction that silently
   // corrupts a scheduler built on them.
   // Keyed on rn_valid, NOT on `accept`. `accept` is the decoupling-queue POP; in a redirect
   // cycle it is high while d_take is low, so the instruction is discarded rather than
   // consumed and its sources are never read. rn_valid = m_advance & d_take is the
   // cycle the operands actually move into M.
   // The shadow asserted that every source was ready or bypassed at the cycle X handed an
   // instruction to M. Consumption has moved to ISSUE and the scheduler enforces the same
   // property structurally -- an entry is not selectable until every source is ready -- so
   // the check moves with it: nothing may issue with a source still pending and no forward.
      always @(posedge clk) if (!reset & iss_m) begin
      // "pending" is the REGISTER, cleared at the writeback edge -- so an entry the
      // scheduler woke this cycle is legitimately still marked pending while its value
      // arrives on a forward. The property is that a source is available, by either route.
      if (q_rs1_v & ~pnd_i1)
         $fatal(1, "ooo2: executed with rs1 p%0d still pending (rob=%0d pc=%h insn=%h ord=%b)",
                i_ps1, i_rob, q_pc, q_insn, q_ord);
      // ...except a plain store, whose rs2 is deliberately not waited on: ooo2_sq captures
      // it by snooping. m_rs2_rdy records whether the PRF read was valid, and the buffer's
      // own assertion catches an entry that can never be woken.
      if (q_rs2_v & ~pnd_i2 & ~(q_is_store & ~q_is_amo & ~q_is_cbo))
         $fatal(1, "ooo2: executed with rs2 p%0d still pending (rob=%0d)", i_ps2, i_rob);
      if (q_rs3_v & ~pnd_i3)
         $fatal(1, "ooo2: executed with rs3 p%0d still pending (rob=%0d)", i_ps3, i_rob);
   end
   always @(posedge clk) if (!reset & iss_alu) begin
      if (qa_rs1_v & ~pnd_a1)
         $fatal(1, "ooo2: ALU port executed with rs1 p%0d still pending (rob=%0d pc=%h)", a_ps1, a_rob, qa_pc);
      if (qa_rs2_v & ~pnd_a2)
         $fatal(1, "ooo2: ALU port executed with rs2 p%0d still pending (rob=%0d)", a_ps2, a_rob);
   end
   always @(posedge clk) if (!reset & iss_alu2) begin
      if (qb_rs1_v & ~pnd_b1)
         $fatal(1, "ooo2: second ALU port executed with rs1 p%0d still pending (rob=%0d pc=%h)", a2_ps1, a2_rob, qb_pc);
      if (qb_rs2_v & ~pnd_b2)
         $fatal(1, "ooo2: second ALU port executed with rs2 p%0d still pending (rob=%0d)", a2_ps2, a2_rob);
   end

   // ---- scheduler + execute payload (WIRED, NOT YET STEERING) -------------------------
   // Dispatch fills the scheduler and the payload alongside the existing in-order path;
   // issue drains it the next cycle. Nothing is steered by it yet -- the machine still
   // feeds M from X. What this buys is that the pack/unpack of a 35-field payload, which
   // is where silent corruption would live, is checked every cycle against the m_*
   // registers holding the very same instruction (see the assertion below).
   // ---- THREE SCHEDULERS, ONE PER UNIT CLASS -----------------------------------------
   // One scheduler per class is also one per PRF shard, which is the condition doc 7 names
   // for the writeback arbiter to disappear. It also stops the integer entries paying for
   // FMA's third operand: only FP needs NSRC=3.
   //
   // Sizes are a timing knob. The integer scheduler should be grown until it is JUST BARELY
   // the critical path -- as large as the clock allows and no larger. 10 for now.
   //
   // NO AGE ANYWHERE. The one ordering constraint that survives -- memory against memory,
   // until there is disambiguation -- is the LOAD scheduler's head pointer (INORDER), which
   // is a pointer match rather than the N^2 is-oldest matrix it replaces.
   // SIZE 10/12, AND THE MICROBENCHMARK THAT SAID 8/8 WAS WRONG.
   // workloads/aesbench at OOO2_HW=4 gives cycles/byte 4/4 123.20, 6/6 122.68, 8/8 122.18,
   // 10/12 122.70, and 12/12 / 16/16 / 20/20 bit-identical at 122.70 -- an apparent optimum
   // at 8, worth 0.43%, and it bought +95 ps of probe_clk margin. The full GB5 suite then
   // measured 8/8 at **-4.1% geomean** against 10/12, and on the very workload aesbench
   // models:
   //
   //     AES-XTS  950.6 -> 851.1 KB/sec  (-10.5%)  -- and the Crypto score 1 -> 0
   //     Ray Tracing -16.7%   PDF Rendering -14.8%   SQLite -11.8%
   //
   // aesbench is L1-resident and single-phase; the real AES-XTS runs under virtual memory
   // with real misses and a mixed instruction stream, and a smaller window hurts there.
   // The rule this is here to record: a microbenchmark can VALIDATE a change the real
   // workload REJECTS. Size the schedulers on the suite, and treat a microbenchmark win as
   // provisional until a suite run confirms it. The 95 ps has to be found somewhere else.
   localparam integer NI = 10, IBI = 4;    // integer: pure ALU, reorders freely
   localparam integer NL = 12, IBL = 4;    // every M-class op: memory, mul/div, CSR,
                                           // branches, FP -- one in-order stream
   // NF=8, the policy minimum, since 2026-08-28 (47e1d26a); every gated build since has
   // closed 166.67 MHz with it, at +0.001-0.002 ns. Before that NF=5 was the largest FP
   // scheduler that closed. FREQUENCY IS NOT A KNOB -- it is never traded away except for a
   // diagnostic run -- and INTEGER PERFORMANCE IS NEVER TRADED FOR FP (Tommy, 2026-09-05):
   // when slack is needed, NF gives first, back to 5. The history that set 5, kept -- the
   // scheduler was dialled down one entry at a time until it passed:
   //
   //   NF=8  -0.012   "frontend PC increment"    24 levels,  6x CARRY8
   //   NF=7  -0.082   u_csr/mhpmcounter[12]      32 levels, 10x CARRY8
   //   NF=6  -0.210   fpnew i_fpnew_cast_multi internal pipeline
   //   NF=5  +0.038   PASSES
   //
   // The NF=8 family has since been DIAGNOSED and REMOVED: it was not an increment at all
   // but `cti_ok` leaking from the aligner into the BTB read address (`apc`, since removed)
   // through ooo2_predictor's `hit`/`p_ret` (see that module's `predict`, and rule I6). NF is back
   // at 8 -- the standing policy -- to retest with that path gone.
   //
   // FOUR SIZES, FOUR DIFFERENT FAILING FAMILIES, and none of them the scheduler. The
   // design sits within ~100 ps of the limit on several paths at once and placement decides
   // which one bites (rule I2: 81 ps spread over IDENTICAL RTL). So this number is where
   // the search stopped, not a measurement that 5 is faster than 6 -- and the standing
   // policy remains a minimum of 8, recoverable by fixing the families above rather than
   // by a better scheduler.
   localparam integer NF = 8,  IBF = 3;   // 8 since C1: mul/div share it (was 5)    // FP arith, three sources, its OWN unit. 5 since gate V4 (2026-09-05):
                                           // the two-wide core closed at exactly 0.000 ns and did not boot; FP gives first
   localparam integer OFF_I = 0, OFF_L = NI, OFF_F = NI + NL;
   localparam integer RS_IDXB = 4;         // widest per-class entry index (IBI)
   localparam integer NWB_C   = 5;         // writeback ports watched: one per PRF shard (SH_IE3 since Stage 3)
   localparam integer PL_N = NI + NL + NF, PL_IB = 5;
   localparam integer SQ_N = 8, SQ_IB = 3;      // store buffer: entries, index width
   localparam integer SQ_TB = SQ_IB + 1;         // ...and its seqno: the index plus a wrap bit
                                                 // (ooo2_sq's head/tail counters), so that a load
                                                 // dispatched against a FULL queue counts NENT
                                                 // older stores, not zero
   localparam integer LQ_N = 4, LQ_IB = 2;      // load queue:   entries, index width
   localparam [1:0] C_I = 2'd0, C_L = 2'd1, C_F = 2'd2, C_I2 = 2'd3;   // C_I2: slot B's ALU ops, the second integer scheduler (10d-ii)

   // ORDERED: anything that can trap, redirect, touch memory or hold a unit for more than a
   // cycle. Those go to the in-order schedulers; what is left free to reorder is the pure
   // ALU op, which is exactly what queues up behind a consumer waiting on a load.
   // An FP load/store is a MEMORY op, not an FP-unit op -- it must go to the load scheduler
   // or memory ordering is silently broken for half the accesses.
   // FP ARITH NOW HAS ITS OWN SCHEDULER AND ITS OWN UNIT (the F stage below), so it no
   // longer queues behind every load, CSR and branch in the in-order stream (mul/div and
   // the branches left it too: the F/CTF/MD port, below).
   //
   // Three schedulers used to deadlock because all three fed ONE execute stage: an op
   // reached M, found it had to be ROB head to retire, and an older op in a different
   // scheduler could not issue to free M. That is fixed by removal, not by arbitration --
   // an FP arith op never enters M at all now. It cannot head-block, because it cannot
   // trap: the only FP trap is illegal, and both of its causes are settled before dispatch
   // (a bad encoding is ~d_fp_valid, and mstatus.FS=Off is gated below).
   //
   // fs_off: a write to FS REDIRECTS and refetches younger ops (csr_file do_fschg), so the
   // value read at dispatch is the one every in-flight FP op will retire under. When FS is
   // off, FP arith routes to M as before and takes its illegal-instruction trap there --
   // unchanged, and the reason the F stage needs no trap path.
   wire d_fp_valid, d_use_fpu;
   decode_fp u_dfp_disp
     (.insn(d_insn), .fp_valid(d_fp_valid), .use_fpu(d_use_fpu), .fp_class(),
      .op(), .op_mod(), .src_fmt(), .dst_fmt(), .int_fmt(),
      .rnd(), .op0_sel(), .op1_sel(), .op2_sel(), .op0_int(), .wr_fp());

   wire d_cls_f = d_fp_valid & d_use_fpu & ~d_is_mem & ~d_is_amo
                & ~fs_off & ~d_illegal & ~d_fault & ~d_is_irqop;
   // Control flow (jal/jalr/bXX) is its OWN class now: it leaves the ordered/M pipe for the
   // FP/CTF pipe so a branch co-issues with a load (CTF-on-FP). A fetch-faulted CTI or an
   // irqop pseudo-op stays ordered (d_cls_l), so M keeps the single trap site; a real CTI
   // never traps at execute (RVC targets are 2-byte aligned, jalr clears bit 0).
   wire d_cls_c = (d_is_branch | d_is_jump | d_is_jalr) & ~d_illegal & ~d_fault & ~d_is_irqop;
   // A mul/div is the F/CTF port's third drain (C1, 2026-09-17): it leaves the ordered queue,
   // where a 64-cycle divide held every younger load, and lands on SH_FE by tag like the FPU.
   wire d_cls_m = d_is_mul & ~d_illegal & ~d_fault & ~d_is_irqop;
   wire d_cls_s = (d_insn[6:2] == 5'b11100) & ~d_illegal & ~d_fault;   // SYSTEM opcode, the irqop included (C3 step 3)
   wire d_cls_l = d_ord & ~d_cls_f & ~d_cls_c & ~d_cls_m & ~d_cls_s;
   wire d_cls_i = ~d_ord;
   wire d_cls_fc = d_cls_f | d_cls_c | d_cls_m | d_cls_s;   // all four share the FP/CTF/MD/SYS issue queue (u_iq_f)

   // Control flow falls to C_F here so d2_hold treats FP and CTF as one class: they share the
   // FP/CTF queue's single dispatch port, so A and B cannot both go there in a cycle.
   wire [1:0] d_cls = d_cls_i ? C_I : d_cls_l ? C_L : C_F;

   wire [RN_PBITS-1:0] d_prd_g = d_rd_v ? rn_prd : {RN_PBITS{1'b0}};
   // A plain store's rs2 is not an operand of the INSTRUCTION any more -- it is an operand
   // of its store-buffer entry, which watches for it independently (ooo2_sq's snoop). So the
   // scheduler must not wait on it, and this is the whole of that change: one term, no
   // per-entry state, nothing added to select. The store issues on its address alone.
   wire       d_st_nb = d_is_store & ~d_is_amo & ~d_is_cbo;   // "buffered store" -- rule C1
   wire [2:0] d_srdy = {pnd_r3 | ~d_rs3_v, pnd_r2 | ~d_rs2_v | d_st_nb, pnd_r1 | ~d_rs1_v};

   // ---- SLOT B (item 10b, 2026-09-05): a second instruction dispatches beside A when ----
   //   * A dispatches (in order), and neither is anything but a plain op: no serializing,
   //     fence.i, CBO, AMO, CSR, illegal, fault or interrupt pseudo-op on either side, so
   //     every "alone in flight" rule keeps its one site;
   //   * B goes to a DIFFERENT scheduler than A: each scheduler and each payload memory
   //     keeps its single write port;
   //   * at most one load and one store between them: one allocation per queue per cycle;
   //     a load in B behind a store in A captures the tag AFTER that store's, so it sees it
   //     as older (the queue's d_tag is the tail before this cycle's allocation);
   //   * room for two in the ROB, and nothing in flight is being redirected.
   wire d2_fp_valid, d2_use_fpu;
   decode_fp u_dfp_disp2
     (.insn(d2_insn), .fp_valid(d2_fp_valid), .use_fpu(d2_use_fpu), .fp_class(),
      .op(), .op_mod(), .src_fmt(), .dst_fmt(), .int_fmt(),
      .rnd(), .op0_sel(), .op1_sel(), .op2_sel(), .op0_int(), .wr_fp());
   wire d2_cls_f = d2_fp_valid & d2_use_fpu & ~d2_is_mem & ~d2_is_amo
                 & ~fs_off & ~d2_illegal & ~d2_fault & ~d2_is_irqop;
   wire d2_cls_c = (d2_is_branch | d2_is_jump | d2_is_jalr) & ~d2_illegal & ~d2_fault & ~d2_is_irqop;
   wire d2_cls_m = d2_is_mul & ~d2_illegal & ~d2_fault & ~d2_is_irqop;
   wire d2_cls_s = (d2_insn[6:2] == 5'b11100) & ~d2_illegal & ~d2_fault;   // SYSTEM opcode, the irqop included (C3 step 3)
   wire d2_cls_l = d2_ord & ~d2_cls_f & ~d2_cls_c & ~d2_cls_m & ~d2_cls_s;
   wire d2_cls_i = ~d2_ord;
   wire d2_cls_fc = d2_cls_f | d2_cls_c | d2_cls_m | d2_cls_s;
   wire [1:0] d2_cls = d2_cls_i ? C_I2 : d2_cls_l ? C_L : C_F;   // CTF falls to C_F (shares u_iq_f)
   wire [RN_PBITS-1:0] d2_prd_g = d2_rd_v ? rn_prd_b : {RN_PBITS{1'b0}};
   wire       d2_st_nb = d2_is_store & ~d2_is_amo & ~d2_is_cbo;
   wire       d2_ld_nb = d2_is_mem & ~d2_is_store & ~d2_is_amo & ~d2_is_cbo;
   wire [2:0] d2_srdy = {pnd_r3_b | ~d2_rs3_v, pnd_r2_b | ~d2_rs2_v | d2_st_nb, pnd_r1_b | ~d2_rs1_v};
   // slot C dispatch (Stage 3). three_wide (= IW>=3) is the master enable; at IW=2 it is 0,
   // so the frontend never presents slot C and d3_take/rn_valid_c stay 0 -- retire-identical.
   // rn_valid_c itself is defined at the d3_take steering below; used forward here (a net).
   localparam TW3 = (IW >= 3) ? 1'b1 : 1'b0;
   wire three_wide = TW3;
   // Slot C full classes: the dispatch swizzle lets slot C reach ANY pipe now (the 3rd ALU is
   // gone), so it needs the same LS/FC/ALU split as slots A and B, not just "ALU-only".
   wire d3_fp_valid, d3_use_fpu;
   decode_fp u_d3fp_disp
     (.insn(d3_insn), .fp_valid(d3_fp_valid), .use_fpu(d3_use_fpu), .fp_class(),
      .op(), .op_mod(), .src_fmt(), .dst_fmt(), .int_fmt(),
      .rnd(), .op0_sel(), .op1_sel(), .op2_sel(), .op0_int(), .wr_fp());
   wire d3_cls_f = d3_fp_valid & d3_use_fpu & ~d3_is_mem & ~d3_is_amo
                 & ~fs_off & ~d3_illegal & ~d3_fault & ~d3_is_irqop;
   wire d3_cls_c = (d3_is_branch | d3_is_jump | d3_is_jalr) & ~d3_illegal & ~d3_fault & ~d3_is_irqop;
   wire d3_cls_m = d3_is_mul & ~d3_illegal & ~d3_fault & ~d3_is_irqop;
   wire d3_cls_s = (d3_insn[6:2] == 5'b11100) & ~d3_illegal & ~d3_fault;   // SYSTEM opcode, the irqop included (C3 step 3)
   wire d3_cls_l = d3_ord & ~d3_cls_f & ~d3_cls_c & ~d3_cls_m & ~d3_cls_s;
   wire d3_cls_i = ~d3_ord;
   wire d3_cls_fc = d3_cls_f | d3_cls_c | d3_cls_m | d3_cls_s;
   // Destination shard = the UNIT that writes it (declared with its rationale above).
   assign d_shard  = (d_is_mem | d_is_amo) ? SH_LD
                   : (d_is_fp | d_is_mul)                          ? SH_FE   // FP ops, in-core included: the base routing
                   : d_cls_c                          ? SH_FE   // jal/jalr link: the FP/CTF pipe's slice
                   : d_ord                            ? SH_LD   // CSR, in-core FP: M writes
                   :                                    SH_IE;  // the ALU, alone
   assign d2_shard = (d2_is_mem | d2_is_amo) ? SH_LD
                   : (d2_is_fp | d2_is_mul)                            ? SH_FE
                   : d2_cls_c                            ? SH_FE   // jal/jalr link: FP/CTF pipe's slice
                   : d2_ord                              ? SH_LD
                   :                                       SH_IE2;   // the second ALU's shard
   assign d3_shard = (d3_is_mem | d3_is_amo) ? SH_LD
                   : (d3_is_fp | d3_is_mul)                            ? SH_FE
                   : d3_cls_c                            ? SH_FE   // jal/jalr link: the FP/CTF slice
                   : d3_ord                              ? SH_LD
                   :                        (d2_cls_i ? SH_IE : SH_IE2);  // ALU: swizzled -- ALUa if I2 is ALU, else ALUb (ALUb preferred)
   wire [RN_PBITS-1:0] d3_prd_g = d3_rd_v ? rn_prd_c : {RN_PBITS{1'b0}};
   // slot C source readiness (mirror d2_srdy); slot C is ALU-only here, so no store term.
   wire pnd_s1_c, pnd_s2_c, pnd_s3_c, pnd_m1_c, pnd_m2_c, pnd_m3_c;
   wire pnd_r1_c = ~rn_byp1_c & (rn_lv1_c ? pnd_s1_c : pnd_m1_c);
   wire pnd_r2_c = ~rn_byp2_c & (rn_lv2_c ? pnd_s2_c : pnd_m2_c);
   wire pnd_r3_c = ~rn_byp3_c & (rn_lv3_c ? pnd_s3_c : pnd_m3_c);
   wire       d3_st_nb = d3_is_store & ~d3_is_amo & ~d3_is_cbo;
   wire       d3_ld_nb = d3_is_mem & ~d3_is_store & ~d3_is_amo & ~d3_is_cbo;
   wire [2:0] d3_srdy = {pnd_r3_c | ~d3_rs3_v, pnd_r2_c | ~d3_rs2_v | d3_st_nb, pnd_r1_c | ~d3_rs1_v};
   wire d3_plain = ~(d3_is_serialize | d3_is_fencei | d3_is_cbo | d3_is_amo | d3_is_csr | d3_illegal | d3_fault | d3_is_irqop);
   wire d_plain  = ~(d_is_serialize  | d_is_fencei  | d_is_cbo  | d_is_amo  | d_is_csr  | d_illegal  | d_fault  | d_is_irqop);
   wire d2_plain = ~(d2_is_serialize | d2_is_fencei | d2_is_cbo | d2_is_amo | d2_is_csr | d2_illegal | d2_fault | d2_is_irqop);

   wire [NWB_C-1:0]        wkv  = {we_ie3, we_ie2, we_fe, we_ld, we_ie};
   wire [NWB_C*RN_PBITS-1:0] wkp = {wa_ie3, wa_ie2, wa_fe, wa_ld, wa_ie};

   wire ri_ready, ri_iss_v, ri_blk_v;  wire [IBI-1:0] ri_d_ent, ri_iss_ent;
   wire ri2_ready, ri2_iss_v, ri2_blk_v; wire [IBI-1:0] ri2_d_ent, ri2_iss_ent;
   wire [ROB_IDXB-1:0] ri2_iss_rob;  wire [RN_PBITS-1:0] ri2_blk_pr;  wire [IBI:0] ri2_occ;  wire ri2_take;
   // 3rd integer scheduler (the third ALU, Stage 3): dead at IW=2 (rn_valid_c=0)
   wire ri3_ready, ri3_iss_v, ri3_blk_v; wire [IBI-1:0] ri3_d_ent, ri3_iss_ent;
   wire [ROB_IDXB-1:0] ri3_iss_rob;  wire [RN_PBITS-1:0] ri3_blk_pr;  wire [IBI:0] ri3_occ;  wire ri3_take;
   wire [ROB_IDXB-1:0] ri_iss_rob;
   wire [RN_PBITS-1:0] ri_blk_pr;      wire [IBI:0] ri_occ;
   wire rl_ready, rl_iss_v, rl_blk_v;  wire [IBL-1:0] rl_d_ent, rl_iss_ent;
   wire [ROB_IDXB-1:0] rl_iss_rob;
   wire [RN_PBITS-1:0] rl_blk_pr;      wire [IBL:0] rl_occ;
   wire rf_ready, rf_iss_v, rf_blk_v;  wire [IBF-1:0] rf_d_ent, rf_iss_ent;
   wire [ROB_IDXB-1:0] rf_iss_rob;
   wire [RN_PBITS-1:0] rf_blk_pr;      wire [IBF:0] rf_occ;
   wire ri_take, rl_take, rf_take;
   wire i_needs_m;                     // the issue register holds an M-class op
   wire i_needs_f;                     // ...or an F-class one
   wire f_advance;
   wire cf_advance;                    // the control-flow completion stage can take a new op

   // ---- THE ALU'S OWN ISSUE PORT (item 10d-i, 2026-09-05) -----------------------------
   // One issue register served every scheduler, so the machine issued ONE instruction per
   // cycle whatever dispatch and retire did, and the ALU waited whenever a load, store or
   // branch went first. The integer scheduler now picks into its own register, beside the
   // M/F one: an ALU op and a memory op issue in the same cycle. The ALU op completes in
   // its issue cycle, so this port is free every cycle (unit_busy 0); it has two operand
   // reads of its own (ra4/ra5), its own exec unit and the same one-deep forward.
   reg                 a_v;
   reg [IBI-1:0]       a_ent;
   reg [ROB_IDXB-1:0]  a_rob;
   reg [RN_PBITS-1:0]  a_ps1, a_ps2;
   initial begin a_v = 1'b0; a_ent = {IBI{1'b0}}; a_rob = {ROB_IDXB{1'b0}}; a_ps1 = {RN_PBITS{1'b0}}; a_ps2 = {RN_PBITS{1'b0}}; end
   wire   iss_alu = a_v & ~redirect;                      // completes here, always
   reg                 a2_v;                               // the second ALU's port (10d-ii)
   reg [IBI-1:0]       a2_ent;
   reg [ROB_IDXB-1:0]  a2_rob;
   reg [RN_PBITS-1:0]  a2_ps1, a2_ps2;
   initial begin a2_v = 1'b0; a2_ent = {IBI{1'b0}}; a2_rob = {ROB_IDXB{1'b0}}; a2_ps1 = {RN_PBITS{1'b0}}; a2_ps2 = {RN_PBITS{1'b0}}; end
   wire   iss_alu2 = a2_v & ~redirect;
   reg                 a3_v;                               // the third ALU's port (Stage 3)
   reg [IBI-1:0]       a3_ent;
   reg [ROB_IDXB-1:0]  a3_rob;
   reg [RN_PBITS-1:0]  a3_ps1, a3_ps2;
   initial begin a3_v = 1'b0; a3_ent = {IBI{1'b0}}; a3_rob = {ROB_IDXB{1'b0}}; a3_ps1 = {RN_PBITS{1'b0}}; a3_ps2 = {RN_PBITS{1'b0}}; end
   wire   iss_alu3 = a3_v & ~redirect;

   // ---- DISPATCH STAGE (cycle boundary) -------------------------------------------------
   // The swizzle crossbar (slot->pipe mux, selects gated by the deep d3_take accept chain)
   // fed the schedulers' e_r/e_ps registers combinationally, and that mux+hit sat on the
   // dispatch->e_r critical path (-0.42 ns vs the pre-swizzle 2-ALU core). This stage is a
   // per-pipe register between the crossbar output and each scheduler: the crossbar (and the
   // per-slot same-cycle wakeup fold, below) resolve into stg_* at T; the scheduler, psmem
   // and plmem consume the REGISTERED stg_* at T+1. Cost: +1 cycle to refill after a redirect.
   // Each pipe takes at most one dispatch per cycle (the swizzle's <=1 LS, <=1 FC, <=1 per
   // ALU), so each stage is 1-deep. Back-pressure stays at the frontend: iq_ready becomes
   // "~stg_v | sched_room" so a stuck stage (scheduler full) holds dispatch, never drops it.
   // Wakeup-catch: an op waiting in the stage must not miss a writeback that fires while it
   // waits. The dispatch-cycle (T) writeback is folded per-slot into stg_r at load (srdy_hit
   // below, computed from the early slot pregs so it stays off the mux->register path); every
   // later stuck cycle ORs a fresh wkv snoop of the stage's own pregs; and the move cycle is
   // caught by the scheduler's own fill hit(d_ps). See docs/OOO2-Spec.md 15.
   reg              stg_v_ia, stg_v_ib, stg_v_l, stg_v_f;
   reg [ROB_IDXB-1:0] stg_rob_ia, stg_rob_ib, stg_rob_l, stg_rob_f;
   reg [3*RN_PBITS-1:0] stg_ps_ia, stg_ps_ib, stg_ps_l, stg_ps_f;
   reg [1:0]        stg_r_ia, stg_r_ib;
   reg [2:0]        stg_r_l, stg_r_f;
   reg [RN_PBITS-1:0] stg_prd_ia, stg_prd_ib, stg_prd_l, stg_prd_f;
   reg [PLW-1:0]    stg_pl_ia, stg_pl_ib, stg_pl_l, stg_pl_f;
   initial begin stg_v_ia=1'b0; stg_v_ib=1'b0; stg_v_l=1'b0; stg_v_f=1'b0; end
   // The scheduler accepts the stage op when it has room; back-pressure to the frontend below.
   wire mv_ia = stg_v_ia & ri_ready;
   wire mv_ib = stg_v_ib & ri2_ready;
   wire mv_l  = stg_v_l  & rl_ready;
   wire mv_f  = stg_v_f  & rf_ready;

   ooo2_iq #(.NENT(NI),.IDXB(IBI),.NSRC(2),.ROBB(ROB_IDXB),.PBITS(RN_PBITS),.NWB(NWB_C),
             .FIXEDL(1),.INORDER(0),.REGRDY(1)) u_iq_i
     (.clk(clk),.reset(reset),
      .d_valid(mv_ia),.d_ready(ri_ready),.d_rob(stg_rob_ia),
      .d_ps(stg_ps_ia[2*RN_PBITS-1:0]),.d_r(stg_r_ia),.d_prd(stg_prd_ia),.d_ent(ri_d_ent),
      .wb_v(wkv),.wb_preg(wkp),
      .unit_busy(1'b0),.iss_v(ri_iss_v),.iss_ent(ri_iss_ent),.iss_rob(ri_iss_rob),
     .iss_take(ri_take),
      .hold_v(a_v),.hold_ent(a_ent),
      .blk_v(ri_blk_v),.blk_pr(ri_blk_pr),.flush(redirect),.occupancy(ri_occ));
   // THE SECOND INTEGER SCHEDULER (item 10d-ii): slot B's ALU ops, into the second ALU.
   ooo2_iq #(.NENT(NI),.IDXB(IBI),.NSRC(2),.ROBB(ROB_IDXB),.PBITS(RN_PBITS),.NWB(NWB_C),
             .FIXEDL(1),.INORDER(0),.REGRDY(1)) u_iq_i2
     (.clk(clk),.reset(reset),
      .d_valid(mv_ib),.d_ready(ri2_ready),.d_rob(stg_rob_ib),
      .d_ps(stg_ps_ib[2*RN_PBITS-1:0]),.d_r(stg_r_ib),.d_prd(stg_prd_ib),.d_ent(ri2_d_ent),
      .wb_v(wkv),.wb_preg(wkp),
      .unit_busy(1'b0),.iss_v(ri2_iss_v),.iss_ent(ri2_iss_ent),.iss_rob(ri2_iss_rob),
     .iss_take(ri2_take),
      .hold_v(a2_v),.hold_ent(a2_ent),
      .blk_v(ri2_blk_v),.blk_pr(ri2_blk_pr),.flush(redirect),.occupancy(ri2_occ));
   // THE THIRD INTEGER SCHEDULER (Stage 3): slot C's ALU ops, into the third ALU. Dead at
   // IW=2 (rn_valid_c=0); the C4 dispatch step gives it the real slot-C route.
   ooo2_iq #(.NENT(NI),.IDXB(IBI),.NSRC(2),.ROBB(ROB_IDXB),.PBITS(RN_PBITS),.NWB(NWB_C),
             .FIXEDL(1),.INORDER(0)) u_iq_i3
     (.clk(clk),.reset(reset),
      .d_valid(1'b0),.d_ready(ri3_ready),.d_rob(rob_d_idx3),   // DEAD: the 3rd ALU is removed; slot-3 ALU swizzles to ALUa/ALUb
      .d_ps({rn_prs2_c, rn_prs1_c}),.d_r(d3_srdy[1:0]),.d_prd(d3_prd_g),.d_ent(ri3_d_ent),
      .wb_v(wkv),.wb_preg(wkp),
      .unit_busy(1'b0),.iss_v(ri3_iss_v),.iss_ent(ri3_iss_ent),.iss_rob(ri3_iss_rob),
     .iss_take(ri3_take),
      .hold_v(a3_v),.hold_ent(a3_ent),
      .blk_v(ri3_blk_v),.blk_pr(ri3_blk_pr),.flush(redirect),.occupancy(ri3_occ));

   ooo2_iq #(.NENT(NL),.IDXB(IBL),.NSRC(3),.ROBB(ROB_IDXB),.PBITS(RN_PBITS),.NWB(NWB_C),
             .FIXEDL(0),.INORDER(1)) u_iq_l
     (.clk(clk),.reset(reset),
      .d_valid(mv_l),.d_ready(rl_ready),
      .d_rob(stg_rob_l),
      .d_ps(stg_ps_l),.d_r(stg_r_l),
      .d_prd(stg_prd_l),.d_ent(rl_d_ent),
      .wb_v(wkv),.wb_preg(wkp),
      .unit_busy(~m_advance | (i_v & i_needs_m)),
      .iss_v(rl_iss_v),.iss_ent(rl_iss_ent),.iss_rob(rl_iss_rob),
     .iss_take(rl_take),
      .hold_v(i_v & (i_cls == C_L)),.hold_ent(i_ent[IBL-1:0]),
      .blk_v(rl_blk_v),.blk_pr(rl_blk_pr),.flush(redirect),.occupancy(rl_occ));

   // INORDER(0): FP arith may reorder freely. It has no memory ordering to respect and
   // cannot trap, and reordering is the entire point -- the Gaussian Blur loop
   // (workloads/blurbench) is a 4-deep serial fadds chain whose taps are independent, and
   // it ran at exactly its critical path (39.50 cyc/px against 5 levels x 8 cycles) because
   // in-order issue would not let the NEXT iteration's multiplies start early.
   ooo2_iq #(.NENT(NF),.IDXB(IBF),.NSRC(3),.ROBB(ROB_IDXB),.PBITS(RN_PBITS),.NWB(NWB_C),
             .FIXEDL(0),.INORDER(0)) u_iq_f
     (.clk(clk),.reset(reset),
      // FP-arith AND control flow (jal/jalr/bXX) share this one queue and issue slot.
      .d_valid(mv_f),.d_ready(rf_ready),
      .d_rob(stg_rob_f),
      .d_ps(stg_ps_f),.d_r(stg_r_f),
      .d_prd(stg_prd_f),.d_ent(rf_d_ent),
      .wb_v(wkv),.wb_preg(wkp),
      .unit_busy(j_v & ~j_adv),           // j_* can't take: occupied and its op not draining
      .iss_v(rf_iss_v),.iss_ent(rf_iss_ent),.iss_rob(rf_iss_rob),
     .iss_take(rf_take),
      .hold_v(j_v),.hold_ent(j_ent),
      .blk_v(rf_blk_v),.blk_pr(rf_blk_pr),.flush(redirect),.occupancy(rf_occ));

   // Dispatch back-pressure comes from whichever scheduler this instruction is routed to.
   // Stage-aware: a slot may dispatch iff its target pipe's dispatch stage is empty OR drains
   // into the scheduler this cycle (~stg_v | sched_room). A stuck stage (scheduler full) thus
   // holds dispatch at the frontend rather than losing the op behind the stage register.
   wire alua_room = ~stg_v_ia | ri_ready;
   wire alub_room = ~stg_v_ib | ri2_ready;
   wire m_room    = ~stg_v_l  | rl_ready;
   wire f_room    = ~stg_v_f  | rf_ready;
   wire iq_ready = d_cls_i ? alua_room : d_cls_l ? m_room : f_room;   // CTF falls to F (shared queue)
   wire iq_ready_b = d2_cls_i ? alub_room : d2_cls_l ? m_room : f_room;
   wire b_to_i2 = rn_valid_b & d2_cls_i, b_to_l = rn_valid_b & d2_cls_l, b_to_f = rn_valid_b & d2_cls_fc;
   // ---- dispatch swizzle: slot C reaches ANY pipe (the 3rd ALU is gone) ----
   // ALUa <- I1 if ALU, else I3(ALU & ALU2); ALUb <- I2 if ALU, else I3(ALU & ~ALU2).
   // I3's ALU PREFERS ALUb (falls to ALUa only when ALUb is taken by an I2 ALU): I1 always
   // takes ALUa, so ALUa is busier under the fetch-length bias; this evens the two ALUs.
   // LS/FC <- whichever slot is LS/FC (the accepted prefix has <=1 of each).
   wire c_to_ib = rn_valid_c & d3_cls_i & ~d2_cls_i;  // slot3 ALU -> ALUb (PREFERRED: free unless I2 is ALU)
   wire c_to_ia = rn_valid_c & d3_cls_i &  d2_cls_i;  // slot3 ALU -> ALUa (ALUb taken by an I2 ALU)
   wire c_to_l  = rn_valid_c & d3_cls_l;              // slot3 LS  -> M
   wire c_to_f  = rn_valid_c & d3_cls_fc;             // slot3 FC  -> F/CTF
   wire l_slot0 = rn_valid   & d_cls_l;               // slot1 -> M
   wire f_slot0 = rn_valid   & d_cls_fc;              // slot1 -> F/CTF
   // slot-3's destination-scheduler ready (by class, NOT rn_valid_c -> no combinational loop
   // through d3_take): ALU->ALUa/ALUb by I1's class, LS->M, FC->F.
   wire iq_ready_c = d3_cls_i ? (d2_cls_i ? alua_room : alub_room)
                   : d3_cls_l ? m_room : f_room;
   wire [RS_IDXB-1:0] iq_d_ent = d_cls_i ? {{(RS_IDXB-IBI){1'b0}}, ri_d_ent}
                               : d_cls_l ? {{(RS_IDXB-IBL){1'b0}}, rl_d_ent}
                               :           {{(RS_IDXB-IBF){1'b0}}, rf_d_ent};

   // ISSUE ARBITRATION, one per cycle into the single issue register. Long-latency classes
   // win: they are gated on M being free anyway, so they only bid when they can make
   // progress, while an ALU op can always go next cycle instead.
   wire pick_l = rl_iss_v;
   wire pick_i = ri_iss_v;                                // its own port: never waits for M
   wire pick_i2 = ri2_iss_v;                              // the second ALU's, likewise
   wire pick_i3 = ri3_iss_v;                              // the third ALU's, likewise
   // The F scheduler no longer shares this port: it issues into its own select register
   // j_* (CTF-on-FP). i_* is M-only now, so no arbitration and no class mux here.
   wire iq_iss_v = pick_l;                                // the M port
   wire [1:0] pick_cls = C_L;                             // i_* only ever holds an ordered op
   wire [RS_IDXB-1:0] iq_iss_ent = {{(RS_IDXB-IBL){1'b0}}, rl_iss_ent};
   wire [ROB_IDXB-1:0] iq_iss_rob = rl_iss_rob;
   // THE SCHEDULER'S JOB IS TO PRODUCE AN INDEX; everything else about the uop is looked up
   // with it. The source tags used to come OUT of each queue as `iss_ps` -- an async read of
   // the entry array (e_ps[sel], a 16:1 mux over FLOPS) then a 3-way class mux, two levels
   // landing on the i_ps* capture flops, and the tail of the post-route critical path
   // (m_addr_reg[12]_replica -> i_ps1_reg[3]/D, WNS -0.041 at DIV8=48).
   //
   // e_ps CANNOT be a LUTRAM: ooo2_iq.v:130 broadcasts every entry's tags to the wakeup
   // comparators and distributed RAM has one read port per instance. Synthesis proves the
   // split inside that very module -- e_prd and e_rob, read only at [sel], became RAM32M;
   // e_ps and e_r, read by every comparator, stayed flops. e_r MUST be flops, it is the
   // wakeup state. e_ps need not be, so the tags are kept REDUNDANTLY here, written at
   // dispatch beside plmem and read at the selected index. plmem is already unified across
   // classes (OFF_I/OFF_L/OFF_F), so one indexed read collapses BOTH muxes.
   //
   // ON THIS FPGA THE WIN IS ROUTING, NOT LEVELS. The failing paths are ~68% route / ~32%
   // logic, so a mux over N scattered flop groups is paying for the GATHER, and a LUTRAM is
   // one compact primitive with local routing. Flop-to-flop through random logic is not
   // automatically better than RAM-to-RAM here; that is an ASIC intuition.
   //
   // NO NEW PIPELINE STAGE, deliberately. i_ps* were already flops; this replaces the logic
   // FEEDING them, so depth is unchanged and the cosim is BIT-IDENTICAL (14,657,366 retires).
   // The index is COMBINATIONAL (this cycle's pick), not i_ent (last cycle's): reading
   // plmem's registered port instead would put a LUTRAM output on the PRF address pins
   // ra1/ra2/ra3, which rule I6 forbids -- that trades this path for a worse one.
   //
   // ONE READ PER CLASS, AT THAT CLASS'S OWN CANDIDATE; THE PICK SELECTS THE RESULT. The
   // pick (pick_l/pick_f/pick_i) is the last thing the issue cycle knows -- it carries
   // every class's readiness, and through the memory class's unit_busy the LSU's completion
   // and the dTLB compare -- and it used to be the SELECT of the mux on this array's address
   // pin: rule I6 broken at the exact spot 3b518832 had just cleared. Read the array three
   // times (duplication buys read ports, and this array is 30 x 27 bits) at addresses that
   // are, for the in-order class, a head pointer plus a constant, and let the pick choose
   // among three 27-bit results one LUT before the capture flop. Same value, same cycle.
   // ONE ARRAY PER SCHEDULER (item 10b): slot A and slot B dispatch to different schedulers,
   // so each array keeps one write per cycle; the issue-side select on pick_* was already a
   // 3:1 mux, now on three reads instead of three indexes.
   reg  [3*RN_PBITS-1:0] psmem_i [0:NI-1];
   reg  [3*RN_PBITS-1:0] psmem_i2 [0:NI-1];
   reg  [3*RN_PBITS-1:0] psmem_l [0:NL-1];
   reg  [3*RN_PBITS-1:0] psmem_f [0:NF-1];
   wire [3*RN_PBITS-1:0] ps_out   = psmem_l[rl_iss_ent];
   wire [3*RN_PBITS-1:0] ps_out_f = psmem_f[rf_iss_ent];   // FP/CTF source tags into j_*
   wire [3*RN_PBITS-1:0] ps_out_a = psmem_i[ri_iss_ent];
   wire [3*RN_PBITS-1:0] ps_out_a2 = psmem_i2[ri2_iss_ent];
   wire [3*RN_PBITS-1:0] ps_out_a3 = {(3*RN_PBITS){1'b0}};   // the third ALU is dead since the swizzle (u_iq_i3 never fills)
   wire [3*RN_PBITS-1:0] ps_in   = {rn_prs3, rn_prs2, rn_prs1};
   wire [3*RN_PBITS-1:0] ps_in_b = {rn_prs3_b, rn_prs2_b, rn_prs1_b};
   wire [3*RN_PBITS-1:0] ps_in_c = {rn_prs3_c, rn_prs2_c, rn_prs1_c};
   always @(posedge clk) begin
      // Written when the stage op moves into its scheduler (T+1), at the entry that scheduler
      // allocates (*_d_ent), from the REGISTERED stage payload -- co-timed with the e_ps write.
      if (mv_ia) psmem_i[ri_d_ent]  <= stg_ps_ia;
      if (mv_ib) psmem_i2[ri2_d_ent] <= stg_ps_ib;
      if (mv_l)  psmem_l[rl_d_ent]  <= stg_ps_l;
      if (mv_f)  psmem_f[rf_d_ent]  <= stg_ps_f;
   end
   wire [RN_PBITS-1:0] iq_iss_ps1 = ps_out[0 +: RN_PBITS];
   wire [RN_PBITS-1:0] iq_iss_ps2 = ps_out[RN_PBITS +: RN_PBITS];
   wire [RN_PBITS-1:0] iq_iss_ps3 = ps_out[2*RN_PBITS +: RN_PBITS];
   wire iq_iss_take;
   assign ri_take = pick_i;                              // the ALU port takes every cycle; a take in
   assign ri2_take = pick_i2;                            // the redirect cycle dies in a_v/a2_v (cleared)
   assign ri3_take = pick_i3;                            // the third ALU, likewise (dead at IW=2)
   assign rl_take = pick_l & iq_iss_take;
   // The shared FP/CTF queue feeds j_* directly (single source). j_isctf, from the picked op's
   // payload, routes the drain: control flow to cf_*, FP to f_valid. (CTF-over-FP priority is a
   // later addition; for now the queue picks by its own policy.)
   wire j_isctf   = qf_is_branch | qf_is_jump | qf_is_jalr;  // valid when j_v (qf_* = j_*'s payload)
   wire j_ismd    = qf_is_mul;                           // ...or a mul/div (C1): the MD stage's drain
   wire j_issys   = (qf_insn[6:2] == 5'b11100);         // ...or a system op (C3 step 3): the SYSQ's drain
   wire md_advance;                                      // the MD stage can take one (defined with it)
   wire sy_advance;                                      // the SYSQ can take one (defined with it)
   wire j_needs_c = j_v & j_isctf;                       // j_* holds a control-flow op...
   wire j_needs_m = j_v & j_ismd;                        // ...or a mul/div...
   wire j_needs_s = j_v & j_issys;                       // ...or a system op...
   wire j_needs_f = j_v & ~j_isctf & ~j_ismd & ~j_issys; // ...or an FP op
   wire j_adv     = j_needs_c ? cf_advance : j_needs_m ? md_advance : j_needs_s ? sy_advance
                  : (j_needs_f ? f_advance : 1'b1);
   wire j_ready   = ~j_v | j_adv;
   assign rf_take = rf_iss_v;   // rf_iss_v is already gated by unit_busy = j_v & ~j_adv
   wire iq_blk_v = rl_blk_v | rf_blk_v | ri_blk_v;
   wire [RN_PBITS-1:0] iq_blk_pr = rl_blk_v ? rl_blk_pr : rf_blk_v ? rf_blk_pr : ri_blk_pr;

   // ---- SELECT GETS ITS OWN STAGE -------------------------------------------------
   // Select, payload read, register read, execute and writeback in ONE cycle was 50 logic
   // levels and 13.1 ns against a 6 ns period. doc 14 permits exactly this cut: "an
   // implementation may pipeline select -> operand read -> execute".
   //
   // Cycle N selects and registers the choice. Cycle N+1 reads the payload and the register
   // file, executes and writes back. Stage N+1 is SHORTER than the in-order X stage it
   // replaces -- its register-file address is a flop here, where X first had to walk the
   // rename map -- so the pipeline it feeds is unaffected. The cost lands on the redirect
   // path, one stage deeper, which is where a scheduler's cost belongs.
   //
   // BACK-TO-BACK DEPENDENTS ARE PRESERVED. The producer selected at N executes at N+1;
   // ooo2_iq wakes its consumer AT SELECT, so the consumer is selected at N+1 and executes
   // at N+2 -- consecutive execute cycles, no bubble. The same timing is why no operand
   // forwarding exists anywhere here: every producer's write has landed before its consumer
   // reads.
   reg                 i_v;
   reg [1:0]           i_cls;      // which scheduler it came from -- selects the hold port
   reg [RS_IDXB-1:0]   i_ent;
   reg [ROB_IDXB-1:0]  i_rob;
   reg [RN_PBITS-1:0]  i_ps1, i_ps2, i_ps3;
   initial i_v = 1'b0;

   // The independent FP/CTF select register (CTF-on-FP). i_* is M-only now; this port has its
   // OWN three PRF read ports, so a branch or FP op issues in the same cycle as a load instead
   // of contending for the M select slot. Shared by the FP scheduler (rf) and the control-flow
   // scheduler (rc), with CTF winning the slot (pick_ctf). j_isctf says which side it holds:
   // a control-flow op drains into cf_* (the branch completion stage), an FP op into f_valid.
   reg                 j_v;
   reg [IBF-1:0]       j_ent;
   reg [ROB_IDXB-1:0]  j_rob;
   reg [RN_PBITS-1:0]  j_ps1, j_ps2, j_ps3;
   initial j_v = 1'b0;

   // An ordered op completes when M takes it; an ALU op completes unconditionally, because
   // SH_IE has exactly one writer and it is this one. Nothing can be in the way.
   assign i_needs_f   = i_v & (i_cls == C_F);
   assign i_needs_m   = i_v & q_ord & ~i_needs_f;
   wire   i_done      = i_v & (i_needs_f ? f_advance : m_advance);   // only M/F ops live here now
   wire   iss_ready   = ~i_v | i_done;
   assign iq_iss_take = iq_iss_v & iss_ready;            // likewise: i_v is cleared by the redirect
   // ~redirect on BOTH. The register is cleared on a redirect, but these are
   // combinational off i_v -- without the guard an instruction being squashed still writes
   // the register file and still marks its ROB slot done, in the very cycle rename is
   // rolling that physical register back. That is the zombie writeback in miniature, and it
   // is what failed all 61 virtual-memory tests: they are the ones that trap often.
   wire   iss_m       = i_needs_m & m_advance & ~redirect;     // loads M this cycle
   wire   iss_f       = j_needs_f & f_advance & ~redirect;    // j_* drains an FP op into f_valid
   wire   iss_c       = j_needs_c & cf_advance & ~redirect;   // ...or a control-flow op into cf_*
   wire   iss_md      = j_needs_m & md_advance & ~redirect;   // ...or a mul/div into the MD stage (C1)
   wire   iss_sys     = j_needs_s & sy_advance & ~redirect;   // ...or a system op into the SYSQ (C3 step 3)
      always @(posedge clk) begin                            // the ALU port (see above)
      if (reset | redirect) a_v <= 1'b0;
      else begin
         a_v <= ri_take;
         if (ri_take) begin
            a_ent <= ri_iss_ent;  a_rob <= ri_iss_rob;
            a_ps1 <= ps_out_a[0 +: RN_PBITS];  a_ps2 <= ps_out_a[RN_PBITS +: RN_PBITS];
         end
      end
   end
   always @(posedge clk) begin                            // the second ALU port
      if (reset | redirect) a2_v <= 1'b0;
      else begin
         a2_v <= ri2_take;
         if (ri2_take) begin
            a2_ent <= ri2_iss_ent;  a2_rob <= ri2_iss_rob;
            a2_ps1 <= ps_out_a2[0 +: RN_PBITS];  a2_ps2 <= ps_out_a2[RN_PBITS +: RN_PBITS];
         end
      end
   end
   always @(posedge clk) begin                            // the third ALU port (Stage 3)
      if (reset | redirect) a3_v <= 1'b0;
      else begin
         a3_v <= ri3_take;
         if (ri3_take) begin
            a3_ent <= ri3_iss_ent;  a3_rob <= ri3_iss_rob;
            a3_ps1 <= ps_out_a3[0 +: RN_PBITS];  a3_ps2 <= ps_out_a3[RN_PBITS +: RN_PBITS];
         end
      end
   end

   always @(posedge clk) begin
      if (reset | redirect) i_v <= 1'b0;
      else if (iss_ready) begin
         i_v <= iq_iss_take;
         if (iq_iss_take) begin
            i_cls <= pick_cls;    i_ent <= iq_iss_ent;  i_rob <= iq_iss_rob;
            i_ps1 <= iq_iss_ps1;  i_ps2 <= iq_iss_ps2;  i_ps3 <= iq_iss_ps3;
         end
      end
   end

   // j_* loads the FP/CTF queue's pick when it can accept (empty, or its op draining into
   // cf_*/f_valid this cycle).
   always @(posedge clk) begin
      if (reset | redirect) j_v <= 1'b0;
      else if (j_ready) begin
         j_v <= rf_take;
         if (rf_take) begin
            j_ent <= rf_iss_ent;  j_rob <= rf_iss_rob;
            j_ps1 <= ps_out_f[0 +: RN_PBITS];  j_ps2 <= ps_out_f[RN_PBITS +: RN_PBITS];
            j_ps3 <= ps_out_f[2*RN_PBITS +: RN_PBITS];
         end
      end
   end

   // Payload: packed at dispatch, unpacked at issue with the SAME concatenation, so a
   // width or ordering mistake is a lint error rather than a wrong instruction.
   localparam integer PLW = PCW + 32 + 1 + SEQW + PDW + PCW + 6 + 1 + RN_PBITS + 3 + 6   // shard is 3 bits
                          + 64 + 2 + 1 + 1 + 1 + 1 + 5 + 1 + 1 + 1 + 1 + 1 + 3 + 1 + 1
                          + 1 + 1 + 1 + 1 + 1 + 1 + 4 + 64
                          + 6 + 1 + 1 + 2 + 1 + 1 + 3 + 1 + 1   // execute controls
                          + 1 + 1 + 1                           // source-valid bits
                          + 1                                   // ordered
                          + SQ_IB                               // store-buffer slot (a store's)
                          + LQ_IB;                              // load-queue slot
   wire [PLW-1:0] pl_in = {d_pc, d_insn, d_rvc, d_seq, d_pdet, d_pred_npc, d_rd, d_rd_v,
                           (d_rd_v ? rn_prd : {RN_PBITS{1'b0}}), d_shard, d_rs1, d_imm,
                           d_mem_size, d_mem_signed, d_is_mem, d_is_store, d_is_amo,
                           d_amo_func, d_is_branch, d_is_jump, d_is_jalr, d_is_mul,
                           d_is_csr, d_csr_func, d_is_serialize, d_is_fp, d_is_fencei,
                           d_is_cbo, d_cbo_zero, d_cbo_keep, d_illegal, d_fault,
                           d_fault_cause, d_fault_tval,
                           d_alu_op, d_alu_w, d_alu_uw, d_op1_sel, d_op2_imm, d_res_link,
                           // decode-redirect (dcr0) makes taken the prediction: taken now
                           // matches (mis_taken=0), and a not-taken resolve must redirect
                           // back to fall-through (mis_nt=1). See dcr0/dec_red above.
                           d_br_func, (dcr0 ? 1'b0 : d_mis_taken), (dcr0 ? 1'b1 : d_mis_nt),
                           d_rs1_v, d_rs2_v, d_rs3_v, d_ord, sq_d_idx, lq_d_idx};
   wire [PLW-1:0] pl_in_b = {d2_pc, d2_insn, d2_rvc, d2_seq, d2_pdet, d2_pred_npc, d2_rd, d2_rd_v,
                           (d2_rd_v ? rn_prd_b : {RN_PBITS{1'b0}}), d2_shard, d2_rs1, d2_imm,
                           d2_mem_size, d2_mem_signed, d2_is_mem, d2_is_store, d2_is_amo,
                           d2_amo_func, d2_is_branch, d2_is_jump, d2_is_jalr, d2_is_mul,
                           d2_is_csr, d2_csr_func, d2_is_serialize, d2_is_fp, d2_is_fencei,
                           d2_is_cbo, d2_cbo_zero, d2_cbo_keep, d2_illegal, d2_fault,
                           d2_fault_cause, d2_fault_tval,
                           d2_alu_op, d2_alu_w, d2_alu_uw, d2_op1_sel, d2_op2_imm, d2_res_link,
                           d2_br_func, (dcr1 ? 1'b0 : d2_mis_taken), (dcr1 ? 1'b1 : d2_mis_nt),
                           d2_rs1_v, d2_rs2_v, d2_rs3_v, d2_ord, sq_d_idx, lq_d_idx};
   // slot C's payload (Stage 3): identical shape, for the third ALU. Dead at IW=2.
   wire [PLW-1:0] pl_in_c = {d3_pc, d3_insn, d3_rvc, d3_seq, d3_pdet, d3_pred_npc, d3_rd, d3_rd_v,
                           (d3_rd_v ? rn_prd_c : {RN_PBITS{1'b0}}), d3_shard, d3_rs1, d3_imm,
                           d3_mem_size, d3_mem_signed, d3_is_mem, d3_is_store, d3_is_amo,
                           d3_amo_func, d3_is_branch, d3_is_jump, d3_is_jalr, d3_is_mul,
                           d3_is_csr, d3_csr_func, d3_is_serialize, d3_is_fp, d3_is_fencei,
                           d3_is_cbo, d3_cbo_zero, d3_cbo_keep, d3_illegal, d3_fault,
                           d3_fault_cause, d3_fault_tval,
                           d3_alu_op, d3_alu_w, d3_alu_uw, d3_op1_sel, d3_op2_imm, d3_res_link,
                           d3_br_func, (dcr2 ? 1'b0 : d3_mis_taken), (dcr2 ? 1'b1 : d3_mis_nt),
                           d3_rs1_v, d3_rs2_v, d3_rs3_v, d3_ord, sq_d_idx, lq_d_idx};
   // ---- dispatch-stage load + wakeup snoop ----------------------------------------------
   // The dispatch-cycle (T) writeback fold, computed PER SLOT from the early slot pregs so the
   // wkv compare parallels the accept chain and stays off the crossbar-mux->stg_r path. d_srdy
   // reflects writebacks up to T-1 (the pending read is pure); wk(p) contributes T's.
   function automatic wk;
      input [RN_PBITS-1:0] p;
      wk = (wkv[0] & (wkp[0*RN_PBITS +: RN_PBITS] == p))
         | (wkv[1] & (wkp[1*RN_PBITS +: RN_PBITS] == p))
         | (wkv[2] & (wkp[2*RN_PBITS +: RN_PBITS] == p))
         | (wkv[3] & (wkp[3*RN_PBITS +: RN_PBITS] == p))
         | (wkv[4] & (wkp[4*RN_PBITS +: RN_PBITS] == p));
   endfunction
   wire [2:0] srdy_hit0 = d_srdy  | {wk(rn_prs3),   wk(rn_prs2),   wk(rn_prs1)};
   wire [2:0] srdy_hit1 = d2_srdy | {wk(rn_prs3_b), wk(rn_prs2_b), wk(rn_prs1_b)};
   wire [2:0] srdy_hit2 = d3_srdy | {wk(rn_prs3_c), wk(rn_prs2_c), wk(rn_prs1_c)};
   // Stuck-cycle snoop: fresh wkv match of the stage's OWN pregs, ORed into its ready bits.
   wire [1:0] snp_ia = {wk(stg_ps_ia[1*RN_PBITS +: RN_PBITS]), wk(stg_ps_ia[0*RN_PBITS +: RN_PBITS])};
   wire [1:0] snp_ib = {wk(stg_ps_ib[1*RN_PBITS +: RN_PBITS]), wk(stg_ps_ib[0*RN_PBITS +: RN_PBITS])};
   wire [2:0] snp_l  = {wk(stg_ps_l[2*RN_PBITS +: RN_PBITS]), wk(stg_ps_l[1*RN_PBITS +: RN_PBITS]), wk(stg_ps_l[0*RN_PBITS +: RN_PBITS])};
   wire [2:0] snp_f  = {wk(stg_ps_f[2*RN_PBITS +: RN_PBITS]), wk(stg_ps_f[1*RN_PBITS +: RN_PBITS]), wk(stg_ps_f[0*RN_PBITS +: RN_PBITS])};
   always @(posedge clk) begin
      if (reset) begin stg_v_ia<=1'b0; stg_v_ib<=1'b0; stg_v_l<=1'b0; stg_v_f<=1'b0; end
      else begin
         // ALUa: I1(ALU)->ALUa, or I3(ALU & I2 ALU)->ALUa. Single-in (the swizzle's <=2 ALU).
         if ((rn_valid & d_cls_i) | c_to_ia) begin
            stg_v_ia   <= 1'b1;
            stg_rob_ia <= c_to_ia ? rob_d_idx3 : rob_d_idx;
            stg_ps_ia  <= c_to_ia ? ps_in_c : ps_in;
            stg_r_ia   <= c_to_ia ? srdy_hit2[1:0] : srdy_hit0[1:0];
            stg_prd_ia <= c_to_ia ? d3_prd_g : d_prd_g;
            stg_pl_ia  <= c_to_ia ? pl_in_c : pl_in;
         end else if (mv_ia) stg_v_ia <= 1'b0;
         else if (stg_v_ia) stg_r_ia <= stg_r_ia | snp_ia;
         // ALUb: I2(ALU)->ALUb, or I3(ALU & ~I2 ALU)->ALUb.
         if (b_to_i2 | c_to_ib) begin
            stg_v_ib   <= 1'b1;
            stg_rob_ib <= c_to_ib ? rob_d_idx3 : rob_d_idx2;
            stg_ps_ib  <= c_to_ib ? ps_in_c : ps_in_b;
            stg_r_ib   <= c_to_ib ? srdy_hit2[1:0] : srdy_hit1[1:0];
            stg_prd_ib <= c_to_ib ? d3_prd_g : d2_prd_g;
            stg_pl_ib  <= c_to_ib ? pl_in_c : pl_in_b;
         end else if (mv_ib) stg_v_ib <= 1'b0;
         else if (stg_v_ib) stg_r_ib <= stg_r_ib | snp_ib;
         // M (load/store): whichever slot is the (single) LS.
         if (l_slot0 | b_to_l | c_to_l) begin
            stg_v_l   <= 1'b1;
            stg_rob_l <= l_slot0 ? rob_d_idx : b_to_l ? rob_d_idx2 : rob_d_idx3;
            stg_ps_l  <= l_slot0 ? ps_in : b_to_l ? ps_in_b : ps_in_c;
            stg_r_l   <= l_slot0 ? srdy_hit0 : b_to_l ? srdy_hit1 : srdy_hit2;
            stg_prd_l <= l_slot0 ? d_prd_g : b_to_l ? d2_prd_g : d3_prd_g;
            stg_pl_l  <= l_slot0 ? pl_in : b_to_l ? pl_in_b : pl_in_c;
         end else if (mv_l) stg_v_l <= 1'b0;
         else if (stg_v_l) stg_r_l <= stg_r_l | snp_l;
         // F (FP/CTF): whichever slot is the (single) FC.
         if (f_slot0 | b_to_f | c_to_f) begin
            stg_v_f   <= 1'b1;
            stg_rob_f <= f_slot0 ? rob_d_idx : b_to_f ? rob_d_idx2 : rob_d_idx3;
            stg_ps_f  <= f_slot0 ? ps_in : b_to_f ? ps_in_b : ps_in_c;
            stg_r_f   <= f_slot0 ? srdy_hit0 : b_to_f ? srdy_hit1 : srdy_hit2;
            stg_prd_f <= f_slot0 ? d_prd_g : b_to_f ? d2_prd_g : d3_prd_g;
            stg_pl_f  <= f_slot0 ? pl_in : b_to_f ? pl_in_b : pl_in_c;
         end else if (mv_f) stg_v_f <= 1'b0;
         else if (stg_v_f) stg_r_f <= stg_r_f | snp_f;
         // Flush LAST (rule I11: redirect never gates an enable; flush arm wins on order).
         if (redirect) begin stg_v_ia<=1'b0; stg_v_ib<=1'b0; stg_v_l<=1'b0; stg_v_f<=1'b0; end
      end
   end

   // ONE payload array across all three schedulers, indexed by a flat slot number with a
   // per-class offset -- each scheduler has its own entry-number space, and the offsets are
   // what stop them aliasing.
   reg [PLW-1:0] plmem_i [0:NI-1];
   reg [PLW-1:0] plmem_i2 [0:NI-1];
   reg [PLW-1:0] plmem_l [0:NL-1];
   reg [PLW-1:0] plmem_f [0:NF-1];
   // THE PAYLOADS ARE READ AT PICK AND REGISTERED WITH THE TAGS (plan item T1, step 3,
   // 2026-09-06). They used to be read in the execute cycle at last cycle's entry, so every
   // control derived from them -- the pending table's dispatch-cycle compare, the store
   // queue's data valid, the FPU's request -- began with a LUTRAM read: T1F2's census had
   // `i_ent_reg -> u_pend/pend_reg` (462 endpoints, 14 levels) and `-> u_sq/dv` at the top.
   // Now, like psmem's tags: one read per port at that port's own candidate (the LUTRAM
   // address is the scheduler's select, not the pick), and the port's load captures it. An
   // entry issues no earlier than the cycle after its dispatch, so a read never meets its
   // own write.
   wire [PLW-1:0] pl_cand  = plmem_l[rl_iss_ent];   // i_* is M-only now
   reg  [PLW-1:0] pl_q, pla_q, pla2_q, pla3_q, plf_q;
   always @(posedge clk) begin
      if (iss_ready & iq_iss_take) pl_q   <= pl_cand;
      if (ri_take)                 pla_q  <= plmem_i[ri_iss_ent];
      if (ri2_take)                pla2_q <= plmem_i2[ri2_iss_ent];
      if (ri3_take)                pla3_q <= {PLW{1'b0}};   // dead: the third ALU never issues
      if (rf_take)                 plf_q  <= plmem_f[rf_iss_ent];   // the FP/CTF port's payload
   end
   wire [PLW-1:0] pl_out   = pl_q;
   wire [PLW-1:0] pla_out  = pla_q;                         // the ALU port's payload
   wire [PLW-1:0] pla2_out = pla2_q;                        // the second ALU port's
   wire [PLW-1:0] pla3_out = pla3_q;                        // the third ALU port's
   wire [PLW-1:0] plf_out  = plf_q;                         // the F/CTF port's
   always @(posedge clk) begin
      // Co-timed with the scheduler fill (T+1), from the REGISTERED stage payload.
      if (mv_ia) plmem_i[ri_d_ent]  <= stg_pl_ia;
      if (mv_ib) plmem_i2[ri2_d_ent] <= stg_pl_ib;
      if (mv_l)  plmem_l[rl_d_ent]  <= stg_pl_l;
      if (mv_f)  plmem_f[rf_d_ent]  <= stg_pl_f;
   end

   wire [PCW-1:0]      q_pc, q_pred_npc, q_fault_tval;
   wire [31:0]         q_insn;
   wire [SQ_IB-1:0]    q_sq_tag;
   wire [LQ_IB-1:0]    q_lq_idx;
   wire                q_rvc, q_rd_v, q_mem_signed, q_is_mem, q_is_store, q_is_amo;
   wire [SEQW-1:0]     q_seq;
   wire [PDW-1:0]      q_pdet;
   wire [5:0]          q_rd, q_rs1;
   wire [RN_PBITS-1:0] q_prd;
   wire [2:0]          q_shard;
   wire [1:0]          q_mem_size;
   wire [63:0]         q_imm;
   wire [4:0]          q_amo_func;
   wire                q_is_branch, q_is_jump, q_is_jalr, q_is_mul, q_is_csr;
   wire [2:0]          q_csr_func;
   wire                q_is_serialize, q_is_fp, q_is_fencei, q_is_cbo, q_cbo_zero;
   wire                q_cbo_keep, q_illegal, q_fault;
   wire [3:0]          q_fault_cause;
   wire [5:0]          q_alu_op;
   wire                q_alu_w, q_alu_uw, q_op2_imm, q_res_link, q_mis_taken, q_mis_nt;
   wire [1:0]          q_op1_sel;
   wire [2:0]          q_br_func;
   wire                q_rs1_v, q_rs2_v, q_rs3_v, q_ord;
   assign {q_pc, q_insn, q_rvc, q_seq, q_pdet, q_pred_npc, q_rd, q_rd_v, q_prd, q_shard,
           q_rs1, q_imm, q_mem_size, q_mem_signed, q_is_mem, q_is_store, q_is_amo,
           q_amo_func, q_is_branch, q_is_jump, q_is_jalr, q_is_mul, q_is_csr, q_csr_func,
           q_is_serialize, q_is_fp, q_is_fencei, q_is_cbo, q_cbo_zero, q_cbo_keep,
           q_illegal, q_fault, q_fault_cause, q_fault_tval,
           q_alu_op, q_alu_w, q_alu_uw, q_op1_sel, q_op2_imm, q_res_link,
           q_br_func, q_mis_taken, q_mis_nt,
           q_rs1_v, q_rs2_v, q_rs3_v, q_ord, q_sq_tag, q_lq_idx} = pl_out;
   // the same unpack for the ALU port (unused fields fall away)
   wire [PCW-1:0]      qa_pc, qa_pred_npc, qa_fault_tval;
   wire [31:0]         qa_insn;
   wire [SQ_IB-1:0]    qa_sq_tag;
   wire [LQ_IB-1:0]    qa_lq_idx;
   wire                qa_rvc, qa_rd_v, qa_mem_signed, qa_is_mem, qa_is_store, qa_is_amo;
   wire [SEQW-1:0]     qa_seq;
   wire [PDW-1:0]      qa_pdet;
   wire [5:0]          qa_rd, qa_rs1;
   wire [RN_PBITS-1:0] qa_prd;
   wire [2:0]          qa_shard;
   wire [1:0]          qa_mem_size;
   wire [63:0]         qa_imm;
   wire [4:0]          qa_amo_func;
   wire                qa_is_branch, qa_is_jump, qa_is_jalr, qa_is_mul, qa_is_csr;
   wire [2:0]          qa_csr_func;
   wire                qa_is_serialize, qa_is_fp, qa_is_fencei, qa_is_cbo, qa_cbo_zero;
   wire                qa_cbo_keep, qa_illegal, qa_fault;
   wire [3:0]          qa_fault_cause;
   wire [5:0]          qa_alu_op;
   wire                qa_alu_w, qa_alu_uw, qa_op2_imm, qa_res_link, qa_mis_taken, qa_mis_nt;
   wire [1:0]          qa_op1_sel;
   wire [2:0]          qa_br_func;
   wire                qa_rs1_v, qa_rs2_v, qa_rs3_v, qa_ord;
   assign {qa_pc, qa_insn, qa_rvc, qa_seq, qa_pdet, qa_pred_npc, qa_rd, qa_rd_v, qa_prd, qa_shard,
           qa_rs1, qa_imm, qa_mem_size, qa_mem_signed, qa_is_mem, qa_is_store, qa_is_amo,
           qa_amo_func, qa_is_branch, qa_is_jump, qa_is_jalr, qa_is_mul, qa_is_csr, qa_csr_func,
           qa_is_serialize, qa_is_fp, qa_is_fencei, qa_is_cbo, qa_cbo_zero, qa_cbo_keep,
           qa_illegal, qa_fault, qa_fault_cause, qa_fault_tval,
           qa_alu_op, qa_alu_w, qa_alu_uw, qa_op1_sel, qa_op2_imm, qa_res_link,
           qa_br_func, qa_mis_taken, qa_mis_nt,
           qa_rs1_v, qa_rs2_v, qa_rs3_v, qa_ord, qa_sq_tag, qa_lq_idx} = pla_out;
   // ...and for the second ALU port
   wire [PCW-1:0]      qb_pc, qb_pred_npc, qb_fault_tval;
   wire [31:0]         qb_insn;
   wire [SQ_IB-1:0]    qb_sq_tag;
   wire [LQ_IB-1:0]    qb_lq_idx;
   wire                qb_rvc, qb_rd_v, qb_mem_signed, qb_is_mem, qb_is_store, qb_is_amo;
   wire [SEQW-1:0]     qb_seq;
   wire [PDW-1:0]      qb_pdet;
   wire [5:0]          qb_rd, qb_rs1;
   wire [RN_PBITS-1:0] qb_prd;
   wire [2:0]          qb_shard;
   wire [1:0]          qb_mem_size;
   wire [63:0]         qb_imm;
   wire [4:0]          qb_amo_func;
   wire                qb_is_branch, qb_is_jump, qb_is_jalr, qb_is_mul, qb_is_csr;
   wire [2:0]          qb_csr_func;
   wire                qb_is_serialize, qb_is_fp, qb_is_fencei, qb_is_cbo, qb_cbo_zero;
   wire                qb_cbo_keep, qb_illegal, qb_fault;
   wire [3:0]          qb_fault_cause;
   wire [5:0]          qb_alu_op;
   wire                qb_alu_w, qb_alu_uw, qb_op2_imm, qb_res_link, qb_mis_taken, qb_mis_nt;
   wire [1:0]          qb_op1_sel;
   wire [2:0]          qb_br_func;
   wire                qb_rs1_v, qb_rs2_v, qb_rs3_v, qb_ord;
   assign {qb_pc, qb_insn, qb_rvc, qb_seq, qb_pdet, qb_pred_npc, qb_rd, qb_rd_v, qb_prd, qb_shard,
           qb_rs1, qb_imm, qb_mem_size, qb_mem_signed, qb_is_mem, qb_is_store, qb_is_amo,
           qb_amo_func, qb_is_branch, qb_is_jump, qb_is_jalr, qb_is_mul, qb_is_csr, qb_csr_func,
           qb_is_serialize, qb_is_fp, qb_is_fencei, qb_is_cbo, qb_cbo_zero, qb_cbo_keep,
           qb_illegal, qb_fault, qb_fault_cause, qb_fault_tval,
           qb_alu_op, qb_alu_w, qb_alu_uw, qb_op1_sel, qb_op2_imm, qb_res_link,
           qb_br_func, qb_mis_taken, qb_mis_nt,
           qb_rs1_v, qb_rs2_v, qb_rs3_v, qb_ord, qb_sq_tag, qb_lq_idx} = pla2_out;
   // ...and for the third ALU port (Stage 3)
   wire [PCW-1:0]      qc_pc, qc_pred_npc, qc_fault_tval;
   wire [31:0]         qc_insn;
   wire [SQ_IB-1:0]    qc_sq_tag;
   wire [LQ_IB-1:0]    qc_lq_idx;
   wire                qc_rvc, qc_rd_v, qc_mem_signed, qc_is_mem, qc_is_store, qc_is_amo;
   wire [SEQW-1:0]     qc_seq;
   wire [PDW-1:0]      qc_pdet;
   wire [5:0]          qc_rd, qc_rs1;
   wire [RN_PBITS-1:0] qc_prd;
   wire [2:0]          qc_shard;
   wire [1:0]          qc_mem_size;
   wire [63:0]         qc_imm;
   wire [4:0]          qc_amo_func;
   wire                qc_is_branch, qc_is_jump, qc_is_jalr, qc_is_mul, qc_is_csr;
   wire [2:0]          qc_csr_func;
   wire                qc_is_serialize, qc_is_fp, qc_is_fencei, qc_is_cbo, qc_cbo_zero;
   wire                qc_cbo_keep, qc_illegal, qc_fault;
   wire [3:0]          qc_fault_cause;
   wire [5:0]          qc_alu_op;
   wire                qc_alu_w, qc_alu_uw, qc_op2_imm, qc_res_link, qc_mis_taken, qc_mis_nt;
   wire [1:0]          qc_op1_sel;
   wire [2:0]          qc_br_func;
   wire                qc_rs1_v, qc_rs2_v, qc_rs3_v, qc_ord;
   assign {qc_pc, qc_insn, qc_rvc, qc_seq, qc_pdet, qc_pred_npc, qc_rd, qc_rd_v, qc_prd, qc_shard,
           qc_rs1, qc_imm, qc_mem_size, qc_mem_signed, qc_is_mem, qc_is_store, qc_is_amo,
           qc_amo_func, qc_is_branch, qc_is_jump, qc_is_jalr, qc_is_mul, qc_is_csr, qc_csr_func,
           qc_is_serialize, qc_is_fp, qc_is_fencei, qc_is_cbo, qc_cbo_zero, qc_cbo_keep,
           qc_illegal, qc_fault, qc_fault_cause, qc_fault_tval,
           qc_alu_op, qc_alu_w, qc_alu_uw, qc_op1_sel, qc_op2_imm, qc_res_link,
           qc_br_func, qc_mis_taken, qc_mis_nt,
           qc_rs1_v, qc_rs2_v, qc_rs3_v, qc_ord, qc_sq_tag, qc_lq_idx} = pla3_out;
   // ...and for the independent F/CTF port (CTF-on-FP). Full unpack, same concatenation; the
   // F stage uses insn/rd/rd_v/prd/shard today, the CTF exec will use the branch fields.
   wire [PCW-1:0]      qf_pc, qf_pred_npc, qf_fault_tval;
   wire [31:0]         qf_insn;
   wire [SQ_IB-1:0]    qf_sq_tag;
   wire [LQ_IB-1:0]    qf_lq_idx;
   wire                qf_rvc, qf_rd_v, qf_mem_signed, qf_is_mem, qf_is_store, qf_is_amo;
   wire [SEQW-1:0]     qf_seq;
   wire [PDW-1:0]      qf_pdet;
   wire [5:0]          qf_rd, qf_rs1;
   wire [RN_PBITS-1:0] qf_prd;
   wire [2:0]          qf_shard;
   wire [1:0]          qf_mem_size;
   wire [63:0]         qf_imm;
   wire [4:0]          qf_amo_func;
   wire                qf_is_branch, qf_is_jump, qf_is_jalr, qf_is_mul, qf_is_csr;
   wire [2:0]          qf_csr_func;
   wire                qf_is_serialize, qf_is_fp, qf_is_fencei, qf_is_cbo, qf_cbo_zero;
   wire                qf_cbo_keep, qf_illegal, qf_fault;
   wire [3:0]          qf_fault_cause;
   wire [5:0]          qf_alu_op;
   wire                qf_alu_w, qf_alu_uw, qf_op2_imm, qf_res_link, qf_mis_taken, qf_mis_nt;
   wire [1:0]          qf_op1_sel;
   wire [2:0]          qf_br_func;
   wire                qf_rs1_v, qf_rs2_v, qf_rs3_v, qf_ord;
   assign {qf_pc, qf_insn, qf_rvc, qf_seq, qf_pdet, qf_pred_npc, qf_rd, qf_rd_v, qf_prd, qf_shard,
           qf_rs1, qf_imm, qf_mem_size, qf_mem_signed, qf_is_mem, qf_is_store, qf_is_amo,
           qf_amo_func, qf_is_branch, qf_is_jump, qf_is_jalr, qf_is_mul, qf_is_csr, qf_csr_func,
           qf_is_serialize, qf_is_fp, qf_is_fencei, qf_is_cbo, qf_cbo_zero, qf_cbo_keep,
           qf_illegal, qf_fault, qf_fault_cause, qf_fault_tval,
           qf_alu_op, qf_alu_w, qf_alu_uw, qf_op1_sel, qf_op2_imm, qf_res_link,
           qf_br_func, qf_mis_taken, qf_mis_nt,
           qf_rs1_v, qf_rs2_v, qf_rs3_v, qf_ord, qf_sq_tag, qf_lq_idx} = plf_out;

   // The pack/unpack check that stood here compared the payload against the m_* registers
   // while BOTH were written from d_*. The payload is now the only source for m_*, so the
   // comparison is tautological and gone. The cosim is what checks it instead: every
   // retired instruction's pc, instruction word and value, against simmerv.

   // ------------------------------------------------------------------ STORE BUFFER
   // docs/Area-Efficient-Scalar-OoO.md 11. A store issues on its ADDRESS alone and commits
   // when its data has arrived and it is the oldest -- which is what stops it sitting at the
   // head of u_iq_l for the ~21 cycles an fadds takes, with the next iteration's loads
   // queued behind work they do not depend on (spec 15, Camera).
   //
   // FLUSH IS WHOLESALE, and that is correct rather than merely convenient: `redirect` is
   // gated by head_block, so a redirect fires only when the redirecting instruction is at
   // the ROB head -- every older instruction has therefore already retired, and a store
   // retires only when this buffer has written it. Everything still live is younger.
   wire                sq_d_ready, sq_c_v, sq_c_unc, sq_ld_older;
   wire                sq_ld_block;      // instrumentation: candidate held by an alias
   wire [LQ_N-1:0]     sq_l_older;       // per load, registered: an older store is live
   wire [LQ_N-1:0]     sq_l_block_live;  // the live alias block, the oracle for the registered copy the queue reads
   wire [SQ_IB:0]      sq_occ;
   wire [LQ_N-1:0]     sq_l_block_unk_q;   // counters: the candidate's block is an UNKNOWN older address
   wire                sq_av_any;
   wire [SQ_IB-1:0]    sq_d_idx;
   wire [SQ_TB-1:0]    sq_d_tag;
   wire [ROB_IDXB-1:0] sq_c_rob;
   wire [55:0]         sq_c_addr;
   wire [63:0]         sq_c_data;
   wire [1:0]          sq_c_size;
   wire                lsu_pt_done, lsu_pt_ack, lsu_pt_is_store, lsu_pt_ld_done, lsu_pt_ld_kill, lsu_xo_v, lsu_xo_unc, lsu_xo_mem;
   wire                lsu_pt_fast_done;  wire [LQ_IB-1:0] lsu_pt_rtag;   // the fast path's landing, by tag
   wire [55:0]         lsu_xo_pa;
   wire st_a = rn_valid   & d_st_nb;
   wire st_b = rn_valid_b & d2_st_nb;
   wire st_c = rn_valid_c & d3_st_nb;                 // slot C store (swizzle)
   wire d_st_alloc = st_a | st_b | st_c;              // <=1 LS/cycle -> at most one true
   // The head entry may go to memory only once it IS the ROB head: that is the point at
   // which no older instruction can still trap and no redirect can still squash it.
   // A committed store drains whenever the port is free: it was released by the ROB's
   // irrevocable pointer (sq_k_take below), not by reaching the head, so the head no
   // longer sits on every store for the ~6 cycles the cache takes.
   wire sq_go     = sq_c_v;
   wire                sq_kc_v;   wire [ROB_IDXB-1:0] sq_kc_rob;  wire [55:0] sq_kc_addr;
   wire [63:0]         sq_kc_data; wire [1:0] sq_kc_size;   // the cosim's store-data check
   wire [ROB_IDXB-1:0] rob_irr_idx;  wire rob_irr_v;
   wire sq_k_take = sq_kc_v & rob_irr_v & (sq_kc_rob == rob_irr_idx);
   // The queue pops at the HANDOFF to the LSU (pt_ack for a store), which registers the data;
   // the LSU holds the store until the D$ takes it and nothing can pass it there.
   wire sq_c_take = lsu_pt_ack & pt_store;

   ooo2_sq #(.NENT(SQ_N), .IDXB(SQ_IB), .PAW(56), .PBITS(RN_PBITS),
             .ROBB(ROB_IDXB), .NWB(NWB_C), .LQN(LQ_N), .LQIB(LQ_IB)) u_sq
     (.clk(clk), .reset(reset),
      .d_alloc(d_st_alloc), .d_rob(st_c ? rob_d_idx3 : st_b ? rob_d_idx2 : rob_d_idx), .d_dpreg(st_c ? rn_prs2_c : st_b ? rn_prs2_b : rn_prs2),
      .d_ready(sq_d_ready), .d_idx(sq_d_idx), .d_tag(sq_d_tag), .av_any(sq_av_any),
      .a_v(m_sq_fill), .a_idx(m_sq_tag), .a_addr(lsu_xo_pa), .a_size(m_mem_size),
      .a_unc(lsu_xo_unc), .a_data_v(m_rs2_rdy), .a_data(m_st_data),
      .wb_v(wkv), .wb_preg(wkp), .wb_data({wb_ie3, wb_ie2, wb_fe, wb_ld, wb_ie}),
      .c_v(sq_c_v), .c_rob(sq_c_rob), .c_addr(sq_c_addr), .c_data(sq_c_data),
      .c_size(sq_c_size), .c_unc(sq_c_unc), .c_take(sq_c_take),
      .kc_v(sq_kc_v), .kc_rob(sq_kc_rob), .kc_addr(sq_kc_addr), .kc_data(sq_kc_data), .kc_size(sq_kc_size), .k_take(sq_k_take),
      // THE ALIAS TEST LIVES HERE, not at issue: ooo2_lq exports its entries, ooo2_sq keeps
      // a conflict matrix updated wherever an address arrives, and issue reads a flop.
      .l_pa(lq_e_pa), .l_size(lq_e_size), .l_tag(lq_e_tag), .l_av(lq_e_av),
      .l_fill(m_lq_fill), .l_fill_ix(m_lq_idx),
      .l_fill_pa(lsu_xo_pa), .l_fill_size(m_mem_size),
      .l_block(sq_l_block_live), .l_block_q(lq_e_block), .l_older(sq_l_older),
      .ld_tag(lq_q_tag), .ld_older(sq_ld_older),
      .l_block_unk_q(sq_l_block_unk_q), .occupancy(sq_occ), .flush(redirect));

   // Instrumentation for "did a load actually get reordered past a store". A load STARTS
   // its access only when ~ld_block, so a start with an older store still live is exactly
   // one reordering that the old in-order machine could not have done.
   wire sq_ld_reorder = lq_x_take & sq_ld_older;

   // ------------------------------------------------------------------- LOAD QUEUE
   wire                lq_d_ready, lq_x_v, lq_x_signed, lq_x_fp, lq_x_unc, lq_x_head, lq_l_rd_v, lq_b_ok;
   wire [LQ_IB-1:0]    lq_d_idx, lq_x_idx;
   wire [55:0]         lq_x_pa;
   wire [1:0]          lq_x_size;
   wire [LQ_N*56-1:0]  lq_e_pa;
   wire [LQ_N*2-1:0]   lq_e_size;
   wire [LQ_N*SQ_TB-1:0] lq_e_tag;
   wire [LQ_N-1:0]     lq_e_av, lq_e_block;
   wire [SQ_TB-1:0]    lq_q_tag;
   wire [RN_PBITS-1:0] lq_l_prd;
   wire [5:0]          lq_l_rd;
   wire [ROB_IDXB-1:0] lq_l_rob;
   wire [55:0]         lq_l_pa;      // the landing load's own PA (cosim memory effect)
   wire [LQ_IB:0]      lq_occ;  wire lq_av_any;
   wire                lq_x_devwait;        // counters: a device load waiting for the head
   wire d_ld_nb    = d_is_mem & ~d_is_store & ~d_is_amo & ~d_is_cbo;  // plain load, rule C1
   wire ld_a = rn_valid   & d_ld_nb;
   wire ld_b = rn_valid_b & d2_ld_nb;
   wire ld_c = rn_valid_c & d3_ld_nb;                 // slot C load (swizzle)
   wire d_ld_alloc = ld_a | ld_b | ld_c;              // <=1 LS/cycle -> at most one true
   // a load behind a store in the same cycle captures the tag AFTER that store's
   wire [SQ_TB-1:0] ld_sqtag = sq_d_tag;   // <=1 LS/cycle: a store and a load never co-dispatch

   ooo2_lq #(.NENT(LQ_N), .IDXB(LQ_IB), .PAW(56), .PBITS(RN_PBITS),
             .ROBB(ROB_IDXB), .SQIB(SQ_TB), .LRAM_BASE(LBASE), .LRAM_LG2(LRAM_LG2)) u_lq
     (.clk(clk), .reset(reset),
      .d_alloc(d_ld_alloc), .d_rob(ld_c ? rob_d_idx3 : ld_b ? rob_d_idx2 : rob_d_idx), .d_prd(ld_c ? d3_prd_g : ld_b ? d2_prd_g : d_prd_g),
      .d_rd(ld_c ? d3_rd : ld_b ? d2_rd : d_rd), .d_rd_v(ld_c ? d3_rd_v : ld_b ? d2_rd_v : d_rd_v), .d_sqtag(ld_sqtag),
      .d_ready(lq_d_ready), .d_idx(lq_d_idx),
      .a_v(m_lq_fill), .a_sent(lsu_xo_early), .a_idx(m_lq_idx),
      .a_pa(lsu_xo_pa), .a_size(m_mem_size),
      .a_signed(m_mem_signed), .a_fp(m_is_fp), .a_unc(lsu_xo_unc), .a_mem(lsu_xo_mem),
      .e_pa(lq_e_pa), .e_size(lq_e_size), .e_tag(lq_e_tag), .e_av(lq_e_av),
      .e_block(lq_e_block), .x_block(sq_ld_block), .q_tag(lq_q_tag),
      .b_idx(m_lq_idx), .b_ok(lq_b_ok),
      .x_v(lq_x_v), .x_idx(lq_x_idx), .x_pa(lq_x_pa), .x_size(lq_x_size),
      .x_signed(lq_x_signed), .x_fp(lq_x_fp), .x_unc(lq_x_unc), .x_head(lq_x_head), .x_take(lq_x_take),
      .l_v(ld_land), .l_idx(ld_land_idx),
      .l_prd(lq_l_prd), .l_rd(lq_l_rd), .l_rd_v(lq_l_rd_v), .l_rob(lq_l_rob), .l_pa(lq_l_pa),
      .x_devwait(lq_x_devwait), .occupancy(lq_occ), .av_any(lq_av_any), .rob_head(rob_head_idx), .flush(redirect));

   // ------------------------------------------------- COLLAPSING FILL AND ACCESS
   // The queue costs a load two cycles -- one to register the address, one to select the
   // candidate and run the alias test against it. The SECOND is what pays for the test, so
   // a load with no store OLDER than it still live should not pay it: the access issues in
   // the same M pass that fills the entry. Fill, M's early release and the landing path are
   // ALL unchanged -- only the start moves.
   //
   // That last point is the whole design. An earlier attempt dropped the entry and let M
   // keep the load through its access instead, which also skipped the fill cycle -- and it
   // was a REGRESSION (ldbench 6.00 -> 7.00 cyc/load, Camera unmoved). The queue's two
   // cycles are not overhead: releasing M lets the following non-memory instructions execute
   // while the data is in flight, and that is worth more than the latency it costs.
   //
   // TIMING-SAFE BY CONSTRUCTION, which is the only reason this may gate a start at all.
   // ooo2_sq's ld_older is v[]/head/ld_tag alone -- no address, nothing off the translate
   // path -- and lq_b_ok is a 2-bit index compare on flops. Both settle at the top of the
   // cycle, in parallel with the dTLB lookup they qualify. ld_BLOCK, the address compare, is
   // what must never come back here, and does not. The ADDRESS still reaches mem_raddr from
   // t_paddr in this cycle, which is the path ec6ad3e closed at 166 MHz.
   //
   // ld_older answers about OUR load only while the queue's query port is asking about it:
   // q_tag is sqt[acc], so `b_idx == acc` -- inside lq_b_ok -- is what makes the read sound.
   // ...and since 2026-09-07 the answer M reads is the store queue's REGISTERED per-load
   // copy at M's own load index (a register), not the live query: see ooo2_sq l_older. The
   // live one is the oracle, and the copy may only ever be the more conservative.
   wire lq_b_early = m_ld_nb & lq_b_ok & ~sq_l_older[m_lq_idx];
   // Checked only while M holds the load: m_lq_idx is a register that keeps the LAST load's
   // index, and a load dispatched into that entry a cycle ago is the queue's candidate
   // (lq_b_ok) before its copy has caught up -- one cycle, and M is not looking.
   always @(posedge clk)
      if (!reset && m_ld_nb && lq_b_ok && sq_ld_older && !sq_l_older[m_lq_idx])
         $fatal(1, "ooo2_core: the registered older-store answer (0) is less conservative than the live one (1) for load %0d", m_lq_idx);

   // ONE pre-translated port, two users. The committing store wins: it is at the ROB head,
   // so it is unconditionally older than any queued load, and it frees the port immediately.
   // A load waiting a cycle for it costs nothing that the store's own drain did not already.
   wire pt_v      = sq_go | lq_x_v;
   wire pt_store  = sq_go;
   wire lq_x_take = lq_x_v & ~sq_go & lsu_pt_ack;
   // Two landing paths (C4a). The FAST one names its entry with the tag the response carried;
   // the SLOW one (a straddle, a device, an uncached load) still parks the LSU's FSM, so the
   // one register below identifies it -- there can only be the one.
   wire ld_land_fast = lsu_pt_fast_done;
   wire ld_land_slow = lsu_pt_ld_done & ~lsu_pt_ld_kill;   // ...unless it was a wrong-path speculative load (squashed)
   wire ld_land      = ld_land_fast | ld_land_slow;
   // The landing INDEX selects on the raw fast response (dmem_rvalid_c: the D$'s registered
   // rd_valid and tag class), not on ld_land_fast: the load queue's arrays are read at this
   // index, and putting the per-tag o_v/o_kill lookups in front of that read was the worst
   // core family of the first C4a build (rd_resp_tag -> e_r, 20 levels; IW=3 -0.070). It is
   // exact: a fast response with a live tag makes the slow path yield (slow_rv), so a slow
   // landing never coincides with one, and a killed fast response lands nothing (asserted).
   wire [LQ_IB-1:0] ld_land_idx = dmem_rvalid_c ? dmem_rtag_resp : ld_inflight_idx;
   always @(posedge clk) if (!reset) begin
      if (ld_land_fast && ld_land_slow) $fatal(1, "ooo2_core: a load landed on both the fast and the slow path");
      if (ld_land_slow && dmem_rvalid_c) $fatal(1, "ooo2_core: a slow landing under a fast response (the index would be wrong)");
      if (ld_land_fast && (lsu_pt_rtag != dmem_rtag_resp)) $fatal(1, "ooo2_core: the fast landing's tag is not the response's");
   end
   // The registered alias block the queue's candidate select reads may only ever be the
   // MORE conservative: a load that starts (x_v, on the copy) is never one the live block holds.
   always @(posedge clk)
      if (!reset && lq_x_v && sq_l_block_live[lq_x_idx])
         $fatal(1, "ooo2_core: load %0d starts on the registered block copy while the live block holds it", lq_x_idx);
   // The tag of the access in flight. One at a time today, so a single register; when loads
   // are pipelined this becomes the D$'s rd_tag and the queue interface does not change.
   // Two ways an access leaves for memory now, and they name their entry differently: the
   // candidate path by lq_x_idx, the early path by the index M is holding. They are mutually
   // exclusive -- an early start needs ~av[acc], a candidate start needs av[acc] -- which
   // ooo2_lq asserts rather than assumes.
   wire lq_b_take = lsu_xo_early;          // the LSU says whether the early start happened
   reg  [LQ_IB-1:0] ld_inflight_idx;
   initial ld_inflight_idx = {LQ_IB{1'b0}};
   always @(posedge clk) if      (lq_x_take) ld_inflight_idx <= lq_x_idx;
                         else if (lq_b_take) ld_inflight_idx <= m_lq_idx;

   // NW=4: the store's ROB slot completes when the BUFFER writes it, not when it executes.
   // Routed through the existing completion mechanism (rule C2), which is parameterised on
   // exactly this -- not a private path to the ROB.
   ooo2_rob #(.DEPTH(ROB_DEPTH), .IDXB(ROB_IDXB), .PBITS(RN_PBITS), .IW(IW), .NW(8)) u_rob
     (.clk(clk), .reset(reset),
      // prd is ZERO when nothing is written: rename drives r_prd unconditionally, and
      // `d_prd != 0` is what replaces the stored rd_v bit.
      .d_valid(rn_valid), .d_rd(d_rd),
      .d_prd(d_rd_v ? rn_prd : {RN_PBITS{1'b0}}), .d_noret(d_is_irqop),
      .d_ready(rob_ready), .d_idx(rob_d_idx),
      .d_valid2(rn_valid_b), .d_rd2(d2_rd), .d_prd2(d2_prd_g), .d_noret2(1'b0), .d_ready2(rob_ready2), .d_idx2(rob_d_idx2),
      // third alloc port: dead at IW<3 (no third dispatched uop yet); the dispatch-widening
      // step connects d_valid3 to the third rename slot. d_ready3/d_idx3/c3 outputs are
      // gated 0 inside the ROB at IW<3, so leaving them open is harmless.
      .d_valid3(rn_valid_c), .d_rd3(d3_rd), .d_prd3(d3_prd_g), .d_noret3(1'b0), .d_ready3(rob_ready3), .d_idx3(rob_d_idx3),
      .w_v({md_wb, cf_land, iss_alu3, iss_alu2, sq_k_take, fp_land, iss_alu, rob_w_valid}),
      .w_ix({md_rob, cf_land_rob, a3_rob, a2_rob, sq_kc_rob, ft_rob, a_rob, rob_w_idx}),
      .c_kill((m_valid & m_done & m_trap) | sy_trap),
      .c2_kill(m_valid & (m_rob_idx == rob_head2_idx)),   // M's op retires only from the head
      .c_valid(rob_c_valid), .c_rd(rob_c_rd), .c_rd_v(rob_c_rd_v),
      .c_prd(rob_c_prd), .c_noret(rob_c_noret),
      .c2_valid(rob_c2_valid), .c2_rd(rob_c2_rd), .c2_rd_v(rob_c2_rd_v), .c2_prd(rob_c2_prd), .c2_noret(rob_c2_noret),
      .c3_kill(1'b0),
      .c3_valid(rob_c3_valid), .c3_rd(rob_c3_rd), .c3_rd_v(rob_c3_rd_v), .c3_prd(rob_c3_prd), .c3_noret(rob_c3_noret),
      .flush(redirect), .empty(rob_empty), .head_idx(rob_head_idx),
      .irr_idx(rob_irr_idx), .irr_v(rob_irr_v));

   // The M-equivalence assertion that guarded the previous two commits is GONE, deliberately
   // and by construction: it said the ROB's commit equals what M would have done in the same
   // cycle, which was true only because M blocked. It no longer does. Its replacement is the
   // ROB's own always-on set (double completion, completion of a dead slot, committing an
   // invalid head, occupancy overflow) plus the scoreboard's, above.
   //
   // `rob_ready` is now real back-pressure rather than an assertion: with M releasing loads
   // early, the ROB genuinely fills behind a head that is waiting for its data.

   // M-stage registers (declared here: the bypass reads them)
   reg              m_valid, m_rvc, m_rd_v;
   reg  [PCW-1:0]   m_pc, m_pred_npc, m_fault_tval, m_target, m_taken_tgt;
   reg  [31:0]      m_insn;
   reg  [SEQW-1:0]  m_seq;
   reg  [PDW-1:0]   m_pdet;   // this op's predict details, carried F->X->M
   reg  [5:0]       m_rd, m_rs1;
   reg  [RN_PBITS-1:0] m_prd;           // rename result, carried X->M
   reg  [2:0]       m_shard;
   reg  [63:0]      m_imm, m_result, m_addr, m_st_data, m_rs1_val, m_rs3_val;
   reg  [1:0]       m_mem_size;
   reg              m_mem_signed, m_is_mem, m_is_store, m_is_amo;
   reg  [4:0]       m_amo_func;
   reg              m_is_branch, m_is_jump, m_is_jalr, m_redirect, m_taken;
   reg              m_is_mul, m_is_csr, m_is_serialize, m_is_fp, m_is_fencei;
   reg  [2:0]       m_csr_func;
   reg              m_is_cbo, m_cbo_zero, m_cbo_keep;
   reg              m_illegal, m_fault;
   reg  [3:0]       m_fault_cause;
   // Store-buffer slot, and whether the issue-cycle PRF read of rs2 was valid. A plain
   // store may now issue with rs2 still pending, so m_st_data is meaningful only when
   // m_rs2_rdy -- otherwise ooo2_sq's snoop supplies the value instead.
   // ONE field serves both roles, because a buffered store's own slot IS the tail it
   // captured at dispatch: for a store it names the entry to fill. (A load's store-seqno
   // goes straight into ooo2_lq at dispatch and is one bit wider -- SQ_TB.)
   reg  [SQ_IB-1:0] m_sq_tag;
   reg  [LQ_IB-1:0] m_lq_idx;
   reg              m_rs2_rdy;
   initial begin m_valid = 1'b0; end

   // the M-stage writeback value, and the bypass source (which is NOT the same thing --
   // see the writeback comment: a CSR result is never bypassable)
   wire [63:0] m_wb_val, m_byp_val;
   wire [63:0] wb_ie, wb_ld, wb_fe;   // per-shard write data

   // one bypass level: M -> X. An instruction two ahead has already landed in the RF.
   //
   // AN OP THAT WRITES LATE MUST NOT BYPASS. A non-blocking load and an FP op both leave M
   // with no result in hand -- m_unit_res_q is latched at DISPATCH, before the unit has
   // produced anything -- and both write the PRF from their scoreboard slot instead. Their
   // consumers are covered by the pending-tag interlock, which holds X until the value is
   // in the PRF, so the bypass is not merely wrong here, it is unnecessary.
   //
   // Not theoretical, and the FPU is what exposed it. m_done is forced low on ld_land and
   // fp_land, so an FP op can still be sitting in M for cycles AFTER its result has landed
   // and fb_busy has cleared -- interlock off, m_rd still asserted. An fmin's consumer read
   // the FP result bus as it stood before the op ever issued.
   // THE M->X BYPASS IS GONE. It existed because operands were read at DISPATCH, one stage
   // before the producer's result reached the register file. Operands are now read at
   // ISSUE, and the scheduler will not select an entry until its sources are ready, so the
   // only collision left is the exact-cycle one: an entry woken by a writeback issues in
   // that same cycle, and the PRF read returns the pre-edge value. That is what `fwd` below
   // handles -- the datapath twin of the scheduler's wakeup, matching on the same physical
   // register numbers and the same three write ports.
   // THE EVERY-CYCLE SHADOW COMPARISON IS GONE, and deliberately.
   //
   // It asserted that a renamed read equals the architectural register file, which held only
   // because -- in ooo2_rename's own words -- "issue and commit remain IN ORDER at this
   // milestone". This commit is what ends that. Once a result lands in the PRF at COMPLETION
   // while the architectural file is written at COMMIT, the two differ for every instruction
   // in that window, which is most of them; and rv_regfile, having no rename, cannot model
   // out-of-order writeback at all.
   //
   // It earned its keep: five defects during rename bring-up, plus two in this change (a
   // pending source read before its load returned, and the load-format controls being taken
   // live from a stage that had moved on). What replaces it is strictly stronger and already
   // running -- the cosim compares every retired instruction's value against simmerv, in
   // program order, for billions of retirements. rv_regfile itself stays, written in commit
   // order, because tb_ooo2_riscv traces it.

   // rn_stall IS in the stall path now (see d_hold). It used to be a $fatal, on the grounds
   // that only ~2 instructions are ever in flight so no shard can run dry -- an argument that
   // expired when M stopped blocking and the ROB started filling behind a waiting load. It
   // can fire at ROB_DEPTH=32: the IE shard holds 32 free registers beyond the architectural
   // set, and LOWAT stops fetch before they run out. It is a stall, handled here, not a fault.

   // OPERANDS NOW COME FROM THE SHARDED PRF.  The bypass is retained rather than leaning
   // on ooo2_prf's write-through: both deliver the same value in the M->X case, and keeping
   // the existing mux means this commit changes the operand SOURCE without also changing
   // the operand TIMING PATH.  One variable at a time.
   // ooo2_prf's write-through is OFF (WRTHRU=0), which is only safe while every read that
   // collides with the writeback is bypassed.  That is a property of IN-ORDER issue, not a
   // law -- so check it rather than remember it.  When out-of-order issue lands and this
   // fires, the fix is WRTHRU=1, not a patch here.
   // The WRTHRU check that lived here asserted a property of DISPATCH-time reads (every
   // read colliding with the writeback is bypassed). Reads happen at issue now and the
   // collision is covered by `fwd`, so the check is replaced by one on the new mechanism:
   // an issuing entry whose source is written this cycle must take the forwarded value.
   always @(posedge clk) if (!reset & iq_iss_v & (WRTHRU_OFF == 0)) begin
      // placeholder: WRTHRU_OFF is 1, so this never fires. Kept as the anchor for the
      // read-during-write property so it is stated somewhere rather than remembered.
      if (1'b0) $fatal(1, "unreachable");
   end

   localparam WRTHRU_OFF = 1;
   // Writeback -> issue forward. Matches the SAME three ports the scheduler wakes on, so
   // readiness and data agree by construction: if wb_hit() said ready, fwd() has the value.
   // NO OPERAND FORWARDING, and none can be needed. Select has its own stage, so a
   // consumer reads the register file two cycles after its producer was selected while the
   // producer wrote it at the end of the cycle in between -- the value is always already
   // there. This is what the extra stage buys back: the forward mux, its self-forwarding
   // loop, and the whole question of which writebacks are forwardable all disappear.
   // THE ALU'S WRITEBACK IS REGISTERED (2026-09-05). Read -> ALU -> PRF write in one cycle
   // was the design's critical path (13 levels, 82% route: issue register to the int-exec
   // LUTRAM's data pin, +0.001 ns on Q, -0.034 on S), and a second ALU on it is hopeless.
   // The value now lands in alu_q at the end of the issue cycle and is written a cycle
   // later. Everything ISSUE-timed stays at issue -- the wake, the pending clear, the store
   // queue's data snoop, the ROB's done, the cosim capture -- so no consumer waits longer;
   // the one cycle in which a consumer could read the register before the write lands is
   // covered by this forward from the writeback register, a tag compare and a 2:1 mux. A
   // squashed op's write still lands: it goes to a register rename rolled back and nothing
   // can have re-allocated before the edge after the flush, and its pending bit was
   // cleared at issue, as before.
   reg                 alu_q_v;
   reg  [RN_PBITS-1:0] alu_q_prd;
   reg  [63:0]         alu_q_val;
   initial begin alu_q_v = 1'b0; alu_q_prd = {RN_PBITS{1'b0}}; alu_q_val = 64'd0; end
   // the second ALU's writeback register (10d-ii); its issue port is wired below
   reg                 alu2_q_v;
   reg  [RN_PBITS-1:0] alu2_q_prd;
   reg  [63:0]         alu2_q_val;
   initial begin alu2_q_v = 1'b0; alu2_q_prd = {RN_PBITS{1'b0}}; alu2_q_val = 64'd0; end
   // the third ALU's writeback register (Stage 3); its issue port is wired below
   reg                 alu3_q_v;
   reg  [RN_PBITS-1:0] alu3_q_prd;
   reg  [63:0]         alu3_q_val;
   initial begin alu3_q_v = 1'b0; alu3_q_prd = {RN_PBITS{1'b0}}; alu3_q_val = 64'd0; end

   // 3-producer forward: a source can be waiting on any of the three ALUs' writeback registers.
   // A physical register has one writer, so at most one of {fwd,fwdb,fwdc} is true -- the mux
   // order is immaterial.
   wire fwd1 = alu_q_v & (i_ps1 == alu_q_prd), fwd1b = alu2_q_v & (i_ps1 == alu2_q_prd), fwd1c = alu3_q_v & (i_ps1 == alu3_q_prd);
   wire fwd2 = alu_q_v & (i_ps2 == alu_q_prd), fwd2b = alu2_q_v & (i_ps2 == alu2_q_prd), fwd2c = alu3_q_v & (i_ps2 == alu3_q_prd);
   wire fwd3 = alu_q_v & (i_ps3 == alu_q_prd), fwd3b = alu2_q_v & (i_ps3 == alu2_q_prd), fwd3c = alu3_q_v & (i_ps3 == alu3_q_prd);
   wire [63:0] x_rs1 = fwd1 ? alu_q_val : fwd1b ? alu2_q_val : fwd1c ? alu3_q_val : prf_rs1;
   wire [63:0] x_rs2 = fwd2 ? alu_q_val : fwd2b ? alu2_q_val : fwd2c ? alu3_q_val : prf_rs2;
   wire [63:0] x_rs3 = fwd3 ? alu_q_val : fwd3b ? alu2_q_val : fwd3c ? alu3_q_val : prf_rs3;
   // The F/CTF port's operands, forwarded from the ALUs exactly like M's x_rs* (an FP op with
   // an integer source, or a CTF op, may consume an ALU result produced the same cycle).
   wire fwdf1 = alu_q_v & (j_ps1 == alu_q_prd), fwdf1b = alu2_q_v & (j_ps1 == alu2_q_prd), fwdf1c = alu3_q_v & (j_ps1 == alu3_q_prd);
   wire fwdf2 = alu_q_v & (j_ps2 == alu_q_prd), fwdf2b = alu2_q_v & (j_ps2 == alu2_q_prd), fwdf2c = alu3_q_v & (j_ps2 == alu3_q_prd);
   wire fwdf3 = alu_q_v & (j_ps3 == alu_q_prd), fwdf3b = alu2_q_v & (j_ps3 == alu2_q_prd), fwdf3c = alu3_q_v & (j_ps3 == alu3_q_prd);
   wire [63:0] xf_rs1 = fwdf1 ? alu_q_val : fwdf1b ? alu2_q_val : fwdf1c ? alu3_q_val : prf_f1;
   wire [63:0] xf_rs2 = fwdf2 ? alu_q_val : fwdf2b ? alu2_q_val : fwdf2c ? alu3_q_val : prf_f2;
   wire [63:0] xf_rs3 = fwdf3 ? alu_q_val : fwdf3b ? alu2_q_val : fwdf3c ? alu3_q_val : prf_f3;
   // the ALU port's operands, with the same forward
   wire fwd_a1 = alu_q_v & (a_ps1 == alu_q_prd), fwd_a1b = alu2_q_v & (a_ps1 == alu2_q_prd), fwd_a1c = alu3_q_v & (a_ps1 == alu3_q_prd);
   wire fwd_a2 = alu_q_v & (a_ps2 == alu_q_prd), fwd_a2b = alu2_q_v & (a_ps2 == alu2_q_prd), fwd_a2c = alu3_q_v & (a_ps2 == alu3_q_prd);
   wire [63:0] xa_rs1 = fwd_a1 ? alu_q_val : fwd_a1b ? alu2_q_val : fwd_a1c ? alu3_q_val : prf_a1;
   wire [63:0] xa_rs2 = fwd_a2 ? alu_q_val : fwd_a2b ? alu2_q_val : fwd_a2c ? alu3_q_val : prf_a2;
   wire [63:0] xa_result;
   // the second ALU port's operands and unit (10d-ii)
   wire fwd_b1 = alu_q_v & (a2_ps1 == alu_q_prd), fwd_b1b = alu2_q_v & (a2_ps1 == alu2_q_prd), fwd_b1c = alu3_q_v & (a2_ps1 == alu3_q_prd);
   wire fwd_b2 = alu_q_v & (a2_ps2 == alu_q_prd), fwd_b2b = alu2_q_v & (a2_ps2 == alu2_q_prd), fwd_b2c = alu3_q_v & (a2_ps2 == alu3_q_prd);
   wire [63:0] xb_rs1 = fwd_b1 ? alu_q_val : fwd_b1b ? alu2_q_val : fwd_b1c ? alu3_q_val : prf_a21;
   wire [63:0] xb_rs2 = fwd_b2 ? alu_q_val : fwd_b2b ? alu2_q_val : fwd_b2c ? alu3_q_val : prf_a22;
   wire [63:0] xb_result;
   ooo2_exec u_xb
     (.alu_op(qb_alu_op), .alu_w(qb_alu_w), .alu_uw(qb_alu_uw), .op1_sel(qb_op1_sel),
      .op2_imm(qb_op2_imm), .res_link(qb_res_link), .is_rvc(qb_rvc),
      .is_branch(1'b0), .is_jump(1'b0), .is_jalr(1'b0), .br_func(3'd0),
      .rs1_val(xb_rs1), .rs2_val(xb_rs2), .imm(qb_imm), .pc(qb_pc),
      .pred_npc(qb_pred_npc), .mis_taken(1'b0), .mis_nt(1'b0),
      .result(xb_result), .addr(), .redirect(), .target(), .taken(), .taken_tgt());
   // the third ALU port's operands and unit (Stage 3)
   wire fwd_c1 = alu_q_v & (a3_ps1 == alu_q_prd), fwd_c1b = alu2_q_v & (a3_ps1 == alu2_q_prd), fwd_c1c = alu3_q_v & (a3_ps1 == alu3_q_prd);
   wire fwd_c2 = alu_q_v & (a3_ps2 == alu_q_prd), fwd_c2b = alu2_q_v & (a3_ps2 == alu2_q_prd), fwd_c2c = alu3_q_v & (a3_ps2 == alu3_q_prd);
   wire [63:0] xc_rs1 = fwd_c1 ? alu_q_val : fwd_c1b ? alu2_q_val : fwd_c1c ? alu3_q_val : prf_a31;
   wire [63:0] xc_rs2 = fwd_c2 ? alu_q_val : fwd_c2b ? alu2_q_val : fwd_c2c ? alu3_q_val : prf_a32;
   wire [63:0] xc_result;
   ooo2_exec u_xc
     (.alu_op(qc_alu_op), .alu_w(qc_alu_w), .alu_uw(qc_alu_uw), .op1_sel(qc_op1_sel),
      .op2_imm(qc_op2_imm), .res_link(qc_res_link), .is_rvc(qc_rvc),
      .is_branch(1'b0), .is_jump(1'b0), .is_jalr(1'b0), .br_func(3'd0),
      .rs1_val(xc_rs1), .rs2_val(xc_rs2), .imm(qc_imm), .pc(qc_pc),
      .pred_npc(qc_pred_npc), .mis_taken(1'b0), .mis_nt(1'b0),
      .result(xc_result), .addr(), .redirect(), .target(), .taken(), .taken_tgt());
   ooo2_exec u_xa
     (.alu_op(qa_alu_op), .alu_w(qa_alu_w), .alu_uw(qa_alu_uw), .op1_sel(qa_op1_sel),
      .op2_imm(qa_op2_imm), .res_link(qa_res_link), .is_rvc(qa_rvc),
      .is_branch(1'b0), .is_jump(1'b0), .is_jalr(1'b0), .br_func(3'd0),
      .rs1_val(xa_rs1), .rs2_val(xa_rs2), .imm(qa_imm), .pc(qa_pc),
      .pred_npc(qa_pred_npc), .mis_taken(1'b0), .mis_nt(1'b0),
      .result(xa_result), .addr(), .redirect(), .target(), .taken(), .taken_tgt());

   wire [63:0] x_result, x_addr, x_target, x_taken_tgt;
   wire        x_redirect, x_taken;

   ooo2_exec u_x
     (.alu_op(q_alu_op), .alu_w(q_alu_w), .alu_uw(q_alu_uw), .op1_sel(q_op1_sel),
      .op2_imm(q_op2_imm), .res_link(q_res_link), .is_rvc(q_rvc),
      .is_branch(q_is_branch), .is_jump(q_is_jump), .is_jalr(q_is_jalr),
      .br_func(q_br_func),
      .rs1_val(x_rs1), .rs2_val(x_rs2), .imm(q_imm), .pc(q_pc),
      .pred_npc(q_pred_npc), .mis_taken(q_mis_taken), .mis_nt(q_mis_nt),
      .result(x_result), .addr(x_addr), .redirect(x_redirect), .target(x_target),
      .taken(x_taken), .taken_tgt(x_taken_tgt));

   // =========================================================== stage M
`include "smolrv64_fp_ops.vh"           // fcmp_s/d, fclass_s/d (in-core FP ops)

   // ---- FS-disabled: any FP instruction (arith, in-core, or FP load/store) executed
   // with mstatus.FS==Off raises illegal-instruction, like Linux lazy-FP. A write to FS
   // redirects and refetches younger ops (csr_file do_fschg) and FP ops are not
   // serializing, but M is the commit point -- so by the time an FP op is here, fs_off
   // is the retired value. Precise by construction.
   wire        fs_off, m_ill_eff;
   wire [2:0]  csr_frm;
   assign m_ill_eff = m_illegal | (m_valid & m_is_fp & fs_off);

   // ---- LSU ----
   // lsu_started is the point after which a load cannot fault -- what lets M let go of it
   // without a ROB walk. Not consumed yet: cutting m_done over to it is the next step.
   wire        lsu_started;
   wire        lsu_done, lsu_done_acc, lsu_fault, lsu_idle, lsu_xo_early, lsu_ld_busy;
   wire [55:0] lsu_cos_pa;  wire [1:0] lsu_cos_kind;   // cosim memory-effect capture
   wire [63:0] lsu_cos_data; wire [3:0] lsu_cos_size;
   wire [63:0] lsu_rd_val, lsu_fault_tval;
   wire        lsu_dtlb_walking, lsu_dtlb_walk_beg;   // HPM: DT_WALK / DTLB_MISS
   wire [3:0]  lsu_fault_cause;
   wire        m_mem_op = m_valid & (m_is_mem | m_is_amo) & ~m_fault & ~m_ill_eff;

   // A CBO executes from M and is not serialized (cbo.zero clears every page the kernel
   // hands out), but it writes memory (cbo.zero, cbo.inval), and M speculates past unresolved
   // branches: it starts only when its op is the ROB head, and the LSU asserts that (rule D17).
   // At the head every older load has landed, so no load-queue entry is older than the CBO
   // (asserted below). Older stores can still sit in the queue -- the senior store queue holds
   // RETIRED stores until they drain -- and "an older store is live" is `sq_av_any`, an entry
   // WITH AN ADDRESS, never the occupancy: entries are allocated at dispatch, so the queue can
   // hold stores younger than the CBO, which wait for M (rule C5). The other M-executed
   // accesses are covered elsewhere: AMO/LR/SC are serializing (`drained`), a load's early
   // start asks `ld_older`.
   wire m_cbo_wait = m_is_cbo & (~m_at_head | sq_av_any);
   always @(posedge clk)
      if (!reset && m_mem_op && m_is_cbo && m_at_head && lq_av_any)
         $fatal(1, "ooo2_core: a load with an address is live under a CBO at the ROB head (pc %h)", m_pc);
   ooo2_lsu #(.AW(AW), .DRAM_TOP(DRAM_TOP), .LRAM_BASE(LBASE), .LRAM_LG2(LRAM_LG2), .LDTW(LQ_IB)) u_lsu
     (.clk(clk), .reset(reset),
      .dtlb_walking(lsu_dtlb_walking), .dtlb_walk_beg(lsu_dtlb_walk_beg),
      // NOT m_mem_op alone. While M holds a COMPLETED op (its done pulse latched, waiting on
      // ld_land or the ROB head) the request would still be presented, the LSU would fall
      // back to S_IDLE, and xl_req would start the very same access A SECOND TIME -- a store
      // written twice. mul/div are already safe this way via md_started; the LSU was not.
      .req_valid(m_mem_op & ~m_unit_done_q & ~m_cbo_wait),
      // A plain store TRANSLATES here and goes no further: its address lands in ooo2_sq and
      // memory is written later, from the buffer's commit port below.
      // Loads AND buffered stores translate here and go no further; the access itself
      // comes back through the pre-translated port, from ooo2_lq or ooo2_sq.
      .req_xlate(m_st_nb | m_ld_nb), .req_early(lq_b_early), .xo_pa(lsu_xo_pa), .xo_unc(lsu_xo_unc), .xo_mem(lsu_xo_mem), .xo_v(lsu_xo_v),
      .xo_early(lsu_xo_early),
      .pt_v(pt_v), .pt_store(pt_store),
      .pt_pa(pt_store ? sq_c_addr : lq_x_pa), .pt_size(pt_store ? sq_c_size : lq_x_size),
      .pt_data(sq_c_data), .pt_signed(lq_x_signed), .pt_fp(lq_x_fp),
      .pt_unc(pt_store ? sq_c_unc : lq_x_unc), .pt_done(lsu_pt_done),
      .pt_ack(lsu_pt_ack), .pt_is_store(lsu_pt_is_store), .pt_ld_done(lsu_pt_ld_done), .pt_ld_kill(lsu_pt_ld_kill),
      .pt_tag(lq_x_idx), .req_tag(m_lq_idx), .pt_rtag(lsu_pt_rtag), .pt_fast_done(lsu_pt_fast_done),
      .req_store(m_is_store & ~m_is_amo), .req_amo(m_is_amo),
      .req_amo_func(m_amo_func), .req_cbo(m_is_cbo), .req_cbo_zero(m_cbo_zero),
      .req_cbo_keep(m_cbo_keep),
      .req_vaddr(m_addr), .req_size(m_mem_size), .req_signed(m_mem_signed),
      .req_fp(m_is_fp), .req_st_data(m_st_data),
      .xl_satp(satp_data), .xl_priv(mmu_dpriv), .xl_sum(mmu_sum), .xl_mxr(mmu_mxr),
      .xl_flush(mmu_flush), .flush(redirect), .m_head(m_at_head),
      // A committed store, or the LQ's candidate at head: the LSU asserts every non-DRAM start
      // is non-speculative against ITS OWN region decode (rule D12). The head compare alone is
      // exact: a live index is unique, a flush empties the queue, and a candidate whose ROB
      // slot has already retired (a load released early) is committed.
      .pt_nonspec(pt_store | lq_x_head),
      .ptw_addr(dptw_addr), .ptw_read(dptw_read),
      .ptw_rdata(dptw_rdata), .ptw_rvalid(dptw_rvalid),
      .mem_raddr(dmem_raddr), .mem_ren(dmem_ren), .mem_runcached(dmem_runcached),
      .mem_rdata(dmem_rdata), .mem_rvalid(dmem_rvalid),
      .mem_rfast(dmem_rfast), .mem_rtag(dmem_rtag), .mem_rvalid_c(dmem_rvalid_c),
      .mem_rtag_resp(dmem_rtag_resp), .mem_rdata_c(dmem_rdata_c), .mem_rbusy(dmem_rbusy),
      .mem_wen(dmem_wen), .mem_waddr(dmem_waddr), .mem_wabase(dmem_wabase), .mem_wdata(dmem_wdata),
      .mem_wmask(dmem_wmask), .mem_wuncached(dmem_wuncached),
      .mem_cbo(dmem_cbo), .mem_cbo_zero(dmem_cbo_zero), .mem_cbo_keep(dmem_cbo_keep),
      .mem_wready(dmem_wready), .mem_waccept(dmem_waccept),
      .cos_pa(lsu_cos_pa), .cos_kind(lsu_cos_kind), .cos_data(lsu_cos_data), .cos_size(lsu_cos_size),
      .started(lsu_started), .done(lsu_done), .done_acc(lsu_done_acc), .rd_val(lsu_rd_val), .fault(lsu_fault),
      .fault_cause(lsu_fault_cause), .fault_tval(lsu_fault_tval), .ld_busy(lsu_ld_busy),
      .err(lsu_err), .idle(lsu_idle));
   assign dmem_idle = lsu_idle;

   // ---- the MD stage: mul/div off the ordered pipe (C1, 2026-09-17) ----
   // The F/CTF port's third drain. One op at a time (the divider is single-occupancy; mul3
   // pipelines but its `busy` admits one), started from the port's forwarded reads in the issue
   // cycle exactly as the F stage captures them, and landing on SH_FE by its own tag like the
   // FPU: rob and prd ride in the stage, never in M. M's completion cone, the load shard's
   // write port and the ordered queue no longer know a multiplier exists.
   reg                md_v, md_div, md_rd_v, md_pend;
   reg [ROB_IDXB-1:0] md_rob;
   reg [RN_PBITS-1:0] md_prd;
   reg [63:0]         md_res_q;
   initial begin md_v = 1'b0; md_pend = 1'b0; end
   wire        mul_done, div_done, mul_busy, div_busy;
   wire [63:0] mul_result, div_result;
   wire        md_f3_2 = qf_insn[14];                     // funct3[2]: div/rem
   // The result is LATCHED on the unit's done pulse (mul3's persists, the divider's is one cycle)
   // and written when SH_FE is free: the FPU and the CTF link cannot hold theirs, this can.
   wire        md_done = md_v & ~md_pend & (md_div ? div_done : mul_done);
   wire        md_wb   = md_pend & (~md_rd_v | (~fp_wb & ~cf_link_wb));   // the op completes (ROB)
   wire        md_wr   = md_wb & md_rd_v;                                    // ...and writes SH_FE
   assign      md_advance = ~md_v | md_wb;
   mul3 u_mul
     (.clk(clk), .reset(reset), .start(iss_md & ~md_f3_2), .abort(redirect),
      .rs1(xf_rs1), .rs2(xf_rs2), .f3(qf_insn[14:12]), .is_w(qf_insn[6:2] == 5'b01110),
      .busy(mul_busy), .done(mul_done), .result(mul_result));
   divider u_div
     (.clk(clk), .reset(reset), .start(iss_md & md_f3_2), .abort(redirect),
      .rs1(xf_rs1), .rs2(xf_rs2), .f3(qf_insn[14:12]), .is_w(qf_insn[6:2] == 5'b01110),
      .busy(div_busy), .done(div_done), .result(div_result));
   always @(posedge clk) begin
      // The writeback's clear comes FIRST: an issue in the same cycle (md_advance = md_wb admits
      // it) must win, or the stage forgets an op the unit is already computing and the next
      // issue starts into a busy multiplier, which ignores it -- rv64um-p-mul hung at retire 111.
      if (md_done) begin md_pend <= 1'b1; md_res_q <= md_div ? div_result : mul_result; end
      if (md_wb)   begin md_v <= 1'b0; md_pend <= 1'b0; end
      if (iss_md) begin
         md_v <= 1'b1; md_div <= md_f3_2; md_rob <= j_rob; md_prd <= qf_prd; md_rd_v <= qf_rd_v; md_pend <= 1'b0;
      end
      // abort(redirect) on the units: a redirect is head-gated, so an in-flight mul/div is
      // younger than the redirecting op = wrong-path. The flush arm is last (rule I11).
      if (redirect) begin md_v <= 1'b0; md_pend <= 1'b0; end
   end
   always @(posedge clk) if (!reset) begin
      if (iss_md & (qf_shard != SH_FE))  $fatal(1, "ooo2_core: a mul/div issued with shard %0d, not SH_FE", qf_shard);
      if (iss_md & md_v & ~md_wb)        $fatal(1, "ooo2_core: a mul/div issued into a busy MD stage");
      if (iss_md & (mul_busy | div_busy)) $fatal(1, "ooo2_core: a mul/div started into a busy unit (the start would be ignored)");
      if (m_valid & m_is_mul)            $fatal(1, "ooo2_core: a mul/div reached M");
      if (md_wr & (fp_wb | cf_link_wb))  $fatal(1, "ooo2_core: the MD stage wrote SH_FE together with the FPU or the link");
      if (md_done & (mul_done & div_done)) $fatal(1, "ooo2_core: both mul and div done at once");
   end

   // ---- FP unit (CVFPU) + the in-core FP ops ----
   // Everything the OoO core needs for squash recovery -- the zombie/drain latch, the
   // abort-by-seqno, the FS-dirty commit gate -- is absent here: M is the commit point,
   // so an FP op in flight can never be squashed, and its decode/operands are stable for
   // its whole (multi-cycle) stay. `decode_fp` runs off the registered m_insn.

   // ---- the SYSQ (C3 step 3, 2026-09-18): system ops fire at the ROB head from flops ------------
   // A CSR/system op (SYSTEM opcode: csr*, ecall/ebreak/xret/wfi/sfence.vma, the irqop pseudo-op)
   // is the F/CTF/MD port's fourth drain. Every one of them is serialising at dispatch
   // (ser_block drains the ROB and the store queue before it and lets nothing dispatch behind
   // it), so at most ONE is in flight and it is the ROB head the moment it exists: the queue is
   // this one register, and its fire is a flop compare. csr_file's upd_* port is driven from
   // these flops (step 2's lesson: a LUTRAM read in front of csr_file's combinational redirect
   // cost IW=3 its closure); the CSR read address is sy_addr, a flop, like m_imm before it.
   // The read's value goes to SH_LD through M's port (M is empty by construction, asserted);
   // the completion through M's ROB port; a trap through c_kill; a redirect through the one
   // redirect gate. A read of instret takes its second cycle at head so the delayed retire
   // count has every older retirement (M1b), unchanged.
   reg                sy_v, sy_rd_v, sy_is_csr, sy_head_q;
   reg [ROB_IDXB-1:0] sy_rob;
   reg [RN_PBITS-1:0] sy_prd;
   reg [2:0]          sy_func;
   reg [11:0]         sy_addr;
   reg [63:0]         sy_src;
   reg [PCW-1:0]      sy_pc;
   reg [SEQW-1:0]     sy_seq;
   reg [31:0]         sy_insn;   // for the cosim's trap record
   initial begin sy_v = 1'b0; sy_head_q = 1'b0; end
   wire sy_at_head = sy_v & (sy_rob == rob_head_idx);
   wire sy_instret = sy_is_csr & ((sy_addr == 12'hC02) | (sy_addr == 12'hB02));
   wire sy_fire    = sy_at_head & ~port_yield & (~sy_instret | sy_head_q);
   wire sy_red     = sy_fire & csr_redir_v;              // trap, xret, illegal CSR: a redirect
   wire sy_trap    = sy_fire & csr_redir_trap;           // ...that is an exception: kill, no result
   wire sy_done    = sy_fire & ~sy_trap;                 // completes through M's ROB port
   wire sy_wr      = sy_done & sy_rd_v & ~csr_illegal;   // the CSR read's value onto SH_LD
   assign sy_advance = ~sy_v | sy_fire;
   always @(posedge clk) begin
      sy_head_q <= ~reset & sy_at_head & ~sy_fire;
      if (sy_fire) sy_v <= 1'b0;
      if (iss_sys) begin
         sy_v <= 1'b1; sy_rob <= j_rob; sy_prd <= qf_prd; sy_rd_v <= qf_rd_v;
         sy_is_csr <= qf_is_csr; sy_func <= qf_csr_func; sy_addr <= qf_imm[11:0];
         sy_src <= qf_csr_func[2] ? {59'b0, qf_imm[16:12]} : xf_rs1;
         sy_pc <= qf_pc; sy_seq <= qf_seq; sy_insn <= qf_insn;
      end
      if (reset | redirect) sy_v <= 1'b0;                // flush arm last (I11); its own redirect included
   end
   always @(posedge clk) if (!reset) begin
      if (iss_sys & sy_v & ~sy_fire)     $fatal(1, "ooo2_core: a system op issued into a busy SYSQ (it is serialising)");
      if (iss_sys & (qf_shard != SH_LD)) $fatal(1, "ooo2_core: a system op issued with shard %0d, not SH_LD", qf_shard);
      if (sy_fire & m_valid)             $fatal(1, "ooo2_core: a system op fires with M busy (pc %h): the drain is broken", sy_pc);
      if (sy_fire & (fpu_busy | f_valid)) $fatal(1, "ooo2_core: a system op fires with FP work in flight -- frm/fflags may change under it");
      if (sy_fire & ld_land)             $fatal(1, "ooo2_core: a system op fires in a load's landing cycle: the port is not free");
      if (m_valid & m_is_sys)            $fatal(1, "ooo2_core: a system op reached M (pc %h)", m_pc);
      if (sy_red & (m_red_fire | cf_red_fire)) $fatal(1, "ooo2_core: the SYSQ redirects together with M or the CTF pipe");
   end

   wire        fp_valid_d, fp_use_fpu, fp_o0i, fp_wrfp, fp_mod;
   wire [2:0]  fp_cls, fp_src, fp_dst, fp_rnd;
   wire [3:0]  fp_op;
   wire [1:0]  fp_int, fp_o0, fp_o1, fp_o2;
   decode_fp u_dfp
     (.insn(m_insn), .fp_valid(fp_valid_d), .use_fpu(fp_use_fpu), .fp_class(fp_cls),
      .op(fp_op), .op_mod(fp_mod), .src_fmt(fp_src), .dst_fmt(fp_dst), .int_fmt(fp_int),
      .rnd(fp_rnd), .op0_sel(fp_o0), .op1_sel(fp_o1), .op2_sel(fp_o2),
      .op0_int(fp_o0i), .wr_fp(fp_wrfp));

   // unified regfile -> the FP sources are just rs1/rs2/rs3 with the fp bit set
   wire [63:0] op1f = m_rs1_val, op2f = m_st_data, op3f = 64'd0;   // no M op reads rs3 (see u_prf's ra3)

   function [63:0] fpsel; input [1:0] s; input [63:0] a, b, c;
      fpsel = (s==2'd1) ? a : (s==2'd2) ? b : (s==2'd3) ? c : 64'd0; endfunction
   // unbox a single from an f-register: a properly NaN-boxed value yields its low 32
   // bits, anything else is the canonical single NaN (RISC-V spec).
   function [31:0] unbox_s; input [63:0] x;
      unbox_s = (x[63:32]==32'hffffffff) ? x[31:0] : 32'h7fc00000; endfunction

   wire [63:0] fpo0r = fpsel(fp_o0, op1f, op2f, op3f);
   wire [63:0] fpo1r = fpsel(fp_o1, op1f, op2f, op3f);
   wire [63:0] fpo2r = fpsel(fp_o2, op1f, op2f, op3f);
   wire        fp_src32 = (fp_src == 3'd0);
   // op0 may be an INTEGER source (I2F / FMV.W.X) -> pass it through unmolested
   wire [63:0] fpo0 = (fp_src32 & ~fp_o0i) ? {32'hffffffff, unbox_s(fpo0r)} : fpo0r;
   wire [63:0] fpo1 = fp_src32 ? {32'hffffffff, unbox_s(fpo1r)} : fpo1r;
   wire [63:0] fpo2 = fp_src32 ? {32'hffffffff, unbox_s(fpo2r)} : fpo2r;

   // DEAD BY CONSTRUCTION, kept for the assertion below. d_cls_f routes every FPU op to
   // the F stage, and the only way one reaches M is with FS off -- which makes m_ill_eff
   // true and so clears this anyway.
   wire        fp_arith  = m_valid & fp_valid_d &  fp_use_fpu & ~m_ill_eff;
   wire        fp_incore = m_valid & fp_valid_d & ~fp_use_fpu & ~m_ill_eff;

   wire        fp_iss_ready, fp_res_valid, fpu_busy;
   wire [63:0] fp_res_data;
   wire [4:0]  fp_res_fflags;
   // FP32 result heading for an f-register -> NaN-box it on writeback
   wire        fp_dst32 = (fp_dst == 3'd0) & fp_wrfp;
   localparam integer FTAGW = 2 + 6 + ROB_IDXB + RN_PBITS;   // dst32, rd_v, rd, rob, prd
   wire [FTAGW-1:0] fp_res_tag;

   // THE TAG CARRIES THE DESTINATION. rule: a response is matched by a tag the requester
   // allocated. 21 bits inside the wrapper's 24: with results returning out of issue order
   // (fpnew's op groups have different latencies), a result must say where it goes rather
   // than be matched against "the one in flight".
   // PIPE_REGS 5 (fp_unit's default is 4): fpnew's own datapath was a 25-level, -0.51 ns
   // family at IW=3. One more cycle of FP latency; the integer side does not pay.
   fp_unit #(.TAGW(FTAGW), .NFLIGHT(4), .PIPE_REGS(5)) u_fpu
     (.clk(clk), .reset(reset),
      .iss_valid(fp_start), .iss_ready(fp_iss_ready),
      .iss_op(ff_op), .iss_op_mod(ff_mod), .iss_src_fmt(ff_src), .iss_dst_fmt(ff_dst),
      .iss_int_fmt(ff_int),
      .iss_rnd(ff_rnd == 3'b111 ? csr_frm : ff_rnd),      // dynamic rm -> fcsr.frm
      .iss_operands({ffo2, ffo1, ffo0}),
      .iss_tag({ff_dst32, f_rd_v, f_rd, f_rob, f_prd}),
      .res_valid(fp_res_valid), .res_ready(1'b1), .res_data(fp_res_data),
      // FLUSH ON REDIRECT. With results landing asynchronously by tag, an op still in
      // fpnew's pipeline when a squash happens would write back after rename had rolled
      // its physreg away -- the zombie writeback, caught by ooo2_pending as "writeback to
      // pN, which was not pending" on rv64ud-p-structural. It only became reachable once
      // FP could reorder and run several deep.
      //
      // Flushing EVERY in-flight op is safe only because head_block still holds: a
      // redirect fires only when its op is the ROB head, so everything older has already
      // committed, and anything in flight is younger by construction. If head_block goes
      // (work list P2), this needs an age or epoch tag instead.
      .res_fflags(fp_res_fflags), .res_tag(fp_res_tag), .flush(redirect), .busy(fpu_busy));

   wire fp_complete = fp_res_valid;

   // ============================================================== stage F (FP arith)
   // A one-entry execute stage parallel to M, fed by u_iq_f. An FP arith op is loaded here
   // at issue and NEVER enters M, which is what makes three schedulers safe: it cannot
   // occupy the shared stage and it cannot head-block, having no trap path (see d_cls_f).
   //
   // Completion is already independent of this stage: the destination rides in the FPU tag
   // and lands through fp_land, its own ROB completion port and the FE shard. That work is
   // what made the split cheap -- all that is added here is dispatch.
   reg         f_valid;
   reg [31:0]  f_insn;
   reg [5:0]   f_rd;
   reg         f_rd_v;
   reg [RN_PBITS-1:0] f_prd;
   reg [ROB_IDXB-1:0] f_rob;
   reg [63:0]  f_rs1_val, f_rs2_val, f_rs3_val;
   initial     f_valid = 1'b0;

   wire        ff_valid_d, ff_use_fpu, ff_o0i, ff_wrfp, ff_mod;
   wire [2:0]  ff_cls, ff_src, ff_dst, ff_rnd;
   wire [3:0]  ff_op;
   wire [1:0]  ff_int, ff_o0, ff_o1, ff_o2;
   decode_fp u_dfp_f
     (.insn(f_insn), .fp_valid(ff_valid_d), .use_fpu(ff_use_fpu), .fp_class(ff_cls),
      .op(ff_op), .op_mod(ff_mod), .src_fmt(ff_src), .dst_fmt(ff_dst), .int_fmt(ff_int),
      .rnd(ff_rnd), .op0_sel(ff_o0), .op1_sel(ff_o1), .op2_sel(ff_o2),
      .op0_int(ff_o0i), .wr_fp(ff_wrfp));

   wire        ff_src32 = (ff_src == 3'd0);
   wire [63:0] ffo0r = fpsel(ff_o0, f_rs1_val, f_rs2_val, f_rs3_val);
   wire [63:0] ffo1r = fpsel(ff_o1, f_rs1_val, f_rs2_val, f_rs3_val);
   wire [63:0] ffo2r = fpsel(ff_o2, f_rs1_val, f_rs2_val, f_rs3_val);
   wire [63:0] ffo0 = (ff_src32 & ~ff_o0i) ? {32'hffffffff, unbox_s(ffo0r)} : ffo0r;
   wire [63:0] ffo1 = ff_src32 ? {32'hffffffff, unbox_s(ffo1r)} : ffo1r;
   wire [63:0] ffo2 = ff_src32 ? {32'hffffffff, unbox_s(ffo2r)} : ffo2r;
   wire        ff_dst32 = (ff_dst == 3'd0) & ff_wrfp;

   wire        fp_start = f_valid & ~redirect;
   wire        fp_disp  = fp_start & fp_iss_ready;   // accepted by the unit this cycle
   assign      f_advance = ~f_valid | fp_disp;

   always @(posedge clk) begin
      if (reset | redirect) f_valid <= 1'b0;
      else if (f_advance) begin
         f_valid   <= iss_f;
         f_insn    <= qf_insn;  f_rd    <= qf_rd;   f_rd_v <= qf_rd_v;
         f_prd     <= qf_prd;   f_rob   <= j_rob;
         f_rs1_val <= xf_rs1;   f_rs2_val <= xf_rs2; f_rs3_val <= xf_rs3;
      end
   end

   always @(posedge clk) if (!reset) begin
      // The F stage only ever holds an FPU op. d_cls_f is decided from decode_fp on d_insn
      // and the same decoder runs here on the payload's insn, so a disagreement means the
      // payload and the classification came from different instructions.
      if (f_valid & ~(ff_valid_d & ff_use_fpu))
         $fatal(1, "ooo2_core: F stage holds a non-FPU op (insn %08x)", f_insn);
      if (iss_f & (qf_shard != SH_FE))
         $fatal(1, "ooo2_core: F-class op is not SH_FE");
      // fp_arith is dead by construction now; if one ever reaches M it would sit there
      // forever, since m_unit_ok no longer has an arm for it.
      if (fp_arith)
         $fatal(1, "ooo2_core: an FPU op reached M -- d_cls_f must route it to the F stage");
      // ROUNDING MODE IS AN FP BARRIER, and out-of-order FP depends on it.
      //
      // Reordering FP is safe for the exception FLAGS because they accumulate: csr_file
      // does `fcsr[4:0] <= fcsr[4:0] | fp_fflags`, an OR, so the order results land in
      // cannot change the answer. The ROUNDING MODE is not like that. A dynamic-rm op
      // (rnd == 3'b111) reads csr_frm when the F stage hands it to the unit, so an frm
      // change must not overtake, or be overtaken by, any FP op in flight.
      //
      // Today that holds for a reason that is not about FP at all: decode_exec sets
      // is_serialize on EVERY CSRRW/S/C, and ser_block drains the ROB before such an op
      // dispatches and lets nothing dispatch behind it until it commits. An FP op commits
      // only once its result has landed, so a drained ROB means nothing is in flight.
      //
      // That is an accident of a broader rule, and it would evaporate the moment CSR ops
      // stop being serializing -- an obvious future optimisation, since serialising every
      // CSR read to make frm safe is heavy-handed. Assert the property directly so it
      // cannot be lost silently.
      if (m_valid & m_is_csr)
         $fatal(1, "ooo2_core: a CSR op is in M (they issue through the SYSQ since C3 step 3)");
   end

   // =========================================================== stage CTF (control flow)
   // A completion stage parallel to M and the F stage, for jal/jalr/bXX, fed from j_* (iss_c).
   // It MIRRORS M's branch handling one-for-one: resolve on its own AGU/comparator, fire the
   // early frontend restart (fr_set) as soon as the mispredict is known, and -- only on a
   // mispredict -- hold until ROB head for the squash, exactly as head_block does for M. So the
   // whole redirect / fr_v / BTB-training block downstream stays structurally identical, just
   // sourced from cf_* instead of m_*. u_iq_c is IN-ORDER, so the oldest branch resolves first
   // and the fr_v "oldest wins" interlock still holds without doc 12's age compare.
   wire [63:0] xf_result, xf_target, xf_taken_tgt;
   wire        xf_redirect, xf_taken;
   ooo2_exec u_xf
     (.alu_op(qf_alu_op), .alu_w(qf_alu_w), .alu_uw(qf_alu_uw), .op1_sel(qf_op1_sel),
      .op2_imm(qf_op2_imm), .res_link(qf_res_link), .is_rvc(qf_rvc),
      .is_branch(qf_is_branch), .is_jump(qf_is_jump), .is_jalr(qf_is_jalr),
      .br_func(qf_br_func),
      .rs1_val(xf_rs1), .rs2_val(xf_rs2), .imm(qf_imm), .pc(qf_pc),
      .pred_npc(qf_pred_npc), .mis_taken(qf_mis_taken), .mis_nt(qf_mis_nt),
      .result(xf_result), .addr(), .redirect(xf_redirect), .target(xf_target),
      .taken(xf_taken), .taken_tgt(xf_taken_tgt));

   reg                cf_valid;
   reg [ROB_IDXB-1:0] cf_rob;
   reg [SEQW-1:0]     cf_seq;
   reg [PCW-1:0]      cf_pc, cf_target, cf_taken_tgt;
   reg [63:0]         cf_link;
   reg                cf_redirect, cf_taken, cf_is_branch, cf_is_jump, cf_is_jalr, cf_rvc;
   reg [5:0]          cf_rd, cf_rs1;
   reg                cf_rd_v;
   reg [RN_PBITS-1:0] cf_prd;
   reg [PDW-1:0]      cf_pdet;      // predictor snapshot, for BTB/YAGS/RAS training (res_pdet_q)
   reg                cf_link_wrote;
   initial begin cf_valid = 1'b0; cf_link_wrote = 1'b0; end

   // RESOLVE-AND-FREE: the branch does NOT hold the stage until head (that would deadlock an
   // out-of-order pipe where a younger branch could occupy it ahead of an older one). It
   // resolves, writes its link, marks its ROB entry done, records any mispredict into the
   // restart tracker (fr_* below), and frees. The squash fires later when that ROB entry heads.
   wire cf_link_pend  = cf_valid & cf_rd_v & ~cf_link_wrote;    // a jal/jalr link still to write
   // THE LINK NEVER WAITS ON M. The old `& ~m_wb_fe` put M's whole completion cone (the SQ's
   // commit, the MMU's state) in front of the CTF pipe's wakeup broadcast -- u_sq/kcc -> m_wb_fe
   // -> cf_link_wb -> we_fe -> e_r, 22 levels. Now the link takes the port when the FPU is not
   // landing, and M YIELDS its FE-shard write for that cycle (m_fe_yield in m_done, like the
   // yields to ld_land and fp_land). Routing the in-core FP ops to SH_LD instead (W1) killed
   // the board's NIC path within seconds while every cosim stayed lockstep-clean.
   wire cf_link_wb    = cf_link_pend & ~fp_wb;
   wire m_fe_yield    = (cf_link_wb | md_wr) & (m_shard == SH_FE);   // registers only: cf_*, md_*, the FPU's out_valid_q, m_shard
   // THE ONE YIELD GATE: every completion that shares M's write and ROB ports (a landing load,
   // the FPU, the link/MD on SH_FE) is gathered here once and applied at every site -- M's
   // three dones and the SYSQ's fire -- never re-derived per unit (docs/rtl-rules.md).
   wire port_yield    = ld_land | fp_land | m_fe_yield;
   wire cf_done       = cf_valid & ~cf_link_pend;               // resolved + link written -> free stage
   assign cf_advance  = ~cf_valid | cf_done;
   // ROB completion. A correctly-predicted branch completes at resolve and retires in order. A
   // MISPREDICT must NOT retire before its squash -- otherwise rob_head advances past fr_rob and
   // cf_red_fire never fires -- so it is marked done only at the squash (cf_red_fire, at head),
   // by fr_rob (the stage has since freed, so cf_rob no longer names it). See the ROB w_ix mux.
   wire cf_land       = (cf_done & ~cf_redirect) | cf_red_fire;
   wire [ROB_IDXB-1:0] cf_land_rob = cf_red_fire ? fr_rob : cf_rob;
   // call/return classification for the RAS, mirroring M's m_link_rd/m_link_rs
   wire cf_link_rd    = cf_rd_v & ((cf_rd == 6'd1) | (cf_rd == 6'd5));
   wire cf_link_rs    = (cf_rs1 == 6'd1) | (cf_rs1 == 6'd5);

   always @(posedge clk) begin
      if (reset | redirect) cf_valid <= 1'b0;
      else if (cf_advance) begin
         cf_valid <= iss_c;
         if (iss_c) begin
            cf_rob <= j_rob;  cf_seq <= qf_seq;  cf_pc <= qf_pc;  cf_rvc <= qf_rvc;
            cf_redirect <= xf_redirect;  cf_target <= xf_target;
            cf_taken <= xf_taken;  cf_taken_tgt <= xf_taken_tgt;
            cf_is_branch <= qf_is_branch;  cf_is_jump <= qf_is_jump;  cf_is_jalr <= qf_is_jalr;
            cf_link <= xf_result;  cf_rd <= qf_rd;  cf_rd_v <= qf_rd_v;  cf_prd <= qf_prd;
            cf_rs1 <= qf_rs1;  cf_pdet <= qf_pdet;  cf_link_wrote <= 1'b0;
         end
      end else if (cf_link_wb) cf_link_wrote <= 1'b1;   // link landed while the branch waits for head
   end

   always @(posedge clk) if (!reset & cf_valid) begin
      if (~(cf_is_branch | cf_is_jump | cf_is_jalr))
         $fatal(1, "ooo2_core: CTF stage holds a non-control-flow op");
   end

   // ---- in-core FP ops (single-cycle, like the ALU): SGNJ/CMP/MVXF/MVFX/FCLASS ----
   wire        fp_isd = m_insn[25];              // 0=single 1=double
   wire [2:0]  fp_f3  = m_insn[14:12];
   wire [31:0] us1 = unbox_s(op1f);              // FMV.X.W is a raw bit-move: never unboxed
   wire [31:0] us2 = unbox_s(op2f);
   wire [1:0]  cmp_d2 = fcmp_d(fp_f3, op1f, op2f);
   wire [1:0]  cmp_s2 = fcmp_s(fp_f3, us1, us2);
   reg  [63:0] fp_incore_res;
   always @* begin
      case (fp_cls)
        3'd1: fp_incore_res = fp_isd                                    // FSGNJ/N/X .D / .S
               ? (fp_f3==3'b000 ? { op2f[63],            op1f[62:0]}
                : fp_f3==3'b001 ? {~op2f[63],            op1f[62:0]}
                :                 { op2f[63]^op1f[63],   op1f[62:0]})
               : {32'hffffffff, (fp_f3==3'b000 ? { us2[31],          us1[30:0]}
                               : fp_f3==3'b001 ? {~us2[31],          us1[30:0]}
                               :                 { us2[31]^us1[31],  us1[30:0]})};
        3'd2: fp_incore_res = {63'd0, (fp_isd ? cmp_d2[0] : cmp_s2[0])};    // FEQ/FLT/FLE -> int
        3'd3: fp_incore_res = fp_isd ? op1f : {{32{op1f[31]}}, op1f[31:0]}; // FMV.X.D/W -> int (raw)
        3'd4: fp_incore_res = fp_isd ? op1f : {32'hffffffff, op1f[31:0]};   // FMV.D/W.X -> fp (box)
        3'd5: fp_incore_res = fp_isd ? fclass_d(op1f) : fclass_s(op1f);     // FCLASS -> int
        default: fp_incore_res = 64'd0;
      endcase
   end

   // fcsr.fflags accumulation: a CVFPU completion, or an in-core compare's NV bit.
   // (in-core ops other than compares raise no flags.) Both are one-cycle events: the
   // completion pulses once, and an in-core op occupies M for exactly one cycle.
   wire       fp_icmp    = fp_incore & (fp_cls == 3'd2);
   wire       fp_icmp_nv = fp_isd ? cmp_d2[1] : cmp_s2[1];
   // OR, not a priority mux. While the FPU blocked M these two could never coincide -- an
   // in-core compare could not be in M while an FPU op was completing there. With the FPU
   // released at issue they can, and the mux silently DROPPED the compare's NV flag.
   wire       fp_flags_we = fp_complete | fp_icmp;
   wire [4:0] fp_flags    = (fp_complete ? fp_res_fflags      : 5'd0)
                          | (fp_icmp     ? {fp_icmp_nv, 4'd0} : 5'd0);

   // ---- CSR file ----
   wire        m_is_sys  = m_valid & (m_insn[6:2] == 5'b11100) & ~m_ill_eff & ~m_fault;
   wire [63:0] csr_rdata, csr_redir_tgt;
   wire        csr_redir_v, csr_redir_trap, csr_illegal;
   wire        csr_irq_v;
   wire [3:0]  csr_irq_cause;

   // external (non-system-op) traps: fetch fault, illegal instruction, data fault
   wire        xtrap_v     = m_valid & (m_fault | m_ill_eff | (m_mem_op & m_lsu_flt));
   wire        m_done_red;                // M's done with the LSU arm removed; defined with redirect
   wire [3:0]  xtrap_cause = m_fault  ? m_fault_cause
                           : m_ill_eff? 4'd2                     // illegal instruction
                           :            m_lsu_fc;
   wire [63:0] xtrap_tval  = m_fault  ? m_fault_tval
                           : m_ill_eff? 64'd0
                           :            lsu_fault_tval;

   // ---- stall attribution: turn CPI into a CPI stack ----
   // The pipe fails to retire on a given cycle for exactly one of two reasons: M is
   // holding an instruction that has not completed (charged to the unit it is waiting
   // on), or X had no instruction to give (a frontend bubble, sub-attributed to the
   // iMMU walking vs the I$ having no window). `st_ser` is the third case: M is free
   // but a serializing op in flight keeps the frontend from handing anything over.
   // REDEFINED when the LSU and the FPU stopped blocking M. The wait did not go away, it
   // MOVED: M advances and the consumer is held in X instead, which landed in `st_ser`
   // (accept's ~d_hold term) and made a dependent stall read as a serializing op. Each
   // dependent wait is now charged to the unit that owns the register it is waiting for,
   // so the stack stays additive and comparable across the change. st_m and m_advance are
   // exact complements, so the two halves of each bucket cannot double-count.
   wire st_m      = m_valid & ~m_done;              // M stalled at all
   // The dependent wait no longer happens in X -- it happens in the scheduler, which
   // reports WHICH register its oldest entry is blocked on. A physical register carries its
   // shard in the top bits, so the stall is charged to the unit that owns the result.
   // ---- DEPENDENCY ATTRIBUTION: per scheduler, not through a priority mux -------------
   // `iq_blk_pr` is `rl_blk_v ? rl_blk_pr : rf_blk_v ? rf_blk_pr : ri_blk_pr` -- a priority
   // mux built when there was one scheduler and kept when there were three. With three, an
   // FP dependency in u_iq_f is INVISIBLE in any cycle u_iq_l is also blocked, so it was
   // charged to ST_MEM instead. That is not a small skew: on workloads/mlbench, whose
   // critical path is an FP add chain, ST_FPU read 0% and ST_MEM read 41%.
   //
   // Each scheduler's blocking source is classified on its own shard and OR-ed. A cycle can
   // now count in more than one event, which is correct -- the machine really is waiting on
   // both -- and matches how these events already behaved (the stack sums past 100%).
   wire [2:0] bsh_l = rl_blk_pr[RN_PBITS-1:RN_IDXB];
   wire [2:0] bsh_f = rf_blk_pr[RN_PBITS-1:RN_IDXB];
   wire [2:0] bsh_i = ri_blk_pr[RN_PBITS-1:RN_IDXB];
   wire [2:0] bsh_i2 = ri2_blk_pr[RN_PBITS-1:RN_IDXB];
   // Gated on "nothing issued", NOT on m_advance. The old `m_advance &` gate dated from M
   // being the only unit, where "M could accept" was the same thing as "issue could
   // proceed". With three units it silently masks: on workloads/mlbench M is busy with
   // loads nearly every cycle, so dep_fp could never fire and the FP add chain that IS the
   // critical path reported 0%.
   wire no_issue  = ~(iq_iss_v | pick_i | pick_i2);
   wire dep_ld    = no_issue & ((rl_blk_v & (bsh_l == SH_LD))
                              | (rf_blk_v & (bsh_f == SH_LD))
                              | (ri_blk_v & (bsh_i == SH_LD))
                              | (ri2_blk_v & (bsh_i2 == SH_LD)));
   wire dep_fp    = no_issue & ((rl_blk_v & (bsh_l == SH_FE))
                              | (rf_blk_v & (bsh_f == SH_FE))
                              | (ri_blk_v & (bsh_i == SH_FE))
                              | (ri2_blk_v & (bsh_i2 == SH_FE)));
   wire st_mem    = (st_m & m_mem_op) | dep_ld;     // ...on the LSU
   wire st_div    = md_v &  md_div;                 // the MD stage holds a divide (C1: occupancy, not an M stall)
   wire st_mul    = md_v & ~md_div;                 // ...a multiply
   // ST_FPU MUST WATCH STAGE F. It used to be `(st_m & fp_arith) | dep_fp`, and fp_arith is
   // identically 0 since FP stopped entering M -- so the FPU's own occupancy vanished from
   // the stack the moment it got its own stage. `f_valid & ~fp_disp` is stage F holding an
   // op the unit will not yet take, which is exactly the old `st_m & fp_arith` term in its
   // new home. (~f_advance is the same expression; written out for clarity.)
   wire st_fpu    = (f_valid & ~fp_disp) | dep_fp;  // ...on the FPU
   // ~st_rob: the two are now disjoint, so the stack does not count a ROB-full cycle
   // twice under two different names.
   // THE DISPATCH HOLD, NAMED (2026-09-07). `st_ser` was this whole residual -- M could advance,
   // dispatch did not, and it was not a load or FP dependency or the ROB -- and it read 6-13%
   // of the cycles in EVERY Geekbench subtest, where serializing ops are rare: what it held
   // was the schedulers filling up behind integer chains, the store and load queues, the
   // rename free lists. Each cause has its own event now, in d_hold's order so they are
   // disjoint and sum to ST_DSP, the bucket kept whole for the 13-counter `cpi` set.
   wire st_dsp    = m_advance & ~accept & ~dep_ld & ~dep_fp & ~st_rob;
   wire st_iq     = st_dsp & ~iq_ready;                              // the class's scheduler is full
   wire st_rn     = st_dsp &  iq_ready & rn_stall;                   // rename: a free list is empty
   wire st_sq     = st_dsp &  iq_ready & ~rn_stall & (d_st_nb & ~sq_d_ready);
   wire st_lq     = st_dsp &  iq_ready & ~rn_stall & ~(d_st_nb & ~sq_d_ready) & (d_ld_nb & ~lq_d_ready);
   wire st_srz    = st_dsp &  iq_ready & ~rn_stall & ~(d_st_nb & ~sq_d_ready) & ~(d_ld_nb & ~lq_d_ready);
   wire st_ser    = st_dsp;                                          // the bus's bit 11, as before
   wire fe_bub    = ~st_m & ~d_valid & ~redirect;   // X starved, M not already stalled
   wire fe_mmu    = fe_bub & ~immu_ready;           // ...iMMU walking
   wire fe_ic     = fe_bub &  immu_ready & (imem_avail_g == {$clog2(HW+2){1'b0}});

   // fe_ic only fires when the fetch window is EXACTLY empty, so everything else landed in
   // an unattributed remainder -- 21% of cycles on a pure-ALU loop at 166 MHz, and 31% with
   // compression off, with the LSU, caches, MMU and branches all out of the picture.  It was
   // the largest single bucket in the machine and nothing said what it was.  Two cases hide
   // in there and they call for opposite fixes:
   //   fe_aln  fetch had BYTES but could not assemble an instruction (partial window /
   //           straddle).  Fix = wider or better-aligned fetch.
   //   fe_que  fetch DID assemble one; the decoupling queue still had nothing for decode
   //           (refill latency after a drain).  Fix = deeper queue / earlier restart.
   // That it grows with instruction size points at fe_aln, but pointing is not measuring.
   wire fe_rest   = fe_bub &  immu_ready & (imem_avail_g != {$clog2(HW+2){1'b0}});
   wire fe_aln    = fe_rest & ~fe_dq_valid;
   wire fe_que    = fe_rest &  fe_dq_valid;

   // REDIR was one counter for every reason the pipe restarts, so a 3.4-per-1000 redirect
   // rate could not be attributed to conditional branches, indirect jumps, or traps -- and
   // predictor work would have been tuning blind.  csr_red wins the priority: a trap that
   // lands on a branch is a trap.  REDIR total minus these three is the remainder
   // (fence.i and direct-jal mispredicts), so nothing needs a fourth counter.
   wire red_trap  = (m_red_fire & csr_red) | sy_trap;   // a trap redirect: M's or the SYSQ's
   wire red_br    = cf_red_fire & cf_is_branch;
   wire red_jalr  = cf_red_fire & cf_is_jalr;

   // ST_ROB: dispatch has an instruction and the ROB has no room. Split out of ST_SER
   // because that event's name says "serializing op" while it actually absorbed EVERY
   // non-dependency dispatch stall -- and on workloads/mlbench the dominant one is not
   // serialisation at all: d_hold is 99% rob_full, because a dot-product accumulator chain
   // blocks RETIREMENT rather than issue, so it never appears as a dependency stall and
   // ST_FPU correctly reads 0%. Without this bit that workload's real limiter is invisible
   // in the CPI stack.
   wire st_rob = d_valid & ~rob_ready;
   // The mispredict DRAIN (plan item 5, 2026-09-05): a redirect resolved in M waits for the
   // ROB head (head_block) before it fires. These are the cycles P7's rename walk-back
   // would recover; on the stack they show what the drain costs before it is built.
   wire rd_wait = fr_v & ~cf_red_fire;   // a tracked branch restart still waiting to reach head
   // THE MEMORY BUCKETS (2026-09-17, program C0/B2): ST_MEM was one bit for everything the
   // backend did, and a backend rewrite would move one number. Each of these is a fact the
   // queues already compute, registered here like the rest; none is in any completion cone.
   wire mem_hitser    = lq_x_v & ~lq_x_take;                 // a ready load candidate the door did not take
   wire mem_ldinfl    = lsu_ld_busy;                          // a load access in flight (hit ~3 cycles; the rest is miss wait)
   wire mem_stdoor    = dmem_wen & ~dmem_waccept;             // a store at the D$ door, unaccepted
   wire mem_alias_unk = sq_ld_block &  sq_l_block_unk_q[lq_x_idx];   // blocked: an older store's address is unknown
   wire mem_alias_ovl = sq_ld_block & ~sq_l_block_unk_q[lq_x_idx];   // blocked: a known older store overlaps
   wire mem_reord     = sq_ld_reorder;                        // a load issued past an uncommitted older store (the payoff)
   wire mem_wpkill    = lsu_pt_ld_done & lsu_pt_ld_kill;      // a wrong-path load's landing killed after its access ran
   wire mem_devwait   = lq_x_devwait;                         // a device load waiting to be the head
   wire [38:0] hpm_ev = {mem_devwait, mem_wpkill, mem_reord, mem_alias_ovl, mem_alias_unk,
                         mem_stdoor, mem_ldinfl, mem_hitser,
                         st_srz, st_lq, st_sq, st_rn, st_iq,
                         lsu_dtlb_walk_beg, lsu_dtlb_walking, rd_wait, st_rob, hpm_fb_rhit, hpm_fb_hit,
                         fe_que, fe_aln, red_trap, red_jalr, red_br,
                         fe_ic, fe_mmu, fe_bub, st_ser, st_fpu, st_mul, st_div, st_mem,
                         hpm_ic_miss, hpm_ic_access, hpm_dc_miss, hpm_dc_access,
                         // LOAD/STORE completions from REGISTERED landings (2026-09-17): the old
                         // `m_valid & m_is_mem & lsu_done` started at the dTLB compare (m_addr) and
                         // was the worst family of the C0 IW=3 census (m_addr_reg -> hpm_ev_q_reg).
                         // A load completes when it lands; a store when the LSU takes it from the
                         // senior queue -- which is what "completion" means since the queues.
                         redirect, lsu_pt_ack & pt_store, ld_land};

   // FMAX: the Zihpm event bus is REGISTERED. hpm_ev -> hpm_inc -> a 64-bit mhpmcounter
   // carry chain was 823 of 3113 failing endpoints at 6 ns and the WORST family in the
   // design (m_addr -> ... -> u_csr/mhpmcounter[12][63]).  These 15 bits are pure
   // instrumentation and cost nothing to delay: a counter is read through a CSR many
   // cycles later, and no software can observe which cycle an event landed on.  They only
   // became timing-critical when 9a6f8de3 correctly un-gated perf_access/perf_miss from
   // `ifdef PERF_TRACE -- before that the cache events read zero in every bitstream ever
   // built, so this cone did not exist.
   // minstret takes the delayed copy too since a head-gated op waits a second cycle at head
   // (m_head_q), which makes the one-cycle lag invisible to any CSR read.
   //
   // But that argument covers minstret ONLY, and the first version of this fix stopped
   // there -- leaving the OTHER route from the same source alive. `retire` is
   // `rob_c_valid`, and ooo2_rob's `head_done` write-forwards across every writeback port,
   // so retire sits downstream of every unit's completion in the cycle it happens:
   //   m_addr -> lsu_done -> rob w_hits -> retire -> retire_cnt
   //          -> hpm_inc's INSTRET arm -> 13 event muxes -> 13x 64-bit carry chain
   // and `u_csr/mhpmcounter[12]` came back as the worst family at NF=7 (-0.082, 32 levels,
   // 10x CARRY8). mhpmcounterN is instrumentation by the same argument as hpm_ev above --
   // read through a CSR many cycles later, and no software can observe which cycle an
   // event landed on -- so it takes the delayed copy and minstret keeps the live one.
   // Registering it here rather than in csr_file also keeps the src/ OoO core, which
   // shares that module, bit-identical: it passes its live count to both ports.
   reg [38:0] hpm_ev_q;
   reg [5:0]  hpm_lqocc_q, hpm_sqocc_q;   // queue occupancies, per cycle (MEM_LQOCC / MEM_SQOCC)
   reg [5:0]  hpm_ret_q;
   initial begin hpm_ev_q = 39'd0; hpm_ret_q = 6'd0; hpm_lqocc_q = 6'd0; hpm_sqocc_q = 6'd0; end
   always @(posedge clk) begin
      hpm_ev_q  <= reset ? 39'd0 : hpm_ev;
      hpm_lqocc_q <= reset ? 6'd0 : {{(6-LQ_IB-1){1'b0}}, lq_occ};
      hpm_sqocc_q <= reset ? 6'd0 : {{(6-SQ_IB-1){1'b0}}, sq_occ};
      hpm_ret_q <= reset ? 6'd0 : {5'd0, retire} + {5'd0, retire2} + {5'd0, retire3};
   end

   csr_file u_csr
     (.clk(clk), .reset(reset),
      .raddr(sy_addr), .rdata(csr_rdata),
      .redir_target(csr_redir_tgt), .redir_valid(csr_redir_v),
      .redir_is_trap(csr_redir_trap), .csr_illegal(csr_illegal),
      .o_satp(mmu_satp), .o_priv(mmu_priv), .o_dpriv(mmu_dpriv),
      .o_sum(mmu_sum), .o_mxr(mmu_mxr), .o_frm(csr_frm), .o_fs_off(fs_off),
      .fp_fflags_we(fp_flags_we), .fp_fflags(fp_flags),
      // mstatus.FS -> Dirty when an FP-state writer RETIRES. In-order that is exactly
      // "a retiring instruction wrote an f-register" (arch 32..63), which covers FP
      // arith, the in-core FP writers and FP loads -- and excludes FSW/FSD, which
      // write memory, not FP state. Commit-gated by construction: `retire` is the
      // commit event, so a trapping op never dirties FS.
      .fp_dirty_commit(retire & rob_c_rd_v & rob_c_rd[5]),
      .o_tlb_flush(mmu_flush),
      // GATED BY m_done. Neither of these was, because a SYSTEM op or a poisoned instruction
      // always completed in its single M cycle -- so `in M` and `completing` were the same
      // thing. head_block and ld_land can now hold one in M for several cycles, and an
      // ungated effect applies EARLY (before the op is the ROB head) and then AGAIN on every
      // stalled cycle. That is what put the machine in supervisor mode one instruction ahead
      // of the reference, at the paging transition.
      // m_done_red, not m_done: a trap request is never a live memory completion (a data
      // fault reaches here latched), and csr_file's redir_valid is combinational in this
      // input -- with m_done here the LSU's whole done sat inside csr_redir_v -> redirect.
      .xtrap_v(xtrap_v & m_done_red), .xtrap_intr(1'b0), .xtrap_cause(xtrap_cause),
      .xtrap_epc(m_pc), .xtrap_tval(xtrap_tval),
      // retire3 (3rd commit port, IW>=3) must be summed too or minstret undercounts at
      // 3-wide; retire3 is hard 0 at IW<3, so this stays bit-identical at the shipping width.
      // Both counts are the DELAYED copy: minstret is exact because a CSR op completes only
      // in its second cycle at the ROB head (m_head_q), by which time every older retirement
      // has been counted; csr_file drops the CSR op's own retirement after a minstret write.
      .hw_ip(hw_ip), .mtime(mtime), .retire_cnt(hpm_ret_q),
      .hpm_retire_cnt(hpm_ret_q), .hpm_ev(hpm_ev_q), .hpm_lqocc(hpm_lqocc_q), .hpm_sqocc(hpm_sqocc_q),
      .irq_v(csr_irq_v), .irq_cause(csr_irq_cause),
      // csr_file's ILA debug bus. The SoC puts no ILA on the CSR file, so these
      // outputs go nowhere -- named and left EMPTY on purpose. PINMISSING gates this build
      // and PINCONNECTEMPTY does not, so a deliberate non-connection has to say so instead
      // of being silently omitted (same treatment as the iMMU's t_uncached).
      .dbg_timer(), .dbg_mtvec(), .dbg_mtvec_we(), .dbg_csrop(), .dbg_csrop_v(),
      // m_done_red here too: a system op is never a memory op, and upd_valid feeds the CSR
      // unit's redirect and trap-target logic -- with the full m_done, build D of 2026-09-04
      // had 1357 near-critical endpoints starting at m_addr: dTLB compare -> lsu_done ->
      // m_done -> upd_valid -> mepc/priv -> the vectored trap-target adder -> fe_red_tgt_q.
      .upd_valid(sy_fire), .upd_is_csr(sy_is_csr), .upd_func(sy_func),   // the SYSQ's flops (C3 step 3)
      .upd_addr(sy_addr),
      .upd_src(sy_src),
      .upd_pc(sy_pc));

   // ---- NON-BLOCKING LOADS ------------------------------------------------------------
   // M lets go of a plain load at DISPATCH instead of at data-return. Safe with no ROB walk
   // because ooo2_lsu decides the fault before the access starts (see ooo2_lsu.started): past
   // S_IDLE a load cannot fault, so nothing older can still trap once M has moved on.
   //
   // Exactly ONE load in flight, and that is not a simplification to revisit casually -- the
   // PRF has one write address (ooo2_prf: one `wa`, three shard enables), so two completions
   // in a cycle have nowhere to go. Multiple outstanding loads is the step that has to solve
   // that, together with a load queue and D$ MSHRs.
   wire m_ld_nb  = m_mem_op & ~m_is_store & ~m_is_amo & ~m_is_cbo;  // plain load, rule C1
   // The 1-deep load scoreboard that stood here is GONE, replaced by ooo2_lq. It tracked a
   // load from the cycle its ACCESS STARTED, which is why the ordering test had nowhere to
   // live but ooo2_lsu's start gate, on the end of the translate path. The queue tracks it
   // from the cycle its ADDRESS IS KNOWN instead, and holds that address in a flop -- which
   // is the entire point. Its "a second load dispatched with one already in flight"
   // assertion is likewise retired: several in flight is now the intent, not a bug.
   //   sb_preg/sb_rd/sb_rd_v/sb_rob -> lq_l_prd/lq_l_rd/lq_l_rd_v/lq_l_rob

   // ---- FP scoreboard: the FPU releases M at ISSUE, not at result -------------------
   // Same shape as the load slot above, and the same argument makes it safe: an FP op
   // cannot fault (it reports exceptions in fflags, never as a trap), so once it is in the
   // unit it is architecturally guaranteed to complete, and nothing older than it can trap
   // either -- everything that CAN trap is head-gated and would not have left X.
   //
   // ONE DIFFERENCE, forced by the unit. `res_valid` is a one-cycle pulse and two of the
   // three fp_unit variants (fp_unit_synth.sv, fp_unit_stub.sv) ignore `res_ready`
   // entirely, so the result cannot be parked in the FPU. It is captured HERE and written
   // when the PRF's single port is free. The load wins that arbitration because
   // `lsu_rd_val` is transient while this register is not.
   // The scoreboard that stood here (fb_busy/fb_preg/fb_rob/fb_val/fb_got, plus a held
   // result for the cycle a load stole the ROB port) is GONE. Every field it carried now
   // rides in the tag and comes back with the result, which is what lets more than one op
   // be in flight at all -- fpnew returns them out of issue order across op groups.
   wire        ft_dst32 = fp_res_tag[FTAGW-1];
   wire        ft_rd_v  = fp_res_tag[FTAGW-2];
   wire [5:0]  ft_rd    = fp_res_tag[FTAGW-3 -: 6];
   wire [ROB_IDXB-1:0]  ft_rob = fp_res_tag[RN_PBITS +: ROB_IDXB];
   wire [RN_PBITS-1:0]  ft_prd = fp_res_tag[RN_PBITS-1:0];
   wire [63:0] fp_wval  = ft_dst32 ? {32'hffffffff, fp_res_data[31:0]} : fp_res_data;
   // No ~ld_land: FP has its own ROB completion port now, so a load landing in the same
   // cycle no longer displaces it and there is nothing to hold.
   wire        fp_land  = fp_complete;
   wire        fp_wb    = fp_land & ft_rd_v;
   always @(posedge clk) if (!reset) begin
      // The SH_FE check moved to the F stage, where it compares the shard of the op being
      // dispatched. Left here it compared fp_disp -- now the F stage's -- against M's
      // shard, two unrelated instructions, and fired on rv64ud-p-fcvt.
      // The tag is the only thing naming the destination now, so a result that arrives
      // unowned would write a live register silently instead of being caught by fb_busy.
      if (fp_land & (ft_prd == {RN_PBITS{1'b0}}) & ft_rd_v)
         $fatal(1, "ooo2_core: FP result claims rd_v with physreg 0");
   end

   // ---- completion ----
   wire m_unit_ok = ~m_valid             ? 1'b1
                 : m_fault | m_ill_eff   ? 1'b1   // poisoned: traps immediately
                 // `done` is now M's alone: it is fault | translate-only | an access this
                 // stage started, and every access ooo2_lq or ooo2_sq starts reports on
                 // pt_done instead (the own_pt latch in ooo2_lsu). The ~sb_busy qualifier
                 // that used to be needed here -- an older load's completion satisfying a
                 // younger store that never executed -- has no case left to cover.
                 : m_mem_op              ? lsu_done
                 :                         1'b1;  // in-core FP included: single-cycle

   // COMPLETION IS STICKY, and it has to be. Every unit's `done` is a one-cycle PULSE --
   // mul3's is `v3`, the divider's is `(st == S_FIN)` with `S_FIN: st <= S_IDLE`, the FPU's
   // is res_valid -- and that was safe only while nothing could hold m_done low. ld_land now
   // can. A pulse arriving in a held cycle would be LOST: md_started stays set so the unit
   // never restarts, its done never re-asserts, and M waits forever for a completion that
   // already happened. The RESULT has to be latched with it for the same reason; mul3's res3
   // happens to persist, but the divider presents its result only in S_FIN.
   reg        m_unit_done_q;
   reg [63:0] m_unit_res_q;
   // ...and the FAULT with it. `fault` is combinational from req_valid, so withdrawing the
   // request (above) also withdraws the fault: lsu_fault drops, xtrap_v drops, head_block
   // clears, and a misaligned store retires having neither trapped NOR executed. The result
   // was not the only thing that had to survive the pulse.
   reg        m_unit_flt_q;
   reg [3:0]  m_unit_fc_q;
   initial begin m_unit_done_q = 1'b0; m_unit_flt_q = 1'b0; end
   wire [63:0] m_unit_res = (m_is_mem | m_is_amo) ? lsu_rd_val
                          : fp_arith              ? (fp_dst32 ? {32'hffffffff, fp_res_data[31:0]}
                                                              : fp_res_data)
                          : fp_incore             ? fp_incore_res
                          :                         m_result;
   always @(posedge clk)
      if (reset | m_advance)  m_unit_done_q <= 1'b0;
      else if (m_unit_ok) begin
         m_unit_done_q <= 1'b1;
         m_unit_res_q  <= m_unit_res;
         m_unit_flt_q  <= lsu_fault;
         m_unit_fc_q   <= lsu_fault_cause;
      end
   wire m_done_raw = m_unit_ok | m_unit_done_q;
   // EVERY TRAP CONSUMER TAKES THE LATCHED VIEW, AND ONLY THE LATCHED VIEW. A data-side fault
   // is decided by the dTLB compare in the cycle the LSU reports it; taking the trap in that
   // same cycle put m_addr -> TLB -> lsu_fault -> xtrap_v -> redirect -> the fetch adder ->
   // the decoupling queue's write data in one 26-level path. Now the cycle that reports the fault
   // only LATCHES it (m_flt_pulse holds m_done low for that one cycle); the trap, the
   // redirect and the head gate all read the copy next cycle. One cycle per data fault, and
   // faults are the rarest thing M does. fault_tval needs no latch: it is req_vaddr, which
   // is m_addr, a register.
   wire       m_flt_pulse = m_mem_op & lsu_fault & ~m_unit_done_q;
   wire       m_lsu_flt = m_unit_done_q & m_unit_flt_q;
   wire [3:0] m_lsu_fc  = m_unit_fc_q;

   // A trap or a redirect may only fire when M IS THE ROB HEAD. The trapping instruction is
   // YOUNGER than an outstanding load, and `flush` kills everything -- including that older
   // entry, whose register write would be lost even though it is architecturally before the
   // trap and cannot itself fault. Waiting costs nothing measurable: redirects run 3.4 per
   // 1000 instructions, and the load it waits on is already on its way back.
   // Stated over the instruction CLASS, not over csr_red/m_trap. Those are csr_file outputs,
   // and the effects that must be head-gated (upd_valid, xtrap_v) are csr_file INPUTS -- so
   // gating them through csr_red would close a combinational loop. Every term here is either
   // registered or decoded from m_insn.
   reg  fr_v;   initial fr_v = 1'b0;
   reg [SEQW-1:0]     fr_seq;   // seqno of the oldest pending restart; younger restarts are ignored
   reg [ROB_IDXB-1:0] fr_rob;   // its ROB slot: the backend squash fires when this reaches head
   wire m_needs_head = m_redirect | m_is_fencei
                     | m_fault | m_ill_eff | (m_mem_op & m_lsu_flt);
   // A HEAD-GATED OP COMPLETES IN ITS SECOND CYCLE AT HEAD, not its first. Nothing retires
   // while it waits (retire is in order and it is the head), so by its second cycle every
   // older retirement is two edges old -- which is what lets minstret take the same delayed
   // retire count as the Zihpm counters (hpm_ret_q) and still read exactly: the live count
   // was m_addr -> dTLB -> lsu_done -> rob w_hits -> retire -> minstret, 28 levels, the
   // deepest path in the IW=3 build. One cycle on a CSR op, a trap, a fence.i or a system op.
   // ONLY A CSR READ OF instret/minstret TAKES THE SECOND CYCLE (M1b, 2026-09-17): every other
   // head-gated op (the irqop, sret/ecall, fence.i, traps) completes in its first cycle at
   // head exactly as before, and a minstret WRITE needs no hold (the retirements the delayed
   // count still carries are older than the write and are subsumed by it; csr_file drops the
   // writer's own). Holding every head-gated op broke the board (the NIC path died once init
   // started) while every cosim stayed lockstep-clean.
   reg  m_head_q;   initial m_head_q = 1'b0;
   always @(posedge clk) m_head_q <= ~reset & m_valid & m_at_head & ~m_done;
   wire head_block   = m_valid & m_needs_head & ~m_at_head;   // the instret second cycle is the SYSQ's now

   // One write port, one ROB completion port: when a load lands, M yields the cycle. Costs
   // ~0.3 cycles per load against the ~2.3 the early release saves.
   assign m_done = m_done_raw & ~head_block & ~port_yield & ~m_flt_pulse;
   assign m_advance = ~m_valid | m_done;


   // ---- the trap shadow (C3 step 1, 2026-09-18; the system-op half became the SYSQ in step 3) ------
   // Everything M feeds csr_file with -- the upd_* payload of a CSR/system op and the xtrap_*
   // payload of a trap -- is captured here per ROB slot at the point it becomes known: a decode
   // fault or an illegal instruction at dispatch, a system op's operands as M takes it, a data
   // fault as the LSU reports it. Nothing reads these entries yet. Every cycle M drives one of
   // the two ports, the entry at m_rob_idx must exist with the right kind and agree in every
   // field; a silent disagreement here would be a wrong trap once csr_file reads the entry
   // instead of M. Always on (docs/rtl-rules.md A6). Synthesis removes the arrays (no reader).
   // TRIED AND REJECTED (step 2, 2026-09-18): feeding csr_file's upd_*/xtrap_* from these arrays
   // read at m_rob_idx was retire-identical everywhere but cost IW=3 its closure (0.000 -> -0.030):
   // the LUTRAM read sits in front of csr_file's combinational redirect, and m_rob_idx -> read ->
   // csr_illegal/redir -> the schedulers' kill became the worst family. The payload csr_file
   // consumes must be FLOPS: step 3 registers the head entry a cycle ahead of its fire.
   localparam [1:0] SYK_NONE = 2'd0, SYK_XTRAP = 2'd2;
   reg [1:0]     xtq_kind   [0:ROB_DEPTH-1];
   reg [PCW-1:0] xtq_pc     [0:ROB_DEPTH-1];
   reg [3:0]     xtq_cause  [0:ROB_DEPTH-1];
   reg [63:0]    xtq_tval   [0:ROB_DEPTH-1];
   integer sqi;
   initial for (sqi = 0; sqi < ROB_DEPTH; sqi = sqi + 1) xtq_kind[sqi] = SYK_NONE;
   always @(posedge clk) if (!reset) begin
      // (a) dispatch: a decode fault or an illegal instruction traps with what decode knows
      if (rn_valid) begin
         xtq_kind[rob_d_idx]  <= (d_fault | d_illegal) ? SYK_XTRAP : SYK_NONE;
         xtq_cause[rob_d_idx] <= d_fault ? d_fault_cause : 4'd2;
         xtq_tval[rob_d_idx]  <= d_fault ? {{(64-PCW){1'b0}}, d_fault_tval} : 64'd0;
         xtq_pc[rob_d_idx]    <= d_pc;
      end
      if (rn_valid_b) begin
         xtq_kind[rob_d_idx2]  <= (d2_fault | d2_illegal) ? SYK_XTRAP : SYK_NONE;
         xtq_cause[rob_d_idx2] <= d2_fault ? d2_fault_cause : 4'd2;
         xtq_tval[rob_d_idx2]  <= d2_fault ? {{(64-PCW){1'b0}}, d2_fault_tval} : 64'd0;
         xtq_pc[rob_d_idx2]    <= d2_pc;
      end
      if (rn_valid_c) begin
         xtq_kind[rob_d_idx3]  <= (d3_fault | d3_illegal) ? SYK_XTRAP : SYK_NONE;
         xtq_cause[rob_d_idx3] <= d3_fault ? d3_fault_cause : 4'd2;
         xtq_tval[rob_d_idx3]  <= d3_fault ? {{(64-PCW){1'b0}}, d3_fault_tval} : 64'd0;
         xtq_pc[rob_d_idx3]    <= d3_pc;
      end
      // (b) M takes an op: the FP-off illegal is decided here
      if (iss_m & ~q_illegal & ~q_fault & q_is_fp & fs_off) begin
         xtq_kind[i_rob] <= SYK_XTRAP;  xtq_cause[i_rob] <= 4'd2;  xtq_tval[i_rob] <= 64'd0;
      end
      // (c) the LSU reports a data fault for M's op
      if (m_flt_pulse) begin
         xtq_kind[m_rob_idx]  <= SYK_XTRAP;
         xtq_cause[m_rob_idx] <= lsu_fault_cause;
         xtq_tval[m_rob_idx]  <= lsu_fault_tval;
      end
      // flush arm last (rule I11): a redirect fires at the head, every live entry is younger
      if (redirect) for (sqi = 0; sqi < ROB_DEPTH; sqi = sqi + 1) xtq_kind[sqi] <= SYK_NONE;
   end
   always @(posedge clk) if (!reset) begin
      if (xtrap_v & m_done_red) begin
         if (xtq_kind[m_rob_idx] != SYK_XTRAP)
            $fatal(1, "sysq shadow: M traps rob %0d (pc %h cause %0d) but the entry's kind is %0d",
                   m_rob_idx, m_pc, xtrap_cause, xtq_kind[m_rob_idx]);
         if (xtq_cause[m_rob_idx] != xtrap_cause || xtq_tval[m_rob_idx] != xtrap_tval
             || xtq_pc[m_rob_idx] != m_pc)
            $fatal(1, "sysq shadow: trap payload differs for rob %0d pc %h/%h: cause %0d/%0d tval %h/%h",
                   m_rob_idx, xtq_pc[m_rob_idx], m_pc, xtq_cause[m_rob_idx], xtrap_cause,
                   xtq_tval[m_rob_idx], xtrap_tval);
      end
   end

   // ---- trap / redirect ----
   wire m_trap = xtrap_v;                 // a system op's trap is the SYSQ's (sy_trap), since C3 step 3
   wire csr_red = xtrap_v;
   // THE REDIRECT DOES NOT CARRY THE LSU'S LIVE COMPLETION. A memory op redirects only as a
   // trap, and a trap is taken from the latched copy (m_unit_done_q); a branch, a system op
   // and fence.i never go through the LSU. So the redirect's "done" is m_done with the
   // memory arm of m_unit_ok removed -- logically the same signal on every cycle a redirect
   // can fire, asserted below, and structurally free of dTLB -> lsu_done -> m_unit_ok, which
   // was the head of the u_sq/v_reg -> fe/q_dat family (326 endpoints, 26 levels).
   wire m_unit_ok_nomem = ~m_valid            ? 1'b1
                        : m_fault | m_ill_eff ? 1'b1
                        : m_mem_op            ? 1'b0
                        :                       1'b1;
   assign m_done_red = (m_unit_ok_nomem | m_unit_done_q) & ~head_block & ~port_yield;
   // M's ORDERED redirect: CSR write, fence.i, or a trap (csr_red carries xtrap_v). Fires only
   // at the ROB head (m_done_red gates on ~head_block). Branches left M, so m_redirect is 0 here
   // now; the branch squash comes from the CTF pipe (cf_red_fire).
   wire m_red_fire = m_valid & m_done_red & (csr_red | m_is_fencei);
   wire m_red_ref  = m_valid & m_done     & (csr_red | m_is_fencei);
   // The branch squash: the tracked mispredict has reached the ROB head. The head is unique, so
   // m_red_fire and cf_red_fire are mutually exclusive.
   // THE SQUASH WAITS FOR THE LINK IT OWES. A mispredicting jal/jalr resolves, early-restarts
   // the frontend (fr_set) and stays in the CTF stage until its link is written -- and the link
   // waits for the FE shard's write port whenever the FPU is landing (cf_link_wb = ~fp_wb). If the
   // branch reaches the ROB head first, the squash below fired anyway, `redirect` cleared the
   // stage (the reset arm above), and the link was never written: the branch retired with its
   // rd's physreg holding whatever it held before -- zero, an old FP value, a stale sp -- and the
   // next `ret` jumped there. Geekbench 6 PDF Renderer (virtual calls in FP-dense code) died
   // that way on every IW=3 bitstream, ~90 min in, with epc == ra == 0 (2026-09-20/21);
   // memrand --fpmix reproduces it in ~1 M ops at either width. cf_link_pend is registers only
   // (cf_valid, cf_rd_v, cf_link_wrote), so the fire's cone gains one AND. The wait is bounded:
   // the head does not retire, the ROB fills behind it, dispatch stops, the FPU drains, the
   // port frees. The stage holds the tracked branch exactly while its link is pending (a
   // written link advances it out), so a pending link in the stage is the tracked branch's own
   // or a younger wrong-path one -- either way the fire waits, never the reverse.
   wire cf_red_fire = fr_v & (rob_head_idx == fr_rob) & ~cf_link_pend;
   assign redirect = m_red_fire | sy_red | cf_red_fire;
   always @(posedge clk) if (!reset) begin
      if (m_red_fire != m_red_ref)
         $fatal(1, "ooo2_core: M redirect from the non-memory done disagrees with m_done (%b vs %b)",
                m_red_fire, m_red_ref);
      if ((xtrap_v & m_done_red) != (xtrap_v & m_done))
         $fatal(1, "ooo2_core: trap request from the non-memory done disagrees with m_done");
      if ((m_is_sys & m_done_red) != (m_is_sys & m_done))
         $fatal(1, "ooo2_core: CSR update valid from the non-memory done disagrees with m_done");
   end

   // ---- EARLY FRONTEND RESTART -------------------------------------------------------
   // On a mispredict, do NOT wait to become ROB head before refetching. Note the event,
   // flush the frontend, freeze the renamer, and start fetching the resolved target now;
   // the ROB drains behind us. When the branch reaches the head the squash runs and the
   // renamer is released -- onto a correct path that is already in the decoupling queue.
   //
   // Only a MISPREDICT can do this: its target (m_target) is resolved in execute. A trap's
   // target comes out of csr_file only once the op is at head, so traps keep the late path.
   //
   // The renamer MUST freeze for the whole window. Rollback here is `h := hc` with no
   // snapshot (ooo2_rename), so anything renamed before the squash is undone by it --
   // renaming ahead would not merely waste work, it would lose the instructions. Frozen,
   // the correct path accumulates in the decoupling queue, which the squash does not touch.
   //
   // fr_v is the "frozen by an older redirect" interlock: a mispredict that resolves while one
   // is pending must not retarget the frontend unless it is OLDER. Control flow resolves out of
   // order now (parallel pipe), so "first resolved is oldest" no longer holds -- this is doc 12.
   // We TRACK THE SEQNO OF THE OLDEST RESTART and ignore younger ones: a resolving mispredict
   // (re)starts the frontend only if it is older (smaller seqno, wrap-safe) than the pending one,
   // and the backend squash fires when the tracked branch reaches head (cf_red_fire above).
   //
   // Measured motivation: FE_BUB per redirect went 9.6 -> 54.4 cycles when the window
   // grew from ~2 instructions to 16, while mispredicts fell 37% (docs/OOO2-Spec.md).
   wire        cf_mis   = cf_valid & cf_redirect;                    // a resolved mispredicting branch
   wire        cf_older = ~fr_v | ($signed(cf_seq - fr_seq) < 0);    // wrap-safe: the oldest restart wins
   assign fr_set    = cf_mis & cf_older & ~redirect;
   always @(posedge clk) begin
      if (reset)         fr_v <= 1'b0;
      else if (redirect) fr_v <= 1'b0;      // the squash consumes it
      else if (fr_set)   begin fr_v <= 1'b1; fr_seq <= cf_seq; fr_rob <= cf_rob; end
   end
   // The tracked restart's CTI trains the predictor before its squash fires (rule D16): in its
   // resolve cycle, or for a jal/jalr once its link is written.
   reg  fr_trn;
   wire fr_trn_now = res_v & (cf_seq == fr_seq);
   initial fr_trn = 1'b0;
   always @(posedge clk) begin
      if (reset | redirect)          fr_trn <= 1'b0;
      else if (fr_set)               fr_trn <= res_v;
      else if (fr_v & fr_trn_now)    fr_trn <= 1'b1;
      if (!reset && cf_red_fire && !(fr_trn | fr_trn_now))
         $fatal(1, "ooo2_core: squash of rob %0d (seq %0d) fires but its CTI never trained the predictor",
                fr_rob, fr_seq);
   end

   // Exactly one frontend flush per event. Re-flushing at the squash would discard the
   // correct path this whole mechanism exists to have fetched early.
   // dec_red is the decode-stage direct-CTI resteer. Backend wins (older instruction): a
   // co-firing fr_set/redirect flushes the dispatch stage anyway, and dec_red requires
   // d_take (~redirect_q & ~fr_v), so it never fires under a live freeze.
   // m_red_fire is at head (oldest) and ALWAYS flushes the frontend -- it overrides a younger
   // branch that already early-restarted (which the parallel branch pipe now makes possible),
   // and clears fr_v below. A branch's own squash (cf_red_fire) does NOT re-flush: its fr_set
   // already did. Exactly one frontend flush per event.
   assign fe_red_pulse = m_red_fire | sy_red | fr_set | dec_red;
   assign fe_red_tgt   = (m_red_fire | sy_red) ? redirect_target : fr_set ? cf_target     : dec_red_tgt;
   assign fe_red_seq   = (m_red_fire | sy_red) ? redirect_seq    : fr_set ? (cf_seq + 1'b1) : dec_red_seq;
   assign redirect_target  = (csr_red | sy_red) ? csr_redir_tgt
                           :           (m_pc + (m_rvc ? 64'd2 : 64'd4));   // fence.i
   assign redirect_is_trap = (m_red_fire & m_trap) | sy_trap;
   assign redirect_seq     = sy_red ? (sy_trap ? sy_seq : (sy_seq + 1'b1)) : m_trap ? m_seq : (m_seq + 1'b1);
   assign ifence           = m_valid & m_done & m_is_fencei;

   // ---- branch resolve / BTB training (from the CTF pipe, cf_*) ----
   // Train exactly once per CTI, as it leaves the CTF stage (cf_done), mispredicted or not: the
   // res_* fields below are the stage's own occupant, so the pulse must fire while the CTI is
   // still in it. A mispredict's squash (cf_red_fire) comes later, after the stage has freed and
   // holds another instruction, so cf_land cannot be the training pulse. res_pc_q / res_pdet_q
   // carry the CTI's PC and predictor snapshot to u_bp one cycle later, in step with the
   // frontend redirect.
   // A CTI younger than a pending restart (fr_v) is on the wrong path the restart already left:
   // it resolves on operands that path computed, and it trains nothing. (Wrong-path CTIs that
   // resolve before the older mispredict does still train: control flow resolves out of order.)
   wire   cf_wp     = fr_v & ($signed(cf_seq - fr_seq) > 0);
   assign res_v     = cf_done & (cf_is_branch | cf_is_jump) & ~cf_wp;
   assign res_cbr   = cf_is_branch;
   assign res_call  = cf_is_jump & cf_link_rd;
   assign res_ret   = cf_is_jalr & cf_link_rs & ~cf_link_rd;
   assign res_taken = cf_taken;
   assign res_tgt   = cf_taken_tgt;

   // ---- writeback ----
   // FMAX: split so the BYPASS source excludes csr_rdata. Every CSR op is serializing
   // (decode_exec.v:158 sets is_serialize on CSRRW/S/C), and `ser_block` holds the
   // frontend while one is in M -- so X is empty for the whole time a CSR result is the
   // writeback value, and that result can only ever be read back from the register file
   // by a later instruction. Bypassing it was unreachable logic, and it cost the ALU's
   // operand cone the entire CSR read mux, addressed by m_imm[11:0]:
   //   m_imm[11:0] -> u_csr read mux -> csr_rdata -> m_wb_val -> x_rs1 -> exec
   //               -> x_result / x_target
   // which was the second-worst family at 6 ns (-0.740, 19-32 levels). The invariant
   // that makes this sound is asserted below.
   assign m_byp_val = m_unit_done_q ? m_unit_res_q : m_unit_res;
   assign m_wb_val = m_byp_val;           // no CSR op reaches M since C3 step 3

   // Per-shard write data: each shard sees only its own writer, so the D$ read data reaches
   // the 3 load-shard LUTRAM copies instead of all 9, and the ALU result never leaves
   // int-exec.  Sourced directly, not from the m_wb_val mux -- routing every result through
   // one bus and then to every array is exactly what sharding by writer exists to avoid.
   assign wb_ie = xa_result;                      // one writer: the ALU, on its own port
   wire [63:0] wb_ie2 = xb_result;                // ...and the second ALU's shard, its own writer
   wire [63:0] wb_ie3 = xc_result;                // ...and the third ALU's (Stage 3)
   // SH_LD is now M's shard outright, so this mux carries every result M produces, not
   // just the memory and mul/div ones it started as. The two new arms are the two classes
   // d_shard just moved out of SH_IE: a CSR read, and everything else M completes -- which
   // after the mem/amo/mul arms above is a jump's link register.
   // A CSR READ TAKES THE LIVE csr_rdata, NEVER THE LATCH: m_unit_res has no CSR arm, and a
   // CSR op now always sits at head for a cycle before completing (m_head_q), which sets
   // m_unit_done_q -- the latched arm then handed a stale m_result to the destination. (The
   // same hazard existed whenever a landing load held a CSR op at head.) Live is also what
   // makes the delayed minstret read exact: it is sampled in the completion cycle.
   assign wb_ld = ld_wb                     ? lsu_rd_val
                : sy_wr                     ? csr_rdata    // before M's arms: M is EMPTY when the SYSQ
                : (m_is_mem | m_is_amo)     ? lsu_rd_val   // fires, and its m_is_* are the last op's, stale
                : m_unit_done_q             ? m_unit_res_q
                :                             m_result;
   // SH_FE's writers: the FPU, the CTF link (when the FPU isn't landing) and M's in-core FP ops
   // (M yields the cycle to the link: m_fe_yield).
   assign wb_fe = fp_wb         ? fp_wval
                : cf_link_wb    ? cf_link
                : md_wr         ? md_res_q        // a mul/div result (C1)
                : m_unit_done_q ? m_unit_res_q
                :                 fp_incore_res;
   // Two writers now: M's own completion, and a load landing after M has moved on. They can
   // never coincide -- m_done is forced low on ld_land above -- so the single PRF write
   // address still holds and ooo2_prf keeps its one-write-per-cycle property.
   //
   // THE WRITEBACK VALID IS BUILT FROM THE TERMS THAT CAN ACTUALLY WRITE, not from m_done.
   // we_ld/we_fe are the wakeup broadcast: every scheduler entry compares against them, the
   // pick follows, the source-tag read follows that. m_done carries the LSU's whole
   // completion -- the dTLB compare through xo_ok, the live fault -- and none of it can
   // ever produce a register write from M: a translate-only load is m_ld_nb, a store has
   // no rd, a faulting op traps. The only memory completion that writes a register is an
   // access this stage started (an AMO, LR/SC, a blocking load), which is lsu_done_acc.
   // The trap qualifier likewise takes the latched fault. m_wb_ref below is the old
   // expression, kept only for the assertion that the two never differ.
   wire m_unit_ok_wb = ~m_valid            ? 1'b1
                     : m_fault | m_ill_eff ? 1'b1
                     : m_mem_op            ? lsu_done_acc
                     :                       1'b1;
   wire m_done_wb = (m_unit_ok_wb | m_unit_done_q) & ~head_block & ~port_yield;
   wire m_trap_wb = (m_valid & (m_fault | m_ill_eff | (m_mem_op & m_lsu_flt)));
   wire m_wb  = m_valid & m_done_wb & m_rd_v & ~m_trap_wb & ~m_ld_nb & ~fp_arith;
   wire m_wb_ref = m_valid & m_done & m_rd_v & ~m_trap & ~m_ld_nb & ~fp_arith;
   always @(posedge clk) if (!reset && (m_wb != m_wb_ref))
      $fatal(1, "ooo2_core: writeback valid from the access-only done disagrees with m_done (%b vs %b)",
             m_wb, m_wb_ref);
   wire ld_wb = ld_land & lq_l_rd_v;

   // PER-SHARD WRITE PORTS. Each shard is driven by its OWN writers rather than through a
   // muxed address: IE by M's ALU/CSR result, LD by a landing load or M's mul/div, FE by a
   // landing FP result or M's in-core FP. Nothing changes yet -- m_done is still forced low
   // on ld_land/fp_land, so no shard sees two writers in a cycle, and the ROB's single
   // completion port remains the reason a cycle has to be yielded at all. What this buys is
   // the precondition: doc 7's "give each file its own write port and a single writer, and
   // the arbiter disappears" cannot even be attempted while one address is shared. The
   // one-writer-per-cycle property is asserted below rather than assumed.
   wire m_wb_ie = m_wb & (m_shard == SH_IE);
   wire m_wb_ld = m_wb & (m_shard == SH_LD);
   wire m_wb_fe = m_wb & (m_shard == SH_FE);
   // IE is written by M (a jump's link register, a CSR result) and by an ALU op completing
   // at issue. They cannot collide: unit_busy holds the ALU off in any cycle m_wb_ie is
   // set, which is asserted below.
   wire alu_wb = iss_alu & qa_rd_v;
   wire alu2_wb = iss_alu2 & qb_rd_v;
   wire we_ie2 = alu2_wb;
   wire [RN_PBITS-1:0] wa_ie2 = qb_prd;
   always @(posedge clk) begin
      alu2_q_v <= ~reset & alu2_wb;
      if (alu2_wb) begin alu2_q_prd <= qb_prd; alu2_q_val <= xb_result; end
   end
   wire alu3_wb = iss_alu3 & qc_rd_v;                    // the third ALU (Stage 3)
   wire we_ie3 = alu3_wb;
   wire [RN_PBITS-1:0] wa_ie3 = qc_prd;
   always @(posedge clk) begin
      alu3_q_v <= ~reset & alu3_wb;
      if (alu3_wb) begin alu3_q_prd <= qc_prd; alu3_q_val <= xc_result; end
   end
   wire we_ie = alu_wb;                    // the ISSUE-timed event: wake, pending clear, snoop
   always @(posedge clk) begin             // the write itself, a cycle later (see x_rs1)
      alu_q_v <= ~reset & alu_wb;
      if (alu_wb) begin alu_q_prd <= qa_prd; alu_q_val <= xa_result; end
   end
   wire we_ld = m_wb_ld | ld_wb | sy_wr;
   wire we_fe = m_wb_fe | fp_wb | cf_link_wb | md_wr;
   wire [RN_PBITS-1:0] wa_ie = qa_prd;
   wire [RN_PBITS-1:0] wa_ld = ld_wb ? lq_l_prd : sy_wr ? sy_prd : m_prd;
   wire [RN_PBITS-1:0] wa_fe = fp_wb ? ft_prd : cf_link_wb ? cf_prd : md_wr ? md_prd : m_prd;
   always @(posedge clk) if (!reset) begin
      if (m_wb_ld & ld_wb)
         $fatal(1, "ooo2_core: LD shard written by both M and a landing load");
      if (m_wb_fe & (fp_wb | cf_link_wb | md_wr))
         $fatal(1, "ooo2_core: FE shard written by M together with the FPU or the CTF link");
      // m_wb_ie is dead by construction: d_shard sends every M-routed op to SH_LD or
      // SH_FE. Asserted rather than assumed -- if a future op class reaches M with
      // SH_IE, it would silently drop its result now that we_ie ignores M.
      if (alu3_q_v)
         $fatal(1, "ooo2_core: the third ALU wrote back -- it is dead since the swizzle and SH_IE3 is tied off");
      if (m_wb_ie)
         $fatal(1, "ooo2_core: M wrote the IE shard -- d_shard must route M's ops to SH_LD");
   end

   // The architectural shadow is written AT COMMIT, in order. It has no rename, so it cannot
   // model out-of-order writeback: a younger instruction writes x13, then an older load lands
   // and clobbers the same architectural location. Driving it from the ROB head keeps it a
   // valid architectural model, which is what tb_ooo2_riscv's trace reads it as.
`ifndef SYNTHESIS
   assign rf_we = rob_c_valid & rob_c_rd_v;
   assign rf_wa = rob_c_rd;
   assign rf_wd = cs_val_h;
   assign rf_we2 = rob_c2_valid & rob_c2_rd_v;
   assign rf_wa2 = rob_c2_rd;
   assign rf_wd2 = cs_val_h2;
`else
   assign rf_we = 1'b0;  assign rf_wa = 6'd0;  assign rf_wd = 64'd0;
   assign rf_we2 = 1'b0; assign rf_wa2 = 6'd0; assign rf_wd2 = 64'd0;
`endif

   // RETIRE IS THE ROB HEAD, not the M stage. Not merely for the cosim: it drives minstret
   // through retire_cnt and csr_file's fp_dirty_commit, both architectural, and both of
   // which must count an instruction when it COMMITS rather than when it happens to finish.
   // With M still blocking the two coincide, which is what makes this step checkable.
   assign retire      = rob_c_valid & ~rob_c_noret;
   assign retire2     = rob_c2_valid & ~rob_c2_noret;
   assign retire3     = rob_c3_valid & ~rob_c3_noret;
   wire [ROB_IDXB-1:0] rob_head2_idx = rob_head_idx + 1'b1;
   wire [ROB_IDXB-1:0] rob_head3_idx = rob_head_idx + 2'd2;

   // retire_pc/retire_insn are verification payload -- tb_ooo2_riscv traces them and
   // rv_soc_top leaves both unconnected -- so they come from a simulation-only side array
   // rather than widening the ROB by 96 bits an entry. docs/Area-Efficient-Scalar-OoO.md 2:
   // the reorder buffer holds status, not data.
`ifndef SYNTHESIS
   reg [PCW-1:0] cs_pc   [0:ROB_DEPTH-1];
   reg [31:0]    cs_insn [0:ROB_DEPTH-1];
   always @(posedge clk) begin
      if (rn_valid)   begin cs_pc[rob_d_idx]  <= d_pc;  cs_insn[rob_d_idx]  <= d_insn;  end
      if (rn_valid_b) begin cs_pc[rob_d_idx2] <= d2_pc; cs_insn[rob_d_idx2] <= d2_insn; end
      if (rn_valid_c) begin cs_pc[rob_d_idx3] <= d3_pc; cs_insn[rob_d_idx3] <= d3_insn; end
   end
   assign retire_pc   = cs_pc[rob_head_idx];
   assign retire_insn = cs_insn[rob_head_idx];
   assign retire2_pc   = cs_pc[rob_head2_idx];
   assign retire2_insn = cs_insn[rob_head2_idx];

   // Values that are only known at COMPLETION, held per ROB slot until that slot commits.
   // Simulation-only, so the ROB stays status-only in hardware. The capture is keyed on M's
   // completion because M still blocks; when a load's data starts arriving after M has moved
   // on, this trigger is the one thing here that has to follow it to the writeback event.
   reg [63:0] cs_val   [0:ROB_DEPTH-1];
   reg [1:0]  cs_mkind [0:ROB_DEPTH-1];
   reg [55:0] cs_mpa   [0:ROB_DEPTH-1];
   reg [63:0] cs_mdata [0:ROB_DEPTH-1];   // a store's value and log2 size (4'hF: not data-checked),
   reg [3:0]  cs_msz   [0:ROB_DEPTH-1];   // for the cosim's byte-exact store check (B5, 2026-09-17)
   // Captured at the WRITEBACK event, which for a load is no longer M's cycle.
   always @(posedge clk) if (!reset) begin
      // On the completion PULSE, not on m_done: lsu_cos_* are only this op's while the LSU
      // still holds it, and m_done can now assert cycles later off the sticky latch.
      if (m_valid && m_unit_ok && !m_unit_done_q && !m_ld_nb && !fp_arith) begin
         cs_val[m_rob_idx]   <= m_wb_val;
      end
      // A CSR READ'S VALUE IS THE ONE AT ITS WRITE, not at the unit-ok pulse: the op
      // completes in its second cycle at head (m_head_q) and reads csr_rdata live then, so
      // `time`/`cycle` are one tick past the pulse's value. Ordered after the arm above so
      // it wins. (Found by the 300 M cosim: rdtime retired as t, computed with t+1.)
      if (sy_wr) cs_val[sy_rob] <= csr_rdata;
      if (m_valid && m_unit_ok && !m_unit_done_q && !m_ld_nb && !fp_arith) begin
         // A BUFFERED STORE HAS NO MEMORY EFFECT YET. Its M pass only translates, so
         // lsu_cos_* still hold the PREVIOUS access's values -- reporting them here would
         // hand the cosim a stale PA under this store's seqno. Worse, it would often be
         // kind 0, and probe_cosim.cpp deliberately SKIPS the address compare whenever
         // either side reports no access ("a model that reports no access never forces a
         // false abort"), so the mistake would be invisible rather than loud: every
         // buffered store would go unchecked. Captured at commit instead, below.
         cs_mkind[m_rob_idx] <= (m_mem_op & ~m_st_nb) ? lsu_cos_kind : 2'd0;
         cs_mpa[m_rob_idx]   <= m_st_nb ? 56'd0 : lsu_cos_pa;
         cs_mdata[m_rob_idx] <= m_st_nb ? 64'd0 : lsu_cos_data;
         cs_msz[m_rob_idx]   <= m_st_nb ? 4'hF  : lsu_cos_size;
      end
      // The buffered store's real memory effect, recorded when the ROB RELEASES it: from
      // then on the store may retire before the LSU drains it, so the queue's own PA is
      // the source, not lsu_cos_* (which would name whatever the port did last).
      if (sq_k_take) begin
         cs_mkind[sq_kc_rob] <= 2'd2;
         cs_mpa[sq_kc_rob]   <= sq_kc_addr;
         cs_mdata[sq_kc_rob] <= sq_kc_data;
         cs_msz[sq_kc_rob]   <= {2'b0, sq_kc_size};
      end
      // A LANDING LOAD'S EFFECT COMES FROM THE ENTRY THAT OWNS IT. This read lsu_cos_*,
      // justified as "they are still this load's values when it lands: the LSU is
      // single-outstanding, so nothing else can have started in between". b9dbdd0 starts a
      // queued load's access in its TRANSLATE pass, so a store does start in between, and the
      // load then committed carrying the store's kind -- with the store's PA too, which looked
      // right precisely when it was most wrong, because a store the load reads back has the
      // same address. docs/rtl-rules.md: matched by a tag the requester allocated, never by
      // "only one in flight". The load queue entry IS that tag, and it holds the load's own
      // PA; the kind is a load by construction, because only loads are queued here.
      if (ld_land) begin
         cs_val[lq_l_rob]   <= lsu_rd_val;
         cs_mkind[lq_l_rob] <= 2'd1;
         cs_mpa[lq_l_rob]   <= lq_l_pa;
      end
      if (md_wb) begin
         cs_val[md_rob]   <= md_res_q;
         cs_mkind[md_rob] <= 2'd0;      // a mul/div has no memory effect
      end
      if (fp_land) begin
         cs_val[ft_rob]   <= fp_wval;
         cs_mkind[ft_rob] <= 2'd0;      // an FP op has no memory effect
         cs_mpa[ft_rob]   <= 56'd0;
      end
      // The CTF pipe's link write (jal/jalr rd). Captured at the link writeback (cf_link_wb),
      // which the link-pend gap guarantees is at least a cycle before the branch retires, so
      // cs_val[cf_rob] holds the link when the head reads it. No mem effect.
      if (cf_link_wb) begin
         cs_val[cf_rob]   <= cf_link;
         cs_mkind[cf_rob] <= 2'd0;
         cs_mpa[cf_rob]   <= 56'd0;
      end
      if (iss_alu2) begin               // the second ALU port
         cs_val[a2_rob]   <= xb_result;
         cs_mkind[a2_rob] <= 2'd0;
         cs_mpa[a2_rob]   <= 56'd0;
      end
      if (iss_alu3) begin               // the third ALU port (Stage 3)
         cs_val[a3_rob]   <= xc_result;
         cs_mkind[a3_rob] <= 2'd0;
         cs_mpa[a3_rob]   <= 56'd0;
      end
      if (iss_alu) begin                // completed at issue on the ALU port, never saw M
         cs_val[a_rob]   <= xa_result;
         cs_mkind[a_rob] <= 2'd0;
         cs_mpa[a_rob]   <= 56'd0;
      end
   end
   // ...and the same write-forward the ROB's head_done needs, for the same reason: a slot can
   // be captured in the very cycle it commits, so the array read returns pre-edge contents.
   // Missing it reported rd=0 for the second instruction of the boot.
   wire        cs_hit_m   = m_valid & m_unit_ok & ~m_unit_done_q & ~m_ld_nb & ~fp_arith
                          & (m_rob_idx == rob_head_idx);
   // A CSR op completes (and retires, by the ROB's same-cycle bypass) in its SECOND cycle
   // at head, reading csr_rdata live then -- after cs_hit_m's pulse and before the capture
   // above lands. `time`/`cycle` differ by a tick between the two: the head takes it live.
   wire        cs_hit_csr = sy_wr & (sy_rob == rob_head_idx);
   wire        cs_hit_ld  = ld_land & (lq_l_rob == rob_head_idx);
   wire        cs_hit_sq  = sq_k_take & (sq_kc_rob == rob_head_idx);
   wire        cs_hit_fp  = fp_land & (ft_rob == rob_head_idx);
   wire        cs_hit_md  = md_wb & (md_rob == rob_head_idx);
   // The CTF link can write in the very cycle its mispredicted, red-firing branch retires (the
   // "link-pend gap" comment at the capture assumes otherwise), so the head bypasses it like the
   // rest (2026-09-21: four `memrand --fpmix` seeds read a stale cs_val for a jalr's link).
   wire        cs_hit_cf  = cf_link_wb & (cf_rob == rob_head_idx);
   wire        cs_hit_alu = iss_alu & (a_rob == rob_head_idx);
   wire        cs_hit_alu2 = iss_alu2 & (a2_rob == rob_head_idx);
   wire        cs_hit_alu3 = iss_alu3 & (a3_rob == rob_head_idx);
   wire [63:0] cs_val_h   = cs_hit_csr ? csr_rdata     // the CSR read's value at its write
                          : cs_hit_sq ? 64'd0          // a store writes no register
                          : cs_hit_ld ? lsu_rd_val
                          : cs_hit_alu ? xa_result
                          : cs_hit_alu2 ? xb_result
                          : cs_hit_alu3 ? xc_result
                          : cs_hit_md ? md_res_q : cs_hit_fp ? fp_wval : cs_hit_cf ? cf_link
                          : cs_hit_m  ? m_wb_val : cs_val[rob_head_idx];
   wire [1:0]  cs_mkind_h = cs_hit_sq ? 2'd2
                          : cs_hit_ld ? 2'd1
                          : cs_hit_alu ? 2'd0
                          : cs_hit_alu2 ? 2'd0
                          : cs_hit_alu3 ? 2'd0
                          : cs_hit_md ? 2'd0 : cs_hit_fp ? 2'd0
                          : cs_hit_m  ? (m_mem_op ? lsu_cos_kind : 2'd0) : cs_mkind[rob_head_idx];
   wire [55:0] cs_mpa_h   = cs_hit_sq ? sq_kc_addr
                          : cs_hit_ld ? lq_l_pa
                          : cs_hit_alu ? 56'd0
                          : cs_hit_alu2 ? 56'd0
                          : cs_hit_alu3 ? 56'd0
                          : cs_hit_md ? 56'd0 : cs_hit_fp ? 56'd0
                          : cs_hit_m  ? lsu_cos_pa : cs_mpa[rob_head_idx];
   wire [63:0] cs_mdata_h = cs_hit_sq ? sq_kc_data : cs_hit_m ? lsu_cos_data : cs_mdata[rob_head_idx];
   wire [3:0]  cs_msz_h   = cs_hit_sq ? {2'b0, sq_kc_size} : cs_hit_m ? lsu_cos_size : cs_msz[rob_head_idx];
   // ...and for the entry behind the head, retiring in the same cycle (item 10c)
   wire        cs2_hit_m   = m_valid & m_unit_ok & ~m_unit_done_q & ~m_ld_nb & ~fp_arith & (m_rob_idx == rob_head2_idx);
   wire        cs2_hit_ld  = ld_land & (lq_l_rob == rob_head2_idx);
   wire        cs2_hit_sq  = sq_k_take & (sq_kc_rob == rob_head2_idx);
   wire        cs2_hit_fp  = fp_land & (ft_rob == rob_head2_idx);
   wire        cs2_hit_md  = md_wb & (md_rob == rob_head2_idx);
   wire        cs2_hit_alu = iss_alu & (a_rob == rob_head2_idx);
   wire        cs2_hit_alu2 = iss_alu2 & (a2_rob == rob_head2_idx);
   wire        cs2_hit_alu3 = iss_alu3 & (a3_rob == rob_head2_idx);
   wire [63:0] cs_val_h2   = cs2_hit_sq ? 64'd0 : cs2_hit_ld ? lsu_rd_val : cs2_hit_alu ? xa_result : cs2_hit_alu2 ? xb_result
                           : cs2_hit_alu3 ? xc_result
                           : cs2_hit_md ? md_res_q : cs2_hit_fp ? fp_wval : cs2_hit_m ? m_wb_val : cs_val[rob_head2_idx];
   wire [1:0]  cs_mkind_h2 = cs2_hit_sq ? 2'd2 : cs2_hit_ld ? 2'd1 : cs2_hit_alu ? 2'd0 : cs2_hit_alu2 ? 2'd0 : cs2_hit_alu3 ? 2'd0 : cs2_hit_md ? 2'd0 : cs2_hit_fp ? 2'd0
                           : cs2_hit_m ? (m_mem_op ? lsu_cos_kind : 2'd0) : cs_mkind[rob_head2_idx];
   wire [55:0] cs_mpa_h2   = cs2_hit_sq ? sq_kc_addr : cs2_hit_ld ? lq_l_pa : cs2_hit_alu ? 56'd0 : cs2_hit_alu2 ? 56'd0 : cs2_hit_alu3 ? 56'd0 : cs2_hit_md ? 56'd0 : cs2_hit_fp ? 56'd0
                           : cs2_hit_m ? lsu_cos_pa : cs_mpa[rob_head2_idx];
   wire [63:0] cs_mdata_h2 = cs2_hit_sq ? sq_kc_data : cs2_hit_m ? lsu_cos_data : cs_mdata[rob_head2_idx];
   wire [3:0]  cs_msz_h2   = cs2_hit_sq ? {2'b0, sq_kc_size} : cs2_hit_m ? lsu_cos_size : cs_msz[rob_head2_idx];
   // ...and the third entry, retiring in the same cycle (Stage 3, IW=3)
   wire        cs3_hit_m   = m_valid & m_unit_ok & ~m_unit_done_q & ~m_ld_nb & ~fp_arith & (m_rob_idx == rob_head3_idx);
   wire        cs3_hit_ld  = ld_land & (lq_l_rob == rob_head3_idx);
   wire        cs3_hit_sq  = sq_k_take & (sq_kc_rob == rob_head3_idx);
   wire        cs3_hit_fp  = fp_land & (ft_rob == rob_head3_idx);
   wire        cs3_hit_md  = md_wb & (md_rob == rob_head3_idx);
   wire        cs3_hit_alu = iss_alu & (a_rob == rob_head3_idx);
   wire        cs3_hit_alu2 = iss_alu2 & (a2_rob == rob_head3_idx);
   wire        cs3_hit_alu3 = iss_alu3 & (a3_rob == rob_head3_idx);
   wire [63:0] cs_val_h3   = cs3_hit_sq ? 64'd0 : cs3_hit_ld ? lsu_rd_val : cs3_hit_alu ? xa_result : cs3_hit_alu2 ? xb_result
                           : cs3_hit_alu3 ? xc_result
                           : cs3_hit_md ? md_res_q : cs3_hit_fp ? fp_wval : cs3_hit_m ? m_wb_val : cs_val[rob_head3_idx];
   wire [1:0]  cs_mkind_h3 = cs3_hit_sq ? 2'd2 : cs3_hit_ld ? 2'd1 : cs3_hit_alu ? 2'd0 : cs3_hit_alu2 ? 2'd0 : cs3_hit_alu3 ? 2'd0 : cs3_hit_md ? 2'd0 : cs3_hit_fp ? 2'd0
                           : cs3_hit_m ? (m_mem_op ? lsu_cos_kind : 2'd0) : cs_mkind[rob_head3_idx];
   wire [55:0] cs_mpa_h3   = cs3_hit_sq ? sq_kc_addr : cs3_hit_ld ? lq_l_pa : cs3_hit_alu ? 56'd0 : cs3_hit_alu2 ? 56'd0 : cs3_hit_alu3 ? 56'd0 : cs3_hit_md ? 56'd0 : cs3_hit_fp ? 56'd0
                           : cs3_hit_m ? lsu_cos_pa : cs_mpa[rob_head3_idx];
   wire [63:0] cs_mdata_h3 = cs3_hit_sq ? sq_kc_data : cs3_hit_m ? lsu_cos_data : cs_mdata[rob_head3_idx];
   wire [3:0]  cs_msz_h3   = cs3_hit_sq ? {2'b0, sq_kc_size} : cs3_hit_m ? lsu_cos_size : cs_msz[rob_head3_idx];
`else
   assign retire_pc   = {PCW{1'b0}};
   assign retire_insn = 32'd0;
   assign retire2_pc   = {PCW{1'b0}};
   assign retire2_insn = 32'd0;
`endif

   // Nothing may sit in X while a serializing op is in M. This is what `ser_block`
   // exists to guarantee, and it is what makes the CSR result unbypassable above.
   always @(posedge clk)
     if (!reset && (rn_valid | rn_valid_b) && m_valid && m_is_serialize)
       $fatal(1, "ooo2_core: dispatched behind a serializing op in M (pc=%h)", m_pc);

   // ---- interrupt injection: a solo SYSTEM pseudo-op that traps in M ----
   // FMAX: REGISTERED, for the same reason redirect_q is (see the note above the
   // redirect_q declaration). irq_inject is a select in fetch's npc mux (fetch.v:168),
   // and npc is the only combinational input to the BTB read register
   // (ooo2_predictor.v:250) -- so while this was a wire it was the ONE path by which the
   // backend reached the fetch PC, and it dragged in everything `redirect` depends on:
   //   m_rs1_val -> csr next-state -> csr_writes -> do_satp/do_fschg -> csr_redir_v
   //             -> csr_red -> redirect -> irq_inject -> u_fetch/npc -> u_bp/btb_q
   // and, through redirect's m_done term, lsu_done as well. That is both the worst
   // path at 6 ns and the "tail everything shares". With this flop, NOTHING
   // combinational from M reaches the frontend's PC or predictor: `accept`/`consume`
   // remain, and they feed only the queue pop and the IR register's clock enable.
   //
   // Cost is one cycle of interrupt latency, which nothing observes -- csr_irq_v is a
   // level, so the injection simply happens a cycle later.
   //
   // THE INTERLOCK IS KEYED TO `irq_taken`, NOT `accept`. 49eecf48 gave this module's
   // frontend an decoupling queue and changed fetch's ready from `accept` to `~q_full`, so the
   // pseudo-op is consumed on the queue PUSH while inject_inflight was still armed by the
   // queue POP. Whenever the backend stalled -- accept low, queue not full -- fetch
   // re-emitted the SAME interrupt every cycle, because the only thing that would have
   // stopped it was waiting for an event that had stopped coinciding. That bitstream hung
   // the board the moment Linux enabled its first PLIC source, and no simulation gate
   // could see it: cosim locksteps retired instruction RESULTS, and this is a duplicated
   // trap, not a wrong value. Found by hardware bisect (49eecf48 BAD, parent 9c298425
   // GOOD), 2026-08-20.
   reg inject_inflight, irq_inject_q;
   wire irq_taken;
   initial begin inject_inflight = 1'b0; irq_inject_q = 1'b0; end
   assign irq_inject = irq_inject_q;
   always @(posedge clk) begin
      if (reset) begin
         inject_inflight <= 1'b0;
         irq_inject_q    <= 1'b0;
      end else begin
         // Present it until fetch actually TAKES it, then latch the interlock on that
         // same event. Scheduling and holding are separated so one interrupt can never be
         // presented twice while the interlock is still catching up.
         // Registered redirect terms only (rule I11, 2026-09-05): the live redirect and fr_set
         // are M's completion, and this register's input cone fanned out into the whole fetch
         // cycle (gate V5's worst family started here). A pseudo-op presented in the redirect
         // cycle is taken into a queue the redirect empties at the same edge; inject_inflight
         // is cleared by redirect_q the cycle after, and the interrupt, still pending, is
         // presented again.
         irq_inject_q <= ~redirect_q & ~fr_v &
                         (irq_inject_q ? ~irq_taken                     // hold until taken
                                       : csr_irq_v & ~inject_inflight); // schedule

         if (irq_taken)                               inject_inflight <= 1'b1;
         else if (redirect_q | fr_v | ~csr_irq_v)     inject_inflight <= 1'b0;
      end
   end

   // At most one interrupt pseudo-op may be in flight: presenting one while another is
   // already accepted would inject two traps for one interrupt.
   always @(posedge clk)
     if (!reset && irq_inject_q && inject_inflight)
       $fatal(1, "ooo2_core: interrupt injection presented while one is already in flight");

   // ...and fetch may never consume two. This is the invariant 49eecf48 broke; it is
   // checked directly now instead of being implied by a handshake that stopped holding.
   reg irq_taken_q;
   initial irq_taken_q = 1'b0;
   always @(posedge clk) begin
      irq_taken_q <= reset ? 1'b0 : irq_taken;
      if (!reset && irq_taken && irq_taken_q)
        $fatal(1, "ooo2_core: interrupt pseudo-op consumed twice for one interrupt");
   end

`ifdef OOO2_COSIM
   // ======================= cosim retire stream (VERIFY-ONLY) =======================
   // Hand every retiring instruction (and every trap) to simmerv via probe_retire(),
   // the DPI contract src/probe_cosim.cpp implements.
   //
   // The OoO core needs ~250 lines here: a 40-deep FIFO to rebuild program order from
   // out-of-order commit, per-entry value-ready tracking (an ALU op commits on the
   // issue count, before its writeback), squash-by-seqno tail truncation, and a
   // by-PC search to convert a committed entry into its own trap. In-order needs
   // NONE of it -- M retires one instruction per cycle in program order, and its rd
   // value is live on the writeback bus in that very cycle.
   //
   // The one thing that IS needed is the same 1-cycle lag the OoO harness uses: the
   // retiring instruction's CSR/regfile writes land on the edge that ends its M
   // cycle, so `mepc` is read LIVE one cycle later (probe_cosim wants mepc AFTER the
   // retire -- e.g. after an mret, or after a trap has captured it). Everything that
   // a trap changes at that same edge -- privilege above all -- must therefore be
   // REGISTERED at retire time, not read live.
   reg [1:0]  e_mkind;   reg [55:0] e_mpa;      // cosim memory-effect capture
   reg [63:0] e_mdata;   reg [3:0]  e_msz;
   import "DPI-C" function void probe_retire(
      input longint unsigned pc,
      input int     unsigned insn,
      input byte    unsigned rd_kind,     // 0 none, 1 int, 2 fp
      input byte    unsigned rd_idx,
      input byte    unsigned prv,
      input byte    unsigned trapped,
      input longint unsigned rd_val,
      input longint unsigned trap_cause,
      input longint unsigned trap_tval,
      input longint unsigned mtime_v,
      input longint unsigned mtimecmp_v,
      input longint unsigned mepc_v,
      input byte    unsigned seip_v,
      // memory effect: 0 none / 1 load / 2 store, and the EXACT physical address.
      // The reference reports the same, so a store landing at the wrong PA -- which has
      // no architectural result and is otherwise invisible -- aborts at the store.
      input byte    unsigned mem_kind,
      input longint unsigned mem_pa,
      input longint unsigned mem_data,     // a plain store's value (raw rs2) and log2 size; 0xF = not data-checked
      input byte    unsigned mem_size);

   // csr_file internals, tapped exactly as backend_top does
   wire        cot_fire  = u_csr.trap_v;
   wire        cot_take  = cot_fire & ((m_valid & m_done) | sy_fire);   // the cycle a trap is taken
   wire [63:0] cot_cause = u_csr.trap_cause;
   wire [63:0] cot_tval  = u_csr.trap_tval;
   wire        cot_intr  = cot_cause[63];
   // an instruction-side fault retires no instruction (insn=0), like an interrupt
   wire        cot_ifault = ~cot_intr & ((cot_cause[5:0]==6'd0) | (cot_cause[5:0]==6'd1)
                                       | (cot_cause[5:0]==6'd12));

   // Destination class comes from the COMMITTING entry, not from M -- the ROB already
   // carries rd/rd_v, so this needs no side array.
   wire [1:0] ck_rk = ~rob_c_rd_v ? 2'd0 : rob_c_rd[5] ? 2'd2 : 2'd1;
   wire [1:0] ck_rk2 = ~rob_c2_rd_v ? 2'd0 : rob_c2_rd[5] ? 2'd2 : 2'd1;
   wire [1:0] ck_rk3 = ~rob_c3_rd_v ? 2'd0 : rob_c3_rd[5] ? 2'd2 : 2'd1;
   reg        e_v, e_trap;
   reg [63:0] e_pc, e_val, e_cause, e_tval;
   reg [31:0] e_insn;
   reg [1:0]  e_rk, e_prv;
   reg [4:0]  e_ri;
   initial    e_v = 1'b0;
   // the second retire of the cycle (item 10c): its own record, handed over after the first
   reg        e2_v;
   reg [63:0] e2_pc, e2_val;
   reg [31:0] e2_insn;
   reg [1:0]  e2_rk, e2_mkind;
   reg [4:0]  e2_ri;
   reg [55:0] e2_mpa;    reg [63:0] e2_mdata;  reg [3:0] e2_msz;
   initial    e2_v = 1'b0;
   // the third retire of the cycle (Stage 3, IW=3): its own record
   reg        e3_v;
   reg [63:0] e3_pc, e3_val;
   reg [31:0] e3_insn;
   reg [1:0]  e3_rk, e3_mkind;
   reg [4:0]  e3_ri;
   reg [55:0] e3_mpa;    reg [63:0] e3_mdata;  reg [3:0] e3_msz;
   initial    e3_v = 1'b0;

   // A trap and a retire remain mutually exclusive, but they are no longer both "an M cycle":
   // the trap is M's (and fires only when M is the ROB head), the retire is the head's.
   always @(posedge clk) begin
      e_v <= 1'b0;
      if (!reset) begin
         if (cot_take) begin                     // M's trap, or the SYSQ's (C3 step 3)
            e_v <= 1'b1;  e_trap <= 1'b1;
            e_pc <= sy_fire ? sy_pc : m_pc;
            e_insn <= (cot_intr | cot_ifault) ? 32'd0 : sy_fire ? sy_insn : m_insn;
            e_rk <= 2'd0;  e_ri <= 5'd0;  e_val <= 64'd0;
            e_cause <= cot_cause;  e_tval <= cot_tval;
            e_prv <= u_csr.priv;                  // privilege BEFORE the trap
            e_mkind <= 2'd0;  e_mpa <= 56'd0;     // a trap performed no data access
            e_mdata <= 64'd0; e_msz <= 4'hF;
         end else if (retire) begin
            e_v <= 1'b1;  e_trap <= 1'b0;
            e_pc <= cs_pc[rob_head_idx];  e_insn <= cs_insn[rob_head_idx];
            e_rk <= ck_rk;  e_ri <= rob_c_rd[4:0];  e_val <= cs_val_h;
            e_cause <= 64'd0;  e_tval <= 64'd0;
            e_prv <= u_csr.priv;
            // Memory effect of the COMMITTING instruction, for the cosim's store/load check
            // -- captured when it completed, replayed when it commits.
            e_mkind <= cs_mkind_h;
            e_mpa   <= cs_mpa_h;
            e_mdata <= cs_mdata_h;  e_msz <= cs_msz_h;
         end
         e2_v <= 1'b0;
         if (retire2 && !cot_take) begin
            e2_v <= 1'b1;
            e2_pc <= cs_pc[rob_head2_idx];  e2_insn <= cs_insn[rob_head2_idx];
            e2_rk <= ck_rk2;  e2_ri <= rob_c2_rd[4:0];  e2_val <= cs_val_h2;
            e2_mkind <= cs_mkind_h2;  e2_mpa <= cs_mpa_h2;  e2_mdata <= cs_mdata_h2;  e2_msz <= cs_msz_h2;
         end
         e3_v <= 1'b0;
         if (retire3 && !cot_take) begin
            e3_v <= 1'b1;
            e3_pc <= cs_pc[rob_head3_idx];  e3_insn <= cs_insn[rob_head3_idx];
            e3_rk <= ck_rk3;  e3_ri <= rob_c3_rd[4:0];  e3_val <= cs_val_h3;
            e3_mkind <= cs_mkind_h3;  e3_mpa <= cs_mpa_h3;  e3_mdata <= cs_mdata_h3;  e3_msz <= cs_msz_h3;
         end
      end
      // emit one cycle later, so this instruction's own CSR writes have landed
      if (e_v)
         probe_retire(e_pc, e_insn, {6'd0, e_rk},
                      (e_rk == 2'd0) ? 8'd0 : {3'd0, e_ri},
                      {6'd0, e_prv}, {7'd0, e_trap}, e_val, e_cause, e_tval,
                      64'd0, {64{1'b1}}, `VA_UNPACK40(u_csr.mepc), 8'd0,
                      {6'd0, e_mkind}, {8'd0, e_mpa}, e_mdata, {4'd0, e_msz});
      if (e2_v)
         probe_retire(e2_pc, e2_insn, {6'd0, e2_rk},
                      (e2_rk == 2'd0) ? 8'd0 : {3'd0, e2_ri},
                      {6'd0, e_prv}, 8'd0, e2_val, 64'd0, 64'd0,
                      64'd0, {64{1'b1}}, `VA_UNPACK40(u_csr.mepc), 8'd0,
                      {6'd0, e2_mkind}, {8'd0, e2_mpa}, e2_mdata, {4'd0, e2_msz});
      if (e3_v)
         probe_retire(e3_pc, e3_insn, {6'd0, e3_rk},
                      (e3_rk == 2'd0) ? 8'd0 : {3'd0, e3_ri},
                      {6'd0, e_prv}, 8'd0, e3_val, 64'd0, 64'd0,
                      64'd0, {64{1'b1}}, `VA_UNPACK40(u_csr.mepc), 8'd0,
                      {6'd0, e3_mkind}, {8'd0, e3_mpa}, e3_mdata, {4'd0, e3_msz});
   end
`endif

   // =========================================================== flow control
   // Nothing follows a serializing op into X until it has left M, so CSR values,
   // privilege, satp and mstatus are never read stale.
   // SERIALIZATION, restated for a machine whose dispatch runs ahead. The old rule was
   // "nothing may sit in X while a serializing op is in M", which worked because X fed M
   // directly and emptied when it handed over. Dispatch is decoupled now, so the property
   // that actually matters is that a serializing op is ALONE IN FLIGHT: it may not dispatch
   // until the window has drained, and nothing may dispatch behind it until it has
   // committed. `drained` is the drain test, and it is exact -- nothing dispatches while
   // ser_inflight, so the window can only shrink. It is the ROB empty AND the store queue
   // empty: since the senior store queue (2026-09-04) a store retires when the ROB releases
   // it and the LSU drains it later, so an empty ROB no longer means its stores are in the
   // cache. A fence, fence.i, sfence.vma, AMO, CSR or trap op therefore waits for the queue
   // exactly as it did when every store drained at the head -- the ONE site for that
   // precondition (docs/rtl-rules.md C5).
   wire drained = rob_empty & (sq_occ == {(SQ_IB+1){1'b0}});
   reg  ser_inflight;
   initial ser_inflight = 1'b0;
   always @(posedge clk)
      if (reset | redirect)               ser_inflight <= 1'b0;
      else if (rn_valid & d_is_serialize) ser_inflight <= 1'b1;
      else if (drained)                   ser_inflight <= 1'b0;
   wire ser_block = ser_inflight | (d_valid & d_is_serialize & ~drained);

   // An instruction may not enter M while it reads the in-flight load's destination. Compared
   // as TAGS, not through a pending bit per physical register: NPHYS is 320, so a pending
   // vector would be three 320:1 muxes on the operand read -- the docs/rtl-rules.md I4 shape
   // deleted from the predictor, put back where there is no margin. Three 9-bit compares
   // instead, and the structure still works when there is more than one tag to check.
   // The pending tag must include the load DISPATCHING THIS CYCLE, not just one already
   // recorded: sb_busy is set at the edge, so in the dispatch cycle itself it is still 0 and
   // a dependent instruction would walk into M and take m_byp_val -- which for a load is
   // lsu_rd_val, before the data exists. That is the whole failure signature of the load and
   // store tests, and it is the same read-in-the-write-cycle shape as the ROB's head_done
   // and the cosim side array. Forward it here too.
   // One slot per long-latency unit, each comparing the SAME three renamed sources. The
   // dispatching-this-cycle term is in both: the slot register does not hold the tag until
   // the next edge, and a consumer right behind the producer would otherwise read a stale
   // physreg (the defect the load slot already paid for).
   // The per-unit tag interlock that lived here is gone with src_pend. Readiness is
   // ooo2_pending's business now, and WAITING is the scheduler's -- which is exactly what
   // stops a waiting consumer from blocking everything behind it.
   // ...and two resources that could not run out while only ~2 instructions were in flight.
   // rn_stall used to be a $fatal on exactly this reasoning; with a ROB behind a waiting load
   // it is a legitimate condition and has to be back-pressure instead.
   // src_pend IS GONE. Dispatch no longer waits for an instruction's operands -- that wait
   // moves into the scheduler, which is the entire point. What still blocks dispatch is
   // structural only: no ROB slot, no scheduler entry, or a rename shard run dry.
   // Back-pressure at the FRONTEND, never at issue: a full store buffer holds dispatch,
   // which costs nothing at the head of the machine and keeps unit state out of the
   // scheduler's select (docs/OOO2-Spec.md 15).
   wire d_hold = d_valid & (~rob_ready | ~iq_ready | rn_stall | ser_block
                            | (d_st_nb & ~sq_d_ready)
                            | (d_ld_nb & ~lq_d_ready));
   // NOT GATED ON THIS CYCLE'S REDIRECT (2026-09-05, gate V3 at -0.919 ns). The redirect is
   // M's completion, which a landing load can veto (ld_land), which the load queue's store
   // ordering decides: through `~redirect` here that whole chain -- the store queue's conflict
   // compare, the LSU/MMU arbitration, M's done -- ran on into rename port B, the pending
   // table's queries and the schedulers' entry writes, 34 levels. An instruction dispatched
   // in the redirect cycle is younger than the redirecting op and dies with everything else
   // younger: every structure's flush arm is ordered after its allocation and wins (the
   // ROB, the schedulers, the pending table, the load and store queues, rename). The
   // registered `redirect_q` stays: the frontend has nothing valid the cycle after anyway.
   // ...and not on the live fr_set either (gate V5, 2026-09-05, -0.968 ns): fr_set is M's
   // resolved mispredict that cannot redirect yet (`~redirect`, itself ld_land and the store
   // queue's drain), and through fr_active it re-imported the whole completion cone this
   // gate had just been freed of. The registered fr_v holds dispatch from the cycle after
   // the branch resolves; the one cycle of wrong-path dispatch before that is the flush's.
   wire d_take = d_valid & ~d_hold & ~redirect_q & ~fr_v & ~dec_red_q;   // ~dec_red_q: hold the one-cycle-late decode-redirect window (mirrors redirect_q)
   // slot B's own hold (see the rules where d2_cls is defined); d_take carries the redirect terms
   wire d2_hold = ~d_plain | ~d2_plain | ((d_cls_l & d2_cls_l) | (d_cls_fc & d2_cls_fc)) | ~iq_ready_b | ~rob_ready2 | rn_stall
                | (d2_st_nb & (d_st_nb | ~sq_d_ready)) | (d2_ld_nb & (d_ld_nb | ~lq_d_ready));
   wire d2_take = d2_valid & d_take & ~d2_hold & ~dcr0;   // ~dcr0: slot 0 decode-redirects -> squash the younger slots packed after it
   assign rn_valid_b = d2_take;
   // slot C (dispatch swizzle): reaches ANY pipe now (the 3rd ALU is gone). It is accepted per
   // the take3 rule and swizzled -- LS->M, FC->F, ALU->ALUb if I1 is ALU else ALUa. three_wide
   // is the master enable (0 at IW=2 -> C never dispatches -> retire-identical to the 2-wide core).
   // take3 class-accept (spec): ALU3 ok unless I1&I2 both ALU; LS3/FC3 ok unless already used.
   wire d3_accept = (d3_cls_i  & ~(d_cls_i & d2_cls_i))
                  | (d3_cls_l  & ~d_cls_l  & ~d2_cls_l)
                  | (d3_cls_fc & ~d_cls_fc & ~d2_cls_fc);
   wire d3_hold = ~three_wide | ~d3_accept | ~d_plain | ~d2_plain | ~d3_plain
                | ~iq_ready_c | ~rob_ready3 | rn_stall
                | (d3_st_nb & ~sq_d_ready) | (d3_ld_nb & ~lq_d_ready);
   wire d3_take = d3_valid & d2_take & ~d3_hold & ~dcr1;  // ~dcr1 (and ~dcr0 via d2_take): squash slots after a decode-redirect CTI
   assign rn_valid_c = d3_take;
   // The oldest DISPATCHED decode-redirect CTI drives the frontend resteer (fe_red below).
   // d2_take already carries ~dcr0 and d3_take ~dcr0&~dcr1, so at most one dr* is high.
   wire dr0 = d_take  & dcr0;
   wire dr1 = d2_take & dcr1;
   wire dr2 = d3_take & dcr2;
   assign dec_red     = dr0 | dr1 | dr2;
   assign dec_red_tgt = dr0 ? (d_pc  + d_imm)
                      : dr1 ? (d2_pc + d2_imm)
                      :       (d3_pc + d3_imm);
   assign dec_red_seq = (dr0 ? d_seq : dr1 ? d2_seq : d3_seq) + 1'b1;

   // ---- the RAS top every frontend redirect restores (ooo2_predictor rb_rsp) ----------------
   // A redirect restores the RAS pointer to where the redirecting instruction leaves it:
   //   head (M, SYSQ): every older instruction has retired, so the pointer the retired calls
   //                   and returns leave: rsp_r, below.
   //   CTF restart:    the mispredicting CTI's fetch-time snapshot plus its own push or pop.
   //   decode resteer: the redirected slot's snapshot, plus its push when it is a call.
   // A call or return is x1/x5 linkage: a call writes a link register, a return is a jalr that
   // reads one and writes none.
   function [1:0] cti_cls;     // {call, ret}
      input is_jump, is_jalr, rd_v;  input [5:0] rd, rs1;
      reg lrd, lrs;
      begin
         lrd = rd_v & ((rd == 6'd1) | (rd == 6'd5));
         lrs = (rs1 == 6'd1) | (rs1 == 6'd5);
         cti_cls = {is_jump & lrd, is_jalr & lrs & ~lrd};
      end
   endfunction
   wire [1:0] d_cls1 = cti_cls(d_is_jump,  d_is_jalr,  d_rd_v,  d_rd,  d_rs1);
   wire [1:0] d_cls2 = cti_cls(d2_is_jump, d2_is_jalr, d2_rd_v, d2_rd, d2_rs1);
   wire [1:0] d_cls3 = cti_cls(d3_is_jump, d3_is_jalr, d3_rd_v, d3_rd, d3_rs1);
   // The retired pointer: each ROB entry's {call, ret}, written at dispatch, read at commit.
   reg [1:0]      rcls [0:ROB_DEPTH-1];
   reg [RASB-1:0] rsp_r;
   integer ri;
   initial begin rsp_r = {RASB{1'b0}}; for (ri = 0; ri < ROB_DEPTH; ri = ri + 1) rcls[ri] = 2'b00; end
   always @(posedge clk) begin
      if (rn_valid)   rcls[rob_d_idx]  <= d_cls1;
      if (rn_valid_b) rcls[rob_d_idx2] <= d_cls2;
      if (rn_valid_c) rcls[rob_d_idx3] <= d_cls3;
   end
   function [RASB-1:0] rsp_step;    // +1 for a call, -1 for a return
      input [1:0] cls;
      rsp_step = cls[1] ? {{(RASB-1){1'b0}}, 1'b1} : cls[0] ? {RASB{1'b1}} : {RASB{1'b0}};
   endfunction
   wire [RASB-1:0] rsp_c1 = rob_c_valid  ? rsp_step(rcls[rob_head_idx])  : {RASB{1'b0}};
   wire [RASB-1:0] rsp_c2 = rob_c2_valid ? rsp_step(rcls[rob_head2_idx]) : {RASB{1'b0}};
   wire [RASB-1:0] rsp_c3 = rob_c3_valid ? rsp_step(rcls[rob_head3_idx]) : {RASB{1'b0}};
   always @(posedge clk) rsp_r <= reset ? {RASB{1'b0}} : rsp_r + rsp_c1 + rsp_c2 + rsp_c3;
   wire [RASB-1:0] cf_rsp  = cf_pdet[PD_RSP +: RASB]
                           + rsp_step({cf_is_jump & cf_link_rd, cf_is_jalr & cf_link_rs & ~cf_link_rd});
   wire [RASB-1:0] dec_rsp = dr0 ? d_pdet[PD_RSP +: RASB]  + rsp_step({d_cls1[1],  1'b0})
                           : dr1 ? d2_pdet[PD_RSP +: RASB] + rsp_step({d_cls2[1], 1'b0})
                           :       d3_pdet[PD_RSP +: RASB] + rsp_step({d_cls3[1], 1'b0});
   wire [RASB-1:0] fe_red_rsp = (m_red_fire | sy_red) ? rsp_r : fr_set ? cf_rsp : dec_rsp;
   always @(posedge clk) fe_red_rsp_q <= fe_red_rsp;
   // ---- the history every frontend redirect restores (ooo2_predictor rb_ghr) ------------------
   // The redirecting instruction's snapshot, plus its outcome when it is a conditional the BTB
   // knew (the only ones the history holds): a CTF restart's own branch, or none for a decode
   // resteer (a jal, or a backward branch the BTB missed). A redirect at the head (a trap, an
   // xret, a serializing op) starts a new context and restarts the history at zero.
   wire [GHL-1:0] cf_ghr  = cf_pdet[PD_GHR +: GHL];
   wire [GHL-1:0] fr_ghr  = (cf_is_branch & cf_pdet[DCR_HIT]) ? {cf_ghr[GHL-2:0], cf_taken} : cf_ghr;
   wire [GHL-1:0] dec_ghr = dr0 ? d_pdet[PD_GHR +: GHL] : dr1 ? d2_pdet[PD_GHR +: GHL] : d3_pdet[PD_GHR +: GHL];
   wire [GHL-1:0] fe_red_ghr = (m_red_fire | sy_red) ? {GHL{1'b0}} : fr_set ? fr_ghr : dec_ghr;
   always @(posedge clk) fe_red_ghr_q <= fe_red_ghr;
   always @(posedge clk) if (!reset) begin
      if (rn_valid_b & ~rn_valid)       $fatal(1, "ooo2_core: slot B dispatched without slot A");
      if (rn_valid_b & (d2_cls == d_cls)) $fatal(1, "ooo2_core: slot B dispatched to slot A's scheduler");
      if (rn_valid_b & ((d_st_nb & d2_st_nb) | (d_ld_nb & d2_ld_nb)))
         $fatal(1, "ooo2_core: two allocations into one memory queue");
      if (rn_valid_c & ~rn_valid_b)     $fatal(1, "ooo2_core: slot C dispatched without slot B");
      // swizzle mutual-exclusion: no two dispatched ops can target the same pipe.
      if (c_to_ia & (rn_valid & d_cls_i))   $fatal(1, "ooo2_core: swizzle put two ops on ALUa");
      if (c_to_ib & b_to_i2)                $fatal(1, "ooo2_core: swizzle put two ops on ALUb");
      if ((l_slot0 + b_to_l + c_to_l) > 2'd1) $fatal(1, "ooo2_core: swizzle put two ops on the M pipe");
      if ((f_slot0 + b_to_f + c_to_f) > 2'd1) $fatal(1, "ooo2_core: swizzle put two ops on the F pipe");
   end

   // `accept` means X CAN TAKE A NEW BUNDLE -- it is free, or it is being dispatched this
   // cycle. It used to double as "the backend is ready", which was the same thing only
   // because X fed M directly. Conflating them again would overwrite an undispatched
   // instruction, because the frontend's load-on-accept wins over its clear-on-consume.
   assign accept  = ~d_valid | rn_valid;

   always @(posedge clk) begin
      if (reset) begin
         m_valid <= 1'b0;
      end else begin

         // Flush a wrong-path op M is HOLDING across a redirect. With control flow off M, M now
         // speculates past an unresolved branch, so its held op (a walking load, a mul/div) can
         // be younger than a branch that squashes -- m_advance=0 would otherwise keep it, and its
         // late LSU fill/land would hit a flushed lq slot. A redirect is head-gated, so a held
         // op that is not the redirecting op is younger, hence wrong-path. (m_red_fire retires
         // its own op via the m_advance path below, as before.)
         if (~m_advance & redirect) m_valid <= 1'b0;

         if (m_advance) begin
            m_valid       <= iss_m;
            m_pc          <= q_pc;
            m_insn        <= q_insn;
            m_rvc         <= q_rvc;
            m_seq         <= q_seq;
            m_pdet        <= q_pdet;
            m_pred_npc    <= q_pred_npc;
            m_rd          <= q_rd;
            m_rd_v        <= q_rd_v;
            m_prd         <= q_prd;
            m_shard       <= q_shard;
            m_rs1         <= q_rs1;
            m_imm         <= q_imm;
            m_result      <= x_result;
            m_addr        <= x_addr;
            m_st_data     <= x_rs2;
            // "x_rs2 is valid this cycle": the pending register says ready, OR a writeback
            // is naming it right now and the PRF's write-through returns it anyway. This is
            // exactly the condition the shadow check above asserts for every other operand.
            m_rs2_rdy     <= pnd_i2
                           | (wkv[0] & (wkp[0*RN_PBITS +: RN_PBITS] == i_ps2))
                           | (wkv[1] & (wkp[1*RN_PBITS +: RN_PBITS] == i_ps2))
                           | (wkv[2] & (wkp[2*RN_PBITS +: RN_PBITS] == i_ps2));
            m_sq_tag      <= q_sq_tag;
            m_lq_idx      <= q_lq_idx;
            m_rs1_val     <= x_rs1;
            m_rs3_val     <= x_rs3;   // FMA 3rd operand
            m_mem_size    <= q_mem_size;
            m_mem_signed  <= q_mem_signed;
            m_is_mem      <= q_is_mem;
            m_is_store    <= q_is_store;
            m_is_amo      <= q_is_amo;
            m_amo_func    <= q_amo_func;
            m_is_branch   <= q_is_branch;
            m_is_jump     <= q_is_jump;
            m_is_jalr     <= q_is_jalr;
            m_redirect    <= x_redirect;
            m_target      <= x_target;
            m_taken       <= x_taken;
            m_taken_tgt   <= x_taken_tgt;
            m_is_mul      <= q_is_mul;
            m_is_csr      <= q_is_csr;
            m_csr_func    <= q_csr_func;
            m_is_serialize<= q_is_serialize;
            m_is_fp       <= q_is_fp;
            m_is_fencei   <= q_is_fencei;
            m_is_cbo      <= q_is_cbo;
            m_cbo_zero    <= q_cbo_zero;
            m_cbo_keep    <= q_cbo_keep;
            m_illegal     <= q_illegal;
            m_fault       <= q_fault;
            m_fault_cause <= q_fault_cause;
            m_fault_tval  <= q_fault_tval;
         end
      end
   end
endmodule

`default_nettype wire
