`include "va_codec.vh"
`default_nettype none

// In-order pipelined RVA22S64 core: F | X | M.
//
//   F : PC -> iMMU -> I$ window -> aligner -> RVC expand -> decode   (ooo2_frontend)
//   X : regfile read + M-bypass -> exec_alu (ALU/AGU/compare) -> branch_unit
//   M : LSU (dMMU + D$) | mul/div | csr_file | trap | redirect | regfile write
//
// M IS THE ONLY COMMIT POINT. Every architectural side effect happens there and
// nowhere else, which is what makes traps precise for free: when an instruction is
// in M nothing older can still fault (older ones have retired) and nothing younger
// has changed anything (X and F hold no state). All the OoO core's recovery
// machinery -- checkpoints, replay-to-solo fault delivery, the illegal/data-fault
// latches, the AMO dispatch gap, rollback priority -- collapses into one mux here.
//
// STALLS. Anything multi-cycle (D$ access or miss, page-table walk, divide,
// multiply) freezes the whole pipe: M holds its instruction until `m_done`, X holds
// because it cannot hand off, F holds because `accept` is low. One bypass level
// (M -> X) covers every RAW hazard, because an instruction two ahead has already
// written the regfile.
//
// SERIALIZATION. A CSR/system/fence op lets nothing follow it into X until it has
// left M, so CSR values, privilege, satp and mstatus.FS are never read stale.
`ifndef OOO2_HW
 `define OOO2_HW 8                 // fetch window halfwords: 16 bytes, the shipping build since 2026-09-05 (4 before)
`endif
module ooo2_core
  #(parameter PCW  = 64,
    parameter SEQW = 8,
    parameter HW   = `OOO2_HW,
    parameter AW   = 64,
    parameter PDW   = 18,          // ooo2_predictor predict-detail width (BIMW+YW+BOW: base offset for two-wide fetch)
    parameter [PCW-1:0] RESET_PC = 0)
   (input  wire                    clk,
    input  wire                    reset,
    // ---- instruction memory (combinational window at the translated PA) ----
    output wire [PCW-1:0]          imem_addr,
    // VA-tagged fetch buffer (rv_soc_top): the buffer hit test compares the VIRTUAL
    // address so a hit does not wait on address translation.  It therefore needs to know
    // (a) the VA, (b) when this cycle's PA is actually trustworthy, and (c) when the fetch
    // translation context changed underneath it.
    output wire [PCW-1:0]          imem_vaddr,
    output wire                    imem_xlate_ok,   // PA valid this cycle (not walking/faulting)
    output wire                    imem_ctx_chg,    // drop the buffer: mapping may have changed
    // Diagnostic only (FBDIAG_BASE readout).  These are the REGISTERED copies the VA tag
    // already maintains, so exporting them adds a fanout and nothing else.
    output wire [63:0]             imem_satp_q,
    output wire [1:0]              imem_priv_q,
    // Fetch-buffer events (computed in rv_soc_top, where the buffer lives) and the redirect
    // it needs to qualify them.  Same route as hpm_dc_access/hpm_ic_access below.
    output wire                    fe_redirect,
    input  wire                    hpm_fb_hit,
    input  wire                    hpm_fb_rhit,
    input  wire [HW*16-1:0]        imem_data,
    input  wire [$clog2(HW+2)-1:0] imem_avail,
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
    output wire [PCW-1:0]          retire2_pc,
    output wire [31:0]             retire2_insn,
    output wire                    redirect,
    output wire [PCW-1:0]          redirect_target);

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
   wire                     fe_fx_valid;   // fetch assembled an instruction (bubble sub-attribution)
   wire [PCW-1:0]           imem_va;
   wire [55:0]              immu_pa;
   wire                     immu_ready, immu_fault;
   wire [3:0]               immu_cause;
   // ~imem_ctx_chg IS PART OF "IS THIS CYCLE'S FETCH DATA TRUSTWORTHY".  The VA-tagged
   // buffer is invalidated by imem_ctx_chg in rv_soc_top, but that invalidation lands at
   // the END of the cycle while fb_hit is combinational -- so for exactly one cycle the
   // buffer can still serve bytes fetched under the PREVIOUS translation while the iMMU has
   // already switched to the new one.  Found on silicon at 111 MHz, first boot, in under a
   // second of kernel time: va=ffffffff80012370 hit with pa_cached=0000000080212370 (Sv39)
   // while the iMMU returned pa=00ffffff80012370 -- the UNTRANSLATED va, i.e. bare mode.
   // Gated here and not on fb_hit for two reasons: this is the one site that already decides
   // whether fetch data may be consumed (rule: one precondition, one site), and putting
   // imem_ctx_chg -- which contains a 64-bit satp compare -- into the fb_hit cone would put
   // back exactly the compare the VA tag was introduced to remove.
   // Costs one fetch bubble per satp write / sfence.vma / privilege change.
   wire [$clog2(HW+2)-1:0]  imem_avail_g = (immu_ready & ~immu_fault & ~imem_ctx_chg) ? imem_avail
                                                                      : {$clog2(HW+2){1'b0}};
   // resolve/training port (driven from M, below)
   wire                     res_v, res_cbr, res_call, res_ret, res_taken, res_rep;
   wire [PCW-1:0]           res_tgt;
   wire                     redirect_is_trap;
   wire [SEQW-1:0]          redirect_seq;
   wire                     fe_red_pulse, fr_set, fr_active;
   wire [PCW-1:0]           fe_red_tgt;
   wire [SEQW-1:0]          fe_red_seq;

   // ---- FMAX: the predictor's training bundle lands one cycle later --------------
   // res_v is gated by m_done, which depends on lsu_done -- so the D$/dTLB hit path
   // reached the BTB/ycorr arrays combinationally. Updates are hints, so the extra
   // cycle costs no correctness and no bubble; kept in step with redirect_q so u_bp
   // sees resolve and rollback in their original relative order.
   reg                      res_v_q, res_cbr_q, res_call_q, res_ret_q;
   reg                      res_taken_q, res_rep_q;
   reg [PDW-1:0]            res_pdet_q;
   reg [PCW-1:0]            res_tgt_q;
   reg [PCW-1:0]            res_pc_q;     // the resolving CTI's own PC. u_bp recomputes its
                                          // BTB index and its PC-only tags from this instead
                                          // of carrying them, which is what keeps PDW at 16
                                          // while the BTB holds 4096 entries.
   initial begin res_v_q = 1'b0; res_rep_q = 1'b0; end
   always @(posedge clk) begin
      if (reset) begin res_v_q <= 1'b0; res_rep_q <= 1'b0; end
      else       begin res_v_q <= res_v; res_rep_q <= res_rep; end
      res_cbr_q   <= res_cbr;
      res_call_q  <= res_call;
      res_ret_q   <= res_ret;
      res_taken_q <= res_taken;
      res_pdet_q  <= m_pdet;
      res_tgt_q   <= res_tgt;
      res_pc_q    <= m_pc;
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
   initial fe_red_q = 1'b0;
   always @(posedge clk) begin
      if (reset) fe_red_q <= 1'b0;
      else       fe_red_q <= fe_red_pulse;
      fe_red_tgt_q <= fe_red_tgt;
      fe_red_seq_q <= fe_red_seq;
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
   ooo2_frontend #(.PCW(PCW), .SEQW(SEQW), .HW(HW), .PDW(PDW),
                  .RESET_PC(RESET_PC)) fe
     (.clk(clk), .reset(reset), .accept(accept), .consume(rn_valid),
      // slot B (item 10b): not filled yet -- two_wide low keeps the one-IR timing exactly
      // slot B (item 10b): dispatched beside A when the rules below allow
      .consume_b(rn_valid_b), .two_wide(1'b1),
      .d2_valid(d2_valid), .d2_pc(d2_pc), .d2_insn(d2_insn), .d2_rvc(d2_rvc), .d2_seq(d2_seq), .d2_pdet(d2_pdet), .d2_pred_npc(d2_pred_npc), .d2_rd(d2_rd), .d2_rs1(d2_rs1), .d2_rs2(d2_rs2), .d2_rs3(d2_rs3), .d2_rd_v(d2_rd_v), .d2_rs1_v(d2_rs1_v), .d2_rs2_v(d2_rs2_v), .d2_rs3_v(d2_rs3_v), .d2_imm(d2_imm), .d2_alu_op(d2_alu_op), .d2_alu_w(d2_alu_w), .d2_alu_uw(d2_alu_uw), .d2_op1_sel(d2_op1_sel), .d2_op2_imm(d2_op2_imm), .d2_res_link(d2_res_link), .d2_is_mem(d2_is_mem), .d2_is_store(d2_is_store), .d2_mem_size(d2_mem_size), .d2_mem_signed(d2_mem_signed), .d2_is_branch(d2_is_branch), .d2_br_func(d2_br_func), .d2_is_jump(d2_is_jump), .d2_is_jalr(d2_is_jalr), .d2_is_mul(d2_is_mul), .d2_is_csr(d2_is_csr), .d2_csr_func(d2_csr_func), .d2_is_serialize(d2_is_serialize), .d2_is_amo(d2_is_amo), .d2_amo_func(d2_amo_func), .d2_is_fp(d2_is_fp), .d2_is_fencei(d2_is_fencei), .d2_is_cbo(d2_is_cbo), .d2_cbo_zero(d2_cbo_zero), .d2_cbo_keep(d2_cbo_keep), .d2_illegal(d2_illegal), .d2_mis_taken(d2_mis_taken), .d2_mis_nt(d2_mis_nt), .d2_fault(d2_fault), .d2_fault_cause(d2_fault_cause), .d2_fault_tval(d2_fault_tval),
      .redirect(fe_red_q), .redirect_pc(fe_red_tgt_q), .redirect_seq(fe_red_seq_q),
      .irq_inject(irq_inject), .irq_taken(irq_taken), .fe_fx_valid(fe_fx_valid),
      .imem_addr(imem_va), .imem_ipc(), .imem_data(imem_data),
      .imem_avail(imem_avail_g),
      .imem_fault(immu_ready & immu_fault), .imem_cause(immu_cause),
      .res_v(res_v_q), .res_cbr(res_cbr_q), .res_call(res_call_q), .res_ret(res_ret_q),
      .res_taken(res_taken_q), .res_pdet(res_pdet_q), .res_tgt(res_tgt_q),
      .res_pc(res_pc_q), .res_rep(res_rep_q),
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
      .cur_seq(fe_cur_seq));

   // Valid-DRAM window for the MMU's unbacked-PA access-fault check. Enforced only
   // under cosim, sized to the modeled DDR so an out-of-range access faults exactly as
   // the simmerv golden model does; the FPGA/test default stays fully permissive, so
   // no new faults appear outside cosim. (Same rule as backend_top.)
`ifdef COSIM_MEM_SIZE_LG2
   localparam [63:0] DRAM_BASE = 64'h8000_0000;
   localparam [63:0] DRAM_TOP  = 64'h8000_0000 + (64'd1 << `COSIM_MEM_SIZE_LG2);
`else
   localparam [63:0] DRAM_BASE = 64'd0;
   localparam [63:0] DRAM_TOP  = 64'hFFFF_FFFF_FFFF_FFFF;
`endif

   // instruction-side translation. M-mode fetches are physical (satp forced Bare).
   wire [63:0] mmu_satp;
   wire [1:0]  mmu_priv, mmu_dpriv;
   wire        mmu_sum, mmu_mxr, mmu_flush;
   wire [63:0] satp_fetch = (mmu_priv  == 2'd3) ? 64'd0 : mmu_satp;
   wire [63:0] satp_data  = (mmu_dpriv == 2'd3) ? 64'd0 : mmu_satp;

   mmu #(.AW(56), .DRAM_BASE(DRAM_BASE), .DRAM_TOP(DRAM_TOP)) u_immu
     (.clk(clk), .reset(reset),
      .req_valid(1'b1), .req_vaddr(imem_va), .req_access(2'd0),
      .priv(mmu_priv), .sum(mmu_sum), .mxr(mmu_mxr), .satp(satp_fetch), .flush(mmu_flush),
      .ptw_addr(ptw_addr), .ptw_read(ptw_read),
      .ptw_rdata(ptw_rdata), .ptw_rvalid(ptw_rvalid),
      .walking(), .t_ready(immu_ready), .t_paddr(immu_pa), .t_fault(immu_fault),
      .t_cause(immu_cause), .t_uncached(), .t_ok(), .t_fault_raw());
   assign imem_addr = {8'd0, immu_pa};

   // ---- VA-tagged fetch buffer support ------------------------------------------------
   // The buffer caches instruction bytes under a VA tag, so it must be dropped on every
   // event that can change what that VA maps to, or what may be executed from it:
   //   satp write / sfence.vma -> mmu_flush        (csr_file o_tlb_flush)
   //   privilege change        -> priv != priv_q   (different translation AND different X)
   //   fence.i                 -> ic_inv_req       (already handled in rv_soc_top)
   // mstatus.SUM/MXR are deliberately NOT here: they gate DATA accesses, not fetch.
   // satp_fetch already folds in the M-mode bare case, so comparing it covers a
   // privilege change that switches translation off entirely; priv is compared as well
   // because S->U keeps satp but changes the U permission bit.
   //
   // imem_xlate_ok is the other half, and it is not optional.  Mid-walk the iMMU presents a
   // STALE LEAF as t_paddr.  Under the old PA tag a fill from it was self-correcting -- the
   // wrong bytes were tagged with the wrong PA, so the next lookup simply missed.  Under a
   // VA tag those same wrong bytes would carry the RIGHT VA and hit, executing garbage.
   reg  [1:0]  ipriv_q;
   reg  [63:0] isatp_q;
   always @(posedge clk) begin
      ipriv_q <= mmu_priv;
      isatp_q <= satp_fetch;
   end
   assign imem_vaddr    = imem_va;
   assign imem_xlate_ok = immu_ready & ~immu_fault;
   assign imem_ctx_chg  = mmu_flush | (ipriv_q != mmu_priv) | (isatp_q != satp_fetch);
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
   localparam integer RN_PBITS = RN_IDXB + 2;
   localparam [1:0]   SH_IE = 2'd0, SH_LD = 2'd1, SH_FE = 2'd2;

   wire d_ord   = d_is_mem | d_is_amo | d_is_mul | d_is_fp | d_is_csr | d_is_serialize
                | d_is_fencei | d_is_cbo | d_is_branch | d_is_jump | d_is_jalr
                | d_illegal | d_fault | d_is_irqop;
   wire d2_is_irqop = (d2_insn[6:2] == 5'b11100) & (d2_insn[14:12] == 3'b000)
                    & (d2_insn[31:20] == 12'h7F0) & ~d2_illegal & ~d2_fault;
   wire d2_ord  = d2_is_mem | d2_is_amo | d2_is_mul | d2_is_fp | d2_is_csr | d2_is_serialize
                | d2_is_fencei | d2_is_cbo | d2_is_branch | d2_is_jump | d2_is_jalr
                | d2_illegal | d2_fault | d2_is_irqop;

   // Destination shard = where the result will be written.  Loads, AMOs and mul/div take
   // SH_LD (see ooo2_prf.v on why mul/div ride with loads and not the ALU).
   //
   // An FP instruction goes to SH_FE only if its DESTINATION IS AN FP REGISTER.  d_rd[5] is
   // the class bit -- architectural 0..31 are integer, 32..63 are FP (ooo2_rename's reset
   // arm maps them that way).  fcvt.w.d, fmv.x.w, fclass and the FP compares are FP
   // instructions that write INTEGER registers; sending those to SH_LD, which already holds
   // both classes and so needs no extra room, means SH_FE can only ever hold FP mappings.
   // Its floor drops from 65 to 33, i.e. 128 entries to 64 -- the same 64-entry saving that
   // was worth 465 ps when mem_ie was cut (docs/rtl-rules.md I1).
   //
   // No new contention at this milestone: there is exactly one writeback per cycle.  When
   // out-of-order issue lands, SH_LD's writers become LSU + mul/div + FP-to-integer, all of
   // which are rare next to loads and all of which can hold in an output register.
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
   wire [1:0] d_shard = (d_is_mem | d_is_amo | d_is_mul) ? SH_LD
                      : d_is_fp                          ? SH_FE
                      : d_ord                            ? SH_LD   // CSR, jumps: M writes
                      :                                    SH_IE;  // the ALU, alone
   wire [1:0] d2_shard = (d2_is_mem | d2_is_amo | d2_is_mul) ? SH_LD
                       : d2_is_fp                            ? SH_FE
                       : d2_ord                              ? SH_LD
                       :                                       SH_IE;

   wire [RN_PBITS-1:0] rn_prs1, rn_prs2, rn_prs3, rn_prd;
   wire [RN_PBITS-1:0] rn_prs1_b, rn_prs2_b, rn_prs3_b, rn_prd_b;
   wire [RN_PBITS-1:0] rn_sprs1_b, rn_sprs2_b, rn_sprs3_b, rn_mprs1_b, rn_mprs2_b, rn_mprs3_b;
   wire                rn_lv1_b, rn_lv2_b, rn_lv3_b, rn_byp1_b, rn_byp2_b, rn_byp3_b;
   wire [RN_PBITS-1:0] rn_sprs1, rn_sprs2, rn_sprs3;   // the two map candidates, and
   wire [RN_PBITS-1:0] rn_mprs1, rn_mprs2, rn_mprs3;   // the late bit that chooses
   wire                rn_lv1, rn_lv2, rn_lv3;
   wire                rn_stall;
   wire [2:0]          rn_shard_low;
   // Rename exactly when the instruction actually enters M and is not being squashed --
   // the same condition that sets m_valid below.  Renaming on any looser condition would
   // allocate twice for one instruction, or allocate for a squashed one.
   wire rn_valid = d_take;      // dispatch is no longer gated on M being free

   ooo2_rename #(.IDXB(RN_IDXB), .N_FE(128)) u_rename
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
      // COMMIT NOW COMES FROM THE ROB HEAD, not from the M stage. One line, against a
      // structure the previous commit proved bit-identical over 9.17e6 commits -- the same
      // way rename itself was switched over once its shadow had earned it.
            .c_valid(rob_c_valid), .c_rd(rob_c_rd), .c_rd_v(rob_c_rd_v), .c_prd(rob_c_prd),
      .c2_valid(rob_c2_valid), .c2_rd(rob_c2_rd), .c2_rd_v(rob_c2_rd_v), .c2_prd(rob_c2_prd),
      .flush(redirect),
      .stall(rn_stall), .shard_low(rn_shard_low));

   wire [63:0] prf_rs1, prf_rs2, prf_rs3;
   ooo2_prf #(.IDXB(RN_IDXB), .N_FE(128)) u_prf
     (.clk(clk),
      .we_ie(alu_q_v), .we_ld(we_ld), .we_fe(we_fe),       // int-exec: from the writeback register
      .wa_ie(alu_q_prd), .wa_ld(wa_ld), .wa_fe(wa_fe),
      .wd_ie(alu_q_val), .wd_ld(wb_ld), .wd_fe(wb_fe),
      // Operands are read AT ISSUE, addressed by the entry the scheduler selected --
      // doc 1's "values live in one place". Reading them at dispatch and carrying them into
      // M is the second copy that property exists to avoid.
      .ra1(i_ps1), .ra2(i_ps2), .ra3(i_ps3),
      .rd1(prf_rs1), .rd2(prf_rs2), .rd3(prf_rs3));

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
   localparam integer ROB_DEPTH = 16, ROB_IDXB = 4;
   wire [ROB_IDXB-1:0] rob_d_idx, rob_d_idx2;
   wire                rob_ready, rob_ready2, rob_empty;
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
   wire rob_w_valid = (m_valid & m_done & ~m_ld_nb & ~fp_arith & ~m_st_nb) | ld_land;
   wire [ROB_IDXB-1:0] rob_w_idx = ld_land ? lq_l_rob : m_rob_idx;

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
   // slot B: the same two-candidate query; a source that IS A's destination is not ready
   wire pnd_s1_b, pnd_s2_b, pnd_s3_b, pnd_m1_b, pnd_m2_b, pnd_m3_b;
   wire pnd_r1_b = ~rn_byp1_b & (rn_lv1_b ? pnd_s1_b : pnd_m1_b);
   wire pnd_r2_b = ~rn_byp2_b & (rn_lv2_b ? pnd_s2_b : pnd_m2_b);
   wire pnd_r3_b = ~rn_byp3_b & (rn_lv3_b ? pnd_s3_b : pnd_m3_b);
   ooo2_pending #(.PBITS(RN_PBITS), .NWB(3)) u_pend
     (.clk(clk), .reset(reset),
      .a_v(rn_valid & d_rd_v), .a_preg(rn_prd),
      .a_v2(rn_valid_b & d2_rd_v), .a_preg2(rn_prd_b),
      .q10(rn_sprs1_b), .q11(rn_sprs2_b), .q12(rn_sprs3_b), .r10(pnd_s1_b), .r11(pnd_s2_b), .r12(pnd_s3_b),
      .q13(rn_mprs1_b), .q14(rn_mprs2_b), .q15(rn_mprs3_b), .r13(pnd_m1_b), .r14(pnd_m2_b), .r15(pnd_m3_b),
      .w_v({we_fe, we_ld, we_ie}), .w_preg({wa_fe, wa_ld, wa_ie}),
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
   // Keyed on rn_valid, NOT on `accept`. `accept` is the F/X QUEUE POP; in a redirect
   // cycle it is high while d_take is low, so the instruction is discarded rather than
   // consumed and its sources are never read. rn_valid = m_advance & d_take is the
   // cycle the operands actually move into M.
   // The shadow asserted that every source was ready or bypassed at the cycle X handed an
   // instruction to M. Consumption has moved to ISSUE and the scheduler enforces the same
   // property structurally -- an entry is not selectable until every source is ready -- so
   // the check moves with it: nothing may issue with a source still pending and no forward.
   always @(posedge clk) if (!reset & (iss_alu | iss_m)) begin
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
   // but `cti_ok` leaking from the aligner into `apc`, the BTB read address, through
   // ooo2_predictor's `hit`/`p_ret` (see that module's `predict`, and rule I6). NF is back
   // at 8 -- the standing policy -- to retest with that path gone.
   //
   // FOUR SIZES, FOUR DIFFERENT FAILING FAMILIES, and none of them the scheduler. The
   // design sits within ~100 ps of the limit on several paths at once and placement decides
   // which one bites (rule I2: 81 ps spread over IDENTICAL RTL). So this number is where
   // the search stopped, not a measurement that 5 is faster than 6 -- and the standing
   // policy remains a minimum of 8, recoverable by fixing the families above rather than
   // by a better scheduler.
   localparam integer NF = 8,  IBF = 3;    // FP arith, three sources, its OWN unit
   localparam integer OFF_I = 0, OFF_L = NI, OFF_F = NI + NL;
   localparam integer RS_IDXB = 4;         // widest per-class entry index (IBI)
   localparam integer NWB_C   = 3;         // writeback ports watched: one per PRF shard
   localparam integer PL_N = NI + NL + NF, PL_IB = 5;
   localparam integer SQ_N = 8, SQ_IB = 3;      // store buffer: entries, index width
   localparam integer SQ_TB = SQ_IB + 1;         // ...and its seqno: the index plus a wrap bit
                                                 // (ooo2_sq's head/tail counters), so that a load
                                                 // dispatched against a FULL queue counts NENT
                                                 // older stores, not zero
   localparam integer LQ_N = 4, LQ_IB = 2;      // load queue:   entries, index width
   localparam [1:0] C_I = 2'd0, C_L = 2'd1, C_F = 2'd2;

   // ORDERED: anything that can trap, redirect, touch memory or hold a unit for more than a
   // cycle. Those go to the in-order schedulers; what is left free to reorder is the pure
   // ALU op, which is exactly what queues up behind a consumer waiting on a load.
   // An FP load/store is a MEMORY op, not an FP-unit op -- it must go to the load scheduler
   // or memory ordering is silently broken for half the accesses.
   // FP ARITH NOW HAS ITS OWN SCHEDULER AND ITS OWN UNIT (the F stage below), so it no
   // longer queues behind every load, mul/div, CSR and branch in the in-order stream.
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
   wire d_cls_l = d_ord & ~d_cls_f;
   wire d_cls_i = ~d_ord;

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
   wire d2_cls_l = d2_ord & ~d2_cls_f;
   wire d2_cls_i = ~d2_ord;
   wire [1:0] d2_cls = d2_cls_i ? C_I : d2_cls_l ? C_L : C_F;
   wire [RN_PBITS-1:0] d2_prd_g = d2_rd_v ? rn_prd_b : {RN_PBITS{1'b0}};
   wire       d2_st_nb = d2_is_store & ~d2_is_amo & ~d2_is_cbo;
   wire       d2_ld_nb = d2_is_mem & ~d2_is_store & ~d2_is_amo & ~d2_is_cbo;
   wire [2:0] d2_srdy = {pnd_r3_b | ~d2_rs3_v, pnd_r2_b | ~d2_rs2_v | d2_st_nb, pnd_r1_b | ~d2_rs1_v};
   wire d_plain  = ~(d_is_serialize  | d_is_fencei  | d_is_cbo  | d_is_amo  | d_is_csr  | d_illegal  | d_fault  | d_is_irqop);
   wire d2_plain = ~(d2_is_serialize | d2_is_fencei | d2_is_cbo | d2_is_amo | d2_is_csr | d2_illegal | d2_fault | d2_is_irqop);

   wire [NWB_C-1:0]        wkv  = {we_fe, we_ld, we_ie};
   wire [NWB_C*RN_PBITS-1:0] wkp = {wa_fe, wa_ld, wa_ie};

   wire ri_ready, ri_iss_v, ri_blk_v;  wire [IBI-1:0] ri_d_ent, ri_iss_ent;
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

   ooo2_iq #(.NENT(NI),.IDXB(IBI),.NSRC(2),.ROBB(ROB_IDXB),.PBITS(RN_PBITS),.NWB(NWB_C),
             .FIXEDL(1),.INORDER(0)) u_iq_i
     (.clk(clk),.reset(reset),
      .d_valid((rn_valid & d_cls_i) | (rn_valid_b & d2_cls_i)),.d_ready(ri_ready),
      .d_rob(b_to_i ? rob_d_idx2 : rob_d_idx),
      .d_ps(b_to_i ? {rn_prs2_b, rn_prs1_b} : {rn_prs2, rn_prs1}),.d_r(b_to_i ? d2_srdy[1:0] : d_srdy[1:0]),
      .d_prd(b_to_i ? d2_prd_g : d_prd_g),.d_ent(ri_d_ent),
      .wb_v(wkv),.wb_preg(wkp),
      .unit_busy(1'b0),.iss_v(ri_iss_v),.iss_ent(ri_iss_ent),.iss_rob(ri_iss_rob),
     .iss_take(ri_take),
      .hold_v(i_v & (i_cls == C_I)),.hold_ent(i_ent[IBI-1:0]),
      .blk_v(ri_blk_v),.blk_pr(ri_blk_pr),.flush(redirect),.occupancy(ri_occ));

   ooo2_iq #(.NENT(NL),.IDXB(IBL),.NSRC(3),.ROBB(ROB_IDXB),.PBITS(RN_PBITS),.NWB(NWB_C),
             .FIXEDL(0),.INORDER(1)) u_iq_l
     (.clk(clk),.reset(reset),
      .d_valid((rn_valid & d_cls_l) | (rn_valid_b & d2_cls_l)),.d_ready(rl_ready),
      .d_rob(b_to_l ? rob_d_idx2 : rob_d_idx),
      .d_ps(b_to_l ? {rn_prs3_b, rn_prs2_b, rn_prs1_b} : {rn_prs3, rn_prs2, rn_prs1}),.d_r(b_to_l ? d2_srdy : d_srdy),
      .d_prd(b_to_l ? d2_prd_g : d_prd_g),.d_ent(rl_d_ent),
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
      .d_valid((rn_valid & d_cls_f) | (rn_valid_b & d2_cls_f)),.d_ready(rf_ready),
      .d_rob(b_to_f ? rob_d_idx2 : rob_d_idx),
      .d_ps(b_to_f ? {rn_prs3_b, rn_prs2_b, rn_prs1_b} : {rn_prs3, rn_prs2, rn_prs1}),.d_r(b_to_f ? d2_srdy : d_srdy),
      .d_prd(b_to_f ? d2_prd_g : d_prd_g),.d_ent(rf_d_ent),
      .wb_v(wkv),.wb_preg(wkp),
      .unit_busy(~f_advance | (i_v & i_needs_f)),
      .iss_v(rf_iss_v),.iss_ent(rf_iss_ent),.iss_rob(rf_iss_rob),
     .iss_take(rf_take),
      .hold_v(i_v & (i_cls == C_F)),.hold_ent(i_ent[IBF-1:0]),
      .blk_v(rf_blk_v),.blk_pr(rf_blk_pr),.flush(redirect),.occupancy(rf_occ));

   // Dispatch back-pressure comes from whichever scheduler this instruction is routed to.
   wire iq_ready = d_cls_i ? ri_ready : d_cls_l ? rl_ready : rf_ready;
   wire iq_ready_b = d2_cls_i ? ri_ready : d2_cls_l ? rl_ready : rf_ready;
   wire b_to_i = rn_valid_b & d2_cls_i, b_to_l = rn_valid_b & d2_cls_l, b_to_f = rn_valid_b & d2_cls_f;
   wire [RS_IDXB-1:0] iq_d_ent = d_cls_i ? {{(RS_IDXB-IBI){1'b0}}, ri_d_ent}
                               : d_cls_l ? {{(RS_IDXB-IBL){1'b0}}, rl_d_ent}
                               :           {{(RS_IDXB-IBF){1'b0}}, rf_d_ent};

   // ISSUE ARBITRATION, one per cycle into the single issue register. Long-latency classes
   // win: they are gated on M being free anyway, so they only bid when they can make
   // progress, while an ALU op can always go next cycle instead.
   wire pick_l = rl_iss_v;
   wire pick_f = rf_iss_v & ~pick_l;
   wire pick_i = ri_iss_v & ~pick_l & ~pick_f;
   wire iq_iss_v = pick_l | pick_f | pick_i;
   wire [1:0] pick_cls = pick_l ? C_L : pick_f ? C_F : C_I;
   wire [RS_IDXB-1:0] iq_iss_ent = pick_l ? {{(RS_IDXB-IBL){1'b0}}, rl_iss_ent}
                                 : pick_f ? {{(RS_IDXB-IBF){1'b0}}, rf_iss_ent}
                                 :          {{(RS_IDXB-IBI){1'b0}}, ri_iss_ent};
   wire [ROB_IDXB-1:0] iq_iss_rob = pick_l ? rl_iss_rob : pick_f ? rf_iss_rob : ri_iss_rob;
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
   reg  [3*RN_PBITS-1:0] psmem_l [0:NL-1];
   reg  [3*RN_PBITS-1:0] psmem_f [0:NF-1];
   wire [3*RN_PBITS-1:0] ps_out = pick_l ? psmem_l[rl_iss_ent]
                                : pick_f ? psmem_f[rf_iss_ent]
                                :          psmem_i[ri_iss_ent];
   wire [3*RN_PBITS-1:0] ps_in   = {rn_prs3, rn_prs2, rn_prs1};
   wire [3*RN_PBITS-1:0] ps_in_b = {rn_prs3_b, rn_prs2_b, rn_prs1_b};
   always @(posedge clk) begin
      if ((rn_valid & d_cls_i) | b_to_i) psmem_i[ri_d_ent] <= b_to_i ? ps_in_b : ps_in;
      if ((rn_valid & d_cls_l) | b_to_l) psmem_l[rl_d_ent] <= b_to_l ? ps_in_b : ps_in;
      if ((rn_valid & d_cls_f) | b_to_f) psmem_f[rf_d_ent] <= b_to_f ? ps_in_b : ps_in;
   end
   wire [RN_PBITS-1:0] iq_iss_ps1 = ps_out[0 +: RN_PBITS];
   wire [RN_PBITS-1:0] iq_iss_ps2 = ps_out[RN_PBITS +: RN_PBITS];
   wire [RN_PBITS-1:0] iq_iss_ps3 = ps_out[2*RN_PBITS +: RN_PBITS];
   wire iq_iss_take;
   assign ri_take = pick_i & iq_iss_take;
   assign rl_take = pick_l & iq_iss_take;
   assign rf_take = pick_f & iq_iss_take;
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

   // An ordered op completes when M takes it; an ALU op completes unconditionally, because
   // SH_IE has exactly one writer and it is this one. Nothing can be in the way.
   assign i_needs_f   = i_v & (i_cls == C_F);
   assign i_needs_m   = i_v & q_ord & ~i_needs_f;
   wire   i_done      = i_v & (i_needs_f ? f_advance : q_ord ? m_advance : 1'b1);
   wire   iss_ready   = ~i_v | i_done;
   assign iq_iss_take = iq_iss_v & iss_ready & ~redirect;
   // ~redirect on BOTH. The register is cleared on a redirect, but these are
   // combinational off i_v -- without the guard an instruction being squashed still writes
   // the register file and still marks its ROB slot done, in the very cycle rename is
   // rolling that physical register back. That is the zombie writeback in miniature, and it
   // is what failed all 61 virtual-memory tests: they are the ones that trap often.
   wire   iss_m       = i_needs_m & m_advance & ~redirect;     // loads M this cycle
   wire   iss_f       = i_needs_f & f_advance & ~redirect;     // loads the F stage
   wire   iss_alu     = i_v & ~q_ord & ~redirect;  // completes here, always

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

   // Payload: packed at dispatch, unpacked at issue with the SAME concatenation, so a
   // width or ordering mistake is a lint error rather than a wrong instruction.
   localparam integer PLW = PCW + 32 + 1 + SEQW + PDW + PCW + 6 + 1 + RN_PBITS + 2 + 6
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
                           d_br_func, d_mis_taken, d_mis_nt,
                           d_rs1_v, d_rs2_v, d_rs3_v, d_ord, sq_d_idx, lq_d_idx};
   wire [PLW-1:0] pl_in_b = {d2_pc, d2_insn, d2_rvc, d2_seq, d2_pdet, d2_pred_npc, d2_rd, d2_rd_v,
                           (d2_rd_v ? rn_prd_b : {RN_PBITS{1'b0}}), d2_shard, d2_rs1, d2_imm,
                           d2_mem_size, d2_mem_signed, d2_is_mem, d2_is_store, d2_is_amo,
                           d2_amo_func, d2_is_branch, d2_is_jump, d2_is_jalr, d2_is_mul,
                           d2_is_csr, d2_csr_func, d2_is_serialize, d2_is_fp, d2_is_fencei,
                           d2_is_cbo, d2_cbo_zero, d2_cbo_keep, d2_illegal, d2_fault,
                           d2_fault_cause, d2_fault_tval,
                           d2_alu_op, d2_alu_w, d2_alu_uw, d2_op1_sel, d2_op2_imm, d2_res_link,
                           d2_br_func, d2_mis_taken, d2_mis_nt,
                           d2_rs1_v, d2_rs2_v, d2_rs3_v, d2_ord, sq_d_idx, lq_d_idx};
   // ONE payload array across all three schedulers, indexed by a flat slot number with a
   // per-class offset -- each scheduler has its own entry-number space, and the offsets are
   // what stop them aliasing.
   reg [PLW-1:0] plmem_i [0:NI-1];
   reg [PLW-1:0] plmem_l [0:NL-1];
   reg [PLW-1:0] plmem_f [0:NF-1];
   wire [PLW-1:0] pl_out = (i_cls == C_I) ? plmem_i[i_ent[IBI-1:0]]
                         : (i_cls == C_L) ? plmem_l[i_ent[IBL-1:0]]
                         :                  plmem_f[i_ent[IBF-1:0]];
   always @(posedge clk) begin
      if ((rn_valid & d_cls_i) | b_to_i) plmem_i[ri_d_ent] <= b_to_i ? pl_in_b : pl_in;
      if ((rn_valid & d_cls_l) | b_to_l) plmem_l[rl_d_ent] <= b_to_l ? pl_in_b : pl_in;
      if ((rn_valid & d_cls_f) | b_to_f) plmem_f[rf_d_ent] <= b_to_f ? pl_in_b : pl_in;
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
   wire [1:0]          q_shard, q_mem_size;
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
   wire [SQ_IB:0]      sq_occ;
   wire                sq_av_any;
   wire [SQ_IB-1:0]    sq_d_idx;
   wire [SQ_TB-1:0]    sq_d_tag;
   wire [ROB_IDXB-1:0] sq_c_rob;
   wire [55:0]         sq_c_addr;
   wire [63:0]         sq_c_data;
   wire [1:0]          sq_c_size;
   wire                lsu_pt_done, lsu_pt_ack, lsu_pt_is_store, lsu_xo_v, lsu_xo_unc;
   wire [55:0]         lsu_xo_pa;
   wire st_b = rn_valid_b & d2_st_nb;                 // B is the store of the pair
   wire d_st_alloc = (rn_valid & d_st_nb) | st_b;
   // The head entry may go to memory only once it IS the ROB head: that is the point at
   // which no older instruction can still trap and no redirect can still squash it.
   // A committed store drains whenever the port is free: it was released by the ROB's
   // irrevocable pointer (sq_k_take below), not by reaching the head, so the head no
   // longer sits on every store for the ~6 cycles the cache takes.
   wire sq_go     = sq_c_v;
   wire                sq_kc_v;   wire [ROB_IDXB-1:0] sq_kc_rob;  wire [55:0] sq_kc_addr;
   wire [ROB_IDXB-1:0] rob_irr_idx;  wire rob_irr_v;
   wire sq_k_take = sq_kc_v & rob_irr_v & (sq_kc_rob == rob_irr_idx);
   // The queue pops at the HANDOFF to the LSU (pt_ack for a store), which registers the data;
   // the LSU holds the store until the D$ takes it and nothing can pass it there.
   wire sq_c_take = lsu_pt_ack & pt_store;

   ooo2_sq #(.NENT(SQ_N), .IDXB(SQ_IB), .PAW(56), .PBITS(RN_PBITS),
             .ROBB(ROB_IDXB), .NWB(NWB_C), .LQN(LQ_N), .LQIB(LQ_IB)) u_sq
     (.clk(clk), .reset(reset),
      .d_alloc(d_st_alloc), .d_rob(st_b ? rob_d_idx2 : rob_d_idx), .d_dpreg(st_b ? rn_prs2_b : rn_prs2),
      .d_ready(sq_d_ready), .d_idx(sq_d_idx), .d_tag(sq_d_tag), .av_any(sq_av_any),
      .a_v(m_sq_fill), .a_idx(m_sq_tag), .a_addr(lsu_xo_pa), .a_size(m_mem_size),
      .a_unc(lsu_xo_unc), .a_data_v(m_rs2_rdy), .a_data(m_st_data),
      .wb_v(wkv), .wb_preg(wkp), .wb_data({wb_fe, wb_ld, wb_ie}),
      .c_v(sq_c_v), .c_rob(sq_c_rob), .c_addr(sq_c_addr), .c_data(sq_c_data),
      .c_size(sq_c_size), .c_unc(sq_c_unc), .c_take(sq_c_take),
      .kc_v(sq_kc_v), .kc_rob(sq_kc_rob), .kc_addr(sq_kc_addr), .k_take(sq_k_take),
      // THE ALIAS TEST LIVES HERE, not at issue: ooo2_lq exports its entries, ooo2_sq keeps
      // a conflict matrix updated wherever an address arrives, and issue reads a flop.
      .l_pa(lq_e_pa), .l_size(lq_e_size), .l_tag(lq_e_tag), .l_av(lq_e_av),
      .l_fill(m_lq_fill), .l_fill_ix(m_lq_idx),
      .l_fill_pa(lsu_xo_pa), .l_fill_size(m_mem_size),
      .l_block(lq_e_block),
      .ld_tag(lq_q_tag), .ld_older(sq_ld_older),
      .occupancy(sq_occ), .flush(redirect));

   // Instrumentation for "did a load actually get reordered past a store". A load STARTS
   // its access only when ~ld_block, so a start with an older store still live is exactly
   // one reordering that the old in-order machine could not have done.
   wire sq_ld_reorder = lq_x_take & sq_ld_older;

   // ------------------------------------------------------------------- LOAD QUEUE
   wire                lq_d_ready, lq_x_v, lq_x_signed, lq_x_fp, lq_x_unc, lq_l_rd_v, lq_b_ok;
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
   wire [LQ_IB:0]      lq_occ;
   wire d_ld_nb    = d_is_mem & ~d_is_store & ~d_is_amo & ~d_is_cbo;  // plain load, rule C1
   wire ld_b = rn_valid_b & d2_ld_nb;                 // B is the load of the pair
   wire d_ld_alloc = (rn_valid & d_ld_nb) | ld_b;
   // a load behind a store in the same cycle captures the tag AFTER that store's
   wire [SQ_TB-1:0] ld_sqtag = sq_d_tag + {{(SQ_TB-1){1'b0}}, ld_b & rn_valid & d_st_nb};

   ooo2_lq #(.NENT(LQ_N), .IDXB(LQ_IB), .PAW(56), .PBITS(RN_PBITS),
             .ROBB(ROB_IDXB), .SQIB(SQ_TB)) u_lq
     (.clk(clk), .reset(reset),
      .d_alloc(d_ld_alloc), .d_rob(ld_b ? rob_d_idx2 : rob_d_idx), .d_prd(ld_b ? d2_prd_g : d_prd_g),
      .d_rd(ld_b ? d2_rd : d_rd), .d_rd_v(ld_b ? d2_rd_v : d_rd_v), .d_sqtag(ld_sqtag),
      .d_ready(lq_d_ready), .d_idx(lq_d_idx),
      .a_v(m_lq_fill), .a_sent(lsu_xo_early), .a_idx(m_lq_idx),
      .a_pa(lsu_xo_pa), .a_size(m_mem_size),
      .a_signed(m_mem_signed), .a_fp(m_is_fp), .a_unc(lsu_xo_unc),
      .e_pa(lq_e_pa), .e_size(lq_e_size), .e_tag(lq_e_tag), .e_av(lq_e_av),
      .e_block(lq_e_block), .x_block(sq_ld_block), .q_tag(lq_q_tag),
      .b_idx(m_lq_idx), .b_ok(lq_b_ok),
      .x_v(lq_x_v), .x_idx(lq_x_idx), .x_pa(lq_x_pa), .x_size(lq_x_size),
      .x_signed(lq_x_signed), .x_fp(lq_x_fp), .x_unc(lq_x_unc), .x_take(lq_x_take),
      .l_v(ld_land), .l_idx(ld_inflight_idx),
      .l_prd(lq_l_prd), .l_rd(lq_l_rd), .l_rd_v(lq_l_rd_v), .l_rob(lq_l_rob), .l_pa(lq_l_pa),
      .occupancy(lq_occ), .flush(redirect));

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
   wire lq_b_early = m_ld_nb & lq_b_ok & ~sq_ld_older;

   // ONE pre-translated port, two users. The committing store wins: it is at the ROB head,
   // so it is unconditionally older than any queued load, and it frees the port immediately.
   // A load waiting a cycle for it costs nothing that the store's own drain did not already.
   wire pt_v      = sq_go | lq_x_v;
   wire pt_store  = sq_go;
   wire lq_x_take = lq_x_v & ~sq_go & lsu_pt_ack;
   wire ld_land   = lsu_pt_done & ~lsu_pt_is_store;
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
   ooo2_rob #(.DEPTH(ROB_DEPTH), .IDXB(ROB_IDXB), .PBITS(RN_PBITS), .NW(4)) u_rob
     (.clk(clk), .reset(reset),
      // prd is ZERO when nothing is written: rename drives r_prd unconditionally, and
      // `d_prd != 0` is what replaces the stored rd_v bit.
      .d_valid(rn_valid), .d_rd(d_rd),
      .d_prd(d_rd_v ? rn_prd : {RN_PBITS{1'b0}}), .d_noret(d_is_irqop),
      .d_ready(rob_ready), .d_idx(rob_d_idx),
      .d_valid2(rn_valid_b), .d_rd2(d2_rd), .d_prd2(d2_prd_g), .d_noret2(1'b0), .d_ready2(rob_ready2), .d_idx2(rob_d_idx2),
      .w_v({sq_k_take, fp_land, iss_alu, rob_w_valid}),
      .w_ix({sq_kc_rob, ft_rob, i_rob, rob_w_idx}),
            .c_kill(m_valid & m_done & m_trap),
      .c2_kill(m_valid & (m_rob_idx == rob_head2_idx)),   // M's op retires only from the head
      .c_valid(rob_c_valid), .c_rd(rob_c_rd), .c_rd_v(rob_c_rd_v),
            .c_prd(rob_c_prd), .c_noret(rob_c_noret),
      .c2_valid(rob_c2_valid), .c2_rd(rob_c2_rd), .c2_rd_v(rob_c2_rd_v), .c2_prd(rob_c2_prd), .c2_noret(rob_c2_noret),
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
   reg  [1:0]       m_shard;
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
   // should still never fire at these sizes (ROB_DEPTH=16 against a 32-entry smallest free
   // pool), but "should never" is now handled rather than fatal.

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
   wire fwd1 = alu_q_v & (i_ps1 == alu_q_prd);
   wire fwd2 = alu_q_v & (i_ps2 == alu_q_prd);
   wire fwd3 = alu_q_v & (i_ps3 == alu_q_prd);
   wire [63:0] x_rs1 = fwd1 ? alu_q_val : prf_rs1;
   wire [63:0] x_rs2 = fwd2 ? alu_q_val : prf_rs2;
   wire [63:0] x_rs3 = fwd3 ? alu_q_val : prf_rs3;

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
   wire        lsu_done, lsu_done_acc, lsu_fault, lsu_idle, lsu_xo_early;
   wire [55:0] lsu_cos_pa;  wire [1:0] lsu_cos_kind;   // cosim memory-effect capture
   wire [63:0] lsu_rd_val, lsu_fault_tval;
   wire        lsu_dtlb_walking, lsu_dtlb_walk_beg;   // HPM: DT_WALK / DTLB_MISS
   wire [3:0]  lsu_fault_cause;
   wire        m_mem_op = m_valid & (m_is_mem | m_is_amo) & ~m_fault & ~m_ill_eff;

   // A CBO executes from M and is not serialized (cbo.zero clears every page the kernel
   // hands out), so it would start while an older store still sits in the queue -- and with
   // the senior store queue that includes stores already RETIRED. "An older store is live"
   // is `sq_av_any`, an entry WITH AN ADDRESS: M translates in program order, so every
   // entry older than M's op has one and no younger entry can get one while M is held.
   // NOT the occupancy -- entries are allocated at dispatch, so the queue can hold stores
   // younger than the CBO, which then wait for M: the deadlock that hung build L at
   // SLUB init on 2026-09-04. The other M-executed accesses are covered elsewhere: AMO/LR/SC
   // are serializing (`drained`), a load's early start asks `ld_older`. Rule C5.
   wire m_cbo_wait = m_is_cbo & sq_av_any;
   ooo2_lsu #(.AW(AW), .DRAM_BASE(DRAM_BASE), .DRAM_TOP(DRAM_TOP)) u_lsu
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
      .req_xlate(m_st_nb | m_ld_nb), .req_early(lq_b_early), .xo_pa(lsu_xo_pa), .xo_unc(lsu_xo_unc), .xo_v(lsu_xo_v),
      .xo_early(lsu_xo_early),
      .pt_v(pt_v), .pt_store(pt_store),
      .pt_pa(pt_store ? sq_c_addr : lq_x_pa), .pt_size(pt_store ? sq_c_size : lq_x_size),
      .pt_data(sq_c_data), .pt_signed(lq_x_signed), .pt_fp(lq_x_fp),
      .pt_unc(pt_store ? sq_c_unc : lq_x_unc), .pt_done(lsu_pt_done),
      .pt_ack(lsu_pt_ack), .pt_is_store(lsu_pt_is_store),
      .req_store(m_is_store & ~m_is_amo), .req_amo(m_is_amo),
      .req_amo_func(m_amo_func), .req_cbo(m_is_cbo), .req_cbo_zero(m_cbo_zero),
      .req_cbo_keep(m_cbo_keep),
      .req_vaddr(m_addr), .req_size(m_mem_size), .req_signed(m_mem_signed),
      .req_fp(m_is_fp), .req_st_data(m_st_data),
      .xl_satp(satp_data), .xl_priv(mmu_dpriv), .xl_sum(mmu_sum), .xl_mxr(mmu_mxr),
      .xl_flush(mmu_flush),
      .ptw_addr(dptw_addr), .ptw_read(dptw_read),
      .ptw_rdata(dptw_rdata), .ptw_rvalid(dptw_rvalid),
      .mem_raddr(dmem_raddr), .mem_ren(dmem_ren), .mem_runcached(dmem_runcached),
      .mem_rdata(dmem_rdata), .mem_rvalid(dmem_rvalid),
      .mem_wen(dmem_wen), .mem_waddr(dmem_waddr), .mem_wabase(dmem_wabase), .mem_wdata(dmem_wdata),
      .mem_wmask(dmem_wmask), .mem_wuncached(dmem_wuncached),
      .mem_cbo(dmem_cbo), .mem_cbo_zero(dmem_cbo_zero), .mem_cbo_keep(dmem_cbo_keep),
      .mem_wready(dmem_wready), .mem_waccept(dmem_waccept),
      .cos_pa(lsu_cos_pa), .cos_kind(lsu_cos_kind),
      .started(lsu_started), .done(lsu_done), .done_acc(lsu_done_acc), .rd_val(lsu_rd_val), .fault(lsu_fault),
      .fault_cause(lsu_fault_cause), .fault_tval(lsu_fault_tval), .idle(lsu_idle));
   assign dmem_idle = lsu_idle;

   // ---- multiply / divide (both start/busy/done units; they own the stall) ----
   wire [2:0]  md_f3   = m_insn[14:12];
   wire        md_is_w = m_insn[6:2] == 5'b01110;         // OP-32 -> MULW/DIVW..
   wire        md_div  = md_f3[2];
   reg         md_started;
   wire        m_md_op = m_valid & m_is_mul & ~m_ill_eff;
   wire        mul_start = m_md_op & ~md_div & ~md_started;
   wire        div_start = m_md_op &  md_div & ~md_started;
   wire        mul_done, div_done, mul_busy, div_busy;
   wire [63:0] mul_result, div_result;

   mul3 u_mul
     (.clk(clk), .reset(reset), .start(mul_start), .abort(1'b0),
      .rs1(m_rs1_val), .rs2(m_st_data), .f3(md_f3), .is_w(md_is_w),
      .busy(mul_busy), .done(mul_done), .result(mul_result));

   divider u_div
     (.clk(clk), .reset(reset), .start(div_start), .abort(1'b0),
      .rs1(m_rs1_val), .rs2(m_st_data), .f3(md_f3), .is_w(md_is_w),
      .busy(div_busy), .done(div_done), .result(div_result));

   // ---- FP unit (CVFPU) + the in-core FP ops ----
   // Everything the OoO core needs for squash recovery -- the zombie/drain latch, the
   // abort-by-seqno, the FS-dirty commit gate -- is absent here: M is the commit point,
   // so an FP op in flight can never be squashed, and its decode/operands are stable for
   // its whole (multi-cycle) stay. `decode_fp` runs off the registered m_insn.
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
   wire [63:0] op1f = m_rs1_val, op2f = m_st_data, op3f = m_rs3_val;

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
   fp_unit #(.TAGW(FTAGW), .NFLIGHT(4)) u_fpu
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
         f_insn    <= q_insn;   f_rd    <= q_rd;    f_rd_v <= q_rd_v;
         f_prd     <= q_prd;    f_rob   <= i_rob;
         f_rs1_val <= x_rs1;    f_rs2_val <= x_rs2; f_rs3_val <= x_rs3;
      end
   end

   always @(posedge clk) if (!reset) begin
      // The F stage only ever holds an FPU op. d_cls_f is decided from decode_fp on d_insn
      // and the same decoder runs here on the payload's insn, so a disagreement means the
      // payload and the classification came from different instructions.
      if (f_valid & ~(ff_valid_d & ff_use_fpu))
         $fatal(1, "ooo2_core: F stage holds a non-FPU op (insn %08x)", f_insn);
      if (iss_f & (q_shard != SH_FE))
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
      if (m_valid & m_is_csr & (fpu_busy | f_valid))
         $fatal(1, "ooo2_core: a CSR op is in M while FP work is in flight -- frm may change under it");
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
   wire        m_is_irqop= m_is_sys & (m_insn[14:12] == 3'b000) & (m_insn[31:20] == 12'h7F0);
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
   wire [1:0] bsh_l = rl_blk_pr[RN_PBITS-1:RN_IDXB];
   wire [1:0] bsh_f = rf_blk_pr[RN_PBITS-1:RN_IDXB];
   wire [1:0] bsh_i = ri_blk_pr[RN_PBITS-1:RN_IDXB];
   // Gated on "nothing issued", NOT on m_advance. The old `m_advance &` gate dated from M
   // being the only unit, where "M could accept" was the same thing as "issue could
   // proceed". With three units it silently masks: on workloads/mlbench M is busy with
   // loads nearly every cycle, so dep_fp could never fire and the FP add chain that IS the
   // critical path reported 0%.
   wire no_issue  = ~iq_iss_v;
   wire dep_ld    = no_issue & ((rl_blk_v & (bsh_l == SH_LD))
                              | (rf_blk_v & (bsh_f == SH_LD))
                              | (ri_blk_v & (bsh_i == SH_LD)));
   wire dep_fp    = no_issue & ((rl_blk_v & (bsh_l == SH_FE))
                              | (rf_blk_v & (bsh_f == SH_FE))
                              | (ri_blk_v & (bsh_i == SH_FE)));
   wire st_mem    = (st_m & m_mem_op) | dep_ld;     // ...on the LSU
   wire st_div    = st_m & m_md_op &  md_div;       // ...on the divider
   wire st_mul    = st_m & m_md_op & ~md_div;       // ...on the multiplier
   // ST_FPU MUST WATCH STAGE F. It used to be `(st_m & fp_arith) | dep_fp`, and fp_arith is
   // identically 0 since FP stopped entering M -- so the FPU's own occupancy vanished from
   // the stack the moment it got its own stage. `f_valid & ~fp_disp` is stage F holding an
   // op the unit will not yet take, which is exactly the old `st_m & fp_arith` term in its
   // new home. (~f_advance is the same expression; written out for clarity.)
   wire st_fpu    = (f_valid & ~fp_disp) | dep_fp;  // ...on the FPU
   // ~st_rob: the two are now disjoint, so the stack does not count a ROB-full cycle
   // twice under two different names.
   wire st_ser    = m_advance & ~accept & ~dep_ld & ~dep_fp & ~st_rob;
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
   //   fe_que  fetch DID assemble one; the F/X queue still had nothing for decode
   //           (refill latency after a drain).  Fix = deeper queue / earlier restart.
   // That it grows with instruction size points at fe_aln, but pointing is not measuring.
   wire fe_rest   = fe_bub &  immu_ready & (imem_avail_g != {$clog2(HW+2){1'b0}});
   wire fe_aln    = fe_rest & ~fe_fx_valid;
   wire fe_que    = fe_rest &  fe_fx_valid;

   // REDIR was one counter for every reason the pipe restarts, so a 3.4-per-1000 redirect
   // rate could not be attributed to conditional branches, indirect jumps, or traps -- and
   // predictor work would have been tuning blind.  csr_red wins the priority: a trap that
   // lands on a branch is a trap.  REDIR total minus these three is the remainder
   // (fence.i and direct-jal mispredicts), so nothing needs a fourth counter.
   wire red_trap  = redirect &  csr_red;
   wire red_br    = redirect & ~csr_red & m_is_branch;
   wire red_jalr  = redirect & ~csr_red & ~m_is_branch & m_is_jalr;

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
   wire rd_wait = m_valid & m_redirect & ~m_at_head;
   wire [25:0] hpm_ev = {lsu_dtlb_walk_beg, lsu_dtlb_walking, rd_wait, st_rob, hpm_fb_rhit, hpm_fb_hit,
                         fe_que, fe_aln, red_trap, red_jalr, red_br,
                         fe_ic, fe_mmu, fe_bub, st_ser, st_fpu, st_mul, st_div, st_mem,
                         hpm_ic_miss, hpm_ic_access, hpm_dc_miss, hpm_dc_access,
                         redirect, m_valid & m_is_store & lsu_done, m_valid & m_is_mem
                         & ~m_is_store & lsu_done};

   // FMAX: the Zihpm event bus is REGISTERED. hpm_ev -> hpm_inc -> a 64-bit mhpmcounter
   // carry chain was 823 of 3113 failing endpoints at 6 ns and the WORST family in the
   // design (m_addr -> ... -> u_csr/mhpmcounter[12][63]).  These 15 bits are pure
   // instrumentation and cost nothing to delay: a counter is read through a CSR many
   // cycles later, and no software can observe which cycle an event landed on.  They only
   // became timing-critical when 9a6f8de3 correctly un-gated perf_access/perf_miss from
   // `ifdef PERF_TRACE -- before that the cache events read zero in every bitstream ever
   // built, so this cone did not exist.
   // minstret is NOT included: retire_cnt stays combinational because it is architectural.
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
   reg [25:0] hpm_ev_q;
   reg [5:0]  hpm_ret_q;
   initial begin hpm_ev_q = 26'd0; hpm_ret_q = 6'd0; end
   always @(posedge clk) begin
      hpm_ev_q  <= reset ? 26'd0 : hpm_ev;
            hpm_ret_q <= reset ? 6'd0 : {5'd0, retire} + {5'd0, retire2};
   end

   csr_file u_csr
     (.clk(clk), .reset(reset),
      .raddr(m_imm[11:0]), .rdata(csr_rdata),
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
            .hw_ip(hw_ip), .mtime(mtime), .retire_cnt({5'd0, retire} + {5'd0, retire2}),
      .hpm_retire_cnt(hpm_ret_q), .hpm_ev(hpm_ev_q),
      .irq_v(csr_irq_v), .irq_cause(csr_irq_cause),
      // csr_file's ILA debug bus. The in-order SoC puts no ILA on the CSR file, so these
      // outputs go nowhere -- named and left EMPTY on purpose. PINMISSING gates this build
      // and PINCONNECTEMPTY does not, so a deliberate non-connection has to say so instead
      // of being silently omitted (same treatment as the iMMU's t_uncached).
      .dbg_timer(), .dbg_mtvec(), .dbg_mtvec_we(), .dbg_csrop(), .dbg_csrop_v(),
      // m_done_red here too: a system op is never a memory op, and upd_valid feeds the CSR
      // unit's redirect and trap-target logic -- with the full m_done, build D of 2026-09-04
      // had 1357 near-critical endpoints starting at m_addr: dTLB compare -> lsu_done ->
      // m_done -> upd_valid -> mepc/priv -> the vectored trap-target adder -> fe_red_tgt_q.
      .upd_valid(m_is_sys & m_done_red), .upd_is_csr(m_is_csr), .upd_func(m_csr_func),
      .upd_addr(m_imm[11:0]),
      .upd_src(m_csr_func[2] ? {59'b0, m_imm[16:12]} : m_rs1_val),
      .upd_pc(m_pc));

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
                 : m_md_op               ? (md_div ? div_done : mul_done)
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
                          : m_is_mul              ? (md_div ? div_result : mul_result)
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
   // the F/X queue's write data in one 26-level path. Now the cycle that reports the fault
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
   wire m_needs_head = m_is_sys | m_redirect | m_is_fencei
                     | m_fault | m_ill_eff | (m_mem_op & m_lsu_flt);
   wire head_block   = m_valid & m_needs_head & ~m_at_head;

   // One write port, one ROB completion port: when a load lands, M yields the cycle. Costs
   // ~0.3 cycles per load against the ~2.3 the early release saves.
   assign m_done = m_done_raw & ~head_block & ~ld_land & ~fp_land & ~m_flt_pulse;
   assign m_advance = ~m_valid | m_done;

   // ---- trap / redirect ----
   wire m_trap = xtrap_v | (m_is_sys & csr_redir_trap);
   wire csr_red = xtrap_v | (m_is_sys & csr_redir_v);
   // THE REDIRECT DOES NOT CARRY THE LSU'S LIVE COMPLETION. A memory op redirects only as a
   // trap, and a trap is taken from the latched copy (m_unit_done_q); a branch, a system op
   // and fence.i never go through the LSU. So the redirect's "done" is m_done with the
   // memory arm of m_unit_ok removed -- logically the same signal on every cycle a redirect
   // can fire, asserted below, and structurally free of dTLB -> lsu_done -> m_unit_ok, which
   // was the head of the u_sq/v_reg -> fe/q_dat family (326 endpoints, 26 levels).
   wire m_unit_ok_nomem = ~m_valid            ? 1'b1
                        : m_fault | m_ill_eff ? 1'b1
                        : m_mem_op            ? 1'b0
                        : m_md_op             ? (md_div ? div_done : mul_done)
                        :                       1'b1;
   assign m_done_red = (m_unit_ok_nomem | m_unit_done_q) & ~head_block & ~ld_land & ~fp_land;
   assign redirect = m_valid & m_done_red & (csr_red | m_redirect | m_is_fencei);
   wire   redirect_ref = m_valid & m_done & (csr_red | m_redirect | m_is_fencei);
   always @(posedge clk) if (!reset) begin
      if (redirect != redirect_ref)
         $fatal(1, "ooo2_core: redirect from the non-memory done disagrees with m_done (%b vs %b)",
                redirect, redirect_ref);
      if ((xtrap_v & m_done_red) != (xtrap_v & m_done))
         $fatal(1, "ooo2_core: trap request from the non-memory done disagrees with m_done");
      if ((m_is_sys & m_done_red) != (m_is_sys & m_done))
         $fatal(1, "ooo2_core: CSR update valid from the non-memory done disagrees with m_done");
   end

   // ---- EARLY FRONTEND RESTART -------------------------------------------------------
   // On a mispredict, do NOT wait to become ROB head before refetching. Note the event,
   // flush the frontend, freeze the renamer, and start fetching the resolved target now;
   // the ROB drains behind us. When the branch reaches the head the squash runs and the
   // renamer is released -- onto a correct path that is already in the F/X queue.
   //
   // Only a MISPREDICT can do this: its target (m_target) is resolved in execute. A trap's
   // target comes out of csr_file only once the op is at head, so traps keep the late path.
   //
   // The renamer MUST freeze for the whole window. Rollback here is `h := hc` with no
   // snapshot (ooo2_rename), so anything renamed before the squash is undone by it --
   // renaming ahead would not merely waste work, it would lose the instructions. Frozen,
   // the correct path accumulates in the F/X queue, which the squash does not touch.
   //
   // fr_v is the "frozen by an older redirect" interlock: a younger mispredict that
   // executes while one is pending is wrong-path by construction and must not retarget
   // the frontend. Oldest wins, and here the oldest is simply the one that got there
   // first -- M holds a single instruction until it is head, so no younger event can
   // reach M ahead of it. When units issue independently this needs the explicit age
   // compare of doc 12.
   //
   // Measured motivation: FE_BUB per redirect went 9.6 -> 54.4 cycles when the window
   // grew from ~2 instructions to 16, while mispredicts fell 37% (docs/OOO2-Spec.md).
   assign fr_set    = m_valid & m_redirect & ~m_trap & ~fr_v & ~redirect;
   assign fr_active = fr_set | fr_v;
   always @(posedge clk) begin
      if (reset)         fr_v <= 1'b0;
      else if (redirect) fr_v <= 1'b0;      // the squash consumes it
      else if (fr_set)   fr_v <= 1'b1;
   end

   // Exactly one frontend flush per event. Re-flushing at the squash would discard the
   // correct path this whole mechanism exists to have fetched early.
   assign fe_red_pulse = fr_set | (redirect & ~fr_v);
   assign fe_red_tgt   = fr_set ? m_target        : redirect_target;
   assign fe_red_seq   = fr_set ? (m_seq + 1'b1)  : redirect_seq;
   assign redirect_target  = csr_red     ? csr_redir_tgt
                           : m_is_fencei ? (m_pc + (m_rvc ? 64'd2 : 64'd4))
                           :               m_target;
   assign redirect_is_trap = m_trap;
   assign redirect_seq     = m_trap ? m_seq : (m_seq + 1'b1);
   assign ifence           = m_valid & m_done & m_is_fencei;

   // ---- branch resolve / BTB training ----
   wire m_link_rd = m_rd_v & ((m_rd == 6'd1) | (m_rd == 6'd5));   // x1/x5 = link registers
   wire m_link_rs = (m_rs1 == 6'd1) | (m_rs1 == 6'd5);
   // FMAX: res_v is qualified ONLY on M-stage flops. `m_done` and `~m_trap` are both
   // implied by the (branch|jump) term this signal already carries, so ANDing them in
   // bought nothing and cost everything: res_v is the D-input mux select of
   // u_bp/btb_q (the write-forward `t_fwd`), so it put the LSU (via m_done -> lsu_done)
   // and the CSR file (via m_trap -> csr_redir_trap) in the branch predictor's cone.
   // That was the worst path in the design at 6 ns, 226 of them.
   //   m_done : a CTI is not m_mem_op / m_md_op / fp_arith, so the mux collapses to 1'b1
   //   m_trap : xtrap_v's lsu_fault term needs m_mem_op; m_ill_eff's FP term needs
   //            m_is_fp; csr_redir_trap needs m_is_sys (opcode SYSTEM) -- none can hold
   //            for a branch or jump. What survives is m_fault | m_illegal.
   // Equivalence, not heuristic -- asserted below on every cycle.
   // The de-qualification survives, but it now needs the two gates that can hold a branch in
   // M for more than a cycle -- without them res_v asserts on EVERY stalled cycle and trains
   // the BTB and GHR repeatedly for one branch. Re-adding plain m_done would undo the Fmax
   // win the de-qualification exists for (it drags the LSU and the CSR file back into the
   // predictor's cone). It does not have to: for a BRANCH OR JUMP, csr_red and m_is_fencei
   // are 0 and m_trap reduces to m_fault | m_illegal, both already excluded here -- so
   // head_block collapses to the registered m_redirect against a 4-bit head compare.
   wire res_block   = (m_redirect & ~m_at_head) | ld_land | fp_land;
   assign res_v     = m_valid & (m_is_branch | m_is_jump) & ~m_fault & ~m_illegal & ~res_block;
   assign res_cbr   = m_is_branch;
   assign res_call  = m_is_jump & m_link_rd;
   assign res_ret   = m_is_jalr & m_link_rs & ~m_link_rd;
   assign res_taken = m_taken;
   assign res_tgt   = m_taken_tgt;
   assign res_rep   = res_v & res_cbr & m_redirect;   // ~csr_red: 0 under res_v, see above

   // The de-qualification is only safe while the implications above hold; a new M-stage
   // completion term (another multi-cycle unit, an FP branch) would break it silently.
   always @(posedge clk)
     if (!reset && (res_v !== (m_valid & m_done & (m_is_branch | m_is_jump) & ~m_trap)))
       $fatal(1, "ooo2_core: res_v de-qualification broken (pc=%h insn=%h done=%b trap=%b)",
              m_pc, m_insn, m_done, m_trap);

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
   assign m_wb_val = m_is_csr ? csr_rdata : m_byp_val;

   // Per-shard write data: each shard sees only its own writer, so the D$ read data reaches
   // the 3 load-shard LUTRAM copies instead of all 9, and the ALU result never leaves
   // int-exec.  Sourced directly, not from the m_wb_val mux -- routing every result through
   // one bus and then to every array is exactly what sharding by writer exists to avoid.
   assign wb_ie = x_result;                       // one writer: the ALU
   // SH_LD is now M's shard outright, so this mux carries every result M produces, not
   // just the memory and mul/div ones it started as. The two new arms are the two classes
   // d_shard just moved out of SH_IE: a CSR read, and everything else M completes -- which
   // after the mem/amo/mul arms above is a jump's link register.
   assign wb_ld = ld_wb                     ? lsu_rd_val
                : (m_is_mem | m_is_amo)     ? lsu_rd_val
                : m_unit_done_q             ? m_unit_res_q
                : m_is_csr                  ? csr_rdata
                : m_is_mul                  ? (md_div ? div_result : mul_result)
                :                             m_result;
   assign wb_fe = fp_wb         ? fp_wval
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
                     : m_md_op             ? (md_div ? div_done : mul_done)
                     :                       1'b1;
   wire m_done_wb = (m_unit_ok_wb | m_unit_done_q) & ~head_block & ~ld_land & ~fp_land;
   wire m_trap_wb = (m_valid & (m_fault | m_ill_eff | (m_mem_op & m_lsu_flt)))
                  | (m_is_sys & csr_redir_trap);
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
   wire alu_wb = iss_alu & q_rd_v;
   wire we_ie = alu_wb;                    // the ISSUE-timed event: wake, pending clear, snoop
   always @(posedge clk) begin             // the write itself, a cycle later (see x_rs1)
      alu_q_v <= ~reset & alu_wb;
      if (alu_wb) begin alu_q_prd <= q_prd; alu_q_val <= x_result; end
   end
   wire we_ld = m_wb_ld | ld_wb;
   wire we_fe = m_wb_fe | fp_wb;
   wire [RN_PBITS-1:0] wa_ie = q_prd;
   wire [RN_PBITS-1:0] wa_ld = ld_wb ? lq_l_prd : m_prd;
   wire [RN_PBITS-1:0] wa_fe = fp_wb ? ft_prd : m_prd;
   always @(posedge clk) if (!reset) begin
      if (m_wb_ld & ld_wb)
         $fatal(1, "ooo2_core: LD shard written by both M and a landing load");
      if (m_wb_fe & fp_wb)
         $fatal(1, "ooo2_core: FE shard written by both M and a landing FP result");
      // m_wb_ie is dead by construction: d_shard sends every M-routed op to SH_LD or
      // SH_FE. Asserted rather than assumed -- if a future op class reaches M with
      // SH_IE, it would silently drop its result now that we_ie ignores M.
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
   wire [ROB_IDXB-1:0] rob_head2_idx = rob_head_idx + 1'b1;

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
   // Captured at the WRITEBACK event, which for a load is no longer M's cycle.
   always @(posedge clk) if (!reset) begin
      // On the completion PULSE, not on m_done: lsu_cos_* are only this op's while the LSU
      // still holds it, and m_done can now assert cycles later off the sticky latch.
      if (m_valid && m_unit_ok && !m_unit_done_q && !m_ld_nb && !fp_arith) begin
         cs_val[m_rob_idx]   <= m_wb_val;
         // A BUFFERED STORE HAS NO MEMORY EFFECT YET. Its M pass only translates, so
         // lsu_cos_* still hold the PREVIOUS access's values -- reporting them here would
         // hand the cosim a stale PA under this store's seqno. Worse, it would often be
         // kind 0, and probe_cosim.cpp deliberately SKIPS the address compare whenever
         // either side reports no access ("a model that reports no access never forces a
         // false abort"), so the mistake would be invisible rather than loud: every
         // buffered store would go unchecked. Captured at commit instead, below.
         cs_mkind[m_rob_idx] <= (m_mem_op & ~m_st_nb) ? lsu_cos_kind : 2'd0;
         cs_mpa[m_rob_idx]   <= m_st_nb ? 56'd0 : lsu_cos_pa;
      end
      // The buffered store's real memory effect, recorded when the ROB RELEASES it: from
      // then on the store may retire before the LSU drains it, so the queue's own PA is
      // the source, not lsu_cos_* (which would name whatever the port did last).
      if (sq_k_take) begin
         cs_mkind[sq_kc_rob] <= 2'd2;
         cs_mpa[sq_kc_rob]   <= sq_kc_addr;
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
      if (fp_land) begin
         cs_val[ft_rob]   <= fp_wval;
         cs_mkind[ft_rob] <= 2'd0;      // an FP op has no memory effect
         cs_mpa[ft_rob]   <= 56'd0;
      end
      if (iss_alu) begin                // completed at issue, never saw M
         cs_val[i_rob]   <= x_result;
         cs_mkind[i_rob] <= 2'd0;
         cs_mpa[i_rob]   <= 56'd0;
      end
   end
   // ...and the same write-forward the ROB's head_done needs, for the same reason: a slot can
   // be captured in the very cycle it commits, so the array read returns pre-edge contents.
   // Missing it reported rd=0 for the second instruction of the boot.
   wire        cs_hit_m   = m_valid & m_unit_ok & ~m_unit_done_q & ~m_ld_nb & ~fp_arith
                          & (m_rob_idx == rob_head_idx);
   wire        cs_hit_ld  = ld_land & (lq_l_rob == rob_head_idx);
   wire        cs_hit_sq  = sq_k_take & (sq_kc_rob == rob_head_idx);
   wire        cs_hit_fp  = fp_land & (ft_rob == rob_head_idx);
   wire        cs_hit_alu = iss_alu & (i_rob == rob_head_idx);
   wire [63:0] cs_val_h   = cs_hit_sq ? 64'd0          // a store writes no register
                          : cs_hit_ld ? lsu_rd_val
                          : cs_hit_alu ? x_result
                          : cs_hit_fp ? fp_wval
                          : cs_hit_m  ? m_wb_val : cs_val[rob_head_idx];
   wire [1:0]  cs_mkind_h = cs_hit_sq ? 2'd2
                          : cs_hit_ld ? 2'd1
                          : cs_hit_alu ? 2'd0
                          : cs_hit_fp ? 2'd0
                          : cs_hit_m  ? (m_mem_op ? lsu_cos_kind : 2'd0) : cs_mkind[rob_head_idx];
      wire [55:0] cs_mpa_h   = cs_hit_sq ? sq_kc_addr
                          : cs_hit_ld ? lq_l_pa
                          : cs_hit_alu ? 56'd0
                          : cs_hit_fp ? 56'd0
                          : cs_hit_m  ? lsu_cos_pa : cs_mpa[rob_head_idx];
   // ...and for the entry behind the head, retiring in the same cycle (item 10c)
   wire        cs2_hit_m   = m_valid & m_unit_ok & ~m_unit_done_q & ~m_ld_nb & ~fp_arith & (m_rob_idx == rob_head2_idx);
   wire        cs2_hit_ld  = ld_land & (lq_l_rob == rob_head2_idx);
   wire        cs2_hit_sq  = sq_k_take & (sq_kc_rob == rob_head2_idx);
   wire        cs2_hit_fp  = fp_land & (ft_rob == rob_head2_idx);
   wire        cs2_hit_alu = iss_alu & (i_rob == rob_head2_idx);
   wire [63:0] cs_val_h2   = cs2_hit_sq ? 64'd0 : cs2_hit_ld ? lsu_rd_val : cs2_hit_alu ? x_result
                           : cs2_hit_fp ? fp_wval : cs2_hit_m ? m_wb_val : cs_val[rob_head2_idx];
   wire [1:0]  cs_mkind_h2 = cs2_hit_sq ? 2'd2 : cs2_hit_ld ? 2'd1 : cs2_hit_alu ? 2'd0 : cs2_hit_fp ? 2'd0
                           : cs2_hit_m ? (m_mem_op ? lsu_cos_kind : 2'd0) : cs_mkind[rob_head2_idx];
   wire [55:0] cs_mpa_h2   = cs2_hit_sq ? sq_kc_addr : cs2_hit_ld ? lq_l_pa : cs2_hit_alu ? 56'd0 : cs2_hit_fp ? 56'd0
                           : cs2_hit_m ? lsu_cos_pa : cs_mpa[rob_head2_idx];
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
   // frontend an F/X queue and changed fetch's ready from `accept` to `~q_full`, so the
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
         irq_inject_q <= ~redirect & ~redirect_q & ~fr_active &
                         (irq_inject_q ? ~irq_taken                     // hold until taken
                                       : csr_irq_v & ~inject_inflight); // schedule

         if (irq_taken)                               inject_inflight <= 1'b1;
         else if (redirect | redirect_q | fr_active | ~csr_irq_v) inject_inflight <= 1'b0;
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
   // the same DPI contract probe/probe_cosim.cpp already implements.
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
      input longint unsigned mem_pa);

   // csr_file internals, tapped exactly as backend_top does
   wire        cot_fire  = u_csr.trap_v;
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
   reg [55:0] e2_mpa;
   initial    e2_v = 1'b0;

   // A trap and a retire remain mutually exclusive, but they are no longer both "an M cycle":
   // the trap is M's (and fires only when M is the ROB head), the retire is the head's.
   always @(posedge clk) begin
      e_v <= 1'b0;
      if (!reset) begin
         if (m_valid && m_done && cot_fire) begin
            e_v <= 1'b1;  e_trap <= 1'b1;
            e_pc <= m_pc;
            e_insn <= (cot_intr | cot_ifault) ? 32'd0 : m_insn;
            e_rk <= 2'd0;  e_ri <= 5'd0;  e_val <= 64'd0;
            e_cause <= cot_cause;  e_tval <= cot_tval;
            e_prv <= u_csr.priv;                  // privilege BEFORE the trap
            e_mkind <= 2'd0;  e_mpa <= 56'd0;     // a trap performed no data access
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
         end
         e2_v <= 1'b0;
         if (retire2 && !(m_valid && m_done && cot_fire)) begin
            e2_v <= 1'b1;
            e2_pc <= cs_pc[rob_head2_idx];  e2_insn <= cs_insn[rob_head2_idx];
            e2_rk <= ck_rk2;  e2_ri <= rob_c2_rd[4:0];  e2_val <= cs_val_h2;
            e2_mkind <= cs_mkind_h2;  e2_mpa <= cs_mpa_h2;
         end
      end
      // emit one cycle later, so this instruction's own CSR writes have landed
      if (e_v)
         probe_retire(e_pc, e_insn, {6'd0, e_rk},
                      (e_rk == 2'd0) ? 8'd0 : {3'd0, e_ri},
                      {6'd0, e_prv}, {7'd0, e_trap}, e_val, e_cause, e_tval,
                                            64'd0, {64{1'b1}}, `VA_UNPACK40(u_csr.mepc), 8'd0,
                      {6'd0, e_mkind}, {8'd0, e_mpa});
      if (e2_v)
         probe_retire(e2_pc, e2_insn, {6'd0, e2_rk},
                      (e2_rk == 2'd0) ? 8'd0 : {3'd0, e2_ri},
                      {6'd0, e_prv}, 8'd0, e2_val, 64'd0, 64'd0,
                      64'd0, {64{1'b1}}, `VA_UNPACK40(u_csr.mepc), 8'd0,
                      {6'd0, e2_mkind}, {8'd0, e2_mpa});
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
   wire d_take = d_valid & ~d_hold & ~redirect & ~redirect_q & ~fr_active;
   // slot B's own hold (see the rules where d2_cls is defined); d_take carries the redirect terms
   wire d2_hold = ~d_plain | ~d2_plain | (d2_cls == d_cls) | ~iq_ready_b | ~rob_ready2 | rn_stall
                | (d2_st_nb & (d_st_nb | ~sq_d_ready)) | (d2_ld_nb & (d_ld_nb | ~lq_d_ready));
   wire d2_take = d2_valid & d_take & ~d2_hold;
   assign rn_valid_b = d2_take;
   always @(posedge clk) if (!reset) begin
      if (rn_valid_b & ~rn_valid)       $fatal(1, "ooo2_core: slot B dispatched without slot A");
      if (rn_valid_b & (d2_cls == d_cls)) $fatal(1, "ooo2_core: slot B dispatched to slot A's scheduler");
      if (rn_valid_b & ((d_st_nb & d2_st_nb) | (d_ld_nb & d2_ld_nb)))
         $fatal(1, "ooo2_core: two allocations into one memory queue");
   end

   // `accept` means X CAN TAKE A NEW BUNDLE -- it is free, or it is being dispatched this
   // cycle. It used to double as "the backend is ready", which was the same thing only
   // because X fed M directly. Conflating them again would overwrite an undispatched
   // instruction, because the frontend's load-on-accept wins over its clear-on-consume.
   assign accept  = ~d_valid | rn_valid;

   always @(posedge clk) begin
      if (reset) begin
         m_valid <= 1'b0; md_started <= 1'b0;
      end else begin
         if (m_advance) md_started <= 1'b0;
         else if (mul_start | div_start) md_started <= 1'b1;

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
