`include "va_codec.vh"
`include "smolrv64_irec.vh"
// An issued op's payload, as dispatch packs it (pl_in) and an issue port unpacks it.
// PL_DECL(P) declares the fields under prefix P; PL_REC(P) is their order in the payload.
`define PL_DECL(P) wire [PCW-1:0] P``pc, P``pred_npc, P``fault_tval; wire [31:0] P``insn; \
   wire [SQ_IB-1:0] P``sq_tag; wire [LQ_IB-1:0] P``lq_idx; \
   wire P``rvc, P``rd_v, P``mem_signed, P``is_mem, P``is_store, P``is_amo; \
   wire [SEQW-1:0] P``seq; wire [PDW-1:0] P``pdet; wire [5:0] P``rd, P``rs1; \
   wire [RN_PBITS-1:0] P``prd; wire [2:0] P``shard; wire [1:0] P``mem_size; wire [63:0] P``imm; \
   wire [4:0] P``amo_func; wire P``is_branch, P``is_jump, P``is_jalr, P``is_mul, P``is_csr; \
   wire [2:0] P``csr_func; wire P``is_serialize, P``is_fp, P``is_fencei, P``is_cbo, P``cbo_zero; \
   wire P``cbo_keep, P``illegal, P``fault; wire [3:0] P``fault_cause; wire [5:0] P``alu_op; \
   wire P``alu_w, P``alu_uw, P``op2_imm, P``res_link, P``mis_taken, P``mis_nt; \
   wire [1:0] P``op1_sel; wire [2:0] P``br_func; wire P``rs1_v, P``rs2_v, P``rs3_v, P``ord;
`define PL_REC(P) {P``pc, P``insn, P``rvc, P``seq, P``pdet, P``pred_npc, P``rd, P``rd_v, P``prd, P``shard, \
   P``rs1, P``imm, P``mem_size, P``mem_signed, P``is_mem, P``is_store, P``is_amo, \
   P``amo_func, P``is_branch, P``is_jump, P``is_jalr, P``is_mul, P``is_csr, P``csr_func, \
   P``is_serialize, P``is_fp, P``is_fencei, P``is_cbo, P``cbo_zero, P``cbo_keep, \
   P``illegal, P``fault, P``fault_cause, P``fault_tval, \
   P``alu_op, P``alu_w, P``alu_uw, P``op1_sel, P``op2_imm, P``res_link, \
   P``br_func, P``mis_taken, P``mis_nt, \
   P``rs1_v, P``rs2_v, P``rs3_v, P``ord, P``sq_tag, P``lq_idx}
`default_nettype none

// In-order pipelined RVA22S64 core: F | X | M.
//
//   F : PC -> iMMU -> I$ window -> aligner -> RVC expand -> decode   (smolrv64_frontend)
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
`ifndef SMOLRV64_HW
 `define SMOLRV64_HW 8                 // fetch window halfwords: 16 bytes, the shipping build since 2026-09-05 (4 before)
`endif
`ifndef SMOLRV64_IW
 `define SMOLRV64_IW 3                 // pipeline width (instructions/cycle); 3 = shipping. Stage 3 width knob.
`endif
module smolrv64_core
  #(parameter PCW  = 64,
    parameter SEQW = 8,
    parameter HW   = `SMOLRV64_HW,
    parameter IW   = `SMOLRV64_IW,
    parameter AW   = 64,
    // smolrv64_predictor's predict-detail width: {rsp, ghr, yhit, yctr, yidx, hit, ctr}, 3+11+14+3 bits.
    parameter PDW   = 31,
    parameter [PCW-1:0] RESET_PC = 0,
    parameter [63:0] LBASE    = 64'h7000_0000,   // the local SRAM, for the LSU's alignment rule
    parameter        LRAM_LG2 = 18,
    parameter        PABITS   = 36,              // the architectural physical-address width
    parameter        LQ_IB    = 3)               // the load queue's index width: 1<<LQ_IB entries
   (input  wire                    clk,
    input  wire                    reset,
    // ---- instruction memory: the fetch ring's stream into the I$ (rv_icache) ----
    output wire [PCW-1:0]          imem_addr,       // the fetch PC's PA (diagnostics)
    output wire                    imem_ctx_chg,    // a mapping change: the I$ advances its epoch
    output wire                    imem_hold,       // fetch is past a weakly predicted branch: an I$ miss waits
    output wire                    imem_cancel,     // the fetch stream restarts: a waiting I$ miss is stale
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
    output wire                    fe_redirect,
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
    input  wire                    dmem_wroom,         // the D$ takes a store presented this cycle
    output wire                    dmem_idle,          // no memory op in flight and no store queued (fence.i drain)
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
    output wire [IW-1:0]           retire,             // commit port k retired an instruction (k = 0: the ROB head)
    output wire [IW*PCW-1:0]       retire_pc,
    output wire [IW*32-1:0]        retire_insn,
    output wire                    redirect,
    output wire [PCW-1:0]          redirect_target,
    // The LSU's invariants, on their way to the SoC's integrity log (rv_errlog). Pure
    // pass-through: registered in the LSU, read in rv_soc_top, nothing in between.
    output wire [15:0]             lsu_err,
    output wire [15:0]             fe_err,         // the frontend's invariants (smolrv64_frontend)
    output wire [127:0]            core_dbg);      // why nothing retires: the board's wedge ILA (ILA_MEM)

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
   // AN FP INSTRUCTION WITH mstatus.FS OFF IS ILLEGAL AT DISPATCH (decode folds it into the
   // record's illegal bit, smolrv64_dslot). FS changes only by a CSR write, which serialises, so
   // at dispatch it already reflects every older instruction; the op then traps from the SYSQ
   // like any other illegal instruction.
   wire [15:0]              d_gc;                   // smolrv64_gclass, from the frontend's decode
   localparam integer GC_F = 0, GC_C = 1, GC_M = 2, GC_S = 3, GC_L = 4, GC_I = 5, GC_FC = 6,
                      GC_PLAIN = 7, GC_IRQOP = 8, GC_ORD = 9, GC_FPV = 10, GC_DCR = 11,
                      GC_LD = 12, GC_ST = 13, GC_CSR = 14, GC_SER = 15;
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
   // The fetch ring (smolrv64_fring, in the frontend) holds bytes read under a real translation of
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
         $fatal(1, "smolrv64_core: the fetch ring serves a window on a faulting translation (va=%h cause=%0d)", imem_va, immu_cause);
   // The count, for the counters only (FE_QUE's "no bytes" attribution below).
   wire [$clog2(HW+2)-1:0]  imem_avail_g = imem_ok ? imem_avail : {$clog2(HW+2){1'b0}};
   // resolve/training port (driven from M, below)
   wire                     res_v, res_cbr, res_call, res_ret, res_taken;
   wire [PCW-1:0]           res_tgt;
   wire                     redirect_is_trap;
   wire [SEQW-1:0]          redirect_seq;
   wire                     fe_red_pulse, fr_set;
   // The predict details' fields the core reads: the RAS-top snapshot is the top field, the
   // history snapshot the next (smolrv64_predictor's pd_mk/pd_no).
   localparam integer RASB   = 3;
   localparam integer GHL    = 11;
   localparam integer PD_RSP = PDW - RASB;                   // the RAS snapshot's LSB
   localparam integer PD_GHR = PD_RSP - GHL;                 // the history snapshot's LSB
   reg  [RASB-1:0]          fe_red_rsp_q;                    // the RAS top the redirect restores
   reg  [GHL-1:0]           fe_red_ghr_q;                    // ...and the history
   wire [PCW-1:0]           fe_red_tgt;
   wire [SEQW-1:0]          fe_red_seq;
   // decode-stage direct-CTI redirect (static JAL / backward-branch resteer): assigned
   // after the dispatch steering; each slot's detection (s_dcr) is in the slot block.
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
      res_pdet_q  <= tr_pdet;       // control flow resolves in the lanes
      res_tgt_q   <= res_tgt;
      res_pc_q    <= tr_pc;
      res_rvc_q   <= tr_rvc;
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

   localparam DCR_HIT = 2;   // BIMW-1: the BTB-hit bit inside the carried predict details
   localparam integer DCR_BACK = 1;   // static-taken for a BACKWARD conditional branch on a BTB miss (loop back-edge bet)
   smolrv64_frontend #(.PCW(PCW), .SEQW(SEQW), .HW(HW), .IW(IW), .PDW(PDW), .RASB(RASB), .DCR_HIT(DCR_HIT), .DCR_BACK(DCR_BACK),
                  .RESET_PC(RESET_PC)) fe
     (.clk(clk), .reset(reset), .accept(accept), .consume(fe_consume),
      .d_valid(fe_dv), .d_rec(fe_rec),
      .fs_off(fs_off), .crd(crd), .hd_v(fe_hd_v), .hd_take(fe_hd_take), .hd_gc(fe_hd_gc),
      .redirect(fe_red_q), .redirect_pc(fe_red_tgt_q), .redirect_seq(fe_red_seq_q), .redirect_rsp(fe_red_rsp_q), .redirect_ghr(fe_red_ghr_q),
      .irq_inject(irq_inject), .irq_taken(irq_taken), .fe_dq_valid(fe_dq_valid),
      .imem_addr(imem_va), .imem_ipc(), .imem_pa(imem_addr), .imem_xlvl(immu_lvl),
      .imem_xlate_ok(immu_ready & ~immu_fault), .imem_freeze(ic_busy | imem_ctx_chg), .imem_flush(imem_cancel),
      .fe_avail(imem_avail), .fe_ok(imem_ok),
      .ic_req(ic_req), .ic_va(ic_va), .ic_pa(ic_pa), .ic_tag(ic_tag),
      .ic_ack(ic_ack), .ic_valid(ic_valid), .ic_data(ic_data), .ic_rtag(ic_rtag),
      .imem_fault(immu_ready & immu_fault), .imem_cause(immu_cause),
      .res_v(res_v_q), .res_cbr(res_cbr_q), .res_call(res_call_q), .res_ret(res_ret_q),
      .res_taken(res_taken_q), .res_pdet(res_pdet_q), .res_tgt(res_tgt_q),
      .res_pc(res_pc_q), .res_rvc(res_rvc_q),
      .cur_seq(fe_cur_seq), .fe_err(fe_err_f));

   // the dispatch group: slot k's decoded record (smolrv64_irec.vh), unpacked under its names
   localparam integer IRW = `IR_RECW;
   wire [IW-1:0]     fe_dv;
   wire [IW*IRW-1:0] fe_rec;
   wire [IW-1:0]     fe_consume;
   assign d_valid = fe_dv[0];  assign `IR_REC(d_) = fe_rec[0 +: IRW];   // slot A by name
   assign fe_consume = s_takev;

   // ---- decode-stage direct-CTI redirect (static) -------------------------------------
   // Take a control transfer the predictor called fall-through AT DISPATCH, instead of
   // eating a full exec-resolved mispredict: a JAL (unconditionally taken) or a BACKWARD
   // conditional branch (the loop-back bet). d_mis_taken is the frontend's precomputed
   // (taken_target != pred_npc), so it is 1 exactly when the BTB did not already steer to
   // the taken target. d_pdet[DCR_HIT] is the BTB hit bit ({hit,ctr} = the low BIMW bits of
   // the carried details): gate BACKWARD BRANCHES on a MISS so a TRAINED not-taken bimodal
   // is never overridden by the static-taken bet; a JAL fires on any miss/wrong-target.
   // The target (dec_red_tgt below) is d_pc+d_imm -- a registered decode-stage value, off
   // the guarded fetch array cone (docs/rtl-rules.md, smolrv64_predictor's timing note).
   // Suppress the decode-redirect while an interrupt pseudo-op is being presented or is in
   // flight: fe_red would flush it out of the frontend without the irq FSM's redirect_q/fr_v
   // ever seeing it, wedging inject_inflight forever (the rv64mi-p-illegal vectored-interrupt
   // spin). The interrupt must make progress; a JAL that coincides falls back to the exec
   // redirect -- rare, and cheaper than a livelock.
   wire dcr_arm = ~irq_inject & ~inject_inflight;


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
      $fatal(1, "smolrv64_core: DRAM_TOP=%h lies beyond the %0d-bit physical address space", DRAM_TOP, PABITS);

   // instruction-side translation. M-mode fetches are physical (satp forced Bare).
   wire [63:0] mmu_satp;
   wire [1:0]  mmu_priv, mmu_dpriv;
   wire        mmu_sum, mmu_mxr, mmu_flush;
   // THE TLB FLUSH IS A REGISTER: csr_file's is its system op's live fire (an sfence.vma or a satp
   // write), and through the fetch freeze and the iTLB it reached the I$ request and the fetch
   // ring. It takes effect a cycle later, the cycle the op's own redirect reaches the front end
   // (fe_red_q), and the freeze it raises then withholds that cycle's fetch, so no fetch uses a
   // stale translation; every younger op dies at the redirect.
   reg         mmu_flush_q;
   initial     mmu_flush_q = 1'b0;
   always @(posedge clk) mmu_flush_q <= ~reset & mmu_flush;
   wire [63:0] satp_fetch = (mmu_priv  == 2'd3) ? 64'd0 : mmu_satp;
   wire [63:0] satp_data  = (mmu_dpriv == 2'd3) ? 64'd0 : mmu_satp;

   mmu #(.AW(56), .DRAM_TOP(DRAM_TOP), .TLBN(64), .TLBI(6)) u_immu
     (.clk(clk), .reset(reset),
      .req_valid(1'b1), .req_vaddr(imem_va), .req_access(2'd0),
      .priv(mmu_priv), .sum(mmu_sum), .mxr(mmu_mxr), .satp(satp_fetch), .flush(mmu_flush_q),
      .ptw_addr(ptw_addr), .ptw_read(ptw_read),
      .ptw_rdata(ptw_rdata), .ptw_rvalid(ptw_rvalid),
      .walking(), .t_ready(immu_ready), .t_paddr(immu_pa), .t_fault(immu_fault),
      .t_cause(immu_cause), .t_lvl(immu_lvl), .t_uncached(), .t_ok(), .t_fault_raw(),
      .walk_ok(~walk_hold),
      .s_vaddr(64'd0), .s_access(2'd1), .s_ok(), .s_flt(), .s_cause(), .s_paddr(), .s_nc());   // the second lookup is the dTLB's
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
   assign imem_ctx_chg  = mmu_flush_q | (isatp_q != satp_fetch);
   assign fe_redirect   = fe_red_pulse;
   assign imem_satp_q   = isatp_q;
   assign imem_priv_q   = ipriv_q;

   // =========================================================== stage X
   // the architectural writes of commit port k, for tb_smolrv64_riscv's trace
   wire [IW-1:0]    rf_we;
   wire [IW*6-1:0]  rf_wa;
   wire [IW*64-1:0] rf_wd;

   // ---- renaming and the sharded PRF ----------------------------------------------
   localparam integer RN_IDXB  = 7;
   localparam integer RN_PBITS = RN_IDXB + 3;   // 3 shard bits: room for a 5th shard (the 3rd ALU)
`include "smolrv64_shards.vh"
   localparam integer NL = IW;                     // the lanes: slot k's ALU-class ops go to lane k
   function automatic [3:0] ones_nl(input [NL-1:0] v);   // how many of the lanes' bits are set
      integer i;
      begin ones_nl = 4'd0; for (i = 0; i < NL; i = i + 1) ones_nl = ones_nl + {3'd0, v[i]}; end
   endfunction
   function automatic sh_used(input [2:0] sh);     // lanes 0..IW-1 and FP slices 4..4+IW-1
      sh_used = ({1'b0, sh[1:0]} < IW[2:0]);
   endfunction


   // An f-register destination takes the FP file's slice of its slot (SH_F0/F1/F2 for A/B/C),
   // whoever writes it: the file has a bank per writer (smolrv64_prf). Every other destination
   // takes its slot's lane shard (SH_IE/IE2/IE3), whatever produces it: the lane's write
   // register is the shard's one writer, carrying its ALU and multiply results and the
   // landings of every other unit (loads, AMOs, CSR reads, the FP ops' integer results,
   // divides). Assigned after the class wires below (d_cls_f is declared there).

   // rename's ports, a field per slot (slot k at [k])
   wire [IW*RN_PBITS-1:0] rn_prs1v, rn_prs2v, rn_prs3v, rn_sprs1v, rn_sprs2v, rn_sprs3v;
   wire [IW*RN_PBITS-1:0] rn_mprs1v, rn_mprs2v, rn_mprs3v, rn_prdv;
   wire [IW-1:0]          rn_lv1v, rn_lv2v, rn_lv3v, rn_byp1v, rn_byp2v, rn_byp3v;
   wire [IW*6-1:0]        s_rs1v, s_rs2v, s_rs3v, s_rdv;
   wire [IW-1:0]          s_rd_vv, s_takev;
   wire [IW*3-1:0]        s_shardv;
   wire                rn_stall;
   wire [7:0]          rn_shard_low;   // per shard
   // Rename exactly when the instruction is dispatched (d_take: structural room, no fault
   // replay, not the cycle after a redirect). It MAY be renamed in the redirect cycle
   // itself: that instruction is younger than the redirecting op and the rename's flush arm
   // rolls the pointers back over it (rule I11) -- gating on the same-cycle redirect put M's
   // completion in front of every dispatch write (gate V3, 2026-09-05).
   wire rn_valid = d_take;      // dispatch is no longer gated on M being free

   smolrv64_rename #(.IDXB(RN_IDXB), .IW(IW)) u_rename
     (.clk(clk), .reset(reset),
      .r_valid(s_takev), .r_cand(fe_dv),
      .r_rs1(s_rs1v), .r_rs2(s_rs2v), .r_rs3(s_rs3v), .r_rd(s_rdv), .r_rd_v(s_rd_vv), .r_shard(s_shardv),
      .r_prs1(rn_prs1v), .r_prs2(rn_prs2v), .r_prs3(rn_prs3v),
      .r_sprs1(rn_sprs1v), .r_sprs2(rn_sprs2v), .r_sprs3(rn_sprs3v),
      .r_mprs1(rn_mprs1v), .r_mprs2(rn_mprs2v), .r_mprs3(rn_mprs3v),
      .r_lv1(rn_lv1v), .r_lv2(rn_lv2v), .r_lv3(rn_lv3v),
      .r_byp1(rn_byp1v), .r_byp2(rn_byp2v), .r_byp3(rn_byp3v),
      .r_prd(rn_prdv),
      // commit comes from the ROB head and the entries behind it
      .c_valid(rc_v), .c_rd(rc_rd), .c_rd_v(rc_rdv), .c_prd(rc_prd),
      .flush(redirect),
      .stall(rn_stall), .shard_low(rn_shard_low));

   wire [63:0]            prf_sq;                     // the store queue's data read
   wire [63:0]            prf_f1, prf_f2, prf_f3;     // the F/CTF port's
   wire [NL-1:0]          l_qvv;                      // the lanes' write registers, packed
   wire [NL*RN_PBITS-1:0] l_qprdv;
   wire [NL*64-1:0]       l_qvalv;
   wire [2*NL*RN_PBITS-1:0] l_psv;                    // the lanes' operand tags, packed...
   wire [2*NL*64-1:0]     l_opv;                      // ...and their values
   // Operands are read AT ISSUE, addressed by the entry the scheduler selected -- doc 1's
   // "values live in one place".
   smolrv64_prf #(.IDXB(RN_IDXB), .NL(NL)) u_prf
     (.clk(clk),
      .we_l(l_qvv), .wa_l(l_qprdv), .wd_l(l_qvalv),
      .we_ld(we_ld), .wa_ld(wa_ld), .wd_ld(wb_ld),
      .we_fe(we_fe), .wa_fe(wa_fe), .wd_fe(wb_fe),
      .ra_l(l_psv), .rd_l(l_opv),
      .ra_sq(sq_r_preg), .rd_sq(prf_sq),
      .ra_f({j_ps3, j_ps2, j_ps1}), .rd_f({prf_f3, prf_f2, prf_f1}));

   // ---- reorder buffer, running as a SHADOW ------------------------------------------
   // The step from "M is the commit point" to "the ROB head is the commit point" -- the
   // substrate out-of-order issue needs. Brought up exactly the way rename was (rule I3):
   // it is driven by the real instruction stream and its commit decision is CHECKED against
   // the live one every cycle, while still driving nothing. Switching smolrv64_rename's commit
   // port over is then one line against a proven structure.
   //
   // It is small because smolrv64_rename already did the hard part: SMAP/RMAP/lv and a per-shard
   // free list with separate speculative and committed heads, where rollback is a pointer
   // restore. That already supports N uncommitted instructions; N is only ever 1 today
   // because M blocks. So the ROB holds the commit RECORD and re-orders it, nothing else.
   localparam integer ROB_DEPTH = 32, ROB_IDXB = 5;
   wire [IW*ROB_IDXB-1:0] rob_d_idxv;           // the ROB slot each slot takes
   wire [IW-1:0]          rob_readyv;           // [k]: room for k+1
   wire                   rob_empty;
   wire [ROB_IDXB:0]   rob_occ;
   wire                ldw_int;            // the LD stream's write goes to a lane (completes at its drain)
   wire                few_int;            // ...and the FE stream's
   wire                sy_lane_ok;                   // lane A's slot is free for the SYSQ
   // the landing buffers (defined with the LD stream): per lane, LBN entries
   localparam integer LBN = 4, LBB = 2;
   reg  [RN_PBITS-1:0] lb_prd [0:NL*LBN-1];
   reg  [63:0]         lb_dat [0:NL*LBN-1];
   reg  [ROB_IDXB-1:0] lb_rob [0:NL*LBN-1];
   reg  [LBB-1:0]      lb_h [0:NL-1], lb_t [0:NL-1];
   reg  [LBB:0]        lb_n [0:NL-1];
   reg                 lb_fe [0:NL*LBN-1];  // the entry is the FE stream's (the stall counters)

   // the ROB's completion ports: a lane's op at issue (a multiply in its reserved slot, a
   // mispredicting CTI at its squash; a load or store never), the FP and MD landings, the store
   // queue, M's port (which carries the landing loads). Lanes 1.. take ports 4.. (the lane
   // generate assigns them).
   localparam integer ROB_NW = NL + 5;
   wire [ROB_NW-1:0]          rob_wv;
   wire [ROB_NW*ROB_IDXB-1:0] rob_wix;
   assign rob_wv[3:0] = {sq_k_take, fp_land & ~(fp_wb & few_int & ~(|by2)), l_wv[0], rob_w_valid & ~(ldw_int & ~(|by0))};
   assign rob_wix[4*ROB_IDXB-1:0] = {sq_kc_rob, ft_rob, l_wix[0], rob_w_idx};
   assign rob_wv[ROB_NW-1 -: 2] = {md_wb & ~(md_wr & few_int & ~(|by2)), cf_red_fire};
   assign rob_wix[ROB_NW*ROB_IDXB-1 -: 2*ROB_IDXB] = {md_rob, fr_rob};
   wire                lq_d_ready2, sq_d_ready2;
   wire [IBF:0]        rf_free;
`include "smolrv64_credits.vh"
   wire [CRW-1:0]      crd;               // the dispatch credits
   wire                fe_hd_v, fe_hd_take;   // the queue head: an instruction, and whether it pops
   wire [15:0]         fe_hd_gc;
   // Whether the M instruction is the OLDEST in flight. Once M stops blocking, a trap or a
   // redirect may only fire when it is: the trapping instruction is YOUNGER than an
   // outstanding load, and `flush` would otherwise kill that older entry and lose its
   // register write. Not consumed yet -- see the note above lsu_started.
   wire [ROB_IDXB-1:0] rob_head_idx;
   wire                m_at_head = (rob_head_idx == m_rob_idx);
   // the ROB's commit ports: k = 0 is the head, k the entry k behind it (rc_idx[k])
   wire [IW-1:0]          rc_v, rc_rdv, rc_noret, rc_kill, rc_f;
   wire [IW*6-1:0]        rc_rd;
   wire [IW*RN_PBITS-1:0] rc_prd;
   wire [ROB_IDXB-1:0]    rc_idx [0:IW-1];
   // Mirrors m_is_irqop (m_is_sys & funct3==0 & imm==0x7F0) one stage earlier; an op that
   // traps at dispatch never reaches M, so this decode is exact.
   wire d_is_irqop = d_gc[GC_IRQOP];
   reg  [ROB_IDXB-1:0] m_rob_idx;          // rides with the op, names its slot at completion
   initial m_rob_idx = {ROB_IDXB{1'b0}};
   always @(posedge clk) if (m_advance) m_rob_idx <= m_go_rob;

   // Completion. While M blocks this is just "M finished", so the head is always the M
   // instruction; when the blocking is cut, this becomes one input per unit.
   // A load's done bit is set when its DATA lands, not when M released it -- otherwise the
   // ROB would commit it before its register write exists. A FAULTING load never sets done
   // at all: it traps, and the redirect's flush retires the entry.
   // An ALU op completing at issue is the SECOND completion port: it never enters M, so
   // its ROB entry has to be marked done from here. This is why smolrv64_rob's port was widened.
   // FP has its OWN completion port now. It used to share this one, which is why a landing
   // FP result had to be held whenever a load landed in the same cycle -- with several FP
   // ops in flight that collision stops being rare, and holding stops being cheap.
   // A plain store leaves M with no result AND no memory effect yet -- the buffer owns both.
   // Its ROB slot is completed by sq_c_take above, in the cycle memory is actually written.
   wire m_st_nb   = m_is_store & ~m_is_amo & ~m_is_cbo;
   wire m_sq_fill = m_valid & m_st_nb & lsu_xo_v;
   wire m_lq_fill = m_valid & m_ld_nb & lsu_xo_v;
   wire rob_w_valid = (m_valid & m_done & ~m_ld_nb & ~m_st_nb) | ld_land;
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
   // The integrity log's top three bits carry the backend's invariants on the frontend's bus:
   // a writeback to a register that is not pending (a squashed op's result), a register allocated
   // while pending, and an FPU result with nothing in flight or overwriting an unconsumed one.
   wire [1:0]  pend_err;
   wire        fpu_err;
   wire [15:0] fe_err_f;
   assign fe_err = {fpu_err, pend_err, fe_err_f[12:0]};
   // the queries: each slot's six map candidates (rs1..rs3 speculative, then committed), then
   // each lane's two operands (the issue check below)
   localparam integer NQ = 6*IW + 2*NL;
   wire [NQ-1:0] pnd_q;
   wire [6*IW*RN_PBITS-1:0] pnd_sq;              // slot k's six candidates at [6k..6k+5]
   wire [IW-1:0]            pnd_av;
   smolrv64_pending #(.PBITS(RN_PBITS), .NS(IW), .NWB(NWB_C), .NQ(NQ)) u_pend
     (.clk(clk), .reset(reset),
      // EVERY CANDIDATE'S PENDING BIT IS SET, TAKEN OR NOT: the set enable is the slot's own
      // valid and destination and the registered rename stall, never the dispatch take, whose
      // late terms (mstatus.FS through the illegal decode, the queues' room, the redirect) fanned
      // into all 1,024 pending flops. A candidate is the head of its free list (the stall keeps
      // the group inside the free set), so an untaken one is a free register no source names,
      // and it is set again when it is taken.
      .a_v(pnd_av), .a_preg(rn_prdv),
      .w_v(wkv), .w_preg(wkp), .err(pend_err),
      .q({l_psv, pnd_sq}),
      .r(pnd_q),
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
   // workloads/aesbench at SMOLRV64_HW=4 gives cycles/byte 4/4 123.20, 6/6 122.68, 8/8 122.18,
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
   // through smolrv64_predictor's `hit`/`p_ret` (see that module's `predict`, and rule I6). NF is back
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
   localparam integer RS_IDXB = 4;         // widest per-class entry index (IBI)
   localparam integer NWB_C   = NL + 2;    // writeback broadcasts: the lanes' ports and the two streams'
   localparam integer SQ_N = 8, SQ_IB = 3;      // store buffer: entries, index width
   localparam integer SQ_TB = SQ_IB + 1;         // ...and its seqno: the index plus a wrap bit
                                                 // (smolrv64_sq's head/tail counters), so that a load
                                                 // dispatched against a FULL queue counts NENT
                                                 // older stores, not zero
   localparam integer LQ_N = 1 << LQ_IB;         // load queue entries
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
   // (a bad encoding is not FP arith, and mstatus.FS=Off makes the op illegal at decode).
   // When FS is off, FP arith routes to the SYSQ as a trap at dispatch, the reason the F stage
   // needs no trap path. The class is smolrv64_gclass's, carried in the instruction's record.

   wire d_cls_f = d_gc[GC_F];
   // Control flow (jal/jalr/bXX) is its OWN class now: it leaves the ordered/M pipe for the
   // FP/CTF pipe so a branch co-issues with a load (CTF-on-FP). A fetch-faulted CTI or an
   // irqop pseudo-op stays ordered (d_cls_l), so M keeps the single trap site; a real CTI
   // never traps at execute (RVC targets are 2-byte aligned, jalr clears bit 0).
   wire d_cls_c = d_gc[GC_C];
   // A divide is the F port's third drain into the MD stage; its result lands by the stage's
   // tag like the FPU's.
   wire d_cls_m = d_gc[GC_M];
   // SYSTEM opcode (the irqop included), FENCE and FENCE.I (MISC-MEM, funct3 00x; CBO is 010),
   // and an instruction that traps at dispatch -- a fetch fault or an illegal instruction: every
   // one serialises and fires at the ROB head from the SYSQ's flops.
   function is_sysq(input [31:0] i, input ill, input flt);
      is_sysq = (i[6:2] == 5'b11100) | ((i[6:2] == 5'b00011) & (i[14:13] == 2'b00)) | ill | flt;
   endfunction
   wire d_cls_s = d_gc[GC_S];
   wire d_cls_l = d_gc[GC_L];
   wire d_cls_i = d_gc[GC_I];
   wire d_cls_fc = d_gc[GC_FC];

   // A plain store's rs2 is not an operand of the INSTRUCTION any more -- it is an operand
   // of its store-buffer entry, which watches for it independently (smolrv64_sq's snoop). So the
   // scheduler must not wait on it, and this is the whole of that change: one term, no
   // per-entry state, nothing added to select. The store issues on its address alone.
   wire       d_st_nb = d_gc[GC_ST];   // "buffered store" -- rule C1

   // Destination shard = the UNIT that writes it (declared with its rationale above).
   // From the opcode alone, never from the trap decode: a trapping op writes nothing, so any
   // shard serves it, and the trap decode carries mstatus.FS (an FP op with FS off is illegal)
   // into the new register's number and every pending-table set behind it.
   function [2:0] shard_of(input rd_fp, input [31:0] i, input mem, input amo, input fp,
                           input mul, input [2:0] f_slice, input [2:0] alu);
      shard_of = rd_fp                                ? f_slice   // an f-register
               : (mem | amo)                          ? alu       // the lane's landing slot
               :                                        alu;      // the lane: an ALU op, a link, a load, an AMO, a CSR read
   endfunction

   // ======================================================== THE SLOTS
   // Slot k of the dispatch group (the frontend forms it whole: every member plain, at most one
   // load, one store and one FP/control-flow op, a head op alone): its record, its classes, its
   // destination's shard, its renamed sources and their readiness, its payload and its ROB slot.
   // Slot k's ALU-class op goes to lane k.
   wire [IW-1:0]          s_v, s_cls_i, s_cls_l, s_cls_fc, s_ld_nb, s_st_nb, s_dcr, s_r2rdy;
   wire [2:0]             s_srdy_hit [0:IW-1];
   wire [RN_PBITS-1:0]    s_prd_g [0:IW-1], s_prs2 [0:IW-1];
   wire [3*RN_PBITS-1:0]  s_ps_in [0:IW-1];
   wire [PLW-1:0]         s_pl_in [0:IW-1];
   wire [ROB_IDXB-1:0]    s_rob [0:IW-1];
   wire [PCW-1:0]         s_pc [0:IW-1], s_tgt [0:IW-1], s_fault_tval [0:IW-1];
   wire [SEQW-1:0]        s_seq [0:IW-1];
   wire [31:0]            s_insn [0:IW-1];
   wire [PDW-1:0]         s_pdet [0:IW-1];
   wire [5:0]             s_rd [0:IW-1];
   wire [3:0]             s_fault_cause [0:IW-1];
   wire [IW-1:0]          s_fault, s_illegal, s_is_branch;
   wire [1:0]             s_ccls [0:IW-1];          // a call or a return (the retired RAS pointer)
   wire [IW*RN_PBITS-1:0] s_prd_gv;
   genvar gs;
   generate for (gs = 0; gs < IW; gs = gs + 1) begin: sl
      `IR_DECL(q_)
      assign `IR_REC(q_) = fe_rec[gs*IRW +: IRW];
      localparam [2:0] LANE = gs, SLICE = SH_F0 + gs;
      wire st_nb = q_gc[GC_ST], ld_nb = q_gc[GC_LD];
      wire [2:0] shard = shard_of(q_rd[5], q_insn, q_is_mem, q_is_amo, q_is_fp, q_is_mul, SLICE, LANE);
      wire [RN_PBITS-1:0] prs1 = rn_prs1v[gs*RN_PBITS +: RN_PBITS], prs2 = rn_prs2v[gs*RN_PBITS +: RN_PBITS],
                          prs3 = rn_prs3v[gs*RN_PBITS +: RN_PBITS], prd = rn_prdv[gs*RN_PBITS +: RN_PBITS];
      // readiness: both map candidates' pending bits, the late lv picking the RESULT; a source that
      // is an earlier slot's new register is pending by definition
      wire pr1 = ~rn_byp1v[gs] & (rn_lv1v[gs] ? pnd_q[6*gs]     : pnd_q[6*gs + 3]);
      wire pr2 = ~rn_byp2v[gs] & (rn_lv2v[gs] ? pnd_q[6*gs + 1] : pnd_q[6*gs + 4]);
      wire pr3 = ~rn_byp3v[gs] & (rn_lv3v[gs] ? pnd_q[6*gs + 2] : pnd_q[6*gs + 5]);
      // A plain store's rs2 is an operand of its store-buffer entry, which watches for it itself
      // (smolrv64_sq's snoop), so the scheduler does not wait on it: the store issues on its address.
      wire [2:0] srdy = {pr3 | ~q_rs3_v, pr2 | ~q_rs2_v | st_nb, pr1 | ~q_rs1_v};
      wire [RN_PBITS-1:0] prd_g = q_rd_v ? prd : {RN_PBITS{1'b0}};
      // decode-redirect (s_dcr) makes taken the prediction: taken now matches (mis_taken=0), and a
      // not-taken resolve must redirect back to fall-through (mis_nt=1)
      wire dcr = dcr_arm & q_gc[GC_DCR];
      assign s_pl_in[gs] = {q_pc, q_insn, q_rvc, q_seq, q_pdet, q_pred_npc, q_rd, q_rd_v,
                            prd_g, shard, q_rs1, q_imm,
                            q_mem_size, q_mem_signed, q_is_mem, q_is_store, q_is_amo,
                            q_amo_func, q_is_branch, q_is_jump, q_is_jalr, q_is_mul,
                            q_is_csr, q_csr_func, q_is_serialize, q_is_fp, q_is_fencei,
                            q_is_cbo, q_cbo_zero, q_cbo_keep, q_illegal, q_fault,
                            q_fault_cause, q_fault_tval,
                            q_alu_op, q_alu_w, q_alu_uw, q_op1_sel, q_op2_imm, q_res_link,
                            q_br_func, (dcr ? 1'b0 : q_mis_taken), (dcr ? 1'b1 : q_mis_nt),
                            q_rs1_v, q_rs2_v, q_rs3_v, q_gc[GC_ORD], sq_d_idx, lq_d_idx};
      assign s_v[gs] = fe_dv[gs];
      assign s_takev[gs] = fe_dv[gs] & d_take;       // a group dispatches whole
      assign s_cls_i[gs] = q_gc[GC_I];  assign s_cls_l[gs] = q_gc[GC_L];  assign s_cls_fc[gs] = q_gc[GC_FC];
      assign s_ld_nb[gs] = ld_nb;  assign s_st_nb[gs] = st_nb;  assign s_dcr[gs] = dcr;
      assign s_rs1v[gs*6 +: 6] = q_rs1;  assign s_rs2v[gs*6 +: 6] = q_rs2;  assign s_rs3v[gs*6 +: 6] = q_rs3;
      assign s_rdv[gs*6 +: 6] = q_rd;    assign s_rd_vv[gs] = q_rd_v;     assign s_shardv[gs*3 +: 3] = shard;
      assign pnd_sq[(6*gs)*RN_PBITS +: 6*RN_PBITS] =
             {rn_mprs3v[gs*RN_PBITS +: RN_PBITS], rn_mprs2v[gs*RN_PBITS +: RN_PBITS], rn_mprs1v[gs*RN_PBITS +: RN_PBITS],
              rn_sprs3v[gs*RN_PBITS +: RN_PBITS], rn_sprs2v[gs*RN_PBITS +: RN_PBITS], rn_sprs1v[gs*RN_PBITS +: RN_PBITS]};
      // EVERY CANDIDATE'S PENDING BIT IS SET, TAKEN OR NOT (see u_pend)
      assign pnd_av[gs] = fe_dv[gs] & q_rd_v & ~rn_stall;
      assign s_srdy_hit[gs] = srdy | {wk(prs3), wk(prs2), wk(prs1)};
      assign s_prd_g[gs] = prd_g;  assign s_prd_gv[gs*RN_PBITS +: RN_PBITS] = prd_g;
      assign s_prs2[gs] = prs2;    assign s_r2rdy[gs] = pr2 | ~q_rs2_v;
      assign s_ps_in[gs] = {prs3, prs2, prs1};
      assign s_rob[gs] = rob_d_idxv[gs*ROB_IDXB +: ROB_IDXB];
      assign s_pc[gs] = q_pc;  assign s_tgt[gs] = q_pc + q_imm;  assign s_seq[gs] = q_seq;  assign s_insn[gs] = q_insn;
      assign s_pdet[gs] = q_pdet;  assign s_rd[gs] = q_rd;  assign s_is_branch[gs] = q_is_branch;
      assign s_fault[gs] = q_fault;  assign s_illegal[gs] = q_illegal;
      assign s_fault_cause[gs] = q_fault_cause;  assign s_fault_tval[gs] = q_fault_tval;
      assign s_ccls[gs] = cti_cls(q_is_jump, q_is_jalr, q_rd_v, q_rd, q_rs1);
      // what the slot hands its lane: an ALU-class op, its sources (a store's rs2 zeroed: the store
      // queue's), their readiness with this cycle's wakes folded in, its destination and payload
      assign sl_li[gs]  = s_takev[gs] & q_gc[GC_I];
      assign sl_rob[gs] = s_rob[gs];
      assign sl_ps[gs]  = st_nb ? {prs3, {RN_PBITS{1'b0}}, prs1} : {prs3, prs2, prs1};
      assign sl_rdy[gs] = s_srdy_hit[gs][1:0];
      assign sl_prd[gs] = prd_g;
      assign sl_pl[gs]  = s_pl_in[gs];
      always @(posedge clk) if (!reset)
         if (sl_li[gs] & q_rd_v & ~q_rd[5] & (shard != LANE))
            $fatal(1, "smolrv64_core: slot %0d's ALU op is in shard %0d, not its lane's", gs, shard);
   end endgenerate

   wire [NL-1:0]             wkv_l;          // the lanes' ports: their write registers' issue-timed writes
   wire [NL*RN_PBITS-1:0]    wkp_l;
   wire [NWB_C-1:0]          wkv  = {wk_fe, wk_ld, wkv_l};
   wire [NWB_C*RN_PBITS-1:0] wkp  = {wa_fe, wa_ld, wkp_l};

   wire rf_ready, rf_iss_v, rf_blk_v;  wire [IBF-1:0] rf_d_ent, rf_iss_ent;
   wire [ROB_IDXB-1:0] rf_iss_rob;
   wire [RN_PBITS-1:0] rf_blk_pr;      wire [IBF:0] rf_occ;
   wire rf_take;
   wire s_win;                         // M takes a lane's load or store this cycle
   wire f_advance;


   // ---- DISPATCH STAGE (cycle boundary) -------------------------------------------------
   // The swizzle crossbar (slot->pipe mux, selects gated by the deep group-take chain)
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
   // caught by the scheduler's own fill hit(d_ps). See docs/SmolRV64-Spec.md 15.
   reg              stg_v_f;
   reg [ROB_IDXB-1:0] stg_rob_f;
   reg [3*RN_PBITS-1:0] stg_ps_f;
   reg [2:0]        stg_r_f;
   reg [RN_PBITS-1:0] stg_prd_f;
   reg [PLW-1:0]    stg_pl_f;
   initial stg_v_f = 1'b0;
   // The scheduler accepts the stage op when it has room; back-pressure to the frontend below.
   wire mv_f  = stg_v_f  & rf_ready;

   function automatic is_mulop(input [31:0] i);     // MUL, MULH*, MULW: OP or OP-32, M, funct3 < 4
      is_mulop = ((i[6:2] == 5'b01100) | (i[6:2] == 5'b01110)) & (i[31:25] == 7'b0000001) & ~i[14];
   endfunction
   function automatic is_lmem(input [31:0] i);      // a plain load or store: a lane generates its address
      is_lmem = (i[6:3] == 4'b0000) | (i[6:3] == 4'b0100);   // LOAD/LOAD-FP, STORE/STORE-FP
   endfunction
   function automatic is_hop(input [31:0] i);       // an AMO, LR/SC or CBO: a head op
      is_hop = (i[6:2] == 5'b01011) | ((i[6:2] == 5'b00011) & (i[14:12] == 3'b010));
   endfunction
   // a lane op that does not complete in its execute cycle: no write, wake or ROB completion there
   function automatic lane_late(input [31:0] i);
      lane_late = is_mulop(i) | is_lmem(i) | is_hop(i);
   endfunction
   localparam integer PL_INSN = PLW - PCW - 32;   // the instruction's place in the payload (pl_in)


   // INORDER(0): FP arith may reorder freely. It has no memory ordering to respect and
   // cannot trap, and reordering is the entire point -- the Gaussian Blur loop
   // (workloads/blurbench) is a 4-deep serial fadds chain whose taps are independent, and
   // it ran at exactly its critical path (39.50 cyc/px against 5 levels x 8 cycles) because
   // in-order issue would not let the NEXT iteration's multiplies start early.
   smolrv64_iq #(.NENT(NF),.IDXB(IBF),.NSRC(3),.ROBB(ROB_IDXB),.PBITS(RN_PBITS),.NWB(NWB_C),
             .FIXEDL(0),.INORDER(0)) u_iq_f
     (.clk(clk),.reset(reset),
      // FP-arith AND control flow (jal/jalr/bXX) share this one queue and issue slot.
      .d_valid(mv_f),.d_ready(rf_ready),
      .d_rob(stg_rob_f),
      .d_ps(stg_ps_f),.d_r(stg_r_f),
      .d_prd(stg_prd_f),.d_long(1'b0),.d_ent(rf_d_ent),
      .wb_v(wkv),.wb_preg(wkp),
      .unit_busy(j_v & ~j_adv),           // j_* can't take: occupied and its op not draining
      .iss_v(rf_iss_v),.iss_ent(rf_iss_ent),.iss_rob(rf_iss_rob),
     .iss_take(rf_take),
      .hold_v(j_v),.hold_ent(j_ent),
      .blk_v(rf_blk_v),.blk_pr(rf_blk_pr),.flush(redirect),.occupancy(rf_occ),.free_n(rf_free));

   // Dispatch back-pressure comes from whichever scheduler this instruction is routed to.
   // Stage-aware: a slot may dispatch iff its target pipe's dispatch stage is empty OR drains
   // into the scheduler this cycle (~stg_v | sched_room). A stuck stage (scheduler full) thus
   // holds dispatch at the frontend rather than losing the op behind the stage register.
   wire f_room    = ~stg_v_f  | rf_ready;
   // the group's FP/control-flow op (at most one) and what its slot hands the F/CTF stage
   wire [IW-1:0]         s_tf = s_takev & s_cls_fc;
   reg  [ROB_IDXB-1:0]   f_rob_in;
   reg  [3*RN_PBITS-1:0] f_ps_in;
   reg  [2:0]            f_r_in;
   reg  [RN_PBITS-1:0]   f_prd_in;
   reg  [PLW-1:0]        f_pl_in;
   integer               fsk;
   always @* begin
      f_rob_in = s_rob[0];  f_ps_in = s_ps_in[0];  f_r_in = s_srdy_hit[0];  f_prd_in = s_prd_g[0];  f_pl_in = s_pl_in[0];
      for (fsk = IW - 1; fsk >= 0; fsk = fsk - 1)
         if (s_tf[fsk]) begin
            f_rob_in = s_rob[fsk];  f_ps_in = s_ps_in[fsk];  f_r_in = s_srdy_hit[fsk];
            f_prd_in = s_prd_g[fsk];  f_pl_in = s_pl_in[fsk];
         end
   end

   // ISSUE ARBITRATION, one per cycle into the single issue register. Long-latency classes
   // win: they are gated on M being free anyway, so they only bid when they can make
   // progress, while an ALU op can always go next cycle instead.
   // The F scheduler no longer shares this port: it issues into its own select register
   // j_* (CTF-on-FP). i_* is M-only now, so no arbitration and no class mux here.
   // THE SCHEDULER'S JOB IS TO PRODUCE AN INDEX; everything else about the uop is looked up
   // with it. The source tags used to come OUT of each queue as `iss_ps` -- an async read of
   // the entry array (e_ps[sel], a 16:1 mux over FLOPS) then a 3-way class mux, two levels
   // landing on the i_ps* capture flops, and the tail of the post-route critical path
   // (m_addr_reg[12]_replica -> i_ps1_reg[3]/D, WNS -0.041 at DIV8=48).
   //
   // e_ps CANNOT be a LUTRAM: smolrv64_iq.v:130 broadcasts every entry's tags to the wakeup
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
   reg  [3*RN_PBITS-1:0] psmem_f [0:NF-1];
   wire [3*RN_PBITS-1:0] ps_out_f = psmem_f[rf_iss_ent];   // FP/CTF source tags into j_*
   always @(posedge clk) begin
      // Written when the stage op moves into its scheduler (T+1), at the entry that scheduler
      // allocates (*_d_ent), from the REGISTERED stage payload -- co-timed with the e_ps write.
      if (mv_f)  psmem_f[rf_d_ent]  <= stg_ps_f;
   end
   // The shared FP/CTF queue feeds j_* directly (single source). j_isctf, from the picked op's
   // payload, routes the drain: control flow to cf_*, FP to f_valid. (CTF-over-FP priority is a
   // later addition; for now the queue picks by its own policy.)
   wire j_istrap  = qf_illegal | qf_fault;               // a trap: the SYSQ, whatever its decode says
   wire j_ismd    = qf_is_mul & ~j_istrap;              // ...or a mul/div (C1): the MD stage's drain
   wire j_issys   = is_sysq(qf_insn, qf_illegal, qf_fault);  // ...or a system op, a fence or a trap: the SYSQ's drain
   wire md_advance;                                      // the MD stage can take one (defined with it)
   wire sy_advance;                                      // the SYSQ can take one (defined with it)
   wire j_needs_m = j_v & j_ismd;                        // j_* holds a mul/div...
   wire j_needs_s = j_v & j_issys;                       // ...or a system op...
   wire j_needs_f = j_v & ~j_ismd & ~j_issys;           // ...or an FP op
   wire j_adv     = j_needs_m ? md_advance : j_needs_s ? sy_advance
                  : (j_needs_f ? f_advance : 1'b1);
   wire j_ready   = ~j_v | j_adv;
   assign rf_take = rf_iss_v;   // rf_iss_v is already gated by unit_busy = j_v & ~j_adv
   wire iq_blk_v = rf_blk_v | l_blk_v[0];

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
   // smolrv64_iq wakes its consumer AT SELECT, so the consumer is selected at N+1 and executes
   // at N+2 -- consecutive execute cycles, no bubble. The same timing is why no operand
   // forwarding exists anywhere here: every producer's write has landed before its consumer
   // reads.

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
   wire   iss_f       = j_needs_f & f_advance & ~redirect;    // j_* drains an FP op into f_valid
   wire   iss_md      = j_needs_m & md_advance & ~redirect;   // ...or a mul/div into the MD stage (C1)
   wire   iss_sys     = j_needs_s & sy_advance & ~redirect;   // ...or a system op into the SYSQ (C3 step 3)


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
   // ---- dispatch-stage load + wakeup snoop ----------------------------------------------
   // The dispatch-cycle (T) writeback fold, computed PER SLOT from the early slot pregs so the
   // wkv compare parallels the accept chain and stays off the crossbar-mux->stg_r path. d_srdy
   // reflects writebacks up to T-1 (the pending read is pure); wk(p) contributes T's.
   // a broadcast names this register this cycle
   function automatic wk;
      input [RN_PBITS-1:0] p;
      integer i;
      begin
         wk = 1'b0;
         for (i = 0; i < NWB_C; i = i + 1) wk = wk | (wkv[i] & (wkp[i*RN_PBITS +: RN_PBITS] == p));
      end
   endfunction
   // Stuck-cycle snoop: fresh wkv match of the stage's OWN pregs, ORed into its ready bits.
   wire [2:0] snp_f  = {wk(stg_ps_f[2*RN_PBITS +: RN_PBITS]), wk(stg_ps_f[1*RN_PBITS +: RN_PBITS]), wk(stg_ps_f[0*RN_PBITS +: RN_PBITS])};
   always @(posedge clk) begin
      if (reset) stg_v_f <= 1'b0;
      else begin
         // F (FP/CTF): whichever slot is the (single) FC.
         if (|s_tf) begin
            stg_v_f   <= 1'b1;
            stg_rob_f <= f_rob_in;
            stg_ps_f  <= f_ps_in;
            stg_r_f   <= f_r_in;
            stg_prd_f <= f_prd_in;
            stg_pl_f  <= f_pl_in;
         end else if (mv_f) stg_v_f <= 1'b0;
         else if (stg_v_f) stg_r_f <= stg_r_f | snp_f;
         // Flush LAST (rule I11: redirect never gates an enable; flush arm wins on order).
         if (redirect) stg_v_f <= 1'b0;
      end
   end

   // =========================================================== THE LANES
   // Lane k takes slot k's ALU-class ops (slot = lane): its own dispatch stage, scheduler,
   // payload and tag arrays, select register, two PRF reads, exec unit, multiplier and write
   // register, and the landing slot its landing buffer drains into (see the landing buffers).
   // what each slot hands its lane at dispatch: a store's rs2 is zeroed (the store queue's), and
   // the dispatch cycle's lane wakes are folded into the ready bits
   wire                  sl_li  [0:NL-1];
   wire [ROB_IDXB-1:0]   sl_rob [0:NL-1];
   wire [3*RN_PBITS-1:0] sl_ps  [0:NL-1];
   wire [1:0]            sl_rdy [0:NL-1];
   wire [RN_PBITS-1:0]   sl_prd [0:NL-1];
   wire [PLW-1:0]        sl_pl  [0:NL-1];
   // each lane's PRF reads (smolrv64_prf's ports ra4..ra9)
   wire [63:0]           l_rd1  [0:NL-1], l_rd2 [0:NL-1];
   // what each lane exports
   wire                  l_v [0:NL-1], l_iss [0:NL-1], l_late [0:NL-1], l_m1 [0:NL-1], l_m2 [0:NL-1];
   wire [ROB_IDXB-1:0]   l_rob [0:NL-1], l_mrob2 [0:NL-1], l_wix [0:NL-1];
   wire [RN_PBITS-1:0]   l_ps1 [0:NL-1], l_ps2 [0:NL-1], l_wa [0:NL-1], l_qprd [0:NL-1], l_blk_pr [0:NL-1];
   wire [63:0]           l_wb [0:NL-1], l_qval [0:NL-1], l_res [0:NL-1], l_mres [0:NL-1], l_rs2 [0:NL-1];
   wire                  l_we [0:NL-1], l_qv [0:NL-1], l_wv [0:NL-1], l_cti [0:NL-1], l_mis [0:NL-1];
   wire                  l_ready [0:NL-1], l_stg_v [0:NL-1], l_blk_v [0:NL-1], l_pick [0:NL-1];
   wire [IBI:0]          l_free [0:NL-1];
   wire [PLW-1:0]        l_pl [0:NL-1];
   wire [63:0]           l_addr [0:NL-1], l_tgt [0:NL-1], l_ttgt [0:NL-1];
   wire                  l_red [0:NL-1], l_taken [0:NL-1];
   wire                  l_lm [0:NL-1], l_st [0:NL-1];      // a load or store generating its address
   wire                  l_br [0:NL-1], l_jmp [0:NL-1], l_jalr [0:NL-1], l_rvc [0:NL-1], l_rdv [0:NL-1];
   wire [SEQW-1:0]       l_seq [0:NL-1];
   wire [PCW-1:0]        l_pc [0:NL-1];
   wire [PDW-1:0]        l_pdet [0:NL-1];
   wire [5:0]            l_rd [0:NL-1], l_rs1 [0:NL-1];
   wire [LQ_IB-1:0]      l_lqi [0:NL-1];
   wire [SQ_IB-1:0]      l_sqt [0:NL-1];
   wire [NL*64-1:0]      l_wbv;                               // the lanes' broadcast values, packed
   genvar gl;
   generate for (gl = 0; gl < NL; gl = gl + 1) begin: ln
      // ---- the dispatch stage: the slot's op, held until the scheduler takes it ----
      reg                  stg_v;
      reg [ROB_IDXB-1:0]   stg_rob;
      reg [3*RN_PBITS-1:0] stg_ps;
      reg [1:0]            stg_r;
      reg [RN_PBITS-1:0]   stg_prd;
      reg [PLW-1:0]        stg_pl;
      initial stg_v = 1'b0;
      wire                 ready, iss_v, blk_v;
      wire [IBI-1:0]       d_ent, iss_ent;
      wire [ROB_IDXB-1:0]  iss_rob;
      wire [RN_PBITS-1:0]  blk_pr;
      wire [IBI:0]         occ, free;
      wire                 mv = stg_v & ready;
      wire [1:0]           snp = {wk(stg_ps[RN_PBITS +: RN_PBITS]), wk(stg_ps[0 +: RN_PBITS])};
      always @(posedge clk) begin
         if (reset) stg_v <= 1'b0;
         else begin
            if (sl_li[gl]) begin
               stg_v <= 1'b1;  stg_rob <= sl_rob[gl];  stg_ps <= sl_ps[gl];  stg_r <= sl_rdy[gl];
               stg_prd <= sl_prd[gl];  stg_pl <= sl_pl[gl];
            end else if (mv) stg_v <= 1'b0;
            else if (stg_v) stg_r <= stg_r | snp;
            if (redirect) stg_v <= 1'b0;   // flush last (rule I11)
         end
      end
      // ---- the scheduler: an index into the lane's payload and tag arrays ----
      reg                  m1;
      reg                  v;
      reg [IBI-1:0]        ent;
      smolrv64_iq #(.NENT(NI),.IDXB(IBI),.NSRC(2),.ROBB(ROB_IDXB),.PBITS(RN_PBITS),.NWB(NWB_C),
                .FIXEDL(1),.INORDER(0)) u_iq
        (.clk(clk),.reset(reset),
         .d_valid(mv),.d_ready(ready),.d_rob(stg_rob),
         .d_ps(stg_ps[2*RN_PBITS-1:0]),.d_r(stg_r),.d_prd(stg_prd),.d_long(lane_late(stg_pl[PL_INSN +: 32])),.d_ent(d_ent),
         .wb_v(wkv),.wb_preg(wkp),
         // busy: the multiply's reserved slot, a waiting landing, and in lane 0 a system op at the ROB head
         .unit_busy(m1 | lb_any[gl] | ((gl == 0) & sy_at_head)),.iss_v(iss_v),.iss_ent(iss_ent),.iss_rob(iss_rob),
         .iss_take(iss_v),
         .hold_v(v),.hold_ent(ent),
         .blk_v(blk_v),.blk_pr(blk_pr),.flush(redirect),.occupancy(occ),.free_n(free));
      // the tags, and whether the op completes later than its issue (lane_late), read at the pick
      reg  [3*RN_PBITS:0]   psmem [0:NI-1];
      reg  [PLW-1:0]        plmem [0:NI-1];
      // written as the stage's op moves into the scheduler (T+1), at the entry it allocates
      always @(posedge clk) if (mv) psmem[d_ent] <= {lane_late(stg_pl[PL_INSN +: 32]), stg_ps};
      always @(posedge clk) if (mv) plmem[d_ent] <= stg_pl;
      wire [3*RN_PBITS:0]   ps_out = psmem[iss_ent];
      // ---- the select register: the picked op, executing the next cycle ----
      reg [ROB_IDXB-1:0]   rob;
      reg [RN_PBITS-1:0]   ps1, ps2;
      reg                  late;            // the op completes later than its issue (lane_late)
      reg [PLW-1:0]        pl;
      initial begin v = 1'b0; ent = {IBI{1'b0}}; rob = {ROB_IDXB{1'b0}}; ps1 = {RN_PBITS{1'b0}}; ps2 = {RN_PBITS{1'b0}}; late = 1'b0; end
      always @(posedge clk) begin
         if (reset | redirect) v <= 1'b0;
         else begin
            v <= iss_v;
            if (iss_v) begin
               ent <= iss_ent;  rob <= iss_rob;
               ps1 <= ps_out[0 +: RN_PBITS];  ps2 <= ps_out[RN_PBITS +: RN_PBITS];  late <= ps_out[3*RN_PBITS];
            end
         end
      end
      always @(posedge clk) if (iss_v) pl <= plmem[iss_ent];   // the payload, read at the pick
      wire iss = v & ~redirect;                                  // completes here, always
      `PL_DECL(q_)
      assign `PL_REC(q_) = pl;
      // ---- operands: the PRF, or a lane's write register landing this cycle ----
      reg [63:0] rs1, rs2;
      integer f;
      always @* begin
         rs1 = l_rd1[gl];  rs2 = l_rd2[gl];
         for (f = NL - 1; f >= 0; f = f - 1) begin
            if (l_qv[f] & (ps1 == l_qprd[f])) rs1 = l_qval[f];
            if (l_qv[f] & (ps2 == l_qprd[f])) rs2 = l_qval[f];
         end
      end
      wire [63:0] result, addr, target, taken_tgt;
      wire        red, taken;
      smolrv64_exec u_x
        (.alu_op(q_alu_op), .alu_w(q_alu_w), .alu_uw(q_alu_uw), .op1_sel(q_op1_sel),
         .op2_imm(q_op2_imm), .res_link(q_res_link), .is_rvc(q_rvc),
         .is_branch(q_is_branch), .is_jump(q_is_jump), .is_jalr(q_is_jalr), .br_func(q_br_func),
         .rs1_val(rs1), .rs2_val(rs2), .imm(q_imm), .pc(q_pc),
         .pred_npc(q_pred_npc), .mis_taken(q_mis_taken), .mis_nt(q_mis_nt),
         .result(result), .addr(addr), .redirect(red), .target(target),
         .taken(taken), .taken_tgt(taken_tgt));
      // THE ISSUE CHECK: nothing executes with a source still pending (the scheduler enforces it
      // structurally -- an entry is not selectable until every source is ready); a store's rs2
      // is the store queue's
      always @(posedge clk) if (!reset & iss) begin
         if (q_rs1_v & ~pnd_q[6*IW + 2*gl])
            $fatal(1, "core: lane %0d executed with rs1 p%0d still pending (rob=%0d pc=%h)", gl, ps1, rob, q_pc);
         if (q_rs2_v & ~pnd_q[6*IW + 2*gl + 1] & ~(q_is_store & is_lmem(q_insn)))
            $fatal(1, "core: lane %0d executed with rs2 p%0d still pending (rob=%0d)", gl, ps2, rob);
      end
      wire cti  = iss & (q_is_branch | q_is_jump | q_is_jalr);
      wire mis  = cti & red;
      // ---- the multiplier (lanes step 5.2c) ----
      // mul3 starts from the lane's forwarded operands in the execute cycle T, and the lane
      // reserves its slot at T+2 for it: the scheduler selects nothing at T+1 (m1 is its
      // unit_busy), so nothing else executes at T+2. That slot carries the multiply's wake (its
      // consumers execute at T+3 off the write register), its ROB completion, and its result
      // into the write register at the edge, which the PRF writes at T+3.
      wire                 m_go = iss & is_mulop(q_insn);
      reg                  m2, mrdv1, mrdv2;
      reg [ROB_IDXB-1:0]   mrob1, mrob2;
      reg [RN_PBITS-1:0]   mprd1, mprd2;
      wire                 mpre;
      wire [63:0]          mres;
      initial begin m1 = 1'b0; m2 = 1'b0; end
      mul3 u_mul
        (.clk(clk), .reset(reset), .start(m_go), .abort(redirect),
         .rs1(rs1), .rs2(rs2), .f3(q_insn[14:12]), .is_w(q_insn[6:2] == 5'b01110),
         .busy(), .done(), .result(), .pre_done(mpre), .pre_result(mres));
      always @(posedge clk) begin
         m1 <= ~reset & ~redirect & m_go;
         m2 <= ~reset & ~redirect & m1;
         mrob1 <= rob;    mprd1 <= q_prd;  mrdv1 <= q_rd_v;
         mrob2 <= mrob1;  mprd2 <= mprd1;  mrdv2 <= mrdv1;
      end
      wire m_wr = m2 & mrdv2;
      always @(posedge clk) if (!reset) begin
         if (m2 & v)    $fatal(1, "smolrv64_core: lane %0d executes an op in its multiply's reserved slot", gl);
         if (m2 & ~mpre) $fatal(1, "smolrv64_core: lane %0d's multiply is not at mul3's stage B in its slot", gl);
      end
      // ---- the landing slot: the entry the lane's landing buffer drains this cycle ----
      wire                 dwr, dcm;
      wire [RN_PBITS-1:0]  dprd;
      wire [63:0]          ddat;
      wire [ROB_IDXB-1:0]  drob;
      assign {dwr, dcm, dprd, ddat, drob} = lp[gl];
      wire dp = lb_free[gl] & (lb_any[gl] | p1[gl]);   // the port takes the buffer's head or the SYSQ
      wire d  = dp & dwr;              // ...writing a register
      wire dc = dp & dcm;              // ...completing its op
      wire bs = by0[gl] | by2[gl];     // a stream's landing straight through (its own port's wake and completion)
      // ---- the write register: one write slot a cycle, taken by the landing slot, the ALU op at
      // issue or the multiply in its reserved slot. The wake, the pending clear and the store
      // queue's snoop are issue-timed (we/wa/wb); the PRF write lands a cycle later from the
      // register, and the forward above covers that cycle. The enables do not read the live
      // redirect (rule I11): a redirect fires at the ROB head, so the op executing then is younger
      // and its write lands in a register the rollback frees before any producer of it dispatches.
      wire                 we = (v & q_rd_v & ~late) | m_wr | d;
      wire [RN_PBITS-1:0]  wa = d ? dprd : m2 ? mprd2 : q_prd;
      wire [63:0]          wb = d ? ddat : m2 ? mres : result;
      reg                  qv;
      reg  [RN_PBITS-1:0]  qprd;
      reg  [63:0]          qval;
      initial begin qv = 1'b0; qprd = {RN_PBITS{1'b0}}; qval = 64'd0; end
      always @(posedge clk) begin
         qv <= ~reset & (we | bs);
         if (we | bs) begin
            qprd <= bs ? (by0[gl] ? wa_ld : wa_fe) : wa;
            qval <= bs ? (by0[gl] ? wb_ld : wb_fe) : wb;
         end
      end
      // the lane's ROB completion port: its op at issue (a load or store never; a mispredicting
      // CTI at its squash), its multiply in the reserved slot, its landing at the drain
      assign l_wv[gl]  = (iss & ~mis & ~late) | m2 | dc;
      assign l_wix[gl] = dp ? drob : m2 ? mrob2 : rob;   // dp: registers and the SYSQ's fire
      if (gl > 0) begin: rw
         assign rob_wv[3 + gl] = l_wv[gl];  assign rob_wix[(3 + gl)*ROB_IDXB +: ROB_IDXB] = l_wix[gl];
      end
      assign l_v[gl] = v;          assign l_iss[gl] = iss;      assign l_late[gl] = late;
      assign l_m1[gl] = m1;        assign l_m2[gl] = m2;        assign l_mrob2[gl] = mrob2;
      assign l_mres[gl] = mres;    assign l_rob[gl] = rob;      assign l_ps1[gl] = ps1;
      assign l_ps2[gl] = ps2;      assign l_we[gl] = we;        assign l_wa[gl] = wa;
      assign l_wb[gl] = wb;        assign l_qv[gl] = qv;        assign l_qprd[gl] = qprd;
      assign l_qval[gl] = qval;    assign l_res[gl] = result;   assign l_rs2[gl] = rs2;
      assign l_cti[gl] = cti;      assign l_mis[gl] = mis;      assign l_pl[gl] = pl;
      assign l_ready[gl] = ready;  assign l_stg_v[gl] = stg_v;  assign l_blk_v[gl] = blk_v;
      assign l_blk_pr[gl] = blk_pr;  assign l_free[gl] = free;  assign l_pick[gl] = iss_v;
      assign l_addr[gl] = addr;    assign l_tgt[gl] = target;   assign l_ttgt[gl] = taken_tgt;
      assign l_red[gl] = red;      assign l_taken[gl] = taken;
      assign l_qvv[gl] = qv;  assign l_qprdv[gl*RN_PBITS +: RN_PBITS] = qprd;  assign l_qvalv[gl*64 +: 64] = qval;
      assign l_psv[(2*gl)*RN_PBITS +: RN_PBITS] = ps1;  assign l_psv[(2*gl+1)*RN_PBITS +: RN_PBITS] = ps2;
      assign l_rd1[gl] = l_opv[(2*gl)*64 +: 64];  assign l_rd2[gl] = l_opv[(2*gl+1)*64 +: 64];
      assign l_lm[gl] = v & is_lmem(q_insn);  assign l_st[gl] = q_is_store;
      assign l_br[gl] = q_is_branch;  assign l_jmp[gl] = q_is_jump;  assign l_jalr[gl] = q_is_jalr;
      assign l_rvc[gl] = q_rvc;  assign l_rdv[gl] = q_rd_v;  assign l_seq[gl] = q_seq;  assign l_pc[gl] = q_pc;
      assign l_pdet[gl] = q_pdet;  assign l_rd[gl] = q_rd;  assign l_rs1[gl] = q_rs1;
      assign l_lqi[gl] = q_lq_idx;  assign l_sqt[gl] = q_sq_tag;
      assign wkv_l[gl] = we;  assign wkp_l[gl*RN_PBITS +: RN_PBITS] = wa;  assign l_wbv[gl*64 +: 64] = wb;
   end endgenerate

   // ONE payload array across all three schedulers, indexed by a flat slot number with a
   // per-class offset -- each scheduler has its own entry-number space, and the offsets are
   // what stop them aliasing.
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
   reg  [PLW-1:0] plf_q;
   always @(posedge clk) if (rf_take) plf_q <= plmem_f[rf_iss_ent];   // the FP/CTF port's payload
   wire [PLW-1:0] plf_out  = plf_q;
   // Co-timed with the scheduler fill (T+1), from the REGISTERED stage payload.
   always @(posedge clk) if (mv_f) plmem_f[rf_d_ent] <= stg_pl_f;

   // the F/CTF port's payload, unpacked (unused fields fall away)
   `PL_DECL(qf_)
   assign `PL_REC(qf_) = plf_out;

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
   wire                sq_av_any, sq_uf_any;
   wire                sq_r_v;                 // the store queue reads a store's data...
   wire [RN_PBITS-1:0] sq_r_preg;              // ...from this register, on M's port
   wire [SQ_IB-1:0]    sq_uf_idx;  wire [SEQW-1:0] sq_uf_seq;
   wire [SQ_IB-1:0]    sq_d_idx;
   wire [SQ_TB-1:0]    sq_d_tag;
   wire [ROB_IDXB-1:0] sq_c_rob;
   wire [55:0]         sq_c_addr;
   wire [63:0]         sq_c_data;
   wire [1:0]          sq_c_size;
   wire                lsu_pt_done, lsu_pt_ack, lsu_pt_is_store, lsu_pt_ld_done, lsu_pt_ld_kill, lsu_xo_v, lsu_xo_unc, lsu_xo_mem;
   wire                lsu_pt_fast_done;  wire [LQ_IB-1:0] lsu_pt_rtag;   // the fast path's landing, by tag
   wire [55:0]         lsu_xo_pa;
   wire [IW-1:0] g_st = s_takev & s_st_nb, g_ld = s_takev & s_ld_nb;   // the group's store and load
   wire d_st_alloc = |g_st;                           // a group holds at most one store...
   wire d_ld_alloc = |g_ld;                           // ...and one load
   // each one's slot: its ROB slot, payload, data register and readiness, destination, pc and seq
   reg  [ROB_IDXB-1:0] gst_rob, gld_rob;
   reg  [PLW-1:0]      gst_pl, gld_pl;
   reg  [RN_PBITS-1:0] gst_dpreg, gld_prd;
   reg                 rdy_st, gld_rdv;
   reg  [5:0]          gld_rd;
   reg  [38:0]         gst_pc, gld_pc;
   reg  [SEQW-1:0]     gst_seq, gld_seq;
   reg                 gld_behind_st;                  // the load is younger than the group's store
   integer             mqk, mqj;
   always @* begin
      gst_rob = s_rob[0];  gst_pl = s_pl_in[0];  gst_dpreg = s_prs2[0];  rdy_st = s_r2rdy[0];
      gst_pc = s_pc[0][38:0];  gst_seq = s_seq[0];
      gld_rob = s_rob[0];  gld_pl = s_pl_in[0];  gld_prd = s_prd_g[0];  gld_rd = s_rd[0];  gld_rdv = s_rd_vv[0];
      gld_pc = s_pc[0][38:0];  gld_seq = s_seq[0];
      gld_behind_st = 1'b0;
      for (mqk = IW - 1; mqk >= 0; mqk = mqk - 1) begin
         if (g_st[mqk]) begin
            gst_rob = s_rob[mqk];  gst_pl = s_pl_in[mqk];  gst_dpreg = s_prs2[mqk];  rdy_st = s_r2rdy[mqk];
            gst_pc = s_pc[mqk][38:0];  gst_seq = s_seq[mqk];
         end
         if (g_ld[mqk]) begin
            gld_rob = s_rob[mqk];  gld_pl = s_pl_in[mqk];  gld_prd = s_prd_g[mqk];  gld_rd = s_rd[mqk];
            gld_rdv = s_rd_vv[mqk];  gld_pc = s_pc[mqk][38:0];  gld_seq = s_seq[mqk];
         end
      end
      for (mqk = 0; mqk < IW; mqk = mqk + 1)
         for (mqj = 0; mqj < mqk; mqj = mqj + 1)
            if (g_st[mqj] & g_ld[mqk]) gld_behind_st = 1'b1;
   end
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

   // ---- THE WALKER: a queue entry M handed over untranslated ----
   // The store queue's first uncommitted entry, else the load queue's candidate, whichever has no
   // translation (stores commit and loads reach memory in queue order, so no other entry can be
   // waited on first). Once taken, an entry keeps the walker until it answers -- a store filling
   // meanwhile does not take the walk over -- or a redirect ends the walk with the entry.
   wire                sq_k_v, lq_k_v, sq_f_v, lq_f_v;
   wire [SQ_IB-1:0]    sq_k_idx;
   wire [LQ_IB-1:0]    lq_k_idx;
   wire [38:0]         sq_k_va, lq_k_va, sq_f_pc, lq_f_pc;
   wire [ROB_IDXB-1:0] sq_f_rob, lq_f_rob;
   wire [3:0]          sq_f_fc, lq_f_fc;
   wire [SEQW-1:0]     sq_f_seq, lq_f_seq;
   wire                lsu_wk_done, lsu_wk_unc, lsu_wk_mem, lsu_wk_flt, lsu_xo_tv;
   wire [55:0]         lsu_wk_pa;
   wire                lsu_xo_flt;           // the fill's address alone faults: the entry carries it
   wire [3:0]          lsu_xo_fc;
   wire [3:0]          lsu_wk_fc;
   // THE REQUEST IS A REGISTER. Choosing the entry (the queues' candidate pointers, the store-
   // or-load pick) and the walker's 2048-entry TLB read are a cycle apart, so the choice never
   // reaches the TLB's read address in the cycle it is made.
   reg                 wk_lk, wk_lk_st;
   reg [38:0]          wk_lk_va;
   initial begin wk_lk = 1'b0; wk_lk_st = 1'b0; end
   wire                wk_v  = wk_lk;
   wire                wk_st = wk_lk_st;
   wire [38:0]         wk_va = wk_lk_va;
   always @(posedge clk) begin
      // the payload follows the choice while the walker is free; only the lock bit sees the flush
      // (rule I11: the redirect gates no enable)
      if (~wk_lk) begin wk_lk_st <= sq_k_v;  wk_lk_va <= sq_k_v ? sq_k_va : lq_k_va; end
      if (~wk_lk & (sq_k_v | lq_k_v)) wk_lk <= 1'b1;
      if (lsu_wk_done)                wk_lk <= 1'b0;
      if (reset | redirect)           wk_lk <= 1'b0;
      if (!reset && !redirect && wk_lk && (wk_lk_st ? ~sq_k_v : ~lq_k_v))
         $fatal(1, "smolrv64_core: the walker's entry (%0s) is no longer the untranslated one", wk_lk_st ? "store" : "load");
   end

   smolrv64_sq #(.NENT(SQ_N), .IDXB(SQ_IB), .PAW(56), .PBITS(RN_PBITS),
             .ROBB(ROB_IDXB), .NWB(NWB_C), .LQN(LQ_N), .LQIB(LQ_IB), .SEQW(SEQW)) u_sq
     (.clk(clk), .reset(reset),
      .d_alloc(d_st_alloc), .d_rob(gst_rob), .d_dpreg(gst_dpreg), .d_rdy(rdy_st),
      .d_pc(gst_pc), .d_seq(gst_seq),
      .d_ready(sq_d_ready), .d_ready2(sq_d_ready2), .d_idx(sq_d_idx), .d_tag(sq_d_tag), .av_any(sq_av_any),
      .uf_any(sq_uf_any), .uf_idx(sq_uf_idx), .uf_seq(sq_uf_seq),
      .a_v(m_sq_fill), .a_idx(m_sq_tag), .a_addr(lsu_xo_pa), .a_va(m_addr[38:0]), .a_tv(lsu_xo_tv | lsu_xo_flt),
      .a_flt(lsu_xo_flt), .a_fc(lsu_xo_fc), .a_size(m_mem_size),
      .a_unc(lsu_xo_unc), .r_v(sq_r_v), .r_preg(sq_r_preg), .r_data(prf_sq),
      .wb_v(wkv), .wb_preg(wkp), .wb_data({wb_fe, wb_ld, l_wbv}),
      .c_v(sq_c_v), .c_rob(sq_c_rob), .c_addr(sq_c_addr), .c_data(sq_c_data),
      .c_size(sq_c_size), .c_unc(sq_c_unc), .c_take(sq_c_take),
      .kc_v(sq_kc_v), .kc_rob(sq_kc_rob), .kc_addr(sq_kc_addr), .kc_data(sq_kc_data), .kc_size(sq_kc_size), .k_take(sq_k_take),
      // THE ALIAS TEST LIVES HERE, not at issue: smolrv64_lq exports its entries, smolrv64_sq keeps
      // a conflict matrix updated wherever an address arrives, and issue reads a flop.
      .l_off(lq_e_off), .l_size(lq_e_size), .l_tag(lq_e_tag), .l_av(lq_e_av),
      .l_fill(m_lq_fill), .l_fill_ix(m_lq_idx),
      .l_fill_off(m_addr[11:0]), .l_fill_size(m_mem_size), .l_fill_flt(lsu_xo_flt),
      .l_block(sq_l_block_live), .l_block_q(lq_e_block), .l_older(sq_l_older),
      .ld_tag(lq_q_tag), .ld_older(sq_ld_older),
      .l_block_unk_q(sq_l_block_unk_q),
      .k_v(sq_k_v), .k_idx(sq_k_idx), .k_va(sq_k_va),
      .w_v(lsu_wk_done & wk_st), .w_idx(sq_k_idx), .w_pa(lsu_wk_pa), .w_unc(lsu_wk_unc),
      .w_flt(lsu_wk_flt), .w_fc(lsu_wk_fc),
      .f_v(sq_f_v), .f_rob(sq_f_rob), .f_fc(sq_f_fc), .f_pc(sq_f_pc), .f_seq(sq_f_seq),
      .occupancy(sq_occ), .flush(redirect));

   // Instrumentation for "did a load actually get reordered past a store". A load STARTS
   // its access only when ~ld_block, so a start with an older store still live is exactly
   // one reordering that the old in-order machine could not have done.
   wire sq_ld_reorder = lq_x_take & sq_ld_older;

   // ------------------------------------------------------------------- LOAD QUEUE
   wire                lq_d_ready, lq_x_v, lq_x_signed, lq_x_fp, lq_x_unc, lq_x_head, lq_l_rd_v, lq_b_ok;
   wire [LQ_IB-1:0]    lq_d_idx, lq_x_idx;
   wire [55:0]         lq_x_pa;
   wire [1:0]          lq_x_size;
   wire [LQ_N*12-1:0]  lq_e_off;
   wire [LQ_N*2-1:0]   lq_e_size;
   wire [LQ_N*SQ_TB-1:0] lq_e_tag;
   wire [LQ_N-1:0]     lq_e_av, lq_e_block;
   wire [SQ_TB-1:0]    lq_q_tag;
   wire [RN_PBITS-1:0] lq_l_prd;
   wire [5:0]          lq_l_rd;
   wire [ROB_IDXB-1:0] lq_l_rob;
   wire [55:0]         lq_l_pa;      // the landing load's own PA (cosim memory effect)
   wire [LQ_IB:0]      lq_occ;  wire lq_av_any, lq_uf_any;
   wire [LQ_N*RN_PBITS-1:0] lq_e_dprd;   // the live loads' destinations (the stall counters)
   wire [LQ_IB-1:0]    lq_uf_idx;  wire [SEQW-1:0] lq_uf_seq;
   wire                lq_x_devwait;        // counters: a device load waiting for the head
   wire d_ld_nb    = d_gc[GC_LD];  // plain load, rule C1
   // a load behind a store in the same cycle captures the tag AFTER that store's
   wire [SQ_TB-1:0] ld_sqtag = sq_d_tag + {{SQ_IB{1'b0}}, gld_behind_st};

   smolrv64_lq #(.NENT(LQ_N), .IDXB(LQ_IB), .PAW(56), .PBITS(RN_PBITS),
             .ROBB(ROB_IDXB), .SQIB(SQ_TB), .SEQW(SEQW), .LRAM_BASE(LBASE), .LRAM_LG2(LRAM_LG2)) u_lq
     (.clk(clk), .reset(reset),
      .d_alloc(d_ld_alloc), .d_rob(gld_rob), .d_prd(gld_prd),
      .d_rd(gld_rd), .d_rd_v(gld_rdv), .d_sqtag(ld_sqtag),
      .d_pc(gld_pc), .d_seq(gld_seq),
      .d_ready(lq_d_ready), .d_ready2(lq_d_ready2), .d_idx(lq_d_idx),
      .a_v(m_lq_fill), .a_sent(lsu_xo_early), .a_idx(m_lq_idx),
      .a_pa(lsu_xo_pa), .a_va(m_addr[38:0]), .a_tv(lsu_xo_tv | lsu_xo_flt), .a_flt(lsu_xo_flt), .a_fc(lsu_xo_fc),
      .a_size(m_mem_size),
      .a_signed(m_mem_signed), .a_fp(m_is_fp), .a_unc(lsu_xo_unc), .a_mem(lsu_xo_mem),
      .e_off(lq_e_off), .e_size(lq_e_size), .e_tag(lq_e_tag), .e_av(lq_e_av),
      .e_block(lq_e_block), .x_block(sq_ld_block), .q_tag(lq_q_tag),
      .b_idx(m_lq_idx), .b_ok(lq_b_ok),
      .x_v(lq_x_v), .x_idx(lq_x_idx), .x_pa(lq_x_pa), .x_size(lq_x_size),
      .x_signed(lq_x_signed), .x_fp(lq_x_fp), .x_unc(lq_x_unc), .x_head(lq_x_head), .x_take(lq_x_take),
      .l_v(ld_land), .l_idx(ld_land_idx),
      .l_prd(lq_l_prd), .l_rd(lq_l_rd), .l_rd_v(lq_l_rd_v), .l_rob(lq_l_rob), .l_pa(lq_l_pa),
      .k_v(lq_k_v), .k_idx(lq_k_idx), .k_va(lq_k_va),
      .w_v(lsu_wk_done & ~wk_st), .w_idx(lq_k_idx), .w_pa(lsu_wk_pa), .w_unc(lsu_wk_unc), .w_mem(lsu_wk_mem),
      .w_flt(lsu_wk_flt), .w_fc(lsu_wk_fc),
      .f_v(lq_f_v), .f_rob(lq_f_rob), .f_fc(lq_f_fc), .f_pc(lq_f_pc), .f_seq(lq_f_seq),
      .x_devwait(lq_x_devwait), .occupancy(lq_occ), .av_any(lq_av_any), .e_dprd(lq_e_dprd), .uf_any(lq_uf_any), .uf_idx(lq_uf_idx), .uf_seq(lq_uf_seq), .rob_head(rob_head_idx), .flush(redirect));

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
   // smolrv64_sq's ld_older is v[]/head/ld_tag alone -- no address, nothing off the translate
   // path -- and lq_b_ok is a 2-bit index compare on flops. Both settle at the top of the
   // cycle, in parallel with the dTLB lookup they qualify. ld_BLOCK, the address compare, is
   // what must never come back here, and does not. The ADDRESS still reaches mem_raddr from
   // t_paddr in this cycle, which is the path ec6ad3e closed at 166 MHz.
   //
   // ld_older answers about OUR load only while the queue's query port is asking about it:
   // q_tag is sqt[acc], so `b_idx == acc` -- inside lq_b_ok -- is what makes the read sound.
   // ...and since 2026-09-07 the answer M reads is the store queue's REGISTERED per-load
   // copy at M's own load index (a register), not the live query: see smolrv64_sq l_older. The
   // live one is the oracle, and the copy may only ever be the more conservative.
   wire lq_b_early = m_ld_nb & lq_b_ok & ~sq_l_older[m_lq_idx];
   // Checked only while M holds the load: m_lq_idx is a register that keeps the LAST load's
   // index, and a load dispatched into that entry a cycle ago is the queue's candidate
   // (lq_b_ok) before its copy has caught up -- one cycle, and M is not looking.
   always @(posedge clk)
      if (!reset && m_ld_nb && lq_b_ok && sq_ld_older && !sq_l_older[m_lq_idx])
         $fatal(1, "smolrv64_core: the registered older-store answer (0) is less conservative than the live one (1) for load %0d", m_lq_idx);

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
      if (ld_land_fast && ld_land_slow) $fatal(1, "smolrv64_core: a load landed on both the fast and the slow path");
      if (ld_land_slow && dmem_rvalid_c) $fatal(1, "smolrv64_core: a slow landing under a fast response (the index would be wrong)");
      if (ld_land_fast && (lsu_pt_rtag != dmem_rtag_resp)) $fatal(1, "smolrv64_core: the fast landing's tag is not the response's");
   end
   // The registered alias block the queue's candidate select reads may only ever be the
   // MORE conservative: a load that starts (x_v, on the copy) is never one the live block holds.
   always @(posedge clk)
      if (!reset && lq_x_v && sq_l_block_live[lq_x_idx])
         $fatal(1, "smolrv64_core: load %0d starts on the registered block copy while the live block holds it", lq_x_idx);
   // The tag of the access in flight. One at a time today, so a single register; when loads
   // are pipelined this becomes the D$'s rd_tag and the queue interface does not change.
   // Two ways an access leaves for memory now, and they name their entry differently: the
   // candidate path by lq_x_idx, the early path by the index M is holding. They are mutually
   // exclusive -- an early start needs ~av[acc], a candidate start needs av[acc] -- which
   // smolrv64_lq asserts rather than assumes.
   wire lq_b_take = lsu_xo_early;          // the LSU says whether the early start happened
   reg  [LQ_IB-1:0] ld_inflight_idx;
   initial ld_inflight_idx = {LQ_IB{1'b0}};
   always @(posedge clk) if      (lq_x_take) ld_inflight_idx <= lq_x_idx;
                         else if (lq_b_take) ld_inflight_idx <= m_lq_idx;

   // NW=4: the store's ROB slot completes when the BUFFER writes it, not when it executes.
   // Routed through the existing completion mechanism (rule C2), which is parameterised on
   // exactly this -- not a private path to the ROB.
   // The head's kill: a trap's done is latched (no live dTLB here). Behind the head, M's op
   // retires only from the head.
   assign rc_kill[0] = (m_valid & m_done_red & m_trap) | sy_trap;
   generate for (gs = 0; gs < IW; gs = gs + 1) begin: rc
      localparam [ROB_IDXB-1:0] K = gs;
      assign rc_idx[gs] = rob_head_idx + K;
      assign rc_f[gs]   = rc_rdv[gs] & rc_rd[gs*6+5];     // writes an f-register
      if (gs > 0) begin: kb
         assign rc_kill[gs] = m_valid & (m_rob_idx == rc_idx[gs]);
      end
   end endgenerate
   smolrv64_rob #(.DEPTH(ROB_DEPTH), .IDXB(ROB_IDXB), .PBITS(RN_PBITS), .IW(IW), .NW(ROB_NW), .IRR_FWD({{(ROB_NW-4){1'b0}}, 4'b1000})) u_rob   // w_v[3]: sq_k_take
     (.clk(clk), .reset(reset),
      // prd is ZERO when nothing is written: rename drives r_prd unconditionally, and
      // `d_prd != 0` is what replaces the stored rd_v bit.
      .d_valid(s_takev), .d_rd(s_rdv), .d_prd(s_prd_gv),
      .d_noret({{(IW-1){1'b0}}, d_is_irqop}),          // the interrupt pseudo-op goes alone
      .d_ready(rob_readyv), .d_idx(rob_d_idxv),
      .w_v(rob_wv), .w_ix(rob_wix),
      .h_fin(cf_red_fire | (m_red_fire & ~m_trap) | (sy_done & sy_red)),   // the head completes and flushes (a trap never commits)
      .c_kill(rc_kill), .c_valid(rc_v), .c_rd(rc_rd), .c_rd_v(rc_rdv), .c_prd(rc_prd), .c_noret(rc_noret),
      .flush(redirect), .empty(rob_empty), .occ_n(rob_occ), .head_idx(rob_head_idx),
      .irr_idx(rob_irr_idx), .irr_v(rob_irr_v));

   // The M-equivalence assertion that guarded the previous two commits is GONE, deliberately
   // and by construction: it said the ROB's commit equals what M would have done in the same
   // cycle, which was true only because M blocked. It no longer does. Its replacement is the
   // ROB's own always-on set (double completion, completion of a dead slot, committing an
   // invalid head, occupancy overflow) plus the scoreboard's, above.
   //
   // The ROB's room is real back-pressure rather than an assertion: with M releasing loads
   // early, the ROB genuinely fills behind a head that is waiting for its data.

   // M-stage registers (declared here: the bypass reads them)
   reg              m_valid, m_rvc, m_rd_v;
   reg  [PCW-1:0]   m_pc;
   reg  [31:0]      m_insn;
   reg  [SEQW-1:0]  m_seq;
   reg  [RN_PBITS-1:0] m_prd;           // rename result, carried X->M
   reg  [2:0]       m_shard;
   reg  [63:0]      m_addr, m_st_data;
   reg  [1:0]       m_mem_size;
   reg              m_mem_signed, m_is_mem, m_is_store, m_is_amo;
   reg  [4:0]       m_amo_func;
   reg              m_is_serialize, m_is_fp;
   reg              m_is_cbo, m_cbo_zero, m_cbo_keep;
   // Store-buffer slot. ONE field serves both roles, because a buffered store's own slot IS the tail it
   // captured at dispatch: for a store it names the entry to fill. (A load's store-seqno
   // goes straight into smolrv64_lq at dispatch and is one bit wider -- SQ_TB.)
   reg  [SQ_IB-1:0] m_sq_tag;
   reg  [LQ_IB-1:0] m_lq_idx;
   initial begin m_valid = 1'b0; end

   // the M-stage writeback value, and the bypass source (which is NOT the same thing --
   // see the writeback comment: a CSR result is never bypassable)
   wire [63:0] m_wb_val, m_byp_val;
   wire [63:0] wb_ld, wb_fe;          // the LD and FE streams' write data

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

   // rn_stall IS in the stall path now (see d_hold). It used to be a $fatal, on the grounds
   // that only ~2 instructions are ever in flight so no shard can run dry -- an argument that
   // expired when M stopped blocking and the ROB started filling behind a waiting load. It
   // can fire at ROB_DEPTH=32: the IE shard holds 32 free registers beyond the architectural
   // set, and LOWAT stops fetch before they run out. It is a stall, handled here, not a fault.

   // OPERANDS NOW COME FROM THE SHARDED PRF.  The bypass is retained rather than leaning
   // on smolrv64_prf's write-through: both deliver the same value in the M->X case, and keeping
   // the existing mux means this commit changes the operand SOURCE without also changing
   // the operand TIMING PATH.  One variable at a time.
   // smolrv64_prf's write-through is OFF (WRTHRU=0), which is only safe while every read that
   // collides with the writeback is bypassed.  That is a property of IN-ORDER issue, not a
   // law -- so check it rather than remember it.  When out-of-order issue lands and this
   // fires, the fix is WRTHRU=1, not a patch here.
   // The WRTHRU check that lived here asserted a property of DISPATCH-time reads (every
   // read colliding with the writeback is bypassed). Reads happen at issue now and the
   // collision is covered by `fwd`, so the check is replaced by one on the new mechanism:
   // an issuing entry whose source is written this cycle must take the forwarded value.
   always @(posedge clk) if (!reset & (WRTHRU_OFF == 0)) begin
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

   // 3-producer forward: a source can be waiting on any of the three ALUs' writeback registers.
   // A physical register has one writer, so at most one of {fwd,fwdb,fwdc} is true -- the mux
   // order is immaterial.
   // The F/CTF port's operands, forwarded from the ALUs exactly like M's x_rs* (an FP op with
   // an integer source, or a CTF op, may consume an ALU result produced the same cycle).
   reg [63:0] xf_rs1, xf_rs2, xf_rs3;
   integer xfl;
   always @* begin
      xf_rs1 = prf_f1;  xf_rs2 = prf_f2;  xf_rs3 = prf_f3;
      for (xfl = NL - 1; xfl >= 0; xfl = xfl - 1) begin
         if (l_qv[xfl] & (j_ps1 == l_qprd[xfl])) xf_rs1 = l_qval[xfl];
         if (l_qv[xfl] & (j_ps2 == l_qprd[xfl])) xf_rs2 = l_qval[xfl];
         if (l_qv[xfl] & (j_ps3 == l_qprd[xfl])) xf_rs3 = l_qval[xfl];
      end
   end




   // ---- the lanes' loads and stores reach M through their queue entries (lanes step 5.2d-a) ----
   // A plain load or an integer store issues in its slot's lane, whose adder generates the VA; the
   // lane writes no register, wakes nothing and completes nothing. The VA (and a store's rs2 as the
   // lane read it) arrives at the op's own LQ or SQ entry, and M is the fill stage: the dTLB lookup,
   // the LQ/SQ fill, a load's early start and the address-only faults are M's, unchanged. M fills
   // in program order: it takes the oldest load or store whose address has not arrived, once it
   // has, so an op M holds (a fault waits for the ROB head) never has an older one behind it. The
   // rest of the op rides in a record written at dispatch.
   localparam integer SPW = ROB_IDXB + PLW;        // {ROB slot, payload}
   reg  [SPW-1:0] spl_ld [0:LQ_N-1];
   reg  [SPW-1:0] spl_st [0:SQ_N-1];
   // rs2 already in a register file at dispatch: the store queue reads it. Otherwise its snoop,
   // armed at allocation, takes it from the writeback, so the two never overlap.
   always @(posedge clk) begin
      if (d_ld_alloc) spl_ld[lq_d_idx] <= {gld_rob, gld_pl};
      if (d_st_alloc) spl_st[sq_d_idx] <= {gst_rob, gst_pl};
   end
   reg  [LQ_N-1:0] la_v;                           // a load's VA has arrived and M has not taken it
   reg  [SQ_N-1:0] sa_v;                           // ...a store's
   reg  [63:0]     la_va [0:LQ_N-1];
   reg  [63:0]     sa_va [0:SQ_N-1];
   initial begin la_v = {LQ_N{1'b0}}; sa_v = {SQ_N{1'b0}}; end
   // the oldest unfilled op: a load or a store
   wire              u_ld = lq_uf_any & (~sq_uf_any | older(lq_uf_seq, sq_uf_seq));
   wire              s_st = ~u_ld;
   // ...whose VA has arrived, or is in a lane's adder this cycle: M takes it from there, a cycle
   // sooner (the arrival registers still record it for when M is busy)
   function automatic by(input lm, input st, input [LQ_IB-1:0] li, input [SQ_IB-1:0] si);
      by = lm & (st ? (~u_ld & sq_uf_any & (si == sq_uf_idx)) : (u_ld & (li == lq_uf_idx)));
   endfunction
   wire [NL-1:0]     l_by;                       // lane k holds the oldest unfilled op's address
   reg  [63:0]       by_va;
   integer           bk;
   generate for (gl = 0; gl < NL; gl = gl + 1) begin: g_by
      assign l_by[gl] = by(l_lm[gl], l_st[gl], l_lqi[gl], l_sqt[gl]);
   end endgenerate
   always @* begin
      by_va = 64'd0;
      for (bk = NL - 1; bk >= 0; bk = bk - 1) if (l_by[bk]) by_va = l_addr[bk];
   end
   wire              s_byp = |l_by;
   assign s_win = s_byp | (u_ld ? la_v[lq_uf_idx] : (sq_uf_any & sa_v[sq_uf_idx]));
   wire              s_go = s_win & m_advance & ~redirect;
   wire [SPW-1:0]    s_rec = u_ld ? spl_ld[lq_uf_idx] : spl_st[sq_uf_idx];
   // THE HEAD-OP REGISTER. An AMO, LR/SC or CBO goes alone, so in slot A and lane A; the lane
   // generates its address and reads rs2 (an AMO's or SC's data), and the op waits here, its
   // record written at dispatch, for M, which runs it at the ROB head. One is in flight at a
   // time (the credit), and none while a load or store is unfilled (cbo_any, the serialising).
   reg               ho_v;
   reg  [63:0]       ho_va, ho_dat;
   reg  [SPW-1:0]    ho_rec;
   initial ho_v = 1'b0;
   wire              hA = l_v[0] & is_hop(ln[0].q_insn);   // a head op goes alone: slot A, lane A
   wire              h_go = ho_v & ~s_win & m_advance & ~redirect;
   always @(posedge clk) begin
      if (rn_valid & d_cls_l) ho_rec <= {s_rob[0], s_pl_in[0]};   // a head op goes alone: slot A
      if (hA) begin ho_v <= 1'b1; ho_va <= l_addr[0]; ho_dat <= l_rs2[0]; end
      if (h_go) ho_v <= 1'b0;
      if (reset | redirect) ho_v <= 1'b0;                 // flush last (rule I11)
   end
   wire [PLW-1:0]    pl_m  = s_win ? s_rec[PLW-1:0] : ho_rec[PLW-1:0];   // the op M takes
   wire [ROB_IDXB-1:0] m_go_rob = s_win ? s_rec[PLW +: ROB_IDXB] : ho_rec[PLW +: ROB_IDXB];
   wire              m_go = s_go | h_go;               // M takes an op (the pipe view's M stage)
   wire [63:0]       s_va  = s_byp ? by_va : u_ld ? la_va[lq_uf_idx] : sa_va[sq_uf_idx];
   integer           ak, aj;
   always @(posedge clk) begin
      for (ak = 0; ak < NL; ak = ak + 1)
         if (l_lm[ak] & ~l_st[ak]) begin la_v[l_lqi[ak]] <= 1'b1; la_va[l_lqi[ak]] <= l_addr[ak]; end
      for (ak = 0; ak < NL; ak = ak + 1)
         if (l_lm[ak] &  l_st[ak]) begin sa_v[l_sqt[ak]] <= 1'b1; sa_va[l_sqt[ak]] <= l_addr[ak]; end
      if (s_go &  u_ld) la_v[lq_uf_idx] <= 1'b0;   // after the arrival: a bypassed one is not left set
      if (s_go & ~u_ld) sa_v[sq_uf_idx] <= 1'b0;
      if (reset | redirect) begin la_v <= {LQ_N{1'b0}}; sa_v <= {SQ_N{1'b0}}; end   // flush last (rule I11)
   end
   always @(posedge clk) if (!reset) begin
      for (ak = 0; ak < NL; ak = ak + 1) begin
         for (aj = ak + 1; aj < NL; aj = aj + 1)
            if (l_lm[ak] & l_lm[aj] & (l_st[ak] == l_st[aj]) & (l_st[ak] ? l_sqt[ak] == l_sqt[aj] : l_lqi[ak] == l_lqi[aj]))
               $fatal(1, "smolrv64_core: lanes %0d and %0d deliver an address to the same queue entry", ak, aj);
         if (l_lm[ak] & (l_st[ak] ? sa_v[l_sqt[ak]] : la_v[l_lqi[ak]]))
            $fatal(1, "smolrv64_core: lane %0d delivers an address to an entry whose address M has not taken", ak);
      end
      if (ones_nl(l_by) > 4'd1)
         $fatal(1, "smolrv64_core: two lanes hold the oldest unfilled load or store");
      // the record M takes is this entry's own: a lane load or store written at its dispatch
      // a plain load or store never faults in M: its address-only fault rides in its entry
      if (lsu_fault & (m_ld_nb | m_st_nb))
         $fatal(1, "smolrv64_core: M holds a faulting plain load or store (pc %h)", m_pc);
      if (s_go & ~(is_lmem(qm_insn) & (qm_is_store == s_st) & (s_st ? (qm_sq_tag == sq_uf_idx) : (qm_lq_idx == lq_uf_idx))))
         $fatal(1, "smolrv64_core: M takes a queue entry's arrival whose record is another op's (pc %h)", qm_pc);
      // a head op goes alone, so in slot A; one is in flight at a time; it waits with no load
      // or store unfilled
      if (|(s_takev & s_cls_l & ~{{(IW-1){1'b0}}, 1'b1}))
         $fatal(1, "smolrv64_core: a head op dispatched beside another op");
      for (ak = 1; ak < NL; ak = ak + 1)
         if (l_v[ak] & is_hop(l_pl[ak][PL_INSN +: 32]))
            $fatal(1, "smolrv64_core: a head op executes in lane %0d, not lane A", ak);
      if (hA & ho_v)
         $fatal(1, "smolrv64_core: a head op arrives with the head-op register full");
      if (ho_v & (lq_uf_any | sq_uf_any))
         $fatal(1, "smolrv64_core: a head op waits beside an unfilled load or store (pc %h)", ho_rec[PL_INSN + 32 +: PCW]);
   end
   // the same unpack for the op M takes
   `PL_DECL(qm_)
   assign `PL_REC(qm_) = pl_m;

   // =========================================================== stage M
`include "smolrv64_fp_ops.vh"           // fcmp_s/d, fclass_s/d (in-core FP ops)

   wire        fs_off;
   wire [2:0]  csr_frm;

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
   wire        m_mem_op = m_valid & (m_is_mem | m_is_amo);

   // A CBO executes from M and is not serialized (cbo.zero clears every page the kernel
   // hands out), but it writes memory (cbo.zero, cbo.inval), and M speculates past unresolved
   // branches: it starts only when its op is the ROB head, and the LSU asserts that (rule D17).
   // At the head every older load has landed, so no load-queue entry is older than the CBO
   // (asserted below). Older stores can still sit in the queue -- the senior store queue holds
   // RETIRED stores until they drain -- and "an older store is live" is `sq_av_any`: no load or
   // store dispatches while a CBO is in flight (`cbo_any`), so every store in the queue is older
   // (rule C5). The other M-executed
   // accesses are covered elsewhere: AMO/LR/SC are serializing (`drained`), a load's early
   // start asks `ld_older`.
   wire m_cbo_wait = m_is_cbo & (~m_at_head | sq_av_any);
   always @(posedge clk)
      if (!reset && m_mem_op && m_is_cbo && m_at_head && lq_av_any)
         $fatal(1, "smolrv64_core: a load with an address is live under a CBO at the ROB head (pc %h)", m_pc);
   smolrv64_lsu #(.AW(AW), .DRAM_TOP(DRAM_TOP), .LRAM_BASE(LBASE), .LRAM_LG2(LRAM_LG2), .LDTW(LQ_IB)) u_lsu
     (.clk(clk), .reset(reset),
      .dtlb_walking(lsu_dtlb_walking), .dtlb_walk_beg(lsu_dtlb_walk_beg),
      // NOT m_mem_op alone. While M holds a COMPLETED op (its done pulse latched, waiting on
      // ld_land or the ROB head) the request would still be presented, the LSU would fall
      // back to S_IDLE, and xl_req would start the very same access A SECOND TIME -- a store
      // written twice. mul/div are already safe this way via md_started; the LSU was not.
      .req_valid(m_mem_op & ~m_unit_done_q & ~m_cbo_wait),
      // A plain store TRANSLATES here and goes no further: its address lands in smolrv64_sq and
      // memory is written later, from the buffer's commit port below.
      // Loads AND buffered stores translate here and go no further; the access itself
      // comes back through the pre-translated port, from smolrv64_lq or smolrv64_sq.
      .req_xlate(m_st_nb | m_ld_nb), .req_early(lq_b_early), .xo_pa(lsu_xo_pa), .xo_unc(lsu_xo_unc), .xo_mem(lsu_xo_mem), .xo_v(lsu_xo_v), .xo_flt(lsu_xo_flt), .xo_fc(lsu_xo_fc),
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
      .xo_tv(lsu_xo_tv), .wk_v(wk_v), .wk_st(wk_st), .wk_va(wk_va), .wk_done(lsu_wk_done),
      .wk_pa(lsu_wk_pa), .wk_unc(lsu_wk_unc), .wk_mem(lsu_wk_mem), .wk_flt(lsu_wk_flt), .wk_fc(lsu_wk_fc),
      .req_fp(m_is_fp), .req_st_data(m_st_data),
      .xl_satp(satp_data), .xl_priv(mmu_dpriv), .xl_sum(mmu_sum), .xl_mxr(mmu_mxr),
      .xl_flush(mmu_flush_q), .flush(redirect), .m_head(m_at_head),
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
      .mem_wready(dmem_wready), .mem_waccept(dmem_waccept), .mem_wroom(dmem_wroom),
      .cos_pa(lsu_cos_pa), .cos_kind(lsu_cos_kind), .cos_data(lsu_cos_data), .cos_size(lsu_cos_size),
      .started(lsu_started), .done(lsu_done), .done_acc(lsu_done_acc), .rd_val(lsu_rd_val), .fault(lsu_fault),
      .fault_cause(lsu_fault_cause), .fault_tval(lsu_fault_tval), .ld_busy(lsu_ld_busy),
      .err(lsu_err), .idle(lsu_idle));
   // FENCE.I's drain: every store older than it handed to the D$. The store queue is senior to
   // retirement, so at the fire a committed store may still wait in it; after the redirect it holds
   // only older stores, so the wait ends.
   assign dmem_idle = lsu_idle & (sq_occ == 0);

   // ---- the MD stage: mul/div off the ordered pipe (C1, 2026-09-17) ----
   // The F port's third drain. One divide at a time (the divider is single-occupancy), its
   // operands captured from the port's forwarded reads in the issue cycle exactly as the F stage
   // captures them, and landing by its own tag like the FPU: rob and prd ride in the stage.
   reg                md_v, md_div, md_rd_v, md_pend;
   reg [ROB_IDXB-1:0] md_rob;
   reg [RN_PBITS-1:0] md_prd;
   reg [63:0]         md_res_q;
   initial begin md_v = 1'b0; md_pend = 1'b0; end
   wire        div_done, div_busy;
   wire [63:0] div_result;
   wire        md_f3_2 = qf_insn[14];                     // funct3[2]: div/rem
   // The result is LATCHED on the unit's done pulse (mul3's persists, the divider's is one cycle)
   // and leaves when the FPU is not landing: the FPU cannot hold its result, this can.
   wire        md_done = md_v & ~md_pend & div_done;
   wire        md_wb   = md_pend & (~md_rd_v | ~fp_wb);   // the op completes (ROB)
   wire        md_wr   = md_wb & md_rd_v;                                    // ...and writes its register
   assign      md_advance = ~md_v | md_wb;
   // The divider starts a cycle after the issue, from registered operands: its start-cycle
   // operand preparation (sign, magnitude, the special cases) stays off the port's forwarded reads.
   reg         dv_go;
   reg  [63:0] dv_a, dv_b;
   reg  [2:0]  dv_f3;
   reg         dv_w;
   initial dv_go = 1'b0;
   always @(posedge clk) begin
      dv_go <= ~reset & ~redirect & iss_md & md_f3_2;
      dv_a <= xf_rs1;  dv_b <= xf_rs2;  dv_f3 <= qf_insn[14:12];  dv_w <= qf_insn[6:2] == 5'b01110;
   end
   divider u_div
     (.clk(clk), .reset(reset), .start(dv_go), .abort(redirect),
      .rs1(dv_a), .rs2(dv_b), .f3(dv_f3), .is_w(dv_w),
      .busy(div_busy), .done(div_done), .result(div_result));
   always @(posedge clk) begin
      // The writeback's clear comes FIRST: an issue in the same cycle (md_advance = md_wb admits
      // it) must win, or the stage forgets an op the unit is already computing and the next
      // issue starts into a busy multiplier, which ignores it -- rv64um-p-mul hung at retire 111.
      if (md_done) begin md_pend <= 1'b1; md_res_q <= div_result; end
      if (md_wb)   begin md_v <= 1'b0; md_pend <= 1'b0; end
      if (iss_md) begin
         md_v <= 1'b1; md_div <= md_f3_2; md_rob <= j_rob; md_prd <= qf_prd; md_rd_v <= qf_rd_v; md_pend <= 1'b0;
      end
      // abort(redirect) on the units: a redirect is head-gated, so an in-flight mul/div is
      // younger than the redirecting op = wrong-path. The flush arm is last (rule I11).
      if (redirect) begin md_v <= 1'b0; md_pend <= 1'b0; end
   end
   always @(posedge clk) if (!reset) begin
      if (iss_md & (qf_shard >= SH_F0))  $fatal(1, "smolrv64_core: a divide issued with an FP shard %0d", qf_shard);
      if (iss_md & md_v & ~md_wb)        $fatal(1, "smolrv64_core: a mul/div issued into a busy MD stage");
      if (dv_go & div_busy)              $fatal(1, "smolrv64_core: a divide started into a busy divider (the start would be ignored)");
      if (iss_md & ~md_f3_2)             $fatal(1, "smolrv64_core: a multiply reached the MD stage, not its lane");
      if (m_valid & ~(m_is_mem | m_is_amo)) $fatal(1, "smolrv64_core: M holds an op that is not a load, store or head op (pc %h)", m_pc);
      if (md_wr & fp_wb)                 $fatal(1, "smolrv64_core: the MD stage and the FPU wrote in one cycle");
   end

   // ---- FP unit (CVFPU) + the in-core FP ops ----
   // Everything the OoO core needs for squash recovery -- the zombie/drain latch, the
   // abort-by-seqno, the FS-dirty commit gate -- is absent here: M is the commit point,
   // so an FP op in flight can never be squashed, and its decode/operands are stable for
   // its whole (multi-cycle) stay. `decode_fp` runs off the registered m_insn.

   // ---- the SYSQ (C3 step 3, 2026-09-18): system ops fire at the ROB head from flops ------------
   // A CSR/system op (SYSTEM opcode: csr*, ecall/ebreak/xret/wfi/sfence.vma, the irqop pseudo-op)
   // is the F/CTF/MD port's fourth drain. At most ONE is in flight: every one but a CSR op is
   // serialising at dispatch (ser_block drains the ROB and the store queue before it and lets
   // nothing dispatch behind it), and a CSR op waits at dispatch for the one before it (csr_infl).
   // The queue is this one register, and its fire is a flop compare at the ROB head. csr_file's
   // upd_* port is driven from these flops (step 2's lesson: a LUTRAM read in front of
   // csr_file's combinational redirect cost IW=3 its closure); the CSR read address is sy_addr,
   // a flop. The read's value and the completion go through lane A's write slot, which is free
   // when the SYSQ fires (M, whose op is younger then, does not complete while the SYSQ's op is
   // the head); a trap through c_kill; a redirect through the one redirect gate. A read of instret takes its second cycle at head so the delayed retire
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
   reg                sy_xt;     // the op traps at dispatch (fetch fault, illegal): csr_file's xtrap
   reg                sy_fn;     // a FENCE or FENCE.I: no csr_file action...
   reg                sy_fi;     // ...and FENCE.I flushes the caches and refetches after itself
   reg [3:0]          sy_xcause;
   reg [63:0]         sy_xtval;
   reg                sy_qf;     // the trap is a queue entry's translation fault (the op is a load or store)
   initial begin sy_v = 1'b0; sy_head_q = 1'b0; sy_xt = 1'b0; sy_fn = 1'b0; sy_fi = 1'b0; sy_qf = 1'b0; sy_instret = 1'b0; end
   // A QUEUE ENTRY WHOSE TRANSLATION FAULTED TRAPS FROM HERE. It never completes (a load is never
   // sent, a store never commits), so its op reaches the ROB head and stays; its record then loads
   // the SYSQ as a trap from dispatch does, and fires through sy_fire, the one trap gate. The SYSQ
   // is free then or holds a younger CSR op, which the record displaces.
   // The record is registered on its way in, so the SYSQ's inputs are flops, never the queues'
   // muxes and the head compare; its valid bit leaves out a redirect cycle (the entry dies at that
   // edge), and the payload rides with no redirect in its enable (rule I11).
   // An entry's full VA, written at its fill: a trap's tval (a non-canonical address is wider than
   // the entries' VA[38:0])
   reg  [63:0] fva_l [0:LQ_N-1];
   reg  [63:0] fva_s [0:SQ_N-1];
   always @(posedge clk) begin
      if (m_lq_fill) fva_l[m_lq_idx] <= m_addr;
      if (m_sq_fill) fva_s[m_sq_tag] <= m_addr;
   end
   wire qf_sq = sq_f_v & (sq_f_rob == rob_head_idx) & ~rob_empty;
   wire qf_lq = lq_f_v & (lq_f_rob == rob_head_idx) & ~rob_empty;
   reg                qfr_v;
   reg [ROB_IDXB-1:0] qfr_rob;
   reg [3:0]          qfr_fc;
   reg [63:0]         qfr_va;
   reg [38:0]         qfr_pc;
   reg [SEQW-1:0]     qfr_seq;
   initial qfr_v = 1'b0;
   always @(posedge clk) begin
      qfr_v   <= ~reset & ~redirect & (qf_sq | qf_lq);
      qfr_rob <= qf_sq ? sq_f_rob : lq_f_rob;
      qfr_fc  <= qf_sq ? sq_f_fc  : lq_f_fc;
      qfr_va  <= qf_sq ? fva_s[sq_k_idx] : fva_l[lq_k_idx];   // the faulting entry is the candidate
      qfr_pc  <= qf_sq ? sq_f_pc  : lq_f_pc;
      qfr_seq <= qf_sq ? sq_f_seq : lq_f_seq;
   end
   // The faulting entry is the ROB head, so anything else in the SYSQ (a CSR op waiting for the
   // head) is younger and dies at the trap's redirect: the record takes the SYSQ from it.
   wire sy_at_head = sy_v & (sy_rob == rob_head_idx);
   wire qf_in = qfr_v & ~sy_at_head;
   reg  sy_instret;                     // a CSR op on instret/minstret, decoded as it enters
   wire sy_fire    = sy_at_head & sy_lane_ok & (~sy_instret | sy_head_q);   // its result takes lane A's slot
   wire sy_xfire   = sy_fire & sy_xt;                    // ...a trap from dispatch: csr_file's xtrap
   // csr_file leaves its xtrap input out of redir_valid/redir_is_trap (a combinational loop
   // otherwise), so a trap from dispatch is its own redirect here, as M's data fault is.
   wire sy_red     = sy_fire & (csr_redir_v | sy_xt | sy_fi);  // trap, xret, illegal CSR, fence.i: a redirect
   wire sy_trap    = sy_fire & (csr_redir_trap | sy_xt);  // ...that is an exception: kill, no result
   wire sy_done    = sy_fire & ~sy_trap;                 // completes through lane A's landing slot
   // the CSR read's value into lane A. A CSR op that csr_file traps writes its register too: the
   // trap flushes every reader, and the write and its wake stay clear of csr_file's decision.
   wire sy_wr      = sy_fire & sy_rd_v & ~sy_xt;
   assign sy_advance = ~sy_v | sy_fire;
   always @(posedge clk) begin
      sy_head_q <= ~reset & sy_at_head & ~sy_fire;
      if (sy_fire) sy_v <= 1'b0;
      if (iss_sys) begin
         sy_v <= 1'b1; sy_rob <= j_rob; sy_prd <= qf_prd; sy_rd_v <= qf_rd_v;
         sy_is_csr <= qf_is_csr; sy_func <= qf_csr_func; sy_addr <= qf_imm[11:0];
         sy_instret <= qf_is_csr & ((qf_imm[11:0] == 12'hC02) | (qf_imm[11:0] == 12'hB02));
         sy_src <= qf_csr_func[2] ? {59'b0, qf_imm[16:12]} : xf_rs1;
         sy_pc <= qf_pc; sy_seq <= qf_seq; sy_insn <= qf_insn;
         sy_xt <= j_istrap;
         sy_fn <= ~j_istrap & (qf_insn[6:2] == 5'b00011);
         sy_fi <= ~j_istrap & (qf_insn[6:2] == 5'b00011) & qf_insn[12];
         sy_xcause <= qf_fault ? qf_fault_cause : 4'd2;
         sy_xtval  <= qf_fault ? {{(64-PCW){1'b0}}, qf_fault_tval} : 64'd0;
         sy_qf     <= 1'b0;
      end
      if (qf_in) begin
         sy_v <= 1'b1;  sy_rob <= qfr_rob;  sy_prd <= {RN_PBITS{1'b0}};  sy_rd_v <= 1'b0;
         sy_is_csr <= 1'b0;  sy_func <= 3'd0;  sy_addr <= 12'd0;  sy_src <= 64'd0;  sy_insn <= 32'd0;
         sy_instret <= 1'b0;
         sy_pc  <= {{(PCW-39){qfr_pc[38]}}, qfr_pc};
         sy_seq <= qfr_seq;
         sy_xt <= 1'b1;  sy_fn <= 1'b0;  sy_fi <= 1'b0;  sy_qf <= 1'b1;
         sy_xcause <= qfr_fc;
         sy_xtval  <= qfr_va;
      end
      if (reset | redirect) sy_v <= 1'b0;                // flush arm last (I11); its own redirect included
   end
   always @(posedge clk) if (!reset) begin
      if (iss_sys & sy_v & ~sy_fire)     $fatal(1, "smolrv64_core: a system op issued into a busy SYSQ (it is serialising)");
      if (qf_in & sy_v & ~sy_is_csr)      $fatal(1, "smolrv64_core: a queue entry's fault displaces a SYSQ op that is not a younger CSR op");
      if (qf_sq & qf_lq)                 $fatal(1, "smolrv64_core: a load and a store both fault as the ROB head");
      // (a queue entry's trap is a load's or store's: younger work may be in flight, and dies)
      // (a CSR op does not drain: younger work may be in flight; M yields to it, see m_done)
      if (sy_fire & ~sy_qf & ~sy_is_csr & m_valid) $fatal(1, "smolrv64_core: a system op fires with M busy (pc %h): the drain is broken", sy_pc);
      if (sy_fire & ~sy_qf & ~sy_is_csr & (fpu_busy | f_valid | icr_v)) $fatal(1, "smolrv64_core: a system op fires with FP work in flight -- frm/fflags may change under it");
      if (rn_valid & d_csr_op & csr_infl) $fatal(1, "smolrv64_core: a second CSR op dispatched with one in flight");
      if (m_valid & m_is_sys)            $fatal(1, "smolrv64_core: a system op reached M (pc %h)", m_pc);
      if (m_valid & (m_insn[6:2] == 5'b00011) & (m_insn[14:13] == 2'b00))
                                         $fatal(1, "smolrv64_core: a fence reached M (pc %h)", m_pc);
      if (sy_red & (m_red_fire | cf_red_fire)) $fatal(1, "smolrv64_core: the SYSQ redirects together with M or the CTF pipe");
      // csr_file does not redirect on its xtrap input by itself: every source of one names it
      // in its own redirect (M's csr_red, the SYSQ's sy_red), and this is the check that it did.
      if (((xtrap_v & m_done_red) | sy_xfire) & ~redirect)
         $fatal(1, "smolrv64_core: a trap reached csr_file without a redirect (M %b, SYSQ %b)",
                xtrap_v & m_done_red, sy_xfire);
   end


   function [63:0] fpsel; input [1:0] s; input [63:0] a, b, c;
      fpsel = (s==2'd1) ? a : (s==2'd2) ? b : (s==2'd3) ? c : 64'd0; endfunction
   // unbox a single from an f-register: a properly NaN-boxed value yields its low 32
   // bits, anything else is the canonical single NaN (RISC-V spec).
   function [31:0] unbox_s; input [63:0] x;
      unbox_s = (x[63:32]==32'hffffffff) ? x[31:0] : 32'h7fc00000; endfunction


   wire        fp_iss_ready, fp_res_valid, fpu_busy;
   wire [63:0] fp_res_data;
   wire [4:0]  fp_res_fflags;
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
      // its physreg away -- the zombie writeback, caught by smolrv64_pending as "writeback to
      // pN, which was not pending" on rv64ud-p-structural. It only became reachable once
      // FP could reorder and run several deep.
      //
      // Flushing EVERY in-flight op is safe only because head_block still holds: a
      // redirect fires only when its op is the ROB head, so everything older has already
      // committed, and anything in flight is younger by construction. If head_block goes
      // (work list P2), this needs an age or epoch tag instead.
      .res_fflags(fp_res_fflags), .res_tag(fp_res_tag), .flush(redirect), .busy(fpu_busy), .err(fpu_err), .dbg(fpu_dbg));

   wire fp_complete = fp_res_valid | icr_v;             // a result lands: the FPU's, else the in-core one
   wire [FTAGW-1:0] fl_tag    = fp_res_valid ? fp_res_tag    : icr_tag;
   wire [63:0]      fl_data   = fp_res_valid ? fp_res_data   : icr_data;
   wire [4:0]       fl_fflags = fp_res_valid ? fp_res_fflags : {icr_nv, 4'd0};

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

   // THE IN-CORE FP OPS (FSGNJ/N/X, FEQ/FLT/FLE, FCLASS, FMV both ways) run here in one cycle, off
   // the stage's operand registers, into a one-entry result register that lands through the
   // FPU's own landing (fp_land: the FE shard, the ROB port, the flags) in any cycle the FPU is
   // not landing. The FPU keeps priority, so its handshake is unchanged; a waiting in-core
   // result holds the stage.
   wire        f_ic     = f_valid & ~ff_use_fpu;
   wire        fp_start = f_valid & ff_use_fpu & ~redirect;
   wire        fp_disp  = fp_start & fp_iss_ready;   // accepted by the unit this cycle
   reg         icr_v, icr_nv;
   reg  [63:0] icr_data;
   reg  [FTAGW-1:0] icr_tag;
   initial icr_v = 1'b0;
   wire        icr_land  = icr_v & ~fp_res_valid;
   wire        icr_take  = f_ic & ~redirect & (~icr_v | icr_land);
   assign      f_advance = ~f_valid | fp_disp | icr_take;
   wire        fi_isd = f_insn[25];                  // 0=single 1=double
   wire [2:0]  fi_f3  = f_insn[14:12];
   wire [31:0] fi_u1  = unbox_s(f_rs1_val);          // FMV.X.W is a raw bit-move: never unboxed
   wire [31:0] fi_u2  = unbox_s(f_rs2_val);
   wire [1:0]  fi_cd  = fcmp_d(fi_f3, f_rs1_val, f_rs2_val);
   wire [1:0]  fi_cs  = fcmp_s(fi_f3, fi_u1, fi_u2);
   reg  [63:0] fi_res;
   always @* begin
      case (ff_cls)
        3'd1: fi_res = fi_isd                                          // FSGNJ/N/X .D / .S
               ? (fi_f3==3'b000 ? { f_rs2_val[63],               f_rs1_val[62:0]}
                : fi_f3==3'b001 ? {~f_rs2_val[63],               f_rs1_val[62:0]}
                :                 { f_rs2_val[63]^f_rs1_val[63], f_rs1_val[62:0]})
               : {32'hffffffff, (fi_f3==3'b000 ? { fi_u2[31],            fi_u1[30:0]}
                               : fi_f3==3'b001 ? {~fi_u2[31],            fi_u1[30:0]}
                               :                 { fi_u2[31]^fi_u1[31],  fi_u1[30:0]})};
        3'd2: fi_res = {63'd0, (fi_isd ? fi_cd[0] : fi_cs[0])};          // FEQ/FLT/FLE -> int
        3'd3: fi_res = fi_isd ? f_rs1_val : {{32{f_rs1_val[31]}}, f_rs1_val[31:0]}; // FMV.X.D/W -> int (raw)
        3'd4: fi_res = fi_isd ? f_rs1_val : {32'hffffffff, f_rs1_val[31:0]};        // FMV.D/W.X -> fp (box)
        3'd5: fi_res = fi_isd ? fclass_d(f_rs1_val) : fclass_s(f_rs1_val);           // FCLASS -> int
        default: fi_res = 64'd0;
      endcase
   end
   always @(posedge clk) begin
      if (icr_land) icr_v <= 1'b0;
      if (icr_take) begin
         icr_v <= 1'b1;  icr_data <= fi_res;
         icr_tag <= {1'b0, f_rd_v, f_rd, f_rob, f_prd};   // the FPU's tag layout, never re-boxed
         icr_nv <= (ff_cls == 3'd2) & (fi_isd ? fi_cd[1] : fi_cs[1]);
      end
      // younger than the redirecting head: wrong-path. Ordered last, on the valid bit alone (rule I11)
      if (reset | redirect) icr_v <= 1'b0;
   end

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
      if (f_valid & ~ff_valid_d)
         $fatal(1, "smolrv64_core: F stage holds a non-FP op (insn %08x)", f_insn);
      if (m_valid & m_is_fp & ~m_is_mem)
         $fatal(1, "smolrv64_core: a non-memory FP op reached M (pc %h)", m_pc);
      if (iss_f & ~sh_used(qf_shard))
         $fatal(1, "smolrv64_core: F-class op is in shard %0d, which this width does not use", qf_shard);
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
   end

   // THE ONE YIELD GATE: every completion that shares M's ROB port (a landing load, the FPU)
   // is gathered here once and applied at every site -- M's three dones and the SYSQ's fire --
   // never re-derived per unit (docs/rtl-rules.md).
   wire port_yield    = ld_land | fp_land;

   // =========================================================== control flow, resolved in the lanes
   // A branch, jal or jalr issues in its slot's lane, whose exec resolves it; its link is the
   // lane's ALU result, written into the lane's shard at issue like any ALU op. Each lane
   // registers what it resolved (lr_*), so the restart and the training below read flops, never
   // an exec compare. A correctly predicted CTI completes at issue on its lane's ROB port; a
   // mispredict completes at its squash (cf_red_fire, by fr_rob), so the ROB head stops on it.
   reg  [NL-1:0]       lr_v, lr_mis, lr_br, lr_jmp, lr_jalr, lr_taken, lr_rvc, lr_rdv;
   reg  [SEQW-1:0]     lr_seq  [0:NL-1];
   reg  [ROB_IDXB-1:0] lr_rob  [0:NL-1];
   reg  [PCW-1:0]      lr_pc   [0:NL-1];
   reg  [PCW-1:0]      lr_tgt  [0:NL-1];
   reg  [PCW-1:0]      lr_ttgt [0:NL-1];
   reg  [PDW-1:0]      lr_pdet [0:NL-1];
   reg  [5:0]          lr_rd   [0:NL-1];
   reg  [5:0]          lr_rs1  [0:NL-1];
   initial lr_v = {NL{1'b0}};
   wire [SEQW-1:0]     tr_seq;     // the seq of the CTI that trains this cycle (defined with the training)
   integer lrk;
   always @(posedge clk)
      for (lrk = 0; lrk < NL; lrk = lrk + 1) begin
         lr_v[lrk] <= ~reset & ~redirect & l_cti[lrk];
         lr_mis[lrk] <= l_mis[lrk];
         lr_br[lrk] <= l_br[lrk];  lr_jmp[lrk] <= l_jmp[lrk];  lr_jalr[lrk] <= l_jalr[lrk];
         lr_taken[lrk] <= l_taken[lrk];  lr_rvc[lrk] <= l_rvc[lrk];  lr_rdv[lrk] <= l_rdv[lrk];
         lr_seq[lrk] <= l_seq[lrk];   lr_rob[lrk] <= l_rob[lrk];
         lr_pc[lrk]  <= l_pc[lrk];    lr_tgt[lrk] <= l_tgt[lrk][PCW-1:0];
         lr_ttgt[lrk] <= l_ttgt[lrk][PCW-1:0];  lr_pdet[lrk] <= l_pdet[lrk];
         lr_rd[lrk]  <= l_rd[lrk];    lr_rs1[lrk] <= l_rs1[lrk];
      end
   function older(input [SEQW-1:0] a, input [SEQW-1:0] b);   // wrap-safe: a is older than b
      older = $signed(a - b) < 0;
   endfunction
   // The oldest mispredict among the lanes (the restart below compares it with the tracked one).
   localparam integer NLB = $clog2(NL);
   wire [NL-1:0] lm = lr_v & lr_mis;
   reg  [NL-1:0] lw;
   reg  [NLB-1:0] wl;                                              // the winner's lane
   integer lwi, lwj;
   always @* begin
      wl = {NLB{1'b0}};
      for (lwi = 0; lwi < NL; lwi = lwi + 1) begin
         lw[lwi] = lm[lwi];
         for (lwj = 0; lwj < NL; lwj = lwj + 1)
            if (lwj != lwi) lw[lwi] = lw[lwi] & (~lm[lwj] | older(lr_seq[lwi], lr_seq[lwj]));
      end
      for (lwi = NL - 1; lwi > 0; lwi = lwi - 1) if (lw[lwi]) wl = lwi[NLB-1:0];
   end
   wire                cf_mis       = |lm;
   wire [SEQW-1:0]     cf_seq       = lr_seq[wl];
   wire [ROB_IDXB-1:0] cf_rob       = lr_rob[wl];
   wire [PCW-1:0]      cf_target    = lr_tgt[wl];
   wire [PDW-1:0]      cf_pdet      = lr_pdet[wl];
   wire                cf_is_branch = lr_br[wl], cf_is_jump = lr_jmp[wl], cf_is_jalr = lr_jalr[wl];
   wire                cf_taken     = lr_taken[wl];
   // call/return classification for the RAS
   wire                cf_link_rd   = lr_rdv[wl] & ((lr_rd[wl] == 6'd1) | (lr_rd[wl] == 6'd5));
   wire                cf_link_rs   = (lr_rs1[wl] == 6'd1) | (lr_rs1[wl] == 6'd5);
   always @(posedge clk) if (!reset) begin
      if (ones_nl(lw) > 4'd1)
         $fatal(1, "smolrv64_core: %0d lanes each claim the oldest mispredict", ones_nl(lw));
      if (cf_mis & ~|lw)
         $fatal(1, "smolrv64_core: lanes mispredict but none is the oldest");
   end

   // ---- CSR file ----
   wire        m_is_sys  = m_valid & (m_insn[6:2] == 5'b11100);
   wire [63:0] csr_rdata, csr_redir_tgt;
   wire        csr_redir_v, csr_redir_trap, csr_illegal;
   wire        csr_irq_v;
   wire [3:0]  csr_irq_cause;

   // M's only trap is a data fault: a fetch fault and an illegal instruction trap from the SYSQ
   wire        xtrap_v     = m_mem_op & m_lsu_flt;
   wire        m_done_red;                // M's done with the LSU arm removed; defined with redirect
   wire [3:0]  xtrap_cause = m_lsu_fc;
   wire [63:0] xtrap_tval  = lsu_fault_tval;

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
   // Gated on "no lane issued", not on M: M busy with loads would mask an FP chain that is the
   // critical path.
   // The schedulers' blocking reports are registered first: classified a cycle later, they keep
   // the wake and select out of the event bus.
   reg  no_issue, rf_blk_vq;
   reg  [RN_PBITS-1:0] rf_blk_pq;
   reg  [NL-1:0] l_blk_vq;
   reg  [RN_PBITS-1:0] l_blk_pq [0:NL-1];
   integer nik;
   always @(posedge clk) begin
      no_issue <= 1'b1;
      for (nik = 0; nik < NL; nik = nik + 1) begin
         if (l_pick[nik]) no_issue <= 1'b0;
         l_blk_vq[nik] <= l_blk_v[nik];  l_blk_pq[nik] <= l_blk_pr[nik];
      end
      rf_blk_vq <= rf_blk_v;  rf_blk_pq <= rf_blk_pr;
   end
   // A register a load, AMO or CSR read will write, all in a lane's shard: a live LQ entry's
   // destination, a landing waiting in a lane's buffer, M's AMO or the SYSQ's op. An FP load's is
   // an f-register, counted under dep_fp below.
   // the landing buffers' waiting destinations, 0 for an empty slot (a wire: the function below
   // reads no array, rule F4)
   wire [NL*LBN*RN_PBITS-1:0] lb_live, lb_live_fe;
   genvar glb;
   generate
      for (glb = 0; glb < NL*LBN; glb = glb + 1) begin : g_lb_live
         wire [LBB-1:0] dl = glb[LBB-1:0] - lb_h[glb / LBN];   // the entry's age in its lane's ring
         wire lv = {1'b0, dl} < lb_n[glb / LBN];
         assign lb_live[glb*RN_PBITS +: RN_PBITS]    = (lv & ~lb_fe[glb]) ? lb_prd[glb] : {RN_PBITS{1'b0}};
         assign lb_live_fe[glb*RN_PBITS +: RN_PBITS] = (lv &  lb_fe[glb]) ? lb_prd[glb] : {RN_PBITS{1'b0}};
      end
   endgenerate
   function automatic ld_dst(input [RN_PBITS-1:0] p);
      integer k;
      begin
         ld_dst = (m_valid & m_is_amo & m_rd_v & (m_prd == p)) | (sy_v & sy_rd_v & (sy_prd == p));
         for (k = 0; k < LQ_N; k = k + 1)
            if ((lq_e_dprd[k*RN_PBITS +: RN_PBITS] == p) & (p[RN_PBITS-1:RN_IDXB] < SH_F0)) ld_dst = 1'b1;
         for (k = 0; k < NL*LBN; k = k + 1)
            if ((lb_live[k*RN_PBITS +: RN_PBITS] == p) & (p != {RN_PBITS{1'b0}})) ld_dst = 1'b1;
      end
   endfunction
   reg  dep_ld_l, dep_fp_l;                     // a lane's oldest blocked operand waits on a load / FP
   integer dpk;
   always @* begin
      dep_ld_l = 1'b0;  dep_fp_l = 1'b0;
      for (dpk = 0; dpk < NL; dpk = dpk + 1) begin
         dep_ld_l = dep_ld_l | (l_blk_vq[dpk] & ld_dst(l_blk_pq[dpk]));
         dep_fp_l = dep_fp_l | (l_blk_vq[dpk] & fe_dst(l_blk_pq[dpk]));
      end
   end
   wire dep_ld    = no_issue & ((rf_blk_vq & ld_dst(rf_blk_pq)) | dep_ld_l);
   // An f-register (FP slices: an FP op's or an FP load's), or an FP op's or divide's integer
   // result: in the F stage, in the MD stage, or waiting in a lane's landing buffer.
   function automatic fe_dst(input [RN_PBITS-1:0] p);
      integer k;
      begin
         fe_dst = (p[RN_PBITS-1:RN_IDXB] >= SH_F0) | (f_valid & (f_prd == p))
                | (md_v & md_rd_v & (md_prd == p));
         for (k = 0; k < NL*LBN; k = k + 1)
            if ((lb_live_fe[k*RN_PBITS +: RN_PBITS] == p) & (p != {RN_PBITS{1'b0}})) fe_dst = 1'b1;
      end
   endfunction
   wire dep_fp    = no_issue & ((rf_blk_vq & fe_dst(rf_blk_pq)) | dep_fp_l);
   wire st_mem    = (st_m & m_mem_op) | dep_ld;     // ...on the LSU
   wire st_div    = md_v;                           // the MD stage holds a divide (occupancy, not a stall)
   reg  st_mul;                                     // a multiply in flight in a lane
   integer smk;
   always @* begin st_mul = 1'b0; for (smk = 0; smk < NL; smk = smk + 1) st_mul = st_mul | l_m1[smk] | l_m2[smk]; end
   // ST_FPU watches stage F: `f_valid & ~fp_disp` is stage F holding an op the unit will not
   // yet take. (~f_advance is the same expression; written out for clarity.)
   wire st_fpu    = (f_valid & ~fp_disp) | dep_fp;  // ...on the FPU
   // ~st_rob: the two are now disjoint, so the stack does not count a ROB-full cycle
   // twice under two different names.
   // THE DISPATCH HOLD, NAMED (2026-09-07). `st_ser` was this whole residual -- M could advance,
   // dispatch did not, and it was not a load or FP dependency or the ROB -- and it read 6-13%
   // of the cycles in EVERY Geekbench subtest, where serializing ops are rare: what it held
   // was the schedulers filling up behind integer chains, the store and load queues, the
   // rename free lists. Each cause has its own event now, in d_hold's order so they are
   // disjoint and sum to ST_DSP, the bucket kept whole for the 13-counter `cpi` set.
   // A dispatch stall is the queue head holding an instruction it has no credit for, outside a
   // freeze (the credits, smolrv64_frontend): nothing after the head waits.
   wire hd_wait   = fe_hd_v & ~fe_hd_take & ~redirect_q & ~fr_v & ~dec_red_q;
   wire hd_sched  = (~fe_hd_gc[GC_I] | cr_i[0]) & (~fe_hd_gc[GC_L] | cr_l) & (~fe_hd_gc[GC_FC] | cr_f);
   wire hd_sq     = fe_hd_gc[GC_ST] & ~cr_st;
   wire hd_lq     = fe_hd_gc[GC_LD] & ~cr_ld;
   wire st_dsp    = m_advance & hd_wait & ~dep_ld & ~dep_fp & ~st_rob;
   wire st_iq     = st_dsp & ~hd_sched;                              // the class's scheduler is full
   wire st_rn     = st_dsp &  hd_sched & rn_stall;                   // rename: a free list is low
   wire st_sq     = st_dsp &  hd_sched & ~rn_stall & hd_sq;
   wire st_lq     = st_dsp &  hd_sched & ~rn_stall & ~hd_sq & hd_lq;
   wire st_srz    = st_dsp &  hd_sched & ~rn_stall & ~hd_sq & ~hd_lq;
   wire st_ser    = st_dsp;                                          // the bus's bit 11, as before
   wire fe_bub    = ~st_m & ~d_valid & ~hd_wait & ~redirect;   // X starved, M not already stalled
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
   wire red_br    = cf_red_fire & fr_br;
   wire red_jalr  = cf_red_fire & fr_jalr;

   // ST_ROB: dispatch has an instruction and the ROB has no room. Split out of ST_SER
   // because that event's name says "serializing op" while it actually absorbed EVERY
   // non-dependency dispatch stall -- and on workloads/mlbench the dominant one is not
   // serialisation at all: d_hold is 99% rob_full, because a dot-product accumulator chain
   // blocks RETIREMENT rather than issue, so it never appears as a dependency stall and
   // ST_FPU correctly reads 0%. Without this bit that workload's real limiter is invisible
   // in the CPI stack.
   wire st_rob = hd_wait & ~cr_rob[0];
   // The mispredict DRAIN (plan item 5, 2026-09-05): a redirect resolved in M waits for the
   // ROB head (head_block) before it fires. These are the cycles P7's rename walk-back
   // would recover; on the stack they show what the drain costs before it is built.
   wire rd_wait = fr_v & ~cf_red_fire;   // a tracked branch restart still waiting to reach head
   // THE MEMORY BUCKETS (2026-09-17, program C0/B2): ST_MEM was one bit for everything the
   // backend did, and a backend rewrite would move one number. Each of these is a fact the
   // queues already compute, registered here like the rest; none is in any completion cone.
   wire mem_hitser    = lq_x_v & ~lq_x_take;                 // a ready load candidate the door did not take
   wire mem_ldinfl    = lsu_ld_busy;                          // a load access in flight (hit ~3 cycles; the rest is miss wait)
   wire mem_stdoor    = dmem_wen & ~dmem_wroom & ~dmem_waccept;   // a store at the door, not taken
   wire mem_alias_unk = sq_ld_block &  sq_l_block_unk_q[lq_x_idx];   // blocked: an older store's address is unknown
   wire mem_alias_ovl = sq_ld_block & ~sq_l_block_unk_q[lq_x_idx];   // blocked: a known older store overlaps
   wire mem_reord     = sq_ld_reorder;                        // a load issued past an uncommitted older store (the payoff)
   wire mem_wpkill    = lsu_pt_ld_done & lsu_pt_ld_kill;      // a wrong-path load's landing killed after its access ran
   wire mem_devwait   = lq_x_devwait;                         // a device load waiting to be the head
   // ---- TOP-DOWN (docs/PLAN-2026-09-20-topdown-counters.md, Variant A; SmolRV64-Spec 11.2) -------
   // Every cycle is exactly ONE of bad speculation, front-end, back-end or dispatching, by
   // construction: a redirect or a resolved-but-waiting restart is bad speculation; otherwise an
   // empty decode (fe_bub, which excludes st_m and d_valid) is front-end; otherwise M held or an
   // instruction present and not taken is back-end; what remains dispatched 0..3. td_k names the
   // cause, deepest first, in the order the testbench and the pipe views print it:
   //   0-2 dispatched 1-3, 21 dispatched 0 (a squashed take)   3 redirect   4 drain
   //   5 iMMU  6 no fetch bytes  7 no whole insn  8 queue empty  9 other front-end
   //   10 M on memory  11 M other  12 ROB full  13 wait on a load  14 wait on FP  15 scheduler full
   //   16 rename  17 SQ full  18 LQ full  19 serializing  20 held otherwise
   wire       td_bs  = redirect | rd_wait;
   wire       td_fe  = ~td_bs & fe_bub;
   wire       td_be  = ~td_bs & (st_m | hd_wait | (d_valid & ~d_take));   // M held, the queue head without credit, or a frozen IR
   wire [3:0] td_n4  = ones_nl(s_takev);
   wire [1:0] td_nd  = td_n4[1:0];
   reg  [4:0] td_k;
   always @* begin
      if (td_bs)      td_k = redirect ? 5'd3 : 5'd4;
      else if (td_fe) td_k = fe_mmu ? 5'd5 : fe_ic ? 5'd6 : fe_aln ? 5'd7 : fe_que ? 5'd8 : 5'd9;
      else if (td_be) td_k = st_m ? (m_mem_op ? 5'd10 : 5'd11)
                           : st_rob ? 5'd12 : dep_ld ? 5'd13 : dep_fp ? 5'd14 : st_iq ? 5'd15
                           : st_rn ? 5'd16 : st_sq ? 5'd17 : st_lq ? 5'd18 : st_srz ? 5'd19 : 5'd20;
      else            td_k = (td_nd == 2'd0) ? 5'd21 : {3'b0, td_nd} - 5'd1;
   end
   // the depth events, each a set of td_k values, so each is a subset of its parent
   wire td_be_mem = (td_k == 5'd10) | (td_k == 5'd13);                      // waiting on memory
   wire td_be_rob = (td_k == 5'd12);                                         // the ROB is full
   wire td_be_iq  = (td_k == 5'd15) | (td_k == 5'd17) | (td_k == 5'd18);    // a scheduler or queue is full
   wire td_fe_lat = (td_k == 5'd5)  | (td_k == 5'd6);                        // front-end latency: iMMU walk, no bytes
   wire [43:0] hpm_ev = {td_fe_lat, td_be_iq, td_be_rob, td_be_mem, td_be,
                         mem_devwait, mem_wpkill, mem_reord, mem_alias_ovl, mem_alias_unk,
                         mem_stdoor, mem_ldinfl, mem_hitser,
                         st_srz, st_lq, st_sq, st_rn, st_iq,
                         lsu_dtlb_walk_beg, lsu_dtlb_walking, rd_wait, st_rob, td_fe, td_bs,
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
   // `rc_v`, and smolrv64_rob's `head_done` write-forwards across every writeback port,
   // so retire sits downstream of every unit's completion in the cycle it happens:
   //   m_addr -> lsu_done -> rob w_hits -> retire -> retire_cnt
   //          -> hpm_inc's INSTRET arm -> 13 event muxes -> 13x 64-bit carry chain
   // and `u_csr/mhpmcounter[12]` came back as the worst family at NF=7 (-0.082, 32 levels,
   // 10x CARRY8). mhpmcounterN is instrumentation by the same argument as hpm_ev above --
   // read through a CSR many cycles later, and no software can observe which cycle an
   // event landed on -- so it takes the delayed copy and minstret keeps the live one.
   // Registering it here rather than in csr_file also keeps the src/ OoO core, which
   // shares that module, bit-identical: it passes its live count to both ports.
   // Two stages: the first has no reset or initial value, so synthesis retiming may pull it back
   // into the attribution cone (the top-down events read M's done, the redirect and the door).
   reg [43:0] hpm_ev_p, hpm_ev_q;
   reg [1:0]  hpm_disp_q;                 // instructions dispatched this cycle (DPATCH)
   reg [5:0]  hpm_lqocc_q, hpm_sqocc_q;   // queue occupancies, per cycle (MEM_LQOCC / MEM_SQOCC)
   reg [5:0]  hpm_ret_q;
   initial begin hpm_ev_q = 44'd0; hpm_ret_q = 6'd0; hpm_lqocc_q = 6'd0; hpm_sqocc_q = 6'd0; hpm_disp_q = 2'd0; end
   always @(posedge clk) begin
      hpm_ev_p  <= hpm_ev;
      hpm_ev_q  <= reset ? 44'd0 : hpm_ev_p;
      hpm_disp_q <= reset ? 2'd0 : td_nd;
      hpm_lqocc_q <= reset ? 6'd0 : {{(6-LQ_IB-1){1'b0}}, lq_occ};
      hpm_sqocc_q <= reset ? 6'd0 : {{(6-SQ_IB-1){1'b0}}, sq_occ};
      hpm_ret_q <= reset ? 6'd0 : {2'd0, ones_nl(retire)};
   end

   csr_file u_csr
     (.clk(clk), .reset(reset),
      .raddr(sy_addr), .rdata(csr_rdata),
      .redir_target(csr_redir_tgt), .redir_valid(csr_redir_v),
      .redir_is_trap(csr_redir_trap), .csr_illegal(csr_illegal),
      .o_satp(mmu_satp), .o_priv(mmu_priv), .o_dpriv(mmu_dpriv),
      .o_sum(mmu_sum), .o_mxr(mmu_mxr), .o_frm(csr_frm), .o_fs_off(fs_off),
      .fp_fflags_we(|ret_fflags), .fp_fflags(ret_fflags),
      // mstatus.FS -> Dirty when an op that changes FP state RETIRES, on ANY commit port: one
      // that wrote an f-register (arch 32..63: FP arith, the in-core FP writers, FP loads; not
      // FSW/FSD, which write memory) or whose flags reach fcsr. Linux saves a task's f-registers
      // on a context switch only when FS is Dirty. Commit-gated by construction: the commit
      // valids exclude a trapping op, so it never dirties FS.
      .fp_dirty_commit((|(retire & rc_f)) | (|ret_fflags)),
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
      .xtrap_v((xtrap_v & m_done_red) | sy_xfire), .xtrap_intr(1'b0),
      .xtrap_cause(sy_xt ? sy_xcause : xtrap_cause),
      .xtrap_epc(sy_xt ? sy_pc : m_pc), .xtrap_tval(sy_xt ? sy_xtval : xtrap_tval),
      // Both counts are the DELAYED copy, summed over the commit ports: minstret is exact because a CSR op completes only
      // in its second cycle at the ROB head (m_head_q), by which time every older retirement
      // has been counted; csr_file drops the CSR op's own retirement after a minstret write.
      .hw_ip(hw_ip), .mtime(mtime), .retire_cnt(hpm_ret_q),
      .hpm_retire_cnt(hpm_ret_q), .hpm_ev(hpm_ev_q), .hpm_lqocc(hpm_lqocc_q), .hpm_sqocc(hpm_sqocc_q), .hpm_disp(hpm_disp_q),
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
      .upd_valid(sy_fire & ~sy_xt & ~sy_fn), .upd_is_csr(sy_is_csr), .upd_func(sy_func),   // the SYSQ's flops (C3 step 3)
      .upd_addr(sy_addr),
      .upd_src(sy_src),
      .upd_pc(sy_pc));

   // ---- NON-BLOCKING LOADS ------------------------------------------------------------
   // M lets go of a plain load at DISPATCH instead of at data-return. Safe with no ROB walk
   // because smolrv64_lsu decides the fault before the access starts (see smolrv64_lsu.started): past
   // S_IDLE a load cannot fault, so nothing older can still trap once M has moved on.
   //
   // Exactly ONE load in flight, and that is not a simplification to revisit casually -- the
   // PRF has one write address (smolrv64_prf: one `wa`, three shard enables), so two completions
   // in a cycle have nowhere to go. Multiple outstanding loads is the step that has to solve
   // that, together with a load queue and D$ MSHRs.
   wire m_ld_nb  = m_mem_op & ~m_is_store & ~m_is_amo & ~m_is_cbo;  // plain load, rule C1
   // The 1-deep load scoreboard that stood here is GONE, replaced by smolrv64_lq. It tracked a
   // load from the cycle its ACCESS STARTED, which is why the ordering test had nowhere to
   // live but smolrv64_lsu's start gate, on the end of the translate path. The queue tracks it
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
   wire        ft_dst32 = fl_tag[FTAGW-1];
   wire        ft_rd_v  = fl_tag[FTAGW-2];
   wire [5:0]  ft_rd    = fl_tag[FTAGW-3 -: 6];
   wire [ROB_IDXB-1:0]  ft_rob = fl_tag[RN_PBITS +: ROB_IDXB];
   wire [RN_PBITS-1:0]  ft_prd = fl_tag[RN_PBITS-1:0];
   wire [63:0] fp_wval  = ft_dst32 ? {32'hffffffff, fl_data[31:0]} : fl_data;
   // No ~ld_land: FP has its own ROB completion port now, so a load landing in the same
   // cycle no longer displaces it and there is nothing to hold.
   wire        fp_land  = fp_complete;
   wire        fp_wb    = fp_land & ft_rd_v;
   always @(posedge clk) if (!reset) begin
      // The tag is the only thing naming the destination now, so a result that arrives
      // unowned would write a live register silently instead of being caught by fb_busy.
      if (fp_land & (ft_prd == {RN_PBITS{1'b0}}) & ft_rd_v)
         $fatal(1, "smolrv64_core: FP result claims rd_v with physreg 0");
   end

   // ---- completion ----
   wire m_unit_ok = ~m_valid             ? 1'b1
                 // `done` is now M's alone: it is fault | translate-only | an access this
                 // stage started, and every access smolrv64_lq or smolrv64_sq starts reports on
                 // pt_done instead (the own_pt latch in smolrv64_lsu). The ~sb_busy qualifier
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
   wire [63:0] m_unit_res = lsu_rd_val;
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
   reg  fr_br, fr_jalr;      // the tracked restart's kind, for the redirect counters
   reg [SEQW-1:0]     fr_seq;   // seqno of the oldest pending restart; younger restarts are ignored
   reg [ROB_IDXB-1:0] fr_rob;   // its ROB slot: the backend squash fires when this reaches head
   wire m_needs_head = m_mem_op & m_lsu_flt;
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
   assign m_done = m_done_raw & ~head_block & ~port_yield & ~sy_at_head & ~m_flt_pulse;
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
   integer sqi, xsk;
   initial for (sqi = 0; sqi < ROB_DEPTH; sqi = sqi + 1) xtq_kind[sqi] = SYK_NONE;
   always @(posedge clk) if (!reset) begin
      // (a) dispatch: a decode fault or an illegal instruction traps with what decode knows
      for (xsk = 0; xsk < IW; xsk = xsk + 1)
         if (s_takev[xsk]) begin
            xtq_kind[s_rob[xsk]]  <= (s_fault[xsk] | s_illegal[xsk]) ? SYK_XTRAP : SYK_NONE;
            xtq_cause[s_rob[xsk]] <= s_fault[xsk] ? s_fault_cause[xsk] : 4'd2;
            xtq_tval[s_rob[xsk]]  <= s_fault[xsk] ? {{(64-PCW){1'b0}}, s_fault_tval[xsk]} : 64'd0;
            xtq_pc[s_rob[xsk]]    <= s_pc[xsk];
         end
      // (c) a queue entry's translation fault loads the SYSQ (its PC was recorded at dispatch, so
      // the SYSQ's check below holds the queue's stored PC and seq to it)
      if (qf_in) begin
         xtq_kind[qfr_rob]  <= SYK_XTRAP;
         xtq_cause[qfr_rob] <= qfr_fc;
         xtq_tval[qfr_rob]  <= qfr_va;
      end
      // (b) the LSU reports a data fault for M's op
      if (m_flt_pulse) begin
         xtq_kind[m_rob_idx]  <= SYK_XTRAP;
         xtq_cause[m_rob_idx] <= lsu_fault_cause;
         xtq_tval[m_rob_idx]  <= lsu_fault_tval;
      end
      // flush arm last (rule I11): a redirect fires at the head, every live entry is younger
      if (redirect) for (sqi = 0; sqi < ROB_DEPTH; sqi = sqi + 1) xtq_kind[sqi] <= SYK_NONE;
   end
   always @(posedge clk) if (!reset) begin
      if (sy_xfire) begin
         if (xtq_kind[sy_rob] != SYK_XTRAP)
            $fatal(1, "smolrv64_core: the SYSQ traps at rob %0d (pc %h) but the shadow holds kind %0d",
                   sy_rob, sy_pc, xtq_kind[sy_rob]);
         if (xtq_cause[sy_rob] != sy_xcause || xtq_tval[sy_rob] != sy_xtval || xtq_pc[sy_rob] != sy_pc)
            $fatal(1, "smolrv64_core: the SYSQ's trap at rob %0d disagrees with the shadow (pc %h/%h cause %0d/%0d tval %h/%h)",
                   sy_rob, xtq_pc[sy_rob], sy_pc, xtq_cause[sy_rob], sy_xcause, xtq_tval[sy_rob], sy_xtval);
      end
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
   // the ROB's commit-kill reads M's done without the LSU's live arm: with a trap the fault is
   // latched, so the two are one value; m_done carries the dTLB lookup into retirement
   always @(posedge clk) if (!reset && m_valid && m_trap && (m_done != m_done_red))
      $fatal(1, "smolrv64_core: a trapping op's done differs without the LSU's live arm (pc %h)", m_pc);
   wire csr_red = xtrap_v;
   // THE REDIRECT DOES NOT CARRY THE LSU'S LIVE COMPLETION. A memory op redirects only as a
   // trap, and a trap is taken from the latched copy (m_unit_done_q); a branch, a system op
   // and fence.i never go through the LSU. So the redirect's "done" is m_done with the
   // memory arm of m_unit_ok removed -- logically the same signal on every cycle a redirect
   // can fire, asserted below, and structurally free of dTLB -> lsu_done -> m_unit_ok, which
   // was the head of the u_sq/v_reg -> fe/q_dat family (326 endpoints, 26 levels).
   wire m_unit_ok_nomem = ~m_valid            ? 1'b1
                        : m_mem_op            ? 1'b0
                        :                       1'b1;
   assign m_done_red = (m_unit_ok_nomem | m_unit_done_q) & ~head_block & ~port_yield & ~sy_at_head;
   // M's ORDERED redirect: CSR write, fence.i, or a trap (csr_red carries xtrap_v). Fires only
   // at the ROB head (m_done_red gates on ~head_block). Branches left M, so m_redirect is 0 here
   // now; the branch squash comes from the CTF pipe (cf_red_fire).
   wire m_red_fire = m_valid & m_done_red & csr_red;
   wire m_red_ref  = m_valid & m_done     & csr_red;
   // The branch squash: the tracked mispredict has reached the ROB head. The head is unique, so
   // m_red_fire and cf_red_fire are mutually exclusive.
   wire cf_red_fire = fr_v & (rob_head_idx == fr_rob);
   assign redirect = m_red_fire | sy_red | cf_red_fire;
   always @(posedge clk) if (!reset) begin
      if (m_red_fire != m_red_ref)
         $fatal(1, "smolrv64_core: M redirect from the non-memory done disagrees with m_done (%b vs %b)",
                m_red_fire, m_red_ref);
      if ((xtrap_v & m_done_red) != (xtrap_v & m_done))
         $fatal(1, "smolrv64_core: trap request from the non-memory done disagrees with m_done");
      if ((m_is_sys & m_done_red) != (m_is_sys & m_done))
         $fatal(1, "smolrv64_core: CSR update valid from the non-memory done disagrees with m_done");
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
   // snapshot (smolrv64_rename), so anything renamed before the squash is undone by it --
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
   // grew from ~2 instructions to 16, while mispredicts fell 37% (docs/SmolRV64-Spec.md).
   wire        cf_older = ~fr_v | older(cf_seq, fr_seq);              // the oldest restart wins
   assign fr_set    = cf_mis & cf_older & ~redirect;
   always @(posedge clk) begin
      if (reset)         fr_v <= 1'b0;
      else if (redirect) fr_v <= 1'b0;      // the squash consumes it
      else if (fr_set)   begin fr_v <= 1'b1; fr_seq <= cf_seq; fr_rob <= cf_rob; fr_br <= cf_is_branch; fr_jalr <= cf_is_jalr; end
   end
   // The tracked restart's CTI trains the predictor before its squash fires (rule D16): in its
   // resolve cycle, or for a jal/jalr once its link is written.
   reg  fr_trn;
   wire fr_trn_now = res_v & (tr_seq == fr_seq);
   initial fr_trn = 1'b0;
   always @(posedge clk) begin
      if (reset | redirect)          fr_trn <= 1'b0;
      else if (fr_set)               fr_trn <= res_v & (tr_seq == cf_seq);
      else if (fr_v & fr_trn_now)    fr_trn <= 1'b1;
      if (!reset && cf_red_fire && !(fr_trn | fr_trn_now))
         $fatal(1, "smolrv64_core: squash of rob %0d (seq %0d) fires but its CTI never trained the predictor",
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
   assign redirect_target  = (sy_fire & sy_fi) ? (sy_pc + 64'd4)           // fence.i: refetch after it
                           : (csr_red | sy_red) ? csr_redir_tgt
                           :           (m_pc + (m_rvc ? 64'd2 : 64'd4));
   assign redirect_is_trap = (m_red_fire & m_trap) | sy_trap;
   assign redirect_seq     = sy_red ? (sy_trap ? sy_seq : (sy_seq + 1'b1)) : m_trap ? m_seq : (m_seq + 1'b1);
   assign ifence           = sy_fire & sy_fi;

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
   // Up to three CTIs resolve in a cycle and the predictor trains one. The restart's own CTI
   // trains in its restart cycle (rule D16: before its squash). Every resolving CTI queues in
   // age order (TQN entries, one trains per cycle) unless the queue is full, when it is
   // dropped: training is a hint (tr_drop counts them). The enqueue reads registers only. At
   // the head, the pending restart's own CTI (it trained when it restarted) and anything
   // younger (the path the restart left) train nothing; the queue empties on a squash.
   localparam integer TQN = 8, TQB = 3;
   localparam integer RKB = $clog2(NL);          // a rank: 0..NL-1
   reg  [NL-1:0]  tr_ok;
   reg  [RKB-1:0] rk [0:NL-1];                  // each enqueuing lane's rank among this cycle's: the older ones
   integer tqi, tqj;
   always @* for (tqi = 0; tqi < NL; tqi = tqi + 1) begin
      tr_ok[tqi] = lr_v[tqi] & (lr_br[tqi] | lr_jmp[tqi]) & ~(fr_v & older(fr_seq, lr_seq[tqi]));
   end
   always @* for (tqi = 0; tqi < NL; tqi = tqi + 1) begin
      rk[tqi] = {RKB{1'b0}};
      for (tqj = 0; tqj < NL; tqj = tqj + 1)
         if (tqj != tqi) rk[tqi] = rk[tqi] + {{(RKB-1){1'b0}}, tr_ok[tqj] & older(lr_seq[tqj], lr_seq[tqi])};
   end
   reg  [TQB-1:0] tq_h, tq_t;
   reg  [TQB:0]   tq_n;
   reg  [PCW-1:0] tq_pc [0:TQN-1];
   reg  [PCW-1:0] tq_tgt [0:TQN-1];
   reg  [PDW-1:0] tq_pdet [0:TQN-1];
   reg  [SEQW-1:0] tq_seq [0:TQN-1];
   reg  [ROB_IDXB-1:0] tq_rob [0:TQN-1];
   reg  [TQN-1:0] tq_rvc, tq_cbr, tq_call, tq_ret, tq_taken;
   initial begin tq_h = 0; tq_t = 0; tq_n = 0; end
   wire [TQB:0] tq_free = TQN[TQB:0] - tq_n;
   reg  [NL-1:0] tq_w;
   always @* for (tqi = 0; tqi < NL; tqi = tqi + 1)
      tq_w[tqi] = tr_ok[tqi] & ({{(TQB+1-RKB){1'b0}}, rk[tqi]} < tq_free);
   wire         tq_out = ~fr_set & (tq_n != 0);               // the head trains (or is discarded) this cycle
   wire         tq_dead = fr_v & ~older(tq_seq[tq_h], fr_seq); // ...discarded: the pending restart or past it
   // each lane's CTI is a call (writes x1/x5) or a return (a jalr reading x1/x5, writing neither)
   reg  [NL-1:0] l_lrd, l_lrs;
   always @* for (tqi = 0; tqi < NL; tqi = tqi + 1) begin
      l_lrd[tqi] = lr_rdv[tqi] & ((lr_rd[tqi] == 6'd1) | (lr_rd[tqi] == 6'd5));
      l_lrs[tqi] = (lr_rs1[tqi] == 6'd1) | (lr_rs1[tqi] == 6'd5);
   end
   wire [NL-1:0] l_call = lr_jmp & l_lrd, l_ret = lr_jalr & l_lrs & ~l_lrd;
   integer tk;
   wire [3:0] tq_wn = ones_nl(tq_w);                 // the lanes' CTIs enqueuing this cycle
   always @(posedge clk) begin
      if (reset | redirect) begin
         tq_h <= 0;  tq_t <= 0;  tq_n <= 0;
      end else begin
         for (tk = 0; tk < NL; tk = tk + 1) if (tq_w[tk]) begin : enq
            reg [TQB-1:0] a;
            a = tq_t + {{(TQB-RKB){1'b0}}, rk[tk]};
            tq_pc[a] <= lr_pc[tk];  tq_tgt[a] <= lr_ttgt[tk];  tq_pdet[a] <= lr_pdet[tk];
            tq_seq[a] <= lr_seq[tk];  tq_rob[a] <= lr_rob[tk];  tq_rvc[a] <= lr_rvc[tk];
            tq_cbr[a] <= lr_br[tk];  tq_call[a] <= l_call[tk];  tq_ret[a] <= l_ret[tk];
            tq_taken[a] <= lr_taken[tk];
         end
         tq_t <= tq_t + tq_wn[TQB-1:0];
         tq_h <= tq_h + {{(TQB-1){1'b0}}, tq_out};
         tq_n <= tq_n + tq_wn[TQB:0] - {{TQB{1'b0}}, tq_out};
      end
   end
   wire [3:0]   tq_dn = ones_nl(tr_ok & ~tq_w);
   wire [RKB:0] tr_drop = tq_dn[RKB:0];
   assign tr_seq = fr_set ? lr_seq[wl] : tq_seq[tq_h];
   wire [ROB_IDXB-1:0] tr_rob = fr_set ? lr_rob[wl] : tq_rob[tq_h];   // (the bench marks the trained CTI by it)
   assign res_v     = fr_set ? (lr_br[wl] | lr_jmp[wl]) : (tq_out & ~tq_dead);
   assign res_cbr   = fr_set ? lr_br[wl]    : tq_cbr[tq_h];
   assign res_call  = fr_set ? l_call[wl]  : tq_call[tq_h];
   assign res_ret   = fr_set ? l_ret[wl]   : tq_ret[tq_h];
   assign res_taken = fr_set ? lr_taken[wl] : tq_taken[tq_h];
   assign res_tgt   = fr_set ? lr_ttgt[wl]  : tq_tgt[tq_h];
   wire [PCW-1:0] tr_pc   = fr_set ? lr_pc[wl]   : tq_pc[tq_h];
   wire [PDW-1:0] tr_pdet = fr_set ? lr_pdet[wl] : tq_pdet[tq_h];
   wire           tr_rvc  = fr_set ? lr_rvc[wl]  : tq_rvc[tq_h];
   always @(posedge clk) if (!reset & !redirect) begin
      if (tq_n > TQN[TQB:0]) $fatal(1, "smolrv64_core: the training queue holds %0d of %0d", tq_n, TQN);
   end

   // ---- the fetch stream's exposure to a wrong path: weak conditionals in flight ----
   // A conditional dispatched on a WEAK direction (its effective counter, the corrector's when it
   // hit, is 01 or 10) is counted until it leaves the CTF stage (cf_done: once per CTI, wrong-path
   // ones included). Every counted branch is older than anything fetch asks for, so it resolves
   // whatever fetch does; the backend squash kills every younger one with nothing older left, so it
   // clears the count exactly. While one is in flight the iMMU starts no walk: a page reached past
   // a coin-flip branch is not worth a walk until the branch says so. A pending restart (fr_v)
   // means fetch has already left the wrong path, so the branches younger than it hold nothing.
   localparam integer PD_YHIT = PD_GHR - 1;                  // the corrector-hit bit
   localparam integer PD_YCTR = PD_GHR - 3;                  // the corrector's counter
   function weak_cond(input [PDW-1:0] pd);
      reg [1:0] c;
      begin
         c = pd[PD_YHIT] ? pd[PD_YCTR +: 2] : pd[1:0];
         weak_cond = pd[DCR_HIT] & (c[1] ^ c[0]);
      end
   endfunction
   reg  [1:0] wk_d;                                // weak conditionals dispatching this cycle
   integer wdk;
   always @* begin
      wk_d = 2'd0;
      for (wdk = 0; wdk < IW; wdk = wdk + 1) wk_d = wk_d + {1'b0, s_takev[wdk] & s_is_branch[wdk] & weak_cond(s_pdet[wdk])};
   end
   reg  [1:0] wk_r;                                // weak conditionals resolving this cycle
   integer wkr;
   always @* begin
      wk_r = 2'd0;
      for (wkr = 0; wkr < NL; wkr = wkr + 1) wk_r = wk_r + {1'b0, lr_v[wkr] & lr_br[wkr] & weak_cond(lr_pdet[wkr])};
   end
   reg  [ROB_IDXB:0] wk_n;   initial wk_n = 0;
   always @(posedge clk) begin
      if (reset | redirect) wk_n <= 0;
      else                  wk_n <= wk_n + wk_d - {{(ROB_IDXB-1){1'b0}}, wk_r};
      if (!reset && !redirect && ({{(ROB_IDXB-1){1'b0}}, wk_r} > wk_n + {{(ROB_IDXB-1){1'b0}}, wk_d}))
         $fatal(1, "smolrv64_core: a weak conditional resolved with none counted in flight");
   end
   wire walk_hold = (wk_n != 0) & ~fr_v;
   // ...and the I$ holds a miss's line read on the same terms: the pending restart that ends the
   // hold reaches the fetch ring in the cycle the hold drops, and the I$ takes the cancel first.
   assign imem_hold = walk_hold;

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
   assign wb_ld = lsu_rd_val;              // a landing load's or M's AMO's (the SYSQ's goes to lane A)
   // the FE stream: the F stage's landing, else the MD stage's divide
   assign wb_fe = fp_wb         ? fp_wval
                :                 md_res_q;       // a divide's result
   // The LD stream has two writers, M's own completion and a load landing after M has moved
   // on; they never coincide (m_done is forced low on ld_land).
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
                     : m_mem_op            ? lsu_done_acc
                     :                       1'b1;
   wire m_done_wb = (m_unit_ok_wb | m_unit_done_q) & ~head_block & ~port_yield & ~sy_at_head;
   wire m_trap_wb = m_mem_op & m_lsu_flt;
   wire m_wb  = m_valid & m_done_wb & m_rd_v & ~m_trap_wb & ~m_ld_nb;
   wire m_wb_ref = m_valid & m_done & m_rd_v & ~m_trap & ~m_ld_nb;
   always @(posedge clk) if (!reset && (m_wb != m_wb_ref))
      $fatal(1, "smolrv64_core: writeback valid from the access-only done disagrees with m_done (%b vs %b)",
             m_wb, m_wb_ref);
   wire ld_wb = ld_land & lq_l_rd_v;

   // PER-SHARD WRITE PORTS. Each lane's shard is written by its lane's write register alone,
   // which takes, in order, its landing buffer's head, the ALU at issue and the multiply in its
   // reserved slot; the FP slices by the LD and FE streams.
   // A lane's wake and write-register enables do not read the live redirect (rule I11): a redirect
   // fires at the ROB head, so the op executing in a lane then is younger and flushed everywhere
   // (each flush arm is ordered last); its write lands at T+1 in a register the rollback frees,
   // before any new producer of it can dispatch. A multiply writes in its reserved slot.
   // THE LD STREAM: a landing load or M's AMO result, one a cycle. An FP load's lands on the LD
   // port in its FP slice; an integer load's or AMO's in its lane (ldw_int, the lane by its
   // register's shard).
   wire ld_any = m_wb | ld_wb;
   wire [2:0] ldw_sh = wa_ld[RN_PBITS-1:RN_IDXB];
   assign ldw_int = ld_any & ~ldw_sh[2];
   wire we_ld = ld_any & ~ldw_int;                 // ...an FP slice's write
   // THE FE STREAM: the F stage's landing or the MD stage's result, one a cycle (the divide yields
   // to the FPU). An f-register's lands on the FE port in its FP slice; an integer one in its lane.
   wire fe_any = fp_wb | md_wr;
   wire [2:0] few_sh = wa_fe[RN_PBITS-1:RN_IDXB];
   assign few_int = fe_any & ~few_sh[2];
   wire we_fe = fe_any & ~few_int;
   // Each stream broadcasts on its own port (wake, pending clear, the store queue's snoop) what
   // it writes this cycle: an FP slice's register, or an integer one straight through a lane's
   // write register. A landing that waits in a lane's buffer broadcasts on the lane's port when
   // it drains.
   wire wk_ld = we_ld | (|by0);
   wire wk_fe = we_fe | (|by2);
   wire [RN_PBITS-1:0] wa_ld = ld_wb ? lq_l_prd : m_prd;
   wire [ROB_IDXB-1:0] ldw_rob = ld_wb ? lq_l_rob : m_rob_idx;

   // ---- THE LANDING BUFFERS (lanes step 5.2d-c) ----
   // An integer load's or AMO's result, and the SYSQ's (a CSR read, a system op's completion),
   // lands in its lane: the lane's write register, its wake and pending clear, the store queue's
   // snoop and the lane's ROB completion port -- in a cycle the lane leaves both free (nothing
   // executing that completes at issue, no multiply in its slot). A load's or AMO's goes straight
   // through when the lane is free and nothing waits; otherwise it waits here, and a waiting entry
   // holds the lane's select (unit_busy), so a free cycle comes within two. It completes when it
   // drains, never before, so a waiting entry belongs to an op that has not retired and a redirect
   // empties the buffers. The SYSQ never waits: a system op goes alone, so it is lane A's, and it
   // fires only when lane A is free and nothing waits there (lane A holds its select while a
   // system op is the ROB head), so its write and completion are in its fire cycle, as its
   // redirect needs. Load-hit speculation (waking the consumers from the D$ lookup) is a later step.
   integer             lbi;
   initial for (lbi = 0; lbi < NL; lbi = lbi + 1) begin lb_h[lbi] = 0; lb_t[lbi] = 0; lb_n[lbi] = 0; end
   // the LD and FE streams' pushes into their lanes; the SYSQ's goes through lane A in its fire cycle.
   // M's own result (an AMO, LR or SC: the LSU's live completion) always waits a cycle here, so
   // only a landing load can go straight through (pl): the lane's write address, ROB index and
   // wake never select on the LSU's completion, only its buffer write does.
   wire          pl_v = ld_wb & ~ldw_sh[2];
   wire [ROB_IDXB-1:0] few_rob = fp_wb ? ft_rob : md_rob;
   wire          sp_v = sy_wr | sy_done;
   wire          sp_f = sy_fire;                 // the SYSQ takes lane A's slot (registers and the ROB head)
   // each lane's: the LD stream's push (p0), a landing load's (pl), the SYSQ's (p1, lane A's) and
   // the FE stream's (p2); the lane's write and ROB ports are free (nothing executing that completes
   // at issue, no multiply in its reserved slot: registers only); an entry waits
   wire [NL-1:0] p0, pl, p1, p2, lb_free, lb_any;
   genvar glk;
   generate for (glk = 0; glk < NL; glk = glk + 1) begin: g_lk
      localparam [2:0] K = glk;
      assign p0[glk] = ldw_int & (ldw_sh == K);
      assign pl[glk] = pl_v & (ldw_sh == K);
      assign p2[glk] = few_int & (few_sh == K);
      assign p1[glk] = (glk == 0) & sp_f;
      assign lb_free[glk] = ~(l_v[glk] & ~l_late[glk]) & ~l_m2[glk];
      assign lb_any[glk] = lb_n[glk] != 0;
   end endgenerate
   assign sy_lane_ok = lb_free[0] & ~lb_any[0];
   // each lane drains, in order, its oldest waiting entry, the SYSQ's, a landing load's, the FE's
   wire [NL-1:0] lb_drain = lb_free & (lb_any | pl | p1 | p2);
   // A stream's goes straight through when the lane is free, nothing waits and lane A is not held
   // for a system op at the ROB head (the SYSQ's fire is not in the streams' wake); else it waits.
   wire [NL-1:0] rsv = {{(NL-1){1'b0}}, sy_at_head};
   wire [NL-1:0] by0 = lb_drain & ~lb_any & ~rsv & pl;
   wire [NL-1:0] by2 = lb_drain & ~lb_any & ~rsv & ~pl & p2;
   wire [NL-1:0] en0 = p0 & ~by0;
   wire [NL-1:0] en2 = p2 & ~by2;
   // the drained entry of each lane, {wr, cmp, prd, dat, rob} (a wire per lane: rule F4)
   wire [2+RN_PBITS+64+ROB_IDXB-1:0] lp [0:NL-1];
   wire [LBB-1:0]                    lb_t2 [0:NL-1];         // the FE stream's slot, after the LD's
   genvar glp;
   generate
      for (glp = 0; glp < NL; glp = glp + 1) begin : g_lp
         wire [LBB-1:0] hh = lb_h[glp];
         assign lp[glp] = lb_any[glp] ? {1'b1, 1'b1, lb_prd[glp*LBN + hh], lb_dat[glp*LBN + hh], lb_rob[glp*LBN + hh]}
                        :               {sy_wr, sy_done, sy_prd, csr_rdata, sy_rob};
         assign lb_t2[glp] = lb_t[glp] + {{(LBB-1){1'b0}}, en0[glp]};
      end
   endgenerate
   always @(posedge clk) begin
      for (lbi = 0; lbi < NL; lbi = lbi + 1) begin
         if (en0[lbi]) begin
            lb_prd[lbi*LBN + lb_t[lbi]] <= wa_ld;  lb_dat[lbi*LBN + lb_t[lbi]] <= wb_ld;
            lb_rob[lbi*LBN + lb_t[lbi]] <= ldw_rob;  lb_fe[lbi*LBN + lb_t[lbi]] <= 1'b0;
         end
         if (en2[lbi]) begin
            lb_prd[lbi*LBN + lb_t2[lbi]] <= wa_fe;  lb_dat[lbi*LBN + lb_t2[lbi]] <= wb_fe;
            lb_rob[lbi*LBN + lb_t2[lbi]] <= few_rob;  lb_fe[lbi*LBN + lb_t2[lbi]] <= 1'b1;
         end
         lb_t[lbi] <= lb_t[lbi] + {{(LBB-1){1'b0}}, en0[lbi]} + {{(LBB-1){1'b0}}, en2[lbi]};
         if (lb_drain[lbi] & lb_any[lbi]) lb_h[lbi] <= lb_h[lbi] + 1'b1;
         lb_n[lbi] <= lb_n[lbi] + {{LBB{1'b0}}, en0[lbi]} + {{LBB{1'b0}}, en2[lbi]}
                                - {{LBB{1'b0}}, lb_drain[lbi] & lb_any[lbi]};
         if (reset | redirect) begin lb_h[lbi] <= 0; lb_t[lbi] <= 0; lb_n[lbi] <= 0; end   // flush last (rule I11)
      end
   end
   always @(posedge clk) if (!reset) begin
      for (lbi = 0; lbi < NL; lbi = lbi + 1)
         if ({1'b0, lb_n[lbi]} + en0[lbi] + en2[lbi] - (lb_drain[lbi] & lb_any[lbi]) > LBN)
            $fatal(1, "smolrv64_core: lane %0d's landing buffer overflows", lbi);
      if (sp_v & ~lb_drain[0])
         $fatal(1, "smolrv64_core: the SYSQ fired without lane A's slot");
      if (sy_wr & (sy_prd[RN_PBITS-1:RN_IDXB] != SH_IE))
         $fatal(1, "smolrv64_core: the SYSQ wrote pr=%h, not lane A's shard", sy_prd);
   end
   wire [RN_PBITS-1:0] wa_fe = fp_wb ? ft_prd : md_prd;
   always @(posedge clk) if (!reset) begin
      if (m_wb & ld_wb)
         $fatal(1, "smolrv64_core: LD shard written by both M and a landing load");
      // an integer result lands in its lane, an f-register's in its FP slice, every one in a shard
      // this width uses: an unused shard has no bank, and a result routed there would be dropped
      if ((ld_any & ~sh_used(ldw_sh)) | (fe_any & ~sh_used(few_sh)))
         $fatal(1, "smolrv64_core: a result routed to shard %0d, which this width does not use (pc %h)",
                ld_any ? ldw_sh : few_sh, m_pc);
   end

   // The architectural shadow is written AT COMMIT, in order. It has no rename, so it cannot
   // model out-of-order writeback: a younger instruction writes x13, then an older load lands
   // and clobbers the same architectural location. Driving it from the ROB head keeps it a
   // valid architectural model, which is what tb_smolrv64_riscv's trace reads it as.

   // RETIRE IS THE ROB HEAD, not the M stage. Not merely for the cosim: it drives minstret
   // through retire_cnt and csr_file's fp_dirty_commit, both architectural, and both of
   // which must count an instruction when it COMMITS rather than when it happens to finish.
   // With M still blocking the two coincide, which is what makes this step checkable.
   assign retire = rc_v & ~rc_noret;

   // FP EXCEPTION FLAGS COMMIT WITH THEIR OP. An FP op completes out of order, possibly on a path
   // that a mispredicted branch older than it squashes later, so its flags wait in its ROB slot
   // -- cleared when the slot is allocated, written when the result lands -- and reach fcsr only
   // when the op retires. Completion and commit can be the same cycle (the ROB's w_hits), hence
   // the bypass from the landing result.
   reg  [4:0] rob_ff [0:ROB_DEPTH-1];
   integer    ffk;
   always @(posedge clk) begin
      for (ffk = 0; ffk < IW; ffk = ffk + 1) if (s_takev[ffk]) rob_ff[s_rob[ffk]] <= 5'd0;
      if (fp_complete) rob_ff[ft_rob]     <= fl_fflags;
   end
   reg  [4:0] ret_fflags;
   integer    ffh;
   always @* begin
      ret_fflags = 5'd0;
      for (ffh = 0; ffh < IW; ffh = ffh + 1)
         if (rc_v[ffh])
            ret_fflags = ret_fflags | ((fp_complete & (ft_rob == rc_idx[ffh])) ? fl_fflags : rob_ff[rc_idx[ffh]]);
   end

   // retire_pc/retire_insn are verification payload -- tb_smolrv64_riscv traces them and
   // rv_soc_top leaves both unconnected -- so they come from a simulation-only side array
   // rather than widening the ROB by 96 bits an entry. docs/Area-Efficient-Scalar-OoO.md 2:
   // the reorder buffer holds status, not data.
`ifndef SYNTHESIS
   reg [PCW-1:0] cs_pc   [0:ROB_DEPTH-1];
   reg [31:0]    cs_insn [0:ROB_DEPTH-1];
   integer       cpk;
   always @(posedge clk) begin
      for (cpk = 0; cpk < IW; cpk = cpk + 1)
         if (s_takev[cpk]) begin cs_pc[s_rob[cpk]] <= s_pc[cpk];  cs_insn[s_rob[cpk]] <= s_insn[cpk];  end
   end

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
   integer csk;
   always @(posedge clk) if (!reset) begin
      // On the completion PULSE, not on m_done: lsu_cos_* are only this op's while the LSU
      // still holds it, and m_done can now assert cycles later off the sticky latch.
      if (m_valid && m_unit_ok && !m_unit_done_q && !m_ld_nb) begin
         cs_val[m_rob_idx]   <= m_wb_val;
      end
      // A CSR READ'S VALUE IS THE ONE AT ITS WRITE, not at the unit-ok pulse: the op
      // completes in its second cycle at head (m_head_q) and reads csr_rdata live then, so
      // `time`/`cycle` are one tick past the pulse's value. Ordered after the arm above so
      // it wins. (Found by the 300 M cosim: rdtime retired as t, computed with t+1.)
      if (sy_wr) cs_val[sy_rob] <= csr_rdata;
      if (m_valid && m_unit_ok && !m_unit_done_q && !m_ld_nb) begin
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
      for (csk = 0; csk < NL; csk = csk + 1) begin
         // a lane's multiply in its reserved slot, and an op completed at issue (never saw M)
         if (l_m2[csk]) begin cs_val[l_mrob2[csk]] <= l_mres[csk]; cs_mkind[l_mrob2[csk]] <= 2'd0; cs_mpa[l_mrob2[csk]] <= 56'd0; end
         if (l_iss[csk] & ~l_late[csk]) begin
            cs_val[l_rob[csk]] <= l_res[csk];  cs_mkind[l_rob[csk]] <= 2'd0;  cs_mpa[l_rob[csk]] <= 56'd0;
         end
      end
   end
   // ...and the same write-forward the ROB's head_done needs, for the same reason: a slot can
   // be captured in the very cycle it commits, so the array read returns pre-edge contents.
   // Missing it reported rd=0 for the second instruction of the boot. cs_hl: a lane's op
   // completing at issue commit port k's entry.
   reg  [IW-1:0] cs_hl;
   reg  [63:0]   cs_hl_v [0:IW-1];
   integer       chi, chk;
   always @* for (chi = 0; chi < IW; chi = chi + 1) begin
      cs_hl[chi] = 1'b0;  cs_hl_v[chi] = 64'd0;
      for (chk = NL - 1; chk >= 0; chk = chk - 1)
         if (l_iss[chk] & (l_rob[chk] == rc_idx[chi])) begin
            cs_hl[chi] = 1'b1;  cs_hl_v[chi] = l_res[chk];
         end
   end
   wire [63:0] cs_val_h   [0:IW-1];
   wire [1:0]  cs_mkind_h [0:IW-1];
   wire [55:0] cs_mpa_h   [0:IW-1];
   wire [63:0] cs_mdata_h [0:IW-1];
   wire [3:0]  cs_msz_h   [0:IW-1];
   generate for (gs = 0; gs < IW; gs = gs + 1) begin: ch
      wire [ROB_IDXB-1:0] ix = rc_idx[gs];
      wire hit_m   = m_valid & m_unit_ok & ~m_unit_done_q & ~m_ld_nb & (m_rob_idx == ix);
      // A CSR op completes (and retires, by the ROB's same-cycle bypass) in its SECOND cycle
      // at head, reading csr_rdata live then -- after hit_m's pulse and before the capture
      // above lands. `time`/`cycle` differ by a tick between the two: the head takes it live.
      wire hit_csr = sy_wr & (sy_rob == ix);
      wire hit_ld  = ld_land & (lq_l_rob == ix);
      wire hit_sq  = sq_k_take & (sq_kc_rob == ix);
      wire hit_fp  = fp_land & (ft_rob == ix);
      wire hit_md  = md_wb & (md_rob == ix);
      assign cs_val_h[gs]   = hit_csr ? csr_rdata     // the CSR read's value at its write
                            : hit_sq ? 64'd0          // a store writes no register
                            : hit_ld ? lsu_rd_val
                            : cs_hl[gs] ? cs_hl_v[gs]
                            : hit_md ? md_res_q : hit_fp ? fp_wval
                            : hit_m  ? m_wb_val : cs_val[ix];
      assign cs_mkind_h[gs] = hit_sq ? 2'd2 : hit_ld ? 2'd1
                            : (cs_hl[gs] | hit_md | hit_fp) ? 2'd0
                            : hit_m  ? (m_mem_op ? lsu_cos_kind : 2'd0) : cs_mkind[ix];
      assign cs_mpa_h[gs]   = hit_sq ? sq_kc_addr : hit_ld ? lq_l_pa
                            : (cs_hl[gs] | hit_md | hit_fp) ? 56'd0
                            : hit_m  ? lsu_cos_pa : cs_mpa[ix];
      assign cs_mdata_h[gs] = hit_sq ? sq_kc_data : hit_m ? lsu_cos_data : cs_mdata[ix];
      assign cs_msz_h[gs]   = hit_sq ? {2'b0, sq_kc_size} : hit_m ? lsu_cos_size : cs_msz[ix];
      assign retire_pc[gs*PCW +: PCW]  = cs_pc[ix];
      assign retire_insn[gs*32 +: 32]  = cs_insn[ix];
      assign rf_we[gs]                 = rc_v[gs] & rc_rdv[gs];
      assign rf_wa[gs*6 +: 6]          = rc_rd[gs*6 +: 6];
      assign rf_wd[gs*64 +: 64]        = cs_val_h[gs];
   end endgenerate
`else
   assign retire_pc   = {(IW*PCW){1'b0}};
   assign retire_insn = {(IW*32){1'b0}};
   assign rf_we = {IW{1'b0}};  assign rf_wa = {(IW*6){1'b0}};  assign rf_wd = {(IW*64){1'b0}};
`endif

   // Nothing may sit in X while a serializing op is in M. This is what `ser_block`
   // exists to guarantee, and it is what makes the CSR result unbypassable above.
   always @(posedge clk)
     if (!reset && (|s_takev) && m_valid && m_is_serialize)
       $fatal(1, "smolrv64_core: dispatched behind a serializing op in M (pc=%h)", m_pc);

   // ---- interrupt injection: a solo SYSTEM pseudo-op that traps in M ----
   // FMAX: REGISTERED, for the same reason redirect_q is (see the note above the
   // redirect_q declaration). irq_inject is a select in fetch's npc mux (fetch.v:168),
   // and npc is the only combinational input to the BTB read register
   // (smolrv64_predictor.v:250) -- so while this was a wire it was the ONE path by which the
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
       $fatal(1, "smolrv64_core: interrupt injection presented while one is already in flight");

   // ...and fetch may never consume two. This is the invariant 49eecf48 broke; it is
   // checked directly now instead of being implied by a handshake that stopped holding.
   reg irq_taken_q;
   initial irq_taken_q = 1'b0;
   always @(posedge clk) begin
      irq_taken_q <= reset ? 1'b0 : irq_taken;
      if (!reset && irq_taken && irq_taken_q)
        $fatal(1, "smolrv64_core: interrupt pseudo-op consumed twice for one interrupt");
   end

`ifdef SMOLRV64_COSIM
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
   reg [1:0]  e_mkind [0:IW-1];   reg [55:0] e_mpa [0:IW-1];      // cosim memory-effect capture
   reg [63:0] e_mdata [0:IW-1];   reg [3:0]  e_msz [0:IW-1];
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
   reg        e_trap;
   reg [63:0] e_cause, e_tval;
   reg [1:0]  e_prv;
   reg [IW-1:0] e_v;                     // commit port k's record, emitted a cycle later
   reg [63:0] e_pc [0:IW-1], e_val [0:IW-1];
   reg [31:0] e_insn [0:IW-1];
   reg [1:0]  e_rk [0:IW-1];
   reg [4:0]  e_ri [0:IW-1];
   initial    e_v = {IW{1'b0}};
   integer    ek;

   // A trap and a retire remain mutually exclusive, but they are no longer both "an M cycle":
   // the trap is M's (and fires only when M is the ROB head), the retire is the head's.
   always @(posedge clk) begin
      e_v <= {IW{1'b0}};
      if (!reset) begin
         if (cot_take) begin                     // M's trap, or the SYSQ's (C3 step 3)
            e_v[0] <= 1'b1;  e_trap <= 1'b1;
            e_pc[0] <= sy_fire ? sy_pc : m_pc;
            // a queue entry's fault reaches the SYSQ without its instruction: it is the ROB head's
            e_insn[0] <= (cot_intr | cot_ifault) ? 32'd0 : (sy_fire & sy_qf) ? cs_insn[sy_rob]
                       : sy_fire ? sy_insn : m_insn;
            e_rk[0] <= 2'd0;  e_ri[0] <= 5'd0;  e_val[0] <= 64'd0;
            e_cause <= cot_cause;  e_tval <= cot_tval;
            e_prv <= u_csr.priv;                  // privilege BEFORE the trap
            e_mkind[0] <= 2'd0;  e_mpa[0] <= 56'd0;     // a trap performed no data access
            e_mdata[0] <= 64'd0; e_msz[0] <= 4'hF;
         end else if (retire[0]) begin
            e_trap <= 1'b0;  e_cause <= 64'd0;  e_tval <= 64'd0;
            e_prv <= u_csr.priv;
         end
         // Destination class and memory effect come from the COMMITTING entry: captured when
         // it completed, replayed when it commits.
         if (!cot_take)
            for (ek = 0; ek < IW; ek = ek + 1)
               if (retire[ek]) begin
                  e_v[ek] <= 1'b1;
                  e_pc[ek] <= cs_pc[rc_idx[ek]];  e_insn[ek] <= cs_insn[rc_idx[ek]];
                  e_rk[ek] <= ~rc_rdv[ek] ? 2'd0 : rc_rd[ek*6+5] ? 2'd2 : 2'd1;
                  e_ri[ek] <= rc_rd[ek*6 +: 5];  e_val[ek] <= cs_val_h[ek];
                  e_mkind[ek] <= cs_mkind_h[ek];  e_mpa[ek] <= cs_mpa_h[ek];
                  e_mdata[ek] <= cs_mdata_h[ek];  e_msz[ek] <= cs_msz_h[ek];
               end
      end
      // emit one cycle later, so this instruction's own CSR writes have landed
      for (ek = 0; ek < IW; ek = ek + 1)
         if (e_v[ek])
            probe_retire(e_pc[ek], e_insn[ek], {6'd0, e_rk[ek]},
                         (e_rk[ek] == 2'd0) ? 8'd0 : {3'd0, e_ri[ek]},
                         {6'd0, e_prv}, {7'd0, e_trap}, e_val[ek], e_cause, e_tval,
                         64'd0, {64{1'b1}}, `VA_UNPACK40(u_csr.mepc), 8'd0,
                         {6'd0, e_mkind[ek]}, {8'd0, e_mpa[ek]}, e_mdata[ek], {4'd0, e_msz[ek]});
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
   // An instruction that traps at dispatch serialises like a system op: it fires from the SYSQ,
   // which holds one op and fires it as the ROB head.
   // A CSR OP DOES NOT DRAIN. It waits in the SYSQ for the ROB head and fires there in program
   // order, while younger work dispatches and runs behind it: a write that changes what younger
   // ops already used (FS, the data-translation bits, satp, frm, the envcfg/stateen enables)
   // refetches them through csr_file's redirect, and anything else (sstatus.SIE above all, the
   // kernel's irq save and restore) changes nothing a younger op computed. One CSR op is in
   // flight at a time (csr_infl): the SYSQ holds one, and the F/CTF/MD/SYS queue reorders, so a
   // second could take the SYSQ before an older one and never reach the head.
   wire d_csr_op = d_gc[GC_CSR];
   wire d_ser    = d_gc[GC_SER];
   always @(posedge clk)
      if (reset | redirect)      ser_inflight <= 1'b0;
      else if (rn_valid & d_ser) ser_inflight <= 1'b1;
      else if (drained)          ser_inflight <= 1'b0;
   reg  csr_infl;
   initial csr_infl = 1'b0;
   always @(posedge clk)
      if (reset | redirect)        csr_infl <= 1'b0;
      else if (rn_valid & d_csr_op) csr_infl <= 1'b1;
      else if (sy_fire & sy_is_csr) csr_infl <= 1'b0;
   wire ser_block = ser_inflight | (d_valid & d_ser & ~drained) | (d_valid & (d_csr_op | d_ser) & csr_infl);

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
   // smolrv64_pending's business now, and WAITING is the scheduler's -- which is exactly what
   // stops a waiting consumer from blocking everything behind it.
   // ...and two resources that could not run out while only ~2 instructions were in flight.
   // rn_stall used to be a $fatal on exactly this reasoning; with a ROB behind a waiting load
   // it is a legitimate condition and has to be back-pressure instead.
   // src_pend IS GONE. Dispatch no longer waits for an instruction's operands -- that wait
   // moves into the scheduler, which is the entire point. What still blocks dispatch is
   // structural only: no ROB slot, no scheduler entry, or a rename shard run dry.
   // Back-pressure at the FRONTEND, never at issue: a full store buffer holds dispatch,
   // which costs nothing at the head of the machine and keeps unit state out of the
   // scheduler's select (docs/SmolRV64-Spec.md 15).
   // THE CREDITS (lanes step 5.3c). Nothing after the decoupling queue holds: the queue head pops
   // an instruction only when everything it allocates has room for it, counted against what is
   // already between the head and there -- the IR group (it dispatches this cycle) and, for a
   // scheduler, its dispatch-stage register. Every input is a flop or a sum over flops; this
   // cycle's frees are not credited. Each group allocates at most one entry in each scheduler,
   // the LQ, the SQ and each rename shard, which is what the per-resource checks rely on (and
   // LOWAT=4 covers two groups per shard, so `low` is the rename credit as it stands).
   wire [NL-1:0] ir_i = s_v & s_cls_i;              // the IR's group holds lane k's op
   wire       ir_l  = |(s_v & s_cls_l);
   wire       ir_f  = |(s_v & s_cls_fc);
   wire       ir_ld = |(s_v & s_ld_nb);
   wire       ir_st = |(s_v & s_st_nb);
   wire [3:0] ir_n  = ones_nl(s_v);
   wire [ROB_IDXB+1:0] rob_inflt = {1'b0, rob_occ} + {{(ROB_IDXB-2){1'b0}}, ir_n};
   reg  [NL-1:0] cr_i;                                 // ...and lane k has room for one more
   integer crk;
   always @* for (crk = 0; crk < NL; crk = crk + 1)
      cr_i[crk] = {1'b0, l_free[crk]} > ({{(IBI+1){1'b0}}, l_stg_v[crk]} + {{(IBI+1){1'b0}}, ir_i[crk]});
   wire cr_l  = ~ho_inf & ~ir_l;                       // one head op in flight
   wire cr_f  = {1'b0, rf_free}  > ({1'b0, stg_v_f}  + {1'b0, ir_f});
   // A head op (an AMO, LR/SC or CBO) waits in M for the ROB head, so a CBO dispatches only once
   // every older load and store has its address (an AMO or LR/SC drains first), one head op is in
   // flight at a time (the head-op register holds one), and no load or store dispatches while one
   // is in flight (none can reach memory, or fill the store queue, ahead of it).
   wire       d_cbo_g = d_gc[GC_L] & ~d_gc[GC_PLAIN] & ~d_gc[GC_SER];   // smolrv64_frontend's g0_cbo
   reg        ho_inf;                              // a head op dispatched and not yet done in M
   initial ho_inf = 1'b0;
   wire       ho_end = m_valid & (m_is_amo | m_is_cbo) & m_done;
   always @(posedge clk) begin
      if (rn_valid & d_cls_l) ho_inf <= 1'b1;
      else if (ho_end)        ho_inf <= 1'b0;
      if (reset | redirect)   ho_inf <= 1'b0;
   end
   always @(posedge clk) if (!reset) begin
      if (d_valid & (d_cbo_g != d_is_cbo)) $fatal(1, "smolrv64_core: the dispatch class's CBO decode disagrees with is_cbo");
      if (ho_end & ~ho_inf) $fatal(1, "smolrv64_core: a head op completes with none in flight");
   end
   wire cbo_any = ho_inf | ir_l;
   wire cr_ld = ~cbo_any & (ir_ld ? lq_d_ready2 : lq_d_ready);
   wire cr_st = ~cbo_any & (ir_st ? sq_d_ready2 : sq_d_ready);
   wire cr_cbo = ~lq_uf_any & ~sq_uf_any & ~ir_ld & ~ir_st;
   reg  [IW-1:0] cr_rob;                              // room for the group's slots 0..k
   always @* for (crk = 0; crk < IW; crk = crk + 1) cr_rob[crk] = rob_inflt + crk + 1 <= ROB_DEPTH;
   // the head pops nothing while dispatch is frozen (what the IR holds then is the wrong path and
   // is dropped), while a rename shard is low, or while a serialising op is in flight or in the IR
   wire cr_pop = ~redirect_q & ~fr_v & ~dec_red_q & ~rn_stall & ~ser_inflight & ~(d_valid & d_ser);
   wire cr_ser = drained & ~d_valid & ~csr_infl;            // a serialising op drains first, alone
   wire cr_csr = ~csr_infl & ~(d_valid & d_csr_op);         // a CSR op waits for the one before it
   assign crd = {cr_cbo, cr_csr, cr_ser, cr_pop, cr_rob, cr_st, cr_ld, cr_f, cr_l, cr_i};
   // what the credits guarantee, checked where it is used
   reg  [IW-1:0] s_noroom;                           // a slot's op has no room in what it allocates
   integer       snk;
   always @* for (snk = 0; snk < IW; snk = snk + 1)
      s_noroom[snk] = ~rob_readyv[snk] | ~(s_cls_i[snk] ? (~l_stg_v[snk] | l_ready[snk]) : f_room)
                    | (s_st_nb[snk] & ~sq_d_ready) | (s_ld_nb[snk] & ~lq_d_ready) | ((snk == 0) & ser_block);
   always @(posedge clk) if (!reset) begin
      if (|(s_takev & s_noroom))
         $fatal(1, "smolrv64_core: a group dispatches without room (%b): the credits are wrong", s_takev & s_noroom);
      for (crk = 0; crk < NL; crk = crk + 1)
         if (l_stg_v[crk] & ~l_ready[crk])
            $fatal(1, "smolrv64_core: lane %0d's dispatch-stage register waits for its scheduler: the credits are wrong", crk);
      if (stg_v_f & ~rf_ready)
         $fatal(1, "smolrv64_core: a dispatch-stage register waits for its scheduler: the credits are wrong");
   end
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
   wire d_take = d_valid & ~redirect_q & ~fr_v & ~dec_red_q;   // ~dec_red_q: hold the one-cycle-late decode-redirect window (mirrors redirect_q)
   // The oldest DISPATCHED decode-redirect CTI drives the frontend resteer (fe_red below); the
   // group ends at it, so at most one slot's is set.
   wire [IW-1:0] s_dr = s_takev & s_dcr;
   reg  [PCW-1:0]  dr_tgt;
   reg  [SEQW-1:0] dr_seq;
   reg  [PDW-1:0]  dr_pdet;
   reg  [1:0]      dr_ccls;
   integer         drk;
   always @* begin
      dr_tgt = s_tgt[0];  dr_seq = s_seq[0];  dr_pdet = s_pdet[0];  dr_ccls = s_ccls[0];
      for (drk = IW - 1; drk >= 0; drk = drk - 1)
         if (s_dr[drk]) begin dr_tgt = s_tgt[drk];  dr_seq = s_seq[drk];  dr_pdet = s_pdet[drk];  dr_ccls = s_ccls[drk]; end
   end
   assign dec_red     = |s_dr;
   assign dec_red_tgt = dr_tgt;
   assign dec_red_seq = dr_seq + 1'b1;

   // ---- the RAS top every frontend redirect restores (smolrv64_predictor rb_rsp) ----------------
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
   // The retired pointer: each ROB entry's {call, ret}, written at dispatch, read at commit.
   reg [1:0]      rcls [0:ROB_DEPTH-1];
   integer        rck;
   reg [RASB-1:0] rsp_r;
   integer ri;
   initial begin rsp_r = {RASB{1'b0}}; for (ri = 0; ri < ROB_DEPTH; ri = ri + 1) rcls[ri] = 2'b00; end
   always @(posedge clk) begin
      for (rck = 0; rck < IW; rck = rck + 1) if (s_takev[rck]) rcls[s_rob[rck]] <= s_ccls[rck];
   end
   function [RASB-1:0] rsp_step;    // +1 for a call, -1 for a return
      input [1:0] cls;
      rsp_step = cls[1] ? {{(RASB-1){1'b0}}, 1'b1} : cls[0] ? {RASB{1'b1}} : {RASB{1'b0}};
   endfunction
   reg  [RASB-1:0] rsp_c;
   integer         rsk;
   always @* begin
      rsp_c = {RASB{1'b0}};
      for (rsk = 0; rsk < IW; rsk = rsk + 1) if (rc_v[rsk]) rsp_c = rsp_c + rsp_step(rcls[rc_idx[rsk]]);
   end
   always @(posedge clk) rsp_r <= reset ? {RASB{1'b0}} : rsp_r + rsp_c;
   wire [RASB-1:0] cf_rsp  = cf_pdet[PD_RSP +: RASB]
                           + rsp_step({cf_is_jump & cf_link_rd, cf_is_jalr & cf_link_rs & ~cf_link_rd});
   wire [RASB-1:0] dec_rsp = dr_pdet[PD_RSP +: RASB] + rsp_step({dr_ccls[1], 1'b0});
   wire [RASB-1:0] fe_red_rsp = (m_red_fire | sy_red) ? rsp_r : fr_set ? cf_rsp : dec_rsp;
   always @(posedge clk) fe_red_rsp_q <= fe_red_rsp;
   // ---- the history every frontend redirect restores (smolrv64_predictor rb_ghr) ------------------
   // The redirecting instruction's snapshot, plus its outcome when it is a conditional the BTB
   // knew (the only ones the history holds): a CTF restart's own branch, or none for a decode
   // resteer (a jal, or a backward branch the BTB missed). A redirect at the head (a trap, an
   // xret, a serializing op) starts a new context and restarts the history at zero.
   wire [GHL-1:0] cf_ghr  = cf_pdet[PD_GHR +: GHL];
   wire [GHL-1:0] fr_ghr  = (cf_is_branch & cf_pdet[DCR_HIT]) ? {cf_ghr[GHL-2:0], cf_taken} : cf_ghr;
   wire [GHL-1:0] dec_ghr = dr_pdet[PD_GHR +: GHL];
   wire [GHL-1:0] fe_red_ghr = (m_red_fire | sy_red) ? {GHL{1'b0}} : fr_set ? fr_ghr : dec_ghr;
   always @(posedge clk) fe_red_ghr_q <= fe_red_ghr;
   always @(posedge clk) if (!reset) begin
      // a group dispatches whole and holds at most one load, one store and one F/CTF op
      if (|(s_takev & ~s_v))           $fatal(1, "smolrv64_core: a slot dispatched without an instruction");
      if ((ones_nl(g_st) > 4'd1) | (ones_nl(g_ld) > 4'd1))
         $fatal(1, "smolrv64_core: two allocations into one memory queue");
      if (ones_nl(s_tf) > 4'd1)        $fatal(1, "smolrv64_core: two ops on the F pipe in one group");
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
            m_valid       <= m_go;
            m_pc          <= qm_pc;
            m_insn        <= qm_insn;
            m_rvc         <= qm_rvc;
            m_seq         <= qm_seq;
            m_rd_v        <= qm_rd_v;
            m_prd         <= qm_prd;
            m_shard       <= qm_shard;
            m_addr        <= s_win ? s_va : ho_va;
            m_st_data     <= ho_dat;         // an AMO's or SC's: the store queue reads its own
            m_sq_tag      <= qm_sq_tag;
            m_lq_idx      <= qm_lq_idx;
            m_mem_size    <= qm_mem_size;
            m_mem_signed  <= qm_mem_signed;
            m_is_mem      <= qm_is_mem;
            m_is_store    <= qm_is_store;
            m_is_amo      <= qm_is_amo;
            m_amo_func    <= qm_amo_func;
            m_is_serialize<= qm_is_serialize;
            m_is_fp       <= qm_is_fp;
            m_is_cbo      <= qm_is_cbo;
            m_cbo_zero    <= qm_cbo_zero;
            m_cbo_keep    <= qm_cbo_keep;
         end
      end
   end
   // ---- core_dbg: the state that says what the pipe waits on, for the board's wedge ILA ----
   // [127:112] the FPU's state (fp_unit dbg), [111:90] flags, [89:84] SQ occupancy, [83:78] LQ occupancy, [77:72] ROB head index,
   // [71:40] M's instruction, [39] the FPU busy, [38:0] hpm_ev[38:0] (this cycle's event and stall
   // attribution, the counters' bus; the top-down depth bits above 38 stay off this ILA).
   wire [15:0] fpu_dbg;
   assign core_dbg = {fpu_dbg, rob_empty, rc_v[0], m_valid, m_done, m_at_head, head_block, m_needs_head,
                      irq_inject, inject_inflight, irq_taken, fe_dq_valid, d_take,
                      ic_req, ic_ack, ic_valid, ic_busy, imem_ok, immu_ready,
                      dmem_ren, dmem_idle, lq_x_devwait, redirect,
                      {(6-SQ_IB-1){1'b0}}, sq_occ, {(6-LQ_IB-1){1'b0}}, lq_occ,
                      {(6-ROB_IDXB){1'b0}}, rob_head_idx, m_insn, fpu_busy, hpm_ev[38:0]};
endmodule

`undef PL_DECL
`undef PL_REC
`default_nettype wire
