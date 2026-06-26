`include "exec_pay.vh"
`default_nettype none

// Full sharded-OoO core (frontend + backend), ALU + LSU subset, with commit/CPR:
//   PC -> fetch/align -> decode -> [reg] -> rename -> dispatch
//      -> scheduler (scoreboard issue queues) -> execute (RF + ALU + AGU)
//      -> writeback / unified LSU
//   writeback -> scheduler wake (+ every RF copy)   [write-before-read forwarding]
//
// Commit/CPR (no ROB): one checkpoint per dispatched bundle; commit_ctl counts
// per-checkpoint in-flight instrs (incr at dispatch, decr at completion) and
// commits the oldest in order; bitmap freelists + MAP snapshot recover on a branch
// redirect. Back-pressure (`accept`) freezes the frontend on a full checkpoint
// ring, an empty freelist, a full scheduler, OR a full store buffer / load queue.
//
// LSU (M1): unified store buffer + load queue, flat byte-addressable data memory
// (dmem) stub, physical addresses (dTLB = identity). Loads/stores allocate an LSU
// slot at dispatch (mem_idx threaded through the scheduler like ckpt#), execute
// their AGU in the shard, and the LSU resolves ordering + byte-granular forwarding.
// A load completes at the LSU and writes back on its owner shard's lane (the LSU
// skips lanes busy with an ALU writeback -> no collision). Completion accounting:
// ALU/branch/store at issue; LOADS at LSU completion (excluded from the issue
// decrement, counted via ld_done). TODO: serialize fences/atomics/MMIO via a forced
// unique checkpoint (deferred; the M1 tests don't use them).
module backend_top
  #(parameter IW    = 4,
    parameter HW    = 8,
    parameter PCW   = 64,
    parameter SEQW  = 8,
    parameter ABITS = 6,
    parameter PBITS = 8,
    parameter SCHED_N = 16,      // CAM RS entries/shard. Sweep @3ns: select path was the
                                 // cap (N12=3.17 N16=4.49ns) until the age compare was
                                 // coarsened (low 4 seqno bits dropped) -> N16=2.55ns,
                                 // RS no longer the limiter (shared scoreboard write is).
    parameter CBITS = 2,
    parameter NCHK  = 4,
    parameter DCW   = 3,         // clog2(IW+1)
    parameter CNTW  = 3,         // per-checkpoint outstanding count width
    parameter SBITS = 2,         // clog2(IW) -- owner-shard id width
    parameter AW    = 64,
    parameter SBDEPTH= 4, parameter SBI = 2,   // small store buffer -> shallow byte-merge
    parameter LQDEPTH= 4, parameter LQI = 2,
    parameter MIDXW = 2,         // = max(SBI, LQI)
    parameter [PCW-1:0] RESET_PC = 0)
   (input  wire                    clk,
    input  wire                    reset,
    output wire [PCW-1:0]          imem_addr,
    input  wire [HW*16-1:0]        imem_data,
    input  wire [$clog2(HW+2)-1:0] imem_avail,
    // hardware interrupt-pending lines from the platform CLINT/PLIC:
    // MEIP(11)/SEIP(9)/MTIP(7)/STIP(5)/MSIP(3). Tie to 0 in device-less testbenches.
    input  wire [11:0]             hw_ip,
    // data memory port (flat byte-addressable stub; real D$ later). The READ port is a
    // request/response handshake so a multi-cycle D$ can stall: dmem_ren pulses on a fresh
    // dmem_raddr, dmem_rvalid signals dmem_rdata is valid. Tie dmem_rvalid=1 for a
    // zero-latency memory (combinational read) -> bit-exact 1-cycle loads.
    output wire [AW-1:0]           dmem_raddr,
    output wire                    dmem_ren,
    input  wire [63:0]             dmem_rdata,
    input  wire                    dmem_rvalid,
    output wire                    dmem_wen,
    output wire [AW-1:0]           dmem_waddr,
    output wire [63:0]             dmem_wdata,
    output wire [7:0]              dmem_wmask,
    input  wire                    dmem_wready,        // write accepted/done; tie 1 for 1-cycle writes
    output wire                    dmem_idle,          // LSU store buffer empty (mem current) -- fence.i ordering
    output wire                    ifence,             // FENCE.I redirecting this cycle -- invalidate the I$
    // page-table-walker memory port (registered read; serves the iMMU's TLB misses).
    // Unused in Bare mode (satp.MODE=0) -> may float in the simpler testbenches.
    output wire [55:0]             ptw_addr,
    output wire                    ptw_read,
    input  wire [63:0]             ptw_rdata,
    input  wire                    ptw_rvalid,
    // data-side PTW ports (load path + store/amo path); also float in Bare-mode TBs
    output wire [55:0]             ldptw_addr,
    output wire                    ldptw_read,
    input  wire [63:0]             ldptw_rdata,
    input  wire                    ldptw_rvalid,
    output wire [55:0]             stptw_addr,
    output wire                    stptw_read,
    input  wire [63:0]             stptw_rdata,
    input  wire                    stptw_rvalid,
    // observation: per-shard writeback + the branch redirect
    output wire [IW-1:0]           wb_valid,
    output wire [IW*PBITS-1:0]     wb_pr,
    output wire [IW*64-1:0]        wb_val,
    output wire                    redirect,
    output wire [PCW-1:0]          redirect_target,
    // observation: in-order commit (one checkpoint per pulse)
    output wire                    commit,
    output wire [CBITS-1:0]        commit_idx);

   // ---- redirect (from the execute bundle's oldest mispredicting branch) ----
   wire               eb_redirect;
   wire [63:0]        eb_target;
   wire [SEQW-1:0]    eb_rseq;
   wire [CBITS-1:0]   eb_rckpt;
   wire               eb_rtrap;       // redirect is an exception (roll back TO its ckpt)
   // unified redirect: branch/csr (eb), fetch-fault (iflt), data-fault (dflt). roll_*
   // drives the squash/rollback consumers (scheduler/LSU/commit/issue); fe_red_* drives
   // the frontend PC. A fetch fault fires only when empty -> no squash/rollback needed.
   wire               roll_v, fe_red_v;
   wire [SEQW-1:0]    roll_seq, fe_red_seq;
   wire [CBITS-1:0]   roll_ckpt;
   wire [PCW-1:0]     fe_red_pc;
   assign redirect        = fe_red_v;
   assign redirect_target = fe_red_pc;

   // ---- frontend: fetch -> decode -> rename ----
   wire [IW-1:0]      r_valid, r_rd_v, r_is_branch, fe_stall;
   wire [IW*SEQW-1:0] r_seq;
   wire [IW*ABITS-1:0] r_rd;
   wire [IW*PBITS-1:0] ps1, ps2, ps3, pdst;
   wire [IW*`PAYW-1:0] r_pay;
   wire [CBITS-1:0]   r_ckpt, cur;

   // ---- commit control ----
   wire               cc_commit, cc_rollback, cc_full;
   wire [CBITS-1:0]   cc_commit_idx, cc_rollback_idx, cc_committed;

   // ---- per-slot memory-op classification (from the renamed payload) ----
   wire [IW-1:0]      slot_mem, slot_store, dl_is_load, dl_is_store;
   genvar gi;
   generate for (gi = 0; gi < IW; gi = gi + 1) begin : cls
      assign slot_mem[gi]   = r_pay[gi*`PAYW + `PAY_MEM];
      assign slot_store[gi] = r_pay[gi*`PAYW + `PAY_STORE];
      assign dl_is_load[gi]  = r_valid[gi] & slot_mem[gi] & ~slot_store[gi];
      assign dl_is_store[gi] = r_valid[gi] & slot_mem[gi] &  slot_store[gi];
   end endgenerate

   // ---- LSU dispatch allocation (combinational) ----
   wire [IW*SBI-1:0]  disp_sb_idx;
   wire [IW*LQI-1:0]  disp_lq_idx;
   wire               sb_full, lq_full;
   wire [IW*MIDXW-1:0] disp_mem_idx;
   generate for (gi = 0; gi < IW; gi = gi + 1) begin : midx
      assign disp_mem_idx[gi*MIDXW +: MIDXW] =
         dl_is_store[gi] ? disp_sb_idx[gi*SBI +: SBI] : disp_lq_idx[gi*LQI +: LQI];
   end endgenerate

   // ---- dispatch / back-pressure decision (on the renamed bundle) ----
   wire               lsu_dfault_v;          // data page-fault latched in the LSU (declared early: gates dispatch)
   reg                replay_v;              // fault-replay: refetch the faulting bundle one-op-per-bundle
   initial replay_v = 1'b0;                  // (so the faulting op becomes solo -> precise trap, declared early: feeds frontend)
   reg                ill_v;                 // illegal-instruction fault latched (declared early: gates dispatch)
   reg  [SEQW-1:0]    ill_seq;               // its seqno + checkpoint (set at issue, below)
   reg  [CBITS-1:0]   ill_ckpt;
   initial ill_v = 1'b0;
   wire [IW-1:0]      disp_ready;
   wire               any_valid    = |r_valid;
   // Freeze dispatch while a data page-fault is latched but not yet delivered: it is
   // delivered late (when its checkpoint becomes oldest), and with only NCHK checkpoints
   // wrong-path speculation can wrap the ring and reuse -- thus overwrite -- the faulting
   // checkpoint's chk_seq/chk_pc before delivery. Freezing preserves them. Cannot deadlock:
   // older checkpoints still complete + commit independently of dispatch, so committed
   // advances to the fault's checkpoint and dflt_fire clears the latch.
   wire               can_dispatch = !cc_full && (&disp_ready) && !(|fe_stall)
                                     && !sb_full && !lq_full && !eb_redirect && !lsu_dfault_v && !ill_v;
   wire               disp_fire    = any_valid && can_dispatch;
   wire               accept       = !any_valid || can_dispatch;   // else freeze frontend

   reg  [DCW-1:0]     disp_count;
   integer dc;
   always @* begin
      disp_count = {DCW{1'b0}};
      for (dc = 0; dc < IW; dc = dc + 1) disp_count = disp_count + r_valid[dc];
   end

   // a mispredicting branch/xret reopens the span just AFTER its own bundle (keep it);
   // an EXCEPTION reopens its OWN span (eb_rckpt) so the faulting (solo) op is squashed
   // and its rd allocation annulled -- precise trap.
   wire [CBITS-1:0]   rb_idx = eb_rtrap ? eb_rckpt : (eb_rckpt + 1'b1);

   // ---- instruction-side translation (iMMU): fetch emits a VA; translate to a PA ----
   // Bare mode (satp.MODE=0) is a zero-latency identity passthrough; under Sv39 a TLB
   // hit also resolves combinationally, while a miss forces imem_avail=0 (fetch bubbles)
   // until the PTW fills the TLB. A fetch page fault stalls for now (precise fetch-fault
   // wiring is a later increment; the -v happy path never fetch-faults).
   wire [PCW-1:0]                imem_va;
   wire [55:0]                   immu_pa;
   wire                          immu_ready, immu_fault;
   wire [3:0]                    immu_cause;
   wire [63:0]                   mmu_satp;
   wire [1:0]                    mmu_priv, mmu_dpriv;
   wire                          mmu_sum, mmu_mxr, mmu_flush;
   wire                          eb_fs_off;        // mstatus.FS==Off (FP ops trap illegal)
   wire [$clog2(HW+2)-1:0]       imem_avail_g = (immu_ready & ~immu_fault) ? imem_avail
                                                                           : {$clog2(HW+2){1'b0}};
   assign imem_addr = {8'd0, immu_pa};

   // instruction fetch translates only below M-mode (M fetches are always physical);
   // data accesses translate only when the effective (MPRV-resolved) priv is below M.
   wire [63:0] satp_fetch = (mmu_priv  == 2'd3) ? 64'd0 : mmu_satp;
   wire [63:0] satp_data  = (mmu_dpriv == 2'd3) ? 64'd0 : mmu_satp;

   mmu #(.AW(56)) u_immu
     (.clk(clk), .reset(reset),
      .req_valid(1'b1), .req_vaddr(imem_va), .req_access(2'd0),
      .priv(mmu_priv), .sum(mmu_sum), .mxr(mmu_mxr), .satp(satp_fetch), .flush(mmu_flush),
      .ptw_addr(ptw_addr), .ptw_read(ptw_read), .ptw_rdata(ptw_rdata), .ptw_rvalid(ptw_rvalid),
      .t_ready(immu_ready), .t_paddr(immu_pa), .t_fault(immu_fault), .t_cause(immu_cause));

   // ---- precise page-fault trap injection ----
   // A faulting fetch is the youngest in program order: stall fetch (imem_avail=0,
   // already) and wait until every older op commits (cc_empty); then inject an external
   // trap (epc=tval=faulting VA) into csr_file and redirect to the trap vector. A data
   // (load/store) fault rolls back to the faulting checkpoint first, then injects with
   // epc = that bundle's start PC (a checkpoint is atomic -> re-executing it is correct).
   wire               cc_empty;
   wire [SEQW-1:0]    fe_cur_seq;
   wire               csr_redir_v;
   wire [63:0]        csr_redir_tgt;
   // data-fault trap (assigned below near the LSU); declared early so iflt can defer to it
   wire               dflt_fire;
   wire               dflt_ready;       // faulting mem op is the oldest live checkpoint
   wire               dflt_replay;      // phase 1: roll back + refetch it solo (no trap yet)
   wire               dflt_roll;        // either phase rolls back to the faulting checkpoint
   // pending interrupt (from csr_file) + the precise delivery decision (assigned near dflt)
   wire               csr_irq_v;
   wire [3:0]         csr_irq_cause;
   wire               irq_inject;       // inject the interrupt pseudo-op this cycle

   reg                pend_iflt;
   reg  [63:0]        iflt_va;
   reg  [3:0]         iflt_cause;
   wire               iflt_fire;        // fetch-fault trap fires this cycle
   initial pend_iflt = 1'b0;
   always @(posedge clk) begin
      if (reset) pend_iflt <= 1'b0;
      // A pending fetch fault is for the YOUNGEST (frontier) fetch. ANY redirect (branch,
      // data fault, interrupt) or its own delivery changes the fetch stream, so a fault
      // latched for the now-squashed (wrong-path) frontier is stale -> drop it; the new
      // stream re-faults next cycle if it is genuinely unmapped. Without this, a speculative
      // wrong-path fetch fault survives a rollback and fires spuriously once cc_empty (e.g.
      // a data-fault rollback then a stale iflt to the branch's fail target).
      else if (roll_v) pend_iflt <= 1'b0;
      else if (immu_fault & ~pend_iflt) begin
         pend_iflt <= 1'b1; iflt_va <= imem_va; iflt_cause <= immu_cause;
      end
   end
   // Suppress fetch-fault delivery during a data-fault replay: the replaying op is older,
   // so a younger speculative fetch fault must not preempt it (replay empties the pipe ->
   // cc_empty, which would otherwise let iflt fire). EXCEPTION: a replay that drains to an
   // empty pipe with a pending fetch fault and NO data/illegal fault re-raised has been
   // RECLASSIFIED into that fetch fault -- e.g. an "illegal" op that was really a mis-fetched
   // instruction in an unmapped page past a fetch-window boundary. Let iflt fire (and clear
   // replay_v, below); else replay_v sticks (dflt_fire never comes) and blocks all faults.
   wire replay_to_iflt = replay_v & cc_empty & pend_iflt & ~lsu_dfault_v & ~ill_v;
   assign iflt_fire = pend_iflt & cc_empty & (~replay_v | replay_to_iflt);

   wire [3:0]         dflt_cause;
   wire [63:0]        dflt_epc, dflt_tval;
   // xtrap carries EXCEPTIONS only now (fetch/data page faults). Interrupts are delivered
   // by the injected irq_take pseudo-op through the SYSTEM-op trap path, not here.
   wire               xtrap_v     = iflt_fire | dflt_fire;
   wire               xtrap_intr  = 1'b0;
   wire [3:0]         xtrap_cause = iflt_fire ? iflt_cause : dflt_cause;
   wire [63:0]        xtrap_epc   = iflt_fire ? iflt_va    : dflt_epc;
   wire [63:0]        xtrap_tval  = iflt_fire ? iflt_va    : dflt_tval;

   frontend #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW), .ABITS(ABITS),
              .PBITS(PBITS), .NCHK(NCHK), .CBITS(CBITS), .RESET_PC(RESET_PC)) fe
     (.clk(clk), .reset(reset),
      .redirect(fe_red_v), .redirect_pc(fe_red_pc),
      .redirect_seq(fe_red_seq), .solo_all(replay_v), .irq_inject(irq_inject),
      .imem_addr(imem_va), .imem_data(imem_data), .imem_avail(imem_avail_g),
      .accept(accept),
      .create(disp_fire), .commit(cc_commit), .commit_idx(cc_commit_idx),
      .rollback(cc_rollback), .rollback_idx(cc_rollback_idx),
      .r_valid(r_valid), .r_seq(r_seq), .r_rd(r_rd), .r_rd_v(r_rd_v),
      .ps1(ps1), .ps2(ps2), .ps3(ps3), .pdst(pdst),
      .r_is_branch(r_is_branch),
      .r_pay(r_pay), .r_ckpt(r_ckpt), .cur(cur), .cur_seq(fe_cur_seq), .stall(fe_stall));

   // ---- scheduler bundle ----
   wire [IW-1:0]       iss_valid, iss_pdst_v;
   wire [IW*PBITS-1:0] iss_pdst, iss_ps1, iss_ps2, iss_ps3;
   wire [IW*SEQW-1:0]  iss_seq;
   wire [IW*CBITS-1:0] iss_ckpt;
   wire [IW*MIDXW-1:0] iss_mem_idx;
   wire [IW*`PAYW-1:0] iss_pay;
   wire [IW-1:0]       wkv;          // effective writeback = wake source (ALU ∪ load)
   wire [IW*PBITS-1:0] wkp;
   // per-shard iterative-divide status (exec_bundle -> scheduler stall + commit count)
   wire [IW-1:0]       eb_exec_busy, eb_div_done, eb_fp_done;
   wire [IW*CBITS-1:0] eb_div_done_ckpt, eb_fp_done_ckpt;
   wire [IW-1:0]       q_iss_is_fp;
   // wake bus into the scheduler (select-time + completion-time) + per-shard issue stall
   wire [2*IW-1:0]       sched_wake_v;
   wire [2*IW*PBITS-1:0] sched_wake_pr;
   wire [IW-1:0]         busy_to_sched;

   wire [IW*CBITS-1:0] disp_ckpt = {IW{r_ckpt}};
   wire [IW-1:0]      sched_disp_valid = r_valid & {IW{disp_fire}};

   sched_bundle #(.SHARDS(IW), .PBITS(PBITS), .SEQW(SEQW), .N(SCHED_N),
                  .CBITS(CBITS), .MIDXW(MIDXW)) sb
     (.clk(clk), .reset(reset),
      .disp_valid(sched_disp_valid), .disp_seq(r_seq), .disp_pdst(pdst), .disp_pdst_v(r_rd_v),
      .disp_ps1(ps1), .disp_ps2(ps2),
      .disp_ps3(ps3),                // FMA 3rd operand (renamed; p0 for non-FMA ops via rs3_v=0)
      .disp_ckpt(disp_ckpt), .disp_mem_idx(disp_mem_idx),
      .disp_pay(r_pay), .disp_ready(disp_ready),
      .wake_valid(sched_wake_v), .wake_pr(sched_wake_pr),
      .squash(roll_v), .squash_seq(roll_seq), .exec_busy(busy_to_sched),
      .committed(cc_committed),
      .iss_valid(iss_valid), .iss_pdst(iss_pdst), .iss_pdst_v(iss_pdst_v),
      .iss_ps1(iss_ps1), .iss_ps2(iss_ps2), .iss_ps3(iss_ps3),     // ps3 = FMA 3rd operand
      .iss_seq(iss_seq),
      .iss_ckpt(iss_ckpt), .iss_mem_idx(iss_mem_idx), .iss_pay(iss_pay));

   // ---- per-issue memory-op decode (from the payload, for the LSU execute drive) ----
   wire [IW-1:0]      iss_mem, iss_store, iss_is_load, iss_is_mul, iss_is_amo, iss_is_fp;
   generate for (gi = 0; gi < IW; gi = gi + 1) begin : icl
      assign iss_mem[gi]     = iss_pay[gi*`PAYW + `PAY_MEM];
      assign iss_store[gi]   = iss_pay[gi*`PAYW + `PAY_STORE];
      assign iss_is_load[gi] = iss_valid[gi] & iss_mem[gi] & ~iss_store[gi];
      // M-ops (mul AND div) are deferred multi-cycle -> excluded from select-wake / the
      // issue-time commit decrement, counted at completion, and stall their shard.
      assign iss_is_mul[gi]  = iss_valid[gi] & iss_pay[gi*`PAYW + `PAY_MUL];
      // atomics complete at the LSU (variable latency) -> also excluded from select-wake.
      assign iss_is_amo[gi]  = iss_valid[gi] & iss_pay[gi*`PAYW + `PAY_AMO];
      // FPU-arith (use_fpu) results land at the CVFPU pipe (deferred, like divides) -> their
      // dest must NOT select-wake (a consumer would read the stale RF before fp_done).
      wire iss_fpu;
      decode_fp u_iifp (.insn(iss_pay[gi*`PAYW + 165 +: 32]), .fp_valid(), .use_fpu(iss_fpu),
         .fp_class(), .op(), .op_mod(), .src_fmt(), .dst_fmt(), .int_fmt(), .rnd(),
         .op0_sel(), .op1_sel(), .op2_sel(), .op0_int(), .wr_fp());
      assign iss_is_fp[gi]   = iss_valid[gi] & iss_fpu;
   end endgenerate

   // ================= registered issue stage (select | execute split) =================
   // The scheduler's combinational select (iss_*) is registered here; execute, LSU and
   // commit consume the registered q_iss_*. This breaks the old fused select->RF->ALU->wb
   // megapath. Latency-1 back-to-back is preserved by a SELECT-TIME wake (below): the
   // selected latency-1 dest is broadcast now, so a dependent is selected next cycle and
   // reads the result from the RF the cycle after (write-before-read across this stage).
   // A wrong-path op selected the same cycle a branch redirects is gated out here.
   reg  [IW-1:0]       q_iss_valid, q_iss_pdst_v;
   reg  [IW*PBITS-1:0] q_iss_pdst, q_iss_ps1, q_iss_ps2, q_iss_ps3;
   reg  [IW*SEQW-1:0]  q_iss_seq;
   reg  [IW*CBITS-1:0] q_iss_ckpt;
   reg  [IW*MIDXW-1:0] q_iss_mem_idx;
   reg  [IW*`PAYW-1:0] q_iss_pay;
   wire [IW-1:0]       iss_squashed;
   genvar gq;
   generate for (gq = 0; gq < IW; gq = gq + 1) begin : sq
      assign iss_squashed[gq] = roll_v & ($signed(iss_seq[gq*SEQW +: SEQW] - roll_seq) > 0);
   end endgenerate
   integer qi;
   initial begin q_iss_valid = {IW{1'b0}}; end
   always @(posedge clk) begin
      if (reset) q_iss_valid <= {IW{1'b0}};
      else begin
         q_iss_valid   <= iss_valid & ~iss_squashed;
         q_iss_pdst_v  <= iss_pdst_v;
         q_iss_pdst    <= iss_pdst;   q_iss_ps1 <= iss_ps1;  q_iss_ps2 <= iss_ps2;  q_iss_ps3 <= iss_ps3;
         q_iss_seq     <= iss_seq;    q_iss_ckpt <= iss_ckpt;
         q_iss_mem_idx <= iss_mem_idx; q_iss_pay <= iss_pay;
      end
   end

   // execute-stage op decode (from the registered payload) -> LSU + commit
   wire [IW-1:0]   q_iss_mem, q_iss_store, q_iss_is_load, q_iss_is_store, q_iss_is_mul, q_iss_is_amo, q_iss_defer;
   wire [IW-1:0]   q_iss_is_ill, q_iss_is_ill_eff, q_iss_fp_dis;
   wire            data_xlate = (satp_data[63:60] == 4'd8);   // Sv39 on for data accesses
   wire [IW*4-1:0] q_iss_nb;
   wire [IW-1:0]   q_iss_sgn;
   generate for (gi = 0; gi < IW; gi = gi + 1) begin : qicl
      wire [1:0] qsz = q_iss_pay[gi*`PAYW + 148 +: 2];
      assign q_iss_mem[gi]     = q_iss_pay[gi*`PAYW + `PAY_MEM];
      assign q_iss_store[gi]   = q_iss_pay[gi*`PAYW + `PAY_STORE];
      assign q_iss_sgn[gi]     = q_iss_pay[gi*`PAYW + `PAY_MSGN];
      assign q_iss_nb[gi*4+:4] = (4'd1 << qsz);
      assign q_iss_is_load[gi] = q_iss_valid[gi] & q_iss_mem[gi] & ~q_iss_store[gi];
      assign q_iss_is_store[gi]= q_iss_valid[gi] & q_iss_mem[gi] &  q_iss_store[gi] & ~q_iss_is_amo[gi];
      assign q_iss_is_mul[gi]  = q_iss_valid[gi] & q_iss_pay[gi*`PAYW + `PAY_MUL];
      assign q_iss_is_amo[gi]  = q_iss_valid[gi] & q_iss_pay[gi*`PAYW + `PAY_AMO];
      // an illegal instruction never completes: like a faulting load it is deferred so its
      // checkpoint stays open (never commits) until the illegal-instruction trap is delivered.
      assign q_iss_is_ill[gi]  = q_iss_valid[gi] & q_iss_pay[gi*`PAYW + `PAY_ILL];
      // FS-disabled trap: an FP instruction (LOAD/STORE-FP, OP-FP, FMADD family) executed
      // with mstatus.FS==Off raises illegal-instruction (cause 2), like Linux lazy-FP. A
      // change to FS redirects+refetches younger ops (csr_file do_fschg), so by the time an
      // FP op reaches here fs_off is current -> this execute-time check is precise.
      wire [6:0] qop = q_iss_pay[gi*`PAYW + 165 +: 7];
      wire qi_fpop = (qop==7'b0000111) | (qop==7'b0100111) | (qop==7'b1010011)
                   | (qop==7'b1000011) | (qop==7'b1000111) | (qop==7'b1001011) | (qop==7'b1001111);
      assign q_iss_fp_dis[gi]  = q_iss_valid[gi] & qi_fpop & eb_fs_off;
      assign q_iss_is_ill_eff[gi] = q_iss_is_ill[gi] | q_iss_fp_dis[gi];
      // loads AND atomics complete at the LSU -> deferred (excluded from the issue-time
      // commit decrement, counted via ld_done instead). Under Sv39, plain stores also defer
      // (counted via st_done) so a store page fault is delivered precisely (the store holds
      // its checkpoint open until its translation is checked). In Bare mode stores keep
      // counting at issue -- full (parallel) store throughput, and prompt drain (fence_i).
      // Illegal ops defer too -- they hold their checkpoint for the precise trap.
      assign q_iss_defer[gi]   = q_iss_is_load[gi] | q_iss_is_amo[gi] | q_iss_is_ill_eff[gi]
                                 | (q_iss_is_store[gi] & data_xlate);
      // FPU-arith ops complete at the FP unit (deferred, like divides) -> excluded from the
      // issue count and decremented at fp_done.
      wire qi_fpu;
      decode_fp u_qifp (.insn(q_iss_pay[gi*`PAYW + 165 +: 32]), .fp_valid(), .use_fpu(qi_fpu),
         .fp_class(), .op(), .op_mod(), .src_fmt(), .dst_fmt(), .int_fmt(), .rnd(),
         .op0_sel(), .op1_sel(), .op2_sel(), .op0_int(), .wr_fp());
      assign q_iss_is_fp[gi] = q_iss_valid[gi] & qi_fpu;
   end endgenerate

   // FP-disabled flag aligned to the EX stage (gates the LSU FP load/store dispatch so a
   // disabled FLW/FSW makes no memory access; it is held + trapped via q_iss_is_ill_eff).
   reg [IW-1:0] ex_fp_dis;
   initial ex_fp_dis = {IW{1'b0}};
   always @(posedge clk) begin
      if (reset) ex_fp_dis <= {IW{1'b0}};
      else       ex_fp_dis <= q_iss_fp_dis;
   end

   // ---- wake: select-time (latency-1) + completion-time (load/divide via wb) ----
   // select-wake fires for a selected latency-1 register writer (not mem, not div).
   wire [IW-1:0]       sel_wake_v;
   wire [IW*PBITS-1:0] sel_wake_pr;
   generate for (gi = 0; gi < IW; gi = gi + 1) begin : selw
      assign sel_wake_v[gi]             = iss_valid[gi] & iss_pdst_v[gi]
                                          & ~iss_mem[gi] & ~iss_is_mul[gi] & ~iss_is_amo[gi]
                                          & ~iss_is_fp[gi];
      assign sel_wake_pr[gi*PBITS +: PBITS] = iss_pdst[gi*PBITS +: PBITS];
   end endgenerate
   assign sched_wake_v  = {sel_wake_v, wkv};        // [hi]=select, [lo]=completion
   assign sched_wake_pr = {sel_wake_pr, wkp};

   // a divide heading to / running on a shard's divider stalls that shard's issue
   assign busy_to_sched = q_iss_is_mul | eb_exec_busy;

   // ---- commit control: count by completion (loads at LSU), commit in order ----
   wire               lsu_ld_done;
   wire [CBITS-1:0]   lsu_ld_done_ckpt;
   wire               lsu_st_done;
   wire [CBITS-1:0]   lsu_st_done_ckpt;
   // ---- data page-fault report from the LSU (-> precise trap, below) ----
   wire [SEQW-1:0]    lsu_dfault_seq;
   wire [CBITS-1:0]   lsu_dfault_ckpt;
   wire [3:0]         lsu_dfault_cause;
   wire [AW-1:0]      lsu_dfault_tval;
   commit_ctl #(.NCHK(NCHK), .CBITS(CBITS), .IW(IW), .CNTW(CNTW), .DCW(DCW)) cc
     (.clk(clk), .reset(reset), .cur(cur),
      .disp_fire(disp_fire), .disp_count(disp_count),
      .iss_valid(q_iss_valid), .iss_is_load(q_iss_defer), .iss_is_div(q_iss_is_mul),
      .iss_is_fp(q_iss_is_fp), .fp_done(eb_fp_done), .fp_done_ckpt(eb_fp_done_ckpt), .iss_ckpt(q_iss_ckpt),
      .ld_done(lsu_ld_done), .ld_done_ckpt(lsu_ld_done_ckpt),
      .st_done(lsu_st_done), .st_done_ckpt(lsu_st_done_ckpt),
      .div_done(eb_div_done), .div_done_ckpt(eb_div_done_ckpt),
      .redirect(roll_v), .redirect_ckpt(roll_ckpt),
      .create(), .empty(cc_empty),
      .commit(cc_commit), .commit_idx(cc_commit_idx),
      .rollback(cc_rollback), .rollback_idx(cc_rollback_idx),
      .committed_idx(cc_committed), .full(cc_full));

   assign commit     = cc_commit;
   assign commit_idx = cc_commit_idx;

   // ---- execute bundle (2-stage RR|EX + forwarding + wb broadcast + branch) ----
   wire [IW*64-1:0]   eb_agu, eb_stdata;
   wire [IW-1:0]      eb_amo;
   wire [IW*5-1:0]    eb_amo_func;
   wire [IW*PBITS-1:0] eb_amo_pdst;
   wire [IW-1:0]      eb_wb_busy;
   wire               lsu_ld_wb_v;
   wire [SBITS-1:0]   lsu_ld_wb_owner;
   wire [PBITS-1:0]   lsu_ld_wb_pdst;
   wire [63:0]        lsu_ld_wb_val;
   // EX-stage LSU control (from exec_bundle, aligned with eb_agu/eb_stdata)
   wire [IW-1:0]      ex_valid, ex_mem, ex_store, ex_msigned, ex_fp;
   wire [IW*SEQW-1:0] ex_seq;
   wire [IW*CBITS-1:0] ex_ckpt;
   wire [IW*MIDXW-1:0] ex_mem_idx;
   wire [IW*2-1:0]    ex_msize;

   exec_bundle #(.SHARDS(IW), .SBITS(SBITS), .PBITS(PBITS), .SEQW(SEQW), .CBITS(CBITS), .MIDXW(MIDXW)) eb
     (.clk(clk), .reset(reset),
      .iss_valid(q_iss_valid), .iss_seq(q_iss_seq), .iss_pdst(q_iss_pdst),
      .iss_pdst_v(q_iss_pdst_v), .iss_ps1(q_iss_ps1), .iss_ps2(q_iss_ps2), .iss_ps3(q_iss_ps3),
      .iss_ckpt(q_iss_ckpt), .iss_mem_idx(q_iss_mem_idx), .iss_pay(q_iss_pay),
      .squash(roll_v), .squash_seq(roll_seq),
      .exec_busy(eb_exec_busy), .div_done(eb_div_done), .div_done_ckpt(eb_div_done_ckpt),
      .fp_done(eb_fp_done), .fp_done_ckpt(eb_fp_done_ckpt),
      .lsu_wb_v(lsu_ld_wb_v), .lsu_wb_owner(lsu_ld_wb_owner),
      .lsu_wb_pr(lsu_ld_wb_pdst), .lsu_wb_val(lsu_ld_wb_val), .wb_busy(eb_wb_busy),
      .wb_valid(wkv), .wb_pr(wkp), .wb_val(wb_val),
      .ex_valid(ex_valid), .ex_seq(ex_seq), .ex_ckpt(ex_ckpt), .ex_mem_idx(ex_mem_idx),
      .ex_mem(ex_mem), .ex_store(ex_store), .ex_fp(ex_fp), .ex_msize(ex_msize), .ex_msigned(ex_msigned),
      .agu_addr(eb_agu), .st_data(eb_stdata),
      .ex_amo(eb_amo), .ex_amo_func(eb_amo_func), .ex_amo_pdst(eb_amo_pdst),
      .redirect(eb_redirect), .redirect_target(eb_target),
      .redirect_seq(eb_rseq), .redirect_ckpt(eb_rckpt), .redirect_is_trap(eb_rtrap),
      .ifence(ifence),
      .mmu_satp(mmu_satp), .mmu_priv(mmu_priv), .mmu_dpriv(mmu_dpriv),
      .mmu_sum(mmu_sum), .mmu_mxr(mmu_mxr), .mmu_flush(mmu_flush), .fs_off(eb_fs_off),
      .xtrap_v(xtrap_v), .xtrap_intr(xtrap_intr), .xtrap_cause(xtrap_cause),
      .xtrap_epc(xtrap_epc), .xtrap_tval(xtrap_tval),
      .hw_ip(hw_ip),
      .irq_v(csr_irq_v), .irq_cause(csr_irq_cause),
      .csr_redir_v(csr_redir_v), .csr_redir_tgt(csr_redir_tgt));

   // ---- LSU execute-port drive (EX stage: bypassed AGU/store-data + EX control) ----
   wire [IW-1:0]      exe_st_v, exe_ld_v;
   wire [IW*SBI-1:0]  exe_st_idx;
   wire [IW*LQI-1:0]  exe_ld_idx;
   wire [IW*AW-1:0]   exe_st_addr, exe_ld_addr;
   wire [IW*64-1:0]   exe_st_data;
   wire [IW*4-1:0]    exe_st_nb, exe_ld_nb;
   wire [IW-1:0]      exe_ld_sgn, exe_ld_fp;
   generate for (gi = 0; gi < IW; gi = gi + 1) begin : exd
      assign exe_st_v[gi] = ex_valid[gi] & ex_mem[gi] &  ex_store[gi] & ~ex_fp_dis[gi];
      assign exe_ld_v[gi] = ex_valid[gi] & ex_mem[gi] & ~ex_store[gi] & ~ex_fp_dis[gi];
      assign exe_st_idx[gi*SBI +: SBI] = ex_mem_idx[gi*MIDXW +: SBI];
      assign exe_ld_idx[gi*LQI +: LQI] = ex_mem_idx[gi*MIDXW +: LQI];
      assign exe_st_addr[gi*AW +: AW]  = eb_agu[gi*64 +: AW];
      assign exe_ld_addr[gi*AW +: AW]  = eb_agu[gi*64 +: AW];
      assign exe_st_data[gi*64 +: 64]  = eb_stdata[gi*64 +: 64];
      assign exe_st_nb[gi*4 +: 4]      = (4'd1 << ex_msize[gi*2 +: 2]);
      assign exe_ld_nb[gi*4 +: 4]      = (4'd1 << ex_msize[gi*2 +: 2]);
      assign exe_ld_sgn[gi]            = ex_msigned[gi];
      assign exe_ld_fp[gi]             = ex_fp[gi];           // FLW -> NaN-box the word load
   end endgenerate

   // ---- single active atomic -> LSU amo port (atomics are serialized+solo: <=1 at EX) ----
   reg                amo_v;   reg [4:0] amo_func;  reg [1:0] amo_sz;
   reg  [AW-1:0]      amo_addr; reg [63:0] amo_data;
   reg  [PBITS-1:0]   amo_pdst; reg [SBITS-1:0] amo_owner; reg [CBITS-1:0] amo_ckpt;
   reg  [SEQW-1:0]    amo_seq;
   integer am;
   always @* begin
      amo_v=1'b0; amo_func=5'd0; amo_sz=2'd0; amo_addr={AW{1'b0}}; amo_data=64'd0;
      amo_pdst={PBITS{1'b0}}; amo_owner={SBITS{1'b0}}; amo_ckpt={CBITS{1'b0}}; amo_seq={SEQW{1'b0}};
      for (am = 0; am < IW; am = am + 1) if (eb_amo[am]) begin
         amo_v=1'b1; amo_func=eb_amo_func[am*5 +: 5]; amo_sz=ex_msize[am*2 +: 2];
         amo_addr=eb_agu[am*64 +: AW]; amo_data=eb_stdata[am*64 +: 64];
         amo_pdst=eb_amo_pdst[am*PBITS +: PBITS]; amo_owner=am[SBITS-1:0];
         amo_ckpt=ex_ckpt[am*CBITS +: CBITS]; amo_seq=ex_seq[am*SEQW +: SEQW];
      end
   end

   lsu #(.IW(IW), .SBITS(SBITS), .PBITS(PBITS), .SEQW(SEQW), .CBITS(CBITS), .AW(AW),
         .SBDEPTH(SBDEPTH), .SBI(SBI), .LQDEPTH(LQDEPTH), .LQI(LQI)) u_lsu
     (.clk(clk), .reset(reset),
      .disp_fire(disp_fire), .disp_is_load(dl_is_load), .disp_is_store(dl_is_store),
      .disp_seq(r_seq), .disp_ckpt(disp_ckpt), .disp_pdst(pdst),
      .disp_sb_idx(disp_sb_idx), .disp_lq_idx(disp_lq_idx),
      .sb_full(sb_full), .lq_full(lq_full),
      .exe_st_v(exe_st_v), .exe_st_idx(exe_st_idx), .exe_st_addr(exe_st_addr),
      .exe_st_data(exe_st_data), .exe_st_nb(exe_st_nb),
      .exe_ld_v(exe_ld_v), .exe_ld_idx(exe_ld_idx), .exe_ld_addr(exe_ld_addr),
      .exe_ld_nb(exe_ld_nb), .exe_ld_sgn(exe_ld_sgn), .exe_ld_fp(exe_ld_fp),
      .amo_v(amo_v), .amo_func(amo_func), .amo_addr(amo_addr), .amo_data(amo_data),
      .amo_sz(amo_sz), .amo_pdst(amo_pdst), .amo_owner(amo_owner), .amo_ckpt(amo_ckpt),
      .amo_seq(amo_seq),
      .xl_satp(satp_data), .xl_priv(mmu_dpriv), .xl_sum(mmu_sum), .xl_mxr(mmu_mxr),
      .xl_flush(mmu_flush),
      .ldp_addr(ldptw_addr), .ldp_read(ldptw_read),
      .ldp_rdata(ldptw_rdata), .ldp_rvalid(ldptw_rvalid),
      .stp_addr(stptw_addr), .stp_read(stptw_read),
      .stp_rdata(stptw_rdata), .stp_rvalid(stptw_rvalid),
      .dfault_v(lsu_dfault_v), .dfault_seq(lsu_dfault_seq),
      .dfault_ckpt(lsu_dfault_ckpt), .dfault_cause(lsu_dfault_cause),
      .dfault_tval(lsu_dfault_tval),
      .st_done(lsu_st_done), .st_done_ckpt(lsu_st_done_ckpt), .sb_empty(dmem_idle),
      .mem_raddr(dmem_raddr), .mem_ren(dmem_ren), .mem_rdata(dmem_rdata), .mem_rvalid(dmem_rvalid),
      .mem_wen(dmem_wen), .mem_waddr(dmem_waddr), .mem_wdata(dmem_wdata), .mem_wmask(dmem_wmask),
      .mem_wready(dmem_wready),
      .wb_busy(eb_wb_busy),
      .ld_wb_v(lsu_ld_wb_v), .ld_wb_pdst(lsu_ld_wb_pdst), .ld_wb_owner(lsu_ld_wb_owner),
      .ld_wb_val(lsu_ld_wb_val), .ld_done(lsu_ld_done), .ld_done_ckpt(lsu_ld_done_ckpt),
      .commit(cc_commit), .commit_idx(cc_commit_idx),
      .rollback(roll_v), .rollback_seq(roll_seq), .dfault_taken(dflt_fire));

   // ---- per-checkpoint base PC/seq (precise data-fault trap epc + squash boundary) ----
   reg  [PCW-1:0]  chk_pc  [0:NCHK-1];
   reg  [SEQW-1:0] chk_seq [0:NCHK-1];
   wire [PCW-1:0]  disp_base_pc = r_pay[`PAY_PC];   // slot-0 PC = the bundle's oldest op
   always @(posedge clk) if (disp_fire) begin
      chk_pc [cur] <= disp_base_pc;
      chk_seq[cur] <= r_seq[SEQW-1:0];
   end

   // ---- illegal-instruction fault latch (detected at the registered issue stage) ----
   // An illegal op is deferred (q_iss_defer), so it holds its checkpoint open exactly like
   // a faulting load -- giving the deferred-fault machinery time to deliver a precise trap.
   // Latch the OLDEST pending illegal op; clear it when its own trap fires or any rollback
   // squashes it (mirrors the LSU data-fault latch). Both faults share one replay-to-solo path.
   reg              il_now;
   reg  [SEQW-1:0]  il_nseq;
   reg  [CBITS-1:0] il_nck;
   integer iq;
   always @* begin
      il_now = 1'b0; il_nseq = {SEQW{1'b0}}; il_nck = {CBITS{1'b0}};
      for (iq = 0; iq < IW; iq = iq + 1)
         // exclude an op being squashed by THIS cycle's rollback (newer than roll_seq):
         // otherwise a wrong-path illegal op (e.g. speculation into zero-padding past an
         // ecall) latches ill_v just as it is squashed, and nothing later clears it -> hang.
         if (q_iss_is_ill_eff[iq] &&
             !(roll_v && $signed(roll_seq - q_iss_seq[iq*SEQW +: SEQW]) < 0) &&
             (!il_now || $signed(q_iss_seq[iq*SEQW +: SEQW] - il_nseq) < 0)) begin
            il_now  = 1'b1;
            il_nseq = q_iss_seq[iq*SEQW +: SEQW];
            il_nck  = q_iss_ckpt[iq*CBITS +: CBITS];
         end
   end

   // data page/access OR illegal fault: the faulting/illegal op blocks commit -> handle once
   // its checkpoint is the oldest live one (committed_idx). A checkpoint is atomic, so a
   // mid-bundle faulting op cannot be made precise directly (its older siblings would be
   // annulled too). Two-phase REPLAY-TO-SOLO: phase 1 rolls back to the bundle start and
   // refetches it one-op-per-bundle (solo_all), so the older siblings land in their own
   // (committable) checkpoints; phase 2, with the faulting op now solo, delivers a precise
   // trap (epc = chk_pc = that op's PC) and annuls only it. An already-solo op (AMO, a load
   // alone in its bundle, or a solo illegal op) just pays one extra refetch -- still correct.
   wire             df_oldest = lsu_dfault_v & (cc_committed == lsu_dfault_ckpt);
   wire             il_oldest = ill_v        & (cc_committed == ill_ckpt);
   wire             flt_v     = df_oldest | il_oldest;          // data fault wins ties (same ckpt)
   wire [SEQW-1:0]  flt_seq   = df_oldest ? lsu_dfault_seq   : ill_seq;
   wire [CBITS-1:0] flt_ckpt  = df_oldest ? lsu_dfault_ckpt  : ill_ckpt;
   wire [3:0]       flt_cause = df_oldest ? lsu_dfault_cause : 4'd2;       // 2 = illegal instruction
   wire [AW-1:0]    flt_tval  = df_oldest ? lsu_dfault_tval  : {AW{1'b0}}; // mtval=0 for illegal

   assign dflt_ready  = flt_v & ~iflt_fire;
   // already first in its bundle (no older siblings to commit) -> precise directly, no replay
   wire   dflt_solo   = (flt_seq == chk_seq[flt_ckpt]);
   assign dflt_replay = dflt_ready & ~dflt_solo & ~replay_v;   // phase 1 (mid-bundle fault only)
   assign dflt_fire   = dflt_ready & ( dflt_solo |  replay_v); // phase 2, or direct when already solo
   assign dflt_roll   = dflt_ready;                 // any delivery/replay rolls back the same way
   assign dflt_cause = flt_cause;
   assign dflt_epc   = chk_pc[flt_ckpt];
   assign dflt_tval  = flt_tval;

   // ---- interrupt injection (precise, via the irq_take pseudo-op) ----
   // When an interrupt is enabled+pending, inject a synthetic solo SYSTEM op at the current
   // fetch PC (fetch holds PC). It renames/schedules like an ecall, becomes the oldest, and
   // csr_file delivers the trap there (mepc = that PC; rolls back TO its own checkpoint to
   // squash the displaced/younger ops -- which then re-fetch after mret). This reuses the
   // entire SYSTEM-op exception path, so no roll-oldest-checkpoint machinery is needed (and
   // it lets older in-flight work commit, sidestepping the commit-count-orphan corner).
   // One pseudo-op in flight at a time: inject_inflight latches at injection and clears when
   // the op resolves (its own trap, or any rollback squashes it) or the interrupt clears.
   reg inject_inflight; initial inject_inflight = 1'b0;
   assign irq_inject = csr_irq_v & ~inject_inflight & ~replay_v & ~pend_iflt & ~lsu_dfault_v
                       & ~ill_v & ~eb_redirect & ~dflt_replay & ~dflt_fire & ~iflt_fire & ~roll_v;
   always @(posedge clk) begin
      if (reset)                    inject_inflight <= 1'b0;
      else if (irq_inject & accept) inject_inflight <= 1'b1;   // pseudo-op entered the pipe
      else if (roll_v | ~csr_irq_v) inject_inflight <= 1'b0;   // squashed/delivered/cleared
   end

   always @(posedge clk) begin
      if (reset) ill_v <= 1'b0;
      else if (ill_v && ((dflt_fire & il_oldest) ||
                         (roll_v && $signed(roll_seq - ill_seq) < 0))) ill_v <= 1'b0;
      else if (il_now && (!ill_v || $signed(il_nseq - ill_seq) < 0)) begin
         ill_v <= 1'b1; ill_seq <= il_nseq; ill_ckpt <= il_nck;
      end
   end

   always @(posedge clk) begin
      if (reset)                       replay_v <= 1'b0;
      else if (dflt_fire | iflt_fire)  replay_v <= 1'b0;  // trap delivered (data, or reclassified fetch)
      else if (dflt_replay)            replay_v <= 1'b1;  // entered solo replay
   end

   // unified redirect distribution. A fetch fault fires only when empty, so its rollback
   // is a no-op functionally but keeps the frontend flush paired with a rename rollback
   // (decode_rename restores its map on rollback) -- an unpaired flush leaves the map/
   // checkpoint state stale (count[] -> X). Roll back to the committed (== open) ckpt.
   assign roll_v     = eb_redirect | dflt_roll | iflt_fire;
   assign roll_seq   = iflt_fire ? fe_cur_seq
                     : dflt_roll ? (chk_seq[flt_ckpt] - 1'b1) : eb_rseq;
   assign roll_ckpt  = iflt_fire ? cc_committed
                     : dflt_roll ? flt_ckpt : rb_idx;
   assign fe_red_v   = roll_v | iflt_fire;
   // phase 2 / fetch-fault redirect to the trap vector; phase 1 refetches the start.
   // (an interrupt's redirect rides eb_target -- the irq_take pseudo-op's SYSTEM redirect.)
   assign fe_red_pc  = (iflt_fire | dflt_fire) ? csr_redir_tgt
                     : dflt_replay             ? chk_pc[flt_ckpt]
                     :                           eb_target;
   assign fe_red_seq = iflt_fire ? fe_cur_seq
                     : dflt_roll ? chk_seq[flt_ckpt]
                     : (eb_rseq + 1'b1);

   assign wb_valid = wkv;
   assign wb_pr    = wkp;
endmodule

`default_nettype wire
