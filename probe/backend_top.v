`include "exec_pay.vh"
`include "va_codec.vh"
`default_nettype none

// PROBE_POOL: physregs per shard (default 80 -> NPHYS=320). Shrinking it (e.g.
// -DPROBE_POOL=20) forces aggressive physreg reuse, exposing physreg-lifetime /
// operand-capture bugs in short tests instead of millions of instructions in. The
// freelist needs ARSH=AREGS/SHARDS=16 reserved arch regs/shard, so POOL>=18.
//
// POOL MUST EXCEED AREGS(=64): a dest physreg comes from the DISPATCH lane's shard
// (RF bank = producing lane), while slot-0 carries every solo bundle (CSR/AMO/
// serialize) plus each bundle's first op -- so shard 0 slowly captures the arch
// mappings. With POOL==AREGS all 64 arch regs can map into one shard (observed at
// tiny128 relocate: free=0/64/64/64), and once the machine drains empty with the
// next bundle needing that shard, nothing can ever commit to return a pold ->
// rename deadlocks forever. POOL=80 keeps >=16 free at quiescence: unreachable.
`ifndef PROBE_POOL
 `define PROBE_POOL 80
`endif

// PROBE_IW: issue/shard width -- the ONE width knob. SBITS, PBITS, HW, DCW, CNTW,
// NPHYS (and WAKEN, downstream) all DERIVE from it. Was hardwired 4; default is now 2:
// at probe_clk=66.7 MHz even a perfect IPC=4 is only 267 MIPS (a scalar's reach), so a
// narrower core that closes timing at a higher Fmax wins. Build the 4-wide core with
// -DPROBE_IW=4 (verilator/iverilog) or the RTL default edit (Vivado).
`ifndef PROBE_IW
 `define PROBE_IW 2
`endif

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
  #(parameter IW    = `PROBE_IW,   // issue/shard width -- the ONE knob; all widths below derive
    // physregs/shard (freelist + RF bank depth). Must exceed the arch regs homed per shard
    // (AREGS/SHARDS) by a healthy free margin (~48) or the freelist deadlocks. PROBE_POOL=80
    // covers IW>=2 (ARSH<=32); at IW=1 all 64 arch regs pile into the lone shard (ARSH=64),
    // so scale POOL up. (Only IW=1 exceeds the 80 floor; IW>=2 stay at PROBE_POOL unchanged.)
    parameter POOL  = (`PROBE_POOL >= 64/IW + 48) ? `PROBE_POOL : 64/IW + 48,
    parameter SBITS = ($clog2(IW) < 1) ? 1 : $clog2(IW),   // shard-id width, >=1 (IW=1 = 2^0 still needs a 1b field)
    parameter HW    = 2*IW,          // window halfwords (2*IW = one full 32b bundle/cycle)
    parameter PCW   = 64,
    parameter SEQW  = 8,
    parameter ABITS = 6,
    parameter PBITS = $clog2(POOL)+SBITS,  // {ridx[clog2(POOL)-1:0], shard[SBITS-1:0]}
    parameter SCHED_N = 16,      // CAM RS entries/shard. Sweep @3ns: select path was the
                                 // cap (N12=3.17 N16=4.49ns) until the age compare was
                                 // coarsened (low 4 seqno bits dropped) -> N16=2.55ns,
                                 // RS no longer the limiter (shared scoreboard write is).
    parameter CBITS = 3,             // NCHK=8: at 4 the ring was full 14-23% of boot cycles
    parameter NCHK  = 8,             // (DISP_STATS ccfull) -- too few bundles in flight to
                                     // cover the dispatch->commit latency. All per-ckpt state
                                     // is shallow LUTRAM-class (chk_map 64x8b/shard, snapshots,
                                     // pnpc/pdet, GHR/RAS clones), so 8 deepens arrays without
                                     // touching a critical cone; seq window: 8*IW in-flight
                                     // ops << the +/-128 wrap-compare bound (SEQW=8).
    parameter NPHYS = (1 << SBITS) * POOL,   // pr = {ridx, shard[SBITS-1:0]} spans 2^SBITS*POOL
                                             // (= IW*POOL for power-of-2 IW; sparse/larger for 3,5)
    parameter CKMAX = 1,             // per-bundle. CKMAX>=2 (coarse CPR window growth) has a
                                     // rollback-reopen count-accounting wedge on the ubuntu-mini
                                     // RAM boot -- DEFERRED (repro + notes in commit_ctl /
                                     // project_coarse_checkpoints). CKMAX=1 = proven baseline.
                                     // CKMAX=1 ALSO carries a correctness invariant: the aligner
                                     // makes a SYSTEM op solo in its bundle, so at CKMAX=1 a CSR
                                     // op is alone in its CHECKPOINT and no rollback that targets
                                     // some other op's checkpoint can re-execute it. That matters
                                     // because a CSR write is applied at EX and is NOT undone by a
                                     // rollback, so re-executing `csrrw rd,csr,rd` (a swap) reads
                                     // back the value it just installed -- the mtvec-stranding bug.
                                     // CKMAX>=2 would let a CSR op share a checkpoint with a
                                     // replayable load and reintroduce it; make CSR writes a
                                     // commit-time effect before raising CKMAX.
    parameter DCW   = $clog2(IW+1),  // dispatch count 0..IW (one bundle)
    parameter CNTW  = $clog2(CKMAX+IW+1),  // per-checkpoint count: up to CKMAX (+bundle overshoot)
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
    input  wire [63:0]             mtime,            // free-running CLINT time (Sstc); 0 in device-less TBs
    output wire [63:0]             dbg_timer,        // csr_file timer/irq debug bus (ILA_TIMER; pruned unused)
    output wire [63:0]             dbg_mtvec,        // csr_file mtvec (ILA probe2)
    output wire                    dbg_mtvec_we,     // mtvec write strobe (ILA probe4)
    output wire [63:0]             dbg_lsu,          // full LSU state (ILA probe5; replaces dbg_csrop,
                                                     // whose mtvec-stranding hunt is finished)
    output wire [63:0]             dbg_csrop,        // executing system op {pc,addr,func,is_csr}
    output wire                    dbg_csrop_v,      // ...its strobe (ILA probe6)
    output wire [63:0]             dbg_wedge,        // frontend/dispatch/interrupt state (ILA probe7)
    // Zihpm cache-event pulses from soc_top's D$/I$ (0 in device-less TBs, which have no cache).
    input  wire                    hpm_dc_access, hpm_dc_miss, hpm_ic_access, hpm_ic_miss,
    // data memory port (flat byte-addressable stub; real D$ later). The READ port is a
    // request/response handshake so a multi-cycle D$ can stall: dmem_ren pulses on a fresh
    // dmem_raddr, dmem_rvalid signals dmem_rdata is valid. Tie dmem_rvalid=1 for a
    // zero-latency memory (combinational read) -> bit-exact 1-cycle loads.
    output wire [AW-1:0]           dmem_raddr,
    output wire                    dmem_ren,
    output wire                    dmem_runcached,     // Svpbmt: the read addr is NC/IO (don't cache)
    input  wire [63:0]             dmem_rdata,
    input  wire                    dmem_rvalid,
    output wire                    dmem_wen,
    output wire [AW-1:0]           dmem_waddr,
    output wire [63:0]             dmem_wdata,
    output wire [7:0]              dmem_wmask,
    output wire                    dmem_wuncached,     // Svpbmt: the write addr is NC/IO (flush-around)
    output wire                    dmem_cbo,           // Zicbom/Zicboz: cache-maintenance op at the write port
    output wire                    dmem_cbo_zero,      // cbo.zero: install a zero line
    output wire                    dmem_cbo_keep,      // cbo.clean: writeback but keep line valid
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
`ifdef REDIR_TRACE
   always @(posedge clk) if (fe_red_v) $display("[REDIR] tgt=%h rckpt=%0d chkpc=%h chkseq=%0d rtrap=%b ebtgt=%h ebrseq=%0d cur=%0d",
      fe_red_pc, eb_rckpt, chk_pc[eb_rckpt], chk_seq[eb_rckpt], eb_rtrap, eb_target, eb_rseq, cur);
`endif
   // predictor resolve/training port (exec_bundle -> frontend) + GHR repair strobe
   wire               eb_res_v, eb_res_cbr, eb_res_call, eb_res_ret, eb_res_taken, eb_res_mispred;
   wire [CBITS-1:0]   eb_res_ckpt;
   wire [63:0]        eb_res_tgt;
   wire               bp_rep;

   // ---- frontend: fetch -> decode -> rename ----
   wire [IW-1:0]      r_valid, r_rd_v, r_is_branch, fe_stall;
   wire [IW*SEQW-1:0] r_seq;
   wire [IW*ABITS-1:0] r_rd;
   wire [IW*PBITS-1:0] ps1, ps2, ps3, pdst;
   wire [IW*`PAYW-1:0] r_pay;
   wire [PCW-1:0]     fe_pred_npc;
   wire [CBITS-1:0]   r_ckpt, cur;

   // ---- commit control ----
   wire               cc_commit, cc_rollback, cc_full, cc_create;
   wire [CBITS-1:0]   cc_commit_idx, cc_rollback_idx, cc_committed;
   wire [CNTW-1:0]    cc_commit_count;   // # instructions retiring this cycle (-> minstret)

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
   wire               lsu_devld_v;           // device load wants replay-to-solo (older store shares its ckpt)
   wire [CBITS-1:0]   lsu_devld_ckpt;        // ...the checkpoint THAT LOAD is in (the rollback target)
   wire [3:0]         lsu_dbg_defer;         // {lq_any, sb_any, amo_busy, devrd_pending}
   wire [63:0]        lsu_dbg_lsu;           // full LSU state (ILA probe5)
   wire [IW-1:0]      sch_dbg_any_v, sch_dbg_stuck;
   wire [CNTW-1:0]    cc_dbg_cnt;            // count[committed]: what commit is waiting on
   wire [2:0]         eb_dbg_evap;           // sticky: a deferred op evaporated at EX
   wire               lsu_devld_fire_v;      // a device load fired (ends the device-load solo window)
   reg                devld_solo_v;          // device-load solo replay active (drives solo_all like replay_v)
   initial devld_solo_v = 1'b0;
   reg                ill_v;                 // illegal-instruction fault latched (declared early: gates dispatch)
   reg  [SEQW-1:0]    ill_seq;               // its seqno + checkpoint (set at issue, below)
   reg  [CBITS-1:0]   ill_ckpt;
   initial ill_v = 1'b0;
   wire [IW-1:0]      disp_ready;
   function automatic older_seq;      // a strictly older than b (wrap-safe), as in exec_shard
      input [SEQW-1:0] a, bb; older_seq = ($signed(a - bb) < 0);
   endfunction
   wire               any_valid    = |r_valid;
   // AMO dispatch gap (set/cleared below, after the exec bundle's eb_amo exists):
   // order younger loads behind a dispatched-but-not-yet-executing AMO.
   reg                amo_gap;  reg [SEQW-1:0] amo_gap_seq;
   initial amo_gap = 1'b0;
   // Freeze dispatch while a data page-fault is latched but not yet delivered: it is
   // delivered late (when its checkpoint becomes oldest), and with only NCHK checkpoints
   // wrong-path speculation can wrap the ring and reuse -- thus overwrite -- the faulting
   // checkpoint's chk_seq/chk_pc before delivery. Freezing preserves them. Cannot deadlock:
   // older checkpoints still complete + commit independently of dispatch, so committed
   // advances to the fault's checkpoint and dflt_fire clears the latch.
   // !roll_v: no dispatch in ANY rollback cycle (branch redirect, data-fault replay,
   // fetch fault, device-load replay). A bundle dispatched the same cycle a replay
   // rolls back is wrong-path-by-construction: the hardware squashes it by seqno,
   // but it still runs through the create/freelist path only to be half-cancelled
   // by rollback priority, and the cosim retire FIFO (squash-then-push order) kept
   // it OUT OF PROGRAM ORDER ahead of the replayed refetch -- the frontend flush
   // kills the same bundle anyway, so dispatching it is pure downside. (Previously
   // only !eb_redirect; the replay paths were exposed once the predictor removed
   // the taken-branch bubble ahead of the UART lb's devld_replay.)
   wire               can_dispatch = !cc_full && (&disp_ready) && !(|fe_stall)
                                     && !sb_full && !lq_full && !roll_v && !lsu_dfault_v && !ill_v
                                     && !amo_gap && !cc_stall_barrier;
   wire               disp_fire    = any_valid && can_dispatch;
   wire               accept       = !any_valid || can_dispatch;   // else freeze frontend

   reg  [DCW-1:0]     disp_count;
   integer dc;
   always @* begin
      disp_count = {DCW{1'b0}};
      for (dc = 0; dc < IW; dc = dc + 1) disp_count = disp_count + r_valid[dc];
   end

   // Coarse CPR (approach B): a bundle ENDS its checkpoint if it carries a CTI (branch/jump)
   // or a serialize/CSR/fence/AMO/CBO op. Closing on CTIs keeps ONE CTI per checkpoint, so the
   // per-checkpoint mispredict-reference (pnpc) + predictor train-details (pdet) stay correct
   // and recovery is PRECISE (no replay) -- while the straight-line run between CTIs still
   // coarsens into one checkpoint (window grows ~basic-block-length x). commit_ctl also caps
   // at CKMAX. (Not-taken branches riding along -> per-branch pnpc/pdet, a later step.)
   reg  disp_close;  reg disp_barrier;  integer dcl;
   wire cc_ckpt_open;  wire cc_stall_barrier;
   always @* begin
      disp_close = 1'b0;  disp_barrier = 1'b0;
      for (dcl = 0; dcl < IW; dcl = dcl + 1) begin
         if (r_valid[dcl] & (r_pay[dcl*`PAYW + `PAY_BR]  | r_pay[dcl*`PAYW + `PAY_JMP]
                           | r_pay[dcl*`PAYW + `PAY_SER] | r_pay[dcl*`PAYW + `PAY_CSR]
                           | r_pay[dcl*`PAYW + `PAY_AMO] | r_pay[dcl*`PAYW + `PAY_CBO]
                           | r_pay[dcl*`PAYW + `PAY_FENCEI]))
            disp_close = 1'b1;
         // memory-barrier ops (NON-branch closers): must open a fresh checkpoint so older stores
         // drain first (see commit_ctl). Branches/jumps do NOT need this -- they carry no ordering.
         if (r_valid[dcl] & (r_pay[dcl*`PAYW + `PAY_SER] | r_pay[dcl*`PAYW + `PAY_CSR]
                           | r_pay[dcl*`PAYW + `PAY_AMO] | r_pay[dcl*`PAYW + `PAY_CBO]
                           | r_pay[dcl*`PAYW + `PAY_FENCEI]))
            disp_barrier = 1'b1;
      end
   end

`ifdef DISP_STATS
   // sim-only dispatch/stall attribution: one cause per non-dispatching cycle
   // (else-chain priority) + bundle-size histogram. Dumped every 20M cycles.
   integer st_cyc, st_disp, st_insn, st_nofe, st_icym, st_immu, st_ccfull, st_sbfull,
           st_lqfull, st_roll, st_dflt, st_ill, st_rs, st_festall, st_red;
   integer st_bs [1:4];
   initial begin
      st_cyc=0; st_disp=0; st_insn=0; st_nofe=0; st_icym=0; st_immu=0; st_ccfull=0;
      st_sbfull=0; st_lqfull=0; st_roll=0; st_dflt=0; st_ill=0; st_rs=0; st_festall=0;
      st_red=0; st_bs[1]=0; st_bs[2]=0; st_bs[3]=0; st_bs[4]=0;
   end
   always @(posedge clk) if (!reset) begin
      st_cyc = st_cyc + 1;
      if (disp_fire) begin
         st_disp = st_disp + 1; st_insn = st_insn + disp_count;
         st_bs[disp_count] = st_bs[disp_count] + 1;
      end
      else if (!any_valid) begin                           // frontend supplied no bundle:
         st_nofe = st_nofe + 1;                            // split I$/iMMU starvation from
         if (imem_avail_g == 0) begin                      // decode/redirect bubbles; then
            st_icym = st_icym + 1;                         // split translation vs I$-supply
            if (!immu_ready) st_immu = st_immu + 1;
         end
      end
      else if (roll_v)         st_roll    = st_roll + 1;
      else if (cc_full)        st_ccfull  = st_ccfull + 1; // NCHK ring full
      else if (sb_full)        st_sbfull  = st_sbfull + 1;
      else if (lq_full)        st_lqfull  = st_lqfull + 1;
      else if (lsu_dfault_v)   st_dflt    = st_dflt + 1;
      else if (ill_v)          st_ill     = st_ill + 1;
      else if (!(&disp_ready)) st_rs      = st_rs + 1;     // scheduler: serialize gate / RS full
      else if (|fe_stall)      st_festall = st_festall + 1;
      if (eb_redirect) st_red = st_red + 1;
      if ((st_cyc % 20000000) == 0)
         $display("[DSTAT] cyc=%0d IPC=%f disp%%=%f | noFE%%=%f (icym%%=%f immu%%=%f) roll%%=%f ccfull%%=%f sbfull%%=%f lqfull%%=%f dflt%%=%f ill%%=%f sched%%=%f fe%%=%f | bs1=%0d bs2=%0d bs3=%0d bs4=%0d avgBS=%f | redirs=%0d",
            st_cyc, 1.0*st_insn/st_cyc, 100.0*st_disp/st_cyc,
            100.0*st_nofe/st_cyc, 100.0*st_icym/st_cyc, 100.0*st_immu/st_cyc,
            100.0*st_roll/st_cyc, 100.0*st_ccfull/st_cyc,
            100.0*st_sbfull/st_cyc, 100.0*st_lqfull/st_cyc, 100.0*st_dflt/st_cyc,
            100.0*st_ill/st_cyc, 100.0*st_rs/st_cyc, 100.0*st_festall/st_cyc,
            st_bs[1], st_bs[2], st_bs[3], st_bs[4],
            1.0*st_insn/((st_disp>0)?st_disp:1), st_red);
   end
`endif

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
   wire [PCW-1:0]                imem_ipc;    // PC of the instruction being fetched (fault EPC)
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

   // Valid-DRAM window for the MMU's unbacked-PA access-fault check. Enforced only under
   // cosim (sized to the modeled DDR, so an OOR access faults exactly like simmerv); the
   // FPGA/test default stays fully permissive (base 0, unbounded) -> no behavior change.
`ifdef COSIM_MEM_SIZE_LG2
   localparam [63:0] DRAM_BASE_P = 64'h8000_0000;
   localparam [63:0] DRAM_TOP_P  = 64'h8000_0000 + (64'd1 << `COSIM_MEM_SIZE_LG2);
`else
   localparam [63:0] DRAM_BASE_P = 64'd0;
   localparam [63:0] DRAM_TOP_P  = 64'hFFFF_FFFF_FFFF_FFFF;
`endif

   mmu #(.AW(56), .DRAM_BASE(DRAM_BASE_P), .DRAM_TOP(DRAM_TOP_P)) u_immu
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
   reg  [63:0]        iflt_va;          // faulting VA (-> tval): straddle high half = pc_q+2
   reg  [63:0]        iflt_epc;         // faulting instruction PC (-> epc): pc_q (= iflt_va when no straddle)
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
         pend_iflt <= 1'b1; iflt_va <= imem_va; iflt_epc <= imem_ipc; iflt_cause <= immu_cause;
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
   // ~any_valid: the fault must also wait out any UNDISPATCHED bundle at the rename
   // boundary. Pre-predictor, a frontier fetch PC could only come from an executed
   // redirect, so cc_empty alone proved the fault was for the true next PC. With the
   // BTB, the frontier can be WRONG-PATH-BY-PREDICTION while the mispredicted branch
   // -- whose redirect would drop this fault as stale -- is still sitting uncounted
   // at the boundary; firing then delivers a phantom page fault at the wild predicted
   // target (seen live: a stale VA/PA-aliased BTB entry sent fetch to 0x8000bf7c under
   // Sv39 and the kernel took cause=12 at a bare-physical epc). Letting the bundle
   // dispatch first either redirects (fault dropped) or drains to a genuine fire.
   assign iflt_fire = pend_iflt & cc_empty & ~any_valid & (~replay_v | replay_to_iflt);

   wire [3:0]         dflt_cause;
   wire [63:0]        dflt_epc, dflt_tval;
   // xtrap carries EXCEPTIONS only now (fetch/data page faults). Interrupts are delivered
   // by the injected irq_take pseudo-op through the SYSTEM-op trap path, not here.
   wire               xtrap_v     = iflt_fire | dflt_fire;
   wire               xtrap_intr  = 1'b0;
   wire [3:0]         xtrap_cause = iflt_fire ? iflt_cause : dflt_cause;
   wire [63:0]        xtrap_epc   = iflt_fire ? iflt_epc   : dflt_epc;   // instruction PC
   wire [63:0]        xtrap_tval  = iflt_fire ? iflt_va    : dflt_tval;  // faulting VA

   frontend #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW), .ABITS(ABITS),
              .PBITS(PBITS), .NPHYS(NPHYS), .POOL(POOL), .HPTR($clog2(POOL)), .SBITS(SBITS),
              .NCHK(NCHK), .CBITS(CBITS), .RESET_PC(RESET_PC)) fe
     (.clk(clk), .reset(reset),
      .redirect(fe_red_v), .redirect_pc(fe_red_pc),
      .redirect_seq(fe_red_seq), .solo_all(replay_v | devld_solo_v), .irq_inject(irq_inject),
      .imem_addr(imem_va), .imem_ipc(imem_ipc), .imem_data(imem_data), .imem_avail(imem_avail_g),
      .accept(accept),
      .create(disp_fire), .ckpt_create(cc_create), .commit(cc_commit), .commit_idx(cc_commit_idx),
      .rollback(cc_rollback), .rollback_idx(cc_rollback_idx),
      .res_v(eb_res_v), .res_cbr(eb_res_cbr), .res_call(eb_res_call), .res_ret(eb_res_ret),
      .res_taken(eb_res_taken),
      .res_ckpt(eb_res_ckpt), .res_tgt(eb_res_tgt), .res_rep(bp_rep),
      .r_valid(r_valid), .r_seq(r_seq), .r_rd(r_rd), .r_rd_v(r_rd_v),
      .ps1(ps1), .ps2(ps2), .ps3(ps3), .pdst(pdst),
      .r_is_branch(r_is_branch),
      .r_pay(r_pay), .r_pred_npc(fe_pred_npc),
      .r_ckpt(r_ckpt), .cur(cur), .cur_seq(fe_cur_seq), .stall(fe_stall));

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
`ifdef RN_TRACE
   integer dl_;
   always @(posedge clk) begin
      for (dl_ = 0; dl_ < IW; dl_ = dl_ + 1) if (sched_disp_valid[dl_])
         $display("[RN t=%0t] seq=%0d ck=%0d pdst=%0d pv=%b ps1=%0d ps2=%0d", $time, r_seq[dl_*SEQW+:SEQW],
                  r_ckpt, pdst[dl_*PBITS+:PBITS], r_rd_v[dl_], ps1[dl_*PBITS+:PBITS], ps2[dl_*PBITS+:PBITS]);
      if (roll_v) $display("[ROLL t=%0t] seq=%0d ck=%0d", $time, roll_seq, roll_ckpt);
   end
`endif

   sched_bundle #(.SHARDS(IW), .NPHYS(NPHYS), .PBITS(PBITS), .SEQW(SEQW), .N(SCHED_N),
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
      .iss_ckpt(iss_ckpt), .iss_mem_idx(iss_mem_idx),
      .dbg_any_v(sch_dbg_any_v), .dbg_stuck(sch_dbg_stuck), .iss_pay(iss_pay));

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

   // a divide heading to / running on a shard's divider stalls that shard's issue.
   // q_iss_is_fp closes the same 1-cycle RR window for FP ops: EX has no hold, so an
   // FP op arriving at EX while the CVFPU is busy with the FP op issued one cycle
   // earlier would evaporate (its deferred count never decs -> checkpoint wedge).
   assign busy_to_sched = q_iss_is_mul | q_iss_is_fp | eb_exec_busy;

   // ---- commit control: count by completion (loads at LSU), commit in order ----
   wire               lsu_ld_done;
   wire [CBITS-1:0]   lsu_ld_done_ckpt;
   wire               lsu_st_done;
   wire [CBITS-1:0]   lsu_st_done_ckpt;
   wire               lsu_fp_dirty;       // FP-dest load (FLW/FLD) wrote back -> mstatus.FS Dirty
   wire [IW-1:0]      eb_iss_fp_dirty;    // per-shard issue-time in-core FP writer (-> commit_ctl)
   wire               cc_fp_dirty_commit; // commit_ctl: FS-dirty applied at commit (-> u_csr)
   // ---- data page-fault report from the LSU (-> precise trap, below) ----
   wire [SEQW-1:0]    lsu_dfault_seq;
   wire [CBITS-1:0]   lsu_dfault_ckpt;
   wire [3:0]         lsu_dfault_cause;
   wire [AW-1:0]      lsu_dfault_tval;
   // Owner-guard for the deferred LOAD count decrement. The original leaked-writeback bug is a
   // squashed load whose r_v writeback (and matching ld_done count decrement) fires a cycle or
   // two after r_kill's 1-cycle rollback pulse -- so ld_done can decrement a reused checkpoint's
   // count. Gate it by exec_bundle's per-shard owner-check (eb_ewb_ok), exactly what already
   // guards the RF write: ld_done is aligned with the load's writeback (same cycle; x0->phys0 =>
   // ewb_ok=1), and a legit load matches its owner, so there is no false-suppress / no deadlock.
   // div/fp need NO count gate: mul3/divider .abort resets the unit (no ddone/mdone) and FP's
   // fp_zomb persistently drains a squashed op (no fp_done) -- both cancel at the source. Their
   // done pulses are also combinational while the writeback is registered (1 cycle later) and
   // fire for x0-dest ops, so ewb_ok would misalign and false-suppress them (=> deadlock).
   wire [IW-1:0]      eb_ewb_ok;
   // The hazard this guards is a load whose ld_done lands AFTER its checkpoint index was reused,
   // decrementing the NEW occupant's count. The physreg owner-check (eb_ewb_ok) is the right gate
   // for the RF WRITE -- a leaked writeback must not clobber a reallocated physreg -- but it is
   // the wrong one for the COUNT: whether the physreg is still owned says nothing about whether
   // this checkpoint still needs its decrement, and it false-suppressed a legitimate load. One
   // eaten decrement leaves count[ckpt] stuck at 1 forever -> commit stalls -> cc_full -> the
   // frontend freezes with the LQ/SB/units/RS all empty (measured on hardware: exactly one
   // suppression, exactly one stuck count).
   // Test the ACTUAL condition instead: chk_seq[c] is checkpoint c's first seqno, rewritten when
   // the index is reused. A load older than its checkpoint's current start is therefore a
   // late pulse for a REUSED instance -> drop it. A live load is never older than its own
   // checkpoint's start, so it can never be false-suppressed.
   wire               ld_done_stale = older_seq(lsu_ld_wb_seq, chk_seq[lsu_ld_done_ckpt]);
   wire               lsu_ld_done_g = lsu_ld_done & ~ld_done_stale;
   wire               ld_done_supp  = lsu_ld_done & ld_done_stale;
   reg  [3:0]         ld_supp_cnt; initial ld_supp_cnt = 4'd0;
   always @(posedge clk) if (ld_done_supp && ld_supp_cnt != 4'hf) ld_supp_cnt <= ld_supp_cnt + 4'd1;
   commit_ctl #(.NCHK(NCHK), .CBITS(CBITS), .IW(IW), .CKMAX(CKMAX), .CNTW(CNTW), .DCW(DCW)) cc
     (.clk(clk), .reset(reset), .cur(cur),
      .disp_fire(disp_fire), .disp_count(disp_count), .disp_close(disp_close),
      .irq_req(irq_inject & accept), .solo(replay_v | devld_solo_v),
      .barrier(disp_barrier), .stall_barrier(cc_stall_barrier),
      .iss_valid(q_iss_valid), .iss_is_load(q_iss_defer), .iss_is_div(q_iss_is_mul),
      .iss_is_fp(q_iss_is_fp), .fp_done(eb_fp_done), .fp_done_ckpt(eb_fp_done_ckpt), .iss_ckpt(q_iss_ckpt),
      .iss_fp_dirty(eb_iss_fp_dirty),
      .ld_done(lsu_ld_done_g), .ld_done_ckpt(lsu_ld_done_ckpt), .ld_fp_dirty(lsu_fp_dirty),
      .st_done(lsu_st_done), .st_done_ckpt(lsu_st_done_ckpt),
      .div_done(eb_div_done), .div_done_ckpt(eb_div_done_ckpt),
      .redirect(roll_v), .redirect_ckpt(roll_ckpt),
      .create(cc_create), .ckpt_open(cc_ckpt_open), .empty(cc_empty),
      .commit(cc_commit), .commit_idx(cc_commit_idx),
      .rollback(cc_rollback), .rollback_idx(cc_rollback_idx),
      .committed_idx(cc_committed), .commit_count(cc_commit_count),
      .fp_dirty_commit(cc_fp_dirty_commit), .dbg_cnt(cc_dbg_cnt), .full(cc_full));

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
   wire [SEQW-1:0]    lsu_ld_wb_seq;
   wire [IW*SEQW-1:0] wkq;          // per-lane writeback seqno (cosim seqno-matched capture)
   // EX-stage LSU control (from exec_bundle, aligned with eb_agu/eb_stdata)
   wire [IW-1:0]      ex_valid, ex_mem, ex_store, ex_msigned, ex_fp;
   wire [IW-1:0]      ex_cbo, ex_cbo_zero, ex_cbo_keep;
   wire [IW*SEQW-1:0] ex_seq;
   wire [IW*CBITS-1:0] ex_ckpt;
   wire [IW*MIDXW-1:0] ex_mem_idx;
   wire [IW*2-1:0]    ex_msize;

   // Zihpm event pulses -> csr_file (mhpmcounterN counts its mhpmeventN-selected one):
   // [0]load [1]store (LSU completions) [2]redirect (pipe flush/branch mispredict)
   // [3]dc-access [4]dc-miss [5]ic-access [6]ic-miss (D$/I$ line lookups, from soc_top).
   wire [6:0] hpm_ev = {hpm_ic_miss, hpm_ic_access, hpm_dc_miss, hpm_dc_access,
                        roll_v, lsu_st_done, lsu_ld_done};

   exec_bundle #(.SHARDS(IW), .SBITS(SBITS), .PBITS(PBITS), .NPHYS(NPHYS), .POOL(POOL),
                 .IDXB($clog2(POOL)), .SEQW(SEQW), .CBITS(CBITS), .MIDXW(MIDXW)) eb
     (.clk(clk), .reset(reset),
      .iss_valid(q_iss_valid), .iss_seq(q_iss_seq), .iss_pdst(q_iss_pdst),
      .iss_pdst_v(q_iss_pdst_v), .iss_ps1(q_iss_ps1), .iss_ps2(q_iss_ps2), .iss_ps3(q_iss_ps3),
      .iss_ckpt(q_iss_ckpt), .iss_mem_idx(q_iss_mem_idx), .iss_pay(q_iss_pay),
      .squash(roll_v), .squash_seq(roll_seq),
      .exec_busy(eb_exec_busy), .dbg_evap(eb_dbg_evap), .div_done(eb_div_done), .div_done_ckpt(eb_div_done_ckpt),
      .fp_done(eb_fp_done), .fp_done_ckpt(eb_fp_done_ckpt),
      .iss_fp_dirty(eb_iss_fp_dirty), .fp_dirty_commit(cc_fp_dirty_commit),
      .lsu_wb_v(lsu_ld_wb_v), .lsu_wb_owner(lsu_ld_wb_owner),
      .lsu_wb_pr(lsu_ld_wb_pdst), .lsu_wb_val(lsu_ld_wb_val), .lsu_wb_seq(lsu_ld_wb_seq),
      .wb_busy(eb_wb_busy),
      .wb_valid(wkv), .wb_pr(wkp), .wb_val(wb_val), .wb_seq(wkq), .ewb_ok(eb_ewb_ok),
      .ex_valid(ex_valid), .ex_seq(ex_seq), .ex_ckpt(ex_ckpt), .ex_mem_idx(ex_mem_idx),
      .ex_mem(ex_mem), .ex_store(ex_store), .ex_fp(ex_fp), .ex_msize(ex_msize), .ex_msigned(ex_msigned),
      .ex_cbo(ex_cbo), .ex_cbo_zero(ex_cbo_zero), .ex_cbo_keep(ex_cbo_keep),
      .agu_addr(eb_agu), .st_data(eb_stdata),
      .ex_amo(eb_amo), .ex_amo_func(eb_amo_func), .ex_amo_pdst(eb_amo_pdst),
      .disp_v(disp_fire), .disp_ckpt(r_ckpt), .disp_pnpc(fe_pred_npc),
      .res_v(eb_res_v), .res_cbr(eb_res_cbr), .res_call(eb_res_call), .res_ret(eb_res_ret),
      .res_taken(eb_res_taken),
      .res_ckpt(eb_res_ckpt), .res_tgt(eb_res_tgt), .res_mispred(eb_res_mispred),
      .redirect(eb_redirect), .redirect_target(eb_target),
      .redirect_seq(eb_rseq), .redirect_ckpt(eb_rckpt), .redirect_is_trap(eb_rtrap),
      .ifence(ifence),
      .mmu_satp(mmu_satp), .mmu_priv(mmu_priv), .mmu_dpriv(mmu_dpriv),
      .mmu_sum(mmu_sum), .mmu_mxr(mmu_mxr), .mmu_flush(mmu_flush), .fs_off(eb_fs_off),
      .xtrap_v(xtrap_v), .xtrap_intr(xtrap_intr), .xtrap_cause(xtrap_cause),
      .xtrap_epc(xtrap_epc), .xtrap_tval(xtrap_tval),
      .hw_ip(hw_ip), .mtime(mtime), .retire_cnt({{(6-CNTW){1'b0}}, cc_commit_count}), .hpm_ev(hpm_ev),
      .irq_v(csr_irq_v), .irq_cause(csr_irq_cause), .dbg_timer(dbg_timer), .dbg_mtvec(dbg_mtvec), .dbg_mtvec_we(dbg_mtvec_we), .dbg_csrop(dbg_csrop), .dbg_csrop_v(dbg_csrop_v),
      .csr_redir_v(csr_redir_v), .csr_redir_tgt(csr_redir_tgt));

   // ---- AMO dispatch gap (reg declared at the dispatch gate) ----
   // The LSU's load-select gate (ast==A_IDLE & ~amo_v) only sees an AMO at/after EX.
   // A younger load dispatched in the gap between the AMO's dispatch and its
   // serialized (oldest-only) issue can be selected while ast is still A_IDLE and
   // read pre-RMW memory. NCHK=4 masked this by accident -- the shallow ring could
   // not dispatch the load's span until the AMO was already executing; NCHK=8
   // dispatches it ~4 spans earlier (rv64ua test5: the lw after amoadd read stale
   // memory). Freeze dispatch from the AMO's own dispatch (it is solo, slot 0)
   // until it reaches EX -- from there the LSU gate owns the ordering. AMOs are
   // already serialize-when-oldest, so the frozen window is the pipe drain they
   // pay anyway.
   always @(posedge clk) begin
      if (reset)                                        amo_gap <= 1'b0;
      else if (disp_fire & r_pay[`PAY_AMO])             begin amo_gap <= 1'b1; amo_gap_seq <= r_seq[SEQW-1:0]; end
      else if (|eb_amo)                                 amo_gap <= 1'b0;  // at EX: LSU gate takes over
      else if (roll_v & ($signed(roll_seq - amo_gap_seq) < 0)) amo_gap <= 1'b0;  // the AMO itself was squashed
   end

   // ---- LSU execute-port drive (EX stage: bypassed AGU/store-data + EX control) ----
   wire [IW-1:0]      exe_st_v, exe_ld_v;
   wire [IW*SBI-1:0]  exe_st_idx;
   wire [IW*LQI-1:0]  exe_ld_idx;
   wire [IW*AW-1:0]   exe_st_addr, exe_ld_addr;
   wire [IW*64-1:0]   exe_st_data;
   wire [IW*4-1:0]    exe_st_nb, exe_ld_nb;
   wire [IW-1:0]      exe_ld_sgn, exe_ld_fp;
   wire [IW-1:0]      exe_st_cbo, exe_st_cbo_zero, exe_st_cbo_keep;
   generate for (gi = 0; gi < IW; gi = gi + 1) begin : exd
      assign exe_st_v[gi] = ex_valid[gi] & ex_mem[gi] &  ex_store[gi] & ~ex_fp_dis[gi];
      assign exe_st_cbo[gi]      = ex_cbo[gi];
      assign exe_st_cbo_zero[gi] = ex_cbo_zero[gi];
      assign exe_st_cbo_keep[gi] = ex_cbo_keep[gi];
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
         .SBDEPTH(SBDEPTH), .SBI(SBI), .LQDEPTH(LQDEPTH), .LQI(LQI),
         .DRAM_BASE(DRAM_BASE_P), .DRAM_TOP(DRAM_TOP_P)) u_lsu
     (.clk(clk), .reset(reset),
      .disp_fire(disp_fire), .disp_is_load(dl_is_load), .disp_is_store(dl_is_store),
      .disp_seq(r_seq), .disp_ckpt(disp_ckpt), .disp_pdst(pdst),
      .disp_sb_idx(disp_sb_idx), .disp_lq_idx(disp_lq_idx),
      .sb_full(sb_full), .lq_full(lq_full),
      .exe_st_v(exe_st_v), .exe_st_idx(exe_st_idx), .exe_st_addr(exe_st_addr),
      .exe_st_data(exe_st_data), .exe_st_nb(exe_st_nb),
      .exe_st_cbo(exe_st_cbo), .exe_st_cbo_zero(exe_st_cbo_zero), .exe_st_cbo_keep(exe_st_cbo_keep),
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
      .devld_v(lsu_devld_v), .devld_ckpt(lsu_devld_ckpt), .devld_fire_v(lsu_devld_fire_v), .dbg_defer(lsu_dbg_defer), .dbg_lsu(lsu_dbg_lsu),
      .st_done(lsu_st_done), .st_done_ckpt(lsu_st_done_ckpt), .sb_empty(dmem_idle),
      .mem_raddr(dmem_raddr), .mem_ren(dmem_ren), .mem_runcached(dmem_runcached),
      .mem_rdata(dmem_rdata), .mem_rvalid(dmem_rvalid),
      .mem_wen(dmem_wen), .mem_waddr(dmem_waddr), .mem_wdata(dmem_wdata), .mem_wmask(dmem_wmask),
      .mem_wuncached(dmem_wuncached),
      .mem_cbo(dmem_cbo), .mem_cbo_zero(dmem_cbo_zero), .mem_cbo_keep(dmem_cbo_keep),
      .mem_wready(dmem_wready),
      .wb_busy(eb_wb_busy),
      .ld_wb_v(lsu_ld_wb_v), .ld_wb_pdst(lsu_ld_wb_pdst), .ld_wb_owner(lsu_ld_wb_owner),
      .ld_wb_val(lsu_ld_wb_val), .ld_wb_seq(lsu_ld_wb_seq), .ld_done(lsu_ld_done), .ld_done_ckpt(lsu_ld_done_ckpt),
      .fp_dirty(lsu_fp_dirty),
      .commit(cc_commit), .commit_idx(cc_commit_idx), .committed(cc_committed),
      .rollback(roll_v), .rollback_seq(roll_seq), .dfault_taken(dflt_fire));

   // ---- per-checkpoint base PC/seq (precise data-fault trap epc + squash boundary) ----
   reg  [PCW-1:0]  chk_pc  [0:NCHK-1];
   reg  [SEQW-1:0] chk_seq [0:NCHK-1];
   wire [PCW-1:0]  disp_base_pc = r_pay[`PAY_PC];   // slot-0 PC = the bundle's oldest op
   always @(posedge clk) if (cc_ckpt_open) begin   // only the checkpoint's FIRST bundle sets its start
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

   // ~eb_redirect: a same-cycle EX redirect can come from an OLDER op whose checkpoint
   // already committed -- a serialized CSR write counts at ISSUE, so its ckpt commits
   // 1-2 cycles before its EX-time redirect (do_dxchg/do_satp/do_sfence) asserts. In
   // that window `committed` reaches the faulting op's ckpt and a latched fault whose
   // verdict is STALE (e.g. a store check-translated under pre-csrw sstatus.SUM=0)
   // would deliver first (dflt_roll outranks eb below). Defer one cycle: an older
   // redirect then squashes+clears the latch (lsu df_v rollback arm) and the refetched
   // op re-translates; a younger redirect leaves the latch set and the fault delivers
   // next cycle. Same guard devld_replay already carries.
   assign dflt_ready  = flt_v & ~iflt_fire & ~eb_redirect;
   // already first in its bundle (no older siblings to commit) -> precise directly, no replay
   wire   dflt_solo   = (flt_seq == chk_seq[flt_ckpt]);
   assign dflt_replay = dflt_ready & ~dflt_solo & ~replay_v;   // phase 1 (mid-bundle fault only)
   assign dflt_fire   = dflt_ready & ( dflt_solo |  replay_v); // phase 2, or direct when already solo
   assign dflt_roll   = dflt_ready;                 // any delivery/replay rolls back the same way
   assign dflt_cause = flt_cause;
   assign dflt_epc   = chk_pc[flt_ckpt];
   assign dflt_tval  = flt_tval;

   // device-load replay-to-solo (phase-1-style, NO trap): the LSU reports a device load that
   // either is still speculative or shares its checkpoint with an older store. Roll back to
   // THAT LOAD'S checkpoint (lsu_devld_ckpt) and refetch one-op-per-bundle (devld_solo_v ->
   // solo_all), so an older store lands in an earlier committable+drainable checkpoint and the
   // now-separated load reads the post-store device state.
   // The target MUST be the load's own checkpoint, exactly like the data-fault path's flt_ckpt.
   // It used to be chk_*[cc_committed] on the assumption that the load always sits in the oldest
   // live checkpoint -- true only for the shares-with-older-store case. For a merely speculative
   // load (lsu.v `~ld_committed`) the load is in a YOUNGER checkpoint, so rewinding to the oldest
   // live one re-executed everything in between. A serializing CSR op there had already applied
   // its CSR write (applied at EX, NOT undone by rollback) while the rollback restored its source
   // register, so OpenSBI's `csrrw a2,mtvec,a2` probe install re-ran and swapped a2 against the
   // value it had just installed: a2 came back holding __sbi_expected_trap, the paired restore
   // wrote THAT to mtvec, and every later S-mode ecall was swallowed -- SBI calls silently
   // no-op'd, the timer was never re-armed, and the Ubuntu boot died with the core still running.
   // Lower priority than faults/fetch-faults/branch redirects (they reshape the pipe anyway).
   wire devld_replay = lsu_devld_v & ~devld_solo_v & ~replay_v & ~dflt_ready
                       & ~iflt_fire & ~eb_redirect & ~ill_v & ~lsu_dfault_v;

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
   assign irq_inject = csr_irq_v & ~inject_inflight & ~replay_v & ~devld_solo_v & ~pend_iflt & ~lsu_dfault_v
                       & ~ill_v & ~eb_redirect & ~dflt_replay & ~dflt_fire & ~iflt_fire & ~roll_v;
   always @(posedge clk) begin
      if (reset)                    inject_inflight <= 1'b0;
      else if (irq_inject & accept) inject_inflight <= 1'b1;   // pseudo-op entered the pipe
      else if (roll_v | ~csr_irq_v) inject_inflight <= 1'b0;   // squashed/delivered/cleared
   end
`ifdef IRQDBG
   // dbg_cyc MUST be 64-bit: as a 32-bit integer it wrapped negative at c=2.1B and the
   // unsigned comparison against the unsized `IRQDBG_T0 literal read as always-true --
   // the per-rollback ROLL print then produced ~1GB/B-cycles of log (2x 8.5GiB overnight).
   // ROLL is also gated to inject_inflight: only rollbacks that can interact with an
   // in-flight interrupt pseudo-op matter for interrupt debugging.
   reg [63:0] dbg_cyc; initial dbg_cyc = 0;
   wire dbgw = (dbg_cyc > `IRQDBG_T0);
   always @(posedge clk) begin
      dbg_cyc <= dbg_cyc + 64'd1;
      if (dbgw & irq_inject & accept)
         $display("[IRQD] INJ c=%0d pc=%h cause=%0d cur=%0d committed=%0d", dbg_cyc, imem_ipc,
                  csr_irq_cause, cur, cc_committed);
      if (dbgw & inject_inflight & ~csr_irq_v & ~roll_v)
         $display("[IRQD] CLR-cause-gone c=%0d (pseudo-op in flight, irq deasserted)", dbg_cyc);
      if (dbgw & roll_v & inject_inflight)
         $display("[IRQD] ROLL c=%0d eb=%b iflt=%b dfltR=%b dfltF=%b devld=%b seq=%0d ck=%0d infl=%b",
                  dbg_cyc, eb_redirect, iflt_fire, dflt_replay, dflt_fire, devld_replay,
                  roll_seq, roll_ckpt, inject_inflight);
   end
`endif

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

   // device-load solo window: set on the replay, cleared when a device load fires solo. (A
   // device load needing replay is, by construction, not firing this cycle, so the two never
   // collide; clear-first matches replay_v's shape.)
   //
   // ALSO cleared by any LATER rollback: the window is armed for a specific refetched
   // load, and if that load gets squashed (a mispredict inside the window) no device
   // load may ever fire again on the new path -- the window then latches FOREVER, and
   // since irq_inject is gated ~devld_solo_v, ALL interrupt delivery is vetoed for the
   // rest of time. Seen on FPGA: /init wedged on its first console write with the ILA
   // showing UART src_level=1, PLIC pending[10]=1, seip=1 held, and the core never
   // claiming -- the injected-pseudo-op path was permanently blocked. A cleared window
   // re-arms cleanly: the (still-live) device load re-requests devld_replay when
   // re-selected, and older work commits between arms, so no livelock. (~devld_replay
   // excludes the ARMING pulse itself, which drives roll_v the same cycle.)
   always @(posedge clk) begin
      if (reset)                        devld_solo_v <= 1'b0;
      else if (lsu_devld_fire_v)        devld_solo_v <= 1'b0;
      else if (roll_v & ~devld_replay)  devld_solo_v <= 1'b0;
      else if (devld_replay)            devld_solo_v <= 1'b1;
   end

   // unified redirect distribution. A fetch fault fires only when empty, so its rollback
   // is a no-op functionally but keeps the frontend flush paired with a rename rollback
   // (decode_rename restores its map on rollback) -- an unpaired flush leaves the map/
   // checkpoint state stale (count[] -> X). Roll back to the committed (== open) ckpt.
   assign roll_v     = eb_redirect | dflt_roll | iflt_fire | devld_replay;
   // GHR LSB repair (plan B2): only when THIS rollback is the resolved cond-branch's
   // own mispredict. Approach B: CTIs close their checkpoint, so a branch is the last op
   // of its checkpoint -> recovery stays PRECISE (roll to eb_rckpt+1, redirect to eb_target),
   // no replay. All other rollback causes (traps, faults, replays) restore verbatim.
   assign bp_rep     = eb_res_v & eb_res_cbr & eb_res_mispred
                     & ~iflt_fire & ~dflt_roll & ~devld_replay;
   assign roll_seq   = iflt_fire    ? fe_cur_seq
                     : dflt_roll    ? (chk_seq[flt_ckpt]    - 1'b1)
                     : devld_replay ? (chk_seq[lsu_devld_ckpt] - 1'b1) : eb_rseq;
   assign roll_ckpt  = iflt_fire    ? cc_committed
                     : dflt_roll    ? flt_ckpt
                     : devld_replay ? lsu_devld_ckpt : rb_idx;
   assign fe_red_v   = roll_v | iflt_fire;
   // phase 2 / fetch-fault redirect to the trap vector; phase 1 (data fault OR device-load
   // replay) refetches the checkpoint start.
   // (an interrupt's redirect rides eb_target -- the irq_take pseudo-op's SYSTEM redirect.)
   assign fe_red_pc  = (iflt_fire | dflt_fire) ? csr_redir_tgt
                     : dflt_replay             ? chk_pc[flt_ckpt]
                     : devld_replay            ? chk_pc[lsu_devld_ckpt]
                     :                           eb_target;
   assign fe_red_seq = iflt_fire    ? fe_cur_seq
                     : dflt_roll    ? chk_seq[flt_ckpt]
                     : devld_replay ? chk_seq[lsu_devld_ckpt]
                     : (eb_rseq + 1'b1);

   assign wb_valid = wkv;
   assign wb_pr    = wkp;

`ifdef PROBE_COSIM
   // ============================ cosim retire stream ============================
   // Reconstruct an in-order architectural retire stream from the OoO/CPR backend
   // and hand each committed instruction (and each trap) to simmerv via a DPI call
   // (probe_cosim.cpp). Records per-(ckpt,slot) info at dispatch, fills the result
   // value at writeback (unique phys-dest match), and emits at commit in slot
   // order; a trap is emitted when csr_file delivers one (its op is the oldest live
   // checkpoint, so interleaving with commits preserves program order). next_pc is
   // computed C-side by buffering one retire (= the next retire's pc). VERIFY-ONLY.
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
      input byte    unsigned seip_v);

   // In-order retire FIFO (decouples emission from commit: the probe commits ALU
   // ops on the issue count, which can fire a cycle or two BEFORE the value writes
   // back -- so we cannot read rd_val at commit). Push at dispatch (program order),
   // fill rd_val at writeback (unique phys-dest match), mark committed at commit,
   // truncate the tail by seqno on a squash, and emit the head only once its value
   // is ready (or it has no dest). A trap pushes a ready entry so it stays ordered
   // behind older committed instructions. Blocking assignments => sequential FIFO.
   localparam QN = 40;
   reg  [SEQW-1:0]  q_seq  [0:QN-1];
   reg  [CBITS-1:0] q_ck   [0:QN-1];
   reg  [63:0]      q_pc   [0:QN-1];
   reg  [31:0]      q_insn [0:QN-1];
   reg  [1:0]       q_rk   [0:QN-1];
   reg  [4:0]       q_ri   [0:QN-1];
   reg  [PBITS-1:0] q_prd  [0:QN-1];
   reg  [1:0]       q_prv  [0:QN-1];
   reg  [63:0]      q_mepc [0:QN-1];
   reg  [63:0]      q_val  [0:QN-1];
   reg              q_vok  [0:QN-1];
   reg              q_cmt  [0:QN-1];
   reg              q_trap [0:QN-1];
   reg  [63:0]      q_cause[0:QN-1];
   reg  [63:0]      q_tval [0:QN-1];
   // mepc reported in RETIRE ORDER: updated only when a mepc-changing instruction actually
   // emits (a trap -> its cot_mepc, or a CSR write to mepc -> the landed value). A YOUNGER
   // op's out-of-order mepc write (e.g. an ebreak/interrupt that becomes oldest right after
   // an older op commits) cannot perturb it until that op retires in order -- so an older
   // normal retire never leaks the younger trap's mepc (the cosim harness read-race).
   reg  [63:0]      mepc_retire; initial mepc_retire = 64'd0;
   reg  [PBITS-1:0] q_ps1  [0:QN-1];   // renamed source physregs (rename-correctness check)
   reg  [PBITS-1:0] q_ps2  [0:QN-1];
   reg  [4:0]       q_rs1  [0:QN-1];   // source arch regs (from the insn fields)
   reg  [4:0]       q_rs2  [0:QN-1];
   reg  [SBI-1:0]   q_sbidx[0:QN-1];   // store's SB slot (to read back sb_data at retire)
   integer          qn; initial qn = 0;
   reg              cot_found;

   // Rename-correctness invariant: a retiring op's source physreg (ps) MUST equal the
   // arch reg's current physreg (last committed writer). Tracked in retire (= program)
   // order, so it's exact with no false positives. Fires at the BAD RENAME (e.g. the
   // store reading a wrong-mapped x8) -- pinpointing the rename/chk_map bug locally.
   reg [PBITS-1:0]  arch_phys [0:63];
   integer          api; initial for (api = 0; api < 64; api = api + 1) arch_phys[api] = api[PBITS-1:0];
   // Shadow-ARF VALUE: last committed writer's result per arch reg. REN-CHK proves the physreg
   // MAPPING is right; STVAL-CHK (below) proves the physreg's VALUE is right -- catching a
   // stale/lost PRF value under a correct mapping (the class the freelist/rename invariants can't
   // see; this is what caught the original boot corruption on main at c=33.95B).
   reg [63:0]       arch_val  [0:63];
   integer          avi; initial for (avi = 0; avi < 64; avi = avi + 1) arch_val[avi] = 64'd0;
   reg [6:0]        ck_op;  reg ck_u1, ck_u2;  integer ck_da;

   // ---- trap-fire fields (combinational, sampled at the delivery edge) ----
   wire        cot_fire  = eb.u_csr.trap_v;
   wire [63:0] cot_cause = eb.u_csr.trap_cause;
   wire [63:0] cot_epc   = eb.u_csr.trap_epc;      // trapping/interrupted PC
   wire [63:0] cot_tval  = eb.u_csr.trap_tval;
   wire        cot_to_s  = eb.u_csr.trap_to_s;
   wire [63:0] cot_mepc  = cot_to_s ? `VA_UNPACK40(eb.u_csr.mepc) : cot_epc;   // mepc after retire
   wire [1:0]  cot_prv   = eb.u_csr.priv;          // privilege BEFORE the trap
   wire        cot_intr  = cot_cause[63];
   // instruction-side faults retire no instruction (insn=0), like async interrupts
   wire        cot_ifault = ~cot_intr & ((cot_cause[5:0]==6'd0) | (cot_cause[5:0]==6'd1)
                                       | (cot_cause[5:0]==6'd12));
   // the trapping op's insn: look it up by PC in the (pre-squash) FIFO
   reg  [31:0] cot_insn; integer tlk;
   always @* begin
      cot_insn = 32'd0;
      if (~cot_intr & ~cot_ifault)
         for (tlk = 0; tlk < QN; tlk = tlk + 1)
            if ((tlk < qn) && (q_pc[tlk] == cot_epc)) cot_insn = q_insn[tlk];
   end

   integer fi, fl, cut;
   always @(posedge clk) begin
      if (reset) begin qn = 0; mepc_retire = 64'd0; end
      else begin
         // ISS-CHK: the registered issued source physregs (what execute reads) must equal
         // the dispatched ps for the same seqno -- catches the scheduler corrupting/swapping
         // an op's source payload between rename and execute, which the dispatch-time REN-CHK
         // cannot see. (ps==0/p0 for unused sources matches trivially -> no opcode gating.)
         for (fl = 0; fl < IW; fl = fl + 1) if (q_iss_valid[fl])
            for (fi = 0; fi < QN; fi = fi + 1)
               if ((fi < qn) && !q_trap[fi] && (q_seq[fi] == q_iss_seq[fl*SEQW +: SEQW])) begin
                  if (q_iss_ps1[fl*PBITS +: PBITS] != q_ps1[fi])
                     $display("[%0t] *** ISS-CHK seq=%0d pc=%h iss_ps1=%0d != disp_ps1=%0d", $time,
                        q_iss_seq[fl*SEQW +: SEQW], q_pc[fi], q_iss_ps1[fl*PBITS +: PBITS], q_ps1[fi]);
                  if (q_iss_ps2[fl*PBITS +: PBITS] != q_ps2[fi])
                     $display("[%0t] *** ISS-CHK seq=%0d pc=%h iss_ps2=%0d != disp_ps2=%0d", $time,
                        q_iss_seq[fl*SEQW +: SEQW], q_pc[fi], q_iss_ps2[fl*PBITS +: PBITS], q_ps2[fi]);
               end
         // 0. emit: drain the head while committed AND value-ready (or no dest/trap).
         //    Runs FIRST so it acts on entries committed in a PRIOR cycle -- a 1-cycle
         //    lag past commit, by which time a CSR op's mepc/csr write has landed, so
         //    the LIVE mepc read here is the correct mepc-after-retire for normal ops.
         //    GUARD: a trap being delivered THIS cycle (cot_fire) stamps the head entry
         //    (pc==cot_epc) into a trap retire in section 5 below -- which runs AFTER this
         //    emit. Without the guard, an interrupt pseudo-op (OP_IRQ, rk=0) at the head
         //    drains RAW (trap=0/cause=0) before the stamp, leaking a bogus instruction
         //    retire (the cosim interrupt-timing divergence). Hold the emit so the stamp wins.
         for (fl = 0; fl < QN; fl = fl + 1)
            if (qn > 0 && q_cmt[0] && (q_trap[0] || q_rk[0] == 2'd0 || q_vok[0])
                && ~(cot_fire & ~q_trap[0] & (q_pc[0] == cot_epc))) begin
            // An un-trapped interrupt pseudo-op (OP_IRQ, insn 0x7f000073) reaching the head is
            // a SUPERSEDED GHOST: a real interrupt's cot_fire stamp converts its own OP_IRQ to
            // trap=1/insn=0 (section 5) before it ever drains here, so only orphans stay
            // un-trapped. Such an orphan is a speculative injection whose checkpoint committed
            // (count-at-issue) but whose trap delivered at a different PC (the latent CPR
            // commit-count-orphan). It can't be squashed (committed) and the real instruction
            // at its PC is separately present, so DROP it: advance the FIFO without emitting a
            // bogus retire. (Cosim-side workaround for the IRQ-orphan; the underlying DUT-vs-
            // harness question is deferred -- see project_cosim_irq_orphan.)
            if (q_insn[0] == 32'h7f000073 && ~q_trap[0]) begin
               // drop: fall through to the shift below, no probe_retire / arch_phys update
`ifdef IRQDBG
               $display("[IRQD] ORPHAN-DROP c=%0d seq=%0d ck=%0d pc=%h qn=%0d", dbg_cyc,
                        q_seq[0], q_ck[0], q_pc[0], qn);
`endif
            end else begin
               // rename-correctness check (integer-source ops only; skip FP-source + traps)
               if (!q_trap[0]) begin
                  ck_op = q_insn[0][6:0];
                  // integer-rs1 ops: OP/OP32/OP-IMM/IMM32/LOAD/STORE/BRANCH/JALR/AMO
                  ck_u1 = (ck_op==7'h33)|(ck_op==7'h3b)|(ck_op==7'h13)|(ck_op==7'h1b)
                        | (ck_op==7'h03)|(ck_op==7'h23)|(ck_op==7'h63)|(ck_op==7'h67)|(ck_op==7'h2f);
                  // integer-rs2 ops: OP/OP32/STORE/BRANCH/AMO
                  ck_u2 = (ck_op==7'h33)|(ck_op==7'h3b)|(ck_op==7'h23)|(ck_op==7'h63)|(ck_op==7'h2f);
                  if (ck_u1 && (arch_phys[q_rs1[0]] != q_ps1[0]))
                     $display("[%0t] *** REN-CHK pc=%h insn=%h rs1=x%0d ps1=%0d != live=%0d",
                        $time, q_pc[0], q_insn[0], q_rs1[0], q_ps1[0], arch_phys[q_rs1[0]]);
                  if (ck_u2 && (arch_phys[q_rs2[0]] != q_ps2[0]))
                     $display("[%0t] *** REN-CHK pc=%h insn=%h rs2=x%0d ps2=%0d != live=%0d",
                        $time, q_pc[0], q_insn[0], q_rs2[0], q_ps2[0], arch_phys[q_rs2[0]]);
                  // STVAL-CHK: a full-word store's committed data (read back from the SB at
                  // retire) MUST equal its rs2's shadow-ARF VALUE (last committed writer). Fires
                  // at a wrong PRF value under a correct map -- the store-blind-spot bug that
                  // survives REN-CHK + the freelist assertions.
                  if ((ck_op==7'h23) && (q_insn[0][14:12]==3'b011) && (q_rs2[0]!=5'd0)
                      && (u_lsu.sb_data[q_sbidx[0]] !== arch_val[{1'b0,q_rs2[0]}]))
                     $display("[%0t] *** STVAL-CHK pc=%h insn=%h rs2=x%0d sb_data=%h != arch_val=%h (map OK -> PRF value bug)",
                        $time, q_pc[0], q_insn[0], q_rs2[0], u_lsu.sb_data[q_sbidx[0]], arch_val[{1'b0,q_rs2[0]}]);
               end
               if (q_rk[0] != 2'd0) begin            // update tracked arch->phys (+ value)
                  ck_da = (q_rk[0]==2'd2) ? (32 + q_ri[0]) : {1'b0, q_ri[0]};
                  arch_phys[ck_da] = q_prd[0];
                  arch_val [ck_da] = q_val[0];
               end
`ifdef PRFVAL_CHK
               // PRFVAL-CHK (IW=1): at an int-writer's retire the PRF entry for its physreg MUST
               // hold its committed value -- fires if the writeback was LOST/overwritten (the
               // leaked-writeback class), distinguishing lost-writeback (here, at the writer)
               // from valid-then-clobbered (STVAL-CHK, later, at the reader). Perf geometry:
               // physreg pr lives at bank[pr[SBITS-1:0]][pr>>SBITS].
               if (q_rk[0]==2'd1 && q_ri[0]!=5'd0
                   && (eb.lane[0].sh.rf.bank[q_prd[0][SBITS-1:0]][q_prd[0][PBITS-1:SBITS]] !== q_val[0]))
                  $display("[%0t] *** PRFVAL-CHK pc=%h rd=x%0d prd=%0d PRF=%h != q_val=%h",
                     $time, q_pc[0], q_ri[0], q_prd[0],
                     eb.lane[0].sh.rf.bank[q_prd[0][SBITS-1:0]][q_prd[0][PBITS-1:SBITS]], q_val[0]);
`endif
               // advance retire-order mepc: a trap -> its own mepc; a CSR write to mepc
               // (csr 0x341, SYSTEM funct3!=0) -> the now-landed value; else unchanged.
               if (q_trap[0])
                  mepc_retire = q_mepc[0];
               else if ((q_insn[0][6:0]==7'h73) && (q_insn[0][14:12]!=3'd0)
                        && (q_insn[0][31:20]==12'h341))
                  mepc_retire = `VA_UNPACK40(eb.u_csr.mepc);
`ifdef STDATA_TAP
               // print each committed STORE's SB data (read back at retire, pre-drain) ->
               // 0 = operand/PRF-read bug; correct value = ordering/memory bug.
               if (!q_trap[0] && (q_insn[0][6:0]==7'h23))
                  $display("[%0t] ST-RETIRE pc=%h insn=%h sbidx=%0d sb_data=%h",
                     $time, q_pc[0], q_insn[0], q_sbidx[0], u_lsu.sb_data[q_sbidx[0]]);
`endif
               probe_retire(q_pc[0], q_insn[0], {6'd0, q_rk[0]},
                  (q_rk[0]==2'd0) ? 8'd0 : {3'd0, q_ri[0]},
                  {6'd0, q_prv[0]}, {7'd0, q_trap[0]}, q_val[0], q_cause[0], q_tval[0],
                  64'd0, {64{1'b1}}, mepc_retire, 8'd0);   // mepc in retire order (immune to younger writes)
               end
               for (fi = 0; fi < QN-1; fi = fi + 1) begin
                  q_seq[fi]=q_seq[fi+1]; q_ck[fi]=q_ck[fi+1]; q_pc[fi]=q_pc[fi+1];
                  q_insn[fi]=q_insn[fi+1]; q_rk[fi]=q_rk[fi+1]; q_ri[fi]=q_ri[fi+1];
                  q_prd[fi]=q_prd[fi+1]; q_prv[fi]=q_prv[fi+1]; q_mepc[fi]=q_mepc[fi+1];
                  q_val[fi]=q_val[fi+1]; q_vok[fi]=q_vok[fi+1]; q_cmt[fi]=q_cmt[fi+1];
                  q_trap[fi]=q_trap[fi+1]; q_cause[fi]=q_cause[fi+1]; q_tval[fi]=q_tval[fi+1];
                  q_ps1[fi]=q_ps1[fi+1]; q_ps2[fi]=q_ps2[fi+1]; q_rs1[fi]=q_rs1[fi+1]; q_rs2[fi]=q_rs2[fi+1];
                  q_sbidx[fi]=q_sbidx[fi+1];
               end
               qn = qn - 1;
            end
         // 1. squash: drop tail entries (uncommitted, seq younger than roll_seq)
         if (roll_v) begin
            cut = qn;
            for (fi = QN-1; fi >= 0; fi = fi - 1)
               if ((fi < qn) && !q_cmt[fi] && ($signed(q_seq[fi] - roll_seq) > 0)) cut = fi;
            qn = cut;
         end
         // 2. writeback: attribute each writeback to the in-flight entry whose SEQNO
         //    matches it (not its pdst). A physreg is reused rapidly, and a
         //    WRONG-PATH op (later squashed) can write a physreg that a younger right-path
         //    op reuses; matching purely by pdst then captures the wrong-path value (or an
         //    rd=x0 op's value) since both share the number. The writeback seqno (wkq) is
         //    unique to the producing op, so it lands on exactly that op's entry -- a
         //    squashed op's stray writeback matches no live entry and is harmlessly dropped.
         for (fl = 0; fl < IW; fl = fl + 1) if (wkv[fl])
            for (fi = 0; fi < QN; fi = fi + 1)
               if ((fi < qn) && !q_vok[fi]
                   && (q_seq[fi] == wkq[fl*SEQW +: SEQW])) begin
                  if (q_rk[fi] != 2'd0) q_val[fi] = wb_val[fl*64 +: 64];
                  q_vok[fi] = 1'b1;
               end
         // 3. commit: mark this bundle's (so-far uncommitted) entries committed
         if (cc_commit)
            for (fi = 0; fi < QN; fi = fi + 1)
               if ((fi < qn) && !q_cmt[fi] && (q_ck[fi] == cc_commit_idx)) begin
                  q_cmt[fi] = 1'b1;
               end
         // 4. dispatch: push each valid slot in program order
         if (disp_fire)
            for (fl = 0; fl < IW; fl = fl + 1) if (r_valid[fl]) begin
`ifdef IRQDBG
               if (dbgw && r_pay[fl*`PAYW + 165 +: 32] == 32'h7f000073)
                  $display("[IRQD] QPUSH-OPIRQ c=%0d seq=%0d ck=%0d pc=%h qn=%0d", dbg_cyc,
                           r_seq[fl*SEQW +: SEQW], cur, r_pay[fl*`PAYW + 78 +: 64], qn);
`endif
               q_seq [qn] = r_seq[fl*SEQW +: SEQW];
               q_ck  [qn] = cur;
               q_pc  [qn] = r_pay[fl*`PAYW + 78  +: 64];   // PAY_PC
               q_insn[qn] = r_pay[fl*`PAYW + 165 +: 32];   // PAY_INSN
               q_ri  [qn] = r_rd[fl*ABITS +: 5];
               q_rk  [qn] = !r_rd_v[fl]              ? 2'd0 :
                             r_rd[fl*ABITS + 5]      ? 2'd2 :
                            (r_rd[fl*ABITS +: 5]==0) ? 2'd0 : 2'd1;
               q_prd [qn] = pdst[fl*PBITS +: PBITS];
               q_ps1 [qn] = ps1[fl*PBITS +: PBITS];
               q_ps2 [qn] = ps2[fl*PBITS +: PBITS];
               q_rs1 [qn] = r_pay[fl*`PAYW + 165 + 15 +: 5];   // insn[19:15]
               q_rs2 [qn] = r_pay[fl*`PAYW + 165 + 20 +: 5];   // insn[24:20]
               q_sbidx[qn] = disp_sb_idx[fl*SBI +: SBI];
               q_prv [qn] = mmu_priv;
               q_mepc[qn] = 64'd0;          // filled at commit+1 (mepc-after-retire) or trap stamp
               q_val [qn] = 64'd0;  q_vok[qn] = 1'b0;  q_cmt[qn] = 1'b0;
               q_trap[qn] = 1'b0;   q_cause[qn] = 64'd0; q_tval[qn] = 64'd0;
               qn = qn + 1;
            end
         // 5. trap: deliver as the precise trap retire. ecall/ebreak/illegal-CSR and
         //    the interrupt pseudo-op COMMIT (count at issue) and trap -- convert that
         //    committed entry in place so it doesn't emit as a normal retire. Data/
         //    illegal/fetch faults annul their op (squashed above) -> push a fresh entry.
         if (cot_fire) begin
            cot_found = 1'b0;
            for (fi = 0; fi < QN; fi = fi + 1)
               if (!cot_found && (fi < qn) && !q_trap[fi] && (q_pc[fi] == cot_epc)) begin
                  q_trap[fi] = 1'b1; q_cmt[fi] = 1'b1; q_vok[fi] = 1'b1; q_rk[fi] = 2'd0;
`ifdef IRQDBG
                  if (dbgw) $display("[IRQD] STAMP c=%0d epc=%h cause=%h at fifo[%0d] seq=%0d ck=%0d insn-was=%h qn=%0d",
                           dbg_cyc, cot_epc, cot_cause, fi, q_seq[fi], q_ck[fi], q_insn[fi], qn);
`endif
                  q_insn[fi] = cot_insn; q_prv[fi] = cot_prv; q_mepc[fi] = cot_mepc;
                  q_cause[fi] = cot_cause; q_tval[fi] = cot_tval; cot_found = 1'b1;
               end
`ifdef IRQDBG
            if (dbgw & !cot_found)
               $display("[IRQD] STAMP-MISS c=%0d epc=%h cause=%h qn=%0d head pc=%h insn=%h (fresh-push fallback)",
                        dbg_cyc, cot_epc, cot_cause, qn, q_pc[0], q_insn[0]);
`endif
            if (!cot_found) begin
               q_seq [qn] = {SEQW{1'b0}};  q_ck[qn] = {CBITS{1'b0}};
               q_pc  [qn] = cot_epc;       q_insn[qn] = cot_insn;
               q_rk  [qn] = 2'd0;          q_ri[qn] = 5'd0;
               q_prd [qn] = {PBITS{1'b0}}; q_prv[qn] = cot_prv;  q_mepc[qn] = cot_mepc;
               q_val [qn] = 64'd0;         q_vok[qn] = 1'b1;     q_cmt[qn] = 1'b1;
               q_trap[qn] = 1'b1;          q_cause[qn] = cot_cause; q_tval[qn] = cot_tval;
               qn = qn + 1;
            end
         end
      end
   end
`endif

`ifdef SEQROB
   // ============================================================================
   // Sim-only seqno-ROB value-flow checker. A LOGICAL (seqno) shadow that is immune
   // to physreg reuse: a seqno-RAT maps each integer arch reg to its producer's
   // {hwseq, abs}; a finite ROB (indexed by hwseq, tagged with a monotone abs to
   // detect eviction) records each producer's writeback. At EXECUTE we assert that
   // each integer source's producer has already written back -- a consumer reading
   // before its producer completes (the physreg-reuse early-wake bug) trips it AT THE
   // CYCLE, no cosim needed. The seqno-RAT is recovered on rollback exactly like the
   // rename MAP (snapshot at create, restore at rollback). A producer whose ROB entry
   // was evicted (tag mismatch) is too far in the past -> skipped (finite window).
   localparam ABW = 32;
   integer sa, sl, swb;
   reg [ABW-1:0]  g_abs;
   reg            sr_v  [0:31];                     // seqno-RAT: arch -> producer
   reg [SEQW-1:0] sr_hs [0:31];
   reg [ABW-1:0]  sr_ab [0:31];
   reg            cr_v  [0:NCHK-1][0:31];           // per-checkpoint snapshot (rollback)
   reg [SEQW-1:0] cr_hs [0:NCHK-1][0:31];
   reg [ABW-1:0]  cr_ab [0:NCHK-1][0:31];
   reg            o1v [0:255]; reg [SEQW-1:0] o1hs [0:255]; reg [ABW-1:0] o1ab [0:255];
   reg            o2v [0:255]; reg [SEQW-1:0] o2hs [0:255]; reg [ABW-1:0] o2ab [0:255];
   reg [ABW-1:0]  rb_tag [0:255];                   // ROB (by hwseq): producer abs tag
   reg            rb_wbv [0:255];                   // ... writeback done
   reg [63:0]     rb_val [0:255];                   // ... value (for the value check)
   reg [ABW-1:0]  po     [0:NPHYS-1];               // phys-reg -> owner abs (set at allocation)
   reg            tv [0:31]; reg [SEQW-1:0] ths [0:31]; reg [ABW-1:0] tab [0:31];
   reg [ABW-1:0]  wab;
   reg [6:0]      sop; reg [4:0] srs1, srs2, srd; reg srdfp, sck1, sck2;
   reg [SEQW-1:0] shs, xhs; reg wbnow1, wbnow2;
   initial begin
      g_abs = 1;
      for (sa = 0; sa < 32;  sa = sa + 1) begin sr_v[sa]=0; sr_hs[sa]=0; sr_ab[sa]=0; end
      for (sl = 0; sl < NCHK; sl = sl + 1)
         for (sa = 0; sa < 32; sa = sa + 1) begin cr_v[sl][sa]=0; cr_hs[sl][sa]=0; cr_ab[sl][sa]=0; end
      for (sa = 0; sa < 256; sa = sa + 1) begin o1v[sa]=0; o2v[sa]=0; rb_tag[sa]=0; rb_wbv[sa]=0; end
      for (sa = 0; sa < NPHYS; sa = sa + 1) po[sa]=0;
   end
   // dispatch: capture sources from the seqno-RAT, open ROB tags, update + snapshot RAT
   always @(posedge clk) if (!reset) begin
      if (cc_rollback) begin
         for (sa = 0; sa < 32; sa = sa + 1) begin
            sr_v[sa] <= cr_v[cc_rollback_idx][sa]; sr_hs[sa] <= cr_hs[cc_rollback_idx][sa];
            sr_ab[sa] <= cr_ab[cc_rollback_idx][sa];
         end
      end else if (disp_fire) begin
         for (sa = 0; sa < 32; sa = sa + 1) begin tv[sa]=sr_v[sa]; ths[sa]=sr_hs[sa]; tab[sa]=sr_ab[sa]; end
         wab = g_abs;
         for (sl = 0; sl < IW; sl = sl + 1) if (r_valid[sl]) begin
            shs  = r_seq[sl*SEQW +: SEQW];
            sop  = r_pay[sl*`PAYW + 165 +: 7];
            srs1 = r_pay[sl*`PAYW + 165 + 15 +: 5];
            srs2 = r_pay[sl*`PAYW + 165 + 20 +: 5];
            srd  = r_rd[sl*ABITS +: 5];  srdfp = r_rd[sl*ABITS + 5];
            sck1 = (sop==7'h33)|(sop==7'h3b)|(sop==7'h13)|(sop==7'h1b)
                 | (sop==7'h03)|(sop==7'h23)|(sop==7'h63)|(sop==7'h67)|(sop==7'h2f);
            sck2 = (sop==7'h33)|(sop==7'h3b)|(sop==7'h23)|(sop==7'h63)|(sop==7'h2f);
            o1v[shs] <= sck1 & tv[srs1] & (srs1 != 5'd0); o1hs[shs] <= ths[srs1]; o1ab[shs] <= tab[srs1];
            o2v[shs] <= sck2 & tv[srs2] & (srs2 != 5'd0); o2hs[shs] <= ths[srs2]; o2ab[shs] <= tab[srs2];
            rb_tag[shs] <= wab;  rb_wbv[shs] <= 1'b0;
            if (r_rd_v[sl]) po[pdst[sl*PBITS +: PBITS]] <= wab;   // this physreg now belongs to this op
            if (r_rd_v[sl] & ~srdfp & (srd != 5'd0)) begin tv[srd]=1'b1; ths[srd]=shs; tab[srd]=wab; end
            wab = wab + 1'b1;
         end
         for (sa = 0; sa < 32; sa = sa + 1) begin
            sr_v[sa] <= tv[sa]; sr_hs[sa] <= ths[sa]; sr_ab[sa] <= tab[sa];
            cr_v[cur+1'b1][sa] <= tv[sa]; cr_hs[cur+1'b1][sa] <= ths[sa]; cr_ab[cur+1'b1][sa] <= tab[sa];
         end
         g_abs <= wab;
      end
      // writeback: mark each producer's ROB entry done + record value (same block to
      // keep rb_* single-driver; runs every cycle, after the dispatch open above)
      for (sl = 0; sl < IW; sl = sl + 1) if (wkv[sl]) begin
         // FAULTY-WRITEBACK: the physreg being written must still belong to the writer.
         // If it has been reallocated (owner abs != writer abs), this is a stale/squashed
         // writeback overwriting a good register -- the leaked-writeback bug.
         if ((po[wkp[sl*PBITS +: PBITS]] != 0) && (rb_tag[wkq[sl*SEQW +: SEQW]] != 0)
             && (po[wkp[sl*PBITS +: PBITS]] != rb_tag[wkq[sl*SEQW +: SEQW]]))
            $display("[%0t] *** SEQROB FAULTY-WRITEBACK: op hwseq=%0d (abs=%0d) writes phys %0d val=%h, but it is owned by abs=%0d",
               $time, wkq[sl*SEQW +: SEQW], rb_tag[wkq[sl*SEQW +: SEQW]],
               wkp[sl*PBITS +: PBITS], wb_val[sl*64 +: 64], po[wkp[sl*PBITS +: PBITS]]);
         rb_wbv[wkq[sl*SEQW +: SEQW]] <= 1'b1;
         rb_val[wkq[sl*SEQW +: SEQW]] <= wb_val[sl*64 +: 64];
      end
   end
   // execute: a source's producer (still in the ROB window) must have written back
   always @(posedge clk) if (!reset)
      for (sl = 0; sl < IW; sl = sl + 1) if (ex_valid[sl]) begin
         xhs = ex_seq[sl*SEQW +: SEQW];
         wbnow1 = 1'b0; wbnow2 = 1'b0;                  // producer writing back THIS cycle? (EX/wb race)
         for (swb = 0; swb < IW; swb = swb + 1) if (wkv[swb]) begin
            if (wkq[swb*SEQW +: SEQW] == o1hs[xhs]) wbnow1 = 1'b1;
            if (wkq[swb*SEQW +: SEQW] == o2hs[xhs]) wbnow2 = 1'b1;
         end
         if (o1v[xhs] && (rb_tag[o1hs[xhs]] == o1ab[xhs]) && !rb_wbv[o1hs[xhs]] && !wbnow1)
            $display("[%0t] *** SEQROB EARLY-READ: op hwseq=%0d read rs1 from producer hwseq=%0d (abs=%0d) before writeback",
               $time, xhs, o1hs[xhs], o1ab[xhs]);
         if (o2v[xhs] && (rb_tag[o2hs[xhs]] == o2ab[xhs]) && !rb_wbv[o2hs[xhs]] && !wbnow2)
            $display("[%0t] *** SEQROB EARLY-READ: op hwseq=%0d read rs2 from producer hwseq=%0d (abs=%0d) before writeback",
               $time, xhs, o2hs[xhs], o2ab[xhs]);
      end
`endif

`ifdef PERF_TRACE
   // Performance event trace (docs/perf-observability-plan.md, step 1). One DPI call
   // per pipeline event into perf_trace.cpp; perf_cyc is the trace clock. seqno is the
   // cross-stage key, ckpid back-attributes; READY is derived offline from DISPATCH's
   // ps1/ps2 + WRITEBACK clocks. kind: 1=DISPATCH 2=SELECT 3=WRITEBACK 4=COMMIT 5=SQUASH.
   import "DPI-C" function void perf_ev(input longint cyc, input int kind, input int seq,
                                        input int ckp, input int rdv, input int pdst,
                                        input int ps1, input int ps2, input longint data,
                                        input int insn);
   reg [63:0] perf_cyc; integer pti;
   initial perf_cyc = 64'd0;
   always @(posedge clk) if (!reset) begin
      perf_cyc <= perf_cyc + 64'd1;
      if (disp_fire)
         for (pti = 0; pti < IW; pti = pti + 1) if (r_valid[pti])
            perf_ev(perf_cyc, 1, {24'd0, r_seq[pti*SEQW +: SEQW]}, {30'd0, cur},
                    {31'd0, r_rd_v[pti]}, {{(32-PBITS){1'b0}}, pdst[pti*PBITS +: PBITS]},
                    {{(32-PBITS){1'b0}}, ps1[pti*PBITS +: PBITS]},
                    {{(32-PBITS){1'b0}}, ps2[pti*PBITS +: PBITS]},
                    r_pay[pti*`PAYW + 78 +: 64], r_pay[pti*`PAYW + 165 +: 32]); // PAY_PC, PAY_INSN
      for (pti = 0; pti < IW; pti = pti + 1) if (iss_valid[pti])
         perf_ev(perf_cyc, 2, {24'd0, iss_seq[pti*SEQW +: SEQW]},
                 {30'd0, iss_ckpt[pti*CBITS +: CBITS]}, 0, 0, 0, 0, 64'd0, 0);
      for (pti = 0; pti < IW; pti = pti + 1) if (wkv[pti])
         perf_ev(perf_cyc, 3, {24'd0, wkq[pti*SEQW +: SEQW]}, 0, 0,
                 {{(32-PBITS){1'b0}}, wkp[pti*PBITS +: PBITS]}, 0, 0,
                 wb_val[pti*64 +: 64], 0);
      if (cc_commit) perf_ev(perf_cyc, 4, 0, {30'd0, cc_commit_idx}, 0, 0, 0, 0, 64'd0, 0);
      // rdv field carries the roll cause: 0=branch-mispredict 1=data-fault 3=trap 2=other
      if (roll_v)    perf_ev(perf_cyc, 5, {24'd0, roll_seq}, {30'd0, roll_ckpt},
                             (dflt_roll ? 32'd1 : eb_rtrap ? 32'd3 : eb_redirect ? 32'd0 : 32'd2),
                             0, 0, 0, 64'd0, 0);
      // KIND 6 = dispatch STALL: a bundle is ready but can't dispatch. The `seq` field
      // carries an 8-bit reason mask (the exact terms of can_dispatch):
      //   b0 cc_full(checkpoints)  b1 fe_stall(free regs)  b2 !disp_ready(scheduler)
      //   b3 sb_full(store buf)    b4 lq_full(load queue)  b5 dfault|ill(fault freeze)
      //   b6 eb_redirect
      if (any_valid && !can_dispatch)
         perf_ev(perf_cyc, 6,
                 {24'd0, 1'b0, eb_redirect, (lsu_dfault_v | ill_v), lq_full, sb_full,
                  ~(&disp_ready), (|fe_stall), cc_full},
                 0, 0, 0, 0, 0, 64'd0, 0);
      // KIND 7 = FETCH-EMPTY: the frontend delivered NO bundle to dispatch (the
      // `frontend-empty` accounting bucket). The `seq` field carries the reason this
      // fetch slot is empty, so an I$ miss is told apart from a redirect refetch:
      //   b0 redirect (frontend being re-steered: branch/trap/fence flush)
      //   b1 immu-wait (iTLB miss / PTW in progress -> imem gated off)
      //   b2 immu-fault (fetch page/access fault pending)
      //   b3 icache-miss (translation OK but the I$ returned 0 halfwords)
      //   b4 other (none above: fetch/decode/rename pipeline bubble or aligner truncate)
      if (!any_valid) begin : fetch_empty_ev
         reg fe_rd, fe_iw, fe_if, fe_im, fe_ot;
         fe_rd = fe_red_v;
         fe_iw = ~immu_ready & ~fe_red_v;
         fe_if = immu_fault & ~fe_red_v;
         fe_im = immu_ready & ~immu_fault & (imem_avail == 0) & ~fe_red_v;
         fe_ot = ~(fe_rd | fe_iw | fe_if | fe_im);
         perf_ev(perf_cyc, 7,
                 {27'd0, fe_ot, fe_im, fe_if, fe_iw, fe_rd},
                 0, 0, 0, 0, 0, 64'd0, 0);
      end
   end
`endif
   // ---- wedge bus (ILA probe7) ----------------------------------------------------
   // The board freezes in U-mode with a pending+enabled S-timer interrupt it never takes,
   // fetch parked on one PA. Interrupts here are a frontend-INJECTED pseudo-op gated by
   // `accept`, so a frontend that never accepts (or an inject_inflight that never clears)
   // blocks delivery forever -- userspace is never preempted and every systemd job times
   // out. These bits say which of the two it is, and why. The dispatch-stall and
   // fetch-empty reason bits are the SAME encodings the PERF_TRACE KIND 6/7 events use.
   // [63:50] reserved. [49:38] name WHAT commit is waiting on: cc_full alone only says "a
   // checkpoint's count never reached zero". dbg_cnt IS that count, and the rest say whether the
   // missing completion is a deferred LSU/unit op still outstanding, or an op that never issued
   // at all (a live scheduler entry that never became eligible = a lost operand wakeup).
   assign dbg_lsu = lsu_dbg_lsu;
   assign dbg_wedge = {
      7'd0,
      ld_supp_cnt,              // [56:53] saturating count of ld_done decrements EATEN by the
                                //         owner guard -- a false-suppress leaks the count forever
      eb_dbg_evap,              // [52:50] STICKY evaporation: [50] mul/div, [51] FP-unit-busy,
                                //         [52] CVFPU not iss_ready. Any of these = a deferred op
                                //         vanished at EX and its checkpoint count can never reach 0.
      cc_dbg_cnt,               // [49:48] count[committed] (CNTW=2)
      |sch_dbg_stuck,           // [47] a live RS entry that is NOT eligible (never issues)
      |sch_dbg_any_v,           // [46] any live RS entry at all
      |eb_exec_busy,            // [45] a mul/div/FP unit still running
      lsu_dbg_defer,            // [44:41] {lq_any, sb_any, amo_busy, devrd_pending}
      amo_gap,                  // [40] AMO dispatch freeze
      cc_stall_barrier,         // [39] barrier store-drain stall
      1'b0,                     // [38]
      {{(4-CBITS){1'b0}}, cc_committed},  // [37:34] oldest live checkpoint (CBITS<=4)
      {{(4-CBITS){1'b0}}, cur},           // [33:30] newest checkpoint
      iflt_fire,                // [29]
      dflt_fire,                // [28]
      dflt_replay,              // [27]
      pend_iflt,                // [26]
      devld_solo_v,             // [25]
      replay_v,                 // [24]
      roll_v,                   // [23]
      irq_inject,               // [22] injection allowed THIS cycle
      inject_inflight,          // [21] a pseudo-op is already in flight (blocks new ones)
      csr_irq_v,                // [20] an enabled+pending interrupt is deliverable
      (imem_avail == 0),        // [19] fetch-empty: I$ returned nothing
      immu_fault,               // [18] fetch-empty: page/access fault pending
      immu_ready,               // [17] fetch-empty: 0 = iTLB miss / PTW in progress
      fe_red_v,                 // [16] fetch-empty: frontend being re-steered
      cc_empty,                 // [15]
      cc_commit,                // [14]
      ill_v,                    // [13]
      lsu_dfault_v,             // [12]
      eb_redirect,              // [11]
      lq_full,                  // [10] dispatch-stall reasons (PERF_TRACE KIND 6 mask)
      sb_full,                  // [9]
      ~(&disp_ready),           // [8]
      (|fe_stall),              // [7]
      cc_full,                  // [6]
      accept,                   // [5]  0 = frontend frozen (no injection can land)
      can_dispatch,             // [4]
      any_valid,                // [3]
      3'd0 };
endmodule

`default_nettype wire
