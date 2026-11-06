`default_nettype none

// mimpid = the truncated git commit of the build's HEAD, injected by the build
// (build.tcl for FPGA, the sim run-scripts for simulation) -- mirrors SmolRV64's
// SMOLRV64_GIT_COMMIT mechanism. Fallback 0 for an un-passed build. It is the INIT value of
// kept flops (mimpid_q), not a constant in the read mux: a constant shapes the mux by its bits,
// so every commit hash would synthesize a different netlist and draw a different placement.
`ifndef SMOLRV64_GIT_COMMIT
 `define SMOLRV64_GIT_COMMIT 32'h0
`endif

// Architectural CSR state, M/S/U privilege model (written for the sharded-OoO core, now
// used by smolrv64_core; comments naming backend_top/exec_shard describe that retired core).
// Read is combinational; a single update per cycle (one system op executes at a
// time -- the scheduler gates a serializing op until its checkpoint is oldest, so
// the update is precise/non-speculative).
//
// Implements: privilege tracking (M/S/U); mstatus with the standard writable
// fields + hardwired SXL=UXL=2; S-mode CSRs (sstatus/sie/sip as views of
// mstatus/mie/mip, stvec/sepc/scause/stval/sscratch/scounteren/satp); trap
// delegation (medeleg/mideleg route a trap from priv<=S to S-mode); ecall cause by
// privilege; mret/sret. The shard drives the system-op request; this module decodes
// ecall/ebreak/mret/sret from imm[11:0], updates state, and produces the redirect
// target. Interrupts/MMU enforcement are not wired yet (mie/mip/satp are stored).
module csr_file
   (input  wire        clk,
    input  wire        reset,
    // combinational read (for a CSR op's rd = old value)
    input  wire [11:0] raddr,
    output reg  [63:0] rdata,
    // redirect for the current system op (combinational)
    output reg  [39:0] redir_target,  // a PC: va_codec.vh's 40-bit form (smolrv64_pc.vh)
    output wire        redir_valid,   // this op redirects (trap / xret / illegal-CSR)
    output wire        redir_is_trap, // redirect is an exception (roll back TO this op's ckpt)
    output wire        csr_illegal,   // active CSR op is an illegal access (suppress rd)
    // ---- translation context for the iMMU/dMMU (combinational) ----
    output wire [63:0] o_satp,        // satp (MODE/ASID/PPN)
    output wire [1:0]  o_priv,        // current privilege (instruction-fetch priv)
    output wire        o_ifx_d,       // fetch translates NEXT cycle (priv not M, satp MODE Sv39): a flop's D
    output wire [1:0]  o_dpriv,       // effective data-access priv (honors MPRV/MPP)
    output wire        o_sum,         // mstatus.SUM
    output wire        o_mxr,         // mstatus.MXR
    output wire [2:0]  o_frm,         // fcsr.frm -> FP units for dynamic rounding
    output wire        o_fs_off,      // mstatus.FS==Off -> FP instructions trap illegal
    // FP exception-flag accumulation: OR fp_fflags into fcsr.fflags when fp_fflags_we
    // (the flags of the FP ops retiring this cycle, reduced by the caller).
    input  wire        fp_fflags_we,
    input  wire [4:0]  fp_fflags,
    // mstatus.FS -> Dirty when an op that changes FP state RETIRES: the caller raises this in the
    // cycle any committing op wrote an f-register or retired flags into fcsr. Commit-gated (never
    // speculative) so a squashed FP op never dirties FS.
    input  wire        fp_dirty_commit,
    output wire        o_tlb_flush,   // 1-cycle: sfence.vma or satp write -> flush TLBs
    // ---- external trap injection (page faults from the iMMU/LSU; precise) ----
    // Fired by backend_top once the fault is the oldest (fetch: pipeline empty; data:
    // rolled back to the faulting checkpoint). Mutually exclusive with a system op.
    input  wire        xtrap_v,
    input  wire        xtrap_intr,    // the injected trap is an interrupt (mcause MSB, vectored)
    input  wire [3:0]  xtrap_cause,   // exception: 12/13/15 page fault; interrupt: cause number
    input  wire [39:0] xtrap_epc,     // resume PC (faulting VA / faulting bundle start), 40-bit form
    input  wire [63:0] xtrap_tval,    // faulting virtual address
    // ---- hardware interrupt-pending lines (from CLINT/PLIC, combinational) ----
    // The device-owned mip bits: MEIP(11)/SEIP(9)/MTIP(7)/STIP(5)/MSIP(3). OR'd into
    // the effective mip; the read-only set (MEIP/MTIP/MSIP) is masked out of CSR writes
    // so software clears them only at the device (mtimecmp/msip), never via mip.
    input  wire [11:0] hw_ip,
    input  wire [63:0] mtime,       // free-running CLINT time (Sstc stimecmp compare); 0 in device-less TBs
    input  wire [5:0]  retire_cnt,  // # instructions retiring this cycle (commit_ctl) -> minstret
                                    // (a coarse checkpoint retires up to CKMAX at once)
    input  wire [5:0]  hpm_retire_cnt, // the SAME count for the Zihpm counters, and the caller is
                                    // free to delay it. minstret is architectural and must be
                                    // exact this cycle; mhpmcounterN is instrumentation read
                                    // through a CSR many cycles later, so a cycle of lag is
                                    // unobservable -- exactly the argument `hpm_ev` already
                                    // rests on. Splitting them keeps the retire cone out of
                                    // THIRTEEN event muxes and 64-bit carry chains. See
                                    // smolrv64_core's hpm_ret_q.
    // Zihpm event bits (each +1 in a cycle it is high), selected per counter by mhpmeventN;
    // the event map below names every bit and says what it counts (docs/SmolRV64-Spec.md 11).
    input  wire [43:0] hpm_ev,
    input  wire [5:0]  hpm_lqocc,        // load-queue occupancy this cycle (MEM_LQOCC)
    input  wire [5:0]  hpm_sqocc,        // store-queue occupancy this cycle (MEM_SQOCC)
    input  wire [5:0]  hpm_ldinfl,       // load reads in flight at the D$ this cycle (MEM_LDINFL)
    input  wire [2:0]  hpm_disp,         // instructions dispatched this cycle (DPATCH)
    input  wire [4:0]  hpm_tdk,          // this cycle's top-down cause td_k (the TDK_* events)
    // ---- pending interrupt (combinational): backend fires it via xtrap_* when it can ----
    output wire [63:0] dbg_timer,     // timer/interrupt-path debug bus (wrapper ILA_TIMER; pruned when unused)
    output wire        dbg_mtvec_we,  // 1-cycle: an executing CSR op writes mtvec (ILA probe4)
    output wire        dbg_csrop_v,   // 1-cycle: ANY system op reaches this update port (ILA probe6)
    output wire [63:0] dbg_csrop,     // ...and which one: {pc, addr, func, is_csr} (ILA probe5).
                                      // Splits the stranding hypothesis in two: if the lost
                                      // `csrrw x12,mtvec,x12` shows up here, its write was dropped
                                      // inside csr_file; if it never appears, the op never reached
                                      // EX and was lost in fetch/issue/squash.
    output wire [63:0] dbg_mtvec,     // M trap vector (ILA probe2): catches mtvec left at
                                      // OpenSBI's __sbi_expected_trap, which silently skips
                                      // every ecall (SBI calls no-op -> timer never armed)
    output wire        irq_v,         // an enabled+pending interrupt is deliverable now
    output wire [3:0]  irq_cause,     // its cause number (highest priority)
    // single update (driven at EX by the oldest system op -> non-speculative)
    input  wire        upd_valid,
    input  wire        upd_is_csr,
    input  wire [2:0]  upd_func,     // CSR funct3 (csrrw/s/c + imm forms)
    input  wire [11:0] upd_addr,     // CSR addr (is_csr) OR system-op selector (imm[11:0])
    input  wire [63:0] upd_src,      // rs1 value or zimm
    input  wire [39:0] upd_pc);      // the system op's PC, 40-bit form

   localparam [1:0] M=2'd3, S=2'd1, U=2'd0;

   localparam [11:0] MSTATUS=12'h300, MISA=12'h301, MEDELEG=12'h302, MIDELEG=12'h303,
                     MIE=12'h304, MTVEC=12'h305, MCOUNTEREN=12'h306,
                     MSCRATCH=12'h340, MEPC=12'h341, MCAUSE=12'h342, MTVAL=12'h343,
                     MIP=12'h344, MHARTID=12'hF14, MTOPI=12'hFB0,
                     MNSTATUS=12'h744,
                     SSTATUS=12'h100, SIE=12'h104, STVEC=12'h105, SCOUNTEREN=12'h106,
                     SSCRATCH=12'h140, SEPC=12'h141, SCAUSE=12'h142, STVAL=12'h143,
                     SIP=12'h144, SATP=12'h180, SENVCFG=12'h10A,
                     MVENDORID=12'hF11, MARCHID=12'hF12, MIMPID=12'hF13,
                     FFLAGS=12'h001, FRM=12'h002, FCSR=12'h003,
                     STIMECMP=12'h14D, MENVCFG=12'h30A,
                     MSTATEEN0=12'h30C, SSTATEEN0=12'h10C,
                     // Zicntr: M-mode counters + their U/S read-only shadows. time is the
                     // hardware-backed CLINT mtime (like SmolRV64); cycle/instret shadow
                     // the free-running mcycle / retired-instruction minstret.
                     MCYCLE=12'hB00, MTIME=12'hB01, MINSTRET=12'hB02,
                     CYCLE=12'hC00, TIME=12'hC01, INSTRET=12'hC02,
                     // Zihpm: mcountinhibit + programmable mhpmcounter3.. / mhpmevent3.. and
                     // their U/S read-only shadows hpmcounter3.. . SmolRV64 layout + event
                     // encoding so the DTB pmu node and OpenSBI's SBI-PMU work unchanged --
                     // without real (WARL, readback) counter CSRs, pmu_sbi_devinit hangs
                     // because its counter start/stop writes vanish into the read-0 default.
                     MCOUNTINHIBIT=12'h320, MHPMEVENT3=12'h323,
                     MHPMCOUNTER3=12'hB03, HPMCOUNTER3=12'hC03,
                     // Sscofpmf: the OF bits of mhpmevent3.. as one read-only S-mode CSR.
                     SCOUNTOVF=12'hDA0;

   // system-op selectors (imm[11:0] of a funct3==0 SYSTEM op)
   localparam [11:0] OP_ECALL=12'h000, OP_EBREAK=12'h001, OP_SRET=12'h102,
                     OP_MRET=12'h302, OP_WFI=12'h105,
                     // synthetic SYSTEM selector for the interrupt pseudo-instruction the
                     // frontend injects when an interrupt is pending. It reuses the whole
                     // ecall trap path (solo checkpoint, roll-back-TO-ckpt, vector redirect);
                     // delivery here just substitutes the interrupt cause/epc semantics.
                     OP_IRQ=12'h7F0;

   // RV64 misa = MXL(2)<<62 | I C M A S U
   localparam [63:0] MISA_VAL = (64'd2<<62) | (64'd1<<0)  /*A*/ | (64'd1<<2)  /*C*/
                              | (64'd1<<8) /*I*/ | (64'd1<<12) /*M*/
                              | (64'd1<<3) /*D*/  | (64'd1<<5)  /*F*/
                              | (64'd1<<18) /*S*/ | (64'd1<<20) /*U*/;

   // mstatus writable bits (M-mode write); SXL/UXL (35:32) are hardwired to 2.
   localparam [63:0] MSTATUS_WMASK = 64'h0000_0000_007E_79AA;
   // sstatus view: SIE,SPIE,SPP,FS,VS,XS,SUM,MXR,UXL + SD
   localparam [63:0] SSTATUS_RMASK = 64'h8000_0003_000D_E133;
   localparam [63:0] SSTATUS_WMASK = 64'h0000_0000_000C_6122;
   // interrupt-enable/pending S-visible bits (SSIE/STIE/SEIE = 1,5,9, Sscofpmf's LCOFIE = 13)
   localparam [63:0] S_INT_MASK = 64'h0000_0000_0000_2222;
   localparam        LCOFIP_B = 13;   // Sscofpmf local counter-overflow interrupt
   // mip bits owned read-only by hardware (MEIP/MTIP/MSIP = 11,7,3): software MIP
   // writes can't touch them (they reflect the device lines, cleared at the device).
   localparam [63:0] HW_RO_MASK = 64'h0000_0000_0000_0888;

   reg [1:0]  priv;
   reg [63:0] mstatus, mcause, mscratch, mie, mip,
              medeleg, mideleg, mcounteren, satp, mnstatus,
              stimecmp, menvcfg,   // Sstc: supervisor timer-compare + menvcfg.STCE enable
              senvcfg,             // S-mode envcfg (FIOM + Zicbom/Zicboz U-mode CBO enables)
              mstateen0;           // Smstateen: only SE0(63)/ENVCFG(62) implemented
   reg [63:0] scause, sscratch, scounteren;
   // VA-holding CSRs stored 40-bit Sv39-compressed (low 39 + non-canonical flag):
   // mepc/sepc can capture a non-canonical fetch-fault target, mtval/stval the
   // faulting VA -- both need the flag.  Non-VA tval values (illegal-instr encoding,
   // 0) fit in 39 bits with bit38=0 so they round-trip exactly.  See va_codec.vh.
   // mtvec/stvec are PCs too (their MODE in bits [1:0]): the trap target is a PC's 40 bits.
   reg [39:0] mepc, mtval, sepc, stval, mtvec, stvec;
   reg [63:0] mcycle, minstret;      // Zicntr: free-running cycles + retired instructions
   (* dont_touch = "true" *) reg [31:0] mimpid_q = `SMOLRV64_GIT_COMMIT;
   always @(posedge clk) mimpid_q <= mimpid_q;

   // Zihpm: HPMN counters mhpmcounter3 .. mhpmcounter31, matching the DTB's event->counter map.
   // The HPMG GENERAL counters (3..15) each count the event their mhpmeventN selects: 1 for a bit
   // that is high, or a count (retirements, occupancies, dispatches); an unknown code adds 0. The
   // HPMF FIXED counters (16..31) each count one top-down cause, counter 16+k the cycles with
   // td_k == HPM_TDK[k]: every cause that can occur outside dispatching -- bad speculation's 3-4,
   // the front-end's 5-8, the back-end's 10 and 12-20 (9, 11 and 21 cannot occur, asserted) -- so
   // a session holds all of td_k with only the dispatch widths on general counters. Such a counter
   // has no select, and counts only while its mhpmeventN holds its cause's code 0x05nn (WARL: the
   // code field keeps that code or 0).
   localparam integer HPMG = 13, HPMF = 16, HPMN = HPMG + HPMF;
   localparam [79:0]  HPM_TDK = {5'd20, 5'd19, 5'd18, 5'd17, 5'd16, 5'd15, 5'd14, 5'd13,
                                 5'd12, 5'd10, 5'd8,  5'd7,  5'd6,  5'd5,  5'd4,  5'd3};   // [5k +: 5]
   localparam [63:0]  HPM_INHIBIT_MASK = (((64'h1 << (HPMN+3)) - 1) & ~64'h2); // 0,2,3..31 (not TIME)
   // THE event map. docs/smolrv64-perf-events.json is GENERATED from this block by
   // tools/gen-perf-events.py -- do not edit the JSON by hand, and keep one event per line
   // with a trailing // description, because that is what the generator parses.
   //
   // 0x01xx are the caches and the dMMU; 0x03xx the stall events, which OVERLAP (each says when
   // it fires; ST_DIV and ST_MUL are occupancies) and the memory events; 0x04xx the top-down
   // events, which partition every cycle (SmolRV64-Spec 11.2); 0x05nn is the top-down cause
   // td_k == nn itself, one event per TOPDOWN-SIM bucket. 0x02xx is avoided -- the DTB maps perf
   // BUS_CYCLES onto 0x0202.
   localparam [15:0]  HPMEV_CYCLES = 16'h0001,   // Clock cycles while this event is selected
                      HPMEV_INSTRET= 16'h0002,   // Instructions retired
                      HPMEV_LOAD   = 16'h0003,   // Load landings: a plain or FP load's data written back, fast or slow path or forwarded (a wrong-path or replayed landing included; AMO and LR not)
                      HPMEV_STORE  = 16'h0004,   // Stores the store queue hands to the LSU after commit (AMO, SC and CBO writes not)
                      HPMEV_REDIR  = 16'h0005,   // Pipeline restarts: a squash at the ROB head, or a mispredict's release (decode resteers not)
                      HPMEV_DCACC  = 16'h0100,   // D$ requests taken: a read at its door (loads and page-table-walk reads) or a write into its store queue
                      HPMEV_DCMISS = 16'h0102,   // D$ line fills started (an MSHR allocated) by a load, store, cbo.zero or page-table-walk read; a join of a fill in flight not
                      HPMEV_ICACC  = 16'h0110,   // I$ requests taken, one 16-byte pair each (run-ahead and wrong-path ones included)
                      HPMEV_ICMISS = 16'h0112,   // I$ requests whose lookup misses the virtual tag (a physical-tag reconcile, a prefetch-buffer hit or an L2 fill follows)
                      HPMEV_ST_MEM = 16'h0300,   // M holds an unfinished AMO, LR, SC or CBO, + no lane issued while a scheduler's lowest-slot waiting entry needs an integer load's, AMO's or CSR read's result
                      HPMEV_ST_DIV = 16'h0301,   // The MD stage holds a divide (occupancy)
                      HPMEV_ST_MUL = 16'h0302,   // A multiply in flight in a lane (occupancy)
                      HPMEV_ST_FPU = 16'h0303,   // Stage F holds an op it cannot advance, + no lane issued while a scheduler's lowest-slot waiting entry needs an FP op's, FP load's, FP-port or divide result
                      HPMEV_ST_DSP = 16'h0304,   // The queue head waits for a credit other than the ROB's while M can advance and no load or FP dependency holds issue: ST_IQ + ST_RN + ST_SQ + ST_LQ + ST_SRZ
                      HPMEV_ST_ROB = 16'h0305,   // The queue head waits for ROB room
                      HPMEV_ST_IQ  = 16'h0306,   // The queue head: its lane's scheduler, the FP FIFO and scheduler, the head-op register or the divide credit is full
                      HPMEV_ST_RN  = 16'h0307,   // The queue head: a rename shard is below LOWAT, or the map's sweep after a squash runs
                      HPMEV_ST_SQ  = 16'h0308,   // The queue head holds a store and has no store-queue credit (none while a head op is in flight or in the IR)
                      HPMEV_ST_LQ  = 16'h0309,   // The queue head holds a load and has no load-queue credit (none while a head op is in flight or in the IR)
                      HPMEV_ST_SRZ = 16'h030a,   // The queue head: a serializing op drains or is in flight, a CSR op waits for the one before it, or a CBO waits for older addresses (the rest of ST_DSP)
                      HPMEV_FE_BUB = 16'h0310,   // The IR is empty, the queue head waits for no credit, M is free, no redirect is live and no back-end refill is due: FE_MMU + FE_IC + FE_ALN + FE_QUE
                      HPMEV_FE_MMU = 16'h0311,   // FE_BUB while the iMMU cannot translate the fetch address (a second-level lookup, a walk, or a walk held behind a weak branch)
                      HPMEV_FE_IC  = 16'h0312,   // FE_BUB while the fetch window is empty
                      HPMEV_FE_ALN = 16'h0313,   // FE_BUB while fetch has bytes but no complete instruction
                      HPMEV_FE_QUE = 16'h0314,   // FE_BUB while fetch has an instruction that has not reached the IR (queue latency or a dispatch freeze)
                      HPMEV_RED_BR = 16'h0006,   // Redirect: conditional branch mispredict
                      HPMEV_RED_JLR= 16'h0007,   // Redirect: indirect jump (jalr) target
                      HPMEV_RED_TRP= 16'h0008,   // Redirect: an exception, an interrupt, a dispatch-time or queue-entry fault, or a memory-ordering replay (xRET, sfence.vma, CSR-write redirects and fence.i not)
                      HPMEV_RD_WAIT= 16'h0317,   // A mispredict's restart: from its resolve to its release or its squash at the ROB head
                     HPMEV_DT_WALK= 16'h0318,   // dMMU walker busy (cycles): a second-level lookup or a walk, for a queued load or store or for M's head op
                     HPMEV_DTLB_MISS=16'h0104,  // dMMU walker starts: a request the 64-entry first level does not hold (a second-level lookup, and a walk when that misses)
                     // The memory buckets (2026-09-17): what ST_MEM is made of. Cycles unless noted.
                     HPMEV_MEM_HITSER   = 16'h0319,   // The LQ has a ready candidate the LSU port does not take (a store has the port, the FSM is busy, the D$ read door is full, or its tag's response is due)
                     HPMEV_MEM_LDINFL   = 16'h031a,   // Load reads in flight at the D$, summed per cycle (fast-path tags awaiting their response, and the FSM's load): / MEM_LDSENT is the mean load-read latency
                     HPMEV_MEM_STDOOR   = 16'h031b,   // The LSU presents a write (a store, an AMO's write, a CBO) that the D$ store queue has no room for and no device acknowledges
                     HPMEV_MEM_ALIAS_UNK= 16'h031c,   // An LQ candidate is alias-blocked while a blocked load with its wait bit set waits on an older store whose address is unknown
                     HPMEV_MEM_ALIAS_OVL= 16'h031d,   // An LQ candidate is alias-blocked and no blocked load waits on an unknown older address (a known older store overlaps)
                     HPMEV_MEM_REORD    = 16'h031e,   // Loads the port takes while an older store, committed or not, is live in the store queue (count)
                     HPMEV_MEM_WPKILL   = 16'h031f,   // Loads whose landing is dropped because a kill arrived while their access ran, fast or slow path (count)
                     HPMEV_MEM_DEVWAIT  = 16'h0320,   // No LQ candidate is ready and an unblocked device load waits to be the ROB head
                     HPMEV_MEM_LQOCC    = 16'h0321,   // load-queue occupancy, summed per cycle
                     HPMEV_MEM_SQOCC    = 16'h0322,   // store-queue occupancy, summed per cycle
                     // TOP-DOWN (SmolRV64-Spec 11.2): TD_BS + TD_FE + TD_BE + the dispatching cycles = cycles, exactly
                     HPMEV_TD_BS        = 16'h0401,   // top-down: bad speculation (a redirect, or a resolved restart waiting)
                     HPMEV_TD_BE        = 16'h0402,   // top-down: back-end (M held, the queue head without a credit, a frozen IR, or the refill cycle after one)
                     HPMEV_TD_BE_MEM    = 16'h0403,   // top-down: back-end waiting on memory (td_k 10 or 13: M on a head op, or a dependency on an integer load, AMO or CSR read)
                     HPMEV_TD_BE_ROB    = 16'h0404,   // top-down: back-end, no ROB room (td_k 12)
                     HPMEV_TD_BE_IQ     = 16'h0405,   // top-down: back-end, no scheduler, store-queue or load-queue credit (td_k 15, 17, 18)
                     HPMEV_TD_FE_LAT    = 16'h0406,   // top-down: front-end latency (td_k 5, 6: iMMU, or no fetch bytes)
                     HPMEV_TD_FE        = 16'h0407,   // top-down: front-end (FE_BUB outside bad speculation)
                     HPMEV_DPATCH       = 16'h0408,   // Instructions dispatched (0..IW per cycle)
                     HPMEV_MEM_LDSENT   = 16'h0323,   // Load reads sent to the D$ (count: a fast-path tag issued, or the FSM starting a load)
                     // td_k itself (smolrv64_core's TOP-DOWN block): exactly one per cycle, the TOPDOWN-SIM buckets
                     HPMEV_TDK_OK1      = 16'h0500,   // top-down cause td_k = 0: dispatched 1 (ok:1)
                     HPMEV_TDK_OK2      = 16'h0501,   // top-down cause td_k = 1: dispatched 2 (ok:2)
                     HPMEV_TDK_OK3      = 16'h0502,   // top-down cause td_k = 2: dispatched 3 (ok:3)
                     HPMEV_TDK_REDIRECT = 16'h0503,   // top-down cause td_k = 3: bad speculation: a redirect (bs:redirect)
                     HPMEV_TDK_DRAIN    = 16'h0504,   // top-down cause td_k = 4: bad speculation: a resolved restart waiting (bs:drain)
                     HPMEV_TDK_FE_IMMU  = 16'h0505,   // top-down cause td_k = 5: front-end: the iMMU (fe:immu)
                     HPMEV_TDK_FE_ICACHE= 16'h0506,   // top-down cause td_k = 6: front-end: no fetch bytes (fe:icache)
                     HPMEV_TDK_FE_ALIGN = 16'h0507,   // top-down cause td_k = 7: front-end: bytes but no whole instruction (fe:align)
                     HPMEV_TDK_FE_QUEUE = 16'h0508,   // top-down cause td_k = 8: front-end: an instruction not yet in the IR (fe:queue)
                     HPMEV_TDK_FE_OTHER = 16'h0509,   // top-down cause td_k = 9: front-end: any other (fe:other)
                     HPMEV_TDK_BE_MMEM  = 16'h050a,   // top-down cause td_k = 10: back-end: M holds a head op (be:M-mem)
                     HPMEV_TDK_BE_MOTHER= 16'h050b,   // top-down cause td_k = 11: back-end: M holds a non-memory op (be:M-other)
                     HPMEV_TDK_BE_ROB   = 16'h050c,   // top-down cause td_k = 12: back-end: no ROB room (be:rob-full)
                     HPMEV_TDK_BE_DEPLD = 16'h050d,   // top-down cause td_k = 13: back-end: issue waits on an integer load, AMO or CSR read (be:dep-load)
                     HPMEV_TDK_BE_DEPFP = 16'h050e,   // top-down cause td_k = 14: back-end: issue waits on an FP, FP-load, FP-port or divide result (be:dep-fp)
                     HPMEV_TDK_BE_IQ    = 16'h050f,   // top-down cause td_k = 15: back-end: no scheduler credit at the queue head (be:iq-full)
                     HPMEV_TDK_BE_RN    = 16'h0510,   // top-down cause td_k = 16: back-end: a rename shard below LOWAT, or the map's sweep (be:rename)
                     HPMEV_TDK_BE_SQ    = 16'h0511,   // top-down cause td_k = 17: back-end: no store-queue credit (be:sq-full)
                     HPMEV_TDK_BE_LQ    = 16'h0512,   // top-down cause td_k = 18: back-end: no load-queue credit (be:lq-full)
                     HPMEV_TDK_BE_SER   = 16'h0513,   // top-down cause td_k = 19: back-end: serialisation, a CSR op or a CBO waiting (be:serialize)
                     HPMEV_TDK_BE_HELD  = 16'h0514,   // top-down cause td_k = 20: back-end: the IR held otherwise (be:held)
                     HPMEV_TDK_OK0      = 16'h0515,   // top-down cause td_k = 21: dispatched 0: the take squashed (ok:0)
                     HPMEV_TDK_OK4      = 16'h0516;   // top-down cause td_k = 22: dispatched 4 (ok:4)
   // per-counter increment this cycle for the mhpmeventN-selected event (0..retire_cnt).
   // EVERY input here is a register as far as this module is concerned: `hpm_ev` and
   // `hpm_retire_cnt` are both the caller's delayed copies. That is what keeps the mux and
   // the 64-bit adder below a path that starts at the top of the cycle.
   // THE EVENT SELECT IS DECODED FROM mhpmeventN, into a registered index (hpm_esel).
   // The per-cycle increment is then a mux on a 7-bit register, not a 16-bit event-code case per
   // counter per cycle: with the bus at 41 sources that case was a 644-endpoint family at +0.006 ns
   // (`m_imm_reg -> mhpmcounter_reg`, the C0 IW=3 census) and took the build to WNS 0.000.
   // hpm_inc (the code form) stays as the oracle the assertion below holds hpm_val to.
   // index: 0 cycles, 1 instret, 2+b = hpm_ev[b], 46-49 the counts, 64+n = td_k == n, 127 none
   localparam [6:0] HSEL_NONE = 7'd127, HSEL_CYC = 7'd0, HSEL_RET = 7'd1, HSEL_LQ = 7'd46, HSEL_SQ = 7'd47,
                    HSEL_DSP = 7'd48, HSEL_LDI = 7'd49, HSEL_TDK = 7'd64;
   localparam [7:0] TDK_N = 8'd23;   // td_k's values, 0..22
   function [6:0] hpm_sel_of;   // event code -> index
      input [15:0] ev;
      case (ev)
        HPMEV_CYCLES: hpm_sel_of = HSEL_CYC;    HPMEV_INSTRET: hpm_sel_of = HSEL_RET;
        HPMEV_LOAD: hpm_sel_of = 7'd2+0;   HPMEV_STORE: hpm_sel_of = 7'd2+1;   HPMEV_REDIR: hpm_sel_of = 7'd2+2;
        HPMEV_DCACC: hpm_sel_of = 7'd2+3;  HPMEV_DCMISS: hpm_sel_of = 7'd2+4;  HPMEV_ICACC: hpm_sel_of = 7'd2+5;
        HPMEV_ICMISS: hpm_sel_of = 7'd2+6; HPMEV_ST_MEM: hpm_sel_of = 7'd2+7;  HPMEV_ST_DIV: hpm_sel_of = 7'd2+8;
        HPMEV_ST_MUL: hpm_sel_of = 7'd2+9; HPMEV_ST_FPU: hpm_sel_of = 7'd2+10; HPMEV_ST_DSP: hpm_sel_of = 7'd2+11;
        HPMEV_FE_BUB: hpm_sel_of = 7'd2+12; HPMEV_FE_MMU: hpm_sel_of = 7'd2+13; HPMEV_FE_IC: hpm_sel_of = 7'd2+14;
        HPMEV_RED_BR: hpm_sel_of = 7'd2+15; HPMEV_RED_JLR: hpm_sel_of = 7'd2+16; HPMEV_RED_TRP: hpm_sel_of = 7'd2+17;
        HPMEV_FE_ALN: hpm_sel_of = 7'd2+18; HPMEV_FE_QUE: hpm_sel_of = 7'd2+19; HPMEV_TD_BS: hpm_sel_of = 7'd2+20;
        HPMEV_TD_FE: hpm_sel_of = 7'd2+21;  HPMEV_ST_ROB: hpm_sel_of = 7'd2+22; HPMEV_RD_WAIT: hpm_sel_of = 7'd2+23;
        HPMEV_DT_WALK: hpm_sel_of = 7'd2+24; HPMEV_DTLB_MISS: hpm_sel_of = 7'd2+25; HPMEV_ST_IQ: hpm_sel_of = 7'd2+26;
        HPMEV_ST_RN: hpm_sel_of = 7'd2+27; HPMEV_ST_SQ: hpm_sel_of = 7'd2+28; HPMEV_ST_LQ: hpm_sel_of = 7'd2+29;
        HPMEV_ST_SRZ: hpm_sel_of = 7'd2+30; HPMEV_MEM_HITSER: hpm_sel_of = 7'd2+31; HPMEV_MEM_LDINFL: hpm_sel_of = HSEL_LDI;
        HPMEV_MEM_STDOOR: hpm_sel_of = 7'd2+33; HPMEV_MEM_ALIAS_UNK: hpm_sel_of = 7'd2+34; HPMEV_MEM_ALIAS_OVL: hpm_sel_of = 7'd2+35;
        HPMEV_MEM_REORD: hpm_sel_of = 7'd2+36; HPMEV_MEM_WPKILL: hpm_sel_of = 7'd2+37; HPMEV_MEM_DEVWAIT: hpm_sel_of = 7'd2+38;
        HPMEV_TD_BE: hpm_sel_of = 7'd2+39;  HPMEV_TD_BE_MEM: hpm_sel_of = 7'd2+40; HPMEV_TD_BE_ROB: hpm_sel_of = 7'd2+41;
        HPMEV_TD_BE_IQ: hpm_sel_of = 7'd2+42; HPMEV_TD_FE_LAT: hpm_sel_of = 7'd2+43;
        HPMEV_MEM_LQOCC: hpm_sel_of = HSEL_LQ; HPMEV_MEM_SQOCC: hpm_sel_of = HSEL_SQ; HPMEV_DPATCH: hpm_sel_of = HSEL_DSP;
        HPMEV_MEM_LDSENT: hpm_sel_of = 7'd2+32;
        default: hpm_sel_of = ((ev[15:8] == 8'h05) && (ev[7:0] < TDK_N)) ? (HSEL_TDK | {2'b00, ev[4:0]}) : HSEL_NONE;
      endcase
   endfunction
   function [5:0] hpm_val;      // index -> this cycle's increment
      input [6:0] sel;
      if (sel == HSEL_NONE)      hpm_val = 6'd0;
      else if (sel[6])           hpm_val = {5'd0, hpm_tdk == sel[4:0]};
      else if (sel == HSEL_CYC)  hpm_val = 6'd1;
      else if (sel == HSEL_RET)  hpm_val = hpm_retire_cnt;
      else if (sel == HSEL_LQ)   hpm_val = hpm_lqocc;
      else if (sel == HSEL_SQ)   hpm_val = hpm_sqocc;
      else if (sel == HSEL_DSP)  hpm_val = {3'd0, hpm_disp};
      else if (sel == HSEL_LDI)  hpm_val = hpm_ldinfl;
      else if (sel <= 7'd45)     hpm_val = {5'd0, hpm_ev[sel[5:0] - 6'd2]};
      else                       hpm_val = 6'd0;
   endfunction
   reg [6:0] hpm_esel [0:HPMG-1];
   function [5:0] hpm_inc;
      input [15:0] ev;
      case (ev)
        HPMEV_CYCLES:  hpm_inc = 6'd1;
        HPMEV_INSTRET: hpm_inc = hpm_retire_cnt;
        HPMEV_LOAD:    hpm_inc = {5'd0, hpm_ev[0]};
        HPMEV_STORE:   hpm_inc = {5'd0, hpm_ev[1]};
        HPMEV_REDIR:   hpm_inc = {5'd0, hpm_ev[2]};
        HPMEV_DCACC:   hpm_inc = {5'd0, hpm_ev[3]};
        HPMEV_DCMISS:  hpm_inc = {5'd0, hpm_ev[4]};
        HPMEV_ICACC:   hpm_inc = {5'd0, hpm_ev[5]};
        HPMEV_ICMISS:  hpm_inc = {5'd0, hpm_ev[6]};
        HPMEV_ST_MEM:  hpm_inc = {5'd0, hpm_ev[7]};
        HPMEV_ST_DIV:  hpm_inc = {5'd0, hpm_ev[8]};
        HPMEV_ST_MUL:  hpm_inc = {5'd0, hpm_ev[9]};
        HPMEV_ST_FPU:  hpm_inc = {5'd0, hpm_ev[10]};
        HPMEV_ST_DSP:  hpm_inc = {5'd0, hpm_ev[11]};
        HPMEV_FE_BUB:  hpm_inc = {5'd0, hpm_ev[12]};
        HPMEV_FE_MMU:  hpm_inc = {5'd0, hpm_ev[13]};
        HPMEV_FE_IC:   hpm_inc = {5'd0, hpm_ev[14]};
        HPMEV_RED_BR:  hpm_inc = {5'd0, hpm_ev[15]};
        HPMEV_RED_JLR: hpm_inc = {5'd0, hpm_ev[16]};
        HPMEV_RED_TRP: hpm_inc = {5'd0, hpm_ev[17]};
        HPMEV_FE_ALN:  hpm_inc = {5'd0, hpm_ev[18]};
        HPMEV_FE_QUE:  hpm_inc = {5'd0, hpm_ev[19]};
        HPMEV_TD_BS:   hpm_inc = {5'd0, hpm_ev[20]};
        HPMEV_TD_FE:   hpm_inc = {5'd0, hpm_ev[21]};
        HPMEV_ST_ROB:  hpm_inc = {5'd0, hpm_ev[22]};
        HPMEV_ST_IQ:   hpm_inc = {5'd0, hpm_ev[26]};
        HPMEV_ST_RN:   hpm_inc = {5'd0, hpm_ev[27]};
        HPMEV_ST_SQ:   hpm_inc = {5'd0, hpm_ev[28]};
        HPMEV_ST_LQ:   hpm_inc = {5'd0, hpm_ev[29]};
        HPMEV_ST_SRZ:  hpm_inc = {5'd0, hpm_ev[30]};
        HPMEV_RD_WAIT: hpm_inc = {5'd0, hpm_ev[23]};
        HPMEV_DT_WALK: hpm_inc = {5'd0, hpm_ev[24]};
        HPMEV_DTLB_MISS: hpm_inc = {5'd0, hpm_ev[25]};
        HPMEV_MEM_HITSER:    hpm_inc = {5'd0, hpm_ev[31]};
        HPMEV_MEM_LDINFL:    hpm_inc = hpm_ldinfl;
        HPMEV_MEM_LDSENT:    hpm_inc = {5'd0, hpm_ev[32]};
        HPMEV_MEM_STDOOR:    hpm_inc = {5'd0, hpm_ev[33]};
        HPMEV_MEM_ALIAS_UNK: hpm_inc = {5'd0, hpm_ev[34]};
        HPMEV_MEM_ALIAS_OVL: hpm_inc = {5'd0, hpm_ev[35]};
        HPMEV_MEM_REORD:     hpm_inc = {5'd0, hpm_ev[36]};
        HPMEV_MEM_WPKILL:    hpm_inc = {5'd0, hpm_ev[37]};
        HPMEV_MEM_DEVWAIT:   hpm_inc = {5'd0, hpm_ev[38]};
        HPMEV_MEM_LQOCC:     hpm_inc = hpm_lqocc;
        HPMEV_MEM_SQOCC:     hpm_inc = hpm_sqocc;
        HPMEV_TD_BE:         hpm_inc = {5'd0, hpm_ev[39]};
        HPMEV_TD_BE_MEM:     hpm_inc = {5'd0, hpm_ev[40]};
        HPMEV_TD_BE_ROB:     hpm_inc = {5'd0, hpm_ev[41]};
        HPMEV_TD_BE_IQ:      hpm_inc = {5'd0, hpm_ev[42]};
        HPMEV_TD_FE_LAT:     hpm_inc = {5'd0, hpm_ev[43]};
        HPMEV_DPATCH:        hpm_inc = {3'd0, hpm_disp};
        default:       hpm_inc = ((ev[15:8] == 8'h05) && (ev[7:0] < TDK_N)) ? {5'd0, hpm_tdk == ev[4:0]}
                                                                          : 6'd0;   // unknown -> holds
      endcase
   endfunction
   // The registered select is asserted equal to the code form every cycle (rule I3: a shadow
   // proves the cut before the oracle is deleted).
   always @(posedge clk) if (!reset) for (i=0; i<HPMG; i=i+1)
      if (!hpm_wev_q[i] && (hpm_val(hpm_esel[i]) != hpm_inc(mhpmevent[i][15:0])))
         $fatal(1, "csr_file: hpm counter %0d: registered select %0d gives %0d, the code form %0d", i, hpm_esel[i], hpm_val(hpm_esel[i]), hpm_inc(mhpmevent[i][15:0]));
   reg [63:0] mcountinhibit;
   reg [63:0] mhpmevent  [0:HPMN-1];
   // Sscofpmf, in the top bits of mhpmeventN: OF(63) is set when counter N wraps and raises
   // LCOFIP on its 0 -> 1 edge only, so a set OF silences the counter's interrupt until software
   // clears it; MINH/SINH/UINH (62:60) stop the counter in that privilege. VSINH/VUINH (59:58)
   // are stored and never consulted (no H).
   localparam OF_B = 63, MINH_B = 62, SINH_B = 61, UINH_B = 60;
   localparam [63:0] HPM_EV_MASK = {6'h3f, 42'd0, 16'hffff};   // WARL: OF, the INH bits, the event code
   function [15:0] hpm_fix_code;    // a fixed counter's (index HPMG..) one event: its td_k cause
      input integer n;
      begin hpm_fix_code = {11'h028, HPM_TDK[5*(n-HPMG) +: 5]}; end
   endfunction
   wire [HPMN-1:0] hpm_of;            // mhpmeventN.OF, N = 3.. (scountovf's bits)
   reg [7:0]  fcsr;                  // [7:5]=frm  [4:0]=fflags (NV DZ OF UF NX)
   assign o_frm    = fcsr[7:5];
   assign o_fs_off = (mstatus[14:13] == 2'b00);

`include "va_codec.vh"

   // mstatus as seen on a read: force SXL=UXL=2, and derive SD (bit 63) = any of
   // FS/XS/VS == Dirty (read-only summary; not a stored bit). The riscv-tests v-handler
   // saves/compares sstatus and expects SD set once it dirties FS.
   wire status_sd = (mstatus[14:13] == 2'b11)   // FS  dirty
                  | (mstatus[16:15] == 2'b11)   // XS  dirty
                  | (mstatus[10:9]  == 2'b11);  // VS  dirty
   wire [63:0] mstatus_r = {status_sd, mstatus[62:36], 4'b1010, mstatus[31:0]};

   // effective mip = software-held bits OR the hardware-driven device lines (CLINT/PLIC).
   // hw_ip is 0 when no device is wired (current TB) -> eff_mip == mip, no behavior change.
`ifdef SCDBG
   // Trap/xret tracer (storm-onset autopsy): every delivered trap + sret/mret, budgeted,
   // gated from `SCDBG_T0 (ps). With STCW + IRQDBG this reconstructs the exact
   // interrupt-vs-arm interleaving that drives the clockevent state machine into the
   // stopped-with-pending-level-STIP corner.
`ifndef SCDBG_T0
 `define SCDBG_T0 0
`endif
   reg [31:0] trp_np; initial trp_np = 0;
   reg [31:0] mscw_np; initial mscw_np = 0;
   always @(posedge clk) if ($time > `SCDBG_T0 && trp_np < 32'd60000) begin
      if (trap_v) begin
         $display("[TRAP t=%0t cause=%h epc=%h to_s=%b priv=%0d mtime=%h stip=%b]",
                  $time, trap_cause, trap_epc, trap_to_s, priv, mtime, eff_mip[5]);
         trp_np <= trp_np + 1;
      end else if (do_sret | do_mret) begin
         $display("[XRET t=%0t %s to=%h priv=%0d mtime=%h stip=%b sie_after=%b]",
                  $time, do_sret ? "sret" : "mret", redir_target, priv, mtime, eff_mip[5],
                  do_sret ? mstatus[SPIE_B] : mstatus[MPIE_B]);
         trp_np <= trp_np + 1;
      end
   end
   // TIME-read anomaly tracer (hrtimer-storm hunt): every executed TIME/CYCLE csr read,
   // flagged when non-monotonic or jumping > 1M ticks vs the previous read. A glitched
   // time read poisons hrtimer basenow -> the handler never exits (cosim CANNOT see
   // this: counter/TIME reads are DUT-follow there, unchecked).
   reg [63:0] timr_last;  initial timr_last = 64'd0;
   reg [31:0] timr_nprint; initial timr_nprint = 0;
   always @(posedge clk)
      if (upd_valid && upd_is_csr && (upd_addr == TIME || upd_addr == MTIME)) begin
         if ((mtime < timr_last || mtime - timr_last > 64'd1_000_000) && timr_nprint < 32'd20000) begin
            $display("[TIMR-ANOM t=%0t pc=%h rd=%h last=%h]", $time, upd_pc, mtime, timr_last);
            timr_nprint <= timr_nprint + 1;
         end
         timr_last <= mtime;
      end
`endif
   // ---- timer/interrupt-path debug bus (ILA_TIMER; assembled always, pruned when the
   // wrapper doesn't consume it). Layout documented in rk_xcku5p.v's ila_timer block. ----
   reg         stip_sstc;
   initial     stip_sstc = 1'b0;
   wire dbgt_stw  = upd_valid & upd_is_csr & (upd_addr == STIMECMP);
   wire dbgt_msw  = upd_valid & upd_is_csr & (upd_addr == MSCRATCH);
   assign dbg_mtvec = `VA_UNPACK40(mtvec);
   // 1-cycle strobe when an executing CSR op writes mtvec. Distinguishes "the restore
   // was LOST" (no strobe) from "the restore RAN but wrote the probe-handler address
   // back" (strobe, mtvec unchanged) -- the latter is what a re-executed `csrrw x12,
   // mtvec, x12` swap produces, since the swap is NOT idempotent.
   assign dbg_mtvec_we = upd_valid & upd_is_csr & ~csr_illegal & (upd_addr == MTVEC);
   // Every system op that reaches the update port, tagged with its PC: the ILA can then
   // say whether a given instruction executed at all, independent of whether it changed
   // any state. pc[31:0] is enough -- OpenSBI lives at 0x8000_xxxx.
   assign dbg_csrop_v = upd_valid;
   assign dbg_csrop   = {upd_pc[31:0], 16'h0, upd_addr[11:0], upd_func, upd_is_csr};
   assign dbg_timer = {
      stimecmp[19:0],                        // [63:44] deadline (low bits)
      mtime[23:0],                           // [43:20] now (low bits)
      stip_sstc,                             // [19]    the Sstc comparator (= STIP level w/ STCE), registered
      (mscratch[63:20] == 44'h00000000800),  // [18]    mscratch inside OpenSBI (0x800xxxxx)
      menvcfg[63],                           // [17]    STCE
      do_sret, do_mret,                      // [16:15]
      trap_cause[3:0],                       // [14:11]
      trap_is_intr, trap_to_s, trap_v,       // [10:8]
      dbgt_msw, dbgt_stw,                    // [7:6]   mscratch / stimecmp write strobes
      mie[5], eff_mip[5],                    // [5:4]   STIE / STIP
      mstatus[8], mstatus[1],                // [3:2]   SPP / SIE
      priv };                                // [1:0]
   // Sstc: when menvcfg.STCE, sip/mip.STIP(5) is driven by the stimecmp deadline
   // (read-only to software); otherwise it is the software-/device-written bit.
   // REGISTERED, like the CLINT's mtip. The comparator is four CARRY8 stages on two 64-bit
   // registers, and its live output was the head of the interrupt-take cone: stimecmp ->
   // (mtime >= stimecmp) -> eff_mip -> take_m -> irq_v -> the trap and redirect -> the
   // writeback valid's trap qualifier -> the wakeup -> the issue capture enable, 19 levels,
   // the worst family (+0.014, 559 endpoints under +0.35) of the 2026-09-04 build. A
   // level one cycle late is nothing to a timer that ticks every SCALE_DIV core clocks,
   // and STIP is asynchronous to the instruction stream by definition.
   always @(posedge clk) stip_sstc <= menvcfg[63] & ~SSTC_HIDDEN & (mtime >= stimecmp);
   wire [63:0] base_mip  = mip | {52'd0, hw_ip};
   wire [63:0] eff_mip   = (menvcfg[63] & ~SSTC_HIDDEN) ? {base_mip[63:6], stip_sstc, base_mip[4:0]} : base_mip;

   // ---- Zihpm read decode: mhpmevent3.. / mhpmcounter3.. / hpmcounter3.. (0xC03 shadow) ----
   wire        is_ev = (raddr >= MHPMEVENT3)   && (raddr <= MHPMEVENT3   + 12'd28);
   wire        is_mc = (raddr >= MHPMCOUNTER3) && (raddr <= MHPMCOUNTER3 + 12'd28);
   wire        is_hc = (raddr >= HPMCOUNTER3)  && (raddr <= HPMCOUNTER3  + 12'd28);
   wire [11:0] hpm_ri12 = is_ev ? (raddr - MHPMEVENT3)
                        : is_mc ? (raddr - MHPMCOUNTER3) : (raddr - HPMCOUNTER3);
   wire [4:0]  hpm_ri   = hpm_ri12[4:0];   // 0..28; the 12-bit difference is sliced, not truncated
   wire        hpm_sel   = is_ev | is_mc | is_hc;
   wire [4:0]  hpm_rix   = (hpm_ri < HPMN) ? hpm_ri : 5'd0;            // clamp to a valid entry
   wire [63:0] hpm_ctr_r;                                               // the counter's LUTRAM value (below)
   wire [63:0] hpm_rdata = (hpm_ri >= HPMN) ? 64'd0                    // counters past HPMN hardwired 0
                         : is_ev            ? mhpmevent[hpm_rix] : hpm_ctr_r;

   // ---- combinational read ----
   always @* begin
      case (raddr)
        MSTATUS:    rdata = mstatus_r;
        SSTATUS:    rdata = mstatus_r & SSTATUS_RMASK;
        MISA:       rdata = MISA_VAL;
        MTVEC:      rdata = `VA_UNPACK40(mtvec);
        MEPC:       rdata = `VA_UNPACK40(mepc);
        MCAUSE:     rdata = mcause;
        MTVAL:      rdata = `VA_UNPACK40(mtval);
        MSCRATCH:   rdata = mscratch;
        MIE:        rdata = mie;
        MIP:        rdata = eff_mip;
        SIE:        rdata = mie & S_INT_MASK;
        SIP:        rdata = eff_mip & S_INT_MASK;
        MEDELEG:    rdata = medeleg;
        MIDELEG:    rdata = mideleg;
        MCOUNTEREN: rdata = mcounteren;
        SCOUNTEREN: rdata = scounteren;
        STVEC:      rdata = `VA_UNPACK40(stvec);
        SEPC:       rdata = `VA_UNPACK40(sepc);
        SCAUSE:     rdata = scause;
        STVAL:      rdata = `VA_UNPACK40(stval);
        SSCRATCH:   rdata = sscratch;
        SATP:       rdata = satp;
        MNSTATUS:   rdata = mnstatus;
        STIMECMP:   rdata = stimecmp;
        MENVCFG:    rdata = menvcfg;
        SENVCFG:    rdata = {56'd0, senvcfg[7:0]};
        MSTATEEN0:  rdata = mstateen0;
        // mstateen1..3 / sstateen0..3 are read-only zero: they gate only state this
        // core does not implement, so the default case (0) is the whole model.
        // machine ID CSRs -- match SmolRV64 (marchid=9 = YARVI lineage; vendor/hart=0;
        // mimpid = the build's truncated HEAD commit).
        MVENDORID:  rdata = 64'd0;
        MARCHID:    rdata = 64'd9;
        MIMPID:     rdata = {32'd0, mimpid_q};
        MHARTID:    rdata = 64'd0;
        FFLAGS:     rdata = {59'd0, fcsr[4:0]};
        FRM:        rdata = {61'd0, fcsr[7:5]};
        FCSR:       rdata = {56'd0, fcsr};
        MCYCLE, CYCLE:     rdata = mcycle;
        TIME, MTIME:       rdata = mtime;     // hardware-backed CLINT mtime (0xC01 std + 0xB01 SmolRV64 alias)
        MINSTRET, INSTRET: rdata = minstret;
        // PMP (pmpcfg0-15 / pmpaddr0-63, 0x3A0-0x3FF) reads 0 here and ignores writes
        // (no special case) -> 0 PMP entries implemented, matching SmolRV64. CRITICAL: a
        // single writable entry would make OpenSBI report "PMP Count: 1" and then FAIL
        // root-domain hart isolation ("insufficient PMP entries"); 0 entries makes it skip
        // PMP isolation entirely (PMP is optional). All accesses are permitted (M-mode only).
        MCOUNTINHIBIT: rdata = mcountinhibit & HPM_INHIBIT_MASK;
        // Below M a bit reads 0 unless mcounteren grants that counter; U cannot reach it (0xDA0).
        SCOUNTOVF:  rdata = {{(61-HPMN){1'b0}}, hpm_of, 3'b000} & ((priv == M) ? ~64'd0 : mcounteren);
        // Zihpm mhpmevent3.. / mhpmcounter3.. / hpmcounter3.. resolve via hpm_sel (decoded
        // above); otherwise PMP / unimplemented optional CSRs read 0.
        default:    rdata = hpm_sel ? hpm_rdata : 64'd0;
      endcase
   end

   // ---- CSR rmw new-value from funct3 ----
   // MIP/SIP read as (mip | hw_ip): the live device lines are OR'd into the
   // read value. A csrrs/csrrc based on that READ latches a transient device
   // bit (SEIP while a blk IRQ is in flight, STIP from Sstc) into the stored
   // register -- a stale SEIP then storms spurious external interrupts (PLIC
   // claim reads 0, kernel re-traps forever; the perf-stat board wedge, since
   // OpenSBI's SBI PMU path RMWs mip under IRQ load). Per the priv spec,
   // set/clear on aliased bits must operate on the software-writable bit
   // only, so the RMW base for MIP/SIP is the RAW register; reads keep the OR.
   wire [63:0] rmw_base = (upd_addr == MIP || upd_addr == SIP) ? mip : rdata;
   reg [63:0] newv;
   always @* begin
      case (upd_func[1:0])
        2'b01:   newv = upd_src;                // csrrw/wi
        2'b10:   newv = rmw_base | upd_src;     // csrrs/si
        2'b11:   newv = rmw_base & ~upd_src;    // csrrc/ci
        default: newv = rmw_base;
      endcase
   end

   // ---- system-op decode + trap cause/delegation ----
   wire is_ecall  = ~upd_is_csr & (upd_addr == OP_ECALL);
   wire is_ebreak = ~upd_is_csr & (upd_addr == OP_EBREAK);
   wire is_mret   = ~upd_is_csr & (upd_addr == OP_MRET);
   wire is_sret   = ~upd_is_csr & (upd_addr == OP_SRET);
   // the injected interrupt pseudo-op. It delivers ONLY if an interrupt is still
   // enabled+pending when it executes (oldest); else it retires as a NOP -- so an
   // older op that disabled the interrupt before this point correctly suppresses it.
   wire is_irqop  = ~upd_is_csr & (upd_addr == OP_IRQ);
   wire irq_take  = upd_valid & is_irqop & irq_v;
   // sfence.vma: SYSTEM funct3==0 with funct7==9 (imm[11:5]==7'h09); flush the TLBs
   wire is_sfence = ~upd_is_csr & (upd_addr[11:5] == 7'h09);

   // ---- translation context (consumed by the iMMU/dMMU) ----
   // effective data privilege honors MPRV: when set, accesses use MPP (mstatus[12:11]).
   assign o_satp      = satp;
   assign o_priv      = priv;
   assign o_dpriv     = mstatus[17] ? mstatus[12:11] : priv;
   assign o_sum       = mstatus[18];
   assign o_mxr       = mstatus[19];

   // trap-virtual-memory / trap-SRET (mstatus.TVM=20, TSR=22): in S-mode these make
   // sfence.vma + satp access (TVM) and sret (TSR) trap as illegal -- matches smolrv64.
   wire tvm = mstatus[20];
   wire tsr = mstatus[22];

   // illegal CSR access: writing a read-only CSR (addr[11:10]==11 & the op writes),
   // accessing a CSR that needs higher privilege than current (addr[9:8] > priv), or a
   // satp access while priv==S & TVM.
   wire csr_writes  = upd_is_csr & ((upd_func[1:0]==2'b01) | (upd_src != 64'd0));
   wire csr_ro      = (upd_addr[11:10]==2'b11) & csr_writes;
   wire csr_nopriv  = upd_is_csr & (priv < upd_addr[9:8]);
   wire satp_tvm    = upd_is_csr & (upd_addr == SATP) & (priv == S) & tvm;
   // Unimplemented CSR -> illegal (must trap, not silently read 0). OpenSBI probes optional
   // extensions via trap-to-detect-absence: mtopi (AIA/Smaia, not in RVA22) and the
   // m/sstateen0-3 family (Smstateen, also absent: 0x30C-0x30F / 0x10C-0x10F) -- the trap is
   // how it concludes the extension is missing. Add other unimplemented CSRs here as found.
   // Smstateen is IMPLEMENTED now (see MSTATEEN0), so the family no longer traps as
   // unimplemented. Below M-mode the mstateen0 gates apply instead: ENVCFG guards
   // senvcfg, SE0 guards sstateen0, and sstateen1..3 are always denied because
   // mstateen1..3 are hardwired zero. Matches simmerv cpu.rs::stateen_denies.
   wire csr_stateen = 1'b0;
   wire stateen_ill = upd_is_csr & (priv != M) &
                      ( ((upd_addr == SENVCFG)   & ~mstateen0[62])
                      | ((upd_addr == SSTATEEN0) & ~mstateen0[63])
                      | ((upd_addr[11:2] == 10'h043) & (upd_addr[1:0] != 2'b00)) );
`ifdef NO_SSTC
   // Diagnostic (-DNO_SSTC): hide Sstc entirely so OpenSBI's probe traps and it falls
   // back to the CLINT (mtimecmp -> MTIP -> M-mode forwards mip.STIP) timer path.
   // Motivation: with STCE=1 this core drives mip.STIP SOLELY from the stimecmp
   // comparator, so if OpenSBI ever stops programming stimecmp, STIP is pinned high
   // forever and no CLINT-based re-arm can clear it -- the observed S-timer storm.
   localparam SSTC_HIDDEN = 1'b1;
`else
   localparam SSTC_HIDDEN = 1'b0;
`endif
   wire csr_unimpl  = upd_is_csr & ((upd_addr == MTOPI) | csr_stateen
                                    | (SSTC_HIDDEN & (upd_addr == STIMECMP)));
   // Sstc: stimecmp access in S-mode requires menvcfg.STCE (else illegal). M-mode always
   // allowed; U-mode already blocked by csr_nopriv. (Matches simmerv cpu.rs:1384.)
   wire stce_ill    = upd_is_csr & (upd_addr == STIMECMP) & (priv == S) & ~menvcfg[63];
   assign csr_illegal = upd_valid & (csr_ro | csr_nopriv | satp_tvm | csr_unimpl | stce_ill
                                     | stateen_ill);

   // sfence.vma is illegal in U, or in S with TVM; sret is illegal in U, or in S with TSR.
   wire sfence_illegal = is_sfence & ((priv == U) | ((priv == S) & tvm));
   wire sret_illegal   = is_sret   & ((priv == U) | ((priv == S) & tsr));
   wire sys_illegal    = sfence_illegal | sret_illegal;

   assign o_tlb_flush = upd_valid & ~csr_illegal & ~sfence_illegal
                        & (is_sfence | (upd_is_csr & (upd_addr == SATP)));

   // ---- pending interrupt: highest-priority enabled+pending, per delegation/priv ----
   // M-targeted ints (mip&mie&~mideleg) are taken when priv<M, or priv==M with mstatus.MIE.
   // S-targeted ints (mip&mie&mideleg) are taken when priv<S(=U), or priv==S with SIE; never
   // in M. Priority MEI(11),MSI(3),MTI(7),SEI(9),SSI(1),STI(5) -- matches smolrv64.
   wire        m_glob = (priv == M) ? mstatus[3] : 1'b1;            // mstatus.MIE
   wire        s_glob = (priv == S) ? mstatus[1] : (priv == U);    // mstatus.SIE
   wire [13:0] ip_ie  = eff_mip[13:0] & mie[13:0];
   wire [13:0] pend_m = ip_ie & ~mideleg[13:0];
   wire [13:0] pend_s = ip_ie &  mideleg[13:0];
   wire        take_m = m_glob & (pend_m != 14'd0);
   wire        take_s = ~take_m & s_glob & (pend_s != 14'd0);
   wire [13:0] pend   = take_m ? pend_m : (take_s ? pend_s : 14'd0);
   wire        irq_to_s = take_s;
   assign      irq_v     = take_m | take_s;
   assign      irq_cause = pend[11] ? 4'd11 : pend[3] ? 4'd3 : pend[7] ? 4'd7
                         : pend[9]  ? 4'd9  : pend[1] ? 4'd1 : pend[5] ? 4'd5
                         : 4'd13;                                       // LCOFI(13) last

   // exception this op raises (ecall/ebreak/illegal-CSR/illegal-sfence/sret) + cause + delegation
   wire        exc_active = is_ecall | is_ebreak | csr_illegal | sys_illegal;
   wire [63:0] ecall_cause = (priv==M) ? 64'd11 : (priv==S) ? 64'd9 : 64'd8;
   wire [63:0] exc_cause = (csr_illegal | sys_illegal) ? 64'd2 : is_ebreak ? 64'd3 : ecall_cause;

   // unified trap: a system-op exception OR an external (page-fault / interrupt) injection.
   // sysop_exc is the active system op's OWN exception; trap_v adds external injection. Only
   // sysop_exc may feed redir_valid/do_xret below -- NOT xtrap_v -- else the external trap
   // (which reaches the frontend via csr_redir_tgt, not the shard) would close a combinational
   // loop: xtrap_v -> redir_valid -> exec_shard sys_redirect -> eb_redirect -> (irq gating) -> xtrap_v.
   // a trap is: this system op's OWN exception (ecall/ebreak/illegal), the interrupt
   // pseudo-op delivering (irq_take), or an external page-fault injection (xtrap_v).
   // Interrupts now arrive as the irq_take SYSTEM op (mcause MSB, vectored, epc=op PC);
   // xtrap is exception-only (page faults) so xtrap_intr is vestigial (tied 0).
   wire        sysop_exc  = upd_valid & exc_active;
   wire        trap_is_intr = irq_take | (xtrap_v & xtrap_intr);
   wire        trap_v     = sysop_exc | irq_take | xtrap_v;
   wire [63:0] trap_cause = trap_is_intr ? ({1'b1, 63'd0} | {60'd0, (irq_take ? irq_cause : xtrap_cause)})
                          : xtrap_v      ? {60'd0, xtrap_cause} : exc_cause;  // mcause MSB on intr
   wire [39:0] trap_epc   = xtrap_v ? xtrap_epc : upd_pc;   // irq_take -> upd_pc (interrupted PC)
   wire [63:0] trap_tval  = xtrap_v ? xtrap_tval : (is_ebreak ? `VA_UNPACK40(upd_pc) : 64'd0);
   // delegation: interrupts use the precomputed irq_to_s (mideleg); exceptions use medeleg
   wire        trap_to_s  = trap_is_intr ? irq_to_s
                          : trap_v & (priv != M) & medeleg[trap_cause[5:0]];
   wire        do_mret    = upd_valid & is_mret & ~sysop_exc;
   wire        do_sret    = upd_valid & is_sret & ~sysop_exc;
   // sfence.vma redirects to its fall-through (always a 4-byte insn): this squashes and
   // refetches every younger instruction so any store that was check-translated against the
   // pre-sfence page tables is re-executed (and re-walked) against the flushed/new tables.
   wire        do_sfence  = upd_valid & is_sfence & ~sysop_exc;
   // A write that changes mstatus.FS redirects to fall-through (like sfence): younger FP ops
   // already in flight were evaluated against the old FS, so refetch them to re-evaluate the
   // FS-disabled illegal-instruction trap. FS-changes are rare (context switch) so the cost is
   // negligible; the FP arithmetic tests set FS once at startup and never trip this.
   // FMAX: derive the change-detect below from the mstatus REGISTER, not the generic
   // `rdata` mux. These terms are guarded by upd_addr==MSTATUS/SSTATUS, where rdata is
   // exactly mstatus_r (masked for SSTATUS), so this is bit-for-bit identical -- but it
   // keeps the ~100-entry CSR read mux out of the redirect cone, which post-route is the
   // first ~1.6 ns of the core's critical path (m_imm -> csr_rdata -> csr_redir_v ->
   // iMMU -> predictor). MIP/SIP's rmw_base special case cannot apply at these addresses.
   wire [63:0] ms_rmw_base = (upd_addr == SSTATUS) ? (mstatus_r & SSTATUS_RMASK) : mstatus_r;
   reg  [63:0] newv_ms;
   always @* begin
      case (upd_func[1:0])
        2'b01:   newv_ms = upd_src;             // csrrw/wi
        2'b10:   newv_ms = ms_rmw_base | upd_src;   // csrrs/si
        2'b11:   newv_ms = ms_rmw_base & ~upd_src;  // csrrc/ci
        default: newv_ms = ms_rmw_base;
      endcase
   end
   wire [63:0] newms_m = (mstatus & ~MSTATUS_WMASK) | (newv_ms & MSTATUS_WMASK);
   wire [63:0] newms_s = (mstatus & ~SSTATUS_WMASK) | (newv_ms & SSTATUS_WMASK);
   wire [1:0]  newfs_m = newms_m[14:13];   // mstatus.FS, sliced rather than shift-truncated
   wire [1:0]  newfs_s = newms_s[14:13];
   wire        do_fschg = upd_valid & upd_is_csr & ~csr_illegal &
                          ( ((upd_addr==MSTATUS) & (newfs_m != mstatus[14:13]))
                          | ((upd_addr==SSTATUS) & (newfs_s != mstatus[14:13])) );

   // Likewise, a write that changes a DATA-TRANSLATION context bit of mstatus -- MPRV(17),
   // MPP(12:11, the priv MPRV borrows), SUM(18), MXR(19) -- must redirect to fall-through:
   // o_dpriv/o_sum/o_mxr are combinational on live mstatus, so a younger load/store already in
   // flight was translated against the OLD context. Refetch them to re-translate. Without this,
   // OpenSBI's misaligned-emulation reader (csrrs mstatus,MPRV|MXR; lhu 0(faulting_pc)) runs the
   // lhu under stale MPRV=0 -> an M-mode Bare access to a kernel VA -> spurious access fault (a
   // cosim divergence vs simmerv, which applies the write in order). Same fall-through cost as FS.
   localparam [63:0] MSTATUS_DXMASK = 64'h0000_0000_000E_1800;   // MXR|SUM|MPRV | MPP
   // newms_m/newms_s are declared with the FS check above -- one definition, two users.
   wire        do_dxchg = upd_valid & upd_is_csr & ~csr_illegal &
                          ( ((upd_addr==MSTATUS) & (((newms_m ^ mstatus) & MSTATUS_DXMASK) != 64'd0))
                          | ((upd_addr==SSTATUS) & (((newms_s ^ mstatus) & MSTATUS_DXMASK) != 64'd0)) );

   // A satp WRITE redirects to fall-through: everything younger was FETCHED under the
   // old translation. The kernel's relocate trick depends on this precisely: csrw satp
   // with stvec pre-pointed at the VA continuation, expecting the NEXT sequential fetch
   // to page-fault under the new satp -- no identity mapping, no sfence in between. A
   // frontend that has already fetched past the csrw (easy at a ~3-cycle window supply)
   // would otherwise execute those stale bare-fetched bytes instead of faulting. Same
   // fall-through recipe as FS/MPRV; satp writes are context-switch-rare.
   wire        do_satp  = upd_valid & upd_is_csr & ~csr_illegal & csr_writes
                        & (upd_addr == SATP);
   // A CSR op does not drain: younger work runs while it waits for the head. A write that changes
   // what younger ops already used refetches them too: the rounding mode (frm, fcsr) an FP op read
   // at execute, and the enables (menvcfg, senvcfg, mstateen0) decode judged a CBO by and the
   // translation honours (PBMTE). Rare writes, at the cost of FS's refetch.
   wire        do_ctxw  = upd_valid & upd_is_csr & ~csr_illegal & csr_writes
                        & ((upd_addr == FRM) | (upd_addr == FCSR) | (upd_addr == MENVCFG)
                         | (upd_addr == SENVCFG) | (upd_addr == MSTATEEN0));

   // redir_valid/redir_is_trap reflect only the active system op (consumed by exec_shard);
   // external injections (xtrap_v) redirect via csr_redir_tgt in backend_top instead.
   assign redir_valid   = sysop_exc | irq_take | do_mret | do_sret | do_sfence | do_fschg | do_dxchg | do_satp | do_ctxw;
   assign redir_is_trap = sysop_exc | irq_take;     // op's own exception OR delivered interrupt
   // ---- redirect target (combinational) ----
   // An external injection (xtrap_v: page fault / interrupt) takes priority over a coincident
   // xret so backend_top's csr_redir_tgt is the trap vector. Vectored mode (tvec[0]) sends an
   // interrupt to base + 4*cause; exceptions and direct mode go to base.
   wire [39:0] tvec_base = trap_to_s ? {stvec[39:2], 2'b0} : {mtvec[39:2], 2'b0};
   wire        tvec_vec  = trap_is_intr & (trap_to_s ? stvec[0] : mtvec[0]);
   wire [39:0] trap_tgt  = tvec_vec ? (tvec_base + {32'd0, trap_cause[5:0], 2'b00}) : tvec_base;
   always @* begin
      if (xtrap_v)              redir_target = trap_tgt;
      else if (do_mret)         redir_target = mepc;
      else if (do_sret)         redir_target = sepc;
      else if (do_sfence)       redir_target = upd_pc + 40'd4;
      else if (do_fschg)        redir_target = upd_pc + 40'd4;   // CSR op is 4 bytes
      else if (do_dxchg)        redir_target = upd_pc + 40'd4;   // CSR op is 4 bytes
      else if (do_satp)         redir_target = upd_pc + 40'd4;   // refetch under the new satp
      else if (do_ctxw)         redir_target = upd_pc + 40'd4;   // refetch under the new frm/enables
      else                      redir_target = trap_tgt;
   end

   localparam MIE_B=3, SIE_B=1, MPIE_B=7, SPIE_B=5, SPP_B=8;  // [12:11]=MPP

   // ---- Zihpm/Sscofpmf per-counter terms ----
   // A counter-CSR write is an op that writes (a csrr of mhpmeventN rewrites nothing, so it
   // cannot swallow that cycle's increment or overflow). A counter runs unless mcountinhibit or
   // its event's *INH bit for the current privilege stops it; it wraps when the add carries out.
   wire hpm_csr_we = upd_valid & upd_is_csr & csr_writes & ~trap_v & ~csr_illegal;
   wire [HPMN-1:0] hpm_wev, hpm_wct, hpm_run, hpm_wrap;
   // A COUNTER WRITE LOADS A CYCLE LATE. A CSR write's value comes through the read mux and the
   // set/clear (newv); loading it the same cycle would put that cone in front of the counters'
   // adders. The write is registered here (cw_*) and loads the next cycle: mcycle and minstret
   // over that cycle's sum (the sum of the write's own cycle is overwritten too), mhpmcounterN
   // into the LUTRAM with that cycle's count left in its accumulator (below). minstret's writer
   // retires into the count a cycle after the write, so the load drops it, as a write must.
   reg             cw_mcycle, cw_minstret;
   reg  [HPMN-1:0] cw_hpm;
   reg  [63:0]     cw_d;
   always @(posedge clk) begin
      cw_mcycle   <= ~reset & upd_valid & upd_is_csr & csr_writes & ~trap_v & (upd_addr == MCYCLE);
      cw_minstret <= ~reset & upd_valid & upd_is_csr & csr_writes & ~trap_v & (upd_addr == MINSTRET);
      cw_hpm      <= reset ? {HPMN{1'b0}} : hpm_wct;
      cw_d        <= newv;
   end
   // The select is decoded a cycle after mhpmeventN is written, from the registered write data
   // (cw_d, what the CSR now holds) by ONE decoder: the CSR's write data (the read mux and the
   // set/clear) never reaches hpm_esel. In that cycle the select is the old event's, so the
   // counter holds (hpm_wev_q).
   reg  [HPMN-1:0] hpm_wev_q;
   integer         hi;
   wire [6:0]      hpm_sel_w = hpm_sel_of(cw_d[15:0]);   // checked against the stored event below
   always @(posedge clk) begin
      hpm_wev_q <= reset ? {HPMN{1'b0}} : hpm_wev;
      for (hi = 0; hi < HPMG; hi = hi + 1)
         if (reset) hpm_esel[hi] <= HSEL_NONE;
         else if (hpm_wev_q[hi]) hpm_esel[hi] <= hpm_sel_w;
   end
   // THE COUNTERS ARE SWEPT. A counter's value lives in one LUTRAM; what it counts goes first into
   // its accumulator, and a sweep folds one accumulator a cycle into its value through one 64-bit
   // adder, so every counter is folded at least once in HPMN cycles. A READ IS EXACT: it returns the
   // value plus the counter's accumulator (one more adder, on the read port), so software that reads
   // a counter and writes it back -- a sampling handler's stop, read, preload -- loses only the
   // cycles between its read and its write, as with a live counter. A wrap is the fold's carry:
   // OF and LCOFIP follow it at most a sweep late, which is skid, not lost counts. A counter write
   // (cw_hpm, a cycle late as before) takes the write port that cycle, the sweep holds, and the
   // written counter's accumulator restarts empty; a wrap of the value it replaces sets OF.
   // After reset the sweep writes zeros once round.
   localparam integer HSW = 5;                       // sweep index bits
   localparam integer HAW = 12;                      // a general accumulator: 63 a cycle for 65 cycles
   localparam integer HAF = 7;                       // a fixed one: 1 a cycle for 127 cycles
   localparam integer HLI = HPMN - 1;
   localparam [HSW-1:0] HLAST = HLI[HSW-1:0];
   (* ram_style = "distributed" *) reg [63:0] hpm_ctr [0:(1<<HSW)-1];
   wire [HAW-1:0] hpm_acc [0:HPMN-1];                // every accumulator, zero-extended
   reg  [HSW-1:0] hpm_sp;                            // the counter the sweep folds this cycle
   reg            hpm_clr;                           // the zeroing round after reset
   reg  [HSW-1:0] cw_ix;
   integer        ci;
   always @* begin
      cw_ix = {HSW{1'b0}};
      for (ci = 0; ci < HPMN; ci = ci + 1) if (cw_hpm[ci]) cw_ix = ci[HSW-1:0];
   end
   // The adder sums the counter the write port writes: the sweep's, or a written counter's, so a
   // wrap still in its accumulator sets OF before the written value replaces it.
   wire           cw_any  = |cw_hpm;
   wire [HSW-1:0] ctr_wa  = cw_any ? cw_ix : hpm_sp;
   wire [64:0]    ctr_sum = {1'b0, hpm_ctr[ctr_wa]} + {{(65-HAW){1'b0}}, hpm_acc[ctr_wa]};
   wire [63:0]    ctr_wd  = cw_any ? cw_d : hpm_clr ? 64'd0 : ctr_sum[63:0];
   wire           wrap_v  = ctr_sum[64] & ~hpm_clr;  // the summed counter wrapped
   assign hpm_ctr_r = hpm_ctr[hpm_rix] + {{(64-HAW){1'b0}}, hpm_acc[hpm_rix]};
   always @(posedge clk) hpm_ctr[ctr_wa] <= ctr_wd;  // the one write
   always @(posedge clk) begin
      if (reset) begin hpm_sp <= {HSW{1'b0}};  hpm_clr <= 1'b1; end
      else if (!cw_any) begin
         hpm_sp <= (hpm_sp == HLAST) ? {HSW{1'b0}} : hpm_sp + 1'b1;
         if (hpm_sp == HLAST) hpm_clr <= 1'b0;
      end
   end
   genvar g;
   generate for (g = 0; g < HPMN; g = g + 1) begin : g_hpm
      localparam [11:0] EVA = MHPMEVENT3 + g, CTA = MHPMCOUNTER3 + g;
      localparam integer W = (g < HPMG) ? HAW : HAF;
      wire        inh = (priv == M) ? mhpmevent[g][MINH_B]
                      : (priv == S) ? mhpmevent[g][SINH_B] : mhpmevent[g][UINH_B];
      wire        take = cw_hpm[g] | (~cw_any & (hpm_sp == g));   // the accumulator empties
      wire [5:0]  inc;
      reg  [W-1:0] acc;
      wire [W:0]  nx = {1'b0, take ? {W{1'b0}} : acc} + {{(W-5){1'b0}}, inc};
      always @(posedge clk) begin
         acc <= reset ? {W{1'b0}} : nx[W-1:0];
         if (!reset && nx[W]) $fatal(1, "csr_file: hpm counter %0d's accumulator overflowed", g + 3);
      end
      assign hpm_acc[g] = {{(HAW-W){1'b0}}, acc};
      if (g < HPMG) begin : g_gen
         assign inc        = hpm_run[g] ? hpm_val(hpm_esel[g]) : 6'd0;
         assign hpm_run[g] = ~mcountinhibit[g+3] & ~inh & ~hpm_wev_q[g];
      end else begin : g_fix
         localparam [4:0]   TK  = HPM_TDK[5*(g-HPMG) +: 5];
         assign inc        = {5'd0, hpm_run[g] & (hpm_tdk == TK)};
         assign hpm_run[g] = ~mcountinhibit[g+3] & ~inh & (mhpmevent[g][15:0] == {11'h028, TK});
      end
      assign hpm_wev[g]  = hpm_csr_we & (upd_addr == EVA);
      assign hpm_wct[g]  = hpm_csr_we & (upd_addr == CTA);
      assign hpm_wrap[g] = wrap_v & (ctr_wa == g);
      assign hpm_of[g]   = mhpmevent[g][OF_B];
   end endgenerate
   // A wrap with OF clear sets OF and LCOFIP; an mhpmeventN write that cycle wins.
   wire hpm_lcof = |(hpm_wrap & ~hpm_wev & ~hpm_of);

   // THE PRIVILEGE'S ONE UPDATE: a trap's target, an xret's previous mode, M at reset. With satp's
   // MODE as the write below leaves it, it says whether fetch translates next cycle, so the front
   // end registers that (o_ifx_d) and its regime test reads a flop, not this file's registers.
   wire [1:0] priv_d = reset ? M
                     : trap_v ? (trap_to_s ? S : M)
                     : (upd_valid & ~upd_is_csr & is_mret) ? mstatus[12:11]
                     : (upd_valid & ~upd_is_csr & is_sret) ? {1'b0, mstatus[SPP_B]} : priv;
   wire [3:0] satp_mode_d = reset ? 4'd0
                          : (~trap_v & upd_valid & upd_is_csr & (upd_addr == SATP)) ? newv[63:60] : satp[63:60];
   assign o_ifx_d = (priv_d != M) & (satp_mode_d == 4'd8);
   always @(posedge clk) priv <= priv_d;
   reg [3:0] satp_mode_q;
   always @(posedge clk) begin
      satp_mode_q <= satp_mode_d;
      if (!reset && (satp_mode_q != satp[63:60]))
         $fatal(1, "csr_file: satp's MODE %0d is not the %0d its next-value wire said", satp[63:60], satp_mode_q);
   end

   integer i;
   always @(posedge clk) begin
      if (reset) begin
         mstatus<=0; mtvec<=0; mepc<=0; mcause<=0; mtval<=0; mscratch<=0;
         mie<=0; mip<=0; medeleg<=0; mideleg<=0; mcounteren<=0; satp<=0;
         mnstatus<=0;
         stvec<=0; sepc<=0; scause<=0; stval<=0; sscratch<=0; scounteren<=0;
         fcsr<=0; stimecmp<=~64'd0; menvcfg<=64'd0; senvcfg<=64'd0;   // Sstc: stimecmp resets to "no deadline"
         mstateen0<=64'hC000_0000_0000_0000;   // SE0|ENVCFG set: no stateen restrictions
      end else if (trap_v) begin
         // trap (system-op exception OR external page fault); target priv per delegation
         if (trap_to_s) begin
            sepc   <= trap_epc;
            scause <= trap_cause;
            stval  <= `VA_PACK40(trap_tval);
            mstatus[SPIE_B] <= mstatus[SIE_B]; mstatus[SIE_B] <= 1'b0;
            mstatus[SPP_B]  <= priv[0];
         end else begin
            mepc   <= trap_epc;
            mcause <= trap_cause;
            mtval  <= `VA_PACK40(trap_tval);
            mstatus[MPIE_B] <= mstatus[MIE_B]; mstatus[MIE_B] <= 1'b0;
            mstatus[12:11]  <= priv;
         end
      end else if (upd_valid) begin
         if (upd_is_csr) begin
            case (upd_addr)
              MSTATUS:    mstatus <= (mstatus & ~MSTATUS_WMASK) | (newv & MSTATUS_WMASK);
              SSTATUS:    mstatus <= (mstatus & ~SSTATUS_WMASK) | (newv & SSTATUS_WMASK);
              MTVEC:      mtvec   <= `VA_PACK40(newv);
              MEPC:       mepc    <= `VA_PACK40(newv);
              MCAUSE:     mcause  <= newv;
              MTVAL:      mtval   <= `VA_PACK40(newv);
              MSCRATCH:   begin
                 mscratch<= newv;
`ifdef SCDBG
                 // corruption hunt: mscratch should be written ONCE at OpenSBI init and
                 // then only by the trap-entry/exit csrrw swap pairs. Log every write
                 // late in boot (budgeted -- the storm ecalls swap it constantly) -- a
                 // swap pair that doesn't restore the scratch pointer (replay/double-
                 // execution) is the suspected corruption at c~10.7413B.
                 if ($time > `SCDBG_T0 && mscw_np < 32'd120000) begin
                    $display("[MSCW t=%0t pc=%h new=%h priv=%0d]", $time, upd_pc, newv, priv);
                    mscw_np <= mscw_np + 1;
                 end
`endif
              end
              MIE:        mie     <= newv;
              MIP:        mip     <= newv & ~HW_RO_MASK;  // MEIP/MTIP/MSIP read-only (device-owned)
              SIE:        mie     <= (mie & ~S_INT_MASK) | (newv & S_INT_MASK);
              SIP:        mip     <= (mip & ~S_INT_MASK) | (newv & S_INT_MASK);
              MEDELEG:    medeleg <= newv;
              MIDELEG:    mideleg <= newv;
              MCOUNTEREN: mcounteren <= newv;
              SCOUNTEREN: scounteren <= newv;
              STVEC:      stvec   <= `VA_PACK40(newv);
              SEPC:       sepc    <= `VA_PACK40(newv);
              SCAUSE:     scause  <= newv;
              STVAL:      stval   <= `VA_PACK40(newv);
              SSCRATCH:   sscratch<= newv;
              // satp.ASID is 10-bit WARL (matches SmolRV64 TLB_ASID_BITS=10 + simmerv):
              // zero the unimplemented high ASID bits [59:54] so a read-back matches the
              // reference (Linux probes ASID width by writing all-ones and reading back).
              SATP:       satp    <= newv & ~64'h0FC0_0000_0000_0000;
              MNSTATUS:   mnstatus<= newv;
              STIMECMP:   begin
                 stimecmp<= newv;       // Sstc (stored verbatim, like simmerv)
`ifdef SCDBG
                 // storm autopsy: every stimecmp arm, with the deadline's relation to now.
                 // "the kernel stopped re-arming" vs "the DUT lost the write" discriminator.
                 $display("[STCW t=%0t pc=%h new=%h mtime=%h %s]", $time, upd_pc, newv, mtime,
                          (newv > mtime) ? "future" : "PAST");
`endif
              end
              MENVCFG:    menvcfg <= newv;
              SENVCFG:    senvcfg <= newv & 64'h00000000000000f1;  // FIOM + CBZE/CBCFE/CBIE (SmolRV64 mask)
              MSTATEEN0:  mstateen0 <= newv & 64'hC000_0000_0000_0000;   // SE0|ENVCFG only
              FFLAGS:     fcsr[4:0] <= newv[4:0];
              FRM:        fcsr[7:5] <= newv[2:0];
              FCSR:       fcsr      <= newv[7:0];
              default:    ;
            endcase
         end else if (is_mret) begin
            mstatus[MIE_B]  <= mstatus[MPIE_B]; mstatus[MPIE_B] <= 1'b1;
            mstatus[12:11]  <= U;
         end else if (is_sret) begin
            mstatus[SIE_B]  <= mstatus[SPIE_B]; mstatus[SPIE_B] <= 1'b1;
            mstatus[SPP_B]  <= 1'b0;
         end
      end
      // FP exception flags accumulate as their ops retire (the caller ORs the retiring ops'
      // flags). Last write to fcsr[4:0] this cycle, so it ORs on top of a coincident fcsr CSR
      // write, which is older.
      if (!reset && fp_fflags_we) fcsr[4:0] <= fcsr[4:0] | fp_fflags;
      // FS -> Dirty when a retiring op changed FP state: an f-register or retired flags
      // (fp_dirty_commit) or an FP CSR write (fcsr/fflags/frm, itself commit-serialized).
      // Guarded to FP-enabled (FS != Off).
      if (!reset && (mstatus[14:13] != 2'b00) &&
          (fp_dirty_commit | (upd_valid & upd_is_csr & csr_writes & ~trap_v & ~csr_illegal
                              & ((upd_addr==FCSR) | (upd_addr==FFLAGS) | (upd_addr==FRM)))))
         mstatus[14:13] <= 2'b11;
      // Zicntr counters (off the trap/csr chain so they tick every cycle). mcycle counts
      // clocks; minstret adds the committing checkpoint's instruction count. An M-mode
      // write to mcycle/minstret loads the value a cycle late (cw_*, with the Zihpm counters').
      // retire_cnt may be the caller's DELAYED count (smolrv64_core passes the same registered copy
      // it gives the Zihpm counters, and holds a CSR op at the ROB head for a second cycle so
      // the lag is invisible to a read). A minstret WRITE then sees the writing instruction's
      // own retirement arrive one cycle after the write, the cycle the write loads over it, so the
      // value after `csrw minstret` is the written value exactly as with a live count; a read
      // (csrr) writes nothing and counts itself.
      if (reset) begin
         mcycle <= 64'd0; minstret <= 64'd0;
      end else begin
         mcycle   <= cw_mcycle   ? cw_d : mcycle   + (mcountinhibit[0] ? 64'd0 : 64'd1);
         minstret <= cw_minstret ? cw_d : minstret + (mcountinhibit[2] ? 64'd0 : {58'd0, retire_cnt});
      end
      // Zihpm events (the counters themselves are swept, above). A wrap sets OF (Sscofpmf);
      // an mhpmeventN write that cycle wins. mhpmeventN and mcountinhibit are WARL: a general
      // counter's event keeps OF, the INH bits and the code, a fixed one's code only its cause's
      // or 0; an unimplemented code contributes 0 (the counter holds).
      if (reset) begin
         mcountinhibit <= 64'd0;
         for (i=0; i<HPMN; i=i+1) mhpmevent[i] <= 64'd0;
      end else begin
         if (upd_valid && upd_is_csr && !trap_v && !csr_illegal && upd_addr==MCOUNTINHIBIT)
            mcountinhibit <= newv & HPM_INHIBIT_MASK;
         for (i=0; i<HPMN; i=i+1) begin
            if (hpm_wev[i])
               mhpmevent[i] <= (i < HPMG) ? (newv & HPM_EV_MASK)
                             : {newv[63:58], 42'd0, (newv[15:0] == hpm_fix_code(i)) ? newv[15:0] : 16'd0};
            else if (hpm_wrap[i])
               mhpmevent[i][OF_B] <= 1'b1;
         end
         // After the mip CSR write above: an overflow in the cycle software clears LCOFIP stays.
         if (hpm_lcof) mip[LCOFIP_B] <= 1'b1;
      end
   end
endmodule

`default_nettype wire
