`default_nettype none

// mimpid = the truncated git commit of the build's HEAD, injected by the build
// (build.tcl for FPGA, the sim run-scripts for simulation) -- mirrors SmolRV64's
// SMOLRV64_GIT_COMMIT mechanism. Fallback 0 for an un-passed build.
`ifndef SMOLRV64_GIT_COMMIT
 `define SMOLRV64_GIT_COMMIT 32'h0
`endif

// Architectural CSR state for the sharded-OoO backend, M/S/U privilege model.
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
    output reg  [63:0] redir_target,
    output wire        redir_valid,   // this op redirects (trap / xret / illegal-CSR)
    output wire        redir_is_trap, // redirect is an exception (roll back TO this op's ckpt)
    output wire        csr_illegal,   // active CSR op is an illegal access (suppress rd)
    // ---- translation context for the iMMU/dMMU (combinational) ----
    output wire [63:0] o_satp,        // satp (MODE/ASID/PPN)
    output wire [1:0]  o_priv,        // current privilege (instruction-fetch priv)
    output wire [1:0]  o_dpriv,       // effective data-access priv (honors MPRV/MPP)
    output wire        o_sum,         // mstatus.SUM
    output wire        o_mxr,         // mstatus.MXR
    output wire [2:0]  o_frm,         // fcsr.frm -> FP units for dynamic rounding
    output wire        o_fs_off,      // mstatus.FS==Off -> FP instructions trap illegal
    // FP exception-flag accumulation: OR fp_fflags into fcsr.fflags when fp_fflags_we
    // (pre-reduced across shards in backend_top at FP completion).
    input  wire        fp_fflags_we,
    input  wire [4:0]  fp_fflags,
    // mstatus.FS -> Dirty when an FP-state writer RETIRES: commit_ctl raises this at the commit
    // of any checkpoint that held an f-register write (FP arith/compare, in-core FSGNJ/FMV.x.X,
    // FP loads FLW/FLD). Commit-gated (never speculative) so a squashed FP op never dirties FS.
    input  wire        fp_dirty_commit,
    output wire        o_tlb_flush,   // 1-cycle: sfence.vma or satp write -> flush TLBs
    // ---- external trap injection (page faults from the iMMU/LSU; precise) ----
    // Fired by backend_top once the fault is the oldest (fetch: pipeline empty; data:
    // rolled back to the faulting checkpoint). Mutually exclusive with a system op.
    input  wire        xtrap_v,
    input  wire        xtrap_intr,    // the injected trap is an interrupt (mcause MSB, vectored)
    input  wire [3:0]  xtrap_cause,   // exception: 12/13/15 page fault; interrupt: cause number
    input  wire [63:0] xtrap_epc,     // resume PC (faulting VA / faulting bundle start)
    input  wire [63:0] xtrap_tval,    // faulting virtual address
    // ---- hardware interrupt-pending lines (from CLINT/PLIC, combinational) ----
    // The device-owned mip bits: MEIP(11)/SEIP(9)/MTIP(7)/STIP(5)/MSIP(3). OR'd into
    // the effective mip; the read-only set (MEIP/MTIP/MSIP) is masked out of CSR writes
    // so software clears them only at the device (mtimecmp/msip), never via mip.
    input  wire [11:0] hw_ip,
    input  wire [63:0] mtime,       // free-running CLINT time (Sstc stimecmp compare); 0 in device-less TBs
    input  wire [5:0]  retire_cnt,  // # instructions retiring this cycle (commit_ctl) -> minstret
                                    // (a coarse checkpoint retires up to CKMAX at once)
    // Zihpm event pulses (each +1/cycle when high) selected per counter by mhpmeventN:
    // [0]load [1]store [2]redirect(branch mispredict) [3]dc-access [4]dc-miss [5]ic-access [6]ic-miss
    // [6:0] are the original per-op/cache taps. [14:7] are the in-order core's
    // STALL-ATTRIBUTION taps (see ooo2_core.v): they turn a CPI number into a CPI
    // stack. The OoO core drives them zero, so its counters are unchanged.
    input  wire [21:0] hpm_ev,
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
    input  wire [63:0] upd_pc);      // the system op's PC

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
                     MHPMCOUNTER3=12'hB03, HPMCOUNTER3=12'hC03;

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
   // interrupt-enable/pending S-visible bits (SSIE/STIE/SEIE = 1,5,9)
   localparam [63:0] S_INT_MASK = 64'h0000_0000_0000_0222;
   // mip bits owned read-only by hardware (MEIP/MTIP/MSIP = 11,7,3): software MIP
   // writes can't touch them (they reflect the device lines, cleared at the device).
   localparam [63:0] HW_RO_MASK = 64'h0000_0000_0000_0888;

   reg [1:0]  priv;
   reg [63:0] mstatus, mtvec, mcause, mscratch, mie, mip,
              medeleg, mideleg, mcounteren, satp, mnstatus,
              stimecmp, menvcfg,   // Sstc: supervisor timer-compare + menvcfg.STCE enable
              senvcfg,             // S-mode envcfg (FIOM + Zicbom/Zicboz U-mode CBO enables)
              mstateen0;           // Smstateen: only SE0(63)/ENVCFG(62) implemented
   reg [63:0] stvec, scause, sscratch, scounteren;
   // VA-holding CSRs stored 40-bit Sv39-compressed (low 39 + non-canonical flag):
   // mepc/sepc can capture a non-canonical fetch-fault target, mtval/stval the
   // faulting VA -- both need the flag.  Non-VA tval values (illegal-instr encoding,
   // 0) fit in 39 bits with bit38=0 so they round-trip exactly.  See va_codec.vh.
   reg [39:0] mepc, mtval, sepc, stval;
   reg [63:0] mcycle, minstret;      // Zicntr: free-running cycles + retired instructions

   // Zihpm: HPMN programmable counters mhpmcounter3 .. mhpmcounter(2+HPMN) (matches the DTB's
   // event->counter map, counters 3..15). Each counterN adds, per cycle, the retire-count or 1
   // for its mhpmeventN-selected event (SmolRV64 encoding); events not yet tapped read as 0 so
   // the counter simply stays put -- a clean extension point (Phase 2 wires branch/cache/TLB).
   localparam integer HPMN = 13;                                     // counters 3..15
   localparam [63:0]  HPM_INHIBIT_MASK = (((64'h1 << (HPMN+3)) - 1) & ~64'h2); // 0,2,3..15 (not TIME)
   // THE event map. docs/smolrv64-perf-events.json is GENERATED from this block by
   // tools/gen-perf-events.py -- do not edit the JSON by hand, and keep one event per line
   // with a trailing // description, because that is what the generator parses.
   //
   // The 0x03xx block is the in-order stall attribution: every cycle the pipe fails to
   // advance is charged to exactly one of these, so sum(stalls)/instret + 1 reconstructs
   // CPI. 0x02xx is avoided -- the DTB already maps perf BUS_CYCLES onto 0x0202.
   localparam [15:0]  HPMEV_CYCLES = 16'h0001,   // Clock cycles while this event is selected
                      HPMEV_INSTRET= 16'h0002,   // Instructions retired
                      HPMEV_LOAD   = 16'h0003,   // Load completions (LSU)
                      HPMEV_STORE  = 16'h0004,   // Store completions (LSU)
                      HPMEV_REDIR  = 16'h0005,   // Pipeline redirect (branch mispredict / flush)
                      HPMEV_DCACC  = 16'h0100,   // D$ line lookups resolved (hit or miss)
                      HPMEV_DCMISS = 16'h0102,   // D$ line lookups that missed
                      HPMEV_ICACC  = 16'h0110,   // I$ line lookups resolved (hit or miss)
                      HPMEV_ICMISS = 16'h0112,   // I$ line lookups that missed
                      HPMEV_ST_MEM = 16'h0300,   // M stalled on the LSU (D$/dTLB/AMO)
                      HPMEV_ST_DIV = 16'h0301,   // ...on the iterative divider
                      HPMEV_ST_MUL = 16'h0302,   // ...on the 3-cycle multiplier
                      HPMEV_ST_FPU = 16'h0303,   // ...on the CVFPU
                      HPMEV_ST_SER = 16'h0304,   // serializing op holds the frontend off
                      HPMEV_FE_BUB = 16'h0310,   // X idle: frontend supplied no instruction
                      HPMEV_FE_MMU = 16'h0311,   // ...because the iMMU was walking
                      HPMEV_FE_IC  = 16'h0312,   // ...because the fetch window was empty
                      HPMEV_FE_ALN = 16'h0313,   // ...had bytes but no complete instruction
                      HPMEV_FE_QUE = 16'h0314,   // ...had an instruction; F/X queue empty
                      HPMEV_RED_BR = 16'h0006,   // Redirect: conditional branch mispredict
                      HPMEV_RED_JLR= 16'h0007,   // Redirect: indirect jump (jalr) target
                      HPMEV_RED_TRP= 16'h0008,   // Redirect: trap / exception / system op
                      HPMEV_FB_HIT = 16'h0315,   // Fetch buffer served the PC (hit)
                      HPMEV_FB_RHIT= 16'h0316;   // ...on the first fetch after a redirect
   // per-counter increment this cycle for the mhpmeventN-selected event (0..retire_cnt).
   function [5:0] hpm_inc;
      input [15:0] ev;
      case (ev)
        HPMEV_CYCLES:  hpm_inc = 6'd1;
        HPMEV_INSTRET: hpm_inc = retire_cnt;
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
        HPMEV_ST_SER:  hpm_inc = {5'd0, hpm_ev[11]};
        HPMEV_FE_BUB:  hpm_inc = {5'd0, hpm_ev[12]};
        HPMEV_FE_MMU:  hpm_inc = {5'd0, hpm_ev[13]};
        HPMEV_FE_IC:   hpm_inc = {5'd0, hpm_ev[14]};
        HPMEV_RED_BR:  hpm_inc = {5'd0, hpm_ev[15]};
        HPMEV_RED_JLR: hpm_inc = {5'd0, hpm_ev[16]};
        HPMEV_RED_TRP: hpm_inc = {5'd0, hpm_ev[17]};
        HPMEV_FE_ALN:  hpm_inc = {5'd0, hpm_ev[18]};
        HPMEV_FE_QUE:  hpm_inc = {5'd0, hpm_ev[19]};
        HPMEV_FB_HIT:  hpm_inc = {5'd0, hpm_ev[20]};
        HPMEV_FB_RHIT: hpm_inc = {5'd0, hpm_ev[21]};
        default:       hpm_inc = 6'd0;   // unimplemented event -> counter holds
      endcase
   endfunction
   reg [63:0] mcountinhibit;
   reg [63:0] mhpmevent  [0:HPMN-1];
   reg [63:0] mhpmcounter[0:HPMN-1];
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
   wire dbgt_stw  = upd_valid & upd_is_csr & (upd_addr == STIMECMP);
   wire dbgt_msw  = upd_valid & upd_is_csr & (upd_addr == MSCRATCH);
   assign dbg_mtvec = mtvec;
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
      (mtime >= stimecmp),                   // [19]    raw Sstc comparator (= STIP level w/ STCE)
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
   wire        stip_sstc = menvcfg[63] & ~SSTC_HIDDEN & (mtime >= stimecmp);
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
   wire [3:0]  hpm_rix   = (hpm_ri < HPMN) ? hpm_ri[3:0] : 4'd0;       // clamp to a valid entry
   wire [63:0] hpm_rdata = (hpm_ri >= HPMN) ? 64'd0                    // counters 16..31 hardwired 0
                         : is_ev            ? mhpmevent  [hpm_rix]
                                            : mhpmcounter[hpm_rix];

   // ---- combinational read ----
   always @* begin
      case (raddr)
        MSTATUS:    rdata = mstatus_r;
        SSTATUS:    rdata = mstatus_r & SSTATUS_RMASK;
        MISA:       rdata = MISA_VAL;
        MTVEC:      rdata = mtvec;
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
        STVEC:      rdata = stvec;
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
        MIMPID:     rdata = {32'd0, `SMOLRV64_GIT_COMMIT};
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
   wire [11:0] ip_ie  = eff_mip[11:0] & mie[11:0];
   wire [11:0] pend_m = ip_ie & ~mideleg[11:0];
   wire [11:0] pend_s = ip_ie &  mideleg[11:0];
   wire        take_m = m_glob & (pend_m != 12'd0);
   wire        take_s = ~take_m & s_glob & (pend_s != 12'd0);
   wire [11:0] pend   = take_m ? pend_m : (take_s ? pend_s : 12'd0);
   wire        irq_to_s = take_s;
   assign      irq_v     = take_m | take_s;
   assign      irq_cause = pend[11] ? 4'd11 : pend[3] ? 4'd3 : pend[7] ? 4'd7
                         : pend[9]  ? 4'd9  : pend[1] ? 4'd1 : 4'd5;   // STI(5) last

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
   wire [63:0] trap_epc   = xtrap_v ? xtrap_epc : upd_pc;   // irq_take -> upd_pc (interrupted PC)
   wire [63:0] trap_tval  = xtrap_v ? xtrap_tval : (is_ebreak ? upd_pc : 64'd0);
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

   // redir_valid/redir_is_trap reflect only the active system op (consumed by exec_shard);
   // external injections (xtrap_v) redirect via csr_redir_tgt in backend_top instead.
   assign redir_valid   = sysop_exc | irq_take | do_mret | do_sret | do_sfence | do_fschg | do_dxchg | do_satp;
   assign redir_is_trap = sysop_exc | irq_take;     // op's own exception OR delivered interrupt
   // ---- redirect target (combinational) ----
   // An external injection (xtrap_v: page fault / interrupt) takes priority over a coincident
   // xret so backend_top's csr_redir_tgt is the trap vector. Vectored mode (tvec[0]) sends an
   // interrupt to base + 4*cause; exceptions and direct mode go to base.
   wire [63:0] tvec_base = trap_to_s ? {stvec[63:2], 2'b0} : {mtvec[63:2], 2'b0};
   wire        tvec_vec  = trap_is_intr & (trap_to_s ? stvec[0] : mtvec[0]);
   wire [63:0] trap_tgt  = tvec_vec ? (tvec_base + {trap_cause[5:0], 2'b00}) : tvec_base;
   always @* begin
      if (xtrap_v)              redir_target = trap_tgt;
      else if (do_mret)         redir_target = `VA_UNPACK40(mepc);
      else if (do_sret)         redir_target = `VA_UNPACK40(sepc);
      else if (do_sfence)       redir_target = upd_pc + 64'd4;
      else if (do_fschg)        redir_target = upd_pc + 64'd4;   // CSR op is 4 bytes
      else if (do_dxchg)        redir_target = upd_pc + 64'd4;   // CSR op is 4 bytes
      else if (do_satp)         redir_target = upd_pc + 64'd4;   // refetch under the new satp
      else                      redir_target = trap_tgt;
   end

   localparam MIE_B=3, SIE_B=1, MPIE_B=7, SPIE_B=5, SPP_B=8;  // [12:11]=MPP

   integer i;
   always @(posedge clk) begin
      if (reset) begin
         priv<=M; mstatus<=0; mtvec<=0; mepc<=0; mcause<=0; mtval<=0; mscratch<=0;
         mie<=0; mip<=0; medeleg<=0; mideleg<=0; mcounteren<=0; satp<=0;
         mnstatus<=0;
         stvec<=0; sepc<=0; scause<=0; stval<=0; sscratch<=0; scounteren<=0;
         fcsr<=0; stimecmp<=~64'd0; menvcfg<=64'd0; senvcfg<=64'd0;   // Sstc: stimecmp resets to "no deadline"
         mstateen0<=64'hC000_0000_0000_0000;   // SE0|ENVCFG set: no stateen restrictions
      end else if (trap_v) begin
         // trap (system-op exception OR external page fault); target priv per delegation
         if (trap_to_s) begin
            sepc   <= `VA_PACK40(trap_epc);
            scause <= trap_cause;
            stval  <= `VA_PACK40(trap_tval);
            mstatus[SPIE_B] <= mstatus[SIE_B]; mstatus[SIE_B] <= 1'b0;
            mstatus[SPP_B]  <= priv[0];
            priv <= S;
         end else begin
            mepc   <= `VA_PACK40(trap_epc);
            mcause <= trap_cause;
            mtval  <= `VA_PACK40(trap_tval);
            mstatus[MPIE_B] <= mstatus[MIE_B]; mstatus[MIE_B] <= 1'b0;
            mstatus[12:11]  <= priv;
            priv <= M;
         end
      end else if (upd_valid) begin
         if (upd_is_csr) begin
            case (upd_addr)
              MSTATUS:    mstatus <= (mstatus & ~MSTATUS_WMASK) | (newv & MSTATUS_WMASK);
              SSTATUS:    mstatus <= (mstatus & ~SSTATUS_WMASK) | (newv & SSTATUS_WMASK);
              MTVEC:      mtvec   <= newv;
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
              STVEC:      stvec   <= newv;
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
            priv <= mstatus[12:11];
            mstatus[MIE_B]  <= mstatus[MPIE_B]; mstatus[MPIE_B] <= 1'b1;
            mstatus[12:11]  <= U;
         end else if (is_sret) begin
            priv <= {1'b0, mstatus[SPP_B]};
            mstatus[SIE_B]  <= mstatus[SPIE_B]; mstatus[SPIE_B] <= 1'b1;
            mstatus[SPP_B]  <= 1'b0;
         end
      end
      // FP exception flags accumulate (OoO, off the trap/csr chain). Last write to
      // fcsr[4:0] this cycle, so it ORs on top of a coincident fcsr CSR write. FP ops
      // also dirty mstatus.FS (-> SD); harmless when FP is idle.
      // fcsr exception flags accumulate on flag-producing FP ops only.
      if (!reset && fp_fflags_we) fcsr[4:0] <= fcsr[4:0] | fp_fflags;
      // FS -> Dirty when a retiring op wrote FP state: an f-register (fp_dirty_commit, raised at
      // commit by commit_ctl for FP arith/compare + in-core FSGNJ/FMV.x.X + FP loads) or an FP CSR
      // write (fcsr/fflags/frm, itself commit-serialized). Guarded to FP-enabled (FS != Off).
      if (!reset && (mstatus[14:13] != 2'b00) &&
          (fp_dirty_commit | (upd_valid & upd_is_csr & csr_writes & ~trap_v & ~csr_illegal
                              & ((upd_addr==FCSR) | (upd_addr==FFLAGS) | (upd_addr==FRM)))))
         mstatus[14:13] <= 2'b11;
      // Zicntr counters (off the trap/csr chain so they tick every cycle). mcycle counts
      // clocks; minstret adds the committing checkpoint's instruction count. An M-mode
      // write to mcycle/minstret loads the value (this cycle's increment is dropped).
      if (reset) begin
         mcycle <= 64'd0; minstret <= 64'd0;
      end else begin
         mcycle   <= (upd_valid && upd_is_csr && !trap_v && upd_addr==MCYCLE)
                       ? newv : mcycle   + (mcountinhibit[0] ? 64'd0 : 64'd1);
         minstret <= (upd_valid && upd_is_csr && !trap_v && upd_addr==MINSTRET)
                       ? newv : minstret + (mcountinhibit[2] ? 64'd0 : {58'd0, retire_cnt});
      end
      // Zihpm counters (off the trap/csr chain, like Zicntr). Each mhpmcounterN adds its
      // mhpmeventN-selected event's count this cycle unless inhibited (mcountinhibit[N]); an
      // M-mode write loads the value (that cycle's increment dropped). mhpmeventN + mcountinhibit
      // are plain WARL. Only CYCLES/INSTRET are tapped in Phase 1; other event codes contribute
      // 0 (the counter holds) -- the extension point for branch/cache/TLB events.
      if (reset) begin
         mcountinhibit <= 64'd0;
         for (i=0; i<HPMN; i=i+1) begin mhpmevent[i] <= 64'd0; mhpmcounter[i] <= 64'd0; end
      end else begin
         if (upd_valid && upd_is_csr && !trap_v && !csr_illegal && upd_addr==MCOUNTINHIBIT)
            mcountinhibit <= newv & HPM_INHIBIT_MASK;
         for (i=0; i<HPMN; i=i+1) begin
            if (upd_valid && upd_is_csr && !trap_v && !csr_illegal && upd_addr==(MHPMEVENT3+i))
               mhpmevent[i] <= newv;
            if (upd_valid && upd_is_csr && !trap_v && !csr_illegal && upd_addr==(MHPMCOUNTER3+i))
               mhpmcounter[i] <= newv;
            else if (!mcountinhibit[i+3])
               mhpmcounter[i] <= mhpmcounter[i] + {58'd0, hpm_inc(mhpmevent[i][15:0])};
         end
      end
   end
endmodule

`default_nettype wire
