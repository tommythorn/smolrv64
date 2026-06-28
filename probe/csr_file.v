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
    input  wire [2:0]  retire_cnt,  // # instructions retiring this cycle (commit_ctl) -> minstret
    // ---- pending interrupt (combinational): backend fires it via xtrap_* when it can ----
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
                     // Zicntr: M-mode counters + their U/S read-only shadows. time is the
                     // hardware-backed CLINT mtime (like SmolRV64); cycle/instret shadow
                     // the free-running mcycle / retired-instruction minstret.
                     MCYCLE=12'hB00, MTIME=12'hB01, MINSTRET=12'hB02,
                     CYCLE=12'hC00, TIME=12'hC01, INSTRET=12'hC02;

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
   reg [63:0] mstatus, mtvec, mepc, mcause, mtval, mscratch, mie, mip,
              medeleg, mideleg, mcounteren, satp, mnstatus,
              stimecmp, menvcfg,   // Sstc: supervisor timer-compare + menvcfg.STCE enable
              senvcfg;             // S-mode envcfg (FIOM + Zicbom/Zicboz U-mode CBO enables)
   reg [63:0] stvec, sepc, scause, stval, sscratch, scounteren;
   reg [63:0] mcycle, minstret;      // Zicntr: free-running cycles + retired instructions
   reg [7:0]  fcsr;                  // [7:5]=frm  [4:0]=fflags (NV DZ OF UF NX)
   assign o_frm    = fcsr[7:5];
   assign o_fs_off = (mstatus[14:13] == 2'b00);

   // mstatus as seen on a read: force SXL=UXL=2, and derive SD (bit 63) = any of
   // FS/XS/VS == Dirty (read-only summary; not a stored bit). The riscv-tests v-handler
   // saves/compares sstatus and expects SD set once it dirties FS.
   wire status_sd = (mstatus[14:13] == 2'b11)   // FS  dirty
                  | (mstatus[16:15] == 2'b11)   // XS  dirty
                  | (mstatus[10:9]  == 2'b11);  // VS  dirty
   wire [63:0] mstatus_r = {status_sd, mstatus[62:36], 4'b1010, mstatus[31:0]};

   // effective mip = software-held bits OR the hardware-driven device lines (CLINT/PLIC).
   // hw_ip is 0 when no device is wired (current TB) -> eff_mip == mip, no behavior change.
   // Sstc: when menvcfg.STCE, sip/mip.STIP(5) is driven by the stimecmp deadline
   // (read-only to software); otherwise it is the software-/device-written bit.
   wire        stip_sstc = menvcfg[63] & (mtime >= stimecmp);
   wire [63:0] base_mip  = mip | {52'd0, hw_ip};
   wire [63:0] eff_mip   = menvcfg[63] ? {base_mip[63:6], stip_sstc, base_mip[4:0]} : base_mip;

   // ---- combinational read ----
   always @* begin
      case (raddr)
        MSTATUS:    rdata = mstatus_r;
        SSTATUS:    rdata = mstatus_r & SSTATUS_RMASK;
        MISA:       rdata = MISA_VAL;
        MTVEC:      rdata = mtvec;
        MEPC:       rdata = mepc;
        MCAUSE:     rdata = mcause;
        MTVAL:      rdata = mtval;
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
        SEPC:       rdata = sepc;
        SCAUSE:     rdata = scause;
        STVAL:      rdata = stval;
        SSCRATCH:   rdata = sscratch;
        SATP:       rdata = satp;
        MNSTATUS:   rdata = mnstatus;
        STIMECMP:   rdata = stimecmp;
        MENVCFG:    rdata = menvcfg;
        SENVCFG:    rdata = {56'd0, senvcfg[7:0]};
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
        default:    rdata = 64'd0;   // PMP / unimplemented optional CSRs (read 0)
      endcase
   end

   // ---- CSR rmw new-value from funct3 ----
   reg [63:0] newv;
   always @* begin
      case (upd_func[1:0])
        2'b01:   newv = upd_src;             // csrrw/wi
        2'b10:   newv = rdata | upd_src;     // csrrs/si
        2'b11:   newv = rdata & ~upd_src;    // csrrc/ci
        default: newv = rdata;
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
   wire csr_stateen = (upd_addr[11:2]==10'h0C3) | (upd_addr[11:2]==10'h043);  // m/sstateen0-3
   wire csr_unimpl  = upd_is_csr & ((upd_addr == MTOPI) | csr_stateen);
   // Sstc: stimecmp access in S-mode requires menvcfg.STCE (else illegal). M-mode always
   // allowed; U-mode already blocked by csr_nopriv. (Matches simmerv cpu.rs:1384.)
   wire stce_ill    = upd_is_csr & (upd_addr == STIMECMP) & (priv == S) & ~menvcfg[63];
   assign csr_illegal = upd_valid & (csr_ro | csr_nopriv | satp_tvm | csr_unimpl | stce_ill);

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
   wire [1:0]  newfs_m = (((mstatus & ~MSTATUS_WMASK) | (newv & MSTATUS_WMASK)) >> 13);
   wire [1:0]  newfs_s = (((mstatus & ~SSTATUS_WMASK) | (newv & SSTATUS_WMASK)) >> 13);
   wire        do_fschg = upd_valid & upd_is_csr & ~csr_illegal &
                          ( ((upd_addr==MSTATUS) & (newfs_m != mstatus[14:13]))
                          | ((upd_addr==SSTATUS) & (newfs_s != mstatus[14:13])) );

   // redir_valid/redir_is_trap reflect only the active system op (consumed by exec_shard);
   // external injections (xtrap_v) redirect via csr_redir_tgt in backend_top instead.
   assign redir_valid   = sysop_exc | irq_take | do_mret | do_sret | do_sfence | do_fschg;
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
      else if (do_mret)         redir_target = mepc;
      else if (do_sret)         redir_target = sepc;
      else if (do_sfence)       redir_target = upd_pc + 64'd4;
      else if (do_fschg)        redir_target = upd_pc + 64'd4;   // CSR op is 4 bytes
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
      end else if (trap_v) begin
         // trap (system-op exception OR external page fault); target priv per delegation
         if (trap_to_s) begin
            sepc   <= trap_epc;
            scause <= trap_cause;
            stval  <= trap_tval;
            mstatus[SPIE_B] <= mstatus[SIE_B]; mstatus[SIE_B] <= 1'b0;
            mstatus[SPP_B]  <= priv[0];
            priv <= S;
         end else begin
            mepc   <= trap_epc;
            mcause <= trap_cause;
            mtval  <= trap_tval;
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
              MEPC:       mepc    <= newv;
              MCAUSE:     mcause  <= newv;
              MTVAL:      mtval   <= newv;
              MSCRATCH:   mscratch<= newv;
              MIE:        mie     <= newv;
              MIP:        mip     <= newv & ~HW_RO_MASK;  // MEIP/MTIP/MSIP read-only (device-owned)
              SIE:        mie     <= (mie & ~S_INT_MASK) | (newv & S_INT_MASK);
              SIP:        mip     <= (mip & ~S_INT_MASK) | (newv & S_INT_MASK);
              MEDELEG:    medeleg <= newv;
              MIDELEG:    mideleg <= newv;
              MCOUNTEREN: mcounteren <= newv;
              SCOUNTEREN: scounteren <= newv;
              STVEC:      stvec   <= newv;
              SEPC:       sepc    <= newv;
              SCAUSE:     scause  <= newv;
              STVAL:      stval   <= newv;
              SSCRATCH:   sscratch<= newv;
              // satp.ASID is 10-bit WARL (matches SmolRV64 TLB_ASID_BITS=10 + simmerv):
              // zero the unimplemented high ASID bits [59:54] so a read-back matches the
              // reference (Linux probes ASID width by writing all-ones and reading back).
              SATP:       satp    <= newv & ~64'h0FC0_0000_0000_0000;
              MNSTATUS:   mnstatus<= newv;
              STIMECMP:   stimecmp<= newv;       // Sstc (stored verbatim, like simmerv)
              MENVCFG:    menvcfg <= newv;
              SENVCFG:    senvcfg <= newv & 64'h00000000000000f1;  // FIOM + CBZE/CBCFE/CBIE (SmolRV64 mask)
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
      if (!reset && fp_fflags_we) begin
         fcsr[4:0]      <= fcsr[4:0] | fp_fflags;
         mstatus[14:13] <= 2'b11;
      end
      // Zicntr counters (off the trap/csr chain so they tick every cycle). mcycle counts
      // clocks; minstret adds the committing checkpoint's instruction count. An M-mode
      // write to mcycle/minstret loads the value (this cycle's increment is dropped).
      if (reset) begin
         mcycle <= 64'd0; minstret <= 64'd0;
      end else begin
         mcycle   <= (upd_valid && upd_is_csr && !trap_v && upd_addr==MCYCLE)
                       ? newv : mcycle + 64'd1;
         minstret <= (upd_valid && upd_is_csr && !trap_v && upd_addr==MINSTRET)
                       ? newv : minstret + {61'd0, retire_cnt};
      end
   end
endmodule

`default_nettype wire
