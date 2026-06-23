`default_nettype none

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
    output wire        csr_illegal,   // active CSR op is an illegal access (suppress rd)
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
                     MIP=12'h344, MHARTID=12'hF14,
                     PMPCFG0=12'h3A0, PMPADDR0=12'h3B0, MNSTATUS=12'h744,
                     SSTATUS=12'h100, SIE=12'h104, STVEC=12'h105, SCOUNTEREN=12'h106,
                     SSCRATCH=12'h140, SEPC=12'h141, SCAUSE=12'h142, STVAL=12'h143,
                     SIP=12'h144, SATP=12'h180,
                     MVENDORID=12'hF11, MARCHID=12'hF12, MIMPID=12'hF13;

   // system-op selectors (imm[11:0] of a funct3==0 SYSTEM op)
   localparam [11:0] OP_ECALL=12'h000, OP_EBREAK=12'h001, OP_SRET=12'h102,
                     OP_MRET=12'h302, OP_WFI=12'h105;

   // RV64 misa = MXL(2)<<62 | I C M A S U
   localparam [63:0] MISA_VAL = (64'd2<<62) | (64'd1<<0)  /*A*/ | (64'd1<<2)  /*C*/
                              | (64'd1<<8) /*I*/ | (64'd1<<12) /*M*/
                              | (64'd1<<18) /*S*/ | (64'd1<<20) /*U*/;

   // mstatus writable bits (M-mode write); SXL/UXL (35:32) are hardwired to 2.
   localparam [63:0] MSTATUS_WMASK = 64'h0000_0000_007E_79AA;
   // sstatus view: SIE,SPIE,SPP,FS,VS,XS,SUM,MXR,UXL + SD
   localparam [63:0] SSTATUS_RMASK = 64'h8000_0003_000D_E133;
   localparam [63:0] SSTATUS_WMASK = 64'h0000_0000_000C_6122;
   // interrupt-enable/pending S-visible bits (SSIE/STIE/SEIE = 1,5,9)
   localparam [63:0] S_INT_MASK = 64'h0000_0000_0000_0222;

   reg [1:0]  priv;
   reg [63:0] mstatus, mtvec, mepc, mcause, mtval, mscratch, mie, mip,
              medeleg, mideleg, mcounteren, satp, pmpcfg0, pmpaddr0, mnstatus;
   reg [63:0] stvec, sepc, scause, stval, sscratch, scounteren;

   // mstatus as seen on a read: force SXL=UXL=2
   wire [63:0] mstatus_r = {mstatus[63:36], 4'b1010, mstatus[31:0]};

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
        MIP:        rdata = mip;
        SIE:        rdata = mie & S_INT_MASK;
        SIP:        rdata = mip & S_INT_MASK;
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
        PMPCFG0:    rdata = pmpcfg0;
        PMPADDR0:   rdata = pmpaddr0;
        MNSTATUS:   rdata = mnstatus;
        default:    rdata = 64'd0;   // mhartid/mvendorid/marchid/mimpid/unknown
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

   // illegal CSR access: writing a read-only CSR (addr[11:10]==11 & the op writes), or
   // accessing a CSR that needs higher privilege than current (addr[9:8] > priv).
   wire csr_writes = upd_is_csr & ((upd_func[1:0]==2'b01) | (upd_src != 64'd0));
   wire csr_ro     = (upd_addr[11:10]==2'b11) & csr_writes;
   wire csr_nopriv = upd_is_csr & (priv < upd_addr[9:8]);
   assign csr_illegal = upd_valid & (csr_ro | csr_nopriv);

   // exception this op raises (ecall/ebreak/illegal-CSR) + cause + delegation
   wire        exc_active = is_ecall | is_ebreak | csr_illegal;
   wire [63:0] ecall_cause = (priv==M) ? 64'd11 : (priv==S) ? 64'd9 : 64'd8;
   wire [63:0] exc_cause = csr_illegal ? 64'd2 : is_ebreak ? 64'd3 : ecall_cause;
   wire        exc_to_s  = exc_active & (priv != M) & medeleg[exc_cause[5:0]];

   assign redir_valid = upd_valid & (exc_active | is_mret | is_sret);
   // ---- redirect target (combinational) ----
   always @* begin
      if (is_mret)              redir_target = mepc;
      else if (is_sret)         redir_target = sepc;
      else if (exc_to_s)        redir_target = {stvec[63:2], 2'b0};
      else                      redir_target = {mtvec[63:2], 2'b0};
   end

   localparam MIE_B=3, SIE_B=1, MPIE_B=7, SPIE_B=5, SPP_B=8;  // [12:11]=MPP

   integer i;
   always @(posedge clk) begin
      if (reset) begin
         priv<=M; mstatus<=0; mtvec<=0; mepc<=0; mcause<=0; mtval<=0; mscratch<=0;
         mie<=0; mip<=0; medeleg<=0; mideleg<=0; mcounteren<=0; satp<=0;
         pmpcfg0<=0; pmpaddr0<=0; mnstatus<=0;
         stvec<=0; sepc<=0; scause<=0; stval<=0; sscratch<=0; scounteren<=0;
      end else if (upd_valid) begin
         if (exc_active) begin
            // trap (ecall/ebreak/illegal-CSR); target priv per delegation
            if (exc_to_s) begin
               sepc   <= upd_pc;
               scause <= exc_cause;
               stval  <= is_ebreak ? upd_pc : 64'd0;
               mstatus[SPIE_B] <= mstatus[SIE_B]; mstatus[SIE_B] <= 1'b0;
               mstatus[SPP_B]  <= priv[0];
               priv <= S;
            end else begin
               mepc   <= upd_pc;
               mcause <= exc_cause;
               mtval  <= is_ebreak ? upd_pc : 64'd0;
               mstatus[MPIE_B] <= mstatus[MIE_B]; mstatus[MIE_B] <= 1'b0;
               mstatus[12:11]  <= priv;
               priv <= M;
            end
         end else if (upd_is_csr) begin
            case (upd_addr)
              MSTATUS:    mstatus <= (mstatus & ~MSTATUS_WMASK) | (newv & MSTATUS_WMASK);
              SSTATUS:    mstatus <= (mstatus & ~SSTATUS_WMASK) | (newv & SSTATUS_WMASK);
              MTVEC:      mtvec   <= newv;
              MEPC:       mepc    <= newv;
              MCAUSE:     mcause  <= newv;
              MTVAL:      mtval   <= newv;
              MSCRATCH:   mscratch<= newv;
              MIE:        mie     <= newv;
              MIP:        mip     <= newv;
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
              SATP:       satp    <= newv;
              PMPCFG0:    pmpcfg0 <= newv;
              PMPADDR0:   pmpaddr0<= newv;
              MNSTATUS:   mnstatus<= newv;
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
   end
endmodule

`default_nettype wire
