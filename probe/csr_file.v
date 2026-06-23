`default_nettype none

// Architectural CSR state for the sharded-OoO backend. Read is combinational; a
// single update per cycle (one system op executes at a time -- the scheduler gates
// a serializing op until its checkpoint is oldest, so this update is precise).
//
// Scope: M-mode only, privilege NOT modelled (the machine is always "in M-mode").
// ECALL therefore reports cause 11 unconditionally -- the riscv-test trap handler
// routes causes 8/9/11 identically, so this passes rv64ui/rv64um. xRET restores
// MIE<-MPIE but does not actually change privilege. Unknown CSRs read 0 / ignore
// writes (permissive WARL); the named set below covers the riscv-test reset/trap
// sequence (mtvec/mepc/mcause/mstatus + the pmp/satp/mie WARL stubs it writes).
module csr_file
   (input  wire        clk,
    input  wire        reset,
    // combinational read (for a CSR op's rd = old value)
    input  wire [11:0] raddr,
    output reg  [63:0] rdata,
    // redirect-target sources (current values, read combinationally by the shard)
    output wire [63:0] mtvec_o,
    output wire [63:0] mepc_o,
    // single update (driven at EX by the oldest system op -> non-speculative)
    input  wire        upd_valid,
    input  wire        upd_is_csr,
    input  wire [2:0]  upd_func,     // CSR funct3 (csrrw/s/c + imm forms)
    input  wire [11:0] upd_addr,     // CSR addr (is_csr) OR system-op selector (imm[11:0])
    input  wire [63:0] upd_src,      // rs1 value or zimm
    input  wire [63:0] upd_pc);      // the system op's PC (for mepc)

   localparam [11:0] MSTATUS=12'h300, MISA=12'h301, MEDELEG=12'h302, MIDELEG=12'h303,
                     MIE=12'h304, MTVEC=12'h305, MCOUNTEREN=12'h306,
                     MSCRATCH=12'h340, MEPC=12'h341, MCAUSE=12'h342, MTVAL=12'h343,
                     MIP=12'h344, SATP=12'h180, MHARTID=12'hF14,
                     PMPCFG0=12'h3A0, PMPADDR0=12'h3B0, MNSTATUS=12'h744;

   // system-op selectors (imm[11:0] of a funct3==0 SYSTEM op)
   localparam [11:0] OP_ECALL=12'h000, OP_EBREAK=12'h001, OP_SRET=12'h102,
                     OP_MRET=12'h302, OP_WFI=12'h105;

   reg [63:0] mstatus, mtvec, mepc, mcause, mtval, mscratch, mie, mip,
              medeleg, mideleg, satp, mcounteren, pmpcfg0, pmpaddr0, mnstatus;

   assign mtvec_o = mtvec;
   assign mepc_o  = mepc;

   // ---- combinational read ----
   always @* begin
      case (raddr)
        MSTATUS:    rdata = mstatus;
        MTVEC:      rdata = mtvec;
        MEPC:       rdata = mepc;
        MCAUSE:     rdata = mcause;
        MTVAL:      rdata = mtval;
        MSCRATCH:   rdata = mscratch;
        MIE:        rdata = mie;
        MIP:        rdata = mip;
        MEDELEG:    rdata = medeleg;
        MIDELEG:    rdata = mideleg;
        SATP:       rdata = satp;
        MCOUNTEREN: rdata = mcounteren;
        PMPCFG0:    rdata = pmpcfg0;
        PMPADDR0:   rdata = pmpaddr0;
        MNSTATUS:   rdata = mnstatus;
        MHARTID:    rdata = 64'd0;
        default:    rdata = 64'd0;
      endcase
   end

   // ---- CSR rmw new-value from funct3 ----
   //  func[1:0]: 01=rw 10=rs 11=rc ; func[2]=immediate-form (src already = zimm)
   reg [63:0] newv;
   always @* begin
      case (upd_func[1:0])
        2'b01:   newv = upd_src;             // csrrw/wi
        2'b10:   newv = rdata | upd_src;     // csrrs/si  (rdata = old at upd_addr)
        2'b11:   newv = rdata & ~upd_src;    // csrrc/ci
        default: newv = rdata;
      endcase
   end

   // mstatus field positions
   localparam MIE_B=3, MPIE_B=7;             // [12:11]=MPP

   integer init_done;
   always @(posedge clk) begin
      if (reset) begin
         mstatus<=0; mtvec<=0; mepc<=0; mcause<=0; mtval<=0; mscratch<=0;
         mie<=0; mip<=0; medeleg<=0; mideleg<=0; satp<=0; mcounteren<=0;
         pmpcfg0<=0; pmpaddr0<=0; mnstatus<=0;
      end else if (upd_valid) begin
         if (upd_is_csr) begin
            // write at the (combinationally-read) CSR address
            case (upd_addr)
              MSTATUS:    mstatus    <= newv;
              MTVEC:      mtvec      <= newv;
              MEPC:       mepc       <= newv;
              MCAUSE:     mcause     <= newv;
              MTVAL:      mtval      <= newv;
              MSCRATCH:   mscratch   <= newv;
              MIE:        mie        <= newv;
              MIP:        mip        <= newv;
              MEDELEG:    medeleg    <= newv;
              MIDELEG:    mideleg    <= newv;
              SATP:       satp       <= newv;
              MCOUNTEREN: mcounteren <= newv;
              PMPCFG0:    pmpcfg0    <= newv;
              PMPADDR0:   pmpaddr0   <= newv;
              MNSTATUS:   mnstatus   <= newv;
              default:    ;                  // unknown CSR: ignore the write
            endcase
         end else begin
            // system op (funct3==0): trap or return
            case (upd_addr)
              OP_ECALL: begin
                 mepc   <= upd_pc;
                 mcause <= 64'd11;            // ecall-from-M (handler accepts 8/9/11)
                 mstatus[MPIE_B] <= mstatus[MIE_B]; mstatus[MIE_B] <= 1'b0;
                 mstatus[12:11]  <= 2'b11;
              end
              OP_EBREAK: begin
                 mepc   <= upd_pc;
                 mcause <= 64'd3;             // breakpoint
                 mstatus[MPIE_B] <= mstatus[MIE_B]; mstatus[MIE_B] <= 1'b0;
                 mstatus[12:11]  <= 2'b11;
              end
              OP_MRET: begin
                 mstatus[MIE_B]  <= mstatus[MPIE_B]; mstatus[MPIE_B] <= 1'b1;
                 mstatus[12:11]  <= 2'b00;
              end
              default: ;                      // WFI/SRET/SFENCE: redirect-only (handled by shard)
            endcase
         end
      end
   end
endmodule

`default_nettype wire
