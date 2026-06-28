`default_nettype none

// Synthesizable SoC top: the sharded-OoO core (backend_top) + unified I$/D$ (cache.v)
// + l2_arbiter merging all memory traffic onto ONE line memory port + a behavioral
// line RAM. This lifts the proven tb_vl cache adapters (sticky-rvalid read port,
// write-through write port, fence.i drain+invalidate FSM) into a real module, and
// replaces the per-cache L2 responders with the arbiter so I$-fill, D$-fill/write and
// the 3 PTW ports share one memory -- the shape the real DRAM bridge plugs into.
//
// Requester order into the arbiter (lower index = higher priority):
//   0 D$ (fill + write-through)   1 I$ (fill)   2 iPTW   3 ldPTW   4 stPTW
//
// SCOPE: RAM only (no MMIO devices yet -- CLINT/UART routing is the next increment;
// all dmem currently routes to the D$). RAM is byte-addressable internally (loadable
// via $readmemh from a TB) with a 64-byte line port for the arbiter.
module soc_top #(
   parameter IW=4, HW=8, PCW=64, SEQW=8, PBITS=8,
   parameter [63:0] BASE     = 64'h8000_0000,   // DDR
   parameter        RAM_LG2  = 21,              // 2 MiB DDR
   parameter [63:0] LBASE    = 64'h7000_0000,   // on-chip local SRAM (boot/monitor) -- MEM_BASEADDR on the FPGA
   parameter        LRAM_LG2 = 18,              // 256 KiB local SRAM
   parameter [63:0] RESET_PC = BASE,            // tests link @DDR; the platform boots @LBASE
   parameter        SIZE_KB  = 128              // each cache
) (
   input  wire             clk,
   input  wire             reset,
   // observation for a TB (commit + the store stream, to watch tohost)
   output wire             commit,
   output wire             dmem_wen,
   output wire [63:0]      dmem_waddr,
   output wire [63:0]      dmem_wdata,
   output wire [7:0]       dmem_wmask,
   // external DDR line port (cache-backed DRAM @ BASE): sim TB / FPGA DDR4 bridge
   output wire             ddr_req,
   output wire             ddr_we,
   output wire [57:0]      ddr_addr,     // line address PA[63:6]
   output wire [511:0]     ddr_wdata,
   input  wire [511:0]     ddr_rdata,
   input  wire             ddr_ack,
   // UART receive: the TB/host pushes a byte (uart_rx_we while uart_rx_ready) -> the core
   // reads it from RBR. uart_rx_ready = the holding register is empty (DR clear).
   input  wire             uart_rx_we,
   input  wire [7:0]       uart_rx_data,
   output wire             uart_rx_ready,
   // UART transmit byte stream (THR writes): on FPGA this feeds rs232tx (valid/ready
   // handshake). A sim TB ties uart_tx_ready=1 to drain instantly; $write still emits.
   output wire             uart_tx_valid,
   output wire [7:0]       uart_tx_data,
   input  wire             uart_tx_ready
);
   localparam SIZE = 1<<RAM_LG2;
   localparam AW   = 64;
   localparam LAW  = AW-6;                  // line address width = 58

   // ---------------- core <-> caches nets ----------------
   wire [PCW-1:0]      imem_addr;
   wire [HW*16-1:0]    imem_data;
   wire [3:0]          imem_avail;
   wire [63:0]         dmem_raddr;
   wire                dmem_ren;
   wire [63:0]         dmem_rdata;
   wire                dmem_rvalid, dmem_wready, dmem_idle, ifence;
   wire [55:0]         ptw_addr, ldptw_addr, stptw_addr;
   wire                ptw_read, ldptw_read, stptw_read;
   wire [63:0]         ptw_rdata, ldptw_rdata, stptw_rdata;
   wire                ptw_rvalid, ldptw_rvalid, stptw_rvalid;
   wire [IW-1:0]       wb_valid;  wire [IW*PBITS-1:0] wb_pr;  wire [IW*64-1:0] wb_val;
   wire                redirect;  wire [PCW-1:0] redirect_target;

   backend_top #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW), .PBITS(PBITS), .RESET_PC(RESET_PC)) core
     (.clk(clk), .reset(reset),
      .imem_addr(imem_addr), .imem_data(imem_data), .imem_avail(imem_avail), .hw_ip(hw_ip), .mtime(clint_mtime),
      .dmem_raddr(dmem_raddr), .dmem_ren(dmem_ren), .dmem_rdata(dmem_rdata), .dmem_rvalid(dmem_rvalid),
      .dmem_wen(dmem_wen), .dmem_waddr(dmem_waddr), .dmem_wdata(dmem_wdata), .dmem_wmask(dmem_wmask),
      .dmem_wready(dmem_wready), .dmem_idle(dmem_idle), .ifence(ifence),
      .ptw_addr(ptw_addr), .ptw_read(ptw_read), .ptw_rdata(ptw_rdata), .ptw_rvalid(ptw_rvalid),
      .ldptw_addr(ldptw_addr), .ldptw_read(ldptw_read), .ldptw_rdata(ldptw_rdata), .ldptw_rvalid(ldptw_rvalid),
      .stptw_addr(stptw_addr), .stptw_read(stptw_read), .stptw_rdata(stptw_rdata), .stptw_rvalid(stptw_rvalid),
      .wb_valid(wb_valid), .wb_pr(wb_pr), .wb_val(wb_val),
      .redirect(redirect), .redirect_target(redirect_target), .commit(commit), .commit_idx());

   // ---------------- MMIO device routing (CLINT + UART bypass the D$, non-cacheable) ----------------
   localparam [63:0] CLINT_BASE = 64'h0200_0000, UART_BASE = 64'h1000_0000, PLIC_BASE = 64'h0C00_0000;
   // DDR latency HPM window (read-only counters; any write clears). NOT in the DTB -- read it
   // from a bare-metal tool / the monitor; the kernel never touches it.
   localparam [63:0] HPM_BASE   = 64'h1800_0000;
   wire is_clint_r = (dmem_raddr & ~64'hffff)     == CLINT_BASE;
   wire is_uart_r  = (dmem_raddr & ~64'hf)        == UART_BASE;
   wire is_plic_r  = (dmem_raddr & ~64'h3ff_ffff) == PLIC_BASE;   // 64 MiB region
   wire is_hpm_r   = (dmem_raddr & ~64'hff)        == HPM_BASE;    // 256 B window
   wire is_dev_r   = is_clint_r | is_uart_r | is_plic_r | is_hpm_r;
   wire is_clint_w = (dmem_waddr & ~64'hffff)     == CLINT_BASE;
   wire is_uart_w  = (dmem_waddr & ~64'hf)        == UART_BASE;
   wire is_plic_w  = (dmem_waddr & ~64'h3ff_ffff) == PLIC_BASE;
   wire is_hpm_w   = (dmem_waddr & ~64'hff)        == HPM_BASE;
   wire is_dev_w   = is_clint_w | is_uart_w | is_plic_w | is_hpm_w;
   // device read returns 1 cycle after the ren pulse (combinational device data, held addr);
   // device write accepts in 1 cycle (~dev_wack masks the held wen so it writes once).
   reg  dev_rvalid, dev_wack;
   always @(posedge clk) if (reset) begin dev_rvalid<=1'b0; dev_wack<=1'b0; end
      else begin dev_rvalid <= dmem_ren & is_dev_r; dev_wack <= dmem_wen & is_dev_w & ~dev_wack; end
   wire [63:0] clint_rdata;  wire clint_mtip, clint_msip;  wire [63:0] clint_mtime;
   clint #(.SCALE_DIV(8)) u_clint
     (.clk(clk), .reset(reset),
      .we(dmem_wen & is_clint_w & ~dev_wack),
      .addr((dmem_wen & is_clint_w) ? dmem_waddr[15:0] : dmem_raddr[15:0]),
      .wdata(dmem_wdata), .wmask(dmem_wmask), .rdata(clint_rdata),
      .mtip(clint_mtip), .msip(clint_msip), .o_mtime(clint_mtime));
   // PLIC (SiFive layout @ 0x0C00_0000): external-interrupt controller. No real sources yet
   // (the UART is output-only and there is no virtio), so src=0 -- but the kernel still
   // probes/initialises the region at boot, which would otherwise fault as unmapped.
   wire [63:0] plic_rdata;  wire plic_meip, plic_seip;
   wire [63:0] plic_addr = (dmem_wen & is_plic_w) ? dmem_waddr : dmem_raddr;
   plic u_plic
     (.clk(clk), .reset(reset),
      .we(dmem_wen & is_plic_w & ~dev_wack), .re(dmem_ren & is_plic_r),
      .addr(plic_addr[23:0]), .wdata(dmem_wdata), .wmask(dmem_wmask), .rdata(plic_rdata),
      .src(64'd0), .meip(plic_meip), .seip(plic_seip));
   wire [11:0] hw_ip = (clint_mtip ? 12'h080 : 12'h0) | (clint_msip ? 12'h008 : 12'h0)
                     | (plic_meip  ? 12'h800 : 12'h0) | (plic_seip  ? 12'h200 : 12'h0);
   // minimal NS16550A UART: THR write (off 0, DLAB=0) -> emit; LSR (off 5) -> THRE|TEMT|DR;
   // RBR read (off 0, DLAB=0) -> the received byte (clears DR). LCR.DLAB(bit7) gates off 0.
   reg [7:0] uart_lcr;  integer ub;
   reg [7:0] uart_rbr;  reg uart_dr;       // RX holding register + data-ready
   reg [7:0] uart_thr;  reg uart_thr_full; // TX holding register + pending flag
   assign    uart_rx_ready = ~uart_dr;
   // TX byte stream: hold the THR byte until rs232tx accepts it (FPGA backpressure).
   // In a sim TB uart_tx_ready is tied 1, so the byte drains next cycle (THRE stays high).
   assign    uart_tx_valid = uart_thr_full;
   assign    uart_tx_data  = uart_thr;
   // RBR read strobe: a load to offset 0 with DLAB clear consumes the byte
   wire      uart_off0  = ((dmem_raddr - UART_BASE) & 64'h7) == 64'd0;
   wire      uart_rbr_rd = dmem_ren & is_uart_r & uart_off0 & ~uart_lcr[7];
   always @(posedge clk) if (reset) begin uart_lcr<=8'd0; uart_dr<=1'b0; uart_rbr<=8'd0;
                                          uart_thr<=8'd0; uart_thr_full<=1'b0; end
      else begin
         if (uart_thr_full & uart_tx_ready) uart_thr_full <= 1'b0;  // serializer took the byte
         if (dmem_wen & is_uart_w & ~dev_wack)
            for (ub=0; ub<8; ub=ub+1) if (dmem_wmask[ub])
               case ((dmem_waddr - UART_BASE + ub) & 3'h7)
                  3'd0: if (!uart_lcr[7]) begin
                           uart_thr <= dmem_wdata[ub*8 +: 8]; uart_thr_full <= 1'b1;
                           $write("%c", dmem_wdata[ub*8 +: 8]);   // sim-only; synth ignores
                        end
                  3'd3: uart_lcr <= dmem_wdata[ub*8 +: 8];
                  default: ;
               endcase
         if (uart_rx_we & ~uart_dr) begin uart_rbr <= uart_rx_data; uart_dr <= 1'b1; end
         else if (uart_rbr_rd)      uart_dr <= 1'b0;
      end
   function [63:0] uart_rd; input [63:0] a; input [7:0] rbr; input dr; input dlab; input thr_full;
      integer b2; reg [2:0] off;
      begin uart_rd=64'd0; for (b2=0;b2<8;b2=b2+1) begin
         off=(a-UART_BASE+b2)&3'h7;
         uart_rd[b2*8 +: 8] = (off==3'd5) ? ((thr_full?8'h00:8'h60) | (dr?8'h01:8'h00)) // LSR: THRE|TEMT|DR
                            : (off==3'd0 && !dlab) ? rbr                     // RBR
                            : 8'h00; end end
   endfunction
   wire [63:0] hpm_rdata;
   wire [63:0] dev_rdata = is_clint_r ? clint_rdata
                         : is_uart_r  ? uart_rd(dmem_raddr, uart_rbr, uart_dr, uart_lcr[7], uart_thr_full)
                         : is_plic_r  ? plic_rdata
                         : is_hpm_r   ? hpm_rdata   : 64'd0;

   // ---------------- D$ (write-through) + read/write adapters (proven in tb_vl), device-muxed ----------------
   reg          c_rd_pend;
   wire [63:0]  dc_rd_data;  wire dc_rd_valid, dc_wr_ack;  wire [63:0] dc_rd_resp_addr;
   wire         dc_l2_req, dc_l2_we;  wire [LAW-1:0] dc_l2_addr;  wire [511:0] dc_l2_wdata;
   wire [511:0] dc_l2_rdata;  wire dc_l2_ack;
   // The LSU is single-outstanding but a SQUASH abandons an in-flight load and issues a new one
   // ("a new mem_ren supersedes any prior unfinished read"). The cache, already committed to the
   // squashed address, would otherwise deliver that stale line to the new load. Match the cache's
   // response address to the current request (like the I$ does with i_pa); a non-matching response
   // is discarded and the request re-issues for the new address.
   wire         dc_rv_ok   = dc_rd_valid & (dc_rd_resp_addr == dmem_raddr);
   wire         raw_rvalid = is_dev_r ? dev_rvalid : dc_rv_ok;
   wire [63:0]  raw_rdata  = is_dev_r ? dev_rdata  : dc_rd_data;
   wire         c_rd_req = (dmem_ren | c_rd_pend) & ~dc_rv_ok & ~is_dev_r;
   always @(posedge clk) if (reset) c_rd_pend<=1'b0;
      else if (dmem_ren) c_rd_pend<=1'b1; else if (raw_rvalid) c_rd_pend<=1'b0;
   reg          c_rdv_st;  reg [63:0] c_rdd_st;
   always @(posedge clk) if (reset) c_rdv_st<=1'b0;
      else if (dmem_ren) c_rdv_st<=1'b0;
      else if (raw_rvalid) begin c_rdv_st<=1'b1; c_rdd_st<=raw_rdata; end
   wire         c_st_ok = c_rdv_st & ~c_rd_pend & ~dmem_ren;
   assign       dmem_rdata  = c_st_ok ? c_rdd_st : raw_rdata;
   assign       dmem_rvalid = raw_rvalid | c_st_ok;
   assign       dmem_wready = is_dev_w ? dev_wack : dc_wr_ack;

   // D$ is WRITE-BACK (WRTHRU=0): stores ack into the line (dirty), evicted lazily -- the
   // store buffer drains in ~1-2c instead of a full L2 round-trip. PTW reads are routed THROUGH
   // the D$ (the dcr_* read-port arbiter below), so a page-table walk always sees dirty PTEs --
   // the D$ is the coherency point. sfence.vma therefore needs NO D$ flush (just a TLB flush);
   // only fence.i still clean-flushes (the I$ reads L2 directly) -- see the df_* FSM below.
   wire        dcr_req;  wire [63:0] dcr_addr;     // muxed D$ read port (LSU + 3 PTW), assigned below
   wire dc_inv_req, dc_inv_busy;
   cache #(.PAW(64), .SIZE_KB(SIZE_KB), .RDW(64), .WDW(64), .WRITABLE(1), .WRTHRU(0), .PERF_ID(1)) u_dcache
     (.clk(clk), .reset(reset),
      .rd_req(dcr_req), .rd_addr(dcr_addr), .rd_data(dc_rd_data), .rd_valid(dc_rd_valid),
      .rd_resp_addr(dc_rd_resp_addr),
      .wr_req(dmem_wen & ~dc_wr_ack & ~is_dev_w), .wr_addr(dmem_waddr), .wr_data(dmem_wdata),
      .wr_mask(dmem_wmask), .wr_ack(dc_wr_ack), .inv_req(dc_inv_req), .inv_clean(1'b1), .inv_busy(dc_inv_busy),
      .l2_req(dc_l2_req), .l2_we(dc_l2_we), .l2_addr(dc_l2_addr), .l2_wdata(dc_l2_wdata),
      .l2_rdata(dc_l2_rdata), .l2_ack(dc_l2_ack));

   // D$ clean-flush on fence.i ONLY: drain the store buffer, then clean-flush the D$ (write back
   // dirty lines, keep them valid). FENCE.I needs this because the I$ reads L2/DDR directly: with
   // a write-back D$, freshly-stored code sits DIRTY in the D$, so the I$ would refetch STALE
   // bytes after a bare invalidate -- the D$ must write back first. The I$-invalidate FSM (fi)
   // below waits for this flush (df==DF_IDLE) before invalidating, so DDR is current before the
   // refetch. sfence.vma does NOT trigger this: PTW reads go through the D$ (coherent), so the
   // walk never sees stale memory -- sfence only flushes the TLB (in the MMU).
   localparam DF_IDLE=0, DF_DRAIN=1, DF_INV=2, DF_WAIT=3;
   reg [1:0] df;  wire df_stall = (df != DF_IDLE);
   reg dc_inv_req_r;  assign dc_inv_req = dc_inv_req_r;
   always @(posedge clk) if (reset) begin df<=DF_IDLE; dc_inv_req_r<=1'b0; end
      else begin
         dc_inv_req_r <= 1'b0;
         case (df)
           DF_IDLE:  if (ifence) df<=DF_DRAIN;
           DF_DRAIN: if (dmem_idle) begin dc_inv_req_r<=1'b1; df<=DF_INV; end
           DF_INV:   df<=DF_WAIT;
           DF_WAIT:  if (!dc_inv_busy) df<=DF_IDLE;
         endcase
      end

   // ---------------- I$ (read-only) + fetch adapter + fence.i FSM (proven in tb_vl) ----------------
   reg          i_have, i_rd_pend;  reg [63:0] i_pa, i_reqpa;  reg [HW*16-1:0] i_win;
   wire         i_match = i_have & (i_pa == imem_addr);
   wire         i_need  = ~i_match;
   wire [HW*16-1:0] ic_rd_data;  wire ic_rd_valid, ic_inv_busy;
   wire         ic_rd_req  = (i_need | i_rd_pend) & ~ic_rd_valid;
   wire [63:0]  ic_rd_addr = i_rd_pend ? i_reqpa : imem_addr;
   wire         ic_l2_req, ic_l2_we;  wire [LAW-1:0] ic_l2_addr;  wire [511:0] ic_l2_wdata;
   wire [511:0] ic_l2_rdata;  wire ic_l2_ack;
   reg          ic_inv_req;
   always @(posedge clk) if (reset) begin i_have<=1'b0; i_rd_pend<=1'b0; end
      else begin
         if (ic_inv_req) i_have<=1'b0;
         if (~i_rd_pend & i_need) begin i_rd_pend<=1'b1; i_reqpa<=imem_addr; end
         if (ic_rd_valid) begin i_rd_pend<=1'b0; i_have<=1'b1; i_pa<=i_reqpa; i_win<=ic_rd_data; end
      end
   localparam FI_IDLE=0, FI_DRAIN=1, FI_INV=2, FI_WAIT=3;
   reg [1:0] fi;  wire fi_stall = (fi != FI_IDLE);
   always @(posedge clk) if (reset) begin fi<=FI_IDLE; ic_inv_req<=1'b0; end
      else begin
         ic_inv_req <= 1'b0;
         case (fi)
           FI_IDLE:  if (ifence) fi<=FI_DRAIN;
           // wait for the D$ clean-flush (df, also triggered by ifence) to finish so DDR holds
           // the freshly-written code BEFORE invalidating the I$ -> the refetch can't be stale.
           FI_DRAIN: if (dmem_idle & (df == DF_IDLE)) begin ic_inv_req<=1'b1; fi<=FI_INV; end
           FI_INV:   fi<=FI_WAIT;
           FI_WAIT:  if (!ic_inv_busy) fi<=FI_IDLE;
         endcase
      end
   assign imem_data  = i_win;
   // Freeze fetch during a fence.i (fi_stall): the I$ must not refetch until the D$ has written
   // back the freshly-stored code and the I$ has been invalidated. fi_stall spans the whole df
   // clean-flush (fi waits for df==DF_IDLE before invalidating), so it covers df_stall too.
   // sfence.vma no longer freezes fetch: the PTW reads through the coherent D$ (no flush).
   assign imem_avail = fi_stall ? 4'd0 : (i_match ? 4'd8 : 4'd0);

   cache #(.PAW(64), .SIZE_KB(SIZE_KB), .RDW(HW*16), .WDW(64), .WRITABLE(0), .PERF_ID(0)) u_icache
     (.clk(clk), .reset(reset),
      .rd_req(ic_rd_req), .rd_addr(ic_rd_addr), .rd_data(ic_rd_data), .rd_valid(ic_rd_valid),
      .wr_req(1'b0), .wr_addr(64'd0), .wr_data(64'd0), .wr_mask(8'd0), .wr_ack(),
      .inv_req(ic_inv_req), .inv_clean(1'b0), .inv_busy(ic_inv_busy),
      .l2_req(ic_l2_req), .l2_we(ic_l2_we), .l2_addr(ic_l2_addr), .l2_wdata(ic_l2_wdata),
      .l2_rdata(ic_l2_rdata), .l2_ack(ic_l2_ack));

   // ---------------- PTW adapters: PTE word reads routed THROUGH the D$ ----------------
   // g=0 iPTW, 1 ldPTW, 2 stPTW. Each *_read is held (with a stable *_addr) until *_rvalid.
   // A walk reads its 8-byte PTE through the D$ read port (shared with the LSU via the dcr_*
   // arbiter below), so it always observes dirty PTEs -- no sfence.vma flush needed. pw_busy[g]
   // marks an outstanding read; the response is matched by exact byte address (the cache echoes
   // rd_resp_addr = the requested address). Same-address responses safely share data, so no
   // owner tag is needed.
   wire [2:0]   pw_read   = {stptw_read, ldptw_read, ptw_read};
   wire [3*56-1:0] pw_addr = {stptw_addr, ldptw_addr, ptw_addr};
   reg  [2:0]   pw_busy;
   reg  [2:0]   pw_rvalid;
   reg  [63:0]  pw_rdata [0:2];
   wire [2:0]   pw_match;
   wire [511:0] arb_rdata;
   genvar g;
   generate for (g=0; g<3; g=g+1) begin : ptw_adapt
      assign pw_match[g] = dc_rd_valid & pw_busy[g]
                         & (dc_rd_resp_addr == {8'd0, pw_addr[g*56 +: 56]});
      always @(posedge clk) if (reset) begin pw_busy[g]<=1'b0; pw_rvalid[g]<=1'b0; end
         else begin
            pw_rvalid[g] <= 1'b0;
            if (pw_read[g] & ~pw_busy[g] & ~pw_rvalid[g]) pw_busy[g] <= 1'b1;  // new request
            if (pw_match[g]) begin
               pw_busy[g]  <= 1'b0;
               pw_rvalid[g]<= 1'b1;
               pw_rdata[g] <= dc_rd_data;                  // the cache returns the 64-bit word
            end
         end
   end endgenerate
   assign ptw_rdata=pw_rdata[0];   assign ptw_rvalid=pw_rvalid[0];
   assign ldptw_rdata=pw_rdata[1]; assign ldptw_rvalid=pw_rvalid[1];
   assign stptw_rdata=pw_rdata[2]; assign stptw_rvalid=pw_rvalid[2];

   // D$ read-port arbiter: the LSU (c_rd_req/dmem_raddr) and the 3 PTW walks share the single D$
   // read port. Fixed priority LSU > iPTW > ldPTW > stPTW. The cache samples rd_req only at its
   // S_IDLE and self-serializes; each client holds its request until its response matches, so a
   // purely combinational mux suffices (no accept handshake). No deadlock: a load needing ldPTW
   // is itself blocked on translation and not issuing c_rd_req, so the walk gets the port.
   assign dcr_req  = c_rd_req | (|pw_busy);
   assign dcr_addr = c_rd_req    ? dmem_raddr
                   : pw_busy[0]  ? {8'd0, pw_addr[0*56 +: 56]}
                   : pw_busy[1]  ? {8'd0, pw_addr[1*56 +: 56]}
                   :               {8'd0, pw_addr[2*56 +: 56]};

   // ---------------- l2_arbiter (2 requesters: D$, I$) ----------------
   // PTW reads no longer reach the arbiter -- they go through the D$ (dcr_* above), and a D$ miss
   // on a PTE fills via dc_l2_* here like any other line. So the arbiter serves only the two
   // caches' line refills/writebacks.
   localparam NREQ=2;
   wire [NREQ-1:0]     a_req   = {ic_l2_req, dc_l2_req};
   wire [NREQ-1:0]     a_we    = {1'b0, dc_l2_we};
   wire [NREQ*LAW-1:0] a_addr  = {ic_l2_addr, dc_l2_addr};
   wire [NREQ*512-1:0] a_wdata = {ic_l2_wdata, dc_l2_wdata};
   wire [NREQ-1:0]     a_ack;
   wire                m_req, m_we;  wire [LAW-1:0] m_addr;  wire [511:0] m_wdata, m_rdata;  wire m_ack;
   l2_arbiter #(.NREQ(NREQ), .AW(LAW), .DW(512)) u_arb
     (.clk(clk), .reset(reset),
      .req(a_req), .we(a_we), .addr(a_addr), .wdata(a_wdata), .ack(a_ack), .rdata(arb_rdata),
      .mem_req(m_req), .mem_we(m_we), .mem_addr(m_addr), .mem_wdata(m_wdata),
      .mem_rdata(m_rdata), .mem_ack(m_ack));
   assign dc_l2_ack = a_ack[0];  assign dc_l2_rdata = arb_rdata;
   assign ic_l2_ack = a_ack[1];  assign ic_l2_rdata = arb_rdata;

   // ---------------- memory: internal local SRAM (BRAM) + EXTERNAL DDR port ----------------
   // The arbiter's single line transaction is region-decoded: the on-chip local SRAM at
   // LBASE (boot/monitor; FPGA = BRAM init'd from mem.even/odd) is served internally; the
   // DDR region at BASE is forwarded to soc_top's external line port (ddr_*) -- the sim TB's
   // behavioral DRAM, or the real DDR4/MIG bridge on the FPGA. The cache is PIPT so it fills/
   // writes either region transparently. (lram is $readmemh-loadable via dut.lram for boot.)
   localparam LSIZE = 1<<LRAM_LG2;
   wire [63:0] m_pa       = {{6{1'b0}}, m_addr} << 6;       // physical byte addr of the line
   wire        m_is_local = (m_pa >= LBASE) && (m_pa < LBASE + LSIZE);

   // local SRAM responder (on-chip BRAM). Stored as 512-bit LINES (the L2 port is
   // line-granular) so it infers a clean single-read/single-write BRAM -- a byte array
   // with a 64-byte for-loop access does NOT (Vivado can't template it).
   localparam NLLINE = LSIZE/64;                 // number of 64-byte lines
   localparam [LAW-1:0] LLBASE = LBASE >> 6;     // local SRAM base as a line address
   reg [511:0] lmem [0:NLLINE-1];
   // FPGA: bake the monitor image into the BRAM at elaboration (one 64-byte line per hex
   // line). Sim TBs instead pack dut.lmem directly via +monhex, so guard on the define.
`ifdef SOC_BOOT_HEX
   initial $readmemh(`SOC_BOOT_HEX, lmem);
`endif
   reg l_busy; reg [3:0] l_cnt; reg l_we_q; reg [LAW-1:0] l_li_q; reg [511:0] l_wd_q;
   reg [511:0] l_rdata; reg l_ack;
   wire [LAW-1:0] l_line = m_addr - LLBASE;       // local line index
   wire l_req = m_req & m_is_local;
   always @(posedge clk) begin
      l_ack <= 1'b0;
      if (reset) l_busy<=1'b0;
      else if (!l_busy && l_req) begin l_busy<=1'b1; l_cnt<=4'd1; l_we_q<=m_we; l_li_q<=l_line; l_wd_q<=m_wdata; end
      else if (l_busy) begin
         if (l_cnt==0) begin
            if (l_we_q) lmem[l_li_q] <= l_wd_q;
            else        l_rdata     <= lmem[l_li_q];
            l_ack<=1'b1; l_busy<=1'b0;
         end else l_cnt <= l_cnt-1;
      end
   end

   // external DDR line port (sim TB drives it; FPGA = DDR4 bridge)
   assign ddr_req   = m_req & ~m_is_local;
   assign ddr_we    = m_we;
   assign ddr_addr  = m_addr;
   assign ddr_wdata = m_wdata;

   // DDR latency HPM: time ddr_req->ddr_ack (core cycles) into read/write log2 histograms,
   // read-only at HPM_BASE (any write clears). Observation-only; off the core critical path.
   ddr_hpm u_ddr_hpm
     (.clk(clk), .reset(reset),
      .ddr_req(ddr_req), .ddr_we(ddr_we), .ddr_ack(ddr_ack),
      .raddr(dmem_raddr[7:0]), .rdata(hpm_rdata),
      .clr(dmem_wen & is_hpm_w & ~dev_wack));

   // response mux back to the arbiter (m_addr held by the arbiter through the transaction)
   assign m_ack   = m_is_local ? l_ack   : ddr_ack;
   assign m_rdata = m_is_local ? l_rdata : ddr_rdata;
endmodule

`default_nettype wire
