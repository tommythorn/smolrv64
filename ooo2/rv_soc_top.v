`default_nettype none
`ifndef PROBE_CLK_DIV8
 `define PROBE_CLK_DIV8 48        // 166.67 MHz -- THE shipping clock, not a suggestion
`endif
`define PROBE_CLK_HZ ((1_000_000_000 / `PROBE_CLK_DIV8) * 8)

// The I$ fetch window is OOO2_HW halfwords (shipping HW=8 = 16 bytes, the even/odd chunk
// pair) and there is no per-shard writeback bus to size.
//
// Build-id block (0x1000_F000). Guarded so the FPGA build's
// -verilog_define git-commit/stamp/dirty reach it (build.tcl); sim/cosim default to 0.
`ifndef SMOLRV64_GIT_COMMIT
 `define SMOLRV64_GIT_COMMIT 32'h0
`endif
`ifndef SMOLRV64_BUILD_STAMP
 `define SMOLRV64_BUILD_STAMP 64'h0
`endif
`ifndef SMOLRV64_GIT_DIRTY
 `define SMOLRV64_GIT_DIRTY 1'b0
`endif

// The SoC top: ooo2_core + its I$/D$ (rv_cache) + rv_l2_arbiter merging all memory
// traffic onto ONE line memory port + the local boot SRAM. MMIO routing, CLINT/PLIC/UART,
// the virtio bridge and the PTW-through-cache adapters live here. (Forked from the retired
// sharded core's soc_top.v in 2026-08; two page-table walkers, iTLB and dTLB, since the
// LSU translates one op at a time; `retire` is one pulse per retiring instruction.)
//
// The cache adapters here are the ones the retired harness proved (sticky-rvalid read port,
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
`ifndef OOO2_HW
 `define OOO2_HW 8                 // fetch window halfwords (must match ooo2_core.v)
`endif
`ifndef OOO2_IW
 `define OOO2_IW 2                 // pipeline width (must match ooo2_core.v)
`endif
module rv_soc_top #(
   parameter HW=`OOO2_HW, IW=`OOO2_IW, PCW=64, SEQW=8,   // fetch window halfwords / pipeline width
   parameter [63:0] BASE     = 64'h8000_0000,   // DDR
   parameter        RAM_LG2  = 21,              // 2 MiB DDR
   parameter [63:0] LBASE    = 64'h7000_0000,   // on-chip local SRAM (boot/monitor) -- MEM_BASEADDR on the FPGA
   parameter        LRAM_LG2 = 18,              // 256 KiB local SRAM
   parameter [63:0] RESET_PC = BASE,            // tests link @DDR; the platform boots @LBASE
   parameter        SIZE_KB  = 64               // each cache. 128 KB spread the D$ over
                                                 // X 6..72 Y 29..193 -- most of the die -- and
                                                 // its worst INTERNAL route (vw0 -> bank WEA)
                                                 // was 5.7 ns, 82% of it wire.  D$ miss rate
                                                 // on GB5 is 0.195%, so halving costs little.
) (
   input  wire             clk,
   input  wire             reset,
   // observation for a TB (retire + the store stream, to watch tohost)
       output wire             retire,
    output wire             retire2,            // a second retire in the same cycle (item 10c)
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
   input  wire             uart_tx_ready,
   // virtio-mmio passthrough: the sim-only block device (virtio_mmio + virtio_blk) lives
   // in the TB wrapper. The CPU's 32-bit virtio register access is presented as a
   // byte-offset addr[11:0] + 32b write data/byte-enable positioned by addr[2]; read data
   // comes back 32b. virtio_irq raises PLIC source 1. The FPGA build leaves these
   // unconnected (virtio_rdata/irq read 0) -- the region is never touched without a DTB node.
   output wire             fbdiag_reset_req,  // one-shot: fetch-buffer invariant fired, reset to the monitor
   output wire [12:0]      virtio_addr,   // 13-bit: bit12 selects blk(0)/net(0x1000) within the 8 KiB region
   output wire             virtio_read,
   output wire             virtio_write,
   output wire [31:0]      virtio_wdata,
   output wire [3:0]       virtio_be,
   input  wire [31:0]      virtio_rdata,
   input  wire             virtio_rvalid,   // virtio read-data valid (req/rsp; tolerates CDC-bridge latency)
   input  wire             virtio_irq,
   input  wire             virtio_net_irq,   // PLIC source 12 (ubuntu-nfs.dts virtio@10003000)
   output wire [17:0]      irq_dbg,         // interrupt-path debug for the wrapper ILA (probe_clk)
   // Cache data-array integrity (meaningful only in a -DCACHE_PARITY build; tied to 0
   // otherwise). cache_par_err is the ILA_PARITY TRIGGER: it pulses in the cycle a cache
   // data array returns a word whose parity does not match what was stored -- the moment of
   // corruption, rather than the kernel Oops millions of cycles downstream, which is the
   // only evidence the board has offered so far and is far beyond any pre-trigger depth.
   output wire [1:0]       cache_par_err,   // {I$, D$} 1-cycle error pulse
   output wire [63:0]      cache_par_dbg    // {sticky, bank, addr} of the first failure
);
   localparam SIZE = 1<<RAM_LG2;
   localparam AW   = 64;
   localparam LAW  = AW-6;                  // line address width = 58

   // ---------------- core <-> caches nets ----------------
   wire [PCW-1:0]      imem_addr;
   wire [PCW-1:0]      imem_va;                 // VA of the same fetch -- the buffer's tag
   wire [1:0]          immu_xlvl;               // iMMU leaf level of imem_addr (core -> adapter, stamped per chunk)
   wire [1:0]          imem_lvl_srv;            // served chunk's page size (adapter -> core -> fetch enclosing-page cap)
   wire                imem_xlate_ok, imem_ctx_chg;
   wire [63:0]         imem_satp_q;  wire [1:0] imem_priv_q;
   wire                fe_redirect;
   wire [HW*16-1:0]    imem_data;
   wire [$clog2(HW+2)-1:0] imem_avail;   // sized to the frontend port ($clog2(HW+2)); drive HW, not a literal
   wire                imem_ok;      // the served window is the PC's bytes (late; gates the handshake only)
   wire [2:0]          imem_adv_kind;            // the next PC's chunk, as a kind (fetch.adv_kind)
   wire [63:0]         imem_adv_tgt, imem_adv_red;
   wire [63:0]         dmem_raddr;
   wire                dmem_ren;
   wire                dmem_runcached, dmem_wuncached;   // Svpbmt: NC/IO read/write attribute
   wire                dmem_cbo, dmem_cbo_zero, dmem_cbo_keep;  // Zicbom/Zicboz cache maintenance
   wire [63:0]         dmem_rdata;
   wire [63:0]         dmem_wabase;   // store base PA, unmuxed by the straddle beat
   wire                dmem_rvalid, dmem_wready, dmem_waccept, dmem_idle, ifence;
   wire [55:0]         ptw_addr, dptw_addr;
   wire                ptw_read, dptw_read;
   wire [63:0]         ptw_rdata, dptw_rdata;
   wire                ptw_rvalid, dptw_rvalid;
   wire                redirect;  wire [PCW-1:0] redirect_target;

   ooo2_core #(.HW(HW), .IW(IW), .PCW(PCW), .SEQW(SEQW), .RESET_PC(RESET_PC), .LBASE(LBASE), .LRAM_LG2(LRAM_LG2)) core
     (.clk(clk), .reset(reset),
      .imem_addr(imem_addr), .imem_data(imem_data), .imem_avail(imem_avail), .imem_lvl(imem_lvl_srv), .imem_xlvl(immu_xlvl), .imem_ok(imem_ok), .hw_ip(hw_ip), .mtime(clint_mtime),
      .imem_vaddr(imem_va), .imem_xlate_ok(imem_xlate_ok), .imem_ctx_chg(imem_ctx_chg),
      .imem_adv_kind(imem_adv_kind), .imem_adv_tgt(imem_adv_tgt), .imem_adv_red(imem_adv_red),
      .imem_satp_q(imem_satp_q), .imem_priv_q(imem_priv_q),
      .fe_redirect(fe_redirect), .hpm_fb_hit(1'b0), .hpm_fb_rhit(1'b0),   // no fetch buffer (VHPR I$)
      .hpm_dc_access(dc_access), .hpm_dc_miss(dc_miss), .hpm_ic_access(ic_access), .hpm_ic_miss(ic_miss),
      .dmem_raddr(dmem_raddr), .dmem_ren(dmem_ren), .dmem_runcached(dmem_runcached),
      .dmem_rdata(dmem_rdata), .dmem_rvalid(dmem_rvalid),
      .dmem_wen(dmem_wen), .dmem_waddr(dmem_waddr), .dmem_wabase(dmem_wabase),
      .dmem_wdata(dmem_wdata), .dmem_wmask(dmem_wmask),
      .dmem_wuncached(dmem_wuncached),
      .dmem_cbo(dmem_cbo), .dmem_cbo_zero(dmem_cbo_zero), .dmem_cbo_keep(dmem_cbo_keep),
      .dmem_wready(dmem_wready), .dmem_waccept(dmem_waccept), .dmem_idle(dmem_idle), .ifence(ifence),
      .ptw_addr(ptw_addr), .ptw_read(ptw_read), .ptw_rdata(ptw_rdata), .ptw_rvalid(ptw_rvalid),
      .dptw_addr(dptw_addr), .dptw_read(dptw_read), .dptw_rdata(dptw_rdata), .dptw_rvalid(dptw_rvalid),
            .retire(retire), .retire_pc(), .retire_insn(),
      .retire2(retire2), .retire2_pc(), .retire2_insn(),
      .redirect(redirect), .redirect_target(redirect_target));

   // ---------------- MMIO device routing (CLINT + UART bypass the D$, non-cacheable) ----------------
   localparam [63:0] CLINT_BASE = 64'h0200_0000, UART_BASE = 64'h1000_0000, PLIC_BASE = 64'h0C00_0000;
   // DDR latency HPM window (read-only counters; any write clears). NOT in the DTB -- read it
   // from a bare-metal tool / the monitor; the kernel never touches it.
   localparam [63:0] HPM_BASE   = 64'h1800_0000;
   localparam [63:0] VIRTIO_BASE = 64'h1000_2000;                 // virtio-mmio, 8 KiB: blk @+0x0000, net @+0x1000
   localparam [63:0] BUILDID_BASE = 64'h1000_F000;                // build-id (SMOL/stamp/commit/dirty), probe-core-readable
   // Fetch-buffer diagnostic snapshot, 256 B read-only.  Like HPM_BASE it is deliberately
   // NOT in the DTB: the kernel must never touch it, and the only reader is the ROM monitor
   // after a CPU reset (R1000E008 etc).  See the capture block by the fetch buffer.
   localparam [63:0] FBDIAG_BASE  = 64'h1000_E000;
   // ONE address for every device.  Each device used to mux its own
   //     (dmem_wen & is_<dev>_w) ? dmem_waddr : dmem_raddr
   // which put a 64-bit masked compare between the LSU's combinational store address
   // (dmem_waddr, straight off pa2_q) and the device's address port -- three times over,
   // for CLINT, PLIC and virtio.  The LSU is SINGLE-OUTSTANDING: mem_ren is asserted only
   // in S_LD/S_LD2/S_ARD and mem_wen only in S_ST/S_ST2/S_AWR, so a read and a write are
   // never in flight together and the write-qualification was never carrying information.
   // Selected once, applied at one site; the invariant is asserted below, not assumed.
   // dmem_wabase, NOT dmem_waddr: the per-beat address is (st2_go ? pa2_q : pa_q), and that
   // mux select was the startpoint of the design's worst path at 6 ns -- it reached the device
   // decode's 64-bit compares, then dmem_wready -> lsu_done -> redirect -> iMMU -> u_bp/ycorr.
   // A straddling second beat requires xl_can, which requires pa_dram, so a DEVICE access can
   // never be in S_ST2 and the base is the same address the beat would have used. Asserted below.
   wire [63:0] dev_addr = dmem_wen ? dmem_wabase : dmem_raddr;
   always @(posedge clk) if (!reset & dmem_ren & dmem_wen)
      $fatal(1, "rv_soc_top: dmem_ren & dmem_wen asserted together -- dev_addr select is ambiguous");
   // ...and a device write is never the straddling second beat, which is what makes decoding
   // from the base equivalent. If this ever fires, the decode above is addressing the wrong word.
   always @(posedge clk) if (!reset & dmem_wen & is_dev_w & (dmem_waddr != dmem_wabase))
      $fatal(1, "rv_soc_top: device write straddled a word (beat %h base %h)", dmem_waddr, dmem_wabase);
   wire is_clint_r = (dmem_raddr & ~64'hffff)     == CLINT_BASE;
   wire is_uart_r  = (dmem_raddr & ~64'hf)        == UART_BASE;
   wire is_plic_r  = (dmem_raddr & ~64'h3ff_ffff) == PLIC_BASE;   // 64 MiB region
   wire is_hpm_r   = (dmem_raddr & ~64'hff)        == HPM_BASE;    // 256 B window
   wire is_virtio_r = (dmem_raddr & ~64'h1fff)    == VIRTIO_BASE;   // 8 KiB: blk(+0) + net(+0x1000)
   wire is_buildid_r = (dmem_raddr & ~64'hff)     == BUILDID_BASE;  // 256 B window (read-only)
   wire is_fbdiag_r  = (dmem_raddr & ~64'hff)     == FBDIAG_BASE;   // 256 B window (read-only)
   wire [63:0] fbdiag_rdata;   // driven by the capture block down by the fetch buffer
   wire is_dev_r   = is_clint_r | is_uart_r | is_plic_r | is_hpm_r | is_virtio_r | is_buildid_r | is_fbdiag_r;
   wire is_clint_w = (dmem_wabase & ~64'hffff)     == CLINT_BASE;
   wire is_uart_w  = (dmem_wabase & ~64'hf)        == UART_BASE;
   wire is_plic_w  = (dmem_wabase & ~64'h3ff_ffff) == PLIC_BASE;
   wire is_hpm_w   = (dmem_wabase & ~64'hff)        == HPM_BASE;
   wire is_virtio_w = (dmem_wabase & ~64'h1fff)    == VIRTIO_BASE;
   wire is_dev_w   = is_clint_w | is_uart_w | is_plic_w | is_hpm_w | is_virtio_w;
   // virtio-mmio register access: 32-bit. The probe LSU bus is byte-addressed and
   // RIGHT-ALIGNED -- it presents/consumes "8 bytes @ mem_*addr" with the addressed
   // bytes in the LOW lane and the byte mask low-aligned (store drain: sb_data=raw,
   // dr_mask=low-nbytes). So a 32b reg always sits in [31:0] regardless of its offset;
   // do NOT pick a lane by addr[2] (that picks the empty high lane for 0x014/0x038/...).
   assign virtio_addr  = dev_addr[12:0];
   // virtio read AND write are BOTH REQ/RSP: the FPGA wrapper routes them through a probe_clk<->
   // ui_clk CDC bridge with multi-cycle latency (the sim models it, writes included, via virtio_rvalid
   // a few cycles later). A write MUST block until the bridge DELIVERS it: a fire-and-forget write
   // leaves the store buffer in 1 cycle while still in flight through the CDC FIFO, so the device-scope
   // fence releases the following device read, which then OVERTAKES the write at the device (ILA-
   // confirmed: the driver's DeviceFeaturesSel write reached virtio AFTER its DeviceFeatures read ->
   // read saw stale features -> VERSION_1 -22). Holding the store until virtio_rvalid makes "the read
   // waits for an older device store to DRAIN" == "waits for it to be DELIVERED", so reads can't
   // overtake writes. One virtio op is outstanding at a time (LSU is single-outstanding + the fence
   // serialises device ops), so the single virtio_rvalid completes whichever of read/write is pending.
   reg  vio_pending, vio_wpending;  initial begin vio_pending = 1'b0; vio_wpending = 1'b0; end
   wire vio_req  = dmem_ren & is_virtio_r & ~vio_pending & ~vio_wpending;
   wire vio_wreq = dmem_wen & is_virtio_w & ~vio_wpending & ~vio_pending;
   always @(posedge clk)
      if (reset) begin vio_pending <= 1'b0; vio_wpending <= 1'b0; end
      else begin
         if (vio_req)  vio_pending  <= 1'b1;  else if (virtio_rvalid) vio_pending  <= 1'b0;
         if (vio_wreq) vio_wpending <= 1'b1;  else if (virtio_rvalid) vio_wpending <= 1'b0;
      end
   assign virtio_read  = vio_req;
   assign virtio_write = vio_wreq;
   assign virtio_wdata = dmem_wdata[31:0];   // right-aligned: 32b store data is always low lane
   assign virtio_be    = dmem_wmask[3:0];    // and its byte mask is low-aligned (see note above)
   // clint/uart/plic: combinational rdata valid the cycle after the ren pulse (fixed 1-cycle
   // dev_rvalid). virtio is EXCLUDED here -- it completes via the virtio_rvalid req/rsp above.
   // device write accepts in 1 cycle (~dev_wack masks the held wen so it writes once).
   reg  dev_rvalid, dev_wack;
   always @(posedge clk) if (reset) begin dev_rvalid<=1'b0; dev_wack<=1'b0; end
      else begin dev_rvalid <= dmem_ren & is_dev_r & ~is_virtio_r;
                 dev_wack <= dmem_wen & is_dev_w & ~is_virtio_w & ~dev_wack; end   // virtio: own req/rsp
   wire [63:0] clint_rdata;  wire clint_mtip, clint_msip;  wire [63:0] clint_mtime;
   // SCALE_DIV is DERIVED from the probe clock so it tracks a PROBE_CLK_DIV8 sweep: the DTB
   // declares timebase-frequency = 501253, so the CLINT must tick at that rate at ANY
   // probe_clk. Hardcoding 133 (the 66.67 MHz value) made mtime run 1.67x fast at 111 MHz --
   // every kernel deadline, TCP timeout and NFS retry skewed by that factor.
   //
   // NOTE the residual error, which the old integer ladder also had: SCALE_DIV is an integer,
   // so the tick rate is probe_clk/round(probe_clk/501253), not 501253 exactly.  At 66.67 MHz
   // it is exact (133); at 111.11 MHz it is 221 -> 502765 Hz, i.e. the timebase runs 0.30%
   // fast and the GB5 milestone run carried that error.  Now that probe_clk is continuous,
   // the sweep should PREFER frequencies where this divides cleanly.
   clint #(.SCALE_DIV(`PROBE_CLK_HZ / 501_253)) u_clint
     (.clk(clk), .reset(reset),
      .we(dmem_wen & is_clint_w & ~dev_wack),
      .addr(dev_addr[15:0]),
      .wdata(dmem_wdata), .wmask(dmem_wmask), .rdata(clint_rdata),
      .mtip(clint_mtip), .msip(clint_msip), .o_mtime(clint_mtime));
   // PLIC (SiFive layout @ 0x0C00_0000): external-interrupt controller.
   // Sources: 10 = UART (DTS interrupts=<10>), 11 = virtio_blk.
   wire [63:0] plic_rdata;  wire plic_meip, plic_seip;  wire [11:0] plic_dbg;
   wire [63:0] plic_addr = dev_addr;
   wire        uart_irq;
   plic u_plic
     (.clk(clk), .reset(reset),
      .we(dmem_wen & is_plic_w & ~dev_wack), .re(dmem_ren & is_plic_r),
      .addr(plic_addr[23:0]), .wdata(dmem_wdata), .wmask(dmem_wmask), .rdata(plic_rdata),
      .src({51'd0, virtio_net_irq, virtio_irq, uart_irq, 10'd0}), .meip(plic_meip), .seip(plic_seip),
      .dbg(plic_dbg));
   // interrupt-path debug bus out to the wrapper's ILA: {plic src-11 lifecycle (12), a plic MMIO
   // access strobe + its low addr nibble to time claim(0x004)/complete}.
   assign irq_dbg = {dmem_ren & is_plic_r, dmem_wen & is_plic_w, plic_addr[3:0], plic_dbg};

   // ---- cache data-array integrity taps (see the port comment; -DCACHE_PARITY) ----
`ifdef CACHE_PARITY
   // The trigger is EITHER integrity failure: a parity mismatch (the array returned
   // something other than what was stored) or an address-provenance failure (the array
   // returned the wrong ROW, which parity cannot see and which is the class the board's
   // surviving fault belongs to).
   assign cache_par_err = {u_icache.par_err | u_icache.adr_err,
                           u_dcache.par_err | u_dcache.adr_err};
   assign cache_par_dbg = {u_dcache.par_sticky, u_icache.par_sticky,
                           u_dcache.adr_sticky, u_icache.adr_sticky,
                           u_dcache.par_bank, u_icache.par_bank,
                           {(64-8-2*16){1'b0}},
                           u_dcache.par_addr16, u_icache.par_addr16};
`else
   assign cache_par_err = 2'd0;
   assign cache_par_dbg = 64'd0;
`endif
   wire [11:0] hw_ip = (clint_mtip ? 12'h080 : 12'h0) | (clint_msip ? 12'h008 : 12'h0)
                     | (plic_meip  ? 12'h800 : 12'h0) | (plic_seip  ? 12'h200 : 12'h0);
   // NS16550A UART. Semantics ported from the scalar core's Ubuntu-proven model
   // (src/smolrv64.v): full register file (IER/IIR/FCR/MCR/SCR readback), the THRE-pending
   // protocol (set when the THR drains or on a THRI enable edge with the THR empty; cleared
   // by a THR write, THRI disable, or reading IIR while THRE is the reported cause), and an
   // interrupt (RX-DR | THRE) to PLIC source 10. Interrupt-driven TX is load-bearing: the
   // DTS declares the IRQ, so the 8250 driver waits for a THRE interrupt to drain its xmit
   // buffer -- without it userspace wedges in its first console write() while polled printk
   // still looks healthy. DTS fifo-size=<1> makes the 1-deep THR/RBR compliant.
   reg [3:0] uart_ier;  reg uart_fcr_fifo;  reg [4:0] uart_mcr;  reg [7:0] uart_scr;
   reg [7:0] uart_lcr;  integer ub;
   reg [7:0] uart_rbr;  reg uart_dr;       // RX holding register + data-ready
   reg [7:0] uart_thr;  reg uart_thr_full; // TX holding register + pending flag
   reg       uart_thre_pending;
   assign    uart_rx_ready = ~uart_dr;
   // TX byte stream: hold the THR byte until rs232tx accepts it (FPGA backpressure).
   // In a sim TB uart_tx_ready is tied 1, so the byte drains next cycle (THRE stays high).
   assign    uart_tx_valid = uart_thr_full;
   assign    uart_tx_data  = uart_thr;
   wire      uart_rx_ip    = uart_ier[0] & uart_dr;
   wire      uart_thre_ip  = uart_ier[1] & uart_thre_pending;
   wire      uart_iir_thre = ~uart_rx_ip & uart_thre_ip;
   // IIR: bit0=1 means NO interrupt pending; RX-DR (0x4) outranks THRE (0x2); [7:6]=FIFO en
   wire [7:0] uart_iir = uart_rx_ip    ? {uart_fcr_fifo, uart_fcr_fifo, 2'b0, 4'h4}
                       : uart_iir_thre ? {uart_fcr_fifo, uart_fcr_fifo, 2'b0, 4'h2}
                       :                 {uart_fcr_fifo, uart_fcr_fifo, 2'b0, 4'h1};
`ifdef OOO2_IRQ_STIM
   // SIM-ONLY interrupt stimulus (2026-09-17). Every device IRQ is tied off in the cosim, so no
   // simulation had ever taken a PLIC interrupt against the out-of-order pipe -- the board's
   // NIC death under CTF-on-FP was invisible. A spurious level on the UART's PLIC source for
   // 256 of every 32768 cycles makes the 8250's fasteoi flow run (no action: mask + eoi) --
   // thousands of external-interrupt entries, claims, completes and irqop injections.
   // src/plic.v treats source 10 as enabled at priority >= 1 under the same define.
   // Armed by the kernel's first S-mode write to the UART: the console handover, which comes
   // after the 8250 probe has mapped hwirq 10. Any earlier claim would hit an unmapped hwirq,
   // the kernel would never complete it and the gateway would stick in service for good.
   // Never defined for the FPGA build.
   reg        stim_on;   initial stim_on = 1'b0;
   reg [14:0] stim_ctr;  initial stim_ctr = 15'd0;
   always @(posedge clk) begin
      stim_ctr <= stim_ctr + 1'b1;
      if (dmem_wen & is_uart_w & ~dev_wack & (core.mmu_priv == 2'd1)) stim_on <= 1'b1;
   end
   assign    uart_irq = uart_rx_ip | uart_thre_ip | (stim_on & (stim_ctr[14:8] == 7'd0));
`else
   assign    uart_irq = uart_rx_ip | uart_thre_ip;
`endif
   // Read strobes: UART_BASE is 16-aligned and is_uart_r bounds the window, so the register
   // offset is just the low address bits. RBR read pops DR; IIR read clears THRE-pending
   // when THRE is the cause being reported.
   wire [2:0] uart_roff   = dmem_raddr[2:0];
   wire      uart_rbr_rd = dmem_ren & is_uart_r & (uart_roff == 3'd0) & ~uart_lcr[7];
   wire      uart_iir_rd = dmem_ren & is_uart_r & (uart_roff == 3'd2);
   // Like the PLIC claim, read data REGISTERS at the ren strobe and delivers on the 1-cycle-
   // later dev_rvalid, so a side-effecting read returns its pre-side-effect value.
   reg [63:0] uart_rdata_q;
   always @(posedge clk) if (reset) begin
         uart_ier<=4'd0; uart_fcr_fifo<=1'b0; uart_mcr<=5'd0; uart_scr<=8'd0;
         uart_lcr<=8'd0; uart_dr<=1'b0; uart_rbr<=8'd0;
         uart_thr<=8'd0; uart_thr_full<=1'b0; uart_thre_pending<=1'b0; uart_rdata_q<=64'd0;
      end
      else begin
         if (uart_thr_full & uart_tx_ready) begin
            uart_thr_full <= 1'b0;             // serializer took the byte -> THR empty
            uart_thre_pending <= 1'b1;
         end
         if (dmem_ren & is_uart_r) begin
            uart_rdata_q <= uart_rd(dmem_raddr, uart_rbr, uart_dr, uart_lcr, uart_thr_full,
                                    uart_ier, uart_iir, uart_mcr, uart_scr);
            if (uart_iir_rd & uart_iir_thre) uart_thre_pending <= 1'b0;
         end
         if (dmem_wen & is_uart_w & ~dev_wack)
            for (ub=0; ub<8; ub=ub+1) if (dmem_wmask[ub])
               case ((dmem_waddr[2:0] + ub) & 3'h7)   // 16-aligned base, window bounded by is_uart_w: the low bits ARE the offset
                  3'd0: if (!uart_lcr[7]) begin // THR
                           uart_thr <= dmem_wdata[ub*8 +: 8]; uart_thr_full <= 1'b1;
                           uart_thre_pending <= 1'b0;
                           $write("%c", dmem_wdata[ub*8 +: 8]);   // sim-only; synth ignores
                        end
                  3'd1: if (!uart_lcr[7]) begin // IER: THRI enable edge w/ room -> immediate THRE
                           uart_ier <= dmem_wdata[ub*8 +: 4];
                           if (!dmem_wdata[ub*8+1])                 uart_thre_pending <= 1'b0;
                           else if (!uart_ier[1] && !uart_thr_full) uart_thre_pending <= 1'b1;
                        end
                  3'd2: begin                   // FCR (write-only): FIFO enable + RX/TX resets
                           uart_fcr_fifo <= dmem_wdata[ub*8];
                           if (dmem_wdata[ub*8+1]) uart_dr <= 1'b0;
                           if (dmem_wdata[ub*8+2]) begin
                              uart_thr_full <= 1'b0;
                              if (uart_ier[1]) uart_thre_pending <= 1'b1;
                           end
                        end
                  3'd3: uart_lcr <= dmem_wdata[ub*8 +: 8];
                  3'd4: uart_mcr <= dmem_wdata[ub*8 +: 5];
                  3'd7: uart_scr <= dmem_wdata[ub*8 +: 8];
                  default: ;
               endcase
         if (uart_rx_we & ~uart_dr) begin uart_rbr <= uart_rx_data; uart_dr <= 1'b1; end
         else if (uart_rbr_rd)      uart_dr <= 1'b0;
      end
   function [63:0] uart_rd;
      input [63:0] a; input [7:0] rbr; input dr; input [7:0] lcr; input thr_full;
      input [3:0] ier; input [7:0] iir; input [4:0] mcr; input [7:0] scr;
      integer b2; reg [2:0] off; reg [63:0] uoff;
      begin uart_rd=64'd0; for (b2=0;b2<8;b2=b2+1) begin
         uoff=a-UART_BASE+b2; off=uoff[2:0];   // narrow explicitly, not at the assignment
         uart_rd[b2*8 +: 8] =
              (off==3'd0) ? (lcr[7] ? 8'h00 : rbr)                       // RBR (DLL if DLAB)
            : (off==3'd1) ? (lcr[7] ? 8'h00 : {4'd0, ier})               // IER (DLM if DLAB)
            : (off==3'd2) ? iir                                          // IIR
            : (off==3'd3) ? lcr                                          // LCR
            : (off==3'd4) ? {3'd0, mcr}                                  // MCR
            : (off==3'd5) ? ((thr_full?8'h00:8'h60) | (dr?8'h01:8'h00))  // LSR: TEMT|THRE|DR
            : (off==3'd6) ? 8'hB0                                        // MSR: CTS+DSR+CD
            :               scr; end end                                 // SCR
   endfunction
   wire [63:0] hpm_rdata;
   // build-id: 32b words @ 0x1000_F0{00,04,08,0c,10,14}, replicated into both 64b lanes so the
   // LSU extracts the correct half for the load offset (same trick as virtio above). The defines
   // land in localparams first -- part-selecting a bare `define literal isn't legal in Vivado.
   localparam [63:0] BID_STAMP  = `SMOLRV64_BUILD_STAMP;
   localparam [31:0] BID_COMMIT = `SMOLRV64_GIT_COMMIT;
   localparam        BID_DIRTY  = `SMOLRV64_GIT_DIRTY;
   reg [31:0] build_id_word;
   always @* case (dmem_raddr[5:2])
      4'h0:    build_id_word = 32'h534d_4f4c;   // "SMOL" magic -- monitor gates the block on this
      4'h1:    build_id_word = 32'h0000_0001;   // version
      4'h2:    build_id_word = BID_STAMP[31:0];
      4'h3:    build_id_word = BID_STAMP[63:32];
      4'h4:    build_id_word = BID_COMMIT;      // <- rtl= identity (the loaded RTL commit)
      4'h5:    build_id_word = {31'd0, BID_DIRTY};
      default: build_id_word = 32'd0;
   endcase
   wire [63:0] dev_rdata = is_clint_r  ? clint_rdata
                         : is_uart_r   ? uart_rdata_q
                         : is_plic_r   ? plic_rdata
                         : is_virtio_r ? {virtio_rdata, virtio_rdata}  // 32b reg replicated into both lanes: the LSU extracts the half for the load offset, so this is correct regardless of which lane it picks (all virtio-mmio regs are 32b, accessed 32b)
                         : is_buildid_r ? {build_id_word, build_id_word}
                         : is_fbdiag_r ? fbdiag_rdata
                         : is_hpm_r    ? hpm_rdata   : 64'd0;

   // Device read data is REGISTERED before it reaches the load return mux.  It used to be
   // combinational, which put the SLOWEST DEVICE'S read cone on the critical path of every
   // DRAM cache hit -- a load that never touches a device.  Measured worst path at 111 MHz
   // (8.980 ns of 9.000, 32 levels, 70% routing):
   //   u_lsu/pa2_q -> dmem_waddr -> is_clint_w (64-bit compare, the CARRY8s)
   //                -> clint mtime logic -> clint_rdata -> dmem_rdata -> m_result
   // Note the startpoint: a STORE address (dmem_waddr is combinational from pa2_q) reached a
   // concurrent LOAD's return data, coupling two accesses through a device neither touches.
   //
   // MMIO is rare and already multi-cycle, so the extra cycle of latency costs no measurable
   // IPC.  Side effects are unaffected: PLIC's claim fires on `re` (= dmem_ren & is_plic_r)
   // and its rdata is registered valid the cycle after, i.e. exactly during dev_rvalid -- this
   // captures that same value and merely delivers it one cycle later.
   reg [63:0] dev_rdata_q;  reg dev_rvalid_q;
   always @(posedge clk) if (reset) dev_rvalid_q <= 1'b0;
      else begin dev_rvalid_q <= dev_rvalid; dev_rdata_q <= dev_rdata; end

   // ---------------- D$ (write-through) + read/write adapters (proven in tb_vl), device-muxed ----------------
   reg          c_rd_pend;
   wire [63:0]  dc_rd_data;  wire dc_rd_valid, dc_wr_ack, dc_wr_acc, dc_wr_cpl;  wire [63:0] dc_rd_resp_addr;
   wire         dc_l2_req, dc_l2_we;  wire [LAW-1:0] dc_l2_addr;  wire [511:0] dc_l2_wdata;
   wire [511:0] dc_l2_rdata;  wire dc_l2_ack;
   // The LSU is single-outstanding but a SQUASH abandons an in-flight load and issues a new one
   // ("a new mem_ren supersedes any prior unfinished read"). The cache, already committed to the
   // squashed address, would otherwise deliver that stale line to the new load. Match the cache's
   // response address to the current request (like the I$ does with i_pa); a non-matching response
   // is discarded and the request re-issues for the new address.
   // Response matched by an ALLOCATED TAG, not by address (docs/rtl-rules.md). The old
   //     dc_rv_ok = dc_rd_valid & (dc_rd_resp_addr == dmem_raddr)
   // was a 64-bit equality whose CARRY8 chain sat on the design's worst path at 6 ns:
   //   u_lsu/mem_raddr -> this compare -> dmem_rvalid -> lsu_done -> m_done -> redirect
   //                   -> u_fetch/va_q -> fe/u_bp/ycorr_q
   // Tag = {client, generation}: client 0 = LSU, 1 = iPTW, 2 = dPTW. The LSU's generation
   // toggles on every new read, so a response to a SUPERSEDED load (a squash re-issues at a
   // new address) carries the stale generation and is discarded -- exactly what the address
   // compare achieved, in 4 bits instead of 64.
   localparam DRTW = 4;
   reg          lsu_gen;
   always @(posedge clk) if (reset) lsu_gen <= 1'b0; else if (dmem_ren) lsu_gen <= ~lsu_gen;
   // The tag must be CONSTANT for the request's whole lifetime. lsu_gen flips at the clock
   // edge on dmem_ren, so during the request cycle itself the cache would capture the
   // PRE-flip value while every later compare used the POST-flip one -- legitimate responses
   // mismatch, get discarded, and the load re-issues. Present the value lsu_gen is ABOUT to
   // take; compare against the registered one, which equals it from the next cycle on (a
   // response cannot arrive in the capture cycle -- rd_valid is registered).
   wire [DRTW-1:0] lsu_tag_req = {2'd0, 1'b0, lsu_gen ^ dmem_ren};   // -> the cache
   wire [DRTW-1:0] lsu_tag     = {2'd0, 1'b0, lsu_gen};              // -> the compare
   wire [DRTW-1:0] dc_rd_resp_tag;
   wire         dc_rv_ok   = dc_rd_valid & (dc_rd_resp_tag == lsu_tag);
   // virtio completes on its req/rsp virtio_rvalid (CDC latency); clint/uart/plic on the fixed
   // 1-cycle dev_rvalid (combinational rdata valid at delivery -- PLIC's registered read lands
   // exactly here, so the side-effecting CLAIM reads correctly); cache on dc_rv_ok.
   // Same collapse as dmem_wready, on the READ side. This SELECT was the worst path at 6 ns
   // after the write side was fixed:
   //   u_lsu/mem_raddr -> is_uart_r (CARRY8 x3) -> is_dev_r -> raw_rvalid
   //                   -> lsu_done -> redirect -> the decoupling queue -> fe/u_bp/ycorr_q
   // and the select carries no information -- each term is gated by its own device class at
   // its source, so at most one is ever asserted:
   //   dev_rvalid  <= dmem_ren & is_dev_r & ~is_virtio_r
   //   c_rd_req     = (dmem_ren | c_rd_pend) & ~dc_rv_ok & ~is_dev_r  (no cache req for a device)
   //   vio_pending <= set only by dmem_ren & is_virtio_r
   wire         vio_rack   = virtio_rvalid & vio_pending;   // not a write's completion
   wire         raw_rvalid = vio_rack | dev_rvalid_q | dc_rv_ok;
   always @(posedge clk)
      if (!reset & ((vio_rack & dev_rvalid_q) | (vio_rack & dc_rv_ok) | (dev_rvalid_q & dc_rv_ok)))
         $fatal(1, "rv_soc_top: two read responses at once (vio=%b dev=%b dc=%b) -- rvalid ambiguous",
                vio_rack, dev_rvalid_q, dc_rv_ok);
   // ...and raw_rDATA selects the same way, for the same reason. The note that used to stand
   // here -- "a 64-bit mux on a path with slack, and only the VALID reaches lsu_done" -- has
   // expired: at 166 MHz this select was the head of the SECOND-WORST family in the design,
   // 57 failing endpoints at WNS -0.215:
   //   u_lsu/mem_raddr -> is_uart_r (CARRY8 x3) -> is_dev_r -> THIS MUX -> the load byte
   //                   -> align/sign-extend -> m_byp_val -> the X-stage ALU -> m_result
   // 1.27 ns of it spent deciding WHICH responder to listen to, before the 64-bit mux began.
   // The assertion directly above is what licenses the collapse: at most one response lands
   // per cycle, so the responder that FIRED names its own data and no address is re-decoded.
   // Strictly more robust, too -- the select no longer depends on mem_raddr still holding the
   // address the response belongs to (a response is matched by the requester's own bookkeeping,
   // not by an address; docs/rtl-rules.md).
   wire [63:0]  raw_rdata  = vio_rack     ? {virtio_rdata, virtio_rdata}  // 32b reg, valid at virtio_rvalid
                           : dev_rvalid_q ? dev_rdata_q
                           :                dc_rd_data;
   // TWO QUESTIONS, and the one pending bit used to answer both. c_rd_want is "the cache
   // still owes us an accept"; c_rd_pend is "a response is still coming". They coincide only
   // while the cache serves one request at a time. With a pipelined port they do not: rd_valid
   // is REGISTERED, so during the cycle a response is being produced the old request is still
   // asserted, and a cache that can accept in that cycle takes it twice. Dropping on the ack
   // is what makes the request unambiguous (docs/rtl-rules.md D5).
   reg          c_rd_want;
   wire         c_rd_req = (dmem_ren | c_rd_want) & ~is_dev_r;
   wire         lsu_rd_ack;                       // this cycle's accept belonged to the LSU
   always @(posedge clk) if (reset) c_rd_want<=1'b0;
      else if (dmem_ren & ~lsu_rd_ack) c_rd_want<=1'b1;
      else if (lsu_rd_ack)             c_rd_want<=1'b0;
   // ...and c_rd_pend keeps its old meaning and its old users (c_st_ok below reads it as
   // "a response is outstanding"), so it is still cleared by the response, not the accept.
   always @(posedge clk) if (reset) c_rd_pend<=1'b0;
      else if (dmem_ren) c_rd_pend<=1'b1; else if (raw_rvalid) c_rd_pend<=1'b0;
   reg          c_rdv_st;  reg [63:0] c_rdd_st;
   always @(posedge clk) if (reset) c_rdv_st<=1'b0;
      else if (dmem_ren) c_rdv_st<=1'b0;
      else if (raw_rvalid) begin c_rdv_st<=1'b1; c_rdd_st<=raw_rdata; end
   wire         c_st_ok = c_rdv_st & ~c_rd_pend & ~dmem_ren;
   assign       dmem_rdata  = c_st_ok ? c_rdd_st : raw_rdata;
   assign       dmem_rvalid = raw_rvalid | c_st_ok;
   // virtio store completes only when the bridge has DELIVERED it (virtio_rvalid) -- blocking, so the
   // fence-released next read cannot overtake it; other device writes accept in 1 cycle (dev_wack).
   // dmem_wready used to SELECT among these with is_dev_w/is_virtio_w -- 64-bit masked
   // compares off the LSU's store address -- which put the device decode on
   //   pa_q -> is_uart_w (CARRY8 x3) -> dmem_wready -> lsu_done -> redirect -> u_bp/ycorr,
   // the design's worst path at a 6 ns constraint (226 paths, 37 levels).
   //
   // The select carried no information. Each term is already gated by its own device class
   // AT ITS SOURCE, so at most one can ever be asserted:
   //   dev_wack      <= dmem_wen & is_dev_w & ~is_virtio_w & ~dev_wack
   //   cache .wr_req  = dmem_wen & ~dc_wr_cpl & ~is_dev_w      (a device store never reaches it)
   //   vio_wpending  <= set only by dmem_wen & is_virtio_w
   // So an OR is equivalent -- and it is an OR of registered signals, with no compare in it.
   // The exclusivity is asserted rather than assumed.
   wire         vio_wack = virtio_rvalid & vio_wpending;
   // dc_wr_cpl, NOT dc_wr_ack: the LSU leaves a plain cached store at the D$'s ACCEPT (below),
   // so the ack that write produces two cycles later is nobody's -- and it would land while
   // the LSU waits on a device or virtio store started since, completing THAT one early. The
   // D$ raises wr_cpl only for the writes whose requester waits (CBO, uncached), which the
   // LSU serializes, so the three terms stay exclusive (asserted). Plan item 4, 2026-09-04.
   assign       dmem_wready = vio_wack | dev_wack | dc_wr_cpl;
   // The ACCEPT, for the LSU's store state: the D$ captures a plain write in the cycle it
   // takes it (wr_acc, combinational from the same cone as rd_ack); a device or virtio
   // write is done at its own ack. The device acks MUST be here and not only in wready:
   // a device store carries no NC bit when there are no page tables (bare M-mode has no
   // PBMT), so the LSU classes it as plain and waits on the accept -- and while it waited,
   // the device path re-executed the write every other cycle (cbozero's UART printed `c`
   // forever, 2026-09-05). The three terms are exclusive: a cache accept needs ~is_dev_w,
   // and the LSU has one write in flight.
   assign       dmem_waccept = dc_wr_acc | dev_wack | vio_wack;
   always @(posedge clk)
      if (!reset & ((vio_wack & dev_wack) | (vio_wack & dc_wr_cpl) | (dev_wack & dc_wr_cpl)))
         $fatal(1, "rv_soc_top: two write acks at once (vio=%b dev=%b dc=%b) -- wready ambiguous",
                vio_wack, dev_wack, dc_wr_cpl);

   // D$ is WRITE-BACK (WRTHRU=0): stores ack into the line (dirty), evicted lazily -- the
   // store buffer drains in ~1-2c instead of a full L2 round-trip. PTW reads are routed THROUGH
   // the D$ (the dcr_* read-port arbiter below), so a page-table walk always sees dirty PTEs --
   // the D$ is the coherency point. sfence.vma therefore needs NO D$ flush (just a TLB flush);
   // only fence.i still clean-flushes (the I$ reads L2 directly) -- see the df_* FSM below.
   wire        dcr_req;  wire [63:0] dcr_addr;     // muxed D$ read port (LSU + 3 PTW), assigned below
   wire dc_inv_req, dc_inv_busy;
   // Zihpm cache-event taps (D$/I$ line-lookup + miss pulses) -> core hpm_ev.
   wire dc_access, dc_miss, ic_access, ic_miss;
   rv_cache #(.PAW(64), .PAW_SIG(34), .SIZE_KB(SIZE_KB), .RDW(64), .WDW(64), .WRITABLE(1), .WRTHRU(0), .PREFETCH(0), .PERF_ID(1)) u_dcache
     (.clk(clk), .reset(reset),
      .rd_req(dcr_req), .rd_addr(dcr_addr), .rd_pa(dcr_addr), .rd_data(dc_rd_data), .rd_valid(dc_rd_valid),
      .rd_ack(dc_rd_ack),
      .rd_resp_addr(dc_rd_resp_addr), .rd_tag(dcr_tag), .rd_resp_tag(dc_rd_resp_tag),
      // Svpbmt: only a LSU load read can be NC (PTW reads share dcr but are always cacheable -> 0
      // when c_rd_req is low). The store's NC bit qualifies the write port.
      .rd_uncached(c_rd_req & dmem_runcached), .wr_uncached(dmem_wuncached),
      // ~dc_wr_cpl: a CBO or uncached write holds its request through the cycle its completion
      // is registered, and the D$ is idle again in that cycle -- without this it is taken twice.
      .cbo_req(dmem_cbo & ~dc_wr_cpl & ~is_dev_w), .cbo_zero(dmem_cbo_zero), .cbo_keep(dmem_cbo_keep),
      .wr_req(dmem_wen & ~dc_wr_cpl & ~is_dev_w), .wr_addr(dmem_waddr), .wr_data(dmem_wdata),
      .wr_mask(dmem_wmask), .wr_ack(dc_wr_ack), .wr_acc(dc_wr_acc), .wr_cpl(dc_wr_cpl), .inv_req(dc_inv_req), .inv_clean(1'b1), .ep_bump(1'b0), .inv_busy(dc_inv_busy),
      .l2_req(dc_l2_req), .l2_we(dc_l2_we), .l2_addr(dc_l2_addr), .l2_wdata(dc_l2_wdata),
      .l2_rdata(dc_l2_rdata), .l2_ack(dc_l2_ack),
      .perf_access(dc_access), .perf_miss(dc_miss));

   // D$ clean-flush on fence.i ONLY: drain the store buffer, then clean-flush the D$ (write back
   // dirty lines, keep them valid). FENCE.I needs this because the I$ reads L2/DDR directly: with
   // a write-back D$, freshly-stored code sits DIRTY in the D$, so the I$ would refetch STALE
   // bytes after a bare invalidate -- the D$ must write back first. The I$-invalidate FSM (fi)
   // below waits for this flush (df==DF_IDLE) before invalidating, so DDR is current before the
   // refetch. sfence.vma does NOT trigger this: PTW reads go through the D$ (coherent), so the
   // walk never sees stale memory -- sfence only flushes the TLB (in the MMU).
   // fence.i CHANGES code: drain + clean-flush the D$ (df) then CLEAR the I$ (fi), so the refetch
   // reads current L2. A MAPPING change (satp/sfence) does NOT change code -- it only stales
   // virtual tags -- so it advances the I$ EPOCH (ic_ep_bump): the I$ retains its lines and
   // reconciles them by physical tag on refetch (docs/VHPR.md), with no D$ writeback, no clear.
   wire ic_ep_bump = imem_ctx_chg;
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

   // ---------------- VHPR I$ (virtually hit) + alignment adapter + fence.i FSM ----------------
   // Stage 2: the I$ is virtually indexed/tagged (rd_addr = the fetch VA, rd_pa = its PA), so
   // translation is off the hit path and the ~500-line run-ahead fetch buffer is gone. What is
   // left is a small alignment adapter: two VA-tagged 16-byte chunk slots hold the recent I$
   // reads, the aligner is windowed across the PC's chunk and the next, and one demand I$ read
   // refills in flight. Permissions/faults stay the iMMU's (immu_fault, in ooo2_core), so a
   // plain U<->S privilege change needs NO I$ invalidation -- only a MAPPING change
   // (imem_ctx_chg, narrowed to satp/sfence in ooo2_core) or fence.i does. Stage 2a invalidates
   // with the I$'s existing full flush; the 2-bit epoch that makes it a 1-cycle bump is 2b.
   localparam integer CHB = HW*2;              // chunk bytes = one 16-byte I$ read
   localparam integer CHA = $clog2(HW*2);      // chunk-align shift (=4)

   wire [HW*16-1:0] ic_rd_data;   wire ic_rd_valid, ic_inv_busy;
   wire [63:0]      ic_rd_resp_addr;   wire [3:0] ic_rsp_tag;   wire ic_rd_ack;
   wire             ic_l2_req, ic_l2_we;   wire [LAW-1:0] ic_l2_addr;   wire [511:0] ic_l2_wdata;
   wire [511:0]     ic_l2_rdata;   wire ic_l2_ack;

   // The deleted FBDIAG readout window and its self-reset: tied off (no buffer to observe).
   assign fbdiag_rdata     = 64'd0;
   assign fbdiag_reset_req = 1'b0;

   // fence.i: drain the store buffer + D$ clean-flush (df, above), THEN invalidate the I$ so a
   // refetch cannot read code still dirty in the D$. A MAPPING change invalidates directly --
   // virtual-alias staleness, not I/D coherence, so no D$ writeback wait. Both drive the flush.
   localparam FI_IDLE=0, FI_DRAIN=1, FI_INV=2, FI_WAIT=3;
   reg [1:0] fi;  wire fi_stall = (fi != FI_IDLE);
   reg fi_inv;
   always @(posedge clk) if (reset) begin fi<=FI_IDLE; fi_inv<=1'b0; end
      else begin
         fi_inv <= 1'b0;
         case (fi)
           FI_IDLE:  if (ifence) fi<=FI_DRAIN;
           FI_DRAIN: if (dmem_idle & (df == DF_IDLE)) begin fi_inv<=1'b1; fi<=FI_INV; end
           FI_INV:   fi<=FI_WAIT;
           FI_WAIT:  if (!ic_inv_busy) fi<=FI_IDLE;
         endcase
      end
   wire ic_inv_req = fi_inv;   // fence.i AND mapping changes go through fi (after the D$ clean-flush)

   // The PC's 16-byte chunk (VA) and its PA (the iMMU translated the PC; aligning to 16 stays
   // in the same page), and the next chunk's VA.
   wire [63:0] pc_ca   = {imem_va[63:CHA],   {CHA{1'b0}}};
   wire [63:0] pc_paca = {imem_addr[63:CHA], {CHA{1'b0}}};
   wire [63:0] nx_ca   = pc_ca + CHB;
   wire [63:0] pc_ca_m = pc_ca - CHB;   // only ever latched (rq_vm below): never in the served path

   // Two VA-tagged chunk slots -- the alignment window.
   // EACH SLOT ALSO HOLDS ITS VA MINUS ONE CHUNK (c*_vm), so "is this the PC's NEXT chunk" is
   // c_vm == pc_ca, a compare of two registers, not c_va == pc_ca + CHB. The adder was inside
   // the fetch loop: pc_q -> +CHB -> h1 -> have1 -> the served window -> the aligner -> the
   // next PC and the BTB index, 24 levels, the worst family of the IW=3 census (~1.9 ns from
   // pc_q to the compare's result). The adder still feeds the demand-read address (want_va),
   // which is a BRAM address, in parallel with the compares instead of in front of them.
   reg              c0_v, c1_v;
   reg  [63:0]      c0_va, c1_va, c0_vm, c1_vm;
   reg  [HW*16-1:0] c0_d,  c1_d;
   reg  [1:0]       c0_lvl, c1_lvl;              // page size of each resident chunk (iMMU leaf level at fill)
   initial begin c0_v=1'b0; c1_v=1'b0; end

   wire h0_0 = c0_v & (c0_va == pc_ca);   wire h0_1 = c1_v & (c1_va == pc_ca);
   wire h1_0 = c0_v & (c0_vm == pc_ca);   wire h1_1 = c1_v & (c1_vm == pc_ca);
   wire             have0    = h0_0 | h0_1;
   wire             have1    = h1_0 | h1_1;
   // THE SERVED SLOT IS A REGISTER, DECIDED A CYCLE EARLY. sh_q names the slot holding the
   // PC's chunk and n1_q says the other slot holds the next one; both are picked at the
   // previous edge from fetch's adv_kind (the next PC's chunk: same, next, predicted
   // target, redirect target) against the tags AS THEY WILL BE after that edge (the fill
   // included). The live compares h0_*/h1_* no longer select data -- they only gate
   // imem_ok, one late bit -- so pc_q -> tag compare -> chunk select leaves the fetch loop.
   // A wrong pick (none is expected; asserted below) costs one bubble: imem_ok drops, the
   // fetch holds, and the HOLD arm re-picks from the live tags.
   localparam [2:0] AK_HOLD = 3'd0, AK_SAME = 3'd1, AK_NEXT = 3'd2, AK_TGT = 3'd3, AK_REDIR = 3'd4;
   reg sh_q, n1_q;   initial begin sh_q = 1'b0; n1_q = 1'b0; end
   wire [HW*16-1:0] pc_chunk = sh_q ? c1_d : c0_d;
   wire [HW*16-1:0] nx_chunk = sh_q ? c0_d : c1_d;                 // the OTHER slot
   wire ok_slot = sh_q ? h0_1 : h0_0;                              // live: the hinted slot is the PC's chunk
   wire ok_next = ~n1_q | (sh_q ? h1_0 : h1_1);                    // live: bytes 16..31 promised only if truly the next chunk
   // The served (PC) chunk's page size drives the enclosing-page cap (fetch) and whether the
   // NEXT chunk is still in the same page (samepg): 4 KiB for a 4K leaf, 2 MiB for >=2M (a 1 GiB
   // leaf caps as 2 MiB). Within a superpage the next chunk's PA is pc_paca+CHB (contiguous).
   // The SERVED window's page size follows the hinted slot (it only matters under imem_ok,
   // when the hint is right); the DEMAND-READ decision below takes the page size of the slot
   // that really hits -- a wrong-hint cycle must never turn a 4 KiB boundary into a 2 MiB one
   // and issue a next-chunk fill whose PA (`pc_paca + CHB`) crosses the page.
   wire [1:0]       pc_lvl   = h0_1 ? c1_lvl : c0_lvl;             // the slot that hits (live)
   assign           imem_lvl_srv = sh_q ? c1_lvl : c0_lvl;         // the hinted slot's
   wire             big_pg   = (pc_lvl != 2'd0);
   wire             samepg   = big_pg ? (pc_ca[20:CHA] != {(21-CHA){1'b1}})
                                      : (pc_ca[11:CHA] != {(12-CHA){1'b1}});

   // One demand read in flight.
   reg          rq_v, rq_pois;
   reg  [63:0]  rq_va, rq_pa, rq_vm;                 // rq_vm = rq_va - CHB, latched with it
   reg  [1:0]   rq_lvl;                          // page size captured with the in-flight request
   initial begin rq_v=1'b0; rq_pois=1'b0; end
   wire need_pc = ~have0 & ~(rq_v & (rq_va == pc_ca));
   wire need_nx =  have0 & ~have1 & samepg & ~(rq_v & (rq_vm == pc_ca));
   wire         want    = need_pc | need_nx;
   wire [63:0]  want_va = need_pc ? pc_ca   : nx_ca;
   wire [63:0]  want_pa = need_pc ? pc_paca : (pc_paca + CHB);

   wire         ic_rd_req  = want & ~rq_v & ~ic_inv_busy & imem_xlate_ok;
   wire [63:0]  ic_rd_addr = {25'b0, want_va[38:0]};  // VIRTUAL: canonical Sv39 VA (sign ext masked)
   wire [63:0]  ic_rd_pa   = want_pa;                  // PA for the L2 fill
   wire         ic_tag     = 1'b0;                     // one request in flight

   // ---- serve: {next chunk, PC chunk} shifted to the PC's byte offset ----
   wire [2*HW*16-1:0] win_pair = {(n1_q ? nx_chunk : {(HW*16){1'b0}}), pc_chunk};
   wire [CHA-1:0]     pc_off   = imem_va[CHA-1:0];
   wire [2*HW*16-1:0] win_shf  = win_pair >> {pc_off, 3'b000};
   assign imem_data = win_shf[HW*16-1:0];

   localparam AVW = $clog2(HW+2);
   localparam [AVW-1:0] AV_HW = HW[AVW-1:0];
   wire [6:0] avail_b  = (n1_q ? 7'd32 : 7'd16) - {3'b0, pc_off};   // what the served pair holds (imem_ok qualifies)
   wire [6:0] avail_hw = avail_b >> 1;
   wire       freeze   = fi_stall | ic_inv_busy | imem_ctx_chg;
   assign imem_avail = (avail_hw >= HW) ? AV_HW : avail_hw[AVW-1:0];
   assign imem_ok    = ok_slot & ok_next & ~freeze;

   // ---- the pick for the NEXT cycle: the tags after this edge, the next PC's chunk by kind ----
   wire        fill    = ic_rd_valid & ~rq_pois;
   wire        inval   = fi_stall | ic_inv_busy | imem_ctx_chg;
   wire        f1      = fill & h0_0, f0 = fill & ~h0_0;           // the fill's slot (see the capture below)
   wire        c0_v_n  = ~inval & (c0_v | f0),  c1_v_n  = ~inval & (c1_v | f1);
   wire [63:0] c0_va_n = f0 ? rq_va : c0_va,   c1_va_n = f1 ? rq_va : c1_va;
   wire [63:0] c0_vm_n = f0 ? rq_vm : c0_vm,   c1_vm_n = f1 ? rq_vm : c1_vm;
   wire [63:0] tgt_ca  = {imem_adv_tgt[63:CHA], {CHA{1'b0}}};
   wire [63:0] red_ca  = {imem_adv_red[63:CHA], {CHA{1'b0}}};
   // per candidate chunk A: {the other slot holds A+CHB, c1 holds A (else serve c0)}
   `define AK_PICK(A) {((c1_v_n & (c1_va_n == (A))) ? (c0_v_n & (c0_vm_n == (A))) : (c1_v_n & (c1_vm_n == (A)))), (c1_v_n & (c1_va_n == (A)))}
   wire [1:0] pk_same = `AK_PICK(pc_ca), pk_next = `AK_PICK(nx_ca), pk_tgt = `AK_PICK(tgt_ca), pk_red = `AK_PICK(red_ca);
   wire [1:0] pk_n = (imem_adv_kind == AK_NEXT)  ? pk_next
                   : (imem_adv_kind == AK_TGT)   ? pk_tgt
                   : (imem_adv_kind == AK_REDIR) ? pk_red
                   :                               pk_same;      // HOLD and SAME: pc_q's own chunk
   always @(posedge clk)
      if (reset) begin sh_q <= 1'b0; n1_q <= 1'b0; end
      else       begin sh_q <= pk_n[0]; n1_q <= pk_n[1]; end
   // The pick is exact: a hint that is wrong while the chunk IS resident must be fixed by
   // the very next HOLD re-pick, never two cycles running.
   reg hint_wrong_q;  initial hint_wrong_q = 1'b0;
   wire hint_wrong = have0 & ~freeze & ~(ok_slot & ok_next);
   always @(posedge clk) begin
      hint_wrong_q <= ~reset & hint_wrong;
      if (!reset && hint_wrong)   // STRICT (probe): any wrong hint while the chunk is resident
         $fatal(1, "rv_soc_top: served-slot hint wrong two cycles running (sh_q=%b n1_q=%b h0=%b%b h1=%b%b pc_ca=%h)", sh_q, n1_q, h0_1, h0_0, h1_1, h1_0, pc_ca);
   end

   // ---- window fill + invalidation ----
   always @(posedge clk) begin
      if (reset) begin c0_v<=1'b0; c1_v<=1'b0; rq_v<=1'b0; rq_pois<=1'b0; end
      else begin
         // accept: latch the in-flight demand read
         if (ic_rd_req & ic_rd_ack) begin rq_v<=1'b1; rq_va<=want_va; rq_vm<=(need_pc ? pc_ca_m : pc_ca); rq_pa<=want_pa; rq_pois<=1'b0; rq_lvl<=immu_xlvl; end
         // response: capture into the slot NOT holding the PC's chunk (keep pc_chunk resident)
         if (ic_rd_valid) begin
            rq_v <= 1'b0;
            if (~rq_pois) begin
               if (h0_0) begin c1_va<=rq_va; c1_vm<=rq_vm; c1_d<=ic_rd_data; c1_lvl<=rq_lvl; c1_v<=1'b1; end
               else      begin c0_va<=rq_va; c0_vm<=rq_vm; c0_d<=ic_rd_data; c0_lvl<=rq_lvl; c0_v<=1'b1; end
            end
         end
         // invalidation LAST (wins a same-cycle capture): a mapping change or a flush in
         // progress drops the window and poisons the in-flight response (it is the old mapping).
         if (fi_stall | ic_inv_busy | imem_ctx_chg) begin  // fence.i flush OR a mapping change
            c0_v<=1'b0; c1_v<=1'b0;                           // the window holds old-mapping lines -> drop
            if (rq_v | (ic_rd_req & ic_rd_ack)) rq_pois<=1'b1;
         end
      end
   end
   // The minus-chunk tags are derived state; they must never drift from the VA they shadow.
   always @(posedge clk) if (!reset) begin
      if (c0_v && (c0_vm != c0_va - CHB)) $fatal(1, "rv_soc_top: c0_vm %h != c0_va %h - CHB", c0_vm, c0_va);
      if (c1_v && (c1_vm != c1_va - CHB)) $fatal(1, "rv_soc_top: c1_vm %h != c1_va %h - CHB", c1_vm, c1_va);
      if (rq_v && (rq_vm != rq_va - CHB)) $fatal(1, "rv_soc_top: rq_vm %h != rq_va %h - CHB", rq_vm, rq_va);
   end


   rv_cache #(.PAW(64), .PAW_SIG(39), .SIZE_KB(SIZE_KB), .RDW(HW*16), .WDW(64), .WRITABLE(0), .PREFETCH(1), .VIRT(1),
           .PERF_ID(0)) u_icache
     (.clk(clk), .reset(reset),
      .rd_req(ic_rd_req), .rd_addr(ic_rd_addr), .rd_pa(ic_rd_pa), .rd_data(ic_rd_data), .rd_valid(ic_rd_valid),
      .rd_resp_addr(ic_rd_resp_addr),
      // The tag names the request entry (two in flight, item 10e); the fetch buffer still
      // matches the answer to a SLOT by address (fb_al/fb_pa..fb_pa3), and asserts the two agree.
      .rd_ack(ic_rd_ack), .rd_tag({3'd0, ic_tag}), .rd_resp_tag(ic_rsp_tag),
      .rd_uncached(1'b0),
      .wr_req(1'b0), .wr_addr(64'd0), .wr_data(64'd0), .wr_mask(8'd0), .wr_ack(), .wr_acc(), .wr_cpl(), .wr_uncached(1'b0),
      .cbo_req(1'b0), .cbo_zero(1'b0), .cbo_keep(1'b0),
      .inv_req(ic_inv_req), .inv_clean(1'b0), .ep_bump(ic_ep_bump), .inv_busy(ic_inv_busy),
      .l2_req(ic_l2_req), .l2_we(ic_l2_we), .l2_addr(ic_l2_addr), .l2_wdata(ic_l2_wdata),
      .l2_rdata(ic_l2_rdata), .l2_ack(ic_l2_ack),
      .perf_access(ic_access), .perf_miss(ic_miss));

   // ---------------- PTW adapters: PTE word reads routed THROUGH the D$ ----------------
   // g=0 iPTW, 1 dPTW (loads, stores and atomics -- the in-order LSU has one memory op
   // in flight, so one data walker suffices). Each *_read is held (stable *_addr) until *_rvalid.
   // A walk reads its 8-byte PTE through the D$ read port (shared with the LSU via the dcr_*
   // arbiter below), so it always observes dirty PTEs -- no sfence.vma flush needed. pw_busy[g]
   // marks an outstanding read; the response is matched by exact byte address (the cache echoes
   // rd_resp_addr = the requested address). Same-address responses safely share data, so no
   // owner tag is needed.
   wire [1:0]   pw_read   = {dptw_read, ptw_read};
   wire [2*56-1:0] pw_addr = {dptw_addr, ptw_addr};
   reg  [1:0]   pw_busy;
   reg  [1:0]   pw_sent;      // ...and this walk's request has been accepted (see c_rd_want)
   reg  [1:0]   pw_rvalid;
   wire [1:0]   pw_rq = pw_busy & ~pw_sent;    // still asking for the port
   wire [1:0]   pw_ack;
   reg  [63:0]  pw_rdata [0:1];
   wire [1:0]   pw_match;
   wire [511:0] arb_rdata;
   genvar g;
   generate for (g=0; g<2; g=g+1) begin : ptw_adapt
      assign pw_match[g] = dc_rd_valid & pw_busy[g]
                         & (dc_rd_resp_tag == {(g ? 2'd2 : 2'd1), 2'b00});
      assign pw_ack[g] = dc_rd_ack & ~c_rd_req & (g ? (~pw_rq[0] & pw_rq[1]) : pw_rq[0]);
      always @(posedge clk) if (reset) begin pw_busy[g]<=1'b0; pw_sent[g]<=1'b0; pw_rvalid[g]<=1'b0; end
         else begin
            pw_rvalid[g] <= 1'b0;
            if (pw_read[g] & ~pw_busy[g] & ~pw_rvalid[g]) begin
               pw_busy[g] <= 1'b1; pw_sent[g] <= 1'b0;                          // new request
            end
            if (pw_ack[g]) pw_sent[g] <= 1'b1;
            if (pw_match[g]) begin
               pw_busy[g]  <= 1'b0;
               pw_sent[g]  <= 1'b0;
               pw_rvalid[g]<= 1'b1;
               pw_rdata[g] <= dc_rd_data;                  // the cache returns the 64-bit word
            end
         end
   end endgenerate
   assign ptw_rdata=pw_rdata[0];   assign ptw_rvalid=pw_rvalid[0];
   assign dptw_rdata=pw_rdata[1];  assign dptw_rvalid=pw_rvalid[1];

   // D$ read-port arbiter: the LSU (c_rd_req/dmem_raddr) and the 3 PTW walks share the single D$
   // read port. Fixed priority LSU > iPTW > dPTW. The cache samples rd_req only at its
   // S_IDLE and self-serializes; each client holds its request until its response matches, so a
   // purely combinational mux suffices (no accept handshake). No deadlock: a load needing ldPTW
   // is itself blocked on translation and not issuing c_rd_req, so the walk gets the port.
   // Selected on "still ASKING", not "still outstanding": once the cache has accepted a
   // walk's read, that walk must stop presenting it or a pipelined port serves it twice.
   wire            dc_rd_ack;
   assign lsu_rd_ack = dc_rd_ack & c_rd_req;
   assign dcr_req  = c_rd_req | (|pw_rq);
   assign dcr_addr = c_rd_req  ? dmem_raddr
                   : pw_rq[0]  ? {8'd0, pw_addr[0*56 +: 56]}
                   :             {8'd0, pw_addr[1*56 +: 56]};
   // ...and the tag that names the requester, selected by the SAME priority.
   wire [DRTW-1:0] dcr_tag = c_rd_req ? lsu_tag_req
                           : pw_rq[0] ? {2'd1, 2'b00}
                           :            {2'd2, 2'b00};

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
   rv_l2_arbiter #(.NREQ(NREQ), .AW(LAW), .DW(512)) u_arb
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
   localparam LLW    = $clog2(NLLINE);           // ...and the bits needed to index them
   // LBASE[AW-1:6], not LBASE >> 6: the shift is a 64-bit expression silently truncated
   // at the localparam width. LAW is AW-6, so the part-select is exactly LAW bits by
   // construction and cannot drift if AW changes.
   localparam [LAW-1:0] LLBASE = LBASE[AW-1:6];  // local SRAM base as a line address
   reg [511:0] lmem [0:NLLINE-1];
   // FPGA: bake the monitor image into the BRAM at elaboration (one 64-byte line per hex
   // line). Sim TBs instead pack dut.lmem directly via +monhex, so guard on the define.
`ifdef SOC_BOOT_HEX
   initial $readmemh(`SOC_BOOT_HEX, lmem);
`endif
   reg l_busy; reg [3:0] l_cnt; reg l_we_q; reg [LLW-1:0] l_li_q; reg [511:0] l_wd_q;
   reg [511:0] l_rdata; reg l_ack;
   wire [LAW-1:0] l_line = m_addr - LLBASE;       // local line index
   wire l_req = m_req & m_is_local;
   always @(posedge clk) begin
      l_ack <= 1'b0;
      if (reset) l_busy<=1'b0;
      else if (!l_busy && l_req) begin
         // lmem has NLLINE entries, so a LAW-bit (58) index at the array bracket is a silent
         // truncation -- an out-of-range line would WRAP onto a valid one. m_is_local bounds
         // l_line, so narrow explicitly and assert the precondition rather than trust it.
         if (|l_line[LAW-1:LLW])
            $fatal(1, "rv_soc_top: local SRAM line %h out of range (NLLINE=%0d)", l_line, NLLINE);
         l_busy<=1'b1; l_cnt<=4'd1; l_we_q<=m_we; l_li_q<=l_line[LLW-1:0]; l_wd_q<=m_wdata;
      end
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
