`default_nettype none
`ifndef PROBE_CLK_DIV8
 `define PROBE_CLK_DIV8 48        // 166.67 MHz -- THE shipping clock, not a suggestion
`endif
`define PROBE_CLK_HZ ((1_000_000_000 / `PROBE_CLK_DIV8) * 8)

// The I$ fetch window is SMOLRV64_HW halfwords (shipping HW=8 = 16 bytes, the even/odd chunk
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

// The SoC top: smolrv64_core + its I$/D$ (rv_cache) + rv_l2_arbiter merging all memory
// traffic onto ONE line memory port + the local boot SRAM. MMIO routing, CLINT/PLIC/UART,
// the virtio bridge and the PTW-through-cache adapters live here. (Forked from the retired
// sharded core's soc_top.v in 2026-08; two page-table walkers, iTLB and dTLB, since the
// LSU translates one op at a time; `retire` has a bit per commit port.)
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
`ifndef SMOLRV64_HW
 `define SMOLRV64_HW 8                 // fetch window halfwords (must match smolrv64_core.v)
`endif
`ifndef SMOLRV64_IW
 `define SMOLRV64_IW 3                 // pipeline width (must match smolrv64_core.v)
`endif
module rv_soc_top #(
   parameter HW=`SMOLRV64_HW, IW=`SMOLRV64_IW, PCW=64, SEQW=8,   // fetch window halfwords / pipeline width
   parameter [63:0] BASE     = 64'h8000_0000,   // DDR
   parameter        RAM_LG2  = 21,              // 2 MiB DDR
   parameter [63:0] LBASE    = 64'h7000_0000,   // on-chip local SRAM (boot/monitor) -- MEM_BASEADDR on the FPGA
   parameter        LRAM_LG2 = 18,              // 256 KiB local SRAM
   parameter [63:0] RESET_PC = BASE,            // tests link @DDR; the platform boots @LBASE
   parameter        SIZE_KB  = 128,             // the I$; like the D$, only 166.67 MHz may shrink it
   parameter        DC_KB    = 128              // the D$ (docs/PLAN-2026-09-25-dcache-vhpr.md); only
                                                 // 166.67 MHz may shrink it
) (
   input  wire             clk,
   input  wire             reset,
   // observation for a TB (retire + the store stream, to watch tohost)
   output wire [IW-1:0]    retire,             // the core's commit ports that retired this cycle
   output wire             dmem_wen,
   output wire [63:0]      dmem_waddr,
   output wire [63:0]      dmem_wdata,
   output wire [7:0]       dmem_wmask,
   // external DDR memory port (cache-backed DRAM @ BASE): the sim TB's model, or the FPGA's
   // crossing into the DDR4 controller. Tagged, several transactions outstanding, three
   // valid/ready channels (rv_mem_arbiter.v; docs/PLAN-2026-09-25-dcache-vhpr.md).
   output wire             ddr_q_valid,  // request
   input  wire             ddr_q_ready,
   output wire [4:0]       ddr_q_id,
   output wire             ddr_q_we,
   output wire [57:0]      ddr_q_addr,   // line address PA[63:6]
   output wire [63:0]      ddr_q_wmask,  // per byte of the line
   output wire [511:0]     ddr_q_wdata,
   input  wire             ddr_r_valid,  // read data: four 128-bit beats per line
   output wire             ddr_r_ready,
   input  wire [4:0]       ddr_r_id,
   input  wire [1:0]       ddr_r_beat,
   input  wire             ddr_r_last,
   input  wire [127:0]     ddr_r_data,
   input  wire             ddr_w_valid,  // write done (the DRAM's write response)
   output wire             ddr_w_ready,
   input  wire [4:0]       ddr_w_id,
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
   output wire [14:0]      virtio_addr,   // offset from 0x1000_0000; [14:12] is the page: 2 blk, 3 net, 4 keyboard, 5 video
   output wire             virtio_read,
   output wire             virtio_write,
   output wire [31:0]      virtio_wdata,
   output wire [3:0]       virtio_be,
   input  wire [31:0]      virtio_rdata,
   input  wire             virtio_rvalid,   // virtio read-data valid (req/rsp; tolerates CDC-bridge latency)
   input  wire             virtio_irq,
   input  wire             virtio_net_irq,   // PLIC source 12 (ubuntu-nfs.dts virtio@10003000)
   input  wire             dma_wr,           // a device wrote memory (a pulse, in this clock): fence.i clears the I$
   input  wire             virtio_kbd_irq,   // PLIC source 4 (virtio_mmio@10004000, as in simmerv)
   output wire [17:0]      irq_dbg,         // interrupt-path debug for the wrapper ILA (probe_clk)
   // Cache data-array integrity (meaningful only in a -DCACHE_PARITY build; tied to 0
   // otherwise). cache_par_err is the ILA_PARITY TRIGGER: it pulses in the cycle a cache
   // data array returns a word whose parity does not match what was stored -- the moment of
   // corruption, rather than the kernel Oops millions of cycles downstream, which is the
   // only evidence the board has offered so far and is far beyond any pre-trigger depth.
   output wire [1:0]       cache_par_err,   // {I$, D$} 1-cycle error pulse
   output wire [63:0]      cache_par_dbg,   // {sticky, bank, addr} of the first failure
   output wire [199:0]     core_dbg         // the core's wait state for the board's wedge ILA (ILA_MEM)
);
   localparam SIZE = 1<<RAM_LG2;
   localparam AW   = 64;
   // The architectural physical-address width (64 GiB): the core faults any PA beyond its
   // instance's DRAM, so the D$ tags exactly PABITS bits (smolrv64_core, THE PHYSICAL-ADDRESS CAP).
   localparam integer PABITS = 36;
   localparam integer LQ_IB  = 3;    // the core's load-queue index width: the D$ read tag carries it
   localparam LAW  = AW-6;                  // line address width = 58

   // ---------------- core <-> caches nets ----------------
   wire [PCW-1:0]      imem_addr;
   wire                imem_ctx_chg;
   wire                imem_hold, imem_cancel;   // the I$'s miss hold and its cancel (smolrv64_core)
   wire                ic_rd_req, ic_rd_ack, ic_rd_valid, ic_inv_busy, fi_stall;
   wire [63:0]         ic_rd_addr, ic_rd_pa;
   wire [9:0]          ic_tag, ic_rsp_tag;
   wire [127:0]        ic_rd_data;
   wire [63:0]         imem_satp_q;  wire [1:0] imem_priv_q;
   wire                fe_redirect;
   wire [63:0]         dmem_raddr;
   wire                dmem_ren;
   wire                dmem_runcached, dmem_wuncached;   // Svpbmt: NC/IO read/write attribute
   wire                dmem_cbo, dmem_cbo_zero, dmem_cbo_keep;  // Zicbom/Zicboz cache maintenance
   wire [63:0]         dmem_rdata;
   wire [63:0]         dmem_wabase;   // store base PA, unmuxed by the straddle beat
   wire                dmem_rvalid, dmem_wready, dmem_waccept, dmem_idle, ifence;
   wire                dc_wr_room;
   wire                dmem_rfast, dmem_rvalid_c, dmem_rbusy;      // the tagged fast load path (C4a)
   wire [LQ_IB-1:0]    dmem_rtag, dmem_rtag_resp;
   wire [63:0]         dmem_rdata_c;
   wire [55:0]         ptw_addr, dptw_addr;
   wire                ptw_read, dptw_read;
   wire [63:0]         ptw_rdata, dptw_rdata;
   wire                ptw_rvalid, dptw_rvalid;
   wire                redirect;  wire [PCW-1:0] redirect_target;

   smolrv64_core #(.HW(HW), .IW(IW), .PCW(PCW), .SEQW(SEQW), .RESET_PC(RESET_PC), .LBASE(LBASE), .LRAM_LG2(LRAM_LG2),
               .PABITS(PABITS), .LQ_IB(LQ_IB)) core
     (.clk(clk), .reset(reset),
      .imem_addr(imem_addr), .imem_ctx_chg(imem_ctx_chg), .imem_hold(imem_hold), .imem_cancel(imem_cancel), .hw_ip(hw_ip), .mtime(clint_mtime),
      .ic_busy(fi_stall | ic_inv_busy), .ic_req(ic_rd_req), .ic_va(ic_rd_addr), .ic_pa(ic_rd_pa),
      .ic_tag(ic_tag), .ic_ack(ic_rd_ack), .ic_valid(ic_rd_valid), .ic_data(ic_rd_data), .ic_rtag(ic_rsp_tag),
      .imem_satp_q(imem_satp_q), .imem_priv_q(imem_priv_q),
      .fe_redirect(fe_redirect),
      .hpm_dc_access(dc_access), .hpm_dc_miss(dc_miss), .hpm_ic_access(ic_access), .hpm_ic_miss(ic_miss),
      .dmem_raddr(dmem_raddr), .dmem_ren(dmem_ren), .dmem_runcached(dmem_runcached),
      .dmem_rdata(dmem_rdata), .dmem_rvalid(dmem_rvalid),
      .dmem_rfast(dmem_rfast), .dmem_rtag(dmem_rtag), .dmem_rvalid_c(dmem_rvalid_c),
      .dmem_rtag_resp(dmem_rtag_resp), .dmem_rdata_c(dmem_rdata_c), .dmem_rbusy(dmem_rbusy),
      .dmem_wen(dmem_wen), .dmem_waddr(dmem_waddr), .dmem_wabase(dmem_wabase),
      .dmem_wdata(dmem_wdata), .dmem_wmask(dmem_wmask),
      .dmem_wuncached(dmem_wuncached),
      .dmem_cbo(dmem_cbo), .dmem_cbo_zero(dmem_cbo_zero), .dmem_cbo_keep(dmem_cbo_keep),
      .dmem_wready(dmem_wready), .dmem_waccept(dmem_waccept), .dmem_wroom(dc_wr_room), .dmem_idle(dmem_idle), .ifence(ifence),
      .ptw_addr(ptw_addr), .ptw_read(ptw_read), .ptw_rdata(ptw_rdata), .ptw_rvalid(ptw_rvalid),
      .dptw_addr(dptw_addr), .dptw_read(dptw_read), .dptw_rdata(dptw_rdata), .dptw_rvalid(dptw_rvalid),
      .retire(retire), .retire_pc(), .retire_insn(),
      .redirect(redirect), .redirect_target(redirect_target), .lsu_err(lsu_err), .fe_err(fe_err), .core_dbg(core_dbg_c));

   // ---------------- MMIO device routing (CLINT + UART bypass the D$, non-cacheable) ----------------
   localparam [63:0] CLINT_BASE = 64'h0200_0000, UART_BASE = 64'h1000_0000, PLIC_BASE = 64'h0C00_0000;
   // DDR latency HPM window (read-only counters; any write clears). NOT in the DTB -- read it
   // from a bare-metal tool / the monitor; the kernel never touches it.
   localparam [63:0] HPM_BASE   = 64'h1800_0000;
   // The board's ui_clk device pages, 24 KiB: virtio-blk 0x1000_2000, virtio-net 0x1000_3000,
   // virtio keyboard 0x1000_4000, VGA scanout 0x1000_5000 (pages 6 and 7 read 0). Everything
   // behind the probe_clk<->ui_clk bridge, so one window, one req/rsp path. The page number is
   // passed through as is (virtio_addr[14:12]), so the board's page select is the identity.
   localparam [63:0] VIRTIO_BASE = 64'h1000_0000;
   localparam [63:0] BUILDID_BASE = 64'h1000_F000;                // build-id (SMOL/stamp/commit/dirty), probe-core-readable
   // Integrity log, 256 B read-only -- the design's own invariants, latched so software can
   // read them (see the INTEGRITY LOG block below).  Like HPM_BASE it is deliberately NOT in
   // the DTB: the kernel must never touch it.  Readers are the ROM monitor's banner
   // (R1000E008 = the sticky vector) and a /dev/mem read from Linux after a crash.  The
   // window and its name are inherited from the deleted fetch-buffer diagnostic.
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
   wire is_virtio_r = (dmem_raddr & ~64'h7fff)    == VIRTIO_BASE && dmem_raddr[14:13] != 2'b00;
   wire is_buildid_r = (dmem_raddr & ~64'hff)     == BUILDID_BASE;  // 256 B window (read-only)
   wire is_fbdiag_r  = (dmem_raddr & ~64'hff)     == FBDIAG_BASE;   // 256 B window (read-only)
   wire [63:0] fbdiag_rdata;   // driven by the capture block down by the fetch buffer
   wire is_dev_r   = is_clint_r | is_uart_r | is_plic_r | is_hpm_r | is_virtio_r | is_buildid_r | is_fbdiag_r;
   wire is_clint_w = (dmem_wabase & ~64'hffff)     == CLINT_BASE;
   wire is_uart_w  = (dmem_wabase & ~64'hf)        == UART_BASE;
   wire is_plic_w  = (dmem_wabase & ~64'h3ff_ffff) == PLIC_BASE;
   wire is_hpm_w   = (dmem_wabase & ~64'hff)        == HPM_BASE;
   wire is_virtio_w = (dmem_wabase & ~64'h7fff)    == VIRTIO_BASE && dmem_wabase[14:13] != 2'b00;
   wire is_dev_w   = is_clint_w | is_uart_w | is_plic_w | is_hpm_w | is_virtio_w;
   // virtio-mmio register access: 32-bit. The probe LSU bus is byte-addressed and
   // RIGHT-ALIGNED -- it presents/consumes "8 bytes @ mem_*addr" with the addressed
   // bytes in the LOW lane and the byte mask low-aligned (store drain: sb_data=raw,
   // dr_mask=low-nbytes). So a 32b reg always sits in [31:0] regardless of its offset;
   // do NOT pick a lane by addr[2] (that picks the empty high lane for 0x014/0x038/...).
   assign virtio_addr  = dev_addr[14:0];
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
      .src({51'd0, virtio_net_irq, virtio_irq, uart_irq, 5'd0, virtio_kbd_irq, 4'd0}), .meip(plic_meip), .seip(plic_seip),
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
   // (the D$ half is tied off: rv_dcache carries no parity taps; its invariants are in the log)
   assign cache_par_err = {u_icache.par_err | u_icache.adr_err, 1'b0};
   assign cache_par_dbg = {1'b0, u_icache.par_sticky,
                           1'b0, u_icache.adr_sticky,
                           2'd0, u_icache.par_bank,
                           {(64-8-2*16){1'b0}},
                           16'd0, u_icache.par_addr16};
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
`ifdef SMOLRV64_IRQ_STIM
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

   // ---------------- the D$ + read/write adapters, device-muxed ----------------
   reg          c_rd_pend;
   wire [63:0]  dc_rd_data;  wire dc_rd_valid, dc_wr_acc, dc_wr_cpl;  wire [63:0] dc_rd_resp_addr;
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
   localparam DRTW = LQ_IB + 2;
   // The tag must be CONSTANT for the request's whole lifetime. lsu_gen flips at the clock
   // edge on dmem_ren, so during the request cycle itself the cache would capture the
   // PRE-flip value while every later compare used the POST-flip one -- legitimate responses
   // mismatch, get discarded, and the load re-issues. Present the value lsu_gen is ABOUT to
   // take; compare against the registered one, which equals it from the next cycle on (a
   // response cannot arrive in the capture cycle -- rd_valid is registered).
   // Tag = {client, ...}: the LSU's FAST reads carry the load-queue index the LSU allocated
   // ({2'b00, idx}), its FSM's slow reads one fixed tag of their own (TAG_SLOW), and the two
   // walkers {01,0} and {10,0}. A response is claimed by the tag its requester allocated
   // (rule B1); the LSU's generation toggle that stood here could name one read in flight and
   // is gone. A slow read is one at a time by construction (the FSM parks in S_LD for it).
   localparam [DRTW-1:0] TAG_SLOW = {2'b11, {LQ_IB{1'b0}}};
   wire [DRTW-1:0] lsu_tag_req = dmem_rfast ? {2'b00, dmem_rtag} : TAG_SLOW;   // -> the cache
   wire [DRTW-1:0] dc_rd_resp_tag;
   wire         dc_rv_ok   = dc_rd_valid & (dc_rd_resp_tag == TAG_SLOW);       // the FSM's
   wire         dc_rv_fast = dc_rd_valid & (dc_rd_resp_tag[DRTW-1 -: 2] == 2'b00);     // a queued load's
   assign       dmem_rvalid_c  = dc_rv_fast;
   assign       dmem_rtag_resp = dc_rd_resp_tag[LQ_IB-1:0];
   assign       dmem_rdata_c   = dc_rd_data;
   assign       dmem_rbusy     = c_rd_want;
`ifdef LSUDBG
   reg [63:0] soc_dbg_line; initial if (!$value$plusargs("dbg_line=%h", soc_dbg_line)) soc_dbg_line = 64'hFFFF_FFFF_FFFF_FFFF;
   always @(posedge clk) if (!reset) begin
      if (dcr_req && (dcr_addr[55:6] == soc_dbg_line[55:6]))
         $display("[soc t=%0t DOOR req addr=%h tag=%h ack=%b want=%b ren=%b fast=%b]", $time, dcr_addr, dcr_tag, dc_rd_ack, c_rd_want, dmem_ren, dmem_rfast);
      if (dc_rd_valid && (dc_rd_resp_addr[55:6] == soc_dbg_line[55:6]))
         $display("[soc t=%0t RESP addr=%h tag=%h data=%h]", $time, dc_rd_resp_addr, dc_rd_resp_tag, dc_rd_data);
   end
`endif
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
      // A CACHE read only: a device read never asks the cache and is never acked by it, so a
      // want set on it would stand until the next cache read cleared it; the LSU reads the
      // bit as "the read port is busy" and a stuck want stops every load (UART LSR, 2026-09-04).
      else if (dmem_ren & ~is_dev_r & ~lsu_rd_ack) c_rd_want<=1'b1;
      else if (lsu_rd_ack)                         c_rd_want<=1'b0;
   // ...for the FSM's slow reads only: a fast read's response is a one-cycle strobe the LSU
   // claims by tag, and it must not set a pending bit no slow response will ever clear.
   wire         dmem_ren_slow = dmem_ren & ~dmem_rfast;
   // ...and c_rd_pend keeps its old meaning and its old users (c_st_ok below reads it as
   // "a response is outstanding"), so it is still cleared by the response, not the accept.
   always @(posedge clk) if (reset) c_rd_pend<=1'b0;
      else if (dmem_ren_slow) c_rd_pend<=1'b1; else if (raw_rvalid) c_rd_pend<=1'b0;
   // A slow response nobody asked for is a read that changed its tag while it waited for the
   // accept: the FSM is idle and would drop it. And the request must not move while it waits:
   // the cache samples address and tag on the accept, not when the LSU first presented them.
   always @(posedge clk) if (!reset & dc_rv_ok & ~c_rd_pend)
      $fatal(1, "rv_soc_top: D$ slow response with no slow read outstanding");
   reg [DRTW-1:0] want_tag;  reg [63:0] want_addr;
   always @(posedge clk) if (dmem_ren) begin want_tag <= lsu_tag_req; want_addr <= dmem_raddr; end
   always @(posedge clk)
      if (!reset & c_rd_want & ((lsu_tag_req != want_tag) | (dmem_raddr != want_addr)))
         $fatal(1, "rv_soc_top: D$ read changed under its own accept wait: tag %h->%h addr %h->%h",
                want_tag, lsu_tag_req, want_addr, dmem_raddr);
   reg          c_rdv_st;  reg [63:0] c_rdd_st;
   always @(posedge clk) if (reset) c_rdv_st<=1'b0;
      else if (dmem_ren_slow) c_rdv_st<=1'b0;
      else if (raw_rvalid) begin c_rdv_st<=1'b1; c_rdd_st<=raw_rdata; end
   wire         c_st_ok = c_rdv_st & ~c_rd_pend & ~dmem_ren_slow;
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
   // dc_wr_cpl, NOT a per-store ack: the LSU leaves a plain store to memory when the D$ takes
   // it (dc_wr_room), so a completion for it would be nobody's -- and it would land while the
   // LSU waits on a device or virtio store started since, completing THAT one early. The D$
   // raises wr_cpl only for the writes whose requester waits (CBO, uncached), which the LSU
   // serializes, so the three terms stay exclusive (asserted).
   assign       dmem_wready = vio_wack | dev_wack | dc_wr_cpl;
   // The DEVICE ACCEPT, for the LSU's plain store to a device: done at the device's own ack.
   // A device store carries no NC bit when there are no page tables (bare M-mode has no PBMT),
   // so the LSU classes it as plain, and without the ack here the device path would re-execute
   // the write while the LSU waited. A plain store to memory ends at the D$'s dc_wr_room
   // instead: the LSU knows which of the two it presented (its registered mem_q), so no address
   // decode reaches its state. The two terms are exclusive: the LSU has one device write in
   // flight.
   assign       dmem_waccept = dev_wack | vio_wack;
   always @(posedge clk)
      if (!reset & ((vio_wack & dev_wack) | (vio_wack & dc_wr_cpl) | (dev_wack & dc_wr_cpl)))
         $fatal(1, "rv_soc_top: two write acks at once (vio=%b dev=%b dc=%b) -- wready ambiguous",
                vio_wack, dev_wack, dc_wr_cpl);

   // The D$ (rv_dcache, docs/PLAN-2026-09-25-dcache-vhpr.md) is write-back and non-blocking: a
   // miss parks in its waiter table and hits behind it answer first, by tag. It is a client of
   // rv_mem_arbiter in its own right, up to 8 fills, 2 write-backs and an NC access in flight.
   // VIRT=0: the core translates before it asks, so every request is presented by PA. PTW reads
   // are routed THROUGH the D$ (the dcr_* read-port arbiter below), so a page-table walk always
   // sees dirty PTEs -- the D$ is the coherency point. sfence.vma therefore needs NO D$ flush
   // (just a TLB flush); only fence.i still clean-flushes (the I$ reads memory directly) -- see
   // the df_* FSM below.
   wire        dcr_req;  wire [63:0] dcr_addr;     // muxed D$ read port (LSU + 3 PTW), assigned below
   wire dc_inv_req, dc_inv_busy;
   wire dc_ic_req, dc_ic_ack, dc_ic_dty;          // the I$'s line clean beside each fill
   // Zihpm cache-event taps (D$/I$ line-lookup + miss pulses) -> core hpm_ev.
   wire dc_access, dc_miss, ic_access, ic_miss;
   // the D$'s memory-port client wires (client 0 of the arbiter below)
   localparam integer MC = 2, MCB = 1, MSW = 4, MIDW = MCB + MSW;
   wire [MC-1:0]       mc_q_valid, mc_q_ready, mc_q_we, mc_r_valid, mc_w_valid;
   wire [MC*MSW-1:0]   mc_q_slot;
   wire [MC*LAW-1:0]   mc_q_addr;
   wire [MC*64-1:0]    mc_q_wmask;
   wire [MC*512-1:0]   mc_q_wdata;
   wire [MSW-1:0]      mc_r_slot, mc_w_slot;
   wire [1:0]          mc_r_beat;  wire mc_r_last;  wire [127:0] mc_r_data;
   rv_dcache #(.SIZE_KB(DC_KB), .RTW(DRTW), .PABITS(PABITS), .SW(MSW), .VIRT(0)) u_dcache
     (.clk(clk), .reset(reset),
      .rd_req(dcr_req), .rd_va(dcr_addr), .rd_pa(dcr_addr), .rd_tag(dcr_tag), .rd_ack(dc_rd_ack),
      // Svpbmt: only a LSU load read can be NC (PTW reads share dcr but are always cacheable -> 0
      // when c_rd_req is low). The store's NC bit qualifies the write port.
      .rd_phys(1'b1), .rd_nc(c_rd_req & dmem_runcached),
      .rd_valid(dc_rd_valid), .rd_data(dc_rd_data), .rd_resp_tag(dc_rd_resp_tag), .rd_resp_addr(dc_rd_resp_addr),
      // ~dc_wr_cpl: a CBO or uncached write holds its request through the cycle its completion
      // is registered, and the D$ is idle again in that cycle -- without this it is taken twice.
      .wr_req(dmem_wen & ~dc_wr_cpl & ~is_dev_w), .wr_va(dmem_waddr), .wr_pa(dmem_waddr),
      .wr_data(dmem_wdata), .wr_mask(dmem_wmask), .wr_nc(dmem_wuncached),
      .cbo_req(dmem_cbo & ~dc_wr_cpl & ~is_dev_w), .cbo_zero(dmem_cbo_zero), .cbo_keep(dmem_cbo_keep),
      .wr_room(dc_wr_room), .wr_acc(dc_wr_acc), .wr_cpl(dc_wr_cpl),
      .ic_req(dc_ic_req), .ic_pa({ic_l2_addr, 6'd0}), .ic_ack(dc_ic_ack), .ic_dty(dc_ic_dty),
      .ep_bump(1'b0), .inv_req(dc_inv_req), .inv_busy(dc_inv_busy),
      .cq_valid(mc_q_valid[0]), .cq_ready(mc_q_ready[0]), .cq_slot(mc_q_slot[0*MSW +: MSW]),
      .cq_we(mc_q_we[0]), .cq_addr(mc_q_addr[0*LAW +: LAW]), .cq_wmask(mc_q_wmask[0*64 +: 64]),
      .cq_wdata(mc_q_wdata[0*512 +: 512]),
      .cr_valid(mc_r_valid[0]), .cr_slot(mc_r_slot), .cr_beat(mc_r_beat), .cr_last(mc_r_last),
      .cr_data(mc_r_data), .cw_valid(mc_w_valid[0]), .cw_slot(mc_w_slot),
      .perf_access(dc_access), .perf_miss(dc_miss), .err(dc_err));

   // fence.i changes code, and the I$ reads memory directly, so an I$ fill must never read a line
   // the D$ holds dirty: before each line read goes to memory the D$ cleans that line by PA (ic_cl,
   // below). So fence.i needs no D$ writeback -- it drains the core and invalidates the I$ (fi).
   // A MAPPING change (satp/sfence) does not change code -- it only stales virtual tags -- so it
   // advances the I$ EPOCH (ic_ep_bump): the I$ keeps its lines and reconciles them by physical tag
   // on refetch (docs/VHPR.md). PTW reads go through the D$, so a walk sees dirty PTEs and
   // sfence.vma flushes only the TLBs.
   wire ic_ep_bump = imem_ctx_chg;
   assign dc_inv_req = 1'b0;

   // ---------------- the VHPR I$ + the fence.i FSM ----------------
   // The I$ (rv_icache) is virtually indexed and tagged and serves the core's fetch ring
   // (smolrv64_fring) one 16-byte pair per request. Permissions and faults stay the iMMU's, so only a
   // mapping change (imem_ctx_chg, an epoch advance) or fence.i (an invalidation) touches it.
   wire [63:0]      ic_rd_resp_addr;
   wire             ic_l2_req, ic_l2_we;   wire [LAW-1:0] ic_l2_addr;   wire [511:0] ic_l2_wdata;
   wire [511:0]     ic_l2_rdata;   wire ic_l2_ack;

   // ================= INTEGRITY LOG (rv_errlog) ======================================
   // docs/rtl-rules.md A1 makes every invariant always-on -- in simulation. On hardware
   // $fatal is a no-op and synthesis deletes the condition, so the bitstream that ships
   // enforces none of them, and the longest simulation this project can run (~1.5e9
   // cycles) is two and a half orders of magnitude short of one Geekbench run (~3e11).
   // A fault too rare for any gate we own still kills the board in half an hour -- as a
   // wild jump with nothing attached to it, which is how C4a step 2 ended.
   //
   // So the conditions are latched and published. Each unit registers its own error
   // vector (nothing combinational crosses a hierarchy boundary for this) and the log
   // keeps a sticky bit per invariant plus the index and cycle stamp of the FIRST one to
   // fire. Read-only over MMIO: the monitor prints it in its banner, and Linux userland
   // reads it through /dev/mem after a crash. Either answer is worth having -- "D$
   // invariant 6, 4 billion cycles before the Oops" locates the bug, and "nothing fired"
   // exonerates the whole memory backend in one read.
   //
   // BIT ASSIGNMENT -- fixed, so an index read off a dead board keeps its meaning:
   //   [15: 0] D$   rv_cache err[] (see the INTEGRITY LOG block in rv_cache.v)
   //   [31:16] I$   the same cache, the same numbering
   //   [47:32] LSU  smolrv64_lsu err[]
   //   [63:48] the frontend: smolrv64_frontend fe_err (the fetch ring, the predictor, fetch)
   // The units are ordered so that on a simultaneous violation first_idx names the one
   // closest to the data.
   wire [15:0] dc_err, ic_err, lsu_err, fe_err;
   wire [127:0] core_dbg_c;
   wire [63:0] err_sticky;
   // core_dbg: [199:136] the integrity log's sticky vector, [133:128] the device path (virtio
   // response, virtio write/read pending, the fixed devices' read response, a store, a device
   // read), [127:0] the core's (see smolrv64_core)
   assign core_dbg = {err_sticky, 2'd0, virtio_rvalid, vio_wpending, vio_pending, dev_rvalid,
                      dmem_wen, dmem_ren & is_dev_r, core_dbg_c};
   wire [7:0]  err_first_idx;
   wire [47:0] err_first_cyc;
   rv_errlog #(.N(64), .CW(48)) u_errlog
     (.clk(clk), .reset(reset),
      .err({fe_err, lsu_err, ic_err, dc_err}),
      .sticky(err_sticky), .first_idx(err_first_idx), .first_cyc(err_first_cyc));

   // The readout, in the window the deleted fetch-buffer diagnostic used to occupy: it is
   // already decoded, already read-only, already in is_dev_r, and the ROM monitor already
   // knows the address. Reusing it adds NO comparator to the dmem_raddr -> is_dev_r cone,
   // which is on the LSU's critical path (see the decode notes above) -- a recorder that
   // costs the design timing would not survive its first build.
   //   +0x00  {version, "ERRL"}  -- magic, so a reader can tell the window is populated
   //   +0x08  sticky[63:0]
   //   +0x10  {8'd0, first_idx[7:0], first_cyc[47:0]}
   // 64-bit reads. The LSU right-aligns a narrower load, so a 32-bit read at +0x00 still
   // gets the magic; every other register is one whole 64-bit word by design.
   assign fbdiag_rdata = (dmem_raddr[4:3] == 2'd0) ? 64'h0000_0001_4552_524c
                       : (dmem_raddr[4:3] == 2'd1) ? err_sticky
                       : (dmem_raddr[4:3] == 2'd2) ? {8'd0, err_first_idx, err_first_cyc}
                       : 64'd0;
   // The fetch-buffer invariant's self-reset went with the buffer. An integrity fault does
   // NOT reset the machine: resetting into the monitor would destroy the run that produced
   // the evidence, and the cycle stamp already says when it happened.
   assign fbdiag_reset_req = 1'b0;

   // THE I$ IS COHERENT WITH THE CORE'S STORES: every write the core presents probes its line in
   // the I$ (ic_pb, a cycle later), which invalidates it wherever it lives, and every I$ line read
   // is cleaned in the D$ beside the fill (the coherent fill, above). So fence.i only drains: every
   // older store handed to the D$ (dmem_idle) and its probe applied (~ic_pb_v) -- the fetch ring and
   // predictions follow its redirect. A device write since the last fence.i clears the I$ as well:
   // device writes do not probe.
   reg        ic_pb_v;  reg [63:0] ic_pb_pa;
   always @(posedge clk) begin ic_pb_v <= ~reset & dmem_wen;  ic_pb_pa <= dmem_waddr; end
   reg        dma_dirty;                           // a device wrote memory since the I$ was last cleared
   localparam FI_IDLE=0, FI_DRAIN=1, FI_INV=2, FI_WAIT=3;
   reg [1:0] fi;  assign fi_stall = (fi != FI_IDLE);
   reg fi_inv;
   always @(posedge clk) if (reset) begin fi<=FI_IDLE; fi_inv<=1'b0; dma_dirty<=1'b0; end
      else begin
         fi_inv <= 1'b0;
         if (dma_wr) dma_dirty <= 1'b1;
         case (fi)
           FI_IDLE:  if (ifence) fi<=FI_DRAIN;
           FI_DRAIN: if (dmem_idle & ~ic_pb_v) begin
                        if (dma_dirty) begin fi_inv<=1'b1; fi<=FI_INV; if (~dma_wr) dma_dirty<=1'b0; end
                        else fi<=FI_IDLE;
                     end
           FI_INV:   fi<=FI_WAIT;
           FI_WAIT:  if (!ic_inv_busy) fi<=FI_IDLE;
           default:  $fatal(1, "rv_soc_top: fence.i state %0d", fi);
         endcase
      end
   wire ic_inv_req = fi_inv;   // fence.i AND mapping changes go through fi (after the D$ clean-flush)

   // The read-only VHPR I$ (core/rv_icache.v): a pair per cycle, hit on the virtual tag alone.
   rv_icache #(.SIZE_KB(SIZE_KB), .HW(8), .RTW(10)) u_icache
     (.clk(clk), .reset(reset),
      .rd_req(ic_rd_req), .rd_addr(ic_rd_addr), .rd_pa(ic_rd_pa), .rd_tag(ic_tag),
      .rd_ack(ic_rd_ack), .rd_data(ic_rd_data), .rd_valid(ic_rd_valid),
      .rd_resp_addr(ic_rd_resp_addr), .rd_resp_tag(ic_rsp_tag),
      .inv_req(ic_inv_req), .ep_bump(ic_ep_bump), .inv_busy(ic_inv_busy),
      .pb_v(ic_pb_v), .pb_pa(ic_pb_pa), .fill_hold(imem_hold), .fill_cancel(imem_cancel),
      .l2_req(ic_l2_req), .l2_we(ic_l2_we), .l2_addr(ic_l2_addr), .l2_wdata(ic_l2_wdata),
      .l2_rdata(ic_l2_rdata), .l2_ack(ic_l2_ack),
      .perf_access(ic_access), .perf_miss(ic_miss), .err(ic_err));

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
   genvar g;
   generate for (g=0; g<2; g=g+1) begin : ptw_adapt
      assign pw_match[g] = dc_rd_valid & pw_busy[g]
                         & (dc_rd_resp_tag == {(g ? 2'd2 : 2'd1), {LQ_IB{1'b0}}});
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
                           : pw_rq[0] ? {2'd1, {LQ_IB{1'b0}}}
                           :            {2'd2, {LQ_IB{1'b0}}};

   // ---------------- the memory port: the caches as clients of rv_mem_arbiter ----------------
   // PTW reads go through the D$ (dcr_* above), and a D$ miss on a PTE fills like any other line.
   // The D$ is client 0 with its own slots (above); the I$ keeps its single-outstanding line
   // handshake behind rv_mem_line_client as client 1 (a read of the D$ wins a tie).
   // THE I$ FILLS COHERENTLY, AT NO COST IN THE COMMON CASE. The I$ pulses l2_req, holds l2_addr
   // until l2_ack and has one read outstanding. The read goes to memory at once, and the D$ is
   // asked in parallel to clean the line (dc_ic_req, held until dc_ic_ack; it queues behind every
   // store taken before it). The data is used only if the D$ has answered by then that the line
   // was clean (~dc_ic_dty); otherwise -- a write-back, or data faster than the answer -- the data
   // is dropped and the line read again once the D$ has answered, when memory is current.
   reg  ic_p, ic_ak, ic_cl, ic_rd;                // a read owed; the D$ answered; answered clean; a read in flight
   wire ic_mack;                                  // the memory's answer to the read in flight
   wire ic_ok   = (ic_ak & ic_cl) | (dc_ic_ack & ~dc_ic_dty);
   wire ic_use  = ic_mack & ic_ok;                // the data is current: the I$'s l2_ack
   // read again when the data was dropped and the answer is in -- a cycle after the dropped read's
   // answer at the earliest: the line client's l2_ack ends the last read at that edge
   wire ic_rr   = ic_p & ~ic_rd & (ic_ak | dc_ic_ack);
   always @(posedge clk)
      if (reset) begin ic_p <= 1'b0;  ic_ak <= 1'b0;  ic_cl <= 1'b0;  ic_rd <= 1'b0; end
      else begin
         if (ic_l2_req) begin ic_p <= 1'b1;  ic_ak <= 1'b0;  ic_rd <= 1'b1; end
         else begin
            if (dc_ic_ack) begin ic_ak <= 1'b1;  ic_cl <= ~dc_ic_dty; end
            if (ic_use) ic_p <= 1'b0;
            if (ic_mack) ic_rd <= 1'b0;
            if (ic_rr) begin ic_rd <= 1'b1;  ic_cl <= 1'b1; end   // after the answer memory is current
         end
      end
   assign dc_ic_req = ic_p & ~ic_ak;
   assign ic_l2_ack = ic_use;
   always @(posedge clk) if (!reset & ic_l2_req & ic_p)
      $fatal(1, "rv_soc_top: an I$ line read asked while the last one is still owed");
   rv_mem_line_client #(.SW(MSW), .AW(LAW)) u_ic_mem
     (.clk(clk), .reset(reset),
      .l2_req(ic_l2_req | ic_rr), .l2_we(ic_l2_we), .l2_addr(ic_l2_addr), .l2_wdata(ic_l2_wdata),
      .l2_ack(ic_mack), .l2_rdata(ic_l2_rdata),
      .cq_valid(mc_q_valid[1]), .cq_ready(mc_q_ready[1]), .cq_slot(mc_q_slot[1*MSW +: MSW]),
      .cq_we(mc_q_we[1]), .cq_addr(mc_q_addr[1*LAW +: LAW]), .cq_wmask(mc_q_wmask[1*64 +: 64]),
      .cq_wdata(mc_q_wdata[1*512 +: 512]),
      .cr_valid(mc_r_valid[1]), .cr_slot(mc_r_slot), .cr_beat(mc_r_beat), .cr_last(mc_r_last),
      .cr_data(mc_r_data), .cw_valid(mc_w_valid[1]), .cw_slot(mc_w_slot));

   wire            m_q_valid, m_q_ready, m_q_we;
   wire [MIDW-1:0] m_q_id;  wire [LAW-1:0] m_q_addr;  wire [63:0] m_q_wmask;  wire [511:0] m_q_wdata;
   wire            m_r_valid, m_r_ready, m_r_last, m_w_valid, m_w_ready;
   wire [MIDW-1:0] m_r_id, m_w_id;  wire [1:0] m_r_beat;  wire [127:0] m_r_data;
   rv_mem_arbiter #(.NC(MC), .CB(MCB), .SW(MSW), .AW(LAW)) u_arb
     (.clk(clk), .reset(reset),
      .cq_valid(mc_q_valid), .cq_ready(mc_q_ready), .cq_slot(mc_q_slot), .cq_we(mc_q_we),
      .cq_addr(mc_q_addr), .cq_wmask(mc_q_wmask), .cq_wdata(mc_q_wdata),
      .cr_valid(mc_r_valid), .cr_slot(mc_r_slot), .cr_beat(mc_r_beat), .cr_last(mc_r_last),
      .cr_data(mc_r_data), .cw_valid(mc_w_valid), .cw_slot(mc_w_slot),
      .mq_valid(m_q_valid), .mq_ready(m_q_ready), .mq_id(m_q_id), .mq_we(m_q_we),
      .mq_addr(m_q_addr), .mq_wmask(m_q_wmask), .mq_wdata(m_q_wdata),
      .mr_valid(m_r_valid), .mr_ready(m_r_ready), .mr_id(m_r_id), .mr_beat(m_r_beat),
      .mr_last(m_r_last), .mr_data(m_r_data),
      .mw_valid(m_w_valid), .mw_ready(m_w_ready), .mw_id(m_w_id));

   // ---------------- memory: internal local SRAM (BRAM) + EXTERNAL DDR port ----------------
   // Each request is region-decoded: the on-chip local SRAM at LBASE (boot/monitor; FPGA = BRAM
   // init'd from mem.even/odd) is served here; the DDR region at BASE goes out on the ddr_*
   // port -- the sim TB's model, or the FPGA's crossing into the DDR4 controller. The cache is
   // PIPT so it fills/writes either region transparently. (lmem is $readmemh-loadable.)
   localparam LSIZE = 1<<LRAM_LG2;
   wire [63:0] m_pa       = {{6{1'b0}}, m_q_addr} << 6;     // physical byte addr of the line
   wire        m_is_local = (m_pa >= LBASE) && (m_pa < LBASE + LSIZE);

   // local SRAM responder (on-chip BRAM). Stored as 512-bit LINES (the port is line-granular)
   // so it infers a clean single-read/single-write BRAM -- a byte array with a 64-byte for-loop
   // access does NOT (Vivado can't template it). One transaction at a time: it reads the line,
   // then hands out its four beats (or one write done) whenever the DDR side is not answering
   // in that cycle; the DDR side has no reason to wait, the boot SRAM's traffic is rare.
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
   wire [LAW-1:0] l_line = m_q_addr - LLBASE;    // local line index
   reg            l_busy, l_rd, l_wd;           // taken; a read's beats / a write's done owed
   reg            l_ld;                          // the line is being read out of lmem this cycle
   reg  [MIDW-1:0] l_id;
   reg  [1:0]     l_beat;
   reg  [511:0]   l_q;
   wire           l_take   = m_q_valid & m_is_local & ~l_busy;
   wire           l_r_go   = l_rd & ~ddr_r_valid;               // a beat leaves this cycle
   wire           l_w_go   = l_wd & ~ddr_w_valid;
   always @(posedge clk) begin
      if (l_take) begin
         // lmem has NLLINE entries, so a LAW-bit (58) index at the array bracket is a silent
         // truncation -- an out-of-range line would WRAP onto a valid one. m_is_local bounds
         // l_line, so narrow explicitly and assert the precondition rather than trust it.
         if (|l_line[LAW-1:LLW])
            $fatal(1, "rv_soc_top: local SRAM line %h out of range (NLLINE=%0d)", l_line, NLLINE);
         if (m_q_we && m_q_wmask != {64{1'b1}})
            $fatal(1, "rv_soc_top: a partial-line write to the local SRAM (mask %h)", m_q_wmask);
         if (m_q_we) lmem[l_line[LLW-1:0]] <= m_q_wdata;
         else        l_q <= lmem[l_line[LLW-1:0]];
         l_id <= m_q_id;
      end
      if (reset) begin l_busy <= 1'b0; l_rd <= 1'b0; l_wd <= 1'b0; l_ld <= 1'b0; end
      else begin
         l_ld <= l_take & ~m_q_we;
         if (l_take) begin l_busy <= 1'b1; l_beat <= 2'd0; l_wd <= m_q_we; end
         if (l_ld) l_rd <= 1'b1;
         if (l_r_go) begin
            l_beat <= l_beat + 2'd1;
            if (l_beat == 2'd3) begin l_rd <= 1'b0; l_busy <= 1'b0; end
         end
         if (l_w_go) begin l_wd <= 1'b0; l_busy <= 1'b0; end
      end
   end

   // the external DDR port: requests outside the SRAM go out as they are
   assign ddr_q_valid = m_q_valid & ~m_is_local;
   assign ddr_q_id    = m_q_id;
   assign ddr_q_we    = m_q_we;
   assign ddr_q_addr  = m_q_addr;
   assign ddr_q_wmask = m_q_wmask;
   assign ddr_q_wdata = m_q_wdata;
   assign m_q_ready   = m_is_local ? ~l_busy : ddr_q_ready;
   assign ddr_r_ready = m_r_ready;
   assign ddr_w_ready = m_w_ready;

   // responses: the DDR side's, else the SRAM's
   assign m_r_valid = ddr_r_valid | l_r_go;
   assign m_r_id    = ddr_r_valid ? ddr_r_id   : l_id;
   assign m_r_beat  = ddr_r_valid ? ddr_r_beat : l_beat;
   assign m_r_last  = ddr_r_valid ? ddr_r_last : (l_beat == 2'd3);
   assign m_r_data  = ddr_r_valid ? ddr_r_data : l_q[l_beat*128 +: 128];
   assign m_w_valid = ddr_w_valid | l_w_go;
   assign m_w_id    = ddr_w_valid ? ddr_w_id : l_id;

   // DDR latency HPM: per request id, the cycles from the request's acceptance to its read's last
   // beat or its write done (core cycles), into read/write log2 histograms, read-only at
   // HPM_BASE (any write clears). Observation-only; off the core critical path.
   ddr_hpm #(.IDW(MIDW)) u_ddr_hpm
     (.clk(clk), .reset(reset),
      .q_fire(ddr_q_valid & ddr_q_ready), .q_id(ddr_q_id),
      .r_done(ddr_r_valid & ddr_r_ready & ddr_r_last), .r_id(ddr_r_id),
      .w_done(ddr_w_valid & ddr_w_ready), .w_id(ddr_w_id),
      .raddr(dmem_raddr[7:0]), .rdata(hpm_rdata),
      .clr(dmem_wen & is_hpm_w & ~dev_wack));
endmodule

`default_nettype wire
