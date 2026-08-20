`default_nettype none
`ifndef PROBE_CLK_DIV8
 `define PROBE_CLK_DIV8 120
`endif
`define PROBE_CLK_HZ ((1_000_000_000 / `PROBE_CLK_DIV8) * 8)

// No width knobs: the core is scalar, so the I$ window is fixed at HW=2 halfwords
// (one 32-bit instruction) and there is no per-shard writeback bus to size.
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

// FORK of probe/soc_top.v, retargeted to the in-order core. Everything outside the
// core instance -- MMIO routing, CLINT/PLIC/UART, virtio bridge, the I$/D$ adapters,
// the PTW-through-D$ adapters, the l2_arbiter, local SRAM and the DDR line port -- is
// carried over verbatim; keep the two in sync when touching those.
//
// Deltas vs soc_top.v:
//   * backend_top -> ino_core (scalar: HW=2, one 32-bit fetch window; no POOL/PBITS,
//     no per-shard writeback observation bus).
//   * TWO page-table walkers, not three. The OoO LSU runs separate load and store
//     walkers because loads and stores translate in parallel; the in-order LSU has
//     one memory op in flight, so one data walker serves loads, stores and atomics.
//   * `commit` -> `retire` (one instruction per pulse, in program order).
//
// Synthesizable SoC top: the in-order core (ino_core) + unified I$/D$ (cache.v)
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
`ifndef INO_HW
 `define INO_HW 2                 // fetch window halfwords (must match ino_core.v)
`endif
module ino_soc_top #(
   parameter HW=`INO_HW, PCW=64, SEQW=8,   // fetch window halfwords (must match ino_core.v)
   parameter [63:0] BASE     = 64'h8000_0000,   // DDR
   parameter        RAM_LG2  = 21,              // 2 MiB DDR
   parameter [63:0] LBASE    = 64'h7000_0000,   // on-chip local SRAM (boot/monitor) -- MEM_BASEADDR on the FPGA
   parameter        LRAM_LG2 = 18,              // 256 KiB local SRAM
   parameter [63:0] RESET_PC = BASE,            // tests link @DDR; the platform boots @LBASE
   parameter        SIZE_KB  = 128              // each cache
) (
   input  wire             clk,
   input  wire             reset,
   // observation for a TB (retire + the store stream, to watch tohost)
   output wire             retire,
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
   output wire [12:0]      virtio_addr,   // 13-bit: bit12 selects blk(0)/net(0x1000) within the 8 KiB region
   output wire             virtio_read,
   output wire             virtio_write,
   output wire [31:0]      virtio_wdata,
   output wire [3:0]       virtio_be,
   input  wire [31:0]      virtio_rdata,
   input  wire             virtio_rvalid,   // virtio read-data valid (req/rsp; tolerates CDC-bridge latency)
   input  wire             virtio_irq,
   input  wire             virtio_net_irq,   // PLIC source 12 (ubuntu-nfs.dts virtio@10003000)
   output wire [17:0]      irq_dbg          // interrupt-path debug for the wrapper ILA (probe_clk)
);
   localparam SIZE = 1<<RAM_LG2;
   localparam AW   = 64;
   localparam LAW  = AW-6;                  // line address width = 58

   // ---------------- core <-> caches nets ----------------
   wire [PCW-1:0]      imem_addr;
   wire [HW*16-1:0]    imem_data;
   wire [$clog2(HW+2)-1:0] imem_avail;   // sized to the frontend port ($clog2(HW+2)); drive HW, not a literal
   wire [63:0]         dmem_raddr;
   wire                dmem_ren;
   wire                dmem_runcached, dmem_wuncached;   // Svpbmt: NC/IO read/write attribute
   wire                dmem_cbo, dmem_cbo_zero, dmem_cbo_keep;  // Zicbom/Zicboz cache maintenance
   wire [63:0]         dmem_rdata;
   wire [63:0]         dmem_wabase;   // store base PA, unmuxed by the straddle beat
   wire                dmem_rvalid, dmem_wready, dmem_idle, ifence;
   wire [55:0]         ptw_addr, dptw_addr;
   wire                ptw_read, dptw_read;
   wire [63:0]         ptw_rdata, dptw_rdata;
   wire                ptw_rvalid, dptw_rvalid;
   wire                redirect;  wire [PCW-1:0] redirect_target;

   ino_core #(.HW(HW), .PCW(PCW), .SEQW(SEQW), .RESET_PC(RESET_PC)) core
     (.clk(clk), .reset(reset),
      .imem_addr(imem_addr), .imem_data(imem_data), .imem_avail(imem_avail), .hw_ip(hw_ip), .mtime(clint_mtime),
      .hpm_dc_access(dc_access), .hpm_dc_miss(dc_miss), .hpm_ic_access(ic_access), .hpm_ic_miss(ic_miss),
      .dmem_raddr(dmem_raddr), .dmem_ren(dmem_ren), .dmem_runcached(dmem_runcached),
      .dmem_rdata(dmem_rdata), .dmem_rvalid(dmem_rvalid),
      .dmem_wen(dmem_wen), .dmem_waddr(dmem_waddr), .dmem_wabase(dmem_wabase),
      .dmem_wdata(dmem_wdata), .dmem_wmask(dmem_wmask),
      .dmem_wuncached(dmem_wuncached),
      .dmem_cbo(dmem_cbo), .dmem_cbo_zero(dmem_cbo_zero), .dmem_cbo_keep(dmem_cbo_keep),
      .dmem_wready(dmem_wready), .dmem_idle(dmem_idle), .ifence(ifence),
      .ptw_addr(ptw_addr), .ptw_read(ptw_read), .ptw_rdata(ptw_rdata), .ptw_rvalid(ptw_rvalid),
      .dptw_addr(dptw_addr), .dptw_read(dptw_read), .dptw_rdata(dptw_rdata), .dptw_rvalid(dptw_rvalid),
      .retire(retire), .retire_pc(), .retire_insn(),
      .redirect(redirect), .redirect_target(redirect_target));

   // ---------------- MMIO device routing (CLINT + UART bypass the D$, non-cacheable) ----------------
   localparam [63:0] CLINT_BASE = 64'h0200_0000, UART_BASE = 64'h1000_0000, PLIC_BASE = 64'h0C00_0000;
   // DDR latency HPM window (read-only counters; any write clears). NOT in the DTB -- read it
   // from a bare-metal tool / the monitor; the kernel never touches it.
   localparam [63:0] HPM_BASE   = 64'h1800_0000;
   localparam [63:0] VIRTIO_BASE = 64'h1000_2000;                 // virtio-mmio, 8 KiB: blk @+0x0000, net @+0x1000
   localparam [63:0] BUILDID_BASE = 64'h1000_F000;                // build-id (SMOL/stamp/commit/dirty), probe-core-readable
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
      $fatal(1, "ino_soc_top: dmem_ren & dmem_wen asserted together -- dev_addr select is ambiguous");
   // ...and a device write is never the straddling second beat, which is what makes decoding
   // from the base equivalent. If this ever fires, the decode above is addressing the wrong word.
   always @(posedge clk) if (!reset & dmem_wen & is_dev_w & (dmem_waddr != dmem_wabase))
      $fatal(1, "ino_soc_top: device write straddled a word (beat %h base %h)", dmem_waddr, dmem_wabase);
   wire is_clint_r = (dmem_raddr & ~64'hffff)     == CLINT_BASE;
   wire is_uart_r  = (dmem_raddr & ~64'hf)        == UART_BASE;
   wire is_plic_r  = (dmem_raddr & ~64'h3ff_ffff) == PLIC_BASE;   // 64 MiB region
   wire is_hpm_r   = (dmem_raddr & ~64'hff)        == HPM_BASE;    // 256 B window
   wire is_virtio_r = (dmem_raddr & ~64'h1fff)    == VIRTIO_BASE;   // 8 KiB: blk(+0) + net(+0x1000)
   wire is_buildid_r = (dmem_raddr & ~64'hff)     == BUILDID_BASE;  // 256 B window (read-only)
   wire is_dev_r   = is_clint_r | is_uart_r | is_plic_r | is_hpm_r | is_virtio_r | is_buildid_r;
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
   assign    uart_irq = uart_rx_ip | uart_thre_ip;
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
               case ((dmem_waddr - UART_BASE + ub) & 3'h7)
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
   wire [63:0]  dc_rd_data;  wire dc_rd_valid, dc_wr_ack;  wire [63:0] dc_rd_resp_addr;
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
   wire         raw_rvalid = is_virtio_r ? (virtio_rvalid & vio_pending)  // not a write's completion
                           : is_dev_r    ? dev_rvalid_q
                           : dc_rv_ok;
   wire [63:0]  raw_rdata  = is_virtio_r ? {virtio_rdata, virtio_rdata}  // 32b reg, valid at virtio_rvalid
                           : is_dev_r    ? dev_rdata_q
                           : dc_rd_data;
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
   //   cache .wr_req  = dmem_wen & ~dc_wr_ack & ~is_dev_w      (a device store never reaches it)
   //   vio_wpending  <= set only by dmem_wen & is_virtio_w
   // So an OR is equivalent -- and it is an OR of registered signals, with no compare in it.
   // The exclusivity is asserted rather than assumed.
   wire         vio_wack = virtio_rvalid & vio_wpending;
   assign       dmem_wready = vio_wack | dev_wack | dc_wr_ack;
   always @(posedge clk)
      if (!reset & ((vio_wack & dev_wack) | (vio_wack & dc_wr_ack) | (dev_wack & dc_wr_ack)))
         $fatal(1, "ino_soc_top: two write acks at once (vio=%b dev=%b dc=%b) -- wready ambiguous",
                vio_wack, dev_wack, dc_wr_ack);

   // D$ is WRITE-BACK (WRTHRU=0): stores ack into the line (dirty), evicted lazily -- the
   // store buffer drains in ~1-2c instead of a full L2 round-trip. PTW reads are routed THROUGH
   // the D$ (the dcr_* read-port arbiter below), so a page-table walk always sees dirty PTEs --
   // the D$ is the coherency point. sfence.vma therefore needs NO D$ flush (just a TLB flush);
   // only fence.i still clean-flushes (the I$ reads L2 directly) -- see the df_* FSM below.
   wire        dcr_req;  wire [63:0] dcr_addr;     // muxed D$ read port (LSU + 3 PTW), assigned below
   wire dc_inv_req, dc_inv_busy;
   // Zihpm cache-event taps (D$/I$ line-lookup + miss pulses) -> core hpm_ev.
   wire dc_access, dc_miss, ic_access, ic_miss;
   ino_cache #(.PAW(64), .SIZE_KB(SIZE_KB), .RDW(64), .WDW(64), .WRITABLE(1), .WRTHRU(0), .PERF_ID(1)) u_dcache
     (.clk(clk), .reset(reset),
      .rd_req(dcr_req), .rd_addr(dcr_addr), .rd_data(dc_rd_data), .rd_valid(dc_rd_valid),
      .rd_resp_addr(dc_rd_resp_addr), .rd_tag(dcr_tag), .rd_resp_tag(dc_rd_resp_tag),
      // Svpbmt: only a LSU load read can be NC (PTW reads share dcr but are always cacheable -> 0
      // when c_rd_req is low). The store's NC bit qualifies the write port.
      .rd_uncached(c_rd_req & dmem_runcached), .wr_uncached(dmem_wuncached),
      .cbo_req(dmem_cbo & ~dc_wr_ack & ~is_dev_w), .cbo_zero(dmem_cbo_zero), .cbo_keep(dmem_cbo_keep),
      .wr_req(dmem_wen & ~dc_wr_ack & ~is_dev_w), .wr_addr(dmem_waddr), .wr_data(dmem_wdata),
      .wr_mask(dmem_wmask), .wr_ack(dc_wr_ack), .inv_req(dc_inv_req), .inv_clean(1'b1), .inv_busy(dc_inv_busy),
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
   // ---- CHUNK-ALIGNED RUN-AHEAD FETCH BUFFER --------------------------------------
   // Replaces a one-window adapter that tagged its single held window with the EXACT byte
   // address it was fetched at (`i_pa == imem_addr`).  Every PC change therefore missed and
   // re-looked-up the I$: 1.004 lookups per RETIRED INSTRUCTION, measured, costing ~2.0 CPI
   // of FE_BUB on silicon on every workload tried (cold boot, tight hot loop, gzip alike) --
   // ~1.0 waiting for a hit that need not have happened plus ~1.0 of arrival bubble.  It was
   // invisible because at HW=2 the window is exactly one instruction wide, so an exact-match
   // tag and a range check behave identically; the comparison only becomes WRONG once the
   // window is wider than the instruction being fetched.
   //
   // Now: two CHUNK-ALIGNED windows held back to back, served to the frontend by a byte shift
   // across the pair.  The I$ is looked up once per CHUNK (2 instructions at HW=4, up to 4
   // with RVC) instead of once per instruction, and because the cache is idle while a chunk is
   // being consumed, chunk1 is prefetched into that idle slot -- so the lookup latency lands
   // off the critical path entirely rather than in front of every instruction.
   //
   // Requests are chunk-ALIGNED, so the I$ never sees a line-crossing fetch.  Given what the
   // D-cache span path cost this project, removing a whole class of straddling access from the
   // I$ is worth as much as the cycles.
   //
   // PAGES.  imem_addr is a PHYSICAL address, and the virtually-next page is not the
   // physically-next page, so chunk1 is prefetched ONLY when it lies in the same 4 KiB page as
   // chunk0.  At a page boundary fb_v1 simply stays 0, the next PC misses, and the buffer
   // realigns on the freshly translated PA -- one extra lookup per page, which is nothing.
   // This also keeps the pair within one translation, so serving across the pair is always
   // serving bytes the iMMU actually vouched for.
   localparam integer CHB  = HW*2;              // chunk bytes
   localparam integer CHA  = $clog2(HW*2);      // chunk-align shift
   localparam integer PGB  = 12;                // 4 KiB page

   reg  [63:0]      fb_pa;                      // PA of chunk0 (chunk-aligned)
   // chunk1 is only fetchable if it shares chunk0's page (see PAGES above)
   wire [63:0]      fb_pa1;
   wire             fb_samepg;
   reg              fb_v0, fb_v1;
   reg  [HW*16-1:0] fb_w0, fb_w1;
   reg              fb_pend;  reg [63:0] fb_reqpa;

   wire [HW*16-1:0] ic_rd_data;  wire ic_rd_valid, ic_inv_busy;  wire [63:0] ic_rd_resp_addr;
   assign fb_pa1    = fb_pa + CHB;
   assign fb_samepg = (fb_pa1[63:PGB] == fb_pa[63:PGB]);

   // Hit test is an EQUALITY on chunk-aligned addresses, not a subtract-and-compare on byte
   // offsets. The first cut computed `imem_addr - fb_pa` and compared the 64-bit result against
   // CHB and 2*CHB; that put a 64-bit subtractor plus two magnitude comparators directly in the
   // fetch path and cost 0.79 ns of WNS (+0.061 -> -0.728 at 111 MHz). Equality against a
   // registered address is a comparator tree with no carry chain, and the offset within the
   // chunk is then just the PC's low bits -- free.
   wire [63:0]      fb_al  = {imem_addr[63:CHA], {CHA{1'b0}}};  // chunk containing the PC
   wire [CHA-1:0]   fb_lo  = imem_addr[CHA-1:0];                // byte offset within that chunk
   wire             fb_in0 = fb_v0 & (fb_al == fb_pa);
   wire             fb_in1 = fb_v1 & (fb_al == fb_pa1);
   wire             fb_hit = fb_in0 | fb_in1;
   wire             fb_miss = ~fb_hit;
   // offset into the PAIR: chunk1 hits start CHB bytes in. No subtractor.
   wire [CHA+1-1:0] fb_off = {fb_in1, fb_lo};

   // what we want next: the PC's chunk on a miss, else fill 0, else prefetch 1
   wire [63:0] fb_want  = fb_miss ? fb_al : (~fb_v0 ? fb_pa : fb_pa1);
   wire        fb_wantv = fb_miss | ~fb_v0 | (fb_v0 & ~fb_v1 & fb_samepg);

   // NOT gated on fi_stall: it is declared further down, and the adapter this replaces did not
   // gate on it either -- ic_inv_req clears the buffer, and imem_avail below holds fetch off.
   wire         ic_rd_req  = (fb_wantv | fb_pend) & ~ic_rd_valid;
   wire [63:0]  ic_rd_addr = fb_pend ? fb_reqpa : fb_want;
   wire         ic_l2_req, ic_l2_we;  wire [LAW-1:0] ic_l2_addr;  wire [511:0] ic_l2_wdata;
   wire [511:0] ic_l2_rdata;  wire ic_l2_ack;
   reg          ic_inv_req;
   // Advance/realign and fill are resolved TOGETHER, because they can land in the same cycle
   // and the fill targets slots named relative to the CURRENT fb_pa. Ordering matters: each arm
   // assigns the shift first and then lets a matching fill overwrite it (last nonblocking
   // assignment wins), so a response arriving exactly as the buffer moves is not lost.
   always @(posedge clk) if (reset) begin
         fb_v0 <= 1'b0; fb_v1 <= 1'b0; fb_pend <= 1'b0; fb_pa <= 64'd0;
      end else begin
         if (~fb_pend & fb_wantv) begin fb_pend <= 1'b1; fb_reqpa <= fb_want; end
         if (ic_rd_valid) fb_pend <= 1'b0;

         if (ic_inv_req) begin                                  // fence.i: drop everything
            fb_v0 <= 1'b0; fb_v1 <= 1'b0;
         end else if (fb_miss) begin                            // redirect / page cross: realign
            fb_pa <= fb_al; fb_v0 <= 1'b0; fb_v1 <= 1'b0;
            if (ic_rd_valid & (ic_rd_resp_addr == fb_al)) begin fb_w0 <= ic_rd_data; fb_v0 <= 1'b1; end
         end else if (fb_in1) begin                             // PC moved on: chunk1 -> chunk0
            fb_pa <= fb_pa1;
            fb_w0 <= fb_w1;  fb_v0 <= fb_v1;  fb_v1 <= 1'b0;
            if (ic_rd_valid & (ic_rd_resp_addr == fb_pa1)) begin fb_w0 <= ic_rd_data; fb_v0 <= 1'b1; end
         end else begin                                         // steady state: just fill
            if (ic_rd_valid & (ic_rd_resp_addr == fb_pa )) begin fb_w0 <= ic_rd_data; fb_v0 <= 1'b1; end
            if (ic_rd_valid & (ic_rd_resp_addr == fb_pa1)) begin fb_w1 <= ic_rd_data; fb_v1 <= 1'b1; end
         end
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
   // Arrival bypass: serve the window COMBINATIONALLY the cycle the I$ delivers it
   // (ic_rd_data is a register inside the cache, so this adds a mux, not logic
   // depth from the arrays). Guard with the response address: a redirect can move
   // pc while a window is in flight, and the stale response must read as a miss.
   // Arrival bypass, kept from the old adapter: serve COMBINATIONALLY the cycle the I$ delivers
   // the chunk the PC is in (ic_rd_data is already a register inside the cache, so this is a mux,
   // not array depth). Without it every redirect pays an extra cycle, and redirects are 11.5% of
   // instructions here.
   wire             fb_arr  = ic_rd_valid & (ic_rd_resp_addr == fb_al);
   wire [CHA-1:0]   fb_aoff = fb_lo;                            // offset within the arriving chunk

   // Serve for a hit in EITHER chunk. The first cut served only on fb_in0, so the cycle the PC
   // crossed into chunk1 imem_avail read 0 and the frontend took a bubble -- one per chunk
   // crossing, i.e. roughly one per 3 instructions at HW=4, which ate much of the win. The pair
   // is contiguous and within one page by construction, so a chunk1 hit is served by shifting
   // CHB further into the same pair; the shift is by fb_off, which now costs no arithmetic.
   wire [2*HW*16-1:0] fb_pair = {fb_w1, fb_w0};
   wire [2*HW*16-1:0] fb_shf  = fb_pair >> {fb_off, 3'b000};
   wire [HW*16-1:0]   fb_srv  = fb_shf[HW*16-1:0];
   wire [HW*16-1:0]   fb_asrv = ic_rd_data >> {fb_aoff, 3'b000};
   assign imem_data  = fb_hit ? fb_srv : (fb_arr ? fb_asrv : {(HW*16){1'b0}});

   // Valid halfwords from the PC: to the end of chunk1 if it is present, else to the end of
   // chunk0, capped at the HW the frontend asked for. fetch.v caps this again at the 4 KiB
   // boundary (eff_avail), so page-straddling instructions stay its business, not ours.
   // Valid bytes from the PC: to the end of chunk1 when it is present, else to the end of
   // chunk0. Small arithmetic on CHA+1 bits, not on 64.
   // CHB is a localparam integer, so `2*CHB` and `CHB - x` are 32-bit expressions that were
   // truncated at these narrow wires. Sized copies keep the arithmetic at the declared width.
   localparam integer   CHB_2I = 2*CHB;      // 32-bit intermediates, part-selected to width
   localparam [CHA+1:0] CHB_X2 = CHB_2I[CHA+1:0];
   localparam [CHA+1:0] CHB_X1 = CHB[CHA+1:0];
   localparam [CHA:0]   CHB_A  = CHB[CHA:0];
   wire [CHA+1:0] fb_end = fb_v1 ? CHB_X2 : CHB_X1;             // first invalid byte of the pair
   wire [CHA+1:0] fb_vb  = fb_end - {1'b0, fb_off};             // valid bytes from the PC
   wire [CHA:0]   fb_avb = CHB_A - {1'b0, fb_aoff};             // ditto on the arrival path
   wire [CHA+1:0] fb_vhw = fb_hit ? (fb_vb >> 1) : {1'b0, fb_avb[CHA:1]};
   // Freeze fetch during a fence.i (fi_stall): the I$ must not refetch until the D$ has written
   // back the freshly-stored code and the I$ has been invalidated. fi_stall spans the whole df
   // clean-flush (fi waits for df==DF_IDLE before invalidating), so it covers df_stall too.
   // sfence.vma no longer freezes fetch: the PTW reads through the coherent D$ (no flush).
   // imem_avail is $clog2(HW+2) bits (:108). The literals were 32-bit and fb_vhw is CHA+2
   // bits, so all three arms were truncated here. The last arm is reached only when
   // fb_vhw < HW, so the narrowing cannot lose a value.
   localparam AVW = $clog2(HW+2);
   localparam [AVW-1:0] AV_HW = HW;
   assign imem_avail = (fi_stall | ~(fb_hit | fb_arr)) ? {AVW{1'b0}}
                     : (fb_vhw >= HW)                     ? AV_HW
                     :                                      fb_vhw[AVW-1:0];

   ino_cache #(.PAW(64), .SIZE_KB(SIZE_KB), .RDW(HW*16), .WDW(64), .WRITABLE(0), .PREFETCH(1),
           .PERF_ID(0)) u_icache
     (.clk(clk), .reset(reset),
      .rd_req(ic_rd_req), .rd_addr(ic_rd_addr), .rd_data(ic_rd_data), .rd_valid(ic_rd_valid),
      .rd_resp_addr(ic_rd_resp_addr),
      // The I$ still matches its response by address, in the fetch buffer (fb_al/fb_pa1/fb_pa).
      // Named and empty on purpose: converting it is a separate change with its own measurement.
      .rd_tag(4'd0), .rd_resp_tag(),
      .rd_uncached(1'b0),
      .wr_req(1'b0), .wr_addr(64'd0), .wr_data(64'd0), .wr_mask(8'd0), .wr_ack(), .wr_uncached(1'b0),
      .cbo_req(1'b0), .cbo_zero(1'b0), .cbo_keep(1'b0),
      .inv_req(ic_inv_req), .inv_clean(1'b0), .inv_busy(ic_inv_busy),
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
   reg  [1:0]   pw_rvalid;
   reg  [63:0]  pw_rdata [0:1];
   wire [1:0]   pw_match;
   wire [511:0] arb_rdata;
   genvar g;
   generate for (g=0; g<2; g=g+1) begin : ptw_adapt
      assign pw_match[g] = dc_rd_valid & pw_busy[g]
                         & (dc_rd_resp_tag == {(g ? 2'd2 : 2'd1), 2'b00});
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
   assign dptw_rdata=pw_rdata[1];  assign dptw_rvalid=pw_rvalid[1];

   // D$ read-port arbiter: the LSU (c_rd_req/dmem_raddr) and the 3 PTW walks share the single D$
   // read port. Fixed priority LSU > iPTW > dPTW. The cache samples rd_req only at its
   // S_IDLE and self-serializes; each client holds its request until its response matches, so a
   // purely combinational mux suffices (no accept handshake). No deadlock: a load needing ldPTW
   // is itself blocked on translation and not issuing c_rd_req, so the walk gets the port.
   assign dcr_req  = c_rd_req | (|pw_busy);
   assign dcr_addr = c_rd_req    ? dmem_raddr
                   : pw_busy[0]  ? {8'd0, pw_addr[0*56 +: 56]}
                   :               {8'd0, pw_addr[1*56 +: 56]};
   // ...and the tag that names the requester, selected by the SAME priority.
   wire [DRTW-1:0] dcr_tag = c_rd_req   ? lsu_tag_req
                           : pw_busy[0] ? {2'd1, 2'b00}
                           :              {2'd2, 2'b00};

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
   ino_l2_arbiter #(.NREQ(NREQ), .AW(LAW), .DW(512)) u_arb
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
            $fatal(1, "ino_soc_top: local SRAM line %h out of range (NLLINE=%0d)", l_line, NLLINE);
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
