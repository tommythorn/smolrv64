`timescale 1ns / 1ps

// ---- probe-core clock (BUFGCE_DIV: ui_clk / N) --------------------------------------
// ONE knob for the Fmax sweep. The UART CLK_FREQ below and the CLINT SCALE_DIV in
// core/rv_soc_top.v are DERIVED from it, so a sweep cannot silently
// skew the console baud or the timebase (both bit us before -- see the comments at each
// site).
//
// It is BUFGCE_DIV(ui_clk)/N, reaching 333.3 / 166.7 / 111.1 / 83.3 / 66.7 MHz.  An MMCM
// was tried here (8e71c72e, reverted 2026-08-20) to make probe_clk continuous; it did not,
// because the timer quantizes probe_clk to ui_clk regardless -- see the knob comment below
// for the measurements.  Every rung of this ladder is a 3.000 ns multiple, which is exactly
// the set that ever closed.
//
// probe_clk is a BUFGCE_DIV integer divide of ui_clk.  The knob keeps its eighths-of-a-
// nanosecond meaning (period = PROBE_CLK_DIV8 / 8 ns) so build.tcl, the Makefile and
// tools/check-dts-timebase.py are unchanged, but ONLY MULTIPLES OF 24 ARE LEGAL:
//
//     probe_clk = ui_clk / (PROBE_CLK_DIV8 / 24)
//     24 -> 333.33   48 -> 166.67   72 -> 111.11   96 -> 83.33   120 -> 66.67
//
// THE MMCM THAT USED TO SIT HERE BOUGHT NOTHING.  It was added (8e71c72e) to make Fmax
// continuous, on the reasoning that "nothing depended on the phase relationship, because
// every probe_clk <-> ui_clk crossing already goes through a real CDC structure".  The
// crossings do -- but the TIMER does not know that.  probe_mmcm was fed from ui_clk, so
// Vivado treated the two as RELATED clocks and timed every crossing on EDGE ALIGNMENT.
// Whenever probe's period was not an integer multiple of ui_clk's 3.000 ns, the tightest
// launch->capture pair collapsed, and the placer then wrecked the rest of the design
// chasing a path it could never meet.  Measured 2026-08-20, identical directives:
//
//     DIV8 120 (15.000 ns, 5x)  edge pair 3.000 ns  ->  WNS +0.159  MET
//     DIV8  72 ( 9.000 ns, 3x)  edge pair 3.000 ns  ->  WNS +0.134  MET
//     DIV8  64 ( 8.000 ns)      edge pair 1.000 ns  ->  WNS -1.080
//     DIV8  68 ( 8.500 ns)      edge pair 0.500 ns  ->  WNS -1.198
//     DIV8  70 ( 8.750 ns)      edge pair 0.250 ns  ->  WNS -2.249
//     DIV8  71 ( 8.875 ns)      edge pair 0.125 ns  ->  WNS -2.372
//
// So the usable set was ALWAYS the old BUFGCE_DIV ladder, and the MMCM only added jitter,
// ~0.3 ns of clock insertion delay, a relock failure mode, and a knob that advertises
// frequencies the timer will never accept.  Two sessions burned builds on that lie.
// A BUFGCE_DIV is phase-aligned with ui_clk, adds no jitter, and cannot be asked for a
// frequency that does not exist.
//
// Override from build.tcl via env PROBE_CLK_DIV8.  The old PROBE_CLK_DIV is retired and
// build.tcl hard-errors on it: it named a different ladder, and a stale `PROBE_CLK_DIV=3`
// silently meaning 66.7 MHz is exactly the class of build divergence that has cost us days.
`ifndef PROBE_CLK_DIV8
 `define PROBE_CLK_DIV8 120
`endif
`define PROBE_CLK_HZ ((1_000_000_000 / `PROBE_CLK_DIV8) * 8)
`define PROBE_CLK_DIVIDE   (`PROBE_CLK_DIV8 / 24)   // ui_clk divisor, 1..8
`default_nettype none

`ifndef SMOLRV64_BUILD_STAMP
`define SMOLRV64_BUILD_STAMP 64'h0
`endif
`ifndef SMOLRV64_GIT_COMMIT
`define SMOLRV64_GIT_COMMIT 32'h0
`endif
`ifndef SMOLRV64_GIT_DIRTY
`define SMOLRV64_GIT_DIRTY 1'b0
`endif

module rk_xcku5p(
    // Differential 200 MHz system clock (fed directly to DDR4 IP)
    input  wire       sys_clk_p,
    input  wire       sys_clk_n,
    input  wire [3:0] key,
    input  wire       rxd,
    output wire [3:0] led,
    output wire       txd,
    output wire       sd_clk,    // SPI mode: SCK
    output wire       sd_cmd,    // SPI mode: MOSI (DI)
    input  wire       sd_cd,
    inout  wire [3:0] sd_d,

    // RGMII to RTL8211F-CG Ethernet PHY (pins per the board 12_UDP_TEST
    // design).  Driven by gmii_to_rgmii + eth_tx_engine / eth_mac_rx below.
    input  wire       eth_rxc,
    input  wire [3:0] eth_rxd,
    input  wire       eth_rx_ctl,
    output wire       eth_txc,
    output wire [3:0] eth_txd,
    output wire       eth_tx_ctl,

    // TinyVGA adapter on the 40-pin header, IO14..IO17: RGB222 and the syncs (vga_scanout).
    output wire       vga_hs,
    output wire       vga_vs,
    output wire [1:0] vga_r,
    output wire [1:0] vga_g,
    output wire [1:0] vga_b,

    // DDR4 physical ports
    output wire        c0_ddr4_act_n,
    output wire [16:0] c0_ddr4_adr,
    output wire [1:0]  c0_ddr4_ba,
    output wire [0:0]  c0_ddr4_bg,
    output wire [0:0]  c0_ddr4_cke,
    output wire [0:0]  c0_ddr4_odt,
    output wire [0:0]  c0_ddr4_cs_n,
    output wire [0:0]  c0_ddr4_ck_t,
    output wire [0:0]  c0_ddr4_ck_c,
    output wire        c0_ddr4_reset_n,
    inout  wire [3:0]  c0_ddr4_dm_dbi_n,
    inout  wire [31:0] c0_ddr4_dq,
    inout  wire [3:0]  c0_ddr4_dqs_t,
    inout  wire [3:0]  c0_ddr4_dqs_c
    );

   // DDR4 UI clock (333.33 MHz) and reset from the IP
   wire ui_clk;
   wire ui_rst;           // c0_ddr4_ui_clk_sync_rst (active high)
   wire init_calib_complete;

   // CPU is held in reset until calibration completes.
   // key[1] is a soft-reset button (active low): pulses the CPU reset without
   // touching the DDR4 MIG, so calibration is preserved and mig_* latency
   // stats CSRs (which aren't in the CPU's reset block) survive across a
   // soft reset. Press key[1] to return to the monitor from a hung workload.
   // fbdiag_reset_req: the in-order SoC's fetch-buffer invariant fired and wants the CPU
   // reset into the ROM monitor so the snapshot at 0x1000_E000 can be read out with R<addr>.
   // key[1] is a physical button and there is no VIO, so without this the only way to read
   // the evidence is to be standing at the board.  It is a ONE-SHOT by construction: the
   // sticky bit that raises it clears only on power-on, so the edge cannot repeat.
   // probe_clk -> ui_clk, so it crosses through a 2-FF synchronizer with an XDC false path;
   // an un-exceptioned crossing between these RELATED clocks is what collapsed the timing
   // requirement to 0.125 ns before -- see the mmio_clock_bridge comments.
   wire fbdiag_reset_req;
   (* async_reg = "true" *) reg [1:0] fbdiag_rst_sync = 2'b00;
   always @(posedge ui_clk) fbdiag_rst_sync <= {fbdiag_rst_sync[0], fbdiag_reset_req};
   wire ui_cpu_reset_req = ui_rst | ~init_calib_complete | ~key[1] | fbdiag_rst_sync[1];
   reg  [1:0] ui_cpu_reset_sync = 2'b11;
   wire ui_cpu_reset = ui_cpu_reset_sync[1];

`ifndef PROBE_DIAG
   assign led = 4'b0000;
`endif

   always @(posedge ui_clk) begin
      if (ui_cpu_reset_req)
         ui_cpu_reset_sync <= 2'b11;
      else
         ui_cpu_reset_sync <= {ui_cpu_reset_sync[0], 1'b0};
   end

   // The core runs on probe_clk,
   // built by the MMCM below from PROBE_CLK_DIV8 -- see the knob at the top of this file.
   //
   // History, because the numbers below get quoted: on the old BUFGCE_DIV ladder the sharded
   // OoO core sat at /5 = 66.7 MHz. /4 = 83.3 MHz was TRIED and FAILED routing (WNS -1.78 ns
   // at the 12 ns target => real Fmax ~72.5 MHz, probe_clk paths dominating the failing set),
   // so /5 stood until the redirect/wake/MMIO cones got pipelined. The in-order core reached
   // /3 = 111.1 MHz and shipped the GB5 milestone there. Those are ladder rungs, not measured
   // Fmax: the real Fmax was always somewhere between two rungs and the MMCM is what lets us
   // find out where.
   //
   // The ddr_* memory port crosses into ui_clk (MIG/arbiter/bridge) via ddr_port_cdc.
   // The UART CLK_FREQ below AND the CLINT SCALE_DIV (core/rv_soc_top.v)
   // are derived from the same knob, so they cannot drift out of sync with a sweep.
   wire probe_clk;

`ifdef PROBE_DIAG
   // DIAGNOSTIC: decouple probe_clk/probe_reset from ui_rst (= DDR-cal-gated). A local POR
   // deasserts after ui_clk has run 1024 cycles (PLL locked; ui_clk runs post-lock independent
   // of DDR calibration on UltraScale), so the BRAM monitor boots REGARDLESS of DDR cal. If the
   // banner appears only with this build, the "dead core" was probe_clk held off by ui_rst
   // (cal/reset gating -- physical); if still dead, the core is genuinely functionally dead.
   reg [9:0] probe_por_cnt = 10'd0;
   reg       probe_por = 1'b1;
   always @(posedge ui_clk) begin
      if (probe_por_cnt != 10'h3ff) probe_por_cnt <= probe_por_cnt + 1'b1;
      else                          probe_por <= 1'b0;
   end
   wire probe_mmcm_rst  = probe_por;
   wire probe_reset_src = probe_por;
`else
   wire probe_mmcm_rst  = ui_rst;
   wire probe_reset_src = ui_cpu_reset;
`endif

   // ui_clk (333.333 MHz) / PROBE_CLK_DIVIDE -> probe_clk, phase-aligned by construction.
   // The instance keeps the name `probe_clk_buf`: rk_xcku5p.xdc finds the clock with
   // `get_clocks -of_objects [get_pins ... *probe_clk_buf/O]` rather than by literal name,
   // and probe_clk_check.tcl asserts that lookup finds exactly one clock.
   BUFGCE_DIV #(
      .BUFGCE_DIVIDE (`PROBE_CLK_DIVIDE)
   ) probe_clk_buf (
      .I   (ui_clk),
      .CE  (1'b1),
      .CLR (probe_mmcm_rst),
      .O   (probe_clk)
   );

   // No lock to wait for: a BUFGCE_DIV output is correct from its first edge (it is a
   // divider in the clock network, not an acquiring loop).  probe_clk is merely LATE, which
   // ui_rst/ui_cpu_reset already covers -- this is what the MMCM's LOCKED gating replaced.
   wire probe_mmcm_locked = 1'b1;                  // kept for the LED, see below
   wire probe_reset_async = probe_reset_src;
   (* async_reg = "true" *) reg [1:0] probe_reset_sync = 2'b11;
   always @(posedge probe_clk or posedge probe_reset_async)
      if (probe_reset_async) probe_reset_sync <= 2'b11;
      else                   probe_reset_sync <= {probe_reset_sync[0], 1'b0};
   wire probe_reset = probe_reset_sync[1];

`ifdef PROBE_DIAG
   // LEDs (core-independent): observe cal/clock health at a glance.  led[1] is now the MMCM
   // lock, not just a probe_clk heartbeat: a dark led[1] says the clock never came up.
   reg [26:0] hb_ui = 27'd0;    always @(posedge ui_clk)    hb_ui  <= hb_ui  + 1'b1;
   reg [23:0] hb_pr = 24'd0;    always @(posedge probe_clk) hb_pr  <= hb_pr  + 1'b1;
   assign led = {hb_ui[26], hb_pr[23] & probe_mmcm_locked, ui_rst, init_calib_complete};
`endif

   wire         dbg_clk;
   wire [511:0] dbg_bus;
   wire         c0_ddr4_reset_n_int;
   assign c0_ddr4_reset_n = c0_ddr4_reset_n_int;

   // AXI4 wires between smolrv64 and the DDR4 IP (64-bit data, 31-bit byte addr,
   // 3-bit ID, 8-byte beats, single-beat bursts).
   // arbiter->MIG AW/W skid nets (slice instance sits by the R slice below)
   wire [2:0]  mig_awid;   wire [30:0] mig_awaddr; wire [7:0] mig_awlen; wire [2:0] mig_awsize;
   wire [1:0]  mig_awburst; wire mig_awlock; wire [3:0] mig_awcache; wire [2:0] mig_awprot;
   wire [3:0]  mig_awqos;  wire mig_awvalid; wire mig_awready;
   wire [63:0] mig_wdata;  wire [7:0] mig_wstrb; wire mig_wlast; wire mig_wvalid; wire mig_wready;
   wire [ 2:0] m_axi_awid;
   wire [30:0] m_axi_awaddr;
   wire [ 7:0] m_axi_awlen;
   wire [ 2:0] m_axi_awsize;
   wire [ 1:0] m_axi_awburst;
   wire        m_axi_awlock;
   wire [ 3:0] m_axi_awcache;
   wire [ 2:0] m_axi_awprot;
   wire [ 3:0] m_axi_awqos;
   wire        m_axi_awvalid;
   wire        m_axi_awready;
   wire [63:0] m_axi_wdata;
   wire [ 7:0] m_axi_wstrb;
   wire        m_axi_wlast;
   wire        m_axi_wvalid;
   wire        m_axi_wready;
   wire [ 2:0] m_axi_bid;
   wire [ 1:0] m_axi_bresp;
   wire        m_axi_bvalid;
   wire        m_axi_bready;
   wire [ 2:0] m_axi_arid;
   wire [30:0] m_axi_araddr;
   wire [ 7:0] m_axi_arlen;
   wire [ 2:0] m_axi_arsize;
   wire [ 1:0] m_axi_arburst;
   wire        m_axi_arlock;
   wire [ 3:0] m_axi_arcache;
   wire [ 2:0] m_axi_arprot;
   wire [ 3:0] m_axi_arqos;
   wire        m_axi_arvalid;
   wire        m_axi_arready;
   wire [ 2:0] m_axi_rid;
   wire [63:0] m_axi_rdata;
   wire [ 1:0] m_axi_rresp;
   wire        m_axi_rlast;
   wire        m_axi_rvalid;
   wire        m_axi_rready;

   wire [ 2:0] core_axi_awid;
   wire [30:0] core_axi_awaddr;
   wire [ 7:0] core_axi_awlen;
   wire [ 2:0] core_axi_awsize;
   wire [ 1:0] core_axi_awburst;
   wire        core_axi_awlock;
   wire [ 3:0] core_axi_awcache;
   wire [ 2:0] core_axi_awprot;
   wire [ 3:0] core_axi_awqos;
   wire        core_axi_awvalid;
   wire        core_axi_awready;
   wire [63:0] core_axi_wdata;
   wire [ 7:0] core_axi_wstrb;
   wire        core_axi_wlast;
   wire        core_axi_wvalid;
   wire        core_axi_wready;
   wire [ 2:0] core_axi_bid;
   wire [ 1:0] core_axi_bresp;
   wire        core_axi_bvalid;
   wire        core_axi_bready;
   wire [ 2:0] core_axi_arid;
   wire [30:0] core_axi_araddr;
   wire [ 7:0] core_axi_arlen;
   wire [ 2:0] core_axi_arsize;
   wire [ 1:0] core_axi_arburst;
   wire        core_axi_arlock;
   wire [ 3:0] core_axi_arcache;
   wire [ 2:0] core_axi_arprot;
   wire [ 3:0] core_axi_arqos;
   wire        core_axi_arvalid;
   wire        core_axi_arready;
   wire [ 2:0] core_axi_rid;
   wire [63:0] core_axi_rdata;
   wire [ 1:0] core_axi_rresp;
   wire        core_axi_rlast;
   wire        core_axi_rvalid;
   wire        core_axi_rready;

   localparam USE_DDR_ARB = 1'b1;

   // virtio-blk is backed by the SD card in SPI mode (sd_spi_host inside
   // virtio_blk_backend). Capacity is read from the card's CSD at init and
   // reported to the guest via virtio config.
   wire [31:0] virtio_blk_capacity;
   wire [31:0] virtio_blk_debug_word;   // SD/blk debug overlay at 0x10002f00
   wire [21:0] virtio_blk_dbg;          // always-on backend FSM/SD/DMA state for ILA_DEV

   // DDR4 MIG IP instantiation (AXI4 slave)
   ddr4_0 u_ddr4_0 (
      .sys_rst                        (~key[0]),          // active-high; key[0] low = pressed = reset

      .c0_sys_clk_p                   (sys_clk_p),
      .c0_sys_clk_n                   (sys_clk_n),
      .c0_init_calib_complete         (init_calib_complete),
      .c0_ddr4_act_n                  (c0_ddr4_act_n),
      .c0_ddr4_adr                    (c0_ddr4_adr),
      .c0_ddr4_ba                     (c0_ddr4_ba),
      .c0_ddr4_bg                     (c0_ddr4_bg),
      .c0_ddr4_cke                    (c0_ddr4_cke),
      .c0_ddr4_odt                    (c0_ddr4_odt),
      .c0_ddr4_cs_n                   (c0_ddr4_cs_n),
      .c0_ddr4_ck_t                   (c0_ddr4_ck_t),
      .c0_ddr4_ck_c                   (c0_ddr4_ck_c),
      .c0_ddr4_reset_n                (c0_ddr4_reset_n_int),
      .c0_ddr4_dm_dbi_n               (c0_ddr4_dm_dbi_n),
      .c0_ddr4_dq                     (c0_ddr4_dq),
      .c0_ddr4_dqs_c                  (c0_ddr4_dqs_c),
      .c0_ddr4_dqs_t                  (c0_ddr4_dqs_t),

      .c0_ddr4_ui_clk                 (ui_clk),
      .c0_ddr4_ui_clk_sync_rst        (ui_rst),
      .dbg_clk                        (dbg_clk),

      .c0_ddr4_aresetn                (~ui_rst),
      .c0_ddr4_s_axi_awid(mig_awid),
      .c0_ddr4_s_axi_awaddr(mig_awaddr),
      .c0_ddr4_s_axi_awlen(mig_awlen),
      .c0_ddr4_s_axi_awsize(mig_awsize),
      .c0_ddr4_s_axi_awburst(mig_awburst),
      .c0_ddr4_s_axi_awlock(mig_awlock),
      .c0_ddr4_s_axi_awcache(mig_awcache),
      .c0_ddr4_s_axi_awprot(mig_awprot),
      .c0_ddr4_s_axi_awqos(mig_awqos),
      .c0_ddr4_s_axi_awvalid(mig_awvalid),
      .c0_ddr4_s_axi_awready(mig_awready),
      .c0_ddr4_s_axi_wdata(mig_wdata),
      .c0_ddr4_s_axi_wstrb(mig_wstrb),
      .c0_ddr4_s_axi_wlast(mig_wlast),
      .c0_ddr4_s_axi_wvalid(mig_wvalid),
      .c0_ddr4_s_axi_wready(mig_wready),
      .c0_ddr4_s_axi_bid              (m_axi_bid),
      .c0_ddr4_s_axi_bresp            (m_axi_bresp),
      .c0_ddr4_s_axi_bvalid           (m_axi_bvalid),
      .c0_ddr4_s_axi_bready           (m_axi_bready),
      .c0_ddr4_s_axi_arid             (m_axi_arid),
      .c0_ddr4_s_axi_araddr           (m_axi_araddr),
      .c0_ddr4_s_axi_arlen            (m_axi_arlen),
      .c0_ddr4_s_axi_arsize           (m_axi_arsize),
      .c0_ddr4_s_axi_arburst          (m_axi_arburst),
      .c0_ddr4_s_axi_arlock           (m_axi_arlock),
      .c0_ddr4_s_axi_arcache          (m_axi_arcache),
      .c0_ddr4_s_axi_arprot           (m_axi_arprot),
      .c0_ddr4_s_axi_arqos            (m_axi_arqos),
      .c0_ddr4_s_axi_arvalid          (m_axi_arvalid),
      .c0_ddr4_s_axi_arready          (m_axi_arready),
      .c0_ddr4_s_axi_rid              (m_axi_rid),
      .c0_ddr4_s_axi_rdata            (m_axi_rdata),
      .c0_ddr4_s_axi_rresp            (m_axi_rresp),
      .c0_ddr4_s_axi_rlast            (m_axi_rlast),
      .c0_ddr4_s_axi_rvalid           (m_axi_rvalid),
      .c0_ddr4_s_axi_rready           (m_axi_rready),
      .dbg_bus                        (dbg_bus)
   );

   wire [19:0] core_mmio_address;
   wire        core_mmio_read;
   wire        core_mmio_write;
   wire [31:0] core_mmio_writedata;
   wire [ 3:0] core_mmio_byteenable;
   wire        core_mmio_readdatavalid;
   wire [31:0] core_mmio_readdata;
   wire [19:0] ui_mmio_address;
   wire        ui_mmio_read;
   wire        ui_mmio_write;
   wire [31:0] ui_mmio_writedata;
   wire [ 3:0] ui_mmio_byteenable;
   wire        ui_mmio_readdatavalid;
   wire [31:0] ui_mmio_readdata;

   // SPI-mode SD pins, driven by sd_spi_host inside virtio_blk_backend.
   //   SCK = sd_clk, MOSI = sd_cmd, MISO = sd_d[0], CS = sd_d[3] (active-low).
   wire        blk_sck, blk_mosi, blk_miso, blk_cs_n;
   wire [ 3:0] sd_d_i, sd_d_o, sd_d_t;
   reg         sd_cd_meta = 1'b1;
   reg         sd_cd_sync = 1'b1;

   assign sd_clk      = blk_sck;
   assign sd_cmd      = blk_mosi;
   assign blk_miso    = sd_d_i[0];
   assign sd_d_o      = {blk_cs_n, 1'b1, 1'b1, 1'b1};   // sd_d[3]=CS, [2:1] high
   assign sd_d_t      = 4'b0001;                         // sd_d[0]=MISO (input)

   genvar sd_i;
   generate
      for (sd_i = 0; sd_i < 4; sd_i = sd_i + 1) begin : sd_d_iobufs
         IOBUF sd_d_iobuf(
            .I  (sd_d_o[sd_i]),
            .O  (sd_d_i[sd_i]),
            .T  (sd_d_t[sd_i]),
            .IO (sd_d[sd_i])
         );
      end
   endgenerate

   wire        sd_cd_gpio_sel = ui_mmio_address[19:8] == 12'h012;
   // 0x10001100: writable SD SPI transfer-clock divider (SCK half-period in
   // ui_clk cycles - 1). SCK = 333.33 MHz / (2*(val+1)).  Set from the monitor
   // (e.g. WW10001100 0x0C -> ~12.8 MHz) before booting to experiment.
   wire        spi_speed_sel = ui_mmio_address[19:8] == 12'h011;
   reg  [15:0] spi_fast_half = 16'd6;    // 23.8 MHz default (HW-swept clean; ceiling ~28 MHz)
   wire        virtio_blk_sel = ui_mmio_address[19:12] == 8'h02;
   wire        virtio_net_sel = ui_mmio_address[19:12] == 8'h03;
   wire        build_id_sel   = ui_mmio_address[19:8] == 12'h0f0;
   wire        vga_sel        = ui_mmio_address[19:12] == 8'h05;
   wire        kbd_sel        = ui_mmio_address[19:12] == 8'h04;
   wire [31:0] kbd_mmio_rdata;
   wire [31:0] vga_mmio_rdata;
   wire [31:0] sd_cd_gpio_readdata = {31'd0, sd_cd_sync};
   wire [31:0] virtio_blk_readdata;
   wire [31:0] virtio_net_readdata;
   localparam [63:0] BUILD_ID_STAMP = `SMOLRV64_BUILD_STAMP;
   localparam [31:0] BUILD_ID_GIT_COMMIT = `SMOLRV64_GIT_COMMIT;
   localparam        BUILD_ID_GIT_DIRTY = `SMOLRV64_GIT_DIRTY;
   reg  [31:0] build_id_readdata;
   wire        virtio_blk_irq;
   wire        virtio_net_irq;
   wire        virtio_net_queue_notify_pulse;
   wire [31:0] virtio_net_queue_notify_value;
   wire        virtio_net_used_buffer_interrupt;
   wire [31:0] virtio_net_queue1_num;
   wire        virtio_net_queue1_ready;
   wire [63:0] virtio_net_queue1_desc;
   wire [63:0] virtio_net_queue1_driver;
   wire [63:0] virtio_net_queue1_device;
   wire [31:0] virtio_net_queue0_num;
   wire        virtio_net_queue0_ready;
   wire [63:0] virtio_net_queue0_desc;
   wire [63:0] virtio_net_queue0_driver;
   wire [63:0] virtio_net_queue0_device;
   wire [ 7:0] virtio_net_device_status;

   wire        virtio_blk_queue_notify_pulse;
   wire [31:0] virtio_blk_queue_notify_value;
   wire        virtio_blk_used_buffer_interrupt;
   wire [31:0] virtio_blk_queue0_num;
   wire        virtio_blk_queue0_ready;
   wire [63:0] virtio_blk_queue0_desc;
   wire [63:0] virtio_blk_queue0_driver;
   wire [63:0] virtio_blk_queue0_device;
   wire [ 7:0] virtio_blk_device_status;

   wire [31:0] virtio_net_debug_status;
   wire [31:0] virtio_net_debug_notify_count;
   wire [31:0] virtio_net_debug_read_avail_count;
   wire [31:0] virtio_net_debug_empty_avail_count;
   wire [31:0] virtio_net_debug_read_ring_count;
   wire [31:0] virtio_net_debug_complete_count;
   wire [31:0] virtio_net_debug_irq_count;
   wire [31:0] virtio_net_debug_dma_error_count;
   wire [31:0] virtio_net_debug_indices;
   wire [31:0] virtio_net_debug_used_head;
   wire [31:0] virtio_net_debug_last_avail_word_lo;
   wire [31:0] virtio_net_debug_last_avail_word_hi;
   wire [31:0] virtio_net_debug_last_ring_word_lo;
   wire [31:0] virtio_net_debug_last_ring_word_hi;
   wire [ 2:0] virtio_net_axi_awid;
   wire [30:0] virtio_net_axi_awaddr;
   wire [ 7:0] virtio_net_axi_awlen;
   wire [ 2:0] virtio_net_axi_awsize;
   wire [ 1:0] virtio_net_axi_awburst;
   wire        virtio_net_axi_awlock;
   wire [ 3:0] virtio_net_axi_awcache;
   wire [ 2:0] virtio_net_axi_awprot;
   wire [ 3:0] virtio_net_axi_awqos;
   wire        virtio_net_axi_awvalid;
   wire        virtio_net_axi_awready;
   wire [63:0] virtio_net_axi_wdata;
   wire [ 7:0] virtio_net_axi_wstrb;
   wire        virtio_net_axi_wlast;
   wire        virtio_net_axi_wvalid;
   wire        virtio_net_axi_wready;
   wire [ 2:0] virtio_net_axi_bid;
   wire [ 1:0] virtio_net_axi_bresp;
   wire        virtio_net_axi_bvalid;
   wire        virtio_net_axi_bready;
   wire [ 2:0] virtio_net_axi_arid;
   wire [30:0] virtio_net_axi_araddr;
   wire [ 7:0] virtio_net_axi_arlen;
   wire [ 2:0] virtio_net_axi_arsize;
   wire [ 1:0] virtio_net_axi_arburst;
   wire        virtio_net_axi_arlock;
   wire [ 3:0] virtio_net_axi_arcache;
   wire [ 2:0] virtio_net_axi_arprot;
   wire [ 3:0] virtio_net_axi_arqos;
   wire        virtio_net_axi_arvalid;
   wire        virtio_net_axi_arready;
   wire [ 2:0] virtio_net_axi_rid;
   wire [63:0] virtio_net_axi_rdata;
   wire [ 1:0] virtio_net_axi_rresp;
   wire        virtio_net_axi_rlast;
   wire        virtio_net_axi_rvalid;
   wire        virtio_net_axi_rready;
   // Net debug-overlay word (0x10003f00+); declared out here so it stays visible when
   // NO_VIRTIO_NET compiles the net block below out.
   reg  [31:0] virtio_net_debug_word;

   // virtio-blk backend DMA master + the device-side arbiter output that merges
   // net + blk into one master before the existing core-vs-device arbiter.
   wire [ 2:0] virtio_blk_axi_awid;
   wire [30:0] virtio_blk_axi_awaddr;
   wire [ 7:0] virtio_blk_axi_awlen;
   wire [ 2:0] virtio_blk_axi_awsize;
   wire [ 1:0] virtio_blk_axi_awburst;
   wire        virtio_blk_axi_awlock;
   wire [ 3:0] virtio_blk_axi_awcache;
   wire [ 2:0] virtio_blk_axi_awprot;
   wire [ 3:0] virtio_blk_axi_awqos;
   wire        virtio_blk_axi_awvalid;
   wire        virtio_blk_axi_awready;
   wire [63:0] virtio_blk_axi_wdata;
   wire [ 7:0] virtio_blk_axi_wstrb;
   wire        virtio_blk_axi_wlast;
   wire        virtio_blk_axi_wvalid;
   wire        virtio_blk_axi_wready;
   wire [ 2:0] virtio_blk_axi_bid;
   wire [ 1:0] virtio_blk_axi_bresp;
   wire        virtio_blk_axi_bvalid;
   wire        virtio_blk_axi_bready;
   wire [ 2:0] virtio_blk_axi_arid;
   wire [30:0] virtio_blk_axi_araddr;
   wire [ 7:0] virtio_blk_axi_arlen;
   wire [ 2:0] virtio_blk_axi_arsize;
   wire [ 1:0] virtio_blk_axi_arburst;
   wire        virtio_blk_axi_arlock;
   wire [ 3:0] virtio_blk_axi_arcache;
   wire [ 2:0] virtio_blk_axi_arprot;
   wire [ 3:0] virtio_blk_axi_arqos;
   wire        virtio_blk_axi_arvalid;
   wire        virtio_blk_axi_arready;
   wire [ 2:0] virtio_blk_axi_rid;
   wire [63:0] virtio_blk_axi_rdata;
   wire [ 1:0] virtio_blk_axi_rresp;
   wire        virtio_blk_axi_rlast;
   wire        virtio_blk_axi_rvalid;
   wire        virtio_blk_axi_rready;

   wire [ 2:0] devab_axi_awid;
   wire [30:0] devab_axi_awaddr;
   wire [ 7:0] devab_axi_awlen;
   wire [ 2:0] devab_axi_awsize;
   wire [ 1:0] devab_axi_awburst;
   wire        devab_axi_awlock;
   wire [ 3:0] devab_axi_awcache;
   wire [ 2:0] devab_axi_awprot;
   wire [ 3:0] devab_axi_awqos;
   wire        devab_axi_awvalid;
   wire        devab_axi_awready;
   wire [63:0] devab_axi_wdata;
   wire [ 7:0] devab_axi_wstrb;
   wire        devab_axi_wlast;
   wire        devab_axi_wvalid;
   wire        devab_axi_wready;
   wire [ 2:0] devab_axi_bid;
   wire [ 1:0] devab_axi_bresp;
   wire        devab_axi_bvalid;
   wire        devab_axi_bready;
   wire [ 2:0] devab_axi_arid;
   wire [30:0] devab_axi_araddr;
   wire [ 7:0] devab_axi_arlen;
   wire [ 2:0] devab_axi_arsize;
   wire [ 1:0] devab_axi_arburst;
   wire        devab_axi_arlock;
   wire [ 3:0] devab_axi_arcache;
   wire [ 2:0] devab_axi_arprot;
   wire [ 3:0] devab_axi_arqos;
   wire        devab_axi_arvalid;
   wire        devab_axi_arready;
   wire [ 2:0] devab_axi_rid;
   wire [63:0] devab_axi_rdata;
   wire [ 1:0] devab_axi_rresp;
   wire        devab_axi_rlast;
   wire        devab_axi_rvalid;
   wire        devab_axi_rready;

   wire [ 2:0] vga_axi_awid;
   wire [30:0] vga_axi_awaddr;
   wire [ 7:0] vga_axi_awlen;
   wire [ 2:0] vga_axi_awsize;
   wire [ 1:0] vga_axi_awburst;
   wire        vga_axi_awlock;
   wire [ 3:0] vga_axi_awcache;
   wire [ 2:0] vga_axi_awprot;
   wire [ 3:0] vga_axi_awqos;
   wire        vga_axi_awvalid;
   wire        vga_axi_awready;
   wire [63:0] vga_axi_wdata;
   wire [ 7:0] vga_axi_wstrb;
   wire        vga_axi_wlast;
   wire        vga_axi_wvalid;
   wire        vga_axi_wready;
   wire [ 2:0] vga_axi_bid;
   wire [ 1:0] vga_axi_bresp;
   wire        vga_axi_bvalid;
   wire        vga_axi_bready;
   wire [ 2:0] vga_axi_arid;
   wire [30:0] vga_axi_araddr;
   wire [ 7:0] vga_axi_arlen;
   wire [ 2:0] vga_axi_arsize;
   wire [ 1:0] vga_axi_arburst;
   wire        vga_axi_arlock;
   wire [ 3:0] vga_axi_arcache;
   wire [ 2:0] vga_axi_arprot;
   wire [ 3:0] vga_axi_arqos;
   wire        vga_axi_arvalid;
   wire        vga_axi_arready;
   wire [ 2:0] vga_axi_rid;
   wire [63:0] vga_axi_rdata;
   wire [ 1:0] vga_axi_rresp;
   wire        vga_axi_rlast;
   wire        vga_axi_rvalid;
   wire        vga_axi_rready;

   wire [ 2:0] kbd_axi_awid;
   wire [30:0] kbd_axi_awaddr;
   wire [ 7:0] kbd_axi_awlen;
   wire [ 2:0] kbd_axi_awsize;
   wire [ 1:0] kbd_axi_awburst;
   wire        kbd_axi_awlock;
   wire [ 3:0] kbd_axi_awcache;
   wire [ 2:0] kbd_axi_awprot;
   wire [ 3:0] kbd_axi_awqos;
   wire        kbd_axi_awvalid;
   wire        kbd_axi_awready;
   wire [63:0] kbd_axi_wdata;
   wire [ 7:0] kbd_axi_wstrb;
   wire        kbd_axi_wlast;
   wire        kbd_axi_wvalid;
   wire        kbd_axi_wready;
   wire [ 2:0] kbd_axi_bid;
   wire [ 1:0] kbd_axi_bresp;
   wire        kbd_axi_bvalid;
   wire        kbd_axi_bready;
   wire [ 2:0] kbd_axi_arid;
   wire [30:0] kbd_axi_araddr;
   wire [ 7:0] kbd_axi_arlen;
   wire [ 2:0] kbd_axi_arsize;
   wire [ 1:0] kbd_axi_arburst;
   wire        kbd_axi_arlock;
   wire [ 3:0] kbd_axi_arcache;
   wire [ 2:0] kbd_axi_arprot;
   wire [ 3:0] kbd_axi_arqos;
   wire        kbd_axi_arvalid;
   wire        kbd_axi_arready;
   wire [ 2:0] kbd_axi_rid;
   wire [63:0] kbd_axi_rdata;
   wire [ 1:0] kbd_axi_rresp;
   wire        kbd_axi_rlast;
   wire        kbd_axi_rvalid;
   wire        kbd_axi_rready;

   wire [ 2:0] periph_axi_awid;
   wire [30:0] periph_axi_awaddr;
   wire [ 7:0] periph_axi_awlen;
   wire [ 2:0] periph_axi_awsize;
   wire [ 1:0] periph_axi_awburst;
   wire        periph_axi_awlock;
   wire [ 3:0] periph_axi_awcache;
   wire [ 2:0] periph_axi_awprot;
   wire [ 3:0] periph_axi_awqos;
   wire        periph_axi_awvalid;
   wire        periph_axi_awready;
   wire [63:0] periph_axi_wdata;
   wire [ 7:0] periph_axi_wstrb;
   wire        periph_axi_wlast;
   wire        periph_axi_wvalid;
   wire        periph_axi_wready;
   wire [ 2:0] periph_axi_bid;
   wire [ 1:0] periph_axi_bresp;
   wire        periph_axi_bvalid;
   wire        periph_axi_bready;
   wire [ 2:0] periph_axi_arid;
   wire [30:0] periph_axi_araddr;
   wire [ 7:0] periph_axi_arlen;
   wire [ 2:0] periph_axi_arsize;
   wire [ 1:0] periph_axi_arburst;
   wire        periph_axi_arlock;
   wire [ 3:0] periph_axi_arcache;
   wire [ 2:0] periph_axi_arprot;
   wire [ 3:0] periph_axi_arqos;
   wire        periph_axi_arvalid;
   wire        periph_axi_arready;
   wire [ 2:0] periph_axi_rid;
   wire [63:0] periph_axi_rdata;
   wire [ 1:0] periph_axi_rresp;
   wire        periph_axi_rlast;
   wire        periph_axi_rvalid;
   wire        periph_axi_rready;

   wire [ 2:0] device_axi_awid;
   wire [30:0] device_axi_awaddr;
   wire [ 7:0] device_axi_awlen;
   wire [ 2:0] device_axi_awsize;
   wire [ 1:0] device_axi_awburst;
   wire        device_axi_awlock;
   wire [ 3:0] device_axi_awcache;
   wire [ 2:0] device_axi_awprot;
   wire [ 3:0] device_axi_awqos;
   wire        device_axi_awvalid;
   wire        device_axi_awready;
   wire [63:0] device_axi_wdata;
   wire [ 7:0] device_axi_wstrb;
   wire        device_axi_wlast;
   wire        device_axi_wvalid;
   wire        device_axi_wready;
   wire [ 2:0] device_axi_bid;
   wire [ 1:0] device_axi_bresp;
   wire        device_axi_bvalid;
   wire        device_axi_bready;
   wire [ 2:0] device_axi_arid;
   wire [30:0] device_axi_araddr;
   wire [ 7:0] device_axi_arlen;
   wire [ 2:0] device_axi_arsize;
   wire [ 1:0] device_axi_arburst;
   wire        device_axi_arlock;
   wire [ 3:0] device_axi_arcache;
   wire [ 2:0] device_axi_arprot;
   wire [ 3:0] device_axi_arqos;
   wire        device_axi_arvalid;
   wire        device_axi_arready;
   wire [ 2:0] device_axi_rid;
   wire [63:0] device_axi_rdata;
   wire [ 1:0] device_axi_rresp;
   wire        device_axi_rlast;
   wire        device_axi_rvalid;
   wire        device_axi_rready;
   reg         mmio_read_d1 = 0;
   reg         mmio_read_d2 = 0;
   reg  [31:0] mmio_readdata_q = 32'd0;
   reg  [ 6:0] rd_sel_q = 7'd0;   reg rd_dbg_q = 1'b0;
   reg  [31:0] rd_spi_q, rd_cd_q, rd_blk_q, rd_blkdb_q, rd_net_q, rd_netdb_q, rd_vga_q, rd_kbd_q, rd_bid_q;

   always @(posedge ui_clk) begin
      if (ui_cpu_reset) begin
         sd_cd_meta <= 1'b1;
         sd_cd_sync <= 1'b1;
         mmio_read_d1 <= 1'b0;
         mmio_read_d2 <= 1'b0;
         mmio_readdata_q <= 32'd0;
         spi_fast_half <= 16'd6;
      end else begin
         sd_cd_meta <= sd_cd;
         sd_cd_sync <= sd_cd_meta;
         mmio_read_d1 <= ui_mmio_read;
         mmio_read_d2 <= mmio_read_d1;
         if (ui_mmio_write && spi_speed_sel)
            spi_fast_half <= ui_mmio_writedata[15:0];
         // Two stages, using the cycle the response already waits (readdatavalid = d2): the
         // read cycle registers the page's select and that device's data; the next cycle muxes
         // the registered words. A one-stage priority chain from every device's register file
         // to mmio_readdata_q was the design's worst path at 333 MHz (-0.097 ns, from
         // virtio_net's queue_sel) once the keyboard and video pages joined it.
         if (ui_mmio_read) begin
            rd_sel_q <= {build_id_sel, kbd_sel, vga_sel, virtio_net_sel, virtio_blk_sel,
                         sd_cd_gpio_sel, spi_speed_sel};
            rd_dbg_q <= ui_mmio_address[11:8] == 4'hf;
            rd_spi_q <= {16'd0, spi_fast_half};       rd_cd_q    <= sd_cd_gpio_readdata;
            rd_blk_q <= virtio_blk_readdata;          rd_blkdb_q <= virtio_blk_debug_word;
            rd_net_q <= virtio_net_readdata;          rd_netdb_q <= virtio_net_debug_word;
            rd_vga_q <= vga_mmio_rdata;               rd_kbd_q   <= kbd_mmio_rdata;
            rd_bid_q <= build_id_readdata;
         end
         if (mmio_read_d1)
            mmio_readdata_q <= ({32{rd_sel_q[0]}} & rd_spi_q)
                             | ({32{rd_sel_q[1]}} & rd_cd_q)
                             | ({32{rd_sel_q[2]}} & (rd_dbg_q ? rd_blkdb_q : rd_blk_q))
                             | ({32{rd_sel_q[3]}} & (rd_dbg_q ? rd_netdb_q : rd_net_q))
                             | ({32{rd_sel_q[4]}} & rd_vga_q)
                             | ({32{rd_sel_q[5]}} & rd_kbd_q)
                             | ({32{rd_sel_q[6]}} & rd_bid_q);
      end
   end

   always @* begin
      case (ui_mmio_address[5:2])
        4'h0: build_id_readdata = 32'h534d4f4c; // "SMOL"
        4'h1: build_id_readdata = 32'h00000001;
        4'h2: build_id_readdata = BUILD_ID_STAMP[31:0];
        4'h3: build_id_readdata = BUILD_ID_STAMP[63:32];
        4'h4: build_id_readdata = BUILD_ID_GIT_COMMIT;
        4'h5: build_id_readdata = {31'd0, BUILD_ID_GIT_DIRTY};
        default: build_id_readdata = 32'd0;
      endcase
   end

   assign ui_mmio_readdatavalid = mmio_read_d2;
   assign ui_mmio_readdata = mmio_readdata_q;

   // The MMIO bridge's core side runs at probe_clk; its async FIFOs are the probe_clk<->ui_clk CDC.
   wire mmio_bridge_clk = probe_clk;   wire mmio_bridge_rst = probe_reset;
   smolrv64_mmio_clock_bridge mmio_clock_bridge_inst(
      .core_clock          (mmio_bridge_clk),
      .core_reset          (mmio_bridge_rst),
      .core_address        (core_mmio_address),
      .core_read           (core_mmio_read),
      .core_write          (core_mmio_write),
      .core_writedata      (core_mmio_writedata),
      .core_byteenable     (core_mmio_byteenable),
      .core_readdatavalid  (core_mmio_readdatavalid),
      .core_readdata       (core_mmio_readdata),
      .ui_clock            (ui_clk),
      .ui_reset            (ui_cpu_reset),
      .ui_address          (ui_mmio_address),
      .ui_read             (ui_mmio_read),
      .ui_write            (ui_mmio_write),
      .ui_writedata        (ui_mmio_writedata),
      .ui_byteenable       (ui_mmio_byteenable),
      .ui_readdatavalid    (ui_mmio_readdatavalid),
      .ui_readdata         (ui_mmio_readdata)
   );

`ifdef ILA_VIRTIO
   // Debug (ILA_VIRTIO=1): capture the virtio-mmio access stream on the bridge ui-side (ui_clk).
   // Trigger on a DeviceFeatures read (probe2 ui_read=1 & probe0 addr==0x010) to see whether the
   // preceding DeviceFeaturesSel write (probe1 ui_write=1, addr 0x014, probe3 wdata=1) reached the
   // device first, and what the device returns (probe4 ui_readdata) -- root-cause VERSION_1 -22.
   ila_virtio u_ila_virtio_bridge (
      .clk    (ui_clk),
      .probe0 (ui_mmio_address[11:0]),   // reg offset: DeviceFeatures=0x010, DeviceFeaturesSel=0x014
      .probe1 (ui_mmio_write),
      .probe2 (ui_mmio_read),
      .probe3 (ui_mmio_writedata),       // Sel value on a write
      .probe4 (ui_mmio_readdata),        // what virtio returns (expect 0x3 for Features w/ Sel=1)
      .probe5 (ui_mmio_readdatavalid)
   );
`endif

   /* The legacy mmc-spi controller (sd_spi_oc_tiny + sd_gpio CS) is gone; the
    * SD card is now driven in SPI mode by sd_spi_host inside virtio_blk_backend. */

   virtio_mmio #(
      .DEVICE_ID(32'd2), /* virtio-blk, native-SD backend below. */
      .QUEUE_NUM_MAX(32'd8)
   ) virtio_blk_inst(
      // virtio-blk config: capacity, a 64-bit count of 512-byte sectors, at offset 0.
      .config_read_data        (ui_mmio_address[11:2] == 10'h040 ? virtio_blk_capacity : 32'd0),
      .config_write            (),
      .config_offset           (),
      .config_write_data       (),
      .config_byteenable       (),
      .clock                   (ui_clk),
      .reset                   (ui_cpu_reset),
      .address                 (ui_mmio_address[11:0]),
      .read                    (ui_mmio_read && virtio_blk_sel),
      .read_data               (virtio_blk_readdata),
      .write                   (ui_mmio_write && virtio_blk_sel),
      .write_data              (ui_mmio_writedata),
      .byteenable              (ui_mmio_byteenable),
      .irq                     (virtio_blk_irq),
      .queue_notify_pulse      (virtio_blk_queue_notify_pulse),
      .queue_notify_value      (virtio_blk_queue_notify_value),
      .used_buffer_interrupt   (virtio_blk_used_buffer_interrupt),
      .config_change_interrupt (1'b0),
      .driver_features_0       (),
      .driver_features_1       (),
      .queue_num               (),
      .queue_ready             (),
      .queue_desc              (),
      .queue_driver            (),
      .queue_device            (),
      .queue0_num              (virtio_blk_queue0_num),
      .queue0_ready            (virtio_blk_queue0_ready),
      .queue0_desc             (virtio_blk_queue0_desc),
      .queue0_driver           (virtio_blk_queue0_driver),
      .queue0_device           (virtio_blk_queue0_device),
      .queue1_num              (),
      .queue1_ready            (),
      .queue1_desc             (),
      .queue1_driver           (),
      .queue1_device           (),
      .device_status           (virtio_blk_device_status)
   );

   virtio_blk #(
      .QUEUE_SIZE(32'd8),
      .SD_SLOW_HALF(16'd416),   /* ui_clk 333 MHz -> ~400 kHz SPI init */
      .SD_FAST_HALF(16'd40),    /*               -> ~4 MHz transfer (margin) */
      .SD_INIT_TICKS(16'd10)    /* init idle bytes */
   ) virtio_blk_backend(
      .clock                   (ui_clk),
      .reset                   (ui_cpu_reset),
      .queue_notify_pulse      (virtio_blk_queue_notify_pulse),
      .queue_notify_value      (virtio_blk_queue_notify_value),
      .queue_num               (virtio_blk_queue0_num),
      .queue_ready             (virtio_blk_queue0_ready),
      .queue_desc              (virtio_blk_queue0_desc),
      .queue_driver            (virtio_blk_queue0_driver),
      .queue_device            (virtio_blk_queue0_device),
      .device_status           (virtio_blk_device_status),
      .used_buffer_interrupt   (virtio_blk_used_buffer_interrupt),
      .capacity_sectors        (virtio_blk_capacity),
      .sd_fast_half            (spi_fast_half),
      .debug_sel               (ui_mmio_address[3:2]),
      .debug_word              (virtio_blk_debug_word),
      .dbg                     (virtio_blk_dbg),
      .sd_sck                  (blk_sck),
      .sd_mosi                 (blk_mosi),
      .sd_miso                 (blk_miso),
      .sd_cs_n                 (blk_cs_n),

      .m_axi_awid              (virtio_blk_axi_awid),
      .m_axi_awaddr            (virtio_blk_axi_awaddr),
      .m_axi_awlen             (virtio_blk_axi_awlen),
      .m_axi_awsize            (virtio_blk_axi_awsize),
      .m_axi_awburst           (virtio_blk_axi_awburst),
      .m_axi_awlock            (virtio_blk_axi_awlock),
      .m_axi_awcache           (virtio_blk_axi_awcache),
      .m_axi_awprot            (virtio_blk_axi_awprot),
      .m_axi_awqos             (virtio_blk_axi_awqos),
      .m_axi_awvalid           (virtio_blk_axi_awvalid),
      .m_axi_awready           (virtio_blk_axi_awready),
      .m_axi_wdata             (virtio_blk_axi_wdata),
      .m_axi_wstrb             (virtio_blk_axi_wstrb),
      .m_axi_wlast             (virtio_blk_axi_wlast),
      .m_axi_wvalid            (virtio_blk_axi_wvalid),
      .m_axi_wready            (virtio_blk_axi_wready),
      .m_axi_bid               (virtio_blk_axi_bid),
      .m_axi_bresp             (virtio_blk_axi_bresp),
      .m_axi_bvalid            (virtio_blk_axi_bvalid),
      .m_axi_bready            (virtio_blk_axi_bready),
      .m_axi_arid              (virtio_blk_axi_arid),
      .m_axi_araddr            (virtio_blk_axi_araddr),
      .m_axi_arlen             (virtio_blk_axi_arlen),
      .m_axi_arsize            (virtio_blk_axi_arsize),
      .m_axi_arburst           (virtio_blk_axi_arburst),
      .m_axi_arlock            (virtio_blk_axi_arlock),
      .m_axi_arcache           (virtio_blk_axi_arcache),
      .m_axi_arprot            (virtio_blk_axi_arprot),
      .m_axi_arqos             (virtio_blk_axi_arqos),
      .m_axi_arvalid           (virtio_blk_axi_arvalid),
      .m_axi_arready           (virtio_blk_axi_arready),
      .m_axi_rid               (virtio_blk_axi_rid),
      .m_axi_rdata             (virtio_blk_axi_rdata),
      .m_axi_rresp             (virtio_blk_axi_rresp),
      .m_axi_rlast             (virtio_blk_axi_rlast),
      .m_axi_rvalid            (virtio_blk_axi_rvalid),
      .m_axi_rready            (virtio_blk_axi_rready)
   );

`ifdef ILA_PARITY
   // Debug (ILA_PARITY=1): cache data-array integrity. TRIGGER on probe0 != 0 -- the
   // cycle a cache data array returned a word whose parity did not match what was
   // stored. With 4096-deep capture and the trigger positioned late, the window holds
   // the surrounding cache/fetch/LSU activity that produced the bad read.
   //   probe0 = {I$,D$} parity-error pulse (the trigger)
   //   probe1 = sticky/bank/addr snapshot of the FIRST failure (survives the pulse)
   //   probe2, probe3 = tied off (they carried the retired core's fetch PA and LSU state)
   ila_parity u_ila_parity (
      .clk    (probe_clk),
      .probe0 (probe_par_err),
      .probe1 (probe_par_dbg),
      .probe2 (64'd0),   // the retired core's fetch PA; rv_soc_top has no such output
      .probe3 (64'd0)    // the retired core's LSU state
   );
`endif

`ifdef ILA_DEV
   // Debug (ILA_DEV=1): capture the virtio_blk backend on ui_clk to see WHERE a block request wedges
   // (the IRQ ILA proved the device never raises the completion IRQ -> it's stuck mid-request).
   //   probe0 = virtio_blk_dbg[21:0]: [21:16]=state [15:9]=sd_spi_state [8]=sd_busy [7]=sd_done
   //            [6]=sd_error [5]=sd_ready [4]=dma_rsp_error [3:0]=sectors_left[3:0]
   //   probe1 = DMA AXI handshakes (is the DMA stalled on the DDR arbiter?)
   //   probe2 = SD SPI pins (is the SPI clock toggling = active, or idle?)
   ila_dev u_ila_dev (
      .clk    (ui_clk),
      .probe0 (virtio_blk_dbg),
      .probe1 ({virtio_blk_axi_awvalid, virtio_blk_axi_awready, virtio_blk_axi_wvalid,
                virtio_blk_axi_wready, virtio_blk_axi_wlast, virtio_blk_axi_bvalid,
                virtio_blk_axi_bready, virtio_blk_axi_arvalid, virtio_blk_axi_arready,
                virtio_blk_axi_rvalid, virtio_blk_axi_rready, virtio_blk_axi_rlast}),
      .probe2 ({blk_sck, blk_cs_n, blk_mosi, blk_miso})
   );
`endif

`ifndef NO_VIRTIO_NET
   virtio_mmio #(
      .DEVICE_ID(32'd1), /* Network device with a minimal TX-drop backend. */
      .QUEUE_NUM_MAX(32'd256), /* virtio-net needs > MAX_SKB_FRAGS+2 (=19) TX slots */
      .QUEUE_COUNT(32'd2)
   ) virtio_net_inst(
      .config_read_data        (32'd0),
      .config_write            (),
      .config_offset           (),
      .config_write_data       (),
      .config_byteenable       (),
      .clock                   (ui_clk),
      .reset                   (ui_cpu_reset),
      .address                 (ui_mmio_address[11:0]),
      .read                    (ui_mmio_read && virtio_net_sel),
      .read_data               (virtio_net_readdata),
      .write                   (ui_mmio_write && virtio_net_sel),
      .write_data              (ui_mmio_writedata),
      .byteenable              (ui_mmio_byteenable),
      .irq                     (virtio_net_irq),
      .queue_notify_pulse      (virtio_net_queue_notify_pulse),
      .queue_notify_value      (virtio_net_queue_notify_value),
      .used_buffer_interrupt   (virtio_net_used_buffer_interrupt),
      .config_change_interrupt (1'b0),
      .driver_features_0       (),
      .driver_features_1       (),
      .queue_num               (),
      .queue_ready             (),
      .queue_desc              (),
      .queue_driver            (),
      .queue_device            (),
      .queue0_num              (virtio_net_queue0_num),
      .queue0_ready            (virtio_net_queue0_ready),
      .queue0_desc             (virtio_net_queue0_desc),
      .queue0_driver           (virtio_net_queue0_driver),
      .queue0_device           (virtio_net_queue0_device),
      .queue1_num              (virtio_net_queue1_num),
      .queue1_ready            (virtio_net_queue1_ready),
      .queue1_desc             (virtio_net_queue1_desc),
      .queue1_driver           (virtio_net_queue1_driver),
      .queue1_device           (virtio_net_queue1_device),
      .device_status           (virtio_net_device_status)
   );

   virtio_net #(
      .QUEUE_SIZE(32'd256)    /* must match virtio_net_inst QUEUE_NUM_MAX */
   ) virtio_net_backend(
      .clock                   (ui_clk),
      .reset                   (ui_cpu_reset),
      .queue_notify_pulse      (virtio_net_queue_notify_pulse),
      .queue_notify_value      (virtio_net_queue_notify_value),
      .tx_queue_num            (virtio_net_queue1_num),
      .tx_queue_ready          (virtio_net_queue1_ready),
      .tx_queue_desc           (virtio_net_queue1_desc),
      .tx_queue_driver         (virtio_net_queue1_driver),
      .tx_queue_device         (virtio_net_queue1_device),
      .device_status           (virtio_net_device_status),
      .used_buffer_interrupt   (virtio_net_used_buffer_interrupt),
      .debug_status            (virtio_net_debug_status),
      .debug_notify_count      (virtio_net_debug_notify_count),
      .debug_read_avail_count  (virtio_net_debug_read_avail_count),
      .debug_empty_avail_count (virtio_net_debug_empty_avail_count),
      .debug_read_ring_count   (virtio_net_debug_read_ring_count),
      .debug_complete_count    (virtio_net_debug_complete_count),
      .debug_irq_count         (virtio_net_debug_irq_count),
      .debug_dma_error_count   (virtio_net_debug_dma_error_count),
      .debug_indices           (virtio_net_debug_indices),
      .debug_used_head         (virtio_net_debug_used_head),
      .debug_last_avail_word_lo(virtio_net_debug_last_avail_word_lo),
      .debug_last_avail_word_hi(virtio_net_debug_last_avail_word_hi),
      .debug_last_ring_word_lo (virtio_net_debug_last_ring_word_lo),
      .debug_last_ring_word_hi (virtio_net_debug_last_ring_word_hi),

      .tx_wr_en                (virtio_net_tx_wr_en),
      .tx_wr_addr              (virtio_net_tx_wr_addr),
      .tx_wr_data              (virtio_net_tx_wr_data),
      .tx_send                 (virtio_net_tx_send),
      .tx_send_len             (virtio_net_tx_send_len),
      .tx_busy                 (virtio_net_tx_busy),
      .debug_tx_frame_count    (virtio_net_debug_tx_frame_count),
      .debug_tx_last_len       (virtio_net_debug_tx_last_len),
      .debug_tx_desc_addr      (virtio_net_debug_tx_desc_addr),
      .debug_tx_desc_len       (virtio_net_debug_tx_desc_len),
      .rx_queue_num            (virtio_net_queue0_num),
      .rx_queue_ready          (virtio_net_queue0_ready),
      .rx_queue_desc           (virtio_net_queue0_desc),
      .rx_queue_driver         (virtio_net_queue0_driver),
      .rx_queue_device         (virtio_net_queue0_device),
      .rx_frame_valid          (eth_rx_frame_valid),
      .rx_frame_len            (eth_rx_frame_len),
      .rx_rd_addr              (eth_rx_rd_addr),
      .rx_rd_data              (eth_rx_rd_data),
      .rx_frame_ack            (eth_rx_frame_ack),
      .debug_rx_deliver_count  (virtio_net_debug_rx_deliver_count),
      .debug_rx_nobuf_count    (virtio_net_debug_rx_nobuf_count),

      .m_axi_awid              (virtio_net_axi_awid),
      .m_axi_awaddr            (virtio_net_axi_awaddr),
      .m_axi_awlen             (virtio_net_axi_awlen),
      .m_axi_awsize            (virtio_net_axi_awsize),
      .m_axi_awburst           (virtio_net_axi_awburst),
      .m_axi_awlock            (virtio_net_axi_awlock),
      .m_axi_awcache           (virtio_net_axi_awcache),
      .m_axi_awprot            (virtio_net_axi_awprot),
      .m_axi_awqos             (virtio_net_axi_awqos),
      .m_axi_awvalid           (virtio_net_axi_awvalid),
      .m_axi_awready           (virtio_net_axi_awready),
      .m_axi_wdata             (virtio_net_axi_wdata),
      .m_axi_wstrb             (virtio_net_axi_wstrb),
      .m_axi_wlast             (virtio_net_axi_wlast),
      .m_axi_wvalid            (virtio_net_axi_wvalid),
      .m_axi_wready            (virtio_net_axi_wready),
      .m_axi_bid               (virtio_net_axi_bid),
      .m_axi_bresp             (virtio_net_axi_bresp),
      .m_axi_bvalid            (virtio_net_axi_bvalid),
      .m_axi_bready            (virtio_net_axi_bready),
      .m_axi_arid              (virtio_net_axi_arid),
      .m_axi_araddr            (virtio_net_axi_araddr),
      .m_axi_arlen             (virtio_net_axi_arlen),
      .m_axi_arsize            (virtio_net_axi_arsize),
      .m_axi_arburst           (virtio_net_axi_arburst),
      .m_axi_arlock            (virtio_net_axi_arlock),
      .m_axi_arcache           (virtio_net_axi_arcache),
      .m_axi_arprot            (virtio_net_axi_arprot),
      .m_axi_arqos             (virtio_net_axi_arqos),
      .m_axi_arvalid           (virtio_net_axi_arvalid),
      .m_axi_arready           (virtio_net_axi_arready),
      .m_axi_rid               (virtio_net_axi_rid),
      .m_axi_rdata             (virtio_net_axi_rdata),
      .m_axi_rresp             (virtio_net_axi_rresp),
      .m_axi_rlast             (virtio_net_axi_rlast),
      .m_axi_rvalid            (virtio_net_axi_rvalid),
      .m_axi_rready            (virtio_net_axi_rready)
   );

   // ===== Ethernet MAC: virtio_net (ui_clk) <-> eth_tx_engine <-> RGMII =====
   // The MAC datapath runs in the PHY recovered RX clock (gmii_rx_clk), which
   // gmii_to_rgmii derives from eth_rxc.  eth_tx_engine bridges ui_clk to that
   // domain.  RX is deframed but not yet delivered to a virtqueue (see
   // VIRTIO_PLAN.md step 6); its counters go to the debug overlay for bring-up.
   wire        virtio_net_tx_wr_en;
   wire [10:0] virtio_net_tx_wr_addr;
   wire [ 7:0] virtio_net_tx_wr_data;
   wire        virtio_net_tx_send;
   wire [10:0] virtio_net_tx_send_len;
   wire        virtio_net_tx_busy;
   wire [31:0] virtio_net_debug_tx_frame_count;
   wire [31:0] virtio_net_debug_tx_last_len;
   wire [31:0] virtio_net_debug_tx_desc_addr;
   wire [31:0] virtio_net_debug_tx_desc_len;
   wire [31:0] virtio_net_debug_rx_deliver_count;
   wire [31:0] virtio_net_debug_rx_nobuf_count;

   wire        gmii_rx_clk;
   wire        gmii_rx_dv;
   wire [ 7:0] gmii_rxd;
   wire        gmii_tx_en;
   wire [ 7:0] gmii_txd;
   wire        eth_rx_valid;
   wire [ 7:0] eth_rx_data;
   wire        eth_rx_last;
   wire        eth_rx_good;
   // eth_rx_engine <-> backend (ui_clk)
   wire        eth_rx_frame_valid;
   wire [10:0] eth_rx_frame_len;
   wire [10:0] eth_rx_rd_addr;
   wire [ 7:0] eth_rx_rd_data;
   wire        eth_rx_frame_ack;
   wire [15:0] eth_rx_drop_count;             // engine: no free slot at a frame's first byte
   wire [15:0] eth_rx_bad_count, eth_rx_oflow_count;   // engine: FCS bad / longer than a slot

   // RX-capture MMCM supervision: MMCME4 does not reliably relock after its
   // input clock (the PHY's rxc) is interrupted -- link renegotiation -- without
   // a reset pulse.  If LOCKED stays low for ~1.5ms, pulse RST for 16 ui_clks
   // and retry periodically until it locks.
   wire eth_mmcm_locked;
   reg  [19:0] eth_mmcm_wd = 20'd0;
   reg         eth_mmcm_rst = 1'b0;
   (* async_reg = "true" *) reg [1:0] eth_lock_sync = 2'b00;
   always @(posedge ui_clk) begin
      eth_lock_sync <= {eth_lock_sync[0], eth_mmcm_locked};
      if (eth_lock_sync[1]) begin
         eth_mmcm_wd  <= 20'd0;
         eth_mmcm_rst <= 1'b0;
      end else begin
         eth_mmcm_wd  <= eth_mmcm_wd + 20'd1;
         eth_mmcm_rst <= &eth_mmcm_wd[19:4];   // top 16 counts of each lap
      end
   end

   // Reset for the gmii_rx_clk domain: synchronize the CPU reset in, and hold
   // the domain in reset until the capture MMCM locks.  If the PHY isn't
   // supplying rxc (no link) the domain simply stays in reset.
   wire gmii_arst = ui_cpu_reset | ~eth_mmcm_locked;
   (* async_reg = "true" *) reg [1:0] gmii_rst_sync = 2'b11;
   always @(posedge gmii_rx_clk or posedge gmii_arst)
      if (gmii_arst) gmii_rst_sync <= 2'b11;
      else           gmii_rst_sync <= {gmii_rst_sync[0], 1'b0};
   wire gmii_rst = gmii_rst_sync[1];

   gmii_to_rgmii gmii_to_rgmii_inst(
      .mmcm_rst     (eth_mmcm_rst),
      .mmcm_locked  (eth_mmcm_locked),
      .gmii_rx_clk  (gmii_rx_clk),
      .gmii_rx_dv   (gmii_rx_dv),
      .gmii_rxd     (gmii_rxd),
      .gmii_tx_clk  (),
      .gmii_tx_en   (gmii_tx_en),
      .gmii_txd     (gmii_txd),
      .rgmii_rxc    (eth_rxc),
      .rgmii_rx_ctl (eth_rx_ctl),
      .rgmii_rxd    (eth_rxd),
      .rgmii_txc    (eth_txc),
      .rgmii_tx_ctl (eth_tx_ctl),
      .rgmii_txd    (eth_txd)
   );

   eth_tx_engine #(.BUF_BYTES(1536)) eth_tx_engine_inst(
      .ui_clk     (ui_clk),
      .ui_rst     (ui_cpu_reset),
      .wr_en      (virtio_net_tx_wr_en),
      .wr_addr    (virtio_net_tx_wr_addr),
      .wr_data    (virtio_net_tx_wr_data),
      .send       (virtio_net_tx_send),
      .send_len   (virtio_net_tx_send_len),
      .busy       (virtio_net_tx_busy),
      .gmii_clk   (gmii_rx_clk),
      .gmii_rst   (gmii_rst),
      .gmii_tx_en (gmii_tx_en),
      .gmii_txd   (gmii_txd)
   );

   eth_mac_rx eth_mac_rx_inst(
      .clk        (gmii_rx_clk),
      .rst_n      (~gmii_rst),
      .gmii_rx_dv (gmii_rx_dv),
      .gmii_rxd   (gmii_rxd),
      .rx_valid   (eth_rx_valid),
      .rx_data    (eth_rx_data),
      .rx_last    (eth_rx_last),
      .rx_good    (eth_rx_good)
   );

   // RX engine buffers each good frame and hands it to the backend (ui_clk),
   // which DMAs it into a guest RX-queue buffer.
   // Eight 2 KiB slots in BRAM (2026-09-05): a gigabit burst is held, not truncated.
   eth_rx_engine #(.SLOTS(8), .SLOT_BYTES(2048)) eth_rx_engine_inst(
      .gmii_clk    (gmii_rx_clk),
      .gmii_rst    (gmii_rst),
      .rx_valid    (eth_rx_valid),
      .rx_data     (eth_rx_data),
      .rx_last     (eth_rx_last),
      .rx_good     (eth_rx_good),
      .ui_clk      (ui_clk),
      .ui_rst      (ui_cpu_reset),
      .frame_valid (eth_rx_frame_valid),
      .frame_len   (eth_rx_frame_len),
      .rd_addr     (eth_rx_rd_addr),
      .rd_data     (eth_rx_rd_data),
      .frame_ack   (eth_rx_frame_ack),
      .drop_count  (eth_rx_drop_count),
      .bad_count   (eth_rx_bad_count),
      .oflow_count (eth_rx_oflow_count)
   );

   // Passive good/bad frame counters (count every frame eth_mac_rx sees, incl
   // ones the single-buffered engine drops while busy) for the debug overlay.
   (* dont_touch = "true" *) reg [15:0] eth_rx_good_cnt = 16'd0;
   (* dont_touch = "true" *) reg [15:0] eth_rx_bad_cnt  = 16'd0;
   // Debug: capture the first 8 deframed bytes + length + FCS-good of the most
   // recent RX frame (good or bad), so devmem can compare against the known
   // host frame (e.g. ARP reply: first 6 bytes = board MAC 4a 18 30 e1 28 bc).
   // Garbage/shift => RGMII RX capture timing; correct bytes => deframer/FCS.
   (* dont_touch = "true" *) reg [63:0] rx_dbg_head = 64'd0;  // bytes 0..7, LE
   (* dont_touch = "true" *) reg [10:0] rx_dbg_len  = 11'd0;
   (* dont_touch = "true" *) reg        rx_dbg_good = 1'b0;
   reg [10:0] rx_dbg_cnt = 11'd0;
   always @(posedge gmii_rx_clk) begin
      if (eth_rx_valid) begin
         if (rx_dbg_cnt < 11'd8)
            rx_dbg_head[rx_dbg_cnt[2:0]*8 +: 8] <= eth_rx_data;
         rx_dbg_cnt <= rx_dbg_cnt + 11'd1;
      end
      if (eth_rx_last) begin
         rx_dbg_len  <= rx_dbg_cnt;
         rx_dbg_good <= eth_rx_good;
         rx_dbg_cnt  <= 11'd0;
         if (eth_rx_good) eth_rx_good_cnt <= eth_rx_good_cnt + 16'd1;
         else             eth_rx_bad_cnt  <= eth_rx_bad_cnt  + 16'd1;
      end
   end

   // Bring the RX counters into ui_clk for the debug overlay. Plain multi-bit
   // sampling: they change rarely (once per RX frame) and are stable between,
   // so a devmem read is coherent in practice (approximate during an increment).
   (* async_reg = "true" *) reg [15:0] eth_rx_good_ui0, eth_rx_good_ui;
   (* async_reg = "true" *) reg [15:0] eth_rx_bad_ui0,  eth_rx_bad_ui;
   (* async_reg = "true" *) reg [63:0] rx_dbg_head_ui0, rx_dbg_head_ui;
   (* async_reg = "true" *) reg [11:0] rx_dbg_lg_ui0, rx_dbg_lg_ui;  // {good, len}
   always @(posedge ui_clk) begin
      eth_rx_good_ui0 <= eth_rx_good_cnt; eth_rx_good_ui <= eth_rx_good_ui0;
      eth_rx_bad_ui0  <= eth_rx_bad_cnt;  eth_rx_bad_ui  <= eth_rx_bad_ui0;
      rx_dbg_head_ui0 <= rx_dbg_head;     rx_dbg_head_ui <= rx_dbg_head_ui0;
      rx_dbg_lg_ui0   <= {rx_dbg_good, rx_dbg_len};
      rx_dbg_lg_ui    <= rx_dbg_lg_ui0;
   end

   // Debug overlay: reads to 0x10003f00..f7c return TX/RX bring-up state
   // (devmem from Linux).  Word index = ui_mmio_address[7:2].
   always @(*) begin
      case (ui_mmio_address[7:2])
        6'd0:  virtio_net_debug_word = virtio_net_debug_status;
        6'd1:  virtio_net_debug_word = virtio_net_debug_notify_count;
        6'd2:  virtio_net_debug_word = virtio_net_debug_read_avail_count;
        6'd3:  virtio_net_debug_word = virtio_net_debug_empty_avail_count;
        6'd4:  virtio_net_debug_word = virtio_net_debug_read_ring_count;
        6'd5:  virtio_net_debug_word = virtio_net_debug_complete_count;
        6'd6:  virtio_net_debug_word = virtio_net_debug_irq_count;
        6'd7:  virtio_net_debug_word = virtio_net_debug_dma_error_count;
        6'd8:  virtio_net_debug_word = virtio_net_debug_indices;
        6'd9:  virtio_net_debug_word = virtio_net_debug_used_head;
        6'd10: virtio_net_debug_word = virtio_net_debug_last_avail_word_lo;
        6'd11: virtio_net_debug_word = virtio_net_debug_last_avail_word_hi;
        6'd12: virtio_net_debug_word = virtio_net_debug_last_ring_word_lo;
        6'd13: virtio_net_debug_word = virtio_net_debug_last_ring_word_hi;
        6'd14: virtio_net_debug_word = virtio_net_debug_tx_frame_count;
        6'd15: virtio_net_debug_word = virtio_net_debug_tx_last_len;
        6'd16: virtio_net_debug_word = virtio_net_debug_tx_desc_addr;
        6'd17: virtio_net_debug_word = virtio_net_debug_tx_desc_len;
        6'd18: virtio_net_debug_word = {eth_rx_bad_ui, eth_rx_good_ui};
        6'd19: virtio_net_debug_word = 32'h4554_4830; // "ETH0" sentinel
        6'd20: virtio_net_debug_word = virtio_net_debug_rx_deliver_count;
        6'd21: virtio_net_debug_word = virtio_net_debug_rx_nobuf_count;
        6'd22: virtio_net_debug_word = {16'd0, eth_rx_drop_count}; // engine busy-drops
        6'd26: virtio_net_debug_word = {eth_rx_oflow_count, eth_rx_bad_count}; // engine: too long / FCS bad
        6'd23: virtio_net_debug_word = rx_dbg_head_ui[31:0];   // RX bytes 0..3
        6'd24: virtio_net_debug_word = rx_dbg_head_ui[63:32];  // RX bytes 4..7
        6'd25: virtio_net_debug_word = {20'd0, rx_dbg_lg_ui};  // {good, len} of last RX
        default: virtio_net_debug_word = 32'd0;
      endcase
   end
`else
   // ================= NO_VIRTIO_NET =================
   // virtio-net + the RGMII eth MAC are compiled out to relieve ui_clk routing
   // congestion (the -0.57ns device-DMA cone + the router collapse sit in the
   // net-adjacent logic). Blk-only device DMA: hold the device arbiter's net s0
   // idle so it passes virtio-blk straight through; the net MMIO window reads 0 and
   // the net IRQ never fires, so Linux finds no net device (same as the sim TBs).
   assign virtio_net_readdata = 32'd0;
   assign virtio_net_irq      = 1'b0;
   always @(*) virtio_net_debug_word = 32'd0;
   assign eth_txc = 1'b0;  assign eth_txd = 4'd0;  assign eth_tx_ctl = 1'b0;
   assign virtio_net_axi_awid    = 3'd0;   assign virtio_net_axi_awaddr  = 31'd0;
   assign virtio_net_axi_awlen   = 8'd0;   assign virtio_net_axi_awsize  = 3'd0;
   assign virtio_net_axi_awburst = 2'd0;   assign virtio_net_axi_awlock  = 1'b0;
   assign virtio_net_axi_awcache = 4'd0;   assign virtio_net_axi_awprot  = 3'd0;
   assign virtio_net_axi_awqos   = 4'd0;   assign virtio_net_axi_awvalid = 1'b0;
   assign virtio_net_axi_wdata   = 64'd0;  assign virtio_net_axi_wstrb   = 8'd0;
   assign virtio_net_axi_wlast   = 1'b0;   assign virtio_net_axi_wvalid  = 1'b0;
   assign virtio_net_axi_bready  = 1'b1;
   assign virtio_net_axi_arid    = 3'd0;   assign virtio_net_axi_araddr  = 31'd0;
   assign virtio_net_axi_arlen   = 8'd0;   assign virtio_net_axi_arsize  = 3'd0;
   assign virtio_net_axi_arburst = 2'd0;   assign virtio_net_axi_arlock  = 1'b0;
   assign virtio_net_axi_arcache = 4'd0;   assign virtio_net_axi_arprot  = 3'd0;
   assign virtio_net_axi_arqos   = 4'd0;   assign virtio_net_axi_arvalid = 1'b0;
   assign virtio_net_axi_rready  = 1'b1;
`endif

   // ===== VGA scanout (simmerv's --graphics framebuffer on a TinyVGA adapter) =====
   // vga_scanout reads the RGB565 framebuffer that Linux's simplefb draws into and drives the
   // header pins RGB222. Its registers are page 0x05 (0x1000_5000), written by the ROM monitor
   // before Linux starts (workloads/ubuntu/ubuntu-boot.sh); Linux never touches them.
   //
   // The pixel clock: an MMCM fed from ui_clk, VCO 1000 MHz (333.33 / 2 * 6), CLKOUT0 / 25 =
   // 40 MHz -- VESA 800x600@60, vga_scanout's reset mode. Software changes the mode by
   // rewriting CLKOUT0's divider over DRP (tools/vga-mode.py): the VCO never moves, so the
   // MMCM's lock and filter settings, which depend only on M, stay valid. DCLK is ui_clk / 2
   // (DRP's limit is below ui_clk), from a BUFGCE_DIV, so ui_clk <-> DCLK is synchronous.
   wire        vga_pixclk_rst, vga_pixclk_locked;
   wire        vga_drp_req, vga_drp_we;   wire [6:0] vga_drp_addr;   wire [15:0] vga_drp_di;
   wire        vga_mmcm_fb, vga_pix_unbuf, vga_pix_clk, vga_drp_clk;
   wire        vga_drp_drdy;   wire [15:0] vga_drp_do;
   reg         vga_drp_ack = 1'b0, vga_drp_started = 1'b0, vga_drp_den = 1'b0;
   reg  [15:0] vga_drp_do_q = 16'd0;

   BUFGCE_DIV #(.BUFGCE_DIVIDE(2)) vga_drp_bufg (
      .I(ui_clk), .CE(1'b1), .CLR(1'b0), .O(vga_drp_clk));

   MMCME4_ADV #(
      .BANDWIDTH          ("OPTIMIZED"),
      .COMPENSATION       ("INTERNAL"),
      .CLKIN1_PERIOD      (3.000),
      .DIVCLK_DIVIDE      (2),
      .CLKFBOUT_MULT_F    (6.000),
      .CLKOUT0_DIVIDE_F   (25.000),
      .CLKOUT0_DUTY_CYCLE (0.5),
      .CLKOUT0_PHASE      (0.0)
   ) vga_mmcm_inst (
      .CLKIN1(ui_clk), .CLKIN2(1'b0), .CLKINSEL(1'b1),
      .CLKFBIN(vga_mmcm_fb), .CLKFBOUT(vga_mmcm_fb), .CLKFBOUTB(),
      .CLKOUT0(vga_pix_unbuf), .CLKOUT0B(), .CLKOUT1(), .CLKOUT1B(), .CLKOUT2(), .CLKOUT2B(),
      .CLKOUT3(), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6(),
      .DCLK(vga_drp_clk), .DEN(vga_drp_den), .DWE(vga_drp_we), .DADDR(vga_drp_addr),
      .DI(vga_drp_di), .DO(vga_drp_do), .DRDY(vga_drp_drdy),
      .PSCLK(1'b0), .PSEN(1'b0), .PSINCDEC(1'b0), .PSDONE(),
      .CDDCREQ(1'b0), .CDDCDONE(),
      .LOCKED(vga_pixclk_locked), .CLKINSTOPPED(), .CLKFBSTOPPED(),
      .PWRDWN(1'b0), .RST(vga_pixclk_rst | ui_cpu_reset));
   BUFG vga_pix_bufg (.I(vga_pix_unbuf), .O(vga_pix_clk));

   // DRP: one access per four-phase handshake with vga_scanout (see its header). DEN is a
   // one-cycle pulse; the address, data and DWE are vga_scanout's registers, held while
   // drp_req is high, and drp_do_q is captured before drp_ack rises.
   (* async_reg = "true" *) reg [1:0] vga_drp_req_s = 2'b00;
   always @(posedge vga_drp_clk) begin
      vga_drp_req_s <= {vga_drp_req_s[0], vga_drp_req};
      vga_drp_den   <= 1'b0;
      if (vga_drp_req_s[1] && !vga_drp_ack && !vga_drp_started) begin
         vga_drp_den <= 1'b1;  vga_drp_started <= 1'b1;
      end
      if (vga_drp_drdy) begin vga_drp_do_q <= vga_drp_do;  vga_drp_ack <= 1'b1; end
      if (!vga_drp_req_s[1]) begin vga_drp_ack <= 1'b0;  vga_drp_started <= 1'b0; end
   end

   (* async_reg = "true" *) reg [1:0] vga_pix_rst_s = 2'b11;
   always @(posedge vga_pix_clk) vga_pix_rst_s <= {vga_pix_rst_s[0], ~vga_pixclk_locked};

   vga_scanout vga_scanout_inst (
      .clk(ui_clk), .reset(ui_cpu_reset),
      .mmio_addr(ui_mmio_address[7:0]), .mmio_write(ui_mmio_write && vga_sel),
      .mmio_wdata(ui_mmio_writedata), .mmio_be(ui_mmio_byteenable), .mmio_rdata(vga_mmio_rdata),
      .m_axi_arid(vga_axi_arid), .m_axi_araddr(vga_axi_araddr), .m_axi_arlen(vga_axi_arlen),
      .m_axi_arsize(vga_axi_arsize), .m_axi_arburst(vga_axi_arburst), .m_axi_arlock(vga_axi_arlock),
      .m_axi_arcache(vga_axi_arcache), .m_axi_arprot(vga_axi_arprot), .m_axi_arqos(vga_axi_arqos),
      .m_axi_arvalid(vga_axi_arvalid), .m_axi_arready(vga_axi_arready),
      .m_axi_rid(vga_axi_rid), .m_axi_rdata(vga_axi_rdata), .m_axi_rresp(vga_axi_rresp),
      .m_axi_rlast(vga_axi_rlast), .m_axi_rvalid(vga_axi_rvalid), .m_axi_rready(vga_axi_rready),
      .pixclk_rst(vga_pixclk_rst), .pixclk_locked(vga_pixclk_locked),
      .drp_req(vga_drp_req), .drp_we(vga_drp_we), .drp_addr(vga_drp_addr), .drp_di(vga_drp_di),
      .drp_ack(vga_drp_ack), .drp_do(vga_drp_do_q),
      .pix_clk(vga_pix_clk), .pix_rst(vga_pix_rst_s[1]),
      .vga_hs(vga_hs), .vga_vs(vga_vs), .vga_r(vga_r), .vga_g(vga_g), .vga_b(vga_b));
   // The scanout only reads: its write channel is idle.
   assign vga_axi_awid = 3'd0;    assign vga_axi_awaddr = 31'd0;  assign vga_axi_awlen = 8'd0;
   assign vga_axi_awsize = 3'd0;  assign vga_axi_awburst = 2'd0;  assign vga_axi_awlock = 1'b0;
   assign vga_axi_awcache = 4'd0; assign vga_axi_awprot = 3'd0;   assign vga_axi_awqos = 4'd0;
   assign vga_axi_awvalid = 1'b0; assign vga_axi_wdata = 64'd0;   assign vga_axi_wstrb = 8'd0;
   assign vga_axi_wlast = 1'b0;   assign vga_axi_wvalid = 1'b0;   assign vga_axi_bready = 1'b1;

   // ===== The virtio keyboard (simmerv's --graphics keyboard), fed from the serial line =====
   // key[3] steers UART RX: each press toggles it between the console (the 16550, as always)
   // and this keyboard, whose bytes are translated into key presses (virtio_input's header).
   // It starts on the console at every reset, so loading over the UART is never affected.
   wire        kbd_irq;
   wire        kbd_byte_valid;   wire [7:0] kbd_byte;
   virtio_input kbd_inst (
      .clock(ui_clk), .reset(ui_cpu_reset),
      .address(ui_mmio_address[11:0]), .read(ui_mmio_read && kbd_sel), .read_data(kbd_mmio_rdata),
      .write(ui_mmio_write && kbd_sel), .write_data(ui_mmio_writedata), .byteenable(ui_mmio_byteenable),
      .irq(kbd_irq), .key_valid(kbd_byte_valid), .key_byte(kbd_byte),
      .m_axi_awid(kbd_axi_awid), .m_axi_awaddr(kbd_axi_awaddr), .m_axi_awlen(kbd_axi_awlen),
      .m_axi_awsize(kbd_axi_awsize), .m_axi_awburst(kbd_axi_awburst), .m_axi_awlock(kbd_axi_awlock),
      .m_axi_awcache(kbd_axi_awcache), .m_axi_awprot(kbd_axi_awprot), .m_axi_awqos(kbd_axi_awqos),
      .m_axi_awvalid(kbd_axi_awvalid), .m_axi_awready(kbd_axi_awready),
      .m_axi_wdata(kbd_axi_wdata), .m_axi_wstrb(kbd_axi_wstrb), .m_axi_wlast(kbd_axi_wlast),
      .m_axi_wvalid(kbd_axi_wvalid), .m_axi_wready(kbd_axi_wready),
      .m_axi_bid(kbd_axi_bid), .m_axi_bresp(kbd_axi_bresp), .m_axi_bvalid(kbd_axi_bvalid),
      .m_axi_bready(kbd_axi_bready),
      .m_axi_arid(kbd_axi_arid), .m_axi_araddr(kbd_axi_araddr), .m_axi_arlen(kbd_axi_arlen),
      .m_axi_arsize(kbd_axi_arsize), .m_axi_arburst(kbd_axi_arburst), .m_axi_arlock(kbd_axi_arlock),
      .m_axi_arcache(kbd_axi_arcache), .m_axi_arprot(kbd_axi_arprot), .m_axi_arqos(kbd_axi_arqos),
      .m_axi_arvalid(kbd_axi_arvalid), .m_axi_arready(kbd_axi_arready),
      .m_axi_rid(kbd_axi_rid), .m_axi_rdata(kbd_axi_rdata), .m_axi_rresp(kbd_axi_rresp),
      .m_axi_rlast(kbd_axi_rlast), .m_axi_rvalid(kbd_axi_rvalid), .m_axi_rready(kbd_axi_rready));

   generate
   if (USE_DDR_ARB) begin : gen_ddr_arbiter
   // Device-side arbiters, a balanced tree under the core-vs-device arbiter below:
   //    device_arbiter_inst  virtio-net (s0) + virtio-blk (s1)      stamps ID bit 0
   //    periph_arbiter_inst  VGA scanout (s0) + virtio keyboard (s1)  stamps ID bit 0
   //    device_top_inst      the two above                          stamps ID bit 1
   //    ddr4_arbiter_inst    core + devices                          stamps ID bit 2
   // Every device master therefore uses ID 0 (the IDs are 3 bits; three stamping levels over a
   // master with its own ID bit would need a fourth). Reusing the proven 2-master arbiter keeps
   // each one 2-input (timing-friendly) instead of growing a wider mux on the DDR path.
   axi_two_master_arbiter #(.STAMP(0)) device_arbiter_inst(
      .clock          (ui_clk),
      .reset          (ui_cpu_reset),

      .s0_axi_awid    (virtio_net_axi_awid),
      .s0_axi_awaddr  (virtio_net_axi_awaddr),
      .s0_axi_awlen   (virtio_net_axi_awlen),
      .s0_axi_awsize  (virtio_net_axi_awsize),
      .s0_axi_awburst (virtio_net_axi_awburst),
      .s0_axi_awlock  (virtio_net_axi_awlock),
      .s0_axi_awcache (virtio_net_axi_awcache),
      .s0_axi_awprot  (virtio_net_axi_awprot),
      .s0_axi_awqos   (virtio_net_axi_awqos),
      .s0_axi_awvalid (virtio_net_axi_awvalid),
      .s0_axi_awready (virtio_net_axi_awready),
      .s0_axi_wdata   (virtio_net_axi_wdata),
      .s0_axi_wstrb   (virtio_net_axi_wstrb),
      .s0_axi_wlast   (virtio_net_axi_wlast),
      .s0_axi_wvalid  (virtio_net_axi_wvalid),
      .s0_axi_wready  (virtio_net_axi_wready),
      .s0_axi_bid     (virtio_net_axi_bid),
      .s0_axi_bresp   (virtio_net_axi_bresp),
      .s0_axi_bvalid  (virtio_net_axi_bvalid),
      .s0_axi_bready  (virtio_net_axi_bready),
      .s0_axi_arid    (virtio_net_axi_arid),
      .s0_axi_araddr  (virtio_net_axi_araddr),
      .s0_axi_arlen   (virtio_net_axi_arlen),
      .s0_axi_arsize  (virtio_net_axi_arsize),
      .s0_axi_arburst (virtio_net_axi_arburst),
      .s0_axi_arlock  (virtio_net_axi_arlock),
      .s0_axi_arcache (virtio_net_axi_arcache),
      .s0_axi_arprot  (virtio_net_axi_arprot),
      .s0_axi_arqos   (virtio_net_axi_arqos),
      .s0_axi_arvalid (virtio_net_axi_arvalid),
      .s0_axi_arready (virtio_net_axi_arready),
      .s0_axi_rid     (virtio_net_axi_rid),
      .s0_axi_rdata   (virtio_net_axi_rdata),
      .s0_axi_rresp   (virtio_net_axi_rresp),
      .s0_axi_rlast   (virtio_net_axi_rlast),
      .s0_axi_rvalid  (virtio_net_axi_rvalid),
      .s0_axi_rready  (virtio_net_axi_rready),

      .s1_axi_awid    (virtio_blk_axi_awid),
      .s1_axi_awaddr  (virtio_blk_axi_awaddr),
      .s1_axi_awlen   (virtio_blk_axi_awlen),
      .s1_axi_awsize  (virtio_blk_axi_awsize),
      .s1_axi_awburst (virtio_blk_axi_awburst),
      .s1_axi_awlock  (virtio_blk_axi_awlock),
      .s1_axi_awcache (virtio_blk_axi_awcache),
      .s1_axi_awprot  (virtio_blk_axi_awprot),
      .s1_axi_awqos   (virtio_blk_axi_awqos),
      .s1_axi_awvalid (virtio_blk_axi_awvalid),
      .s1_axi_awready (virtio_blk_axi_awready),
      .s1_axi_wdata   (virtio_blk_axi_wdata),
      .s1_axi_wstrb   (virtio_blk_axi_wstrb),
      .s1_axi_wlast   (virtio_blk_axi_wlast),
      .s1_axi_wvalid  (virtio_blk_axi_wvalid),
      .s1_axi_wready  (virtio_blk_axi_wready),
      .s1_axi_bid     (virtio_blk_axi_bid),
      .s1_axi_bresp   (virtio_blk_axi_bresp),
      .s1_axi_bvalid  (virtio_blk_axi_bvalid),
      .s1_axi_bready  (virtio_blk_axi_bready),
      .s1_axi_arid    (virtio_blk_axi_arid),
      .s1_axi_araddr  (virtio_blk_axi_araddr),
      .s1_axi_arlen   (virtio_blk_axi_arlen),
      .s1_axi_arsize  (virtio_blk_axi_arsize),
      .s1_axi_arburst (virtio_blk_axi_arburst),
      .s1_axi_arlock  (virtio_blk_axi_arlock),
      .s1_axi_arcache (virtio_blk_axi_arcache),
      .s1_axi_arprot  (virtio_blk_axi_arprot),
      .s1_axi_arqos   (virtio_blk_axi_arqos),
      .s1_axi_arvalid (virtio_blk_axi_arvalid),
      .s1_axi_arready (virtio_blk_axi_arready),
      .s1_axi_rid     (virtio_blk_axi_rid),
      .s1_axi_rdata   (virtio_blk_axi_rdata),
      .s1_axi_rresp   (virtio_blk_axi_rresp),
      .s1_axi_rlast   (virtio_blk_axi_rlast),
      .s1_axi_rvalid  (virtio_blk_axi_rvalid),
      .s1_axi_rready  (virtio_blk_axi_rready),

      .m_axi_awid     (devab_axi_awid),
      .m_axi_awaddr   (devab_axi_awaddr),
      .m_axi_awlen    (devab_axi_awlen),
      .m_axi_awsize   (devab_axi_awsize),
      .m_axi_awburst  (devab_axi_awburst),
      .m_axi_awlock   (devab_axi_awlock),
      .m_axi_awcache  (devab_axi_awcache),
      .m_axi_awprot   (devab_axi_awprot),
      .m_axi_awqos    (devab_axi_awqos),
      .m_axi_awvalid  (devab_axi_awvalid),
      .m_axi_awready  (devab_axi_awready),
      .m_axi_wdata    (devab_axi_wdata),
      .m_axi_wstrb    (devab_axi_wstrb),
      .m_axi_wlast    (devab_axi_wlast),
      .m_axi_wvalid   (devab_axi_wvalid),
      .m_axi_wready   (devab_axi_wready),
      .m_axi_bid      (devab_axi_bid),
      .m_axi_bresp    (devab_axi_bresp),
      .m_axi_bvalid   (devab_axi_bvalid),
      .m_axi_bready   (devab_axi_bready),
      .m_axi_arid     (devab_axi_arid),
      .m_axi_araddr   (devab_axi_araddr),
      .m_axi_arlen    (devab_axi_arlen),
      .m_axi_arsize   (devab_axi_arsize),
      .m_axi_arburst  (devab_axi_arburst),
      .m_axi_arlock   (devab_axi_arlock),
      .m_axi_arcache  (devab_axi_arcache),
      .m_axi_arprot   (devab_axi_arprot),
      .m_axi_arqos    (devab_axi_arqos),
      .m_axi_arvalid  (devab_axi_arvalid),
      .m_axi_arready  (devab_axi_arready),
      .m_axi_rid      (devab_axi_rid),
      .m_axi_rdata    (devab_axi_rdata),
      .m_axi_rresp    (devab_axi_rresp),
      .m_axi_rlast    (devab_axi_rlast),
      .m_axi_rvalid   (devab_axi_rvalid),
      .m_axi_rready   (devab_axi_rready)
   );


   axi_two_master_arbiter #(.STAMP(0)) periph_arbiter_inst(
      .clock          (ui_clk),
      .reset          (ui_cpu_reset),

      .s0_axi_awid     (vga_axi_awid),
      .s0_axi_awaddr   (vga_axi_awaddr),
      .s0_axi_awlen    (vga_axi_awlen),
      .s0_axi_awsize   (vga_axi_awsize),
      .s0_axi_awburst  (vga_axi_awburst),
      .s0_axi_awlock   (vga_axi_awlock),
      .s0_axi_awcache  (vga_axi_awcache),
      .s0_axi_awprot   (vga_axi_awprot),
      .s0_axi_awqos    (vga_axi_awqos),
      .s0_axi_awvalid  (vga_axi_awvalid),
      .s0_axi_awready  (vga_axi_awready),
      .s0_axi_wdata    (vga_axi_wdata),
      .s0_axi_wstrb    (vga_axi_wstrb),
      .s0_axi_wlast    (vga_axi_wlast),
      .s0_axi_wvalid   (vga_axi_wvalid),
      .s0_axi_wready   (vga_axi_wready),
      .s0_axi_bid      (vga_axi_bid),
      .s0_axi_bresp    (vga_axi_bresp),
      .s0_axi_bvalid   (vga_axi_bvalid),
      .s0_axi_bready   (vga_axi_bready),
      .s0_axi_arid     (vga_axi_arid),
      .s0_axi_araddr   (vga_axi_araddr),
      .s0_axi_arlen    (vga_axi_arlen),
      .s0_axi_arsize   (vga_axi_arsize),
      .s0_axi_arburst  (vga_axi_arburst),
      .s0_axi_arlock   (vga_axi_arlock),
      .s0_axi_arcache  (vga_axi_arcache),
      .s0_axi_arprot   (vga_axi_arprot),
      .s0_axi_arqos    (vga_axi_arqos),
      .s0_axi_arvalid  (vga_axi_arvalid),
      .s0_axi_arready  (vga_axi_arready),
      .s0_axi_rid      (vga_axi_rid),
      .s0_axi_rdata    (vga_axi_rdata),
      .s0_axi_rresp    (vga_axi_rresp),
      .s0_axi_rlast    (vga_axi_rlast),
      .s0_axi_rvalid   (vga_axi_rvalid),
      .s0_axi_rready   (vga_axi_rready),

      .s1_axi_awid     (kbd_axi_awid),
      .s1_axi_awaddr   (kbd_axi_awaddr),
      .s1_axi_awlen    (kbd_axi_awlen),
      .s1_axi_awsize   (kbd_axi_awsize),
      .s1_axi_awburst  (kbd_axi_awburst),
      .s1_axi_awlock   (kbd_axi_awlock),
      .s1_axi_awcache  (kbd_axi_awcache),
      .s1_axi_awprot   (kbd_axi_awprot),
      .s1_axi_awqos    (kbd_axi_awqos),
      .s1_axi_awvalid  (kbd_axi_awvalid),
      .s1_axi_awready  (kbd_axi_awready),
      .s1_axi_wdata    (kbd_axi_wdata),
      .s1_axi_wstrb    (kbd_axi_wstrb),
      .s1_axi_wlast    (kbd_axi_wlast),
      .s1_axi_wvalid   (kbd_axi_wvalid),
      .s1_axi_wready   (kbd_axi_wready),
      .s1_axi_bid      (kbd_axi_bid),
      .s1_axi_bresp    (kbd_axi_bresp),
      .s1_axi_bvalid   (kbd_axi_bvalid),
      .s1_axi_bready   (kbd_axi_bready),
      .s1_axi_arid     (kbd_axi_arid),
      .s1_axi_araddr   (kbd_axi_araddr),
      .s1_axi_arlen    (kbd_axi_arlen),
      .s1_axi_arsize   (kbd_axi_arsize),
      .s1_axi_arburst  (kbd_axi_arburst),
      .s1_axi_arlock   (kbd_axi_arlock),
      .s1_axi_arcache  (kbd_axi_arcache),
      .s1_axi_arprot   (kbd_axi_arprot),
      .s1_axi_arqos    (kbd_axi_arqos),
      .s1_axi_arvalid  (kbd_axi_arvalid),
      .s1_axi_arready  (kbd_axi_arready),
      .s1_axi_rid      (kbd_axi_rid),
      .s1_axi_rdata    (kbd_axi_rdata),
      .s1_axi_rresp    (kbd_axi_rresp),
      .s1_axi_rlast    (kbd_axi_rlast),
      .s1_axi_rvalid   (kbd_axi_rvalid),
      .s1_axi_rready   (kbd_axi_rready),

      .m_axi_awid     (periph_axi_awid),
      .m_axi_awaddr   (periph_axi_awaddr),
      .m_axi_awlen    (periph_axi_awlen),
      .m_axi_awsize   (periph_axi_awsize),
      .m_axi_awburst  (periph_axi_awburst),
      .m_axi_awlock   (periph_axi_awlock),
      .m_axi_awcache  (periph_axi_awcache),
      .m_axi_awprot   (periph_axi_awprot),
      .m_axi_awqos    (periph_axi_awqos),
      .m_axi_awvalid  (periph_axi_awvalid),
      .m_axi_awready  (periph_axi_awready),
      .m_axi_wdata    (periph_axi_wdata),
      .m_axi_wstrb    (periph_axi_wstrb),
      .m_axi_wlast    (periph_axi_wlast),
      .m_axi_wvalid   (periph_axi_wvalid),
      .m_axi_wready   (periph_axi_wready),
      .m_axi_bid      (periph_axi_bid),
      .m_axi_bresp    (periph_axi_bresp),
      .m_axi_bvalid   (periph_axi_bvalid),
      .m_axi_bready   (periph_axi_bready),
      .m_axi_arid     (periph_axi_arid),
      .m_axi_araddr   (periph_axi_araddr),
      .m_axi_arlen    (periph_axi_arlen),
      .m_axi_arsize   (periph_axi_arsize),
      .m_axi_arburst  (periph_axi_arburst),
      .m_axi_arlock   (periph_axi_arlock),
      .m_axi_arcache  (periph_axi_arcache),
      .m_axi_arprot   (periph_axi_arprot),
      .m_axi_arqos    (periph_axi_arqos),
      .m_axi_arvalid  (periph_axi_arvalid),
      .m_axi_arready  (periph_axi_arready),
      .m_axi_rid      (periph_axi_rid),
      .m_axi_rdata    (periph_axi_rdata),
      .m_axi_rresp    (periph_axi_rresp),
      .m_axi_rlast    (periph_axi_rlast),
      .m_axi_rvalid   (periph_axi_rvalid),
      .m_axi_rready   (periph_axi_rready)
   );


   axi_two_master_arbiter #(.STAMP(1)) device_top_inst(
      .clock          (ui_clk),
      .reset          (ui_cpu_reset),

      .s0_axi_awid     (devab_axi_awid),
      .s0_axi_awaddr   (devab_axi_awaddr),
      .s0_axi_awlen    (devab_axi_awlen),
      .s0_axi_awsize   (devab_axi_awsize),
      .s0_axi_awburst  (devab_axi_awburst),
      .s0_axi_awlock   (devab_axi_awlock),
      .s0_axi_awcache  (devab_axi_awcache),
      .s0_axi_awprot   (devab_axi_awprot),
      .s0_axi_awqos    (devab_axi_awqos),
      .s0_axi_awvalid  (devab_axi_awvalid),
      .s0_axi_awready  (devab_axi_awready),
      .s0_axi_wdata    (devab_axi_wdata),
      .s0_axi_wstrb    (devab_axi_wstrb),
      .s0_axi_wlast    (devab_axi_wlast),
      .s0_axi_wvalid   (devab_axi_wvalid),
      .s0_axi_wready   (devab_axi_wready),
      .s0_axi_bid      (devab_axi_bid),
      .s0_axi_bresp    (devab_axi_bresp),
      .s0_axi_bvalid   (devab_axi_bvalid),
      .s0_axi_bready   (devab_axi_bready),
      .s0_axi_arid     (devab_axi_arid),
      .s0_axi_araddr   (devab_axi_araddr),
      .s0_axi_arlen    (devab_axi_arlen),
      .s0_axi_arsize   (devab_axi_arsize),
      .s0_axi_arburst  (devab_axi_arburst),
      .s0_axi_arlock   (devab_axi_arlock),
      .s0_axi_arcache  (devab_axi_arcache),
      .s0_axi_arprot   (devab_axi_arprot),
      .s0_axi_arqos    (devab_axi_arqos),
      .s0_axi_arvalid  (devab_axi_arvalid),
      .s0_axi_arready  (devab_axi_arready),
      .s0_axi_rid      (devab_axi_rid),
      .s0_axi_rdata    (devab_axi_rdata),
      .s0_axi_rresp    (devab_axi_rresp),
      .s0_axi_rlast    (devab_axi_rlast),
      .s0_axi_rvalid   (devab_axi_rvalid),
      .s0_axi_rready   (devab_axi_rready),

      .s1_axi_awid     (periph_axi_awid),
      .s1_axi_awaddr   (periph_axi_awaddr),
      .s1_axi_awlen    (periph_axi_awlen),
      .s1_axi_awsize   (periph_axi_awsize),
      .s1_axi_awburst  (periph_axi_awburst),
      .s1_axi_awlock   (periph_axi_awlock),
      .s1_axi_awcache  (periph_axi_awcache),
      .s1_axi_awprot   (periph_axi_awprot),
      .s1_axi_awqos    (periph_axi_awqos),
      .s1_axi_awvalid  (periph_axi_awvalid),
      .s1_axi_awready  (periph_axi_awready),
      .s1_axi_wdata    (periph_axi_wdata),
      .s1_axi_wstrb    (periph_axi_wstrb),
      .s1_axi_wlast    (periph_axi_wlast),
      .s1_axi_wvalid   (periph_axi_wvalid),
      .s1_axi_wready   (periph_axi_wready),
      .s1_axi_bid      (periph_axi_bid),
      .s1_axi_bresp    (periph_axi_bresp),
      .s1_axi_bvalid   (periph_axi_bvalid),
      .s1_axi_bready   (periph_axi_bready),
      .s1_axi_arid     (periph_axi_arid),
      .s1_axi_araddr   (periph_axi_araddr),
      .s1_axi_arlen    (periph_axi_arlen),
      .s1_axi_arsize   (periph_axi_arsize),
      .s1_axi_arburst  (periph_axi_arburst),
      .s1_axi_arlock   (periph_axi_arlock),
      .s1_axi_arcache  (periph_axi_arcache),
      .s1_axi_arprot   (periph_axi_arprot),
      .s1_axi_arqos    (periph_axi_arqos),
      .s1_axi_arvalid  (periph_axi_arvalid),
      .s1_axi_arready  (periph_axi_arready),
      .s1_axi_rid      (periph_axi_rid),
      .s1_axi_rdata    (periph_axi_rdata),
      .s1_axi_rresp    (periph_axi_rresp),
      .s1_axi_rlast    (periph_axi_rlast),
      .s1_axi_rvalid   (periph_axi_rvalid),
      .s1_axi_rready   (periph_axi_rready),

      .m_axi_awid     (device_axi_awid),
      .m_axi_awaddr   (device_axi_awaddr),
      .m_axi_awlen    (device_axi_awlen),
      .m_axi_awsize   (device_axi_awsize),
      .m_axi_awburst  (device_axi_awburst),
      .m_axi_awlock   (device_axi_awlock),
      .m_axi_awcache  (device_axi_awcache),
      .m_axi_awprot   (device_axi_awprot),
      .m_axi_awqos    (device_axi_awqos),
      .m_axi_awvalid  (device_axi_awvalid),
      .m_axi_awready  (device_axi_awready),
      .m_axi_wdata    (device_axi_wdata),
      .m_axi_wstrb    (device_axi_wstrb),
      .m_axi_wlast    (device_axi_wlast),
      .m_axi_wvalid   (device_axi_wvalid),
      .m_axi_wready   (device_axi_wready),
      .m_axi_bid      (device_axi_bid),
      .m_axi_bresp    (device_axi_bresp),
      .m_axi_bvalid   (device_axi_bvalid),
      .m_axi_bready   (device_axi_bready),
      .m_axi_arid     (device_axi_arid),
      .m_axi_araddr   (device_axi_araddr),
      .m_axi_arlen    (device_axi_arlen),
      .m_axi_arsize   (device_axi_arsize),
      .m_axi_arburst  (device_axi_arburst),
      .m_axi_arlock   (device_axi_arlock),
      .m_axi_arcache  (device_axi_arcache),
      .m_axi_arprot   (device_axi_arprot),
      .m_axi_arqos    (device_axi_arqos),
      .m_axi_arvalid  (device_axi_arvalid),
      .m_axi_arready  (device_axi_arready),
      .m_axi_rid      (device_axi_rid),
      .m_axi_rdata    (device_axi_rdata),
      .m_axi_rresp    (device_axi_rresp),
      .m_axi_rlast    (device_axi_rlast),
      .m_axi_rvalid   (device_axi_rvalid),
      .m_axi_rready   (device_axi_rready)
   );

   // ddr4 arbiter s0 (core) R channel goes through a register slice below, so
   // core_axi_r* to probe_bridge is registered (breaks the virtio-FSM -> grant
   // -> 512-bit capture-enable cross-die path). Intermediate = arbiter output.
   wire [2:0]  core_axi_rid_arb;
   wire [63:0] core_axi_rdata_arb;
   wire [1:0]  core_axi_rresp_arb;
   wire        core_axi_rlast_arb;
   wire        core_axi_rvalid_arb;
   wire        core_axi_rready_arb;

   axi_two_master_arbiter ddr4_arbiter_inst(
      .clock          (ui_clk),
      .reset          (ui_cpu_reset),

      .s0_axi_awid    (core_axi_awid),
      .s0_axi_awaddr  (core_axi_awaddr),
      .s0_axi_awlen   (core_axi_awlen),
      .s0_axi_awsize  (core_axi_awsize),
      .s0_axi_awburst (core_axi_awburst),
      .s0_axi_awlock  (core_axi_awlock),
      .s0_axi_awcache (core_axi_awcache),
      .s0_axi_awprot  (core_axi_awprot),
      .s0_axi_awqos   (core_axi_awqos),
      .s0_axi_awvalid (core_axi_awvalid),
      .s0_axi_awready (core_axi_awready),
      .s0_axi_wdata   (core_axi_wdata),
      .s0_axi_wstrb   (core_axi_wstrb),
      .s0_axi_wlast   (core_axi_wlast),
      .s0_axi_wvalid  (core_axi_wvalid),
      .s0_axi_wready  (core_axi_wready),
      .s0_axi_bid     (core_axi_bid),
      .s0_axi_bresp   (core_axi_bresp),
      .s0_axi_bvalid  (core_axi_bvalid),
      .s0_axi_bready  (core_axi_bready),
      .s0_axi_arid    (core_axi_arid),
      .s0_axi_araddr  (core_axi_araddr),
      .s0_axi_arlen   (core_axi_arlen),
      .s0_axi_arsize  (core_axi_arsize),
      .s0_axi_arburst (core_axi_arburst),
      .s0_axi_arlock  (core_axi_arlock),
      .s0_axi_arcache (core_axi_arcache),
      .s0_axi_arprot  (core_axi_arprot),
      .s0_axi_arqos   (core_axi_arqos),
      .s0_axi_arvalid (core_axi_arvalid),
      .s0_axi_arready (core_axi_arready),
      .s0_axi_rid     (core_axi_rid_arb),
      .s0_axi_rdata   (core_axi_rdata_arb),
      .s0_axi_rresp   (core_axi_rresp_arb),
      .s0_axi_rlast   (core_axi_rlast_arb),
      .s0_axi_rvalid  (core_axi_rvalid_arb),
      .s0_axi_rready  (core_axi_rready_arb),

      .s1_axi_awid    (device_axi_awid),
      .s1_axi_awaddr  (device_axi_awaddr),
      .s1_axi_awlen   (device_axi_awlen),
      .s1_axi_awsize  (device_axi_awsize),
      .s1_axi_awburst (device_axi_awburst),
      .s1_axi_awlock  (device_axi_awlock),
      .s1_axi_awcache (device_axi_awcache),
      .s1_axi_awprot  (device_axi_awprot),
      .s1_axi_awqos   (device_axi_awqos),
      .s1_axi_awvalid (device_axi_awvalid),
      .s1_axi_awready (device_axi_awready),
      .s1_axi_wdata   (device_axi_wdata),
      .s1_axi_wstrb   (device_axi_wstrb),
      .s1_axi_wlast   (device_axi_wlast),
      .s1_axi_wvalid  (device_axi_wvalid),
      .s1_axi_wready  (device_axi_wready),
      .s1_axi_bid     (device_axi_bid),
      .s1_axi_bresp   (device_axi_bresp),
      .s1_axi_bvalid  (device_axi_bvalid),
      .s1_axi_bready  (device_axi_bready),
      .s1_axi_arid    (device_axi_arid),
      .s1_axi_araddr  (device_axi_araddr),
      .s1_axi_arlen   (device_axi_arlen),
      .s1_axi_arsize  (device_axi_arsize),
      .s1_axi_arburst (device_axi_arburst),
      .s1_axi_arlock  (device_axi_arlock),
      .s1_axi_arcache (device_axi_arcache),
      .s1_axi_arprot  (device_axi_arprot),
      .s1_axi_arqos   (device_axi_arqos),
      .s1_axi_arvalid (device_axi_arvalid),
      .s1_axi_arready (device_axi_arready),
      .s1_axi_rid     (device_axi_rid),
      .s1_axi_rdata   (device_axi_rdata),
      .s1_axi_rresp   (device_axi_rresp),
      .s1_axi_rlast   (device_axi_rlast),
      .s1_axi_rvalid  (device_axi_rvalid),
      .s1_axi_rready  (device_axi_rready),

      .m_axi_awid     (m_axi_awid),
      .m_axi_awaddr   (m_axi_awaddr),
      .m_axi_awlen    (m_axi_awlen),
      .m_axi_awsize   (m_axi_awsize),
      .m_axi_awburst  (m_axi_awburst),
      .m_axi_awlock   (m_axi_awlock),
      .m_axi_awcache  (m_axi_awcache),
      .m_axi_awprot   (m_axi_awprot),
      .m_axi_awqos    (m_axi_awqos),
      .m_axi_awvalid  (m_axi_awvalid),
      .m_axi_awready  (m_axi_awready),
      .m_axi_wdata    (m_axi_wdata),
      .m_axi_wstrb    (m_axi_wstrb),
      .m_axi_wlast    (m_axi_wlast),
      .m_axi_wvalid   (m_axi_wvalid),
      .m_axi_wready   (m_axi_wready),
      .m_axi_bid      (m_axi_bid),
      .m_axi_bresp    (m_axi_bresp),
      .m_axi_bvalid   (m_axi_bvalid),
      .m_axi_bready   (m_axi_bready),
      .m_axi_arid     (m_axi_arid),
      .m_axi_araddr   (m_axi_araddr),
      .m_axi_arlen    (m_axi_arlen),
      .m_axi_arsize   (m_axi_arsize),
      .m_axi_arburst  (m_axi_arburst),
      .m_axi_arlock   (m_axi_arlock),
      .m_axi_arcache  (m_axi_arcache),
      .m_axi_arprot   (m_axi_arprot),
      .m_axi_arqos    (m_axi_arqos),
      .m_axi_arvalid  (m_axi_arvalid),
      .m_axi_arready  (m_axi_arready),
      .m_axi_rid      (m_axi_rid),
      .m_axi_rdata    (m_axi_rdata),
      .m_axi_rresp    (m_axi_rresp),
      .m_axi_rlast    (m_axi_rlast),
      .m_axi_rvalid   (m_axi_rvalid),
      .m_axi_rready   (m_axi_rready)
   );

   // Register slice on the core-side read response: arbiter s0 R (_arb) -> core_axi_r*.
   // AW/W skid into the MIG: breaks the arbiter->upsizer setup cones (-0.57ns class),
   // same recipe as the R slice below. AR passes through (read-address cone was clean).
   axi_aww_reg_slice #(.IDW(3), .AW(31), .DW(64)) mig_aww_slice (
      .clock(ui_clk), .reset(ui_cpu_reset),
      .s_awid(m_axi_awid), .s_awaddr(m_axi_awaddr), .s_awlen(m_axi_awlen), .s_awsize(m_axi_awsize),
      .s_awburst(m_axi_awburst), .s_awlock(m_axi_awlock), .s_awcache(m_axi_awcache),
      .s_awprot(m_axi_awprot), .s_awqos(m_axi_awqos), .s_awvalid(m_axi_awvalid), .s_awready(m_axi_awready),
      .s_wdata(m_axi_wdata), .s_wstrb(m_axi_wstrb), .s_wlast(m_axi_wlast),
      .s_wvalid(m_axi_wvalid), .s_wready(m_axi_wready),
      .m_awid(mig_awid), .m_awaddr(mig_awaddr), .m_awlen(mig_awlen), .m_awsize(mig_awsize),
      .m_awburst(mig_awburst), .m_awlock(mig_awlock), .m_awcache(mig_awcache),
      .m_awprot(mig_awprot), .m_awqos(mig_awqos), .m_awvalid(mig_awvalid), .m_awready(mig_awready),
      .m_wdata(mig_wdata), .m_wstrb(mig_wstrb), .m_wlast(mig_wlast),
      .m_wvalid(mig_wvalid), .m_wready(mig_wready));

   axi_r_reg_slice #(.IDW(3), .DW(64)) core_r_slice (
      .clock    (ui_clk),        .reset    (ui_cpu_reset),
      .s_rid    (core_axi_rid_arb),   .s_rdata (core_axi_rdata_arb),
      .s_rresp  (core_axi_rresp_arb), .s_rlast (core_axi_rlast_arb),
      .s_rvalid (core_axi_rvalid_arb), .s_rready (core_axi_rready_arb),
      .m_rid    (core_axi_rid),       .m_rdata (core_axi_rdata),
      .m_rresp  (core_axi_rresp),     .m_rlast (core_axi_rlast),
      .m_rvalid (core_axi_rvalid),    .m_rready (core_axi_rready));
   end else begin : gen_ddr_direct
      assign m_axi_awid        = core_axi_awid;
      assign m_axi_awaddr      = core_axi_awaddr;
      assign m_axi_awlen       = core_axi_awlen;
      assign m_axi_awsize      = core_axi_awsize;
      assign m_axi_awburst     = core_axi_awburst;
      assign m_axi_awlock      = core_axi_awlock;
      assign m_axi_awcache     = core_axi_awcache;
      assign m_axi_awprot      = core_axi_awprot;
      assign m_axi_awqos       = core_axi_awqos;
      assign m_axi_awvalid     = core_axi_awvalid;
      assign core_axi_awready  = m_axi_awready;

      assign m_axi_wdata       = core_axi_wdata;
      assign m_axi_wstrb       = core_axi_wstrb;
      assign m_axi_wlast       = core_axi_wlast;
      assign m_axi_wvalid      = core_axi_wvalid;
      assign core_axi_wready   = m_axi_wready;

      assign core_axi_bid      = m_axi_bid;
      assign core_axi_bresp    = m_axi_bresp;
      assign core_axi_bvalid   = m_axi_bvalid;
      assign m_axi_bready      = core_axi_bready;

      assign m_axi_arid        = core_axi_arid;
      assign m_axi_araddr      = core_axi_araddr;
      assign m_axi_arlen       = core_axi_arlen;
      assign m_axi_arsize      = core_axi_arsize;
      assign m_axi_arburst     = core_axi_arburst;
      assign m_axi_arlock      = core_axi_arlock;
      assign m_axi_arcache     = core_axi_arcache;
      assign m_axi_arprot      = core_axi_arprot;
      assign m_axi_arqos       = core_axi_arqos;
      assign m_axi_arvalid     = core_axi_arvalid;
      assign core_axi_arready  = m_axi_arready;

      assign core_axi_rid      = m_axi_rid;
      assign core_axi_rdata    = m_axi_rdata;
      assign core_axi_rresp    = m_axi_rresp;
      assign core_axi_rlast    = m_axi_rlast;
      assign core_axi_rvalid   = m_axi_rvalid;
      assign m_axi_rready      = core_axi_rready;
   end
   endgenerate

   // ===================== The core: rv_soc_top at probe_clk =====================
   // rv_soc_top (core + I$/D$ + CLINT/PLIC/UART + boot SRAM) at probe_clk; its tagged DDR
   // memory port -> ddr_port_cdc (three async FIFOs) -> ddr_port_axi -> the core_axi_* arbiter
   // input (the MIG path). virtio-blk/net and the MMIO bridge hang off its virtio passthrough below.
   wire         pq_valid, pq_ready, pq_we, pr_valid, pr_ready, pr_last, pw_valid, pw_ready;
   wire [4:0]   pq_id, pr_id, pw_id;  wire [57:0] pq_addr;  wire [63:0] pq_wmask;  wire [511:0] pq_wdata;
   wire [1:0]   pr_beat;  wire [127:0] pr_data;
   wire         mq_valid, mq_ready, mq_we, mr_valid, mr_ready, mr_last, mw_valid, mw_ready;
   wire [4:0]   mq_id, mr_id, mw_id;  wire [57:0] mq_addr;  wire [63:0] mq_wmask;  wire [511:0] mq_wdata;
   wire [1:0]   mr_beat;  wire [127:0] mr_data;
   wire        ptx_valid;  wire [7:0] ptx_data;  wire ptx_ready;
   wire        prx_valid;  wire [7:0] prx_data;

   // key[3] (active low) toggles kbd_route: UART RX to the console (0) or to the keyboard (1).
   // Two flops of synchronizer, then a debounce: the level must hold for 2^17 probe_clk cycles
   // (~0.8 ms at 166 MHz) before it counts, so contact bounce gives one toggle per press.
   (* async_reg = "true" *) reg [1:0] key3_s = 2'b11;
   reg        key3_db = 1'b1;
   reg [16:0] key3_cnt = 17'd0;
   reg        kbd_route = 1'b0;
   always @(posedge probe_clk) begin
      key3_s <= {key3_s[0], key[3]};
      if (key3_s[1] == key3_db) key3_cnt <= 17'd0;
      else key3_cnt <= key3_cnt + 17'd1;
      if (&key3_cnt) begin
         key3_db <= key3_s[1];
         if (!key3_s[1]) kbd_route <= ~kbd_route;   // a press
      end
      if (probe_reset) kbd_route <= 1'b0;
   end
   // Keyboard bytes cross to ui_clk, where virtio_input lives. At 3 Mbps a byte every 3.3 us and
   // the reader takes one a cycle, so this never fills.
   // Block RAM, as mmio_cmd_fifo/mmio_rsp_fifo: in LUT-RAM this crossing's read path missed
   // 333 MHz on clock-root skew (-0.0006 ns, build 68b4997b), the failure those two document.
   smolrv64_async_fifo #(.WIDTH(8), .ADDR_BITS(4), .MEMORY_TYPE("block")) kbd_byte_fifo (
      .wr_clock(probe_clk), .rd_clock(ui_clk), .reset(probe_reset),
      .wr_valid(prx_valid && kbd_route), .wr_ready(),
      .wr_data(prx_data),
      .rd_valid(kbd_byte_valid), .rd_ready(1'b1), .rd_data(kbd_byte));

   wire [14:0] p_virtio_addr;  wire p_virtio_read, p_virtio_write;   // [14:12]: the device page
   wire [31:0] p_virtio_wdata; wire [3:0] p_virtio_be;
   // virtio IRQs (ui_clk) synchronized into probe_clk for rv_soc_top's internal PLIC.
   // blk -> src 11, net -> src 12 (both match the DTB `interrupts` properties).
   // The net one was MISSING: virtio_net_irq was only synced into the retired scalar SoC's
   // ext_irq, so the probe core never saw it. Measured symptom: virtio-net InterruptStatus stuck
   // at 1 (asserted, unacknowledged) with `virtio1: 0` in /proc/interrupts while virtio-blk on
   // src 11 took 44k -- the driver never harvested the RX ring, so DHCP never got a reply.
   // Both are LEVEL interrupts (virtio_mmio: irq = interrupt_status != 0), so a 2-FF sync is
   // sufficient -- no pulse to lose.
   (* async_reg = "true" *) reg p_virtio_irq_meta = 1'b0, p_virtio_irq = 1'b0;
   (* async_reg = "true" *) reg p_virtio_net_irq_meta = 1'b0, p_virtio_net_irq = 1'b0;
   (* async_reg = "true" *) reg p_kbd_irq_meta = 1'b0, p_kbd_irq = 1'b0;    // the keyboard: src 4
   always @(posedge probe_clk) p_kbd_irq_meta <= kbd_irq;
   always @(posedge probe_clk) p_kbd_irq <= p_kbd_irq_meta;
   always @(posedge probe_clk) begin
      p_virtio_irq_meta     <= virtio_blk_irq;  p_virtio_irq     <= p_virtio_irq_meta;
      p_virtio_net_irq_meta <= virtio_net_irq;  p_virtio_net_irq <= p_virtio_net_irq_meta;
   end

   wire [1:0]  probe_par_err;   // cache data-array parity error pulses {I$,D$} (-DCACHE_PARITY)
   wire [63:0] probe_par_dbg;   // ...sticky/bank/addr snapshot of the first failure
   wire [17:0] probe_irq_dbg;   // interrupt-path debug (probe_clk) for ILA_IRQ
   wire        core_commit;     // retire pulse (probe_clk) for ILA_CORE
   wire [199:0] probe_core_dbg;  // the core's wait state (rv_soc_top core_dbg) for ILA_MEM
   // A DEVICE WROTE MEMORY, to the core's clock. Device writes do not probe the I$, so the next
   // fence.i clears it after one (rv_soc_top dma_wr). Each write burst's address handshake is held
   // for four ui_clk cycles -- two of probe_clk -- and synchronised; back-to-back bursts may merge,
   // which is all the core needs: that at least one happened.
   reg [3:0] dma_st = 4'd0;                       // ui_clk: the last four cycles' handshakes
   always @(posedge ui_clk) dma_st <= {dma_st[2:0], device_axi_awvalid & device_axi_awready};
   (* ASYNC_REG = "TRUE" *) reg [1:0] dma_sy = 2'd0;
   always @(posedge probe_clk) dma_sy <= {dma_sy[0], |dma_st};
   wire p_dma_wr = dma_sy[1];
   rv_soc_top #(.RESET_PC(64'h7000_0000)) probe_core (
      .clk(probe_clk), .reset(probe_reset), .fbdiag_reset_req(fbdiag_reset_req),
      .retire(core_commit), .dmem_wen(), .dmem_waddr(), .dmem_wdata(), .dmem_wmask(),
      .ddr_q_valid(pq_valid), .ddr_q_ready(pq_ready), .ddr_q_id(pq_id), .ddr_q_we(pq_we),
      .ddr_q_addr(pq_addr), .ddr_q_wmask(pq_wmask), .ddr_q_wdata(pq_wdata),
      .ddr_r_valid(pr_valid), .ddr_r_ready(pr_ready), .ddr_r_id(pr_id), .ddr_r_beat(pr_beat),
      .ddr_r_last(pr_last), .ddr_r_data(pr_data),
      .ddr_w_valid(pw_valid), .ddr_w_ready(pw_ready), .ddr_w_id(pw_id),
      .uart_rx_we(prx_valid && !kbd_route), .uart_rx_data(prx_data), .uart_rx_ready(),
      .uart_tx_valid(ptx_valid), .uart_tx_data(ptx_data), .uart_tx_ready(ptx_ready),
      .irq_dbg(probe_irq_dbg),
      .cache_par_err(probe_par_err), .cache_par_dbg(probe_par_dbg),
      .virtio_addr(p_virtio_addr), .virtio_read(p_virtio_read), .virtio_write(p_virtio_write),
      .virtio_wdata(p_virtio_wdata), .virtio_be(p_virtio_be),
      .virtio_rdata(core_mmio_readdata), .virtio_rvalid(core_mmio_readdatavalid),
      .virtio_irq(p_virtio_irq), .virtio_net_irq(p_virtio_net_irq), .virtio_kbd_irq(p_kbd_irq), .dma_wr(p_dma_wr), .core_dbg(probe_core_dbg));

`ifdef ILA_IRQ
   // Debug (ILA_IRQ=1): capture the virtio_blk interrupt lifecycle on probe_clk. probe_irq_dbg =
   //   [17]=plic_re [16]=plic_we [15:12]=plic_addr[3:0] (claim/complete=0x4)
   //   [11]=complete_evt [10]=do_claim [9]=seip [8]=in_service[11] [7]=pending[11]
   //   [6]=src_level[11] (device raising IRQ) [5:0]=best_irq
   // Diagnose the root-mount hang: does the device raise IRQ (bit6), does it pend (bit7)+seip(bit9),
   // is in_service[11] stuck (bit8), does a claim(bit10)/complete(bit11) happen?
   ila_irq u_ila_irq (
      .clk    (probe_clk),
      .probe0 (probe_irq_dbg)
   );
`endif

`ifdef ILA_CORE
   // Debug (ILA_CORE=1): capture the probe-clk core/DDR-line steady state to localize a
   // post-root-mount wedge (e.g. systemd "Hostname set"). Intended use is a -trigger_now
   // capture AFTER the board has visibly wedged: the frozen probe levels disambiguate
   //   - probe1={q_valid,q_ready,q_we} stuck at valid=1,ready=0 -> DDR port back-pressure deadlock
   //   - probe0 commit frozen + probe3 irq pending, never claimed -> interrupt livelock
   //   - probe0 commit still toggling -> core alive, spinning in userspace (SW / rare-insn)
   //   probe2 = pq_addr[23:0]: which line the port is asking for (region id).
   ila_core u_ila_core (
      .clk    (probe_clk),
      .probe0 (core_commit),                 // 1: retire pulse
      .probe1 ({pq_valid, pq_ready, pq_we}),  // 3: port request handshake
      .probe2 (pq_addr[23:0]),               // 24: requested line address (low bits)
      .probe3 (probe_irq_dbg)                // 18: interrupt-path bus (reused decode)
   );
`endif

`ifdef ILA_MEM
   // Debug (ILA_MEM=1): the memory path from the core's port to the DDR4 controller, captured at
   // the ONSET of a wedge. ila_trig rises when the port has had something waiting (a request not
   // taken, or a read or write owed) with no handshake on any of its channels for 2047 probe_clk
   // cycles (12 us; a DDR read takes ~30), and stays up. Each ILA triggers on it (ila_mem_m
   // through a synchronizer), so with the trigger at the end of the window the 4096 samples
   // before it show how the path stopped. ila_mem_p is the core side, ila_mem_m the ui_clk side:
   // the CDC's far end, the core's and the devices' AXI masters, and the arbiter's AXI into the
   // controller (m_axi_*, and the AW/W slice's output, mig_*).
   reg  [5:0]  ila_rd_out, ila_wr_out;   // reads and writes the port owes a response
   reg  [10:0] ila_quiet;                // cycles waiting with no handshake, saturating
   reg         ila_trig;
   wire ila_waiting  = (pq_valid & ~pq_ready) | (ila_rd_out != 6'd0) | (ila_wr_out != 6'd0);
   wire ila_progress = (pq_valid & pq_ready) | (pr_valid & pr_ready) | (pw_valid & pw_ready);
   always @(posedge probe_clk) begin
      if (probe_reset) begin
         ila_rd_out <= 6'd0; ila_wr_out <= 6'd0; ila_quiet <= 11'd0; ila_trig <= 1'b0;
      end else begin
         ila_rd_out <= ila_rd_out + {5'd0, pq_valid & pq_ready & ~pq_we} - {5'd0, pr_valid & pr_ready & pr_last};
         ila_wr_out <= ila_wr_out + {5'd0, pq_valid & pq_ready &  pq_we} - {5'd0, pw_valid & pw_ready};
         ila_quiet  <= (ila_waiting & ~ila_progress) ? ila_quiet + {10'd0, ~&ila_quiet} : 11'd0;
         if (&ila_quiet) ila_trig <= 1'b1;
      end
   end
   ila_mem_p u_ila_mem_p (
      .clk    (probe_clk),
      .probe0 ({ila_trig, core_commit, pq_valid, pq_ready, pq_we,
                pr_valid, pr_ready, pr_last, pw_valid, pw_ready}),   // 10
      .probe1 ({pq_id, pr_id, pw_id, pr_beat}),                       // 17
      .probe2 (pq_addr[23:0]),                                        // 24: line index, low bits
      .probe3 ({ila_rd_out, ila_wr_out}),                             // 12
      .probe4 (probe_irq_dbg),                                        // 18
      .probe5 (probe_core_dbg)                                        // 200: what the core waits on
   );
   (* ASYNC_REG = "TRUE" *) reg [2:0] ila_trig_m;
   always @(posedge ui_clk) ila_trig_m <= {ila_trig_m[1:0], ila_trig};
   ila_mem_m u_ila_mem_m (
      .clk    (ui_clk),
      .probe0 ({ila_trig_m[2], mq_valid, mq_ready, mq_we,
                mr_valid, mr_ready, mw_valid, mw_ready}),             // 8
      .probe1 ({core_axi_arvalid, core_axi_arready, core_axi_rvalid, core_axi_rready,
                core_axi_rlast, core_axi_awvalid, core_axi_awready, core_axi_wvalid,
                core_axi_wready, core_axi_wlast, core_axi_bvalid, core_axi_bready}),  // 12
      .probe2 ({device_axi_arvalid, device_axi_arready, device_axi_rvalid, device_axi_rready,
                device_axi_rlast, device_axi_awvalid, device_axi_awready, device_axi_wvalid,
                device_axi_wready, device_axi_wlast, device_axi_bvalid, device_axi_bready}),  // 12
      .probe3 ({m_axi_arvalid, m_axi_arready, m_axi_rvalid, m_axi_rready, m_axi_rlast,
                m_axi_awvalid, m_axi_awready, m_axi_wvalid, m_axi_wready, m_axi_wlast,
                m_axi_bvalid, m_axi_bready,
                mig_awvalid, mig_awready, mig_wvalid, mig_wready, mig_wlast}),        // 17
      .probe4 ({m_axi_arid, m_axi_rid, m_axi_awid, m_axi_bid}),      // 12
      .probe5 ({m_axi_araddr[23:6], m_axi_awaddr[23:6]})              // 36: line index, low bits
   );
`endif

   ddr_port_cdc probe_cdc (
      .clk_p(probe_clk), .reset_p(probe_reset),
      .p_q_valid(pq_valid), .p_q_ready(pq_ready), .p_q_id(pq_id), .p_q_we(pq_we), .p_q_addr(pq_addr),
      .p_q_wmask(pq_wmask), .p_q_wdata(pq_wdata),
      .p_r_valid(pr_valid), .p_r_ready(pr_ready), .p_r_id(pr_id), .p_r_beat(pr_beat), .p_r_last(pr_last),
      .p_r_data(pr_data), .p_w_valid(pw_valid), .p_w_ready(pw_ready), .p_w_id(pw_id),
      .clk_m(ui_clk), .reset_m(ui_cpu_reset),
      .m_q_valid(mq_valid), .m_q_ready(mq_ready), .m_q_id(mq_id), .m_q_we(mq_we), .m_q_addr(mq_addr),
      .m_q_wmask(mq_wmask), .m_q_wdata(mq_wdata),
      .m_r_valid(mr_valid), .m_r_ready(mr_ready), .m_r_id(mr_id), .m_r_beat(mr_beat), .m_r_last(mr_last),
      .m_r_data(mr_data), .m_w_valid(mw_valid), .m_w_ready(mw_ready), .m_w_id(mw_id));

   ddr_port_axi probe_bridge (
      .clk(ui_clk), .reset(ui_cpu_reset),
      .q_valid(mq_valid), .q_ready(mq_ready), .q_id(mq_id), .q_we(mq_we), .q_addr(mq_addr),
      .q_wmask(mq_wmask), .q_wdata(mq_wdata),
      .r_valid(mr_valid), .r_ready(mr_ready), .r_id(mr_id), .r_beat(mr_beat), .r_last(mr_last),
      .r_data(mr_data), .w_valid(mw_valid), .w_ready(mw_ready), .w_id(mw_id),
      .m_axi_awid(core_axi_awid), .m_axi_awaddr(core_axi_awaddr), .m_axi_awlen(core_axi_awlen),
      .m_axi_awsize(core_axi_awsize), .m_axi_awburst(core_axi_awburst), .m_axi_awlock(core_axi_awlock),
      .m_axi_awcache(core_axi_awcache), .m_axi_awprot(core_axi_awprot), .m_axi_awqos(core_axi_awqos),
      .m_axi_awvalid(core_axi_awvalid), .m_axi_awready(core_axi_awready),
      .m_axi_wdata(core_axi_wdata), .m_axi_wstrb(core_axi_wstrb), .m_axi_wlast(core_axi_wlast),
      .m_axi_wvalid(core_axi_wvalid), .m_axi_wready(core_axi_wready),
      .m_axi_bid(core_axi_bid), .m_axi_bresp(core_axi_bresp), .m_axi_bvalid(core_axi_bvalid),
      .m_axi_bready(core_axi_bready),
      .m_axi_arid(core_axi_arid), .m_axi_araddr(core_axi_araddr), .m_axi_arlen(core_axi_arlen),
      .m_axi_arsize(core_axi_arsize), .m_axi_arburst(core_axi_arburst), .m_axi_arlock(core_axi_arlock),
      .m_axi_arcache(core_axi_arcache), .m_axi_arprot(core_axi_arprot), .m_axi_arqos(core_axi_arqos),
      .m_axi_arvalid(core_axi_arvalid), .m_axi_arready(core_axi_arready),
      .m_axi_rid(core_axi_rid), .m_axi_rdata(core_axi_rdata), .m_axi_rresp(core_axi_rresp),
      .m_axi_rlast(core_axi_rlast), .m_axi_rvalid(core_axi_rvalid), .m_axi_rready(core_axi_rready));

   // UART at probe_clk. rs232 ROUNDS the divisor (period=(CLK+BAUD/2)/BAUD), so at
   // 66.67 MHz (ui_clk/5) / 3 Mbaud -> period 22 -> 3.030 Mbaud (1.0% err, within tolerance).
   // CLK_FREQ MUST track probe_clk's divider: a stale value skews the baud (leaving 41.67
   // here while the clock runs /5 transmits ~60% too fast -> garbage on the wire).
   // Matches the scalar core + `make connect` (3 Mbaud); 26x faster fw load than 115200.
   // rs232tx.ready (output, ready-to-accept) feeds rv_soc_top.uart_tx_ready directly.
   rs232tx #(.CLK_FREQ(`PROBE_CLK_HZ), .BAUD(3_000_000)) probe_tx
     (.clk(probe_clk), .rst_n(~probe_reset),
      .data(ptx_data), .valid(ptx_valid), .ready(ptx_ready), .tx(txd));
   rs232rx #(.CLK_FREQ(`PROBE_CLK_HZ), .BAUD(3_000_000)) probe_rx
     (.clk(probe_clk), .rst_n(~probe_reset),
      .data(prx_data), .valid(prx_valid), .ready(1'b1), .rxd(rxd), .overflow());

   // Drive the MMIO bridge core side from rv_soc_top's device window. rv_soc_top emits the
   // offset from 0x1000_0000, and its page, [14:12], is the page here as is: 0x02 virtio_blk_inst,
   // 0x03 virtio_net_inst, 0x05 vga_scanout_inst (0x04 is the keyboard's). virtio_net_irq is
   // wired to PLIC src 12 (ext_irq[11]).
   assign core_mmio_address    = {5'd0, p_virtio_addr};
   assign core_mmio_read       = p_virtio_read;
   assign core_mmio_write      = p_virtio_write;
   assign core_mmio_writedata  = p_virtio_wdata;
   assign core_mmio_byteenable = p_virtio_be;
endmodule

module smolrv64_mmio_clock_bridge(
   input  wire        core_clock,
   input  wire        core_reset,
   input  wire [19:0] core_address,
   input  wire        core_read,
   input  wire        core_write,
   input  wire [31:0] core_writedata,
   input  wire [ 3:0] core_byteenable,
   output wire        core_readdatavalid,
   output wire [31:0] core_readdata,

   input  wire        ui_clock,
   input  wire        ui_reset,
   output reg  [19:0] ui_address = 20'd0,
   output reg         ui_read = 1'b0,
   output reg         ui_write = 1'b0,
   output reg  [31:0] ui_writedata = 32'd0,
   output reg  [ 3:0] ui_byteenable = 4'd0,
   input  wire        ui_readdatavalid,
   input  wire [31:0] ui_readdata
);
   // Local replica of ui_reset with bounded fanout. The MIG-driven
   // c0_ddr4_ui_clk_sync_rst was fanning out to 425+ registers via auto-
   // replication, with a 2.5 ns route into mmio_bridge eating timing.
   // Capped fanout forces Vivado to replicate into smaller groups near
   // each consumer, cutting the route delay.
   (* max_fanout = 16 *) reg ui_reset_q = 1'b1;
   always @(posedge ui_clock) ui_reset_q <= ui_reset;

   // Registered FIFO reset with bounded fanout. The two MMIO CDC FIFOs
   // take `core_reset | ui_reset` as their .reset() — combinational
   // signals routing from the core-side reset through
   // an OR into the BRAM-located FIFOs' internal reset FSMs were the new
   // worst paths after the BRAM forcing. Register here, gate fanout.
   (* max_fanout = 8 *) reg fifo_reset_q = 1'b1;
   always @(posedge ui_clock) fifo_reset_q <= core_reset | ui_reset;

   localparam CMD_WIDTH = 58;
   localparam [1:0] UI_IDLE = 2'd0;
   localparam [1:0] UI_WAIT_RSP = 2'd1;
   localparam [1:0] UI_SEND_RSP = 2'd2;

   wire                  cmd_wr_ready;
   wire [CMD_WIDTH-1:0] cmd_wr_data =
      {core_write, core_read, core_address, core_writedata, core_byteenable};
   wire                  cmd_wr_valid = core_read || core_write;
   wire                  cmd_rd_valid;
   wire                  cmd_rd_ready;
   wire [CMD_WIDTH-1:0] cmd_rd_data;

   smolrv64_async_fifo #(
      .WIDTH(CMD_WIDTH),
      .ADDR_BITS(4),
      // Force BRAM: timing on this FIFO's RAMD32-to-doutb path failed at
      // -0.090 ns with auto (SLICEM distributed RAM) due to write/read
      // clock-root skew. BRAM placement is more predictable.
      .MEMORY_TYPE("block")
   ) mmio_cmd_fifo (
      .wr_clock(core_clock),
      .rd_clock(ui_clock),
      .reset(fifo_reset_q),
      .wr_valid(cmd_wr_valid),
      .wr_ready(cmd_wr_ready),
      .wr_data(cmd_wr_data),
      .rd_valid(cmd_rd_valid),
      .rd_ready(cmd_rd_ready),
      .rd_data(cmd_rd_data)
   );

   wire        cmd_is_write = cmd_rd_data[57];
   wire        cmd_is_read = cmd_rd_data[56];
   wire [19:0] cmd_address = cmd_rd_data[55:36];
   wire [31:0] cmd_writedata = cmd_rd_data[35:4];
   wire [ 3:0] cmd_byteenable = cmd_rd_data[3:0];

   reg  [1:0] ui_state = UI_IDLE;
   reg  [31:0] rsp_data_q = 32'd0;
   wire        rsp_wr_ready;
   wire        rsp_wr_valid = ui_state == UI_SEND_RSP;
   wire        rsp_rd_valid;

   assign cmd_rd_ready = cmd_rd_valid && ui_state == UI_IDLE &&
                         (cmd_is_write || cmd_is_read);

   smolrv64_async_fifo #(
      .WIDTH(32),
      .ADDR_BITS(4),
      // Same BRAM-forcing rationale as mmio_cmd_fifo above.
      .MEMORY_TYPE("block")
   ) mmio_rsp_fifo (
      .wr_clock(ui_clock),
      .rd_clock(core_clock),
      .reset(fifo_reset_q),
      .wr_valid(rsp_wr_valid),
      .wr_ready(rsp_wr_ready),
      .wr_data(rsp_data_q),
      .rd_valid(rsp_rd_valid),
      .rd_ready(rsp_rd_valid),
      .rd_data(core_readdata)
   );

   assign core_readdatavalid = rsp_rd_valid;

`ifndef SYNTHESIS
   always @(posedge core_clock) begin
      if (!core_reset && cmd_wr_valid && !cmd_wr_ready)
         $display("%05d MMIO clock bridge command FIFO overflow", $time);
   end
`endif

   always @(posedge ui_clock) begin
      if (ui_reset_q) begin
         ui_address <= 20'd0;
         ui_read <= 1'b0;
         ui_write <= 1'b0;
         ui_writedata <= 32'd0;
         ui_byteenable <= 4'd0;
         ui_state <= UI_IDLE;
         rsp_data_q <= 32'd0;
      end else begin
         ui_read <= 1'b0;
         ui_write <= 1'b0;

         case (ui_state)
           UI_IDLE: begin
              if (cmd_rd_valid) begin
                 ui_address <= cmd_address;
                 ui_writedata <= cmd_writedata;
                 ui_byteenable <= cmd_byteenable;
                 ui_write <= cmd_is_write;
                 ui_read <= cmd_is_read;
                 if (cmd_is_read)
                    ui_state <= UI_WAIT_RSP;
                 else if (cmd_is_write)
                    ui_state <= UI_SEND_RSP;   // writes send a completion too, so rv_soc_top can BLOCK the
                                               // store until delivery (rsp data is don't-care for a write)
              end
           end
           UI_WAIT_RSP: begin
              if (ui_readdatavalid) begin
                 rsp_data_q <= ui_readdata;
                 ui_state <= UI_SEND_RSP;
              end
           end
           UI_SEND_RSP: begin
              if (rsp_wr_ready)
                 ui_state <= UI_IDLE;
           end
           default: ui_state <= UI_IDLE;
         endcase
      end
   end
endmodule

`ifndef SYNTHESIS
module IOBUF(input wire I,
             output wire O,
             input wire T,
             inout wire IO);
   assign IO = T ? 1'bz : I;
   assign O = IO;
endmodule
`endif

module sd_gpio_dat(input  wire        clock,
                   input  wire        reset,
                   input  wire        write,
                   input  wire [31:0] write_data,
                   input  wire [ 3:0] byteenable,
                   output reg  [ 7:0] gpio_out = 8'h01,
                   output wire [31:0] read_data);
   assign read_data = {24'd0, gpio_out};

   always @(posedge clock) begin
      if (reset)
         gpio_out <= 8'h01;
      else if (write && byteenable[0])
         gpio_out <= write_data[7:0];
   end
endmodule

module sd_spi_oc_tiny(input  wire        clock,
                      input  wire        reset,
                      input  wire [ 7:0] address,
                      output reg  [31:0] read_data,
                      input  wire        read,
                      input  wire        write,
                      input  wire [31:0] write_data,
                      input  wire [ 3:0] byteenable,
                      output reg         spi_clk = 1'b0,
                      output reg         spi_mosi = 1'b1,
                      input  wire        spi_miso);
   localparam [7:0] REG_RXDATA  = 8'h00;
   localparam [7:0] REG_TXDATA  = 8'h04;
   localparam [7:0] REG_STATUS  = 8'h08;
   localparam [7:0] REG_CONTROL = 8'h0c;
   localparam [7:0] REG_BAUD    = 8'h10;

   reg [7:0] control = 8'd0;
   reg [7:0] baud = 8'd255;
   reg [7:0] baud_count = 8'd0;
   reg [7:0] tx_shift = 8'hff;
   reg [7:0] rx_shift = 8'd0;
   reg [7:0] pending_tx = 8'hff;
   reg [7:0] txr_data = 8'd0;
   reg [7:0] rx_data = 8'd0;
   reg [2:0] bit_index = 3'd7;
   reg       busy = 1'b0;
   reg       phase = 1'b0;
   reg       pending_valid = 1'b0;
   reg       txr_valid = 1'b0;
   reg       rx_valid = 1'b0;

   wire cpol = control[1];
   wire cpha = control[0];
   wire [7:0] reg_addr = address[7:0] & 8'hfc;
   wire       txdata_write = write && byteenable[0] && reg_addr == REG_TXDATA;
   wire       completing = busy && baud_count == 0 && phase && bit_index == 0;
   wire [7:0] completed_rx = cpha ? {rx_shift[7:1], spi_miso} : rx_shift;

   task start_transfer;
      input [7:0] value;
      begin
         busy <= 1'b1;
         phase <= 1'b0;
         baud_count <= baud;
         bit_index <= 3'd7;
         tx_shift <= value;
         rx_shift <= 8'd0;
         spi_clk <= cpol;
         spi_mosi <= value[7];
      end
   endtask

   always @* begin
      case (reg_addr)
        REG_RXDATA:  read_data = {24'd0, rx_data};
        REG_TXDATA:  read_data = {24'd0, txr_data};
        REG_STATUS:  read_data = {30'd0, txr_valid, !busy && !pending_valid};
        REG_CONTROL: read_data = {24'd0, control};
        REG_BAUD:    read_data = {24'd0, baud};
        default:     read_data = 32'd0;
      endcase
   end

   always @(posedge clock) begin
      if (reset) begin
         control <= 8'd0;
         baud <= 8'd255;
         baud_count <= 8'd0;
         tx_shift <= 8'hff;
         rx_shift <= 8'd0;
         pending_tx <= 8'hff;
         txr_data <= 8'd0;
         rx_data <= 8'd0;
         bit_index <= 3'd7;
         busy <= 1'b0;
         phase <= 1'b0;
         pending_valid <= 1'b0;
         txr_valid <= 1'b0;
         rx_valid <= 1'b0;
         spi_clk <= 1'b0;
         spi_mosi <= 1'b1;
      end else begin
         if (write && byteenable[0]) begin
            case (reg_addr)
              REG_TXDATA: begin
                 txr_valid <= 1'b0;
                 if (rx_valid && !busy && !pending_valid) begin
                    txr_data <= rx_data;
                    txr_valid <= 1'b1;
                    rx_valid <= 1'b0;
                 end
                 if (busy && !completing)
                    {pending_valid, pending_tx} <= {1'b1, write_data[7:0]};
                 else if (!busy)
                    start_transfer(write_data[7:0]);
              end
              REG_STATUS: begin
                 txr_valid <= txr_valid & write_data[1];
                 rx_valid <= rx_valid & write_data[0];
              end
              REG_CONTROL: begin
                 control <= write_data[7:0];
                 if (!busy)
                    spi_clk <= write_data[1];
              end
              REG_BAUD: baud <= write_data[7:0];
              default: begin
              end
            endcase
         end

         if (read && reg_addr == REG_TXDATA && txr_valid)
            txr_valid <= 1'b0;
         if (read && reg_addr == REG_RXDATA && rx_valid)
            rx_valid <= 1'b0;

         if (busy) begin
            if (baud_count != 0) begin
               baud_count <= baud_count - 1'b1;
            end else begin
               baud_count <= baud;
               if (!phase) begin
                  spi_clk <= ~cpol;
                  phase <= 1'b1;
                  if (!cpha)
                     rx_shift[bit_index] <= spi_miso;
                  else
                     spi_mosi <= tx_shift[bit_index];
               end else begin
                  spi_clk <= cpol;
                  phase <= 1'b0;
                  if (cpha)
                     rx_shift[bit_index] <= spi_miso;
                  if (bit_index == 0) begin
                     busy <= 1'b0;
                     spi_mosi <= 1'b1;
                     if (pending_valid || txdata_write) begin
                        txr_data <= completed_rx;
                        txr_valid <= 1'b1;
                        pending_valid <= 1'b0;
                        start_transfer(pending_valid ? pending_tx : write_data[7:0]);
                     end else begin
                        rx_data <= completed_rx;
                        rx_valid <= 1'b1;
                     end
                  end else begin
                     bit_index <= bit_index - 1'b1;
                     spi_mosi <= tx_shift[bit_index - 1'b1];
                  end
               end
            end
         end
      end
   end
endmodule
