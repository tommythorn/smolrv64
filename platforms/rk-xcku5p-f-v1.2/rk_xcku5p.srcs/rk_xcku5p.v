`timescale 1ns / 1ps
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
   wire core_clk;
   wire fpu_clk;
   wire ui_rst;           // c0_ddr4_ui_clk_sync_rst (active high)
   wire init_calib_complete;
   wire halted;

   // CPU is held in reset until calibration completes.
   // key[1] is a soft-reset button (active low): pulses the CPU reset without
   // touching the DDR4 MIG, so calibration is preserved and mig_* latency
   // stats CSRs (which aren't in the CPU's reset block) survive across a
   // soft reset. Press key[1] to return to the monitor from a hung workload.
   wire ui_cpu_reset_req = ui_rst | ~init_calib_complete | ~key[1];
   reg  [1:0] ui_cpu_reset_sync = 2'b11;
   wire ui_cpu_reset = ui_cpu_reset_sync[1];
   reg  [1:0] cpu_reset_core_sync = 2'b11;
   wire cpu_reset = cpu_reset_core_sync[1];

`ifndef PROBE_DIAG
   assign led = 4'b0000;
`endif

   // Run the CPU-side pipeline and hit path at half the DDR4 UI clock.  The
   // cache refill/writeback engine inside smolrv64 still uses ui_clk through
   // the separate mem_clock port.
   BUFGCE_DIV #(
      .BUFGCE_DIVIDE(2)
   ) core_clk_buf (
      .I  (ui_clk),
      .CE (1'b1),
      .CLR(ui_rst),
      .O  (core_clk)
   );

   always @(posedge ui_clk) begin
      if (ui_cpu_reset_req)
         ui_cpu_reset_sync <= 2'b11;
      else
         ui_cpu_reset_sync <= {ui_cpu_reset_sync[0], 1'b0};
   end

   always @(posedge core_clk or posedge ui_cpu_reset) begin
      if (ui_cpu_reset)
         cpu_reset_core_sync <= 2'b11;
      else
         cpu_reset_core_sync <= {cpu_reset_core_sync[0], 1'b0};
   end

   // CVFPU is throughput-capable but much deeper than the integer core.  The
   // core issues one FP operation at a time and waits, so run the FPU island at
   // a conservative divided clock and bridge it inside smolrv64_cvfpu.
   BUFGCE_DIV #(
      .BUFGCE_DIVIDE(4)
   ) fpu_clk_buf (
      .I  (ui_clk),
      .CE (1'b1),
      .CLR(ui_rst),
      .O  (fpu_clk)
   );

`ifdef PROBE_CORE
   // Sharded-OoO probe core: modest-clock bring-up on a divided ui_clk. Post-route timing
   // at /6 showed the global WNS pinned by virtio @333 MHz, with probe_clk absent from the
   // worst-5 setup paths -- i.e. the real probe setup Fmax sits above the ~67 MHz OOC
   // estimate -- so we push to /5 = 66.7 MHz (was /8 = 41.7). /4 = 83.3 MHz was TRIED and
   // FAILED routing (WNS -1.78 ns at the 12 ns target => real Fmax ~72.5 MHz; probe_clk paths
   // dominate the failing set), so /5 stands until the redirect/wake/MMIO cones get pipelined.
   // The ddr_* line port crosses back to ui_clk (MIG/arbiter/bridge) via ddr_line_cdc.
   // Synchronous divide keeps probe_clk phase-related to ui_clk.  The UART CLK_FREQ below AND
   // the CLINT SCALE_DIV (probe/soc_top.v) MUST track this divider.
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
   BUFGCE_DIV #(.BUFGCE_DIVIDE(5)) probe_clk_buf
      (.I(ui_clk), .CE(1'b1), .CLR(probe_por), .O(probe_clk));
   (* async_reg = "true" *) reg [1:0] probe_reset_sync = 2'b11;
   always @(posedge probe_clk or posedge probe_por)
      if (probe_por) probe_reset_sync <= 2'b11;
      else           probe_reset_sync <= {probe_reset_sync[0], 1'b0};
   wire probe_reset = probe_reset_sync[1];
   // LEDs (core-independent): observe cal/clock health at a glance.
   reg [26:0] hb_ui = 27'd0;    always @(posedge ui_clk)    hb_ui  <= hb_ui  + 1'b1;
   reg [23:0] hb_pr = 24'd0;    always @(posedge probe_clk) hb_pr  <= hb_pr  + 1'b1;
   assign led = {hb_ui[26], hb_pr[23], ui_rst, init_calib_complete};
`else
   BUFGCE_DIV #(.BUFGCE_DIVIDE(5)) probe_clk_buf
      (.I(ui_clk), .CE(1'b1), .CLR(ui_rst), .O(probe_clk));
   (* async_reg = "true" *) reg [1:0] probe_reset_sync = 2'b11;
   always @(posedge probe_clk or posedge ui_cpu_reset)
      if (ui_cpu_reset) probe_reset_sync <= 2'b11;
      else              probe_reset_sync <= {probe_reset_sync[0], 1'b0};
   wire probe_reset = probe_reset_sync[1];
`endif
`endif

   wire         dbg_clk;
   wire [511:0] dbg_bus;
   wire         c0_ddr4_reset_n_int;
   assign c0_ddr4_reset_n = c0_ddr4_reset_n_int;

   // AXI4 wires between smolrv64 and the DDR4 IP (64-bit data, 31-bit byte addr,
   // 3-bit ID, 8-byte beats, single-beat bursts).
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

   wire       uart_tx_valid;
   wire [7:0] uart_tx_data;
   wire       rx_valid;
   wire [7:0] rx_data;
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
   wire [31:0] sd_cd_gpio_readdata = {31'd0, sd_cd_sync};
   wire [31:0] virtio_blk_readdata;
   wire [31:0] virtio_net_readdata;
   localparam [63:0] BUILD_ID_STAMP = `SMOLRV64_BUILD_STAMP;
   localparam [31:0] BUILD_ID_GIT_COMMIT = `SMOLRV64_GIT_COMMIT;
   localparam        BUILD_ID_GIT_DIRTY = `SMOLRV64_GIT_DIRTY;
   reg  [31:0] build_id_readdata;
   wire        virtio_blk_irq;
   wire        virtio_net_irq;
   reg         virtio_blk_irq_meta = 1'b0;
   reg         virtio_blk_irq_core = 1'b0;
   reg         virtio_net_irq_meta = 1'b0;
   reg         virtio_net_irq_core = 1'b0;
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

   always @(posedge core_clk) begin
      if (cpu_reset) begin
         virtio_blk_irq_meta <= 1'b0;
         virtio_blk_irq_core <= 1'b0;
         virtio_net_irq_meta <= 1'b0;
         virtio_net_irq_core <= 1'b0;
      end else begin
         virtio_blk_irq_meta <= virtio_blk_irq;
         virtio_blk_irq_core <= virtio_blk_irq_meta;
         virtio_net_irq_meta <= virtio_net_irq;
         virtio_net_irq_core <= virtio_net_irq_meta;
      end
   end

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
         if (ui_mmio_read) begin
            if (spi_speed_sel)
               mmio_readdata_q <= {16'd0, spi_fast_half};
            else if (sd_cd_gpio_sel)
               mmio_readdata_q <= sd_cd_gpio_readdata;
            else if (virtio_blk_sel && ui_mmio_address[11:8] == 4'hf)
               mmio_readdata_q <= virtio_blk_debug_word;  // 0x10002f00: cap/state
            else if (virtio_blk_sel)
               mmio_readdata_q <= virtio_blk_readdata;
            else if (virtio_net_sel && ui_mmio_address[11:8] == 4'hf)
               mmio_readdata_q <= virtio_net_debug_word;  // 0x10003f00+ overlay
            else if (virtio_net_sel)
               mmio_readdata_q <= virtio_net_readdata;
            else if (build_id_sel)
               mmio_readdata_q <= build_id_readdata;
            else
               mmio_readdata_q <= 32'd0;
         end
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

   // Under PROBE_CORE the bridge's core side is driven by the probe soc_top's virtio passthrough
   // at probe_clk (not the scalar core_clk); the async FIFOs handle probe_clk<->ui_clk CDC.
`ifdef PROBE_CORE
 `ifdef NO_VIRTIO_WIRE
   // Experiment: deactivate the virtio path -- bridge idle on core_clk (banner-build baseline).
   wire mmio_bridge_clk = core_clk;    wire mmio_bridge_rst = cpu_reset;
 `else
   wire mmio_bridge_clk = probe_clk;   wire mmio_bridge_rst = probe_reset;
 `endif
`else
   wire mmio_bridge_clk = core_clk;    wire mmio_bridge_rst = cpu_reset;
`endif
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
      .config_capacity_sectors (virtio_blk_capacity),
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

   virtio_mmio #(
      .DEVICE_ID(32'd1), /* Network device with a minimal TX-drop backend. */
      .QUEUE_NUM_MAX(32'd256), /* virtio-net needs > MAX_SKB_FRAGS+2 (=19) TX slots */
      .QUEUE_COUNT(32'd2)
   ) virtio_net_inst(
      .config_capacity_sectors (32'd0),
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
   wire [15:0] eth_rx_drop_count;

   // Reset for the gmii_rx_clk domain: synchronize the CPU reset in.  If the
   // PHY isn't supplying rxc (no link) the domain simply stays in reset.
   (* async_reg = "true" *) reg [1:0] gmii_rst_sync = 2'b11;
   always @(posedge gmii_rx_clk or posedge ui_cpu_reset)
      if (ui_cpu_reset) gmii_rst_sync <= 2'b11;
      else              gmii_rst_sync <= {gmii_rst_sync[0], 1'b0};
   wire gmii_rst = gmii_rst_sync[1];

   gmii_to_rgmii gmii_to_rgmii_inst(
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
   eth_rx_engine #(.BUF_BYTES(1536)) eth_rx_engine_inst(
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
      .drop_count  (eth_rx_drop_count)
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
   reg [31:0] virtio_net_debug_word;
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
        6'd23: virtio_net_debug_word = rx_dbg_head_ui[31:0];   // RX bytes 0..3
        6'd24: virtio_net_debug_word = rx_dbg_head_ui[63:32];  // RX bytes 4..7
        6'd25: virtio_net_debug_word = {20'd0, rx_dbg_lg_ui};  // {good, len} of last RX
        default: virtio_net_debug_word = 32'd0;
      endcase
   end

   generate
   if (USE_DDR_ARB) begin : gen_ddr_arbiter
   // Device-side arbiter: merge virtio-net (s0) and virtio-blk (s1) into one
   // master before the core-vs-device arbiter below. Reusing the proven
   // 2-master arbiter twice keeps each arbiter 2-input (timing-friendly) instead
   // of growing a 3-master mux on the DDR path.
   axi_two_master_arbiter device_arbiter_inst(
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
   wire [2:0]  mig_awid;   wire [30:0] mig_awaddr; wire [7:0] mig_awlen; wire [2:0] mig_awsize;
   wire [1:0]  mig_awburst; wire mig_awlock; wire [3:0] mig_awcache; wire [2:0] mig_awprot;
   wire [3:0]  mig_awqos;  wire mig_awvalid; wire mig_awready;
   wire [63:0] mig_wdata;  wire [7:0] mig_wstrb; wire mig_wlast; wire mig_wvalid; wire mig_wready;
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

`ifndef PROBE_CORE
   smolrv64 smolrv64_inst(
      .clock                (core_clk),
      .mem_clock            (ui_clk),
      .fpu_clock            (fpu_clk),
      .reset                (cpu_reset),
      .mmio_address         (core_mmio_address),
      .mmio_read            (core_mmio_read),
      .mmio_write           (core_mmio_write),
      .mmio_writedata       (core_mmio_writedata),
      .mmio_byteenable      (core_mmio_byteenable),
      .mmio_readdatavalid   (core_mmio_readdatavalid),
      .mmio_readdata        (core_mmio_readdata),

      .ext_irq              ({51'd0, virtio_net_irq_core, virtio_blk_irq_core, 10'd0}),

      .m_axi_awid           (core_axi_awid),
      .m_axi_awaddr         (core_axi_awaddr),
      .m_axi_awlen          (core_axi_awlen),
      .m_axi_awsize         (core_axi_awsize),
      .m_axi_awburst        (core_axi_awburst),
      .m_axi_awlock         (core_axi_awlock),
      .m_axi_awcache        (core_axi_awcache),
      .m_axi_awprot         (core_axi_awprot),
      .m_axi_awqos          (core_axi_awqos),
      .m_axi_awvalid        (core_axi_awvalid),
      .m_axi_awready        (core_axi_awready),
      .m_axi_wdata          (core_axi_wdata),
      .m_axi_wstrb          (core_axi_wstrb),
      .m_axi_wlast          (core_axi_wlast),
      .m_axi_wvalid         (core_axi_wvalid),
      .m_axi_wready         (core_axi_wready),
      .m_axi_bid            (core_axi_bid),
      .m_axi_bresp          (core_axi_bresp),
      .m_axi_bvalid         (core_axi_bvalid),
      .m_axi_bready         (core_axi_bready),
      .m_axi_arid           (core_axi_arid),
      .m_axi_araddr         (core_axi_araddr),
      .m_axi_arlen          (core_axi_arlen),
      .m_axi_arsize         (core_axi_arsize),
      .m_axi_arburst        (core_axi_arburst),
      .m_axi_arlock         (core_axi_arlock),
      .m_axi_arcache        (core_axi_arcache),
      .m_axi_arprot         (core_axi_arprot),
      .m_axi_arqos          (core_axi_arqos),
      .m_axi_arvalid        (core_axi_arvalid),
      .m_axi_arready        (core_axi_arready),
      .m_axi_rid            (core_axi_rid),
      .m_axi_rdata          (core_axi_rdata),
      .m_axi_rresp          (core_axi_rresp),
      .m_axi_rlast          (core_axi_rlast),
      .m_axi_rvalid         (core_axi_rvalid),
      .m_axi_rready         (core_axi_rready),

      .uart_tx_valid        (uart_tx_valid),
      .uart_tx_data         (uart_tx_data),
      .uart_tx_ready        (tx_ready),
      .uart_rx_valid        (rx_valid),
      .uart_rx_data         (rx_data),

      .halted_o             (halted)
   );

   // Core clock is half of the ~333.33 MHz UI clock; keep UART at 3 Mbaud.
   wire tx_ready;
   rs232tx #(.CLK_FREQ(166_666_666), .BAUD(3_000_000)) rs232tx_inst
     (.clk(core_clk), .rst_n(~cpu_reset),
      .data(uart_tx_data), .valid(uart_tx_valid), .ready(tx_ready),
      .tx(txd));

   rs232rx #(.CLK_FREQ(166_666_666), .BAUD(3_000_000)) rs232rx_inst
     (.clk(core_clk), .rst_n(~cpu_reset),
      .data(rx_data), .valid(rx_valid), .ready(1'b1),
      .rxd(rxd), .overflow());
`else
   // ===================== Sharded-OoO probe core (PROBE_CORE) =====================
   // soc_top (core + I$/D$ + CLINT/PLIC/UART + boot SRAM) at probe_clk; its 512-bit
   // line port -> ddr_line_cdc -> ddr_line_axi -> the existing core_axi_* arbiter input
   // (unchanged MIG path). virtio/ethernet/MMIO-bridge stay but idle (trimmed DTB).
   wire        pddr_req, pddr_we;  wire [57:0] pddr_addr;  wire [511:0] pddr_wdata, pddr_rdata;  wire pddr_ack;
   wire        mddr_req, mddr_we;  wire [57:0] mddr_addr;  wire [511:0] mddr_wdata, mddr_rdata;  wire mddr_ack;
   wire        ptx_valid;  wire [7:0] ptx_data;  wire ptx_ready;
   wire        prx_valid;  wire [7:0] prx_data;

   wire [12:0] p_virtio_addr;  wire p_virtio_read, p_virtio_write;   // bit12: blk(0)/net(1)
   wire [31:0] p_virtio_wdata; wire [3:0] p_virtio_be;
   // virtio_blk IRQ (ui_clk) synchronized into probe_clk for soc_top's internal PLIC (src 11).
   (* async_reg = "true" *) reg p_virtio_irq_meta = 1'b0, p_virtio_irq = 1'b0;
   always @(posedge probe_clk) begin p_virtio_irq_meta <= virtio_blk_irq; p_virtio_irq <= p_virtio_irq_meta; end

   wire [17:0] probe_irq_dbg;   // interrupt-path debug (probe_clk) for ILA_IRQ
   wire        core_commit;     // retire pulse (probe_clk) for ILA_CORE
   soc_top #(.RESET_PC(64'h7000_0000)) probe_core (
      .clk(probe_clk), .reset(probe_reset),
      .commit(core_commit), .dmem_wen(), .dmem_waddr(), .dmem_wdata(), .dmem_wmask(),
      .ddr_req(pddr_req), .ddr_we(pddr_we), .ddr_addr(pddr_addr), .ddr_wdata(pddr_wdata),
      .ddr_rdata(pddr_rdata), .ddr_ack(pddr_ack),
      .uart_rx_we(prx_valid), .uart_rx_data(prx_data), .uart_rx_ready(),
      .uart_tx_valid(ptx_valid), .uart_tx_data(ptx_data), .uart_tx_ready(ptx_ready),
      .irq_dbg(probe_irq_dbg),
      // virtio-blk MMIO passthrough -> mmio_clock_bridge core side (probe_clk) -> virtio_blk
      .virtio_addr(p_virtio_addr), .virtio_read(p_virtio_read), .virtio_write(p_virtio_write),
      .virtio_wdata(p_virtio_wdata), .virtio_be(p_virtio_be),
`ifdef NO_VIRTIO_WIRE
      .virtio_rdata(32'd0), .virtio_rvalid(1'b0),
      .virtio_irq(1'b0));
`else
      .virtio_rdata(core_mmio_readdata), .virtio_rvalid(core_mmio_readdatavalid),
      .virtio_irq(p_virtio_irq));
`endif

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
   //   - probe1={req,we,ack} stuck at req=1,ack=0  -> DDR-line path deadlock (CDC/bridge/skid)
   //   - probe0 commit frozen + probe3 irq pending, never claimed -> interrupt livelock
   //   - probe0 commit still toggling -> core alive, spinning in userspace (SW / rare-insn)
   //   probe2 = pddr_addr[23:0]: which line the LSU/cache is blocked on (region id).
   ila_core u_ila_core (
      .clk    (probe_clk),
      .probe0 (core_commit),                 // 1: retire pulse
      .probe1 ({pddr_req, pddr_we, pddr_ack}),// 3: line handshake
      .probe2 (pddr_addr[23:0]),             // 24: stuck line address (low bits)
      .probe3 (probe_irq_dbg)                // 18: interrupt-path bus (reused decode)
   );
`endif

   ddr_line_cdc probe_cdc (
      .clk_p(probe_clk), .reset_p(probe_reset),
      .p_req(pddr_req), .p_we(pddr_we), .p_addr(pddr_addr), .p_wdata(pddr_wdata),
      .p_rdata(pddr_rdata), .p_ack(pddr_ack),
      .clk_m(ui_clk), .reset_m(ui_cpu_reset),
      .m_req(mddr_req), .m_we(mddr_we), .m_addr(mddr_addr), .m_wdata(mddr_wdata),
      .m_rdata(mddr_rdata), .m_ack(mddr_ack));

   ddr_line_axi probe_bridge (
      .clk(ui_clk), .reset(ui_cpu_reset),
      .ddr_req(mddr_req), .ddr_we(mddr_we), .ddr_addr(mddr_addr), .ddr_wdata(mddr_wdata),
      .ddr_rdata(mddr_rdata), .ddr_ack(mddr_ack),
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
   // rs232tx.ready (output, ready-to-accept) feeds soc_top.uart_tx_ready directly.
   rs232tx #(.CLK_FREQ(66_666_666), .BAUD(3_000_000)) probe_tx
     (.clk(probe_clk), .rst_n(~probe_reset),
      .data(ptx_data), .valid(ptx_valid), .ready(ptx_ready), .tx(txd));
   rs232rx #(.CLK_FREQ(66_666_666), .BAUD(3_000_000)) probe_rx
     (.clk(probe_clk), .rst_n(~probe_reset),
      .data(prx_data), .valid(prx_valid), .ready(1'b1), .rxd(rxd), .overflow());

   // Drive the MMIO bridge core side from the probe soc_top's virtio passthrough. soc_top emits
   // the 13-bit offset within its 0x1000_2000 virtio region (8 KiB); addr[12] selects the device
   // (0 -> blk page 0x02, 1 -> net page 0x03) so ui_mmio_address[19:12] routes it to virtio_blk_inst
   // (0x02) or virtio_net_inst (0x03). virtio_net_irq is already wired to PLIC src 12 (ext_irq[11]).
`ifdef NO_VIRTIO_WIRE
   assign core_mmio_address    = 20'd0;
   assign core_mmio_read       = 1'b0;
   assign core_mmio_write      = 1'b0;
   assign core_mmio_writedata  = 32'd0;
   assign core_mmio_byteenable = 4'd0;
`else
   assign core_mmio_address    = {p_virtio_addr[12] ? 8'h03 : 8'h02, p_virtio_addr[11:0]};
   assign core_mmio_read       = p_virtio_read;
   assign core_mmio_write      = p_virtio_write;
   assign core_mmio_writedata  = p_virtio_wdata;
   assign core_mmio_byteenable = p_virtio_be;
`endif
`endif
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
   // signals routing from cpu_reset_core_sync (core_clk domain) through
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
                    ui_state <= UI_SEND_RSP;   // writes send a completion too, so soc_top can BLOCK the
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
