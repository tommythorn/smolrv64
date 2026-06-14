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
    output wire       sd_clk,
    output wire       sd_cmd,
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

   // Keep board LEDs dark by default; status is available through the monitor.
   assign led = 4'b0000;

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
      .c0_ddr4_s_axi_awid             (m_axi_awid),
      .c0_ddr4_s_axi_awaddr           (m_axi_awaddr),
      .c0_ddr4_s_axi_awlen            (m_axi_awlen),
      .c0_ddr4_s_axi_awsize           (m_axi_awsize),
      .c0_ddr4_s_axi_awburst          (m_axi_awburst),
      .c0_ddr4_s_axi_awlock           (m_axi_awlock),
      .c0_ddr4_s_axi_awcache          (m_axi_awcache),
      .c0_ddr4_s_axi_awprot           (m_axi_awprot),
      .c0_ddr4_s_axi_awqos            (m_axi_awqos),
      .c0_ddr4_s_axi_awvalid          (m_axi_awvalid),
      .c0_ddr4_s_axi_awready          (m_axi_awready),
      .c0_ddr4_s_axi_wdata            (m_axi_wdata),
      .c0_ddr4_s_axi_wstrb            (m_axi_wstrb),
      .c0_ddr4_s_axi_wlast            (m_axi_wlast),
      .c0_ddr4_s_axi_wvalid           (m_axi_wvalid),
      .c0_ddr4_s_axi_wready           (m_axi_wready),
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

   wire        spi_sd_clk;
   wire        spi_sd_mosi;
   wire        spi_sd_miso;
   wire [ 3:0] sd_d_i;
   wire [ 3:0] sd_d_o;
   wire [ 3:0] sd_d_t;
   wire [ 7:0] sd_gpio;
   reg         sd_cd_meta = 1'b1;
   reg         sd_cd_sync = 1'b1;

   assign sd_clk  = spi_sd_clk;
   assign sd_cmd  = spi_sd_mosi;
   assign spi_sd_miso = sd_d_i[0];
   assign sd_d_o = {sd_gpio[0], 1'b1, 1'b1, 1'b1};
   assign sd_d_t = 4'b0001;

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

   wire        sd_spi_sel    = ui_mmio_address[19:8] == 12'h010;
   wire        sd_gpio_sel   = ui_mmio_address[19:8] == 12'h011;
   wire        sd_cd_gpio_sel = ui_mmio_address[19:8] == 12'h012;
   wire        virtio_blk_sel = ui_mmio_address[19:12] == 8'h02;
   wire        virtio_net_sel = ui_mmio_address[19:12] == 8'h03;
   wire        build_id_sel   = ui_mmio_address[19:8] == 12'h0f0;
   wire [31:0] sd_spi_readdata;
   wire [31:0] sd_gpio_readdata;
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
   wire [ 7:0] virtio_net_device_status;
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
      end else begin
         sd_cd_meta <= sd_cd;
         sd_cd_sync <= sd_cd_meta;
         mmio_read_d1 <= ui_mmio_read;
         mmio_read_d2 <= mmio_read_d1;
         if (ui_mmio_read) begin
            if (sd_spi_sel)
               mmio_readdata_q <= sd_spi_readdata;
            else if (sd_gpio_sel)
               mmio_readdata_q <= sd_gpio_readdata;
            else if (sd_cd_gpio_sel)
               mmio_readdata_q <= sd_cd_gpio_readdata;
            else if (virtio_blk_sel)
               mmio_readdata_q <= virtio_blk_readdata;
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

   smolrv64_mmio_clock_bridge mmio_clock_bridge_inst(
      .core_clock          (core_clk),
      .core_reset          (cpu_reset),
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

   sd_spi_oc_tiny sd_spi_inst(
      .clock        (ui_clk),
      .reset        (ui_cpu_reset),
      .address      (ui_mmio_address[7:0]),
      .read_data    (sd_spi_readdata),
      .read         (ui_mmio_read && sd_spi_sel),
      .write        (ui_mmio_write && sd_spi_sel),
      .write_data   (ui_mmio_writedata),
      .byteenable   (ui_mmio_byteenable),
      .spi_clk      (spi_sd_clk),
      .spi_mosi     (spi_sd_mosi),
      .spi_miso     (spi_sd_miso)
   );

   sd_gpio_dat sd_gpio_inst(
      .clock        (ui_clk),
      .reset        (ui_cpu_reset),
      .write        (ui_mmio_write && sd_gpio_sel),
      .write_data   (ui_mmio_writedata),
      .byteenable   (ui_mmio_byteenable),
      .gpio_out     (sd_gpio),
      .read_data    (sd_gpio_readdata)
   );

   /* virtio-net TX debug counters removed (TX is fixed). The engine's debug_*
    * outputs now drive nothing and are stripped by synthesis (DCE). */

   virtio_mmio #(
      .DEVICE_ID(32'd0), /* Dormant until a block backend can complete queues. */
      .QUEUE_NUM_MAX(32'd8)
   ) virtio_blk_inst(
      .clock                   (ui_clk),
      .reset                   (ui_cpu_reset),
      .address                 (ui_mmio_address[11:0]),
      .read                    (ui_mmio_read && virtio_blk_sel),
      .read_data               (virtio_blk_readdata),
      .write                   (ui_mmio_write && virtio_blk_sel),
      .write_data              (ui_mmio_writedata),
      .byteenable              (ui_mmio_byteenable),
      .irq                     (virtio_blk_irq),
      .queue_notify_pulse      (),
      .queue_notify_value      (),
      .used_buffer_interrupt   (1'b0),
      .config_change_interrupt (1'b0),
      .driver_features_0       (),
      .driver_features_1       (),
      .queue_num               (),
      .queue_ready             (),
      .queue_desc              (),
      .queue_driver            (),
      .queue_device            (),
      .queue0_num              (),
      .queue0_ready            (),
      .queue0_desc             (),
      .queue0_driver           (),
      .queue0_device           (),
      .queue1_num              (),
      .queue1_ready            (),
      .queue1_desc             (),
      .queue1_driver           (),
      .queue1_device           (),
      .device_status           ()
   );

   virtio_mmio #(
      .DEVICE_ID(32'd1), /* Network device with a minimal TX-drop backend. */
      .QUEUE_NUM_MAX(32'd256), /* virtio-net needs > MAX_SKB_FRAGS+2 (=19) TX slots */
      .QUEUE_COUNT(32'd2)
   ) virtio_net_inst(
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
      .queue0_num              (),
      .queue0_ready            (),
      .queue0_desc             (),
      .queue0_driver           (),
      .queue0_device           (),
      .queue1_num              (virtio_net_queue1_num),
      .queue1_ready            (virtio_net_queue1_ready),
      .queue1_desc             (virtio_net_queue1_desc),
      .queue1_driver           (virtio_net_queue1_driver),
      .queue1_device           (virtio_net_queue1_device),
      .device_status           (virtio_net_device_status)
   );

   virtio_net_tx_drop #(
      .QUEUE_SIZE(32'd256)    /* must match virtio_net_inst QUEUE_NUM_MAX */
   ) virtio_net_tx_drop_inst(
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

   wire        gmii_rx_clk;
   wire        gmii_rx_dv;
   wire [ 7:0] gmii_rxd;
   wire        gmii_tx_en;
   wire [ 7:0] gmii_txd;
   wire        eth_rx_valid;
   wire [ 7:0] eth_rx_data;
   wire        eth_rx_last;
   wire        eth_rx_good;

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

   // RX is deframed but not yet delivered to a virtqueue (VIRTIO_PLAN.md step
   // 6).  Keep the RX path (and the eth_rxd pins) synthesized and observable
   // for bring-up via these dont_touch counters in the gmii_rx_clk domain.
   (* dont_touch = "true" *) reg [15:0] eth_rx_good_cnt = 16'd0;
   (* dont_touch = "true" *) reg [15:0] eth_rx_bad_cnt  = 16'd0;
   (* dont_touch = "true" *) reg [ 7:0] eth_rx_last_byte = 8'd0;
   always @(posedge gmii_rx_clk) begin
      if (eth_rx_valid)               eth_rx_last_byte <= eth_rx_data;
      if (eth_rx_last &&  eth_rx_good) eth_rx_good_cnt <= eth_rx_good_cnt + 16'd1;
      if (eth_rx_last && !eth_rx_good) eth_rx_bad_cnt  <= eth_rx_bad_cnt  + 16'd1;
   end

   generate
   if (USE_DDR_ARB) begin : gen_ddr_arbiter
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
      .s0_axi_rid     (core_axi_rid),
      .s0_axi_rdata   (core_axi_rdata),
      .s0_axi_rresp   (core_axi_rresp),
      .s0_axi_rlast   (core_axi_rlast),
      .s0_axi_rvalid  (core_axi_rvalid),
      .s0_axi_rready  (core_axi_rready),

      .s1_axi_awid    (virtio_net_axi_awid),
      .s1_axi_awaddr  (virtio_net_axi_awaddr),
      .s1_axi_awlen   (virtio_net_axi_awlen),
      .s1_axi_awsize  (virtio_net_axi_awsize),
      .s1_axi_awburst (virtio_net_axi_awburst),
      .s1_axi_awlock  (virtio_net_axi_awlock),
      .s1_axi_awcache (virtio_net_axi_awcache),
      .s1_axi_awprot  (virtio_net_axi_awprot),
      .s1_axi_awqos   (virtio_net_axi_awqos),
      .s1_axi_awvalid (virtio_net_axi_awvalid),
      .s1_axi_awready (virtio_net_axi_awready),
      .s1_axi_wdata   (virtio_net_axi_wdata),
      .s1_axi_wstrb   (virtio_net_axi_wstrb),
      .s1_axi_wlast   (virtio_net_axi_wlast),
      .s1_axi_wvalid  (virtio_net_axi_wvalid),
      .s1_axi_wready  (virtio_net_axi_wready),
      .s1_axi_bid     (virtio_net_axi_bid),
      .s1_axi_bresp   (virtio_net_axi_bresp),
      .s1_axi_bvalid  (virtio_net_axi_bvalid),
      .s1_axi_bready  (virtio_net_axi_bready),
      .s1_axi_arid    (virtio_net_axi_arid),
      .s1_axi_araddr  (virtio_net_axi_araddr),
      .s1_axi_arlen   (virtio_net_axi_arlen),
      .s1_axi_arsize  (virtio_net_axi_arsize),
      .s1_axi_arburst (virtio_net_axi_arburst),
      .s1_axi_arlock  (virtio_net_axi_arlock),
      .s1_axi_arcache (virtio_net_axi_arcache),
      .s1_axi_arprot  (virtio_net_axi_arprot),
      .s1_axi_arqos   (virtio_net_axi_arqos),
      .s1_axi_arvalid (virtio_net_axi_arvalid),
      .s1_axi_arready (virtio_net_axi_arready),
      .s1_axi_rid     (virtio_net_axi_rid),
      .s1_axi_rdata   (virtio_net_axi_rdata),
      .s1_axi_rresp   (virtio_net_axi_rresp),
      .s1_axi_rlast   (virtio_net_axi_rlast),
      .s1_axi_rvalid  (virtio_net_axi_rvalid),
      .s1_axi_rready  (virtio_net_axi_rready),

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
