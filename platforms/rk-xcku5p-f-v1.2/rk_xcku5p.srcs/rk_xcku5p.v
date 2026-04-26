`timescale 1ns / 1ps
`default_nettype none
module rk_xcku5p(
    // Differential 200 MHz system clock (fed directly to DDR4 IP)
    input  wire       sys_clk_p,
    input  wire       sys_clk_n,
    input  wire [3:0] key,
    input  wire       rxd,
    output wire [3:0] led,
    output wire       txd,

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
   wire cpu_reset = ui_rst | ~init_calib_complete | ~key[1];

   // Heartbeat counter driven by UI clock
   reg [34:0] count = 0;
   reg        toggle = 0;
   always @(posedge ui_clk)
     if (count == 'd 333_333_333) begin  // 0.5 Hz at 333.33 MHz
        toggle <= !toggle;
        count <= 0;
     end else
       count <= count + 1;

   // LED assignments
   assign led[0] = init_calib_complete; // goes high ~1s after power-on
   assign led[1] = toggle;              // heartbeat (UI clock domain)
   assign led[2] = halted;
   assign led[3] = key[3];

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

   wire halted;
   wire       uart_tx_valid;
   wire [7:0] uart_tx_data;
   wire       rx_valid;
   wire [7:0] rx_data;
   wire       mmio_read;

   smolrv64 smolrv64_inst(
      .clock                (ui_clk),
      .reset                (cpu_reset),
      .mmio_address         (),
      .mmio_read            (mmio_read),
      .mmio_write           (),
      .mmio_writedata       (),
      .mmio_byteenable      (),
      .mmio_readdatavalid   (mmio_read),
      .mmio_readdata        (32'd0),

      .ext_irq              (63'd0),

      .m_axi_awid           (m_axi_awid),
      .m_axi_awaddr         (m_axi_awaddr),
      .m_axi_awlen          (m_axi_awlen),
      .m_axi_awsize         (m_axi_awsize),
      .m_axi_awburst        (m_axi_awburst),
      .m_axi_awlock         (m_axi_awlock),
      .m_axi_awcache        (m_axi_awcache),
      .m_axi_awprot         (m_axi_awprot),
      .m_axi_awqos          (m_axi_awqos),
      .m_axi_awvalid        (m_axi_awvalid),
      .m_axi_awready        (m_axi_awready),
      .m_axi_wdata          (m_axi_wdata),
      .m_axi_wstrb          (m_axi_wstrb),
      .m_axi_wlast          (m_axi_wlast),
      .m_axi_wvalid         (m_axi_wvalid),
      .m_axi_wready         (m_axi_wready),
      .m_axi_bid            (m_axi_bid),
      .m_axi_bresp          (m_axi_bresp),
      .m_axi_bvalid         (m_axi_bvalid),
      .m_axi_bready         (m_axi_bready),
      .m_axi_arid           (m_axi_arid),
      .m_axi_araddr         (m_axi_araddr),
      .m_axi_arlen          (m_axi_arlen),
      .m_axi_arsize         (m_axi_arsize),
      .m_axi_arburst        (m_axi_arburst),
      .m_axi_arlock         (m_axi_arlock),
      .m_axi_arcache        (m_axi_arcache),
      .m_axi_arprot         (m_axi_arprot),
      .m_axi_arqos          (m_axi_arqos),
      .m_axi_arvalid        (m_axi_arvalid),
      .m_axi_arready        (m_axi_arready),
      .m_axi_rid            (m_axi_rid),
      .m_axi_rdata          (m_axi_rdata),
      .m_axi_rresp          (m_axi_rresp),
      .m_axi_rlast          (m_axi_rlast),
      .m_axi_rvalid         (m_axi_rvalid),
      .m_axi_rready         (m_axi_rready),

      .uart_tx_valid        (uart_tx_valid),
      .uart_tx_data         (uart_tx_data),
      .uart_tx_ready        (tx_ready),
      .uart_rx_valid        (rx_valid),
      .uart_rx_data         (rx_data),

      .halted_o             (halted)
   );

   // UI clock is ~333.33 MHz; keep UART at 3 Mbaud
   wire tx_ready;
   rs232tx #(.CLK_FREQ(333_333_333), .BAUD(3_000_000)) rs232tx_inst
     (.clk(ui_clk), .rst_n(~ui_rst),
      .data(uart_tx_data), .valid(uart_tx_valid), .ready(tx_ready),
      .tx(txd));

   rs232rx #(.CLK_FREQ(333_333_333), .BAUD(3_000_000)) rs232rx_inst
     (.clk(ui_clk), .rst_n(~ui_rst),
      .data(rx_data), .valid(rx_valid), .ready(1'b1),
      .rxd(rxd));
endmodule
