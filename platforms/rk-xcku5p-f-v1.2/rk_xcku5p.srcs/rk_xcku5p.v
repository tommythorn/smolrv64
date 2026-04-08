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

   // CPU is held in reset until calibration completes
   wire cpu_reset = ui_rst | ~init_calib_complete;

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

   // DDR4 native app interface wires
   wire [28:0]  c0_ddr4_app_addr;
   wire [2:0]   c0_ddr4_app_cmd;
   wire         c0_ddr4_app_en;
   wire [255:0] c0_ddr4_app_wdf_data;
   wire         c0_ddr4_app_wdf_end;
   wire [31:0]  c0_ddr4_app_wdf_mask;
   wire         c0_ddr4_app_wdf_wren;
   wire [255:0] c0_ddr4_app_rd_data;
   wire         c0_ddr4_app_rd_data_end;
   wire         c0_ddr4_app_rd_data_valid;
   wire         c0_ddr4_app_rdy;
   wire         c0_ddr4_app_wdf_rdy;
   wire         dbg_clk;
   wire [511:0] dbg_bus;
   wire         c0_ddr4_reset_n_int;
   assign c0_ddr4_reset_n = c0_ddr4_reset_n_int;

   // DDR4 MIG IP instantiation
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

      .c0_ddr4_app_addr               (c0_ddr4_app_addr),
      .c0_ddr4_app_cmd                (c0_ddr4_app_cmd),
      .c0_ddr4_app_en                 (c0_ddr4_app_en),
      .c0_ddr4_app_hi_pri             (1'b0),
      .c0_ddr4_app_wdf_data           (c0_ddr4_app_wdf_data),
      .c0_ddr4_app_wdf_end            (c0_ddr4_app_wdf_end),
      .c0_ddr4_app_wdf_mask           (c0_ddr4_app_wdf_mask),
      .c0_ddr4_app_wdf_wren           (c0_ddr4_app_wdf_wren),
      .c0_ddr4_app_rd_data            (c0_ddr4_app_rd_data),
      .c0_ddr4_app_rd_data_end        (c0_ddr4_app_rd_data_end),
      .c0_ddr4_app_rd_data_valid      (c0_ddr4_app_rd_data_valid),
      .c0_ddr4_app_rdy                (c0_ddr4_app_rdy),
      .c0_ddr4_app_wdf_rdy            (c0_ddr4_app_wdf_rdy),
      .dbg_bus                        (dbg_bus)
   );

   // CPU DRAM bus signals
   wire [25:0]  dram_burst_addr;
   wire         dram_read;
   wire         dram_write;
   wire [255:0] dram_writedata;
   wire [31:0]  dram_byte_mask;
   wire         dram_readdatavalid;
   wire [255:0] dram_readdata;
   wire         dram_write_ready;

   // DDR4 adapter: bridges CPU DRAM bus to MIG native app interface
   ddr4_adapter ddr4_adapter_inst (
      .clk                  (ui_clk),
      .rst_n                (~ui_rst),

      .dram_burst_addr      (dram_burst_addr),
      .dram_read            (dram_read),
      .dram_write           (dram_write),
      .dram_writedata       (dram_writedata),
      .dram_byte_mask       (dram_byte_mask),
      .dram_readdatavalid   (dram_readdatavalid),
      .dram_readdata        (dram_readdata),
      .dram_write_ready     (dram_write_ready),

      .app_addr             (c0_ddr4_app_addr),
      .app_cmd              (c0_ddr4_app_cmd),
      .app_en               (c0_ddr4_app_en),
      .app_rdy              (c0_ddr4_app_rdy),
      .app_wdf_data         (c0_ddr4_app_wdf_data),
      .app_wdf_end          (c0_ddr4_app_wdf_end),
      .app_wdf_mask         (c0_ddr4_app_wdf_mask),
      .app_wdf_wren         (c0_ddr4_app_wdf_wren),
      .app_wdf_rdy          (c0_ddr4_app_wdf_rdy),
      .app_rd_data          (c0_ddr4_app_rd_data),
      .app_rd_data_end      (c0_ddr4_app_rd_data_end),
      .app_rd_data_valid    (c0_ddr4_app_rd_data_valid)
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

      .dram_burst_addr      (dram_burst_addr),
      .dram_read            (dram_read),
      .dram_write           (dram_write),
      .dram_writedata       (dram_writedata),
      .dram_byte_mask       (dram_byte_mask),
      .dram_readdatavalid   (dram_readdatavalid),
      .dram_readdata        (dram_readdata),
      .dram_write_ready     (dram_write_ready),

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
