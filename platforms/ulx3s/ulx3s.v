`timescale 1ns/10ps
`default_nettype none

module top(input  wire      clk_25mhz,
           input  wire [6:0] btn,
           input  wire      ftdi_txd,
           output wire      ftdi_rxd,
           output reg [7:0] led,
           output wire      wifi_gpio0);

   // Tie GPIO0, keep board from rebooting
   assign               wifi_gpio0 = 1;
   wire                 clock = clk_25mhz;
   wire                 halted;
   wire [19:0]          mmio_address;
   wire                 mmio_read;
   wire                 mmio_write;
   wire [31:0]          mmio_writedata;
   wire [ 3:0]          mmio_byteenable;
   wire                 mmio_readdatavalid;
   wire [31:0]          mmio_readdata;

   reg [10:0] reset_counter = 10;
   wire       reset = reset_counter != 0;

   // Internal UART TX/RX byte interface
   wire       uart_tx_valid;
   wire [7:0] uart_tx_data;
   wire       rx_valid;
   wire [7:0] rx_data;

   smolrv64 smolrv64_inst(.clock                (clock),
                          .fpu_clock            (clock),
                          .reset                (reset),
                          .mmio_address         (mmio_address),
                          .mmio_read            (mmio_read),
                          .mmio_write           (mmio_write),
                          .mmio_writedata       (mmio_writedata),
                          .mmio_byteenable      (mmio_byteenable),
                          .mmio_readdatavalid   (mmio_readdatavalid),
                          .mmio_readdata        (mmio_readdata),

                          .ext_irq              (63'd0),

                          .uart_tx_valid        (uart_tx_valid),
                          .uart_tx_data         (uart_tx_data),
                          .uart_rx_valid        (rx_valid),
                          .uart_rx_data         (rx_data),

                          .halted_o             (halted));

   // RS232 TX: byte interface -> serial pin
   wire       tx_ready;
   rs232tx #(.CLK_FREQ(25_000_000), .BAUD(115200)) rs232tx_inst
     (.clk(clock), .rst_n(!reset),
      .data(uart_tx_data), .valid(uart_tx_valid), .ready(tx_ready),
      .tx(ftdi_rxd)); // ftdi_rxd = our TX output

   // RS232 RX: serial pin -> byte interface
   rs232rx #(.CLK_FREQ(25_000_000), .BAUD(115200)) rs232rx_inst
     (.clk(clock), .rst_n(!reset),
      .data(rx_data), .valid(rx_valid),
      .rxd(ftdi_txd)); // ftdi_txd = our RX input

   reg [31:0] counter;
   reg        prev_bit = 1, prev_write = 0;
   reg [ 7:0] bits = 0, writes = 0;

   always @(posedge clk_25mhz) begin
      if (reset_counter != 0)
        reset_counter <= reset_counter - 1;

      counter <= counter + 1;
      prev_bit   <= ftdi_rxd;
      prev_write <= mmio_write;
      bits    <= bits + (ftdi_rxd != prev_bit);
      writes  <= writes + (mmio_write != prev_write);
      led     <= reset  ? 'h5A           :
                 btn[1] ? bits           :
                 writes;

      if (reset) begin
         bits <= 0;
         writes <= 0;
      end

      if (btn[0] == 0) begin
         reset_counter <= !0;
      end
   end
endmodule
