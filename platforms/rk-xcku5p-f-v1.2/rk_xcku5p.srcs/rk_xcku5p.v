`timescale 1ns / 1ps
`default_nettype none
module rk_xcku5p(
    input  wire       sys_clk_p,
    input  wire       sys_clk_n,
    input  wire [3:0] key,
    input  wire       rxd,
    output wire [3:0] led,
    output wire       txd
    );

   wire clk_200_MHz;
   IBUFGDS ibufgds_inst(.I(sys_clk_p), .IB(sys_clk_n), .O(clk_200_MHz));

   // Dividing by 10 M  means blinking at 20 Hz
   reg [31:0] count = 0;
   reg        toggle = 0;

   always @(posedge clk_200_MHz)
     if (count == 'd 10_000_000) begin
        toggle <= !toggle;
        count <= 0;
     end else
       count <= count + 1;

   // Quick test
   assign led[0] = key[0];
   assign led[1] = 1 ^ key[1];
   assign led[2] = 0 ^ key[2];
   assign led[3] = toggle ^ key[3];

   wire halted;
   wire       uart_tx_valid;
   wire [7:0] uart_tx_data;
   wire       rx_valid;
   wire [7:0] rx_data;

   smolrv64 smolrv64_inst(.clock          (clk_200_MHz),
                          .reset          (1'b0),
                          .mmio_address   (),
                          .mmio_read      (),
                          .mmio_write     (),
                          .mmio_writedata (),
                          .mmio_byteenable(),
                          .mmio_readdatavalid(1'b0),
                          .mmio_readdata  (32'd0),

                          .ext_irq        (63'd0),

                          .uart_tx_valid  (uart_tx_valid),
                          .uart_tx_data   (uart_tx_data),
                          .uart_rx_valid  (rx_valid),
                          .uart_rx_data   (rx_data),

                          .halted_o       (halted));

   // On macOS, only speeds up to B230400 are defined in termios.h
   // Curiously macOS/FTDI works at 460800, but higher rates do not.
   wire tx_ready;
   rs232tx #(.CLK_FREQ(200_000_000), .BAUD(3_000_000)) rs232tx_inst
     (.clk(clk_200_MHz), .rst_n(1'b1),
      .data(uart_tx_data), .valid(uart_tx_valid), .ready(tx_ready),
      .tx(txd));

   rs232rx #(.CLK_FREQ(200_000_000), .BAUD(3_000_000)) rs232rx_inst
     (.clk(clk_200_MHz), .rst_n(1'b1),
      .data(rx_data), .valid(rx_valid),
      .rxd(rxd));
endmodule
