`timescale 1ns/10ps
`default_nettype none

module ulx3s_tb;
   reg        clock = 1; always #5 clock = !clock;

   wire [6:0] btn;
   wire       ftdi_txd;
   wire       ftdi_rxd;
   wire [7:0] led;
   wire       wifi_gpio0;

   top top_inst(clock, btn, ftdi_txd, ftdi_rxd, led, wifi_gpio0);
   initial begin
      $dumpfile("ulx3s_tb.vcd");
      $dumpvars(0, top_inst);
      $display("Open the ulx3s_tb.vcd with https://app.surfer-project.org/");

      #40000000
      $finish;
   end
endmodule
