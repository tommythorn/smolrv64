module top(input        clk_25mhz,
           input [6:0]  btn,
           input        ftdi_txd,
           output       ftdi_rxd,
           output [7:0] led,
           output       wifi_gpio0);

   reg [7:0] led_r;

   // Tie GPIO0, keep board from rebooting
   assign wifi_gpio0    = 1;
   // LEDs use positive logic, eg. 1 is ON
   assign led           = led_r;

   localparam W = 26;  // log2(25 MHz) = 24.6
   reg [W-1:0] ctr = 0;

   wire        tx_ready_o;
   wire        tx_valid_i;
   wire [7:0]  tx_data_i;

   wire        rx_ready_i;
   wire        rx_valid_o;
   wire [7:0]  rx_data_o;
   wire        rx_overflow_o;

   wire        halted;

   // On macOS, only speeds up to B230400 are defined in termios.h
   // Curiously macOS/FTDI works at 460800, but higher rates do not.
   rs232tx #(25000000,115200) rs232tx_inst
     (clk_25mhz, tx_data_i, tx_valid_i, tx_ready_o, ftdi_rxd);

   rs232rx #(25000000,115200) rs232rx_inst
     (clk_25mhz, rx_data_o, rx_valid_o, rx_ready_i, ftdi_txd, rx_overflow_o);

   smolrv64 smolrv64_inst(.clock     (clk_25mhz),
                          .tx_ready_i(tx_ready_o),
                          .tx_valid_o(tx_valid_i),
                          .tx_data_o (tx_data_i),
                          .halted_o  (halted));

   always @(posedge clk_25mhz) ctr <= ctr + tx_valid_i;
   always @(posedge clk_25mhz) led_r <= (ctr[W-1:W-8] | {4{halted}}) ^ {8{!btn[0]}};
endmodule
