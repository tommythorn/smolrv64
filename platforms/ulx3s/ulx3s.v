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

   wire [19:0]          mmio_address;
   wire                 mmio_read;
   wire                 mmio_write;
   wire [31:0]          mmio_writedata;
   wire [ 3:0]          mmio_byteenable;
   wire                 mmio_readdatavalid;
   wire [31:0]          mmio_readdata;

   smolrv64 smolrv64_inst(.clock                (clock),

                          .mmio_address         (mmio_address),
                          .mmio_read            (mmio_read),
                          .mmio_write           (mmio_write),
                          .mmio_writedata       (mmio_writedata),
                          .mmio_byteenable      (mmio_byteenable),
                          .mmio_readdatavalid   (mmio_readdatavalid),
                          .mmio_readdata        (mmio_readdata),

                          .halted_o             (halted));

   reg        reset_n = 0;
   wire       mmio_waitrequest; // Currently ignored
   wire       uart5_irq; // Currently ignored

   always @(posedge clk_25mhz) reset_n = 1;

   uart5 uart5_inst(clock, reset_n,
                    mmio_address[4:2],
                    mmio_read, mmio_write, mmio_writedata, mmio_readdata, mmio_waitrequest,

                    ftdi_txd, // = uart_rx, not a typo
                    ftdi_rxd, // = uart_tx, not a typo
                    uart5_irq);

   always @(posedge clk_25mhz) ctr <= ctr + ftdi_rxd;
   always @(posedge clk_25mhz) led_r <= (ctr[W-1:W-8] | {4{halted}}) ^ {8{!btn[0]}};
endmodule
