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

   smolrv64 smolrv64_inst(.clock                (clock),
                          .reset                (reset),
                          .mmio_address         (mmio_address),
                          .mmio_read            (mmio_read),
                          .mmio_write           (mmio_write),
                          .mmio_writedata       (mmio_writedata),
                          .mmio_byteenable      (mmio_byteenable),
                          .mmio_readdatavalid   (mmio_readdatavalid),
                          .mmio_readdata        (mmio_readdata),

                          .halted_o             (halted));

   wire       mmio_waitrequest; // Currently ignored
   wire       uart5_irq; // Currently ignored

   reg [31:0] counter;
   reg        prev_bit = 1, prev_write = 0;
   reg [ 7:0] bits = 0, writes = 0;

   always @(posedge clk_25mhz) begin
      if (reset_counter != 0)
        reset_counter <= reset_counter - 1;
//    if (mmio_write && mmio_address == 0)
//      led <= write_data[7:0];

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

   uart5 uart5_inst(clock,
                    !reset,
                    mmio_address[4:2],
                    mmio_read,
                    mmio_write,
                    mmio_writedata,
                    mmio_readdatavalid,
                    mmio_readdata,
                    mmio_waitrequest,

                    ftdi_txd, // = input uart_rx, not a typo
                    ftdi_rxd, // = output uart_tx, not a typo
                    uart5_irq);
   defparam uart5_inst.CLK_FREQUENCY = 25_000_000;
endmodule
