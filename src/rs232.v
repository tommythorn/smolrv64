// Minimal RS232 TX and RX modules for bridging byte-level interfaces
// to physical serial pins.

`timescale 1ns/10ps
`default_nettype none

module rs232tx #(parameter CLK_FREQ = 25_000_000,
                 parameter BAUD     = 115200)
   (input wire       clk,
    input wire       rst_n,
    input wire [7:0] data,
    input wire       valid,
    output reg       ready = 1,
    output reg       tx = 1);

   localparam DIVISOR = CLK_FREQ / BAUD;
   reg [15:0] baud_cnt = 0;
   reg [ 3:0] bit_cnt  = 0;
   reg [ 9:0] shift    = ~0; // idle = all ones

   always @(posedge clk) begin
      if (!rst_n) begin
         ready    <= 1;
         tx       <= 1;
         baud_cnt <= 0;
         bit_cnt  <= 0;
         shift    <= ~0;
      end else if (ready && valid) begin
         shift    <= {1'b1, data, 1'b0}; // stop + data + start
         bit_cnt  <= 10;
         baud_cnt <= DIVISOR - 1;
         ready    <= 0;
         tx       <= 0; // start bit immediately
      end else if (bit_cnt != 0) begin
         if (baud_cnt == 0) begin
            shift    <= {1'b1, shift[9:1]};
            tx       <= shift[1];
            bit_cnt  <= bit_cnt - 1;
            baud_cnt <= DIVISOR - 1;
            if (bit_cnt == 1)
               ready <= 1;
         end else
            baud_cnt <= baud_cnt - 1;
      end
   end
endmodule

module rs232rx #(parameter CLK_FREQ = 25_000_000,
                 parameter BAUD     = 115200)
   (input wire       clk,
    input wire       rst_n,
    output reg [7:0] data,
    output reg       valid = 0,
    input wire       rxd);

   localparam DIVISOR = CLK_FREQ / BAUD;
   reg [15:0] baud_cnt = 0;
   reg [ 3:0] bit_cnt  = 0;
   reg [ 7:0] shift    = 0;
   reg        rx_sync1 = 1, rx_sync2 = 1;

   always @(posedge clk) begin
      rx_sync1 <= rxd;
      rx_sync2 <= rx_sync1;
      valid    <= 0;

      if (!rst_n) begin
         baud_cnt <= 0;
         bit_cnt  <= 0;
         rx_sync1 <= 1;
         rx_sync2 <= 1;
      end else if (bit_cnt == 0) begin
         // Wait for start bit (falling edge)
         if (!rx_sync2) begin
            baud_cnt <= DIVISOR / 2 - 1; // sample at middle
            bit_cnt  <= 9; // 8 data + stop
         end
      end else begin
         if (baud_cnt == 0) begin
            baud_cnt <= DIVISOR - 1;
            bit_cnt  <= bit_cnt - 1;
            if (bit_cnt == 1) begin
               // Stop bit — output the byte
               data  <= shift;
               valid <= 1;
            end else begin
               shift <= {rx_sync2, shift[7:1]};
            end
         end else
            baud_cnt <= baud_cnt - 1;
      end
   end
endmodule
