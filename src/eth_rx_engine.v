`timescale 1ns / 1ps
`default_nettype none

// RX engine: bridges eth_mac_rx (gmii = PHY RX-clock domain) to the virtio_net
// backend (ui_clk).  eth_mac_rx streams a received L2 frame in; on a good
// frame the engine latches it in an async-read buffer and raises frame_valid
// to the backend, which reads the bytes out and DMAs them into a guest RX
// buffer, then pulses frame_ack to release the engine for the next frame.
//
// Single-buffered: a frame arriving while one is still un-acked is dropped
// (counted).  The buffer is the payload CDC; frame-ready/ack cross via toggle
// synchronizers; frame_len is held stable while frame_valid.
module eth_rx_engine #(
    parameter integer BUF_BYTES = 1536
)(
    // gmii_clk (PHY RX-clock) domain -- from eth_mac_rx
    input  wire        gmii_clk,
    input  wire        gmii_rst,
    input  wire        rx_valid,
    input  wire [ 7:0] rx_data,
    input  wire        rx_last,
    input  wire        rx_good,

    // ui_clk (virtio backend) domain
    input  wire        ui_clk,
    input  wire        ui_rst,
    output reg         frame_valid,   // a good frame is ready to read
    output reg  [10:0] frame_len,     // its length (stable while frame_valid)
    input  wire [10:0] rd_addr,       // backend reads frame bytes
    output wire [ 7:0] rd_data,       // async-read buffer
    input  wire        frame_ack,     // 1-cycle: done, release the buffer
    output reg  [15:0] drop_count     // frames dropped while busy (ui domain)
    );

   // ---- async-read frame buffer (gmii write / ui read) -------------------
   (* ram_style = "distributed" *)
   reg [7:0] rx_buf [0:BUF_BYTES-1];
   /* verilator lint_off WIDTHEXPAND */
   assign rd_data = rx_buf[rd_addr];
   /* verilator lint_on WIDTHEXPAND */

   // ---- gmii-domain capture ---------------------------------------------
   reg        holding;        // a frame is captured, awaiting ui ack
   reg [10:0] wptr;           // bytes captured so far
   reg [10:0] len_g;          // captured frame length (read in ui while held)
   reg        done_tgl;       // toggles on each good captured frame
   reg [15:0] drop_g;
   reg [2:0]  ack_sync;       // ui ack_tgl -> gmii
   wire       ack_pulse_g = ack_sync[2] ^ ack_sync[1];

   always @(posedge gmii_clk) begin
      if (gmii_rst) begin
         holding <= 1'b0; wptr <= 11'd0; len_g <= 11'd0;
         done_tgl <= 1'b0; drop_g <= 16'd0; ack_sync <= 3'd0;
      end else begin
         ack_sync <= {ack_sync[1:0], ack_tgl};
         if (ack_pulse_g) holding <= 1'b0;     // backend consumed it

         if (!holding) begin
            if (rx_valid && wptr < BUF_BYTES[10:0]) begin
               rx_buf[wptr] <= rx_data;
               wptr <= wptr + 11'd1;
            end
            if (rx_last) begin
               if (rx_good && wptr != 11'd0) begin
                  len_g    <= wptr;
                  done_tgl <= ~done_tgl;
                  holding  <= 1'b1;
               end
               wptr <= 11'd0;                  // restart for the next frame
            end
         end else if (rx_last && rx_good) begin
            drop_g <= drop_g + 16'd1;           // busy: drop this frame
         end
      end
   end

   // ---- ui-domain handoff ------------------------------------------------
   reg [2:0] done_sync;       // gmii done_tgl -> ui
   reg       ack_tgl;         // ui -> gmii
   wire      done_pulse_ui = done_sync[2] ^ done_sync[1];

   always @(posedge ui_clk) begin
      if (ui_rst) begin
         frame_valid <= 1'b0; frame_len <= 11'd0;
         done_sync <= 3'd0; ack_tgl <= 1'b0; drop_count <= 16'd0;
      end else begin
         done_sync  <= {done_sync[1:0], done_tgl};
         drop_count <= drop_g;                 // approximate (rarely changes)
         if (done_pulse_ui) begin
            frame_valid <= 1'b1;
            frame_len   <= len_g;               // stable while held
         end else if (frame_ack && frame_valid) begin
            frame_valid <= 1'b0;
            ack_tgl     <= ~ack_tgl;
         end
      end
   end

endmodule

`default_nettype wire
