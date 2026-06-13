`timescale 1ns / 1ps
`default_nettype none

// TX engine: bridges the virtio_net backend (ui_clk domain) to eth_mac_tx
// (gmii_clk = PHY RX-clock domain).  The backend writes the outgoing L2 frame
// byte-by-byte into a frame buffer, then pulses `send` with the length; the
// engine hands the frame to eth_mac_tx in the gmii domain and reports `busy`
// back in ui_clk until transmission completes.
//
// The frame buffer is an async-read RAM (so eth_mac_tx's combinational rd_data
// contract holds) written in ui_clk and read in gmii_clk; it is the CDC for
// the payload.  send/done cross via toggle synchronizers.  send_len is sampled
// only while a transfer is in flight, during which it is held stable.
module eth_tx_engine #(
    parameter integer BUF_BYTES = 1536
)(
    // ui_clk (virtio backend) domain
    input  wire        ui_clk,
    input  wire        ui_rst,
    input  wire        wr_en,       // write one frame byte at wr_addr
    input  wire [10:0] wr_addr,
    input  wire [ 7:0] wr_data,
    input  wire        send,        // 1-cycle pulse: transmit send_len bytes
    input  wire [10:0] send_len,
    output reg         busy,        // high from send accept until TX done

    // gmii_clk (PHY RX-clock) domain
    input  wire        gmii_clk,
    input  wire        gmii_rst,
    output wire        gmii_tx_en,
    output wire [ 7:0] gmii_txd
    );

   // ---- async-read frame buffer (ui write / gmii read) -------------------
   (* ram_style = "distributed" *)
   reg [7:0] frame_buf [0:BUF_BYTES-1];
   wire [10:0] rd_index;
   /* verilator lint_off WIDTHEXPAND */
   wire [7:0]  rd_data = frame_buf[rd_index];
   always @(posedge ui_clk)
      if (wr_en) frame_buf[wr_addr] <= wr_data;
   /* verilator lint_on WIDTHEXPAND */

   // ---- length latch + send/done CDC ------------------------------------
   reg [10:0] len_latch;            // stable while busy (read in gmii domain)
   reg        send_tgl;             // toggles in ui_clk on each accepted send
   reg [2:0]  send_sync;            // -> gmii_clk
   reg        done_tgl;             // toggles in gmii_clk when a TX finishes
   reg [2:0]  done_sync;            // -> ui_clk

   wire send_pulse_gmii = send_sync[2] ^ send_sync[1];
   wire done_pulse_ui   = done_sync[2] ^ done_sync[1];

   always @(posedge ui_clk) begin
      if (ui_rst) begin
         busy <= 1'b0; send_tgl <= 1'b0; done_sync <= 3'd0; len_latch <= 11'd0;
      end else begin
         done_sync <= {done_sync[1:0], done_tgl};
         if (send && !busy) begin
            busy      <= 1'b1;
            len_latch <= send_len;
            send_tgl  <= ~send_tgl;
         end else if (done_pulse_ui) begin
            busy <= 1'b0;
         end
      end
   end

   // ---- gmii-domain transmit --------------------------------------------
   reg        tx_send;
   wire       tx_busy;
   reg        tx_busy_d;

   always @(posedge gmii_clk) begin
      if (gmii_rst) begin
         send_sync <= 3'd0; tx_send <= 1'b0; done_tgl <= 1'b0; tx_busy_d <= 1'b0;
      end else begin
         send_sync <= {send_sync[1:0], send_tgl};
         tx_send   <= send_pulse_gmii;       // 1-cycle kick to eth_mac_tx
         tx_busy_d <= tx_busy;
         if (tx_busy_d && !tx_busy)           // falling edge = TX complete
            done_tgl <= ~done_tgl;
      end
   end

   eth_mac_tx u_tx(
      .clk        (gmii_clk),
      .rst_n      (~gmii_rst),
      .send       (tx_send),
      .frame_len  (len_latch),    // stable across the handshake
      .rd_index   (rd_index),
      .rd_data    (rd_data),
      .busy       (tx_busy),
      .gmii_tx_en (gmii_tx_en),
      .gmii_txd   (gmii_txd)
   );

endmodule

`default_nettype wire
