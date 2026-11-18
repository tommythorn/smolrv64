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
// the payload.  The backend writes it only while busy is low, and send_len is
// held in len_latch until the frame is done.
//
// send and done cross as toggles: a frame is pending while send_tgl and done_tgl
// differ, and busy is that difference as ui_clk sees it.  Neither toggle is ever
// reset, by either domain: the gmii domain is held in reset whenever its clock is
// not good (the PHY's receive clock, which stops while the link renegotiates), and
// a toggle reset on one side of the crossing loses or invents a handshake.  The
// reset aborts only the frame on the wire (eth_mac_tx and `inflight`); the frame
// stays pending and is sent again, whole, when the clock is back.
module eth_tx_engine #(
    parameter integer BUF_BYTES = 1536
)(
    // ui_clk (virtio backend) domain
    input  wire        ui_clk,
    input  wire        wr_en,       // write one frame byte at wr_addr (only while !busy)
    input  wire [10:0] wr_addr,
    input  wire [ 7:0] wr_data,
    input  wire        send,        // 1-cycle pulse: transmit send_len bytes
    input  wire [10:0] send_len,
    output wire        busy,        // high from send accept until TX done

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

   // ---- length latch + send/done toggles ---------------------------------
   reg [10:0] len_latch = 11'd0;    // stable while busy (read in gmii domain)
   reg        send_tgl  = 1'b0;     // ui_clk: flips on each accepted send
   (* async_reg = "true" *) reg [2:0] send_sync = 3'd0;   // -> gmii_clk
   reg        done_tgl  = 1'b0;     // gmii_clk: catches up with send_tgl when a frame is out
   (* async_reg = "true" *) reg [2:0] done_sync = 3'd0;   // -> ui_clk

   assign busy = send_tgl ^ done_sync[2];

   always @(posedge ui_clk) begin
      done_sync <= {done_sync[1:0], done_tgl};
      if (send && !busy) begin
         len_latch <= send_len;
         send_tgl  <= ~send_tgl;
      end
   end

   always @(posedge ui_clk)
      if (wr_en && busy)
         $fatal(1, "eth_tx_engine: frame buffer written while a frame is pending");

   // ---- gmii-domain transmit --------------------------------------------
   reg        tx_send   = 1'b0;
   wire       tx_busy;
   reg        tx_busy_d = 1'b0;
   reg        inflight  = 1'b0;     // eth_mac_tx has the pending frame
   wire       pending   = send_sync[2] ^ done_tgl;

   always @(posedge gmii_clk) begin
      send_sync <= {send_sync[1:0], send_tgl};
      tx_send   <= 1'b0;
      if (gmii_rst) begin
         inflight <= 1'b0;  tx_busy_d <= 1'b0;
      end else begin
         tx_busy_d <= tx_busy;
         if (!inflight && pending) begin
            tx_send  <= 1'b1;               // 1-cycle kick to eth_mac_tx
            inflight <= 1'b1;
         end else if (inflight && tx_busy_d && !tx_busy) begin   // falling edge = TX complete
            done_tgl <= ~done_tgl;
            inflight <= 1'b0;
         end
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
