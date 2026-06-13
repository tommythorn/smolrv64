`timescale 1ns / 1ps
`default_nettype none

// Sim-only loopback: eth_mac_tx GMII output -> eth_mac_rx GMII input, so a
// testbench can confirm a frame round-trips and its FCS validates.  Both MACs
// share one clock here (on hardware both run in the single PHY RX-clock
// domain anyway).
module eth_loop_top(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        send,
    input  wire        corrupt,    // 1 = flip a wire bit into RX (FCS test)
    input  wire [10:0] frame_len,
    output wire [10:0] rd_index,
    input  wire [ 7:0] rd_data,
    output wire        tx_busy,
    output wire        rx_valid,
    output wire [ 7:0] rx_data,
    output wire        rx_last,
    output wire        rx_good
    );

   wire       gmii_en;
   wire [7:0] gmii_d;

   eth_mac_tx u_tx(
      .clk        (clk),
      .rst_n      (rst_n),
      .send       (send),
      .frame_len  (frame_len),
      .rd_index   (rd_index),
      .rd_data    (rd_data),
      .busy       (tx_busy),
      .gmii_tx_en (gmii_en),
      .gmii_txd   (gmii_d)
   );

   // Inject a one-cycle wire error to exercise the RX FCS check.
   wire [7:0] gmii_d_rx = (corrupt && gmii_en) ? (gmii_d ^ 8'h01) : gmii_d;

   eth_mac_rx u_rx(
      .clk        (clk),
      .rst_n      (rst_n),
      .gmii_rx_dv (gmii_en),
      .gmii_rxd   (gmii_d_rx),
      .rx_valid   (rx_valid),
      .rx_data    (rx_data),
      .rx_last    (rx_last),
      .rx_good    (rx_good)
   );

endmodule

`default_nettype wire
