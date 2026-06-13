`timescale 1ns / 1ps
`default_nettype none

// Sim-only: eth_tx_engine -> (GMII) -> eth_mac_rx, to verify the frame buffer,
// the send/done CDC handshake, and the framing roundtrip.
module eth_tx_engine_loop_top(
    input  wire        ui_clk,
    input  wire        ui_rst,
    input  wire        wr_en,
    input  wire [10:0] wr_addr,
    input  wire [ 7:0] wr_data,
    input  wire        send,
    input  wire [10:0] send_len,
    output wire        busy,

    input  wire        gmii_clk,
    input  wire        gmii_rst,
    output wire        rx_valid,
    output wire [ 7:0] rx_data,
    output wire        rx_last,
    output wire        rx_good
    );

   wire       gmii_en;
   wire [7:0] gmii_d;

   eth_tx_engine #(.BUF_BYTES(1536)) u_eng(
      .ui_clk     (ui_clk),
      .ui_rst     (ui_rst),
      .wr_en      (wr_en),
      .wr_addr    (wr_addr),
      .wr_data    (wr_data),
      .send       (send),
      .send_len   (send_len),
      .busy       (busy),
      .gmii_clk   (gmii_clk),
      .gmii_rst   (gmii_rst),
      .gmii_tx_en (gmii_en),
      .gmii_txd   (gmii_d)
   );

   eth_mac_rx u_rx(
      .clk        (gmii_clk),
      .rst_n      (~gmii_rst),
      .gmii_rx_dv (gmii_en),
      .gmii_rxd   (gmii_d),
      .rx_valid   (rx_valid),
      .rx_data    (rx_data),
      .rx_last    (rx_last),
      .rx_good    (rx_good)
   );

endmodule

`default_nettype wire
