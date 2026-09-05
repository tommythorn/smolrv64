`timescale 1ns / 1ps
`default_nettype none
// Sim-only: eth_mac_tx -> (GMII) -> eth_mac_rx -> eth_rx_engine, so a testbench
// can transmit frames and read them back out the eth_rx_engine ui side, across
// independent gmii_clk / ui_clk.  `corrupt` flips a data bit on the GMII bus while
// high (an FCS-bad frame); `tx_busy` lets the bench send frames back to back.
module eth_rx_engine_loop_top #(
    parameter integer SLOTS      = 8,
    parameter integer SLOT_BYTES = 2048
)(
    input  wire        gmii_clk,
    input  wire        gmii_rst,
    input  wire        send,
    input  wire [10:0] tx_frame_len,
    output wire [10:0] tx_rd_index,
    input  wire [ 7:0] tx_rd_data,
    output wire        tx_busy,
    input  wire        corrupt,
    input  wire        ui_clk,
    input  wire        ui_rst,
    output wire        rx_frame_valid,
    output wire [10:0] rx_frame_len,
    input  wire [10:0] rx_rd_addr,
    output wire [ 7:0] rx_rd_data,
    input  wire        rx_frame_ack,
    output wire [15:0] rx_drop_count,
    output wire [15:0] rx_bad_count,
    output wire [15:0] rx_oflow_count
    );
   wire       gmii_en;
   wire [7:0] gmii_d;
   wire       rxv, rxl, rxg;
   wire [7:0] rxd;
   eth_mac_tx u_tx(
      .clk(gmii_clk), .rst_n(~gmii_rst), .send(send), .frame_len(tx_frame_len),
      .rd_index(tx_rd_index), .rd_data(tx_rd_data), .busy(tx_busy),
      .gmii_tx_en(gmii_en), .gmii_txd(gmii_d)
   );
   eth_mac_rx u_rx(
      .clk(gmii_clk), .rst_n(~gmii_rst), .gmii_rx_dv(gmii_en),
      .gmii_rxd(gmii_d ^ {7'd0, corrupt & gmii_en}),
      .rx_valid(rxv), .rx_data(rxd), .rx_last(rxl), .rx_good(rxg)
   );
   eth_rx_engine #(.SLOTS(SLOTS), .SLOT_BYTES(SLOT_BYTES)) u_eng(
      .gmii_clk(gmii_clk), .gmii_rst(gmii_rst),
      .rx_valid(rxv), .rx_data(rxd), .rx_last(rxl), .rx_good(rxg),
      .ui_clk(ui_clk), .ui_rst(ui_rst),
      .frame_valid(rx_frame_valid), .frame_len(rx_frame_len),
      .rd_addr(rx_rd_addr), .rd_data(rx_rd_data),
      .frame_ack(rx_frame_ack), .drop_count(rx_drop_count),
      .bad_count(rx_bad_count), .oflow_count(rx_oflow_count)
   );
endmodule
`default_nettype wire
