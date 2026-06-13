`timescale 1ns / 1ps
`default_nettype none

// RGMII receive: DDR 4-bit RGMII from the PHY -> SDR 8-bit GMII.
// rgmii_rxc is BUFG'd to gmii_rx_clk (the MAC's RX clock domain) and BUFIO'd
// to clock the IDDRE1s.  Relies on the PHY adding the RGMII RX delay
// internally (RTL8211F RXDLY strapping), matching the 12_UDP_TEST design this
// is imported from.  rx_dv = both half-samples of rgmii_rx_ctl agreeing.
module rgmii_rx(
    input  wire        rgmii_rxc,
    input  wire        rgmii_rx_ctl,
    input  wire [3:0]  rgmii_rxd,
    output wire        gmii_rx_clk,
    output wire        gmii_rx_dv,
    output wire [7:0]  gmii_rxd
    );

   wire        rgmii_rxc_bufg;
   wire        rgmii_rxc_bufio;
   wire [1:0]  gmii_rxdv_t;

   assign gmii_rx_clk = rgmii_rxc_bufg;
   assign gmii_rx_dv  = gmii_rxdv_t[0] & gmii_rxdv_t[1];

   BUFG  BUFG_inst  (.I(rgmii_rxc), .O(rgmii_rxc_bufg));
   BUFIO BUFIO_inst (.I(rgmii_rxc), .O(rgmii_rxc_bufio));

   IDDRE1 #(
      .DDR_CLK_EDGE   ("SAME_EDGE_PIPELINED"),
      .IS_CB_INVERTED (1'b0),
      .IS_C_INVERTED  (1'b0)
   ) IDDRE1_ctl (
      .Q1 (gmii_rxdv_t[0]),
      .Q2 (gmii_rxdv_t[1]),
      .C  (rgmii_rxc_bufio),
      .CB (~rgmii_rxc_bufio),
      .D  (rgmii_rx_ctl),
      .R  (1'b0)
   );

   genvar i;
   generate for (i = 0; i < 4; i = i + 1) begin : rxdata_bus
      IDDRE1 #(
         .DDR_CLK_EDGE   ("SAME_EDGE_PIPELINED"),
         .IS_CB_INVERTED (1'b0),
         .IS_C_INVERTED  (1'b0)
      ) IDDRE1_inst (
         .Q1 (gmii_rxd[i]),
         .Q2 (gmii_rxd[4+i]),
         .C  (rgmii_rxc_bufio),
         .CB (~rgmii_rxc_bufio),
         .D  (rgmii_rxd[i]),
         .R  (1'b0)
      );
   end endgenerate

endmodule

`default_nettype wire
