`timescale 1ns / 1ps
`default_nettype none

// GMII <-> RGMII adapter for the RTL8211F-CG PHY.  The whole MAC runs in the
// PHY RX-clock domain: gmii_rx_clk (= BUFG of rgmii_rxc) also drives TX, so no
// MMCM/PLL is needed.  Imported (cleaned) from the board 12_UDP_TEST design.
module gmii_to_rgmii(
    // GMII (to/from the MAC)
    output wire        gmii_rx_clk,
    output wire        gmii_rx_dv,
    output wire [7:0]  gmii_rxd,
    output wire        gmii_tx_clk,
    input  wire        gmii_tx_en,
    input  wire [7:0]  gmii_txd,
    // RGMII (to/from the PHY pins)
    input  wire        rgmii_rxc,
    input  wire        rgmii_rx_ctl,
    input  wire [3:0]  rgmii_rxd,
    output wire        rgmii_txc,
    output wire        rgmii_tx_ctl,
    output wire [3:0]  rgmii_txd
    );

   // TX runs in the recovered RX clock domain.
   assign gmii_tx_clk = gmii_rx_clk;

   rgmii_rx u_rgmii_rx(
      .rgmii_rxc    (rgmii_rxc),
      .rgmii_rx_ctl (rgmii_rx_ctl),
      .rgmii_rxd    (rgmii_rxd),
      .gmii_rx_clk  (gmii_rx_clk),
      .gmii_rx_dv   (gmii_rx_dv),
      .gmii_rxd     (gmii_rxd)
   );

   rgmii_tx u_rgmii_tx(
      .gmii_tx_clk  (gmii_tx_clk),
      .gmii_tx_en   (gmii_tx_en),
      .gmii_txd     (gmii_txd),
      .rgmii_txc    (rgmii_txc),
      .rgmii_tx_ctl (rgmii_tx_ctl),
      .rgmii_txd    (rgmii_txd)
   );

endmodule

`default_nettype wire
