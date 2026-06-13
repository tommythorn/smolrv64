`timescale 1ns / 1ps
`default_nettype none

// RGMII transmit: SDR 8-bit GMII -> DDR 4-bit RGMII to the PHY.
// rgmii_txc is forwarded from gmii_tx_clk; data/ctl go out through ODDRE1s
// (low nibble on the rising edge, high nibble on the falling edge).  Relies on
// the PHY adding the RGMII TX delay internally (RTL8211F TXDLY strapping),
// matching the 12_UDP_TEST design this is imported from.
module rgmii_tx(
    input  wire        gmii_tx_clk,
    input  wire        gmii_tx_en,
    input  wire [7:0]  gmii_txd,
    output wire        rgmii_txc,
    output wire        rgmii_tx_ctl,
    output wire [3:0]  rgmii_txd
    );

   assign rgmii_txc = gmii_tx_clk;

   ODDRE1 #(
      .IS_C_INVERTED  (1'b0),
      .IS_D1_INVERTED (1'b0),
      .IS_D2_INVERTED (1'b0),
      .SIM_DEVICE     ("ULTRASCALE_PLUS"),
      .SRVAL          (1'b0)
   ) ODDRE1_tx_ctl (
      .Q  (rgmii_tx_ctl),
      .C  (gmii_tx_clk),
      .D1 (gmii_tx_en),
      .D2 (gmii_tx_en),
      .SR (1'b0)
   );

   genvar i;
   generate for (i = 0; i < 4; i = i + 1) begin : txdata_bus
      ODDRE1 #(
         .IS_C_INVERTED  (1'b0),
         .IS_D1_INVERTED (1'b0),
         .IS_D2_INVERTED (1'b0),
         .SIM_DEVICE     ("ULTRASCALE_PLUS"),
         .SRVAL          (1'b0)
      ) ODDRE1_inst (
         .Q  (rgmii_txd[i]),
         .C  (gmii_tx_clk),
         .D1 (gmii_txd[i]),
         .D2 (gmii_txd[4+i]),
         .SR (1'b0)
      );
   end endgenerate

endmodule

`default_nettype wire
