`timescale 1ns / 1ps
`default_nettype none

// RGMII receive: DDR 4-bit RGMII from the PHY -> SDR 8-bit GMII.
//
// The PHY (RTL8211F RXDLY strap) centers rxc in the data eye AT THE PIN, but
// UltraScale+ has no BUFIO: the original BUFIO silently retargeted to a BUFGCE
// whose ~2.5ns insertion pushed the internal sampling point ~1.8ns past the
// eye center -- ~0.2ns of setup margin, so RX worked or failed per build/pin
// (measured on HW: 100% FCS-bad frames, bit errors clustered on rxd[0]/rxd[2],
// while TX on the same clock was fine).  An MMCM with BUFG feedback deskews
// the capture clock back to the pin edge, and CLKOUT0_PHASE then places the
// sample at the data-eye center as seen at the IDDR D pins (+0.8ns, matching
// the ~0.77ns pin->D route).  The MAC and TX run on the same deskewed clock.
// rk_xcku5p.xdc constrains the eye (set_input_delay) so every build proves
// the window instead of gambling on it.
//
// mmcm_rst/mmcm_locked: MMCME4 does not reliably relock after its input clock
// is interrupted (PHY link renegotiation) without a reset pulse; the platform
// top supervises LOCKED and pulses mmcm_rst.  rx_dv = both half-samples of
// rgmii_rx_ctl agreeing.
module rgmii_rx(
    input  wire        rgmii_rxc,
    input  wire        rgmii_rx_ctl,
    input  wire [3:0]  rgmii_rxd,
    input  wire        mmcm_rst,
    output wire        mmcm_locked,
    output wire        gmii_rx_clk,
    output wire        gmii_rx_dv,
    output wire [7:0]  gmii_rxd
    );

   wire        clkfb, clkfb_buf, clk0;
   wire [1:0]  gmii_rxdv_t;

   assign gmii_rx_dv = gmii_rxdv_t[0] & gmii_rxdv_t[1];

   MMCME4_BASE #(
      .CLKIN1_PERIOD    (8.000),     // 125 MHz from the PHY
      .DIVCLK_DIVIDE    (1),
      .CLKFBOUT_MULT_F  (10.000),    // VCO 1250 MHz
      .CLKOUT0_DIVIDE_F (10.000),    // 125 MHz out
      .CLKOUT0_PHASE    (36.000)     // +0.8ns: eye center at the IDDR D pins
   ) u_mmcm (
      .CLKIN1   (rgmii_rxc),
      .CLKFBIN  (clkfb_buf),
      .CLKFBOUT (clkfb),
      .CLKFBOUTB(),
      .CLKOUT0  (clk0),
      .CLKOUT0B (),
      .CLKOUT1  (), .CLKOUT2 (), .CLKOUT3 (),
      .CLKOUT4  (), .CLKOUT5 (), .CLKOUT6 (),
      .LOCKED   (mmcm_locked),
      .PWRDWN   (1'b0),
      .RST      (mmcm_rst)
   );
   BUFG BUFG_fb  (.I(clkfb), .O(clkfb_buf));
   BUFG BUFG_out (.I(clk0),  .O(gmii_rx_clk));

   IDDRE1 #(
      .DDR_CLK_EDGE   ("SAME_EDGE_PIPELINED"),
      .IS_CB_INVERTED (1'b0),
      .IS_C_INVERTED  (1'b0)
   ) IDDRE1_ctl (
      .Q1 (gmii_rxdv_t[0]),
      .Q2 (gmii_rxdv_t[1]),
      .C  (gmii_rx_clk),
      .CB (~gmii_rx_clk),
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
         .C  (gmii_rx_clk),
         .CB (~gmii_rx_clk),
         .D  (rgmii_rxd[i]),
         .R  (1'b0)
      );
   end endgenerate

endmodule

`default_nettype wire
