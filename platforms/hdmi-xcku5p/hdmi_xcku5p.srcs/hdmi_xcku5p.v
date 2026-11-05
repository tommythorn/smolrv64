`timescale 1ns / 1ps
`default_nettype none

// The HDMI-XCKU5P board (1 GiB DDR4, HDMI out): its pins, the SoC (smolrv64_shell,
// platforms/xcku5p), its RTL8211EG's GMII (gmii_phy) and the HDMI transmitter (hdmi_tx).
module hdmi_xcku5p(
    // 100 MHz differential reference (X1), the DDR4 IP's system clock
    input  wire        sys_clk_p,
    input  wire        sys_clk_n,
    input  wire [3:0]  key,
    input  wire        rxd,       // the CP2105's interface 0
    output wire [7:0]  led,
    output wire        txd,
    output wire        sd_clk,    // SPI mode: SCK
    output wire        sd_cmd,    // SPI mode: MOSI (DI)
    inout  wire [3:0]  sd_d,

    // RTL8211EG, strapped GMII; its MDC/MDIO do not reach the FPGA, its reset does
    input  wire        eth_rxclk,
    input  wire        eth_rxdv,
    input  wire [7:0]  eth_rxd,
    output wire        eth_gtx_clk,
    output wire        eth_txen,
    output wire [7:0]  eth_txd,
    output wire        eth_rst_n,

    // HDMI: TMDS in bank 65 beside the DDR4 PHY, and the connector's 5 V and HPD buffer enable
    output wire [2:0]  tmds_p,
    output wire [2:0]  tmds_n,
    output wire        tmds_clk_p,
    output wire        tmds_clk_n,
    output wire        hdmi_oe,

    // DDR4 physical ports
    output wire        c0_ddr4_act_n,
    output wire [16:0] c0_ddr4_adr,
    output wire [1:0]  c0_ddr4_ba,
    output wire [0:0]  c0_ddr4_bg,
    output wire [0:0]  c0_ddr4_cke,
    output wire [0:0]  c0_ddr4_odt,
    output wire [0:0]  c0_ddr4_cs_n,
    output wire [0:0]  c0_ddr4_ck_t,
    output wire [0:0]  c0_ddr4_ck_c,
    output wire        c0_ddr4_reset_n,
    inout  wire [3:0]  c0_ddr4_dm_dbi_n,
    inout  wire [31:0] c0_ddr4_dq,
    inout  wire [3:0]  c0_ddr4_dqs_t,
    inout  wire [3:0]  c0_ddr4_dqs_c
    );

   wire        ui_clk;
   wire        gmii_rx_clk, gmii_clk_ok, gmii_rx_dv, gmii_tx_en;
   wire [ 7:0] gmii_rxd, gmii_txd;
   wire        vid_clk, vid_locked, vid_de, vid_hs, vid_vs;
   wire [15:0] vid_rgb;
   wire [ 3:0] shell_led;

   assign led     = {4'd0, shell_led};
   assign hdmi_oe = 1'b1;

   // The pixel clock's MMCM comes up, and is timed, at 1000 MHz / 7 = 142.86 MHz: hdmi_tx's
   // MMCM multiplies it by ten, within its VCO's 1600 MHz, and the monitor's 1080p60 is D = 7.
   smolrv64_shell #(.VID_CLK_D(7)) shell (
      .sys_clk_p        (sys_clk_p),
      .sys_clk_n        (sys_clk_n),
      .key              (key),
      .rxd              (rxd),
      .led              (shell_led),
      .txd              (txd),
      .sd_clk           (sd_clk),
      .sd_cmd           (sd_cmd),
      .sd_cd            (1'b0),       // no card detect on this board
      .sd_d             (sd_d),
      .vid_clk          (vid_clk),
      .vid_locked       (vid_locked),
      .vid_de           (vid_de),
      .vid_hs           (vid_hs),
      .vid_vs           (vid_vs),
      .vid_rgb          (vid_rgb),
      .hid_rxd          (1'b1),       // idle: the HID line is not wired yet
      .c0_ddr4_act_n    (c0_ddr4_act_n),
      .c0_ddr4_adr      (c0_ddr4_adr),
      .c0_ddr4_ba       (c0_ddr4_ba),
      .c0_ddr4_bg       (c0_ddr4_bg),
      .c0_ddr4_cke      (c0_ddr4_cke),
      .c0_ddr4_odt      (c0_ddr4_odt),
      .c0_ddr4_cs_n     (c0_ddr4_cs_n),
      .c0_ddr4_ck_t     (c0_ddr4_ck_t),
      .c0_ddr4_ck_c     (c0_ddr4_ck_c),
      .c0_ddr4_reset_n  (c0_ddr4_reset_n),
      .c0_ddr4_dm_dbi_n (c0_ddr4_dm_dbi_n),
      .c0_ddr4_dq       (c0_ddr4_dq),
      .c0_ddr4_dqs_t    (c0_ddr4_dqs_t),
      .c0_ddr4_dqs_c    (c0_ddr4_dqs_c),
      .gmii_rx_clk      (gmii_rx_clk),
      .gmii_clk_ok      (gmii_clk_ok),
      .gmii_rx_dv       (gmii_rx_dv),
      .gmii_rxd         (gmii_rxd),
      .gmii_tx_en       (gmii_tx_en),
      .gmii_txd         (gmii_txd),
      .ui_clk           (ui_clk)
   );

   gmii_phy eth (
      .ui_clk(ui_clk), .rxclk(eth_rxclk), .rxdv(eth_rxdv), .rxd(eth_rxd),
      .gtx_clk(eth_gtx_clk), .txen(eth_txen), .txd(eth_txd), .phy_rst_n(eth_rst_n),
      .gmii_rx_clk(gmii_rx_clk), .gmii_clk_ok(gmii_clk_ok), .gmii_rx_dv(gmii_rx_dv),
      .gmii_rxd(gmii_rxd), .gmii_tx_en(gmii_tx_en), .gmii_txd(gmii_txd));

   hdmi_tx hdmi (
      .pix_clk(vid_clk), .pix_locked(vid_locked), .de(vid_de), .hs(vid_hs), .vs(vid_vs), .rgb(vid_rgb),
      .tmds_p(tmds_p), .tmds_n(tmds_n), .tmds_clk_p(tmds_clk_p), .tmds_clk_n(tmds_clk_n));
endmodule
