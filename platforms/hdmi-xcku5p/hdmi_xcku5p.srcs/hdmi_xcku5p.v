`timescale 1ns / 1ps
`default_nettype none

// The HDMI-XCKU5P board (1 GiB DDR4, HDMI out): its pins, the SoC (smolrv64_shell,
// platforms/xcku5p), its RTL8211EG's GMII (gmii_phy) and the HDMI transmitter (hdmi_tx).
module hdmi_xcku5p(
    // 100 MHz differential reference (X1), the DDR4 IP's system clock
    input  wire        sys_clk_p,
    input  wire        sys_clk_n,
    input  wire        clk_50m,   // the board's 50 MHz oscillator, for the fan alone
    output reg         fan_pwm = 1'b0,
    input  wire [3:0]  key,
    // USB keyboards and mice through a Raspberry Pi Pico (tools/hid-bridge): its UART TX on
    // PMOD0_0, virtio_input's HID line.
    input  wire        hid_rxd,
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
   wire        aud_valid;                 // virtio_snd's 44.1 kHz frames, in ui_clk
   wire [31:0] aud_frame;
   wire [ 3:0] shell_led;

   assign led     = {4'd0, shell_led};
   assign hdmi_oe = 1'b1;

   // The fan: 25 kHz at 75% (100% is loud). Its driver inverts -- an undriven pin is pulled up, which
   // stops it -- so the pin is LOW for the on part: 1500 of each 2000 cycles of the 50 MHz oscillator.
   // It runs from the board's own oscillator, so it turns from configuration on, whatever DDR4 or the
   // core are doing.
   wire        clk_fan;
   reg  [10:0] fan_cnt = 11'd0;
   BUFG bufg_fan (.I(clk_50m), .O(clk_fan));
   always @(posedge clk_fan) begin
      fan_cnt <= (fan_cnt == 11'd1999) ? 11'd0 : fan_cnt + 11'd1;     // 50 MHz / 2000 = 25 kHz
      fan_pwm <= fan_cnt >= 11'd1500;                                  // high (off) a quarter of it
   end

   // The pixel clock's MMCM comes up, and is timed, at 1000 MHz / 7 = 142.86 MHz: hdmi_tx's
   // MMCM multiplies it by ten, within its VCO's 1600 MHz, and 1080p60 is D = 7. The monitor
   // starts the screen at 1280x720, D = 14.
   // The console runs at 2 Mbps, its CP2105 interface's fastest.
   smolrv64_shell #(.VID_CLK_D(7), .UART_BAUD(2_000_000)) shell (
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
      .aud_valid        (aud_valid),
      .aud_frame        (aud_frame),
      .hid_rxd          (hid_rxd),
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

   // The capture clock reaches the IOB flops 3.5 ns after RXCLK's pin edge (the HD bank's input
   // buffer and BUFGCE_HDIO before the MMCM, which compensates only its own feedback): -130.5
   // degrees (2.9 ns earlier, the nearest MMCM step) centers the sample in the PHY's window.
   gmii_phy #(.RX_PHASE(-130.5)) eth (
      .ui_clk(ui_clk), .rxclk(eth_rxclk), .rxdv(eth_rxdv), .rxd(eth_rxd),
      .gtx_clk(eth_gtx_clk), .txen(eth_txen), .txd(eth_txd), .phy_rst_n(eth_rst_n),
      .gmii_rx_clk(gmii_rx_clk), .gmii_clk_ok(gmii_clk_ok), .gmii_rx_dv(gmii_rx_dv),
      .gmii_rxd(gmii_rxd), .gmii_tx_en(gmii_tx_en), .gmii_txd(gmii_txd));

   hdmi_tx hdmi (
      .pix_clk(vid_clk), .pix_locked(vid_locked), .de(vid_de), .hs(vid_hs), .vs(vid_vs), .rgb(vid_rgb),
      .aud_clk(ui_clk), .aud_valid(aud_valid), .aud_frame(aud_frame),
      .tmds_p(tmds_p), .tmds_n(tmds_n), .tmds_clk_p(tmds_clk_p), .tmds_clk_n(tmds_clk_n));
endmodule
