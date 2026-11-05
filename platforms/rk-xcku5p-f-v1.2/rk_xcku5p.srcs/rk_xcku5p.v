`timescale 1ns / 1ps
`default_nettype none

// The RK-XCKU5P-F v1.2 board: its pins, the SoC (smolrv64_shell, platforms/xcku5p), and the
// adapter from its RTL8211F's RGMII to the shell's GMII.
module rk_xcku5p(
    // Differential 200 MHz system clock (fed directly to DDR4 IP)
    input  wire       sys_clk_p,
    input  wire       sys_clk_n,
    input  wire [3:0] key,
    input  wire       rxd,
    output wire [3:0] led,
    output wire       txd,
    output wire       sd_clk,    // SPI mode: SCK
    output wire       sd_cmd,    // SPI mode: MOSI (DI)
    input  wire       sd_cd,
    inout  wire [3:0] sd_d,

    // RGMII to RTL8211F-CG Ethernet PHY (pins per the board 12_UDP_TEST
    // design), through gmii_to_rgmii below to the shell's MAC.
    input  wire       eth_rxc,
    input  wire [3:0] eth_rxd,
    input  wire       eth_rx_ctl,
    output wire       eth_txc,
    output wire [3:0] eth_txd,
    output wire       eth_tx_ctl,

    // TinyVGA adapter on the 40-pin header, IO14..IO17: RGB222 and the syncs (vga_scanout).
    output wire       vga_hs,
    output wire       vga_vs,
    output wire [1:0] vga_r,
    output wire [1:0] vga_g,
    output wire [1:0] vga_b,

    // USB keyboards and mice through a Raspberry Pi Pico (tools/hid-bridge): its UART TX on the
    // 40-pin header, IO1_P. virtio_input's HID line.
    input  wire       hid_rxd,

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
   wire [15:0] vid_rgb;
   // The TinyVGA adapter's RGB222: the top two bits of each RGB565 channel.
   assign vga_r = vid_rgb[15:14];
   assign vga_g = vid_rgb[10: 9];
   assign vga_b = vid_rgb[ 4: 3];
   wire        gmii_rx_clk, gmii_clk_ok, gmii_rx_dv, gmii_tx_en;
   wire [ 7:0] gmii_rxd, gmii_txd;

   smolrv64_shell shell (
      .sys_clk_p        (sys_clk_p),
      .sys_clk_n        (sys_clk_n),
      .key              (key),
      .rxd              (rxd),
      .led              (led),
      .txd              (txd),
      .sd_clk           (sd_clk),
      .sd_cmd           (sd_cmd),
      .sd_cd            (sd_cd),
      .sd_d             (sd_d),
      .vid_clk          (),
      .vid_locked       (),
      .vid_de           (),
      .vid_hs           (vga_hs),
      .vid_vs           (vga_vs),
      .vid_rgb          (vid_rgb),
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

`ifndef NO_VIRTIO_NET
   // ===== RGMII <-> GMII: the RTL8211F-CG =====
   // RX-capture MMCM supervision: MMCME4 does not reliably relock after its
   // input clock (the PHY's rxc) is interrupted -- link renegotiation -- without
   // a reset pulse.  If LOCKED stays low for ~1.5ms, pulse RST for 16 ui_clks
   // and retry periodically until it locks.
   wire eth_mmcm_locked;
   reg  [19:0] eth_mmcm_wd = 20'd0;
   reg         eth_mmcm_rst = 1'b0;
   (* async_reg = "true" *) reg [1:0] eth_lock_sync = 2'b00;
   always @(posedge ui_clk) begin
      eth_lock_sync <= {eth_lock_sync[0], eth_mmcm_locked};
      if (eth_lock_sync[1]) begin
         eth_mmcm_wd  <= 20'd0;
         eth_mmcm_rst <= 1'b0;
      end else begin
         eth_mmcm_wd  <= eth_mmcm_wd + 20'd1;
         eth_mmcm_rst <= &eth_mmcm_wd[19:4];   // top 16 counts of each lap
      end
   end

   assign gmii_clk_ok = eth_mmcm_locked;

   gmii_to_rgmii gmii_to_rgmii_inst(
      .mmcm_rst     (eth_mmcm_rst),
      .mmcm_locked  (eth_mmcm_locked),
      .gmii_rx_clk  (gmii_rx_clk),
      .gmii_rx_dv   (gmii_rx_dv),
      .gmii_rxd     (gmii_rxd),
      .gmii_tx_clk  (),
      .gmii_tx_en   (gmii_tx_en),
      .gmii_txd     (gmii_txd),
      .rgmii_rxc    (eth_rxc),
      .rgmii_rx_ctl (eth_rx_ctl),
      .rgmii_rxd    (eth_rxd),
      .rgmii_txc    (eth_txc),
      .rgmii_tx_ctl (eth_tx_ctl),
      .rgmii_txd    (eth_txd)
   );
`else
   assign gmii_rx_clk = 1'b0;  assign gmii_clk_ok = 1'b0;  assign gmii_rx_dv = 1'b0;  assign gmii_rxd = 8'd0;
   assign eth_txc = 1'b0;  assign eth_txd = 4'd0;  assign eth_tx_ctl = 1'b0;
`endif
endmodule
