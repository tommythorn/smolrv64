`timescale 1ns / 1ps
`default_nettype none

// Placeholder for the RGMII 1 GbE MAC that will drive the board's
// RTL8211F-CG Ethernet PHY.  It currently holds TX idle and ignores RX; its
// only jobs today are to (a) give the eth_* top-level pins a stable
// instantiation point the real MAC can grow into, and (b) keep the RX input
// pins from being trimmed so their pin constraints resolve.
//
// Remaining work to make this a real NIC (see VIRTIO_PLAN.md "Remaining Work:
// the real Ethernet data path"): drive eth_txc at 125 MHz, GMII<->RGMII DDR
// I/O, FCS/IFG, eth_rxc capture with IDELAY alignment, MDIO PHY bring-up, and
// wiring the virtio_net TX/RX queues to the MAC FIFOs.
//
// Pin assignments are copied from the board's 12_UDP_TEST design (RGMII,
// LVCMOS18).  That design relies on the PHY's power-on strapping defaults, so
// there is no MDIO/MDC or PHY-reset pin here yet.
module rgmii_mac_stub(
    input  wire       sample_clk,   // ui_clk; placeholder sampler only
    input  wire       reset,        // active high

    // RGMII to the RTL8211F-CG PHY
    input  wire       eth_rxc,      // RX reference clock from PHY (125 MHz)
    input  wire [3:0] eth_rxd,      // RX data (DDR, 4-bit)
    input  wire       eth_rx_ctl,   // RX control (DDR: dv / err)
    output wire       eth_txc,      // TX clock to PHY (MAC-driven)
    output wire [3:0] eth_txd,      // TX data (DDR, 4-bit)
    output wire       eth_tx_ctl    // TX control (DDR: en / err)
);

   // TX held idle until the real MAC drives it.
   assign eth_txc    = 1'b0;
   assign eth_txd    = 4'd0;
   assign eth_tx_ctl = 1'b0;

   // Keep the RX input pins alive (so PACKAGE_PIN/IOSTANDARD constraints
   // resolve) until the RX datapath exists.  dont_touch stops synthesis from
   // trimming this sampler.  eth_rxc is sampled as data here (not as a clock);
   // the real MAC will instead declare it a clock with input-delay timing.
   /* verilator lint_off UNUSEDSIGNAL */
   (* dont_touch = "true" *) reg [5:0] eth_rx_keep;
   /* verilator lint_on UNUSEDSIGNAL */
   always @(posedge sample_clk)
      eth_rx_keep <= reset ? 6'd0 : {eth_rxc, eth_rx_ctl, eth_rxd};

endmodule

`default_nettype wire
