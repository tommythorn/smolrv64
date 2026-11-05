`timescale 1ns / 1ps
`default_nettype none

// gmii_phy -- a GMII PHY's pins (the RTL8211EG, strapped GMII) to the shell's GMII, gigabit.
//
// Receive: rxclk through an MMCM whose BUFG feedback deskews its output, as rgmii_rx does
// (UltraScale+ has no BUFIO, and a BUFG's ~2.5 ns insertion alone samples past the eye). The pin
// is in an HD bank, which has no clock management tile beside it, so a BUFG carries it to the
// MMCM, and that BUFG's delay is not compensated: RX_PHASE places the sample in the eye, which the
// board's .xdc constrains with set_input_delay so every build proves the window. RXD and RXDV are captured in the IOB.
// MMCME4 does not reliably relock after its input stops (link renegotiation) without a reset
// pulse: as on the RK board, a ui_clk watchdog pulses RST while LOCKED stays low.
// Transmit runs on the same clock: TXD and TXEN from IOB flops, and GTX_CLK forwarded inverted
// through an ODDRE1, its rising edge in the middle of each bit.
// phy_rst_n holds the PHY in reset for 2^22 ui_clk cycles (12.6 ms) after configuration: its
// reset is the FPGA's on this board, so no state it was left in survives a new bitstream.
module gmii_phy #(
    parameter real RX_PHASE = 0.0          // degrees of the 125 MHz capture clock
) (
    input  wire       ui_clk,              // the DDR4 UI clock, for the supervision
    input  wire       rxclk,
    input  wire       rxdv,
    input  wire [7:0] rxd,
    output wire       gtx_clk,
    output reg        txen = 1'b0,
    output reg  [7:0] txd = 8'd0,
    output wire       phy_rst_n,

    output wire       gmii_rx_clk,
    output wire       gmii_clk_ok,
    output reg        gmii_rx_dv = 1'b0,
    output reg  [7:0] gmii_rxd = 8'd0,
    input  wire       gmii_tx_en,
    input  wire [7:0] gmii_txd
);
    // ---- the PHY's reset, once after configuration ----
    reg [22:0] por = 23'd0;
    always @(posedge ui_clk) if (!por[22]) por <= por + 23'd1;
    assign phy_rst_n = por[22];

    // ---- the capture clock ----
    wire rxclk_g, clkfb, clkfb_buf, clk0, locked;
    BUFG bufg_in (.I(rxclk), .O(rxclk_g));
    reg  mmcm_rst = 1'b0;
    MMCME4_BASE #(
        .CLKIN1_PERIOD    (8.000),         // 125 MHz from the PHY
        .DIVCLK_DIVIDE    (1),
        .CLKFBOUT_MULT_F  (10.000),        // VCO 1250 MHz
        .CLKOUT0_DIVIDE_F (10.000),        // 125 MHz
        .CLKOUT0_PHASE    (RX_PHASE)
    ) rx_mmcm (
        .CLKIN1(rxclk_g), .CLKFBIN(clkfb_buf), .CLKFBOUT(clkfb), .CLKOUT0(clk0),
        .LOCKED(locked), .PWRDWN(1'b0), .RST(mmcm_rst | ~phy_rst_n)
    );
    BUFG bufg_fb  (.I(clkfb), .O(clkfb_buf));
    BUFG bufg_rx  (.I(clk0),  .O(gmii_rx_clk));
    assign gmii_clk_ok = locked;

    // LOCKED low for ~1.5 ms: pulse RST for 16 ui_clks, and again each lap until it locks
    reg [19:0] wd = 20'd0;
    (* ASYNC_REG = "TRUE" *) reg [1:0] lock_s = 2'b00;
    always @(posedge ui_clk) begin
        lock_s <= {lock_s[0], locked};
        if (lock_s[1] || !phy_rst_n) begin wd <= 20'd0;  mmcm_rst <= 1'b0; end
        else begin wd <= wd + 20'd1;  mmcm_rst <= &wd[19:4]; end
    end

    // ---- receive and transmit, in the IOB ----
    (* IOB = "TRUE" *) reg       rxdv_q = 1'b0;
    (* IOB = "TRUE" *) reg [7:0] rxd_q  = 8'd0;
    always @(posedge gmii_rx_clk) begin
        rxdv_q <= rxdv;  rxd_q <= rxd;
        gmii_rx_dv <= rxdv_q;  gmii_rxd <= rxd_q;
        txen <= gmii_tx_en;  txd <= gmii_txd;
    end
    ODDRE1 #(.SRVAL(1'b0)) fwd (.C(gmii_rx_clk), .D1(1'b0), .D2(1'b1), .SR(1'b0), .Q(gtx_clk));
endmodule
