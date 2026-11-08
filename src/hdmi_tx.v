`timescale 1ns / 1ps
`default_nettype none

// hdmi_tx -- vga_scanout's pixel stream and virtio_snd's audio onto four TMDS pairs, in HDMI mode:
// the symbols (video, control, data islands carrying the audio and the InfoFrames) are
// hdmi_symbols', the serializers here.
//
// The serializers are component-mode OSERDESE3, 8:1: on HDMI-XCKU5P the pairs share bank 65 with
// the DDR4 PHY, which owns every BITSLICE_CONTROL and RIU in their bytes, so a native-mode
// serializer cannot sit beside it. Their bit clock is 5x the pixel clock (DDR: 10 bits a pixel),
// from an MMCM fed by the pixel clock at VCO = 10x, so it follows whatever pixel clock the
// monitor's mode gives -- between 80 and 160 MHz, the MMCM's VCO range (1080p60 is 148.5).
// CLK and CLKDIV come from that one MMCM output through matched buffers (the CLK-CLKDIV skew
// limit). At 1080p60 the bit clock, 742.5 MHz, exceeds OSERDESE3's -2 rating (1.600 ns min
// period, 0.720 ns per half): report_pulse_width flags those three checks, and it runs cleanly
// on the board.
//
// pix_locked is the pixel clock's MMCM lock: this MMCM is held in reset while that clock is not
// good, so it relocks after every mode change.
module hdmi_tx (
    input  wire        pix_clk,
    input  wire        pix_locked,
    input  wire        de,
    input  wire        hs,
    input  wire        vs,
    input  wire [15:0] rgb,          // RGB565
    input  wire        aud_clk,      // 44.1 kHz stereo frames, {right, left} 16-bit (virtio_snd)
    input  wire        aud_valid,
    input  wire [31:0] aud_frame,
    output wire [2:0]  tmds_p,       // [2] red, [1] green, [0] blue
    output wire [2:0]  tmds_n,
    output wire        tmds_clk_p,
    output wire        tmds_clk_n
);
    // ---- the serializers' clocks: 5x and 5/4x the pixel clock ----
    wire sfb, sfb_buf, bclk_u, bclk, wclk, slocked;
    MMCME4_BASE #(
        .CLKIN1_PERIOD   (6.75),      // 1080p60 as the monitor sets it: 1000 MHz / 6.75
        .CLKFBOUT_MULT_F (10.0),      // VCO 10x the pixel clock
        .CLKOUT0_DIVIDE_F(2.0)        // the bit clock, 5x (DDR: 10 bits a pixel)
    ) smmcm (
        .CLKIN1(pix_clk), .CLKFBIN(sfb_buf), .CLKFBOUT(sfb),
        .CLKOUT0(bclk_u), .LOCKED(slocked), .PWRDWN(1'b0), .RST(~pix_locked)
    );
    BUFG                            bufg_sfb  (.I(sfb),    .O(sfb_buf));
    BUFGCE                          bufg_bit  (.I(bclk_u), .CE(1'b1), .O(bclk));
    BUFGCE_DIV #(.BUFGCE_DIVIDE(4)) bufg_word (.I(bclk_u), .CE(1'b1), .CLR(1'b0), .O(wclk));

    wire ready = pix_locked & slocked;
    wire prst, wrst;
    hdmi_rst_bridge rb_pix  (.clk(pix_clk), .ok(ready), .rst(prst));
    hdmi_rst_bridge rb_word (.clk(wclk),    .ok(ready), .rst(wrst));

    // ---- TMDS symbols, in the pixel clock ----
    wire [9:0] s_r, s_g, s_b;
    hdmi_symbols syms (
        .pix_clk(pix_clk), .rst(prst), .de(de), .hs(hs), .vs(vs), .rgb(rgb),
        .aud_clk(aud_clk), .aud_valid(aud_valid), .aud_frame(aud_frame),
        .s_r(s_r), .s_g(s_g), .s_b(s_b));

    // the clock lane: 5 bits high, 5 low a pixel
    wire [7:0] d_ck, d_r, d_g, d_b;
    hdmi_tmds_gearbox gb (
        .pclk(pix_clk), .prst(prst), .din({10'b0000011111, s_r, s_g, s_b}),
        .wclk(wclk), .wrst(wrst), .dout({d_ck, d_r, d_g, d_b})
    );

    // ---- four serializers ----
    wire [31:0] d = {d_ck, d_r, d_g, d_b};
    wire [3:0]  tp, tn;
    assign {tmds_clk_p, tmds_p} = tp;
    assign {tmds_clk_n, tmds_n} = tn;
    genvar ln;
    for (ln = 0; ln < 4; ln = ln + 1) begin : lane
        wire oq;
        OSERDESE3 #(.DATA_WIDTH(8), .SIM_DEVICE("ULTRASCALE_PLUS")) ser (
            .CLK(bclk), .CLKDIV(wclk), .D(d[ln*8 +: 8]), .RST(wrst), .T(1'b0), .OQ(oq), .T_OUT());
        OBUFDS obuf (.I(oq), .O(tp[ln]), .OB(tn[ln]));
    end
endmodule

// A reset that asserts asynchronously when ok falls and releases four clocks after it rises.
module hdmi_rst_bridge (input wire clk, input wire ok, output wire rst);
    (* ASYNC_REG = "TRUE" *) reg [3:0] sr = 4'hF;
    always @(posedge clk or negedge ok)
        if (!ok) sr <= 4'hF; else sr <= {sr[2:0], 1'b0};
    assign rst = sr[3];
endmodule

// 4 lanes x 10-bit words @ pclk -> 4 lanes x 8-bit words @ wclk (wclk = 5/4 pclk,
// frequency-locked, any phase). Bit 0 of each word is sent first.
module hdmi_tmds_gearbox (
    input  wire        pclk, prst,
    input  wire [39:0] din,
    input  wire        wclk, wrst,
    output wire [31:0] dout
);
    wire [39:0] word;
    wire [5:0]  level;
    wire        wr_busy, rd_busy;
    reg         started = 0;
    reg  [3:0]  cnt = 0;                  // bits held per lane: 0, 2, 4, 6 or 8
    wire        load = started && cnt < 8; // 4 loads every 5 wclk cycles

    xpm_fifo_async #(
        .FIFO_MEMORY_TYPE("distributed"), .FIFO_WRITE_DEPTH(32),
        .WRITE_DATA_WIDTH(40), .READ_DATA_WIDTH(40),
        .READ_MODE("fwft"), .FIFO_READ_LATENCY(0),
        .RD_DATA_COUNT_WIDTH(6), .USE_ADV_FEATURES("0400")   // rd_data_count
    ) fifo (
        .rst(prst), .wr_clk(pclk), .wr_en(!prst && !wr_busy), .din(din),
        .rd_clk(wclk), .rd_en(load), .dout(word), .rd_data_count(level),
        .wr_rst_busy(wr_busy), .rd_rst_busy(rd_busy),
        .sleep(1'b0), .injectsbiterr(1'b0), .injectdbiterr(1'b0)
    );

    always @(posedge wclk) begin
        if (wrst || rd_busy) started <= 0;
        else if (level >= 12) started <= 1;     // let the FIFO fill to about half
        cnt <= !started ? 4'd0 : load ? cnt + 4'd2 : cnt - 4'd8;
    end

    genvar l;
    for (l = 0; l < 4; l = l + 1) begin : lane
        reg  [17:0] buf_ = 0;
        reg  [7:0]  q = 0;
        wire [17:0] merged = buf_ | ({8'd0, word[l*10 +: 10]} << cnt);
        always @(posedge wclk)
            if (!started)  begin q <= 0;            buf_ <= 0;           end
            else if (load) begin q <= merged[7:0];  buf_ <= merged >> 8; end
            else           begin q <= buf_[7:0];    buf_ <= buf_ >> 8;   end
        assign dout[l*8 +: 8] = q;
    end
endmodule
