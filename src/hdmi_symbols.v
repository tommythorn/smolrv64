`timescale 1ns / 1ps
`default_nettype none

// hdmi_symbols -- the pixel stream as HDMI's three TMDS channels' 10-bit symbols, with audio.
//
// HDMI mode (1.4, section 5.2): video data periods behind an 8-pixel video preamble and a 2-pixel
// guard band; a data island in every line's horizontal blanking; control symbols otherwise. The
// timing generator's stream is delayed DLY pixels inside, so the preamble is sent as the video's
// first pixel is seen arriving -- nothing here knows the mode's numbers.
//
// THE DATA ISLAND starts 4 pixels after the leading edge of HSYNC (its active level is the one it
// does not hold during video): the 8-pixel island preamble, a 2-pixel guard band, two 32-pixel
// packets, a 2-pixel guard band -- 80 pixels. Islands are sent only while the last line's HSYNC
// edge came at least ISL_ROOM pixels before its video (so an island and its trailing control
// period always end before the video preamble; every mode the monitor sets has 112 or more).
// Packet 0 carries the waiting audio frames (an Audio Sample packet, up to four), or nothing (a
// null packet). Packet 1 carries, in this order of priority: Audio Clock Regeneration when a new
// CTS has been measured, the AVI InfoFrame and the Audio InfoFrame once a frame (from the leading
// edge of VSYNC), else nothing.
//
// AUDIO arrives as 44.1 kHz stereo frames in aud_clk (virtio_snd: {right, left}, 16-bit signed,
// one a tick, silence included), held and toggled across: the next comes 7.5 K aud_clk cycles
// later, so the held word is stable long before the pixel side takes it. CTS is measured, not
// computed: the pixel clocks between every 49th frame's arrival (N = 6272 = 128 * 49, 44.1 kHz's
// recommended N), so the sink regenerates the rate the frames arrive at, whatever the pixel clock.
// Channel status is all zero: consumer, linear PCM, 44.1 kHz.
module hdmi_symbols #(
    parameter integer DLY      = 12,          // pixels the stream is delayed (>= 10: the preamble + guard)
    parameter integer ISL_ROOM = 100,         // HSYNC edge to video, at least, for islands
    parameter integer AUDIO    = 1            // 0: DVI (no islands, no preambles)
)(
    input  wire        pix_clk,
    input  wire        rst,                   // pix_clk's
    input  wire        de,
    input  wire        hs,
    input  wire        vs,
    input  wire [15:0] rgb,                   // RGB565
    input  wire        aud_clk,
    input  wire        aud_valid,
    input  wire [31:0] aud_frame,             // {right, left}
    output wire [9:0]  s_r,                   // channel 2
    output wire [9:0]  s_g,                   // channel 1
    output wire [9:0]  s_b                    // channel 0
);
    // ======================== audio frames and the CTS clock (aud_clk) ========================
    reg  [31:0] a_hold;
    reg         a_tog = 1'b0, c_tog = 1'b0;
    reg  [5:0]  a_n = 6'd0;                    // frames since the last CTS mark, mod 49
    always @(posedge aud_clk)
        if (aud_valid) begin
            a_hold <= aud_frame;  a_tog <= ~a_tog;
            if (a_n == 6'd48) begin a_n <= 6'd0;  c_tog <= ~c_tog; end
            else a_n <= a_n + 6'd1;
        end
    (* ASYNC_REG = "TRUE" *) reg [2:0] a_s = 3'd0, c_s = 3'd0;
    always @(posedge pix_clk) begin a_s <= {a_s[1:0], a_tog};  c_s <= {c_s[1:0], c_tog}; end
    wire a_new = a_s[2] ^ a_s[1];
    wire c_new = c_s[2] ^ c_s[1];

    // ======================== the stream, delayed (pix_clk) ========================
    reg [DLY-1:0] dl_de = 0, dl_hs = 0, dl_vs = 0;
    reg [15:0]    dl_rgb [0:DLY-1];
    integer k;
    always @(posedge pix_clk) begin
        dl_de <= {dl_de[DLY-2:0], de};  dl_hs <= {dl_hs[DLY-2:0], hs};  dl_vs <= {dl_vs[DLY-2:0], vs};
        dl_rgb[0] <= rgb;
        for (k = 1; k < DLY; k = k + 1) dl_rgb[k] <= dl_rgb[k-1];
    end
    wire        d_de = dl_de[DLY-1], d_hs = dl_hs[DLY-1], d_vs = dl_vs[DLY-1];
    wire [15:0] d_rgb = dl_rgb[DLY-1];

    // the video preamble and guard band: counted from the undelayed stream's video edge
    reg  de_p = 1'b0;
    reg  [4:0] vc = 5'd0;                      // DLY - vc pixels to the delayed video edge
    always @(posedge pix_clk)
        if (rst) begin de_p <= 1'b0;  vc <= 5'd0; end
        else begin
            de_p <= de;
            if (de && !de_p) vc <= 5'd1;
            else if (vc != 5'd0) vc <= (vc == DLY - 1) ? 5'd0 : vc + 5'd1;
        end
    wire v_pre   = AUDIO != 0 && vc >= DLY - 10 && vc < DLY - 2;
    wire v_guard = AUDIO != 0 && vc >= DLY - 2;

    // ======================== the island's timing (delayed stream) ========================
    reg        hs_idle = 1'b0, vs_idle = 1'b0, hs_p = 1'b0, vs_p = 1'b0, d_de_p = 1'b0;
    reg [11:0] since = 12'd0;                  // pixels since HSYNC's leading edge
    reg        room = 1'b0;                    // the last line's edge-to-video distance was enough
    reg [6:0]  ic = 7'd0;                      // the island's pixel, 1..80 while it runs
    wire       hs_lead = d_hs != hs_idle && hs_p == hs_idle;
    wire       vs_lead = d_vs != vs_idle && vs_p == vs_idle;
    always @(posedge pix_clk)
        if (rst) begin room <= 1'b0;  ic <= 7'd0; end
        else begin
            hs_p <= d_hs;  vs_p <= d_vs;  d_de_p <= d_de;
            if (d_de) begin hs_idle <= d_hs;  vs_idle <= d_vs; end
            since <= hs_lead ? 12'd0 : (since == 12'hfff ? since : since + 12'd1);
            if (d_de && !d_de_p) room <= since >= ISL_ROOM;
            if (hs_lead && room && AUDIO != 0) ic <= 7'd1;
            else if (ic != 7'd0) ic <= (ic == 7'd83) ? 7'd0 : ic + 7'd1;
        end
    // ic 1..3 control, 4..11 preamble, 12..13 guard, 14..45 packet 0, 46..77 packet 1, 78..79 guard
    wire i_pre   = ic >= 7'd4  && ic < 7'd12;
    wire i_guard = (ic >= 7'd12 && ic < 7'd14) || (ic >= 7'd78 && ic < 7'd80);
    wire i_data  = ic >= 7'd14 && ic < 7'd78;
    wire [4:0] i_px = ic - 7'd14;              // the pixel within its packet (mod 32)

    // ======================== the packets ========================
    // frames waiting for an island: up to four, taken as they arrive
    reg [31:0] st [0:3];
    reg [2:0]  sn = 3'd0;
    reg [7:0]  iec = 8'd0;                     // the next frame's place in its 192-frame block
    reg [19:0] pcnt = 20'd0, cts = 20'd0;
    reg        acr_pend = 1'b0, avi_pend = 1'b0, aif_pend = 1'b0, cts_ok = 1'b0;
    reg [247:0] pkt0 = 248'd0, pkt1 = 248'd0;  // {HB2, HB1, HB0, sub3 .. sub0}
    localparam [19:0] N = 20'd6272;
    wire        build = ic == 7'd1;            // the island's packets are formed here
    // the Audio Sample packet of the waiting frames (2-channel layout; V = U = C = 0)
    function [55:0] asp_sub(input [31:0] f);
        reg [23:0] l, r;
        begin
            l = {f[15:0], 8'h00};  r = {f[31:16], 8'h00};
            asp_sub = {^r, 3'b000, ^l, 3'b000, r, l};
        end
    endfunction
    reg [247:0] asp;  reg [3:0] bflag;
    always @* begin : mk_asp
        integer j;  reg [8:0] p;
        bflag = 4'd0;
        for (j = 0; j < 4; j = j + 1) begin
            p = {1'b0, iec} + j;
            if (j < sn && (p == 9'd0 || p == 9'd192)) bflag[j] = 1'b1;
        end
        asp = {bflag, 4'h0, 4'h0, (sn >= 3'd4), (sn >= 3'd3), (sn >= 3'd2), (sn >= 3'd1), 8'h02,
               sn >= 3'd4 ? asp_sub(st[3]) : 56'd0, sn >= 3'd3 ? asp_sub(st[2]) : 56'd0,
               sn >= 3'd2 ? asp_sub(st[1]) : 56'd0, sn >= 3'd1 ? asp_sub(st[0]) : 56'd0};
    end
    wire [55:0]  acr_sub = {N[7:0], N[15:8], 4'h0, N[19:16], cts[7:0], cts[15:8], 4'h0, cts[19:16], 8'h00};
    wire [247:0] acr = {8'h00, 8'h00, 8'h01, acr_sub, acr_sub, acr_sub, acr_sub};
    // AVI InfoFrame v2: RGB, no aspect data (R = same as picture), full-range RGB, no VIC (these are
    // not CEA timings); Audio InfoFrame v1: 2 channels, coding, rate and size from the stream, FL/FR
    wire [247:0] avi = {8'h0D, 8'h02, 8'h82, 168'd0, 8'h00, 8'h00, 8'h00, 8'h08, 8'h08, 8'h00, 8'h5F};
    wire [247:0] aif = {8'h0A, 8'h01, 8'h84, 168'd0, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h01, 8'h70};
    always @(posedge pix_clk)
        if (rst) begin
            sn <= 3'd0;  iec <= 8'd0;  pcnt <= 20'd0;  cts_ok <= 1'b0;
            acr_pend <= 1'b0;  avi_pend <= 1'b0;  aif_pend <= 1'b0;
        end else begin
            // CTS: the pixel clocks between two marks (the first mark only starts the count)
            pcnt <= c_new ? 20'd1 : pcnt + 20'd1;
            if (c_new) begin
                if (cts_ok) begin cts <= pcnt;  acr_pend <= 1'b1; end
                cts_ok <= 1'b1;
            end
            if (vs_lead) begin avi_pend <= 1'b1;  aif_pend <= 1'b1; end
            if (build) begin
                pkt0 <= sn != 3'd0 ? asp : 248'd0;
                iec  <= ({1'b0, iec} + sn >= 9'd192) ? iec + sn - 8'd192 : iec + sn;
                sn   <= 3'd0;                                    // (a frame arriving now: below)
                if (acr_pend)      begin pkt1 <= acr;  acr_pend <= 1'b0; end
                else if (avi_pend) begin pkt1 <= avi;  avi_pend <= 1'b0; end
                else if (aif_pend) begin pkt1 <= aif;  aif_pend <= 1'b0; end
                else pkt1 <= 248'd0;
            end
            if (a_new) begin
                if (build) begin st[0] <= a_hold;  sn <= 3'd1; end
                else if (sn != 3'd4) begin st[sn[1:0]] <= a_hold;  sn <= sn + 3'd1; end
            end
        end
    always @(posedge pix_clk) if (!rst && a_new && !build && sn == 3'd4)
        $fatal(1, "hdmi_symbols: a fifth audio frame waits for an island (islands too rare, or none)");

    // BCH ECC, generator x^8 + x^7 + x^6 + 1, the data LSB first
    function [7:0] ecc(input [55:0] d, input integer n);
        integer b;
        begin
            ecc = 8'd0;
            for (b = 0; b < 56; b = b + 1)
                if (b < n) ecc = (ecc >> 1) ^ ((ecc[0] ^ d[b]) ? 8'h83 : 8'h00);
        end
    endfunction
    // the packet being sent, as shift registers: the header a bit a pixel, the subpackets two
    reg  [31:0] hdr = 32'd0;
    reg  [63:0] sub [0:3];
    wire        load0 = ic == 7'd13, load1 = ic == 7'd45;
    integer m;
    always @(posedge pix_clk)
        if (load0 || load1) begin : ld
            reg [247:0] p;
            p = load0 ? pkt0 : pkt1;
            hdr <= {ecc({32'd0, p[247:224]}, 24), p[247:224]};
            for (m = 0; m < 4; m = m + 1) sub[m] <= {ecc(p[m*56 +: 56], 56), p[m*56 +: 56]};
        end else begin
            hdr <= hdr >> 1;
            for (m = 0; m < 4; m = m + 1) sub[m] <= sub[m] >> 2;
        end
    wire [3:0] d0 = {i_guard || i_px != 5'd0, i_guard || hdr[0], d_vs, d_hs};
    wire [3:0] d1 = {sub[3][0], sub[2][0], sub[1][0], sub[0][0]};
    wire [3:0] d2 = {sub[3][1], sub[2][1], sub[1][1], sub[0][1]};

    // ======================== the symbols ========================
    // TERC4 (HDMI 1.4 table 5-17)
    function [9:0] terc4(input [3:0] d);
        case (d)
            4'h0: terc4 = 10'b1010011100;  4'h1: terc4 = 10'b1001100011;
            4'h2: terc4 = 10'b1011100100;  4'h3: terc4 = 10'b1011100010;
            4'h4: terc4 = 10'b0101110001;  4'h5: terc4 = 10'b0100011110;
            4'h6: terc4 = 10'b0110001110;  4'h7: terc4 = 10'b0100111100;
            4'h8: terc4 = 10'b1011001100;  4'h9: terc4 = 10'b0100111001;
            4'hA: terc4 = 10'b0110011100;  4'hB: terc4 = 10'b1011000110;
            4'hC: terc4 = 10'b1010001110;  4'hD: terc4 = 10'b1001110001;
            4'hE: terc4 = 10'b0101100011;  default: terc4 = 10'b1011000011;
        endcase
    endfunction
    localparam [9:0] GB_A = 10'b1011001100, GB_B = 10'b0100110011;
    // what each channel sends instead of video or a control symbol (alt), and the control bits
    reg  [9:0] alt_r, alt_g, alt_b;  reg use_alt;  reg [3:0] ctl;
    always @* begin
        use_alt = 1'b0;  alt_r = 10'd0;  alt_g = 10'd0;  alt_b = 10'd0;
        ctl = v_pre ? 4'b0001 : (i_pre ? 4'b0101 : 4'b0000);          // {CTL3, CTL2, CTL1, CTL0}
        if (!d_de && v_guard)  begin use_alt = 1'b1;  alt_r = GB_A;         alt_g = GB_B;         alt_b = GB_A;      end
        else if (i_guard)      begin use_alt = 1'b1;  alt_r = GB_B;         alt_g = GB_B;         alt_b = terc4(d0); end
        else if (i_data)       begin use_alt = 1'b1;  alt_r = terc4(d2);    alt_g = terc4(d1);    alt_b = terc4(d0); end
    end
    hdmi_tmds_encoder enc_r (.clk(pix_clk), .vd({d_rgb[15:11], d_rgb[15:13]}), .cd(ctl[3:2]),     .vde(d_de),
                             .use_alt(use_alt), .alt(alt_r), .tmds(s_r));
    hdmi_tmds_encoder enc_g (.clk(pix_clk), .vd({d_rgb[10: 5], d_rgb[10: 9]}), .cd(ctl[1:0]),     .vde(d_de),
                             .use_alt(use_alt), .alt(alt_g), .tmds(s_g));
    hdmi_tmds_encoder enc_b (.clk(pix_clk), .vd({d_rgb[ 4: 0], d_rgb[ 4: 2]}), .cd({d_vs, d_hs}), .vde(d_de),
                             .use_alt(use_alt), .alt(alt_b), .tmds(s_b));

    always @(posedge pix_clk) if (!rst && ic != 7'd0 && (d_de || v_pre || v_guard))
        $fatal(1, "hdmi_symbols: a data island runs into the video (island pixel %0d)", ic);
endmodule

// The DVI 1.0 TMDS encoder: 8 bits to 10, transition-minimized and DC-balanced, or one of the
// four control symbols while vde is low -- or, while vde is low and use_alt is set, alt (a guard
// band or a TERC4 symbol, which carry no disparity).
module hdmi_tmds_encoder (
    input  wire       clk,
    input  wire [7:0] vd,     // video data
    input  wire [1:0] cd,     // control data, sent while vde = 0
    input  wire       vde,
    input  wire       use_alt,
    input  wire [9:0] alt,
    output reg  [9:0] tmds = 10'd0
);
    wire [3:0] n1 = vd[0] + vd[1] + vd[2] + vd[3] + vd[4] + vd[5] + vd[6] + vd[7];
    wire       xnr = (n1 > 4'd4) || (n1 == 4'd4 && vd[0] == 1'b0);
    // each bit the one below XOR the data, XNOR'd when xnr; bit 8 says which
    function [8:0] trans(input [7:0] v, input x);
        integer k;
        begin
            trans[0] = v[0];
            for (k = 1; k < 8; k = k + 1) trans[k] = trans[k-1] ^ v[k] ^ x;
            trans[8] = ~x;
        end
    endfunction
    wire [8:0] q_m = trans(vd, xnr);

    reg  [3:0] acc = 4'd0;                // the running disparity
    wire [3:0] bal = q_m[0] + q_m[1] + q_m[2] + q_m[3] + q_m[4] + q_m[5] + q_m[6] + q_m[7] - 4'd4;
    wire       same = bal[3] == acc[3];
    wire       inv  = (bal == 4'd0 || acc == 4'd0) ? ~q_m[8] : same;
    wire [3:0] inc  = bal - ({q_m[8] ^ ~same} & ~(bal == 4'd0 || acc == 4'd0));
    wire [3:0] acc_n = inv ? acc - inc : acc + inc;
    wire [9:0] data = {inv, q_m[8], q_m[7:0] ^ {8{inv}}};
    wire [9:0] code = cd[1] ? (cd[0] ? 10'b1010101011 : 10'b0101010100)
                            : (cd[0] ? 10'b0010101011 : 10'b1101010100);
    always @(posedge clk) begin
        tmds <= vde ? data : use_alt ? alt : code;
        acc  <= vde ? acc_n : 4'd0;
    end
endmodule

`default_nettype wire
