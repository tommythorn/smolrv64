`timescale 1ns / 1ps
`default_nettype none
// tb_hdmi_audio -- hdmi_symbols against an independent HDMI decoder.
//
// A small mode (64 x 8 active, negative syncs, the HSYNC edge 132 pixels before video) and an
// audio clock unrelated to the pixel clock, a frame every AT of its cycles. The decoder reads the
// three channels' symbols a pixel at a time -- TMDS video, TERC4, the control and guard-band codes
// -- follows the preambles as a sink does, assembles each packet, checks its BCH ECC, and checks:
// the video and the syncs, pixel for pixel, against the input DLY + 1 pixels earlier; every
// preamble's 8 pixels and every guard band's 2; the packet's first-pixel flag; every audio frame,
// in order, with its parity and its 192-frame block flag; the ACR's N and its CTS against the
// clocks' ratio; the AVI and Audio InfoFrames' checksums and their once-a-frame arrival.
module tb_hdmi_audio;
   localparam integer DLY = 12;
   localparam integer HA = 64, HFP = 16, HS = 20, HBP = 112, HT = HA + HFP + HS + HBP;
   localparam integer VA = 8, VFP = 2, VSY = 2, VBP = 4, VT = VA + VFP + VSY + VBP;
   localparam integer AT = 401;                // aud_clk cycles a frame: 1.4 a line, as at 640x480
   reg pclk = 1'b0, aclk = 1'b0, rst = 1'b1;
   always #3.5 pclk = ~pclk;                   // 7 ns pixel clock
   always #1.3 aclk = ~aclk;                   // 2.6 ns audio-side clock: CTS = 49 * AT * 2.6 / 7
   integer errors = 0;

   // ---- the timing generator: negative syncs, RGB a function of the position ----
   integer h = 0, v = 0;
   wire        de_i = h < HA && v < VA;
   wire        hs_i = !(h >= HA + HFP && h < HA + HFP + HS);
   wire        vs_i = !(v >= VA + VFP && v < VA + VFP + VSY);
   wire [15:0] rgb_i = de_i ? (h * 16'h0123 + v * 16'h4567) ^ 16'h5a5a : 16'h0000;
   always @(posedge pclk) if (!rst) begin
      h <= h == HT - 1 ? 0 : h + 1;
      if (h == HT - 1) v <= v == VT - 1 ? 0 : v + 1;
   end

   // ---- the audio frames ----
   reg        av = 1'b0;  reg [31:0] af = 32'd0;  integer ac = 0, a_sent = 0;
   reg [31:0] sent [0:8191];
   always @(posedge aclk) begin
      av <= 1'b0;
      if (!rst) begin
         ac <= ac == AT - 1 ? 0 : ac + 1;
         if (ac == AT - 1) begin
            af = {16'h8000 + a_sent * 16'd977, 16'h1234 ^ a_sent * 16'd613};
            av <= 1'b1;  sent[a_sent] = af;  a_sent = a_sent + 1;
         end
      end
   end

   wire [9:0] s_r, s_g, s_b;
   hdmi_symbols #(.DLY(DLY), .ISL_ROOM(100)) dut (
      .pix_clk(pclk), .rst(rst), .de(de_i), .hs(hs_i), .vs(vs_i), .rgb(rgb_i),
      .aud_clk(aclk), .aud_valid(av), .aud_frame(af), .s_r(s_r), .s_g(s_g), .s_b(s_b));

   // ---- the input, DLY + 1 pixels back ----
   localparam integer LAT = DLY + 1;
   reg [18:0] hist [0:LAT];                    // {de, hs, vs, rgb}
   integer q;
   always @(posedge pclk) begin
      hist[0] <= {de_i, hs_i, vs_i, rgb_i};
      for (q = 1; q <= LAT; q = q + 1) hist[q] <= hist[q-1];
   end
   wire        x_de = hist[LAT-1][18], x_hs = hist[LAT-1][17], x_vs = hist[LAT-1][16];
   wire [15:0] x_rgb = hist[LAT-1][15:0];

   // ---- the decoder ----
   function [4:0] ctl_dec(input [9:0] s);       // {valid, c1, c0}
      case (s)
         10'b1101010100: ctl_dec = 5'b10000;  10'b0010101011: ctl_dec = 5'b10001;
         10'b0101010100: ctl_dec = 5'b10010;  10'b1010101011: ctl_dec = 5'b10011;
         default:        ctl_dec = 5'b00000;
      endcase
   endfunction
   function [4:0] terc_dec(input [9:0] s);      // {valid, d[3:0]}
      case (s)
         10'b1010011100: terc_dec = 5'h10;  10'b1001100011: terc_dec = 5'h11;
         10'b1011100100: terc_dec = 5'h12;  10'b1011100010: terc_dec = 5'h13;
         10'b0101110001: terc_dec = 5'h14;  10'b0100011110: terc_dec = 5'h15;
         10'b0110001110: terc_dec = 5'h16;  10'b0100111100: terc_dec = 5'h17;
         10'b1011001100: terc_dec = 5'h18;  10'b0100111001: terc_dec = 5'h19;
         10'b0110011100: terc_dec = 5'h1A;  10'b1011000110: terc_dec = 5'h1B;
         10'b1010001110: terc_dec = 5'h1C;  10'b1001110001: terc_dec = 5'h1D;
         10'b0101100011: terc_dec = 5'h1E;  10'b1011000011: terc_dec = 5'h1F;
         default:        terc_dec = 5'h00;
      endcase
   endfunction
   function [7:0] tmds_dec(input [9:0] s);
      reg [7:0] d;  integer i;
      begin
         d = s[9] ? ~s[7:0] : s[7:0];
         tmds_dec[0] = d[0];
         for (i = 1; i < 8; i = i + 1) tmds_dec[i] = s[8] ? d[i] ^ d[i-1] : ~(d[i] ^ d[i-1]);
      end
   endfunction
   // the BCH(64,56)/(32,24) check: the parity of the data bits, serially, LSB first
   function [7:0] bch(input [55:0] d, input integer n);
      integer i;  reg fb;
      begin
         bch = 8'd0;
         for (i = 0; i < n; i = i + 1) begin
            fb = bch[0] ^ d[i];
            bch = {1'b0, bch[7:1]};
            if (fb) bch = bch ^ 8'b1000_0011;
         end
      end
   endfunction
   localparam [9:0] GB_A = 10'b1011001100, GB_B = 10'b0100110011;

   // the sink's view: after a run of 8 preamble pixels, what follows is a guard band and then video
   // (video preamble) or packets (island preamble)
   localparam integer K_CTRL = 0, K_VGUARD = 1, K_VIDEO = 2, K_IGUARD = 3, K_ISLAND = 4, K_ITRAIL = 5, K_HUNT = 6;
   integer kind = K_HUNT, pre_n = 0, pre_kind = 0, gn = 0, px = 0;   // a sink locks at a control period
   reg [31:0] hb;  reg [63:0] sb [0:3];
   integer a_got = 0, n_isl = 0, n_acr = 0, n_avi = 0, n_aif = 0, iec = 0;
   integer frames_seen = 0, avi_frame = -1, aif_frame = -1;
   reg [4:0] c0, c1, c2, t0, t1, t2;  reg [7:0] vr, vg, vb;
   reg       vs_prev = 1'b1;
   integer   i;
   real      cts_want;
   task packet;
      reg [23:0] hd;  reg [55:0] sp [0:3];  integer j;  reg [7:0] sum;
      begin
         hd = hb[23:0];
         if (bch({32'd0, hd}, 24) != hb[31:24]) begin $display("FAIL: header %h ECC %h, expected %h", hd, hb[31:24], bch({32'd0, hd}, 24)); errors = errors + 1; end
         for (j = 0; j < 4; j = j + 1) begin
            sp[j] = sb[j][55:0];
            if (bch(sp[j], 56) != sb[j][63:56]) begin $display("FAIL: subpacket %0d ECC", j); errors = errors + 1; end
         end
         case (hd[7:0])
            8'h00: ;
            8'h02: for (j = 0; j < 4; j = j + 1) if (hd[8 + j]) begin
                      if (sp[j][23:0] != {sent[a_got][15:0], 8'h00} || sp[j][47:24] != {sent[a_got][31:16], 8'h00}) begin
                         $display("FAIL: audio frame %0d = %h/%h, expected %h", a_got, sp[j][47:24], sp[j][23:0], sent[a_got]);  errors = errors + 1;
                      end
                      if (sp[j][51] != ^sp[j][23:0] || sp[j][55] != ^sp[j][47:24] || sp[j][50:48] != 0 || sp[j][54:52] != 0) begin
                         $display("FAIL: audio frame %0d parity or V/U/C", a_got);  errors = errors + 1;
                      end
                      if (hd[20 + j] != (iec == 0)) begin
                         $display("FAIL: audio frame %0d block flag %b at IEC frame %0d", a_got, hd[20 + j], iec);  errors = errors + 1;
                      end
                      a_got = a_got + 1;  iec = iec == 191 ? 0 : iec + 1;
                   end
            8'h01: begin
                      n_acr = n_acr + 1;
                      if ({sp[0][35:32], sp[0][47:40], sp[0][55:48]} != 20'd6272) begin $display("FAIL: ACR N"); errors = errors + 1; end
                      cts_want = 49.0 * AT * 2.6 / 7.0;
                      if ({sp[0][11:8], sp[0][23:16], sp[0][31:24]} < cts_want - 3 || {sp[0][11:8], sp[0][23:16], sp[0][31:24]} > cts_want + 3) begin
                         $display("FAIL: ACR CTS %0d, expected about %0f", {sp[0][11:8], sp[0][23:16], sp[0][31:24]}, cts_want);  errors = errors + 1;
                      end
                   end
            8'h82, 8'h84: begin
                      sum = hd[7:0] + hd[15:8] + hd[23:16];
                      for (j = 0; j < 4; j = j + 1) sum = sum + sp[j][7:0] + sp[j][15:8] + sp[j][23:16] + sp[j][31:24] + sp[j][39:32] + sp[j][47:40] + sp[j][55:48];
                      if (sum != 8'd0) begin $display("FAIL: InfoFrame %h checksum", hd[7:0]);  errors = errors + 1; end
                      if (hd[7:0] == 8'h82) begin n_avi = n_avi + 1;  if (avi_frame == frames_seen) begin $display("FAIL: two AVI InfoFrames in a frame"); errors = errors + 1; end  avi_frame = frames_seen; end
                      else                  begin n_aif = n_aif + 1;  aif_frame = frames_seen; end
                   end
            default: begin $display("FAIL: packet type %h", hd[7:0]);  errors = errors + 1; end
         endcase
      end
   endtask

   integer cyc = 0;
   always @(posedge pclk) if (!rst) begin
      cyc <= cyc + 1;
      c0 = ctl_dec(s_b);  c1 = ctl_dec(s_g);  c2 = ctl_dec(s_r);
      t0 = terc_dec(s_b); t1 = terc_dec(s_g); t2 = terc_dec(s_r);
      vr = tmds_dec(s_r); vg = tmds_dec(s_g); vb = tmds_dec(s_b);
      if (cyc > LAT + 2) begin
         if (x_vs == 1'b0 && vs_prev == 1'b1) frames_seen = frames_seen + 1;
         vs_prev = x_vs;
         case (kind)
            K_HUNT: if (c0[4] && c1[4] && c2[4] && !x_de && {c2[1:0], c1[1:0]} == 4'b0000) kind = K_CTRL;
            K_CTRL: begin
               if (c0[4] && c1[4] && c2[4]) begin
                  if ({c0[1], c0[0]} != {x_vs, x_hs}) begin $display("FAIL: control syncs %b%b at cycle %0d, expected %b%b", c0[1], c0[0], cyc, x_vs, x_hs); errors = errors + 1; end
                  if (x_de) begin $display("FAIL: control symbol in video at cycle %0d", cyc); errors = errors + 1; end
                  // {CTL3 CTL2 CTL1 CTL0}: 0001 video preamble, 0101 island preamble
                  if ({c2[1:0], c1[1:0]} == 4'b0001 || {c2[1:0], c1[1:0]} == 4'b0101) begin
                     if (pre_n != 0 && pre_kind != {c2[1:0], c1[1:0]}) begin $display("FAIL: preamble changes kind"); errors = errors + 1; end
                     pre_kind = {c2[1:0], c1[1:0]};  pre_n = pre_n + 1;
                  end else begin
                     if (pre_n != 0) begin $display("FAIL: a %0d-pixel preamble with nothing after it at cycle %0d", pre_n, cyc); errors = errors + 1; end
                     pre_n = 0;
                  end
               end else if (pre_n == 8 && pre_kind == 4'b0001 && s_b == GB_A && s_g == GB_B && s_r == GB_A) begin
                  kind = K_VGUARD;  gn = 1;  pre_n = 0;
               end else if (pre_n == 8 && pre_kind == 4'b0101 && s_g == GB_B && s_r == GB_B && t0[4] && t0[3:2] == 2'b11) begin
                  kind = K_IGUARD;  gn = 1;  pre_n = 0;  n_isl = n_isl + 1;
                  if (t0[1:0] != {x_vs, x_hs}) begin $display("FAIL: island guard syncs"); errors = errors + 1; end
               end else begin
                  $display("FAIL: unexpected symbols %b %b %b after a %0d-pixel preamble at cycle %0d (de %b)", s_r, s_g, s_b, pre_n, cyc, x_de);
                  errors = errors + 1;  pre_n = 0;
               end
            end
            K_VGUARD: begin
               if (!(s_b == GB_A && s_g == GB_B && s_r == GB_A)) begin $display("FAIL: video guard band"); errors = errors + 1; end
               gn = gn + 1;  if (gn == 2) kind = K_VIDEO;
            end
            K_VIDEO: begin
               if (!x_de) begin
                  kind = K_CTRL;
                  if (!(c0[4] && c1[4] && c2[4])) begin $display("FAIL: no control symbol after video at cycle %0d", cyc); errors = errors + 1; end
               end else if ({vr[7:3], vg[7:2], vb[7:3]} != x_rgb) begin
                  $display("FAIL: pixel %h, expected %h at cycle %0d", {vr[7:3], vg[7:2], vb[7:3]}, x_rgb, cyc);
                  errors = errors + 1;
               end
            end
            K_IGUARD: begin
               if (!(s_g == GB_B && s_r == GB_B && t0[4] && t0[3:2] == 2'b11)) begin $display("FAIL: island leading guard"); errors = errors + 1; end
               gn = gn + 1;  if (gn == 2) begin kind = K_ISLAND;  px = 0; end
            end
            K_ISLAND: begin
               if (s_g == GB_B && s_r == GB_B) begin                 // the trailing guard band
                  if (px % 32 != 0 || px == 0) begin $display("FAIL: island of %0d pixels", px); errors = errors + 1; end
                  kind = K_ITRAIL;  gn = 1;
               end else begin
                  if (!(t0[4] && t1[4] && t2[4])) begin $display("FAIL: not TERC4 in an island at cycle %0d", cyc); errors = errors + 1; end
                  if (t0[1:0] != {x_vs, x_hs}) begin $display("FAIL: island syncs at cycle %0d", cyc); errors = errors + 1; end
                  if (t0[3] != (px % 32 != 0)) begin $display("FAIL: first-pixel flag at packet pixel %0d", px % 32); errors = errors + 1; end
                  hb[px % 32] = t0[2];
                  for (i = 0; i < 4; i = i + 1) begin sb[i][2 * (px % 32)] = t1[i];  sb[i][2 * (px % 32) + 1] = t2[i]; end
                  px = px + 1;
                  if (px % 32 == 0) packet;
               end
            end
            K_ITRAIL: begin
               if (!(s_g == GB_B && s_r == GB_B && t0[4] && t0[3:2] == 2'b11)) begin $display("FAIL: island trailing guard"); errors = errors + 1; end
               kind = K_CTRL;
            end
         endcase
      end
   end

   initial begin
      repeat (10) @(posedge pclk);  rst = 1'b0;
      repeat (VT * HT * 12) @(posedge pclk);
      // every frame sent was received, but the few still waiting for an island
      if (a_sent - a_got > 6 || a_got < 100) begin $display("FAIL: %0d audio frames received of %0d sent", a_got, a_sent); errors = errors + 1; end
      if (n_acr < 3) begin $display("FAIL: %0d ACR packets", n_acr); errors = errors + 1; end
      if (n_avi < 9 || n_aif < 9) begin $display("FAIL: %0d AVI and %0d Audio InfoFrames in %0d frames", n_avi, n_aif, frames_seen); errors = errors + 1; end
      $display("tb_hdmi_audio: %0d islands, %0d audio frames, %0d ACR, %0d AVI, %0d AIF, %0d video frames", n_isl, a_got, n_acr, n_avi, n_aif, frames_seen);
      if (errors == 0) $display("tb_hdmi_audio: ALL TESTS PASSED");
      else             $display("tb_hdmi_audio: %0d FAILURES", errors);
      $finish;
   end
endmodule

`default_nettype wire
