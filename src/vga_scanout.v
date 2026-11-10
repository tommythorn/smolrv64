`timescale 1ns / 1ps
`default_nettype none
// vga_scanout -- the display side of simmerv's `--graphics WxH`: scans an RGB565 framebuffer in
// DRAM out to a VGA connector through a programmable timing generator.
//
// The framebuffer is ordinary RAM that Linux's simplefb draws into (docs/PLAN-2026-10-03-
// graphics-console.md); this block only reads it. Two clock domains:
//
//   clk (ui_clk)  the MMIO registers, and the fetcher: an AXI read master that walks the frame,
//                 frame after frame, in 64-byte bursts, several in flight, into a FIFO of beats.
//                 It asks for a burst whenever the FIFO has room for it beside those in flight, so
//                 the FIFO's depth is the DRAM latency the scan rides through, whatever the mode.
//   pix_clk       the timing generator, which takes a beat from the FIFO every four pixels of a
//                 line's STRIDE/2. It starts in the vertical blank, so the FIFO fills before the
//                 first line.
//
// The FIFO's two counts cross as Gray codes (XDC: bus skew). Each beat carries a flag, set on a
// frame's first, so the scan can find its place in the stream again: a beat that is missing when
// a pixel needs it sends the scan hunting -- the screen black, every beat but a frame's first
// thrown away -- until it holds a frame's first in a vertical blank, and the next frame is whole. A frame not shown whole is counted
// (STATUS[31:16]).
// The timing registers cross quasi-statically: software changes them only with CTRL.enable clear
// (XDC: false paths into pix_clk).
//
// THE SYNCS ARE FREE-RUNNING. The timing generator's counters, and the syncs and blanking they
// make, depend on their own state and CTRL.enable alone (a mode change clears it): nothing the
// memory side does, nor the pixel clock's lock, restarts them. A frame the FIFO ran dry in goes
// black from there (counted); a lock loss is counted too (STATUS[15:8]), and the counters ride
// through it on whatever the clock does.
//
// Registers (32-bit, full-word writes only), offsets within the 256-byte page:
//   0x00 CTRL         [0] enable  [1] HSYNC active-high  [2] VSYNC active-high
//   0x04 FB_BASE      physical address of pixel (0,0); 64-byte aligned
//   0x08 STRIDE       bytes per line; a multiple of 64, below 16384; the scan takes a line's
//                     STRIDE/2 pixels, which must fit in H_TOTAL (those past H_ACTIVE unseen)
//   0x10 H_ACTIVE     0x14 H_SYNC_START  0x18 H_SYNC_END  0x1C H_TOTAL   (pixels)
//   0x20 V_ACTIVE     0x24 V_SYNC_START  0x28 V_SYNC_END  0x2C V_TOTAL   (lines)
//   0x30 STATUS (ro)  [0] pixel clock locked  [1] DRP busy  [15:8] pixel clock lock losses while
//                     enabled (wraps)  [31:16] frames not shown whole: a beat missing, or the
//                     scan still looking for a frame's first (wraps)
//   0x40 DRP          write {we[31], addr[22:16], data[15:0]} starts one access of the pixel
//                     clock's MMCM; read gives {busy[31], last read data[15:0]}
//   0x44 PIXCLK_CTRL  [0] hold the pixel-clock MMCM in reset
// Reset values are VESA 640x480@60 (negative syncs) at FB_BASE 0xFF00_0000, the base of the
// framebuffer region, the top 16 MiB of 2 GiB. The board's MMCM comes up at 166.67 MHz, the clock
// the pixel domain is timed at, so the ROM monitor sets each mode's clock over DRP -- its reset
// mode's 25 MHz included -- before it enables the scan (workloads/monitor/video.c).
//
// Pixels are RGB565, little-endian, four to a 64-bit beat; the output is the pixel, zero in the
// blanking, with its data enable and the syncs: a board takes what its connector needs (the RK
// board the top two bits of each channel, RGB222, for the TinyVGA adapter; HDMI all of it).
module vga_scanout (
    input  wire        clk,
    input  wire        reset,

    input  wire [ 7:0] mmio_addr,
    input  wire        mmio_write,
    input  wire [31:0] mmio_wdata,
    input  wire [ 3:0] mmio_be,
    output reg  [31:0] mmio_rdata,

    output wire [ 2:0] m_axi_arid,
    output wire [31:0] m_axi_araddr,
    output wire [ 7:0] m_axi_arlen,
    output wire [ 2:0] m_axi_arsize,
    output wire [ 1:0] m_axi_arburst,
    output wire        m_axi_arlock,
    output wire [ 3:0] m_axi_arcache,
    output wire [ 2:0] m_axi_arprot,
    output wire [ 3:0] m_axi_arqos,
    output wire        m_axi_arvalid,
    input  wire        m_axi_arready,
    input  wire [ 2:0] m_axi_rid,
    input  wire [63:0] m_axi_rdata,
    input  wire [ 1:0] m_axi_rresp,
    input  wire        m_axi_rlast,
    input  wire        m_axi_rvalid,
    output wire        m_axi_rready,

    // The pixel clock's MMCM, owned by the platform. DRP is a four-phase handshake: drp_req
    // rises with the access latched, the platform raises drp_ack with drp_do valid, drp_req
    // falls, drp_ack falls. The platform synchronizes drp_req into its DRP clock.
    output reg         pixclk_rst,
    input  wire        pixclk_locked,
    output reg         drp_req,
    output reg         drp_we,
    output reg  [ 6:0] drp_addr,
    output reg  [15:0] drp_di,
    input  wire        drp_ack,
    input  wire [15:0] drp_do,

    input  wire        pix_clk,
    output reg         vid_de,
    output reg         vid_hs,
    output reg         vid_vs,
    output reg  [15:0] vid_rgb
);
   // ================================ registers (clk) ================================
   reg        en, hs_pol, vs_pol;
   reg [31:0] fb_base;
   reg [13:0] stride;
   reg [11:0] h_active, h_sync_start, h_sync_end, h_total;
   reg [11:0] v_active, v_sync_start, v_sync_end, v_total;
   reg [15:0] dry_frames;
   reg [7:0]  lock_lost;
   reg [15:0] drp_rdata;

   (* async_reg = "true" *) reg [1:0] locked_s, ack_s;
   reg locked_q;
   always @(posedge clk) begin locked_s <= {locked_s[0], pixclk_locked}; ack_s <= {ack_s[0], drp_ack}; locked_q <= locked_s[1]; end
   // a lock lost while the scan runs (a mode change reprograms the clock with the scan off)
   always @(posedge clk)
      if (reset) lock_lost <= 8'd0;
      else if (en & locked_q & ~locked_s[1]) lock_lost <= lock_lost + 8'd1;
   wire drp_busy = drp_req | ack_s[1];

   always @(posedge clk) begin
      if (reset) begin
         en <= 1'b0;  hs_pol <= 1'b0;  vs_pol <= 1'b0;
         fb_base <= 32'hFF00_0000;  stride <= 14'd1280;
         h_active <= 12'd640;  h_sync_start <= 12'd656;  h_sync_end <= 12'd752;  h_total <= 12'd800;
         v_active <= 12'd480;  v_sync_start <= 12'd490;  v_sync_end <= 12'd492;  v_total <= 12'd525;
         pixclk_rst <= 1'b0;
         drp_req <= 1'b0;  drp_we <= 1'b0;  drp_addr <= 7'd0;  drp_di <= 16'd0;  drp_rdata <= 16'd0;
      end else begin
         if (mmio_write) begin
            if (mmio_be != 4'hf)
               $fatal(1, "vga_scanout: partial write (be=%b) at %h; registers take full words", mmio_be, mmio_addr);
            case (mmio_addr[7:2])
               6'h00: {vs_pol, hs_pol, en} <= mmio_wdata[2:0];
               6'h01: fb_base      <= mmio_wdata;
               6'h02: stride       <= mmio_wdata[13:0];
               6'h04: h_active     <= mmio_wdata[11:0];
               6'h05: h_sync_start <= mmio_wdata[11:0];
               6'h06: h_sync_end   <= mmio_wdata[11:0];
               6'h07: h_total      <= mmio_wdata[11:0];
               6'h08: v_active     <= mmio_wdata[11:0];
               6'h09: v_sync_start <= mmio_wdata[11:0];
               6'h0a: v_sync_end   <= mmio_wdata[11:0];
               6'h0b: v_total      <= mmio_wdata[11:0];
               6'h10: begin
                  if (drp_busy)
                     $fatal(1, "vga_scanout: DRP access started while the last one is still busy");
                  drp_req <= 1'b1;  drp_we <= mmio_wdata[31];
                  drp_addr <= mmio_wdata[22:16];  drp_di <= mmio_wdata[15:0];
               end
               6'h11: pixclk_rst <= mmio_wdata[0];
               default: ;   // writes to read-only or unused offsets are ignored, as for a ROM
            endcase
            if (mmio_addr[7:2] == 6'h02 && (mmio_wdata[5:0] != 6'd0 || mmio_wdata[31:14] != 0))
               $fatal(1, "vga_scanout: STRIDE %0d is not a multiple of 64 bytes below 16384", mmio_wdata);

            if (mmio_addr[7:2] == 6'h01 && mmio_wdata[5:0] != 6'd0)
               $fatal(1, "vga_scanout: FB_BASE %h is not 64-byte aligned", mmio_wdata);
         end
         if (drp_req && ack_s[1]) begin drp_req <= 1'b0;  drp_rdata <= drp_do; end
      end
   end

   always @* begin
      case (mmio_addr[7:2])
         6'h00: mmio_rdata = {29'd0, vs_pol, hs_pol, en};
         6'h01: mmio_rdata = fb_base;
         6'h02: mmio_rdata = {18'd0, stride};
         6'h04: mmio_rdata = {20'd0, h_active};
         6'h05: mmio_rdata = {20'd0, h_sync_start};
         6'h06: mmio_rdata = {20'd0, h_sync_end};
         6'h07: mmio_rdata = {20'd0, h_total};
         6'h08: mmio_rdata = {20'd0, v_active};
         6'h09: mmio_rdata = {20'd0, v_sync_start};
         6'h0a: mmio_rdata = {20'd0, v_sync_end};
         6'h0b: mmio_rdata = {20'd0, v_total};
         6'h0c: mmio_rdata = {dry_frames, lock_lost, 6'd0, drp_busy, locked_s[1]};
         6'h10: mmio_rdata = {drp_busy, 15'd0, drp_rdata};
         6'h11: mmio_rdata = {31'd0, pixclk_rst};
         default: mmio_rdata = 32'd0;
      endcase
   end

   // ============== the FIFO: DEPTH beats, written in clk, read in pix_clk ==============
   // 16 KiB: 65 us of a 2560-wide line's scan at 125 MHz, 115 us of 1280 wide at 71.43 MHz.
   localparam integer PB = 11, DEPTH = 1 << PB;
   localparam [PB:0] FULL = DEPTH;
   reg [64:0] lb [0:DEPTH-1];                 // a beat, and [64]: it is a frame's first
   reg        lb_we;
   reg [PB-1:0] lb_waddr;
   reg [64:0] lb_wdata;
   always @(posedge clk) if (lb_we) lb[lb_waddr] <= lb_wdata;

   // The counts are PB+1 bits: full and empty differ. Each moves by at most one a clock of its
   // side, so its Gray code changes one bit at a time.
   function [PB:0] bin_of(input [PB:0] g);
      integer i;
      begin
         bin_of[PB] = g[PB];
         for (i = PB - 1; i >= 0; i = i - 1) bin_of[i] = bin_of[i + 1] ^ g[i];
      end
   endfunction
   reg [PB:0] wp_g;                           // the beats landed (clk), Gray
   reg [PB:0] rp_g;                           // the beats taken (pix_clk), Gray

   // ================================ timing (pix_clk) ================================
   (* async_reg = "true" *) reg [1:0] en_p;
   (* async_reg = "true" *) reg [PB:0] wp_s0, wp_s1;
   reg [PB:0] wp_p;           // the beats landed, as the scan sees them
   reg [11:0] h, v;
   reg [PB:0] rp;             // the beats taken
   reg [64:0] head;           // lb[rp]: the next beat, once the FIFO is not empty
   reg        hunt;           // the screen is black; looking for a frame's first beat
   reg        dry_tog = 1'b0; // a frame not shown whole
   always @(posedge pix_clk) begin
      en_p <= {en_p[0], en};
      wp_s0 <= wp_g;  wp_s1 <= wp_s0;  wp_p <= bin_of(wp_s1);
   end

   wire        last_h  = h == h_total - 12'd1;
   wire        last_v  = v == v_total - 12'd1;
   wire        act_v   = v < v_active;
   wire        in_line = act_v && {1'b0, h} < stride[13:1];     // a pixel the line's beats hold
   wire        empty   = rp == wp_p;
   // A beat's first pixel needs it at the head.
   wire        need    = !hunt && in_line && h[1:0] == 2'd0;
   wire        bad     = need && empty;
   wire        take    = hunt ? !empty && !head[64] : in_line && h[1:0] == 2'd3;
   wire [PB:0] rp_n    = rp + {{PB{1'b0}}, take};
   wire [PB:0] occ_p   = wp_p - rp;
   always @(posedge pix_clk) begin
      head <= lb[rp_n[PB-1:0]];
      if (!en_p[1]) begin
         // the scan starts in the vertical blank: the fetcher has the blank to fill the FIFO
         h <= 12'd0;  v <= v_active;  rp <= {(PB+1){1'b0}};  rp_g <= {(PB+1){1'b0}};  hunt <= 1'b1;
      end else begin
         h <= last_h ? 12'd0 : h + 12'd1;
         if (last_h) v <= last_v ? 12'd0 : v + 12'd1;
         if (take && empty)
            $fatal(1, "vga_scanout: a beat taken from the empty FIFO");
         // the scan takes a frame's beats as the fetcher sends them: a frame's first is at (0, 0)
         if (need && !empty && head[64] != (v == 12'd0 && h == 12'd0))
            $fatal(1, "vga_scanout: the beat at (%0d, %0d) is %0sa frame's first", h, v, head[64] ? "" : "not ");
         if (occ_p > FULL)
            $fatal(1, "vga_scanout: %0d beats landed past the %0d taken, more than the FIFO holds", wp_p, rp);
         rp <= rp_n;  rp_g <= rp ^ (rp >> 1);
         if (bad) hunt <= 1'b1;
         else if (hunt && !act_v && !empty && head[64]) hunt <= 1'b0;
         // a frame not shown whole: it went wrong, or began with the scan hunting
         if (bad || (hunt && v == 12'd0 && h == 12'd0)) dry_tog <= ~dry_tog;
      end
   end

   // Two stages: the pixel (with the syncs and blanking alongside), then the output register.
   reg        act1, hs1, vs1;  reg [15:0] px1;
   always @(posedge pix_clk) begin
      act1 <= en_p[1] && h < h_active && act_v;
      hs1  <= en_p[1] && h >= h_sync_start && h < h_sync_end;
      // VSYNC changes with HSYNC's leading edge, as CEA-861 and VESA timings have it: a sink
      // counts lines from HSYNC to HSYNC, and an edge inside a line moves its count by one.
      if (!en_p[1])               vs1 <= 1'b0;
      else if (h == h_sync_start) vs1 <= v >= v_sync_start && v < v_sync_end;
      px1  <= (hunt || bad || !in_line) ? 16'd0 : head[{h[1:0], 4'd0} +: 16];
   end
   always @(posedge pix_clk) begin
      vid_de  <= act1;
      vid_rgb <= act1 ? px1 : 16'd0;
      vid_hs  <= hs1 ~^ hs_pol;    // active level when in the sync pulse
      vid_vs  <= vs1 ~^ vs_pol;
   end

   // ================================ fetch (clk) ================================
   // The fetcher reads the frame line after line, a burst at a time, up to NOUT bursts in flight
   // (one ID: they answer in order), whenever the FIFO has room for one more beside those it
   // has asked for; the answers land in order, the first beat of each frame flagged.
   localparam [2:0] NOUT = 3'd4;
   localparam [PB:0] BURST = 8, ROOM = DEPTH - 8;
   (* async_reg = "true" *) reg [PB:0] rp_s0, rp_s1;
   (* async_reg = "true" *) reg [2:0] dry_s;
   (* async_reg = "true" *) reg [1:0] en_a;   // the scan is running: it has started over
   reg [PB:0] rp_c;           // the beats taken, as the fetcher sees them
   reg [PB:0] qp;             // the beats asked for: their room is the FIFO's from the asking
   reg [PB:0] lp;             // the beats landed
   reg [11:0] f_line;         // the line being asked for
   reg [ 7:0] f_bur;          // ...and the bursts of it asked for
   reg [31:0] f_addr;
   reg        ar_v;
   reg [ 2:0] f_out;          // bursts in flight
   reg [11:0] r_line;         // the line the answers land in
   reg [10:0] r_word;         // ...and the beat of it
   wire [7:0]  bursts = stride[13:6];                                  // a line's bursts
   wire [10:0] beats  = stride[13:3];                                  // ...and beats
   wire       line_last = f_bur == bursts - 8'd1;
   wire       ar_go = ar_v & m_axi_arready;
   wire       r_go  = m_axi_rvalid & m_axi_rready;
   wire [PB:0] occ_c = qp - rp_c;
   wire       ask   = en & en_a[1] & ~ar_v & (f_out < NOUT) & (occ_c <= ROOM);
   always @(posedge clk) begin
      rp_s0 <= rp_g;  rp_s1 <= rp_s0;  rp_c <= bin_of(rp_s1);
      dry_s <= {dry_s[1:0], dry_tog};  en_a <= {en_a[0], en_p[1]};
      wp_g  <= lp ^ (lp >> 1);       // a cycle after lp: with the beat's write
      lb_we <= 1'b0;
      if (reset) begin
         dry_frames <= 16'd0;  ar_v <= 1'b0;  f_out <= 3'd0;
         qp <= {(PB+1){1'b0}};  lp <= {(PB+1){1'b0}};  f_line <= 12'd0;  f_bur <= 8'd0;  f_addr <= 32'd0;
         r_line <= 12'd0;  r_word <= 11'd0;
      end else begin
         if (dry_s[2] ^ dry_s[1]) dry_frames <= dry_frames + 16'd1;   // counted, never silent (STATUS[31:16])
         if (en && en_a[1] && occ_c > FULL)
            $fatal(1, "vga_scanout: %0d beats asked for past the %0d taken, more than the FIFO holds", qp, rp_c);
         if (mmio_write && mmio_addr[7:2] == 6'h00 && mmio_wdata[0] && !en) begin
            if ({1'b0, h_total} < stride[13:1])
               $fatal(1, "vga_scanout: a line's %0d pixels of STRIDE exceed H_TOTAL %0d", stride[13:1], h_total);
            if (en_a[1] || ar_v || f_out != 3'd0)
               $fatal(1, "vga_scanout: enabled again before the last scan stopped and its bursts landed");
         end
         // asking: a burst at f_addr, then the next; the frame's last line wraps to its first
         if (ask) begin ar_v <= 1'b1;  qp <= qp + BURST; end
         if (ar_go) begin
            ar_v  <= 1'b0;
            f_bur <= line_last ? 8'd0 : f_bur + 8'd1;
            if (line_last) begin
               f_line <= (f_line == v_active - 12'd1) ? 12'd0 : f_line + 12'd1;
               f_addr <= (f_line == v_active - 12'd1) ? fb_base : f_addr + 32'd64;
            end else f_addr <= f_addr + 32'd64;
         end
         f_out <= f_out + {2'd0, ar_go} - {2'd0, r_go & m_axi_rlast};
         // landing: in order, a beat at a time, into the room asked for
         if (r_go) begin
            if (m_axi_rresp != 2'b00 || m_axi_rid != 3'd0)
               $fatal(1, "vga_scanout: read response resp=%b id=%0d", m_axi_rresp, m_axi_rid);
            if (m_axi_rlast != (r_word[2:0] == 3'd7))
               $fatal(1, "vga_scanout: rlast=%b on beat %0d of an 8-beat burst", m_axi_rlast, r_word[2:0]);
            lb_we    <= 1'b1;
            lb_waddr <= lp[PB-1:0];
            lb_wdata <= {r_line == 12'd0 && r_word == 11'd0, m_axi_rdata};
            lp       <= lp + 1'b1;
            if (r_word == beats - 11'd1) begin
               r_word <= 11'd0;
               r_line <= (r_line == v_active - 12'd1) ? 12'd0 : r_line + 12'd1;
            end else r_word <= r_word + 11'd1;
         end
         // disabled: start over at the frame's first line once nothing is in flight
         if (!en & ~ar_v & (f_out == 3'd0)) begin
            qp <= {(PB+1){1'b0}};  lp <= {(PB+1){1'b0}};  f_line <= 12'd0;  f_bur <= 8'd0;  f_addr <= fb_base;
            r_line <= 12'd0;  r_word <= 11'd0;
         end
      end
   end

   assign m_axi_arvalid = ar_v;
`include "dram_window.vh"
   assign m_axi_araddr  = dram_axi({32'd0, f_addr});
   assign m_axi_arlen   = 8'd7;       // 8 beats = 64 bytes; never crosses 4 KiB (64-aligned)
   assign m_axi_arsize  = 3'b011;
   assign m_axi_arburst = 2'b01;
   assign m_axi_arid    = 3'd0;
   assign m_axi_arlock  = 1'b0;
   assign m_axi_arcache = 4'b0011;
   assign m_axi_arprot  = 3'b000;
   assign m_axi_arqos   = 4'd0;
   assign m_axi_rready  = 1'b1;     // a beat always has its place: its room was taken at the asking

   always @(posedge clk)
      if (!reset && ar_v && !dram_ok({32'd0, f_addr}))
         $fatal(1, "vga_scanout: burst address %h is outside DRAM", f_addr);
endmodule

`default_nettype wire
