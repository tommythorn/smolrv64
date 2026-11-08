`timescale 1ns / 1ps
`default_nettype none
// vga_scanout -- the display side of simmerv's `--graphics WxH`: scans an RGB565 framebuffer in
// DRAM out to a VGA connector through a programmable timing generator.
//
// The framebuffer is ordinary RAM that Linux's simplefb draws into (docs/PLAN-2026-10-03-
// graphics-console.md); this block only reads it. Two clock domains:
//
//   clk (ui_clk)  the MMIO registers, and the line fetcher: an AXI read master that reads the
//                 frame line after line into a ring of line slots, 64-byte bursts, several in
//                 flight, as far ahead of the scan as the ring is deep.
//   pix_clk       the timing generator, which scans the slot of the line it shows. It starts in
//                 the vertical blank, so the fetcher fills the ring before the first line.
//
// The two sides tell each other one event a line, as a toggle: a line shown (its slot is free),
// a line fetched whole, and a line scanned before it was whole (counted: STATUS[31:16]). The
// timing registers cross quasi-statically: software changes them only with CTRL.enable clear
// (XDC: false paths into pix_clk).
//
// THE SYNCS ARE FREE-RUNNING. The timing generator's counters, and the syncs and blanking they
// make, depend on their own state and CTRL.enable alone (a mode change clears it): nothing the
// memory side does, nor the pixel clock's lock, restarts them. A late line shows the slot's older
// pixels (counted); a lock loss is counted too (STATUS[15:8]), and the counters ride through it on
// whatever the clock does.
//
// Registers (32-bit, full-word writes only), offsets within the 256-byte page:
//   0x00 CTRL         [0] enable  [1] HSYNC active-high  [2] VSYNC active-high
//   0x04 FB_BASE      physical address of pixel (0,0); 64-byte aligned
//   0x08 STRIDE       bytes per line; a multiple of 64, at most 4096
//   0x10 H_ACTIVE     0x14 H_SYNC_START  0x18 H_SYNC_END  0x1C H_TOTAL   (pixels)
//   0x20 V_ACTIVE     0x24 V_SYNC_START  0x28 V_SYNC_END  0x2C V_TOTAL   (lines)
//   0x30 STATUS (ro)  [0] pixel clock locked  [1] DRP busy  [15:8] pixel clock lock losses while
//                     enabled (wraps)  [31:16] lines scanned before fetched whole
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
   reg [12:0] stride;
   reg [11:0] h_active, h_sync_start, h_sync_end, h_total;
   reg [11:0] v_active, v_sync_start, v_sync_end, v_total;
   reg [15:0] late_lines;
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
         fb_base <= 32'hFF00_0000;  stride <= 13'd1280;
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
               6'h02: stride       <= mmio_wdata[12:0];
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
            if (mmio_addr[7:2] == 6'h02 && (mmio_wdata[5:0] != 6'd0 || mmio_wdata[12:0] > 13'd4096 || mmio_wdata[31:13] != 0))
               $fatal(1, "vga_scanout: STRIDE %0d is not a multiple of 64 bytes up to 4096", mmio_wdata);
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
         6'h02: mmio_rdata = {19'd0, stride};
         6'h04: mmio_rdata = {20'd0, h_active};
         6'h05: mmio_rdata = {20'd0, h_sync_start};
         6'h06: mmio_rdata = {20'd0, h_sync_end};
         6'h07: mmio_rdata = {20'd0, h_total};
         6'h08: mmio_rdata = {20'd0, v_active};
         6'h09: mmio_rdata = {20'd0, v_sync_start};
         6'h0a: mmio_rdata = {20'd0, v_sync_end};
         6'h0b: mmio_rdata = {20'd0, v_total};
         6'h0c: mmio_rdata = {late_lines, lock_lost, 6'd0, drp_busy, locked_s[1]};
         6'h10: mmio_rdata = {drp_busy, 15'd0, drp_rdata};
         6'h11: mmio_rdata = {31'd0, pixclk_rst};
         default: mmio_rdata = 32'd0;
      endcase
   end

   // ===================== line buffer: a ring of NS line slots, 512 beats each ====================
   // Line s of the scan's sequence (every active line, frame after frame) lives in slot s % NS. The
   // fetcher fills a slot as soon as the scan has left the line before it there, so it runs up to
   // NS-1 lines ahead of the scan instead of racing it: a stall of DRAM costs nothing until it is
   // longer than NS-1 line times.
   localparam integer NS = 4, NSB = 2;
   // The line sequences both sides count (seq_p, cmp_p; done_c, f_seq, r_seq) are compared as
   // signed differences, so they are wide enough that no stall of the DMA path wraps one: an 8-bit
   // count took the scan 128 lines ahead (a 1920x1200 redraw's first frame) for the fetcher being
   // ahead, which then stopped until the difference wrapped and stayed 252 lines behind for good --
   // every frame rotated by a fifth. A wrap now needs the scan 32K lines ahead, half a second.
   localparam integer SQB = 16;
   reg [63:0] lb [0:NS*512-1];
   reg        lb_we;
   reg [NSB+8:0] lb_waddr;
   reg [63:0] lb_wdata;
   always @(posedge clk) if (lb_we) lb[lb_waddr] <= lb_wdata;

   // ================================ timing (pix_clk) ================================
   // Three toggles cross, one event a line (a line is microseconds, the clocks nanoseconds):
   // done_tog (the scan has shown a line: its slot is free), cmp_tog (the fetcher has written a
   // whole line) and late_tog (a line's scan began before its line was whole).
   (* async_reg = "true" *) reg [1:0] en_p;
   (* async_reg = "true" *) reg [2:0] cmp_s;
   reg [11:0] h, v;
   reg [SQB-1:0] seq_p;       // the active line being scanned, in sequence
   reg [SQB-1:0] cmp_p;       // the lines the fetcher has finished
   reg        done_tog, late_tog;
   always @(posedge pix_clk) begin en_p <= {en_p[0], en};  cmp_s <= {cmp_s[1:0], cmp_tog}; end

   wire        last_h   = h == h_total - 12'd1;
   wire        last_v   = v == v_total - 12'd1;
   wire        act_v    = v < v_active;
   always @(posedge pix_clk) begin
      if (!en_p[1]) begin
         // the scan starts in the vertical blank: the fetcher has the blank to fill the ring
         h <= 12'd0;  v <= v_active;  seq_p <= {SQB{1'b0}};  cmp_p <= {SQB{1'b0}};  done_tog <= 1'b0;  late_tog <= 1'b0;
      end else begin
         h <= last_h ? 12'd0 : h + 12'd1;
         if (last_h) v <= last_v ? 12'd0 : v + 12'd1;
         if (cmp_s[2] ^ cmp_s[1]) cmp_p <= cmp_p + 1'b1;
         // a line whose scan begins before the fetcher finished it shows the slot's older contents
         if (act_v && h == 12'd0 && !($signed(cmp_p - seq_p) > 0)) late_tog <= ~late_tog;
         // the line just shown frees its slot
         if (act_v && h == h_active) begin seq_p <= seq_p + 1'b1;  done_tog <= ~done_tog; end
      end
   end

   // Two stages: the buffer read (with the syncs and blanking alongside), then the output
   // register. sel1 must be the same stage as lb_q: it picks a pixel out of the beat lb_q holds.
   reg        act1, hs1, vs1;  reg [1:0] sel1;  reg [63:0] lb_q;
   always @(posedge pix_clk) begin
      act1 <= en_p[1] && h < h_active && act_v;
      hs1  <= en_p[1] && h >= h_sync_start && h < h_sync_end;
      vs1  <= en_p[1] && v >= v_sync_start && v < v_sync_end;
      sel1 <= h[1:0];
      lb_q <= lb[{seq_p[NSB-1:0], h[10:2]}];
   end
   wire [15:0] px = lb_q[{sel1, 4'd0} +: 16];
   always @(posedge pix_clk) begin
      vid_de  <= act1;
      vid_rgb <= act1 ? px : 16'd0;
      vid_hs  <= hs1 ~^ hs_pol;    // active level when in the sync pulse
      vid_vs  <= vs1 ~^ vs_pol;
   end

   // ================================ line fetch (clk) ================================
   // The fetcher reads the frame line after line, a burst at a time, up to NOUT bursts in flight
   // (one ID: they answer in order), wherever the line's slot is free (the scan is fewer than NS
   // lines behind it); the answers land in order at the receive pointer.
   localparam [2:0] NOUT = 3'd4;
   (* async_reg = "true" *) reg [2:0] done_s, late_s;
   (* async_reg = "true" *) reg [1:0] en_a;   // the scan is running: it counts the lines the fetcher finishes
   reg [SQB-1:0] done_c;      // the lines the scan has shown
   reg [SQB-1:0] f_seq;       // the line being asked for, in sequence
   reg [11:0] f_line;         // ...its line in the frame
   reg [ 9:0] f_bur;          // ...and the bursts of it asked for
   reg [31:0] f_addr;
   reg        ar_v;
   reg [ 2:0] f_out;          // bursts in flight
   reg [SQB-1:0] r_seq;       // the line the answers land in
   reg [ 8:0] r_word;         // ...the beat they land at
   reg [ 2:0] r_beat;
   reg        cmp_tog;
   wire [9:0] bursts = {3'd0, stride[12:6]};                           // a line's bursts
   wire [9:0] beats  = stride[12:3];                                   // ...and beats
   wire       line_last = f_bur == bursts - 10'd1;
   wire       ar_go = ar_v & m_axi_arready;
   wire       r_go  = m_axi_rvalid & m_axi_rready;
   wire       ask   = en & en_a[1] & ~ar_v & (f_out < NOUT) & ($signed(f_seq - done_c) < NS);
   always @(posedge clk) begin
      done_s <= {done_s[1:0], done_tog};  late_s <= {late_s[1:0], late_tog};  en_a <= {en_a[0], en_p[1]};
      lb_we <= 1'b0;
      if (reset) begin
         late_lines <= 16'd0;  ar_v <= 1'b0;  f_out <= 3'd0;  cmp_tog <= 1'b0;
         done_c <= {SQB{1'b0}};  f_seq <= {SQB{1'b0}};  f_line <= 12'd0;  f_bur <= 10'd0;  f_addr <= 32'd0;
         r_seq <= {SQB{1'b0}};  r_word <= 9'd0;  r_beat <= 3'd0;
      end else begin
         if (en_a[1] & (late_s[2] ^ late_s[1])) late_lines <= late_lines + 16'd1;   // counted, never silent (STATUS[31:16])
         if (done_s[2] ^ done_s[1]) done_c <= done_c + 1'b1;
         // asking: a burst at f_addr, then the next; the frame's last line wraps to its first
         if (ar_go) begin
            ar_v  <= 1'b0;
            f_bur <= line_last ? 10'd0 : f_bur + 10'd1;
            if (line_last) begin
               f_seq  <= f_seq + 1'b1;
               f_line <= (f_line == v_active - 12'd1) ? 12'd0 : f_line + 12'd1;
               f_addr <= (f_line == v_active - 12'd1) ? fb_base : f_addr + 32'd64;
            end else f_addr <= f_addr + 32'd64;
         end
         if (ask) ar_v <= 1'b1;
         f_out <= f_out + {2'd0, ar_go} - {2'd0, r_go & m_axi_rlast};
         // landing: in order, a beat at a time; a line's last beat finishes it
         if (r_go) begin
            if (m_axi_rresp != 2'b00 || m_axi_rid != 3'd0)
               $fatal(1, "vga_scanout: read response resp=%b id=%0d", m_axi_rresp, m_axi_rid);
            if (m_axi_rlast != (r_beat == 3'd7))
               $fatal(1, "vga_scanout: rlast=%b on beat %0d of an 8-beat burst", m_axi_rlast, r_beat);
            lb_we    <= 1'b1;
            lb_waddr <= {r_seq[NSB-1:0], r_word};
            lb_wdata <= m_axi_rdata;
            r_beat   <= r_beat + 3'd1;
            if (r_word == beats[8:0] - 9'd1) begin       // (a 512-beat line's last is 511)
               r_word <= 9'd0;  r_seq <= r_seq + 1'b1;  cmp_tog <= ~cmp_tog;
            end else r_word <= r_word + 9'd1;
         end
         // disabled: start over at the frame's first line once nothing is in flight
         if (!en & ~ar_v & (f_out == 3'd0)) begin
            done_c <= {SQB{1'b0}};  f_seq <= {SQB{1'b0}};  f_line <= 12'd0;  f_bur <= 10'd0;  f_addr <= fb_base;
            r_seq <= {SQB{1'b0}};  r_word <= 9'd0;  r_beat <= 3'd0;
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
   assign m_axi_rready  = 1'b1;     // a beat always has its place: its slot was free when it was asked for

   always @(posedge clk)
      if (!reset && ar_v && !dram_ok({32'd0, f_addr}))
         $fatal(1, "vga_scanout: line address %h is outside DRAM", f_addr);
endmodule

`default_nettype wire
