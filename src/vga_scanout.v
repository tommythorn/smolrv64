`timescale 1ns / 1ps
`default_nettype none
// vga_scanout -- the display side of simmerv's `--graphics WxH`: scans an RGB565 framebuffer in
// DRAM out to a VGA connector through a programmable timing generator.
//
// The framebuffer is ordinary RAM that Linux's simplefb draws into (docs/PLAN-2026-10-03-
// graphics-console.md); this block only reads it. Two clock domains:
//
//   clk (ui_clk)  the MMIO registers, and the line fetcher: an AXI read master that copies one
//                 scanline of the framebuffer into a two-line buffer, 64-byte bursts.
//   pix_clk       the timing generator. At the start of each line's horizontal blank it asks
//                 for the NEXT displayed line (line 0 on the last line of the frame), so a whole
//                 line time -- tens of microseconds -- covers a fetch of a few dozen bursts.
//
// The request crosses as {line, toggle}: the line number is written in the same cycle as the
// toggle flips and then held for a whole line, so it is long stable when the synchronized toggle
// reaches clk. The timing registers cross the same way, quasi-statically: software changes
// them only with CTRL.enable clear (XDC: false paths into pix_clk).
//
// Registers (32-bit, full-word writes only), offsets within the 256-byte page:
//   0x00 CTRL         [0] enable  [1] HSYNC active-high  [2] VSYNC active-high
//   0x04 FB_BASE      physical address of pixel (0,0); 64-byte aligned
//   0x08 STRIDE       bytes per line; a multiple of 64, at most 4096
//   0x10 H_ACTIVE     0x14 H_SYNC_START  0x18 H_SYNC_END  0x1C H_TOTAL   (pixels)
//   0x20 V_ACTIVE     0x24 V_SYNC_START  0x28 V_SYNC_END  0x2C V_TOTAL   (lines)
//   0x30 STATUS (ro)  [0] pixel clock locked  [1] DRP busy  [31:16] lines fetched late
//   0x40 DRP          write {we[31], addr[22:16], data[15:0]} starts one access of the pixel
//                     clock's MMCM; read gives {busy[31], last read data[15:0]}
//   0x44 PIXCLK_CTRL  [0] hold the pixel-clock MMCM in reset
// Reset values are VESA 800x600@60 (40 MHz) at FB_BASE 0xFFF0_0000, the address simmerv gives
// an 800x600 framebuffer in 2 GiB; the board's MMCM comes up at 40 MHz.
//
// Pixels are RGB565, little-endian, four to a 64-bit beat; the output is the top two bits of
// each channel (RGB222, for the TinyVGA adapter).
module vga_scanout (
    input  wire        clk,
    input  wire        reset,

    input  wire [ 7:0] mmio_addr,
    input  wire        mmio_write,
    input  wire [31:0] mmio_wdata,
    input  wire [ 3:0] mmio_be,
    output reg  [31:0] mmio_rdata,

    output wire [ 2:0] m_axi_arid,
    output wire [30:0] m_axi_araddr,
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
    input  wire        pix_rst,
    output reg         vga_hs,
    output reg         vga_vs,
    output reg  [ 1:0] vga_r,
    output reg  [ 1:0] vga_g,
    output reg  [ 1:0] vga_b
);
   // ================================ registers (clk) ================================
   reg        en, hs_pol, vs_pol;
   reg [31:0] fb_base;
   reg [12:0] stride;
   reg [11:0] h_active, h_sync_start, h_sync_end, h_total;
   reg [11:0] v_active, v_sync_start, v_sync_end, v_total;
   reg [15:0] late_lines;
   reg [15:0] drp_rdata;

   (* async_reg = "true" *) reg [1:0] locked_s, ack_s;
   always @(posedge clk) begin locked_s <= {locked_s[0], pixclk_locked}; ack_s <= {ack_s[0], drp_ack}; end
   wire drp_busy = drp_req | ack_s[1];

   always @(posedge clk) begin
      if (reset) begin
         en <= 1'b0;  hs_pol <= 1'b1;  vs_pol <= 1'b1;
         fb_base <= 32'hFFF0_0000;  stride <= 13'd1600;
         h_active <= 12'd800;  h_sync_start <= 12'd840;  h_sync_end <= 12'd968;  h_total <= 12'd1056;
         v_active <= 12'd600;  v_sync_start <= 12'd601;  v_sync_end <= 12'd605;  v_total <= 12'd628;
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
         6'h0c: mmio_rdata = {late_lines, 14'd0, drp_busy, locked_s[1]};
         6'h10: mmio_rdata = {drp_busy, 15'd0, drp_rdata};
         6'h11: mmio_rdata = {31'd0, pixclk_rst};
         default: mmio_rdata = 32'd0;
      endcase
   end

   // ======================= line buffer: two lines of 512 beats =======================
   reg [63:0] lb [0:1023];
   reg        lb_we;
   reg [ 9:0] lb_waddr;
   reg [63:0] lb_wdata;
   always @(posedge clk) if (lb_we) lb[lb_waddr] <= lb_wdata;

   // ================================ timing (pix_clk) ================================
   (* async_reg = "true" *) reg [1:0] en_p;
   reg [11:0] h, v;
   reg [11:0] req_line;
   reg        req_tog;
   always @(posedge pix_clk) en_p <= {en_p[0], en & ~pix_rst};

   wire        last_h   = h == h_total - 12'd1;
   wire        last_v   = v == v_total - 12'd1;
   wire [11:0] next_v   = last_v ? 12'd0 : v + 12'd1;
   always @(posedge pix_clk) begin
      if (!en_p[1]) begin
         h <= 12'd0;  v <= 12'd0;  req_line <= 12'd0;  req_tog <= 1'b0;
      end else begin
         h <= last_h ? 12'd0 : h + 12'd1;
         if (last_h) v <= next_v;
         // The line just shown is done with its half of the buffer: fetch the next one into it.
         if (h == h_active && next_v < v_active) begin req_line <= next_v;  req_tog <= ~req_tog; end
      end
   end

   // Two stages: the buffer read (with the syncs and blanking alongside), then the output
   // register. sel1 must be the same stage as lb_q: it picks a pixel out of the beat lb_q holds.
   reg        act1, hs1, vs1;  reg [1:0] sel1;  reg [63:0] lb_q;
   always @(posedge pix_clk) begin
      act1 <= en_p[1] && h < h_active && v < v_active;
      hs1  <= en_p[1] && h >= h_sync_start && h < h_sync_end;
      vs1  <= en_p[1] && v >= v_sync_start && v < v_sync_end;
      sel1 <= h[1:0];
      lb_q <= lb[{v[0], h[10:2]}];
   end
   wire [15:0] px = lb_q[{sel1, 4'd0} +: 16];
   always @(posedge pix_clk) begin
      vga_r  <= act1 ? px[15:14] : 2'd0;
      vga_g  <= act1 ? px[10: 9] : 2'd0;
      vga_b  <= act1 ? px[ 4: 3] : 2'd0;
      vga_hs <= hs1 ~^ hs_pol;     // active level when in the sync pulse
      vga_vs <= vs1 ~^ vs_pol;
   end

   // ================================ line fetch (clk) ================================
   (* async_reg = "true" *) reg [2:0] tog_s;
   always @(posedge clk) tog_s <= {tog_s[1:0], req_tog};
   wire req = tog_s[2] ^ tog_s[1];

   localparam [2:0] F_IDLE = 3'd0, F_MUL = 3'd1, F_ADD = 3'd2, F_AR = 3'd3, F_R = 3'd4;
   reg [ 2:0] fs;
   reg [11:0] f_line;
   reg [23:0] f_off;
   reg [31:0] f_addr;
   reg [ 9:0] f_word;       // beats of this line written so far
   reg [ 2:0] f_beat;

   always @(posedge clk) begin
      lb_we <= 1'b0;
      if (reset) begin
         fs <= F_IDLE;  late_lines <= 16'd0;
      end else begin
         // A request that finds the fetcher busy is a line that will be shown stale: the DDR
         // arbiter starved us for a whole line time. Counted, never silent (STATUS[31:16]).
         if (req && fs != F_IDLE) late_lines <= late_lines + 16'd1;
         case (fs)
            F_IDLE: if (req) begin f_line <= req_line;  fs <= F_MUL; end
            F_MUL:  begin f_off <= f_line * {11'd0, stride};  fs <= F_ADD; end
            F_ADD:  begin f_addr <= fb_base + {8'd0, f_off};  f_word <= 10'd0;  fs <= F_AR; end
            F_AR:   if (m_axi_arready) begin f_beat <= 3'd0;  fs <= F_R; end
            F_R:    if (m_axi_rvalid) begin
                       if (m_axi_rresp != 2'b00 || m_axi_rid != 3'd0)
                          $fatal(1, "vga_scanout: read response resp=%b id=%0d at %h", m_axi_rresp, m_axi_rid, f_addr);
                       if (m_axi_rlast != (f_beat == 3'd7))
                          $fatal(1, "vga_scanout: rlast=%b on beat %0d of an 8-beat burst", m_axi_rlast, f_beat);
                       lb_we    <= 1'b1;
                       lb_waddr <= {f_line[0], f_word[8:0]};
                       lb_wdata <= m_axi_rdata;
                       f_word   <= f_word + 10'd1;
                       f_beat   <= f_beat + 3'd1;
                       if (m_axi_rlast) begin
                          f_addr <= f_addr + 32'd64;
                          fs <= ({f_word + 10'd1, 3'd0} == stride) ? F_IDLE : F_AR;
                       end
                    end
            default: $fatal(1, "vga_scanout: fetch FSM in undefined state %0d", fs);
         endcase
      end
   end

   assign m_axi_arvalid = fs == F_AR;
   assign m_axi_araddr  = f_addr[30:0];
   assign m_axi_arlen   = 8'd7;       // 8 beats = 64 bytes; never crosses 4 KiB (64-aligned)
   assign m_axi_arsize  = 3'b011;
   assign m_axi_arburst = 2'b01;
   assign m_axi_arid    = 3'd0;
   assign m_axi_arlock  = 1'b0;
   assign m_axi_arcache = 4'b0011;
   assign m_axi_arprot  = 3'b000;
   assign m_axi_arqos   = 4'd0;
   assign m_axi_rready  = fs == F_R;

   always @(posedge clk)
      if (!reset && fs == F_AR && f_addr[31] == 1'b0)
         $fatal(1, "vga_scanout: line address %h is below DRAM", f_addr);
endmodule

`default_nettype wire
