`timescale 1ns / 1ps
`default_nettype none
// tb_vga_scanout -- vga_scanout against an independent model of the frame.
//
// An AXI memory with random AR and R stalls holds a framebuffer whose every pixel is a known
// function of its (x, y). The bench programs a small mode through the MMIO registers, finds the
// first VSYNC edge, and from there predicts every output cycle -- syncs, blanking, and the RGB222
// of each pixel -- for several frames, in two geometries (one burst per line, then three).
// Any difference, any late line, or a frame that never syncs is a FAIL.
module tb_vga_scanout;
   reg clk = 1'b0, pix_clk = 1'b0, reset = 1'b1;
   always #1.5  clk = ~clk;        // 333 MHz, as ui_clk
   always #20 pix_clk = ~pix_clk;    // 25 MHz

   reg  [ 7:0] mmio_addr = 8'd0;  reg mmio_write = 1'b0;  reg [31:0] mmio_wdata = 32'd0;
   wire [31:0] mmio_rdata;
   wire [ 2:0] arid, arsize, arprot;  wire [30:0] araddr;  wire [7:0] arlen;  wire [1:0] arburst;
   wire        arlock, arvalid, rready;  wire [3:0] arcache, arqos;
   reg         arready = 1'b0, rvalid = 1'b0, rlast = 1'b0;  reg [63:0] rdata = 64'd0;
   wire        hs, vs;  wire [1:0] r, g, b;
   wire        pixclk_rst, drp_req, drp_we;  wire [6:0] drp_addr;  wire [15:0] drp_di;

   vga_scanout dut (
      .clk(clk), .reset(reset),
      .mmio_addr(mmio_addr), .mmio_write(mmio_write), .mmio_wdata(mmio_wdata), .mmio_be(4'hf),
      .mmio_rdata(mmio_rdata),
      .m_axi_arid(arid), .m_axi_araddr(araddr), .m_axi_arlen(arlen), .m_axi_arsize(arsize),
      .m_axi_arburst(arburst), .m_axi_arlock(arlock), .m_axi_arcache(arcache), .m_axi_arprot(arprot),
      .m_axi_arqos(arqos), .m_axi_arvalid(arvalid), .m_axi_arready(arready),
      .m_axi_rid(3'd0), .m_axi_rdata(rdata), .m_axi_rresp(2'b00), .m_axi_rlast(rlast),
      .m_axi_rvalid(rvalid), .m_axi_rready(rready),
      .pixclk_rst(pixclk_rst), .pixclk_locked(1'b1),
      .drp_req(drp_req), .drp_we(drp_we), .drp_addr(drp_addr), .drp_di(drp_di),
      .drp_ack(drp_req), .drp_do(16'hBEEF),          // an instant DRP: ack follows req
      .pix_clk(pix_clk), .pix_rst(1'b0),
      .vga_hs(hs), .vga_vs(vs), .vga_r(r), .vga_g(g), .vga_b(b));

   // ---- the framebuffer, as a function: pixel (x, y) ----
   localparam [31:0] BASE = 32'h8000_1000;
   reg [31:0] stride;
   function [15:0] pixel(input integer x, input integer y);
      pixel = 16'(x * 7 + y * 131 + 16'h1234);
   endfunction
   function [63:0] beat(input [31:0] a);   // the 8 bytes at physical address a
      integer i, off;
      begin
         off = a - BASE;
         for (i = 0; i < 4; i = i + 1)
            beat[16*i +: 16] = pixel((off % stride) / 2 + i, off / stride);
      end
   endfunction

   // ---- AXI memory: random AR acceptance, random gaps between R beats ----
   reg [31:0] seed = 32'd1;
   function rnd(input integer pct);
      begin seed = seed * 1103515245 + 12345;  rnd = (seed[30:16] % 100) < pct; end
   endfunction
   reg [31:0] burst_addr;  integer beats_left = 0;
   always @(posedge clk) begin
      if (arvalid && arready) begin
         if (arlen != 8'd7 || arsize != 3'b011 || arburst != 2'b01 || araddr[5:0] != 6'd0 || arid != 3'd0)
            $fatal(1, "AR: len=%0d size=%0d burst=%0d addr=%h id=%0d", arlen, arsize, arburst, araddr, arid);
         burst_addr <= {1'b1, araddr};  beats_left <= 8;
      end
      arready <= beats_left == 0 && !(arvalid && arready) && rnd(30);
      if (rvalid && rready) begin
         burst_addr <= burst_addr + 32'd8;  beats_left <= beats_left - 1;
      end
      if (beats_left != 0 && !(rvalid && rready && beats_left == 1) && rnd(60)) begin
         rvalid <= 1'b1;
         rdata  <= beat((rvalid && rready) ? burst_addr + 32'd8 : burst_addr);
         rlast  <= ((rvalid && rready) ? beats_left - 1 : beats_left) == 1;
      end else if (rvalid && rready || beats_left == 0)
         rvalid <= 1'b0;
   end

   task wr(input [7:0] a, input [31:0] d);
      begin
         @(posedge clk); mmio_addr <= a; mmio_wdata <= d; mmio_write <= 1'b1;
         @(posedge clk); mmio_write <= 1'b0;
      end
   endtask

   // ---- the mode, and the check ----
   integer HA, HSS, HSE, HT, VA, VSS, VSE, VT;
   integer mh, mv, synced = 0, armed = 0, frames = 0, errors = 0;
   reg     vs_d = 1'b0;
   always @(posedge pix_clk) begin
      vs_d <= vs;
      if (armed && !synced && vs && !vs_d) begin
         // VSYNC rises with the line v = VSS, h = 0 (positive polarity): lock the model there.
         synced = 1;  mh = 0;  mv = VSS;
      end
      if (armed && synced) begin : check
         reg exp_hs, exp_vs, act;  reg [15:0] p;  reg [5:0] exp_rgb;
         exp_hs = mh >= HSS && mh < HSE;
         exp_vs = mv >= VSS && mv < VSE;
         act    = mh < HA && mv < VA;
         p      = pixel(mh, mv);
         exp_rgb = act ? {p[15:14], p[10:9], p[4:3]} : 6'd0;
         // The first frame after the edge may show lines fetched before the mode settled.
         if (frames > 0 && (hs !== exp_hs || vs !== exp_vs || {r, g, b} !== exp_rgb)) begin
            if (errors < 10)
               $display("FAIL: frame %0d (x=%0d, y=%0d): hs=%b vs=%b rgb=%b, expected %b %b %b",
                        frames, mh, mv, hs, vs, {r, g, b}, exp_hs, exp_vs, exp_rgb);
            errors = errors + 1;
         end
         mh = mh + 1;
         if (mh == HT) begin
            mh = 0;  mv = mv + 1;
            if (mv == VT) begin mv = 0;  frames = frames + 1; end
         end
      end
   end

   task run_mode(input integer ha, hss, hse, ht, va, vss, vse, vt, input [31:0] st);
      begin
         HA = ha; HSS = hss; HSE = hse; HT = ht; VA = va; VSS = vss; VSE = vse; VT = vt;
         stride = st;  armed = 0;            // the model sits out the mode change
         wr(8'h00, 32'd0);                   // disable, then program
         repeat (200) @(posedge clk);
         wr(8'h04, BASE);  wr(8'h08, st);
         wr(8'h10, ha);  wr(8'h14, hss);  wr(8'h18, hse);  wr(8'h1C, ht);
         wr(8'h20, va);  wr(8'h24, vss);  wr(8'h28, vse);  wr(8'h2C, vt);
         wr(8'h00, 32'd7);                   // enable, both syncs active-high
         // Arm the model only now: CTRL=0 above also made both syncs active-low, so their idle
         // level flipped, and that flip is not the frame's VSYNC edge.
         repeat (4) @(posedge pix_clk);
         frames = 0;  synced = 0;  armed = 1;
         wait (frames == 4);
         @(posedge clk); mmio_addr <= 8'h30;  @(posedge clk);
         if (mmio_rdata[31:16] != 16'd0) begin
            $display("FAIL: %0d lines fetched late", mmio_rdata[31:16]);  errors = errors + 1;
         end
         $display("mode %0dx%0d stride %0d: 3 frames checked, %0d errors", ha, va, st, errors);
      end
   endtask

   initial begin
      repeat (10) @(posedge clk);
      reset = 1'b0;
      if (dut.h_total != 12'd800 || dut.v_total != 12'd525 || dut.fb_base != 32'hFFF0_0000
          || dut.hs_pol || dut.vs_pol)
         $fatal(1, "reset values are not 640x480@60 at 0xFFF00000");
      // One burst per line, then three; odd porches so an off-by-one shows.
      run_mode(32, 35, 39, 44, 6, 7, 9, 11, 32'd64);
      run_mode(96, 101, 110, 117, 5, 6, 8, 9, 32'd192);
      // DRP: one write, one read; the instant ack loops 0xBEEF back.
      wr(8'h40, 32'h8008_1234);
      repeat (10) @(posedge clk);
      if (drp_req) begin $display("FAIL: DRP request never completed"); errors = errors + 1; end
      @(posedge clk); mmio_addr <= 8'h40;  @(posedge clk);
      if (mmio_rdata != 32'h0000_BEEF) begin $display("FAIL: DRP read %h", mmio_rdata); errors = errors + 1; end
      if (errors == 0) $display("PASS");
      else             $display("FAIL: %0d errors", errors);
      $finish;
   end

`ifdef TRACE
   integer tn = 0;
   always @(posedge clk) if (tn < 40) begin
      if (dut.fs == 3'd1) begin $display("t=%0t fetch line %0d (v=%0d h=%0d)", $time, dut.f_line, dut.v, dut.h); tn = tn + 1; end
      if (dut.lb_we && dut.lb_waddr[8:0] < 2) begin $display("t=%0t   lb[%0d] <= %h", $time, dut.lb_waddr, dut.lb_wdata); tn = tn + 1; end
   end
`endif

   initial begin #20_000_000; $display("FAIL: timeout"); $finish; end
endmodule

`default_nettype wire
