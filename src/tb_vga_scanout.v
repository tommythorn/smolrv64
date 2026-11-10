`timescale 1ns / 1ps
`default_nettype none
// tb_vga_scanout -- vga_scanout against an independent model of the frame.
//
// An AXI memory with random AR and R stalls holds a framebuffer whose every pixel is a known
// function of its (x, y). The bench programs a small mode through the MMIO registers, finds the
// first VSYNC edge, and from there predicts every output cycle -- syncs, blanking, and the RGB222
// of each pixel -- for several frames, in several geometries (one burst per line, three, a
// stride wider than the picture, a line of 2560 pixels). Any difference, any frame the scan ran
// not shown whole, or a frame that never syncs is a FAIL.
module tb_vga_scanout;
   reg clk = 1'b0, pix_clk = 1'b0, reset = 1'b1;
   always #1.5  clk = ~clk;        // 333 MHz, as ui_clk
   always #20 pix_clk = ~pix_clk;    // 25 MHz

   reg  [ 7:0] mmio_addr = 8'd0;  reg mmio_write = 1'b0;  reg [31:0] mmio_wdata = 32'd0;
   wire [31:0] mmio_rdata;
   wire [ 2:0] arid, arsize, arprot;  wire [31:0] araddr;  wire [7:0] arlen;  wire [1:0] arburst;
   wire        arlock, arvalid, rready;  wire [3:0] arcache, arqos;
   reg         arready = 1'b0, rvalid = 1'b0, rlast = 1'b0;  reg [63:0] rdata = 64'd0;
   wire        de, hs, vs;  wire [15:0] rgb;
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
      .pix_clk(pix_clk),
      .vid_de(de), .vid_hs(hs), .vid_vs(vs), .vid_rgb(rgb));

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
   reg        stall = 1'b0;                 // the memory takes no request (run_stall)
   always @(posedge clk) begin
      if (arvalid && arready) begin
         if (arlen != 8'd7 || arsize != 3'b011 || arburst != 2'b01 || araddr[5:0] != 6'd0 || arid != 3'd0)
            $fatal(1, "AR: len=%0d size=%0d burst=%0d addr=%h id=%0d", arlen, arsize, arburst, araddr, arid);
         burst_addr <= araddr ^ 32'h8000_0000;  beats_left <= 8;
      end
      arready <= !stall && beats_left == 0 && !(arvalid && arready) && rnd(30);
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
   integer k, dry = 0, mh, mv, synced = 0, armed = 0, frames = 0, errors = 0, checking = 1;
   reg     vs_d = 1'b0;
   always @(posedge pix_clk) begin
      vs_d <= vs;
      if (armed && !synced && vs && !vs_d) begin
         // VSYNC rises with HSYNC's leading edge on the line v = VSS (positive polarity): lock
         // the model there.
         synced = 1;  mh = HSS;  mv = VSS;
      end
      if (armed && synced) begin : check
         reg exp_hs, exp_vs, act;  reg [15:0] p;  reg [15:0] exp_rgb;  integer hl;
         exp_hs = mh >= HSS && mh < HSE;
         hl     = mh >= HSS ? mv : (mv == 0 ? VT - 1 : mv - 1);   // the line, counted from HSYNC
         exp_vs = hl >= VSS && hl < VSE;
         act    = mh < HA && mv < VA;
         p      = pixel(mh, mv);
         exp_rgb = act ? p : 16'd0;
         // The first frame after the edge may show lines fetched before the mode settled.
         if (checking && frames > 0 && (hs !== exp_hs || vs !== exp_vs || de !== act || rgb !== exp_rgb)) begin
            if (errors < 10)
               $display("FAIL: frame %0d (x=%0d, y=%0d): hs=%b vs=%b de=%b rgb=%h, expected %b %b %b %h",
                        frames, mh, mv, hs, vs, de, rgb, exp_hs, exp_vs, act, exp_rgb);
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
            $display("FAIL: %0d frames not shown whole", mmio_rdata[31:16]);  errors = errors + 1;
         end
         $display("mode %0dx%0d stride %0d: 3 frames checked, %0d errors", ha, va, st, errors);
      end
   endtask

   initial begin
      repeat (10) @(posedge clk);
      reset = 1'b0;
      if (dut.h_active != 12'd640 || dut.h_total != 12'd800 || dut.v_active != 12'd480 || dut.v_total != 12'd525
          || dut.stride != 14'd1280 || dut.fb_base != 32'hFF00_0000 || dut.hs_pol || dut.vs_pol)
         $fatal(1, "reset values are not 640x480@60 at 0xFF000000");
      // One burst per line, then three, then a stride wider than the picture (its last 32 pixels
      // taken in the blanking), then a line wider than 2048; odd porches so an off-by-one shows.
      run_mode(32, 35, 39, 44, 6, 7, 9, 11, 32'd64);
      run_mode(96, 101, 110, 117, 5, 6, 8, 9, 32'd192);
      run_mode(96, 101, 110, 140, 5, 6, 8, 9, 32'd256);
      run_mode(2560, 2565, 2570, 2600, 3, 4, 5, 6, 32'd5120);
      // The memory refuses every request for 200 to 450 line times, ending each time at another
      // point of the 9-line frame: the FIFO shows 17 frames (2048 beats, 120 a frame), then the
      // scan runs dry (black, each frame counted, not checked). Once the memory answers, the scan
      // throws away what is left of the frame the fetcher was in, holds the next frame's first
      // beat until a vertical blank, and every frame is the frame again.
      run_mode(96, 101, 110, 117, 5, 6, 8, 9, 32'd192);
      for (k = 0; k < 6; k = k + 1) begin
         checking = 0;  stall = 1'b1;
         repeat (200 * 117 + k * 4931) @(posedge pix_clk);
         stall = 1'b0;
         repeat (40 * 117) @(posedge pix_clk);
         @(posedge clk); mmio_addr <= 8'h30;  @(posedge clk);
         if (mmio_rdata[31:16] - dry < 5) begin
            $display("FAIL: stall %0d: %0d dry frames counted", k, mmio_rdata[31:16] - dry);  errors = errors + 1;
         end
         dry = mmio_rdata[31:16];
         frames = 0;  checking = 1;
         wait (frames == 4);
         @(posedge clk); mmio_addr <= 8'h30;  @(posedge clk);
         if (mmio_rdata[31:16] != dry) begin
            $display("FAIL: %0d frames not shown whole after stall %0d", mmio_rdata[31:16] - dry, k);  errors = errors + 1;
         end
      end
      $display("after six stalls (%0d frames not shown whole): 3 frames checked after each, %0d errors", dry, errors);
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

   initial begin #60_000_000; $display("FAIL: timeout"); $finish; end
endmodule

`default_nettype wire
