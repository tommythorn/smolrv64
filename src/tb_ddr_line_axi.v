`timescale 1ns/1ps
`default_nettype none

// Unit test for ddr_line_axi: drive the 512-bit line port with writes/reads and
// check they land in / come back from a small behavioral 64-bit AXI memory slave.
// Exercises burst assembly (8x64b <-> 512b), back-to-back ops, and ready stalls.
module tb;
   reg clk=0; always #5 clk=~clk;
   reg reset;

   // line side
   reg          ddr_req, ddr_we;
   reg  [57:0]  ddr_addr;
   reg  [511:0] ddr_wdata;
   wire [511:0] ddr_rdata;
   wire         ddr_ack;

   // AXI nets
   wire [2:0]  awid;  wire [30:0] awaddr; wire [7:0] awlen; wire [2:0] awsize;
   wire [1:0]  awburst; wire awlock; wire [3:0] awcache; wire [2:0] awprot;
   wire [3:0]  awqos; wire awvalid; wire awready;
   wire [63:0] wdata; wire [7:0] wstrb; wire wlast; wire wvalid; wire wready;
   wire [2:0]  bid; wire [1:0] bresp; wire bvalid; wire bready;
   wire [2:0]  arid; wire [30:0] araddr; wire [7:0] arlen; wire [2:0] arsize;
   wire [1:0]  arburst; wire arlock; wire [3:0] arcache; wire [2:0] arprot;
   wire [3:0]  arqos; wire arvalid; wire arready;
   wire [2:0]  rid; wire [63:0] rdata; wire [1:0] rresp; wire rlast; wire rvalid; wire rready;

   ddr_line_axi dut (
      .clk(clk), .reset(reset),
      .ddr_req(ddr_req), .ddr_we(ddr_we), .ddr_addr(ddr_addr),
      .ddr_wdata(ddr_wdata), .ddr_rdata(ddr_rdata), .ddr_ack(ddr_ack),
      .m_axi_awid(awid), .m_axi_awaddr(awaddr), .m_axi_awlen(awlen), .m_axi_awsize(awsize),
      .m_axi_awburst(awburst), .m_axi_awlock(awlock), .m_axi_awcache(awcache),
      .m_axi_awprot(awprot), .m_axi_awqos(awqos), .m_axi_awvalid(awvalid), .m_axi_awready(awready),
      .m_axi_wdata(wdata), .m_axi_wstrb(wstrb), .m_axi_wlast(wlast), .m_axi_wvalid(wvalid), .m_axi_wready(wready),
      .m_axi_bid(bid), .m_axi_bresp(bresp), .m_axi_bvalid(bvalid), .m_axi_bready(bready),
      .m_axi_arid(arid), .m_axi_araddr(araddr), .m_axi_arlen(arlen), .m_axi_arsize(arsize),
      .m_axi_arburst(arburst), .m_axi_arlock(arlock), .m_axi_arcache(arcache),
      .m_axi_arprot(arprot), .m_axi_arqos(arqos), .m_axi_arvalid(arvalid), .m_axi_arready(arready),
      .m_axi_rid(rid), .m_axi_rdata(rdata), .m_axi_rresp(rresp), .m_axi_rlast(rlast),
      .m_axi_rvalid(rvalid), .m_axi_rready(rready));

   // ---- behavioral 64-bit AXI memory slave (INCR bursts, a few cycles latency) ----
   localparam NW = 4096;               // 64-bit words
   reg [63:0] mem [0:NW-1];
   integer i;

   // write FSM
   reg [1:0]  ws;  reg [30:0] waddr; reg [7:0] wcnt; reg [3:0] wdelay;
   reg        awready_r, wready_r, bvalid_r;
   assign awready = awready_r; assign wready = wready_r;
   assign bvalid = bvalid_r; assign bresp = 2'b00; assign bid = 3'd0;
   always @(posedge clk) if (reset) begin
         ws<=0; awready_r<=1'b1; wready_r<=1'b0; bvalid_r<=1'b0; wdelay<=0;
      end else begin
         case (ws)
           0: begin awready_r<=1'b1; bvalid_r<=1'b0;
                 if (awvalid && awready_r) begin
                    waddr<=awaddr; wcnt<=0; awready_r<=1'b0; wready_r<=1'b1; ws<=1; end end
           1: if (wvalid && wready_r) begin
                 mem[((waddr>>3)+wcnt)&(NW-1)] <= wdata;
                 wcnt<=wcnt+1'b1;
                 if (wlast) begin wready_r<=1'b0; wdelay<=4'd3; ws<=2; end end
           2: if (wdelay==0) begin bvalid_r<=1'b1; ws<=3; end else wdelay<=wdelay-1'b1;
           3: if (bready) begin bvalid_r<=1'b0; awready_r<=1'b1; ws<=0; end
         endcase
      end

   // read FSM
   reg [1:0]  rs; reg [30:0] raddr; reg [7:0] rcnt; reg [3:0] rdelay;
   reg        arready_r, rvalid_r, rlast_r; reg [63:0] rdata_r;
   assign arready = arready_r; assign rvalid = rvalid_r; assign rlast = rlast_r;
   assign rdata = rdata_r; assign rresp = 2'b00; assign rid = 3'd0;
   always @(posedge clk) if (reset) begin
         rs<=0; arready_r<=1'b1; rvalid_r<=1'b0; rlast_r<=1'b0; rdelay<=0;
      end else begin
         case (rs)
           0: begin arready_r<=1'b1; rvalid_r<=1'b0; rlast_r<=1'b0;
                 if (arvalid && arready_r) begin
                    raddr<=araddr; rcnt<=0; arready_r<=1'b0; rdelay<=4'd3; rs<=1; end end
           1: if (rdelay==0) begin
                 rvalid_r<=1'b1; rdata_r<=mem[((raddr>>3)+rcnt)&(NW-1)]; rlast_r<=(rcnt==arlen); rs<=2;
              end else rdelay<=rdelay-1'b1;
           2: if (rvalid_r && rready) begin
                 if (rlast_r) begin rvalid_r<=1'b0; rlast_r<=1'b0; arready_r<=1'b1; rs<=0; end
                 else begin rcnt<=rcnt+1'b1; rdata_r<=mem[((raddr>>3)+rcnt+1)&(NW-1)]; rlast_r<=(rcnt+1==arlen); end
              end
         endcase
      end

   // ---- driver ----
   task do_write; input [57:0] la; input [511:0] d; begin
      @(negedge clk); ddr_req<=1'b1; ddr_we<=1'b1; ddr_addr<=la; ddr_wdata<=d;
      @(negedge clk); ddr_req<=1'b0; ddr_we<=1'b0;
      wait (ddr_ack); @(negedge clk);
   end endtask
   reg [511:0] got;
   task do_read; input [57:0] la; begin
      @(negedge clk); ddr_req<=1'b1; ddr_we<=1'b0; ddr_addr<=la;
      @(negedge clk); ddr_req<=1'b0;
      wait (ddr_ack); got = ddr_rdata; @(negedge clk);
   end endtask

   integer errors=0;
   task chk; input [511:0] a; input [511:0] b; input [127:0] msg; begin
      if (a!==b) begin errors=errors+1;
         $display("FAIL %0s: got %h exp %h", msg, a, b); end
   end endtask

   reg [511:0] p0, p1, p2;
   initial begin
      ddr_req=0; ddr_we=0; ddr_addr=0; ddr_wdata=0;
      reset=1; repeat(4) @(negedge clk); reset=0; @(negedge clk);

      // distinct byte pattern per 64-bit lane so beat-ordering bugs show
      p0 = {64'hF0F0F0F0_00000007, 64'h11111111_00000006, 64'h22222222_00000005,
            64'h33333333_00000004, 64'h44444444_00000003, 64'h55555555_00000002,
            64'h66666666_00000001, 64'h77777777_00000000};
      p1 = {8{64'hDEADBEEF_CAFEBABE}};
      p2 = 512'h0;
      for (i=0;i<8;i=i+1) p2[i*64 +: 64] = (64'hA5 << (i*8)) | i;

      do_write(58'h10, p0);          // line 0x10 -> byte 0x400
      do_write(58'h11, p1);          // adjacent line
      do_write(58'h2000, p2);        // far line

      do_read(58'h10); chk(got, p0, "rd p0");
      do_read(58'h2000); chk(got, p2, "rd p2");
      do_read(58'h11); chk(got, p1, "rd p1");

      // overwrite + read-back (no stale line)
      do_write(58'h10, p1); do_read(58'h10); chk(got, p1, "rd overwrite");

      if (errors==0) $display("tb_ddr_line_axi: PASS");
      else           $display("tb_ddr_line_axi: FAIL (%0d errors)", errors);
      $finish;
   end

   initial begin #100000; $display("TIMEOUT"); $finish; end
endmodule

`default_nettype wire
