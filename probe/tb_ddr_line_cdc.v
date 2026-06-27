`timescale 1ns/1ps
`default_nettype none

// Unit test: probe-side(slow clk_p) -> ddr_line_cdc -> ddr_line_axi -> behavioral 64b
// AXI mem (fast clk_m). Verifies the line req/ack handshake + payload cross both clocks.
module tb;
   reg clk_p=0; always #20 clk_p=~clk_p;   // 25 MHz
   reg clk_m=0; always #5  clk_m=~clk_m;    // 100 MHz (4:1)
   reg reset;

   // probe side
   reg          p_req, p_we; reg [57:0] p_addr; reg [511:0] p_wdata;
   wire [511:0] p_rdata; wire p_ack;
   // cdc <-> bridge (clk_m)
   wire         m_req, m_we; wire [57:0] m_addr; wire [511:0] m_wdata;
   wire [511:0] m_rdata; wire m_ack;

   ddr_line_cdc u_cdc (
      .clk_p(clk_p), .reset_p(reset), .p_req(p_req), .p_we(p_we), .p_addr(p_addr),
      .p_wdata(p_wdata), .p_rdata(p_rdata), .p_ack(p_ack),
      .clk_m(clk_m), .reset_m(reset), .m_req(m_req), .m_we(m_we), .m_addr(m_addr),
      .m_wdata(m_wdata), .m_rdata(m_rdata), .m_ack(m_ack));

   // AXI nets
   wire [2:0] awid; wire [30:0] awaddr; wire [7:0] awlen; wire [2:0] awsize; wire [1:0] awburst;
   wire awlock; wire [3:0] awcache; wire [2:0] awprot; wire [3:0] awqos; wire awvalid, awready;
   wire [63:0] wdata; wire [7:0] wstrb; wire wlast, wvalid, wready;
   wire [2:0] bid; wire [1:0] bresp; wire bvalid, bready;
   wire [2:0] arid; wire [30:0] araddr; wire [7:0] arlen; wire [2:0] arsize; wire [1:0] arburst;
   wire arlock; wire [3:0] arcache; wire [2:0] arprot; wire [3:0] arqos; wire arvalid, arready;
   wire [2:0] rid; wire [63:0] rdata; wire [1:0] rresp; wire rlast, rvalid, rready;

   ddr_line_axi u_br (
      .clk(clk_m), .reset(reset),
      .ddr_req(m_req), .ddr_we(m_we), .ddr_addr(m_addr), .ddr_wdata(m_wdata),
      .ddr_rdata(m_rdata), .ddr_ack(m_ack),
      .m_axi_awid(awid), .m_axi_awaddr(awaddr), .m_axi_awlen(awlen), .m_axi_awsize(awsize),
      .m_axi_awburst(awburst), .m_axi_awlock(awlock), .m_axi_awcache(awcache), .m_axi_awprot(awprot),
      .m_axi_awqos(awqos), .m_axi_awvalid(awvalid), .m_axi_awready(awready),
      .m_axi_wdata(wdata), .m_axi_wstrb(wstrb), .m_axi_wlast(wlast), .m_axi_wvalid(wvalid), .m_axi_wready(wready),
      .m_axi_bid(bid), .m_axi_bresp(bresp), .m_axi_bvalid(bvalid), .m_axi_bready(bready),
      .m_axi_arid(arid), .m_axi_araddr(araddr), .m_axi_arlen(arlen), .m_axi_arsize(arsize),
      .m_axi_arburst(arburst), .m_axi_arlock(arlock), .m_axi_arcache(arcache), .m_axi_arprot(arprot),
      .m_axi_arqos(arqos), .m_axi_arvalid(arvalid), .m_axi_arready(arready),
      .m_axi_rid(rid), .m_axi_rdata(rdata), .m_axi_rresp(rresp), .m_axi_rlast(rlast),
      .m_axi_rvalid(rvalid), .m_axi_rready(rready));

   // behavioral 64b AXI mem (clk_m)
   localparam NW=4096; reg [63:0] mem [0:NW-1]; integer k;
   reg [1:0] ws; reg [30:0] wa; reg [7:0] wc; reg [3:0] wd; reg awr_r, wr_r, bv_r;
   assign awready=awr_r; assign wready=wr_r; assign bvalid=bv_r; assign bresp=0; assign bid=0;
   always @(posedge clk_m) if (reset) begin ws<=0;awr_r<=1;wr_r<=0;bv_r<=0;wd<=0; end else case(ws)
      0: begin awr_r<=1;bv_r<=0; if(awvalid&&awr_r) begin wa<=awaddr;wc<=0;awr_r<=0;wr_r<=1;ws<=1; end end
      1: if(wvalid&&wr_r) begin mem[((wa>>3)+wc)&(NW-1)]<=wdata; wc<=wc+1; if(wlast) begin wr_r<=0;wd<=3;ws<=2; end end
      2: if(wd==0) begin bv_r<=1;ws<=3; end else wd<=wd-1;
      3: if(bready) begin bv_r<=0;awr_r<=1;ws<=0; end
   endcase
   reg [1:0] rs; reg [30:0] ra; reg [7:0] rc; reg [3:0] rd_; reg ar_r, rv_r, rl_r; reg [63:0] rdr;
   assign arready=ar_r; assign rvalid=rv_r; assign rlast=rl_r; assign rdata=rdr; assign rresp=0; assign rid=0;
   always @(posedge clk_m) if (reset) begin rs<=0;ar_r<=1;rv_r<=0;rl_r<=0;rd_<=0; end else case(rs)
      0: begin ar_r<=1;rv_r<=0;rl_r<=0; if(arvalid&&ar_r) begin ra<=araddr;rc<=0;ar_r<=0;rd_<=3;rs<=1; end end
      1: if(rd_==0) begin rv_r<=1;rdr<=mem[((ra>>3)+rc)&(NW-1)];rl_r<=(rc==arlen);rs<=2; end else rd_<=rd_-1;
      2: if(rv_r&&rready) begin if(rl_r) begin rv_r<=0;rl_r<=0;ar_r<=1;rs<=0; end
            else begin rc<=rc+1;rdr<=mem[((ra>>3)+rc+1)&(NW-1)];rl_r<=(rc+1==arlen); end end
   endcase

   // probe-side driver (clk_p)
   task pwrite; input [57:0] la; input [511:0] d; begin
      @(negedge clk_p); p_req<=1;p_we<=1;p_addr<=la;p_wdata<=d;
      @(negedge clk_p); p_req<=0;p_we<=0; wait(p_ack); @(negedge clk_p); end endtask
   reg [511:0] got;
   task pread; input [57:0] la; begin
      @(negedge clk_p); p_req<=1;p_we<=0;p_addr<=la;
      @(negedge clk_p); p_req<=0; wait(p_ack); got=p_rdata; @(negedge clk_p); end endtask
   integer errs=0;
   task chk; input [511:0] a; input [511:0] b; input [127:0] m; begin
      if(a!==b) begin errs=errs+1; $display("FAIL %0s got %h exp %h",m,a,b); end end endtask

   reg [511:0] p0,p1;
   initial begin
      p_req=0;p_we=0;p_addr=0;p_wdata=0;
      reset=1; repeat(4) @(negedge clk_p); reset=0; @(negedge clk_p);
      p0=512'h0; for(k=0;k<8;k=k+1) p0[k*64+:64]=(64'h11*(k+1))<<(k*4) | k;
      p1={8{64'hA5A5_5A5A_DEAD_C0DE}};
      pwrite(58'h20,p0); pwrite(58'h21,p1);
      pread(58'h20); chk(got,p0,"rd p0");
      pread(58'h21); chk(got,p1,"rd p1");
      pwrite(58'h20,p1); pread(58'h20); chk(got,p1,"overwrite");
      if(errs==0) $display("tb_ddr_line_cdc: PASS"); else $display("tb_ddr_line_cdc: FAIL %0d",errs);
      $finish;
   end
   initial begin #200000; $display("TIMEOUT"); $finish; end
endmodule

`default_nettype wire
