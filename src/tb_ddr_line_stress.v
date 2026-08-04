`timescale 1ns/1ps
`default_nettype none

// Stress test for the FPGA-only line-port glue: soc_top's ddr_* -> ddr_line_cdc ->
// ddr_line_axi -> AXI memory. This path is NEVER exercised by any system simulation
// (tb_virtio/tb_cosim_linux model the DDR behaviorally at soc_top's port), and the
// existing tb_ddr_line_cdc only issues ONE transaction at a time, with waits, at a
// 4:1 clock ratio.
//
// The board runs clk_p = ui_clk/5, and since the D$ pipelining series the port sees
// BACK-TO-BACK traffic (write-back-buffer drain immediately followed by a fill, an
// MSHR fill overlapping a new read, I$ demand + prefetch). This TB reproduces that
// regime: the requester re-asserts as soon as it sees the ack, addresses/data are
// randomized, and every read is scoreboarded against a golden model. A CDC that
// acks a new request while the previous transaction's `done` level has not yet
// propagated back would return the PREVIOUS line's data -- caught here as a
// mismatch.
//
//   REQ_GAP=0 (default) : re-request in the same cycle the ack is seen (worst case)
//   +gap=N              : insert N idle clk_p cycles between transactions
//   +ratio=N            : clk_m:clk_p ratio (default 5, the board's divider)
//   +n=N                : transactions (default 20000)
module tb;
   integer ratio, ntrans, gap, seedv;
   initial begin
      if (!$value$plusargs("ratio=%d", ratio)) ratio = 5;
      if (!$value$plusargs("n=%d",     ntrans)) ntrans = 20000;
      if (!$value$plusargs("gap=%d",   gap))    gap = 0;
      if (!$value$plusargs("seed=%d",  seedv))  seedv = 1;
   end

   // clk_m free-running; clk_p is clk_m divided by `ratio` (BUFGCE_DIV-like: synchronous)
   reg clk_m = 0;  always #5 clk_m = ~clk_m;
   reg clk_p = 0;  integer divcnt = 0;
   always @(posedge clk_m) begin
      divcnt <= divcnt + 1;
      if (divcnt >= ratio-1) begin divcnt <= 0; clk_p <= ~clk_p; end
   end
   reg reset;

   reg          p_req, p_we;  reg [57:0] p_addr;  reg [511:0] p_wdata;
   wire [511:0] p_rdata;  wire p_ack;
   wire         m_req, m_we;  wire [57:0] m_addr;  wire [511:0] m_wdata;
   wire [511:0] m_rdata;  wire m_ack;

   ddr_line_cdc u_cdc (
      .clk_p(clk_p), .reset_p(reset), .p_req(p_req), .p_we(p_we), .p_addr(p_addr),
      .p_wdata(p_wdata), .p_rdata(p_rdata), .p_ack(p_ack),
      .clk_m(clk_m), .reset_m(reset), .m_req(m_req), .m_we(m_we), .m_addr(m_addr),
      .m_wdata(m_wdata), .m_rdata(m_rdata), .m_ack(m_ack));

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

   // ---- behavioral 64b AXI memory (clk_m), with randomized ready/latency jitter so
   //      the AXI side does not settle into one fixed rhythm ----
   localparam NW = 8192;  reg [63:0] mem [0:NW-1];
   reg [1:0] ws; reg [30:0] wa; reg [7:0] wc; reg [3:0] wd; reg awr_r, wr_r, bv_r;
   assign awready=awr_r; assign wready=wr_r; assign bvalid=bv_r; assign bresp=0; assign bid=0;
   always @(posedge clk_m) if (reset) begin ws<=0;awr_r<=1;wr_r<=0;bv_r<=0;wd<=0; end else case(ws)
      0: begin awr_r<=1;bv_r<=0; if(awvalid&&awr_r) begin wa<=awaddr;wc<=0;awr_r<=0;wr_r<=1;ws<=1; end end
      1: if(wvalid&&wr_r) begin mem[((wa>>3)+wc)&(NW-1)]<=wdata; wc<=wc+1; if(wlast) begin wr_r<=0;wd<=$random&3;ws<=2; end end
      2: if(wd==0) begin bv_r<=1;ws<=3; end else wd<=wd-1;
      3: if(bready) begin bv_r<=0;awr_r<=1;ws<=0; end
   endcase
   reg [1:0] rs; reg [30:0] ra; reg [7:0] rc; reg [3:0] rd_; reg ar_r, rv_r, rl_r; reg [63:0] rdr;
   assign arready=ar_r; assign rvalid=rv_r; assign rlast=rl_r; assign rdata=rdr; assign rresp=0; assign rid=0;
   always @(posedge clk_m) if (reset) begin rs<=0;ar_r<=1;rv_r<=0;rl_r<=0;rd_<=0; end else case(rs)
      0: begin ar_r<=1;rv_r<=0;rl_r<=0; if(arvalid&&ar_r) begin ra<=araddr;rc<=0;rd_<=$random&7;rs<=1; end end
      1: if(rd_==0) begin rv_r<=1;rdr<=mem[((ra>>3)+rc)&(NW-1)];rl_r<=(rc==arlen);rs<=2; end else rd_<=rd_-1;
      2: if(rv_r&&rready) begin if(rl_r) begin rv_r<=0;rl_r<=0;ar_r<=1;rs<=0; end
            else begin rc<=rc+1;rdr<=mem[((ra>>3)+rc+1)&(NW-1)];rl_r<=(rc+1==arlen); end end
   endcase

   // ---- golden model + stimulus ----
   localparam NL = 64;                       // distinct lines in play
   reg [511:0] gold [0:NL-1];
   reg [NL-1:0] gvalid;
   integer i, t, errs, nrd, nwr;
   reg [57:0] la; reg [511:0] dat; reg [5:0] li;

   // Re-request as soon as the ack is observed: this is the back-to-back regime the
   // pipelined D$/streaming I$ produce, and the one the old unit test never created.
   task do_write(input [5:0] lidx, input [511:0] d);
      begin
         @(posedge clk_p);
         p_req <= 1'b1; p_we <= 1'b1; p_addr <= {52'd0, lidx}; p_wdata <= d;
         @(posedge clk_p);
         p_req <= 1'b0;                      // 1-cycle PULSE, exactly like l2_arbiter
         while (!p_ack) @(posedge clk_p);
         gold[lidx] = d; gvalid[lidx] = 1'b1; nwr = nwr + 1;
      end
   endtask

   task do_read(input [5:0] lidx);
      begin
         @(posedge clk_p);
         p_req <= 1'b1; p_we <= 1'b0; p_addr <= {52'd0, lidx};
         @(posedge clk_p);
         p_req <= 1'b0;                      // 1-cycle PULSE
         while (!p_ack) @(posedge clk_p);
         if (gvalid[lidx] && (p_rdata !== gold[lidx])) begin
            errs = errs + 1;
            $display("MISMATCH t=%0d line=%0d", t, lidx);
            $display("   got %h", p_rdata);
            $display("   exp %h", gold[lidx]);
            // name the stale-data case explicitly: does it match ANOTHER line?
            for (i = 0; i < NL; i = i + 1)
               if (gvalid[i] && (p_rdata === gold[i]))
                  $display("   *** got line %0d's data (stale/wrong-line return) ***", i);
            if (errs > 5) begin $display("tb_ddr_line_stress: FAIL (%0d)", errs); $finish; end
         end
         nrd = nrd + 1;
      end
   endtask

   initial begin
      p_req=0; p_we=0; p_addr=0; p_wdata=0; errs=0; nrd=0; nwr=0; gvalid=0;
      for (i=0;i<NW;i=i+1) mem[i]=0;
      reset=1; repeat(8) @(posedge clk_p); reset=0; repeat(2) @(posedge clk_p);

      // seed every line so reads have known content
      for (i=0;i<NL;i=i+1) begin
         dat = {16{$random, $random}} ^ {{15{32'd0}}, i[31:0]};
         do_write(i[5:0], dat);
      end

      for (t=0; t<ntrans; t=t+1) begin
         li = $random & (NL-1);
         if (($random & 3) == 0) begin           // 25% writes, 75% reads (fill-heavy, like the D$)
            dat = {16{$random, $random}} ^ {{15{32'd0}}, t[31:0]};
            do_write(li, dat);
         end else
            do_read(li);
         if (gap != 0) repeat(gap) @(posedge clk_p);
      end

      $display("tb_ddr_line_stress: ratio=%0d gap=%0d reads=%0d writes=%0d errors=%0d -> %0s",
               ratio, gap, nrd, nwr, errs, errs==0 ? "PASS" : "FAIL");
      $finish;
   end

   initial begin #500000000; $display("tb_ddr_line_stress: TIMEOUT"); $finish; end
endmodule

`default_nettype wire
