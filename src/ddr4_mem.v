`default_nettype none
// ddr4_mem -- the memory below ddr_port_sync, on ui_clk: ddr4_ctl and its two requesters, the
// caches' line port (ddr4_port_line, buffers 0..11) and the DMA masters' AXI port (ddr4_port_axi,
// buffers 12..15), on the DDR4 IP's PHY-only port. The DMA port's requests go first (its
// transactions are short and the scanout's are due); each requester hands over its own write data
// the cycle after the PHY names its buffer.
module ddr4_mem #(
   parameter integer IDW = 5
) (
   input  wire            clk,                  // ui_clk
   input  wire            reset,
   input  wire            cal_done,
   // ---- the caches' line port (ddr_port_sync's memory side) ----
   input  wire            q_valid,
   output wire            q_ready,
   input  wire [IDW-1:0]  q_id,
   input  wire            q_we,
   input  wire [57:0]     q_addr,
   input  wire [63:0]     q_wmask,
   input  wire [511:0]    q_wdata,
   output wire            r_valid,
   input  wire            r_ready,
   output wire [IDW-1:0]  r_id,
   output wire [1:0]      r_beat,
   output wire            r_last,
   output wire [127:0]    r_data,
   output wire            w_valid,
   input  wire            w_ready,
   output wire [IDW-1:0]  w_id,
   // ---- the DMA masters' AXI4 port ----
   input  wire [2:0]      s_axi_awid,
   input  wire [31:0]     s_axi_awaddr,
   input  wire [7:0]      s_axi_awlen,
   input  wire [2:0]      s_axi_awsize,
   input  wire [1:0]      s_axi_awburst,
   input  wire            s_axi_awvalid,
   output wire            s_axi_awready,
   input  wire [63:0]     s_axi_wdata,
   input  wire [7:0]      s_axi_wstrb,
   input  wire            s_axi_wlast,
   input  wire            s_axi_wvalid,
   output wire            s_axi_wready,
   output wire [2:0]      s_axi_bid,
   output wire [1:0]      s_axi_bresp,
   output wire            s_axi_bvalid,
   input  wire            s_axi_bready,
   input  wire [2:0]      s_axi_arid,
   input  wire [31:0]     s_axi_araddr,
   input  wire [7:0]      s_axi_arlen,
   input  wire [2:0]      s_axi_arsize,
   input  wire [1:0]      s_axi_arburst,
   input  wire            s_axi_arvalid,
   output wire            s_axi_arready,
   output wire [2:0]      s_axi_rid,
   output wire [63:0]     s_axi_rdata,
   output wire [1:0]      s_axi_rresp,
   output wire            s_axi_rlast,
   output wire            s_axi_rvalid,
   input  wire            s_axi_rready,
   // ---- the DMA port's snoops (ddr_port_sync's snoop channel) ----
   output wire            sn_valid,
   output wire [25:0]     sn_line,
   output wire            sn_keep,
   input  wire            sn_ack,
   output wire            sn_rel,               // the controller has the flushed line's DMA write
   // ---- the DDR4 IP's PHY-only port ----
   output wire [7:0]      mc_ACT_n,
   output wire [135:0]    mc_ADR,
   output wire [15:0]     mc_BA,
   output wire [7:0]      mc_BG,
   output wire [7:0]      mc_CS_n,
   output wire [7:0]      mc_ODT,
   output wire [7:0]      mc_CKE,
   output wire            mcRdCAS,
   output wire            mcWrCAS,
   output wire [1:0]      mcCasSlot,
   output wire            mcCasSlot2,
   output wire [4:0]      winBuf,
   output wire            winInjTxn,
   output wire            winRmw,
   output wire [1:0]      winRank,
   output wire            gt_data_ready,
   output wire [255:0]    wrData,
   output wire [31:0]     wrDataMask,
   input  wire            wrDataEn,
   input  wire [4:0]      wrDataAddr,
   input  wire            rdDataEn,
   input  wire [4:0]      rdDataAddr,
   input  wire [255:0]    rdData,
   input  wire            per_rd_done
);
   wire        l_rq_valid, l_rq_ready, l_rq_we;  wire [25:0] l_rq_line;  wire [3:0] l_rq_buf;
   wire        a_rq_valid, a_rq_ready, a_rq_we;  wire [25:0] a_rq_line;  wire [3:0] a_rq_buf;
   wire        rq_ready;
   wire        l_wd_mine, a_wd_mine;  wire [255:0] l_wd_data, a_wd_data;  wire [31:0] l_wd_mask, a_wd_mask;

   assign a_rq_ready = rq_ready;
   assign l_rq_ready = rq_ready && !a_rq_valid;
   // the half the PHY asked for, from the requester it named, registered once: the PHY takes it a cycle
   // after it asks
   reg  [255:0] wd_q;  reg [31:0] wm_q;
   always @(posedge clk) {wm_q, wd_q} <= a_wd_mine ? {a_wd_mask, a_wd_data} : {l_wd_mask, l_wd_data};
   assign wrData     = wd_q;
   assign wrDataMask = wm_q;

   ddr4_ctl u_ctl (
      .clk(clk), .reset(reset), .cal_done(cal_done),
      .rq_valid(a_rq_valid || l_rq_valid), .rq_ready(rq_ready),
      .rq_we  (a_rq_valid ? a_rq_we   : l_rq_we),
      .rq_line(a_rq_valid ? a_rq_line : l_rq_line),
      .rq_buf (a_rq_valid ? a_rq_buf  : l_rq_buf),
      .mc_ACT_n(mc_ACT_n), .mc_ADR(mc_ADR), .mc_BA(mc_BA), .mc_BG(mc_BG), .mc_CS_n(mc_CS_n),
      .mc_ODT(mc_ODT), .mc_CKE(mc_CKE), .mcRdCAS(mcRdCAS), .mcWrCAS(mcWrCAS),
      .mcCasSlot(mcCasSlot), .mcCasSlot2(mcCasSlot2), .winBuf(winBuf), .winInjTxn(winInjTxn),
      .winRmw(winRmw), .winRank(winRank), .gt_data_ready(gt_data_ready),
      .rdDataEn(rdDataEn), .per_rd_done(per_rd_done));

   ddr4_port_line #(.IDW(IDW)) u_line (
      .clk(clk), .reset(reset),
      .q_valid(q_valid), .q_ready(q_ready), .q_id(q_id), .q_we(q_we), .q_addr(q_addr),
      .q_wmask(q_wmask), .q_wdata(q_wdata),
      .r_valid(r_valid), .r_ready(r_ready), .r_id(r_id), .r_beat(r_beat), .r_last(r_last), .r_data(r_data),
      .w_valid(w_valid), .w_ready(w_ready), .w_id(w_id),
      .rq_valid(l_rq_valid), .rq_ready(l_rq_ready), .rq_we(l_rq_we), .rq_line(l_rq_line), .rq_buf(l_rq_buf),
      .rdDataEn(rdDataEn), .rdDataAddr(rdDataAddr), .rdData(rdData),
      .wrDataEn(wrDataEn), .wrDataAddr(wrDataAddr),
      .wd_mine(l_wd_mine), .wd_data(l_wd_data), .wd_mask(l_wd_mask));

   ddr4_port_axi u_axi (
      .clk(clk), .reset(reset),
      .s_axi_awid(s_axi_awid), .s_axi_awaddr(s_axi_awaddr), .s_axi_awlen(s_axi_awlen),
      .s_axi_awsize(s_axi_awsize), .s_axi_awburst(s_axi_awburst), .s_axi_awvalid(s_axi_awvalid),
      .s_axi_awready(s_axi_awready), .s_axi_wdata(s_axi_wdata), .s_axi_wstrb(s_axi_wstrb),
      .s_axi_wlast(s_axi_wlast), .s_axi_wvalid(s_axi_wvalid), .s_axi_wready(s_axi_wready),
      .s_axi_bid(s_axi_bid), .s_axi_bresp(s_axi_bresp), .s_axi_bvalid(s_axi_bvalid), .s_axi_bready(s_axi_bready),
      .s_axi_arid(s_axi_arid), .s_axi_araddr(s_axi_araddr), .s_axi_arlen(s_axi_arlen),
      .s_axi_arsize(s_axi_arsize), .s_axi_arburst(s_axi_arburst), .s_axi_arvalid(s_axi_arvalid),
      .s_axi_arready(s_axi_arready), .s_axi_rid(s_axi_rid), .s_axi_rdata(s_axi_rdata),
      .s_axi_rresp(s_axi_rresp), .s_axi_rlast(s_axi_rlast), .s_axi_rvalid(s_axi_rvalid), .s_axi_rready(s_axi_rready),
      .sn_valid(sn_valid), .sn_line(sn_line), .sn_keep(sn_keep), .sn_ack(sn_ack), .sn_rel(sn_rel),
      .rq_valid(a_rq_valid), .rq_ready(a_rq_ready), .rq_we(a_rq_we), .rq_line(a_rq_line), .rq_buf(a_rq_buf),
      .rdDataEn(rdDataEn), .rdDataAddr(rdDataAddr), .rdData(rdData),
      .wrDataEn(wrDataEn), .wrDataAddr(wrDataAddr),
      .wd_mine(a_wd_mine), .wd_data(a_wd_data), .wd_mask(a_wd_mask));

`ifndef SYNTHESIS
   always @(posedge clk) if (!reset) begin
      if (l_wd_mine && a_wd_mine) $fatal(1, "ddr4_mem: both requesters hand over write data at once");
      if (wrDataEn && !(wrDataAddr[4:3] == 2'b10 || wrDataAddr[4:2] == 3'b111))
         $fatal(1, "ddr4_mem: the PHY asked for write data of tag %b, a read buffer's", wrDataAddr);
   end
`endif
endmodule

`default_nettype wire
