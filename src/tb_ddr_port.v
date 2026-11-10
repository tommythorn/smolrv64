`timescale 1ns/1ps
`default_nettype none
// The FPGA-only memory path under load: rv_soc_top's DDR memory port -> ddr_port_sync -> ddr4_mem
// (the controller, its line port and its DMA AXI port, a DMA master on the latter) -> the DDR4 IP's
// PHY-only port, modelled by ddr4_phy_model, which checks every DRAM rule. No system simulation
// reaches this path (the Linux testbenches model the DDR at rv_soc_top's port), so this is where it
// is proven, across the two clocks -- clk_p is clk_m / DIV (2 shipping; tb_ddr_port_div4.v runs 4)
// and late by 2.7 ns, as the board's BUFGCE_DIV makes it.
//
// The core side keeps up to NFLY transactions in flight across its ids -- random reads, full and
// partial-mask writes, never two in flight on one line (the data cache's promise) -- and checks
// every read's bytes against a shadow, and that every id is answered exactly once. The DMA master
// runs its own line writes and line reads in a range of its own at the same time, reading back
// only lines whose writes have been answered, and checks those against its own shadow. Its snoops
// cross the snoop channel to the core side, which answers each after a random wait, as the D$
// would; every flush answered must come back as one release.
//
//   +n=N (transactions, default 20000)  +seed=N
`include "tb_rand.vh"
module tb #(parameter integer DIV = 2);   // DIV even
   integer ntrans, seedv;
   `TB_RAND(mrnd, mrs)   // the memory side (clk_m)
   `TB_RAND(prnd, prs)   // the port side (clk_p)
   initial begin
      if (!$value$plusargs("n=%d", ntrans)) ntrans = 20000;
      if (!$value$plusargs("seed=%d", seedv)) seedv = 1;
      mrs = `TB_SEED(seedv);  prs = `TB_SEED(seedv + 1);
   end
   // clk_m free-running at 3 ns; clk_p = clk_m / DIV, its edges 2.7 ns after clk_m's (the board's
   // BUFGCE_DIV insertion delay)
   reg clk_m = 0, clk_div = 0;
   integer cdn = 0;
   always #1.5 begin
      clk_m = ~clk_m;
      if (clk_m) begin cdn = cdn + 1; if (cdn == DIV / 2) begin cdn = 0; clk_div = ~clk_div; end end
   end
   wire clk_p;
   assign #2.7 clk_p = clk_div;
   reg reset_p = 1, reset_m = 1;

   // ---------------- the path ----------------
   wire        pq_valid, pq_ready, pr_valid, pr_last, pw_valid;
   wire [4:0]  pq_id, pr_id, pw_id;  wire pq_we;  wire [57:0] pq_addr;  wire [63:0] pq_wmask;
   wire [511:0] pq_wdata;  wire [1:0] pr_beat;  wire [127:0] pr_data;
   wire        mq_valid, mq_ready, mq_we, mr_valid, mr_ready, mr_last, mw_valid, mw_ready;
   wire [4:0]  mq_id, mr_id, mw_id;  wire [57:0] mq_addr;  wire [63:0] mq_wmask;  wire [511:0] mq_wdata;
   wire [1:0]  mr_beat;  wire [127:0] mr_data;
   wire        sm_valid, sm_keep, sm_ack, sm_rel, sp_valid, sp_keep, sp_rel;  wire [25:0] sm_line, sp_line;
   reg         sp_ack = 1'b0;
   ddr_port_sync #(.DIV(DIV)) cdc
     (.clk_p(clk_p), .reset_p(reset_p),
      .p_q_valid(pq_valid), .p_q_ready(pq_ready), .p_q_id(pq_id), .p_q_we(pq_we), .p_q_addr(pq_addr),
      .p_q_wmask(pq_wmask), .p_q_wdata(pq_wdata),
      .p_r_valid(pr_valid), .p_r_ready(1'b1), .p_r_id(pr_id), .p_r_beat(pr_beat), .p_r_last(pr_last),
      .p_r_data(pr_data), .p_w_valid(pw_valid), .p_w_ready(1'b1), .p_w_id(pw_id),
      .clk_m(clk_m), .reset_m(reset_m),
      .m_q_valid(mq_valid), .m_q_ready(mq_ready), .m_q_id(mq_id), .m_q_we(mq_we), .m_q_addr(mq_addr),
      .m_q_wmask(mq_wmask), .m_q_wdata(mq_wdata),
      .m_r_valid(mr_valid), .m_r_ready(mr_ready), .m_r_id(mr_id), .m_r_beat(mr_beat), .m_r_last(mr_last),
      .m_r_data(mr_data), .m_w_valid(mw_valid), .m_w_ready(mw_ready), .m_w_id(mw_id),
      .m_s_valid(sm_valid), .m_s_line(sm_line), .m_s_keep(sm_keep), .m_s_ack(sm_ack), .m_s_rel(sm_rel),
      .p_s_valid(sp_valid), .p_s_line(sp_line), .p_s_keep(sp_keep), .p_s_ack(sp_ack), .p_s_rel(sp_rel));

   // ---------------- the controller on the PHY model ----------------
   wire [2:0]  d_bid, d_rid;  wire [1:0] d_bresp, d_rresp;  wire [63:0] d_rdata;
   wire        d_awready, d_wready, d_bvalid, d_arready, d_rlast, d_rvalid;
   reg         d_awvalid, d_wvalid, d_wlast, d_arvalid;  reg [31:0] d_awaddr, d_araddr;  reg [63:0] d_wdata;
   wire [7:0]   mc_ACT_n, mc_BG, mc_CS_n, mc_ODT, mc_CKE;  wire [135:0] mc_ADR;  wire [15:0] mc_BA;
   wire         mcRdCAS, mcWrCAS, mcCasSlot2, winInjTxn, winRmw, gt_data_ready, calDone;
   wire [1:0]   mcCasSlot, winRank;  wire [4:0] winBuf, rdDataAddr, wrDataAddr;
   wire [255:0] wrData, rdData;  wire [31:0] wrDataMask;  wire rdDataEn, wrDataEn, per_rd_done;
   ddr4_mem mem
     (.clk(clk_m), .reset(reset_m), .cal_done(calDone),
      .q_valid(mq_valid), .q_ready(mq_ready), .q_id(mq_id), .q_we(mq_we), .q_addr(mq_addr),
      .q_wmask(mq_wmask), .q_wdata(mq_wdata),
      .r_valid(mr_valid), .r_ready(mr_ready), .r_id(mr_id), .r_beat(mr_beat), .r_last(mr_last),
      .r_data(mr_data), .w_valid(mw_valid), .w_ready(mw_ready), .w_id(mw_id),
      .s_axi_awid(3'd1), .s_axi_awaddr(d_awaddr), .s_axi_awlen(8'd7), .s_axi_awsize(3'd3), .s_axi_awburst(2'b01),
      .s_axi_awvalid(d_awvalid), .s_axi_awready(d_awready), .s_axi_wdata(d_wdata), .s_axi_wstrb(8'hff),
      .s_axi_wlast(d_wlast), .s_axi_wvalid(d_wvalid), .s_axi_wready(d_wready),
      .s_axi_bid(d_bid), .s_axi_bresp(d_bresp), .s_axi_bvalid(d_bvalid), .s_axi_bready(1'b1),
      .s_axi_arid(3'd1), .s_axi_araddr(d_araddr), .s_axi_arlen(8'd7), .s_axi_arsize(3'd3), .s_axi_arburst(2'b01),
      .s_axi_arvalid(d_arvalid), .s_axi_arready(d_arready), .s_axi_rid(d_rid), .s_axi_rdata(d_rdata),
      .s_axi_rresp(d_rresp), .s_axi_rlast(d_rlast), .s_axi_rvalid(d_rvalid), .s_axi_rready(1'b1),
      .sn_valid(sm_valid), .sn_line(sm_line), .sn_keep(sm_keep), .sn_ack(sm_ack), .sn_rel(sm_rel),
      .mc_ACT_n(mc_ACT_n), .mc_ADR(mc_ADR), .mc_BA(mc_BA), .mc_BG(mc_BG), .mc_CS_n(mc_CS_n), .mc_ODT(mc_ODT),
      .mc_CKE(mc_CKE), .mcRdCAS(mcRdCAS), .mcWrCAS(mcWrCAS), .mcCasSlot(mcCasSlot), .mcCasSlot2(mcCasSlot2),
      .winBuf(winBuf), .winInjTxn(winInjTxn), .winRmw(winRmw), .winRank(winRank), .gt_data_ready(gt_data_ready),
      .wrData(wrData), .wrDataMask(wrDataMask), .wrDataEn(wrDataEn), .wrDataAddr(wrDataAddr),
      .rdDataEn(rdDataEn), .rdDataAddr(rdDataAddr), .rdData(rdData), .per_rd_done(per_rd_done));
   ddr4_phy_model phy
     (.clk(clk_m), .reset(reset_m), .calDone(calDone),
      .mc_ACT_n(mc_ACT_n), .mc_ADR(mc_ADR), .mc_BA(mc_BA), .mc_BG(mc_BG), .mc_CS_n(mc_CS_n), .mc_ODT(mc_ODT),
      .mc_CKE(mc_CKE), .mcRdCAS(mcRdCAS), .mcWrCAS(mcWrCAS), .mcCasSlot(mcCasSlot), .mcCasSlot2(mcCasSlot2),
      .winBuf(winBuf), .winInjTxn(winInjTxn), .winRmw(winRmw), .winRank(winRank), .gt_data_ready(gt_data_ready),
      .wrData(wrData), .wrDataMask(wrDataMask), .wrDataEn(wrDataEn), .wrDataAddr(wrDataAddr),
      .rdDataEn(rdDataEn), .rdDataAddr(rdDataAddr), .rdData(rdData), .per_rd_done(per_rd_done));
   reg [31:0] rnd;
   always @(posedge clk_m) rnd = mrnd(0);

   // ---------------- the DMA master: one line read and one line write at a time ----------------
   // 64 lines at 0x100000..: a write's line joins the shadow when its B comes; a read takes a line
   // already written and not being written now
   localparam integer DLN = 64;
   reg [511:0] d_shadow [0:DLN-1];  reg d_written [0:DLN-1];
   integer dma_r = 0, dma_w = 0, d_wb = 0, d_wl = -1, d_rl = -1, dk;
   reg d_rbusy = 0, d_wbusy = 0;
   reg [511:0] d_wline;
   reg [2:0] d_rbeat = 0;
   initial begin
      d_arvalid = 0; d_awvalid = 0; d_wvalid = 0; d_wlast = 0;
      for (dk = 0; dk < DLN; dk = dk + 1) d_written[dk] = 0;
   end
   always @(posedge clk_m) if (!reset_m) begin
      if (d_arvalid && d_arready) d_arvalid <= 0;
      if (!d_rbusy && !d_arvalid && rnd[20:18] == 3'd0) begin
         dk = rnd[29:24];
         if (d_written[dk] && dk != d_wl) begin
            d_rl = dk;  d_araddr <= 32'h100000 + dk * 64;  d_arvalid <= 1;  d_rbusy <= 1;  d_rbeat <= 0;
         end
      end
      if (d_rvalid) begin
         if (d_rid != 3'd1) begin $display("FAIL: DMA read data with ID %0d", d_rid); $finish; end
         if (d_rdata !== d_shadow[d_rl][d_rbeat*64 +: 64]) begin
            $display("FAIL: DMA read line %0d beat %0d: %h, expected %h", d_rl, d_rbeat, d_rdata, d_shadow[d_rl][d_rbeat*64 +: 64]);
            $finish;
         end
         d_rbeat <= d_rbeat + 1;
         if (d_rlast) begin d_rbusy <= 0; dma_r = dma_r + 1; d_rl = -1; end
      end
      if (d_awvalid && d_awready) d_awvalid <= 0;
      if (d_wvalid && d_wready) begin
         d_wb = d_wb + 1;
         if (d_wlast) begin d_wvalid <= 0; d_wlast <= 0; end
         else begin d_wdata <= d_wline[d_wb*64 +: 64]; d_wlast <= (d_wb == 7); end
      end
      if (d_bvalid) begin
         if (d_bid != 3'd1) begin $display("FAIL: DMA write response with ID %0d", d_bid); $finish; end
         d_shadow[d_wl] = d_wline;  d_written[d_wl] = 1;  d_wl = -1;
         d_wbusy <= 0; dma_w = dma_w + 1;
      end
      if (!d_wbusy && rnd[23:21] == 3'd0) begin
         dk = rnd[31:26];
         if (dk != d_rl) begin
            d_wl = dk;  for (d_wb = 0; d_wb < 16; d_wb = d_wb + 1) d_wline[d_wb*32 +: 32] = mrnd(0);
            d_awaddr <= 32'h100000 + dk * 64;  d_awvalid <= 1;  d_wvalid <= 1;
            d_wdata <= d_wline[63:0];  d_wlast <= 0;  d_wb = 0;  d_wbusy <= 1;
         end
      end
   end

   // ---------------- the core side: NFLY in flight, one per line at a time, checked ----------------
   localparam integer NFLY = 16, NLN = 64;          // lines 0..63 of the first 4 KiB
   reg         busy_ln [0:NLN-1];
   reg         id_live [0:31];  reg id_we [0:31];  reg [5:0] id_ln [0:31];  reg [511:0] id_exp [0:31];
   reg [511:0] shadow [0:NLN-1];
   reg [511:0] got [0:31];  reg [3:0] got_n [0:31];
   integer     issued = 0, done = 0, nfly = 0, errors = 0, k, ln, idf, cyc = 0;
   // the core side's answers to the snoops: after 0..7 clk_p cycles; a flush owes one release
   integer sp_wait = -1, n_snoop = 0, rel_owed = 0;
   always @(posedge clk_p) if (!reset_p) begin
      sp_ack <= 1'b0;
      if (sp_rel) begin
         if (rel_owed == 0) begin $display("FAIL: a release with no flush answered"); errors = errors + 1; end
         rel_owed = rel_owed - 1;
      end
      if (sp_valid && !sp_ack) begin
         if (sp_wait < 0) sp_wait = prnd(0) % 8;
         else if (sp_wait == 0) begin
            sp_ack <= 1'b1;  sp_wait = -1;  n_snoop = n_snoop + 1;
            if (!sp_keep) rel_owed = rel_owed + 1;
            if (rel_owed > 1) begin $display("FAIL: a flush answered before the last one's release"); errors = errors + 1; end
         end else sp_wait = sp_wait - 1;
      end
   end

   reg         q_v = 0;  reg [4:0] q_id;  reg q_we;  reg [57:0] q_addr;  reg [63:0] q_mask;  reg [511:0] q_data;
   reg [511:0] mbits, nd, t_data;  reg [63:0] t_mask;  reg t_we;
   assign pq_valid = q_v;  assign pq_id = q_id;  assign pq_we = q_we;  assign pq_addr = q_addr;
   assign pq_wmask = q_mask;  assign pq_wdata = q_data;
   initial begin
      for (k = 0; k < NLN; k = k + 1) busy_ln[k] = 0;
      for (k = 0; k < 32; k = k + 1) begin id_live[k] = 0; got_n[k] = 0; end
   end
   // the request registers change with non-blocking assignments, so the FIFO samples them before
   // the edge; the bookkeeping is the testbench's own and is blocking
   always @(posedge clk_p) if (!reset_p) begin
      cyc = cyc + 1;
      if (cyc > 40 * ntrans + 100000) begin $display("FAIL: hung (issued %0d done %0d)", issued, done); $finish; end
      if (pr_valid) begin
         if (!id_live[pr_id] || id_we[pr_id]) begin $display("FAIL: read data for id %0d, not a live read", pr_id); errors = errors + 1; end
         got[pr_id][pr_beat*128 +: 128] = pr_data;  got_n[pr_id] = got_n[pr_id] + 1;
         if (pr_last) begin
            if (got_n[pr_id] != 4) begin $display("FAIL: id %0d last beat after %0d beats", pr_id, got_n[pr_id]); errors = errors + 1; end
            if (got[pr_id] !== id_exp[pr_id]) begin
               $display("FAIL: id %0d line %0d read %h expected %h", pr_id, id_ln[pr_id], got[pr_id], id_exp[pr_id]);
               errors = errors + 1;
            end
            got_n[pr_id] = 0;  id_live[pr_id] = 0;  busy_ln[id_ln[pr_id]] = 0;  nfly = nfly - 1;  done = done + 1;
         end
      end
      if (pw_valid) begin
         if (!id_live[pw_id] || !id_we[pw_id]) begin $display("FAIL: write done for id %0d, not a live write", pw_id); errors = errors + 1; end
         id_live[pw_id] = 0;  busy_ln[id_ln[pw_id]] = 0;  nfly = nfly - 1;  done = done + 1;
      end
      if (q_v && pq_ready) q_v <= 1'b0;
      if ((!q_v || pq_ready) && issued < ntrans && nfly < NFLY && rnd[2:0] != 0) begin
         ln = {prnd(0)} % NLN;
         idf = -1;
         for (k = 31; k >= 0; k = k - 1) if (!id_live[k]) idf = k;
         if (!busy_ln[ln] && idf >= 0) begin
            t_we = rnd[3];
            for (k = 0; k < 16; k = k + 1) t_data[k*32 +: 32] = prnd(0);
            t_mask = rnd[4] ? {64{1'b1}} : {prnd(0), prnd(0)};
            q_v <= 1'b1;  q_id <= idf[4:0];  q_addr <= 58'h200_0000 + ln;   // DRAM line ln
            q_we <= t_we;  q_data <= t_data;  q_mask <= t_mask;
            id_live[idf] = 1;  id_we[idf] = t_we;  id_ln[idf] = ln[5:0];  busy_ln[ln] = 1;
            nfly = nfly + 1;  issued = issued + 1;
            if (t_we) begin
               for (k = 0; k < 64; k = k + 1) mbits[k*8 +: 8] = {8{t_mask[k]}};
               nd = (shadow[ln] & ~mbits) | (t_data & mbits);
               shadow[ln] = nd;
            end else id_exp[idf] = shadow[ln];
         end
      end
      if (done == ntrans) begin
         if (errors == 0 && n_snoop >= dma_r + dma_w && n_snoop > 0)
            $display("tb_ddr_port: PASS (DIV %0d, %0d transactions, DMA %0d reads %0d writes, %0d snoops, %0d clk_p cycles)",
                     DIV, ntrans, dma_r, dma_w, n_snoop, cyc);
         else $display("tb_ddr_port: FAIL (%0d errors; %0d snoops for %0d DMA lines, %0d releases owed)", errors, n_snoop, dma_r + dma_w, rel_owed);
         $finish;
      end
   end
   // the shadow starts as the model's never-written bytes: zero
   integer si2;
   initial for (si2 = 0; si2 < NLN; si2 = si2 + 1) shadow[si2] = 512'd0;
   initial begin
      repeat (8) @(posedge clk_m);
      reset_m = 0;
      repeat (4) @(posedge clk_p);
      reset_p = 0;
   end
endmodule

`include "ddr4_phy_model.vh"

`default_nettype wire
