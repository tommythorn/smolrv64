`timescale 1ns/1ps
`default_nettype none
// The FPGA-only memory path under load: rv_soc_top's DDR memory port -> ddr_port_cdc ->
// ddr_port_axi -> axi_two_master_arbiter (s0; a DMA master on s1) -> an AXI slave shaped like the
// DDR4 controller. No system simulation reaches this path (the Linux testbenches model the DDR at
// rv_soc_top's port), so this is where it is proven.
//
// The core side keeps up to NFLY transactions in flight across its ids -- random reads, full and
// partial-mask writes, never two in flight on one line (the data cache's promise) -- and checks
// every read's bytes against a shadow, and that every id is answered exactly once. The DMA master
// on s1 runs its own reads and writes in a disjoint range at the same time. The slave accepts
// AR/AW with random ready, answers each after a random latency with random R bubbles and B delay;
// by default in request order (the MIG's behaviour), with +ooo across different IDs as AXI allows,
// to show nothing above depends on the order.
//
//   +n=N (transactions, default 20000)  +ratio=N (clk_m:clk_p, default 2)  +ooo  +seed=N
`include "tb_rand.vh"
module tb;
   integer ntrans, ratio, seedv, ooo;
   `TB_RAND(mrnd, mrs)   // the memory side (clk_m)
   `TB_RAND(prnd, prs)   // the port side (clk_p)
   initial begin
      if (!$value$plusargs("n=%d", ntrans)) ntrans = 20000;
      if (!$value$plusargs("ratio=%d", ratio)) ratio = 2;
      if (!$value$plusargs("seed=%d", seedv)) seedv = 1;
      mrs = `TB_SEED(seedv);  prs = `TB_SEED(seedv + 1);
      ooo = $test$plusargs("ooo");
   end
   // clk_m free-running; clk_p = clk_m / ratio, synchronous (BUFGCE_DIV-like)
   reg clk_m = 0, clk_p = 0;
   integer div = 0;
   always #1.5 begin
      clk_m = ~clk_m;
      if (clk_m) begin div = div + 1; if (div >= ratio) div = 0; end
      if (clk_m && (div == 0 || div == ratio/2)) clk_p = ~clk_p;
   end
   reg reset_p = 1, reset_m = 1;

   // ---------------- the path ----------------
   wire        pq_valid, pq_ready, pr_valid, pr_last, pw_valid;
   wire [4:0]  pq_id, pr_id, pw_id;  wire pq_we;  wire [57:0] pq_addr;  wire [63:0] pq_wmask;
   wire [511:0] pq_wdata;  wire [1:0] pr_beat;  wire [127:0] pr_data;
   wire        mq_valid, mq_ready, mq_we, mr_valid, mr_ready, mr_last, mw_valid, mw_ready;
   wire [4:0]  mq_id, mr_id, mw_id;  wire [57:0] mq_addr;  wire [63:0] mq_wmask;  wire [511:0] mq_wdata;
   wire [1:0]  mr_beat;  wire [127:0] mr_data;
   ddr_port_cdc cdc
     (.clk_p(clk_p), .reset_p(reset_p),
      .p_q_valid(pq_valid), .p_q_ready(pq_ready), .p_q_id(pq_id), .p_q_we(pq_we), .p_q_addr(pq_addr),
      .p_q_wmask(pq_wmask), .p_q_wdata(pq_wdata),
      .p_r_valid(pr_valid), .p_r_ready(1'b1), .p_r_id(pr_id), .p_r_beat(pr_beat), .p_r_last(pr_last),
      .p_r_data(pr_data), .p_w_valid(pw_valid), .p_w_ready(1'b1), .p_w_id(pw_id),
      .clk_m(clk_m), .reset_m(reset_m),
      .m_q_valid(mq_valid), .m_q_ready(mq_ready), .m_q_id(mq_id), .m_q_we(mq_we), .m_q_addr(mq_addr),
      .m_q_wmask(mq_wmask), .m_q_wdata(mq_wdata),
      .m_r_valid(mr_valid), .m_r_ready(mr_ready), .m_r_id(mr_id), .m_r_beat(mr_beat), .m_r_last(mr_last),
      .m_r_data(mr_data), .m_w_valid(mw_valid), .m_w_ready(mw_ready), .m_w_id(mw_id));

   // AXI: c_* the port's master, d_* the DMA master, s_* the arbiter's output to the slave
   wire [2:0] c_awid, c_arid, c_bid, c_rid, d_bid, d_rid, s_awid, s_arid;
   wire [30:0] c_awaddr, c_araddr, s_awaddr, s_araddr;
   wire [7:0] c_awlen, c_arlen, s_awlen, s_arlen, c_wstrb, s_wstrb;
   wire [2:0] c_awsize, c_arsize, s_awsize, s_arsize, c_awprot, c_arprot, s_awprot, s_arprot;
   wire [1:0] c_awburst, c_arburst, s_awburst, s_arburst, c_bresp, c_rresp, d_bresp, d_rresp;
   wire c_awlock, c_arlock, s_awlock, s_arlock;
   wire [3:0] c_awcache, c_arcache, s_awcache, s_arcache, c_awqos, c_arqos, s_awqos, s_arqos;
   wire c_awvalid, c_awready, c_wlast, c_wvalid, c_wready, c_bvalid, c_bready;
   wire c_arvalid, c_arready, c_rlast, c_rvalid, c_rready;
   wire [63:0] c_wdata, c_rdata, d_rdata, s_wdata;
   wire s_awvalid, s_wlast, s_wvalid, s_bready, s_arvalid, s_rready;
   wire d_awready, d_wready, d_bvalid, d_arready, d_rlast, d_rvalid;
   reg  s_awready, s_wready, s_bvalid, s_arready, s_rlast, s_rvalid;
   reg  [2:0] s_bid, s_rid;  reg [63:0] s_rdata;
   reg  d_awvalid, d_wvalid, d_wlast, d_arvalid;  reg [30:0] d_awaddr, d_araddr;  reg [63:0] d_wdata;
   ddr_port_axi bridge
     (.clk(clk_m), .reset(reset_m),
      .q_valid(mq_valid), .q_ready(mq_ready), .q_id(mq_id), .q_we(mq_we), .q_addr(mq_addr),
      .q_wmask(mq_wmask), .q_wdata(mq_wdata),
      .r_valid(mr_valid), .r_ready(mr_ready), .r_id(mr_id), .r_beat(mr_beat), .r_last(mr_last),
      .r_data(mr_data), .w_valid(mw_valid), .w_ready(mw_ready), .w_id(mw_id),
      .m_axi_awid(c_awid), .m_axi_awaddr(c_awaddr), .m_axi_awlen(c_awlen), .m_axi_awsize(c_awsize),
      .m_axi_awburst(c_awburst), .m_axi_awlock(c_awlock), .m_axi_awcache(c_awcache), .m_axi_awprot(c_awprot),
      .m_axi_awqos(c_awqos), .m_axi_awvalid(c_awvalid), .m_axi_awready(c_awready),
      .m_axi_wdata(c_wdata), .m_axi_wstrb(c_wstrb), .m_axi_wlast(c_wlast), .m_axi_wvalid(c_wvalid),
      .m_axi_wready(c_wready), .m_axi_bid(c_bid), .m_axi_bresp(c_bresp), .m_axi_bvalid(c_bvalid),
      .m_axi_bready(c_bready),
      .m_axi_arid(c_arid), .m_axi_araddr(c_araddr), .m_axi_arlen(c_arlen), .m_axi_arsize(c_arsize),
      .m_axi_arburst(c_arburst), .m_axi_arlock(c_arlock), .m_axi_arcache(c_arcache), .m_axi_arprot(c_arprot),
      .m_axi_arqos(c_arqos), .m_axi_arvalid(c_arvalid), .m_axi_arready(c_arready),
      .m_axi_rid(c_rid), .m_axi_rdata(c_rdata), .m_axi_rresp(c_rresp), .m_axi_rlast(c_rlast),
      .m_axi_rvalid(c_rvalid), .m_axi_rready(c_rready));
   wire [3:0] s_arqos_unused;
   axi_two_master_arbiter arb
     (.clock(clk_m), .reset(reset_m),
      .s0_axi_awid(c_awid), .s0_axi_awaddr(c_awaddr), .s0_axi_awlen(c_awlen), .s0_axi_awsize(c_awsize),
      .s0_axi_awburst(c_awburst), .s0_axi_awlock(c_awlock), .s0_axi_awcache(c_awcache), .s0_axi_awprot(c_awprot),
      .s0_axi_awqos(c_awqos), .s0_axi_awvalid(c_awvalid), .s0_axi_awready(c_awready),
      .s0_axi_wdata(c_wdata), .s0_axi_wstrb(c_wstrb), .s0_axi_wlast(c_wlast), .s0_axi_wvalid(c_wvalid),
      .s0_axi_wready(c_wready), .s0_axi_bid(c_bid), .s0_axi_bresp(c_bresp), .s0_axi_bvalid(c_bvalid),
      .s0_axi_bready(c_bready),
      .s0_axi_arid(c_arid), .s0_axi_araddr(c_araddr), .s0_axi_arlen(c_arlen), .s0_axi_arsize(c_arsize),
      .s0_axi_arburst(c_arburst), .s0_axi_arlock(c_arlock), .s0_axi_arcache(c_arcache), .s0_axi_arprot(c_arprot),
      .s0_axi_arqos(c_arqos), .s0_axi_arvalid(c_arvalid), .s0_axi_arready(c_arready),
      .s0_axi_rid(c_rid), .s0_axi_rdata(c_rdata), .s0_axi_rresp(c_rresp), .s0_axi_rlast(c_rlast),
      .s0_axi_rvalid(c_rvalid), .s0_axi_rready(c_rready),
      .s1_axi_awid(3'd1), .s1_axi_awaddr(d_awaddr), .s1_axi_awlen(8'd7), .s1_axi_awsize(3'd3),
      .s1_axi_awburst(2'b01), .s1_axi_awlock(1'b0), .s1_axi_awcache(4'd0), .s1_axi_awprot(3'd0),
      .s1_axi_awqos(4'd0), .s1_axi_awvalid(d_awvalid), .s1_axi_awready(d_awready),
      .s1_axi_wdata(d_wdata), .s1_axi_wstrb(8'hff), .s1_axi_wlast(d_wlast), .s1_axi_wvalid(d_wvalid),
      .s1_axi_wready(d_wready), .s1_axi_bid(d_bid), .s1_axi_bresp(d_bresp), .s1_axi_bvalid(d_bvalid),
      .s1_axi_bready(1'b1),
      .s1_axi_arid(3'd1), .s1_axi_araddr(d_araddr), .s1_axi_arlen(8'd7), .s1_axi_arsize(3'd3),
      .s1_axi_arburst(2'b01), .s1_axi_arlock(1'b0), .s1_axi_arcache(4'd0), .s1_axi_arprot(3'd0),
      .s1_axi_arqos(4'd0), .s1_axi_arvalid(d_arvalid), .s1_axi_arready(d_arready),
      .s1_axi_rid(d_rid), .s1_axi_rdata(d_rdata), .s1_axi_rresp(d_rresp), .s1_axi_rlast(d_rlast),
      .s1_axi_rvalid(d_rvalid), .s1_axi_rready(1'b1),
      .m_axi_awid(s_awid), .m_axi_awaddr(s_awaddr), .m_axi_awlen(s_awlen), .m_axi_awsize(s_awsize),
      .m_axi_awburst(s_awburst), .m_axi_awlock(s_awlock), .m_axi_awcache(s_awcache), .m_axi_awprot(s_awprot),
      .m_axi_awqos(s_awqos), .m_axi_awvalid(s_awvalid), .m_axi_awready(s_awready),
      .m_axi_wdata(s_wdata), .m_axi_wstrb(s_wstrb), .m_axi_wlast(s_wlast), .m_axi_wvalid(s_wvalid),
      .m_axi_wready(s_wready), .m_axi_bid(s_bid), .m_axi_bresp(2'b00), .m_axi_bvalid(s_bvalid),
      .m_axi_bready(s_bready),
      .m_axi_arid(s_arid), .m_axi_araddr(s_araddr), .m_axi_arlen(s_arlen), .m_axi_arsize(s_arsize),
      .m_axi_arburst(s_arburst), .m_axi_arlock(s_arlock), .m_axi_arcache(s_arcache), .m_axi_arprot(s_arprot),
      .m_axi_arqos(s_arqos), .m_axi_arvalid(s_arvalid), .m_axi_arready(s_arready),
      .m_axi_rid(s_rid), .m_axi_rdata(s_rdata), .m_axi_rresp(2'b00), .m_axi_rlast(s_rlast),
      .m_axi_rvalid(s_rvalid), .m_axi_rready(s_rready));

   // ---------------- the AXI slave: 2 MiB of 64-bit words ----------------
   localparam integer MW = 1 << 18;
   reg [63:0] mem [0:MW-1];
   integer si;
   initial for (si = 0; si < MW; si = si + 1) mem[si] = {$random, $random};
   // reads: a queue of accepted ARs, each with a ready time; in order, or with +ooo any whose
   // ID differs from every older one's
   localparam integer SQ = 16;
   reg        rq_v [0:SQ-1];  reg [2:0] rq_id [0:SQ-1];  reg [30:0] rq_a [0:SQ-1];
   reg [31:0] rq_t [0:SQ-1];  reg [31:0] rq_seq [0:SQ-1];
   reg        wq_v [0:SQ-1];  reg [2:0] wq_id [0:SQ-1];  reg [30:0] wq_a [0:SQ-1];
   reg [31:0] wq_t [0:SQ-1];  reg [31:0] wq_seq [0:SQ-1];  reg wq_dat [0:SQ-1];
   reg [31:0] now = 0, sseq = 0;
   integer    rk = -1, rbeat = 0, wk_data = -1, wbeat = 0;
   reg [31:0] rnd;
   function integer pick_r(input integer dummy);
      integer a, b, ok;
      begin
         pick_r = -1;
         for (a = 0; a < SQ; a = a + 1) if (rq_v[a] && rq_t[a] <= now && pick_r < 0) begin
            ok = 1;
            for (b = 0; b < SQ; b = b + 1)
               if (rq_v[b] && rq_seq[b] < rq_seq[a] && (!ooo || rq_id[b] == rq_id[a])) ok = 0;
            if (ok) pick_r = a;
         end
      end
   endfunction
   function integer pick_b(input integer dummy);
      integer a, b, ok;
      begin
         pick_b = -1;
         for (a = 0; a < SQ; a = a + 1) if (wq_v[a] && wq_dat[a] && wq_t[a] <= now && pick_b < 0) begin
            ok = 1;
            for (b = 0; b < SQ; b = b + 1)
               if (wq_v[b] && wq_seq[b] < wq_seq[a] && (!ooo || wq_id[b] == wq_id[a])) ok = 0;
            if (ok) pick_b = a;
         end
      end
   endfunction
   function integer free_r(input integer dummy);
      integer a; begin free_r = -1; for (a = SQ - 1; a >= 0; a = a - 1) if (!rq_v[a]) free_r = a; end
   endfunction
   function integer free_w(input integer dummy);
      integer a; begin free_w = -1; for (a = SQ - 1; a >= 0; a = a - 1) if (!wq_v[a]) free_w = a; end
   endfunction
   // the oldest write whose data has not come yet: W beats follow AW order
   function integer next_wdata(input integer dummy);
      integer a; reg [31:0] best;
      begin
         next_wdata = -1; best = 32'hffffffff;
         for (a = 0; a < SQ; a = a + 1) if (wq_v[a] && !wq_dat[a] && wq_seq[a] < best) begin best = wq_seq[a]; next_wdata = a; end
      end
   endfunction
   integer fr, fw, pr, pb, nw;
   initial begin
      for (si = 0; si < SQ; si = si + 1) begin rq_v[si] = 0; wq_v[si] = 0; end
      s_arready = 0; s_awready = 0; s_wready = 0; s_rvalid = 0; s_bvalid = 0; s_rlast = 0;
   end
   always @(posedge clk_m) if (!reset_m) begin
      now <= now + 1;
      rnd = mrnd(0);
      // AR / AW accept, random ready
      if (s_arvalid && s_arready) begin
         fr = free_r(0);
         rq_v[fr] <= 1; rq_id[fr] <= s_arid; rq_a[fr] <= s_araddr; rq_seq[fr] <= sseq;
         rq_t[fr] <= now + 8 + (rnd[5:0] & 6'h1f);  sseq = sseq + 1;
      end
      if (s_awvalid && s_awready) begin
         fw = free_w(0);
         wq_v[fw] <= 1; wq_id[fw] <= s_awid; wq_a[fw] <= s_awaddr; wq_seq[fw] <= sseq; wq_dat[fw] <= 0;
         wq_t[fw] <= now + 4 + (rnd[9:6] & 4'hf);  sseq = sseq + 1;
      end
      s_arready <= rnd[10] && (free_r(0) >= 0);
      s_awready <= rnd[11] && (free_w(0) >= 0);
      // W beats into the oldest write still owed its data
      s_wready <= rnd[12];
      if (s_wvalid && s_wready) begin
         nw = next_wdata(0);
         if (nw < 0) begin $display("FAIL: W beat with no AW accepted"); $finish; end
         for (si = 0; si < 8; si = si + 1)
            if (s_wstrb[si]) mem[(wq_a[nw] >> 3) + wbeat][si*8 +: 8] <= s_wdata[si*8 +: 8];
         if (s_wlast != (wbeat == 7)) begin $display("FAIL: wlast on W beat %0d", wbeat); $finish; end
         wbeat = s_wlast ? 0 : wbeat + 1;
         if (s_wlast) wq_dat[nw] <= 1;
      end
      // R: stream the picked read's 8 beats with random bubbles
      if (s_rvalid && s_rready) begin
         if (s_rlast) begin rq_v[rk] <= 0; rk = -1; s_rvalid <= 0; end
         else rbeat = rbeat + 1;
      end
      if (rk < 0 && !(s_rvalid && s_rready && s_rlast)) begin
         pr = pick_r(0);
         if (pr >= 0) begin rk = pr; rbeat = 0; end
      end
      if (rk >= 0 && !(s_rvalid && !s_rready)) begin
         if (rnd[13] || rbeat == 0) begin
            s_rvalid <= 1; s_rid <= rq_id[rk]; s_rdata <= mem[(rq_a[rk] >> 3) + rbeat]; s_rlast <= (rbeat == 7);
         end else s_rvalid <= 0;
      end
      // B: the picked write whose data is in and whose time has come
      if (s_bvalid && s_bready) s_bvalid <= 0;
      else if (!s_bvalid) begin
         pb = pick_b(0);
         if (pb >= 0 && rnd[14]) begin s_bvalid <= 1; s_bid <= wq_id[pb]; wq_v[pb] <= 0; end
      end
   end

   // ---------------- the DMA master on s1: one read and one write at a time ----------------
   // reads at 0x100000.., writes at 0x108000..: disjoint from each other and from the core's lines
   integer dma_r = 0, dma_w = 0, d_wb = 0;
   reg d_rbusy = 0, d_wbusy = 0;
   reg [30:0] d_ra;
   initial begin d_arvalid = 0; d_awvalid = 0; d_wvalid = 0; d_wlast = 0; end
   reg [5:0] d_rbeat = 0;
   always @(posedge clk_m) if (!reset_m) begin
      if (d_arvalid && d_arready) d_arvalid <= 0;
      if (!d_rbusy && !d_arvalid && rnd[20:18] == 3'd0) begin
         d_ra = 31'h100000 + {rnd[28:23], 6'd0};  d_araddr <= d_ra;  d_arvalid <= 1;  d_rbusy <= 1;  d_rbeat <= 0;
      end
      if (d_rvalid) begin
         if (d_rid != 3'd1) begin $display("FAIL: DMA read data with ID %0d", d_rid); $finish; end
         if (d_rdata !== mem[(d_araddr >> 3) + d_rbeat]) begin   // its reads never meet its writes
            $display("FAIL: DMA read %h beat %0d", d_araddr, d_rbeat); $finish;
         end
         d_rbeat <= d_rbeat + 1;
         if (d_rlast) begin d_rbusy <= 0; dma_r = dma_r + 1; end
      end
      if (d_awvalid && d_awready) d_awvalid <= 0;
      if (d_wvalid && d_wready) begin
         d_wb = d_wb + 1;
         if (d_wlast) begin d_wvalid <= 0; d_wlast <= 0; end
         else begin d_wdata <= d_wdata + 1; d_wlast <= (d_wb == 7); end
      end
      if (d_bvalid) begin
         if (d_bid != 3'd1) begin $display("FAIL: DMA write response with ID %0d", d_bid); $finish; end
         d_wbusy <= 0; dma_w = dma_w + 1;
      end
      if (!d_wbusy && rnd[23:21] == 3'd0) begin
         d_awaddr <= 31'h100000 + 31'h8000 + {rnd[30:25], 6'd0};  d_awvalid <= 1;  d_wvalid <= 1;
         d_wdata <= {rnd, rnd};  d_wlast <= 0;  d_wb = 0;  d_wbusy <= 1;
      end
   end

   // ---------------- the core side: NFLY in flight, one per line at a time, checked ----------------
   localparam integer NFLY = 16, NLN = 64;          // lines 0..63 of the first 4 KiB
   reg         busy_ln [0:NLN-1];
   reg         id_live [0:31];  reg id_we [0:31];  reg [5:0] id_ln [0:31];  reg [511:0] id_exp [0:31];
   reg [511:0] shadow [0:NLN-1];
   reg [511:0] got [0:31];  reg [3:0] got_n [0:31];
   integer     issued = 0, done = 0, nfly = 0, errors = 0, k, ln, idf, cyc = 0;
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
            q_v <= 1'b1;  q_id <= idf[4:0];  q_addr <= 58'h200_0000 + ln;   // AXI offset ln*64
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
         if (errors == 0) $display("tb_ddr_port: PASS (%0d transactions, ratio %0d%s, DMA %0d reads %0d writes)",
                                   ntrans, ratio, ooo ? ", reordering slave" : "", dma_r, dma_w);
         else             $display("tb_ddr_port: FAIL (%0d errors)", errors);
         $finish;
      end
   end
   // the shadow starts as the slave's memory
   integer si2;
   initial begin
      #1;
      for (si2 = 0; si2 < NLN; si2 = si2 + 1)
         for (k = 0; k < 8; k = k + 1) shadow[si2][k*64 +: 64] = mem[si2*8 + k];
   end
   initial begin
      repeat (8) @(posedge clk_m);
      reset_m = 0;
      repeat (4) @(posedge clk_p);
      reset_p = 0;
   end
endmodule

`default_nettype wire
