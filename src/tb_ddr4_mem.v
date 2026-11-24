`default_nettype none
`timescale 1ns/1ps

// Unit bench for ddr4_mem (ddr4_ctl, ddr4_port_line, ddr4_port_axi) on ddr4_phy_model, which checks
// every DDR4 rule the controller owns in DRAM clocks. Two requesters run random traffic at once:
// the caches' line port (tagged line reads and masked line writes, back-pressured result ports)
// and the DMA AXI port (single 8-byte beats with strobes, eight-beat line reads). In phases: mixed
// traffic over a few rows of every bank, streams, writes only (VT tracking's injected read with a
// full queue), idle stretches (its injected read alone), one bank's rows in turn (precharge after
// write recovery). A reference memory takes each write when its request enters the controller,
// the one order every requester's data is defined by, and every read's data is checked against
// the line as it stood then. The bench answers the DMA port's snoops after a random wait, as the D$
// would, and checks that each DMA line goes to the controller only after its own snoop's answer (a
// clean for a read, a flush for a write) and that a flushed write's release comes as the controller
// takes it. The DMA port keeps up to four reads asked at once, so its reads overlap, answered in
// order. At the end every request has completed: PASS. A last phase streams line reads alone with a
// fixed snoop wait (+snoop_wait=N, default 30 cycles: the crossing, the D$ and the L2) and reports
// the port's line rate.
module tb_ddr4_mem;
   reg clk = 0, reset = 1;
   always #1.5 clk = ~clk;

   // ---- the line port ----
   reg          q_valid = 0;  wire q_ready;  reg [4:0] q_id;  reg q_we;  reg [57:0] q_addr;
   reg  [63:0]  q_wmask;  reg [511:0] q_wdata;
   wire         r_valid, r_last;  reg r_ready = 0;  wire [4:0] r_id;  wire [1:0] r_beat;  wire [127:0] r_data;
   wire         w_valid;  reg w_ready = 0;  wire [4:0] w_id;
   // ---- the DMA port ----
   reg  [2:0]   awid;  reg [31:0] awaddr;  reg [7:0] awlen;  reg awvalid = 0;  wire awready;
   reg  [63:0]  wdata;  reg [7:0] wstrb;  reg wlast;  reg wvalid = 0;  wire wready;
   wire [2:0]   bid;  wire [1:0] bresp;  wire bvalid;  reg bready = 0;
   reg  [2:0]   arid;  reg [31:0] araddr;  reg [7:0] arlen;  reg arvalid = 0;  wire arready;
   wire [2:0]   rid;  wire [63:0] rdata;  wire [1:0] rresp;  wire rlast, rvalid;  reg rready = 0;
   // ---- the PHY ----
   wire [7:0]   mc_ACT_n, mc_BG, mc_CS_n, mc_ODT, mc_CKE;  wire [135:0] mc_ADR;  wire [15:0] mc_BA;
   wire         mcRdCAS, mcWrCAS, mcCasSlot2, winInjTxn, winRmw, gt_data_ready;
   wire [1:0]   mcCasSlot, winRank;  wire [4:0] winBuf;
   wire [255:0] wrData, rdData;  wire [31:0] wrDataMask;
   wire         wrDataEn, rdDataEn, per_rd_done, calDone;  wire [4:0] wrDataAddr, rdDataAddr;
   // ---- the snoops ----
   wire         sn_valid, sn_keep, sn_rel;  wire [25:0] sn_line;  reg sn_ack = 0;

   ddr4_mem dut (
      .clk(clk), .reset(reset), .cal_done(calDone),
      .q_valid(q_valid), .q_ready(q_ready), .q_id(q_id), .q_we(q_we), .q_addr(q_addr), .q_wmask(q_wmask), .q_wdata(q_wdata),
      .r_valid(r_valid), .r_ready(r_ready), .r_id(r_id), .r_beat(r_beat), .r_last(r_last), .r_data(r_data),
      .w_valid(w_valid), .w_ready(w_ready), .w_id(w_id),
      .s_axi_awid(awid), .s_axi_awaddr(awaddr), .s_axi_awlen(awlen), .s_axi_awsize(3'd3), .s_axi_awburst(2'b01),
      .s_axi_awvalid(awvalid), .s_axi_awready(awready), .s_axi_wdata(wdata), .s_axi_wstrb(wstrb), .s_axi_wlast(wlast),
      .s_axi_wvalid(wvalid), .s_axi_wready(wready), .s_axi_bid(bid), .s_axi_bresp(bresp), .s_axi_bvalid(bvalid),
      .s_axi_bready(bready), .s_axi_arid(arid), .s_axi_araddr(araddr), .s_axi_arlen(arlen), .s_axi_arsize(3'd3),
      .s_axi_arburst(2'b01), .s_axi_arvalid(arvalid), .s_axi_arready(arready), .s_axi_rid(rid), .s_axi_rdata(rdata),
      .s_axi_rresp(rresp), .s_axi_rlast(rlast), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
      .sn_valid(sn_valid), .sn_line(sn_line), .sn_keep(sn_keep), .sn_ack(sn_ack), .sn_rel(sn_rel),
      .mc_ACT_n(mc_ACT_n), .mc_ADR(mc_ADR), .mc_BA(mc_BA), .mc_BG(mc_BG), .mc_CS_n(mc_CS_n), .mc_ODT(mc_ODT),
      .mc_CKE(mc_CKE), .mcRdCAS(mcRdCAS), .mcWrCAS(mcWrCAS), .mcCasSlot(mcCasSlot), .mcCasSlot2(mcCasSlot2),
      .winBuf(winBuf), .winInjTxn(winInjTxn), .winRmw(winRmw), .winRank(winRank), .gt_data_ready(gt_data_ready),
      .wrData(wrData), .wrDataMask(wrDataMask), .wrDataEn(wrDataEn), .wrDataAddr(wrDataAddr),
      .rdDataEn(rdDataEn), .rdDataAddr(rdDataAddr), .rdData(rdData), .per_rd_done(per_rd_done));

   ddr4_phy_model u_phy (
      .clk(clk), .reset(reset), .calDone(calDone),
      .mc_ACT_n(mc_ACT_n), .mc_ADR(mc_ADR), .mc_BA(mc_BA), .mc_BG(mc_BG), .mc_CS_n(mc_CS_n), .mc_ODT(mc_ODT),
      .mc_CKE(mc_CKE), .mcRdCAS(mcRdCAS), .mcWrCAS(mcWrCAS), .mcCasSlot(mcCasSlot), .mcCasSlot2(mcCasSlot2),
      .winBuf(winBuf), .winInjTxn(winInjTxn), .winRmw(winRmw), .winRank(winRank), .gt_data_ready(gt_data_ready),
      .wrData(wrData), .wrDataMask(wrDataMask), .wrDataEn(wrDataEn), .wrDataAddr(wrDataAddr),
      .rdDataEn(rdDataEn), .rdDataAddr(rdDataAddr), .rdData(rdData), .per_rd_done(per_rd_done));

   // ---- the reference memory: the DRAM window's lines, written as requests enter the controller ----
   bit [511:0] ref_mem [bit [25:0]];
   function automatic bit [511:0] ref_line(input bit [25:0] l); return ref_mem.exists(l) ? ref_mem[l] : '0; endfunction
   function automatic void ref_write(input bit [25:0] l, input bit [511:0] d, input bit [63:0] keep_n);
      bit [511:0] o = ref_line(l);
      for (int k = 0; k < 64; k++) if (keep_n[k]) o[k*8 +: 8] = d[k*8 +: 8];
      ref_mem[l] = o;
   endfunction
   // the DRAM window: physical 0x8000_0000 is line 0
   function automatic bit [57:0] pa_line(input bit [25:0] l); return {31'd0, l[25], ~l[25], l[24:0]}; endfunction

   // ---- the line port's requests: taken in order, entering the controller in that order ----
   typedef struct { bit [4:0] id; bit we; bit [25:0] line; bit [511:0] data; bit [63:0] mask; } l_req_t;
   l_req_t     l_pend [$];
   bit [511:0] l_exp [0:31];                // a read's line as it stood when it entered
   bit         l_busy [0:31], l_is_wr [0:31];
   int         l_beat [0:31];
   longint     l_t0 [0:31];
   int         n_l_rd, n_l_wr, n_a_rd, n_a_wr, lat_sum, lat_n, lat_min, lat_max;
   // ---- the DMA port's transactions ----
   typedef struct { bit [2:0] id; int k, left; } a_ar_t;
   a_ar_t      a_ars [$];                  // the reads asked, in order: their beats come in this order
   bit [511:0] a_exps [$];                 // ...and their lines, as each entered the controller
   int         a_k, a_left;  bit a_wr_busy;
   bit [63:0]  a_wstrb_line;  bit [511:0] a_wdata_line;

   // traffic shape
   int  phase;                              // 0 mixed, 1 streams, 2 writes only, 3 idle, 4 one bank's rows, 5 DMA line reads alone
   bit  [25:0] stream_l;
   function automatic bit [25:0] pick_line();
      bit [25:0] l;
      case (phase)
         1: begin stream_l = stream_l + 1; return stream_l; end
         4: begin l = $urandom; l[8:0] = 9'h05A; l[25:12] = 0; return l; end        // one bank, eight rows
         default: begin l = $urandom; l[25:13] = ($urandom % 4 == 0) ? $urandom : 13'd0; return l; end
      endcase
   endfunction

   longint cyc = 0;
   bit draining = 0;
   always @(posedge clk) cyc <= cyc + 1;

   // ---- the line port ----
   always @(posedge clk) if (!reset) begin
      // a request taken joins the port's order; the reference memory moves as it enters the controller
      if (dut.l_rq_valid && dut.l_rq_ready) begin
         automatic l_req_t r = l_pend.pop_front();
         if (r.we) begin ref_write(r.line, r.data, r.mask); n_l_wr++; end
         else begin l_exp[r.id] = ref_line(r.line); n_l_rd++; end
      end
      if (q_valid && q_ready) begin
         l_pend.push_back('{id: q_id, we: q_we, line: q_addr[25:0] ^ 26'h2000000, data: q_wdata, mask: q_wmask});
         l_t0[q_id] = cyc;
         q_valid <= 0;
      end
      // results
      if (r_valid && r_ready) begin
         if (!l_busy[r_id] || l_is_wr[r_id]) $fatal(1, "tb: read beat for id %0d, not a read in flight", r_id);
         if (r_beat != l_beat[r_id][1:0]) $fatal(1, "tb: id %0d beat %0d, expected %0d", r_id, r_beat, l_beat[r_id]);
         if (r_data !== l_exp[r_id][r_beat*128 +: 128])
            $fatal(1, "tb: id %0d beat %0d: %h, expected %h", r_id, r_beat, r_data, l_exp[r_id][r_beat*128 +: 128]);
         l_beat[r_id]++;
         if (r_last != (r_beat == 2'd3)) $fatal(1, "tb: id %0d: r_last on beat %0d", r_id, r_beat);
         if (r_last) begin
            automatic int lat = cyc - l_t0[r_id];
            l_busy[r_id] = 0;  lat_sum += lat;  lat_n++;
            if (lat < lat_min) lat_min = lat;  if (lat > lat_max) lat_max = lat;
         end
      end
      if (w_valid && w_ready) begin
         if (!l_busy[w_id] || !l_is_wr[w_id]) $fatal(1, "tb: write done for id %0d, not a write in flight", w_id);
         l_busy[w_id] = 0;
      end
      r_ready <= ($urandom % 10) != 0;
      w_ready <= ($urandom % 10) != 0;
      // a new request now and then, on a free id
      if ((!q_valid || q_ready) && !draining && phase != 3 && phase != 5 && ($urandom % 3) == 0) begin
         automatic int id = $urandom % 32;
         if (!l_busy[id]) begin
            automatic bit we = (phase == 2) ? 1 : (phase == 1) ? 0 : ($urandom % 5) < 2;
            automatic bit [25:0] l = pick_line();
            q_valid <= 1;  q_id <= id;  q_we <= we;  q_addr <= pa_line(l);
            for (int k = 0; k < 16; k++) q_wdata[k*32 +: 32] <= $urandom;
            q_wmask <= ($urandom % 3 == 0) ? {$urandom, $urandom} : 64'hFFFF_FFFF_FFFF_FFFF;
            l_busy[id] = 1;  l_is_wr[id] = we;  l_beat[id] = 0;
         end
      end
   end

   // ---- the snoops: answered after 0..15 cycles; each DMA line enters behind its own answer (one
   // answered snoop an engine: a clean is the read engine's, a flush the write engine's) ----
   int  sn_wait = -1, n_sn = 0, sn_fixed = 30;
   bit  sn_w_v = 0;  bit [25:0] sn_w_line;
   bit  [25:0] sn_r_q [$];                  // the read engine's lines snooped and not yet in the controller
   initial void'($value$plusargs("snoop_wait=%d", sn_fixed));
   always @(posedge clk) if (!reset) begin
      sn_ack <= 1'b0;
      if (sn_valid && !sn_ack) begin
         if (sn_wait < 0) sn_wait = (phase == 5) ? sn_fixed : $urandom % 16;
         else if (sn_wait == 0) begin
            sn_ack <= 1'b1;  sn_wait = -1;  n_sn++;
            if (!sn_keep && sn_w_v) $fatal(1, "tb: the write engine's second snoop before its line entered");
            if (sn_keep && sn_r_q.size() >= 3) $fatal(1, "tb: the read engine snooped a fourth line ahead");
            if (sn_keep) sn_r_q.push_back(sn_line); else begin sn_w_v = 1;  sn_w_line = sn_line; end
         end else sn_wait--;
      end
      if (sn_rel != (dut.a_rq_valid && dut.rq_ready && dut.a_rq_we))
         $fatal(1, "tb: sn_rel %0d as a DMA write enters %0d", sn_rel, dut.a_rq_valid && dut.rq_ready && dut.a_rq_we);
      if (dut.a_rq_valid && dut.rq_ready) begin
         if (dut.a_rq_we ? !(sn_w_v && sn_w_line == dut.a_rq_line) : !(sn_r_q.size() != 0 && sn_r_q[0] == dut.a_rq_line))
            $fatal(1, "tb: DMA line %h (we %0d) entered without its snoop", dut.a_rq_line, dut.a_rq_we);
         if (dut.a_rq_we) sn_w_v = 0; else void'(sn_r_q.pop_front());
      end
   end

   // ---- the DMA port: up to four reads asked and one write at a time ----
   int  str_n = 0;  longint str_t0 = 0, str_t1 = 0;
   always @(posedge clk) if (!reset) begin
      // the bridge's requests enter the controller: the reference memory moves then
      if (dut.a_rq_valid && dut.rq_ready) begin
         if (dut.a_rq_we) begin
            ref_write(dut.a_rq_line, dut.u_axi.w_buf, dut.u_axi.w_strb);  n_a_wr++;
         end else begin a_exps.push_back(ref_line(dut.a_rq_line)); n_a_rd++; end
      end
      if (arvalid && arready) arvalid <= 0;
      if (awvalid && awready) awvalid <= 0;
      if (rvalid && rready) begin
         if (a_ars.size() == 0 || a_exps.size() == 0) $fatal(1, "tb: AXI read data with no read in the controller");
         a_k = a_ars[0].k;  a_left = a_ars[0].left;
         if (rdata !== a_exps[0][a_k*64 +: 64]) $fatal(1, "tb: AXI read beat at %0d: %h, expected %h", a_k, rdata, a_exps[0][a_k*64 +: 64]);
         if (rlast != (a_left == 0)) $fatal(1, "tb: AXI rlast with %0d beats left", a_left);
         if (rid != a_ars[0].id) $fatal(1, "tb: AXI rid %0d, expected %0d", rid, a_ars[0].id);
         a_ars[0].k++;  a_ars[0].left--;
         if (rlast) begin
            void'(a_ars.pop_front());  void'(a_exps.pop_front());
            if (phase == 5) begin str_n++;  str_t1 = cyc; end
         end
      end
      if (wvalid && wready) wvalid <= 0;
      if (bvalid && bready) begin
         if (!a_wr_busy || bid != awid) $fatal(1, "tb: AXI B for id %0d with no write", bid);
         a_wr_busy = 0;
      end
      rready <= (phase == 5) || ($urandom % 4) != 0;
      bready <= ($urandom % 4) != 0;
      if ((!arvalid || arready) && a_ars.size() < 4 && !draining && phase != 3 && phase != 2
          && (phase == 5 || ($urandom % 40) == 0)) begin
         automatic bit [25:0] l = (phase == 5) ? stream_l + str_n + a_ars.size() + 1 : pick_line();
         automatic bit line = (phase == 5) || ($urandom % 2) == 0;
         automatic bit [2:0] k = line ? 3'd0 : $urandom;
         automatic bit [2:0] id = $urandom;
         arvalid <= 1;  arid <= id;  araddr <= {l, k, 3'b000};  arlen <= line ? 8'd7 : 8'd0;
         a_ars.push_back('{id: id, k: k, left: line ? 7 : 0});
         if (phase == 5 && str_t0 == 0) str_t0 = cyc;
      end
      if (!a_wr_busy && !draining && phase != 3 && phase != 5 && ($urandom % 40) == 0) begin
         automatic bit [25:0] l = pick_line();
         awvalid <= 1;  awid <= $urandom;  awaddr <= {l, 3'($urandom), 3'b000};  awlen <= 8'd0;
         wvalid <= 1;  wdata <= {$urandom, $urandom};  wstrb <= $urandom;  wlast <= 1;
         a_wr_busy = 1;
      end
   end

   // ---- the run ----
   function automatic int busy_n();
      int n = 0; for (int k = 0; k < 32; k++) n += l_busy[k]; return n + a_ars.size() + a_wr_busy + q_valid;
   endfunction
   initial begin
      for (int k = 0; k < 32; k++) begin l_busy[k] = 0; l_beat[k] = 0; end
      a_wr_busy = 0; n_l_rd = 0; n_l_wr = 0; n_a_rd = 0; n_a_wr = 0;
      lat_sum = 0; lat_n = 0; lat_min = 1 << 30; lat_max = 0; phase = 0; stream_l = 26'h12340;
      repeat (10) @(posedge clk);
      reset = 0;
      wait (calDone);
      for (int round = 0; round < 3; round++)
         for (int ph = 0; ph < 5; ph++) begin
            phase = ph;
            repeat (ph == 3 ? 3000 : 20000) @(posedge clk);
         end
      draining = 1;
      repeat (3000) @(posedge clk);
      if (busy_n() != 0) $fatal(1, "tb: %0d requests still in flight after draining", busy_n());
      // the DMA port's line rate: line reads alone, back to back, each snoop answered after sn_fixed
      draining = 0;  phase = 5;  stream_l = 26'h23456;
      wait (str_n == 400);
      phase = 3;  draining = 1;
      repeat (3000) @(posedge clk);
      if (busy_n() != 0) $fatal(1, "tb: %0d requests still in flight after the stream", busy_n());
      $display("tb_ddr4_mem: DMA line reads, snoop wait %0d: %0d lines in %0d cycles, %0d.%02d cycles a line",
               sn_fixed, str_n, str_t1 - str_t0, (str_t1 - str_t0) / str_n, ((str_t1 - str_t0) * 100 / str_n) % 100);
      u_phy.report();
      $display("tb_ddr4_mem: line port %0d reads %0d writes, DMA %0d reads %0d writes (%0d snoops); read latency %0d..%0d, mean %0d.%0d cycles",
               n_l_rd, n_l_wr, n_a_rd, n_a_wr, n_sn, lat_min, lat_max, lat_sum / lat_n, (lat_sum * 10 / lat_n) % 10);
      $display("tb_ddr4_mem: PASS");
      $finish;
   end
endmodule

`include "ddr4_phy_model.vh"

`default_nettype wire
