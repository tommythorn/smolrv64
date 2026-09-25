`timescale 1ns/1ps
`default_nettype none
// Random stress bench for rv_icache (ooo2/run-ooo2-icache-tb.sh). A requester issues 8-byte-aligned
// pairs every cycle -- half of them the next sequential pair, some starting at a line's last chunk
// -- through a per-context page table (VA page -> PA page, with aliases); every answer is checked,
// in order, against the tag, the VA, and the 16 bytes at the PA that was sent. An L2 model answers
// line reads after LAT cycles, one at a time. Mapping changes (ep_bump + a new page table) land at
// random, without draining. Code changes land only as the architecture allows: drain, write
// memory, fence.i (inv_req), wait out inv_busy -- and the new bytes must be what comes back.
//   -DLAT=<cycles> (default 12)
module tb;
`ifndef LAT
   localparam LAT = 12;
`else
   localparam LAT = `LAT;
`endif
   localparam NCYC   = 400000;
   localparam PLINES = 8192;                   // 512 KiB of physical memory (8x the cache)
   localparam NVP    = 256;                    // 1 MiB of virtual pages
   reg clk = 0, reset = 1;
   always #5 clk = ~clk;

   reg  [63:0] mem [0:PLINES*8-1];
   reg  [6:0]  ptab [0:NVP-1];                 // VA page -> PA page (128 of them: aliases)
   integer k;

   // ---- DUT ----
   reg          rq_v;  reg [63:0] rq_va, rq_pa;  reg [3:0] rq_tag;
   wire         rd_ack, rd_valid, inv_busy, l2_req, l2_we, pacc, pmiss;
   wire [127:0] rd_data;  wire [63:0] rd_resp_addr;  wire [3:0] rd_resp_tag;
   wire [57:0]  l2_addr;  wire [511:0] l2_wdata;  wire [15:0] err;
   reg          inv_req = 0, ep_bump = 0, l2_ack = 0;  reg [511:0] l2_rdata;
   rv_icache #(.RTW(4)) dut
     (.clk(clk), .reset(reset),
      .rd_req(rq_v), .rd_addr(rq_va), .rd_pa(rq_pa), .rd_tag(rq_tag), .rd_ack(rd_ack),
      .rd_data(rd_data), .rd_valid(rd_valid), .rd_resp_addr(rd_resp_addr), .rd_resp_tag(rd_resp_tag),
      .inv_req(inv_req), .ep_bump(ep_bump), .inv_busy(inv_busy),
      .l2_req(l2_req), .l2_we(l2_we), .l2_addr(l2_addr), .l2_wdata(l2_wdata),
      .l2_rdata(l2_rdata), .l2_ack(l2_ack),
      .perf_access(pacc), .perf_miss(pmiss), .err(err));

   // ---- the L2 model: one read at a time, answered LAT cycles later ----
   reg        l2_busy = 0;  integer l2_cnt;  reg [57:0] l2_a;
   integer w;
   always @(posedge clk) begin
      l2_ack <= 1'b0;
      if (l2_req) begin
         if (l2_busy) begin $display("FAIL: L2 read while one is outstanding"); $finish; end
         l2_busy <= 1'b1;  l2_cnt <= LAT;  l2_a <= l2_addr;
      end else if (l2_busy) begin
         if (l2_cnt == 0) begin
            for (w = 0; w < 8; w = w + 1) l2_rdata[w*64 +: 64] <= mem[{l2_a[12:0], w[2:0]}];
            l2_ack <= 1'b1;  l2_busy <= 1'b0;
         end else l2_cnt <= l2_cnt - 1;
      end
   end

   // ---- the expected answers, in order ----
   reg [63:0] q_va [0:15], q_pa [0:15];  reg [3:0] q_tag [0:15];
   reg [3:0]  q_wp = 0, q_rp = 0;  reg [4:0] q_n = 0;
   function [127:0] pair_at; input [63:0] pa;
      reg [15:0] wi; begin wi = pa[18:3]; pair_at = {mem[wi + 16'd1], mem[wi]}; end endfunction

   // ---- the requester and the checker ----
   integer cyc = 0, nresp = 0, nreq = 0, nfi = 0, nep = 0, nmiss = 0, nacc = 0, nrc = 0;
   reg     quiesce = 0;
   reg [63:0] va_n;  reg [6:0] pp;
   function [63:0] rnd_va; input dummy;
      reg [63:0] v; begin
         // half the time a hot 32 KiB region, so resident lines outlive mapping changes and reconcile
         v = {48'd0, (($urandom % 2) != 0) ? 8'($urandom % 8) : 8'($urandom % NVP), 12'($urandom) & 12'hff0};
         if (v[11:3] == 9'h1ff) v[11:3] = 9'h1fe;          // never across a 4 KiB page
         rnd_va = v;
      end endfunction
   // THE MAPPING CONTRACT: the cache advances its epoch at the edge ending a cycle with ep_bump
   // high, so a request taken in that cycle belongs to the old mapping and one taken after it to
   // the new. The page table switches at that same edge, and a request still waiting is
   // retranslated, as the real requester (which translates every cycle) would present it.
   always @(posedge clk) if (!reset) begin
      cyc <= cyc + 1;
      // a mapping change remaps about one page in eight; the rest keep their physical page, so
      // their resident lines miss virtually (a new epoch) and reconcile by physical tag
      if (ep_bump) for (k = 0; k < NVP; k = k + 1) if (($urandom % 8) == 0) ptab[k] = 7'($urandom);
      if (pacc) nacc <= nacc + 1;
      if (pmiss) nmiss <= nmiss + 1;
      if (dut.st == 3'd7) nrc <= nrc + 1;           // a reconcile's re-stamp
      if (err != 16'd0) begin $display("FAIL: integrity bits %h at cycle %0d", err, cyc); $finish; end
      // answers
      if (rd_valid) begin
         if (q_n == 0) begin $display("FAIL: an answer with nothing outstanding"); $finish; end
         if (rd_resp_tag !== q_tag[q_rp] || rd_resp_addr !== q_va[q_rp]
             || rd_data !== pair_at(q_pa[q_rp])) begin
            $display("FAIL at cycle %0d: tag %h/%h va %h/%h pa %h data %h expected %h", cyc,
                     rd_resp_tag, q_tag[q_rp], rd_resp_addr, q_va[q_rp], q_pa[q_rp], rd_data, pair_at(q_pa[q_rp]));
            $finish;
         end
         nresp <= nresp + 1;  q_rp <= q_rp + 1;
      end
      // requests: a taken one is recorded; a new one follows (sequential half the time)
      if (rq_v & rd_ack) begin
         q_va[q_wp] <= rq_va;  q_pa[q_wp] <= rq_pa;  q_tag[q_wp] <= rq_tag;  q_wp <= q_wp + 1;
         nreq <= nreq + 1;
      end
      q_n <= q_n + {4'd0, rq_v & rd_ack} - {4'd0, rd_valid};
      if (rq_v & ~rd_ack & ep_bump) rq_pa <= {45'd0, ptab[rq_va[19:12]], rq_va[11:0]};
      if (~rq_v | rd_ack) begin
         rq_v <= 1'b0;
         if (~quiesce && q_n < 12 && ($urandom % 8) != 0) begin
            va_n = (rq_v && ($urandom % 2)) ? rq_va + 64'd16 : rnd_va(0);
            if (va_n[11:0] == 12'h000) va_n = rnd_va(0);
            pp = ptab[va_n[19:12]];
            rq_v <= 1'b1;  rq_va <= va_n;  rq_pa <= {45'd0, pp, va_n[11:0]};  rq_tag <= rq_tag + 1;
         end
      end
      // a mapping change: a new page table, the epoch advanced, nothing drained
      ep_bump <= 1'b0;
      if (!quiesce && ($urandom % 6000) == 0) begin ep_bump <= 1'b1; nep <= nep + 1; end
   end

   // ---- fence.i: drain, change code, invalidate, wait ----
   integer n, m;
   initial begin
      for (k = 0; k < PLINES*8; k = k + 1) mem[k] = {$urandom, $urandom};
      for (k = 0; k < NVP; k = k + 1) ptab[k] = 7'($urandom);
      rq_v = 0;  rq_tag = 0;
      repeat (4) @(posedge clk);  reset = 0;
      while (cyc < NCYC) begin
         repeat (15000 + ($urandom % 10000)) @(posedge clk);
         quiesce = 1;
         while (q_n != 0 || rq_v) @(posedge clk);
         for (m = 0; m < 400; m = m + 1) mem[$urandom % (PLINES*8)] = {$urandom, $urandom};
         @(posedge clk) inv_req = 1;
         @(posedge clk) inv_req = 0;
         @(posedge clk);
         while (inv_busy) @(posedge clk);
         nfi = nfi + 1;
         quiesce = 0;
      end
      quiesce = 1;
      n = 0;
      while ((q_n != 0 || rq_v) && n < 10000) begin @(posedge clk); n = n + 1; end
      if (q_n != 0) begin $display("FAIL: %0d requests never answered", q_n); $finish; end
      $display("ICACHE-TB PASS LAT=%0d: %0d requests answered, %0d lookups, %0d misses (%0d reconciled), %0d fence.i, %0d mapping changes",
               LAT, nresp, nacc, nmiss, nrc, nfi, nep);
      $finish;
   end
endmodule
`default_nettype wire
