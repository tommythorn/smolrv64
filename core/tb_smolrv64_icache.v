`timescale 1ns/1ps
`default_nettype none
// Random stress bench for rv_icache (core/run-icache-tb.sh). A requester issues windows every
// cycle -- 1 to 8 halfwords from any halfword, inside one 4 KiB page, half of them the window after
// the last one, a fifth of them reaching the next line -- through a per-context page table (VA page
// -> PA page, with aliases); every answer is checked, in order, against the tag, the VA, and the
// window's halfwords at the PA that was sent. An L2 model answers
// line reads after LAT cycles, one at a time. Mapping changes (ep_bump + a new page table) land at
// random, without draining. Code changes land two ways: drain (+nodrain: only stop requesting, as
// the SoC does), write memory, fence.i (inv_req),
// wait out inv_busy -- and the new bytes must be what comes back -- and, all the time, a store:
// a line of memory rewritten and probed (pb_v) with requests in flight. A request taken after the
// probe must see the new bytes; one taken before it may see the bytes of either side, each of the
// window's two lines on its own.
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
   reg          rq_v;  reg [63:0] rq_va, rq_pa;  reg [3:0] rq_tag;  reg [2:0] rq_last;
   wire         rd_ack, rd_valid, inv_busy, l2_req, l2_we, pacc, pmiss;
   wire [127:0] rd_data;  wire [63:0] rd_resp_addr;  wire [3:0] rd_resp_tag;
   wire [57:0]  l2_addr;  wire [511:0] l2_wdata;  wire [15:0] err;
   reg          inv_req = 0, ep_bump = 0, l2_ack = 0;  reg [511:0] l2_rdata;
   reg          pb_v = 0;  reg [63:0] pb_pa = 0;
   reg          fill_hold = 0, fill_cancel = 0, hold_q = 0;
`ifndef IC_KB
 `define IC_KB 64                // the cache size: -DIC_KB=128
`endif
   // a 2-bit epoch: a run's ~60 mapping changes wrap it, so every run invalidates by wrap
   rv_icache #(.RTW(4), .SIZE_KB(`IC_KB), .EPW(2)) dut
     (.clk(clk), .reset(reset),
      .rd_req(rq_v), .rd_addr(rq_va), .rd_last(rq_last), .rd_pa(rq_pa), .rd_tag(rq_tag), .rd_ack(rd_ack),
      .rd_data(rd_data), .rd_valid(rd_valid), .rd_resp_addr(rd_resp_addr), .rd_resp_tag(rd_resp_tag),
      .inv_req(inv_req), .ep_bump(ep_bump), .inv_busy(inv_busy), .pb_v(pb_v), .pb_pa(pb_pa),
      .fill_hold(fill_hold), .fill_cancel(fill_cancel),
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
   reg [63:0] q_va [0:15], q_pa [0:15];  reg [3:0] q_tag [0:15];  reg [127:0] q_old [0:15];  reg [2:0] q_last [0:15];
   reg [15:0] q_st = 0;                 // stale: outstanding at a cancel (data unchecked)
   reg [3:0]  q_wp = 0, q_rp = 0;  reg [4:0] q_n = 0;
   // the window's halfwords at pa, the first low
   function [127:0] win_at; input [63:0] pa;
      reg [63:0] a; reg [63:0] d; integer i;
      begin
         for (i = 0; i < 8; i = i + 1) begin
            a = pa + 64'(2 * i);  d = mem[a[18:3]];  win_at[i*16 +: 16] = d[a[2:1]*16 +: 16];
         end
      end endfunction
   // the answer by bank (halfword b at address bits [3:1] == b), in window order for the checks
   function [127:0] by_pos; input [127:0] d; input [2:0] sk;
      reg [2:0] b; integer i;
      begin
         for (i = 0; i < 8; i = i + 1) begin b = sk + 3'(i);  by_pos[i*16 +: 16] = d[b*16 +: 16]; end
      end endfunction
   // an answer matches when each of the window's lines holds its new bytes or its old ones
   function win_ok; input [127:0] d, nw, od; input [63:0] pa; input [2:0] last;
      reg [127:0] m1, m2; reg [63:0] a; integer i;
      begin
         m1 = 128'd0;  m2 = 128'd0;
         for (i = 0; i < 8; i = i + 1) if (i <= last) begin
            a = pa + 64'(2 * i);
            if (a[63:6] == pa[63:6]) m1[i*16 +: 16] = 16'hffff;  else m2[i*16 +: 16] = 16'hffff;
         end
         win_ok = (((d & m1) == (nw & m1)) || ((d & m1) == (od & m1)))
               && (((d & m2) == (nw & m2)) || ((d & m2) == (od & m2)));
      end endfunction

   // ---- the requester and the checker ----
   integer cyc = 0, nresp = 0, nreq = 0, nfi = 0, nep = 0, nmiss = 0, nacc = 0, nrc = 0;
   integer npb = 0, nfdead = 0, npfk = 0, nx = 0, nmx = 0, nrcn = 0;
   integer nhold = 0, ncan = 0, ncanq = 0;
   reg [63:0] pb_va;  reg [57:0] pb_line;
   reg     quiesce = 0;
   reg [63:0] va_n;  reg [6:0] pp;  reg [2:0] last_n;
   function [63:0] rnd_va; input dummy;
      reg [63:0] v; begin
         // half the time a hot 32 KiB region, so resident lines outlive mapping changes and reconcile
         v = {48'd0, (($urandom % 2) != 0) ? 8'($urandom % 8) : 8'($urandom % NVP), 12'($urandom) & 12'hffe};
         rnd_va = v;
      end endfunction
   // a window's last halfword: any, but never past its page
   function [2:0] rnd_last; input [63:0] v;
      reg [11:0] room; begin
         room = 12'h7ff - {1'b0, v[11:1]};
         rnd_last = (room < 12'd7) ? room[2:0] : 3'($urandom % 8);
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
             || (~q_st[q_rp] && !win_ok(by_pos(rd_data, q_pa[q_rp][3:1]), win_at(q_pa[q_rp]), q_old[q_rp], q_pa[q_rp], q_last[q_rp]))) begin
            $display("FAIL at cycle %0d: tag %h/%h va %h/%h pa %h last %0d data %h expected %h", cyc,
                     rd_resp_tag, q_tag[q_rp], rd_resp_addr, q_va[q_rp], q_pa[q_rp], q_last[q_rp], by_pos(rd_data, q_pa[q_rp][3:1]), win_at(q_pa[q_rp]));
            $finish;
         end
         nresp <= nresp + 1;  q_rp <= q_rp + 1;
      end
      // requests: a taken one is recorded; a new one follows (sequential half the time)
      if (rq_v & rd_ack) begin
         q_va[q_wp] <= rq_va;  q_pa[q_wp] <= rq_pa;  q_tag[q_wp] <= rq_tag;  q_old[q_wp] <= win_at(rq_pa);  q_wp <= q_wp + 1;
         q_last[q_wp] <= rq_last;
         if ((rq_va[5:4] == 2'd3) && ({1'b0, rq_va[3:1]} + {1'b0, rq_last} > 4'd7)) nx <= nx + 1;
         q_st[q_wp] <= 1'b0;
         nreq <= nreq + 1;
      end
      q_n <= q_n + {4'd0, rq_v & rd_ack} - {4'd0, rd_valid};
      if (rq_v & ~rd_ack & ep_bump) rq_pa <= {45'd0, ptab[rq_va[19:12]], rq_va[11:0]};
      if (~rq_v | rd_ack) begin
         rq_v <= 1'b0;
         if (~quiesce && q_n < 12 && ($urandom % 8) != 0) begin
            va_n = (rq_v && ($urandom % 2)) ? rq_va + 64'({rq_last, 1'b0}) + 64'd2 : rnd_va(0);
            if (va_n[63:12] != rq_va[63:12]) va_n = rnd_va(0);
            // a fifth of the windows start in a line's last pair and reach the next line
            if (($urandom % 5) == 0) begin va_n[5:4] = 2'd3;  if (va_n[3:1] == 3'd0) va_n[3:1] = 3'd1; end
            last_n = rnd_last(va_n);
            if (va_n[5:4] == 2'd3 && ({1'b0, va_n[3:1]} + {1'b0, last_n} <= 4'd7) && va_n[11:6] != 6'h3f && ($urandom % 2))
               last_n = 3'd7;
            pp = ptab[va_n[19:12]];
            rq_v <= 1'b1;  rq_va <= va_n;  rq_last <= last_n;  rq_pa <= {45'd0, pp, va_n[11:0]};  rq_tag <= rq_tag + 1;
         end
      end
      // a store: a line (in the hot region half the time) rewritten and probed, nothing drained
      pb_v <= 1'b0;
      // a third are aimed at the line the cache is filling or has prefetched (rule G6: aimed)
      if (!quiesce && ($urandom % 150) == 0) begin
         pb_va = rnd_va(0);
         pb_line = {45'd0, ptab[pb_va[19:12]], pb_va[11:6]};
         if (($urandom % 3) == 0) begin
            if (dut.st == 3'd1 || dut.st == 3'd2) pb_line = dut.f_pline;
            else if (dut.pf_infl)                 pb_line = dut.pf_ia;
            else if (dut.pf_val)                  pb_line = dut.pf_addr;
         end
         pb_pa <= {pb_line, 6'd0};
         for (k = 0; k < 8; k = k + 1) mem[{pb_line[12:0], k[2:0]}] = {$urandom, $urandom};
         pb_v <= 1'b1;  npb <= npb + 1;
      end
      if (dut.pb_fill) nfdead <= nfdead + 1;
      if (dut.miss & dut.hit_l & ~dut.s1_rp) nmx <= nmx + 1;     // a first lookup missing the next line only
      if (dut.st == 4'd10 && (dut.rc0 | dut.rc1)) nrcn <= nrcn + 1; // the next line reconciled
      if (pb_v & dut.pf_val & (dut.pf_addr == pb_pa[63:6])) npfk <= npfk + 1;
      // THE FILL HOLD: windows of hold, a cancel now and then (half of them inside a window, aimed
      // at a held miss). A demand read must never leave while the hold is up; a cancel makes every
      // request then outstanding stale, its answer still owed but its data the requester's to drop.
      // (l2_req and f_l2 rise at the edge the read is decided, on the hold of the cycle before it)
      hold_q <= fill_hold;
      if (dut.l2_req & dut.f_l2 & ~dut.pf_infl & hold_q) begin
         $display("FAIL: a demand line read left under the fill hold (cycle %0d)", cyc); $finish;
      end
      if (fill_cancel) begin
         q_st <= 16'hffff;  ncan <= ncan + 1;
         if (dut.st == 4'd1 && !dut.f_l2) ncanq <= ncanq + 1;
      end
      if (dut.st == 4'd1 && fill_hold && !dut.f_l2) nhold <= nhold + 1;
      fill_cancel <= 1'b0;
      if (!quiesce) begin
         if (($urandom % 400) == 0) fill_hold <= ~fill_hold;
         if ((fill_hold && dut.st == 4'd1 && ($urandom % 20) == 0) || ($urandom % 3000) == 0) fill_cancel <= 1'b1;
      end else fill_hold <= 1'b0;
      // a mapping change: a new page table, the epoch advanced, nothing drained
      ep_bump <= 1'b0;
      if (!quiesce && ($urandom % 6000) == 0) begin ep_bump <= 1'b1; nep <= nep + 1; end
   end

   // ---- fence.i: drain, change code, invalidate, wait ----
   integer n, m;
   reg nodrain;
   initial nodrain = $test$plusargs("nodrain");
   initial begin
      for (k = 0; k < PLINES*8; k = k + 1) mem[k] = {$urandom, $urandom};
      for (k = 0; k < NVP; k = k + 1) ptab[k] = 7'($urandom);
      rq_v = 0;  rq_tag = 0;
      repeat (4) @(posedge clk);  reset = 0;
      while (cyc < NCYC) begin
         repeat (15000 + ($urandom % 10000)) @(posedge clk);
         quiesce = 1;
         // +nodrain: the SoC's fence.i freezes the requester but does not wait for its requests
         if (nodrain) repeat ($urandom % 4) @(posedge clk);
         else while (q_n != 0 || rq_v) @(posedge clk);
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
      if (ncanq == 0 || nhold == 0) begin $display("FAIL: the hold never held a miss (%0d cycles) or a cancel never met one (%0d)", nhold, ncanq); $finish; end
      if (nfdead == 0 || npfk == 0) begin $display("FAIL: a probe never met a fill in flight (%0d) or a prefetched line (%0d)", nfdead, npfk); $finish; end
      if (nx == 0 || nmx == 0 || nrcn == 0) begin $display("FAIL: no window reached the next line (%0d), missed there alone (%0d) or reconciled it (%0d)", nx, nmx, nrcn); $finish; end
      $display("ICACHE-TB PASS LAT=%0d: %0d requests answered, %0d taken, %0d reaching the next line, %0d missed on first lookup (%0d on the next line alone, %0d reconciled, %0d of them the next line), %0d fence.i, %0d mapping changes, %0d probes (%0d fills killed, %0d prefetches killed), %0d cycles a miss held, %0d cancels (%0d of a held miss)",
               LAT, nresp, nacc, nx, nmiss, nmx, nrc, nrcn, nfi, nep, npb, nfdead, npfk, nhold, ncan, ncanq);
      $finish;
   end
endmodule
`default_nettype wire
