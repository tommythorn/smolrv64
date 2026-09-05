`default_nettype none
// rv_cache as the D$ -- WRITABLE=1, WRTHRU=0. THE COMPANION TO tb_ooo2_cache.v, WHICH
// INSTANTIATES WRITABLE=0 AND THEREFORE PROVES NOTHING ABOUT THIS INSTANCE.
//
// That distinction cost a bitstream on 2026-09-01. tb_ooo2_cache.v passed all 41 arrival
// cycles, lint was clean and the Linux lockstep verified 14.6 M retires -- and the board
// Oopsed 38 times. It ties wr_req/cbo_req/cbo_zero/rd_uncached to zero and sets
// WRITABLE=0, which compiles the dirty-writeback path out entirely, so it exercised no
// store read-modify-write, no cbo.zero (the kernel's clear_page; Zicboz is advertised) and
// no eviction. rv_cache is ONE MODULE IN TWO ROLES and a pass in one is not a pass in the
// other. Anything that changes the cache runs BOTH.
//
// Sweep `LAT` (the L2 line latency): the board is ~100 cycles through the MIG and the
// shared arbiter, and window length is what makes concurrency defects reachable -- the
// original hit-under-miss bug needed it. -DLAT=4/20/100/200 all pass today. That test ties wr_req/cbo_req/cbo_zero to
// zero and sets WRITABLE=0, which compiles the dirty-writeback path out entirely, so it can
// say nothing about a store's read-modify-write, about cbo.zero, or about eviction.
//
// The hypothesis under test: a store captures a chunk into wlo/whi, merges its bytes, and
// writes the WHOLE chunk back. A bad capture therefore commits zeros to the bytes the store
// never wrote -- persistent, surviving re-reads, which fits 37 identical badaddr values on
// the board where a transient read error does not. Eviction is included because a dirty bit
// lost to a same-cycle write would drop the line silently and read back the pre-store value.
module tb;
   localparam PAW=64, RDW=64, LINEB=512, OFFB=6;
`ifndef LAT
 `define LAT 20
`endif
`ifndef JIT
 `define JIT 0
`endif
   localparam LAT = `LAT;
   localparam JIT = `JIT;
   reg clk=0, reset=1;
   always #5 clk = ~clk;

   reg              req_pend=0;
   wire             rd_req = req_pend;
   reg  [PAW-1:0]   rd_addr=0;
   reg  [3:0]       rd_tag=0;
   wire [RDW-1:0]   rd_data;
   wire             rd_valid, rd_ack;
   wire [3:0]       rd_resp_tag;
   reg              cbo_req=0, cbo_zero=0, cbo_keep=0, rd_unc=0, wr_unc=0;
   reg              wr_req=0;
   reg  [PAW-1:0]   wr_addr=0;
   reg  [63:0]      wr_data=0;
   reg  [7:0]       wr_mask=0;
   wire             wr_ack, wr_acc;
   wire             l2_req, l2_we;
   wire [PAW-OFFB-1:0] l2_addr;
   wire [LINEB-1:0] l2_wdata;
   reg  [LINEB-1:0] l2_rdata=0;
   reg              l2_ack=0;
   always @(posedge clk) if (reset) req_pend<=1'b0; else if (rd_ack) req_pend<=1'b0;

   rv_cache #(.PAW(PAW), .SIZE_KB(128), .RDW(RDW), .WDW(64),
              .WRITABLE(1), .WRTHRU(0), .PREFETCH(0), .PERF_ID(1)) dut
     (.clk(clk), .reset(reset),
      .rd_req(rd_req), .rd_addr(rd_addr), .rd_data(rd_data), .rd_valid(rd_valid),
      .rd_resp_addr(), .rd_tag(rd_tag), .rd_resp_tag(rd_resp_tag),
      .rd_ack(rd_ack), .rd_uncached(rd_unc),
      .wr_req(wr_req), .wr_addr(wr_addr), .wr_data(wr_data), .wr_mask(wr_mask),
      .wr_ack(wr_ack), .wr_acc(wr_acc), .wr_cpl(), .wr_uncached(wr_unc),
      .cbo_req(cbo_req), .cbo_zero(cbo_zero), .cbo_keep(cbo_keep),
      .inv_req(1'b0), .inv_clean(1'b0), .inv_busy(),
      .l2_req(l2_req), .l2_we(l2_we), .l2_addr(l2_addr), .l2_wdata(l2_wdata),
      .l2_rdata(l2_rdata), .l2_ack(l2_ack), .perf_access(), .perf_miss());

   // Backing store: a real memory, so a WRITEBACK that streams the wrong bytes is caught on
   // the next fill of that line rather than being invisible.
   localparam NL = 4096;
   reg [LINEB-1:0] mem [0:NL-1];
   integer mi, w;
   initial for (mi=0;mi<NL;mi=mi+1)
      for (w=0;w<8;w=w+1) mem[mi][w*64 +: 64] = {mi[31:0], w[2:0], 1'b0, 28'h5A5A5A5};

   integer seed = 1;
   integer lcnt; reg lbusy=0; reg [PAW-OFFB-1:0] laddr; reg lwe; reg [LINEB-1:0] lwd;
   always @(posedge clk) begin
      l2_ack <= 1'b0;
      if (reset) lbusy <= 1'b0;
      else if (!lbusy && l2_req) begin
         // VARIABLE, because the real one is: DDR4 through the MIG varies with refresh,
         // bank conflicts and arbitration against the I$. A FIXED latency explores exactly
         // ONE interleaving of fill against lookup, so a race needing any other is invisible
         // however long the sweep. LAT is the floor, JIT the jitter on top.
         lbusy<=1'b1; lcnt<=LAT + ({$random(seed)} % (JIT+1)); laddr<=l2_addr;
         lwe<=l2_we; lwd<=l2_wdata;
      end else if (lbusy) begin
         if (lcnt==0) begin
            if (lwe) mem[laddr[11:0]] <= lwd; else l2_rdata <= mem[laddr[11:0]];
            l2_ack <= 1'b1; lbusy <= 1'b0;
         end else lcnt <= lcnt-1;
      end
   end

   // mem is indexed by laddr[11:0], so a word's identity is the TRUNCATED line index.
   function [63:0] expect_word(input [PAW-1:0] a);
      expect_word = {20'd0, a[17:6], a[5:3], 1'b0, 28'h5A5A5A5};
   endfunction

   reg [63:0] gotv [0:15];
   reg [15:0] gotq;
   initial gotq = 0;
   always @(posedge clk) if (!reset && rd_valid) begin
      gotv[rd_resp_tag] <= rd_data; gotq[rd_resp_tag] <= 1'b1;
   end
   task issue_nb(input [PAW-1:0] a, input [3:0] t);   // issue, do not wait for the answer
      begin rd_addr=a; rd_tag=t; req_pend=1'b1; while (req_pend) @(negedge clk); end
   endtask

   integer errors, i, k, g14, tcyc;
   initial tcyc = 0;
   always @(posedge clk) tcyc <= tcyc + 1;
   reg [63:0] got;
   reg [PAW-1:0] A;

   task do_store(input [PAW-1:0] a, input [63:0] d, input [7:0] m);
      begin
         wr_addr=a; wr_data=d; wr_mask=m; wr_req=1'b1;
         @(negedge clk); while (!wr_ack) @(negedge clk);
         wr_req=1'b0; @(negedge clk);
      end
   endtask
   task do_load(input [PAW-1:0] a, input [3:0] t);
      begin
         rd_addr=a; rd_tag=t; req_pend=1'b1;
         while (req_pend) @(negedge clk);
         i=0; while (!rd_valid && i<(8*LAT+400)) begin @(negedge clk); i=i+1; end
         if (i>=(8*LAT+400)) begin $display("FAIL: load timed out at a=%h", a); errors=errors+1; end
         got = rd_data;
         @(negedge clk);
      end
   endtask
   // A store presented until the D$ TAKES it. wr_acc is combinational: seen once the inputs
   // have settled, it is what the next posedge latches, so the request is dropped only after
   // that edge. Sampling it at the FOLLOWING negedge misses an accept that already happened
   // and then sees a second accept of the same store at the next open door (the phantom that
   // lost T16's store while the bench believed it had been taken, 2026-09-05).
   task store_go(input [PAW-1:0] a, input [63:0] d, input [7:0] m);
      begin
         wr_addr=a; wr_data=d; wr_mask=m; wr_req=1'b1;
         #1; while (!wr_acc) begin @(negedge clk); #1; end
         @(posedge clk); #1; wr_req=1'b0;
      end
   endtask
   // A store, then a load presented from the store's S_CHECK cycle on; `gap` counts the
   // negedges until the load's ack: 2 through the door in S_FIN, 3 through S_IDLE.
   task store_then_load(input [PAW-1:0] sa, input [63:0] d, input [PAW-1:0] la, input [3:0] t, output integer gap);
      begin
         store_go(sa, d, 8'hFF);
         @(negedge clk);
         rd_addr=la; rd_tag=t; req_pend=1'b1; gap=0;
         while (req_pend) begin @(negedge clk); gap=gap+1; end
         i=0; while (!rd_valid && i<(8*LAT+400)) begin @(negedge clk); i=i+1; end
         if (i>=(8*LAT+400)) begin $display("FAIL: load timed out at a=%h", la); errors=errors+1; end
         got = rd_data;
         @(negedge clk);
      end
   endtask
   // cbo.zero rides the WRITE port: wr_req and cbo_req together, acked by wr_ack.
   task do_cbo_zero(input [PAW-1:0] a);
      begin
         wr_addr=a; wr_data=64'd0; wr_mask=8'd0; cbo_zero=1'b1; cbo_req=1'b1; wr_req=1'b1;
         @(negedge clk); while (!wr_ack) @(negedge clk);
         wr_req=1'b0; cbo_req=1'b0; cbo_zero=1'b0; @(negedge clk);
      end
   endtask
   // Zicbom clean (keep=1) / flush+inval (keep=0): the write port with cbo_req, no data.
   task do_cbo(input [PAW-1:0] a, input keep);
      begin
         wr_addr=a; wr_data=64'd0; wr_mask=8'd0; cbo_keep=keep; cbo_req=1'b1; wr_req=1'b1;
         @(negedge clk); while (!wr_ack) @(negedge clk);
         wr_req=1'b0; cbo_req=1'b0; cbo_keep=1'b0; @(negedge clk);
      end
   endtask
   // Svpbmt NC/IO accesses: the same ports with the uncached bit up.
   task do_load_nc(input [PAW-1:0] a, input [3:0] t);
      begin rd_unc=1'b1; do_load(a, t); rd_unc=1'b0; end
   endtask
   task do_store_nc(input [PAW-1:0] a, input [63:0] d, input [7:0] m);
      begin wr_unc=1'b1; do_store(a, d, m); wr_unc=1'b0; end
   endtask
   // A DEVICE writes memory behind the cache (virtio DMA): straight into the backing store,
   // while the cache is idle, one 64-bit word of one line.
   task dma_write(input [PAW-1:0] a, input [63:0] d);
      begin
         @(negedge clk); while (lbusy) @(negedge clk);
         mem[a[17:6]][a[5:3]*64 +: 64] = d;
         @(negedge clk);
      end
   endtask
   task expect64(input [63:0] g, input [63:0] e, input [255:0] what);
      if (g !== e) begin
         $display("FAIL %0s: got %h want %h", what, g, e); errors=errors+1;
      end
   endtask

   initial begin
      errors=0;
      repeat (8) @(negedge clk); reset=0; @(negedge clk);

      // ---- T1: a store must not disturb the other words of its chunk or its line -------
      A = 64'h8000_0000 + 64'h4000;
      do_load(A, 4'h1);                                  // fill the line
      expect64(got, expect_word(A), "prime word0");
      do_store(A + 64'd8, 64'hDEAD_BEEF_1234_5678, 8'hFF);
      do_load(A + 64'd8, 4'h2);
      expect64(got, 64'hDEAD_BEEF_1234_5678, "the stored word");
      for (k=0; k<8; k=k+1) if (k != 1) begin           // every OTHER word of the line
         do_load(A + k*8, 4'h3);
         expect64(got, expect_word(A + k*8), "neighbour word after store");
      end

      // ---- T2: a BYTE store must not zero the rest of its 8-byte chunk ----------------
      do_store(A + 64'd16, 64'h0000_0000_0000_00AA, 8'h01);   // one byte
      do_load(A + 64'd16, 4'h4);
      expect64(got, (expect_word(A + 64'd16) & ~64'hFF) | 64'hAA,
               "byte store must preserve the other 7 bytes");

      // ---- T3: the store must SURVIVE eviction, i.e. be written back ------------------
      // Touch enough distinct lines in the same set to evict, then come back.
      for (k=1; k<=6; k=k+1) do_load(A + (k<<16), 4'h5);      // same index, different tags
      do_load(A + 64'd8, 4'h6);
      expect64(got, 64'hDEAD_BEEF_1234_5678, "stored word after eviction+refill");

      // ---- T4: cbo.zero zeroes ITS line, and ONLY its line ---------------------------
      // clear_page uses this on every page the kernel hands out (Zicboz is advertised), and
      // it is the one path whose whole purpose is writing zeros -- so a mis-targeted line
      // is literally a mechanism for the persistent zeros seen on the board.
      begin : cbo_tests
         reg [PAW-1:0] Z, NXT, SAME;
         Z    = 64'h8000_0000 + 64'h8000;      // the victim of the cbo.zero
         NXT  = Z + 64'd64;                    // the adjacent line
         SAME = Z + (64'd1 << 16);             // same base index, different tag
         do_load(Z,   4'h8); expect64(got, expect_word(Z),   "prime Z");
         do_load(NXT, 4'h9); expect64(got, expect_word(NXT), "prime NXT");
         do_load(SAME,4'hA); expect64(got, expect_word(SAME),"prime SAME");

         do_cbo_zero(Z);

         for (k=0; k<8; k=k+1) begin
            do_load(Z + k*8, 4'hB);
            expect64(got, 64'd0, "cbo.zero must zero every word of its own line");
         end
         for (k=0; k<8; k=k+1) begin
            do_load(NXT + k*8, 4'hC);
            expect64(got, expect_word(NXT + k*8), "cbo.zero must not touch the NEXT line");
         end
         for (k=0; k<8; k=k+1) begin
            do_load(SAME + k*8, 4'hD);
            expect64(got, expect_word(SAME + k*8), "cbo.zero must not touch a same-index line");
         end

         // ---- T5: the zeros are DIRTY and must survive eviction --------------------
         // cbo.zero installs a line L2 has never seen, so if the dirty bit or the
         // writeback is wrong the eviction silently restores the pre-zero contents.
         for (k=1; k<=6; k=k+1) do_load(Z + (k<<20), 4'hE);
         do_load(Z, 4'hF);
         expect64(got, 64'd0, "cbo.zero must survive eviction and refill");

         // ---- T6: cbo.zero on a line that is NOT resident (the miss path) ----------
         do_cbo_zero(64'h8000_0000 + 64'hC000);
         do_load(64'h8000_0000 + 64'hC000, 4'h1);
         expect64(got, 64'd0, "cbo.zero on a miss must zero the allocated line");
      end

      // ---- T7: a second read during a fill that is PRECEDED BY A DIRTY WRITEBACK -----
      // The D$-only window. fill_banks covers F_WBR/F_WBW but not F_WBI/F_WBA, so the
      // pipeline accepts during the L2 write round trip -- ~20 cycles here, ~100 on the
      // board. WRITABLE=0 has no writeback at all, which is why the I$-shaped test could
      // never reach this. Swept over where in that window the second read lands.
      begin : wb_concurrency
         reg [PAW-1:0] V, F, R;
         for (k=0; k<44; k=k+1) begin
            V = 64'h8000_0000 + 64'h10000 + (k<<8);   // made dirty, then evicted
            F = V + (64'd1 << 17);                    // maps to V's set, forces the evict
            R = 64'h8000_0000 + 64'h20000 + (k<<8);   // an unrelated resident line
            gotq = 0;
            do_load(V, 4'h1);                                   // V resident
            do_store(V + 64'd24, 64'hCAFE_F00D_0000_0000 + k, 8'hFF);   // V now DIRTY
            do_load(R, 4'h2);                                   // R resident
            issue_nb(F, 4'h3);                                  // miss -> writeback V, fill F
            repeat (k) @(negedge clk);                          // ...land the 2nd read here
            issue_nb(R, 4'h4);                                  // a hit, mid-writeback
            i=0; while ((!gotq[3] || !gotq[4]) && i<(8*LAT+600)) begin @(negedge clk); i=i+1; end
            if (!gotq[3] || !gotq[4]) begin
               $display("FAIL k=%0d: response lost (v3=%b v4=%b)", k, gotq[3], gotq[4]);
               errors=errors+1; k=44;
            end else begin
               if (gotv[4] !== expect_word(R))
                  begin $display("FAIL k=%0d: hit under writeback got %h want %h", k, gotv[4], expect_word(R)); errors=errors+1; end
               if (gotv[3] !== expect_word(F))
                  begin $display("FAIL k=%0d: the missing read got %h want %h", k, gotv[3], expect_word(F)); errors=errors+1; end
            end
            // and V's dirty word must have reached memory intact
            do_load(V + 64'd24, 4'h5);
            if (got !== (64'hCAFE_F00D_0000_0000 + k))
               begin $display("FAIL k=%0d: evicted dirty word got %h want %h", k, got, 64'hCAFE_F00D_0000_0000+k); errors=errors+1; end
            if (errors > 6) k=44;
         end
      end

      // ---- T8-T13: DMA COHERENCE, the board's contract that no boot exercised in sim. ----
      // Linux with Svpbmt maps DMA-coherent memory NC and does cbo.clean/inval around
      // streaming DMA. On 2026-09-04 the board read stale virtio rings (the queue lost the NC
      // bit, not the cache) -- and nothing here could have said which. These pin the cache's
      // side: an NC read never keeps a line, an NC store reaches memory and keeps no line,
      // cbo.inval drops a clean line, cbo.flush writes back and drops, cbo.clean writes
      // back and keeps.
      begin : dma_coherence
         reg [PAW-1:0] X;
         X = 64'h8000_0000 + 64'h30000;
         // T8: a resident line, the device rewrites it, cbo.inval, then an NC read
         do_load(X, 4'h1);  expect64(got, expect_word(X), "T8 prime");
         dma_write(X, 64'hD0D0_0001_0000_0001);
         do_cbo(X, 1'b0);                                     // inval (clean line: no writeback)
         do_load_nc(X, 4'h2);
         expect64(got, 64'hD0D0_0001_0000_0001, "T8 NC read after cbo.inval sees the device's word");
         // T9: an NC read must not keep the line: the device rewrites, the next NC read sees it
         dma_write(X, 64'hD0D0_0002_0000_0002);
         do_load_nc(X, 4'h3);
         expect64(got, 64'hD0D0_0002_0000_0002, "T9 second NC read sees the newer device word (line not kept)");
         dma_write(X, 64'hD0D0_0003_0000_0003);
         do_load(X, 4'h4);
         expect64(got, 64'hD0D0_0003_0000_0003, "T9 a cacheable read after NC reads fetches from memory");
         // T10: cbo.clean of a DIRTY line writes it back and keeps it
         do_store(X + 64'd8, 64'hC1EA_0000_0000_0010, 8'hFF);
         do_cbo(X, 1'b1);                                     // clean
         if (mem[X[17:6]][1*64 +: 64] !== 64'hC1EA_0000_0000_0010)
            begin $display("FAIL T10 cbo.clean did not write the dirty word back"); errors=errors+1; end
         do_load(X + 64'd8, 4'h5);
         expect64(got, 64'hC1EA_0000_0000_0010, "T10 the line is still readable after clean");
         // T11: cbo.flush of a dirty line writes back AND drops it: a device word lands after
         do_store(X + 64'd16, 64'hF1A5_0000_0000_0020, 8'hFF);
         do_cbo(X, 1'b0);                                     // flush/inval
         if (mem[X[17:6]][2*64 +: 64] !== 64'hF1A5_0000_0000_0020)
            begin $display("FAIL T11 cbo.flush did not write the dirty word back"); errors=errors+1; end
         dma_write(X + 64'd24, 64'hD0D0_0004_0000_0004);
         do_load(X + 64'd24, 4'h6);
         expect64(got, 64'hD0D0_0004_0000_0004, "T11 after flush the line is refetched (device word visible)");
         // T12: cbo.inval on a CLEAN resident line drops it
         do_load(X + 64'd32, 4'h7);
         dma_write(X + 64'd32, 64'hD0D0_0005_0000_0005);
         do_cbo(X, 1'b0);
         do_load(X + 64'd32, 4'h8);
         expect64(got, 64'hD0D0_0005_0000_0005, "T12 cbo.inval of a clean line: the next read refetches");
         // T13: an NC store reaches memory at once and keeps no line
         do_store_nc(X + 64'd40, 64'h5C5C_0000_0000_0050, 8'hFF);
         if (mem[X[17:6]][5*64 +: 64] !== 64'h5C5C_0000_0000_0050)
            begin $display("FAIL T13 NC store not in memory after ack"); errors=errors+1; end
         dma_write(X + 64'd48, 64'hD0D0_0006_0000_0006);
         do_load(X + 64'd48, 4'h9);
         expect64(got, 64'hD0D0_0006_0000_0006, "T13 after an NC store the line was not kept");
      end

      // ---- T14: THE DOOR IN A WRITE'S LAST CYCLE (plan item 4b, 2026-09-05). A request into
      // another set is accepted while the store's chunk is being written (S_FIN admits it);
      // one into the SAME set waits for S_IDLE, because its bank read would collide with the
      // write and return the old chunk. Both read back what was stored; the gap is what proves
      // which door it went through: the load is presented from the write's S_CHECK cycle, and
      // the tb counts negedges until it sees the ack, so an accept in S_FIN counts 2 and one
      // in S_IDLE counts 3. ----
      A = 64'h0000_0000_0010_0000;
      do_store(A,          64'h1111_1111_1111_1111, 8'hFF);       // both lines resident
      do_store(A + 64'd64, 64'h2222_2222_2222_2222, 8'hFF);
      store_then_load(A, 64'hAAAA_0001_AAAA_0001, A + 64'd64, 4'd3, g14);
      expect64(got, 64'h2222_2222_2222_2222, "T14 a load of the NEXT set behind a store");
      if (g14 != 2) begin $display("FAIL T14 next-set load: gap %0d, expected 2 (the door in S_FIN)", g14); errors=errors+1; end
      store_then_load(A + 64'd64, 64'hBBBB_0002_BBBB_0002, A + 64'd64, 4'd4, g14);
      expect64(got, 64'hBBBB_0002_BBBB_0002, "T14 a load of the line JUST stored");
      if (g14 != 3) begin $display("FAIL T14 same-set load: gap %0d, expected 3 (a bank collision is refused, S_IDLE)", g14); errors=errors+1; end
      do_load(A, 4'd5); expect64(got, 64'hAAAA_0001_AAAA_0001, "T14 the first store landed");

      // ---- T15-T17: A PLAIN WRITE IS NOT SOLO (plan item 4c, 2026-09-05). Its miss is
      // completed by the fill machine -- merged as the line lands, installed dirty -- so it
      // is accepted under a fill and blocks nothing while its own fill runs. ----
      begin : wr_under_fill
         reg [PAW-1:0] B, C, D, E;
         integer t0;
         B = 64'h8005_0C40;  C = B + 64'd64;  D = B + 64'd128;  E = B + 64'd192;   // fresh lines, four sets
         // T15: a store MISS, byte-masked, merged as the line lands; the neighbour word is
         // the line's; the line installed DIRTY, so it survives eviction.
         do_store(B + 64'd8, 64'hCC00_0000_0000_00DD, 8'h81);
         do_load(B + 64'd8, 4'h1);
         expect64(got, (expect_word(B + 64'd8) & 64'h00FF_FFFF_FFFF_FF00) | 64'hCC00_0000_0000_00DD, "T15 store miss merged as the line landed");
         do_load(B + 64'd16, 4'h2); expect64(got, expect_word(B + 64'd16), "T15 the neighbour word is the line's");
         for (k=1; k<=6; k=k+1) do_load(B + (k<<16), 4'h3);                     // evict it
         do_load(B + 64'd8, 4'h4);
         expect64(got, (expect_word(B + 64'd8) & 64'h00FF_FFFF_FFFF_FF00) | 64'hCC00_0000_0000_00DD, "T15 the merged line installed dirty and came back from L2");
         // T16: a store accepted UNDER a read's fill, and a hit read accepted under the
         // store's own fill. Blocked would mean waiting out a fill (LAT and more).
         gotq[6] = 1'b0;
         issue_nb(C, 4'h6);                                                    // a read miss: fill in flight
         t0 = tcyc; store_go(D, 64'h0D0D_0D0D_0D0D_0D0D, 8'hFF);
         if (tcyc - t0 > LAT/2 + 8) begin $display("FAIL T16 the store waited %0d cycles under the read's fill (LAT=%0d): still solo", tcyc - t0, LAT); errors=errors+1; end
         i=0; while (!gotq[6] && i<(8*LAT+400)) begin @(negedge clk); i=i+1; end
         expect64(gotv[6], expect_word(C), "T16 the read miss the store went under");
         t0 = tcyc; do_load(B + 64'd8, 4'h7);                                  // a hit while the store's fill runs
         expect64(got, (expect_word(B + 64'd8) & 64'h00FF_FFFF_FFFF_FF00) | 64'hCC00_0000_0000_00DD, "T16 a hit read under the store's fill");
         if (tcyc - t0 > LAT/2 + 8) begin $display("FAIL T16 the hit read took %0d cycles under the store's fill (LAT=%0d)", tcyc - t0, LAT); errors=errors+1; end
         for (i=0; i<3*LAT+100; i=i+1) @(negedge clk);                          // let the store's fill land
         do_load(D, 4'h8); expect64(got, 64'h0D0D_0D0D_0D0D_0D0D, "T16 the store that went under the fill");
         // T17: two stores to the SAME missing line: the second holds under the first's
         // fill, then hits and merges.
         store_go(E, 64'h1E1E_1E1E_1E1E_1E1E, 8'hFF);
         @(negedge clk);
         do_store(E + 64'd8, 64'h2E2E_2E2E_2E2E_2E2E, 8'hFF);                  // accepted at once; holds until E lands
         do_load(E, 4'h9);          expect64(got, 64'h1E1E_1E1E_1E1E_1E1E, "T17 the first store, merged by the fill");
         do_load(E + 64'd8, 4'hA);  expect64(got, 64'h2E2E_2E2E_2E2E_2E2E, "T17 the second store, merged after its hold");
         do_load(E + 64'd16, 4'hB); expect64(got, expect_word(E + 64'd16), "T17 the rest of the line");
      end

      if (errors==0) $display("rv_cache D$ directed: PASS");
      else           $display("rv_cache D$ directed: FAIL (%0d errors)", errors);
      $finish;
   end
endmodule
`default_nettype wire
