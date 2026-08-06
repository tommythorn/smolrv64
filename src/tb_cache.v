`timescale 1ns/1ps
`default_nettype none

// Unit TB for cache.v (unified skewed-2-way PIPT L1). Drives the D$ instance
// through hit / miss-fill / write / dirty-eviction-writeback / line-crossing /
// flush, checking every read against a golden byte memory `refm`, and checking
// writeback correctness by comparing the behavioral L2 (`l2mem`) to `refm` after a
// flush. Then a brief I$-mode (WRITABLE=0, 128-bit read) check.
//
// The read port is a ready/valid request channel (rd_req/rd_rdy: accepted where
// both are high, retract/re-address freely before that) plus an address-tagged
// response (rd_valid/rd_resp_addr). The pipelined-stream phases at the end present
// the next read the cycle after each accept: an all-hit stream -- including
// same-address back-to-back -- must be accepted at one hit/cycle, and a miss/span/
// NC mid-stream must drain through the FSM and keep responses in accept order.
module tb;
   localparam PAW = 34, LINEB = 512, OFFB = 6, L2LAT = 2;
   localparam MEM = 'h30000;                       // test memory (covers distinct-tag set probes)

   reg clk=0; always #5 clk=~clk;
   reg reset;

   reg  [7:0] l2mem [0:MEM-1];                     // backing store (== refm after flush)
   reg  [7:0] refm   [0:MEM-1];                     // golden architectural memory
   integer k, j2, cmb, errs=0, before_reads;

   // ---------------- D$ instance ----------------
   reg          d_rd_req, d_wr_req, d_inv_req;
   reg          d_rd_uncached=0, d_wr_uncached=0;   // Svpbmt NC/IO qualifiers
   reg          d_cbo_req=0, d_cbo_zero=0, d_cbo_keep=0;   // Zicbom/Zicboz
   reg  [PAW-1:0] d_rd_addr, d_wr_addr;
   reg  [63:0]  d_wr_data;  reg [7:0] d_wr_mask;
   wire [63:0]  d_rd_data;  wire d_rd_valid, d_wr_ack, d_inv_busy, d_rd_rdy;
   wire [PAW-1:0] d_rd_resp_addr;
   wire         d_l2_req, d_l2_we;  wire [PAW-OFFB-1:0] d_l2_addr;
   wire [LINEB-1:0] d_l2_wdata;  wire [LINEB/8-1:0] d_l2_wmask;
   reg [LINEB-1:0] d_l2_rdata;  reg d_l2_ack;

   cache #(.PAW(PAW), .SIZE_KB(128), .RDW(64), .WDW(64), .WRITABLE(1)) u_d
     (.clk(clk), .reset(reset),
      .rd_req(d_rd_req), .rd_rdy(d_rd_rdy), .rd_addr(d_rd_addr), .rd_data(d_rd_data), .rd_valid(d_rd_valid),
      .rd_resp_addr(d_rd_resp_addr),
      .wr_req(d_wr_req & ~d_wr_ack), .wr_addr(d_wr_addr), .wr_data(d_wr_data), .wr_mask(d_wr_mask),
      .wr_ack(d_wr_ack), .rd_uncached(d_rd_uncached), .wr_uncached(d_wr_uncached),
      .cbo_req(d_cbo_req), .cbo_zero(d_cbo_zero), .cbo_keep(d_cbo_keep),
      .inv_req(d_inv_req), .inv_clean(1'b0), .inv_busy(d_inv_busy),
      .l2_req(d_l2_req), .l2_we(d_l2_we), .l2_addr(d_l2_addr), .l2_wdata(d_l2_wdata),
      .l2_wmask(d_l2_wmask), .l2_rdata(d_l2_rdata), .l2_ack(d_l2_ack));

   // D$ behavioral L2 (reads + writes l2mem, L2LAT cycles)
   reg dbusy; reg [3:0] dcnt; reg dwe_q; reg [PAW-OFFB-1:0] dad_q; reg [LINEB-1:0] dwd_q;
   reg [LINEB/8-1:0] dwm_q;
   integer d_l2reads = 0;                          // count D$ L2 line reads (detect refill-on-miss)
   always @(posedge clk) begin
      d_l2_ack <= 0;
      if (reset) dbusy <= 0;
      else if (!dbusy && d_l2_req) begin
         dbusy<=1; dcnt<=L2LAT; dwe_q<=d_l2_we; dad_q<=d_l2_addr; dwd_q<=d_l2_wdata; dwm_q<=d_l2_wmask;
      end else if (dbusy) begin
         if (dcnt==0) begin
            if (dwe_q) for (k=0;k<64;k=k+1) l2mem[(dad_q<<OFFB)+k] <= dwm_q[k] ? dwd_q[k*8 +: 8] : l2mem[(dad_q<<OFFB)+k];
            else begin for (k=0;k<64;k=k+1) d_l2_rdata[k*8 +: 8] <= l2mem[(dad_q<<OFFB)+k];
                       d_l2reads <= d_l2reads + 1; end
            d_l2_ack <= 1; dbusy <= 0;
         end else dcnt <= dcnt-1;
      end
   end

   // ---------------- I$ instance (read-only, 128-bit) ----------------
   reg          i_rd_req, i_inv_req;  reg [PAW-1:0] i_rd_addr;
   wire [127:0] i_rd_data;  wire i_rd_valid, i_inv_busy, i_rd_rdy;
   wire         i_l2_req, i_l2_we;  wire [PAW-OFFB-1:0] i_l2_addr;
   wire [LINEB-1:0] i_l2_wdata;  reg [LINEB-1:0] i_l2_rdata;  reg i_l2_ack;
   cache #(.PAW(PAW), .SIZE_KB(64), .RDW(128), .WDW(64), .WRITABLE(0)) u_i
     (.clk(clk), .reset(reset),
      .rd_req(i_rd_req), .rd_rdy(i_rd_rdy), .rd_addr(i_rd_addr), .rd_data(i_rd_data), .rd_valid(i_rd_valid),
      .wr_req(1'b0), .wr_addr(34'd0), .wr_data(64'd0), .wr_mask(8'd0),
      .wr_ack(), .rd_uncached(1'b0), .wr_uncached(1'b0),
      .cbo_req(1'b0), .cbo_zero(1'b0), .cbo_keep(1'b0), .inv_req(i_inv_req), .inv_clean(1'b0), .inv_busy(i_inv_busy),
      .l2_req(i_l2_req), .l2_we(i_l2_we), .l2_addr(i_l2_addr), .l2_wdata(i_l2_wdata),
      .l2_rdata(i_l2_rdata), .l2_ack(i_l2_ack));
   reg ibusy; reg [3:0] icnt; reg [PAW-OFFB-1:0] iad_q;  integer i_l2reads = 0;
   always @(posedge clk) begin
      i_l2_ack <= 0;
      if (reset) ibusy <= 0;
      else if (!ibusy && i_l2_req) begin ibusy<=1; icnt<=L2LAT; iad_q<=i_l2_addr; end
      else if (ibusy) begin
         if (icnt==0) begin
            for (k=0;k<64;k=k+1) i_l2_rdata[k*8 +: 8] <= l2mem[(iad_q<<OFFB)+k];
            i_l2_ack <= 1; ibusy <= 0; i_l2reads <= i_l2reads + 1;
         end else icnt <= icnt-1;
      end
   end

   // ---------------- pipelined-stream driver ----------------
   // Accept = the rd_req&rd_rdy handshake, straight off the ports. d_acc_q reflects
   // "accepted at the last posedge": the driver presents the NEXT address at the
   // following negedge, so a request is stable from presentation until its accept
   // and changes only afterwards, like a pipelined LSU would. Accepts on
   // CONSECUTIVE posedges are by construction pipelined takes (S_IDLE cannot
   // accept twice in a row -- the first accept leaves it), counted as b2b_cnt.
   wire d_acc = d_rd_req & d_rd_rdy;
   wire i_acc = i_rd_req & i_rd_rdy;
   reg  d_acc_q = 0, i_acc_q = 0;
   integer b2b_cnt = 0, s_cyc;
   always @(posedge clk) begin
      d_acc_q <= !reset && d_acc;
      i_acc_q <= !reset && i_acc;
      if (!reset && d_acc && d_acc_q) b2b_cnt = b2b_cnt + 1;
   end
   reg [PAW-1:0] s_addr [0:15];  reg s_nc [0:15];
   integer       s_seq  [0:15];    // per-entry completion order (-1 = pending)

   // Stream s_addr[0..n-1]/s_nc[0..n-1] as a pipelined requester. With
   // hit-under-miss, responses may complete OUT of accept order (a hit overtakes
   // an outstanding fill) -- identity is by rd_resp_addr, so the scoreboard is
   // set-based: every entry must complete exactly once with refm's data, and
   // s_seq records the completion order for the caller to assert on.
   task dstream; input integer n;
      integer ip, j, m, done, seq; reg [63:0] exp; reg matched;
      begin
         for (j=0;j<n;j=j+1) s_seq[j] = -1;
         ip = 1; done = 0; seq = 0; s_cyc = 0;
         @(negedge clk); d_rd_req=1; d_rd_addr=s_addr[0]; d_rd_uncached=s_nc[0];
         while (done < n) begin
            @(negedge clk); s_cyc = s_cyc + 1;
            if (s_cyc > 500) begin
               $display("FAIL stream: timeout (%0d/%0d done)", done, n); errs=errs+1; done = n;
            end
            if (d_rd_valid) begin
               matched = 0;
               for (j=0;j<n;j=j+1) if (!matched && s_seq[j] == -1 && d_rd_resp_addr === s_addr[j]) begin
                  matched = 1; s_seq[j] = seq; seq = seq + 1; done = done + 1;
                  exp = 0; for (m=0;m<8;m=m+1) exp[m*8 +: 8] = refm[s_addr[j]+m];
                  if (d_rd_data !== exp) begin
                     $display("FAIL stream: data @%h got=%h exp=%h", s_addr[j], d_rd_data, exp); errs=errs+1; end
               end
               if (!matched) begin
                  $display("FAIL stream: unexpected resp @%h", d_rd_resp_addr); errs=errs+1; end
            end
            if (d_acc_q) begin
               if (ip < n) begin d_rd_addr = s_addr[ip]; d_rd_uncached = s_nc[ip]; ip = ip + 1; end
               else d_rd_req = 0;
            end
         end
         d_rd_req = 0; d_rd_uncached = 0;
         for (j=0;j<6;j=j+1) begin        // the pipe must deliver EXACTLY n responses
            @(negedge clk);
            if (d_rd_valid) begin $display("FAIL stream: stray rd_valid @%h", d_rd_resp_addr); errs=errs+1; end
         end
      end
   endtask

   // ---------------- helpers ----------------
   task dread; input [PAW-1:0] a; input integer nb; // read nb bytes, check vs refm
      integer j; reg [63:0] got, exp;
      begin
         @(negedge clk); d_rd_req=1; d_rd_addr=a;
         @(negedge clk); while (!d_acc_q) @(negedge clk);   // hold until granted
         d_rd_req=0;
         while (!d_rd_valid) @(posedge clk);
         got = d_rd_data;
         exp = 0; for (j=0;j<nb;j=j+1) exp[j*8 +: 8] = refm[a+j];
         if ((got & ((64'd1<<(nb*8))-1)) !== exp) begin
            $display("FAIL read @%h nb=%0d got=%h exp=%h", a, nb, got, exp); errs=errs+1;
         end else $display("  ok  read @%h nb=%0d = %h", a, nb, exp);
         @(negedge clk);
      end
   endtask

   // Write-port requests are LEVEL-HELD until wr_ack, masked off on the ack cycle
   // (the store-buffer protocol): with the MSHR, a store's ack can come at S_MSHI
   // while its install still occupies the FSM, so a 1-cycle pulse could be missed.
   task dwrite; input [PAW-1:0] a; input [63:0] d; input [7:0] m; input integer nb;
      integer j;
      begin
         @(negedge clk); d_wr_req=1; d_wr_addr=a; d_wr_data=d; d_wr_mask=m;
         @(negedge clk); while (!d_wr_ack) @(negedge clk);
         d_wr_req=0;
         for (j=0;j<nb;j=j+1) if (m[j]) refm[a+j] = d[j*8 +: 8];
         $display("  ok  write @%h data=%h mask=%b", a, d, m);
         @(negedge clk);
      end
   endtask

   // Back-to-back store burst, LSU-style: wr_req is held CONTINUOUSLY and the next
   // store's payload is presented on the negedge right after each ack -- no idle
   // gap between FSM ops (the store-buffer drain pattern the boot runs).
   task dwburst; input integer n;   // uses s_addr[], burst data derived from index
      integer j, b;
      begin
         @(negedge clk); d_wr_req=1;
         for (j=0;j<n;j=j+1) begin
            d_wr_addr=s_addr[j]; d_wr_data=bd[j]; d_wr_mask=bm[j];
            @(negedge clk); while (!d_wr_ack) @(negedge clk);
            for (b=0;b<8;b=b+1) if (bm[j][b]) refm[s_addr[j]+b] = bd[j][b*8 +: 8];
         end
         d_wr_req=0;
         @(negedge clk);
      end
   endtask
   reg [63:0] bd [0:15];  reg [7:0] bm [0:15];

   task dflush;
      begin
         @(negedge clk); d_inv_req=1; @(posedge clk); @(negedge clk); d_inv_req=0;
         while (d_inv_busy) @(posedge clk);
         @(negedge clk);
         // writeback correctness: every byte in L2 must now match refm
         for (k=0;k<MEM;k=k+1) if (l2mem[k] !== refm[k]) begin
            $display("FAIL flush: l2mem[%0d]=%h refm=%h", k, l2mem[k], refm[k]); errs=errs+1;
         end
         $display("  ok  flush -> L2 matches refm");
      end
   endtask

   task dcbo; input [PAW-1:0] a; input z; input keep;   // Zicbom/Zicboz maintenance op
      begin
         @(negedge clk); d_wr_req=1; d_cbo_req=1; d_cbo_zero=z; d_cbo_keep=keep;
                         d_wr_addr=a; d_wr_data=0; d_wr_mask=0;
         @(negedge clk); while (!d_wr_ack) @(negedge clk);
         d_wr_req=0; d_cbo_req=0; d_cbo_zero=0; d_cbo_keep=0;
         @(negedge clk);
      end
   endtask

   task iread; input [PAW-1:0] a;                  // I$ 16-byte read, check vs refm
      integer j; reg [127:0] got, exp;
      begin
         @(negedge clk); i_rd_req=1; i_rd_addr=a;
         @(negedge clk); while (!i_acc_q) @(negedge clk);   // hold until granted
         i_rd_req=0;
         while (!i_rd_valid) @(posedge clk);
         got = i_rd_data;
         exp = 0; for (j=0;j<16;j=j+1) exp[j*8 +: 8] = refm[a+j];
         if (got !== exp) begin $display("FAIL iread @%h got=%h exp=%h",a,got,exp); errs=errs+1; end
         else $display("  ok  iread @%h = %h", a, exp);
         @(negedge clk);
      end
   endtask

   initial begin
      for (k=0;k<MEM;k=k+1) begin refm[k] = (k*7+3) & 8'hff; l2mem[k] = refm[k]; end
      d_rd_req=0; d_wr_req=0; d_inv_req=0; i_rd_req=0; i_inv_req=0;
      reset=1; repeat(3) @(negedge clk); reset=0; @(negedge clk);

      $display("== D$: miss/fill + hit ==");
      dread(34'h080, 8);          // cold miss -> fill from L2
      dread(34'h080, 8);          // hit
      dread(34'h088, 8);          // hit (same line, next word)

      $display("== D$: write + read-back (dirty) ==");
      dwrite(34'h080, 64'hDEADBEEF_CAFEF00D, 8'hFF, 8);
      dread (34'h080, 8);         // sees new data
      dwrite(34'h0A3, 64'h00000000_000000AA, 8'h01, 1);   // sub-word byte store
      dread (34'h0A0, 8);

      $display("== D$: dirty eviction + writeback + refill ==");
      dwrite(34'h00080, 64'h11111111_22222222, 8'hFF, 8); // A: way0[idx2], dirty
      dread (34'h10080, 8);                                // B: way1 (vic toggled)
      dread (34'h20080, 8);                                // C: way0[idx2] evicts A (WB)
      dread (34'h00080, 8);                                // A refilled from L2 -> written value

      $display("== D$: line-crossing read + write ==");
      dread (34'h0BC, 8);                                  // 0xBC..0xC3 spans line 2->3
      dwrite(34'h0BC, 64'h01020304_05060708, 8'hFF, 8);    // spanning write
      dread (34'h0BC, 8);
      dread (34'h0C0, 8);                                  // line 3 reflects the high half

      $display("== D$: flush (writeback) ==");
      dflush();
      dread (34'h080, 8);                                  // post-flush miss, from L2

      $display("== I$: miss/fill + hit + invalidate ==");
      iread(34'h100);
      iread(34'h100);
      iread(34'h1F8);                                      // 16-byte read spanning lines
      @(negedge clk); i_inv_req=1; @(posedge clk); @(negedge clk); i_inv_req=0;
      while (i_inv_busy) @(posedge clk); @(negedge clk);
      iread(34'h100);                                      // re-miss after invalidate

      // regression for the dropped-inv race: a 1-cycle inv_req pulsed WHILE the cache is
      // mid-refill must still invalidate (sticky inv). Old behavior dropped it -> a stale
      // line survived (the fence.i I$-coherency bug seen at 76M of Linux boot).
      $display("== I$: inv pulsed during a fill is not dropped ==");
      iread(34'h100);                                      // re-cache line @0x100
      @(negedge clk); i_rd_req=1; i_rd_addr=34'h2000;      // start a MISS -> fill begins
      @(posedge clk); @(negedge clk); i_rd_req=0;
      @(posedge clk);                                      // a cycle into the fill (cache busy)
      @(negedge clk); i_inv_req=1; @(posedge clk); @(negedge clk); i_inv_req=0;  // 1-cyc pulse mid-fill
      // settle: let the in-flight 0x2000 fill finish, then wait out the deferred (sticky)
      // inv -- it now WALKS the sets (one line/cycle behind inv_busy) instead of the old
      // one-cycle full clear, so a fixed 40-cycle settle no longer covers it. The cache
      // raises inv_busy the cycle the pulse lands (sticky), so polling it is sound.
      repeat (40) @(posedge clk);
      while (i_inv_busy) @(posedge clk); @(negedge clk);
      before_reads = i_l2reads;
      iread(34'h100);                                      // must MISS (invalidated) -> an L2 refill
      if (i_l2reads == before_reads) begin
         $display("FAIL: inv during fill was DROPPED (0x100 still cached)"); errs=errs+1;
      end else $display("  ok  inv during fill honored (0x100 refilled)");

      // ---- Svpbmt NC/IO: flush-around store + no-stale load (write-back D$) ----
      // The hazard write-back introduces for non-coherent DMA: a dirty line is invisible to a
      // DMA engine reading memory, and a DMA write is masked by a stale cached line. NC accesses
      // must (a) push stores straight to L2 and (b) never keep the line, so DMA stays coherent.
      $display("== Svpbmt NC: store flushes around to L2, load never goes stale ==");
      d_wr_uncached = 1;
      dwrite(34'h140, 64'hA5A5A5A5_5A5A5A5A, 8'hFF, 8);     // NC store
      d_wr_uncached = 0;
      for (k=0;k<8;k=k+1) if (l2mem[34'h140+k] !== refm[34'h140+k]) begin
         $display("FAIL NC store: l2mem[%h]=%h exp=%h (did not flush around)",
                  34'h140+k, l2mem[34'h140+k], refm[34'h140+k]); errs=errs+1;
      end
      if (errs==0) $display("  ok  NC store reached L2 immediately");
      // NC load (fills, returns, invalidates), then a backdoor DMA write to L2, then NC load again
      // -> must observe the DMA's NEW value (a surviving stale cached line would fail dread's check).
      d_rd_uncached = 1;
      dread(34'h140, 8);
      for (k=0;k<8;k=k+1) begin l2mem[34'h140+k] = (k*13+1) & 8'hff; refm[34'h140+k] = (k*13+1) & 8'hff; end
      dread(34'h140, 8);                                    // must see DMA's value, not the cached A5..
      d_rd_uncached = 0;

      // ---- Zicbom / Zicboz cache-maintenance ops (write-back D$) ----
      $display("== Zicbom cbo.clean: writeback, keep line valid ==");
      dwrite(34'h180, 64'h01234567_89ABCDEF, 8'hFF, 8);    // dirty line @0x180
      dcbo  (34'h180, 1'b0, 1'b1);                          // cbo.clean (keep)
      for (k=0;k<8;k=k+1) if (l2mem[34'h180+k] !== refm[34'h180+k]) begin
         $display("FAIL cbo.clean: l2mem[%h]=%h exp=%h", 34'h180+k, l2mem[34'h180+k], refm[34'h180+k]); errs=errs+1; end
      before_reads = d_l2reads;
      dread (34'h180, 8);                                   // line kept valid -> no L2 refill
      if (d_l2reads != before_reads) begin $display("FAIL cbo.clean: line not kept (refilled)"); errs=errs+1; end
      else $display("  ok  cbo.clean wrote back + kept the line");

      $display("== Zicbom cbo.flush: writeback + invalidate ==");
      dwrite(34'h1C0, 64'hFEDCBA98_76543210, 8'hFF, 8);     // dirty line @0x1C0
      dcbo  (34'h1C0, 1'b0, 1'b0);                          // cbo.flush (invalidate)
      for (k=0;k<8;k=k+1) if (l2mem[34'h1C0+k] !== refm[34'h1C0+k]) begin
         $display("FAIL cbo.flush: l2mem[%h]=%h exp=%h", 34'h1C0+k, l2mem[34'h1C0+k], refm[34'h1C0+k]); errs=errs+1; end
      before_reads = d_l2reads;
      dread (34'h1C0, 8);                                   // invalidated -> must refill from L2
      if (d_l2reads == before_reads) begin $display("FAIL cbo.flush: line not invalidated"); errs=errs+1; end
      else $display("  ok  cbo.flush wrote back + invalidated");

      $display("== Zicboz cbo.zero: the addressed block becomes zero ==");
      dcbo  (34'h200, 1'b1, 1'b0);                          // cbo.zero @0x200 (cold -> allocate+zero)
      for (k=0;k<64;k=k+1) refm[34'h200+k] = 8'h00;         // golden: whole 64B block is now zero
      dread (34'h200, 8); dread (34'h220, 8); dread (34'h238, 8);   // resident zero line
      dflush();                                             // writeback -> L2 block is zero too

      // ---- pipelined read-hit path ----
      $display("== pipelined stream: all-hit reads accepted back-to-back ==");
      dread(34'h300, 8); dread(34'h340, 8); dread(34'h380, 8); dread(34'h3C0, 8);  // warm 4 lines
      s_addr[0]=34'h300; s_addr[1]=34'h348; s_addr[2]=34'h388; s_addr[3]=34'h3C8;
      s_addr[4]=34'h310; s_addr[5]=34'h350; s_addr[6]=34'h390; s_addr[7]=34'h3D0;
      for (k=0;k<8;k=k+1) s_nc[k]=0;
      before_reads = b2b_cnt;
      dstream(8);
      if (b2b_cnt - before_reads !== 7)
         begin $display("FAIL: all-hit stream b2b=%0d (want 7)", b2b_cnt-before_reads); errs=errs+1; end
      else if (s_cyc > 11)                       // 8 reads: n+2 cycles at 1 hit/cycle
         begin $display("FAIL: all-hit stream took %0d cyc (want <=11)", s_cyc); errs=errs+1; end
      else $display("  ok  8-read hit stream: b2b=7, %0d cyc", s_cyc);

      $display("== pipelined stream: miss/span/NC drain in order ==");
      s_addr[0]=34'h300;   s_nc[0]=0;            // hit
      s_addr[1]=34'h28000; s_nc[1]=0;            // cold -> miss mid-stream (fill, resume)
      s_addr[2]=34'h340;   s_nc[2]=0;            // hit right behind the fill
      s_addr[3]=34'h37C;   s_nc[3]=0;            // line-crossing read 0x37C..0x383
      s_addr[4]=34'h388;   s_nc[4]=1;            // NC read (S_FIN slow path, flush-around)
      s_addr[5]=34'h3C8;   s_nc[5]=0;            // hit after the NC drop
      dstream(6);
      $display("  ok  mixed stream: 6 responses, all matched");

      $display("== pipelined stream: same-address back-to-back reads ==");
      for (k=0;k<4;k=k+1) begin s_addr[k]=34'h300; s_nc[k]=0; end
      before_reads = b2b_cnt;
      dstream(4);
      if (b2b_cnt - before_reads !== 3)
         begin $display("FAIL: same-addr stream b2b=%0d (want 3)", b2b_cnt-before_reads); errs=errs+1; end
      else $display("  ok  4x same-address reads accepted back-to-back");

      // ---- write-back buffer + hit-under-miss ----
      // One stream, no idle gaps, so the internal state is deterministic:
      // C misses into the MSHR (victim = dirty A, pinned), A HITS UNDER the
      // outstanding miss (must complete first), X is a second miss that parks
      // and then takes the MSHR after C's install (which evicts A into the
      // wb-buffer, undrained -- the port stays busy with X's fill), and the
      // final re-read of A finds it ONLY in the buffer: a serialized BOUNCE.
      // A wrong bounce returns stale L2 bytes; a lost dirty bit fails dflush.
      $display("== HUM + WB buffer: hit overtakes miss; evicted line bounces ==");
      dwrite(34'h000C0, 64'h5555AAAA_3333CCCC, 8'hFF, 8);   // dirty A (idx 3, way0)
      dread (34'h100C0, 8);                                  // B fills way1
      s_addr[0]=34'h200C0; s_nc[0]=0;                        // C: miss -> MSHR, victim A
      s_addr[1]=34'h000C0; s_nc[1]=0;                        // A: hit under the miss
      s_addr[2]=34'h28080; s_nc[2]=0;                        // X: 2nd miss -> parks, then MSHR
      s_addr[3]=34'h000C0; s_nc[3]=0;                        // A again: evicted by now -> bounce
      dstream(4);
      if (!(s_seq[1] < s_seq[0]))
         begin $display("FAIL: hit did not overtake the miss (seq %0d vs %0d)", s_seq[1], s_seq[0]); errs=errs+1; end
      else $display("  ok  hit completed under the outstanding miss (seq %0d < %0d)", s_seq[1], s_seq[0]);
      dflush();                                              // bounce kept A dirty; L2==refm

      // Same shape, but the re-read is NC: it must NOT bounce (the flush-around
      // drop would lose the dirty line) -- it parks, the buffer drains to L2,
      // and the fill reads it back fresh. Data + dflush catch either failure.
      $display("== HUM + WB buffer: NC re-read of the buffered line drains first ==");
      dwrite(34'h00100, 64'h0123456789ABCDEE, 8'hFF, 8);     // dirty A2 (idx 4, way0)
      dread (34'h10100, 8);                                  // B2 fills way1
      s_addr[0]=34'h20100; s_nc[0]=0;                        // C2: miss -> MSHR, victim A2
      s_addr[1]=34'h10100; s_nc[1]=0;                        // B2: hit under the miss
      s_addr[2]=34'h280C0; s_nc[2]=0;                        // X2: 2nd miss -> parks, then MSHR
      s_addr[3]=34'h00100; s_nc[3]=1;                        // NC read of A2: drain + fresh fill
      dstream(4);
      dflush();                                              // nothing may be lost; L2==refm

      // ---- NC store whose WIDTH spans the line end (r_span=1) but whose byte mask
      // does not spill (the virtio desc3.flags case: 2 bytes at line offset 60/62) --
      // and the genuinely-spilling variant. All four residency combos of line A/B:
      // the Ubuntu boot corruption was a flags store whose bytes never reached DDR.
      $display("== Svpbmt NC: width-span store at line offset 60/62 (all residency combos) ==");
      for (cmb=0;cmb<4;cmb=cmb+1) begin
         dflush();                                            // reset residency (clobbers k)
         if (cmb[0]) dread(34'h2400, 8);                      // pre-cache line A (clean)
         if (cmb[1]) dread(34'h2440, 8);                      // pre-cache line B (clean)
         d_wr_uncached = 1;
         dwrite(34'h2400+60, 64'h0000000000000001 + cmb, 8'h03, 2); // "flags" store, no spill
         dwrite(34'h2400+62, 64'h00000000BEEF0000 + cmb, 8'h0F, 4); // spills 2 bytes into B
         d_wr_uncached = 0;
         for (j2=0;j2<128;j2=j2+1) if (l2mem[34'h2400+j2] !== refm[34'h2400+j2]) begin
            $display("FAIL NC width-span combo=%0d: l2mem[%h]=%h exp=%h",
                     cmb, 34'h2400+j2, l2mem[34'h2400+j2], refm[34'h2400+j2]); errs=errs+1;
         end
         if (errs==0) $display("  ok  combo=%0d (A cached=%0d B cached=%0d)", cmb, cmb[0], cmb[1]);
      end
      // dirty-A variant: the NC push must carry the dirty line's bytes too
      $display("== Svpbmt NC: width-span store onto a DIRTY line ==");
      dflush();
      dwrite(34'h2480, 64'h1122334455667788, 8'hFF, 8);       // dirty A'
      d_wr_uncached = 1;
      dwrite(34'h2480+60, 64'h0000000000000007, 8'h03, 2);
      d_wr_uncached = 0;
      for (j2=0;j2<64;j2=j2+1) if (l2mem[34'h2480+j2] !== refm[34'h2480+j2]) begin
         $display("FAIL NC dirty width-span: l2mem[%h]=%h exp=%h",
                  34'h2480+j2, l2mem[34'h2480+j2], refm[34'h2480+j2]); errs=errs+1;
      end
      if (errs==0) $display("  ok  dirty-line NC width-span push");
      dflush();

      // ---- back-to-back NC store burst (store-buffer drain pattern): the boot's
      // virtio descriptor build is 2/4/8-byte NC stores to one line with NO idle
      // cycles between FSM ops, including the width-span flags/next stores. The
      // lost desc3.flags corruption appeared only in this adjacency.
      $display("== Svpbmt NC: back-to-back store burst incl. width-span offsets ==");
      for (cmb=0;cmb<2;cmb=cmb+1) begin
         dflush();
         if (cmb[0]) begin dread(34'h2500, 8); dread(34'h2540, 8); end   // resident variant
         d_wr_uncached = 1;
         s_addr[0]=34'h2500+62; bd[0]=64'h0000000000000002; bm[0]=8'h03;  // prev-req "next" (span)
         s_addr[1]=34'h2500+60; bd[1]=64'h0000000000000001; bm[1]=8'h03;  // flags (width-span)
         s_addr[2]=34'h2500+48; bd[2]=64'h0000000081d796f0; bm[2]=8'hFF;  // addr
         s_addr[3]=34'h2500+56; bd[3]=64'h0000000000000010; bm[3]=8'h0F;  // len
         s_addr[4]=34'h2500+62; bd[4]=64'h0000000000000004; bm[4]=8'h03;  // next (width-span)
         s_addr[5]=34'h2500+32; bd[5]=64'h1111111122222222; bm[5]=8'hFF;  // desc2 fields
         s_addr[6]=34'h2500+40; bd[6]=64'h0003000200000200; bm[6]=8'hFF;
         dwburst(7);
         d_wr_uncached = 0;
         for (j2=0;j2<128;j2=j2+1) if (l2mem[34'h2500+j2] !== refm[34'h2500+j2]) begin
            $display("FAIL NC burst cmb=%0d: l2mem[%h]=%h exp=%h",
                     cmb, 34'h2500+j2, l2mem[34'h2500+j2], refm[34'h2500+j2]); errs=errs+1;
         end
         if (errs==0) $display("  ok  NC b2b burst cmb=%0d", cmb);
      end

      // ---- THE descriptor-table clobber (Ubuntu disk-root boot wedge): a spanning
      // NC store whose line1 fill evicts a DIRTY victim. The victim capture (S_WB)
      // reused wb_way/wb_idx -- the very registers holding the line0 push slot -- so
      // S_WTR then streamed the victim's slot (now holding line1) and pushed a WHOLE
      // WRONG LINE under line0's address. Setup: A resident; C dirty at line-B's
      // index; D fills the other way so B's victim is C's (dirty) slot.
      $display("== Svpbmt NC: span store + line1 fill evicting a dirty victim ==");
      dflush();
      dread (34'h02600, 8);                                  // A resident clean (idx 0x98)
      dwrite(34'h12640, 64'hD1D1D1D1D1D1D1D1, 8'hFF, 8);     // C: dirty at idx 0x99, way w
      dread (34'h22640, 8);                                  // D: fills the other way, vicm -> w
      d_wr_uncached = 1;
      dwrite(34'h02600+60, 64'h0000000000000001, 8'h03, 2);  // the desc3.flags store
      d_wr_uncached = 0;
      for (j2=0;j2<128;j2=j2+1) if (l2mem[34'h02600+j2] !== refm[34'h02600+j2]) begin
         $display("FAIL NC span dirty-victim: l2mem[%h]=%h exp=%h",
                  34'h02600+j2, l2mem[34'h02600+j2], refm[34'h02600+j2]); errs=errs+1;
      end
      if (errs==0) $display("  ok  span NC push survives a dirty-victim line1 fill");
      dflush();

      if (errs==0) $display("CACHE-TB: ALL TESTS PASSED"); else $display("CACHE-TB FAIL (%0d errors)", errs);
      $finish;
   end

   initial begin #500000; $display("CACHE-TB TIMEOUT"); $finish; end
endmodule

`default_nettype wire
