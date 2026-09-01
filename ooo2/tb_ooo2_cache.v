`default_nettype none
// rv_cache directed: WHAT THE LOOKUP PIPELINE DOES WHILE A FILL IS IN FLIGHT.
//
// The split of the fill machine out of the lookup path (2e3791c/922ba08) let requests be
// resolved while a line is fetched. Nothing in the Linux cosim exercised the resulting
// concurrency hard enough to fail -- the LSU and the I$ fetch buffer are single-outstanding,
// so before satp is written the D$ pipeline never holds two requests, and three defects
// reached a bitstream. Each is deterministic once the second request is placed by hand, and
// each is swept over WHERE IN THE FILL it arrives, because that is the only variable.
//
//   T1  A second read for the LINE BEING FILLED. It waits on the one MSHR and hits the moment
//       the line lands. Stage B consumes bk_rddata -- the row addressed at the PREVIOUS edge
//       -- and the banks have no read enable, so every cycle it held was a cycle its address
//       was not presented. It shipped bank row 0. Two page-table walkers sharing a level ask
//       this of the D$ constantly, which is why the board died just after enabling paging.
//   T1b A read of a DIFFERENT resident line during the fill, which must not be spoiled by the
//       response port being busy in the F_ANS cycle.
//   T2  An INVALIDATE while a solo request's replay is owed. inv_pend holds off the replay's
//       slot, f_v holds off the scan (F_IDLE waits for !f_v), and only the replay clears f_v.
//       fence.i plus a line-crossing fetch is enough: no MMU, no second client, dead board.
//   T3  (the cache's own invariant) no bank row is read and written in the same cycle. The
//       behavioural RAM returns pre-write data there; a simple-dual-port BRAM returns invalid
//       data, so this one is a hardware bug that simulation reports as a clean run.
//
// Shaped as the I$ -- WRITABLE=0 -- because that instance takes the solo path on every
// line-crossing fetch and its inv_req is fence.i.
module tb;
   localparam PAW=64, RDW=64, LINEB=512, OFFB=6;
   localparam LAT   = 20;        // DDR-shaped line latency: the window under test
   localparam DLYMAX= 40;        // sweep the second request across the whole fill and past it

   reg clk=0, reset=1;
   always #5 clk = ~clk;

   // The requester contract the SoC uses: hold until ACCEPTED, drop on the ack. rd_valid is
   // registered, so "hold until answered" gets the request taken twice (rtl-rules.md D5).
   reg             req_pend=0;
   wire            rd_req = req_pend;
   reg  [PAW-1:0]  rd_addr=0;
   reg  [3:0]      rd_tag=0;
   wire [RDW-1:0]  rd_data;
   wire            rd_valid, rd_ack;
   wire [PAW-1:0]  rd_resp_addr;
   wire [3:0]      rd_resp_tag;
   reg             inv_req=0;
   wire            inv_busy;
   wire            l2_req, l2_we;
   wire [PAW-OFFB-1:0] l2_addr;
   wire [LINEB-1:0] l2_wdata;
   reg  [LINEB-1:0] l2_rdata=0;
   reg              l2_ack=0;
   always @(posedge clk) if (reset) req_pend <= 1'b0; else if (rd_ack) req_pend <= 1'b0;

   rv_cache #(.PAW(PAW), .SIZE_KB(128), .RDW(RDW), .WDW(64),
              .WRITABLE(0), .PREFETCH(0), .PERF_ID(0)) dut
     (.clk(clk), .reset(reset),
      .rd_req(rd_req), .rd_addr(rd_addr), .rd_data(rd_data), .rd_valid(rd_valid),
      .rd_resp_addr(rd_resp_addr), .rd_tag(rd_tag), .rd_resp_tag(rd_resp_tag),
      .rd_ack(rd_ack), .rd_uncached(1'b0),
      .wr_req(1'b0), .wr_addr(64'd0), .wr_data(64'd0), .wr_mask(8'd0), .wr_ack(),
      .wr_uncached(1'b0), .cbo_req(1'b0), .cbo_zero(1'b0), .cbo_keep(1'b0),
      .inv_req(inv_req), .inv_clean(1'b0), .inv_busy(inv_busy),
      .l2_req(l2_req), .l2_we(l2_we), .l2_addr(l2_addr), .l2_wdata(l2_wdata),
      .l2_rdata(l2_rdata), .l2_ack(l2_ack), .perf_access(), .perf_miss());

   // Memory: word w of line L reads {L, w, 5a5a5a5}, so every 64-bit word in the space is
   // distinct and a wrong one names the line it actually came from.
   function [63:0] expect_word(input [PAW-1:0] a);
      expect_word = {a[37:6], a[5:3], 1'b0, 28'h5A5A5A5};
   endfunction

   integer lcnt; reg lbusy=0; reg [PAW-OFFB-1:0] laddr; integer w;
   always @(posedge clk) begin
      l2_ack <= 1'b0;
      if (reset) lbusy <= 1'b0;
      else if (!lbusy && l2_req) begin lbusy<=1'b1; lcnt<=LAT; laddr<=l2_addr; end
      else if (lbusy) begin
         if (lcnt==0) begin
            for (w=0; w<8; w=w+1)
               l2_rdata[w*64 +: 64] <= {laddr[31:0], w[2:0], 1'b0, 28'h5A5A5A5};
            l2_ack <= 1'b1; lbusy <= 1'b0;
         end else lcnt <= lcnt-1;
      end
   end

   integer cyc=0;
   always @(posedge clk) if (!reset) cyc <= cyc+1;

   reg [63:0] got [0:15];
   reg [15:0] got_v;
   initial got_v = 0;
   always @(posedge clk) if (!reset && rd_valid) begin
      got[rd_resp_tag] <= rd_data;
      got_v[rd_resp_tag] <= 1'b1;
      if ($test$plusargs("trace"))
         $display("  [resp] cyc=%0d tag=%h addr=%h data=%h", cyc, rd_resp_tag, rd_resp_addr, rd_data);
   end

   integer dly, i, errors;
   reg [PAW-1:0] A, B, S;

   task issue(input [PAW-1:0] a, input [3:0] t);   // held until accepted, dropped on the ack
      begin
         rd_addr = a; rd_tag = t; req_pend = 1'b1;
         while (req_pend) @(negedge clk);
      end
   endtask

   task expect_eq(input [63:0] g, input [63:0] e, input [255:0] what);
      if (g !== e) begin
         $display("FAIL dly=%0d %0s: read back %h, want %h", dly, what, g, e);
         errors = errors + 1;
      end
   endtask

   initial begin
      errors = 0;
      repeat (8) @(negedge clk);
      reset = 0;
      @(negedge clk);

      // Each iteration uses its own quadrant of the address space, and T2's full invalidate
      // empties the cache between them, so the iterations do not interfere.
      for (dly = 0; dly <= DLYMAX; dly = dly + 1) begin
         A = 64'h8000_0000 + (dly << 16) + 64'h1008;
         B = 64'h8000_0000 + (dly << 16) + 64'h2010;
         S = 64'h8000_0000 + (dly << 16) + 64'h303C;   // offset 60 + 8 > 64 -> span -> solo
         got_v = 0;

         issue(A, 4'h1);                                  // make A resident
         i = 0; while (!got_v[1] && i < 500) begin @(negedge clk); i = i+1; end
         expect_eq(got[1], expect_word(A), "prime");

         issue(B, 4'h2);                                  // misses -> handed to the fill machine
         repeat (dly) @(negedge clk);
         issue(A, 4'h3);                                  // a resident line, mid-fill
         issue(B + 64'd8, 4'h4);                          // B's OWN line, mid-fill
         i = 0;
         while ((!got_v[2] || !got_v[3] || !got_v[4]) && i < 500) begin @(negedge clk); i=i+1; end
         if (!got_v[2] || !got_v[3] || !got_v[4]) begin
            $display("FAIL dly=%0d: response lost (v2=%b v3=%b v4=%b)",
                     dly, got_v[2], got_v[3], got_v[4]);
            errors = errors + 1;
         end else begin
            expect_eq(got[3], expect_word(A),          "resident hit under fill");
            expect_eq(got[4], expect_word(B + 64'd8),  "same-line hit under fill");
            expect_eq(got[2], expect_word(B),          "the missing read");
         end

         issue(S, 4'h7);                                  // solo: its miss is REPLAYED
         repeat (dly) @(negedge clk);
         inv_req = 1'b1; @(negedge clk); inv_req = 1'b0;  // fence.i inside that fill
         i = 0;
         while ((!got_v[7] || inv_busy) && i < 20000) begin @(negedge clk); i = i+1; end
         if (!got_v[7] || inv_busy) begin
            $display("FAIL dly=%0d: WEDGED (resp=%b inv_busy=%b f_v=%b f_replay=%b fst=%0d st=%0d inv_pend=%b)",
                     dly, got_v[7], inv_busy, dut.f_v, dut.f_replay, dut.fst, dut.st, dut.inv_pend);
            errors = errors + 1;
            dly = DLYMAX;                                 // wedged: the rest would only re-wedge
         end
      end

      if (errors == 0) $display("rv_cache directed: PASS (%0d arrival cycles)", DLYMAX+1);
      else             $display("rv_cache directed: FAIL (%0d errors)", errors);
      $finish;
   end
endmodule
`default_nettype wire
