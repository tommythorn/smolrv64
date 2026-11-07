`timescale 1ns/1ps
// Unit TB for smolrv64_iq. The properties, in order of what would hurt most if broken:
//   1. a ready entry issues, and every dispatched entry issues exactly ONCE
//   2. an entry whose source is not ready does NOT issue
//   3. a wakeup arriving in the dispatch cycle is not lost (a producer broadcasts once)
//   4. wake-at-select makes a dependent issuable three cycles after its producer's pick (the column
//      in the cycle after, its wake registered, the ready bit registered)
//   5. the entry held downstream is never reallocated
//   6. full, and flush
//   7. in order (the INORDER instance), and an entry that hands itself on (d_hx) waits while its
//      handoff register is taken (hx_busy) while the others issue past it
//   8. the oldest live entry goes first when it is ready, past a younger one at a lower index (OLDEST)
module tb;
   localparam NENT=8, IDXB=3, NSRC=2, ROBB=4, PBITS=9, NWB=3;
   reg clk=0; always #5 clk=~clk;
   reg reset=1, flush=0, unit_busy=0, iss_take=0, hold_v=0, d_hx=0, hx_busy=0;
   reg  [IDXB-1:0]       hold_ent=0;
   reg                   d_valid=0;
   reg  [ROBB-1:0]       d_rob=0;
   reg  [NSRC*PBITS-1:0] d_ps=0;
   reg  [NSRC-1:0]       d_r=2'b11;
   reg  [PBITS-1:0]      d_prd=0;
   reg  [NWB-1:0]        wb_v=0;
   reg  [NWB*PBITS-1:0]  wb_preg=0;
   wire d_ready, iss_v, blk_v;
   wire [IDXB-1:0] d_ent, iss_ent;
   wire [ROBB-1:0] iss_rob;
   wire [PBITS-1:0] blk_pr;
   wire [IDXB:0] occupancy, free_n;
   integer errs=0;

   wire [NENT*NSRC-1:0] iss_hq, io_iss_hq;   // the registered wake matches (the core's operand select)
   smolrv64_iq #(.NENT(NENT),.IDXB(IDXB),.NSRC(NSRC),.ROBB(ROBB),.PBITS(PBITS),.NWB(NWB),
             .FIXEDL(1),.OLDEST(1)) dut
     (.clk(clk),.reset(reset),
      .d_valid(d_valid),.d_ready(d_ready),.d_rob(d_rob),.d_ps(d_ps),.d_r(d_r),
      .d_prd(d_prd),.d_long(1'b0),.d_hx(d_hx),.d_ag(1'b0),.d_ent(d_ent),
      .wb_v(wb_v),.wb_preg(wb_preg),.iss_hq(iss_hq),
      .unit_busy(unit_busy),.hx_busy(hx_busy),.ag_only(1'b0),.iss_v(iss_v),.iss_ent(iss_ent),.iss_rob(iss_rob),
      .iss_take(iss_take),
      .hold_v(hold_v),.hold_ent(hold_ent),
      .blk_v(blk_v),.blk_pr(blk_pr),.flush(flush),.kd_v(1'b0),.kh({ROBB{1'b0}}),.kb({ROBB{1'b0}}),.occupancy(occupancy),.free_n(free_n));

   // Second instance in INORDER mode -- the load scheduler's configuration.
   reg io_d_valid=0, io_take=0, io_hold=0;
   reg [ROBB-1:0] io_rob=0; reg [NSRC*PBITS-1:0] io_ps=0; reg [NSRC-1:0] io_r=2'b11;
   reg [NWB-1:0] io_wb_v=0; reg [NWB*PBITS-1:0] io_wb_preg=0;
   wire io_iss_v, io_d_ready, io_blk_v;
   wire [ROBB-1:0] io_iss_rob;
   wire [IDXB-1:0] io_d_ent, io_iss_ent;
   wire [PBITS-1:0] io_blk_pr; wire [IDXB:0] io_occ, io_free_n;
   smolrv64_iq #(.NENT(NENT),.IDXB(IDXB),.NSRC(NSRC),.ROBB(ROBB),.PBITS(PBITS),.NWB(NWB),
             .FIXEDL(0),.INORDER(1)) dut_io
     (.clk(clk),.reset(reset),
      .d_valid(io_d_valid),.d_ready(io_d_ready),.d_rob(io_rob),.d_ps(io_ps),.d_r(io_r),
      .d_prd({PBITS{1'b0}}),.d_long(1'b0),.d_hx(1'b0),.d_ag(1'b0),.d_ent(io_d_ent),
      .wb_v(io_wb_v),.wb_preg(io_wb_preg),.iss_hq(io_iss_hq),
      .unit_busy(1'b0),.hx_busy(1'b0),.ag_only(1'b0),.iss_v(io_iss_v),.iss_ent(io_iss_ent),.iss_rob(io_iss_rob),
      .iss_take(io_take),
      .hold_v(io_hold),.hold_ent({IDXB{1'b0}}),
      .blk_v(io_blk_v),.blk_pr(io_blk_pr),.flush(flush),.kd_v(1'b0),.kh({ROBB{1'b0}}),.kb({ROBB{1'b0}}),.occupancy(io_occ),.free_n(io_free_n));

   task disp(input [ROBB-1:0] rob, input [PBITS-1:0] p0, input r0, input [PBITS-1:0] prd);
      begin
         @(negedge clk);
         d_valid=1; d_rob=rob; d_ps={9'd0,p0}; d_r={1'b1,r0}; d_prd=prd;
         @(posedge clk); @(negedge clk); d_valid=0;
      end
   endtask
   task wake(input [PBITS-1:0] p);
      begin @(negedge clk); wb_v=3'b001; wb_preg[PBITS-1:0]=p;
            @(posedge clk); @(negedge clk); wb_v=0; end
   endtask
   task take; begin @(negedge clk); iss_take=1; @(posedge clk); @(negedge clk); iss_take=0; end endtask
   task chk(input [447:0] nm, input exp_v, input [ROBB-1:0] exp_rob);
      begin
         if (iss_v !== exp_v || (exp_v && iss_rob !== exp_rob)) begin
            $display("FAIL %0s: iss_v=%b rob=%0d (want v=%b rob=%0d)", nm, iss_v, iss_rob, exp_v, exp_rob);
            errs=errs+1;
         end else $display("  ok  %0s", nm);
      end
   endtask

   initial begin
      repeat(3) @(posedge clk); @(negedge clk); reset=0;

      // 1/2. not-ready does not issue; ready does
      disp(4'd0, 9'd40, 1'b0, 9'd100);          // waits on p40
      @(negedge clk); chk("unready entry does not issue", 1'b0, 4'd0);
      wake(9'd40);
      @(negedge clk); chk("woken entry issues", 1'b1, 4'd0);
      take;
      @(negedge clk); chk("empty -> nothing", 1'b0, 4'd0);

      // 3. wakeup in the dispatch cycle itself is not lost
      @(negedge clk); wb_v=3'b001; wb_preg[PBITS-1:0]=9'd77;
      d_valid=1; d_rob=4'd2; d_ps={9'd0,9'd77}; d_r=2'b10; d_prd=9'd101;
      @(posedge clk); @(negedge clk); d_valid=0; wb_v=0;
      @(negedge clk); chk("wakeup during dispatch is not lost", 1'b1, 4'd2);
      take;

      // 4. wake-at-select: a consumer of the issuing entry is ready three cycles after its pick
      disp(4'd3, 9'd0,  1'b1, 9'd200);          // producer, dest p200
      disp(4'd4, 9'd200,1'b0, 9'd201);          // consumer of p200
      @(negedge clk); chk("producer ready", 1'b1, 4'd3);
      take;                                      // issuing it wakes the consumer
      chk("consumer not issuable a cycle after the pick", 1'b0, 4'd0);
      @(negedge clk); chk("nor two cycles after", 1'b0, 4'd0);
      @(negedge clk); chk("consumer issuable three cycles after", 1'b1, 4'd4);
      take;

      // 5. the held entry is never reallocated
      begin : holdtest
         integer i; reg bad;
         hold_v=1; hold_ent=3'd0; bad=0;
         // Only NENT-1 slots are free while one is held, so stop when d_ready drops rather
         // than dispatching a fixed count -- otherwise the module's own full-scheduler
         // assertion fires, which is the module being right and the test being wrong.
         for (i=0; i<NENT; i=i+1) begin
            @(negedge clk);
            if (d_ready) begin
               if (d_ent == 3'd0) bad=1;
               d_valid=1; d_rob=i[ROBB-1:0]; d_ps=0; d_r=2'b11; d_prd=9'd0;
               @(posedge clk); @(negedge clk); d_valid=0;
            end
         end
         if (bad) begin $display("FAIL hold: reallocated the held entry"); errs=errs+1; end
         else $display("  ok  held entry never reallocated (occ=%0d of %0d)", occupancy, NENT);
         hold_v=0;
      end

      // 6. flush
      @(negedge clk); flush=1; @(posedge clk); @(negedge clk); flush=0;
      if (occupancy!==0 || iss_v!==1'b0 || free_n!==NENT) begin
         $display("FAIL flush: occ=%0d iss_v=%b free_n=%0d", occupancy, iss_v, free_n); errs=errs+1;
      end else $display("  ok  flush clears");

      // 8. oldest first: a young op in a freed low slot does not pass an older ready one above it
      disp(4'd8, 9'd0,  1'b1, 9'd0);            // entry 0, ready
      disp(4'd9, 9'd60, 1'b0, 9'd0);            // entry 1, waits on p60
      @(negedge clk); chk("the only ready entry", 1'b1, 4'd8);
      take;                                      // entry 0 frees
      disp(4'd10, 9'd0, 1'b1, 9'd0);            // entry 0 again, younger than entry 1
      wake(9'd60);
      @(negedge clk); chk("the oldest ready entry before a younger one at a lower index", 1'b1, 4'd9);
      take;
      @(negedge clk); chk("then the younger one", 1'b1, 4'd10);
      take;

      // 1b. every dispatched entry issues exactly once (order unconstrained)
      begin : allissue
         integer i, n; reg [7:0] seen;
         seen=0;
         for (i=0;i<4;i=i+1) disp(i[ROBB-1:0], 9'd0, 1'b1, 9'd0);
         for (n=0;n<4;n=n+1) begin
            @(negedge clk);
            if (!iss_v) begin $display("FAIL all-issue: nothing ready at %0d",n); errs=errs+1; end
            else seen[iss_rob] = 1'b1;
            take;
         end
         if (seen[3:0]!==4'b1111) begin
            $display("FAIL all-issue: seen=%b", seen[3:0]); errs=errs+1;
         end else $display("  ok  every entry issues exactly once (order unconstrained)");
      end

      // 7. INORDER: the head-pointer mode the load scheduler uses for memory ordering.
      // A younger READY entry must NOT pass an older unready one -- that is the whole
      // property, and it must hold without any age comparison.
      begin : inorder_test
         io_hold = 1'b0;
         @(negedge clk);
         io_d_valid=1; io_rob=4'd5; io_ps={9'd0,9'd50}; io_r=2'b10; // head, waits on p50
         @(posedge clk); @(negedge clk);
         io_rob=4'd6; io_ps={9'd0,9'd0};  io_r=2'b11;               // younger, ready now
         @(posedge clk); @(negedge clk); io_d_valid=0;
         @(negedge clk);
         if (io_iss_v !== 1'b0) begin
            $display("FAIL inorder: younger ready entry passed the head (rob=%0d)", io_iss_rob);
            errs=errs+1;
         end else $display("  ok  inorder: younger ready does NOT pass an unready head");
         @(negedge clk); io_wb_v=3'b001; io_wb_preg[PBITS-1:0]=9'd50;
         @(posedge clk); @(negedge clk); io_wb_v=0;
         @(negedge clk);
         if (io_iss_v !== 1'b1 || io_iss_rob !== 4'd5) begin
            $display("FAIL inorder: head did not issue first (v=%b rob=%0d)", io_iss_v, io_iss_rob);
            errs=errs+1;
         end else $display("  ok  inorder: head issues first once ready");
         @(negedge clk); io_take=1; @(posedge clk); @(negedge clk); io_take=0;
         @(negedge clk);
         if (io_iss_v !== 1'b1 || io_iss_rob !== 4'd6) begin
            $display("FAIL inorder: second did not follow (v=%b rob=%0d)", io_iss_v, io_iss_rob);
            errs=errs+1;
         end else $display("  ok  inorder: then the next in program order");
      end

      // 7b. a handoff entry is no candidate while hx_busy; a plain one behind it issues
      @(negedge clk); hx_busy=1; d_hx=1;
      disp(4'd8, 9'd0, 1'b1, 9'd0);                // ready, hands itself on (lowest free entry)
      d_hx=0;
      @(negedge clk); chk("handoff entry waits while its register is taken", 1'b0, 4'd0);
      disp(4'd9, 9'd0, 1'b1, 9'd0);                // ready, plain
      @(negedge clk); chk("a plain entry issues past it", 1'b1, 4'd9);
      take;
      @(negedge clk); hx_busy=0;
      @(negedge clk); chk("the handoff entry issues once the register is free", 1'b1, 4'd8);
      take;

      $display("---- IQ-TB %s (errs=%0d)", errs==0 ? "PASS":"FAIL", errs);
      $finish;
   end
   initial begin #300000; $display("---- IQ-TB TIMEOUT"); $finish; end
endmodule
