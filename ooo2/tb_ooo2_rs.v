`timescale 1ns/1ps
// Unit TB for ooo2_rs. The properties, in order of what would hurt most if broken:
//   1. a ready entry issues, and every dispatched entry issues exactly ONCE
//   2. an entry whose source is not ready does NOT issue
//   3. a wakeup arriving in the dispatch cycle is not lost (a producer broadcasts once)
//   4. wake-at-select makes a dependent issuable the very next cycle (back to back)
//   5. the entry held downstream is never reallocated
//   6. full, and flush
module tb;
   localparam NENT=8, IDXB=3, NSRC=2, ROBB=4, PBITS=9, NWB=3;
   reg clk=0; always #5 clk=~clk;
   reg reset=1, flush=0, unit_busy=0, iss_take=0, hold_v=0;
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
   wire [NSRC*PBITS-1:0] iss_ps;
   wire [PBITS-1:0] blk_pr;
   wire [IDXB:0] occupancy;
   integer errs=0;

   ooo2_rs #(.NENT(NENT),.IDXB(IDXB),.NSRC(NSRC),.ROBB(ROBB),.PBITS(PBITS),.NWB(NWB),
             .FIXEDL(1)) dut
     (.clk(clk),.reset(reset),
      .d_valid(d_valid),.d_ready(d_ready),.d_rob(d_rob),.d_ps(d_ps),.d_r(d_r),
      .d_prd(d_prd),.d_ent(d_ent),
      .wb_v(wb_v),.wb_preg(wb_preg),
      .unit_busy(unit_busy),.iss_v(iss_v),.iss_ent(iss_ent),.iss_rob(iss_rob),
      .iss_ps(iss_ps),.iss_take(iss_take),
      .hold_v(hold_v),.hold_ent(hold_ent),
      .blk_v(blk_v),.blk_pr(blk_pr),.flush(flush),.occupancy(occupancy));

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

      // 4. wake-at-select: a consumer of the issuing entry is ready the NEXT cycle
      disp(4'd3, 9'd0,  1'b1, 9'd200);          // producer, dest p200
      disp(4'd4, 9'd200,1'b0, 9'd201);          // consumer of p200
      @(negedge clk); chk("producer ready", 1'b1, 4'd3);
      take;                                      // issuing it wakes the consumer
      @(negedge clk); chk("consumer issuable the very next cycle", 1'b1, 4'd4);
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
      if (occupancy!==0 || iss_v!==1'b0) begin
         $display("FAIL flush: occ=%0d iss_v=%b", occupancy, iss_v); errs=errs+1;
      end else $display("  ok  flush clears");

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

      $display("---- RS-TB %s (errs=%0d)", errs==0 ? "PASS":"FAIL", errs);
      $finish;
   end
   initial begin #300000; $display("---- RS-TB TIMEOUT"); $finish; end
endmodule
