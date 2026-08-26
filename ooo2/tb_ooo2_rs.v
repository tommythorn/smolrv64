`timescale 1ns/1ps
// Unit TB for ooo2_rs. The property under test is the one the whole change exists for:
// a YOUNGER ready entry issues before an OLDER stalled one.
module tb;
   localparam NENT=8, IDXB=3, ROBB=4, PBITS=9, NUNIT=4, NWB=3;
   reg clk=0; always #5 clk=~clk;
   reg reset=1, flush=0;
   reg d_valid=0; reg [ROBB-1:0] d_rob=0;
   reg [PBITS-1:0] d_ps1=0,d_ps2=0,d_ps3=0;
   reg d_r1=1,d_r2=1,d_r3=1; reg [NUNIT-1:0] d_unit=4'b0001;
   reg [NWB-1:0] wb_v=0; reg [NWB*PBITS-1:0] wb_preg=0;
   reg [NUNIT-1:0] unit_busy=0; reg [ROBB-1:0] head=0; reg iss_take=0;
   wire d_ready, iss_v; wire [ROBB-1:0] iss_rob; wire [NUNIT-1:0] iss_unit;
   wire [IDXB-1:0] d_ent, iss_ent;
   wire [PBITS-1:0] iss_ps1, iss_ps2, iss_ps3;
   reg in_order = 1'b0;
   wire [IDXB:0] occupancy;
   integer errs=0;

   ooo2_rs #(.NENT(NENT),.IDXB(IDXB),.ROBB(ROBB),.PBITS(PBITS),.NUNIT(NUNIT),.NWB(NWB)) dut
     (.clk(clk),.reset(reset),.d_valid(d_valid),.d_ready(d_ready),.d_rob(d_rob),
      .d_ps1(d_ps1),.d_ps2(d_ps2),.d_ps3(d_ps3),.d_r1(d_r1),.d_r2(d_r2),.d_r3(d_r3),
      .d_unit(d_unit),.d_ent(d_ent),.iss_ent(iss_ent),.in_order(in_order),
      .iss_ps1(iss_ps1),.iss_ps2(iss_ps2),.iss_ps3(iss_ps3),.wb_v(wb_v),.wb_preg(wb_preg),.unit_busy(unit_busy),.head(head),
      .iss_v(iss_v),.iss_rob(iss_rob),.iss_unit(iss_unit),.iss_take(iss_take),
      .flush(flush),.occupancy(occupancy));

   task disp(input [ROBB-1:0] rob, input [PBITS-1:0] p1, input r1, input [NUNIT-1:0] u);
      begin
         @(negedge clk);
         d_valid=1; d_rob=rob; d_ps1=p1; d_r1=r1; d_ps2=0; d_r2=1; d_ps3=0; d_r3=1; d_unit=u;
         @(posedge clk); @(negedge clk); d_valid=0;
      end
   endtask
   task wake(input [PBITS-1:0] p);
      begin @(negedge clk); wb_v=3'b001; wb_preg[PBITS-1:0]=p;
            @(posedge clk); @(negedge clk); wb_v=0; end
   endtask
   task expect_iss(input [447:0] nm, input exp_v, input [ROBB-1:0] exp_rob);
      begin
         if (iss_v !== exp_v || (exp_v && iss_rob !== exp_rob)) begin
            $display("FAIL %0s: iss_v=%b rob=%0d (want v=%b rob=%0d)", nm, iss_v, iss_rob, exp_v, exp_rob);
            errs=errs+1;
         end else $display("  ok  %0s", nm);
      end
   endtask
   task take; begin @(negedge clk); iss_take=1; @(posedge clk); @(negedge clk); iss_take=0; end endtask

   initial begin
      repeat(3) @(posedge clk); @(negedge clk); reset=0;

      // 1. older entry NOT ready, younger entry ready -> younger issues. The point.
      disp(4'd0, 9'd40, 1'b0, 4'b0001);      // rob0 waits on p40
      disp(4'd1, 9'd0,  1'b1, 4'b0001);      // rob1 ready
      @(negedge clk);
      expect_iss("younger-ready issues past older-stalled", 1'b1, 4'd1);
      take;

      // 2. still stalled with nothing else
      @(negedge clk);
      expect_iss("older still stalled -> no issue", 1'b0, 4'd0);

      // 3. wake it -> it issues
      wake(9'd40);
      @(negedge clk);
      expect_iss("woken entry issues", 1'b1, 4'd0);
      take;
      @(negedge clk);
      expect_iss("empty -> no issue", 1'b0, 4'd0);

      // 4. oldest-ready among several ready
      disp(4'd5, 9'd0, 1'b1, 4'b0001);
      disp(4'd3, 9'd0, 1'b1, 4'b0001);       // older by age (head=0)
      disp(4'd7, 9'd0, 1'b1, 4'b0001);
      @(negedge clk);
      expect_iss("oldest ready wins", 1'b1, 4'd3);
      take; @(negedge clk);
      expect_iss("then next oldest", 1'b1, 4'd5);
      take; @(negedge clk);
      expect_iss("then the youngest", 1'b1, 4'd7);
      take;

      // 5. a busy unit must not block a different unit's entry
      disp(4'd0, 9'd0, 1'b1, 4'b0010);       // needs unit1
      disp(4'd1, 9'd0, 1'b1, 4'b0100);       // needs unit2
      @(negedge clk); unit_busy=4'b0010;     // unit1 busy
      @(negedge clk);
      expect_iss("busy unit does not block another unit", 1'b1, 4'd1);
      take; @(negedge clk);
      expect_iss("its own unit still busy -> waits", 1'b0, 4'd0);
      unit_busy=0; @(negedge clk);
      expect_iss("unit freed -> issues", 1'b1, 4'd0);
      take;

      // 6. wakeup landing in the dispatch cycle itself
      @(negedge clk); wb_v=3'b001; wb_preg[PBITS-1:0]=9'd77;
      d_valid=1; d_rob=4'd2; d_ps1=9'd77; d_r1=1'b0; d_ps2=0; d_r2=1; d_ps3=0; d_r3=1; d_unit=4'b0001;
      @(posedge clk); @(negedge clk); d_valid=0; wb_v=0;
      @(negedge clk);
      expect_iss("wakeup in the dispatch cycle is not lost", 1'b1, 4'd2);
      take;

      // 6b. in_order: the SAME shape as test 1 must now NOT reorder.
      in_order = 1'b1;
      disp(4'd4, 9'd88, 1'b0, 4'b0001);      // older, waiting on p88
      disp(4'd5, 9'd0,  1'b1, 4'b0001);      // younger, ready
      @(negedge clk);
      expect_iss("in_order: younger ready does NOT pass older stalled", 1'b0, 4'd0);
      wake(9'd88);
      @(negedge clk);
      expect_iss("in_order: older issues first", 1'b1, 4'd4);
      take; @(negedge clk);
      expect_iss("in_order: then the younger", 1'b1, 4'd5);
      take;
      in_order = 1'b0;

      // 7. fill to full, check d_ready
      begin : fill
         integer i;
         for (i=0;i<NENT;i=i+1) disp(i[ROBB-1:0], 9'd60, 1'b0, 4'b0001);
      end
      @(negedge clk);
      if (d_ready !== 1'b0) begin $display("FAIL full: d_ready=%b occ=%0d", d_ready, occupancy); errs=errs+1; end
      else $display("  ok  full -> d_ready low (occ=%0d)", occupancy);

      // 8. flush empties it
      @(negedge clk); flush=1; @(posedge clk); @(negedge clk); flush=0;
      if (occupancy !== 0 || iss_v !== 1'b0) begin $display("FAIL flush: occ=%0d iss_v=%b", occupancy, iss_v); errs=errs+1; end
      else $display("  ok  flush clears");

      $display("---- RS-TB %s (errs=%0d)", errs==0 ? "PASS":"FAIL", errs);
      $finish;
   end
   initial begin #200000; $display("---- RS-TB TIMEOUT"); $finish; end
endmodule
