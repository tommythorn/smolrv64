`timescale 1ns/1ps
`default_nettype none

// Unit test for plic.v: drive a source level, program priority/enable/threshold over the
// MMIO port, claim the interrupt, complete it, and check meip/seip + claim/complete behave.
module tb;
   reg clk=0; always #5 clk=~clk;
   reg reset;
   reg         we, re;
   reg  [23:0] addr;
   reg  [63:0] wdata;
   reg  [7:0]  wmask;
   wire [63:0] rdata;
   reg  [63:0] src;
   wire        meip, seip;

   plic dut (.clk(clk), .reset(reset), .we(we), .re(re), .addr(addr),
             .wdata(wdata), .wmask(wmask), .rdata(rdata), .src(src),
             .meip(meip), .seip(seip));

   integer errs = 0;
   task wr; input [23:0] a; input [63:0] d; input [7:0] m; begin
      @(negedge clk); addr=a; wdata=d; wmask=m; we=1'b1; @(negedge clk); we=1'b0; end
   endtask
   // pulse a 1-cycle read strobe, then sample rdata the following cycle (registered)
   task rd; input [23:0] a; output [63:0] d; begin
      @(negedge clk); addr=a; re=1'b1; @(negedge clk); re=1'b0; @(negedge clk); d=rdata; end
   endtask
   task chk; input [63:0] got, exp; input [255:0] msg; begin
      if (got!==exp) begin errs=errs+1; $display("FAIL %0s: got %h exp %h", msg, got, exp); end end
   endtask

   reg [63:0] v;
   initial begin
      we=0; re=0; addr=0; wdata=0; wmask=8'hff; src=0;
      reset=1; @(negedge clk); @(negedge clk); reset=0; @(negedge clk);

      // program: priority[10]=7, threshold=0, enable bit 10
      wr(24'h000028, 64'd7, 8'h0f);                 // priority[10] (10*4=0x28)
      wr(24'h201000, 64'd0, 8'h0f);                 // threshold ctx1
      wr(24'h002080, 64'h400, 8'h0f);               // enable bit 10 (low word)

      // no source yet -> no interrupt
      repeat (4) @(negedge clk);
      chk({63'd0,meip}, 64'd0, "meip idle");
      rd(24'h201004, v); chk(v, 64'd0, "claim idle");

      // assert source 10 -> becomes pending+enabled -> meip
      @(negedge clk); src = 64'd1 << 10;
      repeat (4) @(negedge clk);
      chk({63'd0,meip}, 64'd1, "meip asserted");
      chk({63'd0,seip}, 64'd1, "seip asserted");

      // claim -> returns 10, clears pending, asserts in_service
      rd(24'h201004, v); chk(v, 64'd10, "claim=10");
      repeat (3) @(negedge clk);
      // source still high but in service -> not re-pending -> meip drops
      chk({63'd0,meip}, 64'd0, "meip after claim");

      // drop the source, then complete (write source id back) -> rearm; no re-pend
      @(negedge clk); src = 64'd0;
      wr(24'h201004, 64'd10, 8'h0f);                // complete source 10
      repeat (4) @(negedge clk);
      chk({63'd0,meip}, 64'd0, "meip after complete");

      // re-assert -> pending again (gateway rearmed)
      @(negedge clk); src = 64'd1 << 10;
      repeat (4) @(negedge clk);
      chk({63'd0,meip}, 64'd1, "meip re-asserted");
      rd(24'h201004, v); chk(v, 64'd10, "re-claim=10");

      if (errs==0) $display("ALL TESTS PASSED");
      else         $display("plic: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
