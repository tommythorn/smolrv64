`timescale 1ns/1ps
`default_nettype none

// Unit test for clint.v: mtime ticking, mtip compare, msip, and register
// read-back at the SmolRV64-compatible offsets. Uses a tiny scaler (4) so mtime
// advances quickly. Prints "CLINT TEST PASS" / "...FAIL <n>".
module tb;
   localparam SCALE = 4;
   reg          clk=0; always #5 clk=~clk;
   reg          reset;
   reg          we;
   reg  [15:0]  addr;
   reg  [63:0]  wdata;
   reg  [7:0]   wmask;
   wire [63:0]  rdata;
   wire         mtip, msip;
   wire [63:0]  mtime;

   clint #(.SCALE_DIV(SCALE)) dut
     (.clk(clk), .reset(reset), .we(we), .addr(addr), .wdata(wdata),
      .wmask(wmask), .rdata(rdata), .mtip(mtip), .msip(msip), .o_mtime(mtime));

   integer fails=0;
   task chk(input cond, input [8*48-1:0] msg);
      if (!cond) begin $display("  FAIL: %0s", msg); fails=fails+1; end
   endtask

   task wr(input [15:0] a, input [63:0] d, input [7:0] m);
      begin @(negedge clk); we=1; addr=a; wdata=d; wmask=m; @(negedge clk); we=0; end
   endtask

   integer guard;
   initial begin
      we=0; addr=0; wdata=0; wmask=0;
      reset=1; @(negedge clk); @(negedge clk); reset=0; @(negedge clk);

      // reset state
      chk(mtip==0, "mtip 0 at reset");
      chk(msip==0, "msip 0 at reset");
      chk(mtime==0, "mtime 0 at reset");

      // msip set/clear + readback (offset 0x0000)
      wr(16'h0000, 64'd1, 8'h01);
      chk(msip==1, "msip set");
      addr=16'h0000; #1; chk(rdata[0]==1'b1, "msip readback 1");
      wr(16'h0000, 64'd0, 8'h01);
      chk(msip==0, "msip clear");

      // program mtimecmp = 5 (full 64-bit store, wmask[4]=1) and read it back
      wr(16'h4000, 64'd5, 8'hFF);
      addr=16'h4000; #1; chk(rdata==64'd5, "mtimecmp readback 5");

      // mtime must advance and mtip must assert once mtime>=5
      guard=0;
      while (mtip==0 && guard<200) begin @(negedge clk); guard=guard+1; end
      chk(mtip==1, "mtip asserted after mtimecmp reached");
      chk(mtime>=64'd5, "mtime advanced past mtimecmp");

      // raise mtimecmp well above now -> mtip deasserts
      wr(16'h4000, mtime+64'd1000, 8'hFF);
      @(negedge clk); @(negedge clk);
      chk(mtip==0, "mtip deasserted after raising mtimecmp");

      // hi/lo word write path: set mtime hi word, check readback at 0xBFFC
      wr(16'hBFFC, 64'h0000_0007, 8'h0F);
      addr=16'hBFFC; #1; chk(rdata[31:0]==32'd7, "mtime hi word readback");

      if (fails==0) $display("ALL TESTS PASSED");
      else          $display("CLINT TEST FAIL %0d", fails);
      $finish;
   end
endmodule

`default_nettype wire
