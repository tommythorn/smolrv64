`timescale 1ns/1ps
`default_nettype none

// Smoke TB for soc_top: load a flat image into the internal RAM (+hex), run until a
// store to `tohost` (+tohost): 1 => PASS. Exercises the whole core->I$/D$->arbiter->RAM
// path as one module. (No MMIO devices yet; RAM-only tests.)
module tb;
   reg clk=0; always #5 clk=~clk;
   reg reset;
   wire        commit, dmem_wen;
   wire [63:0] dmem_waddr, dmem_wdata;  wire [7:0] dmem_wmask;

   soc_top dut (.clk(clk), .reset(reset), .commit(commit),
                .dmem_wen(dmem_wen), .dmem_waddr(dmem_waddr),
                .dmem_wdata(dmem_wdata), .dmem_wmask(dmem_wmask));

   reg [63:0] tohost;  integer ncyc, c, i;  reg [8*256-1:0] hexfile;
   initial begin
      tohost = 64'h8000_1000;  ncyc = 400000;
      if (!$value$plusargs("hex=%s", hexfile)) begin $display("FATAL: need +hex"); $finish; end
      for (i=0; i<(1<<21); i=i+1) dut.ram[i] = 8'd0;
      $readmemh(hexfile, dut.ram);
      if ($value$plusargs("tohost=%h", tohost)) ;
      if ($value$plusargs("cycles=%d", ncyc)) ;
      reset=1; @(negedge clk); @(negedge clk); reset=0;
      for (c=0; c<ncyc; c=c+1) begin
         @(negedge clk);
         if (dmem_wen && (dmem_waddr - (dmem_waddr%8)) == tohost && dmem_wmask[0]) begin
            if (dmem_wdata[31:0] == 32'd1) $display("SOCTOP-TEST PASS");
            else $display("SOCTOP-TEST FAIL test=%0d (tohost=%h)", dmem_wdata[31:1], dmem_wdata);
            $finish;
         end
      end
      $display("SOCTOP-TEST TIMEOUT after %0d cycles", ncyc);
      $finish;
   end
endmodule

`default_nettype wire
