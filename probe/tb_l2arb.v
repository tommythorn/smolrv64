`timescale 1ns/1ps
`default_nettype none

// Unit TB for l2_arbiter: a behavioral line memory (pulse-req/ack, 2-cycle latency)
// behind a 3-requester arbiter. Checks writes, read-back, and -- the key property --
// that simultaneous one-cycle request pulses from different requesters are all served
// (latched, not lost).
module tb;
   localparam NREQ=3, AW=4, DW=64;
   reg clk=0; always #5 clk=~clk;
   reg reset;
   integer errs=0;

   reg  [NREQ-1:0]    req, we;
   reg  [NREQ*AW-1:0] addr;
   reg  [NREQ*DW-1:0] wdata;
   wire [NREQ-1:0]    ack;
   wire [DW-1:0]      rdata;
   wire               mem_req, mem_we;  wire [AW-1:0] mem_addr;  wire [DW-1:0] mem_wdata;
   reg  [DW-1:0]      mem_rdata;  reg mem_ack;

   l2_arbiter #(.NREQ(NREQ), .AW(AW), .DW(DW)) dut
     (.clk(clk), .reset(reset), .req(req), .we(we), .addr(addr), .wdata(wdata),
      .ack(ack), .rdata(rdata), .mem_req(mem_req), .mem_we(mem_we), .mem_addr(mem_addr),
      .mem_wdata(mem_wdata), .mem_rdata(mem_rdata), .mem_ack(mem_ack));

   // behavioral line memory: 2-cycle latency, pulse-req/ack
   reg [DW-1:0] m [0:(1<<AW)-1];
   reg mbusy; reg [3:0] mcnt; reg mwe_q; reg [AW-1:0] mad_q; reg [DW-1:0] mwd_q;
   always @(posedge clk) begin
      mem_ack <= 1'b0;
      if (reset) mbusy<=1'b0;
      else if (!mbusy && mem_req) begin mbusy<=1'b1; mcnt<=4'd2; mwe_q<=mem_we; mad_q<=mem_addr; mwd_q<=mem_wdata; end
      else if (mbusy) begin
         if (mcnt==0) begin
            if (mwe_q) m[mad_q] <= mwd_q; else mem_rdata <= m[mad_q];
            mem_ack <= 1'b1; mbusy<=1'b0;
         end else mcnt <= mcnt-1;
      end
   end

   // issue a 1-cycle pulse on requester r; wait its ack; return rdata via `got`
   reg [DW-1:0] got;
   task do_req; input integer r; input iswr; input [AW-1:0] a; input [DW-1:0] d;
      begin
         @(negedge clk);
         req[r]=1'b1; we[r]=iswr; addr[r*AW +: AW]=a; wdata[r*DW +: DW]=d;
         @(negedge clk); req[r]=1'b0;          // 1-cycle pulse (addr/wdata stay held below)
         while (!ack[r]) @(negedge clk);
         got = rdata;
      end
   endtask

   integer who;
   initial begin
      req=0; we=0; addr=0; wdata=0;
      for (who=0; who<(1<<AW); who=who+1) m[who]=who;
      reset=1; repeat(3) @(negedge clk); reset=0; @(negedge clk);

      // write line 5 from req0, read it back from req1
      do_req(0, 1, 4'd5, 64'hAAAA_BBBB_CCCC_DDDD);
      do_req(1, 0, 4'd5, 64'd0);
      if (got !== 64'hAAAA_BBBB_CCCC_DDDD) begin $display("FAIL readback got=%h", got); errs=errs+1; end
      else $display("  ok  write/readback line5");

      // KEY TEST: simultaneous 1-cycle pulses on req1 (read line5) and req2 (read line8)
      @(negedge clk);
      req[1]=1; we[1]=0; addr[1*AW +: AW]=4'd5; wdata[1*DW +: DW]=0;
      req[2]=1; we[2]=0; addr[2*AW +: AW]=4'd8; wdata[2*DW +: DW]=0;
      @(negedge clk); req[1]=0; req[2]=0;       // both drop after one cycle
      // collect both acks (order: priority 1 before 2)
      begin : collect
         integer seen1, seen2; reg [DW-1:0] g1, g2;
         seen1=0; seen2=0;
         while (!(seen1 && seen2)) begin
            @(negedge clk);
            if (ack[1]) begin g1=rdata; seen1=1; end
            if (ack[2]) begin g2=rdata; seen2=1; end
         end
         if (g1 !== 64'hAAAA_BBBB_CCCC_DDDD) begin $display("FAIL concurrent req1 got=%h",g1); errs=errs+1; end
         if (g2 !== 64'd8) begin $display("FAIL concurrent req2 got=%h",g2); errs=errs+1; end
         if (errs==0) $display("  ok  concurrent pulses both served (req1=%h req2=%h)", g1, g2);
      end

      if (errs==0) $display("L2ARB-TB: ALL TESTS PASSED"); else $display("L2ARB-TB FAIL (%0d)", errs);
      $finish;
   end
   initial begin #100000; $display("L2ARB-TB TIMEOUT"); $finish; end
endmodule

`default_nettype wire
