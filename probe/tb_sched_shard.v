`timescale 1ns/1ps
`default_nettype none

// sched_shard tests. Self lane (SH=0) is looped back exactly as the bundle will
// wire it: clr[0] = this shard's dispatched dest, wake[0] = this shard's issue.
// Sibling lane 1 is driven directly to model a cross-shard producer.
//
// Timing contract: dispatch at cycle T -> entry in IQ at T+1; issue is
// combinational (oldest eligible), entry freed at the issue edge; a broadcast
// clr/wake at T updates the scoreboard at the T->T+1 edge (so a dependent issues
// the cycle after its producer issues -- 1-cycle ALU wakeup).
module tb;
   localparam SHARDS=4, NPHYS=256, PBITS=8, IQD=8, IQW=3, SEQW=8, LATW=2;

   reg                clk=0; always #5 clk=~clk;
   reg                reset;
   reg                disp_valid, disp_pdst_v, disp_need1, disp_need2;
   reg [SEQW-1:0]     disp_seq;
   reg [PBITS-1:0]    disp_pdst, disp_ps1, disp_ps2;
   reg [LATW-1:0]     disp_lat;
   wire               disp_ready;
   // sibling-driven broadcast lane
   reg                sib_clr_v, sib_wake_v;
   reg [PBITS-1:0]    sib_clr_pr, sib_wake_pr;
   wire               iss_valid, iss_pdst_v;
   wire [SEQW-1:0]    iss_seq;
   wire [PBITS-1:0]   iss_pdst, iss_ps1, iss_ps2;
   wire [LATW-1:0]    iss_lat;
   integer errs=0;

   // broadcast buses: lane0 = self loopback, lane1 = sibling, lanes 2..3 = 0
   wire [SHARDS-1:0]       clr_valid  = {2'b00, sib_clr_v,  (disp_valid & disp_ready & disp_pdst_v)};
   wire [SHARDS*PBITS-1:0] clr_pr     = {{2*PBITS{1'b0}}, sib_clr_pr, disp_pdst};
   wire [SHARDS-1:0]       wake_valid = {2'b00, sib_wake_v, (iss_valid & iss_pdst_v)};
   wire [SHARDS*PBITS-1:0] wake_pr    = {{2*PBITS{1'b0}}, sib_wake_pr, iss_pdst};

   sched_shard #(.SHARDS(SHARDS), .SH(0), .NPHYS(NPHYS), .PBITS(PBITS),
                 .IQD(IQD), .IQW(IQW), .SEQW(SEQW), .LATW(LATW)) dut
     (.clk(clk), .reset(reset),
      .disp_valid(disp_valid), .disp_seq(disp_seq), .disp_pdst(disp_pdst),
      .disp_pdst_v(disp_pdst_v), .disp_ps1(disp_ps1), .disp_need1(disp_need1),
      .disp_ps2(disp_ps2), .disp_need2(disp_need2), .disp_lat(disp_lat),
      .disp_ready(disp_ready),
      .clr_valid(clr_valid), .clr_pr(clr_pr),
      .wake_valid(wake_valid), .wake_pr(wake_pr),
      .squash(1'b0), .squash_seq(8'd0),
      .iss_valid(iss_valid), .iss_seq(iss_seq), .iss_pdst(iss_pdst),
      .iss_pdst_v(iss_pdst_v), .iss_ps1(iss_ps1), .iss_ps2(iss_ps2), .iss_lat(iss_lat));

   task do_disp(input [SEQW-1:0] sq, input [PBITS-1:0] dst, input pdv,
                input [PBITS-1:0] s1, input n1, input [PBITS-1:0] s2, input n2);
      begin disp_valid=1; disp_seq=sq; disp_pdst=dst; disp_pdst_v=pdv;
            disp_ps1=s1; disp_need1=n1; disp_ps2=s2; disp_need2=n2; disp_lat=1; end
   endtask
   task no_disp; begin disp_valid=0; disp_pdst_v=0; disp_need1=0; disp_need2=0; end endtask

   // sample issue state mid-cycle, then advance the edge
   task ckiss(input [127:0] nm, input ev, input [SEQW-1:0] es);
      begin #1;
         if (iss_valid!==ev) begin $display("FAIL %0s iss_valid=%b exp %b",nm,iss_valid,ev); errs=errs+1; end
         else if (ev && iss_seq!==es) begin $display("FAIL %0s iss_seq=%0d exp %0d",nm,iss_seq,es); errs=errs+1; end
      end
   endtask

   initial begin
      reset=1; no_disp; sib_clr_v=0; sib_wake_v=0; sib_clr_pr=0; sib_wake_pr=0;
      @(negedge clk); @(negedge clk); reset=0;

      // ---- Test A: dependent chain A(seq0)->B(seq1)->C(seq2), 1/cycle ----
      @(negedge clk); do_disp(0, 7'd20,1, 7'd0,0, 7'd0,0);   // A: no src deps
      ckiss("A.c0", 1'b0, 0);                                // IQ empty this cycle
      @(negedge clk); do_disp(1, 7'd21,1, 7'd20,1, 7'd0,0);  // B: src1=20 (A)
      ckiss("A.c1", 1'b1, 0);                                // A issues
      @(negedge clk); do_disp(2, 7'd22,1, 7'd21,1, 7'd0,0);  // C: src1=21 (B)
      ckiss("A.c2", 1'b1, 1);                                // B issues (woke by A)
      @(negedge clk); no_disp;
      ckiss("A.c3", 1'b1, 2);                                // C issues
      @(negedge clk);
      ckiss("A.c4", 1'b0, 0);                                // drained

      // ---- Test B: oldest-first contention on a cross-shard producer pr40 ----
      @(negedge clk); sib_clr_v=1; sib_clr_pr=7'd40;         // sibling allocates producer of 40
                      do_disp(10, 7'd41,1, 7'd40,1, 7'd0,0); // Q waits on 40
      ckiss("B.c0", 1'b0, 0);
      @(negedge clk); sib_clr_v=0; do_disp(11, 7'd42,1, 7'd40,1, 7'd0,0); // R waits on 40
      ckiss("B.c1", 1'b0, 0);                                // Q in IQ but 40 not ready
      @(negedge clk); no_disp; sib_wake_v=1; sib_wake_pr=7'd40; // sibling issues producer
      ckiss("B.c2", 1'b0, 0);                                // wake applies at edge; still not ready
      @(negedge clk); sib_wake_v=0;
      ckiss("B.c3", 1'b1, 10);                               // Q,R both eligible -> oldest (10)
      @(negedge clk);
      ckiss("B.c4", 1'b1, 11);                               // then R
      @(negedge clk);
      ckiss("B.c5", 1'b0, 0);

      // ---- Test C: IQ-full backpressure (8 entries all stuck on pr50) ----
      @(negedge clk); sib_clr_v=1; sib_clr_pr=7'd50; do_disp(20, 7'd60,1, 7'd50,1, 7'd0,0);
      @(negedge clk); sib_clr_v=0;
      // dispatch 7 more stuck instrs; disp_ready must stay high until full
      do_disp(21, 7'd61,1, 7'd50,1, 7'd0,0); #1; if(!disp_ready) begin $display("FAIL C dr@2"); errs=errs+1; end
      @(negedge clk); do_disp(22, 7'd62,1, 7'd50,1, 7'd0,0);
      @(negedge clk); do_disp(23, 7'd63,1, 7'd50,1, 7'd0,0);
      @(negedge clk); do_disp(24, 7'd64,1, 7'd50,1, 7'd0,0);
      @(negedge clk); do_disp(25, 7'd65,1, 7'd50,1, 7'd0,0);
      @(negedge clk); do_disp(26, 7'd66,1, 7'd50,1, 7'd0,0);
      @(negedge clk); do_disp(27, 7'd67,1, 7'd50,1, 7'd0,0); // 8th -> fills IQ this edge
      #1; if(!disp_ready) begin $display("FAIL C dr@8 (still room expected)"); errs=errs+1; end
      @(negedge clk); do_disp(28, 7'd68,1, 7'd50,1, 7'd0,0); // 9th attempt
      #1; if(disp_ready) begin $display("FAIL C dr@9 (IQ should be full)"); errs=errs+1; end
      if(iss_valid) begin $display("FAIL C none-should-issue"); errs=errs+1; end
      @(negedge clk); no_disp;

      if (errs==0) $display("sched_shard: ALL TESTS PASSED");
      else         $display("sched_shard: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
