`timescale 1ns/1ps
`default_nettype none

// commit_ctl: per-bundle checkpoints, count-by-ISSUE (not writeback). Verifies
// full/back-pressure when the ring fills, and in-order commit when the oldest
// checkpoint's issued-count drains. The TB mirrors the freelist's `cur` (advances
// on create, jumps on redirect).
module tb;
   localparam NCHK=4, CBITS=2, IW=4, CNTW=3, DCW=3;

   reg                clk=0; always #5 clk=~clk;
   reg                reset, disp_fire, redirect;
   reg  [DCW-1:0]     disp_count;
   reg  [IW-1:0]      iss_valid;
   reg  [IW*CBITS-1:0] iss_ckpt;
   reg  [CBITS-1:0]   redirect_ckpt;
   reg  [CBITS-1:0]   cur_r;            // mirror of freelist.cur
   wire               create, commit, rollback, full;
   wire [CBITS-1:0]   commit_idx, rollback_idx;
   integer errs=0;

   commit_ctl #(.NCHK(NCHK), .CBITS(CBITS), .IW(IW), .CNTW(CNTW), .DCW(DCW)) dut
     (.clk(clk), .reset(reset), .cur(cur_r), .disp_fire(disp_fire), .disp_count(disp_count),
      .iss_valid(iss_valid), .iss_ckpt(iss_ckpt), .redirect(redirect), .redirect_ckpt(redirect_ckpt),
      .create(create), .commit(commit), .commit_idx(commit_idx),
      .rollback(rollback), .rollback_idx(rollback_idx), .full(full));

   // mirror cur the way the freelist would
   always @(posedge clk)
      if (reset) cur_r <= 0;
      else if (redirect) cur_r <= redirect_ckpt;
      else if (create)   cur_r <= cur_r + 1'b1;

   task idle; begin disp_fire=0; disp_count=0; iss_valid=0; iss_ckpt=0; redirect=0; redirect_ckpt=0; end endtask
   task ck(input [127:0] nm, input got, exp);
      begin if (got!==exp) begin $display("FAIL %0s = %b exp %b",nm,got,exp); errs=errs+1; end end
   endtask

   initial begin
      idle; reset=1; @(negedge clk); @(negedge clk); reset=0; @(negedge clk);
      #1 ck("init.full",full,0); ck("init.commit",commit,0);

      // dispatch 3 bundles (ckpt 0,1,2), no issues -> ring fills (1 slot reserved)
      disp_fire=1; disp_count=4; #1 ck("d0.full",full,0); @(posedge clk);   // ckpt0, cur->1
      #1 ck("d1.full",full,0); @(posedge clk);                              // ckpt1, cur->2
      #1 ck("d2.full",full,0); @(posedge clk);                              // ckpt2, cur->3
      idle; #1 ck("d3.full",full,1);                                        // cur=3 -> FULL
      ck("d3.nocommit",commit,0);                                           // count[0]!=0

      // issue ckpt0's 4 instructions (issue is the completion event)
      @(negedge clk); idle; iss_valid=4'b1111; iss_ckpt={2'd0,2'd0,2'd0,2'd0};
      @(posedge clk);                                                       // count[0] -> 0
      @(negedge clk); idle;
      #1 ck("commit0",commit,1); ck("commit0.idx",commit_idx==0,1'b1);      // oldest drained -> commit
      @(posedge clk);                                                       // committed -> 1
      @(negedge clk); idle;
      #1 ck("postcommit.full",full,0);                                      // a slot freed
      ck("postcommit.nocommit",commit,0);                                   // count[1]!=0 yet

      // rollback: redirect to ckpt1 -> squashed counts reset, rollback emitted
      @(negedge clk); redirect=1; redirect_ckpt=2'd1;
      #1 ck("rb.rollback",rollback,1); ck("rb.idx",rollback_idx==1,1'b1); ck("rb.nocommit",commit,0);
      @(posedge clk); idle;

      if (errs==0) $display("commit_ctl: ALL TESTS PASSED");
      else         $display("commit_ctl: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
