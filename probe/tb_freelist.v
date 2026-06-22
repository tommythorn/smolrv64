`timescale 1ns/1ps
`default_nettype none

// freelist tests: allocation (ffs), pold reclamation on commit, and rollback
// returning younger spans' allocations. Small geometry: POOL=8, ARSH=2 -> free
// idx 2..7 = pr 8,12,16,20,24,28 (pr = SH + 4*idx).
module tb;
   localparam SHARDS=4, SH=0, SBITS=2, PBITS=8, POOL=8, LPOOL=3, NCHK=4, CBITS=2, ARSH=2;

   reg                clk=0; always #5 clk=~clk;
   reg                reset, alloc_en, create, commit, rollback;
   reg  [CBITS-1:0]   commit_idx, rollback_idx;
   reg  [SHARDS-1:0]  pold_valid;
   reg  [SHARDS*PBITS-1:0] pold_pr;
   wire [PBITS-1:0]   alloc_pr;
   wire               alloc_ok;
   wire [LPOOL:0]     free_count;
   wire [CBITS-1:0]   cur;
   integer errs=0;

   freelist #(.SHARDS(SHARDS), .SH(SH), .SBITS(SBITS), .PBITS(PBITS), .POOL(POOL),
              .LPOOL(LPOOL), .NCHK(NCHK), .CBITS(CBITS), .ARSH(ARSH)) dut
     (.clk(clk), .reset(reset), .alloc_en(alloc_en), .alloc_pr(alloc_pr),
      .alloc_ok(alloc_ok), .free_count(free_count), .pold_valid(pold_valid),
      .pold_pr(pold_pr), .create(create), .commit(commit), .commit_idx(commit_idx),
      .rollback(rollback), .rollback_idx(rollback_idx), .cur(cur));

   task idle; begin alloc_en=0; create=0; commit=0; rollback=0; commit_idx=0;
                    rollback_idx=0; pold_valid=0; pold_pr=0; end endtask
   task ck(input [127:0] nm, input [31:0] got, exp);
      begin if (got!==exp) begin $display("FAIL %0s = %0d exp %0d",nm,got,exp); errs=errs+1; end end
   endtask

   initial begin
      idle; reset=1; @(negedge clk); @(negedge clk); reset=0; @(negedge clk);
      #1 ck("init.cnt",free_count,6); ck("init.pr",alloc_pr,8); ck("init.cur",cur,0);

      // ---- alloc + commit reclamation ----
      alloc_en=1; #1 ck("a0.pr",alloc_pr,8); @(posedge clk);     // span0 alloc pr8 (idx2)
      idle; @(negedge clk); #1 ck("a0.cnt",free_count,5); ck("a1.pr",alloc_pr,12);
      create=1; @(posedge clk); idle; @(negedge clk); #1 ck("create.cur",cur,1);
      alloc_en=1; #1 ck("a1.pr2",alloc_pr,12); @(posedge clk);   // span1 alloc pr12 (idx3)
      idle; @(negedge clk); #1 ck("a1.cnt",free_count,4);
      // displace pold pr8 (idx2, owner0) in span1
      pold_valid=4'b0001; pold_pr={24'd0, 8'd8}; @(posedge clk);
      idle; @(negedge clk);
      // commit span0: P[0] empty -> no change
      commit=1; commit_idx=0; @(posedge clk); idle; @(negedge clk);
      #1 ck("commit0.cnt",free_count,4);
      // commit span1: frees P[1] = {pr8} -> idx2 back
      commit=1; commit_idx=1; @(posedge clk); idle; @(negedge clk);
      #1 ck("commit1.cnt",free_count,5); ck("commit1.pr",alloc_pr,8);

      // ---- rollback reclamation (fresh) ----
      reset=1; @(negedge clk); @(negedge clk); reset=0; @(negedge clk);
      #1 ck("r.init",free_count,6);
      alloc_en=1; @(posedge clk); idle;             // span0 alloc pr8(idx2)
      create=1; @(posedge clk); idle;               // -> span1
      alloc_en=1; @(posedge clk); idle;             // span1 alloc pr12(idx3)
      create=1; @(posedge clk); idle;               // -> span2
      alloc_en=1; @(posedge clk); idle;             // span2 alloc pr16(idx4)
      @(negedge clk); #1 ck("r.cnt3",free_count,3); ck("r.cur2",cur,2);
      // rollback to span1 (recover before span1): spans 1,2 undone (idx3,4 freed);
      // span0 (idx2) survives. cur reopens at 1.
      rollback=1; rollback_idx=1; @(posedge clk); idle; @(negedge clk);
      #1 ck("rb.cnt",free_count,5); ck("rb.cur",cur,1); ck("rb.pr",alloc_pr,12);

      if (errs==0) $display("freelist: ALL TESTS PASSED");
      else         $display("freelist: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
