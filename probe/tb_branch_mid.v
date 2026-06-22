`timescale 1ns/1ps
`default_nettype none

// Mid-bundle branch recovery (truncation): a taken branch sits at slot 2 of its
// fetch bundle, with correct-path *older* instructions at slots 0/1 that must
// survive AND commit, and a younger wrong-path instruction right behind it.
//
//   PC0  : addi x5,x0,7    correct, OLDER (slot0) -- must commit
//   PC4  : addi x6,x0,8    correct, OLDER (slot1) -- must commit
//   PC8  : beq  x5,x5,+24  TAKEN -> PC32 (slot2 = the branch; bundle truncates here)
//   PC12 : addi x7,x0,99   WRONG-PATH (would be slot3 without truncation)
//   PC32 : addi x7,x0,55   TARGET
//   PC36 : addi x8,x7,1    consumer -> 56 if x7 came from the target (100 if wrong)
//
// The aligner truncates the bundle at the branch, so the branch is the youngest
// instr in its checkpoint and PC8's bundle = {x5,x6,beq}. Without truncation the
// wrong-path slot3 would share that checkpoint, get seqno-squashed, and never
// decrement the checkpoint's outstanding count -> in-order commit DEADLOCKS. So
// `commit` firing is the discriminator that proves the generalization works.
module tb;
   localparam IW=4, HW=8, PCW=64, SEQW=8, PBITS=8, PBW=4;

   reg                clk=0; always #5 clk=~clk;
   reg                reset;
   wire               redirect;
   wire [PCW-1:0]     redirect_target, imem_addr;
   reg  [HW*16-1:0]   imem_data;
   wire [PBW-1:0]     imem_avail = 4'd8;
   wire [IW-1:0]      wb_valid;
   wire [IW*PBITS-1:0] wb_pr;
   wire [IW*64-1:0]   wb_val;
   wire               commit;
   integer errs=0, i, k, c, ncommit=0;

   backend_top #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW), .PBITS(PBITS), .RESET_PC(0)) dut
     (.clk(clk), .reset(reset), .imem_addr(imem_addr), .imem_data(imem_data),
      .imem_avail(imem_avail), .dmem_rdata(64'd0), .wb_valid(wb_valid), .wb_pr(wb_pr), .wb_val(wb_val),
      .redirect(redirect), .redirect_target(redirect_target), .commit(commit));

   reg [15:0] mem [0:63];
   integer m;
   always @(*) for (m=0;m<HW;m=m+1) imem_data[m*16 +: 16] = mem[(imem_addr>>1)+m];

   reg saw_redir, saw7, saw8, saw56, saw100;
   reg [PCW-1:0] redir_tgt;

   initial begin
      for (i=0;i<64;i=i+1) mem[i]=16'h0001;             // c.nop fill
      mem[0]=16'h0293; mem[1]=16'h0070;                 // addi x5,x0,7   (slot0)
      mem[2]=16'h0313; mem[3]=16'h0080;                 // addi x6,x0,8   (slot1)
      mem[4]=16'h8C63; mem[5]=16'h0052;                 // beq x5,x5,+24 -> PC32 (slot2)
      mem[6]=16'h0393; mem[7]=16'h0630;                 // addi x7,x0,99  WRONG (PC12)
      mem[16]=16'h0393; mem[17]=16'h0370;               // addi x7,x0,55  TARGET (PC32)
      mem[18]=16'h8413; mem[19]=16'h0013;               // addi x8,x7,1   consumer (PC36)

      saw_redir=0; saw7=0; saw8=0; saw56=0; saw100=0; redir_tgt=0;
      reset=1; @(negedge clk); @(negedge clk); reset=0;

      for (c=0; c<40; c=c+1) begin
         @(negedge clk);
         if (redirect) begin saw_redir=1; redir_tgt=redirect_target; end
         if (commit) ncommit = ncommit + 1;
         for (k=0;k<IW;k=k+1) if (wb_valid[k]) case (wb_val[k*64+:64])
            64'd7:   saw7=1;
            64'd8:   saw8=1;
            64'd56:  saw56=1;
            64'd100: saw100=1;
         endcase
      end

      if (!saw_redir)              begin $display("FAIL: no redirect fired"); errs=errs+1; end
      else if (redir_tgt!==64'h20) begin $display("FAIL: redirect target=%h exp 0x20",redir_tgt); errs=errs+1; end
      if (!saw7)   begin $display("FAIL: older x5=7 (slot0) missing"); errs=errs+1; end
      if (!saw8)   begin $display("FAIL: older x6=8 (slot1) missing"); errs=errs+1; end
      if (!saw56)  begin $display("FAIL: consumer x8=56 (correct x7=55) not seen"); errs=errs+1; end
      // NB: the wrong-path addi (x7=99) may *execute* speculatively before the
      // branch resolves (it is independent; the branch waits on x5). That is
      // correct OoO behaviour -- what must hold is that it never reaches arch
      // state: the rollback abandons its physreg and the post-redirect rename
      // points x7 at a fresh reg, so the consumer can never read it (x8 != 100).
      if (saw100)  begin $display("FAIL: consumer saw x8=100 (wrong-path x7 reached arch state)"); errs=errs+1; end
      if (ncommit==0) begin $display("FAIL: nothing committed -- checkpoint count stuck (deadlock)"); errs=errs+1; end

      $display("branch_mid: commits observed=%0d", ncommit);
      if (errs==0) $display("branch_mid: ALL TESTS PASSED (mid-bundle branch @ slot2; older slots commit)");
      else         $display("branch_mid: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
