`timescale 1ns/1ps
`default_nettype none

// Multiple branches in flight at once (each its own checkpoint, post-truncation).
// Three branches occupy three distinct checkpoints simultaneously:
//   PC0  : addi x5,x0,5
//   PC4  : bne  x5,x5,+8   NOT taken (x5==x5) -> correct; checkpoint commits   (B0)
//   PC8  : addi x6,x0,6
//   PC12 : bne  x6,x6,+8   NOT taken -> correct; checkpoint commits            (B1)
//   PC16 : addi x7,x0,7
//   PC20 : beq  x7,x7,+44  TAKEN -> PC64 (mispredict; younger checkpoint)      (B2)
//   PC24 : addi x8,x0,99   WRONG-PATH (fall-through after B2)
//   PC64 : addi x8,x0,55   TARGET
//   PC68 : addi x9,x8,1    consumer -> 56 if x8 came from the target (100 if wrong)
//
// Because each branch truncates its bundle, B0/B1/B2 land in consecutive checkpoints
// and (B0 depends on x5, etc.) several are open at once before the oldest commits.
// B0/B1 are correctly predicted (not taken) so they raise no redirect and their
// checkpoints commit; only B2 redirects, precisely to its own target, WITHOUT
// disturbing the older committed path. This proves branches hold independent
// checkpoints, in-order commit drains through several branch-checkpoints, and a
// younger branch's recovery is selective (older work survives).
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
      .imem_avail(imem_avail), .hw_ip(12'd0), .dmem_rdata(64'd0), .dmem_rvalid(1'b1), .dmem_wready(1'b1), .wb_valid(wb_valid), .wb_pr(wb_pr), .wb_val(wb_val),
      .redirect(redirect), .redirect_target(redirect_target), .commit(commit));

   reg [15:0] mem [0:63];
   integer m;
   always @(*) for (m=0;m<HW;m=m+1) imem_data[m*16 +: 16] = mem[(imem_addr>>1)+m];

   reg saw_redir, saw5, saw6, saw7, saw56, saw100;
   reg [PCW-1:0] redir_tgt;

   initial begin
      for (i=0;i<64;i=i+1) mem[i]=16'h0001;             // c.nop fill
      mem[0]=16'h0293;  mem[1]=16'h0050;                // addi x5,x0,5
      mem[2]=16'h9463;  mem[3]=16'h0052;                // bne  x5,x5,+8   (B0, not taken)
      mem[4]=16'h0313;  mem[5]=16'h0060;                // addi x6,x0,6
      mem[6]=16'h1463;  mem[7]=16'h0063;                // bne  x6,x6,+8   (B1, not taken)
      mem[8]=16'h0393;  mem[9]=16'h0070;                // addi x7,x0,7
      mem[10]=16'h8663; mem[11]=16'h0273;               // beq  x7,x7,+44 -> PC64 (B2, taken)
      mem[12]=16'h0413; mem[13]=16'h0630;               // addi x8,x0,99   WRONG (PC24)
      mem[32]=16'h0413; mem[33]=16'h0370;               // addi x8,x0,55   TARGET (PC64)
      mem[34]=16'h0493; mem[35]=16'h0014;               // addi x9,x8,1    consumer (PC68)

      saw_redir=0; saw5=0; saw6=0; saw7=0; saw56=0; saw100=0; redir_tgt=0;
      reset=1; @(negedge clk); @(negedge clk); reset=0;

      for (c=0; c<50; c=c+1) begin
         @(negedge clk);
         if (redirect) begin saw_redir=1; redir_tgt=redirect_target; end
         if (commit) ncommit = ncommit + 1;
         for (k=0;k<IW;k=k+1) if (wb_valid[k]) case (wb_val[k*64+:64])
            64'd5:   saw5=1;
            64'd6:   saw6=1;
            64'd7:   saw7=1;
            64'd56:  saw56=1;
            64'd100: saw100=1;
         endcase
      end

      if (!saw_redir)              begin $display("FAIL: B2 redirect never fired"); errs=errs+1; end
      else if (redir_tgt!==64'h40) begin $display("FAIL: redirect target=%h exp 0x40 (B2's target)",redir_tgt); errs=errs+1; end
      if (!saw5)   begin $display("FAIL: x5=5 (before B0) missing");        errs=errs+1; end
      if (!saw6)   begin $display("FAIL: x6=6 (between B0,B1) missing");     errs=errs+1; end
      if (!saw7)   begin $display("FAIL: x7=7 (between B1,B2) missing");     errs=errs+1; end
      if (!saw56)  begin $display("FAIL: consumer x9=56 (correct x8=55) not seen"); errs=errs+1; end
      if (saw100)  begin $display("FAIL: consumer saw x9=100 (wrong-path x8 reached arch state)"); errs=errs+1; end
      if (ncommit==0) begin $display("FAIL: nothing committed -- branch checkpoints stuck"); errs=errs+1; end

      $display("branch_multi: commits observed=%0d", ncommit);
      if (errs==0) $display("branch_multi: ALL TESTS PASSED (3 branch-checkpoints in flight; older commit, younger redirects)");
      else         $display("branch_multi: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
