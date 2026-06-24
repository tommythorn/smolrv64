`timescale 1ns/1ps
`default_nettype none

// A wrong-path iterative divide gets squashed mid-flight: the branch redirect must
// abort the in-flight divide (free its shard, no spurious completion) and the
// correct path must run and commit. Exercises the squash -> exec_shard -> divider
// abort wiring end-to-end.
//   addi x5,x0,1
//   beq  x5,x5,+12   TAKEN -> PC16
//   div  x9,x5,x5    WRONG-PATH (1/1); starts on its shard, squashed before done
//   PC16 addi x10,x0,42
//   PC20 addi x11,x10,1   -> 43  (correct-path consumer)
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
      .imem_avail(imem_avail), .hw_ip(12'd0), .dmem_rdata(64'd0), .wb_valid(wb_valid), .wb_pr(wb_pr), .wb_val(wb_val),
      .redirect(redirect), .redirect_target(redirect_target), .commit(commit));

   reg [15:0] mem [0:63];
   integer m;
   always @(*) for (m=0;m<HW;m=m+1) imem_data[m*16 +: 16] = mem[(imem_addr>>1)+m];

   reg saw_redir, saw42, saw43;
   reg [PCW-1:0] redir_tgt;

   initial begin
      for (i=0;i<64;i=i+1) mem[i]=16'h0001;
      mem[0]=16'h0293; mem[1]=16'h0010;   // addi x5,x0,1
      mem[2]=16'h8663; mem[3]=16'h0052;   // beq x5,x5,+12 -> PC16
      mem[4]=16'hC4B3; mem[5]=16'h0252;   // div x9,x5,x5  WRONG-PATH
      mem[8]=16'h0513; mem[9]=16'h02A0;   // addi x10,x0,42 (target)
      mem[10]=16'h0593; mem[11]=16'h0015; // addi x11,x10,1 -> 43

      saw_redir=0; saw42=0; saw43=0; redir_tgt=0;
      reset=1; @(negedge clk); @(negedge clk); reset=0;

      // The wrong-path divide, if NOT aborted, would hold its shard ~64 cycles. With
      // abort it frees immediately, so the correct path commits well within ~60 cycles.
      for (c=0; c<60; c=c+1) begin
         @(negedge clk);
         if (redirect) begin saw_redir=1; redir_tgt=redirect_target; end
         if (commit) ncommit = ncommit + 1;
         for (k=0;k<IW;k=k+1) if (wb_valid[k]) case (wb_val[k*64+:64])
            64'd42: saw42=1;
            64'd43: saw43=1;
         endcase
      end

      if (!saw_redir)              begin $display("FAIL: branch redirect never fired"); errs=errs+1; end
      else if (redir_tgt!==64'h10) begin $display("FAIL: redirect target=%h exp 0x10",redir_tgt); errs=errs+1; end
      if (!saw42) begin $display("FAIL: target x10=42 not seen");                 errs=errs+1; end
      if (!saw43) begin $display("FAIL: consumer x11=43 not seen (correct path)"); errs=errs+1; end
      if (ncommit==0) begin $display("FAIL: nothing committed within 60 cyc -- wrong-path divide not aborted?"); errs=errs+1; end

      $display("divsquash: commits observed=%0d", ncommit);
      if (errs==0) $display("divsquash: ALL TESTS PASSED (wrong-path divide aborted; correct path commits)");
      else         $display("divsquash: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
