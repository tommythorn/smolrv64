`timescale 1ns/1ps
`default_nettype none

// Branch-predictor microtest (plan Phase 0): a 20-iteration hot loop, a
// call/return pair, and a self-jump park.
//
//   PC0  : addi x5,x0,20        loop counter
//   PC4  : addi x6,x6,1     <- loop head (bundle base after the taken bne)
//   PC8  : addi x5,x5,-1
//   PC12 : bne  x5,x0,-8        taken 19x, falls through on the 20th
//   PC16 : addi x28,x6,100      consumer: 120 iff the loop ran exactly 20 times
//   PC20 : jal  x1,+12          call f (RAS push)
//   PC24 : jal  x0,0            park: self-jump (return target; trains BTB, then silent)
//   PC32 : jalr x0,0(x1)        f: ret (RAS-predicted from its first execution)
//
// Checks: (a) x28=120 written back -- the predictor must not change architecture
// (every mispredict rollback restored the map/GHR/RAS correctly); (b) the total
// redirect count collapses: unpredicted this program redirects >= 21 times (19
// taken bne + call + ret + park...); with the BTB+RAS working it is the cold
// bne (twice: entry bundle + loop-head bundle are different BTB entries), the
// loop exit, the cold call, the cold ret (one TY_RET training pass -- class
// lives in the BTB now, not the bytes), the cold park. Budget <= 7.
module tb;
   localparam IW=4, HW=8, PCW=64, SEQW=8, PBITS=9, PBW=4;

   reg                clk=0; always #5 clk=~clk;
   reg                reset;
   wire               redirect;
   wire [PCW-1:0]     redirect_target;
   wire [PCW-1:0]     imem_addr;
   reg  [HW*16-1:0]   imem_data;
   wire [PBW-1:0]     imem_avail = 4'd8;
   wire [IW-1:0]      wb_valid;
   wire [IW*PBITS-1:0] wb_pr;
   wire [IW*64-1:0]   wb_val;
   integer errs=0, i, k, c, redirs;

   backend_top #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW), .PBITS(PBITS), .RESET_PC(0)) dut
     (.clk(clk), .reset(reset), .imem_addr(imem_addr), .imem_data(imem_data),
      .imem_avail(imem_avail), .hw_ip(12'd0), .mtime(64'd0), .dmem_rdata(64'd0),
      .dmem_rvalid(1'b1), .dmem_wready(1'b1), .wb_valid(wb_valid), .wb_pr(wb_pr), .wb_val(wb_val),
      .redirect(redirect), .redirect_target(redirect_target));

   reg [15:0] mem [0:63];
   integer m;
   always @(*) for (m=0;m<HW;m=m+1) imem_data[m*16 +: 16] = mem[(imem_addr>>1)+m];

   reg saw120;

   initial begin
      for (i=0;i<64;i=i+1) mem[i]=16'h0001;             // c.nop fill
      mem[0]=16'h0293;  mem[1]=16'h0140;                // addi x5,x0,20
      mem[2]=16'h0313;  mem[3]=16'h0013;                // addi x6,x6,1     (loop head, PC4)
      mem[4]=16'h8293;  mem[5]=16'hFFF2;                // addi x5,x5,-1
      mem[6]=16'h9CE3;  mem[7]=16'hFE02;                // bne  x5,x0,-8
      mem[8]=16'h0E13;  mem[9]=16'h0643;                // addi x28,x6,100  (PC16)
      mem[10]=16'h00EF; mem[11]=16'h00C0;               // jal  x1,+12      (PC20 -> f@32)
      mem[12]=16'h006F; mem[13]=16'h0000;               // jal  x0,0        (PC24: park)
      mem[16]=16'h8067; mem[17]=16'h0000;               // jalr x0,0(x1)    (f@32: ret -> 24)

      saw120=0; redirs=0;
      reset=1; @(negedge clk); @(negedge clk); reset=0;

      for (c=0; c<400; c=c+1) begin
         @(negedge clk);
         if (redirect) redirs=redirs+1;
         for (k=0;k<IW;k=k+1)
            if (wb_valid[k] && wb_val[k*64+:64]==64'd120) saw120=1;
      end

      if (!saw120) begin $display("FAIL: x28=120 never written (loop count wrong / squash broke arch state)"); errs=errs+1; end
      if (redirs > 7)  begin $display("FAIL: %0d redirects -- predictor not engaging (unpredicted would be >=21)", redirs); errs=errs+1; end
      if (redirs == 0) begin $display("FAIL: zero redirects -- test not exercising resolution at all"); errs=errs+1; end

      if (errs==0) $display("bp: ALL TESTS PASSED (%0d redirects for 19 taken branches + call/ret/park)", redirs);
      else         $display("bp: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
