`timescale 1ns/1ps
`default_nettype none

// End-to-end branch test: redirect + rollback must squash the wrong (fall-through)
// path and keep the correct (target) path.
//
//   PC0..12  : addi a0,a1,a2,a3 = 1,2,3,4         (a0 used by the branch)
//   PC16     : beq a0,a0,+32  -> TAKEN, target PC48   (bundle1 slot0, 16-aligned)
//   PC20     : addi x20,x0,99   WRONG-PATH (same bundle, younger)
//   PC32     : addi x20,x0,98   WRONG-PATH (fall-through bundle)
//   PC48     : addi x20,x0,55   TARGET (correct path)
//   PC64     : addi x23,x20,33  consumer -> 55+33 = 88 if x20 came from the target
//                                          (98/99 would give 131/132 -> squash failed)
// Checks: redirect fired to 0x30; consumer value 88 present; 131 and 132 absent.
module tb;
   localparam IW=4, HW=8, PCW=64, SEQW=8, PBITS=8, PBW=4;

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
   integer errs=0, i, k, c;

   backend_top #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW), .PBITS(PBITS), .RESET_PC(0)) dut
     (.clk(clk), .reset(reset), .imem_addr(imem_addr), .imem_data(imem_data),
      .imem_avail(imem_avail), .dmem_rdata(64'd0), .wb_valid(wb_valid), .wb_pr(wb_pr), .wb_val(wb_val),
      .redirect(redirect), .redirect_target(redirect_target));

   reg [15:0] mem [0:63];
   integer m;
   always @(*) for (m=0;m<HW;m=m+1) imem_data[m*16 +: 16] = mem[(imem_addr>>1)+m];

   reg saw_redir, saw88, saw131, saw132, saw55;
   reg [PCW-1:0] redir_tgt;

   initial begin
      for (i=0;i<64;i=i+1) mem[i]=16'h0001;             // c.nop fill
      mem[0]=16'h0513; mem[1]=16'h0010;                 // addi a0,x0,1
      mem[2]=16'h0593; mem[3]=16'h0020;                 // addi a1,x0,2
      mem[4]=16'h0613; mem[5]=16'h0030;                 // addi a2,x0,3
      mem[6]=16'h0693; mem[7]=16'h0040;                 // addi a3,x0,4
      mem[8]=16'h0063; mem[9]=16'h02A5;                 // beq a0,a0,+32  (-> PC48)
      mem[10]=16'h0A13; mem[11]=16'h0630;               // addi x20,x0,99  WRONG
      // mem[12..15] nop
      mem[16]=16'h0A13; mem[17]=16'h0620;               // addi x20,x0,98  WRONG (fall-through)
      // mem[18..23] nop
      mem[24]=16'h0A13; mem[25]=16'h0370;               // addi x20,x0,55  TARGET (PC48)
      // mem[26..31] nop
      mem[32]=16'h0B93; mem[33]=16'h021A;               // addi x23,x20,33 consumer (PC64)

      saw_redir=0; saw88=0; saw131=0; saw132=0; saw55=0; redir_tgt=0;
      reset=1; @(negedge clk); @(negedge clk); reset=0;

      for (c=0; c<30; c=c+1) begin
         @(negedge clk);
         if (redirect) begin saw_redir=1; redir_tgt=redirect_target; end
         for (k=0;k<IW;k=k+1) if (wb_valid[k]) begin
            case (wb_val[k*64+:64])
               64'd88:  saw88=1;
               64'd131: saw131=1;
               64'd132: saw132=1;
               64'd55:  saw55=1;
            endcase
         end
      end

      if (!saw_redir)            begin $display("FAIL: no redirect fired"); errs=errs+1; end
      else if (redir_tgt!==64'h30) begin $display("FAIL: redirect target=%h exp 0x30",redir_tgt); errs=errs+1; end
      if (!saw55)  begin $display("FAIL: target x20=55 never executed"); errs=errs+1; end
      if (!saw88)  begin $display("FAIL: consumer x23=88 (correct x20) not seen"); errs=errs+1; end
      if (saw131)  begin $display("FAIL: consumer saw x20=98 (wrong-path leaked!)"); errs=errs+1; end
      if (saw132)  begin $display("FAIL: consumer saw x20=99 (wrong-path leaked!)"); errs=errs+1; end

      if (errs==0) $display("branch: ALL TESTS PASSED (redirect+rollback)");
      else         $display("branch: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
