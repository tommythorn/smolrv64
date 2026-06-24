`timescale 1ns/1ps
`default_nettype none

// Full-core end-to-end: a real RV64I program flows fetch->decode->rename->
// schedule->execute and the writeback stream is checked.
//
// Bundle 0 (independent):  addi a0,x0,11 / a1,x0,22 / a2,x0,33 / a3,x0,44
// Bundle 1 (RAW on b0):    addi a4,a0,1  / a5,a1,1  / a6,a2,1  / a7,a3,1
// Rename (phys 0..63 reserved for arch): shard i 1st alloc = 64+i, 2nd = 68+i.
// Expected (pr,val) writebacks:
//   (64,11)(65,22)(66,33)(67,44)  then  (68,12)(69,23)(70,34)(71,45)
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
      .imem_avail(imem_avail), .hw_ip(12'd0), .dmem_rdata(64'd0), .wb_valid(wb_valid), .wb_pr(wb_pr), .wb_val(wb_val),
      .redirect(redirect), .redirect_target(redirect_target));

   reg [15:0] mem [0:63];
   integer m;
   always @(*) for (m=0;m<HW;m=m+1) imem_data[m*16 +: 16] = mem[(imem_addr>>1)+m];

   // expected writebacks
   reg [PBITS-1:0] exp_pr  [0:7];
   reg [63:0]      exp_val [0:7];
   reg             found   [0:7];

   initial begin
      for (i=0;i<64;i=i+1) mem[i]=16'h0001;        // c.nop fill
      mem[0]=16'h0513; mem[1]=16'h00B0;            // addi a0,x0,11
      mem[2]=16'h0593; mem[3]=16'h0160;            // addi a1,x0,22
      mem[4]=16'h0613; mem[5]=16'h0210;            // addi a2,x0,33
      mem[6]=16'h0693; mem[7]=16'h02C0;            // addi a3,x0,44
      mem[8]=16'h0713; mem[9]=16'h0015;            // addi a4,a0,1
      mem[10]=16'h8793; mem[11]=16'h0015;          // addi a5,a1,1
      mem[12]=16'h0813; mem[13]=16'h0016;          // addi a6,a2,1
      mem[14]=16'h8893; mem[15]=16'h0016;          // addi a7,a3,1

      exp_pr[0]=64; exp_val[0]=11; exp_pr[1]=65; exp_val[1]=22;
      exp_pr[2]=66; exp_val[2]=33; exp_pr[3]=67; exp_val[3]=44;
      exp_pr[4]=68; exp_val[4]=12; exp_pr[5]=69; exp_val[5]=23;
      exp_pr[6]=70; exp_val[6]=34; exp_pr[7]=71; exp_val[7]=45;
      for (i=0;i<8;i=i+1) found[i]=0;

      reset=1; @(negedge clk); @(negedge clk); reset=0;

      // run and capture writebacks
      for (c=0; c<16; c=c+1) begin
         @(negedge clk);
         for (k=0;k<IW;k=k+1) if (wb_valid[k]) begin
            for (i=0;i<8;i=i+1)
               if (!found[i] && wb_pr[k*PBITS+:PBITS]===exp_pr[i]
                            && wb_val[k*64+:64]===exp_val[i]) begin
                  found[i]=1;
                  $display("  wb pr=%0d val=%0d  (matched exp[%0d])",
                           wb_pr[k*PBITS+:PBITS], wb_val[k*64+:64], i);
               end
         end
      end

      for (i=0;i<8;i=i+1)
         if (!found[i]) begin
            $display("FAIL: missing writeback pr=%0d val=%0d", exp_pr[i], exp_val[i]);
            errs=errs+1;
         end

      if (errs==0) $display("backend: ALL TESTS PASSED");
      else         $display("backend: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
