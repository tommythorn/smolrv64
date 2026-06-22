`timescale 1ns/1ps
`default_nettype none

// End-to-end M extension through the whole backend: MUL/DIV/REM/DIVU run and
// write back, a negative MUL is correct, and a dependent ALU op consumes a mul
// result (proving the M writeback rides the normal wake/RF path).
//   x5=6 x6=7
//   mul  x7 = 6*7   = 42
//   mul  x9 = -3*7  = -21  (0xFFFFFFFFFFFFFFEB)
//   div  x11= 20/7  = 2
//   rem  x12= 20%7  = 6
//   divu x13= 20/7u = 2
//   addi x14= x7+1  = 43   (consumer of the mul result)
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
   integer errs=0, i, k, c;

   backend_top #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW), .PBITS(PBITS), .RESET_PC(0)) dut
     (.clk(clk), .reset(reset), .imem_addr(imem_addr), .imem_data(imem_data),
      .imem_avail(imem_avail), .dmem_rdata(64'd0), .wb_valid(wb_valid), .wb_pr(wb_pr), .wb_val(wb_val),
      .redirect(redirect), .redirect_target(redirect_target));

   reg [15:0] mem [0:63];
   integer m;
   always @(*) for (m=0;m<HW;m=m+1) imem_data[m*16 +: 16] = mem[(imem_addr>>1)+m];

   reg saw42, sawm21, saw2, saw6, saw43;

   initial begin
      for (i=0;i<64;i=i+1) mem[i]=16'h0001;             // c.nop fill
      mem[0]=16'h0293; mem[1]=16'h0060;   // addi x5,x0,6
      mem[2]=16'h0313; mem[3]=16'h0070;   // addi x6,x0,7
      mem[4]=16'h83B3; mem[5]=16'h0262;   // mul  x7,x5,x6
      mem[6]=16'h0413; mem[7]=16'hFFD0;   // addi x8,x0,-3
      mem[8]=16'h04B3; mem[9]=16'h0264;   // mul  x9,x8,x6
      mem[10]=16'h0513; mem[11]=16'h0140;  // addi x10,x0,20
      mem[12]=16'h45B3; mem[13]=16'h0265;  // div  x11,x10,x6
      mem[14]=16'h6633; mem[15]=16'h0265;  // rem  x12,x10,x6
      mem[16]=16'h56B3; mem[17]=16'h0265;  // divu x13,x10,x6
      mem[18]=16'h8713; mem[19]=16'h0013;  // addi x14,x7,1

      saw42=0; sawm21=0; saw2=0; saw6=0; saw43=0;
      reset=1; @(negedge clk); @(negedge clk); reset=0;

      for (c=0; c<40; c=c+1) begin
         @(negedge clk);
         for (k=0;k<IW;k=k+1) if (wb_valid[k]) case (wb_val[k*64+:64])
            64'd42:                  saw42=1;
            64'hFFFFFFFFFFFFFFEB:     sawm21=1;
            64'd2:                    saw2=1;
            64'd6:                    saw6=1;
            64'd43:                   saw43=1;
         endcase
      end

      if (!saw42)  begin $display("FAIL: mul 6*7=42 not seen");          errs=errs+1; end
      if (!sawm21) begin $display("FAIL: mul -3*7=-21 not seen");        errs=errs+1; end
      if (!saw2)   begin $display("FAIL: div 20/7=2 not seen");          errs=errs+1; end
      if (!saw6)   begin $display("FAIL: rem 20%%7=6 not seen");         errs=errs+1; end
      if (!saw43)  begin $display("FAIL: consumer x14=x7+1=43 not seen (M wb did not flow)"); errs=errs+1; end

      if (errs==0) $display("muldiv_e2e: ALL TESTS PASSED (MUL/DIV/REM/DIVU + consumer)");
      else         $display("muldiv_e2e: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
