`timescale 1ns/1ps
`default_nettype none

// Human-readable pipeline trace of the full core running a program. Peeks
// backend_top's internal stage wires (no DUT changes) and prints, per cycle:
//   fetch PC | DISPATCH slot(seq->pdst) | ISSUE shard(seq->pdst) | WB pr=val | REDIRECT
// Uses the tb_branch program so the redirect + rollback are visible in the trace.
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
   integer i, k, c;

   backend_top #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW), .PBITS(PBITS), .RESET_PC(0)) dut
     (.clk(clk), .reset(reset), .imem_addr(imem_addr), .imem_data(imem_data),
      .imem_avail(imem_avail), .hw_ip(12'd0), .mtime(64'd0), .dmem_rdata(64'd0), .dmem_rvalid(1'b1), .dmem_wready(1'b1), .wb_valid(wb_valid), .wb_pr(wb_pr), .wb_val(wb_val),
      .redirect(redirect), .redirect_target(redirect_target));

   reg [15:0] mem [0:63];
   integer m;
   always @(*) for (m=0;m<HW;m=m+1) imem_data[m*16 +: 16] = mem[(imem_addr>>1)+m];

   // one cycle of trace, sampled mid-cycle (combinational stage signals are stable)
   task trace(input integer cyc);
      begin
         $write("c%2d F=0x%03h |", cyc, imem_addr[11:0]);
         $write(" DISP:");
         for (k=0;k<IW;k=k+1)
            if (dut.r_valid[k]) begin
               if (dut.r_rd_v[k]) $write(" s%0d#%0d->p%0d", k, dut.r_seq[k*SEQW+:SEQW], dut.pdst[k*PBITS+:PBITS]);
               else               $write(" s%0d#%0d(nrd)", k, dut.r_seq[k*SEQW+:SEQW]);
            end
         $write(" | ISS:");
         for (k=0;k<IW;k=k+1)
            if (dut.iss_valid[k]) $write(" sh%0d#%0d", k, dut.iss_seq[k*SEQW+:SEQW]);
         $write(" | WB:");
         for (k=0;k<IW;k=k+1)
            if (wb_valid[k]) $write(" p%0d=%0d", wb_pr[k*PBITS+:PBITS], wb_val[k*64+:64]);
         if (redirect) $write("   ***REDIRECT -> 0x%03h***", redirect_target[11:0]);
         $write("\n");
      end
   endtask

   initial begin
      for (i=0;i<64;i=i+1) mem[i]=16'h0001;             // c.nop
      mem[0]=16'h0513; mem[1]=16'h0010;                 // addi a0,x0,1
      mem[2]=16'h0593; mem[3]=16'h0020;                 // addi a1,x0,2
      mem[4]=16'h0613; mem[5]=16'h0030;                 // addi a2,x0,3
      mem[6]=16'h0693; mem[7]=16'h0040;                 // addi a3,x0,4
      mem[8]=16'h0063; mem[9]=16'h02A5;                 // beq a0,a0,+32 -> PC48
      mem[10]=16'h0A13; mem[11]=16'h0630;               // addi x20,x0,99  WRONG
      mem[16]=16'h0A13; mem[17]=16'h0620;               // addi x20,x0,98  WRONG (fall-through)
      mem[24]=16'h0A13; mem[25]=16'h0370;               // addi x20,x0,55  TARGET (PC48)
      mem[32]=16'h0B93; mem[33]=16'h021A;               // addi x23,x20,33 (PC64) -> 88

      $display("=== pipeline trace: FETCH | DISPATCH slot#seq->pdst | ISSUE sh#seq | WB pr=val ===");
      $display("    program: a0..a3=1..4 ; beq a0,a0 (taken)->PC48 ; wrong x20=99/98 ; target x20=55 ; x23=x20+33");
      reset=1; @(negedge clk); @(negedge clk); reset=0;
      for (c=0; c<13; c=c+1) begin   // 13 cycles covers the program (more runs off test mem)
         #1 trace(c);                 // sample just after the (neg)edge, signals settled
         @(negedge clk);
      end
      $finish;
   end
endmodule

`default_nettype wire
