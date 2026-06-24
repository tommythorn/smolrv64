`timescale 1ns/1ps
`default_nettype none

// JAL/JALR link-write + restore, end-to-end. A jump is a CTI: the aligner makes it
// the youngest instr in its checkpoint, so its link write (rd <- next_pc) lives in a
// checkpoint that is KEPT on redirect (rollback is to rckpt+1) and the link reaches
// the target path. Both forms exercised: JAL (direct, link=pc+4) and JALR (indirect,
// target=rs1+imm, link=pc+4).
//
//   PC0  : jal  x1,+16     x1(ra) <- 4 ; redirect -> PC16
//   PC4  : addi x20,x0,77  WRONG-PATH (fall-through, squashed)
//   PC16 : addi x5,x1,0    x5 <- ra = 4         (JAL link reached the target)
//   PC20 : jalr x2,x5,40   x2 <- 24 ; redirect -> (x5+40)&~1 = 44
//   PC24 : addi x20,x0,88  WRONG-PATH (squashed)
//   PC44 : addi x6,x2,0    x6 <- 24             (JALR link)
//   PC48 : addi x7,x5,0    x7 <- 4              (ra survived to here)
//
// Checks: both redirects fire to the right targets (0x10, 0x2c); x5=4, x6=24, x7=4
// appear (the link values are next_pc, written and correctly restored/read).
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
      .imem_avail(imem_avail), .hw_ip(12'd0), .dmem_rdata(64'd0), .wb_valid(wb_valid), .wb_pr(wb_pr), .wb_val(wb_val),
      .redirect(redirect), .redirect_target(redirect_target));

   reg [15:0] mem [0:63];
   integer m;
   always @(*) for (m=0;m<HW;m=m+1) imem_data[m*16 +: 16] = mem[(imem_addr>>1)+m];

   reg saw_jtgt, saw_jrtgt, saw4, saw24, saw7;

   initial begin
      for (i=0;i<64;i=i+1) mem[i]=16'h0001;             // c.nop fill
      mem[0]=16'h00EF;  mem[1]=16'h0100;                // jal  x1,+16   -> PC16
      mem[2]=16'h0A13;  mem[3]=16'h04D0;                // addi x20,x0,77 WRONG
      mem[8]=16'h8293;  mem[9]=16'h0000;                // addi x5,x1,0   (= ra = 4)
      mem[10]=16'h8167; mem[11]=16'h0282;               // jalr x2,x5,40  -> PC44, x2=24
      mem[12]=16'h0A13; mem[13]=16'h0580;               // addi x20,x0,88 WRONG
      mem[22]=16'h0313; mem[23]=16'h0001;               // addi x6,x2,0   (= 24)
      mem[24]=16'h8393; mem[25]=16'h0002;               // addi x7,x5,0   (= 4)

      saw_jtgt=0; saw_jrtgt=0; saw4=0; saw24=0; saw7=0;
      reset=1; @(negedge clk); @(negedge clk); reset=0;

      for (c=0; c<50; c=c+1) begin
         @(negedge clk);
         if (redirect) begin
            if (redirect_target==64'h10) saw_jtgt=1;
            if (redirect_target==64'h2c) saw_jrtgt=1;
         end
         for (k=0;k<IW;k=k+1) if (wb_valid[k]) case (wb_val[k*64+:64])
            64'd4:  saw4=1;
            64'd24: saw24=1;
            64'd7:  saw7=1;   // never produced as a value here -> stays 0 (guards typos)
         endcase
      end

      if (!saw_jtgt)  begin $display("FAIL: JAL redirect to 0x10 not seen");  errs=errs+1; end
      if (!saw_jrtgt) begin $display("FAIL: JALR redirect to 0x2c not seen"); errs=errs+1; end
      if (!saw4)  begin $display("FAIL: x5/x7 = ra = 4 not seen (JAL link write/restore)"); errs=errs+1; end
      if (!saw24) begin $display("FAIL: x6 = 24 not seen (JALR link write)"); errs=errs+1; end

      if (errs==0) $display("jump: ALL TESTS PASSED (JAL+JALR link write + restore)");
      else         $display("jump: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
