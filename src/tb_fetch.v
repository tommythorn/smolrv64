`timescale 1ns/1ps
`default_nettype none

// Fetch streaming test over a known mixed RVC/32b program in a behavioral
// halfword memory. Checks: per-bundle slot PCs/seqs, PC continuity across
// cycles (next base = prev base + 2*consumed, carry-free), stall holds the
// bundle, and redirect jumps PC+seq.
//
// Program halfwords (32b op = lo with [1:0]==11 then a 0x0000 hi; RVC = ..01):
//   0: c.li        @0   (1 hw)
//   1: <32b>       @2   (2 hw)
//   3: c.*         @6
//   4: c.*         @8
//   5: <32b>       @10
//   7: c.*         @14
//   8: <32b>       @16
//   10:c.* c.* c.* @20,22,24 ... rest c.nop
module tb;
   localparam IW=4, HW=8, PCW=64, SEQW=8, PBW=4;

   reg                clk=0; always #5 clk=~clk;
   reg                reset, redirect;
   reg  [PCW-1:0]     redirect_pc;
   reg  [SEQW-1:0]    redirect_seq;
   reg                ready;
   wire [PCW-1:0]     imem_addr;
   reg  [HW*16-1:0]   imem_data;
   wire [PBW-1:0]     imem_avail = 4'd8;
   wire               valid;
   wire [IW-1:0]      slot_valid;
   wire [IW*32-1:0]   inst;
   wire [IW*PCW-1:0]  pc;
   wire [IW*SEQW-1:0] seq;
   integer errs=0;

   fetch #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW), .RESET_PC(0)) dut
     (.clk(clk), .reset(reset), .redirect(redirect), .redirect_pc(redirect_pc),
      .redirect_seq(redirect_seq), .solo_all(1'b0), .irq_inject(1'b0),
      .pred_v(1'b0), .apred_v(1'b0), .pred_tgt(64'd0),
      .imem_addr(imem_addr), .imem_data(imem_data),
      .imem_avail(imem_avail), .imem_ok(1'b1), .ready(ready), .valid(valid),
      .slot_valid(slot_valid), .inst(inst), .pc(pc), .seq(seq));

   // behavioral halfword memory
   reg [15:0] mem [0:63];
   integer m;
   always @(*) for (m=0;m<HW;m=m+1) imem_data[m*16 +: 16] = mem[(imem_addr>>1)+m];

   function [PCW-1:0] PC(input integer k); PC = pc[k*PCW +: PCW]; endfunction
   function [SEQW-1:0] SQ(input integer k); SQ = seq[k*SEQW +: SEQW]; endfunction
   task ckpc(input [127:0] nm, input integer k, input [PCW-1:0] e);
      begin if (PC(k)!==e) begin $display("FAIL %0s pc[%0d]=%0d exp %0d",nm,k,PC(k),e); errs=errs+1; end end
   endtask
   task ckseq(input [127:0] nm, input integer k, input [SEQW-1:0] e);
      begin if (SQ(k)!==e) begin $display("FAIL %0s seq[%0d]=%0d exp %0d",nm,k,SQ(k),e); errs=errs+1; end end
   endtask

   integer i;
   initial begin
      for (i=0;i<64;i=i+1) mem[i]=16'h0001;          // c.nop fill
      mem[0]=16'h4501; mem[1]=16'h0093; mem[2]=16'h0000; mem[3]=16'h4505;
      mem[4]=16'h4509; mem[5]=16'h1093; mem[6]=16'h0000; mem[7]=16'h450d;
      mem[8]=16'h2093; mem[9]=16'h0000; mem[10]=16'h4511; mem[11]=16'h4515;
      mem[12]=16'h4519; mem[13]=16'h451d;

      reset=1; ready=0; redirect=0; redirect_pc=0; redirect_seq=0;
      @(negedge clk); reset=0; ready=1;

      // bundle @ pc=0 : c.li, 32b, c, c  -> pcs 0,2,6,8
      #1;
      if (slot_valid!==4'b1111) begin $display("FAIL b0 slot_valid=%b",slot_valid); errs=errs+1; end
      if (!valid) begin $display("FAIL b0 !valid"); errs=errs+1; end
      ckpc("b0",0,0); ckpc("b0",1,2); ckpc("b0",2,6); ckpc("b0",3,8);
      ckseq("b0",0,0); ckseq("b0",3,3);

      @(negedge clk);   // fired -> pc_q=10, seq_q=4
      // bundle @ pc=10 : 32b, c, 32b, c -> pcs 10,14,16,20
      #1;
      ckpc("b1",0,10); ckpc("b1",1,14); ckpc("b1",2,16); ckpc("b1",3,20);
      ckseq("b1",0,4); ckseq("b1",3,7);

      @(negedge clk);   // fired -> pc_q=22, seq_q=8
      #1; ckpc("b2",0,22); ckseq("b2",0,8);

      // stall: ready low holds the bundle (pc_q frozen at 22)
      ready=0;
      @(negedge clk);
      #1; ckpc("stall",0,22); ckseq("stall",0,8);
      if (!valid) begin $display("FAIL stall !valid (must hold)"); errs=errs+1; end

      // redirect wins: jump to pc=6, seq=99
      ready=1; redirect=1; redirect_pc=64'd6; redirect_seq=8'd99;
      @(negedge clk); redirect=0;
      #1; ckpc("redir",0,6); ckseq("redir",0,99);

      @(negedge clk);
      if (errs==0) $display("fetch: ALL TESTS PASSED");
      else         $display("fetch: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
