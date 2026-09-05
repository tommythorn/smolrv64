`timescale 1ns/1ps
`default_nettype none

// Directed tests for the fetch-window aligner. Halfword 0x..03 (==11) is a
// 32-bit opcode low half; 0x..01 (==01) is an RVC. Cases cover: full RVC bundle,
// full 32b bundle, mixed lengths, a 32b op straddling the window end (must NOT be
// consumed -> carries to next window), and short `avail` (fetch bubble / edge).
module tb;
   localparam IW=4, HW=8, PCW=64, SEQW=8, PBW=4;

   reg  [HW*16-1:0]  hwin;
   reg  [PBW-1:0]    avail;
   reg  [PCW-1:0]    base_pc;
   reg  [SEQW-1:0]   base_seq;
   wire [IW-1:0]     valid;
   wire [IW*32-1:0]  inst;
   wire [IW*PCW-1:0] pc;
   wire [IW*SEQW-1:0] seq;
   wire [PBW-1:0]    consumed;
   integer errs = 0;

   aligner #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW)) dut
     (.hwin(hwin), .avail(avail), .base_pc(base_pc), .base_seq(base_seq),
      .solo_all(1'b0), .bytes_late(1'b0),
      .valid(valid), .inst(inst), .pc(pc), .seq(seq), .consumed(consumed));

   reg [15:0] h [0:HW-1];
   integer i;
   task drive(input [PBW-1:0] av);
      begin for (i=0;i<HW;i=i+1) hwin[i*16 +: 16] = h[i]; avail=av; #1; end
   endtask
   task clr; begin for (i=0;i<HW;i=i+1) h[i]=16'h0000; end endtask
   task ckv(input [127:0] nm, input [IW-1:0] got, exp);
      begin if (got!==exp) begin $display("FAIL %0s valid=%b exp %b",nm,got,exp); errs=errs+1; end end
   endtask
   task ckc(input [127:0] nm, input [PBW-1:0] got, exp);
      begin if (got!==exp) begin $display("FAIL %0s consumed=%0d exp %0d",nm,got,exp); errs=errs+1; end end
   endtask
   task cki(input [127:0] nm, input integer k, input [31:0] exp);
      begin if (inst[k*32+:32]!==exp) begin $display("FAIL %0s inst[%0d]=%h exp %h",nm,k,inst[k*32+:32],exp); errs=errs+1; end end
   endtask
   task ckp(input [127:0] nm, input integer k, input [PCW-1:0] exp);
      begin if (pc[k*PCW+:PCW]!==exp) begin $display("FAIL %0s pc[%0d]=%0d exp %0d",nm,k,pc[k*PCW+:PCW],exp); errs=errs+1; end end
   endtask

   initial begin
      base_pc = 64'h8000_0000; base_seq = 8'd0;

      // 1: four RVC
      clr; h[0]=16'h4501; h[1]=16'h4505; h[2]=16'h4509; h[3]=16'h450d; drive(8);
      ckv("rvc4",valid,4'b1111); ckc("rvc4",consumed,4);
      cki("rvc4",0,32'h45054501); ckp("rvc4",1,64'h8000_0002); ckp("rvc4",3,64'h8000_0006);
      if (seq[3*SEQW+:SEQW]!==8'd3) begin $display("FAIL rvc4 seq3"); errs=errs+1; end

      // 2: four 32-bit
      clr; h[0]=16'h0093; h[2]=16'h1093; h[4]=16'h2093; h[6]=16'h3093; drive(8);
      ckv("w4",valid,4'b1111); ckc("w4",consumed,8);
      cki("w4",0,32'h00000093); cki("w4",1,32'h00001093);
      ckp("w4",1,64'h8000_0004); ckp("w4",3,64'h8000_000c);

      // 3: mixed 32, rvc, 32, rvc
      clr; h[0]=16'h0093; h[2]=16'h4501; h[3]=16'h2093; h[5]=16'h4505; drive(8);
      ckv("mix",valid,4'b1111); ckc("mix",consumed,6);
      cki("mix",0,32'h00000093); cki("mix",2,32'h00002093);
      ckp("mix",0,64'h8000_0000); ckp("mix",1,64'h8000_0004);
      ckp("mix",2,64'h8000_0006); ckp("mix",3,64'h8000_000a);

      // 4: three 32b then a 32b straddling the window end (avail=7) -> not consumed
      clr; h[0]=16'h0093; h[2]=16'h1093; h[4]=16'h2093; h[6]=16'h3093; drive(7);
      ckv("straddle",valid,4'b0111); ckc("straddle",consumed,6);

      // 5: empty window
      clr; drive(0);
      ckv("empty",valid,4'b0000); ckc("empty",consumed,0);

      // 6: four RVC but only 3 halfwords available
      clr; h[0]=16'h4501; h[1]=16'h4505; h[2]=16'h4509; h[3]=16'h450d; drive(3);
      ckv("short",valid,4'b0111); ckc("short",consumed,3);

      // 7: RVC then a 32b straddling immediately (avail=2) -> only the RVC, carry 1
      clr; h[0]=16'h4501; h[1]=16'h0093; drive(2);
      ckv("carry",valid,4'b0001); ckc("carry",consumed,1);

      if (errs==0) $display("aligner: ALL TESTS PASSED");
      else         $display("aligner: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
