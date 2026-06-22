`timescale 1ns/1ps
`default_nettype none

// Directed tests for the combinational RV64 multiply (MUL/MULH/MULHSU/MULHU/MULW).
module tb;
   reg  [63:0] rs1, rs2;
   reg  [2:0]  f3;
   reg         is_w;
   wire [63:0] result;
   integer errs = 0;

   mul dut (.rs1(rs1), .rs2(rs2), .f3(f3), .is_w(is_w), .result(result));

   task ck(input [200:0] nm, input [63:0] a, input [63:0] bb, input w, input [2:0] func, input [63:0] exp);
      begin
         rs1=a; rs2=bb; is_w=w; f3=func; #1;
         if (result!==exp) begin
            $display("FAIL %0s: rs1=%h rs2=%h w=%b f3=%b -> %h exp %h", nm,a,bb,w,func,result,exp);
            errs=errs+1;
         end
      end
   endtask

   localparam M=3'b000, MH=3'b001, MHSU=3'b010, MHU=3'b011;

   initial begin
      ck("mul.pos",     64'd6, 64'd7, 0, M, 64'd42);
      ck("mul.neg",     -64'd3, 64'd5, 0, M, -64'd15);                 // low bits of -15
      ck("mulh.2^62*4", 64'h4000000000000000, 64'd4, 0, MH,  64'd1);   // 2^64 -> high=1
      ck("mulh.-1*-1",  ~64'd0, ~64'd0,            0, MH,  64'd0);
      ck("mulhu.max2",  ~64'd0, ~64'd0,            0, MHU, 64'hFFFFFFFFFFFFFFFE);
      ck("mulhsu.-1*2", ~64'd0, 64'd2,             0, MHSU,64'hFFFFFFFFFFFFFFFF);
      ck("mulw.-1*2",   64'hFFFFFFFF, 64'd2,       1, M, 64'hFFFFFFFFFFFFFFFE);
      ck("mulw.hi-junk",64'hDEAD0000_00000003, 64'hBEEF0000_00000004, 1, M, 64'd12);

      if (errs==0) $display("mul: ALL TESTS PASSED");
      else         $display("mul: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
