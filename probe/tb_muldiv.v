`timescale 1ns/1ps
`default_nettype none

// Directed tests for the RV64 M datapath: all 13 ops, with the spec's tricky cases
// (signed/unsigned high products, truncate-toward-zero, REM sign-of-dividend,
// divide-by-zero, signed overflow, W sign-extension). Expected values hand-computed.
module tb;
   reg  [63:0] rs1, rs2;
   reg  [2:0]  f3;
   reg         is_w;
   wire [63:0] result;
   integer errs = 0;

   muldiv dut (.rs1(rs1), .rs2(rs2), .f3(f3), .is_w(is_w), .result(result));

   task ck(input [200:0] nm, input [63:0] a, input [63:0] bb, input w, input [2:0] func, input [63:0] exp);
      begin
         rs1=a; rs2=bb; is_w=w; f3=func; #1;
         if (result!==exp) begin
            $display("FAIL %0s: rs1=%h rs2=%h w=%b f3=%b -> %h exp %h", nm,a,bb,w,func,result,exp);
            errs=errs+1;
         end
      end
   endtask

   localparam M=3'b000, MH=3'b001, MHSU=3'b010, MHU=3'b011, D=3'b100, DU=3'b101, R=3'b110, RU=3'b111;

   initial begin
      // ---- MUL (low 64) ----
      ck("mul.pos",   64'd6, 64'd7, 0, M, 64'd42);
      ck("mul.neg",   -64'd3, 64'd5, 0, M, -64'd15);                 // low bits of -15
      // ---- MULH / MULHU / MULHSU (high 64) ----
      ck("mulh.2^62*4", 64'h4000000000000000, 64'd4, 0, MH,  64'd1); // 2^64 -> high=1
      ck("mulh.-1*-1",  ~64'd0, ~64'd0,            0, MH,  64'd0);   // (-1)*(-1)=1 -> high=0
      ck("mulhu.max2",  ~64'd0, ~64'd0,            0, MHU, 64'hFFFFFFFFFFFFFFFE);
      ck("mulhsu.-1*2", ~64'd0, 64'd2,             0, MHSU,64'hFFFFFFFFFFFFFFFF); // -2 -> high all ones
      // ---- DIV / REM (signed, trunc toward zero, rem sign = dividend) ----
      ck("div.20/3",   64'd20, 64'd3,  0, D, 64'd6);
      ck("div.-20/3",  -64'd20, 64'd3, 0, D, -64'd6);
      ck("rem.20/3",   64'd20, 64'd3,  0, R, 64'd2);
      ck("rem.-20/3",  -64'd20, 64'd3, 0, R, -64'd2);
      // ---- divide-by-zero (DIV->-1, REM->dividend; DIVU->all-ones, REMU->dividend) ----
      ck("div.x/0",    64'd5,  64'd0,  0, D,  ~64'd0);
      ck("rem.x/0",    64'd5,  64'd0,  0, R,  64'd5);
      ck("divu.x/0",   64'd5,  64'd0,  0, DU, ~64'd0);
      ck("remu.x/0",   64'd5,  64'd0,  0, RU, 64'd5);
      // ---- unsigned div ----
      ck("divu.max/2", ~64'd0, 64'd2,  0, DU, 64'h7FFFFFFFFFFFFFFF);
      // ---- signed overflow (MIN / -1) ----
      ck("div.ovf",    64'h8000000000000000, ~64'd0, 0, D, 64'h8000000000000000);
      ck("rem.ovf",    64'h8000000000000000, ~64'd0, 0, R, 64'd0);
      // ---- W variants (operate on low 32, sign-extend result) ----
      ck("mulw.-1*2",  64'hFFFFFFFF, 64'd2,    1, M, 64'hFFFFFFFFFFFFFFFE);
      ck("divw.-20/3", 64'hFFFFFFEC, 64'd3,    1, D, 64'hFFFFFFFFFFFFFFFA); // -6 sext
      ck("remw.-20/3", 64'hFFFFFFEC, 64'd3,    1, R, 64'hFFFFFFFFFFFFFFFE); // -2 sext
      ck("divuw.x/0",  64'd5,  64'd0,          1, DU, 64'hFFFFFFFFFFFFFFFF);
      ck("remuw.7/4",  64'd7,  64'd4,          1, RU, 64'd3);
      // upper bits of operands must be ignored by W ops:
      ck("mulw.hi-junk",64'hDEAD0000_00000003, 64'hBEEF0000_00000004, 1, M, 64'd12);

      if (errs==0) $display("muldiv: ALL TESTS PASSED (13 ops, spec edge cases)");
      else         $display("muldiv: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
