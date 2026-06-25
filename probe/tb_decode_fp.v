`timescale 1ns/1ps
`default_nettype none

// Directed unit TB for decode_fp: feed representative RV64 F/D encodings and check
// the decoded CVFPU control (op/op_mod/src_fmt/dst_fmt/use_fpu/class/wr_fp) against
// the SmolRV64 / fpnew mapping.
module tb;
   reg  [31:0] insn;
   wire        fp_valid, use_fpu, op0_int, wr_fp;
   wire [2:0]  fp_class, src_fmt, dst_fmt, rnd;
   wire [3:0]  op;  wire op_mod;  wire [1:0] int_fmt, op0_sel, op1_sel, op2_sel;
   integer errs = 0;

   decode_fp dut (.insn(insn), .fp_valid(fp_valid), .use_fpu(use_fpu), .fp_class(fp_class),
      .op(op), .op_mod(op_mod), .src_fmt(src_fmt), .dst_fmt(dst_fmt), .int_fmt(int_fmt),
      .rnd(rnd), .op0_sel(op0_sel), .op1_sel(op1_sel), .op2_sel(op2_sel),
      .op0_int(op0_int), .wr_fp(wr_fp));

   // encoders
   function [31:0] opfp; input [6:0] f7; input [4:0] rs2; input [4:0] rs1; input [2:0] rm; input [4:0] rd;
      opfp = {f7, rs2, rs1, rm, rd, 7'b1010011}; endfunction
   function [31:0] fma; input [6:0] oc; input [4:0] rs3; input [1:0] fmt; input [4:0] rs2;
                        input [4:0] rs1; input [2:0] rm; input [4:0] rd;
      fma = {rs3, fmt, rs2, rs1, rm, rd, oc}; endfunction

   // fpnew op enums
   localparam FMADD=0, FNMSUB=1, ADD=2, MUL=3, DIV=4, SQRT=5, MINMAX=7, F2F=10, F2I=11, I2F=12;
   localparam C_FPU=0, C_SGNJ=1, C_CMP=2, C_MVXF=3, C_MVFX=4, C_FCLASS=5;

   task chk;  // name, expected: valid, usefpu, class, op, opmod, src, dst, wrfp
      input [127:0] nm; input ev, eu; input [2:0] ec; input [3:0] eo; input em;
      input [2:0] es, ed; input ew;
      begin
         #1;
         if (fp_valid!==ev || use_fpu!==eu || fp_class!==ec || (eu && (op!==eo || op_mod!==em
              || src_fmt!==es || dst_fmt!==ed)) || wr_fp!==ew) begin
            $display("FAIL %0s: valid=%b/%b usefpu=%b/%b class=%0d/%0d op=%0d/%0d mod=%b/%b src=%0d/%0d dst=%0d/%0d wrfp=%b/%b",
               nm, fp_valid,ev, use_fpu,eu, fp_class,ec, op,eo, op_mod,em, src_fmt,es, dst_fmt,ed, wr_fp,ew);
            errs=errs+1;
         end else $display("  ok  %0s", nm);
      end
   endtask

   initial begin
      // FADD.S / FADD.D / FSUB.S
      insn=opfp(7'b0000000,5'd3,5'd2,3'd0,5'd1); chk("FADD.S",1,1,C_FPU,ADD,1'b0,3'd0,3'd0,1'b1);
      insn=opfp(7'b0000001,5'd3,5'd2,3'd0,5'd1); chk("FADD.D",1,1,C_FPU,ADD,1'b0,3'd1,3'd1,1'b1);
      insn=opfp(7'b0000100,5'd3,5'd2,3'd0,5'd1); chk("FSUB.S",1,1,C_FPU,ADD,1'b1,3'd0,3'd0,1'b1);
      // FMUL.D / FDIV.S / FSQRT.S
      insn=opfp(7'b0001001,5'd3,5'd2,3'd0,5'd1); chk("FMUL.D",1,1,C_FPU,MUL,1'b0,3'd1,3'd1,1'b1);
      insn=opfp(7'b0001100,5'd3,5'd2,3'd0,5'd1); chk("FDIV.S",1,1,C_FPU,DIV,1'b0,3'd0,3'd0,1'b1);
      insn=opfp(7'b0101100,5'd0,5'd2,3'd0,5'd1); chk("FSQRT.S",1,1,C_FPU,SQRT,1'b0,3'd0,3'd0,1'b1);
      // FMIN.S (use_fpu)
      insn=opfp(7'b0010100,5'd3,5'd2,3'd0,5'd1); chk("FMIN.S",1,1,C_FPU,MINMAX,1'b0,3'd0,3'd0,1'b1);
      // FCVT.S.D (dst S, src D) / FCVT.D.S (dst D, src S)
      insn=opfp(7'b0100000,5'd1,5'd2,3'd0,5'd1); chk("FCVT.S.D",1,1,C_FPU,F2F,1'b0,3'd1,3'd0,1'b1);
      insn=opfp(7'b0100001,5'd0,5'd2,3'd0,5'd1); chk("FCVT.D.S",1,1,C_FPU,F2F,1'b0,3'd0,3'd1,1'b1);
      // FCVT.W.S (F2I, int rd) / FCVT.S.W (I2F, fp rd, int rs1)
      insn=opfp(7'b1100000,5'd0,5'd2,3'd0,5'd1); chk("FCVT.W.S",1,1,C_FPU,F2I,1'b0,3'd0,3'd0,1'b0);
      insn=opfp(7'b1101000,5'd0,5'd2,3'd0,5'd1); chk("FCVT.S.W",1,1,C_FPU,I2F,1'b0,3'd0,3'd0,1'b1);
      // FMADD.S (op FMADD, mod0) / FMSUB.S (op FMADD, mod1)
      insn=fma(7'h43,5'd4,2'd0,5'd3,5'd2,3'd0,5'd1); chk("FMADD.S",1,1,C_FPU,FMADD,1'b0,3'd0,3'd0,1'b1);
      insn=fma(7'h47,5'd4,2'd0,5'd3,5'd2,3'd0,5'd1); chk("FMSUB.S",1,1,C_FPU,FMADD,1'b1,3'd0,3'd0,1'b1);
      // in-core ops: SGNJ / FCMP / FCLASS / FMV.X.W / FMV.W.X
      insn=opfp(7'b0010000,5'd3,5'd2,3'd0,5'd1); chk("FSGNJ.S",1,0,C_SGNJ,4'd0,1'b0,3'd0,3'd0,1'b1);
      insn=opfp(7'b1010000,5'd3,5'd2,3'd2,5'd1); chk("FEQ.S",  1,0,C_CMP, 4'd0,1'b0,3'd0,3'd0,1'b0);
      insn=opfp(7'b1110000,5'd0,5'd2,3'd1,5'd1); chk("FCLASS.S",1,0,C_FCLASS,4'd0,1'b0,3'd0,3'd0,1'b0);
      insn=opfp(7'b1110000,5'd0,5'd2,3'd0,5'd1); chk("FMV.X.W",1,0,C_MVXF,4'd0,1'b0,3'd0,3'd0,1'b0);
      insn=opfp(7'b1111000,5'd0,5'd2,3'd0,5'd1); chk("FMV.W.X",1,0,C_MVFX,4'd0,1'b0,3'd0,3'd0,1'b1);
      // a non-FP op -> fp_valid=0
      insn=32'h00000033 /*ADD*/; chk("ADD(not-fp)",0,0,C_FPU,4'd0,1'b0,3'd0,3'd0,1'b1);

      // spot-check operand routing
      #1; insn=opfp(7'b0000000,5'd3,5'd2,3'd0,5'd1); #1;  // FADD: (ZERO,FRS1,FRS2)
      if (op0_sel!==2'd0 || op1_sel!==2'd1 || op2_sel!==2'd2) begin $display("FAIL FADD routing"); errs=errs+1; end
      insn=fma(7'h43,5'd4,2'd0,5'd3,5'd2,3'd0,5'd1); #1;  // FMA: (FRS1,FRS2,FRS3)
      if (op0_sel!==2'd1 || op1_sel!==2'd2 || op2_sel!==2'd3) begin $display("FAIL FMA routing"); errs=errs+1; end
      insn=opfp(7'b1101000,5'd0,5'd2,3'd0,5'd1); #1;      // FCVT.S.W: op0 from INT rs1
      if (!op0_int) begin $display("FAIL FCVT.S.W op0_int"); errs=errs+1; end

      if (errs==0) $display("DECODE_FP-TB: ALL TESTS PASSED"); else $display("DECODE_FP-TB FAIL (%0d)", errs);
      $finish;
   end
endmodule

`default_nettype wire
