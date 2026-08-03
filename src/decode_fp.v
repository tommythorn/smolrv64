`default_nettype none

// FP operation decode: maps an RV64 F/D instruction (OP-FP 0x53 or the FMADD
// family 0x43/47/4b/4f) to CVFPU control + operand routing, ported from SmolRV64's
// start_cvfpu_issue mapping (smolrv64.v ~5624+) and the fpnew enums.
//
//   op[3:0] (fpnew operation_e): FMADD=0 FNMSUB=1 ADD=2 MUL=3 DIV=4 SQRT=5 SGNJ=6
//      MINMAX=7 CMP=8 CLASSIFY=9 F2F=10 F2I=11 I2F=12 ...   fp_format: FP32=0 FP64=1
//   int_fmt: INT32=2 INT64=3
//
// fp_class routes execution: FPU ops go to the shared CVFPU; SGNJ/CMP/MVXF/MVFX/
// FCLASS are cheap and done in-core (as SmolRV64 does) -- decode emits the class and
// the execute stage handles them without the FPU pipeline.
//
// Operand routing (op{0,1,2}_sel): 0=zero 1=frs1 2=frs2 3=frs3. op0_int marks the
// CVFPU operand-0 as coming from the INTEGER rs1 (I2F / FMV.W.X).
module decode_fp
   (input  wire [31:0] insn,
    output reg         fp_valid,        // a recognized FP-arith op (not load/store)
    output reg         use_fpu,         // routes to the shared CVFPU
    output reg  [2:0]  fp_class,        // 0=FPU 1=SGNJ 2=CMP 3=MVXF 4=MVFX 5=FCLASS
    output reg  [3:0]  op,
    output reg         op_mod,
    output reg  [2:0]  src_fmt,
    output reg  [2:0]  dst_fmt,
    output reg  [1:0]  int_fmt,
    output reg  [2:0]  rnd,             // funct3 (rm / sub-op); execute resolves dynamic rm
    output reg  [1:0]  op0_sel,
    output reg  [1:0]  op1_sel,
    output reg  [1:0]  op2_sel,
    output reg         op0_int,         // CVFPU op0 from integer rs1 (I2F / FMV.W.X)
    output reg         wr_fp);          // destination is an FP register (else integer)

   localparam [3:0] FMADD=0, FNMSUB=1, ADD=2, MUL=3, DIV=4, SQRT=5,
                    SGNJo=6, MINMAX=7, CMPo=8, CLASS=9, F2F=10, F2I=11, I2F=12;
   localparam       FP32=3'd0, FP64=3'd1;
   localparam [1:0] INT32=2'd2, INT64=2'd3;
   localparam       C_FPU=3'd0, C_SGNJ=3'd1, C_CMP=3'd2, C_MVXF=3'd3, C_MVFX=3'd4, C_FCLASS=3'd5;
   localparam [1:0] SZERO=2'd0, SFRS1=2'd1, SFRS2=2'd2, SFRS3=2'd3;

   wire [6:0] opcode = insn[6:0];
   wire [4:0] op5    = insn[31:27];      // OP-FP class = funct7[6:2]
   wire       fmt    = insn[25];         // 0=S(FP32) 1=D(FP64)
   wire [2:0] f3     = insn[14:12];
   wire       isfma  = (insn[6:4]==3'b100) && (insn[1:0]==2'b11);  // FMADD/FMSUB/FNMSUB/FNMADD

   always @* begin
      // defaults: not an FP-arith op
      fp_valid=1'b0; use_fpu=1'b0; fp_class=C_FPU; op=ADD; op_mod=1'b0;
      src_fmt=fmt?FP64:FP32; dst_fmt=fmt?FP64:FP32; int_fmt={1'b1,insn[21]};
      rnd=f3; op0_sel=SZERO; op1_sel=SFRS1; op2_sel=SFRS2; op0_int=1'b0; wr_fp=1'b1;

      if (isfma) begin
         fp_valid=1'b1; use_fpu=1'b1; fp_class=C_FPU;
         op = insn[3] ? FNMSUB : FMADD;  op_mod = insn[2];
         src_fmt = fmt?FP64:FP32; dst_fmt = fmt?FP64:FP32;
         op0_sel=SFRS1; op1_sel=SFRS2; op2_sel=SFRS3; wr_fp=1'b1;
      end else if (opcode == 7'b1010011) begin
         fp_valid=1'b1;
         case (op5)
           5'b00000,5'b00001: begin                 // FADD / FSUB
              use_fpu=1'b1; op=ADD; op_mod=op5[0];
              op0_sel=SZERO; op1_sel=SFRS1; op2_sel=SFRS2; end
           5'b00010: begin use_fpu=1'b1; op=MUL;     // FMUL
              op0_sel=SFRS1; op1_sel=SFRS2; op2_sel=SZERO; end
           5'b00011: begin use_fpu=1'b1; op=DIV;     // FDIV
              op0_sel=SFRS1; op1_sel=SFRS2; op2_sel=SZERO; end
           5'b01011: begin use_fpu=1'b1; op=SQRT;    // FSQRT
              op0_sel=SFRS1; op1_sel=SZERO; op2_sel=SZERO; end
           5'b00101: begin use_fpu=1'b1; op=MINMAX;  // FMIN/FMAX (rnd carries the sub-op)
              op0_sel=SFRS1; op1_sel=SFRS2; op2_sel=SZERO; end
           5'b01000: begin use_fpu=1'b1; op=F2F;     // FCVT.S.D / FCVT.D.S
              dst_fmt=fmt?FP64:FP32; src_fmt=fmt?FP32:FP64;
              op0_sel=SFRS1; op1_sel=SZERO; op2_sel=SZERO; end
           5'b11000: begin use_fpu=1'b1; op=F2I;     // FCVT.W[U]/L[U].fp -> int rd
              op_mod=insn[20]; src_fmt=fmt?FP64:FP32; int_fmt={1'b1,insn[21]};
              op0_sel=SFRS1; op1_sel=SZERO; op2_sel=SZERO; wr_fp=1'b0; end
           5'b11010: begin use_fpu=1'b1; op=I2F;     // FCVT.fp.W[U]/L[U] <- int rs1
              op_mod=insn[20]; dst_fmt=fmt?FP64:FP32; int_fmt={1'b1,insn[21]};
              op0_sel=SFRS1; op0_int=1'b1; op1_sel=SZERO; op2_sel=SZERO; end
           5'b00100: begin use_fpu=1'b0; fp_class=C_SGNJ;  end          // FSGNJ/N/X (in-core)
           5'b10100: begin use_fpu=1'b0; fp_class=C_CMP;  wr_fp=1'b0; end // FEQ/FLT/FLE -> int
           5'b11100: begin use_fpu=1'b0; wr_fp=1'b0;                    // FMV.X.* / FCLASS -> int
              fp_class = (f3==3'b001) ? C_FCLASS : C_MVXF; end
           5'b11110: begin use_fpu=1'b0; fp_class=C_MVFX; op0_int=1'b1; end // FMV.*.X <- int
           default:  fp_valid=1'b0;                                     // unrecognized OP-FP
         endcase
      end
   end
endmodule

`default_nettype wire
