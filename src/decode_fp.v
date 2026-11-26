`default_nettype none

// FP operation decode: maps an RV64 F/D instruction (OP-FP 0x53 or the FMADD
// family 0x43/47/4b/4f) to the FPU's control and operand routing.
//
//   op[3:0]: FMADD=0 FNMSUB=1 ADD=2 MUL=3 DIV=4 SQRT=5 SGNJ=6 MINMAX=7 CMP=8
//      CLASSIFY=9 F2F=10 F2I=11 I2F=12 F2IM=13 (fcvtmod.w.d)   fp_format: FP32=0 FP64=1 FP16=2
//   int_fmt: INT32=2 INT64=3
//
// fp_class routes execution: FPU ops go to fp_unit; SGNJ/CMP/MVXF/MVFX/
// FCLASS/MINMAX/FLI/FROUND are cheap and done in-core -- decode emits the class and
// the execute stage handles them without the FPU pipeline.
//
// Operand routing (op{0,1,2}_sel): 0=zero 1=frs1 2=frs2 3=frs3. op0_int marks the
// FPU's operand 0 as coming from the INTEGER rs1 (I2F / FMV.W.X).
module decode_fp
   (input  wire [31:0] insn,
    output reg         fp_valid,        // a recognized FP-arith op (not load/store)
    output reg         use_fpu,         // routes to fp_unit
    output reg  [3:0]  fp_class,        // 0=FPU 1=SGNJ 2=CMP 3=MVXF 4=MVFX 5=FCLASS 6=MINMAX 7=FLI 8=FROUND
    output reg  [3:0]  op,
    output reg         op_mod,
    output reg  [2:0]  src_fmt,
    output reg  [2:0]  dst_fmt,
    output reg  [1:0]  int_fmt,
    output reg  [2:0]  rnd,             // funct3 (rm / sub-op); execute resolves dynamic rm
    output reg  [1:0]  op0_sel,
    output reg  [1:0]  op1_sel,
    output reg  [1:0]  op2_sel,
    output reg         op0_int,         // FPU op0 from integer rs1 (I2F / FMV.W.X)
    output reg         wr_fp);          // destination is an FP register (else integer)

   localparam [3:0] FMADD=0, FNMSUB=1, ADD=2, MUL=3, DIV=4, SQRT=5,
                    SGNJo=6, MINMAX=7, CMPo=8, CLASS=9, F2F=10, F2I=11, I2F=12, F2IM=13;
   localparam       FP32=3'd0, FP64=3'd1, FP16=3'd2;
   localparam [1:0] INT32=2'd2, INT64=2'd3;
   localparam [3:0] C_FPU=4'd0, C_SGNJ=4'd1, C_CMP=4'd2, C_MVXF=4'd3, C_MVFX=4'd4, C_FCLASS=4'd5,
                    C_MINMAX=4'd6, C_FLI=4'd7, C_FROUND=4'd8;
   localparam [1:0] SZERO=2'd0, SFRS1=2'd1, SFRS2=2'd2, SFRS3=2'd3;

   wire [6:0] opcode = insn[6:0];
   wire [4:0] op5    = insn[31:27];      // OP-FP class = funct7[6:2]
   wire [1:0] fm     = insn[26:25];      // 0=S 1=D 2=H 3=Q
   wire       fmt    = insn[25];         // within S and D: 1=D
   wire       sd     = ~insn[26];        // S or D
   wire [4:0] rs2    = insn[24:20];
   wire [2:0] f3     = insn[14:12];
   wire       isfma  = (insn[6:4]==3'b100) && (insn[1:0]==2'b11);  // FMADD/FMSUB/FNMSUB/FNMADD
   // a format field's encoding (S, D, H; Q is never valid here)
   function [2:0] fenc(input [1:0] x);
      fenc = (x == 2'd1) ? FP64 : (x == 2'd2) ? FP16 : FP32;
   endfunction

   always @* begin
      // defaults: not an FP-arith op
      fp_valid=1'b0; use_fpu=1'b0; fp_class=C_FPU; op=ADD; op_mod=1'b0;
      src_fmt=fmt?FP64:FP32; dst_fmt=fmt?FP64:FP32; int_fmt={1'b1,insn[21]};
      rnd=f3; op0_sel=SZERO; op1_sel=SFRS1; op2_sel=SFRS2; op0_int=1'b0; wr_fp=1'b1;

      if (isfma) begin
         fp_valid=sd; use_fpu=1'b1; fp_class=C_FPU;
         op = insn[3] ? FNMSUB : FMADD;  op_mod = insn[2];
         op0_sel=SFRS1; op1_sel=SFRS2; op2_sel=SFRS3; wr_fp=1'b1;
      end else if (opcode == 7'b1010011) begin
         case (op5)
           5'b00000,5'b00001: begin fp_valid=sd;     // FADD / FSUB
              use_fpu=1'b1; op=ADD; op_mod=op5[0];
              op0_sel=SZERO; op1_sel=SFRS1; op2_sel=SFRS2; end
           5'b00010: begin fp_valid=sd; use_fpu=1'b1; op=MUL;     // FMUL
              op0_sel=SFRS1; op1_sel=SFRS2; op2_sel=SZERO; end
           5'b00011: begin fp_valid=sd; use_fpu=1'b1; op=DIV;     // FDIV
              op0_sel=SFRS1; op1_sel=SFRS2; op2_sel=SZERO; end
           5'b01011: begin fp_valid=sd & (rs2 == 5'd0); use_fpu=1'b1; op=SQRT;    // FSQRT
              op0_sel=SFRS1; op1_sel=SZERO; op2_sel=SZERO; end
           // FMIN/FMAX (funct3 00x) and Zfa's FMINM/FMAXM (01x), in-core
           5'b00101: begin fp_valid=sd & ~f3[2]; fp_class=C_MINMAX; end
           5'b01000: if (rs2[4:1] == 4'b0010) begin                // Zfa FROUND / FROUNDNX (in-core)
                        fp_valid=sd; fp_class=C_FROUND; end
                     else begin                                   // FCVT between S, D and H (Zfhmin)
                        fp_valid=(rs2[4:2] == 3'd0) & (rs2[1:0] != 2'd3) & (fm != 2'd3) & (rs2[1:0] != fm);
                        use_fpu=1'b1; op=F2F; dst_fmt=fenc(fm); src_fmt=fenc(rs2[1:0]);
                        op0_sel=SFRS1; op1_sel=SZERO; op2_sel=SZERO; end
           5'b11000: begin use_fpu=1'b1; wr_fp=1'b0;               // FCVT.W[U]/L[U].fp -> int rd
              op0_sel=SFRS1; op1_sel=SZERO; op2_sel=SZERO;
              if (rs2 == 5'b01000) begin                          // Zfa FCVTMOD.W.D: RTZ only
                 fp_valid=(fm == 2'd1) & (f3 == 3'b001); op=F2IM; int_fmt=INT32; end
              else begin
                 fp_valid=sd & (rs2[4:2] == 3'd0); op=F2I; op_mod=rs2[0]; int_fmt={1'b1,rs2[1]}; end
           end
           5'b11010: begin fp_valid=sd & (rs2[4:2] == 3'd0); use_fpu=1'b1; op=I2F;     // FCVT.fp.W[U]/L[U] <- int rs1
              op_mod=rs2[0]; int_fmt={1'b1,rs2[1]};
              op0_sel=SFRS1; op0_int=1'b1; op1_sel=SZERO; op2_sel=SZERO; end
           5'b00100: begin fp_valid=sd & ~f3[2] & (f3[1:0] != 2'b11); fp_class=C_SGNJ; end       // FSGNJ/N/X (in-core)
           // FEQ/FLT/FLE (funct3 010/001/000) and Zfa's quiet FLTQ/FLEQ (101/100) -> int
           5'b10100: begin fp_valid=sd & (f3 != 3'b011) & (f3[2:1] != 2'b11); fp_class=C_CMP; wr_fp=1'b0; end
           5'b11100: begin wr_fp=1'b0;                            // FMV.X.W/D/H (Zfhmin's .H) / FCLASS -> int
              fp_valid=(rs2 == 5'd0) & ((f3 == 3'b000) ? (fm != 2'd3) : (f3 == 3'b001) & sd);
              fp_class = (f3==3'b001) ? C_FCLASS : C_MVXF; end
           5'b11110: if (rs2 == 5'd1) begin                      // Zfa FLI: rs1 is the table's index
                        fp_valid=sd & (f3 == 3'b000); fp_class=C_FLI; end
                     else begin                                   // FMV.W/D/H.X <- int (Zfhmin's .H)
                        fp_valid=(rs2 == 5'd0) & (f3 == 3'b000) & (fm != 2'd3);
                        fp_class=C_MVFX; op0_int=1'b1; end
           default: ;                                             // unrecognized OP-FP
         endcase
      end
   end
endmodule

`default_nettype wire
