`default_nettype none

// RV64I + F/D operand decode for the sharded decoder. Input is a full 32-bit
// instruction (already RVC-expanded by rvc_expand). Produces the rename-facing
// operand contract:
//   rd/rs1/rs2/rs3 - UNIFIED 6-bit arch register ids: integer -> {1'b0,field},
//                    FP -> {1'b1,field} = arch 32..63 (one rename map / PRF for both).
//   *_v            - per-operand valid bits (validity is explicit). rd_v drops
//                    integer x0 (write discarded); FP f0 is a real register, kept.
//   rs3            - FMA third source (insn[31:27]); rs3_v only for FMADD-family.
//   imm/has_imm    - sign-extended immediate and whether op2 is the imm
//   legal          - recognized opcode
//
// The execution `ctl` blob (incl. is_fp) is decoded separately (decode_exec).
module decode_operands
   (input  wire [31:0] insn,
    output reg  [5:0]  rd,
    output reg         rd_v,
    output reg  [5:0]  rs1,
    output reg         rs1_v,
    output reg  [5:0]  rs2,
    output reg         rs2_v,
    output reg  [5:0]  rs3,
    output reg         rs3_v,
    output reg  [63:0] imm,
    output reg         has_imm,
    output reg         legal);

   localparam [6:0] LUI=7'h37, AUIPC=7'h17, JAL=7'h6f, JALR=7'h67, BRANCH=7'h63,
                    LOAD=7'h03, STORE=7'h23, OPIMM=7'h13, OP=7'h33,
                    OPIMM32=7'h1b, OP32=7'h3b, MISCMEM=7'h0f, SYSTEM=7'h73, AMO=7'h2f,
                    LOADFP=7'h07, STOREFP=7'h27, OPFP=7'h53,
                    MADD=7'h43, MSUB=7'h47, NMSUB=7'h4b, NMADD=7'h4f;

   wire [6:0] opcode = insn[6:0];
   wire [2:0] funct3 = insn[14:12];
   wire [4:0] rdf  = insn[11:7];
   wire [4:0] rs1f = insn[19:15];
   wire [4:0] rs2f = insn[24:20];
   wire [4:0] rs3f = insn[31:27];
   wire [4:0] fpop = insn[31:27];          // OP-FP category = funct7[6:2]

   // immediates (sign-extended to 64)
   wire [63:0] imm_i = {{52{insn[31]}}, insn[31:20]};
   wire [63:0] imm_s = {{52{insn[31]}}, insn[31:25], insn[11:7]};
   wire [63:0] imm_b = {{51{insn[31]}}, insn[31], insn[7], insn[30:25], insn[11:8], 1'b0};
   wire [63:0] imm_u = {{32{insn[31]}}, insn[31:12], 12'b0};
   wire [63:0] imm_j = {{43{insn[31]}}, insn[31], insn[19:12], insn[20], insn[30:21], 1'b0};

   wire csr_ro_wr = (opcode==SYSTEM) && (funct3[1:0]!=2'b00) && (funct3!=3'b100)
                  && (insn[31:30]==2'b11)
                  && ((funct3[1:0]==2'b01) || (rs1f != 5'b0));

   localparam [2:0] N=0, I=1, S=2, B=3, U=4, J=5, C=6;     // imm selectors
   reg [2:0] imm_sel;
   reg       has_rd, has_rs1, has_rs2, has_rs3;
   reg       rd_fp, rs1_fp, rs2_fp, rs3_fp;                 // operand is an FP register

   always @* begin
      has_rd=1'b0; has_rs1=1'b0; has_rs2=1'b0; has_rs3=1'b0; imm_sel=N; legal=1'b1;
      rd_fp=1'b0; rs1_fp=1'b0; rs2_fp=1'b0; rs3_fp=1'b0;
      case (opcode)
        LUI, AUIPC: begin has_rd=1'b1; imm_sel=U; end
        JAL:        begin has_rd=1'b1; imm_sel=J; end
        JALR:       begin has_rd=1'b1; has_rs1=1'b1; imm_sel=I; end
        BRANCH:     begin has_rs1=1'b1; has_rs2=1'b1; imm_sel=B; end
        LOAD:       begin has_rd=1'b1; has_rs1=1'b1; imm_sel=I; end
        STORE:      begin has_rs1=1'b1; has_rs2=1'b1; imm_sel=S; end
        OPIMM, OPIMM32: begin has_rd=1'b1; has_rs1=1'b1; imm_sel=I; end
        OP, OP32:   begin has_rd=1'b1; has_rs1=1'b1; has_rs2=1'b1; end
        AMO:        begin has_rd=1'b1; has_rs1=1'b1; has_rs2=1'b1; end
        MISCMEM:    ;
        // ---- F/D ----
        LOADFP:     begin has_rd=1'b1; rd_fp=1'b1; has_rs1=1'b1; imm_sel=I; end   // FLW/FLD: fd, rs1=int base
        STOREFP:    begin has_rs1=1'b1; has_rs2=1'b1; rs2_fp=1'b1; imm_sel=S; end // FSW/FSD: rs1=int base, fs2 data
        MADD,MSUB,NMSUB,NMADD:                                                    // FMADD family (R4)
                    begin has_rd=1'b1; rd_fp=1'b1; has_rs1=1'b1; rs1_fp=1'b1;
                          has_rs2=1'b1; rs2_fp=1'b1; has_rs3=1'b1; rs3_fp=1'b1; end
        OPFP: begin
                 has_rd=1'b1; has_rs1=1'b1;
                 case (fpop)                                  // funct7[6:2] selects the op class
                   5'b00000,5'b00001,5'b00010,5'b00011,       // FADD/FSUB/FMUL/FDIV
                   5'b00100,5'b00101:                         // FSGNJ, FMIN/FMAX
                       begin rd_fp=1'b1; rs1_fp=1'b1; has_rs2=1'b1; rs2_fp=1'b1; end
                   5'b01011,5'b01000:                         // FSQRT, FCVT.fp.fp
                       begin rd_fp=1'b1; rs1_fp=1'b1; end
                   5'b10100:                                  // FEQ/FLT/FLE -> int rd
                       begin rd_fp=1'b0; rs1_fp=1'b1; has_rs2=1'b1; rs2_fp=1'b1; end
                   5'b11000,5'b11100:                         // FCVT.int.fp, FMV.X.W/FCLASS -> int rd, fp rs1
                       begin rd_fp=1'b0; rs1_fp=1'b1; end
                   5'b11010,5'b11110:                         // FCVT.fp.int, FMV.W.X -> fp rd, int rs1
                       begin rd_fp=1'b1; rs1_fp=1'b0; end
                   default: legal=1'b0;
                 endcase
              end
        SYSTEM: case (funct3)
                  3'b000: ;
                  3'b001,3'b010,3'b011: begin has_rd=1'b1; has_rs1=1'b1; end
                  3'b101,3'b110,3'b111: begin has_rd=1'b1; imm_sel=C; end
                  default: legal=1'b0;
                endcase
        default:    legal=1'b0;
      endcase

      // operand ids: {fp_bit, field}. integer x0 dest is discarded; FP f0 is a real reg.
      rd_v  = legal && has_rd && !csr_ro_wr && (rd_fp || (rdf != 5'b0));
      rs1_v = legal && has_rs1;
      rs2_v = legal && has_rs2;
      rs3_v = legal && has_rs3;
      rd    = {rd_fp, rdf};
      rs1   = rs1_v ? {rs1_fp, rs1f} : 6'd0;       // absent source -> arch x0 (phys p0, always ready)
      rs2   = rs2_v ? {rs2_fp, rs2f} : 6'd0;
      rs3   = rs3_v ? {rs3_fp, rs3f} : 6'd0;

      has_imm = legal && (imm_sel != N);
      case (imm_sel)
        I:       imm = imm_i;
        S:       imm = imm_s;
        B:       imm = imm_b;
        U:       imm = imm_u;
        J:       imm = imm_j;
        C:       imm = {59'b0, rs1f};
        default: imm = 64'b0;
      endcase
      if (opcode == SYSTEM) imm = {47'b0, rs1f, insn[31:20]};
   end
endmodule

`default_nettype wire
