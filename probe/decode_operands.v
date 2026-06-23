`default_nettype none

// RV64I operand decode for the sharded decoder. Input is a full 32-bit
// instruction (already RVC-expanded by rvc_expand). Produces the rename-facing
// operand contract:
//   rd/rs1/rs2  - unified 6-bit arch register ids (integer -> {1'b0,field};
//                 FP would be {1'b1,field} = 32..63, not yet implemented)
//   *_v         - per-operand valid bits (the x0-magic-zero is gone: validity is
//                 explicit). rd_v also drops integer x0 (write discarded).
//   imm/has_imm - sign-extended immediate and whether the ALU's 2nd operand is it
//   legal       - recognized opcode
//
// NOT handled yet (RV64I scope): F/D (FP regs, OP-FP, FMADD…), SFENCE.VMA operand
// reads, AMO. The execution `ctl` blob is decoded separately (opaque to rename).
module decode_operands
   (input  wire [31:0] insn,
    output reg  [5:0]  rd,
    output reg         rd_v,
    output reg  [5:0]  rs1,
    output reg         rs1_v,
    output reg  [5:0]  rs2,
    output reg         rs2_v,
    output reg  [63:0] imm,
    output reg         has_imm,
    output reg         legal);

   localparam [6:0] LUI=7'h37, AUIPC=7'h17, JAL=7'h6f, JALR=7'h67, BRANCH=7'h63,
                    LOAD=7'h03, STORE=7'h23, OPIMM=7'h13, OP=7'h33,
                    OPIMM32=7'h1b, OP32=7'h3b, MISCMEM=7'h0f, SYSTEM=7'h73;

   wire [6:0] opcode = insn[6:0];
   wire [2:0] funct3 = insn[14:12];
   wire [4:0] rdf  = insn[11:7];
   wire [4:0] rs1f = insn[19:15];
   wire [4:0] rs2f = insn[24:20];

   // immediates (sign-extended to 64)
   wire [63:0] imm_i = {{52{insn[31]}}, insn[31:20]};
   wire [63:0] imm_s = {{52{insn[31]}}, insn[31:25], insn[11:7]};
   wire [63:0] imm_b = {{51{insn[31]}}, insn[31], insn[7], insn[30:25], insn[11:8], 1'b0};
   wire [63:0] imm_u = {{32{insn[31]}}, insn[31:12], 12'b0};
   wire [63:0] imm_j = {{43{insn[31]}}, insn[31], insn[19:12], insn[20], insn[30:21], 1'b0};
   wire [63:0] imm_csri = {59'b0, rs1f};                   // zimm (CSR immediate)

   localparam [2:0] N=0, I=1, S=2, B=3, U=4, J=5, C=6;     // imm selectors
   reg [2:0] imm_sel;
   reg       has_rd, has_rs1, has_rs2;

   always @* begin
      has_rd = 1'b0; has_rs1 = 1'b0; has_rs2 = 1'b0; imm_sel = N; legal = 1'b1;
      case (opcode)
        LUI, AUIPC: begin has_rd = 1'b1; imm_sel = U; end
        JAL:        begin has_rd = 1'b1; imm_sel = J; end
        JALR:       begin has_rd = 1'b1; has_rs1 = 1'b1; imm_sel = I; end
        BRANCH:     begin has_rs1 = 1'b1; has_rs2 = 1'b1; imm_sel = B; end
        LOAD:       begin has_rd = 1'b1; has_rs1 = 1'b1; imm_sel = I; end
        STORE:      begin has_rs1 = 1'b1; has_rs2 = 1'b1; imm_sel = S; end
        OPIMM, OPIMM32: begin has_rd = 1'b1; has_rs1 = 1'b1; imm_sel = I; end
        OP, OP32:   begin has_rd = 1'b1; has_rs1 = 1'b1; has_rs2 = 1'b1; end
        MISCMEM:    ;                                       // FENCE/FENCE.I: no GPR deps
        SYSTEM: case (funct3)
                  3'b000: ;                                // ECALL/EBREAK/xRET/WFI
                  3'b001, 3'b010, 3'b011:                  // CSRRW/S/C
                          begin has_rd = 1'b1; has_rs1 = 1'b1; end
                  3'b101, 3'b110, 3'b111:                  // CSRRWI/SI/CI
                          begin has_rd = 1'b1; imm_sel = C; end
                  default: legal = 1'b0;                   // funct3==100
                endcase
        default:    legal = 1'b0;
      endcase

      // operand fields (integer -> top arch bit 0). x0 dest writes are discarded.
      rd_v  = legal && has_rd && (rdf != 5'b0);
      rs1_v = legal && has_rs1;
      rs2_v = legal && has_rs2;
      // An absent source reads as arch x0 (-> phys p0): the constant-zero register is
      // always ready and never written, so it drops out of the scheduler's readiness
      // test with no per-operand "need" bit. (A present x0 source already does this.)
      rd    = {1'b0, rdf};
      rs1   = rs1_v ? {1'b0, rs1f} : 6'd0;
      rs2   = rs2_v ? {1'b0, rs2f} : 6'd0;

      has_imm = legal && (imm_sel != N);
      case (imm_sel)
        I:       imm = imm_i;
        S:       imm = imm_s;
        B:       imm = imm_b;
        U:       imm = imm_u;
        J:       imm = imm_j;
        C:       imm = imm_csri;
        default: imm = 64'b0;
      endcase
   end
endmodule

`default_nettype wire
