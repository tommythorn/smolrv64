`default_nettype none

// RV64 C-extension expander: 16-bit compressed parcel -> 32-bit instruction,
// purely combinational. Output is 32'h00000000 for illegal/reserved encodings
// and for the [1:0]==11 quadrant (not compressed), matching the oracle
// tools/rvc.rs (RVC64_EXPANDED). Verified exhaustively by tb_rvc_expand.v.
module rvc_expand (input wire [15:0] c, output reg [31:0] insn);

   // base-ISA opcodes
   localparam [6:0] OPIMM=7'h13, OP=7'h33, OPIMM32=7'h1b, OP32=7'h3b,
                    LOAD=7'h03, STORE=7'h23, LOADFP=7'h07, STOREFP=7'h27,
                    LUI=7'h37, JAL=7'h6f, JALR=7'h67, BRANCH=7'h63, SYSTEM=7'h73;

   // 32-bit instruction assembly by format
   function [31:0] Itype(input [31:0] imm, input [4:0] rs1, input [2:0] f3,
                         input [4:0] rd, input [6:0] op);
      Itype = {imm[11:0], rs1, f3, rd, op};
   endfunction
   function [31:0] Stype(input [31:0] imm, input [4:0] rs2, input [4:0] rs1,
                         input [2:0] f3, input [6:0] op);
      Stype = {imm[11:5], rs2, rs1, f3, imm[4:0], op};
   endfunction
   function [31:0] Btype(input [31:0] imm, input [4:0] rs2, input [4:0] rs1,
                         input [2:0] f3, input [6:0] op);
      Btype = {imm[12], imm[10:5], rs2, rs1, f3, imm[4:1], imm[11], op};
   endfunction
   function [31:0] Utype(input [31:0] imm, input [4:0] rd, input [6:0] op);
      Utype = {imm[31:12], rd, op};
   endfunction
   function [31:0] Jtype(input [31:0] imm, input [4:0] rd, input [6:0] op);
      Jtype = {imm[20], imm[10:1], imm[11], imm[19:12], rd, op};
   endfunction
   function [31:0] Rtype(input [6:0] f7, input [4:0] rs2, input [4:0] rs1,
                         input [2:0] f3, input [4:0] rd, input [6:0] op);
      Rtype = {f7, rs2, rs1, f3, rd, op};
   endfunction

   // register fields
   wire [4:0] rd   = c[11:7];
   wire [4:0] rs2  = c[6:2];
   wire [4:0] rs1p = {2'b01, c[9:7]};   // x8..x15
   wire [4:0] rdsp = {2'b01, c[4:2]};   // rd'/rs2'  x8..x15

   // immediates (sign/zero-extended to 32 bits)
   wire [31:0] i_ciw = {22'b0, c[10:7], c[12:11], c[5], c[6], 2'b00};        // ADDI4SPN (zext)
   wire [31:0] i_clw = {25'b0, c[5], c[12:10], c[6], 2'b00};                 // LW/SW (zext)
   wire [31:0] i_cld = {24'b0, c[6:5], c[12:10], 3'b000};                    // LD/SD/FLD/FSD (zext)
   wire [31:0] i_ci  = {{27{c[12]}}, c[6:2]};                                // ADDI/LI/ADDIW (sext6)
   wire [31:0] i_lui = {{14{c[12]}}, c[12], c[6:2], 12'b0};                  // LUI (sext, <<12)
   wire [31:0] i_a16 = {{22{c[12]}}, c[12], c[4:3], c[5], c[2], c[6], 4'b0}; // ADDI16SP (sext10)
   wire [31:0] i_shl = {26'b0, c[12], c[6:2]};                              // slli/srli shamt (imm[11:6]=0)
   wire [31:0] i_sra = {20'b0, 6'b010000, c[12], c[6:2]};                   // srai (imm[11:6]=010000)
   wire [31:0] i_cj  = {{20{c[12]}}, c[12], c[8], c[10:9], c[6], c[7], c[2], c[11], c[5:3], 1'b0}; // J (sext12)
   wire [31:0] i_cb  = {{23{c[12]}}, c[12], c[6:5], c[2], c[11:10], c[4:3], 1'b0};                 // BEQZ/BNEZ (sext9)
   wire [31:0] i_lwsp = {24'b0, c[3:2], c[12], c[6:4], 2'b00};               // LWSP (zext)
   wire [31:0] i_ldsp = {23'b0, c[4:2], c[12], c[6:5], 3'b000};              // LDSP/FLDSP (zext)
   wire [31:0] i_swsp = {24'b0, c[8:7], c[12:9], 2'b00};                     // SWSP (zext)
   wire [31:0] i_sdsp = {23'b0, c[9:7], c[12:10], 3'b000};                   // SDSP/FSDSP (zext)

   wire [2:0] f3 = c[15:13];

   always @* begin
      insn = 32'h00000000;            // default: illegal / not-compressed
      case (c[1:0])
        2'b00: case (f3)              // ---- quadrant 0
          3'b000: if (c[12:5] != 8'b0)                       // ADDI4SPN (nzuimm!=0)
                     insn = Itype(i_ciw, 5'd2, 3'b000, rdsp, OPIMM);
          3'b001: insn = Itype(i_cld, rs1p, 3'b011, rdsp, LOADFP);   // FLD
          3'b010: insn = Itype(i_clw, rs1p, 3'b010, rdsp, LOAD);     // LW
          3'b011: insn = Itype(i_cld, rs1p, 3'b011, rdsp, LOAD);     // LD
          3'b101: insn = Stype(i_cld, rdsp, rs1p, 3'b011, STOREFP);  // FSD
          3'b110: insn = Stype(i_clw, rdsp, rs1p, 3'b010, STORE);    // SW
          3'b111: insn = Stype(i_cld, rdsp, rs1p, 3'b011, STORE);    // SD
          default: insn = 32'h0;       // 100 reserved
        endcase
        2'b01: case (f3)              // ---- quadrant 1
          3'b000: insn = Itype(i_ci, rd, 3'b000, rd, OPIMM);        // ADDI (rd=0,imm=0 -> nop)
          3'b001: if (rd != 5'b0)                                   // ADDIW (rd!=0)
                     insn = Itype(i_ci, rd, 3'b000, rd, OPIMM32);
          3'b010: insn = Itype(i_ci, 5'd0, 3'b000, rd, OPIMM);      // LI
          3'b011: if (rd == 5'd2) begin                             // ADDI16SP
                     if ({c[12], c[6:2]} != 6'b0)
                        insn = Itype(i_a16, 5'd2, 3'b000, 5'd2, OPIMM);
                  end else begin                                    // LUI
                     if ({c[12], c[6:2]} != 6'b0)
                        insn = Utype(i_lui, rd, LUI);
                  end
          3'b100: case (c[11:10])     // MISC-ALU
                    2'b00: insn = Itype(i_shl, rs1p, 3'b101, rs1p, OPIMM);   // SRLI
                    2'b01: insn = Itype(i_sra, rs1p, 3'b101, rs1p, OPIMM);   // SRAI
                    2'b10: insn = Itype(i_ci,  rs1p, 3'b111, rs1p, OPIMM);   // ANDI
                    2'b11: case ({c[12], c[6:5]})
                             3'b000: insn = Rtype(7'h20, rdsp, rs1p, 3'b000, rs1p, OP);    // SUB
                             3'b001: insn = Rtype(7'h00, rdsp, rs1p, 3'b100, rs1p, OP);    // XOR
                             3'b010: insn = Rtype(7'h00, rdsp, rs1p, 3'b110, rs1p, OP);    // OR
                             3'b011: insn = Rtype(7'h00, rdsp, rs1p, 3'b111, rs1p, OP);    // AND
                             3'b100: insn = Rtype(7'h20, rdsp, rs1p, 3'b000, rs1p, OP32);  // SUBW
                             3'b101: insn = Rtype(7'h00, rdsp, rs1p, 3'b000, rs1p, OP32);  // ADDW
                             default: insn = 32'h0;                  // 110/111 reserved
                           endcase
                  endcase
          3'b101: insn = Jtype(i_cj, 5'd0, JAL);                    // J
          3'b110: insn = Btype(i_cb, 5'd0, rs1p, 3'b000, BRANCH);   // BEQZ
          3'b111: insn = Btype(i_cb, 5'd0, rs1p, 3'b001, BRANCH);   // BNEZ
        endcase
        2'b10: case (f3)              // ---- quadrant 2
          3'b000: insn = Itype(i_shl, rd, 3'b001, rd, OPIMM);       // SLLI
          3'b001: insn = Itype(i_ldsp, 5'd2, 3'b011, rd, LOADFP);   // FLDSP
          3'b010: if (rd != 5'b0) insn = Itype(i_lwsp, 5'd2, 3'b010, rd, LOAD);  // LWSP
          3'b011: if (rd != 5'b0) insn = Itype(i_ldsp, 5'd2, 3'b011, rd, LOAD);  // LDSP
          3'b100: if (c[12] == 1'b0) begin
                     if (rs2 == 5'b0) begin
                        if (rd != 5'b0) insn = Itype(32'b0, rd, 3'b000, 5'd0, JALR);  // JR
                     end else
                        insn = Rtype(7'h00, rs2, 5'd0, 3'b000, rd, OP);              // MV
                  end else begin
                     if (rs2 == 5'b0) begin
                        if (rd == 5'b0) insn = 32'h00100073;                          // EBREAK
                        else            insn = Itype(32'b0, rd, 3'b000, 5'd1, JALR);  // JALR
                     end else
                        insn = Rtype(7'h00, rs2, rd, 3'b000, rd, OP);                 // ADD
                  end
          3'b101: insn = Stype(i_sdsp, rs2, 5'd2, 3'b011, STOREFP); // FSDSP
          3'b110: insn = Stype(i_swsp, rs2, 5'd2, 3'b010, STORE);   // SWSP
          3'b111: insn = Stype(i_sdsp, rs2, 5'd2, 3'b011, STORE);   // SDSP
        endcase
        default: insn = 32'h00000000;  // 2'b11: not a compressed parcel
      endcase
   end
endmodule

`default_nettype wire
