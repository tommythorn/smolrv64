`default_nettype none

// One decoder lane: raw aligner word -> decoded per-slot IR (no cross-slot yet).
// The aligner delivers a 32-bit word with the top 16 bits ignored for compressed
// instructions (inst[1:0]!=11). We RVC-expand in place, then operand-decode the
// resulting 32-bit form. Per-operand valids are gated by in_valid so an invalid
// slot contributes no dependencies.
module decode_slot #(parameter SEQW = 8)
   (input  wire [31:0]      inst,
    input  wire             in_valid,
    input  wire [SEQW-1:0]  seq_in,
    output wire             valid,
    output wire [SEQW-1:0]  seq,
    output wire             is_rvc,
    output wire [31:0]      expanded,
    output wire [5:0]       rd,
    output wire             rd_v,
    output wire [5:0]       rs1,
    output wire             rs1_v,
    output wire [5:0]       rs2,
    output wire             rs2_v,
    output wire [63:0]      imm,
    output wire             has_imm,
    output wire             legal);

   wire        is_c = (inst[1:0] != 2'b11);
   wire [31:0] exp_rvc;
   rvc_expand u_rvc (.c(inst[15:0]), .insn(exp_rvc));
   wire [31:0] full = is_c ? exp_rvc : inst;

   wire [5:0]  d_rd, d_rs1, d_rs2;
   wire        d_rdv, d_rs1v, d_rs2v, d_himm, d_legal;
   wire [63:0] d_imm;
   decode_operands u_op (.insn(full), .rd(d_rd), .rd_v(d_rdv), .rs1(d_rs1),
      .rs1_v(d_rs1v), .rs2(d_rs2), .rs2_v(d_rs2v), .imm(d_imm),
      .has_imm(d_himm), .legal(d_legal));

   assign valid    = in_valid;
   assign seq      = seq_in;
   assign is_rvc   = is_c;
   assign expanded = full;
   assign rd       = d_rd;
   assign rd_v     = in_valid & d_rdv;
   assign rs1      = d_rs1;
   assign rs1_v    = in_valid & d_rs1v;
   assign rs2      = d_rs2;
   assign rs2_v    = in_valid & d_rs2v;
   assign imm      = d_imm;
   assign has_imm  = in_valid & d_himm;
   assign legal    = in_valid & d_legal;
endmodule

`default_nettype wire
