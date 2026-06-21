`default_nettype none

// Full 4-wide decode stage: IW decoder lanes (RVC expand + operand decode) plus
// the cross-slot dependency matrix. Output is the renamer's exact input contract:
// per slot, the operands with explicit valids, the {ARCH|SLOT} source redirects,
// the is_map_writer (last-writer-wins) flag, the immediate, and carry-through
// valid/seq. Slot 0 = oldest. ABITS=6 unified arch (FP 32..63 later).
module decode_stage #(parameter IW = 4, parameter SEQW = 8,
                      parameter ABITS = 6, parameter SBITS = 2)
   (input  wire [IW*32-1:0]      inst,
    input  wire [IW-1:0]         in_valid,
    input  wire [IW*SEQW-1:0]    seq_in,
    // per-slot decoded IR
    output wire [IW-1:0]         valid,
    output wire [IW*SEQW-1:0]    seq,
    output wire [IW-1:0]         is_rvc,
    output wire [IW*32-1:0]      expanded,
    output wire [IW*ABITS-1:0]   rd,
    output wire [IW-1:0]         rd_v,
    output wire [IW*ABITS-1:0]   rs1,
    output wire [IW-1:0]         rs1_v,
    output wire [IW*ABITS-1:0]   rs2,
    output wire [IW-1:0]         rs2_v,
    output wire [IW*64-1:0]      imm,
    output wire [IW-1:0]         has_imm,
    output wire [IW-1:0]         legal,
    // cross-slot resolution (feeds rename)
    output wire [IW-1:0]         s1_is_slot,
    output wire [IW*SBITS-1:0]   s1_slot,
    output wire [IW-1:0]         s2_is_slot,
    output wire [IW*SBITS-1:0]   s2_slot,
    output wire [IW-1:0]         map_writer);

   genvar g;
   generate
      for (g = 0; g < IW; g = g + 1) begin : lane
         decode_slot #(.SEQW(SEQW)) ds
           (.inst(inst[g*32 +: 32]), .in_valid(in_valid[g]),
            .seq_in(seq_in[g*SEQW +: SEQW]),
            .valid(valid[g]), .seq(seq[g*SEQW +: SEQW]), .is_rvc(is_rvc[g]),
            .expanded(expanded[g*32 +: 32]),
            .rd(rd[g*ABITS +: ABITS]), .rd_v(rd_v[g]),
            .rs1(rs1[g*ABITS +: ABITS]), .rs1_v(rs1_v[g]),
            .rs2(rs2[g*ABITS +: ABITS]), .rs2_v(rs2_v[g]),
            .imm(imm[g*64 +: 64]), .has_imm(has_imm[g]), .legal(legal[g]));
      end
   endgenerate

   decode_xslot #(.IW(IW), .ABITS(ABITS), .SBITS(SBITS)) xs
     (.rs1(rs1), .rs1_v(rs1_v), .rs2(rs2), .rs2_v(rs2_v), .rd(rd), .rd_v(rd_v),
      .s1_is_slot(s1_is_slot), .s1_slot(s1_slot),
      .s2_is_slot(s2_is_slot), .s2_slot(s2_slot), .map_writer(map_writer));
endmodule

`default_nettype wire
