`include "exec_pay.vh"
`default_nettype none

// Decode -> rename, across a registered stage boundary.
//
//   inst/in_valid/seq_in --[decode_stage (comb)]--> contract --[REG]--> renamer
//
// decode_stage produces the renamer's exact input contract (operands with
// explicit valids, {ARCH|SLOT} source redirects, last-writer-wins map_writer).
// We register that whole contract once at the decode/rename boundary, then feed
// the registered copy to renamer_bundle. The O(W^2) cross-slot matrix is thus
// computed once, in decode, and is no longer in the rename critical path -- the
// register cuts the two stages apart. The decoded valid/seq are registered too
// so they stay aligned with the renamed operands for downstream stages.
//
// Back-pressure (freeze the boundary register on stall / no checkpoint / <2 free
// regs) is deliberately NOT wired yet -- this composition establishes the clean
// datapath; the boundary advances every cycle. See the plan's back-pressure note.
module decode_rename
  #(parameter IW     = 4,    // bundle width = SHARDS
    parameter SEQW   = 8,
    parameter ABITS  = 6,
    parameter AREGS  = 64,
    parameter PBITS  = 8,
    parameter NPHYS  = 256,
    parameter POOL   = 64,
    parameter HPTR   = 6,
    parameter SBITS  = 2,
    parameter NCHK   = 4,
    parameter CBITS  = 2)
   (input  wire                 clk,
    input  wire                 reset,    // squashes the boundary (no alloc/MAP write)
    // raw aligner words
    input  wire [IW*32-1:0]     inst,
    input  wire [IW-1:0]        in_valid,
    input  wire [IW*SEQW-1:0]   seq_in,
    input  wire [IW*64-1:0]     pc_in,    // per-slot PC (for the execute payload)
    // commit/free + checkpoint control (already in the rename time domain)
    input  wire [IW*PBITS-1:0]  fr_phys,
    input  wire [IW-1:0]        fr_valid,
    input  wire                 chk_create,
    input  wire [CBITS-1:0]     chk_create_idx,
    input  wire                 chk_restore,
    input  wire [CBITS-1:0]     chk_restore_idx,
    // renamed bundle (one cycle after inst), aligned with r_valid/r_seq
    output wire [IW-1:0]        r_valid,
    output wire [IW*SEQW-1:0]   r_seq,
    output wire [IW*ABITS-1:0]  r_rd,
    output wire [IW-1:0]        r_rd_v,
    output wire [IW*PBITS-1:0]  ps1,
    output wire [IW*PBITS-1:0]  ps2,
    output wire [IW*PBITS-1:0]  pdst,
    output wire [IW-1:0]        r_need1,  // source 1 is a real dependency to wait on
    output wire [IW-1:0]        r_need2,
    output wire [IW*`PAYW-1:0]  r_pay,    // packed execute payload (ctl+imm+pc)
    output wire [IW-1:0]        stall);

   // ---------------------------------------------------------- decode (comb)
   wire [IW-1:0]        d_valid, d_rd_v, d_rs1_v, d_rs2_v;
   wire [IW*SEQW-1:0]   d_seq;
   wire [IW*ABITS-1:0]  d_rd, d_rs1, d_rs2;
   wire [IW-1:0]        d_s1_is_slot, d_s2_is_slot, d_map_writer;
   wire [IW*SBITS-1:0]  d_s1_slot, d_s2_slot;
   wire [IW-1:0]        d_is_rvc, d_alu_w, d_alu_uw, d_op2_imm, d_res_link, d_is_mem;
   wire [IW*64-1:0]     d_imm;
   wire [IW*6-1:0]      d_alu_op;
   wire [IW*2-1:0]      d_op1_sel;

   decode_stage #(.IW(IW), .SEQW(SEQW), .ABITS(ABITS), .SBITS(SBITS)) dec
     (.inst(inst), .in_valid(in_valid), .seq_in(seq_in),
      .valid(d_valid), .seq(d_seq), .is_rvc(d_is_rvc), .expanded(),
      .rd(d_rd), .rd_v(d_rd_v), .rs1(d_rs1), .rs1_v(d_rs1_v),
      .rs2(d_rs2), .rs2_v(d_rs2_v), .imm(d_imm), .has_imm(), .legal(),
      .s1_is_slot(d_s1_is_slot), .s1_slot(d_s1_slot),
      .s2_is_slot(d_s2_is_slot), .s2_slot(d_s2_slot), .map_writer(d_map_writer),
      .alu_op(d_alu_op), .alu_w(d_alu_w), .alu_uw(d_alu_uw), .op1_sel(d_op1_sel),
      .op2_imm(d_op2_imm), .res_link(d_res_link), .is_mem(d_is_mem));

   // -------------------------------------------- decode/rename boundary reg
   reg [IW-1:0]        q_valid, q_rd_v, q_s1_is_slot, q_s2_is_slot, q_map_writer;
   reg [IW*SEQW-1:0]   q_seq;
   reg [IW*ABITS-1:0]  q_rd, q_rs1, q_rs2;
   reg [IW*SBITS-1:0]  q_s1_slot, q_s2_slot;
   // payload + need flags registered alongside the rename contract
   reg [IW-1:0]        q_rs1_v, q_rs2_v, q_is_rvc, q_alu_w, q_alu_uw, q_op2_imm, q_res_link, q_is_mem;
   reg [IW*64-1:0]     q_imm, q_pc;
   reg [IW*6-1:0]      q_alu_op;
   reg [IW*2-1:0]      q_op1_sel;
   initial begin
      q_valid = 0; q_rd_v = 0; q_s1_is_slot = 0; q_s2_is_slot = 0;
      q_map_writer = 0; q_seq = 0; q_rd = 0; q_rs1 = 0; q_rs2 = 0;
      q_s1_slot = 0; q_s2_slot = 0;
   end
   // On reset, clear only the bits that cause downstream action: q_rd_v gates
   // allocation, q_map_writer gates the MAP write, q_valid gates consumers.
   // The rest may latch freely (ignored while their valids are 0).
   always @(posedge clk) begin
      q_valid      <= reset ? {IW{1'b0}} : d_valid;
      q_rd_v       <= reset ? {IW{1'b0}} : d_rd_v;
      q_map_writer <= reset ? {IW{1'b0}} : d_map_writer;
      q_seq        <= d_seq;
      q_rd         <= d_rd;
      q_rs1        <= d_rs1;      q_rs2        <= d_rs2;
      q_s1_is_slot <= d_s1_is_slot; q_s1_slot  <= d_s1_slot;
      q_s2_is_slot <= d_s2_is_slot; q_s2_slot  <= d_s2_slot;
      q_rs1_v <= d_rs1_v; q_rs2_v <= d_rs2_v;
      q_imm <= d_imm; q_pc <= pc_in;
      q_alu_op <= d_alu_op; q_alu_w <= d_alu_w; q_alu_uw <= d_alu_uw;
      q_op1_sel <= d_op1_sel; q_op2_imm <= d_op2_imm; q_res_link <= d_res_link;
      q_is_rvc <= d_is_rvc; q_is_mem <= d_is_mem;
   end

   assign r_valid = q_valid;
   assign r_seq   = q_seq;
   assign r_rd    = q_rd;
   assign r_rd_v  = q_rd_v;
   assign r_need1 = q_rs1_v;
   assign r_need2 = q_rs2_v;

   // pack the execute payload per slot (see exec_pay.vh)
   genvar p;
   generate for (p = 0; p < IW; p = p + 1) begin : pay
      assign r_pay[p*`PAYW +: `PAYW] =
        { q_pc[p*64 +: 64], q_imm[p*64 +: 64], q_is_mem[p], q_is_rvc[p],
          q_res_link[p], q_op2_imm[p], q_op1_sel[p*2 +: 2], q_alu_uw[p],
          q_alu_w[p], q_alu_op[p*6 +: 6] };
   end endgenerate

   // ---------------------------------------------------------- rename core
   renamer_bundle #(.SHARDS(IW), .ABITS(ABITS), .AREGS(AREGS), .PBITS(PBITS),
                    .NPHYS(NPHYS), .POOL(POOL), .HPTR(HPTR), .SBITS(SBITS),
                    .NCHK(NCHK), .CBITS(CBITS)) rn
     (.clk(clk),
      .rs1(q_rs1), .rs2(q_rs2), .rd(q_rd), .rd_v(q_rd_v),
      .s1_is_slot(q_s1_is_slot), .s1_slot(q_s1_slot),
      .s2_is_slot(q_s2_is_slot), .s2_slot(q_s2_slot), .map_writer(q_map_writer),
      .fr_phys(fr_phys), .fr_valid(fr_valid),
      .chk_create(chk_create), .chk_create_idx(chk_create_idx),
      .chk_restore(chk_restore), .chk_restore_idx(chk_restore_idx),
      .ps1(ps1), .ps2(ps2), .pdst(pdst), .stall(stall));
endmodule

`default_nettype wire
