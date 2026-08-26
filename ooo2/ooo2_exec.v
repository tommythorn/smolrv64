`default_nettype none

// Stage X datapath: purely combinational. The surrounding stage registers, the
// register file, and the single M->X bypass level live in ooo2_core.
//
//   result = exec_alu   (ALU / LUI / AUIPC / JAL(R) link)
//   addr   = exec_alu.sum = rs1 + imm   (load/store/AMO address generation)
//   branch = branch_unit over exec_alu's eq/lt/ltu and the AGU sum (JALR target)
//
// `exec_alu` (which wraps src/alu.v) and `branch_unit` are reused unmodified. The
// branch unit only *computes* the redirect here; applying it is M's job, so a
// mispredict cannot squash an instruction that has already changed state.
//
// Multi-cycle units (mul3, divider, the FPU) are NOT here -- they live in M with
// the LSU, so X is always exactly one cycle and needs no interlock of its own.
module ooo2_exec
   (// decoded control
    input  wire [5:0]  alu_op,
    input  wire        alu_w,
    input  wire        alu_uw,
    input  wire [1:0]  op1_sel,
    input  wire        op2_imm,
    input  wire        res_link,
    input  wire        is_rvc,
    input  wire        is_branch,
    input  wire        is_jump,
    input  wire        is_jalr,
    input  wire [2:0]  br_func,
    // operands (already bypassed)
    input  wire [63:0] rs1_val,
    input  wire [63:0] rs2_val,
    input  wire [63:0] imm,
    input  wire [63:0] pc,
    // frontend's prediction for this instruction + its precomputed compares
    input  wire [63:0] pred_npc,
    input  wire        mis_taken,
    input  wire        mis_nt,
    // results
    output wire [63:0] result,
    output wire [63:0] addr,
    output wire        redirect,
    output wire [63:0] target,
    output wire        taken,
    output wire [63:0] taken_tgt);

   wire cmp_eq, cmp_lt, cmp_ltu;

   exec_alu u_alu
     (.alu_op(alu_op), .alu_w(alu_w), .alu_uw(alu_uw), .op1_sel(op1_sel),
      .op2_imm(op2_imm), .res_link(res_link), .is_rvc(is_rvc),
      .rs1_val(rs1_val), .rs2_val(rs2_val), .imm(imm), .pc(pc),
      .result(result), .addr(addr),
      .cmp_eq(cmp_eq), .cmp_lt(cmp_lt), .cmp_ltu(cmp_ltu));

   branch_unit u_br
     (.is_branch(is_branch), .is_jump(is_jump), .is_jalr(is_jalr), .is_rvc(is_rvc),
      .br_func(br_func), .cmp_eq(cmp_eq), .cmp_lt(cmp_lt), .cmp_ltu(cmp_ltu),
      .pc(pc), .imm(imm), .agu_addr(addr),
      .mis_taken(mis_taken), .mis_nt(mis_nt), .pred_npc(pred_npc),
      .redirect(redirect), .target(target), .taken_o(taken), .taken_tgt(taken_tgt));
endmodule

`default_nettype wire
