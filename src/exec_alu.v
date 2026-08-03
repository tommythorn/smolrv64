`default_nettype none

// Per-shard integer execute datapath: operand select -> alu.v -> result routing.
// Pure combinational; the surrounding stage registers issue inputs and the
// writeback. Reuses src/alu.v unchanged (RVA22 + Zicond), keeping its sum/eq/
// lt/ltu live for address-gen and branch resolution.
//
//   op1 = rs1 | pc | 0      (LUI uses 0, AUIPC uses pc)
//   op2 = rs2 | imm
//   result = res_link ? next_pc : alu_result
//   addr   = alu.sum  (= rs1+imm for loads/stores when op2=imm, op=ADD)
//
// next_pc = pc + (is_rvc ? 2 : 4) is the JAL/JALR link value. eq/lt/ltu are
// exported for the (later) branch-resolution unit; this datapath does not redirect.
module exec_alu
  (input  wire [5:0]  alu_op,
   input  wire        alu_w,
   input  wire        alu_uw,
   input  wire [1:0]  op1_sel,     // 0=rs1, 1=pc, 2=zero
   input  wire        op2_imm,     // 1=imm, 0=rs2
   input  wire        res_link,    // 1=result is next_pc
   input  wire        is_rvc,      // link = pc + (is_rvc?2:4)
   input  wire [63:0] rs1_val,
   input  wire [63:0] rs2_val,
   input  wire [63:0] imm,
   input  wire [63:0] pc,
   output wire [63:0] result,
   output wire [63:0] addr,        // AGU: rs1 + imm
   output wire        cmp_eq,
   output wire        cmp_lt,
   output wire        cmp_ltu);

   wire [63:0] op1 = (op1_sel == 2'd1) ? pc :
                     (op1_sel == 2'd2) ? 64'd0 : rs1_val;
   wire [63:0] op2 = op2_imm ? imm : rs2_val;

   wire [63:0] alu_r;
   alu #(.XLEN(64)) core
     (.op(alu_op), .w(alu_w), .uw(alu_uw), .op1(op1), .op2(op2),
      .result(alu_r), .sum(addr), .eq(cmp_eq), .lt(cmp_lt), .ltu(cmp_ltu));

   wire [63:0] next_pc = pc + (is_rvc ? 64'd2 : 64'd4);
   assign result = res_link ? next_pc : alu_r;
endmodule

`default_nettype wire
