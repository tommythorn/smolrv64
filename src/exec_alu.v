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
// PIPE=1: alu's two stages; the result (and eq/lt/ltu) is the second's, addr the first's.
module exec_alu #(parameter PIPE = 0)
  (input  wire        clk,
   input  wire [5:0]  alu_op,
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
   alu #(.XLEN(64), .PIPE(PIPE)) core
     (.clk(clk), .op(alu_op), .w(alu_w), .uw(alu_uw), .op1(op1), .op2(op2),
      .result(alu_r), .sum(addr), .eq(cmp_eq), .lt_o(cmp_lt), .ltu_o(cmp_ltu));

   // the link, in the result's stage
   reg  [63:0] l_pc;  reg l_rvc, l_link;
   always @(posedge clk) begin l_pc <= pc;  l_rvc <= is_rvc;  l_link <= res_link; end
   wire [63:0] pc_r   = PIPE ? l_pc : pc;
   wire        rvc_r  = PIPE ? l_rvc : is_rvc;
   wire        link_r = PIPE ? l_link : res_link;
   wire [63:0] next_pc = pc_r + (rvc_r ? 64'd2 : 64'd4);
   assign result = link_r ? next_pc : alu_r;
endmodule

`default_nettype wire
