`default_nettype none

// OOC probe of exec_alu ALONE (no RF read), to decompose the exec_shard path.
// IN_W=269, OUT_W=131.
module probe_dut (input wire clk, input wire [268:0] din, output wire [130:0] dout);
   wire [5:0]  alu_op   = din[5:0];
   wire        alu_w    = din[6];
   wire        alu_uw   = din[7];
   wire [1:0]  op1_sel  = din[9:8];
   wire        op2_imm  = din[10];
   wire        res_link = din[11];
   wire        is_rvc   = din[12];
   wire [63:0] rs1_val  = din[76:13];
   wire [63:0] rs2_val  = din[140:77];
   wire [63:0] imm      = din[204:141];
   wire [63:0] pc       = din[268:205];

   wire [63:0] result, addr; wire cmp_eq, cmp_lt, cmp_ltu;
   exec_alu dut
     (.alu_op(alu_op), .alu_w(alu_w), .alu_uw(alu_uw), .op1_sel(op1_sel),
      .op2_imm(op2_imm), .res_link(res_link), .is_rvc(is_rvc),
      .rs1_val(rs1_val), .rs2_val(rs2_val), .imm(imm), .pc(pc),
      .result(result), .addr(addr), .cmp_eq(cmp_eq), .cmp_lt(cmp_lt), .cmp_ltu(cmp_ltu));

   assign dout = {cmp_ltu, cmp_lt, cmp_eq, addr, result};
endmodule

module exec_alu_probe (input wire clk, output wire probe_out);
   flopwrap #(.IN_W(269), .OUT_W(131)) u (.clk(clk), .probe_out(probe_out));
endmodule

`default_nettype wire
