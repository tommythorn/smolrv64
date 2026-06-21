`default_nettype none

// OOC timing probe for one execute shard: the issue -> RF read -> alu.v -> wb
// path (the backend's per-shard critical path; the cross-shard wb broadcast is a
// registered next-cycle update, not on this path). IN_W=453, OUT_W=139.
module probe_dut (input wire clk, input wire [452:0] din, output wire [138:0] dout);

   wire        iss_valid  = din[0];
   wire [6:0]  iss_pdst   = din[7:1];
   wire        iss_pdst_v = din[8];
   wire [6:0]  iss_ps1    = din[15:9];
   wire [6:0]  iss_ps2    = din[22:16];
   wire [5:0]  alu_op     = din[28:23];
   wire        alu_w      = din[29];
   wire        alu_uw     = din[30];
   wire [1:0]  op1_sel    = din[32:31];
   wire        op2_imm    = din[33];
   wire        res_link   = din[34];
   wire        is_rvc     = din[35];
   wire        is_mem     = din[36];
   wire [63:0] imm        = din[100:37];
   wire [63:0] pc         = din[164:101];
   wire [3:0]  wb_valid_in = din[168:165];
   wire [27:0] wb_pr_in    = din[196:169];
   wire [255:0] wb_val_in  = din[452:197];

   wire        wb_valid; wire [6:0] wb_pr; wire [63:0] wb_val;
   wire [63:0] agu_addr; wire cmp_eq, cmp_lt, cmp_ltu;

   exec_shard dut
     (.clk(clk), .iss_valid(iss_valid), .iss_pdst(iss_pdst), .iss_pdst_v(iss_pdst_v),
      .iss_ps1(iss_ps1), .iss_ps2(iss_ps2),
      .alu_op(alu_op), .alu_w(alu_w), .alu_uw(alu_uw), .op1_sel(op1_sel),
      .op2_imm(op2_imm), .res_link(res_link), .is_rvc(is_rvc), .is_mem(is_mem),
      .imm(imm), .pc(pc),
      .wb_valid_in(wb_valid_in), .wb_pr_in(wb_pr_in), .wb_val_in(wb_val_in),
      .wb_valid(wb_valid), .wb_pr(wb_pr), .wb_val(wb_val),
      .agu_addr(agu_addr), .cmp_eq(cmp_eq), .cmp_lt(cmp_lt), .cmp_ltu(cmp_ltu));

   assign dout = {cmp_ltu, cmp_lt, cmp_eq, agu_addr, wb_val, wb_pr, wb_valid};
endmodule

module exec_shard_probe (input wire clk, output wire probe_out);
   flopwrap #(.IN_W(453), .OUT_W(139)) u (.clk(clk), .probe_out(probe_out));
endmodule

`default_nettype wire
