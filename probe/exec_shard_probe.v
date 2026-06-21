`default_nettype none

// OOC timing/area probe for one execute shard at the pipeline geometry
// (NPHYS=256, PBITS=8, POOL=64, IDXB=6): the issue -> RF read -> alu.v -> wb path
// (the cross-shard wb broadcast is a registered next-cycle update, off this path).
// IN_W=460, OUT_W=140.
module probe_dut (input wire clk, input wire [459:0] din, output wire [139:0] dout);
   wire        iss_valid  = din[0];
   wire [7:0]  iss_pdst   = din[8:1];
   wire        iss_pdst_v = din[9];
   wire [7:0]  iss_ps1    = din[17:10];
   wire [7:0]  iss_ps2    = din[25:18];
   wire [5:0]  alu_op     = din[31:26];
   wire        alu_w      = din[32];
   wire        alu_uw     = din[33];
   wire [1:0]  op1_sel    = din[35:34];
   wire        op2_imm    = din[36];
   wire        res_link   = din[37];
   wire        is_rvc     = din[38];
   wire        is_mem     = din[39];
   wire [63:0] imm        = din[103:40];
   wire [63:0] pc         = din[167:104];
   wire [3:0]   wb_valid_in = din[171:168];
   wire [31:0]  wb_pr_in    = din[203:172];
   wire [255:0] wb_val_in   = din[459:204];

   wire        wb_valid; wire [7:0] wb_pr; wire [63:0] wb_val;
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
   flopwrap #(.IN_W(460), .OUT_W(140)) u (.clk(clk), .probe_out(probe_out));
endmodule

`default_nettype wire
