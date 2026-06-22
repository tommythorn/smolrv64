`include "exec_pay.vh"
`default_nettype none

// The execute bundle: SHARDS execute slices + the cross-shard writeback broadcast
// net. Each shard's writeback (combinational ALU result) is collected into one
// SHARDS-wide bus that (a) is registered into every shard's RF copy and (b) is the
// bundle's output -> the scheduler's `wake` and the RF-write feed are the same
// broadcast. Per-shard issue comes from the scheduler bundle; the exec payload
// (ctl+imm+pc) rides in packed (see exec_pay.vh).
module exec_bundle
  #(parameter SHARDS = 4,
    parameter SBITS  = 2,
    parameter NPHYS  = 256,
    parameter PBITS  = 8,
    parameter POOL   = 64,
    parameter IDXB   = 6,
    parameter SEQW   = 8)
   (input  wire                    clk,
    input  wire [SHARDS-1:0]       iss_valid,
    input  wire [SHARDS*SEQW-1:0]  iss_seq,
    input  wire [SHARDS*PBITS-1:0] iss_pdst,
    input  wire [SHARDS-1:0]       iss_pdst_v,
    input  wire [SHARDS*PBITS-1:0] iss_ps1,
    input  wire [SHARDS*PBITS-1:0] iss_ps2,
    input  wire [SHARDS*`PAYW-1:0] iss_pay,
    // writeback broadcast out (= RF-write feed, looped internally, + scheduler wake)
    output wire [SHARDS-1:0]       wb_valid,
    output wire [SHARDS*PBITS-1:0] wb_pr,
    output wire [SHARDS*64-1:0]    wb_val,
    // for the later LSU / branch unit
    output wire [SHARDS*64-1:0]    agu_addr,
    output wire [SHARDS-1:0]       cmp_eq, cmp_lt, cmp_ltu,
    // oldest mispredicting branch this cycle -> redirect
    output reg                     redirect,
    output reg  [63:0]             redirect_target,
    output reg  [SEQW-1:0]         redirect_seq);

   wire [SHARDS-1:0]       wbv;
   wire [SHARDS*PBITS-1:0] wbp;
   wire [SHARDS*64-1:0]    wbd;
   wire [SHARDS-1:0]       brd;          // per-shard branch redirect
   wire [SHARDS*64-1:0]    brt;          // per-shard target
   wire [SHARDS*SEQW-1:0]  brs;          // per-shard branch seq

   genvar i;
   generate for (i = 0; i < SHARDS; i = i + 1) begin : lane
      wire [`PAYW-1:0] p = iss_pay[i*`PAYW +: `PAYW];
      exec_shard #(.SHARDS(SHARDS), .SBITS(SBITS), .NPHYS(NPHYS), .PBITS(PBITS),
                   .POOL(POOL), .IDXB(IDXB), .SEQW(SEQW)) sh
        (.clk(clk),
         .iss_valid(iss_valid[i]), .iss_seq(iss_seq[i*SEQW +: SEQW]),
         .iss_pdst(iss_pdst[i*PBITS +: PBITS]), .iss_pdst_v(iss_pdst_v[i]),
         .iss_ps1(iss_ps1[i*PBITS +: PBITS]), .iss_ps2(iss_ps2[i*PBITS +: PBITS]),
         .alu_op(p[`PAY_ALUOP]), .alu_w(p[`PAY_W]), .alu_uw(p[`PAY_UW]),
         .op1_sel(p[`PAY_O1S]), .op2_imm(p[`PAY_O2I]), .res_link(p[`PAY_LINK]),
         .is_rvc(p[`PAY_RVC]), .is_mem(p[`PAY_MEM]),
         .is_branch(p[`PAY_BR]), .is_jump(p[`PAY_JMP]), .br_func(p[`PAY_BRFUNC]),
         .imm(p[`PAY_IMM]), .pc(p[`PAY_PC]),
         .wb_valid_in(wbv), .wb_pr_in(wbp), .wb_val_in(wbd),
         .wb_valid(wbv[i]), .wb_pr(wbp[i*PBITS +: PBITS]), .wb_val(wbd[i*64 +: 64]),
         .br_redirect(brd[i]), .br_target(brt[i*64 +: 64]), .br_seq(brs[i*SEQW +: SEQW]),
         .agu_addr(agu_addr[i*64 +: 64]),
         .cmp_eq(cmp_eq[i]), .cmp_lt(cmp_lt[i]), .cmp_ltu(cmp_ltu[i]));
   end endgenerate

   assign wb_valid = wbv;
   assign wb_pr    = wbp;
   assign wb_val   = wbd;

   // pick the OLDEST mispredicting branch (min seq) -> the architectural redirect
   integer j;
   always @* begin
      redirect = 1'b0; redirect_target = 64'd0; redirect_seq = {SEQW{1'b0}};
      for (j = 0; j < SHARDS; j = j + 1)
         if (brd[j] && (!redirect || brs[j*SEQW +: SEQW] < redirect_seq)) begin
            redirect        = 1'b1;
            redirect_target = brt[j*64 +: 64];
            redirect_seq    = brs[j*SEQW +: SEQW];
         end
   end
endmodule

`default_nettype wire
