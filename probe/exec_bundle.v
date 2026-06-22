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
    parameter SEQW   = 8,
    parameter CBITS  = 2)
   (input  wire                    clk,
    input  wire [SHARDS-1:0]       iss_valid,
    input  wire [SHARDS*SEQW-1:0]  iss_seq,
    input  wire [SHARDS*PBITS-1:0] iss_pdst,
    input  wire [SHARDS-1:0]       iss_pdst_v,
    input  wire [SHARDS*PBITS-1:0] iss_ps1,
    input  wire [SHARDS*PBITS-1:0] iss_ps2,
    input  wire [SHARDS*CBITS-1:0] iss_ckpt,
    input  wire [SHARDS*`PAYW-1:0] iss_pay,
    // LSU load writeback, muxed into the per-lane broadcast (the LSU avoids lanes
    // busy with an ALU writeback, so there is no collision — see wb_busy out)
    input  wire                    lsu_wb_v,
    input  wire [SBITS-1:0]        lsu_wb_owner,
    input  wire [PBITS-1:0]        lsu_wb_pr,
    input  wire [63:0]             lsu_wb_val,
    output wire [SHARDS-1:0]       wb_busy,        // = per-shard ALU wb valid (-> LSU)
    // writeback broadcast out (= RF-write feed, looped internally, + scheduler wake)
    output wire [SHARDS-1:0]       wb_valid,
    output wire [SHARDS*PBITS-1:0] wb_pr,
    output wire [SHARDS*64-1:0]    wb_val,
    // for the LSU / branch unit
    output wire [SHARDS*64-1:0]    agu_addr,
    output wire [SHARDS*64-1:0]    st_data,
    output wire [SHARDS-1:0]       cmp_eq, cmp_lt, cmp_ltu,
    // oldest mispredicting branch this cycle -> redirect
    output reg                     redirect,
    output reg  [63:0]             redirect_target,
    output reg  [SEQW-1:0]         redirect_seq,
    output reg  [CBITS-1:0]        redirect_ckpt);   // the branch's checkpoint

   wire [SHARDS-1:0]       wbv;          // per-shard ALU writeback valid
   wire [SHARDS*PBITS-1:0] wbp;
   wire [SHARDS*64-1:0]    wbd;
   wire [SHARDS-1:0]       brd;          // per-shard branch redirect
   wire [SHARDS*64-1:0]    brt;          // per-shard target
   wire [SHARDS*SEQW-1:0]  brs;          // per-shard branch seq

   // effective per-lane writeback = ALU result, else the LSU load (collision-free,
   // since the LSU never targets a lane with an ALU writeback this cycle).
   wire [SHARDS-1:0]       ewbv;
   wire [SHARDS*PBITS-1:0] ewbp;
   wire [SHARDS*64-1:0]    ewbd;
   assign wb_busy = wbv;

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
         .wb_valid_in(ewbv), .wb_pr_in(ewbp), .wb_val_in(ewbd),
         .wb_valid(wbv[i]), .wb_pr(wbp[i*PBITS +: PBITS]), .wb_val(wbd[i*64 +: 64]),
         .br_redirect(brd[i]), .br_target(brt[i*64 +: 64]), .br_seq(brs[i*SEQW +: SEQW]),
         .agu_addr(agu_addr[i*64 +: 64]), .st_data(st_data[i*64 +: 64]),
         .cmp_eq(cmp_eq[i]), .cmp_lt(cmp_lt[i]), .cmp_ltu(cmp_ltu[i]));
   end endgenerate

   // mux the LSU load writeback onto its owner lane (ALU takes priority; the LSU
   // only ever drives a lane with no ALU writeback, so this is contention-free).
   genvar k;
   generate for (k = 0; k < SHARDS; k = k + 1) begin : wbmux
      wire ld_here = lsu_wb_v && (lsu_wb_owner == k[SBITS-1:0]);
      assign ewbv[k]             = wbv[k] | ld_here;
      assign ewbp[k*PBITS +: PBITS] = wbv[k] ? wbp[k*PBITS +: PBITS] : lsu_wb_pr;
      assign ewbd[k*64 +: 64]    = wbv[k] ? wbd[k*64 +: 64]        : lsu_wb_val;
   end endgenerate

   assign wb_valid = ewbv;
   assign wb_pr    = ewbp;
   assign wb_val   = ewbd;

   // pick the OLDEST mispredicting branch (min seq) -> the architectural redirect
   integer j;
   always @* begin
      redirect = 1'b0; redirect_target = 64'd0; redirect_seq = {SEQW{1'b0}};
      redirect_ckpt = {CBITS{1'b0}};
      for (j = 0; j < SHARDS; j = j + 1)   // oldest mispredict wins (wrap-safe compare)
         if (brd[j] && (!redirect || $signed(brs[j*SEQW +: SEQW] - redirect_seq) < 0)) begin
            redirect        = 1'b1;
            redirect_target = brt[j*64 +: 64];
            redirect_seq    = brs[j*SEQW +: SEQW];
            redirect_ckpt   = iss_ckpt[j*CBITS +: CBITS];
         end
   end
endmodule

`default_nettype wire
