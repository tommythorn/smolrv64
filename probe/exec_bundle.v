`include "exec_pay.vh"
`default_nettype none

// The execute bundle: SHARDS two-stage execute slices (RR | EX) + the cross-shard
// writeback broadcast and the forwarding network. Each shard registers its result
// (flop after the ALU); that registered broadcast is (a) written into every shard's
// RF, (b) the scheduler's completion wake, and (c) the 1-ahead forwarding source. A
// one-cycle-delayed copy is the 2-ahead forwarding source; 3+-behind dependents read
// the RF (write-before-read). The LSU load writeback is muxed onto its owner lane.
module exec_bundle
  #(parameter SHARDS = 4,
    parameter SBITS  = 2,
    parameter NPHYS  = 256,
    parameter PBITS  = 8,
    parameter POOL   = 64,
    parameter IDXB   = 6,
    parameter SEQW   = 8,
    parameter CBITS  = 2,
    parameter MIDXW  = 3)
   (input  wire                    clk,
    input  wire [SHARDS-1:0]       iss_valid,
    input  wire [SHARDS*SEQW-1:0]  iss_seq,
    input  wire [SHARDS*PBITS-1:0] iss_pdst,
    input  wire [SHARDS-1:0]       iss_pdst_v,
    input  wire [SHARDS*PBITS-1:0] iss_ps1,
    input  wire [SHARDS*PBITS-1:0] iss_ps2,
    input  wire [SHARDS*CBITS-1:0] iss_ckpt,
    input  wire [SHARDS*MIDXW-1:0] iss_mem_idx,
    input  wire [SHARDS*`PAYW-1:0] iss_pay,
    input  wire                    squash,
    input  wire [SEQW-1:0]         squash_seq,
    // per-shard M-unit status -> scheduler stall + commit_ctl completion
    output wire [SHARDS-1:0]       exec_busy,
    output wire [SHARDS-1:0]       div_done,
    output wire [SHARDS*CBITS-1:0] div_done_ckpt,
    // LSU load writeback muxed onto its owner lane
    input  wire                    lsu_wb_v,
    input  wire [SBITS-1:0]        lsu_wb_owner,
    input  wire [PBITS-1:0]        lsu_wb_pr,
    input  wire [63:0]             lsu_wb_val,
    output wire [SHARDS-1:0]       wb_busy,        // per-shard ALU wb valid (-> LSU defer)
    // registered writeback broadcast out (RF write feed + scheduler wake)
    output wire [SHARDS-1:0]       wb_valid,
    output wire [SHARDS*PBITS-1:0] wb_pr,
    output wire [SHARDS*64-1:0]    wb_val,
    // EX-stage LSU drive (aligned with agu/st_data)
    output wire [SHARDS-1:0]       ex_valid,
    output wire [SHARDS*SEQW-1:0]  ex_seq,
    output wire [SHARDS*CBITS-1:0] ex_ckpt,
    output wire [SHARDS*MIDXW-1:0] ex_mem_idx,
    output wire [SHARDS-1:0]       ex_mem,
    output wire [SHARDS-1:0]       ex_store,
    output wire [SHARDS*2-1:0]     ex_msize,
    output wire [SHARDS-1:0]       ex_msigned,
    output wire [SHARDS*64-1:0]    agu_addr,
    output wire [SHARDS*64-1:0]    st_data,
    // oldest mispredicting branch this cycle -> redirect
    output reg                     redirect,
    output reg  [63:0]             redirect_target,
    output reg  [SEQW-1:0]         redirect_seq,
    output reg  [CBITS-1:0]        redirect_ckpt);

   wire [SHARDS-1:0]       wbv;          // per-shard registered ALU/M writeback valid
   wire [SHARDS*PBITS-1:0] wbp;
   wire [SHARDS*64-1:0]    wbd;
   wire [SHARDS-1:0]       brd;
   wire [SHARDS*64-1:0]    brt;
   wire [SHARDS*SEQW-1:0]  brs;
   wire [SHARDS*CBITS-1:0] brc;          // EX-stage ckpt of each shard (for redirect)

   // effective per-lane registered writeback = ALU/M result, else the LSU load.
   wire [SHARDS-1:0]       ewbv;
   wire [SHARDS*PBITS-1:0] ewbp;
   wire [SHARDS*64-1:0]    ewbd;
   assign wb_busy = wbv;

   // 2-ahead forwarding source = the broadcast delayed one cycle.
   reg  [SHARDS-1:0]       fw2v;
   reg  [SHARDS*PBITS-1:0] fw2p;
   reg  [SHARDS*64-1:0]    fw2d;
   initial fw2v = {SHARDS{1'b0}};
   always @(posedge clk) begin fw2v <= ewbv; fw2p <= ewbp; fw2d <= ewbd; end

   genvar i;
   generate for (i = 0; i < SHARDS; i = i + 1) begin : lane
      wire [`PAYW-1:0] p = iss_pay[i*`PAYW +: `PAYW];
      exec_shard #(.SHARDS(SHARDS), .SBITS(SBITS), .NPHYS(NPHYS), .PBITS(PBITS),
                   .POOL(POOL), .IDXB(IDXB), .SEQW(SEQW), .CBITS(CBITS), .MIDXW(MIDXW)) sh
        (.clk(clk),
         .iss_valid(iss_valid[i]), .iss_seq(iss_seq[i*SEQW +: SEQW]),
         .iss_pdst(iss_pdst[i*PBITS +: PBITS]), .iss_pdst_v(iss_pdst_v[i]),
         .iss_ps1(iss_ps1[i*PBITS +: PBITS]), .iss_ps2(iss_ps2[i*PBITS +: PBITS]),
         .iss_ckpt(iss_ckpt[i*CBITS +: CBITS]), .iss_mem_idx(iss_mem_idx[i*MIDXW +: MIDXW]),
         .squash(squash), .squash_seq(squash_seq),
         .alu_op(p[`PAY_ALUOP]), .alu_w(p[`PAY_W]), .alu_uw(p[`PAY_UW]),
         .op1_sel(p[`PAY_O1S]), .op2_imm(p[`PAY_O2I]), .res_link(p[`PAY_LINK]),
         .is_rvc(p[`PAY_RVC]), .is_mem(p[`PAY_MEM]), .is_store(p[`PAY_STORE]),
         .mem_size(p[`PAY_MSIZE]), .mem_signed(p[`PAY_MSGN]),
         .is_branch(p[`PAY_BR]), .is_jump(p[`PAY_JMP]), .is_mul(p[`PAY_MUL]), .br_func(p[`PAY_BRFUNC]),
         .imm(p[`PAY_IMM]), .pc(p[`PAY_PC]),
         .wb_valid_in(ewbv), .wb_pr_in(ewbp), .wb_val_in(ewbd),
         .fw2_valid(fw2v), .fw2_pr(fw2p), .fw2_val(fw2d),
         .wb_valid(wbv[i]), .wb_pr(wbp[i*PBITS +: PBITS]), .wb_val(wbd[i*64 +: 64]),
         .br_redirect(brd[i]), .br_target(brt[i*64 +: 64]), .br_seq(brs[i*SEQW +: SEQW]),
         .ex_valid(ex_valid[i]), .ex_seq(ex_seq[i*SEQW +: SEQW]), .ex_ckpt(brc[i*CBITS +: CBITS]),
         .ex_mem_idx(ex_mem_idx[i*MIDXW +: MIDXW]), .ex_mem(ex_mem[i]), .ex_store(ex_store[i]),
         .ex_msize(ex_msize[i*2 +: 2]), .ex_msigned(ex_msigned[i]),
         .agu_addr(agu_addr[i*64 +: 64]), .st_data(st_data[i*64 +: 64]),
         .exec_busy(exec_busy[i]), .div_done(div_done[i]),
         .div_done_ckpt(div_done_ckpt[i*CBITS +: CBITS]));
   end endgenerate

   assign ex_ckpt = brc;

   genvar k;
   generate for (k = 0; k < SHARDS; k = k + 1) begin : wbmux
      wire ld_here = lsu_wb_v && (lsu_wb_owner == k[SBITS-1:0]);
      assign ewbv[k]                = wbv[k] | ld_here;
      assign ewbp[k*PBITS +: PBITS] = wbv[k] ? wbp[k*PBITS +: PBITS] : lsu_wb_pr;
      assign ewbd[k*64 +: 64]       = wbv[k] ? wbd[k*64 +: 64]       : lsu_wb_val;
   end endgenerate

   assign wb_valid = ewbv;
   assign wb_pr    = ewbp;
   assign wb_val   = ewbd;

   // oldest mispredicting branch (EX stage) -> redirect; its checkpoint = EX-stage ckpt
   integer j;
   always @* begin
      redirect = 1'b0; redirect_target = 64'd0; redirect_seq = {SEQW{1'b0}};
      redirect_ckpt = {CBITS{1'b0}};
      for (j = 0; j < SHARDS; j = j + 1)
         if (brd[j] && (!redirect || $signed(brs[j*SEQW +: SEQW] - redirect_seq) < 0)) begin
            redirect        = 1'b1;
            redirect_target = brt[j*64 +: 64];
            redirect_seq    = brs[j*SEQW +: SEQW];
            redirect_ckpt   = brc[j*CBITS +: CBITS];
         end
   end
endmodule

`default_nettype wire
