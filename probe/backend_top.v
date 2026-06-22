`include "exec_pay.vh"
`default_nettype none

// Full sharded-OoO core (frontend + backend), ALU subset, with commit/CPR:
//   PC -> fetch/align -> decode -> [reg] -> rename -> dispatch
//      -> scheduler (scoreboard issue queues) -> execute (RF + ALU) -> writeback
//   writeback -> scheduler wake (+ every RF copy)   [write-before-read forwarding]
//
// Checkpoint / commit / recovery (no ROB):
//   * one checkpoint per dispatched bundle; commit_ctl tracks per-checkpoint
//     in-flight counts (incremented at dispatch, decremented at ISSUE -- issue is
//     the universal completion event, since nops/stores/branches never write back)
//     and commits the oldest in-order once it is closed and drained;
//   * each shard's bitmap freelist reclaims a committed checkpoint's dead polds in
//     one OR, and on a branch redirect rolls back the younger checkpoints' allocs;
//   * the renamer's replicated MAP restores from a per-checkpoint snapshot.
//   * back-pressure: the frontend freezes when the checkpoint ring is full, a
//     freelist is empty, or the scheduler has no room (`accept`).
//
// Branch redirect reopens the span *after* the branch's bundle (rb_idx = br_ckpt+1)
// so the branch's own bundle survives; the scheduler squashes by seqno (precise),
// the freelist/MAP roll back by checkpoint (per-bundle granularity -- a redirecting
// branch must be the youngest in its bundle, as today). LSU and CSR/M/F are future.
module backend_top
  #(parameter IW    = 4,
    parameter HW    = 8,
    parameter PCW   = 64,
    parameter SEQW  = 8,
    parameter ABITS = 6,
    parameter PBITS = 8,
    parameter LATW  = 2,
    parameter CBITS = 2,
    parameter NCHK  = 4,
    parameter DCW   = 3,         // clog2(IW+1)
    parameter CNTW  = 3,         // per-checkpoint outstanding count width
    parameter [PCW-1:0] RESET_PC = 0)
   (input  wire                    clk,
    input  wire                    reset,
    output wire [PCW-1:0]          imem_addr,
    input  wire [HW*16-1:0]        imem_data,
    input  wire [$clog2(HW+2)-1:0] imem_avail,
    // observation: per-shard writeback + the branch redirect
    output wire [IW-1:0]           wb_valid,
    output wire [IW*PBITS-1:0]     wb_pr,
    output wire [IW*64-1:0]        wb_val,
    output wire                    redirect,
    output wire [PCW-1:0]          redirect_target,
    // observation: in-order commit (one checkpoint per pulse)
    output wire                    commit,
    output wire [CBITS-1:0]        commit_idx);

   // ---- redirect (from the execute bundle's oldest mispredicting branch) ----
   wire               eb_redirect;
   wire [63:0]        eb_target;
   wire [SEQW-1:0]    eb_rseq;
   wire [CBITS-1:0]   eb_rckpt;
   assign redirect        = eb_redirect;
   assign redirect_target = eb_target;

   // ---- frontend: fetch -> decode -> rename ----
   wire [IW-1:0]      r_valid, r_rd_v, r_need1, r_need2, r_is_branch, fe_stall;
   wire [IW*SEQW-1:0] r_seq;
   wire [IW*ABITS-1:0] r_rd;
   wire [IW*PBITS-1:0] ps1, ps2, pdst;
   wire [IW*`PAYW-1:0] r_pay;
   wire [CBITS-1:0]   r_ckpt, cur;

   // ---- commit control ----
   wire               cc_commit, cc_rollback, cc_full;
   wire [CBITS-1:0]   cc_commit_idx, cc_rollback_idx;

   // ---- dispatch / back-pressure decision (on the renamed bundle) ----
   wire [IW-1:0]      disp_ready;
   wire               any_valid    = |r_valid;
   wire               can_dispatch = !cc_full && (&disp_ready) && !(|fe_stall) && !eb_redirect;
   wire               disp_fire    = any_valid && can_dispatch;
   wire               accept       = !any_valid || can_dispatch;   // else freeze frontend

   reg  [DCW-1:0]     disp_count;
   integer dc;
   always @* begin
      disp_count = {DCW{1'b0}};
      for (dc = 0; dc < IW; dc = dc + 1) disp_count = disp_count + r_valid[dc];
   end

   // a mispredicting branch reopens the span just after its own bundle's
   wire [CBITS-1:0]   rb_idx = eb_rckpt + 1'b1;

   frontend #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW), .ABITS(ABITS),
              .PBITS(PBITS), .NCHK(NCHK), .CBITS(CBITS), .RESET_PC(RESET_PC)) fe
     (.clk(clk), .reset(reset),
      .redirect(eb_redirect), .redirect_pc(eb_target),
      .redirect_seq(eb_rseq + 1'b1),          // target continues seqno after the branch
      .imem_addr(imem_addr), .imem_data(imem_data), .imem_avail(imem_avail),
      .accept(accept),
      .create(disp_fire), .commit(cc_commit), .commit_idx(cc_commit_idx),
      .rollback(cc_rollback), .rollback_idx(cc_rollback_idx),
      .r_valid(r_valid), .r_seq(r_seq), .r_rd(r_rd), .r_rd_v(r_rd_v),
      .ps1(ps1), .ps2(ps2), .pdst(pdst),
      .r_need1(r_need1), .r_need2(r_need2), .r_is_branch(r_is_branch),
      .r_pay(r_pay), .r_ckpt(r_ckpt), .cur(cur), .stall(fe_stall));

   // ---- scheduler bundle ----
   wire [IW-1:0]       iss_valid, iss_pdst_v;
   wire [IW*PBITS-1:0] iss_pdst, iss_ps1, iss_ps2;
   wire [IW*SEQW-1:0]  iss_seq;
   wire [IW*LATW-1:0]  iss_lat;
   wire [IW*CBITS-1:0] iss_ckpt;
   wire [IW*`PAYW-1:0] iss_pay;
   wire [IW-1:0]       wkv;          // writeback = wake source
   wire [IW*PBITS-1:0] wkp;

   // every ALU op is fixed latency 1 in this subset; tag every slot with the
   // bundle's checkpoint; only dispatch into the IQ when the bundle fires.
   wire [IW*LATW-1:0] disp_lat  = {IW{ {{(LATW-1){1'b0}}, 1'b1} }};
   wire [IW*CBITS-1:0] disp_ckpt = {IW{r_ckpt}};
   wire [IW-1:0]      sched_disp_valid = r_valid & {IW{disp_fire}};

   sched_bundle #(.SHARDS(IW), .PBITS(PBITS), .SEQW(SEQW), .LATW(LATW), .CBITS(CBITS)) sb
     (.clk(clk), .reset(reset),
      .disp_valid(sched_disp_valid), .disp_seq(r_seq), .disp_pdst(pdst), .disp_pdst_v(r_rd_v),
      .disp_ps1(ps1), .disp_need1(r_need1), .disp_ps2(ps2), .disp_need2(r_need2),
      .disp_lat(disp_lat), .disp_ckpt(disp_ckpt), .disp_pay(r_pay), .disp_ready(disp_ready),
      .wake_valid(wkv), .wake_pr(wkp),
      .squash(eb_redirect), .squash_seq(eb_rseq),
      .iss_valid(iss_valid), .iss_pdst(iss_pdst), .iss_pdst_v(iss_pdst_v),
      .iss_ps1(iss_ps1), .iss_ps2(iss_ps2), .iss_seq(iss_seq),
      .iss_lat(iss_lat), .iss_ckpt(iss_ckpt), .iss_pay(iss_pay));

   // ---- commit control: count by ISSUE, in-order commit, rollback on redirect ----
   commit_ctl #(.NCHK(NCHK), .CBITS(CBITS), .IW(IW), .CNTW(CNTW), .DCW(DCW)) cc
     (.clk(clk), .reset(reset), .cur(cur),
      .disp_fire(disp_fire), .disp_count(disp_count),
      .iss_valid(iss_valid), .iss_ckpt(iss_ckpt),
      .redirect(eb_redirect), .redirect_ckpt(rb_idx),
      .create(),                                  // = disp_fire (driven directly above)
      .commit(cc_commit), .commit_idx(cc_commit_idx),
      .rollback(cc_rollback), .rollback_idx(cc_rollback_idx), .full(cc_full));

   assign commit     = cc_commit;
   assign commit_idx = cc_commit_idx;

   // ---- execute bundle (RF + ALU + wb broadcast + branch resolve) ----
   exec_bundle #(.SHARDS(IW), .PBITS(PBITS), .SEQW(SEQW), .CBITS(CBITS)) eb
     (.clk(clk),
      .iss_valid(iss_valid), .iss_seq(iss_seq), .iss_pdst(iss_pdst),
      .iss_pdst_v(iss_pdst_v), .iss_ps1(iss_ps1), .iss_ps2(iss_ps2),
      .iss_ckpt(iss_ckpt), .iss_pay(iss_pay),
      .wb_valid(wkv), .wb_pr(wkp), .wb_val(wb_val),
      .agu_addr(), .cmp_eq(), .cmp_lt(), .cmp_ltu(),
      .redirect(eb_redirect), .redirect_target(eb_target),
      .redirect_seq(eb_rseq), .redirect_ckpt(eb_rckpt));

   assign wb_valid = wkv;
   assign wb_pr    = wkp;
endmodule

`default_nettype wire
