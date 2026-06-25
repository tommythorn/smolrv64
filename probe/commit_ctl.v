`default_nettype none

// Global checkpoint / commit control for CPR. One checkpoint per dispatched
// bundle (the simple periodic scheme that gives commit points even in
// straight-line code). Tracks, per checkpoint, the count of dispatched-but-not-
// yet-issued instructions; the oldest checkpoint commits (in order) once its
// count hits 0 and it is closed (a newer checkpoint exists). "Issued" is the
// completion event: once issued and non-speculative (oldest), an instruction's
// displaced pold is dead, so committing frees freelist.P[C].
//
// `cur` (the open checkpoint) is owned by the freelist and read here for tagging
// / indexing. This module drives create / commit / rollback to the freelist and
// raises `full` (ring full) for back-pressure. On a branch redirect it rolls back
// to the branch's checkpoint and clears the squashed checkpoints' counts.
module commit_ctl
  #(parameter NCHK  = 4,
    parameter CBITS = 2,
    parameter IW    = 4,
    parameter CNTW  = 3,         // per-ckpt outstanding count (<= IW per bundle)
    parameter DCW   = 3)         // clog2(IW+1)
   (input  wire                  clk,
    input  wire                  reset,
    input  wire [CBITS-1:0]      cur,          // freelist's current open checkpoint
    // dispatch: a bundle (tagged `cur`) with disp_count valid instructions
    input  wire                  disp_fire,
    input  wire [DCW-1:0]        disp_count,
    // issue completions (per shard) + their checkpoint. ALU/branch/store complete
    // at issue; LOADS do not (they complete at the LSU) -> excluded here via
    // iss_is_load and counted by the ld_done port instead.
    input  wire [IW-1:0]         iss_valid,
    input  wire [IW-1:0]         iss_is_load,
    input  wire [IW-1:0]         iss_is_div,    // divides also defer (iterative) -> count at div_done
    input  wire [IW-1:0]         iss_is_fp,     // FPU-arith defers (CVFPU) -> count at fp_done
    input  wire [IW*CBITS-1:0]   iss_ckpt,
    // load completion from the LSU (the deferred decrement)
    input  wire                  ld_done,
    input  wire [CBITS-1:0]      ld_done_ckpt,
    // store completion from the LSU (deferred decrement; only under Sv39, where stores are
    // excluded from the issue-time count and complete after their translation check)
    input  wire                  st_done,
    input  wire [CBITS-1:0]      st_done_ckpt,
    // per-shard iterative-divide completion (deferred decrement)
    input  wire [IW-1:0]         div_done,
    input  wire [IW*CBITS-1:0]   div_done_ckpt,
    // per-shard FPU-arith completion (deferred decrement)
    input  wire [IW-1:0]         fp_done,
    input  wire [IW*CBITS-1:0]   fp_done_ckpt,
    // branch redirect -> rollback to the branch's checkpoint
    input  wire                  redirect,
    input  wire [CBITS-1:0]      redirect_ckpt,
    // to freelist + frontend
    output wire                  create,        // open a new checkpoint (advance cur)
    output wire                  commit,
    output wire [CBITS-1:0]      commit_idx,
    output wire                  rollback,
    output wire [CBITS-1:0]      rollback_idx,
    output wire [CBITS-1:0]      committed_idx, // oldest live checkpoint (for serialize gating)
    output wire                  empty,         // no instructions in flight (-> precise fetch trap)
    output wire                  full);         // ring full -> stall dispatch

   reg [CNTW-1:0]  count [0:NCHK-1];
   reg [CBITS-1:0] committed;
   integer i, s;

   initial begin
      for (i = 0; i < NCHK; i = i + 1) count[i] = 0;
      committed = 0;
   end

   // live checkpoints = committed..cur (inclusive); full when all NCHK are live
   wire [CBITS-1:0] live_m1 = (cur - committed) & (NCHK-1);
   assign full     = (live_m1 == (NCHK-1));
   assign create   = disp_fire;                 // frontend gates disp_fire by !full
   assign rollback = redirect;
   assign rollback_idx = redirect_ckpt;
   // commit the oldest once it is closed (newer ckpt exists) and drained
   assign commit     = !redirect && (committed != cur) && (count[committed] == 0);
   assign commit_idx = committed;
   assign committed_idx = committed;
   // empty = no live closed checkpoints AND the open one holds no outstanding ops.
   assign empty = (committed == cur) && (count[committed] == {CNTW{1'b0}});

   // squashed checkpoints on rollback: [redirect_ckpt .. cur] inclusive
   reg [NCHK-1:0]  young;
   reg [CBITS:0]   nyoung;
   integer kk;
   always @* begin
      young  = 0;
      nyoung = ((cur - redirect_ckpt) & (NCHK-1)) + 1'b1;
      for (kk = 0; kk < NCHK; kk = kk + 1)
         if (kk < nyoung) young[(redirect_ckpt + kk) & (NCHK-1)] = 1'b1;
   end

   // per-checkpoint issue decrements
   reg [DCW-1:0] dec [0:NCHK-1];
   always @* begin
      for (i = 0; i < NCHK; i = i + 1) dec[i] = 0;
      for (s = 0; s < IW; s = s + 1)
         if (iss_valid[s] && !iss_is_load[s] && !iss_is_div[s] && !iss_is_fp[s])
            dec[iss_ckpt[s*CBITS +: CBITS]] = dec[iss_ckpt[s*CBITS +: CBITS]] + 1'b1;
      if (ld_done) dec[ld_done_ckpt] = dec[ld_done_ckpt] + 1'b1;   // load completes at LSU
      if (st_done) dec[st_done_ckpt] = dec[st_done_ckpt] + 1'b1;   // store completes at LSU (Sv39)
      for (s = 0; s < IW; s = s + 1) begin                         // divide / FP complete deferred
         if (div_done[s])
            dec[div_done_ckpt[s*CBITS +: CBITS]] = dec[div_done_ckpt[s*CBITS +: CBITS]] + 1'b1;
         if (fp_done[s])
            dec[fp_done_ckpt[s*CBITS +: CBITS]] = dec[fp_done_ckpt[s*CBITS +: CBITS]] + 1'b1;
      end
   end

   always @(posedge clk) begin
      if (reset) begin
         for (i = 0; i < NCHK; i = i + 1) count[i] <= 0;
         committed <= 0;
      end else begin
         for (i = 0; i < NCHK; i = i + 1)
            count[i] <= (redirect && young[i]) ? {CNTW{1'b0}}
                      : count[i] + ((disp_fire && (cur == i[CBITS-1:0])) ? disp_count : {DCW{1'b0}})
                                 - dec[i];
         if (commit) committed <= committed + 1'b1;
      end
   end
endmodule

`default_nettype wire
