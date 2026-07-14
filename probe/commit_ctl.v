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
    // issue-time in-core FP dirty (per shard, aligned with iss_valid): FSGNJ / compare /
    // FMV.x.X write FP state -> mstatus.FS=Dirty, but recorded per-checkpoint here and
    // applied only at commit (never speculatively -- a squashed FP op must not dirty FS).
    input  wire [IW-1:0]         iss_fp_dirty,
    // load completion from the LSU (the deferred decrement)
    input  wire                  ld_done,
    input  wire [CBITS-1:0]      ld_done_ckpt,
    input  wire                  ld_fp_dirty,   // the completing load is FP-dest (FLW/FLD) -> FS Dirty
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
    output wire [DCW-1:0]        commit_count,  // # instructions retiring this cycle (for minstret)
    output wire                  fp_dirty_commit, // a retiring checkpoint held an FP-state writer -> FS Dirty
    output wire                  full);         // ring full -> stall dispatch

   reg [CNTW-1:0]  count [0:NCHK-1];
   reg [DCW-1:0]   ninst [0:NCHK-1];            // bundle size of each checkpoint (for minstret)
   reg [NCHK-1:0]  fp_pend;                      // per-ckpt: a retiring FP-state writer -> mstatus.FS Dirty
   reg [CBITS-1:0] committed;
   integer i, s;

   initial begin
      for (i = 0; i < NCHK; i = i + 1) begin count[i] = 0; ninst[i] = 0; end
      committed = 0; fp_pend = 0;
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
   // a committed checkpoint retires its whole (un-squashed) bundle -> its dispatched
   // instruction count, captured at create. Drives minstret in csr_file.
   assign commit_count = commit ? ninst[committed] : {DCW{1'b0}};

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

   // ---- per-checkpoint pending FS-dirty (commit-gated mstatus.FS=Dirty) ----
   // FS must go Dirty only when an FP-state-writing op RETIRES, never speculatively: a
   // squashed FP op leaking Dirty diverges from the in-order model and is a speculative-
   // visible-state leak. Set a pending bit per checkpoint from the SAME completion events
   // that decrement its count -- issue-time in-core FP (compare/FSGNJ/FMV.x.X), CVFPU arith
   // (fp_done, every one dirties), FP loads (ld_fp_dirty) -- WIPE it on rollback (a squashed
   // checkpoint never dirties) and APPLY at commit. Each set-cycle == that source's decrement
   // cycle, so the bit is registered a full cycle before its checkpoint can commit (commit
   // needs count==0) -> the commit read below is a plain registered lookup, no bypass.
   reg [NCHK-1:0] fp_set;
   always @* begin
      fp_set = 0;
      for (s = 0; s < IW; s = s + 1) begin
         if (iss_fp_dirty[s]) fp_set[iss_ckpt[s*CBITS +: CBITS]]     = 1'b1;
         if (fp_done[s])      fp_set[fp_done_ckpt[s*CBITS +: CBITS]] = 1'b1;
      end
      if (ld_fp_dirty) fp_set[ld_done_ckpt] = 1'b1;
   end
   assign fp_dirty_commit = commit && fp_pend[committed];

   always @(posedge clk) begin
      if (reset) begin
         for (i = 0; i < NCHK; i = i + 1) begin count[i] <= 0; fp_pend[i] <= 1'b0; end
         committed <= 0;
      end else begin
         for (i = 0; i < NCHK; i = i + 1) begin
            count[i] <= (redirect && young[i]) ? {CNTW{1'b0}}
                      : count[i] + ((disp_fire && (cur == i[CBITS-1:0])) ? disp_count : {DCW{1'b0}})
                                 - dec[i];
            // pending FS-dirty: wipe on rollback (squashed -> never dirties) or at commit
            // (applied to mstatus this cycle; clears the slot for reuse), else accumulate.
            fp_pend[i] <= (redirect && young[i])                 ? 1'b0
                        : (commit && (committed == i[CBITS-1:0])) ? 1'b0
                        : fp_pend[i] | fp_set[i];
         end
         if (commit) committed <= committed + 1'b1;
         if (disp_fire) ninst[cur] <= disp_count;   // remember the bundle size for minstret
      end
`ifdef CCDBG
      if (disp_fire) $display("[CC] t=%0t DISP ck=%0d +%0d", $time, cur, disp_count);
      for (i = 0; i < NCHK; i = i + 1)
         if (dec[i] != 0) $display("[CC] t=%0t DEC  ck=%0d -%0d (ld=%b st=%b div=%b fp=%b iss=%b%b%b%b)",
                                   $time, i, dec[i], ld_done, st_done, |div_done, |fp_done,
                                   iss_valid[0], iss_valid[1], iss_valid[2], iss_valid[3]);
      if (redirect) $display("[CC] t=%0t ROLL young=%b (idx=%0d cur=%0d)", $time, young, redirect_ckpt, cur);
      if (commit)   $display("[CC] t=%0t COMMIT ck=%0d", $time, committed);
`endif
   end
endmodule

`default_nettype wire
