`default_nettype none

// Per-shard physical-register freelist with checkpoint reclamation (CPR), the
// bitmap design from the plan. Replaces the array/ring freelist: an array can't
// bulk-release a committed checkpoint's dead registers in one cycle; a bitmap
// does it with one OR.
//
//   free[POOL]      : 1 = this shard's owned physreg (idx i -> pr = SH+SHARDS*i)
//                     is free. idx 0..ARSH-1 are the initial arch mappings
//                     (reserved, incl x0 at shard 0); free starts as idx>=ARSH.
//   A[C][POOL]      : registers ALLOCATED in span C (this shard).
//   P[C][POOL]      : polds DISPLACED in span C that this shard OWNS (dead once C
//                     commits). Other shards route their displaced polds here via
//                     the pold broadcast.
//
//   alloc           : r = ffs(free); free[r]=0; A[cur][r]=1.
//   record pold     : owned displaced polds -> P[cur].
//   create          : open a new (younger) span: cur++, clear A/P[cur+1].
//   commit C        : free |= P[C]  (dead polds return); clear A/P[C].
//   rollback to C   : recover to BEFORE span C -> free |= union(A[k] : k in [C,cur]),
//                     clear A/P[C..cur], cur = C (C reopens; MAP restores snapshot[C]).
// Snapshots leak under reallocate-then-squash with a free-snapshot; A/P do not.
module freelist
  #(parameter SHARDS = 4,
    parameter SH     = 0,
    parameter SBITS  = 2,
    parameter PBITS  = 8,
    parameter POOL   = 64,
    parameter LPOOL  = 6,        // clog2(POOL)
    parameter NCHK   = 4,
    parameter CBITS  = 2,
    parameter ARSH   = 16)       // arch regs owned per shard (reserved head)
   (input  wire                    clk,
    input  wire                    reset,
    // allocation (this shard's instruction, into span `cur`)
    input  wire                    alloc_en,
    output wire [PBITS-1:0]        alloc_pr,
    output wire                    alloc_ok,    // a free reg was available
    output wire [LPOOL:0]          free_count,  // for back-pressure (>=2 margin)
    // displaced polds this cycle from all shards (owner records into its P[cur])
    input  wire [SHARDS-1:0]       pold_valid,
    input  wire [SHARDS*PBITS-1:0] pold_pr,
    // checkpoint control
    input  wire                    create,      // open a new younger span (cur++)
    input  wire                    commit,
    input  wire [CBITS-1:0]        commit_idx,
    input  wire                    rollback,
    input  wire [CBITS-1:0]        rollback_idx,
    output wire [CBITS-1:0]        cur);

   reg [POOL-1:0] free;
   reg [POOL-1:0] A [0:NCHK-1];
   reg [POOL-1:0] P [0:NCHK-1];
   reg [CBITS-1:0] curr;
   assign cur = curr;

   // ----- find-first-set (lowest free idx) -----
   integer i;
   reg            found;
   reg [LPOOL-1:0] ridx;
   always @* begin
      found = 1'b0; ridx = {LPOOL{1'b0}};
      for (i = POOL-1; i >= 0; i = i - 1) if (free[i]) begin found = 1'b1; ridx = i[LPOOL-1:0]; end
   end
   assign alloc_ok = found;
   assign alloc_pr = SH + (ridx << SBITS);

   integer c;
   reg [LPOOL:0] cnt;
   always @* begin cnt = 0; for (c = 0; c < POOL; c = c + 1) cnt = cnt + free[c]; end
   assign free_count = cnt;

   // ----- owned displaced polds this cycle -> a mask into P[cur] -----
   reg [POOL-1:0] pold_mask;
   integer s;
   always @* begin
      pold_mask = {POOL{1'b0}};
      for (s = 0; s < SHARDS; s = s + 1)
         if (pold_valid[s] && (pold_pr[s*PBITS +: SBITS] == SH[SBITS-1:0]))
            pold_mask[pold_pr[s*PBITS+SBITS +: LPOOL]] = 1'b1;
   end

   // ----- which spans are younger than rollback_idx (in ring order) -----
   reg [NCHK-1:0] young;
   integer kk;
   reg [CBITS:0] nyoung;
   always @* begin
      young  = {NCHK{1'b0}};
      nyoung = ((curr - rollback_idx) & (NCHK-1)) + 1'b1;  // spans [rollback_idx..cur] incl
      for (kk = 0; kk < NCHK; kk = kk + 1)
         if (kk < nyoung)
            young[(rollback_idx + kk) & (NCHK-1)] = 1'b1;
   end
   reg [POOL-1:0] roll_union;
   integer u;
   always @* begin
      roll_union = {POOL{1'b0}};
      for (u = 0; u < NCHK; u = u + 1) if (young[u]) roll_union = roll_union | A[u];
   end

   wire           do_alloc = alloc_en && found;
   wire [POOL-1:0] alloc_bit = do_alloc ? ({{(POOL-1){1'b0}},1'b1} << ridx) : {POOL{1'b0}};
   wire [CBITS-1:0] nxt = curr + 1'b1;

   integer k, j;
   initial begin
      for (j = 0; j < POOL; j = j + 1) free[j] = (j >= ARSH);
      for (k = 0; k < NCHK; k = k + 1) begin A[k] = {POOL{1'b0}}; P[k] = {POOL{1'b0}}; end
      curr = 0;
   end

   always @(posedge clk) begin
      if (reset) begin
         for (j = 0; j < POOL; j = j + 1) free[j] <= (j >= ARSH);
         for (k = 0; k < NCHK; k = k + 1) begin A[k] <= {POOL{1'b0}}; P[k] <= {POOL{1'b0}}; end
         curr <= 0;
      end else begin
         // free set: remove allocation, add committed polds / rolled-back allocs
         free <= (free & ~alloc_bit)
               | (commit   ? P[commit_idx] : {POOL{1'b0}})
               | (rollback ? roll_union    : {POOL{1'b0}});
         // per-span A / P
         for (k = 0; k < NCHK; k = k + 1) begin
            A[k] <= (rollback && young[k])        ? {POOL{1'b0}} :
                    (commit && (k==commit_idx))   ? {POOL{1'b0}} :
                    (create && (k==nxt))          ? {POOL{1'b0}} :
                    (k==curr)                     ? (A[k] | alloc_bit) : A[k];
            P[k] <= (rollback && young[k])        ? {POOL{1'b0}} :
                    (commit && (k==commit_idx))   ? {POOL{1'b0}} :
                    (create && (k==nxt))          ? {POOL{1'b0}} :
                    (k==curr)                     ? (P[k] | pold_mask) : P[k];
         end
         curr <= rollback ? rollback_idx : (create ? nxt : curr);
      end
   end
`ifdef FLDBG
   integer flc; initial flc = 0;
   reg [POOL-1:0] aun; integer fda, fdk; integer acnt, ucnt, rcnt;
   always @(posedge clk) begin
      flc <= flc + 1;
      if (SH == 0 && (flc % 100000 == 0)) $display("[FLC] c=%0d free=%0d", flc, cnt);
      if (SH == 0 && flc > `FLDBG_T0 && flc < `FLDBG_T1
          && (do_alloc || rollback || commit || create)) begin
         aun = {POOL{1'b0}};
         for (fdk = 0; fdk < NCHK; fdk = fdk + 1) aun = aun | A[fdk];
         acnt = 0; ucnt = 0; rcnt = 0;
         for (fda = 0; fda < POOL; fda = fda + 1) begin
            acnt = acnt + free[fda]; ucnt = ucnt + aun[fda]; rcnt = rcnt + roll_union[fda];
         end
         $display("[FLD] c=%0d free=%0d unionA=%0d alloc=%b(r%0d) roll=%b(idx=%0d yng=%b run=%0d) cmt=%b(idx=%0d) crt=%b curr=%0d",
                  flc, acnt, ucnt, do_alloc, ridx, rollback, rollback_idx, young, rcnt,
                  commit, commit_idx, create, curr);
      end
   end
`endif

`ifdef FL_DBLALLOC
   // Double-alloc detector. We hand out alloc_pr (idx ridx) this cycle and clear
   // free[ridx] via `& ~alloc_bit`. But a same-cycle commit-P or rollback-union
   // can re-OR that bit. If the NEXT free-set still marks ridx free, then the reg
   // we just handed to this instruction is also free -> the next alloc hands out
   // the SAME physreg = double allocation. There is no legitimate case (a reg we
   // can allocate was free, so it must not be a live pold/rolled-back alloc this
   // cycle), so any fire pinpoints the leak and names the culprit term.
   wire [POOL-1:0] nfree_dbg = (free & ~alloc_bit)
                             | (commit   ? P[commit_idx] : {POOL{1'b0}})
                             | (rollback ? roll_union    : {POOL{1'b0}});
   always @(posedge clk) if (!reset && do_alloc && nfree_dbg[ridx]) begin
      $display("[%0t] *** FL-DBLALLOC sh%0d: phys %0d (idx %0d) handed out but stays FREE | commit=%b cmt_idx=%0d P[ci][r]=%b | rollback=%b rb_idx=%0d roll_union[r]=%b | curr=%0d",
               $time, SH, alloc_pr, ridx, commit, commit_idx,
               commit ? P[commit_idx][ridx] : 1'b0,
               rollback, rollback_idx, rollback ? roll_union[ridx] : 1'b0, curr);
      $finish;
   end
`endif
endmodule

`default_nettype wire
