`include "exec_pay.vh"
`default_nettype none

// One shard of the sharded scheduler: a classic CAM reservation station.
//
// **Stopgap design** (the scheduler is the single most timing-critical structure and
// gets a full rethink later). N=2 entries, 3 source operands each (rs1/rs2/rs3, the
// 3rd for future FMA). Chosen to take the 256-deep `ready[]` lookup OFF the issue
// critical path: each entry carries its source tags + *registered* per-source ready
// bits, woken by a CAM compare of those tags against the result/wake broadcast. So
// issue eligibility is just `r1 & r2 & r3` (an AND of flops) + a 2-way oldest select
// -- no array index, no serial min-scan. (The old scoreboard indexed ready[ps] with
// a 258:1 mux per source on the issue path; that was the bottleneck.)
//
// A small per-phys `ready` table still exists, but is read only at DISPATCH (to seed
// a new entry's ready bits) -- off the issue path. Same-cycle dispatch-vs-wake is
// handled by OR-ing the wake CAM match into the seed.
//
// Wakeup source is unchanged (the result broadcast): a 1-cycle ALU producer drives
// it combinationally at issue, so a dependent wakes at T and issues T+1 (back-to-back);
// multi-cycle units (iterative divide, etc.) broadcast at completion (deferred).
module sched_shard
  #(parameter SHARDS = 4,
    parameter SH     = 0,
    parameter NPHYS  = 256,
    parameter PBITS  = 8,
    parameter N      = 2,        // reservation-station entries (sweep for timing)
    parameter WAKEN  = 4,        // wake compare ports (select-time + completion-time)
    parameter SEQW   = 8,
    parameter CBITS  = 2,
    parameter MIDXW  = 3,
    parameter PAYW   = `PAYW)
   (input  wire                    clk,
    input  wire                    reset,
    // dispatch: this shard's renamed instruction
    input  wire                    disp_valid,
    input  wire [SEQW-1:0]         disp_seq,
    input  wire [PBITS-1:0]        disp_pdst,
    input  wire                    disp_pdst_v,
    input  wire [PBITS-1:0]        disp_ps1,
    input  wire [PBITS-1:0]        disp_ps2,
    input  wire [PBITS-1:0]        disp_ps3,      // 3rd operand (FMA); tied to p0 until FP
    input  wire                    disp_rdy1,     // shared scoreboard read for each source
    input  wire                    disp_rdy2,     // (ready[disp_psX], pre-edge) -- from
    input  wire                    disp_rdy3,     // sched_bundle's single ready[] table
    input  wire [CBITS-1:0]        disp_ckpt,
    input  wire [MIDXW-1:0]        disp_mem_idx,
    input  wire [PAYW-1:0]         disp_pay,
    output wire                    disp_ready,    // a slot is (or becomes) free this cycle
    // cross-shard scoreboard broadcasts (self included)
    input  wire [SHARDS-1:0]       clr_valid,
    input  wire [SHARDS*PBITS-1:0] clr_pr,
    input  wire [WAKEN-1:0]        wake_valid,    // select-time (latency-1) + completion-time
    input  wire [WAKEN*PBITS-1:0]  wake_pr,
    // branch misprediction squash: drop entries younger than the branch
    input  wire                    squash,
    input  wire [SEQW-1:0]         squash_seq,
    // structural stall: this shard's iterative divider is busy -> issue nothing
    input  wire                    exec_busy,
    // oldest live checkpoint: a serializing op (CSR/system/fence) issues only when
    // its checkpoint is the oldest, so it executes non-speculatively (nothing older
    // can squash it) and its CSR/trap side-effect + redirect are precise.
    input  wire [CBITS-1:0]        committed,
    // this shard's issue this cycle (bundle feeds it back as wake_*[SH])
    output wire                    iss_valid,
    output wire [SEQW-1:0]         iss_seq,
    output wire [PBITS-1:0]        iss_pdst,
    output wire                    iss_pdst_v,
    output wire [PBITS-1:0]        iss_ps1,
    output wire [PBITS-1:0]        iss_ps2,
    output wire [PBITS-1:0]        iss_ps3,
    output wire [CBITS-1:0]        iss_ckpt,
    output wire [MIDXW-1:0]        iss_mem_idx,
    output wire [PAYW-1:0]         iss_pay,
    // ---- wedge debug: a live entry that is NOT eligible (operands never became ready, or a
    //      serializing entry whose checkpoint never became the committed one). An op stuck here
    //      never issues, so it never decrements its checkpoint's count -> commit wedge. ----
    output wire                    dbg_any_v,
    output wire                    dbg_stuck);

   localparam NW = (N <= 2) ? 1 : $clog2(N);   // entry-index width (tracks N)

   // -------------------------------------------------------------- state
   // The per-phys "value-present" scoreboard is SHARED across shards (it is bit-identical
   // in every shard: same broadcasts, no per-shard write) -- it lives in sched_bundle and
   // feeds each source's pre-read ready bit in via disp_rdy{1,2,3}. So this shard holds no
   // 256-entry table; it only seeds new entries and CAM-wakes resident ones.
   reg              v   [0:N-1];
   reg [SEQW-1:0]   sq  [0:N-1];
   reg [PBITS-1:0]  pd  [0:N-1];
   reg              pdv [0:N-1];
   reg [PBITS-1:0]  s1  [0:N-1], s2 [0:N-1], s3 [0:N-1];
   reg              r1  [0:N-1], r2 [0:N-1], r3 [0:N-1];   // per-source ready (CAM-woken)
   reg [CBITS-1:0]  ck  [0:N-1];
   reg [MIDXW-1:0]  mi  [0:N-1];
   reg [PAYW-1:0]   py  [0:N-1];

   integer i, k;
   initial for (i = 0; i < N; i = i + 1) v[i] = 1'b0;

   // ---------------------------------------- CAM wake match (tag vs result broadcast)
   function match;
      input [PBITS-1:0] tag;
      integer s;
      begin
         match = 1'b0;
         for (s = 0; s < WAKEN; s = s + 1)
            if (wake_valid[s] && (wake_pr[s*PBITS +: PBITS] == tag)) match = 1'b1;
      end
   endfunction

   // same-cycle clr: a producer being *allocated* this cycle (in any shard) marks its
   // dest not-ready. The seed below reads the ready table combinationally (pre-edge),
   // so it must mask a stale "ready" for a tag being cleared this very cycle -- this is
   // the common intra-bundle dependency (producer + consumer dispatch together).
   function clr_hit;
      input [PBITS-1:0] tag;
      integer s;
      begin
         clr_hit = 1'b0;
         if (tag != {PBITS{1'b0}})                 // p0 is constant-ready, never reallocated
            for (s = 0; s < SHARDS; s = s + 1)
               if (clr_valid[s] && (clr_pr[s*PBITS +: PBITS] == tag)) clr_hit = 1'b1;
      end
   endfunction

   // ------------------------------------------------ eligibility + oldest-of-N select
   // The oldest pick is a fairness heuristic, not a correctness rule (any eligible entry
   // may issue), so the age compare is COARSENED: drop the low AGELSB bits of the seqno.
   // This keeps "really old wins" (no starvation) while shrinking each compare in this
   // serial scan from a full-seqno subtract to (SEQW-AGELSB) bits -- the select chain is
   // what bounds N. The stored/issued seq stays full width (commit/squash need it).
   localparam AGELSB = 4;
   localparam AGEW   = SEQW - AGELSB;
   reg [N-1:0]    elig;
   reg            found;
   reg [NW-1:0]   sel;
   reg [AGEW-1:0] best;
   integer e;
   always @* begin
      found = 1'b0; sel = {NW{1'b0}}; best = {AGEW{1'b0}};
      for (e = 0; e < N; e = e + 1) begin
         // all sources ready (flops, no mux); a serializing entry must also be oldest
         elig[e] = v[e] & r1[e] & r2[e] & r3[e]
                 & (~py[e][`PAY_SER] | (ck[e] == committed));
         if (elig[e] && (!found || $signed(sq[e][SEQW-1:AGELSB] - best) < 0)) begin
            found = 1'b1; sel = e[NW-1:0]; best = sq[e][SEQW-1:AGELSB];
         end
      end
   end

   // free slot: any invalid entry, else the one issuing this cycle (same-cycle reuse so
   // N=2 still sustains 1 dispatch/cycle). issue is gated by exec_busy.
   wire issue = found & ~exec_busy;
   reg           inv_avail;
   reg [NW-1:0]  inv_idx;
   always @* begin
      inv_avail = 1'b0; inv_idx = {NW{1'b0}};
      for (e = N-1; e >= 0; e = e - 1) if (!v[e]) begin inv_avail = 1'b1; inv_idx = e[NW-1:0]; end
   end
   wire [NW-1:0] dst = inv_avail ? inv_idx : sel;       // where a new dispatch lands
   assign disp_ready = inv_avail | issue;

   // v is an unpacked array -> reduce procedurally (elig is packed, indexed to match).
   reg dbg_anyv_r, dbg_stuck_r; integer dv;
   always @* begin
      dbg_anyv_r = 1'b0; dbg_stuck_r = 1'b0;
      for (dv = 0; dv < N; dv = dv + 1) if (v[dv]) begin
         dbg_anyv_r = 1'b1;
         if (!elig[dv]) dbg_stuck_r = 1'b1;
      end
   end
   assign dbg_any_v  = dbg_anyv_r;
   assign dbg_stuck  = dbg_stuck_r;
   assign iss_valid  = issue;
   assign iss_seq    = sq [sel];
   assign iss_pdst   = pd [sel];
   assign iss_pdst_v = pdv[sel];
   assign iss_ps1    = s1 [sel];
   assign iss_ps2    = s2 [sel];
   assign iss_ps3    = s3 [sel];
   assign iss_ckpt   = ck [sel];
   assign iss_mem_idx= mi [sel];
   assign iss_pay    = py [sel];

   // seed a new entry's ready bits: in the table & not cleared this cycle, OR woken this
   // cycle. A non-dependency operand is p0 (ready[0] hardwired), so no "need" term is
   // needed. Computed procedurally at the dispatch edge (NOT a continuous assign --
   // match()/clr_hit() read wake_valid/clr_valid, which a wire's sensitivity would miss,
   // leaving the seed stale; same gotcha as the aligner's hwr()).
   reg seed1, seed2, seed3;

   // ------------------------------------------------------------- sequential
   integer s;
   always @(posedge clk) begin
      if (reset) begin
         for (k = 0; k < N; k = k + 1) v[k] <= 1'b0;
      end else begin
         // CAM wakeup of live entries (catch a tag matching the result broadcast)
         for (k = 0; k < N; k = k + 1) if (v[k]) begin
            if (!r1[k] && match(s1[k])) r1[k] <= 1'b1;
            if (!r2[k] && match(s2[k])) r2[k] <= 1'b1;
            if (!r3[k] && match(s3[k])) r3[k] <= 1'b1;
         end

         // issue: free the selected entry
         if (issue) v[sel] <= 1'b0;

         // branch squash: invalidate entries younger than the mispredicting branch
         if (squash)
            for (k = 0; k < N; k = k + 1)
               if (v[k] && ($signed(sq[k] - squash_seq) > 0)) v[k] <= 1'b0;

         // dispatch: write the new entry (overrides a same-cycle issue-free of this slot)
         if (disp_valid && disp_ready) begin
            // no per-operand "need" bit: a non-dependency arrives as p0 (constant zero,
            // ready[0] hardwired in the shared table), so its seed is just ready & ~clr |
            // wake. disp_rdyX is the shared scoreboard's pre-edge read of ready[disp_psX].
            // A tag being (re)allocated as a dest THIS cycle (clr_hit) belongs to a producer
            // that has only just dispatched -> its value is not ready, and any wake for that
            // tag this cycle is a STALE wake for the PRIOR owner of that physreg (reuse races
            // a lingering broadcast, e.g. after a rollback compresses physreg recycling).
            // Mask the whole seed (rdy AND match) by ~clr_hit, not just rdy.
            seed1 = (disp_rdy1 | match(disp_ps1)) & ~clr_hit(disp_ps1);
            seed2 = (disp_rdy2 | match(disp_ps2)) & ~clr_hit(disp_ps2);
            seed3 = (disp_rdy3 | match(disp_ps3)) & ~clr_hit(disp_ps3);
            v  [dst] <= 1'b1;
            sq [dst] <= disp_seq;
            pd [dst] <= disp_pdst;  pdv[dst] <= disp_pdst_v;
            s1 [dst] <= disp_ps1;   r1 [dst] <= seed1;
            s2 [dst] <= disp_ps2;   r2 [dst] <= seed2;
            s3 [dst] <= disp_ps3;   r3 [dst] <= seed3;
            ck [dst] <= disp_ckpt;
            mi [dst] <= disp_mem_idx; py[dst] <= disp_pay;
         end
      end
   end
endmodule

`default_nettype wire
