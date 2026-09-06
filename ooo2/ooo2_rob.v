`default_nettype none

// Reorder buffer for the in-order core -- the step that turns "M is the commit point" into
// "the ROB head is the commit point", which is the substrate out-of-order issue needs.
//
// WHAT THIS DOES NOT DO, and why it is small. `ooo2_rename` already carries the whole
// speculative/committed split: SMAP/RMAP/lv for the map, and a per-shard free list with a
// speculative head (h_*) and a committed head (hc_*), where rollback is `h := hc` in one
// cycle with no walk. That structure already supports N uncommitted instructions -- today N
// is simply always <= 1 because M blocks. So the ROB does NOT manage the free list, does not
// manage the map, and does not walk anything on a squash. It holds the commit RECORD and
// puts it back in program order. Everything else was already OoO-ready.
//
// docs/Area-Efficient-Scalar-OoO.md 2: "The reorder buffer holds status, not data." Same
// here -- no result values, no PC, no operands. An entry is {rd, rd_v, shard, prd, pold},
// which is exactly and only what `ooo2_rename`'s commit port consumes.
//
// SHADOW BRING-UP (rule I3, the playbook that landed rename and caught five defects no test
// reached). At this milestone M still blocks, so the ROB never holds more than one live
// entry and its commit stream must be BIT-IDENTICAL to the M-stage commit it shadows;
// ooo2_core asserts exactly that, every cycle. What that proves is allocation, the head/tail
// march and wrap, in-order commit, and the rename hand-off. What it cannot prove -- depth,
// out-of-order `done`, and multi-entry squash -- is precisely what the next step adds, on a
// commit path that will by then be known good.
module ooo2_rob
  #(parameter DEPTH = 16,
    parameter IDXB  = 4,              // $clog2(DEPTH)
    parameter PBITS = 9,
    parameter NW    = 3)              // simultaneous completion ports
   (input  wire             clk,
    input  wire             reset,

    // ---- dispatch: in program order, one per cycle, at RENAME time ----
    input  wire             d_valid,
    input  wire [5:0]       d_rd,
    // 0 when the instruction writes no register. Physical register 0 is never allocated --
    // it is architectural x0's permanent mapping and is never freed -- so `d_prd != 0` IS
    // the writes-a-register predicate. doc 5: "There is no `allocates` bit".
    input  wire [PBITS-1:0] d_prd,
    // Commits but must not be COUNTED. The interrupt pseudo-op normally traps, and a trap
    // never commits at all -- but rule D3 records that an injected OP_IRQ can commit with
    // its trap not firing, so c_kill does not cover it and minstret would gain an
    // instruction that architecturally does not exist.
    input  wire             d_noret,
    output wire             d_ready,      // room to allocate
    output wire [IDXB-1:0]  d_idx,        // the slot this dispatch takes; ride it with the op
    // second allocation, the same cycle, YOUNGER (2026-09-05, item 10b): tie d_valid2 low
    // and the buffer allocates one per cycle as before
    input  wire             d_valid2,
    input  wire [5:0]       d_rd2,
    input  wire [PBITS-1:0] d_prd2,
    input  wire             d_noret2,
    output wire             d_ready2,     // room for TWO
    output wire [IDXB-1:0]  d_idx2,

    // ---- completion: out of order, names its slot by the tag it was given ----
    // NW ports. One was enough while a single stage completed everything; with units
    // running independently, an ALU result, a landing load and a landing FP result can
    // all finish in the same cycle, and making them queue for a completion port would
    // reintroduce exactly the serialisation dynamic issue exists to remove.
    input  wire [NW-1:0]           w_v,
    input  wire [NW*IDXB-1:0]      w_ix,

    // ---- commit: the head, in order, straight into ooo2_rename's commit port ----
        input  wire             c_kill,       // head is trapping/squashed: retire it, free nothing
    // The entry behind the head may not retire while M still holds its op: a trap or a
    // redirect resolved in M waits there for the HEAD with the done bit already set, and
    // retiring it from behind the head would retire the wrong path (109 of 240 riscv-tests,
    // 2026-09-05, all at the first mispredict). The core drives this from M's slot compare.
    input  wire             c2_kill,
    output wire             c_valid,
    output wire [5:0]       c_rd,
    output wire             c_rd_v,      // derived: |c_prd
    output wire [PBITS-1:0] c_prd,
        output wire             c_noret,
    // the entry behind the head, retired in the same cycle when it is done (item 10c):
    // a done entry can no longer restart the machine (every restart waits for the head
    // and is done only after), so nothing between the two can intervene
    output wire             c2_valid,
    output wire [5:0]       c2_rd,
    output wire             c2_rd_v,
    output wire [PBITS-1:0] c2_prd,
    output wire             c2_noret,

    // ---- recovery ----
    // Younger-than-the-committing-entry dies. Pointer-only, to match the free list's own
    // rollback: there is nothing to walk because rename never wrote the free-list array.
    input  wire             flush,
    output wire             empty,
    // Which slot is oldest. An instruction that does anything beyond writing its own
    // register -- trap, redirect -- may act only when it IS this slot, or it would squash an
    // older op still in flight ahead of it.
    output wire [IDXB-1:0]  head_idx,
    // THE IRREVOCABLE POINT: the oldest entry that is not done. In this core every op that
    // can restart the machine -- a mispredicted branch, a trap, a system op, fence.i, the
    // interrupt pseudo-op -- waits in M for the ROB head and sets `done` only when it has
    // completed there, so an entry that is done can no longer restart, and everything older
    // than the first not-done entry is settled. A store is committable exactly when this
    // pointer reaches it (its own `done` is what the commit sets); it need not wait for the
    // head, which used to sit on every store for the ~6 cycles the cache takes.
    output wire [IDXB-1:0]  irr_idx,
    output wire             irr_v);       // ...and that entry exists

   // {noret, rd, prd} and nothing else -- 16 bits/entry against 28. rd_v is `|prd`; the
   // destination SHARD is the top bits of prd (a physical register's shard is encoded in its
   // number and never changes); and the DISPLACED register is not carried at all, because
   // ooo2_rename reads rmap[c_rd] at commit and that still holds it. doc 5, doc 5.1.
   // The ROB is the LARGE structure and the scheduler the small one, so anything that can
   // be derived, or that only issue needs, does not belong here.
   localparam EW = 1 + 6 + PBITS;                  // {noret, rd, prd}
   localparam [IDXB:0] DEPTH_S = DEPTH[IDXB:0];   // sized, so the occupancy check cannot truncate

   // TWO PARITY BANKS for the entries since the second allocation (item 10b): tail and tail+1
   // differ in parity, so each bank takes one write per cycle -- expressed as ONE {we, addr,
   // data} per bank; as two statements synthesis saw two write ports and demoted the array
   // to flops (gate V2's RAM-inference check, 2026-09-05). Entry i is bank i[0], index i>>1.
   reg [EW-1:0]    ent0 [0:DEPTH/2-1];
   reg [EW-1:0]    ent1 [0:DEPTH/2-1];
   reg [DEPTH-1:0] v, done;                     // bulk-cleared on flush, so flops by necessity
   reg [IDXB:0]    head, tail;                  // one extra MSB: full and empty differ by it
   reg [IDXB:0]    irr;                         // head <= irr <= tail, same width
   integer         ri, rj;
   initial begin
      v = {DEPTH{1'b0}}; done = {DEPTH{1'b0}}; head = 0; tail = 0; irr = 0;
      for (ri = 0; ri < DEPTH/2; ri = ri + 1) begin ent0[ri] = {EW{1'b0}}; ent1[ri] = {EW{1'b0}}; end
   end

   wire [IDXB-1:0] hidx = head[IDXB-1:0];
   wire [IDXB-1:0] tidx = tail[IDXB-1:0];
   assign empty   = (head == tail);
   wire   full    = (head[IDXB-1:0] == tail[IDXB-1:0]) && (head[IDXB] != tail[IDXB]);
   assign d_ready = ~full;
   wire [IDXB:0] occ = tail - head;
   assign d_ready2 = (occ <= DEPTH_S - 2);
   assign head_idx = hidx;
   assign d_idx   = tidx;
   wire [IDXB-1:0] tidx2 = tidx + 1'b1;
   assign d_idx2  = tidx2;

   // Write-forward on the head's done bit. An op that completes IN the cycle its entry is at
   // the head must commit that same cycle, or every completion costs an extra cycle -- and
   // at this milestone that would make the shadow's commit stream one cycle late and no
   // longer bit-identical to the M-stage commit it is checked against.
   // Write-forward across every port: an op completing IN the cycle its entry reaches the
   // head must commit that same cycle, or every completion costs an extra cycle.
   function automatic w_hits;
      input [IDXB-1:0] ix;
      integer q;
      begin
         w_hits = 1'b0;
         for (q = 0; q < NW; q = q + 1)
            if (w_v[q] && (w_ix[q*IDXB +: IDXB] == ix)) w_hits = 1'b1;
      end
   endfunction
   wire head_done = v[hidx] & (done[hidx] | w_hits(hidx));
   wire [IDXB-1:0] iidx = irr[IDXB-1:0];
   assign irr_idx = iidx;
   assign irr_v   = (irr != tail);
   // one entry per cycle, with the same write-forward the head uses: the commit that sets a
   // store's `done` this cycle moves the pointer past it next cycle
   wire irr_done  = (irr != tail) & v[iidx] & (done[iidx] | w_hits(iidx));

   // c_kill: the head is trapping. It must NOT commit -- a trap does not write rd -- and the
   // redirect that follows flushes it, which returns its allocation through the free list's
   // own pointer rollback. So a trapping instruction is simply never committed; there is no
   // separate "retire without freeing" path to get wrong.
   wire [EW-1:0] he = hidx[0] ? ent1[hidx[IDXB-1:1]] : ent0[hidx[IDXB-1:1]];
   assign c_valid = head_done & ~c_kill;
   assign c_prd   = he[PBITS-1:0];
   assign c_rd    = he[PBITS +: 6];
   assign c_noret = he[PBITS+6];
      assign c_rd_v  = |c_prd;
   wire [IDXB-1:0] h2idx = hidx + 1'b1;
   wire [EW-1:0]   he2   = h2idx[0] ? ent1[h2idx[IDXB-1:1]] : ent0[h2idx[IDXB-1:1]];
   wire            head2_done = v[h2idx] & (done[h2idx] | w_hits(h2idx));
      // ...and never in a flush cycle: a mispredicted branch COMMITS and redirects in the same
   // cycle, and the entry behind it is the wrong path (rv64ui-v-add retired the fall-through
   // of a taken loop branch, 2026-09-05).
   assign c2_valid = c_valid & head2_done & ~c2_kill & ~flush;
   assign c2_prd   = he2[PBITS-1:0];
   assign c2_rd    = he2[PBITS +: 6];
   assign c2_noret = he2[PBITS+6];
   assign c2_rd_v  = |c2_prd;
   wire do_alloc  = d_valid & d_ready;                  // in a flush cycle too: the flush arm below wins
   wire do_alloc2 = do_alloc & d_valid2 & d_ready2;
   wire do_commit = c_valid;
   wire do_commit2 = c2_valid;
   wire [IDXB:0] head_n = head + {{IDXB{1'b0}}, do_commit} + {{IDXB{1'b0}}, do_commit2};
   // the irrevocable pointer never falls behind the head: two entries retiring in one cycle
   // are both done, so the pointer is at least past them
   wire [IDXB:0] irr_step = irr_done ? irr + 1'b1 : irr;
   wire [IDXB:0] irr_n    = (do_commit2 && ((irr_step == head) || (irr_step == head + 1'b1))) ? head + 2'd2 : irr_step;
   always @(posedge clk) if (!reset && d_valid2 && !d_valid)
      $fatal(1, "ooo2_rob: second allocation without a first");
   wire            ew0 = (do_alloc & ~tidx[0]) | (do_alloc2 & ~tidx2[0]);
   wire            ew1 = (do_alloc &  tidx[0]) | (do_alloc2 &  tidx2[0]);
   wire [IDXB-2:0] ea0 = (do_alloc & ~tidx[0]) ? tidx[IDXB-1:1] : tidx2[IDXB-1:1];
   wire [IDXB-2:0] ea1 = (do_alloc &  tidx[0]) ? tidx[IDXB-1:1] : tidx2[IDXB-1:1];
   wire [EW-1:0]   ed0 = (do_alloc & ~tidx[0]) ? {d_noret, d_rd, d_prd} : {d_noret2, d_rd2, d_prd2};
   wire [EW-1:0]   ed1 = (do_alloc &  tidx[0]) ? {d_noret, d_rd, d_prd} : {d_noret2, d_rd2, d_prd2};

   always @(posedge clk) begin
      if (reset) begin
         v <= {DEPTH{1'b0}}; done <= {DEPTH{1'b0}}; head <= 0; tail <= 0;
      end else begin
         if (ew0) ent0[ea0] <= ed0;
         if (ew1) ent1[ea1] <= ed1;
         if (do_alloc) begin
            v[tidx]    <= 1'b1;
            done[tidx] <= 1'b0;
            tail       <= tail + 1'b1 + {{IDXB{1'b0}}, do_alloc2};
         end
         if (do_alloc2) begin
            v[tidx2]    <= 1'b1;
            done[tidx2] <= 1'b0;
         end
         for (ri = 0; ri < NW; ri = ri + 1)
            if (w_v[ri]) done[w_ix[ri*IDXB +: IDXB]] <= 1'b1;
                  if (do_commit) begin
            v[hidx] <= 1'b0;
            if (do_commit2) v[h2idx] <= 1'b0;
            head    <= head_n;
         end
         irr <= irr_n;
         // A flush kills everything YOUNGER than the entry committing this cycle -- the
         // redirecting instruction is itself older and must still commit. Ordered after the
         // commit arm above so the head's own retirement stands.
         if (flush) begin
            v    <= {DEPTH{1'b0}};
            done <= {DEPTH{1'b0}};
                        tail <= head_n;
            irr  <= head_n;
            head <= head_n;
         end
      end
   end
   // The pointer never lags the head and never passes the tail (docs/rtl-rules.md A1).
   always @(posedge clk) if (!reset) begin
      if ((irr - head) > (tail - head))
         $fatal(1, "ooo2_rob: the irrevocable pointer left [head, tail]: head=%0d irr=%0d tail=%0d", head, irr, tail);
   end

   // ---- invariants (always on: docs/rtl-rules.md A1) --------------------------------
   always @(posedge clk) if (!reset) begin
      if (d_valid & ~d_ready)
         $fatal(1, "ooo2_rob: dispatch into a full ROB (head=%0d tail=%0d)", head, tail);
      for (ri = 0; ri < NW; ri = ri + 1) if (w_v[ri]) begin
         if (~v[w_ix[ri*IDXB +: IDXB]])
            $fatal(1, "ooo2_rob: completion for slot %0d, which holds no live entry",
                   w_ix[ri*IDXB +: IDXB]);
         if (done[w_ix[ri*IDXB +: IDXB]])
            $fatal(1, "ooo2_rob: slot %0d completed twice", w_ix[ri*IDXB +: IDXB]);
         for (rj = 0; rj < NW; rj = rj + 1)
            if (rj > ri && w_v[rj] && (w_ix[rj*IDXB +: IDXB] == w_ix[ri*IDXB +: IDXB]))
               $fatal(1, "ooo2_rob: two ports completing slot %0d in one cycle",
                      w_ix[ri*IDXB +: IDXB]);
      end
      if (c_valid & ~v[hidx])
         $fatal(1, "ooo2_rob: committing an invalid head (head=%0d)", head);
      // Occupancy can never exceed the array. Catches a lost commit or a double allocate at
      // the moment it happens rather than as a wedge thousands of cycles later.
      if ((tail - head) > DEPTH_S)
         $fatal(1, "ooo2_rob: occupancy %0d exceeds DEPTH %0d", tail - head, DEPTH);
   end
endmodule

`default_nettype wire
