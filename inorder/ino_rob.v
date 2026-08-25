`default_nettype none

// Reorder buffer for the in-order core -- the step that turns "M is the commit point" into
// "the ROB head is the commit point", which is the substrate out-of-order issue needs.
//
// WHAT THIS DOES NOT DO, and why it is small. `ino_rename` already carries the whole
// speculative/committed split: SMAP/RMAP/lv for the map, and a per-shard free list with a
// speculative head (h_*) and a committed head (hc_*), where rollback is `h := hc` in one
// cycle with no walk. That structure already supports N uncommitted instructions -- today N
// is simply always <= 1 because M blocks. So the ROB does NOT manage the free list, does not
// manage the map, and does not walk anything on a squash. It holds the commit RECORD and
// puts it back in program order. Everything else was already OoO-ready.
//
// docs/Area-Efficient-Scalar-OoO.md 2: "The reorder buffer holds status, not data." Same
// here -- no result values, no PC, no operands. An entry is {rd, rd_v, shard, prd, pold},
// which is exactly and only what `ino_rename`'s commit port consumes.
//
// SHADOW BRING-UP (rule I3, the playbook that landed rename and caught five defects no test
// reached). At this milestone M still blocks, so the ROB never holds more than one live
// entry and its commit stream must be BIT-IDENTICAL to the M-stage commit it shadows;
// ino_core asserts exactly that, every cycle. What that proves is allocation, the head/tail
// march and wrap, in-order commit, and the rename hand-off. What it cannot prove -- depth,
// out-of-order `done`, and multi-entry squash -- is precisely what the next step adds, on a
// commit path that will by then be known good.
module ino_rob
  #(parameter DEPTH = 16,
    parameter IDXB  = 4,              // $clog2(DEPTH)
    parameter PBITS = 9)
   (input  wire             clk,
    input  wire             reset,

    // ---- dispatch: in program order, one per cycle, at RENAME time ----
    input  wire             d_valid,
    input  wire [5:0]       d_rd,
    input  wire             d_rd_v,
    input  wire [1:0]       d_shard,
    input  wire [PBITS-1:0] d_prd,
    input  wire [PBITS-1:0] d_pold,
    output wire             d_ready,      // room to allocate
    output wire [IDXB-1:0]  d_idx,        // the slot this dispatch takes; ride it with the op

    // ---- completion: out of order, names its slot by the tag it was given ----
    input  wire             w_valid,
    input  wire [IDXB-1:0]  w_idx,

    // ---- commit: the head, in order, straight into ino_rename's commit port ----
    input  wire             c_kill,       // head is trapping/squashed: retire it, free nothing
    output wire             c_valid,
    output wire [5:0]       c_rd,
    output wire             c_rd_v,
    output wire [1:0]       c_shard,
    output wire [PBITS-1:0] c_prd,
    output wire [PBITS-1:0] c_pold,

    // ---- recovery ----
    // Younger-than-the-committing-entry dies. Pointer-only, to match the free list's own
    // rollback: there is nothing to walk because rename never wrote the free-list array.
    input  wire             flush,
    output wire             empty,
    // Which slot is oldest. An instruction that does anything beyond writing its own
    // register -- trap, redirect -- may act only when it IS this slot, or it would squash an
    // older op still in flight ahead of it.
    output wire [IDXB-1:0]  head_idx);

   localparam EW = 6 + 1 + 2 + PBITS + PBITS;   // {rd, rd_v, shard, prd, pold}
   localparam [IDXB:0] DEPTH_S = DEPTH[IDXB:0];   // sized, so the occupancy check cannot truncate

   reg [EW-1:0]    ent [0:DEPTH-1];
   reg [DEPTH-1:0] v, done;                     // bulk-cleared on flush, so flops by necessity
   reg [IDXB:0]    head, tail;                  // one extra MSB: full and empty differ by it
   integer         ri;
   initial begin
      v = {DEPTH{1'b0}}; done = {DEPTH{1'b0}}; head = 0; tail = 0;
      for (ri = 0; ri < DEPTH; ri = ri + 1) ent[ri] = {EW{1'b0}};
   end

   wire [IDXB-1:0] hidx = head[IDXB-1:0];
   wire [IDXB-1:0] tidx = tail[IDXB-1:0];
   assign empty   = (head == tail);
   wire   full    = (head[IDXB-1:0] == tail[IDXB-1:0]) && (head[IDXB] != tail[IDXB]);
   assign d_ready = ~full;
   assign head_idx = hidx;
   assign d_idx   = tidx;

   // Write-forward on the head's done bit. An op that completes IN the cycle its entry is at
   // the head must commit that same cycle, or every completion costs an extra cycle -- and
   // at this milestone that would make the shadow's commit stream one cycle late and no
   // longer bit-identical to the M-stage commit it is checked against.
   wire head_done = v[hidx] & (done[hidx] | (w_valid & (w_idx == hidx)));

   // c_kill: the head is trapping. It must NOT commit -- a trap does not write rd -- and the
   // redirect that follows flushes it, which returns its allocation through the free list's
   // own pointer rollback. So a trapping instruction is simply never committed; there is no
   // separate "retire without freeing" path to get wrong.
   wire [EW-1:0] he = ent[hidx];
   assign c_valid = head_done & ~c_kill;
   assign c_pold  = he[PBITS-1:0];
   assign c_prd   = he[2*PBITS-1 -: PBITS];
   assign c_shard = he[2*PBITS +: 2];
   assign c_rd_v  = he[2*PBITS+2];
   assign c_rd    = he[2*PBITS+3 +: 6];

   wire do_alloc  = d_valid & d_ready & ~flush;
   wire do_commit = c_valid;

   always @(posedge clk) begin
      if (reset) begin
         v <= {DEPTH{1'b0}}; done <= {DEPTH{1'b0}}; head <= 0; tail <= 0;
      end else begin
         if (do_alloc) begin
            ent[tidx]  <= {d_rd, d_rd_v, d_shard, d_prd, d_pold};
            v[tidx]    <= 1'b1;
            done[tidx] <= 1'b0;
            tail       <= tail + 1'b1;
         end
         if (w_valid) done[w_idx] <= 1'b1;
         if (do_commit) begin
            v[hidx] <= 1'b0;
            head    <= head + 1'b1;
         end
         // A flush kills everything YOUNGER than the entry committing this cycle -- the
         // redirecting instruction is itself older and must still commit. Ordered after the
         // commit arm above so the head's own retirement stands.
         if (flush) begin
            v    <= {DEPTH{1'b0}};
            done <= {DEPTH{1'b0}};
            tail <= do_commit ? (head + 1'b1) : head;
            if (do_commit) head <= head + 1'b1;
         end
      end
   end

   // ---- invariants (always on: docs/rtl-rules.md A1) --------------------------------
   always @(posedge clk) if (!reset) begin
      if (d_valid & ~d_ready)
         $fatal(1, "ino_rob: dispatch into a full ROB (head=%0d tail=%0d)", head, tail);
      if (w_valid & ~v[w_idx])
         $fatal(1, "ino_rob: completion for slot %0d, which holds no live entry", w_idx);
      if (w_valid & done[w_idx])
         $fatal(1, "ino_rob: slot %0d completed twice", w_idx);
      if (c_valid & ~v[hidx])
         $fatal(1, "ino_rob: committing an invalid head (head=%0d)", head);
      // Occupancy can never exceed the array. Catches a lost commit or a double allocate at
      // the moment it happens rather than as a wedge thousands of cycles later.
      if ((tail - head) > DEPTH_S)
         $fatal(1, "ino_rob: occupancy %0d exceeds DEPTH %0d", tail - head, DEPTH);
   end
endmodule

`default_nettype wire
