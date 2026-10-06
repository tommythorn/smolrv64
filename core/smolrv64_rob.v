`default_nettype none

// Reorder buffer for smolrv64_core: the ROB head is the commit point.
//
// smolrv64_rename carries the whole speculative/committed split: SMAP/RMAP/lv for the map,
// and a per-shard free list with a speculative head and a committed head, where rollback is
// a pointer restore with no walk. So the ROB does not manage the free list or the map and
// walks nothing on a squash: it holds the commit RECORD and puts it back in program order.
//
// docs/Area-Efficient-Scalar-OoO.md 2: "The reorder buffer holds status, not data." An entry
// is {noret, rd, prd}, which is exactly what `smolrv64_rename`'s commit port consumes.
//
// ROWS: a dispatch group takes a row, slot k in column k, so an entry's index is {row, column}
// and column k is written only by slot k and read only by commit port k. The head commits
// along its row from its first uncommitted column; a row may take several cycles to commit,
// and the head moves to the next row when the row's last valid column commits. Positions
// (head, tail, irr) are {wrap, row, column}, ordered as numbers; the tail is always column 0.
module smolrv64_rob
  #(parameter ROWS  = 32,
    parameter IDXB  = 7,              // $clog2(ROWS) + CB: an entry is {row, column}
    parameter PBITS = 9,
    parameter IW    = 2,              // the group width: IW allocations and IW commits a cycle
    parameter NW    = 3,              // simultaneous completion ports
    // the completion ports the irrevocable pointer reads in their own cycle (the store
    // queue's): a store passes `irr` the cycle it completes and leaves the queue a cycle sooner
    parameter [NW-1:0] IRR_FWD = {NW{1'b0}},
    parameter CB    = (IW > 1) ? $clog2(IW) : 1)   // column bits
   (input  wire                   clk,
    input  wire                   reset,

    // ---- dispatch: a group in program order (slot k at [k]; slot k only with k-1), at RENAME ----
    input  wire [IW-1:0]          d_valid,
    input  wire [IW*6-1:0]        d_rd,
    // 0 when the instruction writes no register. Physical register 0 is never allocated --
    // it is architectural x0's permanent mapping and is never freed -- so `d_prd != 0` IS
    // the writes-a-register predicate.
    input  wire [IW*PBITS-1:0]    d_prd,
    // Commits but must not be COUNTED. The interrupt pseudo-op normally traps, and a trap
    // never commits at all -- but rule D3 records that an injected OP_IRQ can commit with
    // its trap not firing, so c_kill does not cover it and minstret would gain an
    // instruction that architecturally does not exist.
    input  wire [IW-1:0]          d_noret,
    input  wire [IW*PBITS-1:0]    d_pold,       // the mapping rd's rename displaced (the walk's)
    output wire [IW-1:0]          d_ready,      // a row is free (every bit)
    output wire [IW*IDXB-1:0]     d_idx,        // the entry each takes, {row, k}; ride it with the op

    // ---- completion: out of order, names its slot by the tag it was given ----
    input  wire [NW-1:0]          w_v,
    input  wire [NW*IDXB-1:0]     w_ix,
    // the head completes in a flush cycle (a mispredict at its squash, a redirecting system
    // op): it commits in that cycle, before the flush. Every other completion commits from
    // `done` a cycle later, which keeps the completion cones out of retire.
    input  wire                   h_fin,

    // ---- commit: column k of the head row on port k, in order, into smolrv64_rename ----
    // c_kill[k] for the head's column: the head is trapping (retire nothing, free nothing). For
    // a column behind it: that entry may not retire this cycle -- it retires only with every
    // one before it, and never while M still holds its op (a trap or redirect resolved in M
    // waits there for the head with done set; retiring it from behind the head retires the
    // wrong path).
    input  wire [IW-1:0]          c_kill,
    output wire [IW-1:0]          c_valid,
    output wire [IW*6-1:0]        c_rd,
    output wire [IW-1:0]          c_rd_v,       // derived: |c_prd
    output wire [IW*PBITS-1:0]    c_prd,
    output wire [IW-1:0]          c_noret,

    // ---- recovery ----
    // Younger-than-the-committing-entry dies. Pointer-only, to match the free list's own
    // rollback: there is nothing to walk because rename never wrote the free-list array.
    input  wire                   flush,
    output wire                   empty,
    output wire [IDXB-CB:0]       occ_n,        // rows allocated now: the dispatch credit
    // Which slot is oldest. An instruction that does anything beyond writing its own
    // register -- trap, redirect -- may act only when it IS this slot, or it would squash an
    // older op still in flight ahead of it.
    output wire [IDXB-1:0]        head_idx,
    // THE IRREVOCABLE POINT: the oldest entry that is not done. Every op that can restart the
    // machine waits for the ROB head and sets `done` only when it has completed there, so an
    // entry that is done can no longer restart, and everything older than the first not-done
    // entry is settled. A store is committable exactly when this pointer reaches it.
    output wire [IDXB-1:0]        irr_idx,
    output wire                   irr_v,        // ...and that entry exists

    // ---- the walk: from the tail back to a mispredicted branch, a row a cycle ----
    // k_v names the branch (a later k_v names an older one: the walk goes on to it). Each cycle
    // the walk names the dead entries of one row that write a register, column k on [k], with
    // their rd, prd and displaced mapping, for smolrv64_rename to undo. wl_done: the walk has
    // reached its branch (until the flush).
    input  wire                   k_v,
    input  wire [IDXB-1:0]        k_idx,
    output wire [IW-1:0]          wl_v,
    output wire [IW*6-1:0]        wl_rd,
    output wire [IW*PBITS-1:0]    wl_prd, wl_pold,
    output wire                   wl_done,
    // THE KILL: every entry younger than the branch, one bit per entry index, from flops, for
    // every structure that holds an op to look its own up in (kd_v & kd[its rob]). Valid from
    // the second cycle after k_v until the flush.
    output reg                    kd_v,
    output reg  [(1 << IDXB)-1:0] kd,
    // THE RELEASE: the walk has reached the branch and every dead op is gone, so the dead entries
    // leave and the tail returns to the row after the branch's; dispatch resumes behind it
    input  wire                   rel);

   // {noret, rd, prd} and nothing else. rd_v is `|prd`; the destination SHARD is the top bits
   // of prd; and the DISPLACED register is not carried at all, because smolrv64_rename reads
   // rmap[c_rd] at commit and that still holds it.
   localparam EW    = PBITS + 1 + 6 + PBITS;      // {pold, noret, rd, prd}
   localparam NCOL  = 1 << CB;                    // columns; those at IW and above are never valid
   localparam DEPTH = ROWS * NCOL;                // the index space
   localparam RB    = IDXB - CB;                  // row bits
   localparam [RB:0] ROWS_S = ROWS[RB:0];         // sized, so the occupancy check cannot truncate
   localparam [IDXB:0] ROW1 = NCOL[IDXB:0];       // one row, as a position step
   reg [DEPTH-1:0] v, done;                     // bulk-cleared on flush, so flops by necessity
   reg [IDXB:0]    head, tail, irr;             // {wrap, row, column}; head <= irr <= tail
   integer         ri, rj;
   initial begin
      v = {DEPTH{1'b0}}; done = {DEPTH{1'b0}}; head = 0; tail = 0; irr = 0;
   end

   wire [IDXB-1:0] hidx = head[IDXB-1:0];
   wire [RB-1:0]   hrow = head[IDXB-1:CB];
   wire [CB-1:0]   hcol = head[CB-1:0];
   wire [RB-1:0]   trow = tail[IDXB-1:CB];
   wire [RB:0]     occ  = tail[IDXB:CB] - head[IDXB:CB];   // rows, the head's counting
   assign empty    = (head == tail);
   assign occ_n    = occ;
   assign head_idx = hidx;
   wire ready = (occ < ROWS_S);
   // slot k reads slot k-1: per-bit variables, so the chains are not loops to the simulator
   wire [IW-1:0]   cv /*verilator split_var*/;
   wire [IW-1:0]   do_alloc = d_valid & {IW{ready}};
   genvar gk;
   generate for (gk = 0; gk < IW; gk = gk + 1) begin: sl
      localparam [CB-1:0] K = gk;
      assign d_idx[gk*IDXB +: IDXB] = {trow, K};
      assign d_ready[gk] = ready;
   end endgenerate

   function automatic w_hits;          // (the h_fin assertion's)
      input [IDXB-1:0] ix;
      integer q;
      begin
         w_hits = 1'b0;
         for (q = 0; q < NW; q = q + 1)
            if (w_v[q] && (w_ix[q*IDXB +: IDXB] == ix)) w_hits = 1'b1;
      end
   endfunction
   wire head_done = v[hidx] & (done[hidx] | h_fin);
   wire [IDXB-1:0] iidx = irr[IDXB-1:0];
   assign irr_idx = iidx;
   assign irr_v   = (irr != tail);
   // one entry per cycle, over entries already done or completing on an IRR_FWD port
   function automatic irr_hits;
      input [IDXB-1:0] ix;
      integer q;
      begin
         irr_hits = 1'b0;
         for (q = 0; q < NW; q = q + 1)
            if (IRR_FWD[q] && w_v[q] && (w_ix[q*IDXB +: IDXB] == ix)) irr_hits = 1'b1;
      end
   endfunction
   wire irr_done = (irr != tail) & v[iidx] & (done[iidx] | irr_hits(iidx));
   // the entry after a position: the next column of its row while that is valid, else the next row
   function automatic [IDXB:0] next_pos(input [IDXB:0] p);
      begin
         if ((p[CB-1:0] != NCOL - 1) && v[p[IDXB-1:0] + 1'b1]) next_pos = p + 1'b1;
         else next_pos = {p[IDXB:CB] + 1'b1, {CB{1'b0}}};
      end
   endfunction

   // ---- the walk ----
   // The next row to walk (wl_row) and the columns of it still to walk (those below wl_lim);
   // the branch is {k_row, k_col}, and only its younger columns die in its own row. When the
   // walk reaches the branch's row it stops there with wl_lim = k_col + 1, so a later, older
   // branch walks on from exactly the entries not yet walked, the old branch included.
   reg            wl_run, wl_end;
   reg [RB-1:0]   wl_row;
   reg [CB:0]     wl_lim;
   reg [RB-1:0]   k_row;
   reg [CB-1:0]   k_col;
   initial begin wl_run = 1'b0; wl_end = 1'b0; wl_row = 0; wl_lim = 0; k_row = 0; k_col = 0; end
   assign wl_done = wl_end & ~wl_run & ~k_v;      // not in a cycle that names a new, older branch
   localparam [CB:0] NCOL_S = NCOL;
   always @(posedge clk) begin
      if (reset | flush | rel) begin
         wl_run <= 1'b0;  wl_end <= 1'b0;
      end else begin
         if (wl_run) begin
            if (wl_row == k_row) begin wl_run <= 1'b0;  wl_end <= 1'b1;  wl_lim <= {1'b0, k_col} + 1'b1; end
            else begin wl_row <= wl_row - 1'b1;  wl_lim <= NCOL_S; end
         end
         if (k_v) begin
            k_row <= k_idx[IDXB-1:CB];  k_col <= k_idx[CB-1:0];  wl_run <= 1'b1;
            if (~wl_run & ~wl_end) begin wl_row <= trow - 1'b1;  wl_lim <= NCOL_S; end
         end
      end
   end
   // the kill vector: a row after the branch's (in ring order from the head) dies whole, and in
   // the branch's own row the columns after it; computed from flops the cycle k_v arrives
   wire [RB-1:0] kb_rel = k_idx[IDXB-1:CB] - hrow;
   reg  [DEPTH-1:0] kd_n;
   integer kr, kc;
   always @* for (kr = 0; kr < ROWS; kr = kr + 1)
      for (kc = 0; kc < NCOL; kc = kc + 1)
         kd_n[kr*NCOL + kc] = ((kr[RB-1:0] - hrow) > kb_rel)
                            | ((kr[RB-1:0] == k_idx[IDXB-1:CB]) & (kc[CB-1:0] > k_idx[CB-1:0]));
   initial begin kd_v = 1'b0; kd = {DEPTH{1'b0}}; end
   always @(posedge clk) begin
      if (k_v) kd <= kd_n;
      kd_v <= ~reset & ~flush & ~rel & (kd_v | k_v);
   end
   // a later branch is older than the one walked to; nothing allocates while the walk runs
   wire [RB-1:0] k_rel  = k_idx[IDXB-1:CB] - hrow, kq_rel = k_row - hrow;
   always @(posedge clk) if (!reset) begin
      if (k_v & (wl_run | wl_end) & ~flush & ~({k_rel, k_idx[CB-1:0]} < {kq_rel, k_col}))
         $fatal(1, "smolrv64_rob: a walk to entry %0d after one to the older or same entry %0d", k_idx, {k_row, k_col});
      if (k_v & ~flush & (~v[k_idx] | empty))
         $fatal(1, "smolrv64_rob: a walk to entry %0d, which holds no live op", k_idx);
      if ((wl_run | k_v) & (|do_alloc))
         $fatal(1, "smolrv64_rob: an allocation while the walk runs");
      if (rel & ~(kd_v & wl_done))
         $fatal(1, "smolrv64_rob: a release before the walk reached its branch or with no kill");
      if (rel & (|do_alloc))
         $fatal(1, "smolrv64_rob: an allocation in the release's cycle");
   end

   // ---- the columns: one write (slot k) and two reads (the head row, the walk's row) each ----
   // c_kill on the head's column: the head is trapping. It must NOT commit -- a trap does not
   // write rd -- and the redirect that follows flushes it, which returns its allocation through
   // the free list's own pointer rollback. A column behind it retires with every one before it,
   // and never in a flush cycle: a mispredicted branch COMMITS and redirects in the same cycle,
   // and the entry behind it is the wrong path.
   generate for (gk = 0; gk < IW; gk = gk + 1) begin: col
      localparam [CB-1:0] K = gk;
      reg [EW-1:0] mem [0:ROWS-1];
      integer bi; initial for (bi = 0; bi < ROWS; bi = bi + 1) mem[bi] = {EW{1'b0}};
      always @(posedge clk) if (do_alloc[gk]) mem[trow] <= {d_pold[gk*PBITS +: PBITS], d_noret[gk], d_rd[gk*6 +: 6], d_prd[gk*PBITS +: PBITS]};
      wire [EW-1:0]   e  = mem[hrow];
      wire [EW-1:0]   we = mem[wl_row];           // the walk's read port
      wire [IDXB-1:0] wx = {wl_row, K};
      assign wl_v[gk] = wl_run & v[wx] & ({1'b0, K} < wl_lim) & ((wl_row != k_row) | (K > k_col))
                      & (we[PBITS-1:0] != {PBITS{1'b0}});
      assign wl_prd[gk*PBITS +: PBITS]  = we[PBITS-1:0];
      assign wl_rd[gk*6 +: 6]           = we[PBITS +: 6];
      assign wl_pold[gk*PBITS +: PBITS] = we[EW-1 -: PBITS];
      wire [IDXB-1:0] ix = {hrow, K};
      if (gk == 0) begin: h
         assign cv[0] = (hcol == K) & head_done & ~c_kill[0];
      end else begin: b
         assign cv[gk] = (hcol == K) ? (head_done & ~c_kill[gk])
                       : (hcol < K) & cv[gk-1] & v[ix] & done[ix] & ~c_kill[gk] & ~flush;
      end
      assign c_prd[gk*PBITS +: PBITS] = e[PBITS-1:0];
      assign c_rd[gk*6 +: 6]          = e[PBITS +: 6];
      assign c_noret[gk]              = e[PBITS+6];
      assign c_rd_v[gk]               = |e[PBITS-1:0];
   end endgenerate
   assign c_valid = cv;
   wire [IW-1:0] cvv = cv;                          // the same, read by the loops below

   // the head after this cycle's commits: past the last committing column, or the next row
   reg  [IDXB:0] head_n;
   integer       hk;
   always @* begin
      head_n = head;
      for (hk = 0; hk < IW; hk = hk + 1)
         if (cvv[hk]) head_n = next_pos({head[IDXB:CB], hk[CB-1:0]});
   end
   // the irrevocable pointer steps one entry and never falls behind the head
   wire [IDXB:0] irr_step = irr_done ? next_pos(irr) : irr;
   wire [IDXB:0] irr_lag  = head_n - irr_step;            // > 0 and < half the space: behind
   wire [IDXB:0] irr_n    = ((irr_lag != 0) & ~irr_lag[IDXB]) ? head_n : irr_step;
   // a flush empties the ROB at a row boundary: the head's row if nothing of it has committed
   wire [IDXB:0] flush_pos = (head_n[CB-1:0] == {CB{1'b0}}) ? head_n : {head_n[IDXB:CB] + 1'b1, {CB{1'b0}}};
   always @(posedge clk) if (!reset)
      for (ri = 1; ri < IW; ri = ri + 1)
         if (d_valid[ri] && !d_valid[ri-1])
            $fatal(1, "smolrv64_rob: allocation %0d without the one before it", ri);

   always @(posedge clk) begin
      if (reset) begin
         v <= {DEPTH{1'b0}}; done <= {DEPTH{1'b0}}; head <= 0; tail <= 0; irr <= 0;
      end else begin
         for (ri = 0; ri < IW; ri = ri + 1)
            if (do_alloc[ri]) begin v[{trow, ri[CB-1:0]}] <= 1'b1;  done[{trow, ri[CB-1:0]}] <= 1'b0; end
         if (|do_alloc) tail <= tail + ROW1;
         for (ri = 0; ri < NW; ri = ri + 1)
            if (w_v[ri]) done[w_ix[ri*IDXB +: IDXB]] <= 1'b1;
         for (ri = 0; ri < IW; ri = ri + 1)
            if (cvv[ri]) v[{hrow, ri[CB-1:0]}] <= 1'b0;
         head <= head_n;
         irr <= irr_n;
         // the release, before the flush (a flush in its cycle wins): the dead entries go and
         // the tail steps back to the row after the branch's
         if (rel) begin
            for (rj = 0; rj < DEPTH; rj = rj + 1) if (kd[rj]) begin v[rj] <= 1'b0;  done[rj] <= 1'b0; end
            tail <= {head[IDXB:CB] + {1'b0, kq_rel} + 1'b1, {CB{1'b0}}};
         end
         // A flush kills everything YOUNGER than the entry committing this cycle -- the
         // redirecting instruction is itself older and must still commit. Ordered after the
         // commit arm above so the head's own retirement stands.
         if (flush) begin
            v    <= {DEPTH{1'b0}};
            done <= {DEPTH{1'b0}};
            tail <= flush_pos;
            irr  <= flush_pos;
            head <= flush_pos;
         end
      end
   end
   // The pointer never lags the head and never passes the tail (docs/rtl-rules.md A1).
   always @(posedge clk) if (!reset) begin
      if ((irr - head) > (tail - head))
         $fatal(1, "smolrv64_rob: the irrevocable pointer left [head, tail]: head=%0d irr=%0d tail=%0d", head, irr, tail);
   end

   // ---- invariants (always on: docs/rtl-rules.md A1) --------------------------------
   always @(posedge clk) if (!reset) begin
      if (d_valid[0] & ~ready)
         $fatal(1, "smolrv64_rob: dispatch into a full ROB (head=%0d tail=%0d)", head, tail);
      if (h_fin & ~w_hits(hidx))
         $fatal(1, "smolrv64_rob: h_fin without a completion of the head (entry %0d)", hidx);
      for (ri = 0; ri < NW; ri = ri + 1) if (w_v[ri]) begin
         if (~v[w_ix[ri*IDXB +: IDXB]])
            $fatal(1, "smolrv64_rob: completion for entry %0d, which holds no live op",
                   w_ix[ri*IDXB +: IDXB]);
         if (done[w_ix[ri*IDXB +: IDXB]])
            $fatal(1, "smolrv64_rob: entry %0d completed twice", w_ix[ri*IDXB +: IDXB]);
         for (rj = 0; rj < NW; rj = rj + 1)
            if (rj > ri && w_v[rj] && (w_ix[rj*IDXB +: IDXB] == w_ix[ri*IDXB +: IDXB]))
               $fatal(1, "smolrv64_rob: two ports completing entry %0d in one cycle",
                      w_ix[ri*IDXB +: IDXB]);
      end
      if (~empty & ~v[hidx])
         $fatal(1, "smolrv64_rob: the head (%0d) holds no live op", head);
      if (tail[CB-1:0] != {CB{1'b0}})
         $fatal(1, "smolrv64_rob: the tail is not at a row's first column (%0d)", tail);
      // Occupancy can never exceed the array. Catches a lost commit or a double allocate at
      // the moment it happens rather than as a wedge thousands of cycles later.
      if (occ > ROWS_S)
         $fatal(1, "smolrv64_rob: occupancy %0d rows exceeds %0d", occ, ROWS);
   end
endmodule

`default_nettype wire
