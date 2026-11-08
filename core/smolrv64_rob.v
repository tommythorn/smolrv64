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
// ROWS: a dispatch group takes a row, its instruction in lane k in column k, so column k is
// written only by lane k and completed only by lane k's ops. The lanes rotate (smolrv64_frontend),
// so a row's instructions start at its FIRST COLUMN fc[row] and run on cyclically: the
// instruction at program position p in the row is in column (fc + p) % IW. An entry's TAG is
// {row, wrap, column}, wrap set when its column is below fc (the instruction is younger than its
// column suggests), so tags order as numbers in program order and every age compare outside --
// smolrv64_kill.vh's kdead, a distance from the head -- is the plain one; the storage index is
// {row, column}, the tag without its wrap bit. Inside, the pointers (head, tail, irr) are
// positions {ring wrap, row, p}, ordered as numbers; the tail is always p = 0. The head commits
// along its row from its first uncommitted position, IW ports in program order (port p is
// position p), so the commit and the walk read column (fc + p) % IW: a rotation of the row's
// records at the ports, from fc. A row may take several cycles to commit, and the head moves to
// the next row when the row's last valid position commits.
module smolrv64_rob
  #(parameter ROWS  = 32,
    parameter CAP   = ROWS,           // the rows it fills (at most ROWS)
    parameter IDXB  = 8,              // a tag: $clog2(ROWS) + 1 + CB, {row, wrap, column}
    parameter PBITS = 9,
    parameter IW    = 2,              // the group width: IW allocations and IW commits a cycle
    parameter NW    = 3,              // simultaneous completion ports
    // the completion ports the irrevocable pointer reads in their own cycle (the store
    // queue's): a store passes `irr` the cycle it completes and leaves the queue a cycle sooner
    parameter [NW-1:0] IRR_FWD = {NW{1'b0}},
    parameter CB    = (IW > 1) ? $clog2(IW) : 1)   // column bits
   (input  wire                   clk,
    input  wire                   reset,

    // ---- dispatch: a group, lane k's instruction at [k], at RENAME ----
    // the group's lanes are a cyclic run from d_fc: (d_fc + p) % IW for p below its size
    input  wire [IW-1:0]          d_valid,
    input  wire [CB-1:0]          d_fc,
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
    output wire [IW*IDXB-1:0]     d_idx,        // the entry each takes, its tag; ride it with the op

    // ---- completion: out of order, names its slot by the tag it was given ----
    input  wire [NW-1:0]          w_v,
    input  wire [NW*IDXB-1:0]     w_ix,
    // the head completes in a flush cycle (a mispredict at its squash, a redirecting system
    // op): it commits in that cycle, before the flush. Every other completion commits from
    // `done` a cycle later, which keeps the completion cones out of retire.
    input  wire                   h_fin,

    // ---- commit: position p of the head row on port p, in order, into smolrv64_rename ----
    // c_kill[p] for the head's position: the head is trapping (retire nothing, free nothing). For
    // a position behind it: that entry may not retire this cycle -- it retires only with every
    // one before it, and never while M still holds its op (a trap or redirect resolved in M
    // waits there for the head with done set; retiring it from behind the head retires the
    // wrong path).
    input  wire [IW-1:0]          c_kill,
    // a load was flagged for replay last cycle (a flop): nothing that completed commits this cycle
    // (a head finishing with its own redirect, h_fin, still does: a flagged load never does that)
    input  wire                   c_hold,
    output wire [IW-1:0]          c_valid,
    output wire [IW-1:0]          c_head,       // port p is the head's (one-hot while the ROB holds any)
    output wire [IW*IDXB-1:0]     c_idx,        // port p's entry, its tag
    output wire [IW*6-1:0]        c_rd,
    output wire [IW-1:0]          c_rd_v,       // derived: |c_prd
    output wire [IW*PBITS-1:0]    c_prd,
    output wire [IW*PBITS-1:0]    c_pold,       // the mapping it displaced: rename frees it
    output wire [IW-1:0]          c_noret,

    // ---- recovery ----
    // Younger-than-the-committing-entry dies. Pointer-only, to match the free list's own
    // rollback: there is nothing to walk because rename never wrote the free-list array.
    input  wire                   flush,
    output wire                   empty,
    output wire [IDXB-CB-1:0]     occ_n,        // rows allocated now: the dispatch credit
    // Which slot is oldest. An instruction that does anything beyond writing its own
    // register -- trap, redirect -- may act only when it IS this slot, or it would squash an
    // older op still in flight ahead of it.
    output wire [IDXB-1:0]        head_idx,
    output wire [IDXB-1:0]        head_n_idx,   // the head after this cycle's commits (a flush aside)
    // THE IRREVOCABLE POINT: the oldest entry that is not done. Every op that can restart the
    // machine waits for the ROB head and sets `done` only when it has completed there, so an
    // entry that is done can no longer restart, and everything older than the first not-done
    // entry is settled. A store is committable exactly when this pointer reaches it.
    output wire [IDXB-1:0]        irr_idx,
    output wire                   irr_v,        // ...and that entry exists
    // An entry that is DONE but must still replay (a load that read before an older store that
    // overlaps it): the pointer stops at it, so nothing younger -- a store -- becomes irrevocable
    // ahead of the replay that will squash it.
    input  wire                   rp_v,
    input  wire [IDXB-1:0]        rp_ix,
    // ...and a load was flagged last cycle, which rp_v does not name yet: the pointer holds this cycle
    input  wire                   rp_all,

    // ---- the walk: from the tail back to a mispredicted branch, a row a cycle ----
    // k_v names the branch (a later k_v names an older one: the walk goes on to it). Each cycle
    // the walk names the dead entries of one row that write a register, position p on [p], with
    // their rd, prd and displaced mapping, for smolrv64_rename to undo. wl_done: the walk has
    // reached its branch (until the flush).
    input  wire                   k_v,
    input  wire [IDXB-1:0]        k_idx,
    output wire [IW-1:0]          wl_v,
    output wire [IW*6-1:0]        wl_rd,
    output wire [IW*PBITS-1:0]    wl_prd, wl_pold,
    output wire                   wl_done,
    // THE KILL: every entry younger than the branch, one bit per storage index, from flops, for
    // every structure that holds an op to look its own up in (kd_v & kd[its storage index]). Valid
    // from the second cycle after k_v until the flush.
    output reg                    kd_v,
    output reg  [ROWS*(1 << CB)-1:0] kd,
    // ...and the same kill as a position, for a structure to test its own entry with
    // smolrv64_kill.vh's kdead(): kh, the head row's first tag when the walk began, and kb, the
    // branch's distance from it. kd stays for the per-entry clears (this ROB's, the core's).
    output reg  [IDXB-1:0]        kh,
    output reg  [IDXB-1:0]        kb,
    // THE RELEASE: the walk has reached the branch and every dead op is gone, so the dead entries
    // leave and the tail returns to the row after the branch's; dispatch resumes behind it
    input  wire                   rel);

   // {pold, noret, rd, prd}. rd_v is `|prd`; the destination SHARD is the top bits of prd; pold,
   // the mapping the op displaced, serves the walk's restore and, at commit, the free.
   localparam EW    = PBITS + 1 + 6 + PBITS;      // {pold, noret, rd, prd}
   localparam NCOL  = 1 << CB;                    // columns; those at IW and above are never valid
   localparam RB    = IDXB - 1 - CB;              // row bits
   localparam SB    = RB + CB;                    // a storage index: {row, column}
   localparam DEPTH = ROWS * NCOL;                // the storage
   localparam [RB:0] CAP_S = CAP[RB:0];           // sized, so the occupancy check cannot truncate
   localparam [SB:0] ROW1 = NCOL[SB:0];           // one row, as a position step
   initial if ((1 << RB) != ROWS) $fatal(1, "smolrv64_rob: IDXB=%0d is not $clog2(ROWS=%0d) + 1 + CB=%0d", IDXB, ROWS, CB);
   initial if (CAP > ROWS || CAP < 1) $fatal(1, "smolrv64_rob: CAP=%0d rows of %0d", CAP, ROWS);
   reg [DEPTH-1:0] v, done;                     // bulk-cleared on flush, so flops by necessity
   reg [SB:0]      head, tail, irr;             // positions {ring wrap, row, p}; head <= irr <= tail
   reg [CB-1:0]    fc [0:ROWS-1];               // each row's first column
   reg [CB-1:0]    hfc, ifc;                    // the head row's and irr's row's (flops: the commit ports
                                                // and the pointer's step rotate by them)
   integer         ri, rj;
   initial begin
      v = {DEPTH{1'b0}}; done = {DEPTH{1'b0}}; head = 0; tail = 0; irr = 0; hfc = 0; ifc = 0;
      for (ri = 0; ri < ROWS; ri = ri + 1) fc[ri] = {CB{1'b0}};
   end

   // a row's position p in storage: column (f + p) % IW, and its wrap bit; positions IW and above
   // are the never-valid columns themselves
   function automatic [CB-1:0] col_of(input [CB-1:0] f, input [CB-1:0] p);
      reg [CB:0] t;
      begin
         t = {1'b0, f} + {1'b0, p};
         col_of = (p >= IW) ? p : (t >= IW) ? t[CB-1:0] - IW[CB-1:0] : t[CB-1:0];
      end
   endfunction
   function automatic wrap_of(input [CB-1:0] f, input [CB-1:0] p);
      wrap_of = (p < IW) & (({1'b0, f} + {1'b0, p}) >= IW);
   endfunction
   function automatic [IDXB-1:0] tag_of(input [RB-1:0] r, input [CB-1:0] f, input [CB-1:0] p);
      tag_of = {r, wrap_of(f, p), col_of(f, p)};
   endfunction
   function automatic [SB-1:0] six(input [IDXB-1:0] t);     // a tag's storage index
      six = {t[IDXB-1:CB+1], t[CB-1:0]};
   endfunction
   function automatic [CB-1:0] pos_of(input [CB-1:0] f, input [CB:0] wc);   // a tag's {wrap, column}: its p
      pos_of = wc[CB] ? wc[CB-1:0] + IW[CB-1:0] - f : wc[CB-1:0] - f;
   endfunction

   wire [RB-1:0]   hrow = head[SB-1:CB];
   wire [CB-1:0]   hp   = head[CB-1:0];
   wire [RB-1:0]   trow = tail[SB-1:CB];
   wire [RB:0]     occ  = tail[SB:CB] - head[SB:CB];   // rows, the head's counting
   wire [SB-1:0]   hsx  = {hrow, col_of(hfc, hp)};      // the head's storage index
   assign empty    = (head == tail);
   assign occ_n    = occ;
   assign head_idx = tag_of(hrow, hfc, hp);
   wire ready = (occ < CAP_S);
   // port p reads port p-1: per-bit variables, so the chains are not loops to the simulator
   wire [IW-1:0]   cv /*verilator split_var*/;
   wire [IW-1:0]   do_alloc = d_valid & {IW{ready}};
   genvar gk;
   generate for (gk = 0; gk < IW; gk = gk + 1) begin: sl
      localparam [CB-1:0] K = gk;
      // the wrap bit: the slot's column lies before the group's first (the last column never does)
      if (gk == IW - 1) begin: last
         assign d_idx[gk*IDXB +: IDXB] = {trow, 1'b0, K};
      end else begin: mid
         assign d_idx[gk*IDXB +: IDXB] = {trow, K < d_fc, K};
      end
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
   // THE COMPLETIONS ARE REGISTERED (the 2x clock: a completion's cone -- a landing, the walker, a
   // lane's resolution -- ends at these flops, not at `done`): an entry's done is set in the cycle
   // after its completion, if it still holds that op (a kill or a flush in between frees the slot,
   // which may be allocated again). h_fin is not delayed: the head commits in its flush cycle.
   reg  [NW-1:0]      wq_v = {NW{1'b0}};
   reg  [NW*IDXB-1:0] wq_ix;
   always @(posedge clk) begin
      wq_v  <= (reset | flush) ? {NW{1'b0}} : w_v;
      wq_ix <= w_ix;
   end
   function automatic wq_hits;         // ...or its registered completion, a cycle on (an ordered redirect's)
      input [IDXB-1:0] ix;
      integer q;
      begin
         wq_hits = 1'b0;
         for (q = 0; q < NW; q = q + 1)
            if (wq_v[q] && (wq_ix[q*IDXB +: IDXB] == ix)) wq_hits = 1'b1;
      end
   endfunction
   wire head_done = v[hsx] & ((done[hsx] & ~c_hold) | h_fin);
   wire [RB-1:0] irow = irr[SB-1:CB];
   assign irr_idx = tag_of(irow, ifc, irr[CB-1:0]);
   assign irr_v   = (irr != tail);
   // A row per cycle: the pointer crosses every position of its row, from its own on, that is done
   // or having its done written on an IRR_FWD port (a position with no op is skipped), and stops at the first
   // that is not; past the row's last it goes to the next row.
   wire [NCOL-1:0]    ir_wait;                 // the position's op is not done
   genvar gic;
   generate for (gic = 0; gic < NCOL; gic = gic + 1) begin: irc
      localparam [CB-1:0] P = gic;
      wire [SB-1:0] ix = {irow, col_of(ifc, P)};
      reg           fw;
      integer       q;
      always @* begin
         fw = 1'b0;
         for (q = 0; q < NW; q = q + 1)
            if (IRR_FWD[q] && wq_v[q] && (six(wq_ix[q*IDXB +: IDXB]) == ix)) fw = 1'b1;
      end
      assign ir_wait[gic] = v[ix] & ((~done[ix] & ~fw) | (rp_v & (ix == six(rp_ix)))) & (gic >= irr[CB-1:0]);
   end endgenerate
   reg  [SB:0] irr_step;
   integer     ick;
   always @* begin
      irr_step = {irr[SB:CB] + 1'b1, {CB{1'b0}}};
      for (ick = NCOL - 1; ick >= 0; ick = ick - 1)
         if (ir_wait[ick]) irr_step = {irr[SB:CB], ick[CB-1:0]};
      if ((irr == tail) | rp_all) irr_step = irr;
   end
   // the position after one of the head row's: the next while it is valid, else the next row
   function automatic [SB:0] next_pos(input [SB:0] p);
      begin
         if ((p[CB-1:0] != NCOL - 1) && v[{hrow, col_of(hfc, p[CB-1:0] + 1'b1)}]) next_pos = p + 1'b1;
         else next_pos = {p[SB:CB] + 1'b1, {CB{1'b0}}};
      end
   endfunction

   // ---- the walk ----
   // The next row to walk (wl_row) and the positions of it still to walk (those below wl_lim);
   // the branch is {k_row, k_p}, and only its younger positions die in its own row. When the
   // walk reaches the branch's row it stops there with wl_lim = k_p + 1, so a later, older
   // branch walks on from exactly the entries not yet walked, the old branch included.
   reg            wl_run, wl_end;
   reg [RB-1:0]   wl_row;
   reg [CB-1:0]   wl_fc;                       // fc[wl_row], loaded with it (fc is written only at
                                               // allocation, and nothing allocates while the walk runs)
   reg [CB:0]     wl_lim;
   reg [RB-1:0]   k_row;
   reg [CB-1:0]   k_p;
   reg [CB:0]     k_wc;                        // the branch's {wrap, column} (the order check's)
   wire [RB-1:0]  kn_row = k_idx[IDXB-1:CB+1];
   wire [CB-1:0]  kn_fc  = fc[kn_row];
   initial begin wl_run = 1'b0; wl_end = 1'b0; wl_row = 0; wl_fc = 0; wl_lim = 0; k_row = 0; k_p = 0; k_wc = 0; end
   assign wl_done = wl_end & ~wl_run & ~k_v;      // not in a cycle that names a new, older branch
   localparam [CB:0] NCOL_S = NCOL;
   always @(posedge clk) begin
      if (reset | flush | rel) begin
         wl_run <= 1'b0;  wl_end <= 1'b0;
      end else begin
         if (wl_run) begin
            if (wl_row == k_row) begin wl_run <= 1'b0;  wl_end <= 1'b1;  wl_lim <= {1'b0, k_p} + 1'b1; end
            else begin wl_row <= wl_row - 1'b1;  wl_fc <= fc[wl_row - 1'b1];  wl_lim <= NCOL_S; end
         end
         if (k_v) begin
            k_row <= kn_row;  k_p <= pos_of(kn_fc, k_idx[CB:0]);  k_wc <= k_idx[CB:0];  wl_run <= 1'b1;
            if (~wl_run & ~wl_end) begin wl_row <= trow - 1'b1;  wl_fc <= fc[trow - 1'b1];  wl_lim <= NCOL_S; end
         end
      end
   end
   // the kill vector: a row after the branch's (in ring order from the head) dies whole, and in
   // the branch's own row the entries after it (a column's tag above the branch's); computed from
   // flops the cycle k_v arrives
   wire [RB-1:0] kb_rel = kn_row - hrow;
   reg  [DEPTH-1:0] kd_n;
   integer kr, kc;
   always @* for (kr = 0; kr < ROWS; kr = kr + 1)
      for (kc = 0; kc < NCOL; kc = kc + 1)
         kd_n[kr*NCOL + kc] = ((kr[RB-1:0] - hrow) > kb_rel)
                            | ((kr[RB-1:0] == kn_row) & ({kc[CB-1:0] < kn_fc, kc[CB-1:0]} > k_idx[CB:0]));
   initial begin kd_v = 1'b0; kd = {DEPTH{1'b0}}; kh = {IDXB{1'b0}}; kb = {IDXB{1'b0}}; end
   always @(posedge clk) begin
      if (k_v) begin kd <= kd_n;  kh <= {hrow, 1'b0, {CB{1'b0}}};  kb <= {kb_rel, k_idx[CB:0]}; end
      kd_v <= ~reset & ~flush & ~rel & (kd_v | k_v);
   end
   // the position form names exactly the vector's entries (kd_n's rows past the branch's from the
   // head, and its own row's later tags, are the entries farther from kh than kb)
   localparam integer ROBB = IDXB;
`include "smolrv64_kill.vh"
   integer kx;
   always @(posedge clk) if (!reset && kd_v)
      for (kx = 0; kx < DEPTH; kx = kx + 1)
         if (kd[kx] != kdead(1'b1, kh, kb, {kx[SB-1:CB], kx[CB-1:0] < fc[kx[SB-1:CB]], kx[CB-1:0]}))
            $fatal(1, "smolrv64_rob: the kill's position (kh %0d, kb %0d) and its vector disagree at entry %0d", kh, kb, kx);
   // a later branch is older than the one walked to; nothing allocates while the walk runs
   wire [RB-1:0] k_rel  = kn_row - hrow, kq_rel = k_row - hrow;
   always @(posedge clk) if (!reset) begin
      if (k_v & (wl_run | wl_end) & ~flush & ~({k_rel, k_idx[CB:0]} < {kq_rel, k_wc}))
         $fatal(1, "smolrv64_rob: a walk to entry %0d after one to the older or same entry %0d", k_idx, {k_row, k_wc});
      if (k_v & ~flush & (~v[six(k_idx)] | empty))
         $fatal(1, "smolrv64_rob: a walk to entry %0d, which holds no live op", k_idx);
      if (k_v & ~flush & (k_idx != tag_of(kn_row, kn_fc, pos_of(kn_fc, k_idx[CB:0]))))
         $fatal(1, "smolrv64_rob: the walk's branch %0d does not carry its row's wrap bit (first column %0d)", k_idx, kn_fc);
      if ((wl_run | k_v) & (|do_alloc))
         $fatal(1, "smolrv64_rob: an allocation while the walk runs");
      if (rel & ~(kd_v & wl_done))
         $fatal(1, "smolrv64_rob: a release before the walk reached its branch or with no kill");
      if (rel & (|do_alloc))
         $fatal(1, "smolrv64_rob: an allocation in the release's cycle");
   end

   // ---- the columns: one write (lane k) and two reads (the head row, the walk's row) each ----
   // The ports rotate the row's columns into program order: port p reads column (fc + p) % IW.
   // c_kill on the head's position: the head is trapping. It must NOT commit -- a trap does not
   // write rd -- and the redirect that follows flushes it, which returns its allocation through
   // the free list's own pointer rollback. A position behind it retires with every one before it,
   // and never in a flush cycle: a mispredicted branch COMMITS and redirects in the same cycle,
   // and the entry behind it is the wrong path.
   wire [EW-1:0] he [0:IW-1], we [0:IW-1];      // column k's entry in the head's row, the walk's row
   wire [CB-1:0] wfc = wl_fc;
   always @(posedge clk) if (!reset && wl_run && wl_fc != fc[wl_row])
      $fatal(1, "smolrv64_rob: the walk's row %0d starts at column %0d, its register says %0d", wl_row, fc[wl_row], wl_fc);
   generate for (gk = 0; gk < IW; gk = gk + 1) begin: col
      reg [EW-1:0] mem [0:ROWS-1];
      integer bi; initial for (bi = 0; bi < ROWS; bi = bi + 1) mem[bi] = {EW{1'b0}};
      always @(posedge clk) if (do_alloc[gk]) mem[trow] <= {d_pold[gk*PBITS +: PBITS], d_noret[gk], d_rd[gk*6 +: 6], d_prd[gk*PBITS +: PBITS]};
      assign he[gk] = mem[hrow];
      assign we[gk] = mem[wl_row];                 // the walk's read port
   end endgenerate
   generate for (gk = 0; gk < IW; gk = gk + 1) begin: port
      localparam [CB-1:0] P = gk;
      // the walk's position p
      wire [CB-1:0]   wc = col_of(wfc, P);
      wire [EW-1:0]   w  = we[wc];
      assign wl_v[gk] = wl_run & v[{wl_row, wc}] & ({1'b0, P} < wl_lim) & ((wl_row != k_row) | (P > k_p))
                      & (w[PBITS-1:0] != {PBITS{1'b0}});
      assign wl_prd[gk*PBITS +: PBITS]  = w[PBITS-1:0];
      assign wl_rd[gk*6 +: 6]           = w[PBITS +: 6];
      assign wl_pold[gk*PBITS +: PBITS] = w[EW-1 -: PBITS];
      // the head row's position p
      wire [CB-1:0]   hc = col_of(hfc, P);
      wire [EW-1:0]   e  = he[hc];
      wire [SB-1:0]   ix = {hrow, hc};
      if (gk == 0) begin: h
         assign cv[0] = (hp == P) & head_done & ~c_kill[0];
      end else begin: b
         assign cv[gk] = (hp == P) ? (head_done & ~c_kill[gk])
                       : (hp < P) & cv[gk-1] & v[ix] & done[ix] & ~c_kill[gk] & ~c_hold & ~flush;
      end
      assign c_head[gk]               = (hp == P);
      assign c_idx[gk*IDXB +: IDXB]   = tag_of(hrow, hfc, P);
      assign c_prd[gk*PBITS +: PBITS] = e[PBITS-1:0];
      assign c_pold[gk*PBITS +: PBITS] = e[EW-1 -: PBITS];
      assign c_rd[gk*6 +: 6]          = e[PBITS +: 6];
      assign c_noret[gk]              = e[PBITS+6];
      assign c_rd_v[gk]               = |e[PBITS-1:0];
   end endgenerate
   assign c_valid = cv;
   wire [IW-1:0] cvv = cv;                          // the same, read by the loops below

   // the head after this cycle's commits: past the last committing position, or the next row
   reg  [SB:0] head_n;
   integer     hk;
   always @* begin
      head_n = head;
      for (hk = 0; hk < IW; hk = hk + 1)
         if (cvv[hk]) head_n = next_pos({head[SB:CB], hk[CB-1:0]});
   end
   // the next row's first column, an allocation into it this cycle forwarded: the head's when it
   // moves there, its row's at p = 0
   wire [RB-1:0] nrow = hrow + 1'b1;
   wire [CB-1:0] nfc  = ((|do_alloc) & (trow == nrow)) ? d_fc : fc[nrow];
   wire          h_adv = (head_n[SB:CB] != head[SB:CB]);
   assign head_n_idx = h_adv ? {nrow, 1'b0, nfc} : tag_of(hrow, hfc, head_n[CB-1:0]);
   // the irrevocable pointer never falls behind the head
   wire [SB:0] irr_lag  = head_n - irr_step;            // > 0 and < half the space: behind
   wire [SB:0] irr_n    = ((irr_lag != 0) & ~irr_lag[SB]) ? head_n : irr_step;
   // a flush empties the ROB at a row boundary: the head's row if nothing of it has committed
   wire [SB:0] flush_pos = (head_n[CB-1:0] == {CB{1'b0}}) ? head_n : {head_n[SB:CB] + 1'b1, {CB{1'b0}}};
   // the head row's first column follows the head: the next row's when the head moves there, and
   // an allocation into the head's row (an empty ROB) sets it; irr's likewise, or the head's when
   // the pointer takes the head's position
   wire          f_adv = flush ? (flush_pos[SB:CB] != head[SB:CB]) : h_adv;
   wire [CB-1:0] hfc_n = f_adv ? nfc : ((|do_alloc) & (trow == hrow)) ? d_fc : hfc;
   wire [RB-1:0] inrow = irow + 1'b1;
   wire [CB-1:0] ifc_n = (flush | ((irr_lag != 0) & ~irr_lag[SB])) ? hfc_n
                       : (irr_step[SB:CB] != irr[SB:CB]) ? (((|do_alloc) & (trow == inrow)) ? d_fc : fc[inrow])
                       : ((|do_alloc) & (trow == irow)) ? d_fc : ifc;
   // a group's lanes are one cyclic run from its first
   reg           e_run;
   integer       ak;
   always @* begin
      e_run = (|d_valid) & ~d_valid[d_fc];
      for (ak = 1; ak < IW; ak = ak + 1)
         if (d_valid[col_of(d_fc, ak[CB-1:0])] & ~d_valid[col_of(d_fc, ak[CB-1:0] - 1'b1)]) e_run = 1'b1;
   end
   always @(posedge clk) if (!reset & e_run)
      $fatal(1, "smolrv64_rob: allocation %b is not a run of lanes from the first, %0d", d_valid, d_fc);

   always @(posedge clk) begin
      if (reset) begin
         v <= {DEPTH{1'b0}}; done <= {DEPTH{1'b0}}; head <= 0; tail <= 0; irr <= 0; hfc <= 0; ifc <= 0;
      end else begin
         for (ri = 0; ri < IW; ri = ri + 1)
            if (do_alloc[ri]) begin v[{trow, ri[CB-1:0]}] <= 1'b1;  done[{trow, ri[CB-1:0]}] <= 1'b0; end
         if (|do_alloc) begin tail <= tail + ROW1;  fc[trow] <= d_fc; end
         for (ri = 0; ri < NW; ri = ri + 1)
            if (wq_v[ri] & v[six(wq_ix[ri*IDXB +: IDXB])]) done[six(wq_ix[ri*IDXB +: IDXB])] <= 1'b1;
         for (ri = 0; ri < IW; ri = ri + 1)
            if (cvv[ri]) v[{hrow, col_of(hfc, ri[CB-1:0])}] <= 1'b0;
         head <= head_n;
         irr <= irr_n;
         hfc <= hfc_n;
         ifc <= ifc_n;
         // the release, before the flush (a flush in its cycle wins): the dead entries go and
         // the tail steps back to the row after the branch's
         if (rel) begin
            for (rj = 0; rj < DEPTH; rj = rj + 1) if (kd[rj]) v[rj] <= 1'b0;   // done is read under v, and cleared at allocation
            tail <= {head[SB:CB] + {1'b0, kq_rel} + 1'b1, {CB{1'b0}}};
         end
         // A flush kills everything YOUNGER than the entry committing this cycle -- the
         // redirecting instruction is itself older and must still commit. Ordered after the
         // commit arm above so the head's own retirement stands.
         if (flush) begin
            v    <= {DEPTH{1'b0}};
            tail <= flush_pos;
            irr  <= flush_pos;
            head <= flush_pos;
         end
         // a column at IW and above never holds an op: its bits are constants, so no completion
         // port decodes them and no read of a row waits on them
         for (rj = 0; rj < DEPTH; rj = rj + 1)
            if ((rj % NCOL) >= IW) begin v[rj] <= 1'b0;  done[rj] <= 1'b0; end
      end
   end
   // The pointer never lags the head and never passes the tail (docs/rtl-rules.md A1).
   always @(posedge clk) if (!reset) begin
      if ((irr - head) > (tail - head))
         $fatal(1, "smolrv64_rob: the irrevocable pointer left [head, tail]: head=%0d irr=%0d tail=%0d", head, irr, tail);
   end

   // ---- invariants (always on: docs/rtl-rules.md A1) --------------------------------
   always @(posedge clk) if (!reset) begin
      if ((|d_valid) & ~ready)
         $fatal(1, "smolrv64_rob: dispatch into a full ROB (head=%0d tail=%0d)", head, tail);
      if (h_fin & ~w_hits(head_idx) & ~wq_hits(head_idx))
         $fatal(1, "smolrv64_rob: h_fin without a completion of the head (entry %0d)", head_idx);
      for (ri = 0; ri < NW; ri = ri + 1) if (w_v[ri]) begin
         if (~v[six(w_ix[ri*IDXB +: IDXB])])
            $fatal(1, "smolrv64_rob: completion for entry %0d, which holds no live op",
                   w_ix[ri*IDXB +: IDXB]);
         if (done[six(w_ix[ri*IDXB +: IDXB])])
            $fatal(1, "smolrv64_rob: entry %0d completed twice", w_ix[ri*IDXB +: IDXB]);
         if (w_ix[ri*IDXB + CB] != (w_ix[ri*IDXB +: CB] < fc[w_ix[ri*IDXB + CB + 1 +: RB]]))
            $fatal(1, "smolrv64_rob: completion for entry %0d with the wrong wrap bit", w_ix[ri*IDXB +: IDXB]);
         for (rj = 0; rj < NW; rj = rj + 1) begin
            if (rj > ri && w_v[rj] && (w_ix[rj*IDXB +: IDXB] == w_ix[ri*IDXB +: IDXB]))
               $fatal(1, "smolrv64_rob: two ports completing entry %0d in one cycle",
                      w_ix[ri*IDXB +: IDXB]);
            if (wq_v[rj] && (wq_ix[rj*IDXB +: IDXB] == w_ix[ri*IDXB +: IDXB]))
               $fatal(1, "smolrv64_rob: entry %0d completed twice (a cycle apart)", w_ix[ri*IDXB +: IDXB]);
         end
      end
      if (~empty & ~v[hsx])
         $fatal(1, "smolrv64_rob: the head (%0d) holds no live op", head);
      if (~empty & (hfc != fc[hrow]))
         $fatal(1, "smolrv64_rob: the head row's first column %0d, the row's %0d", hfc, fc[hrow]);
      if (irr_v & (ifc != fc[irow]))
         $fatal(1, "smolrv64_rob: irr's row's first column %0d, the row's %0d", ifc, fc[irow]);
      if (tail[CB-1:0] != {CB{1'b0}})
         $fatal(1, "smolrv64_rob: the tail is not at a row's first position (%0d)", tail);
      // Occupancy can never exceed the array. Catches a lost commit or a double allocate at
      // the moment it happens rather than as a wedge thousands of cycles later.
      if (occ > CAP_S)
         $fatal(1, "smolrv64_rob: occupancy %0d rows exceeds %0d", occ, CAP);
   end
endmodule

`default_nettype wire
