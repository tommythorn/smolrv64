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
module smolrv64_rob
  #(parameter DEPTH = 16,
    parameter IDXB  = 4,              // $clog2(DEPTH)
    parameter PBITS = 9,
    parameter IW    = 2,              // the group width: IW allocations and IW commits a cycle
    parameter NW    = 3,              // simultaneous completion ports
    // the completion ports the irrevocable pointer reads in their own cycle (the store
    // queue's): a store passes `irr` the cycle it completes and leaves the queue a cycle sooner
    parameter [NW-1:0] IRR_FWD = {NW{1'b0}})
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
    output wire [IW-1:0]          d_ready,      // [k]: room for k+1
    output wire [IW*IDXB-1:0]     d_idx,        // the slot each takes; ride it with the op

    // ---- completion: out of order, names its slot by the tag it was given ----
    input  wire [NW-1:0]          w_v,
    input  wire [NW*IDXB-1:0]     w_ix,
    // the head completes in a flush cycle (a mispredict at its squash, a redirecting system
    // op): it commits in that cycle, before the flush. Every other completion commits from
    // `done` a cycle later, which keeps the completion cones out of retire.
    input  wire                   h_fin,

    // ---- commit: the head and the entries behind it, in order, into smolrv64_rename ----
    // c_kill[0]: the head is trapping (retire nothing, free nothing). c_kill[k], k > 0: entry k
    // may not retire this cycle -- an entry behind the head retires only with every one before
    // it, and never while M still holds its op (a trap or redirect resolved in M waits there for
    // the head with done set; retiring it from behind the head retires the wrong path).
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
    output wire [IDXB:0]          occ_n,        // entries allocated now: the dispatch credit
    // Which slot is oldest. An instruction that does anything beyond writing its own
    // register -- trap, redirect -- may act only when it IS this slot, or it would squash an
    // older op still in flight ahead of it.
    output wire [IDXB-1:0]        head_idx,
    // THE IRREVOCABLE POINT: the oldest entry that is not done. Every op that can restart the
    // machine waits for the ROB head and sets `done` only when it has completed there, so an
    // entry that is done can no longer restart, and everything older than the first not-done
    // entry is settled. A store is committable exactly when this pointer reaches it.
    output wire [IDXB-1:0]        irr_idx,
    output wire                   irr_v);       // ...and that entry exists

   // {noret, rd, prd} and nothing else. rd_v is `|prd`; the destination SHARD is the top bits
   // of prd; and the DISPLACED register is not carried at all, because smolrv64_rename reads
   // rmap[c_rd] at commit and that still holds it.
   localparam EW = 1 + 6 + PBITS;                  // {noret, rd, prd}
   localparam [IDXB:0] DEPTH_S = DEPTH[IDXB:0];   // sized, so the occupancy check cannot truncate

   // N-BANK ENTRY STORE: NBANKS = next_pow2(IW), so the <= IW consecutive allocations always
   // land in DISTINCT banks (bank = pos[LB-1:0], index = pos[IDXB-1:LB]) -- ONE muxed
   // {we,addr,data} per bank (one write statement per LUTRAM bank, else Vivado demotes it).
   localparam integer NBANKS = 1 << $clog2(IW);
   localparam integer LB     = $clog2(NBANKS);
   localparam integer BD     = DEPTH / NBANKS;
   localparam integer CB     = $clog2(IW + 1);    // a count of 0..IW
   wire [EW-1:0] bank_brd [0:NBANKS-1];            // each bank's entry at the head-relative index
   reg [DEPTH-1:0] v, done;                     // bulk-cleared on flush, so flops by necessity
   reg [IDXB:0]    head, tail;                  // one extra MSB: full and empty differ by it
   reg [IDXB:0]    irr;                         // head <= irr <= tail, same width
   integer         ri, rj;
   initial begin
      v = {DEPTH{1'b0}}; done = {DEPTH{1'b0}}; head = 0; tail = 0; irr = 0;
   end

   wire [IDXB-1:0] hidx = head[IDXB-1:0];
   wire [IDXB-1:0] tidx = tail[IDXB-1:0];
   wire [IDXB:0]   occ  = tail - head;
   assign empty    = (head == tail);
   assign occ_n    = occ;
   assign head_idx = hidx;
   // the group's slots (tail + k) and the entries behind the head (head + k)
   wire [IDXB-1:0] ti [0:IW-1];
   wire [IDXB-1:0] hi [0:IW-1];
   // slot k reads slot k-1: per-bit variables, so the chains are not loops to the simulator
   wire [IW-1:0]   do_alloc /*verilator split_var*/, cv /*verilator split_var*/, do_commit;
   wire [IW-1:0]   alloc_q = do_alloc;        // the same, read by the loops below
   genvar gk;
   generate for (gk = 0; gk < IW; gk = gk + 1) begin: sl
      localparam [IDXB:0] K1 = gk + 1;
      assign ti[gk] = tidx + gk[IDXB-1:0];
      assign hi[gk] = hidx + gk[IDXB-1:0];
      assign d_idx[gk*IDXB +: IDXB] = ti[gk];
      assign d_ready[gk] = (occ <= DEPTH_S - K1);
      // in a flush cycle too: the flush arm below wins
      assign do_alloc[gk] = d_valid[gk] & d_ready[gk] & ((gk == 0) ? 1'b1 : do_alloc[(gk == 0) ? 0 : gk - 1]);
   end endgenerate

   // ---- N-bank entry storage: one muxed write per bank, one read per head position ----
   genvar gb;
   generate for (gb = 0; gb < NBANKS; gb = gb + 1) begin: bank
      reg [EW-1:0] mem [0:BD-1];
      integer bi; initial for (bi = 0; bi < BD; bi = bi + 1) mem[bi] = {EW{1'b0}};
      // write: whichever of the (<= IW) allocations targets this bank -- the positions are
      // distinct mod NBANKS, so at most one does, hence a single write statement
      reg              wen;
      reg [IDXB-1:0]   wpos;
      reg [EW-1:0]     wd;
      integer j;
      always @* begin
         wen = 1'b0;  wpos = tidx;  wd = {d_noret[0], d_rd[0 +: 6], d_prd[0 +: PBITS]};
         for (j = IW - 1; j >= 0; j = j - 1)
            if (alloc_q[j] & (ti[j][LB-1:0] == gb[LB-1:0])) begin
               wen = 1'b1;  wpos = ti[j];  wd = {d_noret[j], d_rd[j*6 +: 6], d_prd[j*PBITS +: PBITS]};
            end
      end
      always @(posedge clk) if (wen) mem[wpos[IDXB-1:LB]] <= wd;
      // read: the head-relative position in this bank (head, head+1, ...)
      wire [LB-1:0]   roff = gb[LB-1:0] - hidx[LB-1:0];
      wire [IDXB-1:0] rpos = hidx + {{(IDXB-LB){1'b0}}, roff};
      assign bank_brd[gb] = mem[rpos[IDXB-1:LB]];
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
   wire irr_done  = (irr != tail) & v[iidx] & (done[iidx] | irr_hits(iidx));

   // c_kill[0]: the head is trapping. It must NOT commit -- a trap does not write rd -- and the
   // redirect that follows flushes it, which returns its allocation through the free list's
   // own pointer rollback. An entry behind the head retires with every one before it, and never
   // in a flush cycle: a mispredicted branch COMMITS and redirects in the same cycle, and the
   // entry behind it is the wrong path.
   generate for (gk = 0; gk < IW; gk = gk + 1) begin: cm
      wire [EW-1:0] e = bank_brd[hi[gk][LB-1:0]];
      if (gk == 0) begin: h
         assign cv[0] = head_done & ~c_kill[0];
      end else begin: b
         assign cv[gk] = cv[gk-1] & v[hi[gk]] & done[hi[gk]] & ~c_kill[gk] & ~flush;
      end
      assign c_prd[gk*PBITS +: PBITS] = e[PBITS-1:0];
      assign c_rd[gk*6 +: 6]          = e[PBITS +: 6];
      assign c_noret[gk]              = e[PBITS+6];
      assign c_rd_v[gk]               = |e[PBITS-1:0];
   end endgenerate
   assign c_valid = cv;
   assign do_commit = cv;

   function automatic [CB-1:0] cnt(input [IW-1:0] x);
      integer i;
      begin cnt = {CB{1'b0}}; for (i = 0; i < IW; i = i + 1) cnt = cnt + {{(CB-1){1'b0}}, x[i]}; end
   endfunction
   wire [IDXB:0] head_n = head + {{(IDXB+1-CB){1'b0}}, cnt(do_commit)};
   // the irrevocable pointer never falls behind the head: entries retiring together are all
   // done, so the pointer is at least past them
   wire [IDXB:0] irr_step = irr_done ? irr + 1'b1 : irr;
   reg  [IDXB:0] irr_n;
   integer       ik, im;
   always @* begin
      irr_n = irr_step;
      for (ik = 1; ik < IW; ik = ik + 1)
         if (do_commit[ik])
            for (im = 0; im <= ik; im = im + 1)
               if (irr_step == head + im[IDXB:0]) irr_n = head + ik[IDXB:0] + 1'b1;
   end
   always @(posedge clk) if (!reset)
      for (ri = 1; ri < IW; ri = ri + 1)
         if (d_valid[ri] && !d_valid[ri-1])
            $fatal(1, "smolrv64_rob: allocation %0d without the one before it", ri);

   always @(posedge clk) begin
      if (reset) begin
         v <= {DEPTH{1'b0}}; done <= {DEPTH{1'b0}}; head <= 0; tail <= 0;
      end else begin
         for (ri = 0; ri < IW; ri = ri + 1)
            if (alloc_q[ri]) begin v[ti[ri]] <= 1'b1;  done[ti[ri]] <= 1'b0; end
         tail <= tail + {{(IDXB+1-CB){1'b0}}, cnt(alloc_q)};
         for (ri = 0; ri < NW; ri = ri + 1)
            if (w_v[ri]) done[w_ix[ri*IDXB +: IDXB]] <= 1'b1;
         for (ri = 0; ri < IW; ri = ri + 1)
            if (do_commit[ri]) v[hi[ri]] <= 1'b0;
         head <= head_n;
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
         $fatal(1, "smolrv64_rob: the irrevocable pointer left [head, tail]: head=%0d irr=%0d tail=%0d", head, irr, tail);
   end

   // ---- invariants (always on: docs/rtl-rules.md A1) --------------------------------
   always @(posedge clk) if (!reset) begin
      if (d_valid[0] & ~d_ready[0])
         $fatal(1, "smolrv64_rob: dispatch into a full ROB (head=%0d tail=%0d)", head, tail);
      if (h_fin & ~w_hits(hidx))
         $fatal(1, "smolrv64_rob: h_fin without a completion of the head (slot %0d)", hidx);
      for (ri = 0; ri < NW; ri = ri + 1) if (w_v[ri]) begin
         if (~v[w_ix[ri*IDXB +: IDXB]])
            $fatal(1, "smolrv64_rob: completion for slot %0d, which holds no live entry",
                   w_ix[ri*IDXB +: IDXB]);
         if (done[w_ix[ri*IDXB +: IDXB]])
            $fatal(1, "smolrv64_rob: slot %0d completed twice", w_ix[ri*IDXB +: IDXB]);
         for (rj = 0; rj < NW; rj = rj + 1)
            if (rj > ri && w_v[rj] && (w_ix[rj*IDXB +: IDXB] == w_ix[ri*IDXB +: IDXB]))
               $fatal(1, "smolrv64_rob: two ports completing slot %0d in one cycle",
                      w_ix[ri*IDXB +: IDXB]);
      end
      if (c_valid[0] & ~v[hidx])
         $fatal(1, "smolrv64_rob: committing an invalid head (head=%0d)", head);
      // Occupancy can never exceed the array. Catches a lost commit or a double allocate at
      // the moment it happens rather than as a wedge thousands of cycles later.
      if ((tail - head) > DEPTH_S)
         $fatal(1, "smolrv64_rob: occupancy %0d exceeds DEPTH %0d", tail - head, DEPTH);
   end
endmodule

`default_nettype wire
