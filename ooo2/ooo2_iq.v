`default_nettype none

// One scheduler for one class of unit. docs/Area-Efficient-Scalar-OoO.md 5 and 8.3, reduced
// to what is actually load-bearing: N pairs of (physical register, ready), a wakeup that
// compares each source against the writeback ports, and a pick among the ready ones.
//
// NOTHING HERE SCALES WITH N^2, AND NOTHING COMPARES AGE. An earlier version did both and
// both were mistakes. Age-ordered select was written as "is this older than the best so
// far", which synthesises to a CHAIN of N comparators -- depth linear in the window, and it
// was the critical path at 8.3ns. An is-oldest MATRIX then replaced it to enforce "ordered
// ops issue only when oldest", which fixed the depth and cost N^2 comparators instead.
//
// Neither is needed. Which ready entry goes first does not matter; what matters is how much
// independent work the window HOLDS. And the ordering that genuinely must be preserved --
// memory ops against each other, until there is disambiguation -- is a property of ONE
// class, so it belongs to that class's scheduler as a head pointer (O(1)), not to every
// scheduler as an age comparison.
//
// Split by class, each instance is small and their select trees are independent:
//   integer  ~12 entries, 2 sources
//   loads     ~8 entries, 2 sources (address, and store data)
//   FP        ~4 entries, 3 sources (FMA is the only thing that needs a third)
// which is also one scheduler per PRF shard -- the condition doc 7 names for the writeback
// arbiter to disappear.
module ooo2_iq
  #(parameter NENT   = 12,
    parameter IDXB   = 4,             // $clog2(NENT)
    parameter NSRC   = 2,             // 3 only for the FP scheduler (FMA)
    parameter ROBB   = 4,
    parameter PBITS  = 9,
    parameter NWB    = 3,             // writeback ports watched, one per PRF shard
    // 1 when this scheduler's unit has FIXED one-cycle latency. Its entries are then woken
    // AT SELECT rather than at writeback, which is what keeps dependents back to back once
    // select has its own stage: producer selected at N executes at N+1, consumer woken at N
    // is selected at N+1 and executes at N+2 -- consecutive execute cycles.
    parameter FIXEDL = 0,
    // 1 for a class that must keep PROGRAM ORDER among its own entries -- memory, until
    // there is disambiguation. Allocation becomes circular and only the head may issue, so
    // "oldest" is a pointer compare and not an age comparison. This is the whole reason no
    // scheduler needs age: the one ordering constraint that survives belongs to one class.
    parameter INORDER = 0,
    // 1: d_ready is a REGISTER -- "at least two entries were free last cycle", which
    // guarantees one free now (a cycle admits one dispatch, and the held entry only ever
    // moves). The live |freem sat at the root of the frontend's dispatch decision: a_ent
    // -> held -> freem -> d_ready -> iq_ready -> d_take -> rename -> u_pend, 16 levels.
    // The price is the last entry: the scheduler refuses dispatch at NENT-1 occupied.
    parameter REGRDY = 0)
   (input  wire                  clk,
    input  wire                  reset,

    // ---- dispatch ----
    input  wire                  d_valid,
    output wire                  d_ready,
    input  wire [ROBB-1:0]       d_rob,
    input  wire [NSRC*PBITS-1:0] d_ps,
    input  wire [NSRC-1:0]       d_r,        // source already available
    input  wire [PBITS-1:0]      d_prd,      // destination, for wake-at-select
    output wire [IDXB-1:0]       d_ent,      // slot taken; index the payload with it

    // ---- wakeup ----
    input  wire [NWB-1:0]        wb_v,
    input  wire [NWB*PBITS-1:0]  wb_preg,

    // ---- issue ----
    input  wire                  unit_busy,  // one unit per scheduler, so one bit
    output wire                  iss_v,
    output wire [IDXB-1:0]       iss_ent,
    output wire [ROBB-1:0]       iss_rob,
    input  wire                  iss_take,

    // The entry still held downstream. Its slot must not be reallocated: the payload array
    // is indexed by entry number and read a cycle AFTER selection, so handing the slot to a
    // new dispatch overwrites the payload of an instruction that has not executed yet.
    input  wire                  hold_v,
    input  wire [IDXB-1:0]       hold_ent,

    // Why the pick found nothing, for the stall counters. Never feeds selection.
    output wire                  blk_v,
    output wire [PBITS-1:0]      blk_pr,

    input  wire                  flush,
    output wire [IDXB:0]         occupancy);

   reg [NENT-1:0]        v;
   reg [ROBB-1:0]        e_rob [0:NENT-1];
   reg [NSRC*PBITS-1:0]  e_ps  [0:NENT-1];
   reg [NSRC-1:0]        e_r   [0:NENT-1];
   reg [PBITS-1:0]       e_prd [0:NENT-1];

   integer k, q;
   initial begin
      v = {NENT{1'b0}};
      for (k = 0; k < NENT; k = k + 1) begin
         e_rob[k] = {ROBB{1'b0}}; e_ps[k] = {(NSRC*PBITS){1'b0}};
         e_r[k]   = {NSRC{1'b0}}; e_prd[k] = {PBITS{1'b0}};
      end
   end

   // ---- slot allocation ----
   // Out of order: lowest free index. In order: a circular tail, so entry order IS program
   // order and the head is the oldest by construction.
   wire [NENT-1:0] held  = hold_v ? ({{(NENT-1){1'b0}}, 1'b1} << hold_ent) : {NENT{1'b0}};
   wire [NENT-1:0] freem = ~v & ~held;
   reg  [IDXB-1:0] qhead, qtail;
   initial begin qhead = {IDXB{1'b0}}; qtail = {IDXB{1'b0}}; end

   reg [IDXB-1:0] lowfree;
   always @* begin
      lowfree = {IDXB{1'b0}};
      for (k = NENT-1; k >= 0; k = k - 1) if (freem[k]) lowfree = k[IDXB-1:0];
   end
   wire [IDXB-1:0] fsel    = (INORDER != 0) ? qtail : lowfree;
   reg [IDXB:0] nfree;
   always @* begin
      nfree = {(IDXB+1){1'b0}};
      for (k = 0; k < NENT; k = k + 1) nfree = nfree + {{IDXB{1'b0}}, freem[k]};
   end
   reg d_ready_q;  initial d_ready_q = 1'b0;
   always @(posedge clk) d_ready_q <= ~reset & (nfree >= 2);
   assign          d_ready = (INORDER != 0) ? (~v[qtail] & ~held[qtail])
                           : (REGRDY != 0) ? d_ready_q : |freem;

   // ---- wakeup: NSRC * NWB comparators per entry. Linear in N. ----
   function automatic hit;
      input [PBITS-1:0] p;
      integer w;
      begin
         hit = 1'b0;
         for (w = 0; w < NWB; w = w + 1)
            if (wb_v[w] && (wb_preg[w*PBITS +: PBITS] == p)) hit = 1'b1;
      end
   endfunction

   // ---- ready, and the pick ----
   // Flat, not inside the generate block: a procedural loop cannot index a generate
   // instance with a runtime variable, and the stall attribution below needs exactly that.
   wire [NENT*NSRC-1:0] srdy;
   wire [NENT-1:0]      rdy;
   genvar g, gs;
   generate
      for (g = 0; g < NENT; g = g + 1) begin : g_rdy
         for (gs = 0; gs < NSRC; gs = gs + 1) begin : g_src
            assign srdy[g*NSRC + gs] = e_r[g][gs] | hit(e_ps[g][gs*PBITS +: PBITS]);
         end
         // In order: only the head is a candidate. No age compare, just a pointer match.
         // unit_busy is NOT here. It is the unit's completion this cycle -- for the memory
         // class, M's advance, which is the dTLB compare and the LSU's done -- and putting
         // it inside every entry's ready bit made it the ROOT of the priority select. It
         // qualifies only the RESULT below: the pick is a function of readiness alone, and
         // the late signal is one AND on the issue valid instead of NENT ANDs feeding a tree.
         assign rdy[g] = v[g] & (&srdy[g*NSRC +: NSRC])
                       & ((INORDER == 0) | (g[IDXB-1:0] == qhead));
      end
   endgenerate

   // FIXED PRIORITY. Deterministic, and starvation is bounded by the ROB rather than by the
   // policy: if a low-priority entry never wins, its ROB slot never completes, commit is in
   // order, the ROB fills, dispatch stops, every other entry drains, and it is all that is
   // left. The in-order commit point is what makes a cheap pick safe.
   reg              sel_v;
   reg [IDXB-1:0]   sel;
   always @* begin
      sel_v = (|rdy) & ~unit_busy;
      sel   = {IDXB{1'b0}};
      for (k = NENT-1; k >= 0; k = k - 1) if (rdy[k]) sel = k[IDXB-1:0];
   end
   // THE IN-ORDER QUEUE'S ISSUE INDEX IS ITS HEAD POINTER, A REGISTER. rdy is nonzero only
   // at qhead when INORDER, so `sel` equals qhead whenever iss_v -- but written as the
   // priority loop it is a function of the wakeup compares, and the core reads the source
   // tags at this index (rule I6: a late signal never reaches a RAM address). Out of order
   // the pick is genuinely combinational, and only that class pays for it.
   wire [IDXB-1:0]  isel = (INORDER != 0) ? qhead : sel;

   assign d_ent   = fsel;
   assign iss_v   = sel_v;
   assign iss_ent = isel;
   assign iss_rob = e_rob[isel];

   reg [IDXB:0] occ;
   always @* begin
      occ = {(IDXB+1){1'b0}};
      for (k = 0; k < NENT; k = k + 1) occ = occ + {{IDXB{1'b0}}, v[k]};
   end
   assign occupancy = occ;

   // Neither gated on the flush: a dispatch or an issue in the flush cycle is wrong-path and
   // the flush arm, ordered last below, clears it. Gating them put M's completion (which a
   // landing load decides) in front of every entry write (gate V3, 2026-09-05).
   wire do_disp = d_valid & d_ready;
   wire do_iss  = iss_v & iss_take;

   // WAKE-AT-SELECT AS A DEPENDENCY MATRIX (fixed-latency schedulers). dep[k*NSRC+q][j] says
   // entry k's source q is produced by entry j OF THIS SCHEDULER. Rows are written at
   // dispatch by comparing the new entry's sources against every live destination -- register
   // against register, off the select path -- and read at select as ONE COLUMN. The previous
   // form read e_prd at the selected index (a LUTRAM read the select decides, fan-out 248) and
   // compared that tag against every source: sel -> RADR -> self_pr -> compare -> e_r was the
   // tail of every path into e_r. Results from other units still arrive by tag on wb_v/wb_preg.
   // (Wong's matrix scheduler, applied to the intra-queue wake alone.) A source can only be
   // produced by an OLDER entry, so "producer live when the consumer dispatched" is exact.
   wire             self_v  = (FIXEDL != 0) & do_iss;
   wire [NENT-1:0]  isel_oh = {{(NENT-1){1'b0}}, 1'b1} << isel;
   reg  [NENT-1:0]  dep [0:NENT*NSRC-1];
   reg  [NENT-1:0]  drow [0:NSRC-1];                 // the dispatching entry's rows
   integer dj, dq;
   always @* begin
      for (dq = 0; dq < NSRC; dq = dq + 1)
         for (dj = 0; dj < NENT; dj = dj + 1)
            drow[dq][dj] = v[dj] & (e_prd[dj] == d_ps[dq*PBITS +: PBITS]);
   end
   initial for (dq = 0; dq < NENT*NSRC; dq = dq + 1) dep[dq] = {NENT{1'b0}};

   // Stall attribution: the lowest-indexed live entry that is not ready, and the first
   // source it is waiting on. Counters only.
   reg              b_v;
   reg [PBITS-1:0]  b_pr;
   always @* begin
      b_v = 1'b0; b_pr = {PBITS{1'b0}};
      for (k = NENT-1; k >= 0; k = k - 1)
         if (v[k] && !(&srdy[k*NSRC +: NSRC])) begin
            b_v = 1'b1;
            b_pr = e_ps[k][0 +: PBITS];
            for (q = NSRC-1; q >= 0; q = q - 1)
               if (!srdy[k*NSRC + q]) b_pr = e_ps[k][q*PBITS +: PBITS];
         end
   end
   assign blk_v  = b_v;
   assign blk_pr = b_pr;

   always @(posedge clk) begin
      if (reset) begin
         v <= {NENT{1'b0}};
         qhead <= {IDXB{1'b0}}; qtail <= {IDXB{1'b0}};
      end else begin
         for (k = 0; k < NENT; k = k + 1) if (v[k])
            for (q = 0; q < NSRC; q = q + 1)
               if (hit(e_ps[k][q*PBITS +: PBITS])
                   || (self_v && |(dep[k*NSRC+q] & isel_oh)))
                  e_r[k][q] <= 1'b1;
         if (do_iss) begin
            v[isel] <= 1'b0;
            if (INORDER != 0) qhead <= (qhead == (NENT-1)) ? {IDXB{1'b0}} : qhead + 1'b1;
         end
         if (do_disp) begin
            if (INORDER != 0) qtail <= (qtail == (NENT-1)) ? {IDXB{1'b0}} : qtail + 1'b1;
            v[fsel]     <= 1'b1;
            e_rob[fsel] <= d_rob;
            e_ps[fsel]  <= d_ps;
            e_prd[fsel] <= d_prd;
            for (q = 0; q < NSRC; q = q + 1) begin
               dep[fsel*NSRC+q] <= drow[q];               // registers only; the column clear below wins over it
               e_r[fsel][q] <= d_r[q] | hit(d_ps[q*PBITS +: PBITS])
                             | (self_v && |(drow[q] & isel_oh));
            end
         end
         // The column of the issuing entry, AFTER the row write so it wins for the row that
         // was written this cycle too: its slot's next occupant is a stranger. Ordered here,
         // not inside the do_iss arm, so the row write's D input is the register compare
         // alone -- masking it by the select put the load-landing cone (D$ response tag ->
         // wakeup -> select) on 359 dep endpoints.
         if (do_iss) for (k = 0; k < NENT*NSRC; k = k + 1) dep[k][isel] <= 1'b0;
         if (flush) begin                                  // last: wins over the dispatch above
            v <= {NENT{1'b0}};
            qhead <= {IDXB{1'b0}}; qtail <= {IDXB{1'b0}};
         end
      end
   end

   // ---- invariants (always on: docs/rtl-rules.md A1) ---------------------------------
   integer ak, aq;
   always @(posedge clk) if (!reset && self_v) begin
      for (ak = 0; ak < NENT; ak = ak + 1) if (v[ak] && (ak != isel))
         for (aq = 0; aq < NSRC; aq = aq + 1)
            if ((e_ps[ak][aq*PBITS +: PBITS] != {PBITS{1'b0}})
                && (dep[ak*NSRC+aq][isel] != (e_prd[isel] == e_ps[ak][aq*PBITS +: PBITS])))
               $fatal(1, "ooo2_iq: dependency matrix disagrees with the tag compare (entry %0d src %0d, producer %0d)", ak, aq, isel);
   end
   always @(posedge clk) if (!reset) begin
      if ((REGRDY != 0) & (INORDER == 0) & d_ready_q & ~|freem)
         $fatal(1, "ooo2_iq: the registered ready promised a free entry and there is none");
      if (d_valid & ~d_ready & ~flush)
         $fatal(1, "ooo2_iq: dispatch into a full scheduler");
      if (do_disp & v[fsel])
         $fatal(1, "ooo2_iq: dispatch into occupied entry %0d", fsel);
      if (do_disp & held[fsel])
         $fatal(1, "ooo2_iq: dispatch into the entry still held downstream (%0d)", fsel);
      if (iss_take & ~iss_v)
         $fatal(1, "ooo2_iq: consumer took an issue that was not offered");
      if (do_iss & ~v[isel])
         $fatal(1, "ooo2_iq: issued entry %0d holds nothing", isel);
      if (do_iss & unit_busy)
         $fatal(1, "ooo2_iq: issued to a busy unit");
      // isel is qhead by construction when INORDER; this is the check that the priority
      // loop agrees, i.e. that nothing but the head can ever be ready.
      if ((INORDER != 0) & do_iss & (sel != qhead))
         $fatal(1, "ooo2_iq: in-order scheduler picked %0d, not the head %0d", sel, qhead);
   end
endmodule

`default_nettype wire
