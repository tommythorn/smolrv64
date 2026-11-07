`default_nettype none

// One scheduler for one class of unit. docs/Area-Efficient-Scalar-OoO.md 5 and 8.3, reduced
// to what is actually load-bearing: N pairs of (physical register, ready), a wakeup that
// compares each source against the writeback ports, and a pick among the ready ones.
//
// NO TAG COMPARES AGE, AND NO AGE IS ON THE PICK'S PATH. A chain of "is this older than the best
// so far" is depth linear in the window, and an is-oldest test of every candidate is N^2
// comparators; both were tried and both set the clock. The ordering that must be preserved --
// memory ops against each other, until there is disambiguation -- is a property of ONE class,
// so it belongs to that class's scheduler as a head pointer (O(1)).
//
// What age buys is the critical path: a lowest-index pick hands a freed low slot's young op
// priority over every older one, and an old op on the chain everything waits for can sit ready
// for many cycles. So the pick prefers ONE entry, the oldest live one (OLDEST): which entry that
// is comes from an order matrix written at dispatch (a new op is younger than every live one,
// flops, no compare) and is registered a cycle ahead, so the pick sees an index and a ready bit,
// and takes the lowest-index ready entry when that one is not ready. (GB5's 77 SimPoints at IW=4,
// 4645a122: -0.56% cycles, every subtest; the ideal oldest-ready pick -0.94%.)
//
// Split by class, each instance is small and their select trees are independent:
//   integer  ~12 entries, 2 sources
//   loads     ~8 entries, 2 sources (address, and store data)
//   FP        ~4 entries, 3 sources (FMA is the only thing that needs a third)
// which is also one scheduler per PRF shard -- the condition doc 7 names for the writeback
// arbiter to disappear.
module smolrv64_iq
  #(parameter NENT   = 12,
    parameter IDXB   = 4,             // $clog2(NENT)
    parameter NSRC   = 2,             // 3 only for the FP scheduler (FMA)
    parameter ROBB   = 4,
    parameter PBITS  = 9,
    parameter NWB    = 3,             // writeback ports watched, one per PRF shard
    // 1 when this scheduler's unit has FIXED one-cycle latency. Its entries are then woken
    // AT SELECT rather than at writeback: producer selected at N executes at N+1, its consumer is
    // woken at N+1 (the dependency matrix's column) and selected at N+2 (READY IS A REGISTER).
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
    parameter REGRDY = 0,
    // 1: the pick takes the oldest live entry when it is ready (see the header)
    parameter OLDEST = 0)
   (input  wire                  clk,
    input  wire                  reset,

    // ---- dispatch ----
    input  wire                  d_valid,
    output wire                  d_ready,
    input  wire [ROBB-1:0]       d_rob,
    input  wire [NSRC*PBITS-1:0] d_ps,
    input  wire [NSRC-1:0]       d_r,        // source already available
    input  wire [PBITS-1:0]      d_prd,      // destination, for wake-at-select
    input  wire                  d_long,     // its result comes later than the next cycle (a multiply): no wake at select
    input  wire                  d_hx,       // it hands itself on through a one-op register (hx_busy)
    input  wire                  d_ag,       // it generates an address (a load or store): ag_only
    output wire [IDXB-1:0]       d_ent,      // slot taken; index the payload with it

    // ---- wakeup ----
    input  wire [NWB-1:0]        wb_v,
    input  wire [NWB*PBITS-1:0]  wb_preg,

    // ---- issue ----
    input  wire                  unit_busy,  // one unit per scheduler, so one bit
    // A FLOP (or flops): the handoff register is taken, or will be by the op this scheduler issued
    // last. An entry with d_hx is no candidate while it is set; the rest issue as before.
    input  wire                  hx_busy,
    // A FLOP (or flops): only address generators (d_ag) are candidates -- the lane's write port is
    // owed to a waiting landing, and an address generator executes without it.
    input  wire                  ag_only,
    output wire                  iss_v,
    output wire [IDXB-1:0]       iss_ent,
    output wire [ROBB-1:0]       iss_rob,
    output wire [NENT*NSRC-1:0]  iss_hq,     // each entry's sources' wake matches, a cycle late (see hq)
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
    // the kill (smolrv64_rob kd_v, kh, kb): an entry whose op is younger than a mispredicted
    // branch dies
    input  wire                  kd_v,
    input  wire [ROBB-1:0]       kh,
    input  wire [ROBB-1:0]       kb,
    output wire [IDXB:0]         occupancy,
    output wire [IDXB:0]         free_n);    // entries free now (not valid, not held): the dispatch credit

   reg [NENT-1:0]        v;
`include "smolrv64_kill.vh"
   reg [ROBB-1:0]        e_rob [0:NENT-1];
   reg [NSRC*PBITS-1:0]  e_ps  [0:NENT-1];
   reg [NSRC-1:0]        e_r   [0:NENT-1];
   reg [PBITS-1:0]       e_prd [0:NENT-1];
   reg [NENT-1:0]        e_long;           // a later result: its dependents wake at its writeback
   reg [NENT-1:0]        e_hx;             // hands itself on (d_hx)
   reg [NENT-1:0]        e_ag;             // an address generator (d_ag)
   initial begin e_long = {NENT{1'b0}}; e_hx = {NENT{1'b0}}; e_ag = {NENT{1'b0}}; end

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
   assign free_n = nfree;
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

   // THE SELECT'S WAKE, FROM REGISTERS. An entry issued this cycle wakes its dependents through the
   // dependency matrix's column for it (below), and a dependent is ready in the next cycle -- so the
   // column is read then, from the registered issue (sq_v, sq_oh), as one more source of readiness
   // beside the writeback compares, and the column is cleared then. The pick's index never reaches
   // e_r or dep: a landing's wake -> ready -> pick -> the column -> e_r was one cycle's chain.
   reg              sq_v;
   reg  [NENT-1:0]  sq_oh;
   reg  [IDXB-1:0]  sq_ix;
   initial begin sq_v = 1'b0;  sq_oh = {NENT{1'b0}};  sq_ix = {IDXB{1'b0}}; end
   reg  [NENT-1:0]  dep [0:NENT*NSRC-1];
   wire [NENT*NSRC-1:0] swk;            // the source's producer issued last cycle

   // ---- ready, and the pick ----
   // READY IS A REGISTER: a source is ready in the cycle after its wake (a writeback's compare, the
   // dependency matrix's column) sets e_r, and the pick reads only flops -- the wake's compares and
   // the pick are a cycle each (the 2x clock's two-cycle scheduler; the wake's cycle is the one
   // the producer's result takes into its write register).
   // Flat, not inside the generate block: a procedural loop cannot index a generate
   // instance with a runtime variable, and the stall attribution below needs exactly that.
   wire [NENT*NSRC-1:0] shit;           // the source's register is written back this cycle
   wire [NENT-1:0]      rdy;
   genvar g, gs;
   generate
      for (g = 0; g < NENT; g = g + 1) begin : g_rdy
         for (gs = 0; gs < NSRC; gs = gs + 1) begin : g_src
            assign shit[g*NSRC + gs] = hit(e_ps[g][gs*PBITS +: PBITS]);
            assign swk[g*NSRC + gs]  = sq_v & |(dep[g*NSRC + gs] & sq_oh);
         end
         // In order: only the head is a candidate. No age compare, just a pointer match.
         // unit_busy is NOT here. It is the unit's completion this cycle -- for the memory
         // class, M's advance, which is the dTLB compare and the LSU's done -- and putting
         // it inside every entry's ready bit made it the ROOT of the priority select. It
         // qualifies only the RESULT below: the pick is a function of readiness alone, and
         // the late signal is one AND on the issue valid instead of NENT ANDs feeding a tree.
         // hx_busy and ag_only are flops, and e_hx, e_ag, e_r and v are.
         assign rdy[g] = v[g] & ~(e_hx[g] & hx_busy) & ~(ag_only & ~e_ag[g]) & (&e_r[g])
                       & ((INORDER == 0) | (g[IDXB-1:0] == qhead));
      end
   endgenerate

   // THE ORDER: od[i][j] says entry j holds an older op than entry i. A dispatch writes its entry's
   // row (every live entry is older) and clears its column (it is older than none); a freed entry's
   // bits go stale and v masks them. The oldest live entry is the one with no live older entry,
   // registered (o_q, o_ix) for the next cycle's pick, which reads only the registers.
   reg  [NENT-1:0]  od [0:NENT-1];
   reg  [NENT-1:0]  o_q;
   reg  [IDXB-1:0]  o_ix;
   reg  [NENT-1:0]  o_oh;
   reg  [IDXB-1:0]  o_n;
   reg  [IDXB:0]    o_cnt;                         // the oldest entries named (the invariant's)
   integer          oi;
   initial begin
      for (oi = 0; oi < NENT; oi = oi + 1) od[oi] = {NENT{1'b0}};
      o_q = {NENT{1'b0}};  o_ix = {IDXB{1'b0}};
   end
   always @* begin
      o_n = {IDXB{1'b0}};  o_cnt = {(IDXB+1){1'b0}};
      for (oi = 0; oi < NENT; oi = oi + 1) begin
         o_oh[oi] = (OLDEST != 0) & v[oi] & ~|(od[oi] & v);
         if (o_oh[oi]) o_n = oi[IDXB-1:0];
         o_cnt = o_cnt + {{IDXB{1'b0}}, o_oh[oi]};
      end
   end
   always @(posedge clk) begin
      o_q  <= reset ? {NENT{1'b0}} : o_oh;
      o_ix <= o_n;
   end
   // FIXED PRIORITY behind the oldest. Deterministic, and starvation is bounded by the ROB rather
   // than by the policy: if a low-priority entry never wins, its ROB slot never completes, commit is
   // in order, the ROB fills, dispatch stops, every other entry drains, and it is all that is left.
   // The in-order commit point is what makes a cheap pick safe.
   wire             o_rdy = |(rdy & o_q);          // last cycle's oldest live entry, ready now
   reg              sel_v;
   reg [IDXB-1:0]   sel;
   always @* begin
      sel_v = (|rdy) & ~unit_busy;
      sel   = {IDXB{1'b0}};
      for (k = NENT-1; k >= 0; k = k - 1) if (rdy[k]) sel = k[IDXB-1:0];
      if (o_rdy) sel = o_ix;
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
   // THE BYPASS IS A REGISTER. An entry picked this cycle executes the next, and a source whose
   // register is written back now is then in its producer's write register, not yet the file. hq
   // holds every entry's matches as they stood, so the op's operand select is a flop at its entry,
   // not a tag compare in its execute cycle.
   reg [NENT*NSRC-1:0] hq;
   initial hq = {(NENT*NSRC){1'b0}};
   always @(posedge clk) hq <= shit;
   assign iss_hq = hq;

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
   wire kd_new  = kdead(kd_v, kh, kb, d_rob);              // the kill takes the op dispatched now
   wire do_iss  = iss_v & iss_take;

   // WAKE-AT-SELECT AS A DEPENDENCY MATRIX (fixed-latency schedulers). dep[k*NSRC+q][j] says
   // entry k's source q is produced by entry j OF THIS SCHEDULER. Rows are written at
   // dispatch by comparing the new entry's sources against every live destination -- register
   // against register, off the select path -- and read the cycle after an issue as ONE COLUMN,
   // the issued entry's (swk above), which is cleared then: its slot's next occupant can only be
   // dispatched in that cycle, so no row names it before. Results from other units still arrive by
   // tag on wb_v/wb_preg. (Wong's matrix scheduler, applied to the intra-queue wake alone.) A
   // source can only be produced by an OLDER entry, so "producer live when the consumer
   // dispatched" is exact.
   wire             self_v  = (FIXEDL != 0) & do_iss;
   wire [NENT-1:0]  isel_oh = {{(NENT-1){1'b0}}, 1'b1} << isel;
   reg  [NENT-1:0]  drow [0:NSRC-1];                 // the dispatching entry's rows
   integer dj, dq;
   always @* begin
      for (dq = 0; dq < NSRC; dq = dq + 1)
         for (dj = 0; dj < NENT; dj = dj + 1)
            drow[dq][dj] = v[dj] & ~e_long[dj] & (e_prd[dj] == d_ps[dq*PBITS +: PBITS]);
   end
   initial for (dq = 0; dq < NENT*NSRC; dq = dq + 1) dep[dq] = {NENT{1'b0}};

   // Stall attribution: the lowest-indexed live entry that is not ready, and the first
   // source it is waiting on. Counters only.
   reg              b_v;
   reg [PBITS-1:0]  b_pr;
   always @* begin
      b_v = 1'b0; b_pr = {PBITS{1'b0}};
      for (k = NENT-1; k >= 0; k = k - 1)
         if (v[k] && !(&e_r[k])) begin
            b_v = 1'b1;
            b_pr = e_ps[k][0 +: PBITS];
            for (q = NSRC-1; q >= 0; q = q - 1)
               if (!e_r[k][q]) b_pr = e_ps[k][q*PBITS +: PBITS];
         end
   end
   assign blk_v  = b_v;
   assign blk_pr = b_pr;

   always @(posedge clk) begin
      if (reset) begin
         v <= {NENT{1'b0}};
         qhead <= {IDXB{1'b0}}; qtail <= {IDXB{1'b0}};
         sq_v <= 1'b0;
      end else begin
         sq_v  <= self_v & ~flush;
         sq_oh <= isel_oh;  sq_ix <= isel;
         for (k = 0; k < NENT; k = k + 1) if (v[k])
            for (q = 0; q < NSRC; q = q + 1)
               if (hit(e_ps[k][q*PBITS +: PBITS]) || swk[k*NSRC+q])
                  e_r[k][q] <= 1'b1;
         if (do_iss) begin
            v[isel] <= 1'b0;
            if (INORDER != 0) qhead <= (qhead == (NENT-1)) ? {IDXB{1'b0}} : qhead + 1'b1;
         end
         if (do_disp) begin
            od[fsel] <= v;
            for (k = 0; k < NENT; k = k + 1) od[k][fsel] <= 1'b0;
            if (INORDER != 0) qtail <= (qtail == (NENT-1)) ? {IDXB{1'b0}} : qtail + 1'b1;
            v[fsel]     <= 1'b1;
            e_rob[fsel] <= d_rob;
            e_ps[fsel]  <= d_ps;
            e_prd[fsel] <= d_prd;
            e_long[fsel] <= d_long;
            e_hx[fsel]   <= d_hx;
            e_ag[fsel]   <= d_ag;
            for (q = 0; q < NSRC; q = q + 1) begin
               dep[fsel*NSRC+q] <= drow[q];               // registers only; its producer issuing now wakes it next cycle (swk)
               e_r[fsel][q] <= d_r[q] | hit(d_ps[q*PBITS +: PBITS]);
            end
         end
         // The column of the entry issued LAST cycle, from the registers: read now (swk), and its
         // slot's next occupant is a stranger. A row written now has no bit for it (it was not live
         // when the row was compared).
         if (sq_v) for (k = 0; k < NENT*NSRC; k = k + 1) dep[k][sq_ix] <= 1'b0;
         // the kill: a dead entry's dependents are dead too (a source is an older op's), so
         // no live row keeps a bit for it. Each entry is tested by the op it holds after this edge:
         // the entry dispatched into now by its new op's ROB slot, not its previous occupant's
         // (rule D19: the FP queue fills from the F stage while a kill holds)
         for (k = 0; k < NENT; k = k + 1)
            if ((do_disp & (fsel == k[IDXB-1:0])) ? kd_new : kdead(kd_v, kh, kb, e_rob[k]))
               v[k] <= 1'b0;
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
                && (dep[ak*NSRC+aq][isel] != (~e_long[isel] & (e_prd[isel] == e_ps[ak][aq*PBITS +: PBITS]))))
               $fatal(1, "smolrv64_iq: dependency matrix disagrees with the tag compare (entry %0d src %0d, producer %0d)", ak, aq, isel);
   end
   // the order names at most one oldest live entry, and one whenever an entry is live
   always @(posedge clk) if (!reset && (OLDEST != 0) && ((o_cnt > 1) || ((o_cnt == 0) && (|v))))
      $fatal(1, "smolrv64_iq: the order names %0d oldest entries among live %b", o_cnt, v);
   always @(posedge clk) if (!reset) begin
      if ((REGRDY != 0) & (INORDER == 0) & d_ready_q & ~|freem)
         $fatal(1, "smolrv64_iq: the registered ready promised a free entry and there is none");
      if (d_valid & ~d_ready & ~flush)
         $fatal(1, "smolrv64_iq: dispatch into a full scheduler");
      if (kd_v & (INORDER != 0))
         $fatal(1, "smolrv64_iq: a kill in an in-order scheduler, whose ring cannot drop an entry");
      if (do_disp & v[fsel])
         $fatal(1, "smolrv64_iq: dispatch into occupied entry %0d", fsel);
      if (do_disp & held[fsel])
         $fatal(1, "smolrv64_iq: dispatch into the entry still held downstream (%0d)", fsel);
      if (iss_take & ~iss_v)
         $fatal(1, "smolrv64_iq: consumer took an issue that was not offered");
      if (do_iss & ~v[isel])
         $fatal(1, "smolrv64_iq: issued entry %0d holds nothing", isel);
      if (do_iss & unit_busy)
         $fatal(1, "smolrv64_iq: issued to a busy unit");
      if (do_iss & e_hx[isel] & hx_busy)
         $fatal(1, "smolrv64_iq: issued entry %0d into a taken handoff register", isel);
      if (do_iss & ag_only & ~e_ag[isel])
         $fatal(1, "smolrv64_iq: issued entry %0d, not an address generator, while only they may issue", isel);
      // isel is qhead by construction when INORDER; this is the check that the priority
      // loop agrees, i.e. that nothing but the head can ever be ready.
      if ((INORDER != 0) & do_iss & (sel != qhead))
         $fatal(1, "smolrv64_iq: in-order scheduler picked %0d, not the head %0d", sel, qhead);
   end
endmodule

`default_nettype wire
