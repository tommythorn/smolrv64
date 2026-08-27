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
module ooo2_rs
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
    parameter INORDER = 0)
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
    output wire [NSRC*PBITS-1:0] iss_ps,
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
   assign          d_ready = (INORDER != 0) ? (~v[qtail] & ~held[qtail]) : |freem;

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
         assign rdy[g] = v[g] & (&srdy[g*NSRC +: NSRC]) & ~unit_busy
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
      sel_v = |rdy;
      sel   = {IDXB{1'b0}};
      for (k = NENT-1; k >= 0; k = k - 1) if (rdy[k]) sel = k[IDXB-1:0];
   end

   assign d_ent   = fsel;
   assign iss_v   = sel_v;
   assign iss_ent = sel;
   assign iss_rob = e_rob[sel];
   assign iss_ps  = e_ps[sel];

   reg [IDXB:0] occ;
   always @* begin
      occ = {(IDXB+1){1'b0}};
      for (k = 0; k < NENT; k = k + 1) occ = occ + {{IDXB{1'b0}}, v[k]};
   end
   assign occupancy = occ;

   wire do_disp = d_valid & d_ready & ~flush;
   wire do_iss  = iss_v & iss_take & ~flush;

   // Wake-at-select, for a fixed-latency unit only. Internal, and it only ever writes
   // registered ready bits -- so selection never feeds back into readiness.
   wire             self_v  = (FIXEDL != 0) & do_iss;
   wire [PBITS-1:0] self_pr = e_prd[sel];

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
      if (reset | flush) begin
         v <= {NENT{1'b0}};
         qhead <= {IDXB{1'b0}}; qtail <= {IDXB{1'b0}};
      end else begin
         for (k = 0; k < NENT; k = k + 1) if (v[k])
            for (q = 0; q < NSRC; q = q + 1)
               if (hit(e_ps[k][q*PBITS +: PBITS])
                   || (self_v && (self_pr == e_ps[k][q*PBITS +: PBITS])))
                  e_r[k][q] <= 1'b1;
         if (do_iss) begin
            v[sel] <= 1'b0;
            if (INORDER != 0) qhead <= (qhead == (NENT-1)) ? {IDXB{1'b0}} : qhead + 1'b1;
         end
         if (do_disp) begin
            if (INORDER != 0) qtail <= (qtail == (NENT-1)) ? {IDXB{1'b0}} : qtail + 1'b1;
            v[fsel]     <= 1'b1;
            e_rob[fsel] <= d_rob;
            e_ps[fsel]  <= d_ps;
            e_prd[fsel] <= d_prd;
            for (q = 0; q < NSRC; q = q + 1)
               e_r[fsel][q] <= d_r[q] | hit(d_ps[q*PBITS +: PBITS])
                             | (self_v && (self_pr == d_ps[q*PBITS +: PBITS]));
         end
      end
   end

   // ---- invariants (always on: docs/rtl-rules.md A1) ---------------------------------
   always @(posedge clk) if (!reset) begin
      if (d_valid & ~d_ready & ~flush)
         $fatal(1, "ooo2_rs: dispatch into a full scheduler");
      if (do_disp & v[fsel])
         $fatal(1, "ooo2_rs: dispatch into occupied entry %0d", fsel);
      if (do_disp & held[fsel])
         $fatal(1, "ooo2_rs: dispatch into the entry still held downstream (%0d)", fsel);
      if (iss_take & ~iss_v)
         $fatal(1, "ooo2_rs: consumer took an issue that was not offered");
      if (do_iss & ~v[sel])
         $fatal(1, "ooo2_rs: issued entry %0d holds nothing", sel);
      if (do_iss & unit_busy)
         $fatal(1, "ooo2_rs: issued to a busy unit");
      if ((INORDER != 0) & do_iss & (sel != qhead))
         $fatal(1, "ooo2_rs: in-order scheduler issued %0d, not the head %0d", sel, qhead);
   end
endmodule

`default_nettype wire
