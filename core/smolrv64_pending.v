`default_nettype none

// Per-physical-register readiness. docs/Area-Efficient-Scalar-OoO.md 5, "Values and
// readiness": `pending[NPHYS]`, set at rename, cleared at writeback, bulk-cleared on flush.
// Dynamic issue makes "how many results are outstanding" unbounded, so readiness is state per
// register rather than a comparison against the units.
//
// Indexed by the FULL physical register number, so the array is 2**PBITS deep and the
// shard bits in the top of the number simply select a region of it: a decode-free index for
// some unreachable flops.
module smolrv64_pending
  #(parameter PBITS = 9,
    parameter NS    = 3,                  // allocations a cycle: one per rename slot
    parameter NWB   = 3,                  // writeback broadcasts
    parameter NQ    = 1)                  // readiness queries
   (input  wire                  clk,
    input  wire                  reset,

    // ---- allocate: rename gives slot k's destination a pending bit, the same cycle ----
    input  wire [NS-1:0]         a_v,
    input  wire [NS*PBITS-1:0]   a_preg,

    // ---- writeback: the value has landed ----
    input  wire [NWB-1:0]        w_v,
    input  wire [NWB*PBITS-1:0]  w_preg,
    // ---- integrity log (registered): {a register allocated while pending, a writeback to a
    // register that is not pending -- a squashed op's result landing after its register died}
    output reg  [1:0]            err,

    // ---- read: "is this register ready?" Readiness is a pure function of `pend` (no
    // same-cycle write-forward: the schedulers cover a producer writing back in the cycle its
    // consumer is written, and a forward here would close a loop through readiness), so a
    // query of either map candidate and a late pick of the RESULT is the same as a query of
    // the picked candidate. Read ports are the one thing duplication genuinely buys.
    input  wire [NQ*PBITS-1:0]   q,
    output wire [NQ-1:0]         r,

    // ---- recovery ----
    input  wire                  flush);

   localparam integer NP = 1 << PBITS;

   reg [NP-1:0] pend;
   integer      i, j;
   initial      pend = {NP{1'b0}};

   // Physical register 0 is architectural x0 and is never allocated, so it is always
   // ready; folding that in here keeps every reader from special-casing it.
   genvar gq;
   generate for (gq = 0; gq < NQ; gq = gq + 1) begin: rq
      wire [PBITS-1:0] x = q[gq*PBITS +: PBITS];
      assign r[gq] = (x == {PBITS{1'b0}}) ? 1'b1 : ~pend[x];
   end endgenerate

   always @(posedge clk) begin
      if (reset) begin
         pend <= {NP{1'b0}};
      end else begin
         for (i = 0; i < NWB; i = i + 1)
            if (w_v[i]) pend[w_preg[i*PBITS +: PBITS]] <= 1'b0;
         // Ordered after the writebacks: an allocation in the same cycle as a writeback to
         // the SAME register means the register was just freed and re-allocated, and the
         // new producer owns it.
         for (i = 0; i < NS; i = i + 1)
            if (a_v[i]) pend[a_preg[i*PBITS +: PBITS]] <= 1'b1;
         // Total recovery, ordered LAST so it wins over an allocation made in the flush
         // cycle (dispatch is not gated on the redirect). Everything uncommitted dies, and
         // every COMMITTED register's value is by definition already in the PRF, so no live
         // pending bit survives a flush. doc 1, property 4.
         if (flush) pend <= {NP{1'b0}};
      end
   end

   // ---- invariants (always on: docs/rtl-rules.md A1) ---------------------------------
   reg e_zombie, e_realloc;
   integer iz, jz;
   always @* begin
      e_zombie = 1'b0;
      for (iz = 0; iz < NWB; iz = iz + 1)
         if (w_v[iz] & ~pend[w_preg[iz*PBITS +: PBITS]] & ~flush) e_zombie = 1'b1;
      // The set is every candidate's (smolrv64_core), so a register may be set again while set:
      // an untaken candidate is a free register. What must never happen is two slots naming one
      // register in a cycle.
      e_realloc = 1'b0;
      for (iz = 0; iz < NS; iz = iz + 1)
         for (jz = iz + 1; jz < NS; jz = jz + 1)
            if (a_v[iz] & a_v[jz] & (a_preg[iz*PBITS +: PBITS] == a_preg[jz*PBITS +: PBITS]) & ~flush)
               e_realloc = 1'b1;
   end
   always @(posedge clk) err <= reset ? 2'b00 : {e_realloc, e_zombie};
   always @(posedge clk) if (!reset) begin
      for (i = 0; i < NS; i = i + 1) begin
         if (a_v[i] & (a_preg[i*PBITS +: PBITS] == {PBITS{1'b0}}))
            $fatal(1, "smolrv64_pending: physical register 0 allocated (slot %0d)", i);
         for (j = i + 1; j < NS; j = j + 1)
            if (a_v[i] & a_v[j] & (a_preg[i*PBITS +: PBITS] == a_preg[j*PBITS +: PBITS]) & ~flush)
               $fatal(1, "smolrv64_pending: slots %0d and %0d both set p%0d", i, j, a_preg[j*PBITS +: PBITS]);
      end
      for (i = 0; i < NWB; i = i + 1)
         if (w_v[i] & ~pend[w_preg[i*PBITS +: PBITS]] & ~flush)
            $fatal(1, "smolrv64_pending: writeback to p%0d, which was not pending",
                   w_preg[i*PBITS +: PBITS]);
   end
endmodule

`default_nettype wire
