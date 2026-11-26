`default_nettype none

// Per-physical-register readiness. docs/Area-Efficient-Scalar-OoO.md 5, "Values and
// readiness": `pending[NPHYS]`, set at rename, cleared at writeback, bulk-cleared on flush.
// Dynamic issue makes "how many results are outstanding" unbounded, so readiness is state per
// register rather than a comparison against the units.
//
// TWO TABLES, ONE PER SIDE: the integer one holds the lanes' shards (NL x SHN bits), the F side's
// SH_FL's, SH_FF's, the vector file's and the VL file's (4 x SHN). A register's number says which
// holds it (shard[2] is the F side, smolrv64_shards.vh) and where: {the shard's low bits, the
// index}, the index below SHN (the VL file's below 16). Each read is a mux over its own side's
// registers alone, not over every number a register could have.
module smolrv64_pending
  #(parameter PBITS = 10,
    parameter IDXB  = 7,                  // index bits within a shard, in a register's number
    parameter SHN   = 64,                 // registers a shard holds (a power of two)
    parameter NL    = 3,                  // integer shards 0..NL-1 (at least two)
    parameter NS    = 3,                  // allocations a cycle: one per rename slot
    parameter NWB   = 3,                  // writeback broadcasts
    parameter NWBF  = 2,                  // ...of which the last NWBF may name an f-register
    parameter NQ    = 1,                  // readiness queries of either file
    parameter NQF   = 1)                  // ...and of an f-register or p0
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

    // ---- read: "is this register ready?" Readiness is a pure function of the tables (no
    // same-cycle write-forward: the schedulers cover a producer writing back in the cycle its
    // consumer is written, and a forward here would close a loop through readiness), so a
    // query of the rename map's read with a late mask of the RESULT (an earlier slot's bypass)
    // is the same as a query of the bypassed register. Read ports are the one thing duplication genuinely buys. A query
    // of either file reads both tables and its number's shard[2] picks; a query that names an
    // f-register or p0 (an absent source) reads the FP table alone.
    input  wire [NQ*PBITS-1:0]   q,
    output wire [NQ-1:0]         r,
    input  wire [NQF*PBITS-1:0]  qf,
    output wire [NQF-1:0]        rf,

    // ---- recovery ----
    input  wire                  flush);

`include "smolrv64_shards.vh"
   localparam integer SB  = $clog2(SHN);           // the index bits a register uses
   localparam integer NI  = NL * SHN;              // the integer table
   localparam integer NF  = 4 * SHN;               // the F side's table: SH_FL, SH_FF, SH_V, SH_VL
   localparam integer IIB = $clog2(NI);
   localparam integer IFB = SB + 2;

   initial begin
      if ((SHN & (SHN - 1)) != 0) $fatal(1, "smolrv64_pending: a shard of %0d registers is not a power of two", SHN);
      if (SHN > (1 << IDXB))      $fatal(1, "smolrv64_pending: %0d registers a shard exceed IDXB=%0d", SHN, IDXB);
      if (NL < 2)                 $fatal(1, "smolrv64_pending: %0d integer shards; the table indexes at least two", NL);
   end
   reg [NI-1:0] ipend;
   reg [NF-1:0] fpend;
   integer      i, j;
   initial begin ipend = {NI{1'b0}}; fpend = {NF{1'b0}}; end

   // a register's file, and its place in that file's table
   function automatic is_f(input [PBITS-1:0] p);
      is_f = p[PBITS-1];
   endfunction
   function automatic [IIB-1:0] ix_i(input [PBITS-1:0] p);
      ix_i = {p[IDXB +: IIB-SB], p[SB-1:0]};
   endfunction
   function automatic [IFB-1:0] ix_f(input [PBITS-1:0] p);
      ix_f = {p[IDXB +: 2], p[SB-1:0]};             // SH_FL 4, SH_FF 5, SH_V 6, SH_VL 7: shard[1:0]
   endfunction

   // Physical register 0 is architectural x0 and is never allocated, so it is always
   // ready; folding that in here keeps every reader from special-casing it.
   genvar gq;
   generate for (gq = 0; gq < NQ; gq = gq + 1) begin: rq
      wire [PBITS-1:0] x  = q[gq*PBITS +: PBITS];
      wire             ri = ~ipend[ix_i(x)], rp = ~fpend[ix_f(x)];
      assign r[gq] = (x == {PBITS{1'b0}}) | (is_f(x) ? rp : ri);
   end endgenerate
   generate for (gq = 0; gq < NQF; gq = gq + 1) begin: rqf
      wire [PBITS-1:0] x = qf[gq*PBITS +: PBITS];
      assign rf[gq] = (x == {PBITS{1'b0}}) | ~fpend[ix_f(x)];
   end endgenerate

   always @(posedge clk) begin
      if (reset) begin
         ipend <= {NI{1'b0}};  fpend <= {NF{1'b0}};
      end else begin
         // a lane's port names an integer register; only the streams' may name an f-register
         for (i = 0; i < NWB; i = i + 1)
            if (w_v[i]) begin
               if (~is_f(w_preg[i*PBITS +: PBITS]))  ipend[ix_i(w_preg[i*PBITS +: PBITS])] <= 1'b0;
               else if (i >= NWB - NWBF)             fpend[ix_f(w_preg[i*PBITS +: PBITS])] <= 1'b0;
            end
         // Ordered after the writebacks: an allocation in the same cycle as a writeback to
         // the SAME register means the register was just freed and re-allocated, and the
         // new producer owns it.
         for (i = 0; i < NS; i = i + 1)
            if (a_v[i]) begin
               if (is_f(a_preg[i*PBITS +: PBITS])) fpend[ix_f(a_preg[i*PBITS +: PBITS])] <= 1'b1;
               else                                ipend[ix_i(a_preg[i*PBITS +: PBITS])] <= 1'b1;
            end
         // Total recovery, ordered LAST so it wins over an allocation made in the flush
         // cycle (dispatch is not gated on the redirect). Everything uncommitted dies, and
         // every COMMITTED register's value is by definition already in the PRF, so no live
         // pending bit survives a flush. doc 1, property 4.
         if (flush) begin ipend <= {NI{1'b0}};  fpend <= {NF{1'b0}}; end
      end
   end

   // ---- invariants (always on: docs/rtl-rules.md A1) ---------------------------------
   function automatic pnd(input [PBITS-1:0] p);
      pnd = is_f(p) ? fpend[ix_f(p)] : ipend[ix_i(p)];
   endfunction
   // a number in neither table: a shard no file has, or an index past its shard's registers
   function automatic stray(input [PBITS-1:0] p);
      reg [2:0] s;
      begin
         s = p[IDXB +: 3];
         stray = ({1'b0, p[IDXB-1:0]} >= SHN[IDXB:0])
               | ((s == SH_VL) & (p[IDXB-1:0] >= 16))
               | (is_f(p) ? 1'b0 : (s >= NL[2:0]));
      end
   endfunction
   reg e_zombie, e_realloc;
   integer iz, jz;
   always @* begin
      e_zombie = 1'b0;
      for (iz = 0; iz < NWB; iz = iz + 1)   // a lane's port reads the integer table alone
         if (w_v[iz] & ~((iz >= NWB - NWBF) ? pnd(w_preg[iz*PBITS +: PBITS]) : ipend[ix_i(w_preg[iz*PBITS +: PBITS])])
             & ~flush) e_zombie = 1'b1;
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
         if (a_v[i] & stray(a_preg[i*PBITS +: PBITS]))
            $fatal(1, "smolrv64_pending: slot %0d allocates p%0h, in neither table", i, a_preg[i*PBITS +: PBITS]);
         for (j = i + 1; j < NS; j = j + 1)
            if (a_v[i] & a_v[j] & (a_preg[i*PBITS +: PBITS] == a_preg[j*PBITS +: PBITS]) & ~flush)
               $fatal(1, "smolrv64_pending: slots %0d and %0d both set p%0d", i, j, a_preg[j*PBITS +: PBITS]);
      end
      for (i = 0; i < NWB; i = i + 1) begin
         if (w_v[i] & stray(w_preg[i*PBITS +: PBITS]))
            $fatal(1, "smolrv64_pending: writeback %0d names p%0h, in neither table", i, w_preg[i*PBITS +: PBITS]);
         if (w_v[i] & is_f(w_preg[i*PBITS +: PBITS]) & (i < NWB - NWBF))
            $fatal(1, "smolrv64_pending: writeback %0d, a lane's, names f-register p%0h", i, w_preg[i*PBITS +: PBITS]);
         if (w_v[i] & ~pnd(w_preg[i*PBITS +: PBITS]) & ~flush)
            $fatal(1, "smolrv64_pending: writeback to p%0d, which was not pending",
                   w_preg[i*PBITS +: PBITS]);
      end
      for (i = 0; i < NQ; i = i + 1)
         if ((q[i*PBITS +: PBITS] != {PBITS{1'b0}}) & stray(q[i*PBITS +: PBITS]))
            $fatal(1, "smolrv64_pending: query %0d names p%0h, in neither table", i, q[i*PBITS +: PBITS]);
      for (i = 0; i < NQF; i = i + 1)
         if ((qf[i*PBITS +: PBITS] != {PBITS{1'b0}}) & (~is_f(qf[i*PBITS +: PBITS]) | stray(qf[i*PBITS +: PBITS])))
            $fatal(1, "smolrv64_pending: FP query %0d names p%0h, not an f-register", i, qf[i*PBITS +: PBITS]);
   end
endmodule

`default_nettype wire
