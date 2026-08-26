`default_nettype none

// Per-physical-register readiness. docs/Area-Efficient-Scalar-OoO.md 5, "Values and
// readiness": `pending[NPHYS]`, set at rename, cleared at writeback, bulk-cleared on flush.
//
// This is what the scheduler needs and what the current interlock is a degenerate case of.
// ooo2_core today compares a source against ONE outstanding load tag and ONE outstanding FP
// tag, which is exactly enough while at most one op of each kind can be in flight. Dynamic
// issue makes "how many results are outstanding" unbounded by construction, so readiness
// has to be state per register rather than a comparison against the units.
//
// Indexed by the FULL physical register number, so the array is 2**PBITS deep and the
// shard bits in the top of the number simply select a region of it. 512 bits at PBITS=9,
// against 320 architecturally reachable registers -- the waste is 192 flops and buys a
// decode-free index.
module ooo2_pending
  #(parameter PBITS = 9,
    parameter NWB   = 3)                  // writeback ports, one per PRF shard
   (input  wire                  clk,
    input  wire                  reset,

    // ---- allocate: rename gives the destination a pending bit ----
    input  wire                  a_v,
    input  wire [PBITS-1:0]      a_preg,

    // ---- writeback: the value has landed ----
    input  wire [NWB-1:0]        w_v,
    input  wire [NWB*PBITS-1:0]  w_preg,

    // ---- read: three sources, combinational ----
    input  wire [PBITS-1:0]      q1, q2, q3,
    output wire                  r1, r2, r3,

    // ---- recovery ----
    input  wire                  flush);

   localparam integer NP = 1 << PBITS;

   reg [NP-1:0] pend;
   integer      i;
   initial      pend = {NP{1'b0}};

   // Physical register 0 is architectural x0 and is never allocated, so it is always
   // ready; folding that in here keeps every reader from special-casing it.
   function automatic rdy_of;
      input [PBITS-1:0] q;
      begin rdy_of = (q == {PBITS{1'b0}}) ? 1'b1 : ~pend[q]; end
   endfunction

   // Write-forward the landing writebacks. A consumer renamed in the very cycle its
   // producer writes back must see ready, or it waits for a broadcast that has already
   // happened -- the same hazard the scheduler handles on its dispatch path.
   function automatic wb_hit;
      input [PBITS-1:0] q;
      integer w;
      begin
         wb_hit = 1'b0;
         for (w = 0; w < NWB; w = w + 1)
            if (w_v[w] && (w_preg[w*PBITS +: PBITS] == q)) wb_hit = 1'b1;
      end
   endfunction

   assign r1 = rdy_of(q1) | wb_hit(q1);
   assign r2 = rdy_of(q2) | wb_hit(q2);
   assign r3 = rdy_of(q3) | wb_hit(q3);

   always @(posedge clk) begin
      if (reset | flush) begin
         // Total recovery. Everything uncommitted dies, and every COMMITTED register's
         // value is by definition already in the PRF, so no live pending bit survives a
         // flush. doc 1, property 4.
         pend <= {NP{1'b0}};
      end else begin
         for (i = 0; i < NWB; i = i + 1)
            if (w_v[i]) pend[w_preg[i*PBITS +: PBITS]] <= 1'b0;
         // Ordered last: an allocation in the same cycle as a writeback to the SAME
         // register means the register was just freed and re-allocated, and the new
         // producer owns it.
         if (a_v) pend[a_preg] <= 1'b1;
      end
   end

   // ---- invariants (always on: docs/rtl-rules.md A1) ---------------------------------
   always @(posedge clk) if (!reset) begin
      if (a_v & (a_preg == {PBITS{1'b0}}))
         $fatal(1, "ooo2_pending: physical register 0 allocated");
      if (a_v & pend[a_preg] & ~flush)
         $fatal(1, "ooo2_pending: p%0d allocated while already pending", a_preg);
      for (i = 0; i < NWB; i = i + 1)
         if (w_v[i] & ~pend[w_preg[i*PBITS +: PBITS]] & ~flush)
            $fatal(1, "ooo2_pending: writeback to p%0d, which was not pending",
                   w_preg[i*PBITS +: PBITS]);
   end
endmodule

`default_nettype wire
