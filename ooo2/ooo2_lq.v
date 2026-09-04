`default_nettype none

// ooo2_lq -- the load queue.
//
// WHY. docs/Area-Efficient-Scalar-OoO.md 11.1: ISSUE is free, ACCESS is ordered. The store
// buffer applied that to stores. This applies it to loads, and without it the ordering test
// has nowhere to live but the translate path:
//
//   ooo2_lsu used to gate `start_ok` on ooo2_sq's ld_block, which is computed from the
//   MMU's t_paddr. So a blocked load sat in S_IDLE re-translating every cycle, holding the
//   whole LSU, with an adder and two 57-bit magnitude compares hanging off the end of the
//   dTLB lookup. It was correct and it was the wrong shape: translate and access were
//   fused, which is exactly the conflation 11.1 exists to undo.
//
// Here the address is REGISTERED into an entry when translation completes, and the alias
// test reads that flop. The compare starts at the top of a cycle instead of at the end of
// the translate path, and the LSU is free the moment the address is known.
//
// SHAPE, deliberately the same as ooo2_sq's, because they are the same idea applied to the
// two directions:
//   allocate  at DISPATCH, in program order. Not at fill: allocating at fill would make
//             entry order depend on ISSUE order, and u_iq_l is meant to stop being in-order
//             (spec 15). Program order is what the ROB index and the store-seqno mean.
//   fill      when the load's address is translated. M releases it here -- from this point
//             the access cannot fault (ooo2_lsu decides faults before it leaves S_IDLE), so
//             the load is architecturally guaranteed to complete.
//   access    the oldest filled entry that no older store can alias. One at a time: the D$
//             has one port, so exactly one entry needs testing per cycle and the queue needs
//             exactly one disambiguation query port into ooo2_sq.
//   land      data returns, writes back, completes the ROB slot.
//
// FILL AND ACCESS COLLAPSE when there is nothing to disambiguate against. The SELECT cycle
// exists to read a registered address into the alias test; a load with no store older than
// it still live has no alias test to run, so the core starts its access in the same pass
// that fills the entry (a_sent) and the entry goes straight to `sent`. Everything else is
// unchanged -- the entry is still filled, M is still released here, and the data still lands
// through l_*. That last part is the point: an earlier attempt let M keep the load through
// its access instead, and giving up the early release cost more than the cycles it saved
// (ldbench 6.00 -> 7.00 cyc/load). The queue's two cycles are not pure overhead.
//
// FLUSH IS WHOLESALE for the same reason as ooo2_sq's: `redirect` is gated by head_block,
// so it fires only with the redirecting instruction at the ROB head, and everything still
// live here is younger than that. A load already ACCESSED but not yet landed is younger
// too -- a load has no side effect, so discarding its result is free.
module ooo2_lq
  #(parameter NENT  = 4,
    parameter IDXB  = 2,               // $clog2(NENT)
    parameter PAW   = 56,
    parameter PBITS = 9,
    parameter ROBB  = 4,
    parameter SQIB  = 3)               // store-seqno width (ooo2_sq's IDXB)
   (input  wire                  clk,
    input  wire                  reset,

    // ---- allocate: at dispatch, program order, one per cycle ----
    input  wire                  d_alloc,
    input  wire [ROBB-1:0]       d_rob,
    input  wire [PBITS-1:0]      d_prd,
    input  wire [5:0]            d_rd,
    input  wire                  d_rd_v,
    input  wire [SQIB-1:0]       d_sqtag,     // store-seqno captured at the same dispatch
    output wire                  d_ready,
    output wire [IDXB-1:0]       d_idx,

    // ---- fill: the address is translated; M lets go here ----
    // a_sent says the SAME cycle also handed the access to memory (see the header). The
    // entry skips straight past `acc` -- it never becomes a candidate, because it has
    // already been what a candidate is for.
    input  wire                  a_v,
    input  wire                  a_sent,
    input  wire [IDXB-1:0]       a_idx,
    input  wire [PAW-1:0]        a_pa,
    input  wire [1:0]            a_size,
    input  wire                  a_signed,
    input  wire                  a_fp,
    input  wire                  a_unc,       // Svpbmt NC/IO: decided by the translation, travels with the entry

    // ---- disambiguation: a BIT PER ENTRY, maintained by ooo2_sq -----------------------
    // The entries are exported so ooo2_sq can run the alias test where an ADDRESS ARRIVES
    // and hand back a flop. Asking it at issue instead put `acc` at the head of a 36-level,
    // 12x CARRY8 cone ending at the frontend's redirect register: WNS -1.698 ns at
    // 166.67 MHz, 18 670 failing endpoints, all five worst paths sourced here. Moving the
    // address off the TRANSLATE path -- this module's original purpose -- left the compare
    // itself, and its whole downstream tail, exactly where they were.
    output wire [NENT*PAW-1:0]   e_pa,
    output wire [NENT*2-1:0]     e_size,
    output wire [NENT*SQIB-1:0]  e_tag,
    output wire [NENT-1:0]       e_av,
    input  wire [NENT-1:0]       e_block,     // per entry: an older store aliases it
    output wire                  x_block,     // ...and the candidate is one, held right now
    // ...and the seqno of the entry being tested, for ooo2_sq's ld_older. That one is
    // pointer arithmetic against head, with no address and no adder in it.
    output wire [SQIB-1:0]       q_tag,

    // ---- early start: may the core collapse fill and access for the entry M holds? ----
    // Only for the CANDIDATE: every entry older than acc has already been sent, so a load
    // starting from here is still the oldest load that has not reached memory, and loads
    // keep reaching memory in program order -- the property cand_v exists to hold.
    input  wire [IDXB-1:0]       b_idx,       // the entry M's load owns
    output wire                  b_ok,        // ...and it is the untranslated candidate

    // ---- access: hand the D$ the oldest entry that is clear ----
    output wire                  x_v,
    output wire [IDXB-1:0]       x_idx,       // the entry being offered -- the response's tag
    output wire [PAW-1:0]        x_pa,
    output wire [1:0]            x_size,
    output wire                  x_signed,
    output wire                  x_fp,
    output wire                  x_unc,
    input  wire                  x_take,      // the LSU accepted it

    // ---- land: data back for an entry that was sent to memory ----
    // The response names its ENTRY, it is not assumed to be the oldest one outstanding
    // (rule B1: a response is matched by a tag the requester allocated). Today the D$
    // returns in order and l_idx is always the head, but the moment loads are pipelined
    // through rd_tag/rd_resp_tag that stops being true, and this interface does not have
    // to change for it.
    input  wire                  l_v,
    input  wire [IDXB-1:0]       l_idx,
    output wire [PBITS-1:0]      l_prd,
    output wire [5:0]            l_rd,
    output wire                  l_rd_v,
    output wire [ROBB-1:0]       l_rob,
    // The landing load's OWN address, for the cosim's memory-effect record. It has to come
    // from the entry that owns the load: the shared lsu_cos_* pair is whatever access ran
    // LAST, and since b9dbdd0 a queued load's access starts in its translate pass, a store
    // can and does start between a load's start and its landing.
    output wire [PAW-1:0]        l_pa,

    output wire [IDXB:0]         occupancy,
    input  wire                  flush);

   // Two pointers and a per-entry `sent` bit. acc..tail are waiting to go to memory; what
   // is behind acc is outstanding or already landed. Pointers are what make flush a reset.
   reg [NENT-1:0]        v, av;              // live / address known
   reg [PAW-1:0]         pa   [0:NENT-1];
   reg [1:0]             sz   [0:NENT-1];
   // EVERY ATTRIBUTE THE ACCESS NEEDS TRAVELS WITH THE ENTRY. `unc` was missing: a queued
   // load went to the LSU with the uncached bit hardwired to 0 (ooo2_core's pt_unc), so a
   // Svpbmt NC load whose access came from the queue -- rather than the early start, which
   // reads the MMU's bit directly -- was cached, the line stayed resident, and every later
   // read of it hit stale data. Linux's virtio rings live in exactly such memory: "id 0 is
   // not a head!" on the board, 2026-09-04, with no simulation able to see it (the cosim's
   // guest maps nothing NC). The store queue had carried its own bit all along.
   reg [NENT-1:0]        sgn, isfp, unc;
   reg [PBITS-1:0]       prd  [0:NENT-1];
   reg [5:0]             rdn  [0:NENT-1];
   reg [NENT-1:0]        rdv;
   reg [ROBB-1:0]        rob  [0:NENT-1];
   reg [SQIB-1:0]        sqt  [0:NENT-1];
   // `sent` replaces a HEAD pointer, which could only say "the OLDEST entry is outstanding".
   // That stops being the same question once an entry can reach memory without ever being
   // the candidate (a_sent). Per entry it is exact, and it is what the landing assertion
   // below actually wants to know.
   reg [NENT-1:0]        sent;                // handed to memory, data not back yet
   reg [IDXB-1:0]        acc, tail;
   reg [IDXB:0]          cnt;

   initial begin v = {NENT{1'b0}}; av = {NENT{1'b0}}; sent = {NENT{1'b0}};
                 acc = {IDXB{1'b0}}; tail = {IDXB{1'b0}};
                 cnt = {(IDXB+1){1'b0}}; end

   // THE SLOT AT THE TAIL MUST BE FREE, not merely "the queue is not full". Loads land out of
   // order once they are pipelined (the tagged fast path; the random bench lands them in any
   // order), so the ring can wrap round to a slot whose load is still in flight while cnt
   // says there is room. Allocating there overwrote the entry and its landing then found
   // "an entry that was never sent" (2026-09-04). Waiting on that one slot is head-of-line
   // blocking at a miss; at NENT=4 the pointer ring is the right size for it.
   assign d_ready   = (cnt != NENT[IDXB:0]) & ~v[tail];
   assign d_idx     = tail;
   assign occupancy = cnt;

   // The candidate: the oldest entry not yet sent to memory. Its address must be known --
   // an entry still waiting for translation cannot be tested and must not be skipped, or
   // loads would access out of program order with respect to each other with nothing
   // ordering them.
   // ...and NOT already sent: with every entry taken and none landed, acc wraps round to the
   // head, which is live, translated and IN FLIGHT, and `v & av` offered it a second time.
   // Unreachable while one load is in flight at a time (the FSM parks), which is why nothing
   // saw it; the random bench (tb_ooo2_lqsq_rand, seed 1, cycle 358) found it in its first
   // run, and the tagged fast path can hold NENT in flight.
   wire cand_v = v[acc] & av[acc] & ~sent[acc];
   // The mirror of cand_v: the candidate is live and its address is NOT yet known, which is
   // exactly a load still in M. q_tag is sqt[acc], so this is also what licenses the core to
   // read ooo2_sq's ld_older as an answer about the load M is holding.
   assign b_ok = (b_idx == acc) & v[acc] & ~av[acc];
   assign q_tag   = sqt[acc];
   genvar ge;
   generate
      for (ge = 0; ge < NENT; ge = ge + 1) begin : g_exp
         assign e_pa  [ge*PAW  +: PAW ] = pa [ge];
         assign e_size[ge*2    +: 2   ] = sz [ge];
         assign e_tag [ge*SQIB +: SQIB] = sqt[ge];
      end
   endgenerate
   assign e_av = av;
   // ONE 4:1 mux from acc, into a flop-sourced bit. That is the whole point.
   assign x_v     = cand_v & ~e_block[acc];
   assign x_block = cand_v &  e_block[acc];   // instrumentation only
   assign x_idx   = acc;
   assign x_pa    = pa[acc];
   assign x_size  = sz[acc];
   assign x_signed= sgn[acc];
   assign x_fp    = isfp[acc];
   assign x_unc   = unc[acc];

   // The landing entry's payload, selected by the tag the response carried.
   assign l_prd  = prd[l_idx];
   assign l_rd   = rdn[l_idx];
   assign l_rd_v = rdv[l_idx];
   assign l_rob  = rob[l_idx];
   assign l_pa   = pa[l_idx];

   always @(posedge clk) begin
      if (reset | flush) begin
         v <= {NENT{1'b0}}; av <= {NENT{1'b0}}; sent <= {NENT{1'b0}};
         acc <= {IDXB{1'b0}}; tail <= {IDXB{1'b0}};
         cnt <= {(IDXB+1){1'b0}};
      end else begin
         // Retiring the entry frees its slot; slot REUSE is governed by cnt and tail, so an
         // out-of-order response simply leaves the slot dead until tail comes round to it.
         if (l_v) begin v[l_idx] <= 1'b0; av[l_idx] <= 1'b0; sent[l_idx] <= 1'b0; end
         if (x_v & x_take) begin acc <= acc + 1'b1; sent[acc] <= 1'b1; end
         if (d_alloc & d_ready) begin
            v[tail] <= 1'b1; av[tail] <= 1'b0; sent[tail] <= 1'b0;
            rob[tail] <= d_rob; prd[tail] <= d_prd; rdn[tail] <= d_rd;
            rdv[tail] <= d_rd_v; sqt[tail] <= d_sqtag;
            tail <= tail + 1'b1;
         end
         if ((d_alloc & d_ready) & ~l_v)      cnt <= cnt + 1'b1;
         else if (~(d_alloc & d_ready) & l_v) cnt <= cnt - 1'b1;

         if (a_v) begin
            pa[a_idx] <= a_pa;  sz[a_idx] <= a_size;
            sgn[a_idx] <= a_signed;  isfp[a_idx] <= a_fp;  unc[a_idx] <= a_unc;  av[a_idx] <= 1'b1;
         end
         // Filled AND already gone. It cannot collide with x_take above: that one needs
         // av[acc], and a_sent is asserted only while b_ok says ~av[acc].
         if (a_v & a_sent) begin sent[a_idx] <= 1'b1; acc <= acc + 1'b1; end
      end
   end

   // Invariants (docs/rtl-rules.md A1). Every one of these is something the queue would
   // otherwise do silently and wrongly.
   always @(posedge clk) if (!reset) begin
      if (d_alloc & ~d_ready)
         $fatal(1, "ooo2_lq: allocate into a full queue");
      if (d_alloc & d_ready & v[tail])
         $fatal(1, "ooo2_lq: allocate into slot %0d while its load is in flight", tail);
      if (a_v & ~v[a_idx])
         $fatal(1, "ooo2_lq: address written to a slot with no live entry (idx %0d)", a_idx);
      if (a_v & av[a_idx])
         $fatal(1, "ooo2_lq: address written twice to slot %0d", a_idx);
      if (l_v & ~v[l_idx])
         $fatal(1, "ooo2_lq: data landed for a slot with no live entry (idx %0d)", l_idx);
      if (l_v & ~sent[l_idx])
         $fatal(1, "ooo2_lq: data landed for an entry that was never sent to memory (idx %0d)", l_idx);
      if (x_v & x_take & ~av[acc])
         $fatal(1, "ooo2_lq: started an access with no translated address");
      if (a_v & a_sent & ~b_ok)
         $fatal(1, "ooo2_lq: entry %0d started early but is not the untranslated candidate (acc %0d)",
                a_idx, acc);
      if (a_v & a_sent & x_v & x_take)
         $fatal(1, "ooo2_lq: an early start and a candidate start in the same cycle");
   end
endmodule
`default_nettype wire
