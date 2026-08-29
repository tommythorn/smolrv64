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
//             entry order depend on ISSUE order, and u_rs_l is meant to stop being in-order
//             (spec 15). Program order is what the ROB index and the store-seqno mean.
//   fill      when the load's address is translated. M releases it here -- from this point
//             the access cannot fault (ooo2_lsu decides faults before it leaves S_IDLE), so
//             the load is architecturally guaranteed to complete.
//   access    the oldest filled entry that no older store can alias. One at a time: the D$
//             has one port, so exactly one entry needs testing per cycle and the queue needs
//             exactly one disambiguation query port into ooo2_sq.
//   land      data returns, writes back, completes the ROB slot.
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
    input  wire                  a_v,
    input  wire [IDXB-1:0]       a_idx,
    input  wire [PAW-1:0]        a_pa,
    input  wire [1:0]            a_size,
    input  wire                  a_signed,
    input  wire                  a_fp,

    // ---- disambiguation: ONE query, for the entry we are trying to start ----
    output wire [PAW-1:0]        q_pa,
    output wire [1:0]            q_size,
    output wire [SQIB-1:0]       q_tag,
    input  wire                  q_block,

    // ---- access: hand the D$ the oldest entry that is clear ----
    output wire                  x_v,
    output wire [PAW-1:0]        x_pa,
    output wire [1:0]            x_size,
    output wire                  x_signed,
    output wire                  x_fp,
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

    output wire [IDXB:0]         occupancy,
    input  wire                  flush);

   // Three pointers, not a state per entry. head..acc are accessed-but-not-landed,
   // acc..tail are waiting. A pointer triple is what makes flush a pointer reset.
   reg [NENT-1:0]        v, av;              // live / address known
   reg [PAW-1:0]         pa   [0:NENT-1];
   reg [1:0]             sz   [0:NENT-1];
   reg [NENT-1:0]        sgn, isfp;
   reg [PBITS-1:0]       prd  [0:NENT-1];
   reg [5:0]             rdn  [0:NENT-1];
   reg [NENT-1:0]        rdv;
   reg [ROBB-1:0]        rob  [0:NENT-1];
   reg [SQIB-1:0]        sqt  [0:NENT-1];
   reg [IDXB-1:0]        head, acc, tail;
   reg [IDXB:0]          cnt;
   integer               k;

   initial begin v = {NENT{1'b0}}; av = {NENT{1'b0}};
                 head = {IDXB{1'b0}}; acc = {IDXB{1'b0}}; tail = {IDXB{1'b0}};
                 cnt = {(IDXB+1){1'b0}}; end

   assign d_ready   = (cnt != NENT[IDXB:0]);
   assign d_idx     = tail;
   assign occupancy = cnt;

   // The candidate: the oldest entry not yet sent to memory. Its address must be known --
   // an entry still waiting for translation cannot be tested and must not be skipped, or
   // loads would access out of program order with respect to each other with nothing
   // ordering them.
   wire cand_v = v[acc] & av[acc];
   assign q_pa    = pa[acc];
   assign q_size  = sz[acc];
   assign q_tag   = sqt[acc];
   assign x_v     = cand_v & ~q_block;
   assign x_pa    = pa[acc];
   assign x_size  = sz[acc];
   assign x_signed= sgn[acc];
   assign x_fp    = isfp[acc];

   // The landing entry's payload, selected by the tag the response carried.
   assign l_prd  = prd[l_idx];
   assign l_rd   = rdn[l_idx];
   assign l_rd_v = rdv[l_idx];
   assign l_rob  = rob[l_idx];

   always @(posedge clk) begin
      if (reset | flush) begin
         v <= {NENT{1'b0}}; av <= {NENT{1'b0}};
         head <= {IDXB{1'b0}}; acc <= {IDXB{1'b0}}; tail <= {IDXB{1'b0}};
         cnt <= {(IDXB+1){1'b0}};
      end else begin
         // Retiring the entry frees its slot. head only advances when the OLDEST entry
         // lands, so an out-of-order response leaves the slot marked dead and the pointer
         // catches up -- which is also what keeps cnt honest.
         if (l_v) begin v[l_idx] <= 1'b0; av[l_idx] <= 1'b0; end
         if (l_v & (l_idx == head)) head <= head + 1'b1;
         if (x_v & x_take) acc <= acc + 1'b1;
         if (d_alloc & d_ready) begin
            v[tail] <= 1'b1; av[tail] <= 1'b0;
            rob[tail] <= d_rob; prd[tail] <= d_prd; rdn[tail] <= d_rd;
            rdv[tail] <= d_rd_v; sqt[tail] <= d_sqtag;
            tail <= tail + 1'b1;
         end
         if ((d_alloc & d_ready) & ~l_v)      cnt <= cnt + 1'b1;
         else if (~(d_alloc & d_ready) & l_v) cnt <= cnt - 1'b1;

         if (a_v) begin
            pa[a_idx] <= a_pa;  sz[a_idx] <= a_size;
            sgn[a_idx] <= a_signed;  isfp[a_idx] <= a_fp;  av[a_idx] <= 1'b1;
         end
      end
   end

   // Invariants (docs/rtl-rules.md A1). Every one of these is something the queue would
   // otherwise do silently and wrongly.
   always @(posedge clk) if (!reset) begin
      if (d_alloc & ~d_ready)
         $fatal(1, "ooo2_lq: allocate into a full queue");
      if (a_v & ~v[a_idx])
         $fatal(1, "ooo2_lq: address written to a slot with no live entry (idx %0d)", a_idx);
      if (a_v & av[a_idx])
         $fatal(1, "ooo2_lq: address written twice to slot %0d", a_idx);
      if (l_v & ~v[l_idx])
         $fatal(1, "ooo2_lq: data landed for a slot with no live entry (idx %0d)", l_idx);
      if (l_v & (l_idx == acc) & (head == acc))
         $fatal(1, "ooo2_lq: data landed for an entry that was never sent to memory");
      if (x_v & x_take & ~av[acc])
         $fatal(1, "ooo2_lq: started an access with no translated address");
   end
endmodule
`default_nettype wire
