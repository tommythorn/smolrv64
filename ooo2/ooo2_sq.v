`default_nettype none

// ooo2_sq -- the store buffer.
//
// WHY. A store currently waits for BOTH its address operand and its data (`rs2`) before it
// may issue, and `u_iq_l` is in-order, so it holds the head of the queue while it waits.
// On workloads/saxpybench (the GB5 Camera loop) the data is an `fadds` result ~21 cycles
// away, and the next element's loads sit behind it -- 22.08 cycles/element for work whose
// elements are completely independent. docs/Area-Efficient-Scalar-OoO.md 11: a store issues
// on its ADDRESS alone and commits when its data arrives.
//
// THE DATA ARRIVES BY SNOOPING, NOT BY A READ PORT. An entry holds `dpreg`, the renamed
// rs2, and watches the writeback ports. When one names `dpreg` the value is captured in
// flight. Holding a register number and reading the PRF at commit would cost a third read
// port, and a read port costs a substantial fraction of the array (doc 15) -- this keeps
// the PRF at 2R1W while still letting address and data arrive apart.
//
// ORDERING. Entries are allocated at DISPATCH in program order. That is not a detail:
// allocating at issue lets younger stores take every slot while an older one waits for its
// operands, and the older store can then never issue to free one (doc 11). Commit is in
// order from the head.
//
// DISAMBIGUATION is a spectrum (doc 11.1) and the cheap end is correct. `ld_block` says an
// older pending store MAY alias:
//   * an entry whose address is not yet computed blocks unconditionally -- nothing can be
//     compared against an unknown address
//   * otherwise the byte ranges are compared, so a load past an adjacent store proceeds
// Byte-range rather than the `|delta| > 8` sketch, because Camera stores y[i] and then
// loads y[i+1] FOUR bytes away: a coarser test would block exactly the case worth winning.
module ooo2_sq
  #(parameter NENT  = 4,
    parameter IDXB  = 2,               // $clog2(NENT)
    parameter PAW   = 56,
    parameter PBITS = 9,
    parameter ROBB  = 4,
    parameter NWB   = 3,
    parameter LQN   = 4,               // ooo2_lq's NENT
    parameter LQIB  = 2)               // ooo2_lq's IDXB
   (input  wire                  clk,
    input  wire                  reset,

    // ---- allocate: at dispatch, program order, one per cycle ----
    input  wire                  d_alloc,
    input  wire [ROBB-1:0]       d_rob,       // rides with the store; commit is at the head
    input  wire [PBITS-1:0]      d_dpreg,     // renamed rs2 AT DISPATCH -- see the snoop
    output wire                  d_ready,
    output wire [IDXB-1:0]       d_idx,
    output wire [IDXB:0]         d_tag,       // the STORE-SEQNO a load captures at dispatch: the
                                              // tail COUNTER, one bit wider than the index (below)

    // ---- address (and possibly data) written when the store executes ----
    input  wire                  a_v,
    input  wire [IDXB-1:0]       a_idx,
    input  wire [PAW-1:0]        a_addr,
    input  wire [1:0]            a_size,     // 0=B 1=H 2=W 3=D
    input  wire                  a_unc,      // uncached, decided at translate time
    input  wire                  a_data_v,   // the issue-time PRF read of rs2 was valid
    input  wire [63:0]           a_data,     //   (ignored once the snoop has the value)

    // ---- snoop the writeback ports for the data operand ----
    input  wire [NWB-1:0]        wb_v,
    input  wire [NWB*PBITS-1:0]  wb_preg,
    input  wire [NWB*64-1:0]     wb_data,

    // ---- commit: the head, once its data has arrived ----
    output wire                  c_v,
    output wire [ROBB-1:0]       c_rob,       // core commits only when this IS the ROB head
    output wire [PAW-1:0]        c_addr,
    output wire [63:0]           c_data,
    output wire [1:0]            c_size,
    output wire                  c_unc,
    input  wire                  c_take,

    // ---- load disambiguation: a CONFLICT MATRIX, not a compare at issue ----------------
    // The alias test runs where an ADDRESS ARRIVES -- one arriving store against every
    // queued load, or one arriving load against every live store -- and the answer is a
    // FLOP. Issue reads a bit.
    //
    // WHY, and it is the whole reason this exists: computing it at issue put ooo2_lq's acc
    // pointer at the head of a 36-level, 12x CARRY8 cone -- acc -> sz[acc] -> these 57-bit
    // overlap compares -> ld_block -> x_v -> pt_start -> start_ok -> lsu_done -> m_done ->
    // redirect -> the frontend's target register. WNS -1.698 ns at 166.67 MHz, 18 670
    // failing endpoints, ALL FIVE worst paths sourced at acc. Moving the address off the
    // TRANSLATE path (which is why ooo2_lq exists) left the compare and its whole downstream
    // tail exactly where they were.
    input  wire [LQN*PAW-1:0]    l_pa,        // the load queue's entries, flattened
    input  wire [LQN*2-1:0]      l_size,
    input  wire [LQN*(IDXB+1)-1:0] l_tag,     // each load's captured store-seqno
    input  wire [LQN-1:0]        l_av,
    input  wire                  l_fill,      // a load's address arrives this cycle...
    input  wire [LQIB-1:0]       l_fill_ix,   // ...into this entry
    input  wire [PAW-1:0]        l_fill_pa,
    input  wire [1:0]            l_fill_size,
    output wire [LQN-1:0]        l_block,     // per entry: an older store aliases it
    // ld_older keeps a query port: it is pointer arithmetic against head, no address and no
    // adder, so it is not part of the cone above.
    input  wire [IDXB:0]         ld_tag,     // this load's captured store-seqno
    output wire                  ld_older,   // an older store is live (aliasing or not) --
                                             // the load is REORDERED past it if it starts

    output wire [IDXB:0]         occupancy,
    input  wire                  flush);

   reg [NENT-1:0]        v, av, dv;          // live / address known / data known
   reg [PAW-1:0]         addr [0:NENT-1];
   reg [63:0]            data [0:NENT-1];
   reg [1:0]             sz   [0:NENT-1];
   reg [NENT-1:0]        unc;
   reg [PBITS-1:0]       dpr  [0:NENT-1];
   reg [ROBB-1:0]        rob  [0:NENT-1];
   // THE SEQNO IS ONE BIT WIDER THAN THE INDEX. A load captures the tail at dispatch and
   // "older" is (slot - head) < (tag - head). With tag and head both IDXB bits, a FULL queue
   // (tail == head) hands the load a distance of ZERO: every live store is older and the
   // arithmetic says none is. The load took the early start, read memory ahead of the store
   // to its own address, and returned the old word -- eight back-to-back stores and a load
   // of the second one's target, Linux's own code (GB5 boot, retire 123,081,278, 2026-09-04);
   // on the board, Ubuntu userspace segfaulted on corrupted pointers. Six days old, from
   // the day the queue was wired in. The counters below carry the wrap bit, so the distance
   // is exact up to NENT; the index is their low bits, as before.
   reg [IDXB:0]          headc, tailc;
   wire [IDXB-1:0]       head = headc[IDXB-1:0], tail = tailc[IDXB-1:0];
   reg [IDXB:0]          cnt;
   integer               k, w;
   reg                   sn_live;         // snoop scratch, see the writeback loop
   reg [PBITS-1:0]       sn_dpr;

   // THE SNOOP TAKES ITS DATA A CYCLE LATE, FROM A REGISTERED COPY OF THE WRITEBACK BUS.
   // wb_data for the IE shard is the ALU result of the instruction issued THIS cycle: the
   // issued source tags, the PRF read, the ALU, and then a 64-bit fanout into every entry's
   // data mux. 392 endpoints of today's routed checkpoint under +0.35 ns were exactly
   // ps_out -> u_prf -> ALU -> u_sq/data_reg (14 levels, 76-80% route). The TAG compare
   // stays live -- it is a registered physreg number against a registered one, and it is
   // what sets dv -- but the bytes are captured from wb_q the cycle after, and for the one
   // cycle in between the head's data is read through a bypass from that same copy. Every
   // observable is unchanged: dv, c_v and c_data have the values they had, on the cycle they
   // had them; only the data register's D input moved from an ALU output to a flop.
   reg [NWB*64-1:0]      wb_q;            // last cycle's writeback data
   reg [NENT-1:0]        ld_v;            // entry k's data lands from wb_q this cycle
   reg [1:0]             ld_w [0:NENT-1]; // ...from this port (NWB <= 4)
   integer               li2;
   initial begin ld_v = {NENT{1'b0}}; for (li2 = 0; li2 < NENT; li2 = li2 + 1) ld_w[li2] = 2'd0; end
   initial if (NWB > 4) $fatal(1, "ooo2_sq: ld_w holds a port index of at most 2 bits");

   initial begin v = {NENT{1'b0}}; av = {NENT{1'b0}}; dv = {NENT{1'b0}};
                 headc = {(IDXB+1){1'b0}}; tailc = {(IDXB+1){1'b0}}; cnt = {(IDXB+1){1'b0}}; end

   assign d_ready   = (cnt != NENT[IDXB:0]);
   assign d_idx     = tail;
   assign d_tag     = tailc;
   assign occupancy = cnt;

   // c_v says the head is READY (address and data present). Committing in program order
   // is the core's job: it takes the entry only when c_rob is the ROB head, so a store
   // reaches memory exactly where the cosim's retire stream expects it.
   assign c_v    = v[head] & av[head] & dv[head];
   assign c_rob  = rob[head];
   assign c_addr = addr[head];
   // the landing bypass: the cycle dv rose, the bytes are still in wb_q
   assign c_data = ld_v[head] ? wb_q[ld_w[head]*64 +: 64] : data[head];
   assign c_size = sz[head];
   assign c_unc  = unc[head];

   // ---- disambiguation: does any live entry overlap this load? ----
   // A byte range is [addr, addr + (1<<size)). Two ranges overlap unless one ends at or
   // before the other begins. An entry with no address yet cannot be compared, so it
   // blocks -- conservative and correct.
   // ONLY ENTRIES OLDER THAN THE LOAD COUNT, and getting this wrong deadlocks rather than
   // corrupting. Entries are allocated at DISPATCH, so the buffer also holds stores YOUNGER
   // than a load sitting in M. Blocking on those is a circular wait: the load waits for a
   // younger store's address, and that store cannot execute because u_iq_l is in-order and
   // the load is at its head. Measured: it hung 115 of 240 tests.
   //
   // The load carries the store-seqno it captured at dispatch -- `d_tag`, the tail at that
   // moment -- and an entry is older than it exactly when it is nearer the head:
   //     dist(x) = (x - head) mod NENT,   older(g) = dist(g) < dist(ld_tag)
   // No wrap bit is needed: a store younger than the load can never commit before the load
   // retires, so the live region cannot cycle past the load's tag.
   localparam [PAW:0] SQ_ONE = {{PAW{1'b0}}, 1'b1};   // width-matched, not a bare literal

   // Byte ranges [a, a + (1<<size)) overlap unless one ends at or before the other begins.
   // ONE definition, used at both matrix-update sites (rule C1) -- a predicate written twice
   // only has to be updated wrong once.
   function ovl_ab;
      input [PAW-1:0] la; input [1:0] ls;
      input [PAW-1:0] sa; input [1:0] ss;
      reg [PAW:0] l_lo, l_hi, s_lo, s_hi;
      begin
         l_lo = {1'b0, la};  l_hi = l_lo + (SQ_ONE << ls);
         s_lo = {1'b0, sa};  s_hi = s_lo + (SQ_ONE << ss);
         ovl_ab = ~((s_hi <= l_lo) | (l_hi <= s_lo));
      end
   endfunction

   // conf[i][g]: load-queue entry i's bytes overlap store g's. Written only when one of the
   // two addresses arrives, so nothing about it is on the issue path.
   reg [NENT-1:0] conf [0:LQN-1];
   integer        li;

   wire [IDXB:0] ld_dist = ld_tag - headc;
   genvar gl, gs;
   generate
      for (gl = 0; gl < LQN; gl = gl + 1) begin : g_lblk
         wire [IDXB:0]   l_dist = l_tag[gl*(IDXB+1) +: IDXB+1] - headc;
         wire [NENT-1:0] oldm;
         for (gs = 0; gs < NENT; gs = gs + 1) begin : g_om
            wire [IDXB-1:0] sd = gs[IDXB-1:0] - head;      // slot distance from the head
            assign oldm[gs] = v[gs] & ({1'b0, sd} < l_dist);
         end
         // An older store whose address has not arrived cannot be compared, so it blocks --
         // conservative and correct, and the same rule the compare form used.
         assign l_block[gl] = l_av[gl] & (|(oldm & (conf[gl] | ~av)));
         // A load can have at most cnt older live stores: a distance beyond the occupancy
         // is a seqno that wrapped, i.e. the defect above in any new clothing.
         always @(posedge clk) if (!reset & l_av[gl] & (l_dist > cnt))
            $fatal(1, "ooo2_sq: load %0d claims %0d older stores with %0d live", gl, l_dist, cnt);
      end
   endgenerate
   // Older-and-live, regardless of overlap. ld_block is the subset that actually conflicts,
   // so `ld_older & ~ld_block` at a load's start is precisely a load reordered past an
   // uncommitted store -- the thing this whole structure exists to allow.
   wire [NENT-1:0] oldv;
   genvar go;
   generate
      for (go = 0; go < NENT; go = go + 1) begin : g_old
         wire [IDXB-1:0] od = go[IDXB-1:0] - head;
         assign oldv[go] = v[go] & ({1'b0, od} < ld_dist);
      end
   endgenerate
   assign ld_older = |oldv;

   always @(posedge clk) begin
      if (reset | flush) begin
         v <= {NENT{1'b0}}; av <= {NENT{1'b0}}; dv <= {NENT{1'b0}};
         ld_v <= {NENT{1'b0}};          // a landing noted for a flushed entry is nobody's
         headc <= {(IDXB+1){1'b0}}; tailc <= {(IDXB+1){1'b0}}; cnt <= {(IDXB+1){1'b0}};
         for (li = 0; li < LQN; li = li + 1) conf[li] <= {NENT{1'b0}};
      end else begin
         // A STORE's address arrives: its COLUMN, against every queued load.
         if (a_v)
            for (li = 0; li < LQN; li = li + 1)
               conf[li][a_idx] <= l_av[li]
                                & ovl_ab(l_pa[li*PAW +: PAW], l_size[li*2 +: 2], a_addr, a_size);
         // A LOAD's address arrives: its ROW, against every live store. Written second, so
         // it wins the one cell both updates can touch -- and it must, because the column
         // update above would compare that cell against addr[a_idx], which is not written
         // until this same edge. The a_v arm here forwards the arriving address instead.
         if (l_fill)
            for (k = 0; k < NENT; k = k + 1)
               conf[l_fill_ix][k] <= (a_v && (k[IDXB-1:0] == a_idx))
                                   ? ovl_ab(l_fill_pa, l_fill_size, a_addr, a_size)
                                   : (av[k] & ovl_ab(l_fill_pa, l_fill_size, addr[k], sz[k]));
         // commit the head
         if (c_v & c_take) begin
            v[head] <= 1'b0; av[head] <= 1'b0; dv[head] <= 1'b0;
            headc <= headc + 1'b1;
         end
         // allocate at the tail
         if (d_alloc & d_ready) begin
            v[tail] <= 1'b1; av[tail] <= 1'b0; dv[tail] <= 1'b0;
            rob[tail] <= d_rob;  dpr[tail] <= d_dpreg;
            tailc <= tailc + 1'b1;
         end
         if ((d_alloc & d_ready) & ~(c_v & c_take)) cnt <= cnt + 1'b1;
         else if (~(d_alloc & d_ready) & (c_v & c_take)) cnt <= cnt - 1'b1;

         // address only. The data operand is NOT captured here -- see the snoop.
         if (a_v) begin
            addr[a_idx] <= a_addr;  sz[a_idx] <= a_size;  av[a_idx] <= 1'b1;
            unc[a_idx]  <= a_unc;
            if (a_data_v & ~dv[a_idx]) begin data[a_idx] <= a_data; dv[a_idx] <= 1'b1; end
         end

         // SNOOP THE WRITEBACK PORTS, ARMED FROM ALLOCATE.
         //
         // Arming at `a_v` instead would be a silent data-loss bug, and it is worth naming
         // because it is not visible from this module alone. A physical register is written
         // back EXACTLY ONCE. Allocate happens at dispatch; the address arrives when the
         // store executes, which is at least one cycle later and unboundedly later behind a
         // PTW. A writeback landing anywhere in that window would find no entry watching for
         // it, and nothing would ever produce it again -- the store would sit in the buffer
         // forever, wedging the ROB head.
         //
         // So `dpr` is captured at ALLOCATE, from the renamer, and the entry watches from
         // the cycle it exists. The `d_alloc` cycle itself is covered by taking the live
         // d_dpreg, because dpr[tail] is written on that same edge.
         //
         // The already-produced case is not the snoop's job: if rs2 was ready when the store
         // read the PRF, `a_data_v` supplies it and no writeback is coming. The two paths are
         // disjoint by construction and `~dv` keeps them so if they ever overlap.
         // The match sets dv NOW and notes the port; the bytes land NEXT cycle from wb_q
         // (below, outside this arm). A flush clears the note with the entry: the slot may
         // be reallocated the cycle after, and a stale landing would overwrite the new
         // entry's data at the same edge its own capture wrote it.
         for (k = 0; k < NENT; k = k + 1) begin
            sn_live = (d_alloc & d_ready & (tail == k[IDXB-1:0])) ? 1'b1     : (v[k] & ~dv[k]);
            sn_dpr  = (d_alloc & d_ready & (tail == k[IDXB-1:0])) ? d_dpreg  : dpr[k];
            ld_v[k] <= 1'b0;
            for (w = 0; w < NWB; w = w + 1)
               if (sn_live & (sn_dpr != {PBITS{1'b0}})
                   & wb_v[w] & (wb_preg[w*PBITS +: PBITS] == sn_dpr)) begin
                  ld_v[k] <= 1'b1;  ld_w[k] <= w[1:0];
                  dv[k]   <= 1'b1;
               end
         end
      end
      wb_q <= wb_data;
      for (k = 0; k < NENT; k = k + 1)
         if (ld_v[k]) data[k] <= wb_q[ld_w[k]*64 +: 64];
   end

   // Invariants (docs/rtl-rules.md A1): anything the design would otherwise drop silently.
   always @(posedge clk) if (!reset) begin
      if (d_alloc & ~d_ready)
         $fatal(1, "ooo2_sq: allocate into a full buffer");
      if (a_v & ~v[a_idx])
         $fatal(1, "ooo2_sq: address written to a slot with no live entry (idx %0d)", a_idx);
      if (a_v & av[a_idx])
         $fatal(1, "ooo2_sq: address written twice to slot %0d", a_idx);
      // A store whose data can never arrive wedges the ROB head forever -- the one failure
      // of this structure that presents as a hang rather than a wrong answer, so it gets an
      // assertion rather than a comment. dpreg 0 is LEGAL (rs2 = x0, or any value already in
      // the PRF): it means no writeback is coming, which is only a bug if a_data_v did not
      // supply the value either.
      if (a_v & ~a_data_v & ~dv[a_idx] & (dpr[a_idx] == {PBITS{1'b0}}))
         $fatal(1, "ooo2_sq: entry %0d has no data and no producer -- it can never commit", a_idx);
      if (c_take & ~c_v)
         $fatal(1, "ooo2_sq: commit taken with no committable head");
      // The landing bypass is exact only if nothing else writes the entry's data in the one
      // cycle the bytes are in flight: the address-time capture is guarded by ~dv, and dv
      // rose with the note, so a collision here is a second producer for one physreg.
      for (li = 0; li < NENT; li = li + 1)
         if (ld_v[li] & a_v & a_data_v & ~dv[li] & (a_idx == li[IDXB-1:0]))
            $fatal(1, "ooo2_sq: entry %0d landing and captured in the same cycle", li);
   end
endmodule
`default_nettype wire
