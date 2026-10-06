`default_nettype none

// smolrv64_sq -- the store buffer.
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
module smolrv64_sq
  #(parameter NENT  = 4,
    parameter IDXB  = 2,               // $clog2(NENT)
    parameter PAW   = 56,
    parameter PBITS = 9,
    parameter ROBB  = 4,
    parameter NWB   = 3,
    parameter LQN   = 4,               // smolrv64_lq's NENT
    parameter LQIB  = 2,               // smolrv64_lq's IDXB
    parameter VAW   = 39,              // Sv39: the VA an entry keeps beside its PA
    parameter SEQW  = 8)               // the op's sequence number (a trap restarts fetch at it)
   (input  wire                  clk,
    input  wire                  reset,

    // ---- allocate: at dispatch, program order, one per cycle ----
    input  wire                  d_alloc,
    input  wire [ROBB-1:0]       d_rob,       // rides with the store; commit is at the head
    input  wire [PBITS-1:0]      d_dpreg,     // renamed rs2 AT DISPATCH -- see the snoop
    input  wire                  d_rdy,       // ...and its value is already in a register file: no writeback is coming
    input  wire [VAW-1:0]        d_pc,        // the store's PC and seq, for a trap from its translation
    input  wire [SEQW-1:0]       d_seq,
    output wire                  d_ready,
    output wire                  d_ready2,    // room for two (the dispatch credit's)
    output wire [IDXB-1:0]       d_idx,
    // An entry with an ADDRESS is a store older than whatever M holds: M translates in
    // program order, and an entry is allocated at dispatch, so the queue can also hold
    // stores YOUNGER than M's op -- entries that cannot get an address until M frees. An
    // M-executed access that must follow every older store (a CBO) waits on this, not on
    // the occupancy: waiting for the occupancy is a deadlock, found on the board 2026-09-04
    // (Ubuntu's clear_page is cbo.zero; the tiny128 kernel never issues one). Rule C5.
    output wire                  av_any,
    output wire                  uf_any,      // an entry whose address has not arrived
    output wire [IDXB-1:0]       uf_idx,      // ...the oldest such, and its sequence number
    output wire [SEQW-1:0]       uf_seq,
    output wire [IDXB:0]         d_tag,       // the STORE-SEQNO a load captures at dispatch: the
                                              // tail COUNTER, one bit wider than the index (below)

    // ---- address (and possibly data) written when the store executes ----
    input  wire                  a_v,
    input  wire [IDXB-1:0]       a_idx,
    input  wire [PAW-1:0]        a_addr,     // ...its PA, when a_tv
    input  wire [VAW-1:0]        a_va,       // the VA, always: the alias test and the walker read it
    input  wire                  a_tv,       // M's lookup translated it (else the walker will)
    input  wire                  a_flt,      // ...or its address alone faults (a_tv is then set)
    input  wire [3:0]            a_fc,
    input  wire [1:0]            a_size,     // 0=B 1=H 2=W 3=D
    input  wire                  a_unc,      // uncached, decided at translate time

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
    // ---- the RELEASE: the ROB says the first uncommitted entry can no longer be undone ----
    // A store is COMMITTED when the ROB's irrevocable pointer reaches it (smolrv64_core computes
    // k_take = kc_v & (kc_rob == rob_irr_idx)); its ROB slot completes then, the head retires
    // it, and the entry stays here until the LSU DRAINS it (c_take). Committed entries survive
    // a flush -- they are architecturally done -- so the flush keeps them and drops from the
    // first uncommitted entry. For a younger load nothing changes: an entry in the queue,
    // committed or not, is a store whose bytes are not in the cache yet.
    output wire                  kc_v,        // the first uncommitted entry exists and has address + data
    // ---- the data read: an entry whose value was in a register file at allocation ----
    output wire                  r_v,         // REGISTERED: read r_preg this cycle...
    output wire [PBITS-1:0]      r_preg,
    input  wire [63:0]           r_data,      // ...and its value, read this cycle
    output wire [ROBB-1:0]       kc_rob,
    output wire [PAW-1:0]        kc_addr,     // ...its PA, for the cosim's retire record
    output wire [63:0]           kc_data,     // ...its value and size, for the cosim's store-data check
    output wire [1:0]            kc_size,
    input  wire                  k_take,      // commit it

    // ---- load disambiguation: a CONFLICT MATRIX, not a compare at issue ----------------
    // The alias test runs where an ADDRESS ARRIVES -- one arriving store against every
    // queued load, or one arriving load against every live store -- and the answer is a
    // FLOP. Issue reads a bit.
    //
    // WHY, and it is the whole reason this exists: computing it at issue put smolrv64_lq's acc
    // pointer at the head of a 36-level, 12x CARRY8 cone -- acc -> sz[acc] -> these 57-bit
    // overlap compares -> ld_block -> x_v -> pt_start -> start_ok -> lsu_done -> m_done ->
    // redirect -> the frontend's target register. WNS -1.698 ns at 166.67 MHz, 18 670
    // failing endpoints, ALL FIVE worst paths sourced at acc. Moving the address off the
    // TRANSLATE path (which is why smolrv64_lq exists) left the compare and its whole downstream
    // tail exactly where they were.
    input  wire [LQN*12-1:0]     l_off,       // the load queue's entries' page offsets, flattened
    input  wire [LQN*2-1:0]      l_size,
    input  wire [LQN*(IDXB+1)-1:0] l_tag,     // each load's captured store-seqno
    input  wire [LQN-1:0]        l_av,
    input  wire                  l_fill,      // a load's address arrives this cycle...
    input  wire [LQIB-1:0]       l_fill_ix,   // ...into this entry
    input  wire [11:0]           l_fill_off,
    input  wire [1:0]            l_fill_size,
    input  wire                  l_fill_flt,  // ...and faults on its address alone (never reaches memory)
    output wire [LQN-1:0]        l_block_unk_q, // per entry, REGISTERED: blocked by an older store whose ADDRESS IS
                                              // UNKNOWN (the rest of l_block_q is a known overlap). Counters only.
    output wire [LQN-1:0]        l_block,     // per entry: an older store aliases it
    output wire [LQN-1:0]        l_older,     // per entry, REGISTERED: an older store is live (see below)
    output wire [LQN-1:0]        l_block_q,   // per entry, REGISTERED: l_block one cycle old, never the less
                                              // conservative -- what smolrv64_lq's candidate select reads (T1 (L) 2)
    // ld_older keeps a query port: it is pointer arithmetic against head, no address and no
    // adder, so it is not part of the cone above.
    input  wire [IDXB:0]         ld_tag,     // this load's captured store-seqno
    output wire                  ld_older,   // an older store is live (aliasing or not) --
                                             // the load is REORDERED past it if it starts

    // ---- the walker: the first uncommitted entry, when it has no translation yet ----
    output wire                  k_v,
    output wire [IDXB-1:0]       k_idx,
    output wire [VAW-1:0]        k_va,
    input  wire                  w_v,        // the walker's answer for entry w_idx
    input  wire [IDXB-1:0]       w_idx,
    input  wire [PAW-1:0]        w_pa,
    input  wire                  w_unc,
    input  wire                  w_flt,
    input  wire [3:0]            w_fc,
    // ---- a translation that faulted, at the first uncommitted entry: it traps at the ROB head ----
    output wire                  f_v,
    output wire [ROBB-1:0]       f_rob,
    output wire [3:0]            f_fc,
    output wire [VAW-1:0]        f_pc,
    output wire [SEQW-1:0]       f_seq,

    output wire [IDXB:0]         occupancy,
    input  wire                  flush,
    // the kill (smolrv64_rob kd): an uncommitted entry whose store is younger than a mispredicted
    // branch dies. They are the youngest, so the tail steps back past them, as the flush cuts
    // at the first uncommitted entry. kmask names every such slot while the kill holds, live or
    // not (nothing allocates meanwhile), so an address a dead store delivers late is dropped too.
    input  wire                  kd_v,
    input  wire [(1 << ROBB)-1:0] kd,
    output wire [NENT-1:0]       kmask);

   reg [NENT-1:0]        v, av, dv;          // live / address known / data known
   reg [PAW-1:0]         addr [0:NENT-1];
   reg [VAW-1:0]         va   [0:NENT-1];
   reg [VAW-1:0]         pc   [0:NENT-1];
   reg [SEQW-1:0]        sqn  [0:NENT-1];
   reg [NENT-1:0]        tv, flt;            // translated / ...and the translation faulted
   reg [3:0]             fc   [0:NENT-1];
   reg [63:0]            data [0:NENT-1];
   reg [1:0]             sz   [0:NENT-1];
   reg [NENT-1:0]        unc;
   reg [PBITS-1:0]       dpr  [0:NENT-1];
   reg [NENT-1:0]        rd;                 // the data is in a register file: the read port fetches it
   reg [ROBB-1:0]        rob  [0:NENT-1];
   reg [NENT-1:0]        cmt;                // committed (released by the ROB), draining
   reg [IDXB:0]          kcc;                // the first uncommitted entry; head <= kc <= tail
   wire [IDXB-1:0]       kc = kcc[IDXB-1:0];
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
   // the kill's entries, and how many
   reg  [NENT-1:0]       kmk, kds;
   reg  [IDXB:0]         k_n;
   integer               kq;
   always @* begin
      k_n = {(IDXB+1){1'b0}};
      for (kq = 0; kq < NENT; kq = kq + 1) begin
         kds[kq] = kd_v & ~cmt[kq] & kd[rob[kq]];
         kmk[kq] = kds[kq] & v[kq];
         k_n = k_n + {{IDXB{1'b0}}, kmk[kq]};
      end
   end
   assign kmask = kds;
   wire uncommitted = (kcc != tailc);           // an uncommitted entry (the kill's check)
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
   reg [2:0]             ld_w [0:NENT-1]; // ...from this port (NWB <= 8)
   integer               li2;
   initial begin ld_v = {NENT{1'b0}}; for (li2 = 0; li2 < NENT; li2 = li2 + 1) ld_w[li2] = 3'd0; end
   initial if (NWB > 8) $fatal(1, "smolrv64_sq: ld_w holds a port index of at most 3 bits");

   initial begin v = {NENT{1'b0}}; av = {NENT{1'b0}}; dv = {NENT{1'b0}}; rd = {NENT{1'b0}}; cmt = {NENT{1'b0}};
                 headc = {(IDXB+1){1'b0}}; tailc = {(IDXB+1){1'b0}}; kcc = {(IDXB+1){1'b0}};
                 cnt = {(IDXB+1){1'b0}}; end

   assign d_ready   = (cnt != NENT[IDXB:0]);
   assign d_ready2  = (cnt <= NENT[IDXB:0] - 2);
   assign d_idx     = tail;
   assign d_tag     = tailc;
   assign av_any    = |(v & av);
   assign uf_any    = |(v & ~av);
   // the data read's pick, from flops: the oldest entry with its address whose value is in a
   // register file and not yet read (every committed entry has its data, so it is at or after
   // the first uncommitted; the data is needed only to commit, which the address precedes)
   reg [IDXB-1:0] rp_i;  integer ri;  reg rp_f;
   always @* begin
      rp_i = kc;  rp_f = 1'b0;
      for (ri = NENT - 1; ri >= 0; ri = ri - 1)
         if (v[kc + ri[IDXB-1:0]] & av[kc + ri[IDXB-1:0]] & rd[kc + ri[IDXB-1:0]]) begin rp_i = kc + ri[IDXB-1:0]; rp_f = 1'b1; end
   end
   wire           rp_v = rp_f;
   wire [IDXB-1:0] rp_idx = rp_i;
   reg            rq_v;  reg [IDXB-1:0] rq_idx;  reg [PBITS-1:0] rq_preg;
   initial rq_v = 1'b0;
   assign r_v    = rq_v;
   assign r_preg = rq_preg;
   // a committed entry has its address, so the oldest unfilled entry is the first one at or
   // after the first uncommitted
   reg [IDXB-1:0] uf_i;  integer ui;
   always @* begin
      uf_i = kc;
      for (ui = NENT - 1; ui >= 0; ui = ui - 1)
         if (v[kc + ui[IDXB-1:0]] & ~av[kc + ui[IDXB-1:0]]) uf_i = kc + ui[IDXB-1:0];
   end
   assign uf_idx    = uf_i;
   assign uf_seq    = sqn[uf_i];
   assign occupancy = cnt;
   // THE WALKER AND THE TRAP READ THE FIRST UNCOMMITTED ENTRY. Stores commit in order, so an
   // untranslated store anywhere else waits behind this one anyway, and a store that faulted
   // never commits: it is this entry when its op reaches the ROB head.
   wire                  kc_live = (kcc != tailc) & v[kc] & av[kc];
   assign k_v   = kc_live & ~tv[kc];
   assign k_idx = kc;
   assign k_va  = va[kc];
   assign f_v   = kc_live & tv[kc] & flt[kc];
   assign f_rob = rob[kc];
   assign f_fc  = fc[kc];
   assign f_pc  = pc[kc];
   assign f_seq = sqn[kc];

   // c_v says the head is READY (address and data present). Committing in program order
   // is the core's job: it takes the entry only when c_rob is the ROB head, so a store
   // reaches memory exactly where the cosim's retire stream expects it.
   // Only a committed store drains -- including one committed THIS cycle at the head, so the
   // release costs no cycle on a store the LSU is waiting for (stbench FWD: 12.75 -> 11.75
   // cycles per store-load pair without it). k_take is a compare of registered values in
   // smolrv64_core, the same shape as the old head-index compare this replaced.
   assign c_v    = v[head] & av[head] & dv[head] & (cmt[head] | (k_take & (kcc == headc)));
   assign kc_v   = (kcc != tailc) & v[kc] & av[kc] & dv[kc] & tv[kc] & ~flt[kc];
   assign kc_rob = rob[kc];
   assign kc_addr= addr[kc];
   // The same landing bypass as c_data: the entry commits in the cycle its bytes are still
   // in wb_q, and data[kc] holds the previous occupant's until the next edge.
   assign kc_data= ld_v[kc] ? wb_q[ld_w[kc]*64 +: 64] : data[kc];
   assign kc_size= sz[kc];
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
   localparam [12:0] SQ_ONE = 13'd1;   // width-matched, not a bare literal

   // Byte ranges [a, a + (1<<size)) overlap unless one ends at or before the other begins.
   // ONE definition, used at both matrix-update sites (rule C1) -- a predicate written twice
   // only has to be updated wrong once. THE RANGES ARE PAGE OFFSETS, VA[11:0] = PA[11:0]: no
   // queued access crosses a page (M faults it), so two in one page compare exactly, two
   // synonyms of one physical page (which share the offset) can never miss each other, and the
   // test needs no translation -- it runs the cycle the VA arrives, whatever the walker does.
   function ovl_ab;
      input [11:0] la; input [1:0] ls;
      input [11:0] sa; input [1:0] ss;
      reg [12:0] l_lo, l_hi, s_lo, s_hi;
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

   // The arriving load's row against every live store, computed ONCE (rule C1): the conf
   // write and the registered block copy below both read it.
   reg [NENT-1:0] fill_row;
   integer        fk;
   always @* begin
      for (fk = 0; fk < NENT; fk = fk + 1)
         fill_row[fk] = (a_v && (fk[IDXB-1:0] == a_idx))
                      ? ovl_ab(l_fill_off, l_fill_size, a_va[11:0], a_size)
                      : (av[fk] & ovl_ab(l_fill_off, l_fill_size, va[fk][11:0], sz[fk]));
   end
   // the stores' address-valid bits as they will stand next cycle (a store only gains one)
   wire [NENT-1:0] av_next = av | (a_v ? ({{(NENT-1){1'b0}}, 1'b1} << a_idx) : {NENT{1'b0}});

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
         // THE BLOCK, TOO, IS A REGISTER PER LOAD (plan item T1 (L) step 2, 2026-09-07). The
         // live l_block above fed the load queue's candidate select and so the LSU's start
         // and, through the shared completion term, the landing wake: gate W6's worst path
         // (u_lq/sqt -> l_block -> x_v -> pt_v -> ... -> u_iq_i2/e_r, 25 levels, -0.556 ns).
         // Once a load's address is in, its block only clears: oldm shrinks, a store's
         // address arriving turns "unknown, blocks" into the overlap, never the reverse. The
         // fill cycle is the one moment the copy would lag the wrong way (l_av rises next
         // cycle), so that cycle computes it from the row being written (fill_row, against
         // av_next). A load is no candidate before its address is in, and the live form
         // stays as the oracle smolrv64_core asserts against.
         reg blk_q;  initial blk_q = 1'b0;
         always @(posedge clk)
            blk_q <= (l_fill & (l_fill_ix == gl[LQIB-1:0]))
                   ? (|(oldm & (fill_row | ~av_next)))
                   : (l_av[gl] & (|(oldm & (conf[gl] | ~av))));
         assign l_block_q[gl] = blk_q;
         reg unk_q;  initial unk_q = 1'b0;      // MEM_ALIAS_UNK: the unknown-address arm alone, a cycle old
         always @(posedge clk) unk_q <= l_av[gl] & (|(oldm & ~av));
         assign l_block_unk_q[gl] = unk_q;
         // THE OLDER-STORE ANSWER IS A REGISTER, PER LOAD (plan item T1 (L), 2026-09-07).
         // The query port below (ld_tag -> ld_older) reads sqt[acc] from the load queue's
         // LUTRAM, subtracts headc and compares NENT distances, and its answer licensed M's
         // early release: gate W5fix's largest family (1,773 endpoints, 21 levels) ran
         // u_lq/sqt -> ld_older -> lq_b_early -> m_done -> iss_ready -> the issue port's
         // payload register. For one load the set of older live stores only SHRINKS -- its
         // tag is fixed at dispatch, a later store is younger by construction, and head and
         // headc only advance -- so a copy one cycle old errs on the conservative side, and
         // a load reaches M no sooner than two cycles after the dispatch that wrote its tag.
         // The live port stays as the oracle (smolrv64_core asserts the copy is never the less
         // conservative of the two).
         reg l_older_q;  initial l_older_q = 1'b0;
         always @(posedge clk) l_older_q <= |oldm;
         assign l_older[gl] = l_older_q;
         // A load can have at most cnt older live stores: a distance beyond the occupancy
         // is a seqno that wrapped, i.e. the defect above in any new clothing.
         always @(posedge clk) if (!reset & l_av[gl] & (l_dist > cnt))
            $fatal(1, "smolrv64_sq: load %0d claims %0d older stores with %0d live", gl, l_dist, cnt);
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
      if (reset) begin
         v <= {NENT{1'b0}}; av <= {NENT{1'b0}}; dv <= {NENT{1'b0}}; cmt <= {NENT{1'b0}};
         ld_v <= {NENT{1'b0}};          // a landing noted for a flushed entry is nobody's
         headc <= {(IDXB+1){1'b0}}; tailc <= {(IDXB+1){1'b0}}; kcc <= {(IDXB+1){1'b0}};
         cnt <= {(IDXB+1){1'b0}};
         for (li = 0; li < LQN; li = li + 1) conf[li] <= {NENT{1'b0}};
      end else begin
         // A STORE's address arrives: its COLUMN, against every queued load.
         if (a_v)
            for (li = 0; li < LQN; li = li + 1)
               conf[li][a_idx] <= l_av[li]
                                & ovl_ab(l_off[li*12 +: 12], l_size[li*2 +: 2], a_va[11:0], a_size);
         // A LOAD's address arrives: its ROW, against every live store. Written second, so
         // it wins the one cell both updates can touch -- and it must, because the column
         // update above would compare that cell against va[a_idx], which is not written
         // until this same edge. The a_v arm (in fill_row) forwards the arriving address.
         if (l_fill) conf[l_fill_ix] <= fill_row;
         // commit the first uncommitted entry (the ROB released it)
         if (k_take) begin
            cmt[kc] <= 1'b1;
            kcc <= kcc + 1'b1;
         end
         // drain the head (the LSU took it) -- AFTER the commit arm: when both name the same
         // entry (released and drained in one cycle) the clear must win
         if (c_v & c_take) begin
            v[head] <= 1'b0; av[head] <= 1'b0; dv[head] <= 1'b0; cmt[head] <= 1'b0;
            headc <= headc + 1'b1;
         end
         // allocate at the tail
         if (d_alloc & d_ready) begin
            v[tail] <= 1'b1; av[tail] <= 1'b0; dv[tail] <= 1'b0; cmt[tail] <= 1'b0;
            rob[tail] <= d_rob;  dpr[tail] <= d_dpreg;  pc[tail] <= d_pc;  sqn[tail] <= d_seq;
            rd[tail] <= d_rdy;
            tailc <= tailc + 1'b1;
         end
         if ((d_alloc & d_ready) & ~(c_v & c_take)) cnt <= cnt + 1'b1;
         else if (~(d_alloc & d_ready) & (c_v & c_take)) cnt <= cnt - 1'b1;
         // A FLUSH KEEPS THE COMMITTED ENTRIES. They are the oldest, so the queue is cut at
         // the first uncommitted one; a drain in this same cycle still counts (the arm above
         // ran). An allocation in the redirect cycle (dispatch is not gated on the redirect
         // since gate V3, 2026-09-05) landed above kcc in the arm above and is cut here.
         if (flush) begin
            for (k = 0; k < NENT; k = k + 1)
               if (~cmt[k]) begin v[k] <= 1'b0; av[k] <= 1'b0; dv[k] <= 1'b0; end
            ld_v  <= {NENT{1'b0}};
            tailc <= kcc;
            cnt   <= (kcc - headc) - {{IDXB{1'b0}}, (c_v & c_take)};
            for (li = 0; li < LQN; li = li + 1) conf[li] <= {NENT{1'b0}};
         end

         // address only. The data comes from the snoop or the data read.
         if (a_v) begin
            addr[a_idx] <= a_addr;  va[a_idx] <= a_va;  sz[a_idx] <= a_size;  av[a_idx] <= 1'b1;
            unc[a_idx]  <= a_unc;  tv[a_idx] <= a_tv;  flt[a_idx] <= a_flt;  fc[a_idx] <= a_fc;
         end
         // the data read: the pick is a register, the value lands the cycle it is read
         if (rq_v & v[rq_idx]) begin data[rq_idx] <= r_data; dv[rq_idx] <= 1'b1; end
         rq_v <= rp_v;  rq_idx <= rp_idx;  rq_preg <= dpr[rp_idx];
         if (rp_v) rd[rp_idx] <= 1'b0;
         if (flush) rq_v <= 1'b0;                      // flush last (rule I11)
         // the walker's answer: the PA and NC bit, or the fault the store traps with at the head
         if (w_v) begin
            addr[w_idx] <= w_pa;  unc[w_idx] <= w_unc;  flt[w_idx] <= w_flt;  fc[w_idx] <= w_fc;
            tv[w_idx]   <= 1'b1;
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
         // The already-produced case is not the snoop's job: a value in a register file at
         // allocation (`d_rdy`) has no writeback coming, and the data read fetches it. The two
         // paths are disjoint by construction, and asserted so.
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
                  ld_v[k] <= 1'b1;  ld_w[k] <= w[2:0];
                  dv[k]   <= 1'b1;
               end
         end
      end
      wb_q <= wb_data;
      for (k = 0; k < NENT; k = k + 1)
         if (ld_v[k]) data[k] <= wb_q[ld_w[k]*64 +: 64];
      // THE KILL, after every other arm (the snoop above included). A flush in its cycle (a trap
      // at the head, which also ends the kill) wins: it cuts at the first uncommitted entry.
      if (!reset & (|kmk) & ~flush) begin
         for (k = 0; k < NENT; k = k + 1)
            if (kmk[k]) begin v[k] <= 1'b0;  av[k] <= 1'b0;  dv[k] <= 1'b0;  ld_v[k] <= 1'b0; end
         // per cell: a whole-row write would drop this cycle's column and row updates for the live
         for (li = 0; li < LQN; li = li + 1)
            for (k = 0; k < NENT; k = k + 1) if (kmk[k]) conf[li][k] <= 1'b0;
         if (rq_v & kmk[rq_idx]) rq_v <= 1'b0;
         tailc <= tailc - k_n;
         cnt   <= cnt - k_n - {{IDXB{1'b0}}, (c_v & c_take)};
      end
   end

   // Invariants (docs/rtl-rules.md A1): anything the design would otherwise drop silently.
   always @(posedge clk) if (!reset) begin
      if (d_alloc & ~d_ready)
         $fatal(1, "smolrv64_sq: allocate into a full buffer");
      if (a_v & ~v[a_idx])
         $fatal(1, "smolrv64_sq: address written to a slot with no live entry (idx %0d)", a_idx);
      if (a_v & av[a_idx])
         $fatal(1, "smolrv64_sq: address written twice to slot %0d", a_idx);
      // A store whose data can never arrive wedges the ROB head forever -- the one failure
      // of this structure that presents as a hang rather than a wrong answer, so it gets an
      // assertion rather than a comment. dpreg 0 is LEGAL (rs2 = x0, or any value already in
      // the PRF): it means no writeback is coming, which is only a bug if a_data_v did not
      // supply the value either.
      if (d_alloc & d_ready & ~d_rdy & (d_dpreg == {PBITS{1'b0}}))
         $fatal(1, "smolrv64_sq: a store allocated with no data and no producer -- it can never commit");
      for (li = 0; li < NENT; li = li + 1)
         if (ld_v[li] & (rd[li] | (rq_v & (rq_idx == li[IDXB-1:0]))))
            $fatal(1, "smolrv64_sq: entry %0d snooped a writeback for a value already in a register file", li);
      if (c_take & ~c_v)
         $fatal(1, "smolrv64_sq: drain taken with no committed, ready head");
      if (k_take & ~kc_v)
         $fatal(1, "smolrv64_sq: commit with no uncommitted, ready entry");
      if (k_take & flush)
         $fatal(1, "smolrv64_sq: a store committed in a redirect cycle (the redirecting op is at the irrevocable point)");
      if (|kmk & d_alloc & ~flush)
         $fatal(1, "smolrv64_sq: a kill in a cycle that allocates");
      if (k_take & kmk[kc])
         $fatal(1, "smolrv64_sq: a dead store committed");
      // the dead entries are the youngest: the ones just behind the tail, contiguously
      for (k = 0; k < NENT; k = k + 1)
         if (kmk[k] & ({1'b0, tail - 1'b1 - k[IDXB-1:0]} >= k_n))
            $fatal(1, "smolrv64_sq: dead entry %0d is not among the %0d youngest", k, k_n);
      if ((kcc - headc) > cnt)
         $fatal(1, "smolrv64_sq: committed count %0d exceeds occupancy %0d", kcc - headc, cnt);
      // The walker answers the entry it asked for, which is still the untranslated first
      // uncommitted one: nothing else moves it (it cannot commit untranslated, and a flush ends
      // the walk with it).
      if (w_v & ~(k_v & (w_idx == kc)))
         $fatal(1, "smolrv64_sq: the walker's answer for entry %0d is not for the untranslated first uncommitted one", w_idx);
      if (w_v & a_v & (a_idx == w_idx))
         $fatal(1, "smolrv64_sq: entry %0d filled and translated in one cycle", w_idx);
      // The alias test compares page offsets, which is exact only because no access that reaches
      // memory crosses a page (one that would faults on its address alone, in its entry).
      if (a_v & ~a_flt & (({1'b0, a_va[11:0]} + (13'd1 << a_size)) > 13'h1000))
         $fatal(1, "smolrv64_sq: store entry %0d at va %h crosses a page", a_idx, a_va);
      if (l_fill & ~l_fill_flt & (({1'b0, l_fill_off} + (13'd1 << l_fill_size)) > 13'h1000))
         $fatal(1, "smolrv64_sq: load entry %0d at offset %h crosses a page", l_fill_ix, l_fill_off);
      if (c_take & ~tv[head])
         $fatal(1, "smolrv64_sq: a store drained without a translation");
   end
endmodule
`default_nettype wire
