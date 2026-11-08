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
    parameter SEQW  = 8,               // the op's sequence number (a trap restarts fetch at it)
    parameter NW    = 1,               // stores allocated a cycle: a dispatch group's row
    parameter RKB   = (NW > 1) ? $clog2(NW) : 1)   // derived: a rank among a row's stores
   (input  wire                  clk,
    input  wire                  reset,

    // ---- allocate: at dispatch, program order, one per cycle ----
    // up to NW a cycle, writer j a dispatch slot ([j*w +: w]), at the tail plus its rank
    input  wire [NW-1:0]         d_alloc,
    input  wire [NW*RKB-1:0]     d_rank,      // writer j's rank among the cycle's stores, in program order
    input  wire [NW*ROBB-1:0]    d_rob,       // rides with the store; commit is at the head
    input  wire [NW*PBITS-1:0]   d_dpreg,     // renamed rs2 AT DISPATCH -- see the snoop
    input  wire [NW-1:0]         d_rdy,       // ...and its value is already in a register file: no writeback is coming
    input  wire [NW*VAW-1:0]     d_pc,        // the store's PC and seq, for a trap from its translation
    input  wire [NW*SEQW-1:0]    d_seq,
    output wire                  d_ready,
    output wire [IDXB:0]         d_free,      // free entries, from flops (the dispatch credit's)
    output wire [IDXB-1:0]       d_idx,
    // An entry with an ADDRESS is a store older than whatever M holds: M translates in
    // program order, and an entry is allocated at dispatch, so the queue can also hold
    // stores YOUNGER than M's op -- entries that cannot get an address until M frees. An
    // M-executed access that must follow every older store (a CBO) waits on this, not on
    // the occupancy: waiting for the occupancy is a deadlock, found on the board 2026-09-04
    // (Ubuntu's clear_page is cbo.zero; the tiny128 kernel never issues one). Rule C5.
    output wire                  av_any,
    output wire                  uf_any,      // an entry that is not filled yet (live, no address in)
    // M's pick: the oldest live entry whose address has ARRIVED (ar: the core holds it for M) but
    // that M has not filled yet. M fills in arrival order, not program order.
    input  wire [NENT-1:0]       ar,
    output wire                  pk_v,
    output wire [IDXB-1:0]       pk_idx,
    output wire [SEQW-1:0]       pk_seq,
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
    input  wire                  a_mem,      // the PA is memory (DRAM or the local SRAM), not a device

    // ---- snoop the writeback ports for the data operand ----
    input  wire [NWB-1:0]        wb_v,
    input  wire [NWB*PBITS-1:0]  wb_preg,
    input  wire [NWB*64-1:0]     wb_data,

    // ---- drain: the oldest committed store not yet handed off, once its data has arrived ----
    // A store is HANDED OFF (c_take) to the LSU and FREED (c_done) when memory has it. A store the
    // LSU's state machine writes is both in one cycle; a plain store to memory goes to the LSU's
    // store register at its handoff and is freed when the D$ takes it, so until then it is still
    // a live entry here and every younger load is ordered against it as before.
    output wire                  c_v,
    output wire [ROBB-1:0]       c_rob,       // core commits only when this IS the ROB head
    output wire [PAW-1:0]        c_addr,
    output wire [63:0]           c_data,
    output wire [1:0]            c_size,
    output wire                  c_unc,
    output wire                  c_fast,      // ...a plain store to memory inside one doubleword: the LSU's store register takes it
    input  wire                  c_take,
    input  wire                  c_done,      // the oldest entry handed off is in memory: free it
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
    // MEMORY-DEPENDENCE SPECULATION: a load whose wait bit (l_wt) is clear goes ahead of older stores
    // whose addresses are unknown; l_acc says a load has reached memory (sent, or landed and still
    // watched), and l_viol flags one that did while an older live store overlaps it: the load read
    // before that store, and must replay.
    input  wire [LQN-1:0]        l_wt,
    input  wire [LQN-1:0]        l_acc,
    output wire [LQN-1:0]        l_unk_q,     // per entry, REGISTERED: an older store's address is unknown
    output wire [LQN-1:0]        l_viol,      // per entry: it went ahead of an older store that overlaps it
    output wire [LQN-1:0]        l_res_c,     // per entry, a pulse: the last unknown older address came in, and it did not overlap
    output wire [LQN-1:0]        l_res_a,     // ...and it did (the load had to wait, or has replayed)
    input  wire                  l_fill,      // a load's address arrives this cycle...
    input  wire [LQIB-1:0]       l_fill_ix,   // ...into this entry
    input  wire [11:0]           l_fill_off,
    input  wire [1:0]            l_fill_size,
    input  wire                  l_fill_flt,  // ...and faults on its address alone (never reaches memory)
    input  wire                  l_fill_go,   // ...and its access started in the same cycle (smolrv64_lsu f_v)
    output wire [LQN-1:0]        l_cancel,    // per entry, of registers: the access it started last cycle
                                              // is void, an older store blocks it (that store's overlap,
                                              // or its unknown address the wait table respects)
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

    // ---- store-to-load forwarding: a held load takes its bytes from the store it waits on ----
    // A page names memory only with its translation regime: {bare, VA[VAW-1:12]}, bare = the access
    // was not translated (M-mode, or satp Bare). A committed store survives a trap's flush, so a
    // user store's VA can meet an M-mode load's PA of the same value; the regime bit tells them apart.
    input  wire [LQN*(VAW-11)-1:0] l_vpn,     // each load's page: a forward needs the store's address,
    input  wire [VAW-12:0]       l_fill_vpn,  // ...not only its page offset (the filling load's)
    input  wire                  a_bare,      // the arriving store's address was not translated
    output wire [LQN-1:0]        l_fwd,       // per entry, REGISTERED: it can take its bytes from a store (below)
    input  wire                  f_go,        // a forward starts this cycle...
    input  wire [LQIB-1:0]       f_lix,       // ...for this load...
    output wire [63:0]           f_data,      // ...and these are its store's bytes as it holds them (its first at bit 0)...
    output wire [2:0]            f_sbo,       // ...which is this byte of the doubleword

    // ---- the walker: the first uncommitted entry, when it has no translation yet ----
    output wire                  k_v,
    output wire [IDXB-1:0]       k_idx,
    output wire [VAW-1:0]        k_va,
    input  wire                  w_v,        // the walker's answer for entry w_idx
    input  wire [IDXB-1:0]       w_idx,
    input  wire [PAW-1:0]        w_pa,
    input  wire                  w_unc,
    input  wire                  w_mem,
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
    input  wire [ROBB-1:0]       kh,
    input  wire [ROBB-1:0]       kb,
    output wire [NENT-1:0]       kmask);

   reg [NENT-1:0]        v, av, dv;          // live / address known / data known
   // THE PA, in two copies of one write each -- M's fill (addr_m) and the walker's answer (addr_w)
   // -- and a table that says which holds an entry's: two writers, so not one array
   (* ram_style = "distributed" *) reg [PAW-1:0] addr_m [0:NENT-1];
   (* ram_style = "distributed" *) reg [PAW-1:0] addr_w [0:NENT-1];
   reg [NENT-1:0]        addr_lv;            // entry k's PA is the walker's
   reg [VAW-1:0]         va   [0:NENT-1];
   reg [NENT-1:0]        tv, flt;            // translated / ...and the translation faulted
   reg [3:0]             fc   [0:NENT-1];
   (* ram_style = "distributed" *) reg [63:0] data [0:NENT-1];   // one write a cycle: the landing below
   reg [1:0]             sz   [0:NENT-1];
   reg [NENT-1:0]        unc;
   reg [NENT-1:0]        mem, dw;            // the PA is memory / the access stays inside one doubleword
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
   reg [IDXB:0]          hoc;                // the next entry to hand off; head <= ho <= kc
   wire [IDXB-1:0]       ho = hoc[IDXB-1:0];
   wire [IDXB:0]         ho_n = hoc - headc, kc_n = kcc - headc;   // handed off and not freed / committed
   reg [IDXB:0]          cnt;
   integer               k, aw;
   // the kill's entries, and how many
`include "smolrv64_kill.vh"
   reg  [NENT-1:0]       kmk, kds;
   reg  [IDXB:0]         k_n;
   integer               kq;
   always @* begin
      k_n = {(IDXB+1){1'b0}};
      for (kq = 0; kq < NENT; kq = kq + 1) begin
         kds[kq] = ~cmt[kq] & kdead(kd_v, kh, kb, rob[kq]);
         kmk[kq] = kds[kq] & v[kq];
         k_n = k_n + {{IDXB{1'b0}}, kmk[kq]};
      end
   end
   assign kmask = kds;
   wire uncommitted = (kcc != tailc);           // an uncommitted entry (the kill's check)

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
   // THE DATA READ LANDS THE SAME WAY: the value read from a register file this cycle is wb_q's
   // slot NWB the cycle after, so the read's 64 bits reach one register, not every entry.
   // ONE ENTRY'S BYTES LAND A CYCLE, so the data array is distributed RAM with one write (the drain,
   // the commit and a forward read it): the data read's when one lands, else the oldest entry whose
   // register a writeback names. An entry the landing leaves out is fetched by the data read, as a
   // value already in a register file at allocation is -- its value is in one now.
   reg [(NWB+1)*64-1:0]  wb_q;            // last cycle's writeback data, and the data read's above it
   reg                   ln_v;            // an entry's bytes land from wb_q this cycle...
   reg [IDXB-1:0]        ln_idx;          // ...this entry's...
   reg [2:0]             ln_w;            // ...from this slot (a writeback port, or NWB: the data read)
   initial begin ln_v = 1'b0; ln_idx = {IDXB{1'b0}}; ln_w = 3'd0; end
   initial if (NWB > 7) $fatal(1, "smolrv64_sq: ln_w holds a slot index of at most 3 bits, NWB=%0d ports and the data read", NWB);
   wire [63:0]           ln_data = wb_q[ln_w*64 +: 64];
   // THE SNOOP, ARMED FROM ALLOCATE (see the writeback loop): entry k's register is written back this
   // cycle, on one port at most -- a register is written once. The entries allocated this cycle
   // (the tail on, whose dpr is written at this edge) match on their own, al_m: they are the
   // youngest, so the landing takes one of them last (al_k), and the dispatch's renamed registers
   // reach no other entry's choice.
   reg  [NENT-1:0]       sn_m;
   reg  [2:0]            sn_w [0:NENT-1];
   reg  [NW-1:0]         al_m;
   reg  [2:0]            al_w [0:NW-1];
   reg  [IDXB-1:0]       al_e [0:NW-1];       // writer j's entry: the tail plus its rank
   reg  [RKB-1:0]        al_k;
   reg  [IDXB:0]         n_alloc;
   integer               sk, sw, aj;
   always @* begin
      for (sk = 0; sk < NENT; sk = sk + 1) begin
         sn_m[sk] = 1'b0;  sn_w[sk] = 3'd0;
         for (sw = 0; sw < NWB; sw = sw + 1)
            if (v[sk] & ~dv[sk] & (dpr[sk] != {PBITS{1'b0}}) & wb_v[sw] & (wb_preg[sw*PBITS +: PBITS] == dpr[sk])) begin
               sn_m[sk] = 1'b1;  sn_w[sk] = sw[2:0];
            end
      end
      n_alloc = {(IDXB+1){1'b0}};  al_k = {RKB{1'b0}};
      for (aj = NW - 1; aj >= 0; aj = aj - 1) begin
         al_e[aj] = tail + {{(IDXB-RKB){1'b0}}, d_rank[aj*RKB +: RKB]};
         n_alloc = n_alloc + {{IDXB{1'b0}}, d_alloc[aj]};
         al_m[aj] = 1'b0;  al_w[aj] = 3'd0;
         for (sw = 0; sw < NWB; sw = sw + 1)
            if (d_alloc[aj] & (d_dpreg[aj*PBITS +: PBITS] != {PBITS{1'b0}}) & wb_v[sw]
                & (wb_preg[sw*PBITS +: PBITS] == d_dpreg[aj*PBITS +: PBITS])) begin
               al_m[aj] = 1'b1;  al_w[aj] = sw[2:0];
            end
         if (al_m[aj]) al_k = aj[RKB-1:0];
      end
   end
   wire                  al_any = |al_m;
   // THE ROW'S FIELDS, A BANK AT A TIME: its stores take consecutive entries from the tail, so they
   // land in distinct banks (an entry's low ABL bits); a bank's writer is muxed once (one-hot,
   // AND-OR) and its entry -- the one at tail + ((bank - tail) mod banks) -- only enabled
   localparam integer ANB = 1 << RKB;   localparam integer ABL = RKB;
   reg  [ANB-1:0]        ab_w;
   reg  [IDXB-1:0]       ab_e  [0:ANB-1];
   reg  [ROBB-1:0]       ab_rob [0:ANB-1];  reg [PBITS-1:0] ab_dpr [0:ANB-1];  reg ab_rd [0:ANB-1];
   integer               abk, abj;
   always @* for (abk = 0; abk < ANB; abk = abk + 1) begin : abank
      reg [ABL-1:0] ob;
      ob = abk[ABL-1:0] - tail[ABL-1:0];
      ab_e[abk] = tail + {{(IDXB-ABL){1'b0}}, ob};
      ab_w[abk] = 1'b0;  ab_rob[abk] = {ROBB{1'b0}};  ab_dpr[abk] = {PBITS{1'b0}};  ab_rd[abk] = 1'b0;
      for (abj = 0; abj < NW; abj = abj + 1)
         if (d_alloc[abj] & (d_rank[abj*RKB +: RKB] == ob)) begin
            ab_w[abk]   = 1'b1;
            ab_rob[abk] = ab_rob[abk] | d_rob[abj*ROBB +: ROBB];
            ab_dpr[abk] = ab_dpr[abk] | d_dpreg[abj*PBITS +: PBITS];
            ab_rd[abk]  = ab_rd[abk]  | d_rdy[abj];
         end
   end
   always @(posedge clk) if (!reset) for (abk = 0; abk < NW; abk = abk + 1) for (abj = abk + 1; abj < NW; abj = abj + 1)
      if (d_alloc[abk] & d_alloc[abj] & (d_rank[abk*RKB +: RKB] == d_rank[abj*RKB +: RKB]))
         $fatal(1, "smolrv64_sq: stores %0d and %0d of a row share rank %0d", abk, abj, d_rank[abk*RKB +: RKB]);

   initial begin v = {NENT{1'b0}}; av = {NENT{1'b0}}; dv = {NENT{1'b0}}; rd = {NENT{1'b0}}; cmt = {NENT{1'b0}};
                 headc = {(IDXB+1){1'b0}}; tailc = {(IDXB+1){1'b0}}; kcc = {(IDXB+1){1'b0}};
                 hoc = {(IDXB+1){1'b0}}; cnt = {(IDXB+1){1'b0}}; end

   assign d_ready   = (cnt != NENT[IDXB:0]);
   assign d_free    = NENT[IDXB:0] - cnt;
   assign d_idx     = tail;
   assign d_tag     = tailc;
   assign av_any    = |(v & av);
   assign uf_any    = |(v & ~av);
`include "smolrv64_ring.vh"
   // the data read's pick, from flops: the oldest entry with its address whose value is in a
   // register file and not yet read (every committed entry has its data, so it is at or after
   // the first uncommitted; the data is needed only to commit, which the address precedes)
   wire [NENT-1:0] rp_m   = v & av & rd;
   wire            rp_v   = |rp_m;
   wire [IDXB-1:0] rp_idx = ring_oldest(rp_m, kc);
   reg            rq_v;  reg [IDXB-1:0] rq_idx;  reg [PBITS-1:0] rq_preg;
   initial rq_v = 1'b0;
   assign r_v    = rq_v;
   assign r_preg = rq_preg;
   // the landing: the data read's (its value is in wb_q for this one cycle), else the oldest snooped,
   // else the allocation's
   wire            rd_ln  = rq_v & v[rq_idx];
   wire            sn_any = |sn_m;
   wire [IDXB-1:0] sn_k   = ring_oldest(sn_m, head);
   // the oldest arrived-and-unfilled entry: the live region runs from the head
   wire [NENT-1:0]   pk_m = v & ~av & ar;
   // M'S PICK IS MADE A CYCLE AHEAD: the oldest entry whose address waits, registered (pk_q), and
   // offered while that entry's address still waits -- the age pick stays out of M's take. One
   // taken last cycle waits no longer: back to back, M takes a waiting address every other cycle.
   reg  [IDXB-1:0] pk_q;
   initial pk_q = {IDXB{1'b0}};
   always @(posedge clk) pk_q <= ring_oldest(pk_m, head);
   assign pk_v      = pk_m[pk_q];
   assign pk_idx    = pk_q;

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
   // the store's PC and seq, read at one or two entries and written by a row: banks of the
   // allocation's (a row's stores take consecutive entries, one bank each), a copy per read
   localparam integer SNB = (NW > 1) ? (1 << $clog2(NW)) : 2;
   wire [NW*IDXB-1:0] al_wp;
   genvar gaw;
   generate for (gaw = 0; gaw < NW; gaw = gaw + 1) begin: g_alwp
      assign al_wp[gaw*IDXB +: IDXB] = al_e[gaw];
   end endgenerate
   smolrv64_bankq #(.W(VAW), .DEPTH(NENT), .NW(NW), .NR(1), .NB(SNB), .PICK(1), .AB(IDXB)) u_pc
     (.clk(clk), .wclk(clk), .reset(reset), .wv(d_alloc), .wp(al_wp), .wd(d_pc), .rp(kc), .ro({$clog2(SNB){1'b0}}), .rd(f_pc));
   smolrv64_bankq #(.W(SEQW), .DEPTH(NENT), .NW(NW), .NR(1), .NB(SNB), .PICK(1), .AB(IDXB)) u_sqn_kc
     (.clk(clk), .wclk(clk), .reset(reset), .wv(d_alloc), .wp(al_wp), .wd(d_seq), .rp(kc), .ro({$clog2(SNB){1'b0}}), .rd(f_seq));
   smolrv64_bankq #(.W(SEQW), .DEPTH(NENT), .NW(NW), .NR(1), .NB(SNB), .PICK(1), .AB(IDXB)) u_sqn_pk
     (.clk(clk), .wclk(clk), .reset(reset), .wv(d_alloc), .wp(al_wp), .wd(d_seq), .rp(pk_idx), .ro({$clog2(SNB){1'b0}}), .rd(pk_seq));

   // c_v says the next entry to hand off is READY (address and data present). Committing in program order
   // is the core's job: it takes the entry only when c_rob is the ROB head, so a store
   // reaches memory exactly where the cosim's retire stream expects it.
   // Only a committed store drains, from the cycle after its commit (cmt, a register): the
   // commit's compare against the ROB's irrevocable pointer stays out of the LSU's start.
   // ONE STORE BETWEEN HANDOFF AND MEMORY: the next is handed off when none is outstanding or the
   // outstanding one is done this cycle (c_done), whichever path it took -- the store register
   // chains one a cycle as before, and a store behind one the LSU's state machine writes waits for
   // its last write, so stores reach memory in program order and stay ordered against loads.
   assign c_v    = v[ho] & av[ho] & dv[ho] & cmt[ho]
                 & ((ho_n == {(IDXB+1){1'b0}}) | c_done);
   assign kc_v   = (kcc != tailc) & v[kc] & av[kc] & dv[kc] & tv[kc] & ~flt[kc];
   assign kc_rob = rob[kc];
   wire [PAW-1:0] kc_am = addr_m[kc], kc_aw = addr_w[kc];
   assign kc_addr= addr_lv[kc] ? kc_aw : kc_am;
   // The same landing bypass as c_data: the entry commits in the cycle its bytes are still
   // in wb_q, and data[kc] holds the previous occupant's until the next edge.
   assign kc_data= (ln_v & (ln_idx == kc)) ? ln_data : data[kc];
   assign kc_size= sz[kc];
   assign c_rob  = rob[ho];
   wire [PAW-1:0] hd_am = addr_m[ho], hd_aw = addr_w[ho];
   assign c_addr = addr_lv[ho] ? hd_aw : hd_am;
   // the landing bypass: the cycle dv rose, the bytes are still in wb_q
   assign c_data = (ln_v & (ln_idx == ho)) ? ln_data : data[ho];
   assign c_size = sz[ho];
   assign c_unc  = unc[ho];
   assign c_fast = mem[ho] & ~unc[ho] & dw[ho];
   // THE FORWARD'S READ, in its start cycle: the load's store, named by the flag's own register (so
   // the store the flag was computed for), through the same landing bypass as c_data, placed at its
   // doubleword offset as the D$ write places it (smolrv64_lsu's mem_wdata).
   wire [IDXB-1:0] f_s  = fws_w[f_lix*IDXB +: IDXB];
   wire [63:0]     f_dw = (ln_v & (ln_idx == f_s)) ? ln_data : data[f_s];
   assign f_data = f_dw;
   assign f_sbo  = va[f_s][2:0];
   always @(posedge clk)
      if (!reset && f_go && !(l_fwd[f_lix] && v[f_s] && dv[f_s] && conf[f_lix][f_s] && cov[f_lix][f_s]))
         $fatal(1, "smolrv64_sq: forward for load %0d from store %0d, not its forwarding store (fwd=%b v=%b dv=%b conf=%b cov=%b)",
                f_lix, f_s, l_fwd[f_lix], v[f_s], dv[f_s], conf[f_lix][f_s], cov[f_lix][f_s]);

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
   // two addresses arrives, so nothing about it is on the issue path. Never cleared: every read
   // is the load's (l_av) against its OLDER live stores (oldm), and those cells are exact -- the
   // load's row is rewritten when its address arrives (0 for a store without one), the column when
   // the store's does. A cell left by a killed or drained store is masked: a store reusing the slot
   // is younger than every load whose row predates it. Checked below.
   reg [NENT-1:0] conf [0:LQN-1];
   integer        li;

   // FORWARDING. cov[i][g]: store g holds every byte of load i -- the same VA page (the offsets
   // alone also match other pages), its bytes covering the load's, and itself inside one
   // doubleword. Written at exactly the sites conf is and read only through conf, so a stale cell
   // behind a clear conf cell is never seen -- and so is the doubleword: two byte ranges that
   // overlap (conf), each inside one doubleword (l_hi <= s_hi <= 8), lie in the same one. VAs compare because everything in the queues shares one
   // address space: an satp write and sfence.vma drain them first. A synonym does not forward.
   function cov_ab;
      input [VAW-12:0] lp; input [11:0] la; input [1:0] ls;
      input [VAW-12:0] sp; input [11:0] sa; input [1:0] ss;
      reg [3:0] l_lo, l_hi, s_lo, s_hi;
      begin
         l_lo = {1'b0, la[2:0]};  l_hi = l_lo + (4'd1 << ls);
         s_lo = {1'b0, sa[2:0]};  s_hi = s_lo + (4'd1 << ss);
         cov_ab = (lp == sp) & (s_lo <= l_lo) & (l_hi <= s_hi) & (s_hi <= 4'd8);
      end
   endfunction
   reg [NENT-1:0] cov [0:LQN-1];
   reg [NENT-1:0] bare;                     // the store's address was not translated (see a_bare)
   reg [NENT-1:0] fill_cov;
   integer        fc2;
   always @* for (fc2 = 0; fc2 < NENT; fc2 = fc2 + 1)
      fill_cov[fc2] = (a_v && (fc2[IDXB-1:0] == a_idx))
                    ? cov_ab(l_fill_vpn, l_fill_off, l_fill_size, {a_bare, a_va[VAW-1:12]}, a_va[11:0], a_size)
                    : cov_ab(l_fill_vpn, l_fill_off, l_fill_size, {bare[fc2], va[fc2][VAW-1:12]}, va[fc2][11:0], sz[fc2]);
   // a one-hot store mask's index
   function [IDXB-1:0] enc1(input [NENT-1:0] m);
      integer i;
      begin enc1 = {IDXB{1'b0}}; for (i = 0; i < NENT; i = i + 1) if (m[i]) enc1 = enc1 | i[IDXB-1:0]; end
   endfunction
   wire [LQN*IDXB-1:0] fws_w;               // each load's forwarding store (valid with l_fwd)

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
`ifndef SYNTHESIS
   wire [LQN-1:0] dep_rc, dep_ra;  wire [LQN*16-1:0] dep_cy;
   reg  [63:0]    sp_clear, sp_alias, sp_clear_cyc, sp_alias_cyc;
   integer        dpi;
   initial begin sp_clear = 0; sp_alias = 0; sp_clear_cyc = 0; sp_alias_cyc = 0; end
   always @(posedge clk) for (dpi = 0; dpi < LQN; dpi = dpi + 1) begin
      if (dep_rc[dpi]) begin sp_clear = sp_clear + 1; sp_clear_cyc = sp_clear_cyc + dep_cy[dpi*16 +: 16]; end
      if (dep_ra[dpi]) begin sp_alias = sp_alias + 1; sp_alias_cyc = sp_alias_cyc + dep_cy[dpi*16 +: 16]; end
   end
`endif
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
         assign l_block[gl] = l_av[gl] & (|(oldm & (conf[gl] | (~av & {NENT{l_wt[gl]}}))));
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
         // a store's address arriving this cycle against this load: the block RISES (an unknown
         // address becoming a known overlap) in the same edge the matrix's bit does
         wire arr_ov = l_av[gl] & a_v & oldm[a_idx]
                     & ovl_ab(l_off[gl*12 +: 12], l_size[gl*2 +: 2], a_va[11:0], a_size);
         reg blk_q;  initial blk_q = 1'b0;
         always @(posedge clk)
            blk_q <= (l_fill & (l_fill_ix == gl[LQIB-1:0]))
                   ? (|(oldm & (fill_row | (~av_next & {NENT{l_wt[gl]}}))))
                   : ((l_av[gl] & (|(oldm & (conf[gl] | (~av & {NENT{l_wt[gl]}}))))) | arr_ov);
         assign l_block_q[gl] = blk_q;
         // FORWARDABLE: exactly one older live store overlaps the load, it holds all the load's bytes
         // (cov), its data is in, it is translated, cached and not faulting, and no older store's
         // address is unknown unless the load may go ahead of unknown addresses (its wait bit
         // clear): a forwarded load counts as having reached memory, so a store whose address then
         // arrives and overlaps it flags it for replay -- its forwarding store aside (fd_*, below). A
         // register, like the block; cleared in the edge its store drains, so the slot it names never
         // outlives the store (and is read in the forward's start cycle, see f_data).
`ifndef SYNTHESIS
         integer ck;
         always @(posedge clk) if (!reset & l_av[gl])
            for (ck = 0; ck < NENT; ck = ck + 1)
               if (oldm[ck] && conf[gl][ck] != (av[ck] & ovl_ab(l_off[gl*12 +: 12], l_size[gl*2 +: 2], va[ck][11:0], sz[ck])))
                  $fatal(1, "smolrv64_sq: load %0d's overlap with older store %0d reads %b", gl, ck, conf[gl][ck]);
`endif
         wire [NENT-1:0] kov = oldm & conf[gl];
         wire            k1  = (kov != {NENT{1'b0}}) & ((kov & (kov - 1'b1)) == {NENT{1'b0}});
         wire            fwn = l_av[gl] & k1 & (|(kov & cov[gl] & dv & tv & ~flt & ~unc))
                             & ~(l_wt[gl] & (|(oldm & ~av))) & ~(c_done & (enc1(kov) == head));   // k1: kov is one-hot
         reg             fw_q;  reg [IDXB-1:0] fws_q;
         initial begin fw_q = 1'b0; fws_q = {IDXB{1'b0}}; end
         always @(posedge clk) begin
            fw_q  <= ~reset & ~flush & fwn;
            fws_q <= enc1(kov);
         end
         assign l_fwd[gl] = fw_q;
         assign fws_w[gl*IDXB +: IDXB] = fws_q;
         // THE FORWARDING STORE, while the load stands forwarded: latched as the forward starts and
         // held while the load counts as having reached memory (a replay or the entry's end drops it).
         // Its overlap is the one that does not overtake the load: the violation check and the
         // training leave it out, and the overlap matrix itself is never written for it.
         reg             fd_v;  reg [IDXB-1:0] fd_s;
         initial begin fd_v = 1'b0; fd_s = {IDXB{1'b0}}; end
         wire            fd_go = f_go & (f_lix == gl[LQIB-1:0]);
         always @(posedge clk) begin
            fd_v <= ~reset & (fd_go | (fd_v & l_acc[gl]));
            if (fd_go) fd_s <= f_s;
         end
         wire [NENT-1:0] fdm = fd_v ? ({{(NENT-1){1'b0}}, 1'b1} << fd_s) : {NENT{1'b0}};
         wire [NENT-1:0] ovr = oldm & conf[gl] & ~fdm;   // older stores overlapping it, its forwarding store aside
         reg unk_q;  initial unk_q = 1'b0;      // MEM_ALIAS_UNK: the unknown-address arm alone, a cycle old
         always @(posedge clk) unk_q <= l_av[gl] & (|(oldm & ~av));
         assign l_block_unk_q[gl] = unk_q;
         assign l_unk_q[gl]       = unk_q;
         // ...evaluated in two places that together leave no cycle uncovered: the matrix's bit (the
         // store's address arrived earlier, or the load reached memory after it), and the store
         // address ARRIVING this cycle against a load that already has (the flag is then a flop in
         // the cycle the store's address is visible, before the store can commit and retire)
         // THE START THE BLOCK CANCELS. M starts a load's access before the alias test can answer
         // (its row is this cycle's whole path); the block that row writes says, a cycle later,
         // whether the access stands. A cancelled load never read memory, so it is no violation.
         reg go_q;  initial go_q = 1'b0;
         always @(posedge clk) go_q <= ~reset & l_fill_go & (l_fill_ix == gl[LQIB-1:0]);
         assign l_cancel[gl]      = go_q & blk_q;
         assign l_viol[gl]        = l_av[gl] & l_acc[gl] & ~l_cancel[gl] & ((|ovr) | arr_ov);
         // RESOLUTION: the cycle the last older store with an unknown address gets one, the load
         // learns whether it overlaps an older store. A pulse (registered), one of two: it did not
         // (l_res_c) or it did (l_res_a) -- the training signal of the wait table, whether the load
         // waited for that address or went ahead of it.
         wire unk_now = l_av[gl] & (|(oldm & ~av));
         reg  was_unk;  reg r_c, r_a;
         initial begin was_unk = 1'b0; r_c = 1'b0; r_a = 1'b0; end
         always @(posedge clk) begin
            r_c <= 1'b0;  r_a <= 1'b0;
            if (!l_av[gl] || reset) was_unk <= 1'b0;
            else begin
               was_unk <= unk_now;
               if (!unk_now && was_unk) begin r_c <= ~(|ovr);  r_a <= |ovr; end
            end
         end
         assign l_res_c[gl] = r_c;  assign l_res_a[gl] = r_a;
`ifndef SYNTHESIS
         reg [15:0] ubc;  reg [15:0] r_cy;
         initial begin ubc = 16'd0; r_cy = 16'd0; end
         always @(posedge clk) begin
            if (!l_av[gl] || reset) ubc <= 16'd0;
            else if (unk_now) ubc <= ubc + 16'd1;
            else if (was_unk) begin r_cy <= ubc;  ubc <= 16'd0; end
         end
         assign dep_rc[gl] = r_c;  assign dep_ra[gl] = r_a;  assign dep_cy[gl*16 +: 16] = r_cy;
`endif
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
         ln_v <= 1'b0;
         headc <= {(IDXB+1){1'b0}}; tailc <= {(IDXB+1){1'b0}}; kcc <= {(IDXB+1){1'b0}};
         hoc <= {(IDXB+1){1'b0}}; cnt <= {(IDXB+1){1'b0}};
      end else begin
         // A STORE's address arrives: its COLUMN, against every queued load.
         if (a_v)
            for (li = 0; li < LQN; li = li + 1) begin
               conf[li][a_idx] <= l_av[li]
                                & ovl_ab(l_off[li*12 +: 12], l_size[li*2 +: 2], a_va[11:0], a_size);
               cov[li][a_idx]  <= cov_ab(l_vpn[li*(VAW-11) +: VAW-11], l_off[li*12 +: 12], l_size[li*2 +: 2],
                                         {a_bare, a_va[VAW-1:12]}, a_va[11:0], a_size);
            end
         // A LOAD's address arrives: its ROW, against every live store. Written second, so
         // it wins the one cell both updates can touch -- and it must, because the column
         // update above would compare that cell against va[a_idx], which is not written
         // until this same edge. The a_v arm (in fill_row) forwards the arriving address.
         if (l_fill) begin conf[l_fill_ix] <= fill_row;  cov[l_fill_ix] <= fill_cov; end
         // commit the first uncommitted entry (the ROB released it)
         if (k_take) begin
            cmt[kc] <= 1'b1;
            kcc <= kcc + 1'b1;
         end
         // hand off, and free the head (memory has it) -- AFTER the commit arm: when both name
         // the same entry (released and drained in one cycle) the clear must win
         if (c_v & c_take) hoc <= hoc + 1'b1;
         if (c_done) begin
            v[head] <= 1'b0; av[head] <= 1'b0; dv[head] <= 1'b0; cmt[head] <= 1'b0;
            headc <= headc + 1'b1;
         end
         // allocate at the tail: a row's stores at the tail plus their ranks
         for (aw = 0; aw < ANB; aw = aw + 1) if (ab_w[aw]) begin
            v[ab_e[aw]] <= 1'b1; av[ab_e[aw]] <= 1'b0; dv[ab_e[aw]] <= 1'b0; cmt[ab_e[aw]] <= 1'b0;
            rob[ab_e[aw]] <= ab_rob[aw];  dpr[ab_e[aw]] <= ab_dpr[aw];  rd[ab_e[aw]] <= ab_rd[aw];
         end
         tailc <= tailc + n_alloc;
         cnt <= cnt + n_alloc - {{IDXB{1'b0}}, c_done};
         // A FLUSH KEEPS THE COMMITTED ENTRIES. They are the oldest, so the queue is cut at
         // the first uncommitted one; a drain in this same cycle still counts (the arm above
         // ran). An allocation in the redirect cycle (dispatch is not gated on the redirect
         // since gate V3, 2026-09-05) landed above kcc in the arm above and is cut here.
         if (flush) begin
            for (k = 0; k < NENT; k = k + 1)
               if (~cmt[k]) begin v[k] <= 1'b0; av[k] <= 1'b0; dv[k] <= 1'b0; end
            tailc <= kcc;
            cnt   <= (kcc - headc) - {{IDXB{1'b0}}, c_done};
         end

         // address only. The data comes from the snoop or the data read.
         if (a_v) begin
            addr_lv[a_idx] <= 1'b0;  va[a_idx] <= a_va;  sz[a_idx] <= a_size;  av[a_idx] <= 1'b1;  bare[a_idx] <= a_bare;
            unc[a_idx]  <= a_unc;  tv[a_idx] <= a_tv;  flt[a_idx] <= a_flt;  fc[a_idx] <= a_fc;
            mem[a_idx]  <= a_mem;  dw[a_idx] <= ({1'b0, a_va[2:0]} + (4'd1 << a_size)) <= 4'd8;
         end
         // the data read: the pick is a register, the value is in the cycle it is read (dv) and
         // lands from wb_q the cycle after (the landing, below)
         if (rq_v & v[rq_idx]) dv[rq_idx] <= 1'b1;
         rq_v <= rp_v;  rq_idx <= rp_idx;  rq_preg <= dpr[rp_idx];
         if (rp_v) rd[rp_idx] <= 1'b0;
         if (flush) rq_v <= 1'b0;                      // flush last (rule I11)
         // the walker's answer: the PA and NC bit, or the fault the store traps with at the head
         if (w_v) begin
            addr_lv[w_idx] <= 1'b1;  unc[w_idx] <= w_unc;  flt[w_idx] <= w_flt;  fc[w_idx] <= w_fc;  mem[w_idx] <= w_mem;
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
         // The landing's entry gets dv NOW and its bytes NEXT cycle from wb_q (the data array's
         // write, below); an entry it leaves out is read from the register file its value is now
         // in. A landing for an entry a flush or the kill ends goes to a slot nothing reads: the
         // slot is allocated again no sooner than the cycle it lands, and its new entry's bytes
         // land later still.
         ln_v   <= rd_ln | sn_any | al_any;
         ln_idx <= rd_ln ? rq_idx : sn_any ? sn_k : al_e[al_k];
         ln_w   <= rd_ln ? NWB[2:0] : sn_any ? sn_w[sn_k] : al_w[al_k];
         for (k = 0; k < NENT; k = k + 1)
            if (sn_m[k]) begin
               if (~rd_ln & (sn_k == k[IDXB-1:0])) dv[k] <= 1'b1;
               else                                 rd[k] <= 1'b1;
            end
         for (aw = 0; aw < NW; aw = aw + 1) if (al_m[aw]) begin
            if (~rd_ln & ~sn_any & (al_k == aw[RKB-1:0])) dv[al_e[aw]] <= 1'b1;
            else                                          rd[al_e[aw]] <= 1'b1;
         end
      end
      wb_q <= {r_data, wb_data};
      // THE KILL, after every other arm (the snoop above included). A flush in its cycle (a trap
      // at the head, which also ends the kill) wins: it cuts at the first uncommitted entry.
      if (!reset & (|kmk) & ~flush) begin
         for (k = 0; k < NENT; k = k + 1)
            if (kmk[k]) begin v[k] <= 1'b0;  av[k] <= 1'b0;  dv[k] <= 1'b0; end
         // the read picked THIS cycle (it lands next) is cancelled when its entry dies; the one landing
         // now (rq_idx) needs nothing, its entry dies with it. Picks come in the order addresses arrive,
         // so a dying entry need not be the one just picked.
         if (rp_v & kmk[rp_idx]) rq_v <= 1'b0;
         tailc <= tailc - k_n;
         cnt   <= cnt - k_n - {{IDXB{1'b0}}, c_done};
      end
   end

   // each PA copy's one write
   always @(posedge clk) if (!reset & a_v) addr_m[a_idx] <= a_addr;
   always @(posedge clk) if (!reset & w_v) addr_w[w_idx] <= w_pa;
   // the data array's one write: the landing
   always @(posedge clk) if (ln_v) data[ln_idx] <= ln_data;
`ifndef SYNTHESIS
   // SQ-SIM: the landings, and the entries one left out (the data read fetched them instead)
   reg [63:0] ln_n, ln_out;
   integer    lno;
   initial begin ln_n = 0; ln_out = 0; end
   always @(posedge clk) if (!reset) begin
      if (rd_ln | sn_any | al_any) ln_n = ln_n + 1;
      for (lno = 0; lno < NENT; lno = lno + 1)
         if (sn_m[lno] & (rd_ln | (sn_k != lno[IDXB-1:0]))) ln_out = ln_out + 1;
      for (lno = 0; lno < NW; lno = lno + 1)
         if (al_m[lno] & (rd_ln | sn_any | (al_k != lno[RKB-1:0]))) ln_out = ln_out + 1;
   end
`endif

   // Invariants (docs/rtl-rules.md A1): anything the design would otherwise drop silently.
   always @(posedge clk) if (!reset) begin
      if ({1'b0, cnt} + {1'b0, n_alloc} > {1'b0, NENT[IDXB:0]})
         $fatal(1, "smolrv64_sq: %0d stores allocate into a buffer holding %0d of %0d", n_alloc, cnt, NENT);
      if (a_v & ~v[a_idx])
         $fatal(1, "smolrv64_sq: address written to a slot with no live entry (idx %0d)", a_idx);
      if (a_v & av[a_idx])
         $fatal(1, "smolrv64_sq: address written twice to slot %0d", a_idx);
      // A store whose data can never arrive wedges the ROB head forever -- the one failure
      // of this structure that presents as a hang rather than a wrong answer, so it gets an
      // assertion rather than a comment. dpreg 0 is LEGAL (rs2 = x0, or any value already in
      // the PRF): it means no writeback is coming, which is only a bug if a_data_v did not
      // supply the value either.
      for (li = 0; li < NW; li = li + 1)
         if (d_alloc[li] & ~d_rdy[li] & (d_dpreg[li*PBITS +: PBITS] == {PBITS{1'b0}}))
            $fatal(1, "smolrv64_sq: a store allocated with no data and no producer -- it can never commit");
      for (li = 0; li < NENT; li = li + 1)
         if (sn_m[li] & (rd[li] | (rq_v & (rq_idx == li[IDXB-1:0]))))
            $fatal(1, "smolrv64_sq: entry %0d snooped a writeback for a value already in a register file (rd %b, read %b)",
                   li, rd[li], rq_v & (rq_idx == li[IDXB-1:0]));
      for (li = 0; li < NW; li = li + 1)
         if (al_m[li] & d_rdy[li])
            $fatal(1, "smolrv64_sq: a store allocated into entry %0d with its value in a register file sees p%0d written back",
                   al_e[li], d_dpreg[li*PBITS +: PBITS]);
      if (c_take & ~c_v)
         $fatal(1, "smolrv64_sq: handoff with no committed, ready entry");
      // the freed entry is one handed off, now or before; at most one waits for memory
      if (c_done & (headc == hoc) & ~(c_v & c_take))
         $fatal(1, "smolrv64_sq: entry %0d freed before its handoff", head);
      if (ho_n > 1)
         $fatal(1, "smolrv64_sq: %0d entries handed off and not in memory (the LSU holds one)", ho_n);
      if (ho_n > kc_n)
         $fatal(1, "smolrv64_sq: the handoff pointer is outside [head, the first uncommitted entry]");
      if (k_take & ~kc_v)
         $fatal(1, "smolrv64_sq: commit with no uncommitted, ready entry");
      if (k_take & flush)
         $fatal(1, "smolrv64_sq: a store committed in a redirect cycle (the redirecting op is at the irrevocable point)");
      if ((|kmk) & (|d_alloc) & ~flush)
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
      if (l_fill_go & (~l_fill | l_fill_flt))
         $fatal(1, "smolrv64_sq: load entry %0d started with no fill, or with its address fault", l_fill_ix);
      if (c_take & ~tv[ho])
         $fatal(1, "smolrv64_sq: a store drained without a translation");
   end
endmodule
`default_nettype wire
