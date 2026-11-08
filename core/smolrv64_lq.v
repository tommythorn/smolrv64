`default_nettype none

// smolrv64_lq -- the load queue.
//
// WHY. docs/Area-Efficient-Scalar-OoO.md 11.1: ISSUE is free, ACCESS is ordered. The store
// buffer applied that to stores. This applies it to loads, and without it the ordering test
// has nowhere to live but the translate path:
//
//   smolrv64_lsu used to gate `start_ok` on smolrv64_sq's ld_block, which is computed from the
//   MMU's t_paddr. So a blocked load sat in S_IDLE re-translating every cycle, holding the
//   whole LSU, with an adder and two 57-bit magnitude compares hanging off the end of the
//   dTLB lookup. It was correct and it was the wrong shape: translate and access were
//   fused, which is exactly the conflation 11.1 exists to undo.
//
// Here the address is REGISTERED into an entry when translation completes, and the alias
// test reads that flop. The compare starts at the top of a cycle instead of at the end of
// the translate path, and the LSU is free the moment the address is known.
//
// SHAPE, deliberately the same as smolrv64_sq's, because they are the same idea applied to the
// two directions:
//   allocate  at DISPATCH, in program order. Not at fill: allocating at fill would make
//             entry order depend on ISSUE order, and u_iq_l is meant to stop being in-order
//             (spec 15). Program order is what the ROB index and the store-seqno mean.
//   fill      when the load's address is translated. M releases it here -- from this point
//             the access cannot fault (smolrv64_lsu decides faults before it leaves S_IDLE), so
//             the load is architecturally guaranteed to complete.
//   access    the oldest filled entry that no older store can alias. One at a time: the D$
//             has one port, so exactly one entry needs testing per cycle and the queue needs
//             exactly one disambiguation query port into smolrv64_sq.
//   land      data returns, writes back, completes the ROB slot.
//
// FILL AND ACCESS COLLAPSE. The core starts a translated load's access in the same pass that
// fills the entry (a_sent), whatever older stores are live, and the entry goes straight to
// `sent`. The alias test answers a cycle later, from the row the fill wrote: a load an older
// store blocks has its access cancelled (a_cancel, smolrv64_sq l_cancel) and is unsent again,
// to forward or wait like any blocked entry. Everything else is unchanged -- the entry is still
// filled, M is still released here, and the data still lands through l_*. That last part is
// the point: an earlier attempt let M keep the load through its access instead, and giving up
// the early release cost more than the cycles it saved (ldbench 6.00 -> 7.00 cyc/load).
//
// FLUSH IS WHOLESALE for the same reason as smolrv64_sq's: `redirect` is gated by head_block,
// so it fires only with the redirecting instruction at the ROB head, and everything still
// live here is younger than that. A load already ACCESSED but not yet landed is younger
// too -- a load has no side effect, so discarding its result is free.
module smolrv64_lq
  #(parameter NENT  = 4,
    parameter IDXB  = 2,               // $clog2(NENT)
    parameter PAW   = 56,
    parameter PBITS = 9,
    parameter ROBB  = 4,
    parameter SQIB  = 3,               // store-seqno width (smolrv64_sq's IDXB)
    parameter VAW   = 39,              // Sv39: the VA an entry keeps beside its PA
    parameter SEQW  = 8,               // the op's sequence number (a trap restarts fetch at it)
    parameter PCHB  = 8,               // bits of the load's PC the wait table is indexed by (PC[PCHB:1])
    parameter NW    = 1,               // loads allocated a cycle: a dispatch group's row
    parameter NB    = 4,               // banks: the entry's low bits; a row's loads take one each
    parameter LDPRE = 1,               // the landing's destination from registers read a cycle ahead (l_pre_a);
                                       // 0 reads it at the landing (the unit benches, which pick it then)
    parameter RKB   = (NW > 1) ? $clog2(NW) : 1,   // derived: a rank among a row's loads
    parameter BL    = $clog2(NB),                  // derived: the bank bits
    parameter BRB   = IDXB - BL,                   // derived: the row bits
    // The memory map, for the D12 cross-check below only (smolrv64_lsu's LSU_DRAM_BASE / LRAM_*).
    // A unit bench with synthetic addresses declares everything memory: DRAM_BASE = 0.
    parameter [63:0] DRAM_BASE = 64'h8000_0000,
    parameter [63:0] LRAM_BASE = 64'h7000_0000,
    parameter        LRAM_LG2  = 18)
   (input  wire                  clk,
    input  wire                  reset,

    // ---- allocate: at dispatch, up to NW a cycle, writer j a dispatch slot ([j*w +: w]) ----
    input  wire [NW-1:0]         d_alloc,
    input  wire [NW*RKB-1:0]     d_rank,      // writer j's rank among the cycle's loads, in program order
    input  wire [NW*ROBB-1:0]    d_rob,
    input  wire [NW*PBITS-1:0]   d_prd,
    input  wire [NW*6-1:0]       d_rd,
    input  wire [NW-1:0]         d_rd_v,
    input  wire [NW*SQIB-1:0]    d_sqtag,     // store-seqno captured at the same dispatch
    input  wire [NW-1:0]         d_wt,        // the load's wait bit: it waits for older stores' unknown addresses
    input  wire [NW*VAW-1:0]     d_pc,        // the load's PC and seq, for a trap from its translation
    input  wire [NW*SEQW-1:0]    d_seq,
    output wire                  d_ready,
    output wire [NB*(BRB+1)-1:0] d_bfree,     // each bank's free entries, from flops (the dispatch credit's)
    output wire [BL-1:0]         d_bp,        // the bank the next cycle's first load takes
    output wire [NW*IDXB-1:0]    d_idx,       // writer j's entry

    // ---- fill: the address is translated; M lets go here ----
    // a_sent says the SAME cycle also handed the access to memory (see the header). The
    // entry goes straight to `sent` -- it never becomes a candidate, because it has already
    // been what a candidate is for.
    input  wire                  a_v,
    input  wire                  a_sent,
    input  wire [NENT-1:0]       a_cancel,    // per entry: last cycle's a_sent is void (smolrv64_sq l_cancel)
    input  wire [IDXB-1:0]       a_idx,
    input  wire [PAW-1:0]        a_pa,        // ...its PA, when a_tv
    input  wire [VAW-1:0]        a_va,        // the VA, always: the alias test and the walker read it
    input  wire                  a_tv,        // M's lookup translated it (else the walker will)
    input  wire                  a_flt,       // ...or its address alone faults (a_tv is then set)
    input  wire [3:0]            a_fc,
    input  wire [1:0]            a_size,
    input  wire                  a_signed,
    input  wire                  a_fp,
    input  wire                  a_unc,       // Svpbmt NC/IO: decided by the translation, travels with the entry
    input  wire                  a_mem,       // the PA is DRAM/LRAM (idempotent) -> may access speculatively
    input  wire                  a_bare,      // the address was not translated (M-mode or satp Bare)

    // ---- disambiguation: a BIT PER ENTRY, maintained by smolrv64_sq -----------------------
    // The entries are exported so smolrv64_sq can run the alias test where an ADDRESS ARRIVES
    // and hand back a flop. Asking it at issue instead put the candidate at the head of a 36-level,
    // 12x CARRY8 cone ending at the frontend's redirect register: WNS -1.698 ns at
    // 166.67 MHz, 18 670 failing endpoints, all five worst paths sourced here. Moving the
    // address off the TRANSLATE path -- this module's original purpose -- left the compare
    // itself, and its whole downstream tail, exactly where they were.
    output wire [NENT*12-1:0]    e_off,       // page offsets: VA[11:0] = PA[11:0]
    output wire [NENT*(VAW-11)-1:0] e_vpn,    // ...and the page, {bare, VA[VAW-1:12]}, for forwarding (smolrv64_sq cov)
    output wire [NENT*2-1:0]     e_size,
    output wire [NENT*SQIB-1:0]  e_tag,
    output wire [NENT-1:0]       e_av,
    output wire [NENT*PCHB-1:0]  e_pch,       // each entry's PC hash (the wait table's index)
    output wire [NENT-1:0]       e_wt,        // each entry's wait bit...
    output wire [NENT-1:0]       e_acc,       // ...and whether it has reached memory (sent, or landed and watched)
    input  wire [NENT-1:0]       unk_q,       // smolrv64_sq: an older store's address is still unknown (registered)
    input  wire [NENT-1:0]       viol,        // smolrv64_sq: it went ahead of an older store that overlaps it
    input  wire [NENT-1:0]       e_block,     // per entry: an older store aliases it
    // ...and it can take its bytes from that store instead (smolrv64_sq, registered): the store
    // covers them, has its data, and every older store's address is known. Forwards wait for their
    // landing in the LSU, two deep; fw_busy says both are waiting.
    input  wire [NENT-1:0]       e_fwd,
    input  wire                  fw_busy,
    output wire                  x_block,     // ...and the candidate is one, held right now
    output wire                  x_devwait,   // the candidate is a device load waiting to be the head (counters only)
    // ...and the seqno of the entry being tested, for smolrv64_sq's ld_older. That one is
    // pointer arithmetic against head, with no address and no adder in it.
    output wire [SQIB-1:0]       q_tag,

    // ---- early start: may the core collapse fill and access for the entry M holds? ----
    // A load with no older live store has nothing to be ordered against (loads reach memory in
    // the order they are ready, see the candidate below), so it may start in its own fill pass.
    input  wire [IDXB-1:0]       b_idx,       // the entry M's load owns
    output wire                  b_ok,        // ...and it is live and not yet filled

    // ---- access: hand the D$ the oldest entry that is clear ----
    output wire                  x_v,
    output wire [IDXB-1:0]       x_idx,       // the entry being offered -- the response's tag
    output wire [PAW-1:0]        x_pa,
    output wire [1:0]            x_size,
    output wire                  x_signed,
    output wire                  x_fp,
    output wire                  x_unc,
    output wire                  x_head,      // the candidate's op is the ROB head (index compare; the core adds ~rob_empty)
    output wire                  x_fwd,       // the candidate is held by a store and takes its bytes from it
    input  wire                  x_take,      // the LSU accepted it

    // ---- land: data back for an entry that was sent to memory ----
    // The response names its ENTRY by the tag the requester allocated (rule B1), not by
    // age: the D$ answers out of order. l_idx is the landing entry; its payload is read at
    // both of the entries it can be, the D$ response's tag (l_tg) and the other candidate
    // (l_alt), each a register, and l_rsp (the D$'s response lands) picks the answer, so
    // the late pick reaches no array's index (rule I6). l_idx is l_rsp ? l_tg : l_alt.
    input  wire                  l_v,
    input  wire [IDXB-1:0]       l_idx,
    input  wire [IDXB-1:0]       l_tg,
    input  wire [IDXB-1:0]       l_alt,
    input  wire                  l_rsp,
    // THE LANDING'S DESTINATION IS READ A CYCLE AHEAD at every entry the next landing can be: the
    // tags the D$'s next answer can carry (l_pre_a, l_pre_b: its lookup and its NC slot, registers)
    // and which one it carries (l_src_b), the forward ring's next oldest (l_fwn) and whether a
    // forward waits (l_fw), and the slow path's entry, the one x_take starts. A landing picks its
    // register by those flags alone, so no index mux or 32-way read stands in front of the wake.
    input  wire [IDXB-1:0]       l_pre_a,
    input  wire [IDXB-1:0]       l_pre_b,
    input  wire                  l_src_b,
    input  wire                  l_fw,
    input  wire [IDXB-1:0]       l_fwn,
    output wire [PBITS-1:0]      l_prd,
    output wire [5:0]            l_rd,
    output wire                  l_rd_v,
    output wire [ROBB-1:0]       l_rob,
    // The landing load's OWN address, for the cosim's memory-effect record. It has to come
    // from the entry that owns the load: the shared lsu_cos_* pair is whatever access ran
    // LAST, and since b9dbdd0 a queued load's access starts in its translate pass, a store
    // can and does start between a load's start and its landing.
    output wire [PAW-1:0]        l_pa,

    // ---- the walker: the candidate, when it has no translation yet ----
    output wire                  k_v,
    output wire [IDXB-1:0]       k_idx,
    output wire [VAW-1:0]        k_va,
    input  wire                  w_v,         // the walker's answer for entry w_idx
    input  wire [IDXB-1:0]       w_idx,
    input  wire [PAW-1:0]        w_pa,
    input  wire                  w_unc,
    input  wire                  w_mem,
    input  wire                  w_flt,
    input  wire [3:0]            w_fc,
    // ---- a translation that faulted, at the candidate: it traps at the ROB head ----
    output wire                  f_new,       // an entry is flagged for replay this cycle (rpl set at this edge)
    output wire                  f_v,
    output wire [IDXB-1:0]       f_idx,       // the entry it is: the oldest one whose translation faulted or that must replay
    output wire                  f_rp,        // ...a replay (it read before an older store that overlaps it), not a fault
    output wire [ROBB-1:0]       f_rob,
    output wire [3:0]            f_fc,
    output wire [VAW-1:0]        f_pc,
    output wire [SEQW-1:0]       f_seq,

    output wire [IDXB:0]         occupancy,
    output wire                  av_any,      // an entry with a known address: a load older than M's op, not landed (rule C5)
    output wire [NENT*PBITS-1:0] e_dprd,      // each live entry's destination, 0 when free (the stall counters)
    output wire                  uf_any,      // an entry that is not filled yet (live, no address in)
    // M's pick: the oldest live entry whose address has ARRIVED (ar: the core holds it for M) but
    // that M has not filled yet. M fills in arrival order, not program order.
    input  wire [NENT-1:0]       ar,
    output wire                  pk_v,
    output wire [IDXB-1:0]       pk_idx,
    output wire [SEQW-1:0]       pk_seq,
    input  wire [ROBB-1:0]       rob_head,    // head-gate an uncached (device) load's access (non-speculative)
    input  wire                  flush,
    // the kill (smolrv64_rob kd): the entries of ops younger than a mispredicted branch die and
    // their slots free. kmask names every slot whose op is dead while the kill holds, live or not
    // (nothing allocates meanwhile): the LSU drops their responses by tag, a load among them
    // starting now included.
    input  wire                  kd_v,
    input  wire [ROBB-1:0]       kh,
    input  wire [ROBB-1:0]       kb,
    output wire [NENT-1:0]       kmask);

   // An age matrix and a per-entry `sent` bit. Entries wait to go to memory in any order they
   // become ready; what is sent is outstanding or already landed. Flush is a reset of `v`.
   reg [NENT-1:0]        v, av;              // live / address known
   // pa: M's fill and the walker's answer each write it, three ports read it -- a bank per writer
   // (u_pa below). pc: read at one entry, but its wait-table bits at every one -- those are flops.
   reg [VAW-1:0]         va   [0:NENT-1];
   reg [PCHB:1]          pch  [0:NENT-1];       // ...its wait-table bits, read at every entry
   reg [SEQW-1:0]        sqn  [0:NENT-1];
   reg [NENT-1:0]        tv, flt;             // translated / ...and the translation faulted
   reg [3:0]             fc   [0:NENT-1];
   reg [1:0]             sz   [0:NENT-1];
   // EVERY ATTRIBUTE THE ACCESS NEEDS TRAVELS WITH THE ENTRY. `unc` was missing: a queued
   // load went to the LSU with the uncached bit hardwired to 0 (smolrv64_core's pt_unc), so a
   // Svpbmt NC load whose access came from the queue -- rather than the early start, which
   // reads the MMU's bit directly -- was cached, the line stayed resident, and every later
   // read of it hit stale data. Linux's virtio rings live in exactly such memory: "id 0 is
   // not a head!" on the board, 2026-09-04, with no simulation able to see it (the cosim's
   // guest maps nothing NC). The store queue had carried its own bit all along.
   reg [NENT-1:0]        sgn, isfp, unc;
   reg [NENT-1:0]        mem;                 // PA is DRAM/LRAM (idempotent): may access speculatively
   reg [NENT-1:0]        bare;                // the address was not translated (smolrv64_sq a_bare)
   reg [PBITS-1:0]       prd  [0:NENT-1];
   reg [5:0]             rdn  [0:NENT-1];
   reg [NENT-1:0]        rdv;
   reg [ROBB-1:0]        rob  [0:NENT-1];
   reg [SQIB-1:0]        sqt  [0:NENT-1];
   // `sent` replaces a HEAD pointer, which could only say "the OLDEST entry is outstanding".
   // That stops being the same question once an entry can reach memory without ever being
   // the candidate (a_sent). Per entry it is exact, and it is what the landing assertion
   // below actually wants to know.
   reg [NENT-1:0]        sent;                // handed to memory, data not back yet (or landed and watched)
   reg [NENT-1:0]        wt;                  // the load waits for older stores' unknown addresses
   reg [NENT-1:0]        lnd;                 // landed, and still here: an older store's address is unknown
   reg [NENT-1:0]        rpl;                 // sticky: it read before an older store that overlaps it
   reg [NENT-1:0]        fwd;                 // sent as a forward: it read a store, not memory
   reg [NENT*NENT-1:0]   om;                  // om[i*NENT + j]: entry j is older than entry i
   reg [IDXB:0]          cnt;

   initial begin v = {NENT{1'b0}}; av = {NENT{1'b0}}; sent = {NENT{1'b0}};
                 wt = {NENT{1'b0}}; lnd = {NENT{1'b0}}; rpl = {NENT{1'b0}}; fwd = {NENT{1'b0}};
                 om = {(NENT*NENT){1'b0}}; cnt = {(IDXB+1){1'b0}}; end
   initial if ((NB & (NB - 1)) != 0 || NB < NW || NENT % NB != 0 || ((NENT / NB) & (NENT / NB - 1)) != 0)
      $fatal(1, "smolrv64_lq: %0d banks for %0d loads a row and %0d entries", NB, NW, NENT);

   // PROGRAM ORDER IS AN AGE MATRIX. An allocation sets its row to the live entries (all older)
   // and clears its column (younger than every one of them); a freed entry's bits are never read,
   // since every set asked about is live. The oldest of a set is the member no other member is
   // older than: one per set, by construction, so the index is the OR of the one-hot.
`include "smolrv64_kill.vh"
   function [IDXB-1:0] oldest(input [NENT-1:0] m, input [NENT*NENT-1:0] o);
      integer i;
      begin
         oldest = {IDXB{1'b0}};
         for (i = 0; i < NENT; i = i + 1)
            if (m[i] && ~|(m & o[i*NENT +: NENT])) oldest = oldest | i[IDXB-1:0];
      end
   endfunction

   // ANY FREE SLOT OF ITS BANK, the lowest. Loads land out of order (the tagged fast path; the
   // random bench lands them in any order), so the next slot of a ring can hold a load still in
   // flight while others are free; the age matrix keeps program order whichever slot an entry
   // takes. A row's loads take the banks that have a free entry in turn from the bank pointer (bp),
   // in program order, so each of them is a different bank's (an entry's bank is its low bits) and
   // every array the allocation writes takes one write a bank; a full bank is passed over, so the
   // queue refuses a load only when fewer banks than the row's loads have room. A slot whose killed load is still in flight is free
   // here: the LSU starts nothing on a tag whose response is still due (its o_v).
   localparam integer ROWS = NENT / NB;
   reg  [BL-1:0]   bp;
   initial bp = {BL{1'b0}};
   // EACH BANK'S FREE ROW IS RESERVED A CYCLE AHEAD (bfr, bhas, bfc: registers), as a scheduler's
   // slot is: the lowest row free now other than the one this cycle allocates in the bank, and the
   // count with it (the dispatch credit's, so the credit never promises a row the reservation lacks).
   // A row freed now is free from the next cycle's reservation on.
   reg  [BRB-1:0]  bfr [0:NB-1];        // bank b's reserved free row
   reg  [NB-1:0]   bhas;                // ...it has one
   reg  [BRB:0]    bfc [0:NB-1];        // ...its free entries
   reg  [BRB-1:0]  bfr_n [0:NB-1];
   reg  [NB-1:0]   bhas_n;
   reg  [BRB:0]    bfc_n [0:NB-1];
   reg  [NB-1:0]   b_w;                 // bank b takes a load this cycle (below)
   integer         fb, fr;
   always @* for (fb = 0; fb < NB; fb = fb + 1) begin
      bfr_n[fb] = {BRB{1'b0}};  bhas_n[fb] = 1'b0;  bfc_n[fb] = {(BRB+1){1'b0}};
      for (fr = ROWS - 1; fr >= 0; fr = fr - 1)
         if (~v[fr*NB + fb] & ~(b_w[fb] & (bfr[fb] == fr[BRB-1:0]))) begin
            bfr_n[fb] = fr[BRB-1:0];  bhas_n[fb] = 1'b1;  bfc_n[fb] = bfc_n[fb] + 1'b1;
         end
   end
   initial bhas = {NB{1'b0}};
   always @(posedge clk) for (fb = 0; fb < NB; fb = fb + 1) begin
      bfr[fb] <= bfr_n[fb];  bhas[fb] <= ~reset & bhas_n[fb];  bfc[fb] <= reset ? {(BRB+1){1'b0}} : bfc_n[fb];
   end
   always @(posedge clk) if (!reset) for (fb = 0; fb < NB; fb = fb + 1)
      if (b_w[fb] & (~bhas[fb] | v[{bfr[fb], fb[BL-1:0]}]))
         $fatal(1, "smolrv64_lq: bank %0d allocates its reserved row %0d, which is not free (has %b)", fb, bfr[fb], bhas[fb]);
   reg  [BL-1:0]   rbk [0:NB-1];        // the banks with room, from bp on: the r-th one a rank-r load takes
   reg  [BL:0]     nbk;                 // ...how many
   reg  [BL-1:0]   wbk [0:NW-1];        // writer j's bank...
   reg  [IDXB-1:0] ws  [0:NW-1];        // ...and entry
   reg  [NENT-1:0] wahead [0:NW-1];     // the cycle's loads ahead of writer j's, by entry
   reg  [IDXB:0]   n_alloc;
   integer         fj, fi2;
   reg  [BL-1:0]   rbb;
   always @* begin
      nbk = {(BL+1){1'b0}};
      for (fj = 0; fj < NB; fj = fj + 1) rbk[fj] = bp;
      for (fj = 0; fj < NB; fj = fj + 1) begin
         rbb = bp + fj[BL-1:0];
         if (bhas[rbb]) begin rbk[nbk[BL-1:0]] = rbb;  nbk = nbk + 1'b1; end
      end
      n_alloc = {(IDXB+1){1'b0}};
      for (fj = 0; fj < NW; fj = fj + 1) begin
         wbk[fj] = rbk[d_rank[fj*RKB +: RKB]];
         ws[fj]  = {bfr[wbk[fj]], wbk[fj]};
         n_alloc = n_alloc + {{IDXB{1'b0}}, d_alloc[fj]};
      end
      for (fj = 0; fj < NW; fj = fj + 1) begin
         wahead[fj] = {NENT{1'b0}};
         for (fi2 = 0; fi2 < NW; fi2 = fi2 + 1)
            if (d_alloc[fi2] & (d_rank[fi2*RKB +: RKB] < d_rank[fj*RKB +: RKB])) wahead[fj][ws[fi2]] = 1'b1;
      end
   end
   genvar gw;
   generate for (gw = 0; gw < NW; gw = gw + 1) begin: g_widx
      assign d_idx[gw*IDXB +: IDXB] = ws[gw];
   end endgenerate
   // THE ROW'S FIELDS, A BANK AT A TIME: its loads take distinct banks (wbk), so a bank's writer is
   // muxed once (one-hot, AND-OR) and each of its entries only enabled -- not a writers' priority mux
   // in front of every entry.
   // (b_w, bank b takes a load this cycle, is declared with the reservation above)
   reg  [NW-1:0]      b_oh [0:NB-1];       // ...writer j's
   reg                b_wt [0:NB-1];  reg b_rdv [0:NB-1];
   reg [ROBB-1:0]     b_rob [0:NB-1];  reg [PBITS-1:0] b_prd [0:NB-1];  reg [5:0] b_rdn [0:NB-1];
   reg [SQIB-1:0]     b_sqt [0:NB-1];  reg [PCHB:1] b_pch [0:NB-1];  reg [SEQW-1:0] b_sqn [0:NB-1];
   integer            fbk, fwj;
   always @* for (fbk = 0; fbk < NB; fbk = fbk + 1) begin
      b_w[fbk] = 1'b0;  b_oh[fbk] = {NW{1'b0}};
      b_wt[fbk] = 1'b0;  b_rdv[fbk] = 1'b0;  b_rob[fbk] = {ROBB{1'b0}};  b_prd[fbk] = {PBITS{1'b0}};
      b_rdn[fbk] = 6'd0;  b_sqt[fbk] = {SQIB{1'b0}};  b_pch[fbk] = {PCHB{1'b0}};  b_sqn[fbk] = {SEQW{1'b0}};
      for (fwj = 0; fwj < NW; fwj = fwj + 1)
         if (d_alloc[fwj] & (wbk[fwj] == fbk[BL-1:0])) begin
            b_w[fbk] = 1'b1;  b_oh[fbk][fwj] = 1'b1;
            b_wt[fbk]  = b_wt[fbk]  | d_wt[fwj];
            b_rdv[fbk] = b_rdv[fbk] | d_rd_v[fwj];
            b_rob[fbk] = b_rob[fbk] | d_rob[fwj*ROBB +: ROBB];
            b_prd[fbk] = b_prd[fbk] | d_prd[fwj*PBITS +: PBITS];
            b_rdn[fbk] = b_rdn[fbk] | d_rd[fwj*6 +: 6];
            b_sqt[fbk] = b_sqt[fbk] | d_sqtag[fwj*SQIB +: SQIB];
            b_pch[fbk] = b_pch[fbk] | d_pc[fwj*VAW + 1 +: PCHB];
            b_sqn[fbk] = b_sqn[fbk] | d_seq[fwj*SEQW +: SEQW];
         end
   end
   always @(posedge clk) if (!reset) for (fbk = 0; fbk < NB; fbk = fbk + 1)
      if ((b_oh[fbk] & (b_oh[fbk] - 1'b1)) != {NW{1'b0}})
         $fatal(1, "smolrv64_lq: two loads of a row take bank %0d (%b)", fbk, b_oh[fbk]);
   generate for (gw = 0; gw < NB; gw = gw + 1) begin: g_bfree
      assign d_bfree[gw*(BRB+1) +: BRB+1] = bfc[gw];
   end endgenerate
   assign d_bp      = bp;
   assign d_ready   = |bhas;                // room for the next load, the row's first
   assign occupancy = cnt;
   assign av_any    = |(v & av & ~lnd);
   genvar gd;
   generate
      for (gd = 0; gd < NENT; gd = gd + 1) begin : g_dprd
         assign e_dprd[gd*PBITS +: PBITS] = (v[gd] & ~lnd[gd]) ? prd[gd] : {PBITS{1'b0}};
      end
   endgenerate
   assign uf_any    = |(v & ~av);
   wire [NENT-1:0] pk_m = v & ~av & ar;
   // M'S PICK IS MADE A CYCLE AHEAD: the oldest entry whose address waits, registered (pk_q), and
   // offered while that entry's address still waits -- the age pick stays out of M's take. One
   // taken last cycle waits no longer: back to back, M takes a waiting address every other cycle.
   reg  [IDXB-1:0] pk_q;
   initial pk_q = {IDXB{1'b0}};
   always @(posedge clk) pk_q <= oldest(pk_m, om);
   assign pk_v      = pk_m[pk_q];
   assign pk_idx    = pk_q;
   assign pk_seq    = sqn[pk_idx];

   // THE ACCESS CANDIDATE is the oldest entry that may go to memory now: filled and translated,
   // not faulting, not yet sent, no older store aliasing it (e_block, a register per entry; an
   // older store whose address is unknown counts), and either idempotent memory or, for a device
   // load, its op at the ROB head. Loads reach memory in the order they become ready, not in
   // program order: a single hart sees no difference between two loads in either order, a store
   // never commits ahead of an older load that has not landed (the irrevocable pointer), and
   // device loads wait for the head, which orders them against everything older.
   // ...and NOT already sent: with every entry taken and none landed the ring wraps round to a
   // live, translated entry IN FLIGHT, and `v & av` would offer it a second time.
   // "ITS OP IS THE ROB HEAD" IS A FLOP PER ENTRY (hd), compared against the head register as
   // it stands: a device load goes the cycle after the head reaches it, and the flag is never
   // set for an entry that is not the head -- a live entry at the head stays there until it
   // lands, and a slot freed and refilled in one cycle is unfilled (av) for longer than the
   // flag's one-cycle lag. The head register is otherwise in front of the candidate pick, and
   // through the LSU's start, in front of M's completion and every scheduler's wake.
   wire [NENT-1:0] cand_m;
   wire [NENT-1:0] head_m;
   reg  [NENT-1:0] hd;
   initial hd = {NENT{1'b0}};
   genvar gc;
   generate for (gc = 0; gc < NENT; gc = gc + 1) begin : g_cand
      assign cand_m[gc] = v[gc] & av[gc] & ~sent[gc] & tv[gc] & ~flt[gc];
      assign head_m[gc] = (rob[gc] == rob_head);
   end endgenerate
   always @(posedge clk) hd <= reset ? {NENT{1'b0}} : (v & head_m);
   always @(posedge clk) if (!reset && |(cand_m & hd & ~head_m))
      $fatal(1, "smolrv64_lq: a candidate's head flag is set while its op is not the head (hd=%b head_m=%b cand=%b)", hd, head_m, cand_m);
   // A held load is ready too when it can forward: idempotent memory only (a device or uncached
   // load reads the device), and while no other forward is in flight.
   wire [NENT-1:0] fwd_m = e_fwd & mem & ~unc & {NENT{~fw_busy}};
   wire [NENT-1:0] rdy_m = cand_m & (~e_block | fwd_m) & (mem | hd);
   // THE WALKER AND THE TRAP READ THE OLDEST ENTRY THAT NEEDS THEM: an untranslated filled load
   // is walked oldest first, and a load whose translation faulted is never sent: it traps when
   // its op reaches the ROB head.
   wire [NENT-1:0] wlk_m = v & av & ~sent & ~tv;
   wire [NENT-1:0] flt_m = (v & av & ~sent & tv & flt) | (v & rpl);
   assign k_v   = |wlk_m;
   assign k_idx = oldest(wlk_m, om);
   assign k_va  = va[k_idx];
   wire [IDXB-1:0] fi = oldest(flt_m, om);
   assign f_v   = |flt_m;
   assign f_new = |(viol & v & ~rpl);
   assign f_idx = fi;
   assign f_rp  = rpl[fi];
   assign f_rob = rob[fi];
   assign f_fc  = fc[fi];
   assign f_seq = sqn[fi];
   // M's load can start its access in its fill pass whenever the entry is live and unfilled
   assign b_ok = v[b_idx] & ~av[b_idx];
   // the older-store query is about M's load when it holds an unfilled one, else the oldest filled
   // entry still waiting to go to memory
   wire [IDXB-1:0] qi = b_ok ? b_idx : oldest(v & av & ~sent, om);
   assign q_tag   = sqt[qi];
   genvar ge;
   generate
      for (ge = 0; ge < NENT; ge = ge + 1) begin : g_exp
         assign e_off [ge*12   +: 12  ] = va [ge][11:0];
         assign e_vpn [ge*(VAW-11) +: VAW-11] = {bare[ge], va[ge][VAW-1:12]};
         assign e_size[ge*2    +: 2   ] = sz [ge];
         assign e_tag [ge*SQIB +: SQIB] = sqt[ge];
      end
   endgenerate
   assign e_av = av;
   assign e_wt = wt;
   generate for (gd = 0; gd < NENT; gd = gd + 1) begin : g_pch
      assign e_pch[gd*PCHB +: PCHB] = pch[gd];
   end endgenerate
   // A forwarded load counts as having reached memory: a store older than it whose address arrives
   // and overlaps it flags it for replay. Its forwarding store does not (the store queue sets that
   // overlap aside while the load stands forwarded).
   assign e_acc = v & sent;
   // THE CANDIDATE IS PICKED A CYCLE AHEAD: the oldest ready entry, registered (xi), offered while
   // it is still ready -- the age pick stays out of the LSU's start. A pick taken last cycle is
   // sent now, so not ready: the cycle after a take offers none.
   wire [IDXB-1:0] xi_n = oldest(rdy_m, om);
   reg  [IDXB-1:0] xi;
   initial xi = {IDXB{1'b0}};
   always @(posedge clk) xi <= xi_n;
   assign x_head  = hd[xi];
   assign x_v     = rdy_m[xi];
   assign x_block = |(cand_m & e_block);   // instrumentation only
   assign x_devwait = |(cand_m & ~e_block & ~mem & ~hd) & ~x_v;   // instrumentation only
   assign x_idx   = xi;
   assign x_fwd   = e_block[xi];
   wire [3*PAW-1:0] pa_rd;                 // the start's entry, the landing's two candidates
   smolrv64_mwram #(.W(PAW), .DEPTH(NENT), .NW(2), .NR(3), .AB(IDXB)) u_pa
     (.clk(clk), .reset(reset), .wv({w_v, a_v} & {2{~reset}}), .wa({w_idx, a_idx}), .wd({w_pa, a_pa}),
      .ra({l_alt, l_tg, xi}), .rd(pa_rd));
   assign x_pa    = pa_rd[0 +: PAW];
   assign x_size  = sz[xi];
   assign x_signed= sgn[xi];
   assign x_fp    = isfp[xi];
   assign x_unc   = unc[xi];

   // The landing entry's payload, read at both candidates and picked by l_rsp; its destination and
   // whether it writes one, from the registers read a cycle ahead (see l_pre_a). An entry the next
   // landing can be is in flight, so nothing rewrites its destination between the read and the landing.
   reg  [PBITS:0] ld_a_q, ld_b_q, ld_f_q, ld_s_q;
   initial begin ld_a_q = {(PBITS+1){1'b0}}; ld_b_q = {(PBITS+1){1'b0}}; ld_f_q = {(PBITS+1){1'b0}}; ld_s_q = {(PBITS+1){1'b0}}; end
   always @(posedge clk) begin
      ld_a_q <= {rdv[l_pre_a], prd[l_pre_a]};
      ld_b_q <= {rdv[l_pre_b], prd[l_pre_b]};
      ld_f_q <= {rdv[l_fwn], prd[l_fwn]};
      if (x_v & x_take) ld_s_q <= {rdv[x_idx], prd[x_idx]};
   end
   wire [PBITS:0] ld_d = l_rsp ? {rdv[l_tg], prd[l_tg]} : {rdv[l_alt], prd[l_alt]};
   wire [PBITS:0] ld_q = (LDPRE == 0) ? ld_d : l_rsp ? (l_src_b ? ld_b_q : ld_a_q) : (l_fw ? ld_f_q : ld_s_q);
   assign l_prd  = ld_q[PBITS-1:0];
   assign l_rd   = l_rsp ? rdn[l_tg] : rdn[l_alt];
   assign l_rd_v = ld_q[PBITS];
   always @(posedge clk) if (!reset && l_v && (ld_q != ld_d))
      $fatal(1, "smolrv64_lq: the landing's destination read a cycle ahead (%b p%0d) is not entry %0d's (%b p%0d)",
             ld_q[PBITS], ld_q[PBITS-1:0], l_rsp ? l_tg : l_alt, ld_d[PBITS], ld_d[PBITS-1:0]);
   assign l_rob  = l_rsp ? rob[l_tg] : rob[l_alt];
   assign l_pa   = l_rsp ? pa_rd[PAW +: PAW] : pa_rd[2*PAW +: PAW];


   // THE KILL. The dead entries are the youngest live ones; their slots free.
   reg  [NENT-1:0] kmk, kds;
   integer         kq, kw, aw;
   always @* begin
      for (kq = 0; kq < NENT; kq = kq + 1) begin
         kds[kq] = kdead(kd_v, kh, kb, rob[kq]);
         kmk[kq] = kds[kq] & v[kq];
      end
   end
   assign kmask = kds;
   wire          k_any = |kmk;
   // WATCHING. A load that landed while an older store's address was still unknown stays (lnd,
   // address valid) until that store's address is in and showed no overlap; one flagged for replay
   // stays until it replays. Everything else leaves at its landing.
   wire            lwatch = l_v & (unk_q[l_idx] | rpl[l_idx] | viol[l_idx]);
   wire [NENT-1:0] wfree  = v & lnd & ~unk_q & ~rpl & ~viol;
   // the entries that leave the live set this cycle (one popcount, so an entry counts once)
   reg  [NENT-1:0] gone;
   reg  [IDXB:0]   gone_n;
   integer         gn;
   always @* begin
      gone = wfree | kmk;
      if (l_v & ~lwatch) gone[l_idx] = 1'b1;
      gone_n = {(IDXB+1){1'b0}};
      for (gn = 0; gn < NENT; gn = gn + 1) gone_n = gone_n + {{IDXB{1'b0}}, gone[gn]};
   end

   always @(posedge clk) begin
      if (reset) begin
         v <= {NENT{1'b0}}; av <= {NENT{1'b0}}; sent <= {NENT{1'b0}};
         lnd <= {NENT{1'b0}}; rpl <= {NENT{1'b0}};
         cnt <= {(IDXB+1){1'b0}};
      end else begin
         // Landing frees the entry's slot, unless it stays to be watched.
         if (l_v & ~lwatch) begin v[l_idx] <= 1'b0; av[l_idx] <= 1'b0; sent[l_idx] <= 1'b0; fwd[l_idx] <= 1'b0; end
         if (l_v & lwatch)  lnd[l_idx] <= 1'b1;
         for (kw = 0; kw < NENT; kw = kw + 1) begin
            if (wfree[kw]) begin v[kw] <= 1'b0; av[kw] <= 1'b0; sent[kw] <= 1'b0; lnd[kw] <= 1'b0; fwd[kw] <= 1'b0; end
            if (viol[kw] & v[kw]) rpl[kw] <= 1'b1;
         end
         if (x_v & x_take) begin sent[x_idx] <= 1'b1;  fwd[x_idx] <= x_fwd; end
         // a row's allocations: each its entry's fields, then every one's column cleared (younger
         // than every live entry), then each one's row: the live entries and the row's loads ahead
         for (aw = 0; aw < NB; aw = aw + 1) if (b_w[aw]) begin : bank_wr
            reg [IDXB-1:0] e;
            e = {bfr[aw], aw[BL-1:0]};
            v[e] <= 1'b1;  av[e] <= 1'b0;  sent[e] <= 1'b0;  lnd[e] <= 1'b0;  rpl[e] <= 1'b0;  fwd[e] <= 1'b0;
            wt[e] <= b_wt[aw];  rob[e] <= b_rob[aw];  prd[e] <= b_prd[aw];  rdn[e] <= b_rdn[aw];
            rdv[e] <= b_rdv[aw];  sqt[e] <= b_sqt[aw];  pch[e] <= b_pch[aw];  sqn[e] <= b_sqn[aw];
         end
         for (aw = 0; aw < NW; aw = aw + 1) if (d_alloc[aw])
            for (kw = 0; kw < NENT; kw = kw + 1) om[kw*NENT + ws[aw]] <= 1'b0;
         for (aw = 0; aw < NW; aw = aw + 1) if (d_alloc[aw]) om[ws[aw]*NENT +: NENT] <= v | wahead[aw];
         cnt <= cnt + n_alloc - gone_n;
         bp  <= bp + n_alloc[BL-1:0];

         if (a_v) begin
            va[a_idx] <= a_va;  sz[a_idx] <= a_size;
            sgn[a_idx] <= a_signed;  isfp[a_idx] <= a_fp;  unc[a_idx] <= a_unc;  mem[a_idx] <= a_mem;  av[a_idx] <= 1'b1;
            bare[a_idx] <= a_bare;
            tv[a_idx] <= a_tv;  flt[a_idx] <= a_flt;  fc[a_idx] <= a_fc;
         end
         // the walker's answer: the PA and its classes, or the fault the load traps with at the head
         if (w_v) begin
            unc[w_idx] <= w_unc;  mem[w_idx] <= w_mem;  flt[w_idx] <= w_flt;  fc[w_idx] <= w_fc;
            tv[w_idx] <= 1'b1;
         end
         // Filled AND already gone. It cannot collide with x_take above: that one needs a filled
         // entry, and a_sent is asserted only for one that is not filled yet (b_ok).
         if (a_v & a_sent) sent[a_idx] <= 1'b1;
         for (kw = 0; kw < NENT; kw = kw + 1)
            if (a_cancel[kw]) sent[kw] <= 1'b0;
         // The kill, after every other arm (a dead entry landing, filled or sent now dies all the
         // same; a dead entry landing now is counted once) and before the flush. Nothing
         // allocates while it holds (asserted).
         if (k_any) begin
            for (kw = 0; kw < NENT; kw = kw + 1)
               if (kmk[kw]) begin v[kw] <= 1'b0;  av[kw] <= 1'b0;  sent[kw] <= 1'b0;  lnd[kw] <= 1'b0;  rpl[kw] <= 1'b0;  fwd[kw] <= 1'b0; end
         end
         // The flush, ordered LAST: it wins over an allocation made in its cycle (dispatch
         // is not gated on the redirect since gate V3, 2026-09-05). Nothing older than the
         // redirecting op is in flight, so every entry is younger and dies.
         if (flush) begin
            v <= {NENT{1'b0}}; av <= {NENT{1'b0}}; sent <= {NENT{1'b0}};
            lnd <= {NENT{1'b0}}; rpl <= {NENT{1'b0}}; fwd <= {NENT{1'b0}};
            cnt <= {(IDXB+1){1'b0}};
         end
      end
   end

   // the load's PC, read at the fault pick: a bank of the allocation's, one write each
   smolrv64_bankq #(.W(VAW), .DEPTH(NENT), .NW(NW), .NR(1), .NB(NB), .PICK(1), .AB(IDXB), .WLATE(1)) u_pcm
     (.clk(clk), .wclk(clk), .reset(reset), .wv(d_alloc), .wp(d_idx), .wd(d_pc), .rp(fi), .ro({BL{1'b0}}), .rd(f_pc));

   // D12 (2026-09-17): the "DRAM, idempotent, may issue speculatively" bit names the entry's OWN
   // address. smolrv64_lsu derives it from the translate; here it is recomputed from the PA the
   // entry stores, so a fill that took ANOTHER access's classification (the port's, the day
   // the board's NIC died) dies at the fill, in every cosim, on the first device load of a
   // boot -- not on the board hours later.
   // (a_pa >= DRAM_BASE as a borrow, so a bench's DRAM_BASE = 0 is not a constant compare)
   wire [PAW:0] a_dram_off = {1'b0, a_pa} - {1'b0, DRAM_BASE[PAW-1:0]};
   wire a_mem_chk = ~a_dram_off[PAW] | (a_pa[PAW-1:LRAM_LG2] == LRAM_BASE[PAW-1:LRAM_LG2]);
   wire [PAW:0] w_dram_off = {1'b0, w_pa} - {1'b0, DRAM_BASE[PAW-1:0]};
   wire w_mem_chk = ~w_dram_off[PAW] | (w_pa[PAW-1:LRAM_LG2] == LRAM_BASE[PAW-1:LRAM_LG2]);
   always @(posedge clk) if (!reset && w_v && ~w_flt && (w_mem != w_mem_chk))
      $fatal(1, "smolrv64_lq: the walker classifies pa=%h for entry %0d as %0s, its region says %0s (rule D12)",
             w_pa, w_idx, w_mem ? "memory" : "device", w_mem_chk ? "memory" : "device");
   always @(posedge clk) if (!reset && a_v && a_tv && ~a_flt && (a_mem != a_mem_chk))
      $fatal(1, "smolrv64_lq: fill %0d classifies pa=%h as %0s, its region says %0s (rule D12)",
             a_idx, a_pa, a_mem ? "memory" : "device", a_mem_chk ? "memory" : "device");

   // Invariants (docs/rtl-rules.md A1). Every one of these is something the queue would
   // otherwise do silently and wrongly.
   integer ia, ic;
   reg [NENT-1:0] v_first;               // the live entries no live entry is older than: one
   reg [IDXB:0]   v_n, f_n;              // ...counted, and the live entries
   always @* begin
      v_n = {(IDXB+1){1'b0}};  f_n = {(IDXB+1){1'b0}};
      for (ia = 0; ia < NENT; ia = ia + 1) begin
         v_first[ia] = v[ia] & ~|(v & om[ia*NENT +: NENT]);
         v_n = v_n + {{IDXB{1'b0}}, v[ia]};
         f_n = f_n + {{IDXB{1'b0}}, v_first[ia]};
      end
   end
   integer ak, aj;
   always @(posedge clk) if (!reset) begin
      for (ak = 0; ak < NW; ak = ak + 1) if (d_alloc[ak]) begin
         if ({{(BL+1-RKB){1'b0}}, d_rank[ak*RKB +: RKB]} >= nbk)
            $fatal(1, "smolrv64_lq: load %0d of the row has rank %0d, and %0d banks have room (%0d live)", ak, d_rank[ak*RKB +: RKB], nbk, cnt);
         for (aj = ak + 1; aj < NW; aj = aj + 1)
            if (d_alloc[aj] & (d_rank[aj*RKB +: RKB] == d_rank[ak*RKB +: RKB]))
               $fatal(1, "smolrv64_lq: loads %0d and %0d of the row share rank %0d", ak, aj, d_rank[ak*RKB +: RKB]);
      end
      if ((|d_alloc) & kd_v & ~flush)
         $fatal(1, "smolrv64_lq: an allocation while the kill holds");
      if (n_alloc > NB[IDXB:0])
         $fatal(1, "smolrv64_lq: %0d loads in a row, %0d banks", n_alloc, NB);
      if (cnt != v_n)
         $fatal(1, "smolrv64_lq: the count %0d is not the live entries' %0d", cnt, v_n);
      if ((|v) && (f_n != {{IDXB{1'b0}}, 1'b1}))
         $fatal(1, "smolrv64_lq: the age matrix names %0d oldest live entries (v=%b)", f_n, v);
      if (a_v & ~v[a_idx])
         $fatal(1, "smolrv64_lq: address written to a slot with no live entry (idx %0d)", a_idx);
      if (a_v & av[a_idx])
         $fatal(1, "smolrv64_lq: address written twice to slot %0d", a_idx);
      if (l_v & (l_idx != (l_rsp ? l_tg : l_alt)))
         $fatal(1, "smolrv64_lq: the landing entry %0d is not the payload's pick (tag %0d, other %0d, response %b)",
                l_idx, l_tg, l_alt, l_rsp);
      if (l_v & ~v[l_idx])
         $fatal(1, "smolrv64_lq: data landed for a slot with no live entry (idx %0d)", l_idx);
      if (l_v & ~sent[l_idx])
         $fatal(1, "smolrv64_lq: data landed for an entry that was never sent to memory (idx %0d)", l_idx);
      if (x_v & x_take & (~av[x_idx] | sent[x_idx]))
         $fatal(1, "smolrv64_lq: started an access for entry %0d with no address or already sent", x_idx);
      if (a_v & a_sent & ~b_ok)
         $fatal(1, "smolrv64_lq: entry %0d started early but is not a live unfilled entry", a_idx);
      if (a_v & a_sent & x_v & x_take & (a_idx == x_idx))
         $fatal(1, "smolrv64_lq: entry %0d started early and as the candidate in one cycle", a_idx);
      for (ic = 0; ic < NENT; ic = ic + 1)
         if (a_cancel[ic] & v[ic] & ~sent[ic])
            $fatal(1, "smolrv64_lq: entry %0d's access cancelled but it was not sent", ic);
      if (a_v & a_sent & ~a_tv)
         $fatal(1, "smolrv64_lq: entry %0d started early without a translation", a_idx);
      if (x_v & x_take & ~tv[x_idx])
         $fatal(1, "smolrv64_lq: the candidate started without a translation");
      if (x_v & x_take & x_fwd & ~(e_fwd[x_idx] & mem[x_idx] & ~unc[x_idx] & ~fw_busy))
         $fatal(1, "smolrv64_lq: entry %0d forwards without a forwarding store (fwd=%b mem=%b unc=%b busy=%b)",
                x_idx, e_fwd[x_idx], mem[x_idx], unc[x_idx], fw_busy);
      if (w_v & ~(v[w_idx] & av[w_idx] & ~tv[w_idx]))
         $fatal(1, "smolrv64_lq: the walker's answer for entry %0d is not for a live untranslated entry", w_idx);
      if (w_v & a_v & (a_idx == w_idx))
         $fatal(1, "smolrv64_lq: entry %0d filled and translated in one cycle", w_idx);
   end
endmodule
`default_nettype wire
