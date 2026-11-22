`default_nettype none

// Register renaming for smolrv64_core: SMAP/RMAP + dirty[], and one free list per PRF shard. It
// gives smolrv64_prf a known destination shard, which is the only cheap way to get more than
// one write port (see smolrv64_prf.v's header).
//
// THE MAP.  docs/Area-Efficient-Scalar-OoO.md 9.2 recovers by bulk-copying rat_commit into
// map -- a 32 x PBITS parallel load.  On this FPGA that fanout is the failure mode: the
// restore value has to reach every map flop.  Instead:
//
//     read     : SMAP[a]
//     rename   : SMAP[rd] <= p_new ;  dirty[rd] <= 1
//     commit   : RMAP[rd] <= p_commit ;  a cycle later dirty[rd] <= 0 if SMAP[rd] is still p_commit
//     walk     : SMAP[rd] <= pold, each entry back to a mispredicted branch (dirty[rd] <= 1)
//     squash   : THE SWEEP -- each dirty register rewritten from RMAP, IW a cycle, rename stalled
//
// A register that is not dirty maps in SMAP to its committed mapping (asserted every cycle), so
// a total squash rewrites only what an uncommitted rename or walk touched: 6 registers on the
// 60 M boot (4.4 of them exactly), 2.4 cycles of IW from the squash's own cycle, mostly under
// the redirect's own freeze. Measured against restoring in one cycle: -0.09% retires on the
// 60 M boot, Navigation +0.07% and SQLite +0.00% cycles (walking back to the head would cost
// +1.4-2.3%). Every read sees SMAP alone, so no consumer looks up a second, committed
// candidate. SMAP and RMAP are arrays the tools infer as LUTRAM, one copy per writer (see the
// map below); dirty is 64 flops.
//
// THE FREE LISTS.  One per shard, because a physical register belongs to exactly one shard.
// This departs from docs/Area-Efficient-Scalar-OoO.md 9.1, which derives the tail from a
// FIXED occupancy ("free entries are always FREE - n_alloc"), true only because exactly 32
// architectural registers are mapped at all times.  Per shard the mapped count VARIES -- 0..32
// for every shard -- so each shard carries a real tail.
//
// Rollback is pointer-only: rename never writes the array (commit is its only writer), so the
// entries between the committed head and the speculative head still hold the in-flight
// allocations, and restoring the speculative head frees them all in one cycle, with no walk
// and no checkpoints.
//
// THE GROUP. Up to IW slots rename in a cycle, one per lane, in program order from the group's
// first lane r_fc on, cyclically (smolrv64_frontend rotates the lanes): slot k is at program
// position (k - r_fc) % IW. A source equal to an earlier slot's destination takes that slot's new
// register (the youngest such: slot k looks at slots k-1, k-2, ... cyclically, nearest first, as
// far back as its position), and a destination shadows an earlier slot's to the same register. A
// group allocates at most one register from each shard (lane k's integer destination is lane k's
// shard, and a group holds one FP load and one FP-unit op), so every slot takes its shard's head.
// IW commits in program order, port k the k-th.

module smolrv64_rename
  #(parameter IDXB  = 7,
    parameter PBITS = IDXB + 3,           // 3 shard bits (smolrv64_shards.vh)
    parameter N_INT = 64,                 // each lane's shard
    parameter N_FL  = 64,                 // the FP loads' shard
    parameter N_FF  = 64,                 // the F stage's shard
    parameter LOWAT = 4,                  // stall fetch when any shard has < LOWAT free
    parameter IW    = 2)                  // the group width -> FLNB = next_pow2(IW) free-list banks
   (input  wire                  clk,
    input  wire                  reset,

    // ---- rename: slot k at [k], the group a cyclic run of slots from r_fc ----
    input  wire [IW-1:0]         r_valid,      // slot k renames this cycle
    input  wire [((IW > 1) ? $clog2(IW) : 1)-1:0] r_fc,   // the slot of the group's first instruction
    // slot k holds an instruction, taken or not: what the group's bypass reads, so no new
    // register or bypassed source waits for the dispatch decision (a group is taken whole)
    input  wire [IW-1:0]         r_cand,
    input  wire [IW*6-1:0]       r_rs1, r_rs2, r_rs3, r_rd,
    input  wire [IW-1:0]         r_rd_v,       // writes a register (x0 destinations excluded)
    input  wire [IW*3-1:0]       r_shard,      // destination shard, from the instruction class
    output wire [IW*PBITS-1:0]   r_prs1, r_prs2, r_prs3,
    // The map's own read, before an earlier slot's bypass, so a lookup whose answer the bypass
    // decides anyway (readiness) starts at the map read; r_byp* says the source IS an earlier
    // slot's new register (pending by definition).
    output wire [IW*PBITS-1:0]   r_sprs1, r_sprs2, r_sprs3,
    output wire [IW-1:0]         r_byp1, r_byp2, r_byp3,      // slot 0's are 0
    output wire [IW*PBITS-1:0]   r_prd,        // the newly allocated physical register
    // the mapping slot k's destination displaces (an earlier slot's new register when the group
    // writes it twice): the ROB keeps it, and the walk below puts it back
    output wire [IW*PBITS-1:0]   r_pold,

    // ---- commit: the head and the entries behind it, in order ----
    input  wire [IW-1:0]         c_valid,
    input  wire [IW*6-1:0]       c_rd,
    input  wire [IW-1:0]         c_rd_v,
    input  wire [IW*PBITS-1:0]   c_prd,        // becomes the committed mapping
    input  wire [IW*PBITS-1:0]   c_pold_in,    // ...and the one it displaced, from the ROB: freed

    // ---- recovery ----
    input  wire                  flush,        // total squash: everything uncommitted dies; the sweep follows
    // The walk: a ROB row a cycle from the tail back to a mispredicted branch, column k on [k].
    // Each entry it names had renamed rd into prd and displaced pold; the map takes pold back
    // (the oldest column winning in a row) and prd's free list steps its head back over prd.
    // Rename is frozen while it runs, so the map's write ports and the heads are idle.
    input  wire                  w_run,        // the walk runs (a register): the map's write ports are its
    input  wire [IW-1:0]         w_v,
    input  wire [IW*6-1:0]       w_rd,
    input  wire [IW*PBITS-1:0]   w_pold, w_prd,
    // the walk has ended and the branch is committing with the flush: the speculative map and
    // heads must be the committed ones after this cycle's commits (checked, then flushed)
    input  wire                  w_chk,

    // ---- back pressure and instrumentation ----
    output wire                  stall,        // ANY shard low, or the sweep running -- see the notes below
    output wire [7:0]            shard_low);   // per shard, for the hpm counters

`include "smolrv64_shards.vh"
   function automatic sh_used(input integer sh);   // lanes 0..IW-1 and the two FP shards
      sh_used = (sh < SH_FL) ? (sh < IW) : ((sh == SH_FL) || (sh == SH_FF));
   endfunction
   localparam integer NB = (IW > 1) ? $clog2(IW) : 1;   // a slot or position number
   localparam integer NS = 3 * IW;                       // the map's source reads
   localparam integer NR = NS + IW;                      // ...and each slot's destination

   // ---- the map -------------------------------------------------------------------
   // IW COPIES OF THE SPECULATIVE MAP, one write port each: slot k writes copy k (as do the
   // walk's position k and the sweep's lane k). A LUTRAM has one write port, and IW renames per
   // cycle need IW; duplicating the array by WRITER is the map's version of the PRF's sharding
   // rule. A register's mapping is the XOR of the copies' words at it: a writer stores its
   // mapping XORed with the other copies' words there, so no table says which copy is current,
   // and a read is one XOR. Two copies never write one register in a cycle: of a group's renames
   // of a register the youngest writes, of a walked row's entries the oldest, the sweep's lanes
   // pick distinct registers, and the three never share a cycle (asserted). The COMMITTED map
   // likewise: commit k writes copy k, the last commit of a register in a cycle alone. RMAP is
   // read only by the sweep (and the checks).
   reg [63:0]      dirty;
   // every source read: slot k's rs1, rs2, rs3 at [3k], [3k+1], [3k+2] and its rd at [NS+k];
   // and every commit's displaced mapping, rmap[c_rd[k]], read before the commit's own write lands
   wire [5:0]       sa [0:NR-1];
   wire [PBITS-1:0] sv [0:IW*NR-1];          // copy c's speculative word at read p
   wire [PBITS-1:0] pv [0:IW*IW-1];          // copy c's committed word at commit k's rd
   wire [PBITS-1:0] so [0:IW*IW-1];          // copy c's speculative word at copy k's write address
   wire [5:0]       s_wa [0:IW-1];           // copy k's write address
   wire [IW-1:0]    shadowed;                // a younger slot of the group writes the same register
   wire [IW-1:0]    s_we, w_shad, c_shad;    // copy k writes; the walk's / the commits' entry k is not its register's last word
   wire [PBITS-1:0] xv [0:IW*IW-1];          // copy c's committed word at the sweep's lane k register
   wire [PBITS-1:0] cv [0:IW*IW-1];          // copy c's speculative word at the register commit k wrote
   // the sweep's write, a cycle after its pick (below): lane k rewrites register x_a[k]
   reg  [IW-1:0]    x_v;
   reg  [5:0]       x_a [0:IW-1];
   wire [PBITS-1:0] x_d [0:IW-1];            // ...with its committed mapping
   reg              x_busy;                  // the sweep runs (rename stalls)
   // a commit's dirty test, a cycle after the commit (registered: c_rd comes off the ROB's array)
   wire [IW-1:0]    cc_hit;                  // commit k's register is still its newest mapping
   reg  [IW-1:0]    cc_v;
   reg  [5:0]       cc_rd  [0:IW-1];
   reg  [PBITS-1:0] cc_prd [0:IW-1];
   genvar gk, gc, gp;
   generate for (gk = 0; gk < IW; gk = gk + 1) begin: src
      assign sa[3*gk]     = r_rs1[gk*6 +: 6];
      assign sa[3*gk + 1] = r_rs2[gk*6 +: 6];
      assign sa[3*gk + 2] = r_rs3[gk*6 +: 6];
      assign sa[NS + gk]  = r_rd[gk*6 +: 6];
   end endgenerate
`ifndef SYNTHESIS
   // The checks' committed map: a flat copy, and r's committed mapping after this cycle's commits
   // (the last commit of r wins).
   reg [PBITS-1:0] cm [0:63];
   integer cmi;
   initial for (cmi = 0; cmi < 32; cmi = cmi + 1) begin
      cm[cmi] = {SH_IE, cmi[IDXB-1:0]};  cm[32 + cmi] = {SH_FL, cmi[IDXB-1:0]};
   end
   reg [PBITS-1:0] cm_after [0:63];
   integer cmr, cmq;
   always @* for (cmr = 0; cmr < 64; cmr = cmr + 1) begin
      cm_after[cmr] = cm[cmr];
      for (cmq = 0; cmq < IW; cmq = cmq + 1)
         if (c_valid[cmq] & c_rd_v[cmq] & (c_rd[cmq*6 +: 6] == cmr[5:0])) cm_after[cmr] = c_prd[cmq*PBITS +: PBITS];
   end
   always @(posedge clk) if (!reset)
      for (cmi = 0; cmi < IW; cmi = cmi + 1)
         if (c_valid[cmi] & c_rd_v[cmi]) cm[c_rd[cmi*6 +: 6]] <= c_prd[cmi*PBITS +: PBITS];
`endif
   generate for (gc = 0; gc < IW; gc = gc + 1) begin: cp
      (* ram_style = "distributed" *) reg [PBITS-1:0] smap [0:63];
      (* ram_style = "distributed" *) reg [PBITS-1:0] rmap [0:63];
      integer j;
      // x0 -> physical 0 (SH_IE index 0), which smolrv64_prf hardwires to read zero and never
      // writes. Integer registers start in SH_IE, FP registers in SH_FL (see the reset note):
      // copy 0 holds the mappings, the others zero.
      localparam [2:0] I_IE = (gc == 0) ? SH_IE : 3'd0, I_FL = (gc == 0) ? SH_FL : 3'd0;
      initial for (j = 0; j < 32; j = j + 1) begin
         rmap[j]      = (gc == 0) ? {I_IE, j[IDXB-1:0]} : {PBITS{1'b0}};  smap[j]      = (gc == 0) ? {I_IE, j[IDXB-1:0]} : {PBITS{1'b0}};
         rmap[32 + j] = (gc == 0) ? {I_FL, j[IDXB-1:0]} : {PBITS{1'b0}};  smap[32 + j] = (gc == 0) ? {I_FL, j[IDXB-1:0]} : {PBITS{1'b0}};
      end
      wire sw = r_valid[gc] & r_rd_v[gc] & ~shadowed[gc];   // this slot's mapping is its register's newest
      wire ww = w_v[gc] & ~w_shad[gc];
      wire cw = c_valid[gc] & c_rd_v[gc] & ~c_shad[gc];
      // one write statement: this slot's rename, the walk's position gc, or the sweep's lane gc.
      // Registers choose the address (rule I6): the sweep's lane, the walk's run, else the rename.
      wire [5:0]       wa = x_v[gc] ? x_a[gc] : w_run ? w_rd[gc*6 +: 6] : r_rd[gc*6 +: 6];
      wire [PBITS-1:0] wd = x_v[gc] ? x_d[gc] : w_run ? w_pold[gc*PBITS +: PBITS] : r_prd[gc*PBITS +: PBITS];
      reg  [PBITS-1:0] wx, cx;                     // the words to store: the mapping XOR the other copies' words
      integer          o;
      always @* begin
         wx = wd;  cx = c_prd[gc*PBITS +: PBITS];
         for (o = 0; o < IW; o = o + 1) if (o != gc) begin
            wx = wx ^ so[o*IW + gc];
            cx = cx ^ pv[o*IW + gc];
         end
      end
      assign s_wa[gc] = wa;
      assign s_we[gc] = sw | ww | x_v[gc];
      always @(posedge clk) if (!reset & s_we[gc]) smap[wa] <= wx;
      // Vivado builds a LUTRAM of at most 15 reads and turns a wider one into flops: past that
      // (IW >= 4), a second copy written alike serves the destinations', the commits' and the
      // writers' reads.
      wire [PBITS-1:0] dv [0:3*IW-1];
      if (NR + 2*IW > 15) begin: dmap
         (* ram_style = "distributed" *) reg [PBITS-1:0] smap_d [0:63];
         initial for (j = 0; j < 32; j = j + 1) begin
            smap_d[j]      = (gc == 0) ? {I_IE, j[IDXB-1:0]} : {PBITS{1'b0}};
            smap_d[32 + j] = (gc == 0) ? {I_FL, j[IDXB-1:0]} : {PBITS{1'b0}};
         end
         always @(posedge clk) if (!reset & s_we[gc]) smap_d[wa] <= wx;
         for (gp = 0; gp < IW; gp = gp + 1) begin: r
            assign dv[gp]        = smap_d[sa[NS + gp]];
            assign dv[IW + gp]   = smap_d[cc_rd[gp]];
            assign dv[2*IW + gp] = smap_d[s_wa[gp]];
         end
      end else begin: dmap
         for (gp = 0; gp < IW; gp = gp + 1) begin: r
            assign dv[gp]        = smap[sa[NS + gp]];
            assign dv[IW + gp]   = smap[cc_rd[gp]];
            assign dv[2*IW + gp] = smap[s_wa[gp]];
         end
      end
      always @(posedge clk) if (!reset & cw) rmap[c_rd[gc*6 +: 6]] <= cx;
      always @(posedge clk) if (!reset & ((r_valid[gc] & r_rd_v[gc] & (w_run | x_v[gc])) | (w_v[gc] & (x_v[gc] | ~w_run))))
         $fatal(1, "smolrv64_rename: map copy %0d is written twice (rename %b, walk %b of %b, sweep %b)",
                gc, r_valid[gc] & r_rd_v[gc], w_v[gc], w_run, x_v[gc]);
      for (gp = 0; gp < NS; gp = gp + 1) begin: rd
         assign sv[gc*NR + gp] = smap[sa[gp]];
      end
      for (gp = 0; gp < IW; gp = gp + 1) begin: pd
         assign sv[gc*NR + NS + gp] = dv[gp];
         assign pv[gc*IW + gp] = rmap[c_rd[gp*6 +: 6]];
         assign xv[gc*IW + gp] = rmap[x_a[gp]];
         assign cv[gc*IW + gp] = dv[IW + gp];
         assign so[gc*IW + gp] = dv[2*IW + gp];
      end
   end endgenerate
   // an entry is not its register's last word: an older entry of the walked row restores the register
   // too (the oldest wins), or a later commit of the cycle writes it
   generate for (gk = 0; gk < IW; gk = gk + 1) begin: shd
      reg     ws_, cs_;
      integer y;
      always @* begin
         ws_ = 1'b0;  cs_ = 1'b0;
         for (y = 0; y < IW; y = y + 1) begin
            if ((y < gk) & w_v[y] & (w_rd[y*6 +: 6] == w_rd[gk*6 +: 6])) ws_ = 1'b1;
            if ((y > gk) & c_valid[y] & c_rd_v[y] & (c_rd[y*6 +: 6] == c_rd[gk*6 +: 6])) cs_ = 1'b1;
         end
      end
      assign w_shad[gk] = ws_;  assign c_shad[gk] = cs_;
   end endgenerate
   integer wi, wj;
   always @(posedge clk) if (!reset)
      for (wi = 0; wi < IW; wi = wi + 1)
         for (wj = wi + 1; wj < IW; wj = wj + 1)
            if (s_we[wi] & s_we[wj] & (s_wa[wi] == s_wa[wj]))
               $fatal(1, "smolrv64_rename: map copies %0d and %0d both write r%0d", wi, wj, s_wa[wi]);
   // each read's mapping: the XOR of the copies' words (a wire per read: rule F4)
   wire [PBITS-1:0] sp [0:NR-1];
   wire [PBITS-1:0] cq [0:IW-1];              // the speculative mapping of the register commit k wrote
   wire [PBITS-1:0] pq [0:IW-1];              // the committed mapping of commit k's rd
   generate
      for (gp = 0; gp < NR; gp = gp + 1) begin: sel
         wire [PBITS-1:0] xs [0:IW];
         assign xs[0] = {PBITS{1'b0}};
         for (gc = 0; gc < IW; gc = gc + 1) begin: c
            assign xs[gc + 1] = xs[gc] ^ sv[gc*NR + gp];
         end
         assign sp[gp] = xs[IW];
      end
      for (gp = 0; gp < IW; gp = gp + 1) begin: sel2
         wire [PBITS-1:0] xx [0:IW], xc [0:IW], xp [0:IW];
         assign xx[0] = {PBITS{1'b0}};  assign xc[0] = {PBITS{1'b0}};  assign xp[0] = {PBITS{1'b0}};
         for (gc = 0; gc < IW; gc = gc + 1) begin: c
            assign xx[gc + 1] = xx[gc] ^ xv[gc*IW + gp];
            assign xc[gc + 1] = xc[gc] ^ cv[gc*IW + gp];
            assign xp[gc + 1] = xp[gc] ^ pv[gc*IW + gp];
         end
         assign x_d[gp] = xx[IW];  assign cq[gp] = xc[IW];  assign pq[gp] = xp[IW];
      end
   endgenerate
`ifndef SYNTHESIS
   // every register against the committed map: RMAP after the walk; SMAP after the walk (every
   // register: a mapping the walk did not restore was committed or clean), and SMAP whenever no
   // sweep is in flight for a register not dirty
   wire [63:0] sw_neq;                         // SWEEP-SIM: the registers whose newest mapping is not the committed one
   genvar gq;
   generate for (gq = 0; gq < 64; gq = gq + 1) begin: fl_chk
      wire [PBITS-1:0] sx [0:IW], rx [0:IW];
      assign sx[0] = {PBITS{1'b0}};  assign rx[0] = {PBITS{1'b0}};
      for (gc = 0; gc < IW; gc = gc + 1) begin: c
         assign sx[gc + 1] = sx[gc] ^ cp[gc].smap[gq];
         assign rx[gc + 1] = rx[gc] ^ cp[gc].rmap[gq];
      end
      assign sw_neq[gq] = sx[IW] != cm_after[gq];
      always @(posedge clk) if (!reset) begin
         if (w_chk && rx[IW] != cm[gq])
            $fatal(1, "smolrv64_rename: rmap holds %h for r%0d, the committed map %h", rx[IW], gq, cm[gq]);
         if (w_chk && sx[IW] != cm_after[gq])
            $fatal(1, "smolrv64_rename: after the walk r%0d maps to %h, committed %h", gq, sx[IW], cm_after[gq]);
         if (~x_busy & ~(|x_v) & ~dirty[gq] && sx[IW] != cm[gq])
            $fatal(1, "smolrv64_rename: r%0d is clean but maps to %h, committed %h", gq, sx[IW], cm[gq]);
      end
   end endgenerate
`endif
   // Reads are of the PRE-rename mapping, including a source equal to the slot's own
   // destination: rd is written at the clock edge, so the reads see the old value. An earlier
   // slot's destination is bypassed in: the youngest earlier slot naming the source wins. Slot k
   // is at position ps[k], and the slot d behind it (cyclically) is earlier when d <= ps[k]: the
   // nearest wins, so the slots are looked at farthest first.
   wire [IW-1:0] writes = r_cand & r_rd_v;   // candidates, not takes
   wire [IW-1:0] alloc  = r_valid & r_rd_v;
   reg  [NB-1:0] ps [0:IW-1];                // slot k's program position in the group
   integer       pk2;
   always @* for (pk2 = 0; pk2 < IW; pk2 = pk2 + 1)
      ps[pk2] = (pk2[NB-1:0] >= r_fc) ? pk2[NB-1:0] - r_fc : pk2[NB-1:0] + IW[NB-1:0] - r_fc;
   wire [PBITS-1:0] np [0:NS-1];               // the youngest earlier slot's new register
   wire [NS-1:0]    byp;
   generate for (gp = 0; gp < NS; gp = gp + 1) begin: by
      localparam integer K = gp / 3;           // the source's slot
      reg              b;
      reg [PBITS-1:0]  n;
      integer d, j;
      always @* begin
         b = 1'b0;  n = {PBITS{1'b0}};
         for (d = IW - 1; d >= 1; d = d - 1) begin
            j = (K + IW - d) % IW;
            if ((d <= ps[K]) & writes[j] & (sa[gp] == r_rd[j*6 +: 6])) begin b = 1'b1;  n = r_prd[j*PBITS +: PBITS]; end
         end
      end
      assign byp[gp] = b;  assign np[gp] = n;
   end endgenerate
   generate for (gk = 0; gk < IW; gk = gk + 1) begin: out
      assign r_sprs1[gk*PBITS +: PBITS] = sp[3*gk];
      assign r_sprs2[gk*PBITS +: PBITS] = sp[3*gk + 1];
      assign r_sprs3[gk*PBITS +: PBITS] = sp[3*gk + 2];
      assign r_byp1[gk] = byp[3*gk];  assign r_byp2[gk] = byp[3*gk + 1];  assign r_byp3[gk] = byp[3*gk + 2];
      assign r_prs1[gk*PBITS +: PBITS] = byp[3*gk]     ? np[3*gk]     : sp[3*gk];
      assign r_prs2[gk*PBITS +: PBITS] = byp[3*gk + 1] ? np[3*gk + 1] : sp[3*gk + 1];
      assign r_prs3[gk*PBITS +: PBITS] = byp[3*gk + 2] ? np[3*gk + 2] : sp[3*gk + 2];
      // the destination's displaced mapping: the youngest earlier slot writing it, else the map's
      reg              db;
      reg [PBITS-1:0]  dn;
      integer          dd, dj;
      always @* begin
         db = 1'b0;  dn = {PBITS{1'b0}};
         for (dd = IW - 1; dd >= 1; dd = dd - 1) begin
            dj = (gk + IW - dd) % IW;
            if ((dd <= ps[gk]) & writes[dj] & (r_rd[dj*6 +: 6] == r_rd[gk*6 +: 6])) begin db = 1'b1;  dn = r_prd[dj*PBITS +: PBITS]; end
         end
      end
      // ...and whether a younger slot writes it too: then this one's mapping is not the newest
      reg              sh_y;
      integer          yj;
      always @* begin
         sh_y = 1'b0;
         for (yj = 0; yj < IW; yj = yj + 1)
            if ((yj != gk) & alloc[yj] & (ps[yj] > ps[gk]) & (r_rd[yj*6 +: 6] == r_rd[gk*6 +: 6])) sh_y = 1'b1;
      end
      assign shadowed[gk] = sh_y;
      assign r_pold[gk*PBITS +: PBITS] = db ? dn : sp[NS + gk];
      // whether commit k's register is still its newest mapping
      assign cc_hit[gk] = cc_v[gk] & (cq[gk] == cc_prd[gk]);
   end endgenerate

   // ---- THE SWEEP -----------------------------------------------------------------
   // A total squash (flush, or reset) leaves SMAP holding uncommitted mappings in the dirty
   // registers, and RMAP the committed state after the squash's own commits. From the squash's
   // own cycle on, the sweep takes the IW lowest dirty registers each cycle and clears their bits,
   // the k-th to lane k; a cycle later lane k's register is rewritten through SMAP copy k
   // from RMAP. The picks come from
   // flops (lane k isolates the lowest dirty register the lanes before it left), and the write's
   // address is the registered pick (rule I6). Starting in the squash's cycle, the first writes
   // land while dispatch is frozen for the redirect anyway (redirect_q), so a squash with at most
   // 2*IW dirty registers costs no cycle. Nothing commits meanwhile (the squash
   // emptied the ROB) and nothing renames (the stall holds the queue's pop; a group popped
   // before the squash is the squash's to drop), so neither map moves under it. x_busy drops
   // after the last pick, so the queue may pop in the cycle the last write lands and the group
   // renames after it.
   localparam integer NG = (64 + IW - 1) / IW;       // a bound on the sweep's cycles
   wire          x_on = (flush | x_busy) & ~reset;
   reg  [IW-1:0] p_v;                                // lane k has a register...
   reg  [6*IW-1:0] p_a;                              // ...the k-th lowest dirty one
   reg  [63:0]   p_m;                                // the picked registers
   // lane k isolates the lowest set bit of what the lanes before it left (d & -d: one carry
   // chain a lane), so the picks are IW short chains, not a count across all 64 registers
   wire [63:0]   p_d [0:IW];                         // what lane k picks from
   wire [63:0]   p_1 [0:IW-1];                       // lane k's pick, one-hot
   assign p_d[0] = dirty;
   genvar gx;
   generate for (gx = 0; gx < IW; gx = gx + 1) begin: xp
      assign p_1[gx]     = p_d[gx] & (~p_d[gx] + 64'd1);
      assign p_d[gx + 1] = p_d[gx] & ~p_1[gx];
   end endgenerate
   integer       xl, xr;
   always @* begin
      p_m = dirty & ~p_d[IW];  p_v = {IW{1'b0}};  p_a = {(6*IW){1'b0}};
      for (xl = 0; xl < IW; xl = xl + 1) begin
         p_v[xl] = |p_1[xl];
         for (xr = 0; xr < 64; xr = xr + 1)
            if (p_1[xl][xr]) p_a[xl*6 +: 6] = p_a[xl*6 +: 6] | xr[5:0];
      end
   end
   integer xk;
   always @(posedge clk)
      for (xk = 0; xk < IW; xk = xk + 1) begin
         x_v[xk] <= x_on & p_v[xk];
         x_a[xk] <= p_a[xk*6 +: 6];
      end

   // ---- the dirty registers: what the sweep must rewrite ------------------------------
   // Set by a rename and by the walk (the walk's restored mapping may still be uncommitted);
   // cleared as the sweep picks, and a cycle after a commit when the committed register is still
   // the register's newest mapping (no younger rename of it in flight). A stale set bit costs a
   // sweep step, never a wrong mapping; reset marks every register (the total squash's sweep
   // then restores the map whatever a runtime reset interrupted).
   reg  [63:0]   dirty_n;
   integer       dk;
   always @* begin
      dirty_n = dirty;
      if (x_on) dirty_n = dirty_n & ~p_m;
      for (dk = 0; dk < IW; dk = dk + 1)
         if (cc_hit[dk]) dirty_n[cc_rd[dk]] = 1'b0;
      for (dk = 0; dk < IW; dk = dk + 1)
         if (r_valid[dk] & r_rd_v[dk]) dirty_n[r_rd[dk*6 +: 6]] = 1'b1;
      for (dk = 0; dk < IW; dk = dk + 1)
         if (w_v[dk]) dirty_n[w_rd[dk*6 +: 6]] = 1'b1;
      if (reset) dirty_n = {{63{1'b1}}, 1'b0};        // x0 is never renamed
   end
   always @(posedge clk) begin
      dirty  <= dirty_n;
      x_busy <= reset | (x_on & (|dirty_n));          // dirty registers left after this cycle's picks
      for (dk = 0; dk < IW; dk = dk + 1) begin
         cc_v[dk]   <= ~reset & c_valid[dk] & c_rd_v[dk];
         cc_rd[dk]  <= c_rd[dk*6 +: 6];
         cc_prd[dk] <= c_prd[dk*PBITS +: PBITS];
      end
   end

   // ---- free lists, one per shard ---------------------------------------------------
   // Shard s's list is the generate block fl[s]: a circular FIFO of the shard's free indices,
   // with a speculative head h (allocation), a committed head hc (the rollback target) and a
   // tail t (frees). Pointers carry an extra MSB so full and empty are distinguishable without
   // a count: empty when h == t, full when h == t ^ {1'b1, 0}.
   //
   // FLNB = next_pow2(IW) banks per list, one muxed write per bank: a cycle's up to IW frees
   // land at t, t+1, ..., which fall in different banks. Bank g reads its next free entry at
   // h's index plus one when g is behind h's bank -- REGISTERS ONLY on the read address. Which
   // slot allocates from which shard (the class decode) selects among the banks' OUTPUTS, not
   // their addresses, and the stall never reaches an address: with it there, the whole rename
   // decision sat in front of the LUTRAM read, and the read's data in front of the dispatch
   // stage and the store queue. Entries read and not consumed are simply read again.
   //
   // NO FUNCTION READS THESE ARRAYS (rule F4): Vivado keeps one read port for a function that
   // reads a RAM, the last call site's, and folds the earlier calls to 0.
   //
   // The FP shards hold f0-f31 and nothing else, one per writer: an FP load's destination
   // allocates from SH_FL, any other f-register from SH_FF.
   localparam integer NSH  = 8;
   localparam integer FLNB = 1 << $clog2(IW);   // free-list banks = next_pow2(IW); >=2
   localparam integer FLLB = $clog2(FLNB);
   localparam integer CW   = $clog2(IW + 1);    // a count of 0..IW
   // A shard's size, and the first free index at reset: SH_IE and SH_FL hold the reset
   // mappings of x0-x31 and f0-f31 at indices 0..31, so their lists start at 32.
   function automatic integer n_of(input integer sh);
      n_of = (sh < SH_FL) ? N_INT : (sh == SH_FL) ? N_FL : N_FF;
   endfunction
   function automatic integer base_of(input integer sh);
      base_of = (sh == SH_IE || sh == SH_FL) ? 32 : 0;
   endfunction

   // Commit and flush can land in the SAME cycle: a mispredicting branch commits while the
   // instructions behind it are squashed.  The restore target must therefore be the
   // POST-commit committed head (hc_n below), not hc.
   // TWO INDEPENDENT SHARDS PER COMMIT.  c_prd's shard is where the instruction ALLOCATED (so
   // it says which head advanced), but the displaced register c_pold belongs to whichever
   // shard last wrote that architectural register -- x29 written by lane A and then by lane B
   // displaces an SH_IE register while allocating an SH_IE2 one.  A register's shard is
   // encoded in its number and never changes, so the free push is routed by c_pold's own
   // shard; routing it by the allocation shard would move registers between shards.
   // The register a commit frees is the mapping it displaced, which the ROB recorded at rename
   // (pold): commits are in program order, so it is what the committed map holds for c_rd at the
   // commit -- after this cycle's earlier commits. The check reads it there.
   wire [IW-1:0]    c_w = c_valid & c_rd_v;
   reg  [PBITS-1:0] c_pold [0:IW-1];
   integer          pk, pj;
   always @* for (pk = 0; pk < IW; pk = pk + 1) c_pold[pk] = c_pold_in[pk*PBITS +: PBITS];
`ifndef SYNTHESIS
   reg  [PBITS-1:0] c_pold_rm;
   always @(posedge clk) if (!reset)
      for (pk = 0; pk < IW; pk = pk + 1) if (c_w[pk]) begin
         c_pold_rm = pq[pk];
         for (pj = 0; pj < pk; pj = pj + 1)
            if (c_w[pj] & (c_rd[pj*6 +: 6] == c_rd[pk*6 +: 6])) c_pold_rm = c_prd[pj*PBITS +: PBITS];
         if (c_pold[pk] != c_pold_rm)
            $fatal(1, "smolrv64_rename: commit %0d frees p%h, the committed map displaces p%h", pk, c_pold[pk], c_pold_rm);
      end
`endif

   // ---- FLOP THE REGISTER RELEASE -------------------------------------------------
   // The free (write + tail advance) is registered one cycle off the retire cone: c_valid rides
   // M's completion into retire, and fanning it to every free list's write ports and tails
   // combinationally was the IW=3 build's largest near-critical family. The register is released
   // one cycle later, which is free: the lists are never the allocation bottleneck, and
   // avail = t - h counts a committed free one cycle later, which is strictly MORE
   // conservative. ROLLBACK-SAFE WITH NO EXTRA HANDLING: the tail advances only on committed
   // frees and flush never touches it, so a pending free is always a committed one that must
   // complete. c_pold is captured HERE, at commit, while RMAP[c_rd] still holds the old mapping.
   reg [NSH-1:0]    fre_q [0:IW-1];
   reg [PBITS-1:0]  c_pold_q [0:IW-1];
   integer          fk;
   initial for (fk = 0; fk < IW; fk = fk + 1) begin fre_q[fk] = {NSH{1'b0}}; c_pold_q[fk] = {PBITS{1'b0}}; end

   // THE FREE-LIST READ ADDRESSES DO NOT SEE THE STALL: a list's next FLNB entries are read from
   // its pointer alone, and alloc advances it. alloc_r is the candidates': a group may hold up to IW
   // of one list's (the FP shards'), and the k-th of them in program order takes the head's k-th entry.
   wire [IW-1:0] alloc_r = r_cand & r_rd_v;

   wire [IDXB-1:0] rdx [0:NSH*IW-1];   // the register list s hands slot k: rdx[s*IW + k]
   wire [NSH-1:0]  low_n;              // each list's low-water flag for next cycle
   wire [NSH-1:0]  fre_n [0:IW-1];     // commit k frees into list s

   genvar gL, gB;
   generate for (gL = 0; gL < NSH; gL = gL + 1) begin: fl
      localparam [2:0] SHL = gL;
      wire [IW-1:0] sel_r, sel, cmt, wsel;
      for (gk = 0; gk < IW; gk = gk + 1) begin: s
         assign sel_r[gk] = alloc_r[gk] & (r_shard[gk*3 +: 3] == SHL);
         assign sel[gk]   = alloc[gk]   & (r_shard[gk*3 +: 3] == SHL);
         assign wsel[gk]  = w_v[gk] & (w_prd[gk*PBITS + IDXB +: 3] == SHL);   // the walk returns prd
         assign cmt[gk]   = c_w[gk] & (c_prd[gk*PBITS + IDXB +: 3] == SHL);   // head advance: the allocation's shard
         assign fre_n[gk][gL] = c_w[gk] & (c_pold[gk][PBITS-1:IDXB] == SHL);   // free push: the register's shard
      end
    if (!sh_used(gL)) begin: none   // a shard this width does not use: no list, never low
      for (gk = 0; gk < IW; gk = gk + 1) begin: z
         assign rdx[gL*IW + gk] = {IDXB{1'b0}};
      end
      assign low_n[gL] = 1'b0;
    end else begin: u
      localparam integer N  = n_of(gL);
      localparam integer PW = $clog2(N) + 1;
      localparam integer  NFREE = N - base_of(gL);   // the initially free entries: base..N-1
      localparam [PW-1:0] T0 = NFREE[PW-1:0];
      function automatic [PW-1:0] cnt(input [IW-1:0] x);
         integer i;
         begin cnt = {PW{1'b0}}; for (i = 0; i < IW; i = i + 1) cnt = cnt + {{(PW-1){1'b0}}, x[i]}; end
      endfunction

      reg [PW-1:0] h, hc, t;
      wire [PW-1:0] hc_n = hc + cnt(cmt);
      // this cycle's frees, registered: commit k's lands after the earlier commits'
      wire [IW-1:0] fq;
      for (gk = 0; gk < IW; gk = gk + 1) begin: q
         assign fq[gk] = fre_q[gk][gL];
      end
      wire [PW-1:0] avail = t - h;
      wire [PW-1:0] t_n = reset ? t : t + cnt(fq);
      wire [PW-1:0] h_n = reset ? hc : flush ? hc_n : h + cnt(sel) - cnt(wsel);
      // a lane's shard gets one register a group, an FP shard up to IW: two groups' worth
      localparam integer LW = (gL >= SH_FL) ? ((LOWAT > 2*IW) ? LOWAT : 2*IW) : LOWAT;
      // THE LOW-WATER FLAG READS REGISTERS ONLY (see `stall`): the list's tail after this cycle's
      // frees (registered a cycle ago) and its head after the allocation of the group the IR holds
      // (sel_r: its slots' destinations, from the IR's registers), against the mark. The group
      // dispatches this cycle or is dropped, so counting it without the dispatch take errs toward
      // the stall, and the flush and the walk only return registers: not low here is at least LW free
      // after this cycle, as with the allocation itself.
      wire [PW-1:0] fr_a = t_n - h - cnt(sel_r);     // modulo the pointers' width: they wrap
      assign low_n[gL] = fr_a < LW[PW-1:0];

      wire [IDXB-1:0]  flrd [0:FLNB-1];
      wire [FLNB-1:0]  behind = ~({FLNB{1'b1}} << h[FLLB-1:0]);   // behind[g] = (g < h's bank): the banks that wrapped
      for (gB = 0; gB < FLNB; gB = gB + 1) begin: bank
         (* ram_style = "distributed" *) reg [IDXB-1:0] mem [0:N/FLNB-1];
         integer jj; integer pp;
         initial for (jj = 0; jj < N/FLNB; jj = jj + 1) begin pp = base_of(gL) + FLNB*jj + gB; mem[jj] = pp[IDXB-1:0]; end
         // free k lands at t + (the earlier frees): at most one lands in this bank
         reg              wen;
         reg [PW-2:0]     wpos;
         reg [IDXB-1:0]   wd;
         reg [PW-2:0]     tk;
         integer          k;
         always @* begin
            wen = 1'b0;  wpos = t[PW-2:0];  wd = c_pold_q[0][IDXB-1:0];  tk = t[PW-2:0];
            for (k = 0; k < IW; k = k + 1) begin
               if (fq[k] & (tk[FLLB-1:0] == gB[FLLB-1:0]) & ~wen) begin
                  wen = 1'b1;  wpos = tk;  wd = c_pold_q[k][IDXB-1:0];
               end
               tk = tk + {{(PW-2){1'b0}}, fq[k]};
            end
         end
         always @(posedge clk) if (wen) mem[wpos[PW-2:FLLB]] <= wd;
         assign flrd[gB] = mem[h[PW-2:FLLB] + {{(PW-2-FLLB){1'b0}}, behind[gB]}];
      end
      // slot k takes the head's entry offset by the list's candidates AHEAD OF IT IN PROGRAM ORDER:
      // the committed head advances as a row retires and the walk backs a row out youngest first,
      // both in program order, so a row's registers leave the list in that order
      for (gk = 0; gk < IW; gk = gk + 1) begin: o
         reg [PW-1:0] pre;
         integer      j;
         always @* begin
            pre = {PW{1'b0}};
            for (j = 0; j < IW; j = j + 1) if (sel_r[j] & (ps[j] < ps[gk])) pre = pre + {{(PW-1){1'b0}}, 1'b1};
         end
         assign rdx[gL*IW + gk] = flrd[h[FLLB-1:0] + pre[FLLB-1:0]];
      end

      // The pointers come from configuration, like the arrays (see the reset note below).
      initial begin h = {PW{1'b0}}; hc = {PW{1'b0}}; t = T0; end
      always @(posedge clk) begin
         h <= h_n;  t <= t_n;
         if (!reset) hc <= hc_n;
      end

      // Allocating past the free set would hand out a register that is still live; the stall
      // is supposed to make it unreachable, and "supposed to" is what assertions are for.
      always @(posedge clk) if (!reset) begin
         if (cnt(sel) > avail)
            $fatal(1, "smolrv64_rename: shard %0d allocated past its free list (%0d free)", gL, avail);
         if (avail > N[PW-1:0])
            $fatal(1, "smolrv64_rename: shard %0d has %0d free of %0d", gL, avail, N);
         if (w_chk && (h != hc_n))
            $fatal(1, "smolrv64_rename: after the walk shard %0d's head is %0d, its committed head %0d", gL, h, hc_n);
      end
      // Sizes MUST be powers of two: the pointers index with their low bits, which only wraps
      // correctly at a power of two.
      initial begin
         if ((N & (N - 1)) != 0) $fatal(1, "smolrv64_rename: shard %0d size %0d is not a power of two", gL, N);
         if (N > (1 << IDXB))    $fatal(1, "smolrv64_rename: shard %0d size %0d exceeds IDXB=%0d", gL, N, IDXB);
      end
    end
   end endgenerate

   always @(posedge clk)
      for (fk = 0; fk < IW; fk = fk + 1) begin
         fre_q[fk]    <= reset ? {NSH{1'b0}} : fre_n[fk];
         c_pold_q[fk] <= c_pold[fk];
      end

   // STALL WHEN *ANY* SHARD IS LOW, not when the destination's shard is.  A shard that runs
   // dry stalls rename regardless of which one the next instruction wants, so throttling on
   // the minimum is what actually prevents the stall; throttling per-destination only
   // discovers it one instruction too late.  The cost is that the stall probability is the
   // union across shards -- the argument for sizing them unequally rather than adding more.
   // THE STALL IS A REGISTER: next cycle's low-water flags, from the pointers' registers and the
   // registered frees (fl.u.low_n), so neither the dispatch take nor the commit and flush cone
   // stands in front of it. It holds the queue's POP;
   // a group already popped dispatches the next cycle whatever the stall says, and rename
   // allocates for every slot that dispatches (LOWAT covers that group and the one dispatching).
   // An allocation the stall suppressed would hand the next group the same registers.
   reg [NSH-1:0] low;
   initial low = {NSH{1'b0}};
   always @(posedge clk) low <= low_n;
   assign shard_low = low;
   // ...and while the sweep runs: x_busy is a register too, and it drops after the sweep's last
   // pick, so a group popped then renames after the last write lands.
   assign stall = |low | x_busy;

   generate for (gk = 0; gk < IW; gk = gk + 1) begin: pr
      wire [2:0] sh = r_shard[gk*3 +: 3];
      assign r_prd[gk*PBITS +: PBITS] = {sh, rdx[sh*IW + gk]};
   end endgenerate

   // THE FREE LISTS AND THE MAPS ARE INITIALISED BY THE BITSTREAM AND NEVER RESET. No RAM can be
   // written at every address in one cycle, and every one of them is one-write/few-read once
   // running: the free lists are circular FIFOs read at the head and written at the tail, and the
   // maps are read at a few indices (the sources, rmap[c_rd], the sweep's) and written at one per
   // copy. On an FPGA the contents come from configuration for free (rule I7).
   //
   // THE VALUES ONLY HAVE TO BE A PERMUTATION of the shard's indices; which permutation is
   // irrelevant, because a free list only ever moves entries around. Identity is used.
   //
   // WHY A RUNTIME RESET IS STILL SAFE. `ui_cpu_reset` is a RUNTIME reset, so a button press
   // restarts the core without reconfiguring and the arrays keep the PREVIOUS run's contents.
   // That is sound because the contents are only meaningful through the pointers, and slots
   // [h, t) still hold exactly the free set. The pointers are therefore not reset either --
   // resetting them over stale slots would republish already-allocated registers as free,
   // handing out DUPLICATE physical registers. Reset instead does what `flush` does (h := hc),
   // which reclaims everything renamed but uncommitted, so nothing leaks across a restart, and
   // the sweep then restores SMAP from RMAP (every register dirty).
   //
   // x0 is safe across all of this: x0 destinations are excluded from rename (r_rd_v) and x0 is
   // never dirty, so rmap[0]/smap[0] are never written and keep the {SH_IE,0} that smolrv64_prf
   // hardwires to read zero.
   integer j;
   initial begin
      dirty = 64'd0;  x_busy = 1'b0;  x_v = {IW{1'b0}};  cc_v = {IW{1'b0}};
   end

`ifndef SYNTHESIS
   // SWEEP-SIM: at each total squash, in its cycle (what the sweep starts from): the dirty
   // registers, the exactly dirty ones (newest mapping not the committed one after the squash's
   // commits), the dirty ones only the walk set and not exactly dirty; the cycles the sweep picks
   // in, and those it stalls rename.
   reg [63:0] ws;                       // set by the walk, not since by a rename or a pick
   reg        fl_q;
   reg [63:0] sw_n, sw_d, sw_x, sw_w, sw_g, sw_c;
   integer    sq, sg;
   reg [7:0]  pd, px, pw;
   initial begin ws = 64'd0; fl_q = 1'b0; sw_n = 0; sw_d = 0; sw_x = 0; sw_w = 0; sw_g = 0; sw_c = 0; end
   always @* begin
      pd = 0; px = 0; pw = 0;
      for (sq = 0; sq < 64; sq = sq + 1) begin
         pd = pd + {7'd0, dirty[sq]};
         px = px + {7'd0, sw_neq[sq]};
         pw = pw + {7'd0, dirty[sq] & ws[sq] & ~sw_neq[sq]};
      end
   end
   always @(posedge clk) begin
      fl_q <= ~reset & flush;
      if (x_on) ws <= ws & ~p_m;
      for (sq = 0; sq < IW; sq = sq + 1) if (r_valid[sq] & r_rd_v[sq]) ws[r_rd[sq*6 +: 6]] <= 1'b0;
      for (sq = 0; sq < IW; sq = sq + 1) if (w_v[sq]) ws[w_rd[sq*6 +: 6]] <= 1'b1;
      if (!reset && flush) begin
         sw_n <= sw_n + 1;  sw_d <= sw_d + pd;  sw_x <= sw_x + px;  sw_w <= sw_w + pw;
      end
      if (x_on && (|p_v)) sw_g <= sw_g + 1;
      if (!reset && x_busy) sw_c <= sw_c + 1;
   end
`endif
   // ---- invariants: ALWAYS ON, per docs/rtl-rules.md ---------------------------------
   // The sweep holds the map: nothing renames or walks while it runs, and it ends within a pick
   // per group (nothing sets a dirty bit meanwhile).
   reg [7:0] x_age;
   initial x_age = 8'd0;
   always @(posedge clk) x_age <= (reset | ~x_busy) ? 8'd0 : x_age + 8'd1;
   always @(posedge clk) if (!reset) begin
      if ((|alloc) & x_busy)
         $fatal(1, "smolrv64_rename: a group renames while the sweep runs (dirty %h)", dirty);
      if ((|w_v) & x_busy)
         $fatal(1, "smolrv64_rename: the walk writes the map while the sweep runs");
      if (x_age > NG + 2)
         $fatal(1, "smolrv64_rename: the sweep has run %0d cycles (dirty %h)", x_age, dirty);
   end
   integer ik;
   always @(posedge clk) if (!reset) begin
      for (ik = 0; ik < IW; ik = ik + 1) begin
         // c_pold may legitimately belong to a DIFFERENT shard than c_prd (see the free
         // push above); what must hold is that the shards exist, so a register is never dropped.
         if (c_w[ik] && !(|fre_n[ik]))
            $fatal(1, "smolrv64_rename: commit %0d frees pr=%h of no shard", ik, c_pold[ik]);
         // the commits are one run of ports (a ROB row from its head's column)
         if (ik > 1 && c_valid[ik] && !c_valid[ik-1] && |(c_valid & ((1 << (ik - 1)) - 1)))
            $fatal(1, "smolrv64_rename: commit %0d is not in one run with the commits before it", ik);
         // x0 must never be renamed: it has no value to hold and freeing it would inject
         // physical register 0 into a free list.
         if (alloc[ik] && r_rd[ik*6 +: 6] == 6'd0)
            $fatal(1, "smolrv64_rename: slot %0d renamed x0", ik);
         if (r_valid[ik] & ~r_cand[ik])
            $fatal(1, "smolrv64_rename: slot %0d renamed and was not a candidate", ik);
         // an f-register goes to an FP shard, any other register to its slot's lane
         if (alloc[ik] && (r_rd[ik*6 + 5] ? ((r_shard[ik*3 +: 3] != SH_FL) && (r_shard[ik*3 +: 3] != SH_FF))
                                          : (r_shard[ik*3 +: 3] != ik[2:0])))
            $fatal(1, "smolrv64_rename: slot %0d renames r%0d into shard %0d", ik, r_rd[ik*6 +: 6], r_shard[ik*3 +: 3]);
         if (alloc[ik] & ~sh_used(r_shard[ik*3 +: 3]))
            $fatal(1, "smolrv64_rename: slot %0d allocates from shard %0d, which this width does not use",
                   ik, r_shard[ik*3 +: 3]);
      end
   end

   initial begin
      if (N_INT <= 32)  $fatal(1, "smolrv64_rename: N_INT=%0d must exceed 32", N_INT);
      if (N_FL <= 32 || N_FF <= 32) $fatal(1, "smolrv64_rename: N_FL=%0d and N_FF=%0d must exceed 32", N_FL, N_FF);
      if (LOWAT < IW)   $fatal(1, "smolrv64_rename: LOWAT=%0d must cover a group of %0d", LOWAT, IW);
   end
endmodule

`default_nettype wire
