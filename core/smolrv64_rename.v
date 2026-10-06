`default_nettype none

// Register renaming for smolrv64_core: SMAP/RMAP + lv[], and one free list per PRF shard. It
// gives smolrv64_prf a known destination shard, which is the only cheap way to get more than
// one write port (see smolrv64_prf.v's header).
//
// THE MAP.  docs/Area-Efficient-Scalar-OoO.md 9.2 recovers by bulk-copying rat_commit into
// map -- a 32 x PBITS parallel load.  On this FPGA that fanout is the failure mode: the
// restore value has to reach every map flop.  Instead:
//
//     read     : lv[a] ? SMAP[a] : RMAP[a]
//     rename   : SMAP[rd] <= p_new ;  lv[rd] <= 1
//     commit   : RMAP[rd] <= p_commit
//     rollback : lv[*] <= 0                      -- ONE control signal, 64 flops
//
// Correct because RMAP holds committed state: after a flush every read falls through to it,
// and stale SMAP entries are simply invisible until re-renamed.  Only lv[] is flops; SMAP
// and RMAP are arrays the tools infer as LUTRAM, one copy per slot (see the map below).
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
// THE GROUP. Up to IW slots rename in a cycle, slot k younger than slot k-1 (slot k only
// with every slot before it): a source of slot k equal to an earlier slot's destination takes
// that slot's new register (the youngest such), a destination of slot k shadows an earlier
// slot's to the same register, and slot k takes the next free entry after the earlier slots'
// allocations from its shard. IW commits likewise, in order.

module smolrv64_rename
  #(parameter IDXB  = 7,
    parameter PBITS = IDXB + 3,           // 3 shard bits (smolrv64_shards.vh)
    parameter N_INT = 64,                 // each lane's shard
    parameter N_FP  = 64,                 // each FP slice, one per rename slot
    parameter LOWAT = 4,                  // stall fetch when any shard has < LOWAT free
    parameter IW    = 2)                  // the group width -> FLNB = next_pow2(IW) free-list banks
   (input  wire                  clk,
    input  wire                  reset,

    // ---- rename: slot k at [k] ----
    input  wire [IW-1:0]         r_valid,      // slot k renames this cycle
    // slot k holds an instruction, taken or not: what the free-list read addresses count, so a
    // candidate register never waits for the dispatch decision (a slot is taken only behind
    // the slots before it, so for every taken slot the count is the same)
    input  wire [IW-1:0]         r_cand,
    input  wire [IW*6-1:0]       r_rs1, r_rs2, r_rs3, r_rd,
    input  wire [IW-1:0]         r_rd_v,       // writes a register (x0 destinations excluded)
    input  wire [IW*3-1:0]       r_shard,      // destination shard, from the instruction class
    output wire [IW*PBITS-1:0]   r_prs1, r_prs2, r_prs3,
    // The two candidates and the bit that chooses between them, so a consumer whose result
    // does not depend on WHICH map won can look both up in parallel and select afterwards.
    // `lv` is late; the map read is not. The speculative candidate is the MAP's: an earlier
    // slot's bypass is applied at r_prs* only, and r_byp* says the source IS an earlier slot's
    // new register (pending by definition).
    output wire [IW*PBITS-1:0]   r_sprs1, r_sprs2, r_sprs3,   // speculative (smap)
    output wire [IW*PBITS-1:0]   r_mprs1, r_mprs2, r_mprs3,   // committed  (rmap)
    output wire [IW-1:0]         r_lv1, r_lv2, r_lv3,         // lv[rs]: 1 = take the speculative
    output wire [IW-1:0]         r_byp1, r_byp2, r_byp3,      // slot 0's are 0
    output wire [IW*PBITS-1:0]   r_prd,        // the newly allocated physical register

    // ---- commit: the head and the entries behind it, in order ----
    input  wire [IW-1:0]         c_valid,
    input  wire [IW*6-1:0]       c_rd,
    input  wire [IW-1:0]         c_rd_v,
    input  wire [IW*PBITS-1:0]   c_prd,        // becomes the committed mapping

    // ---- recovery ----
    input  wire                  flush,        // total squash: everything uncommitted dies

    // ---- back pressure and instrumentation ----
    output wire                  stall,        // ANY shard low -- see the note below
    output wire [7:0]            shard_low);   // per shard, for the hpm counters

`include "smolrv64_shards.vh"
   function automatic sh_used(input integer sh);   // lanes 0..IW-1 and FP slices 4..4+IW-1
      sh_used = (sh % 4) < IW;
   endfunction
   localparam integer NB = (IW > 1) ? $clog2(IW) : 1;   // a copy number: which slot wrote it
   localparam integer NS = 3 * IW;                       // the map's source reads

   // ---- the map -------------------------------------------------------------------
   // IW COPIES OF THE SPECULATIVE MAP, one write port each: slot k writes copy k, and
   // newer[reg] says which copy holds the latest mapping. A LUTRAM has one write port, and IW
   // renames per cycle need IW; duplicating the array by WRITER is the map's version of the
   // PRF's sharding rule. The COMMITTED map likewise: commit k writes copy k, and rnewer[reg]
   // says which is current.
   reg [63:0]      lv;
   reg [NB-1:0]    newer  [0:63];
   reg [NB-1:0]    rnewer [0:63];
   // every source read: slot k's rs1, rs2, rs3 at [3k], [3k+1], [3k+2]; and every commit's
   // displaced mapping, rmap[c_rd[k]], read before the commit's own write lands
   wire [5:0]       sa [0:NS-1];
   wire [PBITS-1:0] sv [0:IW*NS-1];          // copy c's speculative mapping of source p
   wire [PBITS-1:0] mv [0:IW*NS-1];          // ...and its committed one
   wire [PBITS-1:0] pv [0:IW*IW-1];          // copy c's committed mapping of commit k's rd
   genvar gk, gc, gp;
   generate for (gk = 0; gk < IW; gk = gk + 1) begin: src
      assign sa[3*gk]     = r_rs1[gk*6 +: 6];
      assign sa[3*gk + 1] = r_rs2[gk*6 +: 6];
      assign sa[3*gk + 2] = r_rs3[gk*6 +: 6];
   end endgenerate
   generate for (gc = 0; gc < IW; gc = gc + 1) begin: cp
      (* ram_style = "distributed" *) reg [PBITS-1:0] smap [0:63];
      (* ram_style = "distributed" *) reg [PBITS-1:0] rmap [0:63];
      integer j;
      // x0 -> physical 0 (SH_IE index 0), which smolrv64_prf hardwires to read zero and never
      // writes. Integer registers start in SH_IE, FP registers in SH_F0 (see the reset note).
      initial for (j = 0; j < 32; j = j + 1) begin
         rmap[j] = {SH_IE, j[IDXB-1:0]};       smap[j] = {SH_IE, j[IDXB-1:0]};
         rmap[32 + j] = {SH_F0, j[IDXB-1:0]};  smap[32 + j] = {SH_F0, j[IDXB-1:0]};
      end
      wire sw = r_valid[gc] & r_rd_v[gc] & ~stall;   // this slot's allocation
      wire cw = c_valid[gc] & c_rd_v[gc];
      always @(posedge clk) if (!reset & sw) smap[r_rd[gc*6 +: 6]] <= r_prd[gc*PBITS +: PBITS];
      always @(posedge clk) if (!reset & cw) rmap[c_rd[gc*6 +: 6]] <= c_prd[gc*PBITS +: PBITS];
      for (gp = 0; gp < NS; gp = gp + 1) begin: rd
         assign sv[gc*NS + gp] = smap[sa[gp]];
         assign mv[gc*NS + gp] = rmap[sa[gp]];
      end
      for (gp = 0; gp < IW; gp = gp + 1) begin: pd
         assign pv[gc*IW + gp] = rmap[c_rd[gp*6 +: 6]];
      end
   end endgenerate
   // each source's candidates: the copy newer/rnewer names (a wire per read: rule F4)
   wire [PBITS-1:0] sp [0:NS-1], mp [0:NS-1];
   wire [NS-1:0]    lvs;
   generate for (gp = 0; gp < NS; gp = gp + 1) begin: sel
      assign sp[gp]  = sv[newer[sa[gp]] * NS + gp];
      assign mp[gp]  = mv[rnewer[sa[gp]] * NS + gp];
      assign lvs[gp] = lv[sa[gp]];
   end endgenerate
   // Reads are of the PRE-rename mapping, including a source equal to the slot's own
   // destination: rd is written at the clock edge, so the reads see the old value. An earlier
   // slot's destination is bypassed in: the youngest earlier slot naming the source wins.
   wire [IW-1:0] writes = r_cand & r_rd_v & {IW{~stall}};   // candidates, not takes
   wire [PBITS-1:0] np [0:NS-1];               // the youngest earlier slot's new register
   wire [NS-1:0]    byp;
   generate for (gp = 0; gp < NS; gp = gp + 1) begin: by
      localparam integer K = gp / 3;           // the source's slot
      reg              b;
      reg [PBITS-1:0]  n;
      integer j;
      always @* begin
         b = 1'b0;  n = {PBITS{1'b0}};
         for (j = 0; j < K; j = j + 1)
            if (writes[j] & (sa[gp] == r_rd[j*6 +: 6])) begin b = 1'b1;  n = r_prd[j*PBITS +: PBITS]; end
      end
      assign byp[gp] = b;  assign np[gp] = n;
   end endgenerate
   generate for (gk = 0; gk < IW; gk = gk + 1) begin: out
      assign r_sprs1[gk*PBITS +: PBITS] = sp[3*gk];      assign r_mprs1[gk*PBITS +: PBITS] = mp[3*gk];
      assign r_sprs2[gk*PBITS +: PBITS] = sp[3*gk + 1];  assign r_mprs2[gk*PBITS +: PBITS] = mp[3*gk + 1];
      assign r_sprs3[gk*PBITS +: PBITS] = sp[3*gk + 2];  assign r_mprs3[gk*PBITS +: PBITS] = mp[3*gk + 2];
      assign r_lv1[gk] = lvs[3*gk];  assign r_lv2[gk] = lvs[3*gk + 1];  assign r_lv3[gk] = lvs[3*gk + 2];
      assign r_byp1[gk] = byp[3*gk];  assign r_byp2[gk] = byp[3*gk + 1];  assign r_byp3[gk] = byp[3*gk + 2];
      assign r_prs1[gk*PBITS +: PBITS] = byp[3*gk]     ? np[3*gk]     : lvs[3*gk]     ? sp[3*gk]     : mp[3*gk];
      assign r_prs2[gk*PBITS +: PBITS] = byp[3*gk + 1] ? np[3*gk + 1] : lvs[3*gk + 1] ? sp[3*gk + 1] : mp[3*gk + 1];
      assign r_prs3[gk*PBITS +: PBITS] = byp[3*gk + 2] ? np[3*gk + 2] : lvs[3*gk + 2] ? sp[3*gk + 2] : mp[3*gk + 2];
   end endgenerate

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
   // stage and the store queue. A stall holds every slot, so entries read and not consumed are
   // simply read again.
   //
   // NO FUNCTION READS THESE ARRAYS (rule F4): Vivado keeps one read port for a function that
   // reads a RAM, the last call site's, and folds the earlier calls to 0.
   //
   // The FP slices hold f0-f31 and nothing else, one slice per rename slot: an FP destination
   // in slot k allocates from SH_F0 + k, so no FP allocation counts the slots before it.
   localparam integer NSH  = 8;
   localparam integer FLNB = 1 << $clog2(IW);   // free-list banks = next_pow2(IW); >=2
   localparam integer FLLB = $clog2(FLNB);
   localparam integer CW   = $clog2(IW + 1);    // a count of 0..IW
   // A shard's size, and the first free index at reset: SH_IE and SH_F0 hold the reset
   // mappings of x0-x31 and f0-f31 at indices 0..31, so their lists start at 32.
   function automatic integer n_of(input integer sh);
      n_of = (sh < SH_F0) ? N_INT : N_FP;
   endfunction
   function automatic integer base_of(input integer sh);
      base_of = (sh == SH_IE || sh == SH_F0) ? 32 : 0;
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
   // rmap holds committed state, so in the cycle commit k happens rmap[c_rd[k]] is still the
   // mapping it displaced -- unless an earlier commit this cycle wrote the same register, whose
   // new mapping is then the one displaced.
   wire [IW-1:0]    c_w = c_valid & c_rd_v;
   reg  [PBITS-1:0] c_pold [0:IW-1];
   integer          pk, pj;
   always @* for (pk = 0; pk < IW; pk = pk + 1) begin
      c_pold[pk] = pv[rnewer[c_rd[pk*6 +: 6]] * IW + pk];
      for (pj = 0; pj < pk; pj = pj + 1)
         if (c_w[pj] & (c_rd[pj*6 +: 6] == c_rd[pk*6 +: 6])) c_pold[pk] = c_prd[pj*PBITS +: PBITS];
   end

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

   // THE FREE-LIST READ ADDRESSES DO NOT SEE THE STALL (alloc_r addresses; alloc advances).
   wire [IW-1:0] alloc_r = r_cand & r_rd_v;
   wire [IW-1:0] alloc   = r_valid & r_rd_v & {IW{~stall}};

   wire [IDXB-1:0] rdx [0:IW*NSH-1];   // each list's register for slot k at [k*NSH + list]
   wire [NSH-1:0]  low_n;              // each list's low-water flag for next cycle
   wire [NSH-1:0]  fre_n [0:IW-1];     // commit k frees into list s

   genvar gL, gB;
   generate for (gL = 0; gL < NSH; gL = gL + 1) begin: fl
      localparam [2:0] SHL = gL;
      wire [IW-1:0] sel_r, sel, cmt;
      for (gk = 0; gk < IW; gk = gk + 1) begin: s
         assign sel_r[gk] = alloc_r[gk] & (r_shard[gk*3 +: 3] == SHL);
         assign sel[gk]   = alloc[gk]   & (r_shard[gk*3 +: 3] == SHL);
         assign cmt[gk]   = c_w[gk] & (c_prd[gk*PBITS + IDXB +: 3] == SHL);   // head advance: the allocation's shard
         assign fre_n[gk][gL] = c_w[gk] & (c_pold[gk][PBITS-1:IDXB] == SHL);   // free push: the register's shard
      end
    if (!sh_used(gL)) begin: none   // a shard this width does not use: no list, never low
      for (gk = 0; gk < IW; gk = gk + 1) begin: z
         assign rdx[gk*NSH + gL] = {IDXB{1'b0}};
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
      // The pointers' next values, so the low-water flag can be a register (see `stall`).
      wire [PW-1:0] t_n = reset ? t : t + cnt(fq);
      wire [PW-1:0] h_n = reset ? hc : flush ? hc_n : h + cnt(sel);
      assign low_n[gL] = (t_n - h_n) < LOWAT[PW-1:0];

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
      // slot k's entry: the head plus the earlier slots' allocations from this list (a
      // second and third LUTRAM read port, not more pointers; LOWAT >= IW keeps them all inside
      // the free set)
      for (gk = 0; gk < IW; gk = gk + 1) begin: e
         localparam [IW-1:0] BEFORE = (1 << gk) - 1;   // the slots before slot k
         wire [PW-1:0] n = cnt(sel_r & BEFORE);
         wire [PW-2:0] off = h[PW-2:0] + n[PW-2:0];
         assign rdx[gk*NSH + gL] = flrd[off[FLLB-1:0]];
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
   // THE STALL IS A REGISTER: next cycle's low-water flags, from the pointers' next values. It
   // equals the flags of the pointers it is used with, exactly, but no pointer's adder or
   // compare stands in front of the dispatch take.
   reg [NSH-1:0] low;
   initial low = {NSH{1'b0}};
   always @(posedge clk) low <= low_n;
   assign shard_low = low;
   assign stall = |low;

   generate for (gk = 0; gk < IW; gk = gk + 1) begin: pr
      wire [2:0] sh = r_shard[gk*3 +: 3];
      assign r_prd[gk*PBITS +: PBITS] = {sh, rdx[gk*NSH + sh]};
   end endgenerate

   // THE FREE LISTS AND THE MAPS ARE INITIALISED BY THE BITSTREAM AND NEVER RESET. No RAM can be
   // written at every address in one cycle, and every one of them is one-write/few-read once
   // running: the free lists are circular FIFOs read at the head and written at the tail, and the
   // maps are read at the sources (+rmap[c_rd]) and written at one index. On an FPGA the contents
   // come from configuration for free (rule I7).
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
   // which reclaims everything renamed but uncommitted, so nothing leaks across a restart.
   //
   // x0 is safe across all of this: x0 destinations are excluded from rename (r_rd_v), so
   // rmap[0]/smap[0] are never written and keep the {SH_IE,0} that smolrv64_prf hardwires to
   // read zero.
   integer j;
   initial begin
      lv = 64'd0;
      for (j = 0; j < 64; j = j + 1) begin newer[j] = {NB{1'b0}}; rnewer[j] = {NB{1'b0}}; end
   end

   always @(posedge clk) begin
      // RESET IS A TOTAL SQUASH, which is exactly what `flush` already means here: drop the
      // speculative map (lv) and roll the allocation heads back (in fl[*] above).
      if (reset) begin
         lv <= 64'd0;
      end else begin
         // ---- commit: the copy each commit wrote is current; a later commit of the same
         // register this cycle wins (the loop's order)
         for (j = 0; j < IW; j = j + 1)
            if (c_w[j]) rnewer[c_rd[j*6 +: 6]] <= j[NB-1:0];
         // ---- rename: likewise for the speculative map. A flush in the same cycle squashes
         // the group; lv is cleared wholesale so the map write becomes invisible either way.
         for (j = 0; j < IW; j = j + 1)
            if (alloc[j]) begin newer[r_rd[j*6 +: 6]] <= j[NB-1:0];  lv[r_rd[j*6 +: 6]] <= 1'b1; end
         // ---- rollback
         if (flush) lv <= 64'd0;
      end
   end

   // ---- invariants: ALWAYS ON, per docs/rtl-rules.md ---------------------------------
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
         if (ik > 0 && r_valid[ik] && !r_valid[ik-1])
            $fatal(1, "smolrv64_rename: slot %0d renamed without the one before it", ik);
         if (r_valid[ik] & ~r_cand[ik])
            $fatal(1, "smolrv64_rename: slot %0d renamed and was not a candidate", ik);
         // f-registers and FP slices go together, and each slot owns one slice.
         if (alloc[ik] && (r_rd[ik*6 + 5] != r_shard[ik*3 + 2]
                           || (r_shard[ik*3 + 2] && r_shard[ik*3 +: 3] != SH_F0 + ik[2:0])))
            $fatal(1, "smolrv64_rename: slot %0d renames r%0d into shard %0d", ik, r_rd[ik*6 +: 6], r_shard[ik*3 +: 3]);
         if (alloc[ik] & ~sh_used(r_shard[ik*3 +: 3]))
            $fatal(1, "smolrv64_rename: slot %0d allocates from shard %0d, which this width does not use",
                   ik, r_shard[ik*3 +: 3]);
      end
   end

   initial begin
      if (N_INT <= 32)  $fatal(1, "smolrv64_rename: N_INT=%0d must exceed 32", N_INT);
      if (N_FP <= 32)   $fatal(1, "smolrv64_rename: N_FP=%0d must exceed 32", N_FP);
      if (LOWAT < IW)   $fatal(1, "smolrv64_rename: LOWAT=%0d must cover a group of %0d", LOWAT, IW);
   end
endmodule

`default_nettype wire
