`default_nettype none

// Register renaming for smolrv64_core: SMAP/RMAP + lv[], and one free list per PRF
// shard.  Issue and commit remain IN ORDER at this milestone -- this module changes no
// architectural behaviour, so the retire stream must stay bit-identical.  It exists to give
// smolrv64_prf a known destination shard, which is the only cheap way to get more than one write
// port (see smolrv64_prf.v's header).
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
// and RMAP are arrays the tools can infer as LUTRAM.  Two renames per cycle (item 10b):
// SMAP is two copies, one per rename port, plus newer[] -- see the map below.
//
// THE FREE LISTS.  One per shard, because a physical register belongs to exactly one shard.
// This is where the design departs from docs/Area-Efficient-Scalar-OoO.md 9.1: that scheme
// derives the tail from a FIXED occupancy ("free entries are always FREE - n_alloc"), which
// holds only because exactly 32 architectural registers are mapped at all times.  Per shard
// the mapped count VARIES -- 0..32 for every shard -- so occupancy
// is not fixed and the tail cannot be derived.  Each shard therefore carries a real tail.
//
// Rollback is still pointer-only: rename never writes the array (commit is its only
// writer), so the entries between h_comm and h_spec still hold the in-flight allocations.
// Restoring h_spec := h_comm frees them all in one cycle, with no walk and no checkpoints.

module smolrv64_rename
  #(parameter IDXB  = 7,
    parameter PBITS = IDXB + 3,               // 3 shard bits: room for a 5th shard (the 3rd ALU)
    parameter N_IE  = 64,
    parameter N_LD  = 64,                 // integer loads, AMOs, CSR reads
    parameter N_FE  = 64,                 // integer results of the F stage, mul/div, links
    parameter N_FP  = 64,                 // each FP slice (SH_F0..SH_F2), one per rename slot
    parameter N_IE2 = 64,                 // the second ALU's shard (item 10d-ii)
    parameter N_IE3 = 64,                 // the third ALU's shard (Stage 3)
    parameter LOWAT = 4,                  // stall fetch when any shard has < LOWAT free
    parameter IW    = 2)                  // pipeline width -> FLNB = next_pow2(IW) free-list banks
   (input  wire             clk,
    input  wire             reset,

    // ---- rename port (one instruction per cycle) ----
    input  wire             r_valid,      // an instruction is renaming this cycle
    // the slot holds an instruction, taken or not: what the free-list read addresses count, so
    // a candidate register never waits for the dispatch decision (a slot is taken only behind
    // the slots before it, so for every taken slot the count is the same)
    input  wire             r_cand,
    input  wire             r_cand_b,
    input  wire [5:0]       r_rs1,
    input  wire [5:0]       r_rs2,
    input  wire [5:0]       r_rs3,
    input  wire [5:0]       r_rd,
    input  wire             r_rd_v,       // writes a register (x0 destinations excluded)
    input  wire [2:0]       r_shard,      // destination shard, from the instruction class
    output wire [PBITS-1:0] r_prs1,
    output wire [PBITS-1:0] r_prs2,
    output wire [PBITS-1:0] r_prs3,
    // The two candidates and the bit that chooses between them, so a consumer whose result
    // does not depend on WHICH map won can look both up in parallel and select afterwards.
    // `lv` is late; the map read is not. See smolrv64_core's readiness query.
    output wire [PBITS-1:0] r_sprs1, r_sprs2, r_sprs3,   // speculative (smap)
    output wire [PBITS-1:0] r_mprs1, r_mprs2, r_mprs3,   // committed  (rmap)
    output wire             r_lv1, r_lv2, r_lv3,         // lv[rs]: 1 = take the speculative
    output wire [PBITS-1:0] r_prd,        // newly allocated physical register

    // ---- second rename port: slot B, YOUNGER than A in the same cycle (2026-09-05, item 10b) ----
    // B's sources see A's destination (the bypass), B's destination shadows A's when they
    // are the same register, and B takes the second free entry when both allocate from one
    // shard. Tie r_valid_b low and the module is the one-per-cycle unit it was.
    input  wire             r_valid_b,
    input  wire [5:0]       r_rs1_b,
    input  wire [5:0]       r_rs2_b,
    input  wire [5:0]       r_rs3_b,
    input  wire [5:0]       r_rd_b,
    input  wire             r_rd_v_b,
    input  wire [2:0]       r_shard_b,
    output wire [PBITS-1:0] r_prs1_b,
    output wire [PBITS-1:0] r_prs2_b,
    output wire [PBITS-1:0] r_prs3_b,
    output wire [PBITS-1:0] r_sprs1_b, r_sprs2_b, r_sprs3_b, // speculative candidate, the MAP's (the bypass is applied at r_prs*_b)
    output wire [PBITS-1:0] r_mprs1_b, r_mprs2_b, r_mprs3_b, // committed candidate
    output wire             r_lv1_b, r_lv2_b, r_lv3_b,       // 1 = take the speculative
    output wire             r_byp1_b, r_byp2_b, r_byp3_b,    // the source IS A's destination: not ready
    output wire [PBITS-1:0] r_prd_b,

    // ---- third rename port: slot C, YOUNGER than A and B in the same cycle (IW>=3) ----
    // C's sources bypass BOTH A's and B's destinations (B wins on a tie, being younger); C's
    // destination shadows A's/B's when they are the same register. Tie r_valid_c low and the
    // module is the two-per-cycle unit it was.
    input  wire             r_valid_c,
    input  wire [5:0]       r_rs1_c,
    input  wire [5:0]       r_rs2_c,
    input  wire [5:0]       r_rs3_c,
    input  wire [5:0]       r_rd_c,
    input  wire             r_rd_v_c,
    input  wire [2:0]       r_shard_c,
    output wire [PBITS-1:0] r_prs1_c,
    output wire [PBITS-1:0] r_prs2_c,
    output wire [PBITS-1:0] r_prs3_c,
    output wire [PBITS-1:0] r_sprs1_c, r_sprs2_c, r_sprs3_c, // speculative candidate, the MAP's (the bypass is applied at r_prs*_c)
    output wire [PBITS-1:0] r_mprs1_c, r_mprs2_c, r_mprs3_c, // committed candidate
    output wire             r_lv1_c, r_lv2_c, r_lv3_c,       // 1 = take the speculative
    output wire             r_byp1_c, r_byp2_c, r_byp3_c,    // source IS A's or B's destination: not ready
    output wire [PBITS-1:0] r_prd_c,

    // ---- commit port (in order, one per cycle) ----
    input  wire             c_valid,
    input  wire [5:0]       c_rd,
    input  wire             c_rd_v,
    input  wire [PBITS-1:0] c_prd,        // becomes the committed mapping
    // the second commit of the cycle, the entry behind the head (item 10c)
    input  wire             c2_valid,
    input  wire [5:0]       c2_rd,
    input  wire             c2_rd_v,
    input  wire [PBITS-1:0] c2_prd,
    // the third commit of the cycle, the entry two behind the head (IW>=3)
    input  wire             c3_valid,
    input  wire [5:0]       c3_rd,
    input  wire             c3_rd_v,
    input  wire [PBITS-1:0] c3_prd,

    // ---- recovery ----
    input  wire             flush,        // total squash: everything uncommitted dies

    // ---- back pressure and instrumentation ----
    output wire             stall,        // ANY shard low -- see the note below
    output wire [7:0]       shard_low);   // per shard, for the hpm counters

   localparam [2:0] SH_IE = 3'd0, SH_LD = 3'd1, SH_FE = 3'd2, SH_IE2 = 3'd3, SH_IE3 = 3'd4,
                    SH_F0 = 3'd5, SH_F1 = 3'd6, SH_F2 = 3'd7;

   // ---- the map -------------------------------------------------------------------
   // W COPIES OF THE SPECULATIVE MAP, one write port each: port A writes smap_a, B smap_b, C
   // smap_c, and newer[reg] (0/1/2) says which copy holds the latest mapping. A LUTRAM has one
   // write port, and W renames per cycle need W; duplicating the array by WRITER is the map's
   // version of the PRF's sharding rule.
   (* ram_style = "distributed" *) reg [PBITS-1:0] smap_a [0:63];
   (* ram_style = "distributed" *) reg [PBITS-1:0] smap_b [0:63];
   (* ram_style = "distributed" *) reg [PBITS-1:0] smap_c [0:63];   // 3rd rename port (IW>=3)
   // ...and the COMMITTED map likewise, W commits per cycle: rmap_a takes the head's, rmap_b
   // the second's, rmap_c the third's; rnewer[reg] (0/1/2) says which is current.
   (* ram_style = "distributed" *) reg [PBITS-1:0] rmap_a [0:63];
   (* ram_style = "distributed" *) reg [PBITS-1:0] rmap_b [0:63];
   (* ram_style = "distributed" *) reg [PBITS-1:0] rmap_c [0:63];   // 3rd commit (IW>=3)
   reg [63:0]      lv;
   reg [1:0]       newer  [0:63];             // which speculative copy is latest: 0=a, 1=b, 2=c
   reg [1:0]       rnewer [0:63];             // which committed copy is current: 0=a, 1=b, 2=c
   // The copy-select reads are written out per reader (rule F4: no function reads an array).
   // Helper macro: pick the copy `sel` names among (A,B,C) for register `reg`.
   `define RN_SMAP(reg) (newer[reg]  == 2'd0 ? smap_a[reg] : newer[reg]  == 2'd1 ? smap_b[reg] : smap_c[reg])
   `define RN_RMAP(reg) (rnewer[reg] == 2'd0 ? rmap_a[reg] : rnewer[reg] == 2'd1 ? rmap_b[reg] : rmap_c[reg])

   // Reads are of the PRE-rename mapping for all three sources, including the case where a
   // source equals this instruction's own destination -- rd is written at the clock edge,
   // so the combinational reads below see the old value by construction.  (Matches
   // docs/Area-Efficient-Scalar-OoO.md 14.1 "dispatch -> dispatch, map".)
   assign r_sprs1 = `RN_SMAP(r_rs1);  assign r_mprs1 = `RN_RMAP(r_rs1);  assign r_lv1 = lv[r_rs1];
   assign r_sprs2 = `RN_SMAP(r_rs2);  assign r_mprs2 = `RN_RMAP(r_rs2);  assign r_lv2 = lv[r_rs2];
   assign r_sprs3 = `RN_SMAP(r_rs3);  assign r_mprs3 = `RN_RMAP(r_rs3);  assign r_lv3 = lv[r_rs3];
   assign r_prs1 = r_lv1 ? r_sprs1 : r_mprs1;
   assign r_prs2 = r_lv2 ? r_sprs2 : r_mprs2;
   assign r_prs3 = r_lv3 ? r_sprs3 : r_mprs3;
   // Port B reads the same pre-rename map, then A's destination is bypassed in: B is younger,
   // so a source equal to A's rd names A's NEW register, which is speculative and not ready.
   // (candidates, not takes: B is renamed only behind A, C only behind B)
   wire a_writes = r_cand   & r_rd_v   & ~stall;
   wire b_writes = r_cand_b & r_rd_v_b & ~stall;
   assign r_byp1_b = a_writes & (r_rs1_b == r_rd);
   assign r_byp2_b = a_writes & (r_rs2_b == r_rd);
   assign r_byp3_b = a_writes & (r_rs3_b == r_rd);
   // r_sprs*_b / r_lv*_b are the MAP's candidate and select; the bypass is applied at r_prs*_b
   // only. The core's pending lookup indexes r_sprs*_b and masks its result with r_byp*_b (A's
   // new register is pending by definition), so the lookup no longer waits for A's free-list
   // read: d_insn -> class -> free list -> r_prd -> r_sprs_b -> pend -> the dispatch stage's
   // ready bit was 26 levels, the second family of the IW=3 census after round 1.
   assign r_sprs1_b = `RN_SMAP(r_rs1_b);  assign r_mprs1_b = `RN_RMAP(r_rs1_b);  assign r_lv1_b = lv[r_rs1_b];
   assign r_sprs2_b = `RN_SMAP(r_rs2_b);  assign r_mprs2_b = `RN_RMAP(r_rs2_b);  assign r_lv2_b = lv[r_rs2_b];
   assign r_sprs3_b = `RN_SMAP(r_rs3_b);  assign r_mprs3_b = `RN_RMAP(r_rs3_b);  assign r_lv3_b = lv[r_rs3_b];
   assign r_prs1_b = r_byp1_b ? r_prd : r_lv1_b ? r_sprs1_b : r_mprs1_b;
   assign r_prs2_b = r_byp2_b ? r_prd : r_lv2_b ? r_sprs2_b : r_mprs2_b;
   assign r_prs3_b = r_byp3_b ? r_prd : r_lv3_b ? r_sprs3_b : r_mprs3_b;
   // Port C is younger than A and B: a source equal to B's rd takes B's new register, and one
   // equal to A's rd takes A's -- B wins a tie (A and B writing the same reg), being younger.
   wire byp1_c_b = b_writes & (r_rs1_c == r_rd_b),  byp1_c_a = a_writes & (r_rs1_c == r_rd);
   wire byp2_c_b = b_writes & (r_rs2_c == r_rd_b),  byp2_c_a = a_writes & (r_rs2_c == r_rd);
   wire byp3_c_b = b_writes & (r_rs3_c == r_rd_b),  byp3_c_a = a_writes & (r_rs3_c == r_rd);
   assign r_byp1_c = byp1_c_b | byp1_c_a;
   assign r_byp2_c = byp2_c_b | byp2_c_a;
   assign r_byp3_c = byp3_c_b | byp3_c_a;
   assign r_sprs1_c = `RN_SMAP(r_rs1_c);  assign r_mprs1_c = `RN_RMAP(r_rs1_c);  assign r_lv1_c = lv[r_rs1_c];
   assign r_sprs2_c = `RN_SMAP(r_rs2_c);  assign r_mprs2_c = `RN_RMAP(r_rs2_c);  assign r_lv2_c = lv[r_rs2_c];
   assign r_sprs3_c = `RN_SMAP(r_rs3_c);  assign r_mprs3_c = `RN_RMAP(r_rs3_c);  assign r_lv3_c = lv[r_rs3_c];
   assign r_prs1_c = byp1_c_b ? r_prd_b : byp1_c_a ? r_prd : r_lv1_c ? r_sprs1_c : r_mprs1_c;
   assign r_prs2_c = byp2_c_b ? r_prd_b : byp2_c_a ? r_prd : r_lv2_c ? r_sprs2_c : r_mprs2_c;
   assign r_prs3_c = byp3_c_b ? r_prd_b : byp3_c_a ? r_prd : r_lv3_c ? r_sprs3_c : r_mprs3_c;

   // ---- free lists, one per shard ---------------------------------------------------
   // Shard s's list is the generate block fl[s]: a circular FIFO of the shard's free indices,
   // with a speculative head h (allocation), a committed head hc (the rollback target) and a
   // tail t (frees). Pointers carry an extra MSB so full and empty are distinguishable without
   // a count: empty when h == t, full when h == t ^ {1'b1, 0}.
   //
   // FLNB = next_pow2(IW) banks per list, one muxed write per bank: a cycle's up to three frees
   // land at t, t+1, t+2, which fall in different banks. Bank g reads its next free entry at
   // h's index plus one when g is behind h's bank -- REGISTERS ONLY on the read address. Which
   // slot allocates from which shard (the class decode) selects among the banks' OUTPUTS, not
   // their addresses, and the stall never reaches an address: with it there, the whole rename
   // decision (five tail-minus-head subtractions, the low-water compares, the OR) sat in front
   // of the LUTRAM read, and the read's data in front of the dispatch stage and the store queue
   // (t_ld -> avail -> stall -> hb -> flrd -> stg_ps -> u_sq/ld_w, 20-23 levels, eight families
   // of the IW=3 census). A stall holds every slot, so entries read and not consumed are simply
   // read again.
   //
   // NO FUNCTION READS THESE ARRAYS (rule F4): Vivado keeps one read port for a function that
   // reads a RAM, the last call site's, and folds the earlier calls to 0.
   //
   // Rollback is pointer-only: rename never writes the arrays (the frees are their only
   // writer), so the entries between hc and h still hold the in-flight allocations, and
   // h := hc frees them all in one cycle, with no walk and no checkpoints.
   // SH_F0..SH_F2 hold f0-f31 and nothing else, one slice per rename slot: an FP destination in
   // slot A allocates from SH_F0, B from SH_F1, C from SH_F2, so no FP allocation counts the
   // slots before it. Each slice must exceed 32: every f-register may map into one slice.
   localparam integer NSH  = 8;
   localparam integer FLNB = 1 << $clog2(IW);   // free-list banks = next_pow2(IW); >=2
   localparam integer FLLB = $clog2(FLNB);
   // A shard's size, and the first free index at reset: SH_IE and SH_F0 hold the reset
   // mappings of x0-x31 and f0-f31 at indices 0..31, so their lists start at 32.
   function automatic integer n_of(input integer sh);
      n_of = (sh == SH_IE) ? N_IE : (sh == SH_LD) ? N_LD : (sh == SH_FE) ? N_FE
           : (sh == SH_IE2) ? N_IE2 : (sh == SH_IE3) ? N_IE3 : N_FP;
   endfunction
   function automatic integer base_of(input integer sh);
      base_of = (sh == SH_IE || sh == SH_F0) ? 32 : 0;
   endfunction

   // Commit and flush can land in the SAME cycle: a mispredicting branch commits while the
   // instructions behind it are squashed.  The restore target must therefore be the
   // POST-commit committed head (hc_n below), not hc.
   // TWO INDEPENDENT SHARDS PER COMMIT, and conflating them was a bug.  c_prd's shard is where
   // the instruction ALLOCATED (so it says which head advanced), but the displaced register
   // c_pold belongs to whichever shard last wrote that architectural register -- x29 written
   // by the ALU and then by a load displaces an SH_IE register while allocating an SH_LD
   // one.  A register's shard is encoded in its number and never changes, so the free push
   // must be routed by c_pold's own shard.  Routing it by the allocation shard moves registers
   // between shards, which breaks the one-writer-per-bank property the whole design rests on.
   // NEITHER the displaced register NOR the destination shard travels in the ROB (doc 5.1).
   // rmap holds committed state, so in the cycle this entry commits rmap[c_rd] is still the
   // mapping it displaced -- the write below is what replaces it.
   wire [PBITS-1:0] c_pold  = `RN_RMAP(c_rd);
   // the second commit displaces the FIRST's mapping when both write one architectural register
   wire [PBITS-1:0] c2_pold = (c_valid & c_rd_v & (c2_rd == c_rd)) ? c_prd : `RN_RMAP(c2_rd);
   // the third displaces the most recent PRIOR commit of the same register this cycle (c2, then c)
   wire [PBITS-1:0] c3_pold = (c2_valid & c2_rd_v & (c3_rd == c2_rd)) ? c2_prd
                            : (c_valid  & c_rd_v  & (c3_rd == c_rd))  ? c_prd
                            : `RN_RMAP(c3_rd);
   wire c1_w = c_valid  & c_rd_v;
   wire c2_w = c2_valid & c2_rd_v;
   wire c3_w = c3_valid & c3_rd_v;

   // ---- FLOP THE REGISTER RELEASE (2026-09-15) --------------------------------------
   // The free (write + tail advance) is registered one cycle off the retire cone. c_valid
   // rides m_addr -> dTLB -> lsu_done -> retire, and fanning it to every free list's write
   // ports and tails combinationally was ~1400 of the IW=3 near-critical endpoints
   // (m_addr -> u_rename/fl_*.mem_reg). The register is released one cycle later, which is
   // free: the lists are never the allocation bottleneck, and avail = t - h counts a
   // committed free one cycle later, which is strictly MORE conservative. ROLLBACK-SAFE WITH
   // NO EXTRA HANDLING: the tail advances only on committed frees and flush never touches it,
   // so a pending free is always a committed one that must complete. c_pold is captured HERE,
   // at commit, while RMAP[c_rd] still holds the old mapping.
   reg [NSH-1:0]   fre_q, fre2_q, fre3_q;
   reg [PBITS-1:0] c_pold_q, c2_pold_q, c3_pold_q;
   initial begin
      fre_q = {NSH{1'b0}}; fre2_q = {NSH{1'b0}}; fre3_q = {NSH{1'b0}};
      c_pold_q = {PBITS{1'b0}}; c2_pold_q = {PBITS{1'b0}}; c3_pold_q = {PBITS{1'b0}};
   end

   // THE FREE-LIST READ ADDRESSES DO NOT SEE THE STALL (ar_* address; a_* advance).
   wire alloc_r   = r_cand   & r_rd_v;
   wire alloc_r_b = r_cand_b & r_rd_v_b;
   wire alloc   = r_valid   & r_rd_v   & ~stall;
   wire alloc_b = r_valid_b & r_rd_v_b & ~stall;
   wire alloc_c = r_valid_c & r_rd_v_c & ~stall;

   wire [IDXB-1:0] rd_a [0:NSH-1];   // each list's register for slot A, B and C
   wire [IDXB-1:0] rd_b [0:NSH-1];
   wire [IDXB-1:0] rd_c [0:NSH-1];
   wire [NSH-1:0]  low_n;   // each list's low-water flag for next cycle
   wire [NSH-1:0]  sel_r_a, sel_r_b, sel_a, sel_b, sel_c;   // which list each slot allocates from
   wire [NSH-1:0]  cmt_a, cmt_b, cmt_c, fre_a, fre_b, fre_c;

   genvar gL, gB;
   generate for (gL = 0; gL < NSH; gL = gL + 1) begin: fl
      localparam integer N  = n_of(gL);
      localparam integer PW = $clog2(N) + 1;
      localparam integer  NFREE = N - base_of(gL);   // the initially free entries: base..N-1
      localparam [PW-1:0] T0 = NFREE[PW-1:0];

      assign sel_r_a[gL] = alloc_r   & (r_shard   == gL);
      assign sel_r_b[gL] = alloc_r_b & (r_shard_b == gL);
      assign sel_a[gL]   = alloc     & (r_shard   == gL);
      assign sel_b[gL]   = alloc_b   & (r_shard_b == gL);
      assign sel_c[gL]   = alloc_c   & (r_shard_c == gL);
      assign cmt_a[gL]   = c1_w & (c_prd [PBITS-1:IDXB] == gL);   // head advance: allocation shard
      assign cmt_b[gL]   = c2_w & (c2_prd[PBITS-1:IDXB] == gL);
      assign cmt_c[gL]   = c3_w & (c3_prd[PBITS-1:IDXB] == gL);
      assign fre_a[gL]   = c1_w & (c_pold [PBITS-1:IDXB] == gL);  // free push: the register's shard
      assign fre_b[gL]   = c2_w & (c2_pold[PBITS-1:IDXB] == gL);
      assign fre_c[gL]   = c3_w & (c3_pold[PBITS-1:IDXB] == gL);

      reg [PW-1:0] h, hc, t;
      wire [PW-1:0] hc_n = hc + {{(PW-1){1'b0}}, cmt_a[gL]} + {{(PW-1){1'b0}}, cmt_b[gL]}
                              + {{(PW-1){1'b0}}, cmt_c[gL]};
      // the second and third frees' slots: the tail plus the earlier frees this cycle
      wire [PW-2:0] t2  = t[PW-2:0] + {{(PW-2){1'b0}}, fre_q[gL]};
      wire [PW-2:0] t3  = t[PW-2:0] + {{(PW-2){1'b0}}, fre_q[gL]} + {{(PW-2){1'b0}}, fre2_q[gL]};
      // B's entry: the head, or the one after it when A allocates here; C's: the head plus
      // A's and B's allocations here. A second and third LUTRAM read port, not more pointers;
      // LOWAT >= 3 keeps all three inside the free set.
      wire [PW-2:0] hb  = h[PW-2:0] + {{(PW-2){1'b0}}, sel_r_a[gL]};
      wire [PW-2:0] hcc = h[PW-2:0] + {{(PW-2){1'b0}}, sel_r_a[gL]} + {{(PW-2){1'b0}}, sel_r_b[gL]};
      wire [PW-1:0] avail = t - h;
      // The pointers' next values, so the low-water flag can be a register (see `stall`).
      wire [PW-1:0] t_n = reset ? t
                        : t + {{(PW-1){1'b0}}, fre_q[gL]} + {{(PW-1){1'b0}}, fre2_q[gL]} + {{(PW-1){1'b0}}, fre3_q[gL]};
      wire [PW-1:0] h_n = reset ? hc : flush ? hc_n
                        : h + {{(PW-1){1'b0}}, sel_a[gL]} + {{(PW-1){1'b0}}, sel_b[gL]} + {{(PW-1){1'b0}}, sel_c[gL]};
      assign low_n[gL] = (t_n - h_n) < LOWAT[PW-1:0];

      wire [IDXB-1:0]  flrd [0:FLNB-1];
      wire [FLNB-1:0]  behind = ~({FLNB{1'b1}} << h[FLLB-1:0]);   // behind[g] = (g < h's bank): the banks that wrapped
      for (gB = 0; gB < FLNB; gB = gB + 1) begin: bank
         (* ram_style = "distributed" *) reg [IDXB-1:0] mem [0:N/FLNB-1];
         integer jj; integer pp;
         initial for (jj = 0; jj < N/FLNB; jj = jj + 1) begin pp = base_of(gL) + FLNB*jj + gB; mem[jj] = pp[IDXB-1:0]; end
         wire w0 = fre_q [gL] & (t [FLLB-1:0] == gB);
         wire w1 = fre2_q[gL] & (t2[FLLB-1:0] == gB);
         wire w2 = fre3_q[gL] & (t3[FLLB-1:0] == gB);   // 3rd free (IW>=3; dead otherwise)
         always @(posedge clk)
            if (w0 | w1 | w2) mem[w0 ? t[PW-2:FLLB] : w1 ? t2[PW-2:FLLB] : t3[PW-2:FLLB]]
                                 <= w0 ? c_pold_q[IDXB-1:0] : w1 ? c2_pold_q[IDXB-1:0] : c3_pold_q[IDXB-1:0];
         assign flrd[gB] = mem[h[PW-2:FLLB] + {{(PW-2-FLLB){1'b0}}, behind[gB]}];
      end
      assign rd_a[gL] = flrd[h  [FLLB-1:0]];
      assign rd_b[gL] = flrd[hb [FLLB-1:0]];
      assign rd_c[gL] = flrd[hcc[FLLB-1:0]];

      // The pointers come from configuration, like the arrays (see the reset note below).
      initial begin h = {PW{1'b0}}; hc = {PW{1'b0}}; t = T0; end
      always @(posedge clk) begin
         h <= h_n;  t <= t_n;
         if (!reset) hc <= hc_n;
      end

      // Allocating past the free set would hand out a register that is still live; the stall
      // is supposed to make it unreachable, and "supposed to" is what assertions are for.
      always @(posedge clk) if (!reset) begin
         if ({{(PW-2){1'b0}}, sel_a[gL]} + {{(PW-2){1'b0}}, sel_b[gL]} + {{(PW-2){1'b0}}, sel_c[gL]} > avail)
            $fatal(1, "smolrv64_rename: shard %0d allocated past its free list (%0d free)", gL, avail);
         if (avail > N[PW-1:0])
            $fatal(1, "smolrv64_rename: shard %0d has %0d free of %0d", gL, avail, N);
      end
      // Sizes MUST be powers of two: the pointers index with their low bits, which only wraps
      // correctly at a power of two. At N=40 the pointer walked past the end of the array and
      // read 0 -- i.e. handed out physical register 0, the architectural zero.
      initial begin
         if ((N & (N - 1)) != 0) $fatal(1, "smolrv64_rename: shard %0d size %0d is not a power of two", gL, N);
         if (N > (1 << IDXB))    $fatal(1, "smolrv64_rename: shard %0d size %0d exceeds IDXB=%0d", gL, N, IDXB);
      end
   end endgenerate

   always @(posedge clk) begin
      fre_q  <= reset ? {NSH{1'b0}} : fre_a;
      fre2_q <= reset ? {NSH{1'b0}} : fre_b;
      fre3_q <= reset ? {NSH{1'b0}} : fre_c;
      c_pold_q <= c_pold;  c2_pold_q <= c2_pold;  c3_pold_q <= c3_pold;
   end

   // STALL WHEN *ANY* SHARD IS LOW, not when the destination's shard is.  A shard that runs
   // dry stalls rename regardless of which one the next instruction wants, so throttling on
   // the minimum is what actually prevents the stall; throttling per-destination only
   // discovers it one instruction too late.  The cost is that the stall probability is the
   // union across shards -- which is the argument for sizing them unequally rather than
   // adding more of them.
   // THE STALL IS A REGISTER: next cycle's low-water flags, from the pointers' next values. It
   // equals the flags of the pointers it is used with, exactly, but no pointer's adder or
   // compare stands in front of the dispatch take: fl.h -> avail -> low -> stall -> d_take ->
   // the allocation -> the pending table's set was 16 levels at -0.552 ns (lanes step 5.2a).
   reg [NSH-1:0] low;
   initial low = {NSH{1'b0}};
   always @(posedge clk) low <= low_n;
   assign shard_low = low;
   assign stall = |low;

   assign r_prd   = {r_shard,   rd_a[r_shard]};
   assign r_prd_b = {r_shard_b, rd_b[r_shard_b]};
   assign r_prd_c = {r_shard_c, rd_c[r_shard_c]};

   // THE FREE LISTS AND THE MAPS ARE INITIALISED BY THE BITSTREAM AND NEVER RESET.
   //
   // They used to be written at EVERY index in the reset branch. No RAM can be written at
   // every address in one cycle, so that one `for` loop pinned ~3,400 bits into flops even
   // though every one of them is one-write/few-read once running: the free lists are circular
   // FIFOs read at the head and written at the tail, and the maps are read at the sources
   // (+rmap[c_rd]) and written at one index. On an FPGA the contents come from configuration
   // for free. Rule I7.
   //
   // THE VALUES ONLY HAVE TO BE A PERMUTATION of the shard's indices; which permutation is
   // irrelevant, because a free list only ever moves entries around. Identity is used.
   //
   // WHY A RUNTIME RESET IS STILL SAFE. `ui_cpu_reset` is a RUNTIME reset (rk_xcku5p.v:
   // ui_rst | ~init_calib_complete | ~key[1] | fbdiag_rst_sync), so a button press restarts
   // the core without reconfiguring and the arrays keep the PREVIOUS run's contents. That is
   // sound because the contents are only meaningful through the pointers, and slots [h, t)
   // still hold exactly the free set. The pointers are therefore not reset either --
   // resetting them over stale slots is what would republish already-allocated registers as
   // free, handing out DUPLICATE physical registers. Reset instead does what `flush` does
   // (h := hc), which reclaims everything renamed but uncommitted, so nothing leaks across a
   // restart.
   //
   // x0 is safe across all of this: x0 destinations are excluded from rename (r_rd_v), so
   // rmap[0]/smap[0] are never written and keep the {SH_IE,0} that smolrv64_prf hardwires to
   // read zero.
   integer j;
   initial begin
      // x0 -> physical 0 (SH_IE index 0), which smolrv64_prf hardwires to read zero and never
      // writes.  Integer regs start in SH_IE, FP regs in SH_F0.
      for (j = 0; j < 32; j = j + 1) begin
         rmap_a[j]      = {SH_IE, j[IDXB-1:0]};  rmap_b[j]      = {SH_IE, j[IDXB-1:0]};
         smap_a[j]      = {SH_IE, j[IDXB-1:0]};  smap_b[j]      = {SH_IE, j[IDXB-1:0]};
         rmap_a[32 + j] = {SH_F0, j[IDXB-1:0]};  rmap_b[32 + j] = {SH_F0, j[IDXB-1:0]};
         smap_a[32 + j] = {SH_F0, j[IDXB-1:0]};  smap_b[32 + j] = {SH_F0, j[IDXB-1:0]};
      end
      lv = 64'd0;
      for (j = 0; j < 64; j = j + 1) begin newer[j] = 2'd0; rnewer[j] = 2'd0; end
   end

   always @(posedge clk) begin
      // RESET IS A TOTAL SQUASH, which is exactly what `flush` already means here: drop the
      // speculative map (lv) and roll the allocation heads back (in fl[*] above). That
      // RECLAIMS every register renamed but not committed, so restarting the core leaks
      // nothing. It touches no array and no free-list contents.
      if (reset) begin
         lv <= 64'd0;
      end else begin
         // ---- commit: RMAP takes the committed mapping
         if (c_valid & c_rd_v) begin
            rmap_a[c_rd] <= c_prd;  rnewer[c_rd] <= 2'd0;
         end
         if (c2_valid & c2_rd_v) begin           // after the head's: the same register twice keeps the second
            rmap_b[c2_rd] <= c2_prd;  rnewer[c2_rd] <= 2'd1;
         end
         if (c3_valid & c3_rd_v) begin           // ...and the third youngest of all
            rmap_c[c3_rd] <= c3_prd;  rnewer[c3_rd] <= 2'd2;
         end

         // ---- rename: SMAP takes the new mapping.  A flush in the same cycle squashes this
         // instruction; lv is cleared wholesale so the SMAP write becomes invisible either way.
         if (alloc) begin
            smap_a[r_rd] <= r_prd;
            newer[r_rd]  <= 2'd0;
            lv[r_rd]     <= 1'b1;
         end
         if (alloc_b) begin                     // after A: when rd_b == rd, B's is the newer mapping
            smap_b[r_rd_b] <= r_prd_b;
            newer[r_rd_b]  <= 2'd1;
            lv[r_rd_b]     <= 1'b1;
         end
         if (alloc_c) begin                     // youngest: when rd_c == rd/rd_b, C's is the newest
            smap_c[r_rd_c] <= r_prd_c;
            newer[r_rd_c]  <= 2'd2;
            lv[r_rd_c]     <= 1'b1;
         end

         // ---- rollback
         if (flush) lv <= 64'd0;
      end
   end

   // ---- invariants: ALWAYS ON, per docs/rtl-rules.md ---------------------------------
   always @(posedge clk) if (!reset) begin
      // c_pold may legitimately belong to a DIFFERENT shard than c_prd (see fre_* above);
      // what must hold is that the shards exist, so a register is never dropped.
      if (c1_w && !(|fre_a))
         $fatal(1, "smolrv64_rename: commit frees pr=%h of no shard", c_pold);
      if (c1_w && !(|cmt_a))
         $fatal(1, "smolrv64_rename: committing pr=%h of no shard", c_prd);
      if (c2_valid && !c_valid)
         $fatal(1, "smolrv64_rename: second commit without a first");
      // x0 must never be renamed: it has no value to hold and freeing it would inject
      // physical register 0 into a free list.
      if (alloc && r_rd == 6'd0)
         $fatal(1, "smolrv64_rename: renamed x0");
      if (alloc_b && r_rd_b == 6'd0)
         $fatal(1, "smolrv64_rename: renamed x0 (B)");
      if (r_valid_b && !r_valid)
         $fatal(1, "smolrv64_rename: port B without port A");
      if ((r_valid & ~r_cand) | (r_valid_b & ~r_cand_b))
         $fatal(1, "smolrv64_rename: a slot renamed that was not a candidate");
      // f-registers and FP slices go together, and each slot owns one slice.
      if (alloc   && (r_rd[5]   != (r_shard   >= SH_F0) || (r_shard   >= SH_F0 && r_shard   != SH_F0)))
         $fatal(1, "smolrv64_rename: slot A renames r%0d into shard %0d", r_rd, r_shard);
      if (alloc_b && (r_rd_b[5] != (r_shard_b >= SH_F0) || (r_shard_b >= SH_F0 && r_shard_b != SH_F1)))
         $fatal(1, "smolrv64_rename: slot B renames r%0d into shard %0d", r_rd_b, r_shard_b);
      if (alloc_c && (r_rd_c[5] != (r_shard_c >= SH_F0) || (r_shard_c >= SH_F0 && r_shard_c != SH_F2)))
         $fatal(1, "smolrv64_rename: slot C renames r%0d into shard %0d", r_rd_c, r_shard_c);
   end

   initial begin
      if (N_IE <= 32)  $fatal(1, "smolrv64_rename: N_IE=%0d must exceed 32", N_IE);
      if (N_LD <= 32)  $fatal(1, "smolrv64_rename: N_LD=%0d must exceed 32", N_LD);
      if (N_FE <= 32)  $fatal(1, "smolrv64_rename: N_FE=%0d must exceed 32", N_FE);
      if (N_FP <= 32)  $fatal(1, "smolrv64_rename: N_FP=%0d must exceed 32", N_FP);
      if (LOWAT < 1)   $fatal(1, "smolrv64_rename: LOWAT must be >= 1");
   end
   `undef RN_SMAP
   `undef RN_RMAP
endmodule

`default_nettype wire
