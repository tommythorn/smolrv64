`default_nettype none

// Register renaming for the in-order core: SMAP/RMAP + lv[], and one free list per PRF
// shard.  Issue and commit remain IN ORDER at this milestone -- this module changes no
// architectural behaviour, so the retire stream must stay bit-identical.  It exists to give
// ooo2_prf a known destination shard, which is the only cheap way to get more than one write
// port (see ooo2_prf.v's header).
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
// the mapped count VARIES -- 0..64 for the load shard, 0..32 for the others -- so occupancy
// is not fixed and the tail cannot be derived.  Each shard therefore carries a real tail.
//
// Rollback is still pointer-only: rename never writes the array (commit is its only
// writer), so the entries between h_comm and h_spec still hold the in-flight allocations.
// Restoring h_spec := h_comm frees them all in one cycle, with no walk and no checkpoints.

module ooo2_rename
  #(parameter IDXB  = 7,
    parameter PBITS = IDXB + 2,
    parameter N_IE  = 64,
    parameter N_LD  = 128,
    parameter N_FE  = 128,
    parameter LOWAT = 4)                  // stall fetch when any shard has < LOWAT free
   (input  wire             clk,
    input  wire             reset,

    // ---- rename port (one instruction per cycle) ----
    input  wire             r_valid,      // an instruction is renaming this cycle
    input  wire [5:0]       r_rs1,
    input  wire [5:0]       r_rs2,
    input  wire [5:0]       r_rs3,
    input  wire [5:0]       r_rd,
    input  wire             r_rd_v,       // writes a register (x0 destinations excluded)
    input  wire [1:0]       r_shard,      // destination shard, from the instruction class
    output wire [PBITS-1:0] r_prs1,
    output wire [PBITS-1:0] r_prs2,
    output wire [PBITS-1:0] r_prs3,
    // The two candidates and the bit that chooses between them, so a consumer whose result
    // does not depend on WHICH map won can look both up in parallel and select afterwards.
    // `lv` is late; the map read is not. See ooo2_core's readiness query.
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
    input  wire [1:0]       r_shard_b,
    output wire [PBITS-1:0] r_prs1_b,
    output wire [PBITS-1:0] r_prs2_b,
    output wire [PBITS-1:0] r_prs3_b,
    output wire [PBITS-1:0] r_sprs1_b, r_sprs2_b, r_sprs3_b, // speculative candidate (A's prd when bypassed)
    output wire [PBITS-1:0] r_mprs1_b, r_mprs2_b, r_mprs3_b, // committed candidate
    output wire             r_lv1_b, r_lv2_b, r_lv3_b,       // 1 = take the speculative
    output wire             r_byp1_b, r_byp2_b, r_byp3_b,    // the source IS A's destination: not ready
    output wire [PBITS-1:0] r_prd_b,

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

    // ---- recovery ----
    input  wire             flush,        // total squash: everything uncommitted dies

    // ---- back pressure and instrumentation ----
    output wire             stall,        // ANY shard low -- see the note below
    output wire [2:0]       shard_low);   // per-shard, for the hpm counters

   localparam [1:0] SH_IE = 2'd0, SH_LD = 2'd1, SH_FE = 2'd2;
   localparam [IDXB-1:0] OFF32 = 32;   // sized, so the inits do not truncate

   // ---- the map -------------------------------------------------------------------
   // TWO COPIES OF THE SPECULATIVE MAP, one write port each: port A writes smap_a, port B
   // writes smap_b, and newer[] says which copy holds the latest mapping -- the same
   // one-bit select that lv[] already makes between speculative and committed. A LUTRAM has
   // one write port, and two renames per cycle need two; duplicating the array by WRITER
   // is the map's version of the PRF's sharding rule.
   (* ram_style = "distributed" *) reg [PBITS-1:0] smap_a [0:63];
   (* ram_style = "distributed" *) reg [PBITS-1:0] smap_b [0:63];
      // ...and the COMMITTED map likewise, two commits per cycle (item 10c): rmap_a takes the
   // head's, rmap_b the second's, rnewer[] says which is current.
   (* ram_style = "distributed" *) reg [PBITS-1:0] rmap_a [0:63];
   (* ram_style = "distributed" *) reg [PBITS-1:0] rmap_b [0:63];
   reg [63:0]      lv;
   reg [63:0]      newer;                    // 1 = smap_b is the latest speculative mapping
   reg [63:0]      rnewer;                   // 1 = rmap_b is the current committed mapping
   // The copy-select reads are written out per reader (rule F4: no function reads an array;
   // these two happened to survive synthesis with every call site live, the free-list ones
   // did not -- the rule does not distinguish, and tools/check-func-ram-reads.py enforces it).

   // Reads are of the PRE-rename mapping for all three sources, including the case where a
   // source equals this instruction's own destination -- rd is written at the clock edge,
   // so the combinational reads below see the old value by construction.  (Matches
   // docs/Area-Efficient-Scalar-OoO.md 14.1 "dispatch -> dispatch, map".)
   assign r_sprs1 = (newer[r_rs1] ? smap_b[r_rs1] : smap_a[r_rs1]);  assign r_mprs1 = (rnewer[r_rs1] ? rmap_b[r_rs1] : rmap_a[r_rs1]);  assign r_lv1 = lv[r_rs1];
   assign r_sprs2 = (newer[r_rs2] ? smap_b[r_rs2] : smap_a[r_rs2]);  assign r_mprs2 = (rnewer[r_rs2] ? rmap_b[r_rs2] : rmap_a[r_rs2]);  assign r_lv2 = lv[r_rs2];
   assign r_sprs3 = (newer[r_rs3] ? smap_b[r_rs3] : smap_a[r_rs3]);  assign r_mprs3 = (rnewer[r_rs3] ? rmap_b[r_rs3] : rmap_a[r_rs3]);  assign r_lv3 = lv[r_rs3];
   assign r_prs1 = r_lv1 ? r_sprs1 : r_mprs1;
   assign r_prs2 = r_lv2 ? r_sprs2 : r_mprs2;
   assign r_prs3 = r_lv3 ? r_sprs3 : r_mprs3;
   // Port B reads the same pre-rename map, then A's destination is bypassed in: B is younger,
   // so a source equal to A's rd names A's NEW register, which is speculative and not ready.
   wire a_writes = r_valid & r_rd_v & ~stall;
   assign r_byp1_b = a_writes & (r_rs1_b == r_rd);
   assign r_byp2_b = a_writes & (r_rs2_b == r_rd);
   assign r_byp3_b = a_writes & (r_rs3_b == r_rd);
   assign r_sprs1_b = r_byp1_b ? r_prd : (newer[r_rs1_b] ? smap_b[r_rs1_b] : smap_a[r_rs1_b]);  assign r_mprs1_b = (rnewer[r_rs1_b] ? rmap_b[r_rs1_b] : rmap_a[r_rs1_b]);  assign r_lv1_b = r_byp1_b | lv[r_rs1_b];
   assign r_sprs2_b = r_byp2_b ? r_prd : (newer[r_rs2_b] ? smap_b[r_rs2_b] : smap_a[r_rs2_b]);  assign r_mprs2_b = (rnewer[r_rs2_b] ? rmap_b[r_rs2_b] : rmap_a[r_rs2_b]);  assign r_lv2_b = r_byp2_b | lv[r_rs2_b];
   assign r_sprs3_b = r_byp3_b ? r_prd : (newer[r_rs3_b] ? smap_b[r_rs3_b] : smap_a[r_rs3_b]);  assign r_mprs3_b = (rnewer[r_rs3_b] ? rmap_b[r_rs3_b] : rmap_a[r_rs3_b]);  assign r_lv3_b = r_byp3_b | lv[r_rs3_b];
   assign r_prs1_b = r_lv1_b ? r_sprs1_b : r_mprs1_b;
   assign r_prs2_b = r_lv2_b ? r_sprs2_b : r_mprs2_b;
   assign r_prs3_b = r_lv3_b ? r_sprs3_b : r_mprs3_b;

   // ---- free lists, one per shard ---------------------------------------------------
   // Pointers carry an extra MSB so full and empty are distinguishable without a separate
   // count: the list is empty when h == t, full when h == t ^ {1'b1, 0}.
   localparam integer PW_IE = $clog2(N_IE) + 1;
   localparam integer PW_LD = $clog2(N_LD) + 1;
   localparam integer PW_FE = $clog2(N_FE) + 1;

      // TWO PARITY BANKS PER LIST (item 10c): entry i lives in bank i[0] at index i>>1, so the
   // two pushes of a cycle (tail, tail+1) and the two pops (head, head+1) each touch two
   // different banks -- one write per bank per cycle, which is all a LUTRAM has.
   (* ram_style = "distributed" *) reg [IDXB-1:0] fl_ie0 [0:N_IE/2-1];  (* ram_style = "distributed" *) reg [IDXB-1:0] fl_ie1 [0:N_IE/2-1];
   (* ram_style = "distributed" *) reg [IDXB-1:0] fl_ld0 [0:N_LD/2-1];  (* ram_style = "distributed" *) reg [IDXB-1:0] fl_ld1 [0:N_LD/2-1];
   (* ram_style = "distributed" *) reg [IDXB-1:0] fl_fe0 [0:N_FE/2-1];  (* ram_style = "distributed" *) reg [IDXB-1:0] fl_fe1 [0:N_FE/2-1];
   reg [PW_IE-1:0] h_ie, hc_ie, t_ie;
   reg [PW_LD-1:0] h_ld, hc_ld, t_ld;
   reg [PW_FE-1:0] h_fe, hc_fe, t_fe;
   // NO FUNCTION READS THESE ARRAYS (rule I12, 2026-09-06). The parity banks were first read
   // through `rd_ie(i) = i[0] ? fl_ie1[...] : fl_ie0[...]`, called once for port A and once
   // for port B. Vivado keeps ONE read port for a function that reads a RAM: the last call
   // site gets it and every earlier call folds to constant 0 -- port A's r_prd[6:0] became
   // 0 in the netlist, every instruction through port A wrote physical register 0, and the
   // board printed nothing (V4, V7, W2, W2M) while every simulator computed both calls. The
   // reads are continuous assigns at module scope now, one per port, which Vivado
   // replicates into as many LUTRAM read ports as there are readers.

   // Initial tails, sized: SH_IE/SH_FE start with N-32 free, SH_LD with all N.
   localparam [PW_IE-1:0] T0_IE = (N_IE - 32);
   localparam [PW_LD-1:0] T0_LD = N_LD;
   localparam [PW_FE-1:0] T0_FE = (N_FE - 32);

   // Commit and flush can land in the SAME cycle: a mispredicting branch commits while the
   // instructions behind it are squashed.  The restore target must therefore be the
   // POST-commit committed head.  Written as an explicit next-value rather than relying on
   // statement order, because `h <= hc` inside a nonblocking block reads the PRE-commit hc
   // and would hand the just-committed instruction's register back to the free list while
   // it is the live mapping.
   // TWO INDEPENDENT SHARDS PER COMMIT, and conflating them was a bug.  c_shard is where
   // the instruction ALLOCATED (so it says which head advanced), but the displaced register
   // c_pold belongs to whichever shard last wrote that architectural register -- x29 written
   // by the ALU and then by a load displaces an SH_IE register while allocating an SH_LD
   // one.  A register's shard is encoded in its number and never changes, so the free push
   // must be routed by c_pold's own shard.  Routing it by c_shard moves registers between
   // shards, which breaks the one-writer-per-bank property the whole design rests on.
   // NEITHER the displaced register NOR the destination shard travels in the ROB (doc 5.1).
   // rmap holds committed state, so in the cycle this entry commits rmap[c_rd] is still the
   // mapping it displaced -- the write below is what replaces it. And a physical register's
   // shard is the top bits of its number, so the allocation shard is read off c_prd.
      wire [PBITS-1:0] c_pold  = (rnewer[c_rd] ? rmap_b[c_rd] : rmap_a[c_rd]);
   // the second commit displaces the FIRST's mapping when both write one architectural register
   wire [PBITS-1:0] c2_pold = (c_valid & c_rd_v & (c2_rd == c_rd)) ? c_prd : (rnewer[c2_rd] ? rmap_b[c2_rd] : rmap_a[c2_rd]);
   wire [1:0] c2_shard  = c2_prd[PBITS-1:IDXB];
   wire [1:0] pold2_sh  = c2_pold[PBITS-1:IDXB];
   wire cmt2_ie = c2_valid & c2_rd_v & (c2_shard == SH_IE);
   wire cmt2_ld = c2_valid & c2_rd_v & (c2_shard == SH_LD);
   wire cmt2_fe = c2_valid & c2_rd_v & (c2_shard == SH_FE);
   wire fre2_ie = c2_valid & c2_rd_v & (pold2_sh == SH_IE);
   wire fre2_ld = c2_valid & c2_rd_v & (pold2_sh == SH_LD);
   wire fre2_fe = c2_valid & c2_rd_v & (pold2_sh == SH_FE);
   wire [1:0] c_shard = c_prd[PBITS-1:IDXB];
   wire [1:0] pold_sh = c_pold[PBITS-1:IDXB];
   wire cmt_ie = c_valid & c_rd_v & (c_shard == SH_IE);   // head advance: allocation shard
   wire cmt_ld = c_valid & c_rd_v & (c_shard == SH_LD);
   wire cmt_fe = c_valid & c_rd_v & (c_shard == SH_FE);
   wire fre_ie = c_valid & c_rd_v & (pold_sh == SH_IE);   // free push: the register's shard
   wire fre_ld = c_valid & c_rd_v & (pold_sh == SH_LD);
   wire fre_fe = c_valid & c_rd_v & (pold_sh == SH_FE);
      wire [PW_IE-1:0] hc_ie_n = hc_ie + {{(PW_IE-1){1'b0}}, cmt_ie} + {{(PW_IE-1){1'b0}}, cmt2_ie};
   wire [PW_LD-1:0] hc_ld_n = hc_ld + {{(PW_LD-1){1'b0}}, cmt_ld} + {{(PW_LD-1){1'b0}}, cmt2_ld};
   wire [PW_FE-1:0] hc_fe_n = hc_fe + {{(PW_FE-1){1'b0}}, cmt_fe} + {{(PW_FE-1){1'b0}}, cmt2_fe};

      wire [PW_IE-2:0] t2_ie = t_ie[PW_IE-2:0] + {{(PW_IE-2){1'b0}}, fre_ie};   // the second free's slot
   wire [PW_LD-2:0] t2_ld = t_ld[PW_LD-2:0] + {{(PW_LD-2){1'b0}}, fre_ld};
   wire [PW_FE-2:0] t2_fe = t_fe[PW_FE-2:0] + {{(PW_FE-2){1'b0}}, fre_fe};
   wire [PW_IE-1:0] avail_ie = t_ie - h_ie;
   wire [PW_LD-1:0] avail_ld = t_ld - h_ld;
   wire [PW_FE-1:0] avail_fe = t_fe - h_fe;
   // ONE WRITE PER BANK, as one {we, addr, data} each. The head's free and the second's land in
   // different banks by parity (t2 = t + fre), but written as two statements per bank synthesis
   // sees two write ports and demotes every bank to flops -- gate V2's RAM-inference check
   // caught all eight (2026-09-05). The ROB's entries had the same shape; same fix there.
   wire            fw_ie0 = (fre_ie & ~t_ie[0]) | (fre2_ie & ~t2_ie[0]),  fw_ie1 = (fre_ie & t_ie[0]) | (fre2_ie & t2_ie[0]);
   wire [PW_IE-3:0] fa_ie0 = (fre_ie & ~t_ie[0]) ? t_ie[PW_IE-2:1] : t2_ie[PW_IE-2:1];
   wire [PW_IE-3:0] fa_ie1 = (fre_ie &  t_ie[0]) ? t_ie[PW_IE-2:1] : t2_ie[PW_IE-2:1];
   wire [IDXB-1:0]  fd_ie0 = (fre_ie & ~t_ie[0]) ? c_pold[IDXB-1:0] : c2_pold[IDXB-1:0];
   wire [IDXB-1:0]  fd_ie1 = (fre_ie &  t_ie[0]) ? c_pold[IDXB-1:0] : c2_pold[IDXB-1:0];
   wire            fw_ld0 = (fre_ld & ~t_ld[0]) | (fre2_ld & ~t2_ld[0]),  fw_ld1 = (fre_ld & t_ld[0]) | (fre2_ld & t2_ld[0]);
   wire [PW_LD-3:0] fa_ld0 = (fre_ld & ~t_ld[0]) ? t_ld[PW_LD-2:1] : t2_ld[PW_LD-2:1];
   wire [PW_LD-3:0] fa_ld1 = (fre_ld &  t_ld[0]) ? t_ld[PW_LD-2:1] : t2_ld[PW_LD-2:1];
   wire [IDXB-1:0]  fd_ld0 = (fre_ld & ~t_ld[0]) ? c_pold[IDXB-1:0] : c2_pold[IDXB-1:0];
   wire [IDXB-1:0]  fd_ld1 = (fre_ld &  t_ld[0]) ? c_pold[IDXB-1:0] : c2_pold[IDXB-1:0];
   wire            fw_fe0 = (fre_fe & ~t_fe[0]) | (fre2_fe & ~t2_fe[0]),  fw_fe1 = (fre_fe & t_fe[0]) | (fre2_fe & t2_fe[0]);
   wire [PW_FE-3:0] fa_fe0 = (fre_fe & ~t_fe[0]) ? t_fe[PW_FE-2:1] : t2_fe[PW_FE-2:1];
   wire [PW_FE-3:0] fa_fe1 = (fre_fe &  t_fe[0]) ? t_fe[PW_FE-2:1] : t2_fe[PW_FE-2:1];
   wire [IDXB-1:0]  fd_fe0 = (fre_fe & ~t_fe[0]) ? c_pold[IDXB-1:0] : c2_pold[IDXB-1:0];
   wire [IDXB-1:0]  fd_fe1 = (fre_fe &  t_fe[0]) ? c_pold[IDXB-1:0] : c2_pold[IDXB-1:0];

   // STALL WHEN *ANY* SHARD IS LOW, not when the destination's shard is.  A shard that runs
   // dry stalls rename regardless of which one the next instruction wants, so throttling on
   // the minimum is what actually prevents the stall; throttling per-destination only
   // discovers it one instruction too late.  The cost is that the stall probability is the
   // union across shards -- which is the argument for sizing them unequally rather than
   // adding more of them.
   assign shard_low = {avail_fe < LOWAT[PW_FE-1:0],
                       avail_ld < LOWAT[PW_LD-1:0],
                       avail_ie < LOWAT[PW_IE-1:0]};
   assign stall = |shard_low;

   wire alloc   = r_valid   & r_rd_v   & ~stall;
   wire alloc_b = r_valid_b & r_rd_v_b & ~stall;
   wire a_ie = alloc & (r_shard == SH_IE), a_ld = alloc & (r_shard == SH_LD), a_fe = alloc & (r_shard == SH_FE);
   wire b_ie = alloc_b & (r_shard_b == SH_IE), b_ld = alloc_b & (r_shard_b == SH_LD), b_fe = alloc_b & (r_shard_b == SH_FE);
   wire [IDXB-1:0] ha_rd_ie = h_ie[0] ? fl_ie1[h_ie[PW_IE-2:1]] : fl_ie0[h_ie[PW_IE-2:1]];
   wire [IDXB-1:0] ha_rd_ld = h_ld[0] ? fl_ld1[h_ld[PW_LD-2:1]] : fl_ld0[h_ld[PW_LD-2:1]];
   wire [IDXB-1:0] ha_rd_fe = h_fe[0] ? fl_fe1[h_fe[PW_FE-2:1]] : fl_fe0[h_fe[PW_FE-2:1]];
   wire [IDXB-1:0] head_idx = (r_shard == SH_IE) ? ha_rd_ie
                            : (r_shard == SH_LD) ? ha_rd_ld
                                                 : ha_rd_fe;
   assign r_prd = {r_shard, head_idx};
   // B's entry: the head, or the one after it when A allocates from the same shard. A second
   // LUTRAM read port, not a second pointer; LOWAT >= 2 keeps both inside the free set.
   wire [PW_IE-2:0] hb_ie = h_ie[PW_IE-2:0] + {{(PW_IE-2){1'b0}}, a_ie};
   wire [PW_LD-2:0] hb_ld = h_ld[PW_LD-2:0] + {{(PW_LD-2){1'b0}}, a_ld};
   wire [PW_FE-2:0] hb_fe = h_fe[PW_FE-2:0] + {{(PW_FE-2){1'b0}}, a_fe};
   wire [IDXB-1:0] hb_rd_ie = hb_ie[0] ? fl_ie1[hb_ie[PW_IE-2:1]] : fl_ie0[hb_ie[PW_IE-2:1]];
   wire [IDXB-1:0] hb_rd_ld = hb_ld[0] ? fl_ld1[hb_ld[PW_LD-2:1]] : fl_ld0[hb_ld[PW_LD-2:1]];
   wire [IDXB-1:0] hb_rd_fe = hb_fe[0] ? fl_fe1[hb_fe[PW_FE-2:1]] : fl_fe0[hb_fe[PW_FE-2:1]];
   wire [IDXB-1:0] head_idx_b = (r_shard_b == SH_IE) ? hb_rd_ie
                              : (r_shard_b == SH_LD) ? hb_rd_ld
                                                     : hb_rd_fe;
   assign r_prd_b = {r_shard_b, head_idx_b};

   // THESE FIVE ARRAYS ARE INITIALISED BY THE BITSTREAM AND NEVER RESET.
   //
   // They used to be written at EVERY index in the reset branch. No RAM can be written at
   // every address in one cycle, so that one `for` loop pinned ~3,400 bits into flops --
   // fl_ie/fl_ld/fl_fe (~2,240) and rmap/smap (~1,152) -- even though every one of them is
   // one-write/few-read once running: the free lists are circular FIFOs read at the head and
   // written at the tail, and the maps are read at r_rs1/r_rs2/r_rs3 (+rmap[c_rd]) and
   // written at one index. On an FPGA the contents come from configuration for free, so the
   // loop bought nothing and cost the RAM inference plus a reset net fanning out to all of
   // them. Rule I7.
   //
   // THE VALUES ONLY HAVE TO BE A PERMUTATION of the shard's indices; which permutation is
   // irrelevant, because a free list only ever moves entries around. Identity is used.
   //
   // WHY A RUNTIME RESET IS STILL SAFE. `ui_cpu_reset` is a RUNTIME reset (rk_xcku5p.v:
   // ui_rst | ~init_calib_complete | ~key[1] | fbdiag_rst_sync), so a button press restarts
   // the core without reconfiguring and the arrays keep the PREVIOUS run's contents. That is
   // sound because the contents are only meaningful through the pointers, and slots
   // [head,tail) still hold exactly the free set. The pointers are therefore not reset
   // either -- resetting them over stale slots is what would republish already-allocated
   // registers as free. Reset instead does what `flush` does (h_* <= hc_*), which reclaims
   // everything renamed but uncommitted, so nothing leaks across a restart.
   //
   // x0 is safe across all of this: x0 destinations are excluded from rename (r_rd_v), so
   // rmap[0]/smap[0] are never written and keep the {SH_IE,0} that ooo2_prf hardwires to
   // read zero.
   integer j;
   initial begin
      // x0 -> physical 0 (SH_IE index 0), which ooo2_prf hardwires to read zero and never
      // writes.  Integer regs start in SH_IE, FP regs in SH_FE; the load shard starts
      // entirely free.
      for (j = 0; j < 32; j = j + 1) begin
                  rmap_a[j]      = {SH_IE, j[IDXB-1:0]};  rmap_b[j]      = {SH_IE, j[IDXB-1:0]};
         smap_a[j]      = {SH_IE, j[IDXB-1:0]};  smap_b[j]      = {SH_IE, j[IDXB-1:0]};
         rmap_a[32 + j] = {SH_FE, j[IDXB-1:0]};  rmap_b[32 + j] = {SH_FE, j[IDXB-1:0]};
         smap_a[32 + j] = {SH_FE, j[IDXB-1:0]};  smap_b[32 + j] = {SH_FE, j[IDXB-1:0]};
      end
      // Indices 0..31 of SH_IE and SH_FE are taken by the initial architectural mappings, so
      // their free lists start with 32..N-1 -- N-32 entries.  SH_LD starts wholly free.
      // Slots at or beyond the tail are never read (a circular FIFO only reads between head
      // and tail) but are given a legal index anyway so a pointer bug shows up as an
      // assertion rather than as an out-of-range PRF access.
            for (j = 0; j < N_IE/2; j = j + 1) begin fl_ie0[j] = OFF32 + 2*j[IDXB-1:0]; fl_ie1[j] = OFF32 + 2*j[IDXB-1:0] + 1'b1; end
      for (j = 0; j < N_LD/2; j = j + 1) begin fl_ld0[j] = 2*j[IDXB-1:0];         fl_ld1[j] = 2*j[IDXB-1:0] + 1'b1; end
      for (j = 0; j < N_FE/2; j = j + 1) begin fl_fe0[j] = OFF32 + 2*j[IDXB-1:0]; fl_fe1[j] = OFF32 + 2*j[IDXB-1:0] + 1'b1; end
      // The pointers come from configuration for the same reason the arrays do. They are
      // the ONLY thing that says which slots are free, so resetting them while the arrays
      // keep the previous run's contents would republish stale slots as free and hand out
      // DUPLICATE physical registers -- the one combination that is worse than either
      // choice alone.
      h_ie = {PW_IE{1'b0}}; hc_ie = {PW_IE{1'b0}}; t_ie = T0_IE;
      h_ld = {PW_LD{1'b0}}; hc_ld = {PW_LD{1'b0}}; t_ld = T0_LD;
      h_fe = {PW_FE{1'b0}}; hc_fe = {PW_FE{1'b0}}; t_fe = T0_FE;
            lv = 64'd0; newer = 64'd0; rnewer = 64'd0;
   end

   integer i;
   always @(posedge clk) begin
      // RESET IS A TOTAL SQUASH, which is exactly what `flush` already means here, so it
      // does what flush does and nothing else: drop the speculative map and roll the
      // allocation heads back to the committed heads. That RECLAIMS every register renamed
      // but not committed, so restarting the core leaks nothing -- without it, each reset
      // would strand up to a ROB's worth of registers and repeated presses of key[1] would
      // eventually run a shard dry and stall the machine forever.
      // It touches no array and no free-list contents: t_* and hc_* carry the free set
      // across the reset, which is what makes the arrays safe to leave in configuration
      // state. See the initial block above.
      if (reset) begin
         lv   <= 64'd0;
         h_ie <= hc_ie;
         h_ld <= hc_ld;
         h_fe <= hc_fe;
      end else begin
         // ---- commit: RMAP takes the committed mapping, the displaced register is freed
         hc_ie <= hc_ie_n;  hc_ld <= hc_ld_n;  hc_fe <= hc_fe_n;
                  if (c_valid & c_rd_v) begin
            rmap_a[c_rd] <= c_prd;  rnewer[c_rd] <= 1'b0;
            if (c_shard > SH_FE) $fatal(1, "ooo2_rename: commit to shard %0d", c_shard);
         end
         if (c2_valid & c2_rd_v) begin           // after the head's: the same register twice keeps the second
            rmap_b[c2_rd] <= c2_prd;  rnewer[c2_rd] <= 1'b1;
            if (c2_shard > SH_FE) $fatal(1, "ooo2_rename: commit (2) to shard %0d", c2_shard);
         end
         // the frees: the head's at the tail, the second's at the slot after it when both push
         if (fw_ie0) fl_ie0[fa_ie0] <= fd_ie0;   if (fw_ie1) fl_ie1[fa_ie1] <= fd_ie1;
         if (fw_ld0) fl_ld0[fa_ld0] <= fd_ld0;   if (fw_ld1) fl_ld1[fa_ld1] <= fd_ld1;
         if (fw_fe0) fl_fe0[fa_fe0] <= fd_fe0;   if (fw_fe1) fl_fe1[fa_fe1] <= fd_fe1;
         t_ie <= t_ie + {{(PW_IE-1){1'b0}}, fre_ie} + {{(PW_IE-1){1'b0}}, fre2_ie};
         t_ld <= t_ld + {{(PW_LD-1){1'b0}}, fre_ld} + {{(PW_LD-1){1'b0}}, fre2_ld};
         t_fe <= t_fe + {{(PW_FE-1){1'b0}}, fre_fe} + {{(PW_FE-1){1'b0}}, fre2_fe};

         // ---- rename: SMAP takes the new mapping, the head advances.  A flush in the same
         // cycle squashes this instruction, so the flush arm below wins on the pointers;
         // lv is cleared wholesale so the SMAP write becomes invisible either way.
         if (alloc) begin
            smap_a[r_rd] <= r_prd;
            newer[r_rd]  <= 1'b0;
            lv[r_rd]     <= 1'b1;
            if (r_shard > SH_FE) $fatal(1, "ooo2_rename: rename to shard %0d", r_shard);
         end
         if (alloc_b) begin                     // after A: when rd_b == rd, B's is the newer mapping
            smap_b[r_rd_b] <= r_prd_b;
            newer[r_rd_b]  <= 1'b1;
            lv[r_rd_b]     <= 1'b1;
            if (r_shard_b > SH_FE) $fatal(1, "ooo2_rename: rename (B) to shard %0d", r_shard_b);
         end

         // ---- rollback
         if (flush) begin
            lv   <= 64'd0;
            h_ie <= hc_ie_n;
            h_ld <= hc_ld_n;
            h_fe <= hc_fe_n;
         end else begin
            h_ie <= h_ie + {{(PW_IE-1){1'b0}}, a_ie} + {{(PW_IE-1){1'b0}}, b_ie};
            h_ld <= h_ld + {{(PW_LD-1){1'b0}}, a_ld} + {{(PW_LD-1){1'b0}}, b_ld};
            h_fe <= h_fe + {{(PW_FE-1){1'b0}}, a_fe} + {{(PW_FE-1){1'b0}}, b_fe};
         end
      end
   end

   // ---- invariants: ALWAYS ON, per docs/rtl-rules.md ---------------------------------
   always @(posedge clk) if (!reset) begin
      // Allocating from an empty list would hand out a register that is still live.  The
      // stall above is supposed to make this unreachable; "supposed to" is what assertions
      // are for.
      if (alloc && r_shard == SH_IE && avail_ie == 0)
         $fatal(1, "ooo2_rename: allocated from an empty int-exec free list");
      if (alloc && r_shard == SH_LD && avail_ld == 0)
         $fatal(1, "ooo2_rename: allocated from an empty load free list");
      if (alloc && r_shard == SH_FE && avail_fe == 0)
         $fatal(1, "ooo2_rename: allocated from an empty fp-exec free list");
      // c_pold may legitimately belong to a DIFFERENT shard than c_shard (see pold_sh
      // above); what must hold is that it names a shard that exists, so it is returned to a
      // real free list rather than dropped.
      if (c_valid && c_rd_v && (pold_sh > SH_FE))
         $fatal(1, "ooo2_rename: freeing pr=%h whose shard %0d does not exist",
                c_pold, pold_sh);
      if (c_valid && c_rd_v && (c_prd[PBITS-1:IDXB] != c_shard))
         $fatal(1, "ooo2_rename: committing pr=%h whose shard is not %0d", c_prd, c_shard);
      if (c2_valid && !c_valid)
         $fatal(1, "ooo2_rename: second commit without a first");
      if (c2_valid && c2_rd_v && (pold2_sh > SH_FE))
         $fatal(1, "ooo2_rename: freeing (2) pr=%h whose shard %0d does not exist", c2_pold, pold2_sh);
      // x0 must never be renamed: it has no value to hold and freeing it would inject
      // physical register 0 into a free list.
      if (alloc && r_rd == 6'd0)
         $fatal(1, "ooo2_rename: renamed x0");
      if (alloc_b && r_rd_b == 6'd0)
         $fatal(1, "ooo2_rename: renamed x0 (B)");
      if (r_valid_b && !r_valid)
         $fatal(1, "ooo2_rename: port B without port A");
      if (b_ie && (avail_ie < {{(PW_IE-2){1'b0}}, 1'b1, a_ie}))
         $fatal(1, "ooo2_rename: B allocated past the int-exec free list");
      if (b_ld && (avail_ld < {{(PW_LD-2){1'b0}}, 1'b1, a_ld}))
         $fatal(1, "ooo2_rename: B allocated past the load free list");
      if (b_fe && (avail_fe < {{(PW_FE-2){1'b0}}, 1'b1, a_fe}))
         $fatal(1, "ooo2_rename: B allocated past the fp-exec free list");
   end

   initial begin
      // Sizes MUST be powers of two: the free-list pointers carry one extra MSB and index
      // with the low bits, which only wraps correctly at a power of two.  At N=40 the
      // pointer walked past the end of the array and read 0 -- i.e. handed out physical
      // register 0, the architectural zero.  Caught by ooo2_prf's pr0 assertion on the
      // first riscv-test; checked here so it cannot come back.
      if ((N_IE & (N_IE-1)) != 0) $fatal(1, "ooo2_rename: N_IE=%0d is not a power of two", N_IE);
      if ((N_LD & (N_LD-1)) != 0) $fatal(1, "ooo2_rename: N_LD=%0d is not a power of two", N_LD);
      if ((N_FE & (N_FE-1)) != 0) $fatal(1, "ooo2_rename: N_FE=%0d is not a power of two", N_FE);
      if (N_IE <= 32) $fatal(1, "ooo2_rename: N_IE=%0d must exceed 32", N_IE);
      if (N_LD <= 64) $fatal(1, "ooo2_rename: N_LD=%0d must exceed 64", N_LD);
      if (N_FE <= 32) $fatal(1, "ooo2_rename: N_FE=%0d must exceed 32 (fp only)", N_FE);
      if (LOWAT < 1) $fatal(1, "ooo2_rename: LOWAT must be >= 1");
   end
endmodule

`default_nettype wire
