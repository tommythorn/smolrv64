`default_nettype none

// Register renaming for ooo2_core: SMAP/RMAP + lv[], and one free list per PRF
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
    parameter PBITS = IDXB + 3,               // 3 shard bits: room for a 5th shard (the 3rd ALU)
    parameter N_IE  = 64,
    parameter N_LD  = 128,
    parameter N_FE  = 128,
    parameter N_IE2 = 64,                 // the second ALU's shard (item 10d-ii)
    parameter N_IE3 = 64,                 // the third ALU's shard (Stage 3)
    parameter LOWAT = 4,                  // stall fetch when any shard has < LOWAT free
    parameter IW    = 2)                  // pipeline width -> FLNB = next_pow2(IW) free-list banks
   (input  wire             clk,
    input  wire             reset,

    // ---- rename port (one instruction per cycle) ----
    input  wire             r_valid,      // an instruction is renaming this cycle
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
        output wire [4:0]       shard_low);   // per-shard (5 shards), for the hpm counters

      localparam [2:0] SH_IE = 3'd0, SH_LD = 3'd1, SH_FE = 3'd2, SH_IE2 = 3'd3, SH_IE3 = 3'd4;
   localparam [IDXB-1:0] OFF32 = 32;   // sized, so the inits do not truncate

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
   wire a_writes = r_valid   & r_rd_v   & ~stall;
   wire b_writes = r_valid_b & r_rd_v_b & ~stall;
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
   // Pointers carry an extra MSB so full and empty are distinguishable without a separate
   // count: the list is empty when h == t, full when h == t ^ {1'b1, 0}.
   localparam integer PW_IE = $clog2(N_IE) + 1;
   localparam integer PW_LD = $clog2(N_LD) + 1;
   localparam integer PW_FE = $clog2(N_FE) + 1;
   localparam integer PW_I2 = $clog2(N_IE2) + 1;
   localparam integer PW_I3 = $clog2(N_IE3) + 1;
   localparam integer FLNB = 1 << $clog2(IW);   // free-list banks = next_pow2(IW); >=2
   localparam integer FLLB = $clog2(FLNB);

   // FLNB=next_pow2(IW) banks per list (Stage 3 inc 3, was the two parity banks): the generate
   // blocks below hold each shard's list; one muxed write per bank, head/head+1 reads.
   reg [PW_IE-1:0] h_ie, hc_ie, t_ie;
   reg [PW_LD-1:0] h_ld, hc_ld, t_ld;
   reg [PW_FE-1:0] h_fe, hc_fe, t_fe;
   reg [PW_I2-1:0] h_i2, hc_i2, t_i2;
   reg [PW_I3-1:0] h_i3, hc_i3, t_i3;
   // NO FUNCTION READS THESE ARRAYS (rule F4, 2026-09-06): Vivado keeps one read port for a
   // function that reads a RAM, the last call site's, and folds the earlier call to 0 -- port
   // A's r_prd[6:0] was 0 on V4, V7 and W2. One continuous assign per reader, below.

   // Initial tails, sized: SH_IE/SH_FE start with N-32 free, SH_LD with all N.
   localparam [PW_IE-1:0] T0_IE = (N_IE - 32);
   localparam [PW_LD-1:0] T0_LD = N_LD;
   localparam [PW_FE-1:0] T0_FE = (N_FE - 32);
   localparam [PW_I2-1:0] T0_I2 = N_IE2;             // nothing maps there at reset: wholly free
   localparam [PW_I3-1:0] T0_I3 = N_IE3;             // likewise the third ALU's shard

   genvar gS;
   // ---- shard ie free list: FLNB banks, one muxed write (tail/tail+1), head/head+1 reads ----
   wire [IDXB-1:0] ha_rd_ie, hb_rd_ie, hcc_rd_ie;
   wire [IDXB-1:0] flrd_ie [0:FLNB-1];
   wire [FLNB-1:0] behind_ie = ~({FLNB{1'b1}} << h_ie[FLLB-1:0]);   // behind[g] = (g < h's bank): the banks that wrapped
   generate for (gS = 0; gS < FLNB; gS = gS + 1) begin: fl_ie
      (* ram_style = "distributed" *) reg [IDXB-1:0] mem [0:N_IE/FLNB-1];
      integer jj; integer pp; initial for (jj = 0; jj < N_IE/FLNB; jj = jj + 1) begin pp = 32 + FLNB*jj + gS; mem[jj] = pp[IDXB-1:0]; end
      wire w0 = fre_ie  & (t_ie [FLLB-1:0] == gS);
      wire w1 = fre2_ie & (t2_ie[FLLB-1:0] == gS);
      wire w2 = fre3_ie & (t3_ie[FLLB-1:0] == gS);   // 3rd free (IW>=3; dead otherwise)
      always @(posedge clk)
         if (w0 | w1 | w2) mem[w0 ? t_ie[PW_IE-2:FLLB] : w1 ? t2_ie[PW_IE-2:FLLB] : t3_ie[PW_IE-2:FLLB]]
                              <= w0 ? c_pold_q[IDXB-1:0] : w1 ? c2_pold_q[IDXB-1:0] : c3_pold_q[IDXB-1:0];
      // Bank gS's next free entry: h's index, plus one when this bank is behind h's bank.
      // Registers only -- which slot allocates from which shard (the class decode) selects
      // among the banks' OUTPUTS (ha/hb/hcc_rd below), not their addresses.
      assign flrd_ie[gS] = mem[h_ie[PW_IE-2:FLLB] + {{(PW_IE-2-FLLB){1'b0}}, behind_ie[gS]}];
   end endgenerate
   assign ha_rd_ie  = flrd_ie[h_ie [FLLB-1:0]];
   assign hb_rd_ie  = flrd_ie[hb_ie[FLLB-1:0]];
   assign hcc_rd_ie = flrd_ie[hcc_ie[FLLB-1:0]];

   // ---- shard ld free list: FLNB banks, one muxed write (tail/tail+1), head/head+1 reads ----
   wire [IDXB-1:0] ha_rd_ld, hb_rd_ld, hcc_rd_ld;
   wire [IDXB-1:0] flrd_ld [0:FLNB-1];
   wire [FLNB-1:0] behind_ld = ~({FLNB{1'b1}} << h_ld[FLLB-1:0]);   // behind[g] = (g < h's bank): the banks that wrapped
   generate for (gS = 0; gS < FLNB; gS = gS + 1) begin: fl_ld
      (* ram_style = "distributed" *) reg [IDXB-1:0] mem [0:N_LD/FLNB-1];
      integer jj; integer pp; initial for (jj = 0; jj < N_LD/FLNB; jj = jj + 1) begin pp = FLNB*jj + gS; mem[jj] = pp[IDXB-1:0]; end
      wire w0 = fre_ld  & (t_ld [FLLB-1:0] == gS);
      wire w1 = fre2_ld & (t2_ld[FLLB-1:0] == gS);
      wire w2 = fre3_ld & (t3_ld[FLLB-1:0] == gS);
      always @(posedge clk)
         if (w0 | w1 | w2) mem[w0 ? t_ld[PW_LD-2:FLLB] : w1 ? t2_ld[PW_LD-2:FLLB] : t3_ld[PW_LD-2:FLLB]]
                              <= w0 ? c_pold_q[IDXB-1:0] : w1 ? c2_pold_q[IDXB-1:0] : c3_pold_q[IDXB-1:0];
      // Bank gS's next free entry: h's index, plus one when this bank is behind h's bank.
      // Registers only -- which slot allocates from which shard (the class decode) selects
      // among the banks' OUTPUTS (ha/hb/hcc_rd below), not their addresses.
      assign flrd_ld[gS] = mem[h_ld[PW_LD-2:FLLB] + {{(PW_LD-2-FLLB){1'b0}}, behind_ld[gS]}];
   end endgenerate
   assign ha_rd_ld  = flrd_ld[h_ld [FLLB-1:0]];
   assign hb_rd_ld  = flrd_ld[hb_ld[FLLB-1:0]];
   assign hcc_rd_ld = flrd_ld[hcc_ld[FLLB-1:0]];

   // ---- shard fe free list: FLNB banks, one muxed write (tail/tail+1), head/head+1 reads ----
   wire [IDXB-1:0] ha_rd_fe, hb_rd_fe, hcc_rd_fe;
   wire [IDXB-1:0] flrd_fe [0:FLNB-1];
   wire [FLNB-1:0] behind_fe = ~({FLNB{1'b1}} << h_fe[FLLB-1:0]);   // behind[g] = (g < h's bank): the banks that wrapped
   generate for (gS = 0; gS < FLNB; gS = gS + 1) begin: fl_fe
      (* ram_style = "distributed" *) reg [IDXB-1:0] mem [0:N_FE/FLNB-1];
      integer jj; integer pp; initial for (jj = 0; jj < N_FE/FLNB; jj = jj + 1) begin pp = 32 + FLNB*jj + gS; mem[jj] = pp[IDXB-1:0]; end
      wire w0 = fre_fe  & (t_fe [FLLB-1:0] == gS);
      wire w1 = fre2_fe & (t2_fe[FLLB-1:0] == gS);
      wire w2 = fre3_fe & (t3_fe[FLLB-1:0] == gS);
      always @(posedge clk)
         if (w0 | w1 | w2) mem[w0 ? t_fe[PW_FE-2:FLLB] : w1 ? t2_fe[PW_FE-2:FLLB] : t3_fe[PW_FE-2:FLLB]]
                              <= w0 ? c_pold_q[IDXB-1:0] : w1 ? c2_pold_q[IDXB-1:0] : c3_pold_q[IDXB-1:0];
      // Bank gS's next free entry: h's index, plus one when this bank is behind h's bank.
      // Registers only -- which slot allocates from which shard (the class decode) selects
      // among the banks' OUTPUTS (ha/hb/hcc_rd below), not their addresses.
      assign flrd_fe[gS] = mem[h_fe[PW_FE-2:FLLB] + {{(PW_FE-2-FLLB){1'b0}}, behind_fe[gS]}];
   end endgenerate
   assign ha_rd_fe  = flrd_fe[h_fe [FLLB-1:0]];
   assign hb_rd_fe  = flrd_fe[hb_fe[FLLB-1:0]];
   assign hcc_rd_fe = flrd_fe[hcc_fe[FLLB-1:0]];

   // ---- shard i2 free list: FLNB banks, one muxed write (tail/tail+1), head/head+1 reads ----
   wire [IDXB-1:0] ha_rd_i2, hb_rd_i2, hcc_rd_i2;
   wire [IDXB-1:0] flrd_i2 [0:FLNB-1];
   wire [FLNB-1:0] behind_i2 = ~({FLNB{1'b1}} << h_i2[FLLB-1:0]);   // behind[g] = (g < h's bank): the banks that wrapped
   generate for (gS = 0; gS < FLNB; gS = gS + 1) begin: fl_i2
      (* ram_style = "distributed" *) reg [IDXB-1:0] mem [0:N_IE2/FLNB-1];
      integer jj; integer pp; initial for (jj = 0; jj < N_IE2/FLNB; jj = jj + 1) begin pp = FLNB*jj + gS; mem[jj] = pp[IDXB-1:0]; end
      wire w0 = fre_i2  & (t_i2 [FLLB-1:0] == gS);
      wire w1 = fre2_i2 & (t2_i2[FLLB-1:0] == gS);
      wire w2 = fre3_i2 & (t3_i2[FLLB-1:0] == gS);
      always @(posedge clk)
         if (w0 | w1 | w2) mem[w0 ? t_i2[PW_I2-2:FLLB] : w1 ? t2_i2[PW_I2-2:FLLB] : t3_i2[PW_I2-2:FLLB]]
                              <= w0 ? c_pold_q[IDXB-1:0] : w1 ? c2_pold_q[IDXB-1:0] : c3_pold_q[IDXB-1:0];
      // Bank gS's next free entry: h's index, plus one when this bank is behind h's bank.
      // Registers only -- which slot allocates from which shard (the class decode) selects
      // among the banks' OUTPUTS (ha/hb/hcc_rd below), not their addresses.
      assign flrd_i2[gS] = mem[h_i2[PW_I2-2:FLLB] + {{(PW_I2-2-FLLB){1'b0}}, behind_i2[gS]}];
   end endgenerate
   assign ha_rd_i2  = flrd_i2[h_i2 [FLLB-1:0]];
   assign hb_rd_i2  = flrd_i2[hb_i2[FLLB-1:0]];
   assign hcc_rd_i2 = flrd_i2[hcc_i2[FLLB-1:0]];

   // ---- shard i3 free list (the third ALU, Stage 3): identical shape to i2 ----
   wire [IDXB-1:0] ha_rd_i3, hb_rd_i3, hcc_rd_i3;
   wire [IDXB-1:0] flrd_i3 [0:FLNB-1];
   wire [FLNB-1:0] behind_i3 = ~({FLNB{1'b1}} << h_i3[FLLB-1:0]);   // behind[g] = (g < h's bank): the banks that wrapped
   generate for (gS = 0; gS < FLNB; gS = gS + 1) begin: fl_i3
      (* ram_style = "distributed" *) reg [IDXB-1:0] mem [0:N_IE3/FLNB-1];
      integer jj; integer pp; initial for (jj = 0; jj < N_IE3/FLNB; jj = jj + 1) begin pp = FLNB*jj + gS; mem[jj] = pp[IDXB-1:0]; end
      wire w0 = fre_i3  & (t_i3 [FLLB-1:0] == gS);
      wire w1 = fre2_i3 & (t2_i3[FLLB-1:0] == gS);
      wire w2 = fre3_i3 & (t3_i3[FLLB-1:0] == gS);
      always @(posedge clk)
         if (w0 | w1 | w2) mem[w0 ? t_i3[PW_I3-2:FLLB] : w1 ? t2_i3[PW_I3-2:FLLB] : t3_i3[PW_I3-2:FLLB]]
                              <= w0 ? c_pold_q[IDXB-1:0] : w1 ? c2_pold_q[IDXB-1:0] : c3_pold_q[IDXB-1:0];
      // Bank gS's next free entry: h's index, plus one when this bank is behind h's bank.
      // Registers only -- which slot allocates from which shard (the class decode) selects
      // among the banks' OUTPUTS (ha/hb/hcc_rd below), not their addresses.
      assign flrd_i3[gS] = mem[h_i3[PW_I3-2:FLLB] + {{(PW_I3-2-FLLB){1'b0}}, behind_i3[gS]}];
   end endgenerate
   assign ha_rd_i3  = flrd_i3[h_i3 [FLLB-1:0]];
   assign hb_rd_i3  = flrd_i3[hb_i3[FLLB-1:0]];
   assign hcc_rd_i3 = flrd_i3[hcc_i3[FLLB-1:0]];


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
   wire [PBITS-1:0] c_pold  = `RN_RMAP(c_rd);
   // the second commit displaces the FIRST's mapping when both write one architectural register
   wire [PBITS-1:0] c2_pold = (c_valid & c_rd_v & (c2_rd == c_rd)) ? c_prd : `RN_RMAP(c2_rd);
   // the third displaces the most recent PRIOR commit of the same register this cycle (c2, then c)
   wire [PBITS-1:0] c3_pold = (c2_valid & c2_rd_v & (c3_rd == c2_rd)) ? c2_prd
                            : (c_valid  & c_rd_v  & (c3_rd == c_rd))  ? c_prd
                            : `RN_RMAP(c3_rd);
   wire [2:0] c2_shard  = c2_prd[PBITS-1:IDXB];
   wire [2:0] pold2_sh  = c2_pold[PBITS-1:IDXB];
   wire cmt2_ie = c2_valid & c2_rd_v & (c2_shard == SH_IE);
   wire cmt2_ld = c2_valid & c2_rd_v & (c2_shard == SH_LD);
   wire cmt2_fe = c2_valid & c2_rd_v & (c2_shard == SH_FE);
   wire fre2_ie_c = c2_valid & c2_rd_v & (pold2_sh == SH_IE);
   wire fre2_ld_c = c2_valid & c2_rd_v & (pold2_sh == SH_LD);
   wire fre2_fe_c = c2_valid & c2_rd_v & (pold2_sh == SH_FE);
   wire cmt2_i2 = c2_valid & c2_rd_v & (c2_shard == SH_IE2);
   wire fre2_i2_c = c2_valid & c2_rd_v & (pold2_sh == SH_IE2);
   wire cmt2_i3 = c2_valid & c2_rd_v & (c2_shard == SH_IE3);
   wire fre2_i3_c = c2_valid & c2_rd_v & (pold2_sh == SH_IE3);
   wire [2:0] c3_shard  = c3_prd[PBITS-1:IDXB];
   wire [2:0] pold3_sh  = c3_pold[PBITS-1:IDXB];
   wire cmt3_ie = c3_valid & c3_rd_v & (c3_shard == SH_IE);
   wire cmt3_ld = c3_valid & c3_rd_v & (c3_shard == SH_LD);
   wire cmt3_fe = c3_valid & c3_rd_v & (c3_shard == SH_FE);
   wire cmt3_i2 = c3_valid & c3_rd_v & (c3_shard == SH_IE2);
   wire cmt3_i3 = c3_valid & c3_rd_v & (c3_shard == SH_IE3);
   wire fre3_ie_c = c3_valid & c3_rd_v & (pold3_sh == SH_IE);
   wire fre3_ld_c = c3_valid & c3_rd_v & (pold3_sh == SH_LD);
   wire fre3_fe_c = c3_valid & c3_rd_v & (pold3_sh == SH_FE);
   wire fre3_i2_c = c3_valid & c3_rd_v & (pold3_sh == SH_IE2);
   wire fre3_i3_c = c3_valid & c3_rd_v & (pold3_sh == SH_IE3);
   wire [2:0] c_shard = c_prd[PBITS-1:IDXB];
   wire [2:0] pold_sh = c_pold[PBITS-1:IDXB];
   wire cmt_ie = c_valid & c_rd_v & (c_shard == SH_IE);   // head advance: allocation shard
   wire cmt_ld = c_valid & c_rd_v & (c_shard == SH_LD);
   wire cmt_fe = c_valid & c_rd_v & (c_shard == SH_FE);
   wire cmt_i2 = c_valid & c_rd_v & (c_shard == SH_IE2);
   wire cmt_i3 = c_valid & c_rd_v & (c_shard == SH_IE3);
   wire fre_ie_c = c_valid & c_rd_v & (pold_sh == SH_IE);   // free push: the register's shard
   wire fre_ld_c = c_valid & c_rd_v & (pold_sh == SH_LD);
   wire fre_fe_c = c_valid & c_rd_v & (pold_sh == SH_FE);
   wire fre_i2_c = c_valid & c_rd_v & (pold_sh == SH_IE2);
   wire fre_i3_c = c_valid & c_rd_v & (pold_sh == SH_IE3);

   // ---- FLOP THE REGISTER RELEASE (2026-09-15) --------------------------------------
   // The freelist FREE (write + tail advance) is registered one cycle off the retire cone.
   // c_valid rides m_addr -> dTLB -> lsu_done -> retire, and fanning it to all five free
   // lists' write ports and tails combinationally was ~1400 of the IW=3 near-critical
   // endpoints (m_addr -> u_rename/fl_*.mem_reg).  The physreg is released one cycle later,
   // which is free here: the free lists are never the allocation bottleneck (shard_low
   // throttles the HEAD, and avail = tail - head simply counts a committed free one cycle
   // later, which is strictly MORE conservative).  ROLLBACK-SAFE WITH NO EXTRA HANDLING:
   // the tail advances ONLY on committed frees (fre_* come from c_valid) and flush never
   // touches the tail -- it restores the HEAD to hc_* -- so a pending free is always a
   // committed one that must complete, exactly what a plain register does.  c_pold is
   // captured HERE, at commit, while RMAP[c_rd] still holds the old mapping (a cycle later
   // RMAP[c_rd] is c_prd), so the registered copy frees the right register.
   reg fre_ie, fre_ld, fre_fe, fre_i2, fre_i3;
   reg fre2_ie, fre2_ld, fre2_fe, fre2_i2, fre2_i3;
   reg fre3_ie, fre3_ld, fre3_fe, fre3_i2, fre3_i3;
   reg [PBITS-1:0] c_pold_q, c2_pold_q, c3_pold_q;
   initial begin
      fre_ie=1'b0; fre_ld=1'b0; fre_fe=1'b0; fre_i2=1'b0; fre_i3=1'b0;
      fre2_ie=1'b0; fre2_ld=1'b0; fre2_fe=1'b0; fre2_i2=1'b0; fre2_i3=1'b0;
      fre3_ie=1'b0; fre3_ld=1'b0; fre3_fe=1'b0; fre3_i2=1'b0; fre3_i3=1'b0;
      c_pold_q={PBITS{1'b0}}; c2_pold_q={PBITS{1'b0}}; c3_pold_q={PBITS{1'b0}};
   end
   always @(posedge clk) begin
      fre_ie  <= ~reset & fre_ie_c;   fre_ld  <= ~reset & fre_ld_c;   fre_fe  <= ~reset & fre_fe_c;
      fre_i2  <= ~reset & fre_i2_c;   fre_i3  <= ~reset & fre_i3_c;
      fre2_ie <= ~reset & fre2_ie_c;  fre2_ld <= ~reset & fre2_ld_c;  fre2_fe <= ~reset & fre2_fe_c;
      fre2_i2 <= ~reset & fre2_i2_c;  fre2_i3 <= ~reset & fre2_i3_c;
      fre3_ie <= ~reset & fre3_ie_c;  fre3_ld <= ~reset & fre3_ld_c;  fre3_fe <= ~reset & fre3_fe_c;
      fre3_i2 <= ~reset & fre3_i2_c;  fre3_i3 <= ~reset & fre3_i3_c;
      c_pold_q <= c_pold;  c2_pold_q <= c2_pold;  c3_pold_q <= c3_pold;
   end
   wire [PW_IE-1:0] hc_ie_n = hc_ie + {{(PW_IE-1){1'b0}}, cmt_ie} + {{(PW_IE-1){1'b0}}, cmt2_ie} + {{(PW_IE-1){1'b0}}, cmt3_ie};
   wire [PW_LD-1:0] hc_ld_n = hc_ld + {{(PW_LD-1){1'b0}}, cmt_ld} + {{(PW_LD-1){1'b0}}, cmt2_ld} + {{(PW_LD-1){1'b0}}, cmt3_ld};
   wire [PW_FE-1:0] hc_fe_n = hc_fe + {{(PW_FE-1){1'b0}}, cmt_fe} + {{(PW_FE-1){1'b0}}, cmt2_fe} + {{(PW_FE-1){1'b0}}, cmt3_fe};
   wire [PW_I2-1:0] hc_i2_n = hc_i2 + {{(PW_I2-1){1'b0}}, cmt_i2} + {{(PW_I2-1){1'b0}}, cmt2_i2} + {{(PW_I2-1){1'b0}}, cmt3_i2};
   wire [PW_I3-1:0] hc_i3_n = hc_i3 + {{(PW_I3-1){1'b0}}, cmt_i3} + {{(PW_I3-1){1'b0}}, cmt2_i3} + {{(PW_I3-1){1'b0}}, cmt3_i3};

   wire [PW_IE-2:0] t2_ie = t_ie[PW_IE-2:0] + {{(PW_IE-2){1'b0}}, fre_ie};   // the second free's slot
   wire [PW_LD-2:0] t2_ld = t_ld[PW_LD-2:0] + {{(PW_LD-2){1'b0}}, fre_ld};
   wire [PW_FE-2:0] t2_fe = t_fe[PW_FE-2:0] + {{(PW_FE-2){1'b0}}, fre_fe};
   wire [PW_I2-2:0] t2_i2 = t_i2[PW_I2-2:0] + {{(PW_I2-2){1'b0}}, fre_i2};
   wire [PW_I3-2:0] t2_i3 = t_i3[PW_I3-2:0] + {{(PW_I3-2){1'b0}}, fre_i3};
   // the third free's slot = tail + (# of earlier frees this cycle)
   wire [PW_IE-2:0] t3_ie = t_ie[PW_IE-2:0] + {{(PW_IE-2){1'b0}}, fre_ie} + {{(PW_IE-2){1'b0}}, fre2_ie};
   wire [PW_LD-2:0] t3_ld = t_ld[PW_LD-2:0] + {{(PW_LD-2){1'b0}}, fre_ld} + {{(PW_LD-2){1'b0}}, fre2_ld};
   wire [PW_FE-2:0] t3_fe = t_fe[PW_FE-2:0] + {{(PW_FE-2){1'b0}}, fre_fe} + {{(PW_FE-2){1'b0}}, fre2_fe};
   wire [PW_I2-2:0] t3_i2 = t_i2[PW_I2-2:0] + {{(PW_I2-2){1'b0}}, fre_i2} + {{(PW_I2-2){1'b0}}, fre2_i2};
   wire [PW_I3-2:0] t3_i3 = t_i3[PW_I3-2:0] + {{(PW_I3-2){1'b0}}, fre_i3} + {{(PW_I3-2){1'b0}}, fre2_i3};
   wire [PW_IE-1:0] avail_ie = t_ie - h_ie;
   wire [PW_LD-1:0] avail_ld = t_ld - h_ld;
   wire [PW_FE-1:0] avail_fe = t_fe - h_fe;
   wire [PW_I2-1:0] avail_i2 = t_i2 - h_i2;
   wire [PW_I3-1:0] avail_i3 = t_i3 - h_i3;
   // STALL WHEN *ANY* SHARD IS LOW, not when the destination's shard is.  A shard that runs
   // dry stalls rename regardless of which one the next instruction wants, so throttling on
   // the minimum is what actually prevents the stall; throttling per-destination only
   // discovers it one instruction too late.  The cost is that the stall probability is the
   // union across shards -- which is the argument for sizing them unequally rather than
   // adding more of them.
   assign shard_low = {avail_i3 < LOWAT[PW_I3-1:0],
                       avail_i2 < LOWAT[PW_I2-1:0],
                       avail_fe < LOWAT[PW_FE-1:0],
                       avail_ld < LOWAT[PW_LD-1:0],
                       avail_ie < LOWAT[PW_IE-1:0]};
   assign stall = |shard_low;

   // THE FREE-LIST READ ADDRESSES DO NOT SEE THE STALL. A stall holds every slot, so the
   // heads do not move and the entries read at h, h+A, h+A+B are simply not consumed; the
   // stall belongs only on the head ADVANCE and on the alloc outputs. With it in the address
   // the whole rename decision -- five tail-minus-head subtractions, the low-water compares,
   // the OR -- sat in front of the LUTRAM read, the read's data in front of the dispatch
   // stage and the store queue: t_ld -> avail -> stall -> hb -> flrd -> stg_ps -> u_sq/ld_w,
   // 20-23 levels, eight families of the IW=3 census. (ar_*/br_* address; a_*/b_* advance.)
   wire alloc_r   = r_valid   & r_rd_v;
   wire alloc_r_b = r_valid_b & r_rd_v_b;
   wire alloc   = alloc_r   & ~stall;
   wire alloc_b = alloc_r_b & ~stall;
   wire alloc_c = r_valid_c & r_rd_v_c & ~stall;
   wire a_ie = alloc & (r_shard == SH_IE), a_ld = alloc & (r_shard == SH_LD), a_fe = alloc & (r_shard == SH_FE), a_i2 = alloc & (r_shard == SH_IE2), a_i3 = alloc & (r_shard == SH_IE3);
   wire b_ie = alloc_b & (r_shard_b == SH_IE), b_ld = alloc_b & (r_shard_b == SH_LD), b_fe = alloc_b & (r_shard_b == SH_FE), b_i2 = alloc_b & (r_shard_b == SH_IE2), b_i3 = alloc_b & (r_shard_b == SH_IE3);
   wire ar_ie = alloc_r & (r_shard == SH_IE), ar_ld = alloc_r & (r_shard == SH_LD), ar_fe = alloc_r & (r_shard == SH_FE), ar_i2 = alloc_r & (r_shard == SH_IE2), ar_i3 = alloc_r & (r_shard == SH_IE3);
   wire br_ie = alloc_r_b & (r_shard_b == SH_IE), br_ld = alloc_r_b & (r_shard_b == SH_LD), br_fe = alloc_r_b & (r_shard_b == SH_FE), br_i2 = alloc_r_b & (r_shard_b == SH_IE2), br_i3 = alloc_r_b & (r_shard_b == SH_IE3);
   wire c_ie = alloc_c & (r_shard_c == SH_IE), c_ld = alloc_c & (r_shard_c == SH_LD), c_fe = alloc_c & (r_shard_c == SH_FE), c_i2 = alloc_c & (r_shard_c == SH_IE2), c_i3 = alloc_c & (r_shard_c == SH_IE3);
   wire [IDXB-1:0] head_idx = (r_shard == SH_IE) ? ha_rd_ie
                            : (r_shard == SH_LD) ? ha_rd_ld
                            : (r_shard == SH_FE) ? ha_rd_fe
                            : (r_shard == SH_IE2)? ha_rd_i2
                                                 : ha_rd_i3;
   assign r_prd = {r_shard, head_idx};
   // B's entry: the head, or the one after it when A allocates from the same shard. A second
   // LUTRAM read port, not a second pointer; LOWAT >= 2 keeps both inside the free set.
   wire [PW_IE-2:0] hb_ie = h_ie[PW_IE-2:0] + {{(PW_IE-2){1'b0}}, ar_ie};
   wire [PW_LD-2:0] hb_ld = h_ld[PW_LD-2:0] + {{(PW_LD-2){1'b0}}, ar_ld};
   wire [PW_FE-2:0] hb_fe = h_fe[PW_FE-2:0] + {{(PW_FE-2){1'b0}}, ar_fe};
   wire [PW_I2-2:0] hb_i2 = h_i2[PW_I2-2:0] + {{(PW_I2-2){1'b0}}, ar_i2};
   wire [PW_I3-2:0] hb_i3 = h_i3[PW_I3-2:0] + {{(PW_I3-2){1'b0}}, ar_i3};
   wire [IDXB-1:0] head_idx_b = (r_shard_b == SH_IE) ? hb_rd_ie
                              : (r_shard_b == SH_LD) ? hb_rd_ld
                              : (r_shard_b == SH_FE) ? hb_rd_fe
                              : (r_shard_b == SH_IE2)? hb_rd_i2
                                                     : hb_rd_i3;
   assign r_prd_b = {r_shard_b, head_idx_b};
   // C's entry: the head plus the number of EARLIER allocations from the same shard this cycle.
   // A third LUTRAM read port; LOWAT >= 3 keeps all three inside the free set.
   wire [PW_IE-2:0] hcc_ie = h_ie[PW_IE-2:0] + {{(PW_IE-2){1'b0}}, ar_ie} + {{(PW_IE-2){1'b0}}, br_ie};
   wire [PW_LD-2:0] hcc_ld = h_ld[PW_LD-2:0] + {{(PW_LD-2){1'b0}}, ar_ld} + {{(PW_LD-2){1'b0}}, br_ld};
   wire [PW_FE-2:0] hcc_fe = h_fe[PW_FE-2:0] + {{(PW_FE-2){1'b0}}, ar_fe} + {{(PW_FE-2){1'b0}}, br_fe};
   wire [PW_I2-2:0] hcc_i2 = h_i2[PW_I2-2:0] + {{(PW_I2-2){1'b0}}, ar_i2} + {{(PW_I2-2){1'b0}}, br_i2};
   wire [PW_I3-2:0] hcc_i3 = h_i3[PW_I3-2:0] + {{(PW_I3-2){1'b0}}, ar_i3} + {{(PW_I3-2){1'b0}}, br_i3};
   wire [IDXB-1:0] head_idx_c = (r_shard_c == SH_IE) ? hcc_rd_ie
                              : (r_shard_c == SH_LD) ? hcc_rd_ld
                              : (r_shard_c == SH_FE) ? hcc_rd_fe
                              : (r_shard_c == SH_IE2)? hcc_rd_i2
                                                     : hcc_rd_i3;
   assign r_prd_c = {r_shard_c, head_idx_c};

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
      // The pointers come from configuration for the same reason the arrays do. They are
      // the ONLY thing that says which slots are free, so resetting them while the arrays
      // keep the previous run's contents would republish stale slots as free and hand out
      // DUPLICATE physical registers -- the one combination that is worse than either
      // choice alone.
      h_ie = {PW_IE{1'b0}}; hc_ie = {PW_IE{1'b0}}; t_ie = T0_IE;
      h_ld = {PW_LD{1'b0}}; hc_ld = {PW_LD{1'b0}}; t_ld = T0_LD;
      h_fe = {PW_FE{1'b0}}; hc_fe = {PW_FE{1'b0}}; t_fe = T0_FE;
      h_i2 = {PW_I2{1'b0}}; hc_i2 = {PW_I2{1'b0}}; t_i2 = T0_I2;
      h_i3 = {PW_I3{1'b0}}; hc_i3 = {PW_I3{1'b0}}; t_i3 = T0_I3;
      lv = 64'd0;
      for (j = 0; j < 64; j = j + 1) begin newer[j] = 2'd0; rnewer[j] = 2'd0; end
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
         h_i2 <= hc_i2;
         h_i3 <= hc_i3;
      end else begin
         // ---- commit: RMAP takes the committed mapping, the displaced register is freed
         hc_ie <= hc_ie_n;  hc_ld <= hc_ld_n;  hc_fe <= hc_fe_n;  hc_i2 <= hc_i2_n;  hc_i3 <= hc_i3_n;
         if (c_valid & c_rd_v) begin
            rmap_a[c_rd] <= c_prd;  rnewer[c_rd] <= 2'd0;
         end
         if (c2_valid & c2_rd_v) begin           // after the head's: the same register twice keeps the second
            rmap_b[c2_rd] <= c2_prd;  rnewer[c2_rd] <= 2'd1;
         end
         if (c3_valid & c3_rd_v) begin           // ...and the third youngest of all
            rmap_c[c3_rd] <= c3_prd;  rnewer[c3_rd] <= 2'd2;
         end
         // the frees: the head's at the tail, the later ones at the slots after it
         t_ie <= t_ie + {{(PW_IE-1){1'b0}}, fre_ie} + {{(PW_IE-1){1'b0}}, fre2_ie} + {{(PW_IE-1){1'b0}}, fre3_ie};
         t_ld <= t_ld + {{(PW_LD-1){1'b0}}, fre_ld} + {{(PW_LD-1){1'b0}}, fre2_ld} + {{(PW_LD-1){1'b0}}, fre3_ld};
         t_fe <= t_fe + {{(PW_FE-1){1'b0}}, fre_fe} + {{(PW_FE-1){1'b0}}, fre2_fe} + {{(PW_FE-1){1'b0}}, fre3_fe};
         t_i2 <= t_i2 + {{(PW_I2-1){1'b0}}, fre_i2} + {{(PW_I2-1){1'b0}}, fre2_i2} + {{(PW_I2-1){1'b0}}, fre3_i2};
         t_i3 <= t_i3 + {{(PW_I3-1){1'b0}}, fre_i3} + {{(PW_I3-1){1'b0}}, fre2_i3} + {{(PW_I3-1){1'b0}}, fre3_i3};

         // ---- rename: SMAP takes the new mapping, the head advances.  A flush in the same
         // cycle squashes this instruction, so the flush arm below wins on the pointers;
         // lv is cleared wholesale so the SMAP write becomes invisible either way.
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
         if (flush) begin
            lv   <= 64'd0;
            h_ie <= hc_ie_n;
            h_ld <= hc_ld_n;
            h_fe <= hc_fe_n;
            h_i2 <= hc_i2_n;
            h_i3 <= hc_i3_n;
         end else begin
            h_ie <= h_ie + {{(PW_IE-1){1'b0}}, a_ie} + {{(PW_IE-1){1'b0}}, b_ie} + {{(PW_IE-1){1'b0}}, c_ie};
            h_ld <= h_ld + {{(PW_LD-1){1'b0}}, a_ld} + {{(PW_LD-1){1'b0}}, b_ld} + {{(PW_LD-1){1'b0}}, c_ld};
            h_fe <= h_fe + {{(PW_FE-1){1'b0}}, a_fe} + {{(PW_FE-1){1'b0}}, b_fe} + {{(PW_FE-1){1'b0}}, c_fe};
            h_i2 <= h_i2 + {{(PW_I2-1){1'b0}}, a_i2} + {{(PW_I2-1){1'b0}}, b_i2} + {{(PW_I2-1){1'b0}}, c_i2};
            h_i3 <= h_i3 + {{(PW_I3-1){1'b0}}, a_i3} + {{(PW_I3-1){1'b0}}, b_i3} + {{(PW_I3-1){1'b0}}, c_i3};
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
      if (c_valid && c_rd_v && (c_prd[PBITS-1:IDXB] != c_shard))
         $fatal(1, "ooo2_rename: committing pr=%h whose shard is not %0d", c_prd, c_shard);
      if (c2_valid && !c_valid)
         $fatal(1, "ooo2_rename: second commit without a first");
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
   `undef RN_SMAP
   `undef RN_RMAP
endmodule

`default_nettype wire
