`default_nettype none

// Register renaming for the in-order core: SMAP/RMAP + lv[], and one free list per PRF
// shard.  Issue and commit remain IN ORDER at this milestone -- this module changes no
// architectural behaviour, so the retire stream must stay bit-identical.  It exists to give
// ino_prf a known destination shard, which is the only cheap way to get more than one write
// port (see ino_prf.v's header).
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
// and RMAP are arrays the tools can infer as LUTRAM.  Scalar issue only -- at IW>1 SMAP
// needs multiple write ports, which is the same problem one level up.
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

module ino_rename
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
    output wire [PBITS-1:0] r_prd,        // newly allocated physical register
    output wire [PBITS-1:0] r_pold,       // the mapping it displaces -- freed at commit

    // ---- commit port (in order, one per cycle) ----
    input  wire             c_valid,
    input  wire [5:0]       c_rd,
    input  wire             c_rd_v,
    input  wire [1:0]       c_shard,
    input  wire [PBITS-1:0] c_prd,        // becomes the committed mapping
    input  wire [PBITS-1:0] c_pold,       // returns to c_shard's free list

    // ---- recovery ----
    input  wire             flush,        // total squash: everything uncommitted dies

    // ---- back pressure and instrumentation ----
    output wire             stall,        // ANY shard low -- see the note below
    output wire [2:0]       shard_low);   // per-shard, for the hpm counters

   localparam [1:0] SH_IE = 2'd0, SH_LD = 2'd1, SH_FE = 2'd2;
   localparam [IDXB-1:0] OFF32 = 32;   // sized, so the inits do not truncate

   // ---- the map -------------------------------------------------------------------
   reg [PBITS-1:0] smap [0:63];
   reg [PBITS-1:0] rmap [0:63];
   reg [63:0]      lv;

   // Reads are of the PRE-rename mapping for all three sources, including the case where a
   // source equals this instruction's own destination -- rd is written at the clock edge,
   // so the combinational reads below see the old value by construction.  (Matches
   // docs/Area-Efficient-Scalar-OoO.md 14.1 "dispatch -> dispatch, map".)
   assign r_prs1 = lv[r_rs1] ? smap[r_rs1] : rmap[r_rs1];
   assign r_prs2 = lv[r_rs2] ? smap[r_rs2] : rmap[r_rs2];
   assign r_prs3 = lv[r_rs3] ? smap[r_rs3] : rmap[r_rs3];
   assign r_pold = lv[r_rd]  ? smap[r_rd]  : rmap[r_rd];

   // ---- free lists, one per shard ---------------------------------------------------
   // Pointers carry an extra MSB so full and empty are distinguishable without a separate
   // count: the list is empty when h == t, full when h == t ^ {1'b1, 0}.
   localparam integer PW_IE = $clog2(N_IE) + 1;
   localparam integer PW_LD = $clog2(N_LD) + 1;
   localparam integer PW_FE = $clog2(N_FE) + 1;

   reg [IDXB-1:0] fl_ie [0:N_IE-1];   reg [PW_IE-1:0] h_ie, hc_ie, t_ie;
   reg [IDXB-1:0] fl_ld [0:N_LD-1];   reg [PW_LD-1:0] h_ld, hc_ld, t_ld;
   reg [IDXB-1:0] fl_fe [0:N_FE-1];   reg [PW_FE-1:0] h_fe, hc_fe, t_fe;

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
   wire [1:0] pold_sh = c_pold[PBITS-1:IDXB];
   wire cmt_ie = c_valid & c_rd_v & (c_shard == SH_IE);   // head advance: allocation shard
   wire cmt_ld = c_valid & c_rd_v & (c_shard == SH_LD);
   wire cmt_fe = c_valid & c_rd_v & (c_shard == SH_FE);
   wire fre_ie = c_valid & c_rd_v & (pold_sh == SH_IE);   // free push: the register's shard
   wire fre_ld = c_valid & c_rd_v & (pold_sh == SH_LD);
   wire fre_fe = c_valid & c_rd_v & (pold_sh == SH_FE);
   wire [PW_IE-1:0] hc_ie_n = hc_ie + {{(PW_IE-1){1'b0}}, cmt_ie};
   wire [PW_LD-1:0] hc_ld_n = hc_ld + {{(PW_LD-1){1'b0}}, cmt_ld};
   wire [PW_FE-1:0] hc_fe_n = hc_fe + {{(PW_FE-1){1'b0}}, cmt_fe};

   wire [PW_IE-1:0] avail_ie = t_ie - h_ie;
   wire [PW_LD-1:0] avail_ld = t_ld - h_ld;
   wire [PW_FE-1:0] avail_fe = t_fe - h_fe;

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

   wire alloc = r_valid & r_rd_v & ~stall;
   wire [IDXB-1:0] head_idx = (r_shard == SH_IE) ? fl_ie[h_ie[PW_IE-2:0]]
                            : (r_shard == SH_LD) ? fl_ld[h_ld[PW_LD-2:0]]
                                                 : fl_fe[h_fe[PW_FE-2:0]];
   assign r_prd = {r_shard, head_idx};

   integer i;
   always @(posedge clk) begin
      if (reset) begin
         // x0 -> physical 0 (SH_IE index 0), which ino_prf hardwires to read zero and
         // never writes.  Integer regs start in SH_IE, FP regs in SH_FE; the load shard
         // starts entirely free.
         for (i = 0; i < 32; i = i + 1) begin
            rmap[i]      <= {SH_IE, i[IDXB-1:0]};
            smap[i]      <= {SH_IE, i[IDXB-1:0]};
            rmap[32 + i] <= {SH_FE, i[IDXB-1:0]};
            smap[32 + i] <= {SH_FE, i[IDXB-1:0]};
         end
         // Indices 0..31 of SH_IE and SH_FE are taken by the initial architectural
         // mappings, so their free lists start with 32..N-1 -- N-32 entries.  SH_LD starts
         // wholly free.  Slots at or beyond the tail are never read (a circular FIFO only
         // reads between head and tail) but are given a legal index anyway so a pointer bug
         // shows up as an assertion rather than as an out-of-range PRF access.
         for (i = 0; i < N_IE; i = i + 1) fl_ie[i] <= OFF32 + i[IDXB-1:0];
         for (i = 0; i < N_LD; i = i + 1) fl_ld[i] <= i[IDXB-1:0];
         for (i = 0; i < N_FE; i = i + 1) fl_fe[i] <= OFF32 + i[IDXB-1:0];
         lv <= 64'd0;
         h_ie <= {PW_IE{1'b0}}; hc_ie <= {PW_IE{1'b0}}; t_ie <= T0_IE;
         h_ld <= {PW_LD{1'b0}}; hc_ld <= {PW_LD{1'b0}}; t_ld <= T0_LD;
         h_fe <= {PW_FE{1'b0}}; hc_fe <= {PW_FE{1'b0}}; t_fe <= T0_FE;
      end else begin
         // ---- commit: RMAP takes the committed mapping, the displaced register is freed
         hc_ie <= hc_ie_n;  hc_ld <= hc_ld_n;  hc_fe <= hc_fe_n;
         if (c_valid & c_rd_v) begin
            rmap[c_rd] <= c_prd;
            if (fre_ie) begin fl_ie[t_ie[PW_IE-2:0]] <= c_pold[IDXB-1:0]; t_ie <= t_ie + 1'b1; end
            if (fre_ld) begin fl_ld[t_ld[PW_LD-2:0]] <= c_pold[IDXB-1:0]; t_ld <= t_ld + 1'b1; end
            if (fre_fe) begin fl_fe[t_fe[PW_FE-2:0]] <= c_pold[IDXB-1:0]; t_fe <= t_fe + 1'b1; end
            if (c_shard > SH_FE) $fatal(1, "ino_rename: commit to shard %0d", c_shard);
         end

         // ---- rename: SMAP takes the new mapping, the head advances.  A flush in the same
         // cycle squashes this instruction, so the flush arm below wins on the pointers;
         // lv is cleared wholesale so the SMAP write becomes invisible either way.
         if (alloc) begin
            smap[r_rd] <= r_prd;
            lv[r_rd]   <= 1'b1;
            if (r_shard > SH_FE) $fatal(1, "ino_rename: rename to shard %0d", r_shard);
         end

         // ---- rollback
         if (flush) begin
            lv   <= 64'd0;
            h_ie <= hc_ie_n;
            h_ld <= hc_ld_n;
            h_fe <= hc_fe_n;
         end else if (alloc) begin
            if (r_shard == SH_IE) h_ie <= h_ie + 1'b1;
            if (r_shard == SH_LD) h_ld <= h_ld + 1'b1;
            if (r_shard == SH_FE) h_fe <= h_fe + 1'b1;
         end
      end
   end

   // ---- invariants: ALWAYS ON, per docs/rtl-rules.md ---------------------------------
   always @(posedge clk) if (!reset) begin
      // Allocating from an empty list would hand out a register that is still live.  The
      // stall above is supposed to make this unreachable; "supposed to" is what assertions
      // are for.
      if (alloc && r_shard == SH_IE && avail_ie == 0)
         $fatal(1, "ino_rename: allocated from an empty int-exec free list");
      if (alloc && r_shard == SH_LD && avail_ld == 0)
         $fatal(1, "ino_rename: allocated from an empty load free list");
      if (alloc && r_shard == SH_FE && avail_fe == 0)
         $fatal(1, "ino_rename: allocated from an empty fp-exec free list");
      // c_pold may legitimately belong to a DIFFERENT shard than c_shard (see pold_sh
      // above); what must hold is that it names a shard that exists, so it is returned to a
      // real free list rather than dropped.
      if (c_valid && c_rd_v && (pold_sh > SH_FE))
         $fatal(1, "ino_rename: freeing pr=%h whose shard %0d does not exist",
                c_pold, pold_sh);
      if (c_valid && c_rd_v && (c_prd[PBITS-1:IDXB] != c_shard))
         $fatal(1, "ino_rename: committing pr=%h whose shard is not %0d", c_prd, c_shard);
      // x0 must never be renamed: it has no value to hold and freeing it would inject
      // physical register 0 into a free list.
      if (alloc && r_rd == 6'd0)
         $fatal(1, "ino_rename: renamed x0");
   end

   initial begin
      // Sizes MUST be powers of two: the free-list pointers carry one extra MSB and index
      // with the low bits, which only wraps correctly at a power of two.  At N=40 the
      // pointer walked past the end of the array and read 0 -- i.e. handed out physical
      // register 0, the architectural zero.  Caught by ino_prf's pr0 assertion on the
      // first riscv-test; checked here so it cannot come back.
      if ((N_IE & (N_IE-1)) != 0) $fatal(1, "ino_rename: N_IE=%0d is not a power of two", N_IE);
      if ((N_LD & (N_LD-1)) != 0) $fatal(1, "ino_rename: N_LD=%0d is not a power of two", N_LD);
      if ((N_FE & (N_FE-1)) != 0) $fatal(1, "ino_rename: N_FE=%0d is not a power of two", N_FE);
      if (N_IE <= 32) $fatal(1, "ino_rename: N_IE=%0d must exceed 32", N_IE);
      if (N_LD <= 64) $fatal(1, "ino_rename: N_LD=%0d must exceed 64", N_LD);
      if (N_FE <= 32) $fatal(1, "ino_rename: N_FE=%0d must exceed 32 (fp only)", N_FE);
      if (LOWAT < 1) $fatal(1, "ino_rename: LOWAT must be >= 1");
   end
endmodule

`default_nettype wire
