`default_nettype none

// Full 4-wide renamer core: SHARDS rename_shard slices wired through the
// cross-shard broadcast network. The cross-slot dependency matrix
// (decode_xslot) is NOT here -- it lives in the decode stage and its results
// arrive as inputs (s*_is_slot/s*_slot/map_writer/d_is_slot/d_slot), so the
// matrix is computed exactly once and crosses into rename across a registered
// stage boundary (see decode_rename.v). This module is purely the rename loop +
// broadcasts.
//
// Unified geometry: AREGS=64 (int+FP), NPHYS=256, POOL=64/shard, NCHK=4.
//
// Wiring of the broadcasts (all combinational this cycle):
//   alloc[i]    = shard i's freshly allocated phys dest (pdst)
//   al_phys     = {alloc} broadcast  -> resolves SLOT(j) sources + d_slot pold
//   wr_phys     = {alloc} broadcast  -> MAP write data
//   wr_arch     = {rd}    broadcast  -> MAP write address
//   wr_valid    = map_writer (input) -> only the last-writer-per-arch updates MAP
//   pold_bus    = {pold}  broadcast  -> each shard's freelist records the polds it
//                                       owns into P[cur] (freed when cur commits)
//   pold_valid  = {pold_v}           -> valid per displacing instruction
//   d_valid[i]  = rd_v[i]  (input)   -> shard i allocates for any register write
//
// Checkpoint / commit control (create/commit/rollback + indices) is driven in
// lockstep into every shard, so all four freelists keep an identical `cur`; we
// surface shard 0's as the bundle's `cur`.
module renamer_bundle
  #(parameter SHARDS = 4,
    parameter ABITS  = 6,    // unified arch space 0..63
    parameter AREGS  = 64,
    parameter PBITS  = 8,    // 256 phys regs
    parameter NPHYS  = 256,
    parameter POOL   = 64,   // NPHYS/SHARDS
    parameter HPTR   = 6,    // clog2(POOL)
    parameter SBITS  = 2,    // clog2(SHARDS)
    parameter NCHK   = 4,
    parameter CBITS  = 2)
   (input  wire                    clk,
    input  wire                    reset,
    // decoded operands (slot 0 = oldest)
    input  wire [SHARDS*ABITS-1:0] rs1,
    input  wire [SHARDS*ABITS-1:0] rs2,
    input  wire [SHARDS*ABITS-1:0] rs3,
    input  wire [SHARDS*ABITS-1:0] rd,
    input  wire [SHARDS-1:0]       rd_v,
    // cross-slot resolution, precomputed in decode (decode_xslot)
    input  wire [SHARDS-1:0]       s1_is_slot,
    input  wire [SHARDS*SBITS-1:0] s1_slot,
    input  wire [SHARDS-1:0]       s2_is_slot,
    input  wire [SHARDS*SBITS-1:0] s2_slot,
    input  wire [SHARDS-1:0]       s3_is_slot,
    input  wire [SHARDS*SBITS-1:0] s3_slot,
    input  wire [SHARDS-1:0]       map_writer,
    input  wire [SHARDS-1:0]       d_is_slot,
    input  wire [SHARDS*SBITS-1:0] d_slot,
    // checkpoint / commit control
    input  wire                    create,       // per-bundle dispatch (alloc/MAP/pold)
    input  wire                    ckpt_create,  // per-checkpoint close (coarse CPR: span + chk_map)
    input  wire                    commit,
    input  wire [CBITS-1:0]        commit_idx,
    input  wire                    rollback,
    input  wire [CBITS-1:0]        rollback_idx,
    output wire [SHARDS*PBITS-1:0] ps1,
    output wire [SHARDS*PBITS-1:0] ps2,
    output wire [SHARDS*PBITS-1:0] ps3,
    output wire [SHARDS*PBITS-1:0] pdst,
    output wire [CBITS-1:0]        cur,
    output wire [SHARDS-1:0]       stall);

   // --- broadcast buses built from the shards' own allocations / displacements
   wire [SHARDS*PBITS-1:0] alloc;        // = pdst of each shard
   wire [SHARDS*PBITS-1:0] pold_bus;     // = pold of each shard
   wire [SHARDS-1:0]       pold_valid;
   wire [SHARDS*ABITS-1:0] wr_arch  = rd;
   wire [SHARDS*PBITS-1:0] wr_phys  = alloc;
   wire [SHARDS-1:0]       wr_valid = map_writer;
   wire [SHARDS*PBITS-1:0] al_phys  = alloc;
   wire [SHARDS*CBITS-1:0] cur_each;

   assign pdst = alloc;
   assign cur  = cur_each[0*CBITS +: CBITS];   // all shards identical

   // --- one rename_shard per lane
   genvar i;
   generate
      for (i = 0; i < SHARDS; i = i + 1) begin : lane
         rename_shard #(.SHARDS(SHARDS), .SH(i), .AREGS(AREGS), .ABITS(ABITS),
                        .NPHYS(NPHYS), .PBITS(PBITS), .POOL(POOL), .HPTR(HPTR),
                        .SBITS(SBITS), .NCHK(NCHK), .CBITS(CBITS)) sh
           (.clk(clk), .reset(reset),
            .s1_arch(rs1[i*ABITS +: ABITS]), .s1_is_slot(s1_is_slot[i]),
            .s1_slot(s1_slot[i*SBITS +: SBITS]),
            .s2_arch(rs2[i*ABITS +: ABITS]), .s2_is_slot(s2_is_slot[i]),
            .s2_slot(s2_slot[i*SBITS +: SBITS]),
            .s3_arch(rs3[i*ABITS +: ABITS]), .s3_is_slot(s3_is_slot[i]),
            .s3_slot(s3_slot[i*SBITS +: SBITS]),
            .d_arch(rd[i*ABITS +: ABITS]), .d_valid(rd_v[i]),
            .d_is_slot(d_is_slot[i]), .d_slot(d_slot[i*SBITS +: SBITS]),
            .wr_arch(wr_arch), .wr_phys(wr_phys), .wr_valid(wr_valid),
            .al_phys(al_phys),
            .pold_valid(pold_valid), .pold_bus(pold_bus),
            .create(create), .ckpt_create(ckpt_create), .commit(commit), .commit_idx(commit_idx),
            .rollback(rollback), .rollback_idx(rollback_idx),
            .ps1(ps1[i*PBITS +: PBITS]), .ps2(ps2[i*PBITS +: PBITS]),
            .ps3(ps3[i*PBITS +: PBITS]),
            .pdst(alloc[i*PBITS +: PBITS]), .pold(pold_bus[i*PBITS +: PBITS]),
            .pold_v(pold_valid[i]), .cur(cur_each[i*CBITS +: CBITS]),
            .stall(stall[i]));
      end
   endgenerate

`ifdef FL_ASSERT
   // Invariant checker: the architectural map must NEVER point to a FREE physreg.
   // If a rollback/commit frees a physreg that is still some arch reg's live mapping,
   // a later reader (e.g. a re-dispatched store reading rs2) gets garbage. This fires
   // locally the cycle after the bad free -- pinpointing the freelist/recovery bug,
   // vs a downstream value divergence millions of instructions later. The map is
   // replicated (read lane[0]); each shard owns physregs with pr[SBITS-1:0]==shard.
   generate
   if (SBITS == 0) begin : flchk   // IW=1 with a 0-bit shard field -> physreg index == fap
      integer fa; reg [PBITS-1:0] fap;
      always @(posedge clk) if (!reset)
         for (fa = 0; fa < AREGS; fa = fa + 1) begin
            fap = lane[0].sh.map[fa];
            if (lane[0].sh.fl.free[fap]) begin
               $display("[%0t] *** FL-ASSERT: arch r%0d -> phys %0d is FREE (rollback=%b rb_idx=%0d commit=%b cmt_idx=%0d)",
                  $time, fa, fap, rollback, rollback_idx, commit, commit_idx);
               $finish;
            end
         end
   end else begin : flchk
      genvar gj;
      for (gj = 0; gj < SHARDS; gj = gj + 1) begin : perlane
         integer fa; reg [PBITS-1:0] fap;
         always @(posedge clk) if (!reset)
            for (fa = 0; fa < AREGS; fa = fa + 1) begin
               fap = lane[0].sh.map[fa];
               if ((fap[SBITS-1:0] == gj[SBITS-1:0]) && lane[gj].sh.fl.free[fap[PBITS-1:SBITS]]) begin
                  $display("[%0t] *** FL-ASSERT: arch r%0d -> phys %0d is FREE (rollback=%b rb_idx=%0d commit=%b cmt_idx=%0d)",
                     $time, fa, fap, rollback, rollback_idx, commit, commit_idx);
                  $finish;
               end
            end
      end
   end
   endgenerate
`endif
endmodule

`default_nettype wire
