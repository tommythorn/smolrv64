`default_nettype none

// Full 4-wide renamer bundle: the cross-slot dependency matrix (decode_xslot)
// plus SHARDS rename_shard slices wired through the cross-shard broadcast network.
// This is the in-context assembly (unlike the single-slice OOC probe) so the
// cross-shard wiring is real and shows up in timing.
//
// Unified geometry: AREGS=64 (int+FP), NPHYS=128, POOL=32/shard, NCHK=4.
//
// Wiring of the broadcasts (all combinational this cycle):
//   alloc[i]    = shard i's freshly allocated phys dest (pdst)
//   al_phys     = {alloc} broadcast  -> resolves SLOT(j) sources in every shard
//   wr_phys     = {alloc} broadcast  -> MAP write data
//   wr_arch     = {rd}    broadcast  -> MAP write address
//   wr_valid    = map_writer         -> only the last-writer-per-arch updates MAP
//   d_valid[i]  = rd_v[i]            -> shard i allocates for any register write
module renamer_bundle
  #(parameter SHARDS = 4,
    parameter ABITS  = 6,    // unified arch space 0..63
    parameter AREGS  = 64,
    parameter PBITS  = 7,    // 128 phys regs
    parameter NPHYS  = 128,
    parameter POOL   = 32,   // NPHYS/SHARDS
    parameter HPTR   = 5,    // clog2(POOL)
    parameter SBITS  = 2,    // clog2(SHARDS)
    parameter NCHK   = 4,
    parameter CBITS  = 2)
   (input  wire                   clk,
    input  wire [SHARDS*ABITS-1:0] rs1,
    input  wire [SHARDS-1:0]       rs1_v,
    input  wire [SHARDS*ABITS-1:0] rs2,
    input  wire [SHARDS-1:0]       rs2_v,
    input  wire [SHARDS*ABITS-1:0] rd,
    input  wire [SHARDS-1:0]       rd_v,
    input  wire [SHARDS*PBITS-1:0] fr_phys,
    input  wire [SHARDS-1:0]       fr_valid,
    input  wire                    chk_create,
    input  wire [CBITS-1:0]        chk_create_idx,
    input  wire                    chk_restore,
    input  wire [CBITS-1:0]        chk_restore_idx,
    output wire [SHARDS*PBITS-1:0] ps1,
    output wire [SHARDS*PBITS-1:0] ps2,
    output wire [SHARDS*PBITS-1:0] pdst,
    output wire [SHARDS-1:0]       stall);

   // --- cross-slot dependency matrix over the whole bundle
   wire [SHARDS-1:0]       s1_is_slot, s2_is_slot, map_writer;
   wire [SHARDS*SBITS-1:0] s1_slot, s2_slot;
   decode_xslot #(.IW(SHARDS), .ABITS(ABITS), .SBITS(SBITS)) xs
     (.rs1(rs1), .rs1_v(rs1_v), .rs2(rs2), .rs2_v(rs2_v), .rd(rd), .rd_v(rd_v),
      .s1_is_slot(s1_is_slot), .s1_slot(s1_slot),
      .s2_is_slot(s2_is_slot), .s2_slot(s2_slot), .map_writer(map_writer));

   // --- broadcast buses built from the shards' own allocations
   wire [SHARDS*PBITS-1:0] alloc;        // = pdst of each shard
   wire [SHARDS*ABITS-1:0] wr_arch  = rd;
   wire [SHARDS*PBITS-1:0] wr_phys  = alloc;
   wire [SHARDS-1:0]       wr_valid = map_writer;
   wire [SHARDS*PBITS-1:0] al_phys  = alloc;

   assign pdst = alloc;

   // --- one rename_shard per lane
   genvar i;
   generate
      for (i = 0; i < SHARDS; i = i + 1) begin : lane
         rename_shard #(.SHARDS(SHARDS), .SH(i), .AREGS(AREGS), .ABITS(ABITS),
                        .NPHYS(NPHYS), .PBITS(PBITS), .POOL(POOL), .HPTR(HPTR),
                        .SBITS(SBITS), .NCHK(NCHK), .CBITS(CBITS)) sh
           (.clk(clk),
            .s1_arch(rs1[i*ABITS +: ABITS]), .s1_is_slot(s1_is_slot[i]),
            .s1_slot(s1_slot[i*SBITS +: SBITS]),
            .s2_arch(rs2[i*ABITS +: ABITS]), .s2_is_slot(s2_is_slot[i]),
            .s2_slot(s2_slot[i*SBITS +: SBITS]),
            .d_arch(rd[i*ABITS +: ABITS]), .d_valid(rd_v[i]),
            .wr_arch(wr_arch), .wr_phys(wr_phys), .wr_valid(wr_valid),
            .al_phys(al_phys),
            .fr_phys(fr_phys[i*PBITS +: PBITS]), .fr_valid(fr_valid[i]),
            .chk_create(chk_create), .chk_create_idx(chk_create_idx),
            .chk_restore(chk_restore), .chk_restore_idx(chk_restore_idx),
            .ps1(ps1[i*PBITS +: PBITS]), .ps2(ps2[i*PBITS +: PBITS]),
            .pdst(alloc[i*PBITS +: PBITS]), .pold(),  .stall(stall[i]));
      end
   endgenerate
endmodule

`default_nettype wire
