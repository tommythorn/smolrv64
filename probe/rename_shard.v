`default_nettype none

// One shard of a sharded (clustered) superscalar renamer.  Geometry default:
// SHARDS=4, one instruction per shard per cycle (total width W=SHARDS).
//
// Each shard owns a disjoint pool of physical registers and renames only its own
// instruction's destination from its own freelist.  It keeps a full *replicated*
// copy of the architectural MAP: reads are local (this shard's two sources), but
// writes are the whole bundle's destination updates broadcast in, so the MAP copy
// needs W true write ports.  The decoder guarantees at most one MAP-visible writer
// per architectural register per bundle (last-writer-wins), so the W write ports
// always target distinct addresses -> no write-priority mux.
//
// The decoder also pre-resolves intra-bundle dependencies (decode_xslot):
//   * a source is ARCH(a) (read the MAP) or SLOT(j) (take slot j's freshly
//     allocated physreg from the alloc broadcast) -- RAW bypass;
//   * a destination's displaced prior mapping (pold) is map[d] unless an earlier
//     slot j in the bundle also writes d, in which case it is slot j's pdst
//     (d_is_slot/d_slot) -- intra-bundle WAW. So every allocating instruction
//     displaces exactly one register, and reclamation balances.
// Both removials keep the comparator network out of the rename critical path.
//
// Free-register management is the bitmap freelist.v (CPR): per-checkpoint
// allocated/displaced sets, bulk commit/rollback reclamation. This shard keeps
// only the MAP and its per-checkpoint snapshot (chk_map) for rollback; the
// freelist owns the open span `cur`. Displaced polds are routed to their *owner*
// shard's freelist via the bundle's pold broadcast (pold_valid/pold_pr inputs),
// exactly like al_phys for sources.
module rename_shard
  #(parameter SHARDS = 4,                 // bundle width W
    parameter SH     = 0,                  // this shard's id
    parameter AREGS  = 32,
    parameter ABITS  = 5,                  // clog2(AREGS)
    parameter NPHYS  = 64,                 // total physical registers
    parameter PBITS  = 6,                  // clog2(NPHYS)
    parameter POOL   = 16,                 // this shard's pool = NPHYS/SHARDS
    parameter HPTR   = 4,                  // clog2(POOL)
    parameter SBITS  = 2,                  // clog2(SHARDS) -- slot-id width
    parameter NCHK   = 4,
    parameter CBITS  = 2)
   (input  wire                  clk,
    input  wire                  reset,
    // this shard's decoded instruction
    input  wire [ABITS-1:0]      s1_arch,
    input  wire                  s1_is_slot,
    input  wire [SBITS-1:0]      s1_slot,
    input  wire [ABITS-1:0]      s2_arch,
    input  wire                  s2_is_slot,
    input  wire [SBITS-1:0]      s2_slot,
    input  wire [ABITS-1:0]      d_arch,
    input  wire                  d_valid,        // allocates a destination
    input  wire                  d_is_slot,      // pold comes from an earlier slot
    input  wire [SBITS-1:0]      d_slot,         // ... which slot (its pdst)
    // bundle write broadcast -> updates the replicated MAP (W distinct-addr ports)
    input  wire [SHARDS*ABITS-1:0] wr_arch,
    input  wire [SHARDS*PBITS-1:0] wr_phys,
    input  wire [SHARDS-1:0]       wr_valid,
    // bundle alloc broadcast -> resolves SLOT(j) sources and d_slot pold
    input  wire [SHARDS*PBITS-1:0] al_phys,
    // bundle pold broadcast -> this shard's freelist records the ones it owns
    input  wire [SHARDS-1:0]       pold_valid,
    input  wire [SHARDS*PBITS-1:0] pold_bus,
    // checkpoint / commit control (driven in lockstep across all shards)
    input  wire                  create,         // open a new span (cur++); snapshot MAP[cur]
    input  wire                  commit,
    input  wire [CBITS-1:0]      commit_idx,
    input  wire                  rollback,        // branch redirect / exception
    input  wire [CBITS-1:0]      rollback_idx,
    // renamed outputs
    output wire [PBITS-1:0]      ps1,
    output wire [PBITS-1:0]      ps2,
    output wire [PBITS-1:0]      pdst,           // this shard's allocated dest
    output wire [PBITS-1:0]      pold,           // prior mapping of d_arch (freed at commit)
    output wire                  pold_v,         // d_arch was actually displaced
    output wire [CBITS-1:0]      cur,            // freelist's open span (for tagging)
    output wire                  stall);

   localparam ARSH = AREGS / SHARDS;   // arch regs owned per shard = reserved freelist head

   // ---------------------------------------------------------------- MAP state
   reg [PBITS-1:0] map     [0:AREGS-1];               // replicated architectural map
   reg [PBITS-1:0] chk_map [0:NCHK-1][0:AREGS-1];     // per-checkpoint snapshot (rollback)

   // Init: arch reg a -> phys a (map[a]=a), so phys 0..AREGS-1 are the live initial
   // mappings (reserved in the freelist; phys 0 is x0). Once displaced + committed
   // they rejoin the pool.
   integer b, r;
   initial begin
      for (r = 0; r < AREGS; r = r + 1) map[r] = r[PBITS-1:0];
      for (b = 0; b < NCHK; b = b + 1)
         for (r = 0; r < AREGS; r = r + 1) chk_map[b][r] = r[PBITS-1:0];
   end

   // ------------------------------------------------------------- unpack buses
   reg [ABITS-1:0] wa [0:SHARDS-1];
   reg [PBITS-1:0] wp [0:SHARDS-1];
   reg [PBITS-1:0] ap [0:SHARDS-1];
   integer u;
   always @* for (u = 0; u < SHARDS; u = u + 1) begin
      wa[u] = wr_arch[u*ABITS +: ABITS];
      wp[u] = wr_phys[u*PBITS +: PBITS];
      ap[u] = al_phys[u*PBITS +: PBITS];
   end

   // -------------------------------------------------- destination allocation
   // A register is allocated only when this bundle actually dispatches (create);
   // a held (back-pressured) bundle keeps its combinational pdst but pops nothing.
   wire             alloc_ok;
   wire [PBITS-1:0] alloc_pr;
   freelist #(.SHARDS(SHARDS), .SH(SH), .SBITS(SBITS), .PBITS(PBITS), .POOL(POOL),
              .LPOOL(HPTR), .NCHK(NCHK), .CBITS(CBITS), .ARSH(ARSH)) fl
     (.clk(clk), .reset(reset),
      .alloc_en(d_valid & create), .alloc_pr(alloc_pr), .alloc_ok(alloc_ok), .free_count(),
      .pold_valid(pold_valid), .pold_pr(pold_bus),
      .create(create), .commit(commit), .commit_idx(commit_idx),
      .rollback(rollback), .rollback_idx(rollback_idx), .cur(cur));

   assign stall = d_valid && !alloc_ok;
   assign pdst  = alloc_pr;

   // --------------------------------------- source rename (decoder-resolved)
   // ARCH -> MAP read; SLOT(j) -> bundle alloc j; arch 0 -> phys 0.
   assign ps1 = s1_is_slot ? ap[s1_slot]
              : (s1_arch == {ABITS{1'b0}}) ? {PBITS{1'b0}} : map[s1_arch];
   assign ps2 = s2_is_slot ? ap[s2_slot]
              : (s2_arch == {ABITS{1'b0}}) ? {PBITS{1'b0}} : map[s2_arch];
   // displaced prior mapping: an earlier in-bundle writer's pdst, else the MAP.
   assign pold   = d_is_slot ? ap[d_slot] : map[d_arch];
   // Gate by `create` (actual dispatch), exactly like alloc_en: a displaced pold is
   // recorded into the freelist's P[cur] only when the displacing instruction really
   // dispatches. Without this, a bundle stalled at the rename boundary (back-pressure /
   // serialize gating) re-broadcasts its pold every cycle, and across a rollback the
   // same physreg can land in multiple spans' P[] -> freed more than once -> freed while
   // live -> double allocation. (Hand testbenches never stalled, so this stayed latent.)
   assign pold_v = d_valid & create;            // every dispatched alloc displaces exactly one

   // ----------------------------------- MAP next-state: W distinct-addr writes
   reg [PBITS-1:0] nmap [0:AREGS-1];
   integer p;
   always @* begin
      for (r = 0; r < AREGS; r = r + 1) nmap[r] = map[r];
      for (p = 0; p < SHARDS; p = p + 1)
         if (wr_valid[p]) nmap[wa[p]] = wp[p];  // addresses distinct => no priority
   end

   // ------------------------------------------------------------- sequential
   // chk_map[S] holds the MAP to restore when span S is *reopened* = the map state
   // just before span S's bundle = the post-bundle map of span S-1. So at each
   // create (closing span cur, opening cur+1) we snapshot the *post*-bundle map
   // (nmap) into the next span's slot. A branch in span C reopens span C+1, so its
   // snapshot is written at C's own create -- it never depends on a later bundle
   // having renamed. chk_map[0] starts at the identity map (reset state).
   wire [CBITS-1:0] nxt = cur + 1'b1;
   integer k;
   always @(posedge clk) begin
      // MAP changes only on a dispatch (create) or a rollback; a held or empty
      // cycle leaves it untouched.
      if (rollback) begin
         for (k = 0; k < AREGS; k = k + 1) map[k] <= chk_map[rollback_idx][k];
      end else if (create) begin
         for (k = 0; k < AREGS; k = k + 1) begin
            map[k]        <= nmap[k];
            chk_map[nxt][k] <= nmap[k];   // post-bundle map -> reopened-span slot
         end
      end
   end
endmodule

`default_nettype wire
