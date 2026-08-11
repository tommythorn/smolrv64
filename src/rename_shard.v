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
    input  wire [ABITS-1:0]      s3_arch,
    input  wire                  s3_is_slot,
    input  wire [SBITS-1:0]      s3_slot,
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
    input  wire                  create,         // per-BUNDLE dispatch: alloc / MAP update / pold
    input  wire                  ckpt_create,    // per-CHECKPOINT close: span cur++ + chk_map snapshot
    input  wire                  commit,
    input  wire [CBITS-1:0]      commit_idx,
    input  wire                  rollback,        // branch redirect / exception
    input  wire [CBITS-1:0]      rollback_idx,
    // renamed outputs
    output wire [PBITS-1:0]      ps1,
    output wire [PBITS-1:0]      ps2,
    output wire [PBITS-1:0]      ps3,
    output wire [PBITS-1:0]      pdst,           // this shard's allocated dest
    output wire [PBITS-1:0]      pold,           // prior mapping of d_arch (freed at commit)
    output wire                  pold_v,         // d_arch was actually displaced
    output wire [CBITS-1:0]      cur,            // freelist's open span (for tagging)
    output wire                  stall);

   // Arch regs owned per shard = the reserved freelist head. CEILING, not truncation:
   // arch reg a lives at index a/SHARDS, so the largest index in use is (AREGS-1)/SHARDS
   // and the reservation must cover it. Plain AREGS/SHARDS is right only when SHARDS
   // divides AREGS -- at SHARDS=3 (AREGS=64) it reserved 21 while arch r63 maps to index
   // 21, so freelist idx 21 started FREE while it was already r63's live mapping and the
   // first allocation handed out a register the arch map still pointed at. Every IW=3 test
   // failed at 25ns. Same shape as 61f7d0a; the non-power-of-2 widths are the ones that
   // expose it (3 and 5), which is why the power-of-2 sweep points never showed it.
   localparam ARSH = (AREGS + SHARDS - 1) / SHARDS;

   // ---------------------------------------------------------------- MAP state
   reg [PBITS-1:0] map     [0:AREGS-1];               // replicated architectural map
   reg [PBITS-1:0] chk_map [0:NCHK-1][0:AREGS-1];     // per-checkpoint snapshot (rollback)

   // Init: arch reg a -> its HOME physreg {shard=a%SHARDS, ridx=a/SHARDS} = the reserved
   // freelist head, i.e. (a%SHARDS) + ((a/SHARDS)<<SBITS). This equals a exactly when
   // SHARDS==2^SBITS (power-of-2 IW); for non-power-of-2 IW (1,3,5) the shard field is
   // wider than SHARDS, so a bare `map[a]=a` would place odd arch regs in a NON-EXISTENT
   // shard and alias live physregs (observed: IW=1 mul read a stale result). phys 0 is x0.
   integer b, r; reg [31:0] home;
   initial begin
      for (r = 0; r < AREGS; r = r + 1) begin
         home = (r % SHARDS) + ((r / SHARDS) << SBITS);   // 32-bit integer arithmetic ...
         map[r] = home[PBITS-1:0];                        // ... sliced to the physreg width
         for (b = 0; b < NCHK; b = b + 1) chk_map[b][r] = home[PBITS-1:0];
      end
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
      .create(ckpt_create), .commit(commit), .commit_idx(commit_idx),
      .rollback(rollback), .rollback_idx(rollback_idx), .cur(cur));

   assign stall = d_valid && !alloc_ok;
   assign pdst  = alloc_pr;

   // --------------------------------------- source rename (decoder-resolved)
   // ARCH -> MAP read; SLOT(j) -> bundle alloc j; arch 0 -> phys 0.
   assign ps1 = s1_is_slot ? ap[s1_slot]
              : (s1_arch == {ABITS{1'b0}}) ? {PBITS{1'b0}} : map[s1_arch];
   assign ps2 = s2_is_slot ? ap[s2_slot]
              : (s2_arch == {ABITS{1'b0}}) ? {PBITS{1'b0}} : map[s2_arch];
   assign ps3 = s3_is_slot ? ap[s3_slot]
              : (s3_arch == {ABITS{1'b0}}) ? {PBITS{1'b0}} : map[s3_arch];
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
      end else begin
         // MAP advances on every dispatched BUNDLE (create=disp_fire); the chk_map
         // snapshot is taken only when a CHECKPOINT closes (ckpt_create), capturing the
         // post-close map (= the reopened span's start) into the next span's slot.
         if (create)      for (k = 0; k < AREGS; k = k + 1) map[k]          <= nmap[k];
         // chk_map[nxt] = the reopened span's START map = the map AS OF this cycle's end.
         // Normally a checkpoint closes ON a dispatched bundle (create), so that is nmap. But a
         // FORCE-CLOSE (irq/barrier/solo: ckpt_create with disp_fire/create=0) closes with NO
         // dispatch -- the stalled op dispatches INTO nxt next cycle, so its rename writes (nmap)
         // must NOT be captured; nxt starts at the current map. Use the same value `map` takes.
         if (ckpt_create) for (k = 0; k < AREGS; k = k + 1) chk_map[nxt][k] <= create ? nmap[k] : map[k];
      end
   end

`ifdef MAPDBG
   // One-shot divergence hunt (shard 0, arch reg `MAPDBG_AR): full history of the
   // tracked register's map -- renames, snapshot writes, rollback restores -- in a
   // cycle window. The restore that installs a stale physreg names the bad snapshot.
   integer mdc; initial mdc = 0;
   always @(posedge clk) begin
      mdc <= mdc + 1;
      if (SH == 0 && mdc > `MAPDBG_T0 && mdc < `MAPDBG_T1) begin
         if (rollback)
            $display("[MAPD] c=%0d ROLL idx=%0d map%0d<=%0d", mdc, rollback_idx,
                     `MAPDBG_AR, chk_map[rollback_idx][`MAPDBG_AR]);
         else begin
            if (create && (nmap[`MAPDBG_AR] != map[`MAPDBG_AR]))
               $display("[MAPD] c=%0d REN map%0d %0d->%0d (cur=%0d)", mdc,
                        `MAPDBG_AR, map[`MAPDBG_AR], nmap[`MAPDBG_AR], cur);
            if (ckpt_create)
               $display("[MAPD] c=%0d SNAP[%0d]%0d<=%0d (create=%b cur=%0d)", mdc, nxt,
                        `MAPDBG_AR, create ? nmap[`MAPDBG_AR] : map[`MAPDBG_AR], create, cur);
         end
      end
   end
`endif
endmodule

`default_nettype wire
