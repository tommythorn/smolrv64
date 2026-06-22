`default_nettype none

// One shard of a sharded (clustered) superscalar renamer.  Geometry default:
// SHARDS=4, one instruction per shard per cycle (total width W=SHARDS).
//
// Each shard owns a disjoint pool of physical registers and renames only its own
// instruction's destination from its own single-pop free list.  It keeps a full
// *replicated* copy of the architectural MAP: reads are local (this shard's two
// sources), but writes are the whole bundle's destination updates broadcast in,
// so the MAP copy needs W true write ports.  The decoder guarantees at most one
// MAP-visible writer per architectural register per bundle (last-writer-wins),
// so the W write ports always target distinct addresses -> no write-priority mux.
//
// The decoder also pre-resolves intra-bundle source dependencies: a source is
// either ARCH(a) (read the MAP) or SLOT(j) (take slot j's freshly allocated
// physreg from the bundle's alloc broadcast).  That removes the source-vs-dest
// comparator network from the rename critical path -- it becomes a W:1 mux.
//
// Inter-shard signals (wr_* write broadcast, al_phys alloc broadcast) are inputs
// here; in the full machine they come from the sibling shards. Ports are
// flattened buses (Verilog-2001) so the slice drops into the timing probe.
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
    // this shard's decoded instruction
    input  wire [ABITS-1:0]      s1_arch,
    input  wire                  s1_is_slot,
    input  wire [SBITS-1:0]      s1_slot,
    input  wire [ABITS-1:0]      s2_arch,
    input  wire                  s2_is_slot,
    input  wire [SBITS-1:0]      s2_slot,
    input  wire [ABITS-1:0]      d_arch,
    input  wire                  d_valid,        // allocates a destination
    // bundle write broadcast -> updates the replicated MAP (W distinct-addr ports)
    input  wire [SHARDS*ABITS-1:0] wr_arch,
    input  wire [SHARDS*PBITS-1:0] wr_phys,
    input  wire [SHARDS-1:0]       wr_valid,
    // bundle alloc broadcast -> resolves SLOT(j) sources
    input  wire [SHARDS*PBITS-1:0] al_phys,
    // commit/free port (single push into this shard's free list)
    input  wire [PBITS-1:0]      fr_phys,
    input  wire                  fr_valid,
    // checkpoint control
    input  wire                  chk_create,
    input  wire [CBITS-1:0]      chk_create_idx,
    input  wire                  chk_restore,
    input  wire [CBITS-1:0]      chk_restore_idx,
    // renamed outputs
    output wire [PBITS-1:0]      ps1,
    output wire [PBITS-1:0]      ps2,
    output wire [PBITS-1:0]      pdst,           // this shard's allocated dest
    output wire [PBITS-1:0]      pold,           // prior mapping of d_arch (freed at commit)
    output wire                  stall);

   localparam CNTW = HPTR + 1;
   localparam ARSH = AREGS / SHARDS;   // arch regs owned per shard = reserved freelist head

   // ---------------------------------------------------------------- state
   reg [PBITS-1:0] map [0:AREGS-1];        // replicated architectural map
   reg [PBITS-1:0] fl  [0:POOL-1];         // this shard's free-list ring
   reg [HPTR-1:0]  head, tail;
   reg [CNTW-1:0]  count;

   reg [PBITS-1:0] chk_map   [0:NCHK-1][0:AREGS-1];
   reg [PBITS-1:0] chk_fl    [0:NCHK-1][0:POOL-1];
   reg [HPTR-1:0]  chk_head  [0:NCHK-1];
   reg [HPTR-1:0]  chk_tail  [0:NCHK-1];
   reg [CNTW-1:0]  chk_count [0:NCHK-1];

   // Init: arch reg a -> phys a (map[a]=a), so phys 0..AREGS-1 are the live initial
   // mappings and must NOT be free. This shard's owned pool fl[r]=SH+SHARDS*r has its
   // first ARSH entries (fl[0..ARSH-1]) equal to exactly those arch-mapped phys regs,
   // so the free regs are fl[ARSH..POOL-1]: start head past the reserved head. This
   // also reserves phys 0 as x0 (it's arch 0, never allocated, rf reads it as 0).
   integer b, r;
   initial begin
      for (r = 0; r < AREGS; r = r + 1) map[r] = r[PBITS-1:0];
      for (r = 0; r < POOL;  r = r + 1) fl[r]  = (SH + SHARDS*r) % NPHYS;  // this shard's pool
      head = ARSH[HPTR-1:0]; tail = 0; count = POOL - ARSH;  // free = fl[ARSH..POOL-1]
      for (b = 0; b < NCHK; b = b + 1) begin
         for (r = 0; r < AREGS; r = r + 1) chk_map[b][r] = r[PBITS-1:0];
         for (r = 0; r < POOL;  r = r + 1) chk_fl[b][r]  = (SH + SHARDS*r) % NPHYS;
         chk_head[b] = ARSH[HPTR-1:0]; chk_tail[b] = 0; chk_count[b] = POOL - ARSH;
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
   wire           alloc_en = d_valid && (count != 0);
   wire [PBITS-1:0] alloc  = fl[head];          // single-pop free list
   assign stall  = d_valid && (count == 0);
   assign pdst   = alloc;

   // --------------------------------------- source rename (decoder-resolved)
   // ARCH -> MAP read; SLOT(j) -> bundle alloc j; arch 0 -> phys 0.
   assign ps1 = s1_is_slot ? ap[s1_slot]
              : (s1_arch == {ABITS{1'b0}}) ? {PBITS{1'b0}} : map[s1_arch];
   assign ps2 = s2_is_slot ? ap[s2_slot]
              : (s2_arch == {ABITS{1'b0}}) ? {PBITS{1'b0}} : map[s2_arch];
   assign pold = map[d_arch];                   // prior producer, reclaimed at commit

   // ----------------------------------- MAP next-state: W distinct-addr writes
   reg [PBITS-1:0] nmap [0:AREGS-1];
   integer p;
   always @* begin
      for (r = 0; r < AREGS; r = r + 1) nmap[r] = map[r];
      for (p = 0; p < SHARDS; p = p + 1)
         if (wr_valid[p]) nmap[wa[p]] = wp[p];  // addresses distinct => no priority
   end

   // ------------------------------------------------------------- sequential
   integer k;
   always @(posedge clk) begin
      if (chk_restore) begin
         for (k = 0; k < AREGS; k = k + 1) map[k] <= chk_map[chk_restore_idx][k];
         for (k = 0; k < POOL;  k = k + 1) fl[k]  <= chk_fl [chk_restore_idx][k];
         head  <= chk_head [chk_restore_idx];
         tail  <= chk_tail [chk_restore_idx];
         count <= chk_count[chk_restore_idx];
      end else begin
         for (k = 0; k < AREGS; k = k + 1) map[k] <= nmap[k];
         if (fr_valid) fl[tail] <= fr_phys;      // push freed reg
         if (alloc_en) head <= head + 1'b1;      // pop one
         if (fr_valid) tail <= tail + 1'b1;
         count <= count - (alloc_en ? 1'b1 : 1'b0) + (fr_valid ? 1'b1 : 1'b0);
      end

      // checkpoint create: snapshot current architectural state
      if (chk_create) begin
         for (k = 0; k < AREGS; k = k + 1) chk_map[chk_create_idx][k] <= map[k];
         for (k = 0; k < POOL;  k = k + 1) chk_fl [chk_create_idx][k] <= fl[k];
         chk_head [chk_create_idx] <= head;
         chk_tail [chk_create_idx] <= tail;
         chk_count[chk_create_idx] <= count;
      end
   end
endmodule

`default_nettype wire
