`default_nettype none

// Generic superscalar register renamer with checkpoints.
//
// Each cycle accepts IW instructions, each carrying two architectural sources
// and one architectural destination; architectural register 0 means "absent"
// (no source / no write). A 32-entry MAP holds the architectural -> physical
// mapping. A ring-buffer free list supplies fresh physical registers for
// destinations (pop at head) and reclaims them at commit (push at tail).
//
// Intra-bundle dependencies are fully resolved in one cycle:
//   * RAW bypass  - a source reads the youngest *earlier* in-bundle writer of
//                   the same architectural register, else the MAP.
//   * WAW         - the MAP next-state applies slots in order, so the youngest
//                   writer of an arch reg wins.
//
// NCHK checkpoints each hold a full copy of {MAP, free list (array + head/tail/
// count)}. A create snapshots the current state into a selected bank (flop->flop,
// cheap). A restore overwrites MAP and the free list from a selected bank; it is
// a 2:1 mux at the state register inputs, parallel to the rename path, so it does
// not lengthen the rename critical path. Restore and rename are mutually
// exclusive in a cycle.
//
// Ports are flattened buses (Verilog-2001) so the module drops into the timing
// probe.  NPHYS and NCHK are assumed powers of two (free-list pointers wrap in
// their natural bit width).
module renamer
  #(parameter IW    = 4,     // issue width
    parameter AREGS = 32,    // architectural registers
    parameter ABITS = 5,     // clog2(AREGS)
    parameter NPHYS = 64,    // physical registers
    parameter PBITS = 6,     // clog2(NPHYS)
    parameter NCHK  = 4,     // checkpoints
    parameter CBITS = 2)     // clog2(NCHK)
   (input  wire                clk,
    // instruction bundle
    input  wire [IW*ABITS-1:0] src1,
    input  wire [IW*ABITS-1:0] src2,
    input  wire [IW*ABITS-1:0] dst,
    input  wire [IW-1:0]       valid,
    // commit / free port (returns physical registers to the free list)
    input  wire [IW*PBITS-1:0] free,
    input  wire [IW-1:0]       free_valid,
    // checkpoint control
    input  wire                chk_create,
    input  wire [CBITS-1:0]    chk_create_idx,
    input  wire                chk_restore,
    input  wire [CBITS-1:0]    chk_restore_idx,
    // renamed bundle
    output wire [IW*PBITS-1:0] psrc1,
    output wire [IW*PBITS-1:0] psrc2,
    output wire [IW*PBITS-1:0] pdst,   // newly allocated physical destination
    output wire [IW*PBITS-1:0] pold,   // prior phys mapping of dst (freed at commit)
    output wire                stall); // free list cannot supply this bundle

   localparam FPTR = (PBITS == 0) ? 1 : PBITS;        // free-list index width
   localparam CNTW = PBITS + 1;                       // free-count width (0..NPHYS)

   // ---------------------------------------------------------------- state
   reg [PBITS-1:0] map [0:AREGS-1];
   reg [PBITS-1:0] fl  [0:NPHYS-1];                   // ring buffer of free phys ids
   reg [FPTR-1:0]  head, tail;
   reg [CNTW-1:0]  count;

   reg [PBITS-1:0] chk_map   [0:NCHK-1][0:AREGS-1];
   reg [PBITS-1:0] chk_fl    [0:NCHK-1][0:NPHYS-1];
   reg [FPTR-1:0]  chk_head  [0:NCHK-1];
   reg [FPTR-1:0]  chk_tail  [0:NCHK-1];
   reg [CNTW-1:0]  chk_count [0:NCHK-1];

   integer b, r;
   initial begin
      for (r = 0; r < AREGS; r = r + 1) map[r] = r[PBITS-1:0];          // arch r -> phys r
      for (r = 0; r < NPHYS; r = r + 1) fl[r]  = (AREGS + r) % NPHYS;   // free = phys AREGS..NPHYS-1
      head  = 0;
      tail  = (NPHYS - AREGS);
      count = (NPHYS - AREGS);
      for (b = 0; b < NCHK; b = b + 1) begin
         for (r = 0; r < AREGS; r = r + 1) chk_map[b][r] = r[PBITS-1:0];
         for (r = 0; r < NPHYS; r = r + 1) chk_fl[b][r]  = (AREGS + r) % NPHYS;
         chk_head[b]  = 0;
         chk_tail[b]  = (NPHYS - AREGS);
         chk_count[b] = (NPHYS - AREGS);
      end
   end

   // -------------------------------------------------------------- unpack
   reg [ABITS-1:0] a1 [0:IW-1];
   reg [ABITS-1:0] a2 [0:IW-1];
   reg [ABITS-1:0] ad [0:IW-1];
   reg [PBITS-1:0] fr [0:IW-1];
   integer u;
   always @* for (u = 0; u < IW; u = u + 1) begin
      a1[u] = src1[u*ABITS +: ABITS];
      a2[u] = src2[u*ABITS +: ABITS];
      ad[u] = dst [u*ABITS +: ABITS];
      fr[u] = free[u*PBITS +: PBITS];
   end

   // -------------------------------------------------- destination allocation
   // alloc[i] = fl[head + (number of allocating slots before i)]
   reg              need  [0:IW-1];
   reg [PBITS-1:0]  alloc [0:IW-1];
   reg [CNTW-1:0]   nalloc;                 // total allocations this bundle
   integer i, j;
   always @* begin
      nalloc = 0;
      for (i = 0; i < IW; i = i + 1) begin
         need[i]  = valid[i] && (ad[i] != {ABITS{1'b0}});
         alloc[i] = fl[(head + nalloc[FPTR-1:0]) % NPHYS];
         if (need[i]) nalloc = nalloc + 1'b1;
      end
   end
   assign stall = (count < nalloc);

   // --------------------------------------- source rename with intra-bundle bypass
   reg [PBITS-1:0] ps1 [0:IW-1];
   reg [PBITS-1:0] ps2 [0:IW-1];
   reg [PBITS-1:0] pol [0:IW-1];
   always @* begin
      for (i = 0; i < IW; i = i + 1) begin
         // source 1
         ps1[i] = (a1[i] == {ABITS{1'b0}}) ? {PBITS{1'b0}} : map[a1[i]];
         for (j = 0; j < IW; j = j + 1)
            if ((j < i) && need[j] && (a1[i] != {ABITS{1'b0}}) && (ad[j] == a1[i]))
               ps1[i] = alloc[j];
         // source 2
         ps2[i] = (a2[i] == {ABITS{1'b0}}) ? {PBITS{1'b0}} : map[a2[i]];
         for (j = 0; j < IW; j = j + 1)
            if ((j < i) && need[j] && (a2[i] != {ABITS{1'b0}}) && (ad[j] == a2[i]))
               ps2[i] = alloc[j];
         // prior mapping of this destination (freed when this insn commits)
         pol[i] = map[ad[i]];
         for (j = 0; j < IW; j = j + 1)
            if ((j < i) && need[j] && (ad[j] == ad[i]))
               pol[i] = alloc[j];
      end
   end

   // ------------------------------------------------------ MAP next-state (WAW)
   reg [PBITS-1:0] nmap [0:AREGS-1];
   always @* begin
      for (r = 0; r < AREGS; r = r + 1) nmap[r] = map[r];
      for (i = 0; i < IW; i = i + 1)
         if (need[i]) nmap[ad[i]] = alloc[i];   // ascending order => youngest wins
   end

   // ----------------------------------------------- free-list push (commit/free)
   // freed regs are written at tail in slot order.
   reg [FPTR-1:0]  ftgt [0:IW-1];     // ring index each freed reg lands at
   reg [CNTW-1:0]  nfree;
   always @* begin
      nfree = 0;
      for (i = 0; i < IW; i = i + 1) begin
         ftgt[i] = (tail + nfree[FPTR-1:0]) % NPHYS;
         if (free_valid[i]) nfree = nfree + 1'b1;
      end
   end

   // ------------------------------------------------------------- sequential
   integer k;
   always @(posedge clk) begin
      if (chk_restore) begin
         for (k = 0; k < AREGS; k = k + 1) map[k] <= chk_map[chk_restore_idx][k];
         for (k = 0; k < NPHYS; k = k + 1) fl[k]  <= chk_fl [chk_restore_idx][k];
         head  <= chk_head [chk_restore_idx];
         tail  <= chk_tail [chk_restore_idx];
         count <= chk_count[chk_restore_idx];
      end else begin
         for (k = 0; k < AREGS; k = k + 1) map[k] <= nmap[k];
         // pop allocations at head, push frees at tail
         for (i = 0; i < IW; i = i + 1)
            if (free_valid[i]) fl[ftgt[i]] <= fr[i];
         head  <= (head + nalloc[FPTR-1:0]) % NPHYS;
         tail  <= (tail + nfree[FPTR-1:0])  % NPHYS;
         count <= count - nalloc + nfree;
      end

      // checkpoint create: snapshot the current (pre-update) architectural state
      if (chk_create) begin
         for (k = 0; k < AREGS; k = k + 1) chk_map[chk_create_idx][k] <= map[k];
         for (k = 0; k < NPHYS; k = k + 1) chk_fl [chk_create_idx][k] <= fl[k];
         chk_head [chk_create_idx] <= head;
         chk_tail [chk_create_idx] <= tail;
         chk_count[chk_create_idx] <= count;
      end
   end

   // --------------------------------------------------------------- repack out
   genvar g;
   generate
      for (g = 0; g < IW; g = g + 1) begin : pack
         assign psrc1[g*PBITS +: PBITS] = ps1[g];
         assign psrc2[g*PBITS +: PBITS] = ps2[g];
         assign pdst [g*PBITS +: PBITS] = alloc[g];
         assign pold [g*PBITS +: PBITS] = pol[g];
      end
   endgenerate
endmodule

`default_nettype wire
