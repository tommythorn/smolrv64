`default_nettype none
// rv_dcache: the data cache (docs/PLAN-2026-10-03-dcache-vipt-l2.md), an alias-free VIPT L1,
// non-blocking, write-back, on the tagged memory port (rv_mem_arbiter's client side).
//
// GEOMETRY. SIZE_KB (128) in WAYS ways of 4 KiB (32), 64 sets of 64-byte lines. The set is
// PA[11:6], inside the page offset, so it is the VA's too: no synonyms, no colours, nothing to
// stamp. The core hands every request its PA (translation where it is), and a hit is the PA's tag
// PA[PABITS-1:12] matching one valid way of the set.
//
// THE DATA is WAYS arrays of 512 x 64 bits (a RAMB36 each, simple dual port, a write enable per
// byte), laid out DIAGONALLY in groups of eight: chunk c (PA[5:3]) of way w lives at row {set, c}
// of array {w[4:3], w[2:0] + c} -- its group's eight arrays, rotated by the way. So
//   - a load reads row {set, c} of every array and finds way w's chunk in array {w[4:3], w[2:0] + c}:
//     the hit rotated by c within each group, never the 4096 bits of data;
//   - a line's eight chunks sit in its group's eight arrays, so a fill beat (two chunks) writes two
//     arrays at once, one of each parity, and a dirty victim is read out whole in one cycle (its
//     group, then a rotation);
//   - a store writes its bytes into one array, without reading the word.
// TAGS are per way, distributed RAM read at the set: PA[PABITS-1:12] (written by an install only),
// and a second copy for the store lookup. VALID, DIRTY and the not-recently-used bit are a 64 x 1
// distributed RAM per way and field, each written by one statement.
//
// THE STORE PORT is a queue of SD committed stores (the store FIFO). A store is taken whenever the
// queue has room -- wr_room, a register, says so for this cycle, and a store presented then is
// taken then -- so a requester streams one store a cycle with no handshake back. Only the queue's
// head resolves, in order, by one of two lookups:
//   - THE STORE LOOKUP takes a plain store (cached, its bytes in one chunk): the tag copy at its
//     set names the way that holds the line, and the store writes its bytes into that way's array
//     and marks the line dirty, beside whatever the main lookup does. It waits a cycle when the
//     main lookup has anything in the same set but a load that hits (a miss may evict its way, a
//     drop or clean of the line resolves first), when a fill beat writes the array it needs, or
//     when a line installs; a store that has waited FP_WAIT cycles shuts the load door until it
//     writes. Nothing the main lookup does depends on it. A store to a line an
//     MSHR is filling writes its bytes into that MSHR's merge buffer instead, or waits for the
//     line to install once a beat has landed. A store to a line neither cached nor filling
//     goes to the main lookup, as does everything else the queue holds.
//   - THE MAIN LOOKUP takes one request a cycle: a released waiter (replay), else a load, else a
//     queued store-port op -- the store ahead of a load when it is urgent: the queue is full, its
//     head is a CBO, an NC store or an outside request (a requester waits on each), or the head has
//     waited ST_WAIT cycles.
// A CBO or an NC store is taken once (it is held until its wr_cpl, so a second presentation while
// one is queued is the same one). An OUTSIDE REQUEST is a second, lower-priority requester of the
// queue: a line cleaned or flushed by PA (ox_req, held until ox_ack; ox_keep: clean), a CBO that
// answers ox_ack instead of wr_cpl. The I$ asks for a clean before it reads a line from memory, so it
// never fills code the D$ still holds dirty; a DMA master asks for a clean before it reads a line and
// a flush before it writes one. The request queues behind every store taken before it, so memory
// holds their bytes when ox_ack says so. An outside flush's completion BLOCKS its line (xb_*): no
// fill of it starts -- a load, store or zero waits, as on a line being written back -- until ox_rel
// says the DMA master's write is in the controller, so a fill that follows reads the master's
// bytes. A fill already started is the flush's to wait for (it drops what lands), never blocked.
// The block compares the line's low 12 bits only: a miss on another line that matches them waits
// a few cycles too, which costs nothing, and the compare is half as deep.
//
// THE MAIN LOOKUP. A request taken at T reads row {set, c} of every array; at T+1 the set's valid
// bits and tags are compared with its PA.
//   - a hit: a load answers -- rd_valid at T+2, the addressed bytes at bit 0, tagged with rd_tag;
//   - no copy: the MSHR filling the line takes it -- one allocated now, with a reserved way whose
//     line is dropped (a dirty one read out to the write-back buffer) and a read queued on the
//     memory port under the MSHR's id. A load waits on the MSHR, as does a cbo.zero (its line is
//     written when the fill is in); a store that allocates leaves its bytes in the new MSHR's
//     merge buffer and is done. A plain store comes here only when the store lookup found its line
//     neither cached nor filling, and no load to its line gets past this lookup while it is queued
//     (below), so here it never hits or joins (asserted);
//   - a load to the line of a queued store (taken, not yet written or merged), or of the store
//     written at the last edge (the row read collided with the write), waits for it;
//   - no copy and no MSHR can take it (MSHRs full, the line still in the write-back buffer, no
//     write-back entry for a dirty victim): the request waits on the next MSHR or write-back entry
//     to free.
// Waiters live in a table with one entry per requester tag and one for the store; a miss never
// holds the pipeline. The store's completion counts as a freeing; the waiters on any freeing
// replay round robin. A structural conflict (the read-out busy, a row read as a store wrote it)
// replays the request at once.
// THE VICTIM is a way no MSHR reserves: an invalid one, else one whose not-recently-used bit is
// clear, else the first (and the set's bits clear). A hit and an install set the bit; a hit that
// would leave every bit of the set set clears the others.
//
// THE FILL. The beats write the reserved way's arrays as they arrive (a cycle after each, through
// the beat register), the merge buffer's bytes over the memory's. The lookup yields in the cycle
// the last one is written, and in the next the line installs -- the tags, valid, dirty if a store
// merged -- the MSHR is freed and its waiters are released; they replay ahead of the waiters on any
// resource, and hit.
//
// THE WRITE-BACK BUFFER (NWB lines). A dirty line leaving is read out in the next cycle, the lookup
// yielding the arrays' read ports. Its write goes to memory under slot NMSHR + entry, only when no
// read waits for the port, and the entry frees at the write's completion.
//
// NC AND CBOs. An NC access drops any copy (a dirty one written back first) and then goes to
// memory on its own through the NC slot, one at a time, a store with only its bytes; nothing
// allocates. cbo.clean writes a dirty copy back and keeps it, clean; cbo.flush and cbo.inval drop
// the copy, a dirty one written back; cbo.zero drops it unwritten and fills an MSHR whose merge
// buffer is the whole line of zeros. A CBO or an NC store completes (wr_cpl) when memory has its
// write, or at once when there was nothing to write. A line still in the write-back buffer, or
// being filled, holds any of them off until it is done.
//
// INV_REQ (fence.i) cleans the whole cache. With the door shut, it waits until no dirty bytes sit
// outside the arrays (the store queue, a merge buffer), then a walk of the 64 sets, each read across
// every way at once, writes every dirty line back and keeps it; inv_busy holds until memory has
// them all. The requester presents no store from inv_req until inv_busy falls (asserted): one
// taken meanwhile would be left out of the clean. After reset a scan clears every way's state, a
// set a cycle, with the door shut.
//
// RESPONSES are matched by the requester's tag (rule B1) and may complete out of order: a hit
// behind a miss answers first. Every taken load is answered exactly once.
//
// A load is taken at rd_ack and answered once, by tag; a store is taken in a cycle wr_room is set
// (wr_acc says so a cycle later), and a CBO or an NC store completes at wr_cpl.
module rv_dcache #(
   parameter SIZE_KB = 128,
   parameter RTW     = 4,              // the requester's opaque tag (the LQ index, the slow tag, the walkers)
   parameter NRQ     = 1 << RTW,       // the tags the requesters use: 0..NRQ-1, one waiter entry each
   parameter PABITS  = 36,             // significant physical address bits
   parameter NMSHR   = 8,
   parameter NWB     = 2,              // write-back buffer lines
   parameter SW      = 4               // memory-port slot bits (this client's ids)
) (
   input  wire              clk,
   input  wire              reset,
   // ---- loads
   input  wire              rd_req,
   input  wire [63:0]       rd_pa,
   input  wire [RTW-1:0]    rd_tag,
   input  wire              rd_nc,       // Svpbmt NC/IO: no allocation
   output wire              rd_ack,      // taken this cycle
   output reg               rd_valid,
   output reg  [63:0]       rd_data,     // the addressed doubleword as stored (a read names one: below)
   output reg  [RTW-1:0]    rd_resp_tag,
   output reg  [63:0]       rd_resp_addr,// the PA the response answers
   // the tags next cycle's answer can carry, from registers -- the lookup's (a) and the NC slot's
   // (b) -- and, with the answer, which it was: the LSU looks up a landing's state a cycle ahead
   output wire [RTW-1:0]    rd_pre_a,
   output wire [RTW-1:0]    rd_pre_b,
   output reg               rd_src_b,
   // ---- M's load in M's own cycle: taken whenever presented, so it never waits at a door. The
   // index comes from a register, the page number from the dTLB this cycle; an older store that
   // blocks the load voids its lookup a cycle later.
   input  wire              rd_quiet,    // registered: no rd_req can be presented this cycle
   input  wire              f_hint,      // registered: M holds a load that may be presented now
   input  wire              f_req,       // presented now, only while f_ok
   input  wire [63:0]       f_pa,
   input  wire [RTW-1:0]    f_tag,
   output wire              f_ok,        // a load presented now is taken (registers only)
   input  wire              f_cancel,    // the lookup's fast load is void (its T+1)
   output reg               rd_f,        // rd_valid answers the fast load: the core's speculative wake's verdict
   // ---- committed stores: the data sits at its byte lanes of the aligned 8-byte chunk
   input  wire              wr_req,
   input  wire [63:0]       wr_pa,
   input  wire [63:0]       wr_data,
   input  wire [7:0]        wr_mask,
   input  wire              wr_nc,       // Svpbmt NC/IO store
   input  wire              cbo_req,     // a CBO by PA: cbo_zero, else cbo_keep (clean), else flush/inval
   input  wire              cbo_zero,
   input  wire              cbo_keep,
   output reg               wr_room,     // a store presented this cycle is taken
   output reg               wr_acc,      // taken last cycle: a plain store completes on its own
   output reg               wr_cpl,      // an NC store's or a CBO's completion
   // ---- an outside request: the I$'s clean before it fills a line, a DMA master's before it moves one
   input  wire              ox_req,      // held until ox_ack
   input  wire [63:0]       ox_pa,
   input  wire              ox_keep,     // a clean; else a flush (the line leaves the D$)
   output reg               ox_ack,      // memory holds the line's bytes (and the D$ no copy, after a flush)
   output reg               ox_dty,      // ...and the first look did not finish it: memory changed since the ask
   input  wire              ox_rel,      // a pulse: the flushed line's DMA write is in the controller
   input  wire              inv_req,     // clean every dirty line (fence.i)
   output wire              inv_busy,
   // ---- the memory port (rv_mem_arbiter client)
   output wire              cq_valid,
   input  wire              cq_ready,
   output wire [SW-1:0]     cq_slot,
   output wire              cq_we,
   output wire [57:0]       cq_addr,
   output wire [63:0]       cq_wmask,
   output wire [511:0]      cq_wdata,
   input  wire              cr_valid,
   input  wire [SW-1:0]     cr_slot,
   input  wire [1:0]        cr_beat,
   input  wire              cr_last,
   input  wire [127:0]      cr_data,
   input  wire              cw_valid,
   input  wire [SW-1:0]     cw_slot,
   output wire              perf_access,
   output wire              perf_miss,
   output wire [15:0]       err
);
   localparam WAYS  = SIZE_KB / 4;                // a way is a page
   localparam WYB   = $clog2(WAYS);
   localparam PTB   = PABITS - 12;                // the tag: PA[PABITS-1:12]
   localparam LB    = PABITS - 6;                 // significant line-address bits
   localparam MB    = $clog2(NMSHR);
   localparam WBB   = (NWB > 1) ? $clog2(NWB) : 1;
   localparam NW    = NRQ + 1;                    // waiter entries: the tags, then the store
   localparam TB    = $clog2(NW);                 // a request's waiter entry: its tag, or ST
   localparam [TB-1:0] ST = NRQ[TB-1:0];          // the store's waiter entry (only the queue's head waits)
   function automatic [TB-1:0] wi(input [RTW-1:0] t);    // a requester's entry: its tag (< NRQ, asserted)
      integer b;
      begin wi = {TB{1'b0}}; for (b = 0; b < RTW && b < TB; b = b + 1) wi[b] = t[b]; end
   endfunction
   function automatic [RTW-1:0] tg(input [TB-1:0] w);    // ...and the tag of a requester's entry
      integer b;
      begin tg = {RTW{1'b0}}; for (b = 0; b < RTW && b < TB; b = b + 1) tg[b] = w[b]; end
   endfunction
   localparam SD    = 3;                          // store-queue entries
   localparam [2:0] ST_WAIT = 3'd3;               // a queued store goes ahead of loads after this long
   localparam [2:0] FP_WAIT = 3'd3;               // the store lookup shuts the load door after this long
   localparam [2:0] OX_WAIT = 3'd4;               // an outside request refused this long takes the store port's room
   initial begin
      if (WAYS < 8 || (WAYS & (WAYS - 1)) != 0) $fatal(1, "rv_dcache: %0d ways: a power of two, at least eight (a line's chunks in distinct arrays)", WAYS);
      if (NMSHR + NWB > (1 << SW)) $fatal(1, "rv_dcache: %0d MSHRs and %0d write-backs need more than %0d slot bits", NMSHR, NWB, SW);
      if (NMSHR >= WAYS) $fatal(1, "rv_dcache: %0d MSHRs could reserve every way of a set", NMSHR);
   end
   function [63:0] bytes(input [7:0] m);          // a byte mask as a bit mask
      integer b;
      for (b = 0; b < 8; b = b + 1) bytes[b*8 +: 8] = {8{m[b]}};
   endfunction
   function [WAYS-1:0] low(input [WAYS-1:0] m);   // the lowest set bit, one-hot (0 when none)
      low = m & ~(m - 1'b1);
   endfunction
   function [WYB-1:0] onehot_ix(input [WAYS-1:0] m);   // the index of a one-hot (an OR of indices)
      integer f;
      begin
         onehot_ix = {WYB{1'b0}};
         for (f = 0; f < WAYS; f = f + 1) if (m[f]) onehot_ix = onehot_ix | f[WYB-1:0];
      end
   endfunction
   function [PTB-1:0] tag_of(input [WAYS-1:0] oh, input [WAYS*PTB-1:0] t);   // a one-hot way's tag
      integer f;
      begin
         tag_of = {PTB{1'b0}};
         for (f = 0; f < WAYS; f = f + 1) if (oh[f]) tag_of = tag_of | t[f*PTB +: PTB];
      end
   endfunction
   function [WYB-1:0] arr_of(input [WYB-1:0] w, input [2:0] c);   // the array holding way w's chunk c
      arr_of = {w[WYB-1:3], w[2:0] + c};
   endfunction
   function [WYB-1:0] way_at(input integer a, input [2:0] c);      // the way whose chunk c array a holds
      way_at = {a[WYB-1:3], a[2:0] - c};
   endfunction
   integer i;

   // ---------------------------------------------------------------- the MSHRs
   reg              ms_v    [0:NMSHR-1];
   reg [57:0]       ms_pl   [0:NMSHR-1];         // the physical line
   reg [5:0]        ms_set  [0:NMSHR-1];
   reg              ms_sm   [0:NMSHR-1];         // ms_set is the main lookup's set (a flop: below)
   reg              ms_bm   [0:NMSHR-1];         // ms_pl is the store lookup's line (a flop: below)
   reg [WYB-1:0]    ms_way  [0:NMSHR-1];         // the way it reserves
   reg [1:0]        ms_bt   [0:NMSHR-1];         // the next beat expected
   reg              ms_bl   [0:NMSHR-1];         // a beat has landed: the merge buffer is being read
   reg              ms_dty  [0:NMSHR-1];         // a store merged: the line installs dirty
   // the merge buffers: chunk c of MSHR m at {m, c}
   // (distributed RAM, even and odd chunks apart so a beat reads one of each at {m, pair}; a chunk's
   // bytes count only while its bit in mb_cv[m] is set, and mb_z[m] makes the memory's line zeros)
   (* ram_style = "distributed" *) reg [63:0] mbd_e [0:NMSHR*4-1];
   (* ram_style = "distributed" *) reg [63:0] mbd_o [0:NMSHR*4-1];
   (* ram_style = "distributed" *) reg [7:0]  mbm_e [0:NMSHR*4-1];
   (* ram_style = "distributed" *) reg [7:0]  mbm_o [0:NMSHR*4-1];
   reg [7:0]        mb_cv   [0:NMSHR-1];
   reg              mb_z    [0:NMSHR-1];
   // the waiters, one per requester tag and one for the store: what to replay, and on what
   reg              wt_v    [0:NW-1];
   reg              wt_rdy  [0:NW-1];            // replay it
   reg              wt_any  [0:NW-1];            // it waits on any MSHR or write-back entry freeing, not on wt_m
   reg [MB-1:0]     wt_m    [0:NW-1];
   reg [63:0]       wt_pa   [0:NW-1];
   reg              wt_nc   [0:NW-1];
   // the store queue: taken, not yet written, merged or complete. A RING of four slots holding at
   // most SD: entry k (k = 0, the head) is slot sq_hp + k. A store that completes moves sq_hp, so
   // a lookup's verdict reaches two pointer flops, not every slot's enable; a store taken lands
   // at the tail, slot sq_hp + sc, whether or not the head completes in its cycle.
   reg  [1:0]       sc;                           // entries held
   reg  [1:0]       sq_hp;                        // the head's slot
   reg              sf_iss  [0:3];               // issued: in a lookup, or (the head) parked as the store's waiter
   reg              sf_mn   [0:3];               // a plain store the store lookup missed: the main lookup's
   reg  [63:0]      sf_pa   [0:3], sf_wd [0:3];
   reg  [7:0]       sf_wm   [0:3];
   reg              sf_nc   [0:3], sf_cbo [0:3], sf_zero [0:3], sf_keep [0:3];
   reg              sf_ox   [0:3];                // an outside request, not the store port's
   reg              oxq;                          // an outside request is queued
   reg              ox_slow;                      // ...and its first look did not finish it
   reg              xb_v;  reg [11:0] xb_pl;      // the line an outside flush blocks (its low bits), until ox_rel
   wire [1:0]       sq_h1   = sq_hp + 2'd1;       // entry 1's slot
   wire             st_done;
   wire [1:0]       hp_n    = st_done ? sq_h1 : sq_hp;   // the head after this cycle
   wire [63:0]      st_wd_n = (sq_take & (sq_pt == hp_n)) ? wr_data : sf_wd[hp_n];   // ...its data
   always @(posedge clk) if (!reset && (sc != 2'd0) && ((st_wd != sf_wd[sq_hp]) || (st_wm != sf_wm[sq_hp])))
      $fatal(1, "rv_dcache: the head's registered data %h/%h is not slot %0d's %h/%h", st_wd, st_wm, sq_hp, sf_wd[sq_hp], sf_wm[sq_hp]);
   // the head's data and mask (only the head resolves): registers, as every array's write data and
   // the merge read them -- the slot the head names next, as it stands after this cycle's write
   (* max_fanout = 16 *) reg [63:0] st_wd;
   (* max_fanout = 16 *) reg [7:0]  st_wm;
   wire             st_cbo  = sf_cbo[sq_hp], st_zero = sf_zero[sq_hp], st_keep = sf_keep[sq_hp];
   // which lookup the head's and the next entry's op go to: a plain store not yet missed is the
   // store lookup's
   wire             fast_hp = ~sf_nc[sq_hp] & ~sf_cbo[sq_hp] & ~sf_ox[sq_hp] & ~sf_mn[sq_hp];
   wire             fast_h1 = ~sf_nc[sq_h1] & ~sf_cbo[sq_h1] & ~sf_ox[sq_h1] & ~sf_mn[sq_h1];

   // ---------------------------------------------------------------- the write-back buffer
   reg              wb_v    [0:NWB-1];
   reg              wb_rdy  [0:NWB-1];           // read out: may go to memory
   reg              wb_sent [0:NWB-1];
   reg              wb_cpl  [0:NWB-1];           // a CBO completes with this write
   reg [57:0]       wb_pl   [0:NWB-1];
   (* ram_style = "registers" *) reg [511:0] wb_data [0:NWB-1];
   // the read-out: the line of (ro_way, ro_set), all eight chunks in one cycle, landing a cycle later
   reg              ro_on;  reg [WYB-1:0] ro_way;  reg [5:0] ro_set;  reg [WBB-1:0] ro_wb;
   reg              rl0;    reg [WYB-1:0] rl0_way; reg [WBB-1:0] rl0_wb;   // the arrays read (two cycles)
   reg              rl_v;   reg [WYB-1:0] rl_way;  reg [WBB-1:0] rl_wb;

   // ---------------------------------------------------------------- the fill (declared here: the lookup yields to it)
   reg            fb_v;  reg [WYB-1:0] fb_way;  reg [5:0] fb_set;  reg [1:0] fb_beat;
   reg            fb_last;  reg [MB-1:0] fb_m;
   reg            fl_p;                                                            // ...a cycle before it installs
   reg            fl_v;  reg [MB-1:0] fl_m;                                       // the line installing
   // ...and what it writes, taken from its MSHR as the last beat lands (stable from the first beat:
   // a store to the line waits for the install then) and held to the install: every way's state and
   // tag write reads flops
   (* max_fanout = 16 *) reg [5:0] fl_set;
   (* max_fanout = 16 *) reg [WAYS-1:0] fl_woh;
   (* max_fanout = 16 *) reg fl_dty;
   (* max_fanout = 16 *) reg [PTB-1:0] fl_tag;
   // the beat's two arrays (one-hot) and each array parity's write row -- the beat's, else the store
   // lookup's -- taken a cycle ahead: as the beat is, and from SB1
   (* max_fanout = 8 *)  reg [WAYS-1:0] fb_aw;
   (* max_fanout = 16 *) reg [8:0]      wa_e, wa_o;
   (* max_fanout = 16 *) reg [63:0]     wd_e, wd_o;    // ...and its data: the beat's chunks, else SB1's store's

   // ---------------------------------------------------------------- the NC slot: one access at a time
   localparam [SW-1:0] NMS = NMSHR;              // the first write-back slot
   localparam [SW-1:0] NWS = NWB;
   localparam [SW-1:0] NCS = NMSHR + NWB;        // the NC slot's memory-port slot
   reg            nc_v, nc_we, nc_sent, nc_rdy;
   reg  [57:0]    nc_pl;  reg [63:0] nc_pa;  reg [RTW-1:0] nc_tag;  reg [63:0] nc_wd, nc_data;  reg [7:0] nc_wm;
   // ---------------------------------------------------------------- the clean walk (inv_req) and the reset scan
   reg            cl_pend, cl_on, cl_wait;  reg [5:0] cl_r;   // the set it is at
   reg            scan_on;  reg [5:0] scan;

   // ---------------------------------------------------------------- the main lookup's input
   // THE LOOKUP TAKES TWO CYCLES: C1 (s1_v) reads the state at its set and compares, C2 (c2_v)
   // decides from C1's registered results and writes. A request issues only while C1 is empty, so
   // C1 and C2 never both hold one: the request's fields (s1_*) stand through both, C2's writes land
   // before the next C1 reads, and nothing else writes the set's state between its C1 and C2 but the
   // store lookup's dirty bit (folded into C2's copy) -- a line installs only with both empty.
   reg        door;                               // registered: open unless the clean or the scan runs
   reg        sdoor;                              // the stores': open while a clean waits for them
   reg        s1_v, c2_v;  reg [63:0] s1_pa;  reg [TB-1:0] s1_tag;  reg s1_nc;
   reg [NW-1:0] s1_oh;                            // s1_tag one-hot: the waiter entry the lookup writes
   reg        s1_f;                               // the lookup's load is M's fast one (f_cancel may void it)
   reg        s1_head;                            // a store in the lookup is the queue's head
   // the replay pick: the lowest waiter released by its MSHR, a drop or a conflict, else the first
   // released by any resource freeing after the last one picked, round robin -- each freeing hands
   // its resource to whoever replays first, so a fixed order would starve the last entries. ra_m
   // holds the entries after the last pick (none: from entry 0).
   reg  [NW-1:0]    ra_m;
   reg              rm_v, ra_v, ra_hv;  reg [TB-1:0] rm_t, ra_t, ra_ht;
   always @* begin
      rm_v = 1'b0;  rm_t = {TB{1'b0}};  ra_v = 1'b0;  ra_t = {TB{1'b0}};  ra_hv = 1'b0;  ra_ht = {TB{1'b0}};
      for (i = NW - 1; i >= 0; i = i - 1) begin
         if (wt_v[i] & wt_rdy[i] & ~wt_any[i]) begin rm_v = 1'b1; rm_t = i[TB-1:0]; end
         if (wt_v[i] & wt_rdy[i] &  wt_any[i]) begin ra_v = 1'b1; ra_t = i[TB-1:0]; end
         if (wt_v[i] & wt_rdy[i] &  wt_any[i] & ra_m[i]) begin ra_hv = 1'b1; ra_ht = i[TB-1:0]; end
      end
      if (ra_hv) ra_t = ra_ht;
   end
   // The pick is registered, with its entry's fields: the lookup never issues two cycles running
   // (C1 holds it off), so an entry picked a cycle ago is still waiting and ready, its fields as
   // they were, whenever it issues (only C2 writes an entry, and never one in the table). A stale
   // pick -- the entry issued last cycle -- meets the hold.
   reg              pk_v, pk_m, pk_nc;  reg [TB-1:0] pk_t;  reg [63:0] pk_pa;
   always @(posedge clk) begin
      pk_v  <= ~reset & (rm_v | ra_v);
      pk_m  <= rm_v;
      pk_t  <= rm_v ? rm_t : ra_t;
      pk_pa <= wt_pa[rm_v ? rm_t : ra_t];
      pk_nc <= wt_nc[rm_v ? rm_t : ra_t];
   end
   // the lookup yields while C1 holds a request, from a line's last beat until it installs (so none
   // is in C1 or C2 as it installs), while the read-out owns the arrays' read ports, while an NC
   // answer waits for the response port, and while the clean walk or the scan runs
   wire           hold   = s1_v | fb_last | fl_p | ro_on | nc_rdy | cl_on | scan_on;
   wire           rp_v   = pk_v & ~hold;
   wire [TB-1:0]  rp_t   = pk_t;
   // the queue issues its head to the main lookup, or the entry behind it in the cycle the head is
   // decided (C2) -- each only if it is the main lookup's
   wire           s1_hd  = c2_v & (s1_tag == ST) & s1_head;
   wire           sq_i0  = (sc != 2'd0) & ~sf_iss[sq_hp] & ~fast_hp;
   wire           sq_i1  = (sc > 2'd1) & sf_iss[sq_hp] & ~sf_iss[sq_h1] & s1_hd & ~fast_h1;
   // LOADS FIRST (the header): an urgent store goes ahead of a load presented now
   reg  [2:0]     st_age;
   wire           st_urgent = (sc == SD) | sf_cbo[sq_hp] | sf_nc[sq_hp] | sf_ox[sq_hp] | (st_age >= ST_WAIT);
   reg  [2:0]     fp_age;                         // the store lookup's head has waited this long
   wire           fp_hold = (fp_age >= FP_WAIT);   // ...and shuts the load door
   // M's load counts as presented whenever M holds one (f_hint, a register), so the store's turn and
   // the arrays' address never wait for f_req, the dTLB's late answer (rules I6, I8): a load that
   // does not come costs the store that cycle
   wire           st_go  = ~pk_v & ~hold & sdoor & (sq_i0 | sq_i1) & (~(door & (rd_req | f_hint) & ~fp_hold) | st_urgent);
   wire           si     = ~sq_i0;                // the entry issued
   wire [1:0]     sq_ps  = si ? sq_h1 : sq_hp;     // ...its slot
   wire           new_go = ~pk_v & ~hold & door & ~fp_hold & ~st_go & rd_req;
   assign rd_ack = new_go;
   // the fast load is taken when no replay, urgent store, held door or other read is in its way
   assign f_ok   = ~pk_v & ~hold & door & ~fp_hold & ~(sdoor & (sq_i0 | sq_i1) & st_urgent) & rd_quiet;
   wire           a_take = rp_v | st_go | new_go | f_req;
   wire           a_cap  = rp_v | st_go | new_go | f_ok;   // a_take on the state alone: the lookup's fields
   wire [TB-1:0]  a_tag = rp_v ? rp_t        : st_go ? ST    : f_ok ? wi(f_tag) : wi(rd_tag);
   wire           a_nc  = rp_v ? pk_nc       : st_go ? (sf_nc[sq_ps] & ~sf_cbo[sq_ps]) : f_ok ? 1'b0 : rd_nc;
   wire [63:0]    a_pa  = rp_v ? pk_pa       : st_go ? sf_pa[sq_ps] : f_ok ? f_pa : rd_pa;
   wire [5:0]     n1_set = a_cap ? a_pa[11:6] : s1_pa[11:6];   // the lookup's set after this edge
   // taking a store: a CBO or NC store held for its completion is the one already queued
   reg            sq_cn;                          // a CBO or an NC store is queued
   always @* begin
      sq_cn = 1'b0;
      for (i = 0; i < SD; i = i + 1) if ((i < sc) & (sf_cbo[sq_hp + i[1:0]] | sf_nc[sq_hp + i[1:0]]) & ~sf_ox[sq_hp + i[1:0]]) sq_cn = 1'b1;
   end
   // a store is taken when the store port was promised room (wr_room: sc != SD, unless an outside
   // request has it)
   wire           wr_take = wr_req & wr_room & ~((cbo_req | wr_nc) & sq_cn);
   // AN OUTSIDE REQUEST takes a slot no store takes: the store port's promised room stays its own,
   // and a CBO or NC store held for its completion keeps nothing out. One refused OX_WAIT cycles
   // takes the room from the store port until it is in (ox_pri): a store stream never starves it.
   wire           ox_pend = ox_req & ~oxq & ~ox_ack;
   wire           ox_take = ox_pend & (sc != SD) & ~wr_take;
   reg  [2:0]     ox_w;                           // the cycles it has been refused
   wire           ox_pri  = ox_pend & ~ox_take & (ox_w >= OX_WAIT - 3'd1);
   wire           sq_take = wr_take | ox_take;

   // ---------------------------------------------------------------- the store lookup's input
   // TWO CYCLES, as the main lookup: SB1 (sa_v) reads the state at its set and compares, SB2 (sb_v)
   // resolves from those registers and writes. It issues the queue's head, or the entry behind it
   // while the head is in either.
   reg            sa_v, sb_v;  reg [1:0] sa_slot, sb_slot;
   wire           fp_i0 = (sc != 2'd0) & ~sf_iss[sq_hp] & fast_hp;
   wire           fp_i1 = (sc > 2'd1) & ((sa_v & (sa_slot == sq_hp)) | (sb_v & (sb_slot == sq_hp))) & ~sf_iss[sq_h1] & fast_h1;
   wire           fp_go = sdoor & ~scan_on & (fp_i0 | fp_i1);
   wire [1:0]     fp_ps = fp_i0 ? sq_hp : sq_h1;

   // ---------------------------------------------------------------- the per-way state
   // valid, dirty, not-recently-used: a 64 x 1 distributed RAM per way and field, ONE write statement
   // each; the tags: two copies per way (the main lookup's and the clean walk's, the store lookup's),
   // written by an install only. The main lookup reads at its set, or the clean walk at its own.
   wire [5:0]          s1_set  = s1_pa[11:6];
   wire [2:0]          s1_c    = s1_pa[5:3];
   wire [PTB-1:0]      s1_ptg  = s1_pa[PABITS-1:12];
   wire [57:0]         s1_pl   = s1_pa[63:6];
   wire [5:0]          rset    = cl_on ? cl_r : s1_set;
   reg  [63:0]         sa_pa, sb_pa;              // the store lookup's PA in SB1 (taken as it issues), in SB2
   wire [5:0]          sa_set  = sa_pa[11:6];
   wire [5:0]          sb_set  = sb_pa[11:6];
   wire [2:0]          sb_c    = sb_pa[5:3];
   wire [WAYS-1:0]     vr, dr, ur, bv;            // valid, dirty, used at rset; valid at the store's set
   wire [WAYS*PTB-1:0] tr, bt;                    // the tags at rset, at the store's set
   wire [WAYS-1:0]     v_we, d_we, u_we;          // each way's one write (below)
   wire [WAYS*6-1:0]   v_wa, d_wa, u_wa;
   wire [WAYS-1:0]     v_wd, d_wd, u_wd;
   wire [WAYS-1:0]     t_we;
   wire [5:0]          t_wa;
   wire [PTB-1:0]      t_wd;
   genvar gw;
   generate for (gw = 0; gw < WAYS; gw = gw + 1) begin : way
      (* ram_style = "distributed" *) reg           va [0:63];
      (* ram_style = "distributed" *) reg           da [0:63];
      (* ram_style = "distributed" *) reg           ua [0:63];
      (* ram_style = "distributed" *) reg [PTB-1:0] ta [0:63];
      (* ram_style = "distributed" *) reg [PTB-1:0] tb [0:63];
      integer r;
      initial for (r = 0; r < 64; r = r + 1) begin va[r] = 1'b0; da[r] = 1'b0; ua[r] = 1'b0; ta[r] = 0; tb[r] = 0; end
      assign vr[gw] = va[rset];  assign dr[gw] = da[rset];  assign ur[gw] = ua[rset];
      assign tr[gw*PTB +: PTB] = ta[rset];
      assign bv[gw] = va[sa_set];  assign bt[gw*PTB +: PTB] = tb[sa_set];
      always @(posedge clk) if (v_we[gw]) va[v_wa[gw*6 +: 6]] <= v_wd[gw];
      always @(posedge clk) if (d_we[gw]) da[d_wa[gw*6 +: 6]] <= d_wd[gw];
      always @(posedge clk) if (u_we[gw]) ua[u_wa[gw*6 +: 6]] <= u_wd[gw];
      always @(posedge clk) if (t_we[gw]) ta[t_wa] <= t_wd;
      always @(posedge clk) if (t_we[gw]) tb[t_wa] <= t_wd;
   end endgenerate

   // ---------------------------------------------------------------- the store lookup, SB1: the compare
   wire [PTB-1:0]  sa_ptg = sa_pa[PABITS-1:12];
   reg  [WAYS-1:0] sa_hitv;
   always @* for (i = 0; i < WAYS; i = i + 1) sa_hitv[i] = bv[i] & (bt[i*PTB +: PTB] == sa_ptg);
   wire [LB-1:0]   n1_sbl  = fp_go ? sf_pa[fp_ps][6 +: LB] : sa_pa[6 +: LB];   // its line after this edge
   // the MSHR filling its line, if one is (no copy is valid while an MSHR fills its line)
   reg  [MB-1:0]   sa_hk;  reg sa_ms;
   always @* begin
      sa_hk = 0;  sa_ms = 1'b0;
      for (i = NMSHR - 1; i >= 0; i = i - 1)
         if (ms_v[i] & ms_bm[i]) begin sa_ms = 1'b1; sa_hk = i[MB-1:0]; end
   end
   // ---------------------------------------------------------------- the store lookup, SB2: the outcome
   // SB1's results, registered (with the main lookup's). The main lookup's decision in SB1's set
   // (but a load that hits: it may evict the line) and an install there change what SB1 saw: SB2
   // then goes back to the queue (sb_chg).
   reg  [WAYS-1:0] bhitv;  reg [MB-1:0] sb_hk;  reg sb_ms, sb_chg;
   wire            bhit  = |bhitv;
   // the array it writes: the hit rotated by c within each group
   reg  [WAYS-1:0] bsel;
   always @* for (i = 0; i < WAYS; i = i + 1) bsel[i] = bhitv[way_at(i, sb_c)];
   wire            sb_hd = sb_v & (sb_slot == sq_hp);            // it is the head now: it resolves
   wire            sb_ok = sb_hd & ~sb_chg;                      // ...from what SB1 saw
   wire            sb_set1 = c2_v & (s1_set == sb_set);         // the main lookup decides in its set
   wire            sb_mgok = ~ms_bl[sb_hk] & ~(cr_f & (cr_m == sb_hk));   // no beat has landed

   // ---------------------------------------------------------------- the main lookup, C1: the compares
   wire           s1_st  = s1_tag == ST;
   wire           k_cbo = s1_st & st_cbo;
   wire           k_z   = k_cbo & st_zero;
   wire           k_cln = k_cbo & ~st_zero & st_keep;
   wire           k_fl  = k_cbo & ~st_zero & ~st_keep;
   wire           k_nc  = s1_nc;
   wire           k_ld  = ~s1_st & ~s1_nc;
   wire           k_st  = s1_st & ~k_cbo & ~s1_nc;
   // a load behind a queued store to its line, or a load whose row read met the store written at
   // the edge before C1 (w1, below): it waits, and nothing else happens to it. Both are C1's: a
   // store queued then is written at C1's edge at the earliest, after the row was read.
   reg  sq_ln;
   always @* begin
      sq_ln = 1'b0;
      for (i = 0; i < SD; i = i + 1) if ((i < sc) & (sf_pa[sq_hp + i[1:0]][6 +: LB] == s1_pa[6 +: LB])) sq_ln = 1'b1;
   end
   reg            w1_v;  reg [LB-1:0] w1_pl;      // the store written at the last edge, its line
   // a cancelled fast load is no request at all: it neither waits, nor answers, nor fills
   wire s1_live = s1_v & ~(s1_f & f_cancel);
   reg  [WAYS-1:0] c1_hitv;
   always @* for (i = 0; i < WAYS; i = i + 1) c1_hitv[i] = vr[i] & (tr[i*PTB +: PTB] == s1_ptg);
   // MSHR match (the line already being filled) and the ways MSHRs reserve in the set; a free MSHR
   // and a dirty one as they stand (only C2 allocates, and none frees while C1 or C2 is full)
   reg  [NMSHR-1:0] c1_mshv;  reg ms_dirty;  reg [WAYS-1:0] rsv;
   reg  [MB-1:0]    c1_mhk, c1_free;  reg c1_free_v;
   always @* begin
      c1_mshv = 0;  c1_mhk = 0;  c1_free = 0;  c1_free_v = 1'b0;  ms_dirty = 1'b0;  rsv = {WAYS{1'b0}};
      for (i = NMSHR - 1; i >= 0; i = i - 1) begin
         if (ms_v[i] & (ms_pl[i][LB-1:0] == s1_pl[LB-1:0])) begin c1_mshv[i] = 1'b1; c1_mhk = i[MB-1:0]; end
         if (ms_v[i] & ms_sm[i]) rsv[ms_way[i]] = 1'b1;
         if (~ms_v[i]) begin c1_free = i[MB-1:0]; c1_free_v = 1'b1; end
         if (ms_v[i] & ms_dty[i]) ms_dirty = 1'b1;
      end
   end
   // the write-back buffer: the line still in it (C1), a free entry (as it stands)
   reg              c1_wbh, wb_free_v, wb_any;  reg [WBB-1:0] wb_free;
   always @* begin
      c1_wbh = 1'b0;  wb_free_v = 1'b0;  wb_free = 0;  wb_any = 1'b0;
      for (i = NWB - 1; i >= 0; i = i - 1) begin
         if (wb_v[i] & (wb_pl[i][LB-1:0] == s1_pl[LB-1:0])) c1_wbh = 1'b1;
         if (~wb_v[i]) begin wb_free = i[WBB-1:0]; wb_free_v = 1'b1; end
         if (wb_v[i]) wb_any = 1'b1;
      end
   end
   // the victim (the header): an unreserved invalid way, else an unreserved unused one, else the
   // first unreserved -- and then the set's used bits clear. One-hot: each pool's lowest way is
   // taken side by side, and the dirty bit is the lowest valid one's, read without an index (a way
   // in the second or third pool is valid whenever the first is empty).
   wire [WAYS-1:0] cand    = ~rsv;
   wire [WAYS-1:0] v_inv   = cand & ~vr;
   wire [WAYS-1:0] v_nu    = cand & vr & ~ur;
   wire [WAYS-1:0] c1_vicv = (|v_nu) ? low(v_nu) : low(cand);
   wire [WAYS-1:0] c1_vic  = (|v_inv) ? low(v_inv) : c1_vicv;

   // ---------------------------------------------------------------- the main lookup, C2: the decision
   // C1's results, registered (below, every cycle: they are read in C2 only). A resource C1 saw
   // busy may free before C2 (a write-back completing, the store queue's head finishing): C2's wait
   // counts C1's freeing (fr_q) as its own. The block of a line an outside flush finishes is read in
   // C2 itself (xb_hit): it may be set after C1.
   reg              c2_live, ln_q, w1_q, ms_hit, wb_hit, age_q, vic_inv, vic_age, vbad_q, fr_q;
   reg  [WAYS-1:0]  hitv, dr_q, vic_oh, vic_v;
   reg  [MB-1:0]    ms_hk;
   // the free MSHR as C1 saw it (none allocates or frees between C1 and C2: asserted)
   (* max_fanout = 16 *) reg [MB-1:0] ms_free;  reg ms_free_v;
   wire blk_st = c2_live & ~s1_st & ln_q;
   wire blk_w1 = c2_live & ~s1_st & ~ln_q & w1_q;
   // a store behind the head while the head is parked: it goes back to the queue, untouched
   wire sq_bk  = c2_v & s1_st & ~s1_head;
   wire l1_v   = c2_live & ~blk_st & ~blk_w1 & ~sq_bk;   // the lookup proceeds
   wire           hit    = |hitv;
   // THE STORE LOOKUP'S OUTCOME. It waits for the main lookup's decision in its set unless that is
   // a load that hits (a miss may evict its way; a store, an NC access or a CBO may drop, clean or
   // write the line), for a fill beat (the arrays' write buses are the beat's), and for a line
   // installing (the dirty bits' one write port). Its write in C1's set reaches C2 through dr_q;
   // nothing else of the main lookup's depends on it.
   wire            sb_wait = (sb_set1 & ~(k_ld & hit)) | fb_v | fl_v;
   wire            fp_wr   = sb_ok & bhit & ~sb_wait;            // it writes the line: done
   wire            fp_mg   = sb_ok & ~bhit & sb_ms & sb_mgok;    // its bytes into the fill's merge buffer: done
   wire            fp_miss = sb_ok & ~bhit & ~sb_ms;             // to the main lookup
   wire            fp_back = sb_v & ~fp_wr & ~fp_mg & ~fp_miss;  // back to the queue, to issue again
   wire [WYB-1:0] hw     = onehot_ix(hitv);
   wire           cp_dty = |(hitv & dr_q);        // the copy is dirty (one way hits: invariant 4)

   // the answer: a hit
   wire        ans = l1_v & hit & k_ld;
   assign rd_pre_a = tg(s1_tag);
   assign rd_pre_b = nc_tag;
   // the arrays as read at the request's issue (two cycles: in C2): bk_q[a] is way (a - c)'s chunk
   // c; the hit rotated by c selects one
   wire [64*WAYS-1:0] bk_q;
   reg  [63:0] ans_chunk;
   always @* begin
      ans_chunk = 64'd0;
      for (i = 0; i < WAYS; i = i + 1)
         if (hitv[way_at(i, s1_c)]) ans_chunk = ans_chunk | bk_q[i*64 +: 64];
   end

   wire            xb_hit  = xb_v & (xb_pl == s1_pl[11:0]);   // the set and six tag bits: a false match only delays
   wire [WYB-1:0]  vic     = onehot_ix(vic_oh);
   wire            vic_dty = ~vic_inv & (|(vic_v & dr_q));

   // ---------------------------------------------------------------- what the main lookup does (one of these)
   // a copy goes: under an NC access, a flush or a zero
   wire drop_c    = l1_v & hit & (k_nc | k_fl | k_z);
   wire drop_wb   = cp_dty & ~k_z;                                    // ...written back if dirty, unless zeroed
   wire cln_c     = l1_v & k_cln & hit;                               // cbo.clean on a copy
   wire absent    = l1_v & ~hit;
   wire free_ln   = absent & ~wb_hit & ~ms_hit;                       // not cached, filling or being written
   wire alloc_c   = free_ln & ~xb_hit & (k_ld | k_st | k_z) & ms_free_v;
   wire ro_need   = (drop_c & drop_wb) | (alloc_c & vic_dty) | (cln_c & cp_dty);   // a dirty line leaves or cleans
   wire ro_block  = ro_need & ro_on;
   wire wb_block  = ro_need & ~ro_on & ~wb_free_v;
   // each outcome blocks on its own dirty line only (drop_c, alloc_c and cln_c exclude each other)
   wire ro_busy   = ro_on | ~wb_free_v;
   wire m_drop    = drop_c & ~(drop_wb & ro_busy);
   wire m_alloc   = alloc_c & ~(vic_dty & ro_busy);
   wire m_clean   = cln_c & ~(cp_dty & ro_busy);
   wire cr_f;  wire [MB-1:0] cr_m;                                    // a fill beat landing now (below)
   // (no copy is valid while an MSHR fills its line, so a hit and a join never meet)
   wire m_join    = absent & ~wb_hit & ms_hit & (k_ld | k_z);        // wait on the MSHR
   wire m_nc      = free_ln & ~xb_hit & k_nc & ~nc_v;                 // to memory through the NC slot
   wire m_cdone   = free_ln & (k_cln | k_fl);                         // a CBO with nothing cached: done
   wire m_defer   = ro_block | blk_w1;                                // replay at once
   // the outcomes are decided without the dirty bits; a dirty line that cannot leave now only blocks
   wire m_rest    = l1_v & ~ans & ~drop_c & ~alloc_c & ~cln_c & ~m_join & ~m_nc & ~m_cdone;   // none: it waits
   wire m_wait    = blk_st | wb_block | m_rest;
   // it enters the waiter table unless it finishes in the lookup
   wire wt_in     = blk_w1 | blk_st | m_rest | m_join | (drop_c & ~k_fl) | (alloc_c & k_ld) | (ro_need & ro_busy);
   wire age_hit   = ans & age_q;                                      // the hit would leave every used bit set
   // the store-port op finishes: now, or when memory has its write (a CBO's write-back, an NC store)
   wire cw_ok;  wire [SW-1:0] cw_e;                                   // a write-back completes (below)
   wire fin_now   = s1_st & (m_alloc | (m_clean & ~cp_dty) | (m_drop & k_fl & ~drop_wb) | m_cdone);
   wire fin_wb    = cw_ok & wb_cpl[cw_e[WBB-1:0]];
   wire fin_nc    = cw_valid & (cw_slot == NCS) & nc_v & nc_we & nc_sent;
   assign st_done = fin_now | fin_wb | fin_nc | fp_wr | fp_mg;
   wire nc_done   = fin_nc | (nc_rdy & ~ans);                         // the NC slot frees
   wire [1:0] sq_pt = sq_hp + sc;                                     // the tail's slot
   // counted once per request, never per replay: an access is a load or store taken, a miss a line
   // fill started
   assign perf_access = new_go | f_req | wr_take;
   assign perf_miss   = m_alloc;

   // ---------------------------------------------------------------- the data arrays
   // read: every array at the request's {set, c}, or the read-out's line (array a of its group
   // holds its chunk a - way); write: a fill beat's two arrays, else the store lookup's one, through
   // one address and data bus per array parity (the beat's chunk 2b lands in the arrays of its way's
   // parity, 2b+1 in the others; the store lookup never writes beside a beat)
   wire [8:0]     a_row   = {a_pa[11:6], a_pa[5:3]};
   genvar ga;
   generate for (ga = 0; ga < WAYS; ga = ga + 1) begin : arr
      wire [2:0] ro_c = ga[2:0] - ro_way[2:0];      // the read-out's chunk in this array
      wire fw  = fb_aw[ga];
      wire sw  = fp_wr & bsel[ga];
      smolrv64_sdpram_be #(.ADDR_WIDTH(9), .NBYTES(8), .LAT(2)) u_a
        (.clock(clk), .rd_addr(ro_on ? {ro_set, ro_c} : a_row), .rd_data(bk_q[ga*64 +: 64]),
         .wr_be(fw ? 8'hff : sw ? st_wm : 8'h00),
         .wr_addr(ga[0] ? wa_o : wa_e), .wr_data(ga[0] ? wd_o : wd_e));
   end endgenerate
   // the read-out's line (two cycles after its address): its group's eight arrays, then rotated
   // back by the way
   reg  [511:0]   ro_grp;
   always @* for (i = 0; i < 8; i = i + 1) ro_grp[i*64 +: 64] = bk_q[{rl_way[WYB-1:3], i[2:0]}*64 +: 64];

   // ---------------------------------------------------------------- the fill
   // Beats land in order of their MSHR's burst (beat 0..3); each writes two chunks of the reserved
   // way, the merge buffer's bytes over the memory's.
   assign        cr_f   = cr_valid & (cr_slot < NMS);           // a fill beat (else the NC slot's)
   assign        cr_m   = cr_slot[MB-1:0];
   wire [MB+1:0] cr_a   = {cr_m, cr_beat};
   wire [7:0]    cr_me  = mb_cv[cr_m][{cr_beat, 1'b0}] ? mbm_e[cr_a] : 8'd0;
   wire [7:0]    cr_mo  = mb_cv[cr_m][{cr_beat, 1'b1}] ? mbm_o[cr_a] : 8'd0;
   wire [127:0]  cr_bs  = mb_z[cr_m] ? 128'd0 : cr_data;
   wire [63:0]   cr_lo  = (cr_bs[63:0]   & ~bytes(cr_me)) | (mbd_e[cr_a] & bytes(cr_me));
   wire [63:0]   cr_hi  = (cr_bs[127:64] & ~bytes(cr_mo)) | (mbd_o[cr_a] & bytes(cr_mo));

   // THE ONE MERGE-BUFFER WRITE: the store lookup's head over what the MSHR it joins already holds,
   // else the main lookup's store fresh into the free MSHR (dead unless the store allocates it).
   // The head is in one lookup at a time.
   wire           mw_sb   = sb_hd;
   wire [63:0]    mw_pa   = mw_sb ? sb_pa : s1_pa;
   wire [MB-1:0]  mw_m    = mw_sb ? sb_hk : ms_free;
   wire [MB+1:0]  mw_a    = {mw_m, mw_pa[5:4]};
   wire           mw_old  = mw_sb & mb_cv[sb_hk][sb_c];
   wire [63:0]    mw_od   = mw_pa[3] ? mbd_o[mw_a] : mbd_e[mw_a];
   wire [7:0]     mw_om   = mw_pa[3] ? mbm_o[mw_a] : mbm_e[mw_a];
   wire [7:0]     mw_mk   = (mw_old ? mw_om : 8'd0) | st_wm;
   wire [63:0]    mw_d    = mw_old ? ((mw_od & ~bytes(st_wm)) | (st_wd & bytes(st_wm))) : st_wd;
   wire           mw_we   = mw_sb ? fp_mg : c2_v & k_st & ms_free_v;
   always @(posedge clk) begin
      if (mw_we & ~mw_pa[3]) begin mbd_e[mw_a] <= mw_d;  mbm_e[mw_a] <= mw_mk; end
      if (mw_we &  mw_pa[3]) begin mbd_o[mw_a] <= mw_d;  mbm_o[mw_a] <= mw_mk; end
   end

   // ---------------------------------------------------------------- the memory port
   // Reads first: an allocated MSHR's read waits in the issue queue, in allocation order; then the
   // NC access; a write-back goes when neither waits.
   reg  [MB-1:0]   iq [0:NMSHR-1];
   reg             ms_iss [0:NMSHR-1];            // the MSHR's read has gone to memory (the invariant below)
   reg  [MB-1:0]   iq_rd, iq_wr;
   reg  [MB:0]     iq_n;
   reg             ws_v;  reg [WBB-1:0] ws;       // the write-back to send
   always @* begin
      ws_v = 1'b0;  ws = 0;
      for (i = NWB - 1; i >= 0; i = i - 1) if (wb_v[i] & wb_rdy[i] & ~wb_sent[i]) begin ws_v = 1'b1; ws = i[WBB-1:0]; end
   end
   wire   rq_v     = (iq_n != 0);
   wire   nq_v     = nc_v & ~nc_sent;
   assign cq_valid = rq_v | nq_v | ws_v;
   assign cq_we    = rq_v ? 1'b0 : nq_v ? nc_we : 1'b1;
   assign cq_slot  = rq_v ? {{(SW-MB){1'b0}}, iq[iq_rd]} : nq_v ? NCS : NMS + {{(SW-WBB){1'b0}}, ws};
   assign cq_addr  = rq_v ? ms_pl[iq[iq_rd]] : nq_v ? nc_pl : wb_pl[ws];
   assign cq_wmask = rq_v ? 64'd0 : nq_v ? (nc_we ? {56'd0, nc_wm} << {nc_pa[5:3], 3'b000} : 64'd0) : ~64'd0;
   assign cq_wdata = nq_v ? {8{nc_wd}} : wb_data[ws];
   wire   iq_go    = cq_ready & rq_v;
   wire   nq_go    = cq_ready & ~rq_v & nq_v;
   wire   ws_go    = cq_ready & ~rq_v & ~nq_v & ws_v;
   assign cw_e     = cw_slot - NMS;
   assign cw_ok    = cw_valid & (cw_slot >= NMS) & (cw_e < NWS) & wb_v[cw_e[WBB-1:0]] & wb_sent[cw_e[WBB-1:0]];
   wire   freed    = fl_v | cw_ok | st_done | nc_done | ox_rel;   // an MSHR, a write-back entry, the store, the NC slot or a block frees

   // C1's results into C2, and SB1's into SB2. The dirty bits take the store lookup's write in C1's
   // set (it lands between C1's read and C2).
   always @(posedge clk) begin
      c2_live <= s1_live;
      ln_q    <= sq_ln;
      w1_q    <= w1_v & (w1_pl == s1_pa[6 +: LB]);
      hitv    <= c1_hitv;
      dr_q    <= dr | ({WAYS{fp_wr & (sb_set == s1_set)}} & bhitv);
      age_q   <= &(ur | c1_hitv);
      ms_hit  <= |c1_mshv;
      ms_hk   <= c1_mhk;
      ms_free <= c1_free;
      ms_free_v <= c1_free_v;
      wb_hit  <= c1_wbh;
      vic_oh  <= c1_vic;
      vic_v   <= c1_vicv;
      vic_inv <= |v_inv;
      vic_age <= ~(|v_inv) & ~(|v_nu);
      vbad_q  <= (|(rsv & c1_vic)) | ~(|cand);
      fr_q    <= freed;
      bhitv   <= sa_hitv;
      sb_hk   <= sa_hk;
      sb_ms   <= sa_ms;
      sb_chg  <= (c2_v & (s1_set == sa_set) & ~(k_ld & hit)) | (fl_v & (fl_set == sa_set));
   end

   // ---------------------------------------------------------------- the state writes
   // ONE write statement per way and field, its {we, addr, data} muxed first. The lookup is empty
   // while a line installs, the store lookup waits out an install, and the store lookup and the main
   // lookup never both write a way's dirty bit (only the queue's head writes, through one of them):
   // so no two writers meet at one way's field.
   wire [MB-1:0]  im      = fl_m;                       // the MSHR installing (fl_* its fields)
   // the clean walk goes by SET: the set's ways are read at once, the lowest dirty one is written back
   // (with the lookup empty, no install, the read-out and a write-back entry free), and the walk moves
   // on when the set is clean -- 64 steps and one per dirty line
   wire [WAYS-1:0] cl_dv  = vr & dr;
   wire [WAYS-1:0] cl_oh  = low(cl_dv);
   wire [WYB-1:0]  cl_k   = onehot_ix(cl_oh);
   wire            cl_ro  = cl_on & (|cl_dv) & ~s1_v & ~fl_v & ~ro_on & ~rl0 & ~rl_v & wb_free_v;
   wire            cl_adv = cl_on & ~(|cl_dv) & ~s1_v;
   assign t_wa = fl_set;
   assign t_wd = fl_tag;
   // The valid and used arrays' writers besides the scan and an install are the lookup's, and the
   // lookup is empty while a line installs (hold at its last beat) and while the scan runs: one
   // address serves every way.
   wire [5:0] vu_wa = scan_on ? scan : fl_v ? fl_set : s1_set;
   always @(posedge clk) if (!reset && c2_v && ((ms_free_v != c1_free_v) || (ms_free_v && (ms_free != c1_free))))
      $fatal(1, "rv_dcache: the free MSHR changed between C1 and C2 (%b %0d -> %b %0d)", ms_free_v, ms_free, c1_free_v, c1_free);
   always @(posedge clk) if (!reset && fp_wr && ((wd_e != st_wd) || (wd_o != st_wd)))
      $fatal(1, "rv_dcache: the store lookup writes %h/%h, not the head's data %h", wd_e, wd_o, st_wd);
   always @(posedge clk) if (!reset && rp_v && !(wt_v[rp_t] && wt_rdy[rp_t]))
      $fatal(1, "rv_dcache: a replay of waiter %0d, which is not waiting and ready", rp_t);
   always @(posedge clk) if (!reset && (s1_v || c2_v) && (fl_v || scan_on || cl_on))
      $fatal(1, "rv_dcache: the lookup holds a request while a line installs, the scan or the clean walk runs");
   generate for (gw = 0; gw < WAYS; gw = gw + 1) begin : wst
      wire inst_w = fl_v & fl_woh[gw];
      wire fp_w   = fp_wr & bhitv[gw];
      wire hit_w  = hitv[gw];
      assign t_we[gw] = inst_w;
      // valid: an install sets it; the scan, a victim's reservation and a drop clear it
      assign v_we[gw] = scan_on | inst_w | (m_alloc & vic_oh[gw]) | (m_drop & hit_w);
      assign v_wa[gw*6 +: 6] = vu_wa;
      assign v_wd[gw] = inst_w & ~scan_on;
      // dirty: an install (if a store merged), a store's write and the store lookup's set it; a clean
      // of a copy, the clean walk's read-out and the scan clear it
      assign d_we[gw] = scan_on | inst_w | fp_w | (m_clean & cp_dty & hit_w) | (cl_ro & cl_oh[gw]);
      assign d_wa[gw*6 +: 6] = scan_on ? scan : inst_w ? fl_set : fp_w ? sb_set : cl_ro ? cl_r : s1_set;
      assign d_wd[gw] = ~scan_on & (inst_w ? fl_dty : fp_w);
      // used: an install and a hit set it; a hit that would leave the set's bits all set, and an
      // allocation that found no unused way, clear the others
      assign u_we[gw] = scan_on | inst_w | (ans & hit_w) | age_hit | (m_alloc & vic_age);
      assign u_wa[gw*6 +: 6] = vu_wa;
      assign u_wd[gw] = ~scan_on & (inst_w | (ans & hit_w));
   end endgenerate

   // ---------------------------------------------------------------- the machine
   always @(posedge clk) begin
      if (reset) begin
         door <= 1'b0;  sdoor <= 1'b0;  s1_v <= 1'b0;  c2_v <= 1'b0;  s1_f <= 1'b0;  sa_v <= 1'b0;  sb_v <= 1'b0;  rd_valid <= 1'b0;  rd_f <= 1'b0;  wr_acc <= 1'b0;  wr_cpl <= 1'b0;
         sc <= 2'd0;  sq_hp <= 2'd0;  wr_room <= 1'b0;  ox_w <= 3'd0;  oxq <= 1'b0;  xb_v <= 1'b0;  ox_ack <= 1'b0;  ox_dty <= 1'b0;  ox_slow <= 1'b0;
         nc_v <= 1'b0;  nc_rdy <= 1'b0;  cl_pend <= 1'b0;  cl_on <= 1'b0;  cl_wait <= 1'b0;
         scan_on <= 1'b1;  scan <= 6'd0;  fl_p <= 1'b0;  fl_v <= 1'b0;  ra_m <= {NW{1'b0}};  w1_v <= 1'b0;
         st_age <= 3'd0;  fp_age <= 3'd0;
         iq_n <= 0;  iq_rd <= 0;  iq_wr <= 0;  fb_v <= 1'b0;  fb_aw <= {WAYS{1'b0}};  fb_last <= 1'b0;  ro_on <= 1'b0;  rl0 <= 1'b0;  rl_v <= 1'b0;
         for (i = 0; i < NMSHR; i = i + 1) ms_v[i] <= 1'b0;
         for (i = 0; i < NW; i = i + 1)    begin wt_v[i] <= 1'b0; wt_rdy[i] <= 1'b0; end
         for (i = 0; i < NWB; i = i + 1)   wb_v[i] <= 1'b0;
      end else begin
         // the main lookup takes whatever the arrays were addressed for, and decides it a cycle later
         s1_v <= a_take;
         c2_v <= s1_v;
         if (a_cap) begin s1_pa <= a_pa; s1_tag <= a_tag; s1_nc <= a_nc; s1_oh <= {{(NW-1){1'b0}}, 1'b1} << a_tag; s1_f <= f_req; end
         // the head's own replay, the head, or the entry behind a head that resolves now
         if (a_cap) s1_head <= rp_v | ~si | st_done;
         if (rp_v) wt_v[rp_t] <= 1'b0;                  // a replaying waiter leaves the table
         if (rp_v & ~pk_m) ra_m <= {NW{1'b1}} << (rp_t + 1'b1);   // the entries after it
         st_age <= (st_go | ~(sq_i0 | sq_i1)) ? 3'd0 : (st_age == 3'd7) ? st_age : st_age + 3'd1;
         // the store lookup: the head writes or merges, misses to the main lookup, or goes back
         sa_v <= fp_go;  sb_v <= sa_v;  sb_slot <= sa_slot;  sb_pa <= sa_pa;
         if (fp_go) begin sa_slot <= fp_ps;  sa_pa <= sf_pa[fp_ps];  sf_iss[fp_ps] <= 1'b1; end
         if (fp_back) sf_iss[sb_slot] <= 1'b0;
         if (fp_miss) begin sf_iss[sb_slot] <= 1'b0;  sf_mn[sb_slot] <= 1'b1; end
         fp_age <= (sb_hd & bhit & sb_wait) ? ((fp_age == 3'd7) ? fp_age : fp_age + 3'd1) : 3'd0;
         // the store queue: the head leaves when it is done, a store taken joins the tail, an entry
         // behind a parked head goes back to waiting its turn
         wr_acc <= wr_take;
         wr_cpl <= st_done & st_cbo & ~sf_ox[sq_hp] | fin_nc;
         ox_ack <= st_done & sf_ox[sq_hp];
         ox_dty <= ox_slow;
         // the outside request at the head in the lookup: done now and clean, or anything else -- a
         // write-back, a wait on a write-back or a fill of its line -- which may change memory
         if (st_done & sf_ox[sq_hp]) ox_slow <= 1'b0;
         else if (c2_v & s1_st & s1_head & sf_ox[sq_hp] & ~fin_now) ox_slow <= 1'b1;
         if (ox_take) oxq <= 1'b1;  else if (st_done & sf_ox[sq_hp]) oxq <= 1'b0;
         if (st_done & sf_ox[sq_hp] & ~sf_keep[sq_hp]) begin xb_v <= 1'b1;  xb_pl <= sf_pa[sq_hp][17:6]; end
         else if (ox_rel) xb_v <= 1'b0;
         if (st_done) sq_hp <= sq_h1;
         st_wd <= st_wd_n;
         st_wm <= (sq_take & (sq_pt == hp_n)) ? wr_mask : sf_wm[hp_n];
         if (st_go) sf_iss[sq_ps] <= 1'b1;
         if (sq_take) begin                        // a store-port op, else the outside request (a CBO by PA)
            sf_iss[sq_pt] <= 1'b0;  sf_mn[sq_pt] <= 1'b0;  sf_pa[sq_pt] <= wr_take ? wr_pa : ox_pa;
            sf_wd[sq_pt] <= wr_data;  sf_wm[sq_pt] <= wr_mask;  sf_nc[sq_pt] <= wr_take & wr_nc;
            sf_cbo[sq_pt] <= ~wr_take | cbo_req;  sf_zero[sq_pt] <= wr_take & cbo_zero;
            sf_keep[sq_pt] <= wr_take ? cbo_keep : ox_keep;  sf_ox[sq_pt] <= ~wr_take;
         end
         if (sq_bk) sf_iss[sq_h1] <= 1'b0;
         sc      <= sc + {1'b0, sq_take} - {1'b0, st_done};
         wr_room <= ((sc + {1'b0, sq_take} - {1'b0, st_done}) != SD) & ~ox_pri;
         ox_w    <= (ox_pend & ~ox_take) ? ((ox_w == 3'd7) ? ox_w : ox_w + 3'd1) : 3'd0;
         // a load's answer: from the lookup, else the NC slot's -- the doubleword, unshifted
         rd_valid <= ans | nc_rdy;
         rd_src_b <= ~ans & nc_rdy;
         rd_f     <= ans & s1_f;
         if (ans) begin
            rd_data      <= ans_chunk;
            rd_resp_tag  <= tg(s1_tag);  rd_resp_addr <= s1_pa;
         end else if (nc_rdy) begin
            rd_data      <= nc_data;
            rd_resp_tag  <= nc_tag;  rd_resp_addr <= nc_pa;
         end
         // the store written at this edge, for the row the next lookup read as it was written
         w1_v  <= fp_wr;
         w1_pl <= sb_pa[6 +: LB];
         // the release: the installing MSHR's waiters, and on any freeing every waiter on any
         for (i = 0; i < NW; i = i + 1)
            if (wt_v[i] & ((freed & wt_any[i]) | (fl_v & ~wt_any[i] & (wt_m[i] == fl_m)))) wt_rdy[i] <= 1'b1;
         // THE STATE A DECISION FILLS IS WRITTEN WHETHER OR NOT IT IS TAKEN, wherever writing it is
         // dead: the request in the lookup owns its waiter entry (it left the table to get there,
         // or it is new), and the free MSHR is not live. Only the few bits that make an entry or
         // an MSHR live wait for the decision.
         if (c2_live & ~sq_bk) begin wt_pa[s1_tag] <= s1_pa;  wt_nc[s1_tag] <= s1_nc; end
         if (c2_v & ms_free_v) begin
            ms_pl[ms_free] <= s1_pl;  ms_set[ms_free] <= s1_set;  ms_way[ms_free] <= vic;
            ms_bt[ms_free] <= 2'd0;  ms_bl[ms_free] <= 1'b0;  ms_dty[ms_free] <= k_st | k_z;
            // its merge buffer: a store's chunk (written above), a zero's whole line, else empty
            mb_cv[ms_free] <= k_st ? (8'd1 << s1_c) : 8'd0;
            mb_z[ms_free]  <= k_z;
         end
         // each MSHR's set against the main lookup's and its line against the store lookup's, all
         // as they stand after this edge (the victim's reservations and the store's MSHR read them,
         // the compares out of their cones)
         for (i = 0; i < NMSHR; i = i + 1) begin
            ms_sm[i] <= (c2_v & ms_free_v & (ms_free == i[MB-1:0])) ? (s1_set == n1_set) : (ms_set[i] == n1_set);
            ms_bm[i] <= (c2_v & ms_free_v & (ms_free == i[MB-1:0])) ? (s1_pl[LB-1:0] == n1_sbl) : (ms_pl[i][LB-1:0] == n1_sbl);
         end
         // the next slot takes the free MSHR each cycle, the pointer moving on an allocation -- except
         // when the queue is full: then that slot is the oldest unissued MSHR's (iq_wr is iq_rd)
         if (!iq_n[MB]) iq[iq_wr] <= ms_free;
         // a request enters the waiter table
         for (i = 0; i < NW; i = i + 1)
            if (wt_in & s1_oh[i]) begin
               wt_v[i]   <= 1'b1;
               wt_rdy[i] <= m_drop | m_defer | (m_wait & (freed | fr_q));
               wt_any[i] <= m_wait;
               wt_m[i]   <= m_join ? ms_hk : ms_free;
            end
         // an allocation: the MSHR goes live and its read is queued
         if (m_alloc) begin ms_v[ms_free] <= 1'b1;  ms_iss[ms_free] <= 1'b0;  iq_wr <= iq_wr + 1'b1; end
         if (iq_go) ms_iss[iq[iq_rd]] <= 1'b1;
         // a store's bytes joining a live MSHR's merge buffer (the bytes: the one write above)
         if (fp_mg) begin
            mb_cv[sb_hk][sb_c] <= 1'b1;
            ms_dty[sb_hk] <= 1'b1;
         end
         if (iq_go) iq_rd <= iq_rd + 1'b1;
         iq_n <= iq_n + {{MB{1'b0}}, m_alloc} - {{MB{1'b0}}, iq_go};
         // a dirty line leaving or cleaning: a write-back entry, and the read-out of its line -- a
         // victim, a dropped or cleaned copy, or the clean walk's way
         // (the free entry is dead: its fields are written every cycle the read-out is idle, and
         // only wb_v and ro_on wait for the decision)
         if (~ro_on & wb_free_v) begin
            wb_rdy[wb_free] <= 1'b0;  wb_sent[wb_free] <= 1'b0;
            wb_cpl[wb_free] <= ~cl_on & (k_fl | k_cln);
            wb_pl[wb_free]  <= cl_on  ? {{(58-LB){1'b0}}, tag_of(cl_oh, tr), cl_r}
                             : absent ? {{(58-LB){1'b0}}, tag_of(vic_oh, tr), s1_set} : s1_pl;
            ro_wb  <= wb_free;
            ro_way <= cl_on ? cl_k : absent ? vic : hw;
            ro_set <= cl_on ? cl_r : s1_set;
         end
         if ((ro_need & ~ro_block & ~wb_block) | cl_ro) begin
            wb_v[wb_free] <= 1'b1;  ro_on <= 1'b1;
         end else ro_on <= 1'b0;
         rl0  <= ro_on;  rl0_way <= ro_way;  rl0_wb <= ro_wb;
         rl_v <= rl0;    rl_way  <= rl0_way; rl_wb  <= rl0_wb;
         if (rl_v) begin
            for (i = 0; i < 8; i = i + 1) wb_data[rl_wb][i*64 +: 64] <= ro_grp[((rl_way[2:0] + i) & 7)*64 +: 64];
            wb_rdy[rl_wb] <= 1'b1;
         end
         if (ws_go) wb_sent[ws] <= 1'b1;
         if (cw_ok) wb_v[cw_e[WBB-1:0]] <= 1'b0;
         // the NC slot: taken, sent, answered or completed
         if (m_nc) begin
            nc_v <= 1'b1;  nc_we <= s1_st;  nc_sent <= 1'b0;  nc_pl <= s1_pl;  nc_pa <= s1_pa;
            nc_tag <= tg(s1_tag);  nc_wd <= st_wd;  nc_wm <= st_wm;
         end
         if (nq_go) nc_sent <= 1'b1;
         if (cr_valid & (cr_slot == NCS) & (cr_beat == nc_pa[5:4])) nc_data <= nc_pa[3] ? cr_data[127:64] : cr_data[63:0];
         if (cr_valid & (cr_slot == NCS) & cr_last) nc_rdy <= 1'b1;
         if (nc_done) begin nc_v <= 1'b0;  nc_rdy <= 1'b0; end
         // a fill beat: write the reserved way's two chunks; the last installs (the state writes above) and releases
         fb_v <= cr_f;  fb_last <= cr_f & cr_last;  fb_m <= cr_m;
         fb_aw <= cr_f ? (({{(WAYS-1){1'b0}}, 1'b1} << arr_of(ms_way[cr_m], {cr_beat, 1'b0}))
                        | ({{(WAYS-1){1'b0}}, 1'b1} << arr_of(ms_way[cr_m], {cr_beat, 1'b1}))) : {WAYS{1'b0}};
         wa_e <= cr_f ? {ms_set[cr_m], cr_beat,  ms_way[cr_m][0]} : {sa_set, sa_pa[5:3]};
         wa_o <= cr_f ? {ms_set[cr_m], cr_beat, ~ms_way[cr_m][0]} : {sa_set, sa_pa[5:3]};
         wd_e <= cr_f ? (ms_way[cr_m][0] ? cr_hi : cr_lo) : sf_wd[sa_slot];   // SB2 writes only its own slot (the head)
         wd_o <= cr_f ? (ms_way[cr_m][0] ? cr_lo : cr_hi) : sf_wd[sa_slot];
         if (cr_f) begin
            fb_way <= ms_way[cr_m];  fb_set <= ms_set[cr_m];  fb_beat <= cr_beat;
            ms_bt[cr_m] <= cr_beat + 1'b1;  ms_bl[cr_m] <= 1'b1;
         end
         fl_p <= fb_last;  fl_v <= fl_p;
         if (fb_last) begin
            fl_m <= fb_m;  fl_set <= fb_set;  fl_woh <= {{(WAYS-1){1'b0}}, 1'b1} << fb_way;
            fl_tag <= ms_pl[fb_m][6 +: PTB];  fl_dty <= ms_dty[fb_m];
         end
         if (fl_v) ms_v[fl_m] <= 1'b0;
         // the reset scan: a set a cycle, every way's state cleared
         if (scan_on) begin
            scan <= scan + 1'b1;
            if (&scan) scan_on <= 1'b0;
         end
         // the clean walk, then the wait for its write-backs to reach memory
         if (inv_req) cl_pend <= 1'b1;
         // (it reads the state through the lookup's port: the lookup is empty and takes nothing now)
         if ((inv_req | cl_pend) & ~cl_on & ~cl_wait & (sc == 2'd0) & ~ms_dirty & ~sa_v & ~sb_v & ~s1_v & ~a_cap) begin cl_pend <= 1'b0;  cl_on <= 1'b1;  cl_r <= 6'd0; end
         else if (cl_adv) begin
            cl_r <= cl_r + 1'b1;
            if (&cl_r) begin cl_on <= 1'b0;  cl_wait <= 1'b1; end
         end
         if (cl_wait & ~ro_on & ~rl0 & ~rl_v & ~wb_any) cl_wait <= 1'b0;
         // the doors: shut during the scan and the clean
         door  <= ~scan_on & ~cl_pend & ~cl_on & ~cl_wait & ~inv_req;
         sdoor <= ~scan_on & ~cl_on & ~cl_wait;
      end
   end

   assign inv_busy = scan_on | cl_pend | cl_on | cl_wait;

   // ---------------------------------------------------------------- occupancy, for a bench's counters
   reg  [MB:0] ob_live;  reg ob_wait;               // MSHRs live; a request parked in the waiter table
   always @* begin
      ob_live = 0;  ob_wait = 1'b0;
      for (i = 0; i < NMSHR; i = i + 1) ob_live = ob_live + {{MB{1'b0}}, ms_v[i]};
      for (i = 0; i < NW; i = i + 1) if (wt_v[i] & ~wt_rdy[i]) ob_wait = 1'b1;
   end
   wire ob_wbfull = ~wb_free_v;

   // ---------------------------------------------------------------- invariants (always on)
   // Register the conditions; each is a bit of the integrity log, the D$'s bits 0-15 in the
   // numbering rv_soc_top publishes (tools/errlog-read.sh names them):
   //   0 orphan      a fill beat for no MSHR
   //   1 slot        a read response nothing awaits
   //   2 wdone       a write completion for no write sent
   //   3 beat        beats out of order within a burst
   //   4 two         two valid ways hold one line (the main lookup)
   //   5 st_two      two valid ways hold one line (the store lookup)
   //   6 tag_reuse   a requester tag reused while outstanding
   //   7 dead_wait   a waiter parked on an MSHR that is not live
   //   8 ro_race     a fill beat written to the line being read out
   //   9 st_main     a store in both lookups, or a plain store that hits or joins in the main lookup
   //  10 rsv_vic     a victim an MSHR reserves or the store lookup writes, or no unreserved way
   //  11 sq_order    a store sent back to the queue, or the head issued, as the head leaves
   //  12 st_clean    a store taken while a clean is asked for or under way (the clean leaves it out)
   //  13 fast        M's load presented while not taken, beside another read, on a busy tag, or a
   //                 cancel with no fast load in the lookup
   //  14 xblock      an outside flush finishing while a line is blocked, or a release with none
   wire e_orphan  = cr_f & ~ms_v[cr_m];
   wire e_slot    = cr_valid & ~cr_f & ~((cr_slot == NCS) & nc_v & ~nc_we & nc_sent);
   wire e_wdone   = cw_valid & ~cw_ok & ~fin_nc;
   wire e_beat    = cr_f & ms_v[cr_m] & (cr_beat != ms_bt[cr_m]);
   wire e_two     = l1_v & ((hitv & (hitv - 1'b1)) != 0);
   wire e_btwo    = sb_hd & ((bhitv & (bhitv - 1'b1)) != 0);
   wire [TB-1:0] rd_wi = wi(rd_tag), f_wi = wi(f_tag);   // the requests' waiter entries
   wire e_wtbusy  = new_go & (wt_v[rd_wi] | ((s1_v | c2_live) & (s1_tag == rd_wi)));
   // A READ NAMES A DOUBLEWORD and is answered with it as stored: the LSU aligns its loads, a
   // straddle's halves and its atomics (picking the bytes itself), the walkers read PTEs, and no
   // device is behind this port. A byte address here would come back unshifted.
   always @(posedge clk) if (!reset && ((rd_req && rd_pa[2:0] != 3'd0) || (f_req && f_pa[2:0] != 3'd0)))
      $fatal(1, "rv_dcache: a read at %h, not a doubleword's address", rd_req ? rd_pa : f_pa);
   always @(posedge clk) if (!reset && ((rd_req && ({1'b0, rd_tag} >= NRQ[RTW:0])) || (f_req && ({1'b0, f_tag} >= NRQ[RTW:0]))))
      $fatal(1, "rv_dcache: a request tag %0d / %0d past the %0d the table holds", rd_tag, f_tag, NRQ);
   reg  e_wdead;
   always @* begin
      e_wdead = 1'b0;
      for (i = 0; i < NW; i = i + 1) if (wt_v[i] & ~wt_rdy[i] & ~wt_any[i] & ~ms_v[wt_m[i]]) e_wdead = 1'b1;
   end
   wire e_rorace  = fb_v & ro_on & (fb_set == ro_set) & (fb_way == ro_way);
   wire e_wr2     = (sb_hd & (s1_v | c2_v) & s1_st) | (l1_v & k_st & (hit | ms_hit));
   wire e_vic     = m_alloc & (vbad_q | (fp_wr & sb_set1 & (|(bhitv & vic_oh))));
   wire e_sqord   = (sq_bk & st_done) | (st_go & ~si & st_done) | ((fp_wr | fp_mg) & fin_now);
   wire e_stcl    = wr_take & (inv_req | cl_pend | cl_on | cl_wait);
   wire e_fast    = (f_req & (~f_ok | ~f_hint | rd_req | wt_v[f_wi] | ((s1_v | c2_live) & (s1_tag == f_wi))))
                  | (f_cancel & ~(s1_v & s1_f));
   wire e_xb      = (st_done & sf_ox[sq_hp] & ~sf_keep[sq_hp] & xb_v) | (ox_rel & ~xb_v);
   // an MSHR's read issued while the MSHR is not live, or a second time
   wire e_iss     = iq_go & (~ms_v[iq[iq_rd]] | ms_iss[iq[iq_rd]]);
   wire [15:0] e_now = {e_iss, e_xb, e_fast, e_stcl, e_sqord, e_vic, e_wr2, e_rorace, e_wdead, e_wtbusy, e_btwo, e_two, e_beat, e_wdone, e_slot, e_orphan};
   reg  [15:0] e_q;
   always @(posedge clk) e_q <= reset ? 16'd0 : e_now;
   assign err = e_q;
   // an outside request is refused with room in the store queue at most OX_WAIT times (a simulation
   // check: ox_pri bounds it)
   reg  [7:0]  ox_rr;
   always @(posedge clk) begin
      ox_rr <= (reset | ~ox_pend | ox_take) ? 8'd0 : ox_rr + {7'd0, sc != SD};
      if (!reset && ox_pend && (sc != SD) && !ox_take && (ox_rr >= {5'd0, OX_WAIT}))
         $fatal(1, "rv_dcache: an outside request refused %0d times with room in the store queue", ox_rr + 8'd1);
   end
   // the match flops agree with the sets and lines (a simulation check: the compares are what they save)
   reg e_sm;
   always @* begin
      e_sm = 1'b0;
      for (i = 0; i < NMSHR; i = i + 1)
         if (ms_v[i] & ((ms_sm[i] != (ms_set[i] == s1_set)) | (ms_bm[i] != (ms_pl[i][LB-1:0] == sa_pa[6 +: LB])))) e_sm = 1'b1;
   end
   always @(posedge clk) if (!reset && e_sm) $fatal(1, "rv_dcache: an MSHR's set or line match flop disagrees");
   always @(posedge clk) if (!reset && (e_now != 0))
      $fatal(1, "rv_dcache: invariant %b (orphan beat, bad slot, write completion, beat order, two ways, two ways at the store lookup, tag reuse, dead waiter, read-out race, two store writes, reserved victim, store-queue order, store under a clean, fast load, outside block, MSHR issue)", e_now);

endmodule
`default_nettype wire
