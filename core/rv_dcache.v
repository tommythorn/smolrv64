`default_nettype none
// rv_dcache: the VHPR data cache (docs/PLAN-2026-09-25-dcache-vhpr.md), non-blocking, write-back,
// on the tagged memory port (rv_mem_arbiter's client side).
//
// GEOMETRY. SIZE_KB (128) in 2 ways of 64-byte lines, SETS = 1024 indexed by VA[15:6]: 16
// colours (VA[15:12]) over the 64 rows a 4 KiB page covers (VA[11:6] = PA[11:6]); a smaller
// cache has fewer colours. Each way's data
// is two block-RAM banks, the even and the odd 8-byte chunks of a line, read at {set, chunk pair}.
//
// TWO KINDS OF TAG STATE. A line is PHYSICALLY present -- pvalid, dirty, its physical tag
// PA[PABITS-1:12] and the data -- independently of whether a VIRTUAL stamp names it (vvalid, the
// virtual tag VA[VAW-1:16] and the epoch it was stamped in). The virtual tags are per way, indexed
// by the VA set; the physical tags are one array per (way, colour), 64 rows deep, all read at
// PA[11:6], so the 32 places a physical line can live come out of one read (the probe). A stamp is
// written only in the current epoch, and only on a pvalid line.
//
// THE LOOKUP takes one request a cycle: a released waiter (replay), else a committed store, else a
// load. A request taken at T reads the banks; at T+1 its way's vvalid, virtual tag and epoch are
// compared, and the probe compares the 32 candidates' physical tags with its PA. A request never
// spans an 8-byte chunk.
//   - a virtual hit, or the line physically in the request's own set (its data was read with the
//     lookup; the way is re-stamped): a load answers -- rd_valid at T+2, the addressed bytes at
//     bit 0, tagged with rd_tag -- and a store merges its bytes into the chunk it read and writes
//     the chunk back, marking the line dirty;
//   - a match in another colour (a synonym): that copy is dropped (single-copy invariant), a
//     dirty one read out to the write-back buffer, and the request replays, now finding no copy;
//   - no copy: the MSHR filling the line in the request's own set takes it -- one allocated now,
//     with a reserved way whose line is dropped (a dirty one read out to the write-back buffer)
//     and a read queued on the memory port under the MSHR's id. A load waits on the MSHR; a
//     store's bytes go into the MSHR's merge buffer and the store is done, unless a beat has
//     already landed, when it waits too (as does a cbo.zero: its line is written when the fill
//     is in, from the own copy's drop);
//   - a load to the line of the store in flight (taken, not yet written or merged) waits for it:
//     the store was accepted first, so the load must see it;
//   - no copy and no MSHR can take it (MSHRs full, the set's ways both reserved, the line filling
//     into another colour or still in the write-back buffer, no write-back entry for a dirty
//     victim): the request waits on the next MSHR or write-back entry to free.
// Waiters live in a table with one entry per requester tag and one for the store; a miss never
// holds the pipeline. The store's completion counts as a freeing; the waiters on any freeing
// replay round robin. A structural conflict (the victim read-out busy, a store's bank taking a
// fill beat this cycle) replays the request at once.
//
// THE FILL. The beats write the reserved way's banks as they arrive (a cycle after each, through
// the beat register), the merge buffer's bytes over the memory's. The lookup yields in the cycle
// the last one is written, and in the next the line installs -- the physical tag, pvalid, dirty
// if a store merged, and the virtual stamp if the MSHR's epoch is still current -- the MSHR is
// freed and its waiters are released; they replay ahead of the waiters on any resource, and hit.
//
// THE WRITE-BACK BUFFER (NWB lines). A dirty line leaving is read out over the next four cycles,
// the lookup yielding the bank read port. Its write goes to memory under slot NMSHR + entry, only
// when no read waits for the port, and the entry frees at the write's completion.
//
// A BANK ROW WRITTEN AND READ AT ONE EDGE reads invalid data, so the lookup that read the row a
// store wrote takes the store's chunk from the write register (w1). Fill beats need no bypass:
// they write a line nothing can hit until it installs.
//
// BY PA ALONE. A page-table walk (rd_phys), an NC access (rd_nc, wr_nc) and a CBO are looked up
// by their PA in its own colour (PA[15:12]): no virtual hit, no stamp.
//   - a walk is a load that never stamps; it fills into the PA's colour;
//   - an NC access drops any copy (a dirty one written back first) and then goes to memory on its
//     own through the NC slot, one at a time, a store with only its bytes; nothing allocates;
//   - cbo.clean writes a dirty copy back and keeps it, clean; cbo.flush and cbo.inval drop the
//     copy, a dirty one written back; cbo.zero drops it unwritten and fills an MSHR whose merge
//     buffer is the whole line of zeros. A CBO or an NC store completes (wr_cpl) when memory
//     has its write, or at once when there was nothing to write.
// A line still in the write-back buffer, or being filled, holds any of them off until it is done.
//
// INV_REQ (fence.i) cleans the whole cache. With the door shut, it waits until no dirty bytes sit
// outside the arrays (the store in flight, a merge buffer), then a walk of the 64 rows, each read
// across every (way, colour) at once, writes every dirty line back and keeps it; inv_busy holds
// until memory has them all.
//
// A REQUEST'S TRANSLATION holds in the epoch it is taken in: the core bumps the epoch only with
// the store queue drained (sfence.vma and satp writes serialise on it). A request taken in an
// epoch that has since passed is still served by its PA; it stamps nothing.
//
// RESPONSES are matched by the requester's tag (rule B1) and may complete out of order: a hit
// behind a miss answers first. Every taken load is answered exactly once.
//
// EPOCHS. ep_bump advances the epoch: virtual stamps of the old epoch stop hitting; the lines stay
// physically present and re-stamp from the probe. A wrap (the epoch returns to a value still
// stamped somewhere) clears every vvalid over a scan, with the door shut to new requests; the scan
// steps in cycles nothing else writes vvalid. pvalid and dirty survive.
//
// VIRT=0 presents every request by PA alone -- a PIPT cache, the virtual stamps unused -- for as
// long as the core translates before it asks (phase 1 of the plan).
//
// The contract is rv_cache's: a load is taken at rd_ack and answered once, by tag; a store is
// taken the cycle before wr_acc, and a CBO or an NC store completes at wr_cpl.
module rv_dcache #(
   parameter SIZE_KB = 128,
   parameter RTW     = 4,              // the requester's opaque tag (the LQ index, the slow tag, the walkers)
   parameter VAW     = 39,             // Sv39
   parameter PABITS  = 36,             // significant physical address bits
   parameter NMSHR   = 8,
   parameter NWB     = 2,              // write-back buffer lines
   parameter SW      = 4,              // memory-port slot bits (this client's ids)
   parameter VIRT    = 1               // 0: every request by PA
) (
   input  wire              clk,
   input  wire              reset,
   // ---- loads
   input  wire              rd_req,
   input  wire [63:0]       rd_va,
   input  wire [63:0]       rd_pa,
   input  wire [RTW-1:0]    rd_tag,
   input  wire              rd_phys,     // a page-table walk: by PA alone
   input  wire              rd_nc,       // Svpbmt NC/IO: no allocation
   output wire              rd_ack,      // taken this cycle
   output reg               rd_valid,
   output reg  [63:0]       rd_data,     // the addressed bytes at bit 0
   output reg  [RTW-1:0]    rd_resp_tag,
   output reg  [63:0]       rd_resp_addr,// the PA the response answers
   // ---- committed stores: the data sits at its byte lanes of the aligned 8-byte chunk
   input  wire              wr_req,
   input  wire [63:0]       wr_va,
   input  wire [63:0]       wr_pa,
   input  wire [63:0]       wr_data,
   input  wire [7:0]        wr_mask,
   input  wire              wr_nc,       // Svpbmt NC/IO store
   input  wire              cbo_req,     // a CBO by PA: cbo_zero, else cbo_keep (clean), else flush/inval
   input  wire              cbo_zero,
   input  wire              cbo_keep,
   output reg               wr_acc,      // taken last cycle: a plain store completes on its own
   output reg               wr_cpl,      // an NC store's or a CBO's completion
   // ---- epochs
   input  wire              ep_bump,
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
   localparam OFFB  = 6;
   localparam SETS  = (SIZE_KB * 1024) / (2 * 64);
   localparam IB    = $clog2(SETS);               // 10: VA[15:6]
   localparam COLB  = IB - 6;                     // colour bits: VA[15:12] at 128 KiB
   localparam NCOL  = 1 << COLB;
   localparam VTB   = VAW - OFFB - IB;            // virtual tag VA[38:16]
   localparam PTB   = PABITS - 12;                // physical tag PA[35:12]
   localparam LB    = PABITS - OFFB;              // significant line-address bits
   localparam EPW   = 2;
   localparam RB    = IB + 2;                     // bank row {set, chunk pair}
   localparam MB    = $clog2(NMSHR);
   localparam WBB   = (NWB > 1) ? $clog2(NWB) : 1;
   localparam NTAG  = 1 << RTW;
   localparam NW    = NTAG + 1;                   // waiter entries: the tags, then the store
   localparam TB    = RTW + 1;
   localparam [TB-1:0] ST = NTAG;                 // the store's waiter entry
   initial begin
      if (SETS < 128 || (SETS & (SETS - 1)) != 0) $fatal(1, "rv_dcache: %0d sets: a power of two, at least two colours", SETS);
      if (NMSHR + NWB > (1 << SW)) $fatal(1, "rv_dcache: %0d MSHRs and %0d write-backs need more than %0d slot bits", NMSHR, NWB, SW);
   end
   function [63:0] bytes(input [7:0] m);          // a byte mask as a bit mask
      integer b;
      for (b = 0; b < 8; b = b + 1) bytes[b*8 +: 8] = {8{m[b]}};
   endfunction

   // ---------------------------------------------------------------- the arrays
   // virtual stamps, per way, by VA set (distributed RAM, one write statement each)
   reg [VTB-1:0] vt0 [0:SETS-1], vt1 [0:SETS-1];
   reg [EPW-1:0] ep0 [0:SETS-1], ep1 [0:SETS-1];
   reg           vv0 [0:SETS-1], vv1 [0:SETS-1];
   reg           wrr [0:SETS-1];                  // round-robin victim
   // physical state: one distributed RAM per (way, colour) and field, 64 rows by PA[11:6], each
   // written once a cycle and read at two rows (the ptw block below); here, all of them as read
   wire [2*NCOL-1:0]     pv_r, pd_r;               // pvalid, dirty at the lookup's row
   wire [2*NCOL*PTB-1:0] pt_r;                     // the physical tags there
   wire [2*NCOL-1:0]     pv_c, pd_c;               // ...and at the clean walk's row
   wire [2*NCOL*PTB-1:0] pt_c;
   integer i;
   initial for (i = 0; i < SETS; i = i + 1) begin vv0[i] = 1'b0; vv1[i] = 1'b0; wrr[i] = 1'b0; end
   reg [EPW-1:0] cur_ep;

   // ---------------------------------------------------------------- the MSHRs
   reg              ms_v    [0:NMSHR-1];
   reg [57:0]       ms_pl   [0:NMSHR-1];         // the physical line
   reg [IB-1:0]     ms_set  [0:NMSHR-1];         // the VA set it fills
   reg [VTB-1:0]    ms_vt   [0:NMSHR-1];         // the virtual tag it stamps
   reg [EPW-1:0]    ms_ep   [0:NMSHR-1];         // ...in this epoch
   reg              ms_way  [0:NMSHR-1];
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
   reg [63:0]       wt_va   [0:NW-1], wt_pa [0:NW-1];
   reg [EPW-1:0]    wt_ep   [0:NW-1];
   reg              wt_ph   [0:NW-1], wt_nc [0:NW-1];
   // the store-port op in flight: taken, not yet written, merged or complete
   reg              st_busy;
   reg [63:0]       st_wd;  reg [7:0] st_wm;  reg [LB-1:0] st_pl;
   reg              st_cbo, st_zero, st_keep;

   // ---------------------------------------------------------------- the write-back buffer
   reg              wb_v    [0:NWB-1];
   reg              wb_rdy  [0:NWB-1];           // read out: may go to memory
   reg              wb_sent [0:NWB-1];
   reg              wb_cpl  [0:NWB-1];           // a CBO completes with this write
   reg [57:0]       wb_pl   [0:NWB-1];
   reg [511:0]      wb_data [0:NWB-1];
   // the victim read-out: four bank reads of (ro_way, ro_set), each landing a cycle later
   reg              ro_on;  reg [1:0] ro_cnt;  reg ro_way;  reg [IB-1:0] ro_set;  reg [WBB-1:0] ro_wb;
   reg              rl_v;   reg [1:0] rl_row;  reg rl_way;  reg [WBB-1:0] rl_wb;  reg rl_last;

   // ---------------------------------------------------------------- the fill (declared here: the lookup yields to it)
   reg            fb_v;  reg fb_way;  reg [RB-1:0] fb_row;  reg [127:0] fb_data;   // the beat being written
   reg            fb_last;  reg [MB-1:0] fb_m;
   reg            fl_v;  reg [MB-1:0] fl_m;                                       // the line installing
   reg            ms_ph   [0:NMSHR-1];           // filled for a by-PA request: installs unstamped

   // ---------------------------------------------------------------- the NC slot: one access at a time
   localparam [SW-1:0] NMS = NMSHR;              // the first write-back slot
   localparam [SW-1:0] NWS = NWB;
   localparam [SW-1:0] NCS = NMSHR + NWB;        // the NC slot's memory-port slot
   reg            nc_v, nc_we, nc_sent, nc_rdy;
   reg  [57:0]    nc_pl;  reg [63:0] nc_pa;  reg [RTW-1:0] nc_tag;  reg [63:0] nc_wd, nc_data;  reg [7:0] nc_wm;
   // ---------------------------------------------------------------- the clean walk (inv_req)
   reg            cl_pend, cl_on, cl_wait;  reg [5:0] cl_r;   // the row it is at

   // ---------------------------------------------------------------- the lookup input
   reg        door;                               // registered: open unless a scan runs
   reg        s1_v;  reg [63:0] s1_va, s1_pa;  reg [TB-1:0] s1_tag;  reg [EPW-1:0] s1_ep;  reg s1_ph, s1_nc;
   // the replay pick: the lowest waiter released by its MSHR, a drop or a conflict, else the first
   // released by any resource freeing at or after ra_ptr, round robin -- each freeing hands its
   // resource to whoever replays first, so a fixed order would starve the last entries
   reg              rm_v, ra_v, ra_hv;  reg [TB-1:0] rm_t, ra_t, ra_ht;
   reg  [TB-1:0]    ra_ptr;
   always @* begin
      rm_v = 1'b0;  rm_t = {TB{1'b0}};  ra_v = 1'b0;  ra_t = {TB{1'b0}};  ra_hv = 1'b0;  ra_ht = {TB{1'b0}};
      for (i = NW - 1; i >= 0; i = i - 1) begin
         if (wt_v[i] & wt_rdy[i] & ~wt_any[i]) begin rm_v = 1'b1; rm_t = i[TB-1:0]; end
         if (wt_v[i] & wt_rdy[i] &  wt_any[i]) begin
            ra_v = 1'b1;  ra_t = i[TB-1:0];
            if (i[TB-1:0] >= ra_ptr) begin ra_hv = 1'b1;  ra_ht = i[TB-1:0]; end
         end
      end
      if (ra_hv) ra_t = ra_ht;
   end
   // the lookup yields in the cycle a line's last beat is written (so none is in flight while it
   // installs), while the victim read-out owns the bank read port, while an NC answer waits for
   // the response port and while the clean walk runs
   wire           hold   = fb_last | ro_on | nc_rdy | cl_on;
   wire           rp_v   = (rm_v | ra_v) & ~hold;
   wire [TB-1:0]  rp_t   = rm_v ? rm_t : ra_t;
   wire           st_go  = ~(rm_v | ra_v) & ~hold & door & wr_req & ~st_busy;
   wire           new_go = ~(rm_v | ra_v) & ~hold & door & ~st_go & rd_req;
   assign rd_ack = new_go;
   wire           a_take = rp_v | st_go | new_go;
   wire           a_ph  = (VIRT == 0) | (rp_v ? wt_ph[rp_t] : st_go ? (wr_nc | cbo_req) : (rd_phys | rd_nc));
   wire           a_nc  = rp_v ? wt_nc[rp_t] : st_go ? (wr_nc & ~cbo_req) : rd_nc;
   wire [63:0]    a_pa  = rp_v ? wt_pa[rp_t] : st_go ? wr_pa : rd_pa;
   wire [63:0]    a_va  = a_ph ? a_pa : rp_v ? wt_va[rp_t] : st_go ? wr_va : rd_va;   // by PA: the PA's own colour
   wire [TB-1:0]  a_tag = rp_v ? rp_t        : st_go ? ST    : {1'b0, rd_tag};
   wire [EPW-1:0] a_ep  = rp_v ? wt_ep[rp_t] : cur_ep;

   // ---------------------------------------------------------------- the data banks
   // bank = way*2 + chunk parity; row = {set, chunk pair}. The read port serves the lookup and the
   // victim read-out; the write port the fill beats and the store writes.
   wire [RB-1:0]  bk_ra = ro_on ? {ro_set, ro_cnt} : {a_va[OFFB +: IB], a_va[5:4]};
   reg  [RB-1:0]  s1_ra;                          // the row read at the last edge
   wire           sw_v;  wire [1:0] sw_bank;  wire [RB-1:0] sw_row;  wire [63:0] sw_data;   // the store write
   reg            w1_v;  reg  [1:0] w1_bank;  reg  [RB-1:0] w1_row;  reg  [63:0] w1_data;   // ...registered: the bypass
   wire [63:0]    bk_rd [0:3];
   wire [63:0]    bk_q  [0:3];                    // the banks as read, the store written at that edge over them
   genvar gb;
   generate for (gb = 0; gb < 4; gb = gb + 1) begin : bank
      wire fw = fb_v & (fb_way == gb[1]);
      wire sw = sw_v & (sw_bank == gb[1:0]);
      smolrv64_sdpram #(.ADDR_WIDTH(RB), .DATA_WIDTH(64), .READ_LATENCY(1)) u_bank
        (.clock(clk), .rd_addr(bk_ra), .rd_data(bk_rd[gb]),
         .wr_en(fw | sw), .wr_addr(fw ? fb_row : sw_row), .wr_data(fw ? fb_data[gb[0]*64 +: 64] : sw_data));
      assign bk_q[gb] = (w1_v & (w1_bank == gb[1:0]) & (w1_row == s1_ra)) ? w1_data : bk_rd[gb];
   end endgenerate

   // ---------------------------------------------------------------- the lookup (T+1)
   wire           s1_st  = s1_tag[RTW];
   wire [IB-1:0]  s1_set = s1_va[OFFB +: IB];
   wire [VTB-1:0] s1_vtg = s1_va[OFFB+IB +: VTB];
   wire [PTB-1:0] s1_ptg = s1_pa[12 +: PTB];
   wire [5:0]     s1_row = s1_pa[11:6];
   wire [COLB-1:0] s1_col = s1_va[12 +: COLB];
   wire [57:0]    s1_pl  = s1_pa[63:OFFB];
   wire           cur    = (s1_ep == cur_ep);     // the request's epoch is current: it may stamp
   wire h0  = ~s1_ph & vv0[s1_set] & (vt0[s1_set] == s1_vtg) & (ep0[s1_set] == s1_ep);
   wire h1  = ~s1_ph & vv1[s1_set] & (vt1[s1_set] == s1_vtg) & (ep1[s1_set] == s1_ep);
   // what the request is: a cached load (a walk included) or store, an NC access, a CBO
   wire k_cbo = s1_st & st_cbo;
   wire k_z   = k_cbo & st_zero;
   wire k_cln = k_cbo & ~st_zero & st_keep;
   wire k_fl  = k_cbo & ~st_zero & ~st_keep;
   wire k_nc  = s1_nc;
   wire k_ld  = ~s1_st & ~s1_nc;
   wire k_st  = s1_st & ~k_cbo & ~s1_nc;
   // a load behind the store-port op in flight, to its line: it waits, and nothing else happens to it
   wire blk_st = s1_v & ~s1_tag[RTW] & st_busy & (s1_pa[OFFB +: LB] == st_pl);
   wire l1_v   = s1_v & ~blk_st;                  // the lookup proceeds
   wire mis    = l1_v & ~(h0 | h1);

   // the probe: the 32 candidates at PA[11:6] (array k = way*NCOL + colour)
   reg  [2*NCOL-1:0] pm;                          // candidate k holds the request's physical line
   always @* for (i = 0; i < 2*NCOL; i = i + 1) pm[i] = pv_r[i] & (pt_r[i*PTB +: PTB] == s1_ptg);
   wire [COLB:0]     k_own0 = {1'b0, s1_col};     // the request's own set's two candidates
   wire [COLB:0]     k_own1 = {1'b1, s1_col};
   wire              own0  = pm[k_own0];
   wire              own1  = pm[k_own1];
   // (every request by PA, VIRT=0: a line lives only in its PA's colour, so there is no other)
   wire [2*NCOL-1:0] other = (VIRT == 0) ? {2*NCOL{1'b0}}
                           : pm & ~(({{(2*NCOL-1){1'b0}}, 1'b1} << k_own0) | ({{(2*NCOL-1){1'b0}}, 1'b1} << k_own1));
   // the other copy's (way, colour): at most one pvalid copy exists (asserted), so a priority pick names it
   reg  [COLB:0] oth_k;  reg oth_v;
   always @* begin
      oth_v = 1'b0; oth_k = 0;
      for (i = 2*NCOL - 1; i >= 0; i = i - 1) if (other[i]) begin oth_v = 1'b1; oth_k = i[COLB:0]; end
   end
   // the copy, wherever it is: the own set's first, else the other colour's
   wire              cp_own = own0 | own1;
   wire              cp_any = cp_own | oth_v;
   wire [COLB:0]     cp_k   = own0 ? k_own0 : own1 ? k_own1 : oth_k;
   wire              cp_dty = pd_r[cp_k];
   wire [IB-1:0]     cp_set = {cp_k[COLB-1:0], s1_row};

   // the answer: a virtual hit, or the line physically in the request's own set
   wire        srv = l1_v & (h0 | h1 | cp_own) & (k_ld | k_st);   // served in place
   wire        ans = srv & k_ld;
   wire        aw1 = (h0 | h1) ? h1 : own1;
   wire [63:0] ans_chunk = aw1 ? (s1_va[3] ? bk_q[3] : bk_q[2]) : (s1_va[3] ? bk_q[1] : bk_q[0]);
   // counted once per request, never per replay: an access is a load or store taken, a miss a line
   // fill started
   assign perf_access = new_go | st_go;
   assign perf_miss   = m_alloc;

   // MSHR match (the line already being filled) and a free MSHR
   reg  [NMSHR-1:0] ms_hitv;  reg ms_set_busy0, ms_set_busy1, ms_dirty;
   reg  [MB-1:0]    ms_hk, ms_free;  reg ms_free_v;
   always @* begin
      ms_hitv = 0;  ms_set_busy0 = 1'b0;  ms_set_busy1 = 1'b0;  ms_hk = 0;  ms_free = 0;  ms_free_v = 1'b0;  ms_dirty = 1'b0;
      for (i = NMSHR - 1; i >= 0; i = i - 1) begin
         if (ms_v[i] & (ms_pl[i][LB-1:0] == s1_pl[LB-1:0])) begin ms_hitv[i] = 1'b1; ms_hk = i[MB-1:0]; end
         if (ms_v[i] & (ms_set[i] == s1_set) & ~ms_way[i]) ms_set_busy0 = 1'b1;
         if (ms_v[i] & (ms_set[i] == s1_set) &  ms_way[i]) ms_set_busy1 = 1'b1;
         if (~ms_v[i]) begin ms_free = i[MB-1:0]; ms_free_v = 1'b1; end
         if (ms_v[i] & ms_dty[i]) ms_dirty = 1'b1;
      end
   end
   // the write-back buffer: the line still in it, and a free entry
   reg              wb_hit, wb_free_v, wb_any;  reg [WBB-1:0] wb_free;
   always @* begin
      wb_hit = 1'b0;  wb_free_v = 1'b0;  wb_free = 0;  wb_any = 1'b0;
      for (i = NWB - 1; i >= 0; i = i - 1) begin
         if (wb_v[i] & (wb_pl[i][LB-1:0] == s1_pl[LB-1:0])) wb_hit = 1'b1;
         if (~wb_v[i]) begin wb_free = i[WBB-1:0]; wb_free_v = 1'b1; end
         if (wb_v[i]) wb_any = 1'b1;
      end
   end
   // the way a new MSHR reserves: an invalid way (vvalid implies pvalid), else the round-robin
   // one -- not one another MSHR holds
   wire v0_free = ~pv_r[k_own0];
   wire v1_free = ~pv_r[k_own1];
   wire pick1   = ms_set_busy0 ? 1'b1 : ms_set_busy1 ? 1'b0 : v0_free ? 1'b0 : v1_free ? 1'b1 : wrr[s1_set];
   wire way_ok  = ~(ms_set_busy0 & ms_set_busy1);
   wire [COLB:0] k_vic  = {pick1, s1_col};
   wire          vic_dty = pv_r[k_vic] & pd_r[k_vic];

   // ---------------------------------------------------------------- what the lookup does (one of these)
   wire m_own     = mis & cp_own & (k_ld | k_st);                     // served from the own set
   // a copy goes: another colour's under a cached access, any under an NC access, a flush or a zero
   wire drop_c    = l1_v & ~srv & ((oth_v & (k_ld | k_st)) | (cp_any & (k_nc | k_fl | k_z)));
   wire drop_wb   = cp_dty & ~k_z;                                    // ...written back if dirty, unless zeroed
   wire cln_c     = l1_v & k_cln & cp_any;                            // cbo.clean on a copy
   wire absent    = l1_v & ~(h0 | h1) & ~cp_any;
   wire free_ln   = absent & ~wb_hit & ~(|ms_hitv);                   // not cached, filling or being written
   wire alloc_c   = free_ln & (k_ld | k_st | k_z) & ms_free_v & way_ok;
   wire ro_need   = (drop_c & drop_wb) | (alloc_c & vic_dty) | (cln_c & cp_dty);   // a dirty line leaves or cleans
   wire ro_block  = ro_need & ro_on;
   wire wb_block  = ro_need & ~ro_on & ~wb_free_v;
   wire sw_block  = srv & k_st & fb_v & (fb_way == aw1);              // the store's bank is taking a beat
   wire m_drop    = drop_c & ~ro_block & ~wb_block;
   wire m_alloc   = alloc_c & ~ro_block & ~wb_block;
   wire m_clean   = cln_c & ~ro_block & ~wb_block;
   // (no copy is pvalid while an MSHR fills its line, so the join needs no probe)
   wire m_join    = mis & ~wb_hit & (|ms_hitv) & (ms_set[ms_hk] == s1_set) & (k_ld | k_st | k_z);
   wire mg_ok     = ~ms_bl[ms_hk] & ~(cr_f & (cr_m == ms_hk));        // no beat has landed
   wire m_smerge  = m_join & k_st & mg_ok;                            // a store's bytes into the MSHR: done (a zero waits)
   wire m_merge   = m_join & ~m_smerge;                               // wait on the MSHR
   wire m_nc      = free_ln & k_nc & ~nc_v;                           // to memory through the NC slot
   wire m_cdone   = free_ln & (k_cln | k_fl);                         // a CBO with nothing cached: done
   wire m_defer   = ro_block | sw_block;                              // replay at once
   wire s_write   = srv & k_st & ~sw_block;                           // a store writes its chunk
   wire m_wait    = blk_st | (l1_v & ~ans & ~s_write & ~m_defer & ~m_drop & ~m_alloc & ~m_clean & ~m_join & ~m_nc & ~m_cdone);
   wire t_rst     = m_own & ~s1_ph & ~sw_block & cur;                 // re-stamp the own way
   // the store-port op finishes: now, or when memory has its write (a CBO's write-back, an NC store)
   wire fin_now   = s1_st & (s_write | m_smerge | m_alloc | (m_clean & ~cp_dty) | (m_drop & k_fl & ~drop_wb) | m_cdone);
   wire fin_wb    = cw_ok & wb_cpl[cw_e[WBB-1:0]];
   wire fin_nc    = cw_valid & (cw_slot == NCS) & nc_v & nc_we & nc_sent;
   wire st_done   = fin_now | fin_wb | fin_nc;
   wire nc_done   = fin_nc | (nc_rdy & ~ans);                         // the NC slot frees

   // the store write: its bytes over the chunk it read
   assign sw_v    = s_write;
   assign sw_bank = {aw1, s1_va[3]};
   assign sw_row  = {s1_set, s1_va[5:4]};   // (s1_set is the own set: a hit or the own copy)
   assign sw_data = (ans_chunk & ~bytes(st_wm)) | (st_wd & bytes(st_wm));

   // ---------------------------------------------------------------- the fill
   // Beats land in order of their MSHR's burst (beat 0..3); each writes one row of the reserved way,
   // the merge buffer's bytes over the memory's.
   wire          cr_f   = cr_valid & (cr_slot < NMS);           // a fill beat (else the NC slot's)
   wire [MB-1:0] cr_m   = cr_slot[MB-1:0];
   wire [MB+1:0] cr_a   = {cr_m, cr_beat};
   wire [7:0]    cr_me  = mb_cv[cr_m][{cr_beat, 1'b0}] ? mbm_e[cr_a] : 8'd0;
   wire [7:0]    cr_mo  = mb_cv[cr_m][{cr_beat, 1'b1}] ? mbm_o[cr_a] : 8'd0;
   wire [127:0]  cr_bs  = mb_z[cr_m] ? 128'd0 : cr_data;
   wire [63:0]   cr_lo  = (cr_bs[63:0]   & ~bytes(cr_me)) | (mbd_e[cr_a] & bytes(cr_me));
   wire [63:0]   cr_hi  = (cr_bs[127:64] & ~bytes(cr_mo)) | (mbd_o[cr_a] & bytes(cr_mo));

   // THE ONE MERGE-BUFFER WRITE: a store's bytes over what the MSHR it joins already holds, or
   // fresh into the free MSHR (dead unless this store allocates it)
   wire           mw_join = |ms_hitv;
   wire [MB-1:0]  mw_m    = mw_join ? ms_hk : ms_free;
   wire [MB+1:0]  mw_a    = {mw_m, s1_va[5:4]};
   wire           mw_old  = mw_join & mb_cv[mw_m][s1_va[5:3]];
   wire [63:0]    mw_od   = s1_va[3] ? mbd_o[mw_a] : mbd_e[mw_a];
   wire [7:0]     mw_om   = s1_va[3] ? mbm_o[mw_a] : mbm_e[mw_a];
   wire [7:0]     mw_mk   = (mw_old ? mw_om : 8'd0) | st_wm;
   wire [63:0]    mw_d    = mw_old ? ((mw_od & ~bytes(st_wm)) | (st_wd & bytes(st_wm))) : st_wd;
   wire           mw_we   = s1_v & k_st & (mw_join ? m_smerge : ms_free_v);
   always @(posedge clk) begin
      if (mw_we & ~s1_va[3]) begin mbd_e[mw_a] <= mw_d;  mbm_e[mw_a] <= mw_mk; end
      if (mw_we &  s1_va[3]) begin mbd_o[mw_a] <= mw_d;  mbm_o[mw_a] <= mw_mk; end
   end

   // ---------------------------------------------------------------- the memory port
   // Reads first: an allocated MSHR's read waits in the issue queue, in allocation order; then the
   // NC access; a write-back goes when neither waits.
   reg  [MB-1:0]   iq [0:NMSHR-1];
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
   wire [SW-1:0] cw_e = cw_slot - NMS;
   wire   cw_ok    = cw_valid & (cw_slot >= NMS) & (cw_e < NWS) & wb_v[cw_e[WBB-1:0]] & wb_sent[cw_e[WBB-1:0]];
   wire   freed    = fl_v | cw_ok | st_done | nc_done;   // an MSHR, a write-back entry, the store or the NC slot frees

   // ---------------------------------------------------------------- epochs and the scan
   reg  [IB:0] scan;
   reg         scan_on;
   // a wrap: the epoch after the bump is still stamped on some line -- clear every vvalid first
   wire ep_wrap = ep_bump & (cur_ep == {EPW{1'b1}});

   // ---------------------------------------------------------------- the tag writes
   // ONE write statement per array, its {we, addr, data} muxed first. The lookup is empty while a
   // line installs, so an install and a lookup outcome never meet; the scan steps only in cycles
   // nothing else writes vvalid.
   wire [MB-1:0]  im      = fl_m;                       // the MSHR installing
   wire           t_inst  = fl_v;
   wire           inst_cur = (ms_ep[im] == cur_ep) & ~ms_ph[im];
   wire           rst_w1  = ~own0;                      // the re-stamped way
   wire           t_evict = m_alloc;                    // clears the reserved way in the request's set
   wire           cp_w1   = cp_k[COLB];                 // the dropped or cleaned copy's way
   wire           vv_other = t_inst | t_rst | t_evict | m_drop;
   wire           scan_w  = scan_on & ~vv_other;
   // each way's virtual state
   wire           vw0_we = scan_w | (t_inst & ~ms_way[im]) | (t_rst & ~rst_w1) | (t_evict & ~pick1) | (m_drop & ~cp_w1);
   wire           vw1_we = scan_w | (t_inst &  ms_way[im]) | (t_rst &  rst_w1) | (t_evict &  pick1) | (m_drop &  cp_w1);
   wire [IB-1:0]  vw_a   = scan_w ? scan[IB-1:0] : t_inst ? ms_set[im] : m_drop ? cp_set : s1_set;
   wire           vw_d   = scan_w ? 1'b0 : t_inst ? inst_cur : t_rst;
   wire           tw0_we = (t_inst & ~ms_way[im] & inst_cur) | (t_rst & ~rst_w1);
   wire           tw1_we = (t_inst &  ms_way[im] & inst_cur) | (t_rst &  rst_w1);
   wire [IB-1:0]  tw_a   = t_inst ? ms_set[im] : s1_set;
   wire [VTB-1:0] tw_vt  = t_inst ? ms_vt[im] : s1_vtg;
   wire [EPW-1:0] tw_ep  = t_inst ? ms_ep[im] : s1_ep;
   always @(posedge clk) begin
      if (vw0_we) vv0[vw_a] <= vw_d;
      if (vw1_we) vv1[vw_a] <= vw_d;
      if (tw0_we) begin vt0[tw_a] <= tw_vt; ep0[tw_a] <= tw_ep; end
      if (tw1_we) begin vt1[tw_a] <= tw_vt; ep1[tw_a] <= tw_ep; end
      if (t_inst) wrr[ms_set[im]] <= ~ms_way[im];
   end
   // the clean walk goes by ROW: the row's 2*NCOL slots are read at once, the lowest dirty one is
   // written back (with the lookup empty, no install, the read-out and a write-back entry free),
   // and the walk moves on when the row is clean -- 64 steps and one per dirty line
   wire [2*NCOL-1:0] cl_dv = pv_c & pd_c;
   reg  [COLB:0]  cl_k;
   always @* begin
      cl_k = 0;
      for (i = 2*NCOL - 1; i >= 0; i = i - 1) if (cl_dv[i]) cl_k = i[COLB:0];
   end
   wire           cl_ro   = cl_on & (|cl_dv) & ~s1_v & ~t_inst & ~ro_on & wb_free_v;
   wire           cl_adv  = cl_on & ~(|cl_dv) & ~s1_v;
   // the physical state: one write statement per (way, colour) array and field
   genvar gk;
   generate for (gk = 0; gk < 2*NCOL; gk = gk + 1) begin : ptw
      (* ram_style = "distributed" *) reg [PTB-1:0] pt_k [0:63];
      (* ram_style = "distributed" *) reg           pv_k [0:63];
      (* ram_style = "distributed" *) reg           pd_k [0:63];
      integer r;
      initial for (r = 0; r < 64; r = r + 1) begin pv_k[r] = 1'b0; pd_k[r] = 1'b0; end
      assign pv_r[gk] = pv_k[s1_row];  assign pd_r[gk] = pd_k[s1_row];  assign pt_r[gk*PTB +: PTB] = pt_k[s1_row];
      assign pv_c[gk] = pv_k[cl_r];    assign pd_c[gk] = pd_k[cl_r];    assign pt_c[gk*PTB +: PTB] = pt_k[cl_r];
      wire inst_k  = t_inst  & ({ms_way[im], ms_set[im][IB-1:6]} == gk);
      wire evict_k = t_evict & (k_vic == gk);
      wire drop_k  = m_drop  & (cp_k == gk);
      wire dirty_k = s_write & ({aw1, s1_col} == gk);
      wire clean_k = ((m_clean & cp_dty & (cp_k == gk)) | (cl_ro & (cl_k == gk)));
      wire [5:0] row_k = inst_k ? ms_set[im][5:0] : cl_ro ? cl_r : s1_row;
      always @(posedge clk) begin
         if (inst_k | evict_k | drop_k)  pv_k[row_k] <= inst_k;
         if (inst_k)                     pt_k[row_k] <= ms_pl[im][6 +: PTB];
         if (inst_k | dirty_k | clean_k) pd_k[row_k] <= inst_k ? ms_dty[im] : dirty_k;
      end
   end endgenerate

   // ---------------------------------------------------------------- the machine
   always @(posedge clk) begin
      if (reset) begin
         cur_ep <= {EPW{1'b0}};  door <= 1'b0;  s1_v <= 1'b0;  rd_valid <= 1'b0;  wr_acc <= 1'b0;  wr_cpl <= 1'b0;
         nc_v <= 1'b0;  nc_rdy <= 1'b0;  cl_pend <= 1'b0;  cl_on <= 1'b0;  cl_wait <= 1'b0;
         scan_on <= 1'b1;  scan <= {(IB+1){1'b0}};  fl_v <= 1'b0;  st_busy <= 1'b0;  ra_ptr <= {TB{1'b0}};  w1_v <= 1'b0;
         iq_n <= 0;  iq_rd <= 0;  iq_wr <= 0;  fb_v <= 1'b0;  fb_last <= 1'b0;  ro_on <= 1'b0;  rl_v <= 1'b0;
         for (i = 0; i < NMSHR; i = i + 1) ms_v[i] <= 1'b0;
         for (i = 0; i < NW; i = i + 1)    begin wt_v[i] <= 1'b0; wt_rdy[i] <= 1'b0; end
         for (i = 0; i < NWB; i = i + 1)   wb_v[i] <= 1'b0;
      end else begin
         // the lookup stage takes whatever the banks were addressed for
         s1_v <= a_take;  s1_ra <= bk_ra;
         if (a_take) begin s1_va <= a_va; s1_pa <= a_pa; s1_tag <= a_tag; s1_ep <= a_ep; s1_ph <= a_ph; s1_nc <= a_nc; end
         if (rp_v) wt_v[rp_t] <= 1'b0;                  // a replaying waiter leaves the table
         if (rp_v & ~rm_v) ra_ptr <= (rp_t == ST) ? {TB{1'b0}} : rp_t + 1'b1;
         // the store port
         wr_acc <= st_go;
         wr_cpl <= st_done & st_cbo | fin_nc;
         if (st_go) begin
            st_busy <= 1'b1;  st_wd <= wr_data;  st_wm <= wr_mask;  st_pl <= wr_pa[OFFB +: LB];
            st_cbo <= cbo_req;  st_zero <= cbo_zero;  st_keep <= cbo_keep;
         end else if (st_done) st_busy <= 1'b0;
         // a load's answer: from the lookup, else the NC slot's
         rd_valid <= ans | nc_rdy;
         if (ans) begin
            rd_data      <= ans_chunk >> (8 * s1_va[2:0]);
            rd_resp_tag  <= s1_tag[RTW-1:0];  rd_resp_addr <= s1_pa;
         end else if (nc_rdy) begin
            rd_data      <= nc_data >> (8 * nc_pa[2:0]);
            rd_resp_tag  <= nc_tag;  rd_resp_addr <= nc_pa;
         end
         // a store's write, for the bypass
         w1_v <= sw_v;  w1_bank <= sw_bank;  w1_row <= sw_row;  w1_data <= sw_data;
         // the release: the installing MSHR's waiters, and on any freeing every waiter on any
         for (i = 0; i < NW; i = i + 1)
            if (wt_v[i] & ((freed & wt_any[i]) | (fl_v & ~wt_any[i] & (wt_m[i] == fl_m)))) wt_rdy[i] <= 1'b1;
         // THE STATE A DECISION FILLS IS WRITTEN WHETHER OR NOT IT IS TAKEN, wherever writing it is
         // dead: the request in the lookup owns its waiter entry (it left the table to get there,
         // or it is new), and the free MSHR is not live. Only the few bits that make an entry or
         // an MSHR live wait for the probe's decision.
         if (s1_v) begin
            wt_va[s1_tag] <= s1_va;  wt_pa[s1_tag] <= s1_pa;  wt_ep[s1_tag] <= s1_ep;
            wt_ph[s1_tag] <= s1_ph;  wt_nc[s1_tag] <= s1_nc;
         end
         if (s1_v & ms_free_v) begin
            ms_pl[ms_free] <= s1_pl;  ms_set[ms_free] <= s1_set;  ms_vt[ms_free] <= s1_vtg;  ms_ep[ms_free] <= s1_ep;
            ms_way[ms_free] <= pick1;  ms_bt[ms_free] <= 2'd0;  ms_bl[ms_free] <= 1'b0;  ms_ph[ms_free] <= s1_ph;
            ms_dty[ms_free] <= k_st | k_z;
            // its merge buffer: a store's chunk (written above), a zero's whole line, else empty
            mb_cv[ms_free] <= k_st ? (8'd1 << s1_va[5:3]) : 8'd0;
            mb_z[ms_free]  <= k_z;
         end
         iq[iq_wr] <= ms_free;
         // a request enters the waiter table
         if ((m_drop & ~k_fl) | m_defer | m_merge | m_wait | (m_alloc & k_ld)) begin
            wt_v[s1_tag]   <= 1'b1;
            wt_rdy[s1_tag] <= m_drop | m_defer | (m_wait & freed);
            wt_any[s1_tag] <= m_wait;
            wt_m[s1_tag]   <= m_merge ? ms_hk : ms_free;
         end
         // an allocation: the MSHR goes live and its read is queued
         if (m_alloc) begin ms_v[ms_free] <= 1'b1;  iq_wr <= iq_wr + 1'b1; end
         // a store's bytes joining a live MSHR's merge buffer (the bytes: the one write above)
         if (m_smerge) begin
            mb_cv[ms_hk][s1_va[5:3]] <= 1'b1;
            ms_dty[ms_hk] <= 1'b1;
         end
         if (iq_go) iq_rd <= iq_rd + 1'b1;
         iq_n <= iq_n + {{MB{1'b0}}, m_alloc} - {{MB{1'b0}}, iq_go};
         // a dirty line leaving or cleaning: a write-back entry, and the read-out of its four rows --
         // a victim, a dropped or cleaned copy, or the clean walk's slot
         // (the free entry and the idle read-out are dead: their fields are written every idle cycle
         // and only wb_v and ro_on wait for the decision -- a victim when there is no copy to move)
         if (~ro_on & wb_free_v) begin
            wb_rdy[wb_free] <= 1'b0;  wb_sent[wb_free] <= 1'b0;
            wb_cpl[wb_free] <= ~cl_on & (k_fl | k_cln);
            wb_pl[wb_free] <= cl_on  ? {{(52-PTB){1'b0}}, pt_c[cl_k*PTB +: PTB], cl_r}
                            : absent ? {{(52-PTB){1'b0}}, pt_r[k_vic*PTB +: PTB], s1_row} : s1_pl;
            ro_cnt <= 2'd0;  ro_wb <= wb_free;
            ro_way <= cl_on ? cl_k[COLB] : absent ? pick1 : cp_w1;
            ro_set <= cl_on ? {cl_k[COLB-1:0], cl_r} : absent ? s1_set : cp_set;
         end
         if ((ro_need & ~ro_block & ~wb_block) | cl_ro) begin
            wb_v[wb_free] <= 1'b1;  ro_on <= 1'b1;
         end else if (ro_on) begin
            ro_cnt <= ro_cnt + 1'b1;
            if (ro_cnt == 2'd3) ro_on <= 1'b0;
         end
         rl_v <= ro_on;  rl_row <= ro_cnt;  rl_way <= ro_way;  rl_wb <= ro_wb;  rl_last <= ro_on & (ro_cnt == 2'd3);
         if (rl_v) begin
            wb_data[rl_wb][rl_row*128 +: 128] <= rl_way ? {bk_q[3], bk_q[2]} : {bk_q[1], bk_q[0]};
            if (rl_last) wb_rdy[rl_wb] <= 1'b1;
         end
         if (ws_go) wb_sent[ws] <= 1'b1;
         if (cw_ok) wb_v[cw_e[WBB-1:0]] <= 1'b0;
         // the NC slot: taken, sent, answered or completed
         if (m_nc) begin
            nc_v <= 1'b1;  nc_we <= s1_st;  nc_sent <= 1'b0;  nc_pl <= s1_pl;  nc_pa <= s1_pa;
            nc_tag <= s1_tag[RTW-1:0];  nc_wd <= st_wd;  nc_wm <= st_wm;
         end
         if (nq_go) nc_sent <= 1'b1;
         if (cr_valid & (cr_slot == NCS) & (cr_beat == nc_pa[5:4])) nc_data <= nc_pa[3] ? cr_data[127:64] : cr_data[63:0];
         if (cr_valid & (cr_slot == NCS) & cr_last) nc_rdy <= 1'b1;
         if (nc_done) begin nc_v <= 1'b0;  nc_rdy <= 1'b0; end
         // a fill beat: write the reserved way's row; the last installs (the tag writes above) and releases
         fb_v <= cr_f;  fb_last <= cr_f & cr_last;  fb_m <= cr_m;
         if (cr_f) begin
            fb_way <= ms_way[cr_m];  fb_row <= {ms_set[cr_m], cr_beat};  fb_data <= {cr_hi, cr_lo};
            ms_bt[cr_m] <= cr_beat + 1'b1;  ms_bl[cr_m] <= 1'b1;
         end
         fl_v <= fb_last;  fl_m <= fb_m;
         if (fl_v) ms_v[fl_m] <= 1'b0;
         // epochs and the scan
         if (ep_bump) cur_ep <= cur_ep + 1'b1;
         if (ep_wrap) begin scan_on <= 1'b1; scan <= {(IB+1){1'b0}}; end
         else if (scan_w) begin
            scan <= scan + 1'b1;
            if (scan == SETS - 1) scan_on <= 1'b0;
         end
         // the clean walk, then the wait for its write-backs to reach memory
         if (inv_req) cl_pend <= 1'b1;
         if ((inv_req | cl_pend) & ~cl_on & ~cl_wait & ~st_busy & ~ms_dirty) begin cl_pend <= 1'b0;  cl_on <= 1'b1;  cl_r <= 6'd0; end
         else if (cl_adv) begin
            cl_r <= cl_r + 1'b1;
            if (&cl_r) begin cl_on <= 1'b0;  cl_wait <= 1'b1; end
         end
         if (cl_wait & ~ro_on & ~wb_any) cl_wait <= 1'b0;
         // the door: shut during a scan and the clean
         door <= ~scan_on & ~ep_wrap & ~cl_pend & ~cl_on & ~cl_wait & ~inv_req;
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
   //   4 two         two pvalid copies of one physical line
   //   5 span        VA and PA disagree in the page offset
   //   6 tag_reuse   a requester tag reused while outstanding
   //   7 dead_wait   a waiter parked on an MSHR that is not live
   //   8 ro_race     a fill beat for a row of the line being read out, not yet read
   //   9 vv_no_pv    vvalid without pvalid
   //  10 vhit_other  a current-epoch virtual hit on another physical line
   wire e_orphan  = cr_f & ~ms_v[cr_m];                              // a fill beat for no MSHR
   wire e_slot    = cr_valid & ~cr_f & ~((cr_slot == NCS) & nc_v & ~nc_we & nc_sent);   // a read response nothing awaits
   wire e_wdone   = cw_valid & ~cw_ok & ~fin_nc;                     // a write completion for no write sent
   wire e_beat    = cr_f & ms_v[cr_m] & (cr_beat != ms_bt[cr_m]);    // beats out of order within a burst
   wire e_two     = mis & ((pm & (pm - 1'b1)) != 0);                 // two pvalid copies of one physical line
   wire e_span    = (new_go & ~a_ph & (rd_va[11:0] != rd_pa[11:0])) | (st_go & ~a_ph & (wr_va[11:0] != wr_pa[11:0]));   // VA/PA disagree in the page offset
   wire e_wtbusy  = new_go & (wt_v[{1'b0, rd_tag}] | (s1_v & (s1_tag == {1'b0, rd_tag})));   // a tag reused while outstanding
   reg  e_wdead;                                                     // a waiter parked on an MSHR that is not live
   always @* begin
      e_wdead = 1'b0;
      for (i = 0; i < NW; i = i + 1) if (wt_v[i] & ~wt_rdy[i] & ~wt_any[i] & ~ms_v[wt_m[i]]) e_wdead = 1'b1;
   end
   // A fill beat may land in the slot being read out -- the next lookup can reserve a slot a drop
   // just freed, and the on-chip SRAM answers within the read-out's four cycles -- but beat c is
   // written a cycle after it lands, so it is safe while row c is read no later than that: the
   // hazard is a beat for a row the read-out has not reached yet.
   wire e_rorace  = cr_f & ro_on & (ms_way[cr_m] == ro_way) & (ms_set[cr_m] == ro_set) & (cr_beat > ro_cnt);
   wire e_vnp     = l1_v & ((h0 & ~pv_r[k_own0]) | (h1 & ~pv_r[k_own1]));      // vvalid without pvalid
   wire e_vpa     = l1_v & cur & ~s1_ph & ((h0 & ~own0) | (h1 & ~own1));      // a current-epoch virtual hit on another physical line
   wire [15:0] e_now = {5'd0, e_vpa, e_vnp, e_rorace, e_wdead, e_wtbusy, e_span, e_two, e_beat, e_wdone, e_slot, e_orphan};
   reg  [15:0] e_q;
   always @(posedge clk) e_q <= reset ? 16'd0 : e_now;
   assign err = e_q;
   always @(posedge clk) if (!reset && (e_now != 0))
      $fatal(1, "rv_dcache: invariant %b (orphan beat, bad slot, write completion, beat order, two copies, VA/PA offset, tag reuse, dead waiter, read-out race, vvalid without pvalid, virtual hit on another line)", e_now[10:0]);

endmodule
`default_nettype wire
