`default_nettype none
// rv_dcache: the VHPR data cache (docs/PLAN-2026-09-25-dcache-vhpr.md), non-blocking, on the
// tagged memory port (rv_mem_arbiter's client side).
//
// GEOMETRY. SIZE_KB (128) in 2 ways of 64-byte lines, SETS = 1024 indexed by VA[15:6]: 16
// colours (VA[15:12]) over the 64 rows a 4 KiB page covers (VA[11:6] = PA[11:6]). Each way's data
// is two block-RAM banks, the even and the odd 8-byte chunks of a line, read at {set, chunk pair}.
//
// TWO KINDS OF TAG STATE. A line is PHYSICALLY present -- pvalid, its physical tag PA[PABITS-1:12]
// and the data -- independently of whether a VIRTUAL stamp names it (vvalid, the virtual tag
// VA[VAW-1:16] and the epoch it was stamped in). The virtual tags are per way, indexed by the VA
// set; the physical tags are one array per (way, colour), 64 rows deep, all read at PA[11:6], so
// the 32 places a physical line can live come out of one read (the probe).
//
// HIT (virtual only). A request taken at T reads the banks; at T+1 its way's vvalid, virtual tag
// and epoch are compared and the response is registered: rd_valid at T+2, the addressed bytes
// shifted to bit 0 (a request never spans an 8-byte chunk -- asserted), tagged with rd_tag.
//
// MISS. The probe compares the 32 candidates' physical tags with the request's PA:
//   - a match in the request's own set: that way's data was read with the lookup, so the request
//     answers now and the way is re-stamped with its virtual tag and epoch;
//   - a match in another colour (a synonym): that copy is dropped (single-copy invariant) and the
//     request replays, now finding no copy;
//   - no copy: the request waits in the waiter table, one entry per requester tag, on the MSHR
//     filling its physical line in its own set -- one allocated now, with a reserved way whose old
//     line is dropped and a read issued on the memory port under the MSHR's id -- or, when none can
//     be (MSHRs full, the set's ways both reserved, or the line filling into another colour), on
//     the next MSHR to free. A miss never holds the pipeline.
// A fill's beats write the reserved way's banks as they arrive (a cycle after each, through the
// beat register). The cycle after the last one is written the line installs -- the physical tag
// and the virtual stamp, with the epoch the MSHR was allocated in -- the MSHR is freed and its
// waiters are released; they replay through the lookup, ahead of the waiters on any MSHR, and hit.
//
// RESPONSES are matched by the requester's tag (rule B1) and may complete out of order: a hit
// behind a miss answers first. Every taken request is answered exactly once.
//
// EPOCHS. ep_bump advances the epoch: virtual stamps of the old epoch stop hitting; the lines stay
// physically present and re-stamp from the probe. A wrap (the epoch returns to a value still
// stamped somewhere) clears every vvalid over a scan, with the door shut; pvalid survives.
//
// STAGE 1 of the increment: loads (rd_*), the probe, MSHRs, fills and epochs. Stores, the
// write-back buffer, NC, CBOs and the walker's physical reads follow; the ports are already here.
module rv_dcache #(
   parameter SIZE_KB = 128,
   parameter RTW     = 4,              // the requester's opaque tag (the LQ index, the slow tag, the walkers)
   parameter VAW     = 39,             // Sv39
   parameter PABITS  = 36,             // significant physical address bits
   parameter NMSHR   = 8,
   parameter SW      = 4               // memory-port slot bits (this client's ids)
) (
   input  wire              clk,
   input  wire              reset,
   // ---- reads
   input  wire              rd_req,
   input  wire [63:0]       rd_va,
   input  wire [63:0]       rd_pa,
   input  wire [RTW-1:0]    rd_tag,
   output wire              rd_ack,      // taken this cycle
   output reg               rd_valid,
   output reg  [63:0]       rd_data,     // the addressed bytes at bit 0
   output reg  [RTW-1:0]    rd_resp_tag,
   output reg  [63:0]       rd_resp_addr,// the PA the response answers
   // ---- epochs
   input  wire              ep_bump,
   output reg               inv_busy,
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
   localparam COLB  = IB - 6;                     // 4 colour bits: VA[15:12]
   localparam NCOL  = 1 << COLB;
   localparam VTB   = VAW - OFFB - IB;            // virtual tag VA[38:16]
   localparam PTB   = PABITS - 12;                // physical tag PA[35:12]
   localparam EPW   = 2;
   localparam RB    = IB + 2;                     // bank row {set, chunk pair}
   localparam MB    = $clog2(NMSHR);
   localparam NTAG  = 1 << RTW;
   initial begin
      if (SETS != 1024 || COLB != 4) $fatal(1, "rv_dcache: the probe is built for 1024 sets of 16 colours");
      if (NMSHR > (1 << SW)) $fatal(1, "rv_dcache: %0d MSHRs need more than %0d slot bits", NMSHR, SW);
   end

   // ---------------------------------------------------------------- the arrays
   // virtual stamps, per way, by VA set (distributed RAM, one write statement each)
   reg [VTB-1:0] vt0 [0:SETS-1], vt1 [0:SETS-1];
   reg [EPW-1:0] ep0 [0:SETS-1], ep1 [0:SETS-1];
   reg           vv0 [0:SETS-1], vv1 [0:SETS-1];
   reg           rr  [0:SETS-1];                  // round-robin victim
   // physical tags, one array per (way, colour), by PA[11:6]
   reg [PTB-1:0] pt   [0:2*NCOL-1][0:63];
   reg           pv   [0:2*NCOL-1][0:63];
   integer i, j;
   initial begin
      for (i = 0; i < SETS; i = i + 1) begin vv0[i] = 1'b0; vv1[i] = 1'b0; rr[i] = 1'b0; end
      for (i = 0; i < 2*NCOL; i = i + 1) for (j = 0; j < 64; j = j + 1) pv[i][j] = 1'b0;
   end
   reg [EPW-1:0] cur_ep;

   // ---------------------------------------------------------------- the MSHRs
   reg              ms_v    [0:NMSHR-1];
   reg [57:0]       ms_pl   [0:NMSHR-1];         // the physical line
   reg [IB-1:0]     ms_set  [0:NMSHR-1];         // the VA set it fills
   reg [VTB-1:0]    ms_vt   [0:NMSHR-1];         // the virtual tag it stamps
   reg [EPW-1:0]    ms_ep   [0:NMSHR-1];         // ...with this epoch
   reg              ms_way  [0:NMSHR-1];
   reg [1:0]        ms_bt   [0:NMSHR-1];         // beats landed (the next one expected)
   // the waiters, one per requester tag: what to replay, and which MSHR it waits on
   reg              wt_v    [0:NTAG-1];
   reg              wt_rdy  [0:NTAG-1];          // replay it
   reg              wt_any  [0:NTAG-1];          // it waits on any MSHR freeing, not on wt_m
   reg [MB-1:0]     wt_m    [0:NTAG-1];
   reg [63:0]       wt_va   [0:NTAG-1], wt_pa [0:NTAG-1];
   reg [EPW-1:0]    wt_ep   [0:NTAG-1];

   // ---------------------------------------------------------------- the door and the lookup input
   // What the lookup takes this cycle, in priority order: a released waiter (replay) -- the lowest
   // one released by its MSHR or a drop, else the lowest waiting on any MSHR -- or a new request.
   reg        door;                               // registered: open unless a scan runs
   reg        s1_v;  reg [63:0] s1_va, s1_pa;  reg [RTW-1:0] s1_tag;  reg [EPW-1:0] s1_ep;
   reg              rm_v, ra_v;  reg [RTW-1:0] rm_t, ra_t;
   always @* begin
      rm_v = 1'b0;  rm_t = {RTW{1'b0}};  ra_v = 1'b0;  ra_t = {RTW{1'b0}};
      for (i = NTAG - 1; i >= 0; i = i - 1) begin
         if (wt_v[i] & wt_rdy[i] & ~wt_any[i]) begin rm_v = 1'b1; rm_t = i[RTW-1:0]; end
         if (wt_v[i] & wt_rdy[i] &  wt_any[i]) begin ra_v = 1'b1; ra_t = i[RTW-1:0]; end
      end
   end
   wire           rp_v   = rm_v | ra_v;
   wire [RTW-1:0] rp_t   = rm_v ? rm_t : ra_t;
   wire           new_go = ~rp_v & rd_req & door;
   assign rd_ack = new_go;
   wire       a_take = rp_v | new_go;
   wire [63:0]    a_va  = rp_v ? wt_va[rp_t] : rd_va;
   wire [63:0]    a_pa  = rp_v ? wt_pa[rp_t] : rd_pa;
   wire [RTW-1:0] a_tag = rp_v ? rp_t        : rd_tag;
   wire [EPW-1:0] a_ep  = rp_v ? wt_ep[rp_t] : cur_ep;

   // ---------------------------------------------------------------- the data banks
   // bank = way*2 + chunk parity; row = {set, chunk pair}. The read port serves the lookup; the
   // write port the fill beats (a 128-bit beat is one row of both of the reserved way's banks).
   wire [RB-1:0]  bk_ra = {a_va[OFFB +: IB], a_va[5:4]};
   reg            fb_v;  reg fb_way;  reg [RB-1:0] fb_row;  reg [127:0] fb_data;   // the beat being written
   wire [63:0]    bk_rd [0:3];
   genvar gb;
   generate for (gb = 0; gb < 4; gb = gb + 1) begin : bank
      smolrv64_sdpram #(.ADDR_WIDTH(RB), .DATA_WIDTH(64), .READ_LATENCY(1)) u_bank
        (.clock(clk), .rd_addr(bk_ra), .rd_data(bk_rd[gb]),
         .wr_en(fb_v & (fb_way == gb[1])), .wr_addr(fb_row), .wr_data(fb_data[gb[0]*64 +: 64]));
   end endgenerate

   // ---------------------------------------------------------------- the lookup (T+1)
   wire [IB-1:0]  s1_set = s1_va[OFFB +: IB];
   wire [VTB-1:0] s1_vtg = s1_va[OFFB+IB +: VTB];
   wire [PTB-1:0] s1_ptg = s1_pa[12 +: PTB];
   wire [5:0]     s1_row = s1_pa[11:6];
   wire [COLB-1:0] s1_col = s1_va[12 +: COLB];
   wire h0  = vv0[s1_set] & (vt0[s1_set] == s1_vtg) & (ep0[s1_set] == s1_ep);
   wire h1  = vv1[s1_set] & (vt1[s1_set] == s1_vtg) & (ep1[s1_set] == s1_ep);
   wire mis = s1_v & ~(h0 | h1);

   // the probe: the 32 candidates at PA[11:6] (array k = way*NCOL + colour)
   reg  [2*NCOL-1:0] pm;                          // candidate k holds the request's physical line
   always @* for (i = 0; i < 2*NCOL; i = i + 1) pm[i] = pv[i][s1_row] & (pt[i][s1_row] == s1_ptg);
   wire [COLB:0]     k_own0 = {1'b0, s1_col};     // the request's own set's two candidates
   wire [COLB:0]     k_own1 = {1'b1, s1_col};
   wire              own0  = pm[k_own0];
   wire              own1  = pm[k_own1];
   // the answer: a virtual hit, or the line physically in the request's own set (its re-stamp)
   wire        ans = s1_v & (h0 | h1 | own0 | own1);
   wire        aw1 = (h0 | h1) ? h1 : own1;
   wire [63:0] ans_chunk = aw1 ? (s1_va[3] ? bk_rd[3] : bk_rd[2]) : (s1_va[3] ? bk_rd[1] : bk_rd[0]);
   assign perf_access = s1_v;
   assign perf_miss   = s1_v & ~ans;
   wire [2*NCOL-1:0] other = pm & ~(({{(2*NCOL-1){1'b0}}, 1'b1} << k_own0) | ({{(2*NCOL-1){1'b0}}, 1'b1} << k_own1));
   // the other copy's (way, colour): at most one pvalid copy exists (asserted), so a priority pick names it
   reg  [$clog2(2*NCOL)-1:0] oth_k;  reg oth_v;
   always @* begin
      oth_v = 1'b0; oth_k = 0;
      for (i = 2*NCOL - 1; i >= 0; i = i - 1) if (other[i]) begin oth_v = 1'b1; oth_k = i[COLB:0]; end
   end

   // MSHR match (the line already being filled) and a free MSHR
   reg  [NMSHR-1:0] ms_hitv;  reg ms_set_busy0, ms_set_busy1;
   reg  [MB-1:0]    ms_hk, ms_free;  reg ms_free_v;
   always @* begin
      ms_hitv = 0;  ms_set_busy0 = 1'b0;  ms_set_busy1 = 1'b0;  ms_hk = 0;  ms_free = 0;  ms_free_v = 1'b0;
      for (i = NMSHR - 1; i >= 0; i = i - 1) begin
         if (ms_v[i] & (ms_pl[i][PABITS-7:0] == s1_pa[OFFB +: PABITS-6])) begin ms_hitv[i] = 1'b1; ms_hk = i[MB-1:0]; end
         if (ms_v[i] & (ms_set[i] == s1_set) & ~ms_way[i]) ms_set_busy0 = 1'b1;
         if (ms_v[i] & (ms_set[i] == s1_set) &  ms_way[i]) ms_set_busy1 = 1'b1;
         if (~ms_v[i]) begin ms_free = i[MB-1:0]; ms_free_v = 1'b1; end
      end
   end
   // the way a new MSHR reserves: an invalid way, else the round-robin one -- not one another MSHR holds
   wire v0_free = ~vv0[s1_set] & ~pv[k_own0][s1_row];
   wire v1_free = ~vv1[s1_set] & ~pv[k_own1][s1_row];
   wire pick1   = ms_set_busy0 ? 1'b1 : ms_set_busy1 ? 1'b0 : v0_free ? 1'b0 : v1_free ? 1'b1 : rr[s1_set];
   wire way_ok  = ~(ms_set_busy0 & ms_set_busy1);

   // what the miss does (one of these, decided at T+1)
   wire m_absent  = mis & ~(own0 | own1) & ~oth_v;
   wire m_restamp = mis & (own0 | own1);                              // answer and re-stamp
   wire m_drop    = mis & ~(own0 | own1) & oth_v;                     // a synonym copy goes, the request replays
   wire m_merge   = m_absent & (|ms_hitv) & (ms_set[ms_hk] == s1_set);// join the line's MSHR
   wire m_alloc   = m_absent & ~(|ms_hitv) & ms_free_v & way_ok;
   wire m_wait    = m_absent & ~m_merge & ~m_alloc;                   // wait on any MSHR freeing

   // ---------------------------------------------------------------- the fill
   // Beats land in order of their MSHR's burst (beat 0..3); each writes one row of the reserved way.
   wire [MB-1:0] cr_m   = cr_slot[MB-1:0];
   wire          f_last = cr_valid & cr_last;
   // The last beat reaches the bank a cycle later (fb_*) and the line installs -- its tags, the MSHR
   // freed, its waiters released -- the cycle after that, so no lookup sees the tags before the data.
   reg           fb_last;  reg [MB-1:0] fb_m;
   reg           fl_v;  reg [MB-1:0] fl_m;

   // ---------------------------------------------------------------- the memory port: the read issue queue
   // An allocated MSHR's read waits here until the port takes it; MSHR ids in allocation order.
   reg  [MB-1:0]   iq [0:NMSHR-1];
   reg  [MB-1:0]   iq_rd, iq_wr;
   reg  [MB:0]     iq_n;
   assign cq_valid = (iq_n != 0);
   assign cq_slot  = {{(SW-MB){1'b0}}, iq[iq_rd]};
   assign cq_we    = 1'b0;
   assign cq_addr  = ms_pl[iq[iq_rd]];
   assign cq_wmask = 64'd0;
   assign cq_wdata = 512'd0;
   wire   iq_go    = cq_valid & cq_ready;

   // ---------------------------------------------------------------- epochs and the scan
   reg  [IB:0] scan;
   reg         scan_on;
   // a wrap: the epoch after the bump is still stamped on some line -- clear every vvalid first
   wire ep_wrap = ep_bump & (cur_ep == {EPW{1'b1}});

   // ---------------------------------------------------------------- the tag writes
   // ONE write statement per array. vv/vt/ep: the install (fill's last beat), the re-stamp, a
   // dropped synonym's or evicted line's vvalid, the scan. pt/pv: the install, a drop.
   wire [MB-1:0]  im   = fl_m;                          // the MSHR installing
   wire           t_inst = fl_v;
   wire           t_rst  = m_restamp;
   wire           rst_w1 = ~own0;                       // the re-stamped way
   // an eviction (m_alloc) clears the reserved way's stamp and physical copy in the request's set
   wire           t_evict = m_alloc;
   wire [IB-1:0]  oth_set = {oth_k[COLB-1:0], s1_row};  // the synonym's VA set
   wire           oth_w1  = oth_k[COLB];
   always @(posedge clk) begin
      // way 0 virtual state
      if (scan_on)                                   vv0[scan[IB-1:0]] <= 1'b0;
      else if (t_inst & ~ms_way[im])                 vv0[ms_set[im]]   <= 1'b1;
      else if (t_rst & ~rst_w1)                      vv0[s1_set]       <= 1'b1;
      else if (t_evict & ~pick1)                     vv0[s1_set]       <= 1'b0;
      else if (m_drop & ~oth_w1)                     vv0[oth_set]      <= 1'b0;
      if (t_inst & ~ms_way[im])                      begin vt0[ms_set[im]] <= ms_vt[im]; ep0[ms_set[im]] <= ms_ep[im]; end
      else if (t_rst & ~rst_w1)                      begin vt0[s1_set] <= s1_vtg; ep0[s1_set] <= s1_ep; end
      // way 1
      if (scan_on)                                   vv1[scan[IB-1:0]] <= 1'b0;
      else if (t_inst &  ms_way[im])                 vv1[ms_set[im]]   <= 1'b1;
      else if (t_rst &  rst_w1)                      vv1[s1_set]       <= 1'b1;
      else if (t_evict &  pick1)                     vv1[s1_set]       <= 1'b0;
      else if (m_drop &  oth_w1)                     vv1[oth_set]      <= 1'b0;
      if (t_inst &  ms_way[im])                      begin vt1[ms_set[im]] <= ms_vt[im]; ep1[ms_set[im]] <= ms_ep[im]; end
      else if (t_rst &  rst_w1)                      begin vt1[s1_set] <= s1_vtg; ep1[s1_set] <= s1_ep; end
      if (t_inst) rr[ms_set[im]] <= ~ms_way[im];
   end
   // the physical tags: one write statement per (way, colour) array
   genvar gk;
   generate for (gk = 0; gk < 2*NCOL; gk = gk + 1) begin : ptw
      wire inst_k  = t_inst  & ({ms_way[im], ms_set[im][IB-1:6]} == gk);
      wire evict_k = t_evict & ({pick1, s1_col} == gk);
      wire drop_k  = m_drop  & (oth_k == gk);
      wire [5:0] row_k = inst_k ? ms_set[im][5:0] : s1_row;
      always @(posedge clk)
         if (inst_k | evict_k | drop_k) begin
            pv[gk][row_k] <= inst_k;
            if (inst_k) pt[gk][row_k] <= ms_pl[im][6 +: PTB];
         end
   end endgenerate

   // ---------------------------------------------------------------- the machine
   always @(posedge clk) begin
      if (reset) begin
         cur_ep <= {EPW{1'b0}};  door <= 1'b0;  s1_v <= 1'b0;  rd_valid <= 1'b0;  inv_busy <= 1'b1;
         scan_on <= 1'b1;  scan <= {(IB+1){1'b0}};  fl_v <= 1'b0;
         iq_n <= 0;  iq_rd <= 0;  iq_wr <= 0;  fb_v <= 1'b0;  fb_last <= 1'b0;
         for (i = 0; i < NMSHR; i = i + 1) begin ms_v[i] <= 1'b0; end
         for (i = 0; i < NTAG; i = i + 1)  begin wt_v[i] <= 1'b0; wt_rdy[i] <= 1'b0; end
      end else begin
         // the lookup stage takes whatever the banks were addressed for
         s1_v <= a_take;
         if (a_take) begin s1_va <= a_va; s1_pa <= a_pa; s1_tag <= a_tag; s1_ep <= a_ep; end
         if (rp_v) wt_v[rp_t] <= 1'b0;                  // a replaying waiter leaves the table
         // the answer
         rd_valid <= ans;
         if (ans) begin
            rd_data      <= ans_chunk >> (8 * s1_va[2:0]);
            rd_resp_tag  <= s1_tag;  rd_resp_addr <= s1_pa;
         end
         // the release: the installing MSHR's waiters, and every waiter on any MSHR
         if (fl_v)
            for (i = 0; i < NTAG; i = i + 1) if (wt_v[i] & (wt_any[i] | (wt_m[i] == fl_m))) wt_rdy[i] <= 1'b1;
         // a miss enters the waiter table: a dropped synonym ready at once, a waiter on the installing
         // MSHR or on any MSHR ready if the release is this cycle
         if (m_drop | m_merge | m_alloc | m_wait) begin
            wt_v[s1_tag]   <= 1'b1;
            wt_rdy[s1_tag] <= m_drop | (fl_v & (m_wait | (m_merge & (ms_hk == fl_m))));
            wt_any[s1_tag] <= m_wait;
            wt_m[s1_tag]   <= m_merge ? ms_hk : ms_free;
            wt_va[s1_tag] <= s1_va;  wt_pa[s1_tag] <= s1_pa;  wt_ep[s1_tag] <= s1_ep;
         end
         if (m_alloc) begin
            ms_v[ms_free] <= 1'b1;  ms_pl[ms_free] <= s1_pa[63:OFFB];
            ms_set[ms_free] <= s1_set;  ms_vt[ms_free] <= s1_vtg;  ms_ep[ms_free] <= s1_ep;
            ms_way[ms_free] <= pick1;  ms_bt[ms_free] <= 2'd0;
            iq[iq_wr] <= ms_free;  iq_wr <= iq_wr + 1'b1;
         end
         if (iq_go) iq_rd <= iq_rd + 1'b1;
         iq_n <= iq_n + {{MB{1'b0}}, m_alloc} - {{MB{1'b0}}, iq_go};
         // a fill beat: write the reserved way's row; the last installs (the tag writes above) and releases
         fb_v <= cr_valid;  fb_last <= f_last;  fb_m <= cr_m;
         if (cr_valid) begin
            fb_way <= ms_way[cr_m];  fb_row <= {ms_set[cr_m], cr_beat};  fb_data <= cr_data;
            ms_bt[cr_m] <= cr_beat + 1'b1;
         end
         fl_v <= fb_last;  fl_m <= fb_m;
         if (fl_v) ms_v[fl_m] <= 1'b0;
         // epochs and the scan
         if (ep_bump) cur_ep <= cur_ep + 1'b1;
         if (ep_wrap) begin scan_on <= 1'b1; scan <= {(IB+1){1'b0}}; inv_busy <= 1'b1; end
         else if (scan_on) begin
            scan <= scan + 1'b1;
            if (scan == SETS - 1) begin scan_on <= 1'b0; inv_busy <= 1'b0; end
         end
         // the door: shut during a scan
         door <= ~scan_on & ~ep_wrap;
      end
   end

   // ---------------------------------------------------------------- invariants (always on)
   // Register the conditions; each is a bit of the integrity log (the D$'s bits 0-15).
   wire e_orphan  = cr_valid & ~ms_v[cr_m];                          // a fill beat for no MSHR
   wire e_slot    = cr_valid & (cr_slot >= NMSHR);                   // a read response on a slot no MSHR owns
   wire e_wdone   = cw_valid;                                        // a write completion (stage 1 writes nothing)
   wire e_beat    = cr_valid & ms_v[cr_m] & (cr_beat != ms_bt[cr_m]);// beats out of order within a burst
   wire e_two     = mis & ((pm & (pm - 1'b1)) != 0);                 // two pvalid copies of one physical line
   wire e_span    = rd_ack & ((rd_va[2:0] != rd_pa[2:0]) | (rd_va[11:0] != rd_pa[11:0]));   // VA/PA disagree in the page offset
   wire e_wtbusy  = new_go & (wt_v[rd_tag] | (s1_v & (s1_tag == rd_tag)));   // a tag reused while outstanding
   reg  e_wdead;                                                     // a waiter parked on an MSHR that is not live
   always @* begin
      e_wdead = 1'b0;
      for (i = 0; i < NTAG; i = i + 1) if (wt_v[i] & ~wt_rdy[i] & ~wt_any[i] & ~ms_v[wt_m[i]]) e_wdead = 1'b1;
   end
   wire [15:0] e_now = {8'd0, e_wdead, e_wtbusy, e_span, e_two, e_beat, e_wdone, e_slot, e_orphan};
   reg  [15:0] e_q;
   always @(posedge clk) e_q <= reset ? 16'd0 : e_now;
   assign err = e_q;
   always @(posedge clk) if (!reset && (e_now != 0))
      $fatal(1, "rv_dcache: invariant %b (orphan beat, bad slot, write completion, beat order, two copies, VA/PA offset, tag reuse, dead waiter)", e_now[7:0]);

endmodule
`default_nettype wire
