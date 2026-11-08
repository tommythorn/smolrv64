`default_nettype none
// rv_l2 -- the second-level cache: 2 MiB between the caches' memory port (rv_mem_arbiter) and DRAM
// (docs/PLAN-2026-10-21-memory-system.md, "The L2").
//
// The same tagged line port on both sides (rv_mem_arbiter's header): a request {id, we, line, byte
// mask, line data}; read data in four 128-bit beats {id, beat, last}; write done {id}. Above, the
// arbiter's request register holds a request until this cache takes it, and responses are always
// taken. Below, responses are always taken too (ddr_port_sync asserts it). Ids below are this
// cache's own: an MSHR's index for a read, NM plus a write-back buffer's for a write.
//
// A line is keyed by its index in the DRAM window (dram_axi, src/dram_window.vh), the line the port
// below reaches whatever the request's physical address (a wrong-path fetch of PA 0 reads the
// window's wrap there, and here the same line). 2^SB sets of eight ways. The tags, dirty bits and NRU
// bits are block RAM, a set's eight ways one read; the data is all 64 UltraRAMs, a line a word,
// addressed {way, set}.
//
// One request is looked up at a time: its set's tags are read (LK0), compared (LK1), and acted on
// (ACT), which writes the set's tags back. A read hit reads its line four times on the data array's
// port B, a beat each. A read miss takes an MSHR and a victim way (an invalid one, else one not
// recently used, never a way an MSHR is filling), and the request is taken: the line comes from
// below, its beats written into the way on port A and passed up as they arrive. A dirty victim is
// read out into a write-back buffer first. A write hits, or it allocates and comes round again to
// hit: a whole line takes its way with no read below, a partial line (the D$'s NC store) fetches
// the line into its way first. A write's done goes up when its line is in the array. So the data
// array is only written by a hit or a fill, and a request for a line an MSHR is filling or a
// write-back buffer holds waits until it is done (the lookup repeats).
//
// Read data going up: a hit's beats own the channel; a fill's beats pass straight up when it is
// free, else queue behind it. A hit starts only with the queue empty, so the queue stays short.
// Requests below: the MSHRs' reads first, then the write-backs.
//
// COHERENT DMA. A DMA master's line comes here after the D$ has cleaned or flushed it (the snoop,
// held until s_ack; it goes ahead of a waiting request). It is looked up like a request. Dirty here,
// it is written back like a victim and left clean (a clean) or invalid (a flush), and the lookup
// repeats: it waits, as a request would, until the write-back's done, and then the line is answered.
// Not dirty, it is answered at once, a flush invalidating it. A flush's line is blocked from then
// until s_rel (its DMA write is in the controller): a request for it waits, so no fill of the line
// can read DRAM ahead of the DMA's write.
module rv_l2 #(
   parameter integer IDW = 5,                    // port id bits, both sides
   parameter integer AW  = 58,                   // line address width (PA[63:6])
   parameter integer SB  = 12,                   // set bits: 4096 sets of 8 ways, 2 MiB
   parameter integer NM  = 8,                    // MSHRs
   parameter integer NWB = 4,                    // write-back buffers
   parameter integer RL  = 5                     // the data array's read latency (its input register in)
) (
   input  wire              clk,
   input  wire              reset,
   // ---- above: the caches ----
   input  wire              u_q_valid,
   output wire              u_q_ready,
   input  wire [IDW-1:0]    u_q_id,
   input  wire              u_q_we,
   input  wire [AW-1:0]     u_q_addr,
   input  wire [63:0]       u_q_wmask,
   input  wire              u_q_full,        // &u_q_wmask, from the requester's register (it gates u_q_ready)
   input  wire [511:0]      u_q_wdata,
   output reg               u_r_valid,
   input  wire              u_r_ready,
   output reg  [IDW-1:0]    u_r_id,
   output reg  [1:0]        u_r_beat,
   output reg               u_r_last,
   output reg  [127:0]      u_r_data,
   output reg               u_w_valid,
   input  wire              u_w_ready,
   output reg  [IDW-1:0]    u_w_id,
   // ---- below: DRAM ----
   output reg               d_q_valid,
   input  wire              d_q_ready,
   output reg  [IDW-1:0]    d_q_id,
   output reg               d_q_we,
   output reg  [AW-1:0]     d_q_addr,
   output wire [63:0]       d_q_wmask,
   output reg  [511:0]      d_q_wdata,
   input  wire              d_r_valid,
   output wire              d_r_ready,
   input  wire [IDW-1:0]    d_r_id,
   input  wire [1:0]        d_r_beat,
   input  wire              d_r_last,
   input  wire [127:0]      d_r_data,
   input  wire              d_w_valid,
   output wire              d_w_ready,
   input  wire [IDW-1:0]    d_w_id,
   // ---- a DMA master's line, after the D$: cleaned (keep) or flushed ----
   input  wire              s_valid,             // held until s_ack
   input  wire [25:0]       s_line,              // the line's DRAM index (the masters' AXI address / 64)
   input  wire              s_keep,
   output wire              s_ack,
   input  wire              s_rel                // a pulse: the flushed line's DMA write is in the controller
);
`include "dram_window.vh"
   localparam integer NWAY = 8;
   localparam integer WYB  = 3;
   localparam integer DLB  = 26;                 // a line's index in the 4 GiB DRAM window
   localparam integer TGB  = DLB - SB;           // the tag
   localparam integer NSET = 1 << SB;
   localparam integer TE   = TGB + 2;            // a way's tag entry {valid, dirty, tag}
   localparam integer HW   = 4 * TE + 4;         // half a set: four ways' entries and their NRU bits
   localparam integer MB   = (NM  > 1) ? $clog2(NM)  : 1;
   localparam integer BB   = (NWB > 1) ? $clog2(NWB) : 1;
   localparam integer FD   = 16;                 // the fill-beat queue
   localparam integer FB   = 4;
   localparam integer FW   = IDW + 3 + 128;      // a queued beat {id, beat, last, data}
   initial begin
      if (NM + NWB > (1 << IDW)) $fatal(1, "rv_l2: %0d MSHRs and %0d write-backs do not fit %0d id bits", NM, NWB, IDW);
      if (RL < 3) $fatal(1, "rv_l2: the data array's read latency %0d is under 3", RL);
   end
   localparam [IDW-1:0] NMI = NM[IDW-1:0];       // the first write-back id below

   function [DLB-1:0] dl_of(input [AW-1:0] line);
      reg [31:0] a;
      begin a = dram_axi({line, 6'd0}); dl_of = a[31:6]; end
   endfunction
   // the window's own physical line for an index (the inverse of dram_axi inside the window)
   function [AW-1:0] line_of(input [DLB-1:0] dl);
      line_of = {{(AW-DLB-1){1'b0}}, dl[DLB-1], ~dl[DLB-1], dl[DLB-2:0]};
   endfunction

   // ---------------- the request in hand: a snoop, else the arbiter's register (held until taken) ----------------
   localparam [1:0] S_INIT = 2'd0, S_LK0 = 2'd1, S_LK1 = 2'd2, S_ACT = 2'd3;
   reg  [1:0]    st;
   reg           k_sn;                           // the snoop is in hand (chosen at LK0)
   wire          q_sn   = (st == S_LK0) ? s_valid : k_sn;
   wire [DLB-1:0] q_dl  = q_sn ? s_line : dl_of(u_q_addr);
   wire [SB-1:0]  q_set  = q_dl[SB-1:0];
   wire [TGB-1:0] q_tag  = q_dl[DLB-1:SB];
   wire           q_full = u_q_full;
   always @(posedge clk) if (!reset && u_q_valid && (u_q_full != &u_q_wmask))
      $fatal(1, "rv_l2: the request's whole-line flag %b is not its mask's (%h)", u_q_full, u_q_wmask);

   reg  [SB-1:0] ic;                             // the reset sweep's set

   // ---------------- MSHRs and write-back buffers ----------------
   reg  [NM-1:0]  m_v, m_sent, m_fw;             // valid; read sent below; beats go up
   reg  [DLB-1:0] m_dl  [0:NM-1];
   reg  [WYB-1:0] m_way [0:NM-1];
   reg  [IDW-1:0] m_uid [0:NM-1];
   reg  [NWB-1:0] wb_v, wb_rdy, wb_sent;         // valid; the line is read out; write sent below
   reg  [DLB-1:0] wb_dl [0:NWB-1];
   (* ram_style = "distributed" *) reg [511:0] wb_dat [0:NWB-1];

   reg  [MB-1:0]  mf;  reg mf_any;               // the lowest free MSHR
   reg  [BB-1:0]  wf;  reg wf_any;               // the lowest free write-back buffer
   reg  [MB-1:0]  mp;  reg mp_any;               // the lowest MSHR whose read is unsent
   reg  [BB-1:0]  wp;  reg wp_any;               // the lowest write-back ready and unsent
   reg            xb_v;  reg [DLB-1:0] xb_dl;    // a flushed line, blocked until its DMA write is in
   wire           q_xb = xb_v && xb_dl == q_dl;
   reg  [NM-1:0]  q_mhit;                        // MSHRs filling the request's line
   reg  [NWB-1:0] q_wbhit;                       // write-backs holding it
   reg  [NWAY-1:0] q_busy;                       // the request's set's ways an MSHR is filling
   integer i;
   always @* begin
      mf_any = 1'b0; mf = {MB{1'b0}}; mp_any = 1'b0; mp = {MB{1'b0}}; q_busy = {NWAY{1'b0}};
      for (i = NM - 1; i >= 0; i = i - 1) begin
         if (!m_v[i])               begin mf_any = 1'b1; mf = i[MB-1:0]; end
         if (m_v[i] && !m_sent[i])  begin mp_any = 1'b1; mp = i[MB-1:0]; end
         q_mhit[i] = m_v[i] && m_dl[i] == q_dl;
         if (m_v[i] && m_dl[i][SB-1:0] == q_set) q_busy[m_way[i]] = 1'b1;
      end
      wf_any = 1'b0; wf = {BB{1'b0}}; wp_any = 1'b0; wp = {BB{1'b0}};
      for (i = NWB - 1; i >= 0; i = i - 1) begin
         if (!wb_v[i])                           begin wf_any = 1'b1; wf = i[BB-1:0]; end
         if (wb_v[i] && wb_rdy[i] && !wb_sent[i]) begin wp_any = 1'b1; wp = i[BB-1:0]; end
         q_wbhit[i] = wb_v[i] && wb_dl[i] == q_dl;
      end
   end

   // ---------------- tags: two block RAMs of half a set, read at the request's set ----------------
   wire [HW-1:0]  t_rd0, t_rd1;
   wire [2*HW-1:0] t_rd = {t_rd1, t_rd0};
   wire           t_we;
   wire [2*HW-1:0] t_wd;
   smolrv64_sdpram #(.ADDR_WIDTH(SB), .DATA_WIDTH(HW), .READ_LATENCY(1)) u_tag0
     (.clock(clk), .rd_addr(q_set), .rd_data(t_rd0),
      .wr_en(t_we), .wr_addr(st == S_INIT ? ic : q_set), .wr_data(t_wd[HW-1:0]));
   smolrv64_sdpram #(.ADDR_WIDTH(SB), .DATA_WIDTH(HW), .READ_LATENCY(1)) u_tag1
     (.clock(clk), .rd_addr(q_set), .rd_data(t_rd1),
      .wr_en(t_we), .wr_addr(st == S_INIT ? ic : q_set), .wr_data(t_wd[2*HW-1:HW]));

   // the set's ways: entry w is in half w/4 at (w%4)*TE, the NRU bits above the four entries
   reg  [TE-1:0]   t_e [0:NWAY-1];
   reg  [NWAY-1:0] t_v, t_d, t_hit, t_nru;
   always @* for (i = 0; i < NWAY; i = i + 1) begin
      t_e[i]   = t_rd[(i/4)*HW + (i%4)*TE +: TE];
      t_v[i]   = t_e[i][TE-1];
      t_d[i]   = t_e[i][TE-2];
      t_hit[i] = t_v[i] && t_e[i][TGB-1:0] == q_tag;
      t_nru[i] = t_rd[(i/4)*HW + 4*TE + (i%4)];
   end
   // the victim: an invalid way, else one not recently used, never one an MSHR is filling
   wire [NWAY-1:0] c_inv = ~t_v & ~q_busy;
   wire [NWAY-1:0] c_nru = ~t_nru & ~q_busy;
   wire [NWAY-1:0] c_vic = (|c_inv) ? c_inv : (|c_nru) ? c_nru : ~q_busy;
   reg  [WYB-1:0]  vw, hw;
   always @* begin
      vw = {WYB{1'b0}}; hw = {WYB{1'b0}};
      for (i = NWAY - 1; i >= 0; i = i - 1) begin
         if (c_vic[i]) vw = i[WYB-1:0];
         if (t_hit[i]) hw = i[WYB-1:0];
      end
   end

   // ---------------- LK1's registers: what ACT acts on ----------------
   reg            k_hit, k_vd, k_cf;
   reg  [WYB-1:0] k_way;                         // the hit way, else the victim
   wire [WYB-1:0] n_way = (|t_hit) ? hw : vw;
   always @(posedge clk) begin
      if (st == S_LK0) k_sn <= s_valid;
      if (st == S_LK1) begin
         k_hit <= |t_hit;
         k_way <= n_way;
         k_vd  <= t_v[n_way] & t_d[n_way];       // the way is dirty: a miss's victim, a snoop's line
         k_cf  <= (|q_mhit) | (|q_wbhit) | q_xb | (~k_sn & ~(|c_vic));
      end
   end

   // ---------------- ACT ----------------
   // The tags read in LK1 are still the RAMs' output: nothing writes them between, and the address
   // is the held request's set.
   reg  [3:0]     hs_n;  reg [1:0] hs_beat;      // a hit's reads still to issue, and their beat
   reg  [WYB+SB-1:0] hs_a;  reg [IDW-1:0] hs_uid;
   reg  [FB:0]    ff_n;                          // the fill-beat queue's depth
   wire           b_free = (hs_n == 4'd0);       // port B is ACT's this cycle
   wire           ff_empty = (ff_n == {(FB+1){1'b0}});
   wire           a_rd    = ~u_q_we;
   wire           a_whole = u_q_we & q_full;     // a whole-line write allocates without a read below
   wire           a_hitrd = k_hit & a_rd & b_free & ff_empty;
   wire           a_hitwr = k_hit & u_q_we & b_free;
   wire           a_alloc = ~k_hit & (a_whole | mf_any) & (~k_vd | (wf_any & b_free));
   wire           a_sdty  = k_hit & k_vd;        // a snoop's line is dirty here: written back first
   wire           act     = (st == S_ACT) & ~k_cf;
   wire           r_go    = act & ~k_sn & (a_hitrd | a_hitwr | a_alloc);
   wire           s_go    = act &  k_sn & (~a_sdty | (wf_any & b_free));
   wire           a_go    = r_go | s_go;
   assign u_q_ready = r_go & (k_hit | a_rd);     // a write that allocates comes round again
   assign s_ack     = s_go & ~a_sdty;
   wire           b_hrd = r_go & k_hit & a_rd;
   wire           b_wr  = r_go & k_hit & u_q_we;
   wire           b_vrd = (r_go & ~k_hit & k_vd) | (s_go & a_sdty);
   wire           m_new = r_go & ~k_hit & ~a_whole;

   // the set's tags written back: a request's way entry, and its NRU bit (all set: only this one);
   // a snoop's line, clean or invalid, the NRU bits as they are
   reg  [2*HW-1:0] k_nw;
   reg  [NWAY-1:0] n_nru;
   always @* begin
      k_nw  = t_rd;
      n_nru = t_nru | ({{(NWAY-1){1'b0}}, 1'b1} << k_way);
      if (&n_nru) n_nru = {{(NWAY-1){1'b0}}, 1'b1} << k_way;
      if (k_sn) n_nru = t_nru;
      for (i = 0; i < NWAY; i = i + 1) begin
         if (k_way == i[WYB-1:0] && (!k_sn || k_hit))
            k_nw[(i/4)*HW + (i%4)*TE +: TE] = k_sn ? {s_keep, 1'b0, q_tag} : {1'b1, k_hit & (t_d[i] | u_q_we), q_tag};
         k_nw[(i/4)*HW + 4*TE + (i%4)] = n_nru[i];
      end
   end
   assign t_we = (st == S_INIT) | a_go;
   assign t_wd = (st == S_INIT) ? {(2*HW){1'b0}} : k_nw;
   wire [TGB-1:0] k_vtag = t_e[k_way][TGB-1:0];  // the victim's tag (ACT's way is the victim on a miss)

   // ---------------- the data array ----------------
   // port A: fill beats. port B: a hit's four reads, a victim's read, a write hit.
   wire [MB-1:0]  fm   = d_r_id[MB-1:0];
   wire           f_v  = d_r_valid;
   wire [511:0]   b_rdata;
   wire [WYB+SB-1:0] a_addr = {m_way[fm], m_dl[fm][SB-1:0]};
   wire [WYB+SB-1:0] b_addr = b_free ? {k_way, q_set} : hs_a;
   wire           b_en   = b_hrd | b_vrd | b_wr | ~b_free;
   smolrv64_uram_be #(.ADDR_WIDTH(WYB+SB), .RL(RL)) u_data
     (.clock(clk), .b_en(b_en),
      .a_addr(a_addr), .a_be(f_v ? ({48'd0, 16'hFFFF} << {d_r_beat, 4'd0}) : 64'd0), .a_wdata({4{d_r_data}}),
      .b_addr(b_addr), .b_be(b_wr ? u_q_wmask : 64'd0), .b_wdata(u_q_wdata), .b_rdata(b_rdata));

   // the reads in flight on port B: {hit, beat, id or write-back buffer, line}
   reg  [RL-1:0]  rp_v, rp_h;
   reg  [1:0]     rp_beat [0:RL-1];
   reg  [IDW-1:0] rp_uid  [0:RL-1];
   reg  [BB-1:0]  rp_wbi  [0:RL-1];
   reg  [WYB+SB-1:0] rp_a [0:RL-1];
   wire           hb_v = rp_v[RL-1] & rp_h[RL-1];       // a hit's beat leaves the array
   wire           vb_v = rp_v[RL-1] & ~rp_h[RL-1];      // a victim's line leaves the array
   wire [1:0]     hb_beat = rp_beat[RL-1];

   // ---------------- read data going up ----------------
   wire           f_up  = f_v & m_fw[fm];
   wire           f_byp = f_up & ~hb_v & ff_empty;
   wire           ff_push = f_up & ~f_byp;
   wire           ff_pop  = ~hb_v & ~ff_empty;
   (* ram_style = "distributed" *) reg [FW-1:0] ff [0:FD-1];
   reg  [FB-1:0]  ff_wp, ff_rp;
   wire [FW-1:0]  ff_hd = ff[ff_rp];
   always @(posedge clk) if (ff_push) ff[ff_wp] <= {m_uid[fm], d_r_beat, d_r_last, d_r_data};

   assign d_r_ready = 1'b1;
   assign d_w_ready = 1'b1;
   assign d_q_wmask = {64{d_q_we}};
   wire           d_load = (~d_q_valid | d_q_ready) & (mp_any | wp_any);
   wire [IDW-1:0] wd_k = d_w_id - NMI;
   wire [BB-1:0]  wd_j = wd_k[BB-1:0];

   always @(posedge clk) begin
      if (vb_v) wb_dat[rp_wbi[RL-1]] <= b_rdata;

      // the read pipe
      rp_beat[0] <= b_free ? 2'd0 : hs_beat;
      rp_uid[0]  <= b_free ? u_q_id : hs_uid;
      rp_wbi[0]  <= wf;
      rp_a[0]    <= b_addr;
      for (i = 1; i < RL; i = i + 1) begin
         rp_beat[i] <= rp_beat[i-1]; rp_uid[i] <= rp_uid[i-1]; rp_wbi[i] <= rp_wbi[i-1]; rp_a[i] <= rp_a[i-1];
      end

      // a hit's remaining reads
      if (b_hrd) begin hs_a <= {k_way, q_set}; hs_uid <= u_q_id; hs_beat <= 2'd1; end
      else if (!b_free) hs_beat <= hs_beat + 2'd1;

      if (m_new) begin
         m_dl[mf] <= q_dl;  m_way[mf] <= k_way;  m_uid[mf] <= u_q_id;  m_fw[mf] <= a_rd;
      end
      if (b_vrd) wb_dl[wf] <= {k_vtag, q_set};

      // up: a hit's beat, else the queue's head, else a fill's beat as it arrives
      u_w_id <= u_q_id;
      if (hb_v) begin
         u_r_id <= rp_uid[RL-1];  u_r_beat <= hb_beat;  u_r_last <= (hb_beat == 2'd3);
         u_r_data <= b_rdata[{hb_beat, 7'd0} +: 128];
      end else if (ff_pop) begin
         {u_r_id, u_r_beat, u_r_last, u_r_data} <= ff_hd;
      end else begin
         u_r_id <= m_uid[fm];  u_r_beat <= d_r_beat;  u_r_last <= d_r_last;  u_r_data <= d_r_data;
      end

      // down: an MSHR's read, else a write-back
      if (d_load) begin
         d_q_we   <= ~mp_any;
         d_q_id   <= mp_any ? {{(IDW-MB){1'b0}}, mp} : NMI + {{(IDW-BB){1'b0}}, wp};
         d_q_addr <= line_of(mp_any ? m_dl[mp] : wb_dl[wp]);
      end
      // the write-back's data loads whenever the port is free, not only when something is sent:
      // at a send it is the same line, and unsent it is never read -- and its 512 enables no longer
      // wait on the MSHRs' state (m_v -> mp_any -> d_load: 280 endpoints, all route)
      if (~d_q_valid | d_q_ready) d_q_wdata <= wb_dat[wp];

      if (reset) begin
         st <= S_INIT;  ic <= {SB{1'b0}};
         m_v <= {NM{1'b0}};  m_sent <= {NM{1'b0}};
         wb_v <= {NWB{1'b0}};  wb_rdy <= {NWB{1'b0}};  wb_sent <= {NWB{1'b0}};
         hs_n <= 4'd0;  rp_v <= {RL{1'b0}};  rp_h <= {RL{1'b0}};
         ff_n <= {(FB+1){1'b0}};  ff_wp <= {FB{1'b0}};  ff_rp <= {FB{1'b0}};
         u_r_valid <= 1'b0;  u_w_valid <= 1'b0;  d_q_valid <= 1'b0;  xb_v <= 1'b0;
      end else begin
         case (st)
            S_INIT: begin ic <= ic + 1'b1; if (&ic) st <= S_LK0; end
            S_LK0:  if (s_valid | u_q_valid) st <= S_LK1;
            S_LK1:  st <= S_ACT;
            S_ACT:  if (k_cf | a_go) st <= S_LK0;
            default: $fatal(1, "rv_l2: state %0d", st);
         endcase

         rp_v <= {rp_v[RL-2:0], b_hrd | b_vrd | ~b_free};
         rp_h <= {rp_h[RL-2:0], b_hrd | ~b_free};
         if (b_hrd) hs_n <= 4'd3; else if (!b_free) hs_n <= hs_n - 4'd1;

         // MSHRs: taken by a miss, sent below, freed by the fill's last beat
         if (m_new) begin m_v[mf] <= 1'b1; m_sent[mf] <= 1'b0; end
         if (d_load && mp_any) m_sent[mp] <= 1'b1;
         if (f_v && d_r_last) m_v[fm] <= 1'b0;
         // write-backs: taken by a dirty victim, filled RL cycles on, sent, freed by the done
         if (b_vrd) begin wb_v[wf] <= 1'b1; wb_rdy[wf] <= 1'b0; wb_sent[wf] <= 1'b0; end
         if (vb_v) wb_rdy[rp_wbi[RL-1]] <= 1'b1;
         if (d_load && !mp_any) wb_sent[wp] <= 1'b1;
         if (d_w_valid) wb_v[wd_j] <= 1'b0;

         // the fill-beat queue
         if (ff_push) ff_wp <= ff_wp + 1'b1;
         if (ff_pop)  ff_rp <= ff_rp + 1'b1;
         ff_n <= ff_n + {{FB{1'b0}}, ff_push} - {{FB{1'b0}}, ff_pop};

         // a flushed line's block
         if (s_ack && !s_keep) begin xb_v <= 1'b1;  xb_dl <= q_dl; end
         else if (s_rel) xb_v <= 1'b0;

         u_r_valid <= hb_v | ff_pop | f_byp;
         u_w_valid <= b_wr;
         if (d_load) d_q_valid <= 1'b1; else if (d_q_ready) d_q_valid <= 1'b0;
      end
   end

   // ---------------- invariants ----------------
   always @(posedge clk) if (!reset) begin
      if (u_r_valid && !u_r_ready) $fatal(1, "rv_l2: a read beat going up was refused (id %0d)", u_r_id);
      if (u_w_valid && !u_w_ready) $fatal(1, "rv_l2: a write done going up was refused (id %0d)", u_w_id);
      if (f_v && (d_r_id >= NM || !m_v[fm] || !m_sent[fm]))
         $fatal(1, "rv_l2: a read beat from below for id %0d, no MSHR's read in flight", d_r_id);
      if (d_w_valid && (d_w_id < NM || d_w_id >= NM + NWB || !wb_v[wd_j] || !wb_sent[wd_j]))
         $fatal(1, "rv_l2: a write done from below for id %0d, no write-back in flight", d_w_id);
      if (st == S_LK1 && (|(t_hit & (t_hit - 1'b1))))
         $fatal(1, "rv_l2: line %h hits ways %b of set %h", q_dl, t_hit, q_set);
      if (st == S_LK1 && (|(q_mhit & (q_mhit - 1'b1))))
         $fatal(1, "rv_l2: line %h is in two MSHRs (%b)", q_dl, q_mhit);
      if (ff_n > FD || (ff_push && ff_n == FD))
         $fatal(1, "rv_l2: the fill-beat queue overflows (%0d)", ff_n);
      if (hs_n > 4'd3) $fatal(1, "rv_l2: a hit with %0d reads left", hs_n);
      if (a_go && !k_hit && q_busy[k_way])
         $fatal(1, "rv_l2: line %h takes way %0d of set %h, which an MSHR is filling", q_dl, k_way, q_set);
      if ((st == S_LK1 || st == S_ACT) && !(k_sn ? s_valid : u_q_valid))
         $fatal(1, "rv_l2: the %s in hand went away", k_sn ? "snoop" : "request");
      if (s_ack && !s_keep && xb_v) $fatal(1, "rv_l2: line %h flushed while line %h is still blocked", q_dl, xb_dl);
      if (s_rel && !xb_v) $fatal(1, "rv_l2: a flush's release with no line blocked");
      // a fill never writes a way whose victim is still being read out
      for (i = 0; i < RL; i = i + 1)
         if (f_v && rp_v[i] && !rp_h[i] && rp_a[i] == a_addr)
            $fatal(1, "rv_l2: a fill writes line %h of the array while its victim is read out", a_addr);
   end
endmodule
`default_nettype wire
