`default_nettype none
// rv_icache: the read-only VHPR instruction cache (docs/VHPR.md), one 16-byte pair per cycle
// (docs/PLAN-2026-09-24-frontend-stage4.md, increment 0).
//
// GEOMETRY. SIZE_KB (64) in 2 ways of 64-byte lines. A pair is a 16-byte-aligned quarter of a
// line (asserted), so a pair is one line's lookup, and each way's data is two block-RAM banks,
// the even and the odd 8-byte chunks, read at the same row.
//
// HIT. Virtual only: valid, the virtual tag and the request's epoch -- no translation and no
// physical tag on the hit path (docs/VHPR.md), so an I$ hit never waits on the iTLB.
//
// RECONCILE is the MISS path. A virtual miss first compares the same set's two physical tags
// against the request's PA: a match holds the right bytes under a changed mapping, so it is
// re-stamped with the request's virtual tag and epoch (one cycle) and the request replays. Only
// when neither matches is the line filled. The PA is needed only there; today every request still
// carries it (rd_pa), and the translate-on-miss port replaces it with Stage 4's fetch stream.
//
// PIPELINE. A request is taken (rd_ack) in any cycle the door is open -- a register -- and its
// banks are read that cycle; the next cycle compares the tags and registers the response. A new
// request can be taken every cycle. When a lookup misses, the request taken in the same cycle
// (at most one) waits in the skid; the missed line is filled, and both replay in order, so every
// taken request is answered, in order.
//
// MISS. The missing line comes from the prefetch buffer if it holds it (matched by physical
// line), else from L2 (a one-cycle request pulse, the address held until the ack). It installs
// over four cycles, a row of each bank per cycle, and its tags last; the victim is an invalid way,
// else the set's round-robin bit.
//
// EPOCHS. Every request carries the epoch it was taken in. Its lookup hits virtually only a line of
// THAT epoch, and a fill for it installs stamped with that epoch: a request taken before a mapping
// change never sees a line of the new mapping, and its line never becomes current -- it can only
// reconcile by physical tag, whose bytes are right under any mapping (docs/VHPR.md: a fill
// outstanding across an invalidation must not install a current line).
//
// PREFETCH. One line, matched by physical address: after a fill of line L it fetches L+1 when L+1
// is in the same 4 KiB page, and a miss that finds its line there installs it without an L2 read.
// A flush drops the buffer, and a prefetch in flight across a flush lands dead.
//
// INVALIDATION. fence.i (inv_req), an epoch wrap and reset clear every valid bit, one set per
// cycle; inv_busy rises the cycle it is requested and falls when the scan is done. A mapping
// change (ep_bump) advances the epoch: lines stay resident and reconcile by physical tag.
module rv_icache #(
   parameter SIZE_KB = 64,
   parameter HW      = 8,          // the pair in halfwords: 16 bytes
   parameter RTW     = 4,          // the requester's opaque tag
   parameter VAW     = 39,         // virtual address bits the tag covers (Sv39)
   parameter PGW     = 39          // physical address bits the reconcile tag covers
) (
   input  wire              clk,
   input  wire              reset,
   input  wire              rd_req,
   input  wire [63:0]       rd_addr,     // VA of the pair's first byte, 16-byte aligned
   input  wire [63:0]       rd_pa,       // its PA
   input  wire [RTW-1:0]    rd_tag,
   output wire              rd_ack,      // taken this cycle
   output reg  [HW*16-1:0]  rd_data,     // the pair, first chunk low
   output reg               rd_valid,
   output reg  [63:0]       rd_resp_addr,
   output reg  [RTW-1:0]    rd_resp_tag,
   input  wire              inv_req,     // fence.i: clear every line
   input  wire              ep_bump,     // a mapping change: advance the epoch
   output reg               inv_busy,
   output reg               l2_req,
   output wire              l2_we,
   output reg  [57:0]       l2_addr,
   output wire [511:0]      l2_wdata,
   input  wire [511:0]      l2_rdata,
   input  wire              l2_ack,
   output wire              perf_access, // a lookup resolved this cycle
   output wire              perf_miss,   // ...and missed
   output wire [15:0]       err          // the integrity log's I$ bits (below)
);
   initial if (HW != 8) $fatal(1, "rv_icache: the pair is two 8-byte chunks (HW=8), got HW=%0d", HW);
   localparam OFFB = 6;
   localparam SETS = (SIZE_KB * 1024) / (2 * 64);
   localparam IB   = $clog2(SETS);
   localparam VTB  = VAW - OFFB - IB;          // virtual tag
   localparam PTB  = PGW - 12;                 // physical tag: PA above the 4 KiB page offset
   localparam EPW  = 2;
   localparam RB   = IB + 2;                   // bank row: {set, pair}
   assign l2_we    = 1'b0;
   assign l2_wdata = 512'd0;

   // ---------------------------------------------------------------- the arrays
   // tags, per way, in distributed RAM
   reg [VTB-1:0] vt0 [0:SETS-1], vt1 [0:SETS-1];
   reg [PTB-1:0] pt0 [0:SETS-1], pt1 [0:SETS-1];
   reg [EPW-1:0] ep0 [0:SETS-1], ep1 [0:SETS-1];
   reg           vl0 [0:SETS-1], vl1 [0:SETS-1];
   reg           rr  [0:SETS-1];               // round-robin victim
   integer i;
   initial for (i = 0; i < SETS; i = i + 1) begin vl0[i] = 1'b0; vl1[i] = 1'b0; rr[i] = 1'b0; end
   reg [EPW-1:0] cur_ep;
   initial cur_ep = {EPW{1'b0}};

   // ---------------------------------------------------------------- address split
   function [IB+VTB-1:0] line_of;   input [63:0] a; line_of = a[OFFB +: IB+VTB];   endfunction

   // ---------------------------------------------------------------- state
   localparam S_RUN = 3'd0, S_FILL = 3'd1, S_INST = 3'd2, S_RPL = 3'd3, S_CHK = 3'd4,
              S_RPS = 3'd5, S_SCAN = 3'd6, S_RST = 3'd7;
   reg [2:0]  st;
   reg        door;                            // the door is open this cycle (a register)
   // the lookup stage
   reg        s1_v;  reg [63:0] s1_va, s1_pa;  reg [RTW-1:0] s1_tag;  reg [EPW-1:0] s1_ep;
   // the skid (a request taken in the cycle a lookup missed) and the missed request
   reg        sk_v;  reg [63:0] sk_va, sk_pa;  reg [RTW-1:0] sk_tag;  reg [EPW-1:0] sk_ep;
   reg [63:0] f_va, f_pa;  reg [RTW-1:0] f_tag;
   // the fill: which line, where it goes, its epoch at issue, the data
   reg [IB+VTB-1:0] f_line;                    // the VIRTUAL line
   reg [57:0]       f_pline;                   // the PHYSICAL line
   reg [EPW-1:0]    f_ep;                      // the missed request's epoch
   reg              f_way;
   reg [1:0]        f_k;                       // install row
   reg [511:0]      f_data;
   reg              f_l2;                      // a demand L2 read is outstanding
   // the prefetch buffer
   reg              pf_val, pf_want, pf_infl, pf_drop;
   reg [57:0]       pf_addr, pf_next, pf_ia;
   reg [511:0]      pf_line;
   // invalidation
   reg              inv_pend;
   reg [IB:0]       scan;

   assign rd_ack = rd_req & door;

   // ---------------------------------------------------------------- the door and the banks
   // What the banks read this cycle: a new request, or a replay (the missed request, then the skid).
   wire        rp_f  = (st == S_RPL);
   wire        rp_s  = (st == S_RPS);
   wire [63:0] a_va  = rp_f ? f_va  : rp_s ? sk_va  : rd_addr;
   wire [63:0] a_pa  = rp_f ? f_pa  : rp_s ? sk_pa  : rd_pa;
   wire [RTW-1:0] a_tag = rp_f ? f_tag : rp_s ? sk_tag : rd_tag;
   wire [EPW-1:0] a_ep  = rp_f ? f_ep  : rp_s ? sk_ep  : cur_ep;
   wire        a_take = rp_f | rp_s | rd_ack;
   // data: per way, the even and the odd 8-byte chunk of a pair, in two 64-bit banks read at one
   // row (bank = way*2 + parity)
   wire [IB+VTB-1:0] a_line = line_of(a_va);
   wire [RB-1:0]  bk_ra = {a_line[IB-1:0], a_va[5:4]};
   wire [RB-1:0]  bk_wa = {f_line[IB-1:0], f_k};
   wire [63:0]    bk_rd [0:3];
   genvar gb;
   generate for (gb = 0; gb < 4; gb = gb + 1) begin : bank
      smolrv64_sdpram #(.ADDR_WIDTH(RB), .DATA_WIDTH(64), .READ_LATENCY(1)) u_bank
        (.clock(clk), .rd_addr(bk_ra), .rd_data(bk_rd[gb]),
         .wr_en((st == S_INST) & (f_way == gb[1])), .wr_addr(bk_wa),
         .wr_data(f_data[{f_k, gb[0]} * 64 +: 64]));             // chunk 2k (even) or 2k+1 (odd)
   end endgenerate

   // ---------------------------------------------------------------- the lookup
   wire [IB+VTB-1:0] m_line = line_of(s1_va);      // the line, and the one a miss fills
   wire [IB-1:0]  s1_set = m_line[IB-1:0];
   wire [VTB-1:0] s1_vtg = m_line[IB +: VTB];
   wire [PTB-1:0] ptg    = s1_pa[12 +: PTB];
   wire h0   = vl0[s1_set] & (vt0[s1_set] == s1_vtg) & (ep0[s1_set] == s1_ep);
   wire h1   = vl1[s1_set] & (vt1[s1_set] == s1_vtg) & (ep1[s1_set] == s1_ep);
   wire hit  = h0 | h1;
   wire miss = s1_v & ~hit;
   wire [57:0]       m_pl   = s1_pa[63:OFFB];
   // the reconcile probe: the missing line's set, both ways, by physical tag
   wire [IB-1:0]     m_set  = m_line[IB-1:0];
   wire              rc0    = vl0[m_set] & (pt0[m_set] == ptg);
   wire              rc1    = vl1[m_set] & (pt1[m_set] == ptg);
   assign perf_access = s1_v;
   assign perf_miss   = miss;

   // the victim for the line being filled
   wire [IB-1:0] f_set = f_line[IB-1:0];
   wire          v_way = ~vl0[f_set] ? 1'b0 : ~vl1[f_set] ? 1'b1 : rr[f_set];

   // ---------------------------------------------------------------- the tag write port
   // ONE write statement per array (one write port each): the scan clears both ways' valid bits;
   // an install writes the victim way's tags, stamped with the missed request's epoch.
   // A re-stamp (S_RST) writes the reconciled way's virtual tag and epoch only.
   wire          t_inst = (st == S_INST) & (f_k == 2'd3);
   wire          t_rst  = (st == S_RST);
   wire          t_scan = (st == S_SCAN);
   wire          t_vt   = t_inst | t_rst;
   wire [IB-1:0] t_set  = t_scan ? scan[IB-1:0] : f_set;
   wire          t_vld  = ~t_scan;
   always @(posedge clk) begin
      if (t_scan | (t_inst & ~f_way)) vl0[t_set] <= t_vld;
      if (t_scan | (t_inst &  f_way)) vl1[t_set] <= t_vld;
      if (t_vt & ~f_way) begin vt0[t_set] <= f_line[IB +: VTB]; ep0[t_set] <= f_ep; end
      if (t_vt &  f_way) begin vt1[t_set] <= f_line[IB +: VTB]; ep1[t_set] <= f_ep; end
      if (t_inst & ~f_way) pt0[t_set] <= f_pline[12-OFFB +: PTB];
      if (t_inst &  f_way) pt1[t_set] <= f_pline[12-OFFB +: PTB];
      if (t_inst) rr[t_set] <= ~f_way;
   end

   // ---------------------------------------------------------------- the machine
   always @(posedge clk) begin
      if (reset) begin
         st <= S_RUN;  door <= 1'b0;  s1_v <= 1'b0;  sk_v <= 1'b0;  rd_valid <= 1'b0;
         l2_req <= 1'b0;  f_l2 <= 1'b0;  pf_val <= 1'b0;  pf_want <= 1'b0;  pf_infl <= 1'b0;
         pf_drop <= 1'b0;  inv_pend <= 1'b1;  inv_busy <= 1'b1;  cur_ep <= {EPW{1'b0}};
      end else begin
         rd_valid <= 1'b0;
         l2_req   <= 1'b0;
         if (inv_req) begin inv_pend <= 1'b1; inv_busy <= 1'b1; end
         if (ep_bump) begin
            cur_ep <= cur_ep + 1'b1;
            if (cur_ep == {EPW{1'b1}}) begin inv_pend <= 1'b1; inv_busy <= 1'b1; end
         end

         // the lookup stage takes whatever the banks were addressed for this cycle
         s1_v <= 1'b0;
         if (a_take & ~(miss & rd_ack)) begin s1_v <= 1'b1; s1_va <= a_va; s1_pa <= a_pa; s1_tag <= a_tag; s1_ep <= a_ep; end
         if (rp_s) sk_v <= 1'b0;
         // a hit answers; a miss parks the request (and a request taken this cycle in the skid)
         if (s1_v & hit) begin
            rd_valid <= 1'b1;
            rd_data  <= h1 ? {bk_rd[3], bk_rd[2]} : {bk_rd[1], bk_rd[0]};
            rd_resp_addr <= s1_va;  rd_resp_tag <= s1_tag;
         end
         if (miss) begin
            f_va <= s1_va;  f_pa <= s1_pa;  f_tag <= s1_tag;
            f_line <= m_line;  f_pline <= m_pl;  f_ep <= s1_ep;
            if (rd_ack) begin sk_v <= 1'b1; sk_va <= rd_addr; sk_pa <= rd_pa; sk_tag <= rd_tag; sk_ep <= cur_ep; end
            // the line is resident under another mapping: re-stamp it; else fill it
            if (rc0 | rc1) begin f_way <= rc1; st <= S_RST; end
            else st <= S_FILL;
         end

         case (st)
           S_RUN: if (~miss & inv_pend & ~s1_v & ~sk_v & ~pf_infl) begin
                     st <= S_SCAN;  scan <= {(IB+1){1'b0}};  inv_pend <= 1'b0;
                  end
           S_FILL: begin
              // the line is in the prefetch buffer: take it. A prefetch in flight holds the port,
              // so a demand read waits for it -- and it may be the very line.
              if (~f_l2 & ~pf_infl) begin
                 if (pf_val & (pf_addr == f_pline)) begin
                    f_data <= pf_line;  f_k <= 2'd0;  f_way <= v_way;  st <= S_INST;
                    pf_val <= 1'b0;
                    if (f_pline[5:0] != 6'h3f) begin pf_want <= 1'b1; pf_next <= f_pline + 58'd1; end
                 end else begin
                    l2_req <= 1'b1;  l2_addr <= f_pline;  f_l2 <= 1'b1;
                 end
              end
              if (f_l2 & l2_ack) begin
                 f_l2 <= 1'b0;  f_data <= l2_rdata;  f_k <= 2'd0;  f_way <= v_way;  st <= S_INST;
                 if (f_pline[5:0] != 6'h3f) begin pf_want <= 1'b1; pf_next <= f_pline + 58'd1; end
              end
           end
           S_INST: begin
              f_k <= f_k + 2'd1;
              if (f_k == 2'd3) st <= S_RPL;
           end
           S_RST: st <= S_RPL;                    // the reconciled way took its new tag
           S_RPL: st <= S_CHK;                    // the missed request re-reads the banks
           S_CHK: if (~miss) st <= sk_v ? S_RPS : S_RUN;   // (a miss restarts S_FILL above)
           S_RPS: st <= S_RUN;                    // the skid re-reads the banks
           S_SCAN: begin
              scan <= scan + 1'b1;
              if (scan == SETS[IB:0] - 1'b1) begin st <= S_RUN; inv_busy <= inv_pend; end
           end
           default: if (^st !== 1'bx) $fatal(1, "rv_icache: illegal state %0d", st);
         endcase

         // the door: open in S_RUN, and after the skid replays, unless a lookup missed or a flush
         // is waiting
         door <= ((st == S_RUN) | (st == S_RPS)) & ~miss & ~inv_pend & ~inv_req
               & ~(ep_bump & (cur_ep == {EPW{1'b1}}));

         // the prefetch: the port is free when no demand read is outstanding or about to issue
         if (pf_want & ~pf_infl & ~f_l2 & ~l2_req & (st != S_FILL) & (st != S_SCAN)) begin
            l2_req <= 1'b1;  l2_addr <= pf_next;  pf_ia <= pf_next;  pf_infl <= 1'b1;  pf_want <= 1'b0;
         end
         if (pf_infl & l2_ack & ~f_l2) begin
            pf_line <= l2_rdata;  pf_addr <= pf_ia;  pf_val <= ~pf_drop;  pf_infl <= 1'b0;  pf_drop <= 1'b0;
         end
         // a flush drops the buffer; a prefetch in flight across it lands dead
         if (st == S_SCAN) begin pf_val <= 1'b0; pf_want <= 1'b0; if (pf_infl) pf_drop <= 1'b1; end
      end
   end

   // ---------------------------------------------------------------- invariants
   // Always on; the integrity log carries the same conditions (rv_errlog bits 16..31).
   wire e_dual   = s1_v & h0 & h1;                                       // one line in both ways
   wire e_ack    = l2_ack & ~f_l2 & ~pf_infl;                            // an L2 answer nobody asked for
   wire e_align  = rd_ack & (rd_addr[3:0] != 4'd0);                      // a pair not 16-byte aligned
   wire e_skid   = miss & rd_ack & sk_v;                                 // a second request into a full skid
   wire e_pa     = rd_ack & (rd_pa[11:0] != rd_addr[11:0]);              // VA and PA disagree in the page offset
   wire e_two    = f_l2 & pf_infl;                                       // a demand read and a prefetch both outstanding
   wire h_dup0   = vl0[f_set] & (vt0[f_set] == f_line[IB +: VTB]) & (ep0[f_set] == f_ep);
   wire h_dup1   = vl1[f_set] & (vt1[f_set] == f_line[IB +: VTB]) & (ep1[f_set] == f_ep);
   wire e_dup    = t_vt & ((~f_way & h_dup1) | (f_way & h_dup0));        // stamping a line the other way holds
   wire e_rc2    = miss & rc0 & rc1;                                     // one physical line in both ways
   wire [15:0] e_now = {7'd0, e_rc2, e_dup, e_two, e_pa, e_skid, e_align, e_ack, 1'b0, e_dual};
   reg  [15:0] err_q;
   always @(posedge clk) err_q <= reset ? 16'd0 : e_now;
   assign err = err_q;
   always @(posedge clk) if (!reset) begin
      if (e_dual)  $fatal(1, "rv_icache: a line hits in both ways (va %h)", s1_va);
      if (e_ack)   $fatal(1, "rv_icache: an L2 answer with no read outstanding");
      if (e_align) $fatal(1, "rv_icache: a pair not 16-byte aligned (va %h)", rd_addr);
      if (e_skid)  $fatal(1, "rv_icache: a request into a full skid");
      if (e_pa)    $fatal(1, "rv_icache: VA %h and PA %h disagree in the page offset", rd_addr, rd_pa);
      if (e_two)   $fatal(1, "rv_icache: a demand read and a prefetch outstanding together");
      if (e_dup)   $fatal(1, "rv_icache: stamping a line the other way already holds (va line %h)", f_line);
      if (e_rc2)   $fatal(1, "rv_icache: one physical line resident in both ways (va %h pa %h)", s1_va, s1_pa);
   end
endmodule
`default_nettype wire
