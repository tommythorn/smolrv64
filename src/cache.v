`default_nettype none

// Unified skewed-2-way PIPT L1 cache (SmooolRV64 memory subsystem).
//
// ONE module, two roles via parameters -- the I$ (WRITABLE=0, fill-only) and the D$
// (WRITABLE=1, write-back or write-through). Store/dirty/writeback hardware is gated
// by WRITABLE so the I$ instance prunes it.
//
// PIPT, 2-way SKEW-associative (way1 XORs low tag bits into the index; a victim's base
// index is recovered as skewed_index ^ victim_tag).
//
// DATA STORAGE = even/odd BANKS of width BANKW (= RDW), per way -> 2*WAYS smolrv64_sdpram
// (1R1W BRAM). A line is CHUNKS=LINEB/BANKW chunks; even chunks in the even bank, odd in
// the odd bank. Because BANKW == the read width, any RDW-bit read at any byte offset spans
// AT MOST TWO consecutive chunks (off%CBY + RDB <= 2*CBY), and two consecutive chunks have
// opposite parity -> they live in the two different banks and come out in ONE read. The
// 2*BANKW window is byte-shifted (off%CBY) down to RDW -- a minimal mux, no full-line read.
// The I$ uses BANKW=128 (256b window for a misaligned 128-bit / 4-wide fetch); the D$ uses
// BANKW=64 (= store granularity, so a byte-masked store is a read-modify-write of ONE chunk,
// no cross-bank RMW). A store touching two chunks writes two DIFFERENT (even+odd) banks ->
// single write each. Fill / writeback / write-through touch the whole line over HALF cycles.
//
// LINE-CROSSING handled INTERNALLY via the two-phase lookup (phase0=line0, phase1=line1).
// Banks are synchronous (READ_LATENCY=1): an address presented in one cycle is captured
// the next, so multi-word reads serialize a pair (even+odd) per two cycles.
//
// WRITE-BACK BUFFER (D$): a dirty victim is captured into a single-entry buffer and
// its L2 write drains in the background -- the demand fill no longer waits for it.
// See the WBUF declaration block for the full policy (bounce, NC/CBO/flush drains).
module cache #(
   parameter PAW      = 34,
   parameter SIZE_KB  = 128,
   parameter WAYS     = 2,
   parameter LINEB    = 512,
   parameter RDW      = 64,
   parameter WDW      = 64,
   parameter OFFB     = 6,
   parameter WRITABLE = 1,
   parameter WRTHRU   = 0,
   parameter PREFETCH = 0,         // next-line prefetch (I$): single-line stream buffer
   parameter PERF_ID  = 0          // perf-trace cache id (0=I$, 1=D$); see perf block
) (
   input  wire             clk,
   input  wire             reset,
   input  wire             rd_req,
   output wire             rd_rdy,      // request-channel ready: rd_req&rd_rdy = accepted this edge
   input  wire [PAW-1:0]   rd_addr,
   output reg  [RDW-1:0]   rd_data,
   output reg              rd_valid,
   output reg  [PAW-1:0]   rd_resp_addr,
   input  wire             rd_uncached, // Svpbmt: this read is NC/IO -> don't keep the line (flush-around)
   input  wire             wr_req,
   input  wire [PAW-1:0]   wr_addr,
   input  wire [WDW-1:0]   wr_data,
   input  wire [WDW/8-1:0] wr_mask,
   output reg              wr_ack,
   input  wire             wr_uncached, // Svpbmt: this store is NC/IO -> write through to L2 + invalidate
   input  wire             cbo_req,     // Zicbom/Zicboz: this write-port request is a cache-maintenance op
   input  wire             cbo_zero,    // cbo.zero: install a zero line (else clean/flush/inval)
   input  wire             cbo_keep,    // cbo.clean: writeback but keep line valid (else invalidate)
   input  wire             inv_req,
   input  wire             inv_clean,   // inv_req variant: write back dirty lines but KEEP them
                                        // valid+clean (PTW coherency on sfence; not a full flush)
   output reg              inv_busy,
   output reg              l2_req,
   output reg              l2_we,
   output reg  [PAW-OFFB-1:0] l2_addr,
   output reg  [LINEB-1:0] l2_wdata,
   input  wire [LINEB-1:0] l2_rdata,
   input  wire             l2_ack,
   output wire             perf_access, // 1-cycle: a line lookup resolved (hit or miss) this cycle
   output wire             perf_miss    // 1-cycle: that lookup missed -> Zihpm cache-miss event
);
   localparam WORDB = LINEB/8;
   localparam SETS  = (SIZE_KB*1024)/(WAYS*WORDB);
   localparam IDXB  = $clog2(SETS);
   localparam PTAGB = PAW - IDXB - OFFB;
   localparam RDB   = RDW/8;
   localparam WRB   = WDW/8;
   localparam NW    = WAYS*SETS;
   localparam FW    = $clog2(NW);

   localparam BANKW  = RDW;
   localparam CBY    = BANKW/8;
   localparam CHUNKS = LINEB/BANKW;
   localparam HALF   = CHUNKS/2;
   localparam CHB    = $clog2(CHUNKS);
   localparam PAIRB  = (HALF<2) ? 1 : $clog2(HALF);
   localparam LZB    = $clog2(CBY);
   localparam BAW    = IDXB + PAIRB;

   reg [PTAGB-1:0] tagm [0:NW-1];
   reg             valm [0:NW-1];
   reg             dirm [0:NW-1];
   reg             vicm [0:SETS-1];
   integer i;
   initial begin
      for (i=0;i<NW;i=i+1)   begin valm[i]=1'b0; dirm[i]=1'b0; tagm[i]=0; end
      for (i=0;i<SETS;i=i+1) vicm[i]=1'b0;
   end

   function [IDXB-1:0]  base_idx; input [PAW-1:0] a; base_idx = a[OFFB +: IDXB];      endfunction
   function [PTAGB-1:0] tag_of;   input [PAW-1:0] a; tag_of   = a[OFFB+IDXB +: PTAGB]; endfunction
   function [IDXB-1:0]  way_idx; input integer w; input [PAW-1:0] a;
      reg [PTAGB-1:0] t;
      begin t = tag_of(a); way_idx = (w==0) ? base_idx(a) : (base_idx(a) ^ t[IDXB-1:0]); end
   endfunction
   function [FW-1:0] flat; input integer w; input [IDXB-1:0] ix; flat = (w!=0)*SETS + ix; endfunction

   // ---- data banks: index b = way*2 + parity ----
   reg  [BAW-1:0]   bk_rdaddr [0:2*WAYS-1];
   wire [BANKW-1:0] bk_rddata [0:2*WAYS-1];
   reg              bk_wren   [0:2*WAYS-1];
   reg  [BAW-1:0]   bk_wraddr [0:2*WAYS-1];
   reg  [BANKW-1:0] bk_wrdata [0:2*WAYS-1];
   // Each logical bank is SLICED into <=64-bit physical sdpram instances: one RAMB36
   // SDP word is 72b, and a wider single instance (BRAM width-cascade) is the geometry
   // that killed the wide-I$ bit (d3d09157) -- the behavioral sdpram cannot validate
   // it in sim, and smolrv64_sdpram hard-fails synthesis on it. Slicing keeps every
   // physical RAM in the hardware-proven class: the D$ (BANKW=64) is one slice, the
   // I$ (BANKW=128) two. Shared address/enable, disjoint data bits -- bit-identical.
   localparam SLICEW  = 64;
   localparam NSLICES = (BANKW + SLICEW-1)/SLICEW;
   genvar gb, gs;
   generate for (gb=0; gb<2*WAYS; gb=gb+1) begin : banks
      for (gs=0; gs<NSLICES; gs=gs+1) begin : slice
         localparam W = (gs == NSLICES-1) ? (BANKW - gs*SLICEW) : SLICEW;
         smolrv64_sdpram #(.ADDR_WIDTH(BAW), .DATA_WIDTH(W), .READ_LATENCY(1)) u_bank
           (.clock(clk), .rd_addr(bk_rdaddr[gb]), .rd_data(bk_rddata[gb][gs*SLICEW +: W]),
            .wr_en(bk_wren[gb]), .wr_addr(bk_wraddr[gb]), .wr_data(bk_wrdata[gb][gs*SLICEW +: W]));
      end
   end endgenerate

   // ---- request regs ----
   reg            r_is_wr;
   reg            r_uncached;          // Svpbmt: current access is NC/IO (flush-around)
   reg            r_cbo, r_cbo_zero, r_cbo_keep;   // Zicbom/Zicboz maintenance op latched at accept
   reg [PAW-1:0]  r_addr;
   reg [WDW-1:0]  r_wdata;
   reg [WRB-1:0]  r_wmask;
   reg [OFFB-1:0] r_off;
   reg            r_span;
   wire [PAW-1:0] line0 = {r_addr[PAW-1:OFFB], {OFFB{1'b0}}};
   wire [PAW-1:0] line1 = line0 + (1<<OFFB);

   wire [CHB-1:0]   clo    = r_off[OFFB-1 -: CHB];
   wire [LZB-1:0]   bwc    = r_off[LZB-1:0];
   wire [PAIRB-1:0] pair_lo = clo[CHB-1:1];
   wire [CHB-1:0]   chunk_hi = clo + 1'b1;                   // same-line high chunk of a spilling store
   wire [PAIRB-1:0] pair_hi  = chunk_hi[CHB-1:1];            // its pair index (PAIRB-wide: no concat overflow)
   wire [CHB-1:0]   chunk_e = clo[0] ? (clo + 1'b1) : clo;   // even-parity chunk of the window
   wire [CHB-1:0]   chunk_o = clo[0] ? clo : (clo + 1'b1);   // odd-parity chunk of the window
   wire [PAIRB-1:0] pair_e  = chunk_e[CHB-1:1];
   wire [PAIRB-1:0] pair_o  = chunk_o[CHB-1:1];
   wire             store_hi = r_is_wr & (({1'b0,bwc} + WRB) > CBY);  // store spills into high chunk

   // ---- skew lookup of cur_line ----
   reg            phase;
   reg [PAW-1:0]  cur_line;
   wire [IDXB-1:0]  ci0  = way_idx(0, cur_line);
   wire [IDXB-1:0]  ci1  = way_idx(1, cur_line);
   wire [PTAGB-1:0] ctag = tag_of(cur_line);
   wire hit0 = valm[flat(0,ci0)] & (tagm[flat(0,ci0)]==ctag);
   wire hit1 = valm[flat(1,ci1)] & (tagm[flat(1,ci1)]==ctag);
   wire       hit  = hit0 | hit1;
   wire       hway = hit1;
   wire [IDXB-1:0] cih = hit1 ? ci1 : ci0;

   reg            vw;  reg [IDXB-1:0] vi;
   // effective victim: during the MSHR install (msh_ins) the victim pinned at
   // allocation is used; vw/vi stay owned by a possibly-parked second request
   wire             eff_vw = (WRITABLE!=0 && WRTHRU==0 && msh_ins) ? msh_vw : vw;
   wire [IDXB-1:0]  eff_vi = (WRITABLE!=0 && WRTHRU==0 && msh_ins) ? msh_vi : vi;
   wire [FW-1:0]    vflat = flat(eff_vw?1:0, eff_vi);
   wire [PTAGB-1:0] vtag  = tagm[vflat];
   wire [IDXB-1:0]  vbase = eff_vw ? (eff_vi ^ vtag[IDXB-1:0]) : eff_vi;

   reg            flush_clean;        // current flush is clean-only (keep lines valid)
   reg [FW:0]     fscan;
   wire           fway  = fscan[IDXB];
   wire [IDXB-1:0] fidx  = fscan[IDXB-1:0];
   wire [PTAGB-1:0] ftag  = tagm[fscan[FW-1:0]];
   wire [IDXB-1:0]  fbase = fway ? (fidx ^ ftag[IDXB-1:0]) : fidx;

   // ---- window + line buffers ----
   reg [BANKW-1:0] wlo, whi;
   wire [2*BANKW-1:0] win    = {whi, wlo};
   wire [2*BANKW-1:0] win_sh = win >> (bwc*8);
   // live-window variant for the S_CHECK fast read delivery (same bytes that are
   // being registered into wlo/whi this edge)
   wire [BANKW-1:0]   fwlo    = clo[0] ? bk_rddata[hway*2+1] : bk_rddata[hway*2+0];
   wire [BANKW-1:0]   fwhi    = clo[0] ? bk_rddata[hway*2+0] : bk_rddata[hway*2+1];
   wire [2*BANKW-1:0] fast_sh = {fwhi, fwlo} >> (bwc*8);
   reg [LINEB-1:0] linebuf;
   reg [PAIRB:0]   pc;
   reg [IDXB-1:0]  wb_idx;  reg wb_way;            // line currently streamed for WB/WT/flush
   reg             w0_way;  reg [IDXB-1:0] w0_idx; // line0 hit way/idx (for the span store)
   reg [PAW-OFFB-1:0] wb_laddr;                    // L2 line address for the streamed writeback

   localparam [4:0]
      S_IDLE=0, S_LOOK=1, S_CHECK=2, S_FIN=3, S_SPANW=4,
      S_WB=5, S_WBR=6, S_WBW=7, S_WBI=8, S_WBA=9,
      S_FILL=10, S_FILLW=11, S_FILLI=12,
      S_WTR=13, S_WTW=14, S_WTI=15, S_WTA=16,
      S_FLUSH=17, S_FLUSHR=18, S_FLUSHW=19, S_FLUSHI=20, S_FLUSHA=21,
      S_NCI=22, S_ZFILL=23, S_PFI=24, S_WBBI=25, S_MSHI=26;

   // ---- write-back buffer (WRITABLE&&!WRTHRU; the D$) ----
   // A dirty victim is CAPTURED here (S_WBR/S_WBW) instead of being pushed to L2 on
   // the miss path: the fill proceeds immediately and the L2 write drains in the
   // background on idle port cycles, or while the FSM is parked on this buffer.
   // Single entry, so L2 write order == capture order by construction. A miss on
   // the buffered line BOUNCES it back into the cache as DIRTY (S_WBBI -- its data
   // never reached L2, so the dirty bit restores the invariant); NC fills and CBO
   // ops targeting it drain it first (their semantics publish data to L2), and a
   // flush holds inv_busy until the buffer is empty. wbb_val stays set while the
   // drain is in flight (wbb_infl); it clears only at the L2 ack.
   localparam WBUF = (WRITABLE != 0) && (WRTHRU == 0);
   reg                wbb_val, wbb_infl;
   reg [PAW-OFFB-1:0] wbb_addr;
   reg [LINEB-1:0]    wbb_data;

   // ---- miss-status holding register: hit-under-miss (WBUF configs) ----
   // A plain cacheable non-span miss parks HERE instead of holding the FSM: its
   // context moves to msh_* and the FSM returns to S_IDLE, serving hits and
   // stores under the outstanding miss (the read-hit pipe keeps streaming). The
   // fill is launched/caught by parallel engines on the L2 port (msh_launch /
   // msh_infl&l2_ack). When the data lands (msh_rdy), the INSTALL preempts at the
   // next idle or parked-S_CHECK boundary: victim capture (S_WB -- deferred to
   // install time, so stores that dirtied the victim under the miss are included)
   // then S_MSHI, which delivers straight out of msh_line (window cut
   // combinationally; a store-miss's bytes were merged at the catch) and streams
   // the line into the banks via S_FILLI. No re-lookup: r_* may hold a PARKED
   // second request the whole time (context split r_* vs msh_*), and cur_line is
   // never touched by the install, so the parked op resumes via S_LOOK
   // (msh_ret=1) and re-evaluates -- often hitting the just-installed line.
   // Serializing misses (CBO/NC/span/wbb-bounce) and a second plain miss park at
   // S_CHECK until the MSHR frees; flush waits for it at the inv arm.
   localparam HUM = WBUF;
   reg              msh_val, msh_infl, msh_rdy, msh_ins, msh_ret, msh_is_wr;
   reg  [PAW-1:0]   msh_addr;
   reg  [WDW-1:0]   msh_wdata;
   reg  [WRB-1:0]   msh_wmask;
   reg              msh_vw;  reg [IDXB-1:0] msh_vi;
   reg  [LINEB-1:0] msh_line;

   // ---- next-line prefetch (PREFETCH!=0; the I$): single-line stream buffer ----
   // A demand fill of line L arms a prefetch of L+1. The prefetch runs as a
   // PARALLEL engine borrowing the (otherwise idle) L2 port while the FSM is in
   // request-service states -- the I$ front door is never idle during streaming
   // (the frontend holds rd_req continuously), so an idle-launched prefetch
   // would simply starve. Interlock: S_FILL stalls while a prefetch is in
   // flight (one L2 round trip, the classic bounded cost) -- and if that
   // in-flight prefetch IS the missing line, S_FILL consumes it on landing
   // instead of re-reading. A later miss matching the buffer installs it via
   // S_PFI -> S_FILLI, skipping the L2 read entirely, and re-arms for the NEXT
   // line -- consumption-chained streaming pipelines fetch-of-L with
   // fill-of-L+1 (DISP_STATS: I$ starvation was 49% of boot cycles). The
   // buffer is a pure hint-holder: invalidated with the cache (fence.i), never
   // dirty, and a line installed from it is bit-identical to a demand L2 read.
   reg              pf_val, pf_want, pf_infl, pf_drop;
   reg [PAW-OFFB-1:0] pf_addr, pf_next, pf_ia;   // pf_ia = the address actually ISSUED:
                     // pf_next can be re-armed (S_PFI) while a fetch is in flight, so the
                     // ack must be stamped with the issued address, never the live pf_next
   reg [LINEB-1:0]  pf_line;
   localparam PF_EN = (PREFETCH != 0) && (WRITABLE == 0);  // engine is I$-shaped only: its
                     // interlocks assume the FSM's only L2 state is the S_FILL fill path
   // PF_EN implies WRITABLE==0: the pf-hit install path (S_PFI) skips S_WB, sound
   // only when a victim can never be dirty -- i.e. the I$.
   wire pf_hit = PF_EN & pf_val & (pf_addr == cur_line[PAW-1:OFFB])
               & ~r_uncached & ~r_cbo;
   reg [4:0] st;
   reg inv_pend;     // sticky: an inv_req that arrives while the cache is busy is remembered

   // ---- read request-channel ready + the read-hit pipeline take ----
   // The read port is a ready/valid pair on each end: request = rd_req/rd_rdy
   // (accepted at the edge where both are high), response = rd_valid + rd_data
   // tagged with rd_resp_addr. A client presents a request until the handshake and
   // then moves on; retracting or re-addressing an UNaccepted request is legal (the
   // cache samples only at the accepting edge), and a request held PAST its accept
   // is a NEW request -- same-address back-to-back reads are legal and stream.
   // chk_deliver is the fast-hit delivery edge: the one S_CHECK shape that both
   // delivers this cycle AND can take a successor (plain cacheable non-span read
   // hit). The successor is latched at that edge and the FSM stays in S_CHECK: one
   // hit per cycle at 2-cycle latency. Miss/write/span/NC/CBO/pending-invalidate
   // drop rd_rdy and fall back to S_IDLE, so every slow op drains the pipe and
   // runs today's FSM unchanged.
   wire chk_deliver = (st==S_CHECK) & hit & ~phase & ~r_cbo & ~r_span & ~r_is_wr & ~r_uncached;
   // msh_rdy gates accepts: the returned fill's install has priority at the next
   // idle/delivery boundary (it is the oldest operation in the cache)
   assign rd_rdy    = ~reset & ~inv_req & ~inv_pend & ~((HUM!=0) & msh_rdy)
                    & ((st==S_IDLE) | chk_deliver);
   wire pipe_take   = chk_deliver & rd_req & ~inv_req & ~inv_pend & ~((HUM!=0) & msh_rdy);

   // write-back buffer status + drain-issue policy: push on any idle-port cycle
   // (S_IDLE), or wherever the FSM is parked ON the buffer -- each park below has
   // its matching term here, so every park terminates at the drain's L2 ack.
   // The MSHR fill (demand) outranks the drain: wbb_do defers to msh_launch and
   // to an in-flight fill (single-outstanding L2 port).
   wire vic_dirty = (WRITABLE!=0) && (WRTHRU==0) && valm[vflat] && dirm[vflat];
   wire wbb_match = wbb_val && (wbb_addr == cur_line[PAW-1:OFFB]);
   wire msh_ok     = (HUM!=0) && !r_cbo && !r_uncached && !r_span && !phase && !wbb_match;
   wire msh_launch = (HUM!=0) && msh_val && !msh_infl && !msh_rdy && !l2_req && !wbb_infl
                   && st != S_WTI && st != S_WTA;   // states with FSM L2 traffic under the MSHR
   wire wbb_do    = WBUF && wbb_val && !wbb_infl && !l2_req
                  && !((HUM!=0) && (msh_infl || msh_launch))
                  && (  st == S_IDLE
                     || (st == S_WB    && vic_dirty)                // miss needs the buffer free
                     || (st == S_FILL  && wbb_match && r_uncached)  // NC fill must read L2 fresh
                     || (st == S_CHECK && r_cbo && wbb_match)       // CBO publishes to L2
                     || (st == S_FLUSH && fscan == NW));            // flush exit waits on drain
   // store-miss bytes merged into the caught fill line; read window cut from it
   reg [LINEB-1:0] msh_mrg;  integer mb;
   always @* begin
      msh_mrg = l2_rdata;
      if (msh_is_wr) begin
         for (mb=0; mb<WRB; mb=mb+1) if (msh_wmask[mb])
            msh_mrg[(msh_addr[OFFB-1:0]+mb)*8 +: 8] = msh_wdata[mb*8 +: 8];
      end
   end
   wire [LINEB-1:0] msh_shift = msh_line >> {msh_addr[OFFB-1:0], 3'b000};

`ifdef CACHE_PARITY
   // ---- data-array integrity check (ILA trigger source) ----------------------------
   // The behavioral sdpram (simulation) and XPM/BRAM (synthesis) are the ONE element of
   // the data path no simulation can validate -- and the board's corruption looks exactly
   // like "a read returned the wrong bytes". This gives the hardware a way to SAY SO at
   // the moment it happens, instead of leaving a kernel Oops millions of cycles later as
   // the only evidence (unreachable by any ILA pre-trigger depth).
   //
   // A parallel parity array (1 bit per bank word, distributed RAM -- no BRAM geometry
   // change, no data-path change) is written on every bank write and checked on every
   // bank read. A mismatch means the array handed back something other than what was
   // stored. `par_err` is a 1-cycle pulse for the ILA trigger; par_sticky/par_addr latch
   // the first failure for post-mortem readout.
   //
   // Deliberately opt-in (-DCACHE_PARITY): it costs LUTRAM and a read-path XOR reduce,
   // and this is a diagnostic bitstream, not the production one.
   reg  par_mem [0:2*WAYS-1][0:(1<<BAW)-1];
   reg  par_rd_v [0:2*WAYS-1];
   reg  par_exp  [0:2*WAYS-1];
   reg  [BAW-1:0] par_rd_a [0:2*WAYS-1];
   integer pb, pi;
   initial begin
      for (pb=0; pb<2*WAYS; pb=pb+1) begin
         par_rd_v[pb] = 1'b0; par_exp[pb] = 1'b0; par_rd_a[pb] = {BAW{1'b0}};
         for (pi=0; pi<(1<<BAW); pi=pi+1) par_mem[pb][pi] = 1'b0;
      end
   end
   reg par_err;  reg par_sticky;  reg [BAW-1:0] par_addr;  reg [2:0] par_bank;
   // fixed-width view for the debug bus (BAW is 12 here; consumers must not assume a width)
   wire [15:0] par_addr16 = {{(16-BAW){1'b0}}, par_addr};
   initial begin par_err=1'b0; par_sticky=1'b0; par_addr={BAW{1'b0}}; par_bank=3'd0; end
   always @(posedge clk) begin
      par_err <= 1'b0;
      for (pb=0; pb<2*WAYS; pb=pb+1) begin
         // write side: store the parity of every word written into a bank
         if (bk_wren[pb]) par_mem[pb][bk_wraddr[pb]] <= ^bk_wrdata[pb];
         // read side: the address presented this cycle yields data NEXT cycle
         // (READ_LATENCY=1), so carry the expectation forward one cycle.
         par_rd_a[pb] <= bk_rdaddr[pb];
         // NBA read of par_mem yields the OLD parity -- which is exactly what
         // read_first semantics return on the data side for a same-cycle write.
         par_exp [pb] <= par_mem[pb][bk_rdaddr[pb]];
         par_rd_v[pb] <= 1'b1;
         if (par_rd_v[pb] && (^bk_rddata[pb] != par_exp[pb])) begin
            par_err <= 1'b1;
            if (!par_sticky) begin
               par_sticky <= 1'b1; par_addr <= par_rd_a[pb]; par_bank <= pb[2:0];
            end
`ifndef SYNTHESIS
            $display("[cache id=%0d] PARITY ERROR bank=%0d addr=%h data=%h exp_par=%b",
                     PERF_ID, pb, par_rd_a[pb], bk_rddata[pb], par_exp[pb]);
`endif
         end
      end
   end
`endif

`ifndef SYNTHESIS
   // STORE-SLOT invariant: a store merge (S_FIN) must write the slot that actually
   // holds ITS line -- w0_way/w0_idx were captured at the LINE0 lookup, and the tag
   // there must equal the store's own tag. If it ever does not, the store is being
   // merged into an unrelated resident line: silent memory corruption that only
   // surfaces when that other line is read back (or written back to DDR), which is
   // exactly the board's failure shape. Fires at the corruption, not the symptom.
   always @(posedge clk) if (!reset && (WRITABLE != 0))
      // NOT for spanning stores: there w0_* legitimately names LINE0's slot while
      // cur_line has advanced to line1 (the high half goes via S_SPANW).
      if ((st == S_FIN) && r_is_wr && hit && !r_span && !phase && valm[flat(w0_way, w0_idx)]
          && (tagm[flat(w0_way, w0_idx)] != tag_of(cur_line)))
         $fatal(1, "[cache id=%0d] STORE-SLOT MISMATCH: store line=%h (tag=%h) merging into way=%0d idx=%h which holds tag=%h",
                PERF_ID, cur_line, tag_of(cur_line), w0_way, w0_idx, tagm[flat(w0_way, w0_idx)]);

   // SPAN-LOW invariant: a spanning store's LOW half must land in line0's own slot.
   // It used to be written at S_FIN from w0_way/w0_idx -- a slot named at the phase-0
   // lookup and then carried across the phase-1 lookup, which can miss and let an MSHR
   // install (victim pinned long before) reallocate that slot underneath the store.
   // That merged code bytes into an unrelated line, marked it dirty, and published the
   // word to DRAM under the innocent line's address. The write now happens at line0's
   // own live lookup, so this holds by construction; it stays as a regression guard.
   always @(posedge clk) if (!reset && (WRITABLE != 0))
      if ((st == S_CHECK) && !phase && hit && r_is_wr && r_span
          && (tagm[flat(hway, cih)] != tag_of(line0)))
         $fatal(1, "[cache id=%0d] SPAN-LOW SLOT MISMATCH: line0=%h (tag=%h) merging into way=%0d idx=%h which holds tag=%h",
                PERF_ID, line0, tag_of(line0), hway, cih, tagm[flat(hway, cih)]);

   // SPAN invariant: a line-crossing store writes its high half into the slot named by
   // the LIVE phase-1 lookup (hway/cih). If a fill/install lands between that lookup and
   // this write, hway/cih can name a slot that no longer holds line1 -- the high half then
   // lands in an unrelated line. Same class as the S_FIN check but on the path realistic
   // memory timing actually disturbs (the corrupted word sits at a line's last 8 bytes).
   always @(posedge clk) if (!reset && (WRITABLE != 0))
      if ((st == S_SPANW) && hit && valm[flat(hway, cih)]
          && (tagm[flat(hway, cih)] != tag_of(line1)))
         $fatal(1, "[cache id=%0d] SPAN-SLOT MISMATCH: line1=%h (tag=%h) writing way=%0d idx=%h which holds tag=%h (line0=%h)",
                PERF_ID, line1, tag_of(line1), hway, cih, tagm[flat(hway, cih)], line0);

   // LOST-DIRTY-LINE detector. The established failure is that a dirty line's data
   // never reaches DRAM: memory keeps serving the pre-store content while the cache
   // copy (holding the store) is quietly dropped. Track, per slot, whether the dirty
   // data has been CAPTURED into the write-back buffer since it was last dirtied; an
   // install over a still-uncaptured dirty slot destroys committed stores.
   reg cap_ok [0:NW-1];
   integer ci;
   initial for (ci = 0; ci < NW; ci = ci + 1) cap_ok[ci] = 1'b1;
   always @(posedge clk) if (!reset && (WRITABLE != 0) && (WRTHRU == 0)) begin
      if (d_we && d_wd)                       cap_ok[d_wa] <= 1'b0;   // slot just went dirty
      if ((st == S_WBW) && (pc == HALF-1))    cap_ok[flat(wb_way, wb_idx)] <= 1'b1;
      // The line being installed is the MSHR's when msh_ins is set, else cur_line --
      // comparing against the wrong one flags a legitimate refill of the same line.
      if ((st == S_FILLI) && (pc == 0) && valm[vflat] && dirm[vflat] && !cap_ok[vflat]
          && (tagm[vflat] != tag_of(((HUM!=0) && msh_ins) ? msh_addr : cur_line)))
         $fatal(1, "[cache id=%0d] LOST DIRTY LINE: installing %h over way=%b idx=%h holding tag=%h (dirty, never captured)\n   msh_ins=%b msh_vw=%b msh_vi=%h  vw=%b vi=%h  wbb(v=%b addr=%h)  msh(val=%b addr=%h is_wr=%b)",
                PERF_ID, cur_line, eff_vw, eff_vi, tagm[vflat],
                (HUM!=0) && msh_ins, msh_vw, msh_vi, vw, vi,
                wbb_val, {wbb_addr, {OFFB{1'b0}}}, msh_val, msh_addr, msh_is_wr);
   end

   // SINGLE-OUTSTANDING L2 invariant. Four independent users share one untagged L2
   // port -- the FSM fill (S_FILLW), the write-through/flush pushes (S_WTA/S_FLUSHA),
   // the MSHR launcher, the prefetch engine and the write-back drain -- and their
   // mutual exclusion is only implied by scattered state conditions. If two are ever
   // outstanding, BOTH catchers fire on the same untagged l2_ack and one of them
   // adopts the other's line: a cache line silently filled with another address's
   // bytes, which is precisely the corruption shape (line correct except for words
   // that belong somewhere else).
   wire [2:0] l2_out = {2'd0, (st == S_FILLW)} + {2'd0, (st == S_WTA)} + {2'd0, (st == S_FLUSHA)}
                     + {2'd0, ((HUM!=0) && msh_infl)} + {2'd0, (PF_EN && pf_infl)}
                     + {2'd0, (WBUF && wbb_infl)};
   always @(posedge clk) if (!reset && (l2_out > 3'd1))
      $fatal(1, "[cache id=%0d] TWO L2 TRANSACTIONS OUTSTANDING: st=%0d msh_infl=%b pf_infl=%b wbb_infl=%b (one untagged ack feeds both)",
             PERF_ID, st, msh_infl, pf_infl, wbb_infl);

   // WRITE-BACK ADDRESS check. The capture (S_WBR/S_WBW) reads the banks at
   // wb_way/wb_idx and the drain publishes them under wb_laddr, which was computed
   // from that slot's tag back at S_WB. If the slot's tag no longer matches
   // wb_laddr when the data is read, the write-back is publishing one line's bytes
   // under another line's address -- memory then holds bytes no store ever wrote,
   // which is the measured symptom. No shadow bookkeeping, so no false positives
   // from paths I failed to model.
   always @(posedge clk) if (!reset && (WRITABLE != 0) && (WRTHRU == 0))
      if ((st == S_WBW) && valm[flat(wb_way, wb_idx)]
          && (tagm[flat(wb_way, wb_idx)] != wb_laddr[PAW-OFFB-1 -: PTAGB]))
         $fatal(1, "[cache id=%0d] WRITE-BACK ADDRESS MISMATCH: reading way=%0d idx=%h (tag=%h) but publishing as laddr=%h (tag=%h)",
                PERF_ID, wb_way, wb_idx, tagm[flat(wb_way, wb_idx)],
                {wb_laddr, {OFFB{1'b0}}}, wb_laddr[PAW-OFFB-1 -: PTAGB]);

   // Duplicate-line tripwire: the same physical line valid in BOTH ways is a
   // structural fault -- hway=hit1 would silently shadow way 0's (possibly dirty,
   // newer) copy on every access. No install site cross-checks the other way (fills
   // assume miss-implies-absent), so enforce the invariant continuously: round-robin
   // one way-0 set per cycle, reconstruct its line, and check way 1's alias slot.
   // Full sweep every SETS cycles; a real duplicate persists far longer than that.
   reg [IDXB-1:0] dupscan; initial dupscan = 0;
   always @(posedge clk) if (!reset) begin
      dupscan <= dupscan + 1'b1;
      if (valm[flat(0, dupscan)]) begin : dupchk
         reg [PTAGB-1:0] t0; reg [IDXB-1:0] i1;
         t0 = tagm[flat(0, dupscan)];
         i1 = dupscan ^ t0[IDXB-1:0];
         if (valm[flat(1, i1)] && (tagm[flat(1, i1)] == t0))
            $fatal(1, "[cache id=%0d] DUPLICATE LINE: tag=%h way0 idx=%h / way1 idx=%h",
                   PERF_ID, t0, dupscan, i1);
      end
   end
`endif
`ifdef CACHEWATCH
   // Targeted line microscope: every write-port request/ack and FSM activity touching
   // the watched line (PA page in CACHEWATCH_PA<<12), with hit/way/WBB/MSHR context.
   integer cwc; initial cwc = 0;
   wire cw_wr = wr_req && (wr_addr[PAW-1:12] == `CACHEWATCH_PA);
   wire cw_rd = rd_req && (rd_addr[PAW-1:12] == `CACHEWATCH_PA);
`ifdef CACHEWATCH_IDX
   // Watch a SET rather than a page: a conflict-thrash bug cycles many different
   // lines through one index, so the set is the axis the corruption lives on.
   wire cw_set = (base_idx(cur_line) == `CACHEWATCH_IDX) || (eff_vi == `CACHEWATCH_IDX);
`else
   wire cw_set = 1'b0;
`endif
   always @(posedge clk) begin
      cwc <= cwc + 1;
      if (PERF_ID == 1 && cwc > `CACHEWATCH_T0 && cwc < `CACHEWATCH_T1) begin
         if (cw_wr | cw_rd)
            $display("[CW c=%0d %s a=%h d=%h m=%h st=%0d hit=%b ack=%b rdy=%b cbo=%b%b span=%b wbbv=%b wbbA=%h mshv=%b mshA=%h",
                     cwc, cw_wr ? "WR" : "RD", cw_wr ? wr_addr : rd_addr, wr_data, wr_mask,
                     st, hit, wr_ack, rd_rdy, cbo_req, cbo_zero, r_span,
                     wbb_val, {wbb_addr, {OFFB{1'b0}}}, msh_val, msh_addr);
         if (st == S_FIN && r_is_wr && (r_addr[PAW-1:12] == `CACHEWATCH_PA))
            $display("[CW c=%0d FIN a=%h hit=%b w0way=%0d nwin=%h]", cwc, r_addr, hit, w0_way, nwin);
         // FILL PATH: every state the FSM occupies while cur_line is in the watched page,
         // plus the selectors that decide WHERE the installed line's data comes from --
         // fresh L2 read, write-back-buffer bounce (wbb_match), MSHR merge (msh_ins) or
         // prefetch buffer (pf_hit). A line that ends up half-stale was assembled here.
         if ((cur_line[PAW-1:12] == `CACHEWATCH_PA) || cw_set)
            $display("[CWF c=%0d st=%0d line=%h pc=%0d wbbm=%b wbbv=%b wbbA=%h mshins=%b mshA=%h pfhit=%b vicd=%b vw=%b vi=%h lb0=%h lb7=%h]",
                     cwc, st, cur_line, pc, wbb_match, wbb_val, {wbb_addr, {OFFB{1'b0}}},
                     (HUM!=0) && msh_ins, msh_addr, pf_hit, vic_dirty, eff_vw, eff_vi,
                     linebuf[63:0], linebuf[511:448]);
         // data actually delivered to the client for a watched line
         if (rd_valid && ((rd_resp_addr[PAW-1:12] == `CACHEWATCH_PA) || cw_set))
            $display("[CWD c=%0d resp=%h data=%h]", cwc, rd_resp_addr, rd_data);
         // INSTALL journal: the cycle a line is committed into a slot, with the source
         // that supplied linebuf. This is what a half-stale line has to be traced to.
         if ((st == S_FILLI) && (pc == HALF-1) && ((cur_line[PAW-1:12] == `CACHEWATCH_PA) || cw_set))
            $display("[CWI c=%0d INSTALL way=%b idx=%h tag=%h src=%s cur=%h mshA=%h wbbA=%h lb0=%h lb7=%h]",
                     cwc, eff_vw, eff_vi, tagm[vflat],
                     ((HUM!=0) && msh_ins) ? "MSHR" : wbb_match ? "WBB " : pf_hit ? "PF  " : "L2  ",
                     cur_line, msh_addr, {wbb_addr, {OFFB{1'b0}}},
                     linebuf[63:0], linebuf[511:448]);
         if ((st == S_CHECK || st == S_FIN) && (cur_line[PAW-1:12] == `CACHEWATCH_PA))
            $display("[CW c=%0d CHK line=%h hit0=%b hit1=%b hway=%b v0=%b t0=%h v1=%b t1=%h ctag=%h]",
                     cwc, cur_line, hit0, hit1, hway,
                     valm[flat(0,ci0)], tagm[flat(0,ci0)], valm[flat(1,ci1)], tagm[flat(1,ci1)], ctag);
      end
   end
`endif
`ifdef CACHE_BLOCK_STATS
   // A read request can only be ACCEPTED at S_IDLE (see the S_IDLE arm below), so a pending
   // read waits out whatever the FSM is already doing. This splits that wait by what is
   // holding the FSM: a line fill (miss, anyone's -- incl. a PTW's PTE miss), a write-back,
   // or an ordinary lookup already in progress. Sizes the win from an FSM bypass for hits.
   integer cb_pend, cb_fill, cb_wb, cb_look, cb_other, cb_b2b;
   initial begin cb_pend=0; cb_fill=0; cb_wb=0; cb_look=0; cb_other=0; cb_b2b=0; end
   always @(posedge clk) if (!reset) begin
      if (pipe_take) cb_b2b = cb_b2b + 1;
      if (rd_req && st != S_IDLE) begin
         cb_pend = cb_pend + 1;
         if      (st==S_FILL || st==S_FILLW || st==S_FILLI || st==S_ZFILL || st==S_PFI
                             || st==S_WBBI || st==S_MSHI) cb_fill  = cb_fill  + 1;
         else if (st>=S_WB   && st<=S_WBA)             cb_wb    = cb_wb    + 1;
         else if (st==S_LOOK || st==S_CHECK || st==S_FIN) cb_look = cb_look + 1;
         else                                          cb_other = cb_other + 1;
      end
   end
   final if (cb_pend > 0)
      $display("[CACHE-BLK id=%0d] read pending while FSM busy=%0d  fill=%0d writeback=%0d lookup=%0d other=%0d b2b-accepts=%0d",
               PERF_ID, cb_pend, cb_fill, cb_wb, cb_look, cb_other, cb_b2b);
`endif

   integer b, bb, w2;
   reg [2*BANKW-1:0] nwin;
   reg [LZB:0]       pos;

   // ---- single-write-port staging for the status arrays (valm/dirm/vicm) ----
   // Scattered indexed NBA writes (plus a one-cycle full-clear loop) defeat RAM
   // inference: every status flop grew a ~15-LUT write decoder -- valm+dirm
   // synthesized to 63k LUTs, a third of the FPGA. Each FSM site now stages one
   // (we, addr, data) per array per cycle (blocking assigns inside the case);
   // the single write statement at the bottom of the FSM block applies it, so
   // the arrays infer as distributed RAM (~hundreds of LUTs). The full clear
   // (fence.i / inv on the I$) walks S_FLUSH like the write-back flush does.
   reg             v_we, d_we, k_we;
   reg [FW-1:0]    v_wa, d_wa;
   reg [IDXB-1:0]  k_wa;
   reg             v_wd, d_wd, k_wd;

   // live store-merge window: the same merge applied to the bank outputs as they are
   // being registered into wlo/whi. Lets a spanning store commit line0's chunk in the
   // cycle its lookup is live, instead of naming that slot again cycles later.
   integer fbb;
   reg [LZB:0] fpos;
   reg [2*BANKW-1:0] fnwin;
   always @* begin
      fnwin = {fwhi, fwlo};
      for (fbb=0; fbb<WRB; fbb=fbb+1) if (r_wmask[fbb]) begin
         fpos = {1'b0,bwc} + fbb[LZB:0];
         fnwin[fpos*8 +: 8] = r_wdata[fbb*8 +: 8];
      end
   end

   // store-merge window (combinational)
   always @* begin
      nwin = win;
      for (bb=0; bb<WRB; bb=bb+1) if (r_wmask[bb]) begin
         pos = {1'b0,bwc} + bb[LZB:0];
         nwin[pos*8 +: 8] = r_wdata[bb*8 +: 8];
      end
   end

   // live (accept-cycle) window geometry: same derivation as the registered
   // clo/pair_e/pair_o but from the request inputs, so the banks can be
   // addressed in the SAME cycle the request is accepted -- data then lands at
   // S_CHECK with S_LOOK skipped entirely (the 2-cycle hit path).
   wire [PAW-1:0]  a_live    = rd_req ? rd_addr : wr_addr;
   wire [CHB-1:0]  a_clo     = a_live[OFFB-1 -: CHB];
   wire [CHB-1:0]  a_chunk_e = a_clo[0] ? (a_clo + 1'b1) : a_clo;
   wire [CHB-1:0]  a_chunk_o = a_clo[0] ? a_clo : (a_clo + 1'b1);
   wire [PAIRB-1:0] a_pair_e = a_chunk_e[CHB-1:1];
   wire [PAIRB-1:0] a_pair_o = a_chunk_o[CHB-1:1];

   // ---- combinational bank port drive ----
   always @* begin
      for (b=0; b<2*WAYS; b=b+1) begin
         bk_rdaddr[b] = {BAW{1'b0}};
         bk_wren[b]   = 1'b0;
         bk_wraddr[b] = {BAW{1'b0}};
         bk_wrdata[b] = {BANKW{1'b0}};
      end

      // accept-cycle read: address the banks from the LIVE request (way_idx of the
      // full address == way_idx of its line: the index/tag bits exclude the offset).
      // No invalidate guard needed: if the FSM takes the inv arm instead, the read
      // data is simply never consumed.
      // Also driven at S_CHECK: the read ports are idle there, so a read accepted at
      // the fast-hit delivery edge (pipe_take) has its data ready one cycle later --
      // one hit per cycle. If the FSM doesn't accept, the data is never consumed.
      if ((st==S_IDLE && (rd_req || (wr_req && WRITABLE!=0)))
          || (st==S_CHECK && rd_req)) begin
         for (w2=0; w2<WAYS; w2=w2+1) begin
            bk_rdaddr[w2*2+0] = { way_idx(w2, a_live), a_pair_e };
            bk_rdaddr[w2*2+1] = { way_idx(w2, a_live), a_pair_o };
         end
      end
      // window read: present line's chunks so data is valid next cycle (both ways read)
      // (still used by the span phase-1 lookup and the post-fill re-lookup)
      if (st==S_LOOK) begin
         for (w2=0; w2<WAYS; w2=w2+1) begin
            if (!phase) begin
               bk_rdaddr[w2*2+0] = { way_idx(w2,cur_line), pair_e };  // even bank
               bk_rdaddr[w2*2+1] = { way_idx(w2,cur_line), pair_o };  // odd  bank
            end else
               bk_rdaddr[w2*2+0] = { way_idx(w2,cur_line), {PAIRB{1'b0}} };  // line1 chunk0
         end
      end
      // serialized full-line read (writeback / write-through / flush): pair pc, even+odd
      if (st==S_WBR || st==S_WTR || st==S_FLUSHR) begin
         bk_rdaddr[wb_way*2+0] = { wb_idx, pc[PAIRB-1:0] };
         bk_rdaddr[wb_way*2+1] = { wb_idx, pc[PAIRB-1:0] };
      end

      // serialized fill install: write pair pc of the victim from linebuf (even+odd)
      // (eff_*: an MSHR install targets the victim pinned at allocation)
      // spanning store: commit line0's chunk HERE, at line0's live lookup
      if ((WRITABLE!=0) && (st==S_CHECK) && !phase && hit && r_is_wr && r_span) begin
         bk_wren  [hway*2 + clo[0]] = 1'b1;
         bk_wraddr[hway*2 + clo[0]] = { cih, pair_lo };
         bk_wrdata[hway*2 + clo[0]] = fnwin[0 +: BANKW];
      end

      if (st==S_FILLI) begin
         bk_wren  [eff_vw*2+0] = 1'b1;
         bk_wraddr[eff_vw*2+0] = { eff_vi, pc[PAIRB-1:0] };
         bk_wrdata[eff_vw*2+0] = linebuf[(2*pc)  *BANKW +: BANKW];
         bk_wren  [eff_vw*2+1] = 1'b1;
         bk_wraddr[eff_vw*2+1] = { eff_vi, pc[PAIRB-1:0] };
         bk_wrdata[eff_vw*2+1] = linebuf[(2*pc+1)*BANKW +: BANKW];
      end
      // store merge: write the low chunk (and same-line high chunk if the store spilled).
      // Uses the captured LINE0 way/idx (w0_*) -- in a span, the live hway/cih are line1's.
      if (st==S_FIN && r_is_wr && hit) begin
         bk_wren  [w0_way*2 + clo[0]] = 1'b1;
         bk_wraddr[w0_way*2 + clo[0]] = { w0_idx, pair_lo };
         bk_wrdata[w0_way*2 + clo[0]] = nwin[0 +: BANKW];
         if (store_hi && !r_span) begin
            bk_wren  [w0_way*2 + (clo[0]^1'b1)] = 1'b1;
            bk_wraddr[w0_way*2 + (clo[0]^1'b1)] = { w0_idx, pair_hi };
            bk_wrdata[w0_way*2 + (clo[0]^1'b1)] = nwin[BANKW +: BANKW];
         end
      end
      // spanning store high half: write line1 chunk0 (even bank) of the line1 hit way
      if (st==S_SPANW && hit) begin
         bk_wren  [hway*2 + 0] = 1'b1;
         bk_wraddr[hway*2 + 0] = { cih, {PAIRB{1'b0}} };
         bk_wrdata[hway*2 + 0] = nwin[BANKW +: BANKW];
      end
   end

   // ---- FSM ----
   always @(posedge clk) begin
      // status-array write ports: default idle every cycle (blocking; sites override)
      v_we = 1'b0; d_we = 1'b0; k_we = 1'b0;
      v_wa = {FW{1'b0}}; d_wa = {FW{1'b0}}; k_wa = {IDXB{1'b0}};
      v_wd = 1'b0; d_wd = 1'b0; k_wd = 1'b0;
      if (reset) begin
         st <= S_IDLE; rd_valid <= 0; wr_ack <= 0; inv_busy <= 0;
         l2_req <= 0; l2_we <= 0; phase <= 0; fscan <= 0; inv_pend <= 0;
         pf_val <= 0; pf_want <= 0; pf_infl <= 0; pf_drop <= 0;
         wbb_val <= 0; wbb_infl <= 0;
         msh_val <= 0; msh_infl <= 0; msh_rdy <= 0; msh_ins <= 0; msh_ret <= 0;
      end else begin
         rd_valid <= 0; wr_ack <= 0; l2_req <= 0;
         // write-back buffer drain: borrow the L2 port (see wbb_do for the policy);
         // mutual exclusion with FSM issues: every FSM L2-issue site waits for
         // !wbb_infl, and wbb_do only fires in states that never touch the port.
         if (WBUF) begin
            if (wbb_do) begin
               l2_req <= 1; l2_we <= 1; l2_addr <= wbb_addr; l2_wdata <= wbb_data;
               wbb_infl <= 1;
            end
            if (wbb_infl && l2_ack) begin wbb_infl <= 0; wbb_val <= 0; end
         end
         // MSHR fill launcher + catcher: run the outstanding miss's L2 read while
         // the FSM serves hits. msh_launch excludes every FSM-issue state and the
         // drain; a store-miss's bytes are merged into the line at the catch.
         if (HUM) begin
            if (msh_launch) begin
               l2_req <= 1; l2_we <= 0; l2_addr <= msh_addr[PAW-1:OFFB];
               msh_infl <= 1;
            end
            if (msh_infl && l2_ack) begin
               msh_line <= msh_mrg; msh_infl <= 0; msh_rdy <= 1;
            end
         end
         // parallel prefetch engine: issue on the idle L2 port during hit-path
         // states (they never touch L2); mutual exclusion with demand fills is
         // by construction -- S_FILL stalls while pf_infl, l2_req is only ever
         // raised in states outside the issue set, and PF_EN excludes the
         // writeback/write-through/flush L2 states entirely.
         if (PF_EN) begin
            if (!pf_infl && pf_want && !l2_req
                && (st==S_IDLE || st==S_LOOK || st==S_CHECK || st==S_FIN)) begin
               l2_req <= 1; l2_we <= 0; l2_addr <= pf_next;
               pf_ia <= pf_next;
               pf_infl <= 1; pf_want <= 0;
            end
            if (pf_infl && l2_ack) begin
               // pf_drop: an invalidate ran while this read was in flight -- the
               // line predates the flush, so land it DEAD (else a stale line
               // survives fence.i through the buffer).
               pf_line <= l2_rdata; pf_addr <= pf_ia; pf_val <= ~pf_drop;
               pf_infl <= 0; pf_drop <= 0;
            end
         end
         // Latch + ACK an invalidate the cycle it is requested, even if the cache is mid-
         // operation (inv_req is only acted on at S_IDLE). Without this a 1-cycle inv_req
         // pulse arriving during a refill is silently dropped -> a stale line survives a
         // fence.i / sfence flush. Raising inv_busy now also keeps the requester waiting
         // until the invalidate actually runs (it polls !inv_busy).
         if (inv_req) begin inv_pend <= 1'b1; inv_busy <= 1'b1; end
         case (st)
           S_IDLE: begin
              phase <= 0;
              if (HUM && msh_rdy) begin
                 // returned fill: the install (oldest op) preempts at the idle
                 // boundary. S_WB captures the victim if dirty, then S_MSHI.
                 msh_ins <= 1; msh_ret <= 0; st <= S_WB;
              end else if (inv_req | inv_pend) begin
                 if (!(HUM && msh_val)) begin
                 inv_pend <= 1'b0;
                 pf_val <= 0; pf_want <= 0;      // prefetch buffer shares the cache's fate
                 if (pf_infl) pf_drop <= 1;      // in-flight line predates the flush: land it dead
                 // Every config walks S_FLUSH (one line/cycle behind inv_busy, which all
                 // requesters already poll). For WRITABLE==0/WRTHRU the dirty-writeback
                 // branch is compile-time dead, leaving a pure valid-clear scan -- the
                 // old one-cycle full clear was what broke the valm RAM inference.
                 inv_busy <= 1; fscan <= 0; flush_clean <= (WRITABLE!=0 && WRTHRU==0) & inv_clean;
                 st <= S_FLUSH;
                 end
                 // else: an outstanding miss must land + install first (inv_busy
                 // is already up, so the requester keeps waiting)
              end else if (rd_req || (wr_req && WRITABLE!=0
                                      && !(HUM && msh_val && msh_is_wr))) begin
                 r_is_wr  <= wr_req && !rd_req;
                 r_uncached <= rd_req ? rd_uncached : wr_uncached;   // Svpbmt
                 // CBO flags qualify a write-port maintenance op only. Reads win arbitration
                 // (rd_req priority), so a cbo.zero waiting to drain can coincide with a load;
                 // gating by (wr_req && !rd_req) stops cbo_zero latching onto that read and
                 // making its refill zero-fill the line instead of fetching it.
                 r_cbo    <= (wr_req && !rd_req) & cbo_req;
                 r_cbo_zero <= (wr_req && !rd_req) & cbo_zero;
                 r_cbo_keep <= (wr_req && !rd_req) & cbo_keep;   // Zicbom/Zicboz
                 r_addr   <= rd_req ? rd_addr : wr_addr;
                 r_wdata  <= wr_data; r_wmask <= wr_mask;
                 r_off    <= rd_req ? rd_addr[OFFB-1:0] : wr_addr[OFFB-1:0];
                 // a CBO is a single-line op (never spans)
                 r_span   <= ~cbo_req & (({1'b0,(rd_req ? rd_addr[OFFB-1:0] : wr_addr[OFFB-1:0])}
                               + (rd_req ? RDB : WRB)) > WORDB);
                 cur_line <= {(rd_req ? rd_addr[PAW-1:OFFB] : wr_addr[PAW-1:OFFB]), {OFFB{1'b0}}};
                 st <= S_CHECK;    // banks already addressed this cycle (live drive above)
              end
           end

           S_LOOK: st <= S_CHECK;

           S_CHECK: if (HUM && msh_rdy && !hit) begin
              // returned fill preempts a PARKED op (a miss that cannot proceed):
              // install it, then resume this op via S_LOOK (msh_ret) -- it
              // re-evaluates and often hits the just-installed line. cur_line
              // and r_* are untouched by the install, so nothing is lost.
              msh_ins <= 1; msh_ret <= 1; st <= S_WB;
           end else if (r_cbo) begin
              // Zicbom/Zicboz: single-line maintenance on the addressed line.
              if (hit) begin
                 w0_way <= hway; w0_idx <= cih;
                 if (r_cbo_zero) begin
                    // cbo.zero: overwrite the resident line with zeros (install loop), mark dirty
                    vw <= hway; vi <= cih; linebuf <= {LINEB{1'b0}}; pc <= 0; st <= S_FILLI;
                 end else if (WRTHRU==0 && dirm[flat(hway,cih)]) begin
                    // dirty -> write the line back to L2 (reuses the WT push path), then finalize
                    wb_way <= hway; wb_idx <= cih; pc <= 0; st <= S_WTR;
                 end else begin
                    // clean line: flush/inval just invalidates; clean keeps it
                    if (!r_cbo_keep) begin v_we=1; v_wa=flat(hway,cih); v_wd=1'b0; end
                    wr_ack <= 1; st <= S_IDLE;
                 end
              end else begin
                 if (r_cbo_zero) begin                 // miss: allocate a line, then zero-fill it
                    if (HUM && msh_val) begin
                       // park: this install could collide with the MSHR's pinned
                       // victim slot; the preempt above resolves the MSHR first
                    end else begin
                    vw <= vicm[base_idx(cur_line)];
                    vi <= way_idx(vicm[base_idx(cur_line)] ? 1 : 0, cur_line);
                    // cancel a buffered stale copy of this line: the zero line
                    // supersedes it (dirty in cache), and a LATER drain of the old
                    // data would clobber a newer cbo.flush/eviction write in L2
                    if (WBUF && wbb_match && !wbb_infl) wbb_val <= 0;
                    st <= S_WB;
                    end
                 end else if (WBUF && wbb_match) begin
                    // clean/flush/inval of the buffered line: hold until the drain
                    // lands (wbb_do has a term for this park) -- the no-op ack below
                    // promises L2 is current
                 end else begin wr_ack <= 1; st <= S_IDLE; end   // clean/flush/inval miss = no-op
              end
           end else begin
              if (hit) begin
                 if (!phase) begin
                    wlo <= clo[0] ? bk_rddata[hway*2+1] : bk_rddata[hway*2+0];
                    whi <= clo[0] ? bk_rddata[hway*2+0] : bk_rddata[hway*2+1];
                    w0_way <= hway; w0_idx <= cih;       // remember line0 (for span store)
                    if (r_span) begin
                       // line0's data write is driven combinationally this cycle; its
                       // status write and any NC/WT push slot are staged here too, so
                       // nothing about line0 outlives its own lookup.
                       if (r_is_wr) begin
                          if (WRTHRU==0 && !r_uncached) begin d_we=1; d_wa=flat(hway,cih); d_wd=1'b1; end
                          else begin wb_way <= hway; wb_idx <= cih; end
                       end else if (r_uncached) begin
                          v_we=1; v_wa=flat(hway,cih); v_wd=1'b0;   // NC load: drop line0
                       end
                       phase <= 1; cur_line <= line1; st <= S_LOOK;
                    end
                    else if (!r_is_wr && !r_uncached) begin
                       // fast read delivery: the window is live on the bank outputs
                       // (the same values registering into wlo/whi this edge) -- skip
                       // S_FIN. NC reads keep the slow path (S_FIN's flush-around).
                       rd_data  <= fast_sh[RDW-1:0];
                       rd_valid <= 1; rd_resp_addr <= r_addr;
                       if (pipe_take) begin
                          // back-to-back accept (rd_req&rd_rdy at the delivery edge):
                          // banks are already addressed from the live request; latch
                          // its identity and stay in S_CHECK. Write fields stay stale
                          // -- a read never reads them (nwin/store-merge are write-op-only).
                          r_is_wr    <= 1'b0;
                          r_uncached <= rd_uncached;
                          r_cbo <= 1'b0; r_cbo_zero <= 1'b0; r_cbo_keep <= 1'b0;
                          r_addr <= rd_addr; r_off <= rd_addr[OFFB-1:0];
                          r_span <= (({1'b0, rd_addr[OFFB-1:0]} + RDB) > WORDB);
                          cur_line <= {rd_addr[PAW-1:OFFB], {OFFB{1'b0}}};
                       end else
                          st <= S_IDLE;
                    end
                    else st <= S_FIN;
                 end else begin
                    whi <= bk_rddata[hway*2+0];           // line1 chunk0
                    st  <= S_FIN;
                 end
              end else begin
                 vw <= vicm[base_idx(cur_line)];
                 vi <= way_idx(vicm[base_idx(cur_line)] ? 1 : 0, cur_line);
                 if (msh_ok && !msh_val) begin
                    // divert the miss to the MSHR and FREE the FSM: hits and
                    // stores are served under it, the launcher/catcher run the
                    // fill, and the install preempts at the next boundary.
                    msh_val <= 1; msh_addr <= r_addr; msh_is_wr <= r_is_wr;
                    msh_wdata <= r_wdata; msh_wmask <= r_wmask;
                    msh_vw <= vicm[base_idx(cur_line)];
                    msh_vi <= way_idx(vicm[base_idx(cur_line)] ? 1 : 0, cur_line);
                    st <= S_IDLE;
                 end else if (HUM && msh_val) begin
                    // MSHR busy (second miss), or a serializing miss (NC/span/
                    // wbb-bounce) that must drain it first: park; the preempt at
                    // the top of S_CHECK runs the install, then this re-evaluates
                 end else begin
                 // stream-buffer hit: skip the L2 round trip. Capture the line AND
                 // consume the buffer AT THIS EDGE: a prefetch ack can land this very
                 // cycle and overwrite pf_line/pf_addr with a DIFFERENT line -- the
                 // nonblocking reads here take the pre-ack values pf_hit was computed
                 // on (a same-edge ack's fresh line is discarded: hint loss only).
                 if (pf_hit) begin linebuf <= pf_line; pf_val <= 0; end
                 st <= pf_hit ? S_PFI : S_WB;
                 end
              end
           end

           // ---- miss: evict victim (dirty -> CAPTURE into the write-back buffer,
           // the L2 write happens off the miss path) then fill ----
           S_WB: begin
              if (vic_dirty) begin
                 if (!wbb_val) begin          // buffer free: stream the victim into it
                    wb_way <= eff_vw; wb_idx <= eff_vi; pc <= 0;
                    wb_laddr <= {vtag, vbase};
                    st <= S_WBR;
                 end
                 // else parked: the drain engine (wbb_do) is pushing the previous line
              end else st <= (HUM && msh_ins) ? S_MSHI : (r_cbo_zero ? S_ZFILL : S_FILL);
           end
           S_WBR: st <= S_WBW;
           S_WBW: begin
              wbb_data[(2*pc)  *BANKW +: BANKW] <= bk_rddata[wb_way*2+0];
              wbb_data[(2*pc+1)*BANKW +: BANKW] <= bk_rddata[wb_way*2+1];
              if (pc == HALF-1) begin
                 pc <= 0; wbb_val <= 1'b1; wbb_addr <= wb_laddr;
                 st <= (HUM && msh_ins) ? S_MSHI : (r_cbo_zero ? S_ZFILL : S_FILL);
              end else begin pc <= pc + 1'b1; st <= S_WBR; end
           end
           // MSHR install: deliver straight from the caught line -- the window is
           // cut combinationally and a store's bytes were merged at the catch --
           // then stream it into the banks (S_FILLI). No re-lookup needed.
           S_MSHI: begin
              if (!msh_is_wr) begin
                 rd_data  <= msh_shift[RDW-1:0];
                 rd_valid <= 1; rd_resp_addr <= msh_addr;
              end else wr_ack <= 1;
              tagm[vflat] <= tag_of(msh_addr);
              v_we=1; v_wa=vflat; v_wd=1'b1;
              d_we=1; d_wa=vflat; d_wd=msh_is_wr;
              k_we=1; k_wa=base_idx(msh_addr); k_wd=~vicm[base_idx(msh_addr)];
              linebuf <= msh_line; pc <= 0;
              msh_val <= 0; msh_rdy <= 0;
              st <= S_FILLI;
           end

           // cbo.zero miss: victim evicted -> install a fresh zero line (no L2 read) + mark dirty
           S_ZFILL: begin
              tagm[vflat] <= tag_of(cur_line);
              v_we=1; v_wa=vflat; v_wd=1'b1;
              k_we=1; k_wa=base_idx(cur_line); k_wd=~vicm[base_idx(cur_line)];
              linebuf <= {LINEB{1'b0}}; pc <= 0;
              st <= S_FILLI;
           end

           S_FILL: if (WBUF && wbb_infl) begin
              // the L2 port is carrying the buffer drain: wait for its ack
           end else if (WBUF && wbb_match && !r_uncached) begin
              // the missing line IS the buffered victim: bounce it back in as a
              // DIRTY line (its data never reached L2) -- no L2 round trip at all
              linebuf <= wbb_data; wbb_val <= 0;
              st <= S_WBBI;
           end else if (WBUF && wbb_match && r_uncached) begin
              // NC fill of the buffered line: a bounce would mark it dirty and the
              // NC flush-around drop would then LOSE it. Park; wbb_do drains it to
              // L2, after which the normal fill below reads it back fresh.
           end else if (PF_EN && pf_hit) begin
              // the missing line is in (or just landed in) the buffer: install it.
              // Same decision-edge capture/consume as the S_CHECK shortcut.
              linebuf <= pf_line; pf_val <= 0;
              st <= S_PFI;
           end else if (PF_EN && pf_infl) begin
              // a prefetch is mid-flight on the L2 port: wait it out (it may be
              // exactly the missing line, caught by the branch above on landing).
           end else begin l2_req<=1; l2_we<=0; l2_addr<=cur_line[PAW-1:OFFB]; st<=S_FILLW; end
           S_FILLW: if (l2_ack) begin
              linebuf <= l2_rdata; pc <= 0;
              tagm[vflat] <= tag_of(cur_line);
              v_we=1; v_wa=vflat; v_wd=1'b1;
              d_we=1; d_wa=vflat; d_wd=1'b0;
              k_we=1; k_wa=base_idx(cur_line); k_wd=~vicm[base_idx(cur_line)];
              st <= S_FILLI;
              if (PF_EN && !r_uncached && !r_cbo_zero) begin
                 pf_want <= 1; pf_next <= cur_line[PAW-1:OFFB] + 1'b1;  // arm next-line
              end
           end

           // ---- prefetch: victim already chosen at S_CHECK; install the buffered line
           // (same bookkeeping as S_FILLW but sourced from pf_line, no L2), then re-arm
           // for the line after -- consumption-chained streaming.
           // prefetch install: linebuf was captured (and the buffer consumed) at the
           // decision edge in S_CHECK/S_FILL; here only the line bookkeeping + re-arm.
           S_PFI: begin
              pc <= 0;
              tagm[vflat] <= tag_of(cur_line);
              v_we=1; v_wa=vflat; v_wd=1'b1;
              d_we=1; d_wa=vflat; d_wd=1'b0;
              k_we=1; k_wa=base_idx(cur_line); k_wd=~vicm[base_idx(cur_line)];
              pf_want <= 1; pf_next <= cur_line[PAW-1:OFFB] + 1'b1;
              st <= S_FILLI;
           end
           // install the bounced write-back-buffer line: same bookkeeping as a fill
           // but marked DIRTY -- L2 never saw this data
           S_WBBI: begin
              pc <= 0;
              tagm[vflat] <= tag_of(cur_line);
              v_we=1; v_wa=vflat; v_wd=1'b1;
              d_we=1; d_wa=vflat; d_wd=1'b1;
              k_we=1; k_wa=base_idx(cur_line); k_wd=~vicm[base_idx(cur_line)];
              st <= S_FILLI;
           end
           S_FILLI: begin                  // install pair pc (bank writes combinational)
              if (pc == HALF-1) begin
                 pc <= 0;
                 // MSHR install done: already delivered at S_MSHI -- resume the
                 // parked op (S_LOOK re-addresses its banks) or go idle. The
                 // r_cbo_zero belongs to a possibly-PARKED op, so it must not
                 // be consulted while msh_ins.
                 if (HUM && msh_ins) begin
                    msh_ins <= 0;
                    st <= msh_ret ? S_LOOK : S_IDLE;
                 end
                 // cbo.zero: the line is now zero -> mark dirty and finish; else re-lookup the refill
                 else if (r_cbo_zero) begin d_we=1; d_wa=vflat; d_wd=1'b1; wr_ack <= 1; st <= S_IDLE; end
                 else st <= S_LOOK;
              end else pc <= pc + 1'b1;
           end

           // ---- hit terminal: deliver read / commit store ----
           S_FIN: begin
              if (!r_is_wr) begin
                 rd_data  <= win_sh[RDW-1:0];
                 rd_valid <= 1; rd_resp_addr <= r_addr;
                 // Svpbmt NC/IO load: return the (just-filled, current) word but don't keep the
                 // line, so a later DMA write isn't masked by a stale hit on the next NC load.
                 // A span clears line1 in S_NCI (one status write per cycle; data already out).
                 // (a span dropped line0 at its own lookup; S_NCI drops line1)
                 if (r_uncached && !r_span) begin v_we=1; v_wa=flat(w0_way,w0_idx); v_wd=1'b0; end
                 st <= (r_uncached && r_span) ? S_NCI : S_IDLE;
              end else begin
                 // low-chunk (and same-line high) write driven combinationally this cycle.
                 if (r_span && store_hi) begin
                    // line1 hit way/idx are live (phase1); write its chunk0 next cycle.
                    // line0 was written+dirtied at its own lookup (phase 0).
                    st <= S_SPANW;
                 end else if (r_span) begin
                    // line0-only span (the width test ignores the byte mask): done
                    wr_ack <= 1; st <= S_IDLE;
                 end else if (WRTHRU!=0 || r_uncached) begin
                    // write-through, OR a Svpbmt NC/IO store -> push to L2 (DMA sees it) and
                    // invalidate the line at S_WTA so nothing dirty/stale lingers (flush-around).
                    wb_way <= w0_way; wb_idx <= w0_idx; pc <= 0; st <= S_WTR;
                 end else begin
                    // (a line-crossing store always has store_hi -> the S_SPANW arm above,
                    // so no second dirty write can be needed here)
                    d_we=1; d_wa=flat(w0_way,w0_idx); d_wd=1'b1;
                    wr_ack <= 1; st <= S_IDLE;
                 end
              end
           end
           S_NCI: begin                     // NC span load: drop line1 too (flush-around)
              v_we=1; v_wa=flat(hway,cih); v_wd=1'b0;
              st <= S_IDLE;
           end
           S_SPANW: begin                   // spanning store high half written combinationally
              if (WRTHRU!=0 || r_uncached) begin pc <= 0; st <= S_WTR; end   // wb_* captured at phase 0
              else begin
                 d_we=1; d_wa=flat(hway,cih); d_wd=1'b1;   // line0's dirty was staged at S_FIN
                 wr_ack <= 1; st <= S_IDLE;
              end
           end

           // ---- write-through: read the updated full line0, push to L2 ----
           // (span: only line0 pushed here; line1's WT push is omitted for now -- spanning
           //  stores in write-through configs are not exercised by the probe D$ tests.)
           S_WTR: st <= S_WTW;
           S_WTW: begin
              linebuf[(2*pc)  *BANKW +: BANKW] <= bk_rddata[wb_way*2+0];
              linebuf[(2*pc+1)*BANKW +: BANKW] <= bk_rddata[wb_way*2+1];
              if (pc == HALF-1) begin pc <= 0; st <= S_WTI; end
              else begin pc <= pc + 1'b1; st <= S_WTR; end
           end
           S_WTI: if (!(WBUF && wbb_infl) && !(HUM && msh_infl)) begin l2_req<=1; l2_we<=1; l2_addr<=line0[PAW-1:OFFB]; l2_wdata<=linebuf; st<=S_WTA; end
           S_WTA: if (l2_ack) begin
              if (r_cbo) begin                                     // Zicbom writeback complete
                 d_we=1; d_wa=flat(wb_way,wb_idx); d_wd=1'b0;      // it is now clean in L2
                 if (!r_cbo_keep) begin v_we=1; v_wa=flat(wb_way,wb_idx); v_wd=1'b0; end  // flush/inval drop
              end else if (r_uncached) begin v_we=1; v_wa=flat(wb_way,wb_idx); v_wd=1'b0; end  // NC store: flush-around
              wr_ack <= 1; st <= S_IDLE;
           end

           // ---- flush (write-back configs) ----
           S_FLUSH: begin
              if (fscan == NW) begin
                 // hold inv_busy until the write-back buffer drains: flush/fence
                 // semantics promise L2 is current when inv_busy drops
                 if (!(WBUF && (wbb_val || wbb_infl))) begin inv_busy <= 0; st <= S_IDLE; end
              end
              else if (WRITABLE!=0 && WRTHRU==0 && valm[fscan[FW-1:0]] && dirm[fscan[FW-1:0]]) begin
                 wb_way <= fscan[FW-1]; wb_idx <= fidx; pc <= 0; wb_laddr <= {ftag, fbase};
                 st <= S_FLUSHR;
              end else begin
                 if (!flush_clean) begin v_we=1; v_wa=fscan[FW-1:0]; v_wd=1'b0; end  // clean flush keeps lines valid
                 d_we=1; d_wa=fscan[FW-1:0]; d_wd=1'b0;
                 fscan <= fscan + 1'b1;
              end
           end
           S_FLUSHR: st <= S_FLUSHW;
           S_FLUSHW: begin
              linebuf[(2*pc)  *BANKW +: BANKW] <= bk_rddata[wb_way*2+0];
              linebuf[(2*pc+1)*BANKW +: BANKW] <= bk_rddata[wb_way*2+1];
              if (pc == HALF-1) begin pc <= 0; st <= S_FLUSHI; end
              else begin pc <= pc + 1'b1; st <= S_FLUSHR; end
           end
           S_FLUSHI: if (!(WBUF && wbb_infl)) begin l2_req<=1; l2_we<=1; l2_addr<=wb_laddr; l2_wdata<=linebuf; st<=S_FLUSHA; end
           S_FLUSHA: if (l2_ack) begin
              if (!flush_clean) begin v_we=1; v_wa=fscan[FW-1:0]; v_wd=1'b0; end  // clean flush: written back, stays valid+clean
              d_we=1; d_wa=fscan[FW-1:0]; d_wd=1'b0;
              fscan <= fscan + 1'b1; st <= S_FLUSH;
           end
         endcase
      end
      // the single write port of each status array (see staging decl above)
      if (v_we) valm[v_wa] <= v_wd;
      if (d_we) dirm[d_wa] <= d_wd;
      if (k_we) vicm[k_wa] <= k_wd;
`ifdef CDBG
      if (st==S_FLUSH && (fscan[3:0]==0 || fscan==NW))
         $display("[CDBG %m] t=%0t S_FLUSH fscan=%0d/%0d invb=%b", $time, fscan, NW, inv_busy);
      if (st==S_IDLE && (inv_req|inv_pend))
         $display("[CDBG %m] t=%0t IDLE->inv (pend=%b)", $time, inv_pend);
`endif
   end

`ifdef PERF_TRACE
   // Zihpm hardware cache events (always-on, unlike the PERF_TRACE DPI trace below): one pulse
   // per resolved line lookup (S_CHECK), and per miss. soc_top taps these -> backend_top hpm_ev.
   // A PARKED S_CHECK (miss waiting on the MSHR / a CBO waiting on the wb-buffer)
   // loops in S_CHECK without resolving anything -- mask it or misses overcount.
   wire chk_park = (st == S_CHECK) & ~hit
                 & ((((HUM!=0) ? msh_val : 1'b0)) | (WBUF && r_cbo && wbb_match));
   assign perf_access = (st == S_CHECK) & ~chk_park;
   assign perf_miss   = (st == S_CHECK) & ~hit & ~chk_park;

   // Cache memory-system events (docs/perf-observability-plan.md, step 2). Self-contained
   // (own perf_cyc, in lockstep with backend_top's since same clk/reset) into the shared
   // perf_ev sink. KIND=8 CACHE; the `ckp` field = PERF_ID (0=I$, 1=D$), `rdv` = is_write,
   // `data` = physical address, and `seq` = subtype:
   //   0 HIT   a line lookup hit            (rate: hits vs misses; addr -> set index)
   //   1 MISS  a line lookup missed -> fill (conflict-vs-capacity via set-index histogram)
   //   2 FILL  refill complete; `insn` = miss penalty (miss-detect -> re-lookup, in cycles)
   //   3 STORE store committed; `insn` = store latency (accept -> wr_ack). For the
   //           write-through D$ this is the full L2 round-trip = the store-buffer drain rate.
   import "DPI-C" function void perf_ev(input longint cyc, input int kind, input int seq,
                                        input int ckp, input int rdv, input int pdst,
                                        input int ps1, input int ps2, input longint data,
                                        input int insn);
   reg [63:0] perf_cyc, perf_miss_cyc, perf_st_cyc;
   initial perf_cyc = 64'd0;
   always @(posedge clk) if (!reset) begin
      perf_cyc <= perf_cyc + 64'd1;
      // lookup resolved this cycle (one event per line lookup; a span resolves twice;
      // parked S_CHECK cycles are masked -- they resolve nothing)
      if (st == S_CHECK && !chk_park) begin
         perf_ev(perf_cyc, 8, hit ? 0 : 1, PERF_ID, {31'd0, r_is_wr},
                 0, 0, 0, {{(64-PAW){1'b0}}, cur_line}, 0);
         if (!hit) perf_miss_cyc <= perf_cyc;
      end
      // capture store accept (for the accept->ack latency)
      if (st == S_IDLE && wr_req && !rd_req && WRITABLE != 0) perf_st_cyc <= perf_cyc;
      // refill complete -> re-lookup: emit the miss penalty
      if (st == S_FILLI && pc == HALF-1)
         perf_ev(perf_cyc, 8, 2, PERF_ID, 0, 0, 0, 0, {{(64-PAW){1'b0}}, cur_line},
                 (perf_cyc - perf_miss_cyc));
      // store committed (wr_ack asserted last cycle; +1 constant offset, fine for a hist)
      if (wr_ack)
         perf_ev(perf_cyc, 8, 3, PERF_ID, 1, 0, 0, 0, {{(64-PAW){1'b0}}, r_addr},
                 (perf_cyc - perf_st_cyc));
   end
`endif
endmodule

`default_nettype wire
