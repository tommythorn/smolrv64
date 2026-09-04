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
module rv_cache #(
   parameter RTW      = 4,      // opaque request-tag width (see rd_tag). 4 leaves room for
                               // a load-queue index when multiple outstanding loads land.
   parameter PAW      = 34,
   // THE TAG COVERS THE SIGNIFICANT PHYSICAL ADDRESS BITS, NOT THE PORT WIDTH. The SoC's
   // ports are 64 wide (a PA rides in a 64-bit bus), but the platform decodes 34 bits: 2 GiB
   // of DDR at 0x8000_0000 and every device below it. Tagging all 64 made the tag 49 bits
   // wide -- 30 of them structurally zero -- so the compare was 49 bits, the tag array
   // 1K x 49 (RAM64M8 x 224 per read port) and its index fanned out to every one of them.
   // A PA at or above 2^PAW_SIG cannot be tagged and is asserted never to arrive; the
   // writeback address is rebuilt from the tag and zero-extended, which is exact under that
   // assertion. Spec section 9.1 / 10.3.
   parameter PAW_SIG  = 34,
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
   input  wire [PAW-1:0]   rd_addr,
   output reg  [RDW-1:0]   rd_data,
   output reg              rd_valid,
   output reg  [PAW-1:0]   rd_resp_addr,
   // OPAQUE request tag. The cache never interprets it -- it captures whatever the
   // requester supplied and echoes it with the response, so the requester can match a
   // response to the request it allocated instead of comparing addresses
   // (docs/rtl-rules.md: "matched by a tag the requester allocated, not by address").
   // Opaque on purpose: when the LSU gains multiple outstanding loads, only its tag
   // ALLOCATION changes and nothing in here moves.
   input  wire [RTW-1:0]   rd_tag,
   output reg  [RTW-1:0]   rd_resp_tag,
   // ACCEPTED THIS CYCLE. A pipelined port cannot use "hold the request until the response
   // matches" -- rd_valid is registered, so during the cycle a response is being produced the
   // old request is still asserted and would be taken twice. The requester drops (or advances)
   // on this instead: a request is accepted or it is not (docs/rtl-rules.md D5).
   output wire             rd_ack,
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
   localparam PTAGB = PAW_SIG - IDXB - OFFB;
   initial if (PAW < PAW_SIG) $fatal(1, "rv_cache: PAW_SIG=%0d exceeds the port width PAW=%0d", PAW_SIG, PAW);
   localparam RDB   = RDW/8;
   localparam WRB   = WDW/8;
   localparam NW    = WAYS*SETS;
   localparam FW    = $clog2(NW);
   // flat() concatenates {way, index}, which is only the flat index when WAYS is 2.
   initial if (WAYS != 2) $fatal(1, "rv_cache: flat() assumes WAYS==2, got %0d", WAYS);

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
`ifdef OOO2_UNSKEWED
      // Unskewed: slot identity is tag-independent, so an index stays valid across a
      // line-crossing access even as cur_line advances from line0 to line1.
      begin t = tag_of(a); way_idx = base_idx(a); end
`else
      begin t = tag_of(a); way_idx = (w==0) ? base_idx(a) : (base_idx(a) ^ t[IDXB-1:0]); end
`endif
   endfunction
   // {way, index}, not (w!=0)*SETS+ix: the multiply-add is a 32-bit expression that only
   // happened to fit at SIZE_KB=128 and truncates at 64.  FW is clog2(WAYS*SETS) =
   // 1+IDXB for WAYS=2, so the concatenation IS the flat index, exactly FW bits.
   function [FW-1:0] flat; input integer w; input [IDXB-1:0] ix; flat = {(w!=0), ix}; endfunction

   // ---- data banks: index b = way*2 + parity ----
   reg  [BAW-1:0]   bk_rdaddr [0:2*WAYS-1];
   wire [BANKW-1:0] bk_rddata [0:2*WAYS-1];
   reg              bk_wren   [0:2*WAYS-1];
   reg  [BAW-1:0]   bk_wraddr [0:2*WAYS-1];
   reg  [BANKW-1:0] bk_wrdata [0:2*WAYS-1];
   genvar gb;
   generate for (gb=0; gb<2*WAYS; gb=gb+1) begin : banks
      smolrv64_sdpram #(.ADDR_WIDTH(BAW), .DATA_WIDTH(BANKW), .READ_LATENCY(1)) u_bank
        (.clock(clk), .rd_addr(bk_rdaddr[gb]), .rd_data(bk_rddata[gb]),
         .wr_en(bk_wren[gb]), .wr_addr(bk_wraddr[gb]), .wr_data(bk_wrdata[gb]));
   end endgenerate

   // ---- request regs ----
   reg            r_is_wr;
   reg            r_uncached;          // Svpbmt: current access is NC/IO (flush-around)
   reg            r_cbo, r_cbo_zero, r_cbo_keep;   // Zicbom/Zicboz maintenance op latched at accept
   reg [PAW-1:0]  r_addr;
   reg [RTW-1:0]  r_tag;
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
   wire [FW-1:0]    vflat = flat(vw?1:0, vi);
   wire [PTAGB-1:0] vtag  = tagm[vflat];
`ifdef OOO2_UNSKEWED
   wire [IDXB-1:0]  vbase = vi;                       // unskewed: base == index
`else
   wire [IDXB-1:0]  vbase = vw ? (vi ^ vtag[IDXB-1:0]) : vi;
`endif

   reg            flush_clean;        // current flush is clean-only (keep lines valid)
   reg [FW:0]     fscan;
   wire           fway  = fscan[IDXB];
   wire [IDXB-1:0] fidx  = fscan[IDXB-1:0];
   wire [PTAGB-1:0] ftag  = tagm[fscan[FW-1:0]];
`ifdef OOO2_UNSKEWED
   wire [IDXB-1:0]  fbase = fidx;                     // unskewed: base == index
`else
   wire [IDXB-1:0]  fbase = fway ? (fidx ^ ftag[IDXB-1:0]) : fidx;
`endif

   // ---- window + line buffers ----
   reg [BANKW-1:0] wlo, whi;
   wire [2*BANKW-1:0] win    = {whi, wlo};
   wire [2*BANKW-1:0] win_sh = win >> (bwc*8);
   // live-window variant for the S_CHECK fast read delivery (same bytes that are
   // being registered into wlo/whi this edge).
   // SHIFT PER WAY, THEN SELECT THE WAY. The way is the LAST thing this cycle learns --
   // it is the tag compare -- so it must be the last mux on the data, not the first. The
   // old form selected the way's two chunks first (128 mux selects on `hway`) and shifted
   // the result; this shifts both ways' windows from registers and bank outputs alone and
   // lets `hway` pick one 64-bit result. Same bytes, one 2:1 mux after the compare.
   wire [2*BANKW-1:0] fast0 = (clo[0] ? {bk_rddata[0], bk_rddata[1]} : {bk_rddata[1], bk_rddata[0]}) >> (bwc*8);
   wire [2*BANKW-1:0] fast1 = (clo[0] ? {bk_rddata[2], bk_rddata[3]} : {bk_rddata[3], bk_rddata[2]}) >> (bwc*8);
   reg [LINEB-1:0] linebuf;
   reg [PAIRB:0]   pc;
   reg [IDXB-1:0]  wb_idx;  reg wb_way;            // line currently streamed for WB/WT/flush
   reg             w0_way;  reg [IDXB-1:0] w0_idx; // line0 hit way/idx (for the span store)
   reg             w1_way;  reg [IDXB-1:0] w1_idx; // line1 hit way/idx (phase 1), see S_CHECK
   reg [PAW-OFFB-1:0] wb_laddr;                    // L2 line address for the streamed writeback

   // TWO machines now, not one. The LOOKUP pipeline resolves a request against the arrays;
   // the FILL machine owns everything that takes an L2 round trip. They were one `case`, and
   // that -- not the FSM's shape -- is why the cache served one request at a time: a miss put
   // the machine in states from which the accept state was simply unreachable.
   localparam [3:0]
      S_IDLE=0, S_LOOK=1, S_CHECK=2, S_FIN=3, S_SPANW=4, S_NCI=5,
      S_WTR=6, S_WTW=7, S_WTI=8, S_WTA=9;
   localparam [4:0]
      F_IDLE=0, F_WB=1, F_WBR=2, F_WBW=3, F_WBI=4, F_WBA=5,
      F_FILL=6, F_FILLW=7, F_FILLI=8, F_ZFILL=9, F_PFI=10,
      F_FLUSH=11, F_FLUSHR=12, F_FLUSHW=13, F_FLUSHI=14, F_FLUSHA=15,
      F_ANS=16;

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
   reg [3:0] st;         // lookup pipeline
   reg [4:0] fst;        // fill machine

   // THE STAGE-B WINDOW IS LIVE. The data banks are synchronous with NO read enable: what
   // S_CHECK compares and shifts is whatever row was addressed at the PREVIOUS edge, and
   // bk_rdaddr falls back to 0 in every cycle no state drives it. So a stage that stalls
   // does not keep its window -- it gets row 0, or the victim chunk the writeback is
   // streaming. This bit says the banks were addressed for the request now in S_CHECK;
   // when it is low the stage re-reads through S_LOOK instead of delivering another line's
   // bytes. fill_banks is in it because the fill machine's victim stream OWNS the read
   // port for those cycles (that is why acc_slot excludes them) and wins the drive below.
   reg b_live;

   // ---- MSHR: the one request the fill machine owns -------------------------------------
   // A miss hands its request over HERE and leaves the pipeline, which is the entire point of
   // the split -- the pipeline is then free to resolve other requests while the line is
   // fetched. The copy is what makes that safe: r_*/cur_line belong to whatever the pipeline
   // is looking at now, and a fill outlives several of those.
   //
   // When the line lands the request is RE-ISSUED into the accept stage rather than answered
   // from linebuf. Answering from linebuf would need a 512->64 variable byte shift, and this
   // cache's whole datapath is built to avoid exactly that (see the header: "a minimal mux,
   // no full-line read"). A replay costs one pipeline pass and no mux at all.
   reg            f_v;                        // a fill is in progress
   reg [PAW-1:0]  f_line;                     // the line being fetched (line1, for a span)
   reg [PAW-1:0]  f_addr;                     // ...and the request to re-issue when it lands
   reg [RTW-1:0]  f_tag;
   reg            f_is_wr, f_uncached, f_cbo, f_cbo_zero, f_cbo_keep, f_span;
   reg [WDW-1:0]  f_wdata;
   reg [WRB-1:0]  f_wmask;
   reg            f_replay;                   // re-issue f_* into the accept stage now
   initial begin f_v = 1'b0; f_replay = 1'b0; end

   // The fill machine's own view of the stream buffer. pf_hit below is the PIPELINE's
   // question, asked about cur_line at the miss; this is the same question asked about the
   // line the fill machine is actually fetching, and they are different lines.
   wire pf_hit_f = PF_EN & pf_val & (pf_addr == f_line[PAW-1:OFFB]) & ~f_uncached & ~f_cbo;

   // Answering a missing READ out of the landed line. The two chunks it needs are selected
   // from linebuf into wlo/whi and then run through win_sh, the SAME network a hit uses --
   // so this is one 8:1 chunk mux, not the 512->64 byte mux that answering from a line
   // buffer normally costs. f_cnx wraps within the line: if the read sits in the last chunk
   // it cannot span (a spanning request is never answered this way), so whi is unused.
   wire [CHB-1:0]     f_clo    = f_addr[OFFB-1 -: CHB];
   wire [CHB-1:0]     f_cnx    = f_clo + 1'b1;
   wire [LZB-1:0]     f_bwc    = f_addr[LZB-1:0];
   wire [2*BANKW-1:0] f_win_sh = {whi, wlo} >> (f_bwc*8);

   // linebuf/pc/wb_* are ONE set of resources. The fill machine's writeback and the
   // pipeline's write-through push both stream a line through them, so the two are mutually
   // exclusive -- enforced by refusing to accept a write-through-shaped request while a fill
   // is live, and asserted rather than trusted at the bottom of this file.
   wire pipe_uses_lb = (st==S_WTR) | (st==S_WTW) | (st==S_WTI) | (st==S_WTA);

   // THE ONE REASON STAGE B HOLDS, computed once and applied at both sites that need it --
   // the S_CHECK arm that holds, and the perf event that must not count a cycle in which
   // nothing resolved. A CBO reads only the status arrays, so it never holds for a window.
   wire pipe_hold = (st == S_CHECK) & ~r_cbo
                  & ( ~b_live
                    | (hit & ~phase & ~r_span & ~r_is_wr & ~r_uncached & (fst == F_ANS))
                    | (~hit & f_v) );

   // THE FILL MACHINE'S WINDOW ON THE BANKS -- both directions. It takes the READ port to
   // stream a victim out, and F_FILLI takes the WRITE port to install. The install used to be
   // outside this: "an install is a bank WRITE and a lookup is a bank READ, on 1R1W banks, so
   // they do not contend at all". That is true of the behavioural model and NOT of the BRAM it
   // becomes -- a simple-dual-port block RAM reading the address port A is writing in the same
   // cycle returns INVALID data, and this file already carries one scar from trusting a sim
   // model of these banks over the hardware (smolrv64_sdpram's width guard). The cache's own
   // invariant already said so and fires on a directed hit-under-miss at the last install
   // cycle. So the pipeline sits out the install too: a few cycles inside a fill tens long.
   wire fill_banks  = (fst==F_WBR) | (fst==F_WBW) | (fst==F_FLUSHR) | (fst==F_FLUSHW)
                    | (fst==F_FILLI);

   // THE FILL MACHINE HAS AN L2 REQUEST ISSUED OR OUTSTANDING. Not "is issuing this
   // cycle": the hazard is the whole ROUND TRIP. l2_req is a one-cycle pulse, so the cycle
   // after the fill machine issues, l2_req is low again and fst is F_FILLW -- both of the
   // old guard's tests pass and the prefetch issues into a port that is still busy. Two
   // requests outstanding, one response, and both consumers latch it.
   wire fill_l2_busy = (fst==F_FILL) | (fst==F_FILLW)
                     | (fst==F_WBI)  | (fst==F_WBA)
                     | (fst==F_FLUSHI) | (fst==F_FLUSHA) | (st==S_WTI) | (st==S_WTA);

   integer b, bb, w2;
   reg [2*BANKW-1:0] nwin;
   reg [LZB:0]       pos;
   reg               bk_rd_drv;      // a bank read address is being presented this cycle

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

   // store-merge window (combinational)
   always @* begin
      nwin = win;
      pos  = {(LZB+1){1'b0}};   // loop temp: assigned only under r_wmask[bb] below, so without
                                // a default it infers a latch holding the previous byte's index
      for (bb=0; bb<WRB; bb=bb+1) if (r_wmask[bb]) begin
         pos = {1'b0,bwc} + bb[LZB:0];
         nwin[pos*8 +: 8] = r_wdata[bb*8 +: 8];
      end
   end

   // live (accept-cycle) window geometry: same derivation as the registered
   // clo/pair_e/pair_o but from the request inputs, so the banks can be
   // addressed in the SAME cycle the request is accepted -- data then lands at
   // S_CHECK with S_LOOK skipped entirely (the 2-cycle hit path).
   // SELECTED BY f_replay, A REGISTER -- not by do_replay, which is
   // acc_slot(st, inv_go, inv_busy, fill_banks) & f_replay and puts that whole decode on
   // the select of a mux that feeds a BRAM ADDRESS pin (docs/rtl-rules.md I6). The two
   // differ only when a replay is owed and the slot is busy, and in those cycles nothing
   // consumes this address: the drive below is overridden by the states that need it, and
   // the FSM reads bk_rddata only in the cycle after it accepted.
   wire [PAW-1:0]  a_live    = f_replay ? f_addr : (rd_req ? rd_addr : wr_addr);
   wire [CHB-1:0]  a_clo     = a_live[OFFB-1 -: CHB];
   wire [CHB-1:0]  a_chunk_e = a_clo[0] ? (a_clo + 1'b1) : a_clo;
   wire [CHB-1:0]  a_chunk_o = a_clo[0] ? a_clo : (a_clo + 1'b1);
   wire [PAIRB-1:0] a_pair_e = a_chunk_e[CHB-1:1];
   wire [PAIRB-1:0] a_pair_o = a_chunk_o[CHB-1:1];

   // ---- acceptance ------------------------------------------------------------------
   // The pipeline takes a request when it is empty and nothing older owns the slot. A
   // completed fill's REPLAY wins it outright: that request is older than anything at the
   // door, and the pipeline was deliberately held empty for it.
   //
   // SOLO requests. A store, an NC access, a CBO and a line-spanning access all share state
   // with the fill machine -- linebuf/pc/wb_* for the write-through push, and the pipeline
   // itself for a replay if they miss -- so they are taken only when the fill machine is
   // idle, and while the MSHR holds one, nothing at all is taken. A plain cached read shares
   // none of it, which is exactly why it is the one that gets to overlap a fill.
   // AN INVALIDATE STOPS THE DOOR, NOT THE REPLAY, and those are two different gates.
   //
   // The replay is a request that ALREADY MISSED and whose line is ALREADY FETCHED; f_v is
   // cleared by nothing else. Blocking it behind an invalidate closes a cycle: inv_pend
   // holds off the replay, f_v holds off the scan (F_IDLE waits for !f_v), and inv_pend is
   // cleared only by the scan starting. Every solo miss -- a store, an NC access, a CBO, and
   // the line-SPANNING fetch the I$ does all day -- opens that window for the length of a
   // fill, and a fence.i landing in it wedges the cache for good: inv_busy stuck high, the
   // fence.i FSM waiting on it forever, imem_avail nailed to 0, the core stopped. It needs
   // no MMU and no second client, which is why it kills the board before Linux is entered.
   // So `do_replay` asks only whether stage A is free.
   //
   // ~inv_busy on the DOOR, and not just ~inv_go. THE FLUSH IS A FILL-MACHINE JOB NOW: `st`
   // used to sit in the flush states, so "st == S_IDLE" kept requests out for the whole scan
   // by itself. Now the pipeline is idle throughout it, inv_pend clears the cycle the scan
   // starts, and a request taken mid-scan that MISSES writes `fst <= F_WB` over it --
   // F_FLUSH's clean arm assigns no fst, so the pipeline's write is the one that lands. The
   // scan is abandoned with inv_pend already clear, so it never resumes: the same dead
   // board by a different road. inv_busy spans request-to-completion, which is the window.
   wire inv_go   = inv_req | inv_pend;
   wire acc_slot = (st == S_IDLE) & ~fill_banks;
   wire req_wr   = wr_req & ~rd_req & (WRITABLE != 0);
   wire req_span = ~cbo_req & (({1'b0,(rd_req ? rd_addr[OFFB-1:0] : wr_addr[OFFB-1:0])}
                     + (rd_req ? RDB : WRB)) > WORDB);
   wire req_solo = req_wr | (rd_req & rd_uncached) | req_span;
   wire f_solo   = f_v & (f_is_wr | f_cbo | f_uncached | f_span);
   wire do_replay = acc_slot & f_replay;
   wire accept    = acc_slot & ~inv_go & ~inv_busy
                  & ~f_replay & ~f_solo & (rd_req | req_wr) & ~(req_solo & f_v);
   assign rd_ack  = accept & rd_req;

   // ---- combinational bank port drive ----
   always @* begin
      bk_rd_drv = 1'b0;
      for (b=0; b<2*WAYS; b=b+1) begin
         bk_rdaddr[b] = {BAW{1'b0}};
         bk_wren[b]   = 1'b0;
         bk_wraddr[b] = {BAW{1'b0}};
         bk_wrdata[b] = {BANKW{1'b0}};
      end

      // THE LOOKUP'S READ ADDRESS IS PRESENTED UNCONDITIONALLY, from the live request.
      // It reaches a BRAM ADDRESS pin, so nothing that decodes FSM state or arbitration
      // belongs on it (docs/rtl-rules.md I6, the rule 0b53c4b paid for) -- and `accept`
      // carries acc_slot, which is st, inv_go, inv_busy and a five-way fst decode. An
      // address the FSM does not consume is free: bk_rddata is read only in the cycle
      // after an accept, and every state that needs a DIFFERENT address drives it below,
      // later in this block, where it wins. That is the same argument the invalidate arm
      // already relied on, applied to the whole condition rather than half of it.
      // ...but the COLLISION CHECK below still means "a read that will be consumed", because
      // a BRAM collision corrupts the read, never the write: an address presented and
      // discarded cannot hurt anything, and flagging it would be a false alarm.
      bk_rd_drv = accept | do_replay;
      for (w2=0; w2<WAYS; w2=w2+1) begin
         bk_rdaddr[w2*2+0] = { way_idx(w2, a_live), a_pair_e };
         bk_rdaddr[w2*2+1] = { way_idx(w2, a_live), a_pair_o };
      end
      // window read: present line's chunks so data is valid next cycle (both ways read)
      // (the span phase-1 lookup, the post-fill re-lookup, and a held stage B re-reading its
      // own row). ~fill_banks for the same reason acc_slot has it: those cycles belong to the
      // fill machine, and b_live tells stage B its window did not happen.
      if (st==S_LOOK && !fill_banks) begin
         bk_rd_drv = 1'b1;
         for (w2=0; w2<WAYS; w2=w2+1) begin
            if (!phase) begin
               bk_rdaddr[w2*2+0] = { way_idx(w2,cur_line), pair_e };  // even bank
               bk_rdaddr[w2*2+1] = { way_idx(w2,cur_line), pair_o };  // odd  bank
            end else
               bk_rdaddr[w2*2+0] = { way_idx(w2,cur_line), {PAIRB{1'b0}} };  // line1 chunk0
         end
      end
      // serialized full-line read (writeback / write-through / flush): pair pc, even+odd
      if (fst==F_WBR || fst==F_FLUSHR || st==S_WTR) begin
         bk_rd_drv = 1'b1;
         bk_rdaddr[wb_way*2+0] = { wb_idx, pc[PAIRB-1:0] };
         bk_rdaddr[wb_way*2+1] = { wb_idx, pc[PAIRB-1:0] };
      end

      // serialized fill install: write pair pc of the victim from linebuf (even+odd).
      // A cbo.zero installs ZEROS BY MASKING HERE, not by zeroing linebuf at the lookup.
      // `linebuf <= 0` on the cbo.zero HIT sat under `hit` in S_CHECK, which put the tag
      // compare on the clock-enable of 512 flops: the D$'s worst family in the 2026-09-03
      // build (cur_line -> tagm -> compare -> linebuf/CE, 14 levels, 80% route, 325
      // endpoints). f_cbo_zero is a register captured with the request, so the enable is
      // gone from the compare and the cycle count is unchanged -- and it must be, because
      // Zicboz is advertised and cbo.zero is the kernel's clear_page, not a rarity.
      if (fst==F_FILLI) begin
         bk_wren  [vw*2+0] = 1'b1;
         bk_wraddr[vw*2+0] = { vi, pc[PAIRB-1:0] };
         bk_wrdata[vw*2+0] = f_cbo_zero ? {BANKW{1'b0}} : linebuf[(2*pc)  *BANKW +: BANKW];
         bk_wren  [vw*2+1] = 1'b1;
         bk_wraddr[vw*2+1] = { vi, pc[PAIRB-1:0] };
         bk_wrdata[vw*2+1] = f_cbo_zero ? {BANKW{1'b0}} : linebuf[(2*pc+1)*BANKW +: BANKW];
      end
      // store merge: write the low chunk (and same-line high chunk if the store spilled).
      // Uses the captured LINE0 way/idx (w0_*) -- in a span, the live hway/cih are line1's.
      // NOT qualified by `hit`: S_FIN is reached only through a hit, the request is solo, so
      // nothing can move the line between the two states -- asserted below, where it is a
      // check instead of a re-evaluation of the whole tag compare on the bank write enable.
      // ~r_cbo: a Zicbom op also passes through S_FIN (its dirty test) and carries no data.
      if (st==S_FIN && r_is_wr && !r_cbo) begin
         bk_wren  [w0_way*2 + clo[0]] = 1'b1;
         bk_wraddr[w0_way*2 + clo[0]] = { w0_idx, pair_lo };
         bk_wrdata[w0_way*2 + clo[0]] = nwin[0 +: BANKW];
         if (store_hi && !r_span) begin
            bk_wren  [w0_way*2 + (clo[0]^1'b1)] = 1'b1;
            bk_wraddr[w0_way*2 + (clo[0]^1'b1)] = { w0_idx, pair_hi };
            bk_wrdata[w0_way*2 + (clo[0]^1'b1)] = nwin[BANKW +: BANKW];
         end
      end
      // spanning store high half: write line1 chunk0 (even bank) of the line1 hit way,
      // captured at the phase-1 lookup (w1_*), not re-derived from the live compare.
      if (st==S_SPANW) begin
         bk_wren  [w1_way*2 + 0] = 1'b1;
         bk_wraddr[w1_way*2 + 0] = { w1_idx, {PAIRB{1'b0}} };
         bk_wrdata[w1_way*2 + 0] = nwin[BANKW +: BANKW];
      end
   end

   // ---- FSM ----
   reg inv_pend;     // sticky: an inv_req that arrives while the cache is busy is remembered
   always @(posedge clk) begin
      // status-array write ports: default idle every cycle (blocking; sites override)
      v_we = 1'b0; d_we = 1'b0; k_we = 1'b0;
      v_wa = {FW{1'b0}}; d_wa = {FW{1'b0}}; k_wa = {IDXB{1'b0}};
      v_wd = 1'b0; d_wd = 1'b0; k_wd = 1'b0;
      if (reset) begin
         st <= S_IDLE; fst <= F_IDLE; f_v <= 1'b0; f_replay <= 1'b0; b_live <= 1'b0;
         rd_valid <= 0; wr_ack <= 0; inv_busy <= 0;
         l2_req <= 0; l2_we <= 0; phase <= 0; fscan <= 0; inv_pend <= 0;
         pf_val <= 0; pf_want <= 0; pf_infl <= 0; pf_drop <= 0;
      end else begin
         rd_valid <= 0; wr_ack <= 0; l2_req <= 0;
         // The three cycles that address the banks for the request S_CHECK will hold next
         // cycle -- and only if the fill machine's victim stream did not take the read port
         // out from under them (that drive is last in the comb block, so it wins).
         b_live <= (accept | do_replay | (st == S_LOOK)) & ~fill_banks;
         // parallel prefetch engine: issue on the idle L2 port during hit-path
         // states (they never touch L2); mutual exclusion with demand fills is
         // GATED ON fst, NOT st. The original guard read "l2_req is only ever raised in
         // states outside the issue set" and named st states -- true while ONE machine owned
         // both the hit path and the fills, false the moment the fill machine was split out.
         // Since the split, st is S_IDLE for the WHOLE of a fill, so the prefetch's issue
         // window covers exactly the cycles the fill machine is using the L2 port.
         //
         // When both raise l2_req in the SAME cycle the collision is silent and lethal: this
         // block runs before both case statements, so the fill machine's l2_addr <= f_line
         // overwrites l2_addr <= pf_next and ONE request goes out -- but pf_infl is set, so on
         // the ack BOTH consume it. F_FILLW takes the line into linebuf and the prefetch takes
         // the SAME data into pf_line under pf_ia, the address it never actually fetched. A
         // later miss on pf_ia installs the demand line's bytes under pf_ia's tag: THE WRONG
         // LINE, with a correct tag, correct parity (computed on the write) and correct address
         // provenance (the row read is the row asked for). Every check in this file is blind to
         // it, and it corrupts INSTRUCTIONS, because PF_EN is the I$.
         //
         // ~fill_l2_busy: the prefetch takes the port only when the fill machine has nothing
         // issued OR outstanding on it. Blocking just the ISSUE cycle is not enough -- the
         // cycle after, l2_req is low and fst is F_FILLW, so the old guard would let the
         // prefetch straight in while the demand response is still in flight. It still
         // prefetches freely during F_WB, the writeback streaming, F_FILLI and F_ANS, which
         // is most of a fill.
         if (PF_EN) begin
            if (!pf_infl && pf_want && !l2_req && !fill_l2_busy
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
              // THE REQUEST REGISTERS ARE CAPTURED EVERY IDLE CYCLE, FROM THE SAME MUX THAT
              // ADDRESSES THE BANKS. Their clock-enable used to be `accept`, which is the
              // whole front door -- the requester's request (for the I$, the fetch buffer's
              // hit test behind the iTLB compare), acc_slot, the invalidate gates and the
              // solo rules -- fanned out to ~200 flops. A value captured in a cycle that is
              // NOT accepted is never read: nothing consumes r_*/cur_line while st is S_IDLE,
              // and the next idle cycle overwrites it. So the enable is the state alone and
              // `accept` decides only whether the pipeline leaves S_IDLE. The select is
              // f_replay, a register, exactly as a_live's is (rule I6 for the bank address).
              //
              // A completed fill's own request re-enters the pipeline that was held empty for
              // it; the banks were addressed from f_addr this cycle by the same a_live mux
              // the door uses, so from S_CHECK on it is an ordinary request again.
              r_is_wr    <= f_replay ? f_is_wr    : (wr_req && !rd_req);
              r_uncached <= f_replay ? f_uncached : (rd_req ? rd_uncached : wr_uncached);   // Svpbmt
              // CBO flags qualify a write-port maintenance op only. Reads win arbitration
              // (rd_req priority), so a cbo.zero waiting to drain can coincide with a load;
              // gating by (wr_req && !rd_req) stops cbo_zero latching onto that read and
              // making its refill zero-fill the line instead of fetching it.
              r_cbo      <= f_replay ? f_cbo      : ((wr_req && !rd_req) & cbo_req);
              r_cbo_zero <= f_replay ? f_cbo_zero : ((wr_req && !rd_req) & cbo_zero);
              r_cbo_keep <= f_replay ? f_cbo_keep : ((wr_req && !rd_req) & cbo_keep);   // Zicbom/Zicboz
              r_addr     <= a_live;
              r_tag      <= f_replay ? f_tag      : rd_tag;
              r_wdata    <= f_replay ? f_wdata    : wr_data;
              r_wmask    <= f_replay ? f_wmask    : wr_mask;
              r_off      <= a_live[OFFB-1:0];
              // a CBO is a single-line op (never spans); req_span is the door's own test
              r_span     <= f_replay ? f_span     : req_span;
              cur_line   <= {a_live[PAW-1:OFFB], {OFFB{1'b0}}};
              if (do_replay) begin
                 f_v <= 1'b0;  f_replay <= 1'b0;
                 st <= S_CHECK;
              end else if (accept)
                 st <= S_CHECK;    // banks already addressed this cycle (live drive above)
           end

           S_LOOK: st <= S_CHECK;

           S_CHECK: begin
              // THE COMPARE DECIDES; IT DOES NOT ENABLE. `hit` is the last signal this cycle
              // produces -- cur_line, the tag array, a 49-bit compare -- and it used to sit on
              // the clock-enable of the MSHR copy (f_*, ~160 flops), the window registers,
              // the response registers and the stream-buffer capture: ~900 enables on one
              // late net, 65-83% route. Everything below that is a COPY is captured on a
              // register-decoded condition instead and left as garbage when the other
              // outcome happens; only the bits that ARE the decision (st, f_v, fst, rd_valid,
              // the victim and the hit way/index) still wait for the compare.
              //
              // The MSHR copy: taken every S_CHECK cycle while no fill owns it. f_v is the
              // only bit that has to know whether this request missed, and the copy is
              // consumed only once f_v is set -- in the same cycle, from the same edge, as
              // it was under the old `hit` qualifier. A hit simply overwrites a copy nobody
              // reads. Never while f_v: those fields belong to the fill in flight.
              if (!f_v) begin
                 f_line <= cur_line;  f_addr <= r_addr;  f_tag <= r_tag;
                 f_is_wr <= r_is_wr;  f_uncached <= r_uncached;  f_span <= r_span;
                 f_cbo <= r_cbo;  f_cbo_zero <= r_cbo_zero;  f_cbo_keep <= r_cbo_keep;
                 f_wdata <= r_wdata;  f_wmask <= r_wmask;
              end
              // The stage-B window and the response fields, on the SAME register-decoded
              // qualifiers the decisions below use minus the compare. wlo/whi stay reserved
              // for solo requests (F_ANS borrows them for a plain read's fill answer, and a
              // solo request is never in the pipeline with a fill live); the fast read's
              // data and response registers are overwritten harmlessly when it misses or
              // holds, because rd_valid -- which does wait for the compare -- is not set.
              // A span's move to line1 is likewise captured here; a miss leaves through
              // S_IDLE, which resets phase, and the fill copy above took line0 this edge.
              if (!r_cbo && b_live) begin
                 if (!phase) begin
                    if (r_span || r_is_wr || r_uncached) begin
                       wlo <= clo[0] ? bk_rddata[hway*2+1] : bk_rddata[hway*2+0];
                       whi <= clo[0] ? bk_rddata[hway*2+0] : bk_rddata[hway*2+1];
                    end
                    if (r_span) begin phase <= 1; cur_line <= line1; end
                    rd_data <= hway ? fast1[RDW-1:0] : fast0[RDW-1:0];
                    rd_resp_addr <= r_addr; rd_resp_tag <= r_tag;
                 end else
                    whi <= bk_rddata[hway*2+0];           // line1 chunk0
              end
              // The stream buffer's line, for the miss that will install it (F_PFI). It has
              // to be taken AT THIS EDGE -- a prefetch ack can land this very cycle and
              // overwrite pf_line with a different line -- but not under `hit`: pf_hit is a
              // register compare, and ~f_v says the fill machine is idle, so linebuf is free.
              if (PF_EN && pf_hit && !f_v) linebuf <= pf_line;

              if (r_cbo) begin
                 // Zicbom/Zicboz: single-line maintenance on the addressed line.
                 if (hit) begin
                    w0_way <= hway; w0_idx <= cih;
                    if (r_cbo_zero) begin
                       // cbo.zero: overwrite the resident line with zeros (the install loop
                       // masks the data on f_cbo_zero) and mark it dirty
                       vw <= hway; vi <= cih; pc <= 0;
                       f_v <= 1'b1;
                       fst <= F_FILLI; st <= S_IDLE;
                    end else
                       // clean/flush/inval: the dirty test runs NEXT cycle in S_FIN, on the
                       // way/index just registered. Reading dirm[flat(hway,cih)] here was a
                       // second array read ADDRESSED BY THE FIRST ONE's compare -- the
                       // 18-level path (RAMD64E x2) that limited the module alone.
                       st <= S_FIN;
                 end else begin
                    if (r_cbo_zero) begin                 // miss: allocate a line, then zero-fill it
                       vw <= vicm[base_idx(cur_line)];
                       vi <= way_idx(vicm[base_idx(cur_line)] ? 1 : 0, cur_line);
                       f_v <= 1'b1;
                       fst <= F_WB; st <= S_IDLE;
                    end else begin wr_ack <= 1; st <= S_IDLE; end   // clean/flush/inval miss = no-op
                 end
              end else if (pipe_hold) begin
              // ONE REASON TO HOLD, not three. Everything S_CHECK consumes below comes out of
              // bk_rddata, which is the row addressed at the previous edge, so a stage that
              // waits has to be re-read and cannot simply sit still:
              //   ~b_live       the banks were not addressed for this request last cycle --
              //                 nothing drove them, or the fill machine's victim stream took
              //                 the read port.
              //   fst == F_ANS  an older request owns the response port this cycle (the fill
              //                 machine's case runs second, so its rd_* would win anyway).
              //   ~hit & f_v    ONE MSHR: a second miss waits for the fill in flight. It
              //                 cannot deadlock against it -- a plain read, the only thing
              //                 accepted while a fill runs, is answered by F_ANS and needs
              //                 nothing from this pipeline.
              // S_LOOK re-presents cur_line's chunks and comes straight back, so the window a
              // hit delivers was always read in the cycle before it. Holding IN S_CHECK is
              // what shipped another line's bytes to a load -- and to a page-table walk.
                 st <= S_LOOK;
              end else begin
                 if (hit) begin
                    if (!phase) begin
                       w0_way <= hway; w0_idx <= cih;       // remember line0 (for span store)
                       if (r_span) st <= S_LOOK;            // line1 next: window/cur_line above
                       else if (!r_is_wr && !r_uncached) begin
                          // fast read delivery: the window is live on the bank outputs and
                          // was captured above -- skip S_FIN. NC reads keep the slow path
                          // (S_FIN's flush-around). Reaching here means b_live and
                          // fst != F_ANS: the hold arm above owns both.
                          rd_valid <= 1;
                          st <= S_IDLE;
                       end
                       else st <= S_FIN;
                    end else begin
                       w1_way <= hway; w1_idx <= cih;       // line1's slot, for S_SPANW/S_NCI
                       st  <= S_FIN;
                    end
                 end else begin
                    vw <= vicm[base_idx(cur_line)];
                    vi <= way_idx(vicm[base_idx(cur_line)] ? 1 : 0, cur_line);
                    // stream-buffer hit: skip the L2 round trip. The line was captured
                    // above at this same edge; consume the buffer here (a same-edge ack's
                    // fresh line is discarded: hint loss only).
                    if (pf_hit) pf_val <= 0;
                    f_v <= 1'b1;
                    // THE REQUEST LEAVES THE PIPELINE HERE. That is the split: what used to be
                    // `st <= S_WB`, dragging the one machine into ten states from which the
                    // accept state was unreachable, is now a handoff that empties stage B.
                    fst <= pf_hit ? F_PFI : F_WB;
                    st  <= S_IDLE;
                 end
              end
           end

           // ---- hit terminal: deliver read / commit store ----
           S_FIN: begin
              if (r_cbo) begin
                 // Zicbom clean/flush/inval on a RESIDENT line, the cycle after the lookup:
                 // the dirty bit is read at the slot registered in S_CHECK. Dirty -> write
                 // the line back to L2 (reuses the WT push path), then finalize at S_WTA;
                 // clean -> flush/inval just invalidates, clean keeps it. One cycle later
                 // than before, for the only Zicbom ops there are; cbo.zero does not come
                 // here (its install loop starts straight from S_CHECK).
                 if (WRTHRU==0 && dirm[flat(w0_way,w0_idx)]) begin
                    wb_way <= w0_way; wb_idx <= w0_idx; pc <= 0; st <= S_WTR;
                 end else begin
                    if (!r_cbo_keep) begin v_we=1; v_wa=flat(w0_way,w0_idx); v_wd=1'b0; end
                    wr_ack <= 1; st <= S_IDLE;
                 end
              end else if (!r_is_wr) begin
                 rd_data  <= win_sh[RDW-1:0];
                 rd_valid <= 1; rd_resp_addr <= r_addr; rd_resp_tag <= r_tag;
                 // Svpbmt NC/IO load: return the (just-filled, current) word but don't keep the
                 // line, so a later DMA write isn't masked by a stale hit on the next NC load.
                 // A span clears line1 in S_NCI (one status write per cycle; data already out).
                 if (r_uncached) begin v_we=1; v_wa=flat(w0_way,w0_idx); v_wd=1'b0; end
                 st <= (r_uncached && r_span) ? S_NCI : S_IDLE;
              end else begin
                 // low-chunk (and same-line high) write driven combinationally this cycle.
                 if (r_span && store_hi) begin
                    // line1 hit way/idx are live (phase1); write its chunk0 next cycle.
                    // line0's dirty is staged here, line1's in S_SPANW (one write/cycle).
                    if (WRTHRU==0 && !r_uncached) begin d_we=1; d_wa=flat(w0_way,w0_idx); d_wd=1'b1; end
                    st <= S_SPANW;
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
              v_we=1; v_wa=flat(w1_way,w1_idx); v_wd=1'b0;
              st <= S_IDLE;
           end
           S_SPANW: begin                   // spanning store high half written combinationally
              if (WRTHRU!=0 || r_uncached) begin wb_way <= w0_way; wb_idx <= w0_idx; pc <= 0; st <= S_WTR; end
              else begin
                 d_we=1; d_wa=flat(w1_way,w1_idx); d_wd=1'b1;   // line0's dirty was staged at S_FIN
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
           S_WTI: begin l2_req<=1; l2_we<=1; l2_addr<=line0[PAW-1:OFFB]; l2_wdata<=linebuf; st<=S_WTA; end
           S_WTA: if (l2_ack) begin
              if (r_cbo) begin                                     // Zicbom writeback complete
                 d_we=1; d_wa=flat(wb_way,wb_idx); d_wd=1'b0;      // it is now clean in L2
                 if (!r_cbo_keep) begin v_we=1; v_wa=flat(wb_way,wb_idx); v_wd=1'b0; end  // flush/inval drop
              end else if (r_uncached) begin v_we=1; v_wa=flat(wb_way,wb_idx); v_wd=1'b0; end  // NC store: flush-around
              wr_ack <= 1; st <= S_IDLE;
           end

           // "This cannot happen" is an assertion or it is deleted (docs/rtl-rules.md).
           default: $fatal(1, "[cache id=%0d] FSM reached an undefined state st=%0d", PERF_ID, st);
         endcase

         // ==================== FILL MACHINE ====================================
         // Everything that takes an L2 round trip. It runs CONCURRENTLY with the lookup
         // pipeline above and, because this case comes second, its blocking assignments to
         // the status-array write ports simply win -- which is the fill-has-priority rule,
         // expressed by ordering rather than by a mux. `fill_status` is how the pipeline
         // finds out it lost and holds, instead of dropping its write on the floor.
         case (fst)
           F_IDLE: begin
              // An invalidate is a fill-machine job: it walks the whole array and it uses
              // linebuf. Accepts are already blocked by inv_go, so the pipeline drains into
              // this state on its own.
              if (inv_go && (st == S_IDLE) && !f_v) begin
                 inv_pend <= 1'b0;
                 pf_val <= 0; pf_want <= 0;      // prefetch buffer shares the cache's fate
                 if (pf_infl) pf_drop <= 1;      // in-flight line predates the flush: land it dead
                 // Every config walks F_FLUSH (one line/cycle behind inv_busy, which all
                 // requesters already poll). For WRITABLE==0/WRTHRU the dirty-writeback
                 // branch is compile-time dead, leaving a pure valid-clear scan -- the
                 // old one-cycle full clear was what broke the valm RAM inference.
                 inv_busy <= 1; fscan <= 0; flush_clean <= (WRITABLE!=0 && WRTHRU==0) & inv_clean;
                 fst <= F_FLUSH;
              end
           end

           // ---- miss: evict victim (writeback if dirty) then fill ----
           F_WB: begin
              // THE VICTIM STOPS BEING VALID HERE, before a single byte of it is
              // overwritten. Its data is still readable for the writeback below (that
              // streams by {wb_way,wb_idx}, not by the valid bit), and from this cycle on a
              // lookup of the old tag MISSES instead of hitting a line that is half the old
              // one and half the new. Free -- F_WB used no status write port -- and it is
              // what makes the install safe to run alongside lookups.
              v_we=1; v_wa=vflat; v_wd=1'b0;
              if (WRITABLE!=0 && WRTHRU==0 && valm[vflat] && dirm[vflat]) begin
                 wb_way <= vw?1'b1:1'b0; wb_idx <= vi; pc <= 0;
                 wb_laddr <= {{(PAW-PAW_SIG){1'b0}}, vtag, vbase};
                 fst <= F_WBR;
              end else fst <= f_cbo_zero ? F_ZFILL : F_FILL;
           end
           F_WBR: fst <= F_WBW;
           F_WBW: begin
              linebuf[(2*pc)  *BANKW +: BANKW] <= bk_rddata[wb_way*2+0];
              linebuf[(2*pc+1)*BANKW +: BANKW] <= bk_rddata[wb_way*2+1];
              if (pc == HALF-1) begin pc <= 0; fst <= F_WBI; end
              else begin pc <= pc + 1'b1; fst <= F_WBR; end
           end
           F_WBI: begin l2_req<=1; l2_we<=1; l2_addr<=wb_laddr; l2_wdata<=linebuf; fst<=F_WBA; end
           F_WBA: if (l2_ack) fst <= f_cbo_zero ? F_ZFILL : F_FILL;

           // cbo.zero miss: victim evicted -> install a fresh zero line (no L2 read) + mark
           // dirty. The zeros come from the install loop's f_cbo_zero mask; this state only
           // starts the loop, and it stays a state so the miss path's cycle count is unchanged.
           F_ZFILL: begin                   // bookkeeping deferred to the last install cycle
              pc <= 0;
              fst <= F_FILLI;
           end

           F_FILL: if (PF_EN && pf_hit_f) begin
              // the missing line is in (or just landed in) the buffer: install it.
              // Same decision-edge capture/consume as the S_CHECK shortcut.
              linebuf <= pf_line; pf_val <= 0;
              fst <= F_PFI;
           end else if (PF_EN && pf_infl) begin
              // a prefetch is mid-flight on the L2 port: wait it out (it may be
              // exactly the missing line, caught by the branch above on landing).
           end else begin l2_req<=1; l2_we<=0; l2_addr<=f_line[PAW-1:OFFB]; fst<=F_FILLW; end
           F_FILLW: if (l2_ack) begin
              // The line bookkeeping that stood here -- tag, valid, dirty, victim -- has
              // moved to the LAST install cycle. Advertising a line as valid before its
              // data is in the banks is only harmless while nothing can look it up; the
              // whole point of splitting the fill machine out is that something can.
              linebuf <= l2_rdata; pc <= 0;
              fst <= F_FILLI;
              if (PF_EN && !f_uncached && !f_cbo_zero) begin
                 pf_want <= 1; pf_next <= f_line[PAW-1:OFFB] + 1'b1;  // arm next-line
              end
           end

           // ---- prefetch: victim already chosen at S_CHECK; install the buffered line
           // (same bookkeeping as F_FILLW but sourced from pf_line, no L2), then re-arm
           // for the line after -- consumption-chained streaming.
           // prefetch install: linebuf was captured (and the buffer consumed) at the
           // decision edge in S_CHECK/F_FILL; here only the line bookkeeping + re-arm.
           F_PFI: begin
              pc <= 0;
              // S_CHECK jumps straight here on a stream-buffer hit, skipping F_WB, so this
              // is the one fill entry that owes the victim invalidate itself. Reaching it
              // FROM F_FILL invalidates twice, which is idempotent.
              v_we=1; v_wa=vflat; v_wd=1'b0;
              pf_want <= 1; pf_next <= f_line[PAW-1:OFFB] + 1'b1;
              fst <= F_FILLI;
           end
           F_FILLI: begin                  // install pair pc (bank writes combinational)
              if (pc == HALF-1) begin
                 pc <= 0;
                 // THE LINE BECOMES VALID HERE, in the cycle its last chunk is written, and
                 // not before. One site for every fill path -- demand, cbo.zero and the
                 // stream buffer -- so there is one answer to "when is a filled line
                 // visible". cbo.zero installs a line that is dirty by construction: it was
                 // never read from L2, so L2 does not have these zeros.
                 tagm[vflat] <= tag_of(f_line);
                 v_we=1; v_wa=vflat; v_wd=1'b1;
                 d_we=1; d_wa=vflat; d_wd=f_cbo_zero;
                 k_we=1; k_wa=base_idx(f_line); k_wd=~vicm[base_idx(f_line)];
                 // HOW THE MISSING REQUEST IS COMPLETED, and there are two answers because
                 // a read wants data out and a store wants data in.
                 //   plain cached read -> answered HERE, from linebuf, in F_ANS. It never
                 //     re-enters the pipeline, and that is not an optimisation: a second miss
                 //     stalls stage B, which would block the replay that frees the MSHR that
                 //     stage B is waiting for. Answering directly is what has no deadlock.
                 //   everything else    -> replayed through the pipeline exactly as before.
                 //     Safe because such a request is `solo`: the pipeline was held empty for
                 //     it, so the slot is there when it asks.
                 // The two chunks the read needs are selected out of linebuf and handed to the
                 // SAME shift network a hit uses (win_sh), so this costs one 8:1 chunk mux and
                 // not the 512->64 byte mux that answering from a line buffer usually implies.
                 if (f_cbo_zero) begin wr_ack <= 1; fst <= F_IDLE; f_v <= 1'b0; end
                 else if (f_is_wr | f_cbo | f_uncached | f_span) begin
                    f_replay <= 1'b1; fst <= F_IDLE;
                 end else begin
                    wlo <= linebuf[(f_clo*BANKW) +: BANKW];
                    whi <= linebuf[(f_cnx*BANKW) +: BANKW];
                    fst <= F_ANS;
                 end
              end else pc <= pc + 1'b1;
           end


           // The one cycle that belongs to neither machine: the missing READ is answered
           // from the line that just landed. It owns the response port here -- the pipeline
           // yields (see pipe_deliver), because this request is older than anything in it.
           F_ANS: begin
              rd_data  <= f_win_sh[RDW-1:0];
              rd_valid <= 1; rd_resp_addr <= f_addr; rd_resp_tag <= f_tag;
              f_v <= 1'b0;
              fst <= F_IDLE;
           end

           // ---- flush (write-back configs) ----
           F_FLUSH: begin
              if (fscan == NW) begin inv_busy <= 0; fst <= F_IDLE; end
              else if (WRITABLE!=0 && WRTHRU==0 && valm[fscan[FW-1:0]] && dirm[fscan[FW-1:0]]) begin
                 wb_way <= fscan[FW-1]; wb_idx <= fidx; pc <= 0;
                 wb_laddr <= {{(PAW-PAW_SIG){1'b0}}, ftag, fbase};
                 fst <= F_FLUSHR;
              end else begin
                 if (!flush_clean) begin v_we=1; v_wa=fscan[FW-1:0]; v_wd=1'b0; end  // clean flush keeps lines valid
                 d_we=1; d_wa=fscan[FW-1:0]; d_wd=1'b0;
                 fscan <= fscan + 1'b1;
              end
           end
           F_FLUSHR: fst <= F_FLUSHW;
           F_FLUSHW: begin
              linebuf[(2*pc)  *BANKW +: BANKW] <= bk_rddata[wb_way*2+0];
              linebuf[(2*pc+1)*BANKW +: BANKW] <= bk_rddata[wb_way*2+1];
              if (pc == HALF-1) begin pc <= 0; fst <= F_FLUSHI; end
              else begin pc <= pc + 1'b1; fst <= F_FLUSHR; end
           end
           F_FLUSHI: begin l2_req<=1; l2_we<=1; l2_addr<=wb_laddr; l2_wdata<=linebuf; fst<=F_FLUSHA; end
           F_FLUSHA: if (l2_ack) begin
              if (!flush_clean) begin v_we=1; v_wa=fscan[FW-1:0]; v_wd=1'b0; end  // clean flush: written back, stays valid+clean
              d_we=1; d_wa=fscan[FW-1:0]; d_wd=1'b0;
              fscan <= fscan + 1'b1; fst <= F_FLUSH;
           end
           default: $fatal(1, "[cache id=%0d] fill machine reached an undefined state fst=%0d", PERF_ID, fst);
         endcase
      end
      // the single write port of each status array (see staging decl above)
      if (v_we) valm[v_wa] <= v_wd;
      if (d_we) dirm[d_wa] <= d_wd;
      if (k_we) vicm[k_wa] <= k_wd;
`ifdef CDBG
      if (fst==F_FLUSH && (fscan[3:0]==0 || fscan==NW))
         $display("[CDBG %m] t=%0t S_FLUSH fscan=%0d/%0d invb=%b", $time, fscan, NW, inv_busy);
      if (st==S_IDLE && inv_go)
         $display("[CDBG %m] t=%0t IDLE->inv (pend=%b)", $time, inv_pend);
`endif
   end

`ifdef CACHE_PARITY
   // ---- data-array integrity check (ILA_PARITY trigger source) -----------------------
   // PORTED FROM src/cache.v, which this file is a fork of and which had it while the
   // shipping cache did not -- the same drift that left the fork without two upstream
   // repairs. The board's corruption is SILENT: by the time the kernel Oopses we are
   // millions of cycles past the bad read, far beyond any ILA pre-trigger depth. A parity
   // bit per bank word, written on every bank write and checked on every bank read, makes
   // the hardware SAY "this array just returned something other than what was stored" in
   // the cycle it happens.
   //
   // Either outcome is decisive. If it fires, the D$ data path is corrupting on real BRAM
   // and we have the address and the bank. If it NEVER fires while the board still Oopses,
   // the data arrays are EXONERATED and the fault is upstream of them -- the tag/valid
   // logic, the LSU, the MMU -- which is exactly the discrimination seven simulation
   // approaches could not make.
   //
   // A parallel array in distributed RAM: no BRAM geometry change, no data-path change,
   // and the compare feeds a FLOP rather than the read path, so it does not lengthen the
   // cone that is already at the timing limit. Opt-in (-DCACHE_PARITY): a diagnostic
   // bitstream, not the production one.
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
   // ---- ADDRESS PROVENANCE (the half parity is blind to) ----------------------------
   // Parity proves the array returned what was stored AT THE ADDRESS PRESENTED. It cannot
   // see the wrong address being presented -- and that is exactly the defect class already
   // found here: a held stage B delivered bank ROW 0, which is valid data with valid parity
   // for row 0. The board's remaining fault survived a parity-clean boot, so it is in the
   // addressing, not the bits.
   //
   // This is the simulation-only WINDOW PROVENANCE assertion at the bottom of this file,
   // made synthesizable and folded into the same error pulse: on the cycle a hit consumes
   // its window, the address that was on the bank read ports at the previous edge must be
   // THIS request's row. Comparators only, feeding a flop, so it adds no depth to the read
   // path. adr_sticky distinguishes it from a parity failure in the snapshot.
   reg [BAW-1:0] b_rda_s [0:2*WAYS-1];
   integer brs;
   always @(posedge clk) for (brs=0; brs<2*WAYS; brs=brs+1) b_rda_s[brs] <= bk_rdaddr[brs];
   // PIPELINED BY A CYCLE, deliberately. Comparing combinationally off st/hit/b_live/hway
   // put a compare on the hit path and cost 0.818 ns -- on a design with 24 ps of margin
   // that is not a diagnostic, it is a build failure. Everything the check needs is latched
   // first and compared from FLOPS the next cycle, so the only new logic is
   // flop -> compare -> flop, off every existing path. The snapshot is read post-mortem
   // over JTAG; one cycle of delay is irrelevant to it.
   reg            chk_v;
   reg            chk_phase;
   reg [BAW-1:0]  chk_got_e, chk_got_o, chk_exp_e, chk_exp_o;
   always @(posedge clk) begin
      chk_v     <= (st == S_CHECK) && !r_cbo && hit && b_live;
      chk_phase <= phase;
      chk_got_e <= b_rda_s[hway*2+0];
      chk_got_o <= b_rda_s[hway*2+1];
      chk_exp_e <= phase ? {cih, {PAIRB{1'b0}}} : {cih, pair_e};
      chk_exp_o <= {cih, pair_o};
   end
   wire adr_bad = chk_v && ((chk_got_e != chk_exp_e) ||
                            (!chk_phase && (chk_got_o != chk_exp_o)));
   reg adr_err; reg adr_sticky; reg [BAW-1:0] adr_got; reg [BAW-1:0] adr_want;
   initial begin adr_err=1'b0; adr_sticky=1'b0; adr_got={BAW{1'b0}}; adr_want={BAW{1'b0}}; end
   always @(posedge clk) begin
      adr_err <= 1'b0;
      if (reset) adr_sticky <= 1'b0;
      else if (adr_bad) begin
         adr_err <= 1'b1;
         if (!adr_sticky) begin
            adr_sticky <= 1'b1; adr_got <= chk_got_e; adr_want <= chk_exp_e;
         end
`ifndef SYNTHESIS
         $display("[cache id=%0d] ADDR PROVENANCE: bank read %h, this hit needs %h",
                  PERF_ID, chk_got_e, chk_exp_e);
`endif
      end
   end

   reg par_err;  reg par_sticky;  reg [BAW-1:0] par_addr;  reg [2:0] par_bank;
   wire [15:0] par_addr16 = {{(16-BAW){1'b0}}, par_addr};
   initial begin par_err=1'b0; par_sticky=1'b0; par_addr={BAW{1'b0}}; par_bank=3'd0; end
   always @(posedge clk) begin
      par_err <= 1'b0;
      for (pb=0; pb<2*WAYS; pb=pb+1) begin
         if (bk_wren[pb]) par_mem[pb][bk_wraddr[pb]] <= ^bk_wrdata[pb];
         // the address presented this cycle yields data NEXT cycle (READ_LATENCY=1), so
         // carry the expectation forward. The NBA read of par_mem yields the OLD parity,
         // which is what read_first returns on the data side for a same-cycle write.
         par_rd_a[pb] <= bk_rdaddr[pb];
         par_exp [pb] <= par_mem[pb][bk_rdaddr[pb]];
         // ONLY READS THAT WILL BE CONSUMED. This fork drives the lookup's bank address
         // UNCONDITIONALLY (rule I6 -- nothing that decodes FSM state belongs on a BRAM
         // address pin), so most cycles present an address nobody will look at, including
         // same-address collisions with an install. Those are harmless -- a BRAM collision
         // corrupts the read, never the write, and the read is discarded -- but a checker
         // that flags them cries wolf on every fill, which is exactly what it did before
         // this line existed. bk_rd_drv is the same qualifier the collision invariant uses.
         par_rd_v[pb] <= bk_rd_drv;
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
      if (reset) begin par_sticky <= 1'b0; par_err <= 1'b0; end
   end
`endif

   // ---- invariants (docs/rtl-rules.md A1): always on, no `ifdef ----------------------
   // A FLUSH IS BOUNDED: one line per cycle plus, on the D$, a writeback per dirty line. So
   // inv_busy being high is bounded too, and BOTH ways this cache has lost a scan end the
   // same way -- inv_busy high forever with every requester polling it. That has exactly one
   // symptom on a board, which is that the board stops, so the bound is checked here where
   // it can name the cache and the state it died in. Nothing but the $fatal reads inv_age,
   // so synthesis drops it; simulation keeps it.
   localparam INV_MAX = NW * 1024;
   reg [$clog2(INV_MAX+1)-1:0] inv_age;
   always @(posedge clk)
      if (reset || !inv_busy)        inv_age <= 0;
      else if (inv_age != INV_MAX)   inv_age <= inv_age + 1'b1;

   integer ai;
   always @(posedge clk) if (!reset) begin
      // linebuf/pc/wb_* are ONE set of registers with two users -- the fill machine's
      // writeback and the pipeline's write-through push. They are kept apart by the solo
      // rule, not by a mux, so the day that rule is loosened this is what says so.
      if (pipe_uses_lb && (fst != F_IDLE))
         $fatal(1, "[cache id=%0d] write-through push and fill machine both own linebuf (st=%0d fst=%0d)",
                PERF_ID, st, fst);
      // A solo request's miss is replayed through this pipeline, which is only sound while
      // the pipeline is empty for it.
      if (accept && req_solo && f_v)
         $fatal(1, "[cache id=%0d] solo request accepted while a fill is live", PERF_ID);
      // THE TAG IS PAW_SIG BITS WIDE. Two addresses that differ only above bit PAW_SIG-1
      // would hit the same line; the platform has no memory there, so such an address is a
      // defect somewhere upstream (a wild PTE, a device decode that let something through)
      // and the cache is where it would become silent corruption. It is a fault here instead.
      if (PAW > PAW_SIG && accept && (|a_live[PAW-1:PAW_SIG]))
         $fatal(1, "[cache id=%0d] request %h lies above the %0d-bit tagged physical range",
                PERF_ID, a_live, PAW_SIG);
      // THE LINE A SOLO REQUEST HIT IS STILL THERE IN THE STATES AFTER S_CHECK. The store
      // merge (S_FIN), the span's second half (S_SPANW), the NC drop (S_NCI) and the Zicbom
      // dirty test (S_FIN) all act on the way/index REGISTERED at the lookup and no longer
      // re-evaluate the compare; that is sound because a solo request is alone in the cache
      // -- no fill is live, no invalidate scan can start while st != S_IDLE -- so the tag
      // cannot move under it. This is that assumption, checked every cycle.
      if ((st == S_FIN || st == S_SPANW || st == S_NCI) && !hit)
         $fatal(1, "[cache id=%0d] line %h vanished between S_CHECK and st=%0d", PERF_ID, cur_line, st);
      // A CBO is solo; the MSHR copy taken in S_CHECK assumes no fill owns f_* while one is
      // in stage B, and the replay's select assumes f_replay implies a fill in flight.
      if ((st == S_CHECK) && r_cbo && f_v)
         $fatal(1, "[cache id=%0d] a CBO is in stage B while a fill is live", PERF_ID);
      if (f_replay && !f_v)
         $fatal(1, "[cache id=%0d] replay owed with no fill in flight", PERF_ID);
      // TWO RAISERS, ONE CYCLE. 5ce1666a asserted "a second L2 request while one is
      // outstanding" (l2_req && l2_out && !l2_ack) and concluded the race does not occur in
      // 300 M cycles. It cannot: when the prefetch and the fill machine raise l2_req in the
      // SAME cycle, l2_out is not yet set, one address wins the register, and both consume
      // the ack. That is the shape that files a demand line under a prefetch address.
      // TWO CONSUMERS, ONE ACK -- stated as the hazard rather than as one way of reaching
      // it. The prefetch waiting on l2_ack while the fill machine is ALSO waiting means one
      // response will be latched by both: F_FILLW takes it into linebuf and the prefetch
      // takes the same bytes into pf_line under pf_ia, an address it never fetched. A later
      // miss on pf_ia then installs the wrong line under a correct tag -- invisible to
      // parity (computed on the write) and to address provenance (the row read is the row
      // asked for). Checking the ISSUE condition instead would only restate whichever gate
      // is currently in the guard, and would stop firing the moment the guard changed.
      if (PF_EN && pf_infl && ((fst==F_FILLW) || (fst==F_WBA) || (fst==F_FLUSHA)))
         $fatal(1, "[cache id=%0d] prefetch and fill machine both awaiting l2_ack (fst=%0d pf_ia=%h f_line=%h)",
                PERF_ID, fst, pf_ia, f_line[PAW-1:OFFB]);
      if (inv_age == INV_MAX)
         $fatal(1, "[cache id=%0d] invalidate has not finished in %0d cycles -- the scan was lost (fscan=%0d/%0d st=%0d fst=%0d f_v=%b f_replay=%b inv_pend=%b)",
                PERF_ID, INV_MAX, fscan, NW, st, fst, f_v, f_replay, inv_pend);
      // THE SCAN OWNS THE FILL MACHINE. A miss taken during an invalidate hands off with
      // `fst <= F_WB`, and F_FLUSH's clean arm assigns no fst, so that write lands and the
      // scan is abandoned -- with inv_pend already clear, leaving inv_busy high for good
      // and the fence.i FSM waiting on it forever. acc_slot's ~inv_busy is what stops it;
      // this is the check that says so, because the silent version of this is a dead board.
      if (f_v && (fst == F_FLUSH || fst == F_FLUSHR || fst == F_FLUSHW
                  || fst == F_FLUSHI || fst == F_FLUSHA))
         $fatal(1, "[cache id=%0d] a fill is live during the invalidate scan (fscan=%0d) -- the scan is being abandoned",
                PERF_ID, fscan);
      // NO BANK ROW IS READ AND WRITTEN IN THE SAME CYCLE, whoever the reader is. It was
      // qualified by `accept`, which named one of four readers -- the accept-cycle lookup, the
      // S_LOOK window read, the writeback stream and the write-through push -- and the one it
      // did not name is the one that fires: a lookup re-reading its row while F_FILLI installs.
      // On the behavioural model that read returns pre-write data; on the BRAM it returns
      // INVALID data, which is why this is a hardware bug that simulation shows as a clean run.
      // It is also the cross-module check for the day the LSU goes multiple-outstanding:
      // ooo2_sq blocks a load whose bytes overlap an older uncommitted store and holds the
      // entry until this cache acknowledges the write, so no such pair can arrive back to back.
      for (ai=0; ai<2*WAYS; ai=ai+1)
         if (bk_rd_drv && bk_wren[ai] && (bk_rdaddr[ai] == bk_wraddr[ai]))
            $fatal(1, "[cache id=%0d] bank %0d row %0d read and written in the same cycle",
                   PERF_ID, ai, bk_wraddr[ai]);
   end

   // Zihpm hardware cache events: one pulse per resolved line lookup (S_CHECK), and per miss.
   // soc_top taps these -> hpm_ev -> csr_file's mhpmcounters.
   //
   // These were INSIDE `ifdef PERF_TRACE below, contradicting their own comment ("always-on,
   // unlike the PERF_TRACE DPI trace"). No FPGA build defines PERF_TRACE, so in every
   // bitstream these outputs were undriven -- tied low by synthesis -- and mhpmcounter's
   // DCACC/DCMISS/ICACC/ICMISS events counted zero on hardware. Verilator's UNDRIVEN caught
   // it the moment the in-order core was brought under src/lint.sh.
   //
   // Same defect and same fix as src/cache.v, which this file is a fork of; it was repaired
   // upstream and the fork never picked it up. Upstream additionally masks a PARKED S_CHECK
   // (a miss looping while it waits on the MSHR, or a CBO waiting on the write buffer) so
   // misses do not overcount. This fork has neither an MSHR nor a write buffer, so that term
   // is identically zero here and is omitted rather than carried as dead code.
   // One pulse per RESOLVED lookup. A stage-B request that is holding -- for the MSHR, or
   // for the response port -- has not resolved anything and must not be counted twice.
   wire pipe_resolve  = (st == S_CHECK) & ~pipe_hold;
   assign perf_access = pipe_resolve;
   assign perf_miss   = pipe_resolve & ~hit;

`ifdef PERF_TRACE
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
      // lookup resolved this cycle (one event per line lookup; a span resolves twice)
      if (st == S_CHECK) begin
         perf_ev(perf_cyc, 8, hit ? 0 : 1, PERF_ID, {31'd0, r_is_wr},
                 0, 0, 0, {{(64-PAW){1'b0}}, cur_line}, 0);
         if (!hit) perf_miss_cyc <= perf_cyc;
      end
      // capture store accept (for the accept->ack latency)
      if (st == S_IDLE && wr_req && !rd_req && WRITABLE != 0) perf_st_cyc <= perf_cyc;
      // refill complete -> re-lookup: emit the miss penalty
      if (fst == F_FILLI && pc == HALF-1)
         perf_ev(perf_cyc, 8, 2, PERF_ID, 0, 0, 0, 0, {{(64-PAW){1'b0}}, cur_line},
                 (perf_cyc - perf_miss_cyc));
      // store committed (wr_ack asserted last cycle; +1 constant offset, fine for a hist)
      if (wr_ack)
         perf_ev(perf_cyc, 8, 3, PERF_ID, 1, 0, 0, 0, {{(64-PAW){1'b0}}, r_addr},
                 (perf_cyc - perf_st_cyc));
   end
`endif

`ifndef SYNTHESIS
   // WINDOW PROVENANCE. The defect this checks independently of the b_live bit that fixes
   // it: S_CHECK shifts bk_rddata, which is whatever address was on the bank read ports at
   // the previous edge, and every cycle the stage holds is a cycle that address was some
   // other row (0 when nothing drove it, the victim's chunk while a writeback streams).
   // Delivered to a load it is a wrong value; delivered to a page-table walk it is a wrong
   // PTE, and the board dies with a paging fault on an address that is mapped.
   reg [BAW-1:0] b_rda [0:2*WAYS-1];
   integer bri;
   always @(posedge clk) for (bri=0; bri<2*WAYS; bri=bri+1) b_rda[bri] <= bk_rdaddr[bri];
   always @(posedge clk) if (!reset && (st == S_CHECK) && !r_cbo && hit && b_live) begin
      if (b_rda[hway*2+0] != (phase ? {cih, {PAIRB{1'b0}}} : {cih, pair_e}))
         $fatal(1, "[cache id=%0d] STALE WINDOW: even bank read %h, this hit needs %h (line=%h phase=%b fst=%0d)",
                PERF_ID, b_rda[hway*2+0], phase ? {cih, {PAIRB{1'b0}}} : {cih, pair_e}, cur_line, phase, fst);
      if (!phase && (b_rda[hway*2+1] != {cih, pair_o}))
         $fatal(1, "[cache id=%0d] STALE WINDOW: odd bank read %h, this hit needs %h (line=%h fst=%0d)",
                PERF_ID, b_rda[hway*2+1], {cih, pair_o}, cur_line, fst);
   end

   // ---- corruption invariants (ported from src/cache.v, c7f06b39 / dc95e905) --------
   // This fork predates both fixes, so it has neither the fix nor the guard. These fire
   // AT the bad write instead of at the symptom, which on the board arrives millions of
   // cycles later as a wrong load.

   // STORE-SLOT: a non-spanning store merge at S_FIN must write the slot that actually
   // holds ITS line. If the tag there is no longer this store's tag, the store is being
   // merged into an unrelated resident line -- silent corruption, surfacing only when
   // that line is read back.
   always @(posedge clk) if (!reset && (WRITABLE != 0))
      if ((st == S_FIN) && r_is_wr && !r_cbo && hit && !r_span && !phase
          && valm[flat(w0_way, w0_idx)]
          && (tagm[flat(w0_way, w0_idx)] != tag_of(cur_line)))
         $fatal(1, "[cache id=%0d] STORE-SLOT MISMATCH: store line=%h (tag=%h) into way=%0d idx=%h holding tag=%h",
                PERF_ID, cur_line, tag_of(cur_line), w0_way, w0_idx,
                tagm[flat(w0_way, w0_idx)]);

   // NO-SPAN (experiment, inorder-nospan): with the LSU splitting line-crossing
   // accesses into two aligned ones, the D$ must never be asked to span. If this ever
   // fires the experiment is invalid -- the span path is still live. The I$ is exempt:
   // instruction fetch legitimately spans lines for a misaligned 128-bit window.
   always @(posedge clk) if (!reset && (WRITABLE != 0) && r_span)
      $fatal(1, "[cache id=%0d] NO-SPAN VIOLATED: D$ saw a spanning request addr=%h", PERF_ID, r_addr);

   // SPAN-LOW-STALE: the exposure this fork actually has. A line-crossing store writes
   // line0's low half at S_FIN via w0_way/w0_idx -- a slot named at the phase-0 lookup
   // and carried across the phase-1 lookup for line1. That lookup can miss, and its fill
   // can evict line0's slot underneath the store; S_FIN then merges into whichever line
   // moved in and marks it dirty, publishing a foreign word to DRAM under the innocent
   // line's address. Upstream removed the exposure by committing line0 at its own live
   // lookup (c7f06b39); here we check the slot still holds line0 at the write.
   always @(posedge clk) if (!reset && (WRITABLE != 0))
      if ((st == S_FIN) && r_is_wr && hit && r_span
          && valm[flat(w0_way, w0_idx)]
          && (tagm[flat(w0_way, w0_idx)] != tag_of(line0)))
         $fatal(1, "[cache id=%0d] SPAN-LOW-STALE: span line0=%h (tag=%h) into way=%0d idx=%h holding tag=%h",
                PERF_ID, line0, tag_of(line0), w0_way, w0_idx,
                tagm[flat(w0_way, w0_idx)]);
`endif

endmodule

`default_nettype wire
