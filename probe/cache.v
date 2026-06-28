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
   parameter PERF_ID  = 0          // perf-trace cache id (0=I$, 1=D$); see perf block
) (
   input  wire             clk,
   input  wire             reset,
   input  wire             rd_req,
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
   input  wire             l2_ack
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
   wire [IDXB-1:0]  vbase = vw ? (vi ^ vtag[IDXB-1:0]) : vi;

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
      S_INVDONE=22, S_ZFILL=23;
   reg [4:0] st;

   integer b, bb, w2;
   reg [2*BANKW-1:0] nwin;
   reg [LZB:0]       pos;

   // store-merge window (combinational)
   always @* begin
      nwin = win;
      for (bb=0; bb<WRB; bb=bb+1) if (r_wmask[bb]) begin
         pos = {1'b0,bwc} + bb[LZB:0];
         nwin[pos*8 +: 8] = r_wdata[bb*8 +: 8];
      end
   end

   // ---- combinational bank port drive ----
   always @* begin
      for (b=0; b<2*WAYS; b=b+1) begin
         bk_rdaddr[b] = {BAW{1'b0}};
         bk_wren[b]   = 1'b0;
         bk_wraddr[b] = {BAW{1'b0}};
         bk_wrdata[b] = {BANKW{1'b0}};
      end

      // window read: present line's chunks so data is valid next cycle (both ways read)
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
      if (st==S_FILLI) begin
         bk_wren  [vw*2+0] = 1'b1;
         bk_wraddr[vw*2+0] = { vi, pc[PAIRB-1:0] };
         bk_wrdata[vw*2+0] = linebuf[(2*pc)  *BANKW +: BANKW];
         bk_wren  [vw*2+1] = 1'b1;
         bk_wraddr[vw*2+1] = { vi, pc[PAIRB-1:0] };
         bk_wrdata[vw*2+1] = linebuf[(2*pc+1)*BANKW +: BANKW];
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
   reg inv_pend;     // sticky: an inv_req that arrives while the cache is busy is remembered
   always @(posedge clk) begin
      if (reset) begin
         st <= S_IDLE; rd_valid <= 0; wr_ack <= 0; inv_busy <= 0;
         l2_req <= 0; l2_we <= 0; phase <= 0; fscan <= 0; inv_pend <= 0;
      end else begin
         rd_valid <= 0; wr_ack <= 0; l2_req <= 0;
         // Latch + ACK an invalidate the cycle it is requested, even if the cache is mid-
         // operation (inv_req is only acted on at S_IDLE). Without this a 1-cycle inv_req
         // pulse arriving during a refill is silently dropped -> a stale line survives a
         // fence.i / sfence flush. Raising inv_busy now also keeps the requester waiting
         // until the invalidate actually runs (it polls !inv_busy).
         if (inv_req) begin inv_pend <= 1'b1; inv_busy <= 1'b1; end
         case (st)
           S_IDLE: begin
              phase <= 0;
              if (inv_req | inv_pend) begin
                 inv_pend <= 1'b0;
                 if (WRITABLE==0 || WRTHRU!=0) begin
                    for (b=0;b<NW;b=b+1) valm[b] <= 1'b0;
                    inv_busy <= 1; st <= S_INVDONE;
                 end else begin inv_busy <= 1; fscan <= 0; flush_clean <= inv_clean; st <= S_FLUSH; end
              end else if (rd_req || (wr_req && WRITABLE!=0)) begin
                 r_is_wr  <= wr_req && !rd_req;
                 r_uncached <= rd_req ? rd_uncached : wr_uncached;   // Svpbmt
                 r_cbo    <= cbo_req; r_cbo_zero <= cbo_zero; r_cbo_keep <= cbo_keep;   // Zicbom/Zicboz
                 r_addr   <= rd_req ? rd_addr : wr_addr;
                 r_wdata  <= wr_data; r_wmask <= wr_mask;
                 r_off    <= rd_req ? rd_addr[OFFB-1:0] : wr_addr[OFFB-1:0];
                 // a CBO is a single-line op (never spans)
                 r_span   <= ~cbo_req & (({1'b0,(rd_req ? rd_addr[OFFB-1:0] : wr_addr[OFFB-1:0])}
                               + (rd_req ? RDB : WRB)) > WORDB);
                 cur_line <= {(rd_req ? rd_addr[PAW-1:OFFB] : wr_addr[PAW-1:OFFB]), {OFFB{1'b0}}};
                 st <= S_LOOK;
              end
           end

           S_LOOK: st <= S_CHECK;

           S_CHECK: if (r_cbo) begin
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
                    if (!r_cbo_keep) valm[flat(hway,cih)] <= 1'b0;
                    wr_ack <= 1; st <= S_IDLE;
                 end
              end else begin
                 if (r_cbo_zero) begin                 // miss: allocate a line, then zero-fill it
                    vw <= vicm[base_idx(cur_line)];
                    vi <= way_idx(vicm[base_idx(cur_line)] ? 1 : 0, cur_line);
                    st <= S_WB;
                 end else begin wr_ack <= 1; st <= S_IDLE; end   // clean/flush/inval miss = no-op
              end
           end else begin
              if (hit) begin
                 if (!phase) begin
                    wlo <= clo[0] ? bk_rddata[hway*2+1] : bk_rddata[hway*2+0];
                    whi <= clo[0] ? bk_rddata[hway*2+0] : bk_rddata[hway*2+1];
                    w0_way <= hway; w0_idx <= cih;       // remember line0 (for span store)
                    if (r_span) begin phase <= 1; cur_line <= line1; st <= S_LOOK; end
                    else st <= S_FIN;
                 end else begin
                    whi <= bk_rddata[hway*2+0];           // line1 chunk0
                    st  <= S_FIN;
                 end
              end else begin
                 vw <= vicm[base_idx(cur_line)];
                 vi <= way_idx(vicm[base_idx(cur_line)] ? 1 : 0, cur_line);
                 st <= S_WB;
              end
           end

           // ---- miss: evict victim (writeback if dirty) then fill ----
           S_WB: begin
              if (WRITABLE!=0 && WRTHRU==0 && valm[vflat] && dirm[vflat]) begin
                 wb_way <= vw?1'b1:1'b0; wb_idx <= vi; pc <= 0;
                 wb_laddr <= {vtag, vbase};
                 st <= S_WBR;
              end else st <= r_cbo_zero ? S_ZFILL : S_FILL;
           end
           S_WBR: st <= S_WBW;
           S_WBW: begin
              linebuf[(2*pc)  *BANKW +: BANKW] <= bk_rddata[wb_way*2+0];
              linebuf[(2*pc+1)*BANKW +: BANKW] <= bk_rddata[wb_way*2+1];
              if (pc == HALF-1) begin pc <= 0; st <= S_WBI; end
              else begin pc <= pc + 1'b1; st <= S_WBR; end
           end
           S_WBI: begin l2_req<=1; l2_we<=1; l2_addr<=wb_laddr; l2_wdata<=linebuf; st<=S_WBA; end
           S_WBA: if (l2_ack) st <= r_cbo_zero ? S_ZFILL : S_FILL;

           // cbo.zero miss: victim evicted -> install a fresh zero line (no L2 read) + mark dirty
           S_ZFILL: begin
              tagm[vflat] <= tag_of(cur_line);
              valm[vflat] <= 1'b1;
              vicm[base_idx(cur_line)] <= ~vicm[base_idx(cur_line)];
              linebuf <= {LINEB{1'b0}}; pc <= 0;
              st <= S_FILLI;
           end

           S_FILL: begin l2_req<=1; l2_we<=0; l2_addr<=cur_line[PAW-1:OFFB]; st<=S_FILLW; end
           S_FILLW: if (l2_ack) begin
              linebuf <= l2_rdata; pc <= 0;
              tagm[vflat] <= tag_of(cur_line);
              valm[vflat] <= 1'b1;
              dirm[vflat] <= 1'b0;
              vicm[base_idx(cur_line)] <= ~vicm[base_idx(cur_line)];
              st <= S_FILLI;
           end
           S_FILLI: begin                  // install pair pc (bank writes combinational)
              if (pc == HALF-1) begin
                 pc <= 0;
                 // cbo.zero: the line is now zero -> mark dirty and finish; else re-lookup the refill
                 if (r_cbo_zero) begin dirm[vflat] <= 1'b1; wr_ack <= 1; st <= S_IDLE; end
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
                 if (r_uncached) begin
                    valm[flat(w0_way,w0_idx)] <= 1'b0;
                    if (r_span) valm[flat(hway,cih)] <= 1'b0;
                 end
                 st <= S_IDLE;
              end else begin
                 // low-chunk (and same-line high) write driven combinationally this cycle.
                 if (r_span && store_hi) begin
                    // line1 hit way/idx are live (phase1); write its chunk0 next cycle
                    st <= S_SPANW;
                 end else if (WRTHRU!=0 || r_uncached) begin
                    // write-through, OR a Svpbmt NC/IO store -> push to L2 (DMA sees it) and
                    // invalidate the line at S_WTA so nothing dirty/stale lingers (flush-around).
                    wb_way <= w0_way; wb_idx <= w0_idx; pc <= 0; st <= S_WTR;
                 end else begin
                    dirm[flat(w0_way,w0_idx)] <= 1'b1;
                    if (r_span) dirm[flat(hway,cih)] <= 1'b1;
                    wr_ack <= 1; st <= S_IDLE;
                 end
              end
           end
           S_SPANW: begin                   // spanning store high half written combinationally
              if (WRTHRU!=0 || r_uncached) begin wb_way <= w0_way; wb_idx <= w0_idx; pc <= 0; st <= S_WTR; end
              else begin
                 dirm[flat(w0_way,w0_idx)] <= 1'b1;
                 dirm[flat(hway,cih)]      <= 1'b1;
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
                 dirm[flat(wb_way,wb_idx)] <= 1'b0;                // it is now clean in L2
                 if (!r_cbo_keep) valm[flat(wb_way,wb_idx)] <= 1'b0;  // flush/inval drop; clean keeps
              end else if (r_uncached) valm[flat(wb_way,wb_idx)] <= 1'b0;  // Svpbmt NC store: flush-around
              wr_ack <= 1; st <= S_IDLE;
           end

           // ---- flush (write-back configs) ----
           S_FLUSH: begin
              if (fscan == NW) begin inv_busy <= 0; st <= S_IDLE; end
              else if (WRITABLE!=0 && WRTHRU==0 && valm[fscan[FW-1:0]] && dirm[fscan[FW-1:0]]) begin
                 wb_way <= fscan[FW-1]; wb_idx <= fidx; pc <= 0; wb_laddr <= {ftag, fbase};
                 st <= S_FLUSHR;
              end else begin
                 if (!flush_clean) valm[fscan[FW-1:0]] <= 1'b0;   // clean flush keeps lines valid
                 dirm[fscan[FW-1:0]] <= 1'b0;
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
           S_FLUSHI: begin l2_req<=1; l2_we<=1; l2_addr<=wb_laddr; l2_wdata<=linebuf; st<=S_FLUSHA; end
           S_FLUSHA: if (l2_ack) begin
              if (!flush_clean) valm[fscan[FW-1:0]] <= 1'b0;       // clean flush: written back, stays valid+clean
              dirm[fscan[FW-1:0]] <= 1'b0;
              fscan <= fscan + 1'b1; st <= S_FLUSH;
           end

           S_INVDONE: begin inv_busy <= 1'b0; st <= S_IDLE; end
         endcase
      end
   end

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
