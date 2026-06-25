`default_nettype none

// Unified skewed-2-way PIPT L1 cache (SmooolRV64 memory subsystem).
//
// ONE module, two roles via parameters -- instantiated as the I$ (WRITABLE=0,
// fill-only) and the D$ (WRITABLE=1, write-back with dirty + writeback). The
// store/dirty/writeback hardware is gated by WRITABLE so the I$ instance prunes it.
//
// PIPT: the request address is PHYSICAL (the consumer translates first -- LSU
// dMMU / frontend iMMU). No synonyms, so a physical line lives in exactly one
// place per way; correctness is the full physical-tag compare. 2-way SKEW-
// associative: way1 XORs the low tag bits into the index to cut conflict misses.
// Since the skew folds only stored tag bits, the tag compare stays sufficient, and
// a victim's base index is recovered as (skewed_index ^ victim_tag[IDXB-1:0]).
//
// The consumer port is BYTE-ADDRESSED and returns RDW bits starting at the byte
// address (rd_data[b] = mem[rd_addr+b]) -- a drop-in for the LSU's mem_raddr/
// mem_rdata port and tb_vl's behavioral memory. LINE-CROSSING is handled
// INTERNALLY (an access spanning two lines reads/writes both), so "full misaligned
// performance" needs no consumer change.
//
// CORRECTNESS-FIRST: one in-flight request, serialized through an FSM (SmolRV64
// cache_state shape); arrays read combinationally. Pipelining / sync-BRAM timing
// is a later integration refinement. Write policy = WRITE-BACK (validates the
// harder path); a write-through mode is a later config.
module cache #(
   parameter PAW      = 34,
   parameter SIZE_KB  = 128,
   parameter WAYS     = 2,
   parameter LINEB    = 512,
   parameter RDW      = 64,
   parameter WDW      = 64,
   parameter OFFB     = 6,         // line offset bits (64 B line)
   parameter WRITABLE = 1,
   parameter WRTHRU   = 0          // 1 = write-through (write-allocate, never dirty -> L2 always
                                   //     current, so PTW reads flat memory coherently); 0 = write-back
) (
   input  wire             clk,
   input  wire             reset,

   // ---- consumer read port (byte-addressed; single outstanding) ----
   input  wire             rd_req,
   input  wire [PAW-1:0]   rd_addr,
   output reg  [RDW-1:0]   rd_data,
   output reg              rd_valid,

   // ---- consumer write port (WRITABLE only; byte-masked) ----
   input  wire             wr_req,
   input  wire [PAW-1:0]   wr_addr,
   input  wire [WDW-1:0]   wr_data,
   input  wire [WDW/8-1:0] wr_mask,
   output reg              wr_ack,

   // ---- invalidate / flush ----
   input  wire             inv_req,
   output reg              inv_busy,

   // ---- L2 / DRAM line bus (single transaction at a time) ----
   output reg              l2_req,
   output reg              l2_we,
   output reg  [PAW-OFFB-1:0] l2_addr,      // line address = PA[PAW-1:OFFB]
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

   reg [PTAGB-1:0] tagm [0:NW-1];
   reg             valm [0:NW-1];
   reg             dirm [0:NW-1];
   reg [LINEB-1:0] datm [0:NW-1];
   reg             vicm [0:SETS-1];
   integer i;
   initial begin
      for (i=0;i<NW;i=i+1)   begin valm[i]=1'b0; dirm[i]=1'b0; tagm[i]=0; datm[i]=0; end
      for (i=0;i<SETS;i=i+1) vicm[i]=1'b0;
   end

   function [IDXB-1:0]  base_idx; input [PAW-1:0] a; base_idx = a[OFFB +: IDXB];      endfunction
   function [PTAGB-1:0] tag_of;   input [PAW-1:0] a; tag_of   = a[OFFB+IDXB +: PTAGB]; endfunction
   function [IDXB-1:0]  way_idx; input integer w; input [PAW-1:0] a;
      reg [PTAGB-1:0] t;
      begin t = tag_of(a); way_idx = (w==0) ? base_idx(a) : (base_idx(a) ^ t[IDXB-1:0]); end
   endfunction
   function [FW-1:0] flat; input integer w; input [IDXB-1:0] ix;
      flat = (w!=0)*SETS + ix;
   endfunction

   reg            r_is_wr;
   reg [PAW-1:0]  r_addr;
   reg [WDW-1:0]  r_wdata;
   reg [WRB-1:0]  r_wmask;
   reg [OFFB-1:0] r_off;
   reg            r_span;
   wire [PAW-1:0] line0 = {r_addr[PAW-1:OFFB], {OFFB{1'b0}}};
   wire [PAW-1:0] line1 = line0 + (1<<OFFB);

   reg [LINEB-1:0] lw0, lw1;
   reg [IDXB-1:0]  wi0, wi1;
   reg             ww0, ww1;

   localparam S_IDLE=0, S_LOOK=1, S_CHECK=2, S_WB=3, S_WBW=4, S_FILL=5, S_FILLW=6,
              S_FIN=7, S_FLUSH=8, S_FLUSHW=9, S_WT0=10, S_WT0W=11, S_WT1=12, S_WT1W=13;
   reg [3:0]      st;
   reg            phase;
   reg [PAW-1:0]  cur_line;
   reg            vw;
   reg [IDXB-1:0] vi;
   reg [FW:0]     fscan;

   // combinational lookup of cur_line
   wire [IDXB-1:0]  ci0  = way_idx(0, cur_line);
   wire [IDXB-1:0]  ci1  = way_idx(1, cur_line);
   wire [PTAGB-1:0] ctag = tag_of(cur_line);
   wire hit0 = valm[flat(0,ci0)] & (tagm[flat(0,ci0)]==ctag);
   wire hit1 = valm[flat(1,ci1)] & (tagm[flat(1,ci1)]==ctag);

   // victim address reconstruction (un-skew via the victim's own tag)
   wire [FW-1:0]    vflat = flat(vw?1:0, vi);
   wire [PTAGB-1:0] vtag  = tagm[vflat];
   wire [IDXB-1:0]  vbase = vw ? (vi ^ vtag[IDXB-1:0]) : vi;

   // flush address reconstruction
   wire             fway  = fscan[IDXB];
   wire [IDXB-1:0]  fidx  = fscan[IDXB-1:0];
   wire [PTAGB-1:0] ftag  = tagm[fscan[FW-1:0]];
   wire [IDXB-1:0]  fbase = fway ? (fidx ^ ftag[IDXB-1:0]) : fidx;

   // read assembly: RDW bytes from {lw1,lw0} starting at r_off
   wire [2*LINEB-1:0] win_sh = {lw1, lw0} >> (r_off*8);

   integer b;
   always @(posedge clk) begin
      if (reset) begin
         st <= S_IDLE; rd_valid <= 0; wr_ack <= 0; inv_busy <= 0;
         l2_req <= 0; l2_we <= 0; phase <= 0; fscan <= 0;
      end else begin
         rd_valid <= 0; wr_ack <= 0; l2_req <= 0;
         case (st)
           S_IDLE: begin
              if (inv_req) begin
                 inv_busy <= 1; fscan <= 0; st <= S_FLUSH;
              end else if (rd_req || (wr_req && WRITABLE!=0)) begin
                 r_is_wr  <= wr_req && !rd_req;
                 r_addr   <= rd_req ? rd_addr : wr_addr;
                 r_wdata  <= wr_data; r_wmask <= wr_mask;
                 r_off    <= rd_req ? rd_addr[OFFB-1:0] : wr_addr[OFFB-1:0];
                 r_span   <= ({1'b0,(rd_req ? rd_addr[OFFB-1:0] : wr_addr[OFFB-1:0])}
                               + (rd_req ? RDB : WRB)) > WORDB;
                 phase    <= 0;
                 cur_line <= {(rd_req ? rd_addr[PAW-1:OFFB] : wr_addr[PAW-1:OFFB]), {OFFB{1'b0}}};
                 st <= S_LOOK;
              end
           end
           S_LOOK: st <= S_CHECK;
           S_CHECK: begin
              if (hit0 || hit1) begin
                 if (phase==0) begin
                    lw0 <= hit0 ? datm[flat(0,ci0)] : datm[flat(1,ci1)];
                    wi0 <= hit0 ? ci0 : ci1; ww0 <= ~hit0;
                 end else begin
                    lw1 <= hit0 ? datm[flat(0,ci0)] : datm[flat(1,ci1)];
                    wi1 <= hit0 ? ci0 : ci1; ww1 <= ~hit0;
                 end
                 if (phase==0 && r_span) begin
                    phase <= 1; cur_line <= line1; st <= S_LOOK;
                 end else st <= S_FIN;
              end else begin
                 vw <= vicm[base_idx(cur_line)];
                 vi <= way_idx(vicm[base_idx(cur_line)] ? 1 : 0, cur_line);
                 st <= S_WB;
              end
           end
           S_WB: begin
              if (WRITABLE!=0 && valm[vflat] && dirm[vflat]) begin
                 l2_req <= 1; l2_we <= 1;
                 l2_addr  <= {vtag, vbase};
                 l2_wdata <= datm[vflat];
                 st <= S_WBW;
              end else st <= S_FILL;
           end
           S_WBW: if (l2_ack) st <= S_FILL;
           S_FILL: begin
              l2_req  <= 1; l2_we <= 0;
              l2_addr <= cur_line[PAW-1:OFFB];
              st <= S_FILLW;
           end
           S_FILLW: if (l2_ack) begin
              tagm[vflat] <= tag_of(cur_line);
              valm[vflat] <= 1'b1;
              dirm[vflat] <= 1'b0;
              datm[vflat] <= l2_rdata;
              vicm[base_idx(cur_line)] <= ~vicm[base_idx(cur_line)];
              st <= S_LOOK;
           end
           S_FIN: begin
              if (!r_is_wr) begin
                 rd_data  <= win_sh[RDW-1:0];
                 rd_valid <= 1;
                 st <= S_IDLE;
              end else begin : do_write
                 reg [LINEB-1:0] n0, n1;
                 reg [OFFB:0] pos;
                 n0 = lw0; n1 = lw1;
                 for (b=0;b<WRB;b=b+1) if (r_wmask[b]) begin
                    pos = {1'b0,r_off} + b[OFFB:0];
                    if (pos < WORDB) n0[pos*8 +: 8]          = r_wdata[b*8 +: 8];
                    else             n1[(pos-WORDB)*8 +: 8]  = r_wdata[b*8 +: 8];
                 end
                 datm[flat(ww0?1:0,wi0)] <= n0;
                 if (r_span) datm[flat(ww1?1:0,wi1)] <= n1;
                 if (WRTHRU!=0) begin
                    lw0 <= n0; lw1 <= n1;            // carry the clean line(s) to the L2 write-through
                    st <= S_WT0;
                 end else begin
                    dirm[flat(ww0?1:0,wi0)] <= 1'b1;
                    if (r_span) dirm[flat(ww1?1:0,wi1)] <= 1'b1;
                    wr_ack <= 1; st <= S_IDLE;
                 end
              end
           end
           // write-through: push the just-written clean line(s) to L2 (full-line writes;
           // line stays clean so eviction never writes back, and L2 stays current for PTW).
           S_WT0:  begin l2_req<=1; l2_we<=1; l2_addr<=line0[PAW-1:OFFB]; l2_wdata<=lw0; st<=S_WT0W; end
           S_WT0W: if (l2_ack) begin if (r_span) st<=S_WT1; else begin wr_ack<=1; st<=S_IDLE; end end
           S_WT1:  begin l2_req<=1; l2_we<=1; l2_addr<=line1[PAW-1:OFFB]; l2_wdata<=lw1; st<=S_WT1W; end
           S_WT1W: if (l2_ack) begin wr_ack<=1; st<=S_IDLE; end
           S_FLUSH: begin
              if (fscan == NW) begin
                 inv_busy <= 0; st <= S_IDLE;
              end else if (WRITABLE!=0 && valm[fscan[FW-1:0]] && dirm[fscan[FW-1:0]]) begin
                 l2_req   <= 1; l2_we <= 1;
                 l2_addr  <= {ftag, fbase};
                 l2_wdata <= datm[fscan[FW-1:0]];
                 st <= S_FLUSHW;
              end else begin
                 valm[fscan[FW-1:0]] <= 1'b0; dirm[fscan[FW-1:0]] <= 1'b0;
                 fscan <= fscan + 1'b1;
              end
           end
           S_FLUSHW: if (l2_ack) begin
              valm[fscan[FW-1:0]] <= 1'b0; dirm[fscan[FW-1:0]] <= 1'b0;
              fscan <= fscan + 1'b1; st <= S_FLUSH;
           end
         endcase
      end
   end
endmodule

`default_nettype wire
