`default_nettype none

// The branch predictor, as the fetch stream: it holds the stream's address, looks up the BTB, the
// YAGS corrector and the RAS once per 16-byte window the fetch ring (smolrv64_fring) asks the I$
// for, steers the stream to the next window, and queues the predictions it made for the aligner to
// take back in order (docs/PLAN-2026-09-24-frontend-stage4.md, increment 1b;
// docs/PLAN-2026-10-27-fetch-window.md).
//
// KEYED BY THE LAST HALFWORD. A BTB entry belongs to its CTI's LAST halfword, and a window covers
// eight consecutive halfwords from any halfword, so the BTB and the corrector are each eight banks,
// one per halfword position in 16 bytes (address bits [3:1]), and a window reads one row of each:
// bank b's halfword is in the window's first pair when b is at or after the window's offset, else in
// the pair after it, and each bank reads its own pair's row. Training keys the same way, by
// the resolving CTI's PC and length, so nothing about where a lookup happened rides with the
// instruction, and no two CTIs share an entry. (A coarser key does not survive real code: an 8-byte
// key put 28.7% of the kernel's CTIs beside another, and a 4-byte key still put every
// `jal f; c.bnez a0, loop` in one entry, which the two evicted from each other on every iteration.)
//
// THE STREAM is a registered window: its first pair and its offset in it (sa_q, sk_q). When the ring
// takes the window (adv) or the stream restarts (flush), both load the next address and the banks
// read its rows, so a window's prediction starts from registers and the row address is the next
// address alone.
//
// A WINDOW ENDS AT ITS FIRST PREDICTED-TAKEN TRANSFER. Every known CTI from the stream's address on
// is marked, up to the first one predicted taken, or to the M-th: p_end is that halfword's position
// in the window, and the stream continues at the predicted target, at the halfword after the M-th,
// or 16 bytes on. A not-taken mark does not end the window, so the aligner reads on past it. The
// history steps at most once per window, at its end, so every mark of a window has the same
// pre-state and its corrector reads share the history they were read with. The ring marks each
// marked halfword (p_mk, by bank), and each mark's prediction goes into the queue, in window order.
//
// THE QUEUE. One entry per marked halfword, in order: taken, the target, the corrector details
// training needs, and the RAS pointer and history as they stand BEFORE the CTI. Between two marks
// the stream's state does not change, so an entry's pre-state is the state of every instruction
// from the mark before it up to its own (the stream's own registers past the last entry): it is
// such an instruction's snapshot -- what a redirect by it restores -- and the head's is what a
// restart of the stream (a rejected prediction, a freeze) goes back to. The aligner reads the
// first IW entries, so a bundle holding several marks gives each slot its own entry's details,
// and pops as many as it consumed.
//
// THE HISTORY is the path of taken control transfers (smolrv64_phist.vh), in program order, as
// predicted and then corrected: the stream steps it once per pair that ends at a predicted-taken
// transfer, and a redirect restores the redirecting instruction's own snapshot, stepped by its own
// address when it was taken. It is never rebuilt from resolves: control flow resolves out of
// order, wrong-path branches included.
//
// Everything here is a HINT, never architectural: the mispredict check is the exec-side
// `actual_npc != pred_npc` compare, and the aligner takes a prediction only on a bundle that ends
// on a real branch or jump at the marked halfword (smolrv64_frontend, fetch.v). A stale entry, an
// aliased tag or a mis-restored RAS costs a redirect, never a wrong result.
module smolrv64_predictor
  #(parameter PCW   = 64,
    parameter BTBB  = 13,            // log2 L2 BTB entries, all banks and ways
    parameter L1RB  = 6,             // log2 L1 BTB rows a bank and way
    parameter BTBW  = 1,             // log2 BTB ways (skewed), at least 1
    parameter TAGW  = 12,
    parameter TGTW  = 38,            // stored target bits [38:1] (canonical VA, sign-extended)
    parameter GHL   = 11,            // global history length
    parameter RASB  = 3,             // log2 RAS entries
    parameter YBITS = 11,            // log2 corrector entries, all banks
    parameter YTAGW = 8,
    parameter PQB   = 3,             // log2 prediction-queue entries
    parameter IW    = 3,             // entries the aligner reads, and pops, per cycle
    // Carried predict details {rsp, ghr, yhit, yctr, yidx, way, hit, ctr}. An index or a PC-only tag
    // is recomputed at resolve from the CTI's PC and length; yidx rides because it folds the
    // predict-time history.
    parameter PDW   = RASB + GHL + (1+2+YBITS) + BTBW + (1+2))
   (input  wire                 clk,
    input  wire                 reset,
    // ---- the stream ----
    input  wire                 flush,      // the stream restarts at rst_a...
    input  wire [PCW-1:0]       rst_a,
    input  wire                 rst_np,     // ...predicting nothing in its first pair
    input  wire                 adv,        // the ring took this cycle's pair
    output wire [PCW-1:0]       sa,         // this cycle's window: its first pair's first byte
    output wire [2:0]           sk,         // ...and the window's first halfword in that pair
    output wire                 p_cut,      // the window ends at a mark...
    output wire [2:0]           p_end,      // ...at this position of it (its last when it does not)
    output wire [7:0]           p_mk,       // the window's marked halfwords, by bank
    output wire                 pq_room,
    output wire                 cancel,     // a correction: the ring drops its youngest requests...
    output wire [1:0]           cancel_n,   // ...this many
    output wire [PQB:0]         pq_cnt,
    // ---- the aligner: the first IW entries, entry j at [j] ----
    output wire [IW-1:0]        pq_tk,      // entry j: predicted taken...
    output wire [IW*PCW-1:0]    pq_tgt,     // ...to here
    output wire [IW*PDW-1:0]    pd_mk,      // details of an instruction ending at entry j's mark
    output wire [IW*PDW-1:0]    pd_no,      // ...and of any other before it (after entry j-1's)
    input  wire [$clog2(IW+1)-1:0] pop,     // entries the aligner consumed, or dropped
    // ---- redirect ----
    input  wire                 rollback,   // a redirect is entering the frontend this cycle
    input  wire [RASB-1:0]      rb_rsp,     // ...and the RAS pointer...
    input  wire [GHL-1:0]       rb_ghr,     // ...and the history it restores
    // ---- resolve/training port (oldest resolved CTI this cycle) ----
    input  wire                 res_v,
    input  wire                 res_cbr,    // conditional branch (vs jump/JALR)
    input  wire                 res_call,   // jump with a link dest (rd in {x1,x5})
    input  wire                 res_ret,    // JALR return (rs1 link, rd not)
    input  wire                 res_taken,
    input  wire [PDW-1:0]       res_pdet,   // the resolving instruction's own details
    input  wire [PCW-1:0]       res_tgt,    // taken-target (train the BTB)
    input  wire [PCW-1:0]       res_last,   // the resolving CTI's last halfword's address
    output wire [3:0]           err);       // its invariants, registered (the integrity log)

   localparam W    = 1 << BTBW;             // BTB ways
   initial if (PDW != RASB + GHL + (1+2+YBITS) + BTBW + (1+2))
      $fatal(1, "smolrv64_predictor: the details are %0d bits; {rsp, ghr, yhit, yctr, yidx, way, hit, ctr} is %0d",
             PDW, RASB + GHL + (1+2+YBITS) + BTBW + (1+2));
   localparam RB   = BTBB - 3 - BTBW;       // row bits per L2 bank and way
   initial if (L1RB >= RB) $fatal(1, "smolrv64_predictor: L1 has %0d row bits, L2 %0d", L1RB, RB);
   localparam YB   = YBITS - 3;             // row bits per corrector bank
   localparam NB   = 1 << RB;
   localparam NY   = 1 << YB;
   localparam RASN = 1 << RASB;
   localparam PQD  = 1 << PQB;
   // BTB type: 0xx = cond branch, xx = 2-bit bimodal (00 S_N .. 11 S_T); 1xx = uncond
   localparam [2:0] TY_JMP = 3'b100, TY_CALL = 3'b101, TY_RET = 3'b110;

   // ------------------------------------------------------------ BTB: eight banks of W skewed ways
   // entry = {tag, type[2:0], target[38:1]}, and NO valid bit: the tag carries it. A never-written
   // entry reads as all zeros, believed only by a halfword whose tag is also 0, and it then says "a
   // strongly-not-taken conditional ends here": a mark the aligner drops when no branch ends there.
   // Configuration INIT zeroes every bank. Bank b holds the halfwords at address bits [3:1] == b, so
   // a pair's halfword k is in bank k; each bank is W ways of 1R1W RAM.
   //
   // SKEWED: way w reads row lo ^ rotl(hi, w), lo the RB address bits above the pair and hi the RB
   // bits above those, so halfwords that share a row in one way rarely share it in another, and
   // W ways do the work of 2W set-associative ones. A CTI's W candidate slots are one per way.
   //
   // TAKEN ONLY: every jump is allocated, a conditional only when it resolves taken. A
   // conditional the BTB misses is one never taken (or evicted), and falls through.
   localparam EW = TAGW + 3 + TGTW;
`include "smolrv64_phist.vh"
`include "smolrv64_pc.vh"

   // The tag folds higher bits in, so halfwords that agree in the index but differ above still
   // miss: the kernel's pre-MMU execution at PA 0x802xxxxx aliases its VAs 0xffffffff80xxxxxx in
   // every index bit, and without the fold an early-boot entry steers the paged kernel to a bare
   // address. Its low RB bits carry hi, which the skewed row consumed (rule B9).
   function [TAGW-1:0] btag(input [PCW-1:0] a);
      btag = a[RB+TAGW+3:RB+4] ^ a[RB+2*TAGW+3:RB+TAGW+4] ^ {{(TAGW-1){1'b0}}, a[PCW-1]};
   endfunction
   function [RB-1:0] bslot(input [PCW-1:0] a, input integer w);
      reg [2*RB-1:0] hh;
      begin
         hh    = {a[2*RB+3:RB+4], a[2*RB+3:RB+4]};
         bslot = a[RB+3:4] ^ hh[2*RB-1-w -: RB];
      end
   endfunction

   // ------------------------------------------------------------ YAGS corrector: eight banks
   // A tagged direction corrector indexed by halfword ^ history, consulted for a known
   // conditional. On a tag hit it overrides the BTB's bimodal weight. It holds only bimodal
   // exceptions: it is allocated when a BTB-known conditional's bimodal guess was wrong, and
   // refined on a hit. The bank is the halfword's position, carried as yidx's low three bits.
   localparam YEW = YTAGW + 2;
   function [YB-1:0] yfold(input [GHL-1:0] x);
      yfold = x[YB-1:0] ^ {{(2*YB-GHL){1'b0}}, x[GHL-1:YB]};
   endfunction
`include "smolrv64_ytag.vh"

   // ------------------------------------------------------------ speculative state
   reg [GHL-1:0]  ghr;                       // the stream's history
   reg [PCW-1:0]  ras [0:RASN-1];
   reg [RASB-1:0] ras_ptr;                   // the stream's RAS top
   reg [PCW-1:0]  sa_q;                      // the stream: its window's first pair...
   reg [2:0]      sk_q;                      // ...the window's first halfword in it...
   reg            np_q;                      // ...and predict nothing in it
   integer ii;
   initial begin
      ghr = 0; ras_ptr = 0; sa_q = 0; sk_q = 0; np_q = 1'b0;
      for (ii = 0; ii < RASN; ii = ii + 1) ras[ii] = {PCW{1'b0}};
   end
   assign sa = sa_q;
   assign sk = sk_q;

   // ------------------------------------------------------------ the queue
   // {tk, target, rsp_pre, ghr_pre, yhit, yctr, yidx, way, ctr}
   localparam QEW = 1 + PCW + RASB + GHL + 1 + 2 + YBITS + BTBW + 2;
   // TWO BANKS ON THE POSITION'S PARITY: a pair pushes up to M = 2 entries a cycle, to consecutive
   // positions and so to different banks, and each bank has one muxed write (one write statement
   // per LUTRAM bank: a second write site turns the array into flops).
   reg [QEW-1:0]  pq0 [0:PQD/2-1];
   reg [QEW-1:0]  pq1 [0:PQD/2-1];
   reg [PQB-1:0]  pq_rp, pq_wp;
   reg [PQB:0]    pq_n;
   initial begin pq_rp = 0; pq_wp = 0; pq_n = 0; end
   // entry j's pre-state, or the stream's past the last entry
   wire [RASB-1:0] en_rsp [0:IW-1];
   wire [GHL-1:0]  en_ghr [0:IW-1];
   genvar gj;
   generate for (gj = 0; gj < IW; gj = gj + 1) begin : ent
      wire [PQB-1:0] ps = pq_rp + gj[PQB-1:0];
      wire [QEW-1:0] q  = ps[0] ? pq1[ps[PQB-1:1]] : pq0[ps[PQB-1:1]];
      wire           in = pq_n > gj;
      assign en_rsp[gj] = in ? q[QEW-1-1-PCW -: RASB] : ras_ptr;
      assign en_ghr[gj] = in ? q[QEW-1-1-PCW-RASB -: GHL] : ghr;
      assign pq_tk[gj]               = q[QEW-1];
      assign pq_tgt[gj*PCW +: PCW]   = q[QEW-2 -: PCW];
      assign pd_mk[gj*PDW +: PDW]    = {en_rsp[gj], en_ghr[gj], q[2+BTBW+YBITS+2 -: 1+2+YBITS], q[2 +: BTBW], 1'b1, q[1:0]};
      assign pd_no[gj*PDW +: PDW]    = {en_rsp[gj], en_ghr[gj], 1'b0, 2'b01, {YBITS{1'b0}}, {BTBW{1'b0}}, 1'b0, 2'b01};
   end endgenerate
   // the state as of the aligner: the head's pre-state, or the stream's when nothing is queued
   wire [RASB-1:0] sn_rsp  = en_rsp[0];
   wire [GHL-1:0]  sn_ghr  = en_ghr[0];
   assign pq_cnt  = pq_n;
   wire [PQB:0]    popw    = {{(PQB+1-$clog2(IW+1)){1'b0}}, pop};   // the pop count, at the queue's width
   assign pq_room = pq_n <= PQD - M;                                // room for a pair's marks

   // A restart puts the stream back to the aligner's state, or to the redirect's; every name here
   // is a register or the queue head, so nothing of the aligner reaches the tables' read address.
   wire [GHL-1:0]  rs_ghr  = rollback ? rb_ghr : sn_ghr;
   wire [RASB-1:0] rs_rsp  = rollback ? rb_rsp : sn_rsp;
   reg  [YB-1:0]   yh_q, yr_q;
   initial begin yh_q = {YB{1'b0}};  yr_q = {YB{1'b0}}; end

   // ------------------------------------------------------------ this window's prediction
   // bq[k]/yq[k] are bank k's rows read for this window: its halfword at address bits [3:1] == k, in
   // the first pair (sa_q) when k is at or after the offset sk_q, else in the pair after it (sa16).
   // Every position evaluates its own hit, corrector hit and direction from registers at once; the
   // marks are the hits in WINDOW ORDER -- the banks from sk_q round to sk_q - 1 -- up to the first
   // taken one, or the M-th.
   localparam integer M = 2;                 // marks per window (the queue's pushes per cycle)
   wire [EW-1:0]   bq [0:7];                 // halfword k's entry (its first matching way)...
   wire [7:0]      bh;                       // ...whether any way matched
   wire [BTBW-1:0] bw [0:7];                 // ...and which (training's, carried in the details)
   wire [YEW-1:0]  yq [0:7];
   wire [7:0]      hy, inw, mkv, coh;        // per bank: corrector hit, in the window, marked, ends it
   wire [3:0]      nmk;                      // the marks
   wire [2:0]      hk, pe, ety;              // the bank where a mark ends it, its last position, the type
   wire            cut, tk, call, ret, w_one;
   wire [PCW-1:0]  ftc, p_nx;
   wire [RASB-1:0] rsp_post;
   wire [GHL-1:0]  ghr_post;
   wire [QEW-1:0]  pw_e0, pw_e1;
   wire [8*EW-1:0]   bq_f;
   wire [8*BTBW-1:0] bw_f;
   wire [8*YEW-1:0]  yq_f;
   genvar gq;
   generate for (gq = 0; gq < 8; gq = gq + 1) begin : bqf
      assign bq_f[gq*EW +: EW] = bq[gq];  assign bw_f[gq*BTBW +: BTBW] = bw[gq];  assign yq_f[gq*YEW +: YEW] = yq[gq];
   end endgenerate
   smolrv64_wpick #(.PCW(PCW), .TAGW(TAGW), .TGTW(TGTW), .BTBW(BTBW), .GHL(GHL), .RASB(RASB), .YBITS(YBITS),
                    .YTAGW(YTAGW)) u_pick
     (.sa(sa_q), .sk(sk_q), .np(np_q), .bq_f(bq_f), .bh(bh), .bw_f(bw_f), .yq_f(yq_f),
      .ras_top(ras[ras_ptr]), .ras_ptr(ras_ptr), .ghr(ghr), .yix(yr_q ^ yh_q),
      .hy(hy), .inw(inw), .mkv(mkv), .coh(coh), .nmk(nmk), .hk(hk), .pe(pe), .ety(ety), .cut(cut), .tk(tk),
      .call(call), .ret(ret), .w_one(w_one), .ftc(ftc), .p_nx(p_nx), .rsp_post(rsp_post),
      .ghr_post(ghr_post), .pw_e0(pw_e0), .pw_e1(pw_e1));
   assign p_cut = cut;
   assign p_end = pe;
   assign p_mk  = mkv;
   // the L2 tag of both pairs: the bits above the block's (the second pair's is wrong across a block,
   // and then that pair misses in both levels -- a lost hint, never a wrong one)
   wire [TAGW-1:0] stag = btag(sa_q);
   function [7:0] fpm8(input [2:0] off);                           // the banks at or after off
      fpm8 = ~((8'd1 << off) - 8'd1);
   endfunction
   function [PCW-1:0] tadr(input [EW-1:0] x);                      // an entry's target
      tadr = {{(PCW-TGTW-1){x[TGTW-1]}}, x[TGTW-1:0], 1'b0};
   endfunction
   // each mark's entry: the ending one's prediction, a not-taken one's details, the pair's pre-state
   wire            push     = adv & (|mkv) & ~flush;
   wire [PQB-1:0]  pw_p1 = pq_wp + 1'b1;
   // bank b takes the entry whose position has parity b: entry 0 at pq_wp, entry 1 at pq_wp + 1.
   // BOTH ARE WRITTEN WHENEVER THE QUEUE HAS ROOM, and push only moves the pointers: the two
   // positions past the tail hold nothing until the pointers take them, and a push needs room
   // (fetch advances only with pq_room), so the write enables are pq_n's alone -- not the marks,
   // the iMMU and the I$ handshake that decide the push.
   wire            b0_we = pq_room;
   wire            b1_we = pq_room;
   wire [PQB-2:0]  b0_ix = pq_wp[0] ? pw_p1[PQB-1:1] : pq_wp[PQB-1:1];
   wire [PQB-2:0]  b1_ix = pq_wp[0] ? pq_wp[PQB-1:1] : pw_p1[PQB-1:1];
   wire [QEW-1:0]  b0_d  = pq_wp[0] ? pw_e1 : pw_e0;
   wire [QEW-1:0]  b1_d  = pq_wp[0] ? pw_e0 : pw_e1;
   always @(posedge clk) if (!reset && push && !pq_room)
      $fatal(1, "smolrv64_predictor: a push of %0d marks into a queue holding %0d of %0d", nmk, pq_n, PQD);
   always @(posedge clk) if (b0_we) pq0[b0_ix] <= b0_d;
   always @(posedge clk) if (b1_we) pq1[b1_ix] <= b1_d;

   // ------------------------------------------------------------------ training
   // One BTB write per resolved CTI (oldest per cycle), into the bank of its last halfword.
   // Bimodal: nudge the carried counter toward the outcome; uncond: record the class (call/return
   // classified at resolve from the executed instruction); target: the resolved taken-target.
   wire [PCW-1:0]  t_last  = res_last;
   wire            t_hit   = res_pdet[2];
   wire [1:0]      t_ctr   = res_pdet[1:0];
   // THE HIT COMES WITH THE CTI, so training never reads the entries (they are a block RAM with
   // the stream's read port and training's write port): a CTI ending at a mark hit its pair's
   // rows when it was predicted, in the way its details carry (t_way). A hit rewrites its way; a
   // miss allocates the first way not recently trained (each slot's nu bit, set when trained;
   // when that would set all W, the others clear), or the rotating way when none is. An instance
   // predicted before an earlier one's allocation landed allocates again: a branch's first miss
   // mispredicts and drops everything younger, so few do (the bench's BTB-SIM counts them: 11 to
   // 184 a 9 M trainings on the GB5 slices), and a remembered allocation would go stale and evict
   // live entries instead.
   wire [BTBW-1:0] t_way   = res_pdet[3 +: BTBW];
   wire [RB-1:0]   tsl     [0:W-1];          // way w's slot for the CTI
   wire [1:0]      tr_bc   [0:W-1];          // ...its bimodal counter, now
   wire [W-1:0]    tr_nu;                    // ...its recently-trained bit, now
   genvar gt;
   generate for (gt = 0; gt < W; gt = gt + 1) begin : tw
      assign tsl[gt] = bslot(t_last, gt);
   end endgenerate
   wire [W-1:0]    tm      = t_hit ? ({{(W-1){1'b0}}, 1'b1} << t_way) : {W{1'b0}};
   reg  [BTBW-1:0] vrot;                     // the way a miss takes when every slot is recent
   initial vrot = {BTBW{1'b0}};
   wire            tr_hit  = |tm;
   wire [W-1:0]    tm1     = tm & (~tm + 1'b1);                     // the first matching way
   wire [W-1:0]    tz      = ~tr_nu;
   wire [W-1:0]    tz1     = tz & (~tz + 1'b1);                     // the first not recent
   wire [W-1:0]    tsel    = tr_hit ? tm1 : (|tz) ? tz1 : ({{(W-1){1'b0}}, 1'b1} << vrot);
   wire            tw_v    = res_v & (tr_hit | res_taken);          // rewrite a hit; allocate taken
   wire [W-1:0]    nu_set  = tr_nu | tsel;
   wire [W-1:0]    nu_d    = (&nu_set) ? tsel : nu_set;
   reg  [1:0]      bc_tr;                    // the bimodal counter in the hit way, now
   integer         tk_i;
   always @* begin
      bc_tr = 2'b00;
      for (tk_i = 0; tk_i < W; tk_i = tk_i + 1) if (tm1[tk_i]) bc_tr = tr_bc[tk_i];
   end
   // READ-MODIFY-WRITE: the counters a training nudges are the tables' CURRENT ones (bc_tr, yc_tr,
   // read at the CTI's own slot), never the snapshot it carried from fetch: training waits for the
   // CTI to be non-speculative, and every instance of a loop's branch in flight carries the same
   // stale snapshot, so blind writes of it lose the updates in between.
   wire [YEW-1:0]  yc_tr;                    // the corrector entry at its index, now
   wire [1:0]      t_base  = tr_hit ? bc_tr : (res_taken ? 2'b10 : 2'b01);  // miss -> install weak
   wire [1:0]      t_nudge = res_taken ? ((t_base == 2'b11) ? 2'b11 : t_base + 1'b1)
                                       : ((t_base == 2'b00) ? 2'b00 : t_base - 1'b1);
   wire [2:0]      t_type  = res_cbr  ? {1'b0, t_nudge}
                           : res_ret  ? TY_RET
                           : res_call ? TY_CALL : TY_JMP;
   wire [EW-1:0]   t_ent   = {btag(t_last), t_type, res_tgt[TGTW:1]};
   // The corrector: only a CTI the BTB knew carries a corrector index, so it is allocated when
   // such a conditional's bimodal guess was wrong, and refined whenever it was consulted.
   wire            yc_hit  = res_pdet[3+BTBW+YBITS+2];
   wire [1:0]      yc_ctr  = res_pdet[3+BTBW+YBITS+1 -: 2];
   wire [YBITS-1:0] yc_idx = res_pdet[3+BTBW +: YBITS];
   wire            yc_cur  = yc_hit & (yc_tr[YEW-1 -: YTAGW] == ytagf(t_last));   // still this CTI's
   wire [1:0]      y_base  = yc_cur ? yc_tr[1:0] : (res_taken ? 2'b10 : 2'b01);
   wire [1:0]      y_nudge = res_taken ? ((y_base == 2'b11) ? 2'b11 : y_base + 1'b1)
                                       : ((y_base == 2'b00) ? 2'b00 : y_base - 1'b1);
   wire            bim_pred = t_ctr[1];         // the bimodal guess fetch made
   wire            y_wr    = res_v & res_cbr & t_hit & (yc_hit | (bim_pred != res_taken));
   wire [YEW-1:0]  y_ent   = {ytagf(t_last), y_nudge};

   // ------------------------------------------------------------ the stream's next window
   // The stream moves when the ring takes its window (adv) or it restarts: a flush, to rst_a with
   // the aligner's or the redirect's state, or a correction (cr_q, below), back to a window's own
   // start with its state before it. The ring takes no window in a correction's cycle.
   wire            rs    = flush | cr_q;
   wire            mv    = rs | adv;
   wire [PCW-1:0]  rsa   = flush ? rst_a  : cr_a;
   wire [GHL-1:0]  rsg   = flush ? rs_ghr : cr_ghr;
   wire [RASB-1:0] rsr   = flush ? rs_rsp : cr_rsp;
   wire            rsn   = flush ? rst_np : cr_np;
   wire [PCW-1:0]  na    = rs ? rsa : p_nx;
   // The corrector's rows are read with the history before this window's own shift -- a window of
   // lag -- so this window's direction never reaches the next window's read index; the folded
   // history is registered (yh_q) and carried in the index training uses. They are read AHEAD, at
   // this window's address (the restart's on a restart), the row registered (yr_q) and carried too.
   wire [YB-1:0]   yh    = yfold(rs ? rsg : ghr);
   wire [YB-1:0]   yr    = rs ? rsa[YB+3:4] : sa_q[YB+3:4];

   // ------------------------------------------------------------ L1: the BTB in the loop
   // Eight halfword banks of W skewed ways, L1N rows each, in distributed RAM, read at the START of
   // the cycle at rows of the stream's registers: bank b at its pair's, the first pair's when b is
   // at or after the offset, else the pair after's. So the loop is the stream's registers -> the
   // rows -> the read -> the hits and the end pick -> the next address. An L1 entry mirrors an L2
   // entry in the SAME way: its tag is the L2 index bits above L1's and L2's tag, its type carries
   // the bimodal counter, and the way the details carry is both levels'.
   //
   // THE PAIR AFTER'S SLOT IS THE FIRST PAIR'S PLUS ONE in L1's row bits alone, with the first
   // pair's skew and tag (lslot1): no carry chain in front of the read. It is the pair's own slot
   // except where those bits wrap, and the fill writes a pair after's entries where this reads them,
   // so a window across the wrap finds them too; training writes each CTI's own slot.
   localparam L1N  = 1 << L1RB;
   localparam L1TW = RB - L1RB + TAGW;
   localparam L1EW = L1TW + 3 + TGTW;
   function [L1RB-1:0] lslot(input [PCW-1:0] a, input [BTBW-1:0] w);
      reg [2*L1RB-1:0] hh;
      begin
         hh    = {a[2*L1RB+3:L1RB+4], a[2*L1RB+3:L1RB+4]};
         lslot = a[L1RB+3:4] ^ hh[2*L1RB-1-w -: L1RB];
      end
   endfunction
   function [L1RB-1:0] lslot1(input [PCW-1:0] a, input [BTBW-1:0] w);
      reg [2*L1RB-1:0] hh;
      begin
         hh     = {a[2*L1RB+3:L1RB+4], a[2*L1RB+3:L1RB+4]};
         lslot1 = (a[L1RB+3:4] + 1'b1) ^ hh[2*L1RB-1-w -: L1RB];
      end
   endfunction
   function [L1TW-1:0] ltag(input [PCW-1:0] a, input [TAGW-1:0] t);   // t: the L2 tag
      ltag = {a[RB+3:L1RB+4], t};
   endfunction
   // the pair after's: the bits both levels' rows read, [2RB+3:4], incremented; sa_q's above them
   wire [PCW-1:0]  s16   = {sa_q[PCW-1:2*RB+4], sa_q[2*RB+3:4] + 1'b1, 4'b0000};

   // ------------------------------------------------------------ L2: the check, a cycle behind
   // The window the ring takes is registered (s2_*) with L1's view of it: its last position, where
   // the stream went on, its marks and L1's hits. L2's block RAMs read the same rows in the same
   // cycle, so the next cycle the same end pick (u_chk) gives L2's view. When the two differ in where
   // the window ends or where the stream goes on, the cycle after corrects (cr_q): L1 is filled from
   // L2 at that window, the stream goes back to the window's start with its state before it, the
   // queue gives back what that window and the next pushed, and the ring drops their requests
   // (cr_k). A window differing only in its marks fills L1 and is left alone: the not-taken
   // conditional L1 lacked goes unpredicted once, as a BTB miss does.
   reg             s2_v, s2_np;
   reg  [PCW-1:0]  s2_sa, s2_rtop, s2_nx;
   reg  [2:0]      s2_sk, s2_pe;
   reg  [8*YEW-1:0] s2_yq;
   reg  [RASB-1:0] s2_rsp;
   reg  [GHL-1:0]  s2_ghr;
   reg  [YB-1:0]   s2_yix;
   reg  [7:0]      s2_mk;
   reg  [8*W-1:0]  s2_m1;                    // ...and L1's matching ways, by bank
   reg  [PQB-1:0]  s2_wp;                    // the queue's tail before the window
   reg  [2:0]      s2_n;                     // ...and the entries it pushed
   initial s2_v = 1'b0;
   wire [TAGW-1:0] s2_tag = btag(s2_sa);
   wire [EW-1:0]   bq2 [0:7];                // L2: each bank's entry (its first matching way)...
   wire [7:0]      bh2;                      // ...whether any way matched
   wire [BTBW-1:0] bw2 [0:7];                // ...and which
   wire [7:0]      mk2, inw2;                // ...and its view: marks, the banks in the window,
   wire [2:0]      pe2;                      // ...the last position,
   wire [PCW-1:0]  nx2;                      // ...where the stream goes on
   wire            d_v, d_bad, d_fil;        // the check: it counts, it differs, L1 takes a fill
   // the correction, and the fill it writes into L1: per bank, the way, the row and the entry
   reg             cr_q, cr_np;
   reg             mf_q;                     // a window that differed only in its marks: fill L1
   reg  [PCW-1:0]  cr_a;
   reg  [GHL-1:0]  cr_ghr;
   reg  [RASB-1:0] cr_rsp;
   reg  [PQB-1:0]  cr_wp;
   reg  [2:0]      cr_n;                     // the queue entries to give back
   reg  [1:0]      cr_k;                     // the ring's requests to drop
   reg  [8*W-1:0]  fl_w, fl_c;               // by bank and way: write, and write zeros (clear)
   reg  [8*W*L1RB-1:0] fl_r;                 // ...at this row
   reg  [8*L1EW-1:0] fl_e;                   // by bank: L2's entry
   initial begin cr_q = 1'b0;  mf_q = 1'b0;  fl_w = {8*W{1'b0}}; end
   wire [8*W-1:0]  m1_f;                     // L1's matching ways now, by bank
   assign cancel   = cr_q;
   assign cancel_n = cr_k;

   wire [7:0]      fpk   = fpm8(sk_q);       // the banks of the window's first pair
   wire [7:0]      fpk2  = fpm8(s2_sk);      // ...and of the window the check holds
   genvar gb, gw;
   // every bank and way's slot for the training CTI, {bank, way}; the CTI's bank is picked below
   wire [1:0]      bc_rd [0:8*W-1];
   wire            nu_rd [0:8*W-1];
   wire [YEW-1:0]  yc_rd [0:7];
   generate for (gb = 0; gb < 8; gb = gb + 1) begin : bank
      localparam [2:0] BI = gb;
      wire            fp   = fpk[gb];                       // the halfword is in the first pair
      wire [PCW-1:0]  pa   = fp ? sa_q : s16;               // ...its pair (L2's rows)
      wire [L1TW-1:0] lt   = ltag(sa_q, stag);              // L1's tag, the first pair's for both
      wire            tw_b = tw_v & (t_last[3:1] == BI);
      wire [L1EW-1:0] e1   [0:W-1];                         // L1: each way's entry for the halfword
      wire [W-1:0]    m1;                                   // ...whether it is the halfword's
      wire [EW-1:0]   wq2  [0:W-1];                         // L2: each way's, a cycle later
      wire [W-1:0]    m2;
      for (gw = 0; gw < W; gw = gw + 1) begin : way
         localparam [BTBW-1:0] WI = gw;
         // L1: one write a cycle, training's (the CTI's slot in the way L2 trains), else a fill's
         (* ram_style = "distributed" *) reg [L1EW-1:0] l1 [0:L1N-1];
         integer li;
         initial for (li = 0; li < L1N; li = li + 1) l1[li] = {L1EW{1'b0}};
         wire   l1t = tw_b & tsel[gw];
         wire   l1f = (cr_q | mf_q) & fl_w[gb*W + gw];
         localparam integer FI = gb*W + gw;
         wire [L1RB-1:0] lr  = fp ? lslot(sa_q, WI) : lslot1(sa_q, WI);   // the read's row
         wire [L1RB-1:0] lw  = l1t ? lslot(t_last, WI) : fl_r[FI*L1RB +: L1RB];
         wire [L1EW-1:0] lwd = l1t ? {ltag(t_last, btag(t_last)), t_type, res_tgt[TGTW:1]}
                             : fl_c[FI] ? {~fl_e[gb*L1EW + L1EW-1 -: L1TW], {(L1EW-L1TW){1'b0}}}
                             : fl_e[gb*L1EW +: L1EW];
         assign e1[gw] = l1[lr];
         assign m1[gw] = e1[gw][L1EW-1 -: L1TW] == lt;
         always @(posedge clk) if (l1t | l1f) l1[lw] <= lwd;
         // L2: a block RAM read every cycle at the window's rows, one write (training's); training
         // reads its slots' counters and recency bits (small distributed RAMs), never the entries
         (* ram_style = "block" *) reg [EW-1:0] btb [0:NB-1];
         reg [1:0]    bc  [0:NB-1];           // a conditional's bimodal counter (the entry's
                                              // type field carries only conditional-or-not for it)
         reg          nu  [0:NB-1];           // trained recently
         reg [EW-1:0] q;
         reg [RB-1:0] rq;                     // the row q was read at
         integer bi;
         initial for (bi = 0; bi < NB; bi = bi + 1) begin btb[bi] = {EW{1'b0}}; bc[bi] = 2'b00; nu[bi] = 1'b0; end
         initial begin q = {EW{1'b0}}; rq = {RB{1'b0}}; end
         wire [RB-1:0] r2 = bslot(pa, gw);
         wire [1:0]   cq = bc[rq];
         assign bc_rd[{BI, WI}] = bc[tsl[gw]];
         assign nu_rd[{BI, WI}] = nu[tsl[gw]];
         assign wq2[gw] = {q[EW-1:TGTW+2], q[TGTW+2] ? q[TGTW+1:TGTW] : cq, q[TGTW-1:0]};
         assign m2[gw]  = q[EW-1 -: TAGW] == s2_tag;
         always @(posedge clk) q <= btb[r2];
         always @(posedge clk) if (l1t) btb[tsl[gw]] <= t_ent;
         always @(posedge clk) begin
            rq <= r2;
            if (l1t & res_cbr) bc[tsl[gw]]  <= t_nudge;
            if (tw_b)          nu[tsl[gw]]  <= nu_d[gw];
         end
      end
      // each level's first matching way: L1's for the stream now, L2's for the check
      wire [W-1:0]    f1 = m1 & (~m1 + 1'b1);
      wire [W-1:0]    f2 = m2 & (~m2 + 1'b1);
      reg  [L1EW-1:0] s1e;
      reg  [EW-1:0]   s2e;
      reg  [BTBW-1:0] x1, x2;
      integer         wi;
      always @* begin
         s1e = {L1EW{1'b0}};  s2e = {EW{1'b0}};  x1 = {BTBW{1'b0}};  x2 = {BTBW{1'b0}};
         for (wi = 0; wi < W; wi = wi + 1) begin
            if (f1[wi]) begin s1e = e1[wi];   x1 = wi[BTBW-1:0]; end
            if (f2[wi]) begin s2e = wq2[wi];  x2 = wi[BTBW-1:0]; end
         end
      end
      assign bq[gb]  = {{TAGW{1'b0}}, s1e[TGTW+2:0]};       // the pick reads the type and the target
      assign bh[gb]  = |m1;
      assign m1_f[gb*W +: W] = m1;
      assign bw[gb]  = x1;
      assign bq2[gb] = s2e;
      assign bh2[gb] = |m2;
      assign bw2[gb] = x2;
      // the fill for this bank: L2's entry into L2's way where L2 hit, and every other way that held
      // the halfword in L1 cleared -- L1's rows are not L2's, so an L2 eviction leaves L1's copy, and a
      // copy in a way before L2's would be L1's first match for ever. An entry is never empty (zeros
      // are tag 0, which some windows' tags are), so a cleared way takes the window's L1 tag inverted:
      // it cannot match the window, and the window's next lookup is L2's view (e_conv checks it).
      wire            fp2 = fpk2[gb];
      for (gw = 0; gw < W; gw = gw + 1) begin : fl
         localparam [BTBW-1:0] WI = gw;
         localparam integer    FI = gb*W + gw;
         wire l2w = bh2[gb] & (x2 == WI);                     // L2's way: takes its entry
         always @(posedge clk) begin
            fl_w[FI] <= inw2[gb] & (l2w | s2_m1[FI]);
            fl_c[FI] <= ~l2w;
            fl_r[FI*L1RB +: L1RB] <= fp2 ? lslot(s2_sa, WI) : lslot1(s2_sa, WI);
         end
      end
      always @(posedge clk) fl_e[gb*L1EW +: L1EW] <= {ltag(s2_sa, s2_tag), s2e[TGTW+2:0]};
      reg [YEW-1:0] yc  [0:NY-1];
      reg [YEW-1:0] yqr;
      integer yi;
      initial for (yi = 0; yi < NY; yi = yi + 1) yc[yi] = {YEW{1'b0}};
      initial yqr = {YEW{1'b0}};
      assign yq[gb] = yqr;
      assign yc_rd[gb] = yc[yc_idx[YBITS-1:3]];
      always @(posedge clk) begin
         if (reset | mv) yqr <= yc[yr ^ yh];
         if (y_wr  & (yc_idx[2:0] == gb)) yc[yc_idx[YBITS-1:3]] <= y_ent;
      end
   end endgenerate
   generate for (gw = 0; gw < W; gw = gw + 1) begin : tsel_rd
      localparam [BTBW-1:0] WI = gw;
      assign tr_bc[gw] = bc_rd[{t_last[3:1], WI}];
      assign tr_nu[gw] = nu_rd[{t_last[3:1], WI}];
   end endgenerate
   assign yc_tr = yc_rd[yc_idx[2:0]];

   // L2's view of the window registered last cycle
   wire [8*EW-1:0]   bq2_f;
   wire [8*BTBW-1:0] bw2_f;
   generate for (gq = 0; gq < 8; gq = gq + 1) begin : bq2f
      assign bq2_f[gq*EW +: EW] = bq2[gq];  assign bw2_f[gq*BTBW +: BTBW] = bw2[gq];
   end endgenerate
   smolrv64_wpick #(.PCW(PCW), .TAGW(TAGW), .TGTW(TGTW), .BTBW(BTBW), .GHL(GHL), .RASB(RASB), .YBITS(YBITS),
                    .YTAGW(YTAGW)) u_chk
     (.sa(s2_sa), .sk(s2_sk), .np(s2_np), .bq_f(bq2_f), .bh(bh2), .bw_f(bw2_f), .yq_f(s2_yq),
      .ras_top(s2_rtop), .ras_ptr(s2_rsp), .ghr(s2_ghr), .yix(s2_yix),
      .hy(), .inw(inw2), .mkv(mk2), .coh(), .nmk(), .hk(), .pe(pe2), .ety(), .cut(), .tk(), .call(), .ret(),
      .w_one(), .ftc(), .p_nx(nx2), .rsp_post(), .ghr_post(), .pw_e0(), .pw_e1());
   // a restart in the check's cycle drops the window: a flush, or a correction of the one before it
   assign d_v   = s2_v & ~rs;
   assign d_bad = d_v & ((pe2 != s2_pe) | (nx2 != s2_nx));
   assign d_fil = d_v & (d_bad | (mk2 != s2_mk));

   always @(posedge clk) begin
      s2_v <= ~reset & adv & ~rs;
      if (adv) begin
         s2_sa  <= sa_q;  s2_sk <= sk_q;  s2_np <= np_q;  s2_yq <= yq_f;
         s2_rtop <= ras[ras_ptr];  s2_rsp <= ras_ptr;  s2_ghr <= ghr;  s2_yix <= yr_q ^ yh_q;
         s2_pe  <= pe;  s2_nx <= p_nx;  s2_mk <= mkv;  s2_m1 <= m1_f;
         s2_wp  <= pq_wp;  s2_n <= push ? nmk[2:0] : 3'd0;
      end
      cr_q   <= ~reset & d_bad;
      mf_q   <= ~reset & d_fil & ~d_bad;
      cr_a   <= {s2_sa[PCW-1:4], s2_sk, 1'b0};  cr_np <= s2_np;  cr_ghr <= s2_ghr;  cr_rsp <= s2_rsp;
      cr_wp  <= s2_wp;
      cr_n   <= s2_n + (push ? nmk[2:0] : 3'd0);
      cr_k   <= adv ? 2'd2 : 2'd1;
   end

   always @(posedge clk) begin
      if (reset | mv) begin
         sa_q <= {na[PCW-1:4], 4'b0000};  sk_q <= na[3:1];  yh_q <= yh;  yr_q <= yr;
         np_q <= rs & rsn;
      end
      if (tw_v & ~tr_hit) vrot <= vrot + 1'b1;
      if (reset) begin
         ghr <= 0; ras_ptr <= 0;
         pq_rp <= 0; pq_wp <= 0; pq_n <= 0;
      end else begin
         // The RAS array is not restored: with the pointer back, a wrong-path push sits above it
         // and is unreachable; a wrong-path pop-then-push costs a mispredicted return.
         if (rs) begin
            ghr <= rsg;  ras_ptr <= rsr;
         end else if (adv) begin
            ghr <= ghr_post;  ras_ptr <= rsp_post;
            if (tk & call) ras[ras_ptr + 1'b1] <= ftc;
         end
         if (flush) begin
            pq_rp <= 0;  pq_wp <= 0;  pq_n <= 0;
         end else begin
            pq_rp <= pq_rp + popw[PQB-1:0];
            if (cr_q) begin
               pq_wp <= cr_wp;
               pq_n  <= pq_n - {{(PQB+1-3){1'b0}}, cr_n} - popw;
            end else begin
               if (push) pq_wp <= pq_wp + nmk[PQB-1:0];
               pq_n <= pq_n + (push ? {{(PQB+1-4){1'b0}}, nmk} : {(PQB+1){1'b0}}) - popw;
            end
         end
      end
   end

   wire e_full = ~reset & push & ~pq_room;
   wire e_many = ~reset & push & (nmk > M);
   wire e_empt = ~reset & (popw > pq_n) & ~flush;
   wire e_end  = ~reset & p_cut & ~inw[hk];
   // A CORRECTION CONVERGES: it fills L1 from L2 at its window and restarts there, so that window's
   // next check agrees -- unless another write changed either level since the window's lookup:
   // training (which takes L1's write port from a fill), or a fill of a window before it that covered
   // its banks. One that recurs is a livelock: fetch waits for the window's bytes (the aligner's
   // bytes_late) and they never come.
   wire            wr_any = tw_v | cr_q | mf_q;       // a cycle that writes either level
   reg  [1:0]      wr_h;                              // ...in the two cycles before
   reg             cv_q, cv_t;
   reg  [PCW-1:0]  cv_a;
   initial begin wr_h = 2'b00;  cv_q = 1'b0;  cv_t = 1'b0;  cv_a = {PCW{1'b0}}; end
   always @(posedge clk) begin
      wr_h <= {wr_h[0], wr_any};
      if (reset | flush | d_v) cv_q <= 1'b0;             // its window's check came, or a flush took it
      if (cr_q & ~flush) begin cv_q <= 1'b1;  cv_a <= cr_a;  cv_t <= tw_v | (|wr_h); end
      else if (wr_any) cv_t <= 1'b1;
   end
   wire e_conv = ~reset & d_bad & cv_q & ~cv_t & ({s2_sa[PCW-1:4], s2_sk, 1'b0} == cv_a);
   reg  [3:0] err_q;
   always @(posedge clk) err_q <= reset ? 4'd0 : {e_conv, e_end, e_empt, e_full};
   assign err = err_q;
   always @(posedge clk) begin
      if (e_full) $fatal(1, "smolrv64_predictor: a prediction pushed into a full queue");
      if (e_many) $fatal(1, "smolrv64_predictor: %0d marks in a pair, more than %0d", nmk, M);
      if (e_empt) $fatal(1, "smolrv64_predictor: the aligner popped %0d of %0d predictions", pop, pq_n);
      if (e_end)  $fatal(1, "smolrv64_predictor: a window ends at bank %0d, outside it (offset %0d)", hk, sk_q);
      if (e_conv) $fatal(1, "smolrv64_predictor: the correction at %h did not converge: L1 still disagrees with L2", cv_a);
   end

`ifdef BP_TRACE
   // Per-pair and per-resolve trace (flood volume, so gated; +trace_from/+trace_to).
   reg [63:0] bpt_cyc, bpt_from, bpt_to;
   initial begin
      bpt_cyc = 64'd0;
      if (!$value$plusargs("trace_from=%d", bpt_from)) bpt_from = 64'd0;
      if (!$value$plusargs("trace_to=%d",   bpt_to))   bpt_to   = 64'hFFFF_FFFF_FFFF_FFFF;
   end
   wire bpt_on = (bpt_cyc >= bpt_from) & (bpt_cyc <= bpt_to);
   always @(posedge clk) begin
      bpt_cyc <= bpt_cyc + 64'd1;
      if (bpt_on & adv & (|mkv))
         $display("[BP] c=%0d PRED pair=%h skip=%0d end=%0d type=%b yhit=%b dir=%b tk=%b nx=%h ghr=%h rsp=%0d",
                  bpt_cyc, pc64(sa_q), sk_q, p_end, ety, hy[hk], tk, tk, pc64(p_nx), ghr, ras_ptr);
      if (bpt_on & res_v)
         $display("[BP] c=%0d RES last=%h cbr=%b call=%b ret=%b taken=%b tgt=%h | carried hit=%b ctr=%0d yhit=%b yctr=%0d yidx=%0d | ywr=%b | now hit=%b nu=%b way=%b wr=%b",
                  bpt_cyc, pc64(res_last), res_cbr, res_call, res_ret, res_taken, pc64(res_tgt),
                  t_hit, t_ctr, yc_hit, yc_ctr, yc_idx, y_wr, tr_hit, tr_nu, tsel, tw_v);
      if (bpt_on & flush)
         $display("[BP] c=%0d RESTART at=%h rollback=%b ghr<=%h ras_ptr<=%0d", bpt_cyc, pc64(rst_a), rollback, rs_ghr, rs_rsp);
   end
`endif
endmodule

// THE END PICK (smolrv64_predictor): a window's marks, where it ends and where the stream goes on,
// from its eight banks' entries (the first matching way's) and corrector rows. Bank k holds the
// window's halfword at address bits [3:1] == k, in the first pair (sa) when k is at or after the
// offset sk, else in the pair after it. Every position evaluates its own hit, corrector hit and
// direction at once; the marks are the hits in WINDOW ORDER -- the banks from sk round to sk - 1 --
// up to the first taken one, or the M-th.
module smolrv64_wpick
  #(parameter PCW   = 40,
    parameter TAGW  = 12,
    parameter TGTW  = 38,
    parameter BTBW  = 1,
    parameter GHL   = 11,
    parameter RASB  = 3,
    parameter YBITS = 11,
    parameter YTAGW = 8,
    parameter EW    = TAGW + 3 + TGTW,
    parameter YEW   = YTAGW + 2,
    parameter QEW   = 1 + PCW + RASB + GHL + 1 + 2 + YBITS + BTBW + 2)
   (input  wire [PCW-1:0]      sa,         // the window: its first pair...
    input  wire [2:0]          sk,         // ...its first halfword in it...
    input  wire                np,         // ...and predict nothing in it
    input  wire [8*EW-1:0]     bq_f,         // bank k's entry at [k*EW +: EW]...
    input  wire [7:0]          bh,         // ...whether any way matched
    input  wire [8*BTBW-1:0]   bw_f,         // ...and which
    input  wire [8*YEW-1:0]    yq_f,         // bank k's corrector row
    input  wire [PCW-1:0]      ras_top,
    input  wire [RASB-1:0]     ras_ptr,
    input  wire [GHL-1:0]      ghr,
    input  wire [YBITS-4:0]    yix,        // the corrector row read, as training indexes it
    output reg  [7:0]          hy,         // per bank: the corrector hit...
    output reg  [7:0]          inw,        // ...the bank holds one of the window's halfwords
    output reg  [7:0]          mkv,        // ...marked
    output reg  [7:0]          coh,        // ...where the window ends at a mark (one-hot)
    output reg  [3:0]          nmk,        // the marks
    output reg  [2:0]          hk,         // the bank where a mark ends it (7: none)
    output wire [2:0]          pe,         // its last position
    output wire [2:0]          ety,        // the ending mark's type
    output reg                 cut,        // a mark ends it...
    output wire                tk,         // ...taken
    output wire                call,       // ...a call
    output wire                ret,        // ...a return
    output wire                w_one,      // it stops at its first pair's end (its page's last pair)
    output wire [PCW-1:0]      ftc,        // the halfword after the ending mark
    output wire [PCW-1:0]      p_nx,       // where the stream goes on
    output wire [RASB-1:0]     rsp_post,
    output wire [GHL-1:0]      ghr_post,
    output wire [QEW-1:0]      pw_e0,      // its first two marks' queue entries
    output wire [QEW-1:0]      pw_e1);
   localparam [2:0] TY_CALL = 3'b101, TY_RET = 3'b110;
   localparam integer M = 2;                 // marks per window (the queue's pushes per cycle)
`include "smolrv64_phist.vh"
`include "smolrv64_pc.vh"
`include "smolrv64_ytag.vh"
   wire [EW-1:0]   bq [0:7];
   wire [BTBW-1:0] bw [0:7];
   wire [YEW-1:0]  yq [0:7];
   genvar gu;
   generate for (gu = 0; gu < 8; gu = gu + 1) begin : un
      assign bq[gu] = bq_f[gu*EW +: EW];  assign bw[gu] = bw_f[gu*BTBW +: BTBW];  assign yq[gu] = yq_f[gu*YEW +: YEW];
   end endgenerate
   reg  [7:0]      hv, hd1;                  // per position: hit, direction taken
   reg  [EW-1:0]   e;                        // the ending mark's entry
   reg  [EW-1:0]   ek;
   reg  [YEW-1:0]  ck;
   integer         k;
   wire [PCW-1:0]  sa16 = sa + PC16;       // the pair after the first (an adder from a register)
   // THE WINDOW STOPS AT ITS FIRST PAIR'S END when that pair is the last of its page: the I$ reads
   // one page a request.
   assign          w_one = (&sa[11:4]) & (sk != 3'd0);
   // The corrector's tag for the pair after takes the pair's own bits plus one, wrapping inside them
   // (ak below): wrong only there, and then that window's second pair misses -- a lost hint, never a
   // wrong one (sa + 16 in front of the compares cost five levels on the loop).
   reg  [PCW-1:0]  ak;
   always @* begin
      hv = 8'd0;  hy = 8'd0;  hd1 = 8'd0;  inw = 8'd0;  ak = sa;
      for (k = 0; k < 8; k = k + 1) begin
         ek      = bq[k];
         ck      = yq[k];
         ak      = sa;
         if (k[2:0] < sk) ak[YTAGW:4] = sa[YTAGW:4] + 1'b1;
         ak[3:1] = k[2:0];
         inw[k]  = (k[2:0] >= sk) | ~w_one;
         hv[k]   = ~np & inw[k] & bh[k];
         hy[k]   = ~ek[TGTW+2] & (ck[YEW-1 -: YTAGW] == ytagf(ak));
         hd1[k]  = ek[TGTW+2] | (hy[k] ? ck[1] : ek[TGTW+1]);
      end
   end
   // Where the window ends, one-hot by bank (0: at its own end): at the first taken hit when it is
   // one of the first two, else at the second hit (M = 2), in window order. Each position decides
   // from the hits before it alone -- a taken hit with none before, or any hit with one untaken hit
   // before -- and "before" is a mask of the banks from sk up to it (bmk), a function of the
   // register alone: no find-first chain and no rotation stand between the rows and the next
   // window's address. The marks are the hits up to where it ends, and its entry is the one-hot pick
   // of the eight.
   wire [7:0] tkv = hv & hd1;
   reg  [7:0] bmk [0:7];                     // bank k: the banks before it in the window
   reg  [7:0] lo, ba, bb;
   // THE END PICK WITH CONSTANT MASKS: the window's order is the first pair's banks (fpm, at or after
   // the offset) ascending, then the second pair's ascending, so a bank's hits before it are its own
   // pair's below it -- the constant mask main had -- and, for a second-pair bank, the first pair's
   // as a whole (any of them, exactly one, any taken: shared by every bank). The offset register
   // only splits the hits; it is not an input of each bank's pick.
   function [7:0] fpm8(input [2:0] off);                           // the banks at or after off
      fpm8 = ~((8'd1 << off) - 8'd1);
   endfunction
   wire [7:0] fpm   = fpm8(sk);
   wire [7:0] hva   = hv & fpm,  hvb = hv & ~fpm,  tka = tkv & fpm,  tkb = tkv & ~fpm;
   wire       a_any = |hva;
   wire       a_one = a_any & ~|(hva & (hva - 8'd1));
   wire       a_tk  = |tka;
   always @* begin
      for (k = 0; k < 8; k = k + 1)
         bmk[k] = (k[2:0] >= sk) ? (((8'd1 << k) - 8'd1) & ~((8'd1 << sk) - 8'd1))
                                   : (((8'd1 << k) - 8'd1) |  ~((8'd1 << sk) - 8'd1));
      for (k = 0; k < 8; k = k + 1) begin
         lo     = (8'd1 << k) - 8'd1;
         ba     = hva & lo;
         bb     = hvb & lo;
         coh[k] = (hva[k] & ((~|ba & tka[k]) | (|ba & ~|(ba & (ba - 8'd1)) & ~|(tka & lo))))
                | (hvb[k] & ((~a_any & ~|bb & tkb[k])
                           | (~a_any & |bb & ~|(bb & (bb - 8'd1)) & ~|(tkb & lo))
                           | (a_one & ~a_tk & ~|bb)));
      end
      cut = |coh;
      for (k = 0; k < 8; k = k + 1) mkv[k] = hv[k] & ~|(coh & bmk[k]);
      nmk = {3'd0, |mkv} + {3'd0, |(mkv & (mkv - 8'd1))};
      hk  = {~|(coh & 8'h0f), ~|(coh & 8'h33), ~|(coh & 8'h55)};   // 7 when no mark ends it
      e   = {EW{1'b0}};
      for (k = 0; k < 8; k = k + 1) e = e | ({EW{coh[k]}} & bq[k]);
   end
   // The pair the ending halfword is in, and the pair the halfword after it is in: the first when the
   // ending bank is at or after the offset (bank 7's next halfword starts the pair after), else the
   // pair after. Each is one OR of the end pick against the offset's mask, so the addresses are a
   // two-way pick of registers and their sums. With no mark ending the window, n_fp is 0 and the
   // next window starts 16 bytes on.
   wire            e_fp = |(coh & fpm);
   wire            n_fp = |(coh & fpm & 8'h7f);
   wire [PCW-1:0]  nf   = n_fp ? sa : sa16;                      // the next window's first pair
   wire [PCW-1:0]  h    = {e_fp ? sa[PCW-1:4] : sa16[PCW-1:4], hk, 1'b0};
   assign          ftc  = {nf[PCW-1:4], hk + 3'd1, 1'b0};          // the halfword after the ending CTI
   assign          ety  = e[TGTW +: 3];
   wire [TGTW-1:0] etg  = e[TGTW-1:0];
   wire            dir  = |(coh & hd1);
   assign          call = (ety == TY_CALL);
   assign          ret  = (ety == TY_RET);
   // the window's last position: where a mark ends it, else its own end
   assign          pe   = cut ? hk - sk : w_one ? 3'd7 - sk : 3'd7;
   assign          tk   = dir;                                     // the window ends at a taken mark
   function [PCW-1:0] tadr(input [EW-1:0] x);                      // an entry's target
      tadr = {{(PCW-TGTW-1){x[TGTW-1]}}, x[TGTW-1:0], 1'b0};
   endfunction
   wire [PCW-1:0]  tgt  = ret ? ras_top : tadr(e);
   // the next window when no mark ends this one: 16 bytes on, at the same offset
   assign          p_nx = tk ? tgt : {nf[PCW-1:4], cut ? hk + 3'd1 : w_one ? 3'd0 : sk, 1'b0};
   assign          rsp_post = (tk & call) ? ras_ptr + 1'b1 : (tk & ret) ? ras_ptr - 1'b1 : ras_ptr;
   assign          ghr_post = tk ? phist_step(ghr, h) : ghr;
   // a mark's own fields of its queue entry: {taken, yhit, yctr, bank, way, type}
   localparam integer PFW = 1 + 1 + 2 + 3 + BTBW + 2;
   wire [PFW-1:0]  pf8 [0:7];
   genvar gm;
   generate for (gm = 0; gm < 8; gm = gm + 1) begin : me
      wire [EW-1:0]  me_e = bq[gm];
      wire [YEW-1:0] me_c = yq[gm];
      wire           me_y = ~me_e[TGTW+2] & hy[gm];
      assign pf8[gm] = {coh[gm] & dir, me_y, me_y ? me_c[1:0] : 2'b01, gm[2:0], bw[gm], me_e[TGTW +: 2]};
   end endgenerate

   // the window's entries in mark order (nmk <= M = 2): its first two marks in window order, each
   // one-hot by bank -- no mark before it, exactly one before it -- by the end pick's masks. Only a
   // mark's own fields are picked; the rest of an entry is the window's.
   reg  [7:0]      m1h, m2h, mbl;
   reg  [PFW-1:0]  pf0, pf1;
   integer         pwk;
   always @* begin
      pf0 = {PFW{1'b0}};  pf1 = {PFW{1'b0}};
      for (pwk = 0; pwk < 8; pwk = pwk + 1) begin
         mbl      = mkv & bmk[pwk];
         m1h[pwk] = mkv[pwk] & ~|mbl;
         m2h[pwk] = mkv[pwk] & |mbl & ~|(mbl & (mbl - 8'd1));
         pf0      = pf0 | ({PFW{m1h[pwk]}} & pf8[pwk]);
         pf1      = pf1 | ({PFW{m2h[pwk]}} & pf8[pwk]);
      end
   end
   function [QEW-1:0] pqe(input [PFW-1:0] p);                     // a mark's entry
      pqe = {p[PFW-1], tgt, ras_ptr, ghr, p[PFW-2 -: 3], yix, p[BTBW+2 +: 3], p[BTBW+1:0]};
   endfunction
   assign          pw_e0 = pqe(pf0);
   assign          pw_e1 = pqe(pf1);
endmodule

`default_nettype wire
