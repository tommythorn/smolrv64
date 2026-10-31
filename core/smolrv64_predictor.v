`default_nettype none

// The branch predictor, as the fetch stream: it holds the stream's address, looks up the BTB, the
// YAGS corrector and the RAS once per 16-byte pair the fetch ring (smolrv64_fring) asks the I$ for,
// steers the stream to the next pair, and queues the predictions it made for the aligner to take
// back in order (docs/PLAN-2026-09-24-frontend-stage4.md, increment 1b).
//
// KEYED BY THE LAST HALFWORD. A BTB entry belongs to its CTI's LAST halfword, and a pair covers
// eight halfwords, so the BTB and the corrector are each eight banks, one per halfword position in
// 16 bytes (address bits [3:1]), and a pair reads one row of each. Training keys the same way, by
// the resolving CTI's PC and length, so nothing about where a lookup happened rides with the
// instruction, and no two CTIs share an entry. (A coarser key does not survive real code: an 8-byte
// key put 28.7% of the kernel's CTIs beside another, and a 4-byte key still put every
// `jal f; c.bnez a0, loop` in one entry, which the two evicted from each other on every iteration.)
//
// THE STREAM is a registered pair start and skip (sa_q, sk_q). When the ring takes the pair (adv)
// or the stream restarts (flush), both load the next address and the banks read its rows, so a
// pair's prediction starts from registers and the row address is the next address alone.
//
// A PAIR ENDS AT ITS FIRST PREDICTED-TAKEN TRANSFER. Every known CTI from the stream's address on
// is marked, up to the first one predicted taken, or to the M-th: p_end is that halfword, and the
// stream continues at the predicted target, at the halfword after the M-th, or at the next pair.
// A not-taken mark does not end the pair, so the aligner reads on past it. The history steps at
// most once per pair, at its end, so every mark of a pair has the same pre-state and its corrector
// reads share the history they were read with. The ring marks each marked halfword (p_mk), and
// each mark's prediction goes into the queue, in order.
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
    parameter BTBB  = 11,            // log2 BTB entries, all banks and ways
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
    output wire [PCW-1:0]       sa,         // this cycle's pair: its first byte
    output wire [2:0]           sk,         // ...and the halfwords of it before the stream's address
    output wire                 p_cut,      // the pair ends at a mark...
    output wire [2:0]           p_end,      // ...at this halfword (7 when it does not)
    output wire [7:0]           p_mk,       // the pair's marked halfwords
    output wire                 pq_room,
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
    output wire [2:0]           err);       // its invariants, registered (the integrity log)

   localparam W    = 1 << BTBW;             // BTB ways
   initial if (PDW != RASB + GHL + (1+2+YBITS) + BTBW + (1+2))
      $fatal(1, "smolrv64_predictor: the details are %0d bits; {rsp, ghr, yhit, yctr, yidx, way, hit, ctr} is %0d",
             PDW, RASB + GHL + (1+2+YBITS) + BTBW + (1+2));
   localparam RB   = BTBB - 3 - BTBW;       // row bits per BTB bank and way
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
   // The index consumes the halfword's bits [YB+3:1], so the tag carries them (rule B9).
   function [YTAGW-1:0] ytagf(input [PCW-1:0] a);
      ytagf = a[YTAGW:1] ^ {{(2*YTAGW-YBITS){1'b0}}, a[YBITS:YTAGW+1]}
            ^ a[YBITS+YTAGW:YBITS+1] ^ a[YBITS+2*YTAGW:YBITS+YTAGW+1] ^ {{(YTAGW-1){1'b0}}, a[PCW-1]};
   endfunction

   // ------------------------------------------------------------ speculative state
   reg [GHL-1:0]  ghr;                       // the stream's history
   reg [PCW-1:0]  ras [0:RASN-1];
   reg [RASB-1:0] ras_ptr;                   // the stream's RAS top
   reg [PCW-1:0]  sa_q;                      // the stream: this pair's first byte...
   reg [2:0]      sk_q;                      // ...the halfwords before the stream's address...
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
   // The corrector's rows are read with the history before this pair's own shift -- a pair of
   // lag -- so this pair's direction never reaches the next pair's read index; the folded history
   // the rows were read with is registered (yh_q) and carried in the index training uses.
   wire [YB-1:0]   yh      = yfold(flush ? rs_ghr : ghr);
   reg  [YB-1:0]   yh_q, yr_q;
   initial begin yh_q = {YB{1'b0}};  yr_q = {YB{1'b0}}; end

   // ------------------------------------------------------------ this pair's prediction
   // bq[k]/yq[k] are bank k's rows read for sa_q: halfword k of the pair. Every position evaluates
   // its own hit, corrector hit and direction from registers at once; the marks are the hits in
   // order up to the first taken one, or the M-th.
   localparam integer M = 2;                 // marks per pair (the queue's pushes per cycle)
   wire [EW-1:0]   bq [0:7];                 // halfword k's entry (its first matching way)...
   wire [7:0]      bh;                       // ...whether any way matched
   wire [BTBW-1:0] bw [0:7];                 // ...and which (training's, carried in the details)
   wire [YEW-1:0]  yq [0:7];
   reg  [7:0]      hv, hy, hd1;              // per position: hit, corrector hit, direction taken
   reg  [7:0]      mkv;                      // the marks
   reg  [3:0]      nmk;                      // ...how many
   reg  [2:0]      hk;                       // where the pair ends (7: at its own end)
   reg             cut;                      // ...at a mark
   reg  [EW-1:0]   e;                        // the ending mark's entry
   reg  [EW-1:0]   ek;
   reg  [YEW-1:0]  ck;
   integer         k;
   // The pair is 16-byte aligned (sa_q is ra), so halfword k's address is sa_q with k in bits
   // [3:1] and its BTB tag is the pair's: no adder in front of the compares.
   wire [TAGW-1:0] stag = btag(sa_q);
   reg  [PCW-1:0]  ak;
   always @* begin
      hv = 8'd0;  hy = 8'd0;  hd1 = 8'd0;  ak = sa_q;
      for (k = 0; k < 8; k = k + 1) begin
         ek     = bq[k];
         ck     = yq[k];
         ak[3:1] = k[2:0];
         hv[k]  = ~np_q & (k[2:0] >= sk_q) & bh[k];
         hy[k]  = ~ek[TGTW+2] & (ck[YEW-1 -: YTAGW] == ytagf(ak));
         hd1[k] = ek[TGTW+2] | (hy[k] ? ck[1] : ek[TGTW+1]);
      end
   end
   // Where the pair ends, one-hot (0: at its own end): at the first taken hit when it is one of
   // the first two, else at the second hit (M = 2). Each position decides from the bits below it
   // alone -- a taken hit with no hit below, or any hit with one untaken hit below -- so no
   // find-first chain stands between the rows and the next pair's address. The marks are the
   // hits up to where it ends, and its entry is the one-hot pick of the eight.
   wire [7:0] tkv = hv & hd1;
   reg  [7:0] coh, blo;
   always @* begin
      for (k = 0; k < 8; k = k + 1) begin
         blo    = hv & ((8'd1 << k) - 8'd1);
         coh[k] = hv[k] & ((~|blo & tkv[k])
                         | (|blo & ~|(blo & (blo - 8'd1)) & ~|(tkv & ((8'd1 << k) - 8'd1))));
      end
      cut = |coh;
      for (k = 0; k < 8; k = k + 1) mkv[k] = hv[k] & ~|(coh & ((8'd1 << k) - 8'd1));
      nmk = {3'd0, |mkv} + {3'd0, |(mkv & (mkv - 8'd1))};
      hk  = {~|(coh & 8'h0f), ~|(coh & 8'h33), ~|(coh & 8'h55)};   // 7 when it ends at its end
      e   = {EW{1'b0}};
      for (k = 0; k < 8; k = k + 1) e = e | ({EW{coh[k]}} & bq[k]);
   end
   wire [PCW-1:0]  h    = {sa_q[PCW-1:4], hk, 1'b0};               // its address (the pair's, plus the halfword)
   wire [PCW-1:0]  sa16 = sa_q + PC16;                            // the next pair (an adder from a register)
   wire [2:0]      ety  = e[TGTW +: 3];
   wire [TGTW-1:0] etg  = e[TGTW-1:0];
   wire            dir  = |(coh & hd1);
   wire            call = (ety == TY_CALL), ret = (ety == TY_RET);
   assign p_cut = cut;
   assign p_end = hk;
   assign p_mk  = mkv;
   wire            tk   = dir;                                     // the pair ends at a taken mark
   // the halfword after the CTI: the next one of this pair, or the next pair (no adder behind
   // the mark: the mark's position is the loop's latest signal)
   wire [PCW-1:0]  ftc  = (hk == 3'd7) ? sa16 : {sa_q[PCW-1:4], hk + 3'd1, 1'b0};
   function [PCW-1:0] tadr(input [EW-1:0] x);                      // an entry's target
      tadr = {{(PCW-TGTW-1){x[TGTW-1]}}, x[TGTW-1:0], 1'b0};
   endfunction
   wire [PCW-1:0]  tgt  = ret ? ras[ras_ptr] : tadr(e);
   wire [PCW-1:0]  p_nx = tk ? tgt : cut ? ftc : sa16;
   wire [RASB-1:0] rsp_post = (tk & call) ? ras_ptr + 1'b1 : (tk & ret) ? ras_ptr - 1'b1 : ras_ptr;
   wire [GHL-1:0]  ghr_post = tk ? phist_step(ghr, h) : ghr;
   // each mark's entry: the ending one's prediction, a not-taken one's details, the pair's pre-state
   wire            push     = adv & (|mkv) & ~flush;
   wire [QEW-1:0]  pq_e8 [0:7];
   genvar gm;
   generate for (gm = 0; gm < 8; gm = gm + 1) begin : me
      wire [EW-1:0]  me_e = bq[gm];
      wire [YEW-1:0] me_c = yq[gm];
      wire           me_y = ~me_e[TGTW+2] & hy[gm];
      assign pq_e8[gm] = {cut & (hk == gm) & dir, tgt, ras_ptr, ghr, me_y, me_y ? me_c[1:0] : 2'b01,
                          yr_q ^ yh_q, gm[2:0], bw[gm], me_e[TGTW +: 2]};
   end endgenerate

   // the pair's entries in mark order (nmk <= M = 2), and the bank each lands in
   reg  [QEW-1:0]  pw_e0, pw_e1;
   integer         pwk;
   always @* begin
      pw_e0 = pq_e8[0];  pw_e1 = pq_e8[0];
      for (pwk = 7; pwk >= 0; pwk = pwk - 1)
         if (mkv[pwk]) begin pw_e1 = pw_e0;  pw_e0 = pq_e8[pwk]; end
   end
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

   // ------------------------------------------------------------ the next pair, and its rows
   // The stream's next address: the restart's, or this pair's prediction. A pair is the 16 bytes
   // holding it, so every bank reads the same row, the address's bits above the pair.
   wire            mv    = flush | adv;
   wire [PCW-1:0]  na    = flush ? rst_a : p_nx;
   wire [PCW-1:0]  ra    = {na[PCW-1:4], 4'b0000};
   wire [2:0]      rk    = na[3:1];
   // The corrector's row for the next pair is read AHEAD, at this pair's address (the restart's on a
   // flush): the next pair's address waits on the BTB's read and pick, which put the corrector's read
   // in the predictor's loop. The row read is registered (yr_q) and carried for training.
   wire [YB-1:0]   yr    = flush ? rst_a[YB+3:4] : sa_q[YB+3:4];
   genvar gb, gw;
   // Way w's row for the next pair, each candidate skewed before the pick rather than the picked
   // address after it: the restart's; the ending CTI's target, the RAS top for a return, else its
   // entry's, the eight skewed and picked one-hot as the entry is; this pair's own, after a
   // not-taken mark short of the pair's end (the next halfword is in this pair); else the next.
   wire [RB-1:0]   rr    [0:W-1];
   generate for (gw = 0; gw < W; gw = gw + 1) begin : rw
      reg [RB-1:0] rt;
      integer      ki;
      always @* begin
         rt = {RB{1'b0}};
         for (ki = 0; ki < 8; ki = ki + 1) rt = rt | ({RB{coh[ki]}} & bslot(tadr(bq[ki]), gw));
      end
      assign rr[gw] = flush ? bslot(rst_a, gw) : tk ? (ret ? bslot(ras[ras_ptr], gw) : rt)
                    : (cut & (hk != 3'd7)) ? bslot(sa_q, gw) : bslot(sa16, gw);
      always @(posedge clk) if (!reset && mv && (rr[gw] != bslot(ra, gw)))
         $fatal(1, "smolrv64_predictor: way %0d reads row %0h for the next pair, whose row is %0h", gw, rr[gw], bslot(ra, gw));
   end endgenerate
   // every bank and way's slot for the training CTI, {bank, way}; the CTI's bank is picked below
   wire [1:0]      bc_rd [0:8*W-1];
   wire            nu_rd [0:8*W-1];
   wire [YEW-1:0]  yc_rd [0:7];
   // The BTB's read is the predictor's loop (the rows read for this pair -> the marks -> the next
   // pair's address -> the rows read for it), and each way's entries are a block RAM with one read
   // port, the stream's (q is its output latch), and one write port, training's: training reads
   // its slots' counters and recency bits (small distributed RAMs), never the entries.
   generate for (gb = 0; gb < 8; gb = gb + 1) begin : bank
      localparam [2:0] BI = gb;
      wire [EW-1:0] wq [0:W-1];               // each way's entry for halfword gb
      wire [W-1:0]  wm;                       // ...whether its tag is the pair's
      for (gw = 0; gw < W; gw = gw + 1) begin : way
         localparam [BTBW-1:0] WI = gw;
         (* ram_style = "block" *) reg [EW-1:0] btb [0:NB-1];
         reg [1:0]    bc  [0:NB-1];           // a conditional's bimodal counter (the entry's
                                              // type field carries only conditional-or-not for it)
         reg          nu  [0:NB-1];           // trained recently
         reg [EW-1:0] q;
         reg [1:0]    cq;
         integer bi;
         initial for (bi = 0; bi < NB; bi = bi + 1) begin btb[bi] = {EW{1'b0}}; bc[bi] = 2'b00; nu[bi] = 1'b0; end
         initial begin q = {EW{1'b0}}; cq = 2'b00; end
         assign bc_rd[{BI, WI}] = bc[tsl[gw]];
         assign nu_rd[{BI, WI}] = nu[tsl[gw]];
         assign wq[gw] = {q[EW-1:TGTW+2], q[TGTW+2] ? q[TGTW+1:TGTW] : cq, q[TGTW-1:0]};
         assign wm[gw] = q[EW-1 -: TAGW] == stag;
         wire   tw_b = tw_v & (t_last[3:1] == BI);
         always @(posedge clk) if (reset | mv) q <= btb[rr[gw]];
         always @(posedge clk) if (tw_b & tsel[gw]) btb[tsl[gw]] <= t_ent;
         always @(posedge clk) begin
            if (reset | mv)                cq <= bc[rr[gw]];
            if (tw_b & tsel[gw] & res_cbr) bc[tsl[gw]]  <= t_nudge;
            if (tw_b)                      nu[tsl[gw]]  <= nu_d[gw];
         end
      end
      // the first matching way
      wire [W-1:0]  wm1 = wm & (~wm + 1'b1);
      reg  [EW-1:0] ws;
      integer       wi;
      always @* begin
         ws = {EW{1'b0}};
         for (wi = 0; wi < W; wi = wi + 1) if (wm1[wi]) ws = wq[wi];
      end
      reg  [BTBW-1:0] wx;
      always @* begin
         wx = {BTBW{1'b0}};
         for (wi = 0; wi < W; wi = wi + 1) if (wm1[wi]) wx = wi[BTBW-1:0];
      end
      assign bq[gb] = ws;
      assign bh[gb] = |wm;
      assign bw[gb] = wx;
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

   always @(posedge clk) begin
      if (reset | mv) begin
         sa_q <= ra;  sk_q <= rk;  yh_q <= yh;  yr_q <= yr;
         np_q <= flush & rst_np;
      end
      if (tw_v & ~tr_hit) vrot <= vrot + 1'b1;
      if (reset) begin
         ghr <= 0; ras_ptr <= 0;
         pq_rp <= 0; pq_wp <= 0; pq_n <= 0;
      end else begin
         // The RAS array is not restored: with the pointer back, a wrong-path push sits above it
         // and is unreachable; a wrong-path pop-then-push costs a mispredicted return.
         if (flush) begin
            ghr <= rs_ghr;  ras_ptr <= rs_rsp;
         end else if (adv) begin
            ghr <= ghr_post;  ras_ptr <= rsp_post;
            if (tk & call) ras[ras_ptr + 1'b1] <= ftc;
         end
         if (flush) begin
            pq_rp <= 0;  pq_wp <= 0;  pq_n <= 0;
         end else begin
            if (push) pq_wp <= pq_wp + nmk[PQB-1:0];
            pq_rp <= pq_rp + popw[PQB-1:0];
            pq_n <= pq_n + (push ? {{(PQB+1-4){1'b0}}, nmk} : {(PQB+1){1'b0}}) - popw;
         end
      end
   end
   wire e_full = ~reset & push & ~pq_room;
   wire e_many = ~reset & push & (nmk > M);
   wire e_empt = ~reset & (popw > pq_n) & ~flush;
   wire e_end  = ~reset & p_cut & (hk < sk_q);
   reg  [2:0] err_q;
   always @(posedge clk) err_q <= reset ? 3'd0 : {e_end, e_empt, e_full};
   assign err = err_q;
   always @(posedge clk) begin
      if (e_full) $fatal(1, "smolrv64_predictor: a prediction pushed into a full queue");
      if (e_many) $fatal(1, "smolrv64_predictor: %0d marks in a pair, more than %0d", nmk, M);
      if (e_empt) $fatal(1, "smolrv64_predictor: the aligner popped %0d of %0d predictions", pop, pq_n);
      if (e_end)  $fatal(1, "smolrv64_predictor: a pair ends (%0d) before the stream's address (%0d)", hk, sk_q);
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
                  bpt_cyc, pc64(sa_q), sk_q, p_end, ety, hy[hk], dir, tk, pc64(p_nx), ghr, ras_ptr);
      if (bpt_on & res_v)
         $display("[BP] c=%0d RES last=%h cbr=%b call=%b ret=%b taken=%b tgt=%h | carried hit=%b ctr=%0d yhit=%b yctr=%0d yidx=%0d | ywr=%b | now hit=%b nu=%b way=%b wr=%b",
                  bpt_cyc, pc64(res_last), res_cbr, res_call, res_ret, res_taken, pc64(res_tgt),
                  t_hit, t_ctr, yc_hit, yc_ctr, yc_idx, y_wr, tr_hit, tr_nu, tsel, tw_v);
      if (bpt_on & flush)
         $display("[BP] c=%0d RESTART at=%h rollback=%b ghr<=%h ras_ptr<=%0d", bpt_cyc, pc64(rst_a), rollback, rs_ghr, rs_rsp);
   end
`endif
endmodule

`default_nettype wire
