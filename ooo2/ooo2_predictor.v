`default_nettype none

// The branch predictor, as the fetch stream: it holds the stream's address, looks up the BTB, the
// YAGS corrector and the RAS once per 16-byte pair the fetch ring (ooo2_fring) asks the I$ for,
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
// A PAIR ENDS AT ITS FIRST KNOWN CTI. The first entry at or after the stream's address ends the
// pair there, taken or not: p_end is that halfword, and the stream continues at the predicted
// target, or at the halfword after the CTI. So a pair holds at most one prediction, the history
// shifts at most once per pair, and a pair's corrector reads share the history they were read with.
// The ring marks the pair's last halfword, and the prediction goes into the queue.
//
// THE QUEUE. One entry per marked halfword, in order: taken, the target, the corrector details
// training needs, and the RAS pointer and history as they stand BEFORE the CTI. Between two marks
// the stream's state does not change, so the queue head's pre-state is the state as of wherever
// the aligner is (the stream's own registers when the queue is empty): it is every instruction's
// snapshot -- what a redirect by that instruction restores -- and what a restart of the stream
// (a rejected prediction, a freeze) goes back to.
//
// THE HISTORY is the directions of the conditionals the BTB knew, in program order, as predicted
// and then corrected: the stream shifts it once per pair that ends at one, and a redirect restores
// the redirecting instruction's own snapshot, plus its outcome when it is such a conditional. It is
// never rebuilt from resolves: control flow resolves out of order, wrong-path branches included.
//
// Everything here is a HINT, never architectural: the mispredict check is the exec-side
// `actual_npc != pred_npc` compare, and the aligner takes a prediction only on a bundle that ends
// on a real branch or jump at the marked halfword (ooo2_frontend, fetch.v). A stale entry, an
// aliased tag or a mis-restored RAS costs a redirect, never a wrong result.
module ooo2_predictor
  #(parameter PCW   = 64,
    parameter BTBB  = 11,            // log2 BTB entries, all banks
    parameter TAGW  = 12,
    parameter TGTW  = 38,            // stored target bits [38:1] (canonical VA, sign-extended)
    parameter GHL   = 11,            // global history length
    parameter RASB  = 3,             // log2 RAS entries
    parameter YBITS = 11,            // log2 corrector entries, all banks
    parameter YTAGW = 8,
    parameter PQB   = 3,             // log2 prediction-queue entries
    // Carried predict details {rsp, ghr, yhit, yctr, yidx, hit, ctr}. An index or a PC-only tag
    // is recomputed at resolve from the CTI's PC and length; yidx rides because it folds the
    // predict-time history.
    parameter PDW   = RASB + GHL + (1+2+YBITS) + (1+2))
   (input  wire                 clk,
    input  wire                 reset,
    // ---- the stream ----
    input  wire                 flush,      // the stream restarts at rst_a...
    input  wire [PCW-1:0]       rst_a,
    input  wire                 rst_np,     // ...predicting nothing in its first pair
    input  wire                 adv,        // the ring took this cycle's pair
    output wire [PCW-1:0]       sa,         // this cycle's pair: its first byte
    output wire [2:0]           sk,         // ...and the halfwords of it before the stream's address
    output wire                 p_cut,      // the pair ends at a known CTI...
    output wire [2:0]           p_end,      // ...at this halfword (7 when it does not)
    output wire                 pq_room,
    output wire [PQB:0]         pq_cnt,
    // ---- the aligner ----
    output wire                 pq_tk,      // the head: predicted taken...
    output wire [PCW-1:0]       pq_tgt,     // ...to here
    output wire [PDW-1:0]       pd_mk,      // details of an instruction ending at the head's mark
    output wire [PDW-1:0]       pd_no,      // ...and of any other
    input  wire                 pop,        // the aligner took the head's prediction, or dropped it
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
    input  wire [PCW-1:0]       res_pc,     // the resolving CTI's own PC...
    input  wire                 res_rvc);   // ...and its length

   localparam RB   = BTBB - 3;              // row bits per BTB bank
   localparam YB   = YBITS - 3;             // row bits per corrector bank
   localparam NB   = 1 << RB;
   localparam NY   = 1 << YB;
   localparam RASN = 1 << RASB;
   localparam PQD  = 1 << PQB;
   // BTB type: 0xx = cond branch, xx = 2-bit bimodal (00 S_N .. 11 S_T); 1xx = uncond
   localparam [2:0] TY_JMP = 3'b100, TY_CALL = 3'b101, TY_RET = 3'b110;

   // ------------------------------------------------------------ BTB: eight banks of 1R1W RAM
   // entry = {tag, type[2:0], target[38:1]}, and NO valid bit: the tag carries it. A never-written
   // entry reads as all zeros, believed only by a halfword whose tag is also 0, and it then says "a
   // strongly-not-taken conditional ends here": a mark the aligner drops when no branch ends there.
   // Configuration INIT zeroes every bank. Bank b holds the halfwords at address bits [3:1] == b, so
   // a pair's halfword k is in bank k.
   localparam EW = TAGW + 3 + TGTW;

   // The tag folds higher bits in, so halfwords that agree in the index but differ above still
   // miss: the kernel's pre-MMU execution at PA 0x802xxxxx aliases its VAs 0xffffffff80xxxxxx in
   // every index bit, and without the fold an early-boot entry steers the paged kernel to a bare
   // address.
   function [TAGW-1:0] btag(input [PCW-1:0] a);
      btag = a[RB+TAGW+3:RB+4] ^ a[RB+2*TAGW+3:RB+TAGW+4] ^ {{(TAGW-1){1'b0}}, a[63]};
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
            ^ a[YBITS+YTAGW:YBITS+1] ^ a[YBITS+2*YTAGW:YBITS+YTAGW+1] ^ {{(YTAGW-1){1'b0}}, a[63]};
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
   // {tk, target, rsp_pre, ghr_pre, yhit, yctr, yidx, ctr}
   localparam QEW = 1 + PCW + RASB + GHL + 1 + 2 + YBITS + 2;
   reg [QEW-1:0]  pq [0:PQD-1];
   reg [PQB-1:0]  pq_rp, pq_wp;
   reg [PQB:0]    pq_n;
   initial begin pq_rp = 0; pq_wp = 0; pq_n = 0; end
   wire [QEW-1:0]  hd     = pq[pq_rp];
   wire [RASB-1:0] hd_rsp = hd[QEW-1-1-PCW -: RASB];
   wire [GHL-1:0]  hd_ghr = hd[QEW-1-1-PCW-RASB -: GHL];
   // the state as of the aligner: the head's pre-state, or the stream's when nothing is queued
   wire            pq_e    = (pq_n == {(PQB+1){1'b0}});
   wire [RASB-1:0] sn_rsp  = pq_e ? ras_ptr : hd_rsp;
   wire [GHL-1:0]  sn_ghr  = pq_e ? ghr     : hd_ghr;
   assign pq_tk   = hd[QEW-1];
   assign pq_tgt  = hd[QEW-2 -: PCW];
   assign pq_cnt  = pq_n;
   assign pq_room = ~pq_n[PQB];
   assign pd_mk   = {sn_rsp, sn_ghr, hd[2+YBITS+2 -: 1+2+YBITS], 1'b1, hd[1:0]};
   assign pd_no   = {sn_rsp, sn_ghr, 1'b0, 2'b01, {YBITS{1'b0}}, 1'b0, 2'b01};

   // A restart puts the stream back to the aligner's state, or to the redirect's; every name here
   // is a register or the queue head, so nothing of the aligner reaches the tables' read address.
   wire [GHL-1:0]  rs_ghr  = rollback ? rb_ghr : sn_ghr;
   wire [RASB-1:0] rs_rsp  = rollback ? rb_rsp : sn_rsp;
   // The corrector's rows are read with the history before this pair's own shift -- a pair of
   // lag -- so this pair's direction never reaches the next pair's read index; the folded history
   // the rows were read with is registered (yh_q) and carried in the index training uses.
   wire [YB-1:0]   yh      = yfold(flush ? rs_ghr : ghr);
   reg  [YB-1:0]   yh_q;
   initial yh_q = {YB{1'b0}};

   // ------------------------------------------------------------ this pair's prediction
   // bq[k]/yq[k] are bank k's rows read for sa_q: halfword k of the pair.
   // The first halfword, in pair order, at or after the stream's address whose entry is its own
   // cuts the pair. Every position evaluates its own hit, corrector hit and direction from
   // registers at once, and the one-hot first hit selects among finished results: nothing waits
   // on the pick but the select.
   wire [EW-1:0]   bq [0:7];
   wire [YEW-1:0]  yq [0:7];
   reg  [7:0]      hv, hy, hd1;              // per position: hit, corrector hit, direction taken
   reg  [7:0]      h1;                       // the first hit, one-hot
   reg  [2:0]      hk;                       // ...its position
   reg  [EW-1:0]   e;                        // ...its entry
   reg  [YEW-1:0]  ce;                       // ...its corrector row
   reg  [PCW-1:0]  ek;
   reg  [YEW-1:0]  ck;
   integer         k;
   always @* begin
      hv = 8'd0;  hy = 8'd0;  hd1 = 8'd0;
      for (k = 0; k < 8; k = k + 1) begin
         ek     = bq[k];
         ck     = yq[k];
         hv[k]  = ~np_q & (k[2:0] >= sk_q) & (ek[EW-1 -: TAGW] == btag(sa_q + 2*k));
         hy[k]  = ~ek[TGTW+2] & (ck[YEW-1 -: YTAGW] == ytagf(sa_q + 2*k));
         hd1[k] = ek[TGTW+2] | (hy[k] ? ck[1] : ek[TGTW+1]);
      end
      h1 = hv & ~(hv << 1) & ~(hv << 2) & ~(hv << 3) & ~(hv << 4) & ~(hv << 5) & ~(hv << 6) & ~(hv << 7);
      hk = 3'd7;  e = {EW{1'b0}};  ce = {YEW{1'b0}};
      for (k = 0; k < 8; k = k + 1)
         if (h1[k]) begin hk = k[2:0];  e = bq[k];  ce = yq[k]; end
   end
   wire [PCW-1:0]  h    = sa_q + {60'd0, hk, 1'b0};             // its address
   wire [2:0]      ety  = e[TGTW +: 3];
   wire [TGTW-1:0] etg  = e[TGTW-1:0];
   wire            cond = ~ety[2];
   wire            yhit = |(h1 & hy);
   wire            dir  = |(h1 & hd1);
   wire            call = (ety == TY_CALL), ret = (ety == TY_RET);
   assign p_cut = |hv;
   assign p_end = hk;
   wire            tk   = |(h1 & hd1);
   wire [PCW-1:0]  ftc  = h + 64'd2;                               // the halfword after the CTI
   wire [PCW-1:0]  tgt  = ret ? ras[ras_ptr] : {{(PCW-TGTW-1){etg[TGTW-1]}}, etg, 1'b0};
   wire [PCW-1:0]  p_nx = tk ? tgt : p_cut ? ftc : sa_q + 64'd16;
   wire [RASB-1:0] rsp_post = (tk & call) ? ras_ptr + 1'b1 : (tk & ret) ? ras_ptr - 1'b1 : ras_ptr;
   wire [GHL-1:0]  ghr_post = (p_cut & cond) ? {ghr[GHL-2:0], dir} : ghr;
   wire [YBITS-1:0] p_yidx  = {h[YB+3:4] ^ yh_q, h[3:1]};
   wire            push     = adv & p_cut & ~flush;
   wire [QEW-1:0]  pq_in    = {tk, tgt, ras_ptr, ghr, yhit, yhit ? ce[1:0] : 2'b01, p_yidx, ety[1:0]};

   // ------------------------------------------------------------------ training
   // One BTB write per resolved CTI (oldest per cycle), into the bank of its last halfword.
   // Bimodal: nudge the carried counter toward the outcome; uncond: record the class (call/return
   // classified at resolve from the executed instruction); target: the resolved taken-target.
   wire [PCW-1:0]  t_last  = res_pc + (res_rvc ? 64'd0 : 64'd2);
   wire            t_hit   = res_pdet[2];
   wire [1:0]      t_ctr   = res_pdet[1:0];
   wire [1:0]      t_base  = t_hit ? t_ctr : (res_taken ? 2'b10 : 2'b01);  // miss -> install weak
   wire [1:0]      t_nudge = res_taken ? ((t_base == 2'b11) ? 2'b11 : t_base + 1'b1)
                                       : ((t_base == 2'b00) ? 2'b00 : t_base - 1'b1);
   wire [2:0]      t_type  = res_cbr  ? {1'b0, t_nudge}
                           : res_ret  ? TY_RET
                           : res_call ? TY_CALL : TY_JMP;
   wire [EW-1:0]   t_ent   = {btag(t_last), t_type, res_tgt[TGTW:1]};
   // The corrector: only a CTI the BTB knew carries a corrector index, so it is allocated when
   // such a conditional's bimodal guess was wrong, and refined whenever it was consulted.
   wire            yc_hit  = res_pdet[3+YBITS+2];
   wire [1:0]      yc_ctr  = res_pdet[3+YBITS+1 -: 2];
   wire [YBITS-1:0] yc_idx = res_pdet[3 +: YBITS];
   wire [1:0]      y_base  = yc_hit ? yc_ctr : (res_taken ? 2'b10 : 2'b01);
   wire [1:0]      y_nudge = res_taken ? ((y_base == 2'b11) ? 2'b11 : y_base + 1'b1)
                                       : ((y_base == 2'b00) ? 2'b00 : y_base - 1'b1);
   wire            bim_pred = t_ctr[1];
   wire            y_wr    = res_v & res_cbr & t_hit & (yc_hit | (bim_pred != res_taken));
   wire [YEW-1:0]  y_ent   = {ytagf(t_last), y_nudge};

   // ------------------------------------------------------------ the next pair, and its rows
   // The stream's next address: the restart's, or this pair's prediction. A pair is the 16 bytes
   // holding it, so every bank reads the same row, the address's bits above the pair.
   wire            mv    = flush | adv;
   wire [PCW-1:0]  na    = flush ? rst_a : p_nx;
   wire [PCW-1:0]  ra    = {na[PCW-1:4], 4'b0000};
   wire [2:0]      rk    = na[3:1];
   wire [RB-1:0]   rr    = ra[RB+3:4];
   wire [YB-1:0]   yr    = ra[YB+3:4];
   genvar gb;
   generate for (gb = 0; gb < 8; gb = gb + 1) begin : bank
      reg [EW-1:0]  btb [0:NB-1];
      reg [YEW-1:0] yc  [0:NY-1];
      reg [EW-1:0]  q;
      reg [YEW-1:0] yqr;
      integer bi;
      initial for (bi = 0; bi < NB; bi = bi + 1) btb[bi] = {EW{1'b0}};
      initial for (bi = 0; bi < NY; bi = bi + 1) yc[bi]  = {YEW{1'b0}};
      initial begin q = {EW{1'b0}}; yqr = {YEW{1'b0}}; end
      assign bq[gb] = q;
      assign yq[gb] = yqr;
      always @(posedge clk) begin
         if (reset | mv) begin q <= btb[rr];  yqr <= yc[yr ^ yh]; end
         if (res_v & (t_last[3:1] == gb))         btb[t_last[RB+3:4]]   <= t_ent;
         if (y_wr  & (yc_idx[2:0] == gb))         yc[yc_idx[YBITS-1:3]] <= y_ent;
      end
   end endgenerate

   always @(posedge clk) begin
      if (reset | mv) begin
         sa_q <= ra;  sk_q <= rk;  yh_q <= yh;
         np_q <= flush & rst_np;
      end
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
            if (push) begin pq[pq_wp] <= pq_in;  pq_wp <= pq_wp + 1'b1; end
            if (pop)  pq_rp <= pq_rp + 1'b1;
            pq_n <= pq_n + {{PQB{1'b0}}, push} - {{PQB{1'b0}}, pop};
         end
      end
   end
   always @(posedge clk) if (!reset) begin
      if (push && !pq_room)
         $fatal(1, "ooo2_predictor: a prediction pushed into a full queue");
      if (pop && pq_e && !flush)
         $fatal(1, "ooo2_predictor: the aligner popped an empty prediction queue");
      if (p_cut && hk < sk_q)
         $fatal(1, "ooo2_predictor: a pair ends (%0d) before the stream's address (%0d)", hk, sk_q);
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
      if (bpt_on & adv & p_cut)
         $display("[BP] c=%0d PRED pair=%h skip=%0d end=%0d type=%b yhit=%b dir=%b tk=%b nx=%h ghr=%h rsp=%0d",
                  bpt_cyc, sa_q, sk_q, p_end, ety, yhit, dir, tk, p_nx, ghr, ras_ptr);
      if (bpt_on & res_v)
         $display("[BP] c=%0d RES pc=%h rvc=%b cbr=%b call=%b ret=%b taken=%b tgt=%h | carried hit=%b ctr=%0d yhit=%b yctr=%0d yidx=%0d | ywr=%b",
                  bpt_cyc, res_pc, res_rvc, res_cbr, res_call, res_ret, res_taken, res_tgt,
                  t_hit, t_ctr, yc_hit, yc_ctr, yc_idx, y_wr);
      if (bpt_on & flush)
         $display("[BP] c=%0d RESTART at=%h rollback=%b ghr<=%h ras_ptr<=%0d", bpt_cyc, rst_a, rollback, rs_ghr, rs_rsp);
   end
`endif
endmodule

`default_nettype wire
