`default_nettype none

// The fetch ring: RS halfword slots holding the instruction stream from the PC on, filled from
// the I$ one 16-byte pair at a time along the path the predictor steers
// (docs/PLAN-2026-09-24-frontend-stage4.md, increment 1).
//
// Its head IS the PC: fetch reports how many halfwords it consumed each cycle (adv_hw), and the
// head advances by them. A predicted-taken branch does not empty the ring: the predictor already
// steered the stream to its target, so the target's halfwords follow the branch's in the ring. A
// restart (a redirect, or a prediction the aligner rejected) empties it and restarts the stream at
// the next PC, which the iMMU translates that cycle. The window fetch aligns is a rotation of the
// ring from its registered head, with a mark on each halfword that ends a pair at a CTI the
// predictor knows.
//
// THE STREAM. One pair per accepted I$ request, 16-byte aligned (so a pair never crosses 4 KiB),
// running ahead of the PC while the ring has room for a whole pair and the predictor has room for
// the pair's prediction. The
// predictor (ooo2_predictor) holds the stream's address (sa, sk) and looks the pair up as it is
// requested: a pair ends at the first CTI it knows at or after the stream's address, and the
// stream goes on at the predicted next address.
// The stream stays inside the PC's enclosing page (4 KiB, or 2 MiB for a >= 2 MiB leaf), whose PA
// the iMMU holds for the PC; increment 1d translates on an I$ miss instead. Answers come back in
// order, so a flush marks every request then in flight stale and that many answers are dropped
// (rg_drop); the generation in the tag only cross-checks it. A fence.i or a mapping change
// (freeze) empties the ring while it runs.
module ooo2_fring
  #(parameter HW  = 8,                      // the window fetch aligns, in halfwords
    parameter PQB = 3)                      // the predictor's queue (for the mark count check)
   (input  wire                    clk,
    input  wire                    reset,
    input  wire                    freeze,  // fence.i, an I$ invalidation, a mapping change, a rejected mark
    input  wire                    restart, // a redirect: empty and restart the stream at rst_a
    // what fetch consumed, and the prediction it took (for the head's VA)
    input  wire [$clog2(HW+2)-1:0] adv_hw,
    input  wire                    pop,
    input  wire                    drop,    // clear the head's mark
    input  wire                    pq_tk,
    input  wire [63:0]             pq_tgt,
    // the PC and its translation (the iMMU)
    input  wire [63:0]             pc_va,
    input  wire [63:0]             rst_a,   // where a restarted stream begins: the redirect's target, else the PC
    input  wire [63:0]             pc_pa,
    input  wire [1:0]              pc_lvl,  // leaf level: 0 = 4 KiB, else >= 2 MiB
    input  wire                    xlate_ok,
    // the window: HW halfwords from the head, and their marks
    output wire [HW*16-1:0]        win,
    output wire [HW-1:0]           mk,
    output wire [$clog2(HW+2)-1:0] avail,
    output wire                    ok,
    output wire [1:0]              lvl,     // page size of the bytes held
    // the predictor: the stream's pair, where it ends, and the ring taking it
    input  wire [63:0]             sa,      // the pair's first byte
    input  wire [2:0]              sk,      // the halfwords of it before the stream's address
    input  wire                    p_cut,
    input  wire [2:0]              p_end,
    output wire                    pr_adv,
    input  wire                    pq_room,
    input  wire [PQB:0]            pq_cnt,
    // the I$ (rv_icache)
    output wire                    ic_req,
    output wire [63:0]             ic_va,
    output wire [63:0]             ic_pa,
    output wire [9:0]              ic_tag,
    input  wire                    ic_ack,
    input  wire                    ic_valid,
    input  wire [127:0]            ic_data,
    input  wire [9:0]              ic_rtag);

   localparam integer RS  = 32;                            // ring slots (halfwords): 64 bytes
   localparam integer RSB = $clog2(RS);
   localparam integer GW  = 3;                             // stream generation
   localparam integer AVW = $clog2(HW+2);
   reg  [15:0]     ring [0:RS-1];
   reg  [RS-1:0]   rmk;                                    // the slot ends a pair at a known CTI
   reg  [RSB-1:0]  rg_head;
   reg  [RSB:0]    rg_cnt, rg_resv;                        // halfwords held; held + in flight
   reg  [GW-1:0]   rg_gen;
   reg  [1:0]      rg_lvl;                                 // page size of the bytes held
   reg  [63:0]     rg_hva;                                 // the head's VA (checked, never used)
   reg  [2:0]      rg_infl, rg_drop;                       // requests in flight; of them, stale
   reg  [PQB:0]    rg_mfl;                                 // marked pairs in flight (checked)
   initial begin rg_head = 0; rg_cnt = 0; rg_resv = 0; rg_gen = 0;
                 rg_lvl = 2'd0; rg_infl = 0; rg_drop = 0; rg_mfl = 0; rmk = 0; end
   wire            flush  = restart | freeze;

   // the request: the stream's pair, inside the PC's enclosing page (a pair never crosses 4 KiB,
   // whatever the page size: rv_icache takes both lines' physical tag from the request's frame)
   wire        big_pg  = (pc_lvl != 2'd0);
   wire        inpg    = big_pg ? (sa[63:21] == pc_va[63:21]) : (sa[63:12] == pc_va[63:12]);
   wire [3:0]  rq_n    = {1'b0, p_end} - {1'b0, sk} + 4'd1;            // halfwords it appends
   // NOT gated by a restart: a restart comes out of the whole fetch cone (window -> aligner ->
   // fire), and the I$ door must not wait for it. A request taken in a restart's cycle belongs to
   // the old stream and is dropped by the count below like any other in flight. The room it asks
   // for is a whole pair's, so the door does not wait for the prediction's cut either. The ring
   // holds a window and a pair, so a bundle waiting for its bytes (the aligner's bytes_late) always
   // leaves room for the pair that brings them.
   initial if (RS < HW + 8) $fatal(1, "ooo2_fring: %0d slots cannot hold a %0d-halfword window and a pair", RS, HW);
   assign ic_req = ~freeze & xlate_ok & inpg & pq_room & (rg_resv <= RS[RSB:0] - 5'd8);
   assign ic_va  = {25'b0, sa[38:0]};                      // VIRTUAL: canonical Sv39 VA (sign ext masked)
   assign ic_pa  = big_pg ? {pc_pa[63:21], sa[20:0]} : {pc_pa[63:12], sa[11:0]};
   assign ic_tag = {p_cut, p_end, sk, rg_gen};
   wire   rq_acc = ic_req & ic_ack;
   assign pr_adv = rq_acc;

   // the answer: its halfwords from the skip to the end, appended at the tail, the last marked
   // when its pair ended at a known CTI. An answer landing in a restart's cycle is written and then
   // emptied by the flush's reset of the count: the write enable does not wait for the restart.
   wire         rp_ok   = ic_valid & (rg_drop == 3'd0);
   wire         rp_mk   = ic_rtag[GW+6];
   wire [2:0]   rp_end  = ic_rtag[GW+3 +: 3];
   wire [2:0]   rp_skip = ic_rtag[GW +: 3];
   wire [3:0]   rp_n    = rp_ok ? ({1'b0, rp_end} - {1'b0, rp_skip} + 4'd1) : 4'd0;
   wire [RSB-1:0] tail  = rg_head + rg_cnt[RSB-1:0];
   wire [RSB:0]   adv_w = {{(RSB+1-AVW){1'b0}}, adv_hw};

   // the window: the ring from its head; a mark counts only on a halfword held
   genvar gw;
   generate for (gw = 0; gw < HW; gw = gw + 1) begin : wn
      wire [RSB-1:0] ix = rg_head + gw[RSB-1:0];
      assign win[gw*16 +: 16] = ring[ix];
      assign mk[gw]           = rmk[ix] & (rg_cnt > gw);
   end endgenerate
   assign avail = (rg_cnt >= HW[RSB:0]) ? HW[AVW-1:0] : rg_cnt[AVW-1:0];
   assign ok    = ~freeze & (rg_cnt != {(RSB+1){1'b0}});
   assign lvl   = rg_lvl;

   integer j;
   always @(posedge clk) begin
      // requests in flight, and how many of them a flush made stale (answers come in order)
      rg_infl <= reset ? 3'd0 : rg_infl + {2'd0, rq_acc} - {2'd0, ic_valid};
      rg_drop <= reset ? 3'd0
               : flush ? rg_infl + {2'd0, rq_acc} - {2'd0, ic_valid}
               : rg_drop - {2'd0, ic_valid & (rg_drop != 3'd0)};
      if (reset | flush) begin
         rg_cnt <= 0;  rg_resv <= 0;  rg_head <= 0;  rg_mfl <= 0;
         rg_hva <= rst_a;
         if (flush) rg_gen <= rg_gen + 1'b1;
      end else begin
         rg_head <= rg_head + adv_w[RSB-1:0];
         rg_cnt  <= rg_cnt + {{(RSB-3){1'b0}}, rp_n} - adv_w;
         rg_resv <= rg_resv + (rq_acc ? {{(RSB-3){1'b0}}, rq_n} : {(RSB+1){1'b0}}) - adv_w;
         rg_mfl  <= rg_mfl + {{PQB{1'b0}}, rq_acc & p_cut} - {{PQB{1'b0}}, rp_ok & rp_mk};
         rg_hva  <= (pop & pq_tk) ? pq_tgt : rg_hva + {{(63-AVW){1'b0}}, adv_hw, 1'b0};
         if (rq_acc) rg_lvl <= pc_lvl;
         if (drop) rmk[rg_head] <= 1'b0;
         for (j = 0; j < 8; j = j + 1)
            if (rp_ok && j >= rp_skip && j <= rp_end) begin
               ring[tail + j[RSB-1:0] - {{(RSB-3){1'b0}}, rp_skip}] <= ic_data[j*16 +: 16];
               rmk [tail + j[RSB-1:0] - {{(RSB-3){1'b0}}, rp_skip}] <= rp_mk & (j == rp_end);
            end
      end
   end
   // The ring's head is the PC, what it holds and has in flight fits, and every mark held or in
   // flight has its prediction queued.
   integer k;
   reg [PQB+1:0] n_mk;
   always @* begin
      n_mk = 0;
      for (k = 0; k < RS; k = k + 1)
         if (k < rg_cnt) n_mk = n_mk + {{(PQB+1){1'b0}}, rmk[rg_head + k[RSB-1:0]]};
   end
   always @(posedge clk) if (!reset && !flush) begin
      if (rg_cnt != 0 && rg_hva != pc_va)
         $fatal(1, "ooo2_fring: the ring's head %h is not the PC %h", rg_hva, pc_va);
      if ({1'b0, adv_hw} > rg_cnt)
         $fatal(1, "ooo2_fring: fetch consumed %0d halfwords of a ring holding %0d", adv_hw, rg_cnt);
      if (rg_cnt + rp_n > rg_resv)
         $fatal(1, "ooo2_fring: the ring holds more (%0d) than it reserved (%0d)", rg_cnt + rp_n, rg_resv);
      if (rp_ok && ic_rtag[GW-1:0] != rg_gen)
         $fatal(1, "ooo2_fring: a kept I$ answer is from generation %0d, the stream is %0d", ic_rtag[GW-1:0], rg_gen);
      if (ic_valid && rg_infl == 3'd0)
         $fatal(1, "ooo2_fring: an I$ answer with no request in flight");
      if (n_mk + {1'b0, rg_mfl} != {1'b0, pq_cnt})
         $fatal(1, "ooo2_fring: %0d marks held and %0d in flight, but %0d predictions queued", n_mk, rg_mfl, pq_cnt);
   end
endmodule

`default_nettype wire
