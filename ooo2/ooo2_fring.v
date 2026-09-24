`default_nettype none

// The fetch ring: RS halfword slots holding the instruction stream from the PC on, filled from
// the I$ one 16-byte pair at a time (docs/PLAN-2026-09-24-frontend-stage4.md, increment 1).
//
// Its head IS the PC: fetch reports how pc_q moves each cycle (adv_kind, adv_hw), the head
// advances by the halfwords consumed, and a jump (a predicted-taken fire or a redirect) empties
// the ring and restarts the stream at the new PC, which the iMMU translates that cycle. The
// window fetch aligns is a rotation of the ring from its registered head: no address compare on
// the fetch loop.
//
// THE STREAM. One 16-byte pair per accepted I$ request, 8-byte aligned (16-byte aligned in a
// 4 KiB frame's last chunk: a pair never crosses 4 KiB), running ahead of the PC while the ring
// has room for everything in flight. It stays inside the PC's enclosing page (4 KiB, or 2 MiB
// for a >= 2 MiB leaf), whose PA the iMMU holds for the PC; increment 1d translates on an I$
// miss instead. Answers come back in order, so a flush marks every request then in flight stale
// and that many answers are dropped (rg_drop); the generation in the tag only cross-checks it.
// A fence.i or a mapping change (freeze) empties the ring while it runs.
module ooo2_fring
  #(parameter HW = 8)                       // the window fetch aligns, in halfwords
   (input  wire                    clk,
    input  wire                    reset,
    input  wire                    freeze,  // fence.i, an I$ invalidation, a mapping change
    // how pc_q moves this cycle (fetch.adv_kind, fetch.adv_hw)
    input  wire [2:0]              adv_kind,
    input  wire [$clog2(HW+2)-1:0] adv_hw,
    // the PC and its translation (the iMMU)
    input  wire [63:0]             pc_va,
    input  wire [63:0]             pc_pa,
    input  wire [1:0]              pc_lvl,  // leaf level: 0 = 4 KiB, else >= 2 MiB
    input  wire                    xlate_ok,
    // the window: HW halfwords from the head
    output wire [HW*16-1:0]        win,
    output wire [$clog2(HW+2)-1:0] avail,
    output wire                    ok,
    output wire [1:0]              lvl,     // page size of the bytes held
    // the I$ (rv_icache)
    output wire                    ic_req,
    output wire [63:0]             ic_va,
    output wire [63:0]             ic_pa,
    output wire [5:0]              ic_tag,
    input  wire                    ic_ack,
    input  wire                    ic_valid,
    input  wire [127:0]            ic_data,
    input  wire [5:0]              ic_rtag);

   localparam [2:0] AK_TGT = 3'd3, AK_REDIR = 3'd4;       // fetch.adv_kind: the jumps
   localparam integer RS  = 32;                            // ring slots (halfwords): 64 bytes
   localparam integer RSB = $clog2(RS);
   localparam integer GW  = 3;                             // stream generation
   localparam integer AVW = $clog2(HW+2);
   reg  [15:0]     ring [0:RS-1];
   reg  [RSB-1:0]  rg_head;
   reg  [RSB:0]    rg_cnt, rg_resv;                        // halfwords held; held + in flight
   reg  [GW-1:0]   rg_gen;
   reg  [63:0]     rg_fa;                                  // the stream's next VA
   reg             rg_rst;                                 // restart the stream at the PC
   reg  [1:0]      rg_lvl;                                 // page size of the bytes held
   reg  [63:0]     rg_hva;                                 // the head's VA (checked, never used)
   reg  [2:0]      rg_infl, rg_drop;                       // requests in flight; of them, stale
   initial begin rg_head = 0; rg_cnt = 0; rg_resv = 0; rg_gen = 0; rg_rst = 1'b1; rg_lvl = 2'd0;
                 rg_infl = 0; rg_drop = 0; end
   wire            jump   = (adv_kind == AK_TGT) | (adv_kind == AK_REDIR);
   wire            flush  = jump | freeze;

   // the request: the stream's address, or the PC after a restart
   wire [63:0] fa      = rg_rst ? pc_va : rg_fa;
   wire        big_pg  = (pc_lvl != 2'd0);
   wire        inpg    = big_pg ? (fa[63:21] == pc_va[63:21]) : (fa[63:12] == pc_va[63:12]);
   // a pair never crosses 4 KiB, whatever the page size: rv_icache takes both lines' physical tag
   // from the request's 4 KiB frame
   wire        pg_end  = &fa[11:3];                                     // fa is in a 4 KiB frame's last chunk
   wire [63:0] rq_a    = {fa[63:4], fa[3] & ~pg_end, 3'b000};          // the pair's first byte
   wire [2:0]  rq_skip = {fa[3] & pg_end, fa[2:1]};                    // halfwords before fa
   wire [3:0]  rq_n    = 4'd8 - {1'b0, rq_skip};                       // halfwords it appends
   // NOT gated by a jump: the jump comes out of the whole fetch cone (window -> aligner ->
   // predictor -> fire), and the I$ door must not wait for it. A request taken in a jump's cycle
   // belongs to the old stream and is dropped by the count below like any other in flight.
   assign ic_req = ~freeze & xlate_ok & inpg
                 & ({1'b0, rg_resv} + {{(RSB-3){1'b0}}, rq_n} <= RS[RSB+1:0]);
   assign ic_va  = {25'b0, rq_a[38:0]};                    // VIRTUAL: canonical Sv39 VA (sign ext masked)
   assign ic_pa  = big_pg ? {pc_pa[63:21], rq_a[20:0]} : {pc_pa[63:12], rq_a[11:0]};
   assign ic_tag = {rq_skip, rg_gen};

   // the answer: the halfwords from its skip on, appended at the tail
   wire         rq_acc  = ic_req & ic_ack;
   // an answer landing in a jump's cycle is written and then emptied by the flush's reset of the
   // count: the write enable does not wait for the jump either
   wire         rp_ok   = ic_valid & (rg_drop == 3'd0);
   wire [2:0]   rp_skip = ic_rtag[GW +: 3];
   wire [3:0]   rp_n    = rp_ok ? (4'd8 - {1'b0, rp_skip}) : 4'd0;
   wire [RSB-1:0] tail  = rg_head + rg_cnt[RSB-1:0];
   wire [RSB:0]   adv_w = {{(RSB+1-AVW){1'b0}}, adv_hw};

   // the window: the ring from its head
   genvar gw;
   generate for (gw = 0; gw < HW; gw = gw + 1) begin : wn
      wire [RSB-1:0] ix = rg_head + gw[RSB-1:0];
      assign win[gw*16 +: 16] = ring[ix];
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
         rg_cnt <= 0;  rg_resv <= 0;  rg_head <= 0;  rg_rst <= 1'b1;
         if (flush) rg_gen <= rg_gen + 1'b1;
      end else begin
         rg_head <= rg_head + adv_w[RSB-1:0];
         rg_cnt  <= rg_cnt + {{(RSB-3){1'b0}}, rp_n} - adv_w;
         rg_resv <= rg_resv + (rq_acc ? {{(RSB-3){1'b0}}, rq_n} : {(RSB+1){1'b0}}) - adv_w;
         // the ring is empty while the stream restarts, so nothing is consumed then
         rg_hva  <= (rg_rst & rq_acc) ? pc_va : rg_hva + {{(63-AVW){1'b0}}, adv_hw, 1'b0};
         if (rq_acc) begin rg_fa <= rq_a + 64'd16;  rg_rst <= 1'b0;  rg_lvl <= pc_lvl; end
         for (j = 0; j < 8; j = j + 1)
            if (rp_ok && j >= rp_skip) ring[tail + j[RSB-1:0] - {{(RSB-3){1'b0}}, rp_skip}] <= ic_data[j*16 +: 16];
      end
   end
   // The ring's head is the PC, and what it holds and has in flight fits.
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
   end
endmodule

`default_nettype wire
