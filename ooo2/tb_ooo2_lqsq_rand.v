`timescale 1ns/1ps
`default_nettype none

// tb_ooo2_lqsq_rand: CONSTRAINED-RANDOM ordering test of ooo2_lq + ooo2_sq against a
// program-order model, the bench that would have found the store-seqno wrap and the
// partial-overlap cases in seconds instead of on the board (2026-09-04).
//
// The two queues never move data; the LSU does. What they decide is ORDER: when a load may
// go to memory relative to the stores older than it. So the property checked here is
// exactly that, on every event the queues produce:
//
//   * a load is OFFERED (x_v) only when every older store has its address AND every older
//     store whose bytes overlap the load's has COMMITTED;
//   * an EARLY start (a_sent) happens only when no older store is live at all, and the
//     store queue's ld_older says "an older store is live" if and only if the model has one;
//   * the attributes offered are the ones presented.
//
// The stimulus is a random program-order stream of loads and stores over a handful of
// addresses in one line (sizes 1/2/4/8, so partial overlaps are common), driven the way
// ooo2_core drives the queues: dispatch in order (a load captures the store-seqno the queue
// hands out that cycle, a full store queue stalls a store's dispatch but not a load's),
// addresses in order (M is in-order for memory ops), stores commit in order once every older
// load has landed (the ROB head), loads land in any order, and an occasional wholesale flush
// when nothing is in flight. One action per cycle, chosen at random among those enabled.
//
//   +seed=<n>   (default 1)      +nops=<n>   (default 20000)
// Configuration is stated in the output: NENT=4 both queues, the shipped shape.
module tb;
   localparam NENT=4, IDXB=2, PAW=56, PBITS=9, ROBB=4, NWB=3;
   reg clk=0, reset=1, flush=0;
   always #5 clk = ~clk;

   // ---- load queue ports ----
   reg              lq_d_alloc=0, lq_a_v=0, lq_a_sent=0, lq_x_take=0, lq_l_v=0;
   reg [ROBB-1:0]   lq_d_rob=0;  reg [PBITS-1:0] lq_d_prd=0;  reg [5:0] lq_d_rd=0;  reg lq_d_rd_v=1;
   reg [IDXB:0]     lq_d_sqtag=0;
   reg [IDXB-1:0]   lq_a_idx=0, lq_b_idx=0, lq_l_idx=0;
   reg [PAW-1:0]    lq_a_pa=0;   reg [1:0] lq_a_size=2;  reg lq_a_signed=0, lq_a_fp=0, lq_a_unc=0;
   wire             lq_d_ready, lq_x_v, lq_x_block, lq_b_ok, lq_x_signed, lq_x_fp, lq_x_unc, lq_l_rd_v;
   wire [IDXB-1:0]  lq_d_idx, lq_x_idx;  wire [IDXB:0] lq_q_tag;
   wire [PAW-1:0]   lq_x_pa, lq_l_pa;  wire [1:0] lq_x_size;
   wire [PBITS-1:0] lq_l_prd;  wire [5:0] lq_l_rd;  wire [ROBB-1:0] lq_l_rob;  wire [IDXB:0] lq_occ;
   wire [NENT*PAW-1:0] e_pa;  wire [NENT*2-1:0] e_size;  wire [NENT*(IDXB+1)-1:0] e_tag;
   wire [NENT-1:0]  e_av, e_block;
   // ---- store queue ports ----
   reg              sq_d_alloc=0, sq_a_v=0, sq_a_data_v=0, sq_a_unc=0, sq_c_take=0, sq_k_take=0;
   reg [ROBB-1:0]   sq_d_rob=0;  reg [PBITS-1:0] sq_d_dpreg=0;
   reg [IDXB-1:0]   sq_a_idx=0;  reg [PAW-1:0] sq_a_addr=0;  reg [1:0] sq_a_size=2;  reg [63:0] sq_a_data=0;
   reg [NWB-1:0]    wb_v=0;  reg [NWB*PBITS-1:0] wb_preg=0;  reg [NWB*64-1:0] wb_data=0;
   wire             sq_d_ready, sq_c_v, sq_c_unc, ld_older, sq_kc_v, sq_av_any;
   wire [NENT-1:0]  l_older;   // the registered per-load copy of ld_older (ooo2_core reads this one)
   wire [NENT-1:0]  l_block_q;  // the registered block copy (what ooo2_lq reads in the core)
   wire [ROBB-1:0]  sq_kc_rob;  wire [PAW-1:0] sq_kc_addr;
   wire [IDXB-1:0]  sq_d_idx;  wire [IDXB:0] sq_d_tag;  wire [ROBB-1:0] sq_c_rob;
   wire [PAW-1:0]   sq_c_addr;  wire [63:0] sq_c_data;  wire [1:0] sq_c_size;  wire [IDXB:0] sq_occ;

   ooo2_lq #(.NENT(NENT),.IDXB(IDXB),.PAW(PAW),.PBITS(PBITS),.ROBB(ROBB),.SQIB(IDXB+1)) u_lq
     (.clk(clk),.reset(reset),
      .d_alloc(lq_d_alloc),.d_rob(lq_d_rob),.d_prd(lq_d_prd),.d_rd(lq_d_rd),.d_rd_v(lq_d_rd_v),
      .d_sqtag(lq_d_sqtag),.d_ready(lq_d_ready),.d_idx(lq_d_idx),
      .a_v(lq_a_v),.a_sent(lq_a_sent),.a_idx(lq_a_idx),.a_pa(lq_a_pa),.a_size(lq_a_size),
      .a_signed(lq_a_signed),.a_fp(lq_a_fp),.a_unc(lq_a_unc),
      .e_pa(e_pa),.e_size(e_size),.e_tag(e_tag),.e_av(e_av),.e_block(e_block),.x_block(lq_x_block),
      .q_tag(lq_q_tag),.b_idx(lq_b_idx),.b_ok(lq_b_ok),
      .x_v(lq_x_v),.x_idx(lq_x_idx),.x_pa(lq_x_pa),.x_size(lq_x_size),.x_signed(lq_x_signed),
      .x_fp(lq_x_fp),.x_unc(lq_x_unc),.x_take(lq_x_take),
      .l_v(lq_l_v),.l_idx(lq_l_idx),.l_prd(lq_l_prd),.l_rd(lq_l_rd),.l_rd_v(lq_l_rd_v),.l_rob(lq_l_rob),
      .l_pa(lq_l_pa),.occupancy(lq_occ),.flush(flush));

   ooo2_sq #(.NENT(NENT),.IDXB(IDXB),.PAW(PAW),.PBITS(PBITS),.ROBB(ROBB),.NWB(NWB),.LQN(NENT),.LQIB(IDXB)) u_sq
     (.clk(clk),.reset(reset),
      .d_alloc(sq_d_alloc),.d_rob(sq_d_rob),.d_dpreg(sq_d_dpreg),.d_ready(sq_d_ready),.d_idx(sq_d_idx),.d_tag(sq_d_tag), .av_any(sq_av_any),
      .a_v(sq_a_v),.a_idx(sq_a_idx),.a_addr(sq_a_addr),.a_size(sq_a_size),.a_unc(sq_a_unc),
      .a_data_v(sq_a_data_v),.a_data(sq_a_data),
      .wb_v(wb_v),.wb_preg(wb_preg),.wb_data(wb_data),
      .c_v(sq_c_v),.c_rob(sq_c_rob),.c_addr(sq_c_addr),.c_data(sq_c_data),.c_size(sq_c_size),.c_unc(sq_c_unc),
      .c_take(sq_c_take),
      .kc_v(sq_kc_v),.kc_rob(sq_kc_rob),.kc_addr(sq_kc_addr),.k_take(sq_k_take),
      .l_pa(e_pa),.l_size(e_size),.l_tag(e_tag),.l_av(e_av),
      .l_fill(lq_a_v),.l_fill_ix(lq_a_idx),.l_fill_pa(lq_a_pa),.l_fill_size(lq_a_size),
      .l_block(e_block),.l_block_q(l_block_q),.l_older(l_older),.ld_tag(lq_q_tag),.ld_older(ld_older),
      .occupancy(sq_occ),.flush(flush));

   // ------------------------------------------------------------- the program-order model
   // st: 0 dispatched (no address) | 1 address known (store: uncommitted; load: waiting)
   //     2 store COMMITTED (released by the ROB, still in the queue) / load in flight
   //     3 load landed | 4 dead (flushed) | 5 store DRAINED (the LSU took it)
   // A committed store is still in the queue: younger loads that overlap it wait for the
   // drain, and a flush keeps it. That is the senior store queue (plan item 3).
   localparam MAXOPS = 65536;
   reg           is_st [0:MAXOPS-1];
   reg [PAW-1:0] addr  [0:MAXOPS-1];
   reg [1:0]     sz    [0:MAXOPS-1];
   reg           sgn   [0:MAXOPS-1];
   reg           unc   [0:MAXOPS-1];
   reg [2:0]     st    [0:MAXOPS-1];
   reg [IDXB-1:0] qix  [0:MAXOPS-1];        // the entry the op holds in its queue
   integer lq_op [0:NENT-1];                // queue entry -> op, for the candidate and landings
   integer nops, seed, i, j, op, act, n_early, n_reord, n_full, n_take, n_flush, n_err, n_cyc;
   integer p_disp, p_addr, p_scmt;          // program-order cursors: dispatch, address, store commit
   integer live_ld_inflight, verbose, vfrom, vto;

   // xorshift32 on a module-level state: $random(seed) called through a function leaves
   // the seed untouched under Verilator, and $urandom ignores its argument -- one gave
   // every seed the same run, the other the same action every cycle.
   reg [31:0] xs;
   function [31:0] rnd(input dummy);
      begin
         xs = xs ^ (xs << 13); xs = xs ^ (xs >> 17); xs = xs ^ (xs << 5);
         rnd = xs;
      end
   endfunction
   function ovl(input [PAW-1:0] a, input [1:0] as, input [PAW-1:0] b, input [1:0] bs);
      reg [PAW:0] ae, be;
      begin ae = a + (1 << as); be = b + (1 << bs); ovl = !((be <= a) || (ae <= b)); end
   endfunction
   task step; begin @(posedge clk); #1; end endtask
   task fail(input [511:0] what, input integer o);
      begin
         $display("FAIL: %0s (op %0d %0s addr=%h sz=%0d, cycle %0d)", what, o,
                  is_st[o] ? "store" : "load", addr[o], 1 << sz[o], n_cyc);
         n_err = n_err + 1;
      end
   endtask
   // an older store that is live in the queue (allocated, uncommitted, not dead)?
   function older_live(input integer o);
      integer k; begin older_live = 0;
         for (k = 0; k < o; k = k + 1) if (is_st[k] && (st[k] == 0 || st[k] == 1 || st[k] == 2)) older_live = 1;
      end
   endfunction

   initial begin
      if (!$value$plusargs("seed=%d", seed)) seed = 1;
      xs = (seed == 0) ? 32'h9E37_79B9 : seed[31:0] * 32'h9E37_79B9;   // spread small seeds apart
      if (!$value$plusargs("nops=%d", nops)) nops = 20000;
      verbose = $test$plusargs("verbose");
      if (!$value$plusargs("vfrom=%d", vfrom)) vfrom = 0;
      if (!$value$plusargs("vto=%d", vto)) vto = 1 << 30;
      if (nops > MAXOPS) nops = MAXOPS;
      $display("tb_ooo2_lqsq_rand: NENT=%0d both queues, PAW=%0d, seed=%0d, nops=%0d", NENT, PAW, seed, nops);
      // the program: 45% stores, addresses in one 64-byte line at 0x1000, natural alignment
      for (i = 0; i < nops; i = i + 1) begin
         is_st[i] = (rnd(0) % 100) < 45;
         sz[i]    = rnd(0) % 4;
         addr[i]  = 56'h1000 + ((rnd(0) % (64 >> sz[i])) << sz[i]);
         if ((rnd(0) % 16) == 0) addr[i] = addr[i] + 56'h40;   // the next line, rarely
         sgn[i]   = rnd(0) % 2;  unc[i] = (rnd(0) % 32) == 0;
         st[i]    = 0;
      end
      n_early = 0; n_reord = 0; n_full = 0; n_take = 0; n_flush = 0; n_err = 0; n_cyc = 0;
      p_disp = 0; p_addr = 0; p_scmt = 0; live_ld_inflight = 0;
      repeat (3) @(posedge clk); #1; reset = 0; step;

      while ((p_disp < nops || lq_occ != 0 || sq_occ != 0) && n_err < 8 && n_cyc < 40*nops) begin
         n_cyc = n_cyc + 1;
         act = rnd(0) % 8;
         lq_d_alloc = 0; lq_a_v = 0; lq_a_sent = 0; lq_x_take = 0; lq_l_v = 0;
         sq_d_alloc = 0; sq_a_v = 0; sq_a_data_v = 0; sq_c_take = 0; sq_k_take = 0; flush = 0;
         if (verbose && n_cyc >= vfrom && n_cyc < vto)
            $display("  c=%0d act=%0d p_disp=%0d p_addr=%0d lq_occ=%0d sq_occ=%0d x_v=%b x_idx=%0d(op %0d) c_v=%b",
                     n_cyc, act, p_disp, p_addr, lq_occ, sq_occ, lq_x_v, lq_x_idx, lq_op[lq_x_idx], sq_c_v);
         case (act)
           0, 1: begin // ---- dispatch the next op, in program order ----
              if (p_disp < nops) begin
                 op = p_disp;
                 if (is_st[op] && sq_d_ready) begin
                    sq_d_alloc = 1; sq_d_rob = op; sq_d_dpreg = 9'd7; qix[op] = sq_d_idx;
                    p_disp = p_disp + 1;
                 end else if (!is_st[op] && lq_d_ready) begin
                    lq_d_alloc = 1; lq_d_rob = op; lq_d_prd = 9'd40 + op % 100; lq_d_sqtag = sq_d_tag;
                    qix[op] = lq_d_idx; lq_op[lq_d_idx] = op;
                    if (sq_occ == NENT) n_full = n_full + 1;   // a load dispatched against a FULL store queue
                    p_disp = p_disp + 1;
                 end
              end
           end
           2, 3: begin // ---- the next address, in program order ----
              if (p_addr < p_disp) begin
                 op = p_addr;
                 if (is_st[op]) begin
                    sq_a_v = 1; sq_a_idx = qix[op]; sq_a_addr = addr[op]; sq_a_size = sz[op];
                    sq_a_unc = unc[op]; sq_a_data_v = 1; sq_a_data = {32'hDA7A, op[31:0]};
                    st[op] = 1; p_addr = p_addr + 1;
                 end else begin
                    lq_b_idx = qix[op]; #1;
                    // the early start is offered only to the untranslated candidate with no
                    // older store live -- and the store queue must agree with the model
                    if (lq_b_ok && (ld_older !== older_live(op)))
                       fail(ld_older ? "ld_older says an older store is live; the model has none"
                                     : "ld_older says none; the model has an older live store", op);
                    lq_a_v = 1; lq_a_idx = qix[op]; lq_a_pa = addr[op]; lq_a_size = sz[op];
                    lq_a_signed = sgn[op]; lq_a_unc = unc[op]; lq_a_fp = 0;
                    if (lq_b_ok && !ld_older && (rnd(0) % 4) != 0) begin
                       lq_a_sent = 1; st[op] = 2; n_early = n_early + 1; live_ld_inflight = live_ld_inflight + 1;
                       if (older_live(op)) fail("early start past a live older store", op);
                    end else st[op] = 1;
                    p_addr = p_addr + 1;
                 end
              end
           end
           4: begin // ---- take the candidate: the property under test ----
              if (lq_x_v) begin
                 op = lq_op[lq_x_idx];
                 if (st[op] != 1) begin
                    $display("  x_idx=%0d lq_op={%0d,%0d,%0d,%0d} st[op]=%0d", lq_x_idx, lq_op[0], lq_op[1], lq_op[2], lq_op[3], st[op]);
                    fail("candidate offered is not a waiting load", op);
                 end
                 if (lq_x_pa !== addr[op] || lq_x_size !== sz[op] || lq_x_signed !== sgn[op] || lq_x_unc !== unc[op])
                    fail("candidate attributes differ from those presented", op);
                 for (j = 0; j < op; j = j + 1) if (is_st[j] && st[j] != 4) begin
                    if (st[j] == 0) fail("offered with an older store's address unknown", op);
                    if ((st[j] == 1 || st[j] == 2) && ovl(addr[j], sz[j], addr[op], sz[op]))
                       fail("offered past an undrained older store that overlaps it", op);
                    if (st[j] == 1 || st[j] == 2) n_reord = n_reord + 1;
                 end
                 lq_x_take = 1; st[op] = 2; n_take = n_take + 1; live_ld_inflight = live_ld_inflight + 1;
              end
           end
           5: begin // ---- land a random in-flight load ----
              if (live_ld_inflight > 0) begin
                 j = rnd(0) % NENT;
                 for (i = 0; i < NENT; i = i + 1) begin
                    op = lq_op[(j + i) % NENT];
                    if (op >= 0 && !is_st[op] && st[op] == 2) begin
                       lq_l_v = 1; lq_l_idx = (j + i) % NENT; st[op] = 3;
                       live_ld_inflight = live_ld_inflight - 1; lq_op[(j + i) % NENT] = -1; i = NENT;
                    end
                 end
              end
           end
           6: begin // ---- RELEASE the first unreleased store once nothing older can restart ----
              // the bench plays the ROB's irrevocable pointer: every older op done (loads
              // landed, stores released)
              while (p_scmt < p_disp && (!is_st[p_scmt] || st[p_scmt] == 2 || st[p_scmt] == 5 || st[p_scmt] == 4)) p_scmt = p_scmt + 1;
              if (p_scmt < p_disp && sq_kc_v) begin
                 op = p_scmt; j = 1;
                 for (i = 0; i < op; i = i + 1)
                    if (st[i] != 4 && ((!is_st[i] && st[i] != 3) || (is_st[i] && st[i] != 2 && st[i] != 5))) j = 0;
                 if (j && st[op] == 1) begin
                    if (sq_kc_addr !== addr[op]) fail("the first unreleased entry is not the model's oldest unreleased store", op);
                    sq_k_take = 1; st[op] = 2;
                 end
              end
              // ...and DRAIN the head when it is committed: the LSU taking it, any cycle
              if (sq_c_v && (rnd(0) % 2)) begin
                 op = -1;
                 for (i = 0; i < p_disp; i = i + 1) if (op < 0 && is_st[i] && st[i] == 2) op = i;
                 if (op < 0) fail("a committed head to drain but the model has no committed store", 0);
                 else begin
                    if (sq_c_addr !== addr[op]) fail("draining head is not the model's oldest committed store", op);
                    sq_c_take = 1; st[op] = 5;
                 end
              end
           end
           7: begin // ---- a wholesale flush, rarely, only with nothing in flight ----
              if (live_ld_inflight == 0 && (rnd(0) % 64) == 0 && p_disp > 0) begin
                 flush = 1; n_flush = n_flush + 1;
                 // committed stores (2) survive a flush and drain later; the rest die
                 for (i = 0; i < p_disp; i = i + 1)
                    if ((is_st[i] && st[i] < 2) || (!is_st[i] && st[i] < 3)) st[i] = 4;
                 for (i = 0; i < NENT; i = i + 1) lq_op[i] = -1;
                 // the dead ops are re-run: the program restarts at the first of them
                 for (i = 0; i < p_disp; i = i + 1) if (st[i] == 4) begin
                    for (j = i; j < p_disp; j = j + 1) st[j] = 0;
                    p_disp = i; i = p_disp + 1; j = p_disp;   // and the cursors follow
                 end
                 if (p_addr > p_disp) p_addr = p_disp;
                 if (p_scmt > p_disp) p_scmt = p_disp;
              end
           end
         endcase
         step;
      end
      lq_d_alloc = 0; lq_a_v = 0; lq_a_sent = 0; lq_x_take = 0; lq_l_v = 0;
      sq_d_alloc = 0; sq_a_v = 0; sq_c_take = 0; sq_k_take = 0; flush = 0; step;
      if (n_cyc >= 40*nops) begin
         $display("FAIL: the run wedged (cycle %0d, p_disp=%0d p_addr=%0d p_scmt=%0d lq=%0d sq=%0d x_v=%b x_block=%b c_v=%b inflight=%0d)",
                  n_cyc, p_disp, p_addr, p_scmt, lq_occ, sq_occ, lq_x_v, lq_x_block, sq_c_v, live_ld_inflight);
         for (i = (p_scmt > 6 ? p_scmt - 6 : 0); i < p_disp; i = i + 1)
            $display("   op %0d %s addr=%h sz=%0d st=%0d qix=%0d", i, is_st[i] ? "st" : "ld", addr[i], 1 << sz[i], st[i], qix[i]);
         n_err = n_err + 1;
      end
      $display("---- tb_ooo2_lqsq_rand: ops=%0d cycles=%0d takes=%0d early=%0d reordered-past-older-store=%0d full-queue-loads=%0d flushes=%0d errors=%0d",
               nops, n_cyc, n_take, n_early, n_reord, n_full, n_flush, n_err);
      if (n_err != 0) begin $display("LQSQ-RAND FAIL"); $fatal(1, "tb_ooo2_lqsq_rand FAILED"); end
      $display("LQSQ-RAND PASS");
      $finish;
   end
   initial for (i = 0; i < NENT; i = i + 1) lq_op[i] = -1;
endmodule
`default_nettype wire
