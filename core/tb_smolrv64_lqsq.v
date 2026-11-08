`default_nettype none
// Standalone check for smolrv64_lq AND smolrv64_sq TOGETHER, wired as smolrv64_core wires them.
//
// The two used to have separate benches, each driving the other's side of the alias test
// by hand. 69f1ac6a turned that test into a CONFLICT MATRIX that lives in the store queue
// and is fed by the load queue's entries, so the interface between the two modules IS the
// thing under test, and one bench that owns both is the only honest shape. The old benches
// stopped compiling at that commit and nobody noticed (gate.sh now runs every
// run-*-tb.sh and treats a build failure as a failure).
//
// What is checked, in the order docs/history/HANDOFF-2026-09-03-multiple-loads.md lists it:
//   * a load held because an older store MAY alias it (no address yet, or an overlapping
//     address), released when that store's address disambiguates or the store commits
//   * the no-older-store case: ld_older low, the candidate offered the cycle its address is in
//   * partial overlap and size mismatch, in BOTH orders of address arrival (row and column)
//   * a store commit and a load's address arriving in the same cycle
//   * flush while entries are blocked
//   * the store queue's data path: the snoop, the early writeback, the same-cycle writeback,
//     and the LANDING BYPASS against a MOVING writeback bus (60481808)
//   * the load queue's ordering: no overtaking of an unfilled older entry, two in flight,
//     landing by index, the early start (a_sent)
// Configuration stated in the output: NENT=4 both queues, small, so the directed cases fill and wrap them.
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
   wire             lq_x_head;   // the LQ candidate's head compare (2026-09-17); unused here
   wire             lq_x_devwait; wire [NENT-1:0] sq_l_block_unk_q;   // counter-only outputs (2026-09-17)
   wire [3:0]       lq_bfree;  wire lq_bp;  wire [IDXB:0] sq_free;
   wire             lq_uf_any, sq_uf_any;
   wire [NENT*PBITS-1:0] lq_e_dprd;
   wire             lq_pk_v, sq_pk_v;
   wire [IDXB-1:0]  lq_pk_idx, sq_pk_idx;
   wire [7:0]       lq_pk_seq, sq_pk_seq;
   wire             lq_d_ready, lq_x_v, lq_x_block, lq_b_ok, lq_x_signed, lq_x_fp, lq_x_unc, lq_l_rd_v;
   wire [IDXB-1:0]  lq_d_idx, lq_x_idx;  wire [IDXB:0] lq_q_tag;
   wire [PAW-1:0]   lq_x_pa, lq_l_pa;  wire [1:0] lq_x_size;
   wire [PBITS-1:0] lq_l_prd;  wire [5:0] lq_l_rd;  wire [ROBB-1:0] lq_l_rob;  wire [IDXB:0] lq_occ;
   wire [NENT*12-1:0] e_off;
   // the walker and trap ports: every fill here is translated, so they stay quiet
   wire             lk_v, sk_v, lf_v, sf_v;
   wire [IDXB-1:0]  lf_idx;
   wire             lf_rp;
   wire [NENT-1:0]  e_wt, e_acc, sq_unk_q, sq_viol, sq_res_c, sq_res_a;
   wire [NENT-1:0]  sq_l_cancel;   // an early start an older store blocks, cancelled a cycle later
   wire [NENT*8-1:0] e_pch;
   wire [IDXB-1:0]  lk_idx, sk_idx;
   wire [38:0]      lk_va, sk_va, lf_pc, sf_pc;
   wire [ROBB-1:0]  lf_rob, sf_rob;
   wire [3:0]       lf_fc, sf_fc;
   wire [7:0]       lf_seq, sf_seq;
   always @(posedge clk) if (!reset && (lk_v | sk_v | lf_v | sf_v))
      $fatal(1, "FAIL: a walker or trap request from translated fills (lk %b sk %b lf %b sf %b)", lk_v, sk_v, lf_v, sf_v);  wire [NENT*2-1:0] e_size;  wire [NENT*(IDXB+1)-1:0] e_tag;
   wire [NENT-1:0]  e_av, e_block;
   wire [NENT-1:0]  e_older;   // the registered per-load copy of ld_older (what smolrv64_core reads)
   wire [NENT-1:0]  l_block_q;  // the registered block copy (what smolrv64_lq reads in the core)
   // ---- store queue ports ----
   reg              sq_d_alloc=0, sq_a_v=0, sq_d_rdy=1, sq_a_unc=0, sq_c_take=0, sq_k_take=0;
   // the register file the store queue's data read answers from; a test that dispatches a store
   // whose data a writeback brings later clears sq_d_rdy for it
   reg  [63:0]      rfv [0:(1<<PBITS)-1];
   reg  [PBITS-1:0] dp_of [0:NENT-1];
   wire             sq_r_v;  wire [PBITS-1:0] sq_r_preg;
   reg [ROBB-1:0]   sq_d_rob=0;  reg [PBITS-1:0] sq_d_dpreg=0;
   reg [IDXB-1:0]   sq_a_idx=0;  reg [PAW-1:0] sq_a_addr=0;  reg [1:0] sq_a_size=2;
   reg [NWB-1:0]    wb_v=0;  reg [NWB*PBITS-1:0] wb_preg=0;  reg [NWB*64-1:0] wb_data=0;
   // a writeback IS the register file's write: the model takes it, as the core's does
   integer wbi;
   always @(posedge clk) for (wbi = 0; wbi < NWB; wbi = wbi + 1)
      if (wb_v[wbi]) rfv[wb_preg[wbi*PBITS +: PBITS]] <= wb_data[wbi*64 +: 64];
   wire             sq_av_any, lq_av_any;
   wire             sq_d_ready, sq_c_v, sq_c_unc, ld_older, sq_kc_v;
   wire [ROBB-1:0]  sq_kc_rob;  wire [PAW-1:0] sq_kc_addr;
   wire [63:0]      sq_kc_data; wire [1:0] sq_kc_size;
   wire [IDXB-1:0]  sq_d_idx;  wire [IDXB:0] sq_d_tag;  wire [ROBB-1:0] sq_c_rob;
   wire [PAW-1:0]   sq_c_addr;  wire [63:0] sq_c_data;  wire [1:0] sq_c_size;  wire [IDXB:0] sq_occ;

   wire [NENT-1:0] lq_kmask_nc, sq_kmask_nc;   // the kill is not exercised here
   wire            sq_c_fast_nc;               // the core's store register is not modelled: every drain is a handoff and a free
   wire [NENT*28-1:0] e_vpn_nc;  wire x_fwd_nc;  wire [NENT-1:0] l_fwd_nc;  wire [63:0] f_data_nc;  wire [2:0] f_sbo_nc;   // forwarding is off here
   wire lf_new;                // an entry flagged for replay this cycle (the core's one-cycle hold)
   smolrv64_lq #(.LDPRE(0),.DRAM_BASE(64'd0),.NENT(NENT),.IDXB(IDXB),.PAW(PAW),.PBITS(PBITS),.ROBB(ROBB),.SQIB(IDXB+1),.NB(2)) u_lq
     (.clk(clk),.reset(reset),
      .d_alloc(lq_d_alloc),.d_pc(39'd0),.d_seq(8'd0),.d_rob(lq_d_rob),.d_prd(lq_d_prd),.d_rd(lq_d_rd),.d_rd_v(lq_d_rd_v),
      .d_sqtag(lq_d_sqtag),.d_wt(1'b1),.d_rank(1'b0),.d_ready(lq_d_ready),.d_bfree(lq_bfree),.d_bp(lq_bp),.d_idx(lq_d_idx),
      .a_v(lq_a_v),.a_sent(lq_a_sent),.a_cancel(sq_l_cancel),.a_idx(lq_a_idx),.a_pa(lq_a_pa),.a_va(39'(lq_a_pa)), .a_tv(1'b1),.a_flt(1'b0),.a_fc(4'd0),.a_size(lq_a_size),
      .a_signed(lq_a_signed),.a_fp(lq_a_fp),.a_unc(lq_a_unc),
      .e_off(e_off),.e_size(e_size),.e_tag(e_tag),.e_av(e_av),.e_wt(e_wt),.e_pch(e_pch),.e_acc(e_acc),.unk_q(sq_unk_q),.viol(sq_viol),.e_block(e_block),.x_block(lq_x_block),
      // forwarding is off here (its cases are the random bench's): the blocking cases stay exact
      .e_vpn(e_vpn_nc),.e_fwd({NENT{1'b0}}),.fw_busy(1'b0),.x_fwd(x_fwd_nc),.a_bare(1'b0),
      .q_tag(lq_q_tag),.b_idx(lq_b_idx),.b_ok(lq_b_ok),
      .x_v(lq_x_v),.x_idx(lq_x_idx),.x_pa(lq_x_pa),.x_size(lq_x_size),.x_signed(lq_x_signed),
      .x_fp(lq_x_fp),.x_unc(lq_x_unc), .x_head(lq_x_head),.x_devwait(lq_x_devwait),.x_take(lq_x_take),
      .l_v(lq_l_v),.l_idx(lq_l_idx),.l_tg(lq_l_idx),.l_alt(lq_l_idx),.l_rsp(1'b1),
      .l_pre_a(lq_l_idx),.l_pre_b(lq_l_idx),.l_src_b(1'b0),.l_fw(1'b0),.l_fwn(lq_l_idx),.l_prd(lq_l_prd),.l_rd(lq_l_rd),.l_rd_v(lq_l_rd_v),.l_rob(lq_l_rob),
      .l_pa(lq_l_pa),.k_v(lk_v),.k_idx(lk_idx),.k_va(lk_va),.w_v(1'b0),.w_idx({IDXB{1'b0}}),.w_pa({PAW{1'b0}}),.w_unc(1'b0),.w_mem(1'b0),.w_flt(1'b0),.w_fc(4'd0),.f_new(lf_new),.f_v(lf_v),.f_idx(lf_idx),.f_rp(lf_rp),.f_rob(lf_rob),.f_fc(lf_fc),.f_pc(lf_pc),.f_seq(lf_seq),.occupancy(lq_occ), .av_any(lq_av_any),.e_dprd(lq_e_dprd),.uf_any(lq_uf_any),.ar({NENT{1'b1}}),.pk_v(lq_pk_v),.pk_idx(lq_pk_idx),.pk_seq(lq_pk_seq),.flush(flush),.kd_v(1'b0),.kh({ROBB{1'b0}}),.kb({ROBB{1'b0}}),.kmask(lq_kmask_nc),
      // every bench address is DRAM (speculates freely), so the device head-gate is never taken
      .a_mem(1'b1),.rob_head({ROBB{1'b0}}));

   // a load's address arrives at the store queue as the ROW update, in the same cycle it
   // fills the load queue -- exactly smolrv64_core's m_lq_fill wiring
   smolrv64_sq #(.NENT(NENT),.IDXB(IDXB),.PAW(PAW),.PBITS(PBITS),.ROBB(ROBB),.NWB(NWB),.LQN(NENT),.LQIB(IDXB)) u_sq
     (.clk(clk),.reset(reset),
      .d_alloc(sq_d_alloc),.d_pc(39'd0),.d_seq(8'd0),.d_rob(sq_d_rob),.d_dpreg(sq_d_dpreg),.d_rdy(sq_d_rdy),.d_rank(1'b0),.d_ready(sq_d_ready),.d_free(sq_free),.d_idx(sq_d_idx),.d_tag(sq_d_tag), .av_any(sq_av_any),.uf_any(sq_uf_any),.ar({NENT{1'b1}}),.pk_v(sq_pk_v),.pk_idx(sq_pk_idx),.pk_seq(sq_pk_seq),
      .a_v(sq_a_v),.a_idx(sq_a_idx),.a_addr(sq_a_addr),.a_va(39'(sq_a_addr)), .a_tv(1'b1),.a_flt(1'b0),.a_fc(4'd0),.a_size(sq_a_size),.a_unc(sq_a_unc),.a_mem(1'b1),
      .r_v(sq_r_v),.r_preg(sq_r_preg),.r_data(rfv[sq_r_preg]),
      .wb_v(wb_v),.wb_preg(wb_preg),.wb_data(wb_data),
      .c_v(sq_c_v),.c_rob(sq_c_rob),.c_addr(sq_c_addr),.c_data(sq_c_data),.c_size(sq_c_size),.c_unc(sq_c_unc),
      .c_fast(sq_c_fast_nc),.c_take(sq_c_take),.c_done(sq_c_take),
      .kc_v(sq_kc_v),.kc_rob(sq_kc_rob),.kc_addr(sq_kc_addr),.kc_data(sq_kc_data),.kc_size(sq_kc_size),.k_take(sq_k_take),
      .l_off(e_off),.l_size(e_size),.l_tag(e_tag),.l_av(e_av),.l_wt(e_wt),.l_acc(e_acc),.l_unk_q(sq_unk_q),.l_viol(sq_viol),.l_res_c(sq_res_c),.l_res_a(sq_res_a),
      .l_fill(lq_a_v),.l_fill_ix(lq_a_idx),.l_fill_go(lq_a_v & lq_a_sent),.l_cancel(sq_l_cancel),.l_fill_off(lq_a_pa[11:0]),.l_fill_size(lq_a_size), .l_fill_flt(1'b0),
      .l_vpn({(NENT*28){1'b0}}),.l_fill_vpn(28'd0),.a_bare(1'b0),.l_fwd(l_fwd_nc),.f_go(1'b0),.f_lix({IDXB{1'b0}}),.f_data(f_data_nc),.f_sbo(f_sbo_nc),
      .l_block(e_block),.l_block_unk_q(sq_l_block_unk_q),.l_block_q(l_block_q),.l_older(e_older),.ld_tag(lq_q_tag),.ld_older(ld_older),
      .k_v(sk_v),.k_idx(sk_idx),.k_va(sk_va),.w_v(1'b0),.w_idx({IDXB{1'b0}}),.w_pa({PAW{1'b0}}),.w_unc(1'b0),.w_mem(1'b1),.w_flt(1'b0),.w_fc(4'd0),.f_v(sf_v),.f_rob(sf_rob),.f_fc(sf_fc),.f_pc(sf_pc),.f_seq(sf_seq),.occupancy(sq_occ),.flush(flush),.kd_v(sq_kd_v),.kh(sq_kh),.kb(sq_kb),.kmask(sq_kmask_nc));

   integer pass=0, fail=0;
   task chk(input [255:0] nm, input got, input exp);
      begin if (got===exp) pass=pass+1;
            else begin fail=fail+1; $display("FAIL %0s: got %b want %b", nm, got, exp); end end
   endtask
   task step; begin @(posedge clk); #1; end endtask
   // program-order dispatch helpers: a load captures the store-seqno the queue hands out NOW
   reg [IDXB-1:0] L0, L1, S0, S1, S2, S3;
   reg              sq_kd_v = 0;
   reg [ROBB-1:0]   sq_kh = 0, sq_kb = 0;      // the kill: entries farther than sq_kb from sq_kh
   task disp_store(input [PBITS-1:0] dp, input [ROBB-1:0] rob, output [IDXB-1:0] ix);
      begin sq_d_dpreg=dp; sq_d_rob=rob; sq_d_alloc=1; ix=sq_d_idx; dp_of[ix]=dp; step; sq_d_alloc=0; #1; end
   endtask
   task disp_load(input [PBITS-1:0] prd, input [ROBB-1:0] rob, output [IDXB-1:0] ix);
      begin lq_d_prd=prd; lq_d_rob=rob; lq_d_sqtag=sq_d_tag; lq_d_alloc=1; ix=lq_d_idx; step; lq_d_alloc=0; #1; end
   endtask
   // dv: the store's data was in the register file at dispatch; the queue reads it once the
   // address is in (the pick a cycle later, the value the cycle after)
   task store_addr(input [IDXB-1:0] ix, input [PAW-1:0] a, input [1:0] sz, input dv, input [63:0] d);
      begin
         if (dv) rfv[dp_of[ix]] = d;
         sq_a_v=1; sq_a_idx=ix; sq_a_addr=a; sq_a_size=sz; step; sq_a_v=0; #1;
         if (dv) begin step; step; end
      end
   endtask
   // a load's address fills in the second cycle M presents it (M's two-cycle translate pass): the
   // store queue takes the load's row against the stores in the first
   task load_addr(input [IDXB-1:0] ix, input [PAW-1:0] a, input [1:0] sz);
      begin lq_a_idx=ix; lq_a_pa=a; lq_a_size=sz; step; lq_a_v=1; step; lq_a_v=0; #1; end
   endtask
   // take: the LSU takes the candidate when it is offered (the pick is a cycle ahead: a release
   // offers it the cycle after)
   integer tw;
   task take;   begin tw = 0; while (!lq_x_v && tw < 2) begin step; tw = tw + 1; end
                      chk("take: a candidate is offered", lq_x_v, 1'b1);
                      lq_x_take=1; step; lq_x_take=0; #1; end endtask
   task land(input [IDXB-1:0] ix); begin lq_l_v=1; lq_l_idx=ix; step; lq_l_v=0; #1; end endtask
   // commit = the ROB RELEASES the first unreleased entry (k_take), then the LSU DRAINS the
   // head (c_take): two events since the senior store queue (plan item 3)
   task rob_release; begin sq_k_take=1; step; sq_k_take=0; #1; end endtask
   // the LSU takes the hand-off when it is offered (a cycle after the entry is ready)
   integer dw;
   task lsu_drain; begin dw = 0; while (!sq_c_v && dw < 2) begin step; dw = dw + 1; end
                         chk("drain: a hand-off is offered", sq_c_v, 1'b1);
                         sq_c_take=1; step; sq_c_take=0; #1; end endtask
   task commit;  begin rob_release; lsu_drain; end endtask
   task drain;  begin flush=1; step; flush=0; #1; end endtask

   initial begin
      $display("tb_smolrv64_lqsq: NENT=%0d both queues, PAW=%0d, NWB=%0d", NENT, PAW, NWB);
      repeat (3) @(posedge clk); #1; reset = 0; step;

      // ---- 1. a load with no older store: offered the cycle its address is in ----
      disp_load(9'd40, 4'd1, L0);
      chk("1 nothing offered before the address", lq_x_v, 1'b0);
      load_addr(L0, 56'h3000, 2);
      chk("1 no older store", ld_older, 1'b0);
      chk("1 offered", lq_x_v && lq_x_pa==56'h3000, 1'b1);
      take;
      chk("1 taken: not offered, still live", !lq_x_v && lq_occ==1, 1'b1);
      lq_l_idx=L0; #1; chk("1 prd/rob at the landing index", lq_l_prd==9'd40 && lq_l_rob==4'd1, 1'b1);
      land(L0);
      chk("1 drained", lq_occ==0, 1'b1);

      // ---- 2. an older store with NO ADDRESS blocks the load; its address releases it ----
      disp_store(9'd7, 4'd2, S0);            // program order: S0 then L0
      disp_load(9'd41, 4'd3, L0);
      load_addr(L0, 56'h3000, 2);
      chk("2 an older store is live", ld_older, 1'b1);
      chk("2 unknown address blocks", !lq_x_v && lq_x_block, 1'b1);
      store_addr(S0, 56'h5040, 2, 1'b1, 64'h11);   // disjoint -> released (column update)
      chk("2 disjoint address releases", lq_x_v, 1'b1);
      chk("2 ld_older still true (live, not aliasing)", ld_older, 1'b1);
      take; land(L0); commit;
      chk("2 both drained", lq_occ==0 && sq_occ==0, 1'b1);

      // ---- 2b. the alias test reads page offsets: a store at the load's offset in ANOTHER page
      // holds it (it could be a synonym of the load's page), until it commits ----
      disp_store(9'd7, 4'd2, S0);
      disp_load(9'd41, 4'd3, L0);
      load_addr(L0, 56'h3010, 2);
      store_addr(S0, 56'h7010, 2, 1'b1, 64'h12);
      chk("2b the same offset in another page blocks", !lq_x_v && lq_x_block, 1'b1);
      commit;
      chk("2b the store gone, the load released", lq_x_v, 1'b1);
      take; land(L0);
      chk("2b both drained", lq_occ==0 && sq_occ==0, 1'b1);

      // ---- 3. an OVERLAPPING older store holds the load until it commits ----
      disp_store(9'd7, 4'd4, S0);
      disp_load(9'd42, 4'd5, L0);
      store_addr(S0, 56'h2000, 2, 1'b1, 64'h22);    // W at 0x2000..0x2003, data present
      load_addr(L0, 56'h2002, 1);                    // H at 0x2002: overlaps (row update)
      chk("3 overlap blocks", !lq_x_v && lq_x_block, 1'b1);
      chk("3 the store can commit", sq_kc_v && sq_c_data==64'h22, 1'b1);
      commit;
      step;                                          // the candidate is picked a cycle ahead
      chk("3 commit releases", lq_x_v, 1'b1);
      chk("3 no older store now", ld_older, 1'b0);
      take; land(L0);

      // ---- 4. partial overlap and size, both orders of arrival ----
      // row order: the load's address arrives after the store's
      disp_store(9'd7, 4'd6, S0);  disp_load(9'd43, 4'd7, L0);
      store_addr(S0, 56'h2000, 2, 1'b1, 64'h0);
      load_addr(L0, 56'h2004, 2);                    // adjacent W: the Camera case, passes
      chk("4 adjacent 4B load passes (row)", lq_x_v, 1'b1);
      take; land(L0); commit;
      disp_store(9'd7, 4'd8, S0);  disp_load(9'd44, 4'd9, L0);
      store_addr(S0, 56'h2004, 2, 1'b1, 64'h0);
      load_addr(L0, 56'h2000, 3);                    // D at 0x2000..0x2007: overlaps 0x2004
      chk("4 8B load straddling the store blocks (row)", !lq_x_v, 1'b1);
      commit; step; chk("4 released by commit", lq_x_v, 1'b1); take; land(L0);   // (picked a cycle ahead)
      // column order: the store's address arrives after the load's
      disp_store(9'd7, 4'd10, S0);  disp_load(9'd45, 4'd11, L0);
      load_addr(L0, 56'h2004, 2);
      chk("4 unknown store address blocks (column)", !lq_x_v, 1'b1);
      store_addr(S0, 56'h2000, 2, 1'b1, 64'h0);      // W at 0x2000: adjacent, not overlapping
      chk("4 adjacent 4B load passes (column)", lq_x_v, 1'b1);
      take; land(L0); commit;
      disp_store(9'd7, 4'd12, S0);  disp_load(9'd46, 4'd13, L0);
      load_addr(L0, 56'h2003, 0);                    // B at 0x2003
      store_addr(S0, 56'h2000, 2, 1'b1, 64'h0);      // W 0x2000..0x2003 covers it
      chk("4 byte inside the store blocks (column)", !lq_x_v, 1'b1);
      commit; take; land(L0);

      // ---- 5. a store commits in the SAME cycle a load's address arrives ----
      disp_store(9'd7, 4'd14, S0);  disp_load(9'd47, 4'd15, L0);
      store_addr(S0, 56'h2000, 2, 1'b1, 64'h55);
      chk("5 head ready", sq_kc_v, 1'b1);
      rob_release; repeat (2) if (!sq_c_v) step;   // the hand-off is offered a cycle after it is ready
      lq_a_idx=L0; lq_a_pa=56'h2000; lq_a_size=2; step;   // M presents the load's address (its first cycle)...
      sq_c_take=1; lq_a_v=1; step; sq_c_take=0; lq_a_v=0; #1;
      chk("5 the committed store no longer blocks the load", lq_x_v, 1'b1);
      chk("5 no older store", ld_older, 1'b0);
      take; land(L0);

      // ---- 6. flush while blocked ----
      disp_store(9'd7, 4'd1, S0);  disp_load(9'd48, 4'd2, L0);
      load_addr(L0, 56'h2000, 2);
      chk("6 blocked before the flush", !lq_x_v, 1'b1);
      drain;
      chk("6 flush empties both", lq_occ==0 && sq_occ==0, 1'b1);
      chk("6 nothing offered, nothing older", !lq_x_v && !ld_older, 1'b1);

      // ---- 7. the store queue's data path ----
      sq_d_rdy=0; disp_store(9'd7, 4'd3, S0); sq_d_rdy=1;
      store_addr(S0, 56'h2000, 2, 1'b0, 64'h0);      // address, data pending on p7
      chk("7 no commit without data", sq_kc_v, 1'b0);
      // the writeback lands on port 1 at this edge...
      wb_v=3'b010; wb_preg[1*PBITS +: PBITS]=9'd7; wb_data[1*64 +: 64]=64'hDEAD_BEEF; step;
      // ...and the bus MOVES the very next cycle: the bypass must read the copy, not the bus
      wb_v=0; wb_data[1*64 +: 64]=64'h0BAD_0BAD; #1;
      // the snoop marks the value in a register file; the data read lands it a cycle or two on
      chk("7 not committable before the data read", sq_kc_v, 1'b0);
      repeat (3) if (!sq_kc_v) step;
      #1;
      chk("7 committable once the data read lands", sq_kc_v, 1'b1);
      chk("7 data from the register file, not the moved bus", sq_c_data==64'hDEAD_BEEF, 1'b1);
      chk("7 address", sq_c_addr==56'h2000, 1'b1);
      commit; chk("7 drained", sq_occ==0, 1'b1);
      // in-order commit: allocate two, fill the SECOND first
      disp_store(9'd30, 4'd4, S0); disp_store(9'd31, 4'd5, S1);
      store_addr(S1, 56'h4000, 2, 1'b1, 64'h22);
      chk("7 head not ready -> no commit", sq_kc_v, 1'b0);
      store_addr(S0, 56'h3000, 2, 1'b1, 64'h11);
      chk("7 head ready -> commits first", sq_kc_v && sq_c_data==64'h11, 1'b1);
      commit; chk("7 then the second", sq_kc_v && sq_c_data==64'h22, 1'b1);
      commit; chk("7 drained", sq_occ==0, 1'b1);
      // the early writeback: data BEFORE the address (rule A5 regression)
      sq_d_rdy=0; disp_store(9'd5, 4'd6, S0); sq_d_rdy=1;
      wb_v=3'b001; wb_preg[0 +: PBITS]=9'd5; wb_data[0 +: 64]=64'hFEED_FACE; step; wb_v=0; #1;
      chk("7 no commit without an address", sq_kc_v, 1'b0);
      store_addr(S0, 56'h8000, 3, 1'b0, 64'h0);
      repeat (3) if (!sq_kc_v) step;   // the data read follows the address
      #1;
      chk("7 early writeback captured", sq_kc_v && sq_c_data==64'hFEED_FACE, 1'b1);
      commit;
      // ...and in the ALLOCATE cycle itself
      sq_d_dpreg=9'd6; sq_d_rob=4'd7; sq_d_alloc=1; sq_d_rdy=0; S0=sq_d_idx;
      wb_v=3'b100; wb_preg[2*PBITS +: PBITS]=9'd6; wb_data[2*64 +: 64]=64'hC0FFEE; step;
      sq_d_alloc=0; sq_d_rdy=1; wb_v=0; #1;
      store_addr(S0, 56'h9000, 3, 1'b0, 64'h0);
      repeat (3) if (!sq_kc_v) step;   // the data read follows the address
      #1;
      chk("7 same-cycle writeback captured", sq_kc_v && sq_c_data==64'hC0FFEE, 1'b1);
      commit;
      chk("7 full buffer refuses", sq_d_ready, 1'b1);
      sq_d_alloc=1; repeat (NENT) step; sq_d_alloc=0; #1;
      chk("7 full", sq_occ==NENT && !sq_d_ready, 1'b1);
      drain;

      // ---- 8. the load queue's ordering ----
      disp_load(9'd41, 4'd1, L0); disp_load(9'd42, 4'd2, L1);
      load_addr(L1, 56'h6000, 2);                    // the YOUNGER one only
      step; chk("8 a ready younger load is not held up by an unfilled older one", lq_x_v && lq_x_pa==56'h6000, 1'b1);
      take;
      load_addr(L0, 56'h7000, 2);
      chk("8 the older goes when its address is in", lq_x_v && lq_x_pa==56'h7000, 1'b1);
      take;
      chk("8 both in flight", lq_occ==2 && !lq_x_v, 1'b1);
      lq_l_idx=L0; #1; chk("8 lands the older", lq_l_prd==9'd41, 1'b1);
      land(L0); lq_l_idx=L1; #1; chk("8 then the second", lq_l_prd==9'd42, 1'b1);
      land(L1); chk("8 drained", lq_occ==0, 1'b1);
      // the early start: M's translate pass also STARTS the access (a_sent), so the entry
      // is never a candidate
      disp_load(9'd43, 4'd3, L0);
      lq_b_idx=L0; #1; chk("8 b_ok: M's load is live and unfilled", lq_b_ok, 1'b1);
      lq_a_idx=L0; lq_a_pa=56'h8000; lq_a_size=2; step;   // M's first cycle
      lq_a_v=1; lq_a_sent=1; step; lq_a_v=0; lq_a_sent=0; #1;
      chk("8 early-started entry is not offered", !lq_x_v && lq_occ==1, 1'b1);
      land(L0); chk("8 and lands", lq_occ==0, 1'b1);
      lq_d_alloc=1; repeat (NENT) step; lq_d_alloc=0; #1;
      chk("8 full queue refuses", lq_occ==NENT && !lq_d_ready, 1'b1);
      drain;

      // ---- 9. a load dispatched against a FULL store queue: every store in it is older ----
      // The seqno a load captures is the tail; with tail == head a distance of zero said "no
      // older store" and the load overtook all of them (GB5 boot retire 123,081,278; Ubuntu
      // userspace segfaults on the board, 2026-09-04). NENT stores, none committed, then the
      // load: it must wait for the last of them.
      disp_store(9'd7, 4'd1, S0); disp_store(9'd7, 4'd2, S1);
      disp_store(9'd7, 4'd3, S2); disp_store(9'd7, 4'd4, S3);
      chk("9 store queue full", sq_occ==NENT && !sq_d_ready, 1'b1);
      disp_load(9'd44, 4'd5, L0);                       // tag = head + NENT, the wrap bit set
      load_addr(L0, 56'h9000, 2);
      chk("9 an older store is live (the queue was full at dispatch)", ld_older, 1'b1);
      chk("9 unknown addresses block", !lq_x_v && lq_x_block, 1'b1);
      store_addr(S0, 56'h9100, 2, 1'b1, 64'h1); store_addr(S1, 56'h9200, 2, 1'b1, 64'h2);
      store_addr(S2, 56'h9300, 2, 1'b1, 64'h3);
      chk("9 still blocked by the last unknown address", !lq_x_v && lq_x_block, 1'b1);
      store_addr(S3, 56'h9000, 2, 1'b1, 64'h4);         // the youngest store aliases the load
      chk("9 the aliasing fourth store blocks", !lq_x_v && lq_x_block, 1'b1);
      commit; commit; commit;
      chk("9 three commits do not release it", !lq_x_v && lq_x_block && ld_older, 1'b1);
      commit;
      step; chk("9 the fourth does", lq_x_v && !ld_older, 1'b1);
      take; land(L0);
      chk("9 drained", lq_occ==0 && sq_occ==0, 1'b1);
      // ...and with the pointers WRAPPED: two stores committed first, then the queue refilled,
      // so the tail passed the head before the load captured it.
      disp_store(9'd7, 4'd6, S0); disp_store(9'd7, 4'd7, S1);
      store_addr(S0, 56'hA000, 2, 1'b1, 64'h5); store_addr(S1, 56'hA100, 2, 1'b1, 64'h6);
      commit; commit;
      disp_store(9'd7, 4'd8, S0); disp_store(9'd7, 4'd9, S1);
      disp_store(9'd7, 4'd10, S2); disp_store(9'd7, 4'd11, S3);
      chk("9w full again across the wrap", sq_occ==NENT && !sq_d_ready, 1'b1);
      disp_load(9'd45, 4'd12, L0);
      load_addr(L0, 56'hB000, 2);
      chk("9w older stores live across the wrap", ld_older, 1'b1);
      store_addr(S0, 56'hB100, 2, 1'b1, 64'h7); store_addr(S1, 56'hB200, 2, 1'b1, 64'h8);
      store_addr(S2, 56'hB300, 2, 1'b1, 64'h9); store_addr(S3, 56'hB000, 2, 1'b1, 64'hA);
      chk("9w the aliasing store blocks", !lq_x_v && lq_x_block, 1'b1);
      commit; commit; commit; commit;
      chk("9w released after all four commit", lq_x_v && !ld_older, 1'b1);
      take; land(L0);
      chk("9w drained", lq_occ==0 && sq_occ==0, 1'b1);

      // ---- 14. the data reads come in the order ADDRESSES arrive, not program order (the
      // out-of-order fill): the younger store's address is first, it is picked, the older one's
      // arrives a cycle later and is picked in the very cycle the younger one is killed. The
      // older store must still get its data (a cancel keyed to the previous pick lost it, and the
      // store then sat at the ROB head forever waiting for a value no one would send). ----
      disp_store(9'd11, 4'd3, S0); disp_store(9'd12, 4'd4, S1);
      rfv[9'd11] = 64'hAA11; rfv[9'd12] = 64'hBB22;
      sq_a_v=1; sq_a_idx=S1; sq_a_addr=56'hC100; sq_a_size=2; step; sq_a_v=0;       // S1's address; S1 is picked next
      // the kill holds for cycles, as the ROB's does; the queue's marks are a cycle behind it
      sq_kd_v=1; sq_kh=0; sq_kb=3;
      sq_a_v=1; sq_a_idx=S0; sq_a_addr=56'hC000; sq_a_size=2; step; sq_a_v=0;       // S0's address; S1 lands, S0 is picked at the next edge
      step; sq_kd_v=0; #1;                                                          // ...in the cycle S1 (rob 4) dies
      step; step;
      chk("14 the older store got its data", sq_kc_v && sq_c_data==64'hAA11, 1'b1);
      chk("14 the younger store died", sq_occ==1, 1'b1);
      commit;
      chk("14 drained", sq_occ==0, 1'b1);

      // ---- 10. EVERY attribute presented at fill comes back at take (rule B7) ----
      // The uncached bit was never carried (2026-09-04): the port hardwired it to 0 and a
      // queued NC load was cached. Present every attribute with a distinctive value and read
      // them all back on the candidate, so a field added to the port without its entry
      // fails here rather than on the board.
      disp_load(9'd46, 4'd13, L0);
      lq_a_idx=L0; lq_a_pa=56'h00_ABCD_EF01_2348; lq_a_size=2'd1; lq_a_signed=1; lq_a_fp=1; lq_a_unc=1; step;   // M's first cycle
      lq_a_v=1;
      step; lq_a_v=0; #1;
      step; chk("10 pa round-trips",     lq_x_v && lq_x_pa==56'h00_ABCD_EF01_2348, 1'b1);
      chk("10 size round-trips",   lq_x_size==2'd1, 1'b1);
      chk("10 signed round-trips", lq_x_signed, 1'b1);
      chk("10 fp round-trips",     lq_x_fp, 1'b1);
      chk("10 unc round-trips",    lq_x_unc, 1'b1);
      lq_a_signed=0; lq_a_fp=0; lq_a_unc=0; lq_a_size=2;
      take; land(L0);
      chk("10 drained", lq_occ==0, 1'b1);

      // ---- 11. a FLUSH keeps the released stores and drops the rest (the senior queue) ----
      disp_store(9'd7, 4'd14, S0); disp_store(9'd7, 4'd15, S1);
      store_addr(S0, 56'hC000, 2, 1'b1, 64'hAA); store_addr(S1, 56'hC100, 2, 1'b1, 64'hBB);
      chk("11 nothing drains before a release", sq_c_v, 1'b0);
      chk("11 the first is releasable", sq_kc_v && sq_kc_addr==56'hC000, 1'b1);
      rob_release; step;                            // the hand-off is offered a cycle after it is ready
      chk("11 released head drains, the second is next to release", sq_c_v && sq_c_data==64'hAA && sq_kc_v && sq_kc_addr==56'hC100, 1'b1);
      lsu_drain;                                    // S0 gone; S1 next to release
      chk("11 drained one", sq_occ==1 && !sq_c_v && sq_kc_v, 1'b1);
      rob_release;                                      // S1 released
      disp_store(9'd7, 4'd0, S2);                   // a younger, unreleased store
      store_addr(S2, 56'hC200, 2, 1'b1, 64'hCC);
      chk("11 two live, one released", sq_occ==2 && sq_c_v && sq_c_data==64'hBB, 1'b1);
      flush=1; step; flush=0; #1;
      chk("11 flush keeps the released store, drops the unreleased", sq_occ==1 && sq_c_v && sq_c_data==64'hBB && !sq_kc_v, 1'b1);
      lsu_drain;
      chk("11 drained after the flush", sq_occ==0 && lq_occ==0, 1'b1);

      // ---- 12. released, then drained the cycle after: the release is a register ----
      disp_store(9'd7, 4'd1, S0);
      store_addr(S0, 56'hD000, 2, 1'b1, 64'hDD);
      chk("12 releasable, not draining", sq_kc_v && !sq_c_v, 1'b1);
      sq_k_take=1; #1;
      chk("12 not draining in the release cycle", sq_c_v, 1'b0);
      step; sq_k_take=0; #1;
      chk("12 not offered the cycle after the release", sq_c_v, 1'b0);
      step;
      chk("12 offered two cycles after the release", sq_c_v && sq_c_data==64'hDD && !sq_kc_v, 1'b1);
      lsu_drain;
      chk("12 gone, nothing left to release", sq_occ==0 && !sq_c_v && !sq_kc_v, 1'b1);

      // ---- 13. av_any, the CBO's wait: "an older store HAS AN ADDRESS", never the occupancy.
      // An entry is allocated at dispatch, so a store YOUNGER than the op in M is in the queue
      // without an address; waiting for it to leave is the deadlock that hung build L. ----
      disp_store(9'd7, 4'd2, S0);
      chk("13 an entry without an address is no older store", sq_occ==1 && !sq_av_any, 1'b1);
      chk("13 ...and it is the oldest unfilled one", sq_uf_any && sq_pk_v && sq_pk_idx == S0, 1'b1);
      store_addr(S0, 56'hE000, 2, 1'b1, 64'hEE);
      chk("13 with an address it is one", sq_av_any && !sq_uf_any, 1'b1);
      commit;
      chk("13 drained, none", sq_occ==0 && !sq_av_any, 1'b1);

      $display("---- tb_smolrv64_lqsq pass=%0d fail=%0d", pass, fail);
      if (fail != 0) begin $display("LQSQ-TB FAIL"); $fatal(1, "tb_smolrv64_lqsq FAILED"); end
      $display("LQSQ-TB PASS");
      $finish;
   end

endmodule
`default_nettype wire
