`default_nettype none
// Standalone check for ooo2_lq AND ooo2_sq TOGETHER, wired as ooo2_core wires them.
//
// The two used to have separate benches, each driving the other's side of the alias test
// by hand. 69f1ac6a turned that test into a CONFLICT MATRIX that lives in the store queue
// and is fed by the load queue's entries, so the interface between the two modules IS the
// thing under test, and one bench that owns both is the only honest shape. The old benches
// stopped compiling at that commit and nobody noticed (gate.sh now runs every
// run-ooo2-*-tb.sh and treats a build failure as a failure).
//
// What is checked, in the order docs/HANDOFF-2026-09-03-multiple-loads.md lists it:
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
// Configuration stated in the output: NENT=4 both, the shipped shape.
module tb;
   localparam NENT=4, IDXB=2, PAW=56, PBITS=9, ROBB=4, NWB=3;
   reg clk=0, reset=1, flush=0;
   always #5 clk = ~clk;

   // ---- load queue ports ----
   reg              lq_d_alloc=0, lq_a_v=0, lq_a_sent=0, lq_x_take=0, lq_l_v=0;
   reg [ROBB-1:0]   lq_d_rob=0;  reg [PBITS-1:0] lq_d_prd=0;  reg [5:0] lq_d_rd=0;  reg lq_d_rd_v=1;
   reg [IDXB:0]     lq_d_sqtag=0;
   reg [IDXB-1:0]   lq_a_idx=0, lq_b_idx=0, lq_l_idx=0;
   reg [PAW-1:0]    lq_a_pa=0;   reg [1:0] lq_a_size=2;  reg lq_a_signed=0, lq_a_fp=0;
   wire             lq_d_ready, lq_x_v, lq_x_block, lq_b_ok, lq_x_signed, lq_x_fp, lq_x_unc, lq_l_rd_v;
   wire [IDXB-1:0]  lq_d_idx, lq_x_idx;  wire [IDXB:0] lq_q_tag;
   wire [PAW-1:0]   lq_x_pa, lq_l_pa;  wire [1:0] lq_x_size;
   wire [PBITS-1:0] lq_l_prd;  wire [5:0] lq_l_rd;  wire [ROBB-1:0] lq_l_rob;  wire [IDXB:0] lq_occ;
   wire [NENT*PAW-1:0] e_pa;  wire [NENT*2-1:0] e_size;  wire [NENT*(IDXB+1)-1:0] e_tag;
   wire [NENT-1:0]  e_av, e_block;
   // ---- store queue ports ----
   reg              sq_d_alloc=0, sq_a_v=0, sq_a_data_v=0, sq_a_unc=0, sq_c_take=0;
   reg [ROBB-1:0]   sq_d_rob=0;  reg [PBITS-1:0] sq_d_dpreg=0;
   reg [IDXB-1:0]   sq_a_idx=0;  reg [PAW-1:0] sq_a_addr=0;  reg [1:0] sq_a_size=2;  reg [63:0] sq_a_data=0;
   reg [NWB-1:0]    wb_v=0;  reg [NWB*PBITS-1:0] wb_preg=0;  reg [NWB*64-1:0] wb_data=0;
   wire             sq_d_ready, sq_c_v, sq_c_unc, ld_older;
   wire [IDXB-1:0]  sq_d_idx;  wire [IDXB:0] sq_d_tag;  wire [ROBB-1:0] sq_c_rob;
   wire [PAW-1:0]   sq_c_addr;  wire [63:0] sq_c_data;  wire [1:0] sq_c_size;  wire [IDXB:0] sq_occ;

   ooo2_lq #(.NENT(NENT),.IDXB(IDXB),.PAW(PAW),.PBITS(PBITS),.ROBB(ROBB),.SQIB(IDXB+1)) u_lq
     (.clk(clk),.reset(reset),
      .d_alloc(lq_d_alloc),.d_rob(lq_d_rob),.d_prd(lq_d_prd),.d_rd(lq_d_rd),.d_rd_v(lq_d_rd_v),
      .d_sqtag(lq_d_sqtag),.d_ready(lq_d_ready),.d_idx(lq_d_idx),
      .a_v(lq_a_v),.a_sent(lq_a_sent),.a_idx(lq_a_idx),.a_pa(lq_a_pa),.a_size(lq_a_size),
      .a_signed(lq_a_signed),.a_fp(lq_a_fp),.a_unc(1'b0),
      .e_pa(e_pa),.e_size(e_size),.e_tag(e_tag),.e_av(e_av),.e_block(e_block),.x_block(lq_x_block),
      .q_tag(lq_q_tag),.b_idx(lq_b_idx),.b_ok(lq_b_ok),
      .x_v(lq_x_v),.x_idx(lq_x_idx),.x_pa(lq_x_pa),.x_size(lq_x_size),.x_signed(lq_x_signed),
      .x_fp(lq_x_fp),.x_unc(lq_x_unc),.x_take(lq_x_take),
      .l_v(lq_l_v),.l_idx(lq_l_idx),.l_prd(lq_l_prd),.l_rd(lq_l_rd),.l_rd_v(lq_l_rd_v),.l_rob(lq_l_rob),
      .l_pa(lq_l_pa),.occupancy(lq_occ),.flush(flush));

   // a load's address arrives at the store queue as the ROW update, in the same cycle it
   // fills the load queue -- exactly ooo2_core's m_lq_fill wiring
   ooo2_sq #(.NENT(NENT),.IDXB(IDXB),.PAW(PAW),.PBITS(PBITS),.ROBB(ROBB),.NWB(NWB),.LQN(NENT),.LQIB(IDXB)) u_sq
     (.clk(clk),.reset(reset),
      .d_alloc(sq_d_alloc),.d_rob(sq_d_rob),.d_dpreg(sq_d_dpreg),.d_ready(sq_d_ready),.d_idx(sq_d_idx),.d_tag(sq_d_tag),
      .a_v(sq_a_v),.a_idx(sq_a_idx),.a_addr(sq_a_addr),.a_size(sq_a_size),.a_unc(sq_a_unc),
      .a_data_v(sq_a_data_v),.a_data(sq_a_data),
      .wb_v(wb_v),.wb_preg(wb_preg),.wb_data(wb_data),
      .c_v(sq_c_v),.c_rob(sq_c_rob),.c_addr(sq_c_addr),.c_data(sq_c_data),.c_size(sq_c_size),.c_unc(sq_c_unc),
      .c_take(sq_c_take),
      .l_pa(e_pa),.l_size(e_size),.l_tag(e_tag),.l_av(e_av),
      .l_fill(lq_a_v),.l_fill_ix(lq_a_idx),.l_fill_pa(lq_a_pa),.l_fill_size(lq_a_size),
      .l_block(e_block),.ld_tag(lq_q_tag),.ld_older(ld_older),
      .occupancy(sq_occ),.flush(flush));

   integer pass=0, fail=0;
   task chk(input [255:0] nm, input got, input exp);
      begin if (got===exp) pass=pass+1;
            else begin fail=fail+1; $display("FAIL %0s: got %b want %b", nm, got, exp); end end
   endtask
   task step; begin @(posedge clk); #1; end endtask
   // program-order dispatch helpers: a load captures the store-seqno the queue hands out NOW
   reg [IDXB-1:0] L0, L1, S0, S1, S2, S3;
   task disp_store(input [PBITS-1:0] dp, input [ROBB-1:0] rob, output [IDXB-1:0] ix);
      begin sq_d_dpreg=dp; sq_d_rob=rob; sq_d_alloc=1; ix=sq_d_idx; step; sq_d_alloc=0; #1; end
   endtask
   task disp_load(input [PBITS-1:0] prd, input [ROBB-1:0] rob, output [IDXB-1:0] ix);
      begin lq_d_prd=prd; lq_d_rob=rob; lq_d_sqtag=sq_d_tag; lq_d_alloc=1; ix=lq_d_idx; step; lq_d_alloc=0; #1; end
   endtask
   task store_addr(input [IDXB-1:0] ix, input [PAW-1:0] a, input [1:0] sz, input dv, input [63:0] d);
      begin sq_a_v=1; sq_a_idx=ix; sq_a_addr=a; sq_a_size=sz; sq_a_data_v=dv; sq_a_data=d; step; sq_a_v=0; sq_a_data_v=0; #1; end
   endtask
   task load_addr(input [IDXB-1:0] ix, input [PAW-1:0] a, input [1:0] sz);
      begin lq_a_v=1; lq_a_idx=ix; lq_a_pa=a; lq_a_size=sz; step; lq_a_v=0; #1; end
   endtask
   task take;   begin lq_x_take=1; step; lq_x_take=0; #1; end endtask
   task land(input [IDXB-1:0] ix); begin lq_l_v=1; lq_l_idx=ix; step; lq_l_v=0; #1; end endtask
   task commit; begin sq_c_take=1; step; sq_c_take=0; #1; end endtask
   task drain;  begin flush=1; step; flush=0; #1; end endtask

   initial begin
      $display("tb_ooo2_lqsq: NENT=%0d both queues, PAW=%0d, NWB=%0d", NENT, PAW, NWB);
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
      store_addr(S0, 56'h5000, 2, 1'b1, 64'h11);   // disjoint -> released (column update)
      chk("2 disjoint address releases", lq_x_v, 1'b1);
      chk("2 ld_older still true (live, not aliasing)", ld_older, 1'b1);
      take; land(L0); commit;
      chk("2 both drained", lq_occ==0 && sq_occ==0, 1'b1);

      // ---- 3. an OVERLAPPING older store holds the load until it commits ----
      disp_store(9'd7, 4'd4, S0);
      disp_load(9'd42, 4'd5, L0);
      store_addr(S0, 56'h2000, 2, 1'b1, 64'h22);    // W at 0x2000..0x2003, data present
      load_addr(L0, 56'h2002, 1);                    // H at 0x2002: overlaps (row update)
      chk("3 overlap blocks", !lq_x_v && lq_x_block, 1'b1);
      chk("3 the store can commit", sq_c_v && sq_c_data==64'h22, 1'b1);
      commit;
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
      store_addr(S0, 56'h2000, 2, 1'b1, 64'h0);
      load_addr(L0, 56'h1FFC, 3);                    // D at 0x1FFC..0x2003: overlaps 0x2000
      chk("4 8B load straddling the store blocks (row)", !lq_x_v, 1'b1);
      commit; chk("4 released by commit", lq_x_v, 1'b1); take; land(L0);
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
      chk("5 head ready", sq_c_v, 1'b1);
      sq_c_take=1; lq_a_v=1; lq_a_idx=L0; lq_a_pa=56'h2000; lq_a_size=2; step; sq_c_take=0; lq_a_v=0; #1;
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
      disp_store(9'd7, 4'd3, S0);
      store_addr(S0, 56'h2000, 2, 1'b0, 64'h0);      // address, data pending on p7
      chk("7 no commit without data", sq_c_v, 1'b0);
      // the writeback lands on port 1 at this edge...
      wb_v=3'b010; wb_preg[1*PBITS +: PBITS]=9'd7; wb_data[1*64 +: 64]=64'hDEAD_BEEF; step;
      // ...and the bus MOVES the very next cycle: the bypass must read the copy, not the bus
      wb_v=0; wb_data[1*64 +: 64]=64'h0BAD_0BAD; #1;
      chk("7 committable the cycle after the writeback", sq_c_v, 1'b1);
      chk("7 data through the landing bypass", sq_c_data==64'hDEAD_BEEF, 1'b1);
      step; #1;
      chk("7 data from the entry a cycle later", sq_c_v && sq_c_data==64'hDEAD_BEEF, 1'b1);
      chk("7 address", sq_c_addr==56'h2000, 1'b1);
      commit; chk("7 drained", sq_occ==0, 1'b1);
      // in-order commit: allocate two, fill the SECOND first
      disp_store(9'd0, 4'd4, S0); disp_store(9'd0, 4'd5, S1);
      store_addr(S1, 56'h4000, 2, 1'b1, 64'h22);
      chk("7 head not ready -> no commit", sq_c_v, 1'b0);
      store_addr(S0, 56'h3000, 2, 1'b1, 64'h11);
      chk("7 head ready -> commits first", sq_c_v && sq_c_data==64'h11, 1'b1);
      commit; chk("7 then the second", sq_c_v && sq_c_data==64'h22, 1'b1);
      commit; chk("7 drained", sq_occ==0, 1'b1);
      // the early writeback: data BEFORE the address (rule A5 regression)
      disp_store(9'd5, 4'd6, S0);
      wb_v=3'b001; wb_preg[0 +: PBITS]=9'd5; wb_data[0 +: 64]=64'hFEED_FACE; step; wb_v=0; #1;
      chk("7 no commit without an address", sq_c_v, 1'b0);
      store_addr(S0, 56'h8000, 3, 1'b0, 64'h0);
      chk("7 early writeback captured", sq_c_v && sq_c_data==64'hFEED_FACE, 1'b1);
      commit;
      // ...and in the ALLOCATE cycle itself
      sq_d_dpreg=9'd6; sq_d_rob=4'd7; sq_d_alloc=1; S0=sq_d_idx;
      wb_v=3'b100; wb_preg[2*PBITS +: PBITS]=9'd6; wb_data[2*64 +: 64]=64'hC0FFEE; step;
      sq_d_alloc=0; wb_v=0; #1;
      store_addr(S0, 56'h9000, 3, 1'b0, 64'h0);
      chk("7 same-cycle writeback captured", sq_c_v && sq_c_data==64'hC0FFEE, 1'b1);
      commit;
      chk("7 full buffer refuses", sq_d_ready, 1'b1);
      sq_d_alloc=1; repeat (NENT) step; sq_d_alloc=0; #1;
      chk("7 full", sq_occ==NENT && !sq_d_ready, 1'b1);
      drain;

      // ---- 8. the load queue's ordering ----
      disp_load(9'd41, 4'd1, L0); disp_load(9'd42, 4'd2, L1);
      load_addr(L1, 56'h6000, 2);                    // the YOUNGER one only
      chk("8 no overtake of an unfilled older entry", lq_x_v, 1'b0);
      load_addr(L0, 56'h7000, 2);
      chk("8 the older goes first", lq_x_v && lq_x_pa==56'h7000, 1'b1);
      take;
      chk("8 the second offered while the first is in flight", lq_x_v && lq_x_pa==56'h6000, 1'b1);
      take;
      chk("8 both in flight", lq_occ==2 && !lq_x_v, 1'b1);
      lq_l_idx=L0; #1; chk("8 lands the first", lq_l_prd==9'd41, 1'b1);
      land(L0); lq_l_idx=L1; #1; chk("8 then the second", lq_l_prd==9'd42, 1'b1);
      land(L1); chk("8 drained", lq_occ==0, 1'b1);
      // the early start: M's translate pass also STARTS the access (a_sent), so the entry
      // is never a candidate and the queue's pointer moves past it
      disp_load(9'd43, 4'd3, L0);
      lq_b_idx=L0; #1; chk("8 b_ok: M's load is the untranslated candidate", lq_b_ok, 1'b1);
      lq_a_v=1; lq_a_sent=1; lq_a_idx=L0; lq_a_pa=56'h8000; lq_a_size=2; step; lq_a_v=0; lq_a_sent=0; #1;
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
      chk("9 the fourth does", lq_x_v && !ld_older, 1'b1);
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

      $display("---- tb_ooo2_lqsq pass=%0d fail=%0d", pass, fail);
      if (fail != 0) begin $display("LQSQ-TB FAIL"); $fatal(1, "tb_ooo2_lqsq FAILED"); end
      $display("LQSQ-TB PASS");
      $finish;
   end
endmodule
`default_nettype wire
