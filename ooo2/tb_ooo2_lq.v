`default_nettype none
// Standalone check for ooo2_lq (rule I3: verify the structure before wiring it in).
module tb_ooo2_lq;
   localparam NENT=4, IDXB=2, PAW=56, PBITS=9, ROBB=4, SQIB=3;
   reg clk=0, reset=1, flush=0;
   always #5 clk = ~clk;

   reg d_alloc=0, a_v=0, x_take=0, l_v=0, q_block=0;
   reg [IDXB-1:0] l_idx=0;
   reg [ROBB-1:0]  d_rob=0;
   reg [PBITS-1:0] d_prd=0;
   reg [5:0]       d_rd=0;
   reg             d_rd_v=1, a_signed=0, a_fp=0;
   reg [SQIB-1:0]  d_sqtag=0;
   reg [IDXB-1:0]  a_idx=0;
   reg [PAW-1:0]   a_pa=0;
   reg [1:0]       a_size=2;
   wire d_ready, x_v, x_signed, x_fp, l_rd_v;
   wire [IDXB-1:0] d_idx;
   wire [PAW-1:0]  q_pa, x_pa;
   wire [1:0]      q_size, x_size;
   wire [SQIB-1:0] q_tag;
   wire [PBITS-1:0] l_prd;
   wire [5:0]      l_rd;
   wire [ROBB-1:0] l_rob;
   wire [IDXB:0]   occ;

   ooo2_lq #(.NENT(NENT),.IDXB(IDXB),.PAW(PAW),.PBITS(PBITS),.ROBB(ROBB),.SQIB(SQIB)) dut
     (.clk(clk),.reset(reset),
      .d_alloc(d_alloc),.d_rob(d_rob),.d_prd(d_prd),.d_rd(d_rd),.d_rd_v(d_rd_v),
      .d_sqtag(d_sqtag),.d_ready(d_ready),.d_idx(d_idx),
      .a_v(a_v),.a_idx(a_idx),.a_pa(a_pa),.a_size(a_size),.a_signed(a_signed),.a_fp(a_fp),
      .q_pa(q_pa),.q_size(q_size),.q_tag(q_tag),.q_block(q_block),
      .x_v(x_v),.x_pa(x_pa),.x_size(x_size),.x_signed(x_signed),.x_fp(x_fp),.x_take(x_take),
      .l_v(l_v),.l_idx(l_idx),.l_prd(l_prd),.l_rd(l_rd),.l_rd_v(l_rd_v),.l_rob(l_rob),
      .occupancy(occ),.flush(flush));

   reg [IDXB-1:0] i0, i1;      // the indices actually handed out -- the tail MOVES
   integer pass=0, fail=0;
   task chk(input [255:0] nm, input got, input exp);
      begin if (got===exp) pass=pass+1;
            else begin fail=fail+1; $display("FAIL %0s: got %b want %b", nm, got, exp); end end
   endtask
   task step; begin @(posedge clk); #1; end endtask

   initial begin
      repeat (3) @(posedge clk); #1; reset = 0; step;

      // 1. an allocated entry offers NOTHING until its address is translated
      d_rob=4'd5; d_prd=9'd40; d_rd=6'd7; d_sqtag=3'd2; d_alloc=1; i0=d_idx; step; d_alloc=0; #1;
      chk("1 occ 1", occ==1, 1'b1);
      chk("1 no access without an address", x_v, 1'b0);

      // 2. fill it -- now it is a candidate, and its PA reaches the disambiguation port
      a_v=1; a_idx=i0; a_pa=56'h3000; a_size=2; step; a_v=0; #1;
      chk("2 candidate offered", x_v, 1'b1);
      chk("2 queries its own PA",  q_pa==56'h3000, 1'b1);
      chk("2 carries its seqno",   q_tag==3'd2, 1'b1);

      // 3. an older store that may alias holds it -- WITHOUT holding translation,
      //    which is the whole point of the queue
      q_block=1; #1; chk("3 blocked", x_v, 1'b0);
      q_block=0; #1; chk("3 unblocked", x_v, 1'b1);

      // 4. take it; it stops being a candidate but stays live until its data lands
      x_take=1; step; x_take=0; #1;
      chk("4 no longer offered", x_v, 1'b0);
      chk("4 still live",        occ==1, 1'b1);

      // 5. landing writes back the entry's OWN destination, not the newest one
      l_idx=i0; #1;
      chk("5 prd", l_prd==9'd40, 1'b1);
      chk("5 rob", l_rob==4'd5,  1'b1);
      l_v=1; step; l_v=0; #1;
      chk("5 drained", occ==0, 1'b1);

      // 6. TWO OUTSTANDING. The second is offered while the first is still in flight --
      //    this is item 3 of the Camera list falling out of item 2.
      d_rob=4'd6; d_prd=9'd41; d_alloc=1; i0=d_idx; step;
      d_rob=4'd7; d_prd=9'd42;            i1=d_idx; step; d_alloc=0; #1;
      a_v=1; a_idx=i0; a_pa=56'h4000; step; a_v=0; #1;
      a_v=1; a_idx=i1; a_pa=56'h5000; step; a_v=0; #1;
      x_take=1; step; x_take=0; #1;                    // first sent
      chk("6 second offered in flight", x_v && x_pa==56'h5000, 1'b1);
      x_take=1; step; x_take=0; #1;                    // second sent too
      chk("6 both in flight", occ==2, 1'b1);
      l_idx=i0; #1; chk("6 lands oldest first", l_prd==9'd41, 1'b1);
      l_v=1; step; l_v=0; l_idx=i1; #1;
      chk("6 then the second", l_prd==9'd42, 1'b1);
      l_v=1; step; l_v=0; #1;
      chk("6 drained", occ==0, 1'b1);

      // 7. AN UNFILLED ENTRY IS NOT SKIPPED. If a younger filled load could overtake an
      //    older unfilled one, loads would reach memory out of program order with nothing
      //    ordering them against each other.
      d_alloc=1; i0=d_idx; step; i1=d_idx; step; d_alloc=0; #1;
      a_v=1; a_idx=i1; a_pa=56'h6000; step; a_v=0; #1; // fill the YOUNGER one only
      chk("7 no overtake of unfilled", x_v, 1'b0);
      a_v=1; a_idx=i0; a_pa=56'h7000; step; a_v=0; #1;
      chk("7 older goes first", x_v && x_pa==56'h7000, 1'b1);

      // 8. flush clears everything, in flight included -- a load has no side effect
      flush=1; step; flush=0; #1;
      chk("8 flush empties", occ==0, 1'b1);
      chk("8 nothing offered", x_v, 1'b0);

      // 9. full queue refuses allocation (the frontend takes the back-pressure)
      d_alloc=1; repeat (NENT) step; d_alloc=0; #1;
      chk("9 full", occ==NENT, 1'b1);
      chk("9 d_ready low when full", d_ready, 1'b0);

      $display("---- tb_ooo2_lq pass=%0d fail=%0d", pass, fail);
      if (fail != 0) $fatal(1, "tb_ooo2_lq FAILED");
      $finish;
   end
endmodule
`default_nettype wire
