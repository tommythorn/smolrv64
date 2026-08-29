`default_nettype none
// Standalone check for ooo2_sq (rule I3: verify the structure before wiring it in).
module tb_ooo2_sq;
   localparam NENT=4, IDXB=2, PAW=56, PBITS=9, NWB=3;
   reg clk=0, reset=1, flush=0;
   always #5 clk = ~clk;

   reg d_alloc=0, a_v=0, a_data_v=0, c_take=0;
   reg [3:0] d_rob=0;
   reg [IDXB-1:0]  a_idx=0;
   reg [PAW-1:0]   a_addr=0, ld_addr=0;
   reg [1:0]       a_size=2, ld_size=2;
   reg [PBITS-1:0] d_dpreg=0;
   reg [63:0]      a_data=0;
   reg [NWB-1:0]   wb_v=0;
   reg [NWB*PBITS-1:0] wb_preg=0;
   reg [NWB*64-1:0]    wb_data=0;
   wire d_ready, c_v, ld_block;
   wire [3:0] c_rob;
   wire [IDXB-1:0] d_idx;
   wire [PAW-1:0]  c_addr;
   wire [63:0]     c_data;
   wire [1:0]      c_size;
   wire [IDXB:0]   occ;

   ooo2_sq #(.NENT(NENT),.IDXB(IDXB),.PAW(PAW),.PBITS(PBITS),.ROBB(4),.NWB(NWB)) dut
     (.clk(clk),.reset(reset),.d_alloc(d_alloc),.d_rob(d_rob),.d_dpreg(d_dpreg),.d_ready(d_ready),.d_idx(d_idx),
      .a_v(a_v),.a_idx(a_idx),.a_addr(a_addr),.a_size(a_size),
      .a_data_v(a_data_v),.a_data(a_data),
      .wb_v(wb_v),.wb_preg(wb_preg),.wb_data(wb_data),
      .c_v(c_v),.c_rob(c_rob),.c_addr(c_addr),.c_data(c_data),.c_size(c_size),.c_take(c_take),
      .ld_addr(ld_addr),.ld_size(ld_size),.ld_block(ld_block),
      .occupancy(occ),.flush(flush));

   integer pass=0, fail=0;
   task chk(input [255:0] nm, input got, input exp);
      begin if (got===exp) pass=pass+1;
            else begin fail=fail+1; $display("FAIL %0s: got %b want %b", nm, got, exp); end end
   endtask
   task step; begin @(posedge clk); #1; end endtask

   initial begin
      repeat (3) @(posedge clk); #1; reset = 0; step;

      // 1. allocate one store; nothing to commit until address AND data arrive
      d_dpreg=9'd7; d_alloc=1; step; d_alloc=0; step;
      chk("1 alloc -> occ 1", occ==1, 1'b1);
      chk("1 no commit yet",  c_v, 1'b0);

      // 2. an entry with no address blocks EVERY load -- nothing can be compared
      ld_addr=56'h1000; ld_size=2; #1;
      chk("2 unknown addr blocks", ld_block, 1'b1);

      // 3. give it an address; data still pending on preg 7
      a_v=1; a_idx=0; a_addr=56'h2000; a_size=2; a_data_v=0; step; a_v=0; #1;
      chk("3 still no commit (no data)", c_v, 1'b0);

      // 4. a load elsewhere no longer blocks; one overlapping it does
      ld_addr=56'h1000; #1; chk("4 disjoint load passes", ld_block, 1'b0);
      ld_addr=56'h2000; #1; chk("4 same address blocks",  ld_block, 1'b1);
      ld_addr=56'h2002; #1; chk("4 partial overlap blocks", ld_block, 1'b1);
      // THE CAMERA CASE: store y[i] at 0x2000 size W, load y[i+1] at 0x2004 -- adjacent,
      // NOT overlapping. A |delta|>8 test would block this; byte ranges do not.
      ld_addr=56'h2004; #1; chk("4 adjacent 4B load passes", ld_block, 1'b0);

      // 5. data arrives by SNOOPING a writeback, no read port
      wb_v=3'b010; wb_preg[1*PBITS +: PBITS]=9'd7; wb_data[1*64 +: 64]=64'hDEAD_BEEF;
      step; wb_v=0; #1;
      chk("5 snoop captured data", c_v, 1'b1);
      chk("5 data correct", c_data==64'hDEAD_BEEF, 1'b1);
      chk("5 addr correct", c_addr==56'h2000, 1'b1);

      // 6. commit frees the slot
      c_take=1; step; c_take=0; #1;
      chk("6 occ back to 0", occ==0, 1'b1);
      chk("6 nothing blocks now", ld_block, 1'b0);

      // 7. in-order commit: allocate two, fill the SECOND first, head must still go first
      d_dpreg=9'd0; d_alloc=1; step; d_alloc=1; step; d_alloc=0; step;
      chk("7 occ 2", occ==2, 1'b1);
      a_v=1; a_idx=2; a_addr=56'h4000; a_data_v=1; a_data=64'h22; step; a_v=0; #1;
      chk("7 head not ready -> no commit", c_v, 1'b0);
      a_v=1; a_idx=1; a_addr=56'h3000; a_data_v=1; a_data=64'h11; step; a_v=0; #1;
      chk("7 head ready -> commits", c_v && c_data==64'h11, 1'b1);
      c_take=1; step; #1; chk("7 then the second", c_v && c_data==64'h22, 1'b1);
      c_take=1; step; c_take=0; #1;
      chk("7 drained", occ==0, 1'b1);

      // 8. flush clears everything (every live entry is younger than the redirect)
      d_alloc=1; step; d_alloc=1; step; d_alloc=0; #1;
      chk("8 occ 2 before flush", occ==2, 1'b1);
      flush=1; step; flush=0; #1;
      chk("8 flush empties", occ==0, 1'b1);
      chk("8 flush unblocks", ld_block, 1'b0);

      // 9. full buffer refuses allocation
      d_alloc=1; repeat (NENT) step; d_alloc=0; #1;
      chk("9 full", occ==NENT, 1'b1);
      chk("9 d_ready low when full", d_ready, 1'b0);

      // 10. REGRESSION (rule A5). The writeback may land BETWEEN allocate and the address.
      // Arming the snoop at a_v loses it, and a physical register is written back exactly
      // once -- so the entry would never become committable and would wedge the ROB head.
      flush=1; step; flush=0; d_dpreg=9'd5; d_alloc=1; step; d_alloc=0; #1;
      wb_v=3'b001; wb_preg[0 +: PBITS]=9'd5; wb_data[0 +: 64]=64'hFEED_FACE;
      step; wb_v=0; #1;                              // data arrives with NO address yet
      chk("10 no commit without an address", c_v, 1'b0);
      a_v=1; a_idx=0; a_addr=56'h8000; a_size=3; a_data_v=0; step; a_v=0; #1;
      chk("10 early writeback was captured", c_v, 1'b1);
      chk("10 and its value is right", c_data==64'hFEED_FACE, 1'b1);
      c_take=1; step; c_take=0; #1;

      // 11. and the same writeback in the ALLOCATE cycle itself, where dpr[] is still
      // being written on that very edge.
      d_dpreg=9'd6; d_alloc=1;
      wb_v=3'b100; wb_preg[2*PBITS +: PBITS]=9'd6; wb_data[2*64 +: 64]=64'hC0FFEE;
      step; d_alloc=0; wb_v=0; #1;
      a_v=1; a_idx=1; a_addr=56'h9000; a_size=3; a_data_v=0; step; a_v=0; #1;
      chk("11 same-cycle writeback captured", c_v && c_data==64'hC0FFEE, 1'b1);

      $display("---- tb_ooo2_sq pass=%0d fail=%0d", pass, fail);
      if (fail != 0) $fatal(1, "tb_ooo2_sq FAILED");
      $finish;
   end

endmodule
`default_nettype wire
