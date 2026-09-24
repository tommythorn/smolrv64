`default_nettype none
// Timing spike (Stage 4 increment 0, docs/PLAN-2026-09-24-frontend-stage4.md): the hit path of a
// dedicated read-only VHPR I$ that takes a new 8-byte-aligned pair every cycle. 2 ways, 64-byte
// lines, even and odd 8-byte chunks in separate block-RAM banks. Cycle t: the registered pair
// address `fa_q` addresses both banks of both ways (a pair at the last chunk of a line takes its
// even chunk from the NEXT line, so each bank has its own line index and its own tag lookup).
// Cycle t+1: the banks' data meets the tag compare (valid, epoch, virtual tag) and the way
// select, and the 16 bytes come out in fetch order with a hit bit, registered. A write port
// fills the arrays so synthesis keeps them.
//   make ooc MODULE=ooo2_icache_spike
module ooo2_icache_spike #(parameter SETS = 512, VTW = 24, EPW = 2)
   (input  wire          clk, input wire reset,
    input  wire [63:0]   fa_in,                 // the pair's first byte address (VA)
    input  wire [EPW-1:0] ep_in,                // the current epoch
    input  wire          wr_en, input wire wr_way, input wire [1:0] wr_bank_mask,
    input  wire [$clog2(SETS)+1:0] wr_row,      // {set, chunk>>1} within a bank
    input  wire [63:0]   wr_data,
    input  wire          tw_en, input wire tw_way, input wire [$clog2(SETS)-1:0] tw_set,
    input  wire [VTW+EPW:0] tw_data,            // {valid, epoch, vtag}
    output reg  [127:0]  out,
    output reg           hit);
   localparam SB = $clog2(SETS);
   localparam RW = SB + 2;                       // bank row: set and the chunk pair within the line
   // cycle t: the pair address, registered
   reg [63:0] fa_q;  reg [EPW-1:0] ep_q;
   always @(posedge clk) begin fa_q <= fa_in; ep_q <= ep_in; end
   wire [60:0] c    = fa_q[63:3];                // first chunk
   wire [60:0] c1   = c + 61'd1;                 // second chunk
   wire [60:0] ce   = c[0] ? c1 : c;             // the even chunk of the pair
   wire [60:0] co   = c[0] ? c  : c1;            // the odd chunk
   // each chunk's line (chunk >> 3), set and bank row
   wire [SB-1:0] set_e = ce[3 +: SB], set_o = co[3 +: SB];
   wire [RW-1:0] row_e = {set_e, ce[2:1]}, row_o = {set_o, co[2:1]};
   wire [VTW-1:0] vt_e = ce[3+SB +: VTW], vt_o = co[3+SB +: VTW];

   // the arrays: data in block RAM (synchronous read), tags in distributed RAM
   (* ram_style = "block" *) reg [63:0] de0 [0:4*SETS-1];
   (* ram_style = "block" *) reg [63:0] de1 [0:4*SETS-1];
   (* ram_style = "block" *) reg [63:0] do0 [0:4*SETS-1];
   (* ram_style = "block" *) reg [63:0] do1 [0:4*SETS-1];
   (* ram_style = "distributed" *) reg [VTW+EPW:0] tg0 [0:SETS-1];
   (* ram_style = "distributed" *) reg [VTW+EPW:0] tg1 [0:SETS-1];
   reg [63:0] de0_q, de1_q, do0_q, do1_q;
   always @(posedge clk) begin
      de0_q <= de0[row_e];  de1_q <= de1[row_e];  do0_q <= do0[row_o];  do1_q <= do1[row_o];
      if (wr_en & ~wr_way & wr_bank_mask[0]) de0[wr_row] <= wr_data;
      if (wr_en &  wr_way & wr_bank_mask[0]) de1[wr_row] <= wr_data;
      if (wr_en & ~wr_way & wr_bank_mask[1]) do0[wr_row] <= wr_data;
      if (wr_en &  wr_way & wr_bank_mask[1]) do1[wr_row] <= wr_data;
      if (tw_en & ~tw_way) tg0[tw_set] <= tw_data;
      if (tw_en &  tw_way) tg1[tw_set] <= tw_data;
   end
   // cycle t+1: the tags for each bank's line (read at the registered sets), compared
   reg [SB-1:0] set_e_q, set_o_q;  reg [VTW-1:0] vt_e_q, vt_o_q;  reg odd_first_q;
   always @(posedge clk) begin set_e_q <= set_e; set_o_q <= set_o; vt_e_q <= vt_e; vt_o_q <= vt_o; odd_first_q <= c[0]; end
   wire [VTW+EPW:0] te0 = tg0[set_e_q], te1 = tg1[set_e_q], to0 = tg0[set_o_q], to1 = tg1[set_o_q];
   wire he0 = te0[VTW+EPW] & (te0[VTW +: EPW] == ep_q) & (te0[VTW-1:0] == vt_e_q);
   wire he1 = te1[VTW+EPW] & (te1[VTW +: EPW] == ep_q) & (te1[VTW-1:0] == vt_e_q);
   wire ho0 = to0[VTW+EPW] & (to0[VTW +: EPW] == ep_q) & (to0[VTW-1:0] == vt_o_q);
   wire ho1 = to1[VTW+EPW] & (to1[VTW +: EPW] == ep_q) & (to1[VTW-1:0] == vt_o_q);
   wire [63:0] ev = he1 ? de1_q : de0_q;
   wire [63:0] od = ho1 ? do1_q : do0_q;
   always @(posedge clk) begin
      out <= odd_first_q ? {ev, od} : {od, ev};  // fetch order: the pair's first chunk low
      hit <= ~reset & (he0 | he1) & (ho0 | ho1);
   end
endmodule
`default_nettype wire
