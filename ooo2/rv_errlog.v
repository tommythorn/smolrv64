`default_nettype none
// ---------------------------------------------------------------------------------------
// rv_errlog -- the black-box recorder for the design's own invariants
//
// docs/rtl-rules.md A1 says invariant checks are ALWAYS ON: no `ifdef, one $fatal per
// "this cannot happen". In SIMULATION. On hardware $fatal is a no-op, nothing else reads
// the condition, and synthesis deletes the cone -- so the bitstream that ships enforces
// none of them. That asymmetry is not academic. The longest simulation this project can
// run is ~1.5e9 cycles; Geekbench on the board reaches ~3e11. A fault rate of 1e-11 per
// cycle passes every gate we own and then kills the board in half an hour, as a wild jump
// with no evidence attached -- which is exactly how C4a step 2 ended
// (wip/cache-selfloop-v2: epc == ra == badaddr, cause 12, root cause never found).
//
// This module is the other half of A1: the same conditions, latched. err[i] rises in the
// cycle invariant i is violated; the bit is sticky for the run, and the FIRST violation
// also records its index and the cycle it fired at. rv_soc_top exposes all of it over
// MMIO, so the monitor prints it in its banner and Linux userland can read it through
// /dev/mem after a crash. That turns "the board died, here is an Oops" into "D$ invariant
// 6 fired at cycle 0x2f1a3b0c00, four billion cycles before the Oops" -- or, just as
// usefully, into "the memory system's invariants all held, look somewhere else".
//
// COST AND TIMING. Every err input is REGISTERED AT ITS SOURCE, so all this adds to an
// existing path is one more flop load on nets that already drive flops; the OR-reduce and
// the priority encode in here are flop-to-flop with a full cycle to do it in. That is the
// same discipline the cache's address-provenance check already follows, for the same
// reason: a diagnostic that costs 0.8 ns on a design with 24 ps of margin is not a
// diagnostic, it is a build failure.
//
// Read-only by design. A recorder you can clear by accident is not a recorder; the bits
// clear on reset and nowhere else.
// ---------------------------------------------------------------------------------------
module rv_errlog #(
   parameter N  = 64,        // invariants logged (see rv_soc_top for the bit assignment)
   parameter CW = 48         // cycle stamp: 48 bits is 19 days at 166.67 MHz
)(
   input  wire           clk,
   input  wire           reset,
   input  wire [N-1:0]   err,        // one bit per invariant, registered by its source
   output reg  [N-1:0]   sticky,     // every invariant that has EVER fired
   output reg  [7:0]     first_idx,  // index of the first to fire (8'hff = none)
   output reg  [CW-1:0]  first_cyc   // and the cycle it fired at
);
   reg [CW-1:0] cyc;
   wire         any   = |err;
   wire         armed = ~(|sticky);          // the first fault has not been recorded yet

   // Lowest set index. Simultaneous violations are one event -- sticky keeps all of them,
   // first_idx names the lowest, and the units are assigned to bit ranges (D$ low) so the
   // name that survives is the one closest to the data.
   integer i;
   reg [7:0] lowest;
   always @* begin
      lowest = 8'hff;
      for (i = N-1; i >= 0; i = i - 1) if (err[i]) lowest = i[7:0];
   end

   initial begin cyc = {CW{1'b0}}; sticky = {N{1'b0}}; first_idx = 8'hff; first_cyc = {CW{1'b0}}; end
   always @(posedge clk)
      if (reset) begin
         cyc <= {CW{1'b0}}; sticky <= {N{1'b0}}; first_idx <= 8'hff; first_cyc <= {CW{1'b0}};
      end else begin
         cyc    <= cyc + 1'b1;
         sticky <= sticky | err;
         if (any & armed) begin first_idx <= lowest; first_cyc <= cyc; end
      end
endmodule
`default_nettype wire
