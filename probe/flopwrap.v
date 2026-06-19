`default_nettype none

// Generic flop-in / flop-out timing harness.
//
// Registers a pseudo-random stimulus into the DUT and registers the DUT output,
// so every path that matters is a clean register-to-register path *through*
// probe_dut. Synthesize out-of-context and constrain `clk` (see probe.tcl) to
// read the DUT's Fmax.
//
//   lfsr (reg) -> din_q (reg) -> [ probe_dut ] -> dout_q (reg) -> probe_out (reg)
//
// The LFSR keeps the inputs non-constant so synthesis can't fold the logic away
// (a counter would add its own carry-chain path; an LFSR's feedback is short and
// won't shadow the DUT). probe_out is the only output, so the whole chain is
// preserved. Only `clk` needs a timing constraint; there are no data ports.
module flopwrap #(parameter IN_W = 64, parameter OUT_W = 64)
   (input  wire clk,
    output reg  probe_out = 1'b0);

   // Fibonacci LFSR, seeded non-zero so it never locks at 0.
   reg  [IN_W-1:0] lfsr = {IN_W{1'b1}};
   wire            fb = lfsr[IN_W-1] ^ lfsr[IN_W-2] ^ lfsr[IN_W-4] ^ lfsr[IN_W-5];
   always @(posedge clk)
      lfsr <= {lfsr[IN_W-2:0], fb};

   reg  [IN_W-1:0]  din_q = {IN_W{1'b0}};
   always @(posedge clk)
      din_q <= lfsr;

   wire [OUT_W-1:0] dout_w;
   probe_dut dut (.clk(clk), .din(din_q), .dout(dout_w));

   reg  [OUT_W-1:0] dout_q = {OUT_W{1'b0}};
   always @(posedge clk)
      dout_q <= dout_w;

   always @(posedge clk)
      probe_out <= ^dout_q;
endmodule

`default_nettype wire
