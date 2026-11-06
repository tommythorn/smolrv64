`default_nettype none

// THE TWO CLOCKS' CROSSINGS. clk2x is twice clk, phase-aligned: every clk edge is a clk2x edge
// (smolrv64_shell: both through BUFGCE_DIVs of ui_clk). A clk cycle has two clk2x halves.
//
// The rules every crossing in the design follows:
//   - clk logic reads a clk2x flop that changes ONLY at the clk2x edges that are clk edges (the end
//     of a second half), so what it sees is stable for its whole cycle and its paths keep clk's
//     period;
//   - clk2x logic counts a clk pulse in one half (the first: `& ~h2`), as the pulse is there for both;
//   - a valid/ready channel either way transfers only at a clk edge: the clk2x side sees the other
//     side's valid or ready in the second half alone (`& h2`), and holds its own until the transfer.

// h2: the clk2x cycle is the second half of clk's. tg flips at every clk edge and tg2 is tg as clk2x
// last saw it, so they differ in the first half and agree in the second. In a `posedge clk2x` block,
// `if (h2)` is "this edge is a clk edge".
module smolrv64_x2ph
  (input  wire clk,
   input  wire clk2x,
   output wire h2);
   reg tg = 1'b0, tg2 = 1'b0;
   always @(posedge clk)   tg  <= ~tg;
   always @(posedge clk2x) tg2 <= tg;
   assign h2 = (tg2 == tg);
endmodule

// A clk2x pulse, with its payload, for clk logic: sv is high for exactly one clk cycle after a clk2x
// cycle with v, and sd is the LATEST payload of the clk cycle (two pulses in one clk cycle are one
// for the clk side: a later redirect supersedes an earlier, a predictor update is a hint). Both
// change only at clk edges. A pulse in the first half waits in pv/pd for the edge.
module smolrv64_x2f2s #(parameter integer W = 1)
  (input  wire         clk2x,
   input  wire         reset,
   input  wire         h2,
   input  wire         v,
   input  wire [W-1:0] d,
   output reg          sv,
   output reg  [W-1:0] sd,
   output wire         busy);     // a pulse is pending or held: the clk side has not finished with it
   reg         pv = 1'b0;
   reg [W-1:0] pd;
   initial sv = 1'b0;
   always @(posedge clk2x) begin
      if (reset) begin
         pv <= 1'b0;  sv <= 1'b0;
      end else if (h2) begin
         sv <= v | pv;  pv <= 1'b0;
         if (v) sd <= d;  else if (pv) sd <= pd;
      end else if (v) begin
         pv <= 1'b1;  pd <= d;
      end
   end
   assign busy = v | pv | sv;
endmodule

`default_nettype wire
