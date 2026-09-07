`default_nettype none

// Unified architectural register file for ooo2_core: 64 x 64b, where
// arch 0..31 are the integer registers and 32..63 the FP registers -- the same
// unified numbering decode_operands already emits ({fp_bit, field}), so the
// decoders are reused untouched.
//
// Index 0 (integer x0) reads as zero; f0 (index 32) is a normal register.
// decode_operands already clears rd_v for an integer x0 destination and drives
// unused sources to index 0, so the write port needs no x0 special case and an
// absent operand naturally reads 0.
//
// 3 combinational read ports (rs1/rs2 + rs3 for the FMA family), 1 synchronous
// write port -> infers as LUTRAM (a LUT6 is natively a 64-deep RAM, so a 64-entry
// file fills it exactly; the OoO probe measured this shape at ~0 extra area).
//
// There is NO internal write-before-read forwarding: the in-order pipe reads in X
// and writes at the end of M, and the one bypass level (M -> X) lives in ooo2_exec.
// An instruction two ahead has already landed here by the time X reads it.
module rv_regfile
  #(parameter AREGS = 64,
    parameter ABITS = 6)
   (input  wire             clk,
    input  wire [ABITS-1:0] rs1,
    output wire [63:0]      rs1_val,
    input  wire [ABITS-1:0] rs2,
    output wire [63:0]      rs2_val,
    input  wire [ABITS-1:0] rs3,
    output wire [63:0]      rs3_val,
        input  wire             we,
    input  wire [ABITS-1:0] wa,
    input  wire [63:0]      wd,
    input  wire             we2,     // the second retire of the cycle (item 10c), younger
    input  wire [ABITS-1:0] wa2,
    input  wire [63:0]      wd2);

   reg [63:0] r [0:AREGS-1];
   integer i;
   initial begin
      for (i = 0; i < AREGS; i = i + 1) r[i] = 64'd0;
      // Boot seed: a1 (x11) = DTB pointer, mirroring smolrv64's rf.hex and rf_shard's
      // equivalent. Sim-only (an initial + $value$plusargs; synth ignores it) and inert
      // unless a TB passes +a1= -- so a harness that resets straight to OpenSBI, which
      // expects the DTB pointer in a1, can seed it.
      begin : seed reg [63:0] a1v;
         if ($value$plusargs("a1=%h", a1v)) r[11] = a1v;
      end
   end

   // No read mux for x0: entry 0 is initialized to zero and can never be written, so
   // it simply STAYS zero. `decode_operands` clears rd_v for an integer x0 destination
   // (rd_v = ... && (rd_fp || rdf != 0)) and that write port is the only one, so wa==0
   // never arrives with we set. Absent source operands are driven to index 0 by the
   // same decoder and therefore read 0 for free.
   assign rs1_val = r[rs1];
   assign rs2_val = r[rs2];
   assign rs3_val = r[rs3];

   always @(posedge clk) begin
      if (we)  r[wa]  <= wd;
      if (we2) r[wa2] <= wd2;      // ordered after the first: the younger retire's value stands
   end

`ifndef SYNTHESIS
   // The x0-stays-zero invariant above is load-bearing and would fail silently, so
   // check it in simulation. Costs nothing in synthesis.
   always @(posedge clk)
      if (we && wa == {ABITS{1'b0}})
         $display("*** rv_regfile: write to x0 (val=%h) -- x0 invariant broken", wd);
`endif
endmodule

`default_nettype wire
