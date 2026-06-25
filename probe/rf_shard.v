`default_nettype none

// One shard's copy of the physical register file. Replicated per shard: this
// copy holds ALL physical registers, organised into SHARDS banks by owner. A
// physical register pr is owned by shard pr[SBITS-1:0] (matches the rename
// freelist: shard SH's pool = {SH, SH+SHARDS, ...}), at bank index
// pr[PBITS-1:SBITS]. Each bank therefore has exactly ONE writer -- the owner
// shard's writeback, delivered on lane b of the SHARDS-wide writeback broadcast.
// So no multiport write arbitration; the cost is S banks per shard * S shards =
// S^2 single-write banks plus the broadcast routing.
//
// Two combinational read ports (this shard's two sources). Physical register 0
// is the architectural zero: it is never written and always reads 0.
module rf_shard
  #(parameter SHARDS = 4,
    parameter SBITS  = 2,
    parameter NPHYS  = 256,
    parameter PBITS  = 8,
    parameter POOL   = 64,
    parameter IDXB   = 6)        // PBITS-SBITS = clog2(POOL)
   (input  wire                    clk,
    // writeback broadcast: lane b is shard b writing one of its own registers
    input  wire [SHARDS-1:0]       wr_valid,
    input  wire [SHARDS*PBITS-1:0] wr_pr,
    input  wire [SHARDS*64-1:0]    wr_val,
    // three read ports (rs1/rs2 + rs3 for the FMA 3rd operand)
    input  wire [PBITS-1:0]        ra1,
    input  wire [PBITS-1:0]        ra2,
    input  wire [PBITS-1:0]        ra3,
    output wire [63:0]             rd1,
    output wire [63:0]             rd2,
    output wire [63:0]             rd3);

   reg [63:0] bank [0:SHARDS-1][0:POOL-1];

   // phys 0..AREGS-1 are the initial architectural regs (reset value 0); init the
   // RF to 0 so an arch reg read before its first write returns 0.
   integer ib, ir;
   initial for (ib = 0; ib < SHARDS; ib = ib + 1)
              for (ir = 0; ir < POOL; ir = ir + 1) bank[ib][ir] = 64'd0;

   // each bank written only by its owner lane (single write port per bank)
   integer b;
   always @(posedge clk)
      for (b = 0; b < SHARDS; b = b + 1)
         if (wr_valid[b] && (wr_pr[b*PBITS +: PBITS] != {PBITS{1'b0}}))
            bank[b][wr_pr[b*PBITS+SBITS +: IDXB]] <= wr_val[b*64 +: 64];

   // read port 1: read every bank at the index, then mux by owner; pr0 reads 0
   wire [SBITS-1:0] b1 = ra1[SBITS-1:0];
   wire [IDXB-1:0]  i1 = ra1[PBITS-1:SBITS];
   wire [SBITS-1:0] b2 = ra2[SBITS-1:0];
   wire [IDXB-1:0]  i2 = ra2[PBITS-1:SBITS];
   wire [SBITS-1:0] b3 = ra3[SBITS-1:0];
   wire [IDXB-1:0]  i3 = ra3[PBITS-1:SBITS];

   wire [SHARDS*64-1:0] r1, r2, r3;
   genvar g;
   generate for (g = 0; g < SHARDS; g = g + 1) begin : rd
      assign r1[g*64 +: 64] = bank[g][i1];
      assign r2[g*64 +: 64] = bank[g][i2];
      assign r3[g*64 +: 64] = bank[g][i3];
   end endgenerate

   assign rd1 = (ra1 == {PBITS{1'b0}}) ? 64'd0 : r1[b1*64 +: 64];
   assign rd2 = (ra2 == {PBITS{1'b0}}) ? 64'd0 : r2[b2*64 +: 64];
   assign rd3 = (ra3 == {PBITS{1'b0}}) ? 64'd0 : r3[b3*64 +: 64];
endmodule

`default_nettype wire
