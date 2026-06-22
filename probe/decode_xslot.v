`default_nettype none

// Cross-slot dependency matrix for the sharded decoder.
//
// This is the O(W^2) intra-bundle dependency resolution, deliberately lifted out
// of the rename loop so rename stays cheap. Given the IW instructions of a bundle
// (slot 0 = oldest), each with two architectural sources and one destination
// (unified int+FP arch space 0..63; per-operand valid bits, so x0/no-operand is
// just rs*_v=0 / rd_v=0), it produces:
//
//   * src redirects: for each source, whether it should read an earlier in-bundle
//     producer (SLOT(j), youngest earlier writer of the same arch reg) instead of
//     the MAP, and which slot j. (RAW bypass.)
//   * map_writer: for each destination, whether it is the *youngest* writer of its
//     arch reg in the bundle (last-writer-wins) and thus the only MAP-visible one.
//     (WAW resolution.)
//   * dst redirects (d_is_slot/d_slot): for each destination, whether an *earlier*
//     in-bundle slot also writes the same arch reg, and which (youngest such). The
//     displaced prior mapping (pold, freed at commit) is then that earlier slot's
//     freshly allocated physreg instead of the MAP entry -- so each writer frees
//     exactly one register and reclamation balances. (Intra-bundle WAW pold.)
//
// Pure combinational; ports are flattened buses (Verilog-2001). In the machine
// each shard runs this over its own slot + the neighbours' broadcast dests; here
// it is the whole bundle so the matrix can be tested/probed standalone.
module decode_xslot
  #(parameter IW    = 4,
    parameter ABITS = 6,
    parameter SBITS = 2)              // = clog2(IW), slot-id width
   (input  wire [IW*ABITS-1:0] rs1,
    input  wire [IW-1:0]       rs1_v,
    input  wire [IW*ABITS-1:0] rs2,
    input  wire [IW-1:0]       rs2_v,
    input  wire [IW*ABITS-1:0] rd,
    input  wire [IW-1:0]       rd_v,
    output wire [IW-1:0]       s1_is_slot,
    output wire [IW*SBITS-1:0] s1_slot,
    output wire [IW-1:0]       s2_is_slot,
    output wire [IW*SBITS-1:0] s2_slot,
    output wire [IW-1:0]       map_writer,
    output wire [IW-1:0]       d_is_slot,
    output wire [IW*SBITS-1:0] d_slot);

   reg [IW-1:0]    s1is, s2is, mw, dis;
   reg [SBITS-1:0] s1sl [0:IW-1];
   reg [SBITS-1:0] s2sl [0:IW-1];
   reg [SBITS-1:0] dsl  [0:IW-1];

   integer i, j;
   reg [ABITS-1:0] a1, a2, di;
   always @* begin
      for (i = 0; i < IW; i = i + 1) begin
         a1 = rs1[i*ABITS +: ABITS];
         a2 = rs2[i*ABITS +: ABITS];
         di = rd [i*ABITS +: ABITS];

         // src1 -> youngest earlier in-bundle writer of the same arch reg
         s1is[i] = 1'b0;  s1sl[i] = {SBITS{1'b0}};
         for (j = 0; j < IW; j = j + 1)
            if ((j < i) && rs1_v[i] && rd_v[j] && (rd[j*ABITS +: ABITS] == a1)) begin
               s1is[i] = 1'b1;  s1sl[i] = j[SBITS-1:0];   // ascending => youngest earlier wins
            end

         // src2
         s2is[i] = 1'b0;  s2sl[i] = {SBITS{1'b0}};
         for (j = 0; j < IW; j = j + 1)
            if ((j < i) && rs2_v[i] && rd_v[j] && (rd[j*ABITS +: ABITS] == a2)) begin
               s2is[i] = 1'b1;  s2sl[i] = j[SBITS-1:0];
            end

         // map_writer: this dst is MAP-visible iff no *younger* slot writes the same reg
         mw[i] = rd_v[i];
         for (j = 0; j < IW; j = j + 1)
            if ((j > i) && rd_v[j] && (rd[j*ABITS +: ABITS] == di))
               mw[i] = 1'b0;

         // dst pold -> youngest *earlier* in-bundle writer of the same arch reg
         dis[i] = 1'b0;  dsl[i] = {SBITS{1'b0}};
         for (j = 0; j < IW; j = j + 1)
            if ((j < i) && rd_v[i] && rd_v[j] && (rd[j*ABITS +: ABITS] == di)) begin
               dis[i] = 1'b1;  dsl[i] = j[SBITS-1:0];   // ascending => youngest earlier wins
            end
      end
   end

   genvar g;
   generate
      for (g = 0; g < IW; g = g + 1) begin : pk
         assign s1_slot[g*SBITS +: SBITS] = s1sl[g];
         assign s2_slot[g*SBITS +: SBITS] = s2sl[g];
         assign d_slot [g*SBITS +: SBITS] = dsl[g];
      end
   endgenerate
   assign s1_is_slot = s1is;
   assign s2_is_slot = s2is;
   assign map_writer = mw;
   assign d_is_slot  = dis;
endmodule

`default_nettype wire
