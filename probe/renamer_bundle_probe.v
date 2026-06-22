`default_nettype none

// In-context timing probe for the full 4-wide renamer bundle (renamer_bundle.v):
// cross-slot matrix + 4 rename shards (each with its bitmap freelist) + the
// cross-shard broadcast network (alloc + pold). Unified geometry (AREGS=64,
// NPHYS=128). IN_W=92, OUT_W=90.
module probe_dut (input  wire        clk,
                  input  wire [91:0] din,
                  output wire [89:0] dout);

   wire [23:0] rs1   = din[23:0];
   wire [3:0]  rs1_v = din[27:24];
   wire [23:0] rs2   = din[51:28];
   wire [3:0]  rs2_v = din[55:52];
   wire [23:0] rd    = din[79:56];
   wire [3:0]  rd_v  = din[83:80];
   wire        create       = din[84];
   wire        commit       = din[85];
   wire [1:0]  commit_idx   = din[87:86];
   wire        rollback     = din[88];
   wire [1:0]  rollback_idx = din[90:89];
   wire        reset        = din[91];

   wire [27:0] ps1, ps2, pdst;
   wire [3:0]  stall;
   wire [1:0]  cur;

   // cross-slot matrix is computed in decode now; feed its result to the bundle.
   wire [3:0] s1_is_slot, s2_is_slot, map_writer, d_is_slot;
   wire [7:0] s1_slot, s2_slot, d_slot;
   decode_xslot #(.IW(4), .ABITS(6), .SBITS(2)) xs
     (.rs1(rs1), .rs1_v(rs1_v), .rs2(rs2), .rs2_v(rs2_v), .rd(rd), .rd_v(rd_v),
      .s1_is_slot(s1_is_slot), .s1_slot(s1_slot),
      .s2_is_slot(s2_is_slot), .s2_slot(s2_slot), .map_writer(map_writer),
      .d_is_slot(d_is_slot), .d_slot(d_slot));

   // pinned to the historical 128-phys geometry (this probe's bit widths are 7-bit
   // PBITS); the pipeline default is now NPHYS=256.
   renamer_bundle #(.PBITS(7), .NPHYS(128), .POOL(32), .HPTR(5)) dut
     (.clk(clk), .reset(reset), .rs1(rs1), .rs2(rs2), .rd(rd), .rd_v(rd_v),
      .s1_is_slot(s1_is_slot), .s1_slot(s1_slot),
      .s2_is_slot(s2_is_slot), .s2_slot(s2_slot), .map_writer(map_writer),
      .d_is_slot(d_is_slot), .d_slot(d_slot),
      .create(create), .commit(commit), .commit_idx(commit_idx),
      .rollback(rollback), .rollback_idx(rollback_idx),
      .ps1(ps1), .ps2(ps2), .pdst(pdst), .cur(cur), .stall(stall));

   assign dout = {cur, stall, pdst, ps2, ps1};
endmodule

module renamer_bundle_probe (input wire clk, output wire probe_out);
   flopwrap #(.IN_W(92), .OUT_W(90)) u (.clk(clk), .probe_out(probe_out));
endmodule

`default_nettype wire
