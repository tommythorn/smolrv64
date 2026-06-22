`default_nettype none

// Timing-probe wrapper for one sharded-renamer slice (rename_shard.v) at the
// default geometry: SHARDS=4 (W=4), AREGS=32, NPHYS=64, POOL=16, NCHK=4.
//   IN_W = 133, OUT_W = 28.
// The LFSR drives the decoded instruction, the bundle write/alloc/pold broadcasts
// and the checkpoint controls; dout collects every renamed output. The bitmap
// freelist (find-first-set over POOL) is now inside this slice, so its delay is
// included in the probe.
module probe_dut (input  wire         clk,
                  input  wire [132:0] din,
                  output wire [27:0]  dout);

   wire [4:0]  s1_arch     = din[4:0];
   wire        s1_is_slot  = din[5];
   wire [1:0]  s1_slot     = din[7:6];
   wire [4:0]  s2_arch     = din[12:8];
   wire        s2_is_slot  = din[13];
   wire [1:0]  s2_slot     = din[15:14];
   wire [4:0]  d_arch      = din[20:16];
   wire        d_valid     = din[21];
   wire        d_is_slot   = din[22];
   wire [1:0]  d_slot      = din[24:23];
   wire [19:0] wr_arch     = din[44:25];
   wire [23:0] wr_phys     = din[68:45];
   wire [3:0]  wr_valid    = din[72:69];
   wire [23:0] al_phys     = din[96:73];
   wire [3:0]  pold_valid  = din[100:97];
   wire [23:0] pold_bus    = din[124:101];
   wire        create      = din[125];
   wire        commit      = din[126];
   wire [1:0]  commit_idx  = din[128:127];
   wire        rollback    = din[129];
   wire [1:0]  rollback_idx = din[131:130];
   wire        reset       = din[132];

   wire [5:0] ps1, ps2, pdst, pold;
   wire       pold_v, stall;
   wire [1:0] cur;

   rename_shard #(.SHARDS(4), .SH(0), .AREGS(32), .ABITS(5),
                  .NPHYS(64), .PBITS(6), .POOL(16), .HPTR(4),
                  .SBITS(2), .NCHK(4), .CBITS(2)) dut
     (.clk(clk), .reset(reset),
      .s1_arch(s1_arch), .s1_is_slot(s1_is_slot), .s1_slot(s1_slot),
      .s2_arch(s2_arch), .s2_is_slot(s2_is_slot), .s2_slot(s2_slot),
      .d_arch(d_arch), .d_valid(d_valid), .d_is_slot(d_is_slot), .d_slot(d_slot),
      .wr_arch(wr_arch), .wr_phys(wr_phys), .wr_valid(wr_valid),
      .al_phys(al_phys), .pold_valid(pold_valid), .pold_bus(pold_bus),
      .create(create), .commit(commit), .commit_idx(commit_idx),
      .rollback(rollback), .rollback_idx(rollback_idx),
      .ps1(ps1), .ps2(ps2), .pdst(pdst), .pold(pold), .pold_v(pold_v),
      .cur(cur), .stall(stall));

   assign dout = {cur, stall, pold_v, pold, pdst, ps2, ps1};
endmodule

module rename_shard_probe (input wire clk, output wire probe_out);
   flopwrap #(.IN_W(133), .OUT_W(28)) u (.clk(clk), .probe_out(probe_out));
endmodule

`default_nettype wire
