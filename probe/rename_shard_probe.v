`default_nettype none

// Timing-probe wrapper for one sharded-renamer slice (rename_shard.v) at the
// default geometry: SHARDS=4 (W=4), AREGS=32, NPHYS=64, POOL=16, NCHK=4.
//   IN_W = 107, OUT_W = 25.
// The LFSR drives the decoded instruction, the bundle write/alloc broadcasts,
// the free port and the checkpoint controls; dout collects every renamed output.
module probe_dut (input  wire         clk,
                  input  wire [106:0] din,
                  output wire [24:0]  dout);

   wire [4:0] s1_arch     = din[4:0];
   wire       s1_is_slot  = din[5];
   wire [1:0] s1_slot     = din[7:6];
   wire [4:0] s2_arch     = din[12:8];
   wire       s2_is_slot  = din[13];
   wire [1:0] s2_slot     = din[15:14];
   wire [4:0] d_arch      = din[20:16];
   wire       d_valid     = din[21];
   wire [19:0] wr_arch    = din[41:22];
   wire [23:0] wr_phys    = din[65:42];
   wire [3:0]  wr_valid   = din[69:66];
   wire [23:0] al_phys    = din[93:70];
   wire [5:0]  fr_phys    = din[99:94];
   wire        fr_valid   = din[100];
   wire        chk_create     = din[101];
   wire [1:0]  chk_create_idx = din[103:102];
   wire        chk_restore     = din[104];
   wire [1:0]  chk_restore_idx = din[106:105];

   wire [5:0] ps1, ps2, pdst, pold;
   wire       stall;

   rename_shard #(.SHARDS(4), .SH(0), .AREGS(32), .ABITS(5),
                  .NPHYS(64), .PBITS(6), .POOL(16), .HPTR(4),
                  .SBITS(2), .NCHK(4), .CBITS(2)) dut
     (.clk(clk),
      .s1_arch(s1_arch), .s1_is_slot(s1_is_slot), .s1_slot(s1_slot),
      .s2_arch(s2_arch), .s2_is_slot(s2_is_slot), .s2_slot(s2_slot),
      .d_arch(d_arch), .d_valid(d_valid),
      .wr_arch(wr_arch), .wr_phys(wr_phys), .wr_valid(wr_valid),
      .al_phys(al_phys),
      .fr_phys(fr_phys), .fr_valid(fr_valid),
      .chk_create(chk_create), .chk_create_idx(chk_create_idx),
      .chk_restore(chk_restore), .chk_restore_idx(chk_restore_idx),
      .ps1(ps1), .ps2(ps2), .pdst(pdst), .pold(pold), .stall(stall));

   assign dout = {stall, pold, pdst, ps2, ps1};
endmodule

module rename_shard_probe (input wire clk, output wire probe_out);
   flopwrap #(.IN_W(107), .OUT_W(25)) u (.clk(clk), .probe_out(probe_out));
endmodule

`default_nettype wire
