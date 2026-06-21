`default_nettype none

// Timing-probe wrapper for the renamer (renamer.v) at the default geometry:
//   IW=4, AREGS=32, NPHYS=64, NCHK=4  ->  IN_W=98, OUT_W=97.
//
// The LFSR stimulus word `din` drives the whole instruction bundle, the free
// port and the checkpoint controls; `dout` collects every renamed output (plus
// stall) so nothing is trimmed. The interesting path is the rename combinational
// network: free-list read + intra-bundle source bypass + MAP read/update.
module probe_dut (input  wire        clk,
                  input  wire [97:0] din,
                  output wire [96:0] dout);

   wire [19:0] src1            = din[19:0];
   wire [19:0] src2            = din[39:20];
   wire [19:0] dst             = din[59:40];
   wire [3:0]  valid           = din[63:60];
   wire [23:0] free            = din[87:64];
   wire [3:0]  free_valid      = din[91:88];
   wire        chk_create      = din[92];
   wire [1:0]  chk_create_idx  = din[94:93];
   wire        chk_restore     = din[95];
   wire [1:0]  chk_restore_idx = din[97:96];

   wire [23:0] psrc1, psrc2, pdst, pold;
   wire        stall;

   renamer #(.IW(4), .AREGS(32), .ABITS(5), .NPHYS(64), .PBITS(6),
             .NCHK(4), .CBITS(2)) dut
     (.clk(clk),
      .src1(src1), .src2(src2), .dst(dst), .valid(valid),
      .free(free), .free_valid(free_valid),
      .chk_create(chk_create), .chk_create_idx(chk_create_idx),
      .chk_restore(chk_restore), .chk_restore_idx(chk_restore_idx),
      .psrc1(psrc1), .psrc2(psrc2), .pdst(pdst), .pold(pold), .stall(stall));

   assign dout = {stall, pold, pdst, psrc2, psrc1};
endmodule

module renamer_probe (input wire clk, output wire probe_out);
   flopwrap #(.IN_W(98), .OUT_W(97)) u (.clk(clk), .probe_out(probe_out));
endmodule

`default_nettype wire
