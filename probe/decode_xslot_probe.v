`default_nettype none

// Timing-probe wrapper for the cross-slot dependency matrix (decode_xslot.v) at
// the default geometry IW=4, ABITS=6, SBITS=2. IN_W=84, OUT_W=28.
module probe_dut (input  wire        clk,
                  input  wire [83:0] din,
                  output wire [27:0] dout);

   wire [23:0] rs1   = din[23:0];
   wire [3:0]  rs1_v = din[27:24];
   wire [23:0] rs2   = din[51:28];
   wire [3:0]  rs2_v = din[55:52];
   wire [23:0] rd    = din[79:56];
   wire [3:0]  rd_v  = din[83:80];

   wire [3:0] s1_is_slot, s2_is_slot, map_writer;
   wire [7:0] s1_slot, s2_slot;

   decode_xslot #(.IW(4), .ABITS(6), .SBITS(2)) dut
     (.rs1(rs1), .rs1_v(rs1_v), .rs2(rs2), .rs2_v(rs2_v), .rd(rd), .rd_v(rd_v),
      .s1_is_slot(s1_is_slot), .s1_slot(s1_slot),
      .s2_is_slot(s2_is_slot), .s2_slot(s2_slot), .map_writer(map_writer));

   assign dout = {map_writer, s2_slot, s2_is_slot, s1_slot, s1_is_slot};
endmodule

module decode_xslot_probe (input wire clk, output wire probe_out);
   flopwrap #(.IN_W(84), .OUT_W(28)) u (.clk(clk), .probe_out(probe_out));
endmodule

`default_nettype wire
