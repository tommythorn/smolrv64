`default_nettype none

// Probe wrapper for the RVA22 ALU (../alu_rva22/alu.v). 128-bit stimulus word
// supplies two independent operands plus the op/w/uw control; dout XORs every
// output so nothing is trimmed and the probe measures the worst path across all.
module probe_dut (input  wire         clk,
                  input  wire [127:0] din,
                  output wire [63:0]  dout);

   wire [63:0] op1 = din[63:0];
   wire [63:0] op2 = din[127:64];
   wire [5:0]  op  = din[5:0];
   wire        w   = din[6];
   wire        uw  = din[7];

   wire [63:0] result, sum;
   wire        eq, lt, ltu;

   alu #(.XLEN(64)) dut (.op(op), .w(w), .uw(uw), .op1(op1), .op2(op2),
                         .result(result), .sum(sum), .eq(eq), .lt(lt), .ltu(ltu));

   assign dout = result ^ sum ^ {61'd0, eq, lt, ltu};
endmodule

module alu_rva22_probe (input wire clk, output wire probe_out);
   flopwrap #(.IN_W(128), .OUT_W(64)) u (.clk(clk), .probe_out(probe_out));
endmodule

`default_nettype wire
