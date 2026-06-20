`default_nettype none

// Probe wrapper for the yarvi RV64 ALU (~/github/yarvi/rtl/alu.v).
//
// probe_dut presents the (clk, din, dout) interface flopwrap expects. The
// 128-bit stimulus word supplies two independent 64-bit operands plus the
// control fields; dout XORs every ALU output together so none is trimmed and
// the probe measures the worst path across result / sum / eq / lt / ltu.
//
// Static timing is input-independent, so the random operands only serve to keep
// synthesis from constant-folding the logic away.
module probe_dut (input  wire         clk,
                  input  wire [127:0] din,
                  output wire [63:0]  dout);

   wire [63:0] op1 = din[63:0];
   wire [63:0] op2 = din[127:64];
   wire        sub    = din[0];
   wire        ashr   = din[1];
   wire        w      = din[2];
   wire [2:0]  funct3 = din[5:3];

   wire [63:0] result, sum;
   wire        eq, lt, ltu;

   alu #(.XLEN(64)) dut
     (.sub(sub), .ashr(ashr), .funct3(funct3), .w(w),
      .op1(op1), .op2(op2),
      .result(result), .sum(sum), .eq(eq), .lt(lt), .ltu(ltu));

   assign dout = result ^ sum ^ {61'd0, eq, lt, ltu};
endmodule

// Top: flop-wrap the 128-bit-input DUT (override flopwrap's IN_W).
module alu_probe (input wire clk, output wire probe_out);
   flopwrap #(.IN_W(128), .OUT_W(64)) u (.clk(clk), .probe_out(probe_out));
endmodule

`default_nettype wire
