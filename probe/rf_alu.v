`default_nettype none

// Example circuit for the timing probe: a 32x64 register file feeding a simple
// ALU, with the result written back into the register file. The interesting
// critical path is  regfile-read -> ALU -> regfile-write.
//
// Presented with the probe_dut(clk, din, dout) interface so it drops straight
// into flopwrap. The stimulus word `din` carries the control fields; `dout` is
// the ALU result (also the write-back data), so nothing gets trimmed.
module probe_dut #(parameter XLEN = 64,
                   parameter AW   = 5,
                   parameter IN_W = 64,
                   parameter OUT_W = 64)
   (input  wire             clk,
    input  wire [IN_W-1:0]  din,
    output wire [OUT_W-1:0] dout);

   // Unpack control fields from the stimulus word.
   wire [AW-1:0]   rs1     = din[4:0];
   wire [AW-1:0]   rs2     = din[9:5];
   wire [AW-1:0]   rd      = din[14:10];
   wire [3:0]      op      = din[18:15];
   wire            we      = din[19];
   wire            use_imm = din[20];
   wire [XLEN-1:0] imm     = {{(XLEN-32){din[63]}}, din[63:32]};

   // 32x64 register file, two combinational read ports, one synchronous write.
   reg [XLEN-1:0] rf [0:(1<<AW)-1];

   wire [XLEN-1:0] a = rf[rs1];
   wire [XLEN-1:0] b = use_imm ? imm : rf[rs2];

   // Simple ALU.
   reg [XLEN-1:0] y;
   always @* begin
      case (op)
        4'd0:  y = a + b;
        4'd1:  y = a - b;
        4'd2:  y = a & b;
        4'd3:  y = a | b;
        4'd4:  y = a ^ b;
        4'd5:  y = a << b[5:0];
        4'd6:  y = a >> b[5:0];
        4'd7:  y = $signed(a) >>> b[5:0];
        4'd8:  y = {{(XLEN-1){1'b0}}, $signed(a) < $signed(b)};
        4'd9:  y = {{(XLEN-1){1'b0}}, a < b};
        default: y = b;
      endcase
   end

   always @(posedge clk)
      if (we) rf[rd] <= y;

   assign dout = y;
endmodule

`default_nettype wire
