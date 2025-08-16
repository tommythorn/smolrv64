`default_nettype none

`define rd  [11: 7]
`define rs1 [19:15]
`define rs2 [24:20]
`define csr [31:20]

`ifdef SIMULATE
module smolrv64_tb;
   reg        clock = 1; always #5 clock = !clock;
   wire       tx_ready_o;
   wire       tx_valid_i;
   wire [7:0] tx_data_i;
   wire       halted;
   wire       ftdi_rxd;

   smolrv64 smolrv64_inst     (.clock     (clock),
                               .tx_ready_i(tx_ready_o),
                               .tx_valid_o(tx_valid_i),
                               .tx_data_o (tx_data_i),
                               .halted_o  (halted));

   rs232tx #(1,1) rs232tx_inst(clock, tx_data_i, tx_valid_i, tx_ready_o, ftdi_rxd);

   always @(posedge clock)
     if (tx_ready_o & tx_valid_i)
`ifdef DISASS
       $display("%05d  <<%c>>", $time, tx_data_i);
`else
       $write("%c", tx_data_i);
`endif

   initial begin
      $dumpfile("smolrv64.vcd");
      $dumpvars(0, smolrv64_tb);
      $display("Open the smolrv64.vcd with https://app.surfer-project.org/");
   end
endmodule
`endif

module smolrv64(input             clock,
                input wire        tx_ready_i,
                output reg        tx_valid_o = 0,
                output reg [ 7:0] tx_data_o  = 0,
                output reg        halted_o   = 0);

   reg [31:0] mem[63:0]; initial $readmemh("mem.hex", mem, 0, 63);
   reg [ 5:0] pc = 0;
   reg [63:0] rf[31:0];  initial $readmemh("rf.hex", rf, 0, 31);

   reg [ 5:0] npc = 0;
   wire [31:0] insn = mem[pc];

   wire [63:0] br_offset = {{53{insn[31]}},insn[7],insn[30:25],insn[11:8]};

   always @(posedge clock) begin
      if (tx_ready_i)
        tx_valid_o <= 0;
      npc = pc + 1;

      if ((insn & 32'h0000707f) == 32'h00000013) begin // ADDI

         if (insn`rd != 0) rf[insn`rd] = rf[insn`rs1] + {{20{insn[31]}},insn[31:20]};

`ifdef DISASS
         $display("%05d   %x %x addi x%1d=x%1d,0x%1x    %x", $time,
                  pc, insn, insn`rd, insn`rs1, {{52{insn[31]}},insn[31:20]}, rf[insn`rd]);
`endif

      end else if ((insn & 32'h0000707f) == 32'h00001073) begin // CSRRW

         if (insn`rd != 0) rf[insn`rd] = 0; // XXX CSR value
         tx_valid_o <= 1;
         tx_data_o <= rf[insn`rs1];
         if (tx_ready_i) begin
`ifdef DISASS
            $display("%05d   %x %x csrrw x%1d=0x%1x,x%1d    %x", $time,
                     pc, insn, insn`rd, insn[31:20], insn`rs1, rf[insn`rd]);
`endif
         end else
           npc = pc;

      end else if ((insn & 32'h0000707f) == 32'h00000063) begin // BEQ

         if (rf[insn`rs1] == rf[insn`rs2]) npc = pc + {{53{insn[31]}},insn[7],insn[30:25],insn[11:8]}/2;

`ifdef DISASS
         $display("%05d   %x %x beq x%1d,x%1d", $time, pc, insn, insn`rs1, insn`rs2);
`endif

      end else begin
`ifdef DISASS
         $display("%05d   %x %x illegal or unsupported instruction", $time, pc, insn);
         $finish;
`endif
         npc = pc;
         halted_o = 1;
      end

      pc <= npc;
   end
endmodule
