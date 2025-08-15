`default_nettype none

`define rd  [11: 7]
`define rs1 [19:15]
`define rs2 [24:20]
`define csr [31:20]

module smolrv64;
   reg        clock = 0; always #5 clock = !clock;

   reg [31:0] mem[63:0]; initial $readmemh("mem.hex", mem, 0, 63);
   reg [ 5:0] pc = 0;
   reg [63:0] rf[31:0];  initial $readmemh("rf.hex", rf, 0, 31);

   reg [ 5:0] npc = 0;
   reg [31:0] insn;
   
   wire [63:0] br_offset = {{53{insn[31]}},insn[7],insn[30:25],insn[11:8]};

   always @(posedge clock) begin
      insn = mem[pc];
      npc = pc + 1;
      if ((insn & 32'h0000707f) == 32'h00000013) begin // ADDI
	 if (insn`rd != 0) rf[insn`rd] = rf[insn`rs1] + {{20{insn[31]}},insn[31:20]};
	 $display("   %x %x addi x%1d=x%1d,0x%1x    %x",
		  pc, insn, insn`rd, insn`rs1, {{52{insn[31]}},insn[31:20]}, rf[insn`rd]);
      end else if ((insn & 32'h0000707f) == 32'h00001073) begin // CSRRW
	 if (insn`rd != 0) rf[insn`rd] = 0; // XXX CSR value
	 $display("<<%c>>", rf[insn`rs1]);
	 $display("   %x %x csrrw x%1d=0x%1x,x%1d    %x",
		  pc, insn, insn`rd, insn[31:20], insn`rs1, rf[insn`rd]);
      end else if ((insn & 32'h0000707f) == 32'h00000063) begin // BEQ
	 $display("Branch offset = -%1d insn[31] = %d", -{{53{insn[31]}},insn[7],insn[30:25],insn[11:8]}, insn[31]);
	 if (rf[insn`rs1] == rf[insn`rs2]) npc = pc + {{53{insn[31]}},insn[7],insn[30:25],insn[11:8]}/2;
	 $display("   %x %x beq x%1d,x%1d",
		  pc, insn, insn`rs1, insn`rs2);
      end else begin
	 $display("   %x %x illegal or unsupported instruction", pc, insn);
	 $finish;
      end
      pc = npc;
      // $write("%c", hello[pc]);
   end
endmodule
