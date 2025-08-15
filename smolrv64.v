`default_nettype none

`define insn_rd  [11: 7]
`define insn_rs1 [19:15]
`define insn_rs2 [24:20]
`define insn_csr [31:20]

`ifdef SIMULATE
//`define DISASS 1
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

   always @(posedge clock) if (halted) $finish;

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

`define S_RUN 0
`define S_LOAD 1
   reg [1:0]   s = `S_RUN; // execution state

   reg [63:0]  mem[63:0]; initial $readmemh("mem.hex", mem, 0, 63);
   reg [ 7:0]  pc = 0;
   reg [63:0]  rf[31:0];  initial $readmemh("rf.hex", rf, 0, 31);

   reg [ 7:0]  npc = 0;
   reg [63:0]  imm_i, imm_j, imm_b, imm_u, lea, load_word;
   reg [ 4:0]  rd, rs1, rs2;
   reg [31:0]  insn;
   wire [63:0] br_offset = {{53{insn[31]}},insn[7],insn[30:25],insn[11:8]};

   always @(posedge clock) begin
      if (s == `S_LOAD) begin
         load_word = mem[lea[63:3]] >> (lea[2:0] * 8);

         if ((insn & 32'h0000707f) == 32'h00000003) // LB
           rf[rd] = {{56{load_word[7]}},load_word[7:0]};
         else if ((insn & 32'h0000707f) == 32'h00001003) // LH
           rf[rd] = {{48{load_word[15]}},load_word[15:0]};
         else if ((insn & 32'h0000707f) == 32'h00002003) // LW
           rf[rd] = {{32{load_word[31]}},load_word[31:0]};
         else if ((insn & 32'h0000707f) == 32'h00004003) // LBU
           rf[rd] = load_word[7:0];
         else if ((insn & 32'h0000707f) == 32'h00005003) // LHU
            rf[rd] = load_word[15:0];
         else if ((insn & 32'h0000707f) == 32'h00006003) // LWU
           rf[rd] = load_word[31:0];
         else if ((insn & 32'h0000707f) == 32'h00003003) // LD
           rf[rd] = load_word;


`ifdef DISASS
         if ((insn & 32'h0000707f) == 32'h00000003) // LB
           $display("%05d   %x %x lb      x%1d=%1d(x%1d)    %x", $time, pc, insn, rd, imm_i, rs1, rf[rd]);
         else if ((insn & 32'h0000707f) == 32'h00001003) // LH
           $display("%05d   %x %x lh      x%1d=%1d(x%1d)    %x", $time, pc, insn, rd, imm_i, rs1, rf[rd]);
         else if ((insn & 32'h0000707f) == 32'h00002003) // LW
           $display("%05d   %x %x lw      x%1d=%1d(x%1d)    %x", $time, pc, insn, rd, imm_i, rs1, rf[rd]);
         else if ((insn & 32'h0000707f) == 32'h00004003) // LBU
           $display("%05d   %x %x lbu     x%1d=%1d(x%1d)    %x", $time, pc, insn, rd, imm_i, rs1, rf[rd]);
         else if ((insn & 32'h0000707f) == 32'h00005003) // LHU
           $display("%05d   %x %x lhu     x%1d=%1d(x%1d)    %x", $time, pc, insn, rd, imm_i, rs1, rf[rd]);
         else if ((insn & 32'h0000707f) == 32'h00006003) // LWU
           $display("%05d   %x %x lwu     x%1d=%1d(x%1d)    %x", $time, pc, insn, rd, imm_i, rs1, rf[rd]);
         else if ((insn & 32'h0000707f) == 32'h00003003) // LD
           $display("%05d   %x %x ld      x%1d=%1d(x%1d)    %x", $time, pc, insn, rd, imm_i, rs1, rf[rd]);
         else
           $display("%05d   %x %x illegal or unsupported instruction", $time, pc, insn);
`endif

         s = `S_RUN;
      end

      if (s == `S_RUN) begin
         insn = mem[pc[7:3]] >> (pc[2] ? 32 : 0);
         rd = insn`insn_rd;
         rs1 = insn`insn_rs1;
         rs2 = insn`insn_rs2;

         imm_i = {{52{insn[31]}},insn[31:20]};
         imm_j = {{32{insn[31]}},insn[19:12],insn[20],insn[30:21],1'd0};
         imm_b = {{53{insn[31]}},insn[7],insn[30:25],insn[11:8],1'd0};
         imm_u = {{32{insn[31]}},insn[31:12],12'd0};

         if (tx_ready_i)
           tx_valid_o <= 0;
         npc = pc + 4;

         if ((insn & 32'h0000007f) == 32'h00000037) begin // LUI
            if (rd) rf[rd] = imm_u;
         end else if ((insn & 32'h0000007f) == 32'h00000017) begin // AUIPC
            if (rd) rf[rd] = pc + imm_u;
         end else if ((insn & 32'h0000007f) == 32'h0000006f) begin // JAL
            if (rd) rf[rd] = npc;
            npc = pc + imm_j;
         end else if ((insn & 32'h0000707f) == 32'h00000067) begin // JALR
            if (rd) rf[rd] = npc;
            npc = (rf[rs1] + imm_i) & ~1;
         end else if ((insn & 32'h0000707f) == 32'h00000063) begin // BEQ
            if (rf[rs1] == rf[rs2]) npc = pc + imm_b;
         end else if ((insn & 32'h0000707f) == 32'h00001063) begin // BNE
            if (rf[rs1] != rf[rs2]) npc = pc + imm_b;
         end else if ((insn & 32'h0000707f) == 32'h00004063) begin // BLT
            if ($signed(rf[rs1]) < $signed(rf[rs2])) npc = pc + imm_b;
         end else if ((insn & 32'h0000707f) == 32'h00005063) begin // BGE
            if ($signed(rf[rs1]) >= $signed(rf[rs2])) npc = pc + imm_b;
         end else if ((insn & 32'h0000707f) == 32'h00006063) begin // BLTU
            if (rf[rs1] < rf[rs2]) npc = pc + imm_b;
         end else if ((insn & 32'h0000707f) == 32'h00007063) begin // BGEU
            if (rf[rs1] >= rf[rs2]) npc = pc + imm_b;
         end else if ((insn & 32'h0000707f) == 32'h00000003) begin // LB
            lea = rf[rs1] + imm_i;
            if (rd != 0) s = `S_LOAD;
         end else if ((insn & 32'h0000707f) == 32'h00001003) begin // LH
            lea = rf[rs1] + imm_i;
            if (rd != 0) s = `S_LOAD;
         end else if ((insn & 32'h0000707f) == 32'h00002003) begin // LW
            lea = rf[rs1] + imm_i;
            if (rd != 0) s = `S_LOAD;
         end else if ((insn & 32'h0000707f) == 32'h00004003) begin // LBU
            lea = rf[rs1] + imm_i;
            if (rd != 0) s = `S_LOAD;
         end else if ((insn & 32'h0000707f) == 32'h00005003) begin // LHU
            lea = rf[rs1] + imm_i;
            if (rd != 0) s = `S_LOAD;
         end else if ((insn & 32'h0000707f) == 32'h00006003) begin // LWU
            lea = rf[rs1] + imm_i;
            if (rd != 0) s = `S_LOAD;
         end else if ((insn & 32'h0000707f) == 32'h00003003) begin // LD
            lea = rf[rs1] + imm_i;
            if (rd != 0) s = `S_LOAD;
         end else if ((insn & 32'h0000707f) == 32'h00000013) begin // ADDI
            if (rd != 0) rf[rd] = rf[rs1] + imm_i;
         end else if ((insn & 32'h0000707f) == 32'h00001073) begin // CSRRW
            if (rd != 0) rf[rd] = rf[rs1]; // XXX CSR value
            tx_valid_o <= 1;
            tx_data_o <= rf[rs1];
            if (!tx_ready_i)
              npc = pc;
         end else begin
            npc = pc;
            halted_o = 1;
         end


`ifdef DISASS
         if ((insn & 32'h0000007f) == 32'h00000037) // LUI
           $display("%05d   %x %x lui     x%1d=0x%1x    %x", $time, pc, insn, rd, imm_u, rf[rd]);
         else if ((insn & 32'h0000007f) == 32'h00000017) // AUIPC
           $display("%05d   %x %x auipc   x%1d=0x%1x    %x", $time, pc, insn, rd, imm_u, rf[rd]);
         else if ((insn & 32'h0000007f) == 32'h0000006f) // JAL
           $display("%05d   %x %x jal     x%1d=%1d", $time, pc, insn, rd, imm_j);
         else if ((insn & 32'h0000707f) == 32'h00000067) // JALR
           $display("%05d   %x %x jalr    x%1d=%x", $time, pc, insn, rd, rs1);
         else if ((insn & 32'h0000707f) == 32'h00000063) // BEQ
           $display("%05d   %x %x beq     x%1d,x%1d,%1d", $time, pc, insn, rs1, rs2, $signed(imm_b));
         else if ((insn & 32'h0000707f) == 32'h00001063) // BNE
           $display("%05d   %x %x bne     x%1d,x%1d,%1d", $time, pc, insn, rs1, rs2, $signed(imm_b));
         else if ((insn & 32'h0000707f) == 32'h00004063) // BLT
           $display("%05d   %x %x blt     x%1d,x%1d,%1d", $time, pc, insn, rs1, rs2, $signed(imm_b));
         else if ((insn & 32'h0000707f) == 32'h00005063) // BGE
           $display("%05d   %x %x bge     x%1d,x%1d,%1d", $time, pc, insn, rs1, rs2, $signed(imm_b));
         else if ((insn & 32'h0000707f) == 32'h00006063) // BLTU
           $display("%05d   %x %x bltu    x%1d,x%1d,%1d", $time, pc, insn, rs1, rs2, $signed(imm_b));
         else if ((insn & 32'h0000707f) == 32'h00007063) // BGEU
           $display("%05d   %x %x bgeu    x%1d,x%1d,%1d", $time, pc, insn, rs1, rs2, $signed(imm_b));
         else if ((insn & 32'h0000707f) == 32'h00000013) // ADDI
           $display("%05d   %x %x addi    x%1d=x%1d,0x%1x    %x", $time,
                    pc, insn, rd, rs1, imm_i, rf[rd]);
         else if ((insn & 32'h0000707f) == 32'h00001073) begin // CSRRW
            if (tx_ready_i)
              $display("%05d   %x %x csrrw   x%1d=0x%1x,x%1d    %x", $time,
                       pc, insn, rd, insn[31:20], rs1, rf[rd]);
         end else if ((insn & 32'h0000707f) == 32'h00000003) // LB
           begin end
         else if ((insn & 32'h0000707f) == 32'h00001003) // LH
           begin end
         else if ((insn & 32'h0000707f) == 32'h00002003) // LW
           begin end
         else if ((insn & 32'h0000707f) == 32'h00004003) // LBU
           begin end
         else if ((insn & 32'h0000707f) == 32'h00005003) // LHU
           begin end
         else if ((insn & 32'h0000707f) == 32'h00006003) // LWU
           begin end
         else if ((insn & 32'h0000707f) == 32'h00003003) // LD
           begin end
         else
           $display("%05d   %x %x illegal or unsupported instruction", $time, pc, insn);
`endif

         pc <= npc;
      end
   end
endmodule
