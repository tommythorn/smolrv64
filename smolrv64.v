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

`define S_FETCH 0
`define S_FETCH_COMPLETE 1
`define S_DECODE 2
`define S_EXECUTE 3
`define S_STORE 4
`define S_LOAD_ALIGN 5
   reg [`S_LOAD_ALIGN:0]   s = 1 << `S_FETCH; // execution state

   reg [63:0]  mem[2047:0]; initial $readmemh("mem.hex", mem, 0, 2047); // 16 KiB
   reg [63:0]  pc = 0;
   reg [63:0]  rf[31:0];  initial $readmemh("rf.hex", rf, 0, 31);

   reg [63:0]  mem_addr, s1, s2;
   reg [7:0]   mem_wr_mask;
   wire [63:0] mem_data = mem[mem_addr[63:3]];

   reg [63:0]  npc = 0;
   reg [63:0]  imm_i, imm_j, imm_b, imm_u, imm_s, loaded, aligned;
   reg [ 4:0]  rd, rs1, rs2;
   reg [31:0]  insn;
   wire [63:0] br_offset = {{53{insn[31]}},insn[7],insn[30:25],insn[11:8]};

   always @(posedge clock) begin
      if (tx_ready_i)
        tx_valid_o <= 0;

`ifdef DISASS
      // We disassemble the *previous* instruction so we can read the
      // value written to rd
      if (s[`S_FETCH]) begin

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
           $display("%05d   %x %x addi    x%1d=x%1d,0x%1x    %x", $time, pc, insn, rd, rs1, imm_i, rf[rd]);
         else if ((insn & 32'h0000707f) == 32'h00001073) // CSRRW
            $display("%05d   %x %x csrrw   x%1d=0x%1x,x%1d    %x", $time, pc, insn, rd, insn[31:20], rs1, rf[rd]);
         else if ((insn & 32'h0000707f) == 32'h00000003) // LB
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
         else if ((insn & 32'h0000707f) == 32'h00000023) // SB
           $display("%05d   %x %x sb      x%1d,%1d(x%1d)", $time, pc, insn, rs2, imm_s, rs1);
          else if ((insn & 32'h0000707f) == 32'h00001023) // SH
           $display("%05d   %x %x sh      x%1d,%1d(x%1d)", $time, pc, insn, rs2, imm_s, rs1);
          else if ((insn & 32'h0000707f) == 32'h00002023) // SW
           $display("%05d   %x %x sw      x%1d,%1d(x%1d)", $time, pc, insn, rs2, imm_s, rs1);
          else if ((insn & 32'h0000707f) == 32'h00003023) // SD
           $display("%05d   %x %x sd      x%1d,%1d(x%1d)", $time, pc, insn, rs2, imm_s, rs1);
         else
           $display("%05d   %x %x illegal or unsupported instruction", $time, pc, insn);

      end
`endif

      if (s[`S_FETCH]) begin
         mem_addr <= npc;
         pc <= npc;
         s <= 1 << `S_FETCH_COMPLETE;
      end

      if (s[`S_FETCH_COMPLETE]) begin
         insn = pc[2] ? mem_data[63:32] : mem_data[31:0];
         s <= 1 << `S_DECODE;
      end

      if (s[`S_DECODE]) begin
         rd = insn`insn_rd;
         rs1 = insn`insn_rs1;
         rs2 = insn`insn_rs2;

         s1 <= rf[rs1];
         s2 <= rf[rs2];
         s <= 1 << `S_EXECUTE;
      end

      if (s[`S_EXECUTE]) begin
         s <= 1 << `S_FETCH; // Default next stage

         imm_i = {{52{insn[31]}},insn[31:20]};
         imm_j = {{32{insn[31]}},insn[19:12],insn[20],insn[30:21],1'd0};
         imm_b = {{53{insn[31]}},insn[7],insn[30:25],insn[11:8],1'd0};
         imm_u = {{32{insn[31]}},insn[31:12],12'd0};
         imm_s = {{52{insn[31]}},insn[31:25],insn[11:7]};

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
            npc = (s1 + imm_i) & ~1;
         end else if ((insn & 32'h0000707f) == 32'h00000063) begin // BEQ
            if (s1 == s2) npc = pc + imm_b;
         end else if ((insn & 32'h0000707f) == 32'h00001063) begin // BNE
            if (s1 != s2) npc = pc + imm_b;
         end else if ((insn & 32'h0000707f) == 32'h00004063) begin // BLT
            if ($signed(s1) < $signed(s2)) npc = pc + imm_b;
         end else if ((insn & 32'h0000707f) == 32'h00005063) begin // BGE
            if ($signed(s1) >= $signed(s2)) npc = pc + imm_b;
         end else if ((insn & 32'h0000707f) == 32'h00006063) begin // BLTU
            if (s1 < s2) npc = pc + imm_b;
         end else if ((insn & 32'h0000707f) == 32'h00007063) begin // BGEU
            if (s1 >= s2) npc = pc + imm_b;
         end else if ((insn & 32'h0000707f) == 32'h00000003) begin // LB
            mem_addr <= s1 + imm_i;
            if (rd != 0) s <= 1 << `S_LOAD_ALIGN;
         end else if ((insn & 32'h0000707f) == 32'h00001003) begin // LH
            mem_addr <= s1 + imm_i;
            if (rd != 0) s <= 1 << `S_LOAD_ALIGN;
         end else if ((insn & 32'h0000707f) == 32'h00002003) begin // LW
            mem_addr <= s1 + imm_i;
            if (rd != 0) s <= 1 << `S_LOAD_ALIGN;
         end else if ((insn & 32'h0000707f) == 32'h00003003) begin // LD
            mem_addr <= s1 + imm_i;
            if (rd != 0) s <= 1 << `S_LOAD_ALIGN;
         end else if ((insn & 32'h0000707f) == 32'h00004003) begin // LBU
            mem_addr <= s1 + imm_i;
            if (rd != 0) s <= 1 << `S_LOAD_ALIGN;
         end else if ((insn & 32'h0000707f) == 32'h00005003) begin // LHU
            mem_addr <= s1 + imm_i;
            if (rd != 0) s <= 1 << `S_LOAD_ALIGN;
         end else if ((insn & 32'h0000707f) == 32'h00006003) begin // LWU
            mem_addr <= s1 + imm_i;
            if (rd != 0) s <= 1 << `S_LOAD_ALIGN;
         end else if ((insn & 32'h0000707f) == 32'h00000023) begin // SB
            mem_addr <= s1 + imm_s;
            mem_wr_mask <= 1;
            s <= 1 << `S_STORE;
         end else if ((insn & 32'h0000707f) == 32'h00001023) begin // SH
            mem_addr <= s1 + imm_s;
            mem_wr_mask <= 3;
            s <= 1 << `S_STORE;
         end else if ((insn & 32'h0000707f) == 32'h00002023) begin // SW
            mem_addr <= s1 + imm_s;
            mem_wr_mask <= 15;
            s <= 1 << `S_STORE;
         end else if ((insn & 32'h0000707f) == 32'h00003023) begin // SD
            mem_addr <= s1 + imm_s;
            mem_wr_mask <= 255;
            s <= 1 << `S_STORE;
         end else if ((insn & 32'h0000707f) == 32'h00000013) begin // ADDI
            if (rd != 0) rf[rd] = s1 + imm_i;
         end else if ((insn & 32'h0000707f) == 32'h00001073) begin // CSRRW
            if (rd != 0) rf[rd] = s1; // XXX CSR value
            tx_valid_o <= 1;
            tx_data_o <= s1;
            if (!tx_ready_i)
              s <= 1 << `S_EXECUTE;
         end else begin
            npc = pc;
            halted_o <= 1;
            s <= 1 << `S_EXECUTE;
         end
      end // if (s[`S_EXECUTE])

      if (s[`S_STORE]) begin
          mem_wr_mask = mem_wr_mask << (mem_addr % 8);
          s2 = s2 << (8 * (mem_addr % 8));
          if (mem_wr_mask[0]) mem[mem_addr / 8][ 7: 0] <= s2[ 7: 0];
          if (mem_wr_mask[1]) mem[mem_addr / 8][15: 8] <= s2[15: 8];
          if (mem_wr_mask[2]) mem[mem_addr / 8][23:16] <= s2[23:16];
          if (mem_wr_mask[3]) mem[mem_addr / 8][31:24] <= s2[31:24];
          if (mem_wr_mask[4]) mem[mem_addr / 8][39:32] <= s2[39:32];
          if (mem_wr_mask[5]) mem[mem_addr / 8][47:40] <= s2[47:40];
          if (mem_wr_mask[6]) mem[mem_addr / 8][55:48] <= s2[55:48];
          if (mem_wr_mask[7]) mem[mem_addr / 8][63:56] <= s2[63:56];
          s <= 1 << `S_FETCH;
      end

      if (s[`S_LOAD_ALIGN]) begin
         // XXX Doesn't handle misaligned data correctly (Technically,
         // it does as long as the data doesn't span two words; it
         // would be easy to support unaligned loads, but that would
         // complicate caches later).
         aligned = mem_data >> (mem_addr[2:0] * 8);

         if ((insn & 32'h0000707f) == 32'h00000003) // LB
           rf[rd] = {{56{aligned[7]}},aligned[7:0]};
         else if ((insn & 32'h0000707f) == 32'h00001003) // LH
           rf[rd] = {{48{aligned[15]}},aligned[15:0]};
         else if ((insn & 32'h0000707f) == 32'h00002003) // LW
           rf[rd] = {{32{aligned[31]}},aligned[31:0]};
         else if ((insn & 32'h0000707f) == 32'h00004003) // LBU
           rf[rd] = aligned[7:0];
         else if ((insn & 32'h0000707f) == 32'h00005003) // LHU
            rf[rd] = aligned[15:0];
         else if ((insn & 32'h0000707f) == 32'h00006003) // LWU
           rf[rd] = aligned[31:0];
         else if ((insn & 32'h0000707f) == 32'h00003003) // LD
           rf[rd] = aligned;

         s <= 1 << `S_FETCH;
      end
   end
endmodule
