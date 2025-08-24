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

module smolrv64(input wire        clock,
                input wire        tx_ready_i,
                output reg        tx_valid_o = 0,
                output reg [ 7:0] tx_data_o = 0,
                output reg        halted_o = 0);

// XXX Should I use param/localparam instead?
`define CSR_MSCRATCH 12'h340
`define CSR_MCYCLE   12'hb00
`define CSR_MINSTRET 12'hb02

`define CSR_OP_COPY 0
`define CSR_OP_OR   1
`define CSR_OP_ANDN 2

`define S_FETCH 0
`define S_FETCH_COMPLETE 1
`define S_DECODE 2
`define S_EXECUTE 3
`define S_STORE 4
`define S_LOAD_ALIGN 5
`define S_HANDLE_CSR 6
`define S_ILLEGAL_INSN 7
   reg [3:0]   state = `S_FETCH; // execution state

   // To enable penalty-free unaligned access, memory is split into
   // even and odd word addresses and striped across them.  Any 64-bit
   // word at address A will then be found in {mem1[A/8],mem0[A/8]} if
   // A/4 is even and {mem0[A/8+1],mem1[A/8]} if A/4 is odd.
   reg [31:0]  mem0[4095:0]; initial $readmemh("mem0.hex", mem0, 0, 4095); // 16 KiB
   reg [31:0]  mem1[4095:0]; initial $readmemh("mem1.hex", mem1, 0, 4095); // 16 KiB
   reg [63:0]  rf[31:0];  initial $readmemh("rf.hex", rf, 0, 31);
   reg [63:0]  pc = 0;

   reg [63:0]  mem_addr, s1, s2;
   reg [7:0]   mem_wr_mask;
   wire [31:0] mem_data0 = mem0[mem_addr[63:3] + mem_addr[2]];
   wire [31:0] mem_data1 = mem1[mem_addr[63:3]];

   reg  [ 5:0] write_back_register = 0;
   reg  [63:0] write_back_value;

   reg [63:0]  npc = 0;
   reg [63:0]  imm_i, imm_j, imm_b, imm_u, imm_s, loaded, aligned, csr_arg, csr_read_val, csr_write_val;
   reg [ 9:0]  nzuimm;
   reg [63:0]  imm6;
   reg [63:0]  imm_addi16sp;
   reg [ 4:0]  uimm5w, uimm5d;
   reg [31:0]  sext32;
`ifdef SIMULATE
   reg [127:0] tmp128;
`endif
   reg [ 4:0]  rd, rs1, rs2;
   reg [ 5:0]  shamt;
   reg [11:0]  csrno;
   reg [31:0]  insn;
   wire [63:0] br_offset = {{53{insn[31]}},insn[7],insn[30:25],insn[11:8]};

   reg [ 1:0]  csr_op;

   // CSR state (just a place holder for now)
   reg [63:0]  csr_mscratch = 'hDEADBEEFCAFEF00D,
               csr_mcycle   = 0,
               csr_minstret = ~0; // -1 because we increase it in fetch

   always @(posedge clock) begin
      csr_mcycle <= csr_mcycle + 1;

      if (tx_ready_i)
        tx_valid_o <= 0;

      case (state)
        `S_FETCH: begin
           csr_minstret <= csr_minstret + 1;
           if (write_back_register)
             rf[write_back_register] = write_back_value;

`ifdef DISASS
           // We disassemble the *previous* instruction so we can read the
           // value written to rd

           if (csr_mcycle) begin
           if ((insn & 3) == 3)
             $write("%05d   %x %x ", $time, pc, insn);
           else
             $write("%05d   %x     %x ", $time, pc, insn[15:0]);
           if ((insn & 'hffff) == 'h0000)
             $display("illegal");

           else if ((insn & 'he003) == 'h0000)
             $display("c.addi16sp x%1d,%1d       %x", write_back_register, nzuimm, rf[write_back_register]);
           else if ((insn & 'he003) == 'h2000)
             $display("c.fld   x%1d,%1d(x%1d)    %x UNTESTED", write_back_register, rs1, uimm5d, rf[write_back_register]);
           else if ((insn & 'he003) == 'h4000)
             $display("c.lw    x%1d,%1d(x%1d)    %x UNTESTED", write_back_register, rs1, uimm5w, rf[write_back_register]);
           else if ((insn & 'he003) == 'h6000)
             $display("c.ld    x%1d,%1d(x%1d)    %x UNTESTED", write_back_register, rs1, uimm5d, rf[write_back_register]);
           else if ((insn & 'he003) == 'ha000)
             $display("c.fsd   x%1d,%1d(x%1d)    UNTESTED", rs2, rs1, uimm5d);
           else if ((insn & 'he003) == 'hc000)
             $display("c.sw    x%1d,%1d(x%1d)    UNTESTED", rs2, rs1, uimm5w);
           else if ((insn & 'he003) == 'he000)
             $display("c.sd    x%1d,%1d(x%1d)    UNTESTED", rs2, rs1, uimm5d);

           else if (insn == 1) // XXX untested
             $display("c.nop");
           else if ((insn & 'he003) == 'h0001) // XXX untested
             $display("c.addi  x%1d,%1d           %x", write_back_register, $signed(imm6), rf[write_back_register]);
           else if ((insn & 'he003) == 'h2001) // XXX untested
             $display("c.addiw  x%1d,%1d          %x UNTESTED", write_back_register, $signed(imm6), rf[write_back_register]);
           else if ((insn & 'he003) == 'h4001) // XXX untested
             $display("c.li     x%1d,%1d          %x UNTESTED", write_back_register, $signed(imm6), rf[write_back_register]);
           else if ((insn & 'hef83) == 'h6101) // XXX untested
             $display("c.addi16sp  %1d            %x UNTESTED", write_back_register, imm_addi16sp, rf[write_back_register]);
           else if ((insn & 'he003) == 'h6001) // XXX untested
             $display("c.lui   x%1d,%1d           %x", write_back_register, $signed(imm6), rf[write_back_register]);

           else if ((insn & 'hf07f) == 'h8002) // XXX untested
             $display("c.jr");
           else if ((insn & 'hf003) == 'h8002) // XXX untested
             $display("c.mv    x%1d,x%1d          %x", write_back_register, rs2, rf[write_back_register]);



           else if ((insn & 'h0000007f) == 'h00000037) // LUI
             $display("lui     x%1d,0x%1x        %x", rd, imm_u, rf[rd]);
           else if ((insn & 'h0000007f) == 'h00000017) // AUIPC
             $display("auipc   x%1d,0x%1x    %x", rd, imm_u, rf[rd]);
           else if ((insn & 'h0000007f) == 'h0000006f) // JAL
             $display("jal     x%1d,%1d", rd, imm_j);
           else if ((insn & 'h0000707f) == 'h00000067) // JALR
             $display("jalr    x%1d,%x", rd, rs1);
           else if ((insn & 'h0000707f) == 'h00000063) // BEQ
             $display("beq     x%1d,x%1d,%1d", rs1, rs2, $signed(imm_b));
           else if ((insn & 'h0000707f) == 'h00001063) // BNE
             $display("bne     x%1d,x%1d,%1d", rs1, rs2, $signed(imm_b));
           else if ((insn & 'h0000707f) == 'h00004063) // BLT
             $display("blt     x%1d,x%1d,%1d", rs1, rs2, $signed(imm_b));
           else if ((insn & 'h0000707f) == 'h00005063) // BGE
             $display("bge     x%1d,x%1d,%1d", rs1, rs2, $signed(imm_b));
           else if ((insn & 'h0000707f) == 'h00006063) // BLTU
             $display("bltu    x%1d,x%1d,%1d", rs1, rs2, $signed(imm_b));
           else if ((insn & 'h0000707f) == 'h00007063) // BGEU
             $display("bgeu    x%1d,x%1d,%1d", rs1, rs2, $signed(imm_b));
           else if ((insn & 'h0000707f) == 'h00001073) // CSRRW
             $display("csrrw   x%1d,0x%1x,x%1d    %x", rd, insn[31:20], rs1, rf[rd]);
           else if ((insn & 'h0000707f) == 'h00000003) // LB
             $display("lb      x%1d,%1d(x%1d)    %x", rd, imm_i, rs1, rf[rd]);
           else if ((insn & 'h0000707f) == 'h00001003) // LH
             $display("lh      x%1d,%1d(x%1d)    %x", rd, imm_i, rs1, rf[rd]);
           else if ((insn & 'h0000707f) == 'h00002003) // LW
             $display("lw      x%1d,%1d(x%1d)    %x", rd, imm_i, rs1, rf[rd]);
           else if ((insn & 'h0000707f) == 'h00004003) // LBU
             $display("lbu     x%1d,%1d(x%1d)    %x", rd, imm_i, rs1, rf[rd]);
           else if ((insn & 'h0000707f) == 'h00005003) // LHU
             $display("lhu     x%1d,%1d(x%1d)    %x", rd, imm_i, rs1, rf[rd]);
           else if ((insn & 'h0000707f) == 'h00006003) // LWU
             $display("lwu     x%1d,%1d(x%1d)    %x", rd, imm_i, rs1, rf[rd]);
           else if ((insn & 'h0000707f) == 'h00003003) // LD
             $display("ld      x%1d,%1d(x%1d)    %x", rd, imm_i, rs1, rf[rd]);
           else if ((insn & 'h0000707f) == 'h00000023) // SB
             $display("sb      x%1d,%1d(x%1d)", rs2, imm_s, rs1);
           else if ((insn & 'h0000707f) == 'h00001023) // SH
             $display("sh      x%1d,%1d(x%1d)", rs2, imm_s, rs1);
           else if ((insn & 'h0000707f) == 'h00002023) // SW
             $display("sw      x%1d,%1d(x%1d)", rs2, imm_s, rs1);
           else if ((insn & 'h0000707f) == 'h00003023) // SD
             $display("sd      x%1d,%1d(x%1d)", rs2, imm_s, rs1);
           else if ((insn & 'h0000707f) == 'h00000013) // ADDI
             $display("addi    x%1d,x%1d,0x%1x    %x", rd, rs1, imm_i, rf[rd]);
           else if ((insn & 'h0000707f) == 'h00002013) // SLTI
             $display("slti    x%1d,x%1d,0x%1x    %x", rd, rs1, imm_i, rf[rd]);
           else if ((insn & 'h0000707f) == 'h00003013) // SLTIU
             $display("sltiu   x%1d,x%1d,0x%1x    %x", rd, rs1, imm_i, rf[rd]);
           else if ((insn & 'h0000707f) == 'h00004013) // XORI
             $display("xori    x%1d,x%1d,0x%1x    %x", rd, rs1, imm_i, rf[rd]);
           else if ((insn & 'h0000707f) == 'h00006013) // ORI
             $display("ori     x%1d,x%1d,0x%1x    %x", rd, rs1, imm_i, rf[rd]);
           else if ((insn & 'h0000707f) == 'h00007013) // ANDI
             $display("andi    x%1d,x%1d,0x%1x    %x", rd, rs1, imm_i, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h00000033) // ADD
             $display("add     x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h40000033) // SUB
             $display("sub     x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h00001033) // SLL
             $display("sll     x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h00002033) // SLT
             $display("slt     x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h00003033) // SLTU
             $display("sltu    x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h00004033) // XOR
             $display("xor     x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h00005033) // SRL
             $display("srl     x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h40005033) // SRA
             $display("sra     x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h00006033) // OR
             $display("or      x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h00007033) // AND
             $display("and     x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hf000707f) == 'h0000000f) // FENCE
             $display("fence");
           else if ((insn & 'hf000707f) == 'h8000000f) // FENCE.TSO
             $display("fence.tso");
           else if ((insn & 'hffffffff) == 'h00000073) // ECALL
             $display("ecall");
           else if ((insn & 'hffffffff) == 'h00100073) // EBREAK
             $display("ebreak");
           else if ((insn & 'hfc00707f) == 'h00001013) // SLLI
             $display("slli    x%1d,x%1d,0x%1x    %x", rd, rs1, imm_i, rf[rd]);
           else if ((insn & 'hfc00707f) == 'h00005013) // SRLI
             $display("srli    x%1d,x%1d,0x%1x    %x", rd, rs1, imm_i, rf[rd]);
           else if ((insn & 'hfc00707f) == 'h40005013) // SRAI
             $display("srai    x%1d,x%1d,0x%1x    %x", rd, rs1, imm_i, rf[rd]);
           else if ((insn & 'h0000707f) == 'h0000001b) // ADDIW
             $display("addiw   x%1d,x%1d,0x%1x    %x", rd, rs1, imm_i, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h0000101b) // SLLIW
             $display("srliw   x%1d,x%1d,0x%1x    %x", rd, rs1, imm_i, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h0000501b) // SRLIW
             $display("srliw   x%1d,x%1d,0x%1x    %x", rd, rs1, imm_i, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h4000501b) // SRAIW
             $display("sraiw   x%1d,x%1d,0x%1x    %x", rd, rs1, imm_i, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h0000003b) // ADDW
             $display("addw    x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h4000003b) // SUBW
             $display("subw    x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h0000103b) // SLLW
             $display("sllw    x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h0000503b) // SRLW
             $display("srlw    x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h4000503b) // SRAW
             $display("sraw    x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hffffffff) == 'h0000100f) // FENCE.I
             $display("fence.i");
           else if ((insn & 'h0000707f) == 'h00001073) // CSRRW
             $display("csrrw   x%1d,%3x,x%1d         %x", rd, csrno, rs1, rf[rd]);
           else if ((insn & 'h0000707f) == 'h00002073) // CSRRS
             $display("csrrs   x%1d,%3x,x%1d         %x", rd, csrno, rs1, rf[rd]);
           else if ((insn & 'h0000707f) == 'h00003073) // CSRRC
             $display("csrrc   x%1d,%3x,x%1d         %x", rd, csrno, rs1, rf[rd]);
           else if ((insn & 'h0000707f) == 'h00005073) // CSRRWI
             $display("csrrwi  x%1d,%3x,%1d          %x", rd, csrno, rs1, rf[rd]);
           else if ((insn & 'h0000707f) == 'h00006073) // CSRRSI
             $display("csrrsi  x%1d,%3x,%1d          %x", rd, csrno, rs1, rf[rd]);
           else if ((insn & 'h0000707f) == 'h00007073) // CSRRCI
             $display("csrrci  x%1d,%3x,%1d          %x", rd, csrno, rs1, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h02000033) // MUL
             $display("mul     x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h02001033) // MULH
             $display("mulh    x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h02002033) // MULHSU
             $display("mulhsu  x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h02003033) // MULHU
             $display("mulhu   x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h02004033) // DIV
             $display("div     x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h02005033) // DIVU
             $display("divu    x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h02006033) // REM
             $display("rem     x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h02007033) // REMU
             $display("remu    x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h0200003b) // MULW
             $display("mulw    x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h0200403b) // DIVW
             $display("divw    x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h0200503b) // DIVUW
             $display("divuw   x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h0200603b) // REMW
             $display("remw    x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else if ((insn & 'hfe00707f) == 'h0200703b) // REMUW
             $display("remuw   x%1d,x%1d,x%1d    %x", rd, rs1, rs2, rf[rd]);
           else
             $display("illegal or unsupported instruction");
           end
`endif

           mem_addr <= npc;
           pc <= npc;
           state <= `S_FETCH_COMPLETE;
        end

        `S_FETCH_COMPLETE: begin
           aligned = pc[2] == 0 ? {mem_data1,mem_data0} : {mem_data0,mem_data1};
           //$display("   FETCHED %x: %x", pc, aligned);
           insn = aligned >> (pc[1] * 16);
           //$display("   ALIGNED %x: %x", pc, insn);
           state <= `S_DECODE;
        end

        `S_DECODE: begin
           rd = insn`insn_rd;

           // XXX This is begging for a dedicated test bench
           // The general principle
           case (insn[1:0])
             0: {rs1,rs2} = {5'd8|insn[9:7], 5'd8|insn[4:2]};
             1: {rs1,rs2} = {5'd8|insn[9:7], 5'd8|insn[4:2]};
             2: {rs1,rs2} = {insn[11:7],     insn[6:2]};
             3: {rs1,rs2} = {insn`insn_rs1,  insn`insn_rs2};
           endcase
           // The special case
           if ((insn & 'he003) == 0) rs1 = 2;

           shamt = insn[25:20];

           s1 <= rf[rs1];
           s2 <= rf[rs2];
           write_back_register = 0;
           state <= `S_EXECUTE;
        end

        `S_EXECUTE: begin
           state <= `S_FETCH; // Default next stage

           imm_i = {{52{insn[31]}},insn[31:20]};
           imm_j = {{52{insn[31]}},insn[19:12],insn[20],insn[30:21],1'd0};
           imm_b = {{52{insn[31]}},insn[7],insn[30:25],insn[11:8],1'd0};
           imm_u = {{32{insn[31]}},insn[31:12],12'd0};
           imm_s = {{52{insn[31]}},insn[31:25],insn[11:7]};
           nzuimm = {insn[10:7],insn[12:11],insn[5],insn[6],2'd0};
           uimm5w = {insn[5],insn[12:10],insn[6],2'd0};
           uimm5d = {{59{insn[12]}},insn[6:2]};
           imm6 = {{59{insn[12]}},insn[6:2]};
           imm_addi16sp = {{55{insn[12]}},insn[4:3],insn[5],insn[2],insn[6],4'd0};
           
           csrno = insn[31:20];

           npc = pc + (insn[1:0] == 3 ? 4 : 2);

           // RV64IC decoding
           //
           // The order of instructions [mostly] follows
           // simmerv for ease of reference, who in turn took the
           // ordering from the RISC-V spec.  There is intentionally
           // _no_ overlap in patterns so the order is not important,
           // but we keep the if-else chain in order to catch the
           // unhandled instructions.

           if ((insn & 'he003) == 'h0000) begin // C.ADDI4SPN
              write_back_value = s1 + nzuimm;
              if ((insn & 'hffff) == 0) begin
`ifdef SIMULATE
                 $display("%05d   %x %x illegal C.ADDI4SPN variant", $time, pc, insn);
`endif
                state <= `S_ILLEGAL_INSN;
              end
              else
                write_back_register = 2;
           end

           else if ((insn & 'he003) == 'h0001) begin // C.ADDI
              write_back_register = rs1;
              write_back_value = s1 + imm6;
           end
           // ...

           else if ((insn & 'he003) == 'h4001) begin // C.LI
              write_back_register = insn[11:7];
              write_back_value = imm6;
           end

           else if ((insn & 'he003) == 'h6001) begin // C.ADDI16SP/C.LUI
              if (rs1 == 2) begin
                 write_back_value = s1 + $signed(imm_addi16sp);
                 write_back_register = 2;
              end else begin
                 write_back_value = imm6;
                 write_back_register = insn[11:7];
              end
           end


           // ...
           
           else if ((insn & 'hf003) == 'h8002) begin // C.JR/C.MV XXX untested
              if (rs2 == 0) begin
                 npc = s1 & ~1;
              end else begin
                 write_back_register = rs1;
                 write_back_value = s2;
              end
           end

           else if ((insn & 'h0000007f) == 'h00000037) begin // LUI
              write_back_register = rd;
              write_back_value = imm_u;
           end

           else if ((insn & 'h0000007f) == 'h00000017) begin // AUIPC
              write_back_register = rd;
              write_back_value = pc + imm_u;
           end

           else if ((insn & 'h0000007f) == 'h0000006f) begin // JAL
              write_back_register = rd;
              write_back_value = npc;
              npc = pc + imm_j;
           end

           else if ((insn & 'h0000707f) == 'h00000067) begin // JALR
              write_back_register = rd;
              write_back_value = npc;
              npc = (s1 + imm_i) & ~1;
           end

           else if ((insn & 'h0000707f) == 'h00000063) begin // BEQ
              if (s1 == s2) npc = pc + imm_b;
           end

           else if ((insn & 'h0000707f) == 'h00001063) begin // BNE
              if (s1 != s2) npc = pc + imm_b;
           end

           else if ((insn & 'h0000707f) == 'h00004063) begin // BLT
              if ($signed(s1) < $signed(s2)) npc = pc + imm_b;
           end

           else if ((insn & 'h0000707f) == 'h00005063) begin // BGE
              if ($signed(s1) >= $signed(s2)) npc = pc + imm_b;
           end

           else if ((insn & 'h0000707f) == 'h00006063) begin // BLTU
              if (s1 < s2) npc = pc + imm_b;
           end

           else if ((insn & 'h0000707f) == 'h00007063) begin // BGEU
              if (s1 >= s2) npc = pc + imm_b;
           end

           else if ((insn & 'h0000707f) == 'h00000003) begin // LB
              mem_addr <= s1 + imm_i;
              state <= `S_LOAD_ALIGN;
           end

           else if ((insn & 'h0000707f) == 'h00001003) begin // LH
              mem_addr <= s1 + imm_i;
              state <= `S_LOAD_ALIGN;
           end

           else if ((insn & 'h0000707f) == 'h00002003) begin // LW
              mem_addr <= s1 + imm_i;
              state <= `S_LOAD_ALIGN;
           end

           else if ((insn & 'h0000707f) == 'h00003003) begin // LD
              mem_addr <= s1 + imm_i;
              state <= `S_LOAD_ALIGN;
           end

           else if ((insn & 'h0000707f) == 'h00004003) begin // LBU
              mem_addr <= s1 + imm_i;
              state <= `S_LOAD_ALIGN;
           end

           else if ((insn & 'h0000707f) == 'h00005003) begin // LHU
              mem_addr <= s1 + imm_i;
              state <= `S_LOAD_ALIGN;
           end

           else if ((insn & 'h0000707f) == 'h00006003) begin // LWU
              mem_addr <= s1 + imm_i;
              state <= `S_LOAD_ALIGN;
           end

           else if ((insn & 'h0000707f) == 'h00000023) begin // SB
              mem_addr <= s1 + imm_s;
              mem_wr_mask <= 1;
              state <= `S_STORE;
           end

           else if ((insn & 'h0000707f) == 'h00001023) begin // SH
              mem_addr <= s1 + imm_s;
              mem_wr_mask <= 3;
              state <= `S_STORE;
           end

           else if ((insn & 'h0000707f) == 'h00002023) begin // SW
              mem_addr <= s1 + imm_s;
              mem_wr_mask <= 15;
              state <= `S_STORE;
           end

           else if ((insn & 'h0000707f) == 'h00003023) begin // SD
              mem_addr <= s1 + imm_s;
              mem_wr_mask <= 255;
              state <= `S_STORE;
           end

           else if ((insn & 'h0000707f) == 'h00000013) begin // ADDI
              write_back_register = rd;
              write_back_value = s1 + imm_i;
           end

           else if ((insn & 'h0000707f) == 'h00002013) begin // SLTI
              write_back_register = rd;
              write_back_value = $signed(s1) < $signed(imm_i);
           end

           else if ((insn & 'h0000707f) == 'h00003013) begin // SLTIU
              write_back_register = rd;
              write_back_value = s1 < imm_i;
           end

           else if ((insn & 'h0000707f) == 'h00004013) begin // XORI
              write_back_register = rd;
              write_back_value = s1 ^ imm_i;
           end

           else if ((insn & 'h0000707f) == 'h00006013) begin // ORI
              write_back_register = rd;
              write_back_value = s1 | imm_i;
           end

           else if ((insn & 'h0000707f) == 'h00007013) begin // ANDI
              write_back_register = rd;
              write_back_value = s1 & imm_i;
           end

           else if ((insn & 'hfe00707f) == 'h00000033) begin // ADD
              write_back_register = rd;
              write_back_value = s1 + s2;
           end

           else if ((insn & 'hfe00707f) == 'h40000033) begin // SUB
              write_back_register = rd;
              write_back_value = s1 - s2;
           end

           else if ((insn & 'hfe00707f) == 'h00001033) begin // SLL
              write_back_register = rd;
              write_back_value = s1 << s2[5:0];
           end

           else if ((insn & 'hfe00707f) == 'h00002033) begin // SLT
              write_back_register = rd;
              write_back_value = $signed(s1) < $signed(s2);
           end

           else if ((insn & 'hfe00707f) == 'h00003033) begin // SLTU
              write_back_register = rd;
              write_back_value = s1 < s2;
           end

           else if ((insn & 'hfe00707f) == 'h00004033) begin // XOR
              write_back_register = rd;
              write_back_value = s1 ^ s2;
           end

           else if ((insn & 'hfe00707f) == 'h00005033) begin // SRL
              write_back_register = rd;
              write_back_value = s1 >> s2[5:0];
           end

           else if ((insn & 'hfe00707f) == 'h40005033) begin // SRA
              write_back_register = rd;
              write_back_value = $signed(s1) >> s2[5:0];
           end

           else if ((insn & 'hfe00707f) == 'h00006033) begin // OR
              write_back_register = rd;
              write_back_value = s1 | s2;
           end

           else if ((insn & 'hfe00707f) == 'h00007033) begin // AND
              write_back_register = rd;
              write_back_value = s1 & s2;
           end

           else if ((insn & 'hf000707f) == 'h0000000f) begin // FENCE
              // Nothing to do here
           end

           else if ((insn & 'hf000707f) == 'h8000000f) begin // FENCE.TSO
              // Nothing to do here
           end

           /*
           else if ((insn & 'hffffffff) == 'h00000073) begin // ECALL
            // trap_type = Trap::EnvironmentCallFromUMode + prv
            // tval = pc
            // state <= `S_HANDLE_TRAP; (this is a bit involved and shared)
           end

           else if ((insn & 'hffffffff) == 'h00100073) begin // EBREAK
            // Requires debug mode
           end
           */

           else if ((insn & 'hfc00707f) == 'h00001013) begin // SLLI
              write_back_register = rd;
              write_back_value = s1 << shamt;
           end

           else if ((insn & 'hfc00707f) == 'h00005013) begin // SRLI
              write_back_register = rd;
              write_back_value = s1 >> shamt;
           end

           else if ((insn & 'hfc00707f) == 'h40005013) begin // SRAI
              write_back_register = rd;
              write_back_value = $signed(s1) >> shamt;
           end

           else if ((insn & 'h0000707f) == 'h0000001b) begin // ADDIW
              sext32 = s1[31:0] + imm_i[31:0];
              write_back_register = rd;
              write_back_value = {{32{sext32[31]}},sext32};
           end

           else if ((insn & 'hfe00707f) == 'h0000101b) begin // SLLIW
              sext32 = s1[31:0] << shamt[4:0];
              write_back_register = rd;
              write_back_value = {{32{sext32[31]}},sext32};
           end

           else if ((insn & 'hfe00707f) == 'h0000501b) begin // SRLIW
              sext32 = s1[31:0] >> shamt[4:0];
              write_back_register = rd;
              write_back_value = {{32{sext32[31]}},sext32};
           end

           else if ((insn & 'hfe00707f) == 'h4000501b) begin // SRAIW
              // NB: Yes, this is a crazy instruction with *two*
              // sign-extensions and it does _not_ behave like the MIPS
              // counterpart
              sext32 = $signed(s1[31:0]) >> shamt[4:0];
              write_back_register = rd;
              write_back_value = {{32{sext32[31]}},sext32};
           end

           else if ((insn & 'hfe00707f) == 'h0000003b) begin // ADDW
              sext32 = s1[31:0] + s2[31:0];
              write_back_register = rd;
              write_back_value = {{32{sext32[31]}},sext32};
           end

           else if ((insn & 'hfe00707f) == 'h4000003b) begin // SUBW
              sext32 = s1[31:0] - s2[31:0];
              write_back_register = rd;
              write_back_value = {{32{sext32[31]}},sext32};
           end

           else if ((insn & 'hfe00707f) == 'h0000103b) begin // SLLW
              sext32 = s1[31:0] << s2[4:0];
              write_back_register = rd;
              write_back_value = {{32{sext32[31]}},sext32};
           end

           else if ((insn & 'hfe00707f) == 'h0000503b) begin // SRLW
              sext32 = s1[31:0] >> s2[4:0];
              write_back_register = rd;
              write_back_value = {{32{sext32[31]}},sext32};
           end

           else if ((insn & 'hfe00707f) == 'h4000503b) begin // SRAW
              // NB: Yes, this is a crazy instruction with *two*
              // sign-extensions and it does _not_ behave like the MIPS
              // counterpart
              sext32 = $signed(s1[31:0]) >> s2[4:0];
              write_back_register = rd;
              write_back_value = {{32{sext32[31]}},sext32};
           end

           else if ((insn & 'hffffffff) == 'h0000100f) begin // FENCE.I
              // Nothing to do here [yet]
           end

           else if ((insn & 'h0000707f) == 'h00001073) begin // CSRRW
              // CSRRW and CSRRWI (and only those) do not read the CSR
              // if rd == 0 This matters [only] if the read has side
              // effects (I'm gulty of this part of RISC-V semantics).
              csr_op = `CSR_OP_COPY;
              csr_arg = s1;
              state <= `S_HANDLE_CSR;
           end

           else if ((insn & 'h0000707f) == 'h00002073) begin // CSRRS
              csr_op = `CSR_OP_OR;
              csr_arg = s1;
              state <= `S_HANDLE_CSR;
           end

           else if ((insn & 'h0000707f) == 'h00003073) begin // CSRRC
              csr_op = `CSR_OP_ANDN;
              csr_arg = s1;
              state <= `S_HANDLE_CSR;
           end

           else if ((insn & 'h0000707f) == 'h00005073) begin // CSRRWI
              csr_op = `CSR_OP_COPY;
              csr_arg = rs1;
              state <= `S_HANDLE_CSR;
           end

           else if ((insn & 'h0000707f) == 'h00006073) begin // CSRRSI
              csr_op = `CSR_OP_OR;
              csr_arg = rs1;
              state <= `S_HANDLE_CSR;
           end

           else if ((insn & 'h0000707f) == 'h00007073) begin // CSRRCI
              csr_op = `CSR_OP_ANDN;
              csr_arg = rs1;
              state <= `S_HANDLE_CSR;
           end

           else if ((insn & 'hfe00707f) == 'h02000033) begin // MUL
              write_back_register = rd;
              write_back_value = s1 * s2;
           end

`ifdef SIMULATE
           /*
           else if ((insn & 'hfe00707f) == 'h02001033) begin // MULH
           end

           else if ((insn & 'hfe00707f) == 'h02002033) begin // MULHSU
           end
           */

           else if ((insn & 'hfe00707f) == 'h02003033) begin // MULHU
              tmp128 = {64'd0,s1} * {64'd0,s2};
              $display("%d * %d = %d (%x)", s1, s2, tmp128, tmp128);
              write_back_register = rd;
              write_back_value = tmp128[127:64];
           end

           /*
           else if ((insn & 'hfe00707f) == 'h02004033) begin // DIV
           end

           else if ((insn & 'hfe00707f) == 'h02005033) begin // DIVU
           end

           else if ((insn & 'hfe00707f) == 'h02006033) begin // REM
           end

           else if ((insn & 'hfe00707f) == 'h02007033) begin // REMU
           end
           */

           else if ((insn & 'hfe00707f) == 'h0200003b) begin // MULW
              sext32 = $signed(s1[31:0]) * $signed(s2[31:0]);
              write_back_register = rd;
              write_back_value = {{32{sext32[31]}},sext32};
           end

           /*
           else if ((insn & 'hfe00707f) == 'h0200403b) begin // DIVW
           end

           else if ((insn & 'hfe00707f) == 'h0200503b) begin // DIVUW
           end

           else if ((insn & 'hfe00707f) == 'h0200603b) begin // REMW
           end

           else if ((insn & 'hfe00707f) == 'h0200703b) begin // REMUW
           end
           */
`endif

           else begin
`ifdef SIMULATE
              if (insn[1:0] == 3)
                $display("%05d   %x %x illegal unknown instruction", $time, pc, insn);
              else
                $display("%05d   %x     %x illegal unknown instruction (%1d,%d)", $time, pc, insn[15:0], insn[15:13], insn[1:0]);
              $finish;
`endif
              state <= `S_ILLEGAL_INSN;
           end
        end

        `S_STORE: begin
           mem_wr_mask = mem_wr_mask << (mem_addr % 8);
           s2 = s2 << (8 * (mem_addr % 8));
           // XXX This is wrong for mem_addr[2] == 1
           if (mem_wr_mask[0]) mem0[mem_addr / 8][ 7: 0] <= s2[ 7: 0];
           if (mem_wr_mask[1]) mem0[mem_addr / 8][15: 8] <= s2[15: 8];
           if (mem_wr_mask[2]) mem0[mem_addr / 8][23:16] <= s2[23:16];
           if (mem_wr_mask[3]) mem0[mem_addr / 8][31:24] <= s2[31:24];
           if (mem_wr_mask[4]) mem1[mem_addr / 8][ 7: 0] <= s2[39:32];
           if (mem_wr_mask[5]) mem1[mem_addr / 8][15: 8] <= s2[47:40];
           if (mem_wr_mask[6]) mem1[mem_addr / 8][23:16] <= s2[55:48];
           if (mem_wr_mask[7]) mem1[mem_addr / 8][31:24] <= s2[63:56];
           state <= `S_FETCH;
        end

        `S_LOAD_ALIGN: begin
           aligned = mem_addr[2] == 0 ? {mem_data1,mem_data0} : {mem_data0,mem_data1};
           aligned = aligned >> (mem_addr[2:0] * 8);
           write_back_register = rd;

           if ((insn & 'h0000707f) == 'h00000003) // LB
             write_back_value = {{56{aligned[7]}},aligned[7:0]};
           else if ((insn & 'h0000707f) == 'h00001003) // LH
             write_back_value = {{48{aligned[15]}},aligned[15:0]};
           else if ((insn & 'h0000707f) == 'h00002003) // LW
             write_back_value = {{32{aligned[31]}},aligned[31:0]};
           else if ((insn & 'h0000707f) == 'h00004003) // LBU
             write_back_value = aligned[7:0];
           else if ((insn & 'h0000707f) == 'h00005003) // LHU
             write_back_value = aligned[15:0];
           else if ((insn & 'h0000707f) == 'h00006003) // LWU
             write_back_value = aligned[31:0];
           else if ((insn & 'h0000707f) == 'h00003003) // LD
             write_back_value = aligned;

           state <= `S_FETCH;
        end

        `S_HANDLE_CSR: begin
           state <= `S_FETCH;
           // The CSR is insn[]

           if (rd != 0 || csr_op != `CSR_OP_COPY) begin
              // read the CSR
              case (csrno)
                `CSR_MCYCLE: csr_read_val = csr_mcycle;
                `CSR_MINSTRET: csr_read_val = csr_minstret;
                `CSR_MSCRATCH: csr_read_val = csr_mscratch;
                12'h666: csr_read_val = 0;
                default: begin
`ifdef SIMULATE
                   $display("%05d   %x %x illegal CSR %x (read)", $time, pc, insn, csrno);
`endif
                   state <= `S_ILLEGAL_INSN;
                end
              endcase
           end

           case (csr_op)
             `CSR_OP_COPY: csr_write_val = csr_arg;
             `CSR_OP_OR: csr_write_val = csr_read_val | csr_arg;
             `CSR_OP_ANDN: csr_write_val = csr_read_val & ~csr_arg;
           endcase

           // CSRRS, CSRRC, CSRRSI, and CSRRCI don't write the CSR if rs1 == 0
           if (rs1 != 0 || csr_op == `CSR_OP_COPY) begin
              // write the CSR
              case (csrno)
                `CSR_MCYCLE: csr_mcycle <= csr_write_val;
                `CSR_MINSTRET: csr_minstret <= csr_write_val;
                `CSR_MSCRATCH: csr_mscratch <= csr_write_val;
                12'h666: begin
                   // XXX This is the hacky UART backdoor.  It will be
                   // removed eventually.
                   tx_valid_o <= 1;
                   tx_data_o <= csr_write_val;
                   if (!tx_ready_i)
                     state <= state; // Block here until consumed
                end
                default: begin
`ifdef SIMULATE
                   $display("%05d   %x %x illegal CSR %x (write)", $time, pc, insn, csrno);
`endif
                   state <= `S_ILLEGAL_INSN;
                end
              endcase
           end

           write_back_register = rd;
           write_back_value = csr_read_val;
        end

        `S_ILLEGAL_INSN: begin
           // XXX In future this will raise a trap
           halted_o <= 1;
        end
      endcase
   end
endmodule
