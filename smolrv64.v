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
/*
      $dumpfile("smolrv64.vcd");
      $dumpvars(0, smolrv64_tb);
      $display("Open the smolrv64.vcd with https://app.surfer-project.org/");
*/
`ifndef NO_TIMEOUT
      #400000
`ifdef RISCV_TESTS
      $display("Test Failed with TIMEOUT");
`endif
      $finish;
`endif
   end
endmodule
`endif

module smolrv64(input wire        clock,
                input wire        tx_ready_i,
                output reg        tx_valid_o = 0,
                output reg [ 7:0] tx_data_o = 0,
                output reg        halted_o = 0);

// XXX Should I use param/localparam instead?
`define TRAP_INSTRUCTION_ADDRESS_MISALIGNED      0
`define TRAP_INSTRUCTION_ACCESS_FAULT            1
`define TRAP_ILLEGAL_INSTRUCTION                 2
`define TRAP_BREAKPOINT                          3
`define TRAP_LOAD_ADDRESS_MISALIGNED             4
`define TRAP_LOAD_ACCESS_FAULT                   5
`define TRAP_STORE_ADDRESS_MISALIGNED            6
`define TRAP_STORE_ACCESS_FAULT                  7
`define TRAP_ENVIRONMENT_CALL_FROM_U_MODE        8
`define TRAP_ENVIRONMENT_CALL_FROM_S_MODE        9
// 10 is reserved
`define TRAP_ENVIRONMENT_CALL_FROM_M_MODE       11
`define TRAP_INSTRUCTIONPAGE_FAULT              12
`define TRAP_LOAD_PAGE_FAULT                    13
// 14 is reserved
`define TRAP_STORE_PAGE_FAULT                   15

`define TRAP_USER_SOFTWARE_INTERRUPT            100
`define TRAP_SUPERVISOR_SOFTWARE_INTERRUPT      101
`define TRAP_MACHINE_SOFTWARE_INTERRUPT         103

`define TRAP_USER_TIMER_INTERRUPT               104
`define TRAP_SUPERVISOR_TIMER_INTERRUPT         105
`define TRAP_MACHINE_TIMER_INTERRUPT            107

`define TRAP_USER_EXTERNAL_INTERRUPT            108
`define TRAP_SUPERVISOR_EXTERNAL_INTERRUPT      109
`define TRAP_MACHINE_EXTERNAL_INTERRUPT         111


`define CSR_SCOUNTEREN 12'h106
`define CSR_SATP       12'h180
`define CSR_MSTATUS    12'h300
`define CSR_MISA       12'h301
`define CSR_MEDELEG    12'h302
`define CSR_MIDELEG    12'h303
`define CSR_MIE        12'h304
`define CSR_MTVEC      12'h305
`define CSR_MCOUNTEREN 12'h306
`define CSR_MSCRATCH   12'h340
`define CSR_MEPC       12'h341
`define CSR_MCAUSE     12'h342
`define CSR_MTVAL      12'h343
`define CSR_MIP        12'h344
`define CSR_MCYCLE     12'hb00
`define CSR_MINSTRET   12'hb02
`define CSR_CYCLE      12'hc00
`define CSR_INSTRET    12'hc02
`define CSR_MHARTID    12'hf14
`define CSR_MVENDORID  12'hf11
`define CSR_MARCHID    12'hf12
`define CSR_MIMPID     12'hf13

`define CSR_OP_COPY 0
`define CSR_OP_OR   1
`define CSR_OP_ANDN 2

`define S_FETCH          0
`define S_FETCH_COMPLETE 1
`define S_DECODE         2
`define S_EXECUTE        3
`define S_STORE          4
`define S_LOAD_ALIGN     5
`define S_HANDLE_CSR     6
`define S_EXCEPTION      7
`define S_MUL_RUNNING    8
`define S_DIV_RUNNING    9
`define S_AMO           10
`define S_LAST_STATE    15 // Reminder to update the width of state

   reg [3:0]   state = `S_FETCH; // execution state

`define MEM_START 'h80000000
`define MEM_SIZE_LG2 15
`define MEM_SIZE (1 << `MEM_SIZE_LG2)


   // To enable penalty-free unaligned access, memory is split into
   // even and odd 64b word addresses and striped across them.  Any
   // 64-bit word at address A will then be found in
   // {mem1[A/16],mem0[A/16]} if A/8 is even and
   // {mem0[A/16+1],mem1[A/16]} if A/8 is odd.
   reg  [63:0] mem0[`MEM_SIZE/32-1:0]; initial $readmemh("mem0.hex", mem0, 0, `MEM_SIZE/32-1);
   reg  [63:0] mem1[`MEM_SIZE/32-1:0]; initial $readmemh("mem1.hex", mem1, 0, `MEM_SIZE/32-1);
   reg  [63:0] rf[31:0];  initial $readmemh("rf.hex", rf, 0, 31);
   reg  [63:0] pc = 0;
   reg  [ 1:0] prv = 3;

   reg  [`MEM_SIZE_LG2-4:0] mem_addr0, mem_addr1;
   reg  [63:0] mem_addr, s1, s2;
   reg  [15:0] mem_wr_mask;
   wire [63:0] mem_data0 = mem0[mem_addr0];
   wire [63:0] mem_data1 = mem1[mem_addr1];

   reg  [ 5:0] write_back_register = 0;
   reg  [63:0] write_back_value;

   reg  [63:0] npc = `MEM_START;
   reg  [127:0] aligned;
   reg  [63:0] imm_i, imm_j, imm_b, imm_u, imm_s, csr_arg, csr_read_val, csr_write_val;
   reg  [63:0] c_imm12_8_109_6_7_2_11_53_x2;
   reg  [63:0] c_imm12_65_2_1110_43_x2;
   reg  [ 9:0] c_nzuimm107_1211_5_6_x4;
   reg  [63:0] c_imm12_62;
   reg  [63:0] c_imm12_43_5_2_6_x16;
   reg  [ 4:0] c_uimm5_1210_6_x4;
   reg  [ 8:0] c_uimm42_12_65_x8, c_uimm97_1210_x8;
   reg  [ 7:0] c_uimm32_12_64_x4, c_uimm87_129_x4;
   reg  [ 8:0] c_uimm65_1210_x8;
   reg  [ 5:0] c_uimm12_62;
   reg  [31:0] sext32;
   reg  [ 2:0] load_size_lg2 = 'hx; // 0 = B, 1 = H, 2 = W, 3 = D, +4 for sign-extend
`ifdef SIMULATE
   reg  [127:0] tmp128;
`endif
   reg  [ 4:0] rd, rs1, rs2;
   reg  [ 5:0] shamt;
   reg  [11:0] csrno;
   reg  [31:0] insn = 0;
   wire [63:0] br_offset = {{53{insn[31]}},insn[7],insn[30:25],insn[11:8]};

   reg [ 1:0]  csr_op;

   // CSR state (just a place holder for now)
   reg [63:0]  csr_mie      = 0,
               csr_mtvec    = 0,
               csr_mscratch = 'hDEADBEEFCAFEF00D,
               csr_mepc     = 0,
               csr_mcause   = 0,
               csr_mtval    = 0,
               csr_mip      = 0,
               csr_mcycle   = 0,
               csr_minstret = ~0; // -1 because we increase it in fetch

   // MSTATUS subfields
   // Global interrupt-enable bits
   reg         uie = 0, sie = 0, mie =0;
   // xPIE holds the value of the interrupt-enable bit active prior to the trap
   reg         upie = 0, spie = 0, mpie = 0;
   // xPP holds the previous privilege mode.
   reg         spp = 0;
   reg [  1:0] mpp;
   // FS encodes the status of the FP unit, including fcsr and FP regs
   reg [  1:0] fs = 0, xs = 0;
   // When MPRV=1, load and store memory addresses are translated and
   // protected as though the current privilege mode were set to MPP.
   reg         mprv = 0;
   // When SUM=0, S-mode memory accesses to pages that are accessible
   // by U-mode (U=1 in Figure 4.15) will fault.
   reg         sum;
   // MXR modifies the privilege with which loads access virtual
   // memory.  When 0, only loads from pages marked readable will
   // succeed.  When 1, loads from pages marked either readable or
   // executable (R=1 or X=1) will succeed.
   reg         mxr = 0;
   // Trap Virtual Memory, Timeout Wait, Trap SRET
   reg         tvm = 0, tw = 0, tsr = 0;
   wire [ 1:0] uxl = 2, sxl = 2; // 64-bit user and supervisor mode
   // "Some Dirty"
   wire        sd = fs == 3 || xs == 3;

   reg [ 63:0] mul_b;
   reg [127:0] mul_a, mul_p = 0;
   reg         mul_output_sext32 = 0;
   reg         mul_output_negate = 0;
   reg         mul_output_high_part = 0;
   reg [6:0]   div_count;

   reg [63:0]  reservation = ~0;
   reg         do_atomic = 0;
   reg         csr_access_failure = 0;

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
           // We disassemble the *previous* instruction so we can read
           // the value written to rd (but when csr_mcycle == 0 we
           // have no previous instruction)

           if (csr_mcycle) begin
           if ((insn & 3) == 3)
             $write("%05d   %1d %x %x ", $time, prv, pc, insn);
           else
             $write("%05d   %1d %x     %x ", $time, prv, pc, insn[15:0]);

           if ((insn & 'hffff) == 'h0000)
             $write("illegal instruction");
              // Quadrant 0
           else if ((insn & 'he003) == 'h0000)
             $write("c.addi4spn x%1d,%1d", write_back_register, c_nzuimm107_1211_5_6_x4);
           else if ((insn & 'he003) == 'h2000)
             $write("c.fld   x%1d,%1d(x%1d)            UNTESTED", write_back_register, rs1, c_uimm65_1210_x8);
           else if ((insn & 'he003) == 'h4000)
             $write("c.lw    x%1d,%1d(x%1d)", write_back_register, c_uimm5_1210_6_x4, rs1);
           else if ((insn & 'he003) == 'h6000)
             $write("c.ld    x%1d,%1d(x%1d)", write_back_register, c_uimm65_1210_x8, rs1);
           else if ((insn & 'he003) == 'ha000)
             $write("c.fsd   x%1d,%1d(x%1d)    UNTESTED", rs2, c_uimm65_1210_x8, rs1);
           else if ((insn & 'he003) == 'hc000)
             $write("c.sw    x%1d,%1d(x%1d)", rs2, c_uimm5_1210_6_x4, rs1);
           else if ((insn & 'he003) == 'he000)
             $write("c.sd    x%1d,%1d(x%1d)", rs2, c_uimm65_1210_x8, rs1);

              // Quadrant 1
           else if ((insn & 'hffff) == 'h0001)
             $write("c.nop");
           else if ((insn & 'he003) == 'h0001)
             $write("c.addi  x%1d,%1d", write_back_register, $signed(c_imm12_62));
           else if ((insn & 'he003) == 'h2001)
             $write("c.addiw x%1d,%1d", write_back_register, $signed(c_imm12_62));
           else if ((insn & 'he003) == 'h4001)
             $write("c.li    x%1d,%1d", write_back_register, $signed(c_imm12_62));
           else if ((insn & 'hef83) == 'h6101)
             $write("c.addi16sp x%1d,%x", write_back_register, c_imm12_43_5_2_6_x16);
           else if ((insn & 'he003) == 'h6001)
             $write("c.lui   x%1d,%1d", write_back_register, $signed(c_imm12_62)<<12);
           else if ((insn & 'hec03) == 'h8001)
             $write("c.srli  x%1d,%1d", write_back_register, c_uimm12_62);
           else if ((insn & 'hec03) == 'h8401)
             $write("c.srai  x%1d,%1d", write_back_register, c_uimm12_62);
           else if ((insn & 'hec03) == 'h8801)
             $write("c.andi  x%1d,%x", write_back_register, c_imm12_62);
           else if ((insn & 'hfc63) == 'h8c01)
             $write("c.sub   x%1d,x%1d", write_back_register, rs2);
           else if ((insn & 'hfc63) == 'h8c21)
             $write("c.xor   x%1d,x%1d", write_back_register, rs2);
           else if ((insn & 'hfc63) == 'h8c41)
             $write("c.or    x%1d,x%1d", write_back_register, rs2);
           else if ((insn & 'hfc63) == 'h8c61)
             $write("c.and   x%1d,x%1d", write_back_register, rs2);
           else if ((insn & 'hfc63) == 'h9c01)
             $write("c.subw  x%1d,x%1d", write_back_register, rs2);
           else if ((insn & 'hfc63) == 'h9c21)
             $write("c.addw  x%1d,x%1d", write_back_register, rs2);
           else if ((insn & 'he003) == 'ha001)
             $write("c.j     %8x", pc + $signed(c_imm12_8_109_6_7_2_11_53_x2));
           else if ((insn & 'he003) == 'hc001)
             $write("c.beqz  x%1d,%8x", rs1, pc + $signed(c_imm12_65_2_1110_43_x2));
           else if ((insn & 'he003) == 'he001)
             $write("c.bnez  x%1d,%8x", rs1, pc + $signed(c_imm12_65_2_1110_43_x2));

              // Quadrant 2
           else if ((insn & 'he003) == 'h0002)
             $write("c.slli  x%1d,%1d", write_back_register, c_imm12_62[5:0]);
           else if ((insn & 'he003) == 'h2002)
             $write("c.fldsp x%1d,%1d(x2)       UNTESTED", write_back_register, c_uimm42_12_65_x8);
           else if ((insn & 'he003) == 'h4002)
             $write("c.lwsp  x%1d,%1d(x2)", write_back_register, c_uimm32_12_64_x4);
           else if ((insn & 'he003) == 'h6002)
             $write("c.ldsp  x%1d,%1d(x2)", write_back_register, c_uimm42_12_65_x8);
           else if ((insn & 'hf07f) == 'h8002)
             $write("c.jr    x%1d", rs1);
           else if ((insn & 'hf003) == 'h8002)
             $write("c.mv    x%1d,x%1d", write_back_register, rs2);
           else if ((insn & 'hffff) == 'h9002)
             $write("c.ebreak UNTESTED");
           else if ((insn & 'hf07f) == 'h9002)
             $write("c.jalr  x1,0(x%1d)", rs1);
           else if ((insn & 'hf003) == 'h9002)
             $write("c.add   x%1d,x%1d", write_back_register, rs2);
           else if ((insn & 'he003) == 'ha002)
             $write("c.fsdsp x%1d,%1d(x2)       UNTESTED", rs2, c_uimm97_1210_x8);
           else if ((insn & 'he003) == 'hc002)
             $write("c.swsp  x%1d,%1d(x2)", rs2, c_uimm87_129_x4);
           else if ((insn & 'he003) == 'he002)
             $write("c.sdsp  x%1d,%1d(x2)", rs2, c_uimm97_1210_x8);



           else if ((insn & 'h0000007f) == 'h00000037) // LUI
             $write("lui     x%1d,0x%x", rd, imm_u);
           else if ((insn & 'h0000007f) == 'h00000017) // AUIPC
             $write("auipc   x%1d,0x%x", rd, imm_u);
           else if ((insn & 'h0000007f) == 'h0000006f) // JAL
             $write("jal     x%1d,%x", rd, pc + imm_j);
           else if ((insn & 'h0000707f) == 'h00000067) // JALR
             $write("jalr    x%1d,%x", rd, rs1);
           else if ((insn & 'h0000707f) == 'h00000063) // BEQ
             $write("beq     x%1d,x%1d,%x", rs1, rs2, pc + $signed(imm_b));
           else if ((insn & 'h0000707f) == 'h00001063) // BNE
             $write("bne     x%1d,x%1d,%x", rs1, rs2, pc + $signed(imm_b));
           else if ((insn & 'h0000707f) == 'h00004063) // BLT
             $write("blt     x%1d,x%1d,%x", rs1, rs2, pc + $signed(imm_b));
           else if ((insn & 'h0000707f) == 'h00005063) // BGE
             $write("bge     x%1d,x%1d,%x", rs1, rs2, pc + $signed(imm_b));
           else if ((insn & 'h0000707f) == 'h00006063) // BLTU
             $write("bltu    x%1d,x%1d,%x", rs1, rs2, pc + $signed(imm_b));
           else if ((insn & 'h0000707f) == 'h00007063) // BGEU
             $write("bgeu    x%1d,x%1d,%x", rs1, rs2, pc + $signed(imm_b));
           else if ((insn & 'h0000707f) == 'h00000003) // LB
             $write("lb      x%1d,%1d(x%1d)", rd, imm_i, rs1);
           else if ((insn & 'h0000707f) == 'h00001003) // LH
             $write("lh      x%1d,%1d(x%1d)", rd, imm_i, rs1);
           else if ((insn & 'h0000707f) == 'h00002003) // LW
             $write("lw      x%1d,%1d(x%1d)", rd, imm_i, rs1);
           else if ((insn & 'h0000707f) == 'h00004003) // LBU
             $write("lbu     x%1d,%1d(x%1d)", rd, imm_i, rs1);
           else if ((insn & 'h0000707f) == 'h00005003) // LHU
             $write("lhu     x%1d,%1d(x%1d)", rd, imm_i, rs1);
           else if ((insn & 'h0000707f) == 'h00006003) // LWU
             $write("lwu     x%1d,%1d(x%1d)", rd, imm_i, rs1);
           else if ((insn & 'h0000707f) == 'h00003003) // LD
             $write("ld      x%1d,%1d(x%1d)", rd, imm_i, rs1);
           else if ((insn & 'h0000707f) == 'h00000023) // SB
             $write("sb      x%1d,%1d(x%1d)", rs2, imm_s, rs1);
           else if ((insn & 'h0000707f) == 'h00001023) // SH
             $write("sh      x%1d,%1d(x%1d)", rs2, imm_s, rs1);
           else if ((insn & 'h0000707f) == 'h00002023) // SW
             $write("sw      x%1d,%1d(x%1d)", rs2, imm_s, rs1);
           else if ((insn & 'h0000707f) == 'h00003023) // SD
             $write("sd      x%1d,%1d(x%1d)", rs2, imm_s, rs1);
           else if ((insn & 'h0000707f) == 'h00000013 && rs1 == 0) // LI (ADDI)
             $write("li      x%1d,0x%x", rd, imm_i);
           else if ((insn & 'h0000707f) == 'h00000013) // ADDI
             $write("addi    x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'h0000707f) == 'h00002013) // SLTI
             $write("slti    x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'h0000707f) == 'h00003013) // SLTIU
             $write("sltiu   x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'h0000707f) == 'h00004013) // XORI
             $write("xori    x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'h0000707f) == 'h00006013) // ORI
             $write("ori     x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'h0000707f) == 'h00007013) // ANDI
             $write("andi    x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'hfe00707f) == 'h00000033) // ADD
             $write("add     x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h40000033) // SUB
             $write("sub     x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h00001033) // SLL
             $write("sll     x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h00002033) // SLT
             $write("slt     x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h00003033) // SLTU
             $write("sltu    x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h00004033) // XOR
             $write("xor     x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h00005033) // SRL
             $write("srl     x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h40005033) // SRA
             $write("sra     x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h00006033) // OR
             $write("or      x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h00007033) // AND
             $write("and     x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hf000707f) == 'h0000000f) // FENCE
             $write("fence");
           else if ((insn & 'hf000707f) == 'h8000000f) // FENCE.TSO
             $write("fence.tso");
           else if ((insn & 'hffffffff) == 'h00000073) // ECALL
             $write("ecall");
           else if ((insn & 'hffffffff) == 'h00100073) // EBREAK
             $write("ebreak");
           else if ((insn & 'hfc00707f) == 'h00001013) // SLLI
             $write("slli    x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'hfc00707f) == 'h00005013) // SRLI
             $write("srli    x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'hfc00707f) == 'h40005013) // SRAI
             $write("srai    x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'h0000707f) == 'h0000001b) // ADDIW
             $write("addiw   x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'hfe00707f) == 'h0000101b) // SLLIW
             $write("srliw   x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'hfe00707f) == 'h0000501b) // SRLIW
             $write("srliw   x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'hfe00707f) == 'h4000501b) // SRAIW
             $write("sraiw   x%1d,x%1d,0x%x", rd, rs1, imm_i);
           else if ((insn & 'hfe00707f) == 'h0000003b) // ADDW
             $write("addw    x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h4000003b) // SUBW
             $write("subw    x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h0000103b) // SLLW
             $write("sllw    x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h0000503b) // SRLW
             $write("srlw    x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h4000503b) // SRAW
             $write("sraw    x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hffffffff) == 'h0000100f) // FENCE.I
             $write("fence.i");
           else if ((insn & 'h0000707f) == 'h00001073) // CSRRW
             $write("csrrw   x%1d,csr[%3x],x%1d", rd, csrno, rs1);
           else if ((insn & 'h0000707f) == 'h00002073) // CSRRS
             $write("csrrs   x%1d,csr[%3x],x%1d", rd, csrno, rs1);
           else if ((insn & 'h0000707f) == 'h00003073) // CSRRC
             $write("csrrc   x%1d,csr[%3x],x%1d", rd, csrno, rs1);
           else if ((insn & 'h0000707f) == 'h00005073) // CSRRWI
             $write("csrrwi  x%1d,csr[%3x],%1d", rd, csrno, rs1);
           else if ((insn & 'h0000707f) == 'h00006073) // CSRRSI
             $write("csrrsi  x%1d,csr[%3x],%1d", rd, csrno, rs1);
           else if ((insn & 'h0000707f) == 'h00007073) // CSRRCI
             $write("csrrci  x%1d,csr[%3x],%1d", rd, csrno, rs1);
           else if ((insn & 'hfe00707f) == 'h02000033) // MUL
             $write("mul     x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h02001033) // MULH
             $write("mulh    x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h02002033) // MULHSU
             $write("mulhsu  x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h02003033) // MULHU
             $write("mulhu   x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h02004033) // DIV
             $write("div     x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h02005033) // DIVU
             $write("divu    x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h02006033) // REM
             $write("rem     x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h02007033) // REMU
             $write("remu    x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h0200003b) // MULW
             $write("mulw    x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h0200403b) // DIVW
             $write("divw    x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h0200503b) // DIVUW
             $write("divuw   x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h0200603b) // REMW
             $write("remw    x%1d,x%1d,x%1d", rd, rs1, rs2);
           else if ((insn & 'hfe00707f) == 'h0200703b) // REMUW
             $write("remuw   x%1d,x%1d,x%1d", rd, rs1, rs2);

           else if ((insn & 'hf9f0707f) == 'h1000202f) // LR.W
             $write("lr.w    x%1d,(x%1d)", rd, rs1);
           else if ((insn & 'hf800707f) == 'h1800202f) // SC.W
             $write("sc.w    x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'h0800202f) // AMOSWAP.W
             $write("amoswap.w x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'h0000202f) // AMOADD.W
             $write("amoadd.w x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'h2000202f) // AMOXOR.W
             $write("amoxor.w x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'h6000202f) // AMOAND.W
             $write("amoand.w x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'h4000202f) // AMOOR.W
             $write("amoor.w x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'h8000202f) // AMOMIN.W
             $write("amomin.w x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'ha000202f) // AMOMAX.W
             $write("amomax.w x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'hc000202f) // AMOMINU.W
             $write("amominu.w x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'he000202f) // AMOMAXU.W
             $write("amomaxu.w x%1d,x%1d,(x%1d)", rd, rs2, rs1);

           else if ((insn & 'hf9f0707f) == 'h1000302f) // LR.D
             $write("lr.d    x%1d,(x%1d)", rd, rs1);
           else if ((insn & 'hf800707f) == 'h1800302f) // SC.D
             $write("sc.d    x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'h0800302f) // AMOSWAP.D
             $write("amoswap.d x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'h0000302f) // AMOADD.D
             $write("amoadd.d x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'h2000302f) // AMOXOR.D
             $write("amoxor.d x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'h6000302f) // AMOAND.D
             $write("amoand.d x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'h4000302f) // AMOOR.D
             $write("amoor.d x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'h8000302f) // AMOMIN.D
             $write("amomin.d x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'ha000302f) // AMOMAX.D
             $write("amomax.d x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'hc000302f) // AMOMINU.D
             $write("amominu.d x%1d,x%1d,(x%1d)", rd, rs2, rs1);
           else if ((insn & 'hf800707f) == 'he000302f) // AMOMAXU.D
             $write("amomaxu.d x%1d,x%1d,(x%1d)", rd, rs2, rs1);

           else if ((insn & 'hffffffff) == 'h30200073) // MRET
             $write("mret");
           else
             $write("illegal or unsupported instruction");

           if (write_back_register != 0)
             $display("     x%1d = %x", write_back_register, rf[write_back_register]);
           else 
             $display("");
           end
`endif

           mem_addr0 <= npc[63:4] + npc[3];
           mem_addr1 <= npc[63:4];
           pc <= npc;
           state <= `S_FETCH_COMPLETE;

           if (npc[63:`MEM_SIZE_LG2] != `MEM_START >> `MEM_SIZE_LG2) begin
`ifdef SIMULATE
`ifndef RISCV_TESTS
              $display("%05d   %1d %x illegal fetch address", $time, prv, npc);
`endif
`endif
              csr_mcause = `TRAP_INSTRUCTION_ACCESS_FAULT;
              csr_mepc = pc;
              csr_mtval = 0;
              state <= `S_EXCEPTION;
           end
        end

        `S_FETCH_COMPLETE: begin
           aligned = pc[3] == 0 ? {mem_data1,mem_data0} : {mem_data0,mem_data1};
           insn = aligned >> (pc[2:1] * 16);
           state <= `S_DECODE;
        end

        `S_DECODE: begin
           rd = insn`insn_rd;

           // XXX This is begging for a dedicated test bench
           // The general principle
           case (insn[1:0])
             0: {rs1,rs2} = {5'd8|insn[9:7], 5'd8|insn[4:2]};
             1: {rs1,rs2} = {insn[11:7], 5'd8|insn[4:2]};
             2: {rs1,rs2} = {insn[11:7],     insn[6:2]};
             3: {rs1,rs2} = {insn`insn_rs1,  insn`insn_rs2};
           endcase
           // The exceptions
           if (insn[1:0] == 1 && insn[15])
             rs1 = 5'd8 | insn[9:7];
           if (insn[1:0] == 2 && insn[15:14] == 1)
             rs1 = 2; // sp
           if (insn[1:0] == 2 && 5 <= insn[15:13])
             rs1 = 2; // sp
           if ((insn & 'he003) == 0)
             rs1 = 2; // sp

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

           c_nzuimm107_1211_5_6_x4 = {insn[10:7],insn[12:11],insn[5],insn[6],2'd0};
           c_uimm5_1210_6_x4       = {insn[5],insn[12:10],insn[6],2'd0};
           c_imm12_62              = {{59{insn[12]}},insn[6:2]};
           c_imm12_43_5_2_6_x16    = {{55{insn[12]}},insn[4:3],insn[5],insn[2],insn[6],4'd0};
           c_imm12_8_109_6_7_2_11_53_x2
                                   = {{53{insn[12]}},insn[8],insn[10:9],insn[6],insn[7],insn[2],insn[11],insn[5:3],
                                      1'd0};
           c_imm12_65_2_1110_43_x2 = {{56{insn[12]}},insn[6:5],insn[2],insn[11:10],insn[4:3],1'd0};
           c_uimm42_12_65_x8       = {insn[4:2],insn[12],insn[6:5],3'd0};
           c_uimm32_12_64_x4       = {insn[3:2],insn[12],insn[6:4],2'd0};
           c_uimm97_1210_x8        = {insn[9:7],insn[12:10],3'd0};
           c_uimm87_129_x4         = {insn[8:7],insn[12:9],2'd0};

           c_uimm65_1210_x8        = {insn[6:5],insn[12:10],3'd0};
           c_uimm12_62             = {insn[12],insn[6:2]};

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

           // Quadrant 0
           if ((insn & 'he003) == 'h0000) begin // C.ADDI4SPN/illegal
              write_back_register = rs2;
              write_back_value = s1 + c_nzuimm107_1211_5_6_x4;
              if ((insn & 'hffff) == 0) begin
                 write_back_register = 0;
                 csr_mcause = `TRAP_ILLEGAL_INSTRUCTION;
                 csr_mepc = pc;
                 csr_mtval = insn;
                 state <= `S_EXCEPTION;
              end
           end

           //else if ((insn & 'he003) == 'h2000) begin // C.FLD
             //$display("c.fld   x%1d,%1d(x%1d)    %x UNTESTED", write_back_register, rs1, c_imm12_62, rf[write_back_register]);
           //end

           else if ((insn & 'he003) == 'h4000) begin // C.LW
              write_back_register = rs2;
              load_size_lg2 = 2|4;
              mem_addr = s1 + c_uimm5_1210_6_x4;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              state <= `S_LOAD_ALIGN;
           end

           else if ((insn & 'he003) == 'h6000) begin // C.LD
              write_back_register = rs2;
              load_size_lg2 = 3;
              mem_addr = s1 + c_uimm65_1210_x8;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              state <= `S_LOAD_ALIGN;
           end

           //else if ((insn & 'he003) == 'ha000) begin // C.FSD
           //  $display("c.fsd   x%1d,%1d(x%1d)    UNTESTED", rs2, rs1, c_imm12_62);
           //end

           else if ((insn & 'he003) == 'hc000) begin // C.SW
              mem_addr = s1 + c_uimm5_1210_6_x4;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              mem_wr_mask <= 15;
              state <= `S_STORE;
           end

           else if ((insn & 'he003) == 'he000) begin // C.SD
              mem_addr = s1 + c_uimm65_1210_x8;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              mem_wr_mask <= 255;
              state <= `S_STORE;
           end


              // Quadrant 1
           else if (insn == 1) begin // C.NOP
             // NOP
           end

           else if ((insn & 'he003) == 'h0001) begin // C.ADDI
              write_back_register = rs1;
              write_back_value = s1 + c_imm12_62;
           end

           else if ((insn & 'he003) == 'h2001) begin // C.ADDIW
              write_back_register = rs1;
              sext32 = s1[31:0] + c_imm12_62;
              write_back_value = {{32{sext32[31]}},sext32};
           end

           else if ((insn & 'he003) == 'h4001) begin // C.LI
              write_back_register = insn[11:7];
              write_back_value = c_imm12_62;
           end

           else if ((insn & 'hef83) == 'h6101) begin // C.ADDI16SP
              write_back_register = rs1;
              write_back_value = s1 + c_imm12_43_5_2_6_x16;
           end

           else if ((insn & 'he003) == 'h6001) begin // C.LUI
              write_back_register = rs1;
              write_back_value = c_imm12_62<<12;
           end

           else if ((insn & 'hec03) == 'h8001) begin // C.SRLI
              write_back_register = rs1;
              write_back_value = s1 >> c_imm12_62[5:0];
           end

           else if ((insn & 'hec03) == 'h8401) begin // C.SRAI
              write_back_register = rs1;
              write_back_value = $signed(s1) >>> c_imm12_62[5:0];
           end

           else if ((insn & 'hec03) == 'h8801) begin // C.ANDI
              write_back_register = rs1;
              write_back_value = s1 & c_imm12_62;
           end

           else if ((insn & 'hfc63) == 'h8c01) begin // C.SUB
              write_back_register = rs1;
              write_back_value = s1 - s2;
           end

           else if ((insn & 'hfc63) == 'h8c21) begin // C.XOR
              write_back_register = rs1;
              write_back_value = s1 ^ s2;
           end

           else if ((insn & 'hfc63) == 'h8c41) begin // C.OR
              write_back_register = rs1;
              write_back_value = s1 | s2;
           end

           else if ((insn & 'hfc63) == 'h8c61) begin // C.AND
              write_back_register = rs1;
              write_back_value = s1 & s2;
           end

           else if ((insn & 'hfc63) == 'h9c01) begin // C.SUBW
              sext32 = s1[31:0] - s2[31:0];
              write_back_register = rs1;
              write_back_value = {{32{sext32[31]}},sext32};
           end

           else if ((insn & 'hfc63) == 'h9c21) begin // C.ADDW
              sext32 = s1[31:0] + s2[31:0];
              write_back_register = rs1;
              write_back_value = {{32{sext32[31]}},sext32};
           end

           else if ((insn & 'he003) == 'ha001) begin // C.J
              npc = pc + c_imm12_8_109_6_7_2_11_53_x2;
           end

           else if ((insn & 'he003) == 'hc001) begin // C.BEQZ
              if (s1 == 0)
                npc = pc + $signed(c_imm12_65_2_1110_43_x2);
           end

           else if ((insn & 'he003) == 'he001) begin // C.BNEZ
              if (s1 != 0)
                npc = pc + $signed(c_imm12_65_2_1110_43_x2);
           end


              // Quadrant 2
           else if ((insn & 'he003) == 'h0002) begin // C.SLLI
              write_back_register = rs1;
              write_back_value = s1 << c_imm12_62[5:0];
           end

           //else if ((insn & 'he003) == 'h2002) begin // C.FLDSP
           //  $display("c.fldsp x%1d,%1d(sp)       %x UNTESTED", write_back_register, c_uimm42_12_65_x8, rf[write_back_register]);
           //end

           else if ((insn & 'he003) == 'h4002) begin // C.LWSP
              write_back_register = insn[11:7]; // XXX this is a bit unclean
              mem_addr = s1 + c_uimm32_12_64_x4;
              load_size_lg2 = 2|4;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              state <= `S_LOAD_ALIGN;
           end

           else if ((insn & 'he003) == 'h6002) begin // C.LDSP
              write_back_register = insn[11:7]; // XXX this is a bit unclean
              mem_addr = s1 + c_uimm42_12_65_x8;
              load_size_lg2 = 3;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              state <= `S_LOAD_ALIGN;
           end

           else if ((insn & 'hf07f) == 'h8002) begin // C.JR
              npc = s1 & ~1;
           end

           else if ((insn & 'hf003) == 'h8002) begin // C.MV
              write_back_register = rs1;
              write_back_value = s2;
           end

           //else if ((insn & 'hffff) == 'h9002) begin // C.EBREAK
           //  $display("c.ebreak UNTESTED");
           //end

           else if ((insn & 'hf07f) == 'h9002) begin // C.JALR
              write_back_register = 1;
              write_back_value = pc + 2;
              npc = s1 & ~1;
           end

           else if ((insn & 'hf003) == 'h9002) begin // C.ADD
              write_back_register = rs1;
              write_back_value = s1 + s2;
           end

           // else if ((insn & 'he003) == 'ha002) begin // C.FSDSP
           //  $display("c.fsdsp x%1d,%1d(sp) UNTESTED", rs2, c_uimm97_1210_x8);
           // end

           else if ((insn & 'he003) == 'hc002) begin // C.SWSP
              mem_addr = s1 + c_uimm87_129_x4;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              mem_wr_mask <= 15;
              state <= `S_STORE;
           end

           else if ((insn & 'he003) == 'he002) begin // C.SDSP
              mem_addr = s1 + c_uimm97_1210_x8;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              mem_wr_mask <= 255;
              state <= `S_STORE;
           end

           // Quadrant 3, uncompressed
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
              write_back_register = rd;
              mem_addr = s1 + imm_i;
              load_size_lg2 = 0|4;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              state <= `S_LOAD_ALIGN;
           end

           else if ((insn & 'h0000707f) == 'h00001003) begin // LH
              write_back_register = rd;
              mem_addr = s1 + imm_i;
              load_size_lg2 = 1|4;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              state <= `S_LOAD_ALIGN;
           end

           else if ((insn & 'h0000707f) == 'h00002003) begin // LW
              write_back_register = rd;
              mem_addr = s1 + imm_i;
              load_size_lg2 = 2|4;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              state <= `S_LOAD_ALIGN;
           end

           else if ((insn & 'h0000707f) == 'h00003003) begin // LD
              write_back_register = rd;
              mem_addr = s1 + imm_i;
              load_size_lg2 = 3;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              state <= `S_LOAD_ALIGN;
           end

           else if ((insn & 'h0000707f) == 'h00004003) begin // LBU
              write_back_register = rd;
              mem_addr = s1 + imm_i;
              load_size_lg2 = 0;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              state <= `S_LOAD_ALIGN;
           end

           else if ((insn & 'h0000707f) == 'h00005003) begin // LHU
              write_back_register = rd;
              mem_addr = s1 + imm_i;
              load_size_lg2 = 1;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              state <= `S_LOAD_ALIGN;
           end

           else if ((insn & 'h0000707f) == 'h00006003) begin // LWU
              write_back_register = rd;
              mem_addr = s1 + imm_i;
              load_size_lg2 = 2;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              state <= `S_LOAD_ALIGN;
           end

           else if ((insn & 'h0000707f) == 'h00000023) begin // SB
              mem_addr = s1 + imm_s;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              mem_wr_mask <= 1;
              state <= `S_STORE;
           end

           else if ((insn & 'h0000707f) == 'h00001023) begin // SH
              mem_addr = s1 + imm_s;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              mem_wr_mask <= 3;
              state <= `S_STORE;
           end

           else if ((insn & 'h0000707f) == 'h00002023) begin // SW
              mem_addr = s1 + imm_s;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              mem_wr_mask <= 15;
              state <= `S_STORE;
           end

           else if ((insn & 'h0000707f) == 'h00003023) begin // SD
              mem_addr = s1 + imm_s;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
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
              write_back_value = $signed(s1) >>> s2[5:0];
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

           else if ((insn & 'hffffffff) == 'h00000073) begin // ECALL
              csr_mcause = `TRAP_ENVIRONMENT_CALL_FROM_U_MODE + prv;
              csr_mepc = pc;
              csr_mtval = 0;
              state <= `S_EXCEPTION;
`ifdef RISCV_TESTS
              if (rf[3] & 1) begin
                 if (rf[3] / 2 == 0)
                   $display("Test Passed");
                 else
                   $display("Test Failed with %3d", rf[3] / 2);
                 $finish;
              end
`endif
           end

           else if ((insn & 'hffffffff) == 'h00100073) begin // EBREAK
            // Requires debug mode
           end

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
              write_back_value = $signed(s1) >>> shamt;
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
              sext32 = $signed(s1[31:0]) >>> shamt[4:0];
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
              sext32 = $signed(s1[31:0]) >>> s2[4:0];
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
              mul_a = s1;
              mul_b = s2;
              state <= `S_MUL_RUNNING;
           end

           else if ((insn & 'hfe00707f) == 'h02001033) begin // MULH
              write_back_register = rd;
              mul_output_negate = s1[63] != s2[63];
              mul_a = {64'd0,s1[63] ? -s1 : s1};
              mul_b = s2[63] ? -s2 : s2;
              mul_output_high_part = 1;
              state <= `S_MUL_RUNNING;
           end

           else if ((insn & 'hfe00707f) == 'h02002033) begin // MULHSU
              write_back_register = rd;
              mul_output_negate = s1[63];
              mul_a = {64'd0, s1[63] ? -s1 : s1};
              mul_b = s2;
              mul_output_high_part = 1;
              state <= `S_MUL_RUNNING;
           end

           else if ((insn & 'hfe00707f) == 'h02003033) begin // MULHU
              write_back_register = rd;
              mul_a = {64'd0, s1};
              mul_b = s2;
              mul_output_high_part = 1;
              state <= `S_MUL_RUNNING;
           end


           else if ((insn & 'hfe00707f) == 'h02004033) begin // DIV
              write_back_register = rd;
              mul_output_negate = s1[63] != s2[63];
              if (s2 == 0)
                // No matter s1, this will produce -1 which is the correct answer
                mul_output_negate = 0;
              div_count = 64;
              mul_p = {64'd0,s1[63] ? -s1 : s1};
              mul_a = {s2[63] ? -s2 : s2, 63'd0};
              mul_b = 0;
              state <= `S_DIV_RUNNING;
           end

           else if ((insn & 'hfe00707f) == 'h02005033) begin // DIVU
              write_back_register = rd;
              div_count = 64;
              mul_p = {64'd0, s1};
              mul_a = {s2, 63'd0};
              mul_b = 0;
              state <= `S_DIV_RUNNING;
           end

           else if ((insn & 'hfe00707f) == 'h02006033) begin // REM
              write_back_register = rd;
              // "For REM, the sign of a nonzero result equals the sign of the dividend."
              mul_output_negate = s1[63];
              if (s2 == 0)
                // No matter s1, this will produce -1 which is the correct answer
                mul_output_negate = 0;
              div_count = 64;
              mul_p = {64'd0,s1[63] ? -s1 : s1};
              mul_a = {s2[63] ? -s2 : s2, 63'd0};
              mul_b = 0;
              mul_output_high_part = 1; // XXX abusing variables
              state <= `S_DIV_RUNNING;
           end

           else if ((insn & 'hfe00707f) == 'h02007033) begin // REMU
              write_back_register = rd;
              // "For REM, the sign of a nonzero result equals the sign of the dividend."
              if (s2 == 0)
                // No matter s1, this will produce -1 which is the correct answer
                mul_output_negate = 0;
              div_count = 64;
              mul_p = {64'd0,s1};
              mul_a = {s2, 63'd0};
              mul_b = 0;
              mul_output_high_part = 1; // XXX abusing variables
              state <= `S_DIV_RUNNING;
           end

           else if ((insn & 'hfe00707f) == 'h0200003b) begin // MULW
              write_back_register = rd;
              mul_a = s1[31] ? {96'd0, -s1} : s1;
              mul_b = s1[31] ? {32'd0, -s2} : s2;
              mul_output_sext32 = 1;
              state <= `S_MUL_RUNNING;
           end

           else if ((insn & 'hfe00707f) == 'h0200403b) begin // DIVW
              write_back_register = rd;
              mul_output_negate = s1[31] != s2[31];
              if (s2 == 0)
                // No matter s1, this will produce -1 which is the correct answer
                mul_output_negate = 0;
              div_count = 32;
              mul_p = {96'd0,s1[31] ? -s1[31:0] : s1[31:0]};
              mul_a = {s2[31] ? -s2[31:0] : s2[31:0], 31'd0};
              mul_b = 0;
              mul_output_sext32 = 1;
              state <= `S_DIV_RUNNING;
           end

           else if ((insn & 'hfe00707f) == 'h0200503b) begin // DIVUW
              write_back_register = rd;
              if (s2 == 0)
                // No matter s1, this will produce -1 which is the correct answer
                mul_output_negate = 0;
              div_count = 32;
              mul_p = {96'd0, s1[31:0]};
              mul_a = {s2[31:0], 31'd0};
              mul_b = 0;
              mul_output_sext32 = 1;
              state <= `S_DIV_RUNNING;
           end

           else if ((insn & 'hfe00707f) == 'h0200603b) begin // REMW
              write_back_register = rd;
              // "For REM, the sign of a nonzero result equals the sign of the dividend."
              mul_output_negate = s1[31];
              div_count = 32;
              mul_p = {96'd0,s1[31] ? -s1[31:0] : s1[31:0]};
              mul_a = {s2[31] ? -s2[31:0] : s2[31:0], 31'd0};
              mul_b = 0;
              mul_output_sext32 = 1;
              mul_output_high_part = 1; // XXX abusing variables
              state <= `S_DIV_RUNNING;
           end

           else if ((insn & 'hfe00707f) == 'h0200703b) begin // REMUW
              write_back_register = rd;
              // "For REM, the sign of a nonzero result equals the sign of the dividend."
              div_count = 32;
              mul_p = {96'd0, s1[31:0]};
              mul_a = {s2[31:0], 31'd0};
              mul_b = 0;
              mul_output_sext32 = 1;
              mul_output_high_part = 1; // XXX abusing variables
              state <= `S_DIV_RUNNING;
           end

           else if ((insn & 'hf9f0707f) == 'h1000202f || // LR.W
                    (insn & 'hf9f0707f) == 'h1000302f)   // LR.D
           begin
              write_back_register = rd;
              mem_addr = s1;
              load_size_lg2 = 4 | (insn[12] ? 3 : 2);
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              reservation <= s1;
              state <= `S_LOAD_ALIGN;
           end

           else if ((insn & 'hf800707f) == 'h1800202f || // SC.W
                    (insn & 'hf800707f) == 'h1800302f)   // SC.D
           begin
              write_back_register = rd;
              write_back_value = 1;
              if (reservation == s1) begin // XXX Should use physical address
                 write_back_value = 0;
                 mem_addr = s1;
                 mem_addr0 <= mem_addr[63:4] + mem_addr[3];
                 mem_addr1 <= mem_addr[63:4];
                 mem_wr_mask <= insn[12] ? 255 : 15;
                 state <= `S_STORE;
              end
           end

           else if ((insn & 'hf800707f) == 'h0800202f || // AMOSWAP.W
                    (insn & 'hf800707f) == 'h0000202f || // AMOADD.W
                    (insn & 'hf800707f) == 'h2000202f || // AMOXOR.W
                    (insn & 'hf800707f) == 'h6000202f || // AMOAND.W
                    (insn & 'hf800707f) == 'h4000202f || // AMOOR.W
                    (insn & 'hf800707f) == 'h8000202f || // AMOMIN.W
                    (insn & 'hf800707f) == 'ha000202f || // AMOMAX.W
                    (insn & 'hf800707f) == 'hc000202f || // AMOMINU.W
                    (insn & 'hf800707f) == 'he000202f || // AMOMAXU.W
                    (insn & 'hf800707f) == 'h0800302f || // AMOSWAP.D
                    (insn & 'hf800707f) == 'h0000302f || // AMOADD.D
                    (insn & 'hf800707f) == 'h2000302f || // AMOXOR.D
                    (insn & 'hf800707f) == 'h6000302f || // AMOAND.D
                    (insn & 'hf800707f) == 'h4000302f || // AMOOR.D
                    (insn & 'hf800707f) == 'h8000302f || // AMOMIN.D
                    (insn & 'hf800707f) == 'ha000302f || // AMOMAX.D
                    (insn & 'hf800707f) == 'hc000302f || // AMOMINU.D
                    (insn & 'hf800707f) == 'he000302f)   // AMOMAXU.D
            begin
              write_back_register = rd;
              mem_addr = s1;
              load_size_lg2 = insn[12] ? 3 : 2;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              do_atomic <= 1;
              state <= `S_LOAD_ALIGN;
           end

           else if ((insn & 'hffffffff) == 'h30200073) begin // MRET
              npc <= csr_mepc;

              mprv = mpp == 3 ? mprv : 0;
              prv = mpp;
              mie = mpie;
              mpie = 1;
              mpp = 0;
           end

           else begin
`ifdef SIMULATE
`ifndef RISCV_TESTS
              if (insn[1:0] == 3)
                $display("%05d   %1d %x %x illegal unknown instruction", $time, prv, pc, insn);
              else
                $display("%05d   %1d %x     %x illegal unknown instruction (%1d,%1d)",
                         $time, prv, pc, insn[15:0], insn[15:13], insn[1:0]);
              $finish;
`endif
`endif
              csr_mcause = `TRAP_ILLEGAL_INSTRUCTION;
              csr_mepc = pc;
              csr_mtval = insn;
              state <= `S_EXCEPTION;
           end
        end

        `S_STORE: begin
           state <= `S_FETCH;
           reservation <= ~0;

           aligned = {64'd0,s2} << (8 * (mem_addr % 8));
           mem_wr_mask = mem_wr_mask << (mem_addr % 8);
           if (mem_addr[3]) begin
              aligned = {aligned[63:0], aligned[127:64]};
              mem_wr_mask = {mem_wr_mask[7:0],mem_wr_mask[15:8]};
           end

	   if (mem_addr == 'h10000000 && mem_wr_mask[0]) begin
              tx_valid_o <= 1;
              tx_data_o <= aligned[7:0];
              if (!tx_ready_i)
                state <= `S_STORE; // Block here until consumed

	      mem_wr_mask[1] = 0;
	   end else if (mem_addr[63:`MEM_SIZE_LG2] != `MEM_START >> `MEM_SIZE_LG2) begin
`ifdef SIMULATE
`ifndef RISCV_TESTS
              $display("%05d   %x xxxxxxxx illegal store address %x", $time, prv, mem_addr);
`endif
`endif
              csr_mcause = `TRAP_STORE_ACCESS_FAULT;
              csr_mepc = pc;
              csr_mtval = mem_addr;
	      mem_wr_mask = 0;
              state <= `S_EXCEPTION;
           end

           if (mem_wr_mask[ 0]) mem0[mem_addr0][ 7: 0] <= aligned[ 7: 0];
           if (mem_wr_mask[ 1]) mem0[mem_addr0][15: 8] <= aligned[15: 8];
           if (mem_wr_mask[ 2]) mem0[mem_addr0][23:16] <= aligned[23:16];
           if (mem_wr_mask[ 3]) mem0[mem_addr0][31:24] <= aligned[31:24];
           if (mem_wr_mask[ 4]) mem0[mem_addr0][39:32] <= aligned[39:32];
           if (mem_wr_mask[ 5]) mem0[mem_addr0][47:40] <= aligned[47:40];
           if (mem_wr_mask[ 6]) mem0[mem_addr0][55:48] <= aligned[55:48];
           if (mem_wr_mask[ 7]) mem0[mem_addr0][63:56] <= aligned[63:56];
           if (mem_wr_mask[ 8]) mem1[mem_addr1][ 7: 0] <= aligned[71:64];
           if (mem_wr_mask[ 9]) mem1[mem_addr1][15: 8] <= aligned[79:72];
           if (mem_wr_mask[10]) mem1[mem_addr1][23:16] <= aligned[87:80];
           if (mem_wr_mask[11]) mem1[mem_addr1][31:24] <= aligned[95:88];
           if (mem_wr_mask[12]) mem1[mem_addr1][39:32] <= aligned[103:96];
           if (mem_wr_mask[13]) mem1[mem_addr1][47:40] <= aligned[111:104];
           if (mem_wr_mask[14]) mem1[mem_addr1][55:48] <= aligned[119:112];
           if (mem_wr_mask[15]) mem1[mem_addr1][63:56] <= aligned[127:120];
        end

        `S_LOAD_ALIGN: begin
           aligned = mem_addr[3] ? {mem_data0, mem_data1} : {mem_data1, mem_data0};
           aligned = aligned >> (mem_addr[2:0] * 8);

           case (load_size_lg2)
             0: write_back_value = aligned[ 7:0];
             1: write_back_value = aligned[15:0];
             2: write_back_value = aligned[31:0];
             3: write_back_value = aligned;
             4: write_back_value = {{56{aligned[ 7]}},aligned[ 7:0]};
             5: write_back_value = {{48{aligned[15]}},aligned[15:0]};
             6: write_back_value = {{32{aligned[31]}},aligned[31:0]};
             7: write_back_value = 'hx;
           endcase

           state <= `S_FETCH;

           if (do_atomic)
             state <= `S_AMO;

           if (mem_addr[63:`MEM_SIZE_LG2] != `MEM_START >> `MEM_SIZE_LG2) begin
              // XXX This isn't catching unaligned access that overflows
`ifdef SIMULATE
`ifndef RISCV_TESTS
              $display("%05d   %x xxxxxxxx illegal load address %x", $time, prv, mem_addr);
`endif
`endif
              write_back_register = 0;
              csr_mcause = `TRAP_LOAD_ACCESS_FAULT;
              csr_mepc = pc;
              csr_mtval = mem_addr;
              state <= `S_EXCEPTION;
           end
        end

        `S_AMO: begin
           mem_wr_mask <= 255;
           if (!insn[12]) begin
              write_back_value = {{32{write_back_value[31]}},write_back_value[31:0]};
              s2 = {{32{s2[31]}},s2[31:0]};
              mem_wr_mask <= 15;
           end

           case (insn[31:24])
             'h08: s2 = s2; // AMOSWAP
             'h00: s2 = s2 + write_back_value; // AMOADD
             'h20: s2 = s2 ^ write_back_value; // AMOXOR
             'h60: s2 = s2 & write_back_value; // AMOAND
             'h40: s2 = s2 | write_back_value; // AMOOR
             'h80: s2 = $signed(s2) < $signed(write_back_value) ? s2 : write_back_value; // AMOMIN
             'ha0: s2 = $signed(s2) < $signed(write_back_value) ? write_back_value : s2; // AMOMAX
             'hc0: s2 = s2 < write_back_value ? s2 : write_back_value; // AMOMINU
             'he0: s2 = s2 < write_back_value ? write_back_value : s2; // AMOMAXU
             default: begin
`ifdef SIMULATE
                $display("Impossible AMO"); $finish;
`endif
                s2 = 'hX;
             end
           endcase

           do_atomic = 0;
           state <= `S_STORE;
        end


        `S_HANDLE_CSR: begin
           state <= `S_FETCH;
           csr_access_failure = 0;
           write_back_register = rd;

           if (rd != 0 || csr_op != `CSR_OP_COPY) begin
              // read the CSR
              case (csrno)
                `CSR_SCOUNTEREN: csr_read_val = 0;
                `CSR_MSTATUS:
                  csr_read_val = {sd, 27'd0, sxl, uxl, // 32
                                  9'd0, tsr, tw, tvm, mxr, sum, mprv, xs, // 15
                                  fs, mpp, 2'd0, spp, mpie, 1'd0, spie, upie, mie, 1'd0, sie, uie}; // 17
                `CSR_MISA:     csr_read_val = 64'h8000000000141105; // 64'h800000000014112d with FD
                // Hardwired 1 0100 0001 0001 0010 1101
                //    ZY XWV U TSRQ PONM LKJI HGFE DCBA
                //           U  S      M    I   F  DC A
                //    SUIMAFDC
                // -                            F  D
                // =         1 0100 0001 0001 0000 0101
                `CSR_MIE:      csr_read_val = csr_mie;
                `CSR_MTVEC:    csr_read_val = csr_mtvec;
                `CSR_MCOUNTEREN: csr_read_val = 0;
                `CSR_MSCRATCH: csr_read_val = csr_mscratch;
                `CSR_MEPC:     csr_read_val = csr_mepc;
                `CSR_MCAUSE:   csr_read_val = csr_mcause;
                `CSR_MTVAL:    csr_read_val = csr_mtval;
                `CSR_MIP:      csr_read_val = csr_mip;
                `CSR_MCYCLE:   csr_read_val = csr_mcycle;
                `CSR_MINSTRET: csr_read_val = csr_minstret;
                `CSR_CYCLE:    csr_read_val = csr_mcycle;
                `CSR_INSTRET:  csr_read_val = csr_minstret;
                `CSR_MHARTID:  csr_read_val = 0;
                `CSR_MVENDORID:csr_read_val = 0;
                `CSR_MARCHID:  csr_read_val = 9; // YARVI, Smolrv64 = YARVI4
                `CSR_MIMPID:   csr_read_val = 'h20250907;
                default: begin
`ifdef SIMULATE
`ifndef RISCV_TESTS
                   $display("%05d   %1d %x %x illegal CSR %x (read)", $time, prv, pc, insn, csrno);
`endif
`endif
                   csr_mcause = `TRAP_ILLEGAL_INSTRUCTION;
                   csr_mepc = pc;
                   csr_mtval = insn;
                   state <= `S_EXCEPTION;
                end
              endcase

              // As no side effects (beside exception have happend, we
              // can postpone the priviledge check to here
              if ((csrno >> 8) & 3 > prv) begin
`ifdef SIMULATE
`ifndef RISCV_TESTS
                 $display("%05d   %1d %x %x mode %d isn't priviledged to read CSR %x", $time,
                          prv, pc, insn, prv, csrno);
`endif
`endif
                 csr_access_failure = 1;
              end
           end

           case (csr_op)
             `CSR_OP_COPY: csr_write_val = csr_arg;
             `CSR_OP_OR: csr_write_val = csr_read_val | csr_arg;
             `CSR_OP_ANDN: csr_write_val = csr_read_val & ~csr_arg;
           endcase

           // Write priviledge check
           if (!csr_access_failure && (rs1 != 0 || csr_op == `CSR_OP_COPY)) begin
              if ((csrno >> 8) & 3 > prv) begin
                 csr_access_failure = 1;
`ifdef SIMULATE
`ifndef RISCV_TESTS
                 $display("%05d   %1d %x %x mode isn't priviledged to write CSR %x", $time,
                          prv, pc, insn, csrno);
`endif
`endif
              end

              if ((csrno >> 10) & 3 == 3) begin
`ifdef SIMULATE
`ifndef RISCV_TESTS
                 $display("%05d   %1d %x %x write attempt to Read Only CSR %x", $time,
                          prv, pc, insn, csrno);
`endif
`endif
                 csr_access_failure = 1;
              end
           end



           // CSRRS, CSRRC, CSRRSI, and CSRRCI don't write the CSR if rs1 == 0
           if (!csr_access_failure && (rs1 != 0 || csr_op == `CSR_OP_COPY)) begin
              // write the CSR
              case (csrno)
                `CSR_SCOUNTEREN: begin end
                `CSR_MSTATUS: begin
                   {sie, uie}        = csr_write_val[1:0];
                   {spie, upie, mie} = csr_write_val[5:3];
                   {spp, mpie}       = csr_write_val[8:7];
                   mpp               = csr_write_val[12:11];
                   // fs = csr_write_val[14:13];
                   {tsr, tw, tvm, mxr, sum, mprv} = csr_write_val[22:17];

                   if (mie && (csr_mie & csr_mip) != 0)
                     $display("XXX Interrupts are now pending (%x), not yet supported!", csr_mie & csr_mip);
                end
                `CSR_MISA:     begin end
                `CSR_MIE:      csr_mie      <= csr_write_val;
                `CSR_MTVEC:    csr_mtvec    <= csr_write_val; // XXX enforce 256-byte alignment for vectored interrupts
                `CSR_MCOUNTEREN: begin end
                `CSR_MSCRATCH: csr_mscratch <= csr_write_val;
                `CSR_MEPC:     csr_mepc     <= csr_write_val;
                `CSR_MCAUSE:   csr_mcause   <= csr_write_val;
                `CSR_MTVAL:    csr_mtval    <= csr_write_val;
                `CSR_MIP:      csr_mip      <= csr_write_val;
                `CSR_MCYCLE:   csr_mcycle   <= csr_write_val;
                `CSR_MINSTRET: csr_minstret <= csr_write_val;
                default: begin
                 csr_access_failure = 1;
`ifdef SIMULATE
`ifndef RISCV_TESTS
                   $display("%05d   %1d %x %x illegal CSR %x (write)", $time, prv, pc, insn, csrno);
`endif
`endif
                end
              endcase
           end

           write_back_value = csr_read_val;
           if (csr_access_failure) begin
              csr_mcause = `TRAP_ILLEGAL_INSTRUCTION;
              csr_mepc = pc;
              csr_mtval = insn;

              state <= `S_EXCEPTION;
           end
        end

        `S_EXCEPTION: begin
           write_back_register = 0;

           // XXX There is a *whole* lot missing here

           // get M and S delegation based on it being an interrupt or an exception
           // if no M delegation => M mode
           // else if no S delegation => S mode
           // else => U mode!
           //
           // Next, write the {epc,cause,tval} CSR corresponding to
           // the mode (XXX which means that all the states writing
           // csr_mcause etc directly are wrong) and pick npc from the
           // corresponding tvec CSR (optionally handing vectoring for
           // interrupts)
           //
           // FINALLY, read the corresponding status register updating
           // it accordingly (this is full of obscure settings, fun).
           
           // Currently, we do none of that

           mpie = mie;
           mie = 0;

           mpp <= prv;
           prv <= 3;
           npc <= csr_mtvec;
           
           state <= `S_FETCH;
        end

        `S_MUL_RUNNING: begin
           if (mul_b != 0) begin
              if (mul_b[0])
                mul_p = mul_p + mul_a;
              mul_a = mul_a << 1;
              mul_b = mul_b >> 1;
           end else begin
              if (mul_output_sext32)
                write_back_value = {{32{mul_p[31]}}, mul_p[31:0]};
              else begin
                 if (mul_output_negate)
                   mul_p = ~mul_p + 1;
                 else
                   mul_p = mul_p;

                 if (mul_output_high_part)
                   write_back_value = mul_p[127:64];
                 else
                   write_back_value = mul_p[63:0];
              end

              // Reset to default values
              mul_p = 0;
              mul_output_negate = 0;
              mul_output_high_part = 0;
              mul_output_sext32 = 0;

              state <= `S_FETCH;
           end
        end

        `S_DIV_RUNNING: begin
           if (div_count != 0) begin
              mul_b = mul_b << 1;
              if (mul_p >= mul_a) begin
                 mul_p = mul_p - mul_a;
                 mul_b = mul_b | 1;
              end
              mul_a = mul_a >> 1;
              div_count = div_count  - 1;
           end else begin
              write_back_value = mul_output_negate ? -mul_b : mul_b;
              if (mul_output_high_part)
                // REM
                write_back_value = mul_output_negate ? -mul_p[63:0] : mul_p[63:0];

              if (mul_output_sext32)
                write_back_value = {{32{write_back_value[31]}}, write_back_value[31:0]};

              mul_p = 0;
              mul_output_negate = 0;
              mul_output_high_part = 0;

              state <= `S_FETCH;
           end
        end

      endcase
   end
endmodule
