`timescale 1ns/10ps
`default_nettype none

`define insn_rd  [11: 7]
`define insn_rs1 [19:15]
`define insn_rs2 [24:20]
`define insn_csr [31:20]

`ifdef SIMULATE
//`define DISASS 1
`ifdef VERILATOR
module smolrv64_tb(input wire clock);
`else
module smolrv64_tb;
   reg        clock = 1; always #5 clock = !clock;
`endif
   wire       halted;

   wire [19:0]          mmio_address;
   wire                 mmio_read;
   wire                 mmio_write;
   wire [31:0]          mmio_writedata;
   wire [ 3:0]          mmio_byteenable;
   wire                 mmio_readdatavalid;
   wire [31:0]          mmio_readdata;

   reg                  reset_n = 0; always @(posedge clock) reset_n <= 1;

   wire       mmio_waitrequest; // Currently ignored
   wire       uart_tx_valid;
   wire [7:0] uart_tx_data;
   reg        uart_rx_valid_tb = 0;
   reg  [7:0] uart_rx_data_tb  = 0;

   smolrv64 smolrv64_inst(.clock                (clock),
                          .reset                (!reset_n),

                          .mmio_address         (mmio_address),
                          .mmio_read            (mmio_read),
                          .mmio_write           (mmio_write),
                          .mmio_writedata       (mmio_writedata),
                          .mmio_byteenable      (mmio_byteenable),
                          .mmio_readdatavalid   (mmio_readdatavalid),
                          .mmio_readdata        (mmio_readdata),

                          .ext_irq              (63'd0),

                          .uart_tx_valid        (uart_tx_valid),
                          .uart_tx_data         (uart_tx_data),
                          .uart_tx_ready        (1'b1),
                          .uart_rx_valid        (uart_rx_valid_tb),
                          .uart_rx_data         (uart_rx_data_tb),

                          .halted_o             (halted));

   always @(posedge clock) begin
      if (uart_tx_valid) begin
`ifdef DISASS
         $display("%05d  <<%c>>", $time, uart_tx_data);
`else
  `ifdef VPI
         $tty_write(uart_tx_data);
  `else
         $write("%c", uart_tx_data);
   `endif
`endif
      end

`ifdef VPI
      begin : rx_poll
         integer ch;
         ch = $tty_read;
         if (ch >= 0) begin
            uart_rx_valid_tb <= 1;
            uart_rx_data_tb  <= ch[7:0];
         end else begin
            uart_rx_valid_tb <= 0;
         end
      end
`endif

      if (halted)
        $finish;
   end

   initial begin
/*
      $dumpfile("smolrv64.vcd");
      $dumpvars(0, smolrv64_tb);
      $display("Open the smolrv64.vcd with https://app.surfer-project.org/");
*/
`ifndef NO_TIMEOUT
      #5000000
`ifdef RISCV_TESTS
      $display("Test Failed with TIMEOUT");
`endif
      $finish;
`endif
   end
endmodule
`endif

module smolrv64(input wire        clock,
                input wire        reset,
/*
 The eventual memory bus

                output [`BUS_WIDTH-5:0] bus_address, // -1 - 4 for 128b
                output                  bus_read,
                output                  bus_write,
                output [127:0]          bus_writedata,
                output [ 15:0]          bus_byteenable, // Ignored unless bus_write
                input                   bus_waitrequest,
                input                   bus_readdatavalid,
                input  [127:0]          bus_readdata,
*/

// The MMIO bus is 32b only and always aligned access
                output reg [19:0] mmio_address,    // aligned byte addresses, aligned to 32b
                output reg        mmio_read = 0,
                output reg        mmio_write = 0,
                output reg [31:0] mmio_writedata,  // Invalid unless mmio_write
                output reg [ 3:0] mmio_byteenable, // Invalid unless mmio_write

                input wire        mmio_readdatavalid,
                input wire [31:0] mmio_readdata,

                input wire [63:1] ext_irq,         // External interrupt sources for PLIC

                // NS16550A UART (at 0x10000000)
                output reg        uart_tx_valid = 0, // Active for one cycle on THR write
                output reg [ 7:0] uart_tx_data,
                input wire        uart_tx_ready,      // TX holding register empty (from UART)
                input wire        uart_rx_valid,      // Pulse to enqueue a byte
                input wire [ 7:0] uart_rx_data,

                // External DRAM bus (0x80000000-0xFFFFFFFF), 256-bit wide, 32-byte burst
                output reg [25:0]  dram_burst_addr,    // 32-byte burst address (phys_addr[30:5])
                output reg         dram_read,
                output reg         dram_write,
                output reg [255:0] dram_writedata,     // pre-shifted full 256-bit burst
                output reg [31:0]  dram_byte_mask,     // DDR4 convention: 1=mask out (don't write)
                input wire         dram_readdatavalid,
                input wire [255:0] dram_readdata,
                input wire         dram_write_ready,   // adapter idle, can accept a write

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

`define USER_SOFTWARE_INTERRUPT                  0
`define SUPERVISOR_SOFTWARE_INTERRUPT            1
`define MACHINE_SOFTWARE_INTERRUPT               3

`define USER_TIMER_INTERRUPT                     4
`define SUPERVISOR_TIMER_INTERRUPT               5
`define MACHINE_TIMER_INTERRUPT                  7

`define USER_EXTERNAL_INTERRUPT                  8
`define SUPERVISOR_EXTERNAL_INTERRUPT            9
`define MACHINE_EXTERNAL_INTERRUPT              11

`define CSR_SSTATUS    12'h100
`define CSR_SIE        12'h104
`define CSR_STVEC      12'h105
`define CSR_SCOUNTEREN 12'h106
`define CSR_SSCRATCH   12'h140
`define CSR_SEPC       12'h141
`define CSR_SCAUSE     12'h142
`define CSR_STVAL      12'h143
`define CSR_SIP        12'h144

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

`define CSR_PMPCFG0    12'h3a0
`define CSR_PMPCFG1    12'h3a1
`define CSR_PMPCFG2    12'h3a2
`define CSR_PMPCFG3    12'h3a3
`define CSR_PMPCFG4    12'h3a4
`define CSR_PMPCFG5    12'h3a5
`define CSR_PMPCFG6    12'h3a6
`define CSR_PMPCFG7    12'h3a7
`define CSR_PMPCFG8    12'h3a8
`define CSR_PMPCFG9    12'h3a9
`define CSR_PMPCFG10   12'h3aa
`define CSR_PMPCFG11   12'h3ab
`define CSR_PMPCFG12   12'h3ac
`define CSR_PMPCFG13   12'h3ad
`define CSR_PMPCFG14   12'h3ae
`define CSR_PMPCFG15   12'h3af
`define CSR_PMPADDR0   12'h3b0
`define CSR_PMPADDR1   12'h3b1
`define CSR_PMPADDR2   12'h3b2
`define CSR_PMPADDR3   12'h3b3
`define CSR_PMPADDR4   12'h3b4
`define CSR_PMPADDR5   12'h3b5
`define CSR_PMPADDR6   12'h3b6
`define CSR_PMPADDR7   12'h3b7
`define CSR_PMPADDR8   12'h3b8
`define CSR_PMPADDR9   12'h3b9
`define CSR_PMPADDR10  12'h3ba
`define CSR_PMPADDR11  12'h3bb
`define CSR_PMPADDR12  12'h3bc
`define CSR_PMPADDR13  12'h3bd
`define CSR_PMPADDR14  12'h3be
`define CSR_PMPADDR15  12'h3bf

// https://www.five-embeddev.com/riscv-debug-spec/v0.13-release/hwbp_registers.html
`define CSR_TSELECT    12'h7a0 // which trigger is accessible through the other trigger registers
`define CSR_TDATA1     12'h7a1 // type:4 dmode:1 data:59
`define CSR_TDATA2     12'h7a2
`define CSR_TDATA3     12'h7a3
`define CSR_TINFO      12'h7a4 // RO
`define CSR_TCONTROL   12'h7a5 // This is optional

`define CSR_DCSR       12'h7b0
`define CSR_DSCRATCH   12'h7b2
`define CSR_MNSTATUS   12'h744 // Don't know what that is
`define CSR_MCYCLE     12'hb00
`define CSR_MTIME      12'hb01
`define CSR_MINSTRET   12'hb02
`define CSR_CYCLE      12'hc00
`define CSR_TIME       12'hc01
`define CSR_INSTRET    12'hc02
`define CSR_MHARTID    12'hf14
`define CSR_MVENDORID  12'hf11
`define CSR_MARCHID    12'hf12
`define CSR_MIMPID     12'hf13

`define CSR_OP_COPY 0
`define CSR_OP_OR   1
`define CSR_OP_ANDN 2

// Smolrv64's state machine: Every instruction (unless an interrupt is
// pending) cycles through the first four: FETCH1, FETCH2, RF, and
// EXECUTE, and most return back to FETCH1.  The register writeback is
// overlapped with FETCH1 (a very modest consession to performance).
//
// All traps and interrupt go to EXCEPTION.  Loads go to LOAD_ALIGN,
// and possibly to MMIO_READ and MMIO_ALIGN.  AMOs go through
// LOAD_ALIGN, AMO, and STORE.
//
// CSR handling is factored out of EXECUTE into its own state, as are
// multiplication and divisions.
//
`define S_FETCH1         0
`define S_FETCH2         1
`define S_RF             2
`define S_EXECUTE        3

`define S_EXCEPTION      4

`define S_LOAD_ALIGN     5
`define S_MMIO_READ      6
`define S_MMIO_ALIGN     7
`define S_AMO            8

`define S_STORE          9

`define S_HANDLE_CSR    10

`define S_MUL_RUNNING   11
`define S_DIV_RUNNING   12

`define S_PTW_READ      13

`define S_FINISH        14
`define S_FETCH2_HALF   15

`define S_DRAM_FETCH_WAIT      16  // wait for DRAM instruction fetch
`define S_DRAM_LOAD_WAIT       17  // wait for DRAM load
`define S_DRAM_PTW_WAIT        18  // wait for DRAM page-table-walk PTE
`define S_DRAM_STORE_WAIT      19  // backpressure wait before first store burst
`define S_DRAM_FETCH_HALF_WAIT 20  // wait for 2nd burst of cross-burst fetch
`define S_DRAM_LOAD2_WAIT      21  // wait for 2nd burst of cross-burst load
`define S_DRAM_STORE2          22  // issue 2nd burst of cross-burst store
`define S_RF2                  23  // wait for BRAM regfile read (rs1/rs2 set in S_RF)
`define S_EXECUTE2             24  // complete write_back_value from pre-computed exe_add
`define S_PTW_PROCESS          25  // process PTE latched from mem1 in S_PTW_READ
`define S_RF3                  26  // register BRAM output (s1_bram/s2_bram) into s1/s2 flip-flops
`define S_FETCH1B              27  // register LUTRAM mem0/mem1 output before S_FETCH2 reads insn
`define S_LOAD_LATCH           28  // register LUTRAM mem0/mem1 output before S_LOAD_ALIGN reads data
`define S_LAST_STATE           28  // update state register width accordingly

// pre_exe_op: ALU operation code pre-decoded in S_RF3, consumed in S_EXECUTE.
// Breaking the 50-case priority if-else exe_add path into two pipeline stages
// reduces the critical path from ~15 LUT levels to ~7 LUT levels per stage.
`define EXOP_ADD  4'd0   // exe_add = s1 + pre_exe_b  (s1[31:0]+b[31:0] if sxt)
`define EXOP_SUB  4'd1   // exe_add = s1 - pre_exe_b
`define EXOP_SHL  4'd2   // exe_add = s1 << b[5:0]    (s1[31:0]<<b[4:0] if sxt)
`define EXOP_SHR  4'd3   // exe_add = s1 >> b[5:0]
`define EXOP_SAR  4'd4   // exe_add = $signed(s1) >>> b[5:0]
`define EXOP_XOR  4'd5   // exe_add = s1 ^ b
`define EXOP_OR   4'd6   // exe_add = s1 | b
`define EXOP_AND  4'd7   // exe_add = s1 & b
`define EXOP_LTS  4'd8   // exe_add = ($signed(s1) < $signed(b)) ? 1 : 0
`define EXOP_LTU  4'd9   // exe_add = (s1 < b) ? 1 : 0
`define EXOP_OPB  4'd10  // exe_add = b               (LUI, AUIPC, JAL link, MV, LI)
`define EXOP_ONE  4'd11  // exe_add = 1               (SC.W/D fail)

   reg [4:0]   state = `S_FETCH1; // XXX We should set this on reset

`ifndef MEM_BASEADDR
`define MEM_BASEADDR    64'h80000000  // override with -DMEM_BASEADDR=64'hXXXXXXXX
`endif
`ifndef MEM_SIZE_LG2
`define MEM_SIZE_LG2    15 // 32 KiB, override with -DMEM_SIZE_LG2=N
`endif
`define MEM_SIZE        (1 << `MEM_SIZE_LG2)
`ifndef RESET_PC
`define RESET_PC        `MEM_BASEADDR  // override with -DRESET_PC=64'hXXXXXXXX
`endif

   // To enable penalty-free unaligned access, memory is split into
   // even and odd 64b word addresses and striped across them.  Any
   // 64-bit word at address A will then be found in
   // {mem1[A/16],mem0[A/16]} if A/8 is even and
   // {mem0[A/16+1],mem1[A/16]} if A/8 is odd.
   (*ram_block*)(* ram_style = "block" *)
   reg  [63:0] mem0[`MEM_SIZE/16-1:0];
   (*ram_block*)(* ram_style = "block" *)
   reg  [63:0] mem1[`MEM_SIZE/16-1:0];

`ifdef SIMULATE
   reg [8*200:0] evenhex, oddhex;
`ifdef RISCV_TESTS
   reg [63:0] tohost_phys;
`endif
`endif
   // Forward declaration for init
   reg [ 2:0]  plic_priority [0:63];
   integer i;
   initial begin
`ifdef SIMULATE
      if (!$value$plusargs("even=%s", evenhex)) begin
         $display("ERROR: please specify the +even=<hexfile>");
         $finish;
      end
      if (!$value$plusargs("odd=%s", oddhex)) begin
         $display("ERROR: please specify the +odd=<hexfile>");
         $finish;
      end
`ifdef RISCV_TESTS
      if (!$value$plusargs("tohost=%h", tohost_phys))
         tohost_phys = 64'h80001000;
`endif

       for (i = 0; i < `MEM_SIZE/16; i = i + 1) begin
          mem0[i] = 0;
          mem1[i] = 0;
       end
       for (i = 0; i < 64; i = i + 1) plic_priority[i] = 0;
       $readmemh(evenhex, mem0, 0, `MEM_SIZE/16-1);
       $readmemh(oddhex, mem1, 0, `MEM_SIZE/16-1);
`else
      $readmemh("mem.even", mem0, 0, `MEM_SIZE/16-1);
      $readmemh("mem.odd",  mem1, 0, `MEM_SIZE/16-1);
`endif
   end


   reg  [`MEM_SIZE_LG2-5:0] mem_addr0, mem_addr1;
   reg  [63:0] mem_addr;
   reg  [15:0] mem_wr_mask;
   wire [63:0] mem_data0 = mem0[mem_addr0];
   wire [63:0] mem_data1 = mem1[mem_addr1];
   reg  [63:0] mem_data0_q = 0;  // registered copy latched in S_FETCH1B; used by S_FETCH2
   reg  [63:0] mem_data1_q = 0;

   /* RISC-V Architectural state: operating mode, pc, and registers*/
   reg  [63:0] pc = 0; // XXX We should set this on reset
   reg  [ 1:0] prv = 3; // XXX We should set this on reset

   // Read ports
   reg  [ 4:0] rs1, rs2;
   wire [63:0] s1_bram;   // BRAM registered output; valid from start of S_RF3 onwards
   wire [63:0] s2_bram;
   reg  [63:0] s1 = 0;    // flip-flop copy of s1_bram; captured in S_RF3, used in S_EXECUTE
   reg  [63:0] s2 = 0;

   reg  [ 4:0] write_back_register = 0;
   reg  [63:0] write_back_value;
   reg  [63:0] exe_add   = 0;  // execute-stage intermediate (registered at S_EXECUTE→S_EXECUTE2)
   reg         exe_sext32 = 0; // 1 = sign-extend bit 31 of exe_add
   // Pre-decoded ALU control: computed in S_RF3, consumed in S_EXECUTE case block.
   // Breaks the ~50-condition priority if-else chain critical path into two pipeline stages.
   reg  [ 3:0] pre_exe_op  = 0;  // EXOP_* operation code
   reg  [63:0] pre_exe_b   = 0;  // second operand
   reg         pre_exe_sxt = 0;  // 1 → W-type: operate on [31:0], sign-extend result

   // Pre-registered interrupt check: computed every cycle, consumed in S_FETCH1.
   // Breaks the mtip_reg → cause_priority_encode → state_reg path (~10 LUT levels)
   // into two shorter stages (~4 LUT levels each).
   reg         pre_intr_pending = 0;  // 1 = interrupt pending (checked in S_FETCH1)
   reg  [11:0] pre_intr_cause   = 0;  // encoded interrupt cause number

   regfile rf_inst(.clock(clock),
                   .write_valid(state == `S_FETCH1 && write_back_register != 0),
                   .write_data(write_back_value),
                   .write_addr(write_back_register),
                   .read_addr_0(rs1),
                   .read_addr_1(rs2),
                   .read_data_0(s1_bram),
                   .read_data_1(s2_bram));


   reg  [63:0] npc = `RESET_PC; // XXX We should set this on reset
   reg  [255:0] dram_latched;       // holds first DDR4 burst across states
   reg          fetch_from_dram;    // set when current fetch came from DRAM
   reg          ptw_from_dram;      // set when current PTW PTE came from DRAM
   reg  [25:0]  dram2_addr;         // burst address for 2nd burst of split store
   reg  [63:0]  dram2_data_part;    // overflow bytes for split store
   reg  [31:0]  dram2_mask;         // DDR4 byte mask for split-store second burst
   reg          dram_store_split;   // 1 = second burst pending after DRAM_STORE_WAIT

`ifndef BUS_TIMEOUT_LG2
`define BUS_TIMEOUT_LG2 10  // 1024 cycles before access fault
`endif
   reg [`BUS_TIMEOUT_LG2-1:0] bus_timeout_ctr = 0;
   reg  [127:0] aligned;
   reg  [127:0] pte_latch = 0;    // registered copy of PTE data; set in S_PTW_READ, used in S_PTW_PROCESS
   reg  [63:0] imm_i, imm_j, imm_b, imm_u, imm_s, csr_arg, csr_read_val, csr_write_val;
   reg  [63:0] c_imm12_8_109_6_7_2_11_53_x2;
   reg  [63:0] c_imm12_65_2_1110_43_x2;
   reg  [ 9:0] c_nzuimm107_1211_5_6_x4;
   reg  [63:0] c_imm12_62;
   reg  [63:0] c_imm12_43_5_2_6_x16;
   reg  [ 6:0] c_uimm5_1210_6_x4;
   reg  [ 8:0] c_uimm42_12_65_x8, c_uimm97_1210_x8;
   reg  [ 7:0] c_uimm32_12_64_x4, c_uimm87_129_x4, c_uimm65_1210_x8;
   reg  [31:0] sext32;
   reg  [ 2:0] load_size_lg2; // [1:0] = size (0:B, 1:H, 2:W, 3:D), [2] = sign-extend
`ifdef SIMULATE
   reg  [127:0] tmp128;
`endif
   reg  [ 4:0] rd;
   reg  [ 5:0] shamt;
   reg  [11:0] csrno;
   reg  [31:0] insn = 0; // XXX We should set this on reset
   reg  [ 1:0] csr_op;

   // CSR state
   reg         deleg, cause_intr;
   reg [63:0]  tval,
               tvec;
   reg [11:0]  csr_mie        = 0, // XXX We should set this on reset
               csr_mideleg    = 0,
               // temporary, will not turn into flop
               cause;
   reg [63:0]  csr_stvec      = 0,
               csr_scounteren = 0,
               csr_sscratch   = 0,
               csr_sepc       = 0,
               csr_scause     = 0,
               csr_satp       = 0,
               csr_stval      = 0,
               csr_medeleg    = 0, // XXX We should set this on reset
               csr_mtvec      = 0, // XXX We should set this on reset
               csr_mscratch   = 'hDEADBEEFCAFEF00D,
               csr_mepc       = 0,
               csr_mcause     = 0,
               csr_mtval      = 0,
               csr_mcycle     = 0,
               csr_minstret   = -1; // because we increase it in fetch

   // CLINT
   reg [63:0]  clint_mtime = 0;
   reg [63:0]  clint_mtimecmp = ~0;
   reg         clint_msip = 0;

   // NS16550A UART state (at 0x10000000, IRQ 10 on PLIC)
   reg [7:0]   uart_ier = 0;        // Interrupt Enable Register
   reg         uart_fcr_fifo = 0;   // FCR bit 0: FIFO enable
   reg [7:0]   uart_lcr = 0;        // Line Control Register (DLAB = bit 7)
   reg [7:0]   uart_mcr = 0;        // Modem Control Register
   reg [7:0]   uart_scr = 0;        // Scratch Register
   reg [7:0]   uart_rx_fifo [0:255]; // 256-byte RX FIFO
   reg [7:0]   uart_rx_head = 0, uart_rx_tail = 0;
   wire [8:0]  uart_rx_count = uart_rx_tail - uart_rx_head;
   wire        uart_rx_empty = uart_rx_head == uart_rx_tail;
   wire        uart_rx_ip = uart_ier[0] && !uart_rx_empty;  // RX data available
   wire        uart_thre_ip = uart_ier[1];                   // THR always empty
   // IIR: bit 0 = 0 means interrupt pending, 1 = no pending; bits [7:6] = FIFO status
   wire [7:0]  uart_iir = uart_rx_ip   ? {uart_fcr_fifo, uart_fcr_fifo, 2'b0, 4'h4} :
                          uart_thre_ip ? {uart_fcr_fifo, uart_fcr_fifo, 2'b0, 4'h2} :
                                         {uart_fcr_fifo, uart_fcr_fifo, 2'b0, 4'h1};
   wire [7:0]  uart_lsr = {1'b0, uart_tx_ready, uart_tx_ready, 4'b0, !uart_rx_empty}; // TEMT|THRE + DR
   wire        uart_irq_out = uart_rx_ip || uart_thre_ip;

   // PLIC (SiFive layout, base 0x0C000000)
   // Only S-mode context implemented (context 1)
   // plic_priority declared above (before initial block)
   reg [63:0]  plic_pending = 0;       // Interrupt pending bits
   reg [63:0]  plic_enabled = 0;       // Enable bits (S-mode context)
   reg [ 2:0]  plic_threshold = 0;     // Priority threshold (S-mode)
   reg [ 5:0]  plic_claim = 0;        // Last claimed IRQ

   // Find highest-priority pending+enabled interrupt (2-cycle pipeline)
   // Stage 1: scan 4 groups of 16, register results
   reg [5:0] plic_grp_irq_r [0:3];
   reg [2:0] plic_grp_pri_r [0:3];
   integer   plic_i, plic_g;
   always @(posedge clock) begin : plic_stage1
      reg [5:0] gi;
      reg [2:0] gp;
      for (plic_g = 0; plic_g < 4; plic_g = plic_g + 1) begin
         gi = 0; gp = 0;
         for (plic_i = plic_g * 16; plic_i < (plic_g + 1) * 16; plic_i = plic_i + 1)
            if (plic_i > 0 &&
                plic_pending[plic_i] && plic_enabled[plic_i] &&
                plic_priority[plic_i] > plic_threshold &&
                plic_priority[plic_i] > gp) begin
               gi = plic_i[5:0];
               gp = plic_priority[plic_i];
            end
         plic_grp_irq_r[plic_g] <= gi;
         plic_grp_pri_r[plic_g] <= gp;
      end
   end
   // Stage 2: merge 4 registered group results
   reg [5:0] plic_best_irq = 0;
   reg       plic_has_irq = 0;
   always @(posedge clock) begin : plic_stage2
      reg [5:0] best_irq;
      reg [2:0] best_pri;
      best_irq = plic_grp_irq_r[0]; best_pri = plic_grp_pri_r[0];
      if (plic_grp_pri_r[1] > best_pri) begin best_irq = plic_grp_irq_r[1]; best_pri = plic_grp_pri_r[1]; end
      if (plic_grp_pri_r[2] > best_pri) begin best_irq = plic_grp_irq_r[2]; best_pri = plic_grp_pri_r[2]; end
      if (plic_grp_pri_r[3] > best_pri) begin best_irq = plic_grp_irq_r[3]; best_pri = plic_grp_pri_r[3]; end
      plic_best_irq <= best_irq;
      plic_has_irq  <= best_irq != 0;
   end

   // MIP subfields
   // MEIP/SEIP driven by PLIC, MTIP/MSIP driven by CLINT
   reg         ueip = 0,
               stip = 0, utip = 0,
               ssip = 0, usip = 0;
   wire        meip = plic_has_irq;
   wire        seip = plic_has_irq;
   reg         mtip = 0;
   always @(posedge clock) mtip <= clint_mtime >= clint_mtimecmp;
   wire        msip = clint_msip;

   wire [11:0] csr_mip = {meip, 1'd0, seip, ueip,
                          mtip, 1'd0, stip, utip,
                          msip, 1'd0, ssip, usip};

   // MSTATUS subfields
   // Global interrupt-enable bits
   reg         uie = 0, sie = 0, mie = 0;
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
   reg [127:0] mul_a, muldiv_p;
   reg         muldiv_output_sext32;
   reg         muldiv_output_negate;
   reg         muldiv_output_high_part;
   reg [6:0]   div_count;

   reg [63:0]  reservation = ~0;
   reg         do_atomic;
   reg         csr_access_failure = 0;

   reg [63:0]  store_value;

   // Sv39 page table walk state
   reg [ 1:0]  ptw_level;       // Current walk level (2, 1, 0)
   reg [ 4:0]  ptw_return;      // State to return to after translation
   reg [63:0]  ptw_va;          // Virtual address being translated
   reg [63:0]  ptw_pte_addr;    // Physical address of PTE being read
   reg [ 1:0]  ptw_access;      // 0=fetch, 1=load, 2=store, 3=AMO (R+W)
   reg [ 1:0]  ptw_prv;         // Effective privilege for permission check
   reg         translated = 0;  // Set by PTW, cleared by consumer
   reg [11:0]  ptw_fault_cause; // Computed at top of S_PTW_READ
   reg [15:0]  insn_half;       // Saved lower half for cross-page instruction fetch

`ifdef VERILATOR_COSIM
   import "DPI-C" function void cosim_retire(
       input longint unsigned pc,
       input longint unsigned next_pc,
       input int    unsigned insn,
       input byte   unsigned rd_kind,   // 0=none, 1=int, 2=fp
       input byte   unsigned rd_idx,
       input byte   unsigned prv,       // pre-retire privilege
       input byte   unsigned trapped,
       input int    unsigned fflags,
       input longint unsigned rd_val,
       input longint unsigned trap_cause,
       input longint unsigned trap_tval,
       input longint unsigned mtime
   );
   reg        just_trapped = 0;  // suppress S_FETCH1 retire after trap emission
   reg [1:0]  prv_at_trap  = 0;  // pre-trap privilege, captured in S_EXCEPTION
`endif

   always @(posedge clock) begin
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */
      csr_mcycle <= csr_mcycle + 1;
      clint_mtime <= clint_mtime + 1;
      uart_tx_valid <= 0;

      // Enqueue UART RX data
      if (uart_rx_valid && uart_rx_count < 256) begin
         uart_rx_fifo[uart_rx_tail] <= uart_rx_data;
         uart_rx_tail <= uart_rx_tail + 1;
      end

      // Latch external interrupts into PLIC pending (source 10 = UART)
      plic_pending <= plic_pending | {ext_irq, 1'b0}
                    | (uart_irq_out ? (64'd1 << 10) : 64'd0);

      mmio_write = 0;
      mmio_read = 0;
      dram_read  = 0;
      dram_write = 0;

      // Pre-register interrupt pending for S_FETCH1 timing closure.
      // Computed from current FFs so the result is available as a stable FF in
      // the NEXT cycle (adding ≤1 cycle of interrupt detection latency, which
      // is architecturally legal).
      begin : pre_intr_precompute
         reg [11:0] pi_pm, pi_ps, pi_raw;
         pi_pm = csr_mip & csr_mie & ~csr_mideleg;
         pi_ps = csr_mip & csr_mie &  csr_mideleg;
         if ((prv == 3 ? mie : 1'b1) && pi_pm != 0)
            pi_raw = pi_pm;
         else if (((prv == 1) ? sie : (prv == 0)) && pi_ps != 0)
            pi_raw = pi_ps;
         else
            pi_raw = 0;
         pre_intr_pending <= pi_raw != 0;
         pre_intr_cause   <= pi_raw[`MACHINE_EXTERNAL_INTERRUPT] ? `MACHINE_EXTERNAL_INTERRUPT :
                             pi_raw[`MACHINE_SOFTWARE_INTERRUPT] ? `MACHINE_SOFTWARE_INTERRUPT :
                             pi_raw[`MACHINE_TIMER_INTERRUPT]    ? `MACHINE_TIMER_INTERRUPT :
                             pi_raw[`SUPERVISOR_EXTERNAL_INTERRUPT] ? `SUPERVISOR_EXTERNAL_INTERRUPT :
                             pi_raw[`SUPERVISOR_SOFTWARE_INTERRUPT] ? `SUPERVISOR_SOFTWARE_INTERRUPT :
                             pi_raw[`SUPERVISOR_TIMER_INTERRUPT] ? `SUPERVISOR_TIMER_INTERRUPT :
                             pi_raw[`USER_EXTERNAL_INTERRUPT]    ? `USER_EXTERNAL_INTERRUPT :
                             pi_raw[`USER_SOFTWARE_INTERRUPT]    ? `USER_SOFTWARE_INTERRUPT :
                                                                   `USER_TIMER_INTERRUPT;
      end

      case (state)
        `S_FETCH1: begin
           csr_minstret <= csr_minstret + 1;

           // Reset to default values
           muldiv_p = 0;
           muldiv_output_negate = 0;
           muldiv_output_high_part = 0;
           muldiv_output_sext32 = 0;
           do_atomic = 0;

`ifdef VERILATOR_COSIM
           // Normal retire: pc/insn/write_back_* still hold the just-completed
           // instruction's data; npc is its post-retire pc. Skip the first
           // fetch (csr_mcycle == 0) and the dummy fetch right after a trap
           // (just_trapped set by S_EXCEPTION).
           if (csr_mcycle != 0 && !just_trapped) begin
              cosim_retire(
                  pc,
                  npc,
                  insn,
                  (write_back_register != 0) ? 8'd1 : 8'd0,
                  {3'd0, write_back_register},
                  {6'd0, prv},
                  8'd0,
                  32'd0,
                  write_back_value,
                  64'd0,
                  64'd0,
                  clint_mtime
              );
           end
           just_trapped <= 0;
`endif

`ifdef DISASS
`include "disass.vh"
`endif
`ifdef TRACE
           if (csr_mcycle) begin
              if ((insn & 3) == 3)
                $write("%0d %0d %016x %08x", csr_minstret - 1, prv, pc, insn);
              else
                $write("%0d %0d %016x %04x", csr_minstret - 1, prv, pc, insn[15:0]);
              if (write_back_register != 0)
                $display(" %0d %016x", write_back_register, write_back_value);
              else
                $display("");
           end
`endif

           pc <= npc;

           if (csr_satp[63:60] == 4'd8 && prv != 3) begin
              // Sv39 instruction fetch translation
              ptw_va = npc;
              ptw_level = 2;
              ptw_access = 0;
              ptw_prv = prv;
              ptw_return = `S_FETCH2;
              ptw_pte_addr = {8'd0, csr_satp[43:0], 12'd0} + {52'd0, npc[38:30], 3'd0};
              if (ptw_pte_addr[31] && ptw_pte_addr[63:`MEM_SIZE_LG2] != `MEM_BASEADDR >> `MEM_SIZE_LG2) begin
                 ptw_from_dram <= 1;
                 dram_burst_addr = ptw_pte_addr[30:5];
                 dram_read       = 1;
                 state          <= `S_DRAM_PTW_WAIT;
              end else begin
                 ptw_from_dram <= 0;
                 mem_addr0 <= ptw_pte_addr[`MEM_SIZE_LG2-1:4] + ptw_pte_addr[3];
                 mem_addr1 <= ptw_pte_addr[`MEM_SIZE_LG2-1:4];
                 state <= `S_PTW_READ;
              end
           end else begin
              if (npc[63:31] == 1 && npc[63:`MEM_SIZE_LG2] != `MEM_BASEADDR >> `MEM_SIZE_LG2) begin
                 // DRAM fetch (above BRAM overlay: 0x80400000-0xFFFFFFFF)
                 fetch_from_dram <= 1;
                 dram_burst_addr  = npc[30:5];
                 dram_read        = 1;
                 state           <= `S_DRAM_FETCH_WAIT;
              end else begin
                 // BRAM fetch (overlay 0x80000000-0x803FFFFF, or lower addresses)
                 fetch_from_dram <= 0;
                 mem_addr0 <= npc[`MEM_SIZE_LG2-1:4] + npc[3];
                 mem_addr1 <= npc[`MEM_SIZE_LG2-1:4];
                 state <= `S_FETCH1B;
              end
           end

           // Use pre-registered interrupt check (computed previous cycle) for timing closure.
           // pre_intr_pending/pre_intr_cause are stable FFs; the path to state_reg is short.
           cause_intr = 0;
           if (pre_intr_pending) begin
              cause = pre_intr_cause;
              cause_intr = 1;
              tval = 0;
              state <= `S_EXCEPTION;
           end else if (csr_satp[63:60] != 4'd8 || prv == 3) begin
              // Physical address check only when VM is off
              if (npc[63:`MEM_SIZE_LG2] != `MEM_BASEADDR >> `MEM_SIZE_LG2 &&
                  npc[63:31] != 1) begin
`ifdef SIMULATE
`ifdef VERBOSE
                 $display("%05d   %1d %x illegal fetch address csr_satp[63:60] = %d", $time, prv, npc, csr_satp[63:60]);
`endif
`endif
                 cause = `TRAP_INSTRUCTION_ACCESS_FAULT;
                 tval = 0;
                 state <= `S_EXCEPTION;
              end
           end
        end

        `S_FETCH2: begin
           translated <= 0;
           if (fetch_from_dram) begin
              case (pc[4:3])
                2'b00: aligned = dram_latched[127:0];
                2'b01: aligned = dram_latched[191:64];
                2'b10: aligned = dram_latched[255:128];
                2'b11: aligned = {64'bx, dram_latched[255:192]};
              endcase
           end else begin
              aligned = pc[3] == 0 ? {mem_data1_q,mem_data0_q} : {mem_data0_q,mem_data1_q};
           end
           // Register insn; cross-page check and decode happen in S_RF using stable insn_reg.
           insn <= aligned >> (pc[2:1] * 16);
           write_back_register = 0;
           state <= `S_RF;
        end

        `S_RF: begin
           // insn is registered (captured at S_FETCH2 clock edge).
           // Cross-page instruction fetch: 32-bit insn at last halfword of a page
           // In VM mode, the next page may map to a different physical page
           if (pc[11:0] == 12'hFFE && insn[1:0] == 2'b11 &&
               csr_satp[63:60] == 4'd8 && prv != 3) begin
              insn_half <= insn[15:0];
              ptw_va = pc + 2;
              ptw_level = 2;
              ptw_access = 0;
              ptw_prv = prv;
              ptw_return = `S_FETCH2_HALF;
              ptw_pte_addr = {8'd0, csr_satp[43:0], 12'd0} + {52'd0, ptw_va[38:30], 3'd0};
              if (ptw_pte_addr[31] && ptw_pte_addr[63:`MEM_SIZE_LG2] != `MEM_BASEADDR >> `MEM_SIZE_LG2) begin
                 ptw_from_dram <= 1;
                 dram_burst_addr = ptw_pte_addr[30:5];
                 dram_read       = 1;
                 state          <= `S_DRAM_PTW_WAIT;
              end else begin
                 ptw_from_dram <= 0;
                 mem_addr0 <= ptw_pte_addr[`MEM_SIZE_LG2-1:4] + ptw_pte_addr[3];
                 mem_addr1 <= ptw_pte_addr[`MEM_SIZE_LG2-1:4];
                 state <= `S_PTW_READ;
              end
           // Cross-burst DRAM fetch: 32-bit instruction straddles 32-byte burst boundary
           end else if (fetch_from_dram && pc[4:1] == 4'b1111 && insn[1:0] == 2'b11) begin
              insn_half <= insn[15:0];
              dram_burst_addr = pc[30:5] + 1;
              dram_read       = 1;
              state          <= `S_DRAM_FETCH_HALF_WAIT;
           end else begin
              rd = insn`insn_rd;
              case (insn[1:0])
                0: {rs1,rs2} = {{2'd1,insn[9:7]}, {2'd1,insn[4:2]}};
                1: {rs1,rs2} = {insn[11:7],       {2'd1,insn[4:2]}};
                2: {rs1,rs2} = {insn[11:7],       insn[6:2]};
                3: {rs1,rs2} = {insn`insn_rs1,    insn`insn_rs2};
              endcase
              // The exceptions
              if (insn[1:0] == 1 && insn[15])
                rs1 = {2'd1,insn[9:7]};
              if (insn[1:0] == 2 && insn[15:14] == 1)
                rs1 = 2; // sp
              if (insn[1:0] == 2 && 5 <= insn[15:13])
                rs1 = 2; // sp
              if ((insn & 'he003) == 0)
                rs1 = 2; // sp

              shamt = insn[25:20];

              write_back_register = 0;
              state <= `S_RF2;
           end
        end

        `S_RF2: begin
           // One-cycle wait: BRAM samples new rs1/rs2 (set in S_RF); output settles in S_RF3.
           state <= `S_RF3;
        end

        `S_RF3: begin
           // Register BRAM output into s1/s2 flip-flops.
           // s1_bram/s2_bram are now valid (BRAM read with new rs1/rs2 completed in S_RF2).
           s1 <= s1_bram;
           s2 <= s2_bram;
           state <= `S_EXECUTE;

           // Pre-decode ALU operation and second operand for S_EXECUTE.
           // insn, pc are registered FFs; s2_bram is the BRAM combinational output.
           // All assignments use <= so they register into pre_exe_op/pre_exe_b/pre_exe_sxt.
           // Immediates are computed inline (1-3 LUT from insn_reg) rather than read from
           // the imm_i/imm_u registers (which are only updated with = inside S_EXECUTE).
           begin : rf3_pre_decode
              reg [63:0] d_imm_i, d_imm_u, d_c_imm;

              d_imm_i = {{52{insn[31]}},insn[31:20]};
              d_imm_u = {{32{insn[31]}},insn[31:12],12'd0};
              d_c_imm = {{59{insn[12]}},insn[6:2]};  // c_imm12_62

              // Default: harmless value (only matters for instructions reaching S_EXECUTE2)
              pre_exe_op  <= `EXOP_OPB;
              pre_exe_b   <= 64'd0;
              pre_exe_sxt <= 0;

              // ---- Compressed instructions (insn[1:0] != 2'b11) ----

              // Quadrant 0
              if ((insn & 'he003) == 'h0000) begin // C.ADDI4SPN (rd'=rs2)
                 pre_exe_op <= `EXOP_ADD;
                 pre_exe_b  <= {54'd0, insn[10:7], insn[12:11], insn[5], insn[6], 2'd0};
              end

              // Quadrant 1
              else if ((insn & 'he003) == 'h0001) begin // C.ADDI / C.NOP
                 pre_exe_op <= `EXOP_ADD;
                 pre_exe_b  <= d_c_imm;
              end
              else if ((insn & 'he003) == 'h2001) begin // C.ADDIW (RV64)
                 pre_exe_op  <= `EXOP_ADD;
                 pre_exe_b   <= d_c_imm;
                 pre_exe_sxt <= 1;
              end
              else if ((insn & 'he003) == 'h4001) begin // C.LI
                 pre_exe_op <= `EXOP_OPB;
                 pre_exe_b  <= d_c_imm;
              end
              else if ((insn & 'hef83) == 'h6101) begin // C.ADDI16SP (rd=sp)
                 pre_exe_op <= `EXOP_ADD;
                 pre_exe_b  <= {{55{insn[12]}}, insn[4:3], insn[5], insn[2], insn[6], 4'd0};
              end
              else if ((insn & 'he003) == 'h6001) begin // C.LUI (rd!=0,2)
                 pre_exe_op <= `EXOP_OPB;
                 pre_exe_b  <= {{47{insn[12]}}, insn[6:2], 12'd0};
              end
              else if ((insn & 'hec03) == 'h8001) begin // C.SRLI
                 pre_exe_op <= `EXOP_SHR;
                 pre_exe_b  <= d_c_imm;
              end
              else if ((insn & 'hec03) == 'h8401) begin // C.SRAI
                 pre_exe_op <= `EXOP_SAR;
                 pre_exe_b  <= d_c_imm;
              end
              else if ((insn & 'hec03) == 'h8801) begin // C.ANDI
                 pre_exe_op <= `EXOP_AND;
                 pre_exe_b  <= d_c_imm;
              end
              else if ((insn & 'hfc63) == 'h8c01) begin // C.SUB
                 pre_exe_op <= `EXOP_SUB;
                 pre_exe_b  <= s2_bram;
              end
              else if ((insn & 'hfc63) == 'h8c21) begin // C.XOR
                 pre_exe_op <= `EXOP_XOR;
                 pre_exe_b  <= s2_bram;
              end
              else if ((insn & 'hfc63) == 'h8c41) begin // C.OR
                 pre_exe_op <= `EXOP_OR;
                 pre_exe_b  <= s2_bram;
              end
              else if ((insn & 'hfc63) == 'h8c61) begin // C.AND
                 pre_exe_op <= `EXOP_AND;
                 pre_exe_b  <= s2_bram;
              end
              else if ((insn & 'hfc63) == 'h9c01) begin // C.SUBW
                 pre_exe_op  <= `EXOP_SUB;
                 pre_exe_b   <= s2_bram;
                 pre_exe_sxt <= 1;
              end
              else if ((insn & 'hfc63) == 'h9c21) begin // C.ADDW
                 pre_exe_op  <= `EXOP_ADD;
                 pre_exe_b   <= s2_bram;
                 pre_exe_sxt <= 1;
              end

              // Quadrant 2
              else if ((insn & 'he003) == 'h0002) begin // C.SLLI
                 pre_exe_op <= `EXOP_SHL;
                 pre_exe_b  <= d_c_imm;
              end
              else if ((insn & 'hf07f) == 'h8002) begin // C.JR (no exe_add, default ok)
                 ;
              end
              else if ((insn & 'hf003) == 'h8002) begin // C.MV
                 pre_exe_op <= `EXOP_OPB;
                 pre_exe_b  <= s2_bram;
              end
              else if ((insn & 'hf07f) == 'h9002) begin // C.JALR (link = pc+2)
                 pre_exe_op <= `EXOP_OPB;
                 pre_exe_b  <= pc + 2;
              end
              else if ((insn & 'hf003) == 'h9002) begin // C.ADD
                 pre_exe_op <= `EXOP_ADD;
                 pre_exe_b  <= s2_bram;
              end

              // ---- 32-bit instructions (insn[1:0] == 2'b11) ----
              else if (insn[1:0] == 2'b11) begin
                 case (insn[6:2])
                    5'b01101: begin // LUI
                       pre_exe_op <= `EXOP_OPB;
                       pre_exe_b  <= d_imm_u;
                    end
                    5'b00101: begin // AUIPC
                       pre_exe_op <= `EXOP_OPB;
                       pre_exe_b  <= pc + d_imm_u;
                    end
                    5'b11011: begin // JAL (link = pc+4)
                       pre_exe_op <= `EXOP_OPB;
                       pre_exe_b  <= pc + 4;
                    end
                    5'b11001: begin // JALR (link = pc+4)
                       pre_exe_op <= `EXOP_OPB;
                       pre_exe_b  <= pc + 4;
                    end
                    5'b00100: begin // OP-IMM: funct3 selects operation
                       pre_exe_b <= d_imm_i; // default; shifts override below
                       case (insn[14:12])
                          3'b000: pre_exe_op <= `EXOP_ADD;   // ADDI
                          3'b001: begin pre_exe_op <= `EXOP_SHL; pre_exe_b <= {58'd0, insn[25:20]}; end  // SLLI
                          3'b010: pre_exe_op <= `EXOP_LTS;   // SLTI
                          3'b011: pre_exe_op <= `EXOP_LTU;   // SLTIU
                          3'b100: pre_exe_op <= `EXOP_XOR;   // XORI
                          3'b101: begin // SRLI / SRAI
                             pre_exe_op <= insn[30] ? `EXOP_SAR : `EXOP_SHR;
                             pre_exe_b  <= {58'd0, insn[25:20]};
                          end
                          3'b110: pre_exe_op <= `EXOP_OR;    // ORI
                          3'b111: pre_exe_op <= `EXOP_AND;   // ANDI
                       endcase
                    end
                    5'b01100: begin // OP-REG: funct3+funct7[5] selects operation
                       pre_exe_b <= s2_bram;
                       case (insn[14:12])
                          3'b000: pre_exe_op <= insn[30] ? `EXOP_SUB : `EXOP_ADD;  // ADD/SUB
                          3'b001: pre_exe_op <= `EXOP_SHL;  // SLL
                          3'b010: pre_exe_op <= `EXOP_LTS;  // SLT
                          3'b011: pre_exe_op <= `EXOP_LTU;  // SLTU
                          3'b100: pre_exe_op <= `EXOP_XOR;  // XOR
                          3'b101: pre_exe_op <= insn[30] ? `EXOP_SAR : `EXOP_SHR;  // SRL/SRA
                          3'b110: pre_exe_op <= `EXOP_OR;   // OR
                          3'b111: pre_exe_op <= `EXOP_AND;  // AND
                          // MUL/DIV (funct7[0]=1): exe_add unused; default EXOP_OPB is fine
                       endcase
                    end
                    5'b00110: begin // OP-IMM-32 (W-type immediates)
                       pre_exe_sxt <= 1;
                       case (insn[14:12])
                          3'b000: begin pre_exe_op <= `EXOP_ADD; pre_exe_b <= d_imm_i; end  // ADDIW
                          3'b001: begin pre_exe_op <= `EXOP_SHL; pre_exe_b <= {59'd0, insn[24:20]}; end  // SLLIW
                          3'b101: begin  // SRLIW / SRAIW
                             pre_exe_op <= insn[30] ? `EXOP_SAR : `EXOP_SHR;
                             pre_exe_b  <= {59'd0, insn[24:20]};
                          end
                          default: ; // other funct3: no exe_add
                       endcase
                    end
                    5'b01110: begin // OP-REG-32 (W-type register)
                       pre_exe_sxt <= 1;
                       pre_exe_b <= s2_bram;
                       case (insn[14:12])
                          3'b000: pre_exe_op <= insn[30] ? `EXOP_SUB : `EXOP_ADD;  // ADDW/SUBW
                          3'b001: pre_exe_op <= `EXOP_SHL;  // SLLW
                          3'b101: pre_exe_op <= insn[30] ? `EXOP_SAR : `EXOP_SHR;  // SRLW/SRAW
                          // MUL/DIV-W: exe_add unused
                          default: ;
                       endcase
                    end
                    5'b01011: begin // AMO — SC.W/D fail path writes exe_add = 1
                       pre_exe_op <= `EXOP_ONE;
                    end
                    default: ; // LOAD, STORE, BRANCH, CSR, etc.: exe_add unused
                 endcase
              end
           end // rf3_pre_decode
        end

        `S_FETCH1B: begin
           // Register the async LUTRAM mem0/mem1 output into flip-flops.
           // mem_addr0/mem_addr1 were set (via <=) in S_FETCH1, so are stable now.
           // mem_data0/mem_data1 are combinatorial (async) and fully settled this cycle.
           // S_FETCH2 reads mem_data0_q/mem_data1_q instead of the LUTRAM directly,
           // breaking the LUTRAM-read → pte_latch-LUT → insn-reg timing path.
           mem_data0_q <= mem_data0;
           mem_data1_q <= mem_data1;
           state <= `S_FETCH2;
        end

        `S_LOAD_LATCH: begin
           // Register the async LUTRAM mem0/mem1 output into flip-flops.
           // mem_addr0/mem_addr1 were set (via <=) in the preceding state, so are stable.
           // S_LOAD_ALIGN uses mem_data0_q/mem_data1_q instead of the async LUTRAM wires,
           // breaking the mem_addr0_reg → LUTRAM → alignment → write_back_value_reg path.
           mem_data0_q <= mem_data0;
           mem_data1_q <= mem_data1;
           state <= `S_LOAD_ALIGN;
        end

        `S_EXECUTE: begin
           state <= `S_EXECUTE2; // Default: complete write_back_value

           imm_i = {{52{insn[31]}},insn[31:20]};
           imm_j = {{44{insn[31]}},insn[19:12],insn[20],insn[30:21],1'd0};
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

           csrno                   = insn[31:20];

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
              if ((insn & 'hffff) == 0) begin
                 write_back_register = 0;
                 cause = `TRAP_ILLEGAL_INSTRUCTION;
                 tval = insn;
                 state <= `S_EXCEPTION;
              end
           end

           //else if ((insn & 'he003) == 'h2000) begin // C.FLD
             //$display("c.fld   x%1d,%1d(x%1d)    %x UNTESTED", write_back_register, rs1, c_imm12_62, regfile[write_back_register]);
           //end

           else if ((insn & 'he003) == 'h4000) begin // C.LW
              write_back_register = rs2;
              load_size_lg2 = 2|4;
              mem_addr = s1 + c_uimm5_1210_6_x4;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[`MEM_SIZE_LG2-1:4];
              state <= `S_LOAD_LATCH;
           end

           else if ((insn & 'he003) == 'h6000) begin // C.LD
              write_back_register = rs2;
              load_size_lg2 = 3;
              mem_addr = s1 + c_uimm65_1210_x8;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              state <= `S_LOAD_LATCH;
           end

           //else if ((insn & 'he003) == 'ha000) begin // C.FSD
           //  $display("c.fsd   x%1d,%1d(x%1d)    UNTESTED", rs2, rs1, c_imm12_62);
           //end

           else if ((insn & 'he003) == 'hc000) begin // C.SW
              mem_addr = s1 + c_uimm5_1210_6_x4;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              mem_wr_mask = 15;
              store_value = s2;
              state <= `S_STORE;
           end

           else if ((insn & 'he003) == 'he000) begin // C.SD
              mem_addr = s1 + c_uimm65_1210_x8;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              mem_wr_mask = 255;
              store_value = s2;
              state <= `S_STORE;
           end


              // Quadrant 1
           else if (insn == 1) begin // C.NOP
             // NOP
           end

           else if ((insn & 'he003) == 'h0001) begin // C.ADDI
              write_back_register = rs1;
           end

           else if ((insn & 'he003) == 'h2001) begin // C.ADDIW
              write_back_register = rs1;
           end

           else if ((insn & 'he003) == 'h4001) begin // C.LI
              write_back_register = insn[11:7];
           end

           else if ((insn & 'hef83) == 'h6101) begin // C.ADDI16SP
              write_back_register = rs1;
           end

           else if ((insn & 'he003) == 'h6001) begin // C.LUI
              write_back_register = rs1;
           end

           else if ((insn & 'hec03) == 'h8001) begin // C.SRLI
              write_back_register = rs1;
           end

           else if ((insn & 'hec03) == 'h8401) begin // C.SRAI
              write_back_register = rs1;
           end

           else if ((insn & 'hec03) == 'h8801) begin // C.ANDI
              write_back_register = rs1;
           end

           else if ((insn & 'hfc63) == 'h8c01) begin // C.SUB
              write_back_register = rs1;
           end

           else if ((insn & 'hfc63) == 'h8c21) begin // C.XOR
              write_back_register = rs1;
           end

           else if ((insn & 'hfc63) == 'h8c41) begin // C.OR
              write_back_register = rs1;
           end

           else if ((insn & 'hfc63) == 'h8c61) begin // C.AND
              write_back_register = rs1;
           end

           else if ((insn & 'hfc63) == 'h9c01) begin // C.SUBW
              write_back_register = rs1;
           end

           else if ((insn & 'hfc63) == 'h9c21) begin // C.ADDW
              write_back_register = rs1;
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
           end

           //else if ((insn & 'he003) == 'h2002) begin // C.FLDSP
           //  $display("c.fldsp x%1d,%1d(sp)       %x UNTESTED", write_back_register, c_uimm42_12_65_x8, regfile[write_back_register]);
           //end

           else if ((insn & 'he003) == 'h4002) begin // C.LWSP
              write_back_register = insn[11:7]; // XXX this is a bit unclean
              mem_addr = s1 + c_uimm32_12_64_x4;
              load_size_lg2 = 2|4;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              state <= `S_LOAD_LATCH;
           end

           else if ((insn & 'he003) == 'h6002) begin // C.LDSP
              write_back_register = insn[11:7]; // XXX this is a bit unclean
              mem_addr = s1 + c_uimm42_12_65_x8;
              load_size_lg2 = 3;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              state <= `S_LOAD_LATCH;
           end

           else if ((insn & 'hf07f) == 'h8002) begin // C.JR
              npc = s1 & ~1;
           end

           else if ((insn & 'hf003) == 'h8002) begin // C.MV
              write_back_register = rs1;
           end

           else if ((insn & 'hffff) == 'h9002) begin // C.EBREAK
              cause = `TRAP_BREAKPOINT;
              tval = 0;
              state <= `S_EXCEPTION;
           end

           else if ((insn & 'hf07f) == 'h9002) begin // C.JALR
              write_back_register = 1;
              npc = s1 & ~1;
           end

           else if ((insn & 'hf003) == 'h9002) begin // C.ADD
              write_back_register = rs1;
           end

           // else if ((insn & 'he003) == 'ha002) begin // C.FSDSP
           //  $display("c.fsdsp x%1d,%1d(sp) UNTESTED", rs2, c_uimm97_1210_x8);
           // end

           else if ((insn & 'he003) == 'hc002) begin // C.SWSP
              mem_addr = s1 + c_uimm87_129_x4;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              mem_wr_mask = 15;
              store_value = s2;
              state <= `S_STORE;
           end

           else if ((insn & 'he003) == 'he002) begin // C.SDSP
              mem_addr = s1 + c_uimm97_1210_x8;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              mem_wr_mask = 255;
              store_value = s2;
              state <= `S_STORE;
           end

           // Quadrant 3, uncompressed
           else if ((insn & 'h0000007f) == 'h00000037) begin // LUI
              write_back_register = rd;
           end

           else if ((insn & 'h0000007f) == 'h00000017) begin // AUIPC
              write_back_register = rd;
           end

           else if ((insn & 'h0000007f) == 'h0000006f) begin // JAL
              write_back_register = rd;
              npc = pc + imm_j;
           end

           else if ((insn & 'h0000707f) == 'h00000067) begin // JALR
              write_back_register = rd;
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
              state <= `S_LOAD_LATCH;
           end

           else if ((insn & 'h0000707f) == 'h00001003) begin // LH
              write_back_register = rd;
              mem_addr = s1 + imm_i;
              load_size_lg2 = 1|4;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              state <= `S_LOAD_LATCH;
           end

           else if ((insn & 'h0000707f) == 'h00002003) begin // LW
              write_back_register = rd;
              mem_addr = s1 + imm_i;
              load_size_lg2 = 2|4;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              state <= `S_LOAD_LATCH;
           end

           else if ((insn & 'h0000707f) == 'h00003003) begin // LD
              write_back_register = rd;
              mem_addr = s1 + imm_i;
              load_size_lg2 = 3;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              state <= `S_LOAD_LATCH;
           end

           else if ((insn & 'h0000707f) == 'h00004003) begin // LBU
              write_back_register = rd;
              mem_addr = s1 + imm_i;
              load_size_lg2 = 0;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              state <= `S_LOAD_LATCH;
           end

           else if ((insn & 'h0000707f) == 'h00005003) begin // LHU
              write_back_register = rd;
              mem_addr = s1 + imm_i;
              load_size_lg2 = 1;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              state <= `S_LOAD_LATCH;
           end

           else if ((insn & 'h0000707f) == 'h00006003) begin // LWU
              write_back_register = rd;
              mem_addr = s1 + imm_i;
              load_size_lg2 = 2;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              state <= `S_LOAD_LATCH;
           end

           else if ((insn & 'h0000707f) == 'h00000023) begin // SB
              mem_addr = s1 + imm_s;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              mem_wr_mask = 1;
              store_value = s2;
              state <= `S_STORE;
           end

           else if ((insn & 'h0000707f) == 'h00001023) begin // SH
              mem_addr = s1 + imm_s;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              mem_wr_mask = 3;
              store_value = s2;
              state <= `S_STORE;
           end

           else if ((insn & 'h0000707f) == 'h00002023) begin // SW
              mem_addr = s1 + imm_s;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              mem_wr_mask = 15;
              store_value = s2;
              state <= `S_STORE;
           end

           else if ((insn & 'h0000707f) == 'h00003023) begin // SD
              mem_addr = s1 + imm_s;
              mem_addr0 <= mem_addr[63:4] + mem_addr[3];
              mem_addr1 <= mem_addr[63:4];
              mem_wr_mask = 255;
              store_value = s2;
              state <= `S_STORE;
           end

           else if ((insn & 'h0000707f) == 'h00000013) begin // ADDI
              write_back_register = rd;
           end

           else if ((insn & 'h0000707f) == 'h00002013) begin // SLTI
              write_back_register = rd;
           end

           else if ((insn & 'h0000707f) == 'h00003013) begin // SLTIU
              write_back_register = rd;
           end

           else if ((insn & 'h0000707f) == 'h00004013) begin // XORI
              write_back_register = rd;
           end

           else if ((insn & 'h0000707f) == 'h00006013) begin // ORI
              write_back_register = rd;
           end

           else if ((insn & 'h0000707f) == 'h00007013) begin // ANDI
              write_back_register = rd;
           end

           else if ((insn & 'hfe00707f) == 'h00000033) begin // ADD
              write_back_register = rd;
           end

           else if ((insn & 'hfe00707f) == 'h40000033) begin // SUB
              write_back_register = rd;
           end

           else if ((insn & 'hfe00707f) == 'h00001033) begin // SLL
              write_back_register = rd;
           end

           else if ((insn & 'hfe00707f) == 'h00002033) begin // SLT
              write_back_register = rd;
           end

           else if ((insn & 'hfe00707f) == 'h00003033) begin // SLTU
              write_back_register = rd;
           end

           else if ((insn & 'hfe00707f) == 'h00004033) begin // XOR
              write_back_register = rd;
           end

           else if ((insn & 'hfe00707f) == 'h00005033) begin // SRL
              write_back_register = rd;
           end

           else if ((insn & 'hfe00707f) == 'h40005033) begin // SRA
              write_back_register = rd;
           end

           else if ((insn & 'hfe00707f) == 'h00006033) begin // OR
              write_back_register = rd;
           end

           else if ((insn & 'hfe00707f) == 'h00007033) begin // AND
              write_back_register = rd;
           end

           else if ((insn & 'hf000707f) == 'h0000000f) begin // FENCE
              // Nothing to do here
           end

           else if ((insn & 'hf000707f) == 'h8000000f) begin // FENCE.TSO
              // Nothing to do here
           end

           else if ((insn & 'hffffffff) == 'h00000073) begin // ECALL
              cause = `TRAP_ENVIRONMENT_CALL_FROM_U_MODE + prv;
              tval = 0;
              state <= `S_EXCEPTION;
`ifdef SIMULATE
`ifdef VERBOSE
              $display("ECALL: pc %x prv %d time %0t", pc, prv, $time);
`endif
`endif
           end

           else if ((insn & 'hffffffff) == 'h00100073) begin // EBREAK
              cause = `TRAP_BREAKPOINT;
              tval = 0;
              state <= `S_EXCEPTION;
           end

           else if ((insn & 'hfc00707f) == 'h00001013) begin // SLLI
              write_back_register = rd;
           end

           else if ((insn & 'hfc00707f) == 'h00005013) begin // SRLI
              write_back_register = rd;
           end

           else if ((insn & 'hfc00707f) == 'h40005013) begin // SRAI
              write_back_register = rd;
           end

           else if ((insn & 'h0000707f) == 'h0000001b) begin // ADDIW
              write_back_register = rd;
           end

           else if ((insn & 'hfe00707f) == 'h0000101b) begin // SLLIW
              write_back_register = rd;
           end

           else if ((insn & 'hfe00707f) == 'h0000501b) begin // SRLIW
              write_back_register = rd;
           end

           else if ((insn & 'hfe00707f) == 'h4000501b) begin // SRAIW
              // NB: Yes, this is a crazy instruction with *two*
              // sign-extensions and it does _not_ behave like the MIPS
              // counterpart
              write_back_register = rd;
           end

           else if ((insn & 'hfe00707f) == 'h0000003b) begin // ADDW
              write_back_register = rd;
           end

           else if ((insn & 'hfe00707f) == 'h4000003b) begin // SUBW
              write_back_register = rd;
           end

           else if ((insn & 'hfe00707f) == 'h0000103b) begin // SLLW
              write_back_register = rd;
           end

           else if ((insn & 'hfe00707f) == 'h0000503b) begin // SRLW
              write_back_register = rd;
           end

           else if ((insn & 'hfe00707f) == 'h4000503b) begin // SRAW
              // NB: Yes, this is a crazy instruction with *two*
              // sign-extensions and it does _not_ behave like the MIPS
              // counterpart
              write_back_register = rd;
           end

           else if ((insn & 'hffffffff) == 'h0000100f) begin // FENCE.I
              // Nothing to do here [yet]
           end

           else if ((insn & 'h0000707f) == 'h00001073) begin // CSRRW
              // CSRRW and CSRRWI (and only those) do not read the CSR
              // if rd == 0 This matters [only] if the read has side
              // effects (I'm guilty of this part of RISC-V semantics).
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
              muldiv_output_negate = s1[63] != s2[63];
              mul_a = {64'd0,s1[63] ? -s1 : s1};
              mul_b = s2[63] ? -s2 : s2;
              muldiv_output_high_part = 1;
              state <= `S_MUL_RUNNING;
           end

           else if ((insn & 'hfe00707f) == 'h02002033) begin // MULHSU
              write_back_register = rd;
              muldiv_output_negate = s1[63];
              mul_a = {64'd0, s1[63] ? -s1 : s1};
              mul_b = s2;
              muldiv_output_high_part = 1;
              state <= `S_MUL_RUNNING;
           end

           else if ((insn & 'hfe00707f) == 'h02003033) begin // MULHU
              write_back_register = rd;
              mul_a = {64'd0, s1};
              mul_b = s2;
              muldiv_output_high_part = 1;
              state <= `S_MUL_RUNNING;
           end


           else if ((insn & 'hfe00707f) == 'h02004033) begin // DIV
              write_back_register = rd;
              muldiv_output_negate = s1[63] != s2[63];
              if (s2 == 0)
                // No matter s1, this will produce -1 which is the correct answer
                muldiv_output_negate = 0;
              div_count = 64;
              muldiv_p = {64'd0,s1[63] ? -s1 : s1};
              mul_a = {s2[63] ? -s2 : s2, 63'd0};
              mul_b = 0;
              state <= `S_DIV_RUNNING;
           end

           else if ((insn & 'hfe00707f) == 'h02005033) begin // DIVU
              write_back_register = rd;
              div_count = 64;
              muldiv_p = {64'd0, s1};
              mul_a = {s2, 63'd0};
              mul_b = 0;
              state <= `S_DIV_RUNNING;
           end

           else if ((insn & 'hfe00707f) == 'h02006033) begin // REM
              write_back_register = rd;
              // "For REM, the sign of a nonzero result equals the sign of the dividend."
              muldiv_output_negate = s1[63];
              if (s2 == 0)
                // No matter s1, this will produce -1 which is the correct answer
                muldiv_output_negate = 0;
              div_count = 64;
              muldiv_p = {64'd0,s1[63] ? -s1 : s1};
              mul_a = {s2[63] ? -s2 : s2, 63'd0};
              mul_b = 0;
              muldiv_output_high_part = 1; // XXX abusing variables
              state <= `S_DIV_RUNNING;
           end

           else if ((insn & 'hfe00707f) == 'h02007033) begin // REMU
              write_back_register = rd;
              // "For REM, the sign of a nonzero result equals the sign of the dividend."
              if (s2 == 0)
                // No matter s1, this will produce -1 which is the correct answer
                muldiv_output_negate = 0;
              div_count = 64;
              muldiv_p = {64'd0,s1};
              mul_a = {s2, 63'd0};
              mul_b = 0;
              muldiv_output_high_part = 1; // XXX abusing variables
              state <= `S_DIV_RUNNING;
           end

           else if ((insn & 'hfe00707f) == 'h0200003b) begin // MULW
              write_back_register = rd;
              mul_a = s1[31:0];
              mul_b = s2[31:0];
              muldiv_output_sext32 = 1;
              state <= `S_MUL_RUNNING;
           end

           else if ((insn & 'hfe00707f) == 'h0200403b) begin // DIVW
              write_back_register = rd;
              muldiv_output_negate = s1[31] != s2[31];
              if (s2 == 0)
                // No matter s1, this will produce -1 which is the correct answer
                muldiv_output_negate = 0;
              div_count = 32;
              muldiv_p = {96'd0,s1[31] ? -s1[31:0] : s1[31:0]};
              mul_a = {s2[31] ? -s2[31:0] : s2[31:0], 31'd0};
              mul_b = 0;
              muldiv_output_sext32 = 1;
              state <= `S_DIV_RUNNING;
           end

           else if ((insn & 'hfe00707f) == 'h0200503b) begin // DIVUW
              write_back_register = rd;
              if (s2 == 0)
                // No matter s1, this will produce -1 which is the correct answer
                muldiv_output_negate = 0;
              div_count = 32;
              muldiv_p = {96'd0, s1[31:0]};
              mul_a = {s2[31:0], 31'd0};
              mul_b = 0;
              muldiv_output_sext32 = 1;
              state <= `S_DIV_RUNNING;
           end

           else if ((insn & 'hfe00707f) == 'h0200603b) begin // REMW
              write_back_register = rd;
              // "For REM, the sign of a nonzero result equals the sign of the dividend."
              muldiv_output_negate = s1[31];
              div_count = 32;
              muldiv_p = {96'd0,s1[31] ? -s1[31:0] : s1[31:0]};
              mul_a = {s2[31] ? -s2[31:0] : s2[31:0], 31'd0};
              mul_b = 0;
              muldiv_output_sext32 = 1;
              muldiv_output_high_part = 1; // XXX abusing variables
              state <= `S_DIV_RUNNING;
           end

           else if ((insn & 'hfe00707f) == 'h0200703b) begin // REMUW
              write_back_register = rd;
              // "For REM, the sign of a nonzero result equals the sign of the dividend."
              div_count = 32;
              muldiv_p = {96'd0, s1[31:0]};
              mul_a = {s2[31:0], 31'd0};
              mul_b = 0;
              muldiv_output_sext32 = 1;
              muldiv_output_high_part = 1; // XXX abusing variables
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
              state <= `S_LOAD_LATCH;
           end

           else if ((insn & 'hf800707f) == 'h1800202f || // SC.W
                    (insn & 'hf800707f) == 'h1800302f)   // SC.D
           begin
              write_back_register = rd;
              if (reservation == s1) begin // XXX Should use physical address
                 write_back_value <= 0;
                 mem_addr = s1;
                 mem_addr0 <= mem_addr[63:4] + mem_addr[3];
                 mem_addr1 <= mem_addr[63:4];
                 mem_wr_mask = insn[12] ? 255 : 15;
                 store_value = s2;
                 state <= `S_STORE;
              end else begin
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
              state <= `S_LOAD_LATCH;
           end

           else if ((insn & 'hffffffff) == 'h30200073) begin // MRET
              if (mpp != 3) mprv = 0;
              prv = mpp;
              mpp = 0;
              mie = mpie;
              mpie = 1;
              npc = csr_mepc;
              state <= `S_FETCH1;
           end

           else if ((insn & 'hffffffff) == 'h10200073) begin // SRET
              if (prv == 0 || prv == 1 && tsr) begin
                 cause = `TRAP_ILLEGAL_INSTRUCTION;
                 tval = insn;
                 state <= `S_EXCEPTION;
              end else begin
`ifdef SIMULATE
`ifdef VERBOSE
                 $display("SRET: pc %x prv %d->%d sepc %x time %0t", pc, prv, spp, csr_sepc, $time);
`endif
`endif
                 mprv = 0; // sret can only return to S or U, never M
                 prv = spp;
                 spp = 0;
                 sie = spie;
                 spie = 1;
                 npc = csr_sepc;
              end
           end

           else if ((insn & 'hfe007fff) == 'h12000073) begin // SFENCE.VMA
              if (prv < 1 || prv == 1 && tvm) begin
                 cause = `TRAP_ILLEGAL_INSTRUCTION;
                 tval = insn;
                 state <= `S_EXCEPTION;
              end
           end

           else if ((insn & 'hffffffff) == 'h10500073) begin // WFI
              if (prv == 0 || prv == 1 && tw) begin
                 cause = `TRAP_ILLEGAL_INSTRUCTION;
                 tval = insn;
                 state <= `S_EXCEPTION;
              end else
                state <= `S_FETCH1; // treat as NOP (no real sleep in simulation)
           end

           else begin
`ifdef SIMULATE
`ifdef VERBOSE
              if (insn[1:0] == 3)
                $display("%05d   %1d %x %x illegal unknown instruction", $time, prv, pc, insn);
              else
                $display("%05d   %1d %x     %x illegal unknown instruction (%1d,%1d)",
                         $time, prv, pc, insn[15:0], insn[15:13], insn[1:0]);
              $finish;
`endif
`endif
              cause = `TRAP_ILLEGAL_INSTRUCTION;
              tval = insn;
              state <= `S_EXCEPTION;
           end

           // Pre-registered ALU computation: uses pre_exe_op/pre_exe_b decoded in S_RF3.
           // Both operands (s1, pre_exe_b) and selector (pre_exe_op) are flip-flops,
           // so the critical path is only ~7 LUT levels (vs ~15 with the inline if-else).
           case (pre_exe_op)
              `EXOP_ADD: exe_add <= pre_exe_sxt
                            ? {32'd0, s1[31:0] + pre_exe_b[31:0]}
                            : s1 + pre_exe_b;
              `EXOP_SUB: exe_add <= pre_exe_sxt
                            ? {32'd0, s1[31:0] - pre_exe_b[31:0]}
                            : s1 - pre_exe_b;
              `EXOP_SHL: exe_add <= pre_exe_sxt
                            ? {32'd0, s1[31:0] << pre_exe_b[4:0]}
                            : s1 << pre_exe_b[5:0];
              `EXOP_SHR: exe_add <= pre_exe_sxt
                            ? {32'd0, s1[31:0] >> pre_exe_b[4:0]}
                            : s1 >> pre_exe_b[5:0];
              // EXOP_SAR: use if/else to avoid ternary mixing signed/unsigned arms
              // (Verilog coerces $signed(s1)>>>n to unsigned/logical when the
              //  other ternary arm is unsigned, breaking arithmetic right shift)
              `EXOP_SAR: if (pre_exe_sxt)
                            exe_add <= {32'd0, $signed(s1[31:0]) >>> pre_exe_b[4:0]};
                         else
                            exe_add <= $signed(s1) >>> pre_exe_b[5:0];
              `EXOP_XOR: exe_add <= s1 ^ pre_exe_b;
              `EXOP_OR:  exe_add <= s1 | pre_exe_b;
              `EXOP_AND: exe_add <= s1 & pre_exe_b;
              `EXOP_LTS: exe_add <= $signed(s1) < $signed(pre_exe_b) ? 1 : 0;
              `EXOP_LTU: exe_add <= s1 < pre_exe_b ? 1 : 0;
              `EXOP_OPB: exe_add <= pre_exe_b;
              `EXOP_ONE: exe_add <= 1;
              default:   exe_add <= 0;
           endcase
           exe_sext32 <= pre_exe_sxt;

        end

        `S_EXECUTE2: begin
           write_back_value <= exe_sext32 ? {{32{exe_add[31]}}, exe_add[31:0]} : exe_add;
           state <= `S_FETCH1;
        end

        `S_STORE: begin

           if (csr_satp[63:60] == 4'd8 && (mprv ? mpp : prv) != 3 && !translated) begin
              // Sv39 store address translation
              ptw_va = mem_addr;
              ptw_level = 2;
              ptw_access = 2;
              ptw_prv = mprv ? mpp : prv;
              ptw_return = `S_STORE;
              ptw_pte_addr = {8'd0, csr_satp[43:0], 12'd0} + {52'd0, mem_addr[38:30], 3'd0};
              if (ptw_pte_addr[31] && ptw_pte_addr[63:`MEM_SIZE_LG2] != `MEM_BASEADDR >> `MEM_SIZE_LG2) begin
                 ptw_from_dram <= 1;
                 dram_burst_addr = ptw_pte_addr[30:5];
                 dram_read       = 1;
                 state          <= `S_DRAM_PTW_WAIT;
              end else begin
                 ptw_from_dram <= 0;
                 mem_addr0 <= ptw_pte_addr[`MEM_SIZE_LG2-1:4] + ptw_pte_addr[3];
                 mem_addr1 <= ptw_pte_addr[`MEM_SIZE_LG2-1:4];
                 state <= `S_PTW_READ;
              end
           end else begin
           translated <= 0;
           state <= `S_FETCH1;
           reservation <= ~0;

           if (mem_addr[63:4] == 60'h100_0000) begin
              // NS16550A UART write (0x10000000-0x1000000F)
              case (mem_addr[2:0])
                0: if (!uart_lcr[7]) begin // THR (when DLAB=0)
                      uart_tx_valid <= 1;
                      uart_tx_data <= store_value[7:0];
                   end
                1: if (!uart_lcr[7]) uart_ier <= store_value[3:0]; // Only bits [3:0] valid
                2: begin // FCR (write-only)
                   uart_fcr_fifo <= store_value[0];
                   if (store_value[1]) begin uart_rx_head <= 0; uart_rx_tail <= 0; end
                end
                3: uart_lcr <= store_value[7:0];
                4: uart_mcr <= store_value[4:0];
                7: uart_scr <= store_value[7:0];
              endcase
              mem_wr_mask = 0;
           end else if (mem_addr[63:16] == 48'h0200) begin
              // CLINT: 0x02000000 msip, 0x02004000 mtimecmp, 0x0200BFF8 mtime
              case (mem_addr[15:0])
                16'h0000: clint_msip <= store_value[0];
                16'h4000: if (mem_wr_mask[4])
                             clint_mtimecmp <= store_value;
                          else
                             clint_mtimecmp[31:0] <= store_value[31:0];
                16'h4004: clint_mtimecmp[63:32] <= store_value[31:0];
                16'hBFF8: if (mem_wr_mask[4])
                             clint_mtime <= store_value;
                          else
                             clint_mtime[31:0] <= store_value[31:0];
                16'hBFFC: clint_mtime[63:32] <= store_value[31:0];
              endcase
              mem_wr_mask = 0;
           end else if (mem_addr[63:24] == 40'h0C) begin
              // PLIC write (base 0x0C000000)
              if (mem_addr[23:0] <= 24'h0000FF)
                 plic_priority[mem_addr[7:2]] <= store_value[2:0];
              else if (mem_addr[23:0] >= 24'h002080 && mem_addr[23:0] <= 24'h002087) begin
                 if (mem_wr_mask[4])
                    plic_enabled <= store_value;
                 else if (mem_addr[2])
                    plic_enabled[63:32] <= store_value[31:0];
                 else
                    plic_enabled[31:0] <= store_value[31:0];
              end else if (mem_addr[23:0] >= 24'h201000 && mem_addr[23:0] <= 24'h201003)
                 plic_threshold <= store_value[2:0];
              else if (mem_addr[23:0] >= 24'h201004 && mem_addr[23:0] <= 24'h201007) begin
                 // Claim complete: clear pending bit
                 if (store_value[5:0] != 0)
                    plic_pending[store_value[5:0]] <= 0;
              end
              mem_wr_mask = 0;
           end else if (mem_addr[63:`MEM_SIZE_LG2] == `MEM_BASEADDR >> `MEM_SIZE_LG2) begin
              // BRAM store: fall through to BRAM write code below (mem_wr_mask stays set)
           end else if (mem_addr[63:31] == 0) begin
`ifdef TRACE_MMIO
              $display("%05d  MMIO WRITE %x/%x <- %x", $time, mem_addr, mem_wr_mask, store_value);
`endif

              mmio_address = mem_addr;
              mmio_write = 1;
              mmio_writedata = store_value << (8 * (mem_addr % 4));
              mmio_byteenable = mem_wr_mask << (mem_addr % 4);

              mem_wr_mask = 0;
           end else if (mem_addr[63:31] == 1) begin
              // DRAM store (0x80000000-0xFFFFFFFF)
              begin : dram_store_calc
                 reg [511:0] wide_data;
                 reg  [63:0] wide_mask;
                 wide_data = {448'd0, store_value} << (mem_addr[4:0] * 8);
                 wide_mask = {48'd0,  mem_wr_mask}  << mem_addr[4:0];
                 dram_burst_addr = mem_addr[30:5];
                 dram_writedata  = wide_data[255:0];
                 dram_byte_mask  = ~wide_mask[31:0];
                 dram_write      = 1;
                 mem_wr_mask     = 0;
                 if (|wide_mask[63:32]) begin
                    // Overflow into next burst: save for S_DRAM_STORE2
                    dram2_addr        <= mem_addr[30:5] + 1;
                    dram2_data_part   <= wide_data[319:256];
                    dram2_mask        <= ~wide_mask[63:32];
                    dram_store_split  <= 1;
                    state             <= dram_write_ready ? `S_DRAM_STORE2 : `S_DRAM_STORE_WAIT;
                 end else begin
                    dram_store_split  <= 0;
                    if (!dram_write_ready) state <= `S_DRAM_STORE_WAIT;
                 end
              end
           end else begin
`ifdef SIMULATE
`ifdef VERBOSE
              $display("%05d   %x xxxxxxxx illegal store address %x", $time, prv, mem_addr);
`endif
`endif
              cause = `TRAP_STORE_ACCESS_FAULT;
              tval = mem_addr;
              mem_wr_mask = 0;
              state <= `S_EXCEPTION;
           end

           aligned = {64'd0,store_value} << (8 * (mem_addr % 8));
           mem_wr_mask = mem_wr_mask << (mem_addr % 8);
           if (mem_addr[3]) begin
              aligned = {aligned[63:0], aligned[127:64]};
              mem_wr_mask = {mem_wr_mask[7:0],mem_wr_mask[15:8]};
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

`ifdef RISCV_TESTS
           // tohost detection: store to tohost address terminates simulation
           if (mem_addr0 == tohost_phys[`MEM_SIZE_LG2-1:4] + tohost_phys[3] &&
               mem_wr_mask[0] && store_value != 0) begin
              if (store_value == 1)
                 $display("Test Passed");
              else
                 $display("Test Failed with %3d", store_value);
              $finish;
           end
`endif
           end // else (translated)
        end

        `S_LOAD_ALIGN: begin
           if (csr_satp[63:60] == 4'd8 && (mprv ? mpp : prv) != 3 && !translated) begin
              // Sv39 load/AMO address translation
              ptw_va = mem_addr;
              ptw_level = 2;
              ptw_access = do_atomic ? 2'd3 : 2'd1;
              ptw_prv = mprv ? mpp : prv;
              ptw_return = `S_LOAD_LATCH;
              ptw_pte_addr = {8'd0, csr_satp[43:0], 12'd0} + {52'd0, mem_addr[38:30], 3'd0};
              if (ptw_pte_addr[31] && ptw_pte_addr[63:`MEM_SIZE_LG2] != `MEM_BASEADDR >> `MEM_SIZE_LG2) begin
                 ptw_from_dram <= 1;
                 dram_burst_addr = ptw_pte_addr[30:5];
                 dram_read       = 1;
                 state          <= `S_DRAM_PTW_WAIT;
              end else begin
                 ptw_from_dram <= 0;
                 mem_addr0 <= ptw_pte_addr[`MEM_SIZE_LG2-1:4] + ptw_pte_addr[3];
                 mem_addr1 <= ptw_pte_addr[`MEM_SIZE_LG2-1:4];
                 state <= `S_PTW_READ;
              end
           end else begin
              if (!do_atomic) translated <= 0;

              aligned = mem_addr[3] ? {mem_data0_q, mem_data1_q} : {mem_data1_q, mem_data0_q};
              aligned = aligned >> (mem_addr[2:0] * 8);

              case (load_size_lg2)
                0: write_back_value = aligned[ 7:0];
                1: write_back_value = aligned[15:0];
                2: write_back_value = aligned[31:0];
                3: write_back_value = aligned;
                4: write_back_value = {{56{aligned[ 7]}},aligned[ 7:0]};
                5: write_back_value = {{48{aligned[15]}},aligned[15:0]};
                6: write_back_value = {{32{aligned[31]}},aligned[31:0]};
                7: write_back_value = 64'hx;
              endcase

              state <= `S_FETCH1;

              if (do_atomic)
                state <= `S_AMO;

              if (mem_addr[63:4] == 60'h100_0000) begin
                 // NS16550A UART read (0x10000000-0x1000000F)
                 case (mem_addr[2:0])
                   0: if (!uart_lcr[7]) begin // RBR (when DLAB=0)
                         write_back_value = uart_rx_empty ? 0 : uart_rx_fifo[uart_rx_head];
                         if (!uart_rx_empty) uart_rx_head <= uart_rx_head + 1;
                      end else
                         write_back_value = 0; // DLL (divisor, ignored)
                   1: write_back_value = uart_lcr[7] ? 0 : {4'd0, uart_ier[3:0]};
                   2: write_back_value = uart_iir;
                   3: write_back_value = uart_lcr;
                   4: write_back_value = uart_mcr;
                   5: write_back_value = uart_lsr;
                   6: write_back_value = 8'hB0; // MSR: CTS+DSR+CD asserted
                   7: write_back_value = uart_scr;
                   default: write_back_value = 0;
                 endcase
              end else if (mem_addr[63:16] == 48'h0200) begin
                 // CLINT read: return value directly, no MMIO bus
                 case (mem_addr[15:0])
                   16'h0000: write_back_value = {63'd0, clint_msip};
                   16'h4000: write_back_value = clint_mtimecmp;
                   16'h4004: write_back_value = clint_mtimecmp[63:32];
                   16'hBFF8: write_back_value = clint_mtime;
                   16'hBFFC: write_back_value = clint_mtime[63:32];
                   default:  write_back_value = 0;
                 endcase
                 // 32-bit loads need sign/zero extension
                 if (load_size_lg2 == 2)
                    write_back_value = write_back_value[31:0];
                 else if (load_size_lg2 == 6)
                    write_back_value = {{32{write_back_value[31]}}, write_back_value[31:0]};
              end else if (mem_addr[63:24] == 40'h0C) begin
                 // PLIC read (base 0x0C000000)
                 if (mem_addr[23:0] <= 24'h0000FF)
                    write_back_value = plic_priority[mem_addr[7:2]];
                 else if (mem_addr[23:0] >= 24'h001000 && mem_addr[23:0] <= 24'h00107F)
                    write_back_value = plic_pending >> ((mem_addr[6:0] - 7'h00) * 8);
                 else if (mem_addr[23:0] >= 24'h002080 && mem_addr[23:0] <= 24'h002087)
                    write_back_value = plic_enabled;
                 else if (mem_addr[23:0] >= 24'h201000 && mem_addr[23:0] <= 24'h201003)
                    write_back_value = plic_threshold;
                 else if (mem_addr[23:0] >= 24'h201004 && mem_addr[23:0] <= 24'h201007)
                    write_back_value = plic_best_irq;
                 else
                    write_back_value = 0;
                 if (load_size_lg2 == 2)
                    write_back_value = write_back_value[31:0];
                 else if (load_size_lg2 == 6)
                    write_back_value = {{32{write_back_value[31]}}, write_back_value[31:0]};
              end else if (mem_addr[63:`MEM_SIZE_LG2] == `MEM_BASEADDR >> `MEM_SIZE_LG2) begin
                 // BRAM load: write_back_value already computed from speculative read above
              end else if (mem_addr[63:31] == 0) begin
`ifdef TRACE_MMIO
                 $display("%05d  MMIO READ FROM %x/%x", $time, mem_addr, load_size_lg2);
`endif
                 state <= `S_MMIO_READ;

                 mmio_address = mem_addr;
                 mmio_read = 1;
              end else if (mem_addr[63:31] == 1) begin
                 // DRAM load (0x80000000-0xFFFFFFFF)
                 dram_burst_addr = mem_addr[30:5];
                 dram_read       = 1;
                 state          <= `S_DRAM_LOAD_WAIT;
              end else begin
`ifdef SIMULATE
`ifdef VERBOSE
                 $display("%05d   %x xxxxxxxx illegal load address %x", $time, prv, mem_addr);
`endif
`endif
                 write_back_register = 0;
                 cause = `TRAP_LOAD_ACCESS_FAULT;
                 tval = mem_addr;
                 state <= `S_EXCEPTION;
              end
           end
        end

        `S_MMIO_READ: state <= `S_MMIO_ALIGN;

        `S_MMIO_ALIGN: if (mmio_readdatavalid) begin

           aligned = mmio_readdata >> (mem_addr[1:0] * 8);

/*
           $display("%x >> %d = %x ==? %x",
                    mmio_readdata,
                    (mem_addr[1:0] * 8),
                    mmio_readdata >> (mem_addr[1:0] * 8),
                    aligned);
*/

           case (load_size_lg2)
             0: write_back_value = aligned[ 7:0];
             1: write_back_value = aligned[15:0];
             2: write_back_value = mmio_readdata;
             3: write_back_value = 64'h 6464DEADDEAD6464;
             4: write_back_value = {{56{aligned[ 7]}},aligned[ 7:0]};
             5: write_back_value = {{48{aligned[15]}},aligned[15:0]};
             6: write_back_value = {{32{mmio_readdata[31]}},mmio_readdata[31:0]};
             7: write_back_value = 64'h DEAD_BEEF_C0DE_CAFE;
           endcase

`ifdef TRACE_MMIO
           $display("%05d  MMIO READ GOT %x (aligned %x)", $time, mmio_readdata, write_back_value);
`endif

           state <= `S_FETCH1;

           if (do_atomic) begin
`ifdef SIMULATE
              $display("Sorry, atomics to MMIO aren't supported yet");
              $finish;
`endif
              state <= `S_AMO;
           end
        end

        `S_AMO: begin
           // Note: translated stays set from S_LOAD_ALIGN PTW (ptw_access=3
           // already checked both read and write permission), so S_STORE
           // will skip re-translation and use the physical mem_addr directly.
           mem_wr_mask = 255;
           store_value = s2;
           if (!insn[12]) begin
              write_back_value = {{32{write_back_value[31]}},write_back_value[31:0]};
              store_value = {{32{s2[31]}},s2[31:0]};
              mem_wr_mask = 15;
           end

           // insn[31:27]=funct5, insn[26]=aq, insn[25]=rl, insn[24]=rs2[4]
           // Mask aq/rl/rs2[4] so any register and any ordering variant is handled.
           case ({insn[31:27], 3'b0})
             'h08: begin end // AMOSWAP
             'h00: store_value = store_value + write_back_value; // AMOADD
             'h20: store_value = store_value ^ write_back_value; // AMOXOR
             'h60: store_value = store_value & write_back_value; // AMOAND
             'h40: store_value = store_value | write_back_value; // AMOOR
             'h80: store_value = $signed(store_value) < $signed(write_back_value) ? store_value : write_back_value; // AMOMIN
             'ha0: store_value = $signed(store_value) < $signed(write_back_value) ? write_back_value : store_value; // AMOMAX
             'hc0: store_value = store_value < write_back_value ? store_value : write_back_value; // AMOMINU
             'he0: store_value = store_value < write_back_value ? write_back_value : store_value; // AMOMAXU
             default: begin
`ifdef SIMULATE
                $display("Impossible AMO"); $finish;
`endif
                store_value = 64'hX;
             end
           endcase

           state <= `S_STORE;
        end


        `S_HANDLE_CSR: begin
           state <= `S_FETCH1;
           csr_access_failure = 0;
           write_back_register = rd;

           if (rd != 0 || csr_op != `CSR_OP_COPY) begin
              // read the CSR

              // PMP: pmpcfg0-15 and pmpaddr0-63 — M-mode only, reads zero
              // (0 PMP entries implemented; all accesses permitted).
              if ('h3A0 <= csrno && csrno <= 'h3FF) csr_read_val = 0;
              else case (csrno)
                `CSR_SSTATUS:
                  csr_read_val = {sd, 29'd0,            uxl, 12'd0,  // 63:20
                                                    mxr, sum, 1'd0,  // 19:17
                                  xs,   fs,         4'd0,      spp,  // 16: 8
                                  2'd0, spie, upie, 2'd0, sie, uie}; //  7: 0
                `CSR_SIE:       csr_read_val = csr_mie & 'h222;
                `CSR_STVEC:     csr_read_val = csr_stvec;
                `CSR_SCOUNTEREN:csr_read_val = 0;
                `CSR_SSCRATCH:  csr_read_val = csr_sscratch;
                `CSR_SEPC:      csr_read_val = csr_sepc;
                `CSR_SCAUSE:    csr_read_val = csr_scause;
                `CSR_STVAL:     csr_read_val = csr_stval;
                `CSR_SIP:       csr_read_val = csr_mip & csr_mideleg;

                `CSR_SATP: begin
                   if (prv == 1 && tvm) begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
                   end else
                     csr_read_val = csr_satp;
                end
                `CSR_MSTATUS:
                  csr_read_val = {sd, 27'd0,  sxl,  uxl, 9'd0,                  // 63:23
                                              tsr,  tw,   tvm, mxr, sum, mprv,  // 22:17
                                  xs,         fs,         mpp, 2'd0,      spp,  // 16: 8
                                  mpie, 1'd0, spie, upie, mie, 1'd0, sie, uie}; //  7: 0
                `CSR_MISA:     csr_read_val = 64'h800000000014112d; // 64'h800000000014112d with FD
                // Hardwired 1 0100 0001 0001 0010 1101
                //    ZY XWV U TSRQ PONM LKJI HGFE DCBA
                //           U  S      M    I   F  DC A
                //    SUIMAFDC
                // -                            F  D
                // =         1 0100 0001 0001 0000 0101 = 1105
                `CSR_MEDELEG:  csr_read_val = csr_medeleg;
                `CSR_MIDELEG:  csr_read_val = csr_mideleg;
                `CSR_MIE:      csr_read_val = csr_mie;
                `CSR_MTVEC:    csr_read_val = csr_mtvec;
                `CSR_MCOUNTEREN: csr_read_val = 0;
                `CSR_MSCRATCH: csr_read_val = csr_mscratch;
                `CSR_MEPC:     csr_read_val = csr_mepc;
                `CSR_MCAUSE:   csr_read_val = csr_mcause;
                `CSR_MTVAL:    csr_read_val = csr_mtval;
                `CSR_MIP:      csr_read_val = csr_mip;
                `CSR_TSELECT:  csr_read_val = 0;
                `CSR_TDATA1:   csr_read_val = 0;
                `CSR_TDATA2:   csr_read_val = 0;
                `CSR_TDATA3:   csr_read_val = 0;
                `CSR_TINFO:    csr_read_val = 0;
                `CSR_MCYCLE:   csr_read_val = csr_mcycle;
                `CSR_MTIME:    csr_read_val = clint_mtime; // XXX I'm not sure this is what we want
                `CSR_MINSTRET: csr_read_val = csr_minstret;
                `CSR_CYCLE:    csr_read_val = csr_mcycle;
                `CSR_TIME:     csr_read_val = clint_mtime; // XXX I'm not sure this is what we want
                `CSR_INSTRET:  csr_read_val = csr_minstret;
                `CSR_MHARTID:  csr_read_val = 0;
                `CSR_MVENDORID:csr_read_val = 0;
                `CSR_MARCHID:  csr_read_val = 9; // YARVI, Smolrv64 = YARVI4
                `CSR_MIMPID:   csr_read_val = 'h20250907;
                default: begin
`ifdef SIMULATE
`ifdef VERBOSE
                   $display("%05d   %1d %x %x illegal CSR %x (read)", $time, prv, pc, insn, csrno);
`endif
`endif
                   cause = `TRAP_ILLEGAL_INSTRUCTION;
                   tval = insn;
                   state <= `S_EXCEPTION;
                end
              endcase

              // As no side effects (beside exception have happend, we
              // can postpone the priviledge check to here
              if (prv < csrno[9:8]) begin
`ifdef SIMULATE
`ifdef VERBOSE
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
              if (prv < csrno[9:8]) begin
                 csr_access_failure = 1;
`ifdef SIMULATE
`ifdef VERBOSE
                 $display("%05d   %1d %x %x mode isn't priviledged to write CSR %x", $time,
                          prv, pc, insn, csrno);
`endif
`endif
              end

              if (csrno[11:10] == 3) begin
`ifdef SIMULATE
`ifdef VERBOSE
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

              // PMP: pmpcfg0-15 and pmpaddr0-63 — M-mode only, writes silently ignored
              // (0 PMP entries implemented; all accesses permitted).
              if ('h3A0 <= csrno && csrno <= 'h3FF) begin end
              else case (csrno)
                `CSR_SSTATUS: begin
`ifdef SIMULATE
`ifdef VERBOSE
                   if (sum != csr_write_val[18])
                      $display("SSTATUS: sum %d->%d pc %x time %0t", sum, csr_write_val[18], pc, $time);
`endif
`endif
                   {mxr, sum}        = csr_write_val[19:18];
                   fs                = csr_write_val[14:13];
                   spp               = csr_write_val[8];
                   {spie, upie}      = csr_write_val[5:4];
                   {sie, uie}        = csr_write_val[1:0];
                end
                `CSR_SIE:       csr_mie    = csr_write_val & 'h222 | csr_mie & ~'h222;
                `CSR_STVEC:     csr_stvec  = csr_write_val;
                `CSR_SCOUNTEREN:csr_scounteren = csr_write_val;
                `CSR_SSCRATCH:  csr_sscratch = csr_write_val;
                `CSR_SEPC:      csr_sepc   = csr_write_val & ~1;
                `CSR_SCAUSE:    csr_scause = csr_write_val;
                `CSR_STVAL:     csr_stval  = csr_write_val;
                `CSR_SIP:       begin
                   // Only SSIP (bit 1) is writable via SIP; SEIP/STIP are read-only
                   if (csr_mideleg[1]) ssip = csr_write_val[1];
                   if (csr_mideleg[0]) usip = csr_write_val[0];
                end
                `CSR_SATP: begin
                   if (prv == 1 && tvm) begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
                   end else
                     csr_satp = csr_write_val;
                end
                `CSR_MSTATUS: begin
                   {sie, uie}        = csr_write_val[1:0];
                   {spie, upie, mie} = csr_write_val[5:3];
                   {spp, mpie}       = csr_write_val[8:7];
                   mpp               = csr_write_val[12:11];
                   fs                = csr_write_val[14:13];
                   {tsr, tw, tvm, mxr, sum, mprv} = csr_write_val[22:17];
                end
                `CSR_MISA:     begin end
                `CSR_MEDELEG:  csr_medeleg  = csr_write_val;
                `CSR_MIDELEG:  csr_mideleg  = csr_write_val;
                `CSR_MIE:      csr_mie      = csr_write_val;
                `CSR_MTVEC:    csr_mtvec    = csr_write_val; // XXX enforce 256-byte alignment for vectored interrupts
                `CSR_MCOUNTEREN: begin end
                `CSR_MSCRATCH: csr_mscratch = csr_write_val;
                `CSR_MEPC:     csr_mepc     = csr_write_val & ~1;
                `CSR_MCAUSE:   csr_mcause   = csr_write_val;
                `CSR_MTVAL:    csr_mtval    = csr_write_val;
                `CSR_MIP:      begin
                   // MEIP/SEIP (bits 11,9) are read-only, driven by PLIC
                   // MTIP/MSIP (bits 7,3) are read-only, driven by CLINT
                   ueip = csr_write_val[8];
                   stip = csr_write_val[5];
                   utip = csr_write_val[4];
                   ssip = csr_write_val[1];
                   usip = csr_write_val[0];
                end
                `CSR_TSELECT:  begin end
                `CSR_TDATA1:   begin end
                `CSR_TDATA2:   begin end
                `CSR_TDATA3:   begin end
                `CSR_MCYCLE:   csr_mcycle   = csr_write_val;
                `CSR_MINSTRET: csr_minstret = csr_write_val;
                default: begin
                 csr_access_failure = 1;
`ifdef SIMULATE
`ifdef VERBOSE
                   $display("%05d   %1d %x %x illegal CSR %x (write)", $time, prv, pc, insn, csrno);
`endif
`endif
                end
              endcase
           end

           write_back_value = csr_read_val;
           if (csr_access_failure) begin
              cause = `TRAP_ILLEGAL_INSTRUCTION;
              tval = insn;

              state <= `S_EXCEPTION;
           end
        end

        `S_EXCEPTION: begin
`ifdef SIMULATE
`ifdef VERBOSE
           $display("%05d  ** Exception, cause %x, pc %x, tval %x, prv %d", $time, cause, pc, tval, prv);
           $fflush(0);
`endif
`endif

`ifdef VERILATOR_COSIM
           prv_at_trap = prv;  // capture before the mutation below
`endif

           write_back_register = 0;

           // XXX We would probably save gates by factoring the deleg
           // calculation out to where cause is set as it's unually a
           // constant.
           deleg = prv <= 1 && (cause_intr ? csr_mideleg[cause[3:0]] : csr_medeleg[cause[3:0]]);
           if (deleg) begin
              csr_scause = {cause_intr, 51'd0, cause};
              csr_sepc = pc;
              csr_stval = tval;
              spie = sie;
              sie = 0;
              spp = prv;
              tvec = csr_stvec;
              prv = 1;
           end else begin
              csr_mcause = {cause_intr, 51'd0, cause};
              csr_mepc = pc;
              csr_mtval = tval;
              mpie = mie;
              mie = 0;
              mpp = prv;
              tvec = csr_mtvec;
              prv = 3;
           end

           // Handle vectored interrupts, just to be compatible
           npc = (tvec[0] && cause_intr) ? (tvec & ~3) + cause[11:0] * 4 : tvec & ~3;
`ifdef SIMULATE
`ifdef VERBOSE
           $display("%05d  ** Exception resuming at %x", $time, (tvec[0] && cause_intr) ? (tvec & ~3) + cause[11:0] * 4 : tvec & ~3);
`endif
`endif

`ifdef VERILATOR_COSIM
           // Trap retire: pc/insn still hold the trapping instruction; npc is
           // the trap vector we just computed. prv_at_trap is pre-trap prv.
           cosim_retire(
               pc,
               npc,
               insn,
               8'd0,                         // no writeback on trap
               8'd0,
               {6'd0, prv_at_trap},
               8'd1,                         // trapped
               32'd0,
               64'd0,
               {cause_intr, 51'd0, cause},   // architectural mcause/scause form
               tval,
               clint_mtime
           );
           just_trapped <= 1;
`endif

           state <= `S_FETCH1;
        end

        `S_MUL_RUNNING: begin
           if (mul_b != 0) begin
              if (mul_b[0])
                muldiv_p = muldiv_p + mul_a;
              mul_a = mul_a << 1;
              mul_b = mul_b >> 1;
           end else begin
              if (muldiv_output_sext32)
                write_back_value = {{32{muldiv_p[31]}}, muldiv_p[31:0]};
              else begin
                 if (muldiv_output_negate)
                   muldiv_p = -muldiv_p;
                 else
                   muldiv_p = muldiv_p;

                 if (muldiv_output_high_part)
                   write_back_value = muldiv_p[127:64];
                 else
                   write_back_value = muldiv_p[63:0];
              end

              state <= `S_FETCH1;
           end
        end

        `S_DIV_RUNNING: begin
           if (div_count != 0) begin
              mul_b = mul_b << 1;
              if (muldiv_p >= mul_a) begin
                 muldiv_p = muldiv_p - mul_a;
                 mul_b = mul_b | 1;
              end
              mul_a = mul_a >> 1;
              div_count = div_count  - 1;
           end else begin
              write_back_value = muldiv_output_negate ? -mul_b : mul_b;
              if (muldiv_output_high_part)
                // REM
                write_back_value = muldiv_output_negate ? -muldiv_p[63:0] : muldiv_p[63:0];

              if (muldiv_output_sext32)
                write_back_value = {{32{write_back_value[31]}}, write_back_value[31:0]};

              state <= `S_FETCH1;
           end
        end

        `S_PTW_READ: begin
           // Sv39 page table walk: latch PTE from memory; process in S_PTW_PROCESS.
           // Registering here breaks the LUTRAM-read → dram_burst_addr timing path.
           if (ptw_from_dram)
              pte_latch <= dram_latched >> (ptw_pte_addr[4:3] * 64);
           else
              pte_latch <= ptw_pte_addr[3] ? {mem_data0, mem_data1} : {mem_data1, mem_data0};
           state <= `S_PTW_PROCESS;
        end

        `S_PTW_PROCESS: begin
           // Sv39 page table walk: process PTE latched in S_PTW_READ.
           // PTE is pte_latch[63:0]; use aligned as a local alias for readability.
           aligned = pte_latch;
           // PTE fields: V=[0] R=[1] W=[2] X=[3] U=[4] G=[5] A=[6] D=[7] PPN=[53:10]
`ifdef SIMULATE
`ifdef VERBOSE
           $display("PTW: va %x level %d pte_addr %x pte %x access %d prv %d time %0t",
                    ptw_va, ptw_level, ptw_pte_addr, aligned[63:0], ptw_access, ptw_prv, $time);
`endif
`endif
           ptw_fault_cause = ptw_access == 0 ? `TRAP_INSTRUCTIONPAGE_FAULT :
                             ptw_access == 1 ? `TRAP_LOAD_PAGE_FAULT :
                                               `TRAP_STORE_PAGE_FAULT;

           if (ptw_va[63:39] != {25{ptw_va[38]}}) begin
              // Non-canonical Sv39 virtual address
              cause = ptw_fault_cause;
              tval = ptw_va;
              state <= `S_EXCEPTION;
           end else if (!aligned[0] || !aligned[1] && aligned[2]) begin
              // Invalid PTE: V=0 or (R=0 && W=1)
              cause = ptw_fault_cause;
              tval = ptw_va;
              state <= `S_EXCEPTION;
           end else if (aligned[1] || aligned[3]) begin
              // Leaf PTE (R=1 or X=1)

              if (aligned[63] && (ptw_level != 0 || aligned[13:10] != 4'b1000)) begin
                 // NAPOT: N bit reserved on superpages; at level 0 only 64 KiB supported
                 cause = ptw_fault_cause;
                 tval = ptw_va;
                 state <= `S_EXCEPTION;
              end else if ((ptw_level == 2 && aligned[27:10] != 0) ||
                  (ptw_level == 1 && aligned[18:10] != 0)) begin
                 // Misaligned superpage
                 cause = ptw_fault_cause;
                 tval = ptw_va;
                 state <= `S_EXCEPTION;
              end else if (ptw_access == 0 && !aligned[3]) begin
                 // Fetch requires X
                 cause = ptw_fault_cause;
                 tval = ptw_va;
                 state <= `S_EXCEPTION;
              end else if ((ptw_access == 1 || ptw_access == 3) &&
                           !aligned[1] && !(mxr && aligned[3])) begin
                 // Load/AMO requires R (or X when MXR)
                 cause = ptw_fault_cause;
                 tval = ptw_va;
                 state <= `S_EXCEPTION;
              end else if ((ptw_access == 2 || ptw_access == 3) && !aligned[2]) begin
                 // Store/AMO requires W
                 cause = ptw_fault_cause;
                 tval = ptw_va;
                 state <= `S_EXCEPTION;
              end else if (ptw_prv == 0 && !aligned[4]) begin
                 // U-mode but page not marked U
                 cause = ptw_fault_cause;
                 tval = ptw_va;
                 state <= `S_EXCEPTION;
              end else if (ptw_prv == 1 && aligned[4] && (ptw_access == 0 || !sum)) begin
                 // S-mode accessing U page: forbidden for fetch, or load/store without SUM
                 cause = ptw_fault_cause;
                 tval = ptw_va;
                 state <= `S_EXCEPTION;
              end else if (!aligned[6] || (ptw_access >= 2 && !aligned[7])) begin
                 // Software A/D management (RVA22): fault if A=0 or D=0 for store/AMO
                 cause = ptw_fault_cause;
                 tval = ptw_va;
                 state <= `S_EXCEPTION;
              end else begin
                 // Translation successful - compute physical address
                 case (ptw_level)
                   2: mem_addr = {8'd0, aligned[53:28], ptw_va[29:0]}; // 1 GiB superpage
                   1: mem_addr = {8'd0, aligned[53:19], ptw_va[20:0]}; // 2 MiB superpage
                   0: mem_addr = aligned[63]
                        ? {8'd0, aligned[53:14], ptw_va[15:0]}  // 64 KiB NAPOT
                        : {8'd0, aligned[53:10], ptw_va[11:0]}; // 4 KiB page
                   default: mem_addr = 0;
                 endcase

                  translated <= 1;
                  if (ptw_return == `S_FETCH2 || ptw_return == `S_FETCH2_HALF) begin
                     if (mem_addr[31] && mem_addr[63:`MEM_SIZE_LG2] != `MEM_BASEADDR >> `MEM_SIZE_LG2) begin
                        // DRAM instruction fetch (above BRAM overlay)
                        fetch_from_dram <= 1;
                        dram_burst_addr  = mem_addr[30:5];
                        dram_read        = 1;
                        state           <= (ptw_return == `S_FETCH2) ?
                                           `S_DRAM_FETCH_WAIT : `S_DRAM_FETCH_HALF_WAIT;
                     end else begin
                        fetch_from_dram <= 0;
                        mem_addr0  <= mem_addr[`MEM_SIZE_LG2-1:4] + mem_addr[3];
                        mem_addr1  <= mem_addr[`MEM_SIZE_LG2-1:4];
                        // S_FETCH2 reads from mem_data0_q/mem_data1_q (registered in S_FETCH1B).
                        // Route through S_FETCH1B to capture the LUTRAM output first.
                        state      <= (ptw_return == `S_FETCH2) ? `S_FETCH1B : ptw_return;
                     end
                  end else begin
                     mem_addr0  <= mem_addr[`MEM_SIZE_LG2-1:4] + mem_addr[3];
                     mem_addr1  <= mem_addr[`MEM_SIZE_LG2-1:4];
                     state      <= ptw_return;
                  end
              end
           end else if (ptw_level == 0) begin
              // Non-leaf at level 0: invalid
              cause = ptw_fault_cause;
              tval = ptw_va;
              state <= `S_EXCEPTION;
           end else begin
              // Non-leaf PTE: descend to next level
              ptw_level = ptw_level - 1;
              case (ptw_level)
                1: ptw_pte_addr = {8'd0, aligned[53:10], 12'd0} + {52'd0, ptw_va[29:21], 3'd0};
                0: ptw_pte_addr = {8'd0, aligned[53:10], 12'd0} + {52'd0, ptw_va[20:12], 3'd0};
                default: ptw_pte_addr = 0;
              endcase
              if (ptw_pte_addr[31] && ptw_pte_addr[63:`MEM_SIZE_LG2] != `MEM_BASEADDR >> `MEM_SIZE_LG2) begin
                 ptw_from_dram <= 1;
                 dram_burst_addr = ptw_pte_addr[30:5];
                 dram_read       = 1;
                 state          <= `S_DRAM_PTW_WAIT;
              end else begin
                 ptw_from_dram <= 0;
                 mem_addr0 <= ptw_pte_addr[`MEM_SIZE_LG2-1:4] + ptw_pte_addr[3];
                 mem_addr1 <= ptw_pte_addr[`MEM_SIZE_LG2-1:4];
                 state     <= `S_PTW_READ;  // re-read next PTE level
              end
           end
        end

        `S_FETCH2_HALF: begin
           // Second half of cross-page or cross-burst instruction fetch
           translated <= 0;
           if (fetch_from_dram)
              // pc+2 is at byte 0 of the next burst (dram_latched was updated)
              aligned = dram_latched[127:0];
           else
              // pc+2 is page-aligned (0x...000), so bit 3 is 0
              aligned = {mem_data1, mem_data0};
           insn = {aligned[15:0], insn_half};
           rd = insn`insn_rd;
           {rs1,rs2} = {insn`insn_rs1, insn`insn_rs2}; // Always format 3 (32-bit)

           shamt = insn[25:20];

           write_back_register = 0;
           state <= `S_RF2;  // rs1/rs2 already decoded here; skip S_RF
        end

        `S_DRAM_FETCH_WAIT: begin
           if (dram_readdatavalid) begin
              dram_latched <= dram_readdata;
              state        <= `S_FETCH2;
           end else if (dram_write_ready) begin
              // Adapter is idle but data hasn't returned: our initial dram_read
              // pulse was missed because the adapter was processing a preceding
              // write (e.g. a DRAM stack push).  Re-assert the read; dram_burst_addr
              // still holds the fetch address set in S_FETCH1.
              dram_read = 1;
           end
        end

        `S_DRAM_FETCH_HALF_WAIT: if (dram_readdatavalid) begin
           dram_latched <= dram_readdata;
           state        <= `S_FETCH2_HALF;
        end

        `S_DRAM_PTW_WAIT: if (dram_readdatavalid) begin
           dram_latched <= dram_readdata;
           state        <= `S_PTW_READ;
        end

        `S_DRAM_LOAD_WAIT: if (dram_readdatavalid) begin
           if ({1'b0, mem_addr[4:0]} + (1 << (load_size_lg2 & 3)) > 32) begin
              // Access crosses burst boundary — need second read
              dram_latched    <= dram_readdata;
              dram_burst_addr  = mem_addr[30:5] + 1;
              dram_read        = 1;
              state           <= `S_DRAM_LOAD2_WAIT;
           end else begin
              // Single burst: extract value
              aligned = dram_readdata >> (mem_addr[4:0] * 8);
              case (load_size_lg2)
                0: write_back_value = aligned[7:0];
                1: write_back_value = aligned[15:0];
                2: write_back_value = aligned[31:0];
                3: write_back_value = aligned[63:0];
                4: write_back_value = {{56{aligned[7]}},  aligned[7:0]};
                5: write_back_value = {{48{aligned[15]}}, aligned[15:0]};
                6: write_back_value = {{32{aligned[31]}}, aligned[31:0]};
                default: write_back_value = 0;
              endcase
              state <= do_atomic ? `S_AMO : `S_FETCH1;
           end
        end

        `S_DRAM_LOAD2_WAIT: if (dram_readdatavalid) begin
           begin : dram_load2
              reg [511:0] combo;
              combo = {dram_readdata, dram_latched} >> (mem_addr[4:0] * 8);
              case (load_size_lg2)
                0: write_back_value = combo[7:0];
                1: write_back_value = combo[15:0];
                2: write_back_value = combo[31:0];
                3: write_back_value = combo[63:0];
                4: write_back_value = {{56{combo[7]}},  combo[7:0]};
                5: write_back_value = {{48{combo[15]}}, combo[15:0]};
                6: write_back_value = {{32{combo[31]}}, combo[31:0]};
                default: write_back_value = 0;
              endcase
           end
           state <= do_atomic ? `S_AMO : `S_FETCH1;
        end

        `S_DRAM_STORE_WAIT: if (dram_write_ready) begin
           // Issue the first write now that adapter is ready
           dram_write <= 1;
           state      <= dram_store_split ? `S_DRAM_STORE2 : `S_FETCH1;
        end

        `S_DRAM_STORE2: if (dram_write_ready) begin
           dram_burst_addr <= dram2_addr;
           dram_writedata  <= {192'd0, dram2_data_part};
           dram_byte_mask  <= dram2_mask;
           dram_write      <= 1;
           state           <= `S_FETCH1;
        end

      endcase

      // Bus timeout: fault if an external bus access doesn't respond
      begin : bus_timeout_logic
         reg bus_waiting;
         bus_waiting = state == `S_DRAM_FETCH_WAIT || state == `S_DRAM_FETCH_HALF_WAIT ||
                       state == `S_DRAM_LOAD_WAIT  || state == `S_DRAM_LOAD2_WAIT ||
                       state == `S_DRAM_PTW_WAIT   ||
                       state == `S_DRAM_STORE_WAIT || state == `S_DRAM_STORE2 ||
                       state == `S_MMIO_ALIGN;
         if (bus_waiting) begin
            bus_timeout_ctr <= bus_timeout_ctr + 1;
            if (&bus_timeout_ctr) begin
               cause_intr = 0;
               case (state)
                 `S_DRAM_FETCH_WAIT, `S_DRAM_FETCH_HALF_WAIT: begin
                    cause = `TRAP_INSTRUCTION_ACCESS_FAULT;
                    tval = pc;
                 end
                 `S_DRAM_STORE_WAIT, `S_DRAM_STORE2: begin
                    cause = `TRAP_STORE_ACCESS_FAULT;
                    tval = mem_addr;
                 end
                 `S_DRAM_PTW_WAIT: begin
                    // PTW timeout: fault depends on what triggered the walk
                    cause = ptw_access == 0 ? `TRAP_INSTRUCTION_ACCESS_FAULT :
                            ptw_access == 2 || ptw_access == 3 ? `TRAP_STORE_ACCESS_FAULT :
                            `TRAP_LOAD_ACCESS_FAULT;
                    tval = ptw_va;
                 end
                 default: begin // S_DRAM_LOAD_WAIT, S_DRAM_LOAD2_WAIT, S_MMIO_ALIGN
                    cause = `TRAP_LOAD_ACCESS_FAULT;
                    tval = mem_addr;
                 end
               endcase
`ifdef SIMULATE
`ifdef VERBOSE
               $display("%05d  ** Bus timeout in state %0d, cause %0d, tval %x", $time, state, cause, tval);
`endif
`endif
               write_back_register = 0;
               state <= `S_EXCEPTION;
            end
         end else
            bus_timeout_ctr <= 0;
      end

      if (reset) begin
         state <= `S_FETCH1;
         csr_minstret <= 0;
         csr_mcycle <= 0;
         clint_mtime <= 0;
         write_back_register <= 0;
         npc <= `RESET_PC;
         bus_timeout_ctr <= 0;
         // XXX and a lot more
      end
   end
endmodule



module regfile(input wire         clock,
               input wire         write_valid,
               input wire [ 4:0]  write_addr,
               input wire [63:0]  write_data,
               input wire [ 4:0]  read_addr_0,
               input wire [ 4:0]  read_addr_1,

`ifdef ASYNC_RF
               output wire [63:0] read_data_0,
               output wire [63:0] read_data_1
`else
               output reg  [63:0] read_data_0,
               output reg  [63:0] read_data_1
`endif
);

   (* ram_style = "block" *)
   // Since the memory currently is baked into SmolRV64 and we
   // don't have a device tree, we take the shortcut of
   // - embedding the frequency into register 7 of the UART
   // - initializing sp to the end of physical memory.
   // This is only true for now and will definitely change.
   reg  [63:0] regfile[31:0];
   reg [8*200:0] rf_path;
   initial begin
      if ($value$plusargs("rf=%s", rf_path))
         $readmemh(rf_path, regfile, 0, 31);
      else
         $readmemh("rf.hex", regfile, 0, 31);
   end

   always @(posedge clock) begin
`ifndef ASYNC_RF
      // Non-blocking: samples read_addr at the clock edge (before any blocking
      // assignments from other always blocks), ensuring deterministic simulation.
      read_data_0 <= regfile[read_addr_0];
      read_data_1 <= regfile[read_addr_1];
`endif

      if (write_valid) regfile[write_addr] <= write_data;
   end

`ifdef ASYNC_RF
   assign read_data_0 = regfile[read_addr_0];
   assign read_data_1 = regfile[read_addr_1];
`endif
endmodule
