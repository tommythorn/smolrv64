`timescale 1ns/10ps
`default_nettype none

`define insn_rd  [11: 7]
`define insn_rs1 [19:15]
`define insn_rs2 [24:20]
`define insn_csr [31:20]

`ifdef SIMULATE
//`define DISASS 1
`ifndef AXI_MEM_SIZE_LG2
`define AXI_MEM_SIZE_LG2 27
`endif
`define AXI_MEM_SIZE (1 << `AXI_MEM_SIZE_LG2)
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
   wire                 mmio_readdatavalid = 1'b0;
   wire [31:0]          mmio_readdata = 32'd0;

   reg                  reset_n = 0; always @(posedge clock) reset_n <= 1;

   wire       mmio_waitrequest; // Currently ignored
   wire       uart_tx_valid;
   wire [7:0] uart_tx_data;
   reg        uart_rx_valid_tb = 0;
   reg  [7:0] uart_rx_data_tb  = 0;
   wire [ 2:0] m_axi_awid;
   wire [30:0] m_axi_awaddr;
   wire [ 7:0] m_axi_awlen;
   wire [ 2:0] m_axi_awsize;
   wire [ 1:0] m_axi_awburst;
   wire        m_axi_awlock;
   wire [ 3:0] m_axi_awcache;
   wire [ 2:0] m_axi_awprot;
   wire [ 3:0] m_axi_awqos;
   wire        m_axi_awvalid;
   wire        m_axi_awready;
   wire [63:0] m_axi_wdata;
   wire [ 7:0] m_axi_wstrb;
   wire        m_axi_wlast;
   wire        m_axi_wvalid;
   wire        m_axi_wready;
   reg  [ 2:0] m_axi_bid = 0;
   reg  [ 1:0] m_axi_bresp = 0;
   reg         m_axi_bvalid = 0;
   wire        m_axi_bready;
   wire [ 2:0] m_axi_arid;
   wire [30:0] m_axi_araddr;
   wire [ 7:0] m_axi_arlen;
   wire [ 2:0] m_axi_arsize;
   wire [ 1:0] m_axi_arburst;
   wire        m_axi_arlock;
   wire [ 3:0] m_axi_arcache;
   wire [ 2:0] m_axi_arprot;
   wire [ 3:0] m_axi_arqos;
   wire        m_axi_arvalid;
   wire        m_axi_arready;
   reg  [ 2:0] m_axi_rid = 0;
   reg  [63:0] m_axi_rdata = 0;
   reg  [ 1:0] m_axi_rresp = 0;
   reg         m_axi_rlast = 1'b1;
   reg         m_axi_rvalid = 0;
   wire        m_axi_rready;

   // Simulation DDR model: same 16-byte-striped even/odd 64-bit banking as the
   // on-core SRAM arrays, so existing mem.even/mem.odd style images can be reused.
   reg [63:0] axi_mem0[`AXI_MEM_SIZE/16-1:0];
   reg [63:0] axi_mem1[`AXI_MEM_SIZE/16-1:0];
   reg [8*200:0] evenhex = 0, oddhex = 0, axi_evenhex = 0, axi_oddhex = 0;
   reg            axi_aw_seen = 0;
   reg            axi_w_seen = 0;
   reg [30:0]     axi_awaddr_q = 0;
   reg [63:0]     axi_wdata_q = 0;
   reg [ 7:0]     axi_wstrb_q = 0;
   reg            axi_b_pending = 0;
   reg [30:0]     axi_araddr_q = 0;
   reg            axi_r_pending = 0;
   integer        tb_i, tb_b;

   assign m_axi_awready = !axi_aw_seen && !axi_b_pending;
   assign m_axi_wready  = !axi_w_seen  && !axi_b_pending;
   assign m_axi_arready = !axi_r_pending && !m_axi_rvalid;

   function [63:0] axi_read64;
      input [30:0] addr;
      begin
         if (addr[`AXI_MEM_SIZE_LG2-1:4] >= `AXI_MEM_SIZE/16)
            axi_read64 = 64'd0;
         else if (addr[3])
            axi_read64 = axi_mem1[addr[`AXI_MEM_SIZE_LG2-1:4]];
         else
            axi_read64 = axi_mem0[addr[`AXI_MEM_SIZE_LG2-1:4]];
      end
   endfunction

   task axi_write64;
      input [30:0] addr;
      input [63:0] data;
      input [ 7:0] strb;
      reg   [63:0] word;
      begin
         if (addr[`AXI_MEM_SIZE_LG2-1:4] < `AXI_MEM_SIZE/16) begin
            word = axi_read64(addr);
            for (tb_b = 0; tb_b < 8; tb_b = tb_b + 1)
               if (strb[tb_b])
                  word[8*tb_b +: 8] = data[8*tb_b +: 8];
            if (addr[3])
               axi_mem1[addr[`AXI_MEM_SIZE_LG2-1:4]] = word;
            else
               axi_mem0[addr[`AXI_MEM_SIZE_LG2-1:4]] = word;
         end
      end
   endtask

   initial begin
      for (tb_i = 0; tb_i < `AXI_MEM_SIZE/16; tb_i = tb_i + 1) begin
         axi_mem0[tb_i] = 0;
         axi_mem1[tb_i] = 0;
      end

      if ($value$plusargs("even=%s", evenhex)) begin
         if (!$value$plusargs("odd=%s", oddhex)) begin
            $display("ERROR: please specify the +odd=<hexfile>");
            $finish;
         end
         $readmemh(evenhex, axi_mem0, 0, `AXI_MEM_SIZE/16-1);
         $readmemh(oddhex,  axi_mem1, 0, `AXI_MEM_SIZE/16-1);
      end else if ($value$plusargs("odd=%s", oddhex)) begin
         $display("ERROR: please specify the +even=<hexfile>");
         $finish;
      end else if ($value$plusargs("axi_even=%s", axi_evenhex)) begin
         if (!$value$plusargs("axi_odd=%s", axi_oddhex)) begin
            $display("ERROR: please specify the +axi_odd=<hexfile>");
            $finish;
         end
         $readmemh(axi_evenhex, axi_mem0, 0, `AXI_MEM_SIZE/16-1);
         $readmemh(axi_oddhex,  axi_mem1, 0, `AXI_MEM_SIZE/16-1);
      end else if ($value$plusargs("axi_odd=%s", axi_oddhex)) begin
         $display("ERROR: please specify the +axi_even=<hexfile>");
         $finish;
      end
   end

   always @(posedge clock) begin
      if (m_axi_awvalid && m_axi_awready) begin
         axi_aw_seen  <= 1;
         axi_awaddr_q <= m_axi_awaddr;
      end
      if (m_axi_wvalid && m_axi_wready) begin
         axi_w_seen  <= 1;
         axi_wdata_q <= m_axi_wdata;
         axi_wstrb_q <= m_axi_wstrb;
      end
      if (axi_aw_seen && axi_w_seen && !axi_b_pending) begin
         axi_write64(axi_awaddr_q, axi_wdata_q, axi_wstrb_q);
         axi_aw_seen  <= 0;
         axi_w_seen   <= 0;
         axi_b_pending <= 1;
      end
      if (axi_b_pending && !m_axi_bvalid) begin
         m_axi_bid    <= 3'b000;
         m_axi_bresp  <= 2'b00;
         m_axi_bvalid <= 1;
      end else if (m_axi_bvalid && m_axi_bready) begin
         m_axi_bvalid   <= 0;
         axi_b_pending  <= 0;
      end

      if (m_axi_arvalid && m_axi_arready) begin
         axi_araddr_q <= m_axi_araddr;
         axi_r_pending <= 1;
      end
      if (axi_r_pending && !m_axi_rvalid) begin
         m_axi_rid    <= 3'b000;
         m_axi_rdata  <= axi_read64(axi_araddr_q);
         m_axi_rresp  <= 2'b00;
         m_axi_rlast  <= 1'b1;
         m_axi_rvalid <= 1;
         axi_r_pending <= 0;
      end else if (m_axi_rvalid && m_axi_rready) begin
         m_axi_rvalid <= 0;
      end

      if (!reset_n) begin
         m_axi_bvalid  <= 0;
         m_axi_rvalid  <= 0;
         axi_aw_seen   <= 0;
         axi_w_seen    <= 0;
         axi_b_pending <= 0;
         axi_r_pending <= 0;
      end
   end

   smolrv64 smolrv64_inst(.clock                (clock),
                          .fpu_clock            (clock),
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

                          .m_axi_awid           (m_axi_awid),
                          .m_axi_awaddr         (m_axi_awaddr),
                          .m_axi_awlen          (m_axi_awlen),
                          .m_axi_awsize         (m_axi_awsize),
                          .m_axi_awburst        (m_axi_awburst),
                          .m_axi_awlock         (m_axi_awlock),
                          .m_axi_awcache        (m_axi_awcache),
                          .m_axi_awprot         (m_axi_awprot),
                          .m_axi_awqos          (m_axi_awqos),
                          .m_axi_awvalid        (m_axi_awvalid),
                          .m_axi_awready        (m_axi_awready),
                          .m_axi_wdata          (m_axi_wdata),
                          .m_axi_wstrb          (m_axi_wstrb),
                          .m_axi_wlast          (m_axi_wlast),
                          .m_axi_wvalid         (m_axi_wvalid),
                          .m_axi_wready         (m_axi_wready),
                          .m_axi_bid            (m_axi_bid),
                          .m_axi_bresp          (m_axi_bresp),
                          .m_axi_bvalid         (m_axi_bvalid),
                          .m_axi_bready         (m_axi_bready),
                          .m_axi_arid           (m_axi_arid),
                          .m_axi_araddr         (m_axi_araddr),
                          .m_axi_arlen          (m_axi_arlen),
                          .m_axi_arsize         (m_axi_arsize),
                          .m_axi_arburst        (m_axi_arburst),
                          .m_axi_arlock         (m_axi_arlock),
                          .m_axi_arcache        (m_axi_arcache),
                          .m_axi_arprot         (m_axi_arprot),
                          .m_axi_arqos          (m_axi_arqos),
                          .m_axi_arvalid        (m_axi_arvalid),
                          .m_axi_arready        (m_axi_arready),
                          .m_axi_rid            (m_axi_rid),
                          .m_axi_rdata          (m_axi_rdata),
                          .m_axi_rresp          (m_axi_rresp),
                          .m_axi_rlast          (m_axi_rlast),
                          .m_axi_rvalid         (m_axi_rvalid),
                          .m_axi_rready         (m_axi_rready),

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
         $fflush(1);
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
      #10000000
`ifdef RISCV_TESTS
      $display("Test Failed with TIMEOUT");
`endif
      $finish;
`endif
   end
endmodule
`endif

module smolrv64(input wire        clock,
                input wire        fpu_clock,
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

                // AXI4 master to DDR4 (0x80000000-0xFFFFFFFF, 64-bit data).
                // Fixed: arsize/awsize=8B, arlen/awlen=0 (single-beat).  At most one
                // read and one write in flight.
                output wire [ 2:0] m_axi_awid,
                output wire [30:0] m_axi_awaddr,
                output wire [ 7:0] m_axi_awlen,
                output wire [ 2:0] m_axi_awsize,
                output wire [ 1:0] m_axi_awburst,
                output wire        m_axi_awlock,
                output wire [ 3:0] m_axi_awcache,
                output wire [ 2:0] m_axi_awprot,
                output wire [ 3:0] m_axi_awqos,
                output wire        m_axi_awvalid,
                input  wire        m_axi_awready,
                output wire [63:0] m_axi_wdata,
                output wire [ 7:0] m_axi_wstrb,
                output wire        m_axi_wlast,
                output wire        m_axi_wvalid,
                input  wire        m_axi_wready,
                input  wire [ 2:0] m_axi_bid,
                input  wire [ 1:0] m_axi_bresp,
                input  wire        m_axi_bvalid,
                output wire        m_axi_bready,
                output wire [ 2:0] m_axi_arid,
                output wire [30:0] m_axi_araddr,
                output wire [ 7:0] m_axi_arlen,
                output wire [ 2:0] m_axi_arsize,
                output wire [ 1:0] m_axi_arburst,
                output wire        m_axi_arlock,
                output wire [ 3:0] m_axi_arcache,
                output wire [ 2:0] m_axi_arprot,
                output wire [ 3:0] m_axi_arqos,
                output wire        m_axi_arvalid,
                input  wire        m_axi_arready,
                input  wire [ 2:0] m_axi_rid,
                input  wire [63:0] m_axi_rdata,
                input  wire [ 1:0] m_axi_rresp,
                input  wire        m_axi_rlast,
                input  wire        m_axi_rvalid,
                output wire        m_axi_rready,

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
`define LOCAL_COUNTER_OVERFLOW_INTERRUPT        13

`define CSR_FFLAGS     12'h001
`define CSR_FRM        12'h002
`define CSR_FCSR       12'h003

`define CSR_SSTATUS    12'h100
`define CSR_SIE        12'h104
`define CSR_STVEC      12'h105
`define CSR_SCOUNTEREN 12'h106
`define CSR_SSCRATCH   12'h140
`define CSR_SEPC       12'h141
`define CSR_SCAUSE     12'h142
`define CSR_STVAL      12'h143
`define CSR_SIP        12'h144
`define CSR_SCOUNTOVF  12'hda0

`define CSR_SATP       12'h180

`define CSR_MSTATUS    12'h300
`define CSR_MISA       12'h301
`define CSR_MEDELEG    12'h302
`define CSR_MIDELEG    12'h303
`define CSR_MIE        12'h304
`define CSR_MTVEC      12'h305
`define CSR_MCOUNTEREN 12'h306
`define CSR_MCOUNTINHIBIT 12'h320
`define CSR_MCYCLECFG  12'h321
`define CSR_MINSTRETCFG 12'h322
`define CSR_MHPMEVENT3 12'h323

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
`define CSR_MHPMCOUNTER3 12'hb03
`define CSR_CYCLE      12'hc00
`define CSR_TIME       12'hc01
`define CSR_INSTRET    12'hc02
`define CSR_HPMCOUNTER3 12'hc03
`define CSR_MHARTID    12'hf14
`define CSR_MVENDORID  12'hf11
`define CSR_MARCHID    12'hf12
`define CSR_MIMPID     12'hf13

// Custom MRO CSRs: DDR4 transaction latency stats (cycles spent in S_DRAM_* waits)
`define CSR_MIG_MIN      12'hfc0
`define CSR_MIG_MAX      12'hfc1
`define CSR_MIG_TOTAL    12'hfc2
`define CSR_MIG_COUNT    12'hfc3
`define CSR_MIG_TIMEOUTS 12'hfc4
`define CSR_MIG_TO_PC    12'hfc5
`define CSR_MIG_TO_TVAL  12'hfc6
`define CSR_MIG_TO_STATE 12'hfc7
`define CSR_MIG_TO_CAUSE 12'hfc8
`define CSR_MIG_TO_ADDR  12'hfc9

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

`define S_PTW_LAUNCH    14  // launch PTW PTE fetch after ptw_* request fields are registered
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
`define S_FETCH1B              27  // register SRAM mem0/mem1 output before S_FETCH2 reads insn
`define S_LOAD_LATCH           28  // register SRAM mem0/mem1 output before S_LOAD_ALIGN reads data
`define S_DRAM_STORE_RESP_WAIT 29  // wait for an issued DRAM store to fully drain
`define S_DRAM_STORE_RESP_ARM  30  // absorb one cycle so AXI busy flags see a new write
`define S_STORE_COMMIT         31  // commit a store after translation/routing decision
`define S_STORE_BRAM_WRITE     32  // full-word writeback after BRAM store read/modify
`define S_CVFPU_ISSUE          33  // present a CVFPU operation until accepted
`define S_CVFPU_WAIT           34  // wait for a CVFPU result
`define S_CVFPU_FMA_RF2        35  // wait for rs3 FP regfile read
`define S_CVFPU_FMA_RF3        36  // issue CVFPU fused multiply-add/subtract
`define S_LAST_STATE           36  // update state register width accordingly

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

// pre_mem_op: memory access class pre-decoded in S_RF3, consumed in S_EXECUTE.
// Collapses the 22 per-insn load/store/AMO branches into one shared block
// (single mem_addr adder, single mem_addr0/mem_addr1 splitter).
`define MEMOP_NONE  3'd0
`define MEMOP_LOAD  3'd1  // L{B,H,W,D}{,U}, FLW/FLD, compressed integer/FP loads
`define MEMOP_STORE 3'd2  // S{B,H,W,D}, FSW/FSD, compressed integer/FP stores
`define MEMOP_LR    3'd3  // LR.W / LR.D
`define MEMOP_SC    3'd4  // SC.W / SC.D
`define MEMOP_AMO   3'd5  // AMO*.W / AMO*.D

`define HPM_COUNTERS 13
`define HPM_LAST     (3 + `HPM_COUNTERS - 1)
`define HPM_COUNTER_MASK ((64'h1 << (`HPM_COUNTERS + 3)) - 1)
`define HPM_INHIBIT_MASK (`HPM_COUNTER_MASK & ~64'h2)
`define HPM_OF_BIT 63

`define HPM_EVENT_NONE             16'h0000
`define HPM_EVENT_CYCLES           16'h0001
`define HPM_EVENT_INSTRUCTIONS     16'h0002
`define HPM_EVENT_CACHE_READ       16'h0100
`define HPM_EVENT_CACHE_HIT        16'h0101
`define HPM_EVENT_CACHE_MISS       16'h0102
`define HPM_EVENT_CACHE_FILL_LINE  16'h0103
`define HPM_EVENT_CACHE_FILL_BEAT  16'h0104
`define HPM_EVENT_CACHE_WRITE      16'h0105
`define HPM_EVENT_AXI_READ         16'h0200
`define HPM_EVENT_AXI_WRITE        16'h0201
`define HPM_EVENT_BUS_WAIT_CYCLE   16'h0202

`define REGION_UART    3'd0
`define REGION_CLINT   3'd1
`define REGION_PLIC    3'd2
`define REGION_BRAM    3'd3
`define REGION_MMIO    3'd4
`define REGION_DRAM    3'd5
`define REGION_ILLEGAL 3'd6

   reg [5:0]   state = `S_FETCH1; // XXX We should set this on reset
   reg         core_reset_pending = 0;
   wire        core_reset_home;
   wire        core_reset_now;

`ifndef MEM_BASEADDR
`define MEM_BASEADDR    64'h80000000  // override with -DMEM_BASEADDR=64'hXXXXXXXX
`endif
`ifndef MEM_SIZE_LG2
`define MEM_SIZE_LG2    15 // 32 KiB, override with -DMEM_SIZE_LG2=N
`endif
`define MEM_SIZE        (1 << `MEM_SIZE_LG2)
`ifndef CACHE_INDEX_BITS
`define CACHE_INDEX_BITS 14 // 1 MiB: 16k direct-mapped 64-byte lines
`endif
`define CACHE_LINES     (1 << `CACHE_INDEX_BITS)
`define CACHE_TAG_BITS  (64 - `CACHE_INDEX_BITS - 6)
`define CACHE_META_BITS (`CACHE_TAG_BITS + 1)
`ifndef RESET_PC
`define RESET_PC        `MEM_BASEADDR  // override with -DRESET_PC=64'hXXXXXXXX
`endif
`ifndef SRAM_EVENHEX
`define SRAM_EVENHEX    "mem.even"
`endif
`ifndef SRAM_ODDHEX
`define SRAM_ODDHEX     "mem.odd"
`endif

   // To enable penalty-free unaligned access, memory is split into
   // even and odd 64b word addresses and striped across them.  Any
   // 64-bit word at address A will then be found in
   // {mem1[A/16],mem0[A/16]} if A/8 is even and
   // {mem0[A/16+1],mem1[A/16]} if A/8 is odd.
   (* ram_style = "block" *) reg  [63:0] mem0[0:`MEM_SIZE/16-1];
   (* ram_style = "block" *) reg  [63:0] mem1[0:`MEM_SIZE/16-1];

`ifdef SIMULATE
   reg [8*200:0] sram_evenhex = 0, sram_oddhex = 0;
`ifdef RISCV_TESTS
   reg [63:0] tohost_phys;
`endif
`endif
   // Forward declaration for init
   reg [ 2:0]  plic_priority [0:63];
   integer i;
   initial begin
`ifdef SIMULATE
      if ($value$plusargs("sram_even=%s", sram_evenhex)) begin
         if (!$value$plusargs("sram_odd=%s", sram_oddhex)) begin
            $display("ERROR: please specify the +sram_odd=<hexfile>");
            $finish;
         end
      end else if ($value$plusargs("sram_odd=%s", sram_oddhex)) begin
         $display("ERROR: please specify the +sram_even=<hexfile>");
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
       if (sram_evenhex != 0)
          $readmemh(sram_evenhex, mem0, 0, `MEM_SIZE/16-1);
       if (sram_oddhex != 0)
          $readmemh(sram_oddhex, mem1, 0, `MEM_SIZE/16-1);
`ifdef DEBUG_SRAM_INIT
       $display("SRAM_INIT even=%0s odd=%0s mem0[0]=%016x mem1[0]=%016x",
                sram_evenhex, sram_oddhex, mem0[0], mem1[0]);
`endif
`else
      $readmemh(`SRAM_EVENHEX, mem0, 0, `MEM_SIZE/16-1);
      $readmemh(`SRAM_ODDHEX,  mem1, 0, `MEM_SIZE/16-1);
`endif
   end


   reg  [`MEM_SIZE_LG2-5:0] mem_addr0, mem_addr1;
   reg  [63:0] mem_addr;
   reg  [15:0] mem_wr_mask;
   reg  [63:0] mem_data0_q = 0;  // registered copy latched in S_FETCH1B; used by S_FETCH2
   reg  [63:0] mem_data1_q = 0;
   reg         fetch_latch_half = 0; // S_FETCH1B should continue to S_FETCH2_HALF
   reg  [127:0] bram_store_aligned = 0;
   reg  [ 15:0] bram_store_mask = 0;

   function [63:0] merge_store_bytes;
      input [63:0] old_word;
      input [63:0] new_word;
      input [ 7:0] byte_mask;
      integer byte_i;
      begin
         merge_store_bytes = old_word;
         for (byte_i = 0; byte_i < 8; byte_i = byte_i + 1)
            if (byte_mask[byte_i])
               merge_store_bytes[byte_i*8 +: 8] = new_word[byte_i*8 +: 8];
      end
   endfunction

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
   // Parallel FP writeback path. Unlike the int path, there is no x0-style
   // hardwire: f0 is a real register, so a separate _valid bit gates writes.
   reg         write_back_fp_valid = 0;
   reg  [ 4:0] write_back_fp_register = 0;
   reg  [63:0] write_back_fp_value;
   wire [63:0] f1_bram;     // FP regfile read port 0 (addressed by rs1)
   wire [63:0] f2_bram;     // FP regfile read port 1 (addressed by rs2)
   reg  [63:0] f1 = 0;      // flip-flop copy of f1_bram; captured in S_RF3
   reg  [63:0] f2 = 0;
   // Single-precision operand reads: if the f-reg isn't properly NaN-boxed,
   // the spec says single-precision ops see the canonical qNaN 0x7fc00000.
   wire [31:0] f1_s = (&f1[63:32]) ? f1[31:0] : 32'h7fc00000;
   wire [31:0] f2_s = (&f2[63:32]) ? f2[31:0] : 32'h7fc00000;
   reg  [63:0] exe_add   = 0;  // execute-stage intermediate (registered at S_EXECUTE→S_EXECUTE2)
   reg         exe_sext32 = 0; // 1 = sign-extend bit 31 of exe_add
   // Pre-decoded ALU control: computed in S_RF3, consumed in S_EXECUTE case block.
   // Breaks the ~50-condition priority if-else chain critical path into two pipeline stages.
   reg  [ 3:0] pre_exe_op  = 0;  // EXOP_* operation code
   reg  [63:0] pre_exe_b   = 0;  // second operand
   reg         pre_exe_sxt = 0;  // 1 → W-type: operate on [31:0], sign-extend result

   // Pre-decoded mem access: computed in S_RF3, consumed in S_EXECUTE shared block.
   // Collapses 22 load/store/AMO branches into one; shares a single s1+offset adder.
   reg  [ 2:0] pre_mem_op       = 0; // MEMOP_* class code (NONE/LOAD/STORE/LR/SC/AMO)
   reg  [63:0] pre_mem_offset   = 0; // byte offset added to s1 to form mem_addr
   reg  [ 2:0] pre_load_size_lg2= 0; // size/sign for loads+LR+AMO (matches load_size_lg2)
   reg  [ 7:0] pre_mem_wr_mask  = 0; // byte-enable for stores+SC
   reg  [ 4:0] pre_mem_wb_reg   = 0; // destination register for loads/LR/SC/AMO (0 for stores)
   reg         pre_mem_fp       = 0; // 1 = FP load/store (route via f-regfile, NaN-box FLW)
   reg  [ 2:0] load_size_lg2; // [1:0] = size (0:B, 1:H, 2:W, 3:D), [2] = sign-extend

   // FP load retiring: data came through write_back_value (integer path). NaN-box FLW (size=010).
   // FP bit-ops set pre_mem_fp=0; their result is already in write_back_fp_value.
   wire        fp_load_retiring   = write_back_fp_valid && pre_mem_fp;
   wire [63:0] fp_writeback_data  = fp_load_retiring
                                    ? (load_size_lg2[0] ? write_back_value
                                                        : {32'hffffffff, write_back_value[31:0]})
                                    : write_back_fp_value;

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

   fregfile f_rf_inst(.clock(clock),
                      .write_valid(state == `S_FETCH1 && write_back_fp_valid),
                      .write_data(fp_writeback_data),
                      .write_addr(write_back_fp_register),
                      .read_addr_0(rs1),
                      .read_addr_1(rs2),
                      .read_data_0(f1_bram),
                      .read_data_1(f2_bram));

`ifdef USE_CVFPU
   reg         cvfpu_in_valid = 0;
   reg  [2:0][63:0] cvfpu_operands = '0;
   reg  [ 2:0] cvfpu_rnd_mode = 0;
   reg  [ 2:0] pre_fp_rnd_mode = 0;
   reg         pre_fp_rmode_ok = 1'b1;
   reg  [ 3:0] cvfpu_op = 0;
   reg         cvfpu_op_mod = 0;
   reg  [ 2:0] cvfpu_src_fmt = 0;
   reg  [ 2:0] cvfpu_dst_fmt = 0;
   reg  [ 1:0] cvfpu_int_fmt = 0;
   reg  [ 7:0] cvfpu_tag_in = 0;
   reg         cvfpu_write_fp = 1'b1;
   wire        cvfpu_in_ready;
   wire [63:0] cvfpu_result;
   wire [ 4:0] cvfpu_fflags;
   wire [ 7:0] cvfpu_tag_out;
   wire        cvfpu_out_valid;
   wire        cvfpu_busy;

   smolrv64_cvfpu cvfpu_inst(
      .clock     ( clock ),
      .fpu_clock ( fpu_clock ),
      .reset     ( core_reset_now ),
      .in_valid  ( cvfpu_in_valid ),
      .in_ready  ( cvfpu_in_ready ),
      .operands  ( cvfpu_operands ),
      .rnd_mode  ( cvfpu_rnd_mode ),
      .op        ( cvfpu_op ),
      .op_mod    ( cvfpu_op_mod ),
      .src_fmt   ( cvfpu_src_fmt ),
      .dst_fmt   ( cvfpu_dst_fmt ),
      .int_fmt   ( cvfpu_int_fmt ),
      .tag_in    ( cvfpu_tag_in ),
      .result    ( cvfpu_result ),
      .fflags    ( cvfpu_fflags ),
      .tag_out   ( cvfpu_tag_out ),
      .out_valid ( cvfpu_out_valid ),
      .out_ready ( 1'b1 ),
      .flush     ( 1'b0 ),
      .busy      ( cvfpu_busy )
   );
`endif


   reg  [63:0] npc = `RESET_PC; // XXX We should set this on reset
   reg  [63:0] pre_npc = `RESET_PC;

   // CPU<->AXI master signalling (master block lives at the bottom of this module).
   // dram_addr is the 8B-aligned doubleword address (= phys[30:3]).
   reg  [27:0]  dram_addr;
   reg          dram_read = 0;
   reg          dram_write = 0;
   reg  [63:0]  dram_writedata;
   reg  [ 7:0]  dram_wstrb;          // AXI convention: 1 = write byte
   wire         dram_readdatavalid;
   wire [63:0]  dram_readdata;
   wire [63:0]  dram_readdata_next;
   wire         dram_readdata_next_valid;
   wire         dram_write_ready;    // master idle (no AW/W/B in flight)
   wire         dram_write_done;     // write response observed for issued write

   reg  [63:0]  dram_latched;        // holds first 8B chunk across states
   reg  [63:0]  dram_latched_next;   // holds ADDR+8 chunk for misaligned access
   reg          dram_latched_next_valid;
   reg          fetch_from_dram;     // set when current fetch came from DRAM
   reg          ptw_from_dram;       // set when current PTW PTE came from DRAM
   reg  [27:0]  dram2_addr;          // 8B-doubleword addr for 2nd half of split store
   reg  [63:0]  dram2_data_part;     // overflow bytes for split store
   reg  [ 7:0]  dram2_wstrb;         // AXI wstrb for split-store second beat
   reg          dram_store_split;    // 1 = second beat pending after DRAM_STORE_WAIT

   // Physical direct-mapped write-through cache for external DRAM reads.
   // The core-side granularity stays 64-bit; misses fill the surrounding
   // 64-byte line as eight 64-bit beats from the AXI backing path.
   localparam [2:0] CACHE_IDLE      = 3'd0;
   localparam [2:0] CACHE_TAG_READ  = 3'd1;
   localparam [2:0] CACHE_TAG_CHECK = 3'd2;
   localparam [2:0] CACHE_FILL_REQ  = 3'd3;
   localparam [2:0] CACHE_FILL_WAIT = 3'd4;
   localparam [2:0] CACHE_HIT_RESP  = 3'd5;

   (* ram_style = "block" *) reg [63:0] cache_bank0[0:`CACHE_LINES-1];
   (* ram_style = "block" *) reg [63:0] cache_bank1[0:`CACHE_LINES-1];
   (* ram_style = "block" *) reg [63:0] cache_bank2[0:`CACHE_LINES-1];
   (* ram_style = "block" *) reg [63:0] cache_bank3[0:`CACHE_LINES-1];
   (* ram_style = "block" *) reg [63:0] cache_bank4[0:`CACHE_LINES-1];
   (* ram_style = "block" *) reg [63:0] cache_bank5[0:`CACHE_LINES-1];
   (* ram_style = "block" *) reg [63:0] cache_bank6[0:`CACHE_LINES-1];
   (* ram_style = "block" *) reg [63:0] cache_bank7[0:`CACHE_LINES-1];
   // Duplicate tag RAM keeps the line-crossing next-word hit check synchronous
   // without asking Vivado to build a multi-read-port tag memory in LUTs.
   // Tag metadata is {valid, tag}; the RAMs power up zeroed, so all lines start
   // invalid. Soft reset waits for this cache to go idle and does not flush it.

   reg [ 2:0] cache_state = CACHE_IDLE;
   reg [63:0] cache_addr = 0;
   reg [63:0] cache_fill_base = 0;
   reg [ 2:0] cache_fill_beat = 0;
   reg [ 2:0] cache_req_bank = 0;
   reg [ 2:0] cache_req_next_bank = 0;
   reg        cache_req_same_line = 0;
   reg [`CACHE_TAG_BITS-1:0] cache_req_tag = 0;
   reg [`CACHE_TAG_BITS-1:0] cache_req_next_tag = 0;
   reg [63:0] cache_fill_return_data = 0;
   reg [63:0] cache_fill_next_data = 0;
   reg [63:0] cache_lookup_data = 0;
   reg [63:0] cache_lookup_next_data = 0;
   reg        cache_lookup_hit = 0;
   reg        cache_lookup_next_hit = 0;
   reg        cache_lookup_next_valid = 0;
   reg        dram_readdatavalid_r = 0;
   reg [63:0] dram_readdata_r = 0;
   reg [63:0] dram_readdata_next_r = 0;
   reg        dram_readdata_next_valid_r = 0;
   wire       cache_idle = cache_state == CACHE_IDLE;

   reg        axi_read = 0;
   reg [27:0] axi_read_addr = 0;
   wire       axi_readdatavalid;
   wire [63:0] axi_readdata;
   reg        axi_write = 0;
   reg [27:0] axi_write_addr = 0;
   reg [63:0] axi_write_data = 0;
   reg [ 7:0] axi_write_strb = 0;
   wire       axi_write_ready;
   wire       axi_write_done;

   reg        ar_busy = 0;
   reg        r_busy  = 0;
   reg [27:0] ar_addr_r;
   reg [63:0] rdata_r;
   reg        aw_busy = 0;
   reg        w_busy  = 0;
   reg        b_busy  = 0;
   reg [27:0] aw_addr_r;
   reg [63:0] w_data_r;
   reg [ 7:0] w_strb_r;
   reg        axi_readdatavalid_r = 0;

   wire       axi_master_idle = !ar_busy && !r_busy && !aw_busy && !w_busy && !b_busy;
   assign     core_reset_home = state == `S_FETCH1 && cache_state == CACHE_IDLE &&
                                !dram_read && !dram_write && !axi_read && !axi_write &&
                                axi_master_idle;
   assign     core_reset_now  = (reset || core_reset_pending) && core_reset_home;

   wire       hpm_instret_pulse = state == `S_FETCH1;
   wire       hpm_cache_read_pulse = cache_state == CACHE_IDLE && dram_read;
   wire       hpm_cache_write_pulse = cache_state == CACHE_IDLE && dram_write &&
                                      axi_write_ready && !axi_read;
   wire       hpm_cache_hit_pulse = cache_state == CACHE_HIT_RESP && cache_lookup_hit;
   wire       hpm_cache_miss_pulse = cache_state == CACHE_HIT_RESP && !cache_lookup_hit;
   wire       hpm_cache_fill_beat_pulse = cache_state == CACHE_FILL_WAIT && axi_readdatavalid;
   wire       hpm_cache_fill_line_pulse = hpm_cache_fill_beat_pulse && cache_fill_beat == 3'd7;
   wire       hpm_axi_read_pulse = axi_read;
   wire       hpm_axi_write_pulse = axi_write && axi_write_ready;
   wire       hpm_bus_wait_cycle = state == `S_DRAM_FETCH_WAIT || state == `S_DRAM_FETCH_HALF_WAIT ||
                                   state == `S_DRAM_LOAD_WAIT  || state == `S_DRAM_LOAD2_WAIT ||
                                   state == `S_DRAM_PTW_WAIT   ||
                                   state == `S_DRAM_STORE_WAIT || state == `S_DRAM_STORE2 ||
                                   state == `S_DRAM_STORE_RESP_WAIT || state == `S_DRAM_STORE_RESP_ARM ||
                                   state == `S_MMIO_ALIGN;

   wire [63:0] cache_dram_addr = {33'd0, dram_addr, 3'b000};
   wire [63:0] cache_dram_next_addr = cache_dram_addr + 64'd8;
   wire [`CACHE_INDEX_BITS-1:0] cache_dram_idx =
        cache_dram_addr[`CACHE_INDEX_BITS+5:6];
   wire [`CACHE_INDEX_BITS-1:0] cache_dram_next_idx =
        cache_dram_next_addr[`CACHE_INDEX_BITS+5:6];
   wire [`CACHE_TAG_BITS-1:0] cache_dram_tag =
        cache_dram_addr[63:`CACHE_INDEX_BITS+6];
   wire [`CACHE_TAG_BITS-1:0] cache_dram_next_tag =
        cache_dram_next_addr[63:`CACHE_INDEX_BITS+6];
   wire [`CACHE_INDEX_BITS-1:0] cache_fill_idx =
        cache_fill_base[`CACHE_INDEX_BITS+5:6];
   wire [`CACHE_TAG_BITS-1:0] cache_fill_tag =
        cache_fill_base[63:`CACHE_INDEX_BITS+6];

   reg [`CACHE_INDEX_BITS-1:0] cache_rd_idx = 0;
   reg [`CACHE_INDEX_BITS-1:0] cache_bank0_rd_idx = 0;
   reg [`CACHE_INDEX_BITS-1:0] cache_next_rd_idx = 0;
   wire [`CACHE_META_BITS-1:0] cache_tag_rd_data;
   wire [`CACHE_META_BITS-1:0] cache_tag_next_rd_data;
   reg                         cache_tag_wr_en = 0;
   reg  [`CACHE_INDEX_BITS-1:0] cache_tag_wr_idx = 0;
   reg  [`CACHE_META_BITS-1:0]  cache_tag_wr_data = 0;
   reg [63:0] cache_bank0_rd_data = 0;
   reg [63:0] cache_bank1_rd_data = 0;
   reg [63:0] cache_bank2_rd_data = 0;
   reg [63:0] cache_bank3_rd_data = 0;
   reg [63:0] cache_bank4_rd_data = 0;
   reg [63:0] cache_bank5_rd_data = 0;
   reg [63:0] cache_bank6_rd_data = 0;
   reg [63:0] cache_bank7_rd_data = 0;

   function [63:0] cache_selected_bank_data;
      input [2:0] bank;
      begin
         case (bank)
           3'd0: cache_selected_bank_data = cache_bank0_rd_data;
           3'd1: cache_selected_bank_data = cache_bank1_rd_data;
           3'd2: cache_selected_bank_data = cache_bank2_rd_data;
           3'd3: cache_selected_bank_data = cache_bank3_rd_data;
           3'd4: cache_selected_bank_data = cache_bank4_rd_data;
           3'd5: cache_selected_bank_data = cache_bank5_rd_data;
           3'd6: cache_selected_bank_data = cache_bank6_rd_data;
           3'd7: cache_selected_bank_data = cache_bank7_rd_data;
           default: cache_selected_bank_data = 64'd0;
         endcase
      end
   endfunction

   function hpm_event_active;
      input [15:0] event_code;
      input        instret_pulse;
      input        cache_read_pulse;
      input        cache_hit_pulse;
      input        cache_miss_pulse;
      input        cache_fill_line_pulse;
      input        cache_fill_beat_pulse;
      input        cache_write_pulse;
      input        axi_read_pulse;
      input        axi_write_pulse;
      input        bus_wait_cycle;
      begin
         case (event_code)
           `HPM_EVENT_CYCLES:          hpm_event_active = 1'b1;
           `HPM_EVENT_INSTRUCTIONS:    hpm_event_active = instret_pulse;
           `HPM_EVENT_CACHE_READ:      hpm_event_active = cache_read_pulse;
           `HPM_EVENT_CACHE_HIT:       hpm_event_active = cache_hit_pulse;
           `HPM_EVENT_CACHE_MISS:      hpm_event_active = cache_miss_pulse;
           `HPM_EVENT_CACHE_FILL_LINE: hpm_event_active = cache_fill_line_pulse;
           `HPM_EVENT_CACHE_FILL_BEAT: hpm_event_active = cache_fill_beat_pulse;
           `HPM_EVENT_CACHE_WRITE:     hpm_event_active = cache_write_pulse;
           `HPM_EVENT_AXI_READ:        hpm_event_active = axi_read_pulse;
           `HPM_EVENT_AXI_WRITE:       hpm_event_active = axi_write_pulse;
           `HPM_EVENT_BUS_WAIT_CYCLE:  hpm_event_active = bus_wait_cycle;
           default:                    hpm_event_active = 1'b0;
         endcase
      end
   endfunction

   smolrv64_sdpram #(
      .ADDR_WIDTH(`CACHE_INDEX_BITS),
      .DATA_WIDTH(`CACHE_META_BITS)
   ) cache_tag_ram (
      .clock   ( clock ),
      .rd_addr ( cache_rd_idx ),
      .rd_data ( cache_tag_rd_data ),
      .wr_en   ( cache_tag_wr_en ),
      .wr_addr ( cache_tag_wr_idx ),
      .wr_data ( cache_tag_wr_data )
   );

   smolrv64_sdpram #(
      .ADDR_WIDTH(`CACHE_INDEX_BITS),
      .DATA_WIDTH(`CACHE_META_BITS)
   ) cache_tag_next_ram (
      .clock   ( clock ),
      .rd_addr ( cache_next_rd_idx ),
      .rd_data ( cache_tag_next_rd_data ),
      .wr_en   ( cache_tag_wr_en ),
      .wr_addr ( cache_tag_wr_idx ),
      .wr_data ( cache_tag_wr_data )
   );

`ifndef BUS_TIMEOUT_LG2
`define BUS_TIMEOUT_LG2 24  // ~16M cycles before access fault
`endif
   reg [`BUS_TIMEOUT_LG2-1:0] bus_timeout_ctr = 0;
   // Registered one-cycle-ahead terminal-count, so the cause/state
   // combinational cone sees a FF output instead of a deep AND tree.
   // Fires one cycle after bus_timeout_ctr saturates — negligible at
   // 16M-cycle threshold, but critical for timing closure.
   reg        bus_timeout_expired = 0;
   reg [63:0] bus_timeout_tval = 0;
   reg [11:0] bus_timeout_cause = 0;

   // DDR4 transaction latency stats (cycles spent in S_DRAM_* wait states)
   reg [31:0] mig_latency_ctr = 0;
   reg        mig_prev_waiting = 0;
   reg [31:0] csr_mig_min    = 32'hFFFFFFFF;
   reg [31:0] csr_mig_max    = 0;
   reg [63:0] csr_mig_total  = 0;
   reg [63:0] csr_mig_count  = 0;
   reg [63:0] csr_mig_timeouts = 0;
   // First-timeout context latches (captured once per measurement window)
   reg [63:0] csr_mig_to_pc    = 0;
   reg [63:0] csr_mig_to_tval  = 0;
   reg [63:0] csr_mig_to_state = 0;
   reg [63:0] csr_mig_to_cause = 0;
   reg [63:0] csr_mig_to_addr  = 0;
   reg  [127:0] aligned;
   reg  [127:0] pte_latch = 0;    // registered copy of PTE data; set in S_PTW_READ, used in S_PTW_PROCESS
   reg  [63:0] imm_i, imm_j, imm_b, imm_u, imm_s, csr_arg, csr_read_val, csr_write_val, csr_satp_write_val;
   reg  [63:0] c_imm12_8_109_6_7_2_11_53_x2;
   reg  [63:0] c_imm12_65_2_1110_43_x2;
   reg  [ 9:0] c_nzuimm107_1211_5_6_x4;
   reg  [63:0] c_imm12_62;
   reg  [63:0] c_imm12_43_5_2_6_x16;
   reg  [ 6:0] c_uimm5_1210_6_x4;
   reg  [ 8:0] c_uimm42_12_65_x8, c_uimm97_1210_x8;
   reg  [ 7:0] c_uimm32_12_64_x4, c_uimm87_129_x4, c_uimm65_1210_x8;
   reg  [31:0] sext32;
`ifdef SIMULATE
   reg  [127:0] tmp128;
`endif
   reg  [ 4:0] rd;
   reg  [ 5:0] shamt;
   (* max_fanout = 16 *) reg [11:0] csrno;
   reg  [31:0] insn = 0; // XXX We should set this on reset
   reg  [ 1:0] csr_op;
   reg  [ 1:0] fcmp_result; // {nv, result} from fcmp_s/fcmp_d

   // CSR state
   reg         deleg, cause_intr;
   reg [63:0]  tval,
               tvec;
   reg [13:0]  csr_mie        = 0, // XXX We should set this on reset
               csr_mideleg    = 0,
               // temporary, will not turn into flop
               cause;
   reg [63:0]  csr_stvec      = 0,
               csr_scounteren = 0,
               csr_mcounteren = 0,
               csr_mcountinhibit = 0,
               csr_mcyclecfg  = 0,
               csr_minstretcfg = 0,
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
   reg [63:0]  csr_mhpmcounter[0:`HPM_COUNTERS-1];
   reg [63:0]  csr_mhpmevent[0:`HPM_COUNTERS-1];
   reg [63:0]  csr_scountovf_read_val = 0;
   integer     hpm_i, hpm_j;

   initial begin
      for (hpm_i = 0; hpm_i < `HPM_COUNTERS; hpm_i = hpm_i + 1) begin
         csr_mhpmcounter[hpm_i] = 0;
         csr_mhpmevent[hpm_i] = 0;
      end
   end

   always @(*) begin
      csr_scountovf_read_val = 0;
      for (hpm_j = 0; hpm_j < `HPM_COUNTERS; hpm_j = hpm_j + 1)
         csr_scountovf_read_val[hpm_j + 3] = csr_mhpmevent[hpm_j][`HPM_OF_BIT];
   end

   function counter_access_allowed;
      input [4:0] counter_idx;
      begin
         if (prv == 3)
           counter_access_allowed = 1;
         else if (prv == 1)
           counter_access_allowed = csr_mcounteren[counter_idx];
         else
           counter_access_allowed = csr_mcounteren[counter_idx] && csr_scounteren[counter_idx];
      end
   endfunction

   function hpm_mode_enabled;
      input [63:0] event_sel;
      begin
         case (prv)
           3: hpm_mode_enabled = !event_sel[62]; // MINH
           1: hpm_mode_enabled = !event_sel[61]; // SINH
           default: hpm_mode_enabled = !event_sel[60]; // UINH
         endcase
      end
   endfunction

   // fcsr: fflags[4:0] (NV|DZ|OF|UF|NX) + frm[2:0]. Phase 1 has no arithmetic
   // producers of fflags, so it stays at whatever software wrote.
   reg [ 4:0]  fflags = 0;
   reg [ 2:0]  frm = 0;

   // CLINT
   reg [63:0]  clint_mtime = 0;
   reg [13:0]  clint_mtime_clock_scaler = 0;
`ifdef VERILATOR_COSIM
   // One-cycle-delayed snapshot of clint_mtime. CSR TIME reads latch a value
   // that retires the cycle after; pass the delayed snapshot to simmerv so
   // its CSR TIME read returns the same value.
   reg [63:0]  clint_mtime_prev = 0;
   always @(posedge clock) clint_mtime_prev <= clint_mtime;
`endif
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
`ifdef PC_TRACE
   // Debug PC tracer (see tracer block below). When armed, SW sees a dummy
   // always-ready UART; the tracer owns the physical TX exclusively.
   reg        dbg_armed   = 0;
   reg [ 7:0] dbg_s_count = 0;
   reg        dbg_busy    = 0;
   reg [63:0] dbg_pc      = 0;
   reg [ 1:0] dbg_mode    = 0;
   reg        dbg_is_trap = 0;
   reg [ 5:0] dbg_cause   = 0;
   reg [ 5:0] dbg_pos     = 0;
   localparam integer DBG_N_RETIRE = 64;
   function [7:0] dbg_hex;
      input [3:0] n;
      dbg_hex = n < 10 ? 8'h30 + {4'd0, n} : 8'h57 + {4'd0, n};
   endfunction
   function [7:0] dbg_byte_at;
      input [ 5:0] pos;
      input [63:0] p;
      input [ 1:0] m;
      input        trap;
      input [ 5:0] cs;
      begin
         case (pos)
           6'd0:  dbg_byte_at = m == 2'd3 ? "M" : m == 2'd1 ? "S" : "U";
           6'd1:  dbg_byte_at = " ";
           6'd2:  dbg_byte_at = dbg_hex(p[63:60]);
           6'd3:  dbg_byte_at = dbg_hex(p[59:56]);
           6'd4:  dbg_byte_at = dbg_hex(p[55:52]);
           6'd5:  dbg_byte_at = dbg_hex(p[51:48]);
           6'd6:  dbg_byte_at = dbg_hex(p[47:44]);
           6'd7:  dbg_byte_at = dbg_hex(p[43:40]);
           6'd8:  dbg_byte_at = dbg_hex(p[39:36]);
           6'd9:  dbg_byte_at = dbg_hex(p[35:32]);
           6'd10: dbg_byte_at = dbg_hex(p[31:28]);
           6'd11: dbg_byte_at = dbg_hex(p[27:24]);
           6'd12: dbg_byte_at = dbg_hex(p[23:20]);
           6'd13: dbg_byte_at = dbg_hex(p[19:16]);
           6'd14: dbg_byte_at = dbg_hex(p[15:12]);
           6'd15: dbg_byte_at = dbg_hex(p[11: 8]);
           6'd16: dbg_byte_at = dbg_hex(p[ 7: 4]);
           6'd17: dbg_byte_at = dbg_hex(p[ 3: 0]);
           6'd18: dbg_byte_at = trap ? " " : "\n";
           6'd19: dbg_byte_at = "c";
           6'd20: dbg_byte_at = "=";
           6'd21: dbg_byte_at = dbg_hex({2'd0, cs[5:4]});
           6'd22: dbg_byte_at = dbg_hex(cs[3:0]);
           6'd23: dbg_byte_at = "\n";
           default: dbg_byte_at = "?";
         endcase
      end
   endfunction
   wire [7:0]  uart_lsr = dbg_armed
        ? {1'b0, 1'b1,           1'b1,           4'b0, !uart_rx_empty}
        : {1'b0, uart_tx_ready,  uart_tx_ready,  4'b0, !uart_rx_empty};
`else
   wire [7:0]  uart_lsr = {1'b0, uart_tx_ready, uart_tx_ready, 4'b0, !uart_rx_empty}; // TEMT|THRE + DR
`endif
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
               lcofip = 0,
               stip = 0, utip = 0,
               ssip = 0, usip = 0;
   wire        meip = plic_has_irq;
   wire        seip = plic_has_irq;
   reg         mtip = 0;
   always @(posedge clock) mtip <= clint_mtime >= clint_mtimecmp;
   wire        msip = clint_msip;

   wire [13:0] csr_mip = {lcofip, 1'd0, meip, 1'd0, seip, ueip,
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
   // Registered LR/SC reservation hit, computed at S_RF3→S_EXECUTE edge
   // (reservation == s1_bram) so S_EXECUTE's SC branch doesn't have to do
   // the 64-bit compare in-flight with the mem_addr next-D mux.
   reg         reservation_match = 0;
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

   // Marks the S_FETCH1 cycle immediately after S_EXCEPTION. Used to
   // suppress a stale pre_intr_pending (sampled before S_EXCEPTION's
   // mie:=0 landed). Also used by VERILATOR_COSIM to skip the dummy
   // post-trap retire. Set in S_EXCEPTION, cleared in S_FETCH1.
   reg        just_trapped = 0;

   // Marks the S_FETCH1 cycle(s) of the instruction immediately after an
   // xRET. xRET can re-enable interrupts (via mie/sie <- mpie/spie) and
   // pre_intr_pending may catch that change in time to fire on the very
   // next S_FETCH1 — causing the xRET target to never retire. Spec allows
   // either behavior, but for cosim we pick "let the xRET target retire
   // first" uniformly, regardless of whether the interrupt was pending
   // before xRET or became pending after. Set in MRET/SRET, cleared in
   // S_FETCH1.
   reg        just_xret = 0;

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
       input longint unsigned mtime,
       input longint unsigned mtimecmp,
       input longint unsigned mepc,
       input byte     unsigned seip
   );
`endif
   // Pre-retire / pre-trap privilege snapshots. Also used by PC_TRACE.
   reg [1:0]  prv_at_trap  = 0;  // pre-trap privilege, captured in S_EXCEPTION
   reg [1:0]  prv_retire   = 0;  // prv at instruction start (MRET/SRET change prv mid-execute)

   // IEEE 754 FCLASS: 10-bit one-hot classification (bit 0 = -inf, ..., bit 9 = qNaN).
   // Single-precision variant enforces NaN-boxing: unboxed value → canonical qNaN.
   function [63:0] fclass_d;
      input [63:0] v;
      reg         sign;
      reg [10:0]  exp;
      reg [51:0]  mant;
      reg         exp_all1, exp_0, mant_0, qbit;
      begin
         sign = v[63]; exp = v[62:52]; mant = v[51:0];
         exp_all1 = &exp;  exp_0 = exp == 0;  mant_0 = mant == 0;  qbit = mant[51];
         if      (exp_all1 && mant_0)   fclass_d = sign ? 64'h001 : 64'h080; // ±inf
         else if (exp_all1)             fclass_d = qbit ? 64'h200 : 64'h100; // qNaN / sNaN
         else if (exp_0 && mant_0)      fclass_d = sign ? 64'h008 : 64'h010; // ±0
         else if (exp_0)                fclass_d = sign ? 64'h004 : 64'h020; // ±subnormal
         else                           fclass_d = sign ? 64'h002 : 64'h040; // ±normal
      end
   endfunction

   function [63:0] fclass_s;
      input [63:0] v;
      reg         sign;
      reg [ 7:0]  exp;
      reg [22:0]  mant;
      reg         exp_all1, exp_0, mant_0, qbit;
      begin
         if (~(&v[63:32])) fclass_s = 64'h200; // improperly NaN-boxed → canonical qNaN
         else begin
            sign = v[31]; exp = v[30:23]; mant = v[22:0];
            exp_all1 = &exp;  exp_0 = exp == 0;  mant_0 = mant == 0;  qbit = mant[22];
            if      (exp_all1 && mant_0)   fclass_s = sign ? 64'h001 : 64'h080;
            else if (exp_all1)             fclass_s = qbit ? 64'h200 : 64'h100;
            else if (exp_0 && mant_0)      fclass_s = sign ? 64'h008 : 64'h010;
            else if (exp_0)                fclass_s = sign ? 64'h004 : 64'h020;
            else                           fclass_s = sign ? 64'h002 : 64'h040;
         end
      end
   endfunction

   // FP compares: returns {nv, result} where nv→fflags.NV, result→integer rd bit 0.
   // op (from insn[14:12]): 000=FLE, 001=FLT, 010=FEQ.
   // FEQ sets NV only on signaling NaN; FLT/FLE set NV on any NaN.
   function [1:0] fcmp_s;
      input [2:0] op;
      input [31:0] a, b;
      reg a_nan, b_nan, a_snan, b_snan, both_zero, eq, lt, le;
      begin
         a_nan = (a[30:23] == 8'hff) && (a[22:0] != 0);
         b_nan = (b[30:23] == 8'hff) && (b[22:0] != 0);
         a_snan = a_nan && !a[22];
         b_snan = b_nan && !b[22];
         both_zero = (a[30:0] == 0) && (b[30:0] == 0);
         if (a_nan || b_nan)        begin eq = 0; lt = 0; le = 0; end
         else if (both_zero)        begin eq = 1; lt = 0; le = 1; end
         else if (a[31] != b[31])   begin eq = 0; lt = a[31]; le = a[31]; end
         else if (!a[31])           begin eq = (a == b); lt = (a <  b); le = (a <= b); end
         else                       begin eq = (a == b); lt = (a >  b); le = (a >= b); end
         case (op)
            3'b000:  fcmp_s = {a_nan  | b_nan,  le};
            3'b001:  fcmp_s = {a_nan  | b_nan,  lt};
            3'b010:  fcmp_s = {a_snan | b_snan, eq};
            default: fcmp_s = 2'b00;
         endcase
      end
   endfunction

   function [1:0] fcmp_d;
      input [2:0] op;
      input [63:0] a, b;
      reg a_nan, b_nan, a_snan, b_snan, both_zero, eq, lt, le;
      begin
         a_nan = (a[62:52] == 11'h7ff) && (a[51:0] != 0);
         b_nan = (b[62:52] == 11'h7ff) && (b[51:0] != 0);
         a_snan = a_nan && !a[51];
         b_snan = b_nan && !b[51];
         both_zero = (a[62:0] == 0) && (b[62:0] == 0);
         if (a_nan || b_nan)        begin eq = 0; lt = 0; le = 0; end
         else if (both_zero)        begin eq = 1; lt = 0; le = 1; end
         else if (a[63] != b[63])   begin eq = 0; lt = a[63]; le = a[63]; end
         else if (!a[63])           begin eq = (a == b); lt = (a <  b); le = (a <= b); end
         else                       begin eq = (a == b); lt = (a >  b); le = (a >= b); end
         case (op)
            3'b000:  fcmp_d = {a_nan  | b_nan,  le};
            3'b001:  fcmp_d = {a_nan  | b_nan,  lt};
            3'b010:  fcmp_d = {a_snan | b_snan, eq};
            default: fcmp_d = 2'b00;
         endcase
      end
   endfunction

   function [2:0] phys_region;
      input [63:0] addr;
      begin
         if ((addr & 64'hffff_ffff_ffff_fff0) == 64'h0000_0000_1000_0000)
            phys_region = `REGION_UART;
         else if ((addr & 64'hffff_ffff_ffff_0000) == 64'h0000_0000_0200_0000)
            phys_region = `REGION_CLINT;
         else if ((addr & 64'hffff_ffff_ff00_0000) == 64'h0000_0000_0c00_0000)
            phys_region = `REGION_PLIC;
         else if (((addr ^ `MEM_BASEADDR) & (64'hffff_ffff_ffff_ffff << `MEM_SIZE_LG2)) == 0)
            phys_region = `REGION_BRAM;
         else if (addr[63:31] == 0)
            phys_region = `REGION_MMIO;
         else if (addr[63:31] == 1)
            phys_region = `REGION_DRAM;
         else
            phys_region = `REGION_ILLEGAL;
      end
   endfunction

   task start_ptw;
      input [63:0] req_va;
      input [ 1:0] req_access;
      input [ 1:0] req_prv;
      input [ 4:0] req_return;
      begin
         ptw_va       = req_va;
         ptw_level    = 2;
         ptw_access   = req_access;
         ptw_prv      = req_prv;
         ptw_return   = req_return;
         ptw_pte_addr <= {8'd0, csr_satp[43:0], 12'd0} + {52'd0, req_va[38:30], 3'd0};
         state        <= `S_PTW_LAUNCH;
      end
   endtask

   always @(posedge clock) begin
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */
      if (!csr_mcountinhibit[0] && hpm_mode_enabled(csr_mcyclecfg))
         csr_mcycle <= csr_mcycle + 1;
      for (hpm_i = 0; hpm_i < `HPM_COUNTERS; hpm_i = hpm_i + 1) begin
         if (!csr_mcountinhibit[hpm_i + 3] &&
             hpm_mode_enabled(csr_mhpmevent[hpm_i]) &&
             hpm_event_active(csr_mhpmevent[hpm_i][15:0],
                              hpm_instret_pulse,
                              hpm_cache_read_pulse,
                              hpm_cache_hit_pulse,
                              hpm_cache_miss_pulse,
                              hpm_cache_fill_line_pulse,
                              hpm_cache_fill_beat_pulse,
                              hpm_cache_write_pulse,
                              hpm_axi_read_pulse,
                              hpm_axi_write_pulse,
                              hpm_bus_wait_cycle)) begin
            if (csr_mhpmcounter[hpm_i] == 64'hffff_ffff_ffff_ffff &&
                !csr_mhpmevent[hpm_i][`HPM_OF_BIT]) begin
               csr_mhpmevent[hpm_i] <= csr_mhpmevent[hpm_i] | 64'h8000_0000_0000_0000;
               lcofip <= 1;
            end
            csr_mhpmcounter[hpm_i] <= csr_mhpmcounter[hpm_i] + 1;
         end
      end
      if (reset)
         core_reset_pending <= 1;
      // XXX This isn't very portable
      if (clint_mtime_clock_scaler[13]) begin
         clint_mtime_clock_scaler <= 3333 - 2; // 333.3333.. MHz / 3333 ~ 100.01 kHz
         clint_mtime <= clint_mtime + 1;
      end else
        clint_mtime_clock_scaler <= clint_mtime_clock_scaler - 1;
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
      dram_read  <= 0;
      dram_write <= 0;

      // Pre-register interrupt pending for S_FETCH1 timing closure.
      // Computed from current FFs so the result is available as a stable FF in
      // the NEXT cycle (adding ≤1 cycle of interrupt detection latency, which
      // is architecturally legal).
      begin : pre_intr_precompute
         reg [13:0] pi_pm, pi_ps, pi_raw;
         pi_pm = csr_mip & csr_mie & ~csr_mideleg;
         pi_ps = csr_mip & csr_mie &  csr_mideleg;
         if ((prv == 3 ? mie : 1'b1) && pi_pm != 0)
            pi_raw = pi_pm;
         else if (((prv == 1) ? sie : (prv == 0)) && pi_ps != 0)
            pi_raw = pi_ps;
         else
            pi_raw = 0;
         pre_intr_pending <= pi_raw != 0;
         pre_intr_cause   <= pi_raw[`LOCAL_COUNTER_OVERFLOW_INTERRUPT] ? `LOCAL_COUNTER_OVERFLOW_INTERRUPT :
                             pi_raw[`MACHINE_EXTERNAL_INTERRUPT] ? `MACHINE_EXTERNAL_INTERRUPT :
                             pi_raw[`MACHINE_SOFTWARE_INTERRUPT] ? `MACHINE_SOFTWARE_INTERRUPT :
                             pi_raw[`MACHINE_TIMER_INTERRUPT]    ? `MACHINE_TIMER_INTERRUPT :
                             pi_raw[`SUPERVISOR_EXTERNAL_INTERRUPT] ? `SUPERVISOR_EXTERNAL_INTERRUPT :
                             pi_raw[`SUPERVISOR_SOFTWARE_INTERRUPT] ? `SUPERVISOR_SOFTWARE_INTERRUPT :
                             pi_raw[`SUPERVISOR_TIMER_INTERRUPT] ? `SUPERVISOR_TIMER_INTERRUPT :
                             pi_raw[`USER_EXTERNAL_INTERRUPT]    ? `USER_EXTERNAL_INTERRUPT :
                             pi_raw[`USER_SOFTWARE_INTERRUPT]    ? `USER_SOFTWARE_INTERRUPT :
                                                                   `USER_TIMER_INTERRUPT;
      end

`ifdef PC_TRACE
      if (dbg_busy) begin
         // Gate on !uart_tx_valid so we don't re-assert valid before the UART
         // has latched the previous byte (rs232tx needs ready & valid to
         // coincide for exactly one cycle; ready stays high until it sees valid).
         if (uart_tx_ready && !uart_tx_valid) begin
            uart_tx_valid <= 1;
            uart_tx_data  <= dbg_byte_at(dbg_pos, dbg_pc, dbg_mode, dbg_is_trap, dbg_cause);
            dbg_pos       <= dbg_pos + 1;
            if (dbg_pos == (dbg_is_trap ? 6'd23 : 6'd18))
               dbg_busy <= 0;
         end
      end else
`endif
      case (state)
        `S_FETCH1: begin
           if (!csr_mcountinhibit[2] && hpm_mode_enabled(csr_minstretcfg))
              csr_minstret <= csr_minstret + 1;

           // Reset to default values
           muldiv_p = 0;
           muldiv_output_negate = 0;
           muldiv_output_high_part = 0;
           muldiv_output_sext32 = 0;
           do_atomic = 0;
`ifdef USE_CVFPU
           cvfpu_in_valid <= 0;
           cvfpu_write_fp <= 1'b1;
`endif

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
                  // rd_kind: 2=fp (takes priority — FP writes never coexist with int),
                  //          1=int (nonzero write_back_register), 0=none.
                  write_back_fp_valid        ? 8'd2 :
                  (write_back_register != 0) ? 8'd1 : 8'd0,
                  write_back_fp_valid        ? {3'd0, write_back_fp_register}
                                             : {3'd0, write_back_register},
                  {6'd0, prv_retire},
                  8'd0,
                  {27'd0, fflags},
                  write_back_fp_valid ? write_back_fp_value : write_back_value,
                  64'd0,
                  64'd0,
                  clint_mtime_prev,
                  clint_mtimecmp,
                  csr_mepc,
                  {7'd0, seip}
              );
           end
`endif
`ifdef PC_TRACE
           if (csr_mcycle != 0 && !just_trapped) begin
              // Arm on first M→S transition (OpenSBI's mret into Linux).
              if (!dbg_armed && prv_retire == 2'd3 && prv != 2'd3) begin
                 dbg_armed <= 1;
              end else if (dbg_armed && dbg_s_count < DBG_N_RETIRE[7:0] && !dbg_busy) begin
                 dbg_busy    <= 1;
                 dbg_pc      <= pc;
                 dbg_mode    <= prv_retire;
                 dbg_is_trap <= 0;
                 dbg_pos     <= 0;
                 dbg_s_count <= dbg_s_count + 1;
              end
           end
`endif
           just_trapped <= 0;
           just_xret <= 0;

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
              start_ptw(npc, 2'd0, prv, `S_FETCH2);
           end else begin
              if (npc[63:31] == 1 && npc[63:`MEM_SIZE_LG2] != `MEM_BASEADDR >> `MEM_SIZE_LG2) begin
                 // DRAM fetch physical (above BRAM: 0x80000000-0xFFFFFFFF)
                 fetch_from_dram  <= 1;
                 dram_addr  <= npc[30:3];
                 dram_read        <= 1;
                 state            <= `S_DRAM_FETCH_WAIT;
              end else begin
                 // BRAM fetch physical
                 fetch_from_dram  <= 0;
                 mem_addr0        <= npc[`MEM_SIZE_LG2-1:4] + npc[3];
                 mem_addr1        <= npc[`MEM_SIZE_LG2-1:4];
                 state            <= `S_FETCH1B;
              end
           end

           // Use pre-registered interrupt check (computed previous cycle) for timing closure.
           // pre_intr_pending/pre_intr_cause are stable FFs; the path to state_reg is short.
           // Suppress on the S_FETCH1 right after a trap or xRET: in both cases
           // the interrupt-enable/mask was just changed and we want the target
           // instruction to retire before any newly-unmasked interrupt fires.
           cause_intr = 0;
           if (pre_intr_pending && !just_trapped && !just_xret) begin
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
              // Cross-doubleword case (pc[2:1]==2'b11 && insn[1:0]==2'b11) is
              // detected and re-fetched in S_RF; the upper 64 bits are don't-care.
              aligned = {64'bx, dram_latched};
           end else begin
              aligned = pc[3] == 0 ? {mem_data1_q,mem_data0_q} : {mem_data0_q,mem_data1_q};
           end
           // Register insn; cross-page check and decode happen in S_RF using stable insn_reg.
           insn <= aligned >> (pc[2:1] * 16);
           write_back_register = 0;
           write_back_fp_valid = 0;
           state <= `S_RF;
        end

        `S_RF: begin
           // insn is registered (captured at S_FETCH2 clock edge).
           // Cross-page instruction fetch: 32-bit insn at last halfword of a page
           // In VM mode, the next page may map to a different physical page
           if (pc[11:0] == 12'hFFE && insn[1:0] == 2'b11 &&
               csr_satp[63:60] == 4'd8 && prv != 3) begin
              insn_half <= insn[15:0];
              start_ptw(pc + 2, 2'd0, prv, `S_FETCH2_HALF);
           // Cross-doubleword DRAM fetch: refill the next 8-byte chunk whenever the
           // instruction starts in the last halfword of the current chunk. Even for a
           // 16-bit compressed insn, cosim/debug expect the upper 16 bits to reflect
           // the following halfword rather than zero/X.
           end else if (fetch_from_dram && pc[2:1] == 2'b11) begin
              insn_half       <= insn[15:0];
              if (dram_latched_next_valid) begin
                 dram_latched <= dram_latched_next;
                 state        <= `S_FETCH2_HALF;
              end else begin
                 // For translated fetches, mem_addr still holds the physical
                 // address of the current fetch chunk from the PTW result.
                 dram_addr <= (csr_satp[63:60] == 4'd8 && prv != 3)
                              ? mem_addr[30:3] + 1
                              : pc[30:3] + 1;
                 dram_read       <= 1;
                 state           <= `S_DRAM_FETCH_HALF_WAIT;
              end
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
              if (insn[1:0] == 2 && (insn[15:13] == 3'b001 || insn[15:14] == 2'b01))
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
           f1 <= f1_bram;
           f2 <= f2_bram;
           // Pre-compute SC reservation match one cycle early; S_EXECUTE's
           // SC branch then only sees a 1-bit registered hit.
           reservation_match <= (reservation == s1_bram);
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

           begin : rf3_npc_decode
              reg [63:0] d_imm_i, d_imm_j, d_imm_b, d_c_j, d_c_b;

              d_imm_i = {{52{insn[31]}}, insn[31:20]};
              d_imm_j = {{44{insn[31]}}, insn[19:12], insn[20], insn[30:21], 1'b0};
              d_imm_b = {{52{insn[31]}}, insn[7], insn[30:25], insn[11:8], 1'b0};
              d_c_j   = {{53{insn[12]}}, insn[8], insn[10:9], insn[6], insn[7],
                         insn[2], insn[11], insn[5:3], 1'b0};
              d_c_b   = {{56{insn[12]}}, insn[6:5], insn[2], insn[11:10],
                         insn[4:3], 1'b0};

              pre_npc <= pc + (insn[1:0] == 2'b11 ? 64'd4 : 64'd2);

              if ((insn & 'he003) == 'ha001) begin // C.J
                 pre_npc <= pc + d_c_j;
              end else if ((insn & 'he003) == 'hc001) begin // C.BEQZ
                 if (s1_bram == 0) pre_npc <= pc + d_c_b;
              end else if ((insn & 'he003) == 'he001) begin // C.BNEZ
                 if (s1_bram != 0) pre_npc <= pc + d_c_b;
              end else if ((insn & 'hf07f) == 'h8002) begin // C.JR
                 pre_npc <= s1_bram & ~64'd1;
              end else if ((insn & 'hf07f) == 'h9002) begin // C.JALR
                 pre_npc <= s1_bram & ~64'd1;
              end else if ((insn & 'h0000007f) == 'h0000006f) begin // JAL
                 pre_npc <= pc + d_imm_j;
              end else if ((insn & 'h0000707f) == 'h00000067) begin // JALR
                 pre_npc <= (s1_bram + d_imm_i) & ~64'd1;
              end else if ((insn & 'h0000707f) == 'h00000063) begin // BEQ
                 if (s1_bram == s2_bram) pre_npc <= pc + d_imm_b;
              end else if ((insn & 'h0000707f) == 'h00001063) begin // BNE
                 if (s1_bram != s2_bram) pre_npc <= pc + d_imm_b;
              end else if ((insn & 'h0000707f) == 'h00004063) begin // BLT
                 if ($signed(s1_bram) < $signed(s2_bram)) pre_npc <= pc + d_imm_b;
              end else if ((insn & 'h0000707f) == 'h00005063) begin // BGE
                 if ($signed(s1_bram) >= $signed(s2_bram)) pre_npc <= pc + d_imm_b;
              end else if ((insn & 'h0000707f) == 'h00006063) begin // BLTU
                 if (s1_bram < s2_bram) pre_npc <= pc + d_imm_b;
              end else if ((insn & 'h0000707f) == 'h00007063) begin // BGEU
                 if (s1_bram >= s2_bram) pre_npc <= pc + d_imm_b;
              end
           end // rf3_npc_decode

`ifdef USE_CVFPU
           pre_fp_rnd_mode <= insn[14:12] == 3'b111 ? frm : insn[14:12];
           pre_fp_rmode_ok <= !(insn[14:12] == 3'b101 || insn[14:12] == 3'b110 ||
                                (insn[14:12] == 3'b111 && frm > 3'b100));
`endif

           // Mem pre-decode: compute offset/size/op/mask/wb-reg one cycle
           // early so S_EXECUTE can share a single s1+offset adder instead of
           // selecting between 22 parallel adders. Immediates are computed
           // inline from insn bits (the imm_*/c_uimm* registers are written
           // in S_EXECUTE and therefore stale here).
           begin : rf3_mem_decode
              reg [63:0] d_imm_i_s, d_imm_s_s;
              reg [63:0] d_clw_off, d_cld_off, d_clwsp_off, d_cldsp_off,
                         d_cswsp_off, d_csdsp_off;

              d_imm_i_s    = {{52{insn[31]}}, insn[31:20]};
              d_imm_s_s    = {{52{insn[31]}}, insn[31:25], insn[11:7]};
              d_clw_off    = {57'd0, insn[5],    insn[12:10], insn[6],     2'd0};
              d_cld_off    = {56'd0, insn[6:5],  insn[12:10],              3'd0};
              d_clwsp_off  = {56'd0, insn[3:2],  insn[12],    insn[6:4],   2'd0};
              d_cldsp_off  = {55'd0, insn[4:2],  insn[12],    insn[6:5],   3'd0};
              d_cswsp_off  = {56'd0, insn[8:7],  insn[12:9],               2'd0};
              d_csdsp_off  = {55'd0, insn[9:7],  insn[12:10],              3'd0};

              // Defaults: non-mem instruction
              pre_mem_op        <= `MEMOP_NONE;
              pre_mem_offset    <= 64'd0;
              pre_load_size_lg2 <= 3'd0;
              pre_mem_wr_mask   <= 8'd0;
              pre_mem_wb_reg    <= 5'd0;
              pre_mem_fp        <= 1'b0;

              // Compressed loads / stores (quadrants 0 & 2)
              if ((insn & 'he003) == 'h4000) begin // C.LW
                 pre_mem_op        <= `MEMOP_LOAD;
                 pre_mem_offset    <= d_clw_off;
                 pre_load_size_lg2 <= 3'b110; // W, sign-extend
                 pre_mem_wb_reg    <= {2'b01, insn[4:2]};
              end
              else if ((insn & 'he003) == 'h2000) begin // C.FLD
                 pre_mem_op        <= `MEMOP_LOAD;
                 pre_mem_offset    <= d_cld_off;
                 pre_load_size_lg2 <= 3'b011; // D
                 pre_mem_wb_reg    <= {2'b01, insn[4:2]};
                 pre_mem_fp        <= 1'b1;
              end
              else if ((insn & 'he003) == 'h6000) begin // C.LD
                 pre_mem_op        <= `MEMOP_LOAD;
                 pre_mem_offset    <= d_cld_off;
                 pre_load_size_lg2 <= 3'b011; // D
                 pre_mem_wb_reg    <= {2'b01, insn[4:2]};
              end
              else if ((insn & 'he003) == 'hc000) begin // C.SW
                 pre_mem_op      <= `MEMOP_STORE;
                 pre_mem_offset  <= d_clw_off;
                 pre_mem_wr_mask <= 8'h0f;
              end
              else if ((insn & 'he003) == 'ha000) begin // C.FSD
                 pre_mem_op      <= `MEMOP_STORE;
                 pre_mem_offset  <= d_cld_off;
                 pre_mem_wr_mask <= 8'hff;
                 pre_mem_fp      <= 1'b1;
              end
              else if ((insn & 'he003) == 'he000) begin // C.SD
                 pre_mem_op      <= `MEMOP_STORE;
                 pre_mem_offset  <= d_cld_off;
                 pre_mem_wr_mask <= 8'hff;
              end
              else if ((insn & 'he003) == 'h4002) begin // C.LWSP
                 pre_mem_op        <= `MEMOP_LOAD;
                 pre_mem_offset    <= d_clwsp_off;
                 pre_load_size_lg2 <= 3'b110;
                 pre_mem_wb_reg    <= insn[11:7];
              end
              else if ((insn & 'he003) == 'h2002) begin // C.FLDSP
                 pre_mem_op        <= `MEMOP_LOAD;
                 pre_mem_offset    <= d_cldsp_off;
                 pre_load_size_lg2 <= 3'b011;
                 pre_mem_wb_reg    <= insn[11:7];
                 pre_mem_fp        <= 1'b1;
              end
              else if ((insn & 'he003) == 'h6002) begin // C.LDSP
                 pre_mem_op        <= `MEMOP_LOAD;
                 pre_mem_offset    <= d_cldsp_off;
                 pre_load_size_lg2 <= 3'b011;
                 pre_mem_wb_reg    <= insn[11:7];
              end
              else if ((insn & 'he003) == 'hc002) begin // C.SWSP
                 pre_mem_op      <= `MEMOP_STORE;
                 pre_mem_offset  <= d_cswsp_off;
                 pre_mem_wr_mask <= 8'h0f;
              end
              else if ((insn & 'he003) == 'ha002) begin // C.FSDSP
                 pre_mem_op      <= `MEMOP_STORE;
                 pre_mem_offset  <= d_csdsp_off;
                 pre_mem_wr_mask <= 8'hff;
                 pre_mem_fp      <= 1'b1;
              end
              else if ((insn & 'he003) == 'he002) begin // C.SDSP
                 pre_mem_op      <= `MEMOP_STORE;
                 pre_mem_offset  <= d_csdsp_off;
                 pre_mem_wr_mask <= 8'hff;
              end

              // Uncompressed loads / stores / atomics
              else if (insn[1:0] == 2'b11 && insn[6:2] == 5'b00000) begin // LOAD
                 pre_mem_op        <= `MEMOP_LOAD;
                 pre_mem_offset    <= d_imm_i_s;
                 // funct3 = insn[14:12]: {2:0] = size; [2] = 1 → NO sign-ext (U-variant); invert to match
                 // Current encoding: load_size_lg2 = {sxt, size[1:0]} where sxt=1 means sign-ext.
                 //   LB=0|4, LH=1|4, LW=2|4, LD=3, LBU=0, LHU=1, LWU=2.
                 // RISC-V: funct3[2]=0 is signed (B/H/W), funct3[2]=1 is unsigned (BU/HU/WU); LD has funct3=011 (size=3, no sxt).
                 // So load_size_lg2 = {~funct3[2] & (funct3[1:0] != 2'b11), funct3[1:0]}.
                 pre_load_size_lg2 <= {~insn[14] & ~(insn[13] & insn[12]), insn[13:12]};
                 pre_mem_wb_reg    <= insn[11:7];
              end
              else if (insn[1:0] == 2'b11 && insn[6:2] == 5'b01000) begin // STORE
                 pre_mem_op     <= `MEMOP_STORE;
                 pre_mem_offset <= d_imm_s_s;
                 // wr_mask = (1 << (1 << funct3[1:0])) - 1
                 case (insn[13:12])
                    2'b00: pre_mem_wr_mask <= 8'h01; // SB
                    2'b01: pre_mem_wr_mask <= 8'h03; // SH
                    2'b10: pre_mem_wr_mask <= 8'h0f; // SW
                    2'b11: pre_mem_wr_mask <= 8'hff; // SD
                 endcase
              end
              else if ((insn & 'hf9f0707f) == 'h1000202f ||  // LR.W
                       (insn & 'hf9f0707f) == 'h1000302f) begin // LR.D
                 pre_mem_op        <= `MEMOP_LR;
                 pre_mem_offset    <= 64'd0;
                 pre_load_size_lg2 <= insn[12] ? 3'b011 : 3'b110; // D : W(sign-ext)
                 pre_mem_wb_reg    <= insn[11:7];
              end
              else if ((insn & 'hf800707f) == 'h1800202f ||  // SC.W
                       (insn & 'hf800707f) == 'h1800302f) begin // SC.D
                 pre_mem_op      <= `MEMOP_SC;
                 pre_mem_offset  <= 64'd0;
                 pre_mem_wr_mask <= insn[12] ? 8'hff : 8'h0f;
                 pre_mem_wb_reg  <= insn[11:7];
              end
              // FP loads: FLW (funct3=010) and FLD (funct3=011); opcode 0000111
              else if (insn[1:0] == 2'b11 && insn[6:2] == 5'b00001 &&
                       (insn[14:12] == 3'b010 || insn[14:12] == 3'b011)) begin
                 pre_mem_op        <= `MEMOP_LOAD;
                 pre_mem_offset    <= d_imm_i_s;
                 // FLW: 32-bit zero-extend (load_size_lg2=010), NaN-box in S_LOAD_ALIGN.
                 // FLD: 64-bit (load_size_lg2=011).
                 pre_load_size_lg2 <= {1'b0, insn[13:12]};
                 pre_mem_wb_reg    <= insn[11:7];
                 pre_mem_fp        <= 1'b1;
              end
              // FP stores: FSW (funct3=010) and FSD (funct3=011); opcode 0100111
              else if (insn[1:0] == 2'b11 && insn[6:2] == 5'b01001 &&
                       (insn[14:12] == 3'b010 || insn[14:12] == 3'b011)) begin
                 pre_mem_op        <= `MEMOP_STORE;
                 pre_mem_offset    <= d_imm_s_s;
                 pre_mem_wr_mask   <= insn[12] ? 8'hff : 8'h0f;
                 pre_mem_fp        <= 1'b1;
              end
              else if (insn[1:0] == 2'b11 && insn[6:2] == 5'b01011 &&
                       (insn[14:12] == 3'b010 || insn[14:12] == 3'b011)) begin // AMO*.W / AMO*.D
                 // funct5 must be one of the 9 defined AMO variants; otherwise
                 // leave pre_mem_op = MEMOP_NONE so S_EXECUTE traps illegal-insn.
                 // (LR/SC are funct5 00010/00011, already matched above.)
                 case (insn[31:27])
                    5'b00000, 5'b00001, 5'b00100, 5'b01000, 5'b01100,
                    5'b10000, 5'b10100, 5'b11000, 5'b11100: begin
                       pre_mem_op        <= `MEMOP_AMO;
                       pre_mem_offset    <= 64'd0;
                       pre_load_size_lg2 <= insn[12] ? 3'b011 : 3'b010; // D : W(no sxt)
                       pre_mem_wb_reg    <= insn[11:7];
                    end
                    default: ; // illegal AMO funct5: falls through
                 endcase
              end
           end // rf3_mem_decode
        end

        `S_FETCH1B: begin
           // Synchronous SRAM read.  mem_addr0/mem_addr1 were set in the
           // preceding state; S_FETCH2/S_FETCH2_HALF consume the registered data.
           mem_data0_q <= mem0[mem_addr0];
           mem_data1_q <= mem1[mem_addr1];
           state <= fetch_latch_half ? `S_FETCH2_HALF : `S_FETCH2;
           fetch_latch_half <= 0;
        end

        `S_LOAD_LATCH: begin
           // Synchronous SRAM read.  S_LOAD_ALIGN consumes the registered data.
           mem_data0_q <= mem0[mem_addr0];
           mem_data1_q <= mem1[mem_addr1];
           state <= `S_LOAD_ALIGN;
        end

        `S_EXECUTE: begin
           state <= `S_EXECUTE2; // Default: complete write_back_value
           prv_retire <= prv;    // snapshot pre-execution prv (MRET/SRET mutate prv below)

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

           npc = pre_npc;

           // RV64IC decoding
           //
           // The order of instructions [mostly] follows
           // simmerv for ease of reference, who in turn took the
           // ordering from the RISC-V spec.  There is intentionally
           // _no_ overlap in patterns so the order is not important,
           // but we keep the if-else chain in order to catch the
           // unhandled instructions.

           // Shared mem-access block — collapses all load/store/LR/SC/AMO
           // branches using the pre-decoded signals from rf3_mem_decode.
           // One s1+pre_mem_offset adder and one mem_addr0/1 splitter
           // replace 22 parallel copies, shrinking the mem_addr critical
           // path from ~14 LUT levels to ~7.
           if (pre_mem_op != `MEMOP_NONE) begin
              if (pre_mem_fp && fs == 0) begin
                 cause = `TRAP_ILLEGAL_INSTRUCTION;
                 tval = insn;
                 state <= `S_EXCEPTION;
              end else begin
              if (pre_mem_fp) fs = 3;
              write_back_register    = pre_mem_fp ? 5'd0 : pre_mem_wb_reg;
              write_back_fp_valid    = pre_mem_fp && pre_mem_op == `MEMOP_LOAD;
              write_back_fp_register = pre_mem_wb_reg;
              mem_addr      = s1 + pre_mem_offset;
              load_size_lg2 = pre_load_size_lg2;
              mem_addr0    <= mem_addr[63:4] + mem_addr[3];
              mem_addr1    <= mem_addr[63:4];
              begin : mem_access_dispatch
                 reg [12:0] mem_access_bytes;

                 case (pre_mem_op)
                   `MEMOP_STORE,
                   `MEMOP_SC: begin
                      case (pre_mem_wr_mask)
                        8'hff: mem_access_bytes = 13'd8;
                        8'h0f: mem_access_bytes = 13'd4;
                        8'h03: mem_access_bytes = 13'd2;
                        default: mem_access_bytes = 13'd1;
                      endcase
                   end
                   default: mem_access_bytes = 13'd1 << pre_load_size_lg2[1:0];
                 endcase

                 if (csr_satp[63:60] == 4'd8 && (mprv ? mpp : prv) != 3 &&
                     (pre_mem_op != `MEMOP_SC || reservation_match) &&
                     ({1'b0, mem_addr[11:0]} + mem_access_bytes > 13'd4096)) begin
                    cause = (pre_mem_op == `MEMOP_STORE || pre_mem_op == `MEMOP_SC || pre_mem_op == `MEMOP_AMO)
                            ? `TRAP_STORE_ADDRESS_MISALIGNED
                            : `TRAP_LOAD_ADDRESS_MISALIGNED;
                    tval = mem_addr;
                    write_back_register = 0;
                    write_back_fp_valid = 0;
                    state <= `S_EXCEPTION;
                 end else begin
                    case (pre_mem_op)
                       `MEMOP_LOAD: state <= `S_LOAD_LATCH;
                       `MEMOP_STORE: begin
                          mem_wr_mask = pre_mem_wr_mask;
                          store_value = pre_mem_fp ? f2 : s2;
                          state <= `S_STORE;
                       end
                       `MEMOP_LR: begin
                          reservation <= s1;
                          state <= `S_LOAD_LATCH;
                       end
                       `MEMOP_SC: begin
                          if (reservation_match) begin
                             write_back_value <= 0;
                             mem_wr_mask = pre_mem_wr_mask;
                             store_value = s2;
                             state <= `S_STORE;
                          end
                          // SC fail: write_back_value = 1 from EXOP_ONE in rf3_pre_decode;
                          // default state <= S_EXECUTE2 at top of S_EXECUTE retires it.
                       end
                       `MEMOP_AMO: begin
                          do_atomic <= 1;
                          state <= `S_LOAD_LATCH;
                       end
                       default: ;
                    endcase
                 end
              end
              end // else: !(pre_mem_fp && fs == 0)
           end

           // Quadrant 0
           else if ((insn & 'he003) == 'h0000) begin // C.ADDI4SPN/illegal
              write_back_register = rs2;
              if ((insn & 'hffff) == 0) begin
                 write_back_register = 0;
                 cause = `TRAP_ILLEGAL_INSTRUCTION;
                 tval = insn;
                 state <= `S_EXCEPTION;
              end
           end

           // Compressed integer/FP loads and stores are handled by the shared mem block above.

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
           end

           else if ((insn & 'he003) == 'hc001) begin // C.BEQZ
           end

           else if ((insn & 'he003) == 'he001) begin // C.BNEZ
           end


              // Quadrant 2
           else if ((insn & 'he003) == 'h0002) begin // C.SLLI
              write_back_register = rs1;
           end

           // C.LWSP / C.LDSP / C.FLDSP handled by shared mem block above.

           else if ((insn & 'hf07f) == 'h8002) begin // C.JR
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
           end

           else if ((insn & 'hf003) == 'h9002) begin // C.ADD
              write_back_register = rs1;
           end

           // C.SWSP / C.SDSP / C.FSDSP handled by shared mem block above.

           // Quadrant 3, uncompressed
           else if ((insn & 'h0000007f) == 'h00000037) begin // LUI
              write_back_register = rd;
           end

           else if ((insn & 'h0000007f) == 'h00000017) begin // AUIPC
              write_back_register = rd;
           end

           else if ((insn & 'h0000007f) == 'h0000006f) begin // JAL
              write_back_register = rd;
           end

           else if ((insn & 'h0000707f) == 'h00000067) begin // JALR
              write_back_register = rd;
           end

           else if ((insn & 'h0000707f) == 'h00000063) begin // BEQ
           end

           else if ((insn & 'h0000707f) == 'h00001063) begin // BNE
           end

           else if ((insn & 'h0000707f) == 'h00004063) begin // BLT
           end

           else if ((insn & 'h0000707f) == 'h00005063) begin // BGE
           end

           else if ((insn & 'h0000707f) == 'h00006063) begin // BLTU
           end

           else if ((insn & 'h0000707f) == 'h00007063) begin // BGEU
           end

           // LB/LH/LW/LD/LBU/LHU/LWU and SB/SH/SW/SD handled by shared mem block above.

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

           // LR.W/D, SC.W/D, and all AMO*.W/D variants handled by shared mem block above.

           else if ((insn & 'hffffffff) == 'h30200073) begin // MRET
              if (mpp != 3) mprv = 0;
              prv = mpp;
              mpp = 0;
              mie = mpie;
              mpie = 1;
              npc = csr_mepc;
              just_xret <= 1;
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
                 just_xret <= 1;
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

           // OP-FP (opcode 0x53) — arithmetic-free Phase 1 insns:
           // FMV.{W.X,X.W,D.X,X.D}, FSGNJ{,N,X}.{S,D}, FCLASS.{S,D}.
           // Everything else in this opcode falls through to illegal.
           else if (insn[6:0] == 7'b1010011) begin
              if (fs == 0) begin
                 // FP state disabled by mstatus.FS — any FP insn traps.
                 cause = `TRAP_ILLEGAL_INSTRUCTION;
                 tval = insn;
                 state <= `S_EXCEPTION;
              end else begin
                 // Reaching an FP insn dirties the FP state.
                 fs = 3;
                 case (insn[31:25])
                   // FADD.S / FSUB.S
                   7'b0000000,
                   7'b0000100: begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         cvfpu_operands[0] <= 64'd0;
                         cvfpu_operands[1] <= f1;
                         cvfpu_operands[2] <= f2;
                         cvfpu_rnd_mode <= pre_fp_rnd_mode;
                         cvfpu_op       <= 4'd2; // fpnew_pkg::ADD
                         cvfpu_op_mod   <= insn[27]; // 0=add, 1=sub
                         cvfpu_src_fmt  <= 3'd0; // fpnew_pkg::FP32
                         cvfpu_dst_fmt  <= 3'd0; // fpnew_pkg::FP32
                         cvfpu_int_fmt  <= 2'd3; // fpnew_pkg::INT64 (unused)
                         cvfpu_tag_in   <= {3'd0, rd};
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
`endif
                   end
                   // FADD.D / FSUB.D
                   7'b0000001,
                   7'b0000101: begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         cvfpu_operands[0] <= 64'd0;
                         cvfpu_operands[1] <= f1;
                         cvfpu_operands[2] <= f2;
                         cvfpu_rnd_mode <= pre_fp_rnd_mode;
                         cvfpu_op       <= 4'd2; // fpnew_pkg::ADD
                         cvfpu_op_mod   <= insn[27]; // 0=add, 1=sub
                         cvfpu_src_fmt  <= 3'd1; // fpnew_pkg::FP64
                         cvfpu_dst_fmt  <= 3'd1; // fpnew_pkg::FP64
                         cvfpu_int_fmt  <= 2'd3; // fpnew_pkg::INT64 (unused)
                         cvfpu_tag_in   <= {3'd0, rd};
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
`endif
                   end
                   // FMUL.S
                   7'b0001000: begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         cvfpu_operands[0] <= f1;
                         cvfpu_operands[1] <= f2;
                         cvfpu_operands[2] <= 64'd0;
                         cvfpu_rnd_mode <= pre_fp_rnd_mode;
                         cvfpu_op       <= 4'd3; // fpnew_pkg::MUL
                         cvfpu_op_mod   <= 1'b0;
                         cvfpu_src_fmt  <= 3'd0; // fpnew_pkg::FP32
                         cvfpu_dst_fmt  <= 3'd0; // fpnew_pkg::FP32
                         cvfpu_int_fmt  <= 2'd3; // fpnew_pkg::INT64 (unused)
                         cvfpu_tag_in   <= {3'd0, rd};
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
`endif
                   end
                   // FMUL.D
                   7'b0001001: begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         cvfpu_operands[0] <= f1;
                         cvfpu_operands[1] <= f2;
                         cvfpu_operands[2] <= 64'd0;
                         cvfpu_rnd_mode <= pre_fp_rnd_mode;
                         cvfpu_op       <= 4'd3; // fpnew_pkg::MUL
                         cvfpu_op_mod   <= 1'b0;
                         cvfpu_src_fmt  <= 3'd1; // fpnew_pkg::FP64
                         cvfpu_dst_fmt  <= 3'd1; // fpnew_pkg::FP64
                         cvfpu_int_fmt  <= 2'd3; // fpnew_pkg::INT64 (unused)
                         cvfpu_tag_in   <= {3'd0, rd};
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
`endif
                   end
                   // FDIV.S
                   7'b0001100: begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         cvfpu_operands[0] <= f1;
                         cvfpu_operands[1] <= f2;
                         cvfpu_operands[2] <= 64'd0;
                         cvfpu_rnd_mode <= pre_fp_rnd_mode;
                         cvfpu_op       <= 4'd4; // fpnew_pkg::DIV
                         cvfpu_op_mod   <= 1'b0;
                         cvfpu_src_fmt  <= 3'd0; // fpnew_pkg::FP32
                         cvfpu_dst_fmt  <= 3'd0; // fpnew_pkg::FP32
                         cvfpu_int_fmt  <= 2'd3; // fpnew_pkg::INT64 (unused)
                         cvfpu_tag_in   <= {3'd0, rd};
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
`endif
                   end
                   // FDIV.D
                   7'b0001101: begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         cvfpu_operands[0] <= f1;
                         cvfpu_operands[1] <= f2;
                         cvfpu_operands[2] <= 64'd0;
                         cvfpu_rnd_mode <= pre_fp_rnd_mode;
                         cvfpu_op       <= 4'd4; // fpnew_pkg::DIV
                         cvfpu_op_mod   <= 1'b0;
                         cvfpu_src_fmt  <= 3'd1; // fpnew_pkg::FP64
                         cvfpu_dst_fmt  <= 3'd1; // fpnew_pkg::FP64
                         cvfpu_int_fmt  <= 2'd3; // fpnew_pkg::INT64 (unused)
                         cvfpu_tag_in   <= {3'd0, rd};
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
`endif
                   end
                   // FSQRT.S
                   7'b0101100: if (insn[24:20] == 5'd0) begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         cvfpu_operands[0] <= f1;
                         cvfpu_operands[1] <= 64'd0;
                         cvfpu_operands[2] <= 64'd0;
                         cvfpu_rnd_mode <= pre_fp_rnd_mode;
                         cvfpu_op       <= 4'd5; // fpnew_pkg::SQRT
                         cvfpu_op_mod   <= 1'b0;
                         cvfpu_src_fmt  <= 3'd0; // fpnew_pkg::FP32
                         cvfpu_dst_fmt  <= 3'd0; // fpnew_pkg::FP32
                         cvfpu_int_fmt  <= 2'd3; // fpnew_pkg::INT64 (unused)
                         cvfpu_tag_in   <= {3'd0, rd};
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
`endif
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
                   end
                   // FSQRT.D
                   7'b0101101: if (insn[24:20] == 5'd0) begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         cvfpu_operands[0] <= f1;
                         cvfpu_operands[1] <= 64'd0;
                         cvfpu_operands[2] <= 64'd0;
                         cvfpu_rnd_mode <= pre_fp_rnd_mode;
                         cvfpu_op       <= 4'd5; // fpnew_pkg::SQRT
                         cvfpu_op_mod   <= 1'b0;
                         cvfpu_src_fmt  <= 3'd1; // fpnew_pkg::FP64
                         cvfpu_dst_fmt  <= 3'd1; // fpnew_pkg::FP64
                         cvfpu_int_fmt  <= 2'd3; // fpnew_pkg::INT64 (unused)
                         cvfpu_tag_in   <= {3'd0, rd};
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
`endif
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
                   end
                   // FMIN.S / FMAX.S
                   7'b0010100: begin
`ifdef USE_CVFPU
                      if (insn[14:12] > 3'b001) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         cvfpu_operands[0] <= f1;
                         cvfpu_operands[1] <= f2;
                         cvfpu_operands[2] <= 64'd0;
                         cvfpu_rnd_mode <= {2'b00, insn[12]}; // RNE=min, RTZ=max
                         cvfpu_op       <= 4'd7; // fpnew_pkg::MINMAX
                         cvfpu_op_mod   <= 1'b0;
                         cvfpu_src_fmt  <= 3'd0; // fpnew_pkg::FP32
                         cvfpu_dst_fmt  <= 3'd0; // fpnew_pkg::FP32
                         cvfpu_int_fmt  <= 2'd3; // fpnew_pkg::INT64 (unused)
                         cvfpu_tag_in   <= {3'd0, rd};
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
`endif
                   end
                   // FMIN.D / FMAX.D
                   7'b0010101: begin
`ifdef USE_CVFPU
                      if (insn[14:12] > 3'b001) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         cvfpu_operands[0] <= f1;
                         cvfpu_operands[1] <= f2;
                         cvfpu_operands[2] <= 64'd0;
                         cvfpu_rnd_mode <= {2'b00, insn[12]}; // RNE=min, RTZ=max
                         cvfpu_op       <= 4'd7; // fpnew_pkg::MINMAX
                         cvfpu_op_mod   <= 1'b0;
                         cvfpu_src_fmt  <= 3'd1; // fpnew_pkg::FP64
                         cvfpu_dst_fmt  <= 3'd1; // fpnew_pkg::FP64
                         cvfpu_int_fmt  <= 2'd3; // fpnew_pkg::INT64 (unused)
                         cvfpu_tag_in   <= {3'd0, rd};
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
`endif
                   end
                   // FCVT.S.D
                   7'b0100000: if (insn[24:20] == 5'd1) begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         cvfpu_operands[0] <= f1;
                         cvfpu_operands[1] <= 64'd0;
                         cvfpu_operands[2] <= 64'd0;
                         cvfpu_rnd_mode <= pre_fp_rnd_mode;
                         cvfpu_op       <= 4'd10; // fpnew_pkg::F2F
                         cvfpu_op_mod   <= 1'b0;
                         cvfpu_src_fmt  <= 3'd1; // fpnew_pkg::FP64
                         cvfpu_dst_fmt  <= 3'd0; // fpnew_pkg::FP32
                         cvfpu_int_fmt  <= 2'd3; // fpnew_pkg::INT64 (unused)
                         cvfpu_tag_in   <= {3'd0, rd};
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
`endif
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
                   end
                   // FCVT.D.S
                   7'b0100001: if (insn[24:20] == 5'd0) begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         cvfpu_operands[0] <= f1;
                         cvfpu_operands[1] <= 64'd0;
                         cvfpu_operands[2] <= 64'd0;
                         cvfpu_rnd_mode <= pre_fp_rnd_mode;
                         cvfpu_op       <= 4'd10; // fpnew_pkg::F2F
                         cvfpu_op_mod   <= 1'b0;
                         cvfpu_src_fmt  <= 3'd0; // fpnew_pkg::FP32
                         cvfpu_dst_fmt  <= 3'd1; // fpnew_pkg::FP64
                         cvfpu_int_fmt  <= 2'd3; // fpnew_pkg::INT64 (unused)
                         cvfpu_tag_in   <= {3'd0, rd};
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
`endif
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
                   end
                   // FSGNJ/N/X .S — NaN-box-check operands; NaN-box result.
                   7'b0010000: begin
                      write_back_fp_valid    = 1;
                      write_back_fp_register = rd;
                      case (insn[14:12])
                        3'b000: write_back_fp_value <= {32'hffffffff, f2_s[31],             f1_s[30:0]};
                        3'b001: write_back_fp_value <= {32'hffffffff, ~f2_s[31],            f1_s[30:0]};
                        3'b010: write_back_fp_value <= {32'hffffffff, f2_s[31] ^ f1_s[31],  f1_s[30:0]};
                        default: begin
                           write_back_fp_valid = 0;
                           cause = `TRAP_ILLEGAL_INSTRUCTION;
                           tval = insn;
                           state <= `S_EXCEPTION;
                        end
                      endcase
                      if (insn[14:12] < 3) state <= `S_FETCH1;
                   end
                   // FSGNJ/N/X .D — no boxing check; 64-bit direct.
                   7'b0010001: begin
                      write_back_fp_valid    = 1;
                      write_back_fp_register = rd;
                      case (insn[14:12])
                        3'b000: write_back_fp_value <= {f2[63],         f1[62:0]};
                        3'b001: write_back_fp_value <= {~f2[63],        f1[62:0]};
                        3'b010: write_back_fp_value <= {f2[63] ^ f1[63], f1[62:0]};
                        default: begin
                           write_back_fp_valid = 0;
                           cause = `TRAP_ILLEGAL_INSTRUCTION;
                           tval = insn;
                           state <= `S_EXCEPTION;
                        end
                      endcase
                      if (insn[14:12] < 3) state <= `S_FETCH1;
                   end
                   // FEQ.S / FLT.S / FLE.S — integer rd; NV flag on NaN per op.
                   7'b1010000: if (insn[14:12] <= 3'b010) begin
                      fcmp_result = fcmp_s(insn[14:12], f1_s, f2_s);
                      write_back_register = rd;
                      write_back_value    <= {63'd0, fcmp_result[0]};
                      if (fcmp_result[1]) fflags = fflags | 5'b10000;
                      state               <= `S_FETCH1;
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
                   end
                   // FEQ.D / FLT.D / FLE.D
                   7'b1010001: if (insn[14:12] <= 3'b010) begin
                      fcmp_result = fcmp_d(insn[14:12], f1, f2);
                      write_back_register = rd;
                      write_back_value    <= {63'd0, fcmp_result[0]};
                      if (fcmp_result[1]) fflags = fflags | 5'b10000;
                      state               <= `S_FETCH1;
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
                   end
                   // FCVT.W[U].S / FCVT.L[U].S
                   7'b1100000: if (insn[24:20] <= 5'd3) begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         cvfpu_operands[0] <= f1;
                         cvfpu_operands[1] <= 64'd0;
                         cvfpu_operands[2] <= 64'd0;
                         cvfpu_rnd_mode <= pre_fp_rnd_mode;
                         cvfpu_op       <= 4'd11; // fpnew_pkg::F2I
                         cvfpu_op_mod   <= insn[20]; // 0=signed, 1=unsigned
                         cvfpu_src_fmt  <= 3'd0; // fpnew_pkg::FP32
                         cvfpu_dst_fmt  <= 3'd0; // unused
                         cvfpu_int_fmt  <= insn[21] ? 2'd3 : 2'd2; // INT64 : INT32
                         cvfpu_tag_in   <= {3'd0, rd};
                         cvfpu_write_fp <= 1'b0;
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
`endif
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
                   end
                   // FCVT.W[U].D / FCVT.L[U].D
                   7'b1100001: if (insn[24:20] <= 5'd3) begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         cvfpu_operands[0] <= f1;
                         cvfpu_operands[1] <= 64'd0;
                         cvfpu_operands[2] <= 64'd0;
                         cvfpu_rnd_mode <= pre_fp_rnd_mode;
                         cvfpu_op       <= 4'd11; // fpnew_pkg::F2I
                         cvfpu_op_mod   <= insn[20]; // 0=signed, 1=unsigned
                         cvfpu_src_fmt  <= 3'd1; // fpnew_pkg::FP64
                         cvfpu_dst_fmt  <= 3'd0; // unused
                         cvfpu_int_fmt  <= insn[21] ? 2'd3 : 2'd2; // INT64 : INT32
                         cvfpu_tag_in   <= {3'd0, rd};
                         cvfpu_write_fp <= 1'b0;
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
`endif
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
                   end
                   // FCVT.S.W[U] / FCVT.S.L[U]
                   7'b1101000: if (insn[24:20] <= 5'd3) begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         cvfpu_operands[0] <= s1;
                         cvfpu_operands[1] <= 64'd0;
                         cvfpu_operands[2] <= 64'd0;
                         cvfpu_rnd_mode <= pre_fp_rnd_mode;
                         cvfpu_op       <= 4'd12; // fpnew_pkg::I2F
                         cvfpu_op_mod   <= insn[20]; // 0=signed, 1=unsigned
                         cvfpu_src_fmt  <= 3'd0; // unused
                         cvfpu_dst_fmt  <= 3'd0; // fpnew_pkg::FP32
                         cvfpu_int_fmt  <= insn[21] ? 2'd3 : 2'd2; // INT64 : INT32
                         cvfpu_tag_in   <= {3'd0, rd};
                         cvfpu_write_fp <= 1'b1;
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
`endif
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
                   end
                   // FCVT.D.W[U] / FCVT.D.L[U]
                   7'b1101001: if (insn[24:20] <= 5'd3) begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         cvfpu_operands[0] <= s1;
                         cvfpu_operands[1] <= 64'd0;
                         cvfpu_operands[2] <= 64'd0;
                         cvfpu_rnd_mode <= pre_fp_rnd_mode;
                         cvfpu_op       <= 4'd12; // fpnew_pkg::I2F
                         cvfpu_op_mod   <= insn[20]; // 0=signed, 1=unsigned
                         cvfpu_src_fmt  <= 3'd0; // unused
                         cvfpu_dst_fmt  <= 3'd1; // fpnew_pkg::FP64
                         cvfpu_int_fmt  <= insn[21] ? 2'd3 : 2'd2; // INT64 : INT32
                         cvfpu_tag_in   <= {3'd0, rd};
                         cvfpu_write_fp <= 1'b1;
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
`endif
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
                   end
                   // FMV.X.W (rs2=0, rm=0) or FCLASS.S (rs2=0, rm=1).
                   7'b1110000: if (insn[24:20] == 5'd0 && insn[14:12] == 3'b000) begin
                      write_back_register = rd;
                      write_back_value    <= {{32{f1[31]}}, f1[31:0]};
                      state               <= `S_FETCH1;
                   end else if (insn[24:20] == 5'd0 && insn[14:12] == 3'b001) begin
                      write_back_register = rd;
                      write_back_value    <= fclass_s(f1);
                      state               <= `S_FETCH1;
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
                   end
                   // FMV.X.D (rs2=0, rm=0) or FCLASS.D (rs2=0, rm=1).
                   7'b1110001: if (insn[24:20] == 5'd0 && insn[14:12] == 3'b000) begin
                      write_back_register = rd;
                      write_back_value    <= f1;
                      state               <= `S_FETCH1;
                   end else if (insn[24:20] == 5'd0 && insn[14:12] == 3'b001) begin
                      write_back_register = rd;
                      write_back_value    <= fclass_d(f1);
                      state               <= `S_FETCH1;
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
                   end
                   // FMV.W.X (rs2=0, rm=0): NaN-box s1[31:0] into f[rd].
                   7'b1111000: if (insn[24:20] == 5'd0 && insn[14:12] == 3'b000) begin
                      write_back_fp_valid    = 1;
                      write_back_fp_register = rd;
                      write_back_fp_value    <= {32'hffffffff, s1[31:0]};
                      state                  <= `S_FETCH1;
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
                   end
                   // FMV.D.X (rs2=0, rm=0): full 64-bit move.
                   7'b1111001: if (insn[24:20] == 5'd0 && insn[14:12] == 3'b000) begin
                      write_back_fp_valid    = 1;
                      write_back_fp_register = rd;
                      write_back_fp_value    <= s1;
                      state                  <= `S_FETCH1;
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
                   end
                   default: begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
                   end
                 endcase
              end
           end

           // R4 fused multiply-add/subtract family: FMADD/FMSUB/FNMSUB/FNMADD.
           else if (insn[6:4] == 3'b100 && insn[1:0] == 2'b11) begin
              if (fs == 0) begin
                 cause = `TRAP_ILLEGAL_INSTRUCTION;
                 tval = insn;
                 state <= `S_EXCEPTION;
              end else begin
                 fs = 3;
`ifdef USE_CVFPU
                 if (insn[26:25] > 2'b01 || !pre_fp_rmode_ok) begin
                    cause = `TRAP_ILLEGAL_INSTRUCTION;
                    tval = insn;
                    state <= `S_EXCEPTION;
                 end else begin
                    rs1 <= insn[31:27]; // rs3; reuse FP read port 0
                    state <= `S_CVFPU_FMA_RF2;
                 end
`else
                 cause = `TRAP_ILLEGAL_INSTRUCTION;
                 tval = insn;
                 state <= `S_EXCEPTION;
`endif
              end
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

`ifdef USE_CVFPU
        `S_CVFPU_FMA_RF2: begin
           state <= `S_CVFPU_FMA_RF3;
        end

        `S_CVFPU_FMA_RF3: begin
           cvfpu_operands[0] <= f1;
           cvfpu_operands[1] <= f2;
           cvfpu_operands[2] <= f1_bram;
           cvfpu_rnd_mode <= pre_fp_rnd_mode;
           cvfpu_op       <= insn[3] ? 4'd1 : 4'd0; // FNMSUB : FMADD
           cvfpu_op_mod   <= insn[2]; // add/sub variant
           cvfpu_src_fmt  <= {2'd0, insn[25]}; // FP32/FP64
           cvfpu_dst_fmt  <= {2'd0, insn[25]}; // FP32/FP64
           cvfpu_int_fmt  <= 2'd3; // fpnew_pkg::INT64 (unused)
           cvfpu_tag_in   <= {3'd0, rd};
           cvfpu_write_fp <= 1'b1;
           cvfpu_in_valid <= 1'b1;
           state          <= `S_CVFPU_ISSUE;
        end

        `S_CVFPU_ISSUE: begin
           if (cvfpu_in_ready) begin
              cvfpu_in_valid <= 1'b0;
              if (cvfpu_out_valid) begin
                 if (cvfpu_write_fp) begin
                    write_back_fp_valid    = 1;
                    write_back_fp_register = cvfpu_tag_out[4:0];
                    write_back_fp_value    <= cvfpu_result;
                 end else begin
                    write_back_register = cvfpu_tag_out[4:0];
                    write_back_value    <= cvfpu_result;
                 end
                 fflags = fflags | cvfpu_fflags;
                 state  <= `S_FETCH1;
              end else begin
                 state <= `S_CVFPU_WAIT;
              end
           end
        end

        `S_CVFPU_WAIT: begin
           if (cvfpu_out_valid) begin
              if (cvfpu_write_fp) begin
                 write_back_fp_valid    = 1;
                 write_back_fp_register = cvfpu_tag_out[4:0];
                 write_back_fp_value    <= cvfpu_result;
              end else begin
                 write_back_register = cvfpu_tag_out[4:0];
                 write_back_value    <= cvfpu_result;
              end
              fflags = fflags | cvfpu_fflags;
              state  <= `S_FETCH1;
           end
        end
`endif

        `S_STORE: begin

           if (csr_satp[63:60] == 4'd8 && (mprv ? mpp : prv) != 3 && !translated) begin
              // Sv39 store address translation
              start_ptw(mem_addr, 2'd2, mprv ? mpp : prv, `S_STORE);
           end else if (phys_region(mem_addr) == `REGION_UART) begin
              translated <= 0;
              state <= `S_FETCH1;
              reservation <= ~0;

              // Keep UART writes on the original store cycle; only BRAM writes
              // need the delayed commit state for SRAM WE timing.
              case (mem_addr[2:0])
                0: if (!uart_lcr[7]) begin // THR (when DLAB=0)
`ifdef PC_TRACE
                      if (!dbg_armed) begin
                         uart_tx_valid <= 1;
                         uart_tx_data <= store_value[7:0];
                      end
`else
                      uart_tx_valid <= 1;
                      uart_tx_data <= store_value[7:0];
`endif
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
           end else begin
              state <= `S_STORE_COMMIT;
           end
        end

        `S_STORE_COMMIT: begin
           translated <= 0;
           state <= `S_FETCH1;
           reservation <= ~0;
           // Keep the mem_data*_q clock-enable independent of the address
           // region decode; only REGION_BRAM consumes these latched words.
           mem_data0_q <= mem0[mem_addr0];
           mem_data1_q <= mem1[mem_addr1];

`ifdef RISCV_TESTS
           // riscv-tests signal completion by storing gp to "tohost". In the
           // FPGA-like sim config the program image lives behind AXI, so this
           // terminating store must be recognized before the store is routed.
           if (mem_addr == tohost_phys && mem_wr_mask[0] && store_value != 0) begin
              if (store_value == 1)
                 $display("Test Passed");
              else
                 $display("Test Failed with %3d", store_value);
              $finish;
           end
`endif

           case (phys_region(mem_addr))
             `REGION_UART: begin
              // NS16550A UART write (0x10000000-0x1000000F)
              case (mem_addr[2:0])
                0: if (!uart_lcr[7]) begin // THR (when DLAB=0)
`ifdef PC_TRACE
                      if (!dbg_armed) begin
                         uart_tx_valid <= 1;
                         uart_tx_data <= store_value[7:0];
                      end
`else
                      uart_tx_valid <= 1;
                      uart_tx_data <= store_value[7:0];
`endif
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
             end
             `REGION_CLINT: begin
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
             end
             `REGION_PLIC: begin
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
             end
             `REGION_BRAM: begin
              // BRAM store: read the affected words now and merge/write full
              // 64-bit words in S_STORE_BRAM_WRITE.  This removes the old
              // byte-wide write-enable fanout from the store commit state.
              begin : bram_store_prepare
                 reg [127:0] bram_aligned;
                 reg [ 15:0] bram_mask;
                 bram_aligned = {64'd0,store_value} << (8 * (mem_addr % 8));
                 bram_mask = mem_wr_mask << (mem_addr % 8);
                 if (mem_addr[3]) begin
                    bram_aligned = {bram_aligned[63:0], bram_aligned[127:64]};
                    bram_mask = {bram_mask[7:0], bram_mask[15:8]};
                 end
                 bram_store_aligned <= bram_aligned;
                 bram_store_mask    <= bram_mask;
                 mem_wr_mask = 0;
                 state <= `S_STORE_BRAM_WRITE;
              end
             end
             `REGION_MMIO: begin
`ifdef TRACE_MMIO
              $display("%05d  MMIO WRITE %x/%x <- %x", $time, mem_addr, mem_wr_mask, store_value);
`endif

              mmio_address = mem_addr;
              mmio_write = 1;
              mmio_writedata = store_value << (8 * (mem_addr % 4));
              mmio_byteenable = mem_wr_mask << (mem_addr % 4);

              mem_wr_mask = 0;
             end
             `REGION_DRAM: begin
              // DRAM store (0x80000000-0xFFFFFFFF)
              begin : dram_store_calc
                 reg [127:0] wide_data;
                 reg  [15:0] wide_mask;
                 wide_data = {64'd0, store_value} << (mem_addr[2:0] * 8);
                 wide_mask = {8'd0, mem_wr_mask[7:0]} << mem_addr[2:0];
                 dram_addr       <= mem_addr[30:3];
                 dram_writedata  <= wide_data[63:0];
                 dram_wstrb      <= wide_mask[7:0];
                 mem_wr_mask     = 0;
                 if (|wide_mask[15:8]) begin
                    // Overflow into next 8-byte chunk: save for S_DRAM_STORE2
                    dram2_addr        <= mem_addr[30:3] + 1;
                    dram2_data_part   <= wide_data[127:64];
                    dram2_wstrb       <= wide_mask[15:8];
                    dram_store_split  <= 1;
                    if (dram_write_ready) begin
                       dram_write <= 1;
                       state      <= `S_DRAM_STORE_RESP_ARM;
                    end else begin
                       state      <= `S_DRAM_STORE_WAIT;
                    end
                 end else begin
                    dram_store_split  <= 0;
                    if (dram_write_ready) begin
                       dram_write <= 1;
                       state      <= `S_DRAM_STORE_RESP_ARM;
                    end else begin
                       state      <= `S_DRAM_STORE_WAIT;
                    end
                 end
              end
             end
             default: begin
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
           endcase

        end

        `S_STORE_BRAM_WRITE: begin
           if (|bram_store_mask[7:0])
              mem0[mem_addr0] <= merge_store_bytes(mem_data0_q,
                                                   bram_store_aligned[63:0],
                                                   bram_store_mask[7:0]);
           if (|bram_store_mask[15:8])
              mem1[mem_addr1] <= merge_store_bytes(mem_data1_q,
                                                   bram_store_aligned[127:64],
                                                   bram_store_mask[15:8]);
           state <= `S_FETCH1;
        end

        `S_LOAD_ALIGN: begin
           if (csr_satp[63:60] == 4'd8 && (mprv ? mpp : prv) != 3 && !translated) begin
              // Sv39 load/AMO address translation
              start_ptw(mem_addr, do_atomic ? 2'd3 : 2'd1, mprv ? mpp : prv, `S_LOAD_LATCH);
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

              case (phys_region(mem_addr))
                `REGION_UART: begin
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
                 // Byte/half sign extension for LB/LH (load_size_lg2 = {sxt, size}).
                 if (load_size_lg2 == 4) // LB
                    write_back_value = {{56{write_back_value[7]}}, write_back_value[7:0]};
                 else if (load_size_lg2 == 5) // LH
                    write_back_value = {{48{write_back_value[15]}}, write_back_value[15:0]};
                end
                `REGION_CLINT: begin
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
                end
                `REGION_PLIC: begin
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
                end
                `REGION_BRAM: begin
                 // BRAM load: write_back_value already computed from speculative read above
                end
                `REGION_MMIO: begin
`ifdef TRACE_MMIO
                 $display("%05d  MMIO READ FROM %x/%x", $time, mem_addr, load_size_lg2);
`endif
                 state <= `S_MMIO_READ;

                 mmio_address = mem_addr;
                 mmio_read = 1;
                end
                `REGION_DRAM: begin
                 // DRAM load (0x80000000-0xFFFFFFFF)
                 dram_addr <= mem_addr[30:3];
                 dram_read       <= 1;
                 state           <= `S_DRAM_LOAD_WAIT;
                end
                default: begin
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
              endcase
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
           state <= `S_EXECUTE2;
           csr_access_failure = 0;
           write_back_register = rd;

           if (rd != 0 || csr_op != `CSR_OP_COPY) begin
              // read the CSR

              // PMP: pmpcfg0-15 and pmpaddr0-63 — M-mode only, reads zero
              // (0 PMP entries implemented; all accesses permitted).
              if ('h3A0 <= csrno && csrno <= 'h3FF) csr_read_val = 0;
              else if (`CSR_MHPMEVENT3 <= csrno && csrno <= 12'h33f) begin
                 if (csrno <= `CSR_MHPMEVENT3 + (`HPM_COUNTERS - 1))
                    csr_read_val = csr_mhpmevent[csrno - `CSR_MHPMEVENT3];
                 else
                    csr_read_val = 0;
              end else if (`CSR_MHPMCOUNTER3 <= csrno && csrno <= 12'hb1f) begin
                 if (csrno <= `CSR_MHPMCOUNTER3 + (`HPM_COUNTERS - 1))
                    csr_read_val = csr_mhpmcounter[csrno - `CSR_MHPMCOUNTER3];
                 else
                    csr_read_val = 0;
              end else if (`CSR_HPMCOUNTER3 <= csrno && csrno <= 12'hc1f) begin
                 if (csrno <= `CSR_HPMCOUNTER3 + (`HPM_COUNTERS - 1))
                    csr_read_val = csr_mhpmcounter[csrno - `CSR_HPMCOUNTER3];
                 else
                    csr_read_val = 0;
              end
              else case (csrno)
                `CSR_FFLAGS:    csr_read_val = {59'd0, fflags};
                `CSR_FRM:       csr_read_val = {61'd0, frm};
                `CSR_FCSR:      csr_read_val = {56'd0, frm, fflags};
                `CSR_SSTATUS:
                  csr_read_val = {sd, 29'd0,            uxl, 12'd0,  // 63:20
                                                    mxr, sum, 1'd0,  // 19:17
                                  xs,   fs,         4'd0,      spp,  // 16: 8
                                  2'd0, spie, upie, 2'd0, sie, uie}; //  7: 0
                `CSR_SIE:       csr_read_val = csr_mie & 14'h2222;
                `CSR_STVEC:     csr_read_val = csr_stvec;
                `CSR_SCOUNTEREN:csr_read_val = csr_scounteren;
                `CSR_SSCRATCH:  csr_read_val = csr_sscratch;
                `CSR_SEPC:      csr_read_val = csr_sepc;
                `CSR_SCAUSE:    csr_read_val = csr_scause;
                `CSR_STVAL:     csr_read_val = csr_stval;
                `CSR_SIP:       csr_read_val = csr_mip & csr_mideleg;
                `CSR_SCOUNTOVF: csr_read_val = csr_scountovf_read_val;

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
                // F/D are NOT implemented yet — advertising them causes OS/test
                // F/D bits ON since FP Phase 2 (regfile + bit-ops + FL*/FS*).
                // Arithmetic FP insns (FADD/FMUL/FCVT/FEQ/...) will still trap
                // illegal until Phase 4 lands. Accepted: OpenSBI/Linux/riscv-tests
                // that issue arithmetic will break temporarily.
                `CSR_MISA:     csr_read_val = 64'h800000000014112d;
                // Hardwired 1 0100 0001 0001 0010 1101
                //    ZY XWV U TSRQ PONM LKJI HGFE DCBA
                //           U  S      M    I F  DC A
                //    SUIMAFDC
                `CSR_MEDELEG:  csr_read_val = csr_medeleg;
                `CSR_MIDELEG:  csr_read_val = csr_mideleg;
                `CSR_MIE:      csr_read_val = csr_mie;
                `CSR_MTVEC:    csr_read_val = csr_mtvec;
                `CSR_MCOUNTEREN: csr_read_val = csr_mcounteren;
                `CSR_MCOUNTINHIBIT: csr_read_val = csr_mcountinhibit & `HPM_INHIBIT_MASK;
                `CSR_MCYCLECFG: csr_read_val = csr_mcyclecfg;
                `CSR_MINSTRETCFG: csr_read_val = csr_minstretcfg;
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
                `CSR_MIG_MIN:  csr_read_val = {32'd0, csr_mig_min};
                `CSR_MIG_MAX:  csr_read_val = {32'd0, csr_mig_max};
                `CSR_MIG_TOTAL:csr_read_val = csr_mig_total;
                `CSR_MIG_COUNT:csr_read_val = csr_mig_count;
                `CSR_MIG_TIMEOUTS:csr_read_val = csr_mig_timeouts;
                `CSR_MIG_TO_PC:   csr_read_val = csr_mig_to_pc;
                `CSR_MIG_TO_TVAL: csr_read_val = csr_mig_to_tval;
                `CSR_MIG_TO_STATE:csr_read_val = csr_mig_to_state;
                `CSR_MIG_TO_CAUSE:csr_read_val = csr_mig_to_cause;
                `CSR_MIG_TO_ADDR: csr_read_val = csr_mig_to_addr;
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
              if (!csr_access_failure) begin
                 if (csrno == `CSR_CYCLE && !counter_access_allowed(5'd0))
                    csr_access_failure = 1;
                 else if (csrno == `CSR_TIME && !counter_access_allowed(5'd1))
                    csr_access_failure = 1;
                 else if (csrno == `CSR_INSTRET && !counter_access_allowed(5'd2))
                    csr_access_failure = 1;
                 else if (`CSR_HPMCOUNTER3 <= csrno && csrno <= 12'hc1f &&
                          !counter_access_allowed({1'b0, csrno[4:0]}))
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
              else if (`CSR_MHPMEVENT3 <= csrno && csrno <= 12'h33f) begin
                 if (csrno <= `CSR_MHPMEVENT3 + (`HPM_COUNTERS - 1))
                    csr_mhpmevent[csrno - `CSR_MHPMEVENT3] <= csr_write_val;
              end else if (`CSR_MHPMCOUNTER3 <= csrno && csrno <= 12'hb1f) begin
                 if (csrno <= `CSR_MHPMCOUNTER3 + (`HPM_COUNTERS - 1))
                    csr_mhpmcounter[csrno - `CSR_MHPMCOUNTER3] <= csr_write_val;
              end
              else case (csrno)
                // fcsr: fflags aliased at [4:0], frm aliased at [7:5].
                // Writing any of these is an implicit "FP state touched"
                // event, so we mark FS=Dirty at the same time.
                `CSR_FFLAGS: begin fflags      = csr_write_val[4:0];      fs = 3; end
                `CSR_FRM:    begin frm         = csr_write_val[2:0];      fs = 3; end
                `CSR_FCSR:   begin {frm,fflags}= csr_write_val[7:0];      fs = 3; end
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
                `CSR_SIE:       csr_mie    = csr_write_val & 14'h2222 | csr_mie & ~14'h2222;
                `CSR_STVEC:     csr_stvec  = csr_write_val;
                `CSR_SCOUNTEREN:csr_scounteren = csr_write_val & `HPM_COUNTER_MASK;
                `CSR_SSCRATCH:  csr_sscratch = csr_write_val;
                `CSR_SEPC:      csr_sepc   = csr_write_val & ~1;
                `CSR_SCAUSE:    csr_scause = csr_write_val;
                `CSR_STVAL:     csr_stval  = csr_write_val;
                `CSR_SIP:       begin
                   // Only SSIP (bit 1) is writable via SIP; SEIP/STIP are read-only
                   if (csr_mideleg[13]) lcofip = csr_write_val[13];
                   if (csr_mideleg[1]) ssip = csr_write_val[1];
                   if (csr_mideleg[0]) usip = csr_write_val[0];
                end
                `CSR_SATP: begin
                   if (prv == 1 && tvm) begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = insn;
                      state <= `S_EXCEPTION;
                   end else begin
                     case (csr_op)
                       `CSR_OP_COPY: csr_satp_write_val = csr_arg;
                       `CSR_OP_OR:   csr_satp_write_val = csr_satp | csr_arg;
                       default:      csr_satp_write_val = csr_satp & ~csr_arg;
                     endcase

                     if (csr_satp_write_val[63:60] == 4'd0 ||
                         csr_satp_write_val[63:60] == 4'd8) begin
                       // WARL: only Bare and Sv39 are supported; unsupported
                       // MODE values cause the entire write to have no effect.
                       csr_satp = csr_satp_write_val;
                     end
                   end
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
                `CSR_MCOUNTEREN: csr_mcounteren = csr_write_val & `HPM_COUNTER_MASK;
                `CSR_MCOUNTINHIBIT: csr_mcountinhibit = csr_write_val & `HPM_INHIBIT_MASK;
                `CSR_MCYCLECFG: csr_mcyclecfg = csr_write_val;
                `CSR_MINSTRETCFG: csr_minstretcfg = csr_write_val;
                `CSR_MSCRATCH: csr_mscratch = csr_write_val;
                `CSR_MEPC:     csr_mepc     = csr_write_val & ~1;
                `CSR_MCAUSE:   csr_mcause   = csr_write_val;
                `CSR_MTVAL:    csr_mtval    = csr_write_val;
                `CSR_MIP:      begin
                   // MEIP/SEIP (bits 11,9) are read-only, driven by PLIC
                   // MTIP/MSIP (bits 7,3) are read-only, driven by CLINT
                   lcofip = csr_write_val[13];
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
                `CSR_MCYCLE:   csr_mcycle   <= csr_write_val;
                `CSR_MINSTRET: csr_minstret <= csr_write_val;
                // Any write to any mig_* CSR clears all four to their initial
                // sentinels (fresh measurement window).  The written value is
                // ignored; this is the "clear stats" knob for the monitor.
                `CSR_MIG_MIN,
                `CSR_MIG_MAX,
                `CSR_MIG_TOTAL,
                `CSR_MIG_COUNT,
                `CSR_MIG_TIMEOUTS,
                `CSR_MIG_TO_PC,
                `CSR_MIG_TO_TVAL,
                `CSR_MIG_TO_STATE,
                `CSR_MIG_TO_CAUSE,
                `CSR_MIG_TO_ADDR: begin
                   csr_mig_min      <= 32'hFFFFFFFF;
                   csr_mig_max      <= 0;
                   csr_mig_total    <= 0;
                   csr_mig_count    <= 0;
                   csr_mig_timeouts <= 0;
                   csr_mig_to_pc    <= 0;
                   csr_mig_to_tval  <= 0;
                   csr_mig_to_state <= 0;
                   csr_mig_to_cause <= 0;
                   csr_mig_to_addr  <= 0;
                end
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

           exe_add <= csr_read_val;
           exe_sext32 <= 0;
           if (csr_access_failure) begin
              cause = `TRAP_ILLEGAL_INSTRUCTION;
              tval = insn;

              state <= `S_EXCEPTION;
           end

           // Any CSR write may change interrupt-enable/pending state
           // (sstatus/mstatus/sie/mie/mip/mideleg). pre_intr_pending
           // is a FF sampled the cycle BEFORE S_FETCH1 from current CSR
           // FF values, so it is stale for one cycle after any CSR write.
           // Reuse just_xret as a generic one-cycle suppress flag.
           just_xret <= 1;
        end

        `S_EXCEPTION: begin
`ifdef SIMULATE
`ifdef VERBOSE
           $display("%05d  ** Exception, cause %x, pc %x, tval %x, prv %d", $time, cause, pc, tval, prv);
           $fflush(0);
`endif
`endif

           prv_at_trap = prv;  // capture before the mutation below

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
           // Report insn=0 whenever no instruction actually retired: async
           // interrupts, and instruction-side faults (misaligned/access/page)
           // where the fetch never completed. Matches simmerv's convention.
           cosim_retire(
               pc,
               npc,
               (cause_intr || cause == `TRAP_INSTRUCTION_ADDRESS_MISALIGNED ||
                              cause == `TRAP_INSTRUCTION_ACCESS_FAULT ||
                              cause == `TRAP_INSTRUCTIONPAGE_FAULT)
                   ? 32'd0 : insn,
               8'd0,                         // no writeback on trap
               8'd0,
               {6'd0, prv_at_trap},
               8'd1,                         // trapped
               32'd0,
               64'd0,
               {cause_intr, 51'd0, cause},   // architectural mcause/scause form
               tval,
               clint_mtime_prev,
               clint_mtimecmp,
               csr_mepc,
               {7'd0, seip}
           );
`endif
`ifdef PC_TRACE
           if (dbg_armed && !dbg_busy) begin
              dbg_busy    <= 1;
              dbg_pc      <= pc;
              dbg_mode    <= prv_at_trap;
              dbg_is_trap <= 1;
              dbg_cause   <= cause[5:0];
              dbg_pos     <= 0;
           end
`endif
           just_trapped <= 1;

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
           // Registering here also gives the SRAM a synchronous read port.
           if (ptw_from_dram)
              pte_latch <= dram_latched;
           else
              pte_latch <= ptw_pte_addr[3] ? {mem0[mem_addr0], mem1[mem_addr1]}
                                            : {mem1[mem_addr1], mem0[mem_addr0]};
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
                        dram_addr <= mem_addr[30:3];
                        dram_read       <= 1;
                        state           <= (ptw_return == `S_FETCH2) ?
                                           `S_DRAM_FETCH_WAIT : `S_DRAM_FETCH_HALF_WAIT;
                     end else begin
                        fetch_from_dram <= 0;
                        mem_addr0  <= mem_addr[`MEM_SIZE_LG2-1:4] + mem_addr[3];
                        mem_addr1  <= mem_addr[`MEM_SIZE_LG2-1:4];
                        // Route through S_FETCH1B to capture the synchronous
                        // SRAM output before fetch assembly.
                        fetch_latch_half <= ptw_return == `S_FETCH2_HALF;
                        state      <= `S_FETCH1B;
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
              ptw_level <= ptw_level - 1;
              case (ptw_level)
                2: ptw_pte_addr <= {8'd0, aligned[53:10], 12'd0} + {52'd0, ptw_va[29:21], 3'd0};
                1: ptw_pte_addr <= {8'd0, aligned[53:10], 12'd0} + {52'd0, ptw_va[20:12], 3'd0};
                default: ptw_pte_addr <= 0;
              endcase
              state <= `S_PTW_LAUNCH;
           end
        end

        `S_PTW_LAUNCH: begin
           if (ptw_pte_addr[31] && ptw_pte_addr[63:`MEM_SIZE_LG2] != `MEM_BASEADDR >> `MEM_SIZE_LG2) begin
              ptw_from_dram <= 1;
              dram_addr     <= ptw_pte_addr[30:3];
              dram_read     <= 1;
              state         <= `S_DRAM_PTW_WAIT;
           end else begin
              ptw_from_dram <= 0;
              mem_addr0     <= ptw_pte_addr[`MEM_SIZE_LG2-1:4] + ptw_pte_addr[3];
              mem_addr1     <= ptw_pte_addr[`MEM_SIZE_LG2-1:4];
              state         <= `S_PTW_READ;
           end
        end

        `S_FETCH2_HALF: begin
           // Second half of cross-page or cross-doubleword instruction fetch.
           // Reassemble the 32-bit fetch word and then decode based on its actual
           // low 2 bits, so both compressed and uncompressed cases work.
           translated <= 0;
           if (fetch_from_dram)
              // pc+2 is at byte 0 of the next 8B chunk (dram_latched was updated)
              aligned = {64'bx, dram_latched};
           else
              // pc+2 is page-aligned (0x...000), so bit 3 is 0
              aligned = {mem_data1_q, mem_data0_q};
           insn = {aligned[15:0], insn_half};
           rd = insn`insn_rd;
           case (insn[1:0])
             0: {rs1,rs2} = {{2'd1,insn[9:7]}, {2'd1,insn[4:2]}};
             1: {rs1,rs2} = {insn[11:7],       {2'd1,insn[4:2]}};
             2: {rs1,rs2} = {insn[11:7],       insn[6:2]};
             3: {rs1,rs2} = {insn`insn_rs1,    insn`insn_rs2};
           endcase
           if (insn[1:0] == 1 && insn[15])
             rs1 = {2'd1,insn[9:7]};
           if (insn[1:0] == 2 && (insn[15:13] == 3'b001 || insn[15:14] == 2'b01))
             rs1 = 2; // sp
           if (insn[1:0] == 2 && 5 <= insn[15:13])
             rs1 = 2; // sp
           if ((insn & 'he003) == 0)
             rs1 = 2; // sp

           shamt = insn[25:20];

           write_back_register = 0;
           state <= `S_RF2;  // rs1/rs2 already decoded here; skip S_RF
        end

        `S_DRAM_FETCH_WAIT: if (dram_readdatavalid) begin
           dram_latched            <= dram_readdata;
           dram_latched_next       <= dram_readdata_next;
           dram_latched_next_valid <= dram_readdata_next_valid;
           state                   <= `S_FETCH2;
        end

        `S_DRAM_FETCH_HALF_WAIT: if (dram_readdatavalid) begin
           dram_latched            <= dram_readdata;
           dram_latched_next       <= dram_readdata_next;
           dram_latched_next_valid <= dram_readdata_next_valid;
           state                   <= `S_FETCH2_HALF;
        end

        `S_DRAM_PTW_WAIT: if (dram_readdatavalid) begin
           dram_latched <= dram_readdata;
           state        <= `S_PTW_READ;
        end

        `S_DRAM_LOAD_WAIT: if (dram_readdatavalid) begin
           if ({1'b0, mem_addr[2:0]} + (1 << (load_size_lg2 & 3)) > 8) begin
              if (dram_readdata_next_valid) begin : dram_load_cross_cached
                 reg [127:0] combo;
                 combo = {dram_readdata_next, dram_readdata} >> (mem_addr[2:0] * 8);
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
                 state <= do_atomic ? `S_AMO : `S_FETCH1;
              end else begin
                 // Access crosses a cache-line boundary and the second line
                 // missed during the parallel lookup; request it only now.
                 dram_latched    <= dram_readdata;
                 dram_addr <= mem_addr[30:3] + 1;
                 dram_read       <= 1;
                 state           <= `S_DRAM_LOAD2_WAIT;
              end
           end else begin
              aligned = dram_readdata >> (mem_addr[2:0] * 8);
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
              reg [127:0] combo;
              combo = {dram_readdata, dram_latched} >> (mem_addr[2:0] * 8);
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
           // Issue the first write now that the master is idle
           dram_write <= 1;
           state      <= `S_DRAM_STORE_RESP_ARM;
        end

        `S_DRAM_STORE2: if (dram_write_ready) begin
           dram_addr       <= dram2_addr;
           dram_writedata  <= dram2_data_part;
           dram_wstrb      <= dram2_wstrb;
           dram_write      <= 1;
           dram_store_split <= 0;
           state           <= `S_DRAM_STORE_RESP_ARM;
        end

        `S_DRAM_STORE_RESP_ARM: begin
           state <= `S_DRAM_STORE_RESP_WAIT;
        end

        `S_DRAM_STORE_RESP_WAIT: if (dram_write_done) begin
           state <= dram_store_split ? `S_DRAM_STORE2 : `S_FETCH1;
        end

      endcase

      // Bus timeout: fault if an external bus access doesn't respond
      begin : bus_timeout_logic
         reg bus_waiting;
         bus_waiting = state == `S_DRAM_FETCH_WAIT || state == `S_DRAM_FETCH_HALF_WAIT ||
                       state == `S_DRAM_LOAD_WAIT  || state == `S_DRAM_LOAD2_WAIT ||
                       state == `S_DRAM_PTW_WAIT   ||
                       state == `S_DRAM_STORE_WAIT || state == `S_DRAM_STORE2 ||
                       state == `S_DRAM_STORE_RESP_WAIT || state == `S_DRAM_STORE_RESP_ARM ||
                       state == `S_MMIO_ALIGN;
         bus_timeout_expired <= bus_waiting && &bus_timeout_ctr;
         if (bus_waiting) begin
            bus_timeout_ctr <= bus_timeout_ctr + 1;
            case (state)
              `S_DRAM_FETCH_WAIT, `S_DRAM_FETCH_HALF_WAIT: begin
                 bus_timeout_cause <= `TRAP_INSTRUCTION_ACCESS_FAULT;
                 bus_timeout_tval  <= pc;
              end
              `S_DRAM_STORE_WAIT, `S_DRAM_STORE2, `S_DRAM_STORE_RESP_WAIT, `S_DRAM_STORE_RESP_ARM: begin
                 bus_timeout_cause <= `TRAP_STORE_ACCESS_FAULT;
                 bus_timeout_tval  <= mem_addr;
              end
              `S_DRAM_PTW_WAIT: begin
                 bus_timeout_cause <= ptw_access == 0 ? `TRAP_INSTRUCTION_ACCESS_FAULT :
                                      ptw_access == 2 || ptw_access == 3 ? `TRAP_STORE_ACCESS_FAULT :
                                      `TRAP_LOAD_ACCESS_FAULT;
                 bus_timeout_tval <= ptw_va;
              end
              default: begin // S_DRAM_LOAD_WAIT, S_DRAM_LOAD2_WAIT, S_MMIO_ALIGN
                 bus_timeout_cause <= `TRAP_LOAD_ACCESS_FAULT;
                 bus_timeout_tval  <= mem_addr;
              end
            endcase
            if (bus_timeout_expired) begin
               bus_timeout_ctr     <= 0;
               bus_timeout_expired <= 0;
               csr_mig_timeouts <= csr_mig_timeouts + 1;
               // Late R beats from an abandoned read are silently swallowed by
               // the AXI master (it gates dram_readdatavalid on bus_waiting), so
               // no separate "abandon" handshake is needed.
               cause_intr = 0;
               cause = bus_timeout_cause;
               tval = bus_timeout_tval;
               // Latch context of the FIRST timeout in this measurement
               // window (don't overwrite on aftershock faults in the trap
               // handler).  Cleared when csr_mig_timeouts is cleared.
               if (csr_mig_timeouts == 0) begin
                  csr_mig_to_pc    <= pc;
                  csr_mig_to_tval  <= bus_timeout_tval;
                  csr_mig_to_state <= {59'd0, state};
                  csr_mig_to_cause <= {52'd0, bus_timeout_cause};
                  csr_mig_to_addr  <= {33'd0, dram_addr, 3'd0};
               end
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

         // MIG latency stats: measure cycles in any S_DRAM_* wait state.
         // Counter runs while bus_waiting; on falling edge, fold into stats.
         mig_prev_waiting <= bus_waiting;
         if (bus_waiting) begin
            mig_latency_ctr <= mig_latency_ctr + 1;
         end else begin
            mig_latency_ctr <= 0;
            if (mig_prev_waiting) begin
               csr_mig_count <= csr_mig_count + 1;
               csr_mig_total <= csr_mig_total + {32'd0, mig_latency_ctr};
               if (mig_latency_ctr < csr_mig_min) csr_mig_min <= mig_latency_ctr;
               if (mig_latency_ctr > csr_mig_max) csr_mig_max <= mig_latency_ctr;
            end
         end
      end

      if (core_reset_now) begin
         core_reset_pending <= 0;
         state <= `S_FETCH1;
         csr_minstret <= 0;
         csr_mcycle <= 0;
         clint_mtime <= 0;
         write_back_register <= 0;
         npc <= `RESET_PC;
         pre_npc <= `RESET_PC;
         bus_timeout_ctr <= 0;
         bus_timeout_expired <= 0;
         bus_timeout_tval <= 0;
         bus_timeout_cause <= 0;
         // Note: mig_* stats CSRs intentionally NOT reset here, so they
         // survive a soft reset (e.g. key[1] on the FPGA board).  Initial
         // values come from the reg declarations (FPGA config-time init).
         // Clear them with a CSR write to CSR_MIG_COUNT (see CSR write block).

         // Architectural state needed to cleanly resume execution from
         // RESET_PC in M-mode with paging off (e.g. after a soft reset
         // that returns to the monitor from a running Linux workload).
         prv              <= 3;
         csr_satp         <= 0;
         csr_mie          <= 0;
         csr_mideleg      <= 0;
         csr_medeleg      <= 0;
         csr_mcounteren   <= 0;
         csr_scounteren   <= 0;
         csr_mcountinhibit <= 0;
         csr_mcyclecfg    <= 0;
         csr_minstretcfg  <= 0;
         lcofip           <= 0;
         csr_mtvec        <= 0;
         csr_stvec        <= 0;
         mie              <= 0;
         sie              <= 0;
         uie              <= 0;
         mpie             <= 0;
         spie             <= 0;
         upie             <= 0;
         mpp              <= 0;
         spp              <= 0;
         mprv             <= 0;
         pre_intr_pending <= 0;
`ifdef USE_CVFPU
         pre_fp_rnd_mode  <= 0;
         pre_fp_rmode_ok  <= 1'b1;
`endif
         just_trapped     <= 0;
         just_xret        <= 0;
         fetch_from_dram  <= 0;
         dram_latched_next_valid <= 0;
         translated       <= 0;
         dram_read        <= 0;
         dram_write       <= 0;
         dram_store_split <= 0;
         ptw_from_dram    <= 0;
         mig_latency_ctr  <= 0;
         mig_prev_waiting <= 0;
         for (hpm_i = 0; hpm_i < `HPM_COUNTERS; hpm_i = hpm_i + 1) begin
            csr_mhpmcounter[hpm_i] <= 0;
            csr_mhpmevent[hpm_i] <= 0;
         end
      end
   end

   assign dram_readdatavalid = dram_readdatavalid_r;
   assign dram_readdata      = dram_readdata_r;
   assign dram_readdata_next = dram_readdata_next_r;
   assign dram_readdata_next_valid = dram_readdata_next_valid_r;
   assign dram_write_ready   = cache_idle && axi_write_ready && !axi_read;
   assign dram_write_done    = axi_write_done;

   always @(posedge clock) begin
      cache_bank0_rd_data <= cache_bank0[cache_bank0_rd_idx];
      cache_bank1_rd_data <= cache_bank1[cache_rd_idx];
      cache_bank2_rd_data <= cache_bank2[cache_rd_idx];
      cache_bank3_rd_data <= cache_bank3[cache_rd_idx];
      cache_bank4_rd_data <= cache_bank4[cache_rd_idx];
      cache_bank5_rd_data <= cache_bank5[cache_rd_idx];
      cache_bank6_rd_data <= cache_bank6[cache_rd_idx];
      cache_bank7_rd_data <= cache_bank7[cache_rd_idx];

      if (cache_state == CACHE_FILL_WAIT && axi_readdatavalid) begin
         case (cache_fill_beat)
           3'd0: cache_bank0[cache_fill_idx] <= axi_readdata;
           3'd1: cache_bank1[cache_fill_idx] <= axi_readdata;
           3'd2: cache_bank2[cache_fill_idx] <= axi_readdata;
           3'd3: cache_bank3[cache_fill_idx] <= axi_readdata;
           3'd4: cache_bank4[cache_fill_idx] <= axi_readdata;
           3'd5: cache_bank5[cache_fill_idx] <= axi_readdata;
           3'd6: cache_bank6[cache_fill_idx] <= axi_readdata;
           3'd7: cache_bank7[cache_fill_idx] <= axi_readdata;
         endcase
      end
   end

   always @(posedge clock) begin
      dram_readdatavalid_r <= 0;
      dram_readdata_next_valid_r <= 0;
      axi_read  <= 0;
      axi_write <= 0;
      cache_tag_wr_en <= 0;

      case (cache_state)
        CACHE_IDLE: begin
           if (dram_read) begin
              cache_addr          <= cache_dram_addr;
              cache_req_tag       <= cache_dram_tag;
              cache_req_next_tag  <= cache_dram_next_tag;
              cache_req_bank      <= dram_addr[2:0];
              cache_req_next_bank <= dram_addr[2:0] + 3'd1;
              cache_req_same_line <= dram_addr[2:0] != 3'd7;
              cache_rd_idx        <= cache_dram_idx;
              cache_bank0_rd_idx  <= dram_addr[2:0] == 3'd7 ? cache_dram_next_idx
                                                             : cache_dram_idx;
              cache_next_rd_idx   <= cache_dram_next_idx;
              cache_state         <= CACHE_TAG_READ;
           end else if (dram_write && axi_write_ready && !axi_read) begin
              cache_tag_wr_en   <= 1;
              cache_tag_wr_idx  <= cache_dram_idx;
              cache_tag_wr_data <= {1'b0, cache_dram_tag};
              axi_write_addr <= dram_addr;
              axi_write_data <= dram_writedata;
              axi_write_strb <= dram_wstrb;
              axi_write      <= 1;
           end
        end

        CACHE_TAG_READ: begin
           cache_state <= CACHE_TAG_CHECK;
        end

        CACHE_TAG_CHECK: begin
           cache_lookup_hit <=
                cache_tag_rd_data[`CACHE_TAG_BITS] &&
                cache_tag_rd_data[`CACHE_TAG_BITS-1:0] == cache_req_tag;
           cache_lookup_next_hit <=
                cache_tag_next_rd_data[`CACHE_TAG_BITS] &&
                cache_tag_next_rd_data[`CACHE_TAG_BITS-1:0] == cache_req_next_tag;
           cache_lookup_data <= cache_selected_bank_data(cache_req_bank);
           cache_lookup_next_data <= cache_selected_bank_data(cache_req_next_bank);
           cache_lookup_next_valid <=
                cache_req_same_line ||
                (cache_tag_next_rd_data[`CACHE_TAG_BITS] &&
                 cache_tag_next_rd_data[`CACHE_TAG_BITS-1:0] == cache_req_next_tag);
           cache_state <= CACHE_HIT_RESP;
        end

        CACHE_HIT_RESP: begin
           if (cache_lookup_hit) begin
              dram_readdata_r <= cache_lookup_data;
              dram_readdata_next_r <= cache_lookup_next_data;
              dram_readdata_next_valid_r <= cache_lookup_next_valid;
              dram_readdatavalid_r <= 1;
              cache_state <= CACHE_IDLE;
           end else begin
              cache_fill_base <= {cache_addr[63:6], 6'd0};
              cache_fill_beat <= 0;
              cache_fill_return_data <= 0;
              cache_fill_next_data <= 0;
              cache_state <= CACHE_FILL_REQ;
           end
        end

        CACHE_FILL_REQ: begin
           axi_read_addr <= cache_fill_base[30:3] + {25'd0, cache_fill_beat};
           axi_read      <= 1;
           cache_state   <= CACHE_FILL_WAIT;
        end

        CACHE_FILL_WAIT: begin
           if (axi_readdatavalid) begin
              if (cache_fill_beat == cache_req_bank)
                 cache_fill_return_data <= axi_readdata;
              if (cache_fill_beat == cache_req_next_bank)
                 cache_fill_next_data <= axi_readdata;
              if (cache_fill_beat == 3'd7) begin
                 cache_tag_wr_en   <= 1;
                 cache_tag_wr_idx  <= cache_fill_idx;
                 cache_tag_wr_data <= {1'b1, cache_fill_tag};
                 dram_readdata_r  <= cache_req_bank == 3'd7 ? axi_readdata : cache_fill_return_data;
                 dram_readdata_next_r <= cache_req_same_line
                                          ? (cache_req_next_bank == 3'd7 ? axi_readdata
                                                                         : cache_fill_next_data)
                                          : cache_lookup_next_data;
                 dram_readdata_next_valid_r <= cache_req_same_line || cache_lookup_next_hit;
                 dram_readdatavalid_r <= 1;
                 cache_state      <= CACHE_IDLE;
              end else begin
                 cache_fill_beat <= cache_fill_beat + 1;
                 cache_state     <= CACHE_FILL_REQ;
              end
           end
        end

        default: cache_state <= CACHE_IDLE;
      endcase

      if (core_reset_now) begin
         cache_state <= CACHE_IDLE;
         dram_readdatavalid_r <= 0;
         dram_readdata_next_valid_r <= 0;
         axi_read <= 0;
         axi_write <= 0;
         cache_tag_wr_en <= 0;
      end
   end

   // ----- AXI4 master to DDR4 -----
   // Single read in flight, single write in flight.  arsize/awsize fixed at
   // 8B; arlen/awlen=0 (one beat).  The physical cache above this block
   // emits axi_read/axi_write pulses for line fills and write-through stores.
   assign axi_readdatavalid = axi_readdatavalid_r;
   assign axi_readdata      = rdata_r;
   assign axi_write_ready   = !aw_busy && !w_busy && !b_busy;
   assign axi_write_done    = b_busy && m_axi_bvalid;

   always @(posedge clock) begin
      axi_readdatavalid_r <= 0;

      // AR / R
      if (axi_read) begin
         ar_addr_r <= axi_read_addr;
         ar_busy   <= 1;
         r_busy    <= 1;
      end
      if (ar_busy && m_axi_arready) ar_busy <= 0;
      if (r_busy && m_axi_rvalid) begin
         rdata_r              <= m_axi_rdata;
         r_busy               <= 0;
         axi_readdatavalid_r  <= 1;
      end

      // AW / W / B
      if (axi_write && axi_write_ready) begin
         aw_addr_r <= axi_write_addr;
         w_data_r  <= axi_write_data;
         w_strb_r  <= axi_write_strb;
         aw_busy   <= 1;
         w_busy    <= 1;
         b_busy    <= 1;
      end
      if (aw_busy && m_axi_awready) aw_busy <= 0;
      if (w_busy  && m_axi_wready ) w_busy  <= 0;
      if (b_busy  && m_axi_bvalid ) b_busy  <= 0;

      if (core_reset_now) begin
         ar_busy <= 0; r_busy <= 0;
         aw_busy <= 0; w_busy <= 0; b_busy <= 0;
         axi_readdatavalid_r <= 0;
      end
   end

   assign m_axi_arvalid = ar_busy;
   assign m_axi_araddr  = {ar_addr_r, 3'b000};
   assign m_axi_arlen   = 8'd0;
   assign m_axi_arsize  = 3'b011;       // 8 bytes
   assign m_axi_arburst = 2'b01;        // INCR
   assign m_axi_arid    = 3'b000;
   assign m_axi_arlock  = 1'b0;
   assign m_axi_arcache = 4'b0011;
   assign m_axi_arprot  = 3'b000;
   assign m_axi_arqos   = 4'b0000;
   assign m_axi_rready  = 1'b1;

   assign m_axi_awvalid = aw_busy;
   assign m_axi_awaddr  = {aw_addr_r, 3'b000};
   assign m_axi_awlen   = 8'd0;
   assign m_axi_awsize  = 3'b011;
   assign m_axi_awburst = 2'b01;
   assign m_axi_awid    = 3'b000;
   assign m_axi_awlock  = 1'b0;
   assign m_axi_awcache = 4'b0011;
   assign m_axi_awprot  = 3'b000;
   assign m_axi_awqos   = 4'b0000;
   assign m_axi_wvalid  = w_busy;
   assign m_axi_wdata   = w_data_r;
   assign m_axi_wstrb   = w_strb_r;
   assign m_axi_wlast   = 1'b1;
   assign m_axi_bready  = 1'b1;

endmodule


module smolrv64_sdpram #(
   parameter ADDR_WIDTH = 14,
   parameter DATA_WIDTH = 64
) (
   input  wire                  clock,
   input  wire [ADDR_WIDTH-1:0] rd_addr,
   output wire [DATA_WIDTH-1:0] rd_data,
   input  wire                  wr_en,
   input  wire [ADDR_WIDTH-1:0] wr_addr,
   input  wire [DATA_WIDTH-1:0] wr_data
);
`ifdef SMOLRV64_USE_XPM
   wire [0:0] wr_en_vec = wr_en;

   xpm_memory_sdpram #(
      .ADDR_WIDTH_A        ( ADDR_WIDTH ),
      .ADDR_WIDTH_B        ( ADDR_WIDTH ),
      .AUTO_SLEEP_TIME     ( 0 ),
      .BYTE_WRITE_WIDTH_A  ( DATA_WIDTH ),
      .CASCADE_HEIGHT      ( 0 ),
      .CLOCKING_MODE       ( "common_clock" ),
      .ECC_MODE            ( "no_ecc" ),
      .MEMORY_INIT_FILE    ( "none" ),
      .MEMORY_INIT_PARAM   ( "0" ),
      .MEMORY_OPTIMIZATION ( "true" ),
      .MEMORY_PRIMITIVE    ( "block" ),
      .MEMORY_SIZE         ( DATA_WIDTH * (1 << ADDR_WIDTH) ),
      .MESSAGE_CONTROL     ( 0 ),
      .READ_DATA_WIDTH_B   ( DATA_WIDTH ),
      .READ_LATENCY_B      ( 1 ),
      .READ_RESET_VALUE_B  ( "0" ),
      .RST_MODE_A          ( "SYNC" ),
      .RST_MODE_B          ( "SYNC" ),
      .SIM_ASSERT_CHK      ( 0 ),
      .USE_EMBEDDED_CONSTRAINT( 0 ),
      .USE_MEM_INIT        ( 1 ),
      .WAKEUP_TIME         ( "disable_sleep" ),
      .WRITE_DATA_WIDTH_A  ( DATA_WIDTH ),
      .WRITE_MODE_B        ( "read_first" )
   ) xpm_memory_sdpram_inst (
      .dbiterrb       ( ),
      .doutb          ( rd_data ),
      .sbiterrb       ( ),
      .addra          ( wr_addr ),
      .addrb          ( rd_addr ),
      .clka           ( clock ),
      .clkb           ( clock ),
      .dina           ( wr_data ),
      .ena            ( 1'b1 ),
      .enb            ( 1'b1 ),
      .injectdbiterra ( 1'b0 ),
      .injectsbiterra ( 1'b0 ),
      .regceb         ( 1'b1 ),
      .rstb           ( 1'b0 ),
      .sleep          ( 1'b0 ),
      .wea            ( wr_en_vec )
   );
`elsif SYNTHESIS
   wire [0:0] wr_en_vec = wr_en;

   xpm_memory_sdpram #(
      .ADDR_WIDTH_A        ( ADDR_WIDTH ),
      .ADDR_WIDTH_B        ( ADDR_WIDTH ),
      .AUTO_SLEEP_TIME     ( 0 ),
      .BYTE_WRITE_WIDTH_A  ( DATA_WIDTH ),
      .CASCADE_HEIGHT      ( 0 ),
      .CLOCKING_MODE       ( "common_clock" ),
      .ECC_MODE            ( "no_ecc" ),
      .MEMORY_INIT_FILE    ( "none" ),
      .MEMORY_INIT_PARAM   ( "0" ),
      .MEMORY_OPTIMIZATION ( "true" ),
      .MEMORY_PRIMITIVE    ( "block" ),
      .MEMORY_SIZE         ( DATA_WIDTH * (1 << ADDR_WIDTH) ),
      .MESSAGE_CONTROL     ( 0 ),
      .READ_DATA_WIDTH_B   ( DATA_WIDTH ),
      .READ_LATENCY_B      ( 1 ),
      .READ_RESET_VALUE_B  ( "0" ),
      .RST_MODE_A          ( "SYNC" ),
      .RST_MODE_B          ( "SYNC" ),
      .SIM_ASSERT_CHK      ( 0 ),
      .USE_EMBEDDED_CONSTRAINT( 0 ),
      .USE_MEM_INIT        ( 1 ),
      .WAKEUP_TIME         ( "disable_sleep" ),
      .WRITE_DATA_WIDTH_A  ( DATA_WIDTH ),
      .WRITE_MODE_B        ( "read_first" )
   ) xpm_memory_sdpram_inst (
      .dbiterrb       ( ),
      .doutb          ( rd_data ),
      .sbiterrb       ( ),
      .addra          ( wr_addr ),
      .addrb          ( rd_addr ),
      .clka           ( clock ),
      .clkb           ( clock ),
      .dina           ( wr_data ),
      .ena            ( 1'b1 ),
      .enb            ( 1'b1 ),
      .injectdbiterra ( 1'b0 ),
      .injectsbiterra ( 1'b0 ),
      .regceb         ( 1'b1 ),
      .rstb           ( 1'b0 ),
      .sleep          ( 1'b0 ),
      .wea            ( wr_en_vec )
   );
`else
   (* ram_style = "block" *) reg [DATA_WIDTH-1:0] ram[0:(1 << ADDR_WIDTH)-1];
   reg [DATA_WIDTH-1:0] rd_data_r = 0;
   integer ram_init_i;

   initial begin
      for (ram_init_i = 0; ram_init_i < (1 << ADDR_WIDTH); ram_init_i = ram_init_i + 1)
         ram[ram_init_i] = 0;
   end

   assign rd_data = rd_data_r;

   always @(posedge clock) begin
      rd_data_r <= ram[rd_addr];
      if (wr_en)
         ram[wr_addr] <= wr_data;
   end
`endif
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

// Floating-point register file. Structurally identical to the int regfile,
// but f0 is a real register (no x0 hardwire; gating stays at the instance),
// and the array is initialized to 0 (no rf.hex seed).
module fregfile(input wire         clock,
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
   reg  [63:0] fregfile[31:0];
   integer i;
   initial for (i = 0; i < 32; i = i + 1) fregfile[i] = 0;

   always @(posedge clock) begin
`ifndef ASYNC_RF
      read_data_0 <= fregfile[read_addr_0];
      read_data_1 <= fregfile[read_addr_1];
`endif

      if (write_valid) fregfile[write_addr] <= write_data;
   end

`ifdef ASYNC_RF
   assign read_data_0 = fregfile[read_addr_0];
   assign read_data_1 = fregfile[read_addr_1];
`endif
endmodule
