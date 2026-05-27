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
                          .mem_clock            (clock),
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

`ifndef SMOLRV64_BUILD_STAMP
`define SMOLRV64_BUILD_STAMP 64'h0
`endif

module smolrv64(input wire        clock,
                input wire        mem_clock,
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
`define CSR_SENVCFG    12'h10a
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
`define CSR_VHPR_READS 12'hfca
`define CSR_VHPR_WRITES 12'hfcb
`define CSR_VHPR_READ_HITS 12'hfcc
`define CSR_VHPR_READ_MISSES 12'hfcd
`define CSR_VHPR_WRITE_HITS 12'hfce
`define CSR_VHPR_WRITE_MISSES 12'hfcf
`define CSR_VHPR_FILLS 12'hfd0
`define CSR_VHPR_VICTIM_EVICTS 12'hfd1
`define CSR_VHPR_DIRTY_VICTIM_EVICTS 12'hfd2
`define CSR_VHPR_ALIAS_EVICTS 12'hfd3
`define CSR_VHPR_DIRTY_ALIAS_EVICTS 12'hfd4
`define CSR_VHPR_FLUSH_EVICTS 12'hfd5
`define CSR_VHPR_DIRTY_FLUSH_EVICTS 12'hfd6
`define CSR_VHPR_CBO_PROBES 12'hfd7
`define CSR_VHPR_PTW_PROBES 12'hfd8
`define CSR_VHPR_EPOCH_BUMPS 12'hfd9
`define CSR_VHPR_EPOCH_ROLLOVERS 12'hfda
`define CSR_VHPR_EPOCH 12'hfdb
`define CSR_BUILD_STAMP 12'hfde

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
`define S_RF             2  // legacy unused encoding
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
`define S_RF2                  23  // wait for BRAM regfile read after rs1/rs2 launch
`define S_EXECUTE2             24  // complete write_back_value from pre-computed exe_add
`define S_PTW_PROCESS          25  // process PTE latched from mem1 in S_PTW_READ
`define S_RF3                  26  // register BRAM output (s1_bram/s2_bram) into s1/s2 flip-flops
`define S_FETCH1B              27  // register SRAM mem0/mem1 output before S_FETCH2 reads insn
`define S_LOAD_LATCH           28  // legacy load-align landing state
`define S_CBO_EXEC             29  // execute translated cache-block operation
`define S_CBO_WAIT             30  // wait for cache-block operation completion
`define S_STORE_COMMIT         31  // commit a store after translation/routing decision
`define S_STORE_BRAM_WRITE     32  // full-word writeback after BRAM store read/modify
`define S_CVFPU_ISSUE          33  // present a CVFPU operation until accepted
`define S_CVFPU_WAIT           34  // wait for a CVFPU result
`define S_CVFPU_FMA_RF2        35  // wait for rs3 FP regfile read
`define S_CVFPU_FMA_RF3        36  // issue CVFPU fused multiply-add/subtract
`define S_TLB_LOOKUP           37  // wait for direct-mapped TLB RAM outputs
`define S_TLB_CHECK            38  // compare direct-mapped TLB entries
`define S_TLB_HIT              39  // route the registered TLB hit result
`define S_TLB_START_FETCH      40  // start instruction-fetch translation after fetch miss decision
`define S_TLB_START_FETCH_HALF 41  // start cross-page instruction-fetch translation
`define S_PTW_START            42  // start PTW after TLB miss decision
`define S_DRAM_STORE_RESP_WAIT 43  // wait for an issued DRAM store to fully drain
`define S_DRAM_STORE_RESP_ARM  44  // absorb one cycle so AXI busy flags see a new write
`define S_FETCH2_DRAM          45  // latch instruction from DRAM fetch without fetch-source mux
`define S_FETCH_BUF_CHECK      46  // fallback register for fetch-buffer hit decision
`define S_FETCH_BUF_USE        47  // fallback consume for registered fetch-buffer hit
`define S_MULDIV_START         48  // initialize iterative M-extension datapath
`define S_TLB_DECIDE           49  // consume registered TLB hit decision
`define S_FETCH_REQ            50  // issue registered PC/context fetch request
`define S_FRONTEND_MISS_WAIT   51  // wait for speculative frontend cache miss after backend retire
`define S_HANDLE_CSR_COMMIT    52  // retire registered CSR readback after CSR side effects
`define S_FP_INT_COMMIT        53  // retire staged FP result for integer register writes
`define S_LOCAL_LOAD           54  // commit local UART/CLINT/PLIC load data after address dispatch
`define S_TLB_INSERT           55  // commit staged PTW result into the TLB, then route translated PA
`define S_BRANCH_RESOLVE       56  // resolve branch/JALR from registered RF operands
`define S_BUS_TIMEOUT          57  // enter a bus-timeout exception after timeout context is registered
`define S_LAST_STATE           57  // update state register width accordingly

// f_state: the free-running frontend FSM. Drives the cache-hit fetch path
// (FETCH_REQ -> FETCH_BUF_CHECK -> FETCH_BUF_USE -> enqueue to rf_decode_*)
// independently of the backend `state` register, so frontend work overlaps
// with backend long-latency states (CVFPU, MULDIV, AMO, DRAM, etc).
//
// Cache miss / TLB miss / DRAM fetch / cross-doubleword fetch still escalate
// to the backend FSM for this patch — those resources are shared with the
// load/store path and require arbitration that is out of scope here.
`define F_IDLE                  0  // no fetch in flight
`define F_FETCH_BUF_CHECK       1  // latch frontend_rsp_* into f_latched_*
`define F_FETCH_BUF_USE         2  // on hit, enqueue rf_decode; on miss, hand to backend

// ex_state: scaffolding for the back-half pipeline split. Eventually owns
// S_EXECUTE / S_EXECUTE2 (and the various memory/EX states) so an instruction
// can be in EX while the next is in RF.
`define EX_IDLE                 0  // EX stage empty; nothing in flight
`define EX_EXECUTE2             2  // compute write_back_value from exe_add / exe_sext32

`define MULDIV_MUL             4'd0
`define MULDIV_MULH            4'd1
`define MULDIV_MULHSU          4'd2
`define MULDIV_MULHU           4'd3
`define MULDIV_DIV             4'd4
`define MULDIV_DIVU            4'd5
`define MULDIV_REM             4'd6
`define MULDIV_REMU            4'd7
`define MULDIV_MULW            4'd8
`define MULDIV_DIVW            4'd9
`define MULDIV_DIVUW           4'd10
`define MULDIV_REMW            4'd11
`define MULDIV_REMUW           4'd12

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
// (single mem_addr adder).
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
`define HPM_EVENT_TLB_LOOKUP       16'h0300
`define HPM_EVENT_TLB_HIT          16'h0301
`define HPM_EVENT_TLB_MISS         16'h0302
`define HPM_EVENT_TLB_HIT_4K       16'h0303
`define HPM_EVENT_TLB_HIT_2M       16'h0304
`define HPM_EVENT_TLB_INSERT_4K    16'h0305
`define HPM_EVENT_TLB_INSERT_2M    16'h0306
`define HPM_EVENT_TLB_EVICT_4K     16'h0307
`define HPM_EVENT_TLB_EVICT_2M     16'h0308
`define HPM_EVENT_TLB_UNCACHED_1G  16'h0309
`define HPM_EVENT_TLB_UNCACHED_NAPOT 16'h030a
`define HPM_EVENT_PTW_LEAF_4K      16'h0310
`define HPM_EVENT_PTW_LEAF_2M      16'h0311
`define HPM_EVENT_PTW_LEAF_1G      16'h0312
`define HPM_EVENT_PTW_LEAF_NAPOT   16'h0313

`define REGION_UART    3'd0
`define REGION_CLINT   3'd1
`define REGION_PLIC    3'd2
`define REGION_BRAM    3'd3
`define REGION_MMIO    3'd4
`define REGION_DRAM    3'd5
`define REGION_ILLEGAL 3'd6

   reg [5:0]   state = `S_FETCH1; // XXX We should set this on reset
   reg [1:0]   f_state = `F_IDLE; // free-running frontend FSM; see F_* defines
   reg [1:0]   ex_state = `EX_IDLE; // back-half EX FSM; see EX_* defines
   reg         f_consumed_hit;    // 1-cycle pulse: F_FETCH_BUF_USE took the hit
   // Set by retire_linear_fetch / retire_prepared_fetch / retire_redirect_fetch
   // (and other real retires) before transitioning to S_FETCH1. Gates retire
   // bookkeeping (csr_minstret, cosim, pc<=npc) so it only fires on real
   // retires, not the extra S_FETCH1 visits introduced by F_FETCH_BUF_USE
   // routing fetches through the queue.
   reg         retire_now_q = 0;
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
   localparam [63:0] MEM_BASEADDR_VALUE = `MEM_BASEADDR;
   localparam TLB_CTX_BITS = 6;
   localparam TLB_ASID_BITS = 10;
   localparam CACHE_PERM_BITS = 5; // {physical, U, X, W, R}
   localparam TLB_SATP_KEY_BITS = TLB_ASID_BITS;
   localparam FRONTEND_EPOCH_BITS = 2;
   localparam VHPR_EPOCH_BITS = 2;
   localparam TLB_2M_TAG_BITS = 18;
   localparam TLB_2M_PBASE_BITS = 43;
   localparam TLB_2M_DATA_BITS = TLB_2M_TAG_BITS + TLB_2M_PBASE_BITS +
                                 TLB_SATP_KEY_BITS + TLB_CTX_BITS +
                                 CACHE_PERM_BITS;
   localparam TLB_4K_TAG_BITS = 27;
   localparam TLB_4K_PBASE_BITS = 52;
   localparam TLB_4K_DATA_BITS = TLB_4K_TAG_BITS + TLB_4K_PBASE_BITS +
                                 TLB_SATP_KEY_BITS + TLB_CTX_BITS +
                                 CACHE_PERM_BITS;
   localparam TLB_CTX_LSB = 0;
   localparam TLB_PERM_LSB = TLB_CTX_LSB + TLB_CTX_BITS;
   localparam TLB_SATP_KEY_LSB = TLB_PERM_LSB + CACHE_PERM_BITS;
   localparam TLB_2M_PBASE_LSB = TLB_SATP_KEY_LSB + TLB_SATP_KEY_BITS;
   localparam TLB_2M_TAG_LSB = TLB_2M_PBASE_LSB + TLB_2M_PBASE_BITS;
   localparam TLB_4K_PBASE_LSB = TLB_SATP_KEY_LSB + TLB_SATP_KEY_BITS;
   localparam TLB_4K_TAG_LSB = TLB_4K_PBASE_LSB + TLB_4K_PBASE_BITS;
`ifndef CACHE_INDEX_BITS
`define CACHE_INDEX_BITS 9 // VHPR L1: 64 KiB, 2 ways, 512 64-byte lines/way
`endif
`define CACHE_WAYS      2
`define CACHE_LINES     (1 << `CACHE_INDEX_BITS)
`ifndef TLB_2M_INDEX_BITS
`define TLB_2M_INDEX_BITS 8
`endif
`ifndef TLB_4K_INDEX_BITS
`define TLB_4K_INDEX_BITS 10
`endif
`define TLB_2M_ENTRIES (1 << `TLB_2M_INDEX_BITS)
`define TLB_4K_ENTRIES (1 << `TLB_4K_INDEX_BITS)
`define TLB_ENTRIES (`TLB_2M_ENTRIES + `TLB_4K_ENTRIES)
`define CACHE_PHYS_BITS 31 // Cached DRAM addresses are {33'd0, dram_addr[27:0], 3'b000}.
`define CACHE_LINE_OFFSET_BITS 6
`define CACHE_LINE_WORDS 8
`define CACHE_PAGE_OFFSET_BITS 12
`define CACHE_COLOR_BITS 3
`define CACHE_PAGE_LINE_BITS (`CACHE_PAGE_OFFSET_BITS - `CACHE_LINE_OFFSET_BITS)
`define CACHE_PHYS_TAG_BITS (`CACHE_PHYS_BITS - `CACHE_PAGE_OFFSET_BITS)
`define CACHE_VTAG_BITS (64 - `CACHE_INDEX_BITS - `CACHE_LINE_OFFSET_BITS)
`define CACHE_PTAG_LSB 0
`define CACHE_VTAG_LSB (`CACHE_PTAG_LSB + `CACHE_PHYS_TAG_BITS)
`define CACHE_ASID_LSB (`CACHE_VTAG_LSB + `CACHE_VTAG_BITS)
`define CACHE_PERM_LSB (`CACHE_ASID_LSB + TLB_ASID_BITS)
`define CACHE_EPOCH_LSB (`CACHE_PERM_LSB + CACHE_PERM_BITS)
`define CACHE_VALID_BIT (`CACHE_EPOCH_LSB + VHPR_EPOCH_BITS)
`define CACHE_DIRTY_BIT (`CACHE_VALID_BIT + 1)
`define CACHE_META_BITS (`CACHE_DIRTY_BIT + 1)
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


   reg  [63:0] mem_addr;
   reg  [63:0] mem_va;
   reg  [TLB_ASID_BITS-1:0] mem_asid;
   reg  [CACHE_PERM_BITS-1:0] mem_perm;
   reg  [TLB_CTX_BITS-1:0] mem_ctx;
   reg  [15:0] mem_wr_mask;
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
   (* max_fanout = 32 *) reg [63:0] s1 = 0; // flip-flop copy of s1_bram; captured in S_RF3, used in S_EXECUTE
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
   (* max_fanout = 32 *) reg [63:0] pre_exe_b = 0; // second operand
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

   // Execute request boundary. S_RF3 asserts this after registering operands
   // and predecode outputs; S_EXECUTE clears it when accepted.
   reg         execute_req_valid = 0;
   reg  [63:0] execute_req_pc = `RESET_PC;
   reg  [63:0] execute_req_next_pc = `RESET_PC;
   reg  [63:0] execute_req_predicted_pc = `RESET_PC;
   reg  [ 1:0] execute_req_prv = 0;
   reg  [FRONTEND_EPOCH_BITS-1:0] execute_req_epoch = 0;
   (* max_fanout = 16 *) reg [31:0] execute_req_insn = 0;
   reg  [ 1:0] execute_req_prediction_kind = 0;
   reg  [ 4:0] execute_req_rd = 0;
   reg  [ 4:0] execute_req_rs1 = 0;
   reg  [ 4:0] execute_req_rs2 = 0;
   reg  [ 5:0] execute_req_shamt = 0;
   wire        ex_accept_ready = !execute_req_valid;
   wire [63:0] ex_pc = execute_req_pc;
   wire [63:0] ex_next_pc = execute_req_next_pc;
   wire [63:0] ex_predicted_pc = execute_req_predicted_pc;
   wire [ 1:0] ex_prv = execute_req_prv;
   wire [FRONTEND_EPOCH_BITS-1:0] ex_epoch = execute_req_epoch;
   wire [31:0] ex_insn = execute_req_insn;
   wire [ 1:0] ex_prediction_kind = execute_req_prediction_kind;
   wire [ 4:0] ex_rd = execute_req_rd;
   wire [ 4:0] ex_rs1 = execute_req_rs1;
   wire [ 4:0] ex_rs2 = execute_req_rs2;
   wire [ 5:0] ex_shamt = execute_req_shamt;
   reg         execute_res_valid = 0;

   // FP load completions latch their boxed result into write_back_fp_value
   // before retire. Keeping writeback data independent of pre_mem_fp lets an
   // early-launched next instruction predecode without changing the bypassed
   // value from the retiring FP load.
   wire [63:0] fp_writeback_data = write_back_fp_value;

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


   (* max_fanout = 32 *) reg [63:0] npc = `RESET_PC; // XXX We should set this on reset
   reg  [63:0] pre_npc = `RESET_PC;
   reg  [63:0] pre_jalr_target = `RESET_PC;
   reg  [63:0] pre_branch_target = `RESET_PC;
   reg         pre_branch_taken = 0;

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
   reg  [27:0]  dram2_addr;          // 8B-doubleword addr for 2nd half of split store
   reg  [63:0]  dram2_va;            // virtual address for split-store second beat
   reg  [TLB_ASID_BITS-1:0] dram2_asid;
   reg  [CACHE_PERM_BITS-1:0] dram2_perm;
   reg  [TLB_CTX_BITS-1:0] dram2_ctx;
   reg  [63:0]  dram2_data_part;     // overflow bytes for split store
   reg  [ 7:0]  dram2_wstrb;         // AXI wstrb for split-store second beat
   reg          dram_store_split;    // 1 = second beat pending after DRAM_STORE_WAIT
   reg          ptw_direct_read = 0; // physical PTW read bypasses VHPR L1
   reg  [27:0]  ptw_direct_addr = 0;
   reg          ptw_direct_probe_pending = 0;
   reg          ptw_direct_wait_probe = 0;
   reg          ptw_direct_pending = 0;
   reg          ptw_direct_wait_bram = 0;
   reg          ptw_direct_wait_axi = 0;
   reg [`MEM_SIZE_LG2-5:0] ptw_direct_bram_word_idx = 0;
   reg          ptw_direct_bram_word_bank = 0;
   reg          ptw_direct_readdatavalid_r = 0;
   reg  [63:0]  ptw_direct_readdata_r = 0;

   // Frontend instruction fetch window.  The old global FSM still launches
   // TLB/cache slow paths; the frontend module owns instruction alignment,
   // the small fetch window, and the registered hit result.
   reg          frontend_buf_flush = 0;
   reg          frontend_buf_fill = 0;
   reg  [63:0]  frontend_buf_fill_base_va = 0;
   reg  [ 1:0]  frontend_buf_fill_prv = 0;
   reg  [TLB_ASID_BITS-1:0] frontend_buf_fill_asid = 0;
   reg  [127:0] frontend_buf_fill_data = 0;
   wire         frontend_rsp_addr_hit;
   wire         frontend_rsp_full_insn_hit;
   wire         frontend_rsp_hit;
   wire [31:0]  frontend_rsp_insn;
   wire [ 3:0]  frontend_rsp_offset;
   // Free-running frontend's fetch-buffer latch. Written by case(f_state)
   // when f_state == F_FETCH_BUF_CHECK; read by backend's S_FETCH_BUF_USE.
   // Replaces the old backend-owned fetch_buf_latched_* set.
   //
   // The cmd_pc/prv/epoch are also latched here so F_FETCH_BUF_USE consumes
   // a self-consistent (pc, insn) pair: backend can update frontend_cmd_*
   // between F_FETCH_BUF_CHECK and F_FETCH_BUF_USE, and without latching we
   // would feed latch_frontend_decode_pending current frontend_cmd_pc + stale
   // f_latched_insn (different PCs) — a hang.
   reg          f_latched_hit = 0;
   reg  [31:0]  f_latched_insn = 0;
   reg  [ 3:0]  f_latched_offset = 0;
   reg  [63:0]  f_latched_next_pc = `RESET_PC;
   reg  [ 1:0]  f_latched_prediction_kind = 0;
   reg  [63:0]  f_latched_cmd_pc = `RESET_PC;
   reg  [ 1:0]  f_latched_cmd_prv = 3;
   reg  [FRONTEND_EPOCH_BITS-1:0] f_latched_cmd_epoch = 0;
   wire         f_latched_full_insn_hit =
      f_latched_insn[1:0] != 2'b11 ||
      f_latched_offset <= 4'd12;
   wire [63:0]  fetch_buf_fill_base_va;
   wire         fetch_buf_fill_page_ok;
   wire [FRONTEND_EPOCH_BITS-1:0] frontend_rsp_active_epoch;
   wire [63:0]  frontend_rsp_predicted_next_pc;
   wire [ 1:0]  frontend_rsp_prediction_kind;
   wire [`CACHE_META_BITS-1:0] icache_way0_tag_rd_data;
   wire [`CACHE_META_BITS-1:0] icache_way1_tag_rd_data;
   wire [`CACHE_META_BITS-1:0] icache_way0_tag_next_rd_data;
   wire [`CACHE_META_BITS-1:0] icache_way1_tag_next_rd_data;
   wire [63:0] icache_way0_bank_rd_data [0:7];
   wire [63:0] icache_way1_bank_rd_data [0:7];
   wire [63:0] icache_way0_bank0_rd_data;
   wire [63:0] icache_way0_bank1_rd_data;
   wire [63:0] icache_way0_bank2_rd_data;
   wire [63:0] icache_way0_bank3_rd_data;
   wire [63:0] icache_way0_bank4_rd_data;
   wire [63:0] icache_way0_bank5_rd_data;
   wire [63:0] icache_way0_bank6_rd_data;
   wire [63:0] icache_way0_bank7_rd_data;
   wire [63:0] icache_way1_bank0_rd_data;
   wire [63:0] icache_way1_bank1_rd_data;
   wire [63:0] icache_way1_bank2_rd_data;
   wire [63:0] icache_way1_bank3_rd_data;
   wire [63:0] icache_way1_bank4_rd_data;
   wire [63:0] icache_way1_bank5_rd_data;
   wire [63:0] icache_way1_bank6_rd_data;
   wire [63:0] icache_way1_bank7_rd_data;

   assign icache_way0_bank_rd_data[0] = icache_way0_bank0_rd_data;
   assign icache_way0_bank_rd_data[1] = icache_way0_bank1_rd_data;
   assign icache_way0_bank_rd_data[2] = icache_way0_bank2_rd_data;
   assign icache_way0_bank_rd_data[3] = icache_way0_bank3_rd_data;
   assign icache_way0_bank_rd_data[4] = icache_way0_bank4_rd_data;
   assign icache_way0_bank_rd_data[5] = icache_way0_bank5_rd_data;
   assign icache_way0_bank_rd_data[6] = icache_way0_bank6_rd_data;
   assign icache_way0_bank_rd_data[7] = icache_way0_bank7_rd_data;
   assign icache_way1_bank_rd_data[0] = icache_way1_bank0_rd_data;
   assign icache_way1_bank_rd_data[1] = icache_way1_bank1_rd_data;
   assign icache_way1_bank_rd_data[2] = icache_way1_bank2_rd_data;
   assign icache_way1_bank_rd_data[3] = icache_way1_bank3_rd_data;
   assign icache_way1_bank_rd_data[4] = icache_way1_bank4_rd_data;
   assign icache_way1_bank_rd_data[5] = icache_way1_bank5_rd_data;
   assign icache_way1_bank_rd_data[6] = icache_way1_bank6_rd_data;
   assign icache_way1_bank_rd_data[7] = icache_way1_bank7_rd_data;

   // Frontend command/result boundary.  The backend writes frontend_cmd_* when
   // it wants the frontend to probe or restart at a PC/context; the frontend
   // returns frontend_rsp_* for the registered command while the old FSM still
   // owns slow-path translation and refill sequencing.
   reg          frontend_cmd_valid = 0;
   reg  [63:0]  frontend_cmd_pc = `RESET_PC;
   reg  [ 1:0]  frontend_cmd_prv = 3;
   reg  [TLB_ASID_BITS-1:0] frontend_cmd_asid = 0;
   reg          frontend_cmd_fast_ready = 0;
   reg          frontend_cmd_speculative = 0;
   reg          frontend_cmd_spec_miss_ready = 0;
   reg  [FRONTEND_EPOCH_BITS-1:0] frontend_cmd_epoch = 0;
   reg  [FRONTEND_EPOCH_BITS-1:0] fetch_epoch = 0;
   reg          frontend_redirect_valid = 0;
   reg  [63:0] frontend_redirect_pc = `RESET_PC;
   reg  [ 1:0] frontend_redirect_prv = 3;
   reg  [FRONTEND_EPOCH_BITS-1:0] frontend_redirect_epoch = 0;
   // Speculative frontend cache miss.  The single global FSM still owns TLB
   // and ordinary fetch misses; this side buffer only overlaps physical
   // cacheable misses with long non-memory backend states.
   reg          frontend_miss_valid = 0;
   reg          frontend_miss_done = 0;
   reg          frontend_flush_this_cycle = 0;
   reg  [63:0]  frontend_miss_pc = `RESET_PC;
   reg  [ 1:0]  frontend_miss_prv = 3;
   reg  [TLB_ASID_BITS-1:0] frontend_miss_asid = 0;
   reg  [FRONTEND_EPOCH_BITS-1:0] frontend_miss_epoch = 0;
   reg  [63:0]  frontend_miss_data = 0;
   reg  [63:0]  frontend_miss_next_data = 0;
   reg          frontend_miss_next_valid = 0;
   localparam [1:0] FRONTEND_MISS_WAIT_CONSUME  = 2'd0,
                    FRONTEND_MISS_WAIT_EXCEPTION = 2'd2;
   reg  [ 1:0]  frontend_miss_wait_action = FRONTEND_MISS_WAIT_CONSUME;

   // Register/decode request boundary. The frontend can now run a little
   // ahead of retirement on fetch-window hits; retirement consumes the oldest
   // matching entry and launches the BRAM register-file read.
   localparam RF_DECODE_QUEUE_BITS = 3;
   localparam RF_DECODE_QUEUE_DEPTH = 1 << RF_DECODE_QUEUE_BITS;
   localparam [RF_DECODE_QUEUE_BITS:0] RF_DECODE_QUEUE_DEPTH_COUNT =
      (1 << RF_DECODE_QUEUE_BITS);
   reg  [RF_DECODE_QUEUE_BITS-1:0] rf_decode_head = 0;
   reg  [RF_DECODE_QUEUE_BITS-1:0] rf_decode_tail = 0;
   reg  [RF_DECODE_QUEUE_BITS:0]   rf_decode_count = 0;
   reg  [63:0]  rf_decode_pc_q [0:RF_DECODE_QUEUE_DEPTH-1];
   reg  [63:0]  rf_decode_next_pc_q [0:RF_DECODE_QUEUE_DEPTH-1];
   reg  [63:0]  rf_decode_predicted_pc_q [0:RF_DECODE_QUEUE_DEPTH-1];
   reg  [31:0]  rf_decode_insn_q [0:RF_DECODE_QUEUE_DEPTH-1];
   reg  [ 1:0]  rf_decode_prv_q [0:RF_DECODE_QUEUE_DEPTH-1];
   reg  [FRONTEND_EPOCH_BITS-1:0] rf_decode_epoch_q [0:RF_DECODE_QUEUE_DEPTH-1];
   reg  [ 1:0]  rf_decode_prediction_kind_q [0:RF_DECODE_QUEUE_DEPTH-1];
   reg          rf_decode_from_dram_q [0:RF_DECODE_QUEUE_DEPTH-1];
   reg  [ 4:0]  rf_decode_rd_q [0:RF_DECODE_QUEUE_DEPTH-1];
   reg  [ 4:0]  rf_decode_rs1_q [0:RF_DECODE_QUEUE_DEPTH-1];
   reg  [ 4:0]  rf_decode_rs2_q [0:RF_DECODE_QUEUE_DEPTH-1];
   reg  [ 5:0]  rf_decode_shamt_q [0:RF_DECODE_QUEUE_DEPTH-1];
   wire         rf_decode_valid = rf_decode_count != 0;
   wire         rf_decode_full = rf_decode_count == RF_DECODE_QUEUE_DEPTH_COUNT;
   wire [63:0]  rf_decode_pc = rf_decode_pc_q[rf_decode_head];
   wire [63:0]  rf_decode_next_pc = rf_decode_next_pc_q[rf_decode_head];
   wire [63:0]  rf_decode_predicted_pc = rf_decode_predicted_pc_q[rf_decode_head];
   wire [31:0]  rf_decode_insn = rf_decode_insn_q[rf_decode_head];
   wire [ 1:0]  rf_decode_prv = rf_decode_prv_q[rf_decode_head];
   wire [FRONTEND_EPOCH_BITS-1:0] rf_decode_epoch = rf_decode_epoch_q[rf_decode_head];
   wire [ 1:0]  rf_decode_prediction_kind = rf_decode_prediction_kind_q[rf_decode_head];
   wire         rf_decode_from_dram = rf_decode_from_dram_q[rf_decode_head];
   wire [ 4:0]  rf_decode_rd = rf_decode_rd_q[rf_decode_head];
   wire [ 4:0]  rf_decode_rs1 = rf_decode_rs1_q[rf_decode_head];
   wire [ 4:0]  rf_decode_rs2 = rf_decode_rs2_q[rf_decode_head];
   wire [ 5:0]  rf_decode_shamt = rf_decode_shamt_q[rf_decode_head];
   reg          rf_decode_pop_this_cycle = 0;
   reg          rf_decode_enqueue_this_cycle = 0;
   reg          rf_decode_prearm_block = 0;
   reg          rf_decode_prearmed = 0;
   reg          frontend_decode_pending_valid = 0;
   reg          frontend_decode_pending_drain = 0;
   reg  [63:0]  frontend_decode_pending_pc = `RESET_PC;
   reg  [63:0]  frontend_decode_pending_next_pc = `RESET_PC;
   reg  [63:0]  frontend_decode_pending_predicted_pc = `RESET_PC;
   reg  [31:0]  frontend_decode_pending_insn = 0;
   reg  [ 1:0]  frontend_decode_pending_prv = 0;
   reg  [FRONTEND_EPOCH_BITS-1:0] frontend_decode_pending_epoch = 0;
   reg  [ 1:0]  frontend_decode_pending_prediction_kind = 0;

   // ID/RF stage boundary. Dispatch accepts one decoded instruction into this
   // payload and launches the BRAM read; S_RF3 consumes it into EX.
   reg          id_valid = 0;
   reg          id_rf_ready = 0;
   reg  [63:0]  id_pc = `RESET_PC;
   reg  [63:0]  id_next_pc = `RESET_PC;
   reg  [63:0]  id_predicted_pc = `RESET_PC;
   reg  [ 1:0]  id_prv = 0;
   reg  [FRONTEND_EPOCH_BITS-1:0] id_epoch = 0;
   reg  [31:0]  id_insn = 0;
   reg  [ 1:0]  id_prediction_kind = 0;
   reg  [ 4:0]  id_rd = 0;
   reg  [ 4:0]  id_rs1 = 0;
   reg  [ 4:0]  id_rs2 = 0;
   reg  [ 5:0]  id_shamt = 0;
   wire         id_ex_fire = id_valid && id_rf_ready && ex_accept_ready;
   wire [63:0]  rf3_pc = id_pc;
   wire [63:0]  rf3_next_pc = id_next_pc;
   wire [63:0]  rf3_predicted_pc = id_predicted_pc;
   wire [ 1:0]  rf3_prv = id_prv;
   wire [FRONTEND_EPOCH_BITS-1:0] rf3_epoch = id_epoch;
   wire [31:0]  rf3_insn = id_insn;
   wire [ 1:0]  rf3_prediction_kind = id_prediction_kind;
   wire [ 4:0]  rf3_rd = id_rd;
   wire [ 4:0]  rf3_rs1 = id_rs1;
   wire [ 4:0]  rf3_rs2 = id_rs2;
   wire [ 5:0]  rf3_shamt = id_shamt;
   wire [63:0]  rf3_s1_value =
      (write_back_register != 0 && rf3_rs1 == write_back_register) ?
      write_back_value : s1_bram;
   wire [63:0]  rf3_s2_value =
      (write_back_register != 0 && rf3_rs2 == write_back_register) ?
      write_back_value : s2_bram;
   wire [63:0]  rf3_f1_value =
      (write_back_fp_valid && rf3_rs1 == write_back_fp_register) ?
      fp_writeback_data : f1_bram;
   wire [63:0]  rf3_f2_value =
      (write_back_fp_valid && rf3_rs2 == write_back_fp_register) ?
      fp_writeback_data : f2_bram;

   // VHPR write-back L1 for BRAM/DRAM.  The hit lookup is virtual
   // (ASID + virtual tag) while miss, writeback, and CBO reconciliation use
   // the stored physical tag to keep a single resident copy per physical line.
   localparam [4:0] CACHE_IDLE      = 5'd0;
   localparam [4:0] CACHE_TAG_READ  = 5'd1;
   localparam [4:0] CACHE_TAG_CHECK = 5'd2;
   localparam [4:0] CACHE_FILL_REQ  = 5'd3;
   localparam [4:0] CACHE_FILL_WAIT = 5'd4;
   localparam [4:0] CACHE_HIT_RESP  = 5'd5;
   localparam [4:0] CACHE_WB_REQ    = 5'd6;
   localparam [4:0] CACHE_WB_WAIT   = 5'd7;
   localparam [4:0] CACHE_WB_PREP   = 5'd8;
   localparam [4:0] CACHE_CBO_TAG_READ  = 5'd9;
   localparam [4:0] CACHE_CBO_TAG_CHECK = 5'd10;
   localparam [4:0] CACHE_CBO_RESP      = 5'd11;
   localparam [4:0] CACHE_BRAM_FILL_READ = 5'd12;
   localparam [4:0] CACHE_BRAM_WB_WRITE = 5'd13;
   localparam [4:0] CACHE_BRAM_FILL_CAPTURE = 5'd14;
   localparam [4:0] CACHE_BRAM_FILL_COMMIT = 5'd15;
   localparam [4:0] CACHE_HIT_WRITE = 5'd16;
   localparam [4:0] CACHE_TAG_WAIT = 5'd17;
   localparam [4:0] CACHE_CBO_TAG_WAIT = 5'd18;
   localparam [4:0] CACHE_WB_READ_WAIT = 5'd19;
   localparam [4:0] CACHE_PROBE_READ = 5'd20;
   localparam [4:0] CACHE_PROBE_WAIT = 5'd21;
   localparam [4:0] CACHE_PROBE_CHECK = 5'd22;
   localparam [4:0] CACHE_INVALIDATE = 5'd23;
   localparam [4:0] CACHE_FLUSH_READ = 5'd24;
   localparam [4:0] CACHE_FLUSH_WAIT = 5'd25;
   localparam [4:0] CACHE_FLUSH_CHECK = 5'd26;
   localparam [4:0] CACHE_FILL_LINE_WAIT = 5'd27;
   localparam [4:0] CACHE_FILL_LINE_INSTALL = 5'd28;
   localparam [4:0] CACHE_LAST_STATE = CACHE_FILL_LINE_INSTALL;

   reg [ 7:0] cache_bank_wr_en = 0;
   reg [`CACHE_INDEX_BITS-1:0] cache_bank_wr_idx = 0;
   reg                         cache_bank_wr_way = 0;
   reg [63:0] cache_bank_wr_data = 0;

   localparam [CACHE_PERM_BITS-1:0] CACHE_PERM_PHYS = 5'b1_1111;

   reg [ 4:0] cache_state = CACHE_IDLE;
   reg [63:0] cache_addr = 0;
   reg [63:0] cache_req_va = 0;
   reg [TLB_ASID_BITS-1:0] cache_req_asid = 0;
   reg [CACHE_PERM_BITS-1:0] cache_req_perm = CACHE_PERM_PHYS;
   reg [TLB_CTX_BITS-1:0] cache_req_ctx = 0;
   reg [63:0] cache_fill_base = 0;
   reg [63:0] cache_wb_base = 0;
   reg [ 2:0] cache_fill_beat = 0;
   reg [ 2:0] cache_wb_beat = 0;
   reg [ 2:0] cache_req_bank = 0;
   reg [ 2:0] cache_req_next_bank = 0;
   reg        cache_req_same_line = 0;
   reg        cache_req_write = 0;
   reg        cache_req_instr = 0;
   reg        cache_req_cbo = 0;
   reg [30:6] cache_req_line_addr = 0;
   reg [`CACHE_VTAG_BITS-1:0] cache_req_vtag = 0;
   reg [`CACHE_VTAG_BITS-1:0] cache_req_next_vtag = 0;
   reg [`CACHE_PHYS_TAG_BITS-1:0] cache_req_ptag = 0;
   reg [`CACHE_PHYS_TAG_BITS-1:0] cache_victim_ptag = 0;
   reg [`CACHE_INDEX_BITS-1:0] cache_victim_idx = 0;
   reg                         cache_victim_way = 0;
   reg                         cache_replace_way = 0;
   reg [63:0] cache_store_data = 0;
   reg [ 7:0] cache_store_strb = 0;
   reg [63:0] cache_fill_return_data = 0;
   reg [63:0] cache_fill_next_data = 0;
   reg [63:0] cache_lookup_data = 0;
   reg [63:0] cache_lookup_next_data = 0;
   reg        cache_lookup_hit = 0;
   reg        cache_lookup_hit_way = 0;
   reg        cache_lookup_next_hit = 0;
   reg        cache_lookup_next_hit_way = 0;
   reg        cache_lookup_next_valid = 0;
   reg        cache_lookup_dirty = 0;
   reg        cache_target_dirty = 0;
   reg        cache_target_valid = 0;
   reg [`CACHE_PHYS_TAG_BITS-1:0] cache_target_ptag = 0;
   reg [`CACHE_INDEX_BITS-1:0] cache_target_idx = 0;
   reg                         cache_target_way = 0;
   reg [2:0]  cache_probe_color = 0;
   reg [`CACHE_INDEX_BITS-1:0] cache_probe_idx = 0;
   reg        cache_probe_found = 0;
   reg        cache_probe_found_way = 0;
   reg [`CACHE_INDEX_BITS-1:0] cache_probe_found_idx = 0;
   reg        cache_probe_found_dirty = 0;
   reg        cache_probe_active = 0;
   reg        cache_wb_then_fill = 0;
   reg        cache_need_target_wb = 0;
   reg        cache_wb_after_cbo = 0;
   reg        cache_cbo_flush = 0;
   reg        cache_flush_req = 0;
   reg        cache_flush_ack = 0;
   wire       cache_flush_all = cache_flush_req != cache_flush_ack;
   reg [`CACHE_INDEX_BITS-1:0] cache_flush_idx = 0;
   reg        cache_flush_way = 0;
   reg        cache_wb_after_flush = 0;
   reg [30:6] cache_cbo_line_addr = 0;
   reg        cache_cbo_done_r = 0;
   reg [63:0] cache_bram_read_data_stage = 0;
   reg [63:0] cache_bram_fill_data = 0;
   reg        cache_bram_write_done = 0;
   reg [`MEM_SIZE_LG2-5:0] cache_bram_word_idx = 0;
   reg        cache_bram_word_bank = 0;
   reg [63:0] cache_bram_wb_data = 0;
   reg [63:0] dram_va = 0;
   reg [TLB_ASID_BITS-1:0] dram_asid = 0;
   reg [CACHE_PERM_BITS-1:0] dram_perm = CACHE_PERM_PHYS;
   reg [TLB_CTX_BITS-1:0] dram_ctx = 0;
   reg        dram_readdatavalid_r = 0;
   reg [63:0] dram_readdata_r = 0;
   reg [63:0] dram_readdata_next_r = 0;
   reg        dram_readdata_next_valid_r = 0;
   reg        dram_write_done_r = 0;
   reg        dram_instr = 0;
   wire       cache_idle = cache_state == CACHE_IDLE;
   wire       cache_cbo_done = cache_cbo_done_r;

   reg        mem_fill_req_valid = 0;
   wire       mem_fill_req_ready;
   reg [24:0] mem_fill_req_line_addr = 0;
   wire       mem_fill_rsp_valid;
   reg        mem_fill_rsp_ready = 0;
   wire [511:0] mem_fill_rsp_data;
   reg        mem_wb_req_valid = 0;
   wire       mem_wb_req_ready;
   reg [24:0] mem_wb_req_line_addr = 0;
   reg [511:0] mem_wb_req_line_data = 0;
   wire       mem_wb_rsp_valid;
   reg        mem_wb_rsp_ready = 0;
   reg        mem_read_req_valid = 0;
   wire       mem_read_req_ready;
   reg [27:0] mem_read_req_addr = 0;
   wire       mem_read_rsp_valid;
   reg        mem_read_rsp_ready = 0;
   wire [63:0] mem_read_rsp_data;
   wire       mem_engine_idle;
   wire       mem_fill_req_fire = mem_fill_req_valid && mem_fill_req_ready;
   wire       mem_wb_req_fire = mem_wb_req_valid && mem_wb_req_ready;
   wire       mem_read_req_fire = mem_read_req_valid && mem_read_req_ready;
   wire       cache_bram_fill_commit = cache_state == CACHE_BRAM_FILL_COMMIT;
   reg [511:0] cache_fill_line_data = 0;
   wire       cache_line_install = cache_state == CACHE_FILL_LINE_INSTALL;
   wire       cache_fill_data_valid = cache_line_install || cache_bram_fill_commit;
   wire [63:0] cache_fill_line_word = cache_fill_line_data[cache_fill_beat * 64 +: 64];
   wire [63:0] cache_fill_data = cache_bram_fill_commit ? cache_bram_fill_data
                                                        : cache_fill_line_word;
   wire [63:0] cache_bram_base_addr = {33'd0, MEM_BASEADDR_VALUE[30:0]};
   wire [63:0] cache_bram_window_mask = 64'hffff_ffff_ffff_ffff << `MEM_SIZE_LG2;
   wire       cache_fill_from_bram =
              ((cache_fill_base ^ cache_bram_base_addr) & cache_bram_window_mask) == 0;
   wire       cache_wb_to_bram =
              ((cache_wb_base ^ cache_bram_base_addr) & cache_bram_window_mask) == 0;
   wire [63:0] ptw_direct_addr64 = {33'd0, ptw_direct_addr, 3'b000};
   wire        ptw_direct_from_bram =
              ((ptw_direct_addr64 ^ cache_bram_base_addr) & cache_bram_window_mask) == 0;

   assign     core_reset_home = state == `S_FETCH1 && cache_state == CACHE_IDLE &&
                                !dram_read && !dram_write &&
                                !mem_fill_req_valid && !mem_wb_req_valid &&
                                !mem_read_req_valid &&
                                !ptw_direct_read && !ptw_direct_probe_pending &&
                                !ptw_direct_wait_probe && !ptw_direct_pending &&
                                !ptw_direct_wait_bram && !ptw_direct_wait_axi &&
                                mem_engine_idle;
   assign     core_reset_now  = (reset || core_reset_pending) && core_reset_home;

   wire       hpm_instret_pulse = state == `S_FETCH1 && retire_now_q;
   wire       hpm_cache_read_pulse = cache_state == CACHE_IDLE && dram_read;
   wire       hpm_cache_write_pulse = cache_state == CACHE_IDLE && dram_write;
   wire       hpm_cache_hit_pulse = cache_state == CACHE_HIT_RESP && cache_lookup_hit;
   wire       hpm_cache_miss_pulse = cache_state == CACHE_HIT_RESP && !cache_lookup_hit;
   wire       hpm_cache_fill_beat_pulse = cache_fill_data_valid;
   wire       hpm_cache_fill_line_pulse = hpm_cache_fill_beat_pulse && cache_fill_beat == 3'd7;
   wire       cache_wb_line_pulse = cache_state == CACHE_WB_REQ &&
                                    (cache_wb_to_bram || mem_wb_req_ready) &&
                                    cache_wb_beat == 3'd0;
   wire       hpm_axi_read_pulse = mem_fill_req_fire || mem_read_req_fire;
   wire       hpm_axi_write_pulse = mem_wb_req_fire;
   wire       hpm_bus_wait_cycle = state == `S_DRAM_FETCH_WAIT || state == `S_DRAM_FETCH_HALF_WAIT ||
                                   state == `S_DRAM_LOAD_WAIT  || state == `S_DRAM_LOAD2_WAIT ||
                                   state == `S_DRAM_PTW_WAIT   ||
                                   state == `S_DRAM_STORE_WAIT || state == `S_DRAM_STORE2 ||
                                   state == `S_DRAM_STORE_RESP_WAIT || state == `S_DRAM_STORE_RESP_ARM ||
                                   state == `S_MMIO_ALIGN;
   reg        hpm_instret_q = 0;
   reg        hpm_cache_read_q = 0;
   reg        hpm_cache_hit_q = 0;
   reg        hpm_cache_miss_q = 0;
   reg        hpm_cache_fill_line_q = 0;
   reg        hpm_cache_fill_beat_q = 0;
   reg        hpm_cache_write_q = 0;
   reg        hpm_axi_read_q = 0;
   reg        hpm_axi_write_q = 0;
   reg        hpm_bus_wait_q = 0;
   reg        hpm_tlb_lookup_q = 0;
   reg        hpm_tlb_hit_q = 0;
   reg        hpm_tlb_miss_q = 0;
   reg        hpm_tlb_hit_4k_q = 0;
   reg        hpm_tlb_hit_2m_q = 0;
   reg        hpm_tlb_insert_4k_q = 0;
   reg        hpm_tlb_insert_2m_q = 0;
   reg        hpm_tlb_evict_4k_q = 0;
   reg        hpm_tlb_evict_2m_q = 0;
   reg        hpm_tlb_uncached_1g_q = 0;
   reg        hpm_tlb_uncached_napot_q = 0;
   reg        hpm_ptw_leaf_4k_q = 0;
   reg        hpm_ptw_leaf_2m_q = 0;
   reg        hpm_ptw_leaf_1g_q = 0;
   reg        hpm_ptw_leaf_napot_q = 0;

`ifdef SIMULATE
   reg cache_trace_enabled = 0;
   reg cache_summary_enabled = 0;
   reg fetch_buf_summary_enabled = 0;
   reg state_summary_enabled = 0;
   integer state_summary_interval = 0;
   reg state_summary_interval_seen = 0;
   reg [63:0] state_summary_next = 0;
   reg [63:0] state_stat_total_cycles = 0;
   reg [63:0] state_stat_instret = 0;
   reg [63:0] state_stat_cycles [0:`S_LAST_STATE];
   reg [63:0] cache_state_stat_cycles [0:CACHE_LAST_STATE];
   reg [63:0] cache_stat_reads = 0;
   reg [63:0] cache_stat_writes = 0;
   reg [63:0] cache_stat_hits = 0;
   reg [63:0] cache_stat_misses = 0;
   reg [63:0] cache_stat_dirty_misses = 0;
   reg [63:0] cache_stat_fill_lines = 0;
   reg [63:0] cache_stat_wb_lines = 0;
   reg [63:0] fetch_buf_stat_hits = 0;
   reg [63:0] fetch_buf_stat_misses = 0;
   reg [63:0] ptw_stat_leaf_4k = 0;
   reg [63:0] ptw_stat_leaf_64k_napot = 0;
   reg [63:0] ptw_stat_leaf_2m = 0;
   reg [63:0] ptw_stat_leaf_1g = 0;
   reg [63:0] tlb_stat_lookups = 0;
   reg [63:0] tlb_stat_hits = 0;
   reg [63:0] tlb_stat_misses = 0;
   reg [63:0] tlb_stat_hits_4k = 0;
   reg [63:0] tlb_stat_hits_2m = 0;
   reg [63:0] tlb_stat_inserts_4k = 0;
   reg [63:0] tlb_stat_inserts_2m = 0;
   reg [63:0] tlb_stat_evicts_4k = 0;
   reg [63:0] tlb_stat_evicts_2m = 0;
   reg [63:0] tlb_stat_uncached_1g = 0;
   reg [63:0] tlb_stat_uncached_napot = 0;
   integer tlb_stat_entries_4k = 0;
   integer tlb_stat_entries_2m = 0;
   integer tlb_stat_entries_1g = 0;
   integer state_stat_i;

   function [8*24-1:0] state_name;
      input [5:0] s;
      begin
         case (s)
           `S_FETCH1:                state_name = "FETCH1";
           `S_FETCH2:                state_name = "FETCH2";
           `S_EXECUTE:               state_name = "EXECUTE";
           `S_EXCEPTION:             state_name = "EXCEPTION";
           `S_LOAD_ALIGN:            state_name = "LOAD_ALIGN";
           `S_MMIO_READ:             state_name = "MMIO_READ";
           `S_MMIO_ALIGN:            state_name = "MMIO_ALIGN";
           `S_AMO:                   state_name = "AMO";
           `S_STORE:                 state_name = "STORE";
           `S_HANDLE_CSR:            state_name = "HANDLE_CSR";
           `S_MUL_RUNNING:           state_name = "MUL_RUNNING";
           `S_DIV_RUNNING:           state_name = "DIV_RUNNING";
           `S_PTW_READ:              state_name = "PTW_READ";
           `S_PTW_LAUNCH:            state_name = "PTW_LAUNCH";
           `S_FETCH2_HALF:           state_name = "FETCH2_HALF";
           `S_DRAM_FETCH_WAIT:       state_name = "DRAM_FETCH_WAIT";
           `S_DRAM_LOAD_WAIT:        state_name = "DRAM_LOAD_WAIT";
           `S_DRAM_PTW_WAIT:         state_name = "DRAM_PTW_WAIT";
           `S_DRAM_STORE_WAIT:       state_name = "DRAM_STORE_WAIT";
           `S_DRAM_FETCH_HALF_WAIT:  state_name = "DRAM_FETCH_HALF_WAIT";
           `S_DRAM_LOAD2_WAIT:       state_name = "DRAM_LOAD2_WAIT";
           `S_DRAM_STORE2:           state_name = "DRAM_STORE2";
           `S_RF2:                   state_name = "RF2";
           `S_EXECUTE2:              state_name = "EXECUTE2";
           `S_PTW_PROCESS:           state_name = "PTW_PROCESS";
           `S_RF3:                   state_name = "RF3";
           `S_FETCH1B:               state_name = "FETCH1B";
           `S_LOAD_LATCH:            state_name = "LOAD_LATCH";
           `S_DRAM_STORE_RESP_WAIT:  state_name = "DRAM_STORE_RESP_WAIT";
           `S_DRAM_STORE_RESP_ARM:   state_name = "DRAM_STORE_RESP_ARM";
           `S_STORE_COMMIT:          state_name = "STORE_COMMIT";
           `S_STORE_BRAM_WRITE:      state_name = "STORE_BRAM_WRITE";
           `S_CVFPU_ISSUE:           state_name = "CVFPU_ISSUE";
           `S_CVFPU_WAIT:            state_name = "CVFPU_WAIT";
           `S_CVFPU_FMA_RF2:         state_name = "CVFPU_FMA_RF2";
           `S_CVFPU_FMA_RF3:         state_name = "CVFPU_FMA_RF3";
           `S_TLB_LOOKUP:            state_name = "TLB_LOOKUP";
           `S_TLB_CHECK:             state_name = "TLB_CHECK";
           `S_TLB_HIT:               state_name = "TLB_HIT";
           `S_TLB_START_FETCH:       state_name = "TLB_START_FETCH";
           `S_TLB_START_FETCH_HALF:  state_name = "TLB_START_FETCH_HALF";
           `S_PTW_START:             state_name = "PTW_START";
           `S_CBO_EXEC:              state_name = "CBO_EXEC";
           `S_CBO_WAIT:              state_name = "CBO_WAIT";
           `S_FETCH2_DRAM:           state_name = "FETCH2_DRAM";
           `S_FETCH_BUF_CHECK:       state_name = "FETCH_BUF_CHECK";
           `S_FETCH_BUF_USE:         state_name = "FETCH_BUF_USE";
           `S_MULDIV_START:          state_name = "MULDIV_START";
           `S_TLB_DECIDE:            state_name = "TLB_DECIDE";
           `S_FETCH_REQ:             state_name = "FETCH_REQ";
           `S_FRONTEND_MISS_WAIT:    state_name = "FRONTEND_MISS_WAIT";
           `S_HANDLE_CSR_COMMIT:     state_name = "HANDLE_CSR_COMMIT";
           `S_FP_INT_COMMIT:         state_name = "FP_INT_COMMIT";
           `S_LOCAL_LOAD:            state_name = "LOCAL_LOAD";
           `S_TLB_INSERT:            state_name = "TLB_INSERT";
           `S_BRANCH_RESOLVE:        state_name = "BRANCH_RESOLVE";
           default:                  state_name = "UNKNOWN";
         endcase
      end
   endfunction

   function [8*16-1:0] cache_state_name;
      input [4:0] s;
      begin
         case (s)
           CACHE_IDLE:      cache_state_name = "IDLE";
           CACHE_TAG_READ:  cache_state_name = "TAG_READ";
           CACHE_TAG_CHECK: cache_state_name = "TAG_CHECK";
           CACHE_FILL_REQ:  cache_state_name = "FILL_REQ";
           CACHE_FILL_WAIT: cache_state_name = "FILL_WAIT";
           CACHE_HIT_RESP:  cache_state_name = "HIT_RESP";
           CACHE_WB_REQ:    cache_state_name = "WB_REQ";
           CACHE_WB_WAIT:   cache_state_name = "WB_WAIT";
           CACHE_WB_PREP:   cache_state_name = "WB_PREP";
           CACHE_CBO_TAG_READ:  cache_state_name = "CBO_TAG_READ";
           CACHE_CBO_TAG_CHECK: cache_state_name = "CBO_TAG_CHECK";
           CACHE_CBO_RESP:      cache_state_name = "CBO_RESP";
           CACHE_BRAM_FILL_READ: cache_state_name = "BRAM_FILL_READ";
           CACHE_BRAM_WB_WRITE: cache_state_name = "BRAM_WB_WRITE";
           CACHE_BRAM_FILL_CAPTURE: cache_state_name = "BRAM_FILL_CAP";
           CACHE_BRAM_FILL_COMMIT: cache_state_name = "BRAM_FILL_COMMIT";
           CACHE_HIT_WRITE: cache_state_name = "HIT_WRITE";
           CACHE_TAG_WAIT: cache_state_name = "TAG_WAIT";
           CACHE_CBO_TAG_WAIT: cache_state_name = "CBO_TAG_WAIT";
           CACHE_WB_READ_WAIT: cache_state_name = "WB_READ_WAIT";
           CACHE_PROBE_READ: cache_state_name = "PROBE_READ";
           CACHE_PROBE_WAIT: cache_state_name = "PROBE_WAIT";
           CACHE_PROBE_CHECK: cache_state_name = "PROBE_CHECK";
           CACHE_INVALIDATE: cache_state_name = "INVALIDATE";
           CACHE_FLUSH_READ: cache_state_name = "FLUSH_READ";
           CACHE_FLUSH_WAIT: cache_state_name = "FLUSH_WAIT";
           CACHE_FLUSH_CHECK: cache_state_name = "FLUSH_CHECK";
           CACHE_FILL_LINE_WAIT: cache_state_name = "FILL_LINE_WAIT";
           CACHE_FILL_LINE_INSTALL: cache_state_name = "FILL_LINE_INST";
           default:         cache_state_name = "UNKNOWN";
         endcase
      end
   endfunction

   task dump_state_summary;
      integer summary_i;
      integer tlb_entries_total;
      reg [63:0] ptw_leaf_total;
      begin
         $display("%05d STATE SUMMARY cycles=%0d instret=%0d",
                  $time, state_stat_total_cycles, state_stat_instret);
         for (summary_i = 0; summary_i <= `S_LAST_STATE; summary_i = summary_i + 1) begin
            if (state_stat_cycles[summary_i] != 0) begin
               $display("%05d STATE %0d %-24s cycles=%0d pct_x100=%0d",
                        $time, summary_i, state_name(summary_i[5:0]), state_stat_cycles[summary_i],
                        state_stat_total_cycles == 0 ? 64'd0 :
                        (state_stat_cycles[summary_i] * 64'd10000) / state_stat_total_cycles);
            end
         end
         for (i = 0; i <= CACHE_LAST_STATE; i = i + 1) begin
            if (cache_state_stat_cycles[i] != 0) begin
               $display("%05d CACHE_STATE %0d %-16s cycles=%0d pct_x100=%0d",
                        $time, i, cache_state_name(i[4:0]), cache_state_stat_cycles[i],
                        state_stat_total_cycles == 0 ? 64'd0 :
                        (cache_state_stat_cycles[i] * 64'd10000) / state_stat_total_cycles);
            end
         end

         tlb_entries_total = tlb_stat_entries_4k +
                             tlb_stat_entries_2m +
                             tlb_stat_entries_1g;
         $display("%05d TLB_ENTRY_PAGE_SIZE name=4K entries=%0d pct_x100=%0d",
                  $time, tlb_stat_entries_4k,
                  tlb_entries_total == 0 ? 0 : (tlb_stat_entries_4k * 10000) / tlb_entries_total);
         $display("%05d TLB_ENTRY_PAGE_SIZE name=64K_NAPOT entries=0 pct_x100=0",
                  $time);
         $display("%05d TLB_ENTRY_PAGE_SIZE name=2M entries=%0d pct_x100=%0d",
                  $time, tlb_stat_entries_2m,
                  tlb_entries_total == 0 ? 0 : (tlb_stat_entries_2m * 10000) / tlb_entries_total);
         $display("%05d TLB_ENTRY_PAGE_SIZE name=1G entries=%0d pct_x100=%0d",
                  $time, tlb_stat_entries_1g,
                  tlb_entries_total == 0 ? 0 : (tlb_stat_entries_1g * 10000) / tlb_entries_total);
         $display("%05d TLB_ENTRY_PAGE_SIZE total=%0d capacity=%0d",
                  $time, tlb_entries_total, `TLB_ENTRIES);
         $display("%05d TLB_CAPACITY name=4K entries=%0d", $time, `TLB_4K_ENTRIES);
         $display("%05d TLB_CAPACITY name=2M entries=%0d", $time, `TLB_2M_ENTRIES);
         $display("%05d TLB_STATS lookups=%0d hits=%0d misses=%0d hit_pct_x100=%0d",
                  $time, tlb_stat_lookups, tlb_stat_hits, tlb_stat_misses,
                  tlb_stat_lookups == 0 ? 64'd0 :
                  (tlb_stat_hits * 64'd10000) / tlb_stat_lookups);
         $display("%05d TLB_STATS_4K hits=%0d inserts=%0d evicts=%0d",
                  $time, tlb_stat_hits_4k, tlb_stat_inserts_4k, tlb_stat_evicts_4k);
         $display("%05d TLB_STATS_2M hits=%0d inserts=%0d evicts=%0d",
                  $time, tlb_stat_hits_2m, tlb_stat_inserts_2m, tlb_stat_evicts_2m);
         $display("%05d TLB_UNCACHED page_1g=%0d napot=%0d",
                  $time, tlb_stat_uncached_1g, tlb_stat_uncached_napot);

         ptw_leaf_total = ptw_stat_leaf_4k + ptw_stat_leaf_64k_napot +
                          ptw_stat_leaf_2m + ptw_stat_leaf_1g;
         $display("%05d PTW_PAGE_SIZE name=4K walks=%0d pct_x100=%0d",
                  $time, ptw_stat_leaf_4k,
                  ptw_leaf_total == 0 ? 64'd0 : (ptw_stat_leaf_4k * 64'd10000) / ptw_leaf_total);
         $display("%05d PTW_PAGE_SIZE name=64K_NAPOT walks=%0d pct_x100=%0d",
                  $time, ptw_stat_leaf_64k_napot,
                  ptw_leaf_total == 0 ? 64'd0 : (ptw_stat_leaf_64k_napot * 64'd10000) / ptw_leaf_total);
         $display("%05d PTW_PAGE_SIZE name=2M walks=%0d pct_x100=%0d",
                  $time, ptw_stat_leaf_2m,
                  ptw_leaf_total == 0 ? 64'd0 : (ptw_stat_leaf_2m * 64'd10000) / ptw_leaf_total);
         $display("%05d PTW_PAGE_SIZE name=1G walks=%0d pct_x100=%0d",
                  $time, ptw_stat_leaf_1g,
                  ptw_leaf_total == 0 ? 64'd0 : (ptw_stat_leaf_1g * 64'd10000) / ptw_leaf_total);
         $display("%05d PTW_PAGE_SIZE total=%0d", $time, ptw_leaf_total);
      end
   endtask

   initial begin
      for (state_stat_i = 0; state_stat_i <= `S_LAST_STATE; state_stat_i = state_stat_i + 1)
         state_stat_cycles[state_stat_i] = 0;
      for (state_stat_i = 0; state_stat_i <= CACHE_LAST_STATE; state_stat_i = state_stat_i + 1)
         cache_state_stat_cycles[state_stat_i] = 0;
      cache_trace_enabled = $test$plusargs("cache_trace");
      cache_summary_enabled = $test$plusargs("cache_summary");
      fetch_buf_summary_enabled = $test$plusargs("fetch_buf_summary");
      state_summary_enabled = $test$plusargs("state_summary");
      state_summary_interval_seen = $value$plusargs("state_summary_interval=%d", state_summary_interval);
      if (state_summary_interval_seen)
         state_summary_enabled = 1;
      else if (state_summary_enabled)
         state_summary_interval = 1000000;
      if (state_summary_interval < 0)
         state_summary_interval = 0;
      state_summary_next = {32'd0, state_summary_interval};
      if (cache_trace_enabled)
         $display("CACHE TRACE ENABLED");
      if (cache_summary_enabled)
         $display("CACHE SUMMARY ENABLED");
      if (fetch_buf_summary_enabled)
         $display("FETCH BUFFER SUMMARY ENABLED");
      if (state_summary_enabled) begin
         if (state_summary_interval != 0)
            $display("STATE SUMMARY ENABLED interval=%0d cycles", state_summary_interval);
         else
            $display("STATE SUMMARY ENABLED");
      end
   end
`endif

   wire [63:0] cache_dram_addr = {33'd0, dram_addr, 3'b000};
   wire [63:0] cache_dram_next_addr = cache_dram_addr + 64'd8;
   wire [63:0] cache_dram_next_va = dram_va + 64'd8;
   wire [`CACHE_PHYS_TAG_BITS-1:0] cache_dram_ptag =
        cache_dram_addr[`CACHE_PHYS_BITS-1:`CACHE_PAGE_OFFSET_BITS];
   wire [`CACHE_PHYS_TAG_BITS-1:0] cache_cbo_ptag =
        cache_cbo_line_addr[30:12];
   wire [`CACHE_INDEX_BITS-1:0] cache_probe_line_index =
        {cache_probe_color, cache_addr[11:6]};
   wire [`CACHE_INDEX_BITS-1:0] cache_req_probe_line_index =
        {cache_probe_color, cache_req_line_addr[11:6]};

   function [2:0] cache_asid_color_mix;
      input [TLB_ASID_BITS-1:0] asid;
      begin
         cache_asid_color_mix = asid[2:0] ^ asid[5:3] ^ {2'd0, asid[6]} ^
                                {1'b0, asid[8:7]} ^ {2'd0, asid[9]};
      end
   endfunction

   function [`CACHE_INDEX_BITS-1:0] cache_way0_index;
      input [63:0] va;
      input [TLB_ASID_BITS-1:0] asid;
      begin
         cache_way0_index = {va[14:12] ^ cache_asid_color_mix(asid), va[11:6]};
      end
   endfunction

   function [`CACHE_INDEX_BITS-1:0] cache_way1_index;
      input [63:0] va;
      input [TLB_ASID_BITS-1:0] asid;
      begin
         cache_way1_index = {va[14:12] ^ cache_asid_color_mix(asid) ^
                             va[17:15] ^ va[23:21], va[11:6]};
      end
   endfunction

   function [`CACHE_VTAG_BITS-1:0] cache_vtag;
      input [63:0] va;
      begin
         cache_vtag = va[63:`CACHE_INDEX_BITS+`CACHE_LINE_OFFSET_BITS];
      end
   endfunction

   function [`CACHE_PHYS_TAG_BITS-1:0] cache_ptag;
      input [63:0] pa;
      begin
         cache_ptag = pa[`CACHE_PHYS_BITS-1:`CACHE_PAGE_OFFSET_BITS];
      end
   endfunction

   function cache_meta_valid;
      input [`CACHE_META_BITS-1:0] meta;
      begin
         cache_meta_valid = meta[`CACHE_VALID_BIT];
      end
   endfunction

   function cache_meta_dirty;
      input [`CACHE_META_BITS-1:0] meta;
      begin
         cache_meta_dirty = meta[`CACHE_DIRTY_BIT];
      end
   endfunction

   function [TLB_ASID_BITS-1:0] cache_meta_asid;
      input [`CACHE_META_BITS-1:0] meta;
      begin
         cache_meta_asid = meta[`CACHE_ASID_LSB +: TLB_ASID_BITS];
      end
   endfunction

   function [`CACHE_VTAG_BITS-1:0] cache_meta_vtag;
      input [`CACHE_META_BITS-1:0] meta;
      begin
         cache_meta_vtag = meta[`CACHE_VTAG_LSB +: `CACHE_VTAG_BITS];
      end
   endfunction

   function [`CACHE_PHYS_TAG_BITS-1:0] cache_meta_ptag;
      input [`CACHE_META_BITS-1:0] meta;
      begin
         cache_meta_ptag = meta[`CACHE_PTAG_LSB +: `CACHE_PHYS_TAG_BITS];
      end
   endfunction

   function [CACHE_PERM_BITS-1:0] cache_meta_perm;
      input [`CACHE_META_BITS-1:0] meta;
      begin
         cache_meta_perm = meta[`CACHE_PERM_LSB +: CACHE_PERM_BITS];
      end
   endfunction

   function [VHPR_EPOCH_BITS-1:0] cache_meta_epoch;
      input [`CACHE_META_BITS-1:0] meta;
      begin
         cache_meta_epoch = meta[`CACHE_EPOCH_LSB +: VHPR_EPOCH_BITS];
      end
   endfunction

   function cache_perm_allows_ctx;
      input [CACHE_PERM_BITS-1:0] perm;
      input [TLB_CTX_BITS-1:0] ctx;
      reg [1:0] access;
      reg [1:0] access_prv;
      reg       access_sum;
      reg       access_mxr;
      reg       pte_r;
      reg       pte_w;
      reg       pte_x;
      reg       pte_u;
      reg       data_read_ok;
      reg       user_ok;
      begin
         access     = ctx[5:4];
         access_prv = ctx[3:2];
         access_sum = ctx[1];
         access_mxr = ctx[0];
         pte_r      = perm[0];
         pte_w      = perm[1];
         pte_x      = perm[2];
         pte_u      = perm[3];

         if (perm[4]) begin
            cache_perm_allows_ctx = 1'b1;
         end else begin
            data_read_ok = pte_r || (access_mxr && pte_x);
            user_ok = access_prv == 0 ? pte_u :
                      access_prv == 1 ? (!pte_u || (access != 2'd0 && access_sum)) :
                                        1'b1;
            case (access)
              2'd0: cache_perm_allows_ctx = pte_x && user_ok;
              2'd1: cache_perm_allows_ctx = data_read_ok && user_ok;
              2'd2: cache_perm_allows_ctx = pte_w && user_ok;
              default: cache_perm_allows_ctx = data_read_ok && pte_w && user_ok;
            endcase
         end
      end
   endfunction

   function [`CACHE_META_BITS-1:0] cache_make_meta;
      input dirty;
      input valid;
      input [TLB_ASID_BITS-1:0] asid;
      input [CACHE_PERM_BITS-1:0] perm;
      input [`CACHE_VTAG_BITS-1:0] vtag;
      input [`CACHE_PHYS_TAG_BITS-1:0] ptag;
      input [VHPR_EPOCH_BITS-1:0] epoch;
      begin
         cache_make_meta = 0;
         cache_make_meta[`CACHE_DIRTY_BIT] = dirty;
         cache_make_meta[`CACHE_VALID_BIT] = valid;
         cache_make_meta[`CACHE_ASID_LSB +: TLB_ASID_BITS] = asid;
         cache_make_meta[`CACHE_PERM_LSB +: CACHE_PERM_BITS] = perm;
         cache_make_meta[`CACHE_VTAG_LSB +: `CACHE_VTAG_BITS] = vtag;
         cache_make_meta[`CACHE_PTAG_LSB +: `CACHE_PHYS_TAG_BITS] = ptag;
         cache_make_meta[`CACHE_EPOCH_LSB +: VHPR_EPOCH_BITS] = epoch;
      end
   endfunction

   reg [`CACHE_INDEX_BITS-1:0] cache_way0_rd_idx = 0;
   reg [`CACHE_INDEX_BITS-1:0] cache_way1_rd_idx = 0;
   reg [`CACHE_INDEX_BITS-1:0] cache_way0_bank0_rd_idx = 0;
   reg [`CACHE_INDEX_BITS-1:0] cache_way1_bank0_rd_idx = 0;
   reg [`CACHE_INDEX_BITS-1:0] cache_way0_next_rd_idx = 0;
   reg [`CACHE_INDEX_BITS-1:0] cache_way1_next_rd_idx = 0;
   wire [`CACHE_META_BITS-1:0] cache_way0_tag_rd_data;
   wire [`CACHE_META_BITS-1:0] cache_way1_tag_rd_data;
   wire [`CACHE_META_BITS-1:0] cache_way0_tag_next_rd_data;
   wire [`CACHE_META_BITS-1:0] cache_way1_tag_next_rd_data;
   reg                         cache_way0_tag_wr_en = 0;
   reg                         cache_way1_tag_wr_en = 0;
   reg                         cache_tag_wr_all = 0;
   reg  [`CACHE_INDEX_BITS-1:0] cache_tag_wr_idx = 0;
   reg  [`CACHE_META_BITS-1:0]  cache_tag_wr_data = 0;
   wire [`CACHE_META_BITS-1:0] dcache_way0_tag_rd_data;
   wire [`CACHE_META_BITS-1:0] dcache_way1_tag_rd_data;
   wire [`CACHE_META_BITS-1:0] dcache_way0_tag_next_rd_data;
   wire [`CACHE_META_BITS-1:0] dcache_way1_tag_next_rd_data;
   wire [63:0] dcache_way0_bank_rd_data [0:7];
   wire [63:0] dcache_way1_bank_rd_data [0:7];

   assign cache_way0_tag_rd_data = cache_req_instr ? icache_way0_tag_rd_data :
                                                     dcache_way0_tag_rd_data;
   assign cache_way1_tag_rd_data = cache_req_instr ? icache_way1_tag_rd_data :
                                                     dcache_way1_tag_rd_data;
   assign cache_way0_tag_next_rd_data = cache_req_instr ? icache_way0_tag_next_rd_data :
                                                          dcache_way0_tag_next_rd_data;
   assign cache_way1_tag_next_rd_data = cache_req_instr ? icache_way1_tag_next_rd_data :
                                                          dcache_way1_tag_next_rd_data;

   function [63:0] cache_selected_bank_data;
      input       way;
      input [2:0] bank;
      begin
         case (bank)
           3'd0: cache_selected_bank_data = cache_req_instr ?
                                            (way ? icache_way1_bank_rd_data[0] : icache_way0_bank_rd_data[0]) :
                                            (way ? dcache_way1_bank_rd_data[0] : dcache_way0_bank_rd_data[0]);
           3'd1: cache_selected_bank_data = cache_req_instr ?
                                            (way ? icache_way1_bank_rd_data[1] : icache_way0_bank_rd_data[1]) :
                                            (way ? dcache_way1_bank_rd_data[1] : dcache_way0_bank_rd_data[1]);
           3'd2: cache_selected_bank_data = cache_req_instr ?
                                            (way ? icache_way1_bank_rd_data[2] : icache_way0_bank_rd_data[2]) :
                                            (way ? dcache_way1_bank_rd_data[2] : dcache_way0_bank_rd_data[2]);
           3'd3: cache_selected_bank_data = cache_req_instr ?
                                            (way ? icache_way1_bank_rd_data[3] : icache_way0_bank_rd_data[3]) :
                                            (way ? dcache_way1_bank_rd_data[3] : dcache_way0_bank_rd_data[3]);
           3'd4: cache_selected_bank_data = cache_req_instr ?
                                            (way ? icache_way1_bank_rd_data[4] : icache_way0_bank_rd_data[4]) :
                                            (way ? dcache_way1_bank_rd_data[4] : dcache_way0_bank_rd_data[4]);
           3'd5: cache_selected_bank_data = cache_req_instr ?
                                            (way ? icache_way1_bank_rd_data[5] : icache_way0_bank_rd_data[5]) :
                                            (way ? dcache_way1_bank_rd_data[5] : dcache_way0_bank_rd_data[5]);
           3'd6: cache_selected_bank_data = cache_req_instr ?
                                            (way ? icache_way1_bank_rd_data[6] : icache_way0_bank_rd_data[6]) :
                                            (way ? dcache_way1_bank_rd_data[6] : dcache_way0_bank_rd_data[6]);
           3'd7: cache_selected_bank_data = cache_req_instr ?
                                            (way ? icache_way1_bank_rd_data[7] : icache_way0_bank_rd_data[7]) :
                                            (way ? dcache_way1_bank_rd_data[7] : dcache_way0_bank_rd_data[7]);
           default: cache_selected_bank_data = 64'd0;
         endcase
      end
   endfunction

   function [511:0] cache_selected_line_data;
      input way;
      begin
         cache_selected_line_data = {
            cache_selected_bank_data(way, 3'd7),
            cache_selected_bank_data(way, 3'd6),
            cache_selected_bank_data(way, 3'd5),
            cache_selected_bank_data(way, 3'd4),
            cache_selected_bank_data(way, 3'd3),
            cache_selected_bank_data(way, 3'd2),
            cache_selected_bank_data(way, 3'd1),
            cache_selected_bank_data(way, 3'd0)
         };
      end
   endfunction

   smolrv64_frontend #(
      .EPOCH_BITS(FRONTEND_EPOCH_BITS),
      .TLB_ASID_BITS(TLB_ASID_BITS),
      .CACHE_PERM_BITS(CACHE_PERM_BITS),
      .VHPR_EPOCH_BITS(VHPR_EPOCH_BITS)
   ) frontend_inst (
      .clock(clock),
      .reset(core_reset_now),
      .flush(frontend_buf_flush),
      .fill(frontend_buf_fill),
      .fill_base_va(frontend_buf_fill_base_va),
      .fill_prv(frontend_buf_fill_prv),
      .fill_asid(frontend_buf_fill_asid),
      .fill_data(frontend_buf_fill_data),
      .cmd_valid(frontend_cmd_valid),
      .cmd_pc(frontend_cmd_pc),
      .cmd_prv(frontend_cmd_prv),
      .cmd_asid(frontend_cmd_asid),
      .cmd_epoch(frontend_cmd_epoch),
      .rsp_hit(frontend_rsp_hit),
      .rsp_addr_hit(frontend_rsp_addr_hit),
      .rsp_full_insn_hit(frontend_rsp_full_insn_hit),
      .rsp_insn(frontend_rsp_insn),
      .rsp_offset(frontend_rsp_offset),
      .rsp_predicted_next_pc(frontend_rsp_predicted_next_pc),
      .rsp_prediction_kind(frontend_rsp_prediction_kind),
      .rsp_active_epoch(frontend_rsp_active_epoch),
      .rsp_fill_base_va(fetch_buf_fill_base_va),
      .rsp_fill_page_ok(fetch_buf_fill_page_ok),

      .icache_way0_rd_idx(cache_way0_rd_idx),
      .icache_way1_rd_idx(cache_way1_rd_idx),
      .icache_way0_bank0_rd_idx(cache_way0_bank0_rd_idx),
      .icache_way1_bank0_rd_idx(cache_way1_bank0_rd_idx),
      .icache_way0_next_rd_idx(cache_way0_next_rd_idx),
      .icache_way1_next_rd_idx(cache_way1_next_rd_idx),
      .icache_way0_tag_wr_en(cache_way0_tag_wr_en && (cache_req_instr || cache_tag_wr_all)),
      .icache_way1_tag_wr_en(cache_way1_tag_wr_en && (cache_req_instr || cache_tag_wr_all)),
      .icache_tag_wr_idx(cache_tag_wr_idx),
      .icache_tag_wr_data(cache_tag_wr_data),
      .icache_bank_wr_en(cache_req_instr ? cache_bank_wr_en : 8'd0),
      .icache_bank_wr_idx(cache_bank_wr_idx),
      .icache_bank_wr_way(cache_bank_wr_way),
      .icache_bank_wr_data(cache_bank_wr_data),
      .icache_way0_tag_rd_data(icache_way0_tag_rd_data),
      .icache_way1_tag_rd_data(icache_way1_tag_rd_data),
      .icache_way0_tag_next_rd_data(icache_way0_tag_next_rd_data),
      .icache_way1_tag_next_rd_data(icache_way1_tag_next_rd_data),
      .icache_way0_bank0_rd_data(icache_way0_bank0_rd_data),
      .icache_way0_bank1_rd_data(icache_way0_bank1_rd_data),
      .icache_way0_bank2_rd_data(icache_way0_bank2_rd_data),
      .icache_way0_bank3_rd_data(icache_way0_bank3_rd_data),
      .icache_way0_bank4_rd_data(icache_way0_bank4_rd_data),
      .icache_way0_bank5_rd_data(icache_way0_bank5_rd_data),
      .icache_way0_bank6_rd_data(icache_way0_bank6_rd_data),
      .icache_way0_bank7_rd_data(icache_way0_bank7_rd_data),
      .icache_way1_bank0_rd_data(icache_way1_bank0_rd_data),
      .icache_way1_bank1_rd_data(icache_way1_bank1_rd_data),
      .icache_way1_bank2_rd_data(icache_way1_bank2_rd_data),
      .icache_way1_bank3_rd_data(icache_way1_bank3_rd_data),
      .icache_way1_bank4_rd_data(icache_way1_bank4_rd_data),
      .icache_way1_bank5_rd_data(icache_way1_bank5_rd_data),
      .icache_way1_bank6_rd_data(icache_way1_bank6_rd_data),
      .icache_way1_bank7_rd_data(icache_way1_bank7_rd_data)
   );

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
      input        tlb_lookup_pulse;
      input        tlb_hit_pulse;
      input        tlb_miss_pulse;
      input        tlb_hit_4k_pulse;
      input        tlb_hit_2m_pulse;
      input        tlb_insert_4k_pulse;
      input        tlb_insert_2m_pulse;
      input        tlb_evict_4k_pulse;
      input        tlb_evict_2m_pulse;
      input        tlb_uncached_1g_pulse;
      input        tlb_uncached_napot_pulse;
      input        ptw_leaf_4k_pulse;
      input        ptw_leaf_2m_pulse;
      input        ptw_leaf_1g_pulse;
      input        ptw_leaf_napot_pulse;
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
           `HPM_EVENT_TLB_LOOKUP:      hpm_event_active = tlb_lookup_pulse;
           `HPM_EVENT_TLB_HIT:         hpm_event_active = tlb_hit_pulse;
           `HPM_EVENT_TLB_MISS:        hpm_event_active = tlb_miss_pulse;
           `HPM_EVENT_TLB_HIT_4K:      hpm_event_active = tlb_hit_4k_pulse;
           `HPM_EVENT_TLB_HIT_2M:      hpm_event_active = tlb_hit_2m_pulse;
           `HPM_EVENT_TLB_INSERT_4K:   hpm_event_active = tlb_insert_4k_pulse;
           `HPM_EVENT_TLB_INSERT_2M:   hpm_event_active = tlb_insert_2m_pulse;
           `HPM_EVENT_TLB_EVICT_4K:    hpm_event_active = tlb_evict_4k_pulse;
           `HPM_EVENT_TLB_EVICT_2M:    hpm_event_active = tlb_evict_2m_pulse;
           `HPM_EVENT_TLB_UNCACHED_1G: hpm_event_active = tlb_uncached_1g_pulse;
           `HPM_EVENT_TLB_UNCACHED_NAPOT: hpm_event_active = tlb_uncached_napot_pulse;
           `HPM_EVENT_PTW_LEAF_4K:     hpm_event_active = ptw_leaf_4k_pulse;
           `HPM_EVENT_PTW_LEAF_2M:     hpm_event_active = ptw_leaf_2m_pulse;
           `HPM_EVENT_PTW_LEAF_1G:     hpm_event_active = ptw_leaf_1g_pulse;
           `HPM_EVENT_PTW_LEAF_NAPOT:  hpm_event_active = ptw_leaf_napot_pulse;
           default:                    hpm_event_active = 1'b0;
         endcase
      end
   endfunction

   smolrv64_sdpram #(
      .ADDR_WIDTH(`CACHE_INDEX_BITS),
      .DATA_WIDTH(`CACHE_META_BITS),
      .READ_LATENCY(2)
   ) cache_way0_tag_ram (
      .clock   ( clock ),
      .rd_addr ( cache_way0_rd_idx ),
      .rd_data ( dcache_way0_tag_rd_data ),
      .wr_en   ( cache_way0_tag_wr_en && (!cache_req_instr || cache_tag_wr_all) ),
      .wr_addr ( cache_tag_wr_idx ),
      .wr_data ( cache_tag_wr_data )
   );

   smolrv64_sdpram #(
      .ADDR_WIDTH(`CACHE_INDEX_BITS),
      .DATA_WIDTH(`CACHE_META_BITS),
      .READ_LATENCY(2)
   ) cache_way1_tag_ram (
      .clock   ( clock ),
      .rd_addr ( cache_way1_rd_idx ),
      .rd_data ( dcache_way1_tag_rd_data ),
      .wr_en   ( cache_way1_tag_wr_en && (!cache_req_instr || cache_tag_wr_all) ),
      .wr_addr ( cache_tag_wr_idx ),
      .wr_data ( cache_tag_wr_data )
   );

   always @* begin
      cache_bank_wr_en = 8'd0;
      cache_bank_wr_idx = cache_target_idx;
      cache_bank_wr_way = cache_target_way;
      cache_bank_wr_data = cache_fill_data;

      if ((cache_state == CACHE_FILL_LINE_INSTALL || cache_state == CACHE_BRAM_FILL_COMMIT) &&
          cache_fill_data_valid) begin
         cache_bank_wr_en = 8'd1 << cache_fill_beat;
         cache_bank_wr_idx = cache_target_idx;
         cache_bank_wr_way = cache_target_way;
         cache_bank_wr_data = cache_fill_data;
         if (cache_req_write && cache_fill_beat == cache_req_bank)
            cache_bank_wr_data = merge_store_bytes(cache_fill_data, cache_store_data, cache_store_strb);
      end

      if (cache_state == CACHE_HIT_WRITE) begin
         cache_bank_wr_en = 8'd1 << cache_req_bank;
         cache_bank_wr_idx = cache_lookup_hit_way ? cache_way1_rd_idx : cache_way0_rd_idx;
         cache_bank_wr_way = cache_lookup_hit_way;
         cache_bank_wr_data = merge_store_bytes(cache_lookup_data, cache_store_data, cache_store_strb);
      end
   end

   genvar cache_bank_gen;
   generate
      for (cache_bank_gen = 0; cache_bank_gen < 8; cache_bank_gen = cache_bank_gen + 1) begin : vhpr_cache_banks
         smolrv64_sdpram #(
            .ADDR_WIDTH(`CACHE_INDEX_BITS),
            .DATA_WIDTH(64),
            .READ_LATENCY(2)
         ) dcache_way0_bank_ram (
            .clock   ( clock ),
            .rd_addr ( cache_bank_gen == 0 ? cache_way0_bank0_rd_idx : cache_way0_rd_idx ),
            .rd_data ( dcache_way0_bank_rd_data[cache_bank_gen] ),
            .wr_en   ( cache_bank_wr_en[cache_bank_gen] && !cache_bank_wr_way && !cache_req_instr ),
            .wr_addr ( cache_bank_wr_idx ),
            .wr_data ( cache_bank_wr_data )
         );

         smolrv64_sdpram #(
            .ADDR_WIDTH(`CACHE_INDEX_BITS),
            .DATA_WIDTH(64),
            .READ_LATENCY(2)
         ) dcache_way1_bank_ram (
            .clock   ( clock ),
            .rd_addr ( cache_bank_gen == 0 ? cache_way1_bank0_rd_idx : cache_way1_rd_idx ),
            .rd_data ( dcache_way1_bank_rd_data[cache_bank_gen] ),
            .wr_en   ( cache_bank_wr_en[cache_bank_gen] && cache_bank_wr_way && !cache_req_instr ),
            .wr_addr ( cache_bank_wr_idx ),
            .wr_data ( cache_bank_wr_data )
         );

      end
   endgenerate

   smolrv64_sdpram #(
      .ADDR_WIDTH(`CACHE_INDEX_BITS),
      .DATA_WIDTH(`CACHE_META_BITS),
      .READ_LATENCY(2)
   ) cache_way0_tag_next_ram (
      .clock   ( clock ),
      .rd_addr ( cache_way0_next_rd_idx ),
      .rd_data ( dcache_way0_tag_next_rd_data ),
      .wr_en   ( cache_way0_tag_wr_en && (!cache_req_instr || cache_tag_wr_all) ),
      .wr_addr ( cache_tag_wr_idx ),
      .wr_data ( cache_tag_wr_data )
   );

   smolrv64_sdpram #(
      .ADDR_WIDTH(`CACHE_INDEX_BITS),
      .DATA_WIDTH(`CACHE_META_BITS),
      .READ_LATENCY(2)
   ) cache_way1_tag_next_ram (
      .clock   ( clock ),
      .rd_addr ( cache_way1_next_rd_idx ),
      .rd_data ( dcache_way1_tag_next_rd_data ),
      .wr_en   ( cache_way1_tag_wr_en && (!cache_req_instr || cache_tag_wr_all) ),
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

   reg [VHPR_EPOCH_BITS-1:0] vhpr_epoch = 0;
   reg [VHPR_EPOCH_BITS-1:0] vhpr_next_epoch = 0;
   reg                       vhpr_epoch_update_pending = 0;
   reg                       vhpr_epoch_bump_req = 0;
   reg                       vhpr_epoch_bump_ack = 0;
   wire                      vhpr_epoch_bump_pending = vhpr_epoch_bump_req != vhpr_epoch_bump_ack;
   reg [63:0] csr_vhpr_reads = 0;
   reg [63:0] csr_vhpr_writes = 0;
   reg [63:0] csr_vhpr_read_hits = 0;
   reg [63:0] csr_vhpr_read_misses = 0;
   reg [63:0] csr_vhpr_write_hits = 0;
   reg [63:0] csr_vhpr_write_misses = 0;
   reg [63:0] csr_vhpr_fills = 0;
   reg [63:0] csr_vhpr_victim_evicts = 0;
   reg [63:0] csr_vhpr_dirty_victim_evicts = 0;
   reg [63:0] csr_vhpr_alias_evicts = 0;
   reg [63:0] csr_vhpr_dirty_alias_evicts = 0;
   reg [63:0] csr_vhpr_flush_evicts = 0;
   reg [63:0] csr_vhpr_dirty_flush_evicts = 0;
   reg [63:0] csr_vhpr_cbo_probes = 0;
   reg [63:0] csr_vhpr_ptw_probes = 0;
   reg [63:0] csr_vhpr_epoch_bumps = 0;
   reg [63:0] csr_vhpr_epoch_rollovers = 0;
   reg        vhpr_stats_clear_req = 0;
   reg        vhpr_stats_clear_ack = 0;
   wire       vhpr_stats_clear_pending = vhpr_stats_clear_req != vhpr_stats_clear_ack;

   task vhpr_clear_stats;
      begin
         csr_vhpr_reads <= 0;
         csr_vhpr_writes <= 0;
         csr_vhpr_read_hits <= 0;
         csr_vhpr_read_misses <= 0;
         csr_vhpr_write_hits <= 0;
         csr_vhpr_write_misses <= 0;
         csr_vhpr_fills <= 0;
         csr_vhpr_victim_evicts <= 0;
         csr_vhpr_dirty_victim_evicts <= 0;
         csr_vhpr_alias_evicts <= 0;
         csr_vhpr_dirty_alias_evicts <= 0;
         csr_vhpr_flush_evicts <= 0;
         csr_vhpr_dirty_flush_evicts <= 0;
         csr_vhpr_cbo_probes <= 0;
         csr_vhpr_ptw_probes <= 0;
         csr_vhpr_epoch_bumps <= 0;
         csr_vhpr_epoch_rollovers <= 0;
      end
   endtask

   task cache_flush_next_line;
      reg [`CACHE_INDEX_BITS-1:0] next_idx;
      begin
         if (!cache_flush_way) begin
            cache_flush_way <= 1'b1;
            next_idx = cache_flush_idx;
            cache_way0_rd_idx <= next_idx;
            cache_way1_rd_idx <= next_idx;
            cache_way0_bank0_rd_idx <= next_idx;
            cache_way1_bank0_rd_idx <= next_idx;
            cache_state <= CACHE_FLUSH_READ;
        end else if (cache_flush_idx == {`CACHE_INDEX_BITS{1'b1}}) begin
            cache_flush_ack <= cache_flush_req;
            cache_cbo_done_r <= 1;
            if (vhpr_epoch_update_pending) begin
               vhpr_epoch <= vhpr_next_epoch;
               vhpr_epoch_update_pending <= 1'b0;
            end
            cache_state <= CACHE_IDLE;
         end else begin
            next_idx = cache_flush_idx + 1'b1;
            cache_flush_idx <= next_idx;
            cache_flush_way <= 1'b0;
            cache_way0_rd_idx <= next_idx;
            cache_way1_rd_idx <= next_idx;
            cache_way0_bank0_rd_idx <= next_idx;
            cache_way1_bank0_rd_idx <= next_idx;
            cache_state <= CACHE_FLUSH_READ;
         end
      end
   endtask

   task cache_start_fill_request;
      begin
         cache_fill_base <= {cache_addr[63:6], 6'd0};
         cache_fill_beat <= 0;
         cache_fill_return_data <= 0;
         cache_fill_next_data <= 0;
         cache_replace_way <= ~cache_replace_way;
         cache_probe_color <= 0;
         cache_probe_found <= 0;
         cache_probe_found_way <= 0;
         cache_probe_found_idx <= 0;
         cache_probe_found_dirty <= 0;
         cache_probe_active <= 1;
         cache_way0_rd_idx <= {3'd0, cache_addr[11:6]};
         cache_way1_rd_idx <= {3'd0, cache_addr[11:6]};
         cache_way0_bank0_rd_idx <= {3'd0, cache_addr[11:6]};
         cache_way1_bank0_rd_idx <= {3'd0, cache_addr[11:6]};
         cache_state <= CACHE_PROBE_READ;
      end
   endtask

   task cache_finish_writeback_line;
      begin
         cache_wb_beat <= 0;
         if (cache_wb_after_cbo) begin
            cache_way0_tag_wr_en <= !cache_victim_way;
            cache_way1_tag_wr_en <= cache_victim_way;
            cache_tag_wr_idx <= cache_victim_idx;
            cache_tag_wr_data <= 0;
            cache_wb_after_cbo <= 1'b0;
            cache_cbo_done_r <= 1;
            cache_state <= CACHE_IDLE;
         end else if (cache_wb_after_flush) begin
            cache_way0_tag_wr_en <= !cache_victim_way;
            cache_way1_tag_wr_en <= cache_victim_way;
            cache_tag_wr_idx <= cache_victim_idx;
            cache_tag_wr_data <= 0;
            cache_wb_after_flush <= 1'b0;
            cache_flush_next_line();
         end else if (cache_wb_then_fill) begin
            cache_wb_then_fill <= 1'b0;
            cache_state <= CACHE_INVALIDATE;
         end else begin
            cache_state <= CACHE_FILL_REQ;
         end
      end
   endtask

   task vhpr_request_full_flush;
      input [7:0] reason;
      begin
         cache_flush_req <= ~cache_flush_ack;
         vhpr_epoch_bump_req <= ~vhpr_epoch_bump_ack;
      end
   endtask

   reg [63:0] bus_timeout_tval = 0;
   reg [11:0] bus_timeout_cause = 0;
   reg [63:0] mmio_timeout_tval = 0;

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
   reg  [63:0] imm_i, imm_j, imm_b, imm_u, imm_s, csr_arg, csr_read_val, csr_satp_write_val;
   reg  [63:0] csr_read_result = 0;
   reg  [63:0] fp_int_result = 0;
   reg  [ 4:0] fp_int_fflags = 0;
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
   (* max_fanout = 16 *) reg [11:0] csrno;
   (* max_fanout = 32 *) reg [31:0] insn = 0; // XXX We should set this on reset
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
   reg [7:0]   csr_senvcfg = 0;
   reg [63:0]  csr_mhpmcounter[0:`HPM_COUNTERS-1];
   reg [63:0]  csr_mhpmevent[0:`HPM_COUNTERS-1];
   reg [63:0]  csr_scountovf_read_val = 0;
   reg         hpm_counter_wr_en = 0;
   reg         hpm_event_wr_en = 0;
   reg [ 3:0]  hpm_wr_idx = 0;

   function [31:0] fetch_buf_pick_insn;
      input [127:0] data;
      input [3:0]   byte_offset;
      begin
         case (byte_offset[3:1])
           3'd0:    fetch_buf_pick_insn = data[31:0];
           3'd1:    fetch_buf_pick_insn = data[47:16];
           3'd2:    fetch_buf_pick_insn = data[63:32];
           3'd3:    fetch_buf_pick_insn = data[79:48];
           3'd4:    fetch_buf_pick_insn = data[95:64];
           3'd5:    fetch_buf_pick_insn = data[111:80];
           3'd6:    fetch_buf_pick_insn = data[127:96];
           default: fetch_buf_pick_insn = {16'd0, data[127:112]};
         endcase
      end
   endfunction

   function [63:0] frontend_fallthrough_pc;
      input [63:0] fetch_pc;
      input [31:0] fetch_insn;
      begin
         frontend_fallthrough_pc =
            fetch_pc + (fetch_insn[1:0] == 2'b11 ? 64'd4 : 64'd2);
      end
   endfunction

   reg [63:0]  hpm_wr_data = 0;
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
      input [5:0] counter_idx;
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

   function [63:0] csr_modify_value;
      input [63:0] old_val;
      input [63:0] write_arg;
      input [ 1:0] op;
      begin
         case (op)
           `CSR_OP_COPY: csr_modify_value = write_arg;
           `CSR_OP_OR:   csr_modify_value = old_val | write_arg;
           `CSR_OP_ANDN: csr_modify_value = old_val & ~write_arg;
           default:      csr_modify_value = old_val;
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
   localparam integer UART_FIFO_INDEX_BITS = 10;
   localparam integer UART_FIFO_DEPTH = 1 << UART_FIFO_INDEX_BITS;
   localparam [UART_FIFO_INDEX_BITS:0] UART_FIFO_DEPTH_COUNT =
      {1'b1, {UART_FIFO_INDEX_BITS{1'b0}}};
   (* ram_style = "block" *)
   reg [7:0]   uart_tx_fifo [0:UART_FIFO_DEPTH-1]; // TX FIFO between 16550 model and RS232
   reg [UART_FIFO_INDEX_BITS:0] uart_tx_head = 0, uart_tx_tail = 0;
   reg         uart_thre_pending = 0;
   (* rw_addr_collision = "yes" *)
   reg [7:0]   uart_rx_fifo [0:UART_FIFO_DEPTH-1]; // RX FIFO
   reg [UART_FIFO_INDEX_BITS:0] uart_rx_head = 0, uart_rx_tail = 0;
   reg [7:0]   uart_rx_front = 0;
   reg         uart_rx_front_valid = 0;
   reg         uart_rx_refill_pending = 0;
   reg [UART_FIFO_INDEX_BITS-1:0] uart_rx_refill_addr = 0;
   reg [2:0]   uart_break_count = 0;
   wire [UART_FIFO_INDEX_BITS:0] uart_tx_count = uart_tx_tail - uart_tx_head;
   wire        uart_tx_empty = uart_tx_head == uart_tx_tail;
   wire        uart_tx_full = uart_tx_count == UART_FIFO_DEPTH_COUNT;
   wire        uart_tx_accept = !uart_tx_full;
   wire        uart_tx_idle = uart_tx_empty && uart_tx_ready;
   wire [UART_FIFO_INDEX_BITS:0] uart_rx_count = uart_rx_tail - uart_rx_head;
   wire        uart_rx_empty = uart_rx_head == uart_rx_tail;
   wire        uart_rx_break_char = uart_rx_valid && uart_rx_data == 8'h18; // Ctrl-X
   wire        uart_rx_rbr_read = state == `S_LOCAL_LOAD &&
                                  phys_region(mem_addr) == `REGION_UART &&
                                  mem_addr[2:0] == 3'd0 && !uart_lcr[7];
   wire        uart_rx_pop = uart_rx_rbr_read && uart_rx_front_valid;
   wire        uart_rx_push = uart_rx_valid && !uart_rx_break_char &&
                              (uart_rx_count < UART_FIFO_DEPTH_COUNT ||
                               uart_rx_pop);
   wire        uart_rx_ip = uart_ier[0] && uart_rx_front_valid;  // RX data available
   wire        uart_thre_ip = uart_ier[1] && uart_thre_pending;
   wire        uart_iir_thre = !uart_rx_ip && uart_thre_ip;
   // IIR: bit 0 = 0 means interrupt pending, 1 = no pending; bits [7:6] = FIFO status
   wire [7:0]  uart_iir = uart_rx_ip   ? {uart_fcr_fifo, uart_fcr_fifo, 2'b0, 4'h4} :
                          uart_iir_thre ? {uart_fcr_fifo, uart_fcr_fifo, 2'b0, 4'h2} :
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
        ? {1'b0, 1'b1,           1'b1,           4'b0, uart_rx_front_valid}
        : {1'b0, uart_tx_idle,   uart_tx_accept, 4'b0, uart_rx_front_valid};
`else
   wire [7:0]  uart_lsr = {1'b0, uart_tx_idle, uart_tx_accept, 4'b0, uart_rx_front_valid}; // TEMT|THRE + DR
`endif
   wire        uart_irq_out = uart_rx_ip || uart_thre_ip;

   // PLIC (SiFive layout, base 0x0C000000)
   // Only S-mode context implemented (context 1)
   // plic_priority declared above (before initial block)
   reg [63:0]  plic_pending = 0;       // Interrupt pending bits
   reg [63:0]  plic_enabled = 0;       // Enable bits (S-mode context)
   reg [63:0]  plic_in_service = 0;    // Gateway has delivered source; wait for completion
   reg [ 2:0]  plic_threshold = 0;     // Priority threshold (S-mode)
   reg [ 5:0]  plic_claim = 0;        // Last claimed IRQ
   wire [63:0] plic_source_level = {ext_irq, 1'b0}
                                  | (uart_irq_out ? (64'd1 << 10) : 64'd0);

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
   reg [ 63:0] pre_mul_abs_s1, pre_mul_abs_s2;
   reg [ 31:0] pre_mul_abs_s1w, pre_mul_abs_s2w;
   reg         muldiv_output_sext32;
   reg         muldiv_output_negate;
   reg         muldiv_output_high_part;
   reg [3:0]   muldiv_start_op = `MULDIV_MUL;
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
   reg [63:0]  ptw_satp;        // SATP value used to launch the walk
   reg         ptw_sum;         // SUM value used for permission check
   reg         ptw_mxr;         // MXR value used for permission check
   reg         translated = 0;  // Set by PTW, cleared by consumer
   reg [11:0]  ptw_fault_cause; // Computed at top of S_PTW_READ
   reg [15:0]  insn_half;       // Saved lower half for cross-page instruction fetch

   reg [`TLB_4K_ENTRIES-1:0] tlb_4k_valid = 0;
   reg [`TLB_2M_ENTRIES-1:0] tlb_2m_valid = 0;
   reg [`TLB_4K_INDEX_BITS-1:0] tlb_4k_rd_idx = 0;
   reg [`TLB_2M_INDEX_BITS-1:0] tlb_2m_rd_idx = 0;
   reg [`TLB_4K_INDEX_BITS-1:0] tlb_4k_wr_idx = 0;
   reg [`TLB_2M_INDEX_BITS-1:0] tlb_2m_wr_idx = 0;
   reg                         tlb_4k_wr_en = 0;
   reg                         tlb_2m_wr_en = 0;
   reg [TLB_4K_DATA_BITS-1:0]  tlb_4k_wr_data = 0;
   reg [TLB_2M_DATA_BITS-1:0]  tlb_2m_wr_data = 0;
   reg [ 1:0]                  tlb_insert_level = 0;
   reg [`TLB_4K_INDEX_BITS-1:0] tlb_insert_4k_idx = 0;
   reg [`TLB_2M_INDEX_BITS-1:0] tlb_insert_2m_idx = 0;
   reg [TLB_4K_DATA_BITS-1:0]  tlb_insert_4k_data = 0;
   reg [TLB_2M_DATA_BITS-1:0]  tlb_insert_2m_data = 0;
   reg [63:0]                  ptw_route_pa = 0;
   reg [CACHE_PERM_BITS-1:0]   ptw_route_perm = 0;
   reg [ 4:0]                  ptw_route_return = 0;
   wire [TLB_4K_DATA_BITS-1:0] tlb_4k_rd_data;
   wire [TLB_2M_DATA_BITS-1:0] tlb_2m_rd_data;
   reg [63:0]  tlb_req_va;
   reg [ 1:0]  tlb_req_access;
   reg [ 1:0]  tlb_req_prv;
   reg         tlb_req_sum;
   reg         tlb_req_mxr;
   reg [ 4:0]  tlb_req_return;
   reg [63:0]  tlb_4k_hit_pa;
   reg [63:0]  tlb_2m_hit_pa;
   reg [63:0]  tlb_hit_pa;
   reg         tlb_latched_4k_hit = 0;
   reg         tlb_latched_2m_hit = 0;
   reg         hpm_tlb_insert_4k_pulse = 0;
   reg         hpm_tlb_insert_2m_pulse = 0;
   reg         hpm_tlb_evict_4k_pulse = 0;
   reg         hpm_tlb_evict_2m_pulse = 0;
   reg         hpm_tlb_uncached_1g_pulse = 0;
   reg         hpm_tlb_uncached_napot_pulse = 0;
   reg         hpm_ptw_leaf_4k_pulse = 0;
   reg         hpm_ptw_leaf_2m_pulse = 0;
   reg         hpm_ptw_leaf_1g_pulse = 0;
   reg         hpm_ptw_leaf_napot_pulse = 0;

   function [`TLB_2M_INDEX_BITS-1:0] tlb_2m_index;
      input [63:0] va;
      input [ 1:0] access;
      input [ 1:0] idx_prv;
      input        idx_sum;
      input        idx_mxr;
      reg [TLB_CTX_BITS-1:0] ctx;
      begin
         ctx = {access, idx_prv, idx_sum, idx_mxr};
         tlb_2m_index = va[28:21] ^ va[36:29] ^ {6'd0, va[38:37]} ^
                        {2'd0, ctx};
      end
   endfunction

   function [`TLB_4K_INDEX_BITS-1:0] tlb_4k_index;
      input [63:0] va;
      input [ 1:0] access;
      input [ 1:0] idx_prv;
      input        idx_sum;
      input        idx_mxr;
      reg [TLB_CTX_BITS-1:0] ctx;
      begin
         ctx = {access, idx_prv, idx_sum, idx_mxr};
         tlb_4k_index = va[21:12] ^ {1'b0, va[30:22]} ^
                        {8'd0, va[38:37]} ^ {4'd0, ctx};
      end
   endfunction

   function [TLB_SATP_KEY_BITS-1:0] satp_tlb_key;
      input [63:0] satp;
      begin
         // TLB entries are keyed by ASID only.  Translation invalidation is
         // driven by SFENCE.VMA, not by the SATP CSR write itself.
         satp_tlb_key = satp[53:44];
      end
   endfunction

   function [63:0] satp_warl_value;
      input [63:0] satp;
      begin
         satp_warl_value = satp;
         // RV64 Sv39 permits up to 16 ASID bits; this core implements 10.
         satp_warl_value[59:54] = 6'd0;
      end
   endfunction

   wire [TLB_CTX_BITS-1:0] tlb_req_ctx = {tlb_req_access, tlb_req_prv,
                                          tlb_req_sum, tlb_req_mxr};
   wire [TLB_SATP_KEY_BITS-1:0] tlb_current_satp_key =
      satp_tlb_key(csr_satp);
   wire [TLB_ASID_BITS-1:0] current_cache_asid =
      csr_satp[63:60] == 4'd8 ? csr_satp[53:44] : {TLB_ASID_BITS{1'b0}};

   function [TLB_ASID_BITS-1:0] fetch_asid_for_context;
      input [1:0] fetch_prv;
      begin
         fetch_asid_for_context =
            (csr_satp[63:60] == 4'd8 && fetch_prv != 2'd3) ?
            csr_satp[53:44] : {TLB_ASID_BITS{1'b0}};
      end
   endfunction

   wire [TLB_4K_TAG_BITS-1:0]    tlb_4k_rd_tag =
      tlb_4k_rd_data[TLB_4K_TAG_LSB +: TLB_4K_TAG_BITS];
   wire [TLB_4K_PBASE_BITS-1:0]  tlb_4k_rd_pbase =
      tlb_4k_rd_data[TLB_4K_PBASE_LSB +: TLB_4K_PBASE_BITS];
   wire [TLB_SATP_KEY_BITS-1:0]  tlb_4k_rd_satp_key =
      tlb_4k_rd_data[TLB_SATP_KEY_LSB +: TLB_SATP_KEY_BITS];
   wire [CACHE_PERM_BITS-1:0]    tlb_4k_rd_perm =
      tlb_4k_rd_data[TLB_PERM_LSB +: CACHE_PERM_BITS];
   wire [TLB_CTX_BITS-1:0]       tlb_4k_rd_ctx =
      tlb_4k_rd_data[TLB_CTX_LSB +: TLB_CTX_BITS];
   wire [TLB_2M_TAG_BITS-1:0]    tlb_2m_rd_tag =
      tlb_2m_rd_data[TLB_2M_TAG_LSB +: TLB_2M_TAG_BITS];
   wire [TLB_2M_PBASE_BITS-1:0]  tlb_2m_rd_pbase =
      tlb_2m_rd_data[TLB_2M_PBASE_LSB +: TLB_2M_PBASE_BITS];
   wire [TLB_SATP_KEY_BITS-1:0]  tlb_2m_rd_satp_key =
      tlb_2m_rd_data[TLB_SATP_KEY_LSB +: TLB_SATP_KEY_BITS];
   wire [CACHE_PERM_BITS-1:0]    tlb_2m_rd_perm =
      tlb_2m_rd_data[TLB_PERM_LSB +: CACHE_PERM_BITS];
   wire [TLB_CTX_BITS-1:0]       tlb_2m_rd_ctx =
      tlb_2m_rd_data[TLB_CTX_LSB +: TLB_CTX_BITS];
   wire tlb_4k_hit = state == `S_TLB_CHECK &&
                     tlb_4k_valid[tlb_4k_rd_idx] &&
                     tlb_4k_rd_tag == tlb_req_va[38:12] &&
                     tlb_4k_rd_satp_key == tlb_current_satp_key &&
                     tlb_4k_rd_ctx == tlb_req_ctx;
   wire tlb_2m_hit = state == `S_TLB_CHECK &&
                     tlb_2m_valid[tlb_2m_rd_idx] &&
                     tlb_2m_rd_tag == tlb_req_va[38:21] &&
                     tlb_2m_rd_satp_key == tlb_current_satp_key &&
                     tlb_2m_rd_ctx == tlb_req_ctx;
   wire hpm_tlb_lookup_pulse = state == `S_TLB_LOOKUP;
   wire hpm_tlb_hit_pulse = state == `S_TLB_DECIDE && (tlb_latched_4k_hit || tlb_latched_2m_hit);
   wire hpm_tlb_miss_pulse = state == `S_TLB_DECIDE && !(tlb_latched_4k_hit || tlb_latched_2m_hit);
   wire hpm_tlb_hit_4k_pulse = state == `S_TLB_DECIDE && tlb_latched_4k_hit;
   wire hpm_tlb_hit_2m_pulse = state == `S_TLB_DECIDE && !tlb_latched_4k_hit && tlb_latched_2m_hit;

   smolrv64_sdpram #(
      .ADDR_WIDTH(`TLB_4K_INDEX_BITS),
      .DATA_WIDTH(TLB_4K_DATA_BITS)
   ) tlb_4k_ram (
      .clock   ( clock ),
      .rd_addr ( tlb_4k_rd_idx ),
      .rd_data ( tlb_4k_rd_data ),
      .wr_en   ( tlb_4k_wr_en ),
      .wr_addr ( tlb_4k_wr_idx ),
      .wr_data ( tlb_4k_wr_data )
   );

   smolrv64_sdpram #(
      .ADDR_WIDTH(`TLB_2M_INDEX_BITS),
      .DATA_WIDTH(TLB_2M_DATA_BITS)
   ) tlb_2m_ram (
      .clock   ( clock ),
      .rd_addr ( tlb_2m_rd_idx ),
      .rd_data ( tlb_2m_rd_data ),
      .wr_en   ( tlb_2m_wr_en ),
      .wr_addr ( tlb_2m_wr_idx ),
      .wr_data ( tlb_2m_wr_data )
   );

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

   function frontend_spec_fetch_state;
      input [5:0] s;
      begin
         case (s)
           `S_RF2,
           `S_RF3,
           `S_LOAD_ALIGN,
           `S_MMIO_READ,
           `S_MMIO_ALIGN,
           `S_AMO,
           `S_STORE,
           `S_STORE_COMMIT,
           `S_MUL_RUNNING,
           `S_DIV_RUNNING,
           `S_BRANCH_RESOLVE,
           `S_EXECUTE,
           `S_EXECUTE2,
           `S_DRAM_LOAD_WAIT,
           `S_DRAM_LOAD2_WAIT,
           `S_DRAM_STORE_WAIT,
           `S_DRAM_STORE2,
           `S_DRAM_STORE_RESP_WAIT,
           `S_DRAM_STORE_RESP_ARM,
           `S_LOAD_LATCH,
           `S_CBO_EXEC,
           `S_CBO_WAIT,
           `S_CVFPU_ISSUE,
           `S_CVFPU_WAIT,
           `S_CVFPU_FMA_RF2,
           `S_CVFPU_FMA_RF3,
           `S_MULDIV_START:
             frontend_spec_fetch_state = 1'b1;
           default:
             frontend_spec_fetch_state = 1'b0;
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

   task flush_tlb;
      begin
         tlb_4k_valid <= 0;
         tlb_2m_valid <= 0;
`ifdef SIMULATE
         tlb_stat_entries_4k = 0;
         tlb_stat_entries_2m = 0;
         tlb_stat_entries_1g = 0;
`endif
      end
   endtask

   task clear_frontend_cmd;
      begin
         frontend_cmd_valid <= 0;
         frontend_cmd_fast_ready <= 0;
         frontend_cmd_speculative <= 0;
         frontend_cmd_spec_miss_ready <= 0;
      end
   endtask

   task arm_frontend_spec_cmd;
      begin
         frontend_cmd_valid <= 1;
         frontend_cmd_fast_ready <= 0;
         frontend_cmd_speculative <= 1;
         frontend_cmd_spec_miss_ready <= 0;
      end
   endtask

   task clear_frontend_fast_cmd;
      begin
         frontend_cmd_fast_ready <= 0;
      end
   endtask

   task clear_decode_queue;
      begin
         rf_decode_head <= 0;
         rf_decode_tail <= 0;
         rf_decode_count <= 0;
         rf_decode_prearmed <= 0;
         rf_decode_prearm_block = 1;
         frontend_decode_pending_valid <= 0;
         frontend_decode_pending_drain = 0;
      end
   endtask

   task squash_decode_execute;
      begin
         clear_decode_queue();
         id_valid <= 0;
         id_rf_ready <= 0;
         execute_req_valid <= 0;
      end
   endtask

   task flush_frontend_speculation;
      begin
         frontend_flush_this_cycle = 1;
         frontend_buf_flush <= 1'b1;
         clear_frontend_cmd();
         frontend_redirect_valid <= 0;
         squash_decode_execute();
         frontend_miss_valid <= 0;
         frontend_miss_done <= 0;
         frontend_miss_next_valid <= 0;
      end
   endtask

   task start_instruction_fetch_miss;
      input [63:0] fetch_va;
      input [ 1:0] fetch_prv;
      begin
         clear_frontend_fast_cmd();
         frontend_cmd_spec_miss_ready <= 0;
         if (csr_satp[63:60] == 4'd8 && fetch_prv != 3) begin
            // Sv39 instruction fetch translation
            state <= `S_TLB_START_FETCH;
         end else begin
            // Both local BRAM and external DRAM fetches use the cache response
            // path.  The cache refill engine chooses BRAM or AXI by line address.
            fetch_from_dram  <= 1;
            dram_addr        <= fetch_va[30:3];
            dram_va          <= fetch_va;
            dram_asid        <= {TLB_ASID_BITS{1'b0}};
            dram_perm        <= CACHE_PERM_PHYS;
            dram_ctx         <= {2'd0, fetch_prv, sum, mxr};
            dram_instr       <= 1;
            dram_read        <= 1;
            state            <= `S_DRAM_FETCH_WAIT;
         end
      end
   endtask

   task accept_instruction_fetch;
      input [63:0] accept_pc;
      input [63:0] accept_predicted_pc;
      input [31:0] accept_insn;
      input [ 1:0] accept_prediction_kind;
      input        accept_from_dram;
      begin
         pc <= accept_pc;
         insn <= accept_insn;
         fetch_from_dram <= accept_from_dram;
         translated <= 0;
         clear_frontend_cmd();
         frontend_redirect_valid <= 0;
         // (rf_decode_head/tail/count resets removed: stage_rf_decode_current
         //  in case 3 still resets the queue; case 1/2 slow paths no longer do.)
         frontend_decode_pending_valid <= 0;
         frontend_decode_pending_drain = 0;
         write_back_register <= 0;
         write_back_fp_valid <= 0;
         if (accept_pc[11:0] == 12'hFFE && accept_insn[1:0] == 2'b11 &&
             csr_satp[63:60] == 4'd8 && prv != 3) begin
            insn_half <= accept_insn[15:0];
            state <= `S_TLB_START_FETCH_HALF;
         end else if (accept_from_dram && accept_pc[2:1] == 2'b11) begin
            insn_half <= accept_insn[15:0];
            if (dram_latched_next_valid) begin
               dram_latched <= dram_latched_next;
               state <= `S_FETCH2_HALF;
            end else begin
               // For translated fetches, mem_addr still holds the physical
               // address only after a TLB translation.  A VHPR hit deliberately
               // avoids the TLB, so translate the second half instead of
               // deriving it from potentially stale mem_addr state.
               if (csr_satp[63:60] == 4'd8 && prv != 3) begin
                  state <= `S_TLB_START_FETCH_HALF;
               end else begin
                  dram_addr <= accept_pc[30:3] + 1;
                  dram_va   <= accept_pc + 64'd2;
                  dram_asid <= {TLB_ASID_BITS{1'b0}};
                  dram_perm <= CACHE_PERM_PHYS;
                  dram_ctx  <= {2'd0, prv, sum, mxr};
                  dram_instr <= 1;
                  dram_read <= 1;
                  state <= `S_DRAM_FETCH_HALF_WAIT;
               end
            end
         end else begin
            // Queue-only path (no fast-path bypass via stage_rf_decode_current).
            // Arbitrate against pending/queue pressure:
            //   - pending empty, queue has room: latch + drain to queue
            //   - queue full: bail to S_FETCH1 so backend can pop
            //   - pending occupied, queue has room: drain fires this cycle,
            //     wait one cycle and try again from S_FETCH_BUF_USE
            if (!frontend_decode_pending_valid && !rf_decode_full) begin
               latch_frontend_decode_pending(
                   accept_pc,
                   frontend_fallthrough_pc(accept_pc, accept_insn),
                   accept_predicted_pc,
                   accept_insn,
                   prv,
                   frontend_cmd_epoch,
                   accept_prediction_kind);
               state <= `S_FETCH1;
            end else if (rf_decode_full) begin
               state <= `S_FETCH1;
            end else begin
               state <= `S_FETCH_BUF_USE;
            end
         end
      end
   endtask

   task decode_rf_sources;
      input [31:0] decode_insn;
      output [ 4:0] decode_rd;
      output [ 4:0] decode_rs1;
      output [ 4:0] decode_rs2;
      output [ 5:0] decode_shamt;
      begin
         decode_rd = decode_insn`insn_rd;
         case (decode_insn[1:0])
           0: {decode_rs1,decode_rs2} = {{2'd1,decode_insn[9:7]}, {2'd1,decode_insn[4:2]}};
           1: {decode_rs1,decode_rs2} = {decode_insn[11:7],       {2'd1,decode_insn[4:2]}};
           2: {decode_rs1,decode_rs2} = {decode_insn[11:7],       decode_insn[6:2]};
           3: {decode_rs1,decode_rs2} = {decode_insn`insn_rs1,    decode_insn`insn_rs2};
         endcase
         // The exceptions
         if (decode_insn[1:0] == 1 && decode_insn[15])
           decode_rs1 = {2'd1,decode_insn[9:7]};
         if (decode_insn[1:0] == 2 && (decode_insn[15:13] == 3'b001 || decode_insn[15:14] == 2'b01))
           decode_rs1 = 2; // sp
         if (decode_insn[1:0] == 2 && 5 <= decode_insn[15:13])
           decode_rs1 = 2; // sp
         if ((decode_insn & 'he003) == 0)
           decode_rs1 = 2; // sp

         decode_shamt = decode_insn[25:20];
      end
   endtask

   task enqueue_rf_decode;
      input [63:0] decode_pc;
      input [63:0] decode_next_pc;
      input [63:0] decode_predicted_pc;
      input [31:0] decode_insn;
      input [ 1:0] decode_prv;
      input [FRONTEND_EPOCH_BITS-1:0] decode_epoch;
      input [ 1:0] decode_prediction_kind;
      input        decode_from_dram;
      reg   [ 4:0] decoded_rd;
      reg   [ 4:0] decoded_rs1;
      reg   [ 4:0] decoded_rs2;
      reg   [ 5:0] decoded_shamt;
      begin
         decode_rf_sources(decode_insn, decoded_rd, decoded_rs1,
                           decoded_rs2, decoded_shamt);
         if (rf_decode_full) begin
`ifdef SIMULATE
            $display("%05d BUG: enqueue into full rf_decode queue", $time);
            $finish;
`endif
         end else begin
            rf_decode_enqueue_this_cycle = 1'b1;
            rf_decode_pc_q[rf_decode_tail] <= decode_pc;
            rf_decode_next_pc_q[rf_decode_tail] <= decode_next_pc;
            rf_decode_predicted_pc_q[rf_decode_tail] <= decode_predicted_pc;
            rf_decode_insn_q[rf_decode_tail] <= decode_insn;
            rf_decode_prv_q[rf_decode_tail] <= decode_prv;
            rf_decode_epoch_q[rf_decode_tail] <= decode_epoch;
            rf_decode_prediction_kind_q[rf_decode_tail] <= decode_prediction_kind;
            rf_decode_from_dram_q[rf_decode_tail] <= decode_from_dram;
            rf_decode_rd_q[rf_decode_tail] <= decoded_rd;
            rf_decode_rs1_q[rf_decode_tail] <= decoded_rs1;
            rf_decode_rs2_q[rf_decode_tail] <= decoded_rs2;
            rf_decode_shamt_q[rf_decode_tail] <= decoded_shamt;
            rf_decode_tail <= rf_decode_tail + 1'b1;
            rf_decode_count <= rf_decode_count + 1'b1;
         end
      end
   endtask

   task latch_frontend_decode_pending;
      input [63:0] decode_pc;
      input [63:0] decode_next_pc;
      input [63:0] decode_predicted_pc;
      input [31:0] decode_insn;
      input [ 1:0] decode_prv;
      input [FRONTEND_EPOCH_BITS-1:0] decode_epoch;
      input [ 1:0] decode_prediction_kind;
      begin
         frontend_decode_pending_valid <= 1;
         frontend_decode_pending_pc <= decode_pc;
         frontend_decode_pending_next_pc <= decode_next_pc;
         frontend_decode_pending_predicted_pc <= decode_predicted_pc;
         frontend_decode_pending_insn <= decode_insn;
         frontend_decode_pending_prv <= decode_prv;
         frontend_decode_pending_epoch <= decode_epoch;
         frontend_decode_pending_prediction_kind <= decode_prediction_kind;
         rf_decode_prearmed <= 0;
         rf_decode_prearm_block = 1;
         frontend_cmd_pc <= decode_predicted_pc;
         clear_frontend_fast_cmd();
         frontend_cmd_spec_miss_ready <= 0;
         if (rf_decode_count == RF_DECODE_QUEUE_DEPTH_COUNT - 1'b1) begin
            frontend_cmd_valid <= 0;
            frontend_cmd_speculative <= 0;
         end else begin
            frontend_cmd_valid <= 1;
            frontend_cmd_speculative <= 1;
         end
      end
   endtask

   task enqueue_frontend_decode_pending;
      begin
         enqueue_rf_decode(frontend_decode_pending_pc,
                           frontend_decode_pending_next_pc,
                           frontend_decode_pending_predicted_pc,
                           frontend_decode_pending_insn,
                           frontend_decode_pending_prv,
                           frontend_decode_pending_epoch,
                           frontend_decode_pending_prediction_kind,
                           1'b0);
         frontend_decode_pending_valid <= 0;
         frontend_decode_pending_drain = 1'b1;
      end
   endtask

   task stage_rf_decode_current;
      input [63:0] decode_pc;
      input [63:0] decode_next_pc;
      input [63:0] decode_predicted_pc;
      input [31:0] decode_insn;
      input [ 1:0] decode_prediction_kind;
      input        decode_from_dram;
      reg   [ 4:0] decoded_rd;
      reg   [ 4:0] decoded_rs1;
      reg   [ 4:0] decoded_rs2;
      reg   [ 5:0] decoded_shamt;
      begin
         if (id_valid) begin
`ifdef SIMULATE
            $display("%05d BUG: stage current decode with busy ID stage", $time);
            $finish;
`endif
            state <= `S_FETCH1;
         end else begin
            decode_rf_sources(decode_insn, decoded_rd, decoded_rs1,
                              decoded_rs2, decoded_shamt);
            write_back_register = 0;
            write_back_fp_valid = 0;
            rf_decode_head <= 0;
            rf_decode_tail <= 0;
            rf_decode_count <= 0;
            rf_decode_prearmed <= 0;
            rf_decode_prearm_block = 1;
            frontend_decode_pending_valid <= 0;
            frontend_decode_pending_drain = 0;
            id_valid <= 1;
            id_rf_ready <= 0;
            id_pc <= decode_pc;
            id_next_pc <= decode_next_pc;
            id_predicted_pc <= decode_predicted_pc;
            id_prv <= prv;
            id_epoch <= fetch_epoch;
            id_insn <= decode_insn;
            id_prediction_kind <= decode_prediction_kind;
            id_rd <= decoded_rd;
            id_rs1 <= decoded_rs1;
            id_rs2 <= decoded_rs2;
            id_shamt <= decoded_shamt;
            rs1 <= decoded_rs1;
            rs2 <= decoded_rs2;
            frontend_cmd_pc <= decode_predicted_pc;
            frontend_cmd_prv <= prv;
            arm_frontend_spec_cmd();
            state <= `S_RF2;
         end
      end
   endtask

   task enqueue_rf_decode_speculative;
      input [63:0] decode_pc;
      input [63:0] decode_next_pc;
      input [63:0] decode_predicted_pc;
      input [31:0] decode_insn;
      input [ 1:0] decode_prv;
      input [FRONTEND_EPOCH_BITS-1:0] decode_epoch;
      input [ 1:0] decode_prediction_kind;
      begin
         enqueue_rf_decode(decode_pc, decode_next_pc, decode_predicted_pc,
                           decode_insn, decode_prv,
                           decode_epoch, decode_prediction_kind, 1'b0);
      end
   endtask

   task launch_rf_decode_read;
      begin
         if (id_valid) begin
`ifdef SIMULATE
            $display("%05d BUG: launch into busy ID stage", $time);
            $finish;
`endif
            state <= `S_FETCH1;
         end else begin
            id_valid <= 1;
            id_rf_ready <= rf_decode_prearmed;
            id_pc <= rf_decode_pc;
            id_next_pc <= rf_decode_next_pc;
            id_predicted_pc <= rf_decode_predicted_pc;
            id_prv <= rf_decode_prv;
            id_epoch <= rf_decode_epoch;
            id_insn <= rf_decode_insn;
            id_prediction_kind <= rf_decode_prediction_kind;
            id_rd <= rf_decode_rd;
            id_rs1 <= rf_decode_rs1;
            id_rs2 <= rf_decode_rs2;
            id_shamt <= rf_decode_shamt;
           rs1 <= rf_decode_rs1;
           rs2 <= rf_decode_rs2;
            rf_decode_pop_this_cycle = 1'b1;
            rf_decode_head <= rf_decode_head + 1'b1;
            rf_decode_count <= rf_decode_enqueue_this_cycle ?
                               rf_decode_count : rf_decode_count - 1'b1;
            rf_decode_prearmed <= 0;
            rf_decode_prearm_block = 1;
            if (rf_decode_count == RF_DECODE_QUEUE_DEPTH_COUNT ||
                rf_decode_enqueue_this_cycle) begin
               arm_frontend_spec_cmd();
            end
            state <= rf_decode_prearmed ? `S_RF3 : `S_RF2;
         end
      end
   endtask

   task launch_rf_decode_read_preserve_state;
      begin
         if (id_valid) begin
`ifdef SIMULATE
            $display("%05d BUG: background launch into busy ID stage", $time);
            $finish;
`endif
         end else begin
            id_valid <= 1;
            id_rf_ready <= rf_decode_prearmed;
            id_pc <= rf_decode_pc;
            id_next_pc <= rf_decode_next_pc;
            id_predicted_pc <= rf_decode_predicted_pc;
            id_prv <= rf_decode_prv;
            id_epoch <= rf_decode_epoch;
            id_insn <= rf_decode_insn;
            id_prediction_kind <= rf_decode_prediction_kind;
            id_rd <= rf_decode_rd;
            id_rs1 <= rf_decode_rs1;
            id_rs2 <= rf_decode_rs2;
            id_shamt <= rf_decode_shamt;
            rs1 <= rf_decode_rs1;
            rs2 <= rf_decode_rs2;
            rf_decode_pop_this_cycle = 1'b1;
            rf_decode_head <= rf_decode_head + 1'b1;
            rf_decode_count <= rf_decode_enqueue_this_cycle ?
                               rf_decode_count : rf_decode_count - 1'b1;
            rf_decode_prearmed <= 0;
            rf_decode_prearm_block = 1;
            if (rf_decode_count == RF_DECODE_QUEUE_DEPTH_COUNT ||
                rf_decode_enqueue_this_cycle) begin
               arm_frontend_spec_cmd();
            end
         end
      end
   endtask

   task prepare_branch_metadata;
      input [63:0] b_pc;
      input [63:0] b_next_pc;
      input [31:0] b_insn;
      input [63:0] b_s1;
      input [63:0] b_s2;
      reg [63:0] d_imm_i, d_imm_j, d_imm_b, d_c_j, d_c_b;
      begin
         d_imm_i = {{52{b_insn[31]}}, b_insn[31:20]};
         d_imm_j = {{44{b_insn[31]}}, b_insn[19:12], b_insn[20], b_insn[30:21], 1'b0};
         d_imm_b = {{52{b_insn[31]}}, b_insn[7], b_insn[30:25], b_insn[11:8], 1'b0};
         d_c_j   = {{53{b_insn[12]}}, b_insn[8], b_insn[10:9], b_insn[6], b_insn[7],
                    b_insn[2], b_insn[11], b_insn[5:3], 1'b0};
         d_c_b   = {{56{b_insn[12]}}, b_insn[6:5], b_insn[2], b_insn[11:10],
                    b_insn[4:3], 1'b0};

         pre_npc <= b_next_pc;
         pre_jalr_target <= (b_s1 + d_imm_i) & ~64'd1;
         pre_branch_target <= b_pc + d_imm_b;
         pre_branch_taken <= 0;

         if ((b_insn & 'he003) == 'ha001) begin // C.J
            pre_npc <= b_pc + d_c_j;
         end else if ((b_insn & 'he003) == 'hc001) begin // C.BEQZ
            pre_branch_target <= b_pc + d_c_b;
            pre_branch_taken <= b_s1 == 0;
         end else if ((b_insn & 'he003) == 'he001) begin // C.BNEZ
            pre_branch_target <= b_pc + d_c_b;
            pre_branch_taken <= b_s1 != 0;
         end else if ((b_insn & 'hf07f) == 'h8002) begin // C.JR
            pre_jalr_target <= b_s1 & ~64'd1;
         end else if ((b_insn & 'hf07f) == 'h9002) begin // C.JALR
            pre_jalr_target <= b_s1 & ~64'd1;
         end else if ((b_insn & 'h0000007f) == 'h0000006f) begin // JAL
            pre_npc <= b_pc + d_imm_j;
         end else if ((b_insn & 'h0000707f) == 'h00000067) begin // JALR
            pre_jalr_target <= (b_s1 + d_imm_i) & ~64'd1;
         end else if ((b_insn & 'h0000707f) == 'h00000063) begin // BEQ
            pre_branch_taken <= b_s1 == b_s2;
         end else if ((b_insn & 'h0000707f) == 'h00001063) begin // BNE
            pre_branch_taken <= b_s1 != b_s2;
         end else if ((b_insn & 'h0000707f) == 'h00004063) begin // BLT
            pre_branch_taken <= $signed(b_s1) < $signed(b_s2);
         end else if ((b_insn & 'h0000707f) == 'h00005063) begin // BGE
            pre_branch_taken <= $signed(b_s1) >= $signed(b_s2);
         end else if ((b_insn & 'h0000707f) == 'h00006063) begin // BLTU
            pre_branch_taken <= b_s1 < b_s2;
         end else if ((b_insn & 'h0000707f) == 'h00007063) begin // BGEU
            pre_branch_taken <= b_s1 >= b_s2;
         end
      end
   endtask


   function id_no_pending_wb_hazard;
      input       pending_int_valid;
      input [4:0] pending_int_rd;
      input       pending_fp_valid;
      input [4:0] pending_fp_rd;
      reg         id_uses_fp_rs3;
      begin
         id_uses_fp_rs3 = id_insn[6:4] == 3'b100 && id_insn[1:0] == 2'b11;
         id_no_pending_wb_hazard =
            (!pending_int_valid || pending_int_rd == 0 ||
             (id_rs1 != pending_int_rd && id_rs2 != pending_int_rd)) &&
            (!pending_fp_valid ||
             (id_rs1 != pending_fp_rd &&
              id_rs2 != pending_fp_rd &&
              (!id_uses_fp_rs3 || id_insn[31:27] != pending_fp_rd)));
      end
   endfunction

   task prepare_execute_req_from_id;
      input preserve_state;
      begin
           // Register RF output into s1/s2/f1/f2 flip-flops.  Early launch can
           // overlap this read with the previous retire's writeback, so use the
           // local writeback bypass before latching operands.
           s1 <= rf3_s1_value;
           s2 <= rf3_s2_value;
           f1 <= rf3_f1_value;
           f2 <= rf3_f2_value;
           // Pre-compute SC reservation match one cycle early; S_EXECUTE's
           // SC branch then only sees a 1-bit registered hit.
           reservation_match <= (reservation == rf3_s1_value);
           pre_mul_abs_s1  <= rf3_s1_value[63] ? -rf3_s1_value : rf3_s1_value;
           pre_mul_abs_s2  <= rf3_s2_value[63] ? -rf3_s2_value : rf3_s2_value;
           pre_mul_abs_s1w <= rf3_s1_value[31] ? -rf3_s1_value[31:0] : rf3_s1_value[31:0];
           pre_mul_abs_s2w <= rf3_s2_value[31] ? -rf3_s2_value[31:0] : rf3_s2_value[31:0];
           id_valid <= 0;
           id_rf_ready <= 0;
           execute_req_pc <= rf3_pc;
           execute_req_next_pc <= rf3_next_pc;
           execute_req_predicted_pc <= rf3_predicted_pc;
           execute_req_prv <= rf3_prv;
           execute_req_epoch <= rf3_epoch;
           execute_req_insn <= rf3_insn;
           execute_req_prediction_kind <= rf3_prediction_kind;
           execute_req_rd <= rf3_rd;
           execute_req_rs1 <= rf3_rs1;
           execute_req_rs2 <= rf3_rs2;
           execute_req_shamt <= rf3_shamt;
           execute_req_valid <= 1;
           prepare_branch_metadata(rf3_pc, rf3_next_pc, rf3_insn,
                                   rf3_s1_value, rf3_s2_value);
           if (!preserve_state)
              state <= `S_BRANCH_RESOLVE;

           // Pre-decode ALU operation and second operand for S_EXECUTE.
           // rf3_insn/rf3_pc are registered FFs; rf3_s2_value is read data
           // after same-cycle writeback bypass.
           // All assignments use <= so they register into pre_exe_op/pre_exe_b/pre_exe_sxt.
           // Immediates are computed inline (1-3 LUT from insn_reg) rather than read from
           // the imm_i/imm_u registers (which are only updated with = inside S_EXECUTE).
           begin : rf3_pre_decode
              reg [63:0] d_imm_i, d_imm_u, d_c_imm;

              d_imm_i = {{52{rf3_insn[31]}},rf3_insn[31:20]};
              d_imm_u = {{32{rf3_insn[31]}},rf3_insn[31:12],12'd0};
              d_c_imm = {{59{rf3_insn[12]}},rf3_insn[6:2]};  // c_imm12_62

              // Default: harmless value (only matters for instructions reaching S_EXECUTE2)
              pre_exe_op  <= `EXOP_OPB;
              pre_exe_b   <= 64'd0;
              pre_exe_sxt <= 0;

              // ---- Compressed instructions (rf3_insn[1:0] != 2'b11) ----

              // Quadrant 0
              if ((rf3_insn & 'he003) == 'h0000) begin // C.ADDI4SPN (rd'=rs2)
                 pre_exe_op <= `EXOP_ADD;
                 pre_exe_b  <= {54'd0, rf3_insn[10:7], rf3_insn[12:11], rf3_insn[5], rf3_insn[6], 2'd0};
              end

              // Quadrant 1
              else if ((rf3_insn & 'he003) == 'h0001) begin // C.ADDI / C.NOP
                 pre_exe_op <= `EXOP_ADD;
                 pre_exe_b  <= d_c_imm;
              end
              else if ((rf3_insn & 'he003) == 'h2001) begin // C.ADDIW (RV64)
                 pre_exe_op  <= `EXOP_ADD;
                 pre_exe_b   <= d_c_imm;
                 pre_exe_sxt <= 1;
              end
              else if ((rf3_insn & 'he003) == 'h4001) begin // C.LI
                 pre_exe_op <= `EXOP_OPB;
                 pre_exe_b  <= d_c_imm;
              end
              else if ((rf3_insn & 'hef83) == 'h6101) begin // C.ADDI16SP (rd=sp)
                 pre_exe_op <= `EXOP_ADD;
                 pre_exe_b  <= {{55{rf3_insn[12]}}, rf3_insn[4:3], rf3_insn[5], rf3_insn[2], rf3_insn[6], 4'd0};
              end
              else if ((rf3_insn & 'he003) == 'h6001) begin // C.LUI (rd!=0,2)
                 pre_exe_op <= `EXOP_OPB;
                 pre_exe_b  <= {{47{rf3_insn[12]}}, rf3_insn[6:2], 12'd0};
              end
              else if ((rf3_insn & 'hec03) == 'h8001) begin // C.SRLI
                 pre_exe_op <= `EXOP_SHR;
                 pre_exe_b  <= d_c_imm;
              end
              else if ((rf3_insn & 'hec03) == 'h8401) begin // C.SRAI
                 pre_exe_op <= `EXOP_SAR;
                 pre_exe_b  <= d_c_imm;
              end
              else if ((rf3_insn & 'hec03) == 'h8801) begin // C.ANDI
                 pre_exe_op <= `EXOP_AND;
                 pre_exe_b  <= d_c_imm;
              end
              else if ((rf3_insn & 'hfc63) == 'h8c01) begin // C.SUB
                 pre_exe_op <= `EXOP_SUB;
                 pre_exe_b  <= rf3_s2_value;
              end
              else if ((rf3_insn & 'hfc63) == 'h8c21) begin // C.XOR
                 pre_exe_op <= `EXOP_XOR;
                 pre_exe_b  <= rf3_s2_value;
              end
              else if ((rf3_insn & 'hfc63) == 'h8c41) begin // C.OR
                 pre_exe_op <= `EXOP_OR;
                 pre_exe_b  <= rf3_s2_value;
              end
              else if ((rf3_insn & 'hfc63) == 'h8c61) begin // C.AND
                 pre_exe_op <= `EXOP_AND;
                 pre_exe_b  <= rf3_s2_value;
              end
              else if ((rf3_insn & 'hfc63) == 'h9c01) begin // C.SUBW
                 pre_exe_op  <= `EXOP_SUB;
                 pre_exe_b   <= rf3_s2_value;
                 pre_exe_sxt <= 1;
              end
              else if ((rf3_insn & 'hfc63) == 'h9c21) begin // C.ADDW
                 pre_exe_op  <= `EXOP_ADD;
                 pre_exe_b   <= rf3_s2_value;
                 pre_exe_sxt <= 1;
              end

              // Quadrant 2
              else if ((rf3_insn & 'he003) == 'h0002) begin // C.SLLI
                 pre_exe_op <= `EXOP_SHL;
                 pre_exe_b  <= d_c_imm;
              end
              else if ((rf3_insn & 'hf07f) == 'h8002) begin // C.JR (no exe_add, default ok)
                 ;
              end
              else if ((rf3_insn & 'hf003) == 'h8002) begin // C.MV
                 pre_exe_op <= `EXOP_OPB;
                 pre_exe_b  <= rf3_s2_value;
              end
              else if ((rf3_insn & 'hf07f) == 'h9002) begin // C.JALR (link = rf3_pc+2)
                 pre_exe_op <= `EXOP_OPB;
                 pre_exe_b  <= rf3_next_pc;
              end
              else if ((rf3_insn & 'hf003) == 'h9002) begin // C.ADD
                 pre_exe_op <= `EXOP_ADD;
                 pre_exe_b  <= rf3_s2_value;
              end

              // ---- 32-bit instructions (rf3_insn[1:0] == 2'b11) ----
              else if (rf3_insn[1:0] == 2'b11) begin
                 case (rf3_insn[6:2])
                    5'b01101: begin // LUI
                       pre_exe_op <= `EXOP_OPB;
                       pre_exe_b  <= d_imm_u;
                    end
                    5'b00101: begin // AUIPC
                       pre_exe_op <= `EXOP_OPB;
                       pre_exe_b  <= rf3_pc + d_imm_u;
                    end
                    5'b11011: begin // JAL (link = rf3_pc+4)
                       pre_exe_op <= `EXOP_OPB;
                       pre_exe_b  <= rf3_next_pc;
                    end
                    5'b11001: begin // JALR (link = rf3_pc+4)
                       pre_exe_op <= `EXOP_OPB;
                       pre_exe_b  <= rf3_next_pc;
                    end
                    5'b00100: begin // OP-IMM: funct3 selects operation
                       pre_exe_b <= d_imm_i; // default; shifts override below
                       case (rf3_insn[14:12])
                          3'b000: pre_exe_op <= `EXOP_ADD;   // ADDI
                          3'b001: begin pre_exe_op <= `EXOP_SHL; pre_exe_b <= {58'd0, rf3_insn[25:20]}; end  // SLLI
                          3'b010: pre_exe_op <= `EXOP_LTS;   // SLTI
                          3'b011: pre_exe_op <= `EXOP_LTU;   // SLTIU
                          3'b100: pre_exe_op <= `EXOP_XOR;   // XORI
                          3'b101: begin // SRLI / SRAI
                             pre_exe_op <= rf3_insn[30] ? `EXOP_SAR : `EXOP_SHR;
                             pre_exe_b  <= {58'd0, rf3_insn[25:20]};
                          end
                          3'b110: pre_exe_op <= `EXOP_OR;    // ORI
                          3'b111: pre_exe_op <= `EXOP_AND;   // ANDI
                       endcase
                    end
                    5'b01100: begin // OP-REG: funct3+funct7[5] selects operation
                       pre_exe_b <= rf3_s2_value;
                       case (rf3_insn[14:12])
                          3'b000: pre_exe_op <= rf3_insn[30] ? `EXOP_SUB : `EXOP_ADD;  // ADD/SUB
                          3'b001: pre_exe_op <= `EXOP_SHL;  // SLL
                          3'b010: pre_exe_op <= `EXOP_LTS;  // SLT
                          3'b011: pre_exe_op <= `EXOP_LTU;  // SLTU
                          3'b100: pre_exe_op <= `EXOP_XOR;  // XOR
                          3'b101: pre_exe_op <= rf3_insn[30] ? `EXOP_SAR : `EXOP_SHR;  // SRL/SRA
                          3'b110: pre_exe_op <= `EXOP_OR;   // OR
                          3'b111: pre_exe_op <= `EXOP_AND;  // AND
                          // MUL/DIV (funct7[0]=1): exe_add unused; default EXOP_OPB is fine
                       endcase
                    end
                    5'b00110: begin // OP-IMM-32 (W-type immediates)
                       pre_exe_sxt <= 1;
                       case (rf3_insn[14:12])
                          3'b000: begin pre_exe_op <= `EXOP_ADD; pre_exe_b <= d_imm_i; end  // ADDIW
                          3'b001: begin pre_exe_op <= `EXOP_SHL; pre_exe_b <= {59'd0, rf3_insn[24:20]}; end  // SLLIW
                          3'b101: begin  // SRLIW / SRAIW
                             pre_exe_op <= rf3_insn[30] ? `EXOP_SAR : `EXOP_SHR;
                             pre_exe_b  <= {59'd0, rf3_insn[24:20]};
                          end
                          default: ; // other funct3: no exe_add
                       endcase
                    end
                    5'b01110: begin // OP-REG-32 (W-type register)
                       pre_exe_sxt <= 1;
                       pre_exe_b <= rf3_s2_value;
                       case (rf3_insn[14:12])
                          3'b000: pre_exe_op <= rf3_insn[30] ? `EXOP_SUB : `EXOP_ADD;  // ADDW/SUBW
                          3'b001: pre_exe_op <= `EXOP_SHL;  // SLLW
                          3'b101: pre_exe_op <= rf3_insn[30] ? `EXOP_SAR : `EXOP_SHR;  // SRLW/SRAW
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

`ifdef USE_CVFPU
           pre_fp_rnd_mode <= rf3_insn[14:12] == 3'b111 ? frm : rf3_insn[14:12];
           pre_fp_rmode_ok <= !(rf3_insn[14:12] == 3'b101 || rf3_insn[14:12] == 3'b110 ||
                                (rf3_insn[14:12] == 3'b111 && frm > 3'b100));
`endif

           // Mem pre-decode: compute offset/size/op/mask/wb-reg one cycle
           // early so S_EXECUTE can share a single s1+offset adder instead of
           // selecting between 22 parallel adders. Immediates are computed
           // inline from rf3_insn bits (the imm_*/c_uimm* registers are written
           // in S_EXECUTE and therefore stale here).
           begin : rf3_mem_decode
              reg [63:0] d_imm_i_s, d_imm_s_s;
              reg [63:0] d_clw_off, d_cld_off, d_clwsp_off, d_cldsp_off,
                         d_cswsp_off, d_csdsp_off;

              d_imm_i_s    = {{52{rf3_insn[31]}}, rf3_insn[31:20]};
              d_imm_s_s    = {{52{rf3_insn[31]}}, rf3_insn[31:25], rf3_insn[11:7]};
              d_clw_off    = {57'd0, rf3_insn[5],    rf3_insn[12:10], rf3_insn[6],     2'd0};
              d_cld_off    = {56'd0, rf3_insn[6:5],  rf3_insn[12:10],              3'd0};
              d_clwsp_off  = {56'd0, rf3_insn[3:2],  rf3_insn[12],    rf3_insn[6:4],   2'd0};
              d_cldsp_off  = {55'd0, rf3_insn[4:2],  rf3_insn[12],    rf3_insn[6:5],   3'd0};
              d_cswsp_off  = {56'd0, rf3_insn[8:7],  rf3_insn[12:9],               2'd0};
              d_csdsp_off  = {55'd0, rf3_insn[9:7],  rf3_insn[12:10],              3'd0};

              // Defaults: non-mem instruction
              pre_mem_op        <= `MEMOP_NONE;
              pre_mem_offset    <= 64'd0;
              pre_load_size_lg2 <= 3'd0;
              pre_mem_wr_mask   <= 8'd0;
              pre_mem_wb_reg    <= 5'd0;
              pre_mem_fp        <= 1'b0;

              // Compressed loads / stores (quadrants 0 & 2)
              if ((rf3_insn & 'he003) == 'h4000) begin // C.LW
                 pre_mem_op        <= `MEMOP_LOAD;
                 pre_mem_offset    <= d_clw_off;
                 pre_load_size_lg2 <= 3'b110; // W, sign-extend
                 pre_mem_wb_reg    <= {2'b01, rf3_insn[4:2]};
              end
              else if ((rf3_insn & 'he003) == 'h2000) begin // C.FLD
                 pre_mem_op        <= `MEMOP_LOAD;
                 pre_mem_offset    <= d_cld_off;
                 pre_load_size_lg2 <= 3'b011; // D
                 pre_mem_wb_reg    <= {2'b01, rf3_insn[4:2]};
                 pre_mem_fp        <= 1'b1;
              end
              else if ((rf3_insn & 'he003) == 'h6000) begin // C.LD
                 pre_mem_op        <= `MEMOP_LOAD;
                 pre_mem_offset    <= d_cld_off;
                 pre_load_size_lg2 <= 3'b011; // D
                 pre_mem_wb_reg    <= {2'b01, rf3_insn[4:2]};
              end
              else if ((rf3_insn & 'he003) == 'hc000) begin // C.SW
                 pre_mem_op      <= `MEMOP_STORE;
                 pre_mem_offset  <= d_clw_off;
                 pre_mem_wr_mask <= 8'h0f;
              end
              else if ((rf3_insn & 'he003) == 'ha000) begin // C.FSD
                 pre_mem_op      <= `MEMOP_STORE;
                 pre_mem_offset  <= d_cld_off;
                 pre_mem_wr_mask <= 8'hff;
                 pre_mem_fp      <= 1'b1;
              end
              else if ((rf3_insn & 'he003) == 'he000) begin // C.SD
                 pre_mem_op      <= `MEMOP_STORE;
                 pre_mem_offset  <= d_cld_off;
                 pre_mem_wr_mask <= 8'hff;
              end
              else if ((rf3_insn & 'he003) == 'h4002) begin // C.LWSP
                 pre_mem_op        <= `MEMOP_LOAD;
                 pre_mem_offset    <= d_clwsp_off;
                 pre_load_size_lg2 <= 3'b110;
                 pre_mem_wb_reg    <= rf3_insn[11:7];
              end
              else if ((rf3_insn & 'he003) == 'h2002) begin // C.FLDSP
                 pre_mem_op        <= `MEMOP_LOAD;
                 pre_mem_offset    <= d_cldsp_off;
                 pre_load_size_lg2 <= 3'b011;
                 pre_mem_wb_reg    <= rf3_insn[11:7];
                 pre_mem_fp        <= 1'b1;
              end
              else if ((rf3_insn & 'he003) == 'h6002) begin // C.LDSP
                 pre_mem_op        <= `MEMOP_LOAD;
                 pre_mem_offset    <= d_cldsp_off;
                 pre_load_size_lg2 <= 3'b011;
                 pre_mem_wb_reg    <= rf3_insn[11:7];
              end
              else if ((rf3_insn & 'he003) == 'hc002) begin // C.SWSP
                 pre_mem_op      <= `MEMOP_STORE;
                 pre_mem_offset  <= d_cswsp_off;
                 pre_mem_wr_mask <= 8'h0f;
              end
              else if ((rf3_insn & 'he003) == 'ha002) begin // C.FSDSP
                 pre_mem_op      <= `MEMOP_STORE;
                 pre_mem_offset  <= d_csdsp_off;
                 pre_mem_wr_mask <= 8'hff;
                 pre_mem_fp      <= 1'b1;
              end
              else if ((rf3_insn & 'he003) == 'he002) begin // C.SDSP
                 pre_mem_op      <= `MEMOP_STORE;
                 pre_mem_offset  <= d_csdsp_off;
                 pre_mem_wr_mask <= 8'hff;
              end

              // Uncompressed loads / stores / atomics
              else if (rf3_insn[1:0] == 2'b11 && rf3_insn[6:2] == 5'b00000) begin // LOAD
                 pre_mem_op        <= `MEMOP_LOAD;
                 pre_mem_offset    <= d_imm_i_s;
                 // funct3 = rf3_insn[14:12]: {2:0] = size; [2] = 1 → NO sign-ext (U-variant); invert to match
                 // Current encoding: load_size_lg2 = {sxt, size[1:0]} where sxt=1 means sign-ext.
                 //   LB=0|4, LH=1|4, LW=2|4, LD=3, LBU=0, LHU=1, LWU=2.
                 // RISC-V: funct3[2]=0 is signed (B/H/W), funct3[2]=1 is unsigned (BU/HU/WU); LD has funct3=011 (size=3, no sxt).
                 // So load_size_lg2 = {~funct3[2] & (funct3[1:0] != 2'b11), funct3[1:0]}.
                 pre_load_size_lg2 <= {~rf3_insn[14] & ~(rf3_insn[13] & rf3_insn[12]), rf3_insn[13:12]};
                 pre_mem_wb_reg    <= rf3_insn[11:7];
              end
              else if (rf3_insn[1:0] == 2'b11 && rf3_insn[6:2] == 5'b01000) begin // STORE
                 pre_mem_op     <= `MEMOP_STORE;
                 pre_mem_offset <= d_imm_s_s;
                 // wr_mask = (1 << (1 << funct3[1:0])) - 1
                 case (rf3_insn[13:12])
                    2'b00: pre_mem_wr_mask <= 8'h01; // SB
                    2'b01: pre_mem_wr_mask <= 8'h03; // SH
                    2'b10: pre_mem_wr_mask <= 8'h0f; // SW
                    2'b11: pre_mem_wr_mask <= 8'hff; // SD
                 endcase
              end
              else if ((rf3_insn & 'hf9f0707f) == 'h1000202f ||  // LR.W
                       (rf3_insn & 'hf9f0707f) == 'h1000302f) begin // LR.D
                 pre_mem_op        <= `MEMOP_LR;
                 pre_mem_offset    <= 64'd0;
                 pre_load_size_lg2 <= rf3_insn[12] ? 3'b011 : 3'b110; // D : W(sign-ext)
                 pre_mem_wb_reg    <= rf3_insn[11:7];
              end
              else if ((rf3_insn & 'hf800707f) == 'h1800202f ||  // SC.W
                       (rf3_insn & 'hf800707f) == 'h1800302f) begin // SC.D
                 pre_mem_op      <= `MEMOP_SC;
                 pre_mem_offset  <= 64'd0;
                 pre_mem_wr_mask <= rf3_insn[12] ? 8'hff : 8'h0f;
                 pre_mem_wb_reg  <= rf3_insn[11:7];
              end
              // FP loads: FLW (funct3=010) and FLD (funct3=011); opcode 0000111
              else if (rf3_insn[1:0] == 2'b11 && rf3_insn[6:2] == 5'b00001 &&
                       (rf3_insn[14:12] == 3'b010 || rf3_insn[14:12] == 3'b011)) begin
                 pre_mem_op        <= `MEMOP_LOAD;
                 pre_mem_offset    <= d_imm_i_s;
                 // FLW: 32-bit zero-extend (load_size_lg2=010), NaN-box in S_LOAD_ALIGN.
                 // FLD: 64-bit (load_size_lg2=011).
                 pre_load_size_lg2 <= {1'b0, rf3_insn[13:12]};
                 pre_mem_wb_reg    <= rf3_insn[11:7];
                 pre_mem_fp        <= 1'b1;
              end
              // FP stores: FSW (funct3=010) and FSD (funct3=011); opcode 0100111
              else if (rf3_insn[1:0] == 2'b11 && rf3_insn[6:2] == 5'b01001 &&
                       (rf3_insn[14:12] == 3'b010 || rf3_insn[14:12] == 3'b011)) begin
                 pre_mem_op        <= `MEMOP_STORE;
                 pre_mem_offset    <= d_imm_s_s;
                 pre_mem_wr_mask   <= rf3_insn[12] ? 8'hff : 8'h0f;
                 pre_mem_fp        <= 1'b1;
              end
              else if (rf3_insn[1:0] == 2'b11 && rf3_insn[6:2] == 5'b01011 &&
                       (rf3_insn[14:12] == 3'b010 || rf3_insn[14:12] == 3'b011)) begin // AMO*.W / AMO*.D
                 // funct5 must be one of the 9 defined AMO variants; otherwise
                 // leave pre_mem_op = MEMOP_NONE so S_EXECUTE traps illegal instruction.
                 // (LR/SC are funct5 00010/00011, already matched above.)
                 case (rf3_insn[31:27])
                    5'b00000, 5'b00001, 5'b00100, 5'b01000, 5'b01100,
                    5'b10000, 5'b10100, 5'b11000, 5'b11100: begin
                       pre_mem_op        <= `MEMOP_AMO;
                       pre_mem_offset    <= 64'd0;
                       pre_load_size_lg2 <= rf3_insn[12] ? 3'b011 : 3'b010; // D : W(no sxt)
                       pre_mem_wb_reg    <= rf3_insn[11:7];
                    end
                    default: ; // illegal AMO funct5: falls through
                 endcase
              end
           end // rf3_mem_decode
      end
   endtask

   task try_issue_queued_decode_preserve_state;
      input       allow_ex_prepare;
      input       pending_int_valid;
      input [4:0] pending_int_rd;
      input       pending_fp_valid;
      input [4:0] pending_fp_rd;
      begin
         if (allow_ex_prepare && id_valid && id_rf_ready && ex_accept_ready &&
             id_no_pending_wb_hazard(pending_int_valid, pending_int_rd,
                                     pending_fp_valid, pending_fp_rd)) begin
            prepare_execute_req_from_id(1'b1);
         end else if (!id_valid && rf_decode_valid && !frontend_miss_valid &&
             rf_decode_epoch == fetch_epoch &&
             rf_decode_pc == npc &&
             rf_decode_prv == prv) begin
            launch_rf_decode_read_preserve_state();
         end
      end
   endtask

   function frontend_physical_fetch_ok;
      input [63:0] fetch_pc;
      begin
         frontend_physical_fetch_ok =
            (((fetch_pc ^ `MEM_BASEADDR) &
              (64'hffff_ffff_ffff_ffff << `MEM_SIZE_LG2)) == 0) ||
            fetch_pc[63:31] == 1;
      end
   endfunction

   function frontend_speculative_fetch_ok;
      input [63:0] fetch_pc;
      input [ 1:0] fetch_prv;
      begin
         frontend_speculative_fetch_ok =
            (csr_satp[63:60] == 4'd8 && fetch_prv != 3) ||
            frontend_physical_fetch_ok(fetch_pc);
      end
   endfunction

   task try_frontend_speculative_fetch_buf_enqueue;
      begin
         if (frontend_cmd_speculative &&
             !rf_decode_full &&
             (!frontend_decode_pending_valid ||
              frontend_decode_pending_drain) &&
             !frontend_miss_valid && !frontend_miss_done &&
             !frontend_cmd_spec_miss_ready &&
             frontend_speculative_fetch_ok(frontend_cmd_pc, frontend_cmd_prv) &&
             frontend_rsp_hit) begin
            latch_frontend_decode_pending(frontend_cmd_pc,
                                           frontend_fallthrough_pc(
                                              frontend_cmd_pc,
                                              frontend_rsp_insn),
                                           frontend_rsp_predicted_next_pc,
                                           frontend_rsp_insn,
                                           frontend_cmd_prv,
                                           frontend_cmd_epoch,
                                           frontend_rsp_prediction_kind);
         end else if (frontend_cmd_speculative && frontend_cmd_valid &&
                      !rf_decode_full &&
                      (!frontend_decode_pending_valid ||
                       frontend_decode_pending_drain) &&
                      !frontend_miss_valid && !frontend_miss_done &&
                      !frontend_cmd_spec_miss_ready &&
                      frontend_speculative_fetch_ok(frontend_cmd_pc, frontend_cmd_prv)) begin
            frontend_cmd_spec_miss_ready <= 1;
         end
      end
   endtask

   function frontend_miss_matches_retire;
      input [63:0] retire_pc;
      input [ 1:0] retire_prv;
      input [FRONTEND_EPOCH_BITS-1:0] retire_epoch;
      begin
         frontend_miss_matches_retire =
            frontend_miss_epoch == retire_epoch &&
            frontend_miss_pc == retire_pc &&
            frontend_miss_prv == retire_prv;
      end
   endfunction

   function id_matches_retire;
      input [63:0] retire_pc;
      input [ 1:0] retire_prv;
      input [FRONTEND_EPOCH_BITS-1:0] retire_epoch;
      begin
         id_matches_retire =
            id_valid &&
            id_pc == retire_pc &&
            id_prv == retire_prv &&
            id_epoch == retire_epoch;
      end
   endfunction

   function execute_req_matches_retire;
      input [63:0] retire_pc;
      input [ 1:0] retire_prv;
      input [FRONTEND_EPOCH_BITS-1:0] retire_epoch;
      begin
         execute_req_matches_retire =
            execute_req_valid &&
            execute_req_pc == retire_pc &&
            execute_req_prv == retire_prv &&
            execute_req_epoch == retire_epoch;
      end
   endfunction

   function frontend_spec_miss_state;
      input [5:0] s;
      begin
         case (s)
           `S_MULDIV_START,
           `S_MUL_RUNNING,
           `S_DIV_RUNNING,
           `S_CVFPU_ISSUE,
           `S_CVFPU_WAIT,
           `S_CVFPU_FMA_RF2,
           `S_CVFPU_FMA_RF3:
             frontend_spec_miss_state = 1'b1;
           default:
             frontend_spec_miss_state = 1'b0;
         endcase
      end
   endfunction

   task try_frontend_speculative_miss_start;
      begin
         if (frontend_cmd_speculative && frontend_cmd_valid &&
             !frontend_miss_valid && !frontend_miss_done &&
             frontend_cmd_spec_miss_ready && cache_idle &&
             (csr_satp[63:60] != 4'd8 || frontend_cmd_prv == 3) &&
             frontend_physical_fetch_ok(frontend_cmd_pc) &&
             frontend_cmd_pc[2:1] != 2'b11) begin
            frontend_miss_valid      <= 1;
            frontend_miss_done       <= 0;
            frontend_miss_pc         <= frontend_cmd_pc;
            frontend_miss_prv        <= frontend_cmd_prv;
            frontend_miss_asid       <= frontend_cmd_asid;
            frontend_miss_epoch      <= frontend_cmd_epoch;
            frontend_miss_next_valid <= 0;
            frontend_cmd_spec_miss_ready <= 0;
            dram_addr                <= frontend_cmd_pc[30:3];
            dram_va                  <= frontend_cmd_pc;
            dram_asid                <= {TLB_ASID_BITS{1'b0}};
            dram_perm                <= CACHE_PERM_PHYS;
            dram_ctx                 <= {2'd0, frontend_cmd_prv, sum, mxr};
            dram_instr               <= 1;
            dram_read                <= 1;
         end
      end
   endtask

   task consume_frontend_miss;
      reg [63:0]  fill_base;
      reg [127:0] miss_aligned;
      reg [31:0]  miss_insn;
      begin
         fill_base = {frontend_miss_pc[63:3], 3'b000};
         miss_aligned = frontend_miss_next_valid ?
                        {frontend_miss_next_data, frontend_miss_data} :
                        {64'bx, frontend_miss_data};
         miss_insn = fetch_buf_pick_insn(miss_aligned, {1'b0, frontend_miss_pc[2:0]});
         if (fill_base[11:0] <= 12'hff0 && frontend_miss_next_valid) begin
         frontend_buf_fill         <= 1'b1;
         frontend_buf_fill_base_va <= fill_base;
         frontend_buf_fill_prv     <= frontend_miss_prv;
         frontend_buf_fill_asid    <= frontend_miss_asid;
         frontend_buf_fill_data    <= miss_aligned;
         end
         frontend_miss_valid <= 0;
         frontend_miss_done  <= 0;
         accept_instruction_fetch(frontend_miss_pc,
                                  frontend_fallthrough_pc(frontend_miss_pc,
                                                          miss_insn),
                                  miss_insn, 2'd0, 1'b1);
      end
   endtask

   task prepare_retire_fetch;
      input [63:0] prepare_pc;
      input [ 1:0] prepare_prv;
      input [FRONTEND_EPOCH_BITS-1:0] prepare_epoch;
      begin
         frontend_cmd_valid <= 1;
         frontend_cmd_pc <= prepare_pc;
         frontend_cmd_prv <= prepare_prv;
         frontend_cmd_asid <= fetch_asid_for_context(prepare_prv);
         frontend_cmd_epoch <= prepare_epoch;
         frontend_cmd_fast_ready <= 1;
         frontend_cmd_speculative <= 0;
         frontend_cmd_spec_miss_ready <= 0;
      end
   endtask

   task prepare_current_epoch_fetch;
      input [63:0] prepare_pc;
      input [ 1:0] prepare_prv;
      begin
         prepare_retire_fetch(prepare_pc, prepare_prv, fetch_epoch);
      end
   endtask

   task issue_frontend_redirect;
      begin
         prepare_retire_fetch(frontend_redirect_pc,
                              frontend_redirect_prv,
                              frontend_redirect_epoch);
         frontend_redirect_valid <= 0;
      end
   endtask

   task redirect_retire_fetch;
      input [63:0] redirect_pc;
      input [ 1:0] redirect_prv;
      reg   [FRONTEND_EPOCH_BITS-1:0] redirect_epoch;
      begin
         frontend_flush_this_cycle = 1;
         redirect_epoch = fetch_epoch + 1'b1;
         fetch_epoch <= redirect_epoch;
         squash_decode_execute();
         clear_frontend_cmd();
         frontend_redirect_valid <= 1;
         frontend_redirect_pc <= redirect_pc;
         frontend_redirect_prv <= redirect_prv;
         frontend_redirect_epoch <= redirect_epoch;
      end
   endtask

   task retire_queued_decode_or_refetch;
      begin
         if (rf_decode_epoch == fetch_epoch &&
             rf_decode_pc == npc &&
             rf_decode_prv == prv) begin
            insn <= rf_decode_insn;
            fetch_from_dram <= rf_decode_from_dram;
            translated <= 0;
            write_back_register = 0;
            write_back_fp_valid = 0;
            launch_rf_decode_read();
         end else begin
            redirect_retire_fetch(npc, prv);
            state <= `S_FETCH_REQ;
         end
      end
   endtask

   task try_early_launch_queued_decode;
      input [63:0] retire_pc;
      input [ 1:0] retire_prv;
      output       launched;
      begin
         launched = 1'b0;
         if (!id_valid && rf_decode_valid && !frontend_miss_valid &&
             rf_decode_epoch == fetch_epoch &&
             rf_decode_pc == retire_pc &&
             rf_decode_prv == retire_prv) begin
            launch_rf_decode_read_preserve_state();
            launched = 1'b1;
         end else if (!id_valid && rf_decode_valid && !frontend_miss_valid) begin
            redirect_retire_fetch(retire_pc, retire_prv);
            launched = 1'b1;
         end
      end
   endtask

   task retire_prepared_fetch;
      reg early_launched;
      begin
         execute_res_valid <= 0;
         retire_now_q <= 1;
         if (npc == ex_predicted_pc) begin
            if (execute_req_matches_retire(npc, prv, fetch_epoch)) begin
               clear_frontend_fast_cmd();
            end else begin
               try_early_launch_queued_decode(npc, prv, early_launched);
               if (!early_launched)
                  prepare_current_epoch_fetch(npc, prv);
            end
         end else begin
            redirect_retire_fetch(npc, prv);
         end
         state <= `S_FETCH1;
      end
   endtask

   task retire_linear_fetch;
      reg early_launched;
      begin
         retire_now_q <= 1;
         if (execute_req_matches_retire(npc, prv, fetch_epoch)) begin
            clear_frontend_fast_cmd();
         end else if (id_matches_retire(npc, prv, fetch_epoch)) begin
            clear_frontend_fast_cmd();
         end else begin
            try_early_launch_queued_decode(npc, prv, early_launched);
            if (!early_launched)
               prepare_current_epoch_fetch(npc, prv);
         end
         state <= `S_FETCH1;
      end
   endtask

   task finish_load_writeback;
      begin
         if (write_back_fp_valid)
            write_back_fp_value = load_size_lg2[0] ?
                                  write_back_value :
                                  {32'hffffffff, write_back_value[31:0]};
      end
   endtask

   task retire_redirect_fetch;
      begin
         execute_res_valid <= 0;
         retire_now_q <= 1;
         redirect_retire_fetch(npc, prv);
         state <= `S_FETCH1;
      end
   endtask

   task retire_pre_exe_b;
      begin
         write_back_value <= pre_exe_b;
         retire_prepared_fetch();
      end
   endtask

/* verilator lint_off WIDTHTRUNC */
   task route_translated_addr;
      input [63:0] req_pa;
      input [CACHE_PERM_BITS-1:0] req_perm;
      input [ 4:0] req_return;
      begin
         mem_addr = req_pa;
         mem_va = tlb_req_va;
         mem_asid <= current_cache_asid;
         mem_perm <= req_perm;
         mem_ctx <= tlb_req_ctx;
         translated <= 1;
         if (req_return == `S_FETCH2 || req_return == `S_FETCH2_HALF) begin
            if (phys_region(mem_addr) == `REGION_BRAM ||
                phys_region(mem_addr) == `REGION_DRAM) begin
               // Cacheable instruction fetch.  Local BRAM is a cache refill
               // source now; it is no longer consumed directly by fetch.
               fetch_from_dram <= 1;
               dram_addr       <= mem_addr[30:3];
               dram_va         <= tlb_req_va;
               dram_asid       <= current_cache_asid;
               dram_perm       <= req_perm;
               dram_ctx        <= tlb_req_ctx;
               dram_instr      <= 1;
               dram_read       <= 1;
               state           <= (req_return == `S_FETCH2) ?
                                  `S_DRAM_FETCH_WAIT : `S_DRAM_FETCH_HALF_WAIT;
            end else begin
               cause = `TRAP_INSTRUCTION_ACCESS_FAULT;
               tval = req_pa;
               state <= `S_EXCEPTION;
            end
         end else begin
            state     <= {1'b0, req_return};
         end
      end
   endtask
/* verilator lint_on WIDTHTRUNC */

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
         ptw_satp     = csr_satp;
         ptw_sum      = sum;
         ptw_mxr      = mxr;
         ptw_return   = req_return;
         ptw_pte_addr <= {8'd0, csr_satp[43:0], 12'd0} + {52'd0, req_va[38:30], 3'd0};
         state        <= `S_PTW_LAUNCH;
      end
   endtask

   task start_translation;
      input [63:0] req_va;
      input [ 1:0] req_access;
      input [ 1:0] req_prv;
      input [ 4:0] req_return;
      begin
         tlb_req_va     <= req_va;
         tlb_req_access <= req_access;
         tlb_req_prv    <= req_prv;
         tlb_req_sum    <= sum;
         tlb_req_mxr    <= mxr;
         tlb_req_return <= req_return;
         tlb_4k_rd_idx  <= tlb_4k_index(req_va, req_access, req_prv, sum, mxr);
         tlb_2m_rd_idx  <= tlb_2m_index(req_va, req_access, req_prv, sum, mxr);
         state          <= `S_TLB_LOOKUP;
      end
   endtask

   task stage_tlb_insert;
      input [63:0] req_va;
      input [63:0] req_pa;
      input [ 1:0] req_level;
      input [ 1:0] req_access;
      input [ 1:0] req_prv;
      input [CACHE_PERM_BITS-1:0] req_perm;
      input [63:0] req_satp;
      input        req_sum;
      input        req_mxr;
      begin
         tlb_insert_level <= req_level;
         tlb_insert_4k_idx <= tlb_4k_index(req_va, req_access, req_prv,
                                           req_sum, req_mxr);
         tlb_insert_2m_idx <= tlb_2m_index(req_va, req_access, req_prv,
                                           req_sum, req_mxr);
         tlb_insert_4k_data <= {req_va[38:12], req_pa[63:12],
                                satp_tlb_key(req_satp),
                                req_perm,
                                {req_access, req_prv, req_sum, req_mxr}};
         tlb_insert_2m_data <= {req_va[38:21], req_pa[63:21],
                                satp_tlb_key(req_satp),
                                req_perm,
                                {req_access, req_prv, req_sum, req_mxr}};
      end
   endtask

   task commit_staged_tlb_insert;
      begin
         if (tlb_insert_level == 1) begin
`ifdef SIMULATE
            if (!tlb_2m_valid[tlb_insert_2m_idx])
               tlb_stat_entries_2m = tlb_stat_entries_2m + 1;
`endif
            hpm_tlb_insert_2m_pulse <= 1;
            if (tlb_2m_valid[tlb_insert_2m_idx])
               hpm_tlb_evict_2m_pulse <= 1;
            tlb_2m_valid[tlb_insert_2m_idx] <= 1;
            tlb_2m_wr_en <= 1;
            tlb_2m_wr_idx <= tlb_insert_2m_idx;
            tlb_2m_wr_data <= tlb_insert_2m_data;
         end else if (tlb_insert_level == 0) begin
`ifdef SIMULATE
            if (!tlb_4k_valid[tlb_insert_4k_idx])
               tlb_stat_entries_4k = tlb_stat_entries_4k + 1;
`endif
            hpm_tlb_insert_4k_pulse <= 1;
            if (tlb_4k_valid[tlb_insert_4k_idx])
               hpm_tlb_evict_4k_pulse <= 1;
            tlb_4k_valid[tlb_insert_4k_idx] <= 1;
            tlb_4k_wr_en <= 1;
            tlb_4k_wr_idx <= tlb_insert_4k_idx;
            tlb_4k_wr_data <= tlb_insert_4k_data;
         end else begin
            hpm_tlb_uncached_1g_pulse <= 1;
         end
      end
   endtask

   always @(posedge clock) begin
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */
      if (!csr_mcountinhibit[0] && hpm_mode_enabled(csr_mcyclecfg))
         csr_mcycle <= csr_mcycle + 1;
      if (core_reset_now) begin
         hpm_instret_q <= 0;
         hpm_cache_read_q <= 0;
         hpm_cache_hit_q <= 0;
         hpm_cache_miss_q <= 0;
         hpm_cache_fill_line_q <= 0;
         hpm_cache_fill_beat_q <= 0;
         hpm_cache_write_q <= 0;
         hpm_axi_read_q <= 0;
         hpm_axi_write_q <= 0;
         hpm_bus_wait_q <= 0;
         hpm_tlb_lookup_q <= 0;
         hpm_tlb_hit_q <= 0;
         hpm_tlb_miss_q <= 0;
         hpm_tlb_hit_4k_q <= 0;
         hpm_tlb_hit_2m_q <= 0;
         hpm_tlb_insert_4k_q <= 0;
         hpm_tlb_insert_2m_q <= 0;
         hpm_tlb_evict_4k_q <= 0;
         hpm_tlb_evict_2m_q <= 0;
         hpm_tlb_uncached_1g_q <= 0;
         hpm_tlb_uncached_napot_q <= 0;
         hpm_ptw_leaf_4k_q <= 0;
         hpm_ptw_leaf_2m_q <= 0;
         hpm_ptw_leaf_1g_q <= 0;
         hpm_ptw_leaf_napot_q <= 0;
      end else begin
         hpm_instret_q <= hpm_instret_pulse;
         hpm_cache_read_q <= hpm_cache_read_pulse;
         hpm_cache_hit_q <= hpm_cache_hit_pulse;
         hpm_cache_miss_q <= hpm_cache_miss_pulse;
         hpm_cache_fill_line_q <= hpm_cache_fill_line_pulse;
         hpm_cache_fill_beat_q <= hpm_cache_fill_beat_pulse;
         hpm_cache_write_q <= hpm_cache_write_pulse;
         hpm_axi_read_q <= hpm_axi_read_pulse;
         hpm_axi_write_q <= hpm_axi_write_pulse;
         hpm_bus_wait_q <= hpm_bus_wait_cycle;
         hpm_tlb_lookup_q <= hpm_tlb_lookup_pulse;
         hpm_tlb_hit_q <= hpm_tlb_hit_pulse;
         hpm_tlb_miss_q <= hpm_tlb_miss_pulse;
         hpm_tlb_hit_4k_q <= hpm_tlb_hit_4k_pulse;
         hpm_tlb_hit_2m_q <= hpm_tlb_hit_2m_pulse;
         hpm_tlb_insert_4k_q <= hpm_tlb_insert_4k_pulse;
         hpm_tlb_insert_2m_q <= hpm_tlb_insert_2m_pulse;
         hpm_tlb_evict_4k_q <= hpm_tlb_evict_4k_pulse;
         hpm_tlb_evict_2m_q <= hpm_tlb_evict_2m_pulse;
         hpm_tlb_uncached_1g_q <= hpm_tlb_uncached_1g_pulse;
         hpm_tlb_uncached_napot_q <= hpm_tlb_uncached_napot_pulse;
         hpm_ptw_leaf_4k_q <= hpm_ptw_leaf_4k_pulse;
         hpm_ptw_leaf_2m_q <= hpm_ptw_leaf_2m_pulse;
         hpm_ptw_leaf_1g_q <= hpm_ptw_leaf_1g_pulse;
         hpm_ptw_leaf_napot_q <= hpm_ptw_leaf_napot_pulse;
      end
      for (hpm_i = 0; hpm_i < `HPM_COUNTERS; hpm_i = hpm_i + 1) begin
         if (core_reset_now) begin
            csr_mhpmcounter[hpm_i] <= 0;
            csr_mhpmevent[hpm_i] <= 0;
         end else begin
            if (hpm_event_wr_en && hpm_wr_idx == hpm_i[3:0]) begin
               csr_mhpmevent[hpm_i] <= hpm_wr_data;
            end
            if (hpm_counter_wr_en && hpm_wr_idx == hpm_i[3:0]) begin
               csr_mhpmcounter[hpm_i] <= hpm_wr_data;
            end else if (!csr_mcountinhibit[hpm_i + 3] &&
                         hpm_mode_enabled(csr_mhpmevent[hpm_i]) &&
                         hpm_event_active(csr_mhpmevent[hpm_i][15:0],
                                          hpm_instret_q,
                                          hpm_cache_read_q,
                                          hpm_cache_hit_q,
                                          hpm_cache_miss_q,
                                          hpm_cache_fill_line_q,
                                          hpm_cache_fill_beat_q,
                                          hpm_cache_write_q,
                                          hpm_axi_read_q,
                                          hpm_axi_write_q,
                                          hpm_bus_wait_q,
                                          hpm_tlb_lookup_q,
                                          hpm_tlb_hit_q,
                                          hpm_tlb_miss_q,
                                          hpm_tlb_hit_4k_q,
                                          hpm_tlb_hit_2m_q,
                                          hpm_tlb_insert_4k_q,
                                          hpm_tlb_insert_2m_q,
                                          hpm_tlb_evict_4k_q,
                                          hpm_tlb_evict_2m_q,
                                          hpm_tlb_uncached_1g_q,
                                          hpm_tlb_uncached_napot_q,
                                          hpm_ptw_leaf_4k_q,
                                          hpm_ptw_leaf_2m_q,
                                          hpm_ptw_leaf_1g_q,
                                          hpm_ptw_leaf_napot_q)) begin
               if (csr_mhpmcounter[hpm_i] == 64'hffff_ffff_ffff_ffff &&
                   !csr_mhpmevent[hpm_i][`HPM_OF_BIT] &&
                   !(hpm_event_wr_en && hpm_wr_idx == hpm_i[3:0])) begin
                  csr_mhpmevent[hpm_i] <= csr_mhpmevent[hpm_i] | 64'h8000_0000_0000_0000;
                  lcofip <= 1;
               end
               csr_mhpmcounter[hpm_i] <= csr_mhpmcounter[hpm_i] + 1;
            end
         end
      end
      if (reset)
         core_reset_pending <= 1;
      if (uart_rx_valid) begin
         if (uart_rx_data == 8'h18) begin
            if (uart_break_count == 3'd4) begin
               core_reset_pending <= 1;
               uart_break_count <= 0;
               uart_rx_head <= 0;
               uart_rx_tail <= 0;
               uart_rx_front_valid <= 0;
               uart_rx_refill_pending <= 0;
            end else begin
               uart_break_count <= uart_break_count + 1'b1;
            end
         end else begin
            uart_break_count <= 0;
         end
      end
      // XXX This isn't very portable
      if (clint_mtime_clock_scaler[13]) begin
         clint_mtime_clock_scaler <= 3333 - 2; // 333.3333.. MHz / 3333 ~ 100.01 kHz
         clint_mtime <= clint_mtime + 1;
      end else
        clint_mtime_clock_scaler <= clint_mtime_clock_scaler - 1;
      uart_tx_valid <= 0;

      if (uart_rx_refill_pending) begin
         uart_rx_front <= uart_rx_fifo[uart_rx_refill_addr];
         uart_rx_front_valid <= 1;
         uart_rx_refill_pending <= 0;
      end

      // Enqueue UART RX data
      if (uart_rx_push) begin
         uart_rx_fifo[uart_rx_tail[UART_FIFO_INDEX_BITS-1:0]] <= uart_rx_data;
         uart_rx_tail <= uart_rx_tail + 1;
         if (uart_rx_empty || (uart_rx_pop && uart_rx_count == 1)) begin
            uart_rx_front <= uart_rx_data;
            uart_rx_front_valid <= 1;
            uart_rx_refill_pending <= 0;
         end
      end

`ifdef PC_TRACE
      if (!dbg_armed && uart_tx_ready && !uart_tx_valid && !uart_tx_empty) begin
`else
      if (uart_tx_ready && !uart_tx_valid && !uart_tx_empty) begin
`endif
         uart_tx_valid <= 1;
         uart_tx_data  <= uart_tx_fifo[uart_tx_head[UART_FIFO_INDEX_BITS-1:0]];
         uart_tx_head  <= uart_tx_head + 1;
         if (uart_tx_count == 1 || uart_tx_full)
            uart_thre_pending <= 1;
      end

      // PLIC gateway model: a source can become pending only while it is not
      // already pending and not in service. Claim clears pending and marks the
      // source in service; completion rearms the gateway.
      plic_pending <= plic_pending | (plic_source_level & ~plic_in_service);

      mmio_write = 0;
      mmio_read = 0;
      frontend_flush_this_cycle = 0;
      f_consumed_hit = 0;
      rf_decode_pop_this_cycle = 0;
      rf_decode_enqueue_this_cycle = 0;
      rf_decode_prearm_block = 0;
      frontend_decode_pending_drain = 0;
      frontend_buf_flush <= 1'b0;
      frontend_buf_fill <= 1'b0;
      dram_read  <= 0;
      dram_write <= 0;
      dram_instr <= 0;
      ptw_direct_read <= 0;
      cache_cbo_flush <= 0;
      hpm_counter_wr_en <= 0;
      hpm_event_wr_en <= 0;
      tlb_4k_wr_en <= 0;
      tlb_2m_wr_en <= 0;
      hpm_tlb_insert_4k_pulse <= 0;
      hpm_tlb_insert_2m_pulse <= 0;
      hpm_tlb_evict_4k_pulse <= 0;
      hpm_tlb_evict_2m_pulse <= 0;
      hpm_tlb_uncached_1g_pulse <= 0;
      hpm_tlb_uncached_napot_pulse <= 0;
      hpm_ptw_leaf_4k_pulse <= 0;
      hpm_ptw_leaf_2m_pulse <= 0;
      hpm_ptw_leaf_1g_pulse <= 0;
      hpm_ptw_leaf_napot_pulse <= 0;

      if (!core_reset_now && frontend_decode_pending_valid &&
          !rf_decode_full) begin
         enqueue_frontend_decode_pending();
      end

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
      end else begin
`endif

      // Free-running frontend FSM. Runs BEFORE case(state) so f_consumed_hit
      // (blocking assign) propagates to the backend's S_FETCH_BUF_USE arm in
      // the same cycle. F_FETCH_BUF_USE owns the simple cache-hit path:
      // latches the just-fetched instruction into frontend_decode_pending_*,
      // which the unconditional drain at the top of this always block then
      // pushes into rf_decode_*. Backend just retires from the queue.
      case (f_state)
        `F_IDLE: begin
           // Self-kick: any cycle where a fetch command is pending and the
           // queue/pending have room, start the lookup. No state-of-backend
           // gating — frontend runs independently. Vintage hazards are
           // handled by latching frontend_cmd_* into f_latched_cmd_* in
           // F_FETCH_BUF_CHECK below.
           if (!core_reset_now && !frontend_flush_this_cycle &&
               frontend_cmd_valid && !frontend_decode_pending_valid &&
               !rf_decode_full && !frontend_miss_valid && !frontend_miss_done &&
               !frontend_redirect_valid) begin
              f_state <= `F_FETCH_BUF_CHECK;
           end
        end
        `F_FETCH_BUF_CHECK: begin
           // Latch the frontend's cache-buffer response AND the cmd context
           // it corresponds to. Backend may mutate frontend_cmd_* in any
           // subsequent cycle; F_FETCH_BUF_USE consumes only f_latched_*.
           f_latched_hit             <= frontend_rsp_addr_hit;
           f_latched_insn            <= frontend_rsp_insn;
           f_latched_offset          <= frontend_rsp_offset;
           f_latched_next_pc         <= frontend_rsp_predicted_next_pc;
           f_latched_prediction_kind <= frontend_rsp_prediction_kind;
           f_latched_cmd_pc          <= frontend_cmd_pc;
           f_latched_cmd_prv         <= frontend_cmd_prv;
           f_latched_cmd_epoch       <= frontend_cmd_epoch;
           f_state                   <= `F_FETCH_BUF_USE;
        end
        `F_FETCH_BUF_USE: begin
           // Simple hit, using only the latched data — page-boundary
           // translated case and queue/pending pressure fall through to the
           // backend's arm.
           if (f_latched_hit && f_latched_full_insn_hit &&
               !(f_latched_cmd_pc[11:0] == 12'hFFE && f_latched_insn[1:0] == 2'b11 &&
                 csr_satp[63:60] == 4'd8 && f_latched_cmd_prv != 3) &&
               !frontend_decode_pending_valid &&
               !rf_decode_full &&
               !id_valid) begin
              f_consumed_hit = 1;
              latch_frontend_decode_pending(
                  f_latched_cmd_pc,
                  frontend_fallthrough_pc(f_latched_cmd_pc, f_latched_insn),
                  f_latched_next_pc,
                  f_latched_insn,
                  f_latched_cmd_prv,
                  f_latched_cmd_epoch,
                  f_latched_prediction_kind);
           end
           f_state <= `F_IDLE;
        end
        default: f_state <= `F_IDLE;
      endcase

      // Back-half EX FSM. Placed lexically before case(state) so future
      // blocking-write signals propagate.
      case (ex_state)
        `EX_IDLE: ;
        `EX_EXECUTE2: begin
           // Compute the integer-ALU writeback value from the pre-decoded
           // exe_add / exe_sext32 set in S_EXECUTE. Kicked at the same
           // cycle as state <= S_EXECUTE2 (default at top of S_EXECUTE).
           // Gate on state == S_EXECUTE2 so we don't clobber write_back_value
           // for branches whose S_EXECUTE arm overrode state to something
           // else (e.g. SC.W/D sets state to a memory path and writes its
           // own reservation-based result).
           if (state == `S_EXECUTE2 && execute_res_valid)
              write_back_value <= exe_sext32 ? {{32{exe_add[31]}}, exe_add[31:0]} : exe_add;
           ex_state <= `EX_IDLE;
        end
        default: ex_state <= `EX_IDLE;
      endcase

      case (state)
        `S_FETCH1: begin
           if (retire_now_q && !csr_mcountinhibit[2] && hpm_mode_enabled(csr_minstretcfg))
              csr_minstret <= csr_minstret + 1;
           retire_now_q <= 0;

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
           // (just_trapped set by S_EXCEPTION). retire_now_q gates out extra
           // S_FETCH1 visits introduced by F_FETCH_BUF_USE's queue path.
           if (retire_now_q && csr_mcycle != 0 && !just_trapped) begin
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
           if (retire_now_q && csr_mcycle != 0 && !just_trapped) begin
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
           execute_res_valid <= 0;

`ifdef DISASS
`include "disass.vh"
`endif
`ifdef TRACE
           if (retire_now_q && csr_mcycle) begin
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

           if (retire_now_q) pc <= npc;

           state <= `S_FETCH_REQ;

           // Use pre-registered interrupt check (computed previous cycle) for timing closure.
           // pre_intr_pending/pre_intr_cause are stable FFs; the path to state_reg is short.
           // Suppress on the S_FETCH1 right after a trap or xRET: in both cases
           // the interrupt-enable/mask was just changed and we want the target
           // instruction to retire before any newly-unmasked interrupt fires.
           cause_intr = 0;
           if (pre_intr_pending && !just_trapped && !just_xret) begin
              clear_frontend_cmd();
              squash_decode_execute();
              frontend_redirect_valid <= 0;
              cause = pre_intr_cause;
              cause_intr = 1;
              tval = 0;
              if (frontend_miss_valid || frontend_miss_done) begin
                 frontend_miss_wait_action <= FRONTEND_MISS_WAIT_EXCEPTION;
                 state <= `S_FRONTEND_MISS_WAIT;
              end else begin
                 state <= `S_EXCEPTION;
              end
           end else if (frontend_miss_done &&
                        frontend_miss_matches_retire(npc, prv, fetch_epoch)) begin
              frontend_miss_wait_action <= FRONTEND_MISS_WAIT_CONSUME;
              consume_frontend_miss();
           end else if (execute_req_matches_retire(npc, prv, fetch_epoch)) begin
              state <= `S_BRANCH_RESOLVE;
           end else if (execute_req_valid) begin
              redirect_retire_fetch(npc, prv);
              state <= `S_FETCH_REQ;
           end else if (id_matches_retire(npc, prv, fetch_epoch)) begin
              state <= `S_RF3;
           end else if (id_valid) begin
              redirect_retire_fetch(npc, prv);
              state <= `S_FETCH_REQ;
           end else if (rf_decode_valid && !frontend_miss_valid) begin
              retire_queued_decode_or_refetch();
           end else if (frontend_miss_valid || frontend_miss_done) begin
              frontend_miss_wait_action <= FRONTEND_MISS_WAIT_CONSUME;
              state <= `S_FRONTEND_MISS_WAIT;
           end else if (frontend_redirect_valid) begin
              clear_frontend_cmd();
              state <= `S_FETCH_REQ;
           end else if (frontend_cmd_fast_ready && frontend_cmd_valid) begin
              clear_frontend_fast_cmd();
              state <= `S_FETCH_BUF_CHECK;
              f_state <= `F_FETCH_BUF_CHECK;
           end else begin
              prepare_current_epoch_fetch(npc, prv);
           end
        end

        `S_FETCH_REQ: begin
`ifdef SIMULATE
           if (fetch_buf_summary_enabled) begin
              if (frontend_rsp_hit) begin
                 fetch_buf_stat_hits <= fetch_buf_stat_hits + 1;
              end else begin
                 fetch_buf_stat_misses <= fetch_buf_stat_misses + 1;
                 if (fetch_buf_stat_misses[17:0] == 18'h3ffff)
                    $display("%05d FETCHBUF SUMMARY hits=%0d misses=%0d",
                             $time,
                             fetch_buf_stat_hits + (frontend_rsp_hit ? 64'd1 : 64'd0),
                             fetch_buf_stat_misses + 64'd1);
              end
           end
`endif

           if (frontend_redirect_valid) begin
              issue_frontend_redirect();
              state <= `S_FETCH_REQ;
           end else if (!frontend_cmd_valid) begin
              clear_frontend_fast_cmd();
              state <= `S_FETCH1;
           end else if ((csr_satp[63:60] != 4'd8 || frontend_cmd_prv == 2'd3) &&
                        !frontend_physical_fetch_ok(frontend_cmd_pc)) begin
`ifdef SIMULATE
`ifdef VERBOSE
              $display("%05d   %1d %x illegal fetch address csr_satp[63:60] = %d",
                       $time, frontend_cmd_prv, frontend_cmd_pc, csr_satp[63:60]);
`endif
`endif
              cause = `TRAP_INSTRUCTION_ACCESS_FAULT;
              tval = 0;
              clear_frontend_cmd();
              squash_decode_execute();
              if (frontend_miss_valid || frontend_miss_done) begin
                 frontend_miss_wait_action <= FRONTEND_MISS_WAIT_EXCEPTION;
                 state <= `S_FRONTEND_MISS_WAIT;
              end else begin
                 state <= `S_EXCEPTION;
              end
           end else begin
              state <= `S_FETCH_BUF_CHECK;
              f_state <= `F_FETCH_BUF_CHECK;
           end
        end

        `S_FETCH_BUF_CHECK: begin
           if ((csr_satp[63:60] != 4'd8 || frontend_cmd_prv == 2'd3) &&
               !frontend_physical_fetch_ok(frontend_cmd_pc)) begin
`ifdef SIMULATE
`ifdef VERBOSE
              $display("%05d   %1d %x illegal fetch address csr_satp[63:60] = %d",
                       $time, frontend_cmd_prv, frontend_cmd_pc, csr_satp[63:60]);
`endif
`endif
              cause = `TRAP_INSTRUCTION_ACCESS_FAULT;
              tval = 0;
              clear_frontend_cmd();
              squash_decode_execute();
              if (frontend_miss_valid || frontend_miss_done) begin
                 frontend_miss_wait_action <= FRONTEND_MISS_WAIT_EXCEPTION;
                 state <= `S_FRONTEND_MISS_WAIT;
              end else begin
                 state <= `S_EXCEPTION;
              end
           end else begin
              // Latching now happens in case(f_state) F_FETCH_BUF_CHECK arm.
              state                    <= `S_FETCH_BUF_USE;
           end
        end

        `S_FETCH_BUF_USE: begin
`ifdef SIMULATE
           if (fetch_buf_summary_enabled) begin
              if (f_latched_hit && f_latched_full_insn_hit)
                 fetch_buf_stat_hits <= fetch_buf_stat_hits + 1;
              else begin
                 fetch_buf_stat_misses <= fetch_buf_stat_misses + 1;
                 if (fetch_buf_stat_misses[17:0] == 18'h3ffff)
                    $display("%05d FETCHBUF SUMMARY hits=%0d misses=%0d",
                             $time,
                             fetch_buf_stat_hits +
                             ((f_latched_hit && f_latched_full_insn_hit) ? 64'd1 : 64'd0),
                             fetch_buf_stat_misses + 64'd1);
              end
           end
`endif
           if (f_consumed_hit) begin
              // Frontend just latched this fetch into frontend_decode_pending_*
              // via case(f_state) earlier in the cycle. Queue drain happens at
              // the top of this always block; backend returns to retire.
              state <= `S_FETCH1;
           end else if (f_latched_hit && f_latched_full_insn_hit) begin
              accept_instruction_fetch(frontend_cmd_pc,
                                       f_latched_next_pc,
                                       f_latched_insn,
                                       f_latched_prediction_kind,
                                       1'b0);
           end else if (!cache_idle) begin
              state <= `S_FETCH_BUF_USE;
           end else begin
              start_instruction_fetch_miss(frontend_cmd_pc, frontend_cmd_prv);
           end
        end

        `S_FRONTEND_MISS_WAIT: begin
           if (frontend_miss_done) begin
              if (frontend_miss_wait_action == FRONTEND_MISS_WAIT_EXCEPTION) begin
                 frontend_miss_valid <= 0;
                 frontend_miss_done  <= 0;
                 state <= `S_EXCEPTION;
              end else if (frontend_miss_wait_action == FRONTEND_MISS_WAIT_CONSUME &&
                           frontend_miss_matches_retire(npc, prv, fetch_epoch)) begin
                 consume_frontend_miss();
              end else begin
                 frontend_miss_valid <= 0;
                 frontend_miss_done  <= 0;
                 redirect_retire_fetch(npc, prv);
                 state <= `S_FETCH_REQ;
              end
           end else begin
              try_issue_queued_decode_preserve_state(1'b1,
                                                     1'b0, 5'd0,
                                                     1'b0, 5'd0);
           end
        end

        `S_FETCH2: begin
           // Cache-backed fetches arrive through S_FETCH2_DRAM.
           state <= `S_FETCH1;
        end

        `S_FETCH2_DRAM: begin
           // Cross-doubleword case (pc[2:1]==2'b11 && insn[1:0]==2'b11) is
           // detected before RF launch and re-fetched on the slow path; the
           // upper 64 bits are don't-care.
           aligned = dram_latched_next_valid ? {dram_latched_next, dram_latched}
                                              : {64'bx, dram_latched};
           if (fetch_buf_fill_page_ok && dram_latched_next_valid) begin
              frontend_buf_fill         <= 1'b1;
              frontend_buf_fill_base_va <= fetch_buf_fill_base_va;
              frontend_buf_fill_prv     <= frontend_cmd_prv;
              frontend_buf_fill_asid    <= frontend_cmd_asid;
              frontend_buf_fill_data    <= aligned;
           end
           insn = aligned >> (frontend_cmd_pc[2:1] * 16);
           accept_instruction_fetch(frontend_cmd_pc,
                                    frontend_fallthrough_pc(frontend_cmd_pc, insn),
                                    insn, 2'd0, 1'b1);
        end

        `S_RF2: begin
           // One-cycle wait: BRAM samples new rs1/rs2 from dispatch; output
           // settles in S_RF3.
           if (id_valid) begin
              id_rf_ready <= 1;
              state <= `S_RF3;
           end else begin
              id_rf_ready <= 0;
              state <= `S_FETCH1;
           end
        end

        `S_RF3: begin
           if (!id_valid) begin
              state <= `S_FETCH1;
           end else if (!id_rf_ready) begin
              id_rf_ready <= 1;
              state <= `S_RF3;
           end else if (!id_ex_fire) begin
              state <= `S_RF3;
           end else begin
              prepare_execute_req_from_id(1'b0);
           end
        end

        `S_FETCH1B: begin
           // BRAM fetches now use the cache refill path.  This state is kept
           // only as a defensive sink for stale encoded states.
           state <= `S_FETCH1;
        end

        `S_LOAD_LATCH: begin
           // Legacy landing state. New load/AMO paths go directly to
           // S_LOAD_ALIGN; keep this as a defensive sink for stale states.
           state <= `S_LOAD_ALIGN;
        end

        `S_BRANCH_RESOLVE: begin
           // Branch metadata is prepared at the RF->EX boundary. This arm is
           // now only a transition point before S_EXECUTE.
           if (!execute_req_valid)
              state <= `S_FETCH1;
           else
              state <= `S_EXECUTE;
        end

        `S_EXECUTE: begin
           if (!execute_req_valid) begin
              state <= `S_FETCH1;
           end else begin
           pc <= ex_pc;
           insn <= ex_insn;
           execute_req_valid <= 0;
           execute_res_valid <= 1;
           state <= `S_EXECUTE2; // Default: complete write_back_value
           ex_state <= `EX_EXECUTE2; // case(ex_state) owns write_back_value compute
           prv_retire <= prv;    // snapshot pre-execution prv (MRET/SRET mutate prv below)

           imm_i = {{52{ex_insn[31]}},ex_insn[31:20]};
           imm_j = {{44{ex_insn[31]}},ex_insn[19:12],ex_insn[20],ex_insn[30:21],1'd0};
           imm_b = {{52{ex_insn[31]}},ex_insn[7],ex_insn[30:25],ex_insn[11:8],1'd0};
           imm_u = {{32{ex_insn[31]}},ex_insn[31:12],12'd0};
           imm_s = {{52{ex_insn[31]}},ex_insn[31:25],ex_insn[11:7]};

           c_nzuimm107_1211_5_6_x4 = {ex_insn[10:7],ex_insn[12:11],ex_insn[5],ex_insn[6],2'd0};
           c_uimm5_1210_6_x4       = {ex_insn[5],ex_insn[12:10],ex_insn[6],2'd0};
           c_imm12_62              = {{59{ex_insn[12]}},ex_insn[6:2]};
           c_imm12_43_5_2_6_x16    = {{55{ex_insn[12]}},ex_insn[4:3],ex_insn[5],ex_insn[2],ex_insn[6],4'd0};
           c_imm12_8_109_6_7_2_11_53_x2
                                   = {{53{ex_insn[12]}},ex_insn[8],ex_insn[10:9],ex_insn[6],ex_insn[7],ex_insn[2],ex_insn[11],ex_insn[5:3],
                                      1'd0};
           c_imm12_65_2_1110_43_x2 = {{56{ex_insn[12]}},ex_insn[6:5],ex_insn[2],ex_insn[11:10],ex_insn[4:3],1'd0};
           c_uimm42_12_65_x8       = {ex_insn[4:2],ex_insn[12],ex_insn[6:5],3'd0};
           c_uimm32_12_64_x4       = {ex_insn[3:2],ex_insn[12],ex_insn[6:4],2'd0};
           c_uimm97_1210_x8        = {ex_insn[9:7],ex_insn[12:10],3'd0};
           c_uimm87_129_x4         = {ex_insn[8:7],ex_insn[12:9],2'd0};

           c_uimm65_1210_x8        = {ex_insn[6:5],ex_insn[12:10],3'd0};

           csrno                   = ex_insn[31:20];

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
           // One s1+pre_mem_offset adder replaces 22 parallel copies,
           // shrinking the mem_addr critical path from ~14 LUT levels to ~7.
           if (pre_mem_op != `MEMOP_NONE) begin
              if (pre_mem_fp && fs == 0) begin
                 cause = `TRAP_ILLEGAL_INSTRUCTION;
                 tval = ex_insn;
                 state <= `S_EXCEPTION;
              end else begin
              if (pre_mem_fp) fs = 3;
              write_back_register    = pre_mem_fp ? 5'd0 : pre_mem_wb_reg;
              write_back_fp_valid    = pre_mem_fp && pre_mem_op == `MEMOP_LOAD;
              write_back_fp_register = pre_mem_wb_reg;
              mem_addr      = s1 + pre_mem_offset;
              mem_va        = s1 + pre_mem_offset;
              mem_asid      <= {TLB_ASID_BITS{1'b0}};
              mem_perm      <= CACHE_PERM_PHYS;
              mem_ctx       <= {((pre_mem_op == `MEMOP_STORE || pre_mem_op == `MEMOP_SC) ? 2'd2 :
                                 (pre_mem_op == `MEMOP_AMO ? 2'd3 : 2'd1)),
                                (mprv ? mpp : prv), sum, mxr};
              load_size_lg2 = pre_load_size_lg2;
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
                       `MEMOP_LOAD: state <= `S_LOAD_ALIGN;
                       `MEMOP_STORE: begin
                          mem_wr_mask = pre_mem_wr_mask;
                          store_value = pre_mem_fp ? f2 : s2;
                          state <= `S_STORE;
                       end
                       `MEMOP_LR: begin
                          reservation <= s1;
                          state <= `S_LOAD_ALIGN;
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
                          state <= `S_LOAD_ALIGN;
                       end
                       default: ;
                    endcase
                 end
              end
              end // else: !(pre_mem_fp && fs == 0)
           end

           // Quadrant 0
           else if ((ex_insn & 'he003) == 'h0000) begin // C.ADDI4SPN/illegal
              write_back_register = ex_rs2;
              if ((ex_insn & 'hffff) == 0) begin
                 write_back_register = 0;
                 cause = `TRAP_ILLEGAL_INSTRUCTION;
                 tval = ex_insn;
                 state <= `S_EXCEPTION;
              end
           end

           // Compressed integer/FP loads and stores are handled by the shared mem block above.

              // Quadrant 1
           else if (ex_insn == 1) begin // C.NOP
             // NOP
             retire_prepared_fetch();
           end

           else if ((ex_insn & 'he003) == 'h0001) begin // C.ADDI
              write_back_register = ex_rs1;
           end

           else if ((ex_insn & 'he003) == 'h2001) begin // C.ADDIW
              write_back_register = ex_rs1;
           end

           else if ((ex_insn & 'he003) == 'h4001) begin // C.LI
              write_back_register = ex_insn[11:7];
              retire_pre_exe_b();
           end

           else if ((ex_insn & 'hef83) == 'h6101) begin // C.ADDI16SP
              write_back_register = ex_rs1;
           end

           else if ((ex_insn & 'he003) == 'h6001) begin // C.LUI
              write_back_register = ex_rs1;
              retire_pre_exe_b();
           end

           else if ((ex_insn & 'hec03) == 'h8001) begin // C.SRLI
              write_back_register = ex_rs1;
           end

           else if ((ex_insn & 'hec03) == 'h8401) begin // C.SRAI
              write_back_register = ex_rs1;
           end

           else if ((ex_insn & 'hec03) == 'h8801) begin // C.ANDI
              write_back_register = ex_rs1;
              write_back_value <= s1 & pre_exe_b;
              retire_prepared_fetch();
           end

           else if ((ex_insn & 'hfc63) == 'h8c01) begin // C.SUB
              write_back_register = ex_rs1;
           end

           else if ((ex_insn & 'hfc63) == 'h8c21) begin // C.XOR
              write_back_register = ex_rs1;
              write_back_value <= s1 ^ pre_exe_b;
              retire_prepared_fetch();
           end

           else if ((ex_insn & 'hfc63) == 'h8c41) begin // C.OR
              write_back_register = ex_rs1;
              write_back_value <= s1 | pre_exe_b;
              retire_prepared_fetch();
           end

           else if ((ex_insn & 'hfc63) == 'h8c61) begin // C.AND
              write_back_register = ex_rs1;
              write_back_value <= s1 & pre_exe_b;
              retire_prepared_fetch();
           end

           else if ((ex_insn & 'hfc63) == 'h9c01) begin // C.SUBW
              write_back_register = ex_rs1;
           end

           else if ((ex_insn & 'hfc63) == 'h9c21) begin // C.ADDW
              write_back_register = ex_rs1;
           end

           else if ((ex_insn & 'he003) == 'ha001) begin // C.J
              retire_prepared_fetch();
           end

           else if ((ex_insn & 'he003) == 'hc001) begin // C.BEQZ
              if (pre_branch_taken) npc = pre_branch_target;
              retire_prepared_fetch();
           end

           else if ((ex_insn & 'he003) == 'he001) begin // C.BNEZ
              if (pre_branch_taken) npc = pre_branch_target;
              retire_prepared_fetch();
           end


              // Quadrant 2
           else if ((ex_insn & 'he003) == 'h0002) begin // C.SLLI
              write_back_register = ex_rs1;
           end

           // C.LWSP / C.LDSP / C.FLDSP handled by shared mem block above.

           else if ((ex_insn & 'hf07f) == 'h8002) begin // C.JR
              npc = pre_jalr_target;
              retire_prepared_fetch();
           end

           else if ((ex_insn & 'hf003) == 'h8002) begin // C.MV
              write_back_register = ex_rs1;
              retire_pre_exe_b();
           end

           else if ((ex_insn & 'hffff) == 'h9002) begin // C.EBREAK
              cause = `TRAP_BREAKPOINT;
              tval = 0;
              state <= `S_EXCEPTION;
           end

           else if ((ex_insn & 'hf07f) == 'h9002) begin // C.JALR
              write_back_register = 1;
              npc = pre_jalr_target;
              retire_pre_exe_b();
           end

           else if ((ex_insn & 'hf003) == 'h9002) begin // C.ADD
              write_back_register = ex_rs1;
           end

           // C.SWSP / C.SDSP / C.FSDSP handled by shared mem block above.

           // Quadrant 3, uncompressed
           else if ((ex_insn & 'h0000007f) == 'h00000037) begin // LUI
              write_back_register = ex_rd;
              retire_pre_exe_b();
           end

           else if ((ex_insn & 'h0000007f) == 'h00000017) begin // AUIPC
              write_back_register = ex_rd;
              retire_pre_exe_b();
           end

           else if ((ex_insn & 'h0000007f) == 'h0000006f) begin // JAL
              write_back_register = ex_rd;
              retire_pre_exe_b();
           end

           else if ((ex_insn & 'h0000707f) == 'h00000067) begin // JALR
              write_back_register = ex_rd;
              npc = pre_jalr_target;
              retire_pre_exe_b();
           end

           else if ((ex_insn & 'h0000707f) == 'h00000063) begin // BEQ
              if (pre_branch_taken) npc = pre_branch_target;
              retire_prepared_fetch();
           end

           else if ((ex_insn & 'h0000707f) == 'h00001063) begin // BNE
              if (pre_branch_taken) npc = pre_branch_target;
              retire_prepared_fetch();
           end

           else if ((ex_insn & 'h0000707f) == 'h00004063) begin // BLT
              if (pre_branch_taken) npc = pre_branch_target;
              retire_prepared_fetch();
           end

           else if ((ex_insn & 'h0000707f) == 'h00005063) begin // BGE
              if (pre_branch_taken) npc = pre_branch_target;
              retire_prepared_fetch();
           end

           else if ((ex_insn & 'h0000707f) == 'h00006063) begin // BLTU
              if (pre_branch_taken) npc = pre_branch_target;
              retire_prepared_fetch();
           end

           else if ((ex_insn & 'h0000707f) == 'h00007063) begin // BGEU
              if (pre_branch_taken) npc = pre_branch_target;
              retire_prepared_fetch();
           end

           // LB/LH/LW/LD/LBU/LHU/LWU and SB/SH/SW/SD handled by shared mem block above.

           else if ((ex_insn & 'h0000707f) == 'h00000013) begin // ADDI
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'h0000707f) == 'h00002013) begin // SLTI
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'h0000707f) == 'h00003013) begin // SLTIU
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'h0000707f) == 'h00004013) begin // XORI
              write_back_register = ex_rd;
              write_back_value <= s1 ^ pre_exe_b;
              retire_prepared_fetch();
           end

           else if ((ex_insn & 'h0000707f) == 'h00006013) begin // ORI
              write_back_register = ex_rd;
              write_back_value <= s1 | pre_exe_b;
              retire_prepared_fetch();
           end

           else if ((ex_insn & 'h0000707f) == 'h00007013) begin // ANDI
              write_back_register = ex_rd;
              write_back_value <= s1 & pre_exe_b;
              retire_prepared_fetch();
           end

           else if ((ex_insn & 'hfe00707f) == 'h00000033) begin // ADD
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h40000033) begin // SUB
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h00001033) begin // SLL
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h00002033) begin // SLT
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h00003033) begin // SLTU
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h00004033) begin // XOR
              write_back_register = ex_rd;
              write_back_value <= s1 ^ pre_exe_b;
              retire_prepared_fetch();
           end

           else if ((ex_insn & 'hfe00707f) == 'h00005033) begin // SRL
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h40005033) begin // SRA
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h00006033) begin // OR
              write_back_register = ex_rd;
              write_back_value <= s1 | pre_exe_b;
              retire_prepared_fetch();
           end

           else if ((ex_insn & 'hfe00707f) == 'h00007033) begin // AND
              write_back_register = ex_rd;
              write_back_value <= s1 & pre_exe_b;
              retire_prepared_fetch();
           end

           else if ((ex_insn & 'hf000707f) == 'h0000000f) begin // FENCE
              // Nothing to do here
              retire_prepared_fetch();
           end

           else if ((ex_insn & 'hf000707f) == 'h8000000f) begin // FENCE.TSO
              // Nothing to do here
              retire_prepared_fetch();
           end

           else if ((ex_insn & 'hfff0707f) == 'h0000200f || // CBO.INVAL
                    (ex_insn & 'hfff0707f) == 'h0010200f || // CBO.CLEAN
                    (ex_insn & 'hfff0707f) == 'h0020200f) begin // CBO.FLUSH
              mem_addr = s1;
              mem_va = s1;
              mem_asid <= {TLB_ASID_BITS{1'b0}};
              mem_perm <= CACHE_PERM_PHYS;
              mem_ctx <= {2'd2, (mprv ? mpp : prv), sum, mxr};
              translated <= 0;
              if (csr_satp[63:60] == 4'd8 && (mprv ? mpp : prv) != 3)
                 start_translation(mem_addr, 2'd2, mprv ? mpp : prv, `S_CBO_EXEC);
              else
                 state <= `S_CBO_EXEC;
           end

           else if ((ex_insn & 'hffffffff) == 'h00000073) begin // ECALL
              cause = `TRAP_ENVIRONMENT_CALL_FROM_U_MODE + prv;
              tval = 0;
              state <= `S_EXCEPTION;
`ifdef SIMULATE
`ifdef VERBOSE
              $display("ECALL: pc %x prv %d time %0t", ex_pc, prv, $time);
`endif
`endif
           end

           else if ((ex_insn & 'hffffffff) == 'h00100073) begin // EBREAK
              cause = `TRAP_BREAKPOINT;
              tval = 0;
              state <= `S_EXCEPTION;
           end

           else if ((ex_insn & 'hfc00707f) == 'h00001013) begin // SLLI
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfc00707f) == 'h00005013) begin // SRLI
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfc00707f) == 'h40005013) begin // SRAI
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'h0000707f) == 'h0000001b) begin // ADDIW
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h0000101b) begin // SLLIW
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h0000501b) begin // SRLIW
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h4000501b) begin // SRAIW
              // NB: Yes, this is a crazy instruction with *two*
              // sign-extensions and it does _not_ behave like the MIPS
              // counterpart
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h0000003b) begin // ADDW
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h4000003b) begin // SUBW
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h0000103b) begin // SLLW
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h0000503b) begin // SRLW
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hfe00707f) == 'h4000503b) begin // SRAW
              // NB: Yes, this is a crazy instruction with *two*
              // sign-extensions and it does _not_ behave like the MIPS
              // counterpart
              write_back_register = ex_rd;
           end

           else if ((ex_insn & 'hffffffff) == 'h0000100f) begin // FENCE.I
              flush_frontend_speculation;
              fetch_epoch <= fetch_epoch + 1'b1;
              vhpr_request_full_flush(8'd1);
              execute_res_valid <= 0;
              state <= `S_CBO_WAIT;
           end

           else if ((ex_insn & 'h0000707f) == 'h00001073) begin // CSRRW
              // CSRRW and CSRRWI (and only those) do not read the CSR
              // if rd == 0 This matters [only] if the read has side
              // effects (I'm guilty of this part of RISC-V semantics).
              csr_op = `CSR_OP_COPY;
              csr_arg = s1;
              state <= `S_HANDLE_CSR;
           end

           else if ((ex_insn & 'h0000707f) == 'h00002073) begin // CSRRS
              csr_op = `CSR_OP_OR;
              csr_arg = s1;
              state <= `S_HANDLE_CSR;
           end

           else if ((ex_insn & 'h0000707f) == 'h00003073) begin // CSRRC
              csr_op = `CSR_OP_ANDN;
              csr_arg = s1;
              state <= `S_HANDLE_CSR;
           end

           else if ((ex_insn & 'h0000707f) == 'h00005073) begin // CSRRWI
              csr_op = `CSR_OP_COPY;
              csr_arg = ex_rs1;
              state <= `S_HANDLE_CSR;
           end

           else if ((ex_insn & 'h0000707f) == 'h00006073) begin // CSRRSI
              csr_op = `CSR_OP_OR;
              csr_arg = ex_rs1;
              state <= `S_HANDLE_CSR;
           end

           else if ((ex_insn & 'h0000707f) == 'h00007073) begin // CSRRCI
              csr_op = `CSR_OP_ANDN;
              csr_arg = ex_rs1;
              state <= `S_HANDLE_CSR;
           end

           else if ((ex_insn & 'hfe00707f) == 'h02000033) begin // MUL
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_MUL;
              state <= `S_MULDIV_START;
           end

           else if ((ex_insn & 'hfe00707f) == 'h02001033) begin // MULH
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_MULH;
              state <= `S_MULDIV_START;
           end

           else if ((ex_insn & 'hfe00707f) == 'h02002033) begin // MULHSU
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_MULHSU;
              state <= `S_MULDIV_START;
           end

           else if ((ex_insn & 'hfe00707f) == 'h02003033) begin // MULHU
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_MULHU;
              state <= `S_MULDIV_START;
           end


           else if ((ex_insn & 'hfe00707f) == 'h02004033) begin // DIV
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_DIV;
              state <= `S_MULDIV_START;
           end

           else if ((ex_insn & 'hfe00707f) == 'h02005033) begin // DIVU
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_DIVU;
              state <= `S_MULDIV_START;
           end

           else if ((ex_insn & 'hfe00707f) == 'h02006033) begin // REM
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_REM;
              state <= `S_MULDIV_START;
           end

           else if ((ex_insn & 'hfe00707f) == 'h02007033) begin // REMU
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_REMU;
              state <= `S_MULDIV_START;
           end

           else if ((ex_insn & 'hfe00707f) == 'h0200003b) begin // MULW
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_MULW;
              state <= `S_MULDIV_START;
           end

           else if ((ex_insn & 'hfe00707f) == 'h0200403b) begin // DIVW
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_DIVW;
              state <= `S_MULDIV_START;
           end

           else if ((ex_insn & 'hfe00707f) == 'h0200503b) begin // DIVUW
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_DIVUW;
              state <= `S_MULDIV_START;
           end

           else if ((ex_insn & 'hfe00707f) == 'h0200603b) begin // REMW
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_REMW;
              state <= `S_MULDIV_START;
           end

           else if ((ex_insn & 'hfe00707f) == 'h0200703b) begin // REMUW
              write_back_register = ex_rd;
              muldiv_start_op <= `MULDIV_REMUW;
              state <= `S_MULDIV_START;
           end

           // LR.W/D, SC.W/D, and all AMO*.W/D variants handled by shared mem block above.

           else if ((ex_insn & 'hffffffff) == 'h30200073) begin // MRET
              frontend_buf_flush <= 1'b1;
              if (mpp != 3) mprv = 0;
              prv = mpp;
              mpp = 0;
              mie = mpie;
              mpie = 1;
              npc = csr_mepc;
              just_xret <= 1;
              retire_now_q <= 1; // MRET retires; bypasses retire helpers
              state <= `S_FETCH1;
           end

           else if ((ex_insn & 'hffffffff) == 'h10200073) begin // SRET
              if (prv == 0 || prv == 1 && tsr) begin
                 cause = `TRAP_ILLEGAL_INSTRUCTION;
                 tval = ex_insn;
                 state <= `S_EXCEPTION;
              end else begin
                 frontend_buf_flush <= 1'b1;
`ifdef SIMULATE
`ifdef VERBOSE
                 $display("SRET: pc %x prv %d->%d sepc %x time %0t", ex_pc, prv, spp, csr_sepc, $time);
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

           else if ((ex_insn & 'hfe007fff) == 'h12000073) begin // SFENCE.VMA
              if (prv < 1 || prv == 1 && tvm) begin
                 cause = `TRAP_ILLEGAL_INSTRUCTION;
                 tval = ex_insn;
                 state <= `S_EXCEPTION;
             end else begin
                 flush_frontend_speculation;
                 fetch_epoch <= fetch_epoch + 1'b1;
                 flush_tlb;
                 vhpr_request_full_flush(8'd2);
                 execute_res_valid <= 0;
                 state <= `S_CBO_WAIT;
              end
           end

           else if ((ex_insn & 'hffffffff) == 'h10500073) begin // WFI
              if (prv == 0 || prv == 1 && tw) begin
                 cause = `TRAP_ILLEGAL_INSTRUCTION;
                 tval = ex_insn;
                 state <= `S_EXCEPTION;
              end else begin
                 retire_prepared_fetch(); // treat as NOP (no real sleep in simulation)
              end
           end

           // OP-FP (opcode 0x53) — arithmetic-free Phase 1 insns:
           // FMV.{W.X,X.W,D.X,X.D}, FSGNJ{,N,X}.{S,D}, FCLASS.{S,D}.
           // Everything else in this opcode falls through to illegal.
           else if (ex_insn[6:0] == 7'b1010011) begin
              if (fs == 0) begin
                 // FP state disabled by mstatus.FS: any FP instruction traps.
                 cause = `TRAP_ILLEGAL_INSTRUCTION;
                 tval = ex_insn;
                 state <= `S_EXCEPTION;
              end else begin
                 // Reaching an FP instruction dirties the FP state.
                 fs = 3;
                 case (ex_insn[31:25])
                   // FADD.S / FSUB.S
                   7'b0000000,
                   7'b0000100: begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         cvfpu_operands[0] <= 64'd0;
                         cvfpu_operands[1] <= f1;
                         cvfpu_operands[2] <= f2;
                         cvfpu_rnd_mode <= pre_fp_rnd_mode;
                         cvfpu_op       <= 4'd2; // fpnew_pkg::ADD
                         cvfpu_op_mod   <= ex_insn[27]; // 0=add, 1=sub
                         cvfpu_src_fmt  <= 3'd0; // fpnew_pkg::FP32
                         cvfpu_dst_fmt  <= 3'd0; // fpnew_pkg::FP32
                         cvfpu_int_fmt  <= 2'd3; // fpnew_pkg::INT64 (unused)
                         cvfpu_tag_in   <= {3'd0, ex_rd};
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
`endif
                   end
                   // FADD.D / FSUB.D
                   7'b0000001,
                   7'b0000101: begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         cvfpu_operands[0] <= 64'd0;
                         cvfpu_operands[1] <= f1;
                         cvfpu_operands[2] <= f2;
                         cvfpu_rnd_mode <= pre_fp_rnd_mode;
                         cvfpu_op       <= 4'd2; // fpnew_pkg::ADD
                         cvfpu_op_mod   <= ex_insn[27]; // 0=add, 1=sub
                         cvfpu_src_fmt  <= 3'd1; // fpnew_pkg::FP64
                         cvfpu_dst_fmt  <= 3'd1; // fpnew_pkg::FP64
                         cvfpu_int_fmt  <= 2'd3; // fpnew_pkg::INT64 (unused)
                         cvfpu_tag_in   <= {3'd0, ex_rd};
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
`endif
                   end
                   // FMUL.S
                   7'b0001000: begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
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
                         cvfpu_tag_in   <= {3'd0, ex_rd};
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
`endif
                   end
                   // FMUL.D
                   7'b0001001: begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
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
                         cvfpu_tag_in   <= {3'd0, ex_rd};
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
`endif
                   end
                   // FDIV.S
                   7'b0001100: begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
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
                         cvfpu_tag_in   <= {3'd0, ex_rd};
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
`endif
                   end
                   // FDIV.D
                   7'b0001101: begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
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
                         cvfpu_tag_in   <= {3'd0, ex_rd};
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
`endif
                   end
                   // FSQRT.S
                   7'b0101100: if (ex_insn[24:20] == 5'd0) begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
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
                         cvfpu_tag_in   <= {3'd0, ex_rd};
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
`endif
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FSQRT.D
                   7'b0101101: if (ex_insn[24:20] == 5'd0) begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
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
                         cvfpu_tag_in   <= {3'd0, ex_rd};
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
`endif
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FMIN.S / FMAX.S
                   7'b0010100: begin
`ifdef USE_CVFPU
                      if (ex_insn[14:12] > 3'b001) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         cvfpu_operands[0] <= f1;
                         cvfpu_operands[1] <= f2;
                         cvfpu_operands[2] <= 64'd0;
                         cvfpu_rnd_mode <= {2'b00, ex_insn[12]}; // RNE=min, RTZ=max
                         cvfpu_op       <= 4'd7; // fpnew_pkg::MINMAX
                         cvfpu_op_mod   <= 1'b0;
                         cvfpu_src_fmt  <= 3'd0; // fpnew_pkg::FP32
                         cvfpu_dst_fmt  <= 3'd0; // fpnew_pkg::FP32
                         cvfpu_int_fmt  <= 2'd3; // fpnew_pkg::INT64 (unused)
                         cvfpu_tag_in   <= {3'd0, ex_rd};
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
`endif
                   end
                   // FMIN.D / FMAX.D
                   7'b0010101: begin
`ifdef USE_CVFPU
                      if (ex_insn[14:12] > 3'b001) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         cvfpu_operands[0] <= f1;
                         cvfpu_operands[1] <= f2;
                         cvfpu_operands[2] <= 64'd0;
                         cvfpu_rnd_mode <= {2'b00, ex_insn[12]}; // RNE=min, RTZ=max
                         cvfpu_op       <= 4'd7; // fpnew_pkg::MINMAX
                         cvfpu_op_mod   <= 1'b0;
                         cvfpu_src_fmt  <= 3'd1; // fpnew_pkg::FP64
                         cvfpu_dst_fmt  <= 3'd1; // fpnew_pkg::FP64
                         cvfpu_int_fmt  <= 2'd3; // fpnew_pkg::INT64 (unused)
                         cvfpu_tag_in   <= {3'd0, ex_rd};
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
`endif
                   end
                   // FCVT.S.D
                   7'b0100000: if (ex_insn[24:20] == 5'd1) begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
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
                         cvfpu_tag_in   <= {3'd0, ex_rd};
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
`endif
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FCVT.D.S
                   7'b0100001: if (ex_insn[24:20] == 5'd0) begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
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
                         cvfpu_tag_in   <= {3'd0, ex_rd};
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
`endif
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FSGNJ/N/X .S — NaN-box-check operands; NaN-box result.
                   7'b0010000: begin
                      write_back_fp_valid    = 1;
                      write_back_fp_register = ex_rd;
                      case (ex_insn[14:12])
                        3'b000: write_back_fp_value <= {32'hffffffff, f2_s[31],             f1_s[30:0]};
                        3'b001: write_back_fp_value <= {32'hffffffff, ~f2_s[31],            f1_s[30:0]};
                        3'b010: write_back_fp_value <= {32'hffffffff, f2_s[31] ^ f1_s[31],  f1_s[30:0]};
                        default: begin
                           write_back_fp_valid = 0;
                           cause = `TRAP_ILLEGAL_INSTRUCTION;
                           tval = ex_insn;
                           state <= `S_EXCEPTION;
                        end
                      endcase
                      if (ex_insn[14:12] < 3) retire_linear_fetch();
                   end
                   // FSGNJ/N/X .D — no boxing check; 64-bit direct.
                   7'b0010001: begin
                      write_back_fp_valid    = 1;
                      write_back_fp_register = ex_rd;
                      case (ex_insn[14:12])
                        3'b000: write_back_fp_value <= {f2[63],         f1[62:0]};
                        3'b001: write_back_fp_value <= {~f2[63],        f1[62:0]};
                        3'b010: write_back_fp_value <= {f2[63] ^ f1[63], f1[62:0]};
                        default: begin
                           write_back_fp_valid = 0;
                           cause = `TRAP_ILLEGAL_INSTRUCTION;
                           tval = ex_insn;
                           state <= `S_EXCEPTION;
                        end
                      endcase
                      if (ex_insn[14:12] < 3) retire_linear_fetch();
                   end
                   // FEQ.S / FLT.S / FLE.S: integer rd; NV flag on NaN per op.
                   7'b1010000: if (ex_insn[14:12] <= 3'b010) begin
                      fcmp_result = fcmp_s(ex_insn[14:12], f1_s, f2_s);
                      write_back_register = ex_rd;
                      fp_int_result       <= {63'd0, fcmp_result[0]};
                      fp_int_fflags       <= fcmp_result[1] ? 5'b10000 : 5'd0;
                      state               <= `S_FP_INT_COMMIT;
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FEQ.D / FLT.D / FLE.D
                   7'b1010001: if (ex_insn[14:12] <= 3'b010) begin
                      fcmp_result = fcmp_d(ex_insn[14:12], f1, f2);
                      write_back_register = ex_rd;
                      fp_int_result       <= {63'd0, fcmp_result[0]};
                      fp_int_fflags       <= fcmp_result[1] ? 5'b10000 : 5'd0;
                      state               <= `S_FP_INT_COMMIT;
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FCVT.W[U].S / FCVT.L[U].S
                   7'b1100000: if (ex_insn[24:20] <= 5'd3) begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         cvfpu_operands[0] <= f1;
                         cvfpu_operands[1] <= 64'd0;
                         cvfpu_operands[2] <= 64'd0;
                         cvfpu_rnd_mode <= pre_fp_rnd_mode;
                         cvfpu_op       <= 4'd11; // fpnew_pkg::F2I
                         cvfpu_op_mod   <= ex_insn[20]; // 0=signed, 1=unsigned
                         cvfpu_src_fmt  <= 3'd0; // fpnew_pkg::FP32
                         cvfpu_dst_fmt  <= 3'd0; // unused
                         cvfpu_int_fmt  <= ex_insn[21] ? 2'd3 : 2'd2; // INT64 : INT32
                         cvfpu_tag_in   <= {3'd0, ex_rd};
                         cvfpu_write_fp <= 1'b0;
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
`endif
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FCVT.W[U].D / FCVT.L[U].D
                   7'b1100001: if (ex_insn[24:20] <= 5'd3) begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         cvfpu_operands[0] <= f1;
                         cvfpu_operands[1] <= 64'd0;
                         cvfpu_operands[2] <= 64'd0;
                         cvfpu_rnd_mode <= pre_fp_rnd_mode;
                         cvfpu_op       <= 4'd11; // fpnew_pkg::F2I
                         cvfpu_op_mod   <= ex_insn[20]; // 0=signed, 1=unsigned
                         cvfpu_src_fmt  <= 3'd1; // fpnew_pkg::FP64
                         cvfpu_dst_fmt  <= 3'd0; // unused
                         cvfpu_int_fmt  <= ex_insn[21] ? 2'd3 : 2'd2; // INT64 : INT32
                         cvfpu_tag_in   <= {3'd0, ex_rd};
                         cvfpu_write_fp <= 1'b0;
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
`endif
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FCVT.S.W[U] / FCVT.S.L[U]
                   7'b1101000: if (ex_insn[24:20] <= 5'd3) begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         cvfpu_operands[0] <= s1;
                         cvfpu_operands[1] <= 64'd0;
                         cvfpu_operands[2] <= 64'd0;
                         cvfpu_rnd_mode <= pre_fp_rnd_mode;
                         cvfpu_op       <= 4'd12; // fpnew_pkg::I2F
                         cvfpu_op_mod   <= ex_insn[20]; // 0=signed, 1=unsigned
                         cvfpu_src_fmt  <= 3'd0; // unused
                         cvfpu_dst_fmt  <= 3'd0; // fpnew_pkg::FP32
                         cvfpu_int_fmt  <= ex_insn[21] ? 2'd3 : 2'd2; // INT64 : INT32
                         cvfpu_tag_in   <= {3'd0, ex_rd};
                         cvfpu_write_fp <= 1'b1;
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
`endif
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FCVT.D.W[U] / FCVT.D.L[U]
                   7'b1101001: if (ex_insn[24:20] <= 5'd3) begin
`ifdef USE_CVFPU
                      if (!pre_fp_rmode_ok) begin
                         cause = `TRAP_ILLEGAL_INSTRUCTION;
                         tval = ex_insn;
                         state <= `S_EXCEPTION;
                      end else begin
                         cvfpu_operands[0] <= s1;
                         cvfpu_operands[1] <= 64'd0;
                         cvfpu_operands[2] <= 64'd0;
                         cvfpu_rnd_mode <= pre_fp_rnd_mode;
                         cvfpu_op       <= 4'd12; // fpnew_pkg::I2F
                         cvfpu_op_mod   <= ex_insn[20]; // 0=signed, 1=unsigned
                         cvfpu_src_fmt  <= 3'd0; // unused
                         cvfpu_dst_fmt  <= 3'd1; // fpnew_pkg::FP64
                         cvfpu_int_fmt  <= ex_insn[21] ? 2'd3 : 2'd2; // INT64 : INT32
                         cvfpu_tag_in   <= {3'd0, ex_rd};
                         cvfpu_write_fp <= 1'b1;
                         cvfpu_in_valid <= 1'b1;
                         state          <= `S_CVFPU_ISSUE;
                      end
`else
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
`endif
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FMV.X.W (rs2=0, rm=0) or FCLASS.S (rs2=0, rm=1).
                   7'b1110000: if (ex_insn[24:20] == 5'd0 && ex_insn[14:12] == 3'b000) begin
                      write_back_register = ex_rd;
                      fp_int_result       <= {{32{f1[31]}}, f1[31:0]};
                      fp_int_fflags       <= 5'd0;
                      state               <= `S_FP_INT_COMMIT;
                   end else if (ex_insn[24:20] == 5'd0 && ex_insn[14:12] == 3'b001) begin
                      write_back_register = ex_rd;
                      fp_int_result       <= fclass_s(f1);
                      fp_int_fflags       <= 5'd0;
                      state               <= `S_FP_INT_COMMIT;
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FMV.X.D (rs2=0, rm=0) or FCLASS.D (rs2=0, rm=1).
                   7'b1110001: if (ex_insn[24:20] == 5'd0 && ex_insn[14:12] == 3'b000) begin
                      write_back_register = ex_rd;
                      fp_int_result       <= f1;
                      fp_int_fflags       <= 5'd0;
                      state               <= `S_FP_INT_COMMIT;
                   end else if (ex_insn[24:20] == 5'd0 && ex_insn[14:12] == 3'b001) begin
                      write_back_register = ex_rd;
                      fp_int_result       <= fclass_d(f1);
                      fp_int_fflags       <= 5'd0;
                      state               <= `S_FP_INT_COMMIT;
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FMV.W.X (rs2=0, rm=0): NaN-box s1[31:0] into f[rd].
                   7'b1111000: if (ex_insn[24:20] == 5'd0 && ex_insn[14:12] == 3'b000) begin
                      write_back_fp_valid    = 1;
                      write_back_fp_register = ex_rd;
                      write_back_fp_value    <= {32'hffffffff, s1[31:0]};
                      retire_linear_fetch();
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   // FMV.D.X (rs2=0, rm=0): full 64-bit move.
                   7'b1111001: if (ex_insn[24:20] == 5'd0 && ex_insn[14:12] == 3'b000) begin
                      write_back_fp_valid    = 1;
                      write_back_fp_register = ex_rd;
                      write_back_fp_value    <= s1;
                      retire_linear_fetch();
                   end else begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                   default: begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      tval = ex_insn;
                      state <= `S_EXCEPTION;
                   end
                 endcase
              end
           end

           // R4 fused multiply-add/subtract family: FMADD/FMSUB/FNMSUB/FNMADD.
           else if (ex_insn[6:4] == 3'b100 && ex_insn[1:0] == 2'b11) begin
              if (fs == 0) begin
                 cause = `TRAP_ILLEGAL_INSTRUCTION;
                 tval = ex_insn;
                 state <= `S_EXCEPTION;
              end else begin
                 fs = 3;
`ifdef USE_CVFPU
                 if (ex_insn[26:25] > 2'b01 || !pre_fp_rmode_ok) begin
                    cause = `TRAP_ILLEGAL_INSTRUCTION;
                    tval = ex_insn;
                    state <= `S_EXCEPTION;
                 end else begin
                    rs1 <= ex_insn[31:27]; // rs3; reuse FP read port 0
                    state <= `S_CVFPU_FMA_RF2;
                 end
`else
                 cause = `TRAP_ILLEGAL_INSTRUCTION;
                 tval = ex_insn;
                 state <= `S_EXCEPTION;
`endif
              end
           end

           else begin
`ifdef SIMULATE
`ifdef VERBOSE
              if (ex_insn[1:0] == 3)
                $display("%05d   %1d %x %x illegal unknown instruction", $time, prv, ex_pc, ex_insn);
              else
                $display("%05d   %1d %x     %x illegal unknown instruction (%1d,%1d)",
                         $time, prv, ex_pc, ex_insn[15:0], ex_insn[15:13], ex_insn[1:0]);
              $finish;
`endif
`endif
              cause = `TRAP_ILLEGAL_INSTRUCTION;
              tval = ex_insn;
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
        end

        `S_EXECUTE2: begin : execute2_stage
           if (execute_res_valid) begin
              // write_back_value compute moved to case(ex_state) EX_EXECUTE2.
              execute_res_valid <= 0;
              retire_linear_fetch();
           end else begin
              prepare_current_epoch_fetch(npc, prv);
              state <= `S_FETCH1;
           end
        end

        `S_FP_INT_COMMIT: begin
           write_back_value <= fp_int_result;
           fflags = fflags | fp_int_fflags;
           retire_linear_fetch();
        end

`ifdef USE_CVFPU
        `S_CVFPU_FMA_RF2: begin
           state <= `S_CVFPU_FMA_RF3;
        end

        `S_CVFPU_FMA_RF3: begin
           cvfpu_operands[0] <= f1;
           cvfpu_operands[1] <= f2;
           cvfpu_operands[2] <= (write_back_fp_valid &&
                                 ex_insn[31:27] == write_back_fp_register) ?
                                fp_writeback_data : f1_bram;
           cvfpu_rnd_mode <= pre_fp_rnd_mode;
           cvfpu_op       <= ex_insn[3] ? 4'd1 : 4'd0; // FNMSUB : FMADD
           cvfpu_op_mod   <= ex_insn[2]; // add/sub variant
           cvfpu_src_fmt  <= {2'd0, ex_insn[25]}; // FP32/FP64
           cvfpu_dst_fmt  <= {2'd0, ex_insn[25]}; // FP32/FP64
           cvfpu_int_fmt  <= 2'd3; // fpnew_pkg::INT64 (unused)
           cvfpu_tag_in   <= {3'd0, ex_rd};
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
                 retire_linear_fetch();
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
              retire_linear_fetch();
           end else begin
              try_issue_queued_decode_preserve_state(1'b1,
                                                     !cvfpu_write_fp, cvfpu_tag_in[4:0],
                                                     cvfpu_write_fp, cvfpu_tag_in[4:0]);
           end
        end
`endif

        `S_CBO_EXEC: begin
           translated <= 0;
           if (phys_region(mem_addr) == `REGION_BRAM ||
               phys_region(mem_addr) == `REGION_DRAM) begin
              if (cache_idle) begin
                 cache_cbo_line_addr <= mem_addr[30:6];
                 cache_cbo_flush <= 1;
                 state <= `S_CBO_WAIT;
              end
           end else begin
              retire_linear_fetch();
           end
        end

        `S_CBO_WAIT: begin
           if (cache_cbo_done) begin
              retire_linear_fetch();
           end else begin
              try_issue_queued_decode_preserve_state(1'b1,
                                                     1'b0, 5'd0,
                                                     1'b0, 5'd0);
           end
        end

        `S_STORE: begin

           if (csr_satp[63:60] == 4'd8 && (mprv ? mpp : prv) != 3 && !translated) begin
              // Sv39 store address translation
              start_translation(mem_addr, 2'd2, mprv ? mpp : prv, `S_STORE);
           end else if (phys_region(mem_addr) == `REGION_UART) begin
              translated <= 0;
              retire_linear_fetch();
              reservation <= ~0;

              // Keep UART writes on the original store cycle; only BRAM writes
              // need the delayed commit state for SRAM WE timing.
              case (mem_addr[2:0])
                0: if (!uart_lcr[7]) begin // THR (when DLAB=0)
`ifdef PC_TRACE
                      if (!dbg_armed) begin
                         if (uart_tx_accept) begin
                            uart_tx_fifo[uart_tx_tail[UART_FIFO_INDEX_BITS-1:0]] <= store_value[7:0];
                            uart_tx_tail <= uart_tx_tail + 1;
                            uart_thre_pending <= 0;
                         end
                      end
`else
                      if (uart_tx_accept) begin
                         uart_tx_fifo[uart_tx_tail[UART_FIFO_INDEX_BITS-1:0]] <= store_value[7:0];
                         uart_tx_tail <= uart_tx_tail + 1;
                         uart_thre_pending <= 0;
                      end
`endif
                   end
                1: if (!uart_lcr[7]) begin // Only bits [3:0] valid
                   uart_ier <= store_value[3:0];
                   if (!store_value[1])
                      uart_thre_pending <= 0;
                   else if (!uart_ier[1] && uart_tx_accept)
                      uart_thre_pending <= 1;
                end
                2: begin // FCR (write-only)
                   uart_fcr_fifo <= store_value[0];
                   if (store_value[1]) begin
                      uart_rx_head <= 0;
                      uart_rx_tail <= 0;
                      uart_rx_front_valid <= 0;
                      uart_rx_refill_pending <= 0;
                   end
                   if (store_value[2]) begin
                      uart_tx_head <= 0;
                      uart_tx_tail <= 0;
                      if (uart_ier[1])
                         uart_thre_pending <= 1;
                   end
                end
                3: uart_lcr <= store_value[7:0];
                4: uart_mcr <= store_value[4:0];
                7: uart_scr <= store_value[7:0];
                default: begin end
              endcase
              mem_wr_mask = 0;
           end else begin
              state <= `S_STORE_COMMIT;
           end
        end

        `S_STORE_COMMIT: begin
           translated <= 0;
           retire_linear_fetch();
           reservation <= ~0;

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
                         if (uart_tx_accept) begin
                            uart_tx_fifo[uart_tx_tail[UART_FIFO_INDEX_BITS-1:0]] <= store_value[7:0];
                            uart_tx_tail <= uart_tx_tail + 1;
                            uart_thre_pending <= 0;
                         end
                      end
`else
                      if (uart_tx_accept) begin
                         uart_tx_fifo[uart_tx_tail[UART_FIFO_INDEX_BITS-1:0]] <= store_value[7:0];
                         uart_tx_tail <= uart_tx_tail + 1;
                         uart_thre_pending <= 0;
                      end
`endif
                   end
                1: if (!uart_lcr[7]) begin // Only bits [3:0] valid
                   uart_ier <= store_value[3:0];
                   if (!store_value[1])
                      uart_thre_pending <= 0;
                   else if (!uart_ier[1] && uart_tx_accept)
                      uart_thre_pending <= 1;
                end
                2: begin // FCR (write-only)
                   uart_fcr_fifo <= store_value[0];
                   if (store_value[1]) begin
                      uart_rx_head <= 0;
                      uart_rx_tail <= 0;
                      uart_rx_front_valid <= 0;
                      uart_rx_refill_pending <= 0;
                   end
                   if (store_value[2]) begin
                      uart_tx_head <= 0;
                      uart_tx_tail <= 0;
                      if (uart_ier[1])
                         uart_thre_pending <= 1;
                   end
                end
                3: uart_lcr <= store_value[7:0];
                4: uart_mcr <= store_value[4:0];
                7: uart_scr <= store_value[7:0];
                default: begin end
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
                default: begin end
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
                 // Completion: rearm the source gateway. If the source is
                 // still asserted, it will become pending again next cycle.
                 if (store_value[5:0] != 0)
                    plic_in_service[store_value[5:0]] <= 0;
              end
              mem_wr_mask = 0;
             end
             `REGION_BRAM,
             `REGION_DRAM: begin
              // Cacheable store.  The cache refill/writeback engine chooses
              // BRAM or AXI backing by line address; MMIO never reaches here.
             begin : cacheable_store_calc
                 reg [127:0] wide_data;
                 reg  [15:0] wide_mask;
                 wide_data = {64'd0, store_value} << (mem_addr[2:0] * 8);
                 wide_mask = {8'd0, mem_wr_mask[7:0]} << mem_addr[2:0];
                 dram_addr       <= mem_addr[30:3];
                 dram_va         <= mem_va;
                 dram_asid       <= mem_asid;
                 dram_perm       <= mem_perm;
                 dram_ctx        <= mem_ctx;
                 dram_writedata  <= wide_data[63:0];
                 dram_wstrb      <= wide_mask[7:0];
                 mem_wr_mask     = 0;
                 if (|wide_mask[15:8]) begin
                    // Overflow into next 8-byte chunk: save for S_DRAM_STORE2
                    dram2_addr        <= mem_addr[30:3] + 1;
                    dram2_va          <= {mem_va[63:3], 3'b000} + 64'd8;
                    dram2_asid        <= mem_asid;
                    dram2_perm        <= mem_perm;
                    dram2_ctx         <= mem_ctx;
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
           // BRAM stores are cacheable stores now; stale entries retire.
           state <= `S_FETCH1;
        end

        `S_LOAD_ALIGN: begin
           if (csr_satp[63:60] == 4'd8 && (mprv ? mpp : prv) != 3 && !translated) begin
              // Sv39 load/AMO address translation
              start_translation(mem_addr, do_atomic ? 2'd3 : 2'd1, mprv ? mpp : prv, `S_LOAD_ALIGN);
           end else begin
              if (!do_atomic) translated <= 0;

              state <= `S_FETCH1;

              if (do_atomic)
                state <= `S_AMO;

              case (phys_region(mem_addr))
                `REGION_UART,
                `REGION_CLINT,
                `REGION_PLIC: begin
                 state <= `S_LOCAL_LOAD;
                end
                `REGION_MMIO: begin
`ifdef TRACE_MMIO
                 $display("%05d  MMIO READ FROM %x/%x", $time, mem_addr, load_size_lg2);
`endif
                 state <= `S_MMIO_READ;
                 mmio_address = mem_addr;
                 mmio_read = 1;
                 mmio_timeout_tval <= mem_va;
                end
                `REGION_BRAM,
                `REGION_DRAM: begin
                 // Cacheable load.  The cache refill engine chooses BRAM or
                 // AXI by line address; MMIO stays on the explicit slow path.
                 dram_addr <= mem_addr[30:3];
                 dram_va   <= mem_va;
                 dram_asid <= mem_asid;
                 dram_perm <= mem_perm;
                 dram_ctx  <= mem_ctx;
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

        `S_LOCAL_LOAD: begin
           if (!do_atomic)
              retire_linear_fetch();
           if (do_atomic)
              state <= `S_AMO;

           case (phys_region(mem_addr))
             `REGION_UART: begin
              // NS16550A UART read (0x10000000-0x1000000F)
              case (mem_addr[2:0])
                0: if (!uart_lcr[7]) begin // RBR (when DLAB=0)
                      write_back_value = uart_rx_front_valid ? uart_rx_front : 0;
                      if (uart_rx_pop) begin
                         uart_rx_head <= uart_rx_head + 1;
                         if (uart_rx_count > 1) begin
                            uart_rx_front_valid <= 0;
                            uart_rx_refill_pending <= 1;
                            uart_rx_refill_addr <= uart_rx_head[UART_FIFO_INDEX_BITS-1:0] + 1'b1;
                         end else if (!uart_rx_push) begin
                            uart_rx_front_valid <= 0;
                         end
                      end
                   end else
                      write_back_value = 0; // DLL (divisor, ignored)
                1: write_back_value = uart_lcr[7] ? 0 : {4'd0, uart_ier[3:0]};
                2: begin
                   write_back_value = uart_iir;
                   if (uart_iir_thre)
                      uart_thre_pending <= 0;
                end
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
              else if (mem_addr[23:0] >= 24'h201004 && mem_addr[23:0] <= 24'h201007) begin
                 write_back_value = plic_best_irq;
                 if (plic_best_irq != 0) begin
                    plic_pending[plic_best_irq] <= 0;
                    plic_in_service[plic_best_irq] <= 1;
                 end
              end
              else
                 write_back_value = 0;
              if (load_size_lg2 == 2)
                 write_back_value = write_back_value[31:0];
              else if (load_size_lg2 == 6)
                 write_back_value = {{32{write_back_value[31]}}, write_back_value[31:0]};
             end
             default: begin
              write_back_register = 0;
              cause = `TRAP_LOAD_ACCESS_FAULT;
              tval = mem_addr;
              state <= `S_EXCEPTION;
             end
           endcase
           finish_load_writeback();
        end

        `S_MMIO_READ: state <= `S_MMIO_ALIGN;

        `S_MMIO_ALIGN: begin
           if (mmio_readdatavalid) begin

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

              finish_load_writeback();
              retire_linear_fetch();

              if (do_atomic) begin
`ifdef SIMULATE
                 $display("Sorry, atomics to MMIO aren't supported yet");
                 $finish;
`endif
                 state <= `S_AMO;
              end
           end else begin
              try_issue_queued_decode_preserve_state(!do_atomic,
                                                     !write_back_fp_valid, write_back_register,
                                                     write_back_fp_valid, write_back_fp_register);
           end
        end

        `S_AMO: begin
           // Note: translated stays set from S_LOAD_ALIGN PTW (ptw_access=3
           // already checked both read and write permission), so S_STORE
           // will skip re-translation and use the physical mem_addr directly.
           mem_wr_mask = 255;
           store_value = s2;
           if (!ex_insn[12]) begin
              write_back_value = {{32{write_back_value[31]}},write_back_value[31:0]};
              store_value = {{32{s2[31]}},s2[31:0]};
              mem_wr_mask = 15;
           end

           // ex_insn[31:27]=funct5, ex_insn[26]=aq, ex_insn[25]=rl, ex_insn[24]=rs2[4]
           // Mask aq/rl/rs2[4] so any register and any ordering variant is handled.
           case ({ex_insn[31:27], 3'b0})
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


        `S_HANDLE_CSR: begin : handle_csr_state
           reg csr_write_failure;
           state <= `S_HANDLE_CSR_COMMIT;
           csr_access_failure = 0;
           csr_write_failure = 0;
           csr_read_val = 0;
           tval = ex_insn;
           write_back_register = ex_rd;

           if (ex_rd != 0 || csr_op != `CSR_OP_COPY) begin
              // read the CSR

              // PMP: pmpcfg0-15 and pmpaddr0-63 — M-mode only, reads zero
              // (0 PMP entries implemented; all accesses permitted).
              if ('h3A0 <= csrno && csrno <= 'h3FF) csr_read_val = 0;
              else if (`CSR_MHPMEVENT3 <= csrno && csrno <= `CSR_MHPMEVENT3 + (`HPM_COUNTERS - 1)) begin
                 if (csrno[3:0] >= 4'd3)
                    csr_read_val = csr_mhpmevent[csrno[3:0] - 4'd3];
                 else
                    csr_read_val = 0;
              end else if (`CSR_MHPMCOUNTER3 <= csrno && csrno <= `CSR_MHPMCOUNTER3 + (`HPM_COUNTERS - 1)) begin
                 if (csrno[3:0] >= 4'd3)
                    csr_read_val = csr_mhpmcounter[csrno[3:0] - 4'd3];
                 else
                    csr_read_val = 0;
              end else if (`CSR_HPMCOUNTER3 <= csrno && csrno <= `CSR_HPMCOUNTER3 + (`HPM_COUNTERS - 1)) begin
                 if (csrno[3:0] >= 4'd3)
                    csr_read_val = csr_mhpmcounter[csrno[3:0] - 4'd3];
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
                `CSR_SENVCFG:   csr_read_val = {56'd0, csr_senvcfg};
                `CSR_SSCRATCH:  csr_read_val = csr_sscratch;
                `CSR_SEPC:      csr_read_val = csr_sepc;
                `CSR_SCAUSE:    csr_read_val = csr_scause;
                `CSR_STVAL:     csr_read_val = csr_stval;
                `CSR_SIP:       csr_read_val = csr_mip & csr_mideleg;
                `CSR_SCOUNTOVF: csr_read_val = csr_scountovf_read_val;

                `CSR_SATP: begin
                   if (prv == 1 && tvm) begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
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
                `CSR_MIMPID:   csr_read_val = 'h20260518;
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
                `CSR_VHPR_READS: csr_read_val = csr_vhpr_reads;
                `CSR_VHPR_WRITES: csr_read_val = csr_vhpr_writes;
                `CSR_VHPR_READ_HITS: csr_read_val = csr_vhpr_read_hits;
                `CSR_VHPR_READ_MISSES: csr_read_val = csr_vhpr_read_misses;
                `CSR_VHPR_WRITE_HITS: csr_read_val = csr_vhpr_write_hits;
                `CSR_VHPR_WRITE_MISSES: csr_read_val = csr_vhpr_write_misses;
                `CSR_VHPR_FILLS: csr_read_val = csr_vhpr_fills;
                `CSR_VHPR_VICTIM_EVICTS: csr_read_val = csr_vhpr_victim_evicts;
                `CSR_VHPR_DIRTY_VICTIM_EVICTS: csr_read_val = csr_vhpr_dirty_victim_evicts;
                `CSR_VHPR_ALIAS_EVICTS: csr_read_val = csr_vhpr_alias_evicts;
                `CSR_VHPR_DIRTY_ALIAS_EVICTS: csr_read_val = csr_vhpr_dirty_alias_evicts;
                `CSR_VHPR_FLUSH_EVICTS: csr_read_val = csr_vhpr_flush_evicts;
                `CSR_VHPR_DIRTY_FLUSH_EVICTS: csr_read_val = csr_vhpr_dirty_flush_evicts;
                `CSR_VHPR_CBO_PROBES: csr_read_val = csr_vhpr_cbo_probes;
                `CSR_VHPR_PTW_PROBES: csr_read_val = csr_vhpr_ptw_probes;
                `CSR_VHPR_EPOCH_BUMPS: csr_read_val = csr_vhpr_epoch_bumps;
                `CSR_VHPR_EPOCH_ROLLOVERS: csr_read_val = csr_vhpr_epoch_rollovers;
                `CSR_VHPR_EPOCH: csr_read_val = {{64-VHPR_EPOCH_BITS{1'b0}}, vhpr_epoch};
                `CSR_BUILD_STAMP: csr_read_val = `SMOLRV64_BUILD_STAMP;
                default: begin
`ifdef SIMULATE
`ifdef VERBOSE
                   $display("%05d   %1d %x %x illegal CSR %x (read)", $time, prv, pc, insn, csrno);
`endif
`endif
                   cause = `TRAP_ILLEGAL_INSTRUCTION;
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
                 if (csrno == `CSR_CYCLE && !counter_access_allowed(6'd0))
                    csr_access_failure = 1;
                 else if (csrno == `CSR_TIME && !counter_access_allowed(6'd1))
                    csr_access_failure = 1;
                 else if (csrno == `CSR_INSTRET && !counter_access_allowed(6'd2))
                    csr_access_failure = 1;
                 else if (`CSR_HPMCOUNTER3 <= csrno && csrno <= `CSR_HPMCOUNTER3 + (`HPM_COUNTERS - 1) &&
                          !counter_access_allowed({1'b0, csrno[4:0]}))
                    csr_access_failure = 1;
              end
           end

           // Write priviledge check
           if (ex_rs1 != 0 || csr_op == `CSR_OP_COPY) begin
              if (prv < csrno[9:8]) begin
                 csr_write_failure = 1;
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
                 csr_write_failure = 1;
              end
           end



           // CSRRS, CSRRC, CSRRSI, and CSRRCI don't write the CSR if rs1 == 0
           if (!csr_write_failure && (ex_rs1 != 0 || csr_op == `CSR_OP_COPY)) begin
              // write the CSR

              // PMP: pmpcfg0-15 and pmpaddr0-63 — M-mode only, writes silently ignored
              // (0 PMP entries implemented; all accesses permitted).
              if ('h3A0 <= csrno && csrno <= 'h3FF) begin end
              else if (`CSR_MHPMEVENT3 <= csrno && csrno <= `CSR_MHPMEVENT3 + (`HPM_COUNTERS - 1)) begin
                 if (csrno[3:0] >= 4'd3) begin
                    hpm_event_wr_en <= 1;
                    hpm_wr_idx <= csrno[3:0] - 4'd3;
                    hpm_wr_data <= csr_modify_value(csr_mhpmevent[csrno[3:0] - 4'd3],
                                                     csr_arg, csr_op);
                 end
              end else if (`CSR_MHPMCOUNTER3 <= csrno && csrno <= `CSR_MHPMCOUNTER3 + (`HPM_COUNTERS - 1)) begin
                 if (csrno[3:0] >= 4'd3) begin
                    hpm_counter_wr_en <= 1;
                    hpm_wr_idx <= csrno[3:0] - 4'd3;
                    hpm_wr_data <= csr_modify_value(csr_mhpmcounter[csrno[3:0] - 4'd3],
                                                     csr_arg, csr_op);
                 end
              end
              else case (csrno)
                // fcsr: fflags aliased at [4:0], frm aliased at [7:5].
                // Writing any of these is an implicit "FP state touched"
                // event, so we mark FS=Dirty at the same time.
                `CSR_FFLAGS: begin : csr_write_fflags
                   reg [63:0] csr_next;
                   csr_next = csr_modify_value({59'd0, fflags}, csr_arg, csr_op);
                   fflags = csr_next[4:0];
                   fs = 3;
                end
                `CSR_FRM: begin : csr_write_frm
                   reg [63:0] csr_next;
                   csr_next = csr_modify_value({61'd0, frm}, csr_arg, csr_op);
                   frm = csr_next[2:0];
                   fs = 3;
                end
                `CSR_FCSR: begin : csr_write_fcsr
                   reg [63:0] csr_next;
                   csr_next = csr_modify_value({56'd0, frm, fflags}, csr_arg, csr_op);
                   {frm, fflags} = csr_next[7:0];
                   fs = 3;
                end
                `CSR_SSTATUS: begin : csr_write_sstatus
                   reg [63:0] csr_next;
                   csr_next = csr_modify_value({sd, 29'd0, uxl, 12'd0, mxr, sum, 1'd0,
                                                xs, fs, 4'd0, spp, 2'd0, spie, upie, 2'd0, sie, uie},
                                               csr_arg, csr_op);
`ifdef SIMULATE
`ifdef VERBOSE
                   if (sum != csr_next[18])
                      $display("SSTATUS: sum %d->%d pc %x time %0t", sum, csr_next[18], pc, $time);
`endif
`endif
                   {mxr, sum}        = csr_next[19:18];
                   fs                = csr_next[14:13];
                   spp               = csr_next[8];
                   {spie, upie}      = csr_next[5:4];
                   {sie, uie}        = csr_next[1:0];
                end
                `CSR_SIE:       csr_mie    = csr_modify_value(csr_mie & 14'h2222, csr_arg, csr_op) & 14'h2222 | csr_mie & ~14'h2222;
                `CSR_STVEC:     csr_stvec  = csr_modify_value(csr_stvec, csr_arg, csr_op);
                `CSR_SCOUNTEREN:csr_scounteren = csr_modify_value(csr_scounteren, csr_arg, csr_op) & `HPM_COUNTER_MASK;
                `CSR_SENVCFG: begin : csr_write_senvcfg
                   reg [63:0] csr_next;
                   csr_next = csr_modify_value({56'd0, csr_senvcfg}, csr_arg, csr_op);
                   csr_senvcfg = csr_next[7:0] & 8'hf1;
                end
                `CSR_SSCRATCH:  csr_sscratch = csr_modify_value(csr_sscratch, csr_arg, csr_op);
                `CSR_SEPC:      csr_sepc   = csr_modify_value(csr_sepc, csr_arg, csr_op) & ~1;
                `CSR_SCAUSE:    csr_scause = csr_modify_value(csr_scause, csr_arg, csr_op);
                `CSR_STVAL:     csr_stval  = csr_modify_value(csr_stval, csr_arg, csr_op);
                `CSR_SIP:       begin : csr_write_sip
                   reg [63:0] csr_next;
                   csr_next = csr_modify_value(csr_mip & csr_mideleg, csr_arg, csr_op);
                   // Only SSIP (bit 1) is writable via SIP; SEIP/STIP are read-only
                   if (csr_mideleg[13]) lcofip = csr_next[13];
                   if (csr_mideleg[1]) ssip = csr_next[1];
                   if (csr_mideleg[0]) usip = csr_next[0];
                end
                `CSR_SATP: begin
                   if (prv == 1 && tvm) begin
                      cause = `TRAP_ILLEGAL_INSTRUCTION;
                      state <= `S_EXCEPTION;
                   end else begin
                     case (csr_op)
                       `CSR_OP_COPY: csr_satp_write_val = csr_arg;
                       `CSR_OP_OR:   csr_satp_write_val = csr_satp | csr_arg;
                       default:      csr_satp_write_val = csr_satp & ~csr_arg;
                     endcase

                     csr_satp_write_val = satp_warl_value(csr_satp_write_val);
                     if (csr_satp_write_val[63:60] == 4'd0 ||
                         csr_satp_write_val[63:60] == 4'd8) begin
                       // WARL: only Bare and Sv39 are supported; unsupported
                       // MODE values cause the entire write to have no effect.
                       csr_satp = csr_satp_write_val;
                     end
                   end
                end
                `CSR_MSTATUS: begin : csr_write_mstatus
                   reg [63:0] csr_next;
                   csr_next = csr_modify_value({sd, 27'd0, sxl, uxl, 9'd0,
                                                tsr, tw, tvm, mxr, sum, mprv,
                                                xs, fs, mpp, 2'd0, spp,
                                                mpie, 1'd0, spie, upie, mie, 1'd0, sie, uie},
                                               csr_arg, csr_op);
                   {sie, uie}        = csr_next[1:0];
                   {spie, upie, mie} = csr_next[5:3];
                   {spp, mpie}       = csr_next[8:7];
                   mpp               = csr_next[12:11];
                   fs                = csr_next[14:13];
                   {tsr, tw, tvm, mxr, sum, mprv} = csr_next[22:17];
                end
                `CSR_MISA:     begin end
                `CSR_MEDELEG:  csr_medeleg  = csr_modify_value(csr_medeleg, csr_arg, csr_op);
                `CSR_MIDELEG:  csr_mideleg  = csr_modify_value(csr_mideleg, csr_arg, csr_op);
                `CSR_MIE:      csr_mie      = csr_modify_value(csr_mie, csr_arg, csr_op);
                `CSR_MTVEC:    csr_mtvec    = csr_modify_value(csr_mtvec, csr_arg, csr_op); // XXX enforce 256-byte alignment for vectored interrupts
                `CSR_MCOUNTEREN: csr_mcounteren = csr_modify_value(csr_mcounteren, csr_arg, csr_op) & `HPM_COUNTER_MASK;
                `CSR_MCOUNTINHIBIT: csr_mcountinhibit = csr_modify_value(csr_mcountinhibit, csr_arg, csr_op) & `HPM_INHIBIT_MASK;
                `CSR_MCYCLECFG: csr_mcyclecfg = csr_modify_value(csr_mcyclecfg, csr_arg, csr_op);
                `CSR_MINSTRETCFG: csr_minstretcfg = csr_modify_value(csr_minstretcfg, csr_arg, csr_op);
                `CSR_MSCRATCH: csr_mscratch = csr_modify_value(csr_mscratch, csr_arg, csr_op);
                `CSR_MEPC:     csr_mepc     = csr_modify_value(csr_mepc, csr_arg, csr_op) & ~1;
                `CSR_MCAUSE:   csr_mcause   = csr_modify_value(csr_mcause, csr_arg, csr_op);
                `CSR_MTVAL:    csr_mtval    = csr_modify_value(csr_mtval, csr_arg, csr_op);
                `CSR_MIP:      begin : csr_write_mip
                   reg [63:0] csr_next;
                   csr_next = csr_modify_value(csr_mip, csr_arg, csr_op);
                   // MEIP/SEIP (bits 11,9) are read-only, driven by PLIC
                   // MTIP/MSIP (bits 7,3) are read-only, driven by CLINT
                   lcofip = csr_next[13];
                   ueip = csr_next[8];
                   stip = csr_next[5];
                   utip = csr_next[4];
                   ssip = csr_next[1];
                   usip = csr_next[0];
                end
                `CSR_TSELECT:  begin end
                `CSR_TDATA1:   begin end
                `CSR_TDATA2:   begin end
                `CSR_TDATA3:   begin end
                `CSR_MCYCLE:   csr_mcycle   <= csr_modify_value(csr_mcycle, csr_arg, csr_op);
                `CSR_MINSTRET: csr_minstret <= csr_modify_value(csr_minstret, csr_arg, csr_op);
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
                `CSR_VHPR_READS,
                `CSR_VHPR_WRITES,
                `CSR_VHPR_READ_HITS,
                `CSR_VHPR_READ_MISSES,
                `CSR_VHPR_WRITE_HITS,
                `CSR_VHPR_WRITE_MISSES,
                `CSR_VHPR_FILLS,
                `CSR_VHPR_VICTIM_EVICTS,
                `CSR_VHPR_DIRTY_VICTIM_EVICTS,
                `CSR_VHPR_ALIAS_EVICTS,
                `CSR_VHPR_DIRTY_ALIAS_EVICTS,
                `CSR_VHPR_FLUSH_EVICTS,
                `CSR_VHPR_DIRTY_FLUSH_EVICTS,
                `CSR_VHPR_CBO_PROBES,
                `CSR_VHPR_PTW_PROBES,
                `CSR_VHPR_EPOCH_BUMPS,
                `CSR_VHPR_EPOCH_ROLLOVERS: begin
                   vhpr_stats_clear_req <= ~vhpr_stats_clear_ack;
                end
                default: begin
                 csr_write_failure = 1;
`ifdef SIMULATE
`ifdef VERBOSE
                   $display("%05d   %1d %x %x illegal CSR %x (write)", $time, prv, pc, insn, csrno);
`endif
`endif
                end
              endcase
           end

           csr_read_result <= csr_read_val;
           if (csr_access_failure || csr_write_failure) begin
              cause = `TRAP_ILLEGAL_INSTRUCTION;
              state <= `S_EXCEPTION;
           end

           // Any CSR write may change interrupt-enable/pending state
           // (sstatus/mstatus/sie/mie/mip/mideleg). pre_intr_pending
           // is a FF sampled the cycle BEFORE S_FETCH1 from current CSR
           // FF values, so it is stale for one cycle after any CSR write.
           // Reuse just_xret as a generic one-cycle suppress flag.
           just_xret <= 1;
        end

        `S_HANDLE_CSR_COMMIT: begin
           write_back_value <= csr_read_result;
           execute_res_valid <= 0;
           retire_prepared_fetch();
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
              csr_scause = {cause_intr, 63'd0} | {50'd0, cause};
              csr_sepc = pc;
              csr_stval = tval;
              spie = sie;
              sie = 0;
              spp = prv;
              tvec = csr_stvec;
              prv = 1;
           end else begin
              csr_mcause = {cause_intr, 63'd0} | {50'd0, cause};
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
               ({cause_intr, 63'd0} | {50'd0, cause}),   // architectural mcause/scause form
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
           frontend_buf_flush <= 1'b1;
           fetch_epoch <= fetch_epoch + 1'b1;
           clear_frontend_cmd();
           frontend_redirect_valid <= 0;
           frontend_miss_valid <= 0;
           frontend_miss_done <= 0;
           squash_decode_execute();

           state <= `S_FETCH1;
        end

        `S_MULDIV_START: begin
           muldiv_output_sext32 = 0;
           muldiv_output_negate = 0;
           muldiv_output_high_part = 0;

           case (muldiv_start_op)
             `MULDIV_MUL: begin
                mul_a = s1;
                mul_b = s2;
                state <= `S_MUL_RUNNING;
             end

             `MULDIV_MULH: begin
                muldiv_output_negate = s1[63] != s2[63];
                mul_a = {64'd0, pre_mul_abs_s1};
                mul_b = pre_mul_abs_s2;
                muldiv_output_high_part = 1;
                state <= `S_MUL_RUNNING;
             end

             `MULDIV_MULHSU: begin
                muldiv_output_negate = s1[63];
                mul_a = {64'd0, pre_mul_abs_s1};
                mul_b = s2;
                muldiv_output_high_part = 1;
                state <= `S_MUL_RUNNING;
             end

             `MULDIV_MULHU: begin
                mul_a = {64'd0, s1};
                mul_b = s2;
                muldiv_output_high_part = 1;
                state <= `S_MUL_RUNNING;
             end

             `MULDIV_DIV: begin
                muldiv_output_negate = s1[63] != s2[63];
                if (s2 == 0)
                  // No matter s1, this will produce -1 which is the correct answer
                  muldiv_output_negate = 0;
                div_count = 64;
                muldiv_p = {64'd0, pre_mul_abs_s1};
                mul_a = {pre_mul_abs_s2, 63'd0};
                mul_b = 0;
                state <= `S_DIV_RUNNING;
             end

             `MULDIV_DIVU: begin
                div_count = 64;
                muldiv_p = {64'd0, s1};
                mul_a = {s2, 63'd0};
                mul_b = 0;
                state <= `S_DIV_RUNNING;
             end

             `MULDIV_REM: begin
                // "For REM, the sign of a nonzero result equals the sign of the dividend."
                muldiv_output_negate = s1[63];
                if (s2 == 0)
                  // Preserve existing divide-by-zero behavior.
                  muldiv_output_negate = 0;
                div_count = 64;
                muldiv_p = {64'd0, pre_mul_abs_s1};
                mul_a = {pre_mul_abs_s2, 63'd0};
                mul_b = 0;
                muldiv_output_high_part = 1; // XXX abusing variables
                state <= `S_DIV_RUNNING;
             end

             `MULDIV_REMU: begin
                if (s2 == 0)
                  // No matter s1, this will produce -1 which is the correct answer
                  muldiv_output_negate = 0;
                div_count = 64;
                muldiv_p = {64'd0, s1};
                mul_a = {s2, 63'd0};
                mul_b = 0;
                muldiv_output_high_part = 1; // XXX abusing variables
                state <= `S_DIV_RUNNING;
             end

             `MULDIV_MULW: begin
                mul_a = s1[31:0];
                mul_b = s2[31:0];
                muldiv_output_sext32 = 1;
                state <= `S_MUL_RUNNING;
             end

             `MULDIV_DIVW: begin
                muldiv_output_negate = s1[31] != s2[31];
                if (s2 == 0)
                  // No matter s1, this will produce -1 which is the correct answer
                  muldiv_output_negate = 0;
                div_count = 32;
                muldiv_p = {96'd0, pre_mul_abs_s1w};
                mul_a = {pre_mul_abs_s2w, 31'd0};
                mul_b = 0;
                muldiv_output_sext32 = 1;
                state <= `S_DIV_RUNNING;
             end

             `MULDIV_DIVUW: begin
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

             `MULDIV_REMW: begin
                // "For REM, the sign of a nonzero result equals the sign of the dividend."
                muldiv_output_negate = s1[31];
                div_count = 32;
                muldiv_p = {96'd0, pre_mul_abs_s1w};
                mul_a = {pre_mul_abs_s2w, 31'd0};
                mul_b = 0;
                muldiv_output_sext32 = 1;
                muldiv_output_high_part = 1; // XXX abusing variables
                state <= `S_DIV_RUNNING;
             end

             `MULDIV_REMUW: begin
                div_count = 32;
                muldiv_p = {96'd0, s1[31:0]};
                mul_a = {s2[31:0], 31'd0};
                mul_b = 0;
                muldiv_output_sext32 = 1;
                muldiv_output_high_part = 1; // XXX abusing variables
                state <= `S_DIV_RUNNING;
             end

             default: begin
                cause = `TRAP_ILLEGAL_INSTRUCTION;
                tval = ex_insn;
                state <= `S_EXCEPTION;
             end
           endcase
        end

        `S_MUL_RUNNING: begin
           if (mul_b != 0) begin
              if (mul_b[0])
                muldiv_p = muldiv_p + mul_a;
              mul_a = mul_a << 1;
              mul_b = mul_b >> 1;
              try_issue_queued_decode_preserve_state(1'b1,
                                                     1'b1, write_back_register,
                                                     1'b0, 5'd0);
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

              retire_linear_fetch();
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
              try_issue_queued_decode_preserve_state(1'b1,
                                                     1'b1, write_back_register,
                                                     1'b0, 5'd0);
           end else begin
              write_back_value = muldiv_output_negate ? -mul_b : mul_b;
              if (muldiv_output_high_part)
                // REM
                write_back_value = muldiv_output_negate ? -muldiv_p[63:0] : muldiv_p[63:0];

              if (muldiv_output_sext32)
                write_back_value = {{32{write_back_value[31]}}, write_back_value[31:0]};

              retire_linear_fetch();
           end
        end

        `S_TLB_LOOKUP: begin
           state <= `S_TLB_CHECK;
        end

        `S_TLB_START_FETCH: begin
           start_translation(frontend_cmd_pc, 2'd0, frontend_cmd_prv, `S_FETCH2);
        end

        `S_TLB_START_FETCH_HALF: begin
           start_translation(pc + 2, 2'd0, prv, `S_FETCH2_HALF);
        end

        `S_TLB_CHECK: begin
           tlb_latched_4k_hit <= tlb_4k_hit;
           tlb_latched_2m_hit <= tlb_2m_hit;
           tlb_4k_hit_pa <= {tlb_4k_rd_pbase, tlb_req_va[11:0]};
           tlb_2m_hit_pa <= {tlb_2m_rd_pbase, tlb_req_va[20:0]};
           state <= `S_TLB_DECIDE;
        end

        `S_TLB_DECIDE: begin
           if (tlb_latched_4k_hit || tlb_latched_2m_hit) begin
              tlb_hit_pa <= tlb_latched_4k_hit ? tlb_4k_hit_pa : tlb_2m_hit_pa;
              state <= `S_TLB_HIT;
           end else begin
              state <= `S_PTW_START;
           end
        end

        `S_TLB_HIT: begin
           route_translated_addr(tlb_hit_pa,
                                 tlb_latched_4k_hit ? tlb_4k_rd_perm : tlb_2m_rd_perm,
                                 tlb_req_return);
        end

        `S_PTW_START: begin
           start_ptw(tlb_req_va, tlb_req_access, tlb_req_prv, tlb_req_return);
        end

        `S_PTW_READ: begin
           // Sv39 page table walk: latch PTE from the cache response path.
           pte_latch <= dram_latched;
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
                           !aligned[1] && !(ptw_mxr && aligned[3])) begin
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
              end else if (ptw_prv == 1 && aligned[4] && (ptw_access == 0 || !ptw_sum)) begin
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

`ifdef SIMULATE
                  case (ptw_level)
                    2: ptw_stat_leaf_1g <= ptw_stat_leaf_1g + 1;
                    1: ptw_stat_leaf_2m <= ptw_stat_leaf_2m + 1;
                    default: begin
                       if (aligned[63])
                          ptw_stat_leaf_64k_napot <= ptw_stat_leaf_64k_napot + 1;
                       else
                          ptw_stat_leaf_4k <= ptw_stat_leaf_4k + 1;
                    end
                  endcase
`endif

                  case (ptw_level)
                    2: hpm_ptw_leaf_1g_pulse <= 1;
                    1: hpm_ptw_leaf_2m_pulse <= 1;
                    default: begin
                       if (aligned[63])
                          hpm_ptw_leaf_napot_pulse <= 1;
                       else
                          hpm_ptw_leaf_4k_pulse <= 1;
                    end
                  endcase

                  if (ptw_level == 0 && aligned[63]) begin
                     hpm_tlb_uncached_napot_pulse <= 1;
                     route_translated_addr(mem_addr, {1'b0, aligned[4:1]}, ptw_return);
                  end else if (ptw_level == 2) begin
                     hpm_tlb_uncached_1g_pulse <= 1;
                     route_translated_addr(mem_addr, {1'b0, aligned[4:1]}, ptw_return);
                  end else begin
                     stage_tlb_insert(ptw_va, mem_addr, ptw_level, ptw_access, ptw_prv,
                                      {1'b0, aligned[4:1]},
                                      ptw_satp, ptw_sum, ptw_mxr);
                     ptw_route_pa <= mem_addr;
                     ptw_route_perm <= {1'b0, aligned[4:1]};
                     ptw_route_return <= ptw_return;
                     state <= `S_TLB_INSERT;
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

        `S_TLB_INSERT: begin
           commit_staged_tlb_insert;
           route_translated_addr(ptw_route_pa, ptw_route_perm, ptw_route_return);
        end

        `S_PTW_LAUNCH: begin
           if (phys_region(ptw_pte_addr) == `REGION_BRAM ||
               phys_region(ptw_pte_addr) == `REGION_DRAM) begin
              ptw_direct_addr  <= ptw_pte_addr[30:3];
              ptw_direct_read  <= 1;
              dram_addr        <= ptw_pte_addr[30:3];
              state            <= `S_DRAM_PTW_WAIT;
           end else begin
              cause = ptw_access == 0 ? `TRAP_INSTRUCTIONPAGE_FAULT :
                      ptw_access == 1 ? `TRAP_LOAD_PAGE_FAULT :
                                        `TRAP_STORE_PAGE_FAULT;
              tval = ptw_va;
              state <= `S_EXCEPTION;
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
              aligned = 128'd0;
           insn = {aligned[15:0], insn_half};
           stage_rf_decode_current(pc, frontend_fallthrough_pc(pc, insn),
                                   frontend_fallthrough_pc(pc, insn),
                                   insn, 2'd0, fetch_from_dram);
        end

        `S_DRAM_FETCH_WAIT: if (dram_readdatavalid) begin
           dram_latched            <= dram_readdata;
           dram_latched_next       <= dram_readdata_next;
           dram_latched_next_valid <= dram_readdata_next_valid;
           state                   <= `S_FETCH2_DRAM;
        end

        `S_DRAM_FETCH_HALF_WAIT: if (dram_readdatavalid) begin
           dram_latched            <= dram_readdata;
           dram_latched_next       <= dram_readdata_next;
           dram_latched_next_valid <= dram_readdata_next_valid;
           state                   <= `S_FETCH2_HALF;
        end

        `S_DRAM_PTW_WAIT: begin
           if (ptw_direct_readdatavalid_r) begin
              dram_latched <= ptw_direct_readdata_r;
              state        <= `S_PTW_READ;
           end else begin
              try_issue_queued_decode_preserve_state(ptw_access == 2'd1 &&
                                                     ptw_return == `S_LOAD_ALIGN,
                                                     !write_back_fp_valid, write_back_register,
                                                     write_back_fp_valid, write_back_fp_register);
           end
        end

        `S_DRAM_LOAD_WAIT: begin
           if (dram_readdatavalid) begin
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
                    finish_load_writeback();
                    if (do_atomic)
                       state <= `S_AMO;
                    else
                       retire_linear_fetch();
                 end else begin
                    // Access crosses a cache-line boundary and the second line
                    // missed during the parallel lookup; request it only now.
                    dram_latched    <= dram_readdata;
                    dram_addr <= mem_addr[30:3] + 1;
                    dram_va   <= {mem_va[63:3], 3'b000} + 64'd8;
                    dram_asid <= mem_asid;
                    dram_perm <= mem_perm;
                    dram_ctx  <= mem_ctx;
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
                 finish_load_writeback();
                 if (do_atomic)
                    state <= `S_AMO;
                 else
                    retire_linear_fetch();
              end
           end else begin
              try_issue_queued_decode_preserve_state(!do_atomic,
                                                     !write_back_fp_valid, write_back_register,
                                                     write_back_fp_valid, write_back_fp_register);
           end
        end

        `S_DRAM_LOAD2_WAIT: begin
           if (dram_readdatavalid) begin
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
              finish_load_writeback();
              if (do_atomic)
                 state <= `S_AMO;
              else
                 retire_linear_fetch();
           end else begin
              try_issue_queued_decode_preserve_state(!do_atomic,
                                                     !write_back_fp_valid, write_back_register,
                                                     write_back_fp_valid, write_back_fp_register);
           end
        end

        `S_DRAM_STORE_WAIT: begin
           if (dram_write_ready) begin
              // Issue the first write now that the master is idle
              dram_write <= 1;
              state      <= `S_DRAM_STORE_RESP_ARM;
           end else begin
              try_issue_queued_decode_preserve_state(1'b1,
                                                     !write_back_fp_valid, write_back_register,
                                                     write_back_fp_valid, write_back_fp_register);
           end
        end

        `S_DRAM_STORE2: begin
           if (dram_write_ready) begin
              dram_addr       <= dram2_addr;
              dram_va         <= dram2_va;
              dram_asid       <= dram2_asid;
              dram_perm       <= dram2_perm;
              dram_ctx        <= dram2_ctx;
              dram_writedata  <= dram2_data_part;
              dram_wstrb      <= dram2_wstrb;
              dram_write      <= 1;
              dram_store_split <= 0;
              state           <= `S_DRAM_STORE_RESP_ARM;
           end else begin
              try_issue_queued_decode_preserve_state(1'b1,
                                                     !write_back_fp_valid, write_back_register,
                                                     write_back_fp_valid, write_back_fp_register);
           end
        end

        `S_DRAM_STORE_RESP_ARM: begin
           try_issue_queued_decode_preserve_state(1'b1,
                                                  !write_back_fp_valid, write_back_register,
                                                  write_back_fp_valid, write_back_fp_register);
           state <= `S_DRAM_STORE_RESP_WAIT;
        end

        `S_DRAM_STORE_RESP_WAIT: begin
           if (dram_write_done) begin
              if (dram_store_split) begin
                 state <= `S_DRAM_STORE2;
              end else begin
                 retire_linear_fetch();
              end
           end else begin
              try_issue_queued_decode_preserve_state(1'b1,
                                                     !write_back_fp_valid, write_back_register,
                                                     write_back_fp_valid, write_back_fp_register);
           end
        end

        `S_BUS_TIMEOUT: begin
           cause_intr = 0;
           cause = bus_timeout_cause;
           tval = bus_timeout_tval;
           write_back_register = 0;
           state <= `S_EXCEPTION;
        end

      endcase
`ifdef PC_TRACE
      end
`endif

      // RF BRAM data is available one cycle after ID drives rs1/rs2.  This
      // readiness advances even while the backend FSM remains in a long
      // execute/wait state after a background decode launch.
      if (!core_reset_now && id_valid && !id_rf_ready)
         id_rf_ready <= 1;

      // Pre-arm BRAM rs1/rs2 reads for the next queued decode. Only when
      // state == S_FETCH1: in any other state, the current instruction may
      // be repurposing rs1/rs2 (e.g. S_EXECUTE for FMA writes rs1 with the
      // rs3 register index before transitioning to S_CVFPU_FMA_RF2 — the
      // pre-arm would clobber it).
      if (!core_reset_now && !id_valid && rf_decode_valid &&
          !rf_decode_prearmed && !rf_decode_prearm_block &&
          state == `S_FETCH1) begin
         rs1 <= rf_decode_rs1;
         rs2 <= rf_decode_rs2;
         rf_decode_prearmed <= 1;
      end

      if (!core_reset_now && !frontend_flush_this_cycle &&
          frontend_spec_fetch_state(state))
         try_frontend_speculative_fetch_buf_enqueue();

      if (!core_reset_now && !frontend_flush_this_cycle &&
          frontend_spec_miss_state(state))
         try_frontend_speculative_miss_start();

      if (!core_reset_now && !frontend_flush_this_cycle &&
          frontend_miss_valid && dram_readdatavalid) begin
         frontend_miss_valid      <= 0;
         frontend_miss_done       <= 1;
         frontend_miss_data       <= dram_readdata;
         frontend_miss_next_data  <= dram_readdata_next;
         frontend_miss_next_valid <= dram_readdata_next_valid;
      end

      // Bus timeout: fault if an external bus access doesn't respond
      begin : bus_timeout_logic
         reg bus_waiting;
         bus_waiting = state == `S_DRAM_FETCH_WAIT || state == `S_DRAM_FETCH_HALF_WAIT ||
                       state == `S_DRAM_LOAD_WAIT  || state == `S_DRAM_LOAD2_WAIT ||
                       state == `S_DRAM_PTW_WAIT   ||
                       state == `S_DRAM_STORE_WAIT || state == `S_DRAM_STORE2 ||
                       state == `S_DRAM_STORE_RESP_WAIT || state == `S_DRAM_STORE_RESP_ARM ||
                       state == `S_MMIO_ALIGN ||
                       (state == `S_FRONTEND_MISS_WAIT && frontend_miss_valid);
         bus_timeout_expired <= bus_waiting && &bus_timeout_ctr;
         if (bus_waiting) begin
            bus_timeout_ctr <= bus_timeout_ctr + 1;
            case (state)
              `S_DRAM_FETCH_WAIT, `S_DRAM_FETCH_HALF_WAIT, `S_FRONTEND_MISS_WAIT: begin
                 bus_timeout_cause <= `TRAP_INSTRUCTION_ACCESS_FAULT;
                 bus_timeout_tval  <= state == `S_FRONTEND_MISS_WAIT ? frontend_miss_pc : dram_va;
              end
              `S_DRAM_STORE_WAIT, `S_DRAM_STORE2, `S_DRAM_STORE_RESP_WAIT, `S_DRAM_STORE_RESP_ARM: begin
                 bus_timeout_cause <= `TRAP_STORE_ACCESS_FAULT;
                 bus_timeout_tval  <= dram_va;
              end
              `S_DRAM_PTW_WAIT: begin
                 bus_timeout_cause <= ptw_access == 0 ? `TRAP_INSTRUCTION_ACCESS_FAULT :
                                      ptw_access == 2 || ptw_access == 3 ? `TRAP_STORE_ACCESS_FAULT :
                                      `TRAP_LOAD_ACCESS_FAULT;
                 bus_timeout_tval <= ptw_va;
              end
              default: begin // S_DRAM_LOAD_WAIT, S_DRAM_LOAD2_WAIT, S_MMIO_ALIGN
                 bus_timeout_cause <= `TRAP_LOAD_ACCESS_FAULT;
                 bus_timeout_tval  <= state == `S_MMIO_ALIGN ? mmio_timeout_tval : dram_va;
              end
            endcase
            if (bus_timeout_expired) begin
               bus_timeout_ctr     <= 0;
               bus_timeout_expired <= 0;
               csr_mig_timeouts <= csr_mig_timeouts + 1;
               // Late R beats from an abandoned read are silently swallowed by
               // the AXI master (it gates dram_readdatavalid on bus_waiting), so
               // no separate "abandon" handshake is needed.
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
               frontend_miss_valid <= 0;
               frontend_miss_done <= 0;
               state <= `S_BUS_TIMEOUT;
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
         fp_int_result <= 0;
         fp_int_fflags <= 0;
         csr_read_result <= 0;
         npc <= `RESET_PC;
         clear_frontend_cmd();
         frontend_cmd_epoch <= 0;
         fetch_epoch <= 0;
         frontend_cmd_pc <= `RESET_PC;
         frontend_cmd_prv <= 3;
         frontend_cmd_asid <= 0;
         frontend_redirect_valid <= 0;
         frontend_redirect_pc <= `RESET_PC;
         frontend_redirect_prv <= 3;
         frontend_redirect_epoch <= 0;
         f_latched_hit <= 0;
         f_latched_insn <= 0;
         f_latched_offset <= 0;
         f_latched_next_pc <= `RESET_PC;
         f_latched_prediction_kind <= 0;
         f_latched_cmd_pc <= `RESET_PC;
         f_latched_cmd_prv <= 3;
         f_latched_cmd_epoch <= 0;
         f_state <= `F_IDLE;
         ex_state <= `EX_IDLE;
         retire_now_q <= 0;
         frontend_miss_valid <= 0;
         frontend_miss_done <= 0;
         frontend_miss_pc <= `RESET_PC;
         frontend_miss_prv <= 3;
         frontend_miss_asid <= 0;
         frontend_miss_epoch <= 0;
         frontend_miss_data <= 0;
         frontend_miss_next_data <= 0;
         frontend_miss_next_valid <= 0;
         frontend_miss_wait_action <= FRONTEND_MISS_WAIT_CONSUME;
         rf_decode_head <= 0;
         rf_decode_tail <= 0;
         rf_decode_count <= 0;
         rf_decode_prearmed <= 0;
         rf_decode_prearm_block = 1;
         frontend_decode_pending_valid <= 0;
         frontend_decode_pending_drain = 0;
         id_valid <= 0;
         id_rf_ready <= 0;
         id_pc <= `RESET_PC;
         id_next_pc <= `RESET_PC;
         id_predicted_pc <= `RESET_PC;
         id_prv <= 0;
         id_epoch <= 0;
         id_insn <= 0;
         id_prediction_kind <= 0;
         id_rd <= 0;
         id_rs1 <= 0;
         id_rs2 <= 0;
         id_shamt <= 0;
         execute_req_valid <= 0;
         execute_req_pc <= `RESET_PC;
         execute_req_next_pc <= `RESET_PC;
         execute_req_predicted_pc <= `RESET_PC;
         execute_req_prv <= 0;
         execute_req_epoch <= 0;
         execute_req_insn <= 0;
         execute_req_prediction_kind <= 0;
         execute_req_rd <= 0;
         execute_req_rs1 <= 0;
         execute_req_rs2 <= 0;
         execute_req_shamt <= 0;
         execute_res_valid <= 0;
         pre_npc <= `RESET_PC;
        pre_jalr_target <= `RESET_PC;
        pre_branch_target <= `RESET_PC;
        pre_branch_taken <= 0;
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
         csr_senvcfg      <= 0;
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
         frontend_buf_flush <= 1'b1;
         f_latched_hit <= 0;
         f_latched_insn <= 0;
         f_latched_offset <= 0;
         f_state <= `F_IDLE;
         ex_state <= `EX_IDLE;
         muldiv_start_op <= `MULDIV_MUL;
         uart_tx_head     <= 0;
         uart_tx_tail     <= 0;
         uart_rx_head     <= 0;
         uart_rx_tail     <= 0;
         uart_rx_front_valid <= 0;
         uart_rx_refill_pending <= 0;
         uart_break_count <= 0;
         uart_thre_pending <= 0;
         plic_pending     <= 0;
         plic_in_service  <= 0;
         plic_enabled     <= 0;
         plic_threshold   <= 0;
         fetch_from_dram  <= 0;
         dram_latched_next_valid <= 0;
         translated       <= 0;
         dram_read        <= 0;
         dram_write       <= 0;
         dram_instr       <= 0;
         dram_va          <= 0;
         dram_asid        <= 0;
         dram_perm        <= CACHE_PERM_PHYS;
         dram_ctx         <= 0;
         mem_va           <= 0;
         mem_asid         <= 0;
         mem_perm         <= CACHE_PERM_PHYS;
         mem_ctx          <= 0;
         mmio_timeout_tval <= 0;
         dram2_va         <= 0;
         dram2_asid       <= 0;
         dram2_perm       <= CACHE_PERM_PHYS;
         dram2_ctx        <= 0;
         dram_store_split <= 0;
         ptw_direct_read <= 0;
         ptw_direct_addr <= 0;
         cache_flush_req  <= 0;
         vhpr_epoch_bump_req <= 0;
         mig_latency_ctr  <= 0;
         mig_prev_waiting <= 0;
         hpm_counter_wr_en <= 0;
         hpm_event_wr_en   <= 0;
         tlb_4k_rd_idx     <= 0;
         tlb_2m_rd_idx     <= 0;
         tlb_4k_wr_idx     <= 0;
         tlb_2m_wr_idx     <= 0;
         tlb_4k_wr_en      <= 0;
         tlb_2m_wr_en      <= 0;
         tlb_4k_wr_data    <= 0;
         tlb_2m_wr_data    <= 0;
         tlb_insert_level  <= 0;
         tlb_insert_4k_idx <= 0;
         tlb_insert_2m_idx <= 0;
         tlb_insert_4k_data <= 0;
         tlb_insert_2m_data <= 0;
         ptw_route_pa      <= 0;
         ptw_route_return  <= 0;
         tlb_4k_hit_pa     <= 0;
         tlb_2m_hit_pa     <= 0;
         tlb_hit_pa        <= 0;
         tlb_latched_4k_hit <= 0;
         tlb_latched_2m_hit <= 0;
         hpm_tlb_insert_4k_pulse <= 0;
         hpm_tlb_insert_2m_pulse <= 0;
         hpm_tlb_evict_4k_pulse <= 0;
         hpm_tlb_evict_2m_pulse <= 0;
         hpm_tlb_uncached_1g_pulse <= 0;
         hpm_tlb_uncached_napot_pulse <= 0;
         hpm_ptw_leaf_4k_pulse <= 0;
         hpm_ptw_leaf_2m_pulse <= 0;
         hpm_ptw_leaf_1g_pulse <= 0;
         hpm_ptw_leaf_napot_pulse <= 0;
         flush_tlb;
`ifdef SIMULATE
         fetch_buf_stat_hits <= 0;
         fetch_buf_stat_misses <= 0;
         ptw_stat_leaf_4k <= 0;
         ptw_stat_leaf_64k_napot <= 0;
         ptw_stat_leaf_2m <= 0;
         ptw_stat_leaf_1g <= 0;
         tlb_stat_lookups <= 0;
         tlb_stat_hits <= 0;
         tlb_stat_misses <= 0;
         tlb_stat_hits_4k <= 0;
         tlb_stat_hits_2m <= 0;
         tlb_stat_inserts_4k <= 0;
         tlb_stat_inserts_2m <= 0;
         tlb_stat_evicts_4k <= 0;
         tlb_stat_evicts_2m <= 0;
         tlb_stat_uncached_1g <= 0;
         tlb_stat_uncached_napot <= 0;
         tlb_stat_entries_4k = 0;
         tlb_stat_entries_2m = 0;
         tlb_stat_entries_1g = 0;
`endif
      end
   end

   assign dram_readdatavalid = dram_readdatavalid_r;
   assign dram_readdata      = dram_readdata_r;
   assign dram_readdata_next = dram_readdata_next_r;
   assign dram_readdata_next_valid = dram_readdata_next_valid_r;
   assign dram_write_ready   = cache_idle;
   assign dram_write_done    = dram_write_done_r;

   always @(posedge clock) begin
      dram_readdatavalid_r <= 0;
      dram_readdata_next_valid_r <= 0;
      ptw_direct_readdatavalid_r <= 0;
      dram_write_done_r <= 0;
      cache_bram_write_done <= 0;
      cache_cbo_done_r <= 0;
      cache_way0_tag_wr_en <= 0;
      cache_way1_tag_wr_en <= 0;
      cache_tag_wr_all <= 0;

      if (ptw_direct_read)
         ptw_direct_probe_pending <= 1;

      if (ptw_direct_wait_probe && cache_cbo_done_r) begin
         ptw_direct_wait_probe <= 0;
         ptw_direct_pending <= 1;
      end

      if (ptw_direct_wait_bram) begin
         ptw_direct_readdata_r <= ptw_direct_bram_word_bank
                                  ? mem1[ptw_direct_bram_word_idx]
                                  : mem0[ptw_direct_bram_word_idx];
         ptw_direct_readdatavalid_r <= 1;
         ptw_direct_wait_bram <= 0;
      end

      if (ptw_direct_wait_axi)
         mem_read_rsp_ready <= 1;
      if (ptw_direct_wait_axi && mem_read_rsp_valid && mem_read_rsp_ready) begin
         ptw_direct_readdata_r <= mem_read_rsp_data;
         ptw_direct_readdatavalid_r <= 1;
         mem_read_rsp_ready <= 0;
         ptw_direct_wait_axi <= 0;
      end

      case (cache_state)
        CACHE_IDLE: begin
           if (cache_flush_all) begin
              if (vhpr_epoch_bump_pending) begin : vhpr_epoch_bump_start
                 reg [VHPR_EPOCH_BITS-1:0] next_epoch;
                 next_epoch = vhpr_epoch + {{VHPR_EPOCH_BITS-1{1'b0}}, 1'b1};
                 vhpr_next_epoch <= next_epoch;
                 vhpr_epoch_update_pending <= 1'b1;
                 vhpr_epoch_bump_ack <= vhpr_epoch_bump_req;
                 csr_vhpr_epoch_bumps <= csr_vhpr_epoch_bumps + 1;
                 if (next_epoch == {VHPR_EPOCH_BITS{1'b0}})
                    csr_vhpr_epoch_rollovers <= csr_vhpr_epoch_rollovers + 1;
              end
              cache_flush_idx <= 0;
              cache_flush_way <= 0;
              cache_req_instr <= 0;
              cache_way0_rd_idx <= 0;
              cache_way1_rd_idx <= 0;
              cache_way0_bank0_rd_idx <= 0;
              cache_way1_bank0_rd_idx <= 0;
              cache_state <= CACHE_FLUSH_READ;
           end else if (vhpr_epoch_bump_pending) begin : vhpr_epoch_bump_only
              reg [VHPR_EPOCH_BITS-1:0] next_epoch;
              next_epoch = vhpr_epoch + {{VHPR_EPOCH_BITS-1{1'b0}}, 1'b1};
              vhpr_epoch_bump_ack <= vhpr_epoch_bump_req;
              csr_vhpr_epoch_bumps <= csr_vhpr_epoch_bumps + 1;
              if (next_epoch == {VHPR_EPOCH_BITS{1'b0}}) begin
                 csr_vhpr_epoch_rollovers <= csr_vhpr_epoch_rollovers + 1;
                 vhpr_next_epoch <= next_epoch;
                 vhpr_epoch_update_pending <= 1'b1;
                 cache_flush_idx <= 0;
                 cache_flush_way <= 0;
                 cache_req_instr <= 0;
                 cache_way0_rd_idx <= 0;
                 cache_way1_rd_idx <= 0;
                 cache_way0_bank0_rd_idx <= 0;
                 cache_way1_bank0_rd_idx <= 0;
                 cache_state <= CACHE_FLUSH_READ;
              end else begin
                 vhpr_epoch <= next_epoch;
              end
	   end else if (cache_cbo_flush) begin
              csr_vhpr_cbo_probes <= csr_vhpr_cbo_probes + 1;
	      cache_addr              <= {33'd0, cache_cbo_line_addr, 6'd0};
	      cache_req_ptag          <= cache_cbo_ptag;
	      cache_req_cbo           <= 1;
	      cache_req_write         <= 0;
	      cache_req_instr         <= 0;
	      cache_req_line_addr     <= cache_cbo_line_addr;
	      cache_probe_color       <= 0;
	      cache_probe_found       <= 0;
	      cache_probe_found_way   <= 0;
	      cache_probe_found_idx   <= 0;
	      cache_probe_found_dirty <= 0;
              cache_probe_active      <= 1;
              cache_way0_rd_idx       <= {3'd0, cache_cbo_line_addr[11:6]};
              cache_way1_rd_idx       <= {3'd0, cache_cbo_line_addr[11:6]};
              cache_way0_bank0_rd_idx <= {3'd0, cache_cbo_line_addr[11:6]};
              cache_way1_bank0_rd_idx <= {3'd0, cache_cbo_line_addr[11:6]};
              cache_state             <= CACHE_PROBE_READ;
	   end else if (ptw_direct_probe_pending) begin
              csr_vhpr_ptw_probes <= csr_vhpr_ptw_probes + 1;
	      cache_addr              <= {33'd0, ptw_direct_addr[27:3], 6'd0};
	      cache_req_ptag          <= ptw_direct_addr[27:9];
	      cache_req_cbo           <= 1;
	      cache_req_write         <= 0;
	      cache_req_instr         <= 0;
	      cache_req_line_addr     <= ptw_direct_addr[27:3];
	      cache_probe_color       <= 0;
	      cache_probe_found       <= 0;
	      cache_probe_found_way   <= 0;
	      cache_probe_found_idx   <= 0;
	      cache_probe_found_dirty <= 0;
              cache_probe_active      <= 1;
              cache_way0_rd_idx       <= {3'd0, ptw_direct_addr[8:3]};
              cache_way1_rd_idx       <= {3'd0, ptw_direct_addr[8:3]};
              cache_way0_bank0_rd_idx <= {3'd0, ptw_direct_addr[8:3]};
              cache_way1_bank0_rd_idx <= {3'd0, ptw_direct_addr[8:3]};
              ptw_direct_probe_pending <= 0;
              ptw_direct_wait_probe    <= 1;
              cache_state             <= CACHE_PROBE_READ;
           end else if (ptw_direct_pending && !ptw_direct_wait_bram &&
                        !ptw_direct_wait_axi) begin
              if (ptw_direct_from_bram) begin
                 begin : ptw_direct_bram_req
                    reg [63:0] bram_addr;
                    bram_addr = ptw_direct_addr64 - cache_bram_base_addr;
                    ptw_direct_bram_word_idx  <= bram_addr[`MEM_SIZE_LG2-1:4];
                    ptw_direct_bram_word_bank <= bram_addr[3];
                 end
                 ptw_direct_wait_bram <= 1;
                 ptw_direct_pending <= 0;
              end else begin
                 if (!mem_read_req_valid) begin
                    mem_read_req_addr <= ptw_direct_addr;
                    mem_read_req_valid <= 1;
                 end else if (mem_read_req_ready) begin
                    mem_read_req_valid <= 0;
                    ptw_direct_wait_axi <= 1;
                    ptw_direct_pending <= 0;
                 end
              end
           end else if (dram_read) begin
              csr_vhpr_reads <= csr_vhpr_reads + 1;
              cache_addr          <= cache_dram_addr;
              cache_req_va        <= dram_va;
              cache_req_asid      <= dram_asid;
              cache_req_perm      <= dram_perm;
              cache_req_ctx       <= dram_ctx;
              cache_req_write     <= 0;
              cache_req_instr     <= dram_instr;
              cache_req_cbo       <= 0;
              cache_req_vtag      <= cache_vtag(dram_va);
              cache_req_next_vtag <= cache_vtag(cache_dram_next_va);
              cache_req_ptag      <= cache_dram_ptag;
              cache_req_bank      <= dram_addr[2:0];
              cache_req_next_bank <= dram_addr[2:0] + 3'd1;
              cache_req_same_line <= dram_addr[2:0] != 3'd7;
              cache_way0_rd_idx   <= cache_way0_index(dram_va, dram_asid);
              cache_way1_rd_idx   <= cache_way1_index(dram_va, dram_asid);
              cache_way0_bank0_rd_idx <= dram_addr[2:0] == 3'd7
                                      ? cache_way0_index(cache_dram_next_va, dram_asid)
                                      : cache_way0_index(dram_va, dram_asid);
              cache_way1_bank0_rd_idx <= dram_addr[2:0] == 3'd7
                                      ? cache_way1_index(cache_dram_next_va, dram_asid)
                                      : cache_way1_index(dram_va, dram_asid);
              cache_way0_next_rd_idx <= cache_way0_index(cache_dram_next_va, dram_asid);
              cache_way1_next_rd_idx <= cache_way1_index(cache_dram_next_va, dram_asid);
              cache_state         <= CACHE_TAG_READ;
           end else if (dram_write) begin
              csr_vhpr_writes <= csr_vhpr_writes + 1;
              cache_addr          <= cache_dram_addr;
              cache_req_va        <= dram_va;
              cache_req_asid      <= dram_asid;
              cache_req_perm      <= dram_perm;
              cache_req_ctx       <= dram_ctx;
              cache_req_write     <= 1;
              cache_req_instr     <= 0;
              cache_req_cbo       <= 0;
              cache_req_vtag      <= cache_vtag(dram_va);
              cache_req_next_vtag <= cache_vtag(cache_dram_next_va);
              cache_req_ptag      <= cache_dram_ptag;
              cache_req_bank      <= dram_addr[2:0];
              cache_req_next_bank <= dram_addr[2:0] + 3'd1;
              cache_req_same_line <= 1;
              cache_store_data    <= dram_writedata;
              cache_store_strb    <= dram_wstrb;
              cache_way0_rd_idx   <= cache_way0_index(dram_va, dram_asid);
              cache_way1_rd_idx   <= cache_way1_index(dram_va, dram_asid);
              cache_way0_bank0_rd_idx <= cache_way0_index(dram_va, dram_asid);
              cache_way1_bank0_rd_idx <= cache_way1_index(dram_va, dram_asid);
              cache_way0_next_rd_idx <= cache_way0_index(cache_dram_next_va, dram_asid);
              cache_way1_next_rd_idx <= cache_way1_index(cache_dram_next_va, dram_asid);
              cache_state         <= CACHE_TAG_READ;
           end
        end

        CACHE_TAG_READ: begin
           cache_state <= CACHE_TAG_WAIT;
        end

        CACHE_TAG_WAIT: begin
           cache_state <= CACHE_TAG_CHECK;
        end

        CACHE_TAG_CHECK: begin : cache_tag_check
           reg way0_hit, way1_hit, way0_next_hit, way1_next_hit;
           reg target_way;
           reg [`CACHE_META_BITS-1:0] target_meta;

           way0_hit = cache_meta_valid(cache_way0_tag_rd_data) &&
                      cache_meta_epoch(cache_way0_tag_rd_data) == vhpr_epoch &&
                      cache_meta_asid(cache_way0_tag_rd_data) == cache_req_asid &&
                      cache_meta_vtag(cache_way0_tag_rd_data) == cache_req_vtag &&
                      cache_perm_allows_ctx(cache_meta_perm(cache_way0_tag_rd_data),
                                            cache_req_ctx);
           way1_hit = cache_meta_valid(cache_way1_tag_rd_data) &&
                      cache_meta_epoch(cache_way1_tag_rd_data) == vhpr_epoch &&
                      cache_meta_asid(cache_way1_tag_rd_data) == cache_req_asid &&
                      cache_meta_vtag(cache_way1_tag_rd_data) == cache_req_vtag &&
                      cache_perm_allows_ctx(cache_meta_perm(cache_way1_tag_rd_data),
                                            cache_req_ctx);
           way0_next_hit = cache_meta_valid(cache_way0_tag_next_rd_data) &&
                           cache_meta_epoch(cache_way0_tag_next_rd_data) == vhpr_epoch &&
                           cache_meta_asid(cache_way0_tag_next_rd_data) == cache_req_asid &&
                           cache_meta_vtag(cache_way0_tag_next_rd_data) == cache_req_next_vtag &&
                           cache_perm_allows_ctx(cache_meta_perm(cache_way0_tag_next_rd_data),
                                                 cache_req_ctx);
           way1_next_hit = cache_meta_valid(cache_way1_tag_next_rd_data) &&
                           cache_meta_epoch(cache_way1_tag_next_rd_data) == vhpr_epoch &&
                           cache_meta_asid(cache_way1_tag_next_rd_data) == cache_req_asid &&
                           cache_meta_vtag(cache_way1_tag_next_rd_data) == cache_req_next_vtag &&
                           cache_perm_allows_ctx(cache_meta_perm(cache_way1_tag_next_rd_data),
                                                 cache_req_ctx);

           cache_lookup_hit <= way0_hit || way1_hit;
           cache_lookup_hit_way <= way1_hit;
           cache_lookup_next_hit <= way0_next_hit || way1_next_hit;
           cache_lookup_next_hit_way <= way1_next_hit;
           cache_lookup_data <= cache_selected_bank_data(way1_hit, cache_req_bank);
           cache_lookup_next_data <= cache_req_same_line
                                    ? cache_selected_bank_data(way1_hit, cache_req_next_bank)
                                    : cache_selected_bank_data(way1_next_hit, 3'd0);
           cache_lookup_next_valid <= cache_req_same_line || way0_next_hit || way1_next_hit;
           target_way = !cache_meta_valid(cache_way0_tag_rd_data) ? 1'b0 :
                        !cache_meta_valid(cache_way1_tag_rd_data) ? 1'b1 :
                        cache_meta_epoch(cache_way0_tag_rd_data) != vhpr_epoch ? 1'b0 :
                        cache_meta_epoch(cache_way1_tag_rd_data) != vhpr_epoch ? 1'b1 :
                        cache_replace_way;
           target_meta = target_way ? cache_way1_tag_rd_data : cache_way0_tag_rd_data;
           cache_lookup_dirty <= (way0_hit || way1_hit)
                                 ? (way1_hit ? cache_meta_dirty(cache_way1_tag_rd_data)
                                             : cache_meta_dirty(cache_way0_tag_rd_data))
                                 : (cache_meta_valid(target_meta) && cache_meta_dirty(target_meta));
           cache_target_way <= target_way;
           cache_target_idx <= target_way ? cache_way1_rd_idx : cache_way0_rd_idx;
           cache_target_valid <= cache_meta_valid(target_meta);
           cache_target_dirty <= cache_meta_valid(target_meta) && cache_meta_dirty(target_meta);
           cache_target_ptag <= cache_meta_ptag(target_meta);
           cache_state <= CACHE_HIT_RESP;
        end

        CACHE_HIT_RESP: begin : cache_hit_resp
           reg        next_line_safe;

           next_line_safe = cache_req_same_line || cache_req_va[11:3] != 9'h1ff;

           if (cache_lookup_hit) begin
              if (cache_req_write) begin
                 csr_vhpr_write_hits <= csr_vhpr_write_hits + 1;
                 cache_state <= CACHE_HIT_WRITE;
              end else begin
                 csr_vhpr_read_hits <= csr_vhpr_read_hits + 1;
                 dram_readdata_r <= cache_lookup_data;
                 dram_readdata_next_r <= cache_lookup_next_data;
                 dram_readdata_next_valid_r <= cache_lookup_next_valid &&
                                                next_line_safe;
                 dram_readdatavalid_r <= 1;
                 cache_state <= CACHE_IDLE;
              end
           end else begin
              if (cache_req_write)
                 csr_vhpr_write_misses <= csr_vhpr_write_misses + 1;
              else
                 csr_vhpr_read_misses <= csr_vhpr_read_misses + 1;
              cache_start_fill_request();
           end
        end

        CACHE_HIT_WRITE: begin
           cache_way0_tag_wr_en <= !cache_lookup_hit_way;
           cache_way1_tag_wr_en <= cache_lookup_hit_way;
           cache_tag_wr_idx     <= cache_lookup_hit_way ? cache_way1_rd_idx : cache_way0_rd_idx;
           cache_tag_wr_data    <= cache_make_meta(1'b1, 1'b1, cache_req_asid,
                                                    cache_req_perm,
                                                    cache_req_vtag, cache_req_ptag,
                                                    vhpr_epoch);
           dram_write_done_r <= 1;
           cache_state <= CACHE_IDLE;
        end

        CACHE_FLUSH_READ: begin
           cache_state <= CACHE_FLUSH_WAIT;
        end

        CACHE_FLUSH_WAIT: begin
           cache_state <= CACHE_FLUSH_CHECK;
        end

        CACHE_FLUSH_CHECK: begin : cache_flush_check
           reg [`CACHE_META_BITS-1:0] flush_meta;

           flush_meta = cache_flush_way ? cache_way1_tag_rd_data : cache_way0_tag_rd_data;
           cache_way0_tag_wr_en <= !cache_flush_way;
           cache_way1_tag_wr_en <= cache_flush_way;
           cache_tag_wr_idx <= cache_flush_idx;
           cache_tag_wr_data <= 0;
           cache_tag_wr_all <= 1;
           if (cache_meta_valid(flush_meta) && cache_meta_dirty(flush_meta)) begin
              csr_vhpr_flush_evicts <= csr_vhpr_flush_evicts + 1;
              csr_vhpr_dirty_flush_evicts <= csr_vhpr_dirty_flush_evicts + 1;
              cache_victim_way <= cache_flush_way;
              cache_victim_idx <= cache_flush_idx;
              cache_victim_ptag <= cache_meta_ptag(flush_meta);
              cache_wb_base <= {33'd0, cache_meta_ptag(flush_meta), cache_flush_idx[5:0], 6'd0};
              cache_wb_beat <= 0;
              cache_wb_after_cbo <= 1'b0;
              cache_wb_then_fill <= 1'b0;
              cache_wb_after_flush <= 1'b1;
              cache_way0_rd_idx <= cache_flush_idx;
              cache_way1_rd_idx <= cache_flush_idx;
              cache_way0_bank0_rd_idx <= cache_flush_idx;
              cache_way1_bank0_rd_idx <= cache_flush_idx;
              cache_state <= CACHE_WB_PREP;
           end else begin
              if (cache_meta_valid(flush_meta)) begin
                 csr_vhpr_flush_evicts <= csr_vhpr_flush_evicts + 1;
              end
              cache_flush_next_line();
           end
        end

        CACHE_PROBE_READ: begin
           cache_state <= CACHE_PROBE_WAIT;
        end

        CACHE_PROBE_WAIT: begin
           cache_state <= CACHE_PROBE_CHECK;
        end

        CACHE_PROBE_CHECK: begin : cache_probe_check
           reg way0_phys_hit, way1_phys_hit;
           reg found;
           reg found_way;
           reg found_dirty;
           reg [`CACHE_INDEX_BITS-1:0] found_idx;
           reg [`CACHE_INDEX_BITS-1:0] next_probe_idx;
           reg same_as_target;

           way0_phys_hit = cache_meta_valid(cache_way0_tag_rd_data) &&
                           cache_meta_ptag(cache_way0_tag_rd_data) == cache_req_ptag;
           way1_phys_hit = cache_meta_valid(cache_way1_tag_rd_data) &&
                           cache_meta_ptag(cache_way1_tag_rd_data) == cache_req_ptag;

           found = cache_probe_found || way0_phys_hit || way1_phys_hit;
           found_way = cache_probe_found ? cache_probe_found_way : way1_phys_hit;
	   found_idx = cache_probe_found ? cache_probe_found_idx :
		       (cache_req_cbo ? cache_req_probe_line_index : cache_probe_line_index);
           found_dirty = cache_probe_found ? cache_probe_found_dirty :
                         (way1_phys_hit ? cache_meta_dirty(cache_way1_tag_rd_data)
                                        : cache_meta_dirty(cache_way0_tag_rd_data));

           if (!cache_probe_found && (way0_phys_hit || way1_phys_hit)) begin
              cache_probe_found <= 1;
              cache_probe_found_way <= way1_phys_hit;
	      cache_probe_found_idx <= cache_req_cbo ? cache_req_probe_line_index : cache_probe_line_index;
              cache_probe_found_dirty <= found_dirty;
           end

           if (cache_probe_color != 3'd7) begin
	      cache_probe_color <= cache_probe_color + 1;
	      next_probe_idx = {cache_probe_color + 1'b1,
				cache_req_cbo ? cache_req_line_addr[11:6] : cache_addr[11:6]};
              cache_way0_rd_idx <= next_probe_idx;
              cache_way1_rd_idx <= next_probe_idx;
              cache_way0_bank0_rd_idx <= next_probe_idx;
              cache_way1_bank0_rd_idx <= next_probe_idx;
              cache_state <= CACHE_PROBE_READ;
           end else begin
              cache_probe_active <= 0;
              cache_victim_way <= found_way;
              cache_victim_idx <= found_idx;
              cache_victim_ptag <= cache_req_ptag;
              same_as_target = found && found_way == cache_target_way && found_idx == cache_target_idx;
              if (found && !cache_req_cbo) begin
                 csr_vhpr_alias_evicts <= csr_vhpr_alias_evicts + 1;
                 if (found_dirty)
                    csr_vhpr_dirty_alias_evicts <= csr_vhpr_dirty_alias_evicts + 1;
              end
              cache_need_target_wb <= !cache_req_cbo && cache_target_dirty && !same_as_target;
              if (found && found_dirty) begin
                 cache_wb_base <= {33'd0, cache_req_ptag, found_idx[5:0], 6'd0};
                 cache_wb_beat <= 0;
                 cache_wb_after_cbo <= cache_req_cbo;
                 cache_wb_then_fill <= !cache_req_cbo;
                 cache_way0_rd_idx <= found_idx;
                 cache_way1_rd_idx <= found_idx;
                 cache_way0_bank0_rd_idx <= found_idx;
                 cache_way1_bank0_rd_idx <= found_idx;
                 cache_state <= CACHE_WB_PREP;
              end else if (found) begin
                 cache_state <= CACHE_INVALIDATE;
              end else if (cache_req_cbo) begin
                 cache_cbo_done_r <= 1;
                 cache_state <= CACHE_IDLE;
              end else if (cache_target_dirty) begin
                 csr_vhpr_victim_evicts <= csr_vhpr_victim_evicts + 1;
                 csr_vhpr_dirty_victim_evicts <= csr_vhpr_dirty_victim_evicts + 1;
                 cache_victim_way <= cache_target_way;
                 cache_victim_idx <= cache_target_idx;
                 cache_victim_ptag <= cache_target_ptag;
                 cache_wb_base <= {33'd0, cache_target_ptag, cache_target_idx[5:0], 6'd0};
                 cache_wb_beat <= 0;
                 cache_wb_after_cbo <= 1'b0;
                 cache_wb_then_fill <= 1'b0;
                 cache_way0_rd_idx <= cache_target_idx;
                 cache_way1_rd_idx <= cache_target_idx;
                 cache_way0_bank0_rd_idx <= cache_target_idx;
                 cache_way1_bank0_rd_idx <= cache_target_idx;
                 cache_state <= CACHE_WB_PREP;
              end else begin
                 cache_state <= CACHE_FILL_REQ;
              end
           end
        end

        CACHE_INVALIDATE: begin
           cache_way0_tag_wr_en <= !cache_victim_way;
           cache_way1_tag_wr_en <= cache_victim_way;
           cache_tag_wr_idx <= cache_victim_idx;
           cache_tag_wr_data <= 0;
           if (cache_req_cbo) begin
              cache_cbo_done_r <= 1;
              cache_state <= CACHE_IDLE;
           end else if (cache_need_target_wb) begin
              cache_need_target_wb <= 0;
              csr_vhpr_victim_evicts <= csr_vhpr_victim_evicts + 1;
              csr_vhpr_dirty_victim_evicts <= csr_vhpr_dirty_victim_evicts + 1;
              cache_victim_way <= cache_target_way;
              cache_victim_idx <= cache_target_idx;
              cache_victim_ptag <= cache_target_ptag;
              cache_wb_base <= {33'd0, cache_target_ptag, cache_target_idx[5:0], 6'd0};
              cache_wb_beat <= 0;
              cache_wb_after_cbo <= 1'b0;
              cache_wb_then_fill <= 1'b0;
              cache_way0_rd_idx <= cache_target_idx;
              cache_way1_rd_idx <= cache_target_idx;
              cache_way0_bank0_rd_idx <= cache_target_idx;
              cache_way1_bank0_rd_idx <= cache_target_idx;
              cache_state <= CACHE_WB_PREP;
           end else begin
              cache_state <= CACHE_FILL_REQ;
           end
        end

        CACHE_WB_PREP: begin
           cache_state <= CACHE_WB_READ_WAIT;
        end

        CACHE_WB_READ_WAIT: begin
           cache_state <= CACHE_WB_REQ;
        end

        CACHE_WB_REQ: begin
           if (cache_wb_to_bram) begin
              begin : cache_bram_wb_req
                 reg [63:0] bram_beat_addr;
                 bram_beat_addr = cache_wb_base + {58'd0, cache_wb_beat, 3'd0} -
                                  cache_bram_base_addr;
                 cache_bram_word_idx  <= bram_beat_addr[`MEM_SIZE_LG2-1:4];
                 cache_bram_word_bank <= bram_beat_addr[3];
                 cache_bram_wb_data   <= cache_selected_bank_data(cache_victim_way, cache_wb_beat);
              end
              cache_state <= CACHE_BRAM_WB_WRITE;
           end else begin
`ifdef SIMULATE
              if (cache_trace_enabled && !mem_wb_req_valid) begin
                 $display("%05d CACHE WBREQ line addr=%016h data0=%016h",
                          $time,
                          cache_wb_base,
                          cache_selected_bank_data(cache_victim_way, 3'd0));
              end
`endif
              if (!mem_wb_req_valid) begin
                 mem_wb_req_line_addr <= cache_wb_base[30:6];
                 mem_wb_req_line_data <= cache_selected_line_data(cache_victim_way);
                 mem_wb_req_valid     <= 1;
              end else if (mem_wb_req_ready) begin
                 mem_wb_req_valid <= 0;
                 cache_state <= CACHE_WB_WAIT;
              end
           end
        end

        CACHE_BRAM_WB_WRITE: begin
           if (cache_bram_word_bank)
              mem1[cache_bram_word_idx] <= cache_bram_wb_data;
           else
              mem0[cache_bram_word_idx] <= cache_bram_wb_data;
           cache_bram_write_done <= 1;
           cache_state <= CACHE_WB_WAIT;
        end

        CACHE_WB_WAIT: begin
           if (cache_wb_to_bram && cache_bram_write_done) begin
              if (cache_wb_beat == 3'd7) begin
                 cache_finish_writeback_line();
              end else begin
                 cache_wb_beat <= cache_wb_beat + 1;
                 cache_state <= CACHE_WB_REQ;
              end
           end else if (!cache_wb_to_bram) begin
              mem_wb_rsp_ready <= 1;
              if (mem_wb_rsp_valid && mem_wb_rsp_ready) begin
                 mem_wb_rsp_ready <= 0;
                 cache_finish_writeback_line();
              end
           end
        end

        CACHE_FILL_REQ: begin
`ifdef SIMULATE
           if (cache_trace_enabled && !cache_fill_from_bram && !mem_fill_req_valid) begin
              $display("%05d CACHE FILLREQ line addr=%016h",
                       $time,
                       cache_fill_base);
           end
`endif
           if (cache_fill_from_bram) begin
              begin : cache_bram_fill_req
                 reg [63:0] bram_beat_addr;
                 bram_beat_addr = cache_fill_base + {58'd0, cache_fill_beat, 3'd0} -
                                  cache_bram_base_addr;
                 cache_bram_word_idx  <= bram_beat_addr[`MEM_SIZE_LG2-1:4];
                 cache_bram_word_bank <= bram_beat_addr[3];
              end
              cache_state <= CACHE_BRAM_FILL_READ;
           end else begin
              if (!mem_fill_req_valid) begin
                 mem_fill_req_line_addr <= cache_fill_base[30:6];
                 mem_fill_req_valid <= 1;
              end else if (mem_fill_req_ready) begin
                 mem_fill_req_valid <= 0;
                 cache_state <= CACHE_FILL_LINE_WAIT;
              end
           end
        end

        CACHE_FILL_LINE_WAIT: begin
           mem_fill_rsp_ready <= 1;
           if (mem_fill_rsp_valid && mem_fill_rsp_ready) begin
              cache_fill_line_data <= mem_fill_rsp_data;
              mem_fill_rsp_ready <= 0;
              cache_fill_beat <= 0;
              cache_state <= CACHE_FILL_LINE_INSTALL;
           end
        end

        CACHE_BRAM_FILL_READ: begin
           cache_bram_read_data_stage <= cache_bram_word_bank ? mem1[cache_bram_word_idx]
                                                              : mem0[cache_bram_word_idx];
           cache_state <= CACHE_BRAM_FILL_CAPTURE;
        end

        CACHE_BRAM_FILL_CAPTURE: begin
           cache_bram_fill_data <= cache_bram_read_data_stage;
           cache_state <= CACHE_BRAM_FILL_COMMIT;
        end

        CACHE_BRAM_FILL_COMMIT,
        CACHE_FILL_LINE_INSTALL: begin
           if (cache_fill_data_valid) begin
              if (cache_fill_beat == cache_req_bank) begin
                 if (cache_req_write)
                    cache_fill_return_data <= merge_store_bytes(cache_fill_data, cache_store_data, cache_store_strb);
                 else
                    cache_fill_return_data <= cache_fill_data;
              end
              if (cache_fill_beat == cache_req_next_bank)
                 cache_fill_next_data <= cache_fill_data;
              if (cache_fill_beat == 3'd7) begin
                 csr_vhpr_fills <= csr_vhpr_fills + 1;
                 if (cache_target_valid && !cache_target_dirty)
                    csr_vhpr_victim_evicts <= csr_vhpr_victim_evicts + 1;
                 cache_way0_tag_wr_en <= !cache_target_way;
                 cache_way1_tag_wr_en <= cache_target_way;
                 cache_tag_wr_idx  <= cache_target_idx;
                 cache_tag_wr_data <= cache_make_meta(cache_req_write, 1'b1,
                                                       cache_req_asid,
                                                       cache_req_perm,
                                                       cache_req_vtag,
                                                       cache_req_ptag,
                                                       vhpr_epoch);
                 if (cache_req_write) begin
                    dram_write_done_r <= 1;
                 end else begin
                    dram_readdata_r <= cache_req_bank == 3'd7 ? cache_fill_data : cache_fill_return_data;
                    dram_readdata_next_r <= cache_req_same_line
                                             ? (cache_req_next_bank == 3'd7 ? cache_fill_data
                                                                            : cache_fill_next_data)
                                             : cache_lookup_next_data;
                    dram_readdata_next_valid_r <= (cache_req_same_line ||
                                                   cache_lookup_next_hit) &&
                                                   (cache_req_same_line ||
                                                    cache_req_va[11:3] != 9'h1ff);
                    dram_readdatavalid_r <= 1;
                 end
                 cache_state      <= CACHE_IDLE;
              end else begin
                 cache_fill_beat <= cache_fill_beat + 1;
                 cache_state     <= cache_bram_fill_commit ? CACHE_FILL_REQ :
                                                            CACHE_FILL_LINE_INSTALL;
              end
           end
        end

        default: cache_state <= CACHE_IDLE;
      endcase

      if (vhpr_stats_clear_pending) begin
         vhpr_clear_stats();
         vhpr_stats_clear_ack <= vhpr_stats_clear_req;
      end

      if (core_reset_now) begin
         cache_state <= CACHE_IDLE;
         dram_readdatavalid_r <= 0;
         dram_readdata_next_valid_r <= 0;
         dram_write_done_r <= 0;
         cache_bram_write_done <= 0;
         mem_fill_req_valid <= 0;
         mem_fill_rsp_ready <= 0;
         mem_wb_req_valid <= 0;
         mem_wb_rsp_ready <= 0;
         mem_read_req_valid <= 0;
         mem_read_rsp_ready <= 0;
         cache_way0_tag_wr_en <= 0;
         cache_way1_tag_wr_en <= 0;
         cache_tag_wr_all <= 0;
	 cache_cbo_done_r <= 0;
	 cache_req_instr <= 0;
	 cache_req_perm <= CACHE_PERM_PHYS;
	 cache_req_ctx <= 0;
	 cache_req_line_addr <= 0;
	 cache_flush_ack <= cache_flush_req;
         cache_wb_after_cbo <= 0;
         cache_wb_then_fill <= 0;
         cache_wb_after_flush <= 0;
         cache_need_target_wb <= 0;
         ptw_direct_probe_pending <= 0;
         ptw_direct_wait_probe <= 0;
         ptw_direct_pending <= 0;
         ptw_direct_wait_bram <= 0;
         ptw_direct_wait_axi <= 0;
         ptw_direct_readdatavalid_r <= 0;
         ptw_direct_readdata_r <= 0;
         vhpr_epoch <= 0;
         vhpr_next_epoch <= 0;
         vhpr_epoch_update_pending <= 0;
         vhpr_epoch_bump_ack <= vhpr_epoch_bump_req;
      end
   end

`ifdef SIMULATE
   always @(posedge clock) begin
      if (state_summary_enabled && !reset) begin
         state_stat_total_cycles <= state_stat_total_cycles + 1;
         if (hpm_instret_pulse)
            state_stat_instret <= state_stat_instret + 1;
         if (state <= `S_LAST_STATE)
            state_stat_cycles[state] <= state_stat_cycles[state] + 1;
         cache_state_stat_cycles[cache_state] <= cache_state_stat_cycles[cache_state] + 1;
         if (state_summary_interval != 0 &&
             state_stat_total_cycles + 1 >= state_summary_next) begin
            dump_state_summary;
            state_summary_next <= state_summary_next + state_summary_interval;
         end
      end

      if (cache_summary_enabled) begin
         if (hpm_cache_read_pulse)
            cache_stat_reads <= cache_stat_reads + 1;
         if (hpm_cache_write_pulse)
            cache_stat_writes <= cache_stat_writes + 1;
         if (hpm_cache_hit_pulse)
            cache_stat_hits <= cache_stat_hits + 1;
         if (hpm_cache_miss_pulse) begin
            cache_stat_misses <= cache_stat_misses + 1;
            if (cache_lookup_dirty)
               cache_stat_dirty_misses <= cache_stat_dirty_misses + 1;
            if (cache_stat_misses[12:0] == 13'h1fff) begin
               $display("%05d CACHE SUMMARY reads=%0d writes=%0d hits=%0d misses=%0d dirty_misses=%0d fill_lines=%0d wb_lines=%0d",
                        $time,
                        cache_stat_reads + (hpm_cache_read_pulse ? 64'd1 : 64'd0),
                        cache_stat_writes + (hpm_cache_write_pulse ? 64'd1 : 64'd0),
                        cache_stat_hits + (hpm_cache_hit_pulse ? 64'd1 : 64'd0),
                        cache_stat_misses + 64'd1,
                        cache_stat_dirty_misses + (cache_lookup_dirty ? 64'd1 : 64'd0),
                        cache_stat_fill_lines + (hpm_cache_fill_line_pulse ? 64'd1 : 64'd0),
                        cache_stat_wb_lines + (cache_wb_line_pulse ? 64'd1 : 64'd0));
            end
         end
         if (hpm_cache_fill_line_pulse)
            cache_stat_fill_lines <= cache_stat_fill_lines + 1;
         if (cache_wb_line_pulse)
            cache_stat_wb_lines <= cache_stat_wb_lines + 1;
      end

      if (hpm_tlb_lookup_pulse)
         tlb_stat_lookups <= tlb_stat_lookups + 1;
      if (hpm_tlb_hit_pulse)
         tlb_stat_hits <= tlb_stat_hits + 1;
      if (hpm_tlb_miss_pulse)
         tlb_stat_misses <= tlb_stat_misses + 1;
      if (hpm_tlb_hit_4k_pulse)
         tlb_stat_hits_4k <= tlb_stat_hits_4k + 1;
      if (hpm_tlb_hit_2m_pulse)
         tlb_stat_hits_2m <= tlb_stat_hits_2m + 1;
      if (hpm_tlb_insert_4k_pulse)
         tlb_stat_inserts_4k <= tlb_stat_inserts_4k + 1;
      if (hpm_tlb_insert_2m_pulse)
         tlb_stat_inserts_2m <= tlb_stat_inserts_2m + 1;
      if (hpm_tlb_evict_4k_pulse)
         tlb_stat_evicts_4k <= tlb_stat_evicts_4k + 1;
      if (hpm_tlb_evict_2m_pulse)
         tlb_stat_evicts_2m <= tlb_stat_evicts_2m + 1;
      if (hpm_tlb_uncached_1g_pulse)
         tlb_stat_uncached_1g <= tlb_stat_uncached_1g + 1;
      if (hpm_tlb_uncached_napot_pulse)
         tlb_stat_uncached_napot <= tlb_stat_uncached_napot + 1;

      if (cache_trace_enabled) begin
         if (hpm_cache_miss_pulse) begin
            $display("%05d CACHE MISS  op=%0d addr=%016h bank=%0d next_bank=%0d same_line=%0d dirty=%0d victim=%016h",
                     $time,
                     cache_req_write,
                     cache_addr,
                     cache_req_bank,
                     cache_req_next_bank,
                     cache_req_same_line,
                     cache_lookup_dirty,
                     {33'd0, cache_victim_ptag, cache_victim_idx[5:0], 6'd0});
         end
         if (hpm_cache_fill_beat_pulse) begin
            $display("%05d CACHE FILLD beat=%0d addr=%016h data=%016h",
                     $time,
                     cache_fill_beat,
                     cache_fill_base + (64'd8 * cache_fill_beat),
                     cache_fill_data);
         end
         if (hpm_cache_fill_line_pulse) begin
            $display("%05d CACHE FILLDONE addr=%016h write=%0d",
                     $time,
                     cache_fill_base,
                     cache_req_write);
         end
      end
   end
`endif

   // ----- Memory-clocked refill/writeback engine -----
   // L1 tag/data hits stay in the core clock domain.  Only slow-path
   // non-BRAM line fills, dirty writebacks, and direct PTW reads cross to the
   // memory clock domain.
   smolrv64_mem_engine mem_engine_inst (
      .core_clock          (clock),
      .mem_clock           (mem_clock),
      .reset               (core_reset_now),
      .idle                (mem_engine_idle),

      .fill_req_valid      (mem_fill_req_valid),
      .fill_req_ready      (mem_fill_req_ready),
      .fill_req_line_addr  (mem_fill_req_line_addr),
      .fill_rsp_valid      (mem_fill_rsp_valid),
      .fill_rsp_ready      (mem_fill_rsp_ready),
      .fill_rsp_data       (mem_fill_rsp_data),

      .wb_req_valid        (mem_wb_req_valid),
      .wb_req_ready        (mem_wb_req_ready),
      .wb_req_line_addr    (mem_wb_req_line_addr),
      .wb_req_line_data    (mem_wb_req_line_data),
      .wb_rsp_valid        (mem_wb_rsp_valid),
      .wb_rsp_ready        (mem_wb_rsp_ready),

      .read_req_valid      (mem_read_req_valid),
      .read_req_ready      (mem_read_req_ready),
      .read_req_addr       (mem_read_req_addr),
      .read_rsp_valid      (mem_read_rsp_valid),
      .read_rsp_ready      (mem_read_rsp_ready),
      .read_rsp_data       (mem_read_rsp_data),

      .m_axi_awid          (m_axi_awid),
      .m_axi_awaddr        (m_axi_awaddr),
      .m_axi_awlen         (m_axi_awlen),
      .m_axi_awsize        (m_axi_awsize),
      .m_axi_awburst       (m_axi_awburst),
      .m_axi_awlock        (m_axi_awlock),
      .m_axi_awcache       (m_axi_awcache),
      .m_axi_awprot        (m_axi_awprot),
      .m_axi_awqos         (m_axi_awqos),
      .m_axi_awvalid       (m_axi_awvalid),
      .m_axi_awready       (m_axi_awready),
      .m_axi_wdata         (m_axi_wdata),
      .m_axi_wstrb         (m_axi_wstrb),
      .m_axi_wlast         (m_axi_wlast),
      .m_axi_wvalid        (m_axi_wvalid),
      .m_axi_wready        (m_axi_wready),
      .m_axi_bid           (m_axi_bid),
      .m_axi_bresp         (m_axi_bresp),
      .m_axi_bvalid        (m_axi_bvalid),
      .m_axi_bready        (m_axi_bready),
      .m_axi_arid          (m_axi_arid),
      .m_axi_araddr        (m_axi_araddr),
      .m_axi_arlen         (m_axi_arlen),
      .m_axi_arsize        (m_axi_arsize),
      .m_axi_arburst       (m_axi_arburst),
      .m_axi_arlock        (m_axi_arlock),
      .m_axi_arcache       (m_axi_arcache),
      .m_axi_arprot        (m_axi_arprot),
      .m_axi_arqos         (m_axi_arqos),
      .m_axi_arvalid       (m_axi_arvalid),
      .m_axi_arready       (m_axi_arready),
      .m_axi_rid           (m_axi_rid),
      .m_axi_rdata         (m_axi_rdata),
      .m_axi_rresp         (m_axi_rresp),
      .m_axi_rlast         (m_axi_rlast),
      .m_axi_rvalid        (m_axi_rvalid),
      .m_axi_rready        (m_axi_rready)
   );

endmodule


module smolrv64_async_fifo #(
   parameter WIDTH = 64,
   parameter ADDR_BITS = 2,
   // "auto" (Vivado picks), "block" (force BRAM — better timing for
   // CDC paths under congestion), "distributed" (SLICEM LUTRAM)
   parameter MEMORY_TYPE = "auto"
) (
   input  wire             wr_clock,
   input  wire             rd_clock,
   input  wire             reset,
   input  wire             wr_valid,
   output wire             wr_ready,
   input  wire [WIDTH-1:0] wr_data,
   output wire             rd_valid,
   input  wire             rd_ready,
   output wire [WIDTH-1:0] rd_data
);
`ifdef SYNTHESIS
   wire full;
   wire empty;

   assign wr_ready = !full;
   assign rd_valid = !empty;

   xpm_fifo_async #(
      .CDC_SYNC_STAGES      ( 2 ),
      .DOUT_RESET_VALUE     ( "0" ),
      .ECC_MODE             ( "no_ecc" ),
      .FIFO_MEMORY_TYPE     ( MEMORY_TYPE ),
      .FIFO_READ_LATENCY    ( 0 ),
      .FIFO_WRITE_DEPTH     ( 1 << ADDR_BITS ),
      .FULL_RESET_VALUE     ( 0 ),
      .PROG_EMPTY_THRESH    ( 3 ),
      .PROG_FULL_THRESH     ( (1 << ADDR_BITS) - 2 ),
      .RD_DATA_COUNT_WIDTH  ( ADDR_BITS + 1 ),
      .READ_DATA_WIDTH      ( WIDTH ),
      .READ_MODE            ( "fwft" ),
      .RELATED_CLOCKS       ( 1 ),
      .SIM_ASSERT_CHK       ( 0 ),
      .USE_ADV_FEATURES     ( "0000" ),
      .WAKEUP_TIME          ( 0 ),
      .WRITE_DATA_WIDTH     ( WIDTH ),
      .WR_DATA_COUNT_WIDTH  ( ADDR_BITS + 1 )
   ) xpm_fifo_async_inst (
      .almost_empty  ( ),
      .almost_full   ( ),
      .data_valid    ( ),
      .dbiterr       ( ),
      .dout          ( rd_data ),
      .empty         ( empty ),
      .full          ( full ),
      .overflow      ( ),
      .prog_empty    ( ),
      .prog_full     ( ),
      .rd_data_count ( ),
      .rd_rst_busy   ( ),
      .sbiterr       ( ),
      .underflow     ( ),
      .wr_ack        ( ),
      .wr_data_count ( ),
      .wr_rst_busy   ( ),
      .din           ( wr_data ),
      .injectdbiterr ( 1'b0 ),
      .injectsbiterr ( 1'b0 ),
      .rd_clk        ( rd_clock ),
      .rd_en         ( rd_valid && rd_ready ),
      .rst           ( reset ),
      .sleep         ( 1'b0 ),
      .wr_clk        ( wr_clock ),
      .wr_en         ( wr_valid && wr_ready )
   );
`else
   localparam DEPTH = 1 << ADDR_BITS;
   reg [WIDTH-1:0] fifo_mem [0:DEPTH-1];
   reg [ADDR_BITS-1:0] wr_ptr = 0;
   reg [ADDR_BITS-1:0] rd_ptr = 0;
   reg [ADDR_BITS:0] count = 0;
   wire wr_fire = wr_valid && wr_ready;
   wire rd_fire = rd_valid && rd_ready;

   assign wr_ready = count != {1'b1, {ADDR_BITS{1'b0}}};
   assign rd_valid = count != 0;
   assign rd_data = fifo_mem[rd_ptr];

   always @(posedge wr_clock) begin
      if (reset) begin
         wr_ptr <= 0;
         rd_ptr <= 0;
         count <= 0;
      end else begin
         if (wr_fire) begin
            fifo_mem[wr_ptr] <= wr_data;
            wr_ptr <= wr_ptr + 1'b1;
         end
         if (rd_fire)
            rd_ptr <= rd_ptr + 1'b1;
         case ({wr_fire, rd_fire})
           2'b10: count <= count + 1'b1;
           2'b01: count <= count - 1'b1;
           default: count <= count;
         endcase
      end
   end
`endif
endmodule


module smolrv64_mem_engine(
   input  wire        core_clock,
   input  wire        mem_clock,
   input  wire        reset,
   output wire        idle,

   input  wire        fill_req_valid,
   output wire        fill_req_ready,
   input  wire [24:0] fill_req_line_addr,
   output wire        fill_rsp_valid,
   input  wire        fill_rsp_ready,
   output wire [511:0] fill_rsp_data,

   input  wire        wb_req_valid,
   output wire        wb_req_ready,
   input  wire [24:0] wb_req_line_addr,
   input  wire [511:0] wb_req_line_data,
   output wire        wb_rsp_valid,
   input  wire        wb_rsp_ready,

   input  wire        read_req_valid,
   output wire        read_req_ready,
   input  wire [27:0] read_req_addr,
   output wire        read_rsp_valid,
   input  wire        read_rsp_ready,
   output wire [63:0] read_rsp_data,

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
   output wire        m_axi_rready
);
   wire        fill_cmd_valid;
   reg         fill_cmd_ready = 0;
   wire [24:0] fill_cmd_line_addr;
   reg         fill_rsp_wr_valid = 0;
   wire        fill_rsp_wr_ready;
   reg  [511:0] fill_rsp_wr_data = 0;

   wire        wb_cmd_valid;
   reg         wb_cmd_ready = 0;
   wire [536:0] wb_cmd_data;
   reg         wb_rsp_wr_valid = 0;
   wire        wb_rsp_wr_ready;

   wire        read_cmd_valid;
   reg         read_cmd_ready = 0;
   wire [27:0] read_cmd_addr;
   reg         read_rsp_wr_valid = 0;
   wire        read_rsp_wr_ready;
   reg  [63:0] read_rsp_wr_data = 0;

   // The core only raises reset for this engine after the memory side and all
   // queues are idle.  Configuration-time initial values are enough here; avoid
   // feeding the complex core-reset-home expression into XPM FIFO reset logic in
   // the 333 MHz memory clock domain.
   // All mem_engine CDC FIFOs forced to BRAM (MEMORY_TYPE="block"): same
   // rationale as the MMIO bridge FIFOs in rk_xcku5p.v — distributed RAM
   // implementation in SLICEM cells gets placed across multiple clock
   // regions, the cross-region skew on doutb_reg paths kills timing.
   // BRAM hard blocks have constrained, predictable placement.
   smolrv64_async_fifo #(.WIDTH(25), .ADDR_BITS(4), .MEMORY_TYPE("block")) fill_req_fifo (
      .wr_clock(core_clock), .rd_clock(mem_clock), .reset(1'b0),
      .wr_valid(fill_req_valid), .wr_ready(fill_req_ready), .wr_data(fill_req_line_addr),
      .rd_valid(fill_cmd_valid), .rd_ready(fill_cmd_ready), .rd_data(fill_cmd_line_addr)
   );
   smolrv64_async_fifo #(.WIDTH(512), .ADDR_BITS(4), .MEMORY_TYPE("block")) fill_rsp_fifo (
      .wr_clock(mem_clock), .rd_clock(core_clock), .reset(1'b0),
      .wr_valid(fill_rsp_wr_valid), .wr_ready(fill_rsp_wr_ready), .wr_data(fill_rsp_wr_data),
      .rd_valid(fill_rsp_valid), .rd_ready(fill_rsp_ready), .rd_data(fill_rsp_data)
   );
   smolrv64_async_fifo #(.WIDTH(537), .ADDR_BITS(4), .MEMORY_TYPE("block")) wb_req_fifo (
      .wr_clock(core_clock), .rd_clock(mem_clock), .reset(1'b0),
      .wr_valid(wb_req_valid), .wr_ready(wb_req_ready),
      .wr_data({wb_req_line_addr, wb_req_line_data}),
      .rd_valid(wb_cmd_valid), .rd_ready(wb_cmd_ready), .rd_data(wb_cmd_data)
   );
   smolrv64_async_fifo #(.WIDTH(1), .ADDR_BITS(4), .MEMORY_TYPE("block")) wb_rsp_fifo (
      .wr_clock(mem_clock), .rd_clock(core_clock), .reset(1'b0),
      .wr_valid(wb_rsp_wr_valid), .wr_ready(wb_rsp_wr_ready), .wr_data(1'b1),
      .rd_valid(wb_rsp_valid), .rd_ready(wb_rsp_ready), .rd_data()
   );
   smolrv64_async_fifo #(.WIDTH(28), .ADDR_BITS(4), .MEMORY_TYPE("block")) read_req_fifo (
      .wr_clock(core_clock), .rd_clock(mem_clock), .reset(1'b0),
      .wr_valid(read_req_valid), .wr_ready(read_req_ready), .wr_data(read_req_addr),
      .rd_valid(read_cmd_valid), .rd_ready(read_cmd_ready), .rd_data(read_cmd_addr)
   );
   smolrv64_async_fifo #(.WIDTH(64), .ADDR_BITS(4), .MEMORY_TYPE("block")) read_rsp_fifo (
      .wr_clock(mem_clock), .rd_clock(core_clock), .reset(1'b0),
      .wr_valid(read_rsp_wr_valid), .wr_ready(read_rsp_wr_ready), .wr_data(read_rsp_wr_data),
      .rd_valid(read_rsp_valid), .rd_ready(read_rsp_ready), .rd_data(read_rsp_data)
   );

   localparam [2:0] MEM_IDLE       = 3'd0,
                    MEM_READ_REQ   = 3'd1,
                    MEM_READ_WAIT  = 3'd2,
                    MEM_READ_RESP  = 3'd3,
                    MEM_FILL_REQ   = 3'd4,
                    MEM_FILL_WAIT  = 3'd5,
                    MEM_FILL_RESP  = 3'd6,
                    MEM_WB_REQ     = 3'd7;
   localparam [1:0] MEM_WB_WAIT = 2'd0,
                    MEM_WB_RESP = 2'd1;

   reg [2:0] mem_state = MEM_IDLE;
   reg [1:0] wb_substate = MEM_WB_WAIT;
   reg [27:0] op_addr = 0;
   reg [24:0] op_line_addr = 0;
   reg [511:0] op_line_data = 0;
   reg [2:0] op_beat = 0;
   reg [511:0] fill_line_data = 0;

   reg        ar_busy = 0;
   reg        r_busy  = 0;
   reg [27:0] ar_addr_r = 0;
   reg        aw_busy = 0;
   reg        w_busy  = 0;
   reg        b_busy  = 0;
   reg [27:0] aw_addr_r = 0;
   reg [63:0] w_data_r = 0;
   reg [7:0]  w_strb_r = 0;

   reg mem_busy = 0;
   reg mem_busy_meta = 0;
   reg mem_busy_sync = 0;
   assign idle = !mem_busy_sync &&
                 fill_req_ready && !fill_rsp_valid &&
                 wb_req_ready && !wb_rsp_valid &&
                 read_req_ready && !read_rsp_valid;

   always @(posedge core_clock) begin
      if (reset) begin
         mem_busy_meta <= 0;
         mem_busy_sync <= 0;
      end else begin
         mem_busy_meta <= mem_busy;
         mem_busy_sync <= mem_busy_meta;
      end
   end

   always @(posedge mem_clock) begin
      if (ar_busy && m_axi_arready)
         ar_busy <= 0;
      if (aw_busy && m_axi_awready)
         aw_busy <= 0;
      if (w_busy && m_axi_wready)
         w_busy <= 0;

      case (mem_state)
        MEM_IDLE: begin
           mem_busy <= 0;
           wb_substate <= MEM_WB_WAIT;
           fill_cmd_ready <= 0;
           wb_cmd_ready <= 0;
           read_cmd_ready <= 0;
           if (read_cmd_valid) begin
              read_cmd_ready <= 1;
              if (read_cmd_ready) begin
                 read_cmd_ready <= 0;
                 op_addr <= read_cmd_addr;
                 mem_busy <= 1;
                 mem_state <= MEM_READ_REQ;
              end
           end else if (fill_cmd_valid) begin
              fill_cmd_ready <= 1;
              if (fill_cmd_ready) begin
                 fill_cmd_ready <= 0;
                 op_line_addr <= fill_cmd_line_addr;
                 op_beat <= 0;
                 fill_line_data <= 0;
                 mem_busy <= 1;
                 mem_state <= MEM_FILL_REQ;
              end
           end else if (wb_cmd_valid) begin
              wb_cmd_ready <= 1;
              if (wb_cmd_ready) begin
                 wb_cmd_ready <= 0;
                 op_line_addr <= wb_cmd_data[536:512];
                 op_line_data <= wb_cmd_data[511:0];
                 op_beat <= 0;
                 mem_busy <= 1;
                 mem_state <= MEM_WB_REQ;
              end
           end
        end

        MEM_READ_REQ: begin
           if (!ar_busy && !r_busy) begin
              ar_addr_r <= op_addr;
              ar_busy <= 1;
              r_busy <= 1;
              mem_state <= MEM_READ_WAIT;
           end
        end

        MEM_READ_WAIT: begin
           if (r_busy && m_axi_rvalid) begin
              r_busy <= 0;
              read_rsp_wr_data <= m_axi_rdata;
              mem_state <= MEM_READ_RESP;
           end
        end

        MEM_READ_RESP: begin
           read_rsp_wr_valid <= 1;
           if (read_rsp_wr_valid && read_rsp_wr_ready) begin
              read_rsp_wr_valid <= 0;
              mem_state <= MEM_IDLE;
           end
        end

        MEM_FILL_REQ: begin
           if (!ar_busy && !r_busy) begin
              ar_addr_r <= {op_line_addr, op_beat};
              ar_busy <= 1;
              r_busy <= 1;
              mem_state <= MEM_FILL_WAIT;
           end
        end

        MEM_FILL_WAIT: begin
           if (r_busy && m_axi_rvalid) begin
              r_busy <= 0;
              fill_line_data[op_beat * 64 +: 64] <= m_axi_rdata;
              if (op_beat == 3'd7) begin
                 mem_state <= MEM_FILL_RESP;
              end else begin
                 op_beat <= op_beat + 1'b1;
                 mem_state <= MEM_FILL_REQ;
              end
           end
        end

        MEM_FILL_RESP: begin
           fill_rsp_wr_data <= fill_line_data;
           fill_rsp_wr_valid <= 1;
           if (fill_rsp_wr_valid && fill_rsp_wr_ready) begin
              fill_rsp_wr_valid <= 0;
              mem_state <= MEM_IDLE;
           end
        end

        MEM_WB_REQ: begin
           case (wb_substate)
             MEM_WB_WAIT: begin
                if (!aw_busy && !w_busy && !b_busy) begin
                   aw_addr_r <= {op_line_addr, op_beat};
                   w_data_r <= op_line_data[op_beat * 64 +: 64];
                   w_strb_r <= 8'hff;
                   aw_busy <= 1;
                   w_busy <= 1;
                   b_busy <= 1;
                   wb_substate <= MEM_WB_RESP;
                end
             end
             MEM_WB_RESP: begin
                if (!b_busy && op_beat == 3'd7) begin
                   wb_rsp_wr_valid <= 1;
                   if (wb_rsp_wr_valid && wb_rsp_wr_ready) begin
                      wb_rsp_wr_valid <= 0;
                      mem_state <= MEM_IDLE;
                   end
                end else if (b_busy && m_axi_bvalid) begin
                   b_busy <= 0;
                   if (op_beat == 3'd7) begin
                      wb_substate <= MEM_WB_RESP;
                   end else begin
                      op_beat <= op_beat + 1'b1;
                      wb_substate <= MEM_WB_WAIT;
                   end
                end
             end
           endcase
        end
      endcase

   end

   assign m_axi_arvalid = ar_busy;
   assign m_axi_araddr  = {ar_addr_r, 3'b000};
   assign m_axi_arlen   = 8'd0;
   assign m_axi_arsize  = 3'b011;
   assign m_axi_arburst = 2'b01;
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


module smolrv64_frontend #(
   parameter EPOCH_BITS = 2,
   parameter TLB_ASID_BITS = 10,
   parameter CACHE_PERM_BITS = 5,
   parameter VHPR_EPOCH_BITS = 2
) (
   input  wire                  clock,
   input  wire                  reset,
   input  wire                  flush,
   input  wire                  fill,
   input  wire [63:0]           fill_base_va,
   input  wire [ 1:0]           fill_prv,
   input  wire [TLB_ASID_BITS-1:0] fill_asid,
   input  wire [127:0]          fill_data,

   input  wire                  cmd_valid,
   input  wire [63:0]           cmd_pc,
   input  wire [ 1:0]           cmd_prv,
   input  wire [TLB_ASID_BITS-1:0] cmd_asid,
   input  wire [EPOCH_BITS-1:0] cmd_epoch,

   output wire                  rsp_hit,
   output wire                  rsp_addr_hit,
   output wire                  rsp_full_insn_hit,
   output wire [31:0]           rsp_insn,
   output wire [ 3:0]           rsp_offset,
   output wire [63:0]           rsp_predicted_next_pc,
   output wire [ 1:0]           rsp_prediction_kind,
   output wire [EPOCH_BITS-1:0] rsp_active_epoch,
   output wire [63:0]           rsp_fill_base_va,
   output wire                  rsp_fill_page_ok,

   input  wire [`CACHE_INDEX_BITS-1:0] icache_way0_rd_idx,
   input  wire [`CACHE_INDEX_BITS-1:0] icache_way1_rd_idx,
   input  wire [`CACHE_INDEX_BITS-1:0] icache_way0_bank0_rd_idx,
   input  wire [`CACHE_INDEX_BITS-1:0] icache_way1_bank0_rd_idx,
   input  wire [`CACHE_INDEX_BITS-1:0] icache_way0_next_rd_idx,
   input  wire [`CACHE_INDEX_BITS-1:0] icache_way1_next_rd_idx,
   input  wire                         icache_way0_tag_wr_en,
   input  wire                         icache_way1_tag_wr_en,
   input  wire [`CACHE_INDEX_BITS-1:0] icache_tag_wr_idx,
   input  wire [`CACHE_META_BITS-1:0]  icache_tag_wr_data,
   input  wire [7:0]                   icache_bank_wr_en,
   input  wire [`CACHE_INDEX_BITS-1:0] icache_bank_wr_idx,
   input  wire                         icache_bank_wr_way,
   input  wire [63:0]                  icache_bank_wr_data,
   output wire [`CACHE_META_BITS-1:0]  icache_way0_tag_rd_data,
   output wire [`CACHE_META_BITS-1:0]  icache_way1_tag_rd_data,
   output wire [`CACHE_META_BITS-1:0]  icache_way0_tag_next_rd_data,
   output wire [`CACHE_META_BITS-1:0]  icache_way1_tag_next_rd_data,
   output wire [63:0]                  icache_way0_bank0_rd_data,
   output wire [63:0]                  icache_way0_bank1_rd_data,
   output wire [63:0]                  icache_way0_bank2_rd_data,
   output wire [63:0]                  icache_way0_bank3_rd_data,
   output wire [63:0]                  icache_way0_bank4_rd_data,
   output wire [63:0]                  icache_way0_bank5_rd_data,
   output wire [63:0]                  icache_way0_bank6_rd_data,
   output wire [63:0]                  icache_way0_bank7_rd_data,
   output wire [63:0]                  icache_way1_bank0_rd_data,
   output wire [63:0]                  icache_way1_bank1_rd_data,
   output wire [63:0]                  icache_way1_bank2_rd_data,
   output wire [63:0]                  icache_way1_bank3_rd_data,
   output wire [63:0]                  icache_way1_bank4_rd_data,
   output wire [63:0]                  icache_way1_bank5_rd_data,
   output wire [63:0]                  icache_way1_bank6_rd_data,
   output wire [63:0]                  icache_way1_bank7_rd_data
);
   localparam [1:0] PRED_FALLTHROUGH = 2'd0;
   localparam [1:0] PRED_DIRECT      = 2'd1;
   localparam [1:0] PRED_BRANCH      = 2'd2;

   reg          buf_valid = 0;
   reg  [63:0]  buf_base_va = 0;
   reg  [59:0]  buf_next_va_hi = 0;
   reg  [ 1:0]  buf_prv = 0;
   reg  [TLB_ASID_BITS-1:0] buf_asid = 0;
   reg  [127:0] buf_data = 0;

   function [31:0] pick_insn;
      input [127:0] data;
      input [3:0]   byte_offset;
      begin
         case (byte_offset[3:1])
           3'd0:    pick_insn = data[31:0];
           3'd1:    pick_insn = data[47:16];
           3'd2:    pick_insn = data[63:32];
           3'd3:    pick_insn = data[79:48];
           3'd4:    pick_insn = data[95:64];
           3'd5:    pick_insn = data[111:80];
           3'd6:    pick_insn = data[127:96];
           default: pick_insn = {16'd0, data[127:112]};
         endcase
      end
   endfunction

   function [63:0] fallthrough_pc;
      input [63:0] pc;
      input [31:0] fetch_insn;
      begin
         fallthrough_pc = pc + (fetch_insn[1:0] == 2'b11 ? 64'd4 : 64'd2);
      end
   endfunction

   function [63:0] jal_target;
      input [63:0] pc;
      input [31:0] fetch_insn;
      begin
         jal_target = pc + {{44{fetch_insn[31]}}, fetch_insn[19:12],
                            fetch_insn[20], fetch_insn[30:21], 1'b0};
      end
   endfunction

   function [63:0] branch_target;
      input [63:0] pc;
      input [31:0] fetch_insn;
      begin
         branch_target = pc + {{52{fetch_insn[31]}}, fetch_insn[7],
                               fetch_insn[30:25], fetch_insn[11:8], 1'b0};
      end
   endfunction

   function [63:0] c_j_target;
      input [63:0] pc;
      input [31:0] fetch_insn;
      begin
         c_j_target = pc + {{53{fetch_insn[12]}}, fetch_insn[8],
                            fetch_insn[10:9], fetch_insn[6], fetch_insn[7],
                            fetch_insn[2], fetch_insn[11],
                            fetch_insn[5:3], 1'b0};
      end
   endfunction

   function [63:0] c_branch_target;
      input [63:0] pc;
      input [31:0] fetch_insn;
      begin
         c_branch_target = pc + {{56{fetch_insn[12]}}, fetch_insn[6:5],
                                 fetch_insn[2], fetch_insn[11:10],
                                 fetch_insn[4:3], 1'b0};
      end
   endfunction

   function branch_is_backward;
      input [31:0] fetch_insn;
      begin
         branch_is_backward = fetch_insn[31];
      end
   endfunction

   function c_branch_is_backward;
      input [31:0] fetch_insn;
      begin
         c_branch_is_backward = fetch_insn[12];
      end
   endfunction

   function [63:0] predict_next_pc;
      input [63:0] pc;
      input [31:0] fetch_insn;
      begin
         if ((fetch_insn & 32'h0000007f) == 32'h0000006f) begin
            predict_next_pc = jal_target(pc, fetch_insn);
         end else if ((fetch_insn & 32'h0000007f) == 32'h00000063 &&
                      branch_is_backward(fetch_insn)) begin
            predict_next_pc = branch_target(pc, fetch_insn);
         end else if ((fetch_insn & 32'he003) == 32'ha001) begin
            predict_next_pc = c_j_target(pc, fetch_insn);
         end else if (((fetch_insn & 32'he003) == 32'hc001 ||
                       (fetch_insn & 32'he003) == 32'he001) &&
                      c_branch_is_backward(fetch_insn)) begin
            predict_next_pc = c_branch_target(pc, fetch_insn);
         end else begin
            predict_next_pc = fallthrough_pc(pc, fetch_insn);
         end
      end
   endfunction

   function [1:0] predict_kind;
      input [31:0] fetch_insn;
      begin
         if ((fetch_insn & 32'h0000007f) == 32'h0000006f ||
             (fetch_insn & 32'he003) == 32'ha001) begin
            predict_kind = PRED_DIRECT;
         end else if (((fetch_insn & 32'h0000007f) == 32'h00000063 &&
                       branch_is_backward(fetch_insn)) ||
                      (((fetch_insn & 32'he003) == 32'hc001 ||
                        (fetch_insn & 32'he003) == 32'he001) &&
                       c_branch_is_backward(fetch_insn))) begin
            predict_kind = PRED_BRANCH;
         end else begin
            predict_kind = PRED_FALLTHROUGH;
         end
      end
   endfunction

   wire        context_hit = cmd_valid && buf_valid &&
                              buf_prv == cmd_prv && buf_asid == cmd_asid;
   wire        addr_same_hi = cmd_pc[63:4] == buf_base_va[63:4];
   wire        addr_next_hi = cmd_pc[63:4] == buf_next_va_hi;

   assign rsp_addr_hit = context_hit && !cmd_pc[0] &&
                     ((!buf_base_va[3] && addr_same_hi) ||
                      ( buf_base_va[3] &&
                        ((addr_same_hi &&  cmd_pc[3]) ||
                         (addr_next_hi && !cmd_pc[3]))));
   assign rsp_offset = buf_base_va[3] ?
                   (addr_same_hi ? {1'b0, cmd_pc[2:0]} :
                                   {1'b1, cmd_pc[2:0]}) :
                   cmd_pc[3:0];
   assign rsp_insn = pick_insn(buf_data, rsp_offset);
   assign rsp_full_insn_hit = rsp_insn[1:0] != 2'b11 || rsp_offset <= 4'd12;
   assign rsp_hit = rsp_addr_hit && rsp_full_insn_hit;
   assign rsp_predicted_next_pc = predict_next_pc(cmd_pc, rsp_insn);
   assign rsp_prediction_kind = predict_kind(rsp_insn);
   assign rsp_active_epoch = cmd_epoch;
   assign rsp_fill_base_va = {cmd_pc[63:3], 3'b000};
   assign rsp_fill_page_ok = rsp_fill_base_va[11:0] <= 12'hff0;
   wire [63:0] icache_way0_bank_rd_data [0:7];
   wire [63:0] icache_way1_bank_rd_data [0:7];

   assign icache_way0_bank0_rd_data = icache_way0_bank_rd_data[0];
   assign icache_way0_bank1_rd_data = icache_way0_bank_rd_data[1];
   assign icache_way0_bank2_rd_data = icache_way0_bank_rd_data[2];
   assign icache_way0_bank3_rd_data = icache_way0_bank_rd_data[3];
   assign icache_way0_bank4_rd_data = icache_way0_bank_rd_data[4];
   assign icache_way0_bank5_rd_data = icache_way0_bank_rd_data[5];
   assign icache_way0_bank6_rd_data = icache_way0_bank_rd_data[6];
   assign icache_way0_bank7_rd_data = icache_way0_bank_rd_data[7];
   assign icache_way1_bank0_rd_data = icache_way1_bank_rd_data[0];
   assign icache_way1_bank1_rd_data = icache_way1_bank_rd_data[1];
   assign icache_way1_bank2_rd_data = icache_way1_bank_rd_data[2];
   assign icache_way1_bank3_rd_data = icache_way1_bank_rd_data[3];
   assign icache_way1_bank4_rd_data = icache_way1_bank_rd_data[4];
   assign icache_way1_bank5_rd_data = icache_way1_bank_rd_data[5];
   assign icache_way1_bank6_rd_data = icache_way1_bank_rd_data[6];
   assign icache_way1_bank7_rd_data = icache_way1_bank_rd_data[7];

   always @(posedge clock) begin
      if (reset || flush) begin
         buf_valid <= 1'b0;
      end else if (fill) begin
         buf_valid      <= 1'b1;
         buf_base_va    <= fill_base_va;
         buf_next_va_hi <= fill_base_va[63:4] + 60'd1;
         buf_prv        <= fill_prv;
         buf_asid       <= fill_asid;
         buf_data       <= fill_data;
      end
   end

   smolrv64_sdpram #(
      .ADDR_WIDTH(`CACHE_INDEX_BITS),
      .DATA_WIDTH(`CACHE_META_BITS),
      .READ_LATENCY(2)
   ) icache_way0_tag_ram (
      .clock   ( clock ),
      .rd_addr ( icache_way0_rd_idx ),
      .rd_data ( icache_way0_tag_rd_data ),
      .wr_en   ( icache_way0_tag_wr_en ),
      .wr_addr ( icache_tag_wr_idx ),
      .wr_data ( icache_tag_wr_data )
   );

   smolrv64_sdpram #(
      .ADDR_WIDTH(`CACHE_INDEX_BITS),
      .DATA_WIDTH(`CACHE_META_BITS),
      .READ_LATENCY(2)
   ) icache_way1_tag_ram (
      .clock   ( clock ),
      .rd_addr ( icache_way1_rd_idx ),
      .rd_data ( icache_way1_tag_rd_data ),
      .wr_en   ( icache_way1_tag_wr_en ),
      .wr_addr ( icache_tag_wr_idx ),
      .wr_data ( icache_tag_wr_data )
   );

   smolrv64_sdpram #(
      .ADDR_WIDTH(`CACHE_INDEX_BITS),
      .DATA_WIDTH(`CACHE_META_BITS),
      .READ_LATENCY(2)
   ) icache_way0_tag_next_ram (
      .clock   ( clock ),
      .rd_addr ( icache_way0_next_rd_idx ),
      .rd_data ( icache_way0_tag_next_rd_data ),
      .wr_en   ( icache_way0_tag_wr_en ),
      .wr_addr ( icache_tag_wr_idx ),
      .wr_data ( icache_tag_wr_data )
   );

   smolrv64_sdpram #(
      .ADDR_WIDTH(`CACHE_INDEX_BITS),
      .DATA_WIDTH(`CACHE_META_BITS),
      .READ_LATENCY(2)
   ) icache_way1_tag_next_ram (
      .clock   ( clock ),
      .rd_addr ( icache_way1_next_rd_idx ),
      .rd_data ( icache_way1_tag_next_rd_data ),
      .wr_en   ( icache_way1_tag_wr_en ),
      .wr_addr ( icache_tag_wr_idx ),
      .wr_data ( icache_tag_wr_data )
   );

   genvar icache_bank_gen;
   generate
      for (icache_bank_gen = 0; icache_bank_gen < 8; icache_bank_gen = icache_bank_gen + 1) begin : frontend_icache_banks
         wire [63:0] way0_rd_data;
         wire [63:0] way1_rd_data;

         smolrv64_sdpram #(
            .ADDR_WIDTH(`CACHE_INDEX_BITS),
            .DATA_WIDTH(64),
            .READ_LATENCY(2)
         ) icache_way0_bank_ram (
            .clock   ( clock ),
            .rd_addr ( icache_bank_gen == 0 ? icache_way0_bank0_rd_idx : icache_way0_rd_idx ),
            .rd_data ( way0_rd_data ),
            .wr_en   ( icache_bank_wr_en[icache_bank_gen] && !icache_bank_wr_way ),
            .wr_addr ( icache_bank_wr_idx ),
            .wr_data ( icache_bank_wr_data )
         );

         smolrv64_sdpram #(
            .ADDR_WIDTH(`CACHE_INDEX_BITS),
            .DATA_WIDTH(64),
            .READ_LATENCY(2)
         ) icache_way1_bank_ram (
            .clock   ( clock ),
            .rd_addr ( icache_bank_gen == 0 ? icache_way1_bank0_rd_idx : icache_way1_rd_idx ),
            .rd_data ( way1_rd_data ),
            .wr_en   ( icache_bank_wr_en[icache_bank_gen] && icache_bank_wr_way ),
            .wr_addr ( icache_bank_wr_idx ),
            .wr_data ( icache_bank_wr_data )
         );

         assign icache_way0_bank_rd_data[icache_bank_gen] = way0_rd_data;
         assign icache_way1_bank_rd_data[icache_bank_gen] = way1_rd_data;
      end
   endgenerate
endmodule


module smolrv64_sdpram #(
   parameter ADDR_WIDTH = 14,
   parameter DATA_WIDTH = 64,
   parameter READ_LATENCY = 1
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
      .READ_LATENCY_B      ( READ_LATENCY ),
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
      .READ_LATENCY_B      ( READ_LATENCY ),
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
   reg [DATA_WIDTH-1:0] rd_data_rr = 0;
   integer ram_init_i;

   initial begin
      for (ram_init_i = 0; ram_init_i < (1 << ADDR_WIDTH); ram_init_i = ram_init_i + 1)
         ram[ram_init_i] = 0;
   end

   assign rd_data = READ_LATENCY == 1 ? rd_data_r : rd_data_rr;

   always @(posedge clock) begin
      rd_data_r <= ram[rd_addr];
      rd_data_rr <= rd_data_r;
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
`ifndef SYNTHESIS
      if ($value$plusargs("rf=%s", rf_path))
         $readmemh(rf_path, regfile, 0, 31);
      else
`endif
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
