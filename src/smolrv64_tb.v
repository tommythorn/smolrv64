`timescale 1ns/10ps
`default_nettype none

// Simulation testbench for the smolrv64 core: AXI memory model, clock/reset,
// image loading ($readmemh of +even/+odd), and the tty/halt plumbing. Compiled
// only for Verilator/sim builds (guarded by `ifdef SIMULATE); the Vivado flow
// does not include this file.

`ifdef SIMULATE
//`define DISASS 1
`ifndef AXI_MEM_SIZE_LG2
`define AXI_MEM_SIZE_LG2 27
`endif
`define AXI_MEM_SIZE (64'd1 << `AXI_MEM_SIZE_LG2)
`ifdef VERILATOR
module smolrv64_tb(input wire clock);
`else
module smolrv64_tb;
   reg        clock = 1; always #5 clock = !clock;
`endif
   wire       halted;

   // Host-stdin -> modeled UART RX. Under Icarus this links in tty_vpi.c
   // ($tty_read); under Verilator the equivalent DPI-C import (sim_main.cpp).
`ifdef RISCV_TESTS
   // Non-interactive test harness: no stdin polling (would grab the tty).
`elsif VERILATOR
   import "DPI-C" function int tty_read();
   `define TTY_READ tty_read()
`elsif VPI
   `define TTY_READ $tty_read
`endif

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
   reg [15:0]     axi_r_delay    = 16'd0;        // DDR read-latency countdown (per beat)
   reg [15:0]     axi_rd_lat_min = 16'd0;        // +axi_rd_lat_min=<cyc> (0 = ideal, default)
   reg [15:0]     axi_rd_lat_max = 16'd0;        // +axi_rd_lat_max=<cyc> (0 = ideal, default)
   reg [15:0]     axi_lat_span   = 16'd1;        // max-min+1 (recomputed in initial)
   reg [31:0]     axi_lat_rng    = 32'h2545F491; // xorshift32 state (+axi_lat_seed)
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

   // xorshift32 PRNG for reproducible, seedable read-latency jitter.
   function [31:0] axi_lat_next;
      input [31:0] x;
      begin
         x = x ^ (x << 13);
         x = x ^ (x >> 17);
         x = x ^ (x << 5);
         axi_lat_next = x;
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

      // Configurable sim DDR read latency (cycles per 64-bit AXI beat).
      // Default 0..0 = the original ideal 1-cycle memory (no behavior change).
      // Enable HW-like stalls with e.g. +axi_rd_lat_min=16 +axi_rd_lat_max=64.
      if ($value$plusargs("axi_rd_lat_min=%d", axi_rd_lat_min)) ;
      if ($value$plusargs("axi_rd_lat_max=%d", axi_rd_lat_max)) ;
      if (axi_rd_lat_max < axi_rd_lat_min) axi_rd_lat_max = axi_rd_lat_min;
      axi_lat_span = (axi_rd_lat_max - axi_rd_lat_min) + 16'd1;
      if ($value$plusargs("axi_lat_seed=%d", axi_lat_rng)) ;
      if (axi_lat_rng == 32'd0) axi_lat_rng = 32'h2545F491;
      if (axi_rd_lat_max != 16'd0)
         $display("sim DDR read latency: %0d..%0d cyc/beat, seed=%08x",
                  axi_rd_lat_min, axi_rd_lat_max, axi_lat_rng);
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
         axi_araddr_q  <= m_axi_araddr;
         axi_r_pending <= 1;
         // Variable DDR read latency the ideal 1-cycle sim memory lacks: hold
         // the response axi_r_delay cycles so the backend parks in its memory-
         // wait states and the frontend can build a real lead (HW-like timing).
         axi_r_delay   <= axi_rd_lat_min + (axi_lat_rng % axi_lat_span);
         axi_lat_rng   <= axi_lat_next(axi_lat_rng);
      end
      if (axi_r_pending && !m_axi_rvalid) begin
         if (axi_r_delay != 16'd0) begin
            axi_r_delay <= axi_r_delay - 16'd1;
         end else begin
            m_axi_rid    <= 3'b000;
            m_axi_rdata  <= axi_read64(axi_araddr_q);
            m_axi_rresp  <= 2'b00;
            m_axi_rlast  <= 1'b1;
            m_axi_rvalid <= 1;
            axi_r_pending <= 0;
         end
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
         axi_r_delay   <= 0;
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

`ifdef TTY_READ
      begin : rx_poll
         integer ch;
         ch = `TTY_READ;
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
