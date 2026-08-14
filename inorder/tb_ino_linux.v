`timescale 1ns/1ps
`default_nettype none

// Linux-boot harness for the in-order SoC. Resets DIRECTLY to OpenSBI
// (0x8000_0000) with a1 = the DTB pointer (+a1=, seeded into ino_regfile's x11),
// so the monitor is bypassed and the DUT starts where the reference model would.
// Console output comes out of ino_soc_top's UART $write.
//
//   +fw=<path> +dtb=<path> [+initrd=<path>]
//   [+dtb_off=<hex>] [+initrd_off=<hex>] [+a1=<hex>] [+cycles=N]   (0 = unbounded)
//
// VIRTIO IS TIED OFF: the tiny128 workload is initrd-based and its DTB carries no
// virtio node, so the region is never touched. A disk-backed workload needs the
// virtio-blk subsystem from probe/tb_cosim_linux.v ported in.
module tb;
   localparam [63:0] BASE = 64'h8000_0000;
`ifdef INO_MEM_SIZE_LG2
   localparam [63:0] DDR_BYTES = 64'd1 << `INO_MEM_SIZE_LG2;
`else
   localparam [63:0] DDR_BYTES = 64'd1 << 29;        // 512 MiB
`endif
   localparam [63:0] OFF_FW = 64'h000_0000, OFF_DTB_DEF = 64'h1ff0_0000,
                     OFF_INITRD_DEF = 64'h1f52_c000;
   reg [63:0] off_dtb, off_initrd;

   reg clk = 0; always #5 clk = ~clk;
   reg reset;
   wire        retire, dmem_wen;
   wire [63:0] dmem_waddr, dmem_wdata;  wire [7:0] dmem_wmask;
   wire        ddr_req, ddr_we;  wire [57:0] ddr_addr;  wire [511:0] ddr_wdata;
   reg  [511:0] ddr_rdata;  reg ddr_ack;
   wire        rx_ready;

   // Model the hardware's 3 Mbps serializer instead of an instant drain: a 10-bit
   // frame at 66.67 MHz is ~222 core cycles, so uart_tx_ready stays low that long
   // after each accepted byte. THR then stays full between characters and THRE
   // interrupts land while the kernel is BUSY -- the interrupt pacing the FPGA sees,
   // which a tied-high ready never exercises.
   localparam TX_DRAIN = 222;
   reg  [7:0] txcnt = 8'd0;
   wire       uart_tx_v;
   wire       uart_tx_rdy = (txcnt == 8'd0);
   always @(posedge clk)
      if (reset)                        txcnt <= 8'd0;
      else if (uart_tx_v & uart_tx_rdy) txcnt <= TX_DRAIN[7:0];
      else if (txcnt != 8'd0)           txcnt <= txcnt - 8'd1;

   ino_soc_top #(.RESET_PC(64'h8000_0000)) dut
     (.clk(clk), .reset(reset), .retire(retire),
      .dmem_wen(dmem_wen), .dmem_waddr(dmem_waddr), .dmem_wdata(dmem_wdata),
      .dmem_wmask(dmem_wmask),
      .ddr_req(ddr_req), .ddr_we(ddr_we), .ddr_addr(ddr_addr),
      .ddr_wdata(ddr_wdata), .ddr_rdata(ddr_rdata), .ddr_ack(ddr_ack),
      .uart_rx_we(1'b0), .uart_rx_data(8'd0), .uart_rx_ready(rx_ready),
      .uart_tx_valid(uart_tx_v), .uart_tx_ready(uart_tx_rdy),
      .virtio_addr(), .virtio_read(), .virtio_write(), .virtio_wdata(), .virtio_be(),
      .virtio_rdata(32'd0), .virtio_rvalid(1'b0), .virtio_irq(1'b0), .irq_dbg());

   // behavioral DDR (4-cycle line latency), modeled as a 512-bit LINE array: the
   // ddr_* port is 64-byte lines, so the element count stays under the array-dimension
   // limit even at 2 GiB (a flat byte array overflows it).
   localparam [63:0] NLINES = DDR_BYTES >> 6;
   localparam [63:0] LBASE  = BASE >> 6;
   reg [511:0] lram [0:NLINES-1];
   reg d_busy; reg [3:0] d_cnt; reg d_we_q; reg [57:0] d_ad_q; reg [511:0] d_wd_q;
   reg [63:0] line;
   always @(posedge clk) begin
      ddr_ack <= 1'b0;
      if (reset) d_busy <= 1'b0;
      else if (!d_busy && ddr_req) begin
         d_busy<=1'b1; d_cnt<=4'd4; d_we_q<=ddr_we; d_ad_q<=ddr_addr; d_wd_q<=ddr_wdata;
      end else if (d_busy) begin
         if (d_cnt==0) begin
            line = ({6'd0, d_ad_q} - LBASE) & (NLINES-1);
            if (d_we_q) lram[line] <= d_wd_q;
            else        ddr_rdata  <= lram[line];
            ddr_ack <= 1'b1; d_busy <= 1'b0;
         end else d_cnt <= d_cnt - 1;
      end
   end

   // $fread fills a packed array MSB-first within each element, so each 64-byte line
   // comes out byte-reversed; swap it back to little-endian.
   task load_bin; input [8*256-1:0] fname; input [63:0] off;
      integer fd, n, j; reg [63:0] sl, nl, li; reg [7:0] t; begin
         fd = $fopen(fname, "rb");
         if (fd == 0) begin $display("FATAL: cannot open %0s", fname); $finish; end
         sl = off >> 6;
         n  = $fread(lram, fd, sl, NLINES - sl);
         $fclose(fd);
         nl = (n + 63) >> 6;
         for (li = sl; li < sl + nl; li = li + 1)
            for (j = 0; j < 32; j = j + 1) begin
               t = lram[li][j*8 +: 8];
               lram[li][j*8 +: 8]      = lram[li][(63-j)*8 +: 8];
               lram[li][(63-j)*8 +: 8] = t;
            end
         $display("[ino-linux: loaded %0d bytes @ DDR+%h (%0d lines)]", n, off, nl);
      end
   endtask

   // Interrupt-path observability. Interrupt-driven UART TX is load-bearing here: the
   // DTS declares the IRQ, so the 8250 driver waits for a THRE interrupt to drain its
   // xmit buffer. If those never land, the kernel keeps running and polled printk still
   // looks healthy while userspace wedges in its first console write() -- a silent
   // failure that looks exactly like "slow". These counters make it visible.
   reg [63:0] n_inject, n_uirq, n_seip;
   always @(posedge clk) if (!reset) begin
      if (dut.core.irq_inject & dut.core.accept) n_inject <= n_inject + 1;
      if (dut.uart_irq)                          n_uirq   <= n_uirq   + 1;
      if (dut.plic_seip)                         n_seip   <= n_seip   + 1;
   end

   reg [8*256-1:0] fw, dtb, initrd;
   reg [63:0] ncyc, c, nret;
   initial begin
      ncyc = 200000000; nret = 0;
      n_inject = 0; n_uirq = 0; n_seip = 0;
      if (!$value$plusargs("fw=%s", fw))   begin $display("FATAL: +fw");  $finish; end
      if (!$value$plusargs("dtb=%s", dtb)) begin $display("FATAL: +dtb"); $finish; end
      if ($value$plusargs("cycles=%d", ncyc)) ;
      off_dtb = OFF_DTB_DEF; off_initrd = OFF_INITRD_DEF;
      if ($value$plusargs("dtb_off=%h",    off_dtb))    ;
      if ($value$plusargs("initrd_off=%h", off_initrd)) ;
      load_bin(fw,  OFF_FW);
      load_bin(dtb, off_dtb);
      if ($value$plusargs("initrd=%s", initrd)) load_bin(initrd, off_initrd);

      reset = 1; @(negedge clk); @(negedge clk); reset = 0;
      // +cycles=0 runs unbounded (stop with an external interrupt / timeout wrapper)
      for (c = 0; (ncyc == 0) || (c < ncyc); c = c + 1) begin
         @(negedge clk);
         if (retire) nret = nret + 1;
         if ((c % 1000000) == 0)
            $display("[c=%0d retires=%0d va=%h pa=%h prv=%0d satp=%h inj=%0d uirq=%0d seip=%0d ier=%h]",
                     c, nret, dut.core.fe.u_fetch.pc_q, dut.imem_addr, dut.core.mmu_priv,
                     dut.core.mmu_satp, n_inject, n_uirq, n_seip, dut.uart_ier);
      end
      $display("INO-LINUX TIMEOUT after %0d cycles (retires=%0d pc~%h)", ncyc, nret,
               dut.imem_addr);
      $finish;
   end
endmodule

`default_nettype wire
