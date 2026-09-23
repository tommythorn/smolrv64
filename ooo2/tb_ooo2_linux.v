`timescale 1ns/1ps
`default_nettype none

// Linux-boot harness for the SoC (rv_soc_top). Resets DIRECTLY to OpenSBI
// (0x8000_0000) with a1 = the DTB pointer (+a1=, seeded into rv_regfile's x11),
// so the monitor is bypassed and the DUT starts where the reference model would.
// Console output comes out of rv_soc_top's UART $write.
//
//   +fw=<path> +dtb=<path> [+initrd=<path>] [+disk=<image> [+disk_ro]] [+vio_trace] [+dma_rand=<seed>]
//   [+dtb_off=<hex>] [+initrd_off=<hex>] [+a1=<hex>] [+cycles=N]   (0 = unbounded)
//
// virtio-blk (virtio_mmio + virtio_blk + a file-backed SD card model) is instantiated below;
// it DMAs non-coherently into the behavioral DDR and mirrors every write into the reference
// (B6, 2026-09-17). The default tiny128 DTB carries no virtio node, so without +disk the
// region is never touched and the run is the plain initrd boot.
module tb;
   localparam [63:0] BASE = 64'h8000_0000;
`ifdef OOO2_MEM_SIZE_LG2
   localparam [63:0] DDR_BYTES = 64'd1 << `OOO2_MEM_SIZE_LG2;
`else
   localparam [63:0] DDR_BYTES = 64'd1 << 29;        // 512 MiB
`endif
   localparam [63:0] OFF_FW = 64'h000_0000, OFF_DTB_DEF = 64'h1ff0_0000,
                     OFF_INITRD_DEF = 64'h1f52_c000;
   reg [63:0] off_dtb, off_initrd;

   reg clk = 0; always #5 clk = ~clk;
   reg reset;
      wire        retire, retire2, retire3, dmem_wen;
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

   // The reset PC is a define so the same bench boots the ROM MONITOR from the SoC's SRAM:
   //   VDEFS='-DSOC_BOOT_HEX="<src/binline.py monitor.bin>" -DTB_RESET_PC=64'"'"'h7000_0000'
   // (2026-09-05: gate V4 printed no monitor banner on the board while every cosim passed;
   // nothing had ever run the monitor's code through the two-wide core in simulation).
`ifndef TB_RESET_PC
 `define TB_RESET_PC 64'h8000_0000
`endif
   // The SoC's integrity log (rv_errlog). Every logged invariant also $fatals a cycle earlier
   // in simulation, so a sticky bit here means a condition is wired wrong, not that a rule
   // broke -- and that is exactly what must never reach a bitstream.
   always @(posedge clk) if (|dut.u_errlog.sticky)
      $fatal(1, "tb: integrity log sticky=%h first=%0d @%0d rose without a $fatal", dut.u_errlog.sticky, dut.u_errlog.first_idx, dut.u_errlog.first_cyc);
   rv_soc_top #(.RESET_PC(`TB_RESET_PC)) dut
     (.clk(clk), .reset(reset), .retire(retire), .retire2(retire2), .retire3(retire3),
      .dmem_wen(dmem_wen), .dmem_waddr(dmem_waddr), .dmem_wdata(dmem_wdata),
      .dmem_wmask(dmem_wmask),
      .ddr_req(ddr_req), .ddr_we(ddr_we), .ddr_addr(ddr_addr),
      .ddr_wdata(ddr_wdata), .ddr_rdata(ddr_rdata), .ddr_ack(ddr_ack),
      .uart_rx_we(1'b0), .uart_rx_data(8'd0), .uart_rx_ready(rx_ready),
      .uart_tx_valid(uart_tx_v), .uart_tx_ready(uart_tx_rdy),
      .virtio_addr(virtio_addr), .virtio_read(virtio_read), .virtio_write(virtio_write),
      .virtio_wdata(virtio_wdata), .virtio_be(virtio_be),
      .virtio_rdata(virtio_rd_q), .virtio_rvalid(virtio_rvalid), .virtio_irq(virtio_irq | dma_irq), .virtio_net_irq(1'b0), .irq_dbg());

   // THE DDR MODEL IS THE MEASURED ONE BY DEFAULT. Until 2026-09-04 the default was a flat
   // 4-cycle line latency, which is not this machine: the D$ admitted two requests under a
   // fill where the board admits dozens, the store queue never filled the way it does on the
   // board, and two six-day-old ordering defects lived through every gate.
   //
   // MEASURED on the board at 166.67 MHz on 2026-09-04 (src/ddr_hpm.v read with
   // workloads/ddrhpm; 1.2 G reads and 139 M writes over an Ubuntu boot, then 443 M reads
   // over a 512 MB kernel memcpy stream and a sort -- the two agree to 1%):
   //     read  (fill):       mean 28.75 cycles -- 96.5% in 16-31, 1.2% in 32-63, 2.3% in 64+
   //     write (write-back): mean 15.62 cycles -- 95.1% in  8-15, 1.2% in 16-31, 2.3% in 32-63, 1.3% in 64+
   // Reads and writes differ because writes post at the bridge, so they draw separately; the
   // tails are occasional excursions (refresh, bank conflicts), not a wide uniform spread.
   // The base bands are placed at the top of the measured bin so the means match: reads
   // 24-31, writes 12-15.
   //
   //   +ddr_lat=N     flat N-cycle latency, reads and writes alike (the old model; sweeps)
   //   -DDDR_LAT=N    the same, at compile time (kept for the existing sweep scripts)
   //   -DDDR_JIT=J    uniform 0..J added on top of whichever base is in force
`ifndef DDR_LAT
 `define DDR_LAT 0
`endif
`ifndef DDR_JIT
 `define DDR_JIT 0
`endif
   reg [63:0] ddr_lat_arg;
   initial if (!$value$plusargs("ddr_lat=%d", ddr_lat_arg)) ddr_lat_arg = 64'd`DDR_LAT;
   reg [15:0] dlfsr; initial dlfsr = 16'hBEEF;
   function [7:0] ddr_draw;      // the measured shape; 0 = flat override in force
      input is_wr;
      reg [9:0] r;               // 0..1023: the thresholds below are the measured fractions
      begin
         r = dlfsr[9:0];
         if (ddr_lat_arg != 0)              ddr_draw = ddr_lat_arg[7:0];
         else if (is_wr) begin
            if      (r < 10'd13)            ddr_draw = 8'd64 + {2'd0, r[5:0]};   // 1.3%: 64-127
            else if (r < 10'd36)            ddr_draw = 8'd32 + {3'd0, r[4:0]};   // 2.3%: 32-63
            else if (r < 10'd48)            ddr_draw = 8'd16 + {4'd0, r[3:0]};   // 1.2%: 16-31
            else                            ddr_draw = 8'd12 + {6'd0, r[1:0]};   // 95%:  12-15, mean 13.5
         end else begin
            if      (r < 10'd24)            ddr_draw = 8'd64 + {2'd0, r[5:0]};   // 2.3%: 64-127
            else if (r < 10'd36)            ddr_draw = 8'd32 + {3'd0, r[4:0]};   // 1.2%: 32-63
            else                            ddr_draw = 8'd24 + {5'd0, r[2:0]};   // 96.5%: 24-31, mean 27.5
         end
      end
   endfunction
   // behavioral DDR, modeled as a 512-bit LINE array: the ddr_* port is 64-byte lines, so
   // the element count stays under the array-dimension limit even at 2 GiB (a flat byte
   // array overflows it).
   localparam [63:0] NLINES = DDR_BYTES >> 6;
   localparam [63:0] LBASE  = BASE >> 6;
   reg [511:0] lram [0:NLINES-1];
   integer ddr_seed = 1;
   reg d_busy; reg [15:0] d_cnt; reg d_we_q; reg [57:0] d_ad_q; reg [511:0] d_wd_q;
   reg [63:0] line;
   always @(posedge clk) begin
      ddr_ack <= 1'b0;
      if (reset) d_busy <= 1'b0;
      else if (!d_busy && ddr_req) begin
         d_busy<=1'b1; d_cnt<={8'd0, ddr_draw(ddr_we)} + ({$random(ddr_seed)} % (`DDR_JIT + 1));
         dlfsr <= {dlfsr[14:0], dlfsr[15]^dlfsr[13]^dlfsr[12]^dlfsr[10]};
         d_we_q<=ddr_we; d_ad_q<=ddr_addr; d_wd_q<=ddr_wdata;
      end else if (d_busy) begin
         if (d_cnt==0) begin
            line = ({6'd0, d_ad_q} - LBASE) & (NLINES-1);
            if (d_we_q) lram[line] <= d_wd_q;
            else        ddr_rdata  <= lram[line];
            ddr_ack <= 1'b1; d_busy <= 1'b0;
         end else d_cnt <= d_cnt - 1;
      end
   end


   // ==================== virtio-blk (B6, 2026-09-17; from the retired tb_cosim_linux.v) ====================
   // +disk=<image> attaches a file-backed SPI-mode SD card behind virtio_blk (virtio_mmio at
   // 0x10002000, PLIC source 11; the DTB must carry the node). The device DMAs straight into
   // lram[] -- non-coherent, as on the board -- and every write beat is mirrored into simmerv
   // (cosim_dma_write, drained between the two retires it separates), so the reference sees the
   // same bytes the DUT's cache does not. Without +disk the region answers as a device with no
   // media and the default DTB never touches it. +disk_ro keeps the image's writes in RAM.
   //
   // soc's window is 8 KiB: addr[12] selects blk(+0)/net(+0x1000). The sim has no net backend:
   // net-window reads return 0 (magic 0 -> the kernel skips the node cleanly); truncating bit 12
   // instead aliased net onto blk = a ghost vdb probe (seen in the retired tb_virtio).
   wire [12:0] virtio_addr;  wire virtio_read, virtio_write;
   wire [31:0] virtio_wdata; wire [3:0] virtio_be;
   wire [31:0] virtio_rdata_comb; wire virtio_irq;
   wire        vio_net = virtio_addr[12];
   wire [31:0] vio_rdata = vio_net ? 32'd0 : virtio_rdata_comb;
   reg vio_trace = 1'b0;
   // FPGA-CDC-shaped latency: the response (reads AND writes) lands 3 cycles after the request
   reg  [31:0] virtio_rd_q;  reg virtio_rvalid;  reg [2:0] vio_lat;
   always @(posedge clk) begin
      virtio_rvalid <= 1'b0;
      if (reset) begin vio_lat <= 3'd0; virtio_rd_q <= 32'd0; end
      else if (virtio_read || virtio_write) begin virtio_rd_q <= vio_rdata; vio_lat <= 3'd3; end
      else if (vio_lat != 3'd0) begin vio_lat <= vio_lat - 3'd1; if (vio_lat == 3'd1) virtio_rvalid <= 1'b1; end
      if ((virtio_read || virtio_write) && vio_trace)   // low-rate (probe/config/notify); +vio_trace
         $display("[VIO c=%0d %s a=%h d=%h]", c, virtio_read ? "R" : "W", virtio_addr,
                  virtio_read ? vio_rdata : virtio_wdata);
   end

   wire        v_notify_pulse;   wire [31:0] v_notify_value;
   wire        v_used_irq;       wire [31:0] v_capacity;
   wire  [7:0] v_dev_status;
   wire [31:0] v_q0_num;  wire v_q0_ready;
   wire [63:0] v_q0_desc, v_q0_driver, v_q0_device;
   virtio_mmio #(.DEVICE_ID(32'd2), .QUEUE_NUM_MAX(32'd8)) u_vmmio
     (.clock(clk), .reset(reset),
      .address(virtio_addr[11:0]), .read(virtio_read && !vio_net), .read_data(virtio_rdata_comb),
      .write(virtio_write && !vio_net), .write_data(virtio_wdata), .byteenable(virtio_be),
      .config_capacity_sectors(v_capacity),
      .irq(virtio_irq),
      .queue_notify_pulse(v_notify_pulse), .queue_notify_value(v_notify_value),
      .used_buffer_interrupt(v_used_irq), .config_change_interrupt(1'b0),
      .driver_features_0(), .driver_features_1(),
      .queue_num(), .queue_ready(), .queue_desc(), .queue_driver(), .queue_device(),
      .queue0_num(v_q0_num), .queue0_ready(v_q0_ready), .queue0_desc(v_q0_desc),
      .queue0_driver(v_q0_driver), .queue0_device(v_q0_device),
      .queue1_num(), .queue1_ready(), .queue1_desc(), .queue1_driver(), .queue1_device(),
      .device_status(v_dev_status));

   wire [2:0]  ax_awid;   wire [30:0] ax_awaddr; wire [7:0] ax_awlen;  wire [2:0] ax_awsize;
   wire [1:0]  ax_awburst; wire ax_awlock; wire [3:0] ax_awcache; wire [2:0] ax_awprot;
   wire [3:0]  ax_awqos;  wire ax_awvalid;  wire ax_awready;
   wire [63:0] ax_wdata;  wire [7:0] ax_wstrb; wire ax_wlast; wire ax_wvalid; wire ax_wready;
   wire [2:0]  ax_bid;    wire [1:0] ax_bresp; wire ax_bvalid; wire ax_bready;
   wire [2:0]  ax_arid;   wire [30:0] ax_araddr; wire [7:0] ax_arlen;  wire [2:0] ax_arsize;
   wire [1:0]  ax_arburst; wire ax_arlock; wire [3:0] ax_arcache; wire [2:0] ax_arprot;
   wire [3:0]  ax_arqos;  wire ax_arvalid;  wire ax_arready;
   wire [2:0]  ax_rid;    wire [63:0] ax_rdata; wire [1:0] ax_rresp; wire ax_rlast; wire ax_rvalid; wire ax_rready;
   wire        blk_sck, blk_mosi, blk_cs_n;  reg blk_miso = 1'b1;

   virtio_blk #(.QUEUE_SIZE(32'd8), .SD_SLOW_HALF(16'd4), .SD_FAST_HALF(16'd2), .SD_INIT_TICKS(16'd10)) u_vblk
     (.clock(clk), .reset(reset),
      .queue_notify_pulse(v_notify_pulse), .queue_notify_value(v_notify_value),
      .queue_num(v_q0_num), .queue_ready(v_q0_ready), .queue_desc(v_q0_desc),
      .queue_driver(v_q0_driver), .queue_device(v_q0_device),
      .device_status(v_dev_status), .used_buffer_interrupt(v_used_irq),
      .capacity_sectors(v_capacity), .sd_fast_half(16'd4),  // >=3: miso_sync sampling margin
      .debug_sel(2'd0), .debug_word(),
      .sd_sck(blk_sck), .sd_mosi(blk_mosi), .sd_miso(blk_miso), .sd_cs_n(blk_cs_n),
      .m_axi_awid(ax_awid), .m_axi_awaddr(ax_awaddr), .m_axi_awlen(ax_awlen), .m_axi_awsize(ax_awsize),
      .m_axi_awburst(ax_awburst), .m_axi_awlock(ax_awlock), .m_axi_awcache(ax_awcache),
      .m_axi_awprot(ax_awprot), .m_axi_awqos(ax_awqos), .m_axi_awvalid(ax_awvalid), .m_axi_awready(ax_awready),
      .m_axi_wdata(ax_wdata), .m_axi_wstrb(ax_wstrb), .m_axi_wlast(ax_wlast), .m_axi_wvalid(ax_wvalid), .m_axi_wready(ax_wready),
      .m_axi_bid(ax_bid), .m_axi_bresp(ax_bresp), .m_axi_bvalid(ax_bvalid), .m_axi_bready(ax_bready),
      .m_axi_arid(ax_arid), .m_axi_araddr(ax_araddr), .m_axi_arlen(ax_arlen), .m_axi_arsize(ax_arsize),
      .m_axi_arburst(ax_arburst), .m_axi_arlock(ax_arlock), .m_axi_arcache(ax_arcache),
      .m_axi_arprot(ax_arprot), .m_axi_arqos(ax_arqos), .m_axi_arvalid(ax_arvalid), .m_axi_arready(ax_arready),
      .m_axi_rid(ax_rid), .m_axi_rdata(ax_rdata), .m_axi_rresp(ax_rresp), .m_axi_rlast(ax_rlast),
      .m_axi_rvalid(ax_rvalid), .m_axi_rready(ax_rready));

   // behavioral always-ready single-beat AXI slave into lram[] (non-coherent DMA);
   // ax_*addr[30:0] is the DDR byte offset (guest_pa - BASE).
   reg axi_rvalid, axi_bvalid;  reg [63:0] axi_rdata;  integer ka;
   reg [63:0] ar_line, aw_line;  reg [9:0] ar_bp, aw_bp;  integer n_dma_rd = 0, n_dma_wr = 0;
   assign ax_arready = 1'b1; assign ax_awready = 1'b1; assign ax_wready = 1'b1;
   assign ax_rvalid = axi_rvalid; assign ax_rdata = axi_rdata; assign ax_rresp = 2'd0;
   assign ax_rlast = 1'b1; assign ax_rid = 3'd1;
   assign ax_bvalid = axi_bvalid; assign ax_bresp = 2'd0; assign ax_bid = 3'd1;
`ifdef OOO2_COSIM
   import "DPI-C" function void cosim_dma_write(input longint off, input longint data, input byte strb);
`endif
   always @(posedge clk) begin
      if (reset) begin axi_rvalid<=1'b0; axi_bvalid<=1'b0; end
      else begin
         if (ax_arvalid && ax_arready && !axi_rvalid) begin
            ar_line = ((ax_araddr & (DDR_BYTES-1)) >> 6) & (NLINES-1);
            ar_bp   = (ax_araddr & 6'h3f) << 3;
            axi_rdata <= lram[ar_line][ar_bp +: 64];
            axi_rvalid <= 1'b1;  n_dma_rd = n_dma_rd + 1;
         end else if (axi_rvalid && ax_rready) axi_rvalid <= 1'b0;
         if (ax_awvalid && ax_awready && ax_wvalid && ax_wready && !axi_bvalid) begin
            aw_line = ((ax_awaddr & (DDR_BYTES-1)) >> 6) & (NLINES-1);
            aw_bp   = (ax_awaddr & 6'h3f) << 3;
            for (ka=0;ka<8;ka=ka+1) if (ax_wstrb[ka]) lram[aw_line][aw_bp + ka*8 +: 8] <= ax_wdata[ka*8 +: 8];
`ifdef OOO2_COSIM
            cosim_dma_write({33'd0, ax_awaddr}, ax_wdata, ax_wstrb);   // mirror into simmerv
`endif
            axi_bvalid <= 1'b1;  n_dma_wr = n_dma_wr + 1;
         end else if (axi_bvalid && ax_bready) axi_bvalid <= 1'b0;
      end
   end


   // ---- the DMA agent (B4 memrand, 2026-09-17): +dma_rand=<seed> ---------------------------------
   // A device that DMAs on its own schedule: every 20-100 k cycles it writes a 64-byte burst of
   // random bytes into a random line of the 4 KiB DMA window (DDR offset 0x120_0000; the last line
   // holds the ack word), mirrors each beat into the reference (the same DPI-ordered queue as the
   // virtio-blk slave), and raises PLIC source 11 (shared with virtio-blk's irq). The program's
   // S-mode handler sums the window through its NC mapping, writes the burst count to the ack word
   // and completes; the agent drops the irq once the ack word in lram shows the count, then
   // schedules the next burst. Race-free by protocol: the interrupt orders every read of the window
   // after the beats on both sides (a polled flag would not be: a beat landing between a poll's
   // execution and its retire is applied to the reference before that poll is compared).
   localparam [63:0] DMA_OFF = 64'h0120_0000;
   reg         dma_on = 1'b0, dma_irq = 1'b0;
   reg  [31:0] dma_seed = 32'd1, dma_lfsr = 32'hACE1_5EED, dma_ctr = 32'd0, dl;
   reg  [63:0] dma_next = 64'd50000, dma_line;  reg [63:0] dma_beat;  integer dk;
   wire [63:0] dma_ack = lram[(DMA_OFF >> 6) + 64'd63][448 +: 64];
   always @(posedge clk) if (dma_on && !reset) begin
      if (!dma_irq && c >= dma_next) begin
         dl = dma_lfsr;
         dma_line = (DMA_OFF >> 6) + {58'd0, dl[5:0] == 6'd63 ? 6'd0 : dl[5:0]};
         for (dk = 0; dk < 8; dk = dk + 1) begin
            dl = dl ^ (dl << 13); dl = dl ^ (dl >> 17); dl = dl ^ (dl << 5);
            dma_beat = {dl, dl ^ 32'h9E37_79B9};
            lram[dma_line][dk*64 +: 64] <= dma_beat;
`ifdef OOO2_COSIM
            cosim_dma_write((dma_line << 6) + dk*8, dma_beat, 8'hFF);
`endif
         end
         dma_lfsr <= dl;  dma_ctr <= dma_ctr + 1;  dma_irq <= 1'b1;
      end else if (dma_irq && dma_ack == {32'd0, dma_ctr}) begin
         dma_irq <= 1'b0;  dma_next <= c + 64'd20000 + {48'd0, dma_lfsr[15:0]};
      end
   end

   // DPI: the file-backed SpiSdCard (src/sd_dpi.cpp) clocked by the SD-SPI pins each cycle
   import "DPI-C" function void sd_attach(input string path);
   import "DPI-C" function int  sd_clock(input int sck, input int cs_n, input int mosi);
   import "DPI-C" function void sd_readonly();
   always @(posedge clk) blk_miso <= sd_clock({31'd0, blk_sck}, {31'd0, blk_cs_n}, {31'd0, blk_mosi}) > 0 ? 1'b1 : 1'b0;

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
         $display("[ooo2-linux: loaded %0d bytes @ DDR+%h (%0d lines)]", n, off, nl);
      end
   endtask

   // Interrupt-path observability. Interrupt-driven UART TX is load-bearing here: the
   // DTS declares the IRQ, so the 8250 driver waits for a THRE interrupt to drain its
   // xmit buffer. If those never land, the kernel keeps running and polled printk still
   // looks healthy while userspace wedges in its first console write() -- a silent
   // failure that looks exactly like "slow". These counters make it visible.
   reg [63:0] n_inject, n_uirq, n_seip;
   initial begin n_ldstart=0; n_ldreord=0; n_ldblk=0; n_sqocc=0; n_sqfull=0; end
   always @(posedge clk) if (!reset) begin
      if (dut.core.irq_inject & dut.core.accept) n_inject <= n_inject + 1;
      if (dut.uart_irq)                          n_uirq   <= n_uirq   + 1;
      if (dut.plic_seip)                         n_seip   <= n_seip   + 1;
   end

   // ---- SIZING THE SCOREBOARD (measurement only; no RTL is changed for this) ----------
   // M blocks until the LSU has the DATA (ooo2_core.v: `m_done = ... m_mem_op ? lsu_done`),
   // and the LSU is 38.8% of GB5 cycles with 94-98% of that being hit latency rather than
   // misses.  A 1-deep, loads-only scoreboard would let X proceed on exactly the cycles
   // counted here: M stalled on the LSU, X holding an instruction that does not need the
   // (single-outstanding) LSU, is not serializing, and does not read the load's result.
   //
   // Reuses the core's own byp1/2/3 for the dependence test rather than re-deriving it --
   // they already are `m_valid & m_rd_v & (m_rd == d_rs*)`.  Qualified by d_rs*_v here so
   // an absent operand cannot look like a dependence.
   //
   // This is a LOWER BOUND: it only credits the immediately-next instruction, not the
   // second and third that would also flow once the first moved.
   // Was byp1/2/3 -- "the next instruction wants the value M is producing". With dispatch
   // decoupled that test moved into the scheduler, which reports whether its OLDEST entry
   // is blocked on a source. Same question, asked where the waiting now happens.
   // ---- PIPEVIEW: cycle-by-cycle waterfall, +pipe=<start> [+pipe_n=<cycles>] ---------
   // Peeks the core; no DUT change. Aggregate counters stopped being trustworthy when the
   // schedulers split -- ST_FPU counts only dep_fp now that FP never enters M, and
   // iq_blk_pr gives the LD scheduler priority, so an FP dependency behind a load is
   // charged to ST_MEM. A waterfall does not have that problem: it shows which stage each
   // instruction sat in, per cycle, and where the gaps are.
   //
   //   DIS  dispatched (rename+ROB alloc)      ISS  selected into the issue register
   //   ALU  completed at issue                 M/F  entered the M or F execute stage
   //   LD   a load landed                      FP   an FP result landed
   //   RET  retired at the ROB head            RED  redirect
   integer pv_from = -1, pv_n = 0, pv_cnt = 0;
   // The event bus counted per cycle (B3). pcode(bit) mirrors src/csr_file.v's hpm_inc map;
   // tools/gen-perf-events.py --check keeps the JSON honest, and perf-cpi-stack.py's identity
   // check exposes a bit that drifted from its code.
   integer pe;  reg [63:0] pev [0:40];
   function [15:0] pcode; input integer b; begin
      case (b)
        0: pcode=16'h0003; 1: pcode=16'h0004; 2: pcode=16'h0005; 3: pcode=16'h0100; 4: pcode=16'h0102;
        5: pcode=16'h0110; 6: pcode=16'h0112; 7: pcode=16'h0300; 8: pcode=16'h0301; 9: pcode=16'h0302;
        10: pcode=16'h0303; 11: pcode=16'h0304; 12: pcode=16'h0310; 13: pcode=16'h0311; 14: pcode=16'h0312;
        15: pcode=16'h0006; 16: pcode=16'h0007; 17: pcode=16'h0008; 18: pcode=16'h0313; 19: pcode=16'h0314;
        20: pcode=16'h0315; 21: pcode=16'h0316; 22: pcode=16'h0305; 23: pcode=16'h0317; 24: pcode=16'h0318;
        25: pcode=16'h0104; 26: pcode=16'h0306; 27: pcode=16'h0307; 28: pcode=16'h0308; 29: pcode=16'h0309;
        30: pcode=16'h030a; 31: pcode=16'h0319; 32: pcode=16'h031a; 33: pcode=16'h031b; 34: pcode=16'h031c;
        35: pcode=16'h031d; 36: pcode=16'h031e; 37: pcode=16'h031f; 38: pcode=16'h0320; 39: pcode=16'h0321;
        40: pcode=16'h0322; default: pcode=16'hffff;
      endcase end endfunction
   initial for (pe = 0; pe < 41; pe = pe + 1) pev[pe] = 64'd0;
   reg [63:0] trace_from, trace_to;             // +trace_from/+trace_to, see the plusargs
   integer pv_c;
   initial pv_c = 0;
   always @(posedge clk) if (!reset) begin
      pv_c <= pv_c + 1;
      if (trace_on && pv_from >= 0 && pv_c >= pv_from && pv_c < pv_from + pv_n) begin
         $write("pv %0d |", pv_c);
         if (dut.core.d_take)      $write(" DIS:%0d", dut.core.rob_d_idx);   else $write("        ");
         if (dut.core.iq_iss_take) $write(" ISS:%s%0d",
              dut.core.pick_l ? "L" : "I", dut.core.iq_iss_rob);   // M-only port now; F issues via rf_take
                                                                             else $write("        ");
         if (dut.core.iss_alu)     $write(" ALU:%0d", dut.core.i_rob);       else $write("        ");
         if (dut.core.iss_m)       $write(" M:%0d",   dut.core.i_rob);       else $write("      ");
         if (dut.core.iss_f)       $write(" F:%0d",   dut.core.j_rob);       else $write("      ");
         if (dut.core.ld_land)     $write(" LD:%0d",  dut.core.lq_l_rob);    else $write("       ");
         if (dut.core.fp_land)     $write(" FP:%0d",  dut.core.ft_rob);      else $write("       ");
         if (dut.core.rob_c_valid) $write(" RET:%0d", dut.core.rob_head_idx);else $write("        ");
         if (dut.core.redirect)    $write(" RED");
         // 2026-09-17 (the storm repro): the backend state a wrong-path device load needs
         $write(" | red=%b cfrf=%b mrf=%b frset=%b frv=%b | M v=%b rob=%0d pc=%h ldnb=%b adv=%b done=%b xo_v=%b walk=%b fill=%b | LQ x_v=%b x_pa=%h cnt=%0d | ROB empty=%b head=%0d | iss_m=%b",
                dut.core.redirect, dut.core.cf_red_fire, dut.core.m_red_fire, dut.core.fr_set, dut.core.fr_v,
                dut.core.m_valid, dut.core.m_rob_idx, dut.core.m_pc, dut.core.m_ld_nb, dut.core.m_advance, dut.core.m_done,
                dut.core.lsu_xo_v, dut.core.u_lsu.mmu_walking, dut.core.m_lq_fill,
                dut.core.lq_x_v, dut.core.lq_x_pa, dut.core.u_lq.cnt, dut.core.rob_empty, dut.core.rob_head_idx, dut.core.iss_m);
         $write("\n");
      end
   end

   wire sb_dep    = dut.core.iq_blk_v;
   wire sb_recov  = dut.core.st_mem & dut.core.d_valid
                  & ~dut.core.d_is_mem & ~dut.core.d_is_amo & ~dut.core.d_is_serialize
                  & ~sb_dep;
   // ...and the two reasons a stalled cycle is NOT recoverable, so the total is attributed
   // rather than leaving a silent remainder.
   wire sb_blk_dep = dut.core.st_mem & dut.core.d_valid & sb_dep;
   wire sb_blk_mem = dut.core.st_mem & dut.core.d_valid & ~sb_dep
                   & (dut.core.d_is_mem | dut.core.d_is_amo);
   wire sb_blk_ser = dut.core.st_mem & dut.core.d_valid & ~sb_dep
                   & ~(dut.core.d_is_mem | dut.core.d_is_amo) & dut.core.d_is_serialize;
   // ...and the bucket that turned out to dominate: M is stalled on a load and X is EMPTY,
   // so there is nothing for a scoreboard to run even if it existed. Sub-attributed with
   // the core's own frontend-bubble taps so it is clear whether the frontend is walking the
   // iMMU, out of fetch window, or something else.
   wire sb_novalid = dut.core.st_mem & ~dut.core.d_valid;
   reg [63:0] n_stmem, n_recov, n_bdep, n_bmem, n_bser, n_nov, n_nov_mmu, n_nov_ic, n_nov_qrdy;
   // Per-unit stall, to size the OTHER units against the LSU. mul3 and cvfpu are both
   // internally pipelined and only held to one outstanding by a busy flag / a discarded
   // tag, so their tier is cheap -- but cheap is not the same as worth doing.
   reg [63:0] n_stmul, n_stdiv, n_stfpu;
   // STORE BUFFER: is it actually reordering anything?
   //   n_ldstart  loads that began an access
   //   n_ldreord  ...of those, ones that passed an older UNCOMMITTED store
   //   n_ldblk    cycles a load was held by disambiguation instead
   //   n_sqocc    summed occupancy, for sizing NENT
   reg [63:0] n_ldstart, n_ldreord, n_ldblk, n_sqocc, n_sqfull;
   // Where did the cycles the scoreboard freed actually go?
   reg [63:0] n_stm, n_hold, n_headblk, n_mempty, n_ldland, n_robfull, n_srcpend;
   // THE STORE PATH, cycle by cycle (plan item 4): where a store waits between the queue
   // and the cache. n_stdoor: the LSU holds a store at the D$ door, not accepted this cycle;
   // n_sqidle: the queue has a drainable head, the LSU is idle and does not start it;
   // n_sqwait: the queue has entries and none is drainable (unreleased, or no data yet).
   reg [63:0] n_stdoor, n_sqidle, n_sqwait;
   always @(posedge clk) if (!reset) begin
      if (dut.core.st_mem) n_stmem <= n_stmem + 1;
      if (sb_recov)        n_recov <= n_recov + 1;
      if (sb_blk_dep)      n_bdep  <= n_bdep  + 1;
      if (sb_blk_mem)      n_bmem  <= n_bmem  + 1;
      if (sb_blk_ser)      n_bser  <= n_bser  + 1;
      if (sb_novalid)      n_nov   <= n_nov   + 1;
      if (sb_novalid & ~dut.core.immu_ready) n_nov_mmu <= n_nov_mmu + 1;
      if (sb_novalid &  dut.core.immu_ready
          & (dut.core.imem_avail_g == 0))    n_nov_ic  <= n_nov_ic  + 1;
      // THE distinction that decides whether the empty-X bucket is recoverable. The IR can
      // only be loaded on `accept` (= m_advance), so a stalled M freezes it even when the
      // decoupling queue behind it is holding instructions. Those cycles ARE recoverable by the
      // scoreboard -- one cycle later, as the IR refills. Only ~q_empty is genuinely dry.
      if (sb_novalid & ~dut.core.fe.q_empty) n_nov_qrdy <= n_nov_qrdy + 1;
      if (dut.core.st_mul) n_stmul <= n_stmul + 1;
      if (dut.core.st_div) n_stdiv <= n_stdiv + 1;
      if (dut.core.st_fpu) n_stfpu <= n_stfpu + 1;
      // Loads no longer start via lsu_started -- they access from the queue's port.
      if (dut.core.lq_x_take)      n_ldstart <= n_ldstart + 1;
      if (dut.core.sq_ld_reorder)  n_ldreord <= n_ldreord + 1;
      if (dut.core.sq_ld_block)    n_ldblk   <= n_ldblk   + 1;
      n_sqocc <= n_sqocc + {61'd0, dut.core.sq_occ};
      if (~dut.core.sq_d_ready)    n_sqfull  <= n_sqfull  + 1;
      if (dut.core.m_valid & ~dut.core.m_done)      n_stm     <= n_stm + 1;
      if (dut.core.d_hold)                          n_hold    <= n_hold + 1;
      if (dut.core.u_lsu.st_go & ~dut.dmem_waccept) n_stdoor  <= n_stdoor + 1;
      if (dut.core.u_lsu.idle & dut.core.sq_c_v & ~dut.core.u_lsu.pt_start) n_sqidle <= n_sqidle + 1;
      if ((dut.core.sq_occ != 0) & ~dut.core.sq_c_v)  n_sqwait  <= n_sqwait + 1;
      if (dut.core.iq_blk_v & dut.core.d_valid)     n_srcpend <= n_srcpend + 1;
      if (~dut.core.rob_ready & dut.core.d_valid)   n_robfull <= n_robfull + 1;
      if (dut.core.head_block)                      n_headblk <= n_headblk + 1;
      if (~dut.core.m_valid)                        n_mempty  <= n_mempty + 1;
      if (dut.core.ld_land)                         n_ldland  <= n_ldland + 1;
   end

   reg [8*256-1:0] fw, dtb, initrd, disk;
   reg [63:0] ncyc, c, nret;
   wire       trace_on = (c >= trace_from) && (c < trace_to);   // the tb-side trace window
   initial begin
      ncyc = 200000000; nret = 0;
      n_inject = 0; n_uirq = 0; n_seip = 0;
      n_stmem = 0; n_recov = 0; n_bdep = 0; n_bmem = 0; n_bser = 0;
      n_nov = 0; n_nov_mmu = 0; n_nov_ic = 0; n_nov_qrdy = 0;
      n_stmul = 0; n_stdiv = 0; n_stfpu = 0;
      n_stm = 0; n_hold = 0; n_headblk = 0; n_mempty = 0; n_ldland = 0;
      n_robfull = 0; n_srcpend = 0; n_stdoor = 0; n_sqidle = 0; n_sqwait = 0;
      // +trace_from=<cycle> +trace_to=<cycle>: the window every tb-side trace honours, so a
      // trace is aimed by plusarg and never by a compile-time literal (three 17 M-cycle
      // rebuilds on 2026-09-04 went to a `$time` threshold in the wrong unit). RTL-side
      // `ifdef` traces read the same two plusargs themselves (rule G6).
      if (!$value$plusargs("trace_from=%d", trace_from)) trace_from = 64'd0;
      if (!$value$plusargs("trace_to=%d",   trace_to))   trace_to   = 64'hFFFF_FFFF_FFFF_FFFF;
      if ($value$plusargs("pipe=%d", pv_from)) pv_n = 200;
      if ($value$plusargs("pipe_n=%d", pv_cnt))  pv_n = pv_cnt;
      if (!$value$plusargs("fw=%s", fw))   begin $display("FATAL: +fw");  $finish; end
      if (!$value$plusargs("dtb=%s", dtb)) begin $display("FATAL: +dtb"); $finish; end
      if ($value$plusargs("cycles=%d", ncyc)) ;
      off_dtb = OFF_DTB_DEF; off_initrd = OFF_INITRD_DEF;
      if ($value$plusargs("dtb_off=%h",    off_dtb))    ;
      if ($value$plusargs("initrd_off=%h", off_initrd)) ;
      load_bin(fw,  OFF_FW);
      load_bin(dtb, off_dtb);
      if ($value$plusargs("initrd=%s", initrd)) load_bin(initrd, off_initrd);
      if ($value$plusargs("disk=%s", disk)) begin
         sd_attach(disk);
         if ($test$plusargs("disk_ro")) begin sd_readonly(); $display("[sd: SNAPSHOT mode -- image writes stay in RAM]"); end
      end
      if ($test$plusargs("vio_trace")) vio_trace = 1'b1;
      if ($value$plusargs("dma_rand=%d", dma_seed)) begin dma_on = 1'b1; dma_lfsr = {dma_seed[15:0], 16'hACE1} ^ 32'h5EED_0000; end

      reset = 1; @(negedge clk); @(negedge clk); reset = 0;
      // +cycles=0 runs unbounded (stop with an external interrupt / timeout wrapper)
      for (c = 0; (ncyc == 0) || (c < ncyc); c = c + 1) begin
         @(negedge clk);
                  nret = nret + retire + retire2 + retire3;   // all three commit ports (IW=3 undercounted before 2026-09-17)
                  // B3 (2026-09-17): the CPI stack from the RTL's own event bus, so the same tool
                  // (tools/perf-cpi-stack.py) grades a simulation and a board run.
                  for (pe = 0; pe < 39; pe = pe + 1) if (dut.core.hpm_ev_q[pe]) pev[pe] = pev[pe] + 1;
                  pev[39] = pev[39] + dut.core.hpm_lqocc_q;  pev[40] = pev[40] + dut.core.hpm_sqocc_q;
         if ((c % 1000000) == 0)
            $display("[c=%0d retires=%0d va=%h pa=%h prv=%0d satp=%h inj=%0d uirq=%0d seip=%0d ier=%h]",
                     c, nret, dut.core.fe.u_fetch.pc_q, dut.imem_addr, dut.core.mmu_priv,
                     dut.core.mmu_satp, n_inject, n_uirq, n_seip, dut.uart_ier);
      end
      $display("INO-LINUX TIMEOUT after %0d cycles (retires=%0d pc~%h)", ncyc, nret,
               dut.imem_addr);
      $display("[blk: dma rd=%0d wr=%0d]", n_dma_rd, n_dma_wr);   // virtio-blk beats (B6); 0/0 without +disk
      $display("[dma-agent: bursts=%0d]", dma_ctr);   // B4 memrand; 0 without +dma_rand
      // B3: perf-stat text, the shape tools/perf-cpi-stack.py parses (`<count> r<code>`).
      $display("perf-stat-sim: begin");
      $display("%0d cycles", ncyc);
      $display("%0d instructions", nret);
      for (pe = 0; pe < 41; pe = pe + 1) $display("%0d r%04h", pev[pe], pcode(pe));
      $display("perf-stat-sim: end");

      $display("SQ  loads=%0d reordered=%0d (%0d.%0d%%)  ld_block cycles=%0d (%0d.%0d%%)  mean occ=%0d.%02d  full=%0d",
               n_ldstart, n_ldreord,
               n_ldstart ? (100*n_ldreord)/n_ldstart : 0,
               n_ldstart ? ((1000*n_ldreord)/n_ldstart)%10 : 0,
               n_ldblk, (100*n_ldblk)/c, ((1000*n_ldblk)/c)%10,
               n_sqocc/c, ((100*n_sqocc)/c)%100, n_sqfull);
      $display("SB-SIZING cycles=%0d retires=%0d st_mem=%0d (%0d.%02d%% of cycles)",
               c, nret, n_stmem, (n_stmem*100)/c, ((n_stmem*10000)/c)%100);
      $display("SB-SIZING   recoverable   %10d  %0d.%02d%% of cycles, %0d%% of st_mem",
               n_recov, (n_recov*100)/c, ((n_recov*10000)/c)%100, (n_recov*100)/n_stmem);
      $display("SB-SIZING   blk next-is-mem%9d  %0d%% of st_mem   (needs a load queue, not a scoreboard)",
               n_bmem, (n_bmem*100)/n_stmem);
      $display("SB-SIZING   blk dependence %9d  %0d%% of st_mem   (irreducible: it wants the value)",
               n_bdep, (n_bdep*100)/n_stmem);
      $display("SB-SIZING   blk serialize  %9d  %0d%% of st_mem",
               n_bser, (n_bser*100)/n_stmem);
      $display("SB-SIZING   X EMPTY        %9d  %0d%% of st_mem   (iMMU %0d, I$ empty %0d, other %0d)",
               n_nov, (n_nov*100)/n_stmem, n_nov_mmu, n_nov_ic, n_nov-n_nov_mmu-n_nov_ic);
      $display("SB-SIZING     ...of which DECOUPLING QUEUE HAS WORK %0d  (%0d%% of X-empty) -- recoverable, IR just cannot reload while M stalls",
               n_nov_qrdy, (n_nov_qrdy*100)/n_nov);
      $display("SB-SIZING   per-unit stall: mul=%0d div=%0d fpu=%0d  (vs LSU %0d)",
               n_stmul, n_stdiv, n_stfpu, n_stmem);
      $display("SB-WHERE  M-stall=%0d  M-empty=%0d | d_hold=%0d (src_pend=%0d rob_full=%0d) head_block=%0d ld_land=%0d",
               n_stm, n_mempty, n_hold, n_srcpend, n_robfull, n_headblk, n_ldland);
      $display("SB-STORE  at the D$ door unaccepted=%0d  drainable but LSU idle=%0d  queued, none drainable=%0d  (sq full at dispatch=%0d)",
               n_stdoor, n_sqidle, n_sqwait, n_sqfull);
      $display("SB-SIZING   FLOOR %0d.%02d%% of cycles / CEILING %0d.%02d%% (floor + queue-ready X-empty)",
               (n_recov*100)/c, ((n_recov*10000)/c)%100,
               ((n_recov+n_nov_qrdy)*100)/c, (((n_recov+n_nov_qrdy)*10000)/c)%100);
      $finish;
   end
endmodule

`default_nettype wire
