`timescale 1ns/1ps
`default_nettype none

// virtio-blk boot harness for soc_top (Verilator + DPI SD card). Boots OpenSBI +
// Linux from DDR like tb_linux, but attaches a file-backed virtio block device so the
// kernel can mount root=/dev/vda. Reuses the existing virtio_mmio + virtio_blk RTL
// (SD-SPI backend) unchanged; the SD card is the C++ SpiSdCard model bridged via DPI
// (sd_dpi.cpp), backed by a real disk image. virtio_blk's AXI master DMAs DIRECTLY into
// the behavioral DDR (bypassing the D$) = the non-coherent DMA the kernel handles with
// Svpbmt NC rings + Zicbom CMO on data buffers.
//   +monhex=... +fw=... +dtb=... +initrd=... +disk=... [+cycles=N]
module tb;
   localparam [63:0] BASE = 64'h8000_0000;
   localparam        DDR_BYTES = 1<<28;            // 256 MiB
   localparam [63:0] OFF_FW = 64'h000_0000, OFF_DTB = 64'h200_0000, OFF_INITRD = 64'h762_b000;

   reg clk=0; always #5 clk=~clk;
   reg reset;
   wire        commit, dmem_wen;
   wire [63:0] dmem_waddr, dmem_wdata;  wire [7:0] dmem_wmask;
   wire        ddr_req, ddr_we;  wire [57:0] ddr_addr;  wire [511:0] ddr_wdata;
   reg  [511:0] ddr_rdata;  reg ddr_ack;
   reg          rx_we;  reg [7:0] rx_data;  wire rx_ready;

   // ---- virtio MMIO passthrough nets (soc_top <-> virtio_mmio) ----
   wire [11:0] virtio_addr;  wire virtio_read, virtio_write;
   wire [31:0] virtio_wdata; wire [3:0] virtio_be;
   wire [31:0] virtio_rdata; wire virtio_irq;

   soc_top #(.RESET_PC(64'h8000_0000)) dut    // reset straight to OpenSBI (a1=DTB via +a1=)
     (.clk(clk), .reset(reset), .commit(commit),
      .dmem_wen(dmem_wen), .dmem_waddr(dmem_waddr), .dmem_wdata(dmem_wdata), .dmem_wmask(dmem_wmask),
      .ddr_req(ddr_req), .ddr_we(ddr_we), .ddr_addr(ddr_addr),
      .ddr_wdata(ddr_wdata), .ddr_rdata(ddr_rdata), .ddr_ack(ddr_ack),
      .uart_rx_we(rx_we), .uart_rx_data(rx_data), .uart_rx_ready(rx_ready),
      .uart_tx_ready(1'b1),
      .virtio_addr(virtio_addr), .virtio_read(virtio_read), .virtio_write(virtio_write),
      .virtio_wdata(virtio_wdata), .virtio_be(virtio_be),
      .virtio_rdata(virtio_rdata), .virtio_irq(virtio_irq));

   // ---------------- behavioral DDR (256 MiB, 4-cycle line latency) ----------------
   reg [7:0] ram [0:DDR_BYTES-1];
   reg d_busy; reg [3:0] d_cnt; reg d_we_q; reg [57:0] d_ad_q; reg [511:0] d_wd_q;
   integer kb; reg [63:0] d_base;
   always @(posedge clk) begin
      ddr_ack <= 1'b0;
      if (reset) d_busy<=1'b0;
      else if (!d_busy && ddr_req) begin d_busy<=1'b1; d_cnt<=4'd4; d_we_q<=ddr_we; d_ad_q<=ddr_addr; d_wd_q<=ddr_wdata; end
      else if (d_busy) begin
         if (d_cnt==0) begin
            d_base = (({{6{1'b0}},d_ad_q} << 6) - BASE) & (DDR_BYTES-1);
            if (d_we_q) for (kb=0;kb<64;kb=kb+1) ram[d_base+kb] <= d_wd_q[kb*8 +: 8];
            else        for (kb=0;kb<64;kb=kb+1) ddr_rdata[kb*8 +: 8] <= ram[d_base+kb];
            ddr_ack<=1'b1; d_busy<=1'b0;
         end else d_cnt <= d_cnt-1;
      end
   end

   // ==================== virtio-blk subsystem ====================
   // control-plane registers (virtio_mmio) <-> backend DMA/SD engine (virtio_blk)
   wire        v_notify_pulse;   wire [31:0] v_notify_value;
   wire        v_used_irq;       wire [31:0] v_capacity;
   wire  [7:0] v_dev_status;
   wire [31:0] v_q0_num;  wire v_q0_ready;
   wire [63:0] v_q0_desc, v_q0_driver, v_q0_device;

   virtio_mmio #(.DEVICE_ID(32'd2), .QUEUE_NUM_MAX(32'd8)) u_vmmio
     (.clock(clk), .reset(reset),
      .address(virtio_addr), .read(virtio_read), .read_data(virtio_rdata),
      .write(virtio_write), .write_data(virtio_wdata), .byteenable(virtio_be),
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

   // virtio_blk AXI master into ram[] (non-coherent DMA), SD pins to the DPI card.
   wire [2:0]  ax_awid;   wire [30:0] ax_awaddr; wire [7:0] ax_awlen;  wire [2:0] ax_awsize;
   wire [1:0]  ax_awburst; wire ax_awlock; wire [3:0] ax_awcache; wire [2:0] ax_awprot;
   wire [3:0]  ax_awqos;  wire ax_awvalid;  wire ax_awready;
   wire [63:0] ax_wdata;  wire [7:0] ax_wstrb; wire ax_wlast; wire ax_wvalid; wire ax_wready;
   wire [2:0]  ax_bid;    wire [1:0] ax_bresp; wire ax_bvalid; wire ax_bready;
   wire [2:0]  ax_arid;   wire [30:0] ax_araddr; wire [7:0] ax_arlen;  wire [2:0] ax_arsize;
   wire [1:0]  ax_arburst; wire ax_arlock; wire [3:0] ax_arcache; wire [2:0] ax_arprot;
   wire [3:0]  ax_arqos;  wire ax_arvalid;  wire ax_arready;
   wire [2:0]  ax_rid;    wire [63:0] ax_rdata; wire [1:0] ax_rresp; wire ax_rlast; wire ax_rvalid; wire ax_rready;
   wire        blk_sck, blk_mosi, blk_cs_n;  reg blk_miso;

   virtio_blk #(.QUEUE_SIZE(32'd8), .SD_SLOW_HALF(16'd4), .SD_FAST_HALF(16'd2), .SD_INIT_TICKS(16'd10)) u_vblk
     (.clock(clk), .reset(reset),
      .queue_notify_pulse(v_notify_pulse), .queue_notify_value(v_notify_value),
      .queue_num(v_q0_num), .queue_ready(v_q0_ready), .queue_desc(v_q0_desc),
      .queue_driver(v_q0_driver), .queue_device(v_q0_device),
      .device_status(v_dev_status), .used_buffer_interrupt(v_used_irq),
      .capacity_sectors(v_capacity), .sd_fast_half(16'd2),
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

   // ---- behavioral always-ready single-beat AXI slave into ram[] (direct DDR = non-coherent) ----
   // ax_*addr[30:0] is already the DDR byte offset: guest_pa = BASE|off, so pa[30:0] = off = pa-BASE.
   reg axi_rvalid, axi_bvalid;  reg [63:0] axi_rdata;  integer ka;  reg [30:0] aw_off;
   assign ax_arready = 1'b1; assign ax_awready = 1'b1; assign ax_wready = 1'b1;
   assign ax_rvalid = axi_rvalid; assign ax_rdata = axi_rdata; assign ax_rresp = 2'd0;
   assign ax_rlast = 1'b1; assign ax_rid = 3'd1;
   assign ax_bvalid = axi_bvalid; assign ax_bresp = 2'd0; assign ax_bid = 3'd1;
   always @(posedge clk) begin
      if (reset) begin axi_rvalid<=1'b0; axi_bvalid<=1'b0; end
      else begin
         // read channel
         if (ax_arvalid && ax_arready && !axi_rvalid) begin
            for (ka=0;ka<8;ka=ka+1) axi_rdata[ka*8 +: 8] <= ram[(ax_araddr & (DDR_BYTES-1)) + ka];
            axi_rvalid <= 1'b1;
         end else if (axi_rvalid && ax_rready) axi_rvalid <= 1'b0;
         // write channel (address + data arrive together for this single-beat master)
         if (ax_awvalid && ax_awready && ax_wvalid && ax_wready && !axi_bvalid) begin
            aw_off = ax_awaddr & (DDR_BYTES-1);
            for (ka=0;ka<8;ka=ka+1) if (ax_wstrb[ka]) ram[aw_off + ka] <= ax_wdata[ka*8 +: 8];
            axi_bvalid <= 1'b1;
         end else if (axi_bvalid && ax_bready) axi_bvalid <= 1'b0;
      end
   end

   // ---- DPI: file-backed SpiSdCard clocked by the SD-SPI pins each cycle ----
   import "DPI-C" function void sd_attach(input string path);
   import "DPI-C" function int  sd_clock(input int sck, input int cs_n, input int mosi);
   always @(posedge clk) blk_miso <= sd_clock({31'd0, blk_sck}, {31'd0, blk_cs_n}, {31'd0, blk_mosi})>0 ? 1'b1 : 1'b0;

   // ---------------- boot image load (reused from tb_linux) ----------------
   task load_bin; input [8*256-1:0] fname; input [63:0] off;
      integer fd, n; begin
         fd = $fopen(fname, "rb");
         if (fd == 0) begin $display("FATAL: cannot open %0s", fname); $finish; end
         n  = $fread(ram, fd, off, DDR_BYTES-off);
         $fclose(fd);
         $display("[tb_virtio: loaded %0d bytes @ DDR+%h]", n, off);
      end
   endtask

   reg [8*256-1:0] fw, dtb, initrd, disk;
   reg [63:0] ncyc, c;
   initial begin
      rx_we=0; rx_data=0; ncyc=200000000; blk_miso=1'b1;
      if (!$value$plusargs("fw=%s", fw))         begin $display("FATAL: +fw"); $finish; end
      if (!$value$plusargs("dtb=%s", dtb))       begin $display("FATAL: +dtb"); $finish; end
      if ($value$plusargs("cycles=%d", ncyc)) ;
      if ($value$plusargs("disk=%s", disk)) sd_attach(disk);
      else $display("[tb_virtio: no +disk -- virtio-blk has no media]");
      load_bin(fw,  OFF_FW);
      load_bin(dtb, OFF_DTB);
      if ($value$plusargs("initrd=%s", initrd)) load_bin(initrd, OFF_INITRD);
      // a1 (=DTB pointer) is seeded into the RF by rf_shard's +a1= plusarg; reset jumps to OpenSBI.

      reset=1; @(negedge clk); @(negedge clk); reset=0;
      for (c=0; c<ncyc; c=c+1) begin
         @(negedge clk);
         if ((c % 1000000) == 0) $display("[c=%0d pc=%h]", c, dut.imem_addr);
      end
      $display("\n[tb_virtio: %0d cycles done]", ncyc);
      $finish;
   end
endmodule

`default_nettype wire
