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
`ifdef COSIM_MEM_SIZE_LG2
   localparam [63:0] DDR_BYTES = 64'd1 << `COSIM_MEM_SIZE_LG2;
`else
   localparam [63:0] DDR_BYTES = 64'd1 << 31;       // 2 GiB (matches HW / ubuntu.dts memory node)
`endif
   localparam [63:0] OFF_FW = 64'h000_0000, OFF_DTB = 64'h200_0000, OFF_INITRD = 64'h762_b000;

   reg clk=0; always #5 clk=~clk;
   reg reset;
   wire        commit, dmem_wen;
   wire [63:0] dmem_waddr, dmem_wdata;  wire [7:0] dmem_wmask;
   wire        ddr_req, ddr_we;  wire [57:0] ddr_addr;  wire [511:0] ddr_wdata;
   reg  [511:0] ddr_rdata;  reg ddr_ack;
   reg          rx_we;  reg [7:0] rx_data;  wire rx_ready;

   // ---- virtio MMIO passthrough nets (soc_top <-> virtio_mmio) ----
   // 8 KiB window: addr[12] selects blk(+0)/net(+0x1000). No net backend in sim: the net
   // window reads 0 (kernel skips the node); truncating bit 12 aliased net onto blk and
   // enumerated a ghost vdb ("virtio_blk virtio1 ... error -2").
   wire [12:0] virtio_addr;  wire virtio_read, virtio_write;
   wire [31:0] virtio_wdata; wire [3:0] virtio_be;
   wire [31:0] virtio_rdata_comb; wire virtio_irq;
   wire        vio_net = virtio_addr[12];
   // Model the FPGA probe_clk<->ui_clk CDC bridge: a virtio read OR write is a 1-cycle req pulse, the
   // completion returns several cycles later with virtio_rvalid (read: latched read_data; write: a
   // delivery ack -- data don't-care). This stresses soc_top's req/rsp handshake (it must wait for
   // rvalid, not assume a fixed latency) AND the blocking-write ordering that the bridge enforces.
   reg [31:0] virtio_rd_q;  reg virtio_rvalid;  reg [2:0] vio_lat;
   always @(posedge clk) begin
      virtio_rvalid <= 1'b0;
      if (reset) vio_lat <= 3'd0;
      else if (virtio_read || virtio_write) begin virtio_rd_q <= vio_net ? 32'd0 : virtio_rdata_comb; vio_lat <= 3'd3; end
      else if (vio_lat != 3'd0) begin vio_lat <= vio_lat - 3'd1; if (vio_lat == 3'd1) virtio_rvalid <= 1'b1; end
   end

   soc_top #(.RESET_PC(64'h8000_0000)) dut    // reset straight to OpenSBI (a1=DTB via +a1=)
     (.clk(clk), .reset(reset), .commit(commit),
      .dmem_wen(dmem_wen), .dmem_waddr(dmem_waddr), .dmem_wdata(dmem_wdata), .dmem_wmask(dmem_wmask),
      .ddr_req(ddr_req), .ddr_we(ddr_we), .ddr_addr(ddr_addr),
      .ddr_wdata(ddr_wdata), .ddr_rdata(ddr_rdata), .ddr_ack(ddr_ack),
      .uart_rx_we(rx_we), .uart_rx_data(rx_data), .uart_rx_ready(rx_ready),
      .uart_tx_ready(1'b1),
      .virtio_addr(virtio_addr), .virtio_read(virtio_read), .virtio_write(virtio_write),
      .virtio_wdata(virtio_wdata), .virtio_be(virtio_be),
      .virtio_rdata(virtio_rd_q), .virtio_rvalid(virtio_rvalid), .virtio_irq(virtio_irq), .virtio_net_irq(1'b0));

   // ---------------- behavioral DDR (DDR_BYTES, 4-cycle line latency) ----------------
   // 512-bit LINE array (ddr_* port is 64-byte lines), so the element count is DDR_BYTES/64 --
   // stays under Verilator's ~1-billion array-dimension limit even at 2 GiB (a flat byte array
   // overflows it). A line holds bytes little-endian: byte k at bits [k*8 +: 8].
   localparam [63:0] NLINES = DDR_BYTES >> 6;
   localparam [63:0] LBASE  = BASE >> 6;            // DDR base as a line address
   reg [511:0] lram [0:NLINES-1];
   reg d_busy; reg [3:0] d_cnt; reg d_we_q; reg [57:0] d_ad_q; reg [511:0] d_wd_q;
   reg [63:0] line;
   always @(posedge clk) begin
      ddr_ack <= 1'b0;
      if (reset) d_busy<=1'b0;
      else if (!d_busy && ddr_req) begin d_busy<=1'b1; d_cnt<=4'd4; d_we_q<=ddr_we; d_ad_q<=ddr_addr; d_wd_q<=ddr_wdata; end
      else if (d_busy) begin
         if (d_cnt==0) begin
            line = ({6'd0, d_ad_q} - LBASE) & (NLINES-1);   // ddr_addr is the 64-byte line index
            if (d_we_q) lram[line] <= d_wd_q;
            else        ddr_rdata  <= lram[line];
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

   // ---- behavioral always-ready single-beat AXI slave into lram[] (direct DDR = non-coherent) ----
   // ax_*addr[30:0] is already the DDR byte offset: guest_pa = BASE|off, so pa[30:0] = off = pa-BASE.
   // Each beat is 8 bytes (axsize=3, 8-aligned) so it lands within one 64-byte line: line = off>>6,
   // byte position bp = (off&63). lram stores byte k at bit k*8, so the 8 bytes are lram[line][bp*8 +: 64].
   reg axi_rvalid, axi_bvalid;  reg [63:0] axi_rdata;  integer ka;
   reg [63:0] ar_line, aw_line;  reg [9:0] ar_bp, aw_bp;
   assign ax_arready = 1'b1; assign ax_awready = 1'b1; assign ax_wready = 1'b1;
   assign ax_rvalid = axi_rvalid; assign ax_rdata = axi_rdata; assign ax_rresp = 2'd0;
   assign ax_rlast = 1'b1; assign ax_rid = 3'd1;
   assign ax_bvalid = axi_bvalid; assign ax_bresp = 2'd0; assign ax_bid = 3'd1;
   always @(posedge clk) begin
      if (reset) begin axi_rvalid<=1'b0; axi_bvalid<=1'b0; end
      else begin
         // read channel
         if (ax_arvalid && ax_arready && !axi_rvalid) begin
            ar_line = ((ax_araddr & (DDR_BYTES-1)) >> 6) & (NLINES-1);
            ar_bp   = (ax_araddr & 6'h3f) << 3;        // bit position of byte (off&63)
            axi_rdata <= lram[ar_line][ar_bp +: 64];
            axi_rvalid <= 1'b1;
         end else if (axi_rvalid && ax_rready) axi_rvalid <= 1'b0;
         // write channel (address + data arrive together for this single-beat master)
         if (ax_awvalid && ax_awready && ax_wvalid && ax_wready && !axi_bvalid) begin
            aw_line = ((ax_awaddr & (DDR_BYTES-1)) >> 6) & (NLINES-1);
            aw_bp   = (ax_awaddr & 6'h3f) << 3;
            for (ka=0;ka<8;ka=ka+1) if (ax_wstrb[ka]) lram[aw_line][aw_bp + ka*8 +: 8] <= ax_wdata[ka*8 +: 8];
            axi_bvalid <= 1'b1;
         end else if (axi_bvalid && ax_bready) axi_bvalid <= 1'b0;
      end
   end

   // ---- DPI: file-backed SpiSdCard clocked by the SD-SPI pins each cycle ----
   import "DPI-C" function void sd_attach(input string path);
   import "DPI-C" function int  sd_clock(input int sck, input int cs_n, input int mosi);
   import "DPI-C" function void sd_readonly();
   always @(posedge clk) blk_miso <= sd_clock({31'd0, blk_sck}, {31'd0, blk_cs_n}, {31'd0, blk_mosi})>0 ? 1'b1 : 1'b0;

   // ---------------- boot image load (line array; reused from tb_cosim_linux) ----------------
   // $fread fills each 512-bit element MSB-first, so reverse the 64 bytes of every loaded line
   // back to the little-endian byte order the ddr_* port + AXI slave use.
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
         $display("[tb_virtio: loaded %0d bytes @ DDR+%h (%0d lines)]", n, off, nl);
         $display("[tb_virtio:   head=%h tail=%h]", lram[sl][63:0], lram[sl+nl-1][63:0]);
      end
   endtask

   // ---- virtio-blk / SD diagnosis taps ----
   reg [63:0] c;
   reg        sderr_q, vblk_done_q;
   reg [5:0]  vbstate_q;
   always @(posedge clk) begin
      if (reset) begin sderr_q <= 1'b0; vbstate_q <= 6'd0; end
      else begin
         // virtio_blk verdict on entry to S_WRITE_STATUS (state 31): status 0=OK/1=IOERR/2=UNSUPP
         vbstate_q <= u_vblk.state;
         if (u_vblk.state == 6'd31 && vbstate_q != 6'd31)
            $display("[VBLK c=%0d STATUS=%0d type=%0d sec=%0d secleft=%0d sderr=%b dmaerr=%b]",
                     c, u_vblk.status_byte, u_vblk.req_type, u_vblk.req_sector,
                     u_vblk.sectors_left, u_vblk.sd_error, u_vblk.dma_rsp_error);
         // sd_error edge: capture which SPI step / sector failed
         sderr_q <= u_vblk.sd_error;
         if (u_vblk.sd_error & ~sderr_q)
            $display("[SDERR c=%0d vblkstate=%0d blk_sector=%0d secleft=%0d spistate=%0d]",
                     c, u_vblk.state, u_vblk.blk_sector, u_vblk.sectors_left, u_vblk.sd.dbg_state);
`ifdef VIRTIO_MMIO_TRACE
         // ---- kick/IRQ/MMIO ordering trace (windowed): splits "kernel never kicked" vs
         // "device never completed" vs "completion never reached the kernel" on a wedge.
         if (c > `VIRTIO_MMIO_T0) begin
            if (dut.virtio_write)
               $display("[VMMIO c=%0d WR addr=%h data=%h be=%b]", c, dut.virtio_addr, dut.virtio_wdata, dut.virtio_be);
            if (dut.virtio_read)
               $display("[VMMIO c=%0d RD addr=%h]", c, dut.virtio_addr);
            if (virtio_irq_q != u_vmmio.irq)
               $display("[VMMIO c=%0d IRQ %b->%b]", c, virtio_irq_q, u_vmmio.irq);
            // PLIC side: claim reads / complete writes / seip + src-11 gateway state
            if (dut.dmem_ren & dut.is_plic_r)
               $display("[PLIC c=%0d RD a=%h rdata=%h seip=%b pend11=%b insvc11=%b]", c,
                        dut.dmem_raddr[23:0], dut.plic_rdata, dut.plic_seip,
                        dut.u_plic.pending[11], dut.u_plic.in_service[11]);
            if (dut.dmem_wen & dut.is_plic_w)
               $display("[PLIC c=%0d WR a=%h wdata=%h seip=%b pend11=%b insvc11=%b]", c,
                        dut.dmem_waddr[23:0], dut.dmem_wdata, dut.plic_seip,
                        dut.u_plic.pending[11], dut.u_plic.in_service[11]);
         end
`endif
      end
   end
`ifdef VIRTIO_MMIO_TRACE
   reg virtio_irq_q; initial virtio_irq_q = 1'b0;
   always @(posedge clk) virtio_irq_q <= u_vmmio.irq;
`endif

   // ---- console-silence watchdog (ubuntu "Hostname set" wedge instrumentation) ----
   // Once the UART has been quiet for WDOG_QUIET cycles, dump the full interrupt/timer
   // state every 10M cycles. One dump discriminates the three wedge classes:
   //   (a) csr_irq_v=1 but inject blocked -> a gate term below is latched high (name it);
   //   (b) irq_v=0 with mtime>=stimecmp & STIE & deleg -> pending/enable plumbing lost it;
   //   (c) no armed timer at all -> software state already wedged earlier (memory bug).
   localparam [63:0] WDOG_QUIET = 64'd50_000_000;
   reg [63:0] last_tx_c; initial last_tx_c = 0;
   always @(posedge clk) if (dut.uart_tx_valid) last_tx_c <= c;
   // Distinct U-mode fetch PAGES seen at dump instants (imem_addr is a PHYSICAL address):
   // at the wedge these pages are hexdumped so the spinning user code can be disassembled
   // and matched byte-for-byte against the initrd cpio's binaries (-> binary + symbol).
   reg [63:0] wdog_pg [0:7]; integer wdog_npg; initial wdog_npg = 0;
   integer wpi; reg wpi_hit;
   task wdog_dump;
      begin
         if (dut.core.eb.u_csr.priv == 2'd0 && wdog_npg < 8) begin
            wpi_hit = 0;
            for (wpi = 0; wpi < wdog_npg; wpi = wpi + 1)
               if (wdog_pg[wpi] == (dut.imem_addr >> 12)) wpi_hit = 1;
            if (!wpi_hit) begin wdog_pg[wdog_npg] = dut.imem_addr >> 12; wdog_npg = wdog_npg + 1; end
         end
         $display("[WDOG c=%0d silent=%0dM pc=%h priv=%0d irq_v=%b cause=%0d infl=%b gate: rpl=%b devld=%b piflt=%b dflt=%b ill=%b red=%b roll=%b]",
                  c, (c - last_tx_c)/1000000, dut.imem_addr,
                  dut.core.eb.u_csr.priv, dut.core.csr_irq_v, dut.core.csr_irq_cause,
                  dut.core.inject_inflight, dut.core.replay_v, dut.core.devld_solo_v,
                  dut.core.pend_iflt, dut.core.lsu_dfault_v, dut.core.ill_v,
                  dut.core.eb_redirect, dut.core.roll_v);
         $display("[WDOG mip=%h mie=%h mideleg=%h mstatus=%h mtime=%h mtimecmp=%h stimecmp=%h stce=%b satp=%h]",
                  dut.core.eb.u_csr.eff_mip, dut.core.eb.u_csr.mie, dut.core.eb.u_csr.mideleg,
                  dut.core.eb.u_csr.mstatus, dut.u_clint.mtime, dut.u_clint.mtimecmp,
                  dut.core.eb.u_csr.stimecmp, dut.core.eb.u_csr.menvcfg[63],
                  dut.core.eb.u_csr.satp);
         // last S-trap: scause=8 + sepc=<user PC> = the syscall sites feeding the kernel copies
         $display("[WDOG scause=%h sepc=%h stval=%h]",
                  dut.core.eb.u_csr.scause, dut.core.eb.u_csr.sepc, dut.core.eb.u_csr.stval);
         // retire progress + LSU/AMO/store-buffer stuck-state: discriminates "loops retire
         // normally" vs "AMO stalls mid-FSM" vs "AMO never issues (operand never ready)"
         $display("[WDOG instret=%0d ast=%0d amo_v=%b amo_pend=%b p_v=%b sb_v=%b%b%b%b%b%b%b%b sb_cmt=%b%b%b%b%b%b%b%b dr_v=%b]",
                  dut.core.eb.u_csr.minstret, dut.core.u_lsu.ast, dut.core.amo_v,
                  dut.core.u_lsu.amo_pend, dut.core.u_lsu.p_v,
                  dut.core.u_lsu.sb_v[0], dut.core.u_lsu.sb_v[1], dut.core.u_lsu.sb_v[2],
                  dut.core.u_lsu.sb_v[3], dut.core.u_lsu.sb_v[4], dut.core.u_lsu.sb_v[5],
                  dut.core.u_lsu.sb_v[6], dut.core.u_lsu.sb_v[7],
                  dut.core.u_lsu.sb_cmt[0], dut.core.u_lsu.sb_cmt[1], dut.core.u_lsu.sb_cmt[2],
                  dut.core.u_lsu.sb_cmt[3], dut.core.u_lsu.sb_cmt[4], dut.core.u_lsu.sb_cmt[5],
                  dut.core.u_lsu.sb_cmt[6], dut.core.u_lsu.sb_cmt[7],
                  dut.core.u_lsu.dr_v);
         wdog_sched;
         $fflush;
      end
   endtask
   // scheduler-entry dump: names WHY a waiting op can't issue (sched_shard eligibility =
   // v & r1 & r2 & r3 & (~ser | ck==committed)). At the wedge this discriminates
   // operand-never-ready (a dropped wake/writeback) vs never-oldest (an older op never
   // completing) vs the amo_gap dispatch freeze.
   task wdog_sched;
      integer e;
      begin
         $display("[WSCHED amo_gap=%b committed=%0d cur=%0d]",
                  dut.core.amo_gap, dut.core.cc_committed, dut.core.cur);
         for (e = 0; e < 16; e = e + 1) begin
            if (dut.core.sb.lane[0].sh.v[e])
               $display("[WSCHED s0e%0d seq=%0d ck=%0d r=%b%b%b ser=%b ps1=%0d ps2=%0d]",
                        e, dut.core.sb.lane[0].sh.sq[e], dut.core.sb.lane[0].sh.ck[e],
                        dut.core.sb.lane[0].sh.r1[e], dut.core.sb.lane[0].sh.r2[e],
                        dut.core.sb.lane[0].sh.r3[e], dut.core.sb.lane[0].sh.py[e][156],
                        dut.core.sb.lane[0].sh.s1[e], dut.core.sb.lane[0].sh.s2[e]);
`ifndef PROBE_IW_1
            // lane[1] exists only at IW>=2; referencing it hierarchically at IW=1 is an
            // elaboration error, so run-virtio.sh defines PROBE_IW_1 for the 1-wide build.
            if (dut.core.sb.lane[1].sh.v[e])
               $display("[WSCHED s1e%0d seq=%0d ck=%0d r=%b%b%b ser=%b ps1=%0d ps2=%0d]",
                        e, dut.core.sb.lane[1].sh.sq[e], dut.core.sb.lane[1].sh.ck[e],
                        dut.core.sb.lane[1].sh.r1[e], dut.core.sb.lane[1].sh.r2[e],
                        dut.core.sb.lane[1].sh.r3[e], dut.core.sb.lane[1].sh.py[e][156],
                        dut.core.sb.lane[1].sh.s1[e], dut.core.sb.lane[1].sh.s2[e]);
`endif
         end
         $fflush;
      end
   endtask
   // ---- IRQs-off-forever detector (run-7 terminal state: kernel inside the S-timer ISR,
   // SIE=0 continuously from c~10.8B to the 22.3B stop, satp/stimecmp frozen, STIP pending,
   // instret advancing -- a livelock over corrupted timer state). No legitimate S-mode
   // IRQs-off section lasts 100M cycles: when the streak hits that, dump a fine PC ring
   // (512 consecutive fetches = the loop body), a coarse ring (every 64K cycles = the
   // macro path in), full state, and stop.
   reg [63:0] pcring  [0:511];  reg [8:0]  pcring_w;  initial pcring_w  = 9'd0;
   reg [63:0] pcringc [0:511];  reg [8:0]  pcringc_w; initial pcringc_w = 9'd0;
   reg [63:0] sie_off_streak;   initial sie_off_streak = 64'd0;
   reg        sie_trig;         initial sie_trig = 1'b0;
   integer wri;
   always @(posedge clk) begin
      pcring[pcring_w] <= dut.imem_addr; pcring_w <= pcring_w + 9'd1;
      if (c[15:0] == 16'd0) begin pcringc[pcringc_w] <= dut.imem_addr; pcringc_w <= pcringc_w + 9'd1; end
      // streak = cycles with S-interrupts NOT deliverable: resets on (S && SIE=1) or U-mode
      // (S-ints deliver from U regardless of SIE); M-mode excursions (OpenSBI misaligned
      // fixups inside the wedge loop) keep counting -- run-8's priv==S-only condition reset
      // on those and never fired.
      if ((dut.core.eb.u_csr.priv == 2'd1 && dut.core.eb.u_csr.mstatus[1])
          || dut.core.eb.u_csr.priv == 2'd0)
           sie_off_streak <= 64'd0;
      else sie_off_streak <= sie_off_streak + 64'd1;
      // belt: masked-streak trigger; suspenders: hard cycle trigger mid-wedge (the wedge is
      // proven deterministic: run 8 replayed run 7 bit-for-bit).
      // hard cycle trigger only in checkpoint-server mode (children): a standalone
      // validation boot must be free to run past 11.5B (it killed the first healthy
      // nohz=off run at the wedge-autopsy cycle).
      if (!sie_trig && ((sie_off_streak > 64'd100_000_000 && c > 64'd4_000_000_000)
                        || (c == 64'd11_500_000_000 && ckpt_c != 64'd0))) begin
         sie_trig <= 1'b1;
         $display("[SIEWEDGE c=%0d pc=%h streak=%0d]", c, dut.imem_addr, sie_off_streak);
         for (wri = 0; wri < 512; wri = wri + 1)
            $display("[SIERING %0d pc=%h]", wri, pcring[(pcring_w + wri[8:0]) & 9'h1ff]);
         for (wri = 0; wri < 512; wri = wri + 1)
            $display("[SIERINGC %0d pc=%h]", wri, pcringc[(pcringc_w + wri[8:0]) & 9'h1ff]);
         wdog_dump;
         $fflush;
         $finish;
      end
   end
   // hexdump one 4K page from the behavioral DDR (64 lines of 64B; bytes little-endian
   // within a line: byte k at bits [k*8 +: 8])
   task wdog_pgdump(input [63:0] pg);
      integer i; reg [63:0] pa;
      begin
         for (i = 0; i < 64; i = i + 1) begin
            pa = (pg << 12) + i*64;
            if ((pa >> 6) >= LBASE && (pa >> 6) < LBASE + NLINES)
               $display("[WMEM %h %h]", pa, lram[(pa >> 6) - LBASE]);
         end
         $fflush;
      end
   endtask

   // ---- fork-based checkpoint server (ckpt_dpi.cpp) ----
   // +ckpt=<cycle> turns the run into a reusable time machine: at that cycle the sim
   // blocks on a command file; each command forks a child that inherits the FULL sim
   // state, enables an optional store write-watch, runs <extra> more cycles into its own
   // log, and exits -- experiments then cost minutes, not the 5.5h replay from reset.
   import "DPI-C" function int ckpt_wait_cmd(input string path,
      output longint wlo, output longint whi, output longint wcyc, output longint wsval);
   reg [63:0] ckpt_c;  string ckpt_cmd;
   longint    ck_wlo, ck_whi, ck_wcyc, ck_sval;
   reg [63:0] watch_lo, watch_hi;
   initial begin
      watch_lo = 0; watch_hi = 0;
      // +watchlo/+watchhi (hex PA range): CPU-store + device-DMA watch from cycle 0
      // (the ckpt command channel below can also set it mid-run).
      if (!$value$plusargs("watchlo=%h", watch_lo)) watch_lo = 0;
      if (!$value$plusargs("watchhi=%h", watch_hi)) watch_hi = 0;
   end
   // +watch_val=<hex>: from CYCLE 0, log any full-width store of this 64-bit value
   // ANYWHERE (corruptor fingerprint hunt -- run-7/childB: the poison expiry
   // 0x189e8ae282285be3 predates the 10.0B checkpoint, so the warm-up run itself
   // must hunt the store that plants it).
   reg [63:0] watch_val; initial watch_val = 64'd0;
   always @(posedge clk) begin
      if (watch_hi != 64'd0 && dmem_wen && dmem_waddr >= watch_lo && dmem_waddr < watch_hi) begin
         $display("[WATCH c=%0d pa=%h data=%h mask=%h pc=%h priv=%0d]", c, dmem_waddr,
                  dmem_wdata, dmem_wmask, dut.imem_addr, dut.core.eb.u_csr.priv);
         $fflush;
      end
      // device-DMA writes into the watch range (DMA bypasses the D$: a hit here on a
      // page the CPU also caches is an incoherence/misdirected-DMA smoking gun)
      if (watch_hi != 64'd0 && ax_awvalid && ax_awready && ax_wvalid && ax_wready
          && ((64'h80000000 | (ax_awaddr & (DDR_BYTES-1))) >= watch_lo)
          && ((64'h80000000 | (ax_awaddr & (DDR_BYTES-1))) <  watch_hi)) begin
         $display("[WATCH-DMA c=%0d pa=%h data=%h strb=%h]", c,
                  64'h80000000 | (ax_awaddr & (DDR_BYTES-1)), ax_wdata, ax_wstrb);
         $fflush;
      end
      if (watch_val != 64'd0 && dmem_wen && dmem_wdata == watch_val) begin
         $display("[VWATCH c=%0d pa=%h data=%h mask=%h pc=%h priv=%0d satp=%h]", c, dmem_waddr,
                  dmem_wdata, dmem_wmask, dut.imem_addr, dut.core.eb.u_csr.priv,
                  dut.core.eb.u_csr.satp);
         $fflush;
      end
   end
   // child memory scan: locate every 8-aligned copy of a 64-bit value in DDR
   task ckpt_scan(input [63:0] val);
      reg [63:0] li; integer lk; reg [511:0] line_v;
      begin
         $display("[SCAN for %h]", val);
         for (li = 0; li < NLINES; li = li + 1) begin
            line_v = lram[li];
            for (lk = 0; lk < 8; lk = lk + 1)
               if (line_v[lk*64 +: 64] == val)
                  $display("[SCANHIT pa=%h]", ((li + LBASE) << 6) + lk*8);
         end
         $display("[SCAN done]"); $fflush;
      end
   endtask

   reg [8*256-1:0] fw, dtb, initrd, disk;
   reg [63:0] ncyc, off_initrd;
   initial begin
      rx_we=0; rx_data=0; ncyc=200000000; blk_miso=1'b1;
      if (!$value$plusargs("fw=%s", fw))         begin $display("FATAL: +fw"); $finish; end
      if (!$value$plusargs("dtb=%s", dtb))       begin $display("FATAL: +dtb"); $finish; end
      if ($value$plusargs("cycles=%d", ncyc)) ;
      if (ncyc == 0) ncyc = ~64'd0;   // +cycles=0 = no cap
      ckpt_c = 64'd0;
      if ($value$plusargs("ckpt=%d", ckpt_c)) ;
      if (!$value$plusargs("ckpt_cmd=%s", ckpt_cmd)) ckpt_cmd = "/tmp/probe-ckpt-cmd";
      if (ckpt_c != 0) $display("[tb_virtio: checkpoint server armed at c=%0d]", ckpt_c);
      if ($value$plusargs("watch_val=%h", watch_val))
         $display("[tb_virtio: value-watch armed for %h]", watch_val);
      // Echo the EFFECTIVE cap: a run must prove what it consumed, not what was passed.
      // $fflush: stdout is FULLY buffered when redirected to a file, so without explicit
      // flushes a fresh run shows nothing (only the DPI's stderr) for minutes.
      $display("[tb_virtio: cycle cap = %0d]", ncyc);
      $fflush;
      if ($value$plusargs("disk=%s", disk)) sd_attach(disk);
      if ($test$plusargs("disk_ro")) begin sd_readonly(); $display("[sd: SNAPSHOT mode -- image writes stay in RAM]"); end
      else $display("[tb_virtio: no +disk -- virtio-blk has no media]");
      load_bin(fw,  OFF_FW);
      load_bin(dtb, OFF_DTB);
      off_initrd = OFF_INITRD;
      if ($value$plusargs("initrd_off=%h", off_initrd)) ;
      if ($value$plusargs("initrd=%s", initrd)) load_bin(initrd, off_initrd);
      // a1 (=DTB pointer) is seeded into the RF by rf_shard's +a1= plusarg; reset jumps to OpenSBI.

      reset=1; @(negedge clk); @(negedge clk); reset=0;
      for (c=0; c<ncyc; c=c+1) begin
         @(negedge clk);
         if (ckpt_c != 0 && c == ckpt_c) begin
            $display("[ckpt: serving commands from %0s]", ckpt_cmd); $fflush;
            if (ckpt_wait_cmd(ckpt_cmd, ck_wlo, ck_whi, ck_wcyc, ck_sval) == 0) begin
               $display("[ckpt: quit]"); $finish;
            end
            // child: apply the experiment and bound its run
            watch_lo = ck_wlo; watch_hi = ck_whi;
            ncyc = c + ck_wcyc;
            $display("[ckpt-child c=%0d watch=%h..%h run-to=%0d scan=%h]", c, watch_lo, watch_hi, ncyc, ck_sval);
            $fflush;
            if (ck_sval != 0) ckpt_scan(ck_sval);
         end
         if ((c % 1000000) == 0) begin
            $display("[c=%0d pc=%h]", c, dut.imem_addr);
            $fflush;   // keep file-redirected logs live (also drains buffered UART text)
            if ((c % 10000000) == 0 && c - last_tx_c > WDOG_QUIET) wdog_dump;
            // Unambiguous wedge: longest legitimate quiet stretch is the initramfs unpack
            // (~6-8B cycles). 12B of silence = frozen; stop so the run self-terminates.
            if (c - last_tx_c > 64'd12_000_000_000) begin
               $display("[WDOG: %0d cycles of console silence -- declaring wedge, stopping]", c - last_tx_c);
               for (wpi = 0; wpi < wdog_npg; wpi = wpi + 1) begin
                  $display("[WMEM-PAGE %h]", wdog_pg[wpi] << 12);
                  wdog_pgdump(wdog_pg[wpi]);
               end
               $finish;
            end
         end
      end
      $display("\n[tb_virtio: %0d cycles done]", ncyc);
      $finish;
   end

`ifdef STRAND
   // Sim analogue of the FPGA mtvec-stranding ILA trigger. OpenSBI installs
   // __sbi_expected_trap (0x80000738) only for the five instructions of its MPRV accessor,
   // so mtvec sitting there is a restore that never executed -- after which every S-mode
   // ecall is swallowed. Keep a ring of the system ops that actually reached the CSR unit
   // (exec_bundle's oldest-select port), so the dump says which of the window's four
   // executed:  e828 csrrw mtvec / e82c csrrs mstatus / e834 csrw mstatus / e838 csrw mtvec.
   localparam [63:0] SBI_PROBE_TRAP = 64'h0000_0000_8000_0738;
   reg [63:0] so_pc [0:63];
   reg [63:0] so_c  [0:63];
   reg [11:0] so_ad [0:63];
   reg [2:0]  so_fn [0:63];
   reg        so_cs [0:63];
   reg [5:0]  so_wp;      initial so_wp = 0;
   reg [31:0] strand_cnt; initial strand_cnt = 0;
   reg        strand_done;initial strand_done = 0;
   integer si, sj;
   always @(posedge clk) if (!reset) begin
      if (dut.core.eb.sv) begin
         so_pc[so_wp] <= dut.core.eb.s_pc;    so_ad[so_wp] <= dut.core.eb.s_addr;
         so_fn[so_wp] <= dut.core.eb.s_func;  so_cs[so_wp] <= dut.core.eb.s_iscsr;
         so_c [so_wp] <= c;                   so_wp        <= so_wp + 6'd1;
      end
      strand_cnt <= (dut.core.eb.u_csr.mtvec == SBI_PROBE_TRAP) ? strand_cnt + 32'd1 : 32'd0;
      if (strand_cnt > 32'd200000 && !strand_done) begin
         strand_done <= 1'b1;
         $display("[STRAND c=%0d mtvec parked at __sbi_expected_trap for %0d cycles]", c, strand_cnt);
         for (si = 0; si < 64; si = si + 1) begin
            sj = (so_wp + si) % 64;                       // oldest first
            $display("[STRAND-OP c=%0d pc=%h %s addr=%h func=%0d]",
                     so_c[sj], so_pc[sj], so_cs[sj] ? "csr" : "sys", so_ad[sj], so_fn[sj]);
         end
         wdog_dump;
      end
   end
`endif
endmodule

`default_nettype wire
