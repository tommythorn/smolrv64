`timescale 1ns/1ps
`default_nettype none

// Linux-boot cosim harness: soc_top reset DIRECTLY to OpenSBI (0x8000_0000) with
// a1=DTB seeded into the RF (+a1=, via rf_shard's PROBE_COSIM seed) -- the monitor
// is bypassed so the DUT and simmerv start at the same PC (mirrors the SmolRV64
// linux-cosim recipe). The cosim retire stream + lockstep lives in backend_top
// (PROBE_COSIM) / probe_cosim.cpp; this TB just loads DDR, clocks, and times out.
//   +fw=... +dtb=... [+initrd=...] [+cycles=N]   (C side reads the same plusargs)
module tb;
   localparam [63:0] BASE = 64'h8000_0000;
`ifdef COSIM_MEM_SIZE_LG2
   localparam [63:0] DDR_BYTES = 64'd1 << `COSIM_MEM_SIZE_LG2;   // 64-bit: LG2>=31 (>=2GiB) ok
`else
   localparam [63:0] DDR_BYTES = 64'd1 << 28;       // 256 MiB (linux default)
`endif
   // DTB/initrd load offsets default to the linux workload but are overridable per
   // workload via +dtb_off=/+initrd_off= (gb5/gb6 place them much higher); fw is @0.
   localparam [63:0] OFF_FW = 64'h000_0000, OFF_DTB_DEF = 64'h200_0000, OFF_INITRD_DEF = 64'h762_b000;
   reg [63:0] off_dtb, off_initrd;

   reg clk=0; always #5 clk=~clk;
   reg reset;
   wire        commit, dmem_wen;
   wire [63:0] dmem_waddr, dmem_wdata;  wire [7:0] dmem_wmask;
   wire        ddr_req, ddr_we;  wire [57:0] ddr_addr;  wire [511:0] ddr_wdata;
   reg  [511:0] ddr_rdata;  reg ddr_ack;
   wire        rx_ready;

   // Model the hardware's 3 Mbps serializer instead of an instant drain: a 10-bit
   // frame at 66.67MHz is ~222 probe_clk cycles, so uart_tx_ready stays low that
   // long after each accepted byte. THR then stays full between characters and
   // THRE interrupts land while the kernel is BUSY -- the interrupt pacing the
   // FPGA sees and a tied-high ready never exercises. (UART is hardwired 3Mbps.)
   localparam TX_DRAIN = 222;
   reg  [7:0]  txcnt = 8'd0;
   wire        uart_tx_v;
   wire        uart_tx_rdy = (txcnt == 8'd0);
   always @(posedge clk)
      if (reset)                     txcnt <= 8'd0;
      else if (uart_tx_v & uart_tx_rdy) txcnt <= TX_DRAIN[7:0];
      else if (txcnt != 8'd0)        txcnt <= txcnt - 8'd1;

   // ---- virtio MMIO passthrough nets + FPGA-CDC-shaped latency (from tb_virtio) ----
   wire [11:0] virtio_addr;  wire virtio_read, virtio_write;
   wire [31:0] virtio_wdata; wire [3:0] virtio_be;
   wire [31:0] virtio_rdata_comb; wire virtio_irq;
   reg [31:0] virtio_rd_q;  reg virtio_rvalid;  reg [2:0] vio_lat;
   always @(posedge clk) begin
      virtio_rvalid <= 1'b0;
      if (reset) vio_lat <= 3'd0;
      else if (virtio_read || virtio_write) begin virtio_rd_q <= virtio_rdata_comb; vio_lat <= 3'd3; end
      else if (vio_lat != 3'd0) begin vio_lat <= vio_lat - 3'd1; if (vio_lat == 3'd1) virtio_rvalid <= 1'b1; end
   end

   soc_top #(.RESET_PC(64'h8000_0000)) dut
     (.clk(clk), .reset(reset), .commit(commit),
      .dmem_wen(dmem_wen), .dmem_waddr(dmem_waddr), .dmem_wdata(dmem_wdata), .dmem_wmask(dmem_wmask),
      .ddr_req(ddr_req), .ddr_we(ddr_we), .ddr_addr(ddr_addr),
      .ddr_wdata(ddr_wdata), .ddr_rdata(ddr_rdata), .ddr_ack(ddr_ack),
      .uart_rx_we(1'b0), .uart_rx_data(8'd0), .uart_rx_ready(rx_ready),
      .uart_tx_valid(uart_tx_v), .uart_tx_ready(uart_tx_rdy),
      .virtio_addr(virtio_addr), .virtio_read(virtio_read), .virtio_write(virtio_write),
      .virtio_wdata(virtio_wdata), .virtio_be(virtio_be),
      .virtio_rdata(virtio_rd_q), .virtio_rvalid(virtio_rvalid), .virtio_irq(virtio_irq));

   // behavioral DDR (DDR_BYTES, 4-cycle line latency). Modeled as a 512-bit LINE array (the
   // ddr_* port is 64-byte lines), so the element count is DDR_BYTES/64 -- which stays under
   // the ~1-billion-element array-dimension limit even at 2 GiB (a flat byte array overflows
   // it). A line holds bytes little-endian: byte k at bits [k*8 +: 8], matching the port.
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

   // ==================== virtio-blk subsystem (from tb_virtio; oracle-aware) ====================
   // Device DMA lands in lram[] AND is mirrored into simmerv via cosim_dma_write (DPI-order
   // exact: probe_cosim drains the queue between the retires the write chronologically
   // separates). MMIO loads on the REF side adopt DUT values (simmerv armed-load path);
   // REF device-store side effects are swallowed (simmerv cosim_inert_devstore).
   wire        v_notify_pulse;   wire [31:0] v_notify_value;
   wire        v_used_irq;       wire [31:0] v_capacity;
   wire  [7:0] v_dev_status;
   wire [31:0] v_q0_num;  wire v_q0_ready;
   wire [63:0] v_q0_desc, v_q0_driver, v_q0_device;

   virtio_mmio #(.DEVICE_ID(32'd2), .QUEUE_NUM_MAX(32'd8)) u_vmmio
     (.clock(clk), .reset(reset),
      .address(virtio_addr), .read(virtio_read), .read_data(virtio_rdata_comb),
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

   // behavioral always-ready single-beat AXI slave into lram[] (non-coherent DMA);
   // ax_*addr[30:0] is the DDR byte offset (guest_pa - BASE).
   reg axi_rvalid, axi_bvalid;  reg [63:0] axi_rdata;  integer ka;
   reg [63:0] ar_line, aw_line;  reg [9:0] ar_bp, aw_bp;
   assign ax_arready = 1'b1; assign ax_awready = 1'b1; assign ax_wready = 1'b1;
   assign ax_rvalid = axi_rvalid; assign ax_rdata = axi_rdata; assign ax_rresp = 2'd0;
   assign ax_rlast = 1'b1; assign ax_rid = 3'd1;
   assign ax_bvalid = axi_bvalid; assign ax_bresp = 2'd0; assign ax_bid = 3'd1;
`ifdef PROBE_COSIM
   import "DPI-C" function void cosim_dma_write(input longint off, input longint data, input byte strb);
`endif
   always @(posedge clk) begin
      if (reset) begin axi_rvalid<=1'b0; axi_bvalid<=1'b0; end
      else begin
         if (ax_arvalid && ax_arready && !axi_rvalid) begin
            ar_line = ((ax_araddr & (DDR_BYTES-1)) >> 6) & (NLINES-1);
            ar_bp   = (ax_araddr & 6'h3f) << 3;
            axi_rdata <= lram[ar_line][ar_bp +: 64];
            axi_rvalid <= 1'b1;
         end else if (axi_rvalid && ax_rready) axi_rvalid <= 1'b0;
         if (ax_awvalid && ax_awready && ax_wvalid && ax_wready && !axi_bvalid) begin
            aw_line = ((ax_awaddr & (DDR_BYTES-1)) >> 6) & (NLINES-1);
            aw_bp   = (ax_awaddr & 6'h3f) << 3;
            for (ka=0;ka<8;ka=ka+1) if (ax_wstrb[ka]) lram[aw_line][aw_bp + ka*8 +: 8] <= ax_wdata[ka*8 +: 8];
`ifdef PROBE_COSIM
            cosim_dma_write({33'd0, ax_awaddr}, ax_wdata, ax_wstrb);   // mirror into simmerv
`endif
            axi_bvalid <= 1'b1;
         end else if (axi_bvalid && ax_bready) axi_bvalid <= 1'b0;
      end
   end

   // ---- DPI: file-backed SpiSdCard clocked by the SD-SPI pins each cycle ----
   import "DPI-C" function void sd_attach(input string path);
   import "DPI-C" function int  sd_clock(input int sck, input int cs_n, input int mosi);
   always @(posedge clk) blk_miso <= sd_clock({31'd0, blk_sck}, {31'd0, blk_cs_n}, {31'd0, blk_mosi})>0 ? 1'b1 : 1'b0;

   // virtio-blk request verdicts (entry to S_WRITE_STATUS: 0=OK/1=IOERR/2=UNSUPP)
   reg [5:0] vbstate_q;
   always @(posedge clk) begin
      if (reset) vbstate_q <= 6'd0;
      else begin
         vbstate_q <= u_vblk.state;
         if (u_vblk.state == 6'd31 && vbstate_q != 6'd31)
            $display("[VBLK c=%0d STATUS=%0d type=%0d sec=%0d secleft=%0d sderr=%b dmaerr=%b]",
                     c, u_vblk.status_byte, u_vblk.req_type, u_vblk.req_sector,
                     u_vblk.sectors_left, u_vblk.sd_error, u_vblk.dma_rsp_error);
      end
   end

   // Load a raw image at byte offset `off` (64-byte aligned for all workloads). $fread fills
   // each 512-bit element MSB-first, so reverse the 64 bytes of every loaded line back to the
   // little-endian byte order the ddr_* port (and the byte-array model it replaced) uses.
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
         $display("[cosim-linux: loaded %0d bytes @ DDR+%h (%0d lines)]", n, off, nl);
      end
   endtask

   reg [8*256-1:0] fw, dtb, initrd, disk;
   integer b2;
   reg [63:0] ncyc, c;        // 64-bit: cosim runs (gb5/sha256) exceed 2^32 cycles
`ifdef PROBE_COSIM
   import "DPI-C" function void probe_dump_ring(input longint fetch_pc);
   reg [31:0] wedge_cnt;      // cycles without a commit (stuck-pipeline detector)
`endif
   initial begin
      ncyc = 200000000;
`ifdef PROBE_COSIM
      wedge_cnt = 0;
`endif
      if (!$value$plusargs("fw=%s", fw))  begin $display("FATAL: +fw");  $finish; end
      if (!$value$plusargs("dtb=%s", dtb)) begin $display("FATAL: +dtb"); $finish; end
      if ($value$plusargs("cycles=%d", ncyc)) ;
      off_dtb = OFF_DTB_DEF; off_initrd = OFF_INITRD_DEF;
      if ($value$plusargs("dtb_off=%h",    off_dtb))    ;
      if ($value$plusargs("initrd_off=%h", off_initrd)) ;
      load_bin(fw,  OFF_FW);
      load_bin(dtb, off_dtb);
      if ($value$plusargs("initrd=%s", initrd)) load_bin(initrd, off_initrd);
      if ($value$plusargs("disk=%s", disk)) sd_attach(disk);   // virtio-blk media (ubuntu)

      // +cycles=0 (or CYC=0) runs UNBOUNDED -- stop only on a cosim divergence (the C
      // harness abort()s) or an external interrupt. Any nonzero value is a hard cycle cap.
      reset=1; @(negedge clk); @(negedge clk); reset=0;
      for (c=0; (ncyc==0) || (c<ncyc); c=c+1) begin
         @(negedge clk);
         if ((c % 1000000) == 0) $display("[c=%0d]", c);
`ifdef PROBE_COSIM
         // stuck-pipeline watchdog: 200k cycles without a single commit is a wedge
         // (wild redirect / interlock deadlock -> no mismatch ever fires). Keyed on
         // commit, NOT on the fetch PC: with the branch predictor a hot self-targeting
         // loop bundle (e.g. the kernel's BSS-clear sd/addi/bltu) legitimately parks
         // the fetch PC on one address for millions of cycles while retiring fine.
         if (dut.core.cc_commit) wedge_cnt = 0; else wedge_cnt = wedge_cnt + 1;
         if (wedge_cnt == 32'd200000) begin
            $display("COSIM-LINUX WEDGE: no commit for 200000 cyc, fetch pc=%h (c=%0d)", dut.imem_addr, c);
            $display("  DF: df=%0d df_stall=%b dmem_idle=%b dc_inv_req=%b dc_inv_busy=%b ifence=%b",
                     dut.df, dut.df_stall, dut.dmem_idle, dut.dc_inv_req, dut.dc_inv_busy, dut.ifence);
            $display("  FI: fi=%0d ifence=%b   PTW: pw_read=%b pw_busy=%b pw_match=%b",
                     dut.fi, dut.ifence, dut.pw_read, dut.pw_busy, dut.pw_match);
            $display("  REDIR: redirect=%b target=%h   satp=%h priv=%0d",
                     dut.core.redirect, dut.core.redirect_target, dut.core.mmu_satp, dut.core.mmu_priv);
            $display("  D$flush: st=%0d fscan=%0d inv_busy=%b inv_pend=%b  l2: req=%b we=%b ack=%b addr=%h",
                     dut.u_dcache.st, dut.u_dcache.fscan, dut.u_dcache.inv_busy, dut.u_dcache.inv_pend,
                     dut.dc_l2_req, dut.dc_l2_we, dut.dc_l2_ack, dut.dc_l2_addr);
            $display("  I$: ic_rd_req=%b ic_l2_req=%b ic_l2_ack=%b i_rd_pend=%b  dcr: req=%b addr=%h",
                     dut.ic_rd_req, dut.ic_l2_req, dut.ic_l2_ack, dut.i_rd_pend, dut.dcr_req, dut.dcr_addr);
            $display("  DISP: any_valid=%b ccfull=%b dispready=%b festall=%b sbfull=%b lqfull=%b rollv=%b dflt=%b ill=%b",
                     dut.core.any_valid, dut.core.cc_full, &dut.core.disp_ready, |dut.core.fe_stall,
                     dut.core.sb_full, dut.core.lq_full, dut.core.roll_v, dut.core.lsu_dfault_v, dut.core.ill_v);
            $display("  IFLT: pend=%b epc=%h ccempty=%b replay=%b devldsolo=%b injinfl=%b csr_irq=%b",
                     dut.core.pend_iflt, dut.core.iflt_epc, dut.core.cc_empty, dut.core.replay_v,
                     dut.core.devld_solo_v, dut.core.inject_inflight, dut.core.csr_irq_v);
            $display("  CC: cur=%0d committed=%0d cnt0=%0d cnt1=%0d cnt2=%0d cnt3=%0d cnt4=%0d cnt5=%0d cnt6=%0d cnt7=%0d",
                     dut.core.cur, dut.core.cc_committed, dut.core.cc.count[0], dut.core.cc.count[1],
                     dut.core.cc.count[2], dut.core.cc.count[3], dut.core.cc.count[4], dut.core.cc.count[5],
                     dut.core.cc.count[6], dut.core.cc.count[7]);
            $display("  FL: stall=%b free=%0d/%0d/%0d/%0d dvalid=%b create=%b",
                     dut.core.fe.u_dr.stall,
                     dut.core.fe.u_dr.rn.lane[0].sh.fl.free_count, dut.core.fe.u_dr.rn.lane[1].sh.fl.free_count,
                     dut.core.fe.u_dr.rn.lane[2].sh.fl.free_count, dut.core.fe.u_dr.rn.lane[3].sh.fl.free_count,
                     dut.core.fe.u_dr.rn.lane[0].sh.d_valid, dut.core.fe.u_dr.rn.lane[0].sh.create);
            probe_dump_ring(dut.imem_addr);
            $finish;
         end
`endif
`ifdef LSU_TAP
         // rename check (any cycle): store sd x8 @ ...8003ddb6 -> its rs2 physreg (ps2)
         // vs x8 producer addi x8 @ ...80002f40 -> its dest (pdst). Mismatch => rename bug.
         // Forwarding trace: whenever a load is HELD in MERGE (p_v) reading the 0xf88 word
         // (VA[11:3]=0x1f1), dump the forwarding decision -- s_use (eligible older stores),
         // mem_rdata (memory), c_val (merge result), and the SB. Shows forward-vs-memory.
         if ($time > 95450000000 && $time < 95470000000 && dut.core.u_lsu.p_v)
            $display("[%0t PV p_w0=%h off=%h c_val=%h]", $time,
               dut.core.u_lsu.p_w0, dut.core.u_lsu.p_w0[8:0], dut.core.u_lsu.c_val);
         if ($time > 90000000000 && dut.core.u_lsu.p_v
             && (dut.core.u_lsu.p_w0[8:0] == 9'h1f1)) begin
            $display("[%0t LD-MERGE p_seq=%0d p_w0=%h p_lb=%0d s_use=%b mem_rdata=%h c_val=%h]",
               $time, dut.core.u_lsu.p_seq, dut.core.u_lsu.p_w0, dut.core.u_lsu.p_lb,
               dut.core.u_lsu.s_use, dut.core.u_lsu.mem_rdata, dut.core.u_lsu.c_val);
            for (b2 = 0; b2 < 4; b2 = b2 + 1)
               $display("    SB[%0d] v=%b rdy=%b cmt=%b seq=%0d w0=%h d0=%h be0=%b", b2,
                  dut.core.u_lsu.sb_v[b2], dut.core.u_lsu.sb_rdy[b2], dut.core.u_lsu.sb_cmt[b2],
                  dut.core.u_lsu.sb_seq[b2], dut.core.u_lsu.sb_w0[b2], dut.core.u_lsu.sb_d0[b2],
                  dut.core.u_lsu.sb_be0[b2]);
         end
`endif
      end
      $display("COSIM-LINUX TIMEOUT after %0d cycles (pc~%h)", ncyc, dut.imem_addr);
`ifdef HPM_DUMP
      $display("DDR-HPM read : cnt=%0d sum=%0d  bins[1..7]=%0d %0d %0d %0d %0d %0d %0d",
               dut.u_ddr_hpm.rd_cnt, dut.u_ddr_hpm.rd_sum, dut.u_ddr_hpm.rdb[1],
               dut.u_ddr_hpm.rdb[2], dut.u_ddr_hpm.rdb[3], dut.u_ddr_hpm.rdb[4],
               dut.u_ddr_hpm.rdb[5], dut.u_ddr_hpm.rdb[6], dut.u_ddr_hpm.rdb[7]);
      $display("DDR-HPM write: cnt=%0d sum=%0d  bins[1..7]=%0d %0d %0d %0d %0d %0d %0d",
               dut.u_ddr_hpm.wr_cnt, dut.u_ddr_hpm.wr_sum, dut.u_ddr_hpm.wrb[1],
               dut.u_ddr_hpm.wrb[2], dut.u_ddr_hpm.wrb[3], dut.u_ddr_hpm.wrb[4],
               dut.u_ddr_hpm.wrb[5], dut.u_ddr_hpm.wrb[6], dut.u_ddr_hpm.wrb[7]);
`endif
      $finish;
   end
endmodule

`default_nettype wire
