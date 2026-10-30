`timescale 1ns/1ps
`default_nettype none

// Linux-boot harness for the SoC (rv_soc_top). Resets DIRECTLY to OpenSBI
// (0x8000_0000) with a1 = the DTB pointer (+a1=, seeded into the PRF's x11),
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
`include "tb_rand.vh"
`include "smolrv64_tags.vh"           // the queue sizes FWD-SIM and DEPLD-SIM walk (SMOLRV64_LQB, SMOLRV64_SQB)
`include "smolrv64_width.vh"
module tb;
   localparam integer IW = `SMOLRV64_IW;     // the core's width (the same define sets its IW)
`include "smolrv64_credits.vh"               // the dispatch credits' indices (CR_*), at this width
`include "smolrv64_shards.vh"                // the register file's shards (SH_*)
   localparam [63:0] BASE = 64'h8000_0000;
`ifdef SMOLRV64_MEM_SIZE_LG2
   localparam [63:0] DDR_BYTES = 64'd1 << `SMOLRV64_MEM_SIZE_LG2;
`else
   localparam [63:0] DDR_BYTES = 64'd1 << 29;        // 512 MiB
`endif
   localparam [63:0] OFF_FW = 64'h000_0000, OFF_DTB_DEF = 64'h1ff0_0000,
                     OFF_INITRD_DEF = 64'h1f52_c000;
   reg [63:0] off_dtb, off_initrd;

   reg clk = 0; always #5 clk = ~clk;
   reg reset;
   wire [`SMOLRV64_IW-1:0] retire;
   wire        dmem_wen;
   wire [63:0] dmem_waddr, dmem_wdata;  wire [7:0] dmem_wmask;
   wire        ddr_q_valid, ddr_q_we, ddr_r_ready, ddr_w_ready;
   wire [4:0]  ddr_q_id;  wire [57:0] ddr_q_addr;  wire [63:0] ddr_q_wmask;  wire [511:0] ddr_q_wdata;
   wire        ddr_q_ready;
`ifdef TB_DDR4
   wire        ddr_r_valid, ddr_r_last, ddr_w_valid;
   wire [4:0]  ddr_r_id, ddr_w_id;  wire [1:0] ddr_r_beat;  wire [127:0] ddr_r_data;
`else
   reg         ddr_r_valid, ddr_r_last, ddr_w_valid;
   reg  [4:0]  ddr_r_id, ddr_w_id;  reg [1:0] ddr_r_beat;  reg [127:0] ddr_r_data;
`endif
   wire        rx_ready;

   // Model the hardware's 3 Mbps serializer instead of an instant drain: a 10-bit
   // frame at 66.67 MHz is ~222 core cycles, so uart_tx_ready stays low that long
   // after each accepted byte. THR then stays full between characters and THRE
   // interrupts land while the kernel is BUSY -- the interrupt pacing the FPGA sees,
   // which a tied-high ready never exercises.
   localparam TX_DRAIN = 222;
   reg  [7:0] txcnt = 8'd0;
   wire       uart_tx_v;
   wire [7:0] uart_tx_d;
   wire       uart_tx_rdy = (txcnt == 8'd0);
   always @(posedge clk)
      if (reset)                        txcnt <= 8'd0;
      else if (uart_tx_v & uart_tx_rdy) txcnt <= TX_DRAIN[7:0];
      else if (txcnt != 8'd0)           txcnt <= txcnt - 8'd1;

   // Console input: +uart_in=<file> types the file into the UART's receive FIFO from cycle
   // +uart_at=<n>, a byte every +uart_gap=<n> cycles (default 555: 3 Mbps at 166.67 MHz), with
   // no flow control, as the board's line has none: a byte that finds the FIFO full is lost.
   reg  [7:0]       uin [0:65535];
   reg  [63:0]      uin_n, uin_i, uin_at, uin_gap, uin_t;
   reg  [8*256-1:0] uin_file;
   integer          uin_fd, uin_ch;
   reg              uin_we;
   reg  [7:0]       uin_d;
   initial begin
      uin_n = 64'd0; uin_i = 64'd0; uin_t = 64'd0; uin_we = 1'b0; uin_d = 8'd0;
      if (!$value$plusargs("uart_at=%d", uin_at))   uin_at  = 64'd0;
      if (!$value$plusargs("uart_gap=%d", uin_gap)) uin_gap = 64'd555;
      if ($value$plusargs("uart_in=%s", uin_file)) begin
         uin_fd = $fopen(uin_file, "rb");
         if (uin_fd == 0) begin $display("FATAL: +uart_in=%0s does not open", uin_file); $finish; end
         for (uin_ch = $fgetc(uin_fd); uin_ch >= 0 && uin_n < 64'd65536; uin_ch = $fgetc(uin_fd)) begin
            uin[uin_n[15:0]] = uin_ch[7:0];  uin_n = uin_n + 64'd1;
         end
         $fclose(uin_fd);
         $display("[uart_in: %0d bytes from cycle %0d, one every %0d cycles]", uin_n, uin_at, uin_gap);
      end
   end
   always @(posedge clk) begin
      uin_t  <= uin_t + 64'd1;
      uin_we <= 1'b0;
      if (uin_we & ~rx_ready) $display("tb: uart_in byte %0d lost: the receive FIFO is full", uin_i - 64'd1);
      if (~reset & (uin_i < uin_n) & (uin_t >= uin_at) & ((uin_t - uin_at) % uin_gap == 64'd0)) begin
         uin_we <= 1'b1;  uin_d <= uin[uin_i[15:0]];  uin_i <= uin_i + 64'd1;
      end
   end

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
`ifndef TB_DC_KB                 // the cache sizes, for an A/B: -DTB_DC_KB=<n>, -DTB_IC_KB=<n>
 `define TB_DC_KB 128
`endif
`ifndef TB_IC_KB
 `define TB_IC_KB 128
`endif
   reg tb_dma_wr = 1'b0;           // a device wrote memory this cycle (the virtio-blk slave, the DMA agent)
   wire        sn_p_valid, sn_p_keep, sn_p_ack, sn_p_rel;  wire [25:0] sn_p_line;   // the tb's DMA snoops (below)
   rv_soc_top #(.RESET_PC(`TB_RESET_PC), .DC_KB(`TB_DC_KB), .SIZE_KB(`TB_IC_KB)) dut
     (.clk(clk), .reset(reset), .retire(retire),
      .dmem_wen(dmem_wen), .dmem_waddr(dmem_waddr), .dmem_wdata(dmem_wdata),
      .dmem_wmask(dmem_wmask),
      .ddr_q_valid(ddr_q_valid), .ddr_q_ready(ddr_q_ready), .ddr_q_id(ddr_q_id), .ddr_q_we(ddr_q_we),
      .ddr_q_addr(ddr_q_addr), .ddr_q_wmask(ddr_q_wmask), .ddr_q_wdata(ddr_q_wdata),
      .ddr_r_valid(ddr_r_valid), .ddr_r_ready(ddr_r_ready), .ddr_r_id(ddr_r_id), .ddr_r_beat(ddr_r_beat),
      .ddr_r_last(ddr_r_last), .ddr_r_data(ddr_r_data),
      .ddr_w_valid(ddr_w_valid), .ddr_w_ready(ddr_w_ready), .ddr_w_id(ddr_w_id),
      .uart_rx_we(uin_we), .uart_rx_data(uin_d), .uart_rx_ready(rx_ready),
      .uart_tx_valid(uart_tx_v), .uart_tx_data(uart_tx_d), .uart_tx_ready(uart_tx_rdy),
      .virtio_addr(virtio_addr), .virtio_read(virtio_read), .virtio_write(virtio_write),
      .virtio_wdata(virtio_wdata), .virtio_be(virtio_be),
      .virtio_rdata(virtio_rd_q), .virtio_rvalid(virtio_rvalid), .virtio_irq(virtio_irq | dma_irq), .virtio_net_irq(1'b0), .virtio_kbd_irq(1'b0), .dma_wr(tb_dma_wr), .irq_dbg(),
      .dma_sn_valid(sn_p_valid), .dma_sn_line(sn_p_line), .dma_sn_keep(sn_p_keep), .dma_sn_ack(sn_p_ack), .dma_sn_rel(sn_p_rel));
   initial if (dut.core.IW != IW) $fatal(1, "tb: the bench is built for IW=%0d, the core has %0d", IW, dut.core.IW);

   // THE DDR MODEL IS THE MEASURED ONE BY DEFAULT. Until 2026-09-04 the default was a flat
   // 4-cycle line latency, which is not this machine: the D$ admitted two requests under a
   // fill where the board admits dozens, the store queue never filled the way it does on the
   // board, and two six-day-old ordering defects lived through every gate.
   //
   // MEASURED on the board at 166.67 MHz on 2026-10-22 (our DDR4 controller behind the synchronous
   // crossing, the 2 GiB board; rv_soc_top's ddr_hpm read with workloads/ddrhpm around each of five
   // kernels -- a dependent chase over 256 KiB, and over 4 MiB arrays a read stream, a copy, a daxpy
   // and a fill):
   //     read  (fill):       mean 20.9-22.5 cycles -- 93-97% in 16-31, 0.6-3.4% in 32-63, 2.7-3.3% in 64+
   //     write (write-back): mean 16.4-18.7 cycles -- 64% in 8-15, 30% in 16-31, 0.8% in 32-63, 3.6% in 64+
   // The traffic barely moves them (a copy's reads 0.1 cycles over the chase's): the port's latency
   // is the controller's and the crossing's, not queueing. A write's done comes once the PHY has
   // its data, so writes draw separately; the tails are refreshes (tRFC is 92 cycles) and row
   // conflicts, not a wide uniform spread.
   // The bands are placed so the means match: reads 18-21, writes 12-15 and 16-19.
   //
   //   +ddr_lat=N     flat N-cycle latency, reads and writes alike (the old model; sweeps)
   //   -DDDR_LAT=N    the same, at compile time (kept for the existing sweep scripts)
   //   -DDDR_JIT=J    uniform 0..J added on top of whichever base is in force
   //   +ddr_cut=N     every draw N cycles shorter, at least the 4 beats (what a faster memory path
   //                  is worth); -DDDR_CUT=N the same, at compile time
`ifndef DDR_LAT
 `define DDR_LAT 0
`endif
`ifndef DDR_JIT
 `define DDR_JIT 0
`endif
`ifndef DDR_CUT
 `define DDR_CUT 0
`endif
   reg [63:0] ddr_lat_arg, ddr_cut_arg;
   initial if (!$value$plusargs("ddr_lat=%d", ddr_lat_arg)) ddr_lat_arg = 64'd`DDR_LAT;
   initial if (!$value$plusargs("ddr_cut=%d", ddr_cut_arg)) ddr_cut_arg = 64'd`DDR_CUT;
   function [7:0] ddr_cut(input [7:0] d);
      ddr_cut = ({56'd0, d} >= ddr_cut_arg + 64'd4) ? d - ddr_cut_arg[7:0] : 8'd4;
   endfunction
   reg [15:0] dlfsr; initial dlfsr = 16'hBEEF;
   function [7:0] ddr_draw;      // the measured shape; 0 = flat override in force
      input is_wr;
      reg [9:0] r;               // 0..1023: the thresholds below are the measured fractions
      begin
         r = dlfsr[9:0];
         if (ddr_lat_arg != 0)              ddr_draw = ddr_lat_arg[7:0];
         else if (is_wr) begin
            if      (r < 10'd37)            ddr_draw = 8'd64 + {2'd0, r[5:0]};   // 3.6%: 64-127
            else if (r < 10'd45)            ddr_draw = 8'd32 + {3'd0, r[4:0]};   // 0.8%: 32-63
            else if (r < 10'd357)           ddr_draw = 8'd16 + {6'd0, r[1:0]};   // 30.5%: 16-19
            else                            ddr_draw = 8'd12 + {6'd0, r[1:0]};   // 65%:  12-15; mean 17.9
         end else begin
            if      (r < 10'd31)            ddr_draw = 8'd64 + {2'd0, r[5:0]};   // 3.0%: 64-127
            else if (r < 10'd37)            ddr_draw = 8'd32 + {3'd0, r[4:0]};   // 0.6%: 32-63
            else                            ddr_draw = 8'd18 + {6'd0, r[1:0]};   // 96.4%: 18-21; mean 22.0
         end
      end
   endfunction
   // behavioral DDR, modeled as a 512-bit LINE array: the port is 64-byte lines, so the element
   // count stays under the array-dimension limit even at 2 GiB (a flat byte array overflows it).
   //
   // THE MIG'S SHAPE: up to DDR_NOUT transactions outstanding; each draws its latency from the
   // measured shape at acceptance (request to last beat); reads answer in request order and write
   // dones in request order, as the DDR4 controller does -- `+ddr_reorder` answers any read or
   // write whose time has come instead, to show the design does not depend on the order. A read
   // streams its four 128-bit beats on consecutive cycles, so a line takes four cycles of the
   // read-data channel. A write lands in the array when it is accepted, so a later read always
   // sees it; its done comes when its latency has run.
   localparam [63:0] NLINES = DDR_BYTES >> 6;
   localparam [63:0] LBASE  = BASE >> 6;
   localparam integer DDR_NOUT = 8;
   reg [511:0] lram [0:NLINES-1];
`ifdef TB_DDR4
   // TB_DDR4 (VDEFS=-DTB_DDR4): the board's path below the port instead of the drawn latencies --
   // ddr_port_sync into a ui_clk of twice the core's, our controller (ddr4_mem) and the PHY-only
   // model, whose DRAM is lram at the controller's map (src/ddr4_phy_model.vh). The DMA masters'
   // port is idle: the testbench's DMA writes lram itself.
   reg ui_clk = 1'b1;  always #2.5 ui_clk = ~ui_clk;          // its rising edges on clk's
   wire        mq_valid, mq_ready, mq_we, mr_valid, mr_ready, mr_last, mw_valid, mw_ready;
   wire [4:0]  mq_id, mr_id, mw_id;  wire [57:0] mq_addr;  wire [63:0] mq_wmask;  wire [511:0] mq_wdata;
   wire [1:0]  mr_beat;  wire [127:0] mr_data;
   wire [7:0]   mc_ACT_n, mc_BG, mc_CS_n, mc_ODT, mc_CKE;  wire [135:0] mc_ADR;  wire [15:0] mc_BA;
   wire         mcRdCAS, mcWrCAS, mcCasSlot2, winInjTxn, winRmw, gt_data_ready, cal_done;
   wire [1:0]   mcCasSlot, winRank;  wire [4:0] winBuf, rdDataAddr, wrDataAddr;
   wire [255:0] wrData, rdData;  wire [31:0] wrDataMask;  wire rdDataEn, wrDataEn, per_rd_done;
   wire         sn_m_valid, sn_m_keep, sn_m_ack, sn_m_rel;  wire [25:0] sn_m_line;
   ddr_port_sync u_ddr_sync (
      .clk_p(clk), .reset_p(reset),
      .p_q_valid(ddr_q_valid), .p_q_ready(ddr_q_ready), .p_q_id(ddr_q_id), .p_q_we(ddr_q_we), .p_q_addr(ddr_q_addr),
      .p_q_wmask(ddr_q_wmask), .p_q_wdata(ddr_q_wdata),
      .p_r_valid(ddr_r_valid), .p_r_ready(ddr_r_ready), .p_r_id(ddr_r_id), .p_r_beat(ddr_r_beat), .p_r_last(ddr_r_last),
      .p_r_data(ddr_r_data), .p_w_valid(ddr_w_valid), .p_w_ready(ddr_w_ready), .p_w_id(ddr_w_id),
      .clk_m(ui_clk), .reset_m(reset),
      .m_q_valid(mq_valid), .m_q_ready(mq_ready), .m_q_id(mq_id), .m_q_we(mq_we), .m_q_addr(mq_addr),
      .m_q_wmask(mq_wmask), .m_q_wdata(mq_wdata),
      .m_r_valid(mr_valid), .m_r_ready(mr_ready), .m_r_id(mr_id), .m_r_beat(mr_beat), .m_r_last(mr_last),
      .m_r_data(mr_data), .m_w_valid(mw_valid), .m_w_ready(mw_ready), .m_w_id(mw_id),
      .m_s_valid(sn_m_valid), .m_s_line(sn_m_line), .m_s_keep(sn_m_keep), .m_s_ack(sn_m_ack), .m_s_rel(sn_m_rel),
      .p_s_valid(), .p_s_line(), .p_s_keep(), .p_s_ack(1'b0), .p_s_rel());   // the tb's DMA snoops on its own (below)
   ddr4_mem u_ddr_mem (
      .clk(ui_clk), .reset(reset), .cal_done(cal_done),
      .q_valid(mq_valid), .q_ready(mq_ready), .q_id(mq_id), .q_we(mq_we), .q_addr(mq_addr),
      .q_wmask(mq_wmask), .q_wdata(mq_wdata),
      .r_valid(mr_valid), .r_ready(mr_ready), .r_id(mr_id), .r_beat(mr_beat), .r_last(mr_last),
      .r_data(mr_data), .w_valid(mw_valid), .w_ready(mw_ready), .w_id(mw_id),
      .s_axi_awid(3'd0), .s_axi_awaddr(32'd0), .s_axi_awlen(8'd0), .s_axi_awsize(3'd3), .s_axi_awburst(2'b01),
      .s_axi_awvalid(1'b0), .s_axi_awready(), .s_axi_wdata(64'd0), .s_axi_wstrb(8'd0), .s_axi_wlast(1'b0),
      .s_axi_wvalid(1'b0), .s_axi_wready(), .s_axi_bid(), .s_axi_bresp(), .s_axi_bvalid(), .s_axi_bready(1'b1),
      .s_axi_arid(3'd0), .s_axi_araddr(32'd0), .s_axi_arlen(8'd0), .s_axi_arsize(3'd3), .s_axi_arburst(2'b01),
      .s_axi_arvalid(1'b0), .s_axi_arready(), .s_axi_rid(), .s_axi_rdata(), .s_axi_rresp(), .s_axi_rlast(),
      .s_axi_rvalid(), .s_axi_rready(1'b1),
      .sn_valid(sn_m_valid), .sn_line(sn_m_line), .sn_keep(sn_m_keep), .sn_ack(sn_m_ack), .sn_rel(sn_m_rel),
      .mc_ACT_n(mc_ACT_n), .mc_ADR(mc_ADR), .mc_BA(mc_BA), .mc_BG(mc_BG), .mc_CS_n(mc_CS_n), .mc_ODT(mc_ODT),
      .mc_CKE(mc_CKE), .mcRdCAS(mcRdCAS), .mcWrCAS(mcWrCAS), .mcCasSlot(mcCasSlot), .mcCasSlot2(mcCasSlot2),
      .winBuf(winBuf), .winInjTxn(winInjTxn), .winRmw(winRmw), .winRank(winRank), .gt_data_ready(gt_data_ready),
      .wrData(wrData), .wrDataMask(wrDataMask), .wrDataEn(wrDataEn), .wrDataAddr(wrDataAddr),
      .rdDataEn(rdDataEn), .rdDataAddr(rdDataAddr), .rdData(rdData), .per_rd_done(per_rd_done));
   ddr4_phy_model #(.CAL(20)) u_ddr_phy (
      .clk(ui_clk), .reset(reset), .calDone(cal_done),
      .mc_ACT_n(mc_ACT_n), .mc_ADR(mc_ADR), .mc_BA(mc_BA), .mc_BG(mc_BG), .mc_CS_n(mc_CS_n), .mc_ODT(mc_ODT),
      .mc_CKE(mc_CKE), .mcRdCAS(mcRdCAS), .mcWrCAS(mcWrCAS), .mcCasSlot(mcCasSlot), .mcCasSlot2(mcCasSlot2),
      .winBuf(winBuf), .winInjTxn(winInjTxn), .winRmw(winRmw), .winRank(winRank), .gt_data_ready(gt_data_ready),
      .wrData(wrData), .wrDataMask(wrDataMask), .wrDataEn(wrDataEn), .wrDataAddr(wrDataAddr),
      .rdDataEn(rdDataEn), .rdDataAddr(rdDataAddr), .rdData(rdData), .per_rd_done(per_rd_done));
`else
   `TB_RAND(ddr_rnd, ddr_rs)
   reg         ddr_reorder;
   initial ddr_reorder = $test$plusargs("ddr_reorder");
   reg         dq_v   [0:DDR_NOUT-1];
   reg         dq_we  [0:DDR_NOUT-1];
   reg [4:0]   dq_id  [0:DDR_NOUT-1];
   reg [57:0]  dq_ad  [0:DDR_NOUT-1];
   reg [63:0]  dq_due [0:DDR_NOUT-1];
   reg [63:0]  dq_seq [0:DDR_NOUT-1];
   reg [63:0]  d_now, d_seq;
   reg         d_rs;               // a read is streaming its beats
   reg [1:0]   d_beat;             // ...this one next
   integer     d_rk;               // ...from this slot
   reg [511:0] d_rline;
   integer     dq_i, dq_k, d_free, d_nv, d_pick, d_wpick;
   reg [63:0]  line;
   reg [511:0] wbits;
   initial begin
      for (dq_i = 0; dq_i < DDR_NOUT; dq_i = dq_i + 1) dq_v[dq_i] = 1'b0;
      d_now = 0; d_seq = 0; d_rs = 1'b0;
   end
   always @* begin
      d_nv = 0; d_free = -1;
      for (dq_i = DDR_NOUT - 1; dq_i >= 0; dq_i = dq_i - 1)
         if (dq_v[dq_i]) d_nv = d_nv + 1; else d_free = dq_i;
   end
   assign ddr_q_ready = (d_free >= 0);
   always @(posedge clk) begin
      ddr_r_valid <= 1'b0;
      ddr_w_valid <= 1'b0;
      d_now <= d_now + 1;
      if (reset) begin
         for (dq_k = 0; dq_k < DDR_NOUT; dq_k = dq_k + 1) dq_v[dq_k] <= 1'b0;
         d_rs <= 1'b0;
      end else begin
         // accept
         if (ddr_q_valid && ddr_q_ready) begin
            dq_v[d_free]  <= 1'b1;  dq_we[d_free] <= ddr_q_we;  dq_id[d_free] <= ddr_q_id;
            dq_ad[d_free] <= ddr_q_addr;  dq_seq[d_free] <= d_seq;  d_seq <= d_seq + 1;
            dq_due[d_free] <= d_now + {48'd0, 8'd0, ddr_cut(ddr_draw(ddr_q_we))} + (ddr_rnd(0) % (`DDR_JIT + 1));
            dlfsr <= {dlfsr[14:0], dlfsr[15]^dlfsr[13]^dlfsr[12]^dlfsr[10]};
            if (ddr_q_we) begin
               line = ({6'd0, ddr_q_addr} - LBASE) & (NLINES-1);
               for (dq_k = 0; dq_k < 64; dq_k = dq_k + 1) wbits[dq_k*8 +: 8] = {8{ddr_q_wmask[dq_k]}};
               lram[line] <= (lram[line] & ~wbits) | (ddr_q_wdata & wbits);
            end
         end
         // a read streams its beats, one per cycle; the last beat frees its slot
         if (d_rs) begin
            if (ddr_r_ready) begin
               ddr_r_valid <= 1'b1;  ddr_r_id <= dq_id[d_rk];  ddr_r_beat <= d_beat;
               ddr_r_last  <= (d_beat == 2'd3);  ddr_r_data <= d_rline[d_beat*128 +: 128];
               d_beat <= d_beat + 2'd1;
               if (d_beat == 2'd3) begin d_rs <= 1'b0; dq_v[d_rk] <= 1'b0; end
            end
         end else begin
            // the next read: the oldest (or, with +ddr_reorder, any) whose first beat is due
            d_pick = -1;
            for (dq_k = DDR_NOUT - 1; dq_k >= 0; dq_k = dq_k - 1)
               if (dq_v[dq_k] && !dq_we[dq_k] && (ddr_reorder ? 1'b1 : ddr_oldest(dq_k, 1'b0)))
                  if (dq_due[dq_k] <= d_now + 4) d_pick = dq_k;   // its last beat lands at its due
            if (d_pick >= 0) begin
               line = ({6'd0, dq_ad[d_pick]} - LBASE) & (NLINES-1);
               d_rline <= lram[line];  d_rs <= 1'b1;  d_rk <= d_pick;  d_beat <= 2'd0;
            end
         end
         // the next write done: the oldest (or any) whose latency has run
         d_wpick = -1;
         for (dq_k = DDR_NOUT - 1; dq_k >= 0; dq_k = dq_k - 1)
            if (dq_v[dq_k] && dq_we[dq_k] && (ddr_reorder ? 1'b1 : ddr_oldest(dq_k, 1'b1)))
               if (dq_due[dq_k] <= d_now) d_wpick = dq_k;
         if (d_wpick >= 0 && ddr_w_ready) begin
            ddr_w_valid <= 1'b1;  ddr_w_id <= dq_id[d_wpick];  dq_v[d_wpick] <= 1'b0;
         end
      end
   end
   // slot k is the oldest live transaction of its direction
   function ddr_oldest(input integer k, input we);
      integer m;
      begin
         ddr_oldest = 1'b1;
         for (m = 0; m < DDR_NOUT; m = m + 1)
            if (dq_v[m] && dq_we[m] == we && dq_seq[m] < dq_seq[k]) ddr_oldest = 1'b0;
      end
   endfunction
   always @(posedge clk) if (!reset) begin
      if (d_nv > DDR_NOUT) $fatal(1, "tb: DDR model holds %0d transactions", d_nv);
   end
`endif


   // ==================== virtio-blk (B6, 2026-09-17; from the retired tb_cosim_linux.v) ====================
   // +disk=<image> attaches a file-backed SPI-mode SD card behind virtio_blk (virtio_mmio at
   // 0x10002000, PLIC source 11; the DTB must carry the node). The device DMAs into lram[] through
   // the coherent AXI slave below -- each line cleaned or flushed in the caches first, as on the
   // board -- and every write beat is mirrored into simmerv (cosim_dma_write, drained between the
   // two retires it separates). Without +disk the region answers as a device with no
   // media and the default DTB never touches it. +disk_ro keeps the image's writes in RAM.
   //
   // soc's device window is pages 2..7 of 0x1000_0000: virtio_addr[14:12] is the page, 2 = blk.
   // The sim has only blk: every other page reads 0 (magic 0 -> the kernel skips the node
   // cleanly); decoding fewer bits instead aliased net onto blk = a ghost vdb probe (seen in
   // the retired tb_virtio).
   wire [14:0] virtio_addr;  wire virtio_read, virtio_write;
   wire [31:0] virtio_wdata; wire [3:0] virtio_be;
   wire [31:0] virtio_rdata_comb; wire virtio_irq;
   wire        vio_net = virtio_addr[14:12] != 3'd2;   // anything but blk
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
   virtio_mmio #(.DEVICE_ID(32'd2), .QUEUE_NUM_MAX(32'd64)) u_vmmio
     (.clock(clk), .reset(reset),
      .address(virtio_addr[11:0]), .read(virtio_read && !vio_net), .read_data(virtio_rdata_comb),
      .write(virtio_write && !vio_net), .write_data(virtio_wdata), .byteenable(virtio_be),
      .config_read_data(virtio_addr[11:2] == 10'h040 ? v_capacity : 32'd0),   // capacity, sectors
      .config_write(), .config_offset(), .config_write_data(), .config_byteenable(),
      .irq(virtio_irq),
      .queue_notify_pulse(v_notify_pulse), .queue_notify_value(v_notify_value),
      .used_buffer_interrupt(v_used_irq), .config_change_interrupt(1'b0),
      .driver_features_0(), .driver_features_1(),
      .queue_num(), .queue_ready(), .queue_desc(), .queue_driver(), .queue_device(),
      .queue0_num(v_q0_num), .queue0_ready(v_q0_ready), .queue0_desc(v_q0_desc),
      .queue0_driver(v_q0_driver), .queue0_device(v_q0_device),
      .queue1_num(), .queue1_ready(), .queue1_desc(), .queue1_driver(), .queue1_device(),
      .device_status(v_dev_status));

   wire [2:0]  ax_awid;   wire [31:0] ax_awaddr; wire [7:0] ax_awlen;  wire [2:0] ax_awsize;
   wire [1:0]  ax_awburst; wire ax_awlock; wire [3:0] ax_awcache; wire [2:0] ax_awprot;
   wire [3:0]  ax_awqos;  wire ax_awvalid;  wire ax_awready;
   wire [63:0] ax_wdata;  wire [7:0] ax_wstrb; wire ax_wlast; wire ax_wvalid; wire ax_wready;
   wire [2:0]  ax_bid;    wire [1:0] ax_bresp; wire ax_bvalid; wire ax_bready;
   wire [2:0]  ax_arid;   wire [31:0] ax_araddr; wire [7:0] ax_arlen;  wire [2:0] ax_arsize;
   wire [1:0]  ax_arburst; wire ax_arlock; wire [3:0] ax_arcache; wire [2:0] ax_arprot;
   wire [3:0]  ax_arqos;  wire ax_arvalid;  wire ax_arready;
   wire [2:0]  ax_rid;    wire [63:0] ax_rdata; wire [1:0] ax_rresp; wire ax_rlast; wire ax_rvalid; wire ax_rready;
   wire        blk_sck, blk_mosi, blk_cs_n;  reg blk_miso = 1'b1;

   virtio_blk #(.QUEUE_SIZE(32'd64), .SD_SLOW_HALF(16'd4), .SD_FAST_HALF(16'd2), .SD_INIT_TICKS(16'd10)) u_vblk
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

   // behavioral AXI slave into lram[], a burst within one line at a time, COHERENT: before the
   // burst moves, its line is snooped through rv_soc_top's DMA port (the D$ cleans it for a read,
   // flushes it for a write, as ddr4_port_axi has it done on the board). A read takes the line
   // whole as its address is taken; a write's beats move into lram, and into simmerv
   // (cosim_dma_write, drained between the two retires it separates), as they come, and its release
   // follows its last -- until then the D$ starts no fill of the line. ax_*addr is the DDR byte
   // offset (src/dram_window.vh), so its line is the snoop's.
   reg axi_bvalid, r_busy = 1'b0, w_act = 1'b0;  reg [511:0] r_ln;  reg [2:0] r_bt, r_left, w_bt, w_left;
   reg [31:0] w_a;  integer ka;
   reg [63:0] ar_line, aw_line;  integer n_dma_rd = 0, n_dma_wr = 0, n_snoop = 0;
   // idle, a read's, a write's, the agent's burst, the agent's look at its ack word
   localparam [2:0] SN_I = 3'd0, SN_R = 3'd1, SN_W = 3'd2, SN_A = 3'd3, SN_P = 3'd4;
   reg  [2:0]  sn_st = SN_I;
   reg         sn_req = 1'b0, sn_got = 1'b0, sn_rel_q = 1'b0, sn_kq = 1'b0;  reg [25:0] sn_lq = 26'd0;
   assign sn_p_valid = sn_req;  assign sn_p_line = sn_lq;  assign sn_p_keep = sn_kq;  assign sn_p_rel = sn_rel_q;
   wire        ag_due, ag_poll;                         // the DMA agent's burst is due, its ack's look (below)
   wire [25:0] ag_line, ag_ackl;
   assign ax_arready = (sn_st == SN_R) && sn_got && !r_busy;
   assign ax_awready = (sn_st == SN_W) && sn_got && !w_act && !axi_bvalid;
   assign ax_wready  = w_act;
   assign ax_rvalid = r_busy; assign ax_rdata = r_ln[{r_bt, 6'd0} +: 64]; assign ax_rresp = 2'd0;
   assign ax_rlast = r_left == 3'd0; assign ax_rid = 3'd1;
   assign ax_bvalid = axi_bvalid; assign ax_bresp = 2'd0; assign ax_bid = 3'd1;
`ifdef SMOLRV64_COSIM
   import "DPI-C" function void cosim_dma_write(input longint off, input longint data, input byte strb);
`endif
   always @(posedge clk) begin
      sn_rel_q <= 1'b0;
      if (reset) begin sn_st <= SN_I;  sn_req <= 1'b0;  sn_got <= 1'b0; end
      else case (sn_st)
         SN_I: if (ax_arvalid && !r_busy) begin
                  sn_st <= SN_R;  sn_req <= 1'b1;  sn_kq <= 1'b1;  sn_lq <= ax_araddr[31:6];
               end else if (ax_awvalid && !axi_bvalid) begin
                  sn_st <= SN_W;  sn_req <= 1'b1;  sn_kq <= 1'b0;  sn_lq <= ax_awaddr[31:6];
               end else if (ag_due) begin
                  sn_st <= SN_A;  sn_req <= 1'b1;  sn_kq <= 1'b0;  sn_lq <= ag_line;
               end else if (ag_poll) begin
                  sn_st <= SN_P;  sn_req <= 1'b1;  sn_kq <= 1'b1;  sn_lq <= ag_ackl;
               end
         default: if (sn_p_ack) begin sn_req <= 1'b0;  sn_got <= 1'b1;  n_snoop = n_snoop + 1; end
                  else if (sn_got && (sn_st != SN_W || (ax_wvalid && ax_wready && ax_wlast))) begin   // the line moved: done
                     sn_st <= SN_I;  sn_got <= 1'b0;  sn_rel_q <= !sn_kq;
                  end
      endcase
   end
   always @(posedge clk) begin
      if (reset) begin r_busy <= 1'b0;  w_act <= 1'b0;  axi_bvalid <= 1'b0; end
      else begin
         if (ax_arvalid && ax_arready) begin
            ar_line = ((ax_araddr & (DDR_BYTES-1)) >> 6) & (NLINES-1);
            r_ln <= lram[ar_line];  r_bt <= ax_araddr[5:3];  r_left <= ax_arlen[2:0];
            r_busy <= 1'b1;  n_dma_rd = n_dma_rd + 1;
         end else if (r_busy && ax_rready) begin
            if (r_left == 3'd0) r_busy <= 1'b0;
            r_bt <= r_bt + 3'd1;  r_left <= r_left - 3'd1;
         end
         if (ax_awvalid && ax_awready) begin
            aw_line = ((ax_awaddr & (DDR_BYTES-1)) >> 6) & (NLINES-1);
            w_a <= ax_awaddr;  w_bt <= ax_awaddr[5:3];  w_left <= ax_awlen[2:0];  w_act <= 1'b1;
         end
         if (ax_wvalid && ax_wready) begin
            for (ka=0;ka<8;ka=ka+1) if (ax_wstrb[ka]) lram[aw_line][{w_bt, 6'd0} + ka*8 +: 8] <= ax_wdata[ka*8 +: 8];
`ifdef SMOLRV64_COSIM
            if (ax_wstrb != 8'd0)
               cosim_dma_write({32'd0, w_a[31:3], 3'b000}, ax_wdata, ax_wstrb);   // mirror into simmerv
`endif
            w_a <= w_a + 32'd8;  w_bt <= w_bt + 3'd1;  w_left <= w_left - 3'd1;
            if (ax_wlast) begin w_act <= 1'b0;  axi_bvalid <= 1'b1;  n_dma_wr = n_dma_wr + 1; end
         end else if (axi_bvalid && ax_bready) axi_bvalid <= 1'b0;
      end
   end
   always @(posedge clk) if (!reset) begin
      if (sn_st == SN_R && sn_got && !(ax_arvalid && ax_arready))
         $fatal(1, "tb: the DMA read's snoop answered, but its AR is gone");
      if (sn_st == SN_W && sn_got && !w_act && !ax_awvalid)
         $fatal(1, "tb: the DMA write's snoop answered, but its AW is gone");
      if (ax_arvalid && ({1'b0, ax_araddr[5:3]} + ax_arlen[3:0] > 4'd7 || ax_arlen[7:3] != 5'd0))
         $fatal(1, "tb: a DMA read burst (addr %h len %0d) that is not within one line", ax_araddr, ax_arlen);
      if (ax_awvalid && ({1'b0, ax_awaddr[5:3]} + ax_awlen[3:0] > 4'd7 || ax_awlen[7:3] != 5'd0))
         $fatal(1, "tb: a DMA write burst (addr %h len %0d) that is not within one line", ax_awaddr, ax_awlen);
      if (ax_wvalid && ax_wready && ax_wlast != (w_left == 3'd0))
         $fatal(1, "tb: a DMA write burst's wlast with %0d beats left", w_left);
   end


   // ---- the DMA agent (B4 memrand, 2026-09-17): +dma_rand=<seed> ---------------------------------
   // A device that DMAs on its own schedule: every 20-100 k cycles it writes a 64-byte burst of
   // random bytes into a random line of the 4 KiB DMA window (DDR offset 0x120_0000; the last line
   // holds the ack word) -- coherently, its line flushed from the caches first (the snoop machine
   // above) -- mirrors each beat into the reference (the same DPI-ordered queue as the virtio-blk
   // slave), and raises PLIC source 11 (shared with virtio-blk's irq). The program's
   // S-mode handler sums the window, writes the burst count to the ack word (an NC store, which the
   // L2 holds like any write) and completes; the agent looks at the ack word every 1000 cycles as a
   // device would, the line cleaned from the caches first, drops the irq once it shows the count,
   // then schedules the next burst. Race-free by protocol: the interrupt orders every read of the window
   // after the beats on both sides (a polled flag would not be: a beat landing between a poll's
   // execution and its retire is applied to the reference before that poll is compared).
   localparam [63:0] DMA_OFF = 64'h0120_0000;
   reg         dma_on = 1'b0, dma_irq = 1'b0;
   reg  [31:0] dma_seed = 32'd1, dma_lfsr = 32'hACE1_5EED, dma_ctr = 32'd0, dl;
   reg  [63:0] dma_next = 64'd50000, dma_line, dma_look = 64'd0;  reg [63:0] dma_beat;  integer dk;
   wire [63:0] dma_ack = lram[(DMA_OFF >> 6) + 64'd63][448 +: 64];
   wire        ag_go = (sn_st == SN_A) && sn_got && !sn_p_ack;   // the agent's line moves now
   wire        ag_look = (sn_st == SN_P) && sn_got && !sn_p_ack; // its ack word is in lram now
   assign ag_due  = dma_on && !dma_irq && c >= dma_next;
   assign ag_poll = dma_on && dma_irq && c >= dma_look;
   assign ag_ackl = DMA_OFF[31:6] + 26'd63;
   assign ag_line = DMA_OFF[31:6] + {20'd0, dma_lfsr[5:0] == 6'd63 ? 6'd0 : dma_lfsr[5:0]};
   always @(posedge clk)
      tb_dma_wr <= ~reset & ((ax_awvalid && ax_awready && ax_wvalid && ax_wready) | ag_go);
   always @(posedge clk) if (dma_on && !reset) begin
      if (ag_go) begin
         dl = dma_lfsr;
         dma_line = {38'd0, ag_line};
         for (dk = 0; dk < 8; dk = dk + 1) begin
            dl = dl ^ (dl << 13); dl = dl ^ (dl >> 17); dl = dl ^ (dl << 5);
            dma_beat = {dl, dl ^ 32'h9E37_79B9};
            lram[dma_line][dk*64 +: 64] <= dma_beat;
`ifdef SMOLRV64_COSIM
            cosim_dma_write((dma_line << 6) + dk*8, dma_beat, 8'hFF);
`endif
         end
         dma_lfsr <= dl;  dma_ctr <= dma_ctr + 1;  dma_irq <= 1'b1;  dma_look <= c + 64'd1000;
      end else if (ag_look) begin
         if (dma_ack == {32'd0, dma_ctr}) begin dma_irq <= 1'b0;  dma_next <= c + 64'd20000 + {48'd0, dma_lfsr[15:0]}; end
         else dma_look <= c + 64'd1000;
      end
   end

   // DPI: the file-backed SpiSdCard (src/sd_dpi.cpp) clocked by the SD-SPI pins each cycle
   import "DPI-C" function void sd_attach(input string path);
   import "DPI-C" function int  sd_clock(input int sck, input int cs_n, input int mosi);
   import "DPI-C" function void sd_readonly();
   always @(posedge clk) blk_miso <= sd_clock({31'd0, blk_sck}, {31'd0, blk_cs_n}, {31'd0, blk_mosi}) > 0 ? 1'b1 : 1'b0;

   // ARCHITECTURAL CHECKPOINTS (docs/PLAN-2026-09-25-arch-checkpoints.md): +ckpt=<dir> loads the
   // checkpoint's memory (its non-zero pages, from <dir>/mem.zst, src/ckpt_dpi.cpp) instead of
   // +fw/+dtb/+initrd, and +lram=<hex> loads the restore stub into the boot SRAM, where a
   // checkpoint build (TB_RESET_PC = 0x7000_0000) starts. The stub restores the registers, CSRs
   // and devices and mret's to the checkpoint's PC and privilege.
   import "DPI-C" context function longint ckpt_load(input string dir, input longint base, input longint nlines);
   export "DPI-C" function ckpt_line;
   function void ckpt_line(input longint idx, input bit [511:0] data);
      lram[idx] = data;
   endfunction

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
         $display("[core-linux: loaded %0d bytes @ DDR+%h (%0d lines)]", n, off, nl);
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

   // The event bus counted per cycle through csr_file's own event map: each code the map names
   // (hpm_sel_of) -- but cycles and instret, which the report prints as cycles and instructions --
   // adds what a counter selecting it adds (hpm_val), so these totals are the board counters'.
   // tools/perf-cpi-stack.py reads them, and tools/gen-perf-events.py keeps the JSON to the map.
   localparam integer PE_N = 128;
   integer    pe, pe_n;
   reg [15:0] pe_code [0:PE_N-1];
   integer    pe_sel  [0:PE_N-1];          // its index in csr_file (hpm_sel_of's value)
   reg [63:0] pev     [0:PE_N-1];
   initial begin
      pe_n = 0;
      for (pe = 3; pe < 32'h10000; pe = pe + 1)
         if (dut.core.u_csr.hpm_sel_of(16'(pe)) != dut.core.u_csr.HSEL_NONE) begin
            if (pe_n == PE_N) $fatal(1, "tb: csr_file's event map names more than %0d events", PE_N);
            pe_code[pe_n] = 16'(pe);  pe_sel[pe_n] = dut.core.u_csr.hpm_sel_of(16'(pe));  pev[pe_n] = 64'd0;
            pe_n = pe_n + 1;
         end
   end
   reg [63:0] trace_from, trace_to;             // +trace_from/+trace_to, see the plusargs
   // +measure_from=<retires>: the statistics count only from the cycle the retire count reaches
   // it (`meas`), so a run restored from a checkpoint reports its warm window, not the cold
   // start; the report at the end divides by the window's cycles and retires (mc, mr).
   reg [63:0] meas_from, meas_c, meas_ret, mc, mr;
   reg [63:0] lq_n;
   reg        meas;
   // FPORT-SIM: retired instructions by the register-file reader their integer operands take --
   // a divide's, the lane it issues in (it hands them to the MD stage); an FP op with an integer
   // source (fcvt/fmv from x), the lane it issues in (it hands rs1 to the F stage); a
   // CSR op reading rs1 and sfence.vma, lane A (it hands rs1 to the SYSQ) -- and the stores,
   // integer and FP, whose data the store queue takes from its own read port when it is in a
   // register file at allocation (else from a writeback), and the reads that port makes
   reg [63:0] fx_div, fx_i2f, fx_sys, fx_st, fx_fst, fx_rd;
   initial begin fx_div = 0; fx_i2f = 0; fx_sys = 0; fx_st = 0; fx_fst = 0; fx_rd = 0; end
   function automatic [2:0] fx_cls(input [31:0] i);   // 1 div, 2 int->FP, 3 sys, 4 store, 5 FP store
      begin
         fx_cls = 3'd0;
         if (((i[6:0] == 7'b0110011) || (i[6:0] == 7'b0111011)) && (i[31:25] == 7'b0000001) && i[14]) fx_cls = 3'd1;
         else if ((i[6:0] == 7'b1010011) && ((i[31:27] == 5'b11010) || (i[31:27] == 5'b11110)))         fx_cls = 3'd2;
         else if ((i[6:0] == 7'b1110011) && (((i[13:12] != 2'b00) && !i[14] && (i[19:15] != 5'd0))
                                             || ((i[14:12] == 3'b000) && (i[31:25] == 7'b0001001))))      fx_cls = 3'd3;
         else if (i[6:0] == 7'b0100011)                                                                    fx_cls = 3'd4;
         else if (i[6:0] == 7'b0100111)                                                                    fx_cls = 3'd5;
      end
   endfunction
   // SQ: the port's load starts, those it takes with an older store live in the store queue (the
   // MEM_REORD event's condition), the cycles a candidate load is alias-held, the queue's occupancy
   // summed per cycle, and the cycles it has no room for a store.
   reg [63:0] n_ldstart, n_ldreord, n_ldblk, n_sqocc, n_sqfull;
   // BTB-SIM: each training's way decision (smolrv64_predictor's tm: the hit way the CTI carried)
   // against the CTI's slots as they are now, which training does not read: an allocation of a CTI
   // a slot already holds (a duplicate), a rewrite of a way that no longer holds it (stale). The
   // tag is btag() at RB=9 (the 8K L2), TAGW=12; an entry's sits above {type, target[38:1]}.
   reg [63:0] bt_n, bt_car, bt_dup, bt_stale;
   initial begin bt_n = 0; bt_car = 0; bt_dup = 0; bt_stale = 0; end
   wire [63:0] bt_a   = 64'($signed(dut.core.fe.u_bp.t_last));   // a PC's sign extension
   wire [11:0] bt_tag = bt_a[24:13] ^ bt_a[36:25] ^ {11'd0, bt_a[63]};
   wire [1:0]  bt_now;
   genvar btw;
   generate for (btw = 0; btw < 2; btw = btw + 1) begin : btn
      reg [52:0] e;
      always @* case (bt_a[3:1])
         3'd0: e = dut.core.fe.u_bp.bank[0].way[btw].btb[dut.core.fe.u_bp.tsl[btw]];
         3'd1: e = dut.core.fe.u_bp.bank[1].way[btw].btb[dut.core.fe.u_bp.tsl[btw]];
         3'd2: e = dut.core.fe.u_bp.bank[2].way[btw].btb[dut.core.fe.u_bp.tsl[btw]];
         3'd3: e = dut.core.fe.u_bp.bank[3].way[btw].btb[dut.core.fe.u_bp.tsl[btw]];
         3'd4: e = dut.core.fe.u_bp.bank[4].way[btw].btb[dut.core.fe.u_bp.tsl[btw]];
         3'd5: e = dut.core.fe.u_bp.bank[5].way[btw].btb[dut.core.fe.u_bp.tsl[btw]];
         3'd6: e = dut.core.fe.u_bp.bank[6].way[btw].btb[dut.core.fe.u_bp.tsl[btw]];
         default: e = dut.core.fe.u_bp.bank[7].way[btw].btb[dut.core.fe.u_bp.tsl[btw]];
      endcase
      assign bt_now[btw] = e[52:41] == bt_tag;
   end endgenerate
   always @(posedge clk) if (!reset && meas && dut.core.fe.u_bp.res_v) begin
      bt_n <= bt_n + 1;
      if (dut.core.fe.u_bp.t_hit)                           bt_car   <= bt_car + 1;
      if (dut.core.fe.u_bp.tw_v & ~dut.core.fe.u_bp.tr_hit & (|bt_now))   bt_dup   <= bt_dup + 1;
      if (dut.core.fe.u_bp.tr_hit & ~|(dut.core.fe.u_bp.tm & bt_now))     bt_stale <= bt_stale + 1;
   end
   // L1BTB-SIM: the two-level BTB (smolrv64_predictor): the windows the ring took, the corrections
   // (L2 disagreed with L1 on where a window ends or where the stream goes on: the window is looked
   // up again, three cycles), and the windows that differed only in their marks (L1 filled).
   reg  [63:0] l1_wn, l1_cr, l1_mf;
   initial begin l1_wn = 0; l1_cr = 0; l1_mf = 0; end
   always @(posedge clk) if (!reset && meas) begin
      if (dut.core.fe.u_bp.adv & ~dut.core.fe.u_bp.rs) l1_wn <= l1_wn + 1;
      if (dut.core.fe.u_bp.cr_q & ~dut.core.fe.u_bp.flush) l1_cr <= l1_cr + 1;
      if (dut.core.fe.u_bp.mf_q) l1_mf <= l1_mf + 1;
   end
   // LQ-SIM: the load queue's occupancy summed per cycle, and the cycles the queue head has no
   // load-queue credit (smolrv64_core st_lq) by why: a head op in flight or in the IR (cbo_any: cbo),
   // every entry live (full),
   // or room by the count but none in a bank the row's loads take (two); and the entries live in those cycles by state: no address yet, an address but
   // not sent (of them: an older store aliases it, or it is not translated yet), at memory, landed
   // and watched.
   localparam integer LQS_N = 1 << `SMOLRV64_LQB;
   reg [63:0] n_lqocc, lq_cbo, lq_full, lq_two, lq_noa, lq_wait, lq_blk, lq_utv, lq_mem, lq_lnd;
   initial begin n_lqocc = 0; lq_cbo = 0; lq_full = 0; lq_two = 0; lq_noa = 0; lq_wait = 0; lq_blk = 0; lq_utv = 0;
                 lq_mem = 0; lq_lnd = 0; end
   wire [LQS_N-1:0] lq_v = dut.core.u_lq.v, lq_av = dut.core.u_lq.av, lq_sent = dut.core.u_lq.sent, lq_lnd_m = dut.core.u_lq.lnd;
   wire [31:0]      lq_cnt = 32'(dut.core.u_lq.cnt);
   reg  [31:0]      lq_bmin;                      // the fewest free entries a bank of the LQ has (a row takes one a bank)
   always @* begin : lqb
      integer b, e;  reg [31:0] f;
      lq_bmin = 32'hffffffff;
      for (b = 0; b < dut.core.QNB; b = b + 1) begin
         f = 0;
         for (e = b; e < LQS_N; e = e + dut.core.QNB) if (!lq_v[e]) f = f + 1;
         if (f < lq_bmin) lq_bmin = f;
      end
   end
   always @(posedge clk) if (!reset && meas) begin
      n_lqocc <= n_lqocc + 64'(lq_cnt);
      if (dut.core.st_lq) begin
         if (dut.core.cbo_any)     lq_cbo  <= lq_cbo + 1;
         else if (lq_cnt == LQS_N) lq_full <= lq_full + 1;
         else                      lq_two  <= lq_two + 1;
         if (!dut.core.cbo_any && (lq_bmin >= 2))
            $fatal(1, "tb: the load queue refuses a credit with two free entries in every bank (%0d live, IR loads %0d)", lq_cnt, dut.core.ir_ld);
         lq_noa  <= lq_noa  + 64'($countones(lq_v & ~lq_av));
         lq_wait <= lq_wait + 64'($countones(lq_v & lq_av & ~lq_sent & ~lq_lnd_m));
         lq_blk  <= lq_blk  + 64'($countones(lq_v & lq_av & ~lq_sent & ~lq_lnd_m & dut.core.u_lq.e_block));
         lq_utv  <= lq_utv  + 64'($countones(lq_v & lq_av & ~lq_sent & ~lq_lnd_m & ~dut.core.u_lq.tv));
         lq_mem  <= lq_mem  + 64'($countones(lq_v & lq_sent & ~lq_lnd_m));
         lq_lnd  <= lq_lnd  + 64'($countones(lq_v & lq_lnd_m));
      end
   end
   // SQ-DRAIN, the cycles a store waits between the queue and the D$: the LSU holds one at the D$
   // door, not accepted (n_stdoor); the queue's head can drain and the LSU is idle and does not
   // start it (n_sqidle); the queue holds stores and its head cannot drain (n_sqwait: uncommitted,
   // or its address or data not in).
   reg [63:0] n_stdoor, n_sqidle, n_sqwait;
   initial begin n_ldstart = 0; n_ldreord = 0; n_ldblk = 0; n_sqocc = 0; n_sqfull = 0;
                 n_stdoor = 0; n_sqidle = 0; n_sqwait = 0; end
   always @(posedge clk) if (!reset && meas) begin
      if (dut.core.lq_x_take)      n_ldstart <= n_ldstart + 1;
      if (dut.core.mem_reord)      n_ldreord <= n_ldreord + 1;
      if (dut.core.sq_ld_block)    n_ldblk   <= n_ldblk   + 1;
      n_sqocc <= n_sqocc + 64'(dut.core.sq_occ);
      if (~dut.core.sq_d_ready)    n_sqfull  <= n_sqfull  + 1;
      if (dut.core.u_lsu.st_go & ~dut.core.u_lsu.st_fin) n_stdoor <= n_stdoor + 1;
      if (dut.core.u_lsu.idle & dut.core.sq_c_v & ~dut.core.u_lsu.pt_start) n_sqidle <= n_sqidle + 1;
      if ((dut.core.sq_occ != 0) & ~dut.core.sq_c_v)  n_sqwait  <= n_sqwait + 1;
   end

   reg [8*256-1:0] fw, dtb, initrd, disk, ckpt, lramf;
   reg [63:0] ncyc, c, nret, last_ret, max_ret;
   reg [63:0] gap_lim, gap_max, gap_end;      // the longest stretch without a retire, and its bound

   // +stop_after=<text>: the run ends +stop_margin cycles (default 1,000,000) after the console
   // prints <text> -- a userspace test's PASS line -- instead of at +cycles: the lockstep has
   // checked every instruction up to there, and what follows is an idle system. The text is
   // matched against the bytes the UART sends, at most 64 of them.
   reg [8*64-1:0] stop_txt, stop_tail, stop_mask;
   reg [63:0]     stop_margin, stop_at, stop_c, stop_ret;
   reg            stop_on, stop_hit;
   integer        stop_k;
   initial begin
      stop_txt = {(8*64){1'b0}};  stop_tail = {(8*64){1'b0}};  stop_mask = {(8*64){1'b0}};
      stop_on = $value$plusargs("stop_after=%s", stop_txt);  stop_hit = 1'b0;
      stop_at = 64'd0;  stop_c = 64'd0;  stop_ret = 64'd0;
      if (!$value$plusargs("stop_margin=%d", stop_margin)) stop_margin = 64'd1000000;
      for (stop_k = 0; stop_k < 64; stop_k = stop_k + 1)
         if (stop_txt[8*stop_k +: 8] != 8'd0) stop_mask[8*stop_k +: 8] = 8'hFF;
      if (stop_on) $display("[stop_after: '%0s', then %0d cycles]", stop_txt, stop_margin);
   end
   always @(posedge clk) if (!reset && stop_on && !stop_hit && uart_tx_v && uart_tx_rdy) begin
      stop_tail = {stop_tail[8*63-1:0], uart_tx_d};
      if (((stop_tail ^ stop_txt) & stop_mask) == {(8*64){1'b0}}) begin
         stop_hit = 1'b1;  stop_c = c;  stop_ret = nret;  stop_at = c + stop_margin;
      end
   end
   integer    nrk;
   wire       trace_on = (c >= trace_from) && (c < trace_to);   // the tb-side trace window

   // ---- per-cycle accounting: every cycle is exactly one of BADSPEC / FRONTEND / BACKEND /
   // DISPATCHING (Top-Down Variant A, docs/SmolRV64-Spec.md §11.2), with a depth cause -- smolrv64_core's
   // td_k, the classifier its TD_* and TDK_* counters count, so these totals and the board's agree.
   // The same code drives the pipe views' stall rows and the TOPDOWN-SIM totals at the end. The
   // table names every value td_k has; the core asserts that fe:other, be:M-other and ok:0 never occur.
   localparam integer TD_N = 23;
   function [8*14-1:0] td_name;   // the cause's printable name
      input integer k;
      case (k)
        0: td_name = "ok:1";          1: td_name = "ok:2";         2: td_name = "ok:3";
        3: td_name = "bs:redirect";   4: td_name = "bs:drain";
        5: td_name = "fe:immu";       6: td_name = "fe:icache";    7: td_name = "fe:align";
        8: td_name = "fe:queue";      9: td_name = "fe:other";
        10: td_name = "be:M-mem";     11: td_name = "be:M-other";  12: td_name = "be:rob-full";
        13: td_name = "be:dep-load";  14: td_name = "be:dep-fp";   15: td_name = "be:iq-full";
        16: td_name = "be:rename";    17: td_name = "be:sq-full";  18: td_name = "be:lq-full";
        19: td_name = "be:serialize"; 20: td_name = "be:held";     21: td_name = "ok:0";
        22: td_name = "ok:4";
        default: td_name = "?";
      endcase
   endfunction
   function integer td_ndisp(input integer k);   // the instructions a cause dispatches: ok:N's N, else 0
      td_ndisp = (k <= 2) ? k + 1 : (k == 22) ? 4 : 0;
   endfunction
   wire [4:0] td_k = dut.core.td_k;   // the core's classifier: the one the hardware counters count
   reg [63:0] td_cnt [0:TD_N-1];
   // The instructions the IR's groups dispatch in the window, counted apart from td_k: outside a
   // redirect's cycle (td_dn), where every dispatching cycle is an ok:N, and in it (td_drd), where
   // the group is the wrong path the redirect squashes and the cycle is bs:redirect.
   reg [63:0] td_dn, td_drd;
   reg [63:0] td_sum, td_disp;        // the report's: cycles in a cause, the ok:N cycles' dispatches
   integer tdi;
   initial begin for (tdi = 0; tdi < TD_N; tdi = tdi + 1) td_cnt[tdi] = 64'd0; td_dn = 64'd0; td_drd = 64'd0; end
   always @(posedge clk) if (!reset && meas) begin
      if (td_k >= TD_N) $fatal(1, "tb: top-down cause td_k=%0d is outside the table (c=%0d)", td_k, c);
      td_cnt[td_k] <= td_cnt[td_k] + 64'd1;
      if (dut.core.redirect) td_drd <= td_drd + 64'($countones(dut.core.s_takev));
      else                   td_dn  <= td_dn  + 64'($countones(dut.core.s_takev));
   end

`include "smolrv64_geom.vh"
`include "tb_pipetrace.vh"

   // A REFILL CYCLE TAKES THE CAUSE OF THE CYCLE BEFORE IT (smolrv64_core td_rf, §11.2). Each split
   // of a cause below keeps the flags of the last cycle that is not a refill and charges a refill
   // cycle with them, so a cause's split covers exactly its cycles.
   wire td_rf = dut.core.td_rf;

   // ---- RENAME-SIM: be:rename's cycles by the shards whose free list is low, and the sweep ---
   // Rename stalls while any list is low or the map's sweep runs, so a cycle can count in several:
   // which shard's size a rename stall is about. Shards 0..IW-1 are lanes A.., SH_FL takes the FP
   // loads' landings and SH_FF the F stage's results; the report lists the shards this width uses.
   // [8] is the sweep after a squash (smolrv64_rename x_busy).
   reg [63:0] rn_low [0:8];
   reg [8:0]  rn_fp;
   integer rli;
   initial begin for (rli = 0; rli < 9; rli = rli + 1) rn_low[rli] = 64'd0; rn_fp = 9'd0; end
   wire [8:0] rn_now = {dut.core.u_rename.x_busy, dut.core.rn_shard_low};
   wire [8:0] rn_f = td_rf ? rn_fp : rn_now;
   always @(posedge clk) begin
      if (!td_rf) rn_fp <= rn_now;
      if (!reset && meas && (td_k == 5'd16)) begin
         if (rn_f == 9'd0) $fatal(1, "tb: a be:rename cycle with no shard low and no sweep (c=%0d)", c);
         for (rli = 0; rli < 9; rli = rli + 1)
            if (rn_f[rli]) rn_low[rli] <= rn_low[rli] + 64'd1;
      end
   end

   // ---- IQFULL-SIM: be:iq-full's cycles by the credit the queue head misses -------------
   // The scheduler credit the head needs: its lane's scheduler (cr_i[fe_hd_lane]), the head-op
   // register (cr_l), the FP queue (cr_f) or the divide credit (cr_div). An op can miss two (a
   // divide needs its lane's scheduler and the MD stage, a head op lane A's and the register). Of
   // the cycles its lane's scheduler is the only credit missing, those another lane's scheduler has
   // room in: what the lanes' rotation leaves of a full lane beside an empty one.
   reg [63:0] iq_c [0:5];   // its lane, head op, FP queue, divide; its lane alone; ...with another lane's room
   reg [4:0]  iq_fp;
   integer    iqi;
   initial begin for (iqi = 0; iqi < 6; iqi = iqi + 1) iq_c[iqi] = 64'd0; iq_fp = 5'd0; end
   wire [15:0] iq_gc  = dut.core.fe_hd_gc;
   wire [IW-1:0] iq_hl = IW'(1) << dut.core.fe_hd_lane;   // the head's lane
   // ROT-SIM: the instructions each lane dispatched, and the cycles a head op waited for lane A
   reg [63:0] rot_n [0:IW-1];
   reg [63:0] rot_ra;
   initial begin for (iqi = 0; iqi < IW; iqi = iqi + 1) rot_n[iqi] = 64'd0; rot_ra = 64'd0; end
   always @(posedge clk) if (!reset && meas) begin
      for (iqi = 0; iqi < IW; iqi = iqi + 1) if (dut.core.s_takev[iqi]) rot_n[iqi] <= rot_n[iqi] + 64'd1;
      if (dut.core.fe.realign) rot_ra <= rot_ra + 64'd1;
   end
   wire [4:0]  iq_now = {|(dut.core.cr_i & ~iq_hl),
                         iq_gc[dut.core.GC_M] & ~dut.core.cr_div, iq_gc[dut.core.GC_FC] & ~dut.core.cr_f[0],
                         iq_gc[dut.core.GC_L] & ~dut.core.cr_l,   iq_gc[dut.core.GC_I]  & ~|(dut.core.cr_i & iq_hl)};
   wire [4:0]  iq_f   = td_rf ? iq_fp : iq_now;
   always @(posedge clk) begin
      if (!td_rf) iq_fp <= iq_now;
      if (!reset && meas && (td_k == 5'd15)) begin
         if (iq_f[3:0] == 4'd0) $fatal(1, "tb: a be:iq-full cycle whose queue head has every scheduler credit (c=%0d)", c);
         for (iqi = 0; iqi < 4; iqi = iqi + 1) if (iq_f[iqi]) iq_c[iqi] <= iq_c[iqi] + 64'd1;
         if (iq_f[3:0] == 4'b0001) begin
            iq_c[4] <= iq_c[4] + 64'd1;
            if (iq_f[4]) iq_c[5] <= iq_c[5] + 64'd1;
         end
      end
   end

   // ---- DEPLD-SIM: be:dep-load's cycles by where the load being waited on is --------------
   // The register is the first blocked operand the core's dep_ld names (lanes A.., then the FP
   // scheduler: its registered reports l_blk_pq, rf_blk_pq), and its producer is the one the core's
   // ld_dst finds: a load-queue entry, a lane's landing buffer, a CSR read in the SYSQ or an AMO in
   // M. A load-queue entry is, in this order:
   //    0 not issued (no address)            1 its address waiting for M's fill
   //    2 untranslated (the dTLB missed: walking, or waiting for the walker)
   //    3 faulting (it traps at the ROB head)
   //    4 ready, waiting for the port         5 held: an older store's address is unknown (wait bit set)
   //    6 held: an older store overlaps it    7 a device load waiting to be the ROB head
   //    8 forwarding (its bytes in the LSU's forward queue)
   //    9 parked in the D$'s waiter table (a miss, or waiting for a resource or a queued store)
   //   10 in flight at the D$ otherwise (the read door, the lookup, a replay)
   //   11 on the LSU's slow path (a device, uncached or chunk-crossing load)
   // and the rest are 12 landed, in its lane's landing buffer (waiting for the write port),
   // 13 a CSR read in the SYSQ, 14 an AMO in M.
   localparam integer DL_N = 15, DL_PB = RN_PBITS, DL_LQ = 1 << `SMOLRV64_LQB;
   initial if (dut.core.LQ_N != DL_LQ)
      $fatal(1, "tb: DEPLD-SIM reads %0d load-queue entries, the core has %0d", DL_LQ, dut.core.LQ_N);
   reg [63:0] dl_c [0:DL_N-1];
   reg [3:0]  dl_kp;
   integer    dli;
   initial begin for (dli = 0; dli < DL_N; dli = dli + 1) dl_c[dli] = 64'd0; dl_kp = 4'd15; end
   function automatic [3:0] dl_lq(input integer k);   // live load-queue entry k's state
      begin
         if (!dut.core.u_lq.av[k])              dl_lq = dut.core.la_v[k] ? 4'd1 : 4'd0;
         else if (!dut.core.u_lq.tv[k])         dl_lq = 4'd2;
         else if (dut.core.u_lq.flt[k])         dl_lq = 4'd3;
         else if (!dut.core.u_lq.sent[k])
            dl_lq = dut.core.u_lq.rdy_m[k] ? 4'd4
                  : !dut.core.u_lq.e_block[k] ? 4'd7
                  : (dut.core.u_lq.wt[k] & dut.core.sq_l_block_unk_q[k]) ? 4'd5 : 4'd6;
         else if (dut.core.u_lq.fwd[k])         dl_lq = 4'd8;
         else if (dut.core.u_lsu.o_v[k])        dl_lq = dut.u_dcache.wt_v[k] ? 4'd9 : 4'd10;
         else                                   dl_lq = 4'd11;
      end
   endfunction
   always @(posedge clk) if (!reset && (td_k == 5'd13)) begin : depld
      reg [DL_PB-1:0] p;  reg pv;  reg [3:0] dc;  integer j;
      pv = 1'b0;  p = {DL_PB{1'b0}};  dc = dl_kp;
      if (!td_rf) begin
         dc = 4'd15;
         for (j = 0; j < IW; j = j + 1)
            if (!pv && dut.core.l_blk_vq[j] && dut.core.ld_dst(dut.core.l_blk_pq[j])) begin pv = 1'b1;  p = dut.core.l_blk_pq[j]; end
         if (!pv && dut.core.rf_blk_vq && dut.core.ld_dst(dut.core.rf_blk_pq)) begin pv = 1'b1;  p = dut.core.rf_blk_pq; end
         for (j = 0; j < DL_LQ; j = j + 1)
            if (pv && (dc == 4'd15) && !p[DL_PB-1] && dut.core.u_lq.v[j] && (dut.core.lq_e_dprd[j*DL_PB +: DL_PB] == p)) dc = dl_lq(j);
         for (j = 0; j < IW*dut.core.LBN; j = j + 1)
            if (pv && (dc == 4'd15) && (p != {DL_PB{1'b0}}) && (dut.core.lb_live[j*DL_PB +: DL_PB] == p)) dc = 4'd12;
         if (pv && (dc == 4'd15) && dut.core.sy_v && dut.core.sy_rd_v && (dut.core.sy_prd == p)) dc = 4'd13;
         if (pv && (dc == 4'd15) && dut.core.m_valid && dut.core.m_is_amo && dut.core.m_rd_v && (dut.core.m_prd == p)) dc = 4'd14;
         dl_kp <= dc;
      end
      if (meas) begin
         if (dc == 4'd15) $fatal(1, "tb: DEPLD-SIM finds no producer for a be:dep-load cycle's register p%0d (c=%0d)", p, c);
         dl_c[dc] <= dl_c[dc] + 64'd1;
      end
   end

   // ---- TRAP-SIM: the traps csr_file takes, by cause and by the mode they go to (an interrupt's
   // cause is 16 + its number), and the cycles and retires in each privilege mode -------------
   reg [63:0] trc_n [0:31][0:1];
   reg [63:0] trc_cyc [0:3], trc_ret [0:3];
   integer    tri_, trj_;
   initial begin
      for (tri_ = 0; tri_ < 32; tri_ = tri_ + 1) begin trc_n[tri_][0] = 0; trc_n[tri_][1] = 0; end
      for (tri_ = 0; tri_ < 4; tri_ = tri_ + 1) begin trc_cyc[tri_] = 0; trc_ret[tri_] = 0; end
   end
   // ...and the first 16 distinct (pc, cause) of the M-mode traps, each with its count and last tval
   reg [39:0] trp_pc [0:15];  reg [4:0] trp_k [0:15];  reg [63:0] trp_n [0:15], trp_tv [0:15];
   integer    trp_used;
   initial trp_used = 0;
   always @(posedge clk) if (!reset && meas && dut.core.u_csr.trap_v && !dut.core.u_csr.trap_to_s) begin : trappc
      integer k, j, hit;
      k = (dut.core.u_csr.trap_is_intr ? 16 : 0) + 32'(dut.core.u_csr.trap_cause[3:0]);
      hit = -1;
      for (j = 0; j < trp_used; j = j + 1) if (trp_pc[j] == dut.core.u_csr.trap_epc && trp_k[j] == 5'(k)) hit = j;
      if (hit >= 0) begin trp_n[hit] = trp_n[hit] + 1; trp_tv[hit] = dut.core.u_csr.trap_tval; end
      else if (trp_used < 16) begin
         trp_pc[trp_used] = dut.core.u_csr.trap_epc; trp_k[trp_used] = 5'(k); trp_n[trp_used] = 1;
         trp_tv[trp_used] = dut.core.u_csr.trap_tval; trp_used = trp_used + 1;
      end
   end
   always @(posedge clk) if (!reset && meas) begin : trapsim
      integer k;
      k = (dut.core.u_csr.trap_is_intr ? 16 : 0) + 32'(dut.core.u_csr.trap_cause[3:0]);
      if (dut.core.u_csr.trap_v) trc_n[k][dut.core.u_csr.trap_to_s ? 1 : 0] <= trc_n[k][dut.core.u_csr.trap_to_s ? 1 : 0] + 1;
      trc_cyc[dut.core.u_csr.priv] <= trc_cyc[dut.core.u_csr.priv] + 1;
      trc_ret[dut.core.u_csr.priv] <= trc_ret[dut.core.u_csr.priv] + 64'($countones(retire));
   end

   // ---- PORT-SIM: the cycles the load queue offers a ready load and the port does not take it, by
   // the first reason: a committed store drains (it has the port), the LSU's state machine is
   // busy (a store at the D$ door, a slow access), M's walk holds it, a read waits at the D$'s
   // door, the load's tag still has a response due ------------------------------------------
   reg [63:0] pt_n [0:5];
   integer    pti;
   initial for (pti = 0; pti < 6; pti = pti + 1) pt_n[pti] = 0;
   always @(posedge clk) if (!reset && meas && dut.core.lq_x_v && !dut.core.lq_x_take) begin
      if (dut.core.sq_go)                                              pt_n[0] <= pt_n[0] + 1;
      else if (dut.core.u_lsu.st != 3'd0)                              pt_n[1] <= pt_n[1] + 1;
      else if (dut.core.u_lsu.mmu_walking && dut.core.u_lsu.pm_q)      pt_n[2] <= pt_n[2] + 1;
      else if (dut.core.u_lsu.mem_rbusy)                               pt_n[3] <= pt_n[3] + 1;
      else if (dut.core.u_lsu.o_v[dut.core.u_lsu.pt_tag])              pt_n[4] <= pt_n[4] + 1;
      else                                                             pt_n[5] <= pt_n[5] + 1;
   end

   // ---- PORTS-SIM: what more memory ports would find to do, cycles: a load ready to go to the D$
   // (a queue candidate, or M's early start), two or more such loads, a ready load and a committed
   // store wanting the port together, a committed store alone, and two or more loads whose
   // address has arrived waiting for M's single translate pass -----------------------------
   reg [63:0] po_ld1, po_ld2, po_ldst, po_st, po_ar2;
   initial begin po_ld1 = 0; po_ld2 = 0; po_ldst = 0; po_st = 0; po_ar2 = 0; end
   always @(posedge clk) if (!reset && meas) begin : portssim
      integer nl, na;
      nl = $countones(dut.core.u_lq.rdy_m) + ((dut.core.m_lq_fill && dut.core.lsu_xo_early) ? 1 : 0);
      na = $countones(dut.core.u_lq.v & ~dut.core.u_lq.av & dut.core.u_lq.ar);
      if (nl >= 1) po_ld1 <= po_ld1 + 1;
      if (nl >= 2) po_ld2 <= po_ld2 + 1;
      if (nl >= 1 && dut.core.sq_c_v) po_ldst <= po_ldst + 1;
      if (nl == 0 && dut.core.sq_c_v) po_st <= po_st + 1;
      if (na >= 2) po_ar2 <= po_ar2 + 1;
   end

   // ---- DTLB-SIM: the dMMU walker's cycles busy (a second-level lookup or a walk), its lookups,
   // those the second level answers, the full walks after them and their PTE reads ---------
   reg [63:0] dt_busy, dt_lk, dt_l2, dt_walk, dt_pte;
   initial begin dt_busy = 0; dt_lk = 0; dt_l2 = 0; dt_walk = 0; dt_pte = 0; end
   always @(posedge clk) if (!reset && meas) begin
      if (dut.core.u_lsu.u_mmu.st != 2'd0)  dt_busy <= dt_busy + 1;
      if (dut.core.u_lsu.u_mmu.st == 2'd3) begin
         dt_lk <= dt_lk + 1;
         if (dut.core.u_lsu.u_mmu.l2_fill) dt_l2 <= dt_l2 + 1;
      end
      if ((dut.core.u_lsu.u_mmu.st == 2'd3) && dut.core.u_lsu.u_mmu.req_match && !dut.core.u_lsu.u_mmu.l2_hit) dt_walk <= dt_walk + 1;
      if ((dut.core.u_lsu.u_mmu.st == 2'd2) && dut.core.u_lsu.u_mmu.ptw_rvalid) dt_pte <= dt_pte + 1;
   end

   // ---- MEM-SIM: the D$'s misses, printed with TOPDOWN-SIM --------------------------------
   // STLK-SIM: the D$'s store queue, by cycle: the store lookup's head writes or merges into a fill,
   // or waits and why (a fill whose beat has landed; the main lookup's request in its set, a plain
   // load or another kind; a fill beat in its array; a line installing in its way or another);
   // misses, an entry behind the head sent back, the queue full, stores the main lookup completed,
   // the queue not empty; and a plain store's outcome in the main lookup.
   reg [63:0] sl_wr, sl_s1ld, sl_s1x, sl_fb, sl_flw, sl_flx, sl_miss, sl_back, sl_full, sl_main, sl_idle;
   reg [63:0] sm_alloc, sm_wait, sm_defer, sl_mg, sl_fw;
   initial begin sl_wr = 0; sl_s1ld = 0; sl_s1x = 0; sl_fb = 0; sl_flw = 0; sl_flx = 0; sl_miss = 0;
                 sl_back = 0; sl_full = 0; sl_main = 0; sl_idle = 0;
                 sm_alloc = 0; sm_wait = 0; sm_defer = 0; sl_mg = 0; sl_fw = 0; end
   always @(posedge clk) if (!reset && meas) begin : stlk
      reg hb, s1s, fbc;
      hb  = dut.u_dcache.sb_hd & dut.u_dcache.bhit;
      s1s = dut.u_dcache.s1_v & (dut.u_dcache.s1_set == dut.u_dcache.sb_set);
      fbc = dut.u_dcache.fb_v;
      if (dut.u_dcache.fp_mg)                  sl_mg   <= sl_mg + 1;
      if (dut.u_dcache.sb_hd & ~dut.u_dcache.bhit & dut.u_dcache.sb_ms & ~dut.u_dcache.sb_mgok) sl_fw <= sl_fw + 1;
      if (dut.u_dcache.fp_wr)                  sl_wr   <= sl_wr + 1;
      else if (hb & s1s & dut.u_dcache.k_ld)   sl_s1ld <= sl_s1ld + 1;
      else if (hb & s1s)                       sl_s1x  <= sl_s1x + 1;
      else if (hb & fbc)                       sl_fb   <= sl_fb + 1;
      else if (hb & dut.u_dcache.fl_v & dut.u_dcache.bhitv[dut.u_dcache.ms_way[dut.u_dcache.fl_m]]) sl_flw <= sl_flw + 1;
      else if (hb & dut.u_dcache.fl_v)         sl_flx  <= sl_flx + 1;
      if (dut.u_dcache.fp_miss)                sl_miss <= sl_miss + 1;
      if (dut.u_dcache.sb_v & ~dut.u_dcache.sb_hd) sl_back <= sl_back + 1;
      if (dut.u_dcache.sc == 2'd3)             sl_full <= sl_full + 1;
      if (dut.u_dcache.st_done & ~dut.u_dcache.fp_wr) sl_main <= sl_main + 1;
      if (dut.u_dcache.sc != 2'd0)             sl_idle <= sl_idle + 1;
      if (dut.u_dcache.s1_v & dut.u_dcache.k_st & ~dut.u_dcache.sq_bk) begin
         if (dut.u_dcache.m_alloc)        sm_alloc <= sm_alloc + 1;
         else if (dut.u_dcache.m_defer)   sm_defer <= sm_defer + 1;
         else                             sm_wait  <= sm_wait + 1;
      end
   end
   // ---- MFUN-SIM: M takes one plain load or store a cycle, in program order; an address the lanes
   // generated waits in the arrival RAMs until M picks it. The waiting entries summed over the
   // cycles over the fills is the mean wait (Little); cycles with two or more waiting ----------------
   reg [63:0] mf_wait, mf_fill, mf_c2, mf_c1;
   initial begin mf_wait = 0; mf_fill = 0; mf_c2 = 0; mf_c1 = 0; end
   always @(posedge clk) if (!reset && meas) begin : mfun
      integer n;
      n = pop64(64'(dut.core.la_v)) + pop64(64'(dut.core.sa_v));
      mf_wait <= mf_wait + 64'(n);
      if (n >= 1) mf_c1 <= mf_c1 + 1;
      if (n >= 2) mf_c2 <= mf_c2 + 1;
      if (dut.core.m_lq_fill || dut.core.m_sq_fill) mf_fill <= mf_fill + 1;
   end
   // ---- LDPATH-SIM: the path each plain load takes from M. It starts in M's own cycle (the D$
   // takes it there), and then stands or is cancelled a cycle later by an older store that blocks
   // it; or it goes the queue's way, counted by the first condition that kept it from starting:
   // the dTLB does not hold it (or its perms, a page crossing), the D$ takes no load this cycle,
   // or anything else (a page not in DRAM, NC, a doubleword straddle, its tag still busy). Each
   // class is split by whether M took the load straight from its lane or from the arrival RAM.
   reg [63:0] lp_n [0:11];             // [{class, from its lane}]
   reg        lp_byp, lp_st, lp_b;
   reg [7:0]  lp_ix;
   integer    lpi;
   initial begin for (lpi = 0; lpi < 12; lpi = lpi + 1) lp_n[lpi] = 0;  lp_byp = 0;  lp_st = 0; end
   always @(posedge clk) if (!reset) begin : ldpath
      reg [2:0] c;
      lp_byp <= dut.core.s_go & dut.core.s_byp & ~dut.core.s_st;   // M's next load came from its lane
      lp_st  <= 1'b0;
      if (meas && lp_st) begin
         c = dut.core.sq_l_cancel[lp_ix] ? 3'd1 : 3'd0;              // cancelled / stands
         lp_n[{c, lp_b}] <= lp_n[{c, lp_b}] + 1;
      end
      if (meas && dut.core.m_lq_fill) begin
         if (dut.core.lsu_xo_early) begin lp_st <= 1'b1;  lp_b <= lp_byp;  lp_ix <= 8'(dut.core.m_lq_idx); end
         else begin
            if (~dut.core.u_lsu.xo_ok)       c = 3'd2;   // the dTLB does not hold it, perms, page cross
            else if (~dut.core.dmem_fok)     c = 3'd3;   // the D$ takes no load this cycle
            else                             c = 3'd4;   // not DRAM, NC, a straddle, its tag busy
            lp_n[{c, lp_byp}] <= lp_n[{c, lp_byp}] + 1;
         end
      end
   end
   // ---- +ldtrace=<entry> [+ldtrace_from=<cycle>]: every event of one load-queue entry (rule G6) ---
   integer    lt_ix;  reg [63:0] lt_from;  reg lt_on;
   initial begin
      lt_on = $value$plusargs("ldtrace=%d", lt_ix);
      if (!$value$plusargs("ldtrace_from=%d", lt_from)) lt_from = 64'd0;
   end
   always @(posedge clk) if (!reset && lt_on && c >= lt_from) begin : ldtr
      reg al, fl, go, cn, ps, pf, ld, rs, kl;
      al = 1'b0;
      for (int k = 0; k < IW; k = k + 1) if (dut.core.g_ld[k] && (dut.core.s_lqi[k] == lt_ix)) al = 1'b1;
      fl = dut.core.m_lq_fill && (dut.core.m_lq_idx == lt_ix);
      go = dut.core.lsu_xo_early && (dut.core.m_lq_idx == lt_ix);
      cn = dut.core.sq_l_cancel[lt_ix];
      ps = dut.core.u_lsu.pt_start && !dut.core.u_lsu.pt_store && (dut.core.lq_x_idx == lt_ix);
      pf = dut.core.u_lsu.ld_fast_ok && (dut.core.lq_x_idx == lt_ix);
      ld = dut.core.ld_land && (dut.core.ld_land_idx == lt_ix);
      rs = dut.core.dmem_rvalid_c && (dut.core.dmem_rtag_resp == lt_ix);
      kl = dut.core.lq_kmask[lt_ix];
      if (al | fl | go | cn | ps | pf | ld | rs | kl | dut.core.redirect)
         $display("[c=%0d] LDTR e%0d: alloc=%b fill=%b go=%b(mv=%b dead=%b) cancel=%b pstart=%b pfast=%b land=%b(f%b s%b) resp=%b kill=%b flush=%b | v=%b av=%b sent=%b lnd=%b blk=%b o_v=%b o_kill=%b",
                  c, lt_ix, al, fl, go, dut.core.m_valid, dut.core.m_dead, cn, ps, pf, ld, dut.core.ld_land_fast, dut.core.ld_land_slow,
                  rs, kl, dut.core.redirect, dut.core.u_lq.v[lt_ix], dut.core.u_lq.av[lt_ix], dut.core.u_lq.sent[lt_ix],
                  dut.core.u_lq.lnd[lt_ix], dut.core.lq_e_block[lt_ix], dut.core.u_lsu.o_v[lt_ix], dut.core.u_lsu.o_kill[lt_ix]);
   end
   // ---- LAND-SIM: a landing load takes its destination lane's one write slot. Straight through when
   // the lane writes nothing else that cycle (by0), else into the lane's landing buffer (en0), and
   // while anything waits there the lane issues only address generators (its scheduler's ag_only):
   // the lane-cycles a waiting landing holds the lane are what a write-port collision costs.
   reg [63:0] ln_by, ln_buf, ln_hold;
   initial begin ln_by = 0;  ln_buf = 0;  ln_hold = 0; end
   always @(posedge clk) if (!reset && meas) begin
      ln_by   <= ln_by   + 64'(pop64(64'(dut.core.by0)));
      ln_buf  <= ln_buf  + 64'(pop64(64'(dut.core.en0)));
      ln_hold <= ln_hold + 64'(pop64(64'(dut.core.lb_any)));
   end
   // ---- HITSPEC-SIM: what waking a load's dependents in the D$'s compare cycle would gain and cost.
   // A load lookup in its compare (T+1, the D$'s s1) would wake as if it hit: right when it answers
   // in the next cycle, wrong when it is cancelled, misses or waits. The schedulers' entries waiting
   // on its register at that moment are the ops a right wake issues a cycle sooner -- those waiting on
   // it alone, indeed sooner -- and a wrong wake would have to replay. Integer lanes and the FP queue.
   localparam integer HS_NL = `SMOLRV64_IW, HS_NI = NI, HS_NF = NF, HS_PB = RN_PBITS, HS_LQB = `SMOLRV64_LQB;
   // a load queue entry's request: its tag, the D$ waiter entry, is the entry's index (rv_soc_top)
   wire [HS_LQB-1:0] hs_ix = dut.u_dcache.s1_tag[HS_LQB-1:0];
   wire [HS_PB-1:0] hs_prd = dut.core.u_lq.prd[hs_ix];
   wire             hs_ld  = dut.u_dcache.s1_v & ~dut.u_dcache.s1_st & ~dut.u_dcache.s1_nc
                           & ~dut.u_dcache.s1_tag[HS_LQB] & dut.core.u_lq.rdv[hs_ix];
   wire [7:0]       hs_wl [0:HS_NL], hs_ol [0:HS_NL];   // per lane, and the FP queue last: waiting, alone
   wire [7:0]       hs_al [0:HS_NL-1], hs_aol [0:HS_NL-1];   // ...of them address generators (a load or store)
   // an entry's instruction sits under its PC at the top of the lane's payload
   integer hs_ish;
   initial hs_ish = $bits(dut.core.ln[0].plmem[0]) - dut.core.PCW - 32;
   genvar hg;
   generate for (hg = 0; hg < HS_NL; hg = hg + 1) begin : hsl
      reg [7:0] w, o, a, ao;  integer e;  reg m0, m1, ag;  reg [31:0] ins;
      always @* begin
         w = 0;  o = 0;  a = 0;  ao = 0;
         for (e = 0; e < HS_NI; e = e + 1) begin
            m0 = dut.core.ln[hg].u_iq.v[e] & ~dut.core.ln[hg].u_iq.e_r[e][0] & (dut.core.ln[hg].u_iq.e_ps[e][0 +: HS_PB] == hs_prd);
            m1 = dut.core.ln[hg].u_iq.v[e] & ~dut.core.ln[hg].u_iq.e_r[e][1] & (dut.core.ln[hg].u_iq.e_ps[e][HS_PB +: HS_PB] == hs_prd);
            ins = 32'(dut.core.ln[hg].plmem[e] >> hs_ish);
            ag  = (ins[6:3] == 4'b0000) | (ins[6:3] == 4'b0100);   // smolrv64_core is_lmem
            if (m0 | m1) begin
               w = w + 1;  if (ag) a = a + 1;
               if ((m0 | dut.core.ln[hg].u_iq.e_r[e][0]) & (m1 | dut.core.ln[hg].u_iq.e_r[e][1])) begin
                  o = o + 1;  if (ag) ao = ao + 1;
               end
            end
         end
      end
      assign hs_wl[hg] = w;  assign hs_ol[hg] = o;  assign hs_al[hg] = a;  assign hs_aol[hg] = ao;
   end endgenerate
   reg [7:0] hs_fw, hs_fo;  integer hs_e;  reg [2:0] hs_m;
   always @* begin
      hs_fw = 0;  hs_fo = 0;
      for (hs_e = 0; hs_e < HS_NF; hs_e = hs_e + 1) begin
         hs_m[0] = dut.core.u_iq_f.v[hs_e] & ~dut.core.u_iq_f.e_r[hs_e][0] & (dut.core.u_iq_f.e_ps[hs_e][0 +: HS_PB] == hs_prd);
         hs_m[1] = dut.core.u_iq_f.v[hs_e] & ~dut.core.u_iq_f.e_r[hs_e][1] & (dut.core.u_iq_f.e_ps[hs_e][HS_PB +: HS_PB] == hs_prd);
         hs_m[2] = dut.core.u_iq_f.v[hs_e] & ~dut.core.u_iq_f.e_r[hs_e][2] & (dut.core.u_iq_f.e_ps[hs_e][2*HS_PB +: HS_PB] == hs_prd);
         if (|hs_m) begin
            hs_fw = hs_fw + 1;
            if (&(hs_m | dut.core.u_iq_f.e_r[hs_e])) hs_fo = hs_fo + 1;
         end
      end
   end
   assign hs_wl[HS_NL] = hs_fw;  assign hs_ol[HS_NL] = hs_fo;
   reg [63:0] hs_n [0:2], hs_w [0:2], hs_o [0:2], hs_a [0:2], hs_ao [0:2];   // [right, cancelled, missed or waiting]
   integer    hsi;
   initial for (hsi = 0; hsi < 3; hsi = hsi + 1) begin hs_n[hsi] = 0;  hs_w[hsi] = 0;  hs_o[hsi] = 0;  hs_a[hsi] = 0;  hs_ao[hsi] = 0; end
   always @(posedge clk) if (!reset && meas && hs_ld) begin : hspec
      integer k, c;  reg [15:0] w, o, a, ao;
      c = dut.u_dcache.ans ? 0 : (dut.u_dcache.s1_f & dut.u_dcache.f_cancel) ? 1 : 2;
      w = 0;  o = 0;  a = 0;  ao = 0;
      for (k = 0; k <= HS_NL; k = k + 1) begin w = w + {8'd0, hs_wl[k]};  o = o + {8'd0, hs_ol[k]}; end
      for (k = 0; k < HS_NL; k = k + 1) begin a = a + {8'd0, hs_al[k]};  ao = ao + {8'd0, hs_aol[k]}; end
      hs_n[c] <= hs_n[c] + 1;  hs_w[c] <= hs_w[c] + 64'(w);  hs_o[c] <= hs_o[c] + 64'(o);
      hs_a[c] <= hs_a[c] + 64'(a);  hs_ao[c] <= hs_ao[c] + 64'(ao);
   end
   // ---- SPEC-SIM: the speculative wake (D$ plan step 3a): wakes, and the address generators that
   // executed on one -- their address delivered on the load's answer, or suppressed ------------
   reg [63:0] sp_wake, sp_ok, sp_bad;
   wire [IW-1:0] sp_l;
   genvar spg;
   generate for (spg = 0; spg < IW; spg = spg + 1) begin : spl
      assign sp_l[spg] = dut.core.ln[spg].sp1;
   end endgenerate
   initial begin sp_wake = 0;  sp_ok = 0;  sp_bad = 0; end
   always @(posedge clk) if (!reset && meas) begin
      if (dut.core.sw_v) sp_wake <= sp_wake + 1;
      if (dut.core.sv_ok) sp_ok <= sp_ok + 64'(pop64(64'(sp_l))); else sp_bad <= sp_bad + 64'(pop64(64'(sp_l)));
   end
   // ---- RECOV-SIM: each restart's recovery against a forward rebuild ---------------------------
   // A restart (the ROB's k_v) walks the map back from the tail row to the branch's, a row a cycle,
   // racing the branch to the ROB head, where a squash and the sweep (dirty registers, IW a cycle)
   // restore it instead. A forward rebuild would sweep to the committed map at once and replay the
   // rows from the head through the branch's. Counted per restart: the recovery as it ran (to the
   // release, or the squash plus its sweep), the rebuild's estimate, and the frontend's refetch (the
   // first cycle after the restart its queue has a head again): only recovery beyond the refetch is
   // exposed. A restart a later, older one supersedes is not scored. CKPT-K models three map
   // checkpoints every K ROB rows (the oldest three past the head row; the committed map at the head
   // the fourth version): a restore in a cycle, then a replay forward a row a cycle from the newest
   // checkpoint at or before the branch's row, racing the walk and the head as the rebuild does.
   // CKPT-WEAK models three checkpoints right after weakly predicted conditionals (the core's wk_e:
   // dispatched, not yet resolved), held by the oldest three: a mispredicted weak branch holding one
   // restores in a cycle; another replays from the youngest checkpointed weak branch before its row.
   // CWALK models a compressed walk: each register renamed after the branch restored once, IW a
   // cycle, in place of a row a cycle (the distinct destinations the walk wrote; a restart the head
   // squash wins keeps its cost).
   reg        rv_on, rv_fes;  reg [63:0] rv_t0, rv_fe, rv_fwd, rv_b;
   reg [63:0] rv_ck [0:2];  reg [63:0] rv_cke [0:2];   // per K in {4, 8, 16}: the estimate, the exposed sum
   initial begin rv_cke[0] = 0; rv_cke[1] = 0; rv_cke[2] = 0; end
   localparam integer RV_CB = (RV_IW > 2) ? 2 : 1, RV_NC = 1 << RV_CB;
   reg        rv_wk, rv_wkq;  reg [63:0] rv_cw, rv_cwe, rv_nwk;   // the branch was weak; the weak model
   initial begin rv_wk = 0; rv_wkq = 0; rv_cwe = 0; rv_nwk = 0; end
   reg [63:0] rv_set, rv_rows, rv_cwk, rv_dist;  integer rv_p;
   initial begin rv_set = 0; rv_rows = 0; rv_cwk = 0; rv_dist = 0; end
   always @(posedge clk) if (!reset && meas) begin : rvwalk     // the walk's restored destinations
      reg [63:0] nset;
      nset = rv_set;
      for (rv_p = 0; rv_p < RV_IW; rv_p = rv_p + 1)
         if (dut.core.u_rob.wl_v[rv_p]) nset[dut.core.u_rob.wl_rd[rv_p*6 +: 6]] = 1'b1;
      if (dut.core.fr_rel || dut.core.cf_red_fire) begin rv_set <= 0;  rv_rows <= 0; end
      else begin rv_set <= nset;  if (dut.core.u_rob.wl_run) rv_rows <= rv_rows + 1; end
   end
   always @(posedge clk) if (!reset && dut.core.fr_set)        // the resolve: the branch's wk_e bit still holds
      rv_wkq <= dut.core.wk_e[((dut.core.cf_rob >> (RV_CB + 1)) << RV_CB) | (dut.core.cf_rob & (RV_NC - 1))];
   function automatic [63:0] ck_weak(input integer hr, input integer kb, input wk, input [63:0] fwd);
      integer r, col, n, last;
      begin
         n = 0;  last = -1;
         for (r = 0; r < kb; r = r + 1)          // the weak branches in the rows before the branch's, oldest first
            for (col = 0; col < RV_NC; col = col + 1)
               if (dut.core.wk_e[((hr + r) % `SMOLRV64_ROB_ROWS) * RV_NC + col] && n < 3) begin n = n + 1;  last = r; end
         if (wk && n < 3) ck_weak = 1;
         else if (last >= 0) ck_weak = 64'(1 + kb - last + 1);
         else ck_weak = fwd;
      end
   endfunction
   function automatic [63:0] ck_cost(input integer K, input integer hr, input integer kb, input [63:0] fwd);
      integer d0, j, best;
      begin
         d0 = (K - (hr % K)) % K;            // the first checkpoint row at or after the head, from the head
         best = -1;
         for (j = 0; j < 3; j = j + 1) if (d0 + j * K <= kb) best = d0 + j * K;
         ck_cost = (best < 0) ? fwd : 64'(1 + kb - best + 1);
      end
   endfunction
   reg [63:0] rv_n, rv_rel, rv_sq, rv_sup, rv_rows_b, rv_rows_f, rv_fe_sum, rv_now, rv_best, rv_win, rv_rec;
   initial begin rv_on = 0; rv_fes = 0; rv_n = 0; rv_rel = 0; rv_sq = 0; rv_sup = 0; rv_rows_b = 0; rv_rows_f = 0;
                 rv_fe_sum = 0; rv_now = 0; rv_best = 0; rv_win = 0; rv_rec = 0; end
   function automatic integer pop64(input [63:0] x);
      integer b; begin pop64 = 0; for (b = 0; b < 64; b = b + 1) pop64 = pop64 + x[b]; end
   endfunction
   localparam integer RV_IW = `SMOLRV64_IW;
   always @(posedge clk) if (!reset && meas) begin : recov
      reg [63:0] rec, fe, ex_now, ex_best, ex_ck, cwr;  integer k3;
      if (dut.core.u_rob.k_v) begin
         if (rv_on) rv_sup <= rv_sup + 1;
         rv_on <= 1'b1;  rv_t0 <= c;  rv_fes <= 1'b0;
         rv_fwd <= 64'((pop64(dut.core.u_rename.dirty) + RV_IW - 1) / RV_IW) + 64'(dut.core.u_rob.kb_rel) + 1;
         rv_b   <= 64'((dut.core.u_rob.trow - dut.core.u_rob.kn_row - 1) & (`SMOLRV64_ROB_ROWS - 1)) + 1;
         rv_wk <= rv_wkq;
         rv_cw <= ck_weak(dut.core.u_rob.hrow, dut.core.u_rob.kb_rel, rv_wkq,
                          64'((pop64(dut.core.u_rename.dirty) + RV_IW - 1) / RV_IW) + 64'(dut.core.u_rob.kb_rel) + 1);
         rv_ck[0] <= ck_cost(4,  dut.core.u_rob.hrow, dut.core.u_rob.kb_rel,
                             64'((pop64(dut.core.u_rename.dirty) + RV_IW - 1) / RV_IW) + 64'(dut.core.u_rob.kb_rel) + 1);
         rv_ck[1] <= ck_cost(8,  dut.core.u_rob.hrow, dut.core.u_rob.kb_rel,
                             64'((pop64(dut.core.u_rename.dirty) + RV_IW - 1) / RV_IW) + 64'(dut.core.u_rob.kb_rel) + 1);
         rv_ck[2] <= ck_cost(16, dut.core.u_rob.hrow, dut.core.u_rob.kb_rel,
                             64'((pop64(dut.core.u_rename.dirty) + RV_IW - 1) / RV_IW) + 64'(dut.core.u_rob.kb_rel) + 1);
      end else if (rv_on) begin
         if (!rv_fes && dut.core.fe_hd_v && c > rv_t0 + 1) begin rv_fes <= 1'b1;  rv_fe <= c - rv_t0; end
         if (dut.core.fr_rel || dut.core.cf_red_fire) begin
            rec = c - rv_t0 + (dut.core.fr_rel ? 64'd0 : 64'((pop64(dut.core.u_rename.dirty) + RV_IW - 1) / RV_IW));
            fe  = rv_fes ? rv_fe : rec;
            ex_now  = (rec > fe) ? rec - fe : 0;
            ex_best = ((rec < rv_fwd ? rec : rv_fwd) > fe) ? (rec < rv_fwd ? rec : rv_fwd) - fe : 0;
            rv_on <= 1'b0;  rv_n <= rv_n + 1;
            if (dut.core.fr_rel) rv_rel <= rv_rel + 1; else rv_sq <= rv_sq + 1;
            rv_rows_b <= rv_rows_b + rv_b;  rv_rows_f <= rv_rows_f + rv_fwd;  rv_fe_sum <= rv_fe_sum + fe;
            rv_rec <= rv_rec + rec;  rv_now <= rv_now + ex_now;  rv_best <= rv_best + ex_best;
            if (ex_best < ex_now) rv_win <= rv_win + 1;
            ex_ck = ((rec < rv_cw ? rec : rv_cw) > fe) ? (rec < rv_cw ? rec : rv_cw) - fe : 0;
            rv_cwe <= rv_cwe + ex_ck;  if (rv_wk) rv_nwk <= rv_nwk + 1;
            cwr = dut.core.fr_rel ? rec - rv_rows + 64'((pop64(rv_set) + RV_IW - 1) / RV_IW) : rec;
            if (cwr > rec) cwr = rec;
            rv_cwk <= rv_cwk + ((cwr > fe) ? cwr - fe : 0);
            if (dut.core.fr_rel) rv_dist <= rv_dist + 64'(pop64(rv_set));
            for (k3 = 0; k3 < 3; k3 = k3 + 1) begin
               ex_ck = ((rec < rv_ck[k3] ? rec : rv_ck[k3]) > fe) ? (rec < rv_ck[k3] ? rec : rv_ck[k3]) - fe : 0;
               rv_cke[k3] <= rv_cke[k3] + ex_ck;
            end
         end
      end
   end
   // Cycles any fill is in flight and the MSHRs live over them, fills started, cycles a request is
   // parked in the waiter table, cycles the write-back buffer is full, and of the ST_MEM event's
   // cycles (M holds a head op, or issue waits on an integer load's, AMO's or CSR read's result),
   // those with a fill in flight and those with a request parked.
   reg [63:0] ms_fill, ms_fills, ms_park, ms_stmem, ms_stmem_fill, ms_stmem_park, ms_live, ms_wbfull;
   initial begin ms_fill = 0; ms_fills = 0; ms_park = 0; ms_stmem = 0; ms_stmem_fill = 0; ms_stmem_park = 0;
                 ms_live = 0; ms_wbfull = 0; end
   always @(posedge clk) if (!reset && meas) begin : memsim
      reg fv, park;
      fv   = (dut.u_dcache.ob_live != 0);
      park = dut.u_dcache.ob_wait;
      if (fv)                    begin ms_fill <= ms_fill + 1;  ms_live <= ms_live + 64'(dut.u_dcache.ob_live); end
      if (dut.u_dcache.m_alloc)  ms_fills  <= ms_fills + 1;
      if (park)                  ms_park   <= ms_park + 1;
      if (dut.u_dcache.ob_wbfull) ms_wbfull <= ms_wbfull + 1;
      if (dut.core.st_mem) begin
         ms_stmem <= ms_stmem + 1;
         if (fv)   ms_stmem_fill <= ms_stmem_fill + 1;
         if (park) ms_stmem_park <= ms_stmem_park + 1;
      end
   end

   // ---- DISP-SIM: the dispatch group rules and the lanes' schedulers, printed with TOPDOWN-SIM ----
   // Per slot k > 0, the cycles slot k holds an op while slot k-1 dispatches and slot k does not:
   // cut by a group rule (smolrv64_frontend) -- beside a non-plain op (alone); a second load, a
   // second store, a second divide (unit), in that order when it is
   // more than one; after a CTI that redirects fetch at decode (resteer) -- or by the credit it
   // misses: its lane's scheduler, the LQ, the SQ, the FP queue, the divide credit. And what each
   // of the three one-per-group rules costs at most: the instructions the dispatching groups would
   // add with the rule relaxed (the bench's copy of the rules, the same credits, a credit's one bit
   // counted as room for any number), against those they took (asserted equal with none relaxed). Then the cycles the
   // queue head holds an op it may pop (no freeze, no low rename shard, no serialising op in flight
   // or in the IR) and does not: a serialise, CSR or CBO gate, the ROB, or a room credit holds it.
   // Over the lanes' schedulers: the entries ready and not picked (entry-cycles: every integer-class
   // op, whatever its unit, and a scheduler whose unit is busy picks nothing), those in a lane while
   // another lane picks nothing, and the picks that pass over an older ready entry (the pick is by
   // slot, not age); and the accesses M starts itself, and those in back-to-back cycles.
   // [slot][alone, unit, resteer, lane, load, store, fp, divide credits, a second load, a second store]
   reg [63:0] ds_cut [1:IW-1][0:9];
   reg [63:0] ds_hd, al_wait, al_wait_idle, al_young, ls_st, ls_b2b, ds_n, ds_rx [1:7];
   reg        ls_st_q;
   integer    dsi, dsj;
   initial begin
      for (dsi = 1; dsi < IW; dsi = dsi + 1) for (dsj = 0; dsj < 10; dsj = dsj + 1) ds_cut[dsi][dsj] = 64'd0;
      ds_hd = 0; al_wait = 0; al_wait_idle = 0; al_young = 0; ls_st = 0; ls_b2b = 0; ls_st_q = 0; ds_n = 0;
      for (dsi = 1; dsi < 8; dsi = dsi + 1) ds_rx[dsi] = 0;
   end
   function automatic ds_unit(input [15:0] g);   // smolrv64_frontend's one-a-group divide
      ds_unit = g[dut.core.GC_M];
   endfunction
   // the group the IR's head would dispatch with the rules in rx relaxed: [0] loads, [1] stores, [2] unit
   function automatic integer ds_grp(input [2:0] rx);
      reg [15:0] g0, gk;  reg [CRW-1:0] cr;  reg ok, fc, rm;  integer k, n;
      cr = dut.core.fe.crd;  g0 = dut.core.fe.g[0];
      ok = dut.core.fe.take[0];  n = ok ? 1 : 0;
      fc = ds_unit(g0);
      for (k = 1; k < IW; k = k + 1) begin
         gk = dut.core.fe.g[k];
         rm = (~gk[dut.core.GC_I] | dut.core.fe.ia[k]) & (~gk[dut.core.GC_L] | cr[CR_L]) & (~gk[dut.core.GC_LD] | cr[CR_LD + k])
            & (~gk[dut.core.GC_ST] | cr[CR_ST + k]) & (~gk[dut.core.GC_FC] | cr[CR_F + k]) & (~gk[dut.core.GC_M] | cr[CR_DIV]);
         ok = ok & (32'(dut.core.fe.q_cnt) > k) & g0[dut.core.GC_PLAIN] & gk[dut.core.GC_PLAIN]
            & ~dut.core.fe.g[k-1][dut.core.GC_DCR] & ~(ds_unit(gk) & fc & ~rx[2])
            & cr[CR_ROB1 + k] & rm;
         if (ok) n = n + 1;
         fc = fc | ds_unit(gk);
      end
      ds_grp = n;
   endfunction
   always @(negedge clk) if (!reset && meas) begin : dispsim
      reg [15:0] g;  reg [CRW-1:0] cr;  reg old;  integer k, j, e, nr, np, as, ae;
      cr = dut.core.fe.crd;
      for (k = 1; k < IW; k = k + 1)
         if (dut.core.fe.take[k-1] && !dut.core.fe.take[k] && (32'(dut.core.fe.q_cnt) > k)) begin
            g = dut.core.fe.g[k];
            if (!dut.core.fe.rule[k])
               dsj = (!dut.core.fe.g[0][dut.core.GC_PLAIN] || !g[dut.core.GC_PLAIN]) ? 0
                   : dut.core.fe.g[k-1][dut.core.GC_DCR] ? 2 : 1;
            else if (g[dut.core.GC_I] && !dut.core.fe.ia[k]) dsj = 3;   // its lane's (the lanes rotate)
            else if (g[dut.core.GC_LD] && !cr[CR_LD + k]) dsj = 4;
            else if (g[dut.core.GC_ST] && !cr[CR_ST + k]) dsj = 5;
            else if (g[dut.core.GC_FC] && !cr[CR_F + k]) dsj = 6;
            else if (g[dut.core.GC_M] && !cr[CR_DIV])    dsj = 7;
            else $fatal(1, "tb: slot %0d is cut with its group rule and every credit it needs (crd %b, class %h)", k, cr, g);
            ds_cut[k][dsj] = ds_cut[k][dsj] + 1;
         end
      if (dut.core.fe.rule[0] && !dut.core.fe.take[0] && cr[CR_POP]) ds_hd = ds_hd + 1;
      if (dut.core.fe.take[0]) begin
         nr = ds_grp(3'd0);
         if (nr != $countones(dut.core.fe.take))
            $fatal(1, "tb: DISP-SIM's copy of the group rules takes %0d, the frontend %b", nr, dut.core.fe.take);
         ds_n = ds_n + 64'(nr);
         for (j = 1; j < 8; j = j + 1) ds_rx[j] = ds_rx[j] + 64'(ds_grp(3'(j)) - nr);
      end
      np = 0;
      for (j = 0; j < IW; j = j + 1) np = np + (pl_iss_v[j] ? 1 : 0);
      for (j = 0; j < IW; j = j + 1) begin
         nr = $countones(pl_iq_v[j*PT_NI +: PT_NI] & pl_iq_rdy[j*PT_NI +: PT_NI]) - (pl_iss_v[j] ? 1 : 0);
         al_wait = al_wait + 64'(nr);
         if (np - (pl_iss_v[j] ? 1 : 0) < IW - 1) al_wait_idle = al_wait_idle + 64'(nr);
         if (pl_iss_v[j]) begin
            as = (32'(pl_iq_rob[j*PT_NI + 32'(pl_iss_ent[j])]) - 32'(dut.core.rob_head_idx)) & (dut.core.ROB_DEPTH - 1);
            old = 1'b0;
            for (e = 0; e < PT_NI; e = e + 1) begin
               ae = (32'(pl_iq_rob[j*PT_NI + e]) - 32'(dut.core.rob_head_idx)) & (dut.core.ROB_DEPTH - 1);
               if (pl_iq_v[j*PT_NI + e] && pl_iq_rdy[j*PT_NI + e] && (ae < as)) old = 1'b1;
            end
            if (old) al_young = al_young + 1;
         end
      end
      if (dut.core.lsu_started) begin ls_st = ls_st + 1; if (ls_st_q) ls_b2b = ls_b2b + 1; end
      ls_st_q = dut.core.lsu_started;
   end

   // ---- FWD-SIM: the loads the store queue holds, by forwarding, printed with TOPDOWN-SIM ----
   // Every cycle, every load-queue candidate an alias holds (cand_m & e_block), by the store queue's
   // own forwarding flag (l_fwd, fwn registered) and the load being cached memory: forwardable, by
   // what it waits for in the cycle -- the forward queue full (busy), the port (the store drain has
   // it, the LSU takes nothing, or an older candidate has it), or nothing: it starts -- or not, by
   // why: forwardable from the next cycle (fwn set, l_fwd not yet), an older store's unknown address
   // it waits for, more than one older store overlapping it, the one not holding all its bytes
   // (partial, or the page offsets alias another page), the one's data not in, or the rest (that
   // store untranslated, faulting or uncached or draining now, the load uncached, or a block that
   // clears a cycle late). Load-cycles, so a cycle with two held loads counts two. The one store not
   // holding all its bytes splits by its page: the load's (partial) or another (page-alias: the
   // store queue's overlap test reads page offsets, so it waits for that store's drain all the same).
   localparam integer FW_LQ = 1 << `SMOLRV64_LQB, FW_SQ = 1 << `SMOLRV64_SQB, FW_IB = `SMOLRV64_SQB, FW_N = 12;
   wire [FW_LQ-1:0] fw_nx;                    // each load's fwn: it forwards from the next cycle
   genvar gfw;
   generate for (gfw = 0; gfw < FW_LQ; gfw = gfw + 1) begin : fwn_g
      assign fw_nx[gfw] = dut.core.u_sq.g_lblk[gfw].fwn;
   end endgenerate
   reg [63:0] fw_go_n, fw_c [0:FW_N-1];      // busy, drain, LSU, older, start; next, unknown, several, partial, no-data, other, page-alias
   integer    fwi;
   initial begin fw_go_n = 0; for (fwi = 0; fwi < FW_N; fwi = fwi + 1) fw_c[fwi] = 0; end
   always @(posedge clk) if (!reset && meas && dut.core.lq_x_take && dut.core.lq_x_fwd) fw_go_n <= fw_go_n + 1;   // forwards started
   always @(negedge clk) if (!reset && meas) begin : fwdsim
      integer l, s, ys, nov, k;  reg unk;
      reg [FW_IB:0]   ld;
      reg [FW_IB-1:0] sd;
      for (l = 0; l < FW_LQ; l = l + 1) if (dut.core.u_lq.cand_m[l] && dut.core.u_lq.e_block[l]) begin
         if (dut.core.u_sq.l_fwd[l] && dut.core.u_lq.mem[l] && !dut.core.u_lq.unc[l])
            k = dut.core.lsu_fw_busy ? 0
              : (32'(dut.core.lq_x_idx) != l) ? 3
              : dut.core.lq_x_take ? 4 : dut.core.sq_go ? 1 : 2;
         else if (fw_nx[l]) k = 5;
         else begin
            ld = dut.core.u_sq.l_tag[l*(FW_IB+1) +: FW_IB+1] - dut.core.u_sq.headc;
            ys = 0;  unk = 1'b0;  nov = 0;
            for (s = 0; s < FW_SQ; s = s + 1) begin
               sd = s[FW_IB-1:0] - dut.core.u_sq.head;
               if (dut.core.u_sq.v[s] && ({1'b0, sd} < ld)) begin
                  if (!dut.core.u_sq.av[s]) unk = 1'b1;
                  if (dut.core.u_sq.conf[l][s]) begin nov = nov + 1;  ys = s; end
               end
            end
            k = (unk && dut.core.u_lq.wt[l]) ? 6 : (nov > 1) ? 7
              : (nov == 1 && !dut.core.u_sq.cov[l][ys])
                ? (({dut.core.u_sq.bare[ys], dut.core.u_sq.va[ys][38:12]} == {dut.core.u_lq.bare[l], dut.core.u_lq.va[l][38:12]}) ? 8 : 11)
              : (nov == 1 && !dut.core.u_sq.dv[ys]) ? 9 : 10;
         end
         fw_c[k] = fw_c[k] + 1;
      end
   end

   // ---- SER-SIM: be:serialize's cycles by the class of the op that holds them --------------
   // The serialising op in flight (its class captured as it dispatches), else the op at the queue
   // head. The IR's record loads the head's every cycle, so the head's record of a cycle is the
   // IR's the next: a cycle is classified one cycle late, when its head's record is in the IR.
   //   0 csr read, 1 csr write: a CSR op waiting for the CSR op before it
   //   2 amo/lr/sc   3 fence   4 fence.i   5 cbo: waiting for older loads' and stores' addresses
   //   6 sfence.vma  7 another system op (xret, ecall, ebreak, wfi, the interrupt pseudo-op)
   //   8 an op that traps at dispatch (illegal, or a fetch fault)
   localparam integer SER_N = 9;
   reg [63:0] ser_c [0:SER_N-1];
   reg [3:0]  ser_k_q, ser_kp, sp_k;          // the op in flight; the last classified cycle's; last cycle's in flight
   reg        sp_v, sp_rf, sp_inf, sp_m;      // last cycle: be:serialize, a refill, an op in flight, measured
   integer    seri;
   initial begin for (seri = 0; seri < SER_N; seri = seri + 1) ser_c[seri] = 0;
                 ser_k_q = 0; ser_kp = 0; sp_k = 0; sp_v = 0; sp_rf = 0; sp_inf = 0; sp_m = 0; end
   function automatic [3:0] ser_cls(input [31:0] i, input amo, input csr, input fi, input ill);
      begin
         if (ill)                       ser_cls = 4'd8;
         else if (amo)                  ser_cls = 4'd2;
         else if (csr)                  ser_cls = ((i[13:12] == 2'b01) || (i[19:15] != 5'd0)) ? 4'd1 : 4'd0;
         else if (i[6:2] == 5'b00011)   ser_cls = fi ? 4'd4 : (i[14:12] == 3'b010) ? 4'd5 : 4'd3;
         else if (i[6:2] == 5'b11100)   ser_cls = ((i[14:12] == 3'b000) && (i[31:25] == 7'h09)) ? 4'd6 : 4'd7;
         else                           ser_cls = 4'd15;   // not a serialising, CSR or CBO op
      end
   endfunction
   always @(negedge clk) if (!reset) begin : sersim
      reg [3:0] hk, ik;
      // the queue head's op (a held head is the op a be:serialize cycle waits on), and the IR's head
      // op (slot A's: a head op goes alone, to lane A)
      hk = ser_cls(dut.core.fe.h_insn, dut.core.fe.h_is_amo, dut.core.fe.h_is_csr, dut.core.fe.h_is_fencei,
                   dut.core.fe.h_illegal | dut.core.fe.h_fault);
      ik = ser_cls(dut.core.d_insn, dut.core.d_is_amo, dut.core.d_is_csr, dut.core.d_is_fencei,
                   dut.core.d_illegal | dut.core.d_fault);
      if (sp_v) begin                          // last cycle's class: a refill takes the one before it
         ser_kp = sp_rf ? ser_kp : sp_inf ? sp_k : hk;
         if (sp_m) begin
            if (ser_kp >= SER_N) $fatal(1, "tb: be:serialize charged to %h, not a serialising, CSR or CBO op (c=%0d)", dut.core.fe.h_insn, c);
            ser_c[ser_kp] = ser_c[ser_kp] + 1;
         end
      end
      sp_v = (td_k == 5'd19);  sp_rf = td_rf;  sp_inf = dut.core.ser_inflight;  sp_k = ser_k_q;  sp_m = meas;
      if (dut.core.rn_valid & dut.core.d_ser) ser_k_q = ik;   // the IR's op dispatches now
   end

   // ---- FRING-SIM: the fetch stream and its predictions, printed with TOPDOWN-SIM ----------
   // The I$ pairs the stream requests and the halfwords they append; the marks fetch consumes, by
   // prediction: taken (fetch takes it, f_take), not taken (consumed and passed), dropped (a
   // not-taken mark inside a 32-bit op); the marks it rejects, by kind (each restarts the stream: at
   // a non-CTI's fall-through, or inside a 32-bit op predicting nothing); and the cycles the stream
   // could ask for a pair but the prediction queue, or the ring (held and in flight), is full.
   reg [63:0] fr_req, fr_hw, fr_tk, fr_nt, fr_drop, fr_rft, fr_rmid, fr_pqfull, fr_rgfull;
   // Cycles the ring is empty and not frozen, by why the stream is not delivering: the iMMU has no
   // translation of the PC (or it faults), the stream waits for the PC to reach its page, the I$
   // holds its door (a miss being filled), a pair is in flight, or none of these (the restart's own
   // cycle).
   reg [63:0] fe_mmu, fe_page, fe_door, fe_infl, fe_oth;
   initial begin fe_mmu = 0; fe_page = 0; fe_door = 0; fe_infl = 0; fe_oth = 0; end
   // XLATE-SIM: M's translate passes by outcome -- the dTLB holds the address, its address alone
   // faults (a page crossing, non-canonical, beyond the top under Bare), or the walker gets it --,
   // the walker's answers and the faults among them, and the queue entries whose fault traps from
   // the ROB head (D$ plan increment 5b).
   reg [63:0] xs_hit, xs_aflt, xs_nopa, xs_wk, xs_wkf, xs_trap;
   initial begin xs_hit = 0; xs_aflt = 0; xs_nopa = 0; xs_wk = 0; xs_wkf = 0; xs_trap = 0; end
   always @(negedge clk) if (!reset && meas) begin
      if (dut.core.u_lsu.xo_tv)  xs_hit  = xs_hit + 1;
      if (dut.core.u_lsu.xo_v &  dut.core.u_lsu.xo_flt) xs_aflt = xs_aflt + 1;
      if (dut.core.u_lsu.xo_v & ~dut.core.u_lsu.xo_tv & ~dut.core.u_lsu.xo_flt) xs_nopa = xs_nopa + 1;
      if (dut.core.lsu_wk_done)  begin xs_wk = xs_wk + 1; if (dut.core.lsu_wk_flt) xs_wkf = xs_wkf + 1; end
      if (dut.core.qf_in)        xs_trap = xs_trap + 1;
   end
   // ITLB-SIM: does anything retire from each iTLB walk's page? The last 32 walked pages are kept;
   // a retirement from a page marks it, and a page leaving the ring unmarked counts as a walk for a
   // fetch nothing retired from (a wrong path, or a page left before any instruction of it committed).
   reg [26:0] iw_pg [0:31];
   reg [31:0] iw_v, iw_u;
   reg [4:0]  iw_wp;
   reg [63:0] iw_n, iw_waste;
   integer    iwi, iwk;
   initial begin iw_v = 0; iw_u = 0; iw_wp = 0; iw_n = 0; iw_waste = 0; end
   // ...and the same for I$ misses, by VA line: the last 64 missing lines.
   reg [32:0] im_ln [0:63];
   reg [63:0] im_v, im_u;
   reg [5:0]  im_wp;
   reg [63:0] im_n, im_waste;
   initial begin im_v = 0; im_u = 0; im_wp = 0; im_n = 0; im_waste = 0; end
   always @(negedge clk) if (!reset && meas) begin
      for (iwi = 0; iwi < 64; iwi = iwi + 1)
         for (iwk = 0; iwk < dut.core.IW; iwk = iwk + 1)
            if (im_v[iwi] && retire[iwk] && dut.core.cs_pc[dut.core.rc_sx[iwk]][38:6] == im_ln[iwi])
               im_u[iwi] = 1'b1;
      if (dut.u_icache.perf_miss) begin
         if (im_v[im_wp] && !im_u[im_wp]) im_waste = im_waste + 1;
         im_ln[im_wp] = dut.u_icache.s1_va[38:6]; im_v[im_wp] = 1'b1; im_u[im_wp] = 1'b0;
         im_wp = im_wp + 1; im_n = im_n + 1;
      end
   end
   always @(negedge clk) if (!reset && meas) begin
      for (iwi = 0; iwi < 32; iwi = iwi + 1)
         for (iwk = 0; iwk < dut.core.IW; iwk = iwk + 1)
            if (iw_v[iwi] && retire[iwk] && dut.core.cs_pc[dut.core.rc_sx[iwk]][38:12] == iw_pg[iwi])
               iw_u[iwi] = 1'b1;
      if (dut.core.u_immu.start_walk) begin
         if (iw_v[iw_wp] && !iw_u[iw_wp]) iw_waste = iw_waste + 1;
         iw_pg[iw_wp] = dut.core.u_immu.req_vaddr[38:12]; iw_v[iw_wp] = 1'b1; iw_u[iw_wp] = 1'b0;
         iw_wp = iw_wp + 1; iw_n = iw_n + 1;
      end
   end
   always @(posedge clk) if (!reset && meas && dut.core.fe.u_ring.rg_cnt == 0 && !dut.core.fe.u_ring.freeze) begin
      if (!dut.core.fe.u_ring.xlate_ok)                               fe_mmu  <= fe_mmu + 1;
      else if (!dut.core.fe.u_ring.inpg)                              fe_page <= fe_page + 1;
      else if (dut.core.fe.u_ring.ic_req & ~dut.core.fe.u_ring.ic_ack) fe_door <= fe_door + 1;
      else if (dut.core.fe.u_ring.rg_infl != 0)                       fe_infl <= fe_infl + 1;
      else                                                            fe_oth  <= fe_oth + 1;
   end
   initial begin fr_req = 0; fr_hw = 0; fr_tk = 0; fr_nt = 0; fr_drop = 0; fr_rft = 0; fr_rmid = 0;
                 fr_pqfull = 0; fr_rgfull = 0; end
   // TRAIN-SIM: the trainings (smolrv64_core's training queue: its head trains once its CTI is
   // non-speculative, at or behind the ROB's irrevocable point, or retired) and the resolving CTIs
   // a full queue drops.
   reg [63:0] tr_n, tr_drops;
   initial begin tr_n = 0; tr_drops = 0; end
   always @(posedge clk) if (!reset && meas) begin
      if (dut.core.res_v) tr_n <= tr_n + 1;
      tr_drops <= tr_drops + 64'(dut.core.tr_drop);
   end
   wire fr_open = ~dut.core.fe.u_ring.freeze & dut.core.fe.u_ring.xlate_ok & dut.core.fe.u_ring.inpg;
   wire fr_rej  = dut.core.fe.f_rej;
   always @(posedge clk) if (!reset && meas) begin
      if (dut.core.fe.u_ring.rq_acc) begin
         fr_req <= fr_req + 1;  fr_hw <= fr_hw + dut.core.fe.u_ring.rq_n;
      end
      // f_pop counts the marks consumed (a dropped one included); a bundle ends at its taken one
      if (32'(dut.core.fe.f_pop) < 32'(dut.core.fe.f_take) + 32'(dut.core.fe.f_drop))
         $fatal(1, "tb: fetch pops %0d marks, fewer than it takes (%b) and drops (%b)", dut.core.fe.f_pop, dut.core.fe.f_take, dut.core.fe.f_drop);
      fr_tk   <= fr_tk + 64'(dut.core.fe.f_take);
      fr_nt   <= fr_nt + 64'(dut.core.fe.f_pop) - 64'(dut.core.fe.f_take) - 64'(dut.core.fe.f_drop);
      fr_drop <= fr_drop + 64'(dut.core.fe.f_drop);
      if (fr_rej & ~dut.core.fe.f_rej_np)                    fr_rft  <= fr_rft + 1;
      if (fr_rej &  dut.core.fe.f_rej_np)                    fr_rmid <= fr_rmid + 1;
      if (fr_open & ~dut.core.fe.pq_room)                    fr_pqfull <= fr_pqfull + 1;
      if (fr_open & dut.core.fe.pq_room & ~dut.core.fe.u_ring.ic_req) fr_rgfull <= fr_rgfull + 1;
   end

   // +tohost=<hex addr>: a bare-metal program's EXIT, the riscv-tests convention (tohost =
   // code<<1 | 1, an 8-byte store). The run ends there instead of spinning to +cycles, and the
   // end-of-run block reports the cycles actually run (workloads/rvbench). A store to tohost
   // with bit 0 clear is an HTIF syscall, which this harness does not serve: rvbench routes
   // the console to the UART.
   reg [63:0] tohost_addr, tohost_code;  reg tohost_on, tohost_done;
   initial begin tohost_on = $value$plusargs("tohost=%h", tohost_addr); tohost_done = 1'b0; tohost_code = 64'd0; end
   always @(posedge clk) if (!reset && tohost_on && !tohost_done && dmem_wen
                             && dmem_waddr[63:3] == tohost_addr[63:3]) begin
      if (!dmem_wdata[0]) $fatal(1, "tb: HTIF syscall store to tohost (%h) -- not served; route the console to the UART", dmem_wdata);
      tohost_done <= 1'b1;  tohost_code <= dmem_wdata >> 1;
   end

   // +fe_trace: the frontend each cycle inside +trace_from/+trace_to -- the fetch PC, the halfwords
   // the fetch ring holds for it and whether they are its bytes, the window's marks, what fetch
   // consumed, a prediction taken, a stream restart, the predictions queued, fetch firing, the
   // bundle register, the queue, the IR and dispatch.
   reg fe_tr_on;
   initial fe_tr_on = $test$plusargs("fe_trace");
   always @(posedge clk) if (!reset && fe_tr_on && trace_on)
      $display("FE c=%0d pc=%h avail=%0d ok=%b mk=%b adv=%0d taken=%b rej=%b pq=%0d fire=%b pb=%b q=%0d room=%b ir=%b disp=%b red=%b",
               c, dut.core.fe.u_fetch.pc_q, dut.core.fe.imem_avail, dut.core.fe.imem_ok,
               dut.core.fe.imem_mk, dut.core.fe.f_adv_hw, dut.core.fe.f_take,
               dut.core.fe.f_rej, dut.core.fe.pq_cnt, dut.core.fe.fire, dut.core.fe.pb_v,
               dut.core.fe.q_cnt, dut.core.fe.q_room,
               dut.core.s_v, dut.core.s_takev, dut.core.fe.redirect);

   // +mq_trace: M's fill and the load queue each cycle inside +trace_from/+trace_to -- M's op, what
   // M takes (an arrival or a lane's bypass), the lanes' addresses, the arrivals waiting, the load
   // queue's live and filled entries, its tail and allocation, the kill and its release, a landing.
   reg mq_tr_on;
   initial mq_tr_on = $test$plusargs("mq_trace");
   always @(posedge clk) if (!reset && mq_tr_on && trace_on)
      $display("MQ c=%0d M v=%b ld=%b seq=%0d rob=%0d lqi=%0d dead=%b adv=%b | go=%b byp=%b rwin=%b uld=%b sli=%0d pk=%b/%0d/%0d | lm=%b%b%b lqi=%0d,%0d,%0d seq=%0d,%0d,%0d rob=%0d,%0d,%0d | la_v=%b | lq v=%b av=%b alloc=%b@%h kmask=%b | kd=%b rel=%b frv=%b red=%b | land=%b@%0d",
               c, dut.core.m_valid, dut.core.m_ld_nb, dut.core.m_seq, dut.core.m_rob_idx, dut.core.m_lq_idx, dut.core.m_dead, dut.core.m_advance,
               dut.core.s_go, dut.core.s_byp, dut.core.r_win, dut.core.u_ld, dut.core.s_li, dut.core.lq_pk_v, dut.core.lq_pk_idx, dut.core.lq_pk_seq,
               dut.core.l_lm[0], dut.core.l_lm[1], dut.core.l_lm[2], dut.core.l_lqi[0], dut.core.l_lqi[1], dut.core.l_lqi[2],
               dut.core.l_seq[0], dut.core.l_seq[1], dut.core.l_seq[2], dut.core.l_rob[0], dut.core.l_rob[1], dut.core.l_rob[2],
               dut.core.la_v,
               dut.core.u_lq.v, dut.core.u_lq.av, dut.core.g_ld, dut.core.lq_d_idxv, dut.core.lq_kmask,
               dut.core.rob_kd_v, dut.core.fr_rel, dut.core.fr_v, dut.core.redirect,
               dut.core.ld_land, dut.core.ld_land_idx);

   // +watch_pc=<hex pc>: follow ONE instruction -- its issue into the F stage or in a lane, its
   // retirement, and every F-stage PRF write to the physreg it was given.
   // Built for a jal whose link retired as another instruction's data (2026-09-21); a generated
   // straight-line program executes a PC once, so the three lines are the whole story.
   reg [63:0] watch_pc;  reg watch_on;  reg [15:0] watch_prd;  reg watch_prd_v;  reg [15:0] watch_rob;
   initial begin watch_on = $value$plusargs("watch_pc=%h", watch_pc); watch_prd_v = 1'b0; watch_prd = 16'd0; end
   integer ww;
   always @(posedge clk) if (!reset && watch_on) begin
      if (dut.core.iss_f && dut.core.f_iss_pc == watch_pc[39:0]) begin   // a PC's 40 bits (smolrv64_pc.vh)
         $display("[c=%0d] watch F-issue pc=%h rob=%0d rd=x%0d rd_v=%b prd=%0d",
                  c, dut.core.f_iss_pc, dut.core.j_rob, dut.core.qf_rd, dut.core.qf_rd_v, dut.core.qf_prd);
         watch_prd <= dut.core.qf_prd;  watch_prd_v <= dut.core.qf_rd_v;
      end
      // a control-flow op resolving in its lane
      for (ww = 0; ww < dut.core.NL; ww = ww + 1)
         if (dut.core.l_iss[ww] && dut.core.l_pc[ww] == watch_pc[39:0])
            $display("[c=%0d] watch lane-issue lane=%0d cti=%b mis=%b fr_v=%b head=%0d", c,
                     ww, dut.core.l_cti[ww], dut.core.l_mis[ww], dut.core.fr_v, dut.core.rob_head_idx);
      // the watched op retires: the value the cosim REPORTS beside the physreg's REAL contents
      if (retire[0] && dut.core.cs_pc[dut.core.rob_head_idx] == watch_pc[39:0])
         $display("[c=%0d] watch RETIRE   pc=%h head=%0d reported=%h cs_val[head]=%h",
                  c, watch_pc, dut.core.rob_head_idx, dut.core.cs_val_h[0],
                  dut.core.cs_val[dut.core.rob_head_idx]);
      if (watch_prd_v && dut.core.we_fe && dut.core.wa_fe == watch_prd)
         $display("[c=%0d] watch FE-write prd=%0d data=%h  fp_wb=%b md_wr=%b",
                  c, dut.core.wa_fe, dut.core.wb_fe, dut.core.fp_wb, dut.core.md_wr);
      if (dut.core.redirect && watch_prd_v) $display("[c=%0d] watch redirect (prd %0d still watched)", c, watch_prd);
      // retirement on any commit port
      for (ww = 0; ww < dut.core.IW; ww = ww + 1)
         if (dut.core.rc_v[ww] && dut.core.cs_pc[dut.core.rc_sx[ww]] == watch_pc[39:0])
            $display("[c=%0d] watch COMMIT port=%0d", c, ww);
   end
   initial begin
      ncyc = 200000000; nret = 0; last_ret = 0;  gap_max = 0;  gap_end = 0;
      if (!$value$plusargs("gap=%d", gap_lim)) gap_lim = 64'd1000000;
      n_inject = 0; n_uirq = 0; n_seip = 0;  td_sum = 0;  td_disp = 0;
      // +trace_from=<cycle> +trace_to=<cycle>: the window every tb-side trace honours, so a
      // trace is aimed by plusarg and never by a compile-time literal (three 17 M-cycle
      // rebuilds on 2026-09-04 went to a `$time` threshold in the wrong unit). RTL-side
      // `ifdef` traces read the same two plusargs themselves (rule G6).
      if (!$value$plusargs("trace_from=%d", trace_from)) trace_from = 64'd0;
      if (!$value$plusargs("trace_to=%d",   trace_to))   trace_to   = 64'hFFFF_FFFF_FFFF_FFFF;
      if ($value$plusargs("cycles=%d", ncyc)) ;
      if (!$value$plusargs("max_retires=%d", max_ret)) max_ret = 64'd0;   // 0: no retire bound
      if (!$value$plusargs("measure_from=%d", meas_from)) meas_from = 64'd0;
      meas = (meas_from == 64'd0);  meas_c = 64'd0;  meas_ret = 64'd0;
      if ($value$plusargs("lram=%s", lramf)) begin
         $readmemh(lramf, dut.lmem);
         $display("[core-linux: boot SRAM loaded from %0s]", lramf);
      end
      if ($value$plusargs("ckpt=%s", ckpt)) begin
         if (ckpt_load(ckpt, BASE, NLINES) < 0) begin $display("FATAL: +ckpt=%0s", ckpt); $finish; end
      end else begin
         if (!$value$plusargs("fw=%s", fw))   begin $display("FATAL: +fw");  $finish; end
         if (!$value$plusargs("dtb=%s", dtb)) begin $display("FATAL: +dtb"); $finish; end
         off_dtb = OFF_DTB_DEF; off_initrd = OFF_INITRD_DEF;
         if ($value$plusargs("dtb_off=%h",    off_dtb))    ;
         if ($value$plusargs("initrd_off=%h", off_initrd)) ;
         load_bin(fw,  OFF_FW);
         load_bin(dtb, off_dtb);
         if ($value$plusargs("initrd=%s", initrd)) load_bin(initrd, off_initrd);
      end
      if ($value$plusargs("disk=%s", disk)) begin
         sd_attach(disk);
         if ($test$plusargs("disk_ro")) begin sd_readonly(); $display("[sd: SNAPSHOT mode -- image writes stay in RAM]"); end
      end
      if ($test$plusargs("vio_trace")) vio_trace = 1'b1;
      if ($value$plusargs("dma_rand=%d", dma_seed)) begin dma_on = 1'b1; dma_lfsr = {dma_seed[15:0], 16'hACE1} ^ 32'h5EED_0000; end

      reset = 1; @(negedge clk); @(negedge clk); reset = 0;
      // +cycles=0 runs unbounded (stop with an external interrupt / timeout wrapper)
      for (c = 0; ((ncyc == 0) || (c < ncyc)) && !tohost_done && ((max_ret == 0) || (nret < max_ret))
                  && !(stop_hit && (c >= stop_at)); c = c + 1) begin
         @(negedge clk);
                  for (nrk = 0; nrk < `SMOLRV64_IW; nrk = nrk + 1) nret = nret + retire[nrk];
                  if (meas && dut.core.sq_r_v) fx_rd = fx_rd + 1;   // the store queue's data read
                  if (meas)
                     for (nrk = 0; nrk < `SMOLRV64_IW; nrk = nrk + 1)
                        if (retire[nrk])
                           case (fx_cls(dut.core.retire_insn[nrk*32 +: 32]))
                              3'd1: fx_div = fx_div + 1;  3'd2: fx_i2f = fx_i2f + 1;  3'd3: fx_sys = fx_sys + 1;
                              3'd4: fx_st  = fx_st + 1;   3'd5: fx_fst = fx_fst + 1;  default: ;
                           endcase
                  if (!meas && (nret >= meas_from)) begin
                     meas = 1'b1;  meas_c = c + 1;  meas_ret = nret;
                     $display("MEASURE-FROM retire %0d at cycle %0d: the statistics start here", nret, c + 1);
                  end
                  // A HANG IS A FAILURE AT ONCE: nothing in the machine waits a million cycles (WFI
                  // completes, an idle kernel retires), so a run that stops retiring stops here.
                  if (|retire) begin
                     if (c - last_ret > gap_max) begin gap_max = c - last_ret;  gap_end = c; end
                     last_ret = c;
                  end else if (c - last_ret > gap_lim) begin
                     // the restart's state: a wedge with one pending names its branch and what holds it
                     $display("tb: restart fr_v=%b fr_rob=%0d walk run=%b end=%b kill=%b age=%0d fpu_dead=%b rob v[head]=%b done[head]=%b",
                              dut.core.fr_v, dut.core.fr_rob, dut.core.u_rob.wl_run, dut.core.u_rob.wl_end, dut.core.rob_kd_v,
                              dut.core.kd_age, dut.core.fpu_dead, dut.core.u_rob.v[dut.core.u_rob.hsx], dut.core.u_rob.done[dut.core.u_rob.hsx]);
                     $display("tb: fetch: I$ st=%0d door=%b s1=%b sk=%b f_l2=%b pf=%b%b%b inv=%b%b hold=%b | ring cnt=%0d resv=%0d infl=%0d fq %0d..%0d k=%b ic_req=%b xlate_ok=%b inpg=%b pq_room=%b | walk_hold=%b wk_e=%0d immu_ready=%b irq_inject=%b strad=%b rob_empty=%b",
                              dut.u_icache.st, dut.u_icache.door, dut.u_icache.s1_v, dut.u_icache.sk_v, dut.u_icache.f_l2,
                              dut.u_icache.pf_val, dut.u_icache.pf_want, dut.u_icache.pf_infl, dut.u_icache.inv_pend, dut.u_icache.inv_busy,
                              dut.u_icache.fill_hold, dut.core.fe.u_ring.rg_cnt, dut.core.fe.u_ring.rg_resv, dut.core.fe.u_ring.rg_infl,
                              dut.core.fe.u_ring.fq_rp, dut.core.fe.u_ring.fq_wp, dut.core.fe.u_ring.fq_k, dut.core.fe.u_ring.ic_req,
                              dut.core.fe.u_ring.xlate_ok, dut.core.fe.u_ring.inpg, dut.core.fe.u_ring.pq_room,
                              dut.core.walk_hold, $countones(dut.core.wk_e), dut.core.immu_ready, dut.core.irq_inject,
                              dut.core.fe.u_fetch.strad, dut.core.rob_empty);
                     $fatal(1, "tb: nothing retired for %0d cycles (retires=%0d, ROB head %0d, pc~%h)",
                            c - last_ret, nret, dut.core.rob_head_idx, dut.imem_addr);
                  end
                  // B3 (2026-09-17): the CPI stack from the RTL's own event bus, so the same tool
                  // (tools/perf-cpi-stack.py) grades a simulation and a board run.
                  if (meas)
                     for (pe = 0; pe < pe_n; pe = pe + 1) pev[pe] = pev[pe] + 64'(dut.core.u_csr.hpm_val(pe_sel[pe]));
         if ((c % 1000000) == 0)
            $display("[c=%0d retires=%0d va=%h pa=%h prv=%0d satp=%h inj=%0d uirq=%0d seip=%0d ier=%h]",
                     c, nret, dut.core.fe.u_fetch.pc_q, dut.imem_addr, dut.core.mmu_priv,
                     dut.core.mmu_satp, n_inject, n_uirq, n_seip, dut.uart_ier);
      end
      $display("tb: the longest stretch without a retire: %0d cycles, ending at cycle %0d", gap_max, gap_end);
      if (tohost_done)
         $display("TOHOST exit=%0d after %0d cycles (retires=%0d pc~%h)", tohost_code, c, nret, dut.imem_addr);
      else if (stop_hit && (c >= stop_at))
         $display("\nSTOP-AFTER '%0s' at cycle %0d (retires=%0d), stopped at cycle %0d (retires=%0d pc~%h)",
                  stop_txt, stop_c, stop_ret, c, nret, dut.imem_addr);
      else if ((max_ret != 0) && (nret >= max_ret))
         $display("RETIRE-LIMIT after %0d cycles (retires=%0d pc~%h)", c, nret, dut.imem_addr);
      else
         $display("INO-LINUX TIMEOUT after %0d cycles (retires=%0d pc~%h)", ncyc, nret,
                  dut.imem_addr);
      // rv_soc_top's ddr_hpm over the whole run, as workloads/ddrhpm reads it on the board: lines,
      // latency sums (core cycles from acceptance) and the log2 bins [1] [2-3] ... [64+]
      $display("[ddr-hpm: rd %0d sum %0d bins %0d %0d %0d %0d %0d %0d %0d  wr %0d sum %0d bins %0d %0d %0d %0d %0d %0d %0d]",
               dut.u_ddr_hpm.rd_cnt, dut.u_ddr_hpm.rd_sum, dut.u_ddr_hpm.rdb[1], dut.u_ddr_hpm.rdb[2],
               dut.u_ddr_hpm.rdb[3], dut.u_ddr_hpm.rdb[4], dut.u_ddr_hpm.rdb[5], dut.u_ddr_hpm.rdb[6],
               dut.u_ddr_hpm.rdb[7], dut.u_ddr_hpm.wr_cnt, dut.u_ddr_hpm.wr_sum, dut.u_ddr_hpm.wrb[1],
               dut.u_ddr_hpm.wrb[2], dut.u_ddr_hpm.wrb[3], dut.u_ddr_hpm.wrb[4], dut.u_ddr_hpm.wrb[5],
               dut.u_ddr_hpm.wrb[6], dut.u_ddr_hpm.wrb[7]);
`ifdef TB_DDR4
      u_ddr_phy.report();
`endif
      $display("[blk: dma rd=%0d wr=%0d]", n_dma_rd, n_dma_wr);   // virtio-blk beats (B6); 0/0 without +disk
      $display("[dma-agent: bursts=%0d]", dma_ctr);   // B4 memrand; 0 without +dma_rand
      // B3: perf-stat text, the shape tools/perf-cpi-stack.py parses (`<count> r<code>`).
      mc = c - meas_c;  mr = nret - meas_ret;
      if (meas_from != 0)
         $display("MEASURE window: %0d retires in %0d cycles, IPC %0.3f (the statistics below count the window; WALK-SIM and MEMDEP the whole run)",
                  mr, mc, $itor(mr) / $itor(mc));
      $display("TOPDOWN-SIM: every cycle in exactly one cause (docs/SmolRV64-Spec.md 11.2), cycles");
      for (tdi = 0; tdi < TD_N; tdi = tdi + 1) begin
         td_sum  = td_sum + td_cnt[tdi];
         td_disp = td_disp + td_cnt[tdi] * 64'(td_ndisp(tdi));
         if (td_cnt[tdi] != 0)
            $display("TOPDOWN-SIM  %14s %12d  %6.2f%%", td_name(tdi), td_cnt[tdi],
                     100.0 * $itor(td_cnt[tdi]) / $itor(mc));
      end
      $display("TOPDOWN-SIM  %0s: %0d cycles counted of %0d; the ok:N cycles dispatch %0d instructions, the IR took %0d outside redirect cycles (and %0d wrong-path in them)",
               (td_sum == mc && td_disp == td_dn) ? "closed" : "NOT CLOSED", td_sum, mc, td_disp, td_dn, td_drd);
      $write("FETCH-SIM cycles the bundle register takes a bundle=%0d is held=%0d | gets none:", fs_take, fs_held);
      for (tdi = 0; tdi < FK_N; tdi = tdi + 1) $write(" %0s=%0d", fetch_name(4'(tdi)), fs_cnt[tdi]);
      $display("");
      $display("FRING-SIM pairs=%0d halfwords=%0d | marks: taken=%0d not-taken=%0d dropped=%0d | rejected marks, each a restart: fallthrough=%0d inside=%0d | cycles: pq-full=%0d ring-full=%0d",
               fr_req, fr_hw, fr_tk, fr_nt, fr_drop, fr_rft, fr_rmid, fr_pqfull, fr_rgfull);
      $display("FRING-EMPTY cycles: immu=%0d page=%0d icache-door=%0d in-flight=%0d other=%0d",
               fe_mmu, fe_page, fe_door, fe_infl, fe_oth);
      $display("XLATE-SIM M's translate passes: dtlb-hits=%0d address-faults=%0d to-the-walker=%0d | walker answers=%0d (faults %0d) | queue-entry traps=%0d",
               xs_hit, xs_aflt, xs_nopa, xs_wk, xs_wkf, xs_trap);
      $display("ITLB-SIM walks=%0d unused=%0d (a walked page nothing retired from before 32 more walks)", iw_n, iw_waste);
      $display("FWD-SIM forwards=%0d | held load-cycles, forwardable: busy=%0d drain=%0d lsu=%0d older-load=%0d start=%0d | not forwardable: next-cycle=%0d unknown-address=%0d several-stores=%0d partial=%0d page-alias=%0d no-data=%0d other=%0d",
               fw_go_n, fw_c[0], fw_c[1], fw_c[2], fw_c[3], fw_c[4], fw_c[5], fw_c[6], fw_c[7], fw_c[8], fw_c[11], fw_c[9], fw_c[10]);
      for (dsi = 1; dsi < IW; dsi = dsi + 1)
         $display("DISP-SIM slot %c cut behind a dispatching slot %c, cycles: alone=%0d load=%0d store=%0d divide=%0d resteer=%0d | credit: lane=%0d load=%0d store=%0d fp=%0d divide=%0d",
                  8'("A" + dsi), 8'("A" + dsi - 1), ds_cut[dsi][0], ds_cut[dsi][8], ds_cut[dsi][9], ds_cut[dsi][1],
                  ds_cut[dsi][2], ds_cut[dsi][3], ds_cut[dsi][4], ds_cut[dsi][5], ds_cut[dsi][6], ds_cut[dsi][7]);
      $display("DISP-SIM the dispatching groups took %0d instructions; at most more with any number of loads=%0d stores=%0d divides=%0d | loads+stores=%0d loads+divides=%0d stores+divides=%0d all three=%0d",
               ds_n, ds_rx[1], ds_rx[2], ds_rx[4], ds_rx[3], ds_rx[5], ds_rx[6], ds_rx[7]);
      $display("DISP-SIM queue head held with pops allowed (a serialise, CSR or CBO gate, the ROB or a room credit), cycles=%0d", ds_hd);
      $display("DISP-SIM lane schedulers: ready-not-picked entry-cycles=%0d (another lane picking nothing %0d) picks-past-an-older-ready=%0d | M-started accesses=%0d back-to-back=%0d",
               al_wait, al_wait_idle, al_young, ls_st, ls_b2b);
      $display("SER-SIM be:serialize cycles by class: csr-read=%0d csr-write=%0d amo=%0d fence=%0d fence.i=%0d cbo=%0d sfence=%0d xret/ecall/ebreak/wfi/irq=%0d trap=%0d",
               ser_c[0], ser_c[1], ser_c[2], ser_c[3], ser_c[4], ser_c[5], ser_c[6], ser_c[7], ser_c[8]);
      $display("IQFULL-SIM be:iq-full cycles by the head's missing credit (a cycle can miss two): its-lane=%0d head-op=%0d fp-queue=%0d divide=%0d | its lane's alone=%0d, of them another lane with room=%0d",
               iq_c[0], iq_c[1], iq_c[2], iq_c[3], iq_c[4], iq_c[5]);
      $write("ROT-SIM instructions dispatched by lane:");
      for (iqi = 0; iqi < IW; iqi = iqi + 1) $write(" %s=%0d", 8'("A" + iqi), rot_n[iqi]);
      $display(" | cycles a head op waited for lane A=%0d", rot_ra);
      $display("DEPLD-SIM be:dep-load cycles by the load waited on: not-issued=%0d at-M=%0d dtlb-miss=%0d faulting=%0d ready=%0d held-unknown=%0d held-overlap=%0d device-head=%0d forwarding=%0d dcache-waiter=%0d dcache=%0d slow-path=%0d | landing-buffer=%0d sysq-csr=%0d amo-in-M=%0d",
               dl_c[0], dl_c[1], dl_c[2], dl_c[3], dl_c[4], dl_c[5], dl_c[6], dl_c[7], dl_c[8], dl_c[9], dl_c[10],
               dl_c[11], dl_c[12], dl_c[13], dl_c[14]);
      $display("ICMISS-SIM misses=%0d unused=%0d (a missing line nothing retired from before 64 more misses)", im_n, im_waste);
      $display("TRAIN-SIM trainings=%0d dropped-CTIs=%0d", tr_n, tr_drops);
      $display("WALK-SIM squashes-at-the-head=%0d checked-after-the-walk=%0d rows-walked=%0d checked-after-the-kill=%0d releases=%0d", dut.core.wk_sq, dut.core.wk_chk, dut.core.wk_rows, dut.core.wk_kchk, dut.core.wk_rel);
      $display("TRAP-SIM cycles/retires by mode: U=%0d/%0d S=%0d/%0d M=%0d/%0d", trc_cyc[0], trc_ret[0], trc_cyc[1], trc_ret[1], trc_cyc[3], trc_ret[3]);
      $write("TRAP-SIM traps, cause:to-M/to-S (16+n: interrupt n):");
      for (tri_ = 0; tri_ < 32; tri_ = tri_ + 1)
         if (trc_n[tri_][0] != 0 || trc_n[tri_][1] != 0) $write(" %0d:%0d/%0d", tri_, trc_n[tri_][0], trc_n[tri_][1]);
      $display("");
      for (tri_ = 0; tri_ < trp_used; tri_ = tri_ + 1)
         $display("TRAP-SIM to-M pc=%h cause=%0d count=%0d last-tval=%h", trp_pc[tri_], trp_k[tri_], trp_n[tri_], trp_tv[tri_]);
      $display("PORT-SIM a ready queued load not taken, cycles: store-drain=%0d lsu-busy=%0d m-walk=%0d dcache-door=%0d tag-due=%0d other=%0d",
               pt_n[0], pt_n[1], pt_n[2], pt_n[3], pt_n[4], pt_n[5]);
      $display("PORTS-SIM cycles: ready-loads>=1=%0d >=2=%0d | a ready load and a committed store=%0d, a store alone=%0d | two or more arrived loads waiting for M=%0d",
               po_ld1, po_ld2, po_ldst, po_st, po_ar2);
      $display("DTLB-SIM walker busy=%0d cycles | second-level lookups=%0d answered=%0d | walks=%0d pte-reads=%0d", dt_busy, dt_lk, dt_l2, dt_walk, dt_pte);
      $display("MEM-SIM dcache fill-cycles=%0d fills=%0d mean-mshrs=%0.2f waiting-cycles=%0d wb-full-cycles=%0d | ST_MEM cycles=%0d with-fill=%0d with-waiting=%0d",
               ms_fill, ms_fills, (ms_fill != 0) ? $itor(ms_live) / $itor(ms_fill) : 0.0, ms_park, ms_wbfull,
               ms_stmem, ms_stmem_fill, ms_stmem_park);
      $display("STLK-SIM store-queue cycles: lookup-writes=%0d lookup-merges=%0d | head waits: fill-landing=%0d main-lookup-load-in-set=%0d main-lookup-other-in-set=%0d fill-beat=%0d install-same-way=%0d install-other-way=%0d | misses=%0d sent-back-behind-head=%0d queue-full=%0d main-lookup-done=%0d queue-not-empty=%0d",
               sl_wr, sl_mg, sl_fw, sl_s1ld, sl_s1x, sl_fb, sl_flw, sl_flx, sl_miss, sl_back, sl_full, sl_main, sl_idle);
      $display("STLK-SIM a plain store in the main lookup: allocates=%0d deferred=%0d waits=%0d", sm_alloc, sm_defer, sm_wait);
      if (kan_on) begin kan_finish; $fclose(kf); end
      $display("RECOV-SIM restarts=%0d released=%0d squashed-at-the-head=%0d superseded=%0d | cycles: recovery=%0d refetch=%0d exposed=%0d | rows walked back=%0d, forward rebuild (sweep + rows from the head)=%0d | exposed with the rebuild racing=%0d, the rebuild faster in %0d restarts",
               rv_n, rv_rel, rv_sq, rv_sup, rv_rec, rv_fe_sum, rv_now, rv_rows_b, rv_rows_f, rv_best, rv_win);
      $display("RECOV-SIM exposed with three checkpoints racing, every 4 rows=%0d 8 rows=%0d 16 rows=%0d, after weak branches=%0d (weakly predicted restarts=%0d)",
               rv_cke[0], rv_cke[1], rv_cke[2], rv_cwe, rv_nwk);
      $display("RECOV-SIM exposed with a compressed walk=%0d (distinct registers restored by released walks=%0d)", rv_cwk, rv_dist);
      $display("MFUN-SIM plain loads and stores M filled=%0d | waiting in the arrival RAMs: entry-cycles=%0d, cycles one or more=%0d, two or more=%0d", mf_fill, mf_wait, mf_c1, mf_c2);
      $display("LDPATH-SIM loads M filled, from the lane/from the arrival RAM: started in M's cycle and stood=%0d/%0d, cancelled=%0d/%0d | to the queue: not translated=%0d/%0d, the D$ took no load=%0d/%0d, other=%0d/%0d",
               lp_n[1], lp_n[0], lp_n[3], lp_n[2], lp_n[5], lp_n[4], lp_n[7], lp_n[6], lp_n[9], lp_n[8]);
      $display("LAND-SIM integer loads landing straight through=%0d, into the lane's landing buffer=%0d | lane-cycles held by a waiting landing=%0d",
               ln_by, ln_buf, ln_hold);
      $display("HITSPEC-SIM load lookups in their compare, right/cancelled/missed-or-waiting: lookups=%0d/%0d/%0d | consumers waiting on them=%0d/%0d/%0d, waiting on them alone=%0d/%0d/%0d",
               hs_n[0], hs_n[1], hs_n[2], hs_w[0], hs_w[1], hs_w[2], hs_o[0], hs_o[1], hs_o[2]);
      $display("HITSPEC-SIM of the consumers, address generators (a load or store), right/cancelled/missed-or-waiting: waiting=%0d/%0d/%0d, waiting on them alone=%0d/%0d/%0d",
               hs_a[0], hs_a[1], hs_a[2], hs_ao[0], hs_ao[1], hs_ao[2]);
      $display("SPEC-SIM speculative wakes=%0d | address generators issued on one: delivered=%0d suppressed=%0d", sp_wake, sp_ok, sp_bad);
      $display("perf-stat-sim: begin");
      $display("%0d cycles", mc);          // the cycles counted: the window, or the whole run
      $display("%0d instructions", mr);
      for (pe = 0; pe < pe_n; pe = pe + 1) $display("%0d r%04h", pev[pe], pe_code[pe]);
      $display("perf-stat-sim: end");

      $display("MEMDEP replays=%0d", dut.core.rp_cnt);
      $display("SQ-SIM data landings=%0d left-out=%0d (fetched by the data read instead)", dut.core.u_sq.ln_n, dut.core.u_sq.ln_out);
      $display("FPORT-SIM retired instructions: divides=%0d int-to-FP=%0d csr/sfence reading rs1=%0d | stores=%0d FP stores=%0d | the store queue's data reads=%0d",
               fx_div, fx_i2f, fx_sys, fx_st, fx_fst, fx_rd);
      $display("BTB-SIM trainings=%0d carried-hit=%0d | against the slots now: duplicate allocations=%0d stale rewrites=%0d",
               bt_n, bt_car, bt_dup, bt_stale);
      $display("L1BTB-SIM windows=%0d corrections=%0d mark-fills=%0d", l1_wn, l1_cr, l1_mf);
      $write("RENAME-SIM be:rename cycles each shard is low, and the sweep (a cycle can count in several):");
      for (rli = 0; rli < 8; rli = rli + 1)
         if (dut.core.sh_used(3'(rli))) begin
            if (rli < IW)                   $write(" lane-%c=%0d", 8'("A" + rli), rn_low[rli]);
            else if (rli == 32'(SH_FL))     $write(" fp-loads=%0d", rn_low[rli]);
            else                            $write(" f-stage=%0d", rn_low[rli]);
         end
      $write(" sweep=%0d", rn_low[8]);
      $display("");
      $display("SWEEP-SIM squashes=%0d dirty=%0d exact=%0d walk-only=%0d pick-cycles=%0d busy-cycles=%0d", dut.core.u_rename.sw_n, dut.core.u_rename.sw_d, dut.core.u_rename.sw_x, dut.core.u_rename.sw_w, dut.core.u_rename.sw_g, dut.core.u_rename.sw_c);
      $display("MEMDEP loads blocked only by unknown older store addresses: resolved clear=%0d (waited %0d cycles), resolved aliased=%0d (waited %0d cycles)",
               dut.core.u_sq.sp_clear, dut.core.u_sq.sp_clear_cyc, dut.core.u_sq.sp_alias, dut.core.u_sq.sp_alias_cyc);
      $display("SQ  queued-load starts=%0d, past an older store=%0d (%0d.%0d%%) | alias-held cycles=%0d (%0d.%0d%%) | mean occupancy=%0d.%02d entries | full cycles=%0d",
               n_ldstart, n_ldreord,
               n_ldstart ? (100*n_ldreord)/n_ldstart : 0,
               n_ldstart ? ((1000*n_ldreord)/n_ldstart)%10 : 0,
               n_ldblk, (100*n_ldblk)/mc, ((1000*n_ldblk)/mc)%10,
               n_sqocc/mc, ((100*n_sqocc)/mc)%100, n_sqfull);
      lq_n = lq_cbo + lq_full + lq_two;
      $display("LQ-SIM mean occupancy=%0d.%02d entries | no-credit cycles=%0d: cbo=%0d full=%0d two=%0d | entries in them, mean: no-address=%0d.%02d not-sent=%0d.%02d (alias-held=%0d.%02d untranslated=%0d.%02d) at-memory=%0d.%02d landed-watched=%0d.%02d",
               n_lqocc/mc, ((100*n_lqocc)/mc)%100, lq_n, lq_cbo, lq_full, lq_two,
               lq_n ? lq_noa/lq_n : 0,  lq_n ? ((100*lq_noa)/lq_n)%100 : 0,
               lq_n ? lq_wait/lq_n : 0, lq_n ? ((100*lq_wait)/lq_n)%100 : 0,
               lq_n ? lq_blk/lq_n : 0,  lq_n ? ((100*lq_blk)/lq_n)%100 : 0,
               lq_n ? lq_utv/lq_n : 0,  lq_n ? ((100*lq_utv)/lq_n)%100 : 0,
               lq_n ? lq_mem/lq_n : 0,  lq_n ? ((100*lq_mem)/lq_n)%100 : 0,
               lq_n ? lq_lnd/lq_n : 0,  lq_n ? ((100*lq_lnd)/lq_n)%100 : 0);
      $display("SQ-DRAIN cycles: door-unaccepted=%0d drainable-LSU-idle=%0d head-not-drainable=%0d", n_stdoor, n_sqidle, n_sqwait);
      // every cycle in one top-down cause, and the dispatching causes account for every dispatch
      // outside a redirect's cycle: a cycle the IR's group dispatches is a dispatching cycle
      // (smolrv64_core td_disp) unless it is bad speculation
      if (td_sum != mc)
         $fatal(1, "TOPDOWN-SIM NOT CLOSED: %0d cycles counted in a cause, %0d run", td_sum, mc);
      if (td_disp != td_dn)
         $fatal(1, "TOPDOWN-SIM: the ok:N cycles dispatch %0d instructions, the IR took %0d outside redirect cycles", td_disp, td_dn);
      $finish;
   end
endmodule

`default_nettype wire

`ifdef TB_DDR4
 `define DDR4_PHY_STORE tb.lram
 `include "ddr4_phy_model.vh"
`endif
