// tb_virtio_net -- the virtio-net backend against an AXI memory model and stub engines.
//
// No system bench ever drove this device: tb_virtio has no net backend and the cosim
// forwards the board's real device writes. So the DMA engine that fetched every TX word
// with its own round trip and wrote every RX byte with its own AXI transaction shipped
// unmeasured (plan item 7, 2026-09-05). This bench posts one TX frame and one RX buffer
// the way Linux does, checks every byte and the used rings, and prints the cycles per
// frame each way at a DDR-like AXI latency, so the number is a gate, not a guess.
//
//   ./ooo2/run-ooo2-vnet-tb.sh          VNET-TB PASS tx=<cycles/frame> rx=<cycles/frame>
`timescale 1ns/1ps
`default_nettype none
module tb;
   localparam QS   = 256;                 // the board's queue depth
   localparam RLAT = 28, WLAT = 15;       // DDR through the MIG, measured 2026-09-04 (mean cycles)
   localparam FLEN = 1500;                // a full-size frame
   reg clk = 0; always #3 clk = ~clk;
   reg reset = 1;

   // ---- MMIO-side configuration (what virtio_mmio would present) ----
   reg         notify = 0; reg [31:0] notify_q = 0;
   reg [63:0]  txd = 64'h1000, txa = 64'h2000, txu = 64'h3000;   // TX desc / avail (driver) / used (device)
   reg [63:0]  rxd = 64'h4000, rxa = 64'h5000, rxu = 64'h6000;
   reg         qready = 0; reg [7:0] status = 0;
   wire        irq;
   // ---- engines ----
   wire        tx_wr_en; wire [10:0] tx_wr_addr; wire [7:0] tx_wr_data; wire tx_send; wire [10:0] tx_send_len;
   reg         tx_busy = 0;
   reg         rx_frame_valid = 0; reg [10:0] rx_frame_len = 0;
   wire [10:0] rx_rd_addr; wire rx_frame_ack;
   reg  [7:0]  rxbuf [0:2047]; wire [7:0] rx_rd_data = rxbuf[rx_rd_addr];   // async-read, like eth_rx_engine
   reg  [7:0]  txbuf [0:2047];
   always @(posedge clk) if (tx_wr_en) txbuf[tx_wr_addr] <= tx_wr_data;
   // ---- AXI ----
   wire [2:0] awid, arid; wire [30:0] awaddr, araddr; wire [7:0] awlen, arlen; wire [2:0] awsize, arsize;
   wire [1:0] awburst, arburst; wire awlock, arlock; wire [3:0] awcache, arcache, awqos, arqos; wire [2:0] awprot, arprot;
   wire awvalid, wvalid, bready, arvalid, rready, wlast; wire [63:0] wdata; wire [7:0] wstrb;
   reg  awready = 0, wready = 0, bvalid = 0, arready = 0, rvalid = 0; reg [63:0] rdata = 0;

   virtio_net #(.QUEUE_SIZE(QS)) dut (
      .clock(clk), .reset(reset),
      .queue_notify_pulse(notify), .queue_notify_value(notify_q),
      .tx_queue_num(QS), .tx_queue_ready(qready), .tx_queue_desc(txd), .tx_queue_driver(txa), .tx_queue_device(txu),
      .device_status(status), .used_buffer_interrupt(irq),
      .debug_status(), .debug_notify_count(), .debug_read_avail_count(), .debug_empty_avail_count(),
      .debug_read_ring_count(), .debug_complete_count(), .debug_irq_count(), .debug_dma_error_count(),
      .debug_indices(), .debug_used_head(), .debug_last_avail_word_lo(), .debug_last_avail_word_hi(),
      .debug_last_ring_word_lo(), .debug_last_ring_word_hi(),
      .tx_wr_en(tx_wr_en), .tx_wr_addr(tx_wr_addr), .tx_wr_data(tx_wr_data), .tx_send(tx_send), .tx_send_len(tx_send_len),
      .tx_busy(tx_busy), .debug_tx_frame_count(), .debug_tx_last_len(), .debug_tx_desc_addr(), .debug_tx_desc_len(),
      .rx_queue_num(QS), .rx_queue_ready(qready), .rx_queue_desc(rxd), .rx_queue_driver(rxa), .rx_queue_device(rxu),
      .rx_frame_valid(rx_frame_valid), .rx_frame_len(rx_frame_len), .rx_rd_addr(rx_rd_addr), .rx_rd_data(rx_rd_data),
      .rx_frame_ack(rx_frame_ack), .debug_rx_deliver_count(), .debug_rx_nobuf_count(),
      .m_axi_awid(awid), .m_axi_awaddr(awaddr), .m_axi_awlen(awlen), .m_axi_awsize(awsize), .m_axi_awburst(awburst),
      .m_axi_awlock(awlock), .m_axi_awcache(awcache), .m_axi_awprot(awprot), .m_axi_awqos(awqos),
      .m_axi_awvalid(awvalid), .m_axi_awready(awready),
      .m_axi_wdata(wdata), .m_axi_wstrb(wstrb), .m_axi_wlast(wlast), .m_axi_wvalid(wvalid), .m_axi_wready(wready),
      .m_axi_bid(3'd0), .m_axi_bresp(2'b00), .m_axi_bvalid(bvalid), .m_axi_bready(bready),
      .m_axi_arid(arid), .m_axi_araddr(araddr), .m_axi_arlen(arlen), .m_axi_arsize(arsize), .m_axi_arburst(arburst),
      .m_axi_arlock(arlock), .m_axi_arcache(arcache), .m_axi_arprot(arprot), .m_axi_arqos(arqos),
      .m_axi_arvalid(arvalid), .m_axi_arready(arready),
      .m_axi_rid(3'd0), .m_axi_rdata(rdata), .m_axi_rresp(2'b00), .m_axi_rlast(1'b1), .m_axi_rvalid(rvalid), .m_axi_rready(rready));

   // ---- AXI memory: 2 MiB of 64-bit words, one transaction at a time, DDR-like latency ----
   reg [63:0] mem [0:(1<<18)-1];
   integer mi; initial for (mi = 0; mi < (1<<18); mi = mi + 1) mem[mi] = 64'hEEEE_EEEE_EEEE_EEEE;
   function [63:0] merge(input [63:0] old, input [63:0] d, input [7:0] s);
      integer b; begin merge = old; for (b = 0; b < 8; b = b + 1) if (s[b]) merge[b*8 +: 8] = d[b*8 +: 8]; end
   endfunction
   reg [30:0] wa; reg [63:0] wd; reg [7:0] ws; reg aw_got = 0, w_got = 0; integer wcnt = 0, rcnt = 0;
   integer n_wr = 0, n_rd = 0;
   always @(posedge clk) begin
      awready <= 1'b0; wready <= 1'b0; bvalid <= 1'b0; arready <= 1'b0; rvalid <= 1'b0;
      if (awvalid && !aw_got && !awready) begin wa <= awaddr; aw_got <= 1'b1; awready <= 1'b1; end
      if (wvalid && !w_got && !wready)    begin wd <= wdata; ws <= wstrb; w_got <= 1'b1; wready <= 1'b1; end
      if (aw_got && w_got && wcnt == 0) wcnt <= WLAT;
      if (wcnt > 1) wcnt <= wcnt - 1;
      else if (wcnt == 1) begin
         mem[wa[20:3]] <= merge(mem[wa[20:3]], wd, ws); n_wr <= n_wr + 1;
         bvalid <= 1'b1; aw_got <= 1'b0; w_got <= 1'b0; wcnt <= 0;
      end
      if (arvalid && rcnt == 0 && !arready) begin arready <= 1'b1; rcnt <= RLAT; rdata <= mem[araddr[20:3]]; end
      if (rcnt > 1) rcnt <= rcnt - 1;
      else if (rcnt == 1) begin rvalid <= 1'b1; rcnt <= 0; n_rd <= n_rd + 1; end
   end
   // byte access helpers for the tb
   task put8(input [30:0] a, input [7:0] v);  begin mem[a[20:3]][a[2:0]*8 +: 8] = v; end endtask
   function [7:0] get8(input [30:0] a);       begin get8 = mem[a[20:3]][a[2:0]*8 +: 8]; end endfunction
   task put16(input [30:0] a, input [15:0] v); begin put8(a, v[7:0]); put8(a+1, v[15:8]); end endtask
   task put32(input [30:0] a, input [31:0] v); begin put16(a, v[15:0]); put16(a+2, v[31:16]); end endtask
   task put64(input [30:0] a, input [63:0] v); begin put32(a, v[31:0]); put32(a+4, v[63:32]); end endtask
   function [15:0] get16(input [30:0] a); begin get16 = {get8(a+1), get8(a)}; end endfunction
   function [31:0] get32(input [30:0] a); begin get32 = {get16(a+2), get16(a)}; end endfunction
   function [7:0] pat(input integer i); begin pat = i[7:0] ^ {i[11:8], 4'h5}; end endfunction

   integer errors = 0, i, t0, tx_cyc, rx_cyc, cyc = 0;
   always @(posedge clk) cyc <= cyc + 1;
   reg [10:0] sent_len; reg sent = 0;
   always @(posedge clk) if (tx_send) begin sent <= 1'b1; sent_len <= tx_send_len; end
   task step; begin @(posedge clk); #1; end endtask
   task chk(input cond, input [511:0] what); begin if (!cond) begin $display("FAIL %0s", what); errors = errors + 1; end end endtask

   reg [30:0] buf_tx, buf_rx; integer off, n, flen, tx_first, rx_first;
   // TX frames of every alignment and a few lengths (the first word's byte offset and the
   // last word's lane both vary), each its own descriptor and avail slot; RX buffers at
   // every byte offset, so the gathered word's head and tail strobes are all exercised.
   integer tx_len [0:7];  initial begin tx_len[0]=1500; tx_len[1]=60; tx_len[2]=61; tx_len[3]=1514; tx_len[4]=64; tx_len[5]=65; tx_len[6]=1499; tx_len[7]=200; end
   integer rx_len [0:7];  initial begin rx_len[0]=1500; rx_len[1]=60; rx_len[2]=1514; rx_len[3]=61; rx_len[4]=1000; rx_len[5]=64; rx_len[6]=1499; rx_len[7]=65; end
   initial begin
      repeat (4) step; reset = 0; repeat (2) step;
      qready = 1; status = 8'h04; step;
      put16(txa[30:0], 16'd0); put16(txa[30:0] + 2, 16'd0); put16(txu[30:0], 16'd0); put16(txu[30:0] + 2, 16'd0);
      put16(rxa[30:0], 16'd0); put16(rxa[30:0] + 2, 16'd0); put16(rxu[30:0], 16'd0); put16(rxu[30:0] + 2, 16'd0);
      tx_first = 0; rx_first = 0;
      // ================= TX =================
      for (n = 0; n < 8; n = n + 1) begin
         flen = tx_len[n]; off = n;                                        // desc payload at offset n
         buf_tx = 31'h10000 + n * 31'h1000 + off;
         for (i = 0; i < 12; i = i + 1) put8(buf_tx + i, 8'h00);           // virtio_net_hdr_v1
         for (i = 0; i < flen; i = i + 1) put8(buf_tx + 12 + i, pat(i + n));
         put64(txd[30:0] + n*16,      {33'd0, buf_tx});                   // desc[n]
         put32(txd[30:0] + n*16 + 8,  32'd12 + flen);
         put16(txd[30:0] + n*16 + 12, 16'd0); put16(txd[30:0] + n*16 + 14, 16'd0);
         put16(txa[30:0] + 4 + n*2, n[15:0]);                              // avail.ring[n] = desc n
         put16(txa[30:0] + 2, n[15:0] + 16'd1);                            // avail.idx
         for (i = 0; i < 2048; i = i + 1) txbuf[i] = 8'hEE;
         sent = 0; t0 = cyc; notify = 1; notify_q = 1; step; notify = 0;
         i = 0; while (!sent && i < 400000) begin step; i = i + 1; end
         chk(sent, "TX: no tx_send");
         if (n == 0) tx_cyc = cyc - t0;
         chk(sent_len == flen, "TX: send length");
         for (i = 0; i < flen; i = i + 1) if (txbuf[i] !== pat(i + n)) begin errors = errors + 1; if (errors < 6) $display("FAIL TX case %0d byte %0d: %h want %h", n, i, txbuf[i], pat(i + n)); end
         chk(txbuf[flen] == 8'hEE, "TX: wrote past the frame");
         tx_busy = 1; repeat (20) step; tx_busy = 0;
         i = 0; while (get16(txu[30:0] + 2) != n[15:0] + 16'd1 && i < 4000) begin step; i = i + 1; end
         chk(get16(txu[30:0] + 2) == n[15:0] + 16'd1, "TX: used.idx");
         chk(get32(txu[30:0] + 4 + n*8) == n, "TX: used.ring[n].id");
         repeat (4) step;
      end
      // ================= RX =================
      for (n = 0; n < 8; n = n + 1) begin
         flen = rx_len[n]; off = n;
         buf_rx = 31'h20000 + n * 31'h1000 + off;
         for (i = -8; i < 12 + flen + 8; i = i + 1) put8(buf_rx + i, 8'hA5);   // sentinel around and inside
         put64(rxd[30:0] + n*16, {33'd0, buf_rx}); put32(rxd[30:0] + n*16 + 8, 32'd2048);
         put16(rxd[30:0] + n*16 + 12, 16'd2); put16(rxd[30:0] + n*16 + 14, 16'd0);   // VIRTQ_DESC_F_WRITE
         put16(rxa[30:0] + 4 + n*2, n[15:0]); put16(rxa[30:0] + 2, n[15:0] + 16'd1);
         for (i = 0; i < flen; i = i + 1) rxbuf[i] = pat(i + 77 + n);
         rx_frame_len = flen; t0 = cyc; rx_frame_valid = 1;
         i = 0; while (!rx_frame_ack && i < 400000) begin step; i = i + 1; end
         chk(rx_frame_ack, "RX: no frame_ack");
         if (n == 0) rx_cyc = cyc - t0;
         rx_frame_valid = 0; step;
         for (i = 0; i < 12; i = i + 1) if (get8(buf_rx + i) !== (i == 10 ? 8'h01 : 8'h00)) begin errors = errors + 1; if (errors < 6) $display("FAIL RX case %0d hdr byte %0d: %h", n, i, get8(buf_rx + i)); end
         for (i = 0; i < flen; i = i + 1) if (get8(buf_rx + 12 + i) !== pat(i + 77 + n)) begin errors = errors + 1; if (errors < 6) $display("FAIL RX case %0d byte %0d: %h want %h", n, i, get8(buf_rx + 12 + i), pat(i + 77 + n)); end
         for (i = 1; i <= 8; i = i + 1) begin
            chk(get8(buf_rx - i) == 8'hA5, "RX: wrote below the buffer");
            chk(get8(buf_rx + 12 + flen + i - 1) == 8'hA5, "RX: wrote past the frame");
         end
         i = 0; while (get16(rxu[30:0] + 2) != n[15:0] + 16'd1 && i < 4000) begin step; i = i + 1; end
         chk(get16(rxu[30:0] + 2) == n[15:0] + 16'd1, "RX: used.idx");
         chk(get32(rxu[30:0] + 4 + n*8) == n, "RX: used.ring[n].id");
         chk(get32(rxu[30:0] + 8 + n*8) == 32'd12 + flen, "RX: used.ring[n].len");
         repeat (4) step;
      end
      n_rd = n_rd; // (counts cover all cases; the per-frame figures are case 0, 1500 bytes)
      $display("VNET-TB tx=%0d rx=%0d cycles per %0d-byte frame (8 TX + 8 RX cases, every alignment; RLAT=%0d WLAT=%0d)",
               tx_cyc, rx_cyc, FLEN, RLAT, WLAT);
      if (errors == 0) $display("VNET-TB PASS"); else begin $display("VNET-TB FAIL (%0d errors)", errors); $fatal(1, "tb_virtio_net FAILED"); end
      $finish;
   end
endmodule
`default_nettype wire
