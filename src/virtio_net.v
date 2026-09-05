`timescale 1ns / 1ps
`default_nettype none

module virtio_net #(
    parameter [31:0] QUEUE_SIZE = 32'd8  /* virtqueue depth, power of two */
)(
    input  wire        clock,
    input  wire        reset,

    input  wire        queue_notify_pulse,
    input  wire [31:0] queue_notify_value,
    input  wire [31:0] tx_queue_num,
    input  wire        tx_queue_ready,
    input  wire [63:0] tx_queue_desc,
    input  wire [63:0] tx_queue_driver,
    input  wire [63:0] tx_queue_device,
    input  wire [ 7:0] device_status,
    output reg         used_buffer_interrupt,
    output wire [31:0] debug_status,
    output wire [31:0] debug_notify_count,
    output wire [31:0] debug_read_avail_count,
    output wire [31:0] debug_empty_avail_count,
    output wire [31:0] debug_read_ring_count,
    output wire [31:0] debug_complete_count,
    output wire [31:0] debug_irq_count,
    output wire [31:0] debug_dma_error_count,
    output wire [31:0] debug_indices,
    output wire [31:0] debug_used_head,
    output wire [31:0] debug_last_avail_word_lo,
    output wire [31:0] debug_last_avail_word_hi,
    output wire [31:0] debug_last_ring_word_lo,
    output wire [31:0] debug_last_ring_word_hi,

    // TX frame engine (ui_clk side of eth_tx_engine).  The backend DMAs the
    // outgoing L2 frame (descriptor payload past the 12-byte virtio_net_hdr)
    // into the engine buffer, then pulses tx_send; the used-ring completion
    // still runs regardless, so virtio TX never regresses even if a frame is
    // mis-sized on first silicon.
    output reg         tx_wr_en,
    output reg  [10:0] tx_wr_addr,
    output reg  [ 7:0] tx_wr_data,
    output reg         tx_send,
    output reg  [10:0] tx_send_len,
    input  wire        tx_busy,
    output wire [31:0] debug_tx_frame_count,
    output wire [31:0] debug_tx_last_len,
    output wire [31:0] debug_tx_desc_addr,  // guest addr of the desc payload
    output wire [31:0] debug_tx_desc_len,   // descriptor length (= 12 + frame)

    // RX virtqueue (queue 0) + eth_rx_engine (ui side).  When the engine has a
    // received frame, the backend takes a free buffer from the RX avail ring,
    // DMAs a zeroed 12-byte virtio_net_hdr_v1 + the frame into it, completes
    // the RX used ring, and raises the interrupt.
    input  wire [31:0] rx_queue_num,
    input  wire        rx_queue_ready,
    input  wire [63:0] rx_queue_desc,
    input  wire [63:0] rx_queue_driver,
    input  wire [63:0] rx_queue_device,
    input  wire        rx_frame_valid,      // eth_rx_engine has a frame
    input  wire [10:0] rx_frame_len,
    output wire [10:0] rx_rd_addr,          // read frame bytes from the engine (registered read: leads by a cycle)
    input  wire [ 7:0] rx_rd_data,
    output reg         rx_frame_ack,        // release the engine buffer
    output wire [31:0] debug_rx_deliver_count,
    output wire [31:0] debug_rx_nobuf_count,

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
   localparam [4:0] S_IDLE           = 5'd0;
   localparam [4:0] S_READ_AVAIL     = 5'd1;
   localparam [4:0] S_WAIT_AVAIL     = 5'd2;
   localparam [4:0] S_READ_RING      = 5'd3;
   localparam [4:0] S_WAIT_RING      = 5'd4;
   localparam [4:0] S_WRITE_USED_ID  = 5'd5;
   localparam [4:0] S_WAIT_USED_ID   = 5'd6;
   localparam [4:0] S_WRITE_USED_LEN = 5'd7;
   localparam [4:0] S_WAIT_USED_LEN  = 5'd8;
   localparam [4:0] S_WRITE_USED_IDX = 5'd9;
   localparam [4:0] S_WAIT_USED_IDX  = 5'd10;
   localparam [4:0] S_COMPLETE       = 5'd11;
   localparam [4:0] S_RETRY_WAIT     = 5'd12;
   // TX frame fetch: read the descriptor, DMA the frame into eth_tx_engine.
   localparam [4:0] S_DESC_A         = 5'd13;  // read desc[head] addr
   localparam [4:0] S_WAIT_DESC_A    = 5'd14;
   localparam [4:0] S_DESC_B         = 5'd15;  // read desc[head] len/flags/next
   localparam [4:0] S_WAIT_DESC_B    = 5'd16;
   localparam [4:0] S_FRAME_RD       = 5'd17;  // read a 64-bit frame word
   localparam [4:0] S_WAIT_FRAME     = 5'd18;
   localparam [4:0] S_FILL           = 5'd19;  // byte-write the word into engine
   localparam [4:0] S_SEND           = 5'd20;  // pulse tx_send
   localparam [4:0] S_WAIT_SEND      = 5'd21;  // wait for TX to drain
   // RX delivery (queue 0): take a free buffer, DMA hdr+frame in, complete.
   localparam [5:0] S_RX_AVAIL       = 6'd22;
   localparam [5:0] S_RX_WAIT_AVAIL  = 6'd23;
   localparam [5:0] S_RX_RING        = 6'd24;
   localparam [5:0] S_RX_WAIT_RING   = 6'd25;
   localparam [5:0] S_RX_DESC        = 6'd26;
   localparam [5:0] S_RX_WAIT_DESC   = 6'd27;
   localparam [5:0] S_RX_WRITE       = 6'd28;  // write one byte (hdr or frame)
   localparam [5:0] S_RX_WRITE_WAIT  = 6'd29;
   localparam [5:0] S_RX_USED_ID     = 6'd30;
   localparam [5:0] S_RX_WAIT_USED_ID  = 6'd31;
   localparam [5:0] S_RX_USED_LEN    = 6'd32;
   localparam [5:0] S_RX_WAIT_USED_LEN = 6'd33;
   localparam [5:0] S_RX_USED_IDX    = 6'd34;
   localparam [5:0] S_RX_WAIT_USED_IDX = 6'd35;
   localparam [5:0] S_RX_COMPLETE    = 6'd36;
   localparam [5:0] S_RX_DROP        = 6'd37;  // no free buffer: drop the frame
   localparam [5:0] S_RX_GATHER      = 6'd38;  // gather up to 8 bytes into the write word
   localparam [5:0] S_WAIT_NEXT      = 6'd39;  // TX: the read-ahead word has not landed yet
   localparam [5:0] S_TX_FLAGS       = 6'd40;  // re-read avail.flags after used.idx landed
   localparam [5:0] S_TX_FLAGS_WAIT  = 6'd41;
   localparam [5:0] S_RX_FLAGS       = 6'd42;
   localparam [5:0] S_RX_FLAGS_WAIT  = 6'd43;

   localparam [ 7:0] EMPTY_RETRY_COUNT = 8'hff;
   localparam [15:0] EMPTY_RETRY_DELAY = 16'hffff;
   reg [5:0]  state;
   reg [15:0] last_avail_idx;
   reg [15:0] avail_idx;
   reg [15:0] used_idx;
   reg [15:0] head_desc;
   reg        notify_pending;
   reg [ 7:0] empty_retry_count;
   reg [15:0] retry_delay;
   reg [31:0] notify_count;
   reg [31:0] read_avail_count;
   reg [31:0] empty_avail_count;
   reg [31:0] read_ring_count;
   reg [31:0] complete_count;
   reg [31:0] irq_count;
   reg [31:0] dma_error_count;
   reg [63:0] last_avail_word;
   reg [63:0] last_ring_word;

   // TX frame-fetch working state
   reg [63:0] desc_addr;       // guest address of descriptor payload (hdr+frame)
   reg [10:0] frame_total;     // frame bytes to transmit (= desc len - 12)
   reg [10:0] byte_idx;        // bytes written into the engine so far
   reg [63:0] cur_word;        // current 64-bit DMA word being unpacked
   reg [ 2:0] word_byte;       // next byte within cur_word
   reg        first_word;      // apply start_off only to the first word read
   reg [ 2:0] start_off;       // byte offset of frame[0] within its aligned word
   reg        tx_busy_seen;    // saw eth_tx_engine accept the send
   reg [19:0] tx_timeout;      // guard: complete even if the link/TX never drains
   reg [31:0] tx_frame_count;
   reg [10:0] tx_last_len;
   reg [31:0] tx_dbg_desc_addr;
   reg [31:0] tx_dbg_desc_len;

   // RX delivery working state
   reg [15:0] rx_last_avail_idx;
   reg [15:0] rx_avail_idx;
   reg [15:0] rx_used_idx;
   reg [15:0] rx_head_desc;
   reg [63:0] rx_buf_addr;
   reg [10:0] rx_widx;          // bytes written into the guest buffer
   reg [10:0] rx_total;         // 12 + frame_len
   reg [31:0] rx_deliver_count;
   reg [31:0] rx_nobuf_count;

   // VRING_AVAIL_F_NO_INTERRUPT (avail ring flags bit 0), captured on each
   // avail-word read.  NAPI sets it on the first IRQ and polls; a device that
   // ignores it interrupts per frame, which at 67 MHz under an NFS retransmit
   // storm is an IRQ livelock that freezes the whole system (console + ping
   // dead).  Suppression is a hint, so a one-frame-stale value is fine — the
   // driver re-checks the ring after re-enabling.
   reg        tx_no_irq;
   reg        rx_no_irq;

   assign debug_rx_deliver_count = rx_deliver_count;
   assign debug_rx_nobuf_count   = rx_nobuf_count;

   wire rx_queue_configured = rx_queue_ready && rx_queue_num != 32'd0 &&
                              rx_queue_desc[63:32] == 32'd0 &&
                              rx_queue_driver[63:32] == 32'd0 &&
                              rx_queue_device[63:32] == 32'd0;
   wire [15:0] rx_avail_slot = rx_last_avail_idx & ring_mask;
   wire [15:0] rx_used_slot  = rx_used_idx & ring_mask;
   wire [15:0] rx_next_avail_idx = rx_last_avail_idx + 16'd1;
   // byte to write this RX cycle: 12-byte virtio_net_hdr_v1 then the frame.
   // The header is zero except num_buffers (bytes 10..11, LE) = 1, required
   // when VIRTIO_NET_F_MRG_RXBUF is not negotiated.
   wire [ 7:0] rx_wbyte = (rx_widx == 11'd10) ? 8'd1 :
                          (rx_widx <  11'd12) ? 8'd0 : rx_rd_data;
   // The engine's read is REGISTERED (a BRAM ring of slots since 2026-09-05), so the address
   // presented now is the byte consumed NEXT cycle: in S_RX_GATHER that is rx_widx+1, and in
   // any other state it is rx_widx itself (S_RX_WRITE sits between two gathers with rx_widx
   // already advanced, and the RAM holds its output while the address holds). Bytes below
   // 12 are the header, so the address saturates at 0 there and nothing is read.
   wire [10:0] rx_nxt = (state == S_RX_GATHER) ? rx_widx + 11'd1 : rx_widx;
   assign rx_rd_addr = (rx_nxt >= 11'd12) ? rx_nxt - 11'd12 : 11'd0;
   wire [63:0] rx_waddr = rx_buf_addr + {53'd0, rx_widx};
   // The RX write word (plan item 7, 2026-09-05): bytes are gathered one per clock from the
   // engine's async-read buffer into their lanes and go out as ONE AXI write per 8-byte word
   // (strobed at the buffer's head and tail), the previous word's write in flight meanwhile.
   // Before this every byte was its own AXI transaction: 1,518 round trips and 31,919 cycles
   // per 1500-byte frame in tb_virtio_net, which made RX the slow direction of NFS.
   reg  [63:0] rx_wbuf;  reg [7:0] rx_wstrb;  reg rx_wlast;
   // TX read-ahead (plan item 7): the next frame word is fetched while the current one is
   // byte-written into the engine, so the fill hides under the DDR round trip instead of
   // following it. One outstanding read (the master's limit); rd_end is the last word.
   reg  [63:0] nxt_word;  reg nxt_valid, rd_ahead, rd_err;  reg [63:0] rd_end;

   assign debug_tx_frame_count = tx_frame_count;
   assign debug_tx_last_len    = {21'd0, tx_last_len};
   assign debug_tx_desc_addr   = tx_dbg_desc_addr;
   assign debug_tx_desc_len    = tx_dbg_desc_len;

   reg        dma_cmd_valid;
   wire       dma_cmd_ready;
   reg        dma_cmd_write;
   reg [30:0] dma_cmd_addr;
   reg [63:0] dma_cmd_wdata;
   reg [ 7:0] dma_cmd_wstrb;
   wire       dma_rsp_valid;
   wire [63:0] dma_rsp_rdata;
   wire       dma_rsp_error;

   function [15:0] get16;
      input [63:0] word;
      input [2:0]  byte_offset;
      /* verilator lint_off UNUSEDSIGNAL */
      reg [63:0] shifted;
      /* verilator lint_on UNUSEDSIGNAL */
      begin
         shifted = word >> ({3'd0, byte_offset} * 6'd8);
         get16 = shifted[15:0];
      end
   endfunction

   function [7:0] write_strobe;
      input [2:0] byte_offset;
      input [3:0] byte_count;
      begin
         write_strobe = ((8'h01 << byte_count) - 8'h01) << byte_offset;
      end
   endfunction

   function [63:0] write_shift;
      input [63:0] data;
      input [2:0]  byte_offset;
      begin
         write_shift = data << ({3'd0, byte_offset} * 6'd8);
      end
   endfunction

   wire driver_ok = device_status[2];
   wire tx_queue_configured = tx_queue_ready && tx_queue_num != 32'd0 &&
                              tx_queue_desc[63:32] == 32'd0 &&
                              tx_queue_driver[63:32] == 32'd0 &&
                              tx_queue_device[63:32] == 32'd0;
   wire tx_notify = queue_notify_pulse && queue_notify_value == 32'd1;
   wire [15:0] next_avail_idx = last_avail_idx + 16'd1;

   /* Ring-slot indices wrap at the (power-of-two) queue size. virtio-mmio always
    * uses queue_num == QUEUE_NUM_MAX, so a compile-time mask is correct and keeps
    * the DMA address arithmetic cheap. (Was hardcoded mod-8.) */
   wire [15:0] ring_mask  = QUEUE_SIZE[15:0] - 16'd1;
   wire [15:0] avail_slot = last_avail_idx & ring_mask;
   wire [15:0] used_slot  = used_idx & ring_mask;

   /* Registered queue-base copies + a precomputed range check. The bases arrive
    * from virtio_mmio's registers over a long route, and a DMA address used to be
    * base + ring offset through a 64b carry chain AND a 33b magnitude compare in
    * the dma_cmd_addr cycle -- the chronic -0.003ns setup endpoint
    * (queue1_device_q -> dma_cmd_addr). Bases only change during queue setup
    * (before queue_ready; MMIO writes are many cycles apart through the CDC
    * bridge), so 1-cycle-stale copies are safe. The check matches the tasks'
    * addr[63:31] <= 1 clamp exactly, except it also requires the base below the
    * top 4 KiB of the 2 GiB window so no ring offset (< 4 KiB) can carry out of
    * bit 30 -- a ring up there couldn't fit anyway. Descriptor-sourced buffer
    * addresses (desc_addr/rx_waddr, locally registered from DMA data) keep the
    * inline clamp in the 64b tasks. */
   reg  [30:0] txq_drv_b, txq_desc_b, txq_dev_b, rxq_drv_b, rxq_desc_b, rxq_dev_b;
   reg         txq_drv_ok, txq_desc_ok, txq_dev_ok, rxq_drv_ok, rxq_desc_ok, rxq_dev_ok;
   localparam [30:0] BASE_MAX = 31'h7FFF_F000;   // 2 GiB - 4 KiB
   always @(posedge clock) begin
      txq_drv_b   <= tx_queue_driver[30:0];
      txq_desc_b  <= tx_queue_desc[30:0];
      txq_dev_b   <= tx_queue_device[30:0];
      rxq_drv_b   <= rx_queue_driver[30:0];
      rxq_desc_b  <= rx_queue_desc[30:0];
      rxq_dev_b   <= rx_queue_device[30:0];
      txq_drv_ok  <= (tx_queue_driver[63:32] == 32'd0) && (tx_queue_driver[30:0] <= BASE_MAX);
      txq_desc_ok <= (tx_queue_desc[63:32]   == 32'd0) && (tx_queue_desc[30:0]   <= BASE_MAX);
      txq_dev_ok  <= (tx_queue_device[63:32] == 32'd0) && (tx_queue_device[30:0] <= BASE_MAX);
      rxq_drv_ok  <= (rx_queue_driver[63:32] == 32'd0) && (rx_queue_driver[30:0] <= BASE_MAX);
      rxq_desc_ok <= (rx_queue_desc[63:32]   == 32'd0) && (rx_queue_desc[30:0]   <= BASE_MAX);
      rxq_dev_ok  <= (rx_queue_device[63:32] == 32'd0) && (rx_queue_device[30:0] <= BASE_MAX);
   end

   assign debug_status = {device_status, 14'd0, rx_frame_valid, notify_pending,
                          tx_queue_configured, driver_ok, state};
   assign debug_notify_count = notify_count;
   assign debug_read_avail_count = read_avail_count;
   assign debug_empty_avail_count = empty_avail_count;
   assign debug_read_ring_count = read_ring_count;
   assign debug_complete_count = complete_count;
   assign debug_irq_count = irq_count;
   assign debug_dma_error_count = dma_error_count;
   assign debug_indices = {last_avail_idx, avail_idx};
   assign debug_used_head = {used_idx, head_desc};
   assign debug_last_avail_word_lo = last_avail_word[31:0];
   assign debug_last_avail_word_hi = last_avail_word[63:32];
   assign debug_last_ring_word_lo = last_ring_word[31:0];
   assign debug_last_ring_word_hi = last_ring_word[63:32];

   task start_read64;
      input [63:0] addr;
      begin
         dma_cmd_valid <= 1'b1;
         dma_cmd_write <= 1'b0;
         dma_cmd_addr <= addr[63:31] <= 33'd1 ? addr[30:0] : 31'd0;
         dma_cmd_wdata <= 64'd0;
         dma_cmd_wstrb <= 8'd0;
      end
   endtask

   // pre-checked-base variants: the range clamp was resolved at base-register
   // time (see *_ok above), so dma_cmd_addr sees only a 31-bit add + a 2:1 mux
   // on a registered select -- no 64b carry chain, no magnitude compare.
   task start_read31;
      input        ok;
      input [30:0] a31;
      begin
         dma_cmd_valid <= 1'b1;
         dma_cmd_write <= 1'b0;
         dma_cmd_addr <= ok ? a31 : 31'd0;
         dma_cmd_wdata <= 64'd0;
         dma_cmd_wstrb <= 8'd0;
      end
   endtask

   task start_write31;
      input        ok;
      input [30:0] a31;
      input [63:0] data;
      input [7:0]  strobe;
      begin
         dma_cmd_valid <= 1'b1;
         dma_cmd_write <= 1'b1;
         dma_cmd_addr <= ok ? a31 : 31'd0;
         dma_cmd_wdata <= data;
         dma_cmd_wstrb <= strobe;
      end
   endtask

   task start_write;
      input [63:0] addr;
      input [63:0] data;
      input [7:0]  strobe;
      begin
         dma_cmd_valid <= 1'b1;
         dma_cmd_write <= 1'b1;
         dma_cmd_addr <= addr[63:31] <= 33'd1 ? addr[30:0] : 31'd0;
         dma_cmd_wdata <= data;
         dma_cmd_wstrb <= strobe;
      end
   endtask

   always @(posedge clock) if (!reset && state == S_RX_WRITE && dma_cmd_ready && rx_wstrb == 8'd0)
      $fatal(1, "virtio_net: an RX write word with no bytes (widx=%0d total=%0d)", rx_widx, rx_total);
   always @(posedge clock) begin
      dma_cmd_valid <= 1'b0;
      used_buffer_interrupt <= 1'b0;
      tx_wr_en <= 1'b0;
      if (dma_rsp_valid && rd_ahead) begin        // the TX read-ahead word lands
         nxt_word <= dma_rsp_rdata; nxt_valid <= 1'b1; rd_ahead <= 1'b0;
         if (dma_rsp_error) begin rd_err <= 1'b1; dma_error_count <= dma_error_count + 32'd1; end
      end
      tx_send  <= 1'b0;
      rx_frame_ack <= 1'b0;

      if (reset || device_status == 8'd0) begin
         state <= S_IDLE;
         tx_wr_addr <= 11'd0;
         tx_wr_data <= 8'd0;
         tx_send_len <= 11'd0;
         tx_busy_seen <= 1'b0;
         tx_frame_count <= 32'd0;
         tx_last_len <= 11'd0;
         tx_dbg_desc_addr <= 32'd0;
         tx_dbg_desc_len <= 32'd0;
         rx_last_avail_idx <= 16'd0;
         rx_avail_idx <= 16'd0;
         rx_used_idx <= 16'd0;
         rx_head_desc <= 16'd0;
         rx_widx <= 11'd0;
         rx_total <= 11'd0;
         rx_deliver_count <= 32'd0;
         rx_nobuf_count <= 32'd0;
         tx_no_irq <= 1'b0;
         rx_no_irq <= 1'b0;
         last_avail_idx <= 16'd0;
         avail_idx <= 16'd0;
         used_idx <= 16'd0;
         head_desc <= 16'd0;
         notify_pending <= 1'b0;
         empty_retry_count <= 8'd0;
         retry_delay <= 16'd0;
         notify_count <= 32'd0;
         read_avail_count <= 32'd0;
         empty_avail_count <= 32'd0;
         read_ring_count <= 32'd0;
         complete_count <= 32'd0;
         irq_count <= 32'd0;
         dma_error_count <= 32'd0;
         last_avail_word <= 64'd0;
         last_ring_word <= 64'd0;
      end else begin
         if (tx_notify) begin
            notify_pending <= 1'b1;
            notify_count <= notify_count + 32'd1;
         end

         case (state)
           S_IDLE: begin
              // Deliver a received frame first, then service TX notifies.
              if (rx_frame_valid && !rx_frame_ack &&
                  rx_queue_configured && driver_ok) begin
                 state <= S_RX_AVAIL;
              end else if (tx_queue_configured && driver_ok) begin
                 if (tx_notify || notify_pending) begin
                    notify_pending <= 1'b0;
                    empty_retry_count <= EMPTY_RETRY_COUNT;
                    state <= S_READ_AVAIL;
                 end
              end
           end

           S_READ_AVAIL: begin
              if (dma_cmd_ready) begin
                 start_read31(txq_drv_ok, txq_drv_b);
                 state <= S_WAIT_AVAIL;
              end
           end
           S_WAIT_AVAIL: begin
              if (dma_rsp_valid) begin
                 read_avail_count <= read_avail_count + 32'd1;
                 last_avail_word <= dma_rsp_rdata;
                 if (dma_rsp_error)
                    dma_error_count <= dma_error_count + 32'd1;
                 tx_no_irq <= |(get16(dma_rsp_rdata, tx_queue_driver[2:0] & 3'h7) & 16'h0001);
                 avail_idx <= get16(dma_rsp_rdata, (tx_queue_driver[2:0] + 3'd2) & 3'h7);
                 if (dma_rsp_error)
                    state <= S_IDLE;
                 else if (get16(dma_rsp_rdata, (tx_queue_driver[2:0] + 3'd2) & 3'h7) == last_avail_idx) begin
                    empty_avail_count <= empty_avail_count + 32'd1;
                    if (empty_retry_count != 8'd0) begin
                       empty_retry_count <= empty_retry_count - 8'd1;
                       retry_delay <= EMPTY_RETRY_DELAY;
                       state <= S_RETRY_WAIT;
                    end else begin
                       state <= S_IDLE;
                    end
                 end else begin
                    state <= S_READ_RING;
                 end
              end
           end

           S_RETRY_WAIT: begin
              if (retry_delay == 16'd0)
                 state <= S_READ_AVAIL;
              else
                 retry_delay <= retry_delay - 16'd1;
           end

           S_READ_RING: begin
              if (dma_cmd_ready) begin
                 start_read31(txq_drv_ok, txq_drv_b + 31'd4 + {14'd0, avail_slot, 1'b0});
                 state <= S_WAIT_RING;
              end
           end
           S_WAIT_RING: begin
              if (dma_rsp_valid) begin
                 read_ring_count <= read_ring_count + 32'd1;
                 last_ring_word <= dma_rsp_rdata;
                 if (dma_rsp_error)
                    dma_error_count <= dma_error_count + 32'd1;
                 head_desc <= get16(dma_rsp_rdata,
                                    (tx_queue_driver[2:0] + 3'd4 +
                                     {last_avail_idx[1:0], 1'b0}) & 3'h7);
                 // Fetch + transmit the frame, then complete the used ring.
                 state <= dma_rsp_error ? S_IDLE : S_DESC_A;
              end
           end

           // ---- TX frame fetch -------------------------------------------
           // Descriptor entry = {addr[63:0], len[31:0], flags[15:0],
           // next[15:0]} at tx_queue_desc + head_desc*16.  With VERSION_1 and
           // can_push, Linux puts the 12-byte virtio_net_hdr_v1 inline ahead
           // of the frame in one (>=8-byte-aligned) buffer, so the frame is
           // desc payload bytes [12 .. len).  We read word0 (hdr 0..7), then
           // stream from desc_addr+8: that word's high 4 bytes are frame[0..3]
           // (hdr 8..11 in the low 4), and later words are 8 frame bytes each.
           S_DESC_A: begin
              if (dma_cmd_ready) begin
                 start_read31(txq_desc_ok, txq_desc_b + {11'd0, head_desc, 4'd0});
                 state <= S_WAIT_DESC_A;
              end
           end
           S_WAIT_DESC_A: begin
              if (dma_rsp_valid) begin
                 desc_addr <= dma_rsp_rdata;
                 tx_dbg_desc_addr <= dma_rsp_rdata[31:0];
                 state <= dma_rsp_error ? S_WRITE_USED_ID : S_DESC_B;
              end
           end
           S_DESC_B: begin
              if (dma_cmd_ready) begin
                 start_read31(txq_desc_ok, txq_desc_b + {11'd0, head_desc, 4'd0} + 31'd8);
                 state <= S_WAIT_DESC_B;
              end
           end
           S_WAIT_DESC_B: begin
              if (dma_rsp_valid) begin : desc_b
                 // rdata = {next[63:48], flags[47:32], len[31:0]}
                 reg [31:0] dlen;
                 dlen = dma_rsp_rdata[31:0];
                 tx_dbg_desc_len <= dlen;
                 if (dma_rsp_error || dlen <= 32'd12 || dlen > 32'd1548) begin
                    // nothing sensible to send; still complete the used ring
                    state <= S_WRITE_USED_ID;
                 end else begin
                    // The frame starts at desc_addr+12 (after the v1 header),
                    // at any byte alignment.  The AXI master aligns reads to 8
                    // bytes (drops addr[2:0]), so read from the aligned word
                    // containing frame[0] and extract from its byte offset.
                    frame_total <= dlen[10:0] - 11'd12;
                    rd_end      <= (desc_addr + {53'd0, dlen[10:0]} - 64'd1) & ~64'd7;   // the word of the last frame byte
                    nxt_valid <= 1'b0; rd_ahead <= 1'b0; rd_err <= 1'b0;
                    byte_idx    <= 11'd0;
                    first_word  <= 1'b1;
                    start_off   <= desc_addr[2:0] + 3'd4;            // (desc+12)[2:0]
                    desc_addr   <= (desc_addr + 64'd12) & ~64'd7;    // aligned word
                    state       <= S_FRAME_RD;
                 end
              end
           end
           S_FRAME_RD: begin                          // the first word (later ones are read ahead)
              if (dma_cmd_ready) begin
                 start_read64(desc_addr);
                 desc_addr <= desc_addr + 64'd8;
                 state <= S_WAIT_FRAME;
              end
           end
           S_WAIT_FRAME: begin
              if (dma_rsp_valid) begin
                 cur_word  <= dma_rsp_rdata;
                 word_byte <= first_word ? start_off : 3'd0;
                 first_word <= 1'b0;
                 state <= dma_rsp_error ? S_SEND : S_FILL;
              end
           end
           S_FILL: begin
              // byte-write the engine buffer one byte/clock; meanwhile read the next word
              if (!rd_ahead && !nxt_valid && desc_addr <= rd_end && dma_cmd_ready) begin
                 start_read64(desc_addr);
                 desc_addr <= desc_addr + 64'd8;
                 rd_ahead  <= 1'b1;
              end
              tx_wr_en   <= 1'b1;
              tx_wr_addr <= byte_idx;
              tx_wr_data <= cur_word[{word_byte, 3'd0} +: 8];
              byte_idx   <= byte_idx + 11'd1;
              if (byte_idx == frame_total - 11'd1)
                 state <= S_SEND;
              else if (word_byte == 3'd7) begin
                 word_byte <= 3'd0;
                 if (nxt_valid) begin cur_word <= nxt_word; nxt_valid <= 1'b0; end   // landed already: no bubble
                 else state <= rd_err ? S_SEND : S_WAIT_NEXT;
              end else
                 word_byte <= word_byte + 3'd1;
           end
           S_WAIT_NEXT: begin                          // the read-ahead is still in flight
              if (nxt_valid) begin cur_word <= nxt_word; nxt_valid <= 1'b0; state <= S_FILL; end
              else if (rd_err) state <= S_SEND;
           end
           S_SEND: begin
              tx_send      <= 1'b1;
              tx_send_len  <= frame_total;
              tx_busy_seen <= 1'b0;
              tx_timeout   <= 20'hf_ffff;   // ~3 ms @ 333 MHz; >> any frame
              state        <= S_WAIT_SEND;
           end
           S_WAIT_SEND: begin
              // Complete on TX drain, OR on timeout so virtio never wedges if
              // the PHY link is down (gmii_rx_clk not running -> busy never
              // clears).  A dropped frame here is no worse than the old behavior.
              if (tx_busy) tx_busy_seen <= 1'b1;
              tx_timeout <= tx_timeout - 20'd1;
              if ((tx_busy_seen && !tx_busy) || tx_timeout == 20'd0) begin
                 tx_frame_count <= tx_frame_count + 32'd1;
                 tx_last_len    <= frame_total;
                 state <= S_WRITE_USED_ID;
              end
           end

           S_WRITE_USED_ID: begin
              if (dma_cmd_ready) begin
                 start_write31(txq_dev_ok, txq_dev_b + 31'd4 + {12'd0, used_slot, 3'd0},
                             write_shift({48'd0, head_desc}, (tx_queue_device[2:0] + 3'd4) & 3'h7),
                             write_strobe((tx_queue_device[2:0] + 3'd4) & 3'h7, 4'd4));
                 state <= S_WAIT_USED_ID;
              end
           end
           S_WAIT_USED_ID: begin
              if (dma_rsp_valid) begin
                 if (dma_rsp_error)
                    dma_error_count <= dma_error_count + 32'd1;
                 state <= S_WRITE_USED_LEN;
              end
           end

           S_WRITE_USED_LEN: begin
              if (dma_cmd_ready) begin
                 start_write31(txq_dev_ok, txq_dev_b + 31'd8 + {12'd0, used_slot, 3'd0},
                             write_shift(64'd0, tx_queue_device[2:0]),
                             write_strobe(tx_queue_device[2:0], 4'd4));
                 state <= S_WAIT_USED_LEN;
              end
           end
           S_WAIT_USED_LEN: begin
              if (dma_rsp_valid) begin
                 if (dma_rsp_error)
                    dma_error_count <= dma_error_count + 32'd1;
                 state <= S_WRITE_USED_IDX;
              end
           end

           S_WRITE_USED_IDX: begin
              if (dma_cmd_ready) begin
                 start_write31(txq_dev_ok, txq_dev_b,
                             write_shift({32'd0, used_idx + 16'd1, 16'd0}, tx_queue_device[2:0]),
                             write_strobe(tx_queue_device[2:0], 4'd4));
                 state <= S_WAIT_USED_IDX;
              end
           end
           S_WAIT_USED_IDX: begin
              if (dma_rsp_valid) begin
                 if (dma_rsp_error)
                    dma_error_count <= dma_error_count + 32'd1;
                 state <= S_TX_FLAGS;
              end
           end
           // THE INTERRUPT DECISION READS avail.flags AFTER used.idx HAS LANDED (virtio 1.x,
           // "Device Requirements: Notification Suppression"): the driver clears NO_INTERRUPT
           // when it re-arms, then re-checks the used ring; a flag sampled when this frame
           // started can be stale by then, and a suppressed interrupt then is a lost wakeup
           // the driver only recovers from by a timeout -- NFS "server not responding" every
           // few minutes on the board (2026-09-05), rarer when a frame took 30 k cycles.
           S_TX_FLAGS: begin
              if (dma_cmd_ready) begin
                 start_read31(txq_drv_ok, txq_drv_b);
                 state <= S_TX_FLAGS_WAIT;
              end
           end
           S_TX_FLAGS_WAIT: begin
              if (dma_rsp_valid) begin
                 if (dma_rsp_error) dma_error_count <= dma_error_count + 32'd1;
                 else tx_no_irq <= |(get16(dma_rsp_rdata, tx_queue_driver[2:0] & 3'h7) & 16'h0001);
                 state <= S_COMPLETE;
              end
           end

           S_COMPLETE: begin
              used_idx <= used_idx + 16'd1;
              last_avail_idx <= next_avail_idx;
              used_buffer_interrupt <= ~tx_no_irq;
              complete_count <= complete_count + 32'd1;
              if (!tx_no_irq) irq_count <= irq_count + 32'd1;
              if (next_avail_idx != avail_idx)
                 state <= S_READ_RING;
              else if (notify_pending) begin
                 notify_pending <= 1'b0;
                 state <= S_READ_AVAIL;
              end else begin
                 state <= S_IDLE;
              end
           end

           // ---- RX delivery (queue 0): take a buffer, DMA hdr+frame in -----
           S_RX_AVAIL: begin
              if (dma_cmd_ready) begin
                 start_read31(rxq_drv_ok, rxq_drv_b);          // avail flags+idx
                 state <= S_RX_WAIT_AVAIL;
              end
           end
           S_RX_WAIT_AVAIL: begin
              if (dma_rsp_valid) begin
                 if (dma_rsp_error) dma_error_count <= dma_error_count + 32'd1;
                 rx_no_irq    <= |(get16(dma_rsp_rdata, rx_queue_driver[2:0] & 3'h7) & 16'h0001);
                 rx_avail_idx <= get16(dma_rsp_rdata, (rx_queue_driver[2:0] + 3'd2) & 3'h7);
                 if (dma_rsp_error ||
                     get16(dma_rsp_rdata, (rx_queue_driver[2:0] + 3'd2) & 3'h7) == rx_last_avail_idx)
                    state <= S_RX_DROP;                  // no posted buffer
                 else
                    state <= S_RX_RING;
              end
           end
           S_RX_RING: begin
              if (dma_cmd_ready) begin
                 start_read31(rxq_drv_ok, rxq_drv_b + 31'd4 + {14'd0, rx_avail_slot, 1'b0});
                 state <= S_RX_WAIT_RING;
              end
           end
           S_RX_WAIT_RING: begin
              if (dma_rsp_valid) begin
                 if (dma_rsp_error) dma_error_count <= dma_error_count + 32'd1;
                 rx_head_desc <= get16(dma_rsp_rdata,
                                       (rx_queue_driver[2:0] + 3'd4 +
                                        {rx_last_avail_idx[1:0], 1'b0}) & 3'h7);
                 state <= dma_rsp_error ? S_RX_DROP : S_RX_DESC;
              end
           end
           S_RX_DESC: begin
              if (dma_cmd_ready) begin
                 start_read31(rxq_desc_ok, rxq_desc_b + {11'd0, rx_head_desc, 4'd0});
                 state <= S_RX_WAIT_DESC;
              end
           end
           S_RX_WAIT_DESC: begin
              if (dma_rsp_valid) begin
                 rx_buf_addr <= dma_rsp_rdata;           // RX buffer guest addr
                 rx_widx     <= 11'd0;
                 rx_total    <= rx_frame_len + 11'd12;    // 12B hdr + frame
                 rx_wbuf <= 64'd0; rx_wstrb <= 8'd0; rx_wlast <= 1'b0;
                 state <= dma_rsp_error ? S_RX_DROP : S_RX_GATHER;
              end
           end
           S_RX_GATHER: begin
              // One byte per clock into its lane of the write word: the 12-byte header
              // (byte 10 = num_buffers = 1) and then the frame. The word goes out when its
              // last lane fills or the frame ends; the previous word's write is still in
              // flight, and its response is counted here when it lands.
              rx_wbuf[{rx_waddr[2:0], 3'd0} +: 8] <= rx_wbyte;
              rx_wstrb[rx_waddr[2:0]] <= 1'b1;
              rx_widx    <= rx_widx + 11'd1;
              if (dma_rsp_valid && dma_rsp_error) dma_error_count <= dma_error_count + 32'd1;
              if (rx_waddr[2:0] == 3'd7 || rx_widx == rx_total - 11'd1) begin
                 rx_wlast <= (rx_widx == rx_total - 11'd1);
                 state <= S_RX_WRITE;
              end
           end
           S_RX_WRITE: begin
              // The gathered word, to the word that holds its last byte (the master drops
              // addr[2:0]; the strobes carry the alignment). Issued as soon as the master is
              // free, which means the previous write's response has already landed.
              if (dma_rsp_valid && dma_rsp_error) dma_error_count <= dma_error_count + 32'd1;
              if (dma_cmd_ready) begin
                 start_write(rx_buf_addr + {53'd0, rx_widx - 11'd1}, rx_wbuf, rx_wstrb);
                 rx_wbuf <= 64'd0; rx_wstrb <= 8'd0;
                 state <= rx_wlast ? S_RX_WRITE_WAIT : S_RX_GATHER;
              end
           end
           S_RX_WRITE_WAIT: begin                       // the last word's response
              if (dma_rsp_valid) begin
                 if (dma_rsp_error) dma_error_count <= dma_error_count + 32'd1;
                 state <= S_RX_USED_ID;
              end
           end
           S_RX_USED_ID: begin
              if (dma_cmd_ready) begin
                 start_write31(rxq_dev_ok, rxq_dev_b + 31'd4 + {12'd0, rx_used_slot, 3'd0},
                             write_shift({48'd0, rx_head_desc}, (rx_queue_device[2:0] + 3'd4) & 3'h7),
                             write_strobe((rx_queue_device[2:0] + 3'd4) & 3'h7, 4'd4));
                 state <= S_RX_WAIT_USED_ID;
              end
           end
           S_RX_WAIT_USED_ID: begin
              if (dma_rsp_valid) begin
                 if (dma_rsp_error) dma_error_count <= dma_error_count + 32'd1;
                 state <= S_RX_USED_LEN;
              end
           end
           S_RX_USED_LEN: begin
              if (dma_cmd_ready) begin
                 start_write31(rxq_dev_ok, rxq_dev_b + 31'd8 + {12'd0, rx_used_slot, 3'd0},
                             write_shift({53'd0, rx_total}, rx_queue_device[2:0]),
                             write_strobe(rx_queue_device[2:0], 4'd4));
                 state <= S_RX_WAIT_USED_LEN;
              end
           end
           S_RX_WAIT_USED_LEN: begin
              if (dma_rsp_valid) begin
                 if (dma_rsp_error) dma_error_count <= dma_error_count + 32'd1;
                 state <= S_RX_USED_IDX;
              end
           end
           S_RX_USED_IDX: begin
              if (dma_cmd_ready) begin
                 start_write31(rxq_dev_ok, rxq_dev_b,
                             write_shift({32'd0, rx_used_idx + 16'd1, 16'd0}, rx_queue_device[2:0]),
                             write_strobe(rx_queue_device[2:0], 4'd4));
                 state <= S_RX_WAIT_USED_IDX;
              end
           end
           S_RX_WAIT_USED_IDX: begin
              if (dma_rsp_valid) begin
                 if (dma_rsp_error) dma_error_count <= dma_error_count + 32'd1;
                 state <= S_RX_FLAGS;
              end
           end
           S_RX_FLAGS: begin                            // see S_TX_FLAGS
              if (dma_cmd_ready) begin
                 start_read31(rxq_drv_ok, rxq_drv_b);
                 state <= S_RX_FLAGS_WAIT;
              end
           end
           S_RX_FLAGS_WAIT: begin
              if (dma_rsp_valid) begin
                 if (dma_rsp_error) dma_error_count <= dma_error_count + 32'd1;
                 else rx_no_irq <= |(get16(dma_rsp_rdata, rx_queue_driver[2:0] & 3'h7) & 16'h0001);
                 state <= S_RX_COMPLETE;
              end
           end
           S_RX_COMPLETE: begin
              rx_used_idx       <= rx_used_idx + 16'd1;
              rx_last_avail_idx <= rx_next_avail_idx;
              used_buffer_interrupt <= ~rx_no_irq;
              if (!rx_no_irq) irq_count <= irq_count + 32'd1;
              rx_deliver_count  <= rx_deliver_count + 32'd1;
              rx_frame_ack      <= 1'b1;               // release engine buffer
              state <= S_IDLE;
           end
           S_RX_DROP: begin
              rx_nobuf_count <= rx_nobuf_count + 32'd1;
              rx_frame_ack   <= 1'b1;                  // drop: release engine
              state <= S_IDLE;
           end

           default: state <= S_IDLE;
         endcase
      end
   end

   axi_single_beat_master dma_master(
      .clock          (clock),
      .reset          (reset),
      .cmd_valid      (dma_cmd_valid),
      .cmd_ready      (dma_cmd_ready),
      .cmd_write      (dma_cmd_write),
      .cmd_addr       (dma_cmd_addr),
      .cmd_wdata      (dma_cmd_wdata),
      .cmd_wstrb      (dma_cmd_wstrb),
      .rsp_valid      (dma_rsp_valid),
      .rsp_rdata      (dma_rsp_rdata),
      .rsp_error      (dma_rsp_error),

      .m_axi_awid     (m_axi_awid),
      .m_axi_awaddr   (m_axi_awaddr),
      .m_axi_awlen    (m_axi_awlen),
      .m_axi_awsize   (m_axi_awsize),
      .m_axi_awburst  (m_axi_awburst),
      .m_axi_awlock   (m_axi_awlock),
      .m_axi_awcache  (m_axi_awcache),
      .m_axi_awprot   (m_axi_awprot),
      .m_axi_awqos    (m_axi_awqos),
      .m_axi_awvalid  (m_axi_awvalid),
      .m_axi_awready  (m_axi_awready),
      .m_axi_wdata    (m_axi_wdata),
      .m_axi_wstrb    (m_axi_wstrb),
      .m_axi_wlast    (m_axi_wlast),
      .m_axi_wvalid   (m_axi_wvalid),
      .m_axi_wready   (m_axi_wready),
      .m_axi_bid      (m_axi_bid),
      .m_axi_bresp    (m_axi_bresp),
      .m_axi_bvalid   (m_axi_bvalid),
      .m_axi_bready   (m_axi_bready),
      .m_axi_arid     (m_axi_arid),
      .m_axi_araddr   (m_axi_araddr),
      .m_axi_arlen    (m_axi_arlen),
      .m_axi_arsize   (m_axi_arsize),
      .m_axi_arburst  (m_axi_arburst),
      .m_axi_arlock   (m_axi_arlock),
      .m_axi_arcache  (m_axi_arcache),
      .m_axi_arprot   (m_axi_arprot),
      .m_axi_arqos    (m_axi_arqos),
      .m_axi_arvalid  (m_axi_arvalid),
      .m_axi_arready  (m_axi_arready),
      .m_axi_rid      (m_axi_rid),
      .m_axi_rdata    (m_axi_rdata),
      .m_axi_rresp    (m_axi_rresp),
      .m_axi_rlast    (m_axi_rlast),
      .m_axi_rvalid   (m_axi_rvalid),
      .m_axi_rready   (m_axi_rready)
   );

   wire unused_inputs = &{1'b0, tx_queue_desc[31:0]};
endmodule

`default_nettype wire
