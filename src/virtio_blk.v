`timescale 1ns / 1ps
`default_nettype none

// virtio-blk backend with a DDR-backed RAM disk.
//
// Walks the standard 3-descriptor blk request chain (header / data / status),
// and instead of synthesising data (the old virtio_blk_fake), it copies the
// data segment to/from a reserved DDR region:
//
//   sector S, word i  <->  BACKING_BASE + S*512 + i*8   (device AXI address)
//
//   T_IN  (read):  backing -> guest data buffer
//   T_OUT (write): guest data buffer -> backing
//
// The copy is word-at-a-time through the single-beat AXI master (read src,
// write dst), so it is mem-to-mem and slow but correct; a fast storage backend
// replaces the backing store later behind this same frontend.
//
// Assumptions (valid for Linux block I/O):
//   - the data descriptor is one segment (hdr/data/status chain, no extra NEXT)
//   - data buffers are >=8-byte aligned and a whole number of 512-byte sectors,
//     so every copied word is a full, 8-byte-aligned word (wstrb = 0xff).
module virtio_blk #(
    parameter [31:0] QUEUE_SIZE       = 32'd8,        /* virtqueue depth, power of two */
    parameter [31:0] CAPACITY_SECTORS = 32'd131072,   /* 512-byte sectors (64 MB) */
    parameter [30:0] BACKING_BASE     = 31'h7800_0000 /* device AXI addr of sector 0 */
) (
    input  wire        clock,
    input  wire        reset,

    input  wire        queue_notify_pulse,
    input  wire [31:0] queue_notify_value,
    input  wire [31:0] queue_num,
    input  wire        queue_ready,
    input  wire [63:0] queue_desc,
    input  wire [63:0] queue_driver,
    input  wire [63:0] queue_device,
    input  wire [ 7:0] device_status,
    output reg         used_buffer_interrupt,

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
   localparam [5:0] S_IDLE             = 6'd0;
   localparam [5:0] S_READ_AVAIL       = 6'd1;
   localparam [5:0] S_WAIT_AVAIL       = 6'd2;
   localparam [5:0] S_READ_RING        = 6'd3;
   localparam [5:0] S_WAIT_RING        = 6'd4;
   localparam [5:0] S_READ_DESC0_ADDR  = 6'd5;
   localparam [5:0] S_WAIT_DESC0_ADDR  = 6'd6;
   localparam [5:0] S_READ_DESC0_INFO  = 6'd7;
   localparam [5:0] S_WAIT_DESC0_INFO  = 6'd8;
   localparam [5:0] S_READ_DESC1_ADDR  = 6'd9;
   localparam [5:0] S_WAIT_DESC1_ADDR  = 6'd10;
   localparam [5:0] S_READ_DESC1_INFO  = 6'd11;
   localparam [5:0] S_WAIT_DESC1_INFO  = 6'd12;
   localparam [5:0] S_READ_DESC2_ADDR  = 6'd13;
   localparam [5:0] S_WAIT_DESC2_ADDR  = 6'd14;
   localparam [5:0] S_READ_DESC2_INFO  = 6'd15;
   localparam [5:0] S_WAIT_DESC2_INFO  = 6'd16;
   localparam [5:0] S_READ_HDR0        = 6'd17;
   localparam [5:0] S_WAIT_HDR0        = 6'd18;
   localparam [5:0] S_READ_HDR1        = 6'd19;
   localparam [5:0] S_WAIT_HDR1        = 6'd20;
   localparam [5:0] S_COPY_READ        = 6'd21;
   localparam [5:0] S_WAIT_COPY_READ   = 6'd22;
   localparam [5:0] S_COPY_WRITE       = 6'd23;
   localparam [5:0] S_WAIT_COPY_WRITE  = 6'd24;
   localparam [5:0] S_WRITE_STATUS     = 6'd25;
   localparam [5:0] S_WAIT_STATUS      = 6'd26;
   localparam [5:0] S_WRITE_USED_ID    = 6'd27;
   localparam [5:0] S_WAIT_USED_ID     = 6'd28;
   localparam [5:0] S_WRITE_USED_LEN   = 6'd29;
   localparam [5:0] S_WAIT_USED_LEN    = 6'd30;
   localparam [5:0] S_WRITE_USED_IDX   = 6'd31;
   localparam [5:0] S_WAIT_USED_IDX    = 6'd32;
   localparam [5:0] S_COMPLETE         = 6'd33;

   localparam [15:0] VRING_DESC_F_NEXT  = 16'h0001;
   localparam [15:0] VRING_DESC_F_WRITE = 16'h0002;
   localparam [31:0] VIRTIO_BLK_T_IN    = 32'd0;
   localparam [31:0] VIRTIO_BLK_T_OUT   = 32'd1;
   localparam [ 7:0] VIRTIO_BLK_S_OK    = 8'd0;
   localparam [ 7:0] VIRTIO_BLK_S_IOERR = 8'd1;
   localparam [ 7:0] VIRTIO_BLK_S_UNSUPP = 8'd2;

   wire [15:0] ring_mask  = QUEUE_SIZE[15:0] - 16'd1;

   reg [5:0]  state;
   reg [15:0] last_avail_idx;
   reg [15:0] avail_idx;       /* last published avail.idx (for draining a batch) */
   reg        notify_pending;  /* a notify seen while busy; re-poll on completion */
   reg [15:0] used_idx;
   reg [15:0] head_desc;
   reg [15:0] desc1_index;
   reg [15:0] desc2_index;
   reg [63:0] desc0_addr;
   reg [31:0] desc0_len;
   reg [15:0] desc0_flags;
   reg [63:0] desc1_addr;
   reg [31:0] desc1_len;
   reg [15:0] desc1_flags;
   reg [63:0] desc2_addr;
   reg [31:0] desc2_len;
   reg [15:0] desc2_flags;
   reg [31:0] req_type;
   reg [63:0] req_sector;
   reg        req_is_read;
   reg [63:0] copy_data;
   reg [31:0] data_words_left;
   reg [31:0] data_word_index;
   reg [ 7:0] status_byte;
   reg [31:0] used_len;

   wire [15:0] avail_slot = last_avail_idx & ring_mask;
   wire [15:0] used_slot  = used_idx & ring_mask;
   wire [15:0] next_avail_idx = last_avail_idx + 16'd1;
   wire        blk_notify = queue_notify_pulse && queue_notify_value == 32'd0;

   // Data segment word addresses for the current word index.
   wire [63:0] word_off     = {29'd0, data_word_index, 3'd0};
   wire [63:0] backing_addr = {33'd0, BACKING_BASE} + (req_sector << 9) + word_off;
   wire [63:0] guest_addr   = desc1_addr + word_off;
   wire [63:0] copy_src     = req_is_read ? backing_addr : guest_addr;
   wire [63:0] copy_dst     = req_is_read ? guest_addr   : backing_addr;

   // Request stays within the backing store (sector*512 + len <= capacity).
   wire [63:0] req_byte_end = (req_sector << 9) + {32'd0, desc1_len};
   wire        bounds_ok    = req_sector[63:32] == 32'd0 &&
                              req_byte_end <= {23'd0, CAPACITY_SECTORS, 9'd0};

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
      input [2:0] byte_offset;
      /* verilator lint_off UNUSEDSIGNAL */
      reg [63:0] shifted;
      /* verilator lint_on UNUSEDSIGNAL */
      begin
         shifted = word >> ({3'd0, byte_offset} * 6'd8);
         get16 = shifted[15:0];
      end
   endfunction

   function [31:0] get32;
      input [63:0] word;
      input [2:0] byte_offset;
      /* verilator lint_off UNUSEDSIGNAL */
      reg [63:0] shifted;
      /* verilator lint_on UNUSEDSIGNAL */
      begin
         shifted = word >> ({3'd0, byte_offset} * 6'd8);
         get32 = shifted[31:0];
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
      input [2:0] byte_offset;
      begin
         write_shift = data << ({3'd0, byte_offset} * 6'd8);
      end
   endfunction

   wire driver_ok = device_status[2];
   wire queue_configured = queue_ready && queue_num != 32'd0 &&
                           queue_desc[63:32] == 32'd0 &&
                           queue_driver[63:32] == 32'd0 &&
                           queue_device[63:32] == 32'd0;

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

   always @(posedge clock) begin
      dma_cmd_valid <= 1'b0;
      used_buffer_interrupt <= 1'b0;

      if (reset || device_status == 8'd0) begin
         state <= S_IDLE;
         last_avail_idx <= 16'd0;
         avail_idx <= 16'd0;
         notify_pending <= 1'b0;
         used_idx <= 16'd0;
         head_desc <= 16'd0;
         desc1_index <= 16'd0;
         desc2_index <= 16'd0;
         desc0_addr <= 64'd0;
         desc0_len <= 32'd0;
         desc0_flags <= 16'd0;
         desc1_addr <= 64'd0;
         desc1_len <= 32'd0;
         desc1_flags <= 16'd0;
         desc2_addr <= 64'd0;
         desc2_len <= 32'd0;
         desc2_flags <= 16'd0;
         req_type <= 32'd0;
         req_sector <= 64'd0;
         req_is_read <= 1'b0;
         copy_data <= 64'd0;
         data_words_left <= 32'd0;
         data_word_index <= 32'd0;
         status_byte <= VIRTIO_BLK_S_IOERR;
         used_len <= 32'd0;
      end else begin
         if (blk_notify)
            notify_pending <= 1'b1;

         case (state)
           S_IDLE: begin
              if (queue_configured && driver_ok && (blk_notify || notify_pending)) begin
                 notify_pending <= 1'b0;
                 state <= S_READ_AVAIL;
              end
           end

           S_READ_AVAIL: begin
              if (dma_cmd_ready) begin
                 start_read64(queue_driver);
                 state <= S_WAIT_AVAIL;
              end
           end
           S_WAIT_AVAIL: begin
              if (dma_rsp_valid) begin
                 avail_idx <= get16(dma_rsp_rdata, (queue_driver[2:0] + 3'd2) & 3'h7);
                 if (dma_rsp_error ||
                     get16(dma_rsp_rdata, (queue_driver[2:0] + 3'd2) & 3'h7) == last_avail_idx)
                    state <= S_IDLE;
                 else
                    state <= S_READ_RING;
              end
           end

           S_READ_RING: begin
              if (dma_cmd_ready) begin
                 start_read64(queue_driver + 64'd4 + {47'd0, avail_slot, 1'b0});
                 state <= S_WAIT_RING;
              end
           end
           S_WAIT_RING: begin
              if (dma_rsp_valid) begin
                 head_desc <= get16(dma_rsp_rdata,
                                    (queue_driver[2:0] + 3'd4 +
                                     {last_avail_idx[1:0], 1'b0}) & 3'h7);
                 if (dma_rsp_error)
                    state <= S_IDLE;
                 else
                    state <= S_READ_DESC0_ADDR;
              end
           end

           S_READ_DESC0_ADDR: begin
              if (dma_cmd_ready) begin
                 start_read64(queue_desc + {44'd0, head_desc, 4'd0});
                 state <= S_WAIT_DESC0_ADDR;
              end
           end
           S_WAIT_DESC0_ADDR: begin
              if (dma_rsp_valid) begin
                 desc0_addr <= dma_rsp_rdata;
                 state <= dma_rsp_error ? S_IDLE : S_READ_DESC0_INFO;
              end
           end
           S_READ_DESC0_INFO: begin
              if (dma_cmd_ready) begin
                 start_read64(queue_desc + {44'd0, head_desc, 4'd8});
                 state <= S_WAIT_DESC0_INFO;
              end
           end
           S_WAIT_DESC0_INFO: begin
              if (dma_rsp_valid) begin
                 desc0_len <= dma_rsp_rdata[31:0];
                 desc0_flags <= dma_rsp_rdata[47:32];
                 desc1_index <= dma_rsp_rdata[63:48];
                 if (dma_rsp_error || (dma_rsp_rdata[47:32] & VRING_DESC_F_NEXT) == 16'd0)
                    state <= S_IDLE;
                 else
                    state <= S_READ_DESC1_ADDR;
              end
           end

           S_READ_DESC1_ADDR: begin
              if (dma_cmd_ready) begin
                 start_read64(queue_desc + {44'd0, desc1_index, 4'd0});
                 state <= S_WAIT_DESC1_ADDR;
              end
           end
           S_WAIT_DESC1_ADDR: begin
              if (dma_rsp_valid) begin
                 desc1_addr <= dma_rsp_rdata;
                 state <= dma_rsp_error ? S_IDLE : S_READ_DESC1_INFO;
              end
           end
           S_READ_DESC1_INFO: begin
              if (dma_cmd_ready) begin
                 start_read64(queue_desc + {44'd0, desc1_index, 4'd8});
                 state <= S_WAIT_DESC1_INFO;
              end
           end
           S_WAIT_DESC1_INFO: begin
              if (dma_rsp_valid) begin
                 desc1_len <= dma_rsp_rdata[31:0];
                 desc1_flags <= dma_rsp_rdata[47:32];
                 desc2_index <= dma_rsp_rdata[63:48];
                 if (dma_rsp_error || (dma_rsp_rdata[47:32] & VRING_DESC_F_NEXT) == 16'd0)
                    state <= S_IDLE;
                 else
                    state <= S_READ_DESC2_ADDR;
              end
           end

           S_READ_DESC2_ADDR: begin
              if (dma_cmd_ready) begin
                 start_read64(queue_desc + {44'd0, desc2_index, 4'd0});
                 state <= S_WAIT_DESC2_ADDR;
              end
           end
           S_WAIT_DESC2_ADDR: begin
              if (dma_rsp_valid) begin
                 desc2_addr <= dma_rsp_rdata;
                 state <= dma_rsp_error ? S_IDLE : S_READ_DESC2_INFO;
              end
           end
           S_READ_DESC2_INFO: begin
              if (dma_cmd_ready) begin
                 start_read64(queue_desc + {44'd0, desc2_index, 4'd8});
                 state <= S_WAIT_DESC2_INFO;
              end
           end
           S_WAIT_DESC2_INFO: begin
              if (dma_rsp_valid) begin
                 desc2_len <= dma_rsp_rdata[31:0];
                 desc2_flags <= dma_rsp_rdata[47:32];
                 if (dma_rsp_error)
                    state <= S_IDLE;
                 else
                    state <= S_READ_HDR0;
              end
           end

           S_READ_HDR0: begin
              if (dma_cmd_ready) begin
                 start_read64(desc0_addr);
                 state <= S_WAIT_HDR0;
              end
           end
           S_WAIT_HDR0: begin
              if (dma_rsp_valid) begin
                 req_type <= get32(dma_rsp_rdata, desc0_addr[2:0]);
                 state <= dma_rsp_error ? S_IDLE : S_READ_HDR1;
              end
           end
           S_READ_HDR1: begin
              if (dma_cmd_ready) begin
                 start_read64(desc0_addr + 64'd8);
                 state <= S_WAIT_HDR1;
              end
           end
           S_WAIT_HDR1: begin
              if (dma_rsp_valid) begin
                 req_sector <= dma_rsp_rdata >> ({3'd0, desc0_addr[2:0]} * 6'd8);
                 data_word_index <= 32'd0;
                 data_words_left <= {3'd0, desc1_len[31:3]} + {31'd0, |desc1_len[2:0]};
                 status_byte <= VIRTIO_BLK_S_OK;
                 used_len <= 32'd1;
                 if (dma_rsp_error) begin
                    state <= S_IDLE;
                 end else if (!bounds_ok) begin
                    status_byte <= VIRTIO_BLK_S_IOERR;
                    state <= S_WRITE_STATUS;
                 end else if (req_type == VIRTIO_BLK_T_IN &&
                              (desc1_flags & VRING_DESC_F_WRITE) != 16'd0 &&
                              (desc2_flags & VRING_DESC_F_WRITE) != 16'd0 &&
                              desc2_len != 32'd0) begin
                    req_is_read <= 1'b1;
                    used_len <= desc1_len + 32'd1;
                    state <= desc1_len == 32'd0 ? S_WRITE_STATUS : S_COPY_READ;
                 end else if (req_type == VIRTIO_BLK_T_OUT &&
                              (desc1_flags & VRING_DESC_F_WRITE) == 16'd0 &&
                              (desc2_flags & VRING_DESC_F_WRITE) != 16'd0 &&
                              desc2_len != 32'd0) begin
                    req_is_read <= 1'b0;
                    state <= desc1_len == 32'd0 ? S_WRITE_STATUS : S_COPY_READ;
                 end else begin
                    status_byte <= VIRTIO_BLK_S_UNSUPP;
                    state <= S_WRITE_STATUS;
                 end
              end
           end

           // Mem-to-mem data copy: read one source word, write it to dst.
           S_COPY_READ: begin
              if (dma_cmd_ready) begin
                 start_read64(copy_src);
                 state <= S_WAIT_COPY_READ;
              end
           end
           S_WAIT_COPY_READ: begin
              if (dma_rsp_valid) begin
                 copy_data <= dma_rsp_rdata;
                 if (dma_rsp_error) begin
                    status_byte <= VIRTIO_BLK_S_IOERR;
                    state <= S_WRITE_STATUS;
                 end else begin
                    state <= S_COPY_WRITE;
                 end
              end
           end
           S_COPY_WRITE: begin
              if (dma_cmd_ready) begin
                 start_write(copy_dst, copy_data, 8'hff);
                 state <= S_WAIT_COPY_WRITE;
              end
           end
           S_WAIT_COPY_WRITE: begin
              if (dma_rsp_valid) begin
                 if (dma_rsp_error) begin
                    status_byte <= VIRTIO_BLK_S_IOERR;
                    state <= S_WRITE_STATUS;
                 end else if (data_words_left <= 32'd1) begin
                    state <= S_WRITE_STATUS;
                 end else begin
                    data_words_left <= data_words_left - 32'd1;
                    data_word_index <= data_word_index + 32'd1;
                    state <= S_COPY_READ;
                 end
              end
           end

           S_WRITE_STATUS: begin
              if (dma_cmd_ready) begin
                 start_write(desc2_addr,
                             write_shift({56'd0, status_byte}, desc2_addr[2:0]),
                             write_strobe(desc2_addr[2:0], 4'd1));
                 state <= S_WAIT_STATUS;
              end
           end
           S_WAIT_STATUS: begin
              if (dma_rsp_valid)
                 state <= S_WRITE_USED_ID;
           end

           S_WRITE_USED_ID: begin
              if (dma_cmd_ready) begin
                 start_write(queue_device + 64'd4 + {45'd0, used_slot, 3'd0},
                             write_shift({48'd0, head_desc}, (queue_device[2:0] + 3'd4) & 3'h7),
                             write_strobe((queue_device[2:0] + 3'd4) & 3'h7, 4'd4));
                 state <= S_WAIT_USED_ID;
              end
           end
           S_WAIT_USED_ID: begin
              if (dma_rsp_valid)
                 state <= S_WRITE_USED_LEN;
           end
           S_WRITE_USED_LEN: begin
              if (dma_cmd_ready) begin
                 start_write(queue_device + 64'd8 + {45'd0, used_slot, 3'd0},
                             write_shift({32'd0, used_len}, queue_device[2:0]),
                             write_strobe(queue_device[2:0], 4'd4));
                 state <= S_WAIT_USED_LEN;
              end
           end
           S_WAIT_USED_LEN: begin
              if (dma_rsp_valid)
                 state <= S_WRITE_USED_IDX;
           end
           S_WRITE_USED_IDX: begin
              if (dma_cmd_ready) begin
                 start_write(queue_device,
                             write_shift({32'd0, used_idx + 16'd1, 16'd0}, queue_device[2:0]),
                             write_strobe(queue_device[2:0], 4'd4));
                 state <= S_WAIT_USED_IDX;
              end
           end
           S_WAIT_USED_IDX: begin
              if (dma_rsp_valid)
                 state <= S_COMPLETE;
           end

           S_COMPLETE: begin
              used_idx <= used_idx + 16'd1;
              last_avail_idx <= next_avail_idx;
              used_buffer_interrupt <= 1'b1;
              // Drain the rest of this batch, then a notify that arrived while
              // busy, before going idle — so no notify is ever missed.
              if (next_avail_idx != avail_idx)
                 state <= S_READ_RING;
              else if (notify_pending) begin
                 notify_pending <= 1'b0;
                 state <= S_READ_AVAIL;
              end else
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

   wire unused_inputs = &{1'b0, desc0_len, desc0_flags};
endmodule

`default_nettype wire
