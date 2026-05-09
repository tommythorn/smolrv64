`timescale 1ns / 1ps
`default_nettype none

module virtio_blk_fake #(
    parameter [31:0] CAPACITY_SECTORS = 32'd2048
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
   localparam [5:0] S_WRITE_DATA       = 6'd21;
   localparam [5:0] S_WAIT_DATA        = 6'd22;
   localparam [5:0] S_WRITE_STATUS     = 6'd23;
   localparam [5:0] S_WAIT_STATUS      = 6'd24;
   localparam [5:0] S_WRITE_USED_ID    = 6'd25;
   localparam [5:0] S_WAIT_USED_ID     = 6'd26;
   localparam [5:0] S_WRITE_USED_LEN   = 6'd27;
   localparam [5:0] S_WAIT_USED_LEN    = 6'd28;
   localparam [5:0] S_WRITE_USED_IDX   = 6'd29;
   localparam [5:0] S_WAIT_USED_IDX    = 6'd30;
   localparam [5:0] S_COMPLETE         = 6'd31;

   localparam [15:0] VRING_DESC_F_NEXT  = 16'h0001;
   localparam [15:0] VRING_DESC_F_WRITE = 16'h0002;
   localparam [31:0] VIRTIO_BLK_T_IN    = 32'd0;
   localparam [31:0] VIRTIO_BLK_T_OUT   = 32'd1;
   localparam [ 7:0] VIRTIO_BLK_S_OK    = 8'd0;
   localparam [ 7:0] VIRTIO_BLK_S_IOERR = 8'd1;
   localparam [ 7:0] VIRTIO_BLK_S_UNSUPP = 8'd2;

   reg [5:0]  state;
   reg [15:0] last_avail_idx;
   reg [15:0] used_idx;
   reg [15:0] head_desc;
   reg [15:0] desc1_index;
   reg [15:0] desc2_index;
   reg [ 2:0] ring_slot;
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
   reg [31:0] data_words_left;
   reg [31:0] data_word_index;
   reg [ 7:0] status_byte;
   reg [31:0] used_len;

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

   function [63:0] fake_data_word;
      input [63:0] sector;
      input [31:0] word_index;
      begin
         if (sector == 64'd0 && word_index == 32'd0)
            fake_data_word = 64'h5452_4956_4c4f_4d53; /* "SMOLVIRT" */
         else
            fake_data_word = {16'h5642, sector[15:0], word_index[31:0]};
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
         used_idx <= 16'd0;
         head_desc <= 16'd0;
         desc1_index <= 16'd0;
         desc2_index <= 16'd0;
         ring_slot <= 3'd0;
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
         data_words_left <= 32'd0;
         data_word_index <= 32'd0;
         status_byte <= VIRTIO_BLK_S_IOERR;
         used_len <= 32'd0;
      end else begin
         case (state)
           S_IDLE: begin
              if (queue_notify_pulse && queue_notify_value == 32'd0 &&
                  queue_configured && driver_ok) begin
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
                 if (dma_rsp_error || dma_rsp_rdata[31:16] == last_avail_idx)
                    state <= S_IDLE;
                 else
                    state <= S_READ_RING;
              end
           end

           S_READ_RING: begin
              if (dma_cmd_ready) begin
                 ring_slot <= last_avail_idx[2:0];
                 start_read64(queue_driver + 64'd4 + {60'd0, last_avail_idx[2:0], 1'b0});
                 state <= S_WAIT_RING;
              end
           end
           S_WAIT_RING: begin
              if (dma_rsp_valid) begin
                 head_desc <= get16(dma_rsp_rdata, (queue_driver[2:0] + 3'd4 + {last_avail_idx[1:0], 1'b0}) & 3'h7);
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
                 end else if (req_type == VIRTIO_BLK_T_IN &&
                              (desc1_flags & VRING_DESC_F_WRITE) != 16'd0 &&
                              (desc2_flags & VRING_DESC_F_WRITE) != 16'd0 &&
                              desc2_len != 32'd0) begin
                    used_len <= desc1_len + 32'd1;
                    state <= desc1_len == 32'd0 ? S_WRITE_STATUS : S_WRITE_DATA;
                 end else if (req_type == VIRTIO_BLK_T_OUT &&
                              (desc2_flags & VRING_DESC_F_WRITE) != 16'd0 &&
                              desc2_len != 32'd0) begin
                    state <= S_WRITE_STATUS;
                 end else begin
                    status_byte <= VIRTIO_BLK_S_UNSUPP;
                    state <= S_WRITE_STATUS;
                 end
              end
           end

           S_WRITE_DATA: begin
              if (dma_cmd_ready) begin
                 start_write(desc1_addr + {29'd0, data_word_index, 3'd0},
                             fake_data_word(req_sector, data_word_index),
                             data_words_left == 32'd1 && desc1_len[2:0] != 3'd0
                             ? write_strobe(desc1_addr[2:0], {1'b0, desc1_len[2:0]})
                             : 8'hff);
                 state <= S_WAIT_DATA;
              end
           end
           S_WAIT_DATA: begin
              if (dma_rsp_valid) begin
                 if (dma_rsp_error) begin
                    status_byte <= VIRTIO_BLK_S_IOERR;
                    state <= S_WRITE_STATUS;
                 end else if (data_words_left <= 32'd1) begin
                    state <= S_WRITE_STATUS;
                 end else begin
                    data_words_left <= data_words_left - 32'd1;
                    data_word_index <= data_word_index + 32'd1;
                    state <= S_WRITE_DATA;
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
                 start_write(queue_device + 64'd4 + {58'd0, used_idx[2:0], 3'd0},
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
                 start_write(queue_device + 64'd8 + {58'd0, used_idx[2:0], 3'd0},
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
              last_avail_idx <= last_avail_idx + 16'd1;
              used_buffer_interrupt <= 1'b1;
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

   wire unused_inputs = &{1'b0, CAPACITY_SECTORS, desc0_len, desc0_flags, ring_slot};
endmodule

`default_nettype wire
