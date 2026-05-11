`timescale 1ns / 1ps
`default_nettype none

module virtio_net_tx_drop(
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

   localparam [ 7:0] EMPTY_RETRY_COUNT = 8'hff;
   localparam [15:0] EMPTY_RETRY_DELAY = 16'hffff;

   reg [4:0]  state;
   reg [15:0] last_avail_idx;
   reg [15:0] avail_idx;
   reg [15:0] used_idx;
   reg [15:0] head_desc;
   reg        notify_pending;
   reg [ 7:0] empty_retry_count;
   reg [15:0] retry_delay;

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
         used_idx <= 16'd0;
         head_desc <= 16'd0;
         notify_pending <= 1'b0;
         empty_retry_count <= 8'd0;
         retry_delay <= 16'd0;
      end else begin
         if (tx_notify)
            notify_pending <= 1'b1;

         case (state)
           S_IDLE: begin
              if ((tx_notify || notify_pending) && tx_queue_configured && driver_ok) begin
                 notify_pending <= 1'b0;
                 empty_retry_count <= EMPTY_RETRY_COUNT;
                 state <= S_READ_AVAIL;
              end
           end

           S_READ_AVAIL: begin
              if (dma_cmd_ready) begin
                 start_read64(tx_queue_driver);
                 state <= S_WAIT_AVAIL;
              end
           end
           S_WAIT_AVAIL: begin
              if (dma_rsp_valid) begin
                 avail_idx <= get16(dma_rsp_rdata, (tx_queue_driver[2:0] + 3'd2) & 3'h7);
                 if (dma_rsp_error)
                    state <= S_IDLE;
                 else if (get16(dma_rsp_rdata, (tx_queue_driver[2:0] + 3'd2) & 3'h7) == last_avail_idx) begin
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
                 start_read64(tx_queue_driver + 64'd4 + {60'd0, last_avail_idx[2:0], 1'b0});
                 state <= S_WAIT_RING;
              end
           end
           S_WAIT_RING: begin
              if (dma_rsp_valid) begin
                 head_desc <= get16(dma_rsp_rdata,
                                    (tx_queue_driver[2:0] + 3'd4 +
                                     {last_avail_idx[1:0], 1'b0}) & 3'h7);
                 state <= dma_rsp_error ? S_IDLE : S_WRITE_USED_ID;
              end
           end

           S_WRITE_USED_ID: begin
              if (dma_cmd_ready) begin
                 start_write(tx_queue_device + 64'd4 + {58'd0, used_idx[2:0], 3'd0},
                             write_shift({48'd0, head_desc}, (tx_queue_device[2:0] + 3'd4) & 3'h7),
                             write_strobe((tx_queue_device[2:0] + 3'd4) & 3'h7, 4'd4));
                 state <= S_WAIT_USED_ID;
              end
           end
           S_WAIT_USED_ID: begin
              if (dma_rsp_valid)
                 state <= S_WRITE_USED_LEN;
           end

           S_WRITE_USED_LEN: begin
              if (dma_cmd_ready) begin
                 start_write(tx_queue_device + 64'd8 + {58'd0, used_idx[2:0], 3'd0},
                             write_shift(64'd0, tx_queue_device[2:0]),
                             write_strobe(tx_queue_device[2:0], 4'd4));
                 state <= S_WAIT_USED_LEN;
              end
           end
           S_WAIT_USED_LEN: begin
              if (dma_rsp_valid)
                 state <= S_WRITE_USED_IDX;
           end

           S_WRITE_USED_IDX: begin
              if (dma_cmd_ready) begin
                 start_write(tx_queue_device,
                             write_shift({32'd0, used_idx + 16'd1, 16'd0}, tx_queue_device[2:0]),
                             write_strobe(tx_queue_device[2:0], 4'd4));
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
              if (next_avail_idx != avail_idx)
                 state <= S_READ_RING;
              else if (notify_pending) begin
                 notify_pending <= 1'b0;
                 state <= S_READ_AVAIL;
              end else begin
                 state <= S_IDLE;
              end
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
