module smolrv64_mem_engine(
   input  wire        core_clock,
   input  wire        mem_clock,
   input  wire        reset,
   output wire        idle,

   input  wire        fill_req_valid,
   output wire        fill_req_ready,
   input  wire [24:0] fill_req_line_addr,
   output wire        fill_rsp_valid,
   input  wire        fill_rsp_ready,
   output wire [511:0] fill_rsp_data,

   input  wire        wb_req_valid,
   output wire        wb_req_ready,
   input  wire [24:0] wb_req_line_addr,
   input  wire [511:0] wb_req_line_data,
   output wire        wb_rsp_valid,
   input  wire        wb_rsp_ready,

   input  wire        read_req_valid,
   output wire        read_req_ready,
   input  wire [27:0] read_req_addr,
   output wire        read_rsp_valid,
   input  wire        read_rsp_ready,
   output wire [63:0] read_rsp_data,

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
   wire        fill_cmd_valid;
   reg         fill_cmd_ready = 0;
   wire [24:0] fill_cmd_line_addr;
   reg         fill_rsp_wr_valid = 0;
   wire        fill_rsp_wr_ready;
   reg  [511:0] fill_rsp_wr_data = 0;

   wire        wb_cmd_valid;
   reg         wb_cmd_ready = 0;
   wire [536:0] wb_cmd_data;
   reg         wb_rsp_wr_valid = 0;
   wire        wb_rsp_wr_ready;

   wire        read_cmd_valid;
   reg         read_cmd_ready = 0;
   wire [27:0] read_cmd_addr;
   reg         read_rsp_wr_valid = 0;
   wire        read_rsp_wr_ready;
   reg  [63:0] read_rsp_wr_data = 0;

   // The core only raises reset for this engine after the memory side and all
   // queues are idle.  Configuration-time initial values are enough here; avoid
   // feeding the complex core-reset-home expression into XPM FIFO reset logic in
   // the 333 MHz memory clock domain.
   // All mem_engine CDC FIFOs forced to BRAM (MEMORY_TYPE="block"): same
   // rationale as the MMIO bridge FIFOs in rk_xcku5p.v — distributed RAM
   // implementation in SLICEM cells gets placed across multiple clock
   // regions, the cross-region skew on doutb_reg paths kills timing.
   // BRAM hard blocks have constrained, predictable placement.
   smolrv64_async_fifo #(.WIDTH(25), .ADDR_BITS(4), .MEMORY_TYPE("block")) fill_req_fifo (
      .wr_clock(core_clock), .rd_clock(mem_clock), .reset(1'b0),
      .wr_valid(fill_req_valid), .wr_ready(fill_req_ready), .wr_data(fill_req_line_addr),
      .rd_valid(fill_cmd_valid), .rd_ready(fill_cmd_ready), .rd_data(fill_cmd_line_addr)
   );
   smolrv64_async_fifo #(.WIDTH(512), .ADDR_BITS(4), .MEMORY_TYPE("block")) fill_rsp_fifo (
      .wr_clock(mem_clock), .rd_clock(core_clock), .reset(1'b0),
      .wr_valid(fill_rsp_wr_valid), .wr_ready(fill_rsp_wr_ready), .wr_data(fill_rsp_wr_data),
      .rd_valid(fill_rsp_valid), .rd_ready(fill_rsp_ready), .rd_data(fill_rsp_data)
   );
   smolrv64_async_fifo #(.WIDTH(537), .ADDR_BITS(4), .MEMORY_TYPE("block")) wb_req_fifo (
      .wr_clock(core_clock), .rd_clock(mem_clock), .reset(1'b0),
      .wr_valid(wb_req_valid), .wr_ready(wb_req_ready),
      .wr_data({wb_req_line_addr, wb_req_line_data}),
      .rd_valid(wb_cmd_valid), .rd_ready(wb_cmd_ready), .rd_data(wb_cmd_data)
   );
   smolrv64_async_fifo #(.WIDTH(1), .ADDR_BITS(4), .MEMORY_TYPE("block")) wb_rsp_fifo (
      .wr_clock(mem_clock), .rd_clock(core_clock), .reset(1'b0),
      .wr_valid(wb_rsp_wr_valid), .wr_ready(wb_rsp_wr_ready), .wr_data(1'b1),
      .rd_valid(wb_rsp_valid), .rd_ready(wb_rsp_ready), .rd_data()
   );
   smolrv64_async_fifo #(.WIDTH(28), .ADDR_BITS(4), .MEMORY_TYPE("block")) read_req_fifo (
      .wr_clock(core_clock), .rd_clock(mem_clock), .reset(1'b0),
      .wr_valid(read_req_valid), .wr_ready(read_req_ready), .wr_data(read_req_addr),
      .rd_valid(read_cmd_valid), .rd_ready(read_cmd_ready), .rd_data(read_cmd_addr)
   );
   smolrv64_async_fifo #(.WIDTH(64), .ADDR_BITS(4), .MEMORY_TYPE("block")) read_rsp_fifo (
      .wr_clock(mem_clock), .rd_clock(core_clock), .reset(1'b0),
      .wr_valid(read_rsp_wr_valid), .wr_ready(read_rsp_wr_ready), .wr_data(read_rsp_wr_data),
      .rd_valid(read_rsp_valid), .rd_ready(read_rsp_ready), .rd_data(read_rsp_data)
   );

   localparam [2:0] MEM_IDLE       = 3'd0,
                    MEM_READ_REQ   = 3'd1,
                    MEM_READ_WAIT  = 3'd2,
                    MEM_READ_RESP  = 3'd3,
                    MEM_FILL_REQ   = 3'd4,
                    MEM_FILL_WAIT  = 3'd5,
                    MEM_FILL_RESP  = 3'd6,
                    MEM_WB_REQ     = 3'd7;
   localparam [1:0] MEM_WB_WAIT = 2'd0,
                    MEM_WB_RESP = 2'd1;

   reg [2:0] mem_state = MEM_IDLE;
   reg [1:0] wb_substate = MEM_WB_WAIT;
   reg [27:0] op_addr = 0;
   reg [24:0] op_line_addr = 0;
   reg [511:0] op_line_data = 0;
   reg [2:0] op_beat = 0;
   reg [511:0] fill_line_data = 0;

   reg        ar_busy = 0;
   reg        r_busy  = 0;
   reg [27:0] ar_addr_r = 0;
   reg        aw_busy = 0;
   reg        w_busy  = 0;
   reg        b_busy  = 0;
   reg [27:0] aw_addr_r = 0;
   reg [63:0] w_data_r = 0;
   reg [7:0]  w_strb_r = 0;

   reg mem_busy = 0;
   reg mem_busy_meta = 0;
   reg mem_busy_sync = 0;
   assign idle = !mem_busy_sync &&
                 fill_req_ready && !fill_rsp_valid &&
                 wb_req_ready && !wb_rsp_valid &&
                 read_req_ready && !read_rsp_valid;

   always @(posedge core_clock) begin
      if (reset) begin
         mem_busy_meta <= 0;
         mem_busy_sync <= 0;
      end else begin
         mem_busy_meta <= mem_busy;
         mem_busy_sync <= mem_busy_meta;
      end
   end

   always @(posedge mem_clock) begin
      if (ar_busy && m_axi_arready)
         ar_busy <= 0;
      if (aw_busy && m_axi_awready)
         aw_busy <= 0;
      if (w_busy && m_axi_wready)
         w_busy <= 0;

      case (mem_state)
        MEM_IDLE: begin
           mem_busy <= 0;
           wb_substate <= MEM_WB_WAIT;
           fill_cmd_ready <= 0;
           wb_cmd_ready <= 0;
           read_cmd_ready <= 0;
           if (read_cmd_valid) begin
              read_cmd_ready <= 1;
              if (read_cmd_ready) begin
                 read_cmd_ready <= 0;
                 op_addr <= read_cmd_addr;
                 mem_busy <= 1;
                 mem_state <= MEM_READ_REQ;
              end
           end else if (fill_cmd_valid) begin
              fill_cmd_ready <= 1;
              if (fill_cmd_ready) begin
                 fill_cmd_ready <= 0;
                 op_line_addr <= fill_cmd_line_addr;
                 op_beat <= 0;
                 fill_line_data <= 0;
                 mem_busy <= 1;
                 mem_state <= MEM_FILL_REQ;
              end
           end else if (wb_cmd_valid) begin
              wb_cmd_ready <= 1;
              if (wb_cmd_ready) begin
                 wb_cmd_ready <= 0;
                 op_line_addr <= wb_cmd_data[536:512];
                 op_line_data <= wb_cmd_data[511:0];
                 op_beat <= 0;
                 mem_busy <= 1;
                 mem_state <= MEM_WB_REQ;
              end
           end
        end

        MEM_READ_REQ: begin
           if (!ar_busy && !r_busy) begin
              ar_addr_r <= op_addr;
              ar_busy <= 1;
              r_busy <= 1;
              mem_state <= MEM_READ_WAIT;
           end
        end

        MEM_READ_WAIT: begin
           if (r_busy && m_axi_rvalid) begin
              r_busy <= 0;
              read_rsp_wr_data <= m_axi_rdata;
              mem_state <= MEM_READ_RESP;
           end
        end

        MEM_READ_RESP: begin
           read_rsp_wr_valid <= 1;
           if (read_rsp_wr_valid && read_rsp_wr_ready) begin
              read_rsp_wr_valid <= 0;
              mem_state <= MEM_IDLE;
           end
        end

        MEM_FILL_REQ: begin
           if (!ar_busy && !r_busy) begin
              ar_addr_r <= {op_line_addr, op_beat};
              ar_busy <= 1;
              r_busy <= 1;
              mem_state <= MEM_FILL_WAIT;
           end
        end

        MEM_FILL_WAIT: begin
           if (r_busy && m_axi_rvalid) begin
              r_busy <= 0;
              fill_line_data[op_beat * 64 +: 64] <= m_axi_rdata;
              if (op_beat == 3'd7) begin
                 mem_state <= MEM_FILL_RESP;
              end else begin
                 op_beat <= op_beat + 1'b1;
                 mem_state <= MEM_FILL_REQ;
              end
           end
        end

        MEM_FILL_RESP: begin
           fill_rsp_wr_data <= fill_line_data;
           fill_rsp_wr_valid <= 1;
           if (fill_rsp_wr_valid && fill_rsp_wr_ready) begin
              fill_rsp_wr_valid <= 0;
              mem_state <= MEM_IDLE;
           end
        end

        MEM_WB_REQ: begin
           case (wb_substate)
             MEM_WB_WAIT: begin
                if (!aw_busy && !w_busy && !b_busy) begin
                   aw_addr_r <= {op_line_addr, op_beat};
                   w_data_r <= op_line_data[op_beat * 64 +: 64];
                   w_strb_r <= 8'hff;
                   aw_busy <= 1;
                   w_busy <= 1;
                   b_busy <= 1;
                   wb_substate <= MEM_WB_RESP;
                end
             end
             MEM_WB_RESP: begin
                if (!b_busy && op_beat == 3'd7) begin
                   wb_rsp_wr_valid <= 1;
                   if (wb_rsp_wr_valid && wb_rsp_wr_ready) begin
                      wb_rsp_wr_valid <= 0;
                      mem_state <= MEM_IDLE;
                   end
                end else if (b_busy && m_axi_bvalid) begin
                   b_busy <= 0;
                   if (op_beat == 3'd7) begin
                      wb_substate <= MEM_WB_RESP;
                   end else begin
                      op_beat <= op_beat + 1'b1;
                      wb_substate <= MEM_WB_WAIT;
                   end
                end
             end
             default: begin
                wb_substate <= MEM_WB_WAIT;
             end
           endcase
        end
      endcase

   end

   assign m_axi_arvalid = ar_busy;
   assign m_axi_araddr  = {ar_addr_r, 3'b000};
   assign m_axi_arlen   = 8'd0;
   assign m_axi_arsize  = 3'b011;
   assign m_axi_arburst = 2'b01;
   assign m_axi_arid    = 3'b000;
   assign m_axi_arlock  = 1'b0;
   assign m_axi_arcache = 4'b0011;
   assign m_axi_arprot  = 3'b000;
   assign m_axi_arqos   = 4'b0000;
   assign m_axi_rready  = 1'b1;

   assign m_axi_awvalid = aw_busy;
   assign m_axi_awaddr  = {aw_addr_r, 3'b000};
   assign m_axi_awlen   = 8'd0;
   assign m_axi_awsize  = 3'b011;
   assign m_axi_awburst = 2'b01;
   assign m_axi_awid    = 3'b000;
   assign m_axi_awlock  = 1'b0;
   assign m_axi_awcache = 4'b0011;
   assign m_axi_awprot  = 3'b000;
   assign m_axi_awqos   = 4'b0000;
   assign m_axi_wvalid  = w_busy;
   assign m_axi_wdata   = w_data_r;
   assign m_axi_wstrb   = w_strb_r;
   assign m_axi_wlast   = 1'b1;
   assign m_axi_bready  = 1'b1;
endmodule
