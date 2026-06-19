module smolrv64_l2_boundary(
   input  wire        core_clock,
   input  wire        mem_clock,
   input  wire        reset,
   output wire        idle,

   input  wire        icache_fill_req_valid,
   output wire        icache_fill_req_ready,
   input  wire [24:0] icache_fill_req_line_addr,
   output wire        icache_fill_rsp_valid,
   input  wire        icache_fill_rsp_ready,
   output wire [511:0] icache_fill_rsp_data,

   input  wire        dcache_fill_req_valid,
   output wire        dcache_fill_req_ready,
   input  wire [24:0] dcache_fill_req_line_addr,
   output wire        dcache_fill_rsp_valid,
   input  wire        dcache_fill_rsp_ready,
   output wire [511:0] dcache_fill_rsp_data,

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
   wire        fill_req_valid;
   wire        fill_req_ready;
   wire [24:0] fill_req_line_addr;
   wire        fill_rsp_valid;
   wire        fill_rsp_ready;
   wire [511:0] fill_rsp_data;
   reg         fill_rsp_owner_icache = 0;

   assign fill_req_valid = icache_fill_req_valid || dcache_fill_req_valid;
   assign fill_req_line_addr = icache_fill_req_valid ? icache_fill_req_line_addr :
                                                     dcache_fill_req_line_addr;
   assign icache_fill_req_ready = fill_req_ready && icache_fill_req_valid;
   assign dcache_fill_req_ready =
      fill_req_ready && !icache_fill_req_valid && dcache_fill_req_valid;

   assign icache_fill_rsp_valid = fill_rsp_valid && fill_rsp_owner_icache;
   assign dcache_fill_rsp_valid = fill_rsp_valid && !fill_rsp_owner_icache;
   assign icache_fill_rsp_data = fill_rsp_data;
   assign dcache_fill_rsp_data = fill_rsp_data;
   assign fill_rsp_ready = fill_rsp_owner_icache ? icache_fill_rsp_ready :
                                                   dcache_fill_rsp_ready;

   always @(posedge core_clock) begin
      if (reset)
         fill_rsp_owner_icache <= 1'b0;
      else if (fill_req_valid && fill_req_ready)
         fill_rsp_owner_icache <= icache_fill_req_valid;
   end

   smolrv64_mem_engine mem_engine_inst (
      .core_clock          (core_clock),
      .mem_clock           (mem_clock),
      .reset               (reset),
      .idle                (idle),

      .fill_req_valid      (fill_req_valid),
      .fill_req_ready      (fill_req_ready),
      .fill_req_line_addr  (fill_req_line_addr),
      .fill_rsp_valid      (fill_rsp_valid),
      .fill_rsp_ready      (fill_rsp_ready),
      .fill_rsp_data       (fill_rsp_data),

      .wb_req_valid        (wb_req_valid),
      .wb_req_ready        (wb_req_ready),
      .wb_req_line_addr    (wb_req_line_addr),
      .wb_req_line_data    (wb_req_line_data),
      .wb_rsp_valid        (wb_rsp_valid),
      .wb_rsp_ready        (wb_rsp_ready),

      .read_req_valid      (read_req_valid),
      .read_req_ready      (read_req_ready),
      .read_req_addr       (read_req_addr),
      .read_rsp_valid      (read_rsp_valid),
      .read_rsp_ready      (read_rsp_ready),
      .read_rsp_data       (read_rsp_data),

      .m_axi_awid          (m_axi_awid),
      .m_axi_awaddr        (m_axi_awaddr),
      .m_axi_awlen         (m_axi_awlen),
      .m_axi_awsize        (m_axi_awsize),
      .m_axi_awburst       (m_axi_awburst),
      .m_axi_awlock        (m_axi_awlock),
      .m_axi_awcache       (m_axi_awcache),
      .m_axi_awprot        (m_axi_awprot),
      .m_axi_awqos         (m_axi_awqos),
      .m_axi_awvalid       (m_axi_awvalid),
      .m_axi_awready       (m_axi_awready),
      .m_axi_wdata         (m_axi_wdata),
      .m_axi_wstrb         (m_axi_wstrb),
      .m_axi_wlast         (m_axi_wlast),
      .m_axi_wvalid        (m_axi_wvalid),
      .m_axi_wready        (m_axi_wready),
      .m_axi_bid           (m_axi_bid),
      .m_axi_bresp         (m_axi_bresp),
      .m_axi_bvalid        (m_axi_bvalid),
      .m_axi_bready        (m_axi_bready),
      .m_axi_arid          (m_axi_arid),
      .m_axi_araddr        (m_axi_araddr),
      .m_axi_arlen         (m_axi_arlen),
      .m_axi_arsize        (m_axi_arsize),
      .m_axi_arburst       (m_axi_arburst),
      .m_axi_arlock        (m_axi_arlock),
      .m_axi_arcache       (m_axi_arcache),
      .m_axi_arprot        (m_axi_arprot),
      .m_axi_arqos         (m_axi_arqos),
      .m_axi_arvalid       (m_axi_arvalid),
      .m_axi_arready       (m_axi_arready),
      .m_axi_rid           (m_axi_rid),
      .m_axi_rdata         (m_axi_rdata),
      .m_axi_rresp         (m_axi_rresp),
      .m_axi_rlast         (m_axi_rlast),
      .m_axi_rvalid        (m_axi_rvalid),
      .m_axi_rready        (m_axi_rready)
   );
endmodule
