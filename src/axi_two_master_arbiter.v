`timescale 1ns / 1ps
`default_nettype none

module axi_two_master_arbiter(
    input  wire        clock,
    input  wire        reset,

    input  wire [ 2:0] s0_axi_awid,
    input  wire [30:0] s0_axi_awaddr,
    input  wire [ 7:0] s0_axi_awlen,
    input  wire [ 2:0] s0_axi_awsize,
    input  wire [ 1:0] s0_axi_awburst,
    input  wire        s0_axi_awlock,
    input  wire [ 3:0] s0_axi_awcache,
    input  wire [ 2:0] s0_axi_awprot,
    input  wire [ 3:0] s0_axi_awqos,
    input  wire        s0_axi_awvalid,
    output wire        s0_axi_awready,
    input  wire [63:0] s0_axi_wdata,
    input  wire [ 7:0] s0_axi_wstrb,
    input  wire        s0_axi_wlast,
    input  wire        s0_axi_wvalid,
    output wire        s0_axi_wready,
    output wire [ 2:0] s0_axi_bid,
    output wire [ 1:0] s0_axi_bresp,
    output wire        s0_axi_bvalid,
    input  wire        s0_axi_bready,
    input  wire [ 2:0] s0_axi_arid,
    input  wire [30:0] s0_axi_araddr,
    input  wire [ 7:0] s0_axi_arlen,
    input  wire [ 2:0] s0_axi_arsize,
    input  wire [ 1:0] s0_axi_arburst,
    input  wire        s0_axi_arlock,
    input  wire [ 3:0] s0_axi_arcache,
    input  wire [ 2:0] s0_axi_arprot,
    input  wire [ 3:0] s0_axi_arqos,
    input  wire        s0_axi_arvalid,
    output wire        s0_axi_arready,
    output wire [ 2:0] s0_axi_rid,
    output wire [63:0] s0_axi_rdata,
    output wire [ 1:0] s0_axi_rresp,
    output wire        s0_axi_rlast,
    output wire        s0_axi_rvalid,
    input  wire        s0_axi_rready,

    input  wire [ 2:0] s1_axi_awid,
    input  wire [30:0] s1_axi_awaddr,
    input  wire [ 7:0] s1_axi_awlen,
    input  wire [ 2:0] s1_axi_awsize,
    input  wire [ 1:0] s1_axi_awburst,
    input  wire        s1_axi_awlock,
    input  wire [ 3:0] s1_axi_awcache,
    input  wire [ 2:0] s1_axi_awprot,
    input  wire [ 3:0] s1_axi_awqos,
    input  wire        s1_axi_awvalid,
    output wire        s1_axi_awready,
    input  wire [63:0] s1_axi_wdata,
    input  wire [ 7:0] s1_axi_wstrb,
    input  wire        s1_axi_wlast,
    input  wire        s1_axi_wvalid,
    output wire        s1_axi_wready,
    output wire [ 2:0] s1_axi_bid,
    output wire [ 1:0] s1_axi_bresp,
    output wire        s1_axi_bvalid,
    input  wire        s1_axi_bready,
    input  wire [ 2:0] s1_axi_arid,
    input  wire [30:0] s1_axi_araddr,
    input  wire [ 7:0] s1_axi_arlen,
    input  wire [ 2:0] s1_axi_arsize,
    input  wire [ 1:0] s1_axi_arburst,
    input  wire        s1_axi_arlock,
    input  wire [ 3:0] s1_axi_arcache,
    input  wire [ 2:0] s1_axi_arprot,
    input  wire [ 3:0] s1_axi_arqos,
    input  wire        s1_axi_arvalid,
    output wire        s1_axi_arready,
    output wire [ 2:0] s1_axi_rid,
    output wire [63:0] s1_axi_rdata,
    output wire [ 1:0] s1_axi_rresp,
    output wire        s1_axi_rlast,
    output wire        s1_axi_rvalid,
    input  wire        s1_axi_rready,

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
   reg write_active;
   reg write_owner;
   reg read_active;
   reg read_owner;
   reg write_prefer_s1;
   reg read_prefer_s1;

   wire s0_write_req = s0_axi_awvalid | s0_axi_wvalid;
   wire s1_write_req = s1_axi_awvalid | s1_axi_wvalid;
   wire write_grant_s1 = s1_write_req && (!s0_write_req || write_prefer_s1);
   wire read_grant_s1 = s1_axi_arvalid && (!s0_axi_arvalid || read_prefer_s1);
   wire write_sel = write_active ? write_owner : write_grant_s1;
   wire read_sel = read_active ? read_owner : read_grant_s1;

   always @(posedge clock) begin
      if (reset) begin
         write_active <= 1'b0;
         write_owner <= 1'b0;
         read_active <= 1'b0;
         read_owner <= 1'b0;
         write_prefer_s1 <= 1'b0;
         read_prefer_s1 <= 1'b0;
      end else begin
         if (!write_active && (s0_write_req || s1_write_req)) begin
            write_active <= 1'b1;
            write_owner <= write_grant_s1;
         end else if (m_axi_bvalid && m_axi_bready) begin
            write_active <= 1'b0;
            write_prefer_s1 <= !write_owner;
         end

         if (!read_active && (s0_axi_arvalid || s1_axi_arvalid)) begin
            read_active <= 1'b1;
            read_owner <= read_grant_s1;
         end else if (m_axi_rvalid && m_axi_rready && m_axi_rlast) begin
            read_active <= 1'b0;
            read_prefer_s1 <= !read_owner;
         end
      end
   end

   assign m_axi_awid    = write_sel ? s1_axi_awid    : s0_axi_awid;
   assign m_axi_awaddr  = write_sel ? s1_axi_awaddr  : s0_axi_awaddr;
   assign m_axi_awlen   = write_sel ? s1_axi_awlen   : s0_axi_awlen;
   assign m_axi_awsize  = write_sel ? s1_axi_awsize  : s0_axi_awsize;
   assign m_axi_awburst = write_sel ? s1_axi_awburst : s0_axi_awburst;
   assign m_axi_awlock  = write_sel ? s1_axi_awlock  : s0_axi_awlock;
   assign m_axi_awcache = write_sel ? s1_axi_awcache : s0_axi_awcache;
   assign m_axi_awprot  = write_sel ? s1_axi_awprot  : s0_axi_awprot;
   assign m_axi_awqos   = write_sel ? s1_axi_awqos   : s0_axi_awqos;
   assign m_axi_awvalid = write_sel ? s1_axi_awvalid : s0_axi_awvalid;
   assign m_axi_wdata   = write_sel ? s1_axi_wdata   : s0_axi_wdata;
   assign m_axi_wstrb   = write_sel ? s1_axi_wstrb   : s0_axi_wstrb;
   assign m_axi_wlast   = write_sel ? s1_axi_wlast   : s0_axi_wlast;
   assign m_axi_wvalid  = write_sel ? s1_axi_wvalid  : s0_axi_wvalid;
   assign m_axi_bready  = write_sel ? s1_axi_bready  : s0_axi_bready;

   assign s0_axi_awready = !write_sel ? m_axi_awready : 1'b0;
   assign s0_axi_wready  = !write_sel ? m_axi_wready  : 1'b0;
   assign s0_axi_bid     = m_axi_bid;
   assign s0_axi_bresp   = m_axi_bresp;
   assign s0_axi_bvalid  = !write_sel ? m_axi_bvalid : 1'b0;

   assign s1_axi_awready = write_sel ? m_axi_awready : 1'b0;
   assign s1_axi_wready  = write_sel ? m_axi_wready  : 1'b0;
   assign s1_axi_bid     = m_axi_bid;
   assign s1_axi_bresp   = m_axi_bresp;
   assign s1_axi_bvalid  = write_sel ? m_axi_bvalid : 1'b0;

   assign m_axi_arid     = read_sel ? s1_axi_arid    : s0_axi_arid;
   assign m_axi_araddr   = read_sel ? s1_axi_araddr  : s0_axi_araddr;
   assign m_axi_arlen    = read_sel ? s1_axi_arlen   : s0_axi_arlen;
   assign m_axi_arsize   = read_sel ? s1_axi_arsize  : s0_axi_arsize;
   assign m_axi_arburst  = read_sel ? s1_axi_arburst : s0_axi_arburst;
   assign m_axi_arlock   = read_sel ? s1_axi_arlock  : s0_axi_arlock;
   assign m_axi_arcache  = read_sel ? s1_axi_arcache : s0_axi_arcache;
   assign m_axi_arprot   = read_sel ? s1_axi_arprot  : s0_axi_arprot;
   assign m_axi_arqos    = read_sel ? s1_axi_arqos   : s0_axi_arqos;
   assign m_axi_arvalid  = read_sel ? s1_axi_arvalid : s0_axi_arvalid;
   assign m_axi_rready   = read_sel ? s1_axi_rready  : s0_axi_rready;

   assign s0_axi_arready = !read_sel ? m_axi_arready : 1'b0;
   assign s0_axi_rid     = m_axi_rid;
   assign s0_axi_rdata   = m_axi_rdata;
   assign s0_axi_rresp   = m_axi_rresp;
   assign s0_axi_rlast   = m_axi_rlast;
   assign s0_axi_rvalid  = !read_sel ? m_axi_rvalid : 1'b0;

   assign s1_axi_arready = read_sel ? m_axi_arready : 1'b0;
   assign s1_axi_rid     = m_axi_rid;
   assign s1_axi_rdata   = m_axi_rdata;
   assign s1_axi_rresp   = m_axi_rresp;
   assign s1_axi_rlast   = m_axi_rlast;
   assign s1_axi_rvalid  = read_sel ? m_axi_rvalid : 1'b0;
endmodule

// Single-entry AXI read-response (R channel) register slice / skid buffer.
// Registers the forward path (rvalid/rid/rdata/rresp/rlast) so a downstream
// consumer no longer sees a combinational function of the arbiter's grant
// (which combinationally depends on the *other* master's arvalid). Breaks the
// long cross-die path from the virtio DMA FSM to the 512-bit line-buffer
// capture enable, at the cost of one cycle of read latency (invisible to
// block DMA / cache-line fills). Full throughput: accepts a beat every cycle
// the downstream consumes one (no bubble).
module axi_r_reg_slice #(
   parameter IDW = 3,
   parameter DW  = 64
)(
   input  wire            clock,
   input  wire            reset,
   // upstream: from arbiter s0 R output
   input  wire [IDW-1:0]  s_rid,
   input  wire [DW-1:0]   s_rdata,
   input  wire [1:0]      s_rresp,
   input  wire            s_rlast,
   input  wire            s_rvalid,
   output wire            s_rready,
   // downstream: to the line-buffer master
   output wire [IDW-1:0]  m_rid,
   output wire [DW-1:0]   m_rdata,
   output wire [1:0]      m_rresp,
   output wire            m_rlast,
   output wire            m_rvalid,
   input  wire            m_rready
);
   reg            full;
   reg [IDW-1:0]  rid_q;
   reg [DW-1:0]   rdata_q;
   reg [1:0]      rresp_q;
   reg            rlast_q;

   assign s_rready = !full || m_rready;   // can accept when empty or draining
   assign m_rvalid = full;
   assign m_rid    = rid_q;
   assign m_rdata  = rdata_q;
   assign m_rresp  = rresp_q;
   assign m_rlast  = rlast_q;

   always @(posedge clock) begin
      if (reset) begin
         full <= 1'b0;
      end else if (s_rvalid && s_rready) begin
         rid_q   <= s_rid;
         rdata_q <= s_rdata;
         rresp_q <= s_rresp;
         rlast_q <= s_rlast;
         full    <= 1'b1;
      end else if (m_rready) begin
         full    <= 1'b0;
      end
   end
endmodule

`default_nettype wire

// axi_aww_reg_slice -- single-entry skid buffers on the AW and W channels (independent,
// both lossless and order-preserving). Breaks the arbiter->MIG-upsizer setup paths at
// 333 MHz (the -0.57 device-DMA wdata / awaddr cones), same recipe as axi_r_reg_slice.
module axi_aww_reg_slice #(parameter IDW = 3, AW = 31, DW = 64)
  (input  wire            clock,
   input  wire            reset,
   // slave side (from the arbiter)
   input  wire [IDW-1:0]  s_awid,
   input  wire [AW-1:0]   s_awaddr,
   input  wire [7:0]      s_awlen,
   input  wire [2:0]      s_awsize,
   input  wire [1:0]      s_awburst,
   input  wire            s_awlock,
   input  wire [3:0]      s_awcache,
   input  wire [2:0]      s_awprot,
   input  wire [3:0]      s_awqos,
   input  wire            s_awvalid,
   output wire            s_awready,
   input  wire [DW-1:0]   s_wdata,
   input  wire [DW/8-1:0] s_wstrb,
   input  wire            s_wlast,
   input  wire            s_wvalid,
   output wire            s_wready,
   // master side (to the MIG)
   output reg  [IDW-1:0]  m_awid,
   output reg  [AW-1:0]   m_awaddr,
   output reg  [7:0]      m_awlen,
   output reg  [2:0]      m_awsize,
   output reg  [1:0]      m_awburst,
   output reg             m_awlock,
   output reg  [3:0]      m_awcache,
   output reg  [2:0]      m_awprot,
   output reg  [3:0]      m_awqos,
   output wire            m_awvalid,
   input  wire            m_awready,
   output reg  [DW-1:0]   m_wdata,
   output reg  [DW/8-1:0] m_wstrb,
   output reg             m_wlast,
   output wire            m_wvalid,
   input  wire            m_wready);

   reg aw_full, w_full;
   assign s_awready = !aw_full || m_awready;
   assign m_awvalid = aw_full;
   assign s_wready  = !w_full  || m_wready;
   assign m_wvalid  = w_full;
   always @(posedge clock) begin
      if (reset) begin aw_full <= 1'b0; w_full <= 1'b0; end
      else begin
         if (s_awvalid && s_awready) begin
            m_awid <= s_awid; m_awaddr <= s_awaddr; m_awlen <= s_awlen;
            m_awsize <= s_awsize; m_awburst <= s_awburst; m_awlock <= s_awlock;
            m_awcache <= s_awcache; m_awprot <= s_awprot; m_awqos <= s_awqos;
            aw_full <= 1'b1;
         end else if (m_awready) aw_full <= 1'b0;
         if (s_wvalid && s_wready) begin
            m_wdata <= s_wdata; m_wstrb <= s_wstrb; m_wlast <= s_wlast;
            w_full <= 1'b1;
         end else if (m_wready) w_full <= 1'b0;
      end
   end
endmodule
