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

   wire s0_write_req = s0_axi_awvalid | s0_axi_wvalid;
   wire write_sel = write_active ? write_owner : (!s0_write_req && (s1_axi_awvalid | s1_axi_wvalid));
   wire read_sel = read_active ? read_owner : (!s0_axi_arvalid && s1_axi_arvalid);

   always @(posedge clock) begin
      if (reset) begin
         write_active <= 1'b0;
         write_owner <= 1'b0;
         read_active <= 1'b0;
         read_owner <= 1'b0;
      end else begin
         if (!write_active && (s0_write_req || s1_axi_awvalid || s1_axi_wvalid)) begin
            write_active <= 1'b1;
            write_owner <= write_sel;
         end else if (m_axi_bvalid && m_axi_bready) begin
            write_active <= 1'b0;
         end

         if (!read_active && (s0_axi_arvalid || s1_axi_arvalid)) begin
            read_active <= 1'b1;
            read_owner <= read_sel;
         end else if (m_axi_rvalid && m_axi_rready && m_axi_rlast) begin
            read_active <= 1'b0;
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

`default_nettype wire
