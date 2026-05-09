`timescale 1ns / 1ps
`default_nettype none

module axi_single_beat_master(
    input  wire        clock,
    input  wire        reset,

    input  wire        cmd_valid,
    output wire        cmd_ready,
    input  wire        cmd_write,
    input  wire [30:0] cmd_addr,
    input  wire [63:0] cmd_wdata,
    input  wire [ 7:0] cmd_wstrb,
    output reg         rsp_valid,
    output reg  [63:0] rsp_rdata,
    output reg         rsp_error,

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
   localparam [1:0] S_IDLE  = 2'd0;
   localparam [1:0] S_READ  = 2'd1;
   localparam [1:0] S_WRITE = 2'd2;

   reg [1:0]  state;
   reg [30:0] addr_q;
   reg [63:0] wdata_q;
   reg [ 7:0] wstrb_q;
   reg        ar_pending;
   reg        aw_pending;
   reg        w_pending;

   assign cmd_ready = state == S_IDLE;

   always @(posedge clock) begin
      rsp_valid <= 1'b0;

      if (reset) begin
         state <= S_IDLE;
         addr_q <= 31'd0;
         wdata_q <= 64'd0;
         wstrb_q <= 8'd0;
         ar_pending <= 1'b0;
         aw_pending <= 1'b0;
         w_pending <= 1'b0;
         rsp_valid <= 1'b0;
         rsp_rdata <= 64'd0;
         rsp_error <= 1'b0;
      end else begin
         case (state)
           S_IDLE: begin
              if (cmd_valid) begin
                 addr_q <= {cmd_addr[30:3], 3'b000};
                 wdata_q <= cmd_wdata;
                 wstrb_q <= cmd_wstrb;
                 rsp_error <= 1'b0;
                 if (cmd_write) begin
                    state <= S_WRITE;
                    aw_pending <= 1'b1;
                    w_pending <= 1'b1;
                 end else begin
                    state <= S_READ;
                    ar_pending <= 1'b1;
                 end
              end
           end

           S_READ: begin
              if (m_axi_arready)
                 ar_pending <= 1'b0;
              if (m_axi_rvalid) begin
                 rsp_valid <= 1'b1;
                 rsp_rdata <= m_axi_rdata;
                 rsp_error <= m_axi_rresp != 2'b00 || !m_axi_rlast;
                 state <= S_IDLE;
                 ar_pending <= 1'b0;
              end
           end

           S_WRITE: begin
              if (m_axi_awready)
                 aw_pending <= 1'b0;
              if (m_axi_wready)
                 w_pending <= 1'b0;
              if (m_axi_bvalid) begin
                 rsp_valid <= 1'b1;
                 rsp_rdata <= 64'd0;
                 rsp_error <= m_axi_bresp != 2'b00;
                 state <= S_IDLE;
                 aw_pending <= 1'b0;
                 w_pending <= 1'b0;
              end
           end

           default: state <= S_IDLE;
         endcase
      end
   end

   assign m_axi_arvalid = state == S_READ && ar_pending;
   assign m_axi_araddr  = addr_q;
   assign m_axi_arlen   = 8'd0;
   assign m_axi_arsize  = 3'b011;
   assign m_axi_arburst = 2'b01;
   assign m_axi_arid    = 3'b001;
   assign m_axi_arlock  = 1'b0;
   assign m_axi_arcache = 4'b0011;
   assign m_axi_arprot  = 3'b000;
   assign m_axi_arqos   = 4'd0;
   assign m_axi_rready  = state == S_READ;

   assign m_axi_awvalid = state == S_WRITE && aw_pending;
   assign m_axi_awaddr  = addr_q;
   assign m_axi_awlen   = 8'd0;
   assign m_axi_awsize  = 3'b011;
   assign m_axi_awburst = 2'b01;
   assign m_axi_awid    = 3'b001;
   assign m_axi_awlock  = 1'b0;
   assign m_axi_awcache = 4'b0011;
   assign m_axi_awprot  = 3'b000;
   assign m_axi_awqos   = 4'd0;
   assign m_axi_wvalid  = state == S_WRITE && w_pending;
   assign m_axi_wdata   = wdata_q;
   assign m_axi_wstrb   = wstrb_q;
   assign m_axi_wlast   = 1'b1;
   assign m_axi_bready  = state == S_WRITE;

   wire unused_axi_inputs = &{1'b0, m_axi_bid, m_axi_rid, cmd_addr[2:0]};
endmodule

`default_nettype wire
