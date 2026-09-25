`default_nettype none
// ddr_port_axi -- rv_soc_top's DDR memory port (on ui_clk, after ddr_port_cdc) as a 64-bit AXI4
// master into the DDR4 controller, with up to NOUT reads and NOUT writes in flight.
//
// A read is one 8-beat INCR burst from the line's base; every two 64-bit beats go back as one
// 128-bit port beat, beat k holding the line's bytes [16k, 16k+16). A write is AW plus 8 W beats
// with the port's byte mask as WSTRB, and its write done goes back when B arrives. One AXI ID
// throughout: AXI returns same-ID reads in AR order and same-ID responses in AW order, so an
// in-order FIFO of port ids per channel names each returning transaction -- by the AXI rule,
// not by anything the controller happens to do. A read may issue while an earlier write's
// beats are still going out; writes are posted (the next one does not wait for a B).
//
// Line address: the port's addr is the physical line index PA[63:6]; the controller maps DDR at
// BASE, so the AXI byte address is {addr[AXI_AW-7:0], 6'b0} (PA[30:0] for a 2 GiB window).
module ddr_port_axi #(
   parameter integer IDW    = 5,
   parameter integer AXI_AW = 31,
   parameter integer NOUT   = 8                  // reads (and writes) in flight
) (
   input  wire                clk,
   input  wire                reset,
   // -------- the port (from ddr_port_cdc) --------
   input  wire                q_valid,
   output wire                q_ready,
   input  wire [IDW-1:0]      q_id,
   input  wire                q_we,
   input  wire [57:0]         q_addr,
   input  wire [63:0]         q_wmask,
   input  wire [511:0]        q_wdata,
   output wire                r_valid,
   input  wire                r_ready,
   output wire [IDW-1:0]      r_id,
   output wire [1:0]          r_beat,
   output wire                r_last,
   output wire [127:0]        r_data,
   output wire                w_valid,
   input  wire                w_ready,
   output wire [IDW-1:0]      w_id,
   // -------- AXI4 master (to the MIG slave / arbiter) --------
   output wire [2:0]          m_axi_awid,
   output reg  [AXI_AW-1:0]   m_axi_awaddr,
   output wire [7:0]          m_axi_awlen,
   output wire [2:0]          m_axi_awsize,
   output wire [1:0]          m_axi_awburst,
   output wire                m_axi_awlock,
   output wire [3:0]          m_axi_awcache,
   output wire [2:0]          m_axi_awprot,
   output wire [3:0]          m_axi_awqos,
   output reg                 m_axi_awvalid,
   input  wire                m_axi_awready,
   output wire [63:0]         m_axi_wdata,
   output wire [7:0]          m_axi_wstrb,
   output wire                m_axi_wlast,
   output wire                m_axi_wvalid,
   input  wire                m_axi_wready,
   input  wire [2:0]          m_axi_bid,
   input  wire [1:0]          m_axi_bresp,
   input  wire                m_axi_bvalid,
   output wire                m_axi_bready,
   output wire [2:0]          m_axi_arid,
   output reg  [AXI_AW-1:0]   m_axi_araddr,
   output wire [7:0]          m_axi_arlen,
   output wire [2:0]          m_axi_arsize,
   output wire [1:0]          m_axi_arburst,
   output wire                m_axi_arlock,
   output wire [3:0]          m_axi_arcache,
   output wire [2:0]          m_axi_arprot,
   output wire [3:0]          m_axi_arqos,
   output reg                 m_axi_arvalid,
   input  wire                m_axi_arready,
   input  wire [2:0]          m_axi_rid,
   input  wire [63:0]         m_axi_rdata,
   input  wire [1:0]          m_axi_rresp,
   input  wire                m_axi_rlast,
   input  wire                m_axi_rvalid,
   output wire                m_axi_rready
);
   localparam integer NB = $clog2(NOUT);
   // static AXI fields: ID 0, 8 beats of 8 bytes, INCR, normal non-secure, bufferable+modifiable
   assign m_axi_awid = 3'd0;  assign m_axi_arid = 3'd0;
   assign m_axi_awlen = 8'd7; assign m_axi_arlen = 8'd7;
   assign m_axi_awsize = 3'd3; assign m_axi_arsize = 3'd3;
   assign m_axi_awburst = 2'b01; assign m_axi_arburst = 2'b01;
   assign m_axi_awlock = 1'b0; assign m_axi_arlock = 1'b0;
   assign m_axi_awcache = 4'b0011; assign m_axi_arcache = 4'b0011;
   assign m_axi_awprot = 3'b000; assign m_axi_arprot = 3'b000;
   assign m_axi_awqos = 4'b0000; assign m_axi_arqos = 4'b0000;

   // ---- the in-order id FIFOs: reads in AR order, writes in AW order ----
   reg [IDW-1:0] rt [0:NOUT-1];  reg [NB-1:0] rt_rd, rt_wr;  reg [NB:0] rt_n;
   reg [IDW-1:0] bt [0:NOUT-1];  reg [NB-1:0] bt_rd, bt_wr;  reg [NB:0] bt_n;
   wire rt_full = (rt_n == NOUT[NB:0]);
   wire bt_full = (bt_n == NOUT[NB:0]);

   // ---- dispatch: the port's head goes to the AR register or the write engine ----
   reg  [3:0]   wn;          // W beats still to send
   reg  [511:0] wbuf;
   reg  [63:0]  sbuf;
   wire rd_go = q_valid & ~q_we & ~m_axi_arvalid & ~rt_full;
   wire wr_go = q_valid &  q_we & ~m_axi_awvalid & (wn == 4'd0) & ~bt_full;
   assign q_ready = rd_go | wr_go;
   wire [AXI_AW-1:0] line_byte = {q_addr[AXI_AW-7:0], 6'b0};

   // ---- W beats: shifted out of the line, the byte mask as WSTRB ----
   assign m_axi_wvalid = (wn != 4'd0);
   assign m_axi_wdata  = wbuf[63:0];
   assign m_axi_wstrb  = sbuf[7:0];
   assign m_axi_wlast  = (wn == 4'd1);

   // ---- R: two 64-bit beats make a port beat; the id is the oldest read's ----
   reg  [2:0]  rb;           // the burst's beat
   reg  [63:0] r_lo;         // its even beat, waiting for the odd one
   assign m_axi_rready = ~rb[0] | r_ready;
   wire   r_go   = m_axi_rvalid & m_axi_rready;
   assign r_valid = r_go & rb[0];
   assign r_id    = rt[rt_rd];
   assign r_beat  = rb[2:1];
   assign r_last  = m_axi_rlast;
   assign r_data  = {m_axi_rdata, r_lo};

   // ---- B: the oldest write's id goes back as its write done ----
   assign m_axi_bready = w_ready;
   wire   b_go   = m_axi_bvalid & m_axi_bready;
   assign w_valid = b_go;
   assign w_id    = bt[bt_rd];

   always @(posedge clk) begin
      if (reset) begin
         m_axi_arvalid <= 1'b0; m_axi_awvalid <= 1'b0; wn <= 4'd0; rb <= 3'd0;
         rt_rd <= {NB{1'b0}}; rt_wr <= {NB{1'b0}}; rt_n <= {(NB+1){1'b0}};
         bt_rd <= {NB{1'b0}}; bt_wr <= {NB{1'b0}}; bt_n <= {(NB+1){1'b0}};
      end else begin
         if (m_axi_arvalid & m_axi_arready) m_axi_arvalid <= 1'b0;
         if (m_axi_awvalid & m_axi_awready) m_axi_awvalid <= 1'b0;
         if (rd_go) begin
            m_axi_arvalid <= 1'b1;  m_axi_araddr <= line_byte;
            rt[rt_wr] <= q_id;  rt_wr <= rt_wr + 1'b1;
         end
         if (wr_go) begin
            m_axi_awvalid <= 1'b1;  m_axi_awaddr <= line_byte;
            wbuf <= q_wdata;  sbuf <= q_wmask;  wn <= 4'd8;
            bt[bt_wr] <= q_id;  bt_wr <= bt_wr + 1'b1;
         end else if (m_axi_wvalid & m_axi_wready) begin
            wbuf <= wbuf >> 64;  sbuf <= sbuf >> 8;  wn <= wn - 4'd1;
         end
         if (r_go) begin
            rb <= m_axi_rlast ? 3'd0 : rb + 3'd1;
            if (!rb[0]) r_lo <= m_axi_rdata;
            if (m_axi_rlast) rt_rd <= rt_rd + 1'b1;
         end
         if (b_go) bt_rd <= bt_rd + 1'b1;
         rt_n <= rt_n + {{NB{1'b0}}, rd_go} - {{NB{1'b0}}, r_go & m_axi_rlast};
         bt_n <= bt_n + {{NB{1'b0}}, wr_go} - {{NB{1'b0}}, b_go};
      end
   end

`ifndef SYNTHESIS
   always @(posedge clk) if (!reset) begin
      if (r_go && rt_n == 0)            $fatal(1, "ddr_port_axi: read data with no read in flight");
      if (b_go && bt_n == 0)            $fatal(1, "ddr_port_axi: a write response with no write in flight");
      if (r_go && m_axi_rlast && rb != 3'd7)
         $fatal(1, "ddr_port_axi: rlast on beat %0d of an 8-beat burst", rb);
      if (r_go && m_axi_rid != 3'd0)    $fatal(1, "ddr_port_axi: read data for ID %0d; this master uses ID 0", m_axi_rid);
      if (b_go && m_axi_bid != 3'd0)    $fatal(1, "ddr_port_axi: a write response for ID %0d; this master uses ID 0", m_axi_bid);
      if (r_go && m_axi_rresp != 2'b00) $fatal(1, "ddr_port_axi: read error response %b", m_axi_rresp);
      if (b_go && m_axi_bresp != 2'b00) $fatal(1, "ddr_port_axi: write error response %b", m_axi_bresp);
   end
`endif
endmodule

`default_nettype wire
