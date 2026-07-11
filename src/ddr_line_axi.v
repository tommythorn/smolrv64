`default_nettype none

// ddr_line_axi -- bridge the probe SoC's 512-bit cache-line memory port (single
// outstanding req/ack, the shape soc_top.ddr_* exposes) to a 64-bit AXI4 master
// (the shape the XCKU5P DDR4 MIG slave -- and the existing core-vs-device arbiter
// -- already speak). One 512-bit line == an 8-beat INCR burst of 64-bit beats.
//
// Same clock as the SoC core: at the chosen 333 MHz single-clock bring-up the
// MIG UI clock and the core clock are one and the same, so this is a plain FSM
// with no CDC. Single outstanding (the line port is single outstanding), so a
// trivial state machine suffices -- no ID tracking, no write/read interleave.
//
// Line address: ddr_addr is the physical line index PA[63:6]. The MIG slave is
// mapped with DDR at BASE, so the AXI byte address is (PA - BASE) truncated to
// AXI_AW bits. With BASE=0x8000_0000 and a 2 GiB window that is just PA[30:0],
// i.e. {ddr_addr[AXI_AW-7:0], 6'b0}. Writes use full byte strobes: the cache
// hands over a complete merged line (write-through), so every byte is valid.
module ddr_line_axi #(
   parameter AXI_AW   = 31,            // MIG slave byte-address width (2 GiB)
   parameter integer LINE_BITS = 512,  // cache line width
   parameter integer AXI_DW    = 64    // AXI data width
) (
   input  wire                  clk,
   input  wire                  reset,

   // -------- line side (from soc_top.ddr_*) --------
   input  wire                  ddr_req,
   input  wire                  ddr_we,
   input  wire [57:0]           ddr_addr,    // line address PA[63:6]
   input  wire [LINE_BITS-1:0]  ddr_wdata,
   output reg  [LINE_BITS-1:0]  ddr_rdata,
   output reg                   ddr_ack,     // 1-cycle pulse; ddr_rdata valid same cycle for reads

   // -------- AXI4 master (to the MIG slave / arbiter) --------
   output reg  [2:0]            m_axi_awid,
   output reg  [AXI_AW-1:0]     m_axi_awaddr,
   output reg  [7:0]            m_axi_awlen,
   output reg  [2:0]            m_axi_awsize,
   output reg  [1:0]            m_axi_awburst,
   output wire                  m_axi_awlock,
   output wire [3:0]            m_axi_awcache,
   output wire [2:0]            m_axi_awprot,
   output wire [3:0]            m_axi_awqos,
   output reg                   m_axi_awvalid,
   input  wire                  m_axi_awready,
   output reg  [AXI_DW-1:0]     m_axi_wdata,
   output wire [AXI_DW/8-1:0]   m_axi_wstrb,
   output reg                   m_axi_wlast,
   output reg                   m_axi_wvalid,
   input  wire                  m_axi_wready,
   output reg  [2:0]            m_axi_bid,
   input  wire [1:0]            m_axi_bresp,
   input  wire                  m_axi_bvalid,
   output reg                   m_axi_bready,
   output reg  [2:0]            m_axi_arid,
   output reg  [AXI_AW-1:0]     m_axi_araddr,
   output reg  [7:0]            m_axi_arlen,
   output reg  [2:0]            m_axi_arsize,
   output reg  [1:0]            m_axi_arburst,
   output wire                  m_axi_arlock,
   output wire [3:0]            m_axi_arcache,
   output wire [2:0]            m_axi_arprot,
   output wire [3:0]            m_axi_arqos,
   output reg                   m_axi_arvalid,
   input  wire                  m_axi_arready,
   input  wire [2:0]            m_axi_rid,
   input  wire [AXI_DW-1:0]     m_axi_rdata,
   input  wire [1:0]            m_axi_rresp,
   input  wire                  m_axi_rlast,
   input  wire                  m_axi_rvalid,
   output reg                   m_axi_rready
);
   localparam integer BEATS = LINE_BITS / AXI_DW;   // 8
   localparam integer BCW   = $clog2(BEATS);        // 3
   // AXI burst size code = log2(bytes per beat). 64-bit beat = 8 bytes -> 3.
   localparam [2:0] SIZE = AXI_DW == 64 ? 3'd3 : AXI_DW == 32 ? 3'd2 : 3'd4;
   localparam [1:0] INCR = 2'b01;

   // Static AXI qualifiers: normal, non-secure, bufferable+modifiable cache hint.
   assign m_axi_awlock  = 1'b0;
   assign m_axi_arlock  = 1'b0;
   assign m_axi_awcache = 4'b0011;
   assign m_axi_arcache = 4'b0011;
   assign m_axi_awprot  = 3'b000;
   assign m_axi_arprot  = 3'b000;
   assign m_axi_awqos   = 4'b0000;
   assign m_axi_arqos   = 4'b0000;
   assign m_axi_wstrb   = {(AXI_DW/8){1'b1}};   // full-line writes

   localparam [2:0] S_IDLE=3'd0, S_RA=3'd1, S_RD=3'd2, S_AW=3'd3, S_WD=3'd4, S_B=3'd5, S_ACK=3'd6;
   reg [2:0]          state;
   reg [BCW-1:0]      beat;
   reg [LINE_BITS-1:0] buf_data;        // write data shifted out / read data shifted in
   reg [AXI_AW-1:0]   req_addr;

   // line base byte address into the MIG: drop the line's low 6 bits (always 0)
   wire [AXI_AW-1:0]  line_byte_addr = {ddr_addr[AXI_AW-7:0], 6'b0};

   always @(posedge clk) begin
      if (reset) begin
         state         <= S_IDLE;
         beat          <= 0;
         ddr_ack       <= 1'b0;
         m_axi_arvalid <= 1'b0;
         m_axi_rready  <= 1'b0;
         m_axi_awvalid <= 1'b0;
         m_axi_wvalid  <= 1'b0;
         m_axi_wlast   <= 1'b0;
         m_axi_bready  <= 1'b0;
         m_axi_awid    <= 3'd0; m_axi_arid <= 3'd0; m_axi_bid <= 3'd0;
         m_axi_awsize  <= SIZE; m_axi_arsize <= SIZE;
         m_axi_awburst <= INCR; m_axi_arburst <= INCR;
         m_axi_awlen   <= BEATS-1; m_axi_arlen <= BEATS-1;
      end else begin
         ddr_ack <= 1'b0;
         case (state)
           S_IDLE: begin
              beat <= 0;
              if (ddr_req) begin
                 req_addr <= line_byte_addr;
                 buf_data <= ddr_wdata;
                 if (ddr_we) begin
                    m_axi_awaddr  <= line_byte_addr;
                    m_axi_awvalid <= 1'b1;
                    state         <= S_AW;
                 end else begin
                    m_axi_araddr  <= line_byte_addr;
                    m_axi_arvalid <= 1'b1;
                    state         <= S_RA;
                 end
              end
           end

           // ---- read: address then BEATS data beats ----
           S_RA: if (m_axi_arready) begin
                    m_axi_arvalid <= 1'b0;
                    m_axi_rready  <= 1'b1;
                    state         <= S_RD;
                 end
           // shift each beat in from the top -> final layout = beat_i at [i*AXI_DW],
           // identical to the old beat-indexed write but with no variable mux.
           S_RD: if (m_axi_rvalid) begin
                    buf_data <= {m_axi_rdata, buf_data[LINE_BITS-1:AXI_DW]};
                    if (m_axi_rlast) begin
                       m_axi_rready <= 1'b0;
                       ddr_rdata    <= {m_axi_rdata, buf_data[LINE_BITS-1:AXI_DW]};
                       ddr_ack      <= 1'b1;
                       state        <= S_IDLE;
                    end
                 end

           // ---- write: address, then BEATS data beats, then response ----
           S_AW: if (m_axi_awready) begin
                    m_axi_awvalid <= 1'b0;
                    m_axi_wdata   <= buf_data[0 +: AXI_DW];   // beat 0
                    buf_data      <= buf_data >> AXI_DW;      // next beat -> low
                    m_axi_wvalid  <= 1'b1;
                    m_axi_wlast   <= (BEATS == 1);
                    state         <= S_WD;
                 end
           S_WD: if (m_axi_wready) begin
                    if (m_axi_wlast) begin
                       m_axi_wvalid <= 1'b0;
                       m_axi_wlast  <= 1'b0;
                       m_axi_bready <= 1'b1;
                       state        <= S_B;
                    end else begin
                       beat        <= beat + 1'b1;
                       m_axi_wdata <= buf_data[0 +: AXI_DW];  // shifted low = next beat
                       buf_data    <= buf_data >> AXI_DW;
                       m_axi_wlast <= (beat + 1'b1 == BEATS-1);
                    end
                 end
           S_B: if (m_axi_bvalid) begin
                    m_axi_bready <= 1'b0;
                    ddr_ack      <= 1'b1;
                    state        <= S_IDLE;
                 end
           default: state <= S_IDLE;
         endcase
      end
   end
endmodule

`default_nettype wire
