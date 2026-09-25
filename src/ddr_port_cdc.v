`default_nettype none
// ddr_port_cdc -- rv_soc_top's DDR memory port (core clock) across into ui_clk, as three
// asynchronous FIFOs, one per channel: requests go down, read beats and write dones come back.
// Every transaction is independent, so any number may be in flight and none waits for a round
// trip of its own. Block-RAM FIFOs: distributed-RAM async FIFOs do not meet timing across this
// platform's write/read clock-root skew.
//
// The port's channels are described in ooo2/rv_mem_arbiter.v and
// docs/PLAN-2026-09-25-dcache-vhpr.md ("The memory port").
module ddr_port_cdc #(
   parameter integer IDW = 5
) (
   // ---- core side (clk_p) ----
   input  wire           clk_p,
   input  wire           reset_p,
   input  wire           p_q_valid,
   output wire           p_q_ready,
   input  wire [IDW-1:0] p_q_id,
   input  wire           p_q_we,
   input  wire [57:0]    p_q_addr,
   input  wire [63:0]    p_q_wmask,
   input  wire [511:0]   p_q_wdata,
   output wire           p_r_valid,
   input  wire           p_r_ready,
   output wire [IDW-1:0] p_r_id,
   output wire [1:0]     p_r_beat,
   output wire           p_r_last,
   output wire [127:0]   p_r_data,
   output wire           p_w_valid,
   input  wire           p_w_ready,
   output wire [IDW-1:0] p_w_id,
   // ---- memory side (clk_m = ui_clk) ----
   input  wire           clk_m,
   input  wire           reset_m,
   output wire           m_q_valid,
   input  wire           m_q_ready,
   output wire [IDW-1:0] m_q_id,
   output wire           m_q_we,
   output wire [57:0]    m_q_addr,
   output wire [63:0]    m_q_wmask,
   output wire [511:0]   m_q_wdata,
   input  wire           m_r_valid,
   output wire           m_r_ready,
   input  wire [IDW-1:0] m_r_id,
   input  wire [1:0]     m_r_beat,
   input  wire           m_r_last,
   input  wire [127:0]   m_r_data,
   input  wire           m_w_valid,
   output wire           m_w_ready,
   input  wire [IDW-1:0] m_w_id
);
   localparam integer QW = IDW + 1 + 58 + 64 + 512;
   localparam integer RW = IDW + 2 + 1 + 128;
   smolrv64_async_fifo #(.WIDTH(QW), .ADDR_BITS(4), .MEMORY_TYPE("block")) u_q
     (.wr_clock(clk_p), .rd_clock(clk_m), .reset(reset_p),
      .wr_valid(p_q_valid), .wr_ready(p_q_ready), .wr_data({p_q_id, p_q_we, p_q_addr, p_q_wmask, p_q_wdata}),
      .rd_valid(m_q_valid), .rd_ready(m_q_ready), .rd_data({m_q_id, m_q_we, m_q_addr, m_q_wmask, m_q_wdata}));
   smolrv64_async_fifo #(.WIDTH(RW), .ADDR_BITS(4), .MEMORY_TYPE("block")) u_r
     (.wr_clock(clk_m), .rd_clock(clk_p), .reset(reset_m),
      .wr_valid(m_r_valid), .wr_ready(m_r_ready), .wr_data({m_r_id, m_r_beat, m_r_last, m_r_data}),
      .rd_valid(p_r_valid), .rd_ready(p_r_ready), .rd_data({p_r_id, p_r_beat, p_r_last, p_r_data}));
   smolrv64_async_fifo #(.WIDTH(IDW), .ADDR_BITS(4), .MEMORY_TYPE("block")) u_w
     (.wr_clock(clk_m), .rd_clock(clk_p), .reset(reset_m),
      .wr_valid(m_w_valid), .wr_ready(m_w_ready), .wr_data(m_w_id),
      .rd_valid(p_w_valid), .rd_ready(p_w_ready), .rd_data(p_w_id));
endmodule

`default_nettype wire
