`default_nettype none
// ddr4_port_line -- rv_soc_top's DDR memory port (on ui_clk, after ddr_port_sync) as a requester of
// ddr4_ctl: the caches' tagged line port (src/../core/rv_mem_arbiter.v) on the controller's
// buffer-tagged requests.
//
// It owns eight read and four write line buffers (the controller holds no data): a request takes a free
// buffer of its kind and goes on as {buffer, line}; the PHY's read bursts land in the read buffer
// its tag names (rdDataAddr = {buffer, half}), and the write's data waits in its write buffer until
// wrDataEn/wrDataAddr ask for a half, which this hands over the next cycle (wd_*). A read whose
// second burst has landed streams back as four 128-bit port beats, beat k holding the line's
// bytes [16k, 16k+16), in the order the lines completed; a write's done goes back once its second
// burst's data has gone. Buffers 0..7 are its reads, 8..11 its writes; 12..15 are the DMA port's.
module ddr4_port_line #(
   parameter integer IDW = 5
) (
   input  wire            clk,
   input  wire            reset,
   // ---- the port (ddr_port_sync's memory side) ----
   input  wire            q_valid,
   output wire            q_ready,
   input  wire [IDW-1:0]  q_id,
   input  wire            q_we,
   input  wire [57:0]     q_addr,               // the physical line index PA[63:6]
   input  wire [63:0]     q_wmask,
   input  wire [511:0]    q_wdata,
   output reg             r_valid,
   input  wire            r_ready,
   output reg  [IDW-1:0]  r_id,
   output reg  [1:0]      r_beat,
   output reg             r_last,
   output reg  [127:0]    r_data,
   output wire            w_valid,
   input  wire            w_ready,
   output wire [IDW-1:0]  w_id,
   // ---- ddr4_ctl's request port ----
   output reg             rq_valid,
   input  wire            rq_ready,
   output reg             rq_we,
   output reg  [25:0]     rq_line,
   output reg  [3:0]      rq_buf,
   // ---- the PHY's data ----
   input  wire            rdDataEn,
   input  wire [4:0]      rdDataAddr,
   input  wire [255:0]    rdData,
   input  wire            wrDataEn,
   input  wire [4:0]      wrDataAddr,
   output wire            wd_mine,              // the PHY asks this port for write data: wd_* are the half it names
   output wire [255:0]    wd_data,
   output wire [31:0]     wd_mask               // 1: the byte is not written
);
   localparam integer NR = 8, RB = 3;            // read buffers 0..7: tags {0, buffer, half}
   localparam integer NW = 4, WB = 2;            // write buffers 8..11: tags {2'b10, buffer, half}
`include "dram_window.vh"

   // ---- buffers: free maps, owners' ids, the read buffers' landed halves ----
   reg  [NR-1:0]   r_busy;
   reg  [1:0]      r_got [0:NR-1];
   reg  [IDW-1:0]  r_ids [0:NR-1];
   reg  [NW-1:0]   w_busy, w_done;              // allocated; its data has gone, the done is owed
   reg  [IDW-1:0]  w_ids [0:NW-1];
   reg  [127:0]    rev [0:2*NR-1];              // the read lines' even and odd beats at {buffer, half},
   reg  [127:0]    rod [0:2*NR-1];              // written by the PHY
   reg  [287:0]    wlo [0:NW-1];                // the write lines' halves, {mask_n, data}
   reg  [287:0]    whi [0:NW-1];

   // the lowest free buffer of each kind
   reg  [RB-1:0]   r_free;  reg r_any;
   reg  [WB-1:0]   w_free;  reg w_any;
   integer j;
   always @* begin
      r_any = 1'b0; r_free = {RB{1'b0}};
      for (j = NR - 1; j >= 0; j = j - 1) if (!r_busy[j]) begin r_any = 1'b1; r_free = j[RB-1:0]; end
      w_any = 1'b0; w_free = {WB{1'b0}};
      for (j = NW - 1; j >= 0; j = j - 1) if (!w_busy[j]) begin w_any = 1'b1; w_free = j[WB-1:0]; end
   end

   // ---- requests: a buffer of the request's kind as it leaves the crossing, then a register, then
   // the controller (the register keeps the crossing's capture off the controller's entry) ----
   wire        rq_go   = rq_valid && rq_ready;
   assign q_ready = (q_we ? w_any : r_any) && (!rq_valid || rq_go);
   // A line outside the DRAM window (a wrong-path I$ fetch asks for PA 0) is served from the window's
   // wrap, dram_axi's low 32 bits, as the AXI path to the IP's controller always served it.
   wire [31:0] dram_byte = dram_axi({q_addr, 6'b0});

   // ---- read bursts land; a line whose second burst lands joins the stream queue ----
   wire            rd_mine = rdDataEn && !rdDataAddr[4];
   wire [RB-1:0]   rd_j    = rdDataAddr[RB:1];
   reg  [RB-1:0]   sq [0:NR-1];                 // completed lines, in completion order
   reg  [RB:0]     sq_n;
   // ---- the stream: one 128-bit beat a cycle into the port's output register ----
   wire            s_on   = (sq_n != {(RB+1){1'b0}});
   wire [RB-1:0]   s_j    = sq[0];
   reg  [1:0]      s_beat;
   wire            s_load = s_on && (!r_valid || r_ready);
   wire            s_end  = s_load && (s_beat == 2'd3);
   wire [127:0]    s_ev   = rev[{s_j, s_beat[1]}],  s_od = rod[{s_j, s_beat[1]}];

   // ---- writes: the data a half at a time, the next cycle; dones in buffer order ----
   wire            wr_mine = wrDataEn && (wrDataAddr[4:3] == 2'b10);
   wire [WB-1:0]   wr_j    = wrDataAddr[WB:1];
   // the half the PHY asks for, read as it asks; ddr4_mem registers it for the PHY, which takes it a cycle later
   assign wd_mine = wr_mine;
   assign {wd_mask, wd_data} = wrDataAddr[0] ? whi[wr_j] : wlo[wr_j];
   reg             w_any_done;  reg [WB-1:0] w_dj;
   always @* begin
      w_any_done = 1'b0; w_dj = {WB{1'b0}};
      for (j = NW - 1; j >= 0; j = j - 1) if (w_done[j]) begin w_any_done = 1'b1; w_dj = j[WB-1:0]; end
   end
   assign w_valid = w_any_done;
   assign w_id    = w_ids[w_dj];
   wire            w_ret  = w_valid && w_ready;
   reg             w_last_q;  reg [WB-1:0] w_last_j;   // a write's second half went out last cycle

   always @(posedge clk) begin
      // the buffers' contents: one write statement a memory
      if (q_valid && q_ready && !q_we) r_ids[r_free] <= q_id;
      if (q_valid && q_ready &&  q_we) begin
         w_ids[w_free] <= q_id;
         wlo[w_free] <= {~q_wmask[31:0],  q_wdata[255:0]};
         whi[w_free] <= {~q_wmask[63:32], q_wdata[511:256]};
      end
      if (rd_mine) rev[{rd_j, rdDataAddr[0]}] <= rdData[127:0];
      if (rd_mine) rod[{rd_j, rdDataAddr[0]}] <= rdData[255:128];
      w_last_q <= wr_mine && wrDataAddr[0];  w_last_j <= wr_j;
      // the stream's output register
      if (s_load) begin
         r_valid <= 1'b1;  r_id <= r_ids[s_j];  r_beat <= s_beat;  r_last <= (s_beat == 2'd3);
         r_data  <= s_beat[0] ? s_od : s_ev;
      end else if (r_ready) r_valid <= 1'b0;

      if (q_valid && q_ready) begin
         rq_we <= q_we;  rq_line <= dram_byte[31:6];  rq_buf <= q_we ? {2'b10, w_free} : {1'b0, r_free};
      end
      if (reset) begin
         rq_valid <= 1'b0;
         r_busy <= {NR{1'b0}};  w_busy <= {NW{1'b0}};  w_done <= {NW{1'b0}};
         sq_n <= {(RB+1){1'b0}};  s_beat <= 2'd0;  r_valid <= 1'b0;  w_last_q <= 1'b0;
      end else begin
         if (q_valid && q_ready) rq_valid <= 1'b1; else if (rq_go) rq_valid <= 1'b0;
         if (q_valid && q_ready && !q_we) begin r_busy[r_free] <= 1'b1; r_got[r_free] <= 2'b00; end
         if (q_valid && q_ready &&  q_we) w_busy[w_free] <= 1'b1;
         if (rd_mine) r_got[rd_j][rdDataAddr[0]] <= 1'b1;
         // the stream queue: a line joins when its second half lands, leaves after beat 3
         if (s_end) begin
            for (j = 0; j < NR - 1; j = j + 1) sq[j] <= sq[j+1];
            r_busy[s_j] <= 1'b0;
         end
         if (rd_mine && rdDataAddr[0]) sq[sq_n[RB-1:0] - {{(RB-1){1'b0}}, s_end}] <= rd_j;
         sq_n <= sq_n + {{RB{1'b0}}, rd_mine && rdDataAddr[0]} - {{RB{1'b0}}, s_end};
         if (s_load) s_beat <= s_beat + 2'd1;
         // a write's done is owed once its second half has gone; the buffer frees with the done
         if (w_last_q) w_done[w_last_j] <= 1'b1;
         if (w_ret) begin w_done[w_dj] <= 1'b0; w_busy[w_dj] <= 1'b0; end
      end
   end

`ifndef SYNTHESIS
   always @(posedge clk) if (!reset) begin
      if (rd_mine && (!r_busy[rd_j] || r_got[rd_j][rdDataAddr[0]]))
         $fatal(1, "ddr4_port_line: a read burst for tag %b, a buffer not waiting for it", rdDataAddr);
      if (rd_mine && rdDataAddr[0] && !r_got[rd_j][0])
         $fatal(1, "ddr4_port_line: buffer %0d's second burst before its first", rd_j);
      if (wr_mine && (!w_busy[wr_j] || w_done[wr_j]))
         $fatal(1, "ddr4_port_line: the PHY asked for write data of buffer %0d, which holds none", wr_j);
   end
`endif
endmodule

`default_nettype wire
