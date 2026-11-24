`default_nettype none
// ddr4_port_axi -- the DMA masters' AXI4 port (on ui_clk, after the device arbiters) as a requester
// of ddr4_ctl, coherent with the D$ (docs/PLAN-2026-10-21-memory-system.md).
//
// Every transaction the masters make lies within one 64-byte line: dma_master's one 8-byte beat or
// eight-beat line (virtio-blk, virtio-net, virtio-input) and vga_scanout's eight-beat line. So a
// transaction is one line request, and anything else asserts. The read engine keeps up to NR = 3
// reads in flight, each in a slot with a line buffer of its own (the controller's buffers 12, 13
// and 15), in the order their ARs came: the oldest unsnooped slot has the snoop, the oldest snooped
// one goes to the controller, a line lands in its slot as the PHY names the buffer, and the oldest
// slot whose line is in sends its beats on R, in order, through a beat register. So a line's snoop
// and the lines before it in DRAM overlap, where one read at a time paid them one after another
// (a 1920x1200 scan needs a line every 267 ns). The write engine takes one transaction at a time
// (buffer 14): its beats gather with their strobes, the line goes as one masked write, and B goes
// back once the PHY has taken both halves.
//
// COHERENCE. Before its line goes to the controller each engine has the D$ snoop it (sn_*, through
// ddr_port_sync's snoop channel, the engines taking turns): a read has it cleaned, so memory holds
// any bytes the D$ held dirty; a write has it flushed, so the D$ keeps no copy. The answer means
// memory has the D$'s write-back, and the controller takes requests in order, so the line request
// that follows sees it. After a flush the D$ starts no fill of the line until the release (sn_rel,
// the cycle the controller takes the write), so a fill that follows reads the DMA's bytes.
module ddr4_port_axi (
   input  wire          clk,
   input  wire          reset,
   // ---- AXI4 slave, 64-bit data, 32-bit address in the DRAM window ----
   input  wire [2:0]    s_axi_awid,
   input  wire [31:0]   s_axi_awaddr,
   input  wire [7:0]    s_axi_awlen,
   input  wire [2:0]    s_axi_awsize,
   input  wire [1:0]    s_axi_awburst,
   input  wire          s_axi_awvalid,
   output wire          s_axi_awready,
   input  wire [63:0]   s_axi_wdata,
   input  wire [7:0]    s_axi_wstrb,
   input  wire          s_axi_wlast,
   input  wire          s_axi_wvalid,
   output wire          s_axi_wready,
   output reg  [2:0]    s_axi_bid,
   output wire [1:0]    s_axi_bresp,
   output wire          s_axi_bvalid,
   input  wire          s_axi_bready,
   input  wire [2:0]    s_axi_arid,
   input  wire [31:0]   s_axi_araddr,
   input  wire [7:0]    s_axi_arlen,
   input  wire [2:0]    s_axi_arsize,
   input  wire [1:0]    s_axi_arburst,
   input  wire          s_axi_arvalid,
   output wire          s_axi_arready,
   output reg  [2:0]    s_axi_rid,
   output wire [63:0]   s_axi_rdata,
   output wire [1:0]    s_axi_rresp,
   output wire          s_axi_rlast,
   output wire          s_axi_rvalid,
   input  wire          s_axi_rready,
   // ---- the snoop channel (ddr_port_sync) and the held line ----
   output wire          sn_valid,               // held until sn_ack
   output wire [25:0]   sn_line,
   output wire          sn_keep,                // clean (a read's); else flush (a write's)
   input  wire          sn_ack,
   output wire          sn_rel,
   // ---- ddr4_ctl's request port ----
   output wire          rq_valid,
   input  wire          rq_ready,
   output wire          rq_we,
   output wire [25:0]   rq_line,
   output wire [3:0]    rq_buf,
   // ---- the PHY's data ----
   input  wire          rdDataEn,
   input  wire [4:0]    rdDataAddr,
   input  wire [255:0]  rdData,
   input  wire          wrDataEn,
   input  wire [4:0]    wrDataAddr,
   output wire          wd_mine,                // the PHY asks this port for write data: wd_* are the half it names
   output wire [255:0]  wd_data,
   output wire [31:0]   wd_mask                 // 1: the byte is not written
);
   localparam [3:0] WBUF = 4'd14;
   assign s_axi_bresp = 2'b00;
   assign s_axi_rresp = 2'b00;

   // ---- the read engine: NR slots in a ring, in AR order ----
   localparam integer NR = 3;
   function [1:0] nx(input [1:0] p); nx = (p == NR - 1) ? 2'd0 : p + 2'd1; endfunction
   function [3:0] rbuf(input [1:0] p); rbuf = (p == 2'd0) ? 4'd12 : (p == 2'd1) ? 4'd13 : 4'd15; endfunction
   reg  [NR-1:0] s_v, s_sn, s_rq;                // per slot: in use, snooped, asked of the controller
   reg  [1:0]    s_got [0:NR-1];                 // ...the line's halves in
   reg  [25:0]   s_line [0:NR-1];
   reg  [2:0]    s_k0 [0:NR-1], s_n [0:NR-1], s_id [0:NR-1];   // first beat, beats after it, id
   // the slots' lines: a memory per beat position k in a half, at {slot, half}, each written by one
   // statement, so each is distributed RAM
   wire [255:0]  sb_q;                           // the four beats at {rs_p, l_k[2]}
   reg  [1:0]    a_p, sn_p, rq_p, rs_p;          // the slot an AR takes, snoops, asks, answers next
   reg           rs_on;                          // rs_p's beats have begun: r_k, r_left are its
   reg  [2:0]    r_k, r_left;                    // the beat the register takes next, the beats after it
   reg           r_vq, r_lq;                     // the beat register, R's
   reg  [63:0]   r_dq;
   assign s_axi_arready = ~s_v[a_p];
   assign s_axi_rvalid  = r_vq;
   assign s_axi_rdata   = r_dq;
   assign s_axi_rlast   = r_lq;
   wire          r_take = r_vq & s_axi_rready;
   wire          r_load = (~r_vq | r_take) & s_v[rs_p] & (s_got[rs_p] == 2'b11);
   wire [2:0]    l_k    = rs_on ? r_k : s_k0[rs_p];
   wire [2:0]    l_left = rs_on ? r_left : s_n[rs_p];
   wire [25:0]   r_line = s_line[sn_p];          // the line the read engine snoops

   // ---- the write engine: idle, beats, snoop, into the request register, wait for the PHY to take it, B ----
   localparam [2:0] WI = 3'd0, WD = 3'd1, WS = 3'd2, WQ = 3'd3, WW = 3'd4, WB = 3'd5;
   reg  [2:0]   w_st;
   reg  [25:0]  w_line;
   reg  [2:0]   w_k;
   reg  [511:0] w_buf;
   reg  [63:0]  w_strb;                         // the bytes written
   assign s_axi_awready = (w_st == WI);
   assign s_axi_wready  = (w_st == WD);
   assign s_axi_bvalid  = (w_st == WB);

   // ---- the snoop: the engines take turns; an engine owns the channel from its first cycle to the
   // answer ----
   wire r_sn = s_v[sn_p] & ~s_sn[sn_p];
   wire w_sn = (w_st == WS);
   reg  sn_own, sn_w, sn_lastw;                 // the channel is owned, by the write engine, which had it last
   wire sn_pick = w_sn & (~r_sn | ~sn_lastw);
   wire sn_selw = sn_own ? sn_w : sn_pick;
   assign sn_valid = r_sn | w_sn;
   assign sn_line  = sn_selw ? w_line : r_line;
   assign sn_keep  = ~sn_selw;
   wire r_snack = sn_ack & ~sn_w;
   wire w_snack = sn_ack &  sn_w;

   // ---- requests: from a register, the read engine's first, so the controller's queue compares a
   // flop's line, not the slots' pick (a fill when it is empty or being taken) ----
   wire r_rq = s_v[rq_p] & s_sn[rq_p] & ~s_rq[rq_p];
   wire w_rq = (w_st == WQ);
   reg  q_v, q_we;
   reg  [25:0] q_line;
   reg  [3:0]  q_buf;
   assign rq_valid = q_v;
   assign rq_we    = q_we;
   assign rq_line  = q_line;
   assign rq_buf   = q_buf;
   wire rq_go  = q_v & rq_ready;
   wire q_fill = (~q_v | rq_go) & (r_rq | w_rq);
   assign sn_rel = rq_go & q_we;                // the controller has the flushed line's write

   wire       rd_mine = rdDataEn && (rdDataAddr[4:3] == 2'b11) && (rdDataAddr[2:1] != 2'b10);   // 12, 13, 15
   wire [1:0] rd_s    = (rdDataAddr[2:1] == 2'b11) ? 2'd2 : {1'b0, rdDataAddr[1]};          // its slot
   genvar gk;
   generate for (gk = 0; gk < 4; gk = gk + 1) begin : g_sb
      (* ram_style = "distributed" *) reg [63:0] mem [0:2*NR-1];
      always @(posedge clk) if (rd_mine) mem[{rd_s, rdDataAddr[0]}] <= rdData[gk*64 +: 64];
      assign sb_q[gk*64 +: 64] = mem[{rs_p, l_k[2]}];
   end endgenerate
   wire wr_mine = wrDataEn && (wrDataAddr[4:1] == WBUF);
   assign wd_mine = wr_mine;
   assign wd_data = w_buf[wrDataAddr[0]*256 +: 256];
   assign wd_mask = ~w_strb[wrDataAddr[0]*32 +: 32];
   reg  w_sent;                                 // the PHY asked for the second half last cycle
   integer b;

   always @(posedge clk) begin
      if (r_load) begin r_dq <= sb_q[l_k[1:0]*64 +: 64];  r_lq <= (l_left == 3'd0);  s_axi_rid <= s_id[rs_p]; end
      w_sent  <= wr_mine && wrDataAddr[0];
      if (reset) begin
         q_v <= 1'b0;  s_v <= {NR{1'b0}};  s_sn <= {NR{1'b0}};  s_rq <= {NR{1'b0}};  a_p <= 2'd0;  sn_p <= 2'd0;  rq_p <= 2'd0;  rs_p <= 2'd0;  rs_on <= 1'b0;  r_vq <= 1'b0;
         w_st <= WI;  w_sent <= 1'b0;  sn_own <= 1'b0;  sn_w <= 1'b0;  sn_lastw <= 1'b0;
      end else begin
         if (sn_ack) begin sn_own <= 1'b0;  sn_lastw <= sn_w; end
         else if (sn_valid & ~sn_own) begin sn_own <= 1'b1;  sn_w <= sn_pick; end
         // a read: a slot, its snoop, its request, its line's halves, its beats
         if (s_axi_arvalid & s_axi_arready) begin
            s_v[a_p] <= 1'b1;  s_sn[a_p] <= 1'b0;  s_rq[a_p] <= 1'b0;  s_got[a_p] <= 2'b00;
            s_line[a_p] <= s_axi_araddr[31:6];  s_k0[a_p] <= s_axi_araddr[5:3];
            s_n[a_p] <= s_axi_arlen[2:0];  s_id[a_p] <= s_axi_arid;
            a_p <= nx(a_p);
         end
         if (r_snack) begin s_sn[sn_p] <= 1'b1;  sn_p <= nx(sn_p); end
         if (q_fill) begin
            q_v <= 1'b1;  q_we <= ~r_rq;  q_line <= r_rq ? s_line[rq_p] : w_line;  q_buf <= r_rq ? rbuf(rq_p) : WBUF;
         end else if (rq_go) q_v <= 1'b0;
         if (q_fill & r_rq) begin s_rq[rq_p] <= 1'b1;  rq_p <= nx(rq_p); end
         if (rd_mine) s_got[rd_s][rdDataAddr[0]] <= 1'b1;
         if (r_load) begin
            r_vq <= 1'b1;
            if (l_left == 3'd0) begin
               s_v[rs_p] <= 1'b0;  s_sn[rs_p] <= 1'b0;  s_rq[rs_p] <= 1'b0;  rs_p <= nx(rs_p);  rs_on <= 1'b0;
            end
            else begin r_k <= l_k + 3'd1;  r_left <= l_left - 3'd1;  rs_on <= 1'b1; end
         end else if (r_take) r_vq <= 1'b0;
         case (w_st)
            WI: if (s_axi_awvalid) begin
                   w_st <= WD;  w_line <= s_axi_awaddr[31:6];  w_k <= s_axi_awaddr[5:3];
                   s_axi_bid <= s_axi_awid;  w_strb <= 64'd0;
                end
            WD: if (s_axi_wvalid) begin
                   w_buf[w_k*64 +: 64] <= s_axi_wdata;
                   w_strb[w_k*8 +: 8]  <= s_axi_wstrb;
                   w_k <= w_k + 3'd1;
                   if (s_axi_wlast) w_st <= WS;
                end
            WS: if (w_snack) w_st <= WQ;
            WQ: if (q_fill & ~r_rq) w_st <= WW;
            WW: if (w_sent) w_st <= WB;
            WB: if (s_axi_bready) w_st <= WI;
            default: w_st <= WI;
         endcase
      end
   end

   always @(posedge clk) if (!reset) begin
      if (s_axi_arvalid && s_axi_arready &&
          (s_axi_arsize != 3'd3 || s_axi_arburst != 2'b01 || {1'b0, s_axi_araddr[5:3]} + s_axi_arlen[3:0] > 4'd7 || s_axi_arlen[7:3] != 5'd0))
         $fatal(1, "ddr4_port_axi: a read (addr %h len %0d size %0d burst %0d) that is not within one line",
                s_axi_araddr, s_axi_arlen, s_axi_arsize, s_axi_arburst);
      if (s_axi_awvalid && s_axi_awready &&
          (s_axi_awsize != 3'd3 || s_axi_awburst != 2'b01 || {1'b0, s_axi_awaddr[5:3]} + s_axi_awlen[3:0] > 4'd7 || s_axi_awlen[7:3] != 5'd0))
         $fatal(1, "ddr4_port_axi: a write (addr %h len %0d size %0d burst %0d) that is not within one line",
                s_axi_awaddr, s_axi_awlen, s_axi_awsize, s_axi_awburst);
      if (rd_mine && !(s_v[rd_s] && s_rq[rd_s]) || rd_mine && s_got[rd_s][rdDataAddr[0]])
         $fatal(1, "ddr4_port_axi: read data in buffer %0d for no line asked of it", rdDataAddr[4:1]);
      if (rd_mine && rdDataAddr[0] && !s_got[rd_s][0]) $fatal(1, "ddr4_port_axi: the second burst before the first");
      if (wr_mine && w_st != WW)              $fatal(1, "ddr4_port_axi: the PHY asked for write data not queued");
      if (w_st > WB || a_p >= NR || sn_p >= NR || rq_p >= NR || rs_p >= NR)
         $fatal(1, "ddr4_port_axi: write state %0d, slot pointers %0d %0d %0d %0d", w_st, a_p, sn_p, rq_p, rs_p);
      if ((s_sn & ~s_v) != 0 || (s_rq & ~s_sn) != 0)
         $fatal(1, "ddr4_port_axi: a slot asked of the controller before its snoop (v %b sn %b rq %b)", s_v, s_sn, s_rq);
      if (sn_ack && !sn_own)                  $fatal(1, "ddr4_port_axi: a snoop answer with no snoop asked");
      if (sn_own && !(sn_w ? w_sn : r_sn) && !sn_ack) $fatal(1, "ddr4_port_axi: a snoop's engine moved on before its answer");
   end
endmodule

`default_nettype wire
