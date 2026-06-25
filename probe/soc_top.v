`default_nettype none

// Synthesizable SoC top: the sharded-OoO core (backend_top) + unified I$/D$ (cache.v)
// + l2_arbiter merging all memory traffic onto ONE line memory port + a behavioral
// line RAM. This lifts the proven tb_vl cache adapters (sticky-rvalid read port,
// write-through write port, fence.i drain+invalidate FSM) into a real module, and
// replaces the per-cache L2 responders with the arbiter so I$-fill, D$-fill/write and
// the 3 PTW ports share one memory -- the shape the real DRAM bridge plugs into.
//
// Requester order into the arbiter (lower index = higher priority):
//   0 D$ (fill + write-through)   1 I$ (fill)   2 iPTW   3 ldPTW   4 stPTW
//
// SCOPE: RAM only (no MMIO devices yet -- CLINT/UART routing is the next increment;
// all dmem currently routes to the D$). RAM is byte-addressable internally (loadable
// via $readmemh from a TB) with a 64-byte line port for the arbiter.
module soc_top #(
   parameter IW=4, HW=8, PCW=64, SEQW=8, PBITS=8,
   parameter [63:0] BASE = 64'h8000_0000,
   parameter        RAM_LG2 = 21,          // 2 MiB
   parameter        SIZE_KB = 128          // each cache
) (
   input  wire             clk,
   input  wire             reset,
   // observation for a TB (commit + the store stream, to watch tohost)
   output wire             commit,
   output wire             dmem_wen,
   output wire [63:0]      dmem_waddr,
   output wire [63:0]      dmem_wdata,
   output wire [7:0]       dmem_wmask
);
   localparam SIZE = 1<<RAM_LG2;
   localparam AW   = 64;
   localparam LAW  = AW-6;                  // line address width = 58

   // ---------------- core <-> caches nets ----------------
   wire [PCW-1:0]      imem_addr;
   wire [HW*16-1:0]    imem_data;
   wire [3:0]          imem_avail;
   wire [63:0]         dmem_raddr;
   wire                dmem_ren;
   wire [63:0]         dmem_rdata;
   wire                dmem_rvalid, dmem_wready, dmem_idle, ifence;
   wire [55:0]         ptw_addr, ldptw_addr, stptw_addr;
   wire                ptw_read, ldptw_read, stptw_read;
   wire [63:0]         ptw_rdata, ldptw_rdata, stptw_rdata;
   wire                ptw_rvalid, ldptw_rvalid, stptw_rvalid;
   wire [IW-1:0]       wb_valid;  wire [IW*PBITS-1:0] wb_pr;  wire [IW*64-1:0] wb_val;
   wire                redirect;  wire [PCW-1:0] redirect_target;

   backend_top #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW), .PBITS(PBITS), .RESET_PC(BASE)) core
     (.clk(clk), .reset(reset),
      .imem_addr(imem_addr), .imem_data(imem_data), .imem_avail(imem_avail), .hw_ip(12'd0),
      .dmem_raddr(dmem_raddr), .dmem_ren(dmem_ren), .dmem_rdata(dmem_rdata), .dmem_rvalid(dmem_rvalid),
      .dmem_wen(dmem_wen), .dmem_waddr(dmem_waddr), .dmem_wdata(dmem_wdata), .dmem_wmask(dmem_wmask),
      .dmem_wready(dmem_wready), .dmem_idle(dmem_idle), .ifence(ifence),
      .ptw_addr(ptw_addr), .ptw_read(ptw_read), .ptw_rdata(ptw_rdata), .ptw_rvalid(ptw_rvalid),
      .ldptw_addr(ldptw_addr), .ldptw_read(ldptw_read), .ldptw_rdata(ldptw_rdata), .ldptw_rvalid(ldptw_rvalid),
      .stptw_addr(stptw_addr), .stptw_read(stptw_read), .stptw_rdata(stptw_rdata), .stptw_rvalid(stptw_rvalid),
      .wb_valid(wb_valid), .wb_pr(wb_pr), .wb_val(wb_val),
      .redirect(redirect), .redirect_target(redirect_target), .commit(commit), .commit_idx());

   // ---------------- D$ (write-through) + read/write adapters (proven in tb_vl) ----------------
   reg          c_rd_pend;
   wire [63:0]  dc_rd_data;  wire dc_rd_valid, dc_wr_ack;
   wire         dc_l2_req, dc_l2_we;  wire [LAW-1:0] dc_l2_addr;  wire [511:0] dc_l2_wdata;
   wire [511:0] dc_l2_rdata;  wire dc_l2_ack;
   wire         c_rd_req = (dmem_ren | c_rd_pend) & ~dc_rd_valid;
   always @(posedge clk) if (reset) c_rd_pend<=1'b0;
      else if (dmem_ren) c_rd_pend<=1'b1; else if (dc_rd_valid) c_rd_pend<=1'b0;
   reg          c_rdv_st;  reg [63:0] c_rdd_st;
   always @(posedge clk) if (reset) c_rdv_st<=1'b0;
      else if (dmem_ren) c_rdv_st<=1'b0;
      else if (dc_rd_valid) begin c_rdv_st<=1'b1; c_rdd_st<=dc_rd_data; end
   wire         c_st_ok = c_rdv_st & ~c_rd_pend & ~dmem_ren;
   assign       dmem_rdata  = c_st_ok ? c_rdd_st : dc_rd_data;
   assign       dmem_rvalid = dc_rd_valid | c_st_ok;
   assign       dmem_wready = dc_wr_ack;

   cache #(.PAW(64), .SIZE_KB(SIZE_KB), .RDW(64), .WDW(64), .WRITABLE(1), .WRTHRU(1)) u_dcache
     (.clk(clk), .reset(reset),
      .rd_req(c_rd_req), .rd_addr(dmem_raddr), .rd_data(dc_rd_data), .rd_valid(dc_rd_valid),
      .wr_req(dmem_wen & ~dc_wr_ack), .wr_addr(dmem_waddr), .wr_data(dmem_wdata),
      .wr_mask(dmem_wmask), .wr_ack(dc_wr_ack), .inv_req(1'b0), .inv_busy(),
      .l2_req(dc_l2_req), .l2_we(dc_l2_we), .l2_addr(dc_l2_addr), .l2_wdata(dc_l2_wdata),
      .l2_rdata(dc_l2_rdata), .l2_ack(dc_l2_ack));

   // ---------------- I$ (read-only) + fetch adapter + fence.i FSM (proven in tb_vl) ----------------
   reg          i_have, i_rd_pend;  reg [63:0] i_pa, i_reqpa;  reg [HW*16-1:0] i_win;
   wire         i_match = i_have & (i_pa == imem_addr);
   wire         i_need  = ~i_match;
   wire [HW*16-1:0] ic_rd_data;  wire ic_rd_valid, ic_inv_busy;
   wire         ic_rd_req  = (i_need | i_rd_pend) & ~ic_rd_valid;
   wire [63:0]  ic_rd_addr = i_rd_pend ? i_reqpa : imem_addr;
   wire         ic_l2_req, ic_l2_we;  wire [LAW-1:0] ic_l2_addr;  wire [511:0] ic_l2_wdata;
   wire [511:0] ic_l2_rdata;  wire ic_l2_ack;
   reg          ic_inv_req;
   always @(posedge clk) if (reset) begin i_have<=1'b0; i_rd_pend<=1'b0; end
      else begin
         if (ic_inv_req) i_have<=1'b0;
         if (~i_rd_pend & i_need) begin i_rd_pend<=1'b1; i_reqpa<=imem_addr; end
         if (ic_rd_valid) begin i_rd_pend<=1'b0; i_have<=1'b1; i_pa<=i_reqpa; i_win<=ic_rd_data; end
      end
   localparam FI_IDLE=0, FI_DRAIN=1, FI_INV=2, FI_WAIT=3;
   reg [1:0] fi;  wire fi_stall = (fi != FI_IDLE);
   always @(posedge clk) if (reset) begin fi<=FI_IDLE; ic_inv_req<=1'b0; end
      else begin
         ic_inv_req <= 1'b0;
         case (fi)
           FI_IDLE:  if (ifence) fi<=FI_DRAIN;
           FI_DRAIN: if (dmem_idle) begin ic_inv_req<=1'b1; fi<=FI_INV; end
           FI_INV:   fi<=FI_WAIT;
           FI_WAIT:  if (!ic_inv_busy) fi<=FI_IDLE;
         endcase
      end
   assign imem_data  = i_win;
   assign imem_avail = fi_stall ? 4'd0 : (i_match ? 4'd8 : 4'd0);

   cache #(.PAW(64), .SIZE_KB(SIZE_KB), .RDW(HW*16), .WDW(64), .WRITABLE(0)) u_icache
     (.clk(clk), .reset(reset),
      .rd_req(ic_rd_req), .rd_addr(ic_rd_addr), .rd_data(ic_rd_data), .rd_valid(ic_rd_valid),
      .wr_req(1'b0), .wr_addr(64'd0), .wr_data(64'd0), .wr_mask(8'd0), .wr_ack(),
      .inv_req(ic_inv_req), .inv_busy(ic_inv_busy),
      .l2_req(ic_l2_req), .l2_we(ic_l2_we), .l2_addr(ic_l2_addr), .l2_wdata(ic_l2_wdata),
      .l2_rdata(ic_l2_rdata), .l2_ack(ic_l2_ack));

   // ---------------- PTW line adapters (word read of a PTE via a line read) ----------------
   // i=2 iPTW, 3 ldPTW, 4 stPTW. Each: on *_read (held until *_rvalid), request the
   // containing line once, on ack extract the 8-byte word and pulse *_rvalid.
   wire [2:0]   pw_read   = {stptw_read, ldptw_read, ptw_read};
   wire [3*56-1:0] pw_addr = {stptw_addr, ldptw_addr, ptw_addr};
   reg  [2:0]   pw_busy;
   reg  [55:0]  pw_a [0:2];
   wire [2:0]   pw_req;
   wire [2:0]   pw_ack;       // from arbiter
   reg  [2:0]   pw_rvalid;
   reg  [63:0]  pw_rdata [0:2];
   wire [511:0] arb_rdata;
   genvar g;
   generate for (g=0; g<3; g=g+1) begin : ptw_adapt
      assign pw_req[g] = pw_read[g] & ~pw_busy[g];
      always @(posedge clk) if (reset) begin pw_busy[g]<=1'b0; pw_rvalid[g]<=1'b0; end
         else begin
            pw_rvalid[g] <= 1'b0;
            if (pw_req[g]) begin pw_busy[g]<=1'b1; pw_a[g]<=pw_addr[g*56 +: 56]; end
            if (pw_busy[g] & pw_ack[g]) begin
               pw_busy[g]  <= 1'b0;
               pw_rvalid[g]<= 1'b1;
               pw_rdata[g] <= arb_rdata[ {pw_a[g][5:3],6'd0} +: 64 ];   // word within the line
            end
         end
   end endgenerate
   assign ptw_rdata=pw_rdata[0];   assign ptw_rvalid=pw_rvalid[0];
   assign ldptw_rdata=pw_rdata[1]; assign ldptw_rvalid=pw_rvalid[1];
   assign stptw_rdata=pw_rdata[2]; assign stptw_rvalid=pw_rvalid[2];

   // ---------------- l2_arbiter (5 requesters) ----------------
   localparam NREQ=5;
   wire [NREQ-1:0]     a_req   = {pw_req[2], pw_req[1], pw_req[0], ic_l2_req, dc_l2_req};
   wire [NREQ-1:0]     a_we    = {1'b0,1'b0,1'b0, 1'b0, dc_l2_we};
   // PTW byte addr (56b) -> physical line addr PA[63:6] = {8'd0, ptw_addr[55:6]} (LAW=58b)
   wire [NREQ*LAW-1:0] a_addr  = {{8'd0,pw_a[2][55:6]}, {8'd0,pw_a[1][55:6]}, {8'd0,pw_a[0][55:6]},
                                  ic_l2_addr, dc_l2_addr};
   wire [NREQ*512-1:0] a_wdata = {512'd0,512'd0,512'd0, ic_l2_wdata, dc_l2_wdata};
   wire [NREQ-1:0]     a_ack;
   wire                m_req, m_we;  wire [LAW-1:0] m_addr;  wire [511:0] m_wdata, m_rdata;  wire m_ack;
   l2_arbiter #(.NREQ(NREQ), .AW(LAW), .DW(512)) u_arb
     (.clk(clk), .reset(reset),
      .req(a_req), .we(a_we), .addr(a_addr), .wdata(a_wdata), .ack(a_ack), .rdata(arb_rdata),
      .mem_req(m_req), .mem_we(m_we), .mem_addr(m_addr), .mem_wdata(m_wdata),
      .mem_rdata(m_rdata), .mem_ack(m_ack));
   assign dc_l2_ack = a_ack[0];  assign dc_l2_rdata = arb_rdata;
   assign ic_l2_ack = a_ack[1];  assign ic_l2_rdata = arb_rdata;
   assign pw_ack    = a_ack[4:2];

   // ---------------- behavioral line RAM (byte array; $readmemh-loadable) ----------------
   reg [7:0] ram [0:SIZE-1];
   reg m_busy; reg [3:0] m_cnt; reg m_we_q; reg [LAW-1:0] m_ad_q; reg [511:0] m_wd_q;
   reg [511:0] m_rdata_r; reg m_ack_r;
   integer kb; reg [63:0] m_base;
   always @(posedge clk) begin
      m_ack_r <= 1'b0;
      if (reset) m_busy<=1'b0;
      else if (!m_busy && m_req) begin m_busy<=1'b1; m_cnt<=4'd2; m_we_q<=m_we; m_ad_q<=m_addr; m_wd_q<=m_wdata; end
      else if (m_busy) begin
         if (m_cnt==0) begin
            m_base = ({{6{1'b0}},m_ad_q} << 6) - BASE;
            if (m_we_q) for (kb=0;kb<64;kb=kb+1) ram[m_base+kb] <= m_wd_q[kb*8 +: 8];
            else        for (kb=0;kb<64;kb=kb+1) m_rdata_r[kb*8 +: 8] <= ram[m_base+kb];
            m_ack_r<=1'b1; m_busy<=1'b0;
         end else m_cnt <= m_cnt-1;
      end
   end
   assign m_rdata = m_rdata_r;
   assign m_ack   = m_ack_r;
endmodule

`default_nettype wire
