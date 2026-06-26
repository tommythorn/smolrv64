`timescale 1ns/1ps
`default_nettype none

// Parallel-regression harness (verilated) for the sharded-OoO probe backend, a
// companion to iverilog's tb_riscv.v -- this one carries NO hierarchical/trace refs so it
// verilates cleanly, and is built once into a single binary that the parallel
// runner (run-vl-tests.sh) invokes per test with +hex/+tohost/+cycles).
//
// Loads a flat little-endian image (one byte/line hex, via +hex=<file>) based at
// 0x80000000, serves the backend's combinational imem window AND the LSU dmem port
// from that one memory, and watches for the riscv-test exit (a store to `tohost`):
// tohost==1 => PASS, else FAIL with test# = tohost>>1. Times out after +cycles.
module tb;
   localparam IW=4, HW=8, PCW=64, SEQW=8, PBITS=8;
   localparam [63:0] BASE = 64'h8000_0000;
   localparam        SIZE = 1<<21;             // 2 MiB (covers the -v demand-paging pool)

   reg                 clk=0; always #5 clk=~clk;
   reg                 reset;

   wire [PCW-1:0]      imem_addr;
   reg  [HW*16-1:0]    imem_data;
   wire                dmem_idle, ifence;
   // fence.i ordering: stall fetch while a fence.i flush is in progress.
   wire [3:0]          imem_avail = !icache_en ? 4'd8 : (fi_stall ? 4'd0 : (i_match ? 4'd8 : 4'd0));
   wire [63:0]         dmem_raddr;
   wire                dmem_ren;
   reg  [63:0]         dmem_rdata;
   wire                dmem_wen;
   wire [63:0]         dmem_waddr, dmem_wdata;
   wire [7:0]          dmem_wmask;
   wire [55:0]         ptw_addr, ldptw_addr, stptw_addr;
   wire                ptw_read, ldptw_read, stptw_read;
   reg  [63:0]         ptw_rdata, ldptw_rdata, stptw_rdata;
   reg                 ptw_rvalid, ldptw_rvalid, stptw_rvalid;
   wire [IW-1:0]       wb_valid;
   wire [IW*PBITS-1:0] wb_pr;
   wire [IW*64-1:0]    wb_val;
   wire                redirect, commit;
   wire [PCW-1:0]      redirect_target;

   backend_top #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW), .PBITS(PBITS),
                 .RESET_PC(BASE)) dut
     (.clk(clk), .reset(reset),
      .imem_addr(imem_addr), .imem_data(imem_data), .imem_avail(imem_avail),
      .hw_ip(12'd0), .mtime(64'd0),        // no CLINT/PLIC in this device-less harness
      .dmem_raddr(dmem_raddr), .dmem_ren(dmem_ren),
      .dmem_rdata(dmem_rdata), .dmem_rvalid(dmem_rvalid),
      .dmem_wen(dmem_wen), .dmem_waddr(dmem_waddr), .dmem_wdata(dmem_wdata),
      .dmem_wmask(dmem_wmask), .dmem_wready(dmem_wready),
      .dmem_idle(dmem_idle), .ifence(ifence),
      .ptw_addr(ptw_addr), .ptw_read(ptw_read),
      .ptw_rdata(ptw_rdata), .ptw_rvalid(ptw_rvalid),
      .ldptw_addr(ldptw_addr), .ldptw_read(ldptw_read),
      .ldptw_rdata(ldptw_rdata), .ldptw_rvalid(ldptw_rvalid),
      .stptw_addr(stptw_addr), .stptw_read(stptw_read),
      .stptw_rdata(stptw_rdata), .stptw_rvalid(stptw_rvalid),
      .wb_valid(wb_valid), .wb_pr(wb_pr), .wb_val(wb_val),
      .redirect(redirect), .redirect_target(redirect_target),
      .commit(commit), .commit_idx());

   reg [7:0] mem [0:SIZE-1];
   function [63:0] rd64(input [63:0] addr);
      reg [63:0] a; integer b;
      begin a = addr - BASE;
         rd64 = 64'd0;
         for (b=0;b<8;b=b+1) rd64[b*8 +: 8] = mem[a+b];
      end
   endfunction

   reg wtick=0;
   integer m;
   reg [HW*16-1:0] mem_win;
   always @(imem_addr or wtick) begin
      for (m=0;m<HW;m=m+1) begin
         mem_win[m*16 +: 8]   = mem[(imem_addr-BASE)+2*m];
         mem_win[m*16+8 +: 8] = mem[(imem_addr-BASE)+2*m+1];
      end
   end
   always @* imem_data = icache_en ? i_win : mem_win;

   // ---- optional real I$ (cache.v, read-only) on the fetch path: imem_addr (PA) -> I$ ----
   // The frontend holds imem_addr while imem_avail=0, so a miss just stalls fetch; on an I$
   // hit we present the 16-byte window with avail=8. i_pa tracks the window we hold; when the
   // frontend advances (imem_addr != i_pa) we re-request. (fence.i I$-coherence is a step-3
   // follow-up: it needs the core to invalidate the I$ + order behind the D$ drain.)
   wire         icache_en = cache_en;
   reg          i_have, i_rd_pend;  reg [63:0] i_pa, i_reqpa;  reg [HW*16-1:0] i_win;
   wire         i_match  = icache_en & i_have & (i_pa == imem_addr);
   wire         i_need   = icache_en & ~i_match;
   wire [HW*16-1:0] ic_rd_data;  wire ic_rd_valid, ic_inv_busy;
   wire         ic_rd_req  = (i_need | i_rd_pend) & ~ic_rd_valid;
   wire [63:0]  ic_rd_addr = i_rd_pend ? i_reqpa : imem_addr;
   wire         ic_l2_req, ic_l2_we;  wire [57:0] ic_l2_addr;  wire [511:0] ic_l2_wdata;
   reg  [511:0] ic_l2_rdata;  reg ic_l2_ack;
   always @(posedge clk) if (reset) begin i_have<=1'b0; i_rd_pend<=1'b0; end
      else begin
         if (ic_inv_req) i_have<=1'b0;   // fence.i flush -> drop the held window, force a refill
         if (~i_rd_pend & i_need) begin i_rd_pend<=1'b1; i_reqpa<=imem_addr; end
         if (ic_rd_valid) begin i_rd_pend<=1'b0; i_have<=1'b1; i_pa<=i_reqpa; i_win<=ic_rd_data; end
      end
   // fence.i ordering FSM: on a fence.i redirect, stall fetch until the D$ has drained to memory
   // (dmem_idle), then invalidate the I$, then resume -- so the refetch sees the modified code.
   localparam FI_IDLE=0, FI_DRAIN=1, FI_INV=2, FI_WAIT=3;
   reg [1:0] fi;  reg ic_inv_req;
   wire      fi_stall = icache_en & (fi != FI_IDLE);
   always @(posedge clk) if (reset) begin fi<=FI_IDLE; ic_inv_req<=1'b0; end
      else begin
         ic_inv_req <= 1'b0;
         case (fi)
           FI_IDLE:  if (icache_en & ifence) fi<=FI_DRAIN;
           FI_DRAIN: if (dmem_idle) begin ic_inv_req<=1'b1; fi<=FI_INV; end
           FI_INV:   fi<=FI_WAIT;
           FI_WAIT:  if (!ic_inv_busy) fi<=FI_IDLE;
         endcase
      end
   cache #(.PAW(64), .SIZE_KB(128), .RDW(HW*16), .WDW(64), .WRITABLE(0)) u_icache
     (.clk(clk), .reset(reset),
      .rd_req(ic_rd_req), .rd_addr(ic_rd_addr), .rd_data(ic_rd_data), .rd_valid(ic_rd_valid),
      .wr_req(1'b0), .wr_addr(64'd0), .wr_data(64'd0), .wr_mask(8'd0), .wr_ack(),
      .inv_req(ic_inv_req), .inv_busy(ic_inv_busy),
      .l2_req(ic_l2_req), .l2_we(ic_l2_we), .l2_addr(ic_l2_addr), .l2_wdata(ic_l2_wdata),
      .l2_rdata(ic_l2_rdata), .l2_ack(ic_l2_ack));
   // I$ L2 responder (read-only): line read of `mem` (based at BASE), 2-cycle latency
   reg ic_l2busy; reg [3:0] ic_l2cnt; reg [57:0] ic_l2ad_q; integer ik; reg [63:0] ic_l2base;
   always @(posedge clk) begin
      ic_l2_ack <= 1'b0;
      if (reset) ic_l2busy <= 1'b0;
      else if (!ic_l2busy && ic_l2_req) begin ic_l2busy<=1'b1; ic_l2cnt<=4'd2; ic_l2ad_q<=ic_l2_addr; end
      else if (ic_l2busy) begin
         if (ic_l2cnt==0) begin
            ic_l2base = ({{6{1'b0}},ic_l2ad_q} << 6) - BASE;
            for (ik=0;ik<64;ik=ik+1) ic_l2_rdata[ik*8 +: 8] <= mem[ic_l2base+ik];
            ic_l2_ack<=1'b1; ic_l2busy<=1'b0;
         end else ic_l2cnt <= ic_l2cnt-1;
      end
   end
   // ---- optional real D$ (cache.v, write-through) between LSU dmem port and `mem` (L2) ----
   // +cache=1 routes loads/stores through the unified PIPT cache; mem stays current via
   // write-through so the PTW/imem (which read mem directly) remain coherent.
   reg          cache_en;  initial cache_en = 1'b0;
   reg          c_rd_pend;                                  // hold rd_req from the ren pulse
   // hold rd_req from the dmem_ren pulse until rd_valid; mask OFF on the valid cycle so the
   // cache doesn't re-accept the same (still-held) address and emit a spurious 2nd rd_valid.
   wire         c_rd_req = cache_en & (dmem_ren | c_rd_pend) & ~c_rd_valid;
   wire [63:0]  c_rd_data;  wire c_rd_valid, c_wr_ack;
   wire         c_l2_req, c_l2_we;  wire [57:0] c_l2_addr;  // PAW=64 -> line addr [63:6]
   wire [511:0] c_l2_wdata;  reg [511:0] c_l2_rdata;  reg c_l2_ack;
   always @(posedge clk) if (reset) c_rd_pend<=1'b0;
      else if (dmem_ren) c_rd_pend<=1'b1; else if (c_rd_valid) c_rd_pend<=1'b0;
   // STICKY rd result: the LSU MERGE consumes mem_rvalid only when its WB lane is free, and
   // it expects rvalid/rdata to STAY valid until then (the +memlat model holds it high until
   // the next ren). The cache pulses rd_valid for one cycle, so latch it and hold until the
   // next dmem_ren clears it -- otherwise a load whose WB lane is busy that cycle wedges.
   reg          c_rdv_st;  reg [63:0] c_rdd_st;
   always @(posedge clk) if (reset) c_rdv_st<=1'b0;
      else if (dmem_ren) c_rdv_st<=1'b0;
      else if (c_rd_valid) begin c_rdv_st<=1'b1; c_rdd_st<=c_rd_data; end
   // the held result is valid only while NO newer read is issuing (dmem_ren) or in flight
   // (c_rd_pend) -- else the previous load's sticky would leak into the next load's MERGE.
   wire         c_st_ok = c_rdv_st & ~c_rd_pend & ~dmem_ren;

   cache #(.PAW(64), .SIZE_KB(128), .RDW(64), .WDW(64), .WRITABLE(1), .WRTHRU(1)) u_dcache
     (.clk(clk), .reset(reset),
      .rd_req(c_rd_req), .rd_addr(dmem_raddr), .rd_data(c_rd_data), .rd_valid(c_rd_valid),
      .wr_req(cache_en & dmem_wen & ~c_wr_ack), .wr_addr(dmem_waddr), .wr_data(dmem_wdata),
      .wr_mask(dmem_wmask), .wr_ack(c_wr_ack), .inv_req(1'b0), .inv_busy(),
      .l2_req(c_l2_req), .l2_we(c_l2_we), .l2_addr(c_l2_addr), .l2_wdata(c_l2_wdata),
      .l2_rdata(c_l2_rdata), .l2_ack(c_l2_ack));

   // cache L2 responder: line read/write of `mem` (based at BASE), 2-cycle latency
   reg c_l2busy; reg [3:0] c_l2cnt; reg c_l2we_q; reg [57:0] c_l2ad_q; reg [511:0] c_l2wd_q;
   integer kk; reg [63:0] c_l2base;
   always @(posedge clk) begin
      c_l2_ack <= 1'b0;
      if (reset) c_l2busy <= 1'b0;
      else if (!c_l2busy && c_l2_req) begin
         c_l2busy<=1'b1; c_l2cnt<=4'd2; c_l2we_q<=c_l2_we; c_l2ad_q<=c_l2_addr; c_l2wd_q<=c_l2_wdata;
      end else if (c_l2busy) begin
         if (c_l2cnt==0) begin
            c_l2base = ({{6{1'b0}},c_l2ad_q} << 6) - BASE;
            if (c_l2we_q) for (kk=0;kk<64;kk=kk+1) mem[c_l2base+kk] <= c_l2wd_q[kk*8 +: 8];
            else          for (kk=0;kk<64;kk=kk+1) c_l2_rdata[kk*8 +: 8] <= mem[c_l2base+kk];
            c_l2_ack<=1'b1; c_l2busy<=1'b0;
         end else c_l2cnt <= c_l2cnt-1;
      end
   end

   always @(dmem_raddr or wtick or cache_en or c_rd_data or c_rdv_st or c_rdd_st)
      dmem_rdata = cache_en ? (c_st_ok ? c_rdd_st : c_rd_data) : rd64(dmem_raddr);

   // ---- variable load-read latency (proves the LSU miss-stall path) ----
   // +memlat=0 (default): dmem_rvalid==1 always -> combinational memory, bit-exact 1-cycle
   // loads. +memlat=N (N>=1): each load read returns N cycles after its dmem_ren pulse.
   // dmem_rdata stays combinational on the (held) dmem_raddr, so it reflects the address
   // live at the cycle rvalid asserts -- the single-outstanding contract the LSU relies on.
   reg  [15:0] memlat;
   reg         lat_busy; reg [15:0] lat_cnt;
   initial begin memlat = 16'd0; lat_busy = 1'b0; lat_cnt = 16'd0; end
   wire dmem_rvalid = cache_en ? (c_rd_valid | c_st_ok)
                    : (memlat == 16'd0) ? 1'b1
                    : (dmem_ren ? 1'b0 : (lat_busy && lat_cnt == 16'd0));
   wire dmem_wready = cache_en ? c_wr_ack : 1'b1;
   always @(posedge clk) begin
      if (reset)            begin lat_busy <= 1'b0; lat_cnt <= 16'd0; end
      else if (dmem_ren)    begin lat_busy <= 1'b1; lat_cnt <= (memlat==16'd0)?16'd0:(memlat-16'd1); end
      else if (lat_busy && lat_cnt != 16'd0) lat_cnt <= lat_cnt - 16'd1;
   end

   always @(posedge clk) begin
      ptw_rvalid <= ptw_read;
      if (ptw_read) ptw_rdata <= rd64({8'd0, ptw_addr});
      ldptw_rvalid <= ldptw_read;
      if (ldptw_read) ldptw_rdata <= rd64({8'd0, ldptw_addr});
      stptw_rvalid <= stptw_read;
      if (stptw_read) stptw_rdata <= rd64({8'd0, stptw_addr});
   end

   reg [63:0] tohost; integer c, b2;
   integer    ncyc;
   reg [8*256-1:0] hexfile;
   initial begin
      tohost = 64'h8000_1000;
      ncyc   = 200000;
      ptw_rvalid = 1'b0; ldptw_rvalid = 1'b0; stptw_rvalid = 1'b0;
      if (!$value$plusargs("hex=%s", hexfile)) begin
         $display("FATAL: need +hex=<file>"); $finish;
      end
      for (m=0; m<SIZE; m=m+1) mem[m] = 8'd0;
      $readmemh(hexfile, mem);
      if ($value$plusargs("tohost=%h", tohost)) ;
      if ($value$plusargs("cycles=%d", ncyc)) ;
      if ($value$plusargs("memlat=%d", memlat)) ;
      begin : cache_arg integer ce; if ($value$plusargs("cache=%d", ce)) cache_en = (ce!=0); end

      reset=1; @(negedge clk); @(negedge clk); reset=0;

      for (c=0; c<ncyc; c=c+1) begin
         @(negedge clk);
         if (dmem_wen && (dmem_waddr - (dmem_waddr%8)) == tohost && dmem_wmask[0]) begin
            if (dmem_wdata[31:0] == 32'd1)
               $display("RISCV-TEST PASS");
            else
               $display("RISCV-TEST FAIL test=%0d (tohost=%h)", dmem_wdata[31:1], dmem_wdata);
            $finish;
         end
      end
      $display("RISCV-TEST TIMEOUT after %0d cycles (pc~%h)", ncyc, imem_addr);
      $finish;
   end

   // apply stores to memory (direct path only; with the cache, write-through updates `mem`)
   always @(posedge clk) if (!reset && !cache_en && dmem_wen) begin
      for (b2=0;b2<8;b2=b2+1)
         if (dmem_wmask[b2]) mem[(dmem_waddr-BASE)+b2] <= dmem_wdata[b2*8 +: 8];
      wtick <= ~wtick;
   end
   // with the cache, the L2 write-through updates `mem`; pulse wtick so imem refetch sees it
   always @(posedge clk) if (!reset && cache_en && c_l2busy && c_l2cnt==0 && c_l2we_q) wtick <= ~wtick;
endmodule

`default_nettype wire
