`timescale 1ns/1ps
`default_nettype none

// Width knobs (guarded; -DPROBE_IW/-DPROBE_POOL override). The regression tracks the
// core's default width -- HW/PBITS derive from IW/POOL just like backend_top/soc_top.
`ifndef PROBE_IW
 `define PROBE_IW 2
`endif
`ifndef PROBE_POOL
 `define PROBE_POOL 80
`endif

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
   localparam IW=`PROBE_IW, HW=2*IW, PCW=64, SEQW=8,
              PBITS=$clog2(`PROBE_POOL)+(($clog2(IW)<1)?1:$clog2(IW));   // SBITS>=1
   localparam [63:0] BASE = 64'h8000_0000;
   localparam        SIZE = 1<<21;             // 2 MiB (covers the -v demand-paging pool)

   reg                 clk=0; always #5 clk=~clk;
   reg                 reset;

   wire [PCW-1:0]      imem_addr;
   reg  [HW*16-1:0]    imem_data;
   wire                dmem_idle, ifence;
   // fence.i ordering: stall fetch while a fence.i flush is in progress.
   wire [$clog2(HW+2)-1:0] imem_avail = !icache_en ? HW : (fi_stall ? 0 : (i_match ? HW : 0));
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
      .dmem_rdy(dmem_rdy), .dmem_resp_addr(dmem_resp_addr),
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
   wire [HW*16-1:0] ic_rd_data;  wire ic_rd_valid, ic_inv_busy, ic_rd_rdy;
   // ready/valid request channel: present until the rd_req&rd_rdy handshake
   // (i_reqpa latched there), then wait for the response.
   wire         ic_rd_req  = i_need & ~i_rd_pend;
   wire [63:0]  ic_rd_addr = imem_addr;
   wire         ic_l2_req, ic_l2_we;  wire [57:0] ic_l2_addr;  wire [511:0] ic_l2_wdata;
   reg  [511:0] ic_l2_rdata;  reg ic_l2_ack;
   always @(posedge clk) if (reset) begin i_have<=1'b0; i_rd_pend<=1'b0; end
      else begin
         if (ic_inv_req) i_have<=1'b0;   // fence.i flush -> drop the held window, force a refill
         if (ic_rd_req & ic_rd_rdy) begin i_rd_pend<=1'b1; i_reqpa<=imem_addr; end
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
      .rd_req(ic_rd_req), .rd_rdy(ic_rd_rdy), .rd_addr(ic_rd_addr), .rd_data(ic_rd_data), .rd_valid(ic_rd_valid),
      .rd_uncached(1'b0),
      .wr_req(1'b0), .wr_addr(64'd0), .wr_data(64'd0), .wr_mask(8'd0), .wr_ack(), .wr_uncached(1'b0),
      .cbo_req(1'b0), .cbo_zero(1'b0), .cbo_keep(1'b0),
      .inv_req(ic_inv_req), .inv_clean(1'b0), .inv_busy(ic_inv_busy),
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
   // The LSU is multi-outstanding (2 reads) and matches responses by PA
   // (dmem_resp_addr) itself: this adapter only holds an UNGRANTED request in a
   // 1-deep issue register (av/aaddr) until the rd_req&rd_rdy handshake --
   // dmem_rdy backpressures the LSU while it is full -- and forwards the cache's
   // raw address-tagged responses (no pend/sticky machinery).
   reg          av;  reg [63:0] aaddr;
   wire         c_rd_req  = cache_en & (av | dmem_ren);
   wire [63:0]  c_rd_addr = av ? aaddr : dmem_raddr;
   wire [63:0]  c_rd_data;  wire c_rd_valid, c_wr_ack, c_rd_rdy;
   wire [63:0]  c_rd_resp_addr;
   wire         c_l2_req, c_l2_we;  wire [57:0] c_l2_addr;  // PAW=64 -> line addr [63:6]
   wire [511:0] c_l2_wdata;  wire [63:0] c_l2_wmask;  reg [511:0] c_l2_rdata;  reg c_l2_ack;
   always @(posedge clk) if (reset) av<=1'b0;
      else if (c_rd_req & c_rd_rdy) av<=1'b0;
      else if (cache_en & dmem_ren & ~c_rd_rdy) begin av<=1'b1; aaddr<=dmem_raddr; end

   cache #(.PAW(64), .SIZE_KB(128), .RDW(64), .WDW(64), .WRITABLE(1), .WRTHRU(1)) u_dcache
     (.clk(clk), .reset(reset),
      .rd_req(c_rd_req), .rd_rdy(c_rd_rdy), .rd_addr(c_rd_addr), .rd_data(c_rd_data), .rd_valid(c_rd_valid),
      .rd_resp_addr(c_rd_resp_addr),
      .rd_uncached(1'b0),
      .wr_req(cache_en & dmem_wen & ~c_wr_ack), .wr_addr(dmem_waddr), .wr_data(dmem_wdata),
      .wr_mask(dmem_wmask), .wr_ack(c_wr_ack), .wr_uncached(1'b0),
      .cbo_req(1'b0), .cbo_zero(1'b0), .cbo_keep(1'b0), .inv_req(1'b0), .inv_clean(1'b0), .inv_busy(),
      .l2_req(c_l2_req), .l2_we(c_l2_we), .l2_addr(c_l2_addr), .l2_wdata(c_l2_wdata),
      .l2_wmask(c_l2_wmask), .l2_rdata(c_l2_rdata), .l2_ack(c_l2_ack));

   // cache L2 responder: line read/write of `mem` (based at BASE), 2-cycle latency
   reg c_l2busy; reg [3:0] c_l2cnt; reg c_l2we_q; reg [57:0] c_l2ad_q; reg [511:0] c_l2wd_q;
   reg [63:0] c_l2wm_q;
   integer kk; reg [63:0] c_l2base;
   always @(posedge clk) begin
      c_l2_ack <= 1'b0;
      if (reset) c_l2busy <= 1'b0;
      else if (!c_l2busy && c_l2_req) begin
         c_l2busy<=1'b1; c_l2cnt<=4'd2; c_l2we_q<=c_l2_we; c_l2ad_q<=c_l2_addr; c_l2wd_q<=c_l2_wdata; c_l2wm_q<=c_l2_wmask;
      end else if (c_l2busy) begin
         if (c_l2cnt==0) begin
            c_l2base = ({{6{1'b0}},c_l2ad_q} << 6) - BASE;
            if (c_l2we_q) for (kk=0;kk<64;kk=kk+1) mem[c_l2base+kk] <= c_l2wm_q[kk] ? c_l2wd_q[kk*8 +: 8] : mem[c_l2base+kk];
            else          for (kk=0;kk<64;kk=kk+1) c_l2_rdata[kk*8 +: 8] <= mem[c_l2base+kk];
            c_l2_ack<=1'b1; c_l2busy<=1'b0;
         end else c_l2cnt <= c_l2cnt-1;
      end
   end

   // ---- nocache read return: combinational (memlat=0) or a 2-entry latency pipe ----
   // +memlat=N: each read returns N cycles after its ren, with data AND address
   // sampled at ren (safe: the LSU holds store drains while loads are in flight).
   // Two entries match the LSU's two in-flight slots; entry 0 presents first.
   reg  [15:0] memlat;
   reg         le_v [0:1];  reg [15:0] le_cnt [0:1];  reg [63:0] le_a [0:1], le_d [0:1];
   initial begin memlat = 16'd0; le_v[0]=1'b0; le_v[1]=1'b0; end
   wire le0_hit = le_v[0] && le_cnt[0]==16'd0;
   wire le1_hit = le_v[1] && le_cnt[1]==16'd0 && !le0_hit;
   always @(posedge clk) begin
      if (reset) begin le_v[0]<=1'b0; le_v[1]<=1'b0; end
      else begin
         if (le0_hit) le_v[0]<=1'b0; else if (le_v[0] && le_cnt[0]!=16'd0) le_cnt[0]<=le_cnt[0]-16'd1;
         if (le1_hit) le_v[1]<=1'b0; else if (le_v[1] && le_cnt[1]!=16'd0) le_cnt[1]<=le_cnt[1]-16'd1;
         if (!cache_en && memlat!=16'd0 && dmem_ren) begin
            if (!le_v[0] || le0_hit) begin le_v[0]<=1'b1; le_cnt[0]<=memlat-16'd1; le_a[0]<=dmem_raddr; le_d[0]<=rd64(dmem_raddr); end
            else begin le_v[1]<=1'b1; le_cnt[1]<=memlat-16'd1; le_a[1]<=dmem_raddr; le_d[1]<=rd64(dmem_raddr); end
         end
      end
   end
   always @* dmem_rdata = cache_en ? c_rd_data
                        : (memlat==16'd0) ? rd64(dmem_raddr)
                        : le0_hit ? le_d[0] : le_d[1];
   wire dmem_rvalid = cache_en ? c_rd_valid
                    : (memlat == 16'd0) ? 1'b1
                    : (le0_hit | le1_hit);
   wire [63:0] dmem_resp_addr = cache_en ? c_rd_resp_addr
                              : (memlat==16'd0) ? dmem_raddr
                              : le0_hit ? le_a[0] : le_a[1];
   // rdy drops during a live-but-ungranted request cycle too (av sets at the edge):
   // else the LSU fires into a full skid and the request is silently lost.
   wire dmem_rdy    = cache_en ? (~av & ~(dmem_ren & ~c_rd_rdy)) : 1'b1;
   wire dmem_wready = cache_en ? c_wr_ack : 1'b1;

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
      $display("  DISP: any_valid=%b ccfull=%b dispready=%b festall=%b sbfull=%b lqfull=%b rollv=%b dflt=%b ill=%b amogap=%b",
               dut.any_valid, dut.cc_full, &dut.disp_ready, |dut.fe_stall,
               dut.sb_full, dut.lq_full, dut.roll_v, dut.lsu_dfault_v, dut.ill_v, dut.amo_gap);
      $display("  IFLT: pend=%b ccempty=%b replay=%b devldsolo=%b injinfl=%b csr_irq=%b",
               dut.pend_iflt, dut.cc_empty, dut.replay_v, dut.devld_solo_v,
               dut.inject_inflight, dut.csr_irq_v);
      $display("  CC: cur=%0d committed=%0d cnt=%0d/%0d/%0d/%0d/%0d/%0d/%0d/%0d",
               dut.cur, dut.cc_committed, dut.cc.count[0], dut.cc.count[1], dut.cc.count[2], dut.cc.count[3],
               dut.cc.count[4], dut.cc.count[5], dut.cc.count[6], dut.cc.count[7]);
      rs_dump = 1'b1; #1;   // fire the per-shard RS dump (module-scope generate below; tracks IW)
      $finish;
   end

   // Per-shard RS dump for the timeout diagnostic. A runtime lane index can't select a
   // hierarchical path, so unroll over the shards with a genvar (tracks IW); the timeout
   // initial pulses rs_dump just before $finish.
   reg rs_dump = 1'b0;
   genvar gsh;
   generate for (gsh = 0; gsh < IW; gsh = gsh + 1) begin : rsdump
      always @(posedge rs_dump) begin : d
         integer e;
         $display("  SH%0d free_count=%0d rn_stall=%b", gsh,
                  dut.fe.u_dr.rn.lane[gsh].sh.fl.free_count, dut.fe.u_dr.rn.lane[gsh].sh.stall);
         for (e = 0; e < 16; e = e + 1)
            if (dut.sb.lane[gsh].sh.v[e])
               $display("  RS%0d[%0d] seq=%0d ck=%0d rdy=%b%b%b s1=%0d s2=%0d pd=%0d insn=%h", gsh, e,
                        dut.sb.lane[gsh].sh.sq[e], dut.sb.lane[gsh].sh.ck[e],
                        dut.sb.lane[gsh].sh.r1[e], dut.sb.lane[gsh].sh.r2[e],
                        dut.sb.lane[gsh].sh.r3[e], dut.sb.lane[gsh].sh.s1[e],
                        dut.sb.lane[gsh].sh.s2[e], dut.sb.lane[gsh].sh.pd[e],
                        dut.sb.lane[gsh].sh.py[e][196:165]);
      end
   end endgenerate

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
