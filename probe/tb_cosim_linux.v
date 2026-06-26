`timescale 1ns/1ps
`default_nettype none

// Linux-boot cosim harness: soc_top reset DIRECTLY to OpenSBI (0x8000_0000) with
// a1=DTB seeded into the RF (+a1=, via rf_shard's PROBE_COSIM seed) -- the monitor
// is bypassed so the DUT and simmerv start at the same PC (mirrors the SmolRV64
// linux-cosim recipe). The cosim retire stream + lockstep lives in backend_top
// (PROBE_COSIM) / probe_cosim.cpp; this TB just loads DDR, clocks, and times out.
//   +fw=... +dtb=... [+initrd=...] [+cycles=N]   (C side reads the same plusargs)
module tb;
   localparam [63:0] BASE = 64'h8000_0000;
   localparam        DDR_BYTES = 1<<28;            // 256 MiB
   localparam [63:0] OFF_FW = 64'h000_0000, OFF_DTB = 64'h200_0000, OFF_INITRD = 64'h762_b000;

   reg clk=0; always #5 clk=~clk;
   reg reset;
   wire        commit, dmem_wen;
   wire [63:0] dmem_waddr, dmem_wdata;  wire [7:0] dmem_wmask;
   wire        ddr_req, ddr_we;  wire [57:0] ddr_addr;  wire [511:0] ddr_wdata;
   reg  [511:0] ddr_rdata;  reg ddr_ack;
   wire        rx_ready;

   soc_top #(.RESET_PC(64'h8000_0000)) dut
     (.clk(clk), .reset(reset), .commit(commit),
      .dmem_wen(dmem_wen), .dmem_waddr(dmem_waddr), .dmem_wdata(dmem_wdata), .dmem_wmask(dmem_wmask),
      .ddr_req(ddr_req), .ddr_we(ddr_we), .ddr_addr(ddr_addr),
      .ddr_wdata(ddr_wdata), .ddr_rdata(ddr_rdata), .ddr_ack(ddr_ack),
      .uart_rx_we(1'b0), .uart_rx_data(8'd0), .uart_rx_ready(rx_ready));

   // behavioral DDR (256 MiB, 4-cycle line latency)
   reg [7:0] ram [0:DDR_BYTES-1];
   reg d_busy; reg [3:0] d_cnt; reg d_we_q; reg [57:0] d_ad_q; reg [511:0] d_wd_q;
   integer kb; reg [63:0] d_base;
   always @(posedge clk) begin
      ddr_ack <= 1'b0;
      if (reset) d_busy<=1'b0;
      else if (!d_busy && ddr_req) begin d_busy<=1'b1; d_cnt<=4'd4; d_we_q<=ddr_we; d_ad_q<=ddr_addr; d_wd_q<=ddr_wdata; end
      else if (d_busy) begin
         if (d_cnt==0) begin
            d_base = (({{6{1'b0}},d_ad_q} << 6) - BASE) & (DDR_BYTES-1);
            if (d_we_q) for (kb=0;kb<64;kb=kb+1) ram[d_base+kb] <= d_wd_q[kb*8 +: 8];
            else        for (kb=0;kb<64;kb=kb+1) ddr_rdata[kb*8 +: 8] <= ram[d_base+kb];
            ddr_ack<=1'b1; d_busy<=1'b0;
         end else d_cnt <= d_cnt-1;
      end
   end

   task load_bin; input [8*256-1:0] fname; input [63:0] off;
      integer fd, n; begin
         fd = $fopen(fname, "rb");
         if (fd == 0) begin $display("FATAL: cannot open %0s", fname); $finish; end
         n  = $fread(ram, fd, off, DDR_BYTES-off);
         $fclose(fd);
         $display("[cosim-linux: loaded %0d bytes @ DDR+%h]", n, off);
      end
   endtask

   reg [8*256-1:0] fw, dtb, initrd;
   integer ncyc, c, b2;
   initial begin
      ncyc = 200000000;
      if (!$value$plusargs("fw=%s", fw))  begin $display("FATAL: +fw");  $finish; end
      if (!$value$plusargs("dtb=%s", dtb)) begin $display("FATAL: +dtb"); $finish; end
      if ($value$plusargs("cycles=%d", ncyc)) ;
      load_bin(fw,  OFF_FW);
      load_bin(dtb, OFF_DTB);
      if ($value$plusargs("initrd=%s", initrd)) load_bin(initrd, OFF_INITRD);

      reset=1; @(negedge clk); @(negedge clk); reset=0;
      for (c=0; c<ncyc; c=c+1) begin
         @(negedge clk);
         if ((c % 1000000) == 0) $display("[c=%0d pc=%h]", c, dut.imem_addr);
`ifdef LSU_TAP
         // window around the store->load divergence (~retire 3.03M ~ cycle 9.0-9.2M).
         // page offset 0xf88 survives translation -> frame-PA-independent filter.
         if (c > 8800000 && c < 9300000) begin
            if (dut.core.u_lsu.mem_wen && dut.core.u_lsu.mem_waddr[11:0]==12'hf88)
               $display("[c=%0d ST-DRAIN pa=%h data=%h mask=%b]", c,
                  dut.core.u_lsu.mem_waddr, dut.core.u_lsu.mem_wdata, dut.core.u_lsu.mem_wmask);
            // store FILL at execute: is the store's data correct (0xfb0) entering the SB?
            for (b2 = 0; b2 < 4; b2 = b2 + 1)
               if (dut.core.u_lsu.exe_st_v[b2] && dut.core.u_lsu.exe_st_addr[b2*64 +: 12]==12'hf88)
                  $display("[c=%0d ST-FILL lane%0d va=%h data=%h idx=%0d]", c, b2,
                     dut.core.u_lsu.exe_st_addr[b2*64 +: 64], dut.core.u_lsu.exe_st_data[b2*64 +: 64],
                     dut.core.u_lsu.exe_st_idx[b2*2 +: 2]);
            if (dut.core.u_lsu.mem_ren && dut.core.u_lsu.mem_raddr[11:0]==12'hf88)
               $display("[c=%0d LD-REQ  pa=%h]", c, dut.core.u_lsu.mem_raddr);
            if (dut.core.u_lsu.mem_rvalid && dut.core.u_lsu.mem_raddr[11:0]==12'hf88)
               $display("[c=%0d LD-RDATA pa=%h rdata=%h]", c,
                  dut.core.u_lsu.mem_raddr, dut.core.u_lsu.mem_rdata);
            if (dut.core.u_lsu.ld_wb_v && dut.core.u_lsu.ld_wb_val==64'd0
                && dut.core.u_lsu.mem_raddr[11:0]==12'hf88)
               $display("[c=%0d LD-WB    pa=%h val=%h pd=%0d]", c, dut.core.u_lsu.mem_raddr,
                  dut.core.u_lsu.ld_wb_val, dut.core.u_lsu.ld_wb_pdst);
            // AMO drain (different line) -- confirm it's not clobbering 0xf88
            if (dut.core.u_lsu.mem_wen && dut.core.u_lsu.ast != 3'd0)
               $display("[c=%0d AMO-WR  pa=%h data=%h st=%0d]", c,
                  dut.core.u_lsu.mem_waddr, dut.core.u_lsu.mem_wdata, dut.core.u_lsu.ast);
            // full store-buffer dump right at the failing load (c~9061759)
            if (c >= 9061755 && c <= 9061762)
               for (b2 = 0; b2 < 4; b2 = b2 + 1)
                  $display("[c=%0d SB[%0d] v=%b cmt=%b seq=%0d va=%h data=%h nb=%0d ast=%0d ld_sel=%0d p_v=%b]",
                     c, b2, dut.core.u_lsu.sb_v[b2], dut.core.u_lsu.sb_cmt[b2], dut.core.u_lsu.sb_seq[b2],
                     dut.core.u_lsu.sb_addr[b2], dut.core.u_lsu.sb_data[b2], dut.core.u_lsu.sb_nb[b2],
                     dut.core.u_lsu.ast, dut.core.u_lsu.ld_sel, dut.core.u_lsu.p_v);
         end
`endif
      end
      $display("COSIM-LINUX TIMEOUT after %0d cycles (pc~%h)", ncyc, dut.imem_addr);
      $finish;
   end
endmodule

`default_nettype wire
