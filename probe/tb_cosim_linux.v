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
`ifdef COSIM_MEM_SIZE_LG2
   localparam [63:0] DDR_BYTES = 64'd1 << `COSIM_MEM_SIZE_LG2;   // 64-bit: LG2>=31 (>=2GiB) ok
`else
   localparam [63:0] DDR_BYTES = 64'd1 << 28;       // 256 MiB (linux default)
`endif
   // DTB/initrd load offsets default to the linux workload but are overridable per
   // workload via +dtb_off=/+initrd_off= (gb5/gb6 place them much higher); fw is @0.
   localparam [63:0] OFF_FW = 64'h000_0000, OFF_DTB_DEF = 64'h200_0000, OFF_INITRD_DEF = 64'h762_b000;
   reg [63:0] off_dtb, off_initrd;

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

   // behavioral DDR (DDR_BYTES, 4-cycle line latency). Modeled as a 512-bit LINE array (the
   // ddr_* port is 64-byte lines), so the element count is DDR_BYTES/64 -- which stays under
   // the ~1-billion-element array-dimension limit even at 2 GiB (a flat byte array overflows
   // it). A line holds bytes little-endian: byte k at bits [k*8 +: 8], matching the port.
   localparam [63:0] NLINES = DDR_BYTES >> 6;
   localparam [63:0] LBASE  = BASE >> 6;            // DDR base as a line address
   reg [511:0] lram [0:NLINES-1];
   reg d_busy; reg [3:0] d_cnt; reg d_we_q; reg [57:0] d_ad_q; reg [511:0] d_wd_q;
   reg [63:0] line;
   always @(posedge clk) begin
      ddr_ack <= 1'b0;
      if (reset) d_busy<=1'b0;
      else if (!d_busy && ddr_req) begin d_busy<=1'b1; d_cnt<=4'd4; d_we_q<=ddr_we; d_ad_q<=ddr_addr; d_wd_q<=ddr_wdata; end
      else if (d_busy) begin
         if (d_cnt==0) begin
            line = ({6'd0, d_ad_q} - LBASE) & (NLINES-1);   // ddr_addr is the 64-byte line index
            if (d_we_q) lram[line] <= d_wd_q;
            else        ddr_rdata  <= lram[line];
            ddr_ack<=1'b1; d_busy<=1'b0;
         end else d_cnt <= d_cnt-1;
      end
   end

   // Load a raw image at byte offset `off` (64-byte aligned for all workloads). $fread fills
   // each 512-bit element MSB-first, so reverse the 64 bytes of every loaded line back to the
   // little-endian byte order the ddr_* port (and the byte-array model it replaced) uses.
   task load_bin; input [8*256-1:0] fname; input [63:0] off;
      integer fd, n, j; reg [63:0] sl, nl, li; reg [7:0] t; begin
         fd = $fopen(fname, "rb");
         if (fd == 0) begin $display("FATAL: cannot open %0s", fname); $finish; end
         sl = off >> 6;
         n  = $fread(lram, fd, sl, NLINES - sl);
         $fclose(fd);
         nl = (n + 63) >> 6;
         for (li = sl; li < sl + nl; li = li + 1)
            for (j = 0; j < 32; j = j + 1) begin
               t = lram[li][j*8 +: 8];
               lram[li][j*8 +: 8]      = lram[li][(63-j)*8 +: 8];
               lram[li][(63-j)*8 +: 8] = t;
            end
         $display("[cosim-linux: loaded %0d bytes @ DDR+%h (%0d lines)]", n, off, nl);
      end
   endtask

   reg [8*256-1:0] fw, dtb, initrd;
   integer ncyc, c, b2;
   initial begin
      ncyc = 200000000;
      if (!$value$plusargs("fw=%s", fw))  begin $display("FATAL: +fw");  $finish; end
      if (!$value$plusargs("dtb=%s", dtb)) begin $display("FATAL: +dtb"); $finish; end
      if ($value$plusargs("cycles=%d", ncyc)) ;
      off_dtb = OFF_DTB_DEF; off_initrd = OFF_INITRD_DEF;
      if ($value$plusargs("dtb_off=%h",    off_dtb))    ;
      if ($value$plusargs("initrd_off=%h", off_initrd)) ;
      load_bin(fw,  OFF_FW);
      load_bin(dtb, off_dtb);
      if ($value$plusargs("initrd=%s", initrd)) load_bin(initrd, off_initrd);

      reset=1; @(negedge clk); @(negedge clk); reset=0;
      for (c=0; c<ncyc; c=c+1) begin
         @(negedge clk);
         if ((c % 1000000) == 0) $display("[c=%0d pc=%h]", c, dut.imem_addr);
`ifdef LSU_TAP
         // rename check (any cycle): store sd x8 @ ...8003ddb6 -> its rs2 physreg (ps2)
         // vs x8 producer addi x8 @ ...80002f40 -> its dest (pdst). Mismatch => rename bug.
         // Forwarding trace: whenever a load is HELD in MERGE (p_v) reading the 0xf88 word
         // (VA[11:3]=0x1f1), dump the forwarding decision -- s_use (eligible older stores),
         // mem_rdata (memory), c_val (merge result), and the SB. Shows forward-vs-memory.
         if ($time > 95450000000 && $time < 95470000000 && dut.core.u_lsu.p_v)
            $display("[%0t PV p_w0=%h off=%h c_val=%h]", $time,
               dut.core.u_lsu.p_w0, dut.core.u_lsu.p_w0[8:0], dut.core.u_lsu.c_val);
         if ($time > 90000000000 && dut.core.u_lsu.p_v
             && (dut.core.u_lsu.p_w0[8:0] == 9'h1f1)) begin
            $display("[%0t LD-MERGE p_seq=%0d p_w0=%h p_lb=%0d s_use=%b mem_rdata=%h c_val=%h]",
               $time, dut.core.u_lsu.p_seq, dut.core.u_lsu.p_w0, dut.core.u_lsu.p_lb,
               dut.core.u_lsu.s_use, dut.core.u_lsu.mem_rdata, dut.core.u_lsu.c_val);
            for (b2 = 0; b2 < 4; b2 = b2 + 1)
               $display("    SB[%0d] v=%b rdy=%b cmt=%b seq=%0d w0=%h d0=%h be0=%b", b2,
                  dut.core.u_lsu.sb_v[b2], dut.core.u_lsu.sb_rdy[b2], dut.core.u_lsu.sb_cmt[b2],
                  dut.core.u_lsu.sb_seq[b2], dut.core.u_lsu.sb_w0[b2], dut.core.u_lsu.sb_d0[b2],
                  dut.core.u_lsu.sb_be0[b2]);
         end
`endif
      end
      $display("COSIM-LINUX TIMEOUT after %0d cycles (pc~%h)", ncyc, dut.imem_addr);
      $finish;
   end
endmodule

`default_nettype wire
