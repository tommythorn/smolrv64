`timescale 1ns/1ps
`default_nettype none

// Linux boot harness for soc_top. Loads the SmolRV64 monitor into local SRAM (@0x7000_0000,
// reset there) and the OpenSBI fw_payload + DTB + initramfs into the behavioral DDR
// (@0x8000_0000), then drives the monitor over UART RX with "X8000_0000 0 8200_0000" to jump
// into OpenSBI (a0=hartid, a1=DTB). UART TX is printed; the boot log is the result.
//   +fw=...  +dtb=...  +initrd=...  +monhex=...  [+cycles=N]
module tb;
   localparam [63:0] BASE = 64'h8000_0000;
   localparam        DDR_BYTES = 1<<28;            // 256 MiB
   localparam [63:0] OFF_FW = 64'h000_0000, OFF_DTB = 64'h200_0000, OFF_INITRD = 64'h762_b000;

   reg clk=0; always #5 clk=~clk;
   reg reset;
   wire        commit, dmem_wen;
   wire [63:0] dmem_waddr, dmem_wdata;  wire [7:0] dmem_wmask;
   wire        ddr_req, ddr_we;  wire [57:0] ddr_addr;  wire [511:0] ddr_wdata;
   wire [63:0] ddr_wmask;
   reg  [511:0] ddr_rdata;  reg ddr_ack;
   reg          rx_we;  reg [7:0] rx_data;  wire rx_ready;

   soc_top #(.RESET_PC(64'h7000_0000)) dut
     (.clk(clk), .reset(reset), .commit(commit),
      .dmem_wen(dmem_wen), .dmem_waddr(dmem_waddr), .dmem_wdata(dmem_wdata), .dmem_wmask(dmem_wmask),
      .ddr_req(ddr_req), .ddr_we(ddr_we), .ddr_addr(ddr_addr),
      .ddr_wdata(ddr_wdata), .ddr_wmask(ddr_wmask), .ddr_rdata(ddr_rdata), .ddr_ack(ddr_ack),
      .uart_rx_we(rx_we), .uart_rx_data(rx_data), .uart_rx_ready(rx_ready),
      .uart_tx_ready(1'b1));

   // behavioral DDR (256 MiB, 4-cycle line latency)
   reg [7:0] ram [0:DDR_BYTES-1];
   reg d_busy; reg [3:0] d_cnt; reg d_we_q; reg [57:0] d_ad_q; reg [511:0] d_wd_q;
   reg [63:0] d_wm_q;
   integer kb; reg [63:0] d_base;
   always @(posedge clk) begin
      ddr_ack <= 1'b0;
      if (reset) d_busy<=1'b0;
      else if (!d_busy && ddr_req) begin d_busy<=1'b1; d_cnt<=4'd4; d_we_q<=ddr_we; d_ad_q<=ddr_addr; d_wd_q<=ddr_wdata; d_wm_q<=ddr_wmask; end
      else if (d_busy) begin
         if (d_cnt==0) begin
            d_base = (({{6{1'b0}},d_ad_q} << 6) - BASE) & (DDR_BYTES-1);
            if (d_we_q) for (kb=0;kb<64;kb=kb+1) ram[d_base+kb] <= d_wm_q[kb] ? d_wd_q[kb*8 +: 8] : ram[d_base+kb];
            else        for (kb=0;kb<64;kb=kb+1) ddr_rdata[kb*8 +: 8] <= ram[d_base+kb];
            ddr_ack<=1'b1; d_busy<=1'b0;
         end else d_cnt <= d_cnt-1;
      end
   end

   // ---- load a binary file into ram[] at a byte offset (via $fread) ----
   task load_bin; input [8*256-1:0] fname; input [63:0] off;
      integer fd, n; begin
         fd = $fopen(fname, "rb");
         if (fd == 0) begin $display("FATAL: cannot open %0s", fname); $finish; end
         n  = $fread(ram, fd, off, DDR_BYTES-off);
         $fclose(fd);
         $display("[tb_linux: loaded %0d bytes @ DDR+%h]", n, off);
      end
   endtask

   // read 8 little-endian bytes from the behavioral DDR (a physical addr in [BASE, BASE+DDR))
   function [63:0] rd64; input [63:0] pa; integer k; reg [63:0] v;
      begin v=0; for (k=0;k<8;k=k+1) v[k*8 +: 8] = ram[((pa - BASE) & (DDR_BYTES-1)) + k]; rd64=v; end
   endfunction
   // manual Sv39 walk of `satp`'s table for `va`, printing each PTE level + the verdict.
   // Compares against the probe's MMU fault: page mapped here -> probe faulted spuriously.
   task walk_sv39; input [63:0] satp; input [63:0] va;
      reg [63:0] base, pte, pa; reg [8:0] v2,v1,v0; reg leaf, bad;
      begin
         v2=va[38:30]; v1=va[29:21]; v0=va[20:12];
         base={satp[43:0],12'd0};
         $display("  [WALK va=%h satp=%h root=%h v2=%0d v1=%0d v0=%0d]", va, satp, base, v2, v1, v0);
         begin : scan  // dump every non-zero root (L2) entry to characterize the table
            integer e;
            for (e=0; e<512; e=e+1) if (rd64(base + e*8) != 0)
               $display("    root[%0d] = %h", e, rd64(base + e*8));
         end
         pte=rd64(base + v2*8);
         $display("    L2 @%h = %h (V%b R%b W%b X%b)", base+v2*8, pte, pte[0],pte[1],pte[2],pte[3]);
         if (!pte[0]||(!pte[1]&&pte[2])) $display("    -> L2 INVALID (fault)");
         else if (pte[1]|pte[3]) $display("    -> L2 LEAF 1G -> pa=%h", {pte[53:28],va[29:0]});
         else begin
            base={pte[53:10],12'd0};
            pte=rd64(base + v1*8);
            $display("    L1 @%h = %h (V%b R%b W%b X%b)", base+v1*8, pte, pte[0],pte[1],pte[2],pte[3]);
            if (!pte[0]||(!pte[1]&&pte[2])) $display("    -> L1 INVALID (fault)");
            else if (pte[1]|pte[3]) $display("    -> L1 LEAF 2M -> pa=%h", {pte[53:19],va[20:0]});
            else begin
               base={pte[53:10],12'd0};
               pte=rd64(base + v0*8);
               $display("    L0 @%h = %h (V%b R%b W%b X%b A%b D%b)", base+v0*8, pte, pte[0],pte[1],pte[2],pte[3],pte[6],pte[7]);
               if (!pte[0]||(!pte[1]&&pte[2])) $display("    -> L0 INVALID (fault)");
               else if (pte[1]|pte[3]) $display("    -> L0 LEAF 4K -> pa=%h (MAPPED)", {pte[53:10],va[11:0]});
               else $display("    -> L0 not-a-leaf (fault)");
            end
         end
      end
   endtask

   reg [8*256-1:0] monhex, fw, dtb, initrd, cmd;
   integer ci, cmdlen, pi, nwalk;
   reg [63:0] ncyc, c;        // 64-bit: long Linux runs exceed 2^32 cycles
   reg [63:0] srcring [0:63];  reg [63:0] tgtring [0:63];  reg [5:0] pcwr;
   reg frozen, dumped;  reg [63:0] mraddr_q;  integer la_idx, si;
   initial begin pcwr=0; frozen=0; dumped=0; mraddr_q=0; end
   initial begin
      rx_we=0; rx_data=0; ci=0; ncyc=200000000; nwalk=0;
      if (!$value$plusargs("monhex=%s", monhex)) begin $display("FATAL: +monhex"); $finish; end
      if (!$value$plusargs("fw=%s", fw))        begin $display("FATAL: +fw"); $finish; end
      if (!$value$plusargs("dtb=%s", dtb))      begin $display("FATAL: +dtb"); $finish; end
      if ($value$plusargs("cycles=%d", ncyc)) ;
      $readmemh(monhex, dut.lram);
      load_bin(fw,  OFF_FW);
      load_bin(dtb, OFF_DTB);
      if ($value$plusargs("initrd=%s", initrd)) load_bin(initrd, OFF_INITRD);

      // monitor "X<addr> [a0 [a1]]" -> jump to 0x80000000 with a0=0, a1=0x82000000 (DTB)
      cmd = "X80000000 0 82000000\r"; cmdlen = 21;

      reset=1; @(negedge clk); @(negedge clk); reset=0;
      for (c=0; c<ncyc; c=c+1) begin
         @(negedge clk);
         mraddr_q <= dut.core.u_lsu.mem_raddr;     // load addr lags the writeback by ~1 cycle
         rx_we <= 1'b0;
         if (c > 30000 && ci < cmdlen && rx_ready && !rx_we) begin
            rx_we <= 1'b1; rx_data <= cmd[(cmdlen-1-ci)*8 +: 8]; ci <= ci+1;
         end
         if ((c % 500000) == 0) $display("[c=%0d pc=%h commit=%b]", c, dut.imem_addr, commit);
         // fine PC+satp+priv trace through the MMU transition window
         if (c > 10080000 && c < 11100000 && (c % 20000)==0)
            $display("[MMU c=%0d pc=%h satp=%h priv=%0d]", c, dut.imem_addr, dut.core.eb.u_csr.satp, dut.core.eb.u_csr.priv);
         // tap the dMMU (load) PTW around the cause-13 fault: the PTEs it reads + its decision
         if (c > 11055000 && c < 11066000) begin
            if (dut.core.u_lsu.u_ldmmu.st==2'd2 && dut.core.u_lsu.u_ldmmu.ptw_rvalid)
               $display("[dMMU c=%0d lvl=%0d ptw_addr=%h pte=%h]", c, dut.core.u_lsu.u_ldmmu.lvl,
                  dut.core.u_lsu.u_ldmmu.ptw_addr, dut.core.u_lsu.u_ldmmu.ptw_rdata);
            if (dut.core.u_lsu.u_ldmmu.w_done && dut.core.u_lsu.u_ldmmu.w_fault)
               $display("[dMMU c=%0d WALK-FAULT va=%h cause=%0d]", c,
                  dut.core.u_lsu.u_ldmmu.va_q, dut.core.u_lsu.u_ldmmu.w_cause);
            if (dut.core.u_lsu.u_ldmmu.t_fault)
               $display("[dMMU c=%0d t_fault va=%h tlb_hit=%b noncanon=%b hit_pf=%b wdm=%b]", c,
                  dut.core.u_lsu.u_ldmmu.req_vaddr, dut.core.u_lsu.u_ldmmu.tlb_hit,
                  dut.core.u_lsu.u_ldmmu.noncanon, dut.core.u_lsu.u_ldmmu.hit_perm_fault,
                  dut.core.u_lsu.u_ldmmu.wdm);
         end
         // watch stores into the root page table (0x80a8c000) -- did the kernel write root[2]?
         if (c < 10100000 && dmem_wen && (dmem_waddr & ~64'hfff) == 64'h80a8c000)
            $display("[PTSTORE c=%0d addr=%h data=%h mask=%b]", c, dmem_waddr, dmem_wdata, dmem_wmask);
         // adapter request/response sequence: who is the cache serving vs the LSU's load addr?
         if (c > 1783560 && c < 1783612)
            $display("[ADPSEQ c=%0d dmem_ren=%b dmem_raddr=%h | dc_st=%0d dc_cur=%h dc_rd_v=%b | raw_rv=%b dmem_rv=%b c_rd_pend=%b]",
               c, dut.dmem_ren, dut.dmem_raddr, dut.u_dcache.st, dut.u_dcache.cur_line,
               dut.dc_rd_valid, dut.raw_rvalid, dut.dmem_rvalid, dut.c_rd_pend);
         // soc_top D$ read-adapter tap: is the bad value from the CACHE (dc_rd_data) or the
         // sticky-rdata adapter leaking a prior load (c_rdd_st via c_st_ok)?
         if (c > 1783600 && c < 1783615)
            $display("[ADPT c=%0d dmem_rvalid=%b dmem_rdata=%h | dc_rd_v=%b dc_rd_data=%h | c_st_ok=%b c_rdd_st=%h]",
               c, dut.dmem_rvalid, dut.dmem_rdata, dut.dc_rd_valid, dut.dc_rd_data, dut.c_st_ok, dut.c_rdd_st);
         // LSU MERGE tap for the bad load (addr 0x80016ed8): mem_rdata (cache/mem return) vs
         // c_val (merged result). rdata==badval -> stale cache line; rdata ok but c_val bad -> forward.
         if (c > 1783000 && c < 1783650 && dut.core.u_lsu.mem_rvalid
             && dut.core.u_lsu.mem_raddr == 64'h80016ed8)
            $display("[LSU-MERGE c=%0d raddr=%h mem_rdata=%h c_val=%h merge_fire=%b]", c,
               dut.core.u_lsu.mem_raddr, dut.core.u_lsu.mem_rdata, dut.core.u_lsu.c_val,
               dut.core.u_lsu.merge_fire);
         // PC-filtered LSU trace near the failure: the `ld ra,56(sp)` (0x80001e0a) address
         // (-> sp) on every shard, + every store into the suspect stack/RA region. brp[i] is
         // shard i's EX-stage PC (= ex_pc, unconditional); eb_agu[i] is its AGU address.
         if (c > 1778000 && c < 1784000) begin
            for (si=0; si<4; si=si+1) if (dut.core.ex_valid[si]) begin
               if (dut.core.eb.brp[si*64 +: 64]==64'h80001e0a && dut.core.ex_mem[si] && !dut.core.ex_store[si]) begin
                  la_idx = (dut.core.eb_agu[si*64 +: 64] - BASE) & (DDR_BYTES-1);
                  $display("[LDRA  c=%0d sh=%0d ld_addr=%h (sp=%h) ddr[addr]=%h%h%h%h%h%h%h%h]", c, si,
                     dut.core.eb_agu[si*64 +: 64], dut.core.eb_agu[si*64 +: 64]-64'd56,
                     ram[la_idx+7],ram[la_idx+6],ram[la_idx+5],ram[la_idx+4],
                     ram[la_idx+3],ram[la_idx+2],ram[la_idx+1],ram[la_idx+0]);
               end
               if (dut.core.ex_mem[si] && dut.core.ex_store[si] &&
                   dut.core.eb_agu[si*64 +: 64] >= 64'h80016e00 && dut.core.eb_agu[si*64 +: 64] < 64'h80017100)
                  $display("[STK-ST c=%0d pc=%h st_addr=%h data=%h]", c,
                     dut.core.eb.brp[si*64 +: 64], dut.core.eb_agu[si*64 +: 64], dut.core.eb_stdata[si*64 +: 64]);
            end
         end
         // log every trap taken after the jump to DDR
         if (dut.core.eb.u_csr.trap_v && c > 30000) begin
            $display("[TRAP c=%0d cause=%0d epc=%h tval=%h to_s=%b stvec=%h mtvec=%h tgt=%h satp=%h priv=%0d]", c,
               dut.core.eb.u_csr.trap_cause, dut.core.eb.u_csr.trap_epc, dut.core.eb.u_csr.trap_tval,
               dut.core.eb.u_csr.trap_to_s, dut.core.eb.u_csr.stvec, dut.core.eb.u_csr.mtvec,
               dut.core.eb.u_csr.redir_target, dut.core.eb.u_csr.satp, dut.core.eb.u_csr.priv);
            // on the first page fault, manually walk the kernel's table for tval -> spurious?
            if ((dut.core.eb.u_csr.trap_cause==64'd12 || dut.core.eb.u_csr.trap_cause==64'd13
                 || dut.core.eb.u_csr.trap_cause==64'd15) && nwalk < 4) begin
               nwalk <= nwalk + 1;
               walk_sv39(dut.core.eb.u_csr.satp, dut.core.eb.u_csr.trap_tval);
            end
         end
         // ring of architectural control transfers (oldest mispredict redirect): src_pc -> target.
         // Fetch is fall-through, so every taken jump/branch shows here -- the bad jump to the
         // zero page will be in the tail with its source instruction PC.
         if (!frozen && dut.core.eb.redirect) begin
            srcring[pcwr] <= dut.core.eb.redirect_src_pc; tgtring[pcwr] <= dut.core.eb.redirect_target;
            pcwr <= pcwr + 1'b1;
            if (dut.core.eb.redirect_target >= 64'h80016000 && dut.core.eb.redirect_target < 64'h80018000)
               $display("[BADJUMP c=%0d src=%h -> target=%h trap=%b]", c,
                  dut.core.eb.redirect_src_pc, dut.core.eb.redirect_target, dut.core.eb.redirect_is_trap);
         end
         // freeze + dump the recent control transfers on the first illegal-instruction trap
         if (dut.core.eb.u_csr.trap_v && dut.core.eb.u_csr.trap_cause==64'd2 && c>30000 && !frozen)
            frozen <= 1'b1;
         if (frozen && !dumped) begin
            dumped <= 1'b1;
            $display("[recent control transfers before the illegal trap (oldest first), src -> target:]");
            for (pi=0; pi<64; pi=pi+1)
               $display("   %h -> %h", srcring[(pcwr + pi) & 6'h3f], tgtring[(pcwr + pi) & 6'h3f]);
            $finish;
         end
      end
      $display("\n[tb_linux: %0d cycles done]", ncyc);
      $finish;
   end
endmodule

`default_nettype wire
