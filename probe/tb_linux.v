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
   reg  [511:0] ddr_rdata;  reg ddr_ack;
   reg          rx_we;  reg [7:0] rx_data;  wire rx_ready;

   soc_top #(.RESET_PC(64'h7000_0000)) dut
     (.clk(clk), .reset(reset), .commit(commit),
      .dmem_wen(dmem_wen), .dmem_waddr(dmem_waddr), .dmem_wdata(dmem_wdata), .dmem_wmask(dmem_wmask),
      .ddr_req(ddr_req), .ddr_we(ddr_we), .ddr_addr(ddr_addr),
      .ddr_wdata(ddr_wdata), .ddr_rdata(ddr_rdata), .ddr_ack(ddr_ack),
      .uart_rx_we(rx_we), .uart_rx_data(rx_data), .uart_rx_ready(rx_ready));

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

   reg [8*256-1:0] monhex, fw, dtb, initrd, cmd;
   integer ncyc, c, ci, cmdlen, pi;
   reg [63:0] srcring [0:63];  reg [63:0] tgtring [0:63];  reg [5:0] pcwr;
   reg frozen, dumped;  reg [63:0] mraddr_q;  integer la_idx;
   initial begin pcwr=0; frozen=0; dumped=0; mraddr_q=0; end
   initial begin
      rx_we=0; rx_data=0; ci=0; ncyc=200000000;
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
         // near the failure: did the bad RA value 0x80017000 get STORED (and where), or only
         // appear as a LOAD result (-> load/forwarding bug)?
         if (c > 1782000 && c < 1784000) begin
            if (dmem_wen && dmem_wdata == 64'h80017000)
               $display("[STORE-BADVAL c=%0d addr=%h wmask=%b]", c, dmem_waddr, dmem_wmask);
            if (dut.core.lsu_ld_wb_v && dut.core.lsu_ld_wb_val == 64'h80017000) begin
               // the load's phys addr was on mem_raddr a cycle earlier; show DDR content there
               la_idx = (mraddr_q - BASE) & (DDR_BYTES-1);
               $display("[LOAD-BADVAL c=%0d addr~%h ddr[addr]=%h%h%h%h%h%h%h%h]", c, mraddr_q,
                  ram[la_idx+7],ram[la_idx+6],ram[la_idx+5],ram[la_idx+4],
                  ram[la_idx+3],ram[la_idx+2],ram[la_idx+1],ram[la_idx+0]);
            end
         end
         // log every trap taken after the jump to DDR
         if (dut.core.eb.u_csr.trap_v && c > 30000)
            $display("[TRAP c=%0d cause=%0d epc=%h tval=%h intr=%b]", c,
               dut.core.eb.u_csr.trap_cause, dut.core.eb.u_csr.trap_epc,
               dut.core.eb.u_csr.trap_tval, dut.core.eb.u_csr.trap_is_intr);
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
