`timescale 1ns/1ps
`default_nettype none

// Monitor boot harness: load the SmolRV64 monitor into soc_top's local SRAM (@0x7000_0000),
// reset there, and drive the UART RX with a command string so the (interactive) monitor runs
// end-to-end on the probe core. +monhex = monitor image (lram). +cmd injected over UART RX.
module tb;
   localparam [63:0] BASE = 64'h8000_0000;
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

   // behavioral DDR (16 MiB here -- the monitor itself does not touch DRAM)
   localparam DDR_BYTES = 1<<24;
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

   // ---- UART RX command injection: feed `cmd` one byte at a time as the UART accepts ----
   reg [8*256-1:0] cmd;  integer cmdlen, ci;  reg [31:0] startcyc;
   integer ncyc, c;  reg [8*256-1:0] monhex;
   initial begin
      rx_we=0; rx_data=0; ci=0;
      ncyc = 2000000;
      if (!$value$plusargs("monhex=%s", monhex)) begin $display("FATAL: need +monhex"); $finish; end
      for (c=0; c<DDR_BYTES; c=c+1) ram[c]=8'd0;
      $readmemh(monhex, dut.lram);                 // monitor image -> local SRAM @0x7000_0000
      if ($value$plusargs("cycles=%d", ncyc)) ;
      // default injected command: read mem @0x70000000 (echoes back the first monitor word)
      cmd = "R70000000\r"; cmdlen = 10;
      startcyc = 30000;                            // wait for the banner+prompt first
      reset=1; @(negedge clk); @(negedge clk); reset=0;
      for (c=0; c<ncyc; c=c+1) begin
         @(negedge clk);
         // drive one RX byte whenever the UART can accept and we're past the banner
         rx_we <= 1'b0;
         if (c > startcyc && ci < cmdlen && rx_ready && !rx_we) begin
            rx_we <= 1'b1; rx_data <= cmd[(cmdlen-1-ci)*8 +: 8]; ci <= ci+1;
         end
      end
      $display("\n[tb_mon: %0d cycles done, injected %0d/%0d bytes]", ncyc, ci, cmdlen);
      $finish;
   end
endmodule

`default_nettype wire
