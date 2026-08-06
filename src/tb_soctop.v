`timescale 1ns/1ps
`default_nettype none

// Smoke TB for soc_top: behavioral DDR (the cache-backed DRAM @0x8000_0000) drives the
// external ddr_* port; the local SRAM (boot/monitor @0x7000_0000) is internal to soc_top.
// Load a flat image (+hex) into whichever region it links at (default DDR), run until a
// store to `tohost` (+tohost): 1 => PASS. Exercises core -> I$/D$ -> arbiter -> {local SRAM,
// external DDR} + MMIO devices.
module tb;
   localparam [63:0] BASE = 64'h8000_0000;
   reg clk=0; always #5 clk=~clk;
   reg reset;
   wire        commit, dmem_wen;
   wire [63:0] dmem_waddr, dmem_wdata;  wire [7:0] dmem_wmask;
   wire        ddr_req, ddr_we;  wire [57:0] ddr_addr;  wire [511:0] ddr_wdata;
   wire [63:0] ddr_wmask;
   reg  [511:0] ddr_rdata;  reg ddr_ack;

   soc_top dut (.clk(clk), .reset(reset), .commit(commit),
                .dmem_wen(dmem_wen), .dmem_waddr(dmem_waddr),
                .dmem_wdata(dmem_wdata), .dmem_wmask(dmem_wmask),
                .ddr_req(ddr_req), .ddr_we(ddr_we), .ddr_addr(ddr_addr),
                .ddr_wdata(ddr_wdata), .ddr_wmask(ddr_wmask), .ddr_rdata(ddr_rdata), .ddr_ack(ddr_ack),
                .uart_rx_we(1'b0), .uart_rx_data(8'd0), .uart_rx_ready(),
                .uart_tx_ready(1'b1));

   // behavioral DDR (line port, 4-cycle latency): cache-backed DRAM at BASE
   reg [7:0] ram [0:(1<<21)-1];
   reg d_busy; reg [3:0] d_cnt; reg d_we_q; reg [57:0] d_ad_q; reg [511:0] d_wd_q;
   reg [63:0] d_wm_q;
   integer kb; reg [63:0] d_base;
   always @(posedge clk) begin
      ddr_ack <= 1'b0;
      if (reset) d_busy<=1'b0;
      else if (!d_busy && ddr_req) begin d_busy<=1'b1; d_cnt<=4'd4; d_we_q<=ddr_we; d_ad_q<=ddr_addr; d_wd_q<=ddr_wdata; d_wm_q<=ddr_wmask; end
      else if (d_busy) begin
         if (d_cnt==0) begin
            d_base = ({{6{1'b0}},d_ad_q} << 6) - BASE;
            if (d_we_q) for (kb=0;kb<64;kb=kb+1) ram[d_base+kb] <= d_wm_q[kb] ? d_wd_q[kb*8 +: 8] : ram[d_base+kb];
            else        for (kb=0;kb<64;kb=kb+1) ddr_rdata[kb*8 +: 8] <= ram[d_base+kb];
            ddr_ack<=1'b1; d_busy<=1'b0;
         end else d_cnt <= d_cnt-1;
      end
   end

   reg [63:0] tohost;  integer ncyc, c, i;  reg [8*256-1:0] hexfile;
   initial begin
      tohost = 64'h8000_1000;  ncyc = 2000000;
      if (!$value$plusargs("hex=%s", hexfile)) begin $display("FATAL: need +hex"); $finish; end
      for (i=0; i<(1<<21); i=i+1) ram[i] = 8'd0;
      $readmemh(hexfile, ram);                       // test images link @DDR (BASE)
      if ($value$plusargs("tohost=%h", tohost)) ;
      if ($value$plusargs("cycles=%d", ncyc)) ;
      reset=1; @(negedge clk); @(negedge clk); reset=0;
      for (c=0; c<ncyc; c=c+1) begin
         @(negedge clk);
         if (dmem_wen && (dmem_waddr - (dmem_waddr%8)) == tohost && dmem_wmask[0]) begin
            if (dmem_wdata[31:0] == 32'd1) $display("SOCTOP-TEST PASS");
            else $display("SOCTOP-TEST FAIL test=%0d (tohost=%h)", dmem_wdata[31:1], dmem_wdata);
            $finish;
         end
      end
      $display("SOCTOP-TEST TIMEOUT after %0d cycles", ncyc);
      $finish;
   end
endmodule

`default_nettype wire
