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
   wire [3:0]          imem_avail = 4'd8;
   wire [63:0]         dmem_raddr;
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
      .hw_ip(12'd0),                       // no CLINT/PLIC in this device-less harness
      .dmem_raddr(dmem_raddr), .dmem_rdata(dmem_rdata),
      .dmem_wen(dmem_wen), .dmem_waddr(dmem_waddr), .dmem_wdata(dmem_wdata),
      .dmem_wmask(dmem_wmask),
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
   always @(imem_addr or wtick) begin
      for (m=0;m<HW;m=m+1) begin
         imem_data[m*16 +: 8]   = mem[(imem_addr-BASE)+2*m];
         imem_data[m*16+8 +: 8] = mem[(imem_addr-BASE)+2*m+1];
      end
   end
   always @(dmem_raddr or wtick) dmem_rdata = rd64(dmem_raddr);

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

   // apply stores to memory (after the monitor sees them)
   always @(posedge clk) if (!reset && dmem_wen) begin
      for (b2=0;b2<8;b2=b2+1)
         if (dmem_wmask[b2]) mem[(dmem_waddr-BASE)+b2] <= dmem_wdata[b2*8 +: 8];
      wtick <= ~wtick;
   end
endmodule

`default_nettype wire
