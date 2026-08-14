`timescale 1ns/1ps
`default_nettype none

// riscv-tests harness for the in-order core (ino_core), a stripped sibling of
// probe/tb_riscv.v: a flat byte memory at 0x80000000 serves both the
// combinational fetch window and the LSU's data port, and a store to `tohost`
// (+tohost=<hex>) ends the run -- 1 = PASS, else FAIL with test# = tohost>>1.
//
//   +hex=<file>    byte image, one hex byte per line (od -An -v -tx1)
//   +tohost=<hex>  tohost address (default 0x80001000)
//   +cycles=<n>    timeout (default 200000)
//   +trace=1       per-retire disassembly-free trace (pc/insn/rd)
module tb;
   localparam PCW = 64, SEQW = 8, HW = 2;
   localparam [63:0] BASE = 64'h8000_0000;
   // 4 MiB. 2 MiB is NOT enough: rv64ssvnapot-p-napot stores to PA 0x80208010 and
   // reads it back physically. The OoO probe's TB gets away with 2 MiB only because
   // its store buffer forwards the value to the load -- the store itself falls off
   // the end of the array. This core has no store buffer, so the memory must be real.
   localparam        SIZE = 1<<22;

   reg  clk = 0; always #5 clk = ~clk;
   reg  reset;

   wire [PCW-1:0]   imem_addr;
   reg  [HW*16-1:0] imem_data;
   wire [1:0]       imem_avail = HW[1:0];      // window always full from this TB memory
   wire [63:0]      dmem_raddr, dmem_waddr, dmem_wdata;
   reg  [63:0]      dmem_rdata;
   wire             dmem_ren, dmem_wen;
   wire [7:0]       dmem_wmask;
   wire [55:0]      ptw_addr, dptw_addr;
   wire             ptw_read, dptw_read;
   reg  [63:0]      ptw_rdata, dptw_rdata;
   reg              ptw_rvalid, dptw_rvalid;
   wire             retire, redirect;
   wire [PCW-1:0]   retire_pc, redirect_target;
   wire [31:0]      retire_insn;

   ino_core #(.PCW(PCW), .SEQW(SEQW), .HW(HW), .RESET_PC(BASE)) dut
     (.clk(clk), .reset(reset),
      .imem_addr(imem_addr), .imem_data(imem_data), .imem_avail(imem_avail),
      .hw_ip(12'd0), .mtime(64'd0),           // device-less: no CLINT/PLIC
      .hpm_dc_access(1'b0), .hpm_dc_miss(1'b0), .hpm_ic_access(1'b0), .hpm_ic_miss(1'b0),
      .dmem_raddr(dmem_raddr), .dmem_ren(dmem_ren), .dmem_runcached(),
      .dmem_rdata(dmem_rdata), .dmem_rvalid(1'b1),
      .dmem_wen(dmem_wen), .dmem_waddr(dmem_waddr), .dmem_wdata(dmem_wdata),
      .dmem_wmask(dmem_wmask), .dmem_wuncached(),
      .dmem_cbo(), .dmem_cbo_zero(), .dmem_cbo_keep(), .dmem_wready(1'b1),
      .dmem_idle(), .ifence(),
      .ptw_addr(ptw_addr), .ptw_read(ptw_read),
      .ptw_rdata(ptw_rdata), .ptw_rvalid(ptw_rvalid),
      .dptw_addr(dptw_addr), .dptw_read(dptw_read),
      .dptw_rdata(dptw_rdata), .dptw_rvalid(dptw_rvalid),
      .retire(retire), .retire_pc(retire_pc), .retire_insn(retire_insn),
      .redirect(redirect), .redirect_target(redirect_target));

   // ---------------------------------------------------------- byte memory
   reg [7:0] mem [0:SIZE-1];
   function [63:0] rd64(input [63:0] addr);
      reg [63:0] a; integer b;
      begin a = addr - BASE;
         rd64 = 64'd0;
         for (b=0;b<8;b=b+1) rd64[b*8 +: 8] = mem[a+b];
      end
   endfunction

   // Explicit sensitivity, NOT @* -- @* over a 2M-entry array makes iverilog's
   // elaborator enumerate every word and hang. `wtick` re-evaluates after a store.
   reg wtick = 0;
   integer m;
   always @(imem_addr or wtick) begin
      for (m=0;m<HW;m=m+1) begin
         imem_data[m*16   +: 8] = mem[(imem_addr-BASE)+2*m];
         imem_data[m*16+8 +: 8] = mem[(imem_addr-BASE)+2*m+1];
      end
   end
   always @(dmem_raddr or wtick) dmem_rdata = rd64(dmem_raddr);

   // page-table walkers (instruction side + data side): registered PTE reads
   always @(posedge clk) begin
      ptw_rvalid <= ptw_read;
      if (ptw_read) ptw_rdata <= rd64({8'd0, ptw_addr});
      dptw_rvalid <= dptw_read;
      if (dptw_read) dptw_rdata <= rd64({8'd0, dptw_addr});
   end

   // -------------------------------------------------------- exit monitor
   reg [63:0] tohost;
   integer    c, b2, ncyc, trace = 0, nret = 0;
   reg        done;
   reg [8*256-1:0] hexfile;
   initial begin
      tohost = 64'h8000_1000;
      ncyc   = 200000;
      ptw_rvalid = 1'b0; dptw_rvalid = 1'b0; done = 1'b0;
      if (!$value$plusargs("hex=%s", hexfile)) begin
         $display("FATAL: need +hex=<file>"); $finish;
      end
      // zero first: an unmapped PTE then reads V=0 (clean page fault) instead of X
      for (m=0; m<SIZE; m=m+1) mem[m] = 8'd0;
      $readmemh(hexfile, mem);
      if ($value$plusargs("tohost=%h", tohost)) ;
      if ($value$plusargs("cycles=%d", ncyc)) ;

      reset = 1; @(negedge clk); @(negedge clk); reset = 0;

      if ($value$plusargs("trace=%d", trace)) ;
      if ($test$plusargs("vcd")) begin $dumpfile("/tmp/ino.vcd"); $dumpvars(0, dut); end

      // `done` ends the loop explicitly rather than relying on $finish to abort it:
      // iverilog stops on the spot, but Verilator finishes the current time slot, so
      // a bare $finish here let the loop run on and print a bogus TIMEOUT after PASS.
      for (c=0; c<ncyc && !done; c=c+1) begin
         @(negedge clk);
         if (retire) nret = nret + 1;
         if (trace) begin
            if (retire &&  dut.rf_we) $display("[%0d] R pc=%h insn=%h x%0d=%h", c, retire_pc,
                                              retire_insn, dut.rf_wa, dut.rf_wd);
            if (retire && !dut.rf_we) $display("[%0d] R pc=%h insn=%h", c, retire_pc, retire_insn);
            if (dmem_wen) $display("[%0d]   ST @%h data=%h mask=%b", c, dmem_waddr, dmem_wdata,
                                   dmem_wmask);
            if (redirect) $display("[%0d] REDIRECT -> %h", c, redirect_target);
         end
         if (dmem_wen && (dmem_waddr - (dmem_waddr%8)) == tohost && dmem_wmask[0]) begin
            if (dmem_wdata[31:0] == 32'd1) $display("RISCV-TEST PASS retires=%0d", nret);
            else $display("RISCV-TEST FAIL test=%0d (tohost=%h) retires=%0d",
                          dmem_wdata[31:1], dmem_wdata, nret);
            done = 1'b1;
         end
      end
      if (!done)
         $display("RISCV-TEST TIMEOUT after %0d cycles (pc~%h) retires=%0d", ncyc, imem_addr, nret);
      $finish;
   end

   // apply stores after the monitor has seen them
   always @(posedge clk) if (!reset && dmem_wen) begin
      for (b2=0;b2<8;b2=b2+1)
         if (dmem_wmask[b2]) mem[(dmem_waddr-BASE)+b2] <= dmem_wdata[b2*8 +: 8];
      wtick <= ~wtick;
   end
endmodule

`default_nettype wire
