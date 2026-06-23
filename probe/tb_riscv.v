`timescale 1ns/1ps
`default_nettype none

// riscv-tests harness for the sharded-OoO probe backend.
//
// Loads a flat little-endian image (one byte/line hex, via +hex=<file>) into a
// byte memory based at 0x80000000, serves the backend's combinational imem
// window AND the LSU's dmem port from that single memory, and watches for the
// riscv-test exit: a store to `tohost` (+tohost=<hex>). tohost==1 => PASS,
// else FAIL with test# = tohost>>1.  Times out after +cycles (default 200000).
module tb;
   localparam IW=4, HW=8, PCW=64, SEQW=8, PBITS=8;
   localparam [63:0] BASE = 64'h8000_0000;
   localparam        WORDS = 1<<18;            // 256K bytes of image space
   localparam        SIZE  = WORDS;

   reg                 clk=0; always #5 clk=~clk;
   reg                 reset;

   // ---- backend I/O ----
   wire [PCW-1:0]      imem_addr;
   reg  [HW*16-1:0]    imem_data;
   wire [3:0]          imem_avail = 4'd8;       // window always full from this TB memory
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

   // ---- unified byte memory ----
   reg [7:0] mem [0:SIZE-1];
   function [63:0] rd64(input [63:0] addr);
      reg [63:0] a; integer b;
      begin a = addr - BASE;
         rd64 = 64'd0;
         for (b=0;b<8;b=b+1) rd64[b*8 +: 8] = mem[a+b];
      end
   endfunction

   // Memory reads use EXPLICIT sensitivity (the address regs + a write tick), NOT
   // @* -- @* over this 256K-entry array makes iverilog's elaborator enumerate every
   // word into the sensitivity list and hang for minutes. wtick re-evaluates the
   // reads after a store settles (loads of just-stored data otherwise go via LSU
   // forwarding, but this keeps the memory port coherent too).
   reg wtick=0;
   integer mmudbg=0;
   integer m;
   always @(imem_addr or wtick) begin
      for (m=0;m<HW;m=m+1) begin
         imem_data[m*16 +: 8]     = mem[(imem_addr-BASE)+2*m];
         imem_data[m*16+8 +: 8]   = mem[(imem_addr-BASE)+2*m+1];
      end
   end
   always @(dmem_raddr or wtick) dmem_rdata = rd64(dmem_raddr);

   // page-table-walker ports (iMMU + load dMMU + store/amo dMMU): each a registered
   // read of a PTE from physical memory. Independent ports (walks are rare; real HW
   // would arbitrate one cache port -- not needed for the probe).
   always @(posedge clk) begin
      ptw_rvalid <= ptw_read;
      if (ptw_read) ptw_rdata <= rd64({8'd0, ptw_addr});
      if (mmudbg && ptw_read)
         $display("[%0t] iPTW addr=%h pte=%h va=%h satp=%h", $time, ptw_addr,
                  rd64({8'd0, ptw_addr}), dut.imem_va, dut.eb.u_csr.satp);
      ldptw_rvalid <= ldptw_read;
      if (ldptw_read) ldptw_rdata <= rd64({8'd0, ldptw_addr});
      stptw_rvalid <= stptw_read;
      if (stptw_read) stptw_rdata <= rd64({8'd0, stptw_addr});
   end

   // ---- exit monitor ----
   reg [63:0] tohost; integer c, b2;
   integer    ncyc;
   integer    trace=0;
   integer    ncommit=0;
   reg        fl2_16=1'bx;
   reg [8*256-1:0] hexfile;
   initial begin
      tohost = 64'h8000_1000;
      ncyc   = 200000;
      ptw_rvalid = 1'b0; ldptw_rvalid = 1'b0; stptw_rvalid = 1'b0;
      if (!$value$plusargs("hex=%s", hexfile)) begin
         $display("FATAL: need +hex=<file>"); $finish;
      end
      $readmemh(hexfile, mem);
      if ($value$plusargs("tohost=%h", tohost)) ;
      if ($value$plusargs("cycles=%d", ncyc)) ;

      reset=1; @(negedge clk); @(negedge clk); reset=0;

      if ($value$plusargs("trace=%d", trace)) ;
      if ($value$plusargs("mmudbg=%d", mmudbg)) ;
      for (c=0; c<ncyc; c=c+1) begin
         @(negedge clk);
         if (commit) ncommit = ncommit + 1;
         if (trace && c>0 && (c % 100 == 0))
            $display("[%0d] commits=%0d pc=%h full=%b", c, ncommit, imem_addr, dut.cc_full);
         if (trace) begin
            if (dut.eb_redirect)
               $display("[%0d] REDIRECT -> %h (seq %0d)", c, dut.eb_target, dut.eb_rseq);
            for (b2=0;b2<IW;b2=b2+1)
               if (dut.wb_valid[b2])
                  $display("[%0d]   WB lane%0d pr=%0d val=%h", c, b2,
                           dut.wb_pr[b2*PBITS+:PBITS], dut.wb_val[b2*64+:64]);
            if (dmem_wen)
               $display("[%0d]   STORE @%h data=%h mask=%b", c, dmem_waddr, dmem_wdata, dmem_wmask);
         end
         // tohost store?
         if (dmem_wen && (dmem_waddr - (dmem_waddr%8)) == tohost && dmem_wmask[0]) begin
            if (dmem_wdata[31:0] == 32'd1)
               $display("RISCV-TEST PASS");
            else
               $display("RISCV-TEST FAIL test=%0d (tohost=%h)",
                        dmem_wdata[31:1], dmem_wdata);
            $finish;
         end
      end
      $display("RISCV-TEST TIMEOUT after %0d cycles (pc~%h) commits=%0d", ncyc, imem_addr, ncommit);
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
