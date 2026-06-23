`timescale 1ns/1ps
`default_nettype none

// Unit test for the standalone Sv39 MMU. A behavioral 3-level page table maps
//   VA 0x1000 -> PA 0x80003000 (4 KiB, perms RWX U A D)
// and leaves VA 0x2000 unmapped (page fault). Also checks a Bare-mode identity.
module tb;
   localparam AW=56;
   reg clk=0; always #5 clk=~clk;
   reg reset;
   reg        req_valid; reg [63:0] req_vaddr; reg [1:0] req_access, priv;
   reg [63:0] satp; reg sum, mxr, flush;
   wire [AW-1:0] ptw_addr; wire ptw_read;
   reg  [63:0] ptw_rdata; reg ptw_rvalid;
   wire done; wire [AW-1:0] paddr; wire fault; wire [3:0] cause;
   integer errs=0;

   mmu #(.AW(AW)) dut
     (.clk(clk), .reset(reset), .req_valid(req_valid), .req_vaddr(req_vaddr),
      .req_access(req_access), .priv(priv), .sum(sum), .mxr(mxr), .satp(satp),
      .flush(flush), .ptw_addr(ptw_addr), .ptw_read(ptw_read), .ptw_rdata(ptw_rdata),
      .ptw_rvalid(ptw_rvalid), .done(done), .paddr(paddr), .fault(fault), .cause(cause));

   // behavioral page table (registered read, 1-cycle latency)
   always @(posedge clk) begin
      ptw_rvalid <= ptw_read;
      if (ptw_read) case (ptw_addr)
         56'h80010000: ptw_rdata <= (64'h80011 << 10) | 64'd1;          // root[0] -> L1 table
         56'h80011000: ptw_rdata <= (64'h80012 << 10) | 64'd1;          // L1[0]   -> L0 table
         56'h80012008: ptw_rdata <= (64'h80003 << 10) | 64'hDF;         // L0[1]   leaf RWX U A D
         default:      ptw_rdata <= 64'd0;                              // invalid (V=0)
      endcase
   end

   task do_req(input [63:0] va, input [1:0] acc);
      begin
         @(negedge clk); req_valid=1; req_vaddr=va; req_access=acc;
         while (!done) @(negedge clk);
         req_valid=0;
      end
   endtask

   initial begin
      reset=1; req_valid=0; satp=0; priv=0; sum=0; mxr=0; flush=0; req_access=1;
      ptw_rvalid=0; @(negedge clk); @(negedge clk); reset=0; @(negedge clk);

      // Sv39 on, root PPN = 0x80010
      satp = (64'd8 << 60) | 64'h80010;

      // 1) VA 0x1000 load -> PA 0x80003000, no fault
      do_req(64'h1000, 2'd1);
      if (fault || paddr !== 56'h80003000) begin
         $display("FAIL xlate: fault=%b paddr=%h (exp 80003000)", fault, paddr); errs=errs+1; end
      else $display("  ok: VA 1000 -> PA %h", paddr);

      // 2) VA 0x2000 load -> page fault (cause 13)
      do_req(64'h2000, 2'd1);
      if (!fault || cause !== 4'd13) begin
         $display("FAIL fault: fault=%b cause=%0d (exp 1,13)", fault, cause); errs=errs+1; end
      else $display("  ok: VA 2000 page-faults (cause %0d)", cause);

      // 3) the cached translation hits the TLB this time (still correct)
      do_req(64'h1000, 2'd1);
      if (fault || paddr !== 56'h80003000) begin
         $display("FAIL tlb-hit: fault=%b paddr=%h", fault, paddr); errs=errs+1; end
      else $display("  ok: VA 1000 TLB-hit -> PA %h", paddr);

      // 4) store to the RWX page is allowed (W=1, D=1)
      do_req(64'h1000, 2'd2);
      if (fault) begin $display("FAIL store-perm: unexpected fault"); errs=errs+1; end
      else $display("  ok: store to RWX page allowed");

      // 5) Bare mode -> identity
      satp = 0; flush=1; @(negedge clk); flush=0;
      do_req(64'h80007abc, 2'd1);
      if (fault || paddr !== 56'h80007abc) begin
         $display("FAIL bare: fault=%b paddr=%h", fault, paddr); errs=errs+1; end
      else $display("  ok: Bare identity %h", paddr);

      if (errs==0) $display("mmu: ALL TESTS PASSED");
      else         $display("mmu: %0d FAILURES", errs);
      $finish;
   end

   initial begin #20000; $display("mmu: TIMEOUT"); $finish; end
endmodule

`default_nettype wire
