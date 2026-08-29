`timescale 1ns/1ps
`default_nettype none

// Unit test for the standalone Sv39 MMU (combinational-hit interface). A behavioral
// 3-level page table maps VA 0x1000 -> PA 0x80003000 (4 KiB, perms RWX U A D) and
// leaves VA 0x2000 unmapped (page fault). Checks: walk fills + resolves, miss vs.
// hit latency (a hit resolves combinationally with t_ready=1), store permission,
// page fault, and Bare-mode identity.
module tb;
   localparam AW=56;
   reg clk=0; always #5 clk=~clk;
   reg reset;
   reg        req_valid; reg [63:0] req_vaddr; reg [1:0] req_access, priv;
   reg [63:0] satp; reg sum, mxr, flush;
   wire [AW-1:0] ptw_addr; wire ptw_read;
   reg  [63:0] ptw_rdata; reg ptw_rvalid;
   wire t_ready; wire [AW-1:0] t_paddr; wire t_fault; wire [3:0] t_cause; wire t_uncached;
   integer errs=0;

   mmu #(.AW(AW)) dut
     (.clk(clk), .reset(reset), .req_valid(req_valid), .req_vaddr(req_vaddr),
      .req_access(req_access), .priv(priv), .sum(sum), .mxr(mxr), .satp(satp),
      .flush(flush), .ptw_addr(ptw_addr), .ptw_read(ptw_read), .ptw_rdata(ptw_rdata),
      .ptw_rvalid(ptw_rvalid), .walking(), .t_ready(t_ready), .t_paddr(t_paddr),
      .t_fault(t_fault), .t_cause(t_cause), .t_uncached(t_uncached));

   // behavioral page table (registered read, 1-cycle latency)
   always @(posedge clk) begin
      ptw_rvalid <= ptw_read;
      if (ptw_read) case (ptw_addr)
         56'h80010000: ptw_rdata <= (64'h80011 << 10) | 64'd1;          // root[0] -> L1 table
         56'h80011000: ptw_rdata <= (64'h80012 << 10) | 64'd1;          // L1[0]   -> L0 table
         56'h80012008: ptw_rdata <= (64'h80003 << 10) | 64'hDF;         // L0[1]   leaf RWX U A D
         56'h80012018: ptw_rdata <= (64'h80005 << 10) | 64'hDF | (64'd1<<61); // L0[3] leaf, Svpbmt NC
         default:      ptw_rdata <= 64'd0;                              // invalid (V=0)
      endcase
   end

   integer waited;
   // drive a request, stall until t_ready; report cycles spent walking
   task do_req(input [63:0] va, input [1:0] acc);
      begin
         @(negedge clk); req_valid=1; req_vaddr=va; req_access=acc;
         #1;   // settle combinational t_ready before sampling
         waited=0;
         while (!t_ready) begin @(negedge clk); #1; waited=waited+1; end
      end
   endtask

   initial begin
      reset=1; req_valid=0; satp=0; priv=0; sum=0; mxr=0; flush=0; req_access=1;
      ptw_rvalid=0; @(negedge clk); @(negedge clk); reset=0; @(negedge clk);

      // Sv39 on, root PPN = 0x80010
      satp = (64'd8 << 60) | 64'h80010;

      // 1) VA 0x1000 load -> PA 0x80003000, no fault (TLB miss -> walk); cacheable (PBMT=0)
      do_req(64'h1000, 2'd1);
      if (t_fault || t_paddr !== 56'h80003000 || t_uncached) begin
         $display("FAIL xlate: fault=%b paddr=%h unc=%b (exp 80003000,0)", t_fault, t_paddr, t_uncached); errs=errs+1; end
      else if (waited==0) begin
         $display("FAIL xlate: expected a walk (waited=0)"); errs=errs+1; end
      else $display("  ok: VA 1000 -> PA %h (walk took %0d cyc)", t_paddr, waited);
      req_valid=0;

      // 2) VA 0x1000 again -> TLB HIT, resolves combinationally (waited==0)
      do_req(64'h1000, 2'd1);
      if (t_fault || t_paddr !== 56'h80003000 || waited!=0) begin
         $display("FAIL tlb-hit: fault=%b paddr=%h waited=%0d", t_fault, t_paddr, waited); errs=errs+1; end
      else $display("  ok: VA 1000 TLB-hit combinational -> PA %h", t_paddr);
      req_valid=0;

      // 3) VA 0x2000 load -> page fault (cause 13)
      do_req(64'h2000, 2'd1);
      if (!t_fault || t_cause !== 4'd13) begin
         $display("FAIL fault: fault=%b cause=%0d (exp 1,13)", t_fault, t_cause); errs=errs+1; end
      else $display("  ok: VA 2000 page-faults (cause %0d)", t_cause);
      req_valid=0;

      // 4) store to the RWX page is allowed (W=1, D=1) -- TLB hit
      do_req(64'h1000, 2'd2);
      if (t_fault) begin $display("FAIL store-perm: unexpected fault"); errs=errs+1; end
      else $display("  ok: store to RWX page allowed");
      req_valid=0;

      // 4b) Svpbmt: VA 0x3000 -> NC leaf (PBMT=01) -> t_uncached=1, no fault, PA 0x80005000
      do_req(64'h3000, 2'd1);
      if (t_fault || t_paddr !== 56'h80005000 || !t_uncached) begin
         $display("FAIL NC leaf: fault=%b paddr=%h unc=%b (exp 80005000,1)", t_fault, t_paddr, t_uncached); errs=errs+1; end
      else $display("  ok: VA 3000 NC leaf -> PA %h uncached=%b", t_paddr, t_uncached);
      req_valid=0;

      // 5) Bare mode -> identity, resolves combinationally
      satp = 0; flush=1; @(negedge clk); flush=0;
      do_req(64'h80007abc, 2'd1);
      if (t_fault || t_paddr !== 56'h80007abc || waited!=0) begin
         $display("FAIL bare: fault=%b paddr=%h waited=%0d", t_fault, t_paddr, waited); errs=errs+1; end
      else $display("  ok: Bare identity %h", t_paddr);
      req_valid=0;

      if (errs==0) $display("mmu: ALL TESTS PASSED");
      else         $display("mmu: %0d FAILURES", errs);
      $finish;
   end

   initial begin #20000; $display("mmu: TIMEOUT"); $finish; end
endmodule

`default_nettype wire
