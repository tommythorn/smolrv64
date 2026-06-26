`timescale 1ns/1ps
`default_nettype none

// Fetch + iMMU page-cross safety test.
//
// The concern: a *compressed* (16-bit) instruction sitting in the last two bytes
// of a mapped page, with the NEXT page UNMAPPED, must NOT raise a fetch page
// fault. The RVC is complete within the mapped page; the unmapped page beyond it
// is irrelevant until execution actually advances there.
//
// Why the core is safe by construction: the iMMU translates exactly ONE address
// per fetch -- req_vaddr = the fetch PC (backend_top: imem_va = pc_q). It never
// speculatively translates PC+2 or the next page. So an RVC whose PC is in the
// mapped page only ever triggers a translation of THAT page -> no fault. The
// fault for the unmapped page appears only once the PC itself advances into it
// (the genuinely-next instruction), where it is correct (and squashed by a
// rollback if a branch/jump redirects away first).
//
// This test wires the real `fetch` to the real `mmu` over a behavioral Sv39 page
// table (page 0x1000 mapped -> 0x80003000; page 0x2000 unmapped, identical to
// tb_mmu) and a behavioral imem holding a C.NOP at VA 0x1FFE (the last halfword
// of the mapped page). It asserts: (1) PC=0x1FFE fetches fault-free and aligns
// the RVC; (2) PC=0x2000 (the unmapped page) fetch-faults (cause 12) only when
// the PC actually targets it.
module tb;
   localparam IW=4, HW=8, PCW=64, SEQW=8, AW=56;
   reg clk=0; always #5 clk=~clk;
   reg reset;
   integer errs=0;

   // fetch control / outputs
   reg               redirect; reg [PCW-1:0] redirect_pc; reg [SEQW-1:0] redirect_seq;
   reg               ready;
   wire              f_valid;
   wire [IW-1:0]     slot_valid;
   wire [IW*32-1:0]  f_inst;
   wire [IW*PCW-1:0] f_pc;
   wire [IW*SEQW-1:0] f_seq;
   wire [SEQW-1:0]   cur_seq;

   // fetch <-> iMMU <-> behavioral imem
   wire [PCW-1:0]    imem_va;            // fetch's VA out (= pc_q)
   wire [AW-1:0]     immu_pa;
   wire              immu_ready, immu_fault;
   wire [3:0]        immu_cause;
   reg  [63:0]       satp; reg [1:0] priv;

   // behavioral imem: full window claimed available (like soc_top), but gated to 0
   // while the translation is not ready / faulting -- exactly the backend wiring.
   reg  [HW*16-1:0]  imem_data;
   wire [$clog2(HW+2)-1:0] imem_avail_g = (immu_ready & ~immu_fault) ? 4'd8 : 4'd0;

   // PTW port
   wire [AW-1:0]     ptw_addr; wire ptw_read;
   reg  [63:0]       ptw_rdata; reg ptw_rvalid;

   fetch #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW)) u_fetch
     (.clk(clk), .reset(reset),
      .redirect(redirect), .redirect_pc(redirect_pc), .redirect_seq(redirect_seq),
      .solo_all(1'b0), .irq_inject(1'b0),
      .imem_addr(imem_va), .imem_data(imem_data), .imem_avail(imem_avail_g),
      .ready(ready), .valid(f_valid), .slot_valid(slot_valid),
      .inst(f_inst), .pc(f_pc), .seq(f_seq), .cur_seq(cur_seq));

   mmu #(.AW(AW)) u_immu
     (.clk(clk), .reset(reset), .req_valid(1'b1), .req_vaddr(imem_va), .req_access(2'd0),
      .priv(priv), .sum(1'b0), .mxr(1'b0), .satp(satp), .flush(1'b0),
      .ptw_addr(ptw_addr), .ptw_read(ptw_read), .ptw_rdata(ptw_rdata), .ptw_rvalid(ptw_rvalid),
      .t_ready(immu_ready), .t_paddr(immu_pa), .t_fault(immu_fault), .t_cause(immu_cause));

   // behavioral 3-level page table (registered read): VA 0x1000 page -> PA 0x80003000
   // (RWX U A D), VA 0x2000 page UNMAPPED. (Same table as tb_mmu.)
   always @(posedge clk) begin
      ptw_rvalid <= ptw_read;
      if (ptw_read) case (ptw_addr)
         56'h80010000: ptw_rdata <= (64'h80011 << 10) | 64'd1;     // root[0] -> L1 table
         56'h80011000: ptw_rdata <= (64'h80012 << 10) | 64'd1;     // L1[0]   -> L0 table
         56'h80012008: ptw_rdata <= (64'h80003 << 10) | 64'hDF;    // L0[1]   leaf RWX U A D
         default:      ptw_rdata <= 64'd0;                         // invalid (V=0) -> page fault
      endcase
   end

   // behavioral imem: a C.NOP (0x0001, an RVC -> low 2 bits != 11) at PA 0x80003FFE,
   // i.e. VA 0x1FFE, the LAST halfword of the mapped page. Window starts at the PA.
   localparam [15:0] RVC = 16'h0001;
   integer k;
   reg seen_fault2;
   always @* begin
      imem_data = {(HW*16){1'b0}};
      for (k = 0; k < HW; k = k + 1)
         if ((immu_pa + 2*k) == 56'h80003FFE) imem_data[k*16 +: 16] = RVC;
   end

   task wait_xlate;  // hold PC, let the walk resolve (or fault)
      integer n;
      begin n=0; while (!immu_ready && n<40) begin @(negedge clk); #1; n=n+1; end end
   endtask

   initial begin
      reset=1; redirect=0; redirect_pc=0; redirect_seq=0; ready=0; satp=0; priv=0; ptw_rvalid=0;
      @(negedge clk); @(negedge clk); reset=0; @(negedge clk);
      satp = (64'd8 << 60) | 64'h80010;          // Sv39, root PPN 0x80010
      priv = 2'd0;                                // U-mode (the leaf is U=1, X=1)

      // ---- 1) RVC at the last halfword of the mapped page: must NOT fault ----
      @(negedge clk); redirect=1; redirect_pc=64'h1FFE; redirect_seq=0; @(negedge clk); redirect=0;
      ready=0;                                    // hold PC at 0x1FFE
      wait_xlate;
      // observe a few settled cycles
      repeat (3) begin
         @(negedge clk); #1;
         if (imem_va==64'h1FFE && immu_fault) begin
            $display("FAIL: RVC at page-end VA 1FFE raised a fetch fault (cause %0d)", immu_cause);
            errs=errs+1;
         end
      end
      if (immu_fault) begin
         $display("FAIL: page-end RVC fetch faulted (cause %0d)", immu_cause); errs=errs+1;
      end else if (!slot_valid[0]) begin
         $display("FAIL: RVC did not align (slot_valid=%b, ready=%b fault=%b)", slot_valid, immu_ready, immu_fault);
         errs=errs+1;
      end else begin
         $display("  ok: VA 1FFE RVC fetched fault-free -> PA %h, slot0 inst=%h", immu_pa, f_inst[31:0]);
      end

      // ---- 2) the unmapped next page faults (cause 12) only when PC targets it ----
      @(negedge clk); redirect=1; redirect_pc=64'h2000; redirect_seq=1; @(negedge clk); redirect=0;
      ready=0; seen_fault2=0;
      repeat (40) begin
         @(negedge clk); #1;
         if (imem_va==64'h2000 && immu_fault && immu_cause==4'd12) seen_fault2=1;
      end
      if (!seen_fault2) begin
         $display("FAIL: unmapped VA 2000 never fetch-faulted (last: va=%h ready=%b fault=%b cause=%0d)",
                  imem_va, immu_ready, immu_fault, immu_cause);
         errs=errs+1;
      end else begin
         $display("  ok: VA 2000 (unmapped) fetch-faults cause 12 only once PC reaches it");
      end

      if (errs==0) $display("fetch-pagecross: ALL TESTS PASSED");
      else         $display("fetch-pagecross: %0d FAILURES", errs);
      $finish;
   end

   initial begin #80000; $display("fetch-pagecross: TIMEOUT"); $finish; end
endmodule

`default_nettype wire
