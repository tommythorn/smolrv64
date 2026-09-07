`timescale 1ns/1ps
`default_nettype none

// Fetch + iMMU page-cross test. Drives the real `fetch` against the real `mmu`
// over a behavioral Sv39 table, covering the three page-boundary cases:
//
//   A) COMPRESSED op in a page's last 2 bytes, next page UNMAPPED -> NO fault
//      (complete in the mapped page; the fetch never translates the next page).
//   B) 32-bit op STRADDLING into a MAPPED but NON-CONTIGUOUS next page -> the
//      fetch lazily translates PC+2 and emits the correct {hi,lo} combined word
//      (proving it uses the real translation, not contiguous physical bytes).
//   C) 32-bit op STRADDLING into an UNMAPPED next page -> a PRECISE fetch fault:
//      epc = the instruction PC (imem_ipc = pc_q), tval = the faulting VA
//      (imem_addr = pc_q+2), cause 12 -- matching simmerv's memop_code.
//
// Page table (all VAs share VPN2=VPN1=0, so leaves live in one L0 table):
//   L0[1] VA 0x1000 -> PA 0x80003000   (A: RVC at 0x1FFE; next page 0x2000 unmapped)
//   L0[2] VA 0x2000 -> unmapped
//   L0[3] VA 0x3000 -> PA 0x80005000   (C: 32-bit lo at 0x3FFE; next page 0x4000 unmapped)
//   L0[4] VA 0x4000 -> unmapped
//   L0[5] VA 0x5000 -> PA 0x80007000   (B: 32-bit lo at 0x5FFE)
//   L0[6] VA 0x6000 -> PA 0x80009000   (B: 32-bit hi -- NON-contiguous w/ 0x80007xxx)
module tb;
   localparam IW=4, HW=8, PCW=64, SEQW=8, AW=56;
   reg clk=0; always #5 clk=~clk;
   reg reset;
   integer errs=0;

   reg               redirect; reg [PCW-1:0] redirect_pc; reg [SEQW-1:0] redirect_seq;
   reg               ready;
   wire              f_valid;
   wire [IW-1:0]     slot_valid;
   wire [IW*32-1:0]  f_inst;
   wire [IW*PCW-1:0] f_pc;
   wire [IW*SEQW-1:0] f_seq;
   wire [SEQW-1:0]   cur_seq;

   wire [PCW-1:0]    f_imem_va;          // fetch's translate VA (pc_q, or pc_q+2 in straddle)
   wire [PCW-1:0]    f_imem_ipc;         // fetch's instruction PC (fault epc)
   wire [AW-1:0]     immu_pa;
   wire              immu_ready, immu_fault;
   wire [3:0]        immu_cause;
   reg  [63:0]       satp; reg [1:0] priv;

   reg  [HW*16-1:0]  imem_data;
   wire [$clog2(HW+2)-1:0] imem_avail_g = (immu_ready & ~immu_fault) ? 4'd8 : 4'd0;

   wire [AW-1:0]     ptw_addr; wire ptw_read;
   reg  [63:0]       ptw_rdata; reg ptw_rvalid;

   fetch #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW)) u_fetch
     (.clk(clk), .reset(reset),
      .redirect(redirect), .redirect_pc(redirect_pc), .redirect_seq(redirect_seq),
      .solo_all(1'b0), .irq_inject(1'b0),
      .pred_v(1'b0), .apred_v(1'b0), .pred_tgt(64'd0),
      .imem_addr(f_imem_va), .imem_ipc(f_imem_ipc),
      .imem_data(imem_data), .imem_avail(imem_avail_g), .imem_ok(1'b1),
      .ready(ready), .valid(f_valid), .slot_valid(slot_valid),
      .inst(f_inst), .pc(f_pc), .seq(f_seq), .cur_seq(cur_seq));

   mmu #(.AW(AW)) u_immu
     (.clk(clk), .reset(reset), .req_valid(1'b1), .req_vaddr(f_imem_va), .req_access(2'd0),
      .priv(priv), .sum(1'b0), .mxr(1'b0), .satp(satp), .flush(1'b0),
      .ptw_addr(ptw_addr), .ptw_read(ptw_read), .ptw_rdata(ptw_rdata), .ptw_rvalid(ptw_rvalid),
      .walking(), .t_ready(immu_ready), .t_paddr(immu_pa), .t_fault(immu_fault), .t_cause(immu_cause));

   // behavioral 3-level page table (registered read)
   always @(posedge clk) begin
      ptw_rvalid <= ptw_read;
      if (ptw_read) case (ptw_addr)
         56'h80010000: ptw_rdata <= (64'h80011 << 10) | 64'd1;     // root[0] -> L1
         56'h80011000: ptw_rdata <= (64'h80012 << 10) | 64'd1;     // L1[0]   -> L0
         56'h80012008: ptw_rdata <= (64'h80003 << 10) | 64'hDF;    // L0[1] VA 0x1000 leaf
         56'h80012018: ptw_rdata <= (64'h80005 << 10) | 64'hDF;    // L0[3] VA 0x3000 leaf
         56'h80012028: ptw_rdata <= (64'h80007 << 10) | 64'hDF;    // L0[5] VA 0x5000 leaf
         56'h80012030: ptw_rdata <= (64'h80009 << 10) | 64'hDF;    // L0[6] VA 0x6000 leaf
         default:      ptw_rdata <= 64'd0;                         // invalid -> page fault
      endcase
   end

   // behavioral imem: halfword at a PA. C.NOP (RVC) at 0x80003FFE; 32-bit lo halves
   // (low2==11) at 0x80005FFE and 0x80007FFE; the case-B hi half at 0x80009000.
   integer k;
   always @* begin
      imem_data = {(HW*16){1'b0}};
      for (k = 0; k < HW; k = k + 1) begin
         if ((immu_pa + 2*k) == 56'h80003FFE) imem_data[k*16 +: 16] = 16'h0001;  // A: c.nop
         if ((immu_pa + 2*k) == 56'h80005FFE) imem_data[k*16 +: 16] = 16'h0013;  // C: lo (32-bit)
         if ((immu_pa + 2*k) == 56'h80007FFE) imem_data[k*16 +: 16] = 16'hB0B7;  // B: lo (32-bit)
         if ((immu_pa + 2*k) == 56'h80009000) imem_data[k*16 +: 16] = 16'hDEAD;  // B: hi
      end
   end

   reg seen; reg [31:0] got;
   task settle; integer n; begin n=0; while (n<40) begin @(negedge clk); #1; n=n+1; end end endtask

   initial begin
      reset=1; redirect=0; redirect_pc=0; redirect_seq=0; ready=0; satp=0; priv=0; ptw_rvalid=0;
      @(negedge clk); @(negedge clk); reset=0; @(negedge clk);
      satp = (64'd8 << 60) | 64'h80010; priv = 2'd0;

      // ---- A) RVC at page end, next page unmapped: no fault ----
      @(negedge clk); redirect=1; redirect_pc=64'h1FFE; redirect_seq=0; @(negedge clk); redirect=0;
      ready=0; seen=0;
      repeat (40) begin @(negedge clk); #1;
         if (f_imem_va==64'h1FFE && immu_fault) seen=1; end  // any fault while fetching the RVC = bug
      if (seen)                  begin $display("FAIL A: RVC at 0x1FFE raised a fetch fault"); errs=errs+1; end
      else if (!slot_valid[0])   begin $display("FAIL A: RVC did not align (sv=%b ready=%b)", slot_valid, immu_ready); errs=errs+1; end
      else $display("  okA: RVC at 0x1FFE fetched fault-free (PA %h)", immu_pa);

      // ---- B) 32-bit op straddling into a MAPPED non-contiguous page: correct combine ----
      @(negedge clk); redirect=1; redirect_pc=64'h5FFE; redirect_seq=0; @(negedge clk); redirect=0;
      ready=0; seen=0; got=0;
      repeat (40) begin @(negedge clk); #1;
         if (slot_valid[0] && !immu_fault && f_pc[PCW-1:0]==64'h5FFE) begin seen=1; got=f_inst[31:0]; end end
      if (!seen)                 begin $display("FAIL B: straddler never emitted (sv=%b fault=%b va=%h)", slot_valid, immu_fault, f_imem_va); errs=errs+1; end
      else if (got!==32'hDEADB0B7) begin $display("FAIL B: combined inst=%h exp DEADB0B7", got); errs=errs+1; end
      else $display("  okB: 32-bit straddler -> %h (hi from non-contiguous page 0x80009000)", got);

      // ---- C) 32-bit op straddling into an UNMAPPED page: precise fault ----
      @(negedge clk); redirect=1; redirect_pc=64'h3FFE; redirect_seq=0; @(negedge clk); redirect=0;
      ready=0; seen=0;
      repeat (40) begin @(negedge clk); #1;
         // precise: epc = instruction PC (0x3FFE), tval = faulting VA (0x4000), cause 12
         if (immu_fault && immu_cause==4'd12 && f_imem_va==64'h4000 && f_imem_ipc==64'h3FFE) seen=1; end
      if (!seen) begin
         $display("FAIL C: straddle into unmapped page not precise (fault=%b cause=%0d va=%h ipc=%h)",
                  immu_fault, immu_cause, f_imem_va, f_imem_ipc);
         errs=errs+1;
      end else $display("  okC: straddle-into-unmapped faults cause 12, epc=0x3FFE tval=0x4000");

      if (errs==0) $display("fetch-pagecross: ALL TESTS PASSED");
      else         $display("fetch-pagecross: %0d FAILURES", errs);
      $finish;
   end

   initial begin #200000; $display("fetch-pagecross: TIMEOUT"); $finish; end
endmodule

`default_nettype wire
