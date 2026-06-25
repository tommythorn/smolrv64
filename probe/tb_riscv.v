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
   localparam        WORDS = 1<<21;            // 2 MiB: covers the -v page pool (env/v
                                                // demand-paging allocates phys pages well
                                                // past the image, e.g. ~0x8007b000)
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
      .hw_ip(12'd0),                       // no CLINT/PLIC in this device-less harness
      .dmem_raddr(dmem_raddr), .dmem_rdata(dmem_rdata), .dmem_rvalid(1'b1), .dmem_wready(1'b1),
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
      if (mmudbg && dmem_wen && dmem_waddr >= 64'h80004000 && dmem_waddr < 64'h8000a000)
         $display("[%0t] PTwrite @%h data=%h mask=%b", $time, dmem_waddr, dmem_wdata, dmem_wmask);
      stptw_rvalid <= stptw_read;
      if (stptw_read) stptw_rdata <= rd64({8'd0, stptw_addr});
   end

   // ---- exit monitor ----
   reg [63:0] tohost; integer c, b2;
   integer    ncyc;
   integer    trace=0;
   integer    storelog=0;
   integer    ncommit=0;
   integer    dlo=999999999, dhi=0;
   reg        fl2_16=1'bx;
   reg [8*256-1:0] hexfile;
   initial begin
      tohost = 64'h8000_1000;
      ncyc   = 200000;
      ptw_rvalid = 1'b0; ldptw_rvalid = 1'b0; stptw_rvalid = 1'b0;
      if (!$value$plusargs("hex=%s", hexfile)) begin
         $display("FATAL: need +hex=<file>"); $finish;
      end
      // zero memory first (real BSS/page-table RAM is zeroed): an unmapped PTE then
      // reads V=0 -> clean page fault, instead of X (which the walk can't fault on).
      for (m=0; m<SIZE; m=m+1) mem[m] = 8'd0;
      $readmemh(hexfile, mem);
      if ($value$plusargs("tohost=%h", tohost)) ;
      if ($value$plusargs("cycles=%d", ncyc)) ;

      reset=1; @(negedge clk); @(negedge clk); reset=0;

      if ($value$plusargs("trace=%d", trace)) ;
      if ($value$plusargs("storelog=%d", storelog)) ;
      if ($value$plusargs("mmudbg=%d", mmudbg)) ;
      if ($value$plusargs("dlo=%d", dlo)) ;
      if ($value$plusargs("dhi=%d", dhi)) ;
      // dump only the core (NOT tb.mem -- dumping the multi-MiB memory array OOMs).
      if ($test$plusargs("vcd")) begin $dumpfile("/tmp/dump.vcd"); $dumpvars(0, dut); end
      for (c=0; c<ncyc; c=c+1) begin
         @(negedge clk);
         if (commit) ncommit = ncommit + 1;
         if (trace && c>0 && (c % 100 == 0))
            $display("[%0d] commits=%0d pc=%h full=%b", c, ncommit, imem_addr, dut.cc_full);
         if (mmudbg && dut.lsu_dfault_v && (c % 200 == 0))
            $display("[%0d] DFAULT ckpt=%0d committed=%0d cause=%0d tval=%h fire=%b empty=%b",
                     c, dut.lsu_dfault_ckpt, dut.cc_committed, dut.lsu_dfault_cause,
                     dut.lsu_dfault_tval, dut.dflt_fire, dut.cc_empty);
         if (mmudbg && dut.roll_v)
            $display("[%0d] ROLL ckpt=%0d seq=%0d cur=%0d cmtd=%0d eb=%b dflt=%b iflt=%b full=%b",
                     c, dut.roll_ckpt, dut.roll_seq, dut.cur, dut.cc_committed,
                     dut.eb_redirect, dut.dflt_fire, dut.iflt_fire, dut.cc_full);
         if (mmudbg && dut.xtrap_v)
            $display("[%0d] XTRAP cause=%0d epc=%h tval=%h -> %h (priv %0d)",
                     c, dut.xtrap_cause, dut.xtrap_epc, dut.xtrap_tval,
                     dut.csr_redir_tgt, dut.eb.u_csr.priv);
         if (c >= dlo && c <= dhi)
            $display("[%0d] pcq=%h va=%h rdy=%b flt=%b ist=%0d vaq=%h | ebr=%b ebtgt=%h roll=%b feV=%b fePC=%h feSQ=%0d | cur=%0d cmt=%0d full=%b empt=%b acc=%b disp=%b stall=%b",
                     c, dut.fe.u_fetch.pc_q, dut.imem_va, dut.immu_ready, dut.immu_fault,
                     dut.u_immu.st, dut.u_immu.va_q, dut.eb_redirect, dut.eb_target, dut.roll_v,
                     dut.fe_red_v, dut.fe_red_pc, dut.fe_red_seq, dut.cur, dut.cc_committed,
                     dut.cc_full, dut.cc_empty, dut.accept, dut.disp_fire, dut.fe_stall);
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
   integer szp; reg [63:0] vlp;
   always @(posedge clk) if (!reset && dmem_wen) begin
      for (b2=0;b2<8;b2=b2+1)
         if (dmem_wmask[b2]) mem[(dmem_waddr-BASE)+b2] <= dmem_wdata[b2*8 +: 8];
      wtick <= ~wtick;
      // cosim store-stream log, matching simmerv's SIMMERV_STORELOG format:
      //   ST <pa,16hex> <nbytes> <value,16hex>   (mask is low-contiguous for these tests)
      if (storelog) begin
         szp = 0; for (b2=0;b2<8;b2=b2+1) szp = szp + dmem_wmask[b2];
         vlp = (szp==8) ? dmem_wdata : (dmem_wdata & ((64'd1 << (szp*8)) - 64'd1));
         // log the VIRTUAL store address (dbg_st_va) -> frame-allocation-independent, matches simmerv
         $display("ST %016x %0d %016x", dut.u_lsu.dbg_st_va, szp, vlp);
      end
   end
endmodule

`default_nettype wire
