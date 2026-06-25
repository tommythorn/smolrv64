`timescale 1ns/1ps
`default_nettype none

// Probe-level SoC harness: the sharded-OoO backend wired to a behavioral RAM AND a
// real clint.v, through an address decoder on the LSU memory port. This is the first
// SoC-integration milestone -- it lands the deferred CLINT<->LSU MMIO routing (#17)
// and the hw_ip wiring, and is the scaffold the real D$/UART/virtio plug into later.
//
//   RAM   @ 0x8000_0000 (image + data; also serves the imem window and PTW reads)
//   CLINT @ 0x0200_0000 (64-bit aligned sd/ld -> clint.we/addr/wdata/wmask, rdata)
//   clint.mtip -> hw_ip[7] (MTIP), clint.msip -> hw_ip[3] (MSIP)
//
// Loads a flat image (+hex), runs until a store to `tohost` (+tohost): 1 => PASS.
// MMIO is 64-bit-aligned only here (sub-word lane alignment lands with UART/PLIC).
module tb;
   localparam IW=4, HW=8, PCW=64, SEQW=8, PBITS=8;
   localparam [63:0] BASE      = 64'h8000_0000;
   localparam [63:0] CLINT_BASE= 64'h0200_0000;
   localparam        SIZE      = 1<<21;          // 2 MiB

   reg                 clk=0; always #5 clk=~clk;
   reg                 reset;

   wire [PCW-1:0]      imem_addr;
   reg  [HW*16-1:0]    imem_data;
   wire [3:0]          imem_avail = 4'd8;
   wire [63:0]         dmem_raddr;
   wire                dmem_ren;
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

   // ---- CLINT (fast tick for sim) + address decode ----
   wire        is_clint_r = (dmem_raddr & ~64'hffff) == CLINT_BASE;
   wire        is_clint_w = (dmem_waddr & ~64'hffff) == CLINT_BASE;
   wire [63:0] clint_rdata;
   wire        clint_mtip, clint_msip;
   clint #(.SCALE_DIV(8)) u_clint
     (.clk(clk), .reset(reset),
      .we(dmem_wen & is_clint_w),
      // shared addr: writes win (a concurrent CLINT read+write to different offsets
      // does not occur in these tests); reads otherwise drive the combinational rdata.
      .addr((dmem_wen & is_clint_w) ? dmem_waddr[15:0] : dmem_raddr[15:0]),
      .wdata(dmem_wdata), .wmask(dmem_wmask), .rdata(clint_rdata),
      .mtip(clint_mtip), .msip(clint_msip), .o_mtime());
   wire [11:0] hw_ip = (clint_mtip ? 12'h080 : 12'h000)    // MTIP = bit 7
                     | (clint_msip ? 12'h008 : 12'h000);   // MSIP = bit 3

   backend_top #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW), .PBITS(PBITS),
                 .RESET_PC(BASE)) dut
     (.clk(clk), .reset(reset),
      .imem_addr(imem_addr), .imem_data(imem_data), .imem_avail(imem_avail),
      .hw_ip(hw_ip),
      .dmem_raddr(dmem_raddr), .dmem_ren(dmem_ren),
      .dmem_rdata(dmem_rdata), .dmem_rvalid(1'b1),
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

   // ---- behavioral RAM (serves imem window, dmem RAM reads, PTW reads) ----
   reg [7:0] mem [0:SIZE-1];
   function [63:0] rd64(input [63:0] addr);
      reg [63:0] a; integer b;
      begin a = addr - BASE; rd64 = 64'd0;
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
   // dmem read mux: CLINT region -> clint.rdata, else RAM
   always @(dmem_raddr or wtick or is_clint_r or clint_rdata)
      dmem_rdata = is_clint_r ? clint_rdata : rd64(dmem_raddr);

   always @(posedge clk) begin
      ptw_rvalid <= ptw_read;     if (ptw_read)   ptw_rdata   <= rd64({8'd0, ptw_addr});
      ldptw_rvalid <= ldptw_read; if (ldptw_read) ldptw_rdata <= rd64({8'd0, ldptw_addr});
      stptw_rvalid <= stptw_read; if (stptw_read) stptw_rdata <= rd64({8'd0, stptw_addr});
   end

   // ---- exit monitor ----
   reg [63:0] tohost; integer c, b2; integer ncyc; integer ncommit=0;
   reg [8*256-1:0] hexfile;
   initial begin
      tohost = 64'h8000_1000; ncyc = 20000;
      ptw_rvalid=0; ldptw_rvalid=0; stptw_rvalid=0;
      if (!$value$plusargs("hex=%s", hexfile)) begin $display("FATAL: need +hex=<file>"); $finish; end
      for (m=0; m<SIZE; m=m+1) mem[m] = 8'd0;
      $readmemh(hexfile, mem);
      if ($value$plusargs("tohost=%h", tohost)) ;
      if ($value$plusargs("cycles=%d", ncyc)) ;
      reset=1; @(negedge clk); @(negedge clk); reset=0;
      for (c=0; c<ncyc; c=c+1) begin
         @(negedge clk);
         if (commit) ncommit = ncommit + 1;
         if (dmem_wen && (dmem_waddr - (dmem_waddr%8)) == tohost && dmem_wmask[0]) begin
            if (dmem_wdata[31:0] == 32'd1) $display("SOC-TEST PASS (commits=%0d)", ncommit);
            else $display("SOC-TEST FAIL (tohost=%h)", dmem_wdata);
            $finish;
         end
      end
      $display("SOC-TEST TIMEOUT after %0d cycles (pc~%h commits=%0d)", ncyc, imem_addr, ncommit);
      $finish;
   end

   // apply RAM stores (CLINT writes go to the device, not memory)
   always @(posedge clk) if (!reset && dmem_wen && !is_clint_w) begin
      for (b2=0;b2<8;b2=b2+1)
         if (dmem_wmask[b2]) mem[(dmem_waddr-BASE)+b2] <= dmem_wdata[b2*8 +: 8];
      wtick <= ~wtick;
   end
endmodule

`default_nettype wire
