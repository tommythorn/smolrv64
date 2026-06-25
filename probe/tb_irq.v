`timescale 1ns/1ps
`default_nettype none

// End-to-end interrupt-delivery test for the hw_ip port: holds MTIP (hw_ip[7])
// high and loads a tiny M-mode program (+hex) that sets mtvec, enables
// mie.MTIE + mstatus.MIE, then spins. The core must take a machine timer
// interrupt (mcause = (1<<63)|7) and run the handler, which stores 1 to tohost.
// PASS = a store of 1 to +tohost; FAIL = wrong cause / timeout. Bare mode (no PTW).
module tb;
   localparam IW=4, HW=8, PCW=64, SEQW=8, PBITS=8;
   localparam [63:0] BASE = 64'h8000_0000;
   localparam        SIZE = 1<<20;

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
   wire [IW-1:0]       wb_valid;
   wire [IW*PBITS-1:0] wb_pr;
   wire [IW*64-1:0]    wb_val;
   wire                redirect, commit;
   wire [PCW-1:0]      redirect_target;

   // hold MTIP asserted; the core takes it once the program enables interrupts.
   wire [11:0]         hw_ip = 12'h080;       // bit 7 = MTIP

   backend_top #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW), .PBITS(PBITS),
                 .RESET_PC(BASE)) dut
     (.clk(clk), .reset(reset),
      .imem_addr(imem_addr), .imem_data(imem_data), .imem_avail(imem_avail),
      .hw_ip(hw_ip),
      .dmem_raddr(dmem_raddr), .dmem_rdata(dmem_rdata), .dmem_rvalid(1'b1), .dmem_wready(1'b1),
      .dmem_wen(dmem_wen), .dmem_waddr(dmem_waddr), .dmem_wdata(dmem_wdata),
      .dmem_wmask(dmem_wmask),
      .ptw_addr(ptw_addr), .ptw_read(ptw_read), .ptw_rdata(64'd0), .ptw_rvalid(1'b0),
      .ldptw_addr(ldptw_addr), .ldptw_read(ldptw_read), .ldptw_rdata(64'd0), .ldptw_rvalid(1'b0),
      .stptw_addr(stptw_addr), .stptw_read(stptw_read), .stptw_rdata(64'd0), .stptw_rvalid(1'b0),
      .wb_valid(wb_valid), .wb_pr(wb_pr), .wb_val(wb_val),
      .redirect(redirect), .redirect_target(redirect_target),
      .commit(commit), .commit_idx());

   reg [7:0] mem [0:SIZE-1];
   integer m;
   reg wtick=0;
   always @(imem_addr or wtick)
      for (m=0;m<HW;m=m+1) begin
         imem_data[m*16 +: 8]   = mem[(imem_addr-BASE)+2*m];
         imem_data[m*16+8 +: 8] = mem[(imem_addr-BASE)+2*m+1];
      end
   integer b;
   always @(dmem_raddr or wtick) begin
      dmem_rdata = 64'd0;
      for (b=0;b<8;b=b+1) dmem_rdata[b*8 +: 8] = mem[(dmem_raddr-BASE)+b];
   end
   always @(posedge clk) if (!reset && dmem_wen) begin
      for (b=0;b<8;b=b+1) if (dmem_wmask[b]) mem[(dmem_waddr-BASE)+b] <= dmem_wdata[b*8 +: 8];
      wtick <= ~wtick;
   end

   reg [63:0] tohost; integer c, ncyc;
   reg [8*256-1:0] hexfile;
   initial begin
      tohost = 64'h8000_0078; ncyc = 20000;
      if (!$value$plusargs("hex=%s", hexfile)) begin $display("FATAL need +hex"); $finish; end
      for (m=0;m<SIZE;m=m+1) mem[m]=8'd0;
      $readmemh(hexfile, mem);
      if ($value$plusargs("tohost=%h", tohost)) ;
      if ($value$plusargs("cycles=%d", ncyc)) ;
      reset=1; @(negedge clk); @(negedge clk); reset=0;
      for (c=0;c<ncyc;c=c+1) begin
         @(negedge clk);
         if (dmem_wen && (dmem_waddr-(dmem_waddr%8))==tohost && dmem_wmask[0]) begin
            if (dmem_wdata[31:0]==32'd1) $display("IRQ TEST PASS (mtimer delivered, mcause checked)");
            else $display("IRQ TEST FAIL wrong-cause tohost=%h", dmem_wdata);
            $finish;
         end
      end
      $display("IRQ TEST FAIL TIMEOUT after %0d cycles (pc~%h)", ncyc, imem_addr);
      $finish;
   end
endmodule

`default_nettype wire
