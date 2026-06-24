`timescale 1ns/1ps
`default_nettype none

// End-to-end load/store through the full core (fetch->decode->rename->sched->
// exec/AGU->LSU->writeback), with a flat data-memory stub. Program:
//   addi a0,x0,256      ; base address
//   addi a1,x0,0xAB     ; value
//   sd   a1,0(a0)       ; store (uncommitted)
//   ld   a2,0(a0)       ; load  -> must FORWARD a1 from the store buffer (0xAB)
//   addi a3,a2,1        ; a3 = a2+1 = 0xAC  (proves the load value reached a consumer)
// Checks: writeback stream contains 256, 0xAB, 0xAC; data memory holds 0xAB at
// byte 256 after the store commits and drains. If forwarding failed the load would
// read the memory sentinel and a3 would not be 0xAC.
module tb;
   localparam IW=4, HW=8, PCW=64, SEQW=8, PBITS=8, PBW=4, AW=64;

   reg                clk=0; always #5 clk=~clk;
   reg                reset;
   wire               redirect; wire [PCW-1:0] redirect_target;
   wire [PCW-1:0]     imem_addr;
   reg  [HW*16-1:0]   imem_data;
   wire [PBW-1:0]     imem_avail = 4'd8;
   wire [AW-1:0]      dmem_raddr; wire [63:0] dmem_rdata;
   wire               dmem_wen;   wire [AW-1:0] dmem_waddr;
   wire [63:0]        dmem_wdata; wire [7:0]    dmem_wmask;
   wire [IW-1:0]      wb_valid; wire [IW*PBITS-1:0] wb_pr; wire [IW*64-1:0] wb_val;
   wire               commit; wire [1:0] commit_idx;
   integer errs=0, i, k, c;

   backend_top #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW), .PBITS(PBITS), .RESET_PC(0)) dut
     (.clk(clk), .reset(reset), .imem_addr(imem_addr), .imem_data(imem_data), .imem_avail(imem_avail), .hw_ip(12'd0),
      .dmem_raddr(dmem_raddr), .dmem_rdata(dmem_rdata), .dmem_wen(dmem_wen),
      .dmem_waddr(dmem_waddr), .dmem_wdata(dmem_wdata), .dmem_wmask(dmem_wmask),
      .wb_valid(wb_valid), .wb_pr(wb_pr), .wb_val(wb_val),
      .redirect(redirect), .redirect_target(redirect_target),
      .commit(commit), .commit_idx(commit_idx));

   // imem
   reg [15:0] imem [0:63];
   integer m;
   always @(*) for (m=0;m<HW;m=m+1) imem_data[m*16 +: 16] = imem[(imem_addr>>1)+m];

   // flat data memory (byte addressable), sentinel-initialised
   reg [7:0] dmem [0:1023];
   wire [9:0] dra = dmem_raddr[9:0];
   assign dmem_rdata = {dmem[dra+7],dmem[dra+6],dmem[dra+5],dmem[dra+4],
                        dmem[dra+3],dmem[dra+2],dmem[dra+1],dmem[dra+0]};
   integer mb;
   always @(posedge clk)
      if (dmem_wen)
         for (mb=0; mb<8; mb=mb+1)
            if (dmem_wmask[mb]) dmem[dmem_waddr[9:0]+mb] <= dmem_wdata[mb*8 +: 8];

   // observe writeback values
   reg seen256, seenAB, seenAC, seenBad;

   initial begin
      for (i=0;i<64;i=i+1) imem[i]=16'h0001;            // c.nop fill
      imem[0]=16'h0513; imem[1]=16'h1000;               // addi a0,x0,256
      imem[2]=16'h0593; imem[3]=16'h0AB0;               // addi a1,x0,0xAB
      imem[4]=16'h3023; imem[5]=16'h00B5;               // sd   a1,0(a0)
      imem[6]=16'h3603; imem[7]=16'h0005;               // ld   a2,0(a0)
      imem[8]=16'h0693; imem[9]=16'h0016;               // addi a3,a2,1
      for (i=0;i<1024;i=i+1) dmem[i]=8'hEE;             // sentinel

      seen256=0; seenAB=0; seenAC=0; seenBad=0;
      reset=1; @(negedge clk); @(negedge clk); reset=0;

      for (c=0; c<40; c=c+1) begin
         @(negedge clk);
         for (k=0;k<IW;k=k+1) if (wb_valid[k]) begin
            case (wb_val[k*64+:64])
              64'd256:   seen256=1;
              64'h0AB:   seenAB =1;
              64'h0AC:   seenAC =1;
              64'hEF, 64'hFFFFFFFFFFFFFFEF: seenBad=1;   // sentinel+1 => load missed forward
            endcase
         end
      end

      if (!seen256) begin $display("FAIL: a0=256 never written back"); errs=errs+1; end
      if (!seenAB)  begin $display("FAIL: 0xAB never written back");   errs=errs+1; end
      if (!seenAC)  begin $display("FAIL: a3=0xAC missing (load did not forward to consumer)"); errs=errs+1; end
      if (seenBad)  begin $display("FAIL: consumer saw memory sentinel (forward failed)"); errs=errs+1; end

      // store must have committed + drained to memory
      if ({dmem[10'd263],dmem[10'd262],dmem[10'd261],dmem[10'd260],
           dmem[10'd259],dmem[10'd258],dmem[10'd257],dmem[10'd256]} !== 64'h00000000000000AB) begin
         $display("FAIL: dmem[256]=%h exp 0xAB (store not drained)",
            {dmem[10'd263],dmem[10'd262],dmem[10'd261],dmem[10'd260],
             dmem[10'd259],dmem[10'd258],dmem[10'd257],dmem[10'd256]});
         errs=errs+1;
      end

      if (errs==0) $display("ldst: ALL TESTS PASSED (store-to-load forward + drain end-to-end)");
      else         $display("ldst: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
