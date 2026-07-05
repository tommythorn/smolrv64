`timescale 1ns/1ps
`default_nettype none

// Full-frontend integration: PC -> fetch/align -> decode -> [reg] -> rename.
// Bundle 0 of the program is byte-for-byte the tb_decode_rename bundle:
//   @0  c.li  a0,5        (0x4515,        RVC, 1hw)
//   @2  add   a1,a0,a0    (0x00A505B3,    32b, 2hw)
//   @6  c.mv  a2,a1       (0x862E,        RVC, 1hw)
//   @8  addi  a0,a2,1     (0x00160513,    32b, 2hw)
// So the renamed result reaching rename one cycle after fetch must match
// tb_decode_rename exactly: pdst=[0,1,2,3], intra-bundle SLOT RAW resolved,
// r_rd=[10,11,12,10]. This proves the fetch path feeds decode->rename correctly.
module tb;
   localparam IW=4, HW=8, PCW=64, SEQW=8, ABITS=6, PBITS=8, PBW=4;

   reg                clk=0; always #5 clk=~clk;
   reg                reset, redirect=0;
   reg  [PCW-1:0]     redirect_pc=0;
   reg  [SEQW-1:0]    redirect_seq=0;
   wire [PCW-1:0]     imem_addr;
   reg  [HW*16-1:0]   imem_data;
   wire [PBW-1:0]     imem_avail = 4'd8;
   reg                create=1'b1, commit=0, rollback=0, accept=1'b1;
   reg  [1:0]         commit_idx=0, rollback_idx=0;
   wire [IW-1:0]      r_valid, r_rd_v, stall;
   wire [IW*SEQW-1:0] r_seq;
   wire [IW*ABITS-1:0] r_rd;
   wire [IW*PBITS-1:0] ps1, ps2, pdst;
   wire [1:0]         r_ckpt, cur;
   integer errs=0;

   frontend #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW), .RESET_PC(0),
              .ABITS(ABITS), .PBITS(PBITS)) dut
     (.clk(clk), .reset(reset), .redirect(redirect), .redirect_pc(redirect_pc),
      .redirect_seq(redirect_seq), .solo_all(1'b0), .irq_inject(1'b0), .imem_addr(imem_addr), .imem_data(imem_data),
      .imem_avail(imem_avail), .accept(accept),
      .create(create), .commit(commit), .commit_idx(commit_idx),
      .rollback(rollback), .rollback_idx(rollback_idx),
      .res_v(1'b0), .res_cbr(1'b0), .res_taken(1'b0),
      .res_ckpt(2'd0), .res_tgt(64'd0), .res_rep(1'b0),
      .r_valid(r_valid), .r_seq(r_seq), .r_rd(r_rd), .r_rd_v(r_rd_v),
      .ps1(ps1), .ps2(ps2), .pdst(pdst),
      .r_is_branch(), .r_pay(), .r_ckpt(r_ckpt), .cur(cur),
      .stall(stall));

   reg [15:0] mem [0:63];
   integer m;
   always @(*) for (m=0;m<HW;m=m+1) imem_data[m*16 +: 16] = mem[(imem_addr>>1)+m];

   function [PBITS-1:0] P(input [IW*PBITS-1:0] bus, input integer k);
      P = bus[k*PBITS +: PBITS]; endfunction
   task ckp(input [127:0] nm, input [PBITS-1:0] got, exp);
      begin if (got!==exp) begin $display("FAIL %0s = %0d exp %0d",nm,got,exp); errs=errs+1; end end
   endtask

   integer i;
   initial begin
      for (i=0;i<64;i=i+1) mem[i]=16'h0001;          // c.nop fill
      mem[0]=16'h4515;                               // c.li a0,5
      mem[1]=16'h05B3; mem[2]=16'h00A5;              // add a1,a0,a0 = 0x00A505B3
      mem[3]=16'h862E;                               // c.mv a2,a1
      mem[4]=16'h0513; mem[5]=16'h0016;              // addi a0,a2,1 = 0x00160513

      reset=1;
      @(negedge clk); reset=0;
      // cycle A: fetch presents bundle 0 (pc=0); decode comb; latched next edge
      @(posedge clk);    // fetch fires (pc->12); decode/rename boundary <= bundle0
      @(negedge clk);    // rename now processing bundle 0
      #1;
      // ---- renamed bundle 0 must match tb_decode_rename ----
      if (r_valid!==4'b1111) begin $display("FAIL r_valid=%b",r_valid); errs=errs+1; end
      if (r_rd_v !==4'b1111) begin $display("FAIL r_rd_v=%b",r_rd_v); errs=errs+1; end
      ckp("pdst0",P(pdst,0),8'd64); ckp("pdst1",P(pdst,1),8'd65);  // phys 0..63 reserved for arch
      ckp("pdst2",P(pdst,2),8'd66); ckp("pdst3",P(pdst,3),8'd67);
      ckp("ps1[1]<-pdst0",P(ps1,1),P(pdst,0));   // add a1,a0,a0 : both srcs SLOT(0)
      ckp("ps2[1]<-pdst0",P(ps2,1),P(pdst,0));
      ckp("ps2[2]<-pdst1",P(ps2,2),P(pdst,1));   // c.mv a2,a1 : src2 SLOT(1)
      ckp("ps1[3]<-pdst2",P(ps1,3),P(pdst,2));   // addi a0,a2,1 : src1 SLOT(2)
      if (r_rd[0*ABITS+:ABITS]!==6'd10 || r_rd[1*ABITS+:ABITS]!==6'd11 ||
          r_rd[2*ABITS+:ABITS]!==6'd12 || r_rd[3*ABITS+:ABITS]!==6'd10)
         begin $display("FAIL r_rd"); errs=errs+1; end
      if (r_seq[0*SEQW+:SEQW]!==8'd0 || r_seq[3*SEQW+:SEQW]!==8'd3)
         begin $display("FAIL r_seq"); errs=errs+1; end

      @(negedge clk);
      if (errs==0) $display("frontend: ALL TESTS PASSED");
      else         $display("frontend: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
