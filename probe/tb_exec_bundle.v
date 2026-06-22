`timescale 1ns/1ps
`include "exec_pay.vh"
`default_nettype none

// exec_bundle: 4 execute slices + wb broadcast net. T0 shards 0/1 produce values
// into pr4/pr5; T1 shards 2/3 read them back (cross-shard, via the wb broadcast ->
// every RF copy) and compute.
module tb;
   localparam SHARDS=4, PBITS=8;
   reg                clk=0; always #5 clk=~clk;
   reg  [SHARDS-1:0]       iss_valid, iss_pdst_v;
   reg  [SHARDS*PBITS-1:0] iss_pdst, iss_ps1, iss_ps2;
   reg  [SHARDS*`PAYW-1:0] iss_pay;
   wire [SHARDS-1:0]       wb_valid, cmp_eq, cmp_lt, cmp_ltu;
   wire [SHARDS*PBITS-1:0] wb_pr;
   wire [SHARDS*64-1:0]    wb_val, agu_addr;
   integer errs=0;

   exec_bundle dut
     (.clk(clk), .iss_valid(iss_valid), .iss_pdst(iss_pdst), .iss_pdst_v(iss_pdst_v),
      .iss_ps1(iss_ps1), .iss_ps2(iss_ps2), .iss_pay(iss_pay),
      .wb_valid(wb_valid), .wb_pr(wb_pr), .wb_val(wb_val), .agu_addr(agu_addr),
      .cmp_eq(cmp_eq), .cmp_lt(cmp_lt), .cmp_ltu(cmp_ltu));

   function [`PAYW-1:0] mkpay(input [5:0] op, input [1:0] o1s, input o2i,
                              input [63:0] imm, input [63:0] pc);
      mkpay = {pc, imm, 1'b0/*mem*/, 1'b0/*rvc*/, 1'b0/*link*/, o2i, o1s,
               1'b0/*uw*/, 1'b0/*w*/, op};
   endfunction
   task ckval(input [127:0] nm, input integer k, input [63:0] e);
      begin if (wb_val[k*64 +: 64]!==e) begin
        $display("FAIL %0s wb_val[%0d]=%0d exp %0d",nm,k,wb_val[k*64+:64],e); errs=errs+1; end end
   endtask

   initial begin
      iss_valid=0; iss_pdst_v=0; iss_pdst=0; iss_ps1=0; iss_ps2=0; iss_pay=0;
      @(negedge clk);

      // T0: shard0: pr4 <- 100 (LUI-style: op1=ZERO, op2=imm); shard1: pr5 <- 200
      iss_valid=4'b0011; iss_pdst_v=4'b0011;
      iss_pdst = {8'd0, 8'd0, 8'd5, 8'd4};
      iss_pay  = { {`PAYW{1'b0}}, {`PAYW{1'b0}},
                   mkpay(6'd0, 2'd2, 1'b1, 64'd200, 0),
                   mkpay(6'd0, 2'd2, 1'b1, 64'd100, 0) };
      #1; ckval("T0.s0",0,64'd100); ckval("T0.s1",1,64'd200);

      @(negedge clk);
      // T1: shard2: pr6 <- RF[4]+1 = 101 ; shard3: pr7 <- RF[5]+1 = 201 (cross-shard)
      iss_valid=4'b1100; iss_pdst_v=4'b1100;
      iss_pdst = {8'd7, 8'd6, 8'd0, 8'd0};
      iss_ps1  = {8'd5, 8'd4, 8'd0, 8'd0};
      iss_pay  = { mkpay(6'd0, 2'd0, 1'b1, 64'd1, 0),
                   mkpay(6'd0, 2'd0, 1'b1, 64'd1, 0),
                   {`PAYW{1'b0}}, {`PAYW{1'b0}} };
      #1; ckval("T1.s2",2,64'd101); ckval("T1.s3",3,64'd201);

      @(negedge clk); iss_valid=0;
      if (errs==0) $display("exec_bundle: ALL TESTS PASSED");
      else         $display("exec_bundle: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
