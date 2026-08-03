`timescale 1ns/1ps

// Unit TB for fp_unit (CVFPU wrapper): issue real FP ops, check results + tag round-trip.
module tb;
   logic clk=0; always #5 clk=~clk;
   logic reset;

   logic        iss_valid, iss_op_mod;  logic [3:0] iss_op;
   logic [2:0]  iss_src_fmt, iss_dst_fmt, iss_rnd;  logic [1:0] iss_int_fmt;
   logic [191:0] iss_operands;  logic [23:0] iss_tag;
   wire         iss_ready, res_valid, busy;
   wire [63:0]  res_data;  wire [4:0] res_fflags;  wire [23:0] res_tag;
   integer errs=0;

   fp_unit #(.TAGW(24)) dut
     (.clk(clk), .reset(reset),
      .iss_valid(iss_valid), .iss_ready(iss_ready),
      .iss_op(iss_op), .iss_op_mod(iss_op_mod),
      .iss_src_fmt(iss_src_fmt), .iss_dst_fmt(iss_dst_fmt), .iss_int_fmt(iss_int_fmt),
      .iss_rnd(iss_rnd), .iss_operands(iss_operands), .iss_tag(iss_tag),
      .res_valid(res_valid), .res_ready(1'b1), .res_data(res_data),
      .res_fflags(res_fflags), .res_tag(res_tag), .busy(busy));

   localparam [3:0] ADD=2, MUL=3;
   // issue one op (op0/op1/op2 per fpnew routing), wait the result, check
   task do_op; input [127:0] nm; input [3:0] op; input om; input [2:0] fmt;
                input [63:0] o0,o1,o2; input [23:0] tg; input [63:0] expv;
      begin
         @(negedge clk);
         iss_valid=1; iss_op=op; iss_op_mod=om; iss_src_fmt=fmt; iss_dst_fmt=fmt;
         iss_int_fmt=2'd3; iss_rnd=3'd0; iss_operands={o2,o1,o0}; iss_tag=tg;
         @(posedge clk);
         while (!iss_ready) @(posedge clk);          // accepted
         @(negedge clk); iss_valid=0;
         while (!res_valid) @(posedge clk);
         if (res_data!==expv || res_tag!==tg) begin
            $display("FAIL %0s: got=%h exp=%h tag=%h/%h", nm, res_data, expv, res_tag, tg); errs=errs+1;
         end else $display("  ok  %0s = %h (tag %h)", nm, res_data, tg);
         @(negedge clk);
      end
   endtask

   // IEEE-754 double constants
   localparam [63:0] D1=64'h3FF0000000000000, D2=64'h4000000000000000, D3=64'h4008000000000000,
                     D6=64'h4018000000000000, D5=64'h4014000000000000;
   localparam [31:0] S1=32'h3F800000, S2=32'h40000000, S3=32'h40400000;
   initial begin
      iss_valid=0; iss_op=0; iss_op_mod=0; iss_src_fmt=1; iss_dst_fmt=1; iss_int_fmt=3;
      iss_rnd=0; iss_operands=0; iss_tag=0;
      reset=1; repeat(4) @(negedge clk); reset=0; @(negedge clk);
      // FADD.D: 1.0 + 2.0 = 3.0  (ADD: operands[1]+operands[2], op0 unused)
      do_op("FADD.D 1+2", ADD, 1'b0, 3'd1, 64'd0, D1, D2, 24'hA1, D3);
      // FSUB.D: 3.0 - 2.0 = 1.0  (op_mod=1)
      do_op("FSUB.D 3-2", ADD, 1'b1, 3'd1, 64'd0, D3, D2, 24'hB2, D1);
      // FMUL.D: 2.0 * 3.0 = 6.0  (MUL: operands[0]*operands[1])
      do_op("FMUL.D 2*3", MUL, 1'b0, 3'd1, D2, D3, 64'd0, 24'hC3, D6);
      // FADD.S: 1.0 + 2.0 = 3.0  (NaN-boxed single)
      do_op("FADD.S 1+2", ADD, 1'b0, 3'd0, 64'd0, {32'hffffffff,S1}, {32'hffffffff,S2}, 24'hD4, {32'hffffffff,S3});

      if (errs==0) $display("FP_UNIT-TB: ALL TESTS PASSED"); else $display("FP_UNIT-TB FAIL (%0d)", errs);
      $finish;
   end
   initial begin #200000; $display("FP_UNIT-TB TIMEOUT"); $finish; end
endmodule
