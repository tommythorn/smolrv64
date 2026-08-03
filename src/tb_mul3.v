`timescale 1ns/1ps
`default_nettype none

// 3-cycle pipelined multiply: drive start, wait for done, check result. Same MUL/
// MULH/MULHSU/MULHU/MULW cases as the combinational unit, plus latency=3 and abort.
module tb;
   reg         clk=0; always #5 clk=~clk;
   reg         reset, start, abort;
   reg  [63:0] rs1, rs2;
   reg  [2:0]  f3;
   reg         is_w;
   wire        busy, done;
   wire [63:0] result;
   integer errs=0;

   mul3 dut (.clk(clk), .reset(reset), .start(start), .abort(abort),
             .rs1(rs1), .rs2(rs2), .f3(f3), .is_w(is_w),
             .busy(busy), .done(done), .result(result));

   localparam M=3'b000, MH=3'b001, MHSU=3'b010, MHU=3'b011;

   task run(input [200:0] nm, input [63:0] a, input [63:0] bb, input w, input [2:0] func, input [63:0] exp);
      integer g;
      begin
         @(negedge clk); rs1=a; rs2=bb; is_w=w; f3=func; start=1;
         @(negedge clk); start=0;
         g=0; while (!done && g<10) begin @(negedge clk); g=g+1; end
         if (!done) begin $display("FAIL %0s: never completed", nm); errs=errs+1; end
         else if (result!==exp) begin $display("FAIL %0s: %h*%h w=%b f3=%b -> %h exp %h",nm,a,bb,w,func,result,exp); errs=errs+1; end
      end
   endtask

   integer g;
   initial begin
      reset=1; start=0; abort=0; rs1=0; rs2=0; f3=0; is_w=0;
      @(negedge clk); @(negedge clk); reset=0;

      run("mul.pos",   64'd6, 64'd7, 0, M, 64'd42);
      run("mul.neg",   -64'd3, 64'd5, 0, M, -64'd15);
      run("mulh.hi",   64'h4000000000000000, 64'd4, 0, MH,  64'd1);
      run("mulh.-1-1", ~64'd0, ~64'd0,            0, MH,  64'd0);
      run("mulhu.max", ~64'd0, ~64'd0,            0, MHU, 64'hFFFFFFFFFFFFFFFE);
      run("mulhsu",    ~64'd0, 64'd2,             0, MHSU,64'hFFFFFFFFFFFFFFFF);
      run("mulw.-1*2", 64'hFFFFFFFF, 64'd2,       1, M, 64'hFFFFFFFFFFFFFFFE);
      run("mulw.junk", 64'hDEAD0000_00000003, 64'hBEEF0000_00000004, 1, M, 64'd12);

      // latency is exactly 3: start, then done on the 3rd edge after
      @(negedge clk); rs1=64'd9; rs2=64'd9; is_w=0; f3=M; start=1;
      @(negedge clk); start=0;
      @(posedge clk); if (done) begin $display("FAIL lat: done too early (1)"); errs=errs+1; end
      @(posedge clk); if (done) begin $display("FAIL lat: done too early (2)"); errs=errs+1; end
      @(posedge clk); if (!done || result!==64'd81) begin $display("FAIL lat: done=%b res=%0d @3",done,result); errs=errs+1; end

      // abort mid-flight: no done
      @(negedge clk); rs1=64'd1000; rs2=64'd1000; f3=M; is_w=0; start=1;
      @(negedge clk); start=0;
      abort=1; @(negedge clk); abort=0;
      g=0; begin : aw integer w2; for(w2=0;w2<6;w2=w2+1) begin @(negedge clk); if(done) begin $display("FAIL abort: done fired"); errs=errs+1; end end end
      run("post-abort", 64'd5, 64'd5, 0, M, 64'd25);

      if (errs==0) $display("mul3: ALL TESTS PASSED");
      else         $display("mul3: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
