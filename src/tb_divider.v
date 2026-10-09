`timescale 1ns/1ps
`default_nettype none

// Iterative divider: drive start, wait for done, check result. Same div/rem cases
// as the M datapath test (signed/unsigned, trunc-toward-zero, rem sign-of-dividend,
// divide-by-zero, signed overflow, W sign-extension), plus a back-to-back pair to
// exercise the handshake returning to idle, then random operations of every operand width
// (so every iteration count) against Verilog's / and %, and the latencies they took.
`include "tb_rand.vh"
module tb;
   reg         clk=0; always #5 clk=~clk;
   reg         reset, start, abort;
   reg  [63:0] rs1, rs2;
   reg  [2:0]  f3;
   reg         is_w;
   wire        busy, done;
   wire [63:0] result;
   integer errs=0;

   divider dut (.clk(clk), .reset(reset), .start(start), .abort(abort), .rs1(rs1), .rs2(rs2),
                .f3(f3), .is_w(is_w), .busy(busy), .done(done), .result(result));

   localparam D=3'b100, DU=3'b101, R=3'b110, RU=3'b111;
   integer lat;                                 // the last run's cycles, start to done
   `TB_RAND(rnd, rs)
   // the RISC-V M result, from Verilog's operators
   function [63:0] mref(input [63:0] a, input [63:0] b, input w, input [2:0] func);
      reg [63:0] q, r;
      reg [31:0] q32, r32;
      begin
         if (w) begin
            if (b[31:0] == 0) begin q32 = 32'hffffffff; r32 = a[31:0]; end
            else if (!func[0] && a[31:0] == 32'h80000000 && b[31:0] == 32'hffffffff) begin q32 = a[31:0]; r32 = 0; end
            else if (!func[0]) begin q32 = $signed(a[31:0]) / $signed(b[31:0]); r32 = $signed(a[31:0]) % $signed(b[31:0]); end
            else begin q32 = a[31:0] / b[31:0]; r32 = a[31:0] % b[31:0]; end
            mref = func[1] ? {{32{r32[31]}}, r32} : {{32{q32[31]}}, q32};
         end else begin
            if (b == 0) begin q = ~64'd0; r = a; end
            else if (!func[0] && a == 64'h8000000000000000 && b == ~64'd0) begin q = a; r = 0; end
            else if (!func[0]) begin q = $signed(a) / $signed(b); r = $signed(a) % $signed(b); end
            else begin q = a / b; r = a % b; end
            mref = func[1] ? r : q;
         end
      end
   endfunction

   task run(input [200:0] nm, input [63:0] a, input [63:0] bb, input w, input [2:0] func, input [63:0] exp);
      integer guard;
      begin
         @(negedge clk); rs1=a; rs2=bb; is_w=w; f3=func; start=1'b1;
         @(negedge clk); start=1'b0;
         guard=0;
         while (!done && guard<200) begin @(negedge clk); guard=guard+1; end
         lat=guard+1;
         if (!done) begin $display("FAIL %0s: never completed", nm); errs=errs+1; end
         else if (result!==exp)
            begin $display("FAIL %0s: %h / %h w=%b f3=%b -> %h exp %h", nm,a,bb,w,func,result,exp); errs=errs+1; end
      end
   endtask

   initial begin
      reset=1; start=0; abort=0; rs1=0; rs2=0; f3=0; is_w=0;
      @(negedge clk); @(negedge clk); reset=0;

      run("div.20/3",   64'd20, 64'd3,  0, D,  64'd6);
      run("div.-20/3",  -64'd20, 64'd3, 0, D,  -64'd6);
      run("rem.20/3",   64'd20, 64'd3,  0, R,  64'd2);
      run("rem.-20/3",  -64'd20, 64'd3, 0, R,  -64'd2);
      run("div.-20/-3", -64'd20, -64'd3,0, D,  64'd6);
      run("rem.7/-3",   64'd7,  -64'd3, 0, R,  64'd1);     // rem sign = dividend (+)
      run("div.x/0",    64'd5,  64'd0,  0, D,  ~64'd0);
      run("rem.x/0",    64'd5,  64'd0,  0, R,  64'd5);
      run("divu.x/0",   64'd5,  64'd0,  0, DU, ~64'd0);
      run("remu.x/0",   64'd5,  64'd0,  0, RU, 64'd5);
      run("divu.max/2", ~64'd0, 64'd2,  0, DU, 64'h7FFFFFFFFFFFFFFF);
      run("remu.big",   64'h0000000100000000, 64'd7, 0, RU, 64'd4); // 2^32 % 7 = 4 (2^3==1 mod 7)
      run("div.ovf",    64'h8000000000000000, ~64'd0, 0, D, 64'h8000000000000000);
      run("rem.ovf",    64'h8000000000000000, ~64'd0, 0, R, 64'd0);
      // W variants
      run("divw.-20/3", 64'hFFFFFFEC, 64'd3,        1, D,  64'hFFFFFFFFFFFFFFFA);
      run("remw.-20/3", 64'hFFFFFFEC, 64'd3,        1, R,  64'hFFFFFFFFFFFFFFFE);
      run("divuw.x/0",  64'd5,  64'd0,              1, DU, 64'hFFFFFFFFFFFFFFFF);
      run("remuw.7/4",  64'd7,  64'd4,              1, RU, 64'd3);
      run("divuw.junk", 64'hDEAD000000000022, 64'h1234000000000005, 1, DU, 64'd6); // 34/5=6 (low32)
      run("divw.ovf",   64'h0000000080000000, 64'hFFFFFFFFFFFFFFFF, 1, D, 64'hFFFFFFFF80000000);

      // abort mid-run: start a divide, run partway, abort -> busy drops, no done,
      // and the unit is free to accept a new divide that completes correctly.
      @(negedge clk); rs1=~64'd0 >> 1; rs2=64'd3; is_w=0; f3=D; start=1;   // a 62-bit quotient: 31 steps
      @(negedge clk); start=0;
      repeat (5) @(negedge clk);
      if (!busy) begin $display("FAIL abort: not busy mid-run"); errs=errs+1; end
      abort=1; @(negedge clk); abort=0;
      if (busy) begin $display("FAIL abort: still busy after abort"); errs=errs+1; end
      begin : abwin
         integer w; for (w=0; w<70; w=w+1) begin
            @(negedge clk);
            if (done) begin $display("FAIL abort: aborted divide asserted done"); errs=errs+1; end
         end
      end
      run("post-abort.div", 64'd20, 64'd3, 0, D, 64'd6);   // unit recovered
      // random: operands of every width (each shifted down by 0..63), either sign, all ops
      begin : random
         integer i, hist [0:40], mx; reg [63:0] a, b; reg w; reg [2:0] func; reg [8*40-1:0] nm;
         for (i = 0; i <= 40; i = i + 1) hist[i] = 0;
         mx = 0;
         for (i = 0; i < 20000; i = i + 1) begin
            a = {rnd(0), rnd(0)} >> (rnd(0) % 64);  if (rnd(0) % 4 == 0) a = -a;
            b = {rnd(0), rnd(0)} >> (rnd(0) % 64);  if (rnd(0) % 4 == 0) b = -b;
            if (rnd(0) % 64 == 0) b = 0;
            w = rnd(0) % 3 == 0;  func = {1'b1, 2'(rnd(0) % 4)};
            run("random", a, b, w, func, mref(a, b, w, func));
            if (errs > 10) disable random;
            if (lat > mx) mx = lat;
            hist[lat > 40 ? 40 : lat] = hist[lat > 40 ? 40 : lat] + 1;
         end
         $display("divider: 20000 random ops; latency (cycles, start to done) max %0d; 3:%0d 4:%0d 5:%0d 6:%0d 10:%0d 19:%0d 35:%0d",
                  mx, hist[3], hist[4], hist[5], hist[6], hist[10], hist[19], hist[35]);
      end
      run("lat.small", 64'd100, 64'd7, 0, D, 64'd14);
      $display("divider: 100/7 took %0d cycles", lat);
      run("lat.full", ~64'd0, 64'd1, 0, DU, ~64'd0);
      $display("divider: (2^64-1)/1 took %0d cycles", lat);

      if (errs==0) $display("divider: ALL TESTS PASSED (iterative div/rem)");
      else         $display("divider: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
