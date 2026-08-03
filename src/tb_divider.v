`timescale 1ns/1ps
`default_nettype none

// Iterative divider: drive start, wait for done, check result. Same div/rem cases
// as the M datapath test (signed/unsigned, trunc-toward-zero, rem sign-of-dividend,
// divide-by-zero, signed overflow, W sign-extension), plus a back-to-back pair to
// exercise the handshake returning to idle.
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

   task run(input [200:0] nm, input [63:0] a, input [63:0] bb, input w, input [2:0] func, input [63:0] exp);
      integer guard;
      begin
         @(negedge clk); rs1=a; rs2=bb; is_w=w; f3=func; start=1'b1;
         @(negedge clk); start=1'b0;
         guard=0;
         while (!done && guard<200) begin @(negedge clk); guard=guard+1; end
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
      @(negedge clk); rs1=64'd1000; rs2=64'd7; is_w=0; f3=D; start=1;
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

      if (errs==0) $display("divider: ALL TESTS PASSED (iterative div/rem)");
      else         $display("divider: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
