`default_nettype none

// tb_fp_divsqrt -- fp_divsqrt against tools/fpref.py's exact vectors: every result bit and all five
// flags, ops issued back to back and the answer held back at random (out_ready), a flush now and
// then on an op whose answer is then not checked. +vec=<file> (lines "fmt rm a b result flags", one
// file per op) and +sqrt=0|1 say what the file holds.
module tb;
   reg clk = 0;
   always #5 clk = ~clk;
   reg         reset = 1, flush = 0;
   reg         in_valid = 0, in_sqrt = 0, in_f64 = 0, out_ready = 0;
   reg  [2:0]  in_rm = 0;
   reg  [63:0] in_a = 0, in_b = 0;
   reg  [23:0] in_tag = 0;
   wire        in_ready, out_valid, busy;
   wire [63:0] out_result;
   wire [4:0]  out_flags;
   wire [23:0] out_tag;
   fp_divsqrt #(.TAGW(24)) dut (
      .clk(clk), .reset(reset), .flush(flush),
      .in_valid(in_valid), .in_ready(in_ready), .in_sqrt(in_sqrt), .in_f64(in_f64), .in_rm(in_rm),
      .in_a(in_a), .in_b(in_b), .in_tag(in_tag),
      .out_valid(out_valid), .out_ready(out_ready), .out_result(out_result), .out_flags(out_flags),
      .out_tag(out_tag), .busy(busy));

   integer fd, n, bad, flushed, fmt, rm, cyc, maxcyc, sum, sq;
   reg [63:0] a, b, want;
   reg [7:0]  wf;
   string     vec;
   initial begin
      if (!$value$plusargs("vec=%s", vec)) $fatal(1, "tb_fp_divsqrt: no +vec=<file>");
      if (!$value$plusargs("sqrt=%d", sq)) $fatal(1, "tb_fp_divsqrt: no +sqrt=0|1");
      fd = $fopen(vec, "r");
      if (fd == 0) $fatal(1, "tb_fp_divsqrt: cannot open %s", vec);
      n = 0; bad = 0; flushed = 0; maxcyc = 0; sum = 0;
      repeat (3) @(posedge clk);
      reset = 0;
      while ($fscanf(fd, "%d %d %h %h %h %h\n", fmt, rm, a, b, want, wf) == 6) begin
         @(negedge clk);
         while (!in_ready) @(negedge clk);
         in_valid = 1; in_sqrt = sq[0]; in_f64 = fmt[0]; in_rm = rm[2:0]; in_a = a; in_b = b; in_tag = n[23:0];
         @(negedge clk);
         in_valid = 0;
         cyc = 1;
         if (($urandom % 500) == 0) begin
            // a flush mid-op: the unit must be idle the next cycle, the op gone
            repeat ($urandom % 20) @(negedge clk);
            flush = 1; @(negedge clk); flush = 0;
            if (busy) begin $display("FAIL: busy after a flush"); bad = bad + 1; end
            flushed = flushed + 1; n = n + 1;
            continue;
         end
         while (!out_valid) begin @(negedge clk); cyc = cyc + 1; end
         // hold the answer a few cycles now and then: it must not change
         if (($urandom % 4) == 0) repeat ($urandom % 4) @(negedge clk);
         if (out_result !== want || {3'b000, out_flags} !== wf || out_tag !== n[23:0]) begin
            bad = bad + 1;
            if (bad <= 20)
               $display("FAIL: %s fmt=%0d rm=%0d a=%h b=%h got %h/%02x want %h/%02x (tag %0d/%0d)",
                        sq ? "sqrt" : "div", fmt, rm, a, b, out_result, out_flags, want, wf, out_tag, n[23:0]);
         end
         if (cyc > maxcyc) maxcyc = cyc;
         sum = sum + cyc;
         out_ready = 1; @(negedge clk); out_ready = 0;
         n = n + 1;
      end
      $display("FDIV-TB %s: %0d vectors, %0d flushed, %0d mismatches; latency mean %0d max %0d cycles",
               sq ? "sqrt" : "div", n, flushed, bad, sum / ((n - flushed) > 0 ? (n - flushed) : 1), maxcyc);
      if (bad == 0 && n > 0) $display("FDIV-TB PASS"); else $display("FDIV-TB FAIL");
      $finish;
   end
endmodule

`default_nettype wire
