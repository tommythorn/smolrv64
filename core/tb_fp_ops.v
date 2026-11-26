`default_nettype none

// tb_fp_ops -- the core's own Zfa FP ops (smolrv64_fp_ops.vh: fround_d/_s, fminmax_d/_s with `m`,
// fcmp_d/_s's quiet compares, fli_d/_s) against tools/fpref.py's exact vectors (+vec=<file>, lines
// "op fmt rm a b result flags": op 0 fround, 1 froundnx, 2 fminm, 3 fmaxm, 4 fleq, 5 fltq, 6 fli).
// Each function is applied exactly as smolrv64_core.v's F stage applies it.
module tb;
   `include "smolrv64_fp_ops.vh"
   function [31:0] unbox_s; input [63:0] x;
      unbox_s = (x[63:32] == 32'hffffffff) ? x[31:0] : 32'h7fc00000; endfunction

   integer fd, n, bad, op, fmt, rm;
   reg [63:0] a, b, r, got;  reg [7:0] f;  reg [4:0] gf;
   reg [68:0] fr;  reg [64:0] mm;  reg [1:0] cq;
   string vec;
   initial begin
      if (!$value$plusargs("vec=%s", vec)) $fatal(1, "tb_fp_ops: no +vec=<file>");
      fd = $fopen(vec, "r");
      if (fd == 0) $fatal(1, "tb_fp_ops: cannot open %s", vec);
      n = 0; bad = 0;
      while ($fscanf(fd, "%d %d %d %h %h %h %h\n", op, fmt, rm, a, b, r, f) == 7) begin
         case (op)
           0, 1: begin fr = fmt[0] ? fround_d(a, rm[2:0], op == 1) : fround_s(unbox_s(a), rm[2:0], op == 1);
                       got = fr[63:0];  gf = fr[68:64]; end
           2, 3: begin
                    cq = fmt[0] ? fcmp_d({2'b00, op == 3}, a, b) : fcmp_s({2'b00, op == 3}, unbox_s(a), unbox_s(b));
                    mm = fmt[0] ? fminmax_d(op == 3, 1'b1, a, b, cq[0]) : fminmax_s(op == 3, 1'b1, unbox_s(a), unbox_s(b), cq[0]);
                    got = mm[63:0];  gf = {mm[64], 4'd0}; end
           4, 5: begin cq = fmt[0] ? fcmp_d({2'b10, op == 5}, a, b) : fcmp_s({2'b10, op == 5}, unbox_s(a), unbox_s(b));
                       got = {63'd0, cq[0]};  gf = {cq[1], 4'd0}; end
           default: begin got = fmt[0] ? fli_d(a[4:0]) : {32'hffffffff, fli_s(a[4:0])};  gf = 5'd0; end
         endcase
         if (got !== r || {3'b000, gf} !== f) begin
            bad = bad + 1;
            if (bad <= 20) $display("FAIL: op=%0d fmt=%0d rm=%0d a=%h b=%h got %h/%02x want %h/%02x", op, fmt, rm, a, b, got, gf, r, f);
         end
         n = n + 1;
      end
      $display("FPOPS-TB: %0d vectors, %0d mismatches", n, bad);
      if (bad == 0 && n > 0) $display("FPOPS-TB PASS"); else $display("FPOPS-TB FAIL");
      $finish;
   end
endmodule

`default_nettype wire
