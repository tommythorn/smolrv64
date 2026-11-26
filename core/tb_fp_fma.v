`default_nettype none

// tb_fp_fma -- fp_fma against tools/fpref.py's exact FMA-group vectors (+vec=<file>, lines
// "v fmt rm a b c result flags"): every result bit and flag, ops issued back to back with random
// bubbles, results checked in issue order by tag, a flush now and then (the ops in flight then
// produce nothing).
module tb;
   reg clk = 0;
   always #5 clk = ~clk;
   reg         reset = 1, flush = 0, in_valid = 0, in_il = 0, in_iu = 0;
   reg  [1:0]  in_sf = 0, in_df = 0;
   reg  [3:0]  in_op = 0;
   reg  [2:0]  in_rm = 0;
   reg  [63:0] in_a = 0, in_b = 0, in_c = 0;
   reg  [23:0] in_tag = 0;
   wire        out_valid;
   wire [63:0] out_result;
   wire [4:0]  out_flags;
   wire [23:0] out_tag;
   fp_fma #(.TAGW(24)) dut (
      .clk(clk), .reset(reset), .flush(flush), .in_valid(in_valid), .in_op(in_op), .in_sf(in_sf),
      .in_df(in_df), .in_il(in_il), .in_iu(in_iu), .in_rm(in_rm), .in_a(in_a), .in_b(in_b), .in_c(in_c), .in_tag(in_tag),
      .out_valid(out_valid), .out_result(out_result), .out_flags(out_flags), .out_tag(out_tag));

   localparam integer N = 1 << 20;
   reg  [63:0] want_r [0:N-1];
   reg  [7:0]  want_f [0:N-1];
   reg  [3:0]  op_of  [0:N-1];
   reg  [63:0] a_of [0:N-1], b_of [0:N-1], c_of [0:N-1];
   reg  [8:0]  fr_of [0:N-1];
   reg         dead  [0:N-1];                  // issued, then flushed
   integer fd, n, issued, got, bad, flushed, v, sf, df, rm, xi, lat, t_issue [0:N-1], maxlat;
   reg [63:0] a, b, c, r;  reg [7:0] f;

   // the checker: results come back in issue order, dead ones never
   integer exp_i;
   initial exp_i = 0;
   // a result in the cycle a flush is raised is dropped by fp_unit (its flush wins), and the op is
   // already marked dead here
   always @(posedge clk) if (!reset) begin
      if (out_valid && !flush) begin
         while (exp_i < issued && dead[exp_i]) exp_i = exp_i + 1;
         if (out_tag !== exp_i[23:0]) begin
            bad = bad + 1;
            if (bad <= 20) $display("FAIL: result tag %0d, expected %0d", out_tag, exp_i);
         end else begin
            if (out_result !== want_r[exp_i] || {3'b000, out_flags} !== want_f[exp_i]) begin
               bad = bad + 1;
               if (bad <= 20)
                  $display("FAIL: v=%0d sf.df.il.iu.rm=%03x a=%h b=%h c=%h got %h/%02x want %h/%02x",
                           op_of[exp_i], fr_of[exp_i], a_of[exp_i], b_of[exp_i], c_of[exp_i],
                           out_result, out_flags, want_r[exp_i], want_f[exp_i]);
            end
            lat = $time / 10 - t_issue[exp_i];
            if (lat > maxlat) maxlat = lat;
         end
         got = got + 1;
         exp_i = exp_i + 1;
      end
   end

   string vec;
   initial begin
      if (!$value$plusargs("vec=%s", vec)) $fatal(1, "tb_fp_fma: no +vec=<file>");
      fd = $fopen(vec, "r");
      if (fd == 0) $fatal(1, "tb_fp_fma: cannot open %s", vec);
      issued = 0; got = 0; bad = 0; flushed = 0; maxlat = 0;
      repeat (3) @(posedge clk);
      reset = 0;
      while (issued < N && $fscanf(fd, "%d %d %d %d %d %h %h %h %h %h\n", v, sf, df, rm, xi, a, b, c, r, f) == 10) begin
         @(negedge clk);
         flush = 0;
         if (($urandom % 6) == 0) begin in_valid = 0; @(negedge clk); end   // a bubble
         want_r[issued] = r;  want_f[issued] = f;  op_of[issued] = v[3:0];
         a_of[issued] = a;  b_of[issued] = b;  c_of[issued] = c;  fr_of[issued] = {sf[1:0], df[1:0], xi[1:0], rm[2:0]};
         dead[issued] = 1'b0;  t_issue[issued] = $time / 10 + 1;
         in_valid = 1; in_op = v[3:0]; in_sf = sf[1:0]; in_df = df[1:0]; in_il = xi[1]; in_iu = xi[0]; in_rm = rm[2:0];
         in_a = a; in_b = b; in_c = c; in_tag = issued[23:0];
         issued = issued + 1;
         if (($urandom % 2000) == 0) begin
            // a flush with this op in flight: everything issued and not yet out is gone
            @(negedge clk); in_valid = 0; flush = 1;
            for (n = exp_i; n < issued; n = n + 1) dead[n] = 1'b1;
            flushed = flushed + 1;
         end
      end
      @(negedge clk); in_valid = 0; flush = 0;
      repeat (20) @(posedge clk);
      $display("FMA-TB: %0d issued, %0d results, %0d flushes, %0d mismatches; latency %0d cycles",
               issued, got, flushed, bad, maxlat);
      if (bad == 0 && got > 0) $display("FMA-TB PASS"); else $display("FMA-TB FAIL");
      $finish;
   end
endmodule

`default_nettype wire
