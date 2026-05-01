`timescale 1ns/1ps

module cvfpu_smoke_tb;
   logic clock = 1'b0;
   logic reset = 1'b1;

   logic                 in_valid = 1'b0;
   logic                 in_ready;
   logic [2:0][63:0]     operands = '0;
   logic [2:0]           rnd_mode = 3'd0;
   logic [3:0]           op = 4'd0;
   logic                 op_mod = 1'b0;
   logic [2:0]           src_fmt = 3'd1;
   logic [2:0]           dst_fmt = 3'd1;
   logic [1:0]           int_fmt = 2'd3;
   logic [7:0]           tag_in = 8'h5a;
   logic [63:0]          result;
   logic [4:0]           fflags;
   logic [7:0]           tag_out;
   logic                 out_valid;
   logic                 busy;

   always #5 clock = !clock;

   smolrv64_cvfpu dut(
      .clock     ( clock     ),
      .fpu_clock ( clock     ),
      .reset     ( reset     ),
      .in_valid  ( in_valid  ),
      .in_ready  ( in_ready  ),
      .operands  ( operands  ),
      .rnd_mode  ( rnd_mode  ),
      .op        ( op        ),
      .op_mod    ( op_mod    ),
      .src_fmt   ( src_fmt   ),
      .dst_fmt   ( dst_fmt   ),
      .int_fmt   ( int_fmt   ),
      .tag_in    ( tag_in    ),
      .result    ( result    ),
      .fflags    ( fflags    ),
      .tag_out   ( tag_out   ),
      .out_valid ( out_valid ),
      .out_ready ( 1'b1      ),
      .flush     ( 1'b0      ),
      .busy      ( busy      )
   );

   task automatic check_op(
      input string name,
      input [3:0] op_i,
      input       op_mod_i,
      input [2:0] src_fmt_i,
      input [2:0] dst_fmt_i,
      input [1:0] int_fmt_i,
      input [63:0] op0,
      input [63:0] op1,
      input [63:0] op2,
      input [63:0] expected
   );
      int cycles;
      begin
         @(posedge clock);
         operands[0] <= op0;
         operands[1] <= op1;
         operands[2] <= op2;
         rnd_mode    <= 3'd0;
         op          <= op_i;
         op_mod      <= op_mod_i;
         src_fmt     <= src_fmt_i;
         dst_fmt     <= dst_fmt_i;
         int_fmt     <= int_fmt_i;
         in_valid    <= 1'b1;

         cycles = 0;
         while (!in_ready && cycles < 50) begin
            @(posedge clock);
            cycles++;
         end
         if (!in_ready) begin
            $display("%s timed out waiting for in_ready", name);
            $fatal;
         end

         @(posedge clock);
         in_valid <= 1'b0;

         cycles = 0;
         while (!out_valid && cycles < 400) begin
            @(posedge clock);
            cycles++;
         end
         if (!out_valid) begin
            $display("%s timed out waiting for out_valid", name);
            $fatal;
         end
         if (tag_out != tag_in) begin
            $display("%s tag mismatch: got %02x expected %02x", name, tag_out, tag_in);
            $fatal;
         end
         if (fflags != 5'd0) begin
            $display("%s fflags mismatch: got %02x expected 00", name, fflags);
            $fatal;
         end
         if (result != expected) begin
            $display("%s result mismatch: got %016x expected %016x", name, result, expected);
            $fatal;
         end
         $display("%s passed", name);
      end
   endtask

   initial begin
      repeat (4) @(posedge clock);
      reset <= 1'b0;
      repeat (2) @(posedge clock);

      check_op("fadd.d",  4'd2, 1'b0, 3'd1, 3'd1, 2'd3,
               64'd0, 64'h4000000000000000, 64'h4008000000000000,
               64'h4014000000000000);
      check_op("fmul.d",  4'd3, 1'b0, 3'd1, 3'd1, 2'd3,
               64'h4000000000000000, 64'h4008000000000000, 64'd0,
               64'h4018000000000000);
      check_op("fdiv.d",  4'd4, 1'b0, 3'd1, 3'd1, 2'd3,
               64'h4018000000000000, 64'h4008000000000000, 64'd0,
               64'h4000000000000000);
      check_op("fsqrt.d", 4'd5, 1'b0, 3'd1, 3'd1, 2'd3,
               64'h4010000000000000, 64'd0, 64'd0,
               64'h4000000000000000);
      check_op("fmadd.d", 4'd0, 1'b0, 3'd1, 3'd1, 2'd3,
               64'h4000000000000000, 64'h4008000000000000, 64'h4010000000000000,
               64'h4024000000000000);
      check_op("fcvt.l.d", 4'd11, 1'b0, 3'd1, 3'd0, 2'd3,
               64'h4045000000000000, 64'd0, 64'd0,
               64'd42);
      check_op("fcvt.d.l", 4'd12, 1'b0, 3'd0, 3'd1, 2'd3,
               64'd42, 64'd0, 64'd0,
               64'h4045000000000000);

      $display("CVFPU smoke passed");
      $finish;
   end
endmodule
