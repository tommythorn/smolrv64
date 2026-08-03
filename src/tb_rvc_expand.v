`timescale 1ns/1ps
`default_nettype none

// Exhaustive check of rvc_expand against the oracle table (tools/rvc.rs ->
// rvc_cases.hex). Drives all 65536 16-bit parcels and compares.
module tb;
   reg  [15:0] c;
   wire [31:0] insn;
   reg  [31:0] tbl [0:65535];
   integer i, errs = 0, shown = 0;

   rvc_expand dut (.c(c), .insn(insn));

   initial begin
      $readmemh("rvc_cases.hex", tbl);
      for (i = 0; i < 65536; i = i + 1) begin
         c = i[15:0]; #1;
         if (insn !== tbl[i]) begin
            errs = errs + 1;
            if (shown < 30) begin
               $display("MISS c=%04h  got=%08h  exp=%08h", i[15:0], insn, tbl[i]);
               shown = shown + 1;
            end
         end
      end
      if (errs == 0) $display("rvc_expand: ALL 65536 MATCH");
      else           $display("rvc_expand: %0d / 65536 MISMATCH", errs);
      $finish;
   end
endmodule

`default_nettype wire
