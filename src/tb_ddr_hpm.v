`default_nettype none
`timescale 1ns/1ps

// Unit test for ddr_hpm: drive request/ack with known latencies, check the read/write
// log2 histograms + sum + count via the MMIO read port, then clear and confirm zero.
module tb;
   reg clk = 0, reset = 1;
   reg ddr_req = 0, ddr_we = 0, ddr_ack = 0;
   reg [7:0]  raddr = 0;
   reg clr = 0;
   wire [63:0] rdata;
   integer errors = 0;

   ddr_hpm dut (.clk(clk), .reset(reset), .ddr_req(ddr_req), .ddr_we(ddr_we),
                .ddr_ack(ddr_ack), .raddr(raddr), .rdata(rdata), .clr(clr));

   always #5 clk = ~clk;

`ifdef DBG
   always @(posedge clk) if (dut.stb)
      $display("  REC lat=%0d we=%0d bin=%0d", dut.stb_lat, dut.stb_we, dut.bin);
`endif

   // drive one transaction of recorded latency L (>=1) in the given direction.
   // Inputs change on the NEGEDGE so they are stable at the posedge the DUT samples
   // (avoids the blocking-assign-vs-posedge testbench race).
   task do_txn(input we_i, input [11:0] L);
      integer c;
      begin
         @(negedge clk); ddr_we = we_i;
         for (c = 0; c <= L; c = c + 1) begin
            @(negedge clk);
            ddr_req = 1'b1;
            ddr_ack = (c == L);
         end
         @(negedge clk); ddr_req = 1'b0; ddr_ack = 1'b0;
      end
   endtask

   // combinational read check at byte offset off
   task chk(input [7:0] off, input [63:0] exp, input [127:0] name);
      begin
         raddr = off; #1;
         if (rdata !== exp) begin
            $display("FAIL %0s @off=0x%02h: got %0d exp %0d", name, off, rdata, exp);
            errors = errors + 1;
         end
      end
   endtask

   initial begin
      repeat (3) @(posedge clk);
      reset = 0;
      @(posedge clk);

      do_txn(1'b0, 12'd5);    // read,  bin3 (4-7)
      do_txn(1'b1, 12'd20);   // write, bin5 (16-31)
      do_txn(1'b0, 12'd1);    // read,  bin1
      do_txn(1'b0, 12'd100);  // read,  bin7 (64+)
      @(posedge clk);

      // read histogram: bins are at off = bin*8 (read) / 0x40+bin*8 (write)
      chk(8'h08, 64'd1, "rd_bin1");
      chk(8'h18, 64'd1, "rd_bin3");
      chk(8'h38, 64'd1, "rd_bin7");
      chk(8'h00, 64'd0, "rd_bin0");
      chk(8'h10, 64'd0, "rd_bin2");
      chk(8'h68, 64'd1, "wr_bin5");      // 0x40 + 5*8 = 0x68
      chk(8'h40, 64'd0, "wr_bin0");
      // sums / counts: 0x80 rd_sum, 0x88 wr_sum, 0x90 rd_cnt, 0x98 wr_cnt
      chk(8'h80, 64'd106, "rd_sum");     // 5 + 1 + 100
      chk(8'h88, 64'd20,  "wr_sum");
      chk(8'h90, 64'd3,   "rd_cnt");
      chk(8'h98, 64'd1,   "wr_cnt");

      // clear, then confirm zeroed
      @(negedge clk); clr = 1'b1; @(negedge clk); clr = 1'b0; @(posedge clk);
      chk(8'h90, 64'd0, "rd_cnt_after_clr");
      chk(8'h18, 64'd0, "rd_bin3_after_clr");
      chk(8'h68, 64'd0, "wr_bin5_after_clr");

      if (errors == 0) $display("tb_ddr_hpm: PASS");
      else             $display("tb_ddr_hpm: FAIL (%0d errors)", errors);
      $finish;
   end
endmodule

`default_nettype wire
