`default_nettype none
`timescale 1ns/1ps

// Unit test for ddr_hpm: accept requests and complete them with known latencies -- one at a
// time, then two reads overlapping and completing out of order, and a read and a write
// completing in the same cycle -- and check the read/write log2 histograms + sum + count via
// the MMIO read port, then clear and confirm zero.
module tb;
   reg clk = 0, reset = 1;
   reg q_fire = 0, r_done = 0, w_done = 0;
   reg [4:0] q_id = 0, r_id = 0, w_id = 0;
   reg [7:0]  raddr = 0;
   reg clr = 0;
   wire [63:0] rdata;
   integer errors = 0;

   ddr_hpm dut (.clk(clk), .reset(reset), .q_fire(q_fire), .q_id(q_id), .r_done(r_done),
                .r_id(r_id), .w_done(w_done), .w_id(w_id), .raddr(raddr), .rdata(rdata), .clr(clr));

   always #5 clk = ~clk;

   // Inputs change on the NEGEDGE so they are stable at the posedge the DUT samples.
   task accept(input [4:0] id);
      begin @(negedge clk); q_fire = 1'b1; q_id = id; @(negedge clk); q_fire = 1'b0; end
   endtask
   task finish_rd(input [4:0] id);
      begin r_done = 1'b1; r_id = id; @(negedge clk); r_done = 1'b0; end
   endtask
   // one transaction whose completion comes L cycles after its acceptance
   task do_txn(input we_i, input [4:0] id, input integer L);
      integer c;
      begin
         @(negedge clk); q_fire = 1'b1; q_id = id;
         @(negedge clk); q_fire = 1'b0;
         for (c = 1; c < L; c = c + 1) @(negedge clk);
         if (we_i) begin w_done = 1'b1; w_id = id; end else begin r_done = 1'b1; r_id = id; end
         @(negedge clk); r_done = 1'b0; w_done = 1'b0;
      end
   endtask

   task chk(input [7:0] off, input [63:0] exp, input [127:0] name);
      begin
         raddr = off; #1;
         if (rdata !== exp) begin
            $display("FAIL %0s @off=0x%02h: got %0d exp %0d", name, off, rdata, exp);
            errors = errors + 1;
         end
      end
   endtask

   integer c;
   initial begin
      repeat (3) @(posedge clk);
      reset = 0;
      @(posedge clk);

      do_txn(1'b0, 5'd3, 5);     // read,  bin3 (4-7)
      do_txn(1'b1, 5'd4, 20);    // write, bin5 (16-31)
      do_txn(1'b0, 5'd3, 1);     // read,  bin1
      do_txn(1'b0, 5'd7, 100);   // read,  bin7 (64+)
      // two reads overlapping: id 1 accepted, id 2 two cycles later; id 2 completes first
      @(negedge clk); q_fire = 1'b1; q_id = 5'd1;          // accepted at T
      @(negedge clk); q_fire = 1'b0;
      @(negedge clk); q_fire = 1'b1; q_id = 5'd2;          // accepted at T+2
      @(negedge clk); q_fire = 1'b0;
      for (c = 3; c < 9; c = c + 1) @(negedge clk);
      finish_rd(5'd2);                                     // T+9: latency 7, bin3
      finish_rd(5'd1);                                     // T+10: latency 10, bin4
      // a read and a write completing in the same cycle
      @(negedge clk); q_fire = 1'b1; q_id = 5'd5;
      @(negedge clk); q_id = 5'd6;
      @(negedge clk); q_fire = 1'b0;
      for (c = 2; c < 12; c = c + 1) @(negedge clk);
      r_done = 1'b1; r_id = 5'd5;                          // accepted at U, done at U+12: 12
      w_done = 1'b1; w_id = 5'd6;                          // accepted at U+1, done at U+12: 11
      @(negedge clk); r_done = 1'b0; w_done = 1'b0;
      @(posedge clk); @(posedge clk);

      chk(8'h08, 64'd1, "rd_bin1");
      chk(8'h18, 64'd2, "rd_bin3");      // 5, 7
      chk(8'h20, 64'd2, "rd_bin4");      // 10, 12
      chk(8'h38, 64'd1, "rd_bin7");
      chk(8'h00, 64'd0, "rd_bin0");
      chk(8'h10, 64'd0, "rd_bin2");
      chk(8'h68, 64'd1, "wr_bin5");      // 0x40 + 5*8
      chk(8'h60, 64'd1, "wr_bin4");      // 11
      chk(8'h40, 64'd0, "wr_bin0");
      chk(8'h80, 64'd135, "rd_sum");     // 5 + 1 + 100 + 7 + 10 + 12
      chk(8'h88, 64'd31,  "wr_sum");     // 20 + 11
      chk(8'h90, 64'd6,   "rd_cnt");
      chk(8'h98, 64'd2,   "wr_cnt");

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
