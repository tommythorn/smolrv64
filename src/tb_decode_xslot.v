`timescale 1ns/1ps
`default_nettype none

// Self-checking directed test for decode_xslot. Slot 0 = oldest.
// Bus layout: slot k occupies bits [k*W +: W]; concatenation is {slot3..slot0}.
module tb;
   localparam IW = 4, ABITS = 6, SBITS = 2;

   reg  [IW*ABITS-1:0] rs1, rs2, rd;
   reg  [IW-1:0]       rs1_v, rs2_v, rd_v;
   wire [IW-1:0]       s1_is_slot, s2_is_slot, map_writer, d_is_slot;
   wire [IW*SBITS-1:0] s1_slot, s2_slot, d_slot;
   integer errors = 0, tid = 0;

   decode_xslot #(.IW(IW), .ABITS(ABITS), .SBITS(SBITS)) dut
     (.rs1(rs1), .rs1_v(rs1_v), .rs2(rs2), .rs2_v(rs2_v), .rd(rd), .rd_v(rd_v),
      .s1_is_slot(s1_is_slot), .s1_slot(s1_slot),
      .s2_is_slot(s2_is_slot), .s2_slot(s2_slot), .map_writer(map_writer),
      .d_is_slot(d_is_slot), .d_slot(d_slot));

   // expand an is_slot vector to a per-slot SBITS mask (slot index is don't-care
   // where is_slot==0).
   function [IW*SBITS-1:0] smask(input [IW-1:0] is);
      integer k;
      begin
         smask = 0;
         for (k = 0; k < IW; k = k + 1) smask[k*SBITS +: SBITS] = {SBITS{is[k]}};
      end
   endfunction

   task chk(input [IW-1:0] e1is, input [IW*SBITS-1:0] e1sl,
            input [IW-1:0] e2is, input [IW*SBITS-1:0] e2sl, input [IW-1:0] emw,
            input [IW-1:0] edis, input [IW*SBITS-1:0] edsl);
      begin
         #1; tid = tid + 1;
         if (s1_is_slot !== e1is) begin
            $display("T%0d FAIL s1_is_slot=%b exp %b", tid, s1_is_slot, e1is); errors=errors+1; end
         if ((s1_slot & smask(e1is)) !== (e1sl & smask(e1is))) begin
            $display("T%0d FAIL s1_slot=%b exp %b", tid, s1_slot, e1sl); errors=errors+1; end
         if (s2_is_slot !== e2is) begin
            $display("T%0d FAIL s2_is_slot=%b exp %b", tid, s2_is_slot, e2is); errors=errors+1; end
         if ((s2_slot & smask(e2is)) !== (e2sl & smask(e2is))) begin
            $display("T%0d FAIL s2_slot=%b exp %b", tid, s2_slot, e2sl); errors=errors+1; end
         if (map_writer !== emw) begin
            $display("T%0d FAIL map_writer=%b exp %b", tid, map_writer, emw); errors=errors+1; end
         if (d_is_slot !== edis) begin
            $display("T%0d FAIL d_is_slot=%b exp %b", tid, d_is_slot, edis); errors=errors+1; end
         if ((d_slot & smask(edis)) !== (edsl & smask(edis))) begin
            $display("T%0d FAIL d_slot=%b exp %b", tid, d_slot, edsl); errors=errors+1; end
      end
   endtask

   initial begin
      // T1: fully independent -> no slots, all dests are map writers
      rd={6'd4,6'd3,6'd2,6'd1};   rd_v=4'b1111;
      rs1={6'd20,6'd21,6'd22,6'd23}; rs1_v=4'b1111;
      rs2={6'd30,6'd31,6'd32,6'd33}; rs2_v=4'b1111;
      chk(4'b0000, 8'h00, 4'b0000, 8'h00, 4'b1111, 4'b0000, 8'h00);

      // T2: RAW slot1.rs1 reads slot0.rd(=5)
      rd={6'd0,6'd0,6'd6,6'd5}; rd_v=4'b0011;
      rs1={6'd0,6'd0,6'd5,6'd9}; rs1_v=4'b0011;
      rs2=0; rs2_v=0;
      chk(4'b0010, 8'h00, 4'b0000, 8'h00, 4'b0011, 4'b0000, 8'h00);

      // T3: chain s0:rd5, s1:rd6 rs1=5, s2:rd7 rs1=6  -> s1<-slot0, s2<-slot1
      rd={6'd0,6'd7,6'd6,6'd5}; rd_v=4'b0111;
      rs1={6'd0,6'd6,6'd5,6'd0}; rs1_v=4'b0110;
      rs2=0; rs2_v=0;
      chk(4'b0110, 8'h10, 4'b0000, 8'h00, 4'b0111, 4'b0000, 8'h00);  // slot2->1, slot1->0

      // T4: WAW s0:rd5, s2:rd5; s1.rs1=5->slot0, s3.rs1=5->slot2
      rd={6'd0,6'd5,6'd0,6'd5}; rd_v=4'b0101;
      rs1={6'd5,6'd0,6'd5,6'd0}; rs1_v=4'b1010;
      rs2=0; rs2_v=0;
      chk(4'b1010, 8'h80, 4'b0000, 8'h00, 4'b0100, 4'b0100, 8'h00); // slot3->2(10), slot1->0; mw only slot2; pold slot2<-0

      // T5: youngest-earlier  s0:rd5, s1:rd5, s2.rs1=5 -> slot1
      rd={6'd0,6'd0,6'd5,6'd5}; rd_v=4'b0011;
      rs1={6'd0,6'd5,6'd0,6'd0}; rs1_v=4'b0100;
      rs2=0; rs2_v=0;
      chk(4'b0100, 8'h10, 4'b0000, 8'h00, 4'b0010, 4'b0010, 8'h00); // slot2->1; mw only slot1; pold slot1<-0

      // T6: invalid source is never a slot (rs1_v low) even with a producer
      rd={6'd0,6'd0,6'd0,6'd5}; rd_v=4'b0001;
      rs1={6'd0,6'd0,6'd5,6'd0}; rs1_v=4'b0000;
      rs2=0; rs2_v=0;
      chk(4'b0000, 8'h00, 4'b0000, 8'h00, 4'b0001, 4'b0000, 8'h00);

      // T7: invalid producer (rd_v low, e.g. store/x0) yields no slot
      rd={6'd0,6'd0,6'd0,6'd5}; rd_v=4'b0000;
      rs1={6'd0,6'd0,6'd5,6'd0}; rs1_v=4'b0010;
      rs2=0; rs2_v=0;
      chk(4'b0000, 8'h00, 4'b0000, 8'h00, 4'b0000, 4'b0000, 8'h00);

      // T8: src2 path  s0:rd5, s2.rs2=5 -> slot0
      rd={6'd0,6'd0,6'd0,6'd5}; rd_v=4'b0001;
      rs1=0; rs1_v=0;
      rs2={6'd0,6'd5,6'd0,6'd0}; rs2_v=4'b0100;
      chk(4'b0000, 8'h00, 4'b0100, 8'h00, 4'b0001, 4'b0000, 8'h00);

      // T9: both sources of one slot depend on the same earlier producer
      rd={6'd0,6'd0,6'd0,6'd5}; rd_v=4'b0001;
      rs1={6'd0,6'd0,6'd5,6'd0}; rs1_v=4'b0010;
      rs2={6'd0,6'd0,6'd5,6'd0}; rs2_v=4'b0010;
      chk(4'b0010, 8'h00, 4'b0010, 8'h00, 4'b0001, 4'b0000, 8'h00);

      if (errors == 0) $display("decode_xslot: ALL TESTS PASSED (%0d)", tid);
      else             $display("decode_xslot: %0d FAILURES / %0d tests", errors, tid);
      $finish;
   end
endmodule

`default_nettype wire
