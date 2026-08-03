`timescale 1ns/1ps
`default_nettype none

// Composition test for the full decode stage on a dependency-rich, mixed
// RVC/32-bit bundle (slot 0 = oldest):
//   s0: c.li   a0,5        (RVC)   -> addi a0,x0,5
//   s1: add    a1,a0,a0    (32b)   both srcs read s0's dest  -> SLOT(0),SLOT(0)
//   s2: c.mv   a2,a1       (RVC)   -> add a2,x0,a1 ; src2 reads s1 -> SLOT(1)
//   s3: addi   a0,a2,1     (32b)   src1 reads s2 -> SLOT(2); also writes a0 (WAW)
// So a0 (x10) is written by s0 and s3 -> only s3 is map_writer.
module tb;
   localparam IW=4, SEQW=8, ABITS=6, SBITS=2;
   reg  [IW*32-1:0]    inst;
   reg  [IW-1:0]       in_valid;
   reg  [IW*SEQW-1:0]  seq_in;
   wire [IW-1:0]       valid, is_rvc, rd_v, rs1_v, rs2_v, has_imm, legal;
   wire [IW-1:0]       s1_is_slot, s2_is_slot, map_writer;
   wire [IW*SEQW-1:0]  seq;
   wire [IW*32-1:0]    expanded;
   wire [IW*ABITS-1:0] rd, rs1, rs2;
   wire [IW*64-1:0]    imm;
   wire [IW*SBITS-1:0] s1_slot, s2_slot;
   integer errs = 0;

   decode_stage #(.IW(IW), .SEQW(SEQW), .ABITS(ABITS), .SBITS(SBITS)) dut
     (.inst(inst), .in_valid(in_valid), .seq_in(seq_in),
      .valid(valid), .seq(seq), .is_rvc(is_rvc), .expanded(expanded),
      .rd(rd), .rd_v(rd_v), .rs1(rs1), .rs1_v(rs1_v), .rs2(rs2), .rs2_v(rs2_v),
      .imm(imm), .has_imm(has_imm), .legal(legal),
      .s1_is_slot(s1_is_slot), .s1_slot(s1_slot),
      .s2_is_slot(s2_is_slot), .s2_slot(s2_slot), .map_writer(map_writer));

   function [IW*SBITS-1:0] smask(input [IW-1:0] is);
      integer k; begin smask=0;
         for (k=0;k<IW;k=k+1) smask[k*SBITS +: SBITS] = {SBITS{is[k]}}; end
   endfunction
   task ckv(input [127:0] name, input [IW-1:0] got, input [IW-1:0] exp);
      begin if (got!==exp) begin $display("FAIL %0s = %b exp %b", name, got, exp); errs=errs+1; end end
   endtask

   integer i;
   initial begin
      inst = {32'h00160513, 32'h0000862E, 32'h00A505B3, 32'h00004515};
      in_valid = 4'b1111;
      seq_in   = {8'd3, 8'd2, 8'd1, 8'd0};
      #1;

      ckv("valid",      valid,      4'b1111);
      ckv("legal",      legal,      4'b1111);
      ckv("rd_v",       rd_v,       4'b1111);
      ckv("rs1_v",      rs1_v,      4'b1111);
      ckv("rs2_v",      rs2_v,      4'b0110);
      ckv("is_rvc",     is_rvc,     4'b0101);
      ckv("has_imm",    has_imm,    4'b1001);   // s0 addi, s3 addi
      ckv("map_writer", map_writer, 4'b1110);   // s0 loses a0 to s3
      ckv("s1_is_slot", s1_is_slot, 4'b1010);   // s1<-s0, s3<-s2
      ckv("s2_is_slot", s2_is_slot, 4'b0110);   // s1<-s0, s2<-s1

      // destination arch regs
      if (rd[0*ABITS+:ABITS]!==6'd10) begin $display("FAIL rd0"); errs=errs+1; end
      if (rd[1*ABITS+:ABITS]!==6'd11) begin $display("FAIL rd1"); errs=errs+1; end
      if (rd[2*ABITS+:ABITS]!==6'd12) begin $display("FAIL rd2"); errs=errs+1; end
      if (rd[3*ABITS+:ABITS]!==6'd10) begin $display("FAIL rd3"); errs=errs+1; end
      // source arch regs that matter
      if (rs1[1*ABITS+:ABITS]!==6'd10) begin $display("FAIL rs1[1]"); errs=errs+1; end
      if (rs1[3*ABITS+:ABITS]!==6'd12) begin $display("FAIL rs1[3]"); errs=errs+1; end
      if (rs2[1*ABITS+:ABITS]!==6'd10) begin $display("FAIL rs2[1]"); errs=errs+1; end
      if (rs2[2*ABITS+:ABITS]!==6'd11) begin $display("FAIL rs2[2]"); errs=errs+1; end
      // slot indices (only where is_slot=1)
      if ((s1_slot & smask(4'b1010)) !== ({2'd2,2'd0,2'd0,2'd0} & smask(4'b1010)))
         begin $display("FAIL s1_slot=%b", s1_slot); errs=errs+1; end   // s3->2, s1->0
      if ((s2_slot & smask(4'b0110)) !== ({2'd0,2'd1,2'd0,2'd0} & smask(4'b0110)))
         begin $display("FAIL s2_slot=%b", s2_slot); errs=errs+1; end   // s2->1, s1->0
      // immediates
      if (imm[0*64+:64]!==64'd5) begin $display("FAIL imm0"); errs=errs+1; end
      if (imm[3*64+:64]!==64'd1) begin $display("FAIL imm3"); errs=errs+1; end
      // RVC expansion sanity
      if (expanded[0*32+:32]!==32'h00500513) begin $display("FAIL exp0=%h",expanded[0*32+:32]); errs=errs+1; end
      if (expanded[2*32+:32]!==32'h00b00633) begin $display("FAIL exp2=%h",expanded[2*32+:32]); errs=errs+1; end
      // seq carry-through
      if (seq[2*SEQW+:SEQW]!==8'd2) begin $display("FAIL seq2"); errs=errs+1; end

      if (errs==0) $display("decode_stage: ALL TESTS PASSED");
      else         $display("decode_stage: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
