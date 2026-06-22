`timescale 1ns/1ps
`default_nettype none

// End-to-end decode -> [registered boundary] -> rename.
//
// Bundle 1 (dep-rich, mixed RVC/32b; slot 0 = oldest), same as tb_decode_stage:
//   s0: c.li  a0,5      -> addi a0,x0,5     writes a0
//   s1: add   a1,a0,a0  -> rs1,rs2 = SLOT(0)
//   s2: c.mv  a2,a1     -> add a2,x0,a1 ; rs2 = SLOT(1)
//   s3: addi  a0,a2,1   -> rs1 = SLOT(2) ; writes a0 (WAW; only s3 is map_writer)
// Pools start fl[head]=SH, so the four shards allocate pdst = [0,1,2,3].
// SLOT sources must read those freshly-allocated pdsts (intra-bundle RAW).
//
// Bundle 2 (independent ARCH reads of a0/a1/a2) must see Bundle 1's MAP writes
// one rename cycle later: map[a0]=pdst1[3]=3, map[a1]=pdst1[1]=1, map[a2]=2.
module tb;
   localparam IW=4, SEQW=8, ABITS=6, PBITS=8, SBITS=2;

   reg                    clk = 0;
   always #5 clk = ~clk;

   reg  [IW*32-1:0]   inst;
   reg  [IW-1:0]      in_valid;
   reg  [IW*SEQW-1:0] seq_in;
   reg  [IW*PBITS-1:0] fr_phys;
   reg  [IW-1:0]      fr_valid;
   reg                chk_create=0, chk_restore=0;
   reg  [1:0]         chk_create_idx=0, chk_restore_idx=0;

   wire [IW-1:0]      r_valid, r_rd_v, stall;
   wire [IW*SEQW-1:0] r_seq;
   wire [IW*ABITS-1:0] r_rd;
   wire [IW*PBITS-1:0] ps1, ps2, pdst;
   integer errs = 0;

   decode_rename #(.IW(IW), .SEQW(SEQW), .ABITS(ABITS), .PBITS(PBITS), .SBITS(SBITS)) dut
     (.clk(clk), .reset(1'b0), .flush(1'b0), .inst(inst), .in_valid(in_valid), .seq_in(seq_in),
      .fr_phys(fr_phys), .fr_valid(fr_valid),
      .chk_create(chk_create), .chk_create_idx(chk_create_idx),
      .chk_restore(chk_restore), .chk_restore_idx(chk_restore_idx),
      .r_valid(r_valid), .r_seq(r_seq), .r_rd(r_rd), .r_rd_v(r_rd_v),
      .ps1(ps1), .ps2(ps2), .pdst(pdst), .stall(stall));

   function [PBITS-1:0] P(input [IW*PBITS-1:0] bus, input integer k);
      P = bus[k*PBITS +: PBITS];
   endfunction
   task ckp(input [127:0] nm, input [PBITS-1:0] got, exp);
      begin if (got!==exp) begin $display("FAIL %0s = %0d exp %0d", nm, got, exp); errs=errs+1; end end
   endtask

   task idle; begin inst=0; in_valid=0; seq_in=0; end endtask

   initial begin
      idle; fr_phys=0; fr_valid=0;
      @(negedge clk); @(negedge clk);

      // present Bundle 1 (decode comb; latched at next posedge)
      @(negedge clk);
      inst = {32'h00160513, 32'h0000862E, 32'h00A505B3, 32'h00004515};
      in_valid = 4'b1111;
      seq_in   = {8'd3, 8'd2, 8'd1, 8'd0};

      @(posedge clk);                 // q <= Bundle 1
      @(negedge clk);
      // present Bundle 2 while Bundle 1 is in rename (its MAP writes land next edge)
      inst = {32'h00000013, 32'h00060793, 32'h00058713, 32'h00050693}; // addi a3,a0 / a4,a1 / a5,a2 / nop
      in_valid = 4'b1111;
      seq_in   = {8'd7, 8'd6, 8'd5, 8'd4};
      #1;
      // ---- check Bundle 1's renamed outputs (q now holds Bundle 1) ----
      if (r_valid!==4'b1111) begin $display("FAIL B1 r_valid=%b",r_valid); errs=errs+1; end
      if (r_rd_v !==4'b1111) begin $display("FAIL B1 r_rd_v=%b",r_rd_v); errs=errs+1; end
      // shard i's first allocation = AREGS + i = 64 + i (phys 0..63 reserved for arch)
      ckp("B1 pdst0", P(pdst,0), 8'd64);
      ckp("B1 pdst1", P(pdst,1), 8'd65);
      ckp("B1 pdst2", P(pdst,2), 8'd66);
      ckp("B1 pdst3", P(pdst,3), 8'd67);
      // intra-bundle RAW (SLOT) must resolve to producer pdsts
      ckp("B1 ps1[1]<-pdst0", P(ps1,1), P(pdst,0));
      ckp("B1 ps2[1]<-pdst0", P(ps2,1), P(pdst,0));
      ckp("B1 ps2[2]<-pdst1", P(ps2,2), P(pdst,1));
      ckp("B1 ps1[3]<-pdst2", P(ps1,3), P(pdst,2));
      // dest arch carried through the boundary, aligned with seq
      ckp("B1 r_rd0", r_rd[0*ABITS+:ABITS], 6'd10);
      ckp("B1 r_rd3", r_rd[3*ABITS+:ABITS], 6'd10);
      if (r_seq[0*SEQW+:SEQW]!==8'd0 || r_seq[3*SEQW+:SEQW]!==8'd3)
         begin $display("FAIL B1 r_seq"); errs=errs+1; end

      @(posedge clk);                 // q <= Bundle 2 ; MAP <= Bundle 1 writes
      @(negedge clk); idle;
      #1;
      // ---- check Bundle 2 reads Bundle 1's MAP through the registered path ----
      if (r_valid!==4'b1111) begin $display("FAIL B2 r_valid=%b",r_valid); errs=errs+1; end
      ckp("B2 ps1[0]=map[a0]", P(ps1,0), 8'd67);  // s3 was a0's map_writer -> pdst1[3]=67
      ckp("B2 ps1[1]=map[a1]", P(ps1,1), 8'd65);
      ckp("B2 ps1[2]=map[a2]", P(ps1,2), 8'd66);

      @(negedge clk);
      if (errs==0) $display("decode_rename: ALL TESTS PASSED");
      else         $display("decode_rename: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
