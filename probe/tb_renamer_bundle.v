`timescale 1ns/1ps
`default_nettype none

// End-to-end check that the cross-slot matrix and the rename shards compose:
//  - intra-bundle RAW: a SLOT source gets the producer slot's freshly allocated pr
//  - independent ARCH source reads the MAP (init identity map[r]=r)
//  - cross-cycle: a destination's MAP write is visible to a reader next cycle
module tb;
   localparam SHARDS=4, ABITS=6, PBITS=8;

   reg                     clk = 0;
   always #5 clk = ~clk;

   reg  [SHARDS*ABITS-1:0] rs1, rs2, rd;
   reg  [SHARDS-1:0]       rs1_v, rs2_v, rd_v;
   reg                     reset=0, create=0, commit=0, rollback=0;
   reg  [1:0]              commit_idx=0, rollback_idx=0;
   wire [SHARDS*PBITS-1:0] ps1, ps2, pdst;
   wire [1:0]              cur;
   wire [SHARDS-1:0]       stall;
   integer errors = 0;
   reg  [PBITS-1:0]        cap;

   // cross-slot matrix now lives in decode; the bundle takes its result as
   // input, so the harness computes it (stimulus side, not RTL duplication).
   localparam SBITS=2;
   wire [SHARDS-1:0]       s1_is_slot, s2_is_slot, map_writer, d_is_slot;
   wire [SHARDS*SBITS-1:0] s1_slot, s2_slot, d_slot;
   decode_xslot #(.IW(SHARDS), .ABITS(ABITS), .SBITS(SBITS)) xs
     (.rs1(rs1), .rs1_v(rs1_v), .rs2(rs2), .rs2_v(rs2_v), .rd(rd), .rd_v(rd_v),
      .s1_is_slot(s1_is_slot), .s1_slot(s1_slot),
      .s2_is_slot(s2_is_slot), .s2_slot(s2_slot), .map_writer(map_writer),
      .d_is_slot(d_is_slot), .d_slot(d_slot));

   renamer_bundle dut
     (.clk(clk), .reset(reset), .rs1(rs1), .rs2(rs2), .rd(rd), .rd_v(rd_v),
      .s1_is_slot(s1_is_slot), .s1_slot(s1_slot),
      .s2_is_slot(s2_is_slot), .s2_slot(s2_slot), .map_writer(map_writer),
      .d_is_slot(d_is_slot), .d_slot(d_slot),
      .create(create), .commit(commit), .commit_idx(commit_idx),
      .rollback(rollback), .rollback_idx(rollback_idx),
      .ps1(ps1), .ps2(ps2), .pdst(pdst), .cur(cur), .stall(stall));

   // create=1 every cycle: each presented bundle is a dispatch (allocates + writes
   // the MAP). MAP update and allocation are create-gated in rename_shard now.
   task idle;
      begin rs1=0; rs1_v=0; rs2=0; rs2_v=0; rd=0; rd_v=0;
            create=1; commit=0; rollback=0; commit_idx=0; rollback_idx=0; end
   endtask

   initial begin
      idle;
      @(negedge clk); @(negedge clk);

      // T1: intra-bundle RAW + independent ARCH read, same bundle (combinational)
      @(negedge clk); idle;
      rd [0*ABITS +: ABITS] = 6'd5; rd_v[0] = 1'b1;   // slot0: rd=x5
      rs1[1*ABITS +: ABITS] = 6'd5; rs1_v[1] = 1'b1;  // slot1: rs1=x5 -> SLOT(0)
      rs1[2*ABITS +: ABITS] = 6'd9; rs1_v[2] = 1'b1;  // slot2: rs1=x9 (no producer)
      #1;
      if (ps1[1*PBITS +: PBITS] !== pdst[0*PBITS +: PBITS]) begin
         $display("T1 FAIL: ps1[1]=%0d != pdst[0]=%0d (RAW bypass)",
                  ps1[1*PBITS +: PBITS], pdst[0*PBITS +: PBITS]); errors=errors+1; end
      if (ps1[2*PBITS +: PBITS] !== 7'd9) begin
         $display("T1 FAIL: ps1[2]=%0d != 9 (ARCH identity read)",
                  ps1[2*PBITS +: PBITS]); errors=errors+1; end

      // T2: cross-cycle MAP update — write x5, read it back next cycle
      @(negedge clk); idle;
      rd[0*ABITS +: ABITS] = 6'd5; rd_v[0] = 1'b1;    // slot0 writes x5 (map_writer)
      #1; cap = pdst[0*PBITS +: PBITS];               // capture the allocated pr
      @(posedge clk);                                 // MAP[5] <= cap
      @(negedge clk); idle;
      rs1[0*ABITS +: ABITS] = 6'd5; rs1_v[0] = 1'b1;  // read x5 (ARCH, no producer)
      #1;
      if (ps1[0*PBITS +: PBITS] !== cap) begin
         $display("T2 FAIL: ps1[0]=%0d != captured pr %0d (MAP update)",
                  ps1[0*PBITS +: PBITS], cap); errors=errors+1; end

      // T3: WAW — slot0 and slot2 both write x7; a later reader sees slot2's pr
      @(negedge clk); idle;
      rd[0*ABITS +: ABITS] = 6'd7; rd_v[0] = 1'b1;
      rd[2*ABITS +: ABITS] = 6'd7; rd_v[2] = 1'b1;    // youngest writer of x7
      rs1[3*ABITS +: ABITS] = 6'd7; rs1_v[3] = 1'b1;  // slot3 reads x7 -> SLOT(2)
      #1;
      if (ps1[3*PBITS +: PBITS] !== pdst[2*PBITS +: PBITS]) begin
         $display("T3 FAIL: ps1[3]=%0d != pdst[2]=%0d (WAW youngest)",
                  ps1[3*PBITS +: PBITS], pdst[2*PBITS +: PBITS]); errors=errors+1; end
      @(posedge clk);                                 // MAP[7] <= pdst[2]
      cap = pdst[2*PBITS +: PBITS];
      @(negedge clk); idle;
      rs1[0*ABITS +: ABITS] = 6'd7; rs1_v[0] = 1'b1;  // read x7 next cycle
      #1;
      if (ps1[0*PBITS +: PBITS] !== cap) begin
         $display("T3b FAIL: ps1[0]=%0d != %0d (WAW winner committed to MAP)",
                  ps1[0*PBITS +: PBITS], cap); errors=errors+1; end

      @(negedge clk);
      if (errors == 0) $display("renamer_bundle: ALL TESTS PASSED");
      else             $display("renamer_bundle: %0d FAILURES", errors);
      $finish;
   end
endmodule

`default_nettype wire
