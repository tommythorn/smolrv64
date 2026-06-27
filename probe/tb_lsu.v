`timescale 1ns/1ps
`default_nettype none

// Directed test for the M1 unified LSU. Flat byte-addressable memory model.
// Covers: store -> commit -> drain to memory; byte-granular forwarding with TWO
// overlapping older stores + cache fill (the hard case); the resolved-store
// ordering gate (a load blocked by an unfilled older store); and rollback squash
// of wrong-path store/load by seqno.
module tb;
   localparam IW=4, SBITS=2, PBITS=8, SEQW=8, CBITS=2, AW=64, SBDEPTH=8, SBI=3, LQDEPTH=8, LQI=3;

   reg               clk=0; always #5 clk=~clk;
   reg               reset;
   reg               disp_fire;
   reg  [IW-1:0]     disp_is_load, disp_is_store;
   reg  [IW*SEQW-1:0] disp_seq;
   reg  [IW*CBITS-1:0] disp_ckpt;
   reg  [IW*PBITS-1:0] disp_pdst;
   wire [IW*SBI-1:0] disp_sb_idx;
   wire [IW*LQI-1:0] disp_lq_idx;
   wire              sb_full, lq_full;
   reg  [IW-1:0]     exe_st_v;
   reg  [IW*SBI-1:0] exe_st_idx;
   reg  [IW*AW-1:0]  exe_st_addr;
   reg  [IW*64-1:0]  exe_st_data;
   reg  [IW*4-1:0]   exe_st_nb;
   reg  [IW-1:0]     exe_ld_v;
   reg  [IW*LQI-1:0] exe_ld_idx;
   reg  [IW*AW-1:0]  exe_ld_addr;
   reg  [IW*4-1:0]   exe_ld_nb;
   reg  [IW-1:0]     exe_ld_sgn;
   reg  [IW-1:0]     exe_ld_fp;
   wire [AW-1:0]     mem_raddr;
   wire [63:0]       mem_rdata;
   wire              mem_wen;
   wire [AW-1:0]     mem_waddr;
   wire [63:0]       mem_wdata;
   wire [7:0]        mem_wmask;
   wire             ld_wb_v;
   wire [PBITS-1:0] ld_wb_pdst;
   wire [SBITS-1:0] ld_wb_owner;
   wire [63:0]      ld_wb_val;
   wire             ld_done;
   wire [CBITS-1:0] ld_done_ckpt;
   reg              commit;
   reg  [CBITS-1:0] commit_idx;
   reg              rollback;
   reg  [SEQW-1:0]  rollback_seq;
   integer errs=0;

   // DEV_TOP=0: this TB's flat memory lives at low addresses (0..255) which the real
   // platform maps to MMIO; disable the device non-speculative gate so the TB keeps
   // testing forwarding/blocking on those addresses unchanged.
   lsu #(.IW(IW), .SBITS(SBITS), .PBITS(PBITS), .SEQW(SEQW), .CBITS(CBITS), .AW(AW),
         .SBDEPTH(SBDEPTH), .SBI(SBI), .LQDEPTH(LQDEPTH), .LQI(LQI), .DEV_TOP(64'd0)) dut
     (.clk(clk), .reset(reset),
      .disp_fire(disp_fire), .disp_is_load(disp_is_load), .disp_is_store(disp_is_store),
      .disp_seq(disp_seq), .disp_ckpt(disp_ckpt), .disp_pdst(disp_pdst),
      .disp_sb_idx(disp_sb_idx), .disp_lq_idx(disp_lq_idx), .sb_full(sb_full), .lq_full(lq_full),
      .exe_st_v(exe_st_v), .exe_st_idx(exe_st_idx), .exe_st_addr(exe_st_addr),
      .exe_st_data(exe_st_data), .exe_st_nb(exe_st_nb),
      .exe_ld_v(exe_ld_v), .exe_ld_idx(exe_ld_idx), .exe_ld_addr(exe_ld_addr),
      .exe_ld_nb(exe_ld_nb), .exe_ld_sgn(exe_ld_sgn), .exe_ld_fp(exe_ld_fp),
      .amo_v(1'b0), .amo_func(5'd0), .amo_addr({AW{1'b0}}), .amo_data(64'd0),
      .amo_sz(2'd0), .amo_pdst({PBITS{1'b0}}), .amo_owner({SBITS{1'b0}}), .amo_ckpt({CBITS{1'b0}}),
      .xl_satp(64'd0), .xl_priv(2'd0), .xl_sum(1'b0), .xl_mxr(1'b0), .xl_flush(1'b0),
      .ldp_addr(), .ldp_read(), .ldp_rdata(64'd0), .ldp_rvalid(1'b0),
      .stp_addr(), .stp_read(), .stp_rdata(64'd0), .stp_rvalid(1'b0),
      .dfault_v(), .dfault_seq(), .dfault_ckpt(), .dfault_cause(),
      .mem_raddr(mem_raddr), .mem_rdata(mem_rdata), .mem_rvalid(1'b1), .mem_wready(1'b1),
      .mem_wen(mem_wen), .mem_waddr(mem_waddr), .mem_wdata(mem_wdata), .mem_wmask(mem_wmask),
      .wb_busy({IW{1'b0}}),
      .ld_wb_v(ld_wb_v), .ld_wb_pdst(ld_wb_pdst), .ld_wb_owner(ld_wb_owner),
      .ld_wb_val(ld_wb_val), .ld_done(ld_done), .ld_done_ckpt(ld_done_ckpt),
      .commit(commit), .commit_idx(commit_idx), .committed(2'd0),
      .rollback(rollback), .rollback_seq(rollback_seq));

   // ---- flat byte-addressable memory model ----
   reg [7:0] mem [0:255];
   wire [7:0] ra = mem_raddr[7:0];
   assign mem_rdata = {mem[ra+7],mem[ra+6],mem[ra+5],mem[ra+4],
                       mem[ra+3],mem[ra+2],mem[ra+1],mem[ra+0]};
   integer mb;
   always @(posedge clk)
      if (mem_wen)
         for (mb=0; mb<8; mb=mb+1)
            if (mem_wmask[mb]) mem[mem_waddr[7:0]+mb] <= mem_wdata[mb*8 +: 8];

   // capture the WB stream (count-based, timing-robust)
   reg [63:0] last_wb_val; reg [PBITS-1:0] last_wb_pd; integer wb_count, wbc0;
   initial wb_count = 0;
   always @(posedge clk) if (ld_wb_v) begin
      last_wb_val <= ld_wb_val; last_wb_pd <= ld_wb_pdst; wb_count <= wb_count + 1;
   end

   task idle; begin
      disp_fire=0; disp_is_load=0; disp_is_store=0; disp_seq=0; disp_ckpt=0; disp_pdst=0;
      exe_st_v=0; exe_st_idx=0; exe_st_addr=0; exe_st_data=0; exe_st_nb=0;
      exe_ld_v=0; exe_ld_idx=0; exe_ld_addr=0; exe_ld_nb=0; exe_ld_sgn=0; exe_ld_fp=0;
      commit=0; commit_idx=0; rollback=0; rollback_seq=0;
   end endtask

   task ckh(input [127:0] nm, input [63:0] got, exp);
      begin if (got!==exp) begin $display("FAIL %0s = %h exp %h",nm,got,exp); errs=errs+1; end end
   endtask

   integer i;
   reg [SBI-1:0] idxA, idxB, idxS;
   reg [LQI-1:0] idxL;

   initial begin
      for (i=0;i<256;i=i+1) mem[i] = 8'hC0 + i[7:0];   // identifiable background
      idle; reset=1; @(negedge clk); @(negedge clk); reset=0; @(negedge clk);

      // ================= T1: store -> commit -> drain to memory =================
      // store seq=1 ck=0 at addr 0x40, 4 bytes = 0x44332211
      disp_fire=1; disp_is_store=4'b0001; disp_seq[0+:SEQW]=8'd1; disp_ckpt[0+:CBITS]=2'd0;
      #1; idxS = disp_sb_idx[0+:SBI];
      @(posedge clk); idle;
      @(negedge clk);
      exe_st_v=4'b0001; exe_st_idx[0+:SBI]=idxS; exe_st_addr[0+:AW]=64'h40;
      exe_st_data[0+:64]=64'h44332211; exe_st_nb[0+:4]=4'd4;
      @(posedge clk); idle;
      @(negedge clk); commit=1; commit_idx=2'd0;       // commit ckpt 0 -> store drainable
      @(posedge clk); idle;
      repeat (3) @(posedge clk);                        // let it drain
      ckh("T1 mem40", {mem[8'h43],mem[8'h42],mem[8'h41],mem[8'h40]}, 64'h44332211);

      // ============ T2: two overlapping older stores + cache fill ============
      // load[0..7]@0x80 ; store A(seq=10)@0x80 nb4 =0xA3A2A1A0 ; store B(seq=11)@0x82 nb2 =0xB3B2
      // expect bytes: A0 A1 B2 B3 mem[84] mem[85] mem[86] mem[87]
      // store A
      @(negedge clk); disp_fire=1; disp_is_store=4'b0001; disp_seq[0+:SEQW]=8'd10; disp_ckpt[0+:CBITS]=2'd1;
      #1; idxA=disp_sb_idx[0+:SBI]; @(posedge clk); idle;
      @(negedge clk); exe_st_v=4'b0001; exe_st_idx[0+:SBI]=idxA; exe_st_addr[0+:AW]=64'h80;
      exe_st_data[0+:64]=64'hA3A2A1A0; exe_st_nb[0+:4]=4'd4; @(posedge clk); idle;
      // store B
      @(negedge clk); disp_fire=1; disp_is_store=4'b0001; disp_seq[0+:SEQW]=8'd11; disp_ckpt[0+:CBITS]=2'd1;
      #1; idxB=disp_sb_idx[0+:SBI]; @(posedge clk); idle;
      @(negedge clk); exe_st_v=4'b0001; exe_st_idx[0+:SBI]=idxB; exe_st_addr[0+:AW]=64'h82;
      exe_st_data[0+:64]=64'h0000B3B2; exe_st_nb[0+:4]=4'd2; @(posedge clk); idle;
      // load
      @(negedge clk); disp_fire=1; disp_is_load=4'b0001; disp_seq[0+:SEQW]=8'd12; disp_ckpt[0+:CBITS]=2'd1;
      disp_pdst[0+:PBITS]=8'd70; #1; idxL=disp_lq_idx[0+:LQI]; @(posedge clk); idle;
      @(negedge clk); exe_ld_v=4'b0001; exe_ld_idx[0+:LQI]=idxL; exe_ld_addr[0+:AW]=64'h80;
      exe_ld_nb[0+:4]=4'd8; exe_ld_sgn[0]=1'b0; @(posedge clk); idle;
      // wait for WB
      repeat (5) @(posedge clk);
      ckh("T2 fwd merge",
          last_wb_val,
          {mem[8'h87],mem[8'h86],mem[8'h85],mem[8'h84], 8'hB3,8'hB2,8'hA1,8'hA0});

      // ================= T3: ordering gate (older unfilled store) =================
      // dispatch store seq=20 (DON'T fill it yet), then load seq=21@0x90 -> blocked.
      @(negedge clk); disp_fire=1; disp_is_store=4'b0001; disp_seq[0+:SEQW]=8'd20; disp_ckpt[0+:CBITS]=2'd2;
      #1; idxS=disp_sb_idx[0+:SBI]; @(posedge clk); idle;
      @(negedge clk); disp_fire=1; disp_is_load=4'b0001; disp_seq[0+:SEQW]=8'd21; disp_ckpt[0+:CBITS]=2'd2;
      disp_pdst[0+:PBITS]=8'd71; #1; idxL=disp_lq_idx[0+:LQI]; @(posedge clk); idle;
      @(negedge clk); exe_ld_v=4'b0001; exe_ld_idx[0+:LQI]=idxL; exe_ld_addr[0+:AW]=64'h90;
      exe_ld_nb[0+:4]=4'd4; @(posedge clk); idle;
      // load is rdy but blocked by unfilled older store -> no WB for a few cycles
      wbc0 = wb_count;
      repeat (4) @(posedge clk);
      if (wb_count != wbc0) begin $display("FAIL T3: load issued while older store unresolved"); errs=errs+1; end
      // now fill the older store (seq=20)@0x94 nb4 (no overlap with load@0x90) -> load unblocks, reads memory
      @(negedge clk); exe_st_v=4'b0001; exe_st_idx[0+:SBI]=idxS; exe_st_addr[0+:AW]=64'h94;
      exe_st_data[0+:64]=64'hDEADBEEF; exe_st_nb[0+:4]=4'd4; @(posedge clk); idle;
      repeat (5) @(posedge clk);
      if (wb_count != wbc0+1) begin $display("FAIL T3: load did not complete after unblock"); errs=errs+1; end
      ckh("T3 unblocked load", last_wb_val, {32'd0, mem[8'h93],mem[8'h92],mem[8'h91],mem[8'h90]});

      // ================= T4: rollback squashes wrong-path store/load =================
      // store seq=30 ck=3 @0xA0; then rollback to seq=29 (squash >29). It must NOT drain.
      @(negedge clk); disp_fire=1; disp_is_store=4'b0001; disp_seq[0+:SEQW]=8'd30; disp_ckpt[0+:CBITS]=2'd3;
      #1; idxS=disp_sb_idx[0+:SBI]; @(posedge clk); idle;
      @(negedge clk); exe_st_v=4'b0001; exe_st_idx[0+:SBI]=idxS; exe_st_addr[0+:AW]=64'hA0;
      exe_st_data[0+:64]=64'hCAFEBABE; exe_st_nb[0+:4]=4'd4; @(posedge clk); idle;
      @(negedge clk); rollback=1; rollback_seq=8'd29; @(posedge clk); idle;
      // even if we (wrongly) commit ck3, the squashed entry is gone -> memory unchanged
      @(negedge clk); commit=1; commit_idx=2'd3; @(posedge clk); idle;
      repeat (3) @(posedge clk);
      ckh("T4 squashed store not drained",
          {mem[8'hA3],mem[8'hA2],mem[8'hA1],mem[8'hA0]},
          {8'hC0+8'hA3, 8'hC0+8'hA2, 8'hC0+8'hA1, 8'hC0+8'hA0});

      // ========== T5: a squashed wrong-path load never writes back ==========
      // The byte-merge result is now registered (its own stage), and a squash is gated
      // at BOTH selection (ld_sel) and presentation (r_kill). Hold a rollback that
      // targets the load across its whole completion window and confirm it never
      // asserts ld_wb_v / ld_done / advances wb_count. load seq=40 @0xB0, rollback->39.
      @(negedge clk); disp_fire=1; disp_is_load=4'b0001; disp_seq[0+:SEQW]=8'd40; disp_ckpt[0+:CBITS]=2'd0;
      disp_pdst[0+:PBITS]=8'd72; #1; idxL=disp_lq_idx[0+:LQI]; @(posedge clk); idle;
      @(negedge clk); exe_ld_v=4'b0001; exe_ld_idx[0+:LQI]=idxL; exe_ld_addr[0+:AW]=64'hB0;
      exe_ld_nb[0+:4]=4'd4; @(posedge clk); idle;       // lq_rdy=1 -> selectable
      wbc0 = wb_count;
      rollback=1; rollback_seq=8'd39;                    // squash the load (40>39), hold it
      repeat (4) begin
         @(negedge clk); #1;
         if (ld_wb_v) begin $display("FAIL T5: squashed load asserted ld_wb_v"); errs=errs+1; end
         if (ld_done) begin $display("FAIL T5: squashed load asserted ld_done"); errs=errs+1; end
      end
      @(posedge clk); idle;
      repeat (3) @(posedge clk);
      if (wb_count != wbc0) begin $display("FAIL T5: squashed load still completed"); errs=errs+1; end

      if (errs==0) $display("lsu: ALL TESTS PASSED");
      else         $display("lsu: %0d FAILURES", errs);
      $finish;
   end
endmodule

`default_nettype wire
