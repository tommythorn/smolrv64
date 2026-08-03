`timescale 1ns/1ps
`default_nettype none

// Deterministic reproduction of the store->load stale-data bug caught in the Ubuntu cosim
// at 6.58B insn (an aligned 8B load returned stale memory instead of a prior committed
// same-address store's value). The LSU has NO memory-order replay and TRUSTS the memory to
// return current data at mem_rvalid ("no stale-data hazard"). This TB constructs the race:
//
//   older store (seq1) @X, filled + committed but held un-drained; younger load (seq2) @X
//   enters MERGE, pulses mem_ren (its read CAPTURES mem[X]); during the read latency we let
//   the store DRAIN (writes X, leaves the SB); at rvalid the load byte-merges.
//
// Memory model is latent (LAT cycles) and selectable:
//   +stale=0  -> returns CURRENT mem[X] at rvalid (a correct D$)  -> load must get the store's D
//   +stale=1  -> returns the value CAPTURED at mem_ren (pre-drain) -> models a D$ that lets a
//                read race a drain-write; if the load then returns that -> the bug, and it is
//                on the MEMORY side (the LSU dropped the SB forward trusting memory was fresh).
// Sweeps the drain offset within the read window via +draindelay.
//   PASS = load == store data D ; FAIL = load == pre-store OLD (stale).
//
// Run (needs iverilog 4-state -- verilator's 2-state breaks the directly-driven dispatch):
//   iverilog -g2012 -o /tmp/tb_race.vvp lsu.v mmu.v tb_lsu_race.v
//   for s in 0 1; do vvp /tmp/tb_race.vvp +lat=4 +stale=$s +draindelay=1; done
// Post-fix (drain-vs-in-flight-load interlock) BOTH stale modes must print "LSU-RACE: PASS":
// stale=0 checks a correct D$; stale=1 checks that the interlock holds the SB forward even when
// the D$ would race the drain. Pre-fix, +stale=1 aborts with FAIL-STALE (nonzero exit).

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
   wire              mem_ren;
   wire              mem_wen;
   wire [AW-1:0]     mem_waddr;
   wire [63:0]       mem_wdata;
   wire [7:0]        mem_wmask;
   reg               mem_wready;
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

   // ---- latent, optionally-stale memory ----
   reg [7:0]  mem [0:255];
   integer    LAT; reg STALE, DRAINDELAY_dummy; integer DRAINDELAY;
   reg [63:0] rd_cap; integer rd_cnt; reg rd_busy;
   reg        mem_rvalid; reg [63:0] mem_rdata;
   wire [7:0] ra = mem_raddr[7:0];
   wire [63:0] mem_cur = {mem[ra+7],mem[ra+6],mem[ra+5],mem[ra+4],
                          mem[ra+3],mem[ra+2],mem[ra+1],mem[ra+0]};
   integer mb;
   always @(posedge clk) begin
      if (reset) begin rd_busy<=1'b0; mem_rvalid<=1'b0; end
      else begin
         mem_rvalid <= 1'b0;
         if (mem_ren) begin              // fresh read -> capture NOW (pre any later drain)
            rd_cap <= mem_cur; rd_cnt <= LAT; rd_busy <= 1'b1;
            if (LAT==0) begin mem_rvalid<=1'b1; mem_rdata <= mem_cur; rd_busy<=1'b0; end
         end else if (rd_busy) begin
            if (rd_cnt <= 1) begin rd_busy<=1'b0; mem_rvalid<=1'b1;
               mem_rdata <= STALE ? rd_cap : mem_cur;   // stale: value at request; correct: value now
            end else rd_cnt <= rd_cnt - 1;
         end
      end
      if (mem_wen && mem_wready)          // drain write (gated by mem_wready)
         for (mb=0; mb<8; mb=mb+1)
            if (mem_wmask[mb]) mem[mem_waddr[7:0]+mb] <= mem_wdata[mb*8 +: 8];
   end

   lsu #(.IW(IW), .SBITS(SBITS), .PBITS(PBITS), .SEQW(SEQW), .CBITS(CBITS), .AW(AW),
         .SBDEPTH(SBDEPTH), .SBI(SBI), .LQDEPTH(LQDEPTH), .LQI(LQI), .DEV_TOP(64'd0)) dut
     (.clk(clk), .reset(reset),
      .disp_fire(disp_fire), .disp_is_load(disp_is_load), .disp_is_store(disp_is_store),
      .disp_seq(disp_seq), .disp_ckpt(disp_ckpt), .disp_pdst(disp_pdst),
      .disp_sb_idx(disp_sb_idx), .disp_lq_idx(disp_lq_idx), .sb_full(sb_full), .lq_full(lq_full),
      .exe_st_v(exe_st_v), .exe_st_idx(exe_st_idx), .exe_st_addr(exe_st_addr),
      .exe_st_data(exe_st_data), .exe_st_nb(exe_st_nb),
      .exe_st_cbo({IW{1'b0}}), .exe_st_cbo_zero({IW{1'b0}}), .exe_st_cbo_keep({IW{1'b0}}),
      .exe_ld_v(exe_ld_v), .exe_ld_idx(exe_ld_idx), .exe_ld_addr(exe_ld_addr),
      .exe_ld_nb(exe_ld_nb), .exe_ld_sgn(exe_ld_sgn), .exe_ld_fp(exe_ld_fp),
      .amo_v(1'b0), .amo_func(5'd0), .amo_addr({AW{1'b0}}), .amo_data(64'd0),
      .amo_sz(2'd0), .amo_pdst({PBITS{1'b0}}), .amo_owner({SBITS{1'b0}}), .amo_ckpt({CBITS{1'b0}}),
      .xl_satp(64'd0), .xl_priv(2'd0), .xl_sum(1'b0), .xl_mxr(1'b0), .xl_flush(1'b0),
      .ldp_addr(), .ldp_read(), .ldp_rdata(64'd0), .ldp_rvalid(1'b0),
      .stp_addr(), .stp_read(), .stp_rdata(64'd0), .stp_rvalid(1'b0),
      .dfault_v(), .dfault_seq(), .dfault_ckpt(), .dfault_cause(),
      .mem_raddr(mem_raddr), .mem_rdata(mem_rdata), .mem_rvalid(mem_rvalid), .mem_ren(mem_ren),
      .mem_wready(mem_wready),
      .mem_wen(mem_wen), .mem_waddr(mem_waddr), .mem_wdata(mem_wdata), .mem_wmask(mem_wmask),
      .wb_busy({IW{1'b0}}),
      .ld_wb_v(ld_wb_v), .ld_wb_pdst(ld_wb_pdst), .ld_wb_owner(ld_wb_owner),
      .ld_wb_val(ld_wb_val), .ld_done(ld_done), .ld_done_ckpt(ld_done_ckpt),
      .commit(commit), .commit_idx(commit_idx), .committed(2'd0),
      .rollback(rollback), .rollback_seq(rollback_seq));

   reg [63:0] last_wb_val; integer wb_count;
   initial wb_count = 0;
   always @(posedge clk) if (ld_wb_v) begin last_wb_val <= ld_wb_val; wb_count <= wb_count+1; end

   task idle; begin
      disp_fire=0; disp_is_load=0; disp_is_store=0; disp_seq=0; disp_ckpt=0; disp_pdst=0;
      exe_st_v=0; exe_st_idx=0; exe_st_addr=0; exe_st_data=0; exe_st_nb=0;
      exe_ld_v=0; exe_ld_idx=0; exe_ld_addr=0; exe_ld_nb=0; exe_ld_sgn=0; exe_ld_fp=0;
      commit=0; commit_idx=0; rollback=0; rollback_seq=0;
   end endtask

   localparam [63:0] OLD = 64'hAAAAAAAAAAAAAAAA;
   localparam [63:0] DVAL= 64'hD1D2D3D4D5D6D7D8;
   reg [SBI-1:0] sidx; reg [LQI-1:0] lidx; integer k; integer seen_ren; integer wbc0;

   initial begin
      LAT=4; STALE=0; DRAINDELAY=1;
      void'($value$plusargs("lat=%d", LAT));
      void'($value$plusargs("stale=%d", STALE));
      void'($value$plusargs("draindelay=%d", DRAINDELAY));
      for (k=0;k<256;k=k+1) mem[k]=8'h00;
      {mem[8'h47],mem[8'h46],mem[8'h45],mem[8'h44],mem[8'h43],mem[8'h42],mem[8'h41],mem[8'h40]} = OLD;

      idle; mem_wready=0; reset=1; @(negedge clk); @(negedge clk); reset=0; @(negedge clk);

      // older store seq=1 @0x40 = DVAL (8B); fill; commit -> drainable but mem_wready=0 holds it
      disp_fire=1; disp_is_store=4'b0001; disp_seq[0+:SEQW]=8'd1; disp_ckpt[0+:CBITS]=2'd0;
      #1; sidx=disp_sb_idx[0+:SBI]; @(posedge clk); idle;
      @(negedge clk); exe_st_v=4'b0001; exe_st_idx[0+:SBI]=sidx; exe_st_addr[0+:AW]=64'h40;
      exe_st_data[0+:64]=DVAL; exe_st_nb[0+:4]=4'd8; @(posedge clk); idle;
      @(negedge clk); commit=1; commit_idx=2'd0; @(posedge clk); idle;   // commit -> drainable (drain held by mem_wready=0)
      // younger load seq=2 @0x40; fill -> baseline: forward the in-SB (uncommitted) store
      @(negedge clk); disp_fire=1; disp_is_load=4'b0001; disp_seq[0+:SEQW]=8'd2; disp_ckpt[0+:CBITS]=2'd0;
      disp_pdst[0+:PBITS]=8'd70; #1; lidx=disp_lq_idx[0+:LQI]; @(posedge clk); idle;
      @(negedge clk); exe_ld_v=4'b0001; exe_ld_idx[0+:LQI]=lidx; exe_ld_addr[0+:AW]=64'h40;
      exe_ld_nb[0+:4]=4'd8; @(posedge clk); idle;

      // wait for the load's read request (mem_ren), then let the store drain DRAINDELAY cycles later
      seen_ren=0; wbc0=wb_count;
      for (k=0;k<40 && wb_count==wbc0;k=k+1) begin
         @(negedge clk);
         if (mem_ren) seen_ren=1;
         if (seen_ren==1) begin
            if (DRAINDELAY<=0) begin mem_wready=1; end
            else DRAINDELAY = DRAINDELAY - 1;
            if (mem_wready) seen_ren=2;   // one drain-enable window
         end
      end
      @(posedge clk); mem_wready=1;   // ensure it drained
      repeat (6) @(posedge clk);

      $display("RACE lat=%0d stale=%0d : load WB = %h  (D=%h OLD=%h)  mem[40..47]=%h",
               LAT, STALE, last_wb_val, DVAL, OLD, mem_cur);
      if (last_wb_val === DVAL)      begin $display("LSU-RACE: PASS (load saw the store's data)"); $finish; end
      else if (last_wb_val === OLD)  $fatal(1, "LSU-RACE: FAIL-STALE (load returned pre-store memory)");
      else                           $fatal(1, "LSU-RACE: FAIL-OTHER wb=%h", last_wb_val);
   end
endmodule

`default_nettype wire
