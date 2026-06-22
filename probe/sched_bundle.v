`include "exec_pay.vh"
`default_nettype none

// The scheduler bundle: SHARDS scoreboard issue queues + the cross-shard
// broadcast net. Two broadcasts feed every shard's scoreboard:
//   clr  = each shard's freshly dispatched dest PR (built here from dispatch)
//   wake = each shard's executed-result PR (driven in from the execute bundle's
//          writeback -- the value is in the RF the next cycle, so a dependent
//          woken now issues next cycle and reads it: write-before-read)
// Fixed steering: dispatch slot i -> shard i -> issue slot i.
module sched_bundle
  #(parameter SHARDS = 4,
    parameter NPHYS  = 256,
    parameter PBITS  = 8,
    parameter IQD    = 8,
    parameter IQW    = 3,
    parameter SEQW   = 8,
    parameter LATW   = 2,
    parameter PAYW   = `PAYW)
   (input  wire                    clk,
    input  wire                    reset,
    // dispatch (from rename), slot i -> shard i
    input  wire [SHARDS-1:0]       disp_valid,
    input  wire [SHARDS*SEQW-1:0]  disp_seq,
    input  wire [SHARDS*PBITS-1:0] disp_pdst,
    input  wire [SHARDS-1:0]       disp_pdst_v,
    input  wire [SHARDS*PBITS-1:0] disp_ps1,
    input  wire [SHARDS-1:0]       disp_need1,
    input  wire [SHARDS*PBITS-1:0] disp_ps2,
    input  wire [SHARDS-1:0]       disp_need2,
    input  wire [SHARDS*LATW-1:0]  disp_lat,
    input  wire [SHARDS*PAYW-1:0]  disp_pay,
    output wire [SHARDS-1:0]       disp_ready,
    // wake from execute writeback
    input  wire [SHARDS-1:0]       wake_valid,
    input  wire [SHARDS*PBITS-1:0] wake_pr,
    // branch misprediction squash (broadcast to all shards)
    input  wire                    squash,
    input  wire [SEQW-1:0]         squash_seq,
    // issue (to execute), slot i <- shard i
    output wire [SHARDS-1:0]       iss_valid,
    output wire [SHARDS*PBITS-1:0] iss_pdst,
    output wire [SHARDS-1:0]       iss_pdst_v,
    output wire [SHARDS*PBITS-1:0] iss_ps1,
    output wire [SHARDS*PBITS-1:0] iss_ps2,
    output wire [SHARDS*SEQW-1:0]  iss_seq,
    output wire [SHARDS*LATW-1:0]  iss_lat,
    output wire [SHARDS*PAYW-1:0]  iss_pay);

   // clr broadcast: a dispatched dest clears its scoreboard bit (new value pending)
   wire [SHARDS-1:0]       clr_valid;
   wire [SHARDS*PBITS-1:0] clr_pr;
   genvar i;
   generate for (i = 0; i < SHARDS; i = i + 1) begin : clrgen
      assign clr_valid[i]            = disp_valid[i] & disp_ready[i] & disp_pdst_v[i];
      assign clr_pr[i*PBITS +: PBITS] = disp_pdst[i*PBITS +: PBITS];
   end endgenerate

   generate for (i = 0; i < SHARDS; i = i + 1) begin : lane
      sched_shard #(.SHARDS(SHARDS), .SH(i), .NPHYS(NPHYS), .PBITS(PBITS),
                    .IQD(IQD), .IQW(IQW), .SEQW(SEQW), .LATW(LATW), .PAYW(PAYW)) sh
        (.clk(clk), .reset(reset),
         .disp_valid(disp_valid[i]), .disp_seq(disp_seq[i*SEQW +: SEQW]),
         .disp_pdst(disp_pdst[i*PBITS +: PBITS]), .disp_pdst_v(disp_pdst_v[i]),
         .disp_ps1(disp_ps1[i*PBITS +: PBITS]), .disp_need1(disp_need1[i]),
         .disp_ps2(disp_ps2[i*PBITS +: PBITS]), .disp_need2(disp_need2[i]),
         .disp_lat(disp_lat[i*LATW +: LATW]), .disp_pay(disp_pay[i*PAYW +: PAYW]),
         .disp_ready(disp_ready[i]),
         .clr_valid(clr_valid), .clr_pr(clr_pr),
         .wake_valid(wake_valid), .wake_pr(wake_pr),
         .squash(squash), .squash_seq(squash_seq),
         .iss_valid(iss_valid[i]), .iss_seq(iss_seq[i*SEQW +: SEQW]),
         .iss_pdst(iss_pdst[i*PBITS +: PBITS]), .iss_pdst_v(iss_pdst_v[i]),
         .iss_ps1(iss_ps1[i*PBITS +: PBITS]), .iss_ps2(iss_ps2[i*PBITS +: PBITS]),
         .iss_lat(iss_lat[i*LATW +: LATW]), .iss_pay(iss_pay[i*PAYW +: PAYW]));
   end endgenerate
endmodule

`default_nettype wire
