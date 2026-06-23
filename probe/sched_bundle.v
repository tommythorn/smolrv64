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
    parameter N      = 2,        // CAM reservation-station entries per shard (sweep for timing)
    parameter WAKEN  = 2*SHARDS, // wake ports: select-time (SHARDS) + completion-time (SHARDS)
    parameter SEQW   = 8,
    parameter CBITS  = 2,
    parameter MIDXW  = 3,
    parameter PAYW   = `PAYW)
   (input  wire                    clk,
    input  wire                    reset,
    // dispatch (from rename), slot i -> shard i
    input  wire [SHARDS-1:0]       disp_valid,
    input  wire [SHARDS*SEQW-1:0]  disp_seq,
    input  wire [SHARDS*PBITS-1:0] disp_pdst,
    input  wire [SHARDS-1:0]       disp_pdst_v,
    input  wire [SHARDS*PBITS-1:0] disp_ps1,
    input  wire [SHARDS*PBITS-1:0] disp_ps2,
    input  wire [SHARDS*PBITS-1:0] disp_ps3,      // FMA 3rd operand (tied to p0 until FP)
    input  wire [SHARDS*CBITS-1:0] disp_ckpt,
    input  wire [SHARDS*MIDXW-1:0] disp_mem_idx,
    input  wire [SHARDS*PAYW-1:0]  disp_pay,
    output wire [SHARDS-1:0]       disp_ready,
    // wake: select-time (latency-1) + completion-time (load/divide), WAKEN ports
    input  wire [WAKEN-1:0]        wake_valid,
    input  wire [WAKEN*PBITS-1:0]  wake_pr,
    // branch misprediction squash (broadcast to all shards)
    input  wire                    squash,
    input  wire [SEQW-1:0]         squash_seq,
    // per-shard execute stall (iterative divider busy)
    input  wire [SHARDS-1:0]       exec_busy,
    // oldest live checkpoint (serializing ops issue only when oldest)
    input  wire [CBITS-1:0]        committed,
    // issue (to execute), slot i <- shard i
    output wire [SHARDS-1:0]       iss_valid,
    output wire [SHARDS*PBITS-1:0] iss_pdst,
    output wire [SHARDS-1:0]       iss_pdst_v,
    output wire [SHARDS*PBITS-1:0] iss_ps1,
    output wire [SHARDS*PBITS-1:0] iss_ps2,
    output wire [SHARDS*PBITS-1:0] iss_ps3,
    output wire [SHARDS*SEQW-1:0]  iss_seq,
    output wire [SHARDS*CBITS-1:0] iss_ckpt,
    output wire [SHARDS*MIDXW-1:0] iss_mem_idx,
    output wire [SHARDS*PAYW-1:0]  iss_pay);

   // clr broadcast: a dispatched dest clears its scoreboard bit (new value pending)
   wire [SHARDS-1:0]       clr_valid;
   wire [SHARDS*PBITS-1:0] clr_pr;
   genvar i;
   generate for (i = 0; i < SHARDS; i = i + 1) begin : clrgen
      assign clr_valid[i]            = disp_valid[i] & disp_ready[i] & disp_pdst_v[i];
      assign clr_pr[i*PBITS +: PBITS] = disp_pdst[i*PBITS +: PBITS];
   end endgenerate

   // --------------------------------------------- shared per-phys "value-present" scoreboard
   // One table for the whole bundle: it is bit-identical in every shard (all see the same
   // wake/clr broadcasts, no per-shard write), so it lived 4x replicated in the shards. Here
   // it is single: wake sets, a freshly dispatched dest clears, p0 is never cleared. Each
   // shard's three sources are pre-read combinationally and handed down (disp_rdy{1,2,3}).
   reg              ready [0:NPHYS-1];
   integer ix, ws, cs;
   initial for (ix = 0; ix < NPHYS; ix = ix + 1) ready[ix] = 1'b1;   // arch values present
   always @(posedge clk) begin
      if (reset) for (ix = 0; ix < NPHYS; ix = ix + 1) ready[ix] <= 1'b1;
      else begin
         for (ws = 0; ws < WAKEN;  ws = ws + 1)
            if (wake_valid[ws]) ready[wake_pr[ws*PBITS +: PBITS]] <= 1'b1;
         // p0 is the constant-zero reg: always ready, never a real dest -> never cleared
         // (a non-writer's clr would target p0 only if pdst_v leaked; guard it regardless).
         for (cs = 0; cs < SHARDS; cs = cs + 1)
            if (clr_valid[cs] && (clr_pr[cs*PBITS +: PBITS] != {PBITS{1'b0}}))
               ready[clr_pr[cs*PBITS +: PBITS]] <= 1'b0;
      end
   end
   wire [SHARDS-1:0] disp_rdy1, disp_rdy2, disp_rdy3;
   generate for (i = 0; i < SHARDS; i = i + 1) begin : rdgen
      assign disp_rdy1[i] = ready[disp_ps1[i*PBITS +: PBITS]];
      assign disp_rdy2[i] = ready[disp_ps2[i*PBITS +: PBITS]];
      assign disp_rdy3[i] = ready[disp_ps3[i*PBITS +: PBITS]];
   end endgenerate

   generate for (i = 0; i < SHARDS; i = i + 1) begin : lane
      sched_shard #(.SHARDS(SHARDS), .SH(i), .NPHYS(NPHYS), .PBITS(PBITS),
                    .N(N), .WAKEN(WAKEN), .SEQW(SEQW), .CBITS(CBITS),
                    .MIDXW(MIDXW), .PAYW(PAYW)) sh
        (.clk(clk), .reset(reset),
         .disp_valid(disp_valid[i]), .disp_seq(disp_seq[i*SEQW +: SEQW]),
         .disp_pdst(disp_pdst[i*PBITS +: PBITS]), .disp_pdst_v(disp_pdst_v[i]),
         .disp_ps1(disp_ps1[i*PBITS +: PBITS]), .disp_rdy1(disp_rdy1[i]),
         .disp_ps2(disp_ps2[i*PBITS +: PBITS]), .disp_rdy2(disp_rdy2[i]),
         .disp_ps3(disp_ps3[i*PBITS +: PBITS]), .disp_rdy3(disp_rdy3[i]),
         .disp_ckpt(disp_ckpt[i*CBITS +: CBITS]),
         .disp_mem_idx(disp_mem_idx[i*MIDXW +: MIDXW]),
         .disp_pay(disp_pay[i*PAYW +: PAYW]),
         .disp_ready(disp_ready[i]),
         .clr_valid(clr_valid), .clr_pr(clr_pr),
         .wake_valid(wake_valid), .wake_pr(wake_pr),
         .squash(squash), .squash_seq(squash_seq), .exec_busy(exec_busy[i]),
         .committed(committed),
         .iss_valid(iss_valid[i]), .iss_seq(iss_seq[i*SEQW +: SEQW]),
         .iss_pdst(iss_pdst[i*PBITS +: PBITS]), .iss_pdst_v(iss_pdst_v[i]),
         .iss_ps1(iss_ps1[i*PBITS +: PBITS]), .iss_ps2(iss_ps2[i*PBITS +: PBITS]),
         .iss_ps3(iss_ps3[i*PBITS +: PBITS]),
         .iss_ckpt(iss_ckpt[i*CBITS +: CBITS]),
         .iss_mem_idx(iss_mem_idx[i*MIDXW +: MIDXW]),
         .iss_pay(iss_pay[i*PAYW +: PAYW]));
   end endgenerate
endmodule

`default_nettype wire
