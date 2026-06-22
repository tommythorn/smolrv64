`include "exec_pay.vh"
`default_nettype none

// One shard of the sharded scheduler: a non-speculative scoreboard issue queue.
// Eager allocation (the map already holds a commit-lifetime physical register),
// so the scheduler keys purely off physical-register readiness -- no slot naming,
// no PR<->slot translation. This is NOT a matrix and NOT a per-entry wakeup CAM:
// readiness is a shared ready[] bit-array indexed by each entry's stored source
// PRs (a wide mux per source, linear in NPHYS and IQ depth, not quadratic). The
// structure has the same external contract as a fancier scheduler, so it can be
// swapped later without disturbing rename/dispatch or execute.
//
// Per cycle this shard dispatches <=1 renamed instruction into its IQ and issues
// <=1 ready instruction (oldest-first by program-order seq). Cross-shard comms
// are registered next-cycle scoreboard updates carried on SHARDS-wide broadcast
// buses (self included; the bundle loops this shard's own dispatch/issue back in):
//   clr_*  = every shard's freshly dispatched dest PR  -> clear ready (new value pending)
//   wake_* = every shard's issued dest PR (+latency)   -> set ready when the value lands
//
// v1 wakeup is 1-cycle (ALU): an issued producer's dest becomes ready at the next
// edge, so a dependent issues the cycle after its producer (value bypassed). Fixed
// multi-cycle latency (loads = 3) needs a small per-source delay line on the wake
// path; that goes in with the LSU. disp_lat/iss_lat are already carried so adding
// it does not change this interface. WB-slot reservation likewise deferred (no
// execute/WB yet) -- it only matters once latencies mix.
module sched_shard
  #(parameter SHARDS = 4,
    parameter SH     = 0,
    parameter NPHYS  = 256,
    parameter PBITS  = 8,
    parameter IQD    = 8,        // issue-queue depth
    parameter IQW    = 3,        // clog2(IQD)
    parameter SEQW   = 8,
    parameter LATW   = 2,        // latency field width (carried, v1 unused)
    parameter CBITS  = 2,        // checkpoint id width (carried to issue for commit_ctl)
    parameter MIDXW  = 3,        // LSU slot index (sb/lq), allocated at dispatch
    parameter PAYW   = `PAYW)    // opaque execute payload (ctl+imm+pc+branch), see exec_pay.vh
   (input  wire                    clk,
    input  wire                    reset,
    // dispatch: this shard's renamed instruction
    input  wire                    disp_valid,
    input  wire [SEQW-1:0]         disp_seq,
    input  wire [PBITS-1:0]        disp_pdst,
    input  wire                    disp_pdst_v,   // writes a register (allocates dest)
    input  wire [PBITS-1:0]        disp_ps1,
    input  wire                    disp_need1,    // src1 is a real reg to wait on
    input  wire [PBITS-1:0]        disp_ps2,
    input  wire                    disp_need2,
    input  wire [LATW-1:0]         disp_lat,
    input  wire [CBITS-1:0]        disp_ckpt,     // checkpoint this instr belongs to
    input  wire [MIDXW-1:0]        disp_mem_idx,  // LSU sb/lq slot (mem ops)
    input  wire [PAYW-1:0]         disp_pay,      // opaque, stored and emitted at issue
    output wire                    disp_ready,    // IQ has room (backpressure to rename)
    // cross-shard scoreboard broadcasts (self included)
    input  wire [SHARDS-1:0]       clr_valid,
    input  wire [SHARDS*PBITS-1:0] clr_pr,
    input  wire [SHARDS-1:0]       wake_valid,
    input  wire [SHARDS*PBITS-1:0] wake_pr,
    // branch misprediction squash: drop entries younger than the branch
    input  wire                    squash,
    input  wire [SEQW-1:0]         squash_seq,
    // this shard's issue this cycle (bundle feeds it back as wake_*[SH])
    output wire                    iss_valid,
    output wire [SEQW-1:0]         iss_seq,
    output wire [PBITS-1:0]        iss_pdst,
    output wire                    iss_pdst_v,
    output wire [PBITS-1:0]        iss_ps1,
    output wire [PBITS-1:0]        iss_ps2,
    output wire [LATW-1:0]         iss_lat,
    output wire [CBITS-1:0]        iss_ckpt,
    output wire [MIDXW-1:0]        iss_mem_idx,
    output wire [PAYW-1:0]         iss_pay);

   // ---------------------------------------------------------------- state
   reg              ready [0:NPHYS-1];           // scoreboard: phys reg has its value
   reg              iqv   [0:IQD-1];
   reg [SEQW-1:0]   iqseq [0:IQD-1];
   reg [PBITS-1:0]  iqpd  [0:IQD-1];
   reg              iqpdv [0:IQD-1];
   reg [PBITS-1:0]  iqs1  [0:IQD-1];
   reg              iqn1  [0:IQD-1];
   reg [PBITS-1:0]  iqs2  [0:IQD-1];
   reg              iqn2  [0:IQD-1];
   reg [LATW-1:0]   iqlat [0:IQD-1];
   reg [CBITS-1:0]  iqck  [0:IQD-1];
   reg [MIDXW-1:0]  iqmi  [0:IQD-1];
   reg [PAYW-1:0]   iqpay [0:IQD-1];

   integer i;
   initial begin
      for (i = 0; i < NPHYS; i = i + 1) ready[i] = 1'b1;   // arch values present
      for (i = 0; i < IQD;   i = i + 1) iqv[i]   = 1'b0;
   end

   // ------------------------------------------------ eligibility + oldest select
   // A source is satisfied if it is not a real dependency or its PR is ready.
   reg [IQD-1:0]  elig;
   reg            found;
   reg [IQW-1:0]  sel;
   reg [SEQW-1:0] best;
   integer e;
   always @* begin
      found = 1'b0; sel = {IQW{1'b0}}; best = {SEQW{1'b0}};
      for (e = 0; e < IQD; e = e + 1) begin
         elig[e] = iqv[e]
                 && (!iqn1[e] || ready[iqs1[e]])
                 && (!iqn2[e] || ready[iqs2[e]]);
         // oldest-first; wrap-safe program-order compare ("older" = signed diff < 0,
         // valid while the in-flight window stays < 2^(SEQW-1))
         if (elig[e] && (!found || $signed(iqseq[e] - best) < 0)) begin
            found = 1'b1; sel = e[IQW-1:0]; best = iqseq[e];
         end
      end
   end

   // free IQ slot for dispatch (lowest invalid entry)
   reg          have_free;
   reg [IQW-1:0] freeslot;
   always @* begin
      have_free = 1'b0; freeslot = {IQW{1'b0}};
      for (e = IQD-1; e >= 0; e = e - 1)
         if (!iqv[e]) begin have_free = 1'b1; freeslot = e[IQW-1:0]; end
   end

   assign disp_ready = have_free;
   assign iss_valid  = found;
   assign iss_seq    = iqseq[sel];
   assign iss_pdst   = iqpd [sel];
   assign iss_pdst_v = iqpdv[sel];
   assign iss_ps1    = iqs1 [sel];
   assign iss_ps2    = iqs2 [sel];
   assign iss_lat    = iqlat[sel];
   assign iss_ckpt   = iqck [sel];
   assign iss_mem_idx= iqmi [sel];
   assign iss_pay    = iqpay[sel];

   // ------------------------------------------------ unpack broadcast buses
   reg [PBITS-1:0] wkpr [0:SHARDS-1];
   reg [PBITS-1:0] clpr [0:SHARDS-1];
   integer s;
   always @* for (s = 0; s < SHARDS; s = s + 1) begin
      wkpr[s] = wake_pr[s*PBITS +: PBITS];
      clpr[s] = clr_pr [s*PBITS +: PBITS];
   end

   // ------------------------------------------------------------- sequential
   integer k;
   always @(posedge clk) begin
      if (reset) begin
         for (k = 0; k < NPHYS; k = k + 1) ready[k] <= 1'b1;
         for (k = 0; k < IQD;   k = k + 1) iqv[k]   <= 1'b0;
      end else begin
         // scoreboard: set woken producers (v1: 1-cycle), then clear freshly
         // dispatched dests -- clear after set so a same-cycle clash leaves the
         // newly allocated dest not-ready (its value is still pending).
         for (s = 0; s < SHARDS; s = s + 1) if (wake_valid[s]) ready[wkpr[s]] <= 1'b1;
         for (s = 0; s < SHARDS; s = s + 1) if (clr_valid[s])  ready[clpr[s]] <= 1'b0;

         // issue: free the selected entry
         if (found) iqv[sel] <= 1'b0;

         // branch squash: invalidate entries younger (in program order) than the
         // mispredicting branch. (seqno is program order; rolled back on redirect.)
         if (squash)
            for (k = 0; k < IQD; k = k + 1)
               if (iqv[k] && ($signed(iqseq[k] - squash_seq) > 0)) iqv[k] <= 1'b0;  // younger (wrap-safe)

         // dispatch: insert into a free slot (issue's freed slot is not reused
         // this cycle -- have_free only counts currently-invalid entries)
         if (disp_valid && have_free) begin
            iqv  [freeslot] <= 1'b1;
            iqseq[freeslot] <= disp_seq;
            iqpd [freeslot] <= disp_pdst;
            iqpdv[freeslot] <= disp_pdst_v;
            iqs1 [freeslot] <= disp_ps1;
            iqn1 [freeslot] <= disp_need1;
            iqs2 [freeslot] <= disp_ps2;
            iqn2 [freeslot] <= disp_need2;
            iqlat[freeslot] <= disp_lat;
            iqck [freeslot] <= disp_ckpt;
            iqmi [freeslot] <= disp_mem_idx;
            iqpay[freeslot] <= disp_pay;
         end
      end
   end
endmodule

`default_nettype wire
