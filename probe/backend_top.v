`include "exec_pay.vh"
`default_nettype none

// Full sharded-OoO core (frontend + backend), ALU subset:
//   PC -> fetch/align -> decode -> [reg] -> rename -> dispatch
//      -> scheduler (scoreboard issue queues) -> execute (RF + ALU) -> writeback
//   writeback -> scheduler wake (+ every RF copy)   [write-before-read forwarding]
//
// Commit/CPR not wired yet (fr_*/chk_* tied off); no renamer back-pressure yet
// (a bounded program won't fill the IQ or drain the 256-entry freelist). LSU,
// branch redirect, and the CSR/M/F units are future work. Writeback is exposed
// for observation/checking.
module backend_top
  #(parameter IW    = 4,
    parameter HW    = 8,
    parameter PCW   = 64,
    parameter SEQW  = 8,
    parameter ABITS = 6,
    parameter PBITS = 8,
    parameter LATW  = 2,
    parameter [PCW-1:0] RESET_PC = 0)
   (input  wire                    clk,
    input  wire                    reset,
    input  wire                    redirect,
    input  wire [PCW-1:0]          redirect_pc,
    input  wire [SEQW-1:0]         redirect_seq,
    output wire [PCW-1:0]          imem_addr,
    input  wire [HW*16-1:0]        imem_data,
    input  wire [$clog2(HW+2)-1:0] imem_avail,
    // observation: per-shard writeback
    output wire [IW-1:0]           wb_valid,
    output wire [IW*PBITS-1:0]     wb_pr,
    output wire [IW*64-1:0]        wb_val);

   // ---- frontend: fetch -> decode -> rename ----
   wire [IW-1:0]      r_valid, r_rd_v, r_need1, r_need2, fe_stall;
   wire [IW*SEQW-1:0] r_seq;
   wire [IW*ABITS-1:0] r_rd;
   wire [IW*PBITS-1:0] ps1, ps2, pdst;
   wire [IW*`PAYW-1:0] r_pay;

   frontend #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW), .ABITS(ABITS),
              .PBITS(PBITS), .RESET_PC(RESET_PC)) fe
     (.clk(clk), .reset(reset), .redirect(redirect), .redirect_pc(redirect_pc),
      .redirect_seq(redirect_seq), .imem_addr(imem_addr), .imem_data(imem_data),
      .imem_avail(imem_avail),
      .fr_phys({IW*PBITS{1'b0}}), .fr_valid({IW{1'b0}}),
      .chk_create(1'b0), .chk_create_idx(2'b0), .chk_restore(1'b0), .chk_restore_idx(2'b0),
      .r_valid(r_valid), .r_seq(r_seq), .r_rd(r_rd), .r_rd_v(r_rd_v),
      .ps1(ps1), .ps2(ps2), .pdst(pdst),
      .r_need1(r_need1), .r_need2(r_need2), .r_pay(r_pay), .stall(fe_stall));

   // ---- scheduler bundle ----
   wire [IW-1:0]       iss_valid, iss_pdst_v, disp_ready;
   wire [IW*PBITS-1:0] iss_pdst, iss_ps1, iss_ps2;
   wire [IW*SEQW-1:0]  iss_seq;
   wire [IW*LATW-1:0]  iss_lat;
   wire [IW*`PAYW-1:0] iss_pay;
   wire [IW-1:0]       wkv;          // writeback = wake source
   wire [IW*PBITS-1:0] wkp;

   // every ALU op is fixed latency 1 in this subset
   wire [IW*LATW-1:0] disp_lat = {IW{ {{(LATW-1){1'b0}}, 1'b1} }};

   sched_bundle #(.SHARDS(IW), .PBITS(PBITS), .SEQW(SEQW), .LATW(LATW)) sb
     (.clk(clk), .reset(reset),
      .disp_valid(r_valid), .disp_seq(r_seq), .disp_pdst(pdst), .disp_pdst_v(r_rd_v),
      .disp_ps1(ps1), .disp_need1(r_need1), .disp_ps2(ps2), .disp_need2(r_need2),
      .disp_lat(disp_lat), .disp_pay(r_pay), .disp_ready(disp_ready),
      .wake_valid(wkv), .wake_pr(wkp),
      .iss_valid(iss_valid), .iss_pdst(iss_pdst), .iss_pdst_v(iss_pdst_v),
      .iss_ps1(iss_ps1), .iss_ps2(iss_ps2), .iss_seq(iss_seq),
      .iss_lat(iss_lat), .iss_pay(iss_pay));

   // ---- execute bundle (RF + ALU + wb broadcast) ----
   exec_bundle #(.SHARDS(IW), .PBITS(PBITS)) eb
     (.clk(clk),
      .iss_valid(iss_valid), .iss_pdst(iss_pdst), .iss_pdst_v(iss_pdst_v),
      .iss_ps1(iss_ps1), .iss_ps2(iss_ps2), .iss_pay(iss_pay),
      .wb_valid(wkv), .wb_pr(wkp), .wb_val(wb_val),
      .agu_addr(), .cmp_eq(), .cmp_lt(), .cmp_ltu());

   assign wb_valid = wkv;
   assign wb_pr    = wkp;
endmodule

`default_nettype wire
