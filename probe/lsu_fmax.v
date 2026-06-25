`default_nettype none

// Flop-wrapped LSU for the timing probe (Fmax via the `make probe` P&R flow).
//
//   lfsr -> din_q (reg) -> [ lsu (+ its 2 mmu walkers) ] -> dout_q (reg) -> probe_out
//
// Every LSU input is driven from a slice of a registered pseudo-random word and
// every LSU output is registered, so the paths that matter are clean reg-to-reg
// paths *through* the LSU (the SELECT scan, the MERGE byte-merge, the drain
// select, the AMO FSM, the translation walkers). This is the methodology behind
// the meaningful ~2.4 ns LSU figure -- bare `synth_design -top lsu` gives a
// flatten artifact instead.
//
// Params match backend_top's LSU instance (SBDEPTH/LQDEPTH = 4). `reset` is tied
// low (a timing probe runs the steady-state logic; FF init comes from the LSU's
// `initial` values).
//
// Usage:  make probe TOP=lsu_fmax PERIOD=2.5 REGION="CLOCKREGION_X0Y0:CLOCKREGION_X1Y2" \
//              SRCS="lsu_fmax.v lsu.v mmu.v"
//
// CAVEAT (measured 2026-06-24): unlike rf_alu (a clean datapath), the LSU is
// control/search-heavy, so fully-independent random stimulus DEFEATS the pruning
// it gets in-context (correlated inputs) and bloats it to ~14.6k LUTs (~3-4x its
// real backend footprint). The result is ROUTE-DOMINATED (~60-70% route on a big
// netlist) -> absolute Fmax (~139 MHz) is PESSIMISTIC, not the LSU's real ceiling.
// The meaningful signal is the LOGIC delay (~2.2 ns, matches the historical
// ~2.39 ns), and RELATIVE A/B deltas across LSU edits. The worst path is the
// data-fault-report cone (*_seq/sb_xck -> df_*_r/CE), not the load datapath. For a
// true LSU Fmax, measure in-context (backend / platform), not in isolation.
module lsu_fmax (input wire clk, output reg probe_out = 1'b0);

   localparam IW = 4, SBITS = 2, PBITS = 8, SEQW = 8, CBITS = 2, AW = 64;
   localparam SBDEPTH = 4, SBI = 2, LQDEPTH = 4, LQI = 2;

   // ---- total input bit-width (symbolic sum: single source of truth) ----
   localparam IN_W =
        1                       // disp_fire
      + IW                      // disp_is_load
      + IW                      // disp_is_store
      + IW*SEQW                 // disp_seq
      + IW*CBITS                // disp_ckpt
      + IW*PBITS                // disp_pdst
      + IW                      // exe_st_v
      + IW*SBI                  // exe_st_idx
      + IW*AW                   // exe_st_addr
      + IW*64                   // exe_st_data
      + IW*4                    // exe_st_nb
      + IW                      // exe_ld_v
      + IW*LQI                  // exe_ld_idx
      + IW*AW                   // exe_ld_addr
      + IW*4                    // exe_ld_nb
      + IW                      // exe_ld_sgn
      + 1 + 5 + AW + 64 + 2 + PBITS + SBITS + CBITS + SEQW   // amo_*
      + 64 + 2 + 1 + 1 + 1      // xl_satp/priv/sum/mxr/flush
      + 64 + 1                  // ldp_rdata/rvalid
      + 64 + 1                  // stp_rdata/rvalid
      + 64 + 1                  // mem_rdata/rvalid
      + IW                      // wb_busy
      + 1 + CBITS               // commit/commit_idx
      + 1 + SEQW                // rollback/rollback_seq
      + 1;                      // dfault_taken

   // ---- Fibonacci LFSR -> din_q (registered stimulus) ----
   reg  [IN_W-1:0] lfsr = {IN_W{1'b1}};
   wire fb = lfsr[IN_W-1] ^ lfsr[IN_W-2] ^ lfsr[IN_W-4] ^ lfsr[IN_W-5];
   always @(posedge clk) lfsr <= {lfsr[IN_W-2:0], fb};
   reg  [IN_W-1:0] din_q = {IN_W{1'b0}};
   always @(posedge clk) din_q <= lfsr;

   // ---- LSU input nets (driven by one ordered concat from din_q) ----
   wire                  disp_fire;
   wire [IW-1:0]         disp_is_load, disp_is_store;
   wire [IW*SEQW-1:0]    disp_seq;
   wire [IW*CBITS-1:0]   disp_ckpt;
   wire [IW*PBITS-1:0]   disp_pdst;
   wire [IW-1:0]         exe_st_v;
   wire [IW*SBI-1:0]     exe_st_idx;
   wire [IW*AW-1:0]      exe_st_addr;
   wire [IW*64-1:0]      exe_st_data;
   wire [IW*4-1:0]       exe_st_nb;
   wire [IW-1:0]         exe_ld_v;
   wire [IW*LQI-1:0]     exe_ld_idx;
   wire [IW*AW-1:0]      exe_ld_addr;
   wire [IW*4-1:0]       exe_ld_nb;
   wire [IW-1:0]         exe_ld_sgn;
   wire                  amo_v;
   wire [4:0]            amo_func;
   wire [AW-1:0]         amo_addr;
   wire [63:0]           amo_data;
   wire [1:0]            amo_sz;
   wire [PBITS-1:0]      amo_pdst;
   wire [SBITS-1:0]      amo_owner;
   wire [CBITS-1:0]      amo_ckpt;
   wire [SEQW-1:0]       amo_seq;
   wire [63:0]           xl_satp;
   wire [1:0]            xl_priv;
   wire                  xl_sum, xl_mxr, xl_flush;
   wire [63:0]           ldp_rdata;
   wire                  ldp_rvalid;
   wire [63:0]           stp_rdata;
   wire                  stp_rvalid;
   wire [63:0]           mem_rdata;
   wire                  mem_rvalid;
   wire [IW-1:0]         wb_busy;
   wire                  commit;
   wire [CBITS-1:0]      commit_idx;
   wire                  rollback;
   wire [SEQW-1:0]       rollback_seq;
   wire                  dfault_taken;

   assign {disp_fire, disp_is_load, disp_is_store, disp_seq, disp_ckpt, disp_pdst,
           exe_st_v, exe_st_idx, exe_st_addr, exe_st_data, exe_st_nb,
           exe_ld_v, exe_ld_idx, exe_ld_addr, exe_ld_nb, exe_ld_sgn,
           amo_v, amo_func, amo_addr, amo_data, amo_sz, amo_pdst, amo_owner, amo_ckpt, amo_seq,
           xl_satp, xl_priv, xl_sum, xl_mxr, xl_flush,
           ldp_rdata, ldp_rvalid, stp_rdata, stp_rvalid, mem_rdata, mem_rvalid,
           wb_busy, commit, commit_idx, rollback, rollback_seq, dfault_taken} = din_q;

   // ---- LSU outputs ----
   wire [IW*SBI-1:0]     disp_sb_idx;
   wire [IW*LQI-1:0]     disp_lq_idx;
   wire                  sb_full, lq_full;
   wire                  dfault_v;
   wire [SEQW-1:0]       dfault_seq;
   wire [CBITS-1:0]      dfault_ckpt;
   wire [3:0]            dfault_cause;
   wire [AW-1:0]         dfault_tval;
   wire                  st_done;
   wire [CBITS-1:0]      st_done_ckpt;
   wire [AW-1:0]         mem_raddr;
   wire                  mem_ren, mem_wen;
   wire [AW-1:0]         mem_waddr;
   wire [63:0]           mem_wdata;
   wire [7:0]            mem_wmask;
   wire                  ld_wb_v;
   wire [PBITS-1:0]      ld_wb_pdst;
   wire [SBITS-1:0]      ld_wb_owner;
   wire [63:0]           ld_wb_val;
   wire                  ld_done;
   wire [CBITS-1:0]      ld_done_ckpt;
   wire [55:0]           ldp_addr, stp_addr;
   wire                  ldp_read, stp_read;

   lsu #(.IW(IW), .SBITS(SBITS), .PBITS(PBITS), .SEQW(SEQW), .CBITS(CBITS), .AW(AW),
         .SBDEPTH(SBDEPTH), .SBI(SBI), .LQDEPTH(LQDEPTH), .LQI(LQI)) dut
     (.clk(clk), .reset(1'b0),
      .disp_fire(disp_fire), .disp_is_load(disp_is_load), .disp_is_store(disp_is_store),
      .disp_seq(disp_seq), .disp_ckpt(disp_ckpt), .disp_pdst(disp_pdst),
      .disp_sb_idx(disp_sb_idx), .disp_lq_idx(disp_lq_idx), .sb_full(sb_full), .lq_full(lq_full),
      .exe_st_v(exe_st_v), .exe_st_idx(exe_st_idx), .exe_st_addr(exe_st_addr),
      .exe_st_data(exe_st_data), .exe_st_nb(exe_st_nb),
      .exe_ld_v(exe_ld_v), .exe_ld_idx(exe_ld_idx), .exe_ld_addr(exe_ld_addr),
      .exe_ld_nb(exe_ld_nb), .exe_ld_sgn(exe_ld_sgn),
      .amo_v(amo_v), .amo_func(amo_func), .amo_addr(amo_addr), .amo_data(amo_data),
      .amo_sz(amo_sz), .amo_pdst(amo_pdst), .amo_owner(amo_owner), .amo_ckpt(amo_ckpt), .amo_seq(amo_seq),
      .xl_satp(xl_satp), .xl_priv(xl_priv), .xl_sum(xl_sum), .xl_mxr(xl_mxr), .xl_flush(xl_flush),
      .ldp_addr(ldp_addr), .ldp_read(ldp_read), .ldp_rdata(ldp_rdata), .ldp_rvalid(ldp_rvalid),
      .stp_addr(stp_addr), .stp_read(stp_read), .stp_rdata(stp_rdata), .stp_rvalid(stp_rvalid),
      .dfault_v(dfault_v), .dfault_seq(dfault_seq), .dfault_ckpt(dfault_ckpt),
      .dfault_cause(dfault_cause), .dfault_tval(dfault_tval),
      .st_done(st_done), .st_done_ckpt(st_done_ckpt),
      .mem_raddr(mem_raddr), .mem_ren(mem_ren), .mem_rdata(mem_rdata), .mem_rvalid(mem_rvalid), .mem_wready(1'b1),
      .mem_wen(mem_wen), .mem_waddr(mem_waddr), .mem_wdata(mem_wdata), .mem_wmask(mem_wmask),
      .wb_busy(wb_busy),
      .ld_wb_v(ld_wb_v), .ld_wb_pdst(ld_wb_pdst), .ld_wb_owner(ld_wb_owner),
      .ld_wb_val(ld_wb_val), .ld_done(ld_done), .ld_done_ckpt(ld_done_ckpt),
      .commit(commit), .commit_idx(commit_idx),
      .rollback(rollback), .rollback_seq(rollback_seq), .dfault_taken(dfault_taken));

   // ---- register every output (reg-to-reg through the LSU), then fold to probe_out ----
   localparam OCW =
        IW*SBI + IW*LQI + 1 + 1                       // disp_sb/lq_idx, sb/lq_full
      + 1 + SEQW + CBITS + 4 + AW                     // dfault_*
      + 1 + CBITS                                     // st_done(_ckpt)
      + AW + 1 + 1 + AW + 64 + 8                      // mem_raddr/ren/wen/waddr/wdata/wmask
      + 1 + PBITS + SBITS + 64 + 1 + CBITS            // ld_wb_*, ld_done(_ckpt)
      + 56 + 1 + 56 + 1;                              // ldp/stp addr+read
   wire [OCW-1:0] douts =
        {disp_sb_idx, disp_lq_idx, sb_full, lq_full,
         dfault_v, dfault_seq, dfault_ckpt, dfault_cause, dfault_tval,
         st_done, st_done_ckpt,
         mem_raddr, mem_ren, mem_wen, mem_waddr, mem_wdata, mem_wmask,
         ld_wb_v, ld_wb_pdst, ld_wb_owner, ld_wb_val, ld_done, ld_done_ckpt,
         ldp_addr, ldp_read, stp_addr, stp_read};
   reg [OCW-1:0] dout_q = {OCW{1'b0}};
   always @(posedge clk) dout_q  <= douts;
   always @(posedge clk) probe_out <= ^dout_q;

endmodule

`default_nettype wire
