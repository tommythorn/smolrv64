`include "exec_pay.vh"
`default_nettype none

// Full sharded-OoO core (frontend + backend), ALU + LSU subset, with commit/CPR:
//   PC -> fetch/align -> decode -> [reg] -> rename -> dispatch
//      -> scheduler (scoreboard issue queues) -> execute (RF + ALU + AGU)
//      -> writeback / unified LSU
//   writeback -> scheduler wake (+ every RF copy)   [write-before-read forwarding]
//
// Commit/CPR (no ROB): one checkpoint per dispatched bundle; commit_ctl counts
// per-checkpoint in-flight instrs (incr at dispatch, decr at completion) and
// commits the oldest in order; bitmap freelists + MAP snapshot recover on a branch
// redirect. Back-pressure (`accept`) freezes the frontend on a full checkpoint
// ring, an empty freelist, a full scheduler, OR a full store buffer / load queue.
//
// LSU (M1): unified store buffer + load queue, flat byte-addressable data memory
// (dmem) stub, physical addresses (dTLB = identity). Loads/stores allocate an LSU
// slot at dispatch (mem_idx threaded through the scheduler like ckpt#), execute
// their AGU in the shard, and the LSU resolves ordering + byte-granular forwarding.
// A load completes at the LSU and writes back on its owner shard's lane (the LSU
// skips lanes busy with an ALU writeback -> no collision). Completion accounting:
// ALU/branch/store at issue; LOADS at LSU completion (excluded from the issue
// decrement, counted via ld_done). TODO: serialize fences/atomics/MMIO via a forced
// unique checkpoint (deferred; the M1 tests don't use them).
module backend_top
  #(parameter IW    = 4,
    parameter HW    = 8,
    parameter PCW   = 64,
    parameter SEQW  = 8,
    parameter ABITS = 6,
    parameter PBITS = 8,
    parameter SCHED_N = 16,      // CAM RS entries/shard. Sweep @3ns: select path was the
                                 // cap (N12=3.17 N16=4.49ns) until the age compare was
                                 // coarsened (low 4 seqno bits dropped) -> N16=2.55ns,
                                 // RS no longer the limiter (shared scoreboard write is).
    parameter CBITS = 2,
    parameter NCHK  = 4,
    parameter DCW   = 3,         // clog2(IW+1)
    parameter CNTW  = 3,         // per-checkpoint outstanding count width
    parameter SBITS = 2,         // clog2(IW) -- owner-shard id width
    parameter AW    = 64,
    parameter SBDEPTH= 4, parameter SBI = 2,   // small store buffer -> shallow byte-merge
    parameter LQDEPTH= 4, parameter LQI = 2,
    parameter MIDXW = 2,         // = max(SBI, LQI)
    parameter [PCW-1:0] RESET_PC = 0)
   (input  wire                    clk,
    input  wire                    reset,
    output wire [PCW-1:0]          imem_addr,
    input  wire [HW*16-1:0]        imem_data,
    input  wire [$clog2(HW+2)-1:0] imem_avail,
    // data memory port (flat byte-addressable stub; real D$ later)
    output wire [AW-1:0]           dmem_raddr,
    input  wire [63:0]             dmem_rdata,
    output wire                    dmem_wen,
    output wire [AW-1:0]           dmem_waddr,
    output wire [63:0]             dmem_wdata,
    output wire [7:0]              dmem_wmask,
    // observation: per-shard writeback + the branch redirect
    output wire [IW-1:0]           wb_valid,
    output wire [IW*PBITS-1:0]     wb_pr,
    output wire [IW*64-1:0]        wb_val,
    output wire                    redirect,
    output wire [PCW-1:0]          redirect_target,
    // observation: in-order commit (one checkpoint per pulse)
    output wire                    commit,
    output wire [CBITS-1:0]        commit_idx);

   // ---- redirect (from the execute bundle's oldest mispredicting branch) ----
   wire               eb_redirect;
   wire [63:0]        eb_target;
   wire [SEQW-1:0]    eb_rseq;
   wire [CBITS-1:0]   eb_rckpt;
   assign redirect        = eb_redirect;
   assign redirect_target = eb_target;

   // ---- frontend: fetch -> decode -> rename ----
   wire [IW-1:0]      r_valid, r_rd_v, r_is_branch, fe_stall;
   wire [IW*SEQW-1:0] r_seq;
   wire [IW*ABITS-1:0] r_rd;
   wire [IW*PBITS-1:0] ps1, ps2, pdst;
   wire [IW*`PAYW-1:0] r_pay;
   wire [CBITS-1:0]   r_ckpt, cur;

   // ---- commit control ----
   wire               cc_commit, cc_rollback, cc_full;
   wire [CBITS-1:0]   cc_commit_idx, cc_rollback_idx;

   // ---- per-slot memory-op classification (from the renamed payload) ----
   wire [IW-1:0]      slot_mem, slot_store, dl_is_load, dl_is_store;
   genvar gi;
   generate for (gi = 0; gi < IW; gi = gi + 1) begin : cls
      assign slot_mem[gi]   = r_pay[gi*`PAYW + `PAY_MEM];
      assign slot_store[gi] = r_pay[gi*`PAYW + `PAY_STORE];
      assign dl_is_load[gi]  = r_valid[gi] & slot_mem[gi] & ~slot_store[gi];
      assign dl_is_store[gi] = r_valid[gi] & slot_mem[gi] &  slot_store[gi];
   end endgenerate

   // ---- LSU dispatch allocation (combinational) ----
   wire [IW*SBI-1:0]  disp_sb_idx;
   wire [IW*LQI-1:0]  disp_lq_idx;
   wire               sb_full, lq_full;
   wire [IW*MIDXW-1:0] disp_mem_idx;
   generate for (gi = 0; gi < IW; gi = gi + 1) begin : midx
      assign disp_mem_idx[gi*MIDXW +: MIDXW] =
         dl_is_store[gi] ? disp_sb_idx[gi*SBI +: SBI] : disp_lq_idx[gi*LQI +: LQI];
   end endgenerate

   // ---- dispatch / back-pressure decision (on the renamed bundle) ----
   wire [IW-1:0]      disp_ready;
   wire               any_valid    = |r_valid;
   wire               can_dispatch = !cc_full && (&disp_ready) && !(|fe_stall)
                                     && !sb_full && !lq_full && !eb_redirect;
   wire               disp_fire    = any_valid && can_dispatch;
   wire               accept       = !any_valid || can_dispatch;   // else freeze frontend

   reg  [DCW-1:0]     disp_count;
   integer dc;
   always @* begin
      disp_count = {DCW{1'b0}};
      for (dc = 0; dc < IW; dc = dc + 1) disp_count = disp_count + r_valid[dc];
   end

   // a mispredicting branch reopens the span just after its own bundle's
   wire [CBITS-1:0]   rb_idx = eb_rckpt + 1'b1;

   frontend #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW), .ABITS(ABITS),
              .PBITS(PBITS), .NCHK(NCHK), .CBITS(CBITS), .RESET_PC(RESET_PC)) fe
     (.clk(clk), .reset(reset),
      .redirect(eb_redirect), .redirect_pc(eb_target),
      .redirect_seq(eb_rseq + 1'b1),          // target continues seqno after the branch
      .imem_addr(imem_addr), .imem_data(imem_data), .imem_avail(imem_avail),
      .accept(accept),
      .create(disp_fire), .commit(cc_commit), .commit_idx(cc_commit_idx),
      .rollback(cc_rollback), .rollback_idx(cc_rollback_idx),
      .r_valid(r_valid), .r_seq(r_seq), .r_rd(r_rd), .r_rd_v(r_rd_v),
      .ps1(ps1), .ps2(ps2), .pdst(pdst),
      .r_is_branch(r_is_branch),
      .r_pay(r_pay), .r_ckpt(r_ckpt), .cur(cur), .stall(fe_stall));

   // ---- scheduler bundle ----
   wire [IW-1:0]       iss_valid, iss_pdst_v;
   wire [IW*PBITS-1:0] iss_pdst, iss_ps1, iss_ps2;
   wire [IW*SEQW-1:0]  iss_seq;
   wire [IW*CBITS-1:0] iss_ckpt;
   wire [IW*MIDXW-1:0] iss_mem_idx;
   wire [IW*`PAYW-1:0] iss_pay;
   wire [IW-1:0]       wkv;          // effective writeback = wake source (ALU ∪ load)
   wire [IW*PBITS-1:0] wkp;
   // per-shard iterative-divide status (exec_bundle -> scheduler stall + commit count)
   wire [IW-1:0]       eb_exec_busy, eb_div_done;
   wire [IW*CBITS-1:0] eb_div_done_ckpt;
   // wake bus into the scheduler (select-time + completion-time) + per-shard issue stall
   wire [2*IW-1:0]       sched_wake_v;
   wire [2*IW*PBITS-1:0] sched_wake_pr;
   wire [IW-1:0]         busy_to_sched;

   wire [IW*CBITS-1:0] disp_ckpt = {IW{r_ckpt}};
   wire [IW-1:0]      sched_disp_valid = r_valid & {IW{disp_fire}};

   sched_bundle #(.SHARDS(IW), .PBITS(PBITS), .SEQW(SEQW), .N(SCHED_N),
                  .CBITS(CBITS), .MIDXW(MIDXW)) sb
     (.clk(clk), .reset(reset),
      .disp_valid(sched_disp_valid), .disp_seq(r_seq), .disp_pdst(pdst), .disp_pdst_v(r_rd_v),
      .disp_ps1(ps1), .disp_ps2(ps2),
      .disp_ps3({IW*PBITS{1'b0}}),   // FMA 3rd operand: p0 (always ready) until FP
      .disp_ckpt(disp_ckpt), .disp_mem_idx(disp_mem_idx),
      .disp_pay(r_pay), .disp_ready(disp_ready),
      .wake_valid(sched_wake_v), .wake_pr(sched_wake_pr),
      .squash(eb_redirect), .squash_seq(eb_rseq), .exec_busy(busy_to_sched),
      .iss_valid(iss_valid), .iss_pdst(iss_pdst), .iss_pdst_v(iss_pdst_v),
      .iss_ps1(iss_ps1), .iss_ps2(iss_ps2), .iss_ps3(),     // ps3 unused until FP execute
      .iss_seq(iss_seq),
      .iss_ckpt(iss_ckpt), .iss_mem_idx(iss_mem_idx), .iss_pay(iss_pay));

   // ---- per-issue memory-op decode (from the payload, for the LSU execute drive) ----
   wire [IW-1:0]      iss_mem, iss_store, iss_is_load, iss_is_mul;
   generate for (gi = 0; gi < IW; gi = gi + 1) begin : icl
      assign iss_mem[gi]     = iss_pay[gi*`PAYW + `PAY_MEM];
      assign iss_store[gi]   = iss_pay[gi*`PAYW + `PAY_STORE];
      assign iss_is_load[gi] = iss_valid[gi] & iss_mem[gi] & ~iss_store[gi];
      // M-ops (mul AND div) are deferred multi-cycle -> excluded from select-wake / the
      // issue-time commit decrement, counted at completion, and stall their shard.
      assign iss_is_mul[gi]  = iss_valid[gi] & iss_pay[gi*`PAYW + `PAY_MUL];
   end endgenerate

   // ================= registered issue stage (select | execute split) =================
   // The scheduler's combinational select (iss_*) is registered here; execute, LSU and
   // commit consume the registered q_iss_*. This breaks the old fused select->RF->ALU->wb
   // megapath. Latency-1 back-to-back is preserved by a SELECT-TIME wake (below): the
   // selected latency-1 dest is broadcast now, so a dependent is selected next cycle and
   // reads the result from the RF the cycle after (write-before-read across this stage).
   // A wrong-path op selected the same cycle a branch redirects is gated out here.
   reg  [IW-1:0]       q_iss_valid, q_iss_pdst_v;
   reg  [IW*PBITS-1:0] q_iss_pdst, q_iss_ps1, q_iss_ps2;
   reg  [IW*SEQW-1:0]  q_iss_seq;
   reg  [IW*CBITS-1:0] q_iss_ckpt;
   reg  [IW*MIDXW-1:0] q_iss_mem_idx;
   reg  [IW*`PAYW-1:0] q_iss_pay;
   wire [IW-1:0]       iss_squashed;
   genvar gq;
   generate for (gq = 0; gq < IW; gq = gq + 1) begin : sq
      assign iss_squashed[gq] = eb_redirect & ($signed(iss_seq[gq*SEQW +: SEQW] - eb_rseq) > 0);
   end endgenerate
   integer qi;
   initial begin q_iss_valid = {IW{1'b0}}; end
   always @(posedge clk) begin
      if (reset) q_iss_valid <= {IW{1'b0}};
      else begin
         q_iss_valid   <= iss_valid & ~iss_squashed;
         q_iss_pdst_v  <= iss_pdst_v;
         q_iss_pdst    <= iss_pdst;   q_iss_ps1 <= iss_ps1;  q_iss_ps2 <= iss_ps2;
         q_iss_seq     <= iss_seq;    q_iss_ckpt <= iss_ckpt;
         q_iss_mem_idx <= iss_mem_idx; q_iss_pay <= iss_pay;
      end
   end

   // execute-stage op decode (from the registered payload) -> LSU + commit
   wire [IW-1:0]   q_iss_mem, q_iss_store, q_iss_is_load, q_iss_is_mul;
   wire [IW*4-1:0] q_iss_nb;
   wire [IW-1:0]   q_iss_sgn;
   generate for (gi = 0; gi < IW; gi = gi + 1) begin : qicl
      wire [1:0] qsz = q_iss_pay[gi*`PAYW + 148 +: 2];
      assign q_iss_mem[gi]     = q_iss_pay[gi*`PAYW + `PAY_MEM];
      assign q_iss_store[gi]   = q_iss_pay[gi*`PAYW + `PAY_STORE];
      assign q_iss_sgn[gi]     = q_iss_pay[gi*`PAYW + `PAY_MSGN];
      assign q_iss_nb[gi*4+:4] = (4'd1 << qsz);
      assign q_iss_is_load[gi] = q_iss_valid[gi] & q_iss_mem[gi] & ~q_iss_store[gi];
      assign q_iss_is_mul[gi]  = q_iss_valid[gi] & q_iss_pay[gi*`PAYW + `PAY_MUL];
   end endgenerate

   // ---- wake: select-time (latency-1) + completion-time (load/divide via wb) ----
   // select-wake fires for a selected latency-1 register writer (not mem, not div).
   wire [IW-1:0]       sel_wake_v;
   wire [IW*PBITS-1:0] sel_wake_pr;
   generate for (gi = 0; gi < IW; gi = gi + 1) begin : selw
      assign sel_wake_v[gi]             = iss_valid[gi] & iss_pdst_v[gi]
                                          & ~iss_mem[gi] & ~iss_is_mul[gi];
      assign sel_wake_pr[gi*PBITS +: PBITS] = iss_pdst[gi*PBITS +: PBITS];
   end endgenerate
   assign sched_wake_v  = {sel_wake_v, wkv};        // [hi]=select, [lo]=completion
   assign sched_wake_pr = {sel_wake_pr, wkp};

   // a divide heading to / running on a shard's divider stalls that shard's issue
   assign busy_to_sched = q_iss_is_mul | eb_exec_busy;

   // ---- commit control: count by completion (loads at LSU), commit in order ----
   wire               lsu_ld_done;
   wire [CBITS-1:0]   lsu_ld_done_ckpt;
   commit_ctl #(.NCHK(NCHK), .CBITS(CBITS), .IW(IW), .CNTW(CNTW), .DCW(DCW)) cc
     (.clk(clk), .reset(reset), .cur(cur),
      .disp_fire(disp_fire), .disp_count(disp_count),
      .iss_valid(q_iss_valid), .iss_is_load(q_iss_is_load), .iss_is_div(q_iss_is_mul), .iss_ckpt(q_iss_ckpt),
      .ld_done(lsu_ld_done), .ld_done_ckpt(lsu_ld_done_ckpt),
      .div_done(eb_div_done), .div_done_ckpt(eb_div_done_ckpt),
      .redirect(eb_redirect), .redirect_ckpt(rb_idx),
      .create(),
      .commit(cc_commit), .commit_idx(cc_commit_idx),
      .rollback(cc_rollback), .rollback_idx(cc_rollback_idx), .full(cc_full));

   assign commit     = cc_commit;
   assign commit_idx = cc_commit_idx;

   // ---- execute bundle (2-stage RR|EX + forwarding + wb broadcast + branch) ----
   wire [IW*64-1:0]   eb_agu, eb_stdata;
   wire [IW-1:0]      eb_wb_busy;
   wire               lsu_ld_wb_v;
   wire [SBITS-1:0]   lsu_ld_wb_owner;
   wire [PBITS-1:0]   lsu_ld_wb_pdst;
   wire [63:0]        lsu_ld_wb_val;
   // EX-stage LSU control (from exec_bundle, aligned with eb_agu/eb_stdata)
   wire [IW-1:0]      ex_valid, ex_mem, ex_store, ex_msigned;
   wire [IW*SEQW-1:0] ex_seq;
   wire [IW*CBITS-1:0] ex_ckpt;
   wire [IW*MIDXW-1:0] ex_mem_idx;
   wire [IW*2-1:0]    ex_msize;

   exec_bundle #(.SHARDS(IW), .SBITS(SBITS), .PBITS(PBITS), .SEQW(SEQW), .CBITS(CBITS), .MIDXW(MIDXW)) eb
     (.clk(clk),
      .iss_valid(q_iss_valid), .iss_seq(q_iss_seq), .iss_pdst(q_iss_pdst),
      .iss_pdst_v(q_iss_pdst_v), .iss_ps1(q_iss_ps1), .iss_ps2(q_iss_ps2),
      .iss_ckpt(q_iss_ckpt), .iss_mem_idx(q_iss_mem_idx), .iss_pay(q_iss_pay),
      .squash(eb_redirect), .squash_seq(eb_rseq),
      .exec_busy(eb_exec_busy), .div_done(eb_div_done), .div_done_ckpt(eb_div_done_ckpt),
      .lsu_wb_v(lsu_ld_wb_v), .lsu_wb_owner(lsu_ld_wb_owner),
      .lsu_wb_pr(lsu_ld_wb_pdst), .lsu_wb_val(lsu_ld_wb_val), .wb_busy(eb_wb_busy),
      .wb_valid(wkv), .wb_pr(wkp), .wb_val(wb_val),
      .ex_valid(ex_valid), .ex_seq(ex_seq), .ex_ckpt(ex_ckpt), .ex_mem_idx(ex_mem_idx),
      .ex_mem(ex_mem), .ex_store(ex_store), .ex_msize(ex_msize), .ex_msigned(ex_msigned),
      .agu_addr(eb_agu), .st_data(eb_stdata),
      .redirect(eb_redirect), .redirect_target(eb_target),
      .redirect_seq(eb_rseq), .redirect_ckpt(eb_rckpt));

   // ---- LSU execute-port drive (EX stage: bypassed AGU/store-data + EX control) ----
   wire [IW-1:0]      exe_st_v, exe_ld_v;
   wire [IW*SBI-1:0]  exe_st_idx;
   wire [IW*LQI-1:0]  exe_ld_idx;
   wire [IW*AW-1:0]   exe_st_addr, exe_ld_addr;
   wire [IW*64-1:0]   exe_st_data;
   wire [IW*4-1:0]    exe_st_nb, exe_ld_nb;
   wire [IW-1:0]      exe_ld_sgn;
   generate for (gi = 0; gi < IW; gi = gi + 1) begin : exd
      assign exe_st_v[gi] = ex_valid[gi] & ex_mem[gi] &  ex_store[gi];
      assign exe_ld_v[gi] = ex_valid[gi] & ex_mem[gi] & ~ex_store[gi];
      assign exe_st_idx[gi*SBI +: SBI] = ex_mem_idx[gi*MIDXW +: SBI];
      assign exe_ld_idx[gi*LQI +: LQI] = ex_mem_idx[gi*MIDXW +: LQI];
      assign exe_st_addr[gi*AW +: AW]  = eb_agu[gi*64 +: AW];
      assign exe_ld_addr[gi*AW +: AW]  = eb_agu[gi*64 +: AW];
      assign exe_st_data[gi*64 +: 64]  = eb_stdata[gi*64 +: 64];
      assign exe_st_nb[gi*4 +: 4]      = (4'd1 << ex_msize[gi*2 +: 2]);
      assign exe_ld_nb[gi*4 +: 4]      = (4'd1 << ex_msize[gi*2 +: 2]);
      assign exe_ld_sgn[gi]            = ex_msigned[gi];
   end endgenerate

   lsu #(.IW(IW), .SBITS(SBITS), .PBITS(PBITS), .SEQW(SEQW), .CBITS(CBITS), .AW(AW),
         .SBDEPTH(SBDEPTH), .SBI(SBI), .LQDEPTH(LQDEPTH), .LQI(LQI)) u_lsu
     (.clk(clk), .reset(reset),
      .disp_fire(disp_fire), .disp_is_load(dl_is_load), .disp_is_store(dl_is_store),
      .disp_seq(r_seq), .disp_ckpt(disp_ckpt), .disp_pdst(pdst),
      .disp_sb_idx(disp_sb_idx), .disp_lq_idx(disp_lq_idx),
      .sb_full(sb_full), .lq_full(lq_full),
      .exe_st_v(exe_st_v), .exe_st_idx(exe_st_idx), .exe_st_addr(exe_st_addr),
      .exe_st_data(exe_st_data), .exe_st_nb(exe_st_nb),
      .exe_ld_v(exe_ld_v), .exe_ld_idx(exe_ld_idx), .exe_ld_addr(exe_ld_addr),
      .exe_ld_nb(exe_ld_nb), .exe_ld_sgn(exe_ld_sgn),
      .mem_raddr(dmem_raddr), .mem_rdata(dmem_rdata),
      .mem_wen(dmem_wen), .mem_waddr(dmem_waddr), .mem_wdata(dmem_wdata), .mem_wmask(dmem_wmask),
      .wb_busy(eb_wb_busy),
      .ld_wb_v(lsu_ld_wb_v), .ld_wb_pdst(lsu_ld_wb_pdst), .ld_wb_owner(lsu_ld_wb_owner),
      .ld_wb_val(lsu_ld_wb_val), .ld_done(lsu_ld_done), .ld_done_ckpt(lsu_ld_done_ckpt),
      .commit(cc_commit), .commit_idx(cc_commit_idx),
      .rollback(eb_redirect), .rollback_seq(eb_rseq));

   assign wb_valid = wkv;
   assign wb_pr    = wkp;
endmodule

`default_nettype wire
