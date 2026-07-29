`default_nettype none

// One shard's execute slice, TWO pipeline stages:
//   RR : read this shard's RF copy, flop the operands + control.
//   EX : bypass-mux the operands (forward from the registered writeback, 1- and
//        2-ahead), run exec_alu / AGU / branch / the M-units, flop the result.
//   the result flop IS the writeback -> RF-write and the broadcast come from a flop
//   (short path). A dependent reads its producer from: the 1-ahead wb (its EX), the
//   2-ahead wb (delayed one more cycle), or the RF (3+ behind, write-before-read).
//
// Latency-1 ALU is preserved by select-time wake (in the scheduler) + this forwarding.
// Loads / mul / divide are deferred: they complete later and their consumers read the
// RF (woken at completion), so they don't need forwarding.
module exec_shard
  #(parameter SHARDS = 4,
    parameter SBITS  = 2,
    parameter NPHYS  = 256,
    parameter PBITS  = 8,
    parameter POOL   = 64,
    parameter IDXB   = 6,
    parameter SEQW   = 8,
    parameter CBITS  = 2,
    parameter MIDXW  = 3)
   (input  wire                    clk,
    // ---- RR: issue from this shard's scheduler ----
    input  wire                    iss_valid,
    input  wire [SEQW-1:0]         iss_seq,
    input  wire [PBITS-1:0]        iss_pdst,
    input  wire                    iss_pdst_v,
    input  wire [PBITS-1:0]        iss_ps1,
    input  wire [PBITS-1:0]        iss_ps2,
    input  wire [PBITS-1:0]        iss_ps3,      // FMA 3rd operand
    input  wire [CBITS-1:0]        iss_ckpt,
    input  wire [MIDXW-1:0]        iss_mem_idx,
    input  wire [31:0]             iss_insn,      // RVC-expanded instruction (for decode_fp)
    input  wire                    squash,        // branch redirect this cycle
    input  wire [SEQW-1:0]         squash_seq,
    // execute payload (decode_exec ctl + imm/pc)
    input  wire [5:0]              alu_op,
    input  wire                    alu_w,
    input  wire                    alu_uw,
    input  wire [1:0]              op1_sel,
    input  wire                    op2_imm,
    input  wire                    res_link,
    input  wire                    is_rvc,
    input  wire                    is_mem,
    input  wire                    is_store,
    input  wire [1:0]              mem_size,
    input  wire                    mem_signed,
    input  wire                    is_branch,
    input  wire                    is_jump,
    input  wire                    is_mul,
    input  wire                    is_csr,
    input  wire [2:0]              csr_func,
    input  wire                    is_serialize,
    input  wire                    is_fencei,
    input  wire                    is_amo,
    input  wire                    is_cbo,        // Zicbom/Zicboz CBO (rides the store path)
    input  wire                    cbo_zero,      // cbo.zero (else clean/flush/inval)
    input  wire                    cbo_keep,      // cbo.clean keep-valid (else invalidate)
    input  wire [4:0]              amo_func,
    input  wire [2:0]              br_func,
    input  wire [63:0]             imm,
    input  wire [63:0]             pc,
    // CSR file (system op executes here when oldest -> precise)
    input  wire [63:0]             csr_rdata,    // old value at imm[11:0]
    input  wire [63:0]             csr_redir_target, // trap/xret target (from csr_file)
    input  wire                    csr_redir_valid,  // active sys op redirects (trap/xret/illegal)
    input  wire                    csr_redir_is_trap,// the redirect is an exception (roll back TO ckpt)
    input  wire                    csr_illegal,      // active CSR op is illegal -> no rd write
    output wire                    csr_req_v,    // drive the CSR update port
    output wire                    csr_req_is_csr,
    output wire [2:0]              csr_req_func,
    output wire [11:0]             csr_req_addr,
    output wire [63:0]             csr_req_src,
    output wire [63:0]             csr_req_pc,
    output wire [11:0]             csr_rd_addr,  // combinational read addr (= ex_imm[11:0])
    // ---- RF write source (registered ALU/M ∪ the LSU load), all shards ----
    input  wire [SHARDS-1:0]       wb_valid_in,
    input  wire [SHARDS*PBITS-1:0] wb_pr_in,
    input  wire [SHARDS*64-1:0]    wb_val_in,
    // ---- forwarding sources: registered ALU/M only (the LSU load is NOT forwarded --
    //      its consumers wake at completion and read the RF). 1-ahead = byp_*, 2-ahead
    //      = fw2_* (byp delayed one cycle). ----
    input  wire [SHARDS-1:0]       byp_valid,
    input  wire [SHARDS*PBITS-1:0] byp_pr,
    input  wire [SHARDS*64-1:0]    byp_val,
    input  wire [SHARDS-1:0]       fw2_valid,
    input  wire [SHARDS*PBITS-1:0] fw2_pr,
    input  wire [SHARDS*64-1:0]    fw2_val,
    // ---- EX: this shard's registered writeback (broadcast + wake source) ----
    output reg                     wb_valid,
    output reg  [PBITS-1:0]        wb_pr,
    output reg  [63:0]             wb_val,
    output reg  [SEQW-1:0]         wb_seq,         // seqno of this writeback (cosim capture)
    // ---- EX: branch/jump resolution ----
    input  wire [63:0]             pred_npc,      // frontend's chosen next PC for this op's bundle
                                                  // (RR-time value; compares precomputed at RR)
    output wire                    br_redirect,
    output wire [63:0]             br_target,
    output wire [63:0]             br_pc,         // PC of the redirecting op (debug: control-flow trace)
    output wire                    fencei_redir_o,     // this shard is redirecting for a FENCE.I
    output wire [SEQW-1:0]         br_seq,
    output wire                    br_is_trap,    // redirect is an exception (roll back TO ckpt)
    // resolve/training port (predictor): a genuine CTI resolved on this lane
    output wire                    res_v,
    output wire                    res_cbr,       // conditional branch (vs jump)
    output wire                    res_call,      // jump with a link dest (rd in {x1,x5})
    output wire                    res_ret,       // JALR return (rs1 link, rd not)
    output wire                    res_taken,     // resolved direction (jumps: 1)
    output wire [63:0]             res_tgt,       // resolved taken-target
    // ---- EX: LSU drive (aligned with agu/st_data) ----
    output wire                    ex_valid,
    output wire [SEQW-1:0]         ex_seq,
    output wire [CBITS-1:0]        ex_ckpt,
    output wire [MIDXW-1:0]        ex_mem_idx,
    output wire                    ex_mem,
    output wire                    ex_store,
    output wire                    ex_cbo,        // EX op is a CBO maintenance op
    output wire                    ex_cbo_zero,
    output wire                    ex_cbo_keep,
    output wire                    ex_fp,         // EX op is an FP instruction (FLW/FLD box at LSU)
    output wire [1:0]              ex_msize,
    output wire                    ex_msigned,
    output wire [63:0]             agu_addr,
    output wire [63:0]             st_data,
    // ---- EX: atomic (A ext) drive: serialized RMW in the LSU ----
    output wire                    ex_amo,        // this lane has an atomic at EX
    output wire [4:0]              ex_amo_func,
    output wire [PBITS-1:0]        ex_amo_pdst,
    // ---- M-unit status ----
    output wire                    exec_busy,
    output wire                    div_done,
    output wire [CBITS-1:0]        div_done_ckpt,
    output wire                    fp_done,           // FPU-arith completion (deferred, like div_done)
    output wire [CBITS-1:0]        fp_done_ckpt,
    output wire                    fp_flags_we,       // an FP op produced exception flags this cycle
    output wire [4:0]              fp_flags,          // those flags (CVFPU completion OR in-core compare)
    output wire                    iss_fp_dirty,      // issue-time in-core FP writer (compare/FSGNJ/FMV.x.X)
                                                      // -> mstatus.FS Dirty, commit-gated in commit_ctl
    input  wire [2:0]              i_frm,             // fcsr.frm for dynamic rounding (rm==111)
    input  wire                    i_fs_off,          // mstatus.FS==Off -> suppress FP exec (trapped)
    // next-cycle writeback on this shard's lane (for the LSU's lane reservation)
    output wire                    wb_next);

`include "smolrv64_fp_ops.vh"            // fcmp_s/d, fclass_s/d (in-core FP ops)
   function automatic older;          // a strictly older than b (wrap-safe)
      input [SEQW-1:0] a, bb; older = ($signed(a - bb) < 0);
   endfunction

   // ============================== RR stage ==============================
   wire [63:0] rf_rs1, rf_rs2, rf_rs3;
   rf_shard #(.SHARDS(SHARDS), .SBITS(SBITS), .NPHYS(NPHYS), .PBITS(PBITS),
              .POOL(POOL), .IDXB(IDXB)) rf
     (.clk(clk), .wr_valid(wb_valid_in), .wr_pr(wb_pr_in), .wr_val(wb_val_in),
      .ra1(iss_ps1), .ra2(iss_ps2), .ra3(iss_ps3), .rd1(rf_rs1), .rd2(rf_rs2), .rd3(rf_rs3));

   // FP control (decode_fp at RR; registered into EX alongside operands)
   wire        fp_v_d, fp_use_d;  wire [2:0] fp_cls_d, fp_src_d, fp_dst_d, fp_rnd_d;
   wire [3:0]  fp_op_d;  wire fp_mod_d;  wire [1:0] fp_int_d, fp_o0_d, fp_o1_d, fp_o2_d;  wire fp_o0i_d, fp_wrfp_d;
   decode_fp u_dfp (.insn(iss_insn), .fp_valid(fp_v_d), .use_fpu(fp_use_d), .fp_class(fp_cls_d),
      .op(fp_op_d), .op_mod(fp_mod_d), .src_fmt(fp_src_d), .dst_fmt(fp_dst_d), .int_fmt(fp_int_d),
      .rnd(fp_rnd_d), .op0_sel(fp_o0_d), .op1_sel(fp_o1_d), .op2_sel(fp_o2_d),
      .op0_int(fp_o0i_d), .wr_fp(fp_wrfp_d));
   // FS-dirty (issue-time, commit-gated in commit_ctl): the in-core FP ops that write FP
   // state -- FSGNJ (cls 1) / compare (cls 2, sets fcsr flags) / FMV.x.X (cls 4, writes an
   // f-reg). FMV.X.W (cls 3) and FCLASS (cls 5) write an x-reg with no flags -> NOT dirty.
   // CVFPU arith dirties via fp_done; FP loads via the LSU. Aligned with iss_valid/iss_ckpt
   // so commit_ctl sets this checkpoint's pending bit the same cycle it decrements the op.
   assign iss_fp_dirty = iss_valid & fp_v_d & ~fp_use_d
                       & (fp_cls_d==3'd1 | fp_cls_d==3'd2 | fp_cls_d==3'd4);

   // squash an op that becomes wrong-path the cycle it is flopped into EX
   wire rr_kill = squash & older(squash_seq, iss_seq);

   reg              ex_v, ex_pdv, ex_w, ex_uw, ex_o2i, ex_link, ex_rvc, ex_memr,
                    ex_str, ex_msgn, ex_br, ex_jmp, ex_mulr, ex_csr, ex_ser, ex_amor, ex_fencei,
                    ex_cbor, ex_cbozr, ex_cbokr;
   reg  [4:0]       ex_amof;
   reg  [PBITS-1:0] ex_pd, ex_p1, ex_p2, ex_p3;
   reg  [SEQW-1:0]  ex_sq;
   reg  [CBITS-1:0] ex_ck;
   reg  [MIDXW-1:0] ex_mi;
   reg  [63:0]      ex_r1, ex_r2, ex_r3, ex_imm, ex_pc;
   reg  [5:0]       ex_aop;
   reg  [1:0]       ex_o1s, ex_msz;
   reg  [2:0]       ex_bf, ex_csrf;
   // FP control registered into EX
   reg              ex_fpv, ex_fpu;  reg [2:0] ex_fpcls, ex_fpsrc, ex_fpdst, ex_fprnd;
   reg  [3:0]       ex_fpop;  reg ex_fpmod;  reg [1:0] ex_fpint, ex_fpo0, ex_fpo1, ex_fpo2;  reg ex_fpo0i;
   reg              fpu_inflight = 1'b0;  reg [SEQW-1:0] fp_seq;  reg [PBITS-1:0] fp_pd;  reg [CBITS-1:0] fp_ck;
   reg              fp_pdv;               // in-flight op has a real dest (ex_pdv): an F2I to x0 must NOT write back
   reg              fp_dst32;             // in-flight op's FP32 result to an F-REG -> NaN-box the writeback
   reg              ex_fpwrfp;            // dest is an f-register (decode wr_fp; F2I writes an x-reg)
   reg              ex_fs_off;            // this op executed with FS==Off -> suppress FP, it traps
   reg  [31:0]      ex_insn;
   // mispredict compares precomputed at RR (both candidate next-PCs are payload-
   // static for branches/JAL) -> the EX redirect bit is a mux, not a 64b compare.
   reg              ex_mist, ex_misn;
   reg  [63:0]      ex_pnpc;              // for the JALR-only EX-time compare
   always @(posedge clk) begin
      ex_fpv<=fp_v_d; ex_fpu<=fp_use_d; ex_fpcls<=fp_cls_d; ex_fpsrc<=fp_src_d; ex_fpdst<=fp_dst_d;
      ex_fprnd<=fp_rnd_d; ex_fpop<=fp_op_d; ex_fpmod<=fp_mod_d; ex_fpint<=fp_int_d;
      ex_fpo0<=fp_o0_d; ex_fpo1<=fp_o1_d; ex_fpo2<=fp_o2_d;  ex_fpo0i<=fp_o0i_d;  ex_insn<=iss_insn;
      ex_fpwrfp<=fp_wrfp_d;
      ex_fs_off<=i_fs_off;
      ex_v   <= iss_valid & ~rr_kill;
      ex_pdv <= iss_pdst_v; ex_pd <= iss_pdst; ex_p1 <= iss_ps1; ex_p2 <= iss_ps2; ex_p3 <= iss_ps3;
      ex_sq  <= iss_seq; ex_ck <= iss_ckpt; ex_mi <= iss_mem_idx;
      ex_r1  <= rf_rs1; ex_r2 <= rf_rs2; ex_r3 <= rf_rs3; ex_imm <= imm; ex_pc <= pc;
      ex_aop <= alu_op; ex_w <= alu_w; ex_uw <= alu_uw; ex_o1s <= op1_sel;
      ex_o2i <= op2_imm; ex_link <= res_link; ex_rvc <= is_rvc;
      ex_memr <= is_mem; ex_str <= is_store; ex_msz <= mem_size; ex_msgn <= mem_signed;
      ex_br  <= is_branch; ex_jmp <= is_jump; ex_mulr <= is_mul; ex_bf <= br_func;
      ex_csr <= is_csr; ex_csrf <= csr_func; ex_ser <= is_serialize;
      ex_amor <= is_amo; ex_amof <= amo_func; ex_fencei <= is_fencei;
      ex_cbor <= is_cbo; ex_cbozr <= cbo_zero; ex_cbokr <= cbo_keep;
      ex_mist <= ((pc + imm) != pred_npc);
      ex_misn <= ((pc + (is_rvc ? 64'd2 : 64'd4)) != pred_npc);
      ex_pnpc <= pred_npc;
   end

   // ============================== EX stage ==============================
   // operand forwarding: 1-ahead = wb_*_in (this cycle's registered writebacks),
   // 2-ahead = fw2_* (those delayed one more cycle). A physreg is written once, so
   // a tag matches at most one source; prefer the newer (1-ahead).
   reg [63:0] op1f, op2f, op3f;
   integer s;
   always @* begin
      op1f = ex_r1; op2f = ex_r2; op3f = ex_r3;
      for (s = 0; s < SHARDS; s = s + 1) begin
         if (fw2_valid[s] && fw2_pr[s*PBITS +: PBITS] == ex_p1) op1f = fw2_val[s*64 +: 64];
         if (fw2_valid[s] && fw2_pr[s*PBITS +: PBITS] == ex_p2) op2f = fw2_val[s*64 +: 64];
         if (fw2_valid[s] && fw2_pr[s*PBITS +: PBITS] == ex_p3) op3f = fw2_val[s*64 +: 64];
         if (byp_valid[s] && byp_pr[s*PBITS +: PBITS] == ex_p1) op1f = byp_val[s*64 +: 64];
         if (byp_valid[s] && byp_pr[s*PBITS +: PBITS] == ex_p2) op2f = byp_val[s*64 +: 64];
         if (byp_valid[s] && byp_pr[s*PBITS +: PBITS] == ex_p3) op3f = byp_val[s*64 +: 64];
      end
   end

   wire [63:0] result, cmp_e_x;
   wire        cmp_eq, cmp_lt, cmp_ltu;
   exec_alu ea
     (.alu_op(ex_aop), .alu_w(ex_w), .alu_uw(ex_uw), .op1_sel(ex_o1s),
      .op2_imm(ex_o2i), .res_link(ex_link), .is_rvc(ex_rvc),
      .rs1_val(op1f), .rs2_val(op2f), .imm(ex_imm), .pc(ex_pc),
      .result(result), .addr(agu_addr),
      .cmp_eq(cmp_eq), .cmp_lt(cmp_lt), .cmp_ltu(cmp_ltu));

   // M-units (EX-stage start, deferred completion) — shared per shard
   wire mul_op = ex_mulr & ~ex_bf[2];
   wire div_op = ex_mulr &  ex_bf[2];
   wire        mbusy, mdone;  wire [63:0] mres;
   wire        dbusy, ddone;  wire [63:0] dres;
   // CVFPU status (declared here: munit_busy consumes it)
   wire fp_iss_ready, fp_res_valid, fpu_busyo;  wire [63:0] fp_res_data;  wire [4:0] fp_fflags;
   // munit_busy MUST include the FPU's OWN busy/not-ready, not just our single-op
   // fpu_inflight tracker: EX has no hold, so an FP op that arrives when the unit
   // cannot accept it EVAPORATES (never starts, never writes back) and its consumer
   // then reads the destination physreg's STALE value. The window is the tail of an
   // iterative op (fpnew's divsqrt is not pipelined): our fpu_inflight clears when the
   // divide's result returns while the unit is still internally busy, so the very next
   // FP op is lost. Seen on FPGA as grep's `fdiv.s; ...; fcvt.lu.s s1` retiring with s1
   // holding a stale NaN-boxed {ffffffff,1} -> gnulib next_prime grinds forever (the
   // ubuntu boot "hang"). fpu_busyo was wired up but never consumed.
   wire        munit_busy = mbusy | dbusy | fpu_inflight | fpu_busyo | ~fp_iss_ready;
   reg  [PBITS-1:0] m_pdst;  reg [SEQW-1:0] m_seq;  reg [CBITS-1:0] m_ck;  reg m_pdv;
   wire m_squash_now = squash & older(squash_seq, ex_sq);
   wire m_start = ex_v & ex_mulr & ~munit_busy & ~m_squash_now;
   wire m_abort = munit_busy & squash & older(squash_seq, m_seq);
   mul3 mu (.clk(clk), .reset(1'b0), .start(m_start & mul_op), .abort(m_abort),
            .rs1(op1f), .rs2(op2f), .f3(ex_bf), .is_w(ex_w),
            .busy(mbusy), .done(mdone), .result(mres));
   divider dv (.clk(clk), .reset(1'b0), .start(m_start & div_op), .abort(m_abort),
               .rs1(op1f), .rs2(op2f), .f3(ex_bf), .is_w(ex_w),
               .busy(dbusy), .done(ddone), .result(dres));
   always @(posedge clk) if (m_start) begin m_pdst <= ex_pd; m_seq <= ex_sq; m_ck <= ex_ck; m_pdv <= ex_pdv; end
   wire        m_complete = (mdone | ddone) & ~m_abort;
   wire [63:0] m_res = mdone ? mres : dres;
`ifdef MUL_TRACE
   always @(posedge clk) begin
      if (m_start & mul_op)  $display("[MULS] seq=%0d op1=%h op2=%h f3=%b w=%b", ex_sq, op1f, op2f, ex_bf, ex_w);
      if (m_complete & mdone) $display("[MULC] seq=%0d res=%h pd=%0d", m_seq, m_res, m_pdst);
   end
`endif

   // ---- per-shard FP-arith unit (CVFPU): one op in flight, deferred like the divider ----
   // FS-disabled: this FP op executed with mstatus.FS==Off -> it raises an illegal trap
   // (handled in backend_top) and must produce NO FP side effect (no CVFPU start, no in-core
   // writeback, no fflags). Covers arith + in-core; FP load/store are gated at the LSU.
   wire fp_dis   = ex_v & ex_fpv & ex_fs_off;
   wire fp_arith = ex_v & ex_fpv & ex_fpu & ~fp_dis;
   wire fp_squash_now = squash & older(squash_seq, ex_sq);
   wire fp_abort = fpu_inflight & squash & older(squash_seq, fp_seq);
   // A squashed in-flight FP op is NEVER flushed out of the CVFPU -- a mid-op flush
   // leaves the pipe corrupt (observed: the flushed op's result still emerges, and a
   // cleanly restarted I2F then computes a NaN). Instead the op DRAINS: fp_zomb marks
   // it dead, fpu_inflight stays up (the scheduler stalls this shard via munit_busy)
   // and the eventual result is dropped.
   reg  fp_zomb;  initial fp_zomb = 1'b0;
   // a fresh FP op may start only when the unit is free and nothing older is squashing it.
   wire fp_start = fp_arith & ~fpu_inflight & ~mbusy & ~dbusy & ~fp_squash_now;
   function [63:0] fpsel; input [1:0] s; input [63:0] a, b, c;
      fpsel = (s==2'd1) ? a : (s==2'd2) ? b : (s==2'd3) ? c : 64'd0; endfunction  // 3=frs3 (FMA)
   // unbox a single from an f-register: a properly NaN-boxed value yields its low 32 bits;
   // anything else (e.g. a double, or a raw int) is the canonical single NaN (RISC-V spec).
   function [31:0] unbox_s; input [63:0] x;
      unbox_s = (x[63:32]==32'hffffffff) ? x[31:0] : 32'h7fc00000; endfunction
   wire [63:0] fpo0r = fpsel(ex_fpo0, op1f, op2f, op3f);
   wire [63:0] fpo1r = fpsel(ex_fpo1, op1f, op2f, op3f);
   wire [63:0] fpo2r = fpsel(ex_fpo2, op1f, op2f, op3f);
   // Feed the CVFPU a properly-boxed single for FP32 ops (unbox: real single, else canon NaN).
   // op0 may be an INTEGER source (I2F) -> pass it through unmolested.
   wire        src32 = (ex_fpsrc==3'd0);
   wire [63:0] fpo0  = (src32 & ~ex_fpo0i) ? {32'hffffffff, unbox_s(fpo0r)} : fpo0r;
   wire [63:0] fpo1  = src32 ? {32'hffffffff, unbox_s(fpo1r)} : fpo1r;
   wire [63:0] fpo2  = src32 ? {32'hffffffff, unbox_s(fpo2r)} : fpo2r;
   fp_unit #(.TAGW(1)) u_fpu
     (.clk(clk), .reset(1'b0),
      .iss_valid(fp_start), .iss_ready(fp_iss_ready),
      .iss_op(ex_fpop), .iss_op_mod(ex_fpmod), .iss_src_fmt(ex_fpsrc), .iss_dst_fmt(ex_fpdst),
      .iss_int_fmt(ex_fpint), .iss_rnd(ex_fprnd==3'b111 ? i_frm : ex_fprnd),  // dyn rm (rm=111) -> fcsr.frm
      .iss_operands({fpo2,fpo1,fpo0}), .iss_tag(1'b0),
      .res_valid(fp_res_valid), .res_ready(1'b1), .res_data(fp_res_data), .res_fflags(fp_fflags),
      .res_tag(), .flush(1'b0), .busy(fpu_busyo));
   always @(posedge clk) begin
      if (fp_start & fp_iss_ready) begin fpu_inflight<=1'b1; fp_zomb<=1'b0;
                                         fp_seq<=ex_sq; fp_pd<=ex_pd; fp_ck<=ex_ck; fp_pdv<=ex_pdv; fp_dst32<=(ex_fpdst==3'd0) & ex_fpwrfp; end
      else if (fp_res_valid & fpu_inflight)    begin fpu_inflight<=1'b0; fp_zomb<=1'b0; end
      else if (fp_abort)                       fp_zomb<=1'b1;   // drain, don't flush
`ifdef FPDBG
      if (fp_start)     $display("[FPD %m] START t=%0t seq=%0d ck=%0d op=%0d fpo0=%h rdy=%b infl=%b zomb=%b", $time, ex_sq, ex_ck, ex_fpop, fpo0, fp_iss_ready, fpu_inflight, fp_zomb);
      if (fp_arith & ~fp_start) $display("[FPD %m] NOSTART t=%0t seq=%0d ck=%0d infl=%b mb=%b db=%b sqn=%b", $time, ex_sq, ex_ck, fpu_inflight, mbusy, dbusy, fp_squash_now);
      if (fp_start & ~fp_iss_ready) $display("[FPD %m] NORDY t=%0t seq=%0d", $time, ex_sq);
      if (fp_abort)     $display("[FPD %m] ABORT t=%0t fpseq=%0d fpck=%0d sqseq=%0d infl=%b zomb=%b", $time, fp_seq, fp_ck, squash_seq, fpu_inflight, fp_zomb);
      if (fp_res_valid) $display("[FPD %m] RES   t=%0t data=%h infl=%b zomb=%b fpseq=%0d fpck=%0d", $time, fp_res_data, fpu_inflight, fp_zomb, fp_seq, fp_ck);
`endif
   end
   wire        fp_complete = fp_res_valid & fpu_inflight & ~fp_zomb & ~fp_abort;
   wire        fp_wb        = fp_complete & fp_pdv;   // writeback only when there's a real dest (F2I to x0 => no wb)
   assign      fp_done      = fp_complete;            // commit-count + fflags still fire for x0-dest ops
   assign      fp_done_ckpt = fp_ck;
   // ---- in-core FP ops (single-cycle, like the ALU): SGNJ/CMP/MVXF/MVFX/FCLASS ----
   wire       ex_fpd = ex_insn[25];          // 0=single 1=double
   wire [2:0] ex_f3  = ex_insn[14:12];
   // single in-core ops unbox their f-reg sources (non-boxed -> canonical NaN). FMV.X.W is a
   // raw 32-bit bit-move and must NOT unbox.
   wire [31:0] us1 = unbox_s(op1f);
   wire [31:0] us2 = unbox_s(op2f);
   wire [1:0] cmp_d2 = fcmp_d(ex_f3, op1f, op2f);
   wire [1:0] cmp_s2 = fcmp_s(ex_f3, us1, us2);
   reg [63:0] fp_incore_res;
   always @* begin
      case (ex_fpcls)
        3'd1: fp_incore_res = ex_fpd                                   // SGNJ.D / .S (NaN-boxed)
               ? (ex_f3==3'b000 ? {op2f[63], op1f[62:0]}
                : ex_f3==3'b001 ? {~op2f[63], op1f[62:0]}
                :                 {op2f[63]^op1f[63], op1f[62:0]})
               : {32'hffffffff, (ex_f3==3'b000 ? {us2[31], us1[30:0]}
                : ex_f3==3'b001 ? {~us2[31], us1[30:0]}
                :                 {us2[31]^us1[31], us1[30:0]})};
        3'd2: fp_incore_res = {63'd0, (ex_fpd ? cmp_d2[0] : cmp_s2[0])};      // FEQ/FLT/FLE -> int
        3'd3: fp_incore_res = ex_fpd ? op1f : {{32{op1f[31]}}, op1f[31:0]};   // FMV.X.D/W -> int (raw)
        3'd4: fp_incore_res = ex_fpd ? op1f : {32'hffffffff, op1f[31:0]};     // FMV.D/W.X -> fp (box)
        3'd5: fp_incore_res = ex_fpd ? fclass_d(op1f) : fclass_s(op1f);       // FCLASS -> int (unboxes inside)
        default: fp_incore_res = 64'd0;
      endcase
   end
   // ~ex_squash_fp (same term as ex_squash below, needed before its declaration point):
   // the in-core FP paths were MISSING the task-#30 EX-squash gate the ALU/CSR paths
   // have. Ungated, a wrong-path in-core FP op at EX still (a) wrote its physreg --
   // which rollback may already have freed+reused (the leaked-writeback corruption
   // class), (b) OR'd its compare NV flag into fcsr (fcsr must reflect RETIRED ops
   // only), and (c) set mstatus.FS=Dirty via fp_dirty -- spec-legal conservatism but
   // a divergence vs the in-order cosim model (gb5 @3.07B retirements: DUT FS=Dirty,
   // simmerv FS=Clean on the kernel's sstatus read).
   wire ex_squash_fp = squash & older(squash_seq, ex_sq);
   wire fp_incore_wb = ex_v & ex_fpv & ~ex_fpu & ex_pdv & ~fp_dis & ~ex_squash_fp;
   // FP exception flags to fcsr: from a CVFPU completion, or an in-core compare's NV bit.
   // (in-core ops other than compares raise no flags.) fp_incore raises flags even when rd=x0
   // is dropped (a compare always has rd, but gate on the op being valid, not on ex_pdv).
   wire       fp_icmp   = ex_v & ex_fpv & ~ex_fpu & (ex_fpcls==3'd2) & ~fp_dis & ~ex_squash_fp;
   wire       fp_icmp_nv= ex_fpd ? cmp_d2[1] : cmp_s2[1];
   assign     fp_flags_we = fp_complete | fp_icmp;
   assign     fp_flags    = fp_complete ? fp_fflags : {fp_icmp_nv, 4'd0};

   // result flop = writeback. ALU/link results, plus M completions (mux'd in; only one
   // M-op per shard at a time -> no collision). mem ops complete via the LSU.
   // a CSR op writes rd = the OLD csr value (read combinationally from csr_file).
   // An illegal CSR access traps and writes nothing.
   // EX-stage squash: an op already flopped into EX becomes wrong-path when a branch in
   // some lane redirects with a seqno OLDER than this op. rr_kill only covers the RR->EX
   // boundary; without this an in-EX ALU/CSR op still writes back, and since its physreg
   // may already be reclaimed+reused (CPR frees on rollback), the stale writeback corrupts
   // the new owner's value and re-wakes its consumer (the leaked-writeback bug, task #30).
   // The LSU has the equivalent guard (merge_squash); the ALU/CSR path was missing it.
   wire        ex_squash = squash & older(squash_seq, ex_sq);
   // (FS-dirty is now raised at issue -- see iss_fp_dirty above -- and commit-gated in
   // commit_ctl, so it is no longer asserted speculatively here at EX.)
   wire        csr_wb    = ex_v & ex_csr & ex_pdv & ~csr_illegal & ~ex_squash;
   // an atomic's rd comes from the LSU (ld_wb), not the ALU result -> exclude it here.
   // FP ops also don't take the ALU result: FPU-arith writes via fp_complete (below);
   // in-core FP ops (CMP/SGNJ/MV/FCLASS) are handled separately (TODO -- not yet).
   wire        ex_alu_wb = ex_v & ex_pdv & ~ex_memr & ~ex_mulr & ~ex_csr & ~ex_amor & ~ex_fpv & ~ex_squash;
   assign      wb_next   = ex_alu_wb | (m_complete & m_pdv) | csr_wb | fp_wb | fp_incore_wb;   // m_pdv/fp_pdv: a mul/div/F2I with no dest (x0) must NOT write back (stale don't-care pdst) -- done pulse still fires for the commit count
   always @(posedge clk) begin
      wb_valid <= ex_alu_wb | (m_complete & m_pdv) | csr_wb | fp_wb | fp_incore_wb;   // m_pdv/fp_pdv: a mul/div/F2I with no dest (x0) must NOT write back (stale don't-care pdst) -- done pulse still fires for the commit count
      wb_seq   <= fp_wb ? fp_seq : (m_complete ? m_seq : ex_sq);
      wb_pr    <= fp_wb ? fp_pd : (m_complete ? m_pdst : ex_pd);
      wb_val   <= fp_wb ? (fp_dst32 ? {32'hffffffff, fp_res_data[31:0]} : fp_res_data)
                : fp_incore_wb ? fp_incore_res
                : (m_complete ? m_res : (csr_wb ? csr_rdata : result));
   end

`ifdef SCDBG
   // TIME-read writeback bracket (pairs with csr_file's [TIMR-ANOM]): the same read's
   // value as it enters the wb flop. csr_file-clean + here-anomalous => corruption
   // between rdata and writeback.
   reg [63:0] twb_last; initial twb_last = 64'd0;
   reg [31:0] twb_np;   initial twb_np = 0;
   always @(posedge clk)
      if (csr_wb && ex_imm[11:0] == 12'hC01) begin
         if ((csr_rdata < twb_last || csr_rdata - twb_last > 64'd1_000_000) && twb_np < 32'd20000) begin
            $display("[TIMWB-ANOM t=%0t pc=%h val=%h last=%h]", $time, ex_pc, csr_rdata, twb_last);
            twb_np <= twb_np + 1;
         end
         twb_last <= csr_rdata;
      end
`endif
   // ---- CSR/system unit: read addr + update request + redirect ----
   assign csr_rd_addr    = ex_imm[11:0];                       // combinational read
   wire [63:0] csr_src   = ex_csrf[2] ? {59'b0, ex_imm[16:12]} : op1f;  // zimm | rs1
   assign csr_req_v      = ex_v & ex_ser & (ex_insn[6:0]==7'b1110011); // SYSTEM opcode only:
                       // a serializing op is SYSTEM (ecall/ebreak/xret/csr/wfi/sfence, opcode 1110011),
                       // AMO (0101111), or FENCE/FENCE.I (0001111). Only SYSTEM addresses the csr_file;
                       // gating on the opcode (not just ~amo/~fencei) keeps a plain FENCE -- which shares
                       // ex_ser and every other flag with a real SYSTEM op -- from being mis-decoded as one.
   assign csr_req_is_csr = ex_csr;
   assign csr_req_func   = ex_csrf;
   assign csr_req_addr   = ex_imm[11:0];
   assign csr_req_src    = csr_src;
   assign csr_req_pc     = ex_pc;

   // csr_file decides whether a system op redirects and where: ecall/ebreak/illegal-CSR
   // -> m/stvec (by delegation), mret/sret -> m/sepc. A plain LEGAL CSR op does NOT
   // redirect (redir_valid=0): it mutates state at EX non-speculatively (issues only
   // when oldest) and falls through to pc+4 -- the correct path, already in flight.
   wire [63:0] sys_target = csr_redir_target;
   wire sys_redirect = ex_v & ex_ser & csr_redir_valid;

   // busy = unit running OR an M-op in EX about to start it (so no second M-op is
   // selected in the gap before munit_busy rises). RR-stage M-ops stall via q_iss_is_mul.
   // an M-op or FP-op sitting at EX hasn't raised its unit's busy yet (that happens on
   // the next edge) -- both must assert exec_busy for their EX cycle, or a same-unit op
   // issued right behind them arrives at EX with the unit busy and evaporates (EX has
   // no hold; a lost deferred op leaves its checkpoint count stuck -> commit wedge).
   assign exec_busy     = munit_busy | (ex_v & ex_mulr & ~m_squash_now)
                                     | (fp_arith & ~fp_squash_now);
   assign div_done      = m_complete;
   assign div_done_ckpt = m_ck;

   // branch/jump resolution (EX, bypassed operands). The system op's redirect (trap/
   // xret/CSR-barrier) folds into the same per-lane redirect port: a lane is either a
   // branch or a system op, never both, and exec_bundle's oldest-select handles order.
   wire bu_redirect, bu_taken; wire [63:0] bu_target, bu_taken_tgt;
   branch_unit bu
     (.is_branch(ex_br), .is_jump(ex_jmp), .is_jalr(ex_jmp & ex_o2i), .is_rvc(ex_rvc),
      .br_func(ex_bf), .cmp_eq(cmp_eq), .cmp_lt(cmp_lt), .cmp_ltu(cmp_ltu),
      .pc(ex_pc), .imm(ex_imm), .agu_addr(agu_addr),
      .mis_taken(ex_mist), .mis_nt(ex_misn), .pred_npc(ex_pnpc),
      .redirect(bu_redirect), .target(bu_target),
      .taken_o(bu_taken), .taken_tgt(bu_taken_tgt));
   // predictor training: every genuinely-executing CTI (redirecting or not);
   // wrong-path ops killed by a squash in flight must not train. Call/return
   // class comes from the executed (RVC-expanded) instruction bytes here, OFF
   // the fetch critical path -- the predictor stores it in the BTB type.
   function islink(input [4:0] r); islink = (r == 5'd1) || (r == 5'd5); endfunction
   assign res_v     = ex_v & (ex_br | ex_jmp) & ~ex_squash;
   assign res_cbr   = ex_br;
`ifdef BR_TRACE
   always @(posedge clk) begin
      if (ex_v & ex_br & ~ex_squash)
         $display("[BR t=%0t] seq=%0d op1=%h op2=%h p1=%0d p2=%0d f3=%b taken=%b", $time, ex_sq, op1f, op2f, ex_p1, ex_p2, ex_bf, bu_taken);
      if (wb_valid) $display("[WB t=%0t] pr=%0d val=%h seq=%0d", $time, wb_pr, wb_val, wb_seq);
   end
`endif
   assign res_call  = ex_jmp & islink(ex_insn[11:7]);
   assign res_ret   = ex_jmp & ex_o2i & ~islink(ex_insn[11:7]) & islink(ex_insn[19:15]);
   assign res_taken = bu_taken;
   assign res_tgt   = bu_taken_tgt;
   // FENCE.I redirects to its fall-through (pc+4) once it issues -- and it issues only when
   // oldest (is_serialize), so every prior store has committed + drained to memory. The
   // refetch of pc+4 onward then sees the new instruction bytes (I/D coherence). Like the
   // sfence redirect, it's NOT a trap (rolls back to ckpt+1, keeping fence.i itself).
   wire fencei_redir = ex_v & ex_fencei;
   assign fencei_redir_o = fencei_redir;
   assign br_redirect = (ex_v & bu_redirect) | sys_redirect | fencei_redir;
   assign br_target   = sys_redirect ? sys_target
                      : fencei_redir ? (ex_pc + 64'd4) : bu_target;
   assign br_seq      = ex_sq;
   assign br_pc       = ex_pc;
   assign br_is_trap  = sys_redirect & csr_redir_is_trap;   // exception -> precise (TO ckpt)
   assign st_data     = op2f;
   // atomic drive: addr = agu_addr (rs1+0), data = st_data (rs2), size = ex_msz, sign = ex_msgn
   // ~ex_squash: task-#30 EX-squash gate (ALU/CSR/in-core-FP have it; the AMO drive was
   // MISSED). Ungated, a wrong-path atomic at EX in the same cycle an older rollback fires
   // still enters the LSU FSM (whose own squash guard checks ast!=A_IDLE and so misses the
   // latch cycle) and then executes fully: a memory WRITE with wrong-path address/data plus
   // a writeback to a possibly-reallocated physreg. Seen as the ubuntu generator-storm
   // memory corruption (grep/glib hash-size victims).
   assign ex_amo      = ex_v & ex_amor & ~ex_squash;
`ifdef SCDBG
   // count the would-have-been-dead atomics the gate now kills (mechanism evidence)
   always @(posedge clk) if (ex_v & ex_amor & ex_squash)
      $display("[AMOSQ %m t=%0t seq=%0d squash_seq=%0d]", $time, ex_sq, squash_seq);
`endif
   assign ex_amo_func = ex_amof;
   // An AMO with rd=x0 (amoor/amoadd.d x0,... -- atomic update discarding the result,
   // common in kernels) has ex_pdv=0 and no allocated dest, so ex_pd is the renamer's
   // phantom alloc_pr (the next free reg) which a younger op will really allocate. The
   // LSU AMO path writes rd back unconditionally (it carries no dest-valid bit), so
   // without this it clobbers that younger op's register. Map a non-writing AMO's dest
   // to phys0, which rf_shard reads as hardwired zero -> the writeback is inert. (task #30)
   assign ex_amo_pdst = ex_pdv ? ex_pd : {PBITS{1'b0}};

   // EX-stage LSU control (aligned with agu/st_data)
   assign ex_valid    = ex_v;
   assign ex_seq      = ex_sq;
   assign ex_ckpt     = ex_ck;
   assign ex_mem_idx  = ex_mi;
   assign ex_mem      = ex_memr;
   assign ex_store    = ex_str;
   assign ex_cbo      = ex_cbor;
   assign ex_cbo_zero = ex_cbozr;
   assign ex_cbo_keep = ex_cbokr;
   assign ex_fp       = (ex_insn[6:0]==7'b0000111);   // LOAD-FP (FLW/FLD) -> LSU NaN-boxes FLW
   assign ex_msize    = ex_msz;
   assign ex_msigned  = ex_msgn;
endmodule

`default_nettype wire
