`include "exec_pay.vh"
`default_nettype none

// The execute bundle: SHARDS two-stage execute slices (RR | EX) + the cross-shard
// writeback broadcast and the forwarding network. Each shard registers its result
// (flop after the ALU); that registered broadcast is (a) written into every shard's
// RF, (b) the scheduler's completion wake, and (c) the 1-ahead forwarding source. A
// one-cycle-delayed copy is the 2-ahead forwarding source; 3+-behind dependents read
// the RF (write-before-read). The LSU load writeback is muxed onto its owner lane.
module exec_bundle
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
    input  wire                    reset,
    input  wire [SHARDS-1:0]       iss_valid,
    input  wire [SHARDS*SEQW-1:0]  iss_seq,
    input  wire [SHARDS*PBITS-1:0] iss_pdst,
    input  wire [SHARDS-1:0]       iss_pdst_v,
    input  wire [SHARDS*PBITS-1:0] iss_ps1,
    input  wire [SHARDS*PBITS-1:0] iss_ps2,
    input  wire [SHARDS*PBITS-1:0] iss_ps3,
    input  wire [SHARDS*CBITS-1:0] iss_ckpt,
    input  wire [SHARDS*MIDXW-1:0] iss_mem_idx,
    input  wire [SHARDS*`PAYW-1:0] iss_pay,
    input  wire                    squash,
    input  wire [SEQW-1:0]         squash_seq,
    // per-shard M-unit status -> scheduler stall + commit_ctl completion
    output wire [SHARDS-1:0]       exec_busy,
    output wire [SHARDS-1:0]       div_done,
    output wire [SHARDS*CBITS-1:0] div_done_ckpt,
    output wire [SHARDS-1:0]       fp_done,
    output wire [SHARDS*CBITS-1:0] fp_done_ckpt,
    // LSU load writeback muxed onto its owner lane
    input  wire                    lsu_wb_v,
    input  wire [SBITS-1:0]        lsu_wb_owner,
    input  wire [PBITS-1:0]        lsu_wb_pr,
    input  wire [63:0]             lsu_wb_val,
    input  wire [SEQW-1:0]         lsu_wb_seq,     // seqno of the LSU writeback (cosim)
    input  wire                    lsu_fp_dirty,   // an FP-dest load (FLW/FLD) wrote back -> FS Dirty
    output wire [SHARDS-1:0]       wb_busy,        // per-shard ALU wb valid (-> LSU defer)
    // registered writeback broadcast out (RF write feed + scheduler wake)
    output wire [SHARDS-1:0]       wb_valid,
    output wire [SHARDS*PBITS-1:0] wb_pr,
    output wire [SHARDS*64-1:0]    wb_val,
    output wire [SHARDS*SEQW-1:0]  wb_seq,         // per-lane writeback seqno (cosim capture)
    // EX-stage LSU drive (aligned with agu/st_data)
    output wire [SHARDS-1:0]       ex_valid,
    output wire [SHARDS*SEQW-1:0]  ex_seq,
    output wire [SHARDS*CBITS-1:0] ex_ckpt,
    output wire [SHARDS*MIDXW-1:0] ex_mem_idx,
    output wire [SHARDS-1:0]       ex_mem,
    output wire [SHARDS-1:0]       ex_store,
    output wire [SHARDS-1:0]       ex_cbo,
    output wire [SHARDS-1:0]       ex_cbo_zero,
    output wire [SHARDS-1:0]       ex_cbo_keep,
    output wire [SHARDS-1:0]       ex_fp,
    output wire [SHARDS*2-1:0]     ex_msize,
    output wire [SHARDS-1:0]       ex_msigned,
    output wire [SHARDS*64-1:0]    agu_addr,
    output wire [SHARDS*64-1:0]    st_data,
    // ---- per-shard atomic (A ext) drive ----
    output wire [SHARDS-1:0]       ex_amo,
    output wire [SHARDS*5-1:0]     ex_amo_func,
    output wire [SHARDS*PBITS-1:0] ex_amo_pdst,
    // per-checkpoint pred_npc: written at dispatch (one bundle == one ckpt == at
    // most one CTI), read at EX by the op's ckpt tag -- the mispredict reference
    // for branch_unit without widening the IQ payload.
    input  wire                    disp_v,
    input  wire [CBITS-1:0]        disp_ckpt,
    input  wire [63:0]             disp_pnpc,
    // oldest genuinely-resolved CTI this cycle -> predictor training/repair
    output reg                     res_v,
    output reg                     res_cbr,
    output reg                     res_call,
    output reg                     res_ret,
    output reg                     res_taken,
    output reg  [CBITS-1:0]        res_ckpt,
    output reg  [63:0]             res_tgt,
    output reg                     res_mispred,        // this resolve IS the redirecting op
    // oldest mispredicting branch this cycle -> redirect
    output reg                     redirect,
    output reg  [63:0]             redirect_target,
    output reg  [SEQW-1:0]         redirect_seq,
    output reg  [CBITS-1:0]        redirect_ckpt,
    output reg                     redirect_is_trap,   // exception -> roll back TO ckpt (not +1)
    output wire                    ifence,             // a FENCE.I is redirecting this cycle (I$ flush + drain order)
    // ---- translation context passed out to the iMMU/dMMU ----
    output wire [63:0]             mmu_satp,
    output wire [1:0]              mmu_priv,
    output wire [1:0]              mmu_dpriv,
    output wire                    mmu_sum,
    output wire                    mmu_mxr,
    output wire                    fs_off,         // mstatus.FS==Off -> FP ops trap illegal
    output wire                    mmu_flush,
    // ---- external trap injection (page faults) + the resulting redirect target ----
    input  wire                    xtrap_v,
    input  wire                    xtrap_intr,
    input  wire [3:0]              xtrap_cause,
    input  wire [63:0]             xtrap_epc,
    input  wire [63:0]             xtrap_tval,
    input  wire [11:0]             hw_ip,            // hardware interrupt-pending (CLINT/PLIC)
    input  wire [63:0]             mtime,            // free-running CLINT time (Sstc stimecmp compare)
    input  wire [2:0]              retire_cnt,       // # instructions retiring this cycle (-> minstret)
    input  wire [6:0]              hpm_ev,           // Zihpm event pulses (ld/st/redir/dc/ic) -> csr_file
    output wire                    irq_v,            // an interrupt is deliverable now
    output wire [3:0]              irq_cause,
    output wire                    csr_redir_v,      // csr_file redirect this cycle (trap/xret)
    output wire [63:0]             csr_redir_tgt);

   wire [SHARDS-1:0]       wbv;          // per-shard registered ALU/M writeback valid
   wire [SHARDS*PBITS-1:0] wbp;
   wire [SHARDS*64-1:0]    wbd;
   wire [SHARDS*SEQW-1:0]  wbsq;         // per-shard registered ALU/M writeback seqno (cosim)
   wire [SHARDS-1:0]       brd;
   wire [SHARDS-1:0]       fnci;         // per-shard FENCE.I redirect
   assign ifence = |fnci;
   wire [SHARDS*64-1:0]    brt;
   wire [SHARDS*64-1:0]    brp;          // per-shard redirecting-op PC (debug control-flow trace)
   reg  [63:0]             redirect_src_pc;
   wire [SHARDS*SEQW-1:0]  brs;
   wire [SHARDS*CBITS-1:0] brc;          // EX-stage ckpt of each shard (for redirect)
   wire [SHARDS-1:0]       brtr;         // per-shard "redirect is a trap"
   wire [SHARDS-1:0]       rsv;          // per-shard resolved-CTI (predictor training)
   wire [SHARDS-1:0]       rscb, rstk, rscl, rsrt;
   wire [SHARDS*64-1:0]    rstg;

   // dispatch-time pred_npc, indexed by checkpoint (written >=2 cycles before any
   // op of that bundle reaches EX; a ckpt is only reused after its span commits).
   reg [63:0] pnpc [0:(1<<CBITS)-1];
   always @(posedge clk) if (disp_v) pnpc[disp_ckpt] <= disp_pnpc;

   // effective per-lane registered writeback = ALU/M result, else the LSU load.
   wire [SHARDS-1:0]       ewbv;
   wire [SHARDS*PBITS-1:0] ewbp;
   wire [SHARDS*64-1:0]    ewbd;
   wire [SHARDS*SEQW-1:0]  ewbsq;
   // wb_busy to the LSU = the NEXT-cycle writeback per lane (the LSU's registered load
   // result lands a cycle after it selects, so it reserves the lane one cycle ahead).
   wire [SHARDS-1:0]       wbn;
   assign wb_busy = wbn;

   // 2-ahead forwarding source = the registered ALU/M results (wbv/wbp/wbd, NOT the
   // LSU-merged ewb) delayed one cycle. Loads are not forwarded.
   reg  [SHARDS-1:0]       fw2v;
   reg  [SHARDS*PBITS-1:0] fw2p;
   reg  [SHARDS*64-1:0]    fw2d;
   initial fw2v = {SHARDS{1'b0}};
   always @(posedge clk) begin fw2v <= wbv; fw2p <= wbp; fw2d <= wbd; end

   // ---- CSR file (shared; one system op executes at a time -> single port) ----
   wire [63:0]          csr_rdata, csr_redir_target;
   wire                 csr_redir_valid, csr_redir_is_trap, csr_illegal;
   wire [SHARDS-1:0]    csr_req_v, csr_req_is_csr;
   wire [SHARDS*3-1:0]  csr_req_func;
   wire [SHARDS*12-1:0] csr_req_addr, csr_rd_addr;
   wire [SHARDS*64-1:0] csr_req_src, csr_req_pc;
   wire [SHARDS*5-1:0]  fp_fflags_sh;        // per-shard FP flags (valid with fp_flags_we_sh)
   wire [SHARDS-1:0]    fp_flags_we_sh;
   wire [SHARDS-1:0]    fp_dirty_sh;
   wire [2:0]           csr_frm;             // fcsr.frm (from u_csr) -> shards
   // OR-reduce the flags of every shard raising FP flags this cycle into one accumulate pulse
   reg  [4:0] fp_fflags_or; integer fk;
   always @* begin
      fp_fflags_or = 5'd0;
      for (fk = 0; fk < SHARDS; fk = fk + 1)
         if (fp_flags_we_sh[fk]) fp_fflags_or = fp_fflags_or | fp_fflags_sh[fk*5 +: 5];
   end
   wire fp_fflags_we = |fp_flags_we_sh;
   // mstatus.FS -> Dirty: any shard wrote FP state (arith/compare/in-core move) OR an FP-dest
   // load wrote back in the LSU. Broader than fp_fflags_we (flag-producing ops only).
   wire fp_dirty = (|fp_dirty_sh) | lsu_fp_dirty;

   genvar i;
   generate for (i = 0; i < SHARDS; i = i + 1) begin : lane
      wire [`PAYW-1:0] p = iss_pay[i*`PAYW +: `PAYW];
      exec_shard #(.SHARDS(SHARDS), .SBITS(SBITS), .NPHYS(NPHYS), .PBITS(PBITS),
                   .POOL(POOL), .IDXB(IDXB), .SEQW(SEQW), .CBITS(CBITS), .MIDXW(MIDXW)) sh
        (.clk(clk),
         .iss_valid(iss_valid[i]), .iss_seq(iss_seq[i*SEQW +: SEQW]),
         .iss_pdst(iss_pdst[i*PBITS +: PBITS]), .iss_pdst_v(iss_pdst_v[i]),
         .iss_ps1(iss_ps1[i*PBITS +: PBITS]), .iss_ps2(iss_ps2[i*PBITS +: PBITS]),
         .iss_ps3(iss_ps3[i*PBITS +: PBITS]),
         .iss_ckpt(iss_ckpt[i*CBITS +: CBITS]), .iss_mem_idx(iss_mem_idx[i*MIDXW +: MIDXW]),
         .iss_insn(p[`PAY_INSN]),
         .squash(squash), .squash_seq(squash_seq),
         .alu_op(p[`PAY_ALUOP]), .alu_w(p[`PAY_W]), .alu_uw(p[`PAY_UW]),
         .op1_sel(p[`PAY_O1S]), .op2_imm(p[`PAY_O2I]), .res_link(p[`PAY_LINK]),
         .is_rvc(p[`PAY_RVC]), .is_mem(p[`PAY_MEM]), .is_store(p[`PAY_STORE]),
         .mem_size(p[`PAY_MSIZE]), .mem_signed(p[`PAY_MSGN]),
         .is_branch(p[`PAY_BR]), .is_jump(p[`PAY_JMP]), .is_mul(p[`PAY_MUL]), .br_func(p[`PAY_BRFUNC]),
         .is_csr(p[`PAY_CSR]), .csr_func(p[`PAY_CSRF]), .is_serialize(p[`PAY_SER]),
         .is_fencei(p[`PAY_FENCEI]),
         .is_amo(p[`PAY_AMO]), .amo_func(p[`PAY_AMOF]),
         .is_cbo(p[`PAY_CBO]), .cbo_zero(p[`PAY_CBOZ]), .cbo_keep(p[`PAY_CBOK]),
         .ex_cbo(ex_cbo[i]), .ex_cbo_zero(ex_cbo_zero[i]), .ex_cbo_keep(ex_cbo_keep[i]),
         .ex_amo(ex_amo[i]), .ex_amo_func(ex_amo_func[i*5 +: 5]),
         .ex_amo_pdst(ex_amo_pdst[i*PBITS +: PBITS]),
         .imm(p[`PAY_IMM]), .pc(p[`PAY_PC]),
         .csr_rdata(csr_rdata), .csr_redir_target(csr_redir_target),
         .csr_redir_valid(csr_redir_valid), .csr_redir_is_trap(csr_redir_is_trap),
         .csr_illegal(csr_illegal),
         .csr_req_v(csr_req_v[i]), .csr_req_is_csr(csr_req_is_csr[i]),
         .csr_req_func(csr_req_func[i*3 +: 3]), .csr_req_addr(csr_req_addr[i*12 +: 12]),
         .csr_req_src(csr_req_src[i*64 +: 64]), .csr_req_pc(csr_req_pc[i*64 +: 64]),
         .csr_rd_addr(csr_rd_addr[i*12 +: 12]),
         .wb_valid_in(ewbv), .wb_pr_in(ewbp), .wb_val_in(ewbd),      // RF write (incl. load)
         .byp_valid(wbv), .byp_pr(wbp), .byp_val(wbd),               // 1-ahead forward (ALU/M)
         .fw2_valid(fw2v), .fw2_pr(fw2p), .fw2_val(fw2d),            // 2-ahead forward (ALU/M)
         .wb_valid(wbv[i]), .wb_pr(wbp[i*PBITS +: PBITS]), .wb_val(wbd[i*64 +: 64]),
         .wb_seq(wbsq[i*SEQW +: SEQW]),
         .pred_npc(pnpc[iss_ckpt[i*CBITS +: CBITS]]),   // RR-time read (compares flop into EX)
         .br_redirect(brd[i]), .br_target(brt[i*64 +: 64]), .br_pc(brp[i*64 +: 64]),
         .fencei_redir_o(fnci[i]), .br_seq(brs[i*SEQW +: SEQW]),
         .br_is_trap(brtr[i]),
         .res_v(rsv[i]), .res_cbr(rscb[i]), .res_call(rscl[i]), .res_ret(rsrt[i]),
         .res_taken(rstk[i]), .res_tgt(rstg[i*64 +: 64]),
         .ex_valid(ex_valid[i]), .ex_seq(ex_seq[i*SEQW +: SEQW]), .ex_ckpt(brc[i*CBITS +: CBITS]),
         .ex_mem_idx(ex_mem_idx[i*MIDXW +: MIDXW]), .ex_mem(ex_mem[i]), .ex_store(ex_store[i]),
         .ex_fp(ex_fp[i]), .ex_msize(ex_msize[i*2 +: 2]), .ex_msigned(ex_msigned[i]),
         .agu_addr(agu_addr[i*64 +: 64]), .st_data(st_data[i*64 +: 64]),
         .exec_busy(exec_busy[i]), .div_done(div_done[i]),
         .div_done_ckpt(div_done_ckpt[i*CBITS +: CBITS]),
         .fp_done(fp_done[i]), .fp_done_ckpt(fp_done_ckpt[i*CBITS +: CBITS]),
         .fp_flags_we(fp_flags_we_sh[i]), .fp_flags(fp_fflags_sh[i*5 +: 5]),
         .fp_dirty(fp_dirty_sh[i]),
         .i_frm(csr_frm), .i_fs_off(fs_off), .wb_next(wbn[i]));
   end endgenerate

   // pick the single active system op (gated to oldest -> at most one csr_req_v)
   reg               sv, s_iscsr;
   reg  [2:0]        s_func;
   reg  [11:0]       s_addr, s_rdaddr;
   reg  [63:0]       s_src, s_pc;
   integer cn;
   always @* begin
      sv=1'b0; s_iscsr=1'b0; s_func=3'd0; s_addr=12'd0; s_rdaddr=12'd0; s_src=64'd0; s_pc=64'd0;
      for (cn = 0; cn < SHARDS; cn = cn + 1) if (csr_req_v[cn]) begin
         sv=1'b1; s_iscsr=csr_req_is_csr[cn]; s_func=csr_req_func[cn*3 +: 3];
         s_addr=csr_req_addr[cn*12 +: 12]; s_rdaddr=csr_rd_addr[cn*12 +: 12];
         s_src=csr_req_src[cn*64 +: 64]; s_pc=csr_req_pc[cn*64 +: 64];
      end
   end
   csr_file u_csr
     (.clk(clk), .reset(reset),            // squash must NOT reset CSR state (only reset does)
      .raddr(s_rdaddr), .rdata(csr_rdata), .redir_target(csr_redir_target),
      .redir_valid(csr_redir_valid), .redir_is_trap(csr_redir_is_trap), .csr_illegal(csr_illegal),
      .o_satp(mmu_satp), .o_priv(mmu_priv), .o_dpriv(mmu_dpriv),
      .o_sum(mmu_sum), .o_mxr(mmu_mxr), .o_tlb_flush(mmu_flush),
      .o_frm(csr_frm), .o_fs_off(fs_off), .fp_fflags_we(fp_fflags_we), .fp_fflags(fp_fflags_or),
      .fp_dirty(fp_dirty),
      .xtrap_v(xtrap_v), .xtrap_intr(xtrap_intr), .xtrap_cause(xtrap_cause),
      .xtrap_epc(xtrap_epc), .xtrap_tval(xtrap_tval),
      .hw_ip(hw_ip), .mtime(mtime), .retire_cnt(retire_cnt), .hpm_ev(hpm_ev),
      .irq_v(irq_v), .irq_cause(irq_cause),
      .upd_valid(sv), .upd_is_csr(s_iscsr), .upd_func(s_func), .upd_addr(s_addr),
      .upd_src(s_src), .upd_pc(s_pc));

   assign csr_redir_v   = csr_redir_valid;
   assign csr_redir_tgt = csr_redir_target;

   assign ex_ckpt = brc;

   genvar k;
   generate for (k = 0; k < SHARDS; k = k + 1) begin : wbmux
      wire ld_here = lsu_wb_v && (lsu_wb_owner == k[SBITS-1:0]);
      assign ewbv[k]                = wbv[k] | ld_here;
      assign ewbp[k*PBITS +: PBITS] = wbv[k] ? wbp[k*PBITS +: PBITS] : lsu_wb_pr;
      assign ewbd[k*64 +: 64]       = wbv[k] ? wbd[k*64 +: 64]       : lsu_wb_val;
      assign ewbsq[k*SEQW +: SEQW]  = wbv[k] ? wbsq[k*SEQW +: SEQW]  : lsu_wb_seq;
   end endgenerate

   assign wb_valid = ewbv;
   assign wb_pr    = ewbp;
   assign wb_val   = ewbd;
   assign wb_seq   = ewbsq;

   // oldest mispredicting branch (EX stage) -> redirect; its checkpoint = EX-stage ckpt
   integer j;
   always @* begin
      redirect = 1'b0; redirect_target = 64'd0; redirect_seq = {SEQW{1'b0}};
      redirect_ckpt = {CBITS{1'b0}}; redirect_is_trap = 1'b0; redirect_src_pc = 64'd0;
      for (j = 0; j < SHARDS; j = j + 1)
         if (brd[j] && (!redirect || $signed(brs[j*SEQW +: SEQW] - redirect_seq) < 0)) begin
            redirect         = 1'b1;
            redirect_target  = brt[j*64 +: 64];
            redirect_seq     = brs[j*SEQW +: SEQW];
            redirect_ckpt    = brc[j*CBITS +: CBITS];
            redirect_is_trap = brtr[j];
            redirect_src_pc  = brp[j*64 +: 64];
         end
   end

   // oldest resolved CTI -> one predictor training write per cycle. Anything older
   // than the redirect (or the redirecting branch itself) is a genuine resolution;
   // a resolve YOUNGER than a same-cycle redirect (e.g. an older lane's ecall) is
   // wrong-path and suppressed. Losing a younger-of-two-resolves is only a missed
   // hint update.
   integer rj;
   reg [SEQW-1:0] res_seq;
   always @* begin
      res_v = 1'b0; res_cbr = 1'b0; res_call = 1'b0; res_ret = 1'b0;
      res_taken = 1'b0; res_mispred = 1'b0;
      res_ckpt = {CBITS{1'b0}}; res_tgt = 64'd0; res_seq = {SEQW{1'b0}};
      for (rj = 0; rj < SHARDS; rj = rj + 1)
         if (rsv[rj] && (!res_v || $signed(brs[rj*SEQW +: SEQW] - res_seq) < 0)) begin
            res_v     = 1'b1;
            res_cbr   = rscb[rj];
            res_call  = rscl[rj];
            res_ret   = rsrt[rj];
            res_taken = rstk[rj];
            res_ckpt  = brc[rj*CBITS +: CBITS];
            res_tgt   = rstg[rj*64 +: 64];
            res_seq   = brs[rj*SEQW +: SEQW];
         end
      if (res_v && redirect) begin
         if ($signed(redirect_seq - res_seq) < 0) res_v = 1'b0;   // wrong-path resolve
         else res_mispred = (res_seq == redirect_seq);            // it IS the redirect
      end
   end
endmodule

`default_nettype wire
