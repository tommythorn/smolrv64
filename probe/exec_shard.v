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
    input  wire [CBITS-1:0]        iss_ckpt,
    input  wire [MIDXW-1:0]        iss_mem_idx,
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
    input  wire [2:0]              br_func,
    input  wire [63:0]             imm,
    input  wire [63:0]             pc,
    // CSR file (system op executes here when oldest -> precise)
    input  wire [63:0]             csr_rdata,    // old value at imm[11:0]
    input  wire [63:0]             csr_mtvec,    // trap target
    input  wire [63:0]             csr_mepc,     // xret target
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
    // ---- EX: branch/jump resolution ----
    output wire                    br_redirect,
    output wire [63:0]             br_target,
    output wire [SEQW-1:0]         br_seq,
    // ---- EX: LSU drive (aligned with agu/st_data) ----
    output wire                    ex_valid,
    output wire [SEQW-1:0]         ex_seq,
    output wire [CBITS-1:0]        ex_ckpt,
    output wire [MIDXW-1:0]        ex_mem_idx,
    output wire                    ex_mem,
    output wire                    ex_store,
    output wire [1:0]              ex_msize,
    output wire                    ex_msigned,
    output wire [63:0]             agu_addr,
    output wire [63:0]             st_data,
    // ---- M-unit status ----
    output wire                    exec_busy,
    output wire                    div_done,
    output wire [CBITS-1:0]        div_done_ckpt,
    // next-cycle writeback on this shard's lane (for the LSU's lane reservation)
    output wire                    wb_next);

   function automatic older;          // a strictly older than b (wrap-safe)
      input [SEQW-1:0] a, bb; older = ($signed(a - bb) < 0);
   endfunction

   // ============================== RR stage ==============================
   wire [63:0] rf_rs1, rf_rs2;
   rf_shard #(.SHARDS(SHARDS), .SBITS(SBITS), .NPHYS(NPHYS), .PBITS(PBITS),
              .POOL(POOL), .IDXB(IDXB)) rf
     (.clk(clk), .wr_valid(wb_valid_in), .wr_pr(wb_pr_in), .wr_val(wb_val_in),
      .ra1(iss_ps1), .ra2(iss_ps2), .rd1(rf_rs1), .rd2(rf_rs2));

   // squash an op that becomes wrong-path the cycle it is flopped into EX
   wire rr_kill = squash & older(squash_seq, iss_seq);

   reg              ex_v, ex_pdv, ex_w, ex_uw, ex_o2i, ex_link, ex_rvc, ex_memr,
                    ex_str, ex_msgn, ex_br, ex_jmp, ex_mulr, ex_csr, ex_ser;
   reg  [PBITS-1:0] ex_pd, ex_p1, ex_p2;
   reg  [SEQW-1:0]  ex_sq;
   reg  [CBITS-1:0] ex_ck;
   reg  [MIDXW-1:0] ex_mi;
   reg  [63:0]      ex_r1, ex_r2, ex_imm, ex_pc;
   reg  [5:0]       ex_aop;
   reg  [1:0]       ex_o1s, ex_msz;
   reg  [2:0]       ex_bf, ex_csrf;
   always @(posedge clk) begin
      ex_v   <= iss_valid & ~rr_kill;
      ex_pdv <= iss_pdst_v; ex_pd <= iss_pdst; ex_p1 <= iss_ps1; ex_p2 <= iss_ps2;
      ex_sq  <= iss_seq; ex_ck <= iss_ckpt; ex_mi <= iss_mem_idx;
      ex_r1  <= rf_rs1; ex_r2 <= rf_rs2; ex_imm <= imm; ex_pc <= pc;
      ex_aop <= alu_op; ex_w <= alu_w; ex_uw <= alu_uw; ex_o1s <= op1_sel;
      ex_o2i <= op2_imm; ex_link <= res_link; ex_rvc <= is_rvc;
      ex_memr <= is_mem; ex_str <= is_store; ex_msz <= mem_size; ex_msgn <= mem_signed;
      ex_br  <= is_branch; ex_jmp <= is_jump; ex_mulr <= is_mul; ex_bf <= br_func;
      ex_csr <= is_csr; ex_csrf <= csr_func; ex_ser <= is_serialize;
   end

   // ============================== EX stage ==============================
   // operand forwarding: 1-ahead = wb_*_in (this cycle's registered writebacks),
   // 2-ahead = fw2_* (those delayed one more cycle). A physreg is written once, so
   // a tag matches at most one source; prefer the newer (1-ahead).
   reg [63:0] op1f, op2f;
   integer s;
   always @* begin
      op1f = ex_r1; op2f = ex_r2;
      for (s = 0; s < SHARDS; s = s + 1) begin
         if (fw2_valid[s] && fw2_pr[s*PBITS +: PBITS] == ex_p1) op1f = fw2_val[s*64 +: 64];
         if (fw2_valid[s] && fw2_pr[s*PBITS +: PBITS] == ex_p2) op2f = fw2_val[s*64 +: 64];
         if (byp_valid[s] && byp_pr[s*PBITS +: PBITS] == ex_p1) op1f = byp_val[s*64 +: 64];
         if (byp_valid[s] && byp_pr[s*PBITS +: PBITS] == ex_p2) op2f = byp_val[s*64 +: 64];
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
   wire        munit_busy = mbusy | dbusy;
   reg  [PBITS-1:0] m_pdst;  reg [SEQW-1:0] m_seq;  reg [CBITS-1:0] m_ck;
   wire m_squash_now = squash & older(squash_seq, ex_sq);
   wire m_start = ex_v & ex_mulr & ~munit_busy & ~m_squash_now;
   wire m_abort = munit_busy & squash & older(squash_seq, m_seq);
   mul3 mu (.clk(clk), .reset(1'b0), .start(m_start & mul_op), .abort(m_abort),
            .rs1(op1f), .rs2(op2f), .f3(ex_bf), .is_w(ex_w),
            .busy(mbusy), .done(mdone), .result(mres));
   divider dv (.clk(clk), .reset(1'b0), .start(m_start & div_op), .abort(m_abort),
               .rs1(op1f), .rs2(op2f), .f3(ex_bf), .is_w(ex_w),
               .busy(dbusy), .done(ddone), .result(dres));
   always @(posedge clk) if (m_start) begin m_pdst <= ex_pd; m_seq <= ex_sq; m_ck <= ex_ck; end
   wire        m_complete = (mdone | ddone) & ~m_abort;
   wire [63:0] m_res = mdone ? mres : dres;

   // result flop = writeback. ALU/link results, plus M completions (mux'd in; only one
   // M-op per shard at a time -> no collision). mem ops complete via the LSU.
   // a CSR op writes rd = the OLD csr value (read combinationally from csr_file).
   wire        csr_wb    = ex_v & ex_csr & ex_pdv;
   wire        ex_alu_wb = ex_v & ex_pdv & ~ex_memr & ~ex_mulr & ~ex_csr;
   assign      wb_next   = ex_alu_wb | m_complete | csr_wb;   // what this lane writes back next cycle
   always @(posedge clk) begin
      wb_valid <= ex_alu_wb | m_complete | csr_wb;
      wb_pr    <= m_complete ? m_pdst : ex_pd;
      wb_val   <= m_complete ? m_res : (csr_wb ? csr_rdata : result);
   end

   // ---- CSR/system unit: read addr + update request + redirect ----
   assign csr_rd_addr    = ex_imm[11:0];                       // combinational read
   wire [63:0] csr_src   = ex_csrf[2] ? {59'b0, ex_imm[16:12]} : op1f;  // zimm | rs1
   assign csr_req_v      = ex_v & ex_ser;                      // oldest -> non-speculative
   assign csr_req_is_csr = ex_csr;
   assign csr_req_func   = ex_csrf;
   assign csr_req_addr   = ex_imm[11:0];
   assign csr_req_src    = csr_src;
   assign csr_req_pc     = ex_pc;

   wire [63:0] sys_next  = ex_pc + (ex_rvc ? 64'd2 : 64'd4);
   wire is_ecall  = ex_ser & ~ex_csr & (ex_imm[11:0] == 12'h000);
   wire is_ebreak = ex_ser & ~ex_csr & (ex_imm[11:0] == 12'h001);
   wire is_mret   = ex_ser & ~ex_csr & (ex_imm[11:0] == 12'h302);
   wire [63:0] sys_target = (is_ecall | is_ebreak) ? csr_mtvec
                          :  is_mret               ? csr_mepc
                          :                          sys_next;  // csr / wfi / sret / sfence
   wire sys_redirect = ex_v & ex_ser;

   // busy = unit running OR an M-op in EX about to start it (so no second M-op is
   // selected in the gap before munit_busy rises). RR-stage M-ops stall via q_iss_is_mul.
   assign exec_busy     = munit_busy | (ex_v & ex_mulr & ~m_squash_now);
   assign div_done      = m_complete;
   assign div_done_ckpt = m_ck;

   // branch/jump resolution (EX, bypassed operands). The system op's redirect (trap/
   // xret/CSR-barrier) folds into the same per-lane redirect port: a lane is either a
   // branch or a system op, never both, and exec_bundle's oldest-select handles order.
   wire bu_redirect; wire [63:0] bu_target;
   branch_unit bu
     (.is_branch(ex_br), .is_jump(ex_jmp), .is_jalr(ex_jmp & ex_o2i),
      .br_func(ex_bf), .cmp_eq(cmp_eq), .cmp_lt(cmp_lt), .cmp_ltu(cmp_ltu),
      .pc(ex_pc), .imm(ex_imm), .agu_addr(agu_addr),
      .redirect(bu_redirect), .target(bu_target));
   assign br_redirect = (ex_v & bu_redirect) | sys_redirect;
   assign br_target   = sys_redirect ? sys_target : bu_target;
   assign br_seq      = ex_sq;
   assign st_data     = op2f;

   // EX-stage LSU control (aligned with agu/st_data)
   assign ex_valid    = ex_v;
   assign ex_seq      = ex_sq;
   assign ex_ckpt     = ex_ck;
   assign ex_mem_idx  = ex_mi;
   assign ex_mem      = ex_memr;
   assign ex_store    = ex_str;
   assign ex_msize    = ex_msz;
   assign ex_msigned  = ex_msgn;
endmodule

`default_nettype wire
