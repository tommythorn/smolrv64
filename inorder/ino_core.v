`include "va_codec.vh"
`default_nettype none

// In-order pipelined RVA22S64 core: F | X | M.
//
//   F : PC -> iMMU -> I$ window -> aligner -> RVC expand -> decode   (ino_frontend)
//   X : regfile read + M-bypass -> exec_alu (ALU/AGU/compare) -> branch_unit
//   M : LSU (dMMU + D$) | mul/div | csr_file | trap | redirect | regfile write
//
// M IS THE ONLY COMMIT POINT. Every architectural side effect happens there and
// nowhere else, which is what makes traps precise for free: when an instruction is
// in M nothing older can still fault (older ones have retired) and nothing younger
// has changed anything (X and F hold no state). All the OoO core's recovery
// machinery -- checkpoints, replay-to-solo fault delivery, the illegal/data-fault
// latches, the AMO dispatch gap, rollback priority -- collapses into one mux here.
//
// STALLS. Anything multi-cycle (D$ access or miss, page-table walk, divide,
// multiply) freezes the whole pipe: M holds its instruction until `m_done`, X holds
// because it cannot hand off, F holds because `accept` is low. One bypass level
// (M -> X) covers every RAW hazard, because an instruction two ahead has already
// written the regfile.
//
// SERIALIZATION. A CSR/system/fence op lets nothing follow it into X until it has
// left M, so CSR values, privilege, satp and mstatus.FS are never read stale.
`ifndef INO_HW
 `define INO_HW 2                 // fetch window halfwords (one 32-bit instruction)
`endif
module ino_core
  #(parameter PCW  = 64,
    parameter SEQW = 8,
    parameter HW   = `INO_HW,
    parameter AW   = 64,
    parameter CBITS = 2,
    parameter NCHK  = 4,
    parameter [PCW-1:0] RESET_PC = 0)
   (input  wire                    clk,
    input  wire                    reset,
    // ---- instruction memory (combinational window at the translated PA) ----
    output wire [PCW-1:0]          imem_addr,
    input  wire [HW*16-1:0]        imem_data,
    input  wire [$clog2(HW+2)-1:0] imem_avail,
    // ---- platform interrupt lines + time ----
    input  wire [11:0]             hw_ip,
    input  wire [63:0]             mtime,
    input  wire                    hpm_dc_access, hpm_dc_miss, hpm_ic_access, hpm_ic_miss,
    // ---- data memory port ----
    output wire [AW-1:0]           dmem_raddr,
    output wire                    dmem_ren,
    output wire                    dmem_runcached,
    input  wire [63:0]             dmem_rdata,
    input  wire                    dmem_rvalid,
    output wire                    dmem_wen,
    output wire [AW-1:0]           dmem_waddr,
    output wire [63:0]             dmem_wdata,
    output wire [7:0]              dmem_wmask,
    output wire                    dmem_wuncached,
    output wire                    dmem_cbo,
    output wire                    dmem_cbo_zero,
    output wire                    dmem_cbo_keep,
    input  wire                    dmem_wready,
    output wire                    dmem_idle,          // no memory op in flight (fence.i drain)
    output wire                    ifence,             // FENCE.I this cycle -> flush D$/I$
    // ---- page-table-walker ports (instruction side, data side) ----
    output wire [55:0]             ptw_addr,
    output wire                    ptw_read,
    input  wire [63:0]             ptw_rdata,
    input  wire                    ptw_rvalid,
    output wire [55:0]             dptw_addr,
    output wire                    dptw_read,
    input  wire [63:0]             dptw_rdata,
    input  wire                    dptw_rvalid,
    // ---- observation ----
    output wire                    retire,             // an instruction retired this cycle
    output wire [PCW-1:0]          retire_pc,
    output wire [31:0]             retire_insn,
    output wire                    redirect,
    output wire [PCW-1:0]          redirect_target);

   // =========================================================== stage F
   wire                     d_valid, d_rvc, d_rd_v, d_rs1_v, d_rs2_v, d_rs3_v;
   wire [PCW-1:0]           d_pc, d_pred_npc, d_fault_tval;
   wire [31:0]              d_insn;
   wire [SEQW-1:0]          d_seq;
   wire [CBITS-1:0]         d_ckpt;
   wire [5:0]               d_rd, d_rs1, d_rs2, d_rs3;
   wire [63:0]              d_imm;
   wire [5:0]               d_alu_op;
   wire                     d_alu_w, d_alu_uw, d_op2_imm, d_res_link;
   wire [1:0]               d_op1_sel, d_mem_size;
   wire                     d_is_mem, d_is_store, d_mem_signed;
   wire                     d_is_branch, d_is_jump, d_is_jalr;
   wire [2:0]               d_br_func, d_csr_func;
   wire                     d_is_mul, d_is_csr, d_is_serialize, d_is_amo;
   wire [4:0]               d_amo_func;
   wire                     d_is_fp, d_is_fencei, d_is_cbo, d_cbo_zero, d_cbo_keep;
   wire                     d_illegal, d_mis_taken, d_mis_nt, d_fault;
   wire [3:0]               d_fault_cause;
   wire [SEQW-1:0]          fe_cur_seq;

   wire                     accept;
   wire                     m_done, m_advance;   // M completed / M can take a new op
   wire                     irq_inject;
   wire [PCW-1:0]           imem_va;
   wire [55:0]              immu_pa;
   wire                     immu_ready, immu_fault;
   wire [3:0]               immu_cause;
   wire [$clog2(HW+2)-1:0]  imem_avail_g = (immu_ready & ~immu_fault) ? imem_avail
                                                                      : {$clog2(HW+2){1'b0}};
   // resolve/training port (driven from M, below)
   wire                     res_v, res_cbr, res_call, res_ret, res_taken, res_rep;
   wire [CBITS-1:0]         res_ckpt;
   wire [PCW-1:0]           res_tgt;
   wire                     redirect_is_trap;
   wire [CBITS-1:0]         redirect_ckpt;
   wire [SEQW-1:0]          redirect_seq;

   // ---- FMAX: the predictor's training bundle lands one cycle later --------------
   // res_v is gated by m_done, which depends on lsu_done -- so the D$/dTLB hit path
   // reached the BTB/ycorr arrays combinationally. Updates are hints, so the extra
   // cycle costs no correctness and no bubble; kept in step with redirect_q so u_bp
   // sees resolve and rollback in their original relative order.
   reg                      res_v_q, res_cbr_q, res_call_q, res_ret_q;
   reg                      res_taken_q, res_rep_q;
   reg [CBITS-1:0]          res_ckpt_q;
   reg [PCW-1:0]            res_tgt_q;
   initial begin res_v_q = 1'b0; res_rep_q = 1'b0; end
   always @(posedge clk) begin
      if (reset) begin res_v_q <= 1'b0; res_rep_q <= 1'b0; end
      else       begin res_v_q <= res_v; res_rep_q <= res_rep; end
      res_cbr_q   <= res_cbr;
      res_call_q  <= res_call;
      res_ret_q   <= res_ret;
      res_taken_q <= res_taken;
      res_ckpt_q  <= res_ckpt;
      res_tgt_q   <= res_tgt;
   end

   // ---- FMAX: the frontend sees the redirect one cycle late ----------------------
   // Cuts the redirect -> iMMU-translate -> predictor-update cone, which was the whole
   // critical path. redirect_q doubles as the shadow flag: the cycle it is high is
   // exactly the cycle in which the frontend is squashing the extra wrong-path bundle
   // it fetched, and in which M must refuse that bundle.
   reg                      redirect_q, redirect_is_trap_q;
   reg [PCW-1:0]            redirect_target_q;
   reg [CBITS-1:0]          redirect_ckpt_q;
   reg [SEQW-1:0]           redirect_seq_q;
   initial redirect_q = 1'b0;
   always @(posedge clk) begin
      if (reset) redirect_q <= 1'b0;
      else       redirect_q <= redirect;
      redirect_target_q  <= redirect_target;
      redirect_is_trap_q <= redirect_is_trap;
      redirect_ckpt_q    <= redirect_ckpt;
      redirect_seq_q     <= redirect_seq;
   end

   ino_frontend #(.PCW(PCW), .SEQW(SEQW), .HW(HW), .CBITS(CBITS), .NCHK(NCHK),
                  .RESET_PC(RESET_PC)) fe
     (.clk(clk), .reset(reset), .accept(accept), .consume(m_advance),
      .redirect(redirect_q), .redirect_pc(redirect_target_q), .redirect_seq(redirect_seq_q),
      .redirect_is_trap(redirect_is_trap_q), .redirect_ckpt(redirect_ckpt_q),
      .irq_inject(irq_inject),
      .imem_addr(imem_va), .imem_ipc(), .imem_data(imem_data),
      .imem_avail(imem_avail_g),
      .imem_fault(immu_ready & immu_fault), .imem_cause(immu_cause),
      .res_v(res_v_q), .res_cbr(res_cbr_q), .res_call(res_call_q), .res_ret(res_ret_q),
      .res_taken(res_taken_q), .res_ckpt(res_ckpt_q), .res_tgt(res_tgt_q), .res_rep(res_rep_q),
      .d_valid(d_valid), .d_pc(d_pc), .d_insn(d_insn), .d_rvc(d_rvc), .d_seq(d_seq),
      .d_ckpt(d_ckpt), .d_pred_npc(d_pred_npc),
      .d_rd(d_rd), .d_rs1(d_rs1), .d_rs2(d_rs2), .d_rs3(d_rs3),
      .d_rd_v(d_rd_v), .d_rs1_v(d_rs1_v), .d_rs2_v(d_rs2_v), .d_rs3_v(d_rs3_v),
      .d_imm(d_imm),
      .d_alu_op(d_alu_op), .d_alu_w(d_alu_w), .d_alu_uw(d_alu_uw), .d_op1_sel(d_op1_sel),
      .d_op2_imm(d_op2_imm), .d_res_link(d_res_link),
      .d_is_mem(d_is_mem), .d_is_store(d_is_store), .d_mem_size(d_mem_size),
      .d_mem_signed(d_mem_signed), .d_is_branch(d_is_branch), .d_br_func(d_br_func),
      .d_is_jump(d_is_jump), .d_is_jalr(d_is_jalr), .d_is_mul(d_is_mul), .d_is_csr(d_is_csr),
      .d_csr_func(d_csr_func), .d_is_serialize(d_is_serialize), .d_is_amo(d_is_amo),
      .d_amo_func(d_amo_func), .d_is_fp(d_is_fp), .d_is_fencei(d_is_fencei),
      .d_is_cbo(d_is_cbo), .d_cbo_zero(d_cbo_zero), .d_cbo_keep(d_cbo_keep),
      .d_illegal(d_illegal), .d_mis_taken(d_mis_taken), .d_mis_nt(d_mis_nt),
      .d_fault(d_fault), .d_fault_cause(d_fault_cause), .d_fault_tval(d_fault_tval),
      .cur_seq(fe_cur_seq));

   // Valid-DRAM window for the MMU's unbacked-PA access-fault check. Enforced only
   // under cosim, sized to the modeled DDR so an out-of-range access faults exactly as
   // the simmerv golden model does; the FPGA/test default stays fully permissive, so
   // no new faults appear outside cosim. (Same rule as backend_top.)
`ifdef COSIM_MEM_SIZE_LG2
   localparam [63:0] DRAM_BASE = 64'h8000_0000;
   localparam [63:0] DRAM_TOP  = 64'h8000_0000 + (64'd1 << `COSIM_MEM_SIZE_LG2);
`else
   localparam [63:0] DRAM_BASE = 64'd0;
   localparam [63:0] DRAM_TOP  = 64'hFFFF_FFFF_FFFF_FFFF;
`endif

   // instruction-side translation. M-mode fetches are physical (satp forced Bare).
   wire [63:0] mmu_satp;
   wire [1:0]  mmu_priv, mmu_dpriv;
   wire        mmu_sum, mmu_mxr, mmu_flush;
   wire [63:0] satp_fetch = (mmu_priv  == 2'd3) ? 64'd0 : mmu_satp;
   wire [63:0] satp_data  = (mmu_dpriv == 2'd3) ? 64'd0 : mmu_satp;

   mmu #(.AW(56), .DRAM_BASE(DRAM_BASE), .DRAM_TOP(DRAM_TOP)) u_immu
     (.clk(clk), .reset(reset),
      .req_valid(1'b1), .req_vaddr(imem_va), .req_access(2'd0),
      .priv(mmu_priv), .sum(mmu_sum), .mxr(mmu_mxr), .satp(satp_fetch), .flush(mmu_flush),
      .ptw_addr(ptw_addr), .ptw_read(ptw_read),
      .ptw_rdata(ptw_rdata), .ptw_rvalid(ptw_rvalid),
      .t_ready(immu_ready), .t_paddr(immu_pa), .t_fault(immu_fault),
      .t_cause(immu_cause), .t_uncached());
   assign imem_addr = {8'd0, immu_pa};

   // =========================================================== stage X
   wire [63:0] rf_rs1, rf_rs2, rf_rs3;
   wire        rf_we;
   wire [5:0]  rf_wa;
   wire [63:0] rf_wd;

   ino_regfile u_rf
     (.clk(clk), .rs1(d_rs1), .rs1_val(rf_rs1), .rs2(d_rs2), .rs2_val(rf_rs2),
      .rs3(d_rs3), .rs3_val(rf_rs3), .we(rf_we), .wa(rf_wa), .wd(rf_wd));

   // M-stage registers (declared here: the bypass reads them)
   reg              m_valid, m_rvc, m_rd_v;
   reg  [PCW-1:0]   m_pc, m_pred_npc, m_fault_tval, m_target, m_taken_tgt;
   reg  [31:0]      m_insn;
   reg  [SEQW-1:0]  m_seq;
   reg  [CBITS-1:0] m_ckpt;
   reg  [5:0]       m_rd, m_rs1;
   reg  [63:0]      m_imm, m_result, m_addr, m_st_data, m_rs1_val, m_rs3_val;
   reg  [1:0]       m_mem_size;
   reg              m_mem_signed, m_is_mem, m_is_store, m_is_amo;
   reg  [4:0]       m_amo_func;
   reg              m_is_branch, m_is_jump, m_is_jalr, m_redirect, m_taken;
   reg              m_is_mul, m_is_csr, m_is_serialize, m_is_fp, m_is_fencei;
   reg  [2:0]       m_csr_func;
   reg              m_is_cbo, m_cbo_zero, m_cbo_keep;
   reg              m_illegal, m_fault;
   reg  [3:0]       m_fault_cause;
   initial begin m_valid = 1'b0; end

   // the M-stage writeback value (also the single bypass source)
   wire [63:0] m_wb_val;

   // one bypass level: M -> X. An instruction two ahead has already landed in the RF.
   wire       byp1 = m_valid & m_rd_v & (m_rd == d_rs1);
   wire       byp2 = m_valid & m_rd_v & (m_rd == d_rs2);
   wire       byp3 = m_valid & m_rd_v & (m_rd == d_rs3);
   wire [63:0] x_rs1 = byp1 ? m_wb_val : rf_rs1;
   wire [63:0] x_rs2 = byp2 ? m_wb_val : rf_rs2;
   wire [63:0] x_rs3 = byp3 ? m_wb_val : rf_rs3;

   wire [63:0] x_result, x_addr, x_target, x_taken_tgt;
   wire        x_redirect, x_taken;

   ino_exec u_x
     (.alu_op(d_alu_op), .alu_w(d_alu_w), .alu_uw(d_alu_uw), .op1_sel(d_op1_sel),
      .op2_imm(d_op2_imm), .res_link(d_res_link), .is_rvc(d_rvc),
      .is_branch(d_is_branch), .is_jump(d_is_jump), .is_jalr(d_is_jalr),
      .br_func(d_br_func),
      .rs1_val(x_rs1), .rs2_val(x_rs2), .imm(d_imm), .pc(d_pc),
      .pred_npc(d_pred_npc), .mis_taken(d_mis_taken), .mis_nt(d_mis_nt),
      .result(x_result), .addr(x_addr), .redirect(x_redirect), .target(x_target),
      .taken(x_taken), .taken_tgt(x_taken_tgt));

   // =========================================================== stage M
`include "smolrv64_fp_ops.vh"           // fcmp_s/d, fclass_s/d (in-core FP ops)

   // ---- FS-disabled: any FP instruction (arith, in-core, or FP load/store) executed
   // with mstatus.FS==Off raises illegal-instruction, like Linux lazy-FP. A write to FS
   // redirects and refetches younger ops (csr_file do_fschg) and FP ops are not
   // serializing, but M is the commit point -- so by the time an FP op is here, fs_off
   // is the retired value. Precise by construction.
   wire        fs_off, m_ill_eff;
   wire [2:0]  csr_frm;
   assign m_ill_eff = m_illegal | (m_valid & m_is_fp & fs_off);

   // ---- LSU ----
   wire        lsu_done, lsu_fault, lsu_idle;
   wire [63:0] lsu_rd_val, lsu_fault_tval;
   wire [3:0]  lsu_fault_cause;
   wire        m_mem_op = m_valid & (m_is_mem | m_is_amo) & ~m_fault & ~m_ill_eff;

   ino_lsu #(.AW(AW), .DRAM_BASE(DRAM_BASE), .DRAM_TOP(DRAM_TOP)) u_lsu
     (.clk(clk), .reset(reset),
      .req_valid(m_mem_op), .req_store(m_is_store & ~m_is_amo), .req_amo(m_is_amo),
      .req_amo_func(m_amo_func), .req_cbo(m_is_cbo), .req_cbo_zero(m_cbo_zero),
      .req_cbo_keep(m_cbo_keep),
      .req_vaddr(m_addr), .req_size(m_mem_size), .req_signed(m_mem_signed),
      .req_fp(m_is_fp), .req_st_data(m_st_data),
      .xl_satp(satp_data), .xl_priv(mmu_dpriv), .xl_sum(mmu_sum), .xl_mxr(mmu_mxr),
      .xl_flush(mmu_flush),
      .ptw_addr(dptw_addr), .ptw_read(dptw_read),
      .ptw_rdata(dptw_rdata), .ptw_rvalid(dptw_rvalid),
      .mem_raddr(dmem_raddr), .mem_ren(dmem_ren), .mem_runcached(dmem_runcached),
      .mem_rdata(dmem_rdata), .mem_rvalid(dmem_rvalid),
      .mem_wen(dmem_wen), .mem_waddr(dmem_waddr), .mem_wdata(dmem_wdata),
      .mem_wmask(dmem_wmask), .mem_wuncached(dmem_wuncached),
      .mem_cbo(dmem_cbo), .mem_cbo_zero(dmem_cbo_zero), .mem_cbo_keep(dmem_cbo_keep),
      .mem_wready(dmem_wready),
      .done(lsu_done), .rd_val(lsu_rd_val), .fault(lsu_fault),
      .fault_cause(lsu_fault_cause), .fault_tval(lsu_fault_tval), .idle(lsu_idle));
   assign dmem_idle = lsu_idle;

   // ---- multiply / divide (both start/busy/done units; they own the stall) ----
   wire [2:0]  md_f3   = m_insn[14:12];
   wire        md_is_w = m_insn[6:2] == 5'b01110;         // OP-32 -> MULW/DIVW..
   wire        md_div  = md_f3[2];
   reg         md_started;
   wire        m_md_op = m_valid & m_is_mul & ~m_ill_eff;
   wire        mul_start = m_md_op & ~md_div & ~md_started;
   wire        div_start = m_md_op &  md_div & ~md_started;
   wire        mul_done, div_done, mul_busy, div_busy;
   wire [63:0] mul_result, div_result;

   mul3 u_mul
     (.clk(clk), .reset(reset), .start(mul_start), .abort(1'b0),
      .rs1(m_rs1_val), .rs2(m_st_data), .f3(md_f3), .is_w(md_is_w),
      .busy(mul_busy), .done(mul_done), .result(mul_result));

   divider u_div
     (.clk(clk), .reset(reset), .start(div_start), .abort(1'b0),
      .rs1(m_rs1_val), .rs2(m_st_data), .f3(md_f3), .is_w(md_is_w),
      .busy(div_busy), .done(div_done), .result(div_result));

   // ---- FP unit (CVFPU) + the in-core FP ops ----
   // Everything the OoO core needs for squash recovery -- the zombie/drain latch, the
   // abort-by-seqno, the FS-dirty commit gate -- is absent here: M is the commit point,
   // so an FP op in flight can never be squashed, and its decode/operands are stable for
   // its whole (multi-cycle) stay. `decode_fp` runs off the registered m_insn.
   wire        fp_valid_d, fp_use_fpu, fp_o0i, fp_wrfp, fp_mod;
   wire [2:0]  fp_cls, fp_src, fp_dst, fp_rnd;
   wire [3:0]  fp_op;
   wire [1:0]  fp_int, fp_o0, fp_o1, fp_o2;
   decode_fp u_dfp
     (.insn(m_insn), .fp_valid(fp_valid_d), .use_fpu(fp_use_fpu), .fp_class(fp_cls),
      .op(fp_op), .op_mod(fp_mod), .src_fmt(fp_src), .dst_fmt(fp_dst), .int_fmt(fp_int),
      .rnd(fp_rnd), .op0_sel(fp_o0), .op1_sel(fp_o1), .op2_sel(fp_o2),
      .op0_int(fp_o0i), .wr_fp(fp_wrfp));

   // unified regfile -> the FP sources are just rs1/rs2/rs3 with the fp bit set
   wire [63:0] op1f = m_rs1_val, op2f = m_st_data, op3f = m_rs3_val;

   function [63:0] fpsel; input [1:0] s; input [63:0] a, b, c;
      fpsel = (s==2'd1) ? a : (s==2'd2) ? b : (s==2'd3) ? c : 64'd0; endfunction
   // unbox a single from an f-register: a properly NaN-boxed value yields its low 32
   // bits, anything else is the canonical single NaN (RISC-V spec).
   function [31:0] unbox_s; input [63:0] x;
      unbox_s = (x[63:32]==32'hffffffff) ? x[31:0] : 32'h7fc00000; endfunction

   wire [63:0] fpo0r = fpsel(fp_o0, op1f, op2f, op3f);
   wire [63:0] fpo1r = fpsel(fp_o1, op1f, op2f, op3f);
   wire [63:0] fpo2r = fpsel(fp_o2, op1f, op2f, op3f);
   wire        fp_src32 = (fp_src == 3'd0);
   // op0 may be an INTEGER source (I2F / FMV.W.X) -> pass it through unmolested
   wire [63:0] fpo0 = (fp_src32 & ~fp_o0i) ? {32'hffffffff, unbox_s(fpo0r)} : fpo0r;
   wire [63:0] fpo1 = fp_src32 ? {32'hffffffff, unbox_s(fpo1r)} : fpo1r;
   wire [63:0] fpo2 = fp_src32 ? {32'hffffffff, unbox_s(fpo2r)} : fpo2r;

   wire        fp_arith  = m_valid & fp_valid_d &  fp_use_fpu & ~m_ill_eff;
   wire        fp_incore = m_valid & fp_valid_d & ~fp_use_fpu & ~m_ill_eff;
   reg         fpu_inflight;
   initial     fpu_inflight = 1'b0;
   wire        fp_iss_ready, fp_res_valid, fpu_busy;
   wire [63:0] fp_res_data;
   wire [4:0]  fp_res_fflags;
   wire        fp_start = fp_arith & ~fpu_inflight;
   // FP32 result heading for an f-register -> NaN-box it on writeback
   wire        fp_dst32 = (fp_dst == 3'd0) & fp_wrfp;

   fp_unit #(.TAGW(1)) u_fpu
     (.clk(clk), .reset(reset),
      .iss_valid(fp_start), .iss_ready(fp_iss_ready),
      .iss_op(fp_op), .iss_op_mod(fp_mod), .iss_src_fmt(fp_src), .iss_dst_fmt(fp_dst),
      .iss_int_fmt(fp_int),
      .iss_rnd(fp_rnd == 3'b111 ? csr_frm : fp_rnd),      // dynamic rm -> fcsr.frm
      .iss_operands({fpo2, fpo1, fpo0}), .iss_tag(1'b0),
      .res_valid(fp_res_valid), .res_ready(1'b1), .res_data(fp_res_data),
      .res_fflags(fp_res_fflags), .res_tag(), .flush(1'b0), .busy(fpu_busy));

   always @(posedge clk) begin
      if (reset)                          fpu_inflight <= 1'b0;
      else if (fp_res_valid)              fpu_inflight <= 1'b0;
      else if (fp_start & fp_iss_ready)   fpu_inflight <= 1'b1;
   end
   wire fp_complete = fp_res_valid & fpu_inflight;

   // ---- in-core FP ops (single-cycle, like the ALU): SGNJ/CMP/MVXF/MVFX/FCLASS ----
   wire        fp_isd = m_insn[25];              // 0=single 1=double
   wire [2:0]  fp_f3  = m_insn[14:12];
   wire [31:0] us1 = unbox_s(op1f);              // FMV.X.W is a raw bit-move: never unboxed
   wire [31:0] us2 = unbox_s(op2f);
   wire [1:0]  cmp_d2 = fcmp_d(fp_f3, op1f, op2f);
   wire [1:0]  cmp_s2 = fcmp_s(fp_f3, us1, us2);
   reg  [63:0] fp_incore_res;
   always @* begin
      case (fp_cls)
        3'd1: fp_incore_res = fp_isd                                    // FSGNJ/N/X .D / .S
               ? (fp_f3==3'b000 ? { op2f[63],            op1f[62:0]}
                : fp_f3==3'b001 ? {~op2f[63],            op1f[62:0]}
                :                 { op2f[63]^op1f[63],   op1f[62:0]})
               : {32'hffffffff, (fp_f3==3'b000 ? { us2[31],          us1[30:0]}
                               : fp_f3==3'b001 ? {~us2[31],          us1[30:0]}
                               :                 { us2[31]^us1[31],  us1[30:0]})};
        3'd2: fp_incore_res = {63'd0, (fp_isd ? cmp_d2[0] : cmp_s2[0])};    // FEQ/FLT/FLE -> int
        3'd3: fp_incore_res = fp_isd ? op1f : {{32{op1f[31]}}, op1f[31:0]}; // FMV.X.D/W -> int (raw)
        3'd4: fp_incore_res = fp_isd ? op1f : {32'hffffffff, op1f[31:0]};   // FMV.D/W.X -> fp (box)
        3'd5: fp_incore_res = fp_isd ? fclass_d(op1f) : fclass_s(op1f);     // FCLASS -> int
        default: fp_incore_res = 64'd0;
      endcase
   end

   // fcsr.fflags accumulation: a CVFPU completion, or an in-core compare's NV bit.
   // (in-core ops other than compares raise no flags.) Both are one-cycle events: the
   // completion pulses once, and an in-core op occupies M for exactly one cycle.
   wire       fp_icmp    = fp_incore & (fp_cls == 3'd2);
   wire       fp_icmp_nv = fp_isd ? cmp_d2[1] : cmp_s2[1];
   wire       fp_flags_we = fp_complete | fp_icmp;
   wire [4:0] fp_flags    = fp_complete ? fp_res_fflags : {fp_icmp_nv, 4'd0};

   // ---- CSR file ----
   wire        m_is_sys  = m_valid & (m_insn[6:2] == 5'b11100) & ~m_ill_eff & ~m_fault;
   wire        m_is_irqop= m_is_sys & (m_insn[14:12] == 3'b000) & (m_insn[31:20] == 12'h7F0);
   wire [63:0] csr_rdata, csr_redir_tgt;
   wire        csr_redir_v, csr_redir_trap, csr_illegal;
   wire        csr_irq_v;
   wire [3:0]  csr_irq_cause;

   // external (non-system-op) traps: fetch fault, illegal instruction, data fault
   wire        xtrap_v     = m_valid & (m_fault | m_ill_eff | (m_mem_op & lsu_fault));
   wire [3:0]  xtrap_cause = m_fault  ? m_fault_cause
                           : m_ill_eff? 4'd2                     // illegal instruction
                           :            lsu_fault_cause;
   wire [63:0] xtrap_tval  = m_fault  ? m_fault_tval
                           : m_ill_eff? 64'd0
                           :            lsu_fault_tval;

   // ---- stall attribution: turn CPI into a CPI stack ----
   // The pipe fails to retire on a given cycle for exactly one of two reasons: M is
   // holding an instruction that has not completed (charged to the unit it is waiting
   // on), or X had no instruction to give (a frontend bubble, sub-attributed to the
   // iMMU walking vs the I$ having no window). `st_ser` is the third case: M is free
   // but a serializing op in flight keeps the frontend from handing anything over.
   wire st_m      = m_valid & ~m_done;              // M stalled at all
   wire st_mem    = st_m & m_mem_op;                // ...on the LSU
   wire st_div    = st_m & m_md_op &  md_div;       // ...on the divider
   wire st_mul    = st_m & m_md_op & ~md_div;       // ...on the multiplier
   wire st_fpu    = st_m & fp_arith;                // ...on the CVFPU
   wire st_ser    = m_advance & ~accept;            // serialize block holds the frontend
   wire fe_bub    = ~st_m & ~d_valid & ~redirect;   // X starved, M not already stalled
   wire fe_mmu    = fe_bub & ~immu_ready;           // ...iMMU walking
   wire fe_ic     = fe_bub &  immu_ready & (imem_avail_g == {$clog2(HW+2){1'b0}});

   wire [14:0] hpm_ev = {fe_ic, fe_mmu, fe_bub, st_ser, st_fpu, st_mul, st_div, st_mem,
                         hpm_ic_miss, hpm_ic_access, hpm_dc_miss, hpm_dc_access,
                         redirect, m_valid & m_is_store & lsu_done, m_valid & m_is_mem
                         & ~m_is_store & lsu_done};

   csr_file u_csr
     (.clk(clk), .reset(reset),
      .raddr(m_imm[11:0]), .rdata(csr_rdata),
      .redir_target(csr_redir_tgt), .redir_valid(csr_redir_v),
      .redir_is_trap(csr_redir_trap), .csr_illegal(csr_illegal),
      .o_satp(mmu_satp), .o_priv(mmu_priv), .o_dpriv(mmu_dpriv),
      .o_sum(mmu_sum), .o_mxr(mmu_mxr), .o_frm(csr_frm), .o_fs_off(fs_off),
      .fp_fflags_we(fp_flags_we), .fp_fflags(fp_flags),
      // mstatus.FS -> Dirty when an FP-state writer RETIRES. In-order that is exactly
      // "a retiring instruction wrote an f-register" (arch 32..63), which covers FP
      // arith, the in-core FP writers and FP loads -- and excludes FSW/FSD, which
      // write memory, not FP state. Commit-gated by construction: `retire` is the
      // commit event, so a trapping op never dirties FS.
      .fp_dirty_commit(retire & m_rd_v & m_rd[5]),
      .o_tlb_flush(mmu_flush),
      .xtrap_v(xtrap_v), .xtrap_intr(1'b0), .xtrap_cause(xtrap_cause),
      .xtrap_epc(m_pc), .xtrap_tval(xtrap_tval),
      .hw_ip(hw_ip), .mtime(mtime), .retire_cnt(retire ? 6'd1 : 6'd0), .hpm_ev(hpm_ev),
      .irq_v(csr_irq_v), .irq_cause(csr_irq_cause),
      // csr_file's ILA debug bus. The in-order SoC puts no ILA on the CSR file, so these
      // outputs go nowhere -- named and left EMPTY on purpose. PINMISSING gates this build
      // and PINCONNECTEMPTY does not, so a deliberate non-connection has to say so instead
      // of being silently omitted (same treatment as the iMMU's t_uncached).
      .dbg_timer(), .dbg_mtvec(), .dbg_mtvec_we(), .dbg_csrop(), .dbg_csrop_v(),
      .upd_valid(m_is_sys), .upd_is_csr(m_is_csr), .upd_func(m_csr_func),
      .upd_addr(m_imm[11:0]),
      .upd_src(m_csr_func[2] ? {59'b0, m_imm[16:12]} : m_rs1_val),
      .upd_pc(m_pc));

   // ---- completion ----
   assign m_done = ~m_valid              ? 1'b1
                 : m_fault | m_ill_eff   ? 1'b1   // poisoned: traps immediately
                 : m_mem_op              ? lsu_done
                 : m_md_op               ? (md_div ? div_done : mul_done)
                 : fp_arith              ? fp_complete
                 :                         1'b1;  // in-core FP included: single-cycle
   assign m_advance = ~m_valid | m_done;

   // ---- trap / redirect ----
   wire m_trap = xtrap_v | (m_is_sys & csr_redir_trap);
   wire csr_red = xtrap_v | (m_is_sys & csr_redir_v);
   assign redirect         = m_valid & m_done & (csr_red | m_redirect | m_is_fencei);
   assign redirect_target  = csr_red     ? csr_redir_tgt
                           : m_is_fencei ? (m_pc + (m_rvc ? 64'd2 : 64'd4))
                           :               m_target;
   assign redirect_is_trap = m_trap;
   assign redirect_ckpt    = m_ckpt;
   assign redirect_seq     = m_trap ? m_seq : (m_seq + 1'b1);
   assign ifence           = m_valid & m_done & m_is_fencei;

   // ---- branch resolve / BTB training ----
   wire m_link_rd = m_rd_v & ((m_rd == 6'd1) | (m_rd == 6'd5));   // x1/x5 = link registers
   wire m_link_rs = (m_rs1 == 6'd1) | (m_rs1 == 6'd5);
   assign res_v     = m_valid & m_done & (m_is_branch | m_is_jump) & ~m_trap;
   assign res_cbr   = m_is_branch;
   assign res_call  = m_is_jump & m_link_rd;
   assign res_ret   = m_is_jalr & m_link_rs & ~m_link_rd;
   assign res_taken = m_taken;
   assign res_ckpt  = m_ckpt;
   assign res_tgt   = m_taken_tgt;
   assign res_rep   = res_v & res_cbr & m_redirect & ~csr_red;

   // ---- writeback ----
   assign m_wb_val = (m_is_mem | m_is_amo) ? lsu_rd_val   // FP loads too (LSU NaN-boxes FLW)
                   : m_is_csr              ? csr_rdata
                   : m_is_mul              ? (md_div ? div_result : mul_result)
                   : fp_arith              ? (fp_dst32 ? {32'hffffffff, fp_res_data[31:0]}
                                                       : fp_res_data)
                   : fp_incore             ? fp_incore_res
                   :                         m_result;
   assign rf_we = m_valid & m_done & m_rd_v & ~m_trap;
   assign rf_wa = m_rd;
   assign rf_wd = m_wb_val;

   assign retire      = m_valid & m_done & ~m_trap & ~m_is_irqop;
   assign retire_pc   = m_pc;
   assign retire_insn = m_insn;

   // ---- interrupt injection: a solo SYSTEM pseudo-op that traps in M ----
   reg inject_inflight;
   initial inject_inflight = 1'b0;
   assign irq_inject = csr_irq_v & ~inject_inflight & ~redirect & ~redirect_q;
   always @(posedge clk) begin
      if (reset)                        inject_inflight <= 1'b0;
      else if (irq_inject & accept)     inject_inflight <= 1'b1;
      else if (redirect | redirect_q | ~csr_irq_v) inject_inflight <= 1'b0;
   end

`ifdef INO_COSIM
   // ======================= cosim retire stream (VERIFY-ONLY) =======================
   // Hand every retiring instruction (and every trap) to simmerv via probe_retire(),
   // the same DPI contract probe/probe_cosim.cpp already implements.
   //
   // The OoO core needs ~250 lines here: a 40-deep FIFO to rebuild program order from
   // out-of-order commit, per-entry value-ready tracking (an ALU op commits on the
   // issue count, before its writeback), squash-by-seqno tail truncation, and a
   // by-PC search to convert a committed entry into its own trap. In-order needs
   // NONE of it -- M retires one instruction per cycle in program order, and its rd
   // value is live on the writeback bus in that very cycle.
   //
   // The one thing that IS needed is the same 1-cycle lag the OoO harness uses: the
   // retiring instruction's CSR/regfile writes land on the edge that ends its M
   // cycle, so `mepc` is read LIVE one cycle later (probe_cosim wants mepc AFTER the
   // retire -- e.g. after an mret, or after a trap has captured it). Everything that
   // a trap changes at that same edge -- privilege above all -- must therefore be
   // REGISTERED at retire time, not read live.
   import "DPI-C" function void probe_retire(
      input longint unsigned pc,
      input int     unsigned insn,
      input byte    unsigned rd_kind,     // 0 none, 1 int, 2 fp
      input byte    unsigned rd_idx,
      input byte    unsigned prv,
      input byte    unsigned trapped,
      input longint unsigned rd_val,
      input longint unsigned trap_cause,
      input longint unsigned trap_tval,
      input longint unsigned mtime_v,
      input longint unsigned mtimecmp_v,
      input longint unsigned mepc_v,
      input byte    unsigned seip_v);

   // csr_file internals, tapped exactly as backend_top does
   wire        cot_fire  = u_csr.trap_v;
   wire [63:0] cot_cause = u_csr.trap_cause;
   wire [63:0] cot_tval  = u_csr.trap_tval;
   wire        cot_intr  = cot_cause[63];
   // an instruction-side fault retires no instruction (insn=0), like an interrupt
   wire        cot_ifault = ~cot_intr & ((cot_cause[5:0]==6'd0) | (cot_cause[5:0]==6'd1)
                                       | (cot_cause[5:0]==6'd12));

   wire [1:0] ck_rk = ~m_rd_v ? 2'd0 : m_rd[5] ? 2'd2 : 2'd1;

   reg        e_v, e_trap;
   reg [63:0] e_pc, e_val, e_cause, e_tval;
   reg [31:0] e_insn;
   reg [1:0]  e_rk, e_prv;
   reg [4:0]  e_ri;
   initial    e_v = 1'b0;

   always @(posedge clk) begin
      e_v <= 1'b0;
      if (!reset && m_valid && m_done) begin
         // a trap and a retire are mutually exclusive, and every M cycle is one or
         // the other (or neither, for the interrupt pseudo-op, which emits as a trap)
         if (cot_fire) begin
            e_v <= 1'b1;  e_trap <= 1'b1;
            e_pc <= m_pc;
            e_insn <= (cot_intr | cot_ifault) ? 32'd0 : m_insn;
            e_rk <= 2'd0;  e_ri <= 5'd0;  e_val <= 64'd0;
            e_cause <= cot_cause;  e_tval <= cot_tval;
            e_prv <= u_csr.priv;                  // privilege BEFORE the trap
         end else if (retire) begin
            e_v <= 1'b1;  e_trap <= 1'b0;
            e_pc <= m_pc;  e_insn <= m_insn;
            e_rk <= ck_rk;  e_ri <= m_rd[4:0];  e_val <= m_wb_val;
            e_cause <= 64'd0;  e_tval <= 64'd0;
            e_prv <= u_csr.priv;
         end
      end
      // emit one cycle later, so this instruction's own CSR writes have landed
      if (e_v)
         probe_retire(e_pc, e_insn, {6'd0, e_rk},
                      (e_rk == 2'd0) ? 8'd0 : {3'd0, e_ri},
                      {6'd0, e_prv}, {7'd0, e_trap}, e_val, e_cause, e_tval,
                      64'd0, {64{1'b1}}, `VA_UNPACK40(u_csr.mepc), 8'd0);
   end
`endif

   // =========================================================== flow control
   // Nothing follows a serializing op into X until it has left M, so CSR values,
   // privilege, satp and mstatus are never read stale.
   wire ser_block = (d_valid & d_is_serialize) | (m_valid & m_is_serialize);
   assign accept  = m_advance & ~ser_block;

   always @(posedge clk) begin
      if (reset) begin
         m_valid <= 1'b0; md_started <= 1'b0;
      end else begin
         if (m_advance) md_started <= 1'b0;
         else if (mul_start | div_start) md_started <= 1'b1;

         if (m_advance) begin
            m_valid       <= d_valid & ~redirect & ~redirect_q;   // ~q: the shadow bundle
            m_pc          <= d_pc;
            m_insn        <= d_insn;
            m_rvc         <= d_rvc;
            m_seq         <= d_seq;
            m_ckpt        <= d_ckpt;
            m_pred_npc    <= d_pred_npc;
            m_rd          <= d_rd;
            m_rd_v        <= d_rd_v;
            m_rs1         <= d_rs1;
            m_imm         <= d_imm;
            m_result      <= x_result;
            m_addr        <= x_addr;
            m_st_data     <= x_rs2;
            m_rs1_val     <= x_rs1;
            m_rs3_val     <= x_rs3;   // FMA 3rd operand
            m_mem_size    <= d_mem_size;
            m_mem_signed  <= d_mem_signed;
            m_is_mem      <= d_is_mem;
            m_is_store    <= d_is_store;
            m_is_amo      <= d_is_amo;
            m_amo_func    <= d_amo_func;
            m_is_branch   <= d_is_branch;
            m_is_jump     <= d_is_jump;
            m_is_jalr     <= d_is_jalr;
            m_redirect    <= x_redirect;
            m_target      <= x_target;
            m_taken       <= x_taken;
            m_taken_tgt   <= x_taken_tgt;
            m_is_mul      <= d_is_mul;
            m_is_csr      <= d_is_csr;
            m_csr_func    <= d_csr_func;
            m_is_serialize<= d_is_serialize;
            m_is_fp       <= d_is_fp;
            m_is_fencei   <= d_is_fencei;
            m_is_cbo      <= d_is_cbo;
            m_cbo_zero    <= d_cbo_zero;
            m_cbo_keep    <= d_cbo_keep;
            m_illegal     <= d_illegal;
            m_fault       <= d_fault;
            m_fault_cause <= d_fault_cause;
            m_fault_tval  <= d_fault_tval;
         end
      end
   end
endmodule

`default_nettype wire
