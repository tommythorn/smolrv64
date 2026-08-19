`default_nettype none

// In-order frontend (stage F): PC -> iMMU-translated I$ window -> aligner ->
// RVC expand -> decode, terminating at the IR register that feeds stage X.
//
// This is probe/frontend.v with `decode_rename` replaced by a plain register:
// `fetch`, `predictor` and `decode_slot` are instantiated UNMODIFIED. Width is
// scalar (IW=1), so the aligner's cross-slot machinery collapses to one slot and
// `decode_xslot` is not needed at all.
//
// CHECKPOINTS. `predictor` keeps its speculative {ghr, ras, ras_ptr} in a
// checkpoint ring, snapshotting at `create` and restoring at `rollback`. We keep
// that interface exactly, degenerate to ONE CHECKPOINT PER INSTRUCTION: `cur` is a
// rotating counter, `create` pulses the cycle an instruction lands in the IR
// register (one cycle after its fetch handshake -- the same lag the OoO core's
// fetch->dispatch had), and the index rides with the instruction as `d_ckpt` so
// the resolve in M can name it. NCHK=4 is ample: at most two instructions are ever
// in flight past a branch, and `create` advances `cur` by at most one per cycle.
//
// This lag is exactly why branches resolve in M rather than X (see
// docs/inorder-plan.md): resolving in X would collide `create` and the resolve of
// the same instruction in one cycle, so `pdet[cur]` would not yet hold the
// predict details the training reads, and a mispredict would suppress the very
// snapshot its own rollback restores.
//
// FETCH FAULTS. When the iMMU faults on the fetch address the core drives
// `imem_avail`=0 (so `fetch` presents nothing) and raises `imem_fault`. We then
// present a POISONED instruction: valid, no side effects, carrying cause/tval down
// to the single trap point in M. It is marked serializing, so exactly one is
// injected -- nothing follows it into X until the trap redirects.
module ino_frontend
  #(parameter PCW   = 64,
    parameter SEQW  = 8,
    parameter HW    = 2,             // fetch window halfwords (one 32-bit instruction)
    parameter CBITS = 2,
    parameter NCHK  = 4,
    parameter [PCW-1:0] RESET_PC = 0)
   (input  wire                    clk,
    input  wire                    reset,

    // ---- back-pressure ----
    // `consume`: stage X handed its instruction to M this cycle, so the IR register
    // is free. `accept`: a new instruction may be loaded into it (consume, and no
    // serializing op in flight). accept implies consume.
    input  wire                    accept,
    input  wire                    consume,

    // ---- redirect (from M: mispredict, trap, xret, fence.i) ----
    input  wire                    redirect,
    input  wire [PCW-1:0]          redirect_pc,
    input  wire [SEQW-1:0]         redirect_seq,
    input  wire                    redirect_is_trap,  // annul the op (restore TO its ckpt)
    input  wire [CBITS-1:0]        redirect_ckpt,     // the redirecting op's ckpt
    input  wire                    irq_inject,        // present the interrupt pseudo-op

    // ---- instruction memory (combinational read, iMMU-translated by the core) ----
    output wire [PCW-1:0]          imem_addr,         // VA to translate
    output wire [PCW-1:0]          imem_ipc,          // PC of the instruction (fault EPC)
    input  wire [HW*16-1:0]        imem_data,
    input  wire [$clog2(HW+2)-1:0] imem_avail,        // 0 when translating/faulting/missing
    input  wire                    imem_fault,        // iMMU fault on this fetch (qualified ready)
    input  wire [3:0]              imem_cause,

    // ---- branch resolve / training (from M) ----
    input  wire                    res_v,
    input  wire                    res_cbr,
    input  wire                    res_call,
    input  wire                    res_ret,
    input  wire                    res_taken,
    input  wire [CBITS-1:0]        res_ckpt,
    input  wire [PCW-1:0]          res_tgt,
    input  wire                    res_rep,

    // ================= IR register: the F/X boundary =================
    output reg                     d_valid,
    output reg  [PCW-1:0]          d_pc,
    output reg  [31:0]             d_insn,      // RVC-expanded 32-bit form
    output reg                     d_rvc,
    output reg  [SEQW-1:0]         d_seq,
    output reg  [CBITS-1:0]        d_ckpt,
    output reg  [PCW-1:0]          d_pred_npc,
    // operands
    output reg  [5:0]              d_rd, d_rs1, d_rs2, d_rs3,
    output reg                     d_rd_v, d_rs1_v, d_rs2_v, d_rs3_v,
    output reg  [63:0]             d_imm,
    // execute control
    output reg  [5:0]              d_alu_op,
    output reg                     d_alu_w, d_alu_uw,
    output reg  [1:0]              d_op1_sel,
    output reg                     d_op2_imm, d_res_link,
    output reg                     d_is_mem, d_is_store,
    output reg  [1:0]              d_mem_size,
    output reg                     d_mem_signed,
    output reg                     d_is_branch,
    output reg  [2:0]              d_br_func,
    output reg                     d_is_jump, d_is_jalr,
    output reg                     d_is_mul, d_is_csr,
    output reg  [2:0]              d_csr_func,
    output reg                     d_is_serialize,
    output reg                     d_is_amo,
    output reg  [4:0]              d_amo_func,
    output reg                     d_is_fp, d_is_fencei,
    output reg                     d_is_cbo, d_cbo_zero, d_cbo_keep,
    output reg                     d_illegal,
    // precomputed branch compares (payload-static -> keeps them out of X's cone)
    output reg                     d_mis_taken, d_mis_nt,
    // fetch-side fault poison
    output reg                     d_fault,
    output reg  [3:0]              d_fault_cause,
    output reg  [PCW-1:0]          d_fault_tval,
    output wire [SEQW-1:0]         cur_seq);          // fetch PC's seqno (trap resume)

   // ---------------------------------------------------------------- fetch
   wire               f_valid, f_brt, bp_v;
   wire [31:0]        f_inst;
   wire [PCW-1:0]     f_pc;
   wire [SEQW-1:0]    f_seq;
   wire [PCW-1:0]     f_npc, f_pnpc, f_ftn, bp_tgt;

   // checkpoint ring: one per instruction. `create` pulses the cycle the IR
   // register holds a freshly loaded instruction; `cur` is that instruction's index.
   reg  [CBITS-1:0]   cur;
   reg                create;
   // a mispredicting branch reopens the span AFTER itself (it is kept); a trap
   // reopens its OWN span (the faulting op is annulled) -- same rule as the OoO core.
   wire [CBITS-1:0]   rb_idx = redirect_is_trap ? redirect_ckpt : (redirect_ckpt + 1'b1);

   fetch #(.IW(1), .HW(HW), .PCW(PCW), .SEQW(SEQW), .RESET_PC(RESET_PC)) u_fetch
     (.clk(clk), .reset(reset),
      .redirect(redirect), .redirect_pc(redirect_pc), .redirect_seq(redirect_seq),
      .solo_all(1'b0),                 // IW=1: every bundle is already one instruction
      .irq_inject(irq_inject),
      .pred_v(bp_v), .pred_tgt(bp_tgt),
      .npc(f_npc), .pred_npc(f_pnpc), .ft_npc(f_ftn), .br_term(f_brt),
      .imem_addr(imem_addr), .imem_ipc(imem_ipc), .imem_data(imem_data),
      .imem_avail(imem_avail),
      .ready(accept), .valid(f_valid),
      .slot_valid(), .inst(f_inst), .pc(f_pc), .seq(f_seq), .cur_seq(cur_seq));

   wire fire = accept & f_valid;      // mirrors fetch's own handshake

   // ------------------------------------------------------- branch predictor
   // CKPT_RAS=0: no per-checkpoint RAS array snapshot. Costs some return-prediction
   // accuracy on mispredict paths, never correctness (M resolves the truth); buys 2048
   // flops of congestion relief inside u_bp, which is the sink of every failing family.
   predictor #(.PCW(PCW), .CBITS(CBITS), .NCHK(NCHK), .CKPT_RAS(0)) u_bp
     (.clk(clk), .reset(reset),
      .npc(f_npc), .fire(fire), .base_pc(imem_ipc), .ft_npc(f_ftn), .cti_ok(f_brt),
      .pred_v(bp_v), .pred_tgt(bp_tgt),
      .create(create), .cur(cur), .rollback(redirect), .rollback_idx(rb_idx),
      .res_v(res_v), .res_cbr(res_cbr), .res_call(res_call), .res_ret(res_ret),
      .res_taken(res_taken), .res_ckpt(res_ckpt), .res_tgt(res_tgt), .res_rep(res_rep));

   // ---------------------------------------------------------------- decode
   wire        s_rvc, s_rd_v, s_rs1_v, s_rs2_v, s_rs3_v, s_legal;
   wire [31:0] s_exp;
   wire [5:0]  s_rd, s_rs1, s_rs2, s_rs3;
   wire [63:0] s_imm;
   wire        s_has_imm;
   wire [5:0]  s_alu_op;
   wire        s_alu_w, s_alu_uw, s_op2_imm, s_res_link;
   wire [1:0]  s_op1_sel;
   wire        s_is_mem, s_is_store, s_mem_signed, s_is_branch, s_is_jump;
   wire [1:0]  s_mem_size;
   wire [2:0]  s_br_func, s_csr_func;
   wire        s_is_mul, s_is_csr, s_is_serialize, s_is_amo, s_is_fencei;
   wire [4:0]  s_amo_func;
   wire        s_is_cbo, s_cbo_zero, s_cbo_keep, s_illegal;

   decode_slot #(.SEQW(SEQW)) u_dec
     (.inst(f_inst), .in_valid(1'b1), .seq_in(f_seq),
      .valid(), .seq(), .is_rvc(s_rvc), .expanded(s_exp),
      .rd(s_rd), .rd_v(s_rd_v), .rs1(s_rs1), .rs1_v(s_rs1_v),
      .rs2(s_rs2), .rs2_v(s_rs2_v), .rs3(s_rs3), .rs3_v(s_rs3_v),
      .imm(s_imm), .has_imm(s_has_imm), .legal(s_legal),
      .alu_op(s_alu_op), .alu_w(s_alu_w), .alu_uw(s_alu_uw), .op1_sel(s_op1_sel),
      .op2_imm(s_op2_imm), .res_link(s_res_link), .is_mem(s_is_mem),
      .is_store(s_is_store), .mem_size(s_mem_size), .mem_signed(s_mem_signed),
      .is_branch(s_is_branch), .br_func(s_br_func), .is_jump(s_is_jump),
      .is_mul(s_is_mul), .is_csr(s_is_csr), .csr_func(s_csr_func),
      .is_serialize(s_is_serialize), .is_amo(s_is_amo), .amo_func(s_amo_func),
      .is_fencei(s_is_fencei), .is_cbo(s_is_cbo), .cbo_zero(s_cbo_zero),
      .cbo_keep(s_cbo_keep), .illegal(s_illegal));

   // JALR needs distinguishing from JAL for the branch unit (target is rs1+imm,
   // not pc+imm); decode_exec does not export it, but the opcode is right here.
   wire s_is_jalr = (s_exp[6:2] == 5'b11001);

   // Branch mispredict compares are payload-static (pc/imm/pred_npc all known at
   // decode), so precompute them here and hand branch_unit two flops -- only JALR
   // pays an X-time 64-bit compare against its AGU result.
   wire [PCW-1:0] s_taken_pc = f_pc + s_imm;
   wire [PCW-1:0] s_ft_pc    = f_pc + (s_rvc ? 64'd2 : 64'd4);

   // fetch-fault pseudo-op: presented only when fetch itself has nothing (an
   // interrupt injection wins -- fetch forces a valid bundle for it, and an
   // interrupt is taken before the instruction that would have faulted).
   wire fault_op = imem_fault & ~f_valid;
   wire ld_valid = f_valid | fault_op;

   // ------------------------------------------------------- IR register (F/X)
   always @(posedge clk) begin
      if (reset) begin
         d_valid <= 1'b0; create <= 1'b0; cur <= {CBITS{1'b0}};
      end else if (redirect) begin
         d_valid <= 1'b0; create <= 1'b0; cur <= rb_idx;
      end else begin
         create <= 1'b0;
         if (consume) d_valid <= 1'b0;      // X emptied; overridden by the load below
         if (accept) begin
            d_valid <= ld_valid;
            if (ld_valid) begin create <= 1'b1; cur <= cur + 1'b1; end

            d_ckpt        <= cur;
            d_seq         <= f_seq;
            d_pc          <= fault_op ? imem_ipc : f_pc;
            d_pred_npc    <= f_pnpc;
            d_fault       <= fault_op;
            d_fault_cause <= imem_cause;
            d_fault_tval  <= imem_addr;      // faulting VA (straddle: pc+2)

            // A poisoned fetch carries no operation: every class flag is cleared so
            // it slides to M as a pure trap request. Serializing keeps it alone.
            d_insn        <= fault_op ? 32'd0  : s_exp;
            d_rvc         <= fault_op ? 1'b0   : s_rvc;
            d_rd          <= fault_op ? 6'd0   : s_rd;
            d_rd_v        <= fault_op ? 1'b0   : s_rd_v;
            d_rs1         <= fault_op ? 6'd0   : s_rs1;
            d_rs1_v       <= fault_op ? 1'b0   : s_rs1_v;
            d_rs2         <= fault_op ? 6'd0   : s_rs2;
            d_rs2_v       <= fault_op ? 1'b0   : s_rs2_v;
            d_rs3         <= fault_op ? 6'd0   : s_rs3;
            d_rs3_v       <= fault_op ? 1'b0   : s_rs3_v;
            d_imm         <= fault_op ? 64'd0  : s_imm;
            d_alu_op      <= s_alu_op;
            d_alu_w       <= fault_op ? 1'b0   : s_alu_w;
            d_alu_uw      <= fault_op ? 1'b0   : s_alu_uw;
            d_op1_sel     <= fault_op ? 2'd0   : s_op1_sel;
            d_op2_imm     <= fault_op ? 1'b0   : s_op2_imm;
            d_res_link    <= fault_op ? 1'b0   : s_res_link;
            d_is_mem      <= fault_op ? 1'b0   : s_is_mem;
            d_is_store    <= fault_op ? 1'b0   : s_is_store;
            d_mem_size    <= s_mem_size;
            d_mem_signed  <= s_mem_signed;
            d_is_branch   <= fault_op ? 1'b0   : s_is_branch;
            d_br_func     <= s_br_func;
            d_is_jump     <= fault_op ? 1'b0   : s_is_jump;
            d_is_jalr     <= fault_op ? 1'b0   : s_is_jalr;
            d_is_mul      <= fault_op ? 1'b0   : s_is_mul;
            d_is_csr      <= fault_op ? 1'b0   : s_is_csr;
            d_csr_func    <= s_csr_func;
            d_is_serialize<= fault_op ? 1'b1   : s_is_serialize;
            d_is_amo      <= fault_op ? 1'b0   : s_is_amo;
            d_amo_func    <= s_amo_func;
            d_is_fp       <= fault_op ? 1'b0   : (s_exp[6:2] == 5'b10100) |
                                                 (s_exp[6:2] == 5'b10000) | (s_exp[6:2] == 5'b10001) |
                                                 (s_exp[6:2] == 5'b10010) | (s_exp[6:2] == 5'b10011) |
                                                 (s_exp[6:2] == 5'b00001) | (s_exp[6:2] == 5'b01001);
            d_is_fencei   <= fault_op ? 1'b0   : s_is_fencei;
            d_is_cbo      <= fault_op ? 1'b0   : s_is_cbo;
            d_cbo_zero    <= s_cbo_zero;
            d_cbo_keep    <= s_cbo_keep;
            d_illegal     <= fault_op ? 1'b0   : s_illegal;
            d_mis_taken   <= (s_taken_pc != f_pnpc);
            d_mis_nt      <= (s_ft_pc    != f_pnpc);
         end
      end
   end
endmodule

`default_nettype wire
