`default_nettype none

// In-order frontend (stage F): PC -> iMMU-translated I$ window -> aligner ->
// RVC expand -> decode, terminating at the IR register that feeds stage X.
//
// This is probe/frontend.v with `decode_rename` replaced by a plain register:
// `fetch`, `predictor` and `decode_slot` are instantiated UNMODIFIED. Width is
// scalar (IW=1), so the aligner's cross-slot machinery collapses to one slot and
// `decode_xslot` is not needed at all.
//
// CHECKPOINTS. ooo2_predictor predicts at the fetch stream and queues its predictions; fetch
// takes the head's details with the bundle that ends at its mark (pd_mk, else pd_no), the IR
// latches them as d_pdet, and they ride the pipeline to M, coming back as res_pdet when the op
// resolves. No checkpoint ring, no tag: the details are matched to their instruction by BEING
// the instruction's payload.

module ooo2_frontend
  #(parameter PCW   = 64,
    parameter SEQW  = 8,
    parameter HW    = 2,             // fetch window halfwords (one 32-bit instruction)
    parameter IW    = 2,             // pipeline width (instructions/cycle); threaded from OOO2_IW (Stage 3)
    parameter RASB  = 3,             // log2 RAS entries (ooo2_predictor)
    parameter GHL   = 11,            // the predictor's history length
    parameter PDW   = 31,            // ooo2_predictor's predict-detail width
    // decoupling queue depth. 2 was the MINIMUM that lets fetch push every cycle (the count just
    // oscillates 1<->2), never an optimum -- which leaves no buffering at all between a
    // frontend and a backend that both cap at one instruction per cycle. FE_QUE measures
    // what that costs: 0.666 CPI on hardware, 2.7x the entire LSU stall.
    parameter QDEPTH = 8,
    parameter QAW    = 3,            // $clog2(QDEPTH)
    parameter [PCW-1:0] RESET_PC = 0)
   (input  wire                    clk,
    input  wire                    reset,

    // ---- back-pressure ----
    // `consume`: stage X handed its instruction to M this cycle, so the IR register
    // is free. `accept`: a new instruction may be loaded into it (consume, and no
    // serializing op in flight). accept implies consume.
    input  wire                    accept,
    input  wire                    consume,           // slot A dispatched (leaves the IR)
    input  wire                    consume_b,         // slot B dispatched (only ever with A)
    input  wire                    consume_c,         // slot C dispatched (only ever with B) -- IW>=3
    input  wire                    two_wide,          // fill slot B at all; low = the one-IR machine
    input  wire                    three_wide,        // fill slot C at all (IW>=3); low = the two-IR machine

    // ---- redirect (from M: mispredict, trap, xret, fence.i) ----
    input  wire                    redirect,
    input  wire [PCW-1:0]          redirect_pc,
    input  wire [SEQW-1:0]         redirect_seq,
    input  wire [RASB-1:0]         redirect_rsp,      // the RAS top the redirect restores
    input  wire [GHL-1:0]          redirect_ghr,      // ...and the history
    input  wire                    irq_inject,        // present the interrupt pseudo-op
    output wire                    irq_taken,         // ...and fetch CONSUMED it this cycle
    output wire                    fe_dq_valid,       // fetch produced an instruction this cycle

    // ---- instruction memory (combinational read, iMMU-translated by the core) ----
    output wire [PCW-1:0]          imem_addr,         // VA to translate
    output wire [PCW-1:0]          imem_ipc,          // PC of the instruction (fault EPC)
    input  wire [PCW-1:0]          imem_pa,           // its PA (the iMMU)
    input  wire [1:0]              imem_xlvl,         // its leaf level: 0 = 4 KiB, else >= 2 MiB
    input  wire                    imem_xlate_ok,     // the iMMU holds a good translation of imem_addr
    input  wire                    imem_freeze,       // fence.i, an I$ invalidation, a mapping change
    output wire [$clog2(HW+2)-1:0] fe_avail,          // the fetch ring's window: halfwords held (counters)
    output wire                    fe_ok,             // ...and the window is the PC's bytes
    // ---- the I$ (rv_icache): the fetch ring's stream ----
    output wire                    ic_req,
    output wire [63:0]             ic_va,
    output wire [63:0]             ic_pa,
    output wire [9:0]              ic_tag,
    input  wire                    ic_ack,
    input  wire                    ic_valid,
    input  wire [127:0]            ic_data,
    input  wire [9:0]              ic_rtag,
    input  wire                    imem_fault,        // iMMU fault on this fetch (qualified ready)
    input  wire [3:0]              imem_cause,

    // ---- branch resolve / training (from M) ----
    input  wire                    res_v,
    input  wire                    res_cbr,
    input  wire                    res_call,
    input  wire                    res_ret,
    input  wire                    res_taken,
    input  wire [PDW-1:0]          res_pdet,        // resolving op's predict details (carried)
    input  wire [PCW-1:0]          res_pc,          // resolving op's own PC...
    input  wire                    res_rvc,         // ...and length (the predictor's key)
    input  wire [PCW-1:0]          res_tgt,

    // ================= IR register: the decoupling-queue boundary =================
    output reg                     d_valid,
    output reg  [PCW-1:0]          d_pc,
    output reg  [31:0]             d_insn,      // RVC-expanded 32-bit form
    output reg                     d_rvc,
    output reg  [SEQW-1:0]         d_seq,
    output reg  [PDW-1:0]          d_pdet,          // this op's predict details -> rides to M
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

    // ---- slot B: the second IR (2026-09-05, item 10b), a mirror of slot A ----
    output reg                     d2_valid,
    output reg  [PCW-1:0]          d2_pc,
    output reg  [31:0]             d2_insn,      // RVC-expanded 32-bit form
    output reg                     d2_rvc,
    output reg  [SEQW-1:0]         d2_seq,
    output reg  [PDW-1:0]          d2_pdet,          // this op's predict details -> rides to M
    output reg  [PCW-1:0]          d2_pred_npc,
    // operands
    output reg  [5:0]              d2_rd, d2_rs1, d2_rs2, d2_rs3,
    output reg                     d2_rd_v, d2_rs1_v, d2_rs2_v, d2_rs3_v,
    output reg  [63:0]             d2_imm,
    // execute control
    output reg  [5:0]              d2_alu_op,
    output reg                     d2_alu_w, d2_alu_uw,
    output reg  [1:0]              d2_op1_sel,
    output reg                     d2_op2_imm, d2_res_link,
    output reg                     d2_is_mem, d2_is_store,
    output reg  [1:0]              d2_mem_size,
    output reg                     d2_mem_signed,
    output reg                     d2_is_branch,
    output reg  [2:0]              d2_br_func,
    output reg                     d2_is_jump, d2_is_jalr,
    output reg                     d2_is_mul, d2_is_csr,
    output reg  [2:0]              d2_csr_func,
    output reg                     d2_is_serialize,
    output reg                     d2_is_amo,
    output reg  [4:0]              d2_amo_func,
    output reg                     d2_is_fp, d2_is_fencei,
    output reg                     d2_is_cbo, d2_cbo_zero, d2_cbo_keep,
    output reg                     d2_illegal,
    // precomputed branch compares (payload-static -> keeps them out of X's cone)
    output reg                     d2_mis_taken, d2_mis_nt,
    // fetch-side fault poison
    output reg                     d2_fault,
    output reg  [3:0]              d2_fault_cause,
    output reg  [PCW-1:0]          d2_fault_tval,
    // ---- slot C (IW>=3): the third IR. Dead at IW=2 (three_wide=0 keeps it empty) ----
    output reg                     d3_valid,
    output reg  [PCW-1:0]          d3_pc,
    output reg  [31:0]             d3_insn,
    output reg                     d3_rvc,
    output reg  [SEQW-1:0]         d3_seq,
    output reg  [PDW-1:0]          d3_pdet,
    output reg  [PCW-1:0]          d3_pred_npc,
    output reg  [5:0]              d3_rd, d3_rs1, d3_rs2, d3_rs3,
    output reg                     d3_rd_v, d3_rs1_v, d3_rs2_v, d3_rs3_v,
    output reg  [63:0]             d3_imm,
    output reg  [5:0]              d3_alu_op,
    output reg                     d3_alu_w, d3_alu_uw,
    output reg  [1:0]              d3_op1_sel,
    output reg                     d3_op2_imm, d3_res_link,
    output reg                     d3_is_mem, d3_is_store,
    output reg  [1:0]              d3_mem_size,
    output reg                     d3_mem_signed,
    output reg                     d3_is_branch,
    output reg  [2:0]              d3_br_func,
    output reg                     d3_is_jump, d3_is_jalr,
    output reg                     d3_is_mul, d3_is_csr,
    output reg  [2:0]              d3_csr_func,
    output reg                     d3_is_serialize,
    output reg                     d3_is_amo,
    output reg  [4:0]              d3_amo_func,
    output reg                     d3_is_fp, d3_is_fencei,
    output reg                     d3_is_cbo, d3_cbo_zero, d3_cbo_keep,
    output reg                     d3_illegal,
    output reg                     d3_mis_taken, d3_mis_nt,
    output reg                     d3_fault,
    output reg  [3:0]              d3_fault_cause,
    output reg  [PCW-1:0]          d3_fault_tval,
    output wire [SEQW-1:0]         cur_seq);          // fetch PC's seqno (trap resume)

   // ---------------------------------------------------------------- fetch
   // TWO-WIDE FETCH (2026-09-05, plan item 10a): the aligner emits up to two instructions per
   // cycle and both enter the queue in one cycle; decode still pops one. The queue then runs
   // ahead of dispatch and absorbs every fetch gap (a redirect's refill, a taken target's
   // arrival) instead of exposing it two cycles later as FE_QUE -- and it is the frontend a
   // two-wide dispatch (10b) needs. A bundle ends at its first CTI, so slot 1 is never
   // followed by anything and slot 0 is never a CTI when slot 1 is valid: the bundle's
   // prediction (pnpc_kind, target, details) belongs to its LAST valid slot and slot 0
   // falls through.
   localparam FW = IW;              // Stage 3: the width knob. The queue, decode and the IR
   // compacting buffer below are width-generic (1..4); rename/exec widen in later increments.
   initial if (FW < 1 || FW > 4) $fatal(1, "ooo2_frontend: IW=%0d out of range 1..4", FW);
   localparam integer FW3 = (FW >= 3) ? 1 : 0;   // sized gates for the third slot/head/push
   localparam integer FW2 = (FW >= 2) ? 1 : 0;
   wire               dq_valid;
   wire [FW-1:0]      dq_sv;                      // per-slot valid
   wire [FW*32-1:0]   dq_inst;
   wire [FW*PCW-1:0]  dq_pc;
   wire [FW*SEQW-1:0] dq_seq;
   wire [PCW-1:0]     bp_tgt;
   wire [1:0]         dq_pk;

   // ------------------------------------------------------------- the fetch ring
   wire                f_pop, f_drop, f_at_mk, f_rej, f_rej_np, bp_tk;
   // A rejected mark restarts the stream a cycle later, as a freeze of one cycle: the ring
   // withholds its window, empties and restarts at the PC, and the predictor goes back to the
   // aligner's state. It is rare, and registering it keeps the aligner out of the stream's next
   // address (the table rows, the history).
   reg                 rej_q, rej_np_q;
   initial begin rej_q = 1'b0; rej_np_q = 1'b0; end
   always @(posedge clk) begin
      rej_q    <= f_rej & ~reset;
      rej_np_q <= f_rej_np;
   end
   wire [$clog2(HW+2)-1:0] f_adv_hw, imem_avail;
   wire [HW*16-1:0]    imem_data;
   wire [HW-1:0]       imem_mk;
   wire [1:0]          imem_lvl;
   wire                imem_ok;
   wire                pr_adv, p_cut, pq_room;
   wire [63:0]         st_sa;
   wire [2:0]          st_sk, p_end;
   localparam integer  PQB = 3;
   wire [PQB:0]        pq_cnt;
   // A restart of the stream: reset, a redirect, or a freeze (fence.i, an I$ invalidation, a
   // mapping change, a rejected mark). It begins at the redirect's target, else at the PC.
   wire                st_flush = reset | redirect | imem_freeze | rej_q;
   wire [63:0]         st_rst_a = redirect ? redirect_pc : imem_addr;
   ooo2_fring #(.HW(HW), .PQB(PQB)) u_ring
     (.clk(clk), .reset(reset), .freeze(imem_freeze | rej_q), .restart(reset | redirect),
      .adv_hw(f_adv_hw), .pop(f_pop), .drop(f_drop), .pq_tk(bp_tk), .pq_tgt(bp_tgt),
      .pc_va(imem_addr), .rst_a(st_rst_a), .pc_pa(imem_pa), .pc_lvl(imem_xlvl), .xlate_ok(imem_xlate_ok),
      .win(imem_data), .mk(imem_mk), .avail(imem_avail), .ok(imem_ok), .lvl(imem_lvl),
      .sa(st_sa), .sk(st_sk), .p_cut(p_cut), .p_end(p_end), .pr_adv(pr_adv),
      .pq_room(pq_room), .pq_cnt(pq_cnt),
      .ic_req(ic_req), .ic_va(ic_va), .ic_pa(ic_pa), .ic_tag(ic_tag),
      .ic_ack(ic_ack), .ic_valid(ic_valid), .ic_data(ic_data), .ic_rtag(ic_rtag));
   assign fe_avail = imem_avail;
   assign fe_ok    = imem_ok;

   // No checkpoint ring, no `cur`, no `create`, no rb_idx. ooo2_predictor keeps committed
   // scalars instead, and this bundle's predict details ride WITH it as d_pdet -- so there
   // is no tag to allocate and nothing to pin against reuse.

   fetch #(.IW(FW), .HW(HW), .PCW(PCW), .SEQW(SEQW), .RESET_PC(RESET_PC)) u_fetch
     (.clk(clk), .reset(reset),
      .redirect(redirect), .redirect_pc(redirect_pc), .redirect_seq(redirect_seq),
      .solo_all(1'b0),                 // the aligner already cuts bundles at CTIs and SYSTEM ops
      .irq_inject(irq_inject), .irq_pres(irq_pres),
      .imem_mk(imem_mk), .pq_tk(bp_tk), .pq_tgt(bp_tgt),
      .pop(f_pop), .drop(f_drop), .at_mark(f_at_mk), .reject(f_rej), .reject_np(f_rej_np),
      // `pred_npc` (the chosen next PC, with its adder) is left unconnected: the queue
      // stores the CHOICE (pnpc_kind) and the target, and decode rebuilds the value from
      // the length it decodes anyway. See the queue below.
      .pred_npc(), .pnpc_kind(dq_pk), .adv_hw(f_adv_hw),
      .imem_addr(imem_addr), .imem_ipc(imem_ipc), .imem_data(imem_data),
      .imem_avail(imem_avail), .imem_lvl(imem_lvl), .imem_ok(imem_ok),
      .ready(pb_ready), .valid(dq_valid),
      .slot_valid(dq_sv), .inst(dq_inst), .pc(dq_pc), .seq(dq_seq), .cur_seq(cur_seq));

   // THE BUNDLE IS REGISTERED BEFORE THE QUEUE WRITE (plan item T1 (F), step 2, 2026-09-06).
   // With the fetch buffer's compare out of the data path (step 1), the decoupling queue's 826
   // LUTRAM data pins were still the largest near-critical family (3,673 endpoints from
   // irq_inject_q, 20 levels, 4 ns of route): the aligner, the interrupt/straddle bundle
   // muxes and the slot-PC selects all end on those pins, spread across the chip. The
   // queue's write data is now a register, and the fetch handshake lands in it: `fire`
   // (the predictor's training edge, the interrupt pseudo-op's consumption, fetch's PC
   // advance) is the register's load. A bundle in the register dies with the queue on a
   // redirect -- it is younger than the redirecting op by the same argument (rule I11).
   // Cost: one cycle of fetch-to-decode latency, visible only when the queue is empty.
   reg            pb_v;
   wire           pb_ready = ~pb_v | q_room;             // empty, or draining into the queue now
   wire fire = pb_ready & dq_valid;    // fetch handshake: a bundle enters the register

   // The interrupt pseudo-op is consumed by fetch HERE, on the queue push -- not by
   // `accept`, which is the queue POP. Those were the same edge until this module grew a
   // queue (`.ready(accept)` -> `.ready(~q_full)`), and ooo2_core's inject_inflight
   // interlock was left keyed to the old one. Report the real event so the interlock can
   // name it: while the backend stalls, `accept` is low but `fire` is not, so fetch
   // re-emitted the SAME interrupt every cycle with nothing to stop it.
   wire   irq_pres;                    // fetch presents the pseudo-op (a straddle finishes first)
   assign irq_taken = irq_pres & fire;

   // Sub-attribution for the frontend bubble (see ooo2_core's fe_aln/fe_que).  dq_valid is
   // "fetch assembled a complete instruction this cycle".  With it, a bubble that is not
   // the iMMU and not an empty fetch window splits into two very different problems:
   // fetch had BYTES but could not make an instruction (aligner/straddle), versus fetch
   // made one and the queue simply had nothing to hand over (refill latency).
   assign fe_dq_valid = dq_valid;

   // ------------------------------------------------------- branch predictor
   wire [PDW-1:0] pd_mk, pd_no;
   wire [PDW-1:0] pd_fetch = f_at_mk ? pd_mk : pd_no;   // one bundle's details, all its slots
   ooo2_predictor #(.PCW(PCW), .PDW(PDW), .RASB(RASB), .GHL(GHL), .PQB(PQB)) u_bp
     (.clk(clk), .reset(reset),
      .flush(st_flush), .rst_a(st_rst_a), .rst_np(rej_np_q & ~redirect), .adv(pr_adv),
      .sa(st_sa), .sk(st_sk), .p_cut(p_cut), .p_end(p_end), .pq_room(pq_room), .pq_cnt(pq_cnt),
      .pq_tk(bp_tk), .pq_tgt(bp_tgt), .pd_mk(pd_mk), .pd_no(pd_no), .pop(f_pop),
      .rollback(redirect), .rb_rsp(redirect_rsp), .rb_ghr(redirect_ghr),
      .res_v(res_v), .res_cbr(res_cbr), .res_call(res_call), .res_ret(res_ret),
      .res_taken(res_taken), .res_pdet(res_pdet), .res_tgt(res_tgt), .res_pc(res_pc),
      .res_rvc(res_rvc));

   // ------------------------------------------------------------- decoupling queue
   // THE point of this module's shape. fetch's .ready() used to be `accept`, which is
   // m_advance, which is lsu_done -- so the fetch pointer could not move without knowing
   // whether M completed this cycle, and that put the whole memory pipeline in the
   // frontend's timing cone:
   //   u_lsu FSM -> ... -> lsu_done -> u_fetch/va_q -> iMMU tag compare -> u_bp/ycorr_qv
   // was the 132-path family at a 6 ns constraint. .ready() is now ~q_full: a function of
   // a counter, with nothing from the backend in it.
   //
   // Depth 2, not 1: with one entry a clean ready (~q_valid, no `accept` term) drains and
   // refills on alternate cycles, halving throughput. With two the count oscillates 1<->2
   // and fetch pushes every cycle.
   // THE QUEUE STORES THE PREDICTION'S CHOICE, NOT THE SUM. fetch's pred_npc is
   // pc + 2*consumed unless the predictor steered, and `consumed` is the aligner's output:
   // iMMU -> fetch buffer -> aligner -> a 64-bit adder -> this array's data pin was 342
   // endpoints at -0.059 on 2026-09-03 (25 levels, 13 CARRY8). Decode computes the same
   // fall-through from the length it decodes (s_ft_pc), so the queue carries a 2-bit
   // selector and the predictor's target instead, and f_pnpc is rebuilt at the head with
   // one mux. Same value on every bundle, including the straddle (+4 is its 32-bit
   // length) and the interrupt pseudo-op (which holds its PC).
   localparam QW = PDW + PCW + 32 + SEQW + 2 + PCW + 1 + 4 + PCW;
   wire           dq_fault = imem_fault & ~dq_valid;    // fetch-fault pseudo-op, pushed like a bundle
   wire           pb_load  = pb_ready & (dq_valid | dq_fault);   // the register takes a bundle or the fault op
   // UP TO IW ENTRIES PER CYCLE INTO A LUTRAM (Stage 3 inc 2, generalising the old two parity
   // banks): QNB = next_pow2(FW) consecutive-write banks, bank = pos[QLB-1:0], index = pos>>QLB,
   // ONE muxed write per bank (rule: one write statement per LUTRAM bank). `q_room` asks for two
   // free entries so a bundle never splits. At FW=2 this is QNB=2 == the old q_dat0/q_dat1.
   localparam integer QNB = 1 << $clog2(FW);
   localparam integer QLB = $clog2(QNB);
   localparam integer QBD = QDEPTH / QNB;
   wire [QW-1:0] q_bank_rd [0:QNB-1];
   reg  [QAW-1:0] q_rp, q_wp;
   reg  [QAW:0]   q_cnt;
   wire           q_room  = (q_cnt <= QDEPTH[QAW:0] - FW[QAW:0]);   // room for a full bundle (<=FW)
   wire           q_empty = (q_cnt == {(QAW+1){1'b0}});
   reg  [QW-1:0]  pb_in0, pb_in1, pb_in2;
   reg            pb_two, pb_three;
   wire           q_push  = q_room & pb_v;
   wire           q_two   = q_push & pb_two;              // the bundle has a second instruction
   wire           q_three = q_push & pb_three;            // ...and a third (IW>=3)
   wire           q_have2 = (q_cnt >= 2);
   wire           q_have3 = (q_cnt >= 3);
   // slot-2 sources, part-selected only when the bundle can hold three (FW>=3); zero otherwise,
   // so the FW=2 build never indexes dq_* out of range.
   wire [31:0]    dq_inst2;  wire [PCW-1:0] dq_pc2;  wire [SEQW-1:0] dq_seq2;  wire dq_sv2;
   generate if (FW >= 3) begin: g_slot2
      assign dq_inst2 = dq_inst[2*32   +: 32];
      assign dq_pc2   = dq_pc  [2*PCW  +: PCW];
      assign dq_seq2  = dq_seq [2*SEQW +: SEQW];
      assign dq_sv2   = dq_sv[FW>=3 ? 2 : 0];
   end else begin: g_noslot2
      assign dq_inst2 = 32'd0;  assign dq_pc2 = {PCW{1'b0}};  assign dq_seq2 = {SEQW{1'b0}};  assign dq_sv2 = 1'b0;
   end endgenerate
   // slot 0 falls through when a later slot exists; the prediction rides with the bundle's LAST
   // valid slot. Contiguity: dq_sv[1] already covers "a slot 2 follows too", so q_in0 is
   // unchanged from the two-wide form.
   wire [QW-1:0]  q_in0   = {pd_fetch, (dq_fault ? imem_ipc : dq_pc[0 +: PCW]), dq_inst[0 +: 32], dq_seq[0 +: SEQW],
                             (dq_sv[1] ? 2'd0 : dq_pk), bp_tgt, dq_fault, imem_cause, imem_addr};
   // slot 1: falls through if a slot 2 follows (IW>=3), else carries the bundle prediction.
   wire [QW-1:0]  q_in1   = {pd_fetch, dq_pc[PCW +: PCW], dq_inst[32 +: 32], dq_seq[SEQW +: SEQW],
                             (dq_sv2 ? 2'd0 : dq_pk), bp_tgt, 1'b0, imem_cause, imem_addr};
   // slot 2: the bundle's last slot when present, so it carries the prediction.
   wire [QW-1:0]  q_in2   = {pd_fetch, dq_pc2, dq_inst2, dq_seq2,
                             dq_pk, bp_tgt, 1'b0, imem_cause, imem_addr};
   wire [QAW-1:0] q_wp1   = q_wp + 1'b1;
   wire [QAW-1:0] q_wp2   = q_wp + 2'd2;
   always @(posedge clk) begin
      if (reset | redirect)  pb_v <= 1'b0;
      else if (pb_load)       pb_v <= 1'b1;
      else if (q_push)       pb_v <= 1'b0;
      if (pb_load) begin
         pb_in0 <= q_in0; pb_in1 <= q_in1; pb_in2 <= q_in2;
         pb_two <= dq_sv[1] & dq_valid; pb_three <= dq_sv2 & dq_valid;
      end
   end
   // The heads keep the ORIGINAL names, so decode and the IR register below are unchanged.
   wire [PDW-1:0] q_pdet;   wire [PCW-1:0] f_pc;   wire [31:0] f_inst;
   wire [SEQW-1:0] f_seq;   wire [1:0] f_pk;  wire [PCW-1:0] f_tgt;  wire fault_op;
   wire [3:0]     q_cause;  wire [PCW-1:0] q_tval;
   wire [QAW-1:0] q_rp1   = q_rp + 1'b1;
   wire [QAW-1:0] q_rp2   = q_rp + 2'd2;
   // ---- N-bank queue storage: one muxed write per bank, one muxed read per head ----
   genvar qgb;
   generate for (qgb = 0; qgb < QNB; qgb = qgb + 1) begin: qbank
      reg [QW-1:0] mem [0:QBD-1];
      integer qbi; initial for (qbi = 0; qbi < QBD; qbi = qbi + 1) mem[qbi] = {QW{1'b0}};
      wire wsel0 = q_push  & (q_wp [QLB-1:0] == qgb);
      wire wsel1 = q_two   & (q_wp1[QLB-1:0] == qgb);
      wire wsel2 = q_three & (q_wp2[QLB-1:0] == qgb);   // dead at FW<3 (q_three==0)
      always @(posedge clk)
         if (wsel0 | wsel1 | wsel2)
            mem[wsel0 ? q_wp[QAW-1:QLB] : wsel1 ? q_wp1[QAW-1:QLB] : q_wp2[QAW-1:QLB]]
               <= wsel0 ? pb_in0 : wsel1 ? pb_in1 : pb_in2;
      // read: rp / rp+1 / rp+2. At QNB=2 the rp2 arm is unreachable (rp/rp+1 cover both banks)
      // -> synth drops it, so FW=2 is bit-identical to the two-head form.
      wire rsel0 = (q_rp [QLB-1:0] == qgb);
      wire rsel1 = (q_rp1[QLB-1:0] == qgb);
      assign q_bank_rd[qgb] = mem[rsel0 ? q_rp [QAW-1:QLB]
                                : rsel1 ? q_rp1[QAW-1:QLB]
                                :         q_rp2[QAW-1:QLB]];
   end endgenerate
   wire [QW-1:0]  q_head  = q_bank_rd[q_rp [QLB-1:0]];
   wire [QW-1:0]  q_head1 = q_bank_rd[q_rp1[QLB-1:0]];
   wire [QW-1:0]  q_head2 = q_bank_rd[q_rp2[QLB-1:0]];
   assign {q_pdet, f_pc, f_inst, f_seq, f_pk, f_tgt, fault_op, q_cause, q_tval} = q_head;
   wire [PDW-1:0] q1_pdet;  wire [PCW-1:0] f1_pc;  wire [31:0] f1_inst;
   wire [SEQW-1:0] f1_seq;  wire [1:0] f1_pk;  wire [PCW-1:0] f1_tgt;  wire fault1_op;
   wire [3:0]     q1_cause; wire [PCW-1:0] q1_tval;
   assign {q1_pdet, f1_pc, f1_inst, f1_seq, f1_pk, f1_tgt, fault1_op, q1_cause, q1_tval} = q_head1;
   wire [PDW-1:0] q2_pdet;  wire [PCW-1:0] f2_pc;  wire [31:0] f2_inst;
   wire [SEQW-1:0] f2_seq;  wire [1:0] f2_pk;  wire [PCW-1:0] f2_tgt;  wire fault2_op;
   wire [3:0]     q2_cause; wire [PCW-1:0] q2_tval;
   assign {q2_pdet, f2_pc, f2_inst, f2_seq, f2_pk, f2_tgt, fault2_op, q2_cause, q2_tval} = q_head2;
   wire [1:0]     q_pop;                       // 0..3 entries leave this cycle (see the IR)
   wire [1:0]     q_pushn = {1'b0, q_push} + {1'b0, q_two} + {1'b0, q_three};   // 0..3 pushed

   always @(posedge clk) begin
      if (reset | redirect) begin
         q_cnt <= {(QAW+1){1'b0}}; q_rp <= {QAW{1'b0}}; q_wp <= {QAW{1'b0}};
      end else begin
         q_wp  <= q_wp + {{(QAW-2){1'b0}}, q_pushn};
         q_rp  <= q_rp + {{(QAW-2){1'b0}}, q_pop};
         q_cnt <= q_cnt + {{(QAW-1){1'b0}}, q_pushn} - {{(QAW-1){1'b0}}, q_pop};
      end
   end

   // A slot without its predecessor, or a fault pseudo-op beside a real slot, is a fetch defect.
   always @(posedge clk)
      if (!reset && ((dq_sv[1] & ~dq_sv[0]) || (dq_sv2 & ~dq_sv[1]) || (dq_fault & dq_valid)))
         $fatal(1, "ooo2_frontend: malformed bundle sv=%b sv2=%b fault=%b", dq_sv, dq_sv2, dq_fault);

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
   wire        s1_rvc, s1_rd_v, s1_rs1_v, s1_rs2_v, s1_rs3_v, s1_legal;
   wire [31:0] s1_exp;
   wire [5:0]  s1_rd, s1_rs1, s1_rs2, s1_rs3;
   wire [63:0] s1_imm;
   wire        s1_has_imm;
   wire [5:0]  s1_alu_op;
   wire        s1_alu_w, s1_alu_uw, s1_op2_imm, s1_res_link;
   wire [1:0]  s1_op1_sel;
   wire        s1_is_mem, s1_is_store, s1_mem_signed, s1_is_branch, s1_is_jump;
   wire [1:0]  s1_mem_size;
   wire [2:0]  s1_br_func, s1_csr_func;
   wire        s1_is_mul, s1_is_csr, s1_is_serialize, s1_is_amo, s1_is_fencei;
   wire [4:0]  s1_amo_func;
   wire        s1_is_cbo, s1_cbo_zero, s1_cbo_keep, s1_illegal;

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
   // fetch's pred_npc, rebuilt from the queued choice (see the queue above)
   wire [PCW-1:0] f_pnpc     = (f_pk == 2'd2) ? f_pc : (f_pk == 2'd1) ? f_tgt : s_ft_pc;

   // ---- the second head, decoded the same way ----
   decode_slot #(.SEQW(SEQW)) u_dec1
     (.inst(f1_inst), .in_valid(1'b1), .seq_in(f1_seq),
      .valid(), .seq(), .is_rvc(s1_rvc), .expanded(s1_exp),
      .rd(s1_rd), .rd_v(s1_rd_v), .rs1(s1_rs1), .rs1_v(s1_rs1_v),
      .rs2(s1_rs2), .rs2_v(s1_rs2_v), .rs3(s1_rs3), .rs3_v(s1_rs3_v),
      .imm(s1_imm), .has_imm(s1_has_imm), .legal(s1_legal),
      .alu_op(s1_alu_op), .alu_w(s1_alu_w), .alu_uw(s1_alu_uw), .op1_sel(s1_op1_sel),
      .op2_imm(s1_op2_imm), .res_link(s1_res_link), .is_mem(s1_is_mem),
      .is_store(s1_is_store), .mem_size(s1_mem_size), .mem_signed(s1_mem_signed),
      .is_branch(s1_is_branch), .br_func(s1_br_func), .is_jump(s1_is_jump),
      .is_mul(s1_is_mul), .is_csr(s1_is_csr), .csr_func(s1_csr_func),
      .is_serialize(s1_is_serialize), .is_amo(s1_is_amo), .amo_func(s1_amo_func),
      .is_fencei(s1_is_fencei), .is_cbo(s1_is_cbo), .cbo_zero(s1_cbo_zero),
      .cbo_keep(s1_cbo_keep), .illegal(s1_illegal));
   wire s1_is_jalr = (s1_exp[6:2] == 5'b11001);
   wire [PCW-1:0] s1_taken_pc = f1_pc + s1_imm;
   wire [PCW-1:0] s1_ft_pc    = f1_pc + (s1_rvc ? 64'd2 : 64'd4);
   wire [PCW-1:0] f1_pnpc     = (f1_pk == 2'd2) ? f1_pc : (f1_pk == 2'd1) ? f1_tgt : s1_ft_pc;

   // ---- the third head, decoded the same way (IW>=3; at IW=2 q_head2 is never valid) ----
   wire        s2_rvc, s2_rd_v, s2_rs1_v, s2_rs2_v, s2_rs3_v, s2_legal;
   wire [31:0] s2_exp;
   wire [5:0]  s2_rd, s2_rs1, s2_rs2, s2_rs3;
   wire [63:0] s2_imm;
   wire        s2_has_imm;
   wire [5:0]  s2_alu_op;
   wire        s2_alu_w, s2_alu_uw, s2_op2_imm, s2_res_link;
   wire [1:0]  s2_op1_sel;
   wire        s2_is_mem, s2_is_store, s2_mem_signed, s2_is_branch, s2_is_jump;
   wire [1:0]  s2_mem_size;
   wire [2:0]  s2_br_func, s2_csr_func;
   wire        s2_is_mul, s2_is_csr, s2_is_serialize, s2_is_amo, s2_is_fencei;
   wire [4:0]  s2_amo_func;
   wire        s2_is_cbo, s2_cbo_zero, s2_cbo_keep, s2_illegal;
   decode_slot #(.SEQW(SEQW)) u_dec2
     (.inst(f2_inst), .in_valid(1'b1), .seq_in(f2_seq),
      .valid(), .seq(), .is_rvc(s2_rvc), .expanded(s2_exp),
      .rd(s2_rd), .rd_v(s2_rd_v), .rs1(s2_rs1), .rs1_v(s2_rs1_v),
      .rs2(s2_rs2), .rs2_v(s2_rs2_v), .rs3(s2_rs3), .rs3_v(s2_rs3_v),
      .imm(s2_imm), .has_imm(s2_has_imm), .legal(s2_legal),
      .alu_op(s2_alu_op), .alu_w(s2_alu_w), .alu_uw(s2_alu_uw), .op1_sel(s2_op1_sel),
      .op2_imm(s2_op2_imm), .res_link(s2_res_link), .is_mem(s2_is_mem),
      .is_store(s2_is_store), .mem_size(s2_mem_size), .mem_signed(s2_mem_signed),
      .is_branch(s2_is_branch), .br_func(s2_br_func), .is_jump(s2_is_jump),
      .is_mul(s2_is_mul), .is_csr(s2_is_csr), .csr_func(s2_csr_func),
      .is_serialize(s2_is_serialize), .is_amo(s2_is_amo), .amo_func(s2_amo_func),
      .is_fencei(s2_is_fencei), .is_cbo(s2_is_cbo), .cbo_zero(s2_cbo_zero),
      .cbo_keep(s2_cbo_keep), .illegal(s2_illegal));
   wire s2_is_jalr = (s2_exp[6:2] == 5'b11001);
   wire [PCW-1:0] s2_taken_pc = f2_pc + s2_imm;
   wire [PCW-1:0] s2_ft_pc    = f2_pc + (s2_rvc ? 64'd2 : 64'd4);
   wire [PCW-1:0] f2_pnpc     = (f2_pk == 2'd2) ? f2_pc : (f2_pk == 2'd1) ? f2_tgt : s2_ft_pc;

   // fetch-fault pseudo-op: presented only when fetch itself has nothing (an
   // interrupt injection wins -- fetch forces a valid bundle for it, and an
   // interrupt is taken before the instruction that would have faulted).
   wire ld_valid = ~q_empty;      // the head is a real bundle (fault pseudo-op included)

   // ------------------------------------------------------- IR registers: a width-generic
   // COMPACTING BUFFER (Stage 3). Up to W = 1+two_wide+three_wide decoded IRs live in slots
   // [0..W-1], slot 0 oldest. Each cycle dispatch consumes the oldest `ncons` (contiguous from
   // slot 0), the survivors compact down, and fresh decoded heads fill from the queue up to W.
   // At W=2 this reduces bit-for-bit to the old slot-A/slot-B shift (verified retire-identical).
   //
   // One PACKED vector per slot keeps the compaction a handful of muxes instead of ~46 per
   // field. FE_IRV(P) is the field list -- the SINGLE source of truth for pack AND unpack order
   // (cur[], mh[], and the register write below all use it), so the two can never diverge.
   `define FE_IRV(P) {P``pdet, P``pc, P``fault_tval, P``is_fp, P``seq, P``pred_npc, P``fault, P``fault_cause, P``insn, P``rvc, P``rd, P``rd_v, P``rs1, P``rs1_v, P``rs2, P``rs2_v, P``rs3, P``rs3_v, P``imm, P``alu_op, P``alu_w, P``alu_uw, P``op1_sel, P``op2_imm, P``res_link, P``is_mem, P``is_store, P``mem_size, P``mem_signed, P``is_branch, P``br_func, P``is_jump, P``is_jalr, P``is_mul, P``is_csr, P``csr_func, P``is_serialize, P``is_amo, P``amo_func, P``is_fencei, P``is_cbo, P``cbo_zero, P``cbo_keep, P``illegal, P``mis_taken, P``mis_nt}
   localparam integer IRW = 3*PCW + PDW + SEQW + 173;   // width of one packed IR slot
   wire [IRW-1:0] cur [0:3];   // current slot contents, packed (index 3 = 0 guard)
   wire [IRW-1:0] mh  [0:3];   // fresh decoded heads, packed
   assign cur[0] = `FE_IRV(d_);
   assign cur[1] = `FE_IRV(d2_);
   assign cur[2] = `FE_IRV(d3_);
   assign cur[3] = {IRW{1'b0}};
   assign mh[0]  = `FE_IRV(m0_);
   assign mh[1]  = `FE_IRV(m1_);
   assign mh[2]  = `FE_IRV(m2_);
   assign mh[3]  = {IRW{1'b0}};
   wire [1:0] cur_cnt = {1'b0, d_valid} + {1'b0, d2_valid} + {1'b0, d3_valid};   // 0..3 valid slots
   wire [1:0] ncons   = {1'b0, consume} + {1'b0, consume_b} + {1'b0, consume_c}; // 0..3 consumed (contiguous)
   wire [1:0] retain  = cur_cnt - ncons;                                          // survivors
   wire [1:0] wmax    = 2'd1 + {1'b0, two_wide} + {1'b0, three_wide};             // max slots to fill
   wire [3:0] hav     = {1'b0, q_have3, q_have2, ~q_empty};                       // head j available; [3]=0 guard
   // per slot i: retained survivor cur[ncons+i] if i<retain, else fresh head mh[i-retain]
   wire       fr0 = (2'd0 < retain);  wire [1:0] hj0 = 2'd0 - retain;  wire th0 = (2'd0 < wmax) & hav[hj0];
   wire       fr1 = (2'd1 < retain);  wire [1:0] hj1 = 2'd1 - retain;  wire th1 = (2'd1 < wmax) & hav[hj1];
   wire       fr2 = (2'd2 < retain);  wire [1:0] hj2 = 2'd2 - retain;  wire th2 = (2'd2 < wmax) & hav[hj2];
   wire [IRW-1:0] snext0 = fr0 ? cur[ncons + 2'd0] : mh[hj0];
   wire [IRW-1:0] snext1 = fr1 ? cur[ncons + 2'd1] : mh[hj1];
   wire [IRW-1:0] snext2 = fr2 ? cur[ncons + 2'd2] : mh[hj2];
   wire vnext0 = fr0 ? 1'b1 : th0;
   wire vnext1 = fr1 ? 1'b1 : th1;
   wire vnext2 = fr2 ? 1'b1 : th2;
   // heads actually consumed this cycle = the new-head slots that became valid
   assign q_pop = {1'b0, vnext0 & ~fr0} + {1'b0, vnext1 & ~fr1} + {1'b0, vnext2 & ~fr2};
   always @(posedge clk) if (!reset && ((consume_b && !consume) || (consume_c && !consume_b)))
      $fatal(1, "ooo2_frontend: slot consumed out of order (c=%b b=%b a=%b)", consume_c, consume_b, consume);
   // the two heads, decoded and fault-masked, as slot contents
   wire [PDW-1:0] m0_pdet = q_pdet;                 wire [PDW-1:0] m1_pdet = q1_pdet;
   wire [PCW-1:0] m0_pc   = f_pc;                   wire [PCW-1:0] m1_pc   = f1_pc;
   wire [PCW-1:0] m0_fault_tval = q_tval;           wire [PCW-1:0] m1_fault_tval = q1_tval;   // faulting VA (straddle: pc+2)
   wire m0_is_fp = fault_op  ? 1'b0 : ((s_exp[6:2] == 5'b10100) | (s_exp[6:2] == 5'b10000) | (s_exp[6:2] == 5'b10001) | (s_exp[6:2] == 5'b10010) | (s_exp[6:2] == 5'b10011) | (s_exp[6:2] == 5'b00001) | (s_exp[6:2] == 5'b01001));
   wire m1_is_fp = fault1_op ? 1'b0 : ((s1_exp[6:2] == 5'b10100) | (s1_exp[6:2] == 5'b10000) | (s1_exp[6:2] == 5'b10001) | (s1_exp[6:2] == 5'b10010) | (s1_exp[6:2] == 5'b10011) | (s1_exp[6:2] == 5'b00001) | (s1_exp[6:2] == 5'b01001));
   wire [SEQW-1:0] m0_seq = f_seq;
   wire [SEQW-1:0] m1_seq = f1_seq;
   wire [PCW-1:0] m0_pred_npc = f_pnpc;
   wire [PCW-1:0] m1_pred_npc = f1_pnpc;
   wire  m0_fault = fault_op;
   wire  m1_fault = fault1_op;
   wire [3:0] m0_fault_cause = q_cause;
   wire [3:0] m1_fault_cause = q1_cause;
   wire [31:0] m0_insn = fault_op ? 32'd0  : s_exp;
   wire [31:0] m1_insn = fault1_op ? 32'd0  : s1_exp;
   wire  m0_rvc = fault_op ? 1'b0   : s_rvc;
   wire  m1_rvc = fault1_op ? 1'b0   : s1_rvc;
   wire [5:0] m0_rd = fault_op ? 6'd0   : s_rd;
   wire [5:0] m1_rd = fault1_op ? 6'd0   : s1_rd;
   wire  m0_rd_v = fault_op ? 1'b0   : s_rd_v;
   wire  m1_rd_v = fault1_op ? 1'b0   : s1_rd_v;
   wire [5:0] m0_rs1 = fault_op ? 6'd0   : s_rs1;
   wire [5:0] m1_rs1 = fault1_op ? 6'd0   : s1_rs1;
   wire  m0_rs1_v = fault_op ? 1'b0   : s_rs1_v;
   wire  m1_rs1_v = fault1_op ? 1'b0   : s1_rs1_v;
   wire [5:0] m0_rs2 = fault_op ? 6'd0   : s_rs2;
   wire [5:0] m1_rs2 = fault1_op ? 6'd0   : s1_rs2;
   wire  m0_rs2_v = fault_op ? 1'b0   : s_rs2_v;
   wire  m1_rs2_v = fault1_op ? 1'b0   : s1_rs2_v;
   wire [5:0] m0_rs3 = fault_op ? 6'd0   : s_rs3;
   wire [5:0] m1_rs3 = fault1_op ? 6'd0   : s1_rs3;
   wire  m0_rs3_v = fault_op ? 1'b0   : s_rs3_v;
   wire  m1_rs3_v = fault1_op ? 1'b0   : s1_rs3_v;
   wire [63:0] m0_imm = fault_op ? 64'd0  : s_imm;
   wire [63:0] m1_imm = fault1_op ? 64'd0  : s1_imm;
   wire [5:0] m0_alu_op = s_alu_op;
   wire [5:0] m1_alu_op = s1_alu_op;
   wire  m0_alu_w = fault_op ? 1'b0   : s_alu_w;
   wire  m1_alu_w = fault1_op ? 1'b0   : s1_alu_w;
   wire  m0_alu_uw = fault_op ? 1'b0   : s_alu_uw;
   wire  m1_alu_uw = fault1_op ? 1'b0   : s1_alu_uw;
   wire [1:0] m0_op1_sel = fault_op ? 2'd0   : s_op1_sel;
   wire [1:0] m1_op1_sel = fault1_op ? 2'd0   : s1_op1_sel;
   wire  m0_op2_imm = fault_op ? 1'b0   : s_op2_imm;
   wire  m1_op2_imm = fault1_op ? 1'b0   : s1_op2_imm;
   wire  m0_res_link = fault_op ? 1'b0   : s_res_link;
   wire  m1_res_link = fault1_op ? 1'b0   : s1_res_link;
   wire  m0_is_mem = fault_op ? 1'b0   : s_is_mem;
   wire  m1_is_mem = fault1_op ? 1'b0   : s1_is_mem;
   wire  m0_is_store = fault_op ? 1'b0   : s_is_store;
   wire  m1_is_store = fault1_op ? 1'b0   : s1_is_store;
   wire [1:0] m0_mem_size = s_mem_size;
   wire [1:0] m1_mem_size = s1_mem_size;
   wire  m0_mem_signed = s_mem_signed;
   wire  m1_mem_signed = s1_mem_signed;
   wire  m0_is_branch = fault_op ? 1'b0   : s_is_branch;
   wire  m1_is_branch = fault1_op ? 1'b0   : s1_is_branch;
   wire [2:0] m0_br_func = s_br_func;
   wire [2:0] m1_br_func = s1_br_func;
   wire  m0_is_jump = fault_op ? 1'b0   : s_is_jump;
   wire  m1_is_jump = fault1_op ? 1'b0   : s1_is_jump;
   wire  m0_is_jalr = fault_op ? 1'b0   : s_is_jalr;
   wire  m1_is_jalr = fault1_op ? 1'b0   : s1_is_jalr;
   wire  m0_is_mul = fault_op ? 1'b0   : s_is_mul;
   wire  m1_is_mul = fault1_op ? 1'b0   : s1_is_mul;
   wire  m0_is_csr = fault_op ? 1'b0   : s_is_csr;
   wire  m1_is_csr = fault1_op ? 1'b0   : s1_is_csr;
   wire [2:0] m0_csr_func = s_csr_func;
   wire [2:0] m1_csr_func = s1_csr_func;
   wire  m0_is_serialize = fault_op ? 1'b1   : s_is_serialize;
   wire  m1_is_serialize = fault1_op ? 1'b1   : s1_is_serialize;
   wire  m0_is_amo = fault_op ? 1'b0   : s_is_amo;
   wire  m1_is_amo = fault1_op ? 1'b0   : s1_is_amo;
   wire [4:0] m0_amo_func = s_amo_func;
   wire [4:0] m1_amo_func = s1_amo_func;
   wire  m0_is_fencei = fault_op ? 1'b0   : s_is_fencei;
   wire  m1_is_fencei = fault1_op ? 1'b0   : s1_is_fencei;
   wire  m0_is_cbo = fault_op ? 1'b0   : s_is_cbo;
   wire  m1_is_cbo = fault1_op ? 1'b0   : s1_is_cbo;
   wire  m0_cbo_zero = s_cbo_zero;
   wire  m1_cbo_zero = s1_cbo_zero;
   wire  m0_cbo_keep = s_cbo_keep;
   wire  m1_cbo_keep = s1_cbo_keep;
   wire  m0_illegal = fault_op ? 1'b0   : s_illegal;
   wire  m1_illegal = fault1_op ? 1'b0   : s1_illegal;
   wire  m0_mis_taken = (s_taken_pc != f_pnpc);
   wire  m1_mis_taken = (s1_taken_pc != f1_pnpc);
   wire  m0_mis_nt = (s_ft_pc    != f_pnpc);
   wire  m1_mis_nt = (s1_ft_pc    != f1_pnpc);
   // ---- slot 2 masked head (IW>=3); q_head2 is never valid at IW=2 so these are dead ----
   wire [PDW-1:0] m2_pdet = q2_pdet;
   wire [PCW-1:0] m2_pc   = f2_pc;
   wire [PCW-1:0] m2_fault_tval = q2_tval;
   wire m2_is_fp = fault2_op ? 1'b0 : ((s2_exp[6:2] == 5'b10100) | (s2_exp[6:2] == 5'b10000) | (s2_exp[6:2] == 5'b10001) | (s2_exp[6:2] == 5'b10010) | (s2_exp[6:2] == 5'b10011) | (s2_exp[6:2] == 5'b00001) | (s2_exp[6:2] == 5'b01001));
   wire [SEQW-1:0] m2_seq = f2_seq;
   wire [PCW-1:0] m2_pred_npc = f2_pnpc;
   wire  m2_fault = fault2_op;
   wire [3:0] m2_fault_cause = q2_cause;
   wire [31:0] m2_insn = fault2_op ? 32'd0 : s2_exp;
   wire  m2_rvc = fault2_op ? 1'b0 : s2_rvc;
   wire [5:0] m2_rd = fault2_op ? 6'd0 : s2_rd;
   wire  m2_rd_v = fault2_op ? 1'b0 : s2_rd_v;
   wire [5:0] m2_rs1 = fault2_op ? 6'd0 : s2_rs1;
   wire  m2_rs1_v = fault2_op ? 1'b0 : s2_rs1_v;
   wire [5:0] m2_rs2 = fault2_op ? 6'd0 : s2_rs2;
   wire  m2_rs2_v = fault2_op ? 1'b0 : s2_rs2_v;
   wire [5:0] m2_rs3 = fault2_op ? 6'd0 : s2_rs3;
   wire  m2_rs3_v = fault2_op ? 1'b0 : s2_rs3_v;
   wire [63:0] m2_imm = fault2_op ? 64'd0 : s2_imm;
   wire [5:0] m2_alu_op = s2_alu_op;
   wire  m2_alu_w = fault2_op ? 1'b0 : s2_alu_w;
   wire  m2_alu_uw = fault2_op ? 1'b0 : s2_alu_uw;
   wire [1:0] m2_op1_sel = fault2_op ? 2'd0 : s2_op1_sel;
   wire  m2_op2_imm = fault2_op ? 1'b0 : s2_op2_imm;
   wire  m2_res_link = fault2_op ? 1'b0 : s2_res_link;
   wire  m2_is_mem = fault2_op ? 1'b0 : s2_is_mem;
   wire  m2_is_store = fault2_op ? 1'b0 : s2_is_store;
   wire [1:0] m2_mem_size = s2_mem_size;
   wire  m2_mem_signed = s2_mem_signed;
   wire  m2_is_branch = fault2_op ? 1'b0 : s2_is_branch;
   wire [2:0] m2_br_func = s2_br_func;
   wire  m2_is_jump = fault2_op ? 1'b0 : s2_is_jump;
   wire  m2_is_jalr = fault2_op ? 1'b0 : s2_is_jalr;
   wire  m2_is_mul = fault2_op ? 1'b0 : s2_is_mul;
   wire  m2_is_csr = fault2_op ? 1'b0 : s2_is_csr;
   wire [2:0] m2_csr_func = s2_csr_func;
   wire  m2_is_serialize = fault2_op ? 1'b1 : s2_is_serialize;
   wire  m2_is_amo = fault2_op ? 1'b0 : s2_is_amo;
   wire [4:0] m2_amo_func = s2_amo_func;
   wire  m2_is_fencei = fault2_op ? 1'b0 : s2_is_fencei;
   wire  m2_is_cbo = fault2_op ? 1'b0 : s2_is_cbo;
   wire  m2_cbo_zero = s2_cbo_zero;
   wire  m2_cbo_keep = s2_cbo_keep;
   wire  m2_illegal = fault2_op ? 1'b0 : s2_illegal;
   wire  m2_mis_taken = (s2_taken_pc != f2_pnpc);
   wire  m2_mis_nt = (s2_ft_pc    != f2_pnpc);
   // Compact + refill: always write every slot (a held slot re-loads its own value, since
   // snext_i == cur[i] when it is a surviving slot with nothing consumed ahead of it).
   always @(posedge clk) begin
      if (reset | redirect) begin
         d_valid <= 1'b0; d2_valid <= 1'b0; d3_valid <= 1'b0;
      end else begin
         d_valid  <= vnext0;
         d2_valid <= vnext1;
         d3_valid <= vnext2;
         `FE_IRV(d_)  <= snext0;
         `FE_IRV(d2_) <= snext1;
         `FE_IRV(d3_) <= snext2;
      end
   end
   `undef FE_IRV
endmodule

`default_nettype wire
