`default_nettype none
`include "smolrv64_irec.vh"

// In-order frontend (stage F): PC -> iMMU-translated I$ window -> aligner ->
// RVC expand -> decode, terminating at the IR register that feeds stage X.
//
// This is probe/frontend.v with `decode_rename` replaced by a plain register:
// `fetch`, `predictor` and `decode_slot` are instantiated UNMODIFIED. Width is
// scalar (IW=1), so the aligner's cross-slot machinery collapses to one slot and
// `decode_xslot` is not needed at all.
//
// CHECKPOINTS. smolrv64_predictor predicts at the fetch stream and queues its predictions; fetch
// takes the head's details with the bundle that ends at its mark (pd_mk, else pd_no), the IR
// latches them as d_pdet, and they ride the pipeline to M, coming back as res_pdet when the op
// resolves. No checkpoint ring, no tag: the details are matched to their instruction by BEING
// the instruction's payload.

module smolrv64_frontend
  #(parameter PCW   = 64,
    parameter SEQW  = 8,
    parameter HW    = 2,             // fetch window halfwords (one 32-bit instruction)
    parameter IW    = 2,             // pipeline width (instructions/cycle); threaded from SMOLRV64_IW (Stage 3)
    parameter RASB  = 3,             // log2 RAS entries (smolrv64_predictor)
    parameter GHL   = 11,            // the predictor's history length
    parameter PDW   = 31,            // smolrv64_predictor's predict-detail width
    parameter integer DCR_HIT  = 2,  // the BTB-hit bit inside the predict details
    parameter integer DCR_BACK = 1,  // a backward conditional the BTB missed redirects at decode
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
    input  wire [IW-1:0]           consume,           // slot k dispatched (a group goes whole)

    // ---- redirect (from M: mispredict, trap, xret, fence.i) ----
    input  wire                    redirect,
    input  wire                    fs_off,            // mstatus.FS is Off: an FP op decodes illegal
    input  wire [2*IW+7:0]         crd,               // the dispatch credits (smolrv64_credits.vh)
    output wire                    hd_v,              // the queue head has an instruction...
    output wire                    hd_take,           // ...and pops it this cycle
    output wire [15:0]             hd_gc,             // its dispatch class (the stall accounting's)
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
    output wire                    imem_flush,        // the stream restarts: every I$ request in flight is stale
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

    // ================= the IR: one dispatch group, a decoded record per slot =================
    output reg  [IW-1:0]           d_valid,
    output reg  [IW*`IR_RECW-1:0]  d_rec,           // slot k's record at [k*IR_RECW +: IR_RECW]
    output wire [SEQW-1:0]         cur_seq,           // fetch PC's seqno (trap resume)
    // The frontend's invariants for the integrity log (rv_errlog bits 48..63), registered in
    // each unit: [5:0] the ring, [8:6] the predictor, [10:9] fetch, [12:11] this module.
    output wire [15:0]             fe_err);

   // ---------------------------------------------------------------- fetch
   // TWO-WIDE FETCH (2026-09-05, plan item 10a): the aligner emits up to two instructions per
   // cycle and both enter the queue in one cycle; decode still pops one. The queue then runs
   // ahead of dispatch and absorbs every fetch gap (a redirect's refill, a taken target's
   // arrival) instead of exposing it two cycles later as FE_QUE -- and it is the frontend a
   // two-wide dispatch (10b) needs. A bundle ends at its first CTI, so slot 1 is never
   // followed by anything and slot 0 is never a CTI when slot 1 is valid: the bundle's
   // prediction (pnpc_kind, target, details) belongs to its LAST valid slot and slot 0
   // falls through.
   localparam FW = IW;              // the width knob: the queue, decode and the group rules
   initial if (FW < 2 || FW > 4) $fatal(1, "smolrv64_frontend: IW=%0d: the width is 2..4", FW);
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
   wire [5:0]          ring_err;
   wire [2:0]          bp_err;
   wire [1:0]          f_err;
   wire                pr_adv, p_cut, pq_room;
   wire [63:0]         st_sa;
   wire [2:0]          st_sk, p_end;
   localparam integer  PQB = 3;
   wire [PQB:0]        pq_cnt;
   // A restart of the stream: reset, a redirect, or a freeze (fence.i, an I$ invalidation, a
   // mapping change, a rejected mark). It begins at the redirect's target, else at the PC.
   wire                st_flush = reset | redirect | imem_freeze | rej_q;
   assign              imem_flush = st_flush;
   wire [63:0]         st_rst_a = redirect ? redirect_pc : imem_addr;
   smolrv64_fring #(.HW(HW), .PQB(PQB)) u_ring
     (.clk(clk), .reset(reset), .freeze(imem_freeze | rej_q), .restart(reset | redirect),
      .adv_hw(f_adv_hw), .pop(f_pop), .drop(f_drop), .pq_tk(bp_tk), .pq_tgt(bp_tgt),
      .pc_va(imem_addr), .rst_a(st_rst_a), .pc_pa(imem_pa), .pc_lvl(imem_xlvl), .xlate_ok(imem_xlate_ok),
      .win(imem_data), .mk(imem_mk), .avail(imem_avail), .ok(imem_ok), .lvl(imem_lvl),
      .sa(st_sa), .sk(st_sk), .p_cut(p_cut), .p_end(p_end), .pr_adv(pr_adv),
      .pq_room(pq_room), .pq_cnt(pq_cnt),
      .ic_req(ic_req), .ic_va(ic_va), .ic_pa(ic_pa), .ic_tag(ic_tag),
      .ic_ack(ic_ack), .ic_valid(ic_valid), .ic_data(ic_data), .ic_rtag(ic_rtag), .err(ring_err));
   assign fe_avail = imem_avail;
   assign fe_ok    = imem_ok;

   // No checkpoint ring, no `cur`, no `create`, no rb_idx. smolrv64_predictor keeps committed
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
      .err(f_err), .ready(pb_ready), .valid(dq_valid),
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
   // queue (`.ready(accept)` -> `.ready(~q_full)`), and smolrv64_core's inject_inflight
   // interlock was left keyed to the old one. Report the real event so the interlock can
   // name it: while the backend stalls, `accept` is low but `fire` is not, so fetch
   // re-emitted the SAME interrupt every cycle with nothing to stop it.
   wire   irq_pres;                    // fetch presents the pseudo-op (a straddle finishes first)
   assign irq_taken = irq_pres & fire;

   // Sub-attribution for the frontend bubble (see smolrv64_core's fe_aln/fe_que).  dq_valid is
   // "fetch assembled a complete instruction this cycle".  With it, a bubble that is not
   // the iMMU and not an empty fetch window splits into two very different problems:
   // fetch had BYTES but could not make an instruction (aligner/straddle), versus fetch
   // made one and the queue simply had nothing to hand over (refill latency).
   assign fe_dq_valid = dq_valid;

   // ------------------------------------------------------- branch predictor
   wire [PDW-1:0] pd_mk, pd_no;
   wire [PDW-1:0] pd_fetch = f_at_mk ? pd_mk : pd_no;   // one bundle's details, all its slots
   smolrv64_predictor #(.PCW(PCW), .PDW(PDW), .RASB(RASB), .GHL(GHL), .PQB(PQB)) u_bp
     (.clk(clk), .reset(reset),
      .flush(st_flush), .rst_a(st_rst_a), .rst_np(rej_np_q & ~redirect), .adv(pr_adv),
      .sa(st_sa), .sk(st_sk), .p_cut(p_cut), .p_end(p_end), .pq_room(pq_room), .pq_cnt(pq_cnt),
      .pq_tk(bp_tk), .pq_tgt(bp_tgt), .pd_mk(pd_mk), .pd_no(pd_no), .pop(f_pop),
      .rollback(redirect), .rb_rsp(redirect_rsp), .rb_ghr(redirect_ghr),
      .res_v(res_v), .res_cbr(res_cbr), .res_call(res_call), .res_ret(res_ret),
      .res_taken(res_taken), .res_pdet(res_pdet), .res_tgt(res_tgt), .res_pc(res_pc),
      .res_rvc(res_rvc), .err(bp_err));

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
   // One PACKED record per slot (smolrv64_irec.vh) keeps the compaction a handful of muxes
   // instead of ~46 per field, with one field list for pack and unpack.
   localparam integer IRW = `IR_RECW;
   wire [IRW-1:0] mh [0:FW-1];   // the bundle register's slots, decoded and packed (the queue's write data)
   wire           dq_fault = imem_fault & ~dq_valid;    // fetch-fault pseudo-op, pushed like a bundle
   wire           pb_load  = pb_ready & (dq_valid | dq_fault);   // the register takes a bundle or the fault op
   // UP TO FW ENTRIES PER CYCLE INTO A LUTRAM: QNB = next_pow2(FW) consecutive-write banks,
   // bank = pos[QLB-1:0], index = pos>>QLB, ONE muxed write per bank (one write statement per
   // LUTRAM bank). `q_room` asks for a whole bundle's room, so a bundle never splits.
   localparam integer QNB = 1 << $clog2(FW);
   localparam integer QLB = $clog2(QNB);
   localparam integer QBD = QDEPTH / QNB;
   localparam integer CW  = $clog2(FW + 1);             // a count of 0..FW slots
   wire [IRW-1:0] q_bank_rd [0:QNB-1];
   reg  [QAW-1:0] q_rp, q_wp;
   reg  [QAW:0]   q_cnt;
   wire           q_room  = (q_cnt <= QDEPTH[QAW:0] - FW[QAW:0]);   // room for a full bundle (<=FW)
   wire           q_empty = (q_cnt == {(QAW+1){1'b0}});
   reg  [QW-1:0]  pb_in [0:FW-1];
   reg  [FW-1:0]  pb_sv;                                 // the bundle's slots (slot 0: whenever pb_v)
   wire           q_push  = q_room & pb_v;
   wire [FW-1:0]  q_wr    = {FW{q_push}} & pb_sv;        // the slots pushed this cycle: 0..n-1
   function automatic [CW-1:0] cnt(input [FW-1:0] v);
      integer i;
      begin cnt = {CW{1'b0}}; for (i = 0; i < FW; i = i + 1) cnt = cnt + {{(CW-1){1'b0}}, v[i]}; end
   endfunction
   // Slot k's queue record: slot 0 is the fetch-fault pseudo-op when fetch has nothing; a slot
   // falls through when a later one follows, and the bundle's last valid slot carries its
   // prediction (pnpc_kind, the target and the details belong to it).
   wire [QW-1:0]  q_in [0:FW-1];
   genvar gk;
   generate for (gk = 0; gk < FW; gk = gk + 1) begin: qi
      wire last = (gk == FW - 1) ? 1'b1 : ~dq_sv[(gk == FW - 1) ? gk : gk + 1];
      assign q_in[gk] = {pd_fetch, ((gk == 0) & dq_fault) ? imem_ipc : dq_pc[gk*PCW +: PCW],
                         dq_inst[gk*32 +: 32], dq_seq[gk*SEQW +: SEQW],
                         last ? dq_pk : 2'd0, bp_tgt, (gk == 0) & dq_fault, imem_cause, imem_addr};
   end endgenerate
   integer k;
   always @(posedge clk) begin
      if (reset | redirect)  pb_v <= 1'b0;
      else if (pb_load)       pb_v <= 1'b1;
      else if (q_push)       pb_v <= 1'b0;
      if (pb_load) begin
         for (k = 0; k < FW; k = k + 1) pb_in[k] <= q_in[k];
         pb_sv <= {dq_sv[FW-1:1] & {(FW-1){dq_valid}}, 1'b1};
      end
   end
   // ---- N-bank queue storage: one muxed write per bank, one read per bank ----
   genvar qgb;
   generate for (qgb = 0; qgb < QNB; qgb = qgb + 1) begin: qbank
      reg [IRW-1:0] mem [0:QBD-1];
      integer qbi; initial for (qbi = 0; qbi < QBD; qbi = qbi + 1) mem[qbi] = {IRW{1'b0}};
      // slot j writes position q_wp + j; FW <= QNB, so at most one slot lands in this bank
      reg              wen;
      reg  [QAW-1:0]   wpos;
      reg  [IRW-1:0]   wd;
      integer j;
      always @* begin
         wen = 1'b0;  wpos = q_wp;  wd = mh[0];
         for (j = FW - 1; j >= 0; j = j - 1)
            if (q_wr[j] & ((q_wp[QLB-1:0] + j[QLB-1:0]) == qgb[QLB-1:0])) begin
               wen = 1'b1;  wpos = q_wp + j[QAW-1:0];  wd = mh[j];
            end
      end
      always @(posedge clk)
         if (wen) mem[wpos[QAW-1:QLB]] <= wd;
      // read: the position in this bank at or after the head
      wire [QLB-1:0] roff = qgb[QLB-1:0] - q_rp[QLB-1:0];
      wire [QAW-1:0] rpos = q_rp + {{(QAW-QLB){1'b0}}, roff};
      assign q_bank_rd[qgb] = mem[rpos[QAW-1:QLB]];
   end endgenerate
   // the heads: decoded records at q_rp + k, and each one's dispatch class (the record's last field)
   wire [IRW-1:0] qh [0:FW-1];
   wire [15:0]    g  [0:FW-1];
   generate for (gk = 0; gk < FW; gk = gk + 1) begin: hd
      localparam [QAW-1:0] K = gk;
      wire [QAW-1:0] pos = q_rp + K;
      assign qh[gk] = q_bank_rd[pos[QLB-1:0]];
      assign g[gk]  = qh[gk][15:0];
   end endgenerate
   wire [CW-1:0]  q_pop;                          // entries leaving this cycle (the group's take)
   wire [CW-1:0]  q_pushn = cnt(q_wr);            // entries pushed
   always @(posedge clk) begin
      if (reset | redirect) begin
         q_cnt <= {(QAW+1){1'b0}}; q_rp <= {QAW{1'b0}}; q_wp <= {QAW{1'b0}};
      end else begin
         q_wp  <= q_wp + {{(QAW-CW){1'b0}}, q_pushn};
         q_rp  <= q_rp + {{(QAW-CW){1'b0}}, q_pop};
         q_cnt <= q_cnt + {{(QAW+1-CW){1'b0}}, q_pushn} - {{(QAW+1-CW){1'b0}}, q_pop};
      end
   end

   // A slot without its predecessor, or a fault pseudo-op beside a real slot, is a fetch defect.
   wire e_bundle = ~reset & ((|(dq_sv[FW-1:1] & ~dq_sv[FW-2:0])) | (dq_fault & dq_valid));
   always @(posedge clk)
      if (e_bundle) $fatal(1, "smolrv64_frontend: malformed bundle sv=%b fault=%b", dq_sv, dq_fault);

   // ---------------------------------------------------------------- decode
   // The bundle register's slots, decoded into the queue's records.
   generate for (gk = 0; gk < FW; gk = gk + 1) begin: sl
      wire [PDW-1:0]  pdet;  wire [PCW-1:0] pc;  wire [31:0] inst;  wire [SEQW-1:0] seq;
      wire [1:0]      pk;    wire [PCW-1:0] tgt; wire fault_op;    wire [3:0] cause;  wire [PCW-1:0] tval;
      assign {pdet, pc, inst, seq, pk, tgt, fault_op, cause, tval} = pb_in[gk];
      smolrv64_dslot #(.PCW(PCW), .SEQW(SEQW), .PDW(PDW), .DCR_HIT(DCR_HIT), .DCR_BACK(DCR_BACK)) u_ds
        (.pdet(pdet), .pc(pc), .inst(inst), .seq_in(seq), .pk(pk), .tgt(tgt), .fault_op(fault_op),
         .cause(cause), .tval(tval), .fs_off(fs_off), .rec(mh[gk]));
   end endgenerate

   // ------------------------------------------------------- the IR registers: one dispatch group
   // The queue head forms the group. Up to FW heads go together when the dispatch class allows
   // it (smolrv64_gclass, evaluated at decode): every member plain (a head op is not), at most
   // one of the FP/MD/SYS pipe's, one load and one store (the queues' single allocation ports),
   // and nothing behind a CTI that redirects fetch at decode. The IR takes the group when it is
   // empty or dispatching whole this cycle, and dispatch takes a group whole or not at all:
   // there are no survivors.
   localparam integer G_L = 4, G_I = 5, G_FC = 6, G_PLAIN = 7, G_DCR = 11,   // smolrv64_gclass's gc order
                      G_LD = 12, G_ST = 13, G_CSR = 14, G_SER = 15;
   // the credits: a member pops only with room for it in everything it allocates, counted
   // against what is already between here and there (smolrv64_core computes crd from flops)
`include "smolrv64_credits.vh"
   function automatic room(input [15:0] gc, input lane, input l, input ld, input st, input f);
      room = (~gc[G_I] | lane) & (~gc[G_L] | l) & (~gc[G_LD] | ld) & (~gc[G_ST] | st) & (~gc[G_FC] | f);
   endfunction
   // the rules: which heads may go together (cumulative classes of the slots before)
   // slot k reads slot k-1: per-bit variables, so the chain is not a loop to the simulator
   wire [FW-1:0] rule /*verilator split_var*/, take /*verilator split_var*/, c_fc /*verilator split_var*/,
                 c_ld /*verilator split_var*/, c_st /*verilator split_var*/;
   // a CBO is the one head op that does not serialise (an AMO or LR/SC does)
   wire g0_cbo  = g[0][G_L] & ~g[0][G_PLAIN] & ~g[0][G_SER];
   wire head_go = g[0][G_SER] ? crd[CR_SER] : g[0][G_CSR] ? crd[CR_CSR] : g0_cbo ? crd[CR_CBO] : 1'b1;
   assign rule[0] = ~q_empty;
   assign take[0] = rule[0] & crd[CR_POP] & head_go & crd[CR_ROB1]
                  & room(g[0], crd[CR_IA], crd[CR_L], crd[CR_LD], crd[CR_ST], crd[CR_F]);
   assign c_fc[0] = g[0][G_FC];  assign c_ld[0] = g[0][G_LD];  assign c_st[0] = g[0][G_ST];
   generate for (gk = 1; gk < FW; gk = gk + 1) begin: rl
      localparam [QAW:0] K = gk;
      assign rule[gk] = rule[gk-1] & (q_cnt > K) & g[0][G_PLAIN] & g[gk][G_PLAIN] & ~g[gk-1][G_DCR]
                      & ~(g[gk][G_FC] & c_fc[gk-1]) & ~(g[gk][G_LD] & c_ld[gk-1]) & ~(g[gk][G_ST] & c_st[gk-1]);
      assign take[gk] = rule[gk] & take[gk-1] & crd[CR_ROB1 + gk]
                      & room(g[gk], crd[CR_IA + gk], crd[CR_L], crd[CR_LD], crd[CR_ST], crd[CR_F]);
      assign c_fc[gk] = c_fc[gk-1] | g[gk][G_FC];
      assign c_ld[gk] = c_ld[gk-1] | g[gk][G_LD];
      assign c_st[gk] = c_st[gk-1] | g[gk][G_ST];
   end endgenerate
   // The IR loads every cycle: what it holds dispatches the next, or is dropped by a freeze
   // (a redirect, an early restart, a decode resteer), in which case it is the wrong path.
   assign q_pop = cnt(take);
   assign hd_v = rule[0];  assign hd_take = take[0];  assign hd_gc = g[0];
   // a group dispatches whole: a prefix of the slots, all of the IR's
   wire e_order = ~reset & ((|(consume[FW-1:1] & ~consume[FW-2:0])) | (consume[0] & (consume != d_valid)));
   always @(posedge clk)
      if (e_order) $fatal(1, "smolrv64_frontend: a group dispatched in part (consume %b, valid %b)", consume, d_valid);
   reg  [1:0] fe_err_q;
   always @(posedge clk) fe_err_q <= reset ? 2'd0 : {e_order, e_bundle};
   assign fe_err = {3'd0, fe_err_q, f_err, bp_err, ring_err};
   always @(posedge clk) begin
      if (reset | redirect) d_valid <= {FW{1'b0}};
      else begin
         d_valid <= take;
         for (k = 0; k < FW; k = k + 1) d_rec[k*IRW +: IRW] <= qh[k];
      end
   end
endmodule

`default_nettype wire
