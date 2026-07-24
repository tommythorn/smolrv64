`default_nettype none

// Frontend branch predictor, Phase 0 of docs/branch-predictor-plan.md: a
// block-indexed BTB with an embedded 2-bit bimodal direction counter + a return
// address stack, with the speculative state {ghr, ras, ras_ptr} checkpointed as
// a structural clone of rename_shard's chk_map (snapshot at `create`, restore at
// `rollback`, same create/cur/rollback/rollback_idx wires). The GHR is carried
// and checkpointed but not yet used for indexing (Phase 1 = YAGS drops in here).
//
// Everything here is a HINT, never architectural: the mispredict check is the
// exec-side `actual_npc != pred_npc` compare (branch_unit), so a stale BTB entry,
// an aliased tag, or a mis-restored RAS costs an extra redirect, never a wrong
// result.
//
// TIMING SHAPE (this is the load-bearing property): prediction is computed from
// REGISTERED state only -- btb_q (the BTB entry read last cycle at fetch's
// computed next PC and flopped, rule A1) and the RAS registers. NO instruction
// bytes are inspected at fetch: the CTI class (cond / jump / call / return)
// lives in the BTB type field, trained at resolve from the executed
// instruction. So the fetch-cone addition is one 64-bit register equality
// (read-address == bundle base) + a 3-bit type decode + the target mux --
// nothing from the I$-data -> aligner cloud feeds the PC mux. The cost is hint
// quality only: an untrained CTI (including a return's first execution per
// call site) predicts fall-through and pays one mispredict to train.
//
// Bundle-granularity: the aligner ends every bundle at its first CTI, so a
// bundle has at most one control transfer, it is the last valid slot, and the
// bundle's fall-through (ft_npc, fetch's existing +2*consumed path) is exactly
// a call's return address.
//
// Training is resolve-time only (the BTB is a cache, never rolled back):
// exec_bundle exports the oldest genuinely-resolved CTI per cycle {taken,
// taken-target, call/ret class, ckpt}; the predict-time details (index/tag/
// hit/ctr) come from a small per-checkpoint table written at dispatch -- <=1
// CTI per bundle == per ckpt, so the checkpoint tag the op already carries is
// the resolve key. No payload bits.
module predictor
  #(parameter PCW   = 64,
    parameter CBITS = 2,
    parameter NCHK  = 4,
    parameter BTBB  = 8,             // log2 BTB entries
    parameter TAGW  = 12,
    parameter TGTW  = 38,            // stored target bits [38:1] (canonical VA, sign-extended)
    parameter GHL   = 12,            // global history length (dormant until Phase 1)
    parameter RASB  = 3)             // log2 RAS entries
   (input  wire                 clk,
    input  wire                 reset,
    // ---- fetch side (cycle T) ----
    input  wire [PCW-1:0]       npc,        // fetch's computed next PC -> BTB read address
    input  wire                 fire,       // fetch handshake: bundle leaves fetch this cycle
    input  wire [PCW-1:0]       base_pc,    // presented bundle's base PC (= pc_q)
    input  wire [PCW-1:0]       ft_npc,     // presented bundle's fall-through (= call return address)
    input  wire                 cti_ok,     // bundle ends on a real branch/jump (aligner br_term):
                                            // ONLY such bundles have the exec-side compare, so a
                                            // stale entry may never steer any other bundle shape
    output wire                 pred_v,     // predict taken: fetch overrides its next PC
    output wire [PCW-1:0]       pred_tgt,
    // ---- checkpoint control (rename time domain; clone of chk_map's contract) ----
    input  wire                 create,
    input  wire [CBITS-1:0]     cur,
    input  wire                 rollback,
    input  wire [CBITS-1:0]     rollback_idx,
    // ---- resolve/training port (EX domain; oldest resolved CTI this cycle) ----
    input  wire                 res_v,
    input  wire                 res_cbr,     // conditional branch (vs jump/JALR)
    input  wire                 res_call,    // jump with a link dest (rd in {x1,x5})
    input  wire                 res_ret,     // JALR return (rs1 link, rd not)
    input  wire                 res_taken,
    input  wire [CBITS-1:0]     res_ckpt,
    input  wire [PCW-1:0]       res_tgt,     // taken-target (train the BTB)
    input  wire                 res_rep);    // this resolve caused this cycle's rollback ->
                                             // repair the restored GHR's bit 0 (plan B2)

   localparam NBTB = 1 << BTBB;
   localparam RASN = 1 << RASB;
   // BTB type: 0xx = cond branch, xx = 2-bit bimodal (00 S_N .. 11 S_T); 1xx = uncond
   localparam [2:0] TY_JMP = 3'b100, TY_CALL = 3'b101, TY_RET = 3'b110;

   // ------------------------------------------------------------ BTB (1R1W RAM)
   // entry = {tag, type[2:0], target[38:1]}; valid bits kept aside as a flop
   // vector so the data array stays a clean BRAM/LUTRAM inference.
   localparam EW = TAGW + 3 + TGTW;
   reg [EW-1:0]   btb   [0:NBTB-1];
   reg [NBTB-1:0] btb_v;
   reg [EW-1:0]   btb_q;                     // registered read (rule A1)
   reg            btb_qv;
   reg [PCW-1:0]  btb_qpc;                   // address the read was for
   initial begin btb_v = {NBTB{1'b0}}; btb_qv = 1'b0; btb_qpc = {PCW{1'b0}}; end

   // Tag folds higher PC bits in (XOR) so addresses that agree in [20:1] but differ
   // above still miss. Load-bearing across the paging transition: the kernel's
   // pre-MMU execution at PA 0x802xxxxx aliases its post-MMU VAs 0xffffffff80xxxxxx
   // in bits [20:1] exactly (the virtual-to-load offset is 2MB-aligned), so without
   // the fold every early-boot-trained branch leaves a stale physical-target twin
   // that hits under Sv39 and steers fetch to an unmapped bare address.
   function [TAGW-1:0] btag(input [PCW-1:0] a);
      btag = a[BTBB+TAGW:BTBB+1] ^ a[BTBB+2*TAGW:BTBB+TAGW+1]
           ^ {{(TAGW-1){1'b0}}, a[63]};
   endfunction
   function [BTBB-1:0] bidx(input [PCW-1:0] a); bidx = a[BTBB:1];           endfunction

   // ------------------------------------------------------------ YAGS corrector
   // Phase 1: a tagged direction corrector indexed by PC^GHR, consulted only for a
   // known conditional branch. On a tag hit it OVERRIDES the BTB's context-free
   // bimodal weight -- capturing the history-correlated branches bimodal aliases.
   // Trained at resolve from the carried predict details (yidx/ytag), same 1R1W +
   // write-forward discipline as the BTB. Holds only bimodal-exceptions: allocate
   // on a bimodal miss, refine on a corrector hit.
   localparam YBITS = 10, NYAGS = 1 << YBITS, YTAGW = 8, YEW = YTAGW + 2;
   function [YBITS-1:0]  yidx (input [PCW-1:0] a, input [GHL-1:0] h);
      yidx  = a[YBITS:1] ^ h[YBITS-1:0] ^ {{(YBITS-2){1'b0}}, h[GHL-1:YBITS]};
   endfunction
   function [YTAGW-1:0] ytagf(input [PCW-1:0] a);
      ytagf = a[YBITS+YTAGW:YBITS+1] ^ a[YBITS+2*YTAGW:YBITS+YTAGW+1] ^ {{(YTAGW-1){1'b0}}, a[63]};
   endfunction
   reg [YEW-1:0]   ycorr [0:NYAGS-1];
   reg [NYAGS-1:0] ycorr_v;
   reg [YEW-1:0]   ycorr_q;  reg ycorr_qv;
   integer yi;
   initial begin
      ycorr_v = {NYAGS{1'b0}}; ycorr_qv = 1'b0;
      for (yi = 0; yi < NYAGS; yi = yi + 1) ycorr[yi] = {YEW{1'b0}};
   end

   // ------------------------------------------------- speculative state {ghr,ras}
   reg [GHL-1:0]  ghr;
   reg [PCW-1:0]  ras [0:RASN-1];
   reg [RASB-1:0] ras_ptr;                   // top of stack
   reg [GHL-1:0]  chk_ghr  [0:NCHK-1];
   reg [PCW-1:0]  chk_ras  [0:NCHK-1][0:RASN-1];
   reg [RASB-1:0] chk_rptr [0:NCHK-1];
   integer ii, jj;
   initial begin
      ghr = {GHL{1'b0}}; ras_ptr = {RASB{1'b0}};
      for (ii = 0; ii < RASN; ii = ii + 1) ras[ii] = {PCW{1'b0}};
      for (ii = 0; ii < NCHK; ii = ii + 1) begin
         chk_ghr[ii] = {GHL{1'b0}}; chk_rptr[ii] = {RASB{1'b0}};
         for (jj = 0; jj < RASN; jj = jj + 1) chk_ras[ii][jj] = {PCW{1'b0}};
      end
   end

   // ------------------------------------------------------------------- predict
   // (registered state only -- see the timing-shape note above)
   wire            hit      = cti_ok & btb_qv & (btb_qpc == base_pc)
                            & (btb_q[EW-1 -: TAGW] == btag(base_pc));
   wire [2:0]      q_type   = btb_q[TGTW +: 3];
   wire [TGTW-1:0] q_tgt    = btb_q[TGTW-1:0];
   wire [PCW-1:0]  btb_tgt  = {{(PCW-TGTW-1){q_tgt[TGTW-1]}}, q_tgt, 1'b0};  // sign-extend canonical VA
   wire            p_cbr    = hit & ~q_type[2];             // known conditional branch
   wire            p_call   = hit & (q_type == TY_CALL);
   wire            p_ret    = hit & (q_type == TY_RET);
   // YAGS: a tag-hitting corrector overrides the bimodal weight for a conditional
   wire            yhit     = p_cbr & ycorr_qv & (ycorr_q[YEW-1 -: YTAGW] == ytagf(base_pc));
   wire            cbr_taken= yhit ? ycorr_q[1] : q_type[1];
   assign pred_v   = hit & (q_type[2] | (p_cbr & cbr_taken)); // uncond, or predicted-taken cond
   assign pred_tgt = p_ret ? ras[ras_ptr] : btb_tgt;
   wire            pred_dir = cbr_taken;                    // GHR shifts the committed direction

   // ------------------------- per-checkpoint predict details (for training/repair)
   // captured at fetch (cycle T), written to pdet[cur] at the bundle's create
   // (T+1) -- the same one-stage lag as the {ghr,ras} snapshot (plan B1).
   //   bimodal [BIMW-1:0] = {hit,ctr,bidx,btag}  ·  yags [PDW-1 -: YW] = {yhit,yctr,yidx,ytag}
   localparam BIMW = 1 + 2 + BTBB + TAGW;
   localparam YW   = 1 + 2 + YBITS + YTAGW;
   localparam PDW  = BIMW + YW;
   reg [PDW-1:0] pdet_f;
   reg [PDW-1:0] pdet [0:NCHK-1];
   wire [1:0]    ctr_eff  = hit  ? q_type[1:0]  : 2'b01;    // miss -> install weakly-not-taken base
   wire [1:0]    yctr_eff = yhit ? ycorr_q[1:0] : 2'b01;
   initial pdet_f = {PDW{1'b0}};

   // ------------------------------------------------- speculate / snapshot / restore
   wire [CBITS-1:0] nxt = cur + 1'b1;
   integer k;
   always @(posedge clk) begin
      if (reset) begin
         ghr <= {GHL{1'b0}}; ras_ptr <= {RASB{1'b0}};
         for (k = 0; k < NCHK; k = k + 1) begin
            chk_ghr[k] <= {GHL{1'b0}}; chk_rptr[k] <= {RASB{1'b0}};
         end
      end else if (rollback) begin
         // restore the reopened span's pre-state; on a cond-branch mispredict the
         // snapshot's bit 0 is that branch's predicted direction -> overwrite with
         // the resolved one (exact when the branch shifted; a cold branch that
         // never shifted gets one polluted history bit -- hint-only). Jumps/traps:
         // restore verbatim.
         ghr     <= res_rep ? {chk_ghr[rollback_idx][GHL-1:1], res_taken}
                            :  chk_ghr[rollback_idx];
         ras_ptr <= chk_rptr[rollback_idx];
         for (k = 0; k < RASN; k = k + 1) ras[k] <= chk_ras[rollback_idx][k];
      end else begin
         if (fire) begin                     // speculate: advance ONLY on the fetch handshake
            if (p_cbr)  ghr <= {ghr[GHL-2:0], pred_dir};
            if (p_call) begin ras[ras_ptr + 1'b1] <= ft_npc; ras_ptr <= ras_ptr + 1'b1; end
            if (p_ret)  ras_ptr <= ras_ptr - 1'b1;
            pdet_f <= {yhit, yctr_eff, yidx(base_pc, ghr), ytagf(base_pc),
                       hit,  ctr_eff,  bidx(base_pc),      btag(base_pc)};
         end
         if (create) begin                   // snapshot the dispatching bundle's post-state
            chk_ghr[nxt]  <= ghr;            // (registered values = post-state of the bundle
            chk_rptr[nxt] <= ras_ptr;        //  fetched last cycle = the one dispatching now)
            for (k = 0; k < RASN; k = k + 1) chk_ras[nxt][k] <= ras[k];
            pdet[cur]     <= pdet_f;         // the dispatching bundle's own predict details
         end
      end
   end

   // ------------------------------------------------------------------ training
   // one BTB write per resolved CTI (oldest per cycle); read + write ports keep
   // the array 1R1W. Bimodal: nudge the (carried) counter toward the resolved
   // direction; uncond: record the class (call/return classified at resolve
   // from the executed instruction); target: the resolved taken-target.
   wire [PDW-1:0]  td      = pdet[res_ckpt];
   wire            t_hit   = td[BIMW-1];
   wire [1:0]      t_ctr   = td[BIMW-2 -: 2];
   wire [BTBB-1:0] t_idx   = td[TAGW +: BTBB];
   wire [TAGW-1:0] t_tag   = td[TAGW-1:0];
   wire [1:0]      t_base  = t_hit ? t_ctr : (res_taken ? 2'b10 : 2'b01);  // miss -> install weak
   wire [1:0]      t_nudge = res_taken ? ((t_base == 2'b11) ? 2'b11 : t_base + 1'b1)
                                       : ((t_base == 2'b00) ? 2'b00 : t_base - 1'b1);
   wire [2:0]      t_type  = res_cbr  ? {1'b0, t_nudge}
                           : res_ret  ? TY_RET
                           : res_call ? TY_CALL : TY_JMP;
   // YAGS corrector training: decode carried predict details, nudge, decide (re)alloc.
   // Update when the corrector was consulted (yc_hit) OR the bimodal mispredicted this
   // conditional -- so the corrector holds exactly the bimodal-exceptions.
   wire [YW-1:0]    yd     = td[PDW-1 -: YW];
   wire             yc_hit = yd[YW-1];
   wire [1:0]       yc_ctr = yd[YW-2 -: 2];
   wire [YBITS-1:0] yc_idx = yd[YTAGW +: YBITS];
   wire [YTAGW-1:0] yc_tag = yd[YTAGW-1:0];
   wire [1:0]       y_base = yc_hit ? yc_ctr : (res_taken ? 2'b10 : 2'b01);
   wire [1:0]       y_nudge= res_taken ? ((y_base == 2'b11) ? 2'b11 : y_base + 1'b1)
                                       : ((y_base == 2'b00) ? 2'b00 : y_base - 1'b1);
   wire             bim_pred = t_hit & t_ctr[1];            // bimodal predicted-taken (miss = NT)
   wire             y_wr   = res_v & res_cbr & (yc_hit | (bim_pred != res_taken));

   // write-forward: a mispredict's redirected refetch reads the BTB the same edge
   // its own training write lands -- without forwarding the retrained entry is
   // invisible to that first refetch and every cold-taken CTI mispredicts twice.
   wire t_fwd = res_v && (t_idx == bidx(npc));
   wire y_fwd = y_wr  && (yc_idx == yidx(npc, ghr));        // same write-forward for the corrector
   always @(posedge clk) begin
      btb_q   <= t_fwd ? {t_tag, t_type, res_tgt[TGTW:1]} : btb[bidx(npc)];
      btb_qv  <= t_fwd ? 1'b1 : btb_v[bidx(npc)];
      btb_qpc <= npc;
      ycorr_q  <= y_fwd ? {yc_tag, y_nudge} : ycorr[yidx(npc, ghr)];
      ycorr_qv <= y_fwd ? 1'b1              : ycorr_v[yidx(npc, ghr)];
      if (res_v) begin
         btb[t_idx]   <= {t_tag, t_type, res_tgt[TGTW:1]};
         btb_v[t_idx] <= 1'b1;
      end
      if (y_wr) begin
         ycorr[yc_idx]   <= {yc_tag, y_nudge};
         ycorr_v[yc_idx] <= 1'b1;
      end
      if (reset) begin btb_v <= {NBTB{1'b0}}; ycorr_v <= {NYAGS{1'b0}}; end
   end
endmodule

`default_nettype wire
