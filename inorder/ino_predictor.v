`default_nettype none

// IN-ORDER frontend branch predictor. Forked from src/predictor.v, which checkpoints
// {ghr, ras, ras_ptr} into an NCHK-deep ring as a structural clone of rename_shard's
// chk_map. This core has NO checkpoints and needs none:
//
//   M is the only commit point, so at most one instruction can be redirecting and
//   everything younger is squashed wholesale. There is never a second speculative
//   state to choose between, so there is nothing for a ring INDEX to select.
//
// What replaces it: two committed scalars (ghr_c, rptr_c) advanced at resolve and
// restored on redirect, and NO committed RAS array at all. The per-bundle predict
// details ride the pipeline with their instruction (pd_fetch -> res_pdet) instead of
// sitting in a side ring keyed by a checkpoint tag.
//
// Forked rather than parameterised so the in-order core's needs stop being negotiated
// against the sharded-OoO core's. src/predictor.v is left untouched.
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
module ino_predictor
  #(parameter PCW   = 64,
    parameter BTBB  = 8,             // log2 BTB entries
    parameter TAGW  = 12,
    parameter TGTW  = 38,            // stored target bits [38:1] (canonical VA, sign-extended)
    parameter GHL   = 12,            // global history length (dormant until Phase 1)
    parameter RASB  = 3,             // log2 RAS entries
    parameter YBITS = 10,            // in the header so PDW can be a port width
    parameter YTAGW = 8,
    parameter PDW   = (1+2+BTBB+TAGW) + (1+2+YBITS+YTAGW))
   (input  wire                 clk,
    input  wire                 reset,
    // ---- fetch side (cycle T) ----
    input  wire [PCW-1:0]       apc,        // fetch's AHEAD next PC -> BTB read address. A
                                            // register-only PREDICTION of the next base PC,
                                            // not the real one: see fetch.v's `apc`. The
                                            // entry is stamped with it (btb_qpc <= apc) and
                                            // unusable unless btb_qpc == base_pc, so a wrong
                                            // apc loses a prediction and never fakes one.
    input  wire                 fire,       // fetch handshake: bundle leaves fetch this cycle
    input  wire [PCW-1:0]       base_pc,    // presented bundle's base PC (= pc_q)
    input  wire [PCW-1:0]       ft_npc,     // presented bundle's fall-through (= call return address)
    input  wire                 cti_ok,     // bundle ends on a real branch/jump (aligner br_term):
                                            // ONLY such bundles have the exec-side compare, so a
                                            // stale entry may never steer any other bundle shape
    output wire                 pred_v,     // predict taken: fetch overrides its next PC
    output wire [PCW-1:0]       pred_tgt,
    // ---- redirect ----
    input  wire                 rollback,     // a redirect is entering the frontend this cycle
    // ---- predict details, CARRIED WITH THE INSTRUCTION (no checkpoint ring) ----
    output wire [PDW-1:0]       pd_fetch,     // this bundle's details; capture it where the
                                              // frontend latches the bundle (one cycle after fire)
    // ---- resolve/training port (EX domain; oldest resolved CTI this cycle) ----
    input  wire                 res_v,
    input  wire                 res_cbr,     // conditional branch (vs jump/JALR)
    input  wire                 res_call,    // jump with a link dest (rd in {x1,x5})
    input  wire                 res_ret,     // JALR return (rs1 link, rd not)
    input  wire                 res_taken,
    input  wire [PDW-1:0]       res_pdet,     // the RESOLVING instruction's own details
    input  wire [PCW-1:0]       res_tgt,     // taken-target (train the BTB)
    input  wire                 res_rep);    // this resolve caused this cycle's rollback ->
                                             // repair the restored GHR's bit 0 (plan B2)

   localparam NBTB = 1 << BTBB;
   localparam RASN = 1 << RASB;
   // BTB type: 0xx = cond branch, xx = 2-bit bimodal (00 S_N .. 11 S_T); 1xx = uncond
   localparam [2:0] TY_JMP = 3'b100, TY_CALL = 3'b101, TY_RET = 3'b110;

   // ------------------------------------------------------------ BTB (1R1W RAM)
   // entry = {tag, type[2:0], target[38:1]}, and NO valid bit -- the tag carries it, exactly
   // as in the corrector below. A never-written entry reads as all zeros, so it is believed
   // only by a PC whose btag is also 0 (1 in 4096) whose bundle ends on a CTI; what it then
   // says is type=000, ctr=00 -- a strongly-not-taken conditional, i.e. pred_v=0, which is
   // indistinguishable from having no entry. The one case that is not silent needs the
   // CORRECTOR to override that direction from an aliased entry as well (~1e-6): then it
   // predicts taken to target 0, which is a mispredict M resolves and the same resolve
   // retrains, and it is not a new class of event -- the BTB is a cache that is never rolled
   // back, so a stale entry pointing anywhere is already the normal case (see the tag-fold
   // note below, which exists because of exactly that).
   //
   // The read is therefore a plain synchronous-read RAM costing one block-RAM address setup.
   //
   // FMAX (this is why, 166 MHz): the valid bits used to be a separate flop vector read as
   // `btb_v[bidx(npc)]` -- a 256:1, and for the corrector a 1024:1, LUT/MUXF mux hanging off
   // the END of the fetch loop, which is the worst path in the design:
   //   strad -> iMMU translate -> I$ data -> aligner -> ft_npc -> npc -> yidx -> ycorr_v mux
   // Measured on that path at 6.462 ns: the valid mux alone was 1.05 ns, and the index cost
   // another 0.55 ns of pure route because it fanned out to 161 distributed-RAM address pins
   // (1024 deep is 16 LUTRAM primitives deep PER BIT). 1.6 ns of a 6.0 ns budget spent reading
   // an 11-bit entry. Folding valid in and forcing block RAM ends the loop at a BRAM address
   // pin: two loads on the index, no depth mux, no valid mux.
   //
   // With no valid bit there is also nothing for `reset` to mass-clear, which a BRAM cannot
   // do anyway. Configuration INIT zeroes the array, so a cold FPGA and a fresh simulation
   // both start with every entry untagged.
   localparam EW = TAGW + 3 + TGTW;
   (* ram_style = "block" *)
   reg [EW-1:0]   btb   [0:NBTB-1];          // {tag, type, target} -- validity IS the tag match
   reg [EW-1:0]   btb_raw;                   // registered read (rule A1)
   reg [PCW-1:0]  btb_qpc;                   // address the read was for
   integer bi;
   initial begin
      btb_raw = {EW{1'b0}}; btb_qpc = {PCW{1'b0}};
      for (bi = 0; bi < NBTB; bi = bi + 1) btb[bi] = {EW{1'b0}};
   end

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
   localparam NYAGS = 1 << YBITS, YEW = YTAGW + 2;
   function [YBITS-1:0]  yidx (input [PCW-1:0] a, input [GHL-1:0] h);
      yidx  = a[YBITS:1] ^ h[YBITS-1:0] ^ {{(YBITS-2){1'b0}}, h[GHL-1:YBITS]};
   endfunction
   function [YTAGW-1:0] ytagf(input [PCW-1:0] a);
      ytagf = a[YBITS+YTAGW:YBITS+1] ^ a[YBITS+2*YTAGW:YBITS+YTAGW+1] ^ {{(YTAGW-1){1'b0}}, a[63]};
   endfunction
   // NO VALID BIT, unlike the BTB above: the tag already carries it. An entry that has
   // never been written reads as tag 0, so it can only be believed by a PC whose
   // ytagf is also 0 -- 1 in 256 -- and what it then says is ctr=00, "not taken",
   // overriding the bimodal for exactly one execution of one branch. That resolve sets
   // yc_hit in the carried details, so `y_wr` fires, stores the same tag with a nudged
   // counter, and the entry is right from then on. A corrector is a hint whose whole job
   // is to be repaired by training; a valid bit here only buys the first execution of
   // 0.4% of cold branches, and costs a bit of array plus a term in the yhit AND.
   (* ram_style = "block" *)
   reg [YEW-1:0]  ycorr [0:NYAGS-1];         // {tag, ctr} -- validity IS the tag match
   reg [YEW-1:0]  ycorr_raw;
   integer yi;
   initial begin
      ycorr_raw = {YEW{1'b0}};
      for (yi = 0; yi < NYAGS; yi = yi + 1) ycorr[yi] = {YEW{1'b0}};
   end

   // ---------------------------------------- write-forward, applied AFTER the read register
   // A training write lands on the same edge as the read a cycle ahead of it, so the read
   // must see it. Forwarding on the ARRAY OUTPUT is what the previous version did, and it is
   // exactly what stops block-RAM inference: a BRAM's read register is INSIDE the primitive,
   // so a mux between array and flop forces the array back into LUTs. Register the decision
   // instead and mux in the cycle the entry is consumed -- the compare `write index == read
   // index` is evaluated against the same `npc` the read used, so it still means the same
   // thing one cycle later. It also DEFINES the read-during-write result, which a simple
   // dual-port BRAM leaves indeterminate in hardware.
   reg            t_fwd_q, y_fwd_q;
   reg [EW-1:0]   t_dat_q;
   reg [YEW-1:0]  y_dat_q;
   initial begin t_fwd_q = 1'b0; y_fwd_q = 1'b0; end
   wire [EW-1:0]  btb_q    = t_fwd_q ? t_dat_q : btb_raw;
   wire [YEW-1:0] ycorr_q  = y_fwd_q ? y_dat_q : ycorr_raw;

   // ------------------------------------------------- speculative state {ghr,ras}
   reg [GHL-1:0]  ghr;
   reg [PCW-1:0]  ras [0:RASN-1];
   reg [RASB-1:0] ras_ptr;                   // top of stack
   // No checkpoint ring. The in-order core has ONE commit point (M), so at most one
   // instruction can be redirecting and everything younger is squashed wholesale --
   // there is never a second speculative state to choose between. A committed copy of
   // the two scalars is all a redirect needs, and the RAS ARRAY needs nothing: with the
   // pointer restored, a wrong-path push landed above it and is unreachable.
   reg [GHL-1:0]  ghr_c;                     // committed history  (advanced at resolve)
   reg [RASB-1:0] rptr_c;                    // committed RAS top  (advanced at resolve)
   integer ii;
   initial begin
      ghr = {GHL{1'b0}}; ras_ptr = {RASB{1'b0}};
      for (ii = 0; ii < RASN; ii = ii + 1) ras[ii] = {PCW{1'b0}};
      ghr_c = {GHL{1'b0}}; rptr_c = {RASB{1'b0}};
   end

   // ------------------------------------------------------------------- predict
   // (registered state only -- see the timing-shape note above)
   wire            hit      = cti_ok & (btb_qpc == base_pc)
                            & (btb_q[EW-1 -: TAGW] == btag(base_pc));
   wire [2:0]      q_type   = btb_q[TGTW +: 3];
   wire [TGTW-1:0] q_tgt    = btb_q[TGTW-1:0];
   wire [PCW-1:0]  btb_tgt  = {{(PCW-TGTW-1){q_tgt[TGTW-1]}}, q_tgt, 1'b0};  // sign-extend canonical VA
   wire            p_cbr    = hit & ~q_type[2];             // known conditional branch
   wire            p_call   = hit & (q_type == TY_CALL);
   wire            p_ret    = hit & (q_type == TY_RET);
   // YAGS: a tag-hitting corrector overrides the bimodal weight for a conditional
   wire            yhit     = p_cbr & (ycorr_q[YEW-1 -: YTAGW] == ytagf(base_pc));
   wire            cbr_taken= yhit ? ycorr_q[1] : q_type[1];
   assign pred_v   = hit & (q_type[2] | (p_cbr & cbr_taken)); // uncond, or predicted-taken cond
   assign pred_tgt = p_ret ? ras[ras_ptr] : btb_tgt;
   wire            pred_dir = cbr_taken;                    // GHR shifts the committed direction

   // ------------------------------ predict details (for training), carried inline
   // Captured at fetch (cycle T) and presented on pd_fetch at T+1, where the frontend
   // latches it into the bundle. It then rides the pipeline to M and comes back as
   // res_pdet. Carrying it beats a side ring indexed by a tag: there is no tag to
   // allocate, nothing to pin against reuse, and no depth to get wrong when the
   // frontend gains a queue.
   //   bimodal [BIMW-1:0] = {hit,ctr,bidx,btag}  ·  yags [PDW-1 -: YW] = {yhit,yctr,yidx,ytag}
   localparam BIMW = 1 + 2 + BTBB + TAGW;
   localparam YW   = 1 + 2 + YBITS + YTAGW;
   // COMBINATIONAL, not registered: every term is a fetch-time value of the bundle being
   // presented right now, so the consumer latches it in the SAME cycle as `fire` and gets
   // this bundle's details. src/predictor.v registers it into pdet_f and writes the ring a
   // cycle later at `create`, which is why that version needs the lag; carrying the payload
   // removes both the lag and the ring.
   assign pd_fetch = {yhit, yctr_eff, yidx(base_pc, ghr), ytagf(base_pc),
                      hit,  ctr_eff,  bidx(base_pc),      btag(base_pc)};
   wire [1:0]    ctr_eff  = hit  ? q_type[1:0]  : 2'b01;    // miss -> install weakly-not-taken base
   wire [1:0]    yctr_eff = yhit ? ycorr_q[1:0] : 2'b01;

   // ------------------------------------------------- speculate / commit / restore
   always @(posedge clk) begin
      if (reset) begin
         ghr <= {GHL{1'b0}}; ras_ptr <= {RASB{1'b0}};
         ghr_c <= {GHL{1'b0}}; rptr_c <= {RASB{1'b0}};
      end else begin
         // COMMIT: the committed copies advance on every resolved CTI, redirect or not.
         // They are the only rollback state this core needs.
         if (res_v) begin
            if (res_cbr)  ghr_c  <= {ghr_c[GHL-2:0], res_taken};
            if (res_call) rptr_c <= rptr_c + 1'b1;
            if (res_ret)  rptr_c <= rptr_c - 1'b1;
         end

         if (rollback) begin
            // RESTORE from the committed scalars. This cycle's commit update has not
            // landed yet, so apply the resolving CTI's own effect here -- the same repair
            // the checkpoint version made to the snapshot's bit 0.
            ghr     <= (res_rep & res_cbr)  ? {ghr_c[GHL-2:0], res_taken} : ghr_c;
            ras_ptr <= (res_rep & res_call) ? rptr_c + 1'b1
                     : (res_rep & res_ret)  ? rptr_c - 1'b1 : rptr_c;
            // The RAS ARRAY is deliberately NOT restored. With the pointer back where it
            // belongs, a wrong-path push wrote at ptr+1 -- above the live region, where it
            // is unreachable. Only a wrong-path pop-then-push can clobber a live entry, and
            // the RAS is a hint: the cost is a mispredicted return, never a wrong answer,
            // because M resolves the truth.
         end else if (fire) begin            // SPECULATE: only on the fetch handshake
            if (p_cbr)  ghr <= {ghr[GHL-2:0], pred_dir};
            if (p_call) begin ras[ras_ptr + 1'b1] <= ft_npc; ras_ptr <= ras_ptr + 1'b1; end
            if (p_ret)  ras_ptr <= ras_ptr - 1'b1;
         end
      end
   end

   // ------------------------------------------------------------------ training
   // one BTB write per resolved CTI (oldest per cycle); read + write ports keep
   // the array 1R1W. Bimodal: nudge the (carried) counter toward the resolved
   // direction; uncond: record the class (call/return classified at resolve
   // from the executed instruction); target: the resolved taken-target.
   wire [PDW-1:0]  td      = res_pdet;   // carried with the instruction, not looked up
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
   // Decided here, applied a cycle later at btb_q/ycorr_q (see that comment).
   wire t_fwd = res_v && (t_idx == bidx(apc));
   wire y_fwd = y_wr  && (yc_idx == yidx(apc, ghr));        // same write-forward for the corrector
   // Read enable: exactly the cycles in which fetch's base PC MOVES. fetch advances pc_q on
   // reset, on a redirect (unconditionally -- it does not wait for the handshake), and
   // otherwise only on `fire`; every other cycle re-presents the same bundle, so holding the
   // entry is what keeps it matched to it. Without this a stall would re-read at the guessed
   // SUCCESSOR and the held-back bundle would lose the prediction it already had.
   //   This is where the late signal went. `fire` still comes off the aligner, but it now
   // arrives at a 1-bit RAM enable instead of steering a 10-bit index into an array -- and
   // `rollback` is M's registered redirect, which costs nothing.
   wire apc_en = fire | rollback | reset;
   always @(posedge clk) begin
      // Read a cycle ahead (rule A1). Nonblocking, so these see the array as it was BEFORE
      // this edge's write regardless of statement order -- the forward covers the collision.
      if (apc_en) begin
         btb_raw   <= btb[bidx(apc)];
         btb_qpc   <= apc;
         ycorr_raw <= ycorr[yidx(apc, ghr)];
         t_fwd_q   <= t_fwd;   t_dat_q <= {t_tag,  t_type, res_tgt[TGTW:1]};
         y_fwd_q   <= y_fwd;   y_dat_q <= {yc_tag, y_nudge};
      end
      if (res_v) btb[t_idx]    <= {t_tag,  t_type, res_tgt[TGTW:1]};
      if (y_wr)  ycorr[yc_idx] <= {yc_tag, y_nudge};
      // The arrays themselves are NOT cleared: see the BTB declaration for why a stale hint
      // is harmless. Only the forward flags need it, so a reset cannot inject a bogus entry.
      if (reset) begin t_fwd_q <= 1'b0; y_fwd_q <= 1'b0; end
   end
endmodule

`default_nettype wire
