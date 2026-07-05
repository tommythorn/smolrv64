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
// Prediction is bundle-granular: the aligner ends every bundle at its first CTI,
// so a bundle has at most one control transfer and it is the LAST valid slot.
// The class (cond-branch / jump / call / return) is decoded here from the raw
// instruction bytes of that slot -- authoritative, no BTB "learning" of types
// needed for RAS correctness -- and prediction is simply suppressed when the
// last slot is not a real branch/jump (window cut, SYSTEM/FENCE/AMO terminator,
// the interrupt pseudo-op).
//
// Timing rule (plan A1): the BTB RAM read terminates at a register. `npc` is
// fetch's combinationally-computed next PC; we read BTB[npc] and register the
// entry (+ the address it was read for), so the prediction for the bundle at
// pc_q uses last cycle's read. A read-for-the-wrong-address (weird path) fails
// the registered-address compare and is treated as a miss.
//
// Training is resolve-time only (BTB is a cache, never rolled back): exec_bundle
// exports the oldest genuinely-resolved CTI per cycle {taken, taken-target,
// ckpt}; the predict-time details (index/tag/hit/ctr) are looked up in a small
// per-checkpoint table written at dispatch -- ≤1 CTI per bundle == per ckpt, so
// the checkpoint tag the op already carries is the resolve key. No payload bits.
module predictor
  #(parameter IW    = 4,
    parameter PCW   = 64,
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
    input  wire [IW-1:0]        slot_valid, // presented bundle (post-mux: aligner or straddle)
    input  wire [IW*32-1:0]     inst,
    input  wire [IW*PCW-1:0]    pc,
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
    input  wire                 res_taken,
    input  wire [CBITS-1:0]     res_ckpt,
    input  wire [PCW-1:0]       res_tgt,     // taken-target (train the BTB)
    input  wire                 res_rep);    // this resolve caused this cycle's rollback ->
                                             // repair the restored GHR's bit 0 (plan B2)

   localparam NBTB = 1 << BTBB;
   localparam RASN = 1 << RASB;
   // class encoding (predict-side + pdet)
   localparam [2:0] CL_NONE = 3'd0, CL_CBR = 3'd1, CL_JMP = 3'd2, CL_CALL = 3'd3, CL_RET = 3'd4;
   // BTB type: 0xx = cond branch, xx = 2-bit bimodal (00 S_N .. 11 S_T); 1xx = uncond
   localparam [2:0] TY_JMP = 3'b100;

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

   function [TAGW-1:0] btag(input [PCW-1:0] a); btag = a[BTBB+TAGW:BTBB+1]; endfunction
   function [BTBB-1:0] bidx(input [PCW-1:0] a); bidx = a[BTBB:1];           endfunction

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

   // --------------------------------------- last valid slot -> CTI class (bytes)
   reg [31:0]    cti_i;
   reg [PCW-1:0] cti_pc;
   integer ls;
   always @* begin
      cti_i = 32'd0; cti_pc = {PCW{1'b0}};
      for (ls = 0; ls < IW; ls = ls + 1)
         if (slot_valid[ls]) begin cti_i = inst[ls*32 +: 32]; cti_pc = pc[ls*PCW +: PCW]; end
   end
   function islink(input [4:0] r); islink = (r == 5'd1) || (r == 5'd5); endfunction
   // RVC slots carry {next-insn-halfword, rvc-halfword}: classify from the low 16
   // bits when [1:0]!=11 (the same rule the aligner's is_cti uses).
   reg [2:0] cls;
   always @* begin
      cls = CL_NONE;
      if (cti_i[1:0] == 2'b11) case (cti_i[6:0])
         7'b1100011: cls = CL_CBR;                                    // BRANCH
         7'b1101111: cls = islink(cti_i[11:7]) ? CL_CALL : CL_JMP;    // JAL
         7'b1100111: cls = islink(cti_i[11:7]) ? CL_CALL              // JALR: rd link -> call
                         : islink(cti_i[19:15]) ? CL_RET : CL_JMP;    //   rs1 link -> return
         default: ;
      endcase
      else case ({cti_i[1:0], cti_i[15:13]})
         5'b01_101: cls = CL_JMP;                                     // C.J
         5'b01_110, 5'b01_111: cls = CL_CBR;                          // C.BEQZ / C.BNEZ
         5'b10_100: if (cti_i[6:2] == 5'd0 && cti_i[11:7] != 5'd0)    // C.JR / C.JALR (not C.EBREAK)
                       cls = cti_i[12] ? CL_CALL                      // C.JALR (rd=x1)
                           : (islink(cti_i[11:7]) ? CL_RET : CL_JMP); // C.JR: rs1 link -> return
         default: ;
      endcase
   end
   wire        is32     = (cti_i[1:0] == 2'b11);
   wire [PCW-1:0] ret_addr = cti_pc + (is32 ? 64'd4 : 64'd2);

   // ------------------------------------------------------------------- predict
   wire            hit      = btb_qv & (btb_qpc == base_pc) & (btb_q[EW-1 -: TAGW] == btag(base_pc));
   wire [2:0]      q_type   = btb_q[TGTW +: 3];
   wire [TGTW-1:0] q_tgt    = btb_q[TGTW-1:0];
   wire [PCW-1:0]  btb_tgt  = {{(PCW-TGTW-1){q_tgt[TGTW-1]}}, q_tgt, 1'b0};  // sign-extend canonical VA
   // conditional direction: bimodal MSB (a stale uncond-typed alias predicts taken; self-corrects)
   wire            cbr_take = hit & (q_type[2] | q_type[1]);
   assign pred_v   = (cls == CL_CBR) ? cbr_take
                   : (cls == CL_RET) ? 1'b1                 // RAS needs no BTB entry
                   : (cls != CL_NONE) & hit;                // JAL/JALR/call: need the target
   assign pred_tgt = (cls == CL_RET) ? ras[ras_ptr] : btb_tgt;
   wire            pred_dir = pred_v;                       // GHR shift bit for a cond branch

   // ------------------------- per-checkpoint predict details (for training/repair)
   // captured at fetch (cycle T), written to pdet[cur] at the bundle's create
   // (T+1) -- the same one-stage lag as the {ghr,ras} snapshot (plan B1).
   localparam PDW = 1 + 2 + BTBB + TAGW;                    // {hit, ctr, idx, tag}
   reg [PDW-1:0] pdet_f;
   reg [PDW-1:0] pdet [0:NCHK-1];
   wire [1:0]    ctr_eff = hit ? q_type[1:0] : 2'b01;       // miss -> install weakly-not-taken base
   initial pdet_f = {PDW{1'b0}};

   // ------------------------------------------------- speculate / snapshot / restore
   wire [CBITS-1:0] nxt  = cur + 1'b1;
   wire             push = fire & (cls == CL_CALL);
   wire             pop  = fire & (cls == CL_RET);
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
         // the resolved one (exact: one CTI per bundle). Jumps/traps: verbatim.
         ghr     <= res_rep ? {chk_ghr[rollback_idx][GHL-1:1], res_taken}
                            :  chk_ghr[rollback_idx];
         ras_ptr <= chk_rptr[rollback_idx];
         for (k = 0; k < RASN; k = k + 1) ras[k] <= chk_ras[rollback_idx][k];
      end else begin
         if (fire) begin                     // speculate: advance ONLY on the fetch handshake
            if (cls == CL_CBR) ghr <= {ghr[GHL-2:0], pred_dir};
            if (push) begin ras[ras_ptr + 1'b1] <= ret_addr; ras_ptr <= ras_ptr + 1'b1; end
            if (pop)  ras_ptr <= ras_ptr - 1'b1;
            pdet_f <= {hit, ctr_eff, bidx(base_pc), btag(base_pc)};
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
   // direction; uncond: record the class; target: the resolved taken-target.
   wire [PDW-1:0]  td      = pdet[res_ckpt];
   wire            t_hit   = td[PDW-1];
   wire [1:0]      t_ctr   = td[PDW-2 -: 2];
   wire [BTBB-1:0] t_idx   = td[TAGW +: BTBB];
   wire [TAGW-1:0] t_tag   = td[TAGW-1:0];
   wire [1:0]      t_base  = t_hit ? t_ctr : (res_taken ? 2'b10 : 2'b01);  // miss -> install weak
   wire [1:0]      t_nudge = res_taken ? ((t_base == 2'b11) ? 2'b11 : t_base + 1'b1)
                                       : ((t_base == 2'b00) ? 2'b00 : t_base - 1'b1);
   // uncond entries all store TY_JMP: the predict-side class comes from the
   // instruction bytes, so the BTB type only needs to distinguish cond (counter)
   // from uncond (static taken) -- type[2].
   wire [2:0]      t_type  = res_cbr ? {1'b0, t_nudge} : TY_JMP;

   // write-forward: a mispredict's redirected refetch reads the BTB the same edge
   // its own training write lands -- without forwarding the retrained entry is
   // invisible to that first refetch and every cold-taken CTI mispredicts twice.
   wire t_fwd = res_v && (t_idx == bidx(npc));
   always @(posedge clk) begin
      btb_q   <= t_fwd ? {t_tag, t_type, res_tgt[TGTW:1]} : btb[bidx(npc)];
      btb_qv  <= t_fwd ? 1'b1 : btb_v[bidx(npc)];
      btb_qpc <= npc;
      if (res_v) begin
         btb[t_idx]   <= {t_tag, t_type, res_tgt[TGTW:1]};
         btb_v[t_idx] <= 1'b1;
      end
      if (reset) btb_v <= {NBTB{1'b0}};
   end
endmodule

`default_nettype wire
