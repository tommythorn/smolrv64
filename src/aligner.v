`default_nettype none

// Fetch-window aligner: carves up to IW variable-length instructions (RVC 16b /
// base 32b) out of a window of HW contiguous halfwords whose first halfword is
// at base_pc. Produces exactly the decode_stage input contract
// ({inst[31:0], in_valid, seq} per slot) plus per-slot PC and a `consumed`
// halfword count so fetch can advance.
//
// Positions: instruction k begins at the window's k-th START halfword (`st`, which the fetch ring
// writes as the bytes arrive), so every slot's position is known at once and no slot waits for the
// lengths before it. A 32-bit op (hw[p][1:0]==11) takes two halfwords, an RVC one.
//
// The data path is uniform: inst[k] is always the 32-bit window {hw[p+1],hw[p]};
// for an RVC the upper half is the next instruction's first halfword, which
// decode_slot ignores (it re-derives is_c from inst[1:0]). So only the *length*
// depends on [1:0] -- no data mux on is32, and no RVC expansion here.
//
// `avail` = how many halfwords of the window are actually present (normally HW;
// fewer at a fetch bubble or buffer edge). An instruction is valid only if all
// its halfwords are present, and validity is a prefix: the first instruction
// that doesn't fit (e.g. a 32-bit op straddling the window end) stops the bundle
// and is NOT counted in `consumed`, so its leading halfword carries to the next
// window. Fixed steering = slot k -> shard k (the slot order IS the steering).
module aligner
  #(parameter IW   = 4,
    parameter HW   = 8,                 // window halfwords (>= 2*IW for a full 32b bundle)
    parameter PCW  = 64,
    parameter SEQW = 8,
    parameter PBW_O = $clog2(HW+2))     // = PBW below; a port width needs it before the body
   (input  wire [HW*16-1:0]        hwin,
    input  wire [$clog2(HW+2)-1:0] avail,    // halfwords present (0..HW)
    input  wire [PCW-1:0]          base_pc,  // PC of hwin[0]
    input  wire [SEQW-1:0]         base_seq, // seq of slot 0
    input  wire                    solo_all, // force one instruction per bundle (fault replay)
    input  wire                    bytes_late, // the window is short of the page end: more is coming
    input  wire [HW-1:0]           st,       // halfwords that begin an instruction (smolrv64_fring; st[0] is
                                             // the PC, so set whenever a byte is held)
    input  wire [HW-1:0]           mk,       // halfwords that end a CTI the predictor knows (smolrv64_fring):
                                             // an instruction ending on one predicted taken ends the
                                             // bundle, and one whose FIRST halfword is marked does not fit
    input  wire [IW-1:0]           mk_tk,    // the window's marks' predictions in order: taken
    output wire [IW-1:0]           valid,
    output wire [IW*32-1:0]        inst,
    output wire [IW*PCW-1:0]       pc,
    output wire [IW*PBW_O-1:0]     offs,     // each slot's halfword offset from base_pc (fetch builds
                                             // the PCs from precomputed sums: the adder here is
                                             // for the benches, and off the fetch path)
    output wire [IW*SEQW-1:0]      seq,
    output wire [$clog2(HW+2)-1:0] consumed,
    output wire                    br_term,   // bundle ends on a genuine branch/jump (not a
                                              // window cut or a SYSTEM/AMO/FENCE terminator)
    output wire                    mk_term,   // bundle ends on a marked halfword
    output wire [IW-1:0]           smk,       // per slot: it ends on a marked halfword
    output wire [IW-1:0]           sbr,       // per slot: it is a genuine branch/jump
    // ...and per slot from the window alone, whether or not the bundle reaches it, for the selects
    // that follow the bundle's run (fetch): its next position, the marks before it (its own
    // prediction's index), that it ends on a mark, on one predicted taken, and is a branch or jump
    output wire [IW*PBW_O-1:0]     nxt,
    output wire [IW*PBW_O-1:0]     smi,
    output wire [IW-1:0]           semk,
    output wire [IW-1:0]           setk,
    output wire [IW-1:0]           sbrw);

   localparam PBW = $clog2(HW+2);        // holds a position 0..HW (+1 for lookahead)

   // halfword read with out-of-window guard (returns 0 past the window)
   function [15:0] hwr(input [PBW-1:0] idx);
      hwr = (idx < HW) ? hwin[idx*16 +: 16] : 16'b0;
   endfunction
   function mkr(input [PBW-1:0] idx);
      mkr = (idx < HW) ? mk[idx[$clog2(HW)-1:0]] : 1'b0;
   endfunction

   // End-of-bundle predecode (opcode bits only -- no full RVC expansion). A jump, or a
   // serializing op (SYSTEM / AMO / FENCE), *terminates* the bundle: it is the last valid slot,
   // and the rest reappears as slot 0 of the next window (consumed stops at it, like the
   // straddle case). A conditional branch does not, unless it ends on a mark predicted taken:
   // one the predictor calls not taken, or does not know, is predicted to fall through, and the
   // bundle reads on. A branch with younger instructions in its own group is what the core
   // recovers from by its ROB row's columns (smolrv64_rob's kill and walk).
   function is_cti(input [15:0] h0);
      if (h0[1:0] == 2'b11)                 // base 32-bit op (h0[1:0]==11 IS the old `is32` input)
         is_cti = (h0[6:0] == 7'b1100011)   // BRANCH
                | (h0[6:0] == 7'b1101111)   // JAL
                | (h0[6:0] == 7'b1100111)   // JALR
                | (h0[6:0] == 7'b1110011)   // SYSTEM (ecall/ebreak/csr/xret/wfi/sfence)
                | (h0[6:0] == 7'b0101111)   // AMO (atomic: solo, serialized RMW)
                | ((h0[6:0] == 7'b0001111) & (h0[14:13] == 2'b00)); // FENCE / FENCE.I (barrier; excl CBO f3=010)
      else case (h0[1:0])
         2'b01:   is_cti = (h0[15:13] == 3'b101)    // C.J
                         | (h0[15:13] == 3'b110)    // C.BEQZ
                         | (h0[15:13] == 3'b111);   // C.BNEZ
         2'b10:   is_cti = (h0[15:13] == 3'b100)     // C.JR / C.JALR / C.EBREAK
                         & (h0[6:2] == 5'd0);
         default: is_cti = 1'b0;
      endcase
   endfunction

   // Genuine branch/jump (the ops branch_unit's actual_npc != pred_npc compare
   // covers). The predictor may only steer fetch off a bundle ending in one of
   // these: a prediction on any other bundle shape (a stale BTB entry over
   // changed code, a SYSTEM/AMO/FENCE terminator, C.EBREAK) would have NO
   // corrective compare at execute and the wrong path would retire.
   // A conditional branch (BRANCH, C.BEQZ, C.BNEZ).
   function is_cond(input [15:0] h0);
      if (h0[1:0] == 2'b11) is_cond = (h0[6:0] == 7'b1100011);
      else                  is_cond = (h0[1:0] == 2'b01) & ((h0[15:13] == 3'b110) | (h0[15:13] == 3'b111));
   endfunction

   function is_br(input [15:0] h0);
      if (h0[1:0] == 2'b11)
         is_br = (h0[6:0] == 7'b1100011)    // BRANCH
               | (h0[6:0] == 7'b1101111)    // JAL
               | (h0[6:0] == 7'b1100111);   // JALR
      else case (h0[1:0])
         2'b01:   is_br = (h0[15:13] == 3'b101)    // C.J
                        | (h0[15:13] == 3'b110)    // C.BEQZ
                        | (h0[15:13] == 3'b111);   // C.BNEZ
         2'b10:   is_br = (h0[15:13] == 3'b100)    // C.JR / C.JALR
                        & (h0[6:2] == 5'd0)
                        & (h0[11:7] != 5'd0);      // rs1==0 is C.EBREAK
         default: is_br = 1'b0;
      endcase
   endfunction

   // EVERY HALFWORD DECODED AT ONCE, EVERY SLOT A SELECT. `st` says which halfwords begin an
   // instruction (the fetch ring writes it as the bytes arrive: a halfword begins one unless the
   // one before began a 32-bit op), so slot k is the k-th start in the window and no slot's
   // position waits for the lengths before it. The per-halfword facts are decoded for all HW
   // positions in parallel; a slot reads its position's by a one-hot select; and the only chain
   // across slots is the bundle's run, a gate or two per slot.
   localparam CW = $clog2(HW + 1);
   reg  [15:0]   h0;
   reg  [HW-1:0] p32, phave, pmkin, psys, pend, pbr, pemk;   // per halfword, as a start
   reg  [CW-1:0] cs [0:HW];                                  // starts before halfword i
   reg  [CW-1:0] cm [0:HW];                                  // marks before halfword i
   integer i;
   always @* begin
      cs[0] = {CW{1'b0}};  cm[0] = {CW{1'b0}};
      for (i = 0; i < HW; i = i + 1) begin
         h0       = hwin[i*16 +: 16];
         p32[i]   = (h0[1:0] == 2'b11);
         phave[i] = (i < avail) && (!p32[i] || ((i + 1) < avail));
         pmkin[i] = p32[i] & mk[i];                          // a mark inside it: what follows is another path's
         psys[i]  = p32[i] && ((h0[6:0] == 7'b1110011) || (h0[6:0] == 7'b0101111)
                               || ((h0[6:0] == 7'b0001111) && (h0[14:13] == 2'b00)));
         pend[i]  = is_cti(h0) & ~is_cond(h0);               // a jump, SYSTEM, AMO or FENCE ends the bundle
         pbr[i]   = is_br(h0);
         pemk[i]  = p32[i] ? mkr(i[PBW-1:0] + 1'b1) : mk[i];             // it ends on a marked halfword
         cs[i+1]  = cs[i] + {{(CW-1){1'b0}}, st[i]};
         cm[i+1]  = cm[i] + {{(CW-1){1'b0}}, mk[i]};
      end
   end

   // slot k: its start (one-hot over the window; none when the k-th start is past it) and what
   // that halfword says
   reg  [HW-1:0]    ps   [0:IW-1];
   reg  [IW-1:0]    s_in, s_have, s_mkin, s_sys, s_end, s_br, s_emk, s_tk;
   reg  [PBW-1:0]   s_pos[0:IW-1], s_nxt[0:IW-1];
   reg  [CW-1:0]    s_mi [0:IW-1];
   reg  [31:0]      ir   [0:IW-1];
   reg  [PCW-1:0]   pcv  [0:IW-1];
   reg  [SEQW-1:0]  sqv  [0:IW-1];
   reg  [IW-1:0]    v, smv, sbv;
   reg  [PBW-1:0]   cons;
   reg              bt, mt, wt, run;
   reg  [CW-1:0]    mi;
   integer k, q;
   always @* begin
      for (k = 0; k < IW; k = k + 1) begin
         s_in[k] = 1'b0;  s_have[k] = 1'b0;  s_mkin[k] = 1'b0;  s_sys[k] = 1'b0;
         s_end[k] = 1'b0;  s_br[k] = 1'b0;  s_emk[k] = 1'b0;  mi = {CW{1'b0}};
         s_pos[k] = {PBW{1'b0}};  s_nxt[k] = {PBW{1'b0}};  ir[k] = 32'd0;
         for (q = 0; q < HW; q = q + 1) begin
            ps[k][q] = st[q] & (cs[q] == k[CW-1:0]);
            if (ps[k][q]) begin
               s_in[k] = 1'b1;  s_have[k] = phave[q];  s_mkin[k] = pmkin[q];  s_sys[k] = psys[q];
               s_end[k] = pend[q];  s_br[k] = pbr[q];  s_emk[k] = pemk[q];  mi = cm[q];
               s_pos[k] = q[PBW-1:0];  s_nxt[k] = q[PBW-1:0] + (p32[q] ? 2'd2 : 2'd1);
               ir[k] = {hwr(q[PBW-1:0] + 1'b1), hwin[q*16 +: 16]};
            end
         end
         s_tk[k] = mk_tk[mi[$clog2(IW)-1:0]];               // this slot's mark is the mi-th
         s_mi[k] = mi;
         pcv[k]  = base_pc + (s_pos[k] << 1);
         sqv[k]  = base_seq + k[SEQW-1:0];
      end
      // the bundle: slots while each fits and the one before did not end it. A SYSTEM op (or a
      // fault replay) goes alone, so it begins a fresh bundle; a later slot whose bytes are coming
      // makes the whole bundle wait (bytes_late) so its shape is the code's, not the timing's.
      run = 1'b1;  bt = 1'b0;  mt = 1'b0;  wt = 1'b0;  v = {IW{1'b0}};  smv = {IW{1'b0}};  sbv = {IW{1'b0}};
      cons = {PBW{1'b0}};
      for (k = 0; k < IW; k = k + 1) begin
         if (((k != 0) && (solo_all || (s_have[k] && s_sys[k]))) || s_mkin[k]) run = 1'b0;
         else begin
            v[k] = run & s_in[k] & s_have[k];
            if (v[k]) begin
               cons = s_nxt[k];
               sbv[k] = s_br[k];  bt = bt | s_br[k];  smv[k] = s_emk[k];
               if (s_end[k]) run = 1'b0;
               if (s_emk[k] & s_tk[k]) begin run = 1'b0;  mt = 1'b1; end
            end else begin
               if ((k != 0) && run && bytes_late) wt = 1'b1;
               run = 1'b0;
            end
         end
      end
      if (wt) begin v = {IW{1'b0}}; bt = 1'b0; mt = 1'b0; smv = {IW{1'b0}}; sbv = {IW{1'b0}}; cons = {PBW{1'b0}}; end
   end

   genvar g;
   generate
      for (g = 0; g < IW; g = g + 1) begin : pk
         assign inst[g*32  +: 32]   = ir[g];
         assign pc  [g*PCW +: PCW]  = pcv[g];
         assign offs[g*PBW +: PBW]  = s_pos[g];
         assign seq [g*SEQW +: SEQW] = sqv[g];
         assign nxt [g*PBW +: PBW]  = s_nxt[g];
         if (PBW > CW) begin : mw  assign smi[g*PBW +: PBW] = {{(PBW-CW){1'b0}}, s_mi[g]}; end
         else          begin : me  assign smi[g*PBW +: PBW] = s_mi[g]; end
      end
   endgenerate
   assign valid    = v;
   assign consumed = cons;
   assign br_term  = bt;
   assign mk_term  = mt;
   assign smk      = smv;
   assign sbr      = sbv;
   assign semk     = s_emk;
   assign setk     = s_emk & s_tk;
   assign sbrw     = s_br;
endmodule

`default_nettype wire
