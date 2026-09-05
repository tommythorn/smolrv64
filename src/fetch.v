`default_nettype none

// Minimal fetch unit: sequences the PC, reads an HW-halfword window starting at
// PC, and aligns it into a bundle for decode. Runs fall-through unless the
// frontend predictor overrides (pred_v/pred_tgt, for a bundle ending on a
// predicted-taken CTI); corrected by `redirect` (branch mispredict / exception /
// CPR rollback). PC is the only architectural state here.
//
// Carry-free windowing: the window always *starts at PC*, and PC advances by
// 2*consumed. The aligner excludes a window-straddling 32-bit op from
// `consumed`, so that op's first halfword simply reappears as slot 0 of the next
// window -- no leftover/shift buffer, the straddle special-case folds away.
//
// PAGE-CROSS: a 32-bit instruction whose first halfword is the LAST halfword of a
// page (VA offset 0xFFE) has its second halfword in the NEXT page, which needs its
// own address translation. We handle it the same way simmerv (memop_code) does --
// a LAZY two-step fetch, never speculative:
//   1. The window is capped at the page boundary (`eff_avail`), so the aligner
//      never aligns instructions from beyond the translated page (no bytes from a
//      wrong contiguous-PA page, no speculative next-page access).
//   2. When slot 0 IS such a straddler -- pc_q at offset 0xFFE and a genuine 32-bit
//      op (low 2 bits == 11) -- we enter the STRADDLE state: latch the low halfword,
//      present PC+2 to fetch (and translate) the high halfword from the next page,
//      then emit the combined 32-bit op as a one-instruction bundle and advance by 4.
// A compressed op at 0xFFE (low 2 bits != 11) is complete in this page: the aligner
// emits it from the single available halfword and we NEVER touch the next page, so
// an unmapped next page raises no fault. If the next page is unmapped for a real
// 32-bit straddler, the fetch stalls and the iMMU's fault is delivered precisely:
// epc = the instruction PC (imem_ipc = pc_q), tval = the faulting VA (imem_addr =
// pc_q+2) -- exactly simmerv's mepc/mtval for a page-crossing instruction fault.
//
// Instruction memory is an external combinational read (imem_addr -> imem_data /
// imem_avail) so the real dual-bank I$ drops in later. Downstream handshake is
// ready/valid: `valid` holds the presented bundle until `ready` (golden rule:
// valid is not a function of ready). `redirect` takes priority over advance.
module fetch
  #(parameter IW   = 4,
    parameter HW   = 8,
    parameter PCW  = 64,
    parameter SEQW = 8,
    parameter [PCW-1:0] RESET_PC = 0)
   (input  wire                    clk,
    input  wire                    reset,
    // redirect: backend supplies the resume PC and program-order seq
    input  wire                    redirect,
    input  wire [PCW-1:0]          redirect_pc,
    input  wire [SEQW-1:0]         redirect_seq,
    input  wire                    solo_all,    // align one instruction per bundle (fault replay)
    // interrupt injection: present a synthetic solo SYSTEM op (the interrupt pseudo-
    // instruction) at the current PC, WITHOUT advancing PC -- the displaced real
    // instruction re-fetches after the handler returns (mepc = this PC).
    input  wire                    irq_inject,
    // branch prediction: when the presented bundle ends on a predicted-taken CTI,
    // the predictor overrides the fall-through advance. `npc` is the computed
    // next PC (all arms, redirect included) -- the predictor's BTB read address.
    // `pred_npc` is the next PC actually chosen for the PRESENTED bundle; it
    // rides to dispatch and is the exec-side mispredict reference (branch_unit
    // redirects iff actual_npc != pred_npc).
    input  wire                    pred_v,
    input  wire                    apred_v,     // pred_v computed from REGISTERS ONLY (no aligner
                                                // term) -- the `apc` arm. See apc below.
    input  wire [PCW-1:0]          pred_tgt,
    output wire [PCW-1:0]          npc,
    output wire [PCW-1:0]          apc,         // npc PREDICTED from registers only -- see below
    output wire [PCW-1:0]          pred_npc,
    // pred_npc WITHOUT THE SUM: which of {pc + length, pred_tgt, pc} pred_npc is. A consumer
    // that already knows the instruction's length (decode does) rebuilds pred_npc from this
    // and pred_tgt and leaves the +2*consumed adder -- eight CARRY8 at the END of the fetch
    // cloud -- out of whatever it stores. ooo2_frontend's F/X queue is that consumer: its
    // write data was the design's second-worst family on 2026-09-03 (342 endpoints, 25
    // levels, 13 CARRY8, iMMU -> fetch buffer -> aligner -> this adder -> LUTRAM data pin).
    //   0 = fall-through: pc + the presented instruction's length (also the straddle's +4)
    //   1 = the predicted target (pred_tgt)
    //   2 = this PC (the interrupt pseudo-op holds it)
    output wire [1:0]              pnpc_kind,
    output wire [PCW-1:0]          ft_npc,      // presented bundle's fall-through (RAS ret addr)
    output wire                    br_term,     // presented bundle ends on a real branch/jump
                                                // (prediction is only safe on such bundles)
    // instruction memory (combinational read of HW halfwords at imem_addr)
    output wire [PCW-1:0]          imem_addr,
    output wire [PCW-1:0]          imem_ipc,    // PC of the instruction being fetched (fault EPC)
    input  wire [HW*16-1:0]        imem_data,
    input  wire [$clog2(HW+2)-1:0] imem_avail,
    // downstream handshake + aligned bundle
    input  wire                    ready,
    output wire                    valid,
    output wire [IW-1:0]           slot_valid,
    output wire [IW*32-1:0]        inst,
    output wire [IW*PCW-1:0]       pc,
    output wire [IW*SEQW-1:0]      seq,
    output wire [SEQW-1:0]         cur_seq);    // PC register's seqno (for trap resume)

   localparam PBW = $clog2(HW+2);
   // SYSTEM, funct3=000, imm[11:0]=OP_IRQ(0x7F0), rs1=rd=0 -> the interrupt pseudo-op.
   localparam [31:0] IRQ_INSN = 32'h7F00_0073;

   reg [PCW-1:0]  pc_q;
   reg [SEQW-1:0] seq_q;
   // page-cross straddle state: `strad`=1 means we are fetching the high halfword of
   // a 32-bit op from PC+2 (the next page); `strad_lo` holds its already-read low half.
   reg            strad;
   reg [15:0]     strad_lo;
   reg [PCW-1:0]  pc2_q;              // pc_q + 2, captured when the straddle is entered
   initial begin pc_q = RESET_PC; seq_q = 0; strad = 1'b0; pc2_q = RESET_PC + 64'd2; end

   // halfwords from pc_q to the 4 KiB page boundary; cap the aligner's view there.
   wire [11:0]    off      = pc_q[11:0];
   wire           at_bound = (off == 12'hFFE);                  // pc_q is the page's last halfword
   wire [12:0]    hw_bound = (13'd4096 - {1'b0, off}) >> 1;     // 1..2048 halfwords to the boundary
   wire [PBW-1:0] hw_cap   = (hw_bound > HW) ? HW[PBW-1:0] : hw_bound[PBW-1:0];
   wire [PBW-1:0] eff_avail = (imem_avail < hw_cap) ? imem_avail : hw_cap;

   assign cur_seq   = seq_q;
   // While straddling, present PC+2 so the iMMU translates the high halfword's (next) page.
   // FMAX: pc2_q, not a live `pc_q + 2`. This adder was the first two CARRY8 stages of
   // the worst path in the design -- pc_q -> +2 -> u_immu/req_match -> fetch buffer ->
   // I$ data -> aligner -> npc -> predictor read, 28 levels in one cycle. It is pure
   // waste there: `strad` is only ever high while pc_q is FROZEN (the straddle state
   // holds PC until `fire`, and redirect/irq_inject both clear strad), so the sum can be
   // computed once when the straddle is ENTERED, off the critical path, instead of every
   // cycle at the head of it. Asserted below rather than argued.
   assign imem_addr = (strad & ~irq_inject) ? pc2_q : pc_q;
   assign imem_ipc  = pc_q;     // the instruction's PC in both states (trap EPC)

   // ---- aligner over the page-capped window (used in the NORMAL state) ----
   wire [PBW-1:0]    al_consumed;
   wire [IW-1:0]     al_valid;
   wire [IW*32-1:0]  al_inst;
   wire [IW*PCW-1:0] al_pc;
   wire [IW*PBW-1:0] al_offs;
   wire [IW*SEQW-1:0] al_seq;
   wire al_br_term;
   aligner #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW)) u_al
     (.hwin(imem_data), .avail(eff_avail), .base_pc(pc_q), .base_seq(seq_q),
      .solo_all(solo_all),
      .valid(al_valid), .inst(al_inst), .pc(al_pc), .offs(al_offs), .seq(al_seq), .consumed(al_consumed),
      .br_term(al_br_term));
   // THE FALL-THROUGH AND THE SLOT PCs ARE MUXES, NOT ADDERS (2026-09-05). Gate U, the first
   // build of the two-wide fetch, failed at -0.620 ns on pc_q -> iMMU -> fetch buffer ->
   // aligner -> +2*consumed -> the RAS write and the F/X queue's slot-1 PC: 23 levels with
   // ten CARRY8, three quarters of it route. pc_q is a register, so pc_q + 2i for i = 0..2*IW
   // is ready a nanosecond into the cycle; the aligner's count and offsets then SELECT one.
   // al_pc (the aligner's own sum) is kept for the benches and is unused here.
   // A bundle consumes at most the WINDOW, so the sums run to min(2*IW, HW) halfwords -- and
   // that bound is what keeps the loop index inside the count's width: the first cut ran to
   // 2*IW, and at the riscv-tests' HW=2 window (2-bit counts) index 4 read as 0, matched
   // offset 0, and every PC came out pc_q + 8 while the HW=8 cosim was exact.
   localparam integer NP = (2*IW < HW) ? 2*IW : HW;
   wire [PCW-1:0] pc_plus [0:NP];
   genvar gp;
   generate for (gp = 0; gp <= NP; gp = gp + 1) begin : pcp
      localparam [PCW-1:0] K = 2*gp;
      assign pc_plus[gp] = pc_q + K;
   end endgenerate
   // Explicit selects, not a function: a function reading pc_q and pc_plus from module scope
   // is invisible to `always @*` (iverilog's list, and the 240 riscv-tests that timed out
   // under Verilator on the first cut agree), so the block names everything it reads.
   reg  [IW*PCW-1:0] al_pcm;
   reg  [PCW-1:0]    ft_sel;
   integer ps, pi;
   always @* begin
      ft_sel = pc_q;
      for (pi = 1; pi <= NP; pi = pi + 1)
         if (al_consumed == pi[PBW-1:0]) ft_sel = pc_plus[pi];
      for (ps = 0; ps < IW; ps = ps + 1) begin
         al_pcm[ps*PCW +: PCW] = pc_q;
         for (pi = 1; pi <= NP; pi = pi + 1)
            if (al_offs[ps*PBW +: PBW] == pi[PBW-1:0]) al_pcm[ps*PCW +: PCW] = pc_plus[pi];
      end
   end
   // straddle/irq bundles bypass the aligner: never predict on them (the straddle
   // FSM owns its +4 advance; the pseudo-op holds PC).
   assign br_term = al_br_term & ~strad & ~irq_inject;

   // slot-0 page-boundary straddler: pc_q at the last halfword, a 32-bit op (low2==11),
   // and that low halfword actually present (an I$ hit). The aligner excludes it (the
   // capped window has only one halfword), so we take over with the two-step fetch.
   wire lo_avail     = (imem_avail >= 1'b1);
   wire straddle_det = ~strad & at_bound & (imem_data[1:0] == 2'b11) & lo_avail & ~irq_inject;

   // straddle output: the high halfword has arrived (PC+2's page resolved) in imem_data[15:0].
   wire              strad_ready = strad & lo_avail;
   wire [IW*32-1:0]  strad_inst  = {{((IW-1)*32){1'b0}}, imem_data[15:0], strad_lo};

   // bundle mux: irq inject (solo pseudo-op) > straddle (combined op) > aligner.
   assign slot_valid = irq_inject ? {{(IW-1){1'b0}}, 1'b1}
                     : strad      ? (strad_ready ? {{(IW-1){1'b0}}, 1'b1} : {IW{1'b0}})
                     :              al_valid;
   assign inst       = irq_inject ? {{((IW-1)*32){1'b0}}, IRQ_INSN}
                     : strad      ? strad_inst
                     :              al_inst;
   assign pc         = (irq_inject | strad) ? {{((IW-1)*PCW){1'b0}}, pc_q} : al_pcm;
   assign seq        = (irq_inject | strad) ? {{((IW-1)*SEQW){1'b0}}, seq_q} : al_seq;

   assign valid = |slot_valid;          // a bundle is present iff >=1 instr aligned
   wire   fire  = ready && valid;        // advance only on a downstream handshake

   integer c;
   reg [SEQW-1:0] nvalid;
   always @* begin
      nvalid = 0;
      for (c = 0; c < IW; c = c + 1) nvalid = nvalid + slot_valid[c];
   end

   // normal-path advance: predicted-taken CTI -> target, else fall-through. The
   // straddle/irq arms of the advance chain come first, so pred_v is naturally
   // ignored there (the straddle FSM owns its +4; the pseudo-op holds PC).
   assign         ft_npc   = ft_sel;                                            // += 2*consumed, by selection
   wire [PCW-1:0] norm_npc = pred_v ? pred_tgt : ft_npc;
   // the presented bundle's chosen next PC (mispredict reference at execute)
   assign pred_npc = irq_inject ? pc_q
                   : strad      ? pc_plus[2]
                   :              norm_npc;
   // the same choice, as a selector (the straddle's +4 IS its 32-bit instruction's length)
   assign pnpc_kind = irq_inject ? 2'd2 : strad ? 2'd0 : pred_v ? 2'd1 : 2'd0;
   // computed next PC, mirroring the advance chain's priorities exactly -- this
   // is the predictor's BTB read address (registered there, rule A1). The
   // redirect arm MUST be included: without it the first bundle at a redirect
   // target never gets a valid BTB read, and a hot loop re-entered by its own
   // mispredict never re-engages prediction (perpetual mispredict). Only
   // npc[8:1] reaches the BTB address pins (synthesis slices the mux), so the
   // redirect cone's contribution here is a few address bits, not a 64-bit bus.
   assign npc = reset        ? RESET_PC
              : redirect     ? redirect_pc
              : irq_inject   ? pc_q
              : strad        ? (fire ? pc_plus[2] : pc_q)
              : straddle_det ? pc_q
              : fire         ? norm_npc : pc_q;

   // ------------------------------------------------ AHEAD PC (the predictor's read address)
   // `npc` above is the TRUE next PC, and it is not known until the I$ data has been
   // aligned. Measured at 166 MHz: iMMU translate -> I$ data -> aligner puts ft_npc 4.35 ns
   // into a 6.245 ns budget, and hanging the predictor's array read off the end of that is
   // what pins Fmax. `apc` is the same value predicted from state already registered at the
   // top of the cycle, so the array read gets the WHOLE cycle -- the frontend stops caring
   // how late the fetch cloud is. Every arm below is a flop output:
   //   redirect_pc  M drives it through a register (ooo2_core's redirect_target_q)
   //   pc_q, strad  this module's own state
   //   apred_v      the predictor's steer with its `cti_ok` term removed
   //   pred_tgt     the predictor's registered BTB entry / RAS
   // and the one term that is NOT available -- the fall-through -- is predicted here.
   //
   // THE SELECT COUNTS AS MUCH AS THE ARM. `pred_v` and `pred_tgt` both used to carry
   // the aligner's `br_term`, so this mux -- and with it a block-RAM address pin -- sat
   // at the end of iMMU -> I$ -> aligner after all: 22 levels, 5.521 ns of 6.000, 70%
   // route. ooo2_predictor now splits its tag cone from its CTI cone and exports the
   // register-only half as `apred_v`. Using it here is safe by the very argument below:
   // `apred_v` differs from `pred_v` only on a bundle whose entry says taken while the
   // aligner says the bundle is not CTI-terminated, and the stamp-and-compare turns that
   // into one LOST prediction, never a wrong one.
   //
   // WHY THIS IS SAFE, and why the 2026-08-23 attempt was not: the predictor stamps its
   // registered entry with the address it ACTUALLY read (btb_qpc <= apc) and will not use
   // it unless btb_qpc == base_pc. So a wrong `apc` costs one LOST prediction and can never
   // produce a wrong one. The earlier attempt indexed from norm_npc but kept stamping with
   // npc; entry and label then disagreed, a redirect could read a stale entry stamped with
   // the redirect target, and the mispredict it caused re-read the same stale entry -- a
   // loop that never resynchronised (it hung rv64mi-p-illegal). Label with what you read.
   //
   // `straddle_det` is deliberately absent: it implies the aligner produced nothing, so
   // ~fire, so the consumer holds its entry -- which is exactly the right answer, because
   // pc_q holds too.
   //
   // The bundle's fall-through is pc_q + 2*consumed, consumed being 1..2*IW halfwords, so
   // `lenp` is an untagged direct-mapped table of (consumed - 1), trained on every fire from
   // the aligner's own count. Untagged is fine for the same reason as above -- an alias
   // costs a lost prediction, never a wrong one. At IW=1 this was one bit (2 or 4 bytes);
   // at IW=2 (2026-09-05, two-wide fetch) it is two.
   localparam LENB = 12, NLEN = 1 << LENB;   // 4096 (was 1024): aliasing test 2026-09-05
   localparam CW = (IW > 1) ? $clog2(2*IW) : 1;      // holds consumed-1 for consumed in 1..2*IW
   (* ram_style = "distributed" *)
   reg  [CW-1:0] lenp [0:NLEN-1];
   integer li;
   initial for (li = 0; li < NLEN; li = li + 1) lenp[li] = {{(CW-1){1'b0}}, 1'b1};   // cold: 4 bytes
   wire [LENB-1:0] lidx    = pc_q[LENB:1];
   wire [CW-1:0]   len_g   = lenp[lidx];                              // consumed - 1
   wire [63:0]     len_adv = {{(63-CW){1'b0}}, len_g, 1'b0} + 64'd2;  // 2*consumed bytes
   assign apc = reset        ? RESET_PC
              : redirect     ? redirect_pc
              : irq_inject   ? pc_q
              : strad        ? (pc_q + 64'd4)
              : apred_v      ? pred_tgt
              :                (pc_q + len_adv);

   wire [PBW-1:0] al_cons_m1 = al_consumed - 1'b1;
   always @(posedge clk) begin
      // Train from the truth: a straddler is a 32-bit op by construction; the interrupt
      // pseudo-op is not the instruction at pc_q at all, so it must not train.
      if (fire & ~irq_inject) lenp[lidx] <= strad ? {{(CW-1){1'b0}}, 1'b1} : al_cons_m1[CW-1:0];
   end

   always @(posedge clk) begin
      if (reset) begin
         pc_q  <= RESET_PC; seq_q <= 0; strad <= 1'b0;
      end else if (redirect) begin
         pc_q  <= redirect_pc; seq_q <= redirect_seq; strad <= 1'b0;
      end else if (irq_inject) begin
         // inject the pseudo-op at pc_q: hold PC, take one seqno, abandon any straddle
         // (the straddler re-fetches when the handler returns to mepc = pc_q).
         if (fire) seq_q <= seq_q + 1'b1;
         strad <= 1'b0;
      end else if (strad) begin
         // high halfword fetched: emit the combined 32-bit op and advance past it (4 bytes).
         if (fire) begin pc_q <= pc_q + 64'd4; seq_q <= seq_q + 1'b1; strad <= 1'b0; end
      end else if (straddle_det) begin
         // enter straddle: latch the low halfword, hold PC (the op is not consumed yet).
         // pc_q is held from here until the straddle completes, so its +2 is captured
         // once, right here, and read back as a flop for as long as `strad` is high.
         strad <= 1'b1; strad_lo <= imem_data[15:0]; pc2_q <= pc_q + 64'd2;
      end else if (fire) begin
         pc_q  <= norm_npc;
         seq_q <= seq_q + nvalid;
      end
   end

   // The precompute is sound only while `strad` implies a frozen pc_q. If any future
   // path advances PC during a straddle, imem_addr would translate the wrong page and
   // the straddler would splice a halfword from somewhere else entirely.
   always @(posedge clk)
     if (!reset && strad && (pc2_q !== pc_q + 64'd2))
       $fatal(1, "fetch: pc2_q stale during straddle (pc_q=%h pc2_q=%h)", pc_q, pc2_q);
endmodule

`default_nettype wire
