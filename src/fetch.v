`default_nettype none

// Minimal fetch unit: sequences the PC, reads an HW-halfword window starting at
// PC, and aligns it into a bundle for decode. Runs fall-through unless the
// predictor steered the stream (a marked halfword whose prediction is taken);
// corrected by `redirect` (branch mispredict / exception / CPR rollback). PC is
// the only architectural state here.
//
// PREDICTION. The fetch ring (ooo2_fring) marks each halfword that ends a pair at a CTI the
// predictor knows, and the head of the predictor's queue is the first mark's prediction. A
// bundle ends at the first mark: the aligner ends it at an instruction ending on one. When the bundle's
// last instruction ends exactly there, fetch consumes the mark (pop) and, if it is a real branch
// or jump predicted taken, takes the prediction: the ring already holds the target's halfwords
// after the mark. A mark that does not fit the code is rejected (`reject`), and the frontend
// restarts the stream a cycle later, withholding the window meanwhile:
//   - taken, on an instruction that is not a branch or jump (a straddler included): the bundle
//     goes on falling through, and the ring, which holds the target's halfwords after it, restarts
//     at its fall-through;
//   - taken, inside a 32-bit instruction (slot 0 marked, and 32 bits long): nothing is consumed,
//     and the stream restarts at the PC predicting nothing in its first pair, whose lookup
//     produced the mark.
// A not-taken mark on anything else is consumed and ignored, and one inside a 32-bit instruction
// is dropped (the ring clears it) without consuming anything: the ring's halfwords after it are
// the fall-through's either way.
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
//      step pc_q -- the fetch address, always this bare register -- to PC+2 to fetch
//      (and translate) the high halfword from the next page while ipc_q keeps the
//      instruction's PC, then emit the combined 32-bit op as a one-instruction bundle
//      and advance past it.
// A compressed op at 0xFFE (low 2 bits != 11) is complete in this page: the aligner
// emits it from the single available halfword and we NEVER touch the next page, so
// an unmapped next page raises no fault. If the next page is unmapped for a real
// 32-bit straddler, the fetch stalls and the iMMU's fault is delivered precisely:
// epc = the instruction PC (imem_ipc = ipc_q), tval = the faulting VA (imem_addr =
// pc_q, the high halfword's) -- exactly simmerv's mepc/mtval for a page-crossing
// instruction fault.
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
    output wire                    irq_pres,    // ...and it is the presented bundle this cycle: a
                                                // pending injection waits out a straddle (see irq_go)
    // branch prediction (see PREDICTION above). `pred_npc` is the next PC actually chosen for
    // the PRESENTED bundle; it rides to dispatch and is the exec-side mispredict reference
    // (branch_unit redirects iff actual_npc != pred_npc).
    input  wire [HW-1:0]           imem_mk,     // the window's marks
    input  wire                    pq_tk,       // the first mark's prediction: taken...
    input  wire [PCW-1:0]          pq_tgt,      // ...to here
    output wire                    pop,         // fetch took the first mark's prediction, or dropped it
    output wire                    drop,        // ...dropped it: the ring clears the mark
    output wire                    at_mark,     // the presented bundle's last instruction ends at the first mark
    output wire                    reject,      // the first mark does not fit the code: restart the stream...
    output wire                    reject_np,   // ...predicting nothing in its first pair
    output wire [PCW-1:0]          pred_npc,
    // pred_npc WITHOUT THE SUM: which of {pc + length, pq_tgt, pc} pred_npc is. A consumer
    // that already knows the instruction's length (decode does) rebuilds pred_npc from this
    // and pq_tgt and leaves the +2*consumed adder -- eight CARRY8 at the END of the fetch
    // cloud -- out of whatever it stores. ooo2_frontend's decoupling queue is that consumer: its
    // write data was the design's second-worst family on 2026-09-03 (342 endpoints, 25
    // levels, 13 CARRY8, iMMU -> fetch buffer -> aligner -> this adder -> LUTRAM data pin).
    //   0 = fall-through: pc + the presented instruction's length (also the straddle's +4)
    //   1 = the predicted target (pq_tgt)
    //   2 = this PC (the interrupt pseudo-op holds it)
    output wire [1:0]              pnpc_kind,
    // instruction memory (combinational read of HW halfwords at imem_addr)
    output wire [PCW-1:0]          imem_addr,
    output wire [PCW-1:0]          imem_ipc,    // PC of the instruction being fetched (fault EPC)
    input  wire [HW*16-1:0]        imem_data,
    input  wire [$clog2(HW+2)-1:0] imem_avail,
    input  wire [1:0]              imem_lvl,    // served chunk's page size (0=4K, else >=2M): the enclosing-page cap
    input  wire                    imem_ok,     // late: imem_data/imem_avail are the PC's bytes. The
                                                // aligner runs on the (register-derived) window and
                                                // count regardless; this bit gates the bundle's valid,
                                                // the straddle entry and so every state update (T1 (F)).
    // downstream handshake + aligned bundle
    input  wire                    ready,
    output wire                    valid,
    output wire [IW-1:0]           slot_valid,
    output wire [IW*32-1:0]        inst,
    output wire [IW*PCW-1:0]       pc,
    output wire [IW*SEQW-1:0]      seq,
    output wire [$clog2(HW+2)-1:0] adv_hw,      // halfwords fetch consumed from the window this cycle
    output wire [SEQW-1:0]         cur_seq,     // PC register's seqno (for trap resume)
    output wire [1:0]              err);        // its invariants, registered (the integrity log)

   localparam PBW = $clog2(HW+2);
   // SYSTEM, funct3=000, imm[11:0]=OP_IRQ(0x7F0), rs1=rd=0 -> the interrupt pseudo-op.
   localparam [31:0] IRQ_INSN = 32'h7F00_0073;

   reg [PCW-1:0]  pc_q;
   reg [SEQW-1:0] seq_q;
   // page-cross straddle state: `strad`=1 means we are fetching the high halfword of
   // a 32-bit op from PC+2 (the next page); `strad_lo` holds its already-read low half.
   reg            strad;
   reg [15:0]     strad_lo;
   reg [PCW-1:0]  ipc_q;              // the instruction's PC: pc_q, or pc_q - 2 while straddling
   initial begin pc_q = RESET_PC; ipc_q = RESET_PC; seq_q = 0; strad = 1'b0; end

   // Cap the aligner's view at the ENCLOSING page boundary: 4 KiB (imem_lvl==0) or 2 MiB
   // (imem_lvl!=0; a 1 GiB leaf caps conservatively as 2 MiB, Stage 2 inc 1/3). The page size
   // is the SERVED chunk's, carried by the VHPR I$ off the hit path -- the iMMU is never on the
   // hit cone. Inside a 2 MiB page the window is no longer chopped every 4 KiB, and the straddle
   // FSM below fires only at the true 2 MiB boundary, not at every 4 KiB sub-boundary.
   localparam integer HB = $clog2(HW);
   wire           big      = (imem_lvl != 2'd0);                // >= 2 MiB page
   wire [20:0]    off      = pc_q[20:0];                        // enclosing-page offset (up to 2 MiB)
   wire           at_bound = big ? (&off[20:1]) : (&off[11:1]); // pc_q is the enclosing page's last halfword
   // The cap is HW unless pc_q is inside the enclosing page's last HW halfwords, and then it is
   // what is left of them: a reduction-AND of register bits and a (HB+1)-bit difference. The sum
   // form `(pgsz - off) >> 1` is kept as the oracle below (gate W5fix, 2026-09-07).
   wire           in_last  = big ? (&off[20:HB+1]) : (&off[11:HB+1]);  // last HW halfwords of the page
   wire [HB:0]    hw_left  = HW[HB:0] - {1'b0, off[HB:1]};       // 1..HW of them from pc_q (page-size independent)
   wire [PBW-1:0] hw_cap   = in_last ? {{(PBW-HB-1){1'b0}}, hw_left} : HW[PBW-1:0];
   wire           bytes_late = (imem_avail < hw_cap);          // short of the page end: more is coming
   wire [PBW-1:0] eff_avail  = bytes_late ? imem_avail : hw_cap;
   wire [21:0]    pgsz     = big ? 22'h200000 : 22'h001000;     // enclosing page size (2 MiB or 4 KiB)
   wire [20:0]    off_pg   = big ? off : {9'b0, off[11:0]};      // offset within the ENCLOSING page (mask to 4K)
   wire [21:0]    hw_bound = (pgsz - {1'b0, off_pg}) >> 1;       // the oracle: halfwords to the boundary
   wire [PBW-1:0] hw_cap_ref = (hw_bound > HW) ? HW[PBW-1:0] : hw_bound[PBW-1:0];
   wire e_cap = ~reset & (hw_cap != hw_cap_ref);
   always @(posedge clk)
     if (e_cap) $fatal(1, "fetch: hw_cap %0d != %0d at off=%h", hw_cap, hw_cap_ref, off);

   assign cur_seq   = seq_q;
   // THE FETCH ADDRESS IS THE PC REGISTER, BARE (2026-09-07). It was `(strad & ~irq_inject)
   // ? pc2_q : pc_q` -- a mux at the head of the frontend's PC loop, and irq_inject_q was
   // the source of that loop's worst path (gate W5fix: 20 levels through the iMMU's match,
   // the served window, the straddle detect and the bundle count into pc_q). Now pc_q IS the
   // fetch address in every state: a straddle steps it to the high halfword's address and
   // ipc_q keeps the instruction's PC (the bundle's PC, the fault's EPC, the pseudo-op's).
   // The pending interrupt waits out a straddle instead of abandoning it (irq_go): the
   // straddle's own arm finishes first, so no state here reads irq_inject before the
   // bundle muxes. Asserted below rather than argued.
   assign imem_addr = pc_q;
   assign imem_ipc  = ipc_q;
   wire   irq_go    = irq_inject & ~strad;
   assign irq_pres  = irq_go;

   // ---- aligner over the page-capped window (used in the NORMAL state) ----
   wire [PBW-1:0]    al_consumed;
   wire [IW-1:0]     al_valid;
   wire [IW*32-1:0]  al_inst;
   wire [IW*PCW-1:0] al_pc;
   wire [IW*PBW-1:0] al_offs;
   wire [IW*SEQW-1:0] al_seq;
   wire al_br_term, al_mk_term;
   aligner #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW)) u_al
     (.hwin(imem_data), .avail(eff_avail), .base_pc(pc_q), .base_seq(seq_q),
      .solo_all(solo_all), .bytes_late(bytes_late), .mk(imem_mk),
      .valid(al_valid), .inst(al_inst), .pc(al_pc), .offs(al_offs), .seq(al_seq), .consumed(al_consumed),
      .br_term(al_br_term), .mk_term(al_mk_term));
   // THE FALL-THROUGH AND THE SLOT PCs ARE MUXES, NOT ADDERS (2026-09-05). Gate U, the first
   // build of the two-wide fetch, failed at -0.620 ns on pc_q -> iMMU -> fetch buffer ->
   // aligner -> +2*consumed -> the RAS write and the decoupling queue's slot-1 PC: 23 levels with
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
   wire   br_term = al_br_term & ~strad & ~irq_go;

   // slot-0 page-boundary straddler: pc_q at the last halfword, a 32-bit op (low2==11),
   // and that low halfword actually present (an I$ hit). The aligner excludes it (the
   // capped window has only one halfword), so we take over with the two-step fetch.
   wire lo_avail     = imem_ok & (imem_avail >= 1'b1);
   wire straddle_det = ~strad & at_bound & (imem_data[1:0] == 2'b11) & lo_avail & ~irq_go & ~imem_mk[0];

   // straddle output: the high halfword has arrived (PC+2's page resolved) in imem_data[15:0].
   wire              strad_ready = strad & lo_avail;
   wire [IW*32-1:0]  strad_inst  = {{((IW-1)*32){1'b0}}, imem_data[15:0], strad_lo};

   // bundle mux: irq inject (solo pseudo-op) > straddle (combined op) > aligner.
   assign slot_valid = irq_go     ? {{(IW-1){1'b0}}, 1'b1}
                     : strad      ? (strad_ready ? {{(IW-1){1'b0}}, 1'b1} : {IW{1'b0}})
                     :              (al_valid & {IW{imem_ok}});
   assign inst       = irq_go     ? {{((IW-1)*32){1'b0}}, IRQ_INSN}
                     : strad      ? strad_inst
                     :              al_inst;
   assign pc         = (irq_go | strad) ? {{((IW-1)*PCW){1'b0}}, ipc_q} : al_pcm;
   assign seq        = (irq_go | strad) ? {{((IW-1)*SEQW){1'b0}}, seq_q} : al_seq;

   assign valid = |slot_valid;          // a bundle is present iff >=1 instr aligned
   wire   fire  = ready && valid;        // advance only on a downstream handshake

   // the first mark (see PREDICTION): where the bundle ends, what it takes, what it rejects
   assign at_mark   = (strad & imem_mk[0]) | (~strad & ~irq_go & al_mk_term);
   wire   pred_v    = at_mark & pq_tk & br_term;
   wire   mid       = imem_ok & ~strad & ~irq_go & imem_mk[0] & (imem_data[1:0] == 2'b11);
   assign drop      = mid & ~pq_tk;
   assign reject    = ~reset & ~redirect & ((fire & at_mark & pq_tk & ~pred_v) | (mid & pq_tk));
   assign reject_np = mid & pq_tk;
   assign pop       = (fire & at_mark & ~(pq_tk & ~pred_v)) | drop;

   integer c;
   reg [SEQW-1:0] nvalid;
   always @* begin
      nvalid = 0;
      for (c = 0; c < IW; c = c + 1) nvalid = nvalid + slot_valid[c];
   end

   // normal-path advance: predicted-taken CTI -> target, else fall-through. The
   // straddle/irq arms of the advance chain come first, so pred_v is naturally
   // ignored there (the straddle FSM owns its +4; the pseudo-op holds PC).
   wire [PCW-1:0] ft_npc   = ft_sel;                                            // += 2*consumed, by selection
   wire [PCW-1:0] norm_npc = pred_v ? pq_tgt : ft_npc;
   // the presented bundle's chosen next PC (mispredict reference at execute)
   assign pred_npc = irq_go     ? ipc_q
                   : strad      ? pc_plus[1]           // ipc_q + 4: the straddler's length
                   :              norm_npc;
   // the same choice, as a selector (the straddle's +4 IS its 32-bit instruction's length)
   assign pnpc_kind = irq_go ? 2'd2 : strad ? 2'd0 : pred_v ? 2'd1 : 2'd0;

   // How far fetch consumed the window this cycle: the fetch ring advances its head by it (a taken
   // prediction included: its target's halfwords follow the mark in the ring).
   assign adv_hw  = (reset | redirect) ? {PBW{1'b0}}
                  : strad         ? {{(PBW-1){1'b0}}, fire}
                  : irq_go        ? {PBW{1'b0}}
                  : straddle_det  ? {{(PBW-1){1'b0}}, 1'b1}
                  : fire          ? al_consumed
                  :                 {PBW{1'b0}};

   always @(posedge clk) begin
      if (reset) begin
         pc_q  <= RESET_PC; ipc_q <= RESET_PC; seq_q <= 0; strad <= 1'b0;
      end else if (redirect) begin
         pc_q  <= redirect_pc; ipc_q <= redirect_pc; seq_q <= redirect_seq; strad <= 1'b0;
      end else if (strad) begin
         // high halfword fetched: emit the combined 32-bit op and advance past it. pc_q is
         // already the high halfword's address, so the next instruction is one halfword on.
         if (fire) begin pc_q <= pc_plus[1]; ipc_q <= pc_plus[1]; seq_q <= seq_q + 1'b1; strad <= 1'b0; end
      end else if (irq_go) begin
         // inject the pseudo-op at pc_q: hold PC, take one seqno (the displaced instruction
         // re-fetches when the handler returns to mepc = pc_q). Never during a straddle.
         if (fire) seq_q <= seq_q + 1'b1;
      end else if (straddle_det) begin
         // enter straddle: latch the low halfword, step the fetch address to the high one;
         // ipc_q stays on the instruction (the op is not consumed yet).
         strad <= 1'b1; strad_lo <= imem_data[15:0]; pc_q <= pc_plus[1];
      end else if (fire) begin
         pc_q  <= norm_npc;
         ipc_q <= norm_npc;
         seq_q <= seq_q + nvalid;
      end
   end

   // ipc_q is pc_q except during a straddle, where the fetch address is one halfword on.
   // A pending interrupt never sees a straddle (irq_go), so no arm above reads irq_inject
   // while strad is high.
   wire e_pc = ~reset & (pc_q != (strad ? ipc_q + 64'd2 : ipc_q));
   reg  [1:0] err_q;
   always @(posedge clk) err_q <= reset ? 2'd0 : {e_pc, e_cap};
   assign err = err_q;
   always @(posedge clk)
     if (e_pc) $fatal(1, "fetch: pc_q/ipc_q disagree (pc_q=%h ipc_q=%h strad=%b)", pc_q, ipc_q, strad);
endmodule

`default_nettype wire
