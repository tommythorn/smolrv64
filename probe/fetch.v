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
    input  wire [PCW-1:0]          pred_tgt,
    output wire [PCW-1:0]          npc,
    output wire [PCW-1:0]          pred_npc,
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
   initial begin pc_q = RESET_PC; seq_q = 0; strad = 1'b0; end

   // halfwords from pc_q to the 4 KiB page boundary; cap the aligner's view there.
   wire [11:0]    off      = pc_q[11:0];
   wire           at_bound = (off == 12'hFFE);                  // pc_q is the page's last halfword
   wire [12:0]    hw_bound = (13'd4096 - {1'b0, off}) >> 1;     // 1..2048 halfwords to the boundary
   wire [PBW-1:0] hw_cap   = (hw_bound > HW) ? HW[PBW-1:0] : hw_bound[PBW-1:0];
   wire [PBW-1:0] eff_avail = (imem_avail < hw_cap) ? imem_avail : hw_cap;

   assign cur_seq   = seq_q;
   // While straddling, present PC+2 so the iMMU translates the high halfword's (next) page.
   assign imem_addr = (strad & ~irq_inject) ? (pc_q + 64'd2) : pc_q;
   assign imem_ipc  = pc_q;     // the instruction's PC in both states (trap EPC)

   // ---- aligner over the page-capped window (used in the NORMAL state) ----
   wire [PBW-1:0]    al_consumed;
   wire [IW-1:0]     al_valid;
   wire [IW*32-1:0]  al_inst;
   wire [IW*PCW-1:0] al_pc;
   wire [IW*SEQW-1:0] al_seq;
   aligner #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW)) u_al
     (.hwin(imem_data), .avail(eff_avail), .base_pc(pc_q), .base_seq(seq_q),
      .solo_all(solo_all),
      .valid(al_valid), .inst(al_inst), .pc(al_pc), .seq(al_seq), .consumed(al_consumed));

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
   assign pc         = (irq_inject | strad) ? {{((IW-1)*PCW){1'b0}}, pc_q} : al_pc;
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
   wire [PCW-1:0] ft_npc   = pc_q + {{(PCW-PBW-1){1'b0}}, al_consumed, 1'b0};  // += 2*consumed
   wire [PCW-1:0] norm_npc = pred_v ? pred_tgt : ft_npc;
   // the presented bundle's chosen next PC (mispredict reference at execute)
   assign pred_npc = irq_inject ? pc_q
                   : strad      ? (pc_q + 64'd4)
                   :              norm_npc;
   // computed next PC, mirroring the advance chain's priorities exactly -- this
   // is the predictor's BTB read address (registered there, rule A1).
   assign npc = reset        ? RESET_PC
              : redirect     ? redirect_pc
              : irq_inject   ? pc_q
              : strad        ? (fire ? (pc_q + 64'd4) : pc_q)
              : straddle_det ? pc_q
              : fire         ? norm_npc : pc_q;

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
         strad <= 1'b1; strad_lo <= imem_data[15:0];
      end else if (fire) begin
         pc_q  <= norm_npc;
         seq_q <= seq_q + nvalid;
      end
   end
endmodule

`default_nettype wire
