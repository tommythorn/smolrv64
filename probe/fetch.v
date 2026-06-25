`default_nettype none

// Minimal fetch unit: sequences the PC, reads an HW-halfword window starting at
// PC, and aligns it into a bundle for decode. No branch prediction yet -- it
// runs fall-through and is corrected by `redirect` (branch mispredict /
// exception / CPR rollback). PC is the only architectural state here.
//
// Carry-free windowing: the window always *starts at PC*, and PC advances by
// 2*consumed. The aligner excludes a window-straddling 32-bit op from
// `consumed`, so that op's first halfword simply reappears as slot 0 of the next
// window -- no leftover/shift buffer, the straddle special-case folds away.
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
    // instruction memory (combinational read of HW halfwords at imem_addr)
    output wire [PCW-1:0]          imem_addr,
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
   // Decodes (decode_exec) as a serialized solo SYSTEM op; csr_file's OP_IRQ selector
   // turns its (oldest, non-speculative) execution into the interrupt trap.
   localparam [31:0] IRQ_INSN = 32'h7F00_0073;

   reg [PCW-1:0]  pc_q;
   reg [SEQW-1:0] seq_q;
   initial begin pc_q = RESET_PC; seq_q = 0; end

   assign imem_addr = pc_q;
   assign cur_seq   = seq_q;

   wire [PBW-1:0]   al_consumed;
   wire [IW-1:0]    al_valid;
   wire [IW*32-1:0] al_inst;
   wire [IW*PCW-1:0] al_pc;
   wire [IW*SEQW-1:0] al_seq;
   aligner #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW)) u_al
     (.hwin(imem_data), .avail(imem_avail), .base_pc(pc_q), .base_seq(seq_q),
      .solo_all(solo_all),
      .valid(al_valid), .inst(al_inst), .pc(al_pc), .seq(al_seq), .consumed(al_consumed));

   // inject overrides the aligned bundle with a solo synthetic op at {pc_q, seq_q}.
   assign slot_valid = irq_inject ? {{(IW-1){1'b0}}, 1'b1} : al_valid;
   assign inst       = irq_inject ? {{((IW-1)*32){1'b0}}, IRQ_INSN} : al_inst;
   assign pc         = irq_inject ? {{((IW-1)*PCW){1'b0}}, pc_q}     : al_pc;
   assign seq        = irq_inject ? {{((IW-1)*SEQW){1'b0}}, seq_q}   : al_seq;
   // on inject, consume NO halfwords (hold PC) but still take one seqno for the pseudo-op
   wire [PBW-1:0] consumed = irq_inject ? {PBW{1'b0}} : al_consumed;

   assign valid = |slot_valid;          // a bundle is present iff >=1 instr aligned
   wire   fire  = ready && valid;        // advance only on a downstream handshake

   integer c;
   reg [SEQW-1:0] nvalid;
   always @* begin
      nvalid = 0;
      for (c = 0; c < IW; c = c + 1) nvalid = nvalid + slot_valid[c];
   end

   always @(posedge clk) begin
      if (reset) begin
         pc_q  <= RESET_PC;
         seq_q <= 0;
      end else if (redirect) begin
         pc_q  <= redirect_pc;
         seq_q <= redirect_seq;
      end else if (fire) begin
         pc_q  <= pc_q + {{(PCW-PBW-1){1'b0}}, consumed, 1'b0};  // += 2*consumed (0 on inject)
         seq_q <= seq_q + nvalid;
      end
   end
endmodule

`default_nettype wire
