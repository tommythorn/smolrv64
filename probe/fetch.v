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
    output wire [IW*SEQW-1:0]      seq);

   localparam PBW = $clog2(HW+2);

   reg [PCW-1:0]  pc_q;
   reg [SEQW-1:0] seq_q;
   initial begin pc_q = RESET_PC; seq_q = 0; end

   assign imem_addr = pc_q;

   wire [PBW-1:0] consumed;
   aligner #(.IW(IW), .HW(HW), .PCW(PCW), .SEQW(SEQW)) u_al
     (.hwin(imem_data), .avail(imem_avail), .base_pc(pc_q), .base_seq(seq_q),
      .valid(slot_valid), .inst(inst), .pc(pc), .seq(seq), .consumed(consumed));

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
         pc_q  <= pc_q + {{(PCW-PBW-1){1'b0}}, consumed, 1'b0};  // += 2*consumed
         seq_q <= seq_q + nvalid;
      end
   end
endmodule

`default_nettype wire
