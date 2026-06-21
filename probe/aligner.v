`default_nettype none

// Fetch-window aligner: carves up to IW variable-length instructions (RVC 16b /
// base 32b) out of a window of HW contiguous halfwords whose first halfword is
// at base_pc. Produces exactly the decode_stage input contract
// ({inst[31:0], in_valid, seq} per slot) plus per-slot PC and a `consumed`
// halfword count so fetch can advance.
//
// Length scan: instruction k begins at halfword position p[k]; it is 32-bit iff
// hw[p[k]][1:0]==11 (consumes 2 halfwords) else 16-bit (1). p[0]=0,
// p[k+1]=p[k]+len[k]. This is the classic O(W) dependent prefix scan -- short
// (4 deep) and made of +1/+2 adders on a <=4-bit position.
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
    parameter SEQW = 8)
   (input  wire [HW*16-1:0]        hwin,
    input  wire [$clog2(HW+2)-1:0] avail,    // halfwords present (0..HW)
    input  wire [PCW-1:0]          base_pc,  // PC of hwin[0]
    input  wire [SEQW-1:0]         base_seq, // seq of slot 0
    output wire [IW-1:0]           valid,
    output wire [IW*32-1:0]        inst,
    output wire [IW*PCW-1:0]       pc,
    output wire [IW*SEQW-1:0]      seq,
    output wire [$clog2(HW+2)-1:0] consumed);

   localparam PBW = $clog2(HW+2);        // holds a position 0..HW (+1 for lookahead)

   // halfword read with out-of-window guard (returns 0 past the window)
   function [15:0] hwr(input [PBW-1:0] idx);
      hwr = (idx < HW) ? hwin[idx*16 +: 16] : 16'b0;
   endfunction

   reg  [IW-1:0]    v;
   reg  [31:0]      ir   [0:IW-1];
   reg  [PCW-1:0]   pcv  [0:IW-1];
   reg  [SEQW-1:0]  sqv  [0:IW-1];
   reg  [PBW-1:0]   cons;

   integer k;
   reg [PBW-1:0] pos;
   reg           run;
   reg [15:0]    h0;
   reg           is32, have;
   // Explicit sensitivity: hwin is read via the hwr() function, which iverilog's
   // @* does not pull into the list -- name it so the block re-evaluates on it.
   always @(hwin or avail or base_pc or base_seq) begin
      pos = 0;
      run = 1'b1;
      for (k = 0; k < IW; k = k + 1) begin
         h0   = hwr(pos);
         is32 = (h0[1:0] == 2'b11);
         // all needed halfwords present?  first always, second only if 32-bit
         have = (pos < avail) && (!is32 || ((pos + 1'b1) < avail));
         v[k]   = run & have;
         ir[k]  = {hwr(pos + 1'b1), h0};      // uniform 32-bit window
         pcv[k] = base_pc + (pos << 1);
         sqv[k] = base_seq + k[SEQW-1:0];
         if (v[k]) pos = pos + (is32 ? 2'd2 : 2'd1);
         else      run = 1'b0;                // prefix: stop at first that doesn't fit
      end
      cons = pos;                             // halfwords consumed (straddler excluded)
   end

   genvar g;
   generate
      for (g = 0; g < IW; g = g + 1) begin : pk
         assign inst[g*32  +: 32]   = ir[g];
         assign pc  [g*PCW +: PCW]  = pcv[g];
         assign seq [g*SEQW +: SEQW] = sqv[g];
      end
   endgenerate
   assign valid    = v;
   assign consumed = cons;
endmodule

`default_nettype wire
