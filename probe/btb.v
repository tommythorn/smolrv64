// btb.v -- minimal basic-block BTB for the probe frontend (branch-predictor step 1).
//
// Direct-mapped, indexed by the fetch-block start PC. ONE entry per basic block: it predicts,
// at FETCH and before decode, (a) where the block's terminating CTI sits -- `bb_len`, the
// aligner's truncation point, which today the aligner derives by DECODING; (b) the CTI type;
// (c) the target; and (d) for conditionals, the predicted direction with a 4-level confidence.
//
// Direction/confidence -- the trustworthy-`definite` scheme (Michaud-style probabilistic climb,
// asymmetric demote):
//   dir in {NT=0, T=1}  x  conf in {weak=0, strong=1, stronger=2, definite=3}
//   confirm: weak->strong DETERMINISTIC, then strong->stronger and stronger->definite each only
//            when `climb_en` fires (an external LFSR bit ~ prob P) -- so `definite` is hard-earned.
//   miss   : {stronger,definite}->strong, strong->weak, and a miss at weak FLIPS the direction
//            (conf<-weak). Confidence is hard to earn, easy to lose.
//   unseen : a BTB miss reads as (NT, definite) -- predict fall-through, plus the high-confidence
//            marker a future trace cache may trust ONLY on a hit (earned), never on this default.
//   cold   : ALLOCATION, not the FSM, handles cold start -- a taken branch that misses allocates a
//            fresh (T, weak) entry. A not-taken miss allocates nothing (it stays unseen=NT-definite,
//            which is the correct fall-through prediction and keeps not-taken branches out of the BTB).
//
// Predict port is combinational (read in the fetch cycle). Train port is clocked at branch
// resolve and is OFF the fetch critical path, so the FSM (and the external LFSR feeding
// `climb_en`) never gate Fmax. Targets are PC-width = Sv39 VA (40b via va_codec at integration).

`default_nettype none

module btb #(
   parameter PCW    = 40,     // PC / target width (Sv39 canonical VA, 40b)
   parameter NENTRY = 256,    // direct-mapped entries (power of 2)
   parameter IDXW   = 8,      // clog2(NENTRY)
   parameter BBW    = 3,      // basic-block length: halfword offset of the CTI in the fetch block
   parameter TYPEW  = 3,      // CTI type width
   parameter ILO    = 1       // low PC bit to start indexing at (halfword granular; bit0 always 0)
)(
   input  wire              clk,
   input  wire              reset,

   // ---- predict (combinational; fetch cycle) ----
   input  wire [PCW-1:0]    p_pc,        // fetch-block start PC
   output wire              p_hit,       // BTB knows this block
   output wire              p_taken,     // redirect this block? (uncond, or cond with dir=T)
   output wire [PCW-1:0]    p_target,    // predicted target (meaningful when p_taken)
   output wire [BBW-1:0]    p_bb_len,    // CTI offset in block = aligner truncation point (on hit)
   output wire [TYPEW-1:0]  p_type,      // CTI type (meaningful on hit)
   output wire [1:0]        p_conf,      // direction confidence; (NT) definite on a miss

   // ---- train (clocked; at branch resolve, off the fetch path) ----
   input  wire              t_valid,     // a resolved CTI to learn from
   input  wire [PCW-1:0]    t_pc,        // its block-start PC (same basis as p_pc)
   input  wire              t_taken,     // actual outcome (unconditional => 1)
   input  wire [PCW-1:0]    t_target,    // actual target
   input  wire [BBW-1:0]    t_bb_len,    // actual CTI offset in block
   input  wire [TYPEW-1:0]  t_type,      // actual CTI type
   input  wire              climb_en     // external ~prob-P pulse: permit the upper confidence climb
);
   localparam [TYPEW-1:0] TYPE_COND = 0;            // conditional branch (the only direction-predicted type)
   localparam TAGW = PCW - ILO - IDXW;

   // ---- storage (validity + the per-block fields) ----
   reg              v   [0:NENTRY-1];
   reg [TAGW-1:0]   tg  [0:NENTRY-1];
   reg [PCW-1:0]    tgt [0:NENTRY-1];
   reg [BBW-1:0]    bl  [0:NENTRY-1];
   reg [TYPEW-1:0]  ty  [0:NENTRY-1];
   reg              dr  [0:NENTRY-1];   // predicted direction for a conditional (1=taken)
   reg [1:0]        cf  [0:NENTRY-1];   // confidence

   // ---- predict ----
   wire [IDXW-1:0] pidx = p_pc[ILO +: IDXW];
   wire [TAGW-1:0] ptag = p_pc[ILO+IDXW +: TAGW];
   wire            phit = v[pidx] && (tg[pidx] == ptag);
   wire            pcond = (ty[pidx] == TYPE_COND);
   assign p_hit    = phit;
   assign p_taken  = phit && (pcond ? dr[pidx] : 1'b1);   // unconditional types always redirect
   assign p_target = tgt[pidx];
   assign p_bb_len = phit ? bl[pidx] : {BBW{1'b0}};
   assign p_type   = ty[pidx];
   assign p_conf   = phit ? cf[pidx] : 2'd3;              // unseen = definite (the NT default marker)

   // ---- confidence climb on a confirming prediction ----
   function [1:0] climb;
      input [1:0] c;
      input       ce;
      case (c)
         2'd0:    climb = 2'd1;              // weak -> strong (deterministic)
         2'd1:    climb = ce ? 2'd2 : 2'd1;  // strong -> stronger  (prob P)
         2'd2:    climb = ce ? 2'd3 : 2'd2;  // stronger -> definite (prob P)
         default: climb = 2'd3;              // definite (saturate)
      endcase
   endfunction

   // ---- train (clocked) ----
   wire [IDXW-1:0] tidx = t_pc[ILO +: IDXW];
   wire [TAGW-1:0] ttag = t_pc[ILO+IDXW +: TAGW];
   wire            thit = v[tidx] && (tg[tidx] == ttag);
   wire            tcond = (t_type == TYPE_COND);
   wire            pred  = tcond ? dr[tidx] : 1'b1;   // what we'd have predicted for this entry
   wire            confirm = (t_taken == pred);

   integer i;
   always @(posedge clk) begin
      if (reset) begin
         for (i = 0; i < NENTRY; i = i + 1) v[i] <= 1'b0;   // only validity needs clearing
      end else if (t_valid) begin
         if (thit) begin
            // refresh the per-block fields (keeps indirect/ret targets current)
            tgt[tidx] <= t_target;
            bl [tidx] <= t_bb_len;
            ty [tidx] <= t_type;
            if (!tcond) begin
               dr[tidx] <= 1'b1;                       // unconditional: always taken; just climb
               cf[tidx] <= climb(cf[tidx], climb_en);
            end else if (confirm) begin
               cf[tidx] <= climb(cf[tidx], climb_en);  // confirmed direction -> climb confidence
            end else begin                             // mispredicted direction -> demote / flip
               case (cf[tidx])
                  2'd0:    begin dr[tidx] <= t_taken; cf[tidx] <= 2'd0; end  // weak: flip direction
                  2'd1:    cf[tidx] <= 2'd0;                                 // strong -> weak
                  default: cf[tidx] <= 2'd1;                                 // stronger/definite -> strong
               endcase
            end
         end else if (t_taken) begin
            // allocate on a TAKEN miss; a not-taken miss allocates nothing (stays unseen=NT-definite)
            v  [tidx] <= 1'b1;
            tg [tidx] <= ttag;
            tgt[tidx] <= t_target;
            bl [tidx] <= t_bb_len;
            ty [tidx] <= t_type;
            dr [tidx] <= 1'b1;        // it was taken
            cf [tidx] <= 2'd0;        // fresh = weak
         end
      end
   end
endmodule

`default_nettype wire
