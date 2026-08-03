`default_nettype none

// Iterative RV64 divide/remainder unit (restoring division, one quotient bit per
// cycle). A combinational 64-bit divide is a timing bomb; this is a ~64-cycle FSM
// with a start/busy/done handshake so it can be integrated as a deferred-completion
// functional unit (like a load).
//
// Op = {is_w, f3}, same encoding the M datapath uses:
//   f3: 100 DIV  101 DIVU  110 REM  111 REMU   (f3[0]=unsigned, f3[1]=remainder)
//   is_w: operate on the low 32 bits, sign-extend the 32-bit result to 64.
// Spec special cases resolved in one cycle: divide-by-zero (quo=all-ones,
// rem=dividend) and signed overflow MIN/-1 (quo=MIN, rem=0).
//
// Core: operands are reduced to unsigned magnitudes; the restoring loop keeps the
// remainder < divisor (so 64 bits), using a 65-bit candidate {rem,next-bit} for the
// compare/subtract. Result sign/W-truncation applied at the end.
module divider
  (input  wire        clk,
   input  wire        reset,
   input  wire        start,        // pulse: capture operands (ignored unless idle)
   input  wire        abort,        // squash: drop the in-flight divide, no done
   input  wire [63:0] rs1,
   input  wire [63:0] rs2,
   input  wire [2:0]  f3,
   input  wire        is_w,
   output wire        busy,         // st != IDLE (held through the done cycle)
   output wire        done,         // 1 in the result cycle: `result` valid
   output wire [63:0] result);

   localparam S_IDLE=2'd0, S_RUN=2'd1, S_FIN=2'd2;
   reg [1:0]  st;
   reg [6:0]  cnt;
   reg [63:0] divd;        // dividend -> quotient accumulator (unsigned magnitude)
   reg [63:0] rem;         // running remainder (unsigned, kept < divisor)
   reg [63:0] dvsr;        // divisor (unsigned magnitude)
   reg        q_neg, r_neg, want_rem, w_r, spec;
   reg [63:0] spec_res;

   // ---------------- operand prep (combinational; sampled at start) ----------------
   wire        sgn  = ~f3[0];                          // DIV/REM signed, DIVU/REMU not
   wire [31:0] a32  = rs1[31:0], b32 = rs2[31:0];
   wire        an   = sgn & (is_w ? a32[31] : rs1[63]);
   wire        bn   = sgn & (is_w ? b32[31] : rs2[63]);
   wire [31:0] ua_w = an ? -a32 : a32;
   wire [31:0] ub_w = bn ? -b32 : b32;
   wire [63:0] ua   = is_w ? {32'd0, ua_w} : (an ? -rs1 : rs1);
   wire [63:0] ub   = is_w ? {32'd0, ub_w} : (bn ? -rs2 : rs2);
   wire        div0 = is_w ? (b32 == 32'd0) : (rs2 == 64'd0);
   wire        ovf  = sgn & (is_w ? (a32 == 32'h80000000 && b32 == 32'hffffffff)
                                  : (rs1 == 64'h8000000000000000 && rs2 == ~64'd0));
   // special result, already in final (sign-extended for W) form
   wire [63:0] minv = is_w ? 64'hFFFFFFFF80000000 : 64'h8000000000000000;
   wire [63:0] rem0 = is_w ? {{32{a32[31]}}, a32} : rs1;     // dividend, W-sext
   wire [63:0] spec_val = ovf ? (f3[1] ? 64'd0 : minv)       // overflow: rem=0 / quo=MIN
                              : (f3[1] ? rem0  : ~64'd0);     // div0:    rem=dividend / quo=-1

   // ---------------- restoring step (combinational over the regs) -----------------
   wire [64:0] cand = {rem, divd[63]};                 // 65-bit: shift in next dividend bit
   wire        ge   = (cand >= {1'b0, dvsr});
   wire [64:0] dif  = cand - {1'b0, dvsr};

   // ---------------- final result (combinational over the regs) -------------------
   wire [63:0] q_s  = q_neg ? -divd : divd;
   wire [63:0] r_s  = r_neg ? -rem  : rem;
   wire [63:0] pick = want_rem ? r_s : q_s;
   wire [63:0] fin  = w_r ? {{32{pick[31]}}, pick[31:0]} : pick;

   // Outputs combinational off the FSM so busy stays high through the result cycle
   // (the owning shard must not issue then, leaving its WB lane free) and done/result
   // are valid in S_FIN. abort wins -> no done.
   assign busy   = (st != S_IDLE);
   assign done   = (st == S_FIN);
   assign result = spec ? spec_res : fin;

   always @(posedge clk) begin
      if (reset || abort) begin
         st <= S_IDLE;
      end else case (st)
         S_IDLE: if (start) begin
            want_rem <= f3[1]; w_r <= is_w;
            q_neg <= sgn & (an ^ bn); r_neg <= sgn & an;
            if (div0 || ovf) begin
               spec <= 1'b1; spec_res <= spec_val; st <= S_FIN;
            end else begin
               spec <= 1'b0; rem <= 64'd0; divd <= ua; dvsr <= ub;
               cnt <= 7'd64; st <= S_RUN;
            end
         end
         S_RUN: begin
            rem  <= ge ? dif[63:0] : cand[63:0];
            divd <= {divd[62:0], ge};               // shift dividend up, quotient bit in
            cnt  <= cnt - 7'd1;
            if (cnt == 7'd1) st <= S_FIN;            // 64th step done
         end
         S_FIN: st <= S_IDLE;                        // result presented this cycle
         default: st <= S_IDLE;
      endcase
   end
endmodule

`default_nettype wire
