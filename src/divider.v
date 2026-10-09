`default_nettype none

// Iterative RV64 divide/remainder unit: radix 4 (two quotient bits per cycle), with the
// iterations limited to the quotient's significant bits. A start/busy/done handshake makes it a
// deferred-completion functional unit (like a load).
//
// Op = {is_w, f3}, same encoding the M datapath uses:
//   f3: 100 DIV  101 DIVU  110 REM  111 REMU   (f3[0]=unsigned, f3[1]=remainder)
//   is_w: operate on the low 32 bits, sign-extend the 32-bit result to 64.
// Spec special cases resolved at the start: divide-by-zero (quo=all-ones, rem=dividend) and
// signed overflow MIN/-1 (quo=MIN, rem=0).
//
// The cycles: S_IDLE takes the operands and reduces them to unsigned magnitudes a and b;
// S_NORM counts their leading zeros; S_RUN does one radix-4 step per cycle; S_FIN presents the
// result. The quotient has at most clz(b) - clz(a) + 1 significant bits, j of them rounded up to
// an even count: S_NORM preloads the remainder with a >> j (fewer bits than b, so smaller than
// it) and puts a's low j bits at the top of the shift register, and S_RUN runs j/2 steps. A
// dividend smaller than the divisor runs none; a 64-bit quotient, 32. Latency 3 + j/2 cycles.
//
// A step: rem < b, so rem4 = {rem, the next two dividend bits} < 4b. The three candidates
// rem4 - b, - 2b, - 3b are computed in parallel (3b once, in S_NORM), and the quotient digit is
// the largest that does not go negative: one subtract and a select per cycle.
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

   localparam S_IDLE=2'd0, S_NORM=2'd1, S_RUN=2'd2, S_FIN=2'd3;
   reg [1:0]  st;
   reg [5:0]  cnt;         // radix-4 steps left
   reg [63:0] divd;        // dividend bits still to shift in, then the quotient
   reg [63:0] rem;         // running remainder (unsigned, kept < divisor)
   reg [63:0] dvsr;        // divisor magnitude
   reg [65:0] dvsr3;       // 3 x divisor
   reg [63:0] ua_q;        // dividend magnitude, S_IDLE -> S_NORM
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

   // ---------------- normalization (combinational over ua_q, dvsr; used in S_NORM) ------
   function [6:0] clz64(input [63:0] x);
      integer i;
      begin
         clz64 = 7'd64;
         for (i = 0; i < 64; i = i + 1) if (x[i]) clz64 = 7'd63 - i[6:0];
      end
   endfunction
   wire [6:0]  za = clz64(ua_q), zb = clz64(dvsr);
   wire        q0 = za > zb;                           // a < 2^(64-za) <= b: quotient 0
   wire [6:0]  qb = q0 ? 7'd0 : zb - za + 7'd1;          // the quotient's significant bits, 1..64
   wire [6:0]  j  = qb + {6'd0, qb[0]};                  // rounded up to even, 0..64
   wire [63:0] rem_n  = j[6] ? 64'd0 : ua_q >> j[5:0];
   wire [63:0] divd_n = j == 7'd0 ? 64'd0 : ua_q << (7'd64 - j);

   // ---------------- radix-4 step (combinational over the regs) -----------------
   wire [65:0] r4 = {rem, divd[63:62]};
   wire [66:0] c1 = {1'b0, r4} - {3'b000, dvsr};
   wire [66:0] c2 = {1'b0, r4} - {2'b00, dvsr, 1'b0};
   wire [66:0] c3 = {1'b0, r4} - {1'b0, dvsr3};
   wire [1:0]  qd = ~c3[66] ? 2'd3 : ~c2[66] ? 2'd2 : ~c1[66] ? 2'd1 : 2'd0;
   wire [63:0] rem_s = ~c3[66] ? c3[63:0] : ~c2[66] ? c2[63:0] : ~c1[66] ? c1[63:0] : r4[63:0];

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
               spec <= 1'b0; ua_q <= ua; dvsr <= ub; st <= S_NORM;
            end
         end
         S_NORM: begin
            rem   <= rem_n;
            divd  <= divd_n;
            dvsr3 <= {2'b00, dvsr} + {1'b0, dvsr, 1'b0};
            cnt   <= j[6:1];
            st    <= j == 7'd0 ? S_FIN : S_RUN;
         end
         S_RUN: begin
            rem  <= rem_s;
            divd <= {divd[61:0], qd};               // shift the dividend up, two quotient bits in
            cnt  <= cnt - 6'd1;
            if (cnt == 6'd1) st <= S_FIN;            // the last step
         end
         S_FIN: st <= S_IDLE;                        // result presented this cycle
         default: begin
            $fatal(1, "divider: unknown state %0d", st);
            st <= S_IDLE;
         end
      endcase
   end
   always @(posedge clk)
      if (!reset && st == S_RUN && rem_s >= dvsr)
         $fatal(1, "divider: the remainder %h is not below the divisor %h", rem_s, dvsr);
endmodule

`default_nettype wire
