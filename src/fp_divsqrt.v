`default_nettype none

// fp_divsqrt -- RISC-V fdiv.{s,d} and fsqrt.{s,d}: one op at a time, a digit recurrence of STEPS
// result bits a cycle, correctly rounded in every rounding mode with all five flags.
//
// ONE RECURRENCE FOR BOTH. With a partial result Q and a scaled remainder r, every step is
//     b = (2r >= T);   r <- 2r - b*T;   Q <- Q + b*2^-(j+1)
// where T is the divisor for a division, and 2Q + 2^-(j+1) for a square root (the remainder
// r_j = (x - Q_j^2) * 2^j; the new bit's weight is the one-hot `pm`, and Q's bits are all above it,
// so T is Q shifted left with pm OR-ed in). One subtractor per step serves both operations and both
// formats; a binary32 op runs the same datapath with its significand at the top and stops early.
//
// THE FORMATS SHARE ONE INTERNAL FORM: a significand of 53 bits with the leading one at bit 52 (a
// binary32 one left-aligned, its low 29 bits zero) and an unbiased exponent. A subnormal operand is
// normalized a bit a cycle (rare; the latency follows its leading zeros), and so is a subnormal
// result denormalized, a bit a cycle with the dropped bits kept as sticky. Tininess is detected
// after rounding (RISC-V): a result whose exponent is one below the normal range is tiny unless
// rounding it at full precision carries it up to the smallest normal.
//
// The ops: a NaN operand (or a binary32 one not NaN-boxed) gives the canonical NaN, NV for a
// signaling one; inf/inf, 0/0 and the root of a negative number are invalid; x/0 is DZ; sqrt(-0)
// is -0. A binary32 result is NaN-boxed.
module fp_divsqrt #(
    parameter TAGW  = 24,
    parameter STEPS = 3                  // result bits a cycle
)(
    input  wire            clk,
    input  wire            reset,
    input  wire            flush,         // drop the op in flight
    input  wire            in_valid,
    output wire            in_ready,
    input  wire            in_sqrt,       // 1: sqrt(a), 0: a / b
    input  wire            in_f64,        // 1: binary64, 0: binary32
    input  wire [2:0]      in_rm,         // RNE RTZ RDN RUP RMM (resolved: never DYN)
    input  wire [63:0]     in_a,
    input  wire [63:0]     in_b,
    input  wire [TAGW-1:0] in_tag,
    output wire            out_valid,
    input  wire            out_ready,
    output wire [63:0]     out_result,    // held while out_valid and not out_ready
    output wire [4:0]      out_flags,     // {NV, DZ, OF, UF, NX}
    output reg  [TAGW-1:0] out_tag,
    output wire            busy
);
   localparam integer F  = 55;           // the recurrence's fraction bits; Q[F] is the integer bit
   localparam integer RW = F + 3;        // the remainder: 3 integer bits (r < 5 for a square root)

   localparam [2:0] S_IDLE = 3'd0, S_NORM = 3'd1, S_ITER = 3'd2, S_POST = 3'd3, S_DEN = 3'd4,
                    S_RND  = 3'd5, S_DONE = 3'd6;
   reg  [2:0]  st;
   initial st = S_IDLE;

   reg            sq, f64, sgn;
   reg  [2:0]     rm;
   reg  [12:0]    ea, eb;                // unbiased exponents (signed); ea becomes the result's
   reg  [52:0]    ma, mb;                // significands, leading one at 52 once normalized
   reg  [RW-1:0]  r;
   reg  [F:0]     q;
   reg  [F:0]     pm;                    // the weight of the next result bit
   reg  [5:0]     cnt;
   reg  [52:0]    m;                     // the result's significand, right-aligned to its format
   reg            g, s_;                 // the guard bit and the sticky bit below it
   reg  [12:0]    be;                    // the result's biased exponent
   reg            tiny;
   reg  [5:0]     dsh;                   // denormalization still to do

   // A computed result is offered as it is rounded (S_RND), a special one from its register (S_DONE).
   reg  [63:0] sp_q;
   reg  [4:0]  spf_q;
   assign in_ready  = st == S_IDLE;
   assign out_valid = (st == S_DONE) | (st == S_RND);
   assign busy      = st != S_IDLE;

   // ---- the operands, as the two formats lay them out ----
   function [10:0] ex_of(input f64_, input [63:0] x);   // the exponent field
      ex_of = f64_ ? x[62:52] : {3'd0, x[30:23]};
   endfunction
   function [51:0] fr_of(input f64_, input [63:0] x);   // the fraction, left-aligned
      fr_of = f64_ ? x[51:0] : {x[22:0], 29'd0};
   endfunction
   wire        a_box = in_f64 | (&in_a[63:32]);         // a binary32 operand is NaN-boxed
   wire        b_box = in_f64 | (&in_b[63:32]);
   wire [10:0] xa = ex_of(in_f64, in_a), xb = ex_of(in_f64, in_b);
   wire [51:0] fa = fr_of(in_f64, in_a), fb = fr_of(in_f64, in_b);
   wire        sa = in_f64 ? in_a[63] : in_a[31];
   wire        sb = in_f64 ? in_b[63] : in_b[31];
   wire        xa1 = in_f64 ? &xa : &xa[7:0];
   wire        xb1 = in_f64 ? &xb : &xb[7:0];
   wire        a_nan = ~a_box | (xa1 & (fa != 52'd0));
   wire        b_nan = ~b_box | (xb1 & (fb != 52'd0));
   wire        a_snan = a_box & xa1 & (fa != 52'd0) & ~fa[51];
   wire        b_snan = b_box & xb1 & (fb != 52'd0) & ~fb[51];
   wire        a_inf = a_box & xa1 & (fa == 52'd0);
   wire        b_inf = b_box & xb1 & (fb == 52'd0);
   wire        a_zero = a_box & (xa == 11'd0) & (fa == 52'd0);
   wire        b_zero = b_box & (xb == 11'd0) & (fb == 52'd0);
   wire [12:0] bias  = in_f64 ? 13'd1023 : 13'd127;
   // unbiased: a subnormal's exponent is emin, its significand without the leading one
   wire [12:0] ua = ((xa == 11'd0) ? 13'd1 : {2'd0, xa}) - bias;
   wire [12:0] ub = ((xb == 11'd0) ? 13'd1 : {2'd0, xb}) - bias;

   // the special results, decided as the op arrives
   wire        dv_nan  = a_nan | b_nan | (a_inf & b_inf) | (a_zero & b_zero);
   wire        sq_nan  = a_nan | (sa & ~a_zero);        // the root of a negative number (or -inf)
   wire        sp      = in_sqrt ? (sq_nan | a_zero | a_inf) : (dv_nan | a_inf | b_inf | b_zero | a_zero);
   wire        sp_nan  = in_sqrt ? sq_nan : dv_nan;
   wire        sp_nv   = in_sqrt ? (a_snan | (sa & ~a_zero & ~a_nan))
                                 : (a_snan | b_snan | (~a_nan & ~b_nan & ((a_inf & b_inf) | (a_zero & b_zero))));
   wire        sp_dz   = ~in_sqrt & ~dv_nan & ~a_inf & b_zero;
   wire        sp_s    = in_sqrt ? sa : sa ^ sb;
   wire        sp_inf  = in_sqrt ? a_inf : (a_inf | b_zero);   // else a zero
   function [63:0] pack(input f64_, input s, input [10:0] x, input [51:0] fr);  // fr right-aligned
      pack = f64_ ? {s, x, fr} : {32'hFFFF_FFFF, s, x[7:0], fr[22:0]};
   endfunction
   wire [10:0] x_all1 = in_f64 ? 11'h7FF : 11'h0FF;
   wire [63:0] sp_res = sp_nan ? pack(in_f64, 1'b0, x_all1, in_f64 ? 52'h8_0000_0000_0000 : 52'h40_0000)
                                : pack(in_f64, sp_s, sp_inf ? x_all1 : 11'd0, 52'd0);

   // ---- the recurrence ----
   reg [RW-1:0] r_n;  reg [F:0] q_n, pm_n;
   reg [RW:0]   r2, t, d;
   integer      k;
   always @* begin
      r_n = r;  q_n = q;  pm_n = pm;
      for (k = 0; k < STEPS; k = k + 1) begin
         r2 = {r_n, 1'b0};
         t  = sq ? {2'd0, q_n, 1'b0} | {3'd0, pm_n} : {3'd0, mb, 3'd0};
         d  = r2 - t;
         if (~d[RW]) begin r_n = d[RW-1:0];  q_n = q_n | pm_n; end
         else        r_n = r2[RW-1:0];
         pm_n = pm_n >> 1;
      end
   end
   // positions to produce: a division's integer bit and 54 below it, a root's 53 below its one
   // (binary32: 25 and 24), so a quotient below 1 still has its guard bit
   wire [5:0]  n_pos   = f64 ? (sq ? 6'd53 : 6'd55) : (sq ? 6'd24 : 6'd26);
   wire [5:0]  n_cyc   = (n_pos + STEPS[5:0] - 6'd1) / STEPS[5:0];

   // ---- normalization of the quotient or root, and rounding ----
   wire        lead  = q[F];                             // a quotient below 1 has it clear
   wire [F:0]  qn    = lead ? q : {q[F-1:0], 1'b0};
   wire        rnz   = r != {RW{1'b0}};
   wire [52:0] m_pst = f64 ? qn[F:F-52] : {29'd0, qn[F:F-23]};
   wire        g_pst = f64 ? qn[F-53] : qn[F-24];
   wire        s_pst = (f64 ? |qn[F-54:0] : |qn[F-25:0]) | rnz;
   wire [12:0] be_pst = ea - {12'd0, ~lead} + (f64 ? 13'd1023 : 13'd127);
   function ru_of(input [2:0] rm_, input s, input lsb, input g_, input st_);
      case (rm_)
         3'd0:    ru_of = g_ & (st_ | lsb);              // RNE
         3'd1:    ru_of = 1'b0;                          // RTZ
         3'd2:    ru_of = s & (g_ | st_);                // RDN
         3'd3:    ru_of = ~s & (g_ | st_);               // RUP
         default: ru_of = g_;                            // RMM
      endcase
   endfunction
   wire        all1_pst = f64 ? &m_pst : &m_pst[23:0];
   wire [12:0] dsh_pst  = 13'd1 - be_pst;               // the denormalizing shift, when be_pst <= 0
   wire        tiny_pst = be_pst[12] | ((be_pst == 13'd0) & ~(all1_pst & ru_of(rm, sgn, 1'b1, g_pst, s_pst)));

   wire        ru    = ru_of(rm, sgn, m[0], g, s_);
   wire [53:0] m1    = {1'b0, m} + {53'd0, ru};
   wire        cy    = f64 ? m1[53] : m1[24];            // a carry into the next binade
   wire        nx    = g | s_;
   wire        den   = be[12] | (be == 13'd0);           // denormalized (be then reads 0)
   wire        hid   = f64 ? m1[52] : m1[23];
   wire [12:0] x_rnd = den ? {12'd0, hid} : be + {12'd0, cy};
   wire [12:0] x_max = f64 ? 13'd2047 : 13'd255;
   wire        ovf   = ~den & (x_rnd >= x_max);
   wire        to_mx = (rm == 3'd1) | ((rm == 3'd2) & ~sgn) | ((rm == 3'd3) & sgn);
   wire [51:0] fr_r  = (~den & cy) ? 52'd0 : (f64 ? m1[51:0] : {29'd0, m1[22:0]});
   wire [10:0] x_mx1 = f64 ? 11'h7FE : 11'h0FE;
   wire [63:0] rnd_res = ovf ? (to_mx ? pack(f64, sgn, x_mx1, 52'hF_FFFF_FFFF_FFFF)
                                      : pack(f64, sgn, f64 ? 11'h7FF : 11'h0FF, 52'd0))
                             : pack(f64, sgn, x_rnd[10:0], fr_r);
   wire [4:0]  rnd_fl  = {2'b00, ovf, tiny & nx & ~ovf, nx | ovf};
   assign out_result = (st == S_RND) ? rnd_res : sp_q;
   assign out_flags  = (st == S_RND) ? rnd_fl  : spf_q;

   always @(posedge clk) begin
      if (reset | flush) st <= S_IDLE;
      else case (st)
         S_IDLE: if (in_valid) begin
            sq <= in_sqrt;  f64 <= in_f64;  rm <= in_rm;  out_tag <= in_tag;
            sgn <= in_sqrt ? 1'b0 : sa ^ sb;
            ea <= ua;  eb <= ub;
            ma <= {xa != 11'd0, fa};  mb <= {xb != 11'd0, fb};
            if (sp) begin
               sp_q <= sp_res;  spf_q <= {sp_nv, sp_dz, 3'b000};  st <= S_DONE;
            end else st <= S_NORM;
         end
         S_NORM:
            // a subnormal operand: a bit a cycle until its leading one is at 52
            if (~ma[52] | (~sq & ~mb[52])) begin
               if (~ma[52])        begin ma <= {ma[51:0], 1'b0};  ea <= ea - 13'd1; end
               if (~sq & ~mb[52])  begin mb <= {mb[51:0], 1'b0};  eb <= eb - 13'd1; end
            end else begin
               cnt <= n_cyc;
               if (sq) begin
                  // x = ma * 2^ea, ea made even: the root's exponent ea/2, its integer bit 1, and
                  // r = x - 1 (x in [1, 4))
                  ea <= {ea[12], ea[12:1]};
                  q  <= {1'b1, {F{1'b0}}};
                  pm <= {2'b01, {(F-1){1'b0}}};
                  r  <= ea[0] ? {1'b0, ma[51] ? 2'b10 : 2'b01, ma[50:0], 4'd0}
                              : {3'b000, ma[51:0], 3'd0};
               end else begin
                  // r = ma / 2: the first step's 2r >= mb is the quotient's integer bit
                  ea <= ea - eb;
                  q  <= {(F+1){1'b0}};
                  pm <= {1'b1, {F{1'b0}}};
                  r  <= {3'b000, ma, 2'd0};
               end
               st <= S_ITER;
            end
         S_ITER: begin
            r <= r_n;  q <= q_n;  pm <= pm_n;  cnt <= cnt - 6'd1;
            if (cnt == 6'd1) st <= S_POST;
         end
         S_POST: begin
            m <= m_pst;  g <= g_pst;  s_ <= s_pst;  be <= be_pst;  tiny <= tiny_pst;
            // a result below the normal range shifts right by 1 - be, at most past the guard bit
            dsh <= (dsh_pst > 13'd55) ? 6'd55 : dsh_pst[5:0];
            st <= (be_pst[12] | (be_pst == 13'd0)) ? S_DEN : S_RND;
         end
         S_DEN: begin
            // a bit a cycle, the dropped ones into the sticky bit
            m <= {1'b0, m[52:1]};  g <= m[0];  s_ <= s_ | g;  dsh <= dsh - 6'd1;
            if (dsh == 6'd1) st <= S_RND;
         end
         S_RND, S_DONE: if (out_ready) st <= S_IDLE;
         default: $fatal(1, "fp_divsqrt: undefined state %0d", st);
      endcase
   end

   always @(posedge clk) if (!reset && !flush) begin
      if (in_valid && in_ready && in_rm > 3'd4)
         $fatal(1, "fp_divsqrt: rounding mode %0d (DYN must be resolved before the unit)", in_rm);
      if (st == S_POST && sq && !lead)
         $fatal(1, "fp_divsqrt: a square root without its integer bit (q %h)", q);
   end
endmodule

`default_nettype wire
