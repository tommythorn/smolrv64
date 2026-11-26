`default_nettype none

// fp_fma -- the FPU's arithmetic: RISC-V's FMA group (f[n]madd, f[n]msub, fadd, fsub, fmul) and the
// conversions (fcvt between the formats, from and to the integers, fcvtmod.w.d), binary32 and binary64
// (binary16 too, as a conversion's source or destination: Zfhmin), one op a
// cycle, every op 5 register stages from the edge that takes it to its result (out_valid for one cycle; the
// consumer takes it, the pipeline never waits). Correctly rounded in every rounding mode, all five
// flags, tininess after rounding.
//
// ONE DATAPATH, BINARY64 WIDE. A binary32 operand is unpacked into the same 53-bit significand (its
// 24 bits at the top) with its own exponent, the product and the sum are exact, and the one rounding
// happens at the destination's precision and exponent range.
//
// THE WINDOW. The 106-bit product sits at bits [107:2] of a 164-bit window (two bits below it for an
// addend's sticky); the addend starts with its LSB at 110 and shifts right by 56 + ea + eb - ec, so
// bit i weighs 2^(ea + eb - 106 + i). An addend more than two bits above the product (a negative
// shift), or any addend beside a zero product, stays at 110 and the product becomes a sticky bit at 0
// instead (the window's base is then the addend's); a zero addend shifts out entirely. Bits an addend
// loses off the bottom are OR-ed into bit 0, below the product: two guard bits are enough for the one
// rounding, as only an addend near the product can cancel. The magnitude of the sum is normalized
// (leading-zero count, left shift), and a result below the normal range is shifted right back into
// it before it rounds.
//
// THE CONVERSIONS RIDE THE SAME PATH. fcvt between the formats is a product with an exact 1, unpacked
// in the source format and rounded in the destination's. An integer source is the product itself:
// its magnitude replaces the multiplier's output with bit 0 at window bit 2 (weight 1), and the sum
// normalizes and rounds it like any other. A conversion to an integer is the addend: its significand
// aligned so window bit 2 weighs 1 (a shift of 160 - e), beside a zero product, and the sum's bits
// [65:2] are the integer, bit 1 its guard and bit 0 its sticky; it rounds, negates and saturates there.
//
// v (in_op): 0 FMADD a*b+c, 1 FMSUB a*b-c, 2 FNMSUB -a*b+c, 3 FNMADD -a*b-c, 4 ADD a+c, 5 SUB a-c,
// 6 MUL a*b (an exact zero product keeps its sign; no addend takes part), 7 F2F a, 8 I2F a (an
// integer: il 64-bit, iu unsigned), 9 F2I a (to a 64- or 32-bit integer, il/iu; a 32-bit result is
// sign-extended), 10 F2I modulo 2^32 (fcvtmod.w.d: F2I to a signed 32-bit integer, its flags, but
// the low 32 bits of the unbounded result instead of a saturated one, and 0 for a NaN or an
// infinity). in_sf / in_df: the source and destination formats, 0 binary32, 1 binary64, 2 binary16.
module fp_fma #(
    parameter TAGW = 24
)(
    input  wire            clk,
    input  wire            reset,
    input  wire            flush,          // drop every op in flight
    input  wire            in_valid,
    input  wire [3:0]      in_op,
    input  wire [1:0]      in_sf,
    input  wire [1:0]      in_df,
    input  wire            in_il,
    input  wire            in_iu,
    input  wire [2:0]      in_rm,           // RNE RTZ RDN RUP RMM (resolved)
    input  wire [63:0]     in_a,
    input  wire [63:0]     in_b,
    input  wire [63:0]     in_c,
    input  wire [TAGW-1:0] in_tag,
    output wire            out_valid,
    output wire [63:0]     out_result,
    output wire [4:0]      out_flags,       // {NV, DZ, OF, UF, NX}
    output wire [TAGW-1:0] out_tag
);
   localparam integer W = 164;              // the window

   // the op as it arrives: C1 works on it in the cycle it is taken
   wire           v0 = in_valid;
   wire [3:0]     op0 = in_op;
   wire [2:0]     rm0 = in_rm;
   wire [1:0]     sf0 = in_sf, df0 = in_df;
   wire           il0 = in_il, iu0 = in_iu;
   wire [63:0]    a0 = in_a, b0 = in_b, c0 = in_c;
   wire [TAGW-1:0] t0 = in_tag;

   // ---- C1: unpack, the special results, the exponents and the alignment ----
   function [70:0] unpack;                  // {nan, snan, inf, zero, s, e[12:0], m[52:0]}
      input [1:0] fm;  input [63:0] x;      // 0 binary32, 1 binary64, 2 binary16
      reg [10:0] ex;  reg [51:0] fr;  reg s, box, all1, d, h;  reg [12:0] eu;
      begin
         d    = fm == 2'd1;  h = fm == 2'd2;
         box  = d | (h ? &x[63:16] : &x[63:32]);
         ex   = d ? x[62:52] : h ? {6'd0, x[14:10]} : {3'd0, x[30:23]};
         fr   = d ? x[51:0]  : h ? {x[9:0], 42'd0} : {x[22:0], 29'd0};
         s    = d ? x[63] : h ? x[15] : x[31];
         all1 = d ? &ex : h ? &ex[4:0] : &ex[7:0];
         eu   = ((ex == 11'd0) ? 13'd1 : {2'd0, ex}) - (d ? 13'd1023 : h ? 13'd15 : 13'd127);
         unpack = {~box | (all1 & (fr != 52'd0)), box & all1 & (fr != 52'd0) & ~fr[51],
                   box & all1 & (fr == 52'd0), box & (ex == 11'd0) & (fr == 52'd0),
                   s, eu, ex != 11'd0, fr};
      end
   endfunction
   localparam [70:0] ONE  = {5'b00000, 13'd0, 1'b1, 52'd0};          // an exact +1
   localparam [70:0] NONE = {3'b000, 1'b1, 1'b0, 13'd0, 53'd0};      // no addend: +0
   wire        is_add = (op0 == 4'd4) | (op0 == 4'd5);
   wire        is_f2f = op0 == 4'd7;
   wire        is_i2f = op0 == 4'd8;
   wire        is_mod = op0 == 4'd10;                                  // fcvtmod.w.d
   wire        is_f2i = (op0 == 4'd9) | is_mod;
   wire        is_mul = (op0 == 4'd6) | is_f2f | is_i2f;              // no addend
   wire        is_cvi = is_i2f | is_f2i;                               // no FP special results
   wire [70:0] ua = unpack(sf0, a0);
   wire [70:0] ub = (is_add | is_f2f) ? ONE : unpack(sf0, b0);
   wire [70:0] uc = is_mul ? NONE : is_f2i ? ua : unpack(sf0, c0);   // F2I: a is the addend
   wire        a_nan = ua[70], a_snan = ua[69], a_inf = ua[68], a_zero = ua[67];
   wire        b_nan = ub[70], b_snan = ub[69], b_inf = ub[68], b_zero = ub[67];
   wire        c_nan = uc[70], c_snan = uc[69], c_inf = uc[68], c_zero = uc[67];
   wire        negp  = (op0 == 4'd2) | (op0 == 4'd3);
   wire        negc  = (op0 == 4'd1) | (op0 == 4'd3) | (op0 == 4'd5);
   // an integer source: its sign and magnitude (w/wu read the low 32 bits)
   wire [63:0] iv    = il0 ? a0 : iu0 ? {32'd0, a0[31:0]} : {{32{a0[31]}}, a0[31:0]};
   wire        ineg  = ~iu0 & iv[63];
   wire [63:0] imag  = ineg ? ~iv + 64'd1 : iv;
   wire        sp1   = is_i2f ? ineg : ua[66] ^ ub[66] ^ negp;
   wire        sc1   = uc[66] ^ negc;
   wire        inf0  = (a_inf & b_zero) | (b_inf & a_zero);
   wire        p_inf = a_inf | b_inf;
   wire        sp_nan = a_nan | b_nan | c_nan | inf0 | (p_inf & c_inf & (sp1 != sc1));
   wire        sp_nv  = a_snan | b_snan | c_snan | inf0 | (~a_nan & ~b_nan & ~c_nan & p_inf & c_inf & (sp1 != sc1));
   wire        sp1_v  = ~is_cvi & (sp_nan | p_inf | c_inf);
   wire        sp_s   = p_inf ? sp1 : sc1;
   wire [13:0] ea = {ua[65], ua[65:53]}, eb = {ub[65], ub[65:53]}, ec = {uc[65], uc[65:53]};
   wire [13:0] ep = is_cvi ? 14'd104 : ea + eb;                      // a conversion: window bit 2 weighs 1
   wire [13:0] sh = 14'd56 + ep - ec;                                 // the addend's right shift
   // the addend lies above the product -- or the product is zero; a zero addend never decides
   wire        c_top = ~is_cvi & (sh[13] | a_zero | b_zero) & ~c_zero;
   reg         v1, sub1, top1, mul1, pz1, cz1, sp_v1, sp_n1, sp_nv1, sp_s1, sp1_q, sc1_q;
   reg         i2f1, f2i1, md1, il1, iu1, xn1, xi1, xb1;
   reg  [1:0]  f1;
   reg  [2:0]  rm1;  reg [52:0] ma1, mb1, mc1;  reg [7:0] sh1;  reg [13:0] wb1;  reg [63:0] im1;
   reg  [TAGW-1:0] t1;
   always @(posedge clk) begin
      v1 <= ~reset & ~flush & v0;
      f1 <= df0;  rm1 <= rm0;  t1 <= t0;  mul1 <= is_mul;
      ma1 <= is_f2i ? 53'd0 : ua[52:0];  mb1 <= ub[52:0];  mc1 <= is_i2f ? 53'd0 : uc[52:0];
      sub1 <= ~is_cvi & (sp1 ^ sc1);  sp1_q <= sp1;  sc1_q <= is_f2i ? ua[66] : sc1;
      top1 <= c_top;
      // from 163 on the whole addend lies below the window: all of it in the sticky bits
      sh1  <= c_top ? 8'd0 : (c_zero | is_i2f | (sh > 14'd163)) ? 8'd163 : sh[7:0];
      wb1  <= c_top ? ec - 14'd162 : ep - 14'd106;                    // bit 0's weight, 2^wb
      pz1  <= a_zero | b_zero;  cz1 <= c_zero;
      sp_v1 <= sp1_v;  sp_n1 <= sp_nan;  sp_nv1 <= sp_nv;  sp_s1 <= sp_s;
      i2f1 <= is_i2f;  f2i1 <= is_f2i;  md1 <= is_mod;  il1 <= il0;  iu1 <= iu0;  im1 <= imag;
      // F2I: a NaN, an infinity, a magnitude of 2^64 or more (never an integer of either width)
      xn1 <= a_nan;  xi1 <= a_inf;  xb1 <= ~a_nan & ~a_inf & ~ea[13] & (ea > 14'd63);
   end

   // ---- C2, C3: the product (DSPs, two stages); the addend aligned meanwhile ----
   reg  [105:0] pr2, pr3;
   reg  [W-1:0] ca2, ca3;
   reg          cs2, cs3;
   wire [216:0] csh = {1'b0, mc1, 163'd0} >> sh1;           // window bit i is csh bit i + 53: LSB at 110
   reg  [1:0]   f2, f3;
   reg          v2, v3, sub2, sub3, top2, top3, mul2, mul3, pz2, pz3, cz2, cz3;
   reg          spv2, spv3, spn2, spn3, spnv2, spnv3, sps2, sps3, sp2, sp3, sc2, sc3;
   reg          i2f2, f2i2, f2i3, md2, md3, il2, il3, iu2, iu3, xn2, xn3, xi2, xi3, xb2, xb3;
   reg  [2:0]   rm2, rm3;  reg [13:0] wb2, wb3;  reg [63:0] im2;  reg [TAGW-1:0] t2, t3;
   always @(posedge clk) begin
      v2 <= ~reset & ~flush & v1;  v3 <= ~reset & ~flush & v2;
      pr2 <= ma1 * mb1;
      pr3 <= i2f2 ? {42'd0, im2} : pr2;                     // an integer source is the product
      ca2 <= csh[216:53];  cs2 <= |csh[52:0];  ca3 <= ca2;  cs3 <= cs2;
      {f2, sub2, top2, mul2, pz2, cz2, spv2, spn2, spnv2, sps2, sp2, sc2, rm2, wb2, t2} <=
         {f1, sub1, top1, mul1, pz1, cz1, sp_v1, sp_n1, sp_nv1, sp_s1, sp1_q, sc1_q, rm1, wb1, t1};
      {f3, sub3, top3, mul3, pz3, cz3, spv3, spn3, spnv3, sps3, sp3, sc3, rm3, wb3, t3} <=
         {f2, sub2, top2, mul2, pz2, cz2, spv2, spn2, spnv2, sps2, sp2, sc2, rm2, wb2, t2};
      {i2f2, f2i2, md2, il2, iu2, xn2, xi2, xb2, im2} <= {i2f1, f2i1, md1, il1, iu1, xn1, xi1, xb1, im1};
      {f2i3, md3, il3, iu3, xn3, xi3, xb3} <= {f2i2, md2, il2, iu2, xn2, xi2, xb2};
   end

   // ---- C4: the sum's magnitude and sign ----
   wire [W-1:0] pw  = top3 ? {{(W-1){1'b0}}, |pr3} : {56'd0, pr3, 2'b00};
   wire [W-1:0] cw  = ca3 | {{(W-1){1'b0}}, cs3};
   wire [W:0]   x   = sub3 ? {1'b0, pw} - {1'b0, cw} : {1'b0, pw} + {1'b0, cw};
   wire         neg = sub3 & x[W];
   wire [W-1:0] mag = neg ? (~x[W-1:0] + {{(W-1){1'b0}}, 1'b1}) : x[W-1:0];
   reg  [1:0]   f4;
   reg  [W-1:0] mg4;  reg sg4, v4, mul4, pz4, cz4, spv4, spn4, spnv4, sps4, sp4, sc4;
   reg          f2i4, md4, il4, iu4, xn4, xi4, xb4;
   reg  [2:0]   rm4;  reg [13:0] wb4;  reg [TAGW-1:0] t4;
   always @(posedge clk) begin
      v4 <= ~reset & ~flush & v3;
      mg4 <= mag;  sg4 <= f2i3 ? sc3 : neg ? sc3 : sp3;
      {f4, mul4, pz4, cz4, spv4, spn4, spnv4, sps4, sp4, sc4, rm4, wb4, t4} <=
         {f3, mul3, pz3, cz3, spv3, spn3, spnv3, sps3, sp3, sc3, rm3, wb3, t3};
      {f2i4, md4, il4, iu4, xn4, xi4, xb4} <= {f2i3, md3, il3, iu3, xn3, xi3, xb3};
   end

   // ---- C5: normalize (a floating-point result); round to an integer (F2I) ----
   function [7:0] lzc164(input [W-1:0] v);
      integer i;  reg found;  reg [31:0] n;
      begin
         lzc164 = 8'd164;  found = 1'b0;
         for (i = W - 1; i >= 0; i = i - 1)
            if (!found && v[i]) begin n = (W - 1) - i;  lzc164 = n[7:0];  found = 1'b1; end
      end
   endfunction
   function ru_of(input [2:0] rm_, input s, input lsb, input g_, input st_);
      case (rm_)
         3'd0:    ru_of = g_ & (st_ | lsb);
         3'd1:    ru_of = 1'b0;
         3'd2:    ru_of = s & (g_ | st_);
         3'd3:    ru_of = ~s & (g_ | st_);
         default: ru_of = g_;
      endcase
   endfunction
   wire [7:0]   lz  = lzc164(mg4);
   // the sticky bit from the unshifted sum: a bit lies below the guard exactly when the lowest one
   // (tz, the trailing-zero count) is at most 109 - lz positions up (binary32: 138 - lz, binary16:
   // 151 - lz)
   function [7:0] tzc164(input [W-1:0] v);
      integer i;  reg found;  reg [31:0] n;
      begin
         tzc164 = 8'd164;  found = 1'b0;
         for (i = 0; i < W; i = i + 1)
            if (!found && v[i]) begin n = i;  tzc164 = n[7:0];  found = 1'b1; end
      end
   endfunction
   wire [7:0]   tz  = tzc164(mg4);
   wire [8:0]   tl  = {1'b0, tz} + {1'b0, lz};
   wire         zr  = mg4 == {W{1'b0}};
   wire [W-1:0] nm  = mg4 << lz;
   // F2I: bits [65:2] are the integer, 1 its guard, 0 its sticky; anything above 65 is out of range
   wire [64:0]  ri  = {1'b0, mg4[65:2]} + {64'd0, ru_of(rm4, sg4, mg4[2], mg4[1], mg4[0])};
   wire         rhi = |mg4[W-1:66] | ri[64];
   wire         rok = iu4 ? (sg4 ? (ri == 65'd0) : ~rhi & (il4 | (ri[63:32] == 32'd0)))
                          : ~rhi & (sg4 ? (il4 ? (~ri[63] | (ri[62:0] == 63'd0)) : ((ri[63:32] == 32'd0) & (~ri[31] | (ri[30:0] == 31'd0))))
                                        : (il4 ? ~ri[63] : (ri[63:31] == 33'd0)));
   wire [63:0]  rv  = sg4 ? ~ri[63:0] + 64'd1 : ri[63:0];
   wire         bad = xn4 | xi4 | xb4 | ~rok;                         // invalid: saturate, NV
   wire [63:0]  imax = iu4 ? 64'hFFFF_FFFF_FFFF_FFFF : il4 ? 64'h7FFF_FFFF_FFFF_FFFF : 64'h0000_0000_7FFF_FFFF;
   wire [63:0]  imin = iu4 ? 64'd0 : il4 ? 64'h8000_0000_0000_0000 : 64'hFFFF_FFFF_8000_0000;
   // fcvtmod.w.d keeps the low 32 bits whatever the range (the window holds them up to 2^161; above
   // that, and from 2^84, they are zero) and gives 0 for a NaN or an infinity
   wire [63:0]  ires = md4 ? ((xn4 | xi4) ? 64'd0 : {{32{rv[31]}}, rv[31:0]})
                     : bad ? ((sg4 & ~xn4) ? imin : imax) : il4 ? rv : {{32{rv[31]}}, rv[31:0]};
   reg  [1:0]   f5;
   reg  [52:0]  m5;  reg g5, s5, zr5, v5, mul5, pz5, cz5, sg5, spv5, spn5, spnv5, sps5, sp5, sc5;
   reg          f2i5;  reg [63:0] ir5;  reg [4:0] if5;
   reg  [2:0]   rm5;  reg [13:0] e5;  reg [TAGW-1:0] t5;
   always @(posedge clk) begin
      v5 <= ~reset & ~flush & v4;
      // binary64: 53 bits, guard, sticky; binary32: 24 bits right-aligned; binary16: 11
      m5 <= (f4 == 2'd1) ? nm[W-1:W-53] : (f4 == 2'd2) ? {42'd0, nm[W-1:W-11]} : {29'd0, nm[W-1:W-24]};
      g5 <= (f4 == 2'd1) ? nm[W-54] : (f4 == 2'd2) ? nm[W-12] : nm[W-25];
      s5 <= ~zr & (tl < ((f4 == 2'd1) ? 9'd110 : (f4 == 2'd2) ? 9'd152 : 9'd139));
      e5 <= wb4 + 14'd163 - {6'd0, lz};                      // the leading bit's exponent
      zr5 <= zr;
      f2i5 <= f2i4;  ir5 <= ires;  if5 <= {bad, 3'b000, ~bad & (mg4[1] | mg4[0])};
      {f5, mul5, pz5, cz5, sg5, spv5, spn5, spnv5, sps5, sp5, sc5, rm5, t5} <=
         {f4, mul4, pz4, cz4, sg4, spv4, spn4, spnv4, sps4, sp4, sc4, rm4, t4};
   end

   // ---- C6: denormalize, round, pack (into fp_unit's result register) ----
   function [63:0] pack(input [1:0] fm, input s, input [10:0] xx, input [51:0] fr);
      pack = (fm == 2'd1) ? {s, xx, fr} : (fm == 2'd2) ? {48'hFFFF_FFFF_FFFF, s, xx[4:0], fr[9:0]}
                                                      : {32'hFFFF_FFFF, s, xx[7:0], fr[22:0]};
   endfunction
   wire        d5   = f5 == 2'd1, h5 = f5 == 2'd2;
   wire [13:0] be   = e5 + (d5 ? 14'd1023 : h5 ? 14'd15 : 14'd127);   // biased
   wire        den  = be[13] | (be == 14'd0);
   wire [13:0] dsh  = 14'd1 - be;
   wire [5:0]  dn   = ~den ? 6'd0 : (dsh > 14'd55) ? 6'd55 : dsh[5:0];
   wire        all1 = d5 ? &m5 : h5 ? &m5[10:0] : &m5[23:0];
   wire        tiny = be[13] | ((be == 14'd0) & ~(all1 & ru_of(rm5, sg5, 1'b1, g5, s5)));
   // the right shift: {m, g} by dn, every bit it drops into the sticky bit
   wire [108:0] dsx = {m5, g5, 55'd0} >> dn;
   wire [52:0]  md  = dsx[108:56];
   wire         gd  = dsx[55];
   wire         sd  = s5 | (|dsx[54:0]);
   wire         ru  = ru_of(rm5, sg5, md[0], gd, sd);
   wire [53:0]  m1  = {1'b0, md} + {53'd0, ru};
   wire         cy  = d5 ? m1[53] : h5 ? m1[11] : m1[24];
   wire         hid = d5 ? m1[52] : h5 ? m1[10] : m1[23];
   wire         nx  = gd | sd;
   wire [13:0]  xr  = den ? {13'd0, hid} : be + {13'd0, cy};
   wire         ovf = ~den & (xr >= (d5 ? 14'd2047 : h5 ? 14'd31 : 14'd255));
   wire         tmx = (rm5 == 3'd1) | ((rm5 == 3'd2) & ~sg5) | ((rm5 == 3'd3) & sg5);
   wire [51:0]  frr = (~den & cy) ? 52'd0 : (d5 ? m1[51:0] : h5 ? {42'd0, m1[9:0]} : {29'd0, m1[22:0]});
   wire [10:0]  xmax = d5 ? 11'h7FF : h5 ? 11'h01F : 11'h0FF;     // the infinities' exponent
   wire [63:0]  r_num = ovf ? (tmx ? pack(f5, sg5, xmax - 11'd1, 52'hF_FFFF_FFFF_FFFF)
                                   : pack(f5, sg5, xmax, 52'd0))
                              : pack(f5, sg5, xr[10:0], frr);
   // an exact zero: a product without an addend keeps its sign; two zeros of a sign keep it (RDN:
   // either negative gives -0); a cancellation is +0 (RDN: -0)
   wire         zs  = mul5 ? sp5 : (pz5 & cz5) ? ((rm5 == 3'd2) ? (sp5 | sc5) : (sp5 & sc5)) : (rm5 == 3'd2);
   wire [63:0]  r_sp = spn5 ? pack(f5, 1'b0, xmax, d5 ? 52'h8_0000_0000_0000 : h5 ? 52'h200 : 52'h40_0000)
                            : pack(f5, sps5, xmax, 52'd0);
   assign out_valid  = v5;
   assign out_tag    = t5;
   assign out_result = f2i5 ? ir5 : spv5 ? r_sp : zr5 ? pack(f5, zs, 11'd0, 52'd0) : r_num;
   assign out_flags  = f2i5 ? if5 : spv5 ? {spnv5, 4'd0} : zr5 ? 5'd0 : {2'b00, ovf, tiny & nx & ~ovf, nx | ovf};

   always @(posedge clk) if (!reset && v0) begin
      if (rm0 > 3'd4)
         $fatal(1, "fp_fma: rounding mode %0d (DYN must be resolved before the unit)", rm0);
      if ((sf0 == 2'd3) | (df0 == 2'd3) | (((sf0 == 2'd2) | (df0 == 2'd2)) & (op0 != 4'd7)))
         $fatal(1, "fp_fma: format %0d -> %0d for op %0d (binary16 only converts)", sf0, df0, op0);
      if (op0 > 4'd10)
         $fatal(1, "fp_fma: op %0d", op0);
   end
endmodule

`default_nettype wire
