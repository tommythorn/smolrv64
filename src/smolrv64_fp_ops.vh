// Floating-point classify and compare helpers for the smolrv64 core.
// Pure combinational functions of their operands. `include inside the module
// body. (Classification, the compares, FMIN/FMAX, Zfa's FLI and FROUND are done in the core;
// arithmetic is fp_unit's.)

   function [63:0] fclass_d;
      input [63:0] v;
      reg         sign;
      reg [10:0]  exp;
      reg [51:0]  mant;
      reg         exp_all1, exp_0, mant_0, qbit;
      begin
         sign = v[63]; exp = v[62:52]; mant = v[51:0];
         exp_all1 = &exp;  exp_0 = exp == 0;  mant_0 = mant == 0;  qbit = mant[51];
         if      (exp_all1 && mant_0)   fclass_d = sign ? 64'h001 : 64'h080; // ±inf
         else if (exp_all1)             fclass_d = qbit ? 64'h200 : 64'h100; // qNaN / sNaN
         else if (exp_0 && mant_0)      fclass_d = sign ? 64'h008 : 64'h010; // ±0
         else if (exp_0)                fclass_d = sign ? 64'h004 : 64'h020; // ±subnormal
         else                           fclass_d = sign ? 64'h002 : 64'h040; // ±normal
      end
   endfunction

   function [63:0] fclass_s;
      input [63:0] v;
      reg         sign;
      reg [ 7:0]  exp;
      reg [22:0]  mant;
      reg         exp_all1, exp_0, mant_0, qbit;
      begin
         if (~(&v[63:32])) fclass_s = 64'h200; // improperly NaN-boxed → canonical qNaN
         else begin
            sign = v[31]; exp = v[30:23]; mant = v[22:0];
            exp_all1 = &exp;  exp_0 = exp == 0;  mant_0 = mant == 0;  qbit = mant[22];
            if      (exp_all1 && mant_0)   fclass_s = sign ? 64'h001 : 64'h080;
            else if (exp_all1)             fclass_s = qbit ? 64'h200 : 64'h100;
            else if (exp_0 && mant_0)      fclass_s = sign ? 64'h008 : 64'h010;
            else if (exp_0)                fclass_s = sign ? 64'h004 : 64'h020;
            else                           fclass_s = sign ? 64'h002 : 64'h040;
         end
      end
   endfunction

   // FP compares: returns {nv, result} where nv→fflags.NV, result→integer rd bit 0.
   // op (from insn[14:12]): 000=FLE, 001=FLT, 010=FEQ, and Zfa's quiet 100=FLEQ, 101=FLTQ.
   // FEQ, FLEQ and FLTQ set NV only on a signaling NaN; FLT/FLE set NV on any NaN.
   function [1:0] fcmp_s;
      input [2:0] op;
      input [31:0] a, b;
      reg a_nan, b_nan, a_snan, b_snan, both_zero, eq, lt, le;
      begin
         a_nan = (a[30:23] == 8'hff) && (a[22:0] != 0);
         b_nan = (b[30:23] == 8'hff) && (b[22:0] != 0);
         a_snan = a_nan && !a[22];
         b_snan = b_nan && !b[22];
         both_zero = (a[30:0] == 0) && (b[30:0] == 0);
         if (a_nan || b_nan)        begin eq = 0; lt = 0; le = 0; end
         else if (both_zero)        begin eq = 1; lt = 0; le = 1; end
         else if (a[31] != b[31])   begin eq = 0; lt = a[31]; le = a[31]; end
         else if (!a[31])           begin eq = (a == b); lt = (a <  b); le = (a <= b); end
         else                       begin eq = (a == b); lt = (a >  b); le = (a >= b); end
         case (op)
            3'b000:  fcmp_s = {a_nan  | b_nan,  le};
            3'b001:  fcmp_s = {a_nan  | b_nan,  lt};
            3'b010:  fcmp_s = {a_snan | b_snan, eq};
            3'b100:  fcmp_s = {a_snan | b_snan, le};
            3'b101:  fcmp_s = {a_snan | b_snan, lt};
            default: fcmp_s = 2'b00;
         endcase
      end
   endfunction

   function [1:0] fcmp_d;
      input [2:0] op;
      input [63:0] a, b;
      reg a_nan, b_nan, a_snan, b_snan, both_zero, eq, lt, le;
      begin
         a_nan = (a[62:52] == 11'h7ff) && (a[51:0] != 0);
         b_nan = (b[62:52] == 11'h7ff) && (b[51:0] != 0);
         a_snan = a_nan && !a[51];
         b_snan = b_nan && !b[51];
         both_zero = (a[62:0] == 0) && (b[62:0] == 0);
         if (a_nan || b_nan)        begin eq = 0; lt = 0; le = 0; end
         else if (both_zero)        begin eq = 1; lt = 0; le = 1; end
         else if (a[63] != b[63])   begin eq = 0; lt = a[63]; le = a[63]; end
         else if (!a[63])           begin eq = (a == b); lt = (a <  b); le = (a <= b); end
         else                       begin eq = (a == b); lt = (a >  b); le = (a >= b); end
         case (op)
            3'b000:  fcmp_d = {a_nan  | b_nan,  le};
            3'b001:  fcmp_d = {a_nan  | b_nan,  lt};
            3'b010:  fcmp_d = {a_snan | b_snan, eq};
            3'b100:  fcmp_d = {a_snan | b_snan, le};
            3'b101:  fcmp_d = {a_snan | b_snan, lt};
            default: fcmp_d = 2'b00;
         endcase
      end
   endfunction

   // FMIN/FMAX: {nv, result}. `le_lt` is fcmp's answer for the instruction's own funct3 -- FLE (000)
   // for FMIN, FLT (001) for FMAX -- so FMIN takes a when a <= b and FMAX takes b when a < b. A NaN
   // operand gives the other one, two give the canonical NaN, NV only for a signaling NaN; of two
   // zeros FMIN gives -0 and FMAX +0 unless both are -0 / +0. Zfa's FMINM/FMAXM (`m`) give the
   // canonical NaN for a NaN in either operand. The binary32 form takes unboxed operands (an unboxed
   // one is already the canonical NaN) and boxes its result.
   function [64:0] fminmax_d;
      input        max, m;
      input [63:0] a, b;
      input        le_lt;
      reg a_nan, b_nan;
      begin
         a_nan = (a[62:52] == 11'h7ff) && (a[51:0] != 0);
         b_nan = (b[62:52] == 11'h7ff) && (b[51:0] != 0);
         fminmax_d[64] = (a_nan && !a[51]) || (b_nan && !b[51]);
         if (a_nan && b_nan || (m && (a_nan || b_nan))) fminmax_d[63:0] = 64'h7ff8_0000_0000_0000;
         else if (a_nan)                            fminmax_d[63:0] = b;
         else if (b_nan)                            fminmax_d[63:0] = a;
         else if (a[62:0] == 0 && b[62:0] == 0)     fminmax_d[63:0] = {max ? a[63] & b[63] : a[63] | b[63], 63'd0};
         else                                       fminmax_d[63:0] = (max ? le_lt : ~le_lt) ? b : a;
      end
   endfunction

   function [64:0] fminmax_s;
      input        max, m;
      input [31:0] a, b;
      input        le_lt;
      reg a_nan, b_nan;
      begin
         a_nan = (a[30:23] == 8'hff) && (a[22:0] != 0);
         b_nan = (b[30:23] == 8'hff) && (b[22:0] != 0);
         fminmax_s[64] = (a_nan && !a[22]) || (b_nan && !b[22]);
         fminmax_s[63:32] = 32'hffff_ffff;
         if (a_nan && b_nan || (m && (a_nan || b_nan))) fminmax_s[31:0] = 32'h7fc0_0000;
         else if (a_nan)                            fminmax_s[31:0] = b;
         else if (b_nan)                            fminmax_s[31:0] = a;
         else if (a[30:0] == 0 && b[30:0] == 0)     fminmax_s[31:0] = {max ? a[31] & b[31] : a[31] | b[31], 31'd0};
         else                                       fminmax_s[31:0] = (max ? le_lt : ~le_lt) ? b : a;
      end
   endfunction
   // Zfa FROUND / FROUNDNX: round to an integral value in the operand's own format under rm (resolved:
   // RNE RTZ RDN RUP RMM). {flags, result}: a NaN gives the canonical NaN (NV for a signaling one), a
   // zero, an infinity and anything from 2^52 (binary32: 2^23) up pass unchanged; a zero result keeps
   // the operand's sign. The kept bits are the operand's own with the fraction below the binary point
   // cleared, and rounding up adds one at the binary point (a carry into the exponent is the next
   // power of two). FROUNDNX (nx) raises NX when a fraction was dropped.
   function fp_rup(input [2:0] rm, input s, input lsb, input g, input st);
      case (rm)
         3'd0:    fp_rup = g & (st | lsb);
         3'd1:    fp_rup = 1'b0;
         3'd2:    fp_rup = s & (g | st);
         3'd3:    fp_rup = ~s & (g | st);
         default: fp_rup = g;
      endcase
   endfunction
   function [68:0] fround_d;
      input [63:0] a;  input [2:0] rm;  input nx;
      reg [10:0] ex, kk;  reg [5:0] k;  reg [51:0] lo, gs, ls;  reg g, st, lsb, up, nz;
      begin
         ex = a[62:52];
         kk = 11'd1075 - ex;  k = kk[5:0];                // fraction bits below the point, 1..52
         lo = (52'd1 << k) - 52'd1;                       // ... as a mask (k = 52: all of them)
         nz = (ex != 11'd0) | (a[51:0] != 52'd0);
         if (&ex)                                         // NaN or infinity
            fround_d = (a[51:0] != 52'd0) ? {~a[51], 4'd0, 64'h7ff8_0000_0000_0000} : {5'd0, a};
         else if (ex >= 11'd1075)                         // already integral
            fround_d = {5'd0, a};
         else if (ex < 11'd1023) begin                    // below one: 0 or 1, the sign kept
            g  = ex == 11'd1022;                          // at least a half
            st = g ? (a[51:0] != 52'd0) : nz;
            up = fp_rup(rm, a[63], 1'b0, g, st);
            fround_d = {4'd0, nx & nz, a[63], up ? 63'h3ff0_0000_0000_0000 : 63'd0};
         end else begin
            gs  = a[51:0] >> (k - 6'd1);  ls = a[51:0] >> k;
            g   = gs[0];
            st  = (a[51:0] & (lo >> 1)) != 52'd0;
            lsb = (k == 6'd52) | ls[0];                   // k = 52: the hidden bit
            up  = fp_rup(rm, a[63], lsb, g, st);
            fround_d = {4'd0, nx & (g | st), (a & ~{12'd0, lo}) + (up ? {12'd0, lo} + 64'd1 : 64'd0)};
         end
      end
   endfunction
   function [68:0] fround_s;                              // a: unboxed; the result is boxed
      input [31:0] a;  input [2:0] rm;  input nx;
      reg [7:0] ex, kk;  reg [4:0] k;  reg [22:0] lo, gs, ls;  reg g, st, lsb, up, nz;
      begin
         ex = a[30:23];
         kk = 8'd150 - ex;  k = kk[4:0];                  // 1..23
         lo = (23'd1 << k) - 23'd1;
         nz = (ex != 8'd0) | (a[22:0] != 23'd0);
         if (&ex)
            fround_s = (a[22:0] != 23'd0) ? {~a[22], 4'd0, 64'hffff_ffff_7fc0_0000} : {5'd0, 32'hffff_ffff, a};
         else if (ex >= 8'd150)
            fround_s = {5'd0, 32'hffff_ffff, a};
         else if (ex < 8'd127) begin
            g  = ex == 8'd126;
            st = g ? (a[22:0] != 23'd0) : nz;
            up = fp_rup(rm, a[31], 1'b0, g, st);
            fround_s = {4'd0, nx & nz, 32'hffff_ffff, a[31], up ? 31'h3f80_0000 : 31'd0};
         end else begin
            gs  = a[22:0] >> (k - 5'd1);  ls = a[22:0] >> k;
            g   = gs[0];
            st  = (a[22:0] & (lo >> 1)) != 23'd0;
            lsb = (k == 5'd23) | ls[0];
            up  = fp_rup(rm, a[31], lsb, g, st);
            fround_s = {4'd0, nx & (g | st), 32'hffff_ffff, (a & ~{9'd0, lo}) + (up ? {9'd0, lo} + 32'd1 : 32'd0)};
         end
      end
   endfunction
   function [63:0] fli_d;          // Zfa FLI: entry i of the constant table
      input [4:0] i;
      case (i)
         5'd0 : fli_d = 64'hbff0000000000000;
         5'd1 : fli_d = 64'h0010000000000000;
         5'd2 : fli_d = 64'h3ef0000000000000;
         5'd3 : fli_d = 64'h3f00000000000000;
         5'd4 : fli_d = 64'h3f70000000000000;
         5'd5 : fli_d = 64'h3f80000000000000;
         5'd6 : fli_d = 64'h3fb0000000000000;
         5'd7 : fli_d = 64'h3fc0000000000000;
         5'd8 : fli_d = 64'h3fd0000000000000;
         5'd9 : fli_d = 64'h3fd4000000000000;
         5'd10: fli_d = 64'h3fd8000000000000;
         5'd11: fli_d = 64'h3fdc000000000000;
         5'd12: fli_d = 64'h3fe0000000000000;
         5'd13: fli_d = 64'h3fe4000000000000;
         5'd14: fli_d = 64'h3fe8000000000000;
         5'd15: fli_d = 64'h3fec000000000000;
         5'd16: fli_d = 64'h3ff0000000000000;
         5'd17: fli_d = 64'h3ff4000000000000;
         5'd18: fli_d = 64'h3ff8000000000000;
         5'd19: fli_d = 64'h3ffc000000000000;
         5'd20: fli_d = 64'h4000000000000000;
         5'd21: fli_d = 64'h4004000000000000;
         5'd22: fli_d = 64'h4008000000000000;
         5'd23: fli_d = 64'h4010000000000000;
         5'd24: fli_d = 64'h4020000000000000;
         5'd25: fli_d = 64'h4030000000000000;
         5'd26: fli_d = 64'h4060000000000000;
         5'd27: fli_d = 64'h4070000000000000;
         5'd28: fli_d = 64'h40e0000000000000;
         5'd29: fli_d = 64'h40f0000000000000;
         5'd30: fli_d = 64'h7ff0000000000000;
         5'd31: fli_d = 64'h7ff8000000000000;
         default: fli_d = 64'd0;
      endcase
   endfunction
   function [31:0] fli_s;          // Zfa FLI: entry i of the constant table
      input [4:0] i;
      case (i)
         5'd0 : fli_s = 32'hbf800000;
         5'd1 : fli_s = 32'h00800000;
         5'd2 : fli_s = 32'h37800000;
         5'd3 : fli_s = 32'h38000000;
         5'd4 : fli_s = 32'h3b800000;
         5'd5 : fli_s = 32'h3c000000;
         5'd6 : fli_s = 32'h3d800000;
         5'd7 : fli_s = 32'h3e000000;
         5'd8 : fli_s = 32'h3e800000;
         5'd9 : fli_s = 32'h3ea00000;
         5'd10: fli_s = 32'h3ec00000;
         5'd11: fli_s = 32'h3ee00000;
         5'd12: fli_s = 32'h3f000000;
         5'd13: fli_s = 32'h3f200000;
         5'd14: fli_s = 32'h3f400000;
         5'd15: fli_s = 32'h3f600000;
         5'd16: fli_s = 32'h3f800000;
         5'd17: fli_s = 32'h3fa00000;
         5'd18: fli_s = 32'h3fc00000;
         5'd19: fli_s = 32'h3fe00000;
         5'd20: fli_s = 32'h40000000;
         5'd21: fli_s = 32'h40200000;
         5'd22: fli_s = 32'h40400000;
         5'd23: fli_s = 32'h40800000;
         5'd24: fli_s = 32'h41000000;
         5'd25: fli_s = 32'h41800000;
         5'd26: fli_s = 32'h43000000;
         5'd27: fli_s = 32'h43800000;
         5'd28: fli_s = 32'h47000000;
         5'd29: fli_s = 32'h47800000;
         5'd30: fli_s = 32'h7f800000;
         5'd31: fli_s = 32'h7fc00000;
         default: fli_s = 32'd0;
      endcase
   endfunction
