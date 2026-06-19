// Floating-point classify and compare helpers for the smolrv64 core.
// Pure combinational functions of their operands. `include inside the module
// body. (Classification/compare are done in the core; arithmetic is the CV-FPU.)

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
   // op (from insn[14:12]): 000=FLE, 001=FLT, 010=FEQ.
   // FEQ sets NV only on signaling NaN; FLT/FLE set NV on any NaN.
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
            default: fcmp_d = 2'b00;
         endcase
      end
   endfunction
