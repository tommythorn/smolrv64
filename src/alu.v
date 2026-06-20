// -----------------------------------------------------------------------
//
// A purely combinational RVA22 (RV64I + Zba + Zbb + Zbs) ALU — plus the RVA23
// Zicond conditional-zero ops (czero.eqz/czero.nez) — with predecoded
// steering. Evolved from the yarvi RV64I ALU; the elegant shared adder/compare
// is preserved and the bit-manipulation extensions are layered on with maximal
// unit sharing:
//
//   - one adder         : ADD/SUB, the SLT/SLTU compares, MIN/MAX, and the Zba
//                         shift-add family (sh{1,2,3}add[.uw], add.uw)
//   - one funnel shifter: SLL/SRL/SRA, ROL/ROR (Zbb), BEXT (Zbs), slli.uw
//   - one logic unit    : AND/OR/XOR, ANDN/ORN/XNOR (Zbb), BCLR/BSET/BINV (Zbs)
//   - a count unit      : CLZ/CTZ/CPOP (Zbb)
//   - byte/permute      : REV8/ORC.B, SEXT.B/SEXT.H/ZEXT.H (Zbb)
//
// The ALU is *not* the pipeline critical path (it benchmarks ~570 MHz on
// xcku5p-2 even with this logic), so the design favours clarity and unit reuse;
// op1/op2 and op are assumed registered by the surrounding stage.
//
// ISC License.  Copyright (C) 2014 - 2026  Tommy Thorn
// -----------------------------------------------------------------------

/* verilator lint_off WIDTH */

// Operation selector. Grouped by functional unit; the surrounding decoder maps
// instructions onto these. `w` (word) and `uw` (zero-extend op1 from 32 bits)
// are orthogonal modifiers.
`define ALU_ADD    6'd0
`define ALU_SUB    6'd1
`define ALU_SH1ADD 6'd2
`define ALU_SH2ADD 6'd3
`define ALU_SH3ADD 6'd4

`define ALU_SLT    6'd8
`define ALU_SLTU   6'd9
`define ALU_MIN    6'd10
`define ALU_MINU   6'd11
`define ALU_MAX    6'd12
`define ALU_MAXU   6'd13

`define ALU_SLL    6'd16
`define ALU_SRL    6'd17
`define ALU_SRA    6'd18
`define ALU_ROL    6'd19
`define ALU_ROR    6'd20
`define ALU_BEXT   6'd21

`define ALU_AND    6'd24
`define ALU_OR     6'd25
`define ALU_XOR    6'd26
`define ALU_ANDN   6'd27
`define ALU_ORN    6'd28
`define ALU_XNOR   6'd29
`define ALU_BCLR   6'd30
`define ALU_BSET   6'd31
`define ALU_BINV   6'd32

`define ALU_CZEQZ  6'd33   // Zicond czero.eqz (RVA23): rd = (rs2==0) ? 0 : rs1
`define ALU_CZNEZ  6'd34   // Zicond czero.nez (RVA23): rd = (rs2!=0) ? 0 : rs1

`define ALU_CLZ    6'd40
`define ALU_CTZ    6'd41
`define ALU_CPOP   6'd42
`define ALU_REV8   6'd43
`define ALU_ORCB   6'd44
`define ALU_SEXTB  6'd45
`define ALU_SEXTH  6'd46
`define ALU_ZEXTH  6'd47

`default_nettype none

module alu #(parameter XLEN = 64, parameter MSB = XLEN - 1)
   (input  wire [   5:0] op,    // ALU_* operation
    input  wire          w,     // 1 = 32-bit word op (RV64 *W instructions)
    input  wire          uw,    // 1 = zero-extend op1 from 32 bits (Zba .uw / slli.uw)
    input  wire [MSB:0]  op1,
    input  wire [MSB:0]  op2,

    output reg  [MSB:0]  result,
    output wire [MSB:0]  sum,   // op1 + op2 (kept live for address gen / branches)
    output wire          eq,
    output wire          lt,    // signed  op1 < op2
    output wire          ltu);  // unsigned op1 < op2

   localparam SHW = $clog2(XLEN);   // shift-amount width (6 for XLEN=64)

   // -------- bit-reverse helpers (free wiring) --------
   function [XLEN-1:0] brevx;
      input [XLEN-1:0] x; integer i;
      for (i = 0; i < XLEN; i = i + 1) brevx[i] = x[XLEN-1-i];
   endfunction
   function [31:0] brev32;
      input [31:0] x; integer i;
      for (i = 0; i < 32; i = i + 1) brev32[i] = x[31-i];
   endfunction

   // -------- shared adder / comparator --------
   assign      sum = op1 + op2;
   wire [XLEN:0] dif = {1'b0, op1} - {1'b0, op2};
   assign      eq  = op1 == op2;
   assign      ltu = dif[XLEN];
   assign      lt  = (op1[MSB] == op2[MSB]) ? dif[MSB] : op1[MSB];

   // -------- arithmetic --------
   // Plain ADD/SUB feed op1/op2 straight into the (shared) sum/dif adders, so
   // the common path has no pre-adder mux. Zba shift-add and add.uw use their
   // own adders and are merged in at a shallow final-mux level.
   wire [1:0]    zsh    = (op == `ALU_SH1ADD) ? 2'd1 :
                          (op == `ALU_SH2ADD) ? 2'd2 :
                          (op == `ALU_SH3ADD) ? 2'd3 : 2'd0;
   wire [MSB:0]  op1_uw  = {{(XLEN-32){1'b0}}, op1[31:0]};
   wire [MSB:0]  addsub  = (op == `ALU_SUB) ? dif[MSB:0] : sum;  // ADD / SUB
   wire [MSB:0]  zba_sum = ((uw ? op1_uw : op1) << zsh) + op2;   // sh{1,2,3}add[.uw]
   wire [MSB:0]  adduw   = op1_uw + op2;                         // add.uw

   // -------- one funnel shifter: SLL/SRL/SRA/ROL/ROR/BEXT/slli.uw --------
   // Right funnel ({hi,lo} >> shamt); left shifts use the bit-reverse identity
   //   x << s == rev(rev(x) >> s),   rol(x,s) == rev(ror(rev(x),s)).
   // The *W word ops share this funnel: their result is result[31:0], and with
   // shamt<=31 those bits come only from the low operand, so we pack the 32-bit
   // operand+fill into the low half and ignore the high half for word.
   wire is_left = (op == `ALU_SLL) || (op == `ALU_ROL);
   wire is_rot  = (op == `ALU_ROL) || (op == `ALU_ROR);
   wire is_sra  = (op == `ALU_SRA);

   wire [SHW-1:0] sh64 = op2[SHW-1:0];
   wire [4:0]     sh32 = op2[4:0];
   wire [31:0]    o32  = op1[31:0];

   // 64-bit operand (uw zero-extends op1 for slli.uw only).
   wire [XLEN-1:0] sh_in  = (uw && op == `ALU_SLL) ? {{(XLEN-32){1'b0}}, o32} : op1;
   wire [XLEN-1:0] base64 = is_left ? brevx(sh_in) : sh_in;
   wire [XLEN-1:0] hi64   = is_rot ? base64 : (is_sra ? {XLEN{base64[MSB]}} : {XLEN{1'b0}});

   // Word operand packed into 64 bits: low = base, high = fill (0 / sign / wrap).
   wire [31:0] base32 = is_left ? brev32(o32) : o32;
   wire [31:0] hi32   = is_rot ? base32 : (is_sra ? {32{base32[31]}} : 32'b0);

   // Single shared funnel.
   wire [XLEN-1:0]   lo  = w ? {hi32, base32} : base64;
   wire [SHW-1:0]    sh  = w ? {{(SHW-5){1'b0}}, sh32} : sh64;
   wire [2*XLEN-1:0] fun = {hi64, lo} >> sh;
   wire [XLEN-1:0]   fr  = fun[XLEN-1:0];

   wire [XLEN-1:0] shift_raw =
        is_left ? (w ? {{(XLEN-32){1'b0}}, brev32(fr[31:0])} : brevx(fr)) : fr;
   wire [XLEN-1:0] shift_res =
        w ? {{(XLEN-32){shift_raw[31]}}, shift_raw[31:0]} : shift_raw;

   // -------- count unit: CLZ/CTZ/CPOP (and *W variants via `w`) --------
   function [6:0] count_lz;          // leading zeros of the low `n` bits
      input [XLEN-1:0] x; input word; integer i; integer n;
      begin n = word ? 32 : XLEN; count_lz = n;
            for (i = 0; i < XLEN; i = i + 1) if (i < n && x[i]) count_lz = (n-1) - i; end
   endfunction
   function [6:0] count_pop;         // population count of the low `n` bits
      input [XLEN-1:0] x; input word; integer i; integer n;
      begin n = word ? 32 : XLEN; count_pop = 0;
            for (i = 0; i < XLEN; i = i + 1) if (i < n) count_pop = count_pop + x[i]; end
   endfunction
   wire [XLEN-1:0] op1_rev = w ? {{(XLEN-32){1'b0}}, brev32(o32)} : brevx(op1);
   wire [6:0] clz_n = count_lz(op1, w);
   wire [6:0] ctz_n = count_lz(op1_rev, w);   // trailing zeros == leading zeros of reverse
   wire [6:0] pop_n = count_pop(op1, w);

   // -------- byte / permute unit (Zbb) — XLEN=64 layout --------
   wire [XLEN-1:0] rev8 = {op1[ 7: 0], op1[15: 8], op1[23:16], op1[31:24],
                           op1[39:32], op1[47:40], op1[55:48], op1[63:56]};
   function [XLEN-1:0] orcb_f;
      input [XLEN-1:0] x; integer i;
      for (i = 0; i < XLEN/8; i = i + 1) orcb_f[8*i +: 8] = (|x[8*i +: 8]) ? 8'hFF : 8'h00;
   endfunction
   wire [XLEN-1:0] orcb = orcb_f(op1);

   // -------- single-bit (Zbs) mask: 1 << shamt --------
   wire [XLEN-1:0] bmask = {{(XLEN-1){1'b0}}, 1'b1} << (w ? op2[4:0] : op2[SHW-1:0]);

   // -------- result groups (each a small mux, computed in parallel) --------
   reg [MSB:0] r_cmp;     // compares + min/max (from the subtractor)
   always @(*) case (op)
        `ALU_SLT:   r_cmp = {{MSB{1'b0}}, lt};
        `ALU_SLTU:  r_cmp = {{MSB{1'b0}}, ltu};
        `ALU_MIN:   r_cmp = lt  ? op1 : op2;
        `ALU_MINU:  r_cmp = ltu ? op1 : op2;
        `ALU_MAX:   r_cmp = lt  ? op2 : op1;
        default:    r_cmp = ltu ? op2 : op1;   // MAXU
   endcase

   reg [MSB:0] r_bit;     // logic (Zbb negated-logic, Zbs single-bit)
   always @(*) case (op)
        `ALU_OR:    r_bit = op1 | op2;
        `ALU_XOR:   r_bit = op1 ^ op2;
        `ALU_ANDN:  r_bit = op1 & ~op2;
        `ALU_ORN:   r_bit = op1 | ~op2;
        `ALU_XNOR:  r_bit = ~(op1 ^ op2);
        `ALU_BCLR:  r_bit = op1 & ~bmask;
        `ALU_BSET:  r_bit = op1 |  bmask;
        `ALU_BINV:  r_bit = op1 ^  bmask;
        default:    r_bit = op1 & op2;         // AND
   endcase

   wire op2_zero = (op2 == {XLEN{1'b0}});   // Zicond condition

   reg [MSB:0] r_ext;     // count / permute / extend / single-bit / Zicond
   always @(*) case (op)
        `ALU_CLZ:   r_ext = {{(XLEN-7){1'b0}}, clz_n};
        `ALU_CTZ:   r_ext = {{(XLEN-7){1'b0}}, ctz_n};
        `ALU_CPOP:  r_ext = {{(XLEN-7){1'b0}}, pop_n};
        `ALU_REV8:  r_ext = rev8;
        `ALU_ORCB:  r_ext = orcb;
        `ALU_SEXTB: r_ext = {{(XLEN- 8){op1[ 7]}}, op1[ 7:0]};
        `ALU_SEXTH: r_ext = {{(XLEN-16){op1[15]}}, op1[15:0]};
        `ALU_ZEXTH: r_ext = {{(XLEN-16){1'b0}},    op1[15:0]};
        `ALU_CZEQZ: r_ext = op2_zero ? {XLEN{1'b0}} : op1;   // czero.eqz
        `ALU_CZNEZ: r_ext = op2_zero ? op1 : {XLEN{1'b0}};   // czero.nez
        default:    r_ext = {{MSB{1'b0}}, shift_res[0]};   // BEXT
   endcase

   reg [MSB:0] r_arith;   // Zba shift-add / add.uw (ADD/SUB handled separately)
   always @(*) case (op)
        `ALU_SH1ADD, `ALU_SH2ADD, `ALU_SH3ADD: r_arith = zba_sum;
        default:                               r_arith = adduw;   // add.uw
   endcase

   // -------- balanced final select --------
   // The two slowest data sources — the carry-chain adder (ADD/SUB) and the
   // funnel shifter — get the shallowest mux paths (one 2:1 each); the faster
   // logic/compare/extend/Zba groups sit deeper in the tree.
   wire is_addsub = (op == `ALU_ADD && !uw) || (op == `ALU_SUB);  // add.uw -> r_arith
   wire is_shift  = (op == `ALU_SLL) || (op == `ALU_SRL) || (op == `ALU_SRA) ||
                    (op == `ALU_ROL) || (op == `ALU_ROR);
   wire is_cmp    = (op == `ALU_SLT)|| (op == `ALU_SLTU)||
                    (op == `ALU_MIN)|| (op == `ALU_MINU)||
                    (op == `ALU_MAX)|| (op == `ALU_MAXU);
   wire is_bit    = (op == `ALU_AND)|| (op == `ALU_OR) || (op == `ALU_XOR) ||
                    (op == `ALU_ANDN)||(op == `ALU_ORN)||(op == `ALU_XNOR)||
                    (op == `ALU_BCLR)||(op == `ALU_BSET)||(op == `ALU_BINV);
   wire is_zba    = (op == `ALU_SH1ADD)||(op == `ALU_SH2ADD)||(op == `ALU_SH3ADD)||
                    (op == `ALU_ADD && uw);   // add.uw rides the r_arith group

   wire [MSB:0] r_g0 = is_cmp  ? r_cmp   : r_ext;     // fast groups, deepest
   wire [MSB:0] r_g1 = is_bit  ? r_bit   : r_g0;
   wire [MSB:0] r_g2 = is_zba  ? r_arith : r_g1;
   wire [MSB:0] r_g3 = is_shift ? shift_res : r_g2;   // shifter: one 2:1
   wire [MSB:0] r_pre = is_addsub ? addsub : r_g3;    // adder: one 2:1

   always @(*) begin
      result = r_pre;
      // *W instructions: sign-extend the low 32 bits of the result.
      if (XLEN != 32 && w)
         result = {{XLEN/2{result[XLEN/2-1]}}, result[XLEN/2-1:0]};
   end
endmodule

`default_nettype wire
