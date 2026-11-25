// -----------------------------------------------------------------------
//
// A purely combinational RVA22 (RV64I + Zba + Zbb + Zbs) ALU — plus the RVA23
// Zicond conditional-zero ops (czero.eqz/czero.nez) — with predecoded
// steering. Evolved from the yarvi RV64I ALU; the elegant shared adder/compare
// is preserved and the bit-manipulation extensions are layered on with maximal
// unit sharing:
//
//   - one adder         : ADD/SUB and the compares (SLT/SLTU, MIN/MAX, a branch's,
//                         which the decoder gives ALU_SUB): op1 + (op2 ^ sub) + sub
//   - one shift-adder   : the Zba family, sh{1,2,3}add[.uw] and add.uw (a shift of 0)
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

// Operation selector (ALU_*) lives in alu_ops.vh, shared with the smolrv64_alu
// wrapper. `w` (word) and `uw` (zero-extend op1 from 32 bits) are modifiers.
`include "alu_ops.vh"

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
   // One adder adds, or subtracts for SUB and every compare: the inversion of op2 folds into the
   // carry chain's own LUTs. lt and ltu hold only while it subtracts; eq always.
   wire        is_cmp;
   wire        sub = (op == `ALU_SUB) | is_cmp;
   wire [XLEN:0] as = {1'b0, op1} + {1'b0, op2 ^ {XLEN{sub}}} + {{XLEN{1'b0}}, sub};
   assign      sum = as[MSB:0];
   assign      eq  = op1 == op2;
   assign      ltu = ~as[XLEN];
   assign      lt  = (op1[MSB] == op2[MSB]) ? as[MSB] : op1[MSB];

   // -------- arithmetic --------
   // ADD/SUB feed op1/op2 straight into the shared adder, so the common path has no pre-adder
   // mux. The Zba shift-adds and add.uw use the shift-adder, merged in at a shallow final-mux level.
   wire [1:0]    zsh    = (op == `ALU_SH1ADD) ? 2'd1 :
                          (op == `ALU_SH2ADD) ? 2'd2 :
                          (op == `ALU_SH3ADD) ? 2'd3 : 2'd0;
   wire [MSB:0]  op1_uw  = {{(XLEN-32){1'b0}}, op1[31:0]};
   wire [MSB:0]  zba_sum = ((uw ? op1_uw : op1) << zsh) + op2;   // sh{1,2,3}add[.uw], add.uw

   // -------- one rotator: SLL/SRL/SRA/ROL/ROR/BEXT/slli.uw --------
   // Every shift is a right rotation and a mask: x >> s keeps the rotation's bits i < N - s, x << s
   // (a rotation right by -s) its bits i >= s, a rotation keeps all; the others are the fill, the
   // sign for SRA and 0 otherwise. A word op rotates its 32 bits duplicated into both halves, so the
   // low half is the 32-bit rotation for any s < 32, and N is 32; the result's sign extension is
   // the final *W step below.
   wire is_left = (op == `ALU_SLL) || (op == `ALU_ROL);
   wire is_rot  = (op == `ALU_ROL) || (op == `ALU_ROR);
   wire is_sra  = (op == `ALU_SRA);

   wire [31:0]     o32   = op1[31:0];
   wire [SHW-1:0]  s     = w ? {1'b0, op2[4:0]} : op2[SHW-1:0];
   wire [SHW-1:0]  rs    = is_left ? -s : s;                       // the right rotation
   // the operand: slli.uw's zero-extended word, a word op's duplicated word, or op1
   wire [XLEN-1:0] r_in  = w ? {o32, o32} : (uw && op == `ALU_SLL) ? {{(XLEN-32){1'b0}}, o32} : op1;
   reg  [XLEN-1:0] rot;                                            // r_in rotated right by rs
   integer         rk;
   always @(*) begin
      rot = r_in;
      for (rk = 0; rk < SHW; rk = rk + 1)
         if (rs[rk]) rot = (rot >> (1 << rk)) | (rot << (XLEN - (1 << rk)));
   end
   wire            fill  = is_sra & (w ? o32[31] : op1[MSB]);
   reg  [XLEN-1:0] shift_res;
   integer         bi;
   always @(*)
      for (bi = 0; bi < XLEN; bi = bi + 1)
         shift_res[bi] = (is_rot | (is_left ? (bi >= s) : (bi + s < (w ? 32 : XLEN)))) ? rot[bi] : fill;

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
   // one leading-zero count: trailing zeros are the leading zeros of the reverse
   wire [XLEN-1:0] op1_rev = w ? {{(XLEN-32){1'b0}}, brev32(o32)} : brevx(op1);
   wire [6:0] lz_n = count_lz((op == `ALU_CTZ) ? op1_rev : op1, w);
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
        `ALU_CLZ,
        `ALU_CTZ:   r_ext = {{(XLEN-7){1'b0}}, lz_n};
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


   // -------- balanced final select --------
   // The two slowest data sources — the carry-chain adder (ADD/SUB) and the
   // funnel shifter — get the shallowest mux paths (one 2:1 each); the faster
   // logic/compare/extend/Zba groups sit deeper in the tree.
   wire is_addsub = (op == `ALU_ADD && !uw) || (op == `ALU_SUB);  // add.uw -> the shift-adder
   wire is_shift  = (op == `ALU_SLL) || (op == `ALU_SRL) || (op == `ALU_SRA) ||
                    (op == `ALU_ROL) || (op == `ALU_ROR);
   assign is_cmp  = (op == `ALU_SLT)|| (op == `ALU_SLTU)||
                    (op == `ALU_MIN)|| (op == `ALU_MINU)||
                    (op == `ALU_MAX)|| (op == `ALU_MAXU);
   wire is_bit    = (op == `ALU_AND)|| (op == `ALU_OR) || (op == `ALU_XOR) ||
                    (op == `ALU_ANDN)||(op == `ALU_ORN)||(op == `ALU_XNOR)||
                    (op == `ALU_BCLR)||(op == `ALU_BSET)||(op == `ALU_BINV);
   wire is_zba    = (op == `ALU_SH1ADD)||(op == `ALU_SH2ADD)||(op == `ALU_SH3ADD)||
                    (op == `ALU_ADD && uw);   // add.uw rides the r_arith group

   wire [MSB:0] r_g0 = is_cmp  ? r_cmp   : r_ext;     // fast groups, deepest
   wire [MSB:0] r_g1 = is_bit  ? r_bit   : r_g0;
   wire [MSB:0] r_g2 = is_zba  ? zba_sum : r_g1;
   wire [MSB:0] r_g3 = is_shift ? shift_res : r_g2;   // shifter: one 2:1
   wire [MSB:0] r_pre = is_addsub ? sum : r_g3;       // adder: one 2:1

   always @(*) begin
      result = r_pre;
      // *W instructions: sign-extend the low 32 bits of the result.
      if (XLEN != 32 && w)
         result = {{XLEN/2{result[XLEN/2-1]}}, result[XLEN/2-1:0]};
   end
endmodule

`default_nettype wire
