// va_codec.vh -- virtual-address width compression (Sv39).
//
// A 64-bit Sv39 VA carries only 39 meaningful bits [38:0]; canonical form
// requires bits [63:39] == sign-extension of bit 38.  Internal carriers that
// are canonical-by-invariant (an instruction PC that already passed the fetch
// MMU check, any post-translation address) need only 39 bits -- reconstruct by
// sign-extending bit 38.  Carriers that can legitimately hold a NON-canonical
// value (a not-yet-checked branch/jump target, fetch pc_q before the iMMU, and
// the fault sinks epc/tval) need 40 bits: the low 39 verbatim plus one flag bit
// that reconstructs to *a* non-canonical address (not necessarily the exact one
// -- WARL permits this, and it is sufficient to fault identically).
//
//   PACK40(va)  : 64 -> 40   flag = canonical ? va[38] : ~va[38]; lo = va[38:0]
//   UNPACK40(v) : 40 -> 64   [63:39] = {25{flag}};                [38:0] = lo
//     canonical va     -> flag = va[38]  -> top == original, EXACT round-trip
//     non-canonical va -> flag = ~va[38] -> top != bit38 -> stays non-canonical
//   PACK39/UNPACK39    : the canonical-only pair (no flag; plain sign-extend)
//
// NONCANON compares all of [63:39] against sign(bit38) -- the gap-free form, NOT
// [62:39] vs [63] (which misclassifies all-equal-but-!=bit38 as canonical).
//
// Macros (not functions) so the same definitions are reusable across every
// module in one compilation without the include-guard / function-duplication
// trap.  Arguments are pure wires, so the repeated textual evaluation in
// PACK40 is side-effect-free.

`ifndef VA_CODEC_VH
`define VA_CODEC_VH

// NOTE: arguments must be a bare name (identifier / hierarchical / array
// element) -- Verilog bit-select cannot apply to a parenthesized expression,
// so the operands are intentionally un-parenthesized.
`define VA_NONCANON(va) (va[63:39] != {25{va[38]}})
`define VA_PACK39(va)   va[38:0]
`define VA_UNPACK39(v)  {{25{v[38]}}, v}
`define VA_PACK40(va)   {va[38] ^ `VA_NONCANON(va), va[38:0]}
`define VA_UNPACK40(v)  {{25{v[39]}}, v[38:0]}

`endif
