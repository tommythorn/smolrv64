// Bench randomness: xorshift64* on a module-level state, one generator per process that draws.
//
//   `TB_RAND(rnd, rs)          declares the state rs and the function rnd(0), 32 random bits
//   rs = `TB_SEED(seed);       seeds it (any integer, 0 included)
//
// Benches draw from this, never from $random(seed) or $urandom(seed): under Verilator the first
// walks a degenerate sequence -- shifted all-ones words, successive seeds collapsing together --
// and the second's first draw after a reseed is one too, so `$urandom(s) % 8` with s stepping is
// a constant. The state lives in the module because Verilator does not copy an inout argument of
// a function back. src/lint.sh rejects a seeded $random or $urandom in any bench.
`ifndef TB_RAND_VH
`define TB_RAND_VH
`define TB_SEED(s) (64'h9E37_79B9_7F4A_7C15 ^ 64'($unsigned(s)))
`define TB_RAND(fn, st) \
   reg [63:0] st = `TB_SEED(1); \
   function automatic [31:0] fn(input integer unused); \
      st = st ^ (st >> 12);  st = st ^ (st << 25);  st = st ^ (st >> 27); \
      fn = 32'((st * 64'h2545_F491_4F6C_DD1D) >> 32); \
   endfunction
`endif
