# SmolRV64

SmolRV64 is a single-file, single `always @(posedge clock)`
micro-stepped RV64IMAC implementation which aspires to run Ubuntu.

The goal to reach functionality as quickly as possible, thus, the RTL
will prioritize simplicity over efficiency and will resemble a
software simulator.  That's on purpose!  Performance is *not* the goal
(initially).

Once the goal of fully running Ubuntu is attained we will begin work
on getting it fast (and tighten up the RTL a lot).

# Status

RV64IMAC is fully implemented (and modulo bugs), except for some
system features, and all debug. The implementation is accompanied by
the riscv-test test suite (launched with `make`).  As features are
implemented, more tests are migrated from `fails` or `unsupported` to
`passes`.

Currently two dev boards are directly supported: ULX3S and RX-XCKU5P-F.

## Performace

Performance is not a priority, but IPC is about 0.22 at 25 MHz (ECP5)
and 200 MHz (XCKU5P).

# Milestone 1 (Coming soon)

- More bug fixes
- UART on memory address 'h10000000 instead of the CSR666 hack
- Complete CSR support (sans virtual memory)
- Interrupts

# Milestone 2

- DDR4 support on RK-XCKU5P-F
- Cosim against Dromajo or Simmerv

# Milestone 3 (Ubuntu)

- Full system support, including virtual memory
- Devices: CLINT, PLIC, ...
- Floating point support (F and G) support either via bbl or
  implemented
- Caches (two-way virtually tagged skewed, maybe with SIEVE eviction?)
- TLB (level 2 as level 1 is embedded in the cache), possibly with
  some Cookoo hashing scheme
