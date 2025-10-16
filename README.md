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

Milestone 1: RV64IMAC is fully implemented (and modulo bugs), except
for virtual memory support.

System support (for example external mmio memory bus) is high priority
and will come next, followed by Virtual memory and Floating point
support.

The implementation is accompanied by the riscv-test test suite
(launched with `make`).  As features are implemented, more tests are
migrated from `unsupported` to `passes`.

Currently two dev boards are directly supported: ULX3S and
RX-XCKU5P-F.

## Performance

Performance is not [yet] a priority, but IPC is about 0.22 at 25 MHz
(ECP5) and 200 MHz (XCKU5P).

# Milestone 2

- DONE: MMIO bus etc
- DDR4 support on RK-XCKU5P-F

# Milestone 3 (Ubuntu)

- MAYBE: Cosim against Spike, Dromajo, or Simmerv
- LIKELY: Run Tenstorrent's test suite
- Full system support, including virtual memory
- Devices: CLINT, PLIC, ...
- Floating point support (F and G) support either via bbl or
  implemented

# Beyond (Sharpening the saw)

- Caches (two-way virtually tagged skewed, maybe with SIEVE eviction?)
- TLB (level 2 as level 1 is embedded in the cache), possibly with
  some Cookoo hashing scheme
- And finally: make it go fast (pipelining, branch prediction, and
  much more)
