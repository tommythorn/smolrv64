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

194 riscv-tests pass (106 physical-mode, 66 virtual-mode, 22 other).
RV64IMAC with Sv39 virtual memory and Ssvnapot is implemented.
Two FPGA dev boards are directly supported: ULX3S and RX-XCKU5P-F.

## Performance

Performance is not [yet] a priority, but IPC is about 0.22 at 25 MHz
(ECP5) and 200 MHz (XCKU5P).

# Milestones

## M1: Boot a minimal Linux (buildroot, CONFIG_FPU=n)

SmolRV64 targets 100% compatibility with [Simmerv](https://github.com/tommythorn/simmerv)
(an ISA-level reference model which already boots full Ubuntu).
This means we can reuse the same OpenSBI, device tree, kernel, and
initrd images.  For simulation, everything is preloaded into memory;
for FPGA, a tiny bootloader will load from flash/UART.

What's needed:
- [x] Sv39 page table walk with software A/D (RVA22)
- [x] Ssvnapot (64 KiB NAPOT pages)
- [x] Cross-page instruction fetch
- [ ] CLINT (mtime, mtimecmp, msip) — timer interrupts
- [ ] PLIC — external interrupt routing (UART RX at minimum)
- [ ] UART — Linux console (extend existing uart5.v or replace)
- [ ] Bring up OpenSBI (great incremental test for CSR/privilege bugs)
- [ ] Boot Linux with serial console

Not needed for M1: FPU (kernel can trap-emulate or CONFIG_FPU=n),
TLBs, caches (correctness first, performance later).

## M2: Run Ubuntu

- Floating point (F+D) — required for userspace
- iTLB and dTLB — needed for acceptable performance
- Instruction and data caches
- DDR4 support on RK-XCKU5P-F
- Cosim harness against Simmerv for lockstep debugging
  ([docs/cosim.md](docs/cosim.md) — Phase 1 landed)

## Beyond: Make it fast

- Pipeline
- Branch prediction
- Out-of-order, superscalar
- Caches (two-way virtually tagged skewed, maybe with SIEVE eviction?)
- TLB (level 2 as level 1 is embedded in the cache), possibly with
  some Cuckoo hashing scheme
