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

209 riscv-tests pass. RV64IMAC with Sv39 virtual memory and Ssvnapot
is implemented.  Most basic parts of FP (FD extensions) is there, but
no math yet.

RX-XCKU5P-F FPGA dev boards is directly supported.

Devices:
- UART
- CLINT
- PLIC
- SPI-MMC (SDcard)

## Performance

It's insanely slow.  Performance is not yet a priority, however the RK
target is using a 333 MHz clock and CPI is significant due to DDR4.

# Milestones

## M1: Boot a minimal Linux

SmolRV64 targets 100% compatibility with [Simmerv](https://github.com/tommythorn/simmerv)
(an ISA-level reference model which already boots full Ubuntu).
This means we can reuse the same OpenSBI, device tree, kernel, and
initrd images.  For simulation, everything is preloaded into memory;
for FPGA, a tiny bootloader will load from flash/UART.

What's needed:
- [x] Sv39 page table walk with software A/D (RVA22)
- [x] Ssvnapot (64 KiB NAPOT pages)
- [x] Cross-page instruction fetch
- [x] CLINT (mtime, mtimecmp, msip) — timer interrupts
- [x] PLIC — external interrupt routing (UART RX at minimum)
- [x] UART — Linux console (extend existing uart5.v or replace)
- [x] DDR4 support on RK-XCKU5P-F
- [x] Cosim harness against Simmerv for lockstep debugging
      ([docs/cosim.md](docs/cosim.md) — Phase 1 landed)
- [x] Bring up OpenSBI (great incremental test for CSR/privilege bugs)
- [x] Boot Linux with serial console M1!!!

## M2: Run Ubuntu

- [x] SDcard interface for permanent storage
- [ ] Full Compliant Floating point (F+D)
- [ ] Simple directly mapped TLB
- [ ] Simple directly mapped physical cache (64-byte lines)

## Beyond: Making it fast

Planning for this is still in the early stages

- [ ] TBD: more advanced TLBs using some Cuckoo hashing scheme
- [ ] TBD: more advanced skew-associative virtually tagged Instruction
      and data caches
- [ ] Out-of-order (before pipelining for ease of debugging)
- [ ] 4-wide superscalar
- [ ] Branch prediction
- [ ] Pipelining

- [ ] RVA22
- [ ] RVA23 (incl. vector)
