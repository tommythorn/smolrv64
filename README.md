# SmolRV64

SmolRV64 is a single `always @(posedge clock)`
RV64GC (IMAFDC) implementation which aspires to run Ubuntu.

The first goal was functionality, so the RTL still has a simulator-like
shape in many places.  The current work is shifting toward performance
without losing that debuggability.

The primary FPGA target is the RK-XCKU5P-F board with DDR4, SD card, and
a 3 Mbaud serial console.

# Status

RV64GC (IMAFDC) with Sv39 virtual memory and Ssvnapot is implemented.
Floating point (F/D) is implemented via the CV-FPU, which is now always
built in.
The core has a direct-mapped physical write-back cache, a small fetch
buffer, and split direct-mapped TLBs for 4 KiB and 2 MiB pages.

RK-XCKU5P-F FPGA dev boards are directly supported.

Devices:
- UART
- CLINT
- PLIC
- SPI-MMC (SDcard)

Board notes:
- The RK UART is hardwired for 3,000,000 baud.  Use `make connect` in
  `platforms/rk-xcku5p-f-v1.2` or run `screen /dev/ttyUSB1 3000000`.
- The Ubuntu device tree intentionally advertises a faster CLINT
  timebase than the hardware would otherwise imply, so Linux does not
  give up on the machine for being too slow during boot.

## Performance

Performance is now an active area of work.  The RK target runs the core
from the DDR4 UI clock at about 333 MHz, but CPI is still dominated by
frontend, translation, cache, and DDR4 latency.  Recent work added cache,
fetch, TLB, and HPM/stat counters so Linux can expose where cycles go.
The current RK implementation closes timing with a small margin.

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
- [x] UART — Linux console (NS16550A model)
- [x] DDR4 support on RK-XCKU5P-F
- [x] Cosim harness against Simmerv for lockstep debugging
      ([docs/cosim.md](docs/cosim.md) — Phase 1 landed)
- [x] Bring up OpenSBI (great incremental test for CSR/privilege bugs)
- [x] Boot Linux with serial console M1!!!

## M2: Run Ubuntu

- [x] SDcard interface for permanent storage
- [x] Full Compliant Floating point (F+D)
- [x] Split direct-mapped TLBs for 4 KiB and 2 MiB pages
- [x] Direct-mapped physical write-back cache (64-byte lines)
- [x] Close timing with cache, TLB, fetch buffer, and CVFPU enabled

## Beyond: Making it fast

- [ ] More advanced TLBs after the direct-mapped design is characterized
- [ ] Skew-associative virtually indexed frontend/cache experiments
- [ ] Out-of-order (before pipelining for ease of debugging)
- [ ] 4-wide superscalar
- [ ] Branch prediction
- [ ] Pipelining

- [ ] RVA22
- [ ] RVA23 (incl. vector)
