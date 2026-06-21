# SmolRV64

SmolRV64 is a 64-bit RVA22+ compliant RISC-V application-class soft
SoC.  It is regularly tested with Ubuntu and its applications, but
has booted Debian in the past and should be able to run any OS
targeting RVA20+.  The core started out as a small sequential
implementation, thus the name.  The name is now ironic as this is not
a small core.

# Implementation Status

Full RVA22 support + Zicond extension.  The FPU courtesy of CV-FPU.

The core has a 64 KiB skewed 2-way VHPR L1 instruction cache and a
64 KiB skewed 2-way VHPR L1 data cache (coherent), as well as
256 entry 4K TLB and a 64? entry 2M TLB.

RK-XCKU5P-F FPGA dev boards are directly supported.

Devices:
- UART (NS16550 compatible)
- CLINT
- PLIC
- Virtio Blk (backed by SD card)
- Virtio Ethernet

Board notes:
- The RK UART is hardwired for 3,000,000 baud.  Use `make connect` in
  `platforms/rk-xcku5p-f-v1.2` or run `screen /dev/ttyUSB1 3000000`.
- The Ubuntu device tree intentionally advertises a faster CLINT
  timebase than the hardware would otherwise imply, so Linux does not
  give up on the machine for being too slow during boot.

## Performance

Performance is now an active area of work.  The RK target runs the
core from the DDR4 UI clock at about 9.4 CPI @ 166 MHz.  The current
RK implementation closes timing with a small margin.

# Milestones

## M1: Boot a minimal Linux

SmolRV64 targets 100% compatibility with
[Simmerv](https://github.com/tommythorn/simmerv) (an ISA-level
reference model which already boots full Ubuntu).  This means we are
reusing the same OpenSBI, device tree, kernel, and file system images.
For simulation, everything is preloaded into memory; for FPGA, a tiny
bootloader will load from flash/UART.

What's needed:
- [x] Sv39 page table walk with software A/D (RVA22)
- [x] Ssvnapot (64 KiB NAPOT pages)
- [x] Cross-page instruction fetch
- [x] CLINT (mtime, mtimecmp, msip) — timer interrupts
- [x] PLIC — external interrupt routing (UART RX at minimum)
- [x] UART — Linux console (NS16550A model)
- [x] DDR4 support on RK-XCKU5P-F
- [x] Cosim harness against Simmerv for lockstep debugging
      ([docs/cosim.md](docs/cosim.md))
- [x] Bring up OpenSBI (great incremental test for CSR/privilege bugs)
- [x] Boot Linux with serial console

## M2: Run Ubuntu

- [x] SDcard interface for permanent storage
- [x] Full Compliant Floating point (F+D)
- [x] Split direct-mapped TLBs for 4 KiB and 2 MiB pages
- [x] Direct-mapped physical write-back cache (64-byte lines)
- [x] Close timing with cache, TLB, fetch buffer, and CVFPU enabled

## Beyond: Making it fast

- [ ] More advanced TLBs after the direct-mapped design is characterized
- [x] Skew-associative virtually indexed frontend/cache (VHPR)
- [ ] Out-of-order (before pipelining for ease of debugging)
- [ ] 4-wide superscalar
- [ ] Branch prediction
- [ ] Pipelining

These last four are being pursued together as a sharded (clustered)
out-of-order design that replaces the inner core and frontend while
reusing the existing caches, TLBs, devices, and SoC infrastructure.
See [docs/sharded-ooo-plan.md](docs/sharded-ooo-plan.md).

- [x] RVA22 + Zicond (basically RVA23 except for Vector)
- [ ] RVA23 (incl. vector)

# Novelties
- Both I and D caches have 1st class misalignment support (uncommon,
  and unheard of in a softcore)
- The VHPR cache is a new invention



# Appendix: Skewed VHPR (Virtually Hashed Physically Resolved cache)

The VHPR cache uses a hashed virtual index, giving a fast access
without needing TLB.  To avoid the problems usually associated with
VIVT caches, each line have a physical tag as well and the cache
guarantees that no two aliases can be resident at once.  Thus, from
the outside the cache looks like a standard VIPT cache.  We use
different hashes for each way which gives excellent alias resiliency,
almost as good as a traditional 4-way cache.
