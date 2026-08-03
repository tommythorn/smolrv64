# SmolRV64

SmolRV64 is a 64-bit RVA22-class RISC-V application SoC, written from
scratch in Verilog, that boots stock Ubuntu — on an FPGA and in full-RTL
simulation.  The core started out as a small sequential implementation,
thus the name.  The name is now ironic: the current core is a clustered
out-of-order design, and the SoC around it carries the full weight of a
modern OS — Sv39 virtual memory, an FPU, non-coherent virtio DMA with
cache-maintenance operations, and a complete interrupt stack.

## Highlights

- **Runs real software.** Boots unmodified Ubuntu 25.04 (OpenSBI +
  mainline kernels) to a multi-user login on the FPGA target, from a
  virtio-blk root filesystem on SD card, with working virtio-net
  (DHCP/ping over RGMII).  The same images boot in RTL simulation.
- **Out-of-order core.** A clustered ("sharded") OoO machine:
  configurable 1–4 wide (default 2), register renaming with checkpoint
  repair (CPR), scoreboard scheduling, a two-deep out-of-order load
  pipe with store-buffer forwarding, and a BTB + bimodal + RAS branch
  predictor feeding a streaming instruction-fetch client.
- **A real memory system.** Skewed 2-way VHPR L1 caches (128 KiB each,
  a design of this project — see appendix); the write-back D$ is
  pipelined (one op/cycle, 2-cycle hit) with a miss-status register
  for hit-under-miss and a write-back buffer.  Page-table walks go
  *through* the D$, making it the coherency point: `sfence.vma` never
  flushes a cache.
- **Verification as a first-class artifact.** Every committed
  instruction is compared in lockstep against an independent ISA-level
  reference model ([Simmerv](https://github.com/tommythorn/simmerv)),
  through multi-billion-cycle Linux boots and benchmark runs.  Bugs
  get root-caused, not worked around; the debugging instrumentation
  ships in the tree.

## Architecture

**ISA.** `rv64imafdc` + Zicsr, Zifencei, Zicntr, Zihpm, Zba/Zbb/Zbs,
Zicond, Zicbom, Zicboz, Zicbop, Svpbmt — RVA22 with software A/D page
management.  F/D floating point via
[CV-FPU](https://github.com/openhwgroup/cvfpu), with commit-gated
`mstatus.FS` dirty tracking (architectural state never goes dirty on a
squashed path).

**Core** (`src/`).  Fetch translates through a dedicated iMMU and
aligns variable-length RVC bundles; decode/rename dispatches into
per-shard schedulers; execution clusters own their register-file
shards with a broadcast bypass; commit is in-order over coarse CPR
checkpoints (rollback restores rename maps from per-checkpoint
snapshots).  Interrupts are delivered by injecting a pseudo-op into
the normal trap path — one mechanism for traps, faults, and IRQs.
Device loads execute exactly once, non-speculatively, in program
order; replayed side effects are structurally impossible.

**Memory system.**  Three independent Sv39 MMUs (fetch / load / store)
with private TLBs and hardware page-table walkers arbitrating for the
D$ read port.  Svpbmt non-cacheable mappings and Zicbom/Zicboz
cache-maintenance ops support the kernel's non-coherent virtio DMA
rings.  Misaligned loads and stores within a page are handled in
hardware, in both caches — uncommon anywhere, rare in a softcore.

**SoC.**  CLINT, PLIC, NS16550A UART, virtio-blk backed by a real SD
card (SPI host), and virtio-net on RGMII.  One 64-byte-line arbiter
merges all instruction, data, walker, and DMA traffic onto DDR4.

## Verification and debugging

The bar for any change: it boots Linux under the oracle.

- **Lockstep cosimulation** — the RTL runs with a DPI bridge that
  steps Simmerv one instruction per retirement and compares every
  architectural effect (PC, register writes, traps, CSRs).  A
  divergence aborts with the register state and a 65k-entry
  retirement history.  Ubuntu boots have run past four billion
  retirements in lockstep.  ([docs/cosim.md](docs/cosim.md))
- **Full-system RTL simulation** — the identical SoC boots the
  identical disk image under Verilator with a file-backed SD model,
  real interrupt latencies and non-coherent DMA, deep into systemd.
- **riscv-tests** (215 programs across the rv64 suites) as the fast
  gate — one parallel Verilator harness runs them all in seconds.
- **Performance observability** — pipeline event tracing into a Rust
  analysis tool (`src/perftool`) that produces per-instruction
  waterfalls, stall attribution, and cycle accounting; hardware
  performance counters (Zicntr/Zihpm) on the FPGA.
- The plan/findings documents in [docs/](docs/) record how each
  subsystem was designed, measured, and debugged — including the
  wrong turns.

## Performance

Performance work is ongoing and measured, not guessed: every change is
justified by cycle-accounting data from the trace tooling, and the
docs record the numbers.  Recent examples: pipelining the D$ raised
boot IPC 13%; a streaming fetch client raised it another 14%.
Current cycle accounting puts the next wins in branch-handling and
checkpoint capacity — both in progress.  The original sequential core
ran at ~9.4 CPI; the OoO core is at ~0.5 IPC (2-wide) and climbing,
with the roadmap below aimed at the remaining stalls.

## FPGA target

The supported board is the RK-XCKU5P-F (AMD Kintex UltraScale+,
DDR4).  The build is fully scripted (`make bit` in
`platforms/rk-xcku5p-f-v1.2/`) and closes timing; the core runs at
66.7 MHz with the DDR4 controller at 333 MHz.

Board notes:
- The UART is hardwired to 3,000,000 baud: `make connect`, or
  `screen /dev/ttyUSB1 3000000`.
- `make load WORKLOAD=...` builds a payload, rebuilds the bitstream,
  and programs the board over JTAG.

## Repository layout

| Path | Contents |
|---|---|
| `src/` | The core, SoC, devices, testbenches, cosim harness, run scripts, and perftool (files named `*_probe.v` are the module-level Fmax probing harnesses) |
| `platforms/rk-xcku5p-f-v1.2/` | FPGA build: sources, constraints, scripted Vivado flow |
| `workloads/` | Linux/Ubuntu/Geekbench images, device trees, boot scripts |
| `docs/` | Design plans, measurements, and debugging findings |

## Roadmap

- Branch prediction phase 2 and confidence-gated checkpoint formation
  (letting predicted branches ride inside checkpoints — the largest
  measured stall source in benchmark code)
- Larger effective out-of-order window via coarser checkpoints
- RVA23 (vector is the main gap)

## Novelties

- **VHPR caches** — a cache organization devised for this project
  (below).
- **First-class misalignment** in both L1 caches.
- **Interrupts as injected pseudo-ops** — external interrupts reuse
  the precise-exception machinery instead of adding a parallel
  delivery path.
- **Walker-through-cache coherency** — page-table walks read the D$
  itself, so page-table stores are visible without any flush
  protocol.

## Appendix: Skewed VHPR (Virtually Hashed, Physically Resolved)

The VHPR cache indexes with a hash of the virtual address, giving a
TLB-free fast path, but each line also carries a physical tag and the
cache guarantees that no two aliases of the same physical line are
ever resident at once.  From the outside it behaves exactly like a
standard VIPT cache.  Each way uses a different hash, which gives
alias resiliency close to a conventional 4-way design from 2-way
hardware.
