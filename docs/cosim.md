# Cosimulation against simmerv

SmolRV64 can be run under Verilator in lockstep against
[simmerv](https://github.com/tommythorn/simmerv), an RV64GC ISA-level
reference model.  For every instruction the DUT retires, a DPI
callback ships the architectural effect (PC, next PC, rd, writeback
value, privilege, trap state, mtime) to simmerv and compares it
against simmerv's own retirement.  The first divergence aborts with a
ring buffer of the last 32 retirements from both sides.

This is the Phase 1 harness: correctness only, no MMIO cosim, no
multi-hart.  The RTL change is guarded by `VERILATOR_COSIM`, so
non-cosim builds are unaffected.

## Prerequisites

- Verilator (any recent release).
- Rust toolchain (`cargo`, stable).
- A clone of simmerv at `$HOME/github/simmerv` (override with the
  `SIMMERV_DIR` make variable if yours lives elsewhere).

## Building

From `src/`:

    make smolrv64-linux-cosim

This runs `cargo build --release -p simmerv-cosim` in `$SIMMERV_DIR`
to produce `libsimmerv_cosim.a`, then builds the Verilator model and
links the static library in.  The resulting binary is self-contained —
no `LD_LIBRARY_PATH` dance.

To point at a different simmerv checkout:

    make SIMMERV_DIR=/path/to/simmerv smolrv64-linux-cosim

## Running a workload

The cosim binary accepts the same plusargs as the iverilog-built
simulators, plus `+rf=<path>` for seeding the integer register file:

    ./smolrv64-linux-cosim \
        +even=<path>/mem.even \
        +odd=<path>/mem.odd \
        +rf=<path>/rf.hex

`mem.even` / `mem.odd` may contain `@ADDR` directives — the cosim
loader mirrors Verilog's `$readmemh` behavior and loads sparse
layouts correctly (the Linux image places the DTB at offset
`0x01000000`, i.e. physical `0x81000000`).

`rf.hex` is 32 lines of hex, one per integer register x0..x31.  For
workloads that rely on boot arguments (OpenSBI expects a1 = DTB
pointer), cosim uses `simmerv_write_register` to seed simmerv's
registers to match.

### Linux workload

The Linux workload (OpenSBI + vmlinux + DTB) has a prebuilt recipe:

    cd workloads/linux
    make mem.even mem.odd        # regenerates from ../../linux/fw_payload.bin + dts.dtb
    ../../src/smolrv64-linux-cosim \
        +even=$PWD/mem.even \
        +odd=$PWD/mem.odd \
        +rf=$PWD/rf.hex 2>cosim.log

Always wrap long runs in a timeout — the simulator has `-DNO_TIMEOUT`
and will otherwise hang forever on a misconfigured workload:

    timeout 600 ../../src/smolrv64-linux-cosim +even=... 2>cosim.log

### Other workloads

Any workload that builds `mem.even`/`mem.odd` (and optionally
`rf.hex`) with the repo's `mkhex.py` / `evenodd.py` tooling works the
same way.  The monitor and sieve workloads under `workloads/` are the
simplest starting points.

## Expected output

Progress lines on stderr every million retirements:

    cosim: 1 retirements ok (pc=0000000080000000)
    cosim: 1000000 retirements ok (pc=ffffffff80023abc)
    ...

On successful exit (workload reaches `$finish`):

    cosim: completed N retirements without divergence

On divergence, the harness dumps the ring buffer of the last 32
retirements in DUT/REF pairs and aborts:

    *** cosim MISMATCH at retire #N ***
    Recent history (DUT vs REF):
      DUT seq=... pc=... npc=... insn=... ...
      REF seq=... pc=... npc=... ...
      ...
    Diverging retire:
      DUT ...
      REF ...

Fields that must match on every retirement: `pc`, `next_pc`, `insn`,
`prv`, `trapped`, `rd_kind`, `rd_idx`, and `rd_val` (when the
instruction writes rd).  On traps, `trap_cause` and `trap_tval` must
also match.

## Known Phase 1 limitations

- **No F/D yet in smolrv64.**  The reference model advertises F and D
  via misa.  Any workload that reaches an FP instruction will diverge:
  the DUT traps illegal-instruction; simmerv executes it.  The current
  Linux image hits this at roughly 705,000 retirements during
  OpenSBI's FP probe.  Until smolrv64 gains F/D, cosim coverage of
  Linux boot stops there — this is expected, not a bug.
- **fflags is captured but not compared.**  Reserved for when FP
  lands.
- **mtime skew.**  smolrv64 reads `clint_mtime` one cycle before
  retirement (in `S_HANDLE_CSR`); the DPI call adjusts by passing
  `clint_mtime - 1`.  If you touch the CSR pipeline, make sure the
  value reported to cosim still matches what the instruction actually
  observed.
- **mtime is frozen in simmerv.**  `simmerv_create` calls
  `freeze_clint(0)` so the reference only advances mtime when the DUT
  tells it to — otherwise the wall-clock CLINT would race the DUT.

## Layout

- RTL hook: `src/smolrv64.v` — `` `ifdef VERILATOR_COSIM `` blocks
  call `cosim_retire` from `S_FETCH1` (normal retire) and at the end
  of `S_EXCEPTION` (trap retire).
- C++ harness: `src/sim_main.cpp` — image loader (`load_sparse`),
  register seeder (`load_rf`), DPI callback (`cosim_retire`),
  ring-buffer mismatch reporter.
- C ABI: `$SIMMERV_DIR/cosim/simmerv_cosim.h`.
- Rust shim: `$SIMMERV_DIR/cosim/src/lib.rs`.
