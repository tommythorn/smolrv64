# Plan: checkpoint and restore for the lockstep cosim

Status: **held** (2026-09-04). High payoff, high effort; the cheaper items from the same
retrospective landed first (measured DDR model, fail-fast board gate, the random queue
bench, the glibc initramfs, the long-guest runner). This document is written so that it can
be executed without the context of that session.

## 1. Why

The Geekbench boot under the cosim (`ooo2/run-ooo2-cosim-gb5.sh`) found a six-day-old
store-queue defect at retire 123,081,278, cycle ~447 M -- 25 minutes into the run at the
cosim's ~4.6 ms of guest time per second. The first attempt was capped at 400 M cycles and
had to be restarted from zero. Every future run of that gate, and every debug rerun of a
divergence deep in a boot, pays the whole boot again. A checkpoint taken once at the end of
the kernel boot (or a few instructions before a known divergence) turns a 25-minute rerun
into seconds, and makes "run the long guest to userspace" a per-batch gate rather than a
special occasion.

## 2. What exists

Two halves must be saved and restored together, plus the bridge between them.

**simmerv (the reference model).** `~/simmerv/src/lib.rs` already has a snapshot facility:

- `Emulator::snapshot_bytes(&self) -> Vec<u8>` and `write_snapshot(path)` -- CPU state
  (`cpu.write_state`, `~/simmerv/src/cpu.rs:2246`) plus RAM, brotli-compressed, behind a
  magic prefix (`SNAPSHOT_MAGIC`).
- `Emulator::load_snapshot(&mut self, data)` (`lib.rs:537`) -- decompresses, rebuilds the
  device list, hands the UART and net backends to the freshly constructed devices.
- `run_with_periodic_snapshots` exists for the standalone CLI.

Restore has not been exercised much (Tommy, 2026-09-04), and nothing in the cosim path uses
either. The cosim links a separate crate, `~/simmerv/cosim` (`libsimmerv_cosim.a`, header
`cosim/simmerv_cosim.h`), whose C API (`cosim/src/lib.rs`: `simmerv_create`,
`simmerv_write_memory`, `simmerv_read_register`, `simmerv_set_pc`, `simmerv_set_mtime`,
...) has no snapshot entry points.

**The DUT (Verilator).** `ooo2/run-ooo2-cosim-linux.sh` builds `tb_ooo2_linux.v` with
`--binary --timing`. Verilator can serialise the whole model with `--savable`
(`VerilatedSave`/`VerilatedRestore`, `verilated_save.h`), including every Verilog variable,
the big memory arrays (`lram`, the caches' BRAM models), `$random` per-variable seeds, the
simulation time and the context. It does NOT save anything on the C++ side of a DPI
boundary.

**The bridge.** `src/probe_cosim.cpp` holds the lockstep state in C++ globals: the simmerv
context `g_ctx`, the retire sequence counters, the interrupt-injection state (the model's
pending interrupt that the DUT is told to take, the `inj` counter in the progress line), the
device-write forwarding queue (every device AXI write beat is forwarded to simmerv "in DPI
order", `probe_cosim.cpp:299`), the MMIO compare state, and the mismatch history ring. The
DUT calls `probe_retire()` (`ooo2/ooo2_core.v:2137`) once per committed instruction; the
bridge steps simmerv and compares.

## 3. Design

A checkpoint is taken at a **retire boundary**: the cycle in which the DUT's retire count
reaches `R` and `probe_retire()` has returned, so simmerv has executed exactly the same `R`
instructions and the bridge has no comparison in progress. Three files, one name:

    <base>.dut    VerilatedSave of the whole Verilator model, at the end of that cycle
    <base>.ref    simmerv snapshot_bytes() (existing format)
    <base>.brg    the bridge's globals, a flat versioned struct

Restore is the mirror: build the same binary, run with `+restore=<base>`; the testbench's
initial block skips the firmware/DTB/initrd loads (memory comes back from the two images),
`VerilatedRestore` reloads the model, the bridge recreates `g_ctx` from `<base>.ref` and
its globals from `<base>.brg`, and the main loop continues with `c`, `$time` and every
counter where they were. A run that was restored must produce, from that point on, the
same retire stream as a run that never stopped.

Alignment rule: the DUT's cycle count is the clock of the whole system in cosim mode --
simmerv's `mtime` is set from the DUT each retire (`simmerv_set_mtime`) -- so the retire
boundary is a complete state for both sides, and the DDR model's LFSR and `ddr_seed` are
Verilog state and travel in `.dut`.

## 4. Steps, in order, each with its own test

Do them one at a time; each step has a check that does not need the next.

### Step 1 -- simmerv restore is bit-exact (Rust only, ~half a day)

1. In `~/simmerv`, add a CLI test: run tiny128 (`workloads/tiny128`, the `ref` target's
   command line in `workloads/tiny128/Makefile:37`) for N instructions, `write_snapshot`;
   in a fresh process, `load_snapshot` and run to M; separately run straight to M. Compare a
   hash of the retire stream (pc, instruction, rd, value) between the two. Add the hash to
   the CLI if it has none (a `--trace-hash` flag printing a running FNV-1a of each retire).
2. Expect failures the first time: device state that `write_state` does not cover (PLIC
   pending bits, CLINT `mtimecmp`, UART FIFO contents, the virtio queues if any), and
   anything in `Emulator` outside `cpu` (the `snapshot_flag`, HTIF). Fix `write_state`/
   `read_state` until the hashes match at three different N.
3. Acceptance: three (N, M) pairs on tiny128 with identical hashes; one pair on the
   Geekbench image (`workloads/gb5/Makefile`'s `ref` command) with N past the kernel boot.

### Step 2 -- snapshot entry points in the cosim C API (Rust, ~2 hours)

In `~/simmerv/cosim/src/lib.rs`, add:

    int  simmerv_snapshot(const SimmervCtx*, uint8_t** out, size_t* len);   // malloc'd, caller frees
    void simmerv_snapshot_free(uint8_t*);
    int  simmerv_restore(SimmervCtx*, const uint8_t* data, size_t len);

wrapping `snapshot_bytes` and `load_snapshot`, and declare them in
`cosim/simmerv_cosim.h`. `simmerv_restore` must leave the context usable for the same calls
the bridge makes afterwards (`simmerv_set_mtime`, the retire step). Test: a C program that
creates a context, runs a few thousand steps, snapshots, restores into a second context,
and compares registers and a memory range.

### Step 3 -- the bridge serialises itself (C++, ~half a day)

In `src/probe_cosim.cpp`:

1. Gather every global into one `struct CosimState` (do this first as a pure refactor; the
   lockstep must still pass tiny128 unchanged, `retires=11,135,530` at 60 M cycles at the
   measured DDR default).
2. Add `cosim_save(const char* base)` and `cosim_restore(const char* base)`: write/read
   `<base>.brg` (a version word, then the struct; no pointers -- `g_ctx` is recreated) and
   call `simmerv_snapshot`/`simmerv_restore` for `<base>.ref`.
3. Export both to Verilog as DPI functions (`import "DPI-C" function int cosim_save(input
   string base)` next to `probe_retire` in `ooo2/tb_ooo2_linux.v`, not in `ooo2_core.v`).

### Step 4 -- the DUT side (Verilog + build flags, ~half a day)

1. Add `--savable` to the verilator command in `ooo2/run-ooo2-cosim-linux.sh` (and
   `src/probe_cosim.cpp` gets `#include "verilated_save.h"`). Rebuild; fix what
   `--savable` rejects (it refuses a few constructs; the common ones are `$fopen` handles
   held in variables and `--timing` interactions -- if `--timing` and `--savable` conflict
   in this Verilator (5.041 here), the main loop in `tb_ooo2_linux.v` is a plain
   `@(negedge clk)` loop and can be driven from C++ instead; check the Verilator manual
   for the version first).
2. In `tb_ooo2_linux.v`: plusargs `+save_at=<retires> +save=<base>` and `+restore=<base>`.
   At `+save_at`, after the retire that reaches it, call a C++ `dut_save(base)` (a DPI
   function that does `VerilatedSave os; os.open(base+".dut"); os << *contextp; os <<
   *topp;`) and `cosim_save(base)`. Verilator's save must be invoked from C++ with access
   to the model pointer, so the save routine lives in the C++ wrapper rather than in the
   Verilog initial block; the simplest form is a `$finish`-free "save requested" flag that
   the C++ main loop polls between `eval()` calls. The generated `--binary` main cannot be
   edited, so this step replaces `--binary` with a small hand-written `main()` in
   `src/probe_cosim_main.cpp` (Verilator's `--exe` mode); the DDR/UART/console code stays
   in Verilog.
3. On `+restore`: the C++ main does `VerilatedRestore` before the first `eval()`, then
   `cosim_restore(base)`; the Verilog initial block sees `+restore` and skips
   `load_bin(fw/dtb/initrd)`.

### Step 5 -- the round trip is bit-exact (test, ~2 hours)

1. Add a running retire hash to `tb_ooo2_linux.v` (FNV-1a over pc, insn, rd, value at each
   retire; printed at the end and at each 1 M-cycle progress line).
2. tiny128: straight run to 60 M cycles; run with `+save_at=3000000 +save=/tmp/ck`; run with
   `+restore=/tmp/ck +cycles=60000000`. The restored run's final retire count and hash must
   equal the straight run's. Repeat with a save inside a DDR fill (pick `+save_at` so the
   progress line shows the D$ mid-miss -- `d_busy` is Verilog state and must come back).
3. The negative control: corrupt one byte of `<base>.brg` and confirm the restored run
   diverges or aborts, so a silent no-op restore cannot pass.

### Step 6 -- use it (~1 hour)

- `ooo2/run-ooo2-cosim-gb5.sh`: take `ck/gb5-postboot` at the first retire in user mode
  (`prv=0` on the progress line, or a fixed retire count ~130 M), store it under
  `/var/tmp/cosim-ck/` (not the repo: hundreds of MB), and default to restoring it when
  present and when the model source hash on the verdict line matches the one recorded next
  to the checkpoint. A checkpoint is valid only for the exact model it was taken with:
  record `model-src=` from the config line in `<base>.meta` and refuse a mismatch.
- The divergence workflow: on a MISMATCH at retire R, rerun with `+save_at=R-2000` once,
  then iterate `+restore` with traces from there.

## 5. Gotchas known in advance

- **A checkpoint is model-specific.** Any RTL edit invalidates it; the meta file with the
  source hash is not optional.
- **The retire boundary, not the cycle.** Saving at an arbitrary cycle leaves the bridge with
  a half-compared retire and simmerv one instruction ahead or behind.
- **DPI context.** `VerilatedRestore` does not know about `g_ctx`; the C++ side must restore
  simmerv before the DUT's first `probe_retire()` after restore, or the first compare
  dereferences a null context.
- **Interrupt injection.** The model's pending interrupt that the DUT has been told to take
  but has not yet retired the pseudo-op for is bridge state; it must be in `.brg`, and the
  test in step 5 should include a save while `inj` is changing.
- **Verilator `--savable` and `--timing`.** Check compatibility on this Verilator before
  step 4; if they conflict, the `--exe` main with an explicit clock loop is the way out and
  is a modest change to the testbench (the `@(negedge clk)` loop becomes C++).
- **Size and time.** tiny128's `lram` is 512 MB of Verilog array; `VerilatedSave` writes it
  uncompressed. Expect ~0.6 GB and a few seconds per save on this host; Geekbench (2 GiB)
  ~2.2 GB. Acceptable on `/var/tmp`. Sparse saving is a later optimisation, not part of this
  plan.
- **The console.** The UART output already printed is not replayed after restore; the
  restored run's log starts mid-boot. Fine, but the gate scripts that grep for kernel
  console lines must not assume the boot banner is present.

## 6. Acceptance for the whole item

- Step 5's bit-exact round trip on tiny128 at three save points and on Geekbench at one.
- `ooo2/run-ooo2-cosim-gb5.sh` from the post-boot checkpoint reaches the old divergence
  retire (123,081,278, now clean) in under two minutes.
- `docs/OOO2-Spec.md` section 12 lists the checkpointed long-guest run as the per-batch gate
  and names the checkpoint's model hash rule.
