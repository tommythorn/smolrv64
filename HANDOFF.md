# Handoff: smolrv64 Linux boot on XCKU5P FPGA

## Goal

Boot OpenSBI + Linux on smolrv64 (RV64IMAFDC, Verilog) running on a
Xilinx XCKU5P FPGA with DDR4 backing store. Make output match
`workloads/linux/golden-output.txt` from the simmerv gold run.

Simulation boots Linux cleanly. FPGA boot stops after OpenSBI prints
`MEDELEG : 0x000000000000b109` and produces no further output.

## Repository layout (paths are relative to repo root)

- `src/` — core Verilog (`smolrv64.v`), Verilator sim wrappers
  (`sim_main.cpp`), BRAM images (`mem.even`, `mem.odd`).
- `src/Makefile` — builds three Verilator executables:
  - `smolrv64-linux-vl` (fast, production checks)
  - `smolrv64-linux-verbose-vl` (per-retire trace)
  - `smolrv64-linux-cosim` (differential vs simmerv)
- `workloads/linux/` — the consolidated Linux inputs (`fw_payload.bin`,
  `dts.dtb`, `mem.even`, `mem.odd`, `rf.hex`, `golden-output.txt`) and
  the Makefile that drives sim / cosim / FPGA upload.
- `workloads/monitor/` — tiny standalone monitor that runs out of BRAM
  on the FPGA. Supports xmodem loads and the `P`/`Pc` diagnostics
  command. Rebuilt automatically by `make load` at the platform level.
- `workloads/{diag,sieve}/` — small test programs for sim + FPGA.
- `platforms/rk-xcku5p-f-v1.2/` — FPGA project, build Makefile,
  Vivado tcl scripts.
  - `rk_xcku5p.srcs/rk_xcku5p.v` — top level; wires BRAM, DDR4 MIG,
    UART, reset button (`key[1]`) to the CPU.
  - `rk_xcku5p.srcs/ddr4_adapter.v` — 32-byte-burst bridge between the
    smolrv64 DRAM bus and the MIG native app interface. **See Open
    concern.**
- `/home/tommy/github/simmerv/` — the external Rust cosim reference.
  Cargo crate `simmerv_cli`. Caveat: FP uses host native + fenv, not
  trusted as gold for FP.
- `/home/tommy/.claude/plans/` — approved plan files.

## Build, program, run, regression

All Vivado runs are **headless**. Never ask for the GUI.

### RISC-V compliance (must pass before every commit)

```
make -C src 2>&1 | grep 'Test Passed' | wc -l
```

Expected: `209`. Any other value is a regression. This is the
non-negotiable quick gate before pushing.

### Linux simulation

```
make -C workloads/linux run               # Verilator, expect golden
make -C workloads/linux cosim             # smolrv64 vs simmerv diff
make -C workloads/linux are-we-there-yet  # run + diff vs golden
```

`are-we-there-yet` runs with `timeout 40` and `diff`s against
`golden-output.txt`. Prints `YES` on full match. Currently passes
through Linux userspace in sim.

### FPGA flow (run from `platforms/rk-xcku5p-f-v1.2/`)

```
make synth                              # synth only
make impl                               # synth + impl
make bit                                # synth + impl + bitstream
make program                            # JTAG program existing .bit
make load                               # rebuild workload + bit + program
                                        #   (WORKLOAD defaults to ../../workloads/monitor)
make timing                             # open impl_1 ckpt, report WNS/TNS + worst paths
make connect                            # screen /dev/ttyUSB1 3000000
```

Expect `WNS >= 0` (last clean build reported +0.037 ns).
`make timing` must be used for every timing question — do not re-run
`make bit` to inspect timing.

### FPGA Linux boot

1. `cd platforms/rk-xcku5p-f-v1.2 && make load` — builds the monitor
   into BRAM, rebuilds bit, programs the device. Drops you at the
   `smolrv64 monitor >` prompt on the serial console.
2. `cd workloads/linux && ./fpga-boot.sh [screen-session]`
   - Script autodetects the first attached `screen` session if no arg.
   - Uploads `dts.dtb` to `0x81000000` and `fw_payload.bin` to
     `0x80000000` via `sx -k` (xmodem-1k).
   - Waits for `Transfer complete` markers in `screenlog.0` (polls via
     `grep -c`; robust against stale content).
   - Stuffs `X80000000 0 81000000` to launch OpenSBI → Linux.
   - Prereqs: `screen -L` with `screenlog.0` writable, `sx` installed,
     screen's exec escape (`!!`) enabled.

To re-enter the monitor without reprogramming, press `key[1]` (CPU
soft reset). **Soft reset does NOT reset**:

- DDR4 MIG calibration state (so RAM contents survive).
- The custom telemetry CSRs (see below). Any write to any of them
  clears them all to sentinel values.

### Regression matrix

| Test                             | Command                                      | Expected |
| -------------------------------- | -------------------------------------------- | -------- |
| riscv-tests                      | `make -C src 2>&1 \| grep 'Test Passed' \| wc -l` | `209`   |
| Linux sim vs golden              | `make -C workloads/linux are-we-there-yet`   | `YES`    |
| Linux cosim (arch divergence)    | `make -C workloads/linux cosim`              | no diff  |
| Timing closure                   | `make timing` in platform dir                | `WNS>=0` |
| FPGA monitor boot + `P` works    | `make load` + press any key                  | prompt   |
| FPGA Linux reaches MEDELEG       | `fpga-boot.sh`                               | MEDELEG line |
| FPGA Linux reaches golden end    | `fpga-boot.sh`                               | **FAILS — current hang** |

Always use `timeout` when running smolrv64 sim binaries directly —
they can hang indefinitely.

## Diagnostic CSRs (custom MRO range)

All defined in `src/smolrv64.v`. Writing any of them to any value
clears all of them to their sentinel values — that is the monitor's
"clear stats" knob. These counters survive CPU soft reset (key[1]).

| CSR      | Name                     | Meaning |
| -------- | ------------------------ | ------- |
| `0xFC0`  | `CSR_MIG_MIN`            | min MIG round-trip latency (cycles) |
| `0xFC1`  | `CSR_MIG_MAX`            | max MIG round-trip latency |
| `0xFC2`  | `CSR_MIG_TOTAL`          | total latency sum (for avg) |
| `0xFC3`  | `CSR_MIG_COUNT`          | # completed MIG transactions |
| `0xFC4`  | `CSR_MIG_TIMEOUTS`       | # of bus timeouts fired |
| `0xFC5`  | `CSR_MIG_TO_PC`          | PC at first timeout |
| `0xFC6`  | `CSR_MIG_TO_TVAL`        | tval at first timeout |
| `0xFC7`  | `CSR_MIG_TO_STATE`       | CPU state machine state (5-bit) at first timeout |
| `0xFC8`  | `CSR_MIG_TO_CAUSE`       | synthesized trap cause |
| `0xFC9`  | `CSR_MIG_TO_ADDR`        | physical byte addr of the in-flight burst |
| `0xFCA`  | `CSR_MIG_TO_RD_ISSUED`   | `mig_rd_issued_count` snapshot at first timeout |
| `0xFCB`  | `CSR_MIG_TO_RD_RESP`     | `mig_rd_resp_count` snapshot at first timeout |

`CSR_MIG_TO_RD_ISSUED`/`_RESP` are driven from free-running counters in
`ddr4_adapter.v`:

- `mig_rd_issued_count`: increments on the cycle MIG accepts a READ
  command (`state == RD_CMD && app_rdy`).
- `mig_rd_resp_count`: increments on every `app_rd_data_valid` pulse.

Use `P` in the monitor to print everything; `Pc` clears.

## Key facts still valid

- `fw_payload.bin @ 0x80000000`, `dts.dtb @ 0x81000000` (updated from
  earlier `0x82000000` — see commit `57bd22e`).
- BRAM lives at `0x70000000` for monitor/workload loads
  (`MEM_BASE=0x70000000`).
- UART is NS16550A at `0x10000000`. MMIO dispatch in `smolrv64.v`
  keyed on `mem_addr[63:4] == 60'h100_0000`. `uart_tx_valid` asserts
  only when offset==0 and `uart_lcr[7]==0` (DLAB=0).
- `src/uart5.v` was obsolete and has been deleted.
- Cosim + sim both match `golden-output.txt` through userspace given
  the consolidated inputs. Divergences in past have been due to
  different memory images between sim and cosim — always use the
  files under `workloads/linux/`.
- simmerv must not be treated as a golden reference for FP; it uses
  host native + fenv. The blessed Linux-boot gold is the static
  `golden-output.txt`.

## Conventions (do not reinvent)

- **No Co-Authored-By** in commit messages. Terse, present-tense
  subject.
- **Follow existing CLI conventions** — e.g. `-d image,addr`
  comma-separated, not new flags.
- **Never edit generated files** (decoder tables, Vivado IP runs);
  modify the generator or source.
- **Only change what was requested.** If asked to tune dTLB, do not
  touch iTLB.
- **Hypothesis first for any debug.** No build/sim/test runs without
  first stating (a) root-cause hypothesis, (b) cheapest way to confirm
  or refute it, (c) expected signal if right vs. wrong. This is a
  hard rule per `CLAUDE.md`.
- **Vivado only via the platform Makefile** — do not run raw
  `vivado` commands or open the GUI. `LD_LIBRARY_PATH` is set in the
  Makefile to avoid the `libtinfo.so.5` error.
- **Use `timeout`** on every direct smolrv64 invocation.

## Quick reference: state machine encoding

`S_DRAM_PTW_WAIT = 18 (0x12)` — seen in the current hang.
`S_DRAM_FETCH_WAIT = 16 (0x10)`.
`S_DRAM_LOAD_WAIT = 17 (0x11)`.
Full list in `src/smolrv64.v` near the state-machine defines.

## Recent commits (for context)

This might be out of date

```
57bd22e  Match the 0x81000000 dtb address
e588d65  workloads/linux: Are We There Yet?
1c88dc9  smolrv64.v: suppress stale pre_intr_pending after xRET and CSR writes
48a07a4  smolrv64.v: suppress stale pre_intr_pending on first post-trap fetch
679bd48  smolrv64.v: cosim hook fixes for prv and instruction-fetch faults
```

Uncommitted changes in-tree relevant to this handoff:

- `ddr4_adapter.v`: added `mig_rd_issued_count`, `mig_rd_resp_count`
  outputs + increment logic. Not yet reviewed against the MIG spec.
- `rk_xcku5p.v`: wires the above counters from adapter to CPU.
- `smolrv64.v`: adds `CSR_MIG_TO_ADDR` (0xFC9), `CSR_MIG_TO_RD_ISSUED`
  (0xFCA), `CSR_MIG_TO_RD_RESP` (0xFCB) and their first-timeout latch;
  also the earlier `bus_timeout_expired` FF that fixed WNS.
- `workloads/monitor/monitor.c`: `P` command prints the new fields.
- `workloads/linux/fpga-boot.sh`: new script (see **FPGA Linux boot**).
- `platforms/rk-xcku5p-f-v1.2/Makefile`: adds `timing` target and
  `load` convenience target.

Do not commit the Vivado build artifacts under
`platforms/rk-xcku5p-f-v1.2/rk_xcku5p.cache/` or `rk_xcku5p.runs/`.
