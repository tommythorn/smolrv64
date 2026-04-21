# Handoff: smolrv64 Linux boot on XCKU5P FPGA

## Goal

Boot OpenSBI v0.8 + Linux 5.4 on smolrv64 (RV64IMAFDC, Verilog) running
on a Xilinx XCKU5P FPGA with DDR4 backing store. Make output match
`workloads/linux/golden-output.txt` from the simmerv gold run.

Simulation boots Linux cleanly. FPGA boot stops after OpenSBI prints
`MEDELEG : 0x000000000000b109` and produces no further output. The
single `mig timeouts=1` latched during the run is the smoking gun we
are chasing — see **Current working theory** and **Open concern** below.

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

## Current working theory (2026-04-20)

On the last FPGA boot after MEDELEG:

```
state = 0x12  (S_DRAM_PTW_WAIT)
cause = 1     (INSTR_ACCESS_FAULT)
pc    = 0xffffffff800000b6
tval  = 0xffffffff800000b6
addr  = 0x0000000000223fe0   (DDR byte addr of the PTE fetch)
issued = 0x34bc35
resp   = 0x34bc35            (EQUAL)
```

Importantly, even after a power cycle, we can reproduce the issue
repeatedly and always with exactly the values above (ruling out
marginal timing issues and other random events).

Likely relevant is that the last instruction prior to the hang is the
first memory access after turning on virtual memory (a store to
address 0xffffffff8082fce8).  Here is the corresponding execution
trace from the software simulation.  This strongly points the finger
at the page table walker:
```
2757854 1 00000000802000dc 962e     c.add       a2, a2, a1, 0000000000000000      ffffffff80000104
2757855 1 00000000802000de 10561073 csrrw       , a2, x0, 0000000000000105
2757856 1 00000000802000e2 00c55613 srli        a2, a0, x0, 000000000000000c                 80223
2757857 1 00000000802000e6 fff0059b addiw       a1, x0, x0, 00000000ffffffff      ffffffffffffffff
2757858 1 00000000802000ea 15fe     c.slli      a1, a1, x0, 000000000000003f      8000000000000000
2757859 1 00000000802000ec 8e4d     c.or        a2, a2, a1, 0000000000000000      8000000000080223
2757860 1 00000000802000ee 0088c517 auipc       a0, x0, x0, 000000000088c000              80a8c0ee
2757861 1 00000000802000f2 f1250513 addi        a0, a0, x0, 00000000ffffff12              80a8c000
2757862 1 00000000802000f6 8131     c.srli      a0, a0, x0, 000000000000000c                 80a8c
2757863 1 00000000802000f8 8d4d     c.or        a0, a0, a1, 0000000000000000      8000000000080a8c
2757864 1 00000000802000fa 12000073 sfencevma   , x0, x0, 0000000000000000
2757865 1 00000000802000fe 18051073 csrrw       , a0, x0, 0000000000000180
2757867 1 ffffffff80000104 00000517 auipc       a0, x0, x0, 0000000000000000      ffffffff80000104
2757868 1 ffffffff80000108 06850513 addi        a0, a0, x0, 0000000000000068      ffffffff8000016c
2757869 1 ffffffff8000010c 10551073 csrrw       , a0, x0, 0000000000000105
2757870 1 ffffffff80000110 00886197 auipc       gp, x0, x0, 0000000000886000      ffffffff80886110
2757871 1 ffffffff80000114 7b818193 addi        gp, gp, x0, 00000000000007b8      ffffffff808868c8
2757872 1 ffffffff80000118 18061073 csrrw       , a2, x0, 0000000000000180
2757873 1 ffffffff8000011c 12000073 sfencevma   , x0, x0, 0000000000000000
2757874 1 ffffffff80000120 8082     c.jr        , ra, x0, 0000000000000000
2757875 1 ffffffff800000aa 00830217 auipc       tp, x0, x0, 0000000000830000      ffffffff808300aa
2757876 1 ffffffff800000ae c1620213 addi        tp, tp, x0, 00000000fffffc16      ffffffff8082fcc0
2757877 1 ffffffff800000b2 02022423 sw          , tp, x0, 0000000000000028
2757878 1 ffffffff800000b6 0082a117 auipc       sp, x0, x0, 000000000082a000      ffffffff8082a0b6
```


Interpretation:

- The sign-extended VA is a legitimate kernel-space access; PTE offset
  matches, so it is real kernel code, not garbage PC.
- `issued == resp` means the adapter had serviced **every** read it
  had been handed when the timeout latched. No read was in flight from
  the MIG's point of view. **MIG exonerated for this timeout.**
- The CPU was nonetheless sitting in `S_DRAM_PTW_WAIT` waiting for
  `dram_readdatavalid` that (by the counters) had either already been
  delivered, or was never actually issued.
- Simulation does not reproduce this. FPGA is conservatively clocked
  and actively cooled, so pure timing marginality is unlikely.

After the trap, the CPU keeps executing (the monitor's `count` climbs
for hundreds of millions of MIG ops) but produces no further console
output — so Linux is alive but stuck in some silent loop. **Important:**
`P` can only be invoked from the monitor after a soft reset, so you
cannot poll Linux liveness directly with it; the counters only tell
you about the last run window up to the moment key[1] was pressed.

## Open concern: ddr4_adapter handshake correctness

**This is where the user wants the next session to look.** The user is
not convinced the MIG native-app handshake is used correctly in
`platforms/rk-xcku5p-f-v1.2/rk_xcku5p.srcs/ddr4_adapter.v`. Specific
things to audit against the Xilinx DDR4 MIG native interface spec:

1. **`app_en` + `app_rdy` simultaneous-assertion requirement.** The
   MIG spec requires `app_en` high **and** `app_rdy` sampled high on
   the **same** rising edge for the command to be accepted. The
   adapter registers `app_en` (`<=`) so it only becomes high on the
   *next* cycle. IDLE asserts `app_en <= 1` and transitions to RD_CMD;
   RD_CMD then checks `app_rdy` while `app_en` is already high. That
   is the claimed correct moment. Verify:
   - On acceptance, does the default-deassert of `app_en` (from the
     outer `always @` default) correctly stop asserting on the next
     cycle, or could it issue a duplicate command?
   - On the IDLE→RD_CMD transition: is it actually possible for RD_CMD
     to see `app_rdy=1` on its first cycle (giving a 1-cycle total
     issue), or does the MIG require at least one `app_en=1 while
     app_rdy=0` cycle first?

2. **Write data FIFO ordering.** `app_wdf_rdy` can deassert
   independently of `app_rdy`. The current code uses a `wdf_done`
   latch to prevent double-pushing if `app_wdf_rdy` leads `app_rdy`.
   But: is it legal to push WDF data **before** the write command is
   accepted, or must the command be in first / at the same time? The
   comments claim "either order is fine"; confirm against the IP
   product guide (PG150 / UG586 / the MIG configuration report in the
   Vivado project).

3. **`app_wdf_end` pulsing.** Currently latched to 1 alongside the
   write command and never explicitly deasserted during WR_CMD.
   `app_wdf_end` should be high **only** on the final cycle of a
   write-data burst. For a 256-bit APP_DATA_WIDTH equal to the MIG's
   internal burst size, that is one cycle. If the MIG sees
   `app_wdf_end=1 && app_wdf_wren=1` for *two* consecutive cycles
   during a stall, does it treat that as two bursts?

4. **Read response ordering.** For back-to-back reads the MIG can
   return data out of order in some configurations. The adapter
   currently handles only one in-flight read (IDLE → RD_CMD → RD_WAIT
   → IDLE). If two reads could ever be in flight, `app_rd_data_valid`
   could reorder. The counter pair `issued==resp` means no reads were
   dropped, but it does NOT prove that `dram_readdata` captured the
   right burst. Consider checking `app_rd_data_end` too.

5. **`dram_abandon_read` + `RD_DRAIN`.** When the CPU times out, the
   adapter transitions to RD_DRAIN to swallow the late response. But
   during RD_DRAIN the CPU might issue a fresh `dram_read` — and
   `dram_write_ready = (state == IDLE)` is false, so the pulse is
   lost. That is a pure bug source for post-timeout dead-silence,
   matching the observed FPGA symptom. Worth auditing whether the CPU
   can reach a state where it asserts `dram_read` while adapter is
   non-IDLE.

6. **Reset domain.** `ddr4_adapter` uses `rst_n = ~ui_rst` from the
   MIG UI clock. The CPU uses a separate derived `cpu_reset`. If the
   CPU asserts `dram_read` during the window after `cpu_reset` but
   before the adapter has seen any reset, the pulse may be misread.
   The current counters (`mig_rd_issued_count` / `_resp_count`) are
   **intentionally not reset** to allow cross-reset diagnostics.

### Concrete next experiments (cheapest first)

1. **Verify `app_en` never stays high for two consecutive accepted
   cycles.** Add a counter `mig_duplicate_cmd_count` that increments
   when `app_en=1 && app_rdy=1` while the adapter believes it already
   transitioned out of RD_CMD/WR_CMD. Should always be 0.
2. **Add a `dram_read_dropped_count` CSR.** Count cycles where
   `dram_read=1` asserted by the CPU but `dram_write_ready=0` (i.e.
   the adapter was not IDLE). If nonzero at the hang, the adapter is
   silently losing requests — very plausible root cause.
3. **Sim-level MIG assertions.** Write a tiny Verilog testbench that
   instantiates `ddr4_adapter` with a mock MIG model enforcing the
   protocol (no `app_rdy=1` on the same cycle as a previous accepted
   command unless `app_en` dropped and rose again, etc.) and drive it
   with fuzzed patterns. Catch the violation in sim, not silicon.
4. **Re-check `app_wdf_end` single-cycle-pulse rule** against the
   MIG's actual configuration (`APP_DATA_WIDTH=256`, the full MIG
   burst — so one `app_wdf_end` cycle is expected).

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
