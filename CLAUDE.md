## Project Context

- This is a RISC-V softcore project targeting FPGAs (current focus is
  on AMD XCKU5P) and Vivado toolchain integration.  The exernal Rust
  based Simmerv project is used extensively for cosimulation
  verification (which the caveat that it mustn't be taken as a golden
  reference per se).

- Builds and simulations can be slow; prefer targeted verification
  over full rebuilds.

- Always verify before committing that of riscv-test in passing as
  passing as expeced (currently `(make)|&grep 'Test Passed'|wc -l`
  returning 209 is success).

## Code Change Conventions

- Always think really hard about simplifying the problem and the
  solution.  If something needs a special case, go back and revisit
  why it's a special case and see if there's a way to make the special
  case fold in natual to a general case.

- Aggressively favour less and simpler code over more code, refactor
  when it can reduce complexity or size of code.

- Never manually edit generated files (e.g., decoder files); modify
  the generator or source instead.

- Follow existing CLI/argument conventions (e.g., comma-separated
  address args for images/DTB) rather than inventing new flags.

- Only change what was explicitly requested - e.g., if user asks to
  tune dTLB, do not also modify iTLB.

## Debugging Approach

- Always identify the root cause before running builds, simulations,
  or tests - do not 'try things' to burn cycles.

- When given a stale context summary, verify the current state of the
  problem before acting on prior plans.

- Limit initial codebase exploration; if stuck after a few Read/Grep
  calls, ask the user for direction.

## Vivado workflow (XCKU5P platform)

All Vivado operations are driven headlessly via the Makefile in
`platforms/rk-xcku5p-f-v1.2/`. Never ask the user to open the Vivado
GUI or run raw `vivado` commands — invoke the Make targets directly
from that directory. The Makefile sets `LD_LIBRARY_PATH` so `vivado
-mode batch` works without the libtinfo.so.5 error.

Targets (run from `platforms/rk-xcku5p-f-v1.2/`):
- `make synth`   — synthesis only
- `make impl`    — synthesis + implementation
- `make bit`     — synth + impl + bitstream (default `all`)
- `make program` — program the device over JTAG (requires existing .bit)
- `make load`    — build workload (WORKLOAD=path), rebuild bitstream, program
- `make timing`  — open impl_1 checkpoint, print WNS/TNS and worst
                   setup paths (no rebuild). Use this for every timing
                   diagnosis instead of asking the user to run reports.
- `make clean`   — wipe generated outputs (keeps sources and .xpr)
- `make connect` — screen to $(TTY) at 3 Mbaud

Scripts live alongside the Makefile:
- `build.tcl`         — synth/impl/bit flow driver
- `program.tcl`       — JTAG program
- `report_timing.tcl` — WNS/TNS + worst 5 setup + worst 5 failing paths

When the user reports a timing violation, run `make timing` yourself
to get the failing path, then analyze. Do not re-run `make bit` to
"see" timing — the checkpoint already has it.

## General work strategy

Do not run any build, sim, or test immediately. First give me: (a)
your single best hypothesis for the root cause, (b) the cheapest way
to confirm or refute it (ideally reading code or a log, not a full
rebuild), (c) what you'd expect to see if the hypothesis is right
vs. wrong.
