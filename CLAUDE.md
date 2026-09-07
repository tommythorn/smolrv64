## Project Context

- This is a RISC-V softcore project targeting FPGAs (current focus is
  on AMD XCKU5P) and Vivado toolchain integration.  The exernal Rust
  based Simmerv project is used extensively for cosimulation
  verification (which the caveat that it mustn't be taken as a golden
  reference per se).

- Builds and simulations can be slow; prefer targeted verification
  over full rebuilds.

- Always verify before committing that riscv-tests pass: run
  `ooo2/run-ooo2-vl.sh` (or the `tests/run-riscv-tests.sh` wrapper) —
  success is `pass=240 fail=0`. The unit benches are `ooo2/run-ooo2-*-tb.sh`
  and `src/run-tb.sh`; a port change on a shared module means all of them
  (rule G4). The core is `ooo2/`; the sequential and sharded-OoO cores that
  preceded it were deleted in the 2026-09 release.

- Also run `src/lint.sh` before committing RTL — success is `lint: clean`.
  It gates the load-bearing Verilator rules (width truncation, incomplete
  case, latch, comb loop, undriven, duplicate module). Waivers go in
  `src/verilator.vlt` and must name a file; never add a global `-Wno-`.

- Every non-trivial RTL change is validated by the Linux lockstep cosim
  (`CYC=300000000 ooo2/run-ooo2-cosim-linux.sh`, docs/OOO2-Spec.md §12) and
  then on the board (`tools/gate.sh`: build at the shipping configuration,
  program, boot Ubuntu to `login:` with zero faults).

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

## RTL defect rules

`docs/rtl-rules.md` is the full rule set, derived from the project's own
defect record with the commits that paid for each rule. Read it before
touching the core, cache, LSU or MMU.

- `docs/OOO2-Spec.md` is the normative description of the core in `ooo2/`
  (geometry, sizes, latencies, pipeline stages, what stalls, what restarts,
  known limits). It is ALWAYS kept current: if a change moves a number in that
  file, updating it is part of that same commit, not a follow-up. Measured
  figures must name their workload.

The non-negotiable ones:

- Invariant checks are ALWAYS ON (`$fatal`/`$display`, no `` `ifdef ``).
  Only flood-volume tracers and stats are gated. Every FSM `case` gets a
  `default` that asserts. "This cannot happen" is an assertion or it is
  deleted.

- Anything the design currently drops silently must assert instead —
  unmatched response, request accepted while busy, out-of-range index.
  Detection latency, not defect rate, is what costs days here.

- A response is matched by a tag the requester allocated, not by address
  and not by "only one in flight". A slot name (`{way,idx}`, freelist or
  queue index) captured now and dereferenced later carries its tag, or its
  slot is pinned against reallocation.

- A precondition that applies to N units is computed once and applied at
  one site. A new functional unit routes through the existing gate; never
  add a parallel path to the writeback or redirect port.

- Widths: no wide expression truncated at an array bracket, no width
  mismatch across a hierarchy boundary. Lint waivers are file-scoped in
  `verilator.vlt`, never global `-Wno-` flags.

- Fix the generator, not the instance. If a bug is the Nth of a class, the
  deliverable is the rule that makes N+1 impossible.

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

When the user reports a timing violation, run `make timing` yourself
to get the failing path, then analyze. Do not re-run `make bit` to
"see" timing — the checkpoint already has it.

## General work strategy

Do not run any build, sim, or test immediately. First give me: (a)
your single best hypothesis for the root cause, (b) the cheapest way
to confirm or refute it (ideally reading code or a log, not a full
rebuild), (c) what you'd expect to see if the hypothesis is right
vs. wrong.
