# SmolRV64 Pipelining Plan

This plan records the current direction for speeding up SmolRV64 while keeping
progress incremental and testable.  The goal is not to rewrite the core into a
full multi-issue or even fully overlapped pipeline in one pass.  The goal is to
introduce explicit pipeline boundaries one at a time, preserve architectural
behavior at each step, and use each passing boundary as a new stable base.

## Direction

Work from the current `dev` branch.  Keep the timing improvements and features
already landed there, but pause virtio bring-up until the CPU pipeline work has
made measurable progress.

Virtio should be hidden from Linux through the device tree first, not removed
from RTL.  The RK top-level virtio wiring and debug registers can remain
available for later coherency work, but Ubuntu should not probe the virtio
devices while CPU timing and performance are the active focus.

## Pipeline Model

Use a simple valid/enable discipline for new pipeline boundaries:

- A stage output is meaningful only when its `valid` bit is set.
- A stage may update its registered output only when its local `enable` is set.
- If `enable` is low, the stage holds its current output stable.
- Flush paths, including traps, interrupts, xRET redirection, and fetch faults,
  invalidate younger pipeline work.
- Do not allow a later stage to observe partially updated state from an earlier
  stage.

For the first milestones, keep instruction retirement in order and keep the
existing one-instruction-at-a-time FSM semantics.  The early pipeline registers
are timing boundaries first.  True instruction overlap should be considered
only after the fetch, register-read/decode, and execute boundaries are explicit
and tested.

## Current Shape

The core already contains partial pipeline-like staging:

- `S_FETCH1B` registers synchronous SRAM fetch data before `S_FETCH2`.
- `S_FETCH_BUF_CHECK` and `S_FETCH_BUF_USE` stage fetch-buffer lookup/use.
- `S_RF2` and `S_RF3` stage BRAM register-file reads and predecode.
- `S_EXECUTE2` completes registered ALU results from `S_EXECUTE`.

The next work should regularize these boundaries instead of adding another
large ad hoc state split.

## First Milestone: PC/Fetch Boundary

Start with PC generation and instruction fetch.  This is currently centered on
`S_FETCH1`, despite the name no longer describing only a fetch operation.

Add an explicit fetch request/result boundary while preserving existing
behavior:

- Register a fetch request when the retiring instruction's `npc` is accepted.
- Fetch request fields should include at least:
  - requested virtual PC;
  - privilege mode;
  - `satp` context;
  - trap/xRET suppression context needed by the existing interrupt behavior.
- Register a fetch result before decode consumes it.
- Fetch result fields should include at least:
  - fetched instruction bits;
  - instruction PC;
  - whether the fetch came from the fetch buffer, BRAM, DRAM, or translation;
  - fetch fault metadata, if any.
- Keep the existing fetch buffer, TLB/PTW, BRAM, DRAM, and cross-page/cross-8B
  behavior intact for the first patch.
- Rename states only after behavior is stable.  Avoid combining rename churn
  with logic movement.

Acceptance criteria for this milestone:

- No architectural behavior changes.
- `riscv-tests` still pass.
- Verilator Linux/cosim still reaches the same point as before.
- Ubuntu no longer probes virtio devices when using the updated Ubuntu DTS.

## Later Milestones

After the PC/fetch boundary is stable, proceed one boundary at a time:

1. Register/decode boundary
   - Make the `S_RF` to `S_RF3` path follow the same valid/enable convention.
   - Keep register-file BRAM timing explicit.
   - Keep predecode outputs stable until execute accepts them.

2. Execute boundary
   - Convert the existing `S_EXECUTE` to `S_EXECUTE2` split into a named
     execute request/result boundary.
   - Preserve the existing shared memory-address generation and predecoded ALU
     operation behavior.
   - Keep branches, jumps, CSR changes, traps, and memory operations as the
     primary correctness risks.

3. Memory/cache boundary
   - Only after fetch/decode/execute boundaries are stable, revisit load/store
     and cache request staging.
   - Do not mix cache coherency or virtio DMA work into the CPU pipeline patch
     series.

4. Reassess overlap
   - Once explicit boundaries exist, decide whether to allow fetch of the next
     instruction while the current instruction is in decode/execute.
   - Add hazards and flushes only when there is a clear performance target and
     a focused test strategy.

## Testing Strategy

Each pipeline patch should be small enough to test and revert independently.
Use the same test ladder after every behavioral change:

1. Build or lint `src/smolrv64.v`.
2. Run the fast RISC-V architectural tests.
3. Run Verilator Linux/cosim when available.
4. Boot the small Linux workload far enough to compare against the previous
   known-good output.
5. Run FPGA timing only after simulation tests pass.

Record the exact commands and result in the commit message or work log.  If a
test is skipped, record why.

## AI Workflow Rules

Use AI in smaller loops than the failed virtio effort:

- Give agents one bounded implementation target and one acceptance test.
- Prefer "make this exact boundary explicit" over "pipeline the core".
- Ask for falsifiable hypotheses when debugging, not broad fixes.
- Keep implementation, testing, and timing analysis as separate tasks.
- Use subagents only for independent side work, such as:
  - running a fixed test matrix;
  - reading timing reports and ranking critical paths;
  - checking that a patch preserved specific invariants.
- Do not delegate open-ended hardware debugging without a pass/fail signal.
- Stop after each passing boundary, commit or summarize the exact diff, and
  choose the next single boundary.

## Non-Goals For This Phase

- Do not resume virtio-net or virtio-blk implementation.
- Do not implement coherent DMA as part of the pipeline series.
- Do not remove virtio RTL unless it becomes a proven timing blocker.
- Do not attempt full instruction overlap before the stage contracts are
  explicit and verified.
- Do not combine state renaming, large refactors, and timing fixes in one patch.
