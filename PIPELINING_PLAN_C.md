# Pipelining Plan C

Plan C is the current strategy for moving SmolRV64 toward a true in-order
pipeline without repeating the failed pattern of widening old FSM predicates.

## Direction

Stop trying to get overlap by moving individual launch calls earlier. Those
changes can be locally plausible while still failing only under long FPGA Linux
boots. Future work should make explicit stage ownership visible before enabling
new overlap.

The core idea is to introduce real stage boundaries first, keep behavior
unchanged at each structural checkpoint, and only then allow safe overlap across
those boundaries.

## Stage Boundaries

Add explicit stage records or grouped registers with `valid` bits for:

- fetch/decode handoff;
- register-file read;
- execute request.

At first these boundaries may still advance one instruction at a time. That is
acceptable: the first milestone is making ownership and payload stability
obvious in RTL, not improving CPI immediately.

Each boundary should follow the same contract:

- a stage owns its `valid` bit and payload;
- a producer may write only when the stage is empty or accepted in the same
  cycle;
- a consumer may read only when `valid` is set;
- if valid work is not accepted, the payload remains stable;
- redirects invalidate younger work through one explicit path.

## Ownership

Move computation into the earliest stage that naturally owns it:

- frontend/decode owns instruction bytes, decoded register indexes, immediates,
  length, predicted next PC, epoch, and other frontend metadata;
- RF owns integer and FP operand reads plus local writeback bypassing;
- execute owns ALU, branch, load/store, CSR, trap, and retire intent;
- cache/data-side logic owns D-memory request and response packaging.

The global FSM should shrink toward backpressure, rare multi-cycle operations,
and exceptional control. It should not remain the owner of ordinary instruction
flow.

## Checkpoints

1. Add an explicit execute request boundary with no intended behavioral change.
2. Retarget RF launch so it fills that execute request boundary instead of
   directly steering backend execution state.
3. Add assertions or local invariants for stage validity, payload stability, and
   redirect flushing.
4. Allow frontend/decode to stay ahead while execute or memory is stalled.
5. Allow RF for the next instruction while the current instruction is in execute
   when there is no RAW, control, CSR, or memory-ordering hazard.
6. Add bypass or scoreboard machinery only after the stage boundaries make the
   hazard source explicit.
7. Split independent I-cache and D-cache miss progress after the frontend and
   backend have clean cache-side ownership.

## Testing Policy

Pure renames and mechanical payload moves may skip FPGA testing when the change
is clearly non-behavioral.

Any change that alters when work launches, when a stage accepts work, when a
redirect flushes work, or when a memory/cache response is consumed needs the
full hardware-oriented gate. Recent regressions showed that simulator-only
coverage is not enough for those changes.

For behavioral overlap commits, prefer this order:

- fast simulator build;
- riscv-tests or targeted simulator smoke test;
- Linux boot marker simulation when relevant;
- Vivado timing/bitstream;
- FPGA program and Ubuntu boot test.

## Near-Term First Step

The next small structural commit should add an explicit execute-valid payload
boundary around the current execute inputs, with no intended behavioral change.
Once that is stable, RF can target the boundary, and later commits can safely
start overlapping younger work with older execute/memory waits.
