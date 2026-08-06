# Confidence-gated checkpoint inclusion

**Status:** planned (2026-08-05). Design per TT: branches carry a probabilistic
confidence (direction x {weak, strong, stronger, certain}); *stronger*/*certain*
branches are followed and ride INSIDE a checkpoint rather than closing it. If a
branch mispredicts that was not the last in its checkpoint, the whole checkpoint
rolls back, and the frontend must remember to cut the checkpoint before that
branch on the retry.

## Why

`backend_top.v` closes a checkpoint at every CTI, so the window is one basic
block. The dispatch comment already names the blocker: keeping ONE CTI per
checkpoint is what makes the per-checkpoint mispredict reference (`pnpc`) and
predictor train details (`pdet`) correct, and recovery precise. Letting
confident branches ride multiplies the window by the number of confident
branches per checkpoint -- the straight-line run between *unpredictable*
branches, not between all branches.

## What makes it cheap

The 64-bit `pnpc` compare in `exec_shard.v` is **JALR-only** (`ex_pnpc`, "for the
JALR-only EX-time compare"). A conditional branch's prediction is just a
DIRECTION bit -- its target is PC+imm, computed in EX. So conditional branches
need one carried bit, not a carried 64-bit target. Keeping JALR (and, initially,
JAL) as checkpoint closers leaves the existing per-checkpoint `pnpc` array valid
exactly where it is still used.

## Steps (each independently gated on run-vl-tests.sh + a cosim boot)

1. **Predictor confidence.** Widen the bimodal direction counter so each
   direction has 4 strengths (8 states). `hiconf` = the top two strengths.
   Expose it per predicted branch. Inert: nothing consumes it yet.
   - **Mispredict resets confidence to weak** rather than decrementing. A
     decrement from *certain* would still read as *stronger*, so the retry would
     ride the same branch again and roll back again -- a livelock. Reset makes
     the predictor entry itself the "remember to cut" memory TT's design calls
     for, with no new structure (see [[feedback_reuse_existing_framework]]).

2. **Per-branch direction compare.** Carry the predicted direction as a payload
   bit; resolve conditional mispredicts against it instead of the
   per-checkpoint `pnpc`. Leaves JALR on the existing path.

3. **Per-branch train details.** `pdet` is indexed by checkpoint today (the
   predictor header says "<=1 CTI per checkpoint"). Allocate a small branch tag
   at dispatch, index `pdet` by it, and carry only the tag (~4 bits) with the
   instruction.

4. **Whole-checkpoint rollback.** Store each checkpoint's start PC. A mispredict
   on a non-last CTI restores the checkpoint's start map and redirects the
   frontend there. Add a one-shot "close at the first CTI" flag consumed by the
   retry -- step 1's confidence reset already handles this in practice, but the
   flag makes forward progress independent of predictor aliasing/eviction.

5. **Enable the gate.** `disp_close` stops firing for high-confidence
   conditional branches. Everything before this is inert plumbing, so this is
   the single step that changes behaviour and the one to bisect against.

## Correctness notes

- A rolled-back checkpoint must not have retired. Commit is per checkpoint and
  waits for all its ops, and stores sit in the store buffer until commit, so
  this holds today.
- Rollback is imprecise by construction (instructions before the branch are
  re-executed). The retry cuts before the branch, so the second attempt resolves
  it at a checkpoint boundary and converges.
- Barriers (SER/CSR/AMO/CBO/FENCEI) keep closing and keep `disp_barrier`.
