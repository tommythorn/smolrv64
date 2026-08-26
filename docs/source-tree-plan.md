# Where the source tree is going

Recorded 2026-08-19, from Tommy. This is the intended END STATE, not the current one, and
it should be read before any change that "generalises" a module to serve several cores.

## The end state

Two cores, and only two:

1. **The in-order core** — `ooo2/`, what currently ships to the XCKU5P and boots Linux.
2. **A new OoO core** — not yet written (see `docs/Area-Efficient-Scalar-OoO.md` and
   `docs/sharded-ooo-plan.md` for the thinking that feeds it).

Both of these are to be **deleted**:

- the **sequential core** (already retired from the test harness on 2026-08-03, remnants
  remain), and
- the **sharded OoO core** in `src/` (`soc_top`, `backend_top`, `exec_bundle`,
  `rename_shard`, `commit_ctl`, the shard machinery).

## Why this matters for how changes are made

Shared modules in `src/` are currently shared between a core that is staying and a core
that is going. That has already produced two avoidable detours:

- **Parameterising instead of forking.** `predictor.v` grew a `CKPT_RAS` knob so the
  in-order core could opt out of per-checkpoint RAS snapshots while the sharded core kept
  them. The knob was the wrong shape: the in-order core needs no checkpoint ring *at all*,
  and the OoO core that replaces the sharded one will not need one either. The right move
  was `ooo2/ooo2_predictor.v` — a fork with the whole mechanism deleted. Done in the
  commit that added this file.
- **Reasoning about the wrong constraints.** A whole design note was written about how to
  size `NCHK` for a decoupled frontend queue, when the correct answer was that the in-order
  core has no checkpoints to size.

**Rule of thumb until the deletions happen:** when a `src/` module's requirements differ
between the in-order core and the sharded OoO core, FORK IT into `ooo2/` rather than
adding a parameter. A fork is deleted for free when the sharded core goes; a parameter has
to be found and unwound, and in the meantime it makes every reader ask which core a line of
code is for.

The cost of a fork is drift (`ooo2/rv_cache.v` is a fork of `src/cache.v` and has
already missed three upstream fixes — the two wrong-slot corruption fixes and the
`PERF_TRACE` ifdef that left the cache HPM counters reading zero on every bitstream). That
cost is real, but it is bounded and visible, and it ends when the sharded core does.

## What genuinely needs committed + speculative state

Not the predictor: it is a hint structure, and `ooo2_predictor.v` keeps two committed scalars
(GHR, RAS pointer) purely so a redirect resumes with sane history. The structure that will
genuinely need a committed and a speculative copy is the **rename MAP** in the new OoO core.
Nothing in the in-order core needs one.
