# SmolRV64 Pipelining Plan

This document tracks the current pipeline direction.  It is a working plan, not
a constraint: when it conflicts with getting to a real pipeline, rewrite it.

## Goal

Move SmolRV64 from a mostly single-instruction FSM to a simple in-order pipeline
with an independent frontend and a retiring backend.

The target is not out-of-order execution or multi-issue yet.  The target is:

- one instruction retires in order;
- frontend fetch/decode runs ahead of backend execution on the common path;
- the common path is aggressively speculative;
- wrong speculation is cheap to discard and restart;
- redirects from branches, jumps, traps, interrupts, xRET, fences, and address
  space changes flush younger work through one explicit protocol;
- every commit remains buildable, testable, and timing-accounted.

This is a stepping stone toward a speculative superscalar out-of-order design.
The work should bias toward clean stage contracts and short timing paths, not
toward preserving the old FSM state structure.

## Current Baseline

The current kept work is a transitional overlap mechanism inside the old FSM:

- `rf_decode_valid` is now a one-entry frontend/backend handoff candidate.
- Register-file read launch is split from decode enqueue with an `rf_read_*`
  payload.
- `next_pc` and frontend epoch travel with the queued instruction.
- A validated queued decode can launch RF read immediately.
- A direct non-speculative fetch can bypass the decode slot and launch RF read
  in the same cycle.
- Speculative fetch-buffer hits can be consumed while selected backend states
  are busy.
- A conservative speculative frontend miss path exists for physical,
  cacheable, non-cross-doubleword cases.

This proved useful as preparation, but it is not the final pipeline shape.  Do
not keep widening broad FSM predicates indefinitely.  The CSR-state speculation
attempt failed timing with only a small logical change, which is evidence that
the old control structure is now the bottleneck.

## Pipeline Contract

Use ready/valid boundaries and registered payloads:

- A stage owns a `valid` bit and its payload.
- A producer may write a stage only when the stage is empty or the consumer
  accepts it in the same cycle.
- A consumer may read a stage only when `valid` is set.
- If a stage is valid and not accepted, its payload stays stable.
- Redirects invalidate younger stages.
- Each speculative frontend payload carries an epoch.
- Frontend-invalidating events advance the epoch.
- Retirement remains in order at the backend.

Do not protect old state names.  Preserve architectural behavior and timing
discipline.

## Architecture Direction

Build two real halves.

### Frontend

The frontend owns:

- sequential PC generation and simple prediction;
- instruction cache lookup and fill;
- TLB/PTW interaction for instruction fetch;
- fetch alignment, compressed-instruction length handling, and slow-path
  cross-boundary cases;
- decode of register indexes and immediate frontend metadata;
- a one-entry or small decode queue for the backend.

The frontend should be able to run without consulting the backend FSM on the
fast path.  Backend events should arrive as redirects, flushes, and epoch
increments, not as ad hoc state exemptions.

### Backend

The backend owns:

- consuming decoded work in order;
- integer/FP register-file reads;
- execution, load/store, CSR, trap, and retirement state machines;
- redirect generation;
- cache/data-side ownership where required;
- commit-time validation of speculation.

Backend complexity is allowed to remain state-machine based for rare cases.
The common frontend path should not wait for that state machine unless a real
resource or ordering rule requires it.

## Instruction Cache

Add an instruction cache as part of the pipeline, not as a later polish item.

Rationale:

- A real frontend otherwise fights the data/cache path.
- The current fetch buffer can hide some latency, but it is too entangled with
  backend control and fetch request selection.
- A 64-byte I-cache line matches the memory system and gives the frontend a
  natural unit for sequential fetch.
- The I-cache creates a clean timing boundary: PC/index/tag lookup, registered
  hit/miss result, and a separate refill path.

Initial I-cache policy:

- direct-mapped or very small set-associative, whichever is simpler and closes
  timing;
- physically indexed/tagged after translation for the first implementation;
- 64-byte lines;
- no coherence beyond explicit `fence.i` invalidation;
- flush or invalidate on frontend epoch changes that require it;
- slow path for page crossing and awkward unaligned/cross-doubleword cases.

The first I-cache does not need to be clever.  It needs to move the common
sequential instruction stream out of the backend FSM.

## Speculation Policy

Speculate for the common fast path and restart on deviations unless a stall is
clearly cheaper in measured hardware.

Default policy:

- predict fall-through by instruction length;
- let frontend fetch/decode younger sequential work;
- carry only the per-instruction data needed to validate/retire the payload;
- keep slow-changing context as shared architectural state when updates can be
  made serializing;
- at backend retirement, accept the younger work only if the resolved next PC
  and context match;
- otherwise flush younger work, advance epoch as needed, and restart frontend at
  the resolved target.

Restart-first is the default because it keeps timing local.  Stalling requires
hazard checks, wakeup conditions, and muxing that can easily cost more timing
than the occasional restart costs CPI.

Use stalls when:

- the condition is common enough that repeated restart burns measurable CPI;
- the stall check is local and cheap;
- timing reports show the stall machinery is not on a critical path.

Use bypasses only after stall cost is measured.  Bypasses are performance
features, not correctness scaffolding.

## Deviations And Slow Paths

The common path should be straight-line, cached, translated, aligned, and
non-trapping.  Everything else gets a clear recovery strategy:

- taken branch/jump: backend redirect, flush younger work, restart frontend;
- trap/interrupt/xRET: backend redirect/flush, epoch advance where needed;
- `fence.i`: invalidate I-cache/fetch buffer, advance frontend epoch, restart;
- `sfence.vma`: flush relevant translation/frontend state, advance epoch,
  restart;
- SATP write: flush translation/frontend state, advance epoch, restart;
- I-cache miss: frontend refill state machine, then resume;
- page crossing or cross-doubleword instruction: slow frontend state machine;
- data cache conflict with frontend refill: arbitrate explicitly or make the
  frontend wait at the refill boundary, not in the hit path;
- illegal fetch/access fault: frontend reports fault payload, backend retires
  it in order.

The important rule is that deviations are explicit state-machine paths at stage
boundaries.  They should not leak into the fast-path PC and fetch muxes.

## Pipeline-Carried State Policy

Do not automatically copy every global register into every pipeline payload.
For each piece of state that crosses a stage boundary, classify it first:

- **Per-instruction data:** must travel with the instruction because different
  in-flight instructions can legitimately need different values.
- **Slow context:** should have one shared current copy, plus an epoch/checkpoint
  if needed.  Updates are serializing barriers: flush younger frontend work,
  wait for outstanding slow-path work to drain or explicitly kill it, update the
  shared copy, then restart.
- **Derived data:** should be recomputed locally from compact state, or carried
  only after it has been narrowed to the exact bits used by later stages.

Current candidates:

- `satp`: slow context.  The TLB needs only address-space identity, not the
  entire CSR.  For Sv39 this is implemented ASID bits plus root PPN; `MODE` is
  implicit because Bare bypasses translation.  SATP writes are serializing and
  flush frontend/TLB state.
- ASID width: implement 10 ASID bits unless measurements show pressure.  WARL
  zero the unused upper ASID bits so the TLB key does not carry them.
- `sum`/`mxr`: slow context for translation permission checks.  A future pass
  should evaluate whether SSTATUS writes can be made serializing for outstanding
  translation, letting TLB/PTW use shared context instead of carrying these bits
  everywhere.
- privilege and MPRV/MMP context: mixed.  Retired architectural privilege is
  slow context, but effective access privilege for a load/store/fetch may be
  per-operation and must be captured once the operation is issued.
- `frm`/`fflags`/`fs`: FP context.  `frm` is slow unless dynamic rounding mode
  is used; `fflags` is retire/side-effect state and should not sit on common
  integer timing paths.
- frontend epoch: per-speculation metadata, currently 2 bits.  It exists only
  to distinguish stale frontend work in this in-order core, so do not widen it
  unless there are enough independently-live stale payloads to justify the
  extra state.

This review should happen before adding new pipeline fields.  A wide copied
field is a timing smell unless there is a clear per-instruction correctness
reason for it.

## Timing Strategy

Timing is now a design constraint, not a final check.

Rules:

- run FPGA timing for Verilog changes before treating a commit as hardware
  ready;
- when timing fails, fix or revert before stacking more pipeline work;
- do not keep adding cases to broad combinational predicates as a substitute for
  a pipeline boundary;
- prefer registered redirect/flush signals over reading backend state directly
  in frontend selection logic;
- keep PC, epoch, SATP, and privilege muxing shallow;
- use Vivado worst-path reports to choose the next structural cleanup;
- record surprising timing failures in this document when they change the plan.

Known pressure points:

- `fetch_req_pc` selection;
- frontend epoch control;
- broad state predicates;
- cache/fetch buffer hit logic;
- paths that combine backend retirement with next frontend request generation.

## Near-Term Work

1. Clean warning and constraint noise
   - Fix project/XDC warnings that affect timing confidence.
   - Classify generated DDR4-IP warnings separately from repo-owned warnings.
   - Keep timing reports actionable.

2. Introduce explicit frontend command/result records
   - Replace scattered `fetch_req_*` writes with a registered frontend command.
   - Backend sends redirect/restart commands.
   - Frontend produces decode/fault payloads.

3. Add the first I-cache
   - Keep it small and timing-friendly.
   - Make hit lookup the common path.
   - Put refill, page crossing, and invalidation in explicit slow states.

4. Move sequential prediction into the frontend
   - Predict next PC from fetched instruction length.
   - Let backend validate or redirect.
   - Stop growing `frontend_spec_fetch_state` as the main mechanism.

5. Decide restart vs stall with counters
   - Count accepted speculation, flushed speculation, hazard restarts, and
     frontend stalls.
   - Use hardware perf plus counters to decide whether a stall is worth its
     timing cost.

6. Add minimal hazards
   - Start with restart or local stall for true data hazards.
   - Add bypasses only where counters show repeated loss and timing permits.

## Testing Ladder

Every behavioral pipeline commit should pass:

1. `make -C src smolrv64-tester`
2. `make -C src testall`
3. `make -C workloads/linux run` to `Unpacking initramfs...`
4. FPGA timing for timing-risk RTL changes before the commit is treated as
   hardware ready.

Hardware `perf stat sha256sum < /usr/bin/emacs` is the performance arbiter.
Simulation and timing can prove "works"; hardware CPI proves "helped".

## AI Workflow

Use AI for small, falsifiable steps:

- define the stage boundary being changed;
- state whether the patch is structural or expected to improve CPI;
- keep generated Vivado/sim files out of commits;
- inspect timing paths before widening speculation;
- revert quickly when hardware timing or perf contradicts the hypothesis;
- rewrite this plan when it starts defending yesterday's implementation.

Subagents are useful only for independent, checkable tasks: running a fixed test
matrix, reading timing reports, or checking a specific invariant.  Do not give a
subagent an open-ended "pipeline the CPU" task.
