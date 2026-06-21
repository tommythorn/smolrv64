# Frontend Refactor Plan

> **Superseded as the priority (2026-06-17): see `docs/pipelining-findings.md`.**
> This is a refactor sketch (make the frontend boundary explicit). It is NOT the
> CPI blocker — the frontend already runs ahead; the backend discards the lead
> every retire. Useful later for clean stage ownership, not the next move.

This plan treats the current `rf_decode_*` queue as frontend-owned
predecode state, while keeping architectural register reads and operand hazards
in the backend.

## Current Situation

The current frontend organization is split across two places:

- `smolrv64_frontend` owns the fetch buffer, I-cache arrays, instruction
  extraction, fallthrough PC calculation, and static next-PC prediction.
- The top-level core owns `frontend_cmd_*`, the three-state `f_state`, the
  `rf_decode_*` queue, queue pop/match logic, RF read launch, and backend
  hazard checks.

The `rf_decode_*` queue is not a register-value queue. It stores a predecoded
instruction stream:

- `pc`
- `next_pc`
- `predicted_pc`
- `insn`
- `prv`
- `epoch`
- `rd`
- `rs1`
- `rs2`
- `shamt`

Actual register values are read later, after the backend selects the oldest
matching queue entry and drives `rs1`/`rs2` into the BRAM register file.

This means the queue is conceptually a frontend predecode FIFO, but it is
implemented in backend control state.

## Goal

Make the frontend boundary explicit:

```text
frontend:
  fetch buffer / I-cache response window
  static next-PC prediction
  instruction predecode
  predecode FIFO

backend:
  architectural next-PC validation
  redirect / flush / epoch control
  RF read launch
  pending writeback hazard checks
  execute / retire
```

The backend should consume frontend entries through a ready/valid interface
instead of directly manipulating `rf_decode_*` internals.

## Proposed Interface

The frontend should expose the oldest predecoded entry:

```verilog
frontend_decode_valid
frontend_decode_ready
frontend_decode_pc
frontend_decode_next_pc
frontend_decode_predicted_pc
frontend_decode_insn
frontend_decode_prv
frontend_decode_epoch
frontend_decode_rd
frontend_decode_rs1
frontend_decode_rs2
frontend_decode_shamt
frontend_decode_from_ifetch_rsp
```

The backend should control speculation with:

```verilog
frontend_redirect_valid
frontend_redirect_pc
frontend_redirect_prv
frontend_redirect_epoch
frontend_flush
frontend_asid
```

The frontend may also expose queue status for performance/debug:

```verilog
frontend_decode_count
frontend_decode_full
frontend_decode_empty
frontend_decode_high_water
```

## Phase 1: Rename and Document

No behavior change.

1. Rename the conceptual queue in comments from RF/decode queue to predecode
   queue.
2. Add a short lifecycle comment near the queue declarations:

   ```text
   fetch-buffer hit -> predecode FIFO -> backend validates pc/epoch ->
   RF read -> execute
   ```

3. Keep signal names unchanged initially to avoid a large mechanical diff.
4. Add simulation counters:

   - queue enqueue count
   - queue pop count
   - queue high-water mark
   - redirect flush count
   - useful pop count vs. flushed entry count

This phase answers whether the queue fills in real workloads before changing
the structure.

## Phase 2: Extract a Frontend Predecode FIFO

Move the queue storage and enqueue-side decode into `smolrv64_frontend` or a
small helper module instantiated by it.

Move frontend-owned logic:

- `decode_rf_sources`
- `enqueue_rf_decode`
- `enqueue_frontend_decode_hit`
- `frontend_decode_pending_*`
- queue head/tail/count storage
- queue full/empty/high-water tracking

Keep backend-owned logic:

- `load_id_from_rf_decode_head`, rewritten to consume the frontend interface
- `pop_rf_decode_head`, rewritten as `frontend_decode_ready`
- `rf_decode_matches_retire`, rewritten as a comparison against
  `frontend_decode_*`
- `id_no_pending_wb_hazard`
- RF read address driving
- same-cycle writeback bypass

The important boundary is that the frontend may decode register numbers early,
but it must not read architectural register values.

## Phase 3: Pipeline the Fetch-Buffer Hit Path

Replace the current three-state hit sequencer with a small pipeline:

```text
F0: hold/request PC and context
F1: fetch-buffer response, instruction extraction, static prediction
F2: enqueue predecoded entry or hold in skid register
```

The static predictor can remain simple:

- `JAL` predicts target
- `C.J` predicts target
- backward conditional branches predict taken
- forward conditional branches predict fallthrough
- indirect branches stop, fall through, or wait for backend correction

Expected behavior:

- one predecoded enqueue per cycle on sustained fetch-buffer hits
- bubbles on fetch-buffer miss
- bubbles when predicted target is outside the current buffer window
- bubbles on full predecode FIFO
- flush by epoch on redirect/trap/privilege changes

## Phase 4: Keep Miss Handling Conservative

Do not initially make speculative I-cache misses fully independent.

Keep the existing backend-owned arbitration for:

- TLB walks
- I-cache miss requests
- D-cache conflicts
- bus/cache FSM ownership
- instruction access faults

After the pipelined hit path is stable, consider a separate speculative
ifetch request queue. That should be a later change because it affects cache,
TLB, exception, and memory arbitration.

## Phase 5: Optional Predictor Upgrade

Only after the interface and pipeline are stable, consider improving prediction:

- small dynamic conditional predictor
- small BTB for branch and jump targets
- return address stack for returns

These improve the chance that the predecode FIFO fills with useful entries, but
they are not required for the frontend refactor. The first useful step is to
make the existing static-predicted hit path cleaner and more pipelined.

## Correctness Rules

1. The frontend may enqueue speculative predecoded entries, but backend retire
   must validate `pc`, `prv`, and `epoch` before using them.
2. Redirects and traps must invalidate all younger frontend/predecode state.
3. Register values are read only after the backend accepts the oldest matching
   predecode entry.
4. Pending writeback hazards remain backend-owned.
5. Same-cycle writeback bypass remains at the RF-read/execute-request boundary.
6. A queue head payload must remain stable while `valid && !ready`.
7. The frontend must not enqueue into a full FIFO unless a pop occurs in the
   same cycle.

## Minimal First Patch

The least risky first patch is documentation plus counters:

1. Add high-water tracking for the current `rf_decode_count`.
2. Count enqueue/pop/flush events.
3. Rename comments to call the queue a predecode FIFO.
4. Add a simulation summary or perf-event exposure for queue occupancy.

That gives evidence for whether the FIFO is useful before doing the larger
module-boundary refactor.
