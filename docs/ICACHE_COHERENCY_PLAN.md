# SmolRV64 I-Cache Coherency Plan

This note records the planned direction for improving split I-cache/D-cache
coherency. The current design is correct by using conservative flushes, but it
is more expensive than it needs to be.

## Current Contract

- The I-cache is coherent at the architectural `FENCE.I` boundary.
- `FENCE.I` flushes frontend speculation, advances the fetch epoch, and requests
  a full VHPR flush.
- The full VHPR flush walks D-cache indices/ways and invalidates the
  corresponding I-cache index/way tags while clearing or writing back D-cache
  state.
- This satisfies the RISC-V self-modifying-code contract, but it is coarse.

## Goal

Avoid full I-cache invalidation when a known physical cache line is the only
line that needs instruction-side coherence.

The preferred next step is targeted I-cache invalidation by physical line, not
live I-cache line update.

## Why Not Update I-Cache Lines First

Updating an I-cache line from D-cache activity is possible, but it has a larger
hazard surface:

- I-cache hits are currently virtual-tag lookups with ASID/context checks.
- Coherency decisions need physical-line matching, which may require probing
  colored indices and both ways.
- Live updates need a write path into frontend-owned I-cache data arrays.
- Updates must arbitrate against instruction fetch, fills, invalidations, and
  frontend-buffer state.
- Partial D-cache stores would need precise byte merge behavior in the I-cache
  banks, including split-bank cases.

Invalidation avoids those problems. Once the I-cache line is invalidated, the
next fetch refills from coherent backing memory or the D-cache/writeback path.

## Proposed Incremental Design

1. Add a frontend-owned physical-line invalidate request.

   Payload:

   - physical line tag;
   - physical line index bits;
   - operation valid/ready or a simple blocking request/ack while the shared
     cache FSM is still blocking.

2. Add an I-cache physical probe path inside the frontend.

   The probe should compare stored physical tags, not virtual tags. If the
   current skew/coloring scheme means a physical line can reside in multiple
   possible indices, probe each possible color/index across both ways.

3. Invalidate matching I-cache tags.

   Do not modify I-cache data banks. Clearing the matching tag is sufficient.
   Existing fill logic will repopulate the line on demand.

4. Use targeted invalidation for known physical-line maintenance.

   Good first users:

   - CBO operations with a translated physical line;
   - D-cache alias eviction or physical-line reconciliation paths that already
     know the line tag/index.

5. Keep `FENCE.I` as full flush initially.

   `FENCE.I` has no address operand. Making it selective requires tracking
   which physical lines had stores since the last `FENCE.I`, or snooping every
   store into a pending-invalidation structure. That should be a later change.

## Later Selective FENCE.I Options

Option A: dirty/since-fence line tracking.

- Track physical cache lines written by stores since the last `FENCE.I`.
- On `FENCE.I`, invalidate only matching resident I-cache lines.
- Clear the tracking structure after completion.

Option B: small pending invalidation queue.

- On each cacheable store, enqueue the physical line if not already present.
- On `FENCE.I`, drain the queue through the targeted I-cache invalidate path.
- Fall back to full flush if the queue overflows.

Option C: store-time I-cache invalidation.

- On each cacheable store, invalidate any matching I-cache line immediately.
- This is simplest conceptually but may add store-hit latency or contention with
  frontend fetch unless the request can run in parallel.

## Non-Goals For Now

- Do not add live I-cache data updates from D-cache stores yet.
- Do not add I-cache snooping on every D-cache write as the first step.
- Do not make `FENCE.I` selective until targeted physical invalidation exists.
- Do not expose I-cache bank or way internals back to the backend to implement
  this; the frontend should own I-cache probing and invalidation details.

## Testing Plan

- Keep the existing `rv64si-p-icache-alias` test passing.
- Add a directed self-modifying-code test that writes an instruction, executes
  `FENCE.I`, and verifies the new instruction is fetched.
- Add a test where only one physical line is invalidated while neighboring
  I-cache lines remain valid.
- Add a stress test for virtual aliases mapping to the same physical line.
- Run Linux boot after the first targeted invalidation implementation, because
  instruction-side coherency bugs can appear late.

## Current Recommendation

Punt implementation for now. When resumed, implement targeted physical-line
I-cache invalidation first. Keep full `FENCE.I` flush as the correctness
fallback until selective line tracking is designed and tested.
