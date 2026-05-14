# SmolRV64 Pipelining Plan

This document tracks the current pipeline direction.  It is a working plan, not
a constraint: when it conflicts with getting to a real pipeline, rewrite it.

## Goal

Move SmolRV64 from a mostly single-instruction FSM toward a simple in-order
pipeline that overlaps frontend and backend work.

The target is not out-of-order execution or multi-issue.  The target is:

- one instruction retires in order;
- frontend fetch/decode can run while an older instruction is in later stages;
- redirects from branches, jumps, traps, interrupts, and xRET flush younger
  work;
- every commit remains buildable and testable.

This is a stepping stone toward a speculative superscalar out-of-order design.
Prefer clean, concise, human-readable stage contracts over squeezing out every
short-term cycle in the transitional in-order core.

## Pipeline Contract

Use ready/valid boundaries:

- A stage owns a `valid` bit and a registered payload.
- A producer may write a stage only when the stage is not valid or the consumer
  accepts it in the same cycle.
- A consumer may read a stage only when `valid` is set.
- If a stage is valid and not accepted, its payload stays stable.
- Redirects invalidate younger stages.
- Retirement remains in order at the backend.

Do not protect the old FSM shape for its own sake.  Preserve architectural
behavior, not state names.

## Near-Term Architecture

Split the core into two conceptual halves before trying to make everything
overlap:

1. Frontend
   - owns PC generation for the fetch stream;
   - fetches instruction bytes through the existing fetch buffer, cache, TLB,
     PTW, and DRAM paths;
   - expands cross-page and cross-doubleword instructions;
   - enqueues a decoded register-read request.

2. Backend
   - consumes one decoded register-read request;
   - reads integer/FP register files;
   - executes, accesses memory, writes back, and retires in order;
   - sends redirects and flushes to the frontend.

Initially the queues can be one entry deep.  A one-entry queue is enough to
break the current "fetch only after retire" structure and is easier to verify
than a larger instruction FIFO.

## Immediate Steps

1. Centralize fetch-to-decode enqueue
   - Replace the duplicated direct writes to `rf_decode_*`, `rs1`, `rs2`, `rd`,
     and `shamt` with one helper.
   - This is not expected to improve CPI by itself.
   - It creates a single point that can become the frontend/backend queue.

2. Turn `rf_decode_valid` into a real one-entry queue
   - Split enqueue from register-file read launch.
   - Add an explicit `rf_read_*` payload for the instruction whose BRAM
     register-file read is in flight.
   - Do not overwrite a valid decode slot until the backend accepts it.
   - Make `S_RF`/`S_RF2`/`S_RF3` the backend consumer side of that queue.

3. Let the frontend fetch while the backend is busy
   - After an instruction is enqueued, allow the frontend to start the next
     sequential fetch when the decode queue has space.
   - First implementation is intentionally narrow: only consume a speculative
     fetch-buffer hit when the retired instruction resolves to the same
     `npc`, `satp`, and privilege context.
   - Use conservative prediction first: next halfword/word based on the fetched
     instruction length.
   - On any taken branch, jump, trap, interrupt, xRET, `sfence.vma`, or other
     redirecting event, flush the queued younger instruction and restart fetch
     at the resolved target.

4. Add hazards only as needed
   - Data hazards are expected once frontend and backend overlap.
   - Start with a stall policy rather than bypassing:
     if a queued younger instruction reads a pending destination, hold it until
     the older instruction retires or the result is available.
   - Add bypasses later only where perf data justifies the extra timing risk.

5. Reassess the instruction cache
   - A real frontend will otherwise fight the data/cache path.
   - The intended I-cache line size is 64 bytes to match the rest of the memory
     system.
   - Keep awkward unaligned/cross-page cases correct; handle the rare page
     crossing case with a slow path.

## Testing Ladder

Every behavioral pipeline commit should pass:

1. `make -C src smolrv64-tester`
2. `make -C src testall`
3. `make -C workloads/linux run` to `Unpacking initramfs...`
4. FPGA timing before a performance-oriented commit is treated as hardware
   ready.

Hardware `perf stat sha256sum < /usr/bin/emacs` is the performance arbiter.
Simulation and timing can prove "works"; hardware CPI proves "helped".

## AI Workflow

Use AI for small, falsifiable steps:

- define the stage boundary being changed;
- state whether the patch is structural or expected to improve CPI;
- keep generated Vivado/sim files out of commits;
- revert quickly when hardware perf contradicts the hypothesis;
- rewrite this plan when it starts defending yesterday's implementation.

Subagents are useful only for independent, checkable tasks: running a fixed test
matrix, reading timing reports, or checking a specific invariant.  Do not give a
subagent an open-ended "pipeline the CPU" task.
