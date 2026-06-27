# Performance observability for the sharded-OoO core

Goal: make the probe pipeline fully observable so we can attribute every cycle,
build pipetraces, and decide *which* microarchitecture lever to pull — instead of
guessing. Motivated by `sha256sum` on hardware landing at IPC 0.11 (CPI ≈ 9.5): a
dependency-chain-bound kernel where the OoO core performs like a sequential one,
strongly suggesting dependent-op execution latency (no bypass net + ALU latency 2 +
write-before-read PRF) rather than the frontend — but we will *measure*, not assume.

## 0. Two data sources, by design

- **Sim event trace** — the full internal pipeline event stream, DDR model
  *parameterizable*. Gives everything except real DRAM timing.
- **HW HPM** — the real DDR latency distribution (sim's DDR is a synthetic
  line-array, so its latency is whatever we set). The HW histogram feeds *back* into
  the sim DDR model so sim CPI predictions become trustworthy.

## 1. Join model

Every event carries `(clock, seqno, ckpid)` plus payload. Keys:

| key | width | assigned | role |
|---|---|---|---|
| **uid** | 64b, sim-only | at rename, monotonic | canonical pipetrace key — disambiguates the 8-bit `seqno` wrap over a window |
| `seqno` | 8b (SEQW) | at fetch (`seq_q`) | cross-stage hw key; what `rollback`/`commit` reference |
| `ckpid` | 2b (CBITS) | at rename = `create`/`cur` | **back-attributed** to predict/fetch/decode events (they predate it) |
| `mem_idx` | LSU slot | at dispatch | joins the memop sub-events |
| `pdst`/`ps1..3` | PBITS | at rename | links **producer→consumer** for dependency-latency analysis |

`uid` is a free-running counter in the tracer, incremented per renamed instruction in
program order — sim-only, no RTL cost. It is the canonical key; `seqno`/`ckpid` are
recorded attributes (the 8-bit `seqno` wraps every 256 insns, less than a trace
window).

## 2. Per-instruction lifecycle events

| event | RTL observation point | key payload |
|---|---|---|
| **E_PREDICT** | `fetch.v` PC mux (`pc_q`→next) | predicted_npc, taken, source (today always fall-through — emitted anyway, uniform; Phase-0 BP drops in) |
| **E_FETCH** | `fetch.v` imem iface + `aligner` | pc, raw `imem_data`/`avail`, I$ hit/miss, straddle/page-cross |
| **E_DECODE** | `decode_stage` / rename boundary | decoded 32b insn, class bits (branch/load/store/mul/fp/csr/amo), rd/rs1/rs2 |
| **E_RENAME/DISPATCH** | `rename_shard`/`decode_rename`, `create` | **ckpid=cur**, pdst, ps1..3, alloc_ok, stall-reason |
| **E_READY** (wakeup) | `sched_shard` scoreboard set (`wkv`/`wkq`) | cycle each operand's scoreboard bit goes ready |
| **E_SELECT** (issue) | `sched_shard` select (`iss_valid/iss_seq/iss_ckpt`) | lane/shard, mem_idx |
| **E_EXECUTE** | `exec_shard`/`branch_unit` | result-ready cycle (ALU latency 2), branch taken/target/mispredict, AGU addr |
| **E_WRITEBACK** | `rf_shard` broadcast (`wr_valid/wr_pr/wr_val`) | pdst, value |
| **E_COMMIT** (ckp retired) | `commit_ctl` (`cc_commit/cc_commit_idx`) | ckpid, #insns + seqno span |
| **E_SQUASH** (ckp rolled-back) | `backend_top` `roll_v/roll_seq/roll_ckpt` | ckpid, reason (§3) |

"Scheduled" is split into **E_READY** and **E_SELECT** deliberately. The two gaps are
the dependency story:
- `E_WRITEBACK(producer) → E_READY(consumer)` = wakeup latency,
- `E_READY → E_SELECT` = select-bandwidth pressure.

If READY ≈ WB + long and the window is full of not-ready ops → **latency/bypass-bound**.
If many ops are READY but few SELECT → **issue/WB-port-bound**. That distinction
decides bypass-net vs. anything else, and is the first thing to measure.

## 3. Squash reason taxonomy

From `backend_top`'s redirect muxes: branch-mispredict (`eb_redirect`; +predicted-vs-
actual once BP exists), exception/trap (`eb_rtrap`), fetch-fault (`iflt_fire`),
data-fault replay-to-solo (`dflt_*`), sfence/fence.i redirect, interrupt injection.
Each squash gives **wasted work** = insns fetched/executed between the bad bundle and
the redirect.

## 4. Memory/cache sub-events

| event | point | payload |
|---|---|---|
| E_AGU / load enqueue | `lsu.v` load queue | vaddr, mem_idx |
| E_DTLB | `mmu.v` dTLB | hit/miss, PTW start→end cycles |
| E_DC_REQ / E_DC_RESP | `cache.v` | paddr, hit/miss, way, **req→data cycles** |
| E_LD_FORWARD | `lsu.v` store-buf | satisfied-by-store (which) vs cache |
| E_LD_WB | `lsu.v` MERGE | load-use latency, WB lane |
| E_ST_ENQ / E_ST_DRAIN | `lsu.v` store buffer | enqueue; commit-gated drain→L2 cycles (write-through) |
| E_IC_MISS / E_IC_FILL | `soc_top`/`cache.v` | fetch stall on I$ miss |
| E_ITLB / E_PTW | `mmu.v` | iTLB miss, walk cycles |
| E_L2_ARB | `l2_arbiter` | which requestor won, queueing delay |
| **E_DDR_REQ / E_DDR_RESP** | `soc_top` DDR↔AXI bridge | sim-modeled latency; **measured on HW** (§8) |

## 5. Structural / back-pressure counters (why a cycle didn't make progress)

freelist-empty (no physreg) · IQ-full · **checkpoint-full (NCHK exhausted)** ·
store-buf-full · load-queue-full · WB-port conflict · divider/FPU busy (multi-cycle,
non-pipelined) · serialize-solo stall (SYSTEM/AMO/fence issuing only when oldest) ·
dispatch-freeze-on-pending-dfault. These are the denominators of a top-down CPI
account.

## 6. Trace mechanism

- **DPI binary sink**, same pattern as `probe_cosim.cpp`'s `probe_retire`: one
  `perf_event(kind, clock, seqno, ckpid, payload…)` per event into a packed binary
  log; gated by `-DPERF_TRACE` + `+perfwin=<start_cyc>,<len>` so normal runs pay
  nothing.
- **Windowed/triggered**, not whole-run: 700M insns × ~10 events × ~16 B ≈ 100 GB is
  absurd. Capture a steady-state window (a few hundred K–few M insns), triggered by
  cycle range or a software MMIO poke.
- Post-process offline (Python): group by `uid`, sort by clock → per-insn waterfall.

## 7. Derived analyses

Pipetrace (per-uid Gantt) · **top-down CPI breakdown** (each cycle classified:
frontend-bubble / rename-stall / no-ready-ops / select-bound / memory-stall /
squash-waste) · dependency-latency histograms (WB→READY, READY→SELECT) · scheduler
window occupancy over time · issue-width utilization (lanes used / 4) ·
live-checkpoint occupancy (ROB-equivalent) · mispredict & rollback wasted-work ·
load-use latency dist · D$/I$ hit rate · dTLB/PTW cost · serialization cycles.

## 8. HW HPM — DDR latency distribution

- **Observation point:** the DDR↔AXI bridge in `soc_top` (or `l2_arbiter` egress).
  latency = cycles from AR-accept to R (first or last beat — pick one, be
  consistent). Multiple outstanding → tag by AXI ID, match on response.
- **Buckets** via `bucket = clamp(floor(log2(latency)), 0, 6)`:
  1→0, 2-3→1, 4-7→2, 8-15→3, 16-31→4, 32-63→5, 64+→6. Seven counters.
- **Average:** a 64-bit `latency_sum` accumulator (add-by-latency per completion, one
  adder) + `access_count` → avg = sum/count. Optionally min/max.
- **Splits** worth the few extra counters: read vs write, and per-requestor (I$-miss
  / D$-miss / PTW) — very different distributions.
- **Read path:** dedicated MMIO perf-register block (7 buckets + sum + count,
  software-readable), and/or surfaced as `mhpmcounter`s for `perf stat`. Counters off
  the critical path, near-zero timing impact (survives the ~zero-margin FPGA budget).

## 9. The sim↔HW loop

Measure the real DDR histogram on HW → drive the sim behavioral DDR model with a
matching distribution → the sim CPI breakdown (§7) attributes memory stalls
realistically → what-if sweeps (halve DDR latency, add MSHRs, add a bypass net)
entirely in sim.

## 10. Phasing

1. **Sim trace skeleton** — `uid` + the §2 instruction events (predict→commit) +
   E_SQUASH. Gets pipetrace + top-down CPI + WB→READY→SELECT histograms immediately;
   answers the sha256 latency-vs-frontend question. Highest-leverage first slice =
   just the WB→READY→SELECT spacing on the sha256 loop (a small DPI tap on
   `wr_valid/wr_pr`, `wkv/wkq`, `iss_valid/iss_seq`).
2. **Memory/cache sub-events** (§4) + structural counters (§5).
3. **HW DDR HPM** (§8), then close the §9 loop.
