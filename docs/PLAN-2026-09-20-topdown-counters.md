# Plan: a closing top-down counter set (2026-09-20)

The simulator's per-cycle accounting closes, but the board's counters cannot. This plan adds the
smallest set of RTL counters that makes top-down Level-1 an exact partition (sums to 100% of
cycles), then the one slot-level counter that makes Retiring and the wasted-issue real, then
the depth probes, landing each via the standing gates. Status is kept in this file.

Goal: `tools/perf-cpi-stack.py --topdown` reports four buckets — Retiring / Front-End /
Back-End / Bad-Spec — that tile the cycle space by construction, asserts that they do, and
drills down per bucket. Nothing about the report is fabricated or caveated; a "…sums past
100%" note on the back-end disappears because the back-end is a real, exclusive cycle charge.

> **Two variants are on this page.** Everything below the title and up to the `---` before
> `## Increments` is **Variant A** — a cycle-charged exclusive priority encoder in RTL (the
> back-end is measured). The appendix at the bottom, **"Variant B: Icicle-style slot
> primitives"**, is an alternative that gets closure with per-slot primitives and
> classification in software (the back-end is **inferred**). Pick ONE to build; B cancels
> most of A. The status block at the end carries the decision.

## Why the current report cannot close

The `0x03xx` taps are *wait-cycle multipliers*, not a cycle partition (`core/smolrv64_core.v`,
stall-attribution block ~2592-2722):

- `ST_DIV`/`ST_MUL`/`ST_FPU` are **port occupancy** (`md_v`, `f_valid & ~fp_disp`) — since C1
  they count every busy-unit cycle, which overlaps dispatch and retire.
- A cycle can be **charged in more than one dependency event at once** — the RTL says so
  (line ~2617: "the stack sums past 100%"): a load blocked AND an FP result pending AND a
  full scheduler count once each.

So there is no partition to sum, and `Back-End Bound = ST_MEM+ST_DIV+ST_MUL+ST_FPU+ST_DSP+ST_ROB`
reads 116% on a real GB5 run. Fixing it is not a tool change; it needs **one** new tap that
charges each cycle to *the* dominant cause by a priority encoder.

## The accounting that closes (grounded in the RTL wires)

Let `be_q = ~redirect & ~rd_wait` (a "not resolving a mispredict" qualifier). Every cycle
falls into exactly one bucket:

```
BADSPEC = redirect | rd_wait
FRONTEND = be_q & fe_bub                # fe_bub = ~st_m & ~d_valid & ~redirect
BACKEND  = be_q & ( st_m | (d_valid & ~d_take) )
PRODUCED = 1 − (BADSPEC + FRONTEND + BACKEND)          # sustained dispatch; closed with cycles
```

`d_take` is slot A's dispatch (`d_valid & ~d_hold & ~redirect_q & ~fr_v & ~dec_red_q`); the
take signals are `d_take | d2_take | d3_take` for "dispatched this cycle."

Disjoint and exhaustive — the full truth table over {`rd_wait,redirect,st_m,d_valid,d_take`}:

| state | bucket |
|---|---|
| `redirect \| rd_wait` | **BADSPEC** |
| `~st_m & ~d_valid` (−badspec) | **FRONTEND** (frontend supplied nothing) |
| `st_m` (−badspec) | **BACKEND** (M holds an op the pipe can't finish) |
| `~st_m & d_valid & ~d_take` (−badspec) | **BACKEND** (a uop present, dispatch held) |
| `~st_m & d_valid & d_take` (−badspec) | **PRODUCED** |

The three state guards don't overlap (each is on a different combination), and together with
`badspec` they cover all of `[1:0]` — so BADSPEC + FRONTEND + BACKEND + PRODUCED = cycles,
exactly, by construction.

**The headline: only ONE new counter is required to close Level-1** — `TD_BE`, the exclusive
back-end. Everything else already exists:
- BADSPEC = existing `REDIR` (r0005) + `RD_WAIT` (r0317). They are already disjoint
  (`rd_wait = fr_v & ~cf_red_fire`, so the drain is exactly the not-yet-firing window).
- FRONTEND = existing `FE_BUB` (r0310), gated `& ~rd_wait`.
- PRODUCED = inferred (`cycles − ...`), no counter.

## New events (codes in the free `0x04xx` range; `0x02xx` is perf BUS_CYCLES, `0x03xx` used)

| event | code | exact definition | source |
|---|---|---|---|
| `TD_BE` | 0x0402 | `~redirect & ~rd_wait & ( st_m \| (d_valid & ~d_take) )` | **new — the closure tap** |
| `TD_BE_MEM` | 0x0403 | `be_q & ( (m_valid & ~m_done & m_mem_op) \| (d_valid & ~d_take & dep_ld) )` | new; core = `TD_BE − TD_BE_MEM` (inferred) |
| `TD_BE_ROB` | 0x0404 | `TD_BE & st_rob` | new; ROB-full, the big core structural (reads 0 on FP-bound loads — see spec §11) |
| `TD_BE_IQ` | 0x0405 | `TD_BE & (st_iq \| st_sq \| st_lq)` | new; scheduler/queue full |
| `TD_FE_LAT` | 0x0406 | `TD_FE & (fe_mmu \| fe_ic)` | new; front-end **latency** (iMMU walk / no window); shares sum to TD_FE |
| `TD_FE_BW` | 0x0407 | `TD_FE & (fe_aln \| fe_que)` | new; front-end **bandwidth/restart** (aligner / queue empty) |
| `DPATCH` | 0x0408 | `d_take + d2_take + d3_take` (incremented by the count) | new; uops dispatched/cycle — the slot-level key |

Depth provenance note: on the dispatch-hold branch the RTL already decomposes
`st_rob / dep_ld / dep_fp / st_iq / st_rn / st_sq / st_lq / st_srz` into a **disjoint** priority
(`st_dsp` in `d_hold`'s order), so those depth counters add within `TD_BE` cleanly. Only the
M-stall branch (`st_m`) and the typed occupancy (`f_valid&~fp_disp`, `md_v`) overlap with the
dispatch branch — acceptable, because depth lives under an already-closed parent.

`TD_BSPC` = `redirect | rd_wait` and `TD_FE` = `FE_BUB & ~rd_wait` can be **reconstructed
from existing counters**, so they are listed for the spec/JSON only, not as new mux bits.

## Array-width audit (rule G4 / Ram manifest)

`DPATCH` is the only new counter that increments by a count rather than a pulse. It is a
`hpm_ev`-parallel sum `d_take + d2_take + d3_take`, identical in shape to the existing
`hpm_ret_q = retire + retire2 + retire3` (line ~2755) — no new tap points, no netlist
geometry change. All other new events are single-bit `hpm_ev` pulses; they only extend the
registered 39-bit `hpm_ev_q` bus and the `mhpmcounterN` muxes in `csr_file.v`, exactly the
path the C0 "memory backend" taps already took. The 39-bit event bus stays `< HPMN`-sized
easy at 13 counters.

---

## Increments

Each increment: **lint → `core/run-vl.sh` (240/0) → benches (`run-*-tb.sh`,
`src/run-tb.sh`) → 300 M-cycle cosim (`CYC=300000000 core/run-cosim-linux.sh`) →
board gate (`tools/gate.sh`, `login:`, zero faults)**, and the `docs/SmolRV64-Spec.md` §11
counter table is updated in the **same commit** as the RTL change that moves a number.

### Increment 0 — `TD_BE`: Level-1 closes (the one-tap fix)

**Impl.** In `core/smolrv64_core.v` stall-attribution block (~2694): add
`wire td_be = ~redirect & ~rd_wait & (st_m | (d_valid & ~d_take));`, splice it into `hpm_ev`
(~2710) as a new bit, and add `HPMEV_TD_BE = 16'h0402, // exclusive back-end: charge a cycle \
to M-held or dispatch-held, not a mispredict-resolve` to the `csr_file.v` event block. Regen
the JSON (`tools/gen-perf-events.py`), run `lint.sh`.

**Tooling (same commit).** `perf-cpi-stack.py --topdown` gains a **closure path**: when
`TD_BE` is present it reports the four buckets, computes `PRODUCED = cyc − (BAD+FE+BE)`, and
**asserts** `BAD+FE+BE+PRODUCED == cyc` (print the residual; `$fatal`-style nonzero only if
it wanders past a tolerance, so a silently-wrong map shows up as a broken identity, matching
the existing cpi-stack philosophy). Until `TD_BE` exists in a run, `--topdown` keeps the
overlapping wait-share mode — the tool is valid before and after the RTL lands. `perf-smol.sh`
`td` set: drop the now-redundant rack if needed, include `r0402`, keep `r0310,r0005,r0317`.
`docs/SmolRV64-Spec.md` §11: document `TD_BE` 0x0402 and the closure identity.

**Effort.** One `wire`, two mux entries, one CSR line, ~40 lines of tool. Lowest-risk
increment; no timing path (registered instrumentation bus).

**Gate.** `--topdown` on a saved cpi/td run reports a closing Level-1: four buckets summing to
100% within 0.5%, residual printed and green. Board GB5 reproduces the "fills X% of slots"
headline with the back-end as one number that no longer exceeds 100.

### Increment 1 — `DPATCH`: slot-level Retiring and the real wasted-issue

**Impl.** `dpatch = d_take + d2_take + d3_take`, added to `hpm_ev` as a count term (mirror
`hpm_ret_q`, which already sums `retire + retire2 + retire3`), `HPMEV_DPATCH = 16'h0408`.
The event bus gains one count-read entry; `csr_file.v` increments by the count
(`hpm_inc = {3'd0, dpatch}` — 0..3 per cycle), exactly like `INSTRET` uses `hpm_retire_cnt`.

**Tooling.** `--topdown` reports, under PRODUCED: `Retiring = INSTRET/(IW·cycles)` (the
slot-share, as today) and — new — `Wasted issue = (IW·cycles − Σdupatch)/(IW·cycles)`, the
exact cost of a 3-wide machine not saturating its dispatch — previously hidden inside
PRODUCED, now a measured number instead of inferred. This is the "fills 13% of 3 slots" term
made precise.

**Effort.** One count wire + one mux + one CSR line + ~25 tool lines.

**Gate.** On a cached L1-resident loop, `Wasted issue` → the aligner/frontend share; on a
dependency-bound kernel it → ~0 and Back-End takes the slot-fill gap.

### Increment 2 — depth probes (`TD_BE_MEM`, `TD_BE_ROB`, `TD_BE_IQ`, `TD_FE_LAT`, `TD_FE_BW`)

**Impl.** Each is a gated `hpm_ev` pulse as defined above; add the five `HPMEV_*` lines and
mux entries. All are single-bit pulses — no counter-of-counts, no timing concern.

**Tooling.** `--topdown` Level-2/3: Back-End → Memory (`TD_BE_MEM`, core inferred) + Core
(`TD_BE_ROB`, `TD_BE_IQ`, plus the existing `ST_FPU`/`ST_MUL`/`ST_DIV` occupancy); Front-End →
latency vs bandwidth (sum to `TD_FE`), replacing the current per-cause FE_* when present;
Bad-Spec by `RED_BR`/`RED_JLR`/`RED_TRP`. The depth rows are presented under the closed
parent, so they read as mine, not as a re-opened sum.

**Effort.** Five wires + five mux + five CSR lines + ~60 tool lines.

**Gate.** `memcpi`-depth GB5 shows Memory-Bound and ROB-full reconciling with the spec's
measured numbers (§11); no Level-1 row moves from Increment-0's value (depth never recomputes
the parent).

### Increment 3 — tooling consolidation and the board story

`perf-smol.sh`: a single `td` set sized for the full 13 (L1: `TD_BE` + `FE_BUB` + `REDIR` +
`RD_WAIT`; depth: `TD_BE_MEM` + `TD_BE_ROB` + `TD_BE_IQ` + `TD_FE_LAT` + `TD_FE_BW` + `DPATCH`
+ `RED_BR` + `RED_JLR`), and update the header docs to describe a closing Level-1. Keep `cpi`,
`memcpi`, `hold`, `br` as depth-only deep dives. Update `docs/smolrv64-perf-events.json` (via
the generator) and `docs/SmolRV64-Spec.md` §11. Board: `make` the shipping config, boot to
`login:` with **zero faults**, one `td` run that closes to 100%, and the board verdict line.

**Effort.** Script + docs only.

---

## Verification (all increments, standing)

| gate | command | pass |
|---|---|---|
| lint | `src/lint.sh` | `lint: clean` (waivers never global `-Wno-`; they still name a file) |
| riscv-tests | `core/run-vl.sh` (`tests/run-riscv-tests.sh`) | `pass=240 fail=0` |
| unit benches | `core/run-*-tb.sh`, `src/run-tb.sh` | all (port change on the shared `csr_file.v`/event bus → rule G4) |
| cosim | `CYC=300000000 core/run-cosim-linux.sh` | lockstep clean, per §12 |
| board | `platforms/rk-xcku5p-f-v1.2/` `make bit`/`make program`; `tools/gate.sh` | boots Ubuntu to `login:` with zero faults |

## Not in scope (deferred)

- Per-slot *uop* classification (light/heavy retiring, machine-clear vs mispredict split is a
  name, not a new counter) — `DPATCH` already give the aggregate; per-type Retiring needs a
  dispatch-type histogram we do not tap.
- A perf **bottleneck table** / JSON machine-readable top-down output — the analyzer already
  JSON-agnostic; add only if the plot tooling asks.

## Status

- [x] **Variant choice: A** (Tommy, 2026-09-28). The simulator already classified every cycle this
      way and closed, so the counters inherit a working classifier and a validator.
- [x] Increments 0-3, built as one change (2026-09-28, `wip/dcache`):
  - The classifier moved into `smolrv64_core` as `td_k`; `tb_smolrv64_linux`'s `TOPDOWN-SIM` reads it, so
    the counters and the simulator cannot disagree.
  - Level 1 needs **three** exclusive events, not one: `TD_BS` (r0401) and `TD_FE` (r0407) are
    not reconstructable from existing counters -- `FE_BUB` also fires in drain cycles, and a
    redirect can land while a restart waits (`REDIR` and `RD_WAIT` overlap). With `TD_BE`
    (r0402) the dispatching cycles are the remainder.
  - Depth: `TD_BE_MEM`/`TD_BE_ROB`/`TD_BE_IQ` (r0403-5), `TD_FE_LAT` (r0406), each a set of
    `td_k` values; front-end bandwidth is inferred (`TD_FE - TD_FE_LAT`), not a counter.
  - `DPATCH` (r0408) counts dispatched instructions, `d_take + d2_take + d3_take`.
  - `FB_HIT`/`FB_RHIT` (dead since the fetch buffer went) are gone; the bus is 44 bits.
  - The D$/I$ access and miss events count per request, not per replay.
  - `perf-cpi-stack.py` reports the closing top-down when the TD events are present (width 3);
    `perf-smol.sh td` is the 13-event set. Validated end to end: the simulator's `perf-stat-sim`
    output through the tool reproduces `TOPDOWN-SIM` exactly (60 M cycles, IW=3).
- [x] The board run (2026-09-28, 06a88177, board gate PASS): `perf-smol.sh td` closes to 100.00%
      of cycles on the board. On 30 MB of random data:

      | workload | IPC | dispatching | bad spec | front-end | back-end | largest level-2 |
      |---|---|---|---|---|---|---|
      | `sha256sum` | 1.637 | 75.50% | 0.30% | 3.42% | 20.78% | ROB full 10.51% |
      | `xz -6 -T1` | 0.531 | 29.33% | 1.77% | 6.54% | 62.36% | memory 44.37% |

---

# Appendix — Variant B: Icicle-style slot primitives

Alternative design, modeled on the Icicle top-down-microarchitectural-analysis work
(Weingarten et al., IISWC '25, on Rocket/BOOM; RISC-V TMA). Philosophy is the **mirror** of
Variant A:

- **A**: charge each *cycle* to an exclusive bucket with a priority encoder in RTL; back-end
  is *measured*; closure argued by a truth table.
- **B**: expose clean, orthogonal *per-slot* primitives; classify in software; **the back-end
  is inferred** as the residual slot count, so closure is *structural* and cannot fail.

RTL shrinks (the encoder and six depth taps disappear); the software (`--topdown`) becomes a
real classifier plus a trace-based validator. This is the recommended path if the priority
encoder's hand-argued closure ever feels fragile, or if you want reusable primitives instead
of a pre-baked tree.

## Slot accounting (closure by construction)

Denominator is **slots**, not cycles: `total_slots = IW · cycles` (IW = issue width, 3).

```
Retiring  = INSTRET                              (slots — have them)
BadSpec   = Σ over flushes (UOPS_ISSUED − INSTRET) + RECOVERING · IW
FrontEnd  = FETCH_BUBBLE                         (accumulated lanes)
BackEnd   = total_slots − Retiring − BadSpec − FrontEnd      # inferred residual
```

Every term is already in slots; BackEnd is defined as the remainder, so
`Retiring+BadSpec+FrontEnd+BackEnd ≡ total_slots` by construction — the `--topdown` closure
assertion becomes a tautology and needs **no tolerance**, unlike Variant A's 0.5% gate. This
is the property that makes a "sums past 100%" outcome impossible, regardless of how the
underlying ports overlap.

## Primitive event set (codes in the free `0x04xx` range)

| event | code | exact definition | A-cancelled counterpart |
|---|---|---|---|
| `UOPS_ISSUED` | 0x0410 | per-cycle count of **issue-slot valids** (WI signals into the exec ports; selected+woken, not merely dispatched) | cancels `DPATCH` as a dispatch count |
| `RECOVERING` | 0x0411 | pulse per cycle from a **flush** until a valid fetch packet arrives (front-end refill bubbles) | cancels `RD_WAIT`'s drain-cycles role |
| `FETCH_BUBBLE` | 0x0412 | count of decode lanes with "packet valid but lane i did not handshake", **suppressed during recovery / when lane not valid** | cancels `TD_FE` + `TD_FE_LAT/BW` + the four FE_* causes |
| `ICM_BLOCKED` | 0x0413 | cycles a refill is in progress **and** the decoupling buffer is empty (i$ low-level FE) | absorbs `FE_MMU`/`FE_IC` |
| `DCM_BLOCKED` | 0x0414 | issue slots stuck waiting on a **load cache miss** (low-level BE memory-vs-core split) | absorbs `TD_BE_MEM`, `MEM_LDINFL` |
| `FENCE_RET` | 0x0415 | retired `fence`/`sfence.vma` count, to subtract **intentional** flushes from BadSpec | cancels "all redirect is bad-spec" |
| `INSTRET` | (have) | retired uops — the Badspec end-point | — |

Kept from A as optional depth only: `RED_BR`/`RED_JLR` (mispredict cause) and the `ST_*`
occupancy taps — never required for Level-1, which closes on the primitives alone.

Key repointing vs A: **dispatch → issue**. `DPATCH` counted `d_take+d2_take+d3_take`
(dispatched into the scheduler). For the Badspec `issued − retiring` flush math, *dispatched*
over-inflates lost slots because µ-ops sit in issue queues waiting on dependencies and have
not yet passed the branch-in-flight point — exactly the pitfall the paper calls out for BOOM.
Only issue-slot valids are the correct start point; keep `INSTRET` as the end point.

## Fetch-truncation accounting (matches this core)

The 12B → 1-insn case (first insn a correctly predicted-taken branch) is handled the way
Icicle's `FETCH_BUBBLE` is defined: the trailing lanes are **not valid**, so no bubble is
asserted and no slot is charged (the control-flow boundary legitimately ends the block). Only
a *predicted-taken-but-wrong* branch charges it, and then via `UOPS_ISSUED − INSTRET` +
`RECOVERING` under BadSpec — never as a Front-End bubble.

## Cancellation map (which parts of A to drop)

- **A In0** (`TD_BE` + priority encoder + truth table) — **cancelled**; no exclusive BE tap at all.
- **A In1** (`DPATCH`) — **repointed**: becomes `UOPS_ISSUED` (issue slot count, not dispatch).
- **A In2** (5 depth taps `TD_BE_MEM/ROB/IQ`, `TD_FE_LAT/BW`) — **cancelled**; leaner
  `ICM_BLOCKED`/`DCM_BLOCKED` + existing cause taps serve depth.
- **A In3** (tooling consolidation) — **grows**: `--topdown` becomes the classifier; add the
  trace validator. Standing gates otherwise unchanged.

## Variant B increments

Each runs the **same standing gates** as A (lint → 240/0 → benches → 300 M-cosim → board),
and `docs/SmolRV64-Spec.md` §11 moves numbers in the same commit.

### B0 — primitive set + closure classifier
**Impl.** Add the six `hpm_ev` bits above (`UOPS_ISSUED` and `FETCH_BUBBLE` as 0..IW count
sums like the existing `hpm_ret_q`; the rest single-bit pulses), `HPMEV_*` lines, regen JSON.
**Tooling.** `--topdown` reads issued/retiring/fetch-bubble/recovering, computes
`BackEnd = residual`; assert closure = total_slots (no tolerance). `perf-smol.sh` `td` set
rewritten to the six primitives. **Effort.** ~6 wires/CSR lines + ~60 tool lines.
**Gate.** Closure assert can't fail; GB5 Level-1 reproduces the "fills 13% of slots" headline
with BackEnd as the residual.

### B1 — intentional-flush exclusion + recovery refinement
**Impl.** `FENCE_RET` counter; route non-mispredict (trap/`fence`/`sfence.vma`) restarts out
of BadSpec. `RECOVERING` refined to per-slot if the raw cycles-overestimate matters.
**Gate.** A `synch`/fence-dense microbench shows `fence` flushes not inflating BadSpec.

### B2 — depth + Simmerv validation
**Impl.** `ICM_BLOCKED`, `DCM_BLOCKED`; keep optional `RED_BR/JLR` and `ST_*` occupancy rows.
**Tooling.** `--topdown` adds the low-level FE/BE split; add a **trace validator** mirroring
Icicle's: compare `UOPS_ISSUED`/`INSTRET`/`FETCH_BUBBLE` against Simmerv cosim issued/retired/
bubble ground truth on a short benchmark. **Gate.** Validator matches Simmerv within a stated
% ; board boots to `login:` zero faults with one closing `td` run.

## What survives from A unchanged

- The "one or two small clean counters buy closure" spirit (now `UOPS_ISSUED` + `RECOVERING`).
- `DPATCH`'s insight that the slot fill is the real efficiency number — repointed to issue.
- The full standing-gates table and the `spec §11 same-commit` rule.

## Variant B status

- [ ] B0 — primitives + closure classifier
- [ ] B1 — intentional-flush exclusion
- [ ] B2 — depth + Simmerv validation