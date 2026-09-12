# A conventional, width-parameterized ooo2 front end

Status: design plan, 2026-09-12. Nothing here is implemented.
Scope: `ooo2/` only. `src/` is untouched except where a change is explicitly named as shared.

---

## 1. Why

`ooo2/`'s front end is hard to read and hard to explain. It was authored to close Fmax at
166.67 MHz, and the special cases it grew to do that are each individually justified in a
comment and collectively unreadable:

| special case | where | what it is for |
|---|---|---|
| `apc` / `apred_v` ahead-PC split | `src/fetch.v`, `ooo2/ooo2_predictor.v` | keeps the predictor BRAM address register-only |
| `pnpc_kind` | `src/fetch.v` | 2-bit selector disambiguating which next-PC candidate a registered prediction applies to |
| `lenp` | `src/fetch.v` | 1024×1 untagged RVC-length table so `apc` can advance without decoding |
| straddle precompute + `pc2_q` | `src/fetch.v` | a 32-bit op crossing the window end |
| VA-tagged run-ahead fetch buffer | `ooo2/rv_soc_top.v:597-944` | hides iMMU latency in front of a **PIPT** I$ |

The goal is a front end that is **conventional first**, even at some IPC cost, with priority on
**timing and scalability**, parameterizable by width, targeting 3-wide.

### 1.1 Three facts that shape the plan

**(a) The scalar width is not an accident of the front end, and a front-end-only rewrite
cannot fix it.** `docs/ooo2-plan.md`'s Decisions table records it as settled 2026-08-13:

> | Width | **Scalar, IW=1.** No cross-slot hazard logic; `decode_xslot` unused. |

and it is enforced *below* the front end — `ooo2/ooo2_rename.v`'s header says why:

> SMAP/RMAP + lv[] … **Scalar issue only — at IW>1 SMAP needs multiple write ports, which is
> the same problem one level up.**

with `ooo2/ooo2_iq.v` dispatching one instruction (`d_valid`/`d_ready`) and `ooo2/ooo2_rob.v`
committing one per cycle. `docs/PLAN-2026-09-05-ipc.md` item 10 agrees: *"Two-wide dispatch…
Effort: weeks (rename, IQ and ROB write ports). **Last.**"*

So the width has to be taken on both sides of the F/X queue or neither.

**(b) The conventional, width-parameterized front end already exists in this repo.**

- `src/frontend.v` — `IW` 1–4, default 4, **live** at `src/backend_top.v:515`, swept by
  `src/sweep.sh` via `PROBE_IW`. Its structure is exactly the target: one combinational cloud
  from the PC register through `imem` read, the aligner and `decode_stage`, terminating at the
  decode/rename boundary register inside `decode_rename`. *"So a bundle fetched in cycle T is
  renamed in cycle T+1."*
- `src/decode_rename.v` — already implements multi-slot rename **including intra-bundle slot
  forwarding** (`d_s1_is_slot`, `d_s1_slot`, `d_map_writer`).

These are references to fork, not templates to reinvent. `ooo2/` never edits a `src/` file
(`docs/ooo2-plan.md`: *"anything that must change is forked in"*), so each becomes a fork.

**(c) The two front-end bubble numbers that motivated this work are HW=4 numbers.** This is a
correction, and it changes the priority of the work.

| sha256sum, frontend bubble | HW=4 (`docs/PLAN-2026-09-05-ipc.md:11-21`) | HW=8, shipping (`:145-148`) |
|---|---|---|
| iMMU walking | 0.2% | — |
| no fetch bytes at all | 1.4% | — |
| bytes, but no whole instruction (**straddle**) | **10.9%** | **0.5%** |
| insn ready, F/X queue empty | **30.8%** | **7.8%** |
| whole frontend bubble | 43.4% | **10.1%** |

IPC 0.435 → 0.714 over the same change. The `HW=8` row is the shipping configuration
(`OOO2_HW=8` since 2026-09-05), so **the standalone straddle work is no longer worth a
landing** — the same document already concluded it (*"the leftover-halfword aligner (item 2
step 3) is not worth its cycle at 0.5%"*), and a wider bundle subsumes it anyway.

**(d) The case for `IW>=2` is therefore not the frontend bubble.** It is `docs/OOO2-Spec.md`'s
trace-derived limit study:

> Five workloads reach 1.7-1.9x at **`IW2` with today's 16-entry window**. Superscalar pays
> them immediately, with no ROB growth.

That argument is independent of the front end and survives the `HW=8` fix intact. It is the
stronger footing, and it is the one to use.

---

## 2. Stage 0 — Correct the record

Documentation only. Independent of everything below, landable immediately. Doing it first stops
every later stage from being budgeted against wrong numbers.

### 2.1 111.11 MHz is a debug clock

The shipping clock is **166.67 MHz** — closed 2026-08-24, `probe_clk` WNS **+0.142** on the
`Explore` directive, zero failing endpoints, and validated on hardware
(`docs/HANDOFF-2026-08-24-166MHz.md`; `md5sum` × 8 measured 164.2 MHz, 1.5% under nominal from
tick-granularity accounting).

Policy for the sweep:

- **Every statement of the current/shipping clock becomes 166.67 MHz.** Known offenders, all in
  `docs/ooo2-plan.md`: `:345` (*"111.1 MHz … closes; verified on hardware — current"* — this
  one actively misleads), `:352`, `:373`, `:385`, `:396`. The divider table at `:345` loses its
  stale steps.
- **Every historical 111 MHz figure keeps its number and gets a dated debug-build marker.** It
  is a true record of what was measured; deleting it loses provenance. Sites:
  `docs/HANDOFF-2026-08-18.md:9,11`, `docs/HANDOFF-2026-08-19-timing.md`,
  `docs/HANDOFF-2026-08-20-irq-hang.md`, `docs/HANDOFF-2026-08-23-rename-and-166.md:17,172`,
  `docs/HANDOFF-2026-08-18-fetchbuffer.md:80,88,201`.
- **RTL comments carrying 111 MHz WNS deltas justify real code** and must be re-labelled as
  debug-build measurements, not rewritten:
  - `ooo2/rv_soc_top.v:643-648` — *"cost 0.79 ns of WNS (+0.061 -> -0.728 at 111 MHz)"*
  - `ooo2/ooo2_core.v:125-136` — *"Found on silicon at 111 MHz, first boot, in under a second
    of kernel time"*. The clock is incidental to a silicon-bug narrative; re-label only.
- **Leave the contrastive uses alone.** `docs/HANDOFF-2026-08-24-166MHz.md:48,52,71` cite 111
  MHz as the *proof the clock changed* (*"111.11 MHz would have needed 10.87 s of CPU for the
  same cycle count"*). Those are load-bearing as written.

### 2.2 Stale claims the rewrite would otherwise inherit

| file:line | says | truth |
|---|---|---|
| `ooo2/rv_soc_top.v:7-8`, `:28` | "No width knobs: the core is scalar, so the I$ window is fixed at HW=2 halfwords" | `HW=8` |
| `ooo2/rv_soc_top.v:42-43`, spec §9.2:770 | arbiter has 4 (spec: 5) requesters | `NREQ=2` |
| `ooo2/ooo2_frontend.v:20`, `:24-26` | `HW=2` default; `QDEPTH` documented as 2 | `HW=8` default; `QDEPTH=8` |
| spec §10.1:848 | `q_dat` is 8 × **283** | `QW` = 255 (`ooo2_frontend.v:185`) |
| spec §2:35-57 | F/X queue drawn *after* decode | RTL puts it *before* |
| `ooo2/rv_soc_top.v:851-854` | "mispredicts are ~11.5% of instructions" | 3.4 per 1000 (spec §3.4) |
| `docs/cosim.md`, `docs/COSIM_HANDOFF.md` | document the retired sequential core | stale; `ooo2/run-ooo2-cosim*.sh` is the source of truth |

### 2.3 The spec's largest gap

The fetch buffer (`ooo2/rv_soc_top.v:597-944`, ~350 lines, two `$fatal` invariants, a silicon
diagnostic block, a self-reset path, and a poison rule for in-flight fills) has **no normative
description anywhere**. §2's diagram names it, §4.1 never explains it, §9.1 gives only the read
width, §10.2's array table lists only `btb`/`ycorr`/`ras`/`lenp`. Under `docs/rtl-rules.md` H4
(*"a spec that lags the RTL is worse than no spec, because the next person budgets against
it"*) that gap is itself a defect.

Write it, or delete the buffer in Stage 3 and have nothing to document. **Stage 3 recommends
the latter**, so 2.3 is likely a two-line tombstone plus the VHPR section from `docs/VHPR.md`.

### 2.4 Three verification holes, closed here so later stages can use them

1. **`ooo2/tb_ooo2_riscv.v:14` hard-codes `HW = 2`**, and overrides the core's parameter at
   `:40`. `VDEFS=-DOOO2_HW=8` does not reach it. So **riscv-tests — the verilator flow, the
   iverilog flow, and `run-ooo2-directed.sh` — only ever exercise the front end at `HW=2`: a
   full window every cycle, no I$, no fetch buffer, no partial window.** The shipping width is
   covered only by `tb_ooo2_linux.v`. For a width-parameterizable rewrite this is the biggest
   single hole; parameterizing and sweeping `HW` here is the cheapest coverage available.
2. **`ooo2/run-ooo2-tests.sh`'s iverilog source list predates `ooo2_iq.v` (2026-09-03),
   `ooo2_lq.v` (08-29) and `ooo2_sq.v` (08-28)**, and `ooo2_core.v` instantiates all three.
   `-I.` is an include path for iverilog, not a module search path, so it should not link. It
   is in no gate. Verify or delete — do not trust it.
3. **`RD_WAIT` (r0317) is instrumented but has never been captured.** It exists in the RTL
   (`ooo2/ooo2_core.v:1546-1550`), in `src/csr_file.v`, in `docs/smolrv64-perf-events.json`,
   in the spec table, and in `tools/perf-cpi-stack.py` — but the only saved capture in the repo
   stops at r0316, and no `perf-smol.sh` set includes it. Add it to the `cpi` set (dropping a
   redundant event — 13 counters is the hard ceiling) so Stage 4 has a before/after on the
   redirect path.

---

## 3. Stage 1 — Conventional front end, IW-parameterized, IW=1 bit-identical

### 3.1 Shape

One combinational cloud per cycle from a registered PC through the fetch window, the aligner,
RVC expand and per-slot decode, terminating at a bundle register at the rename boundary. **A
bundle fetched in cycle T is renamed in T+1.**

Fork into `ooo2/` (never edit `src/` — `src/fetch.v` is shared with the live sharded core):

- `ooo2/ooo2_fetch.v` — fork of `src/fetch.v` with the Fmax cases removed
- `ooo2/ooo2_decode.v` — fork of `src/decode_slot.v`, one decode per slot
- `ooo2/ooo2_frontend.v` — rewritten around `IW`

### 3.2 What replaces each special case

| today | replacement |
|---|---|
| `apc` / `apred_v` ahead-PC split | **An explicit fetch-target register.** Flop the predictor's predicted next-PC; fetch from it next cycle. One register, no ahead/real split — and the predictor BRAM addresses stay registered (rule I6) *without* the machinery, because the register is the address. |
| `pnpc_kind` (2-bit selector) | **Disappears.** It exists only to disambiguate which of several next-PC candidates a *registered* prediction applies to. A fetch-target register has no such ambiguity. |
| `lenp` untagged RVC length table | **Decode the real bytes already in hand.** `lenp` exists only so `apc` can advance without decoding; a fetch-target register does no pre-decode advance. |
| straddle precompute + `pc2_q` FSM | The aligned window plus the aligner's **existing** carry. A 32-bit straddler reappears as slot 0 of the next window (`src/aligner.v`'s carry-free windowing already does this); at `IW>=2` the wider bundle absorbs it. |

**Keep unchanged:** `src/aligner.v` (already `IW`-parameterized and carry-free, so it is forked
verbatim or not at all) and the page cap `eff_avail = min(imem_avail, hw_cap)` at
`src/fetch.v:111` (*"page-straddling instructions stay its business"*).

### 3.3 The measurement contract — do not break it

`FE_BUB`/`FE_MMU`/`FE_IC`/`FE_ALN`/`FE_QUE` are computed **outside** the front end, in
`ooo2/ooo2_core.v:1511-1528`, from exactly these facts:

```verilog
wire fe_bub  = ~st_m & ~d_valid & ~redirect;                       // X starved, M not stalled
wire fe_mmu  = fe_bub & ~immu_ready;                                // iMMU walking
wire fe_ic   = fe_bub &  immu_ready & (imem_avail_g == 0);           // window exactly empty
wire fe_rest = fe_bub &  immu_ready & (imem_avail_g != 0);
wire fe_aln  = fe_rest & ~fe_fx_valid;                              // bytes, no instruction
wire fe_que  = fe_rest &  fe_fx_valid;                              // instruction, queue empty
```

- Keep the meanings of **`d_valid`**, **`fe_fx_valid`**, **`imem_avail`** (→ `imem_avail_g`),
  **`immu_ready`**. Everything downstream — the perf scripts, the spec tables, the CPI stack —
  keeps working unchanged.
- `ooo2/tb_ooo2_linux.v:226-284` reads **`dut.core.fe.q_empty`** and
  **`dut.core.fe.u_fetch.pc_q`** by hierarchical name. Renaming or removing either breaks the
  testbench.
- Keep the **`hpm_ev` bit order** at `ooo2_core.v:1551`. `src/csr_file.v` is shared with the
  `src/` core, and `docs/smolrv64-perf-events.json` (lint-gated, generated) and every perf
  script are keyed to it. New events go on the **registered** side (`hpm_ev_q`), never
  combinationally into `csr_file` — that cone was *"823 of 3113 failing endpoints at 6 ns and
  the WORST family in the design."*

### 3.4 Acceptance

`ooo2/run-ooo2-cosim-linux.sh` at `VDEFS=-DOOO2_HW=8 CYC=60000000` returns the recorded retire
count exactly: **18,979,633 ± 0.5%** (`ooo2/cosim-expected.txt`). Because the runner hashes
every `.v` under `ooo2/` and `src/`, a verdict cannot come from a stale binary.

---

## 4. Stage 2 — The F/X queue push→pop bypass

`ooo2/ooo2_frontend.v`:

```verilog
wire ld_valid = ~q_empty;      // evaluated BEFORE the push below
...
if (q_push) begin q_dat[q_wp] <= q_in; q_wp <= q_wp + 1'b1; end
```

A push in the cycle the consumer is ready does not feed it — one cycle lost per push-into-empty.
Fix: `~q_empty | q_push`, selecting `q_in` over `q_dat[q_rp]` on that path.

Justification at the shipping width: `FE_QUE` is **7.8% of cycles on sha256sum** and **24% on
the AES kernel**, and it is *unchanged between HW=2 and HW=4* — it is not an alignment artifact.
Spec §15 P5 states the mechanism:

> the frontend makes at most one instruction per cycle and the backend consumes one per cycle,
> so no hiccup is ever recovered.

**Deliberately not in this stage:** the leftover-halfword aligner. At `HW=8` it is 0.5% of
cycles, it was already judged not worth its cycle, and Stage 4's width subsumes it. Adding a
special case to fix a 0.5% term in a project whose stated purpose is *deleting* special cases
would be self-defeating.

**Acceptance:** cosim retire count up, not down. `febench` unchanged (this is not an alignment
fix, and `febench` is straight-line code that never drains the queue).

---

## 5. Stage 3 — VHPR I$: translation off the fetch hit path

### 5.1 The design already exists in this repo

`docs/VHPR.md` specifies **virtually hit, physically reconciled** L1s in full: hit condition
`valid & epoch == cur_epoch & ASID == req_ASID & vtag == req_vtag & perms_ok`; the physical tag
stored per line but **not** on the hit path; indexing as `ASID-mixed virtual color +
line-within-page offset`; a miss path that enumerates all 8 color positions per way
(2 ways × 8 colors = **16 physical tags**, four per cycle searched in four cycles, overlapping a
four-beat 64-byte fill); and the core invariant

> At most one valid line may exist for any physical cache line within that L1.

Do the **I$ first**: *"The instruction cache never holds dirty lines, but otherwise uses the
same virtual-hit, physical-reconcile rules."* No dirty lines means no synonym-writeback hazard
and no duplicate-dirty-owner problem — the hard half of VHPR is the D$.

### 5.2 A virtual PC needs no branch-target lookup

The front end's PC stays **virtual**. A branch target is a VA, so it needs no translation at
all. This is strictly more conventional than carrying a physical PC: the alternative
option — fetch on physical addresses and translate branch targets on demand — adds a lookup on
exactly the path (`redirect → pc_q`) that the measurement already identifies as the trade's
critical one (`decoupled-frontend-plan.md`'s cone, *"The limiter has now moved four times
without shrinking … every one of them ends at `fe/u_bp`"*).

### 5.3 What dissolves

**The VA-tagged run-ahead fetch buffer.** Its entire reason to exist is to hide iMMU latency in
front of a PIPT cache; with a virtual hit there is nothing to hide. And its retention tag is
measurably worthless:

> `FB_RHIT` / `FB_HIT` = 4.46 M / 1.49 G = **0.3%**

which answers the question the RTL itself poses at `rv_soc_top.v:664-681`:

> DOES THE ADDRESS COMPARISON PAY? … **FB_RHIT near zero means the tag earns nothing and a
> stream buffer is free.**

Deleting it removes, in one change: `fb_pois` (poisoned in-flight fills), the two always-on
`$fatal`s at `:697` and `:901`, the whole FBDIAG diagnostic block, the `imem_avail_g` gate at
`ooo2_core.v:137`, and the `imem_ctx_chg`/`imem_xlate_ok`/`imem_vaddr` port set. That is **five
more instances of the "stale context surviving a control event" generator** that
`docs/rtl-rules.md` counts five of already (*"Five mechanisms for one problem"*) — and the
replacement is a single epoch bump.

### 5.4 What must be kept, and asserted

- The `fence.i` FSM (`rv_soc_top.v:869-880`): `FI_IDLE/FI_DRAIN/FI_INV/FI_WAIT`, with
  `FI_DRAIN` waiting on `dmem_idle & (df == DF_IDLE)` before pulsing the invalidate.
- The VHPR invariant asserted after **each** of allocation, eviction, invalidation, synonym
  migration, writeback, SFENCE.VMA flush, and coherence/DMA probe response.
- **Assert the hazard, not the gate** (rule A6). The existing pair is the model to copy — both
  are phrased as the state that must never hold, not as a restatement of the condition that
  guards it:
  > `rv_soc_top.v:697` — "VA-tagged fetch buffer hit with a STALE mapping: va=%h pa_now=%h
  > pa_cached=%h"

  A check phrased in the same terms as its gate *"has the gate's blind spot, and reports
  success from inside it."*
- The `imem_ctx_chg` invalidation set — satp write / `sfence.vma`, privilege change, `fence.i`
  — becomes an epoch bump, but the *set* must be preserved exactly. Note the two deliberate
  exclusions, which must not be re-added: `mstatus.SUM`/`MXR` gate **data** accesses, not
  fetch; and since `satp_fetch` already folds in the M-mode bare case, comparing it covers the
  privilege change that switches translation off entirely.
- Replacing `ooo2_core.v:137`'s single "may fetch data be consumed" site with per-slot gates
  would be the **C1** defect shape (*"a precondition that applies to N units is computed once
  and applied at one site"*). Keep one site.

### 5.5 Also in scope here, and nearly free

`u_icache.rd_tag` is tied to `4'd0` and `rd_resp_tag` is unconnected, so the I$ response is
matched **by address** (`ic_rd_resp_addr` against `fb_al`/`fb_pa`/`fb_pa1`) — a deliberate,
documented **B1** violation:

> `rv_soc_top.v:951-952` — "The I$ still matches its response by address, in the fetch buffer
> (fb_al/fb_pa1/fb_pa). Named and empty on purpose: converting it is a separate change with its
> own measurement."

`rv_cache` already carries the `RTW=4` plumbing (`rd_tag`/`rd_resp_tag`, `rv_cache.v:57-64`), so
this is a requester-side change. With the fetch buffer gone it is also **unavoidable** — there
is nothing left to do the address match.

### 5.6 Cost and risk

- Touches `ooo2/rv_soc_top.v`, `ooo2/rv_cache.v` (virtual tag + ASID + epoch metadata), the
  iMMU's placement, and the invalidation path. The D$ is left alone.
- **It invalidates the buffer-hit mispredict measurements.** Today a mispredict restarts in 4
  cycles on a fetch-buffer hit and 5 on an I$ hit (`READ_LATENCY=1` plus the combinational
  arrival bypass — the delta is exactly one). After Stage 3 those become uniform. Re-baseline
  before Stage 4.
- The correctness surface is stale-alias and synonym reconciliation. Cosim does catch this
  class: `docs/COSIM_HANDOFF.md`'s divergence #1 was a `satp` trampoline, fixed by
  *"Serialize fetch on a value-changing SATP write"* — the same hazard the `$fatal` at
  `rv_soc_top.v:694-697` guards in RTL today. Cosim aborts at the first architecturally wrong
  instruction, and a store to the wrong PA aborts at the store.

---

## 6. Stage 4 — Widen the backend to `IW`

Introduce `OOO2_IW` (default **1**, so this stage lands inert). Land `IW=2` first; `IW=3`
follows.

| structure | today | change |
|---|---|---|
| SMAP (`ooo2_rename.v`) | 1 write port | N write ports, **as flops, not LUTRAM** — per arch register `i`: `if (wr0_v && wr0_a == i) smap[i] <= wr0_d;` … later slot wins by `if` order. Cost 64 × N × (6-bit compare + 9-bit mux); the array is 576 bits today. This is precisely what `ooo2_rename.v`'s header calls out. |
| intra-bundle deps | none | **Alias resolution**: slot k's `ps_k = (an earlier slot writes rs_k) ? that slot's pdst : lv[rs_k] ? smap[rs_k] : rmap[rs_k]`. Path-independent, so it composes with a rewinding speculative map. `src/decode_rename.v` already has this; `docs/Area-Efficient-Scalar-OoO.md` §14.4 names it as the structure that matters at width (*"past a certain width the interesting structure is the intra-group dependency bypass rather than the map at all"*). |
| RMAP | 1 write port | N write ports (committed state, N commits/cycle). |
| free lists `fl_ie`/`fl_ld`/`fl_fe` | 1 head/tail | N reads (head … head+N−1) and N pushes per cycle. **Head-move only** — `Area-Efficient-Scalar-OoO.md` Appendix A: *"**Do not widen the walk.** Undoing k renames per cycle needs k write ports on map."* |
| `ooo2_iq.v` dispatch | `d_valid`/`d_ready` | N ports. The module's own claim — *"nothing here scales with N², and nothing compares age"* — makes this port replication, not restructuring. |
| `ooo2_rob.v` commit | 1/cycle | N/cycle, **and the `irr` pointer must advance N/cycle** or it becomes the bottleneck. `irr` is what commits stores (`sq_k_take` when the entry's index reaches `irr_idx`), so a 1/cycle walk would gate the store queue. |
| `ooo2_pending.v` | 512×1, 3R + 1 set + 3 clear | scales with N. |
| PRF `mem_ld` | 3R/1W, **two writers** | Already flagged in spec §10.1: LD takes a landing load *and* M's mul/div, so it needs an arbiter or mul/div gets its own shard. *"It costs nothing today because `m_done` is forced low on `ld_land`/`fp_land`, and an assertion fires the moment that stops being true."* This is where widening breaks. |
| redirect / rollback | one target, `lv[*] <= 0` | Still one target, but the flush must squash **younger slots inside the same bundle** — a taken branch in slot k kills k+1…N−1. This is the "cross-slot hazard logic" the 2026-08-13 decision avoided, and it must come back. |

### 6.1 Honest sizing

`PLAN-2026-09-05-ipc.md` item 10 calls two-wide dispatch *"weeks (rename, IQ and ROB write
ports). Last."* That assessment predates the front-end work, but it is not wrong about rename
and the ROB. **Budget `IW=2` as the real deliverable and `IW=3` as a follow-on**, and expect
the PRF write-port collision (`mem_ld`) to be the first thing that breaks.

### 6.2 Harness

`src/sweep.sh` sweeps the *other* core (`PROBE_IW` × `PROBE_CKMAX` × `CACHE`, against
`src/sweep-expected.txt`). ooo2 needs its own `ooo2/sweep.sh` over `OOO2_IW ∈ {1,2,3}` with an
expectations file, mirroring that structure — including the `E3`/`WIDTHEXPAND` discipline,
because `$clog2`-derived `imem_avail` widths have already paid for truncation bugs once
(`rv_soc_top.v:925-944`).

---

## 7. Stage 5 — Measure the Fmax cost, re-derive only what is needed

Stage 1's purpose was to delete special cases and **learn their price**. Write the conventional
structure without the Fmax patches, measure at 6 ns, then re-add only what the measurement
demands — each survivor carrying a one-line reason at the site and an entry in the spec.

- **Instrument:** `cd platforms/rk-xcku5p-f-v1.2 && make census` — every endpoint under
  +0.35 ns grouped into families. The current near-critical census already shows
  **`fe/u_fetch` = 129 endpoints**; that family's size is the before/after metric.
  `make timing` gives `probe_clk` WNS.
- **Rule I2 applies:** *"A single build's WNS cannot judge a change smaller than the placement
  noise."* Take two directives (`Explore` and `AltSpreadLogic_medium`), as the 2026-08-24
  handoff did — one number is a sample.
- **The most likely regression is dropping the ahead-PC split.** The measurement to beat:
  *"that head was ~2.9 ns of a 6.06 ns path"* (2026-08-20, at 166.67 MHz). If it bites, the fix
  is a **deeper fetch pipeline** — a real structural answer — not a reinstated selector.
- **`make ooc` silently measures defaults** (it once reported a 128 KB, 2048-set cache nothing
  instantiates). Pass `GENERICS=` or you are measuring an imaginary design.
- **Then re-baseline everything:** `ooo2/cosim-expected.txt` (both rows), the `febench` six
  numbers, and the spec's CPI-stack figures.

---

## 8. Verification

**Per stage, cheapest first.** `tools/gate.sh` globs the `*-tb.sh` set automatically, so new
unit testbenches are picked up without editing it.

1. `ooo2/run-ooo2-iq-tb.sh`, `-lqsq-tb`, `-lqsq-rand-tb`, `-cache-tb`, `-vnet-tb`,
   `-cbozero-tb` — seconds each.
2. `src/lint.sh` → **`lint: clean`**. It lints ooo2 via a second top (`rv_soc_top`) and globs
   `ooo2/*.v` minus `tb_`, so new files are covered automatically. **`PINMISSING` is now a hard
   error**, so an orphaned new module with unconnected ports fails lint. Waivers are
   file-scoped in `src/verilator.vlt` and must name a file; **never a global `-Wno-`**.
3. `ooo2/run-ooo2-vl.sh` → **`pass=240 fail=0`** (this is *this* core's riscv-tests count).
4. **`src/run-vl-tests.sh` → `failures: 0` whenever a shared `src/` file is touched** —
   `fetch.v`, `aligner.v`, `rvc_expand.v`, `predictor.v`, `decode_*.v`, `mmu.v`, `csr_file.v`.
   Precedent: the 2026-08-24 handoff ran it because *"`src/fetch.v` is shared; the OoO frontend
   leaves `.apc()` unconnected."* Stage 4's width parameter is exactly this class.
5. `CACHE=1 src/run-vl-tests.sh` for cache-path changes (Stage 3). `rv64si-p-dirty` is a known
   pre-existing failure there.
6. `ooo2/run-ooo2-directed.sh` — and **add a front-end regression if the rewrite fixes something
   riscv-tests cannot reach** (a straddle at a page boundary, a redirect into a held window,
   `fence.i` against the buffer). That directory's charter says so explicitly: *"each test here
   exists because a real defect hid behind that gap."*
7. `CYC=60000000 ooo2/run-ooo2-cosim-linux.sh` — **the lockstep oracle.** Aborts at the first
   architecturally wrong instruction with a 320-deep DUT/REF history ring, and enforces the
   `cosim-expected.txt` floor (currently **18,979,633 ± 0.5% at HW=8**). `CYC=300000000` is the
   required length before shipping (spec §12).
8. `ooo2/run-ooo2-cosim-gb5.sh` per batch.
9. Timing: `make census`, `make timing`, `make ooc GENERICS="…"`.
10. Board: `tools/board-gate.sh`; `tools/perf-smol.sh cpi|fb | tools/perf-cpi-stack.py`.
11. `src/sweep.sh` for shared width/checkpoint behaviour; the new `ooo2/sweep.sh` for
    `OOO2_IW`.

### 8.1 The front-end microbenchmark

`workloads/febench` is purpose-built: three blocks of 4096 independent 32-bit ALU instructions
with no loads, stores or branches inside, run 64 times, cycles and instructions read around
each. *"IPC = the fetch rate, since dispatch is one per cycle and nothing stalls."*

```
make && FW=$(pwd)/febench.bin ../../ooo2/run-ooo2-linux.sh
```

Six recorded numbers to beat — the sharpest before/after target for Stages 1–3:

| block | HW=4 | HW=8 |
|---|---|---|
| 32-bit, 4-byte aligned | 0.66 | **0.98** |
| 32-bit, 2-byte offset | 0.40 | **0.79** |
| compressed | 0.99 | 0.99 |

Second instrument: `ooo2/tb_ooo2_linux.v`'s `SB-SIZING X EMPTY` / `F/X QUEUE HAS WORK` census.

### 8.2 Discipline

`docs/HANDOFF-2026-08-18-fetchbuffer.md` records six wrong calls on this exact subsystem with a
single through-line: *"every wrong call came from reasoning across non-matched measurements."*
Match the configuration (`OOO2_HW`, `CACHE`, `MEM_LG2`, `CYC`) across every compared pair, and
note that **the width you measure at determines which unit looks like the bottleneck** — at
`HW=2` the RVC aligner looks like the largest stall in the machine; at `HW=4` it is a minor
term. Never build the `HW=2` point.

---

## 9. Ordering

Stage 0 → 1 → 2 → 3 → 4 → 5, each landing and verified on its own.

- Stage 0 is documentation-only and can go immediately.
- Stage 1's deliverable is a **bit-identical IW=1 front end**, so it carries no IPC risk.
- Stages 3 and 4 are independent. Stage 3 first: a simpler fetch path is easier to replicate
  N-wide than a complicated one.
- Stage 2 is small and independent; take it whenever a landing slot is free after Stage 1.

## 10. Spec updates

Part of each commit, per `docs/rtl-rules.md` H4. Sections in the order the stages reach them:

**§2** (pipeline — move the F/X queue ahead of decode), **§2.2**, **§3.1** (stall taxonomy),
**§3.4** (restarts — the redirect path changes in Stage 3), **§4/4.1** (fetch — the ahead-PC
split, `pnpc_kind` and `lenp` all disappear), **§8.1** (translation leaves the fetch hit path),
**§9.1** (the I$ row: PIPT → VHPR), **§9.2** (arbiter requesters, 4/5 → 2), **§10.1** (`q_dat`
283 → 255; the `mem_ld` two-writer arbiter), **§10.2**, **§10.3** (the I$ paragraph), **§11**
(counters — add `RD_WAIT` to the event sets), **§12** (verification — parameterize
`tb_ooo2_riscv.v`), **§13** (FPGA), **§14** (rewrite the `IW=1` bullet), **§15** (P5 closes;
P2 `head_block` and P7 rename walk-back are re-scoped by Stage 4).

Plus §2.3's replacement: the fetch-buffer section becomes the VHPR I$ section, drawn from
`docs/VHPR.md`.
