# Frontend rewrite: conventional, width-parameterized

Status: design plan, 2026-09-13. Supersedes `PLAN-2026-09-12-frontend.md`, which was written
against the pre-release tree. Nothing here is implemented.

## Why

The `ooo2/` front end is a pile of Fmax special cases that are individually justified and
collectively unreadable:

| special case | where | for |
|---|---|---|
| `apc` / `apred_v` ahead-PC split | `src/fetch.v`, `ooo2/ooo2_predictor.v` | keep the predictor BRAM address register-only |
| `pnpc_kind` (2-bit selector) | `src/fetch.v` | disambiguate which next-PC a registered prediction meant |
| `lenp` (1024×1 RVC-length table) | `src/fetch.v` | let `apc` advance without decoding |
| straddle FSM + `pc2_q` | `src/fetch.v` | a 32-bit op crossing the window end |
| VA-tagged run-ahead fetch buffer | `ooo2/rv_soc_top.v` (~350 lines) | hide iMMU latency ahead of a PIPT I$ |

**Priorities, in order.** (1) `probe_clk` **≥ 166.667 MHz is a hard requirement**, never traded
for anything. (2) **IPC is the primary objective** among the designs that meet it. (3) A
conventional, readable, width-scalable (target 3-wide) front end is the *motivation* for the
rewrite, but it buys neither of the above: if going conventional would cost the clock, the spike
below stops it; if it would cost IPC, that is a regression to be bought back, not accepted.
Every special case removed here was added for **timing**, not IPC — the fetch buffer's
forward-only slide even *costs* IPC — so the removals are IPC-neutral or better, and the only
risk they carry is frequency.

## Ground truth (this tree, post-release)

- **One core.** The sharded core was deleted in the 2026-09 release; nothing outside `ooo2/`
  uses `src/fetch.v`, `aligner.v`, `predictor.v`, `decode_*.v`. **Edit them in place — no
  forks.** (The old plan's fork strategy and its `ooo2_fetch.v`/`ooo2_decode.v` are dropped.)
- **References live in git history, not the tree.** The conventional width-parameterized front
  end `src/frontend.v` and the multi-slot renamer `src/decode_rename.v` (with intra-bundle slot
  forwarding) were deleted in the release. Recover them read-only from before `384fd599`:
  `git show 384fd599~1:src/frontend.v`. They are the shape to copy.
- **`src/aligner.v` is already `IW`-parameterized and carry-free.** A 32-bit straddler reappears
  as slot 0 of the next window with no FSM. Keep it; the straddle special case is redundant.
- **Preserve two 2026-09-10 fixes** through any rewrite — they are correctness, not Fmax cruft:
  the corrector tag folding in its index PC bits (rule B9), and the aligner's solo-op test
  gated on halfword presence.

## The load-bearing risk, proven first

The special cases exist because the conventional structure **did not close 166.67 MHz**: the
ahead-PC head was 2.9 ns of a 6.06 ns path (2026-08-20). The plan's bet is that a **deeper fetch
pipeline** buys that back conventionally. That bet decides everything, so prove it before
building Stages 1–4:

> **Spike.** Write the conventional fetch cloud (registered PC → window → aligner → RVC → decode
> → rename-boundary register) with a fetch-target register in place of `apc`, add one pipeline
> stage on the predictor read, synthesize out of context at 6 ns under two directives
> (`Explore`, `AltSpreadLogic_medium` — rule I2). **Go/no-go:** WNS ≥ 0 on both. If it fails, the
> answer is a still-deeper pipeline, never a reinstated selector. If it cannot be made to close
> at 166.667 MHz, the special cases stay — the clock is never lowered to ship a prettier or more
> conventional front end.

## Stages

Each lands and is verified on its own. `OOO2_IW` defaults to **1**, so Stages 1–3 are
bit-identical to today at the shipping width and carry no IPC risk.

**Stage 0 — correct the record (docs only, immediate).** The release already quarantined the
111 MHz claims under `docs/history/`, closed the `run-ooo2-tests.sh` hole (deleted), and
`RD_WAIT` is now captured (GB6, r0317). Residual: fix the `rv_soc_top.v:7` header ("No width
knobs / HW=2 halfwords" → HW=8), the spec's `q_dat` width (283 → 255) and arbiter count
(`NREQ=2`), and **parameterize `tb_ooo2_riscv.v` (line 13 hardcodes `HW=2`)** so riscv-tests can
run the shipping window — the one real verification hole left.

**Stage 1 — conventional fetch, `IW`-parameterized, `IW=1` bit-identical.** One combinational
cloud, PC register to rename-boundary register, T→T+1. Replace: `apc`/`apred_v` → one
**fetch-target register** (flop the predicted next PC, fetch from it; the register *is* the
BRAM address, so rule I6 holds with no ahead/real split); `pnpc_kind` → gone (no ambiguity);
`lenp` → gone (decode the bytes in hand); straddle FSM → the aligner's existing carry. Keep the
page cap `eff_avail = min(imem_avail, hw_cap)`.
*Acceptance:* `ooo2/run-ooo2-cosim-linux.sh` at `HW=8 CYC=60000000` returns the recorded count
(`ooo2/cosim-expected.txt`: 14,301,801 ± 0.5%).

**Stage 2 — F/X queue push→pop bypass.** A push into an empty queue does not feed a ready
consumer: one cycle lost per push-into-empty. `FE_QUE` is 7.8% of sha256sum cycles, 24% on the
AES kernel, and unchanged HW=2→HW=4, so it is not an alignment artifact. Re-derive against the
current **two-bank** `ooo2_frontend.v` (the two-wide work replaced the single-bank queue the old
plan quoted). *Acceptance:* cosim count up; `febench` unchanged (straight-line, never drains).

**Stage 3 — VHPR I$: translation off the fetch hit path (the prize).** Make the I$ virtually
hit (`docs/VHPR.md`; hit = `valid & epoch & ASID & vtag & perms`, physical tag off the hit
path, invariant: at most one valid line per physical line). The PC stays **virtual**, so branch
targets need no translation. This **deletes the fetch buffer** and everything it drags in:
`fb_pois`, the two STALE-mapping `$fatal`s, the FBDIAG block, the self-reset path, the
`imem_ctx_chg`/`imem_xlate_ok`/`imem_vaddr` ports — replaced by one epoch bump. Its retention
tag earns nothing (`FB_RHIT/FB_HIT = 0.3%`), and its forward-only slide is what makes a
predicted loop back-edge refetch the I$ every iteration (brbench, 2026-09-10). Do the **I$ only**
(no dirty lines; the D$ is the hard half, left alone — it is IPC item 9). Keep and assert: the
`fence.i` FSM, the invalidation set (satp/`sfence.vma`/priv/`fence.i`) as an epoch bump, and
the VHPR invariant after every alloc/evict/inval/writeback/flush; convert the I$ response match
from address to the `rd_tag` the requester allocates (rule B1, now unavoidable). *Acceptance:*
cosim clean at `CYC=300000000`; re-baseline the mispredict-cost numbers (buffer-hit vs I$-hit
delta disappears).

**Stage 4 — widen the backend to `IW`.** Land `IW=2` first, `IW=3` after. The write-port work,
in order of what breaks first: SMAP → N write ports **as flops** (per-arch-reg compare-and-mux,
later slot wins by `if` order); **intra-bundle dependency bypass** (slot k's source = an earlier
slot's pdst else the map — the structure that matters at width, `Area-Efficient-Scalar-OoO.md`
§14.4); RMAP and free lists N-wide, **head-move only, never widen the walk**; `ooo2_iq` and
`ooo2_rob` dispatch/commit N/cycle **and the `irr` pointer N/cycle** or it gates the store queue;
`ooo2_pending` N-wide; the redirect must squash younger slots in the same bundle (the cross-slot
hazard the scalar decision avoided). **First thing to break: the `mem_ld` PRF port** — it already
has two writers (a landing load and M's mul/div), asserted safe only because `m_done` is forced
low on `ld_land`. Budget `IW=2` as the deliverable, `IW=3` as follow-on. Needs `ooo2/sweep.sh`
over `OOO2_IW ∈ {1,2,3}` with an expectations file.

**Stage 5 — re-derive only what timing demands.** After each stage, `make census` + `make
timing` (two directives). Re-add a survivor only with a one-line reason at the site and a spec
entry. Then re-baseline `cosim-expected.txt`, the six `febench` numbers, and the spec CPI stack.

## Ordering

Spike → 0 → 1 → 2 → 3 → 4 → 5. Stage 0 is immediate. Stages 2 and 3 are independent of each
other; Stage 3 before Stage 4 (a simple fetch path is easier to widen). The spike gates the
whole plan.

## Gates (per stage, cheapest first)

`ooo2/run-ooo2-*-tb.sh` → `src/lint.sh` (`lint: clean`, PINMISSING is a hard error) →
`ooo2/run-ooo2-vl.sh` (`pass=240 fail=0`) → `ooo2/run-ooo2-directed.sh` (add a front-end
regression: straddle at a page boundary, redirect into a held window, `fence.i` vs the buffer) →
`CYC=60000000 ooo2/run-ooo2-cosim-linux.sh` (300000000 before shipping) →
`ooo2/run-ooo2-cosim-gb5.sh` per batch → `make census`/`timing` → `tools/board-gate.sh`. Match
`OOO2_HW`/`OOO2_IW`/`CACHE`/`CYC` across every compared pair; never build the `HW=2` point.

## Spec (`docs/rtl-rules.md` H4, part of each commit)

§2/§2.2 (F/X queue is before decode), §3.4 (redirect path, Stage 3), §4.1 (ahead-PC/`pnpc_kind`/
`lenp` gone), §8.1/§9.1 (I$ PIPT → VHPR, translation off the hit path), §9.2 (`NREQ=2`), §10.1
(`q_dat` 255, the `mem_ld` two-writer arbiter), §10.2/§10.3, §11 (`RD_WAIT` in the event sets),
§14 (the `IW=1` limit), §15 (P5 closes; P2/P7 re-scoped by Stage 4). Replace the missing
fetch-buffer section with the VHPR I$ section from `docs/VHPR.md`.
