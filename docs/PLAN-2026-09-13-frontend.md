# Frontend rewrite: conventional, width-parameterized

Status: design plan, 2026-09-13 (rev 2). Supersedes `PLAN-2026-09-12-frontend.md`, written
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
  forks.**
- **References live in git history, not the tree.** The conventional width-parameterized front
  end `src/frontend.v` and the multi-slot renamer `src/decode_rename.v` (with intra-bundle slot
  forwarding) were deleted in the release. Recover them read-only from before `384fd599`:
  `git show 384fd599~1:src/frontend.v`. They are the shape to copy.
- **`src/aligner.v` is already `IW`-parameterized and carry-free.** A 32-bit straddler reappears
  as slot 0 of the next window with no FSM. Keep it; the straddle special case is redundant.
- **Preserve two 2026-09-10 fixes** through any rewrite — correctness, not Fmax cruft: the
  corrector tag folding in its index PC bits (rule B9), and the aligner's solo-op test gated on
  halfword presence.

## The front-end pipeline

Named logical steps. A registered PC drives one path; a bundle enters at the top and leaves into
the **decoupling queue**. The VHPR I$ is read *here*, address out in **FP**, data back in **FA**,
with no translation on the hit path (that is Stage 2's point).

| step | name | work |
|---|---|---|
| **FP** | fetch / predict | the PC register (which *is* the fetch-target register) addresses the VHPR I$ (virtual tag) **and** the branch predictor in parallel; the predictor computes the next PC and latches it into the PC register for next cycle |
| **FA** | align | the I$ returns a 16-byte chunk pair (1-cycle synchronous BRAM read); the aligner windows it against the held pair (fetch geometry, below) and carves up to `IW` instructions, cutting at the predicted-taken halfword or, failing that, at the window/page end |
| **FD** | decode | RVC-expand each slot, decode (control blob + operands), and **resolve intra-bundle dependencies**: mark each source that reads an earlier slot's destination with that slot's index |
| — | **decoupling queue** | a small FIFO (depth 8) of decoded bundles; lets FP–FD run ahead of the backend so a fetch bubble or a backend stall does not serialise the two. This is today's opaquely-named "F/X queue" — rename the RTL to `dq_*` and relabel `FE_QUE`, keeping the counter's id/bit (the perf JSON is keyed to it) |
| **R** | rename / dispatch | 2·`IW` map reads, `IW` map writes (see the map below), allocate ROB + scheduler entries, dispatch |

### Fetch geometry (FP/FA)

- **Read per cycle: one 16-byte-aligned chunk pair** (`RDW = HW*16 = 128`, two 8-byte banks).
  Not wider — a wider read needs more banks and the wide-BRAM geometry that "fetched garbage on
  real BRAM" (spec §9.1). 16 bytes sustains `IW`=3 (a parcel is ≤ 12 bytes) on ~3-byte average
  instructions.
- **Alignment latch, not the run-ahead buffer.** Hold the last chunk pair in a 16-byte register
  with a 1-bit valid, and window the aligner across `[latch : this cycle's read]` — up to 32
  bytes, so any `IW`≤3 parcel from any 2-byte offset completes in one cycle without re-reading.
  Because the I$ is virtual-hit, the latch protects no translation: it carries no VA-tag, no
  in-flight-fill poison, no third chunk, no self-reset — just "is the PC still in this pair" (an
  address compare) and a valid cleared on redirect and on an epoch bump. This is what "delete the
  fetch buffer" leaves behind: a depth-1 register, ~5 lines, replacing ~350.
- **Consecutive chunks, confirmed.** The two banks are addressed by their own rows (`pair_e`,
  `pair_o`, `rv_cache.v:187-190`), so a read always returns chunk *n* and *n+1* — consecutive
  whatever the parity — swapped by `clo[0]` and byte-shifted. The wide I$ fetch asserts a
  16-byte-aligned request, so its pair is `{2n, 2n+1}` at one row.
- **Spanning.** A 16-byte pair boundary and a 64-byte line boundary are ordinary: consecutive
  pairs are consecutive I$ reads, and the latch windows across them. The **translation** boundary
  is the only truncating case (a pair never spans even 4 KiB). Cap at the *enclosing page's*
  boundary, not a hardcoded 4096: store **one bit** per VHPR I$ line — `4 K` or `2 M`, the only
  two sizes Linux maps code with — and cap on `off[11:·]` or `off[20:·]`. On Linux's
  2 MiB-mapped code a parcel then spans 4 KiB boundaries freely and only cuts at 2 MiB — ~0.4% of
  fetch cycles back on such code, and a 4 KiB-crossing instruction is fetched whole instead of
  split, so the cross-page straddle path (where the ld.so bug lived) runs 512× less often. The
  straddler restarts as slot 0 of the next page's parcel, which does that page's virtual-tag
  lookup and takes any fault precisely. An `sfence.vma` that re/demotes a page bumps the epoch,
  dropping the line and its stale size bit — no new invalidation logic.
- **The cut-off residual is two different things, distinguished by the PC update.** A
  **predicted-taken** cut at halfword k discards slots after k (wrong path, cost nothing — the
  pair was read anyway) and sets the PC to the **target**. A **window-end / page-end** cut keeps
  the residual (sequential path): the PC advances to the **first uncovered halfword**, which
  becomes slot 0 of next cycle's parcel, served from the latch (window end) or the next page's
  read (page end).

**Stage boundaries.** FP→FA is fixed by the VHPR I$'s one-cycle read. **FA and FD are two
separate stages by default** — that is how the spec has them (implicitly). Merge them into one
align+decode stage **iff it closes timing at IW=3**, and try, because the merge is an IPC win as
well as one fewer register: a shorter front end takes a cycle off every mispredict refill (fewer
fetch stages ahead of the resolving branch). The clock floor decides — it stays two stages
unless the spike shows one holds 166.667 at IW=3. RVC-expand (a ROM lookup) sits inside FD with
decode; that the two share one combinational step within FD is the expectation, and is a
separate question from the FA/FD merge.

## Branch prediction at `IW > 1`

**One prediction per fetch parcel, keyed by the parcel's base PC** — one BTB + direction lookup
per cycle, which scales, unlike `IW` independent per-slot predictors. The entry yields
`{taken?, the taken branch's halfword offset within the parcel, its target}`.

- **Halfword offset, not slot index.** At predict time (FP) the aligner has not run, so slots do
  not exist yet; the cut must be expressed PC-relative. The aligner (FA), which *does* know slot
  boundaries, cuts the bundle at the instruction whose offset matches and discards the rest of
  the parcel. This is your "slot or half-byte index" resolved: it is the halfword offset, and it
  is **exactly the `BOW` field the predictor already carries today** — generalized from "the one
  branch that ends this bundle" to "the first predicted-taken branch in this parcel."
- **In-flight tracking = that carried pair.** Every in-flight branch carries `{parcel base PC,
  halfword offset}` down the pipe. On resolve it (a) recomputes the predictor index and tags to
  train the right entry — already how `IW=1` training works (`res_base = res_pc − offset`) — and
  (b) redirects precisely to the taken slot's target or its fall-through on a miss.
- **The IPC comes from fetching past not-taken branches.** Today the aligner ends a bundle at
  the *first* CTI regardless of prediction, so a not-taken branch mid-parcel wastes the later
  slots. The scheme above lets the predictor pick the cut, so a not-taken branch flows through.
  That is the reason the predictor must name the taken branch rather than the aligner always
  cutting at the first one — and it is where width actually pays.

### Fetch Target Queue — considered, deferred

An FTQ decouples the predictor from fetch: the predictor runs ahead, filling a queue of parcel
PCs that the I$ engine drains.

- **Pros.** (a) The predictor gets ahead of fetch and can drive **fetch-directed I$ prefetch** —
  the real lever on SQLite and Clang, which lose 23-26% of cycles to I$ misses and take 40+
  redirects per 1000 insns; it is the stream buffer done right, and subsumes the deleted one.
  (b) It cleanly separates "where to fetch" from "fetch it," and is the conventional home for the
  parcel prediction above. (c) It lets a multi-cycle predictor hide behind the queue.
- **Cons.** (a) A second queue and its control, plus a predictor that emits a whole block per
  cycle. (b) The run-ahead pays only when the I$ actually misses; on I$-resident code (sha256,
  the boot) it is pure overhead. (c) Area and timing risk against a floor the plain predictor
  only just clears. (d) The static block histogram (mean 4.4, median 3;
  `PLAN-2026-09-06-frontend.md`) makes a 2-cycle predictor initiation interval marginal at `W=2`
  and a no-go at `W=4` without an overriding single-cycle BTB — so an FTQ does not by itself buy
  a slower, cheaper predictor here.
- **Recommendation: not in the conventional-rewrite path.** Build it later, once the single-cycle
  parcel predictor closes timing, as the vehicle for fetch-directed I$ prefetch. It is a plan of
  its own, seeded by `PLAN-2026-09-06-frontend.md` idea 3.

## The load-bearing risk, proven first

The special cases exist because the conventional structure **did not close 166.67 MHz**: the
ahead-PC head was 2.9 ns of a 6.06 ns path (2026-08-20). The plan's bet is that the extra
pipeline boundary (FA/FD split, and the VHPR I$'s FP/FA split removing the iMMU from the path)
buys that back conventionally. That bet decides everything, so prove it before Stages 1–4:

> **Spike.** Write the FP→FA→FD cloud with a fetch-target register in place of `apc` and the
> predictor read pipelined a cycle, synthesize out of context at 6 ns under two directives
> (`Explore`, `AltSpreadLogic_medium` — rule I2). **Go/no-go:** WNS ≥ 0 on both. If it fails, the
> answer is a still-deeper pipeline, never a reinstated selector. If it cannot be made to close
> at 166.667 MHz, the special cases stay — the clock is never lowered to ship a prettier front
> end.

## Stages

Each lands and is verified on its own. `OOO2_IW` defaults to **1**, so Stages 1–2 are
bit-identical to today at the shipping width and carry no IPC risk.

**Stage 0 — correct the record (docs only, immediate).** The release already quarantined the
111 MHz claims under `docs/history/`, closed the `run-ooo2-tests.sh` hole (deleted), and
`RD_WAIT` is captured (GB6, r0317). Residual: the `rv_soc_top.v:7` header ("No width knobs /
HW=2 halfwords" → HW=8), the spec's `q_dat` width (283 → 255) and arbiter count (`NREQ=2`), and
**parameterize `tb_ooo2_riscv.v` (line 13 hardcodes `HW=2`)** so riscv-tests exercise the
shipping window — the one real verification hole left.

**Stage 1 — conventional FP/FA/FD, `IW`-parameterized, `IW=1` bit-identical.** Build the named
pipeline above. Replace `apc`/`apred_v` with the fetch-target register (the register *is* the
BRAM address, so rule I6 holds with no ahead/real split); `pnpc_kind` → gone; `lenp` → gone
(FD decodes the real bytes); straddle FSM → the aligner's carry. Keep the page cap
`eff_avail = min(imem_avail, hw_cap)`. **The push→pop bypass on the decoupling queue is
deliberately NOT done** — it would put a mux on the queue-read path, and per your call we trade
that IPC (`FE_QUE`: 7.8% of sha256sum cycles, 24% on the AES kernel) for timing; revisit only if
a census shows slack. *Acceptance:* `ooo2/run-ooo2-cosim-linux.sh` at `HW=8 CYC=60000000`
returns the recorded count (`ooo2/cosim-expected.txt`: 14,301,801 ± 0.5%).

**Stage 2 — VHPR I$: translation off the fetch hit path (the prize).** Make the I$ virtually hit
(`docs/VHPR.md`; hit = `valid & epoch & ASID & vtag & perms`, physical tag off the hit path,
invariant: at most one valid line per physical line). The PC stays **virtual**, so branch
targets need no translation. This **deletes the fetch buffer** and everything it drags in:
`fb_pois`, the two STALE-mapping `$fatal`s, the FBDIAG block, the self-reset path, the
`imem_ctx_chg`/`imem_xlate_ok`/`imem_vaddr` ports — replaced by one epoch bump. Store the walk's
**page size** (one bit, 4 K or 2 M — the only sizes Linux maps code with) per line so the fetch
cap is at the mapping's real boundary, not a hardcoded 4096 (fetch geometry, above). Its retention tag
earns nothing (`FB_RHIT/FB_HIT = 0.3%`), and its forward-only slide is what makes a predicted
loop back-edge refetch the I$ every iteration (brbench, 2026-09-10). **I$ only** (no dirty lines;
the D$ is the hard half, left alone — IPC item 9). Keep and assert: the `fence.i` FSM, the
invalidation set (satp/`sfence.vma`/priv/`fence.i`) as an epoch bump, the VHPR invariant after
every alloc/evict/inval/writeback/flush; and convert the I$ response match from address to the
`rd_tag` the requester allocates (rule B1, now unavoidable). *Acceptance:* cosim clean at
`CYC=300000000`; re-baseline the mispredict-cost numbers (the buffer-hit vs I$-hit delta
disappears).

**Stage 3 — widen the backend to `IW`.** Land `IW=2` first, `IW=3` after. What each structure
needs, in order of what breaks first:

- **map — LUTRAM banks selected by a flop livemap (your proposal).** Keep the map in LUTRAM;
  generalize the existing `lv[]` from a 1-bit spec/committed valid to a **bank selector**. One
  speculative write-bank per slot (`map0..map_{IW-1}`, each 1W LUTRAM, bank *i* written only by
  slot *i*), the committed `rmap` as the base. Each of the 2·`IW` read ports is
  `lv[x]==committed ? rmap[x] : map_{lv[x]}[x]`; each slot *i*'s rename does `mapᵢ[dᵢ] ← pdstᵢ`
  and `lv[dᵢ] ← i` (younger slot wins a within-bundle WAW by write order). **Flush is a
  bulk-clear of `lv` to committed** — one cycle, no walk, exactly as today. This gets `IW` write
  ports out of 1W LUTRAM with no flop map, and is the smallest change to the existing
  SMAP/RMAP/lv. **Within-bundle deps never reach the map** — FD marked them, so R steers slot
  *j*'s source to slot *i*<*j*'s freshly allocated pdst through a small mux ahead of each read;
  the map handles only cross-cycle deps.
- **RMAP, free lists** `fl_ie/ld/fe`: `IW` commit writes / `IW` head-moves per cycle,
  **head-move only, never widen the walk** (`Area-Efficient-Scalar-OoO.md` Appendix A).
- **`ooo2_iq` dispatch, `ooo2_rob` commit**: `N` ports; **the `irr` pointer must advance `N`/cycle**
  or it gates the store queue (`irr` is what commits stores).
- **`ooo2_pending`** `N`-wide; the redirect squashes younger slots in the same bundle.
- **First thing to break: the `mem_ld` PRF port** — two writers already (a landing load and M's
  mul/div), asserted safe only because `m_done` is forced low on `ld_land`. Budget `IW=2` as the
  deliverable, `IW=3` as follow-on. Needs `ooo2/sweep.sh` over `OOO2_IW ∈ {1,2,3}`.

**Stage 4 — re-derive only what timing demands.** After each stage, `make census` + `make timing`
(two directives). Re-add a survivor only with a one-line reason at the site and a spec entry.
Then re-baseline `cosim-expected.txt`, the six `febench` numbers, and the spec CPI stack.

## Ordering

Spike → 0 → 1 → 2 → 3 → 4. Stage 0 is immediate. Stage 2 before Stage 3 (a simple fetch path is
easier to widen). The spike gates the whole plan.

## Gates (per stage, cheapest first)

`ooo2/run-ooo2-*-tb.sh` → `src/lint.sh` (`lint: clean`, PINMISSING is a hard error) →
`ooo2/run-ooo2-vl.sh` (`pass=240 fail=0`) → `ooo2/run-ooo2-directed.sh` (add a front-end
regression: straddle at a page boundary, a not-taken branch mid-parcel, `fence.i` vs the I$) →
`CYC=60000000 ooo2/run-ooo2-cosim-linux.sh` (300000000 before shipping) →
`ooo2/run-ooo2-cosim-gb5.sh` per batch → `make census`/`timing` → `tools/board-gate.sh`. Match
`OOO2_HW`/`OOO2_IW`/`CACHE`/`CYC` across every compared pair; never build the `HW=2` point.

## Spec (`docs/rtl-rules.md` H4, part of each commit)

§2/§2.2 (the FP/FA/FD pipeline and the decoupling queue, renamed, drawn before rename), §3.4
(redirect path, Stage 2), §4.1/§4.2 (ahead-PC/`pnpc_kind`/`lenp` gone; parcel prediction with the
halfword offset), §5/§5.1 (the livemap-banked map), §8.1/§9.1 (I$ PIPT → VHPR; the per-line 4K/2M cap bit), §9.2 (`NREQ=2`),
§10.1 (`q_dat` 255; the `mem_ld` two-writer arbiter), §10.2/§10.3, §11 (`RD_WAIT`, and the
`FE_QUE` relabel), §14 (the `IW=1` limit), §15 (P5 re-scoped; P2/P7 re-scoped by Stage 3).
Replace the missing fetch-buffer section with the VHPR I$ section from `docs/VHPR.md`.
