# Stage 2 implementation design — the VHPR I-cache (the prize)

Branch `wip/fe-stage2` off `main` (`7ce72f22`, Stage 1 tip). Parent: `PLAN-2026-09-13-frontend.md`
§"Stage 2". Full design: `docs/VHPR.md` — Stage 2 uses only its **I$-only, clean** subset.
Goal: make the L1 instruction cache **virtually hit**, taking translation off the fetch hit
path, and **delete the run-ahead fetch buffer**.

## The key realisation (from mapping the current tree)

The fetch buffer (`ooo2/rv_soc_top.v:591-1096`, ~500 lines) is **already a VA-tagged (VIVT)
structure** sitting in front of the PIPT I$ (`rv_cache` `u_icache`): its hit test `fb_alv` is a
*virtual* tag, precisely so the iMMU's TLB compare stays off the hit cone. Translation
(`u_immu`) still runs every fetch to make the PA the PIPT I$ is looked up by. So today there are
**two** tag structures — virtual (buffer) and physical (cache) — and translation is on the
fill/lookup path.

VHPR folds the virtual hit **into the L1**: the I$ is virtually indexed and tagged, translation
moves to the **miss path only**, and the buffer's whole job disappears. What is left is a
depth-1 16-byte **alignment latch** (~5 lines). Gone with the buffer: `rq_pois` poison, the
self-reset (`fbd_rst_*`), the two STALE-mapping `$fatal`s (`:746`, `:1025`), FBDIAG
(`:751-854`), `FB_TRACE`, and the forward-only page-crossing slide (`fb_samepg`) that was the
ld.so bug's home.

## Geometry (confirmed against the RTL)

I$ = **64 KiB, 2-way skew-associative, 512 sets, 64 B lines**, `RDW=128` (two 64-bit banks),
`WRITABLE=0` (fill-only). Index `VA[14:6]`; with 4 KiB pages the virtual color is `VA[14:12]`
(**8 colors**) and the same-offset synonym domain is 2 ways × 8 = **16 physical-tag candidates**
— exactly `docs/VHPR.md`'s config. The PC stays **virtual**, so branch targets need no
translation.

## Scope — I$-only, clean

The I$ never holds dirty lines, so the full VHPR spec collapses hard:
- No writeback, no dirty-synonym case, no victim buffer. A miss's resident synonym is always
  **clean**: invalidate the old alias, fill the target.
- **No D$ change** — the write-back half is the hard one and is left alone (IPC item 9).
- Reconciliation = translate → probe the 16 same-offset physical tags → invalidate the ≤1 match
  → fill. The **single-copy invariant** (≤1 valid line per physical line) is the one rule.

## Hit condition (FP addresses, data back in FA)

`hit = valid & (epoch == cur_epoch) & (ASID == req_ASID) & (vtag == req_vtag) & perms_ok`. The
physical tag is stored per line but **not** on the hit compare. Cached perms (R/X/U + the page
size) are re-evaluated against current priv/SUM/MXR each hit (VHPR.md "Permissions"), so no TLB
lookup and robust across priv changes. FP drives the I$ address from the virtual PC register; FA
gets the 16-byte chunk pair back (1-cycle sync BRAM read) and the aligner windows it against the
alignment latch.

## Invalidation = epoch bump (2-bit)

`satp` write / `sfence.vma` / privilege change / `fence.i` → **advance the I$ epoch** (reusing
the existing `o_tlb_flush` / `imem_ctx_chg` / `ifence` triggers — the invalidation set is
already wired, it just drove a buffer flush before). A bump makes every stale line miss
(`epoch != cur`) without clearing arrays. **Roll-over** (2-bit wraps to 0): walk both ways
invalidating before publishing epoch 0 (VHPR.md "Epoch Roll-Over"; the I$ is clean so the walk
is invalidate-only, no writeback). `fence.i` keeps its FSM ordering after any required D$
writeback.

## Load-bearing risk

Timing risk is **lower than Stage 1's**: removing the iMMU from the hit path is subtractive, and
the synonym probe is on the multi-cycle **miss** path, not the cycle-time hit path. The bet is
that the hit cone (VA index → tag/data BRAM read → vtag/ASID/epoch/perms compare → aligner)
closes 166.667 MHz — which it should, being simpler than today's buffer+PIPT path. The
**dominant risk is correctness**: the single-copy invariant across alloc/evict/inval/synonym/
epoch-rollover, proven by always-on assertions + the Linux cosim (which is what caught the
ld.so page-cross bug) + a brbench re-baseline.

## Increments (each: lint → 240/0 → cosim ±0.5% → census; board on the last)

0. **Spike — VHPR hit-cone timing** (throwaway). VA-indexed tag+data read, epoch/ASID/vtag/perms
   compare, feeding the aligner cone; OOC at 6 ns under both directives. Go/no-go WNS ≥ 0 before
   increment 2. Expected to pass comfortably (subtractive vs today); if not, deepen the pipeline,
   never reinstate translation on the hit path.
1. **Expose the leaf page size from the iMMU** (`src/mmu.v`, `ooo2/ooo2_core.v`). Add a 1-bit
   size output (`4K` vs `≥2M`; a 1G leaf caps conservatively as 2M) driven from `tlb_lvl[hit]` /
   `lvl` at walk-done, threaded alongside `immu_pa`. Additive, no behaviour change. Gate:
   retire-identical.
2. **VHPR I$ core + buffer deletion + alignment latch** — the big landing.
   `rv_cache.v` gains a **VHPR mode** (a `VIRT` param): virtual index/tag + epoch + ASID + perms
   hit, physical tag stored, miss path translates + 16-candidate synonym probe + single-copy
   invariant + epoch invalidation; the `u_icache` instance selects it. Delete the fetch buffer
   (`rv_soc_top.v:591-1096`); the frontend reads the VHPR I$ directly (`imem_ok`/`imem_data`/
   `imem_avail` from the cache); the buffer becomes the depth-1 alignment latch. Response matched
   by `rd_tag` (rule B1; already echoed). Always-on invariant assertions. Gate: cosim clean at
   `CYC=300000000`, brbench re-baseline.
3. **Page-cap bit + retire the straddle FSM** (`rv_cache` line metadata, `src/fetch.v`). Store the
   4K/2M bit per line at fill (increment 1); cap fetch at the enclosing page boundary; the
   aligner's carry replaces the `strad`/`ipc_q` FSM that Stage 1 kept. Gate: a page-boundary
   cosim regression (straddle at 4K and 2M).
4. **Re-derive timing** — `make census` + `make timing`. Re-baseline `cosim-expected.txt`, the
   febench numbers, and the mispredict-cost numbers (the buffer-hit vs I$-hit delta disappears).

## Correctness assertions (always-on, from VHPR.md)

- At most one valid I$ line per physical line, after every alloc / evict / inval / epoch-rollover.
- An epoch bump makes every stale line miss; roll-over walks both ways before publishing epoch 0.
- The I$ response is matched by the allocated `rd_tag`, never by address (rule B1).
- `fence.i` invalidates I$ state after any required D$ writeback (the ordering FSM is kept).
- Perms are re-checked against current priv/SUM/MXR on every hit.

## Gates + spec updates

Per increment: `src/lint.sh`, `ooo2/run-ooo2-vl.sh` (240/0), the Linux cosim, `make census`.
**Board on increment 4** (stage complete). Spec, per commit: §8.1/§9.1 (I$ PIPT → VHPR; the
per-line 4K/2M cap bit; the 16-candidate synonym probe), §4.1 (fetch geometry: the alignment
latch, the page cap; the straddle FSM retired), and replace the fetch-buffer description with a
VHPR I$ section drawn from `docs/VHPR.md`.

## Not in Stage 2

The D$ stays PIPT write-back (the hard half). `OOO2_IW` stays 1 (widening is Stage 3). No
synonym *migration* (the clean-invalidate-then-fill policy is the first implementation;
VHPR.md's migration optimisation is later if a counter shows it pays).
