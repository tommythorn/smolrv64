# The cost of a superscalar front end — ideas

Exploration only. No RTL was changed in the session that produced this file, and
nothing here has been simulated or built. Numbers quoted from the tree are cited to
where they were measured; everything else is marked as an estimate or an open
question. When an item ships, `docs/OOO2-Spec.md` moves with it (§4.1, §8.1, §10.2).

## Why

Two-wide fetch (plan item 10a) and the run-ahead fetch buffer (10e) put the fetch
cloud back on the critical path: gate U failed at **−0.620 ns** on
`pc_q → iMMU → fetch buffer → aligner → +2*consumed → the RAS write and the F/X
queue's slot-1 PC` (23 levels, ten CARRY8), and closed only after the fall-through
and the slot PCs became muxes of `pc_q + 2i` instead of an adder after the aligner
(`ac5fe12f`). The impl_1 run in the tree closes at **+0.032 ns** with all ten worst
core paths sourced at `u_sq/headc_reg[1]_rep_replica_3` and sinking at
`fe/q_dat0|1` (the F/X queue LUTRAMs) and `fe/u_fetch/pc_q_reg[8]/D` — the store
queue's back-pressure arriving at the frontend's enables, with the fetch cloud
arriving at the same pins on the data side.

166.67 MHz is not tradeable (the next rung is 111). So every further step of
frontend width — `IW=3/4`, `HW=16`, two decodes — has to be paid for out of slack
that is already spent. These ideas are about finding that slack in the front of the
fetch cloud rather than the back — idea 1 clears the front of it, idea 2 clears the
middle and the back (§2.9 argues those two combine to about two stages), and idea 3
lets the predictor run ahead of both, which is what turns the cleared path into fetch
that is early rather than merely short.

---

## Idea 1 — the physical PC

**Track the physical translation of the fetch PC in a register; translate only when
the PC leaves the leaf that maps it.** The iTLB stops being the first 1–3 ns of the
fetch cloud and becomes a unit whose result terminates at a flop.

*Terms, because the difference is the whole point.* A **leaf** is the PTE that maps
the PC — 4 KiB, 64 KiB (Ssvnapot), 2 MiB or 1 GiB. A **frame** is a 4 KiB unit of
physical address; a leaf is 1, 16, 512 or 262 144 frames. Everything about
*validity* in this section is per **leaf**; the only thing that is per **frame** is
the fixed-width concatenation that produces the fetch address (§1.2). Where the text
says 4 KiB, it means the frame and says so.

### 1.1 What is already true

Half of this idea has been built, twice, and it is worth being precise about what
is left:

- **The buffer's hit test is already virtual.** `rv_soc_top` tags its chunks with
  the VA (`fb_alv == fb_va`), not the PA, and `fb_samepg` keeps every chunk inside
  chunk0's 4 KiB page so the pair is valid exactly as long as one translation is.
  The RTL records what that bought: the PA-tagged version put "req_match's 64-bit
  compare plus this PA compute, six CARRY8 stages" in front of the buffer hit, the
  I$ data mux, the aligner, the next PC and the predictor read — **~2.9 ns of a
  6.06 ns path, measured 2026-08-20 at 166.67 MHz**.
- **The arrival bypass is already virtual** (`7f9bcc97`): the chunk arriving is the
  one that was requested, so it is matched by the VA kept with the request.

What is left of address translation in the frontend is exactly three things:

1. `imem_avail_g = (immu_ready & ~immu_fault & ~imem_ctx_chg) ? imem_avail : 0`
   (`ooo2_core.v`). This is the one site that decides whether fetch data may be
   consumed, so it is in front of the aligner, `valid`, `fire`, the `pc_q` enable
   and every F/X queue write enable — the same endpoints the current worst family
   ends at. Inside it:
   - `immu_ready`/`immu_fault`: a 16-entry direct-mapped TLB read indexed by
     `vpn0[3:0]`, a **27-bit tag compare**, the permission function, and the
     level-muxed `leaf_pa` (`mmu.v`);
   - `imem_ctx_chg = mmu_flush | (ipriv_q != mmu_priv) | (isatp_q != satp_fetch)`
     — a **64-bit satp compare** plus a priv compare.
2. `fb_al = {imem_addr[63:CHA], 0}` on the realign arm of `fb_want`, hence
   `ic_rd_addr`, hence the I$ tag array — the RTL says so itself ("the realign arm
   skips it (`fb_al` is on the iMMU path)"), and the request deliberately leaves
   unregistered in its want cycle. This path is live on **every redirect and every
   page change**.
3. `fb_wantv & imem_xlate_ok`: the request valid carries the iMMU too.

### 1.2 The mechanism

A **fetch translation register** (FTR), written by the iMMU and read by nothing
else:

```
ftr_v          1      a usable translation is held
ftr_lvl        2      leaf level: 4 KiB / 64 KiB NAPOT / 2 MiB / 1 GiB
ftr_vpn       27      the leaf's VPN, masked by level -- the stay-in-leaf tag
ftr_base      22      the leaf's base FRAME PPN, PA[33:12]   (PAW_SIG = 34)
ftr_ppn       22      the CURRENT frame PPN  -- the only field the fetch path reads
ftr_fault    1+4      the leaf's fault verdict, captured with it
```

**Always materialise the frame PPN**: `ftr_ppn = ftr_base | (pc_q[38:12] & ~lvl_mask)`.
The 4 KiB here is the granule of the *concatenation*, not an assumption about the
mapping — doing that OR at the *use* site instead would make the concatenation's
width a function of the leaf level, which is a mux on exactly the path being cleared.
On the *update* path it costs nothing: inside a leaf, a fall-through frame crossing is
`ftr_ppn + 1` and a jump elsewhere in the leaf is a fresh OR, both from registers,
both in the cycle that writes `pc_q`. A frame crossing inside a leaf is **not** a
re-translate: `ftr_v` stays set, the iMMU is not consulted, and only `ftr_ppn` moves.
The fetch path then reads a fixed 22+12 concatenation whatever the leaf size is.

Then the whole head of the fetch cloud is a concatenation of two registers:

```
imem_addr = {ftr_ppn, pc_q[11:0]}
avail_gate = ftr_v                       // replaces immu_ready & ~immu_fault & ~imem_ctx_chg
```

and the iMMU's combinational output goes **only** to the FTR's D pins. Same logic,
but it becomes a path of its own with a full cycle after it, instead of the first
1–3 ns of a path that also has to fit a buffer lookup, a 128-bit shift, an aligner,
a decode and a LUTRAM write.

The FTR is loaded in a **re-translate cycle**: `ftr_v` is low, no bundle is
produced, `pc_q` is presented to the iMMU, and the result is captured. A TLB hit
costs one cycle; a miss costs the walk, exactly as today.

### 1.3 Where the "does it stay in the leaf" bit comes from

Not from a compare on the *result* of the PC mux — that would put a 27-bit compare
at the end of the fetch cloud, where the +2*consumed adder used to be. It comes per
arm, the same shape as `pnpc_kind` and `apc` already use, because every arm is a
flop output or a function of `pc_q` alone:

| arm | stay-in-leaf test | when it is ready |
|---|---|---|
| fall-through | `pc_q` is not in the last `2*IW` halfwords of the leaf | from `pc_q`, top of cycle |
| predicted target | `pred_tgt[38:12] == ftr_vpn`, masked by `ftr_lvl` | `pred_tgt` is a registered BTB/RAS read |
| redirect | `redirect_pc[38:12] == ftr_vpn`, masked | `redirect_target_q` is a register |
| interrupt pseudo-op | always (PC is held) | — |
| straddle | never (that is its point) | — |

The final select is the same select the PC mux uses. This is rule I6 applied to
translation instead of to prediction.

### 1.4 The page is the leaf, and the leaf is usually 2 MiB

Sv39 leaves are **4 KiB, 2 MiB and 1 GiB**; this MMU also decodes the one Sv39
Ssvnapot encoding, a **64 KiB** level-0 leaf. The stay-in test is against the
**leaf**, not against 4 KiB, and that is where most of this idea's value sits:
Linux maps kernel text with **2 MiB PMD leaves**, so there are 512× fewer boundaries
for control flow to land on.

What a 2 MiB leaf changes, concretely:

- **Intra-image calls and returns mostly stop crossing** — case (4) in §1.7, the
  only cost that matters, is a function of how often control leaves the leaf, and a
  512× coarser grid catches far less of it. Mostly, not never: see below.
- **Sequential crossing falls to one per ~512 K instructions**, from one per ~1 K.
- **The stay-in test narrows** to VA[38:21] — 18 bits instead of 27.
- **`fetch.v`'s window cap stops cutting bundles.** `hw_bound = (4096 − off) >> 1`
  truncates a bundle 512 times per leaf for no reason once the leaf is 2 MiB, and at
  `IW=2` a truncated bundle is a lost slot.
- **The straddle fires 512× less often**: a 32-bit op at the last halfword of a
  *leaf*, not of a 4 KiB page.
- **The run-ahead buffer stops stalling every 4 KiB.** `fb_samepg`/`fb_samepg2`/
  `fb_samepg3` gate chunk prefetch on the 4 KiB page today, so at every 4 KiB
  boundary `fb_v1` stays 0 and the buffer realigns. "One extra lookup per page,
  which is nothing" at 4 KiB is nothing *at all* at 2 MiB; straight-line
  (`febench`-shaped) code gets those cycles back.

**What still crosses, at any leaf size.** A bigger leaf scales the crossing rate
down; it does not remove it, and what is left is not a tail:

- **Text larger than one leaf.** A 6 MiB `.text` is three or four leaves however it is
  mapped, and it is not leaf-aligned, so every call across an internal boundary
  crosses.
- **Inter-image control flow** — PLT stubs, every call into libc and every return from
  it, every callback. Separate mappings, so a different leaf by construction, and these
  are the *frequent* ones.
- **user↔kernel** — every syscall and every trap: a different leaf *and* a privilege
  change, which invalidates outright (§1.5).
- **A crossing call costs two crossings**, because its return crosses back.

So §1.7 case (4) survives 2 MiB mapping and §1.8 is more likely to be needed than
"kernel text is 2 MiB" alone suggests. But the *shape* of the residue matters as much
as its size: what is left is mostly a **pair of leaves ping-ponging** — a loop calling
memcpy, a syscall path, a PLT stub — and that points at a much cheaper fix than the RAS
and the BTB (§1.8 item 0).

Two places in the existing RTL have a 4 KiB constant that must become leaf-relative
for any of the above to be real, and neither is expensive: `fetch.v`'s window cap
(`hw_bound`) becomes "am I within `2*IW` halfwords of the *leaf* end", which is a
mask compare selected by `ftr_lvl`; and the fetch buffer's `fb_samepg*` prefetch gate
becomes "same leaf", bounded by the buffer's own tag span (§1.10).

**The caveat, and it is the important one: kernel text is 2 MiB-mapped, user text
usually is not.** File-backed executable mappings are 4 KiB unless khugepaged
collapses them (`CONFIG_READ_ONLY_THP_FOR_FS`) or the text sits in a hugepage-backed
mapping. The boot and anything kernel-heavy get most of the win as built; GB5 runs in
userspace and probably does not. That makes the leaf-level histogram (§1.9 item 2)
the measurement that decides how much of §1.8 ever has to exist — and it makes the
guest's THP configuration a **hardware-relevant software knob**: enabling
file-backed THP for a benchmark's text moves it out of case (4) altogether, for no
RTL.

### 1.5 Correctness: the invalidation set

`ftr_v` must clear on every event that can change what the current VA maps to, or
what may be executed from it. That set is exactly `imem_ctx_chg`'s, which the
design already derives and already uses to drop the VA-tagged buffer:

- `mmu_flush` — sfence.vma or a satp write (`csr_file.o_tlb_flush`),
- a privilege change — different translation *and* a different X/U verdict,
- reset.

SUM/MXR stay out: they gate data accesses, not fetch. Two properties make the set
sufficient, and both are already relied on elsewhere in the tree:

- **Only non-faulting translations are ever captured.** A perm-faulting TLB hit is
  treated as a miss inside `mmu.v` precisely so that the fault comes from a fresh
  walk (the RVA22 software-A/D contract: the kernel sets A in memory and retries
  *without* sfence — the bug that livelocked the Ubuntu EXT4 mount). An FTR that
  caches only clean translations inherits that argument unchanged.
- **Removing a permission or a mapping requires an sfence**, which is in the set.

The FTR is also strictly *less* exposed than the buffer it replaces logic for: it
holds one leaf, and it is invalidated by a registered event rather than compared
combinationally against a live satp.

**The one real hazard is the change cycle.** Today `~imem_ctx_chg` is a
*combinational* term in `imem_avail_g`, and it is load-bearing: the comment records
the silicon failure it was added for (`va=ffffffff80012370` hit with
`pa_cached=0000000080212370` while the iMMU had already switched to bare mode). A
registered `ftr_v` clears at the end of that cycle, so one more bundle can be
consumed under the old translation. Whether that matters depends on a question this
plan does not answer: **does a context change squash already-fetched bundles?**
`ser_block` drains the ROB *before* a CSR op dispatches and lets nothing dispatch
behind it, but the F/X queue and the fetch buffer are not flushed — so bundles
fetched under the old mapping already exist regardless, and the exposure may be
pre-existing rather than new. Resolve this before writing RTL; the three exits are
(a) keep `~imem_ctx_chg` combinational in the gate and lose part of the win, (b)
show the redirect covers the cycle, (c) flush the F/X queue on a context change and
be done with the whole class.

### 1.6 Correctness: the invariant

This is a change to *which bytes execute*, so it gets the strongest available
check, and the tree already has the pattern — `587e87f0` gave the arrival bypass
"the silicon twin of its stale-mapping invariant", and `rv_soc_top` already
`$fatal`s when a VA-tag hit names the wrong PA.

The iMMU is idle whenever the FTR is valid, so it can keep translating `pc_q` for
free and the check writes itself:

```
if (ftr_v & immu_ready & ({ftr_ppn, pc_q[11:0]} != immu_pa[33:0]))
    $fatal(1, "fetch: FTR names %h, iMMU says %h at va %h", ...);
if (ftr_v & immu_ready & immu_fault)
    $fatal(1, "fetch: FTR held across a translation that now faults");
```

Always on, no `ifdef`. In synthesis the iMMU's combinational output terminates at
the FTR and at nothing else, so the check costs nothing on the board. Comparing the
*materialised* `{ftr_ppn, pc_q[11:0]}` rather than `ftr_base` means one check covers
both the cached translation and the within-leaf frame arithmetic of §1.2.

### 1.7 What it costs

One cycle per **fetch leaf crossing** — not per frame crossing, which is free
(§1.2) — on top of the I$ access the crossing already pays for. Crossings come from
four places, in rising order of how much they matter:

1. sequential fall-through out of the leaf — one per ~1 K instructions at 4 KiB,
   one per 512 K at 2 MiB. Negligible either way.
2. the straddle — already a stall today; it *folds in*, because the straddle's
   translation of `pc_q+2` **is** the next leaf's translation. Capture it in the
   FTR and the PC lands in a leaf that is already resolved. One mechanism, not two.
3. redirects (mispredicts, traps) to another leaf.
4. **taken CTIs to another leaf** — calls and returns. This is the whole cost.

The recovery mechanisms below exist because of (4), and none of them should be
built before the number in §1.9 is known.

### 1.8 Recovering the bubble — only if the measurement says to

In order of payoff per bit, each guarded by a **translation epoch**: a small counter
bumped on every event in §1.5, carried with any cached PPN, and compared when it is
used. A stale PPN would fetch the wrong bytes at a right VA with nothing to catch
it, so "it is only a prediction" is *not* an available defence here — a prediction
structure may be wrong about where to go, never about what lives at a given VA.

- **0. Make the FTR a small file, not a single entry.** The residue in §1.4 is
  dominated by two leaves ping-ponging, and a single entry thrashes on exactly that:
  the call re-translates, the return re-translates, once per iteration. Two entries
  cover the pair after its first crossing; four cover a nesting. Both halves stay
  where §1.3 put them, so it stays off the fetch path: `imem_addr` becomes a 2:1 or
  4:1 mux of *registered* PPNs on a *registered* select — one LUT level — and the
  stay-in tests grow to entries × arms, every one of them register-to-register with a
  full cycle to resolve. In effect a fully-associative **micro-iTLB placed before the
  PC register**, with the 16-entry iTLB behind it as its refill. No epoch beyond
  invalidate-all, no BTB widening, no RAS change: strictly cheaper than a–c, and it
  should be measured before any of them.
- **a. The PPN rides the redirect.** A branch resolving not-taken redirects to its
  own fall-through; a mispredicted direct branch redirects to `pc + imm`, whose
  same-leaf property is known at decode. Carry `{ppn, valid}` with `redirect_pc`
  and the common mispredict costs no re-translate.
- **b. The RAS pushes the PPN.** A return address is in the caller's leaf, so the
  PPN is in hand at push time; a `ret` then fetches with no translation. Returns
  are half of case (4) and the only half that is not statically predictable.
  Cost: 8 × 22 bits of flops.
- **c. The BTB stores the target PPN** — for cross-leaf targets only; same-leaf
  targets need no storage at all, because the answer is `ftr_ppn`. Cost: real
  BRAM (53 → 75 bits × 1024). Last, and only on evidence.
- **d. Pre-translate from `apc`.** `apc` is the register-only ahead PC that already
  addresses the predictor arrays. Feeding it to the iMMU instead of `pc_q` fills a
  *next*-FTR one cycle early, so a predicted crossing costs nothing. A wrong `apc`
  degrades to a re-translate bubble, never a wrong translation — the same
  degradation argument §4.1 already makes for the predictor. Watch: a speculative
  walk must raise no fault and must not be able to thrash the 16-entry direct-mapped
  iTLB.

### 1.9 The measurements that decide it, none of which need a build

1. **The fetch leaf-crossing rate**, split by cause (fall-through / predicted-taken
   / redirect / straddle), as a fraction of cycles, and reported **at every leaf
   granularity** — `pc[i] >> k != pc[i-1] >> k` for k = 12, 16, 21 — because the
   answer at k=12 and the answer at k=21 are different numbers and the second is the
   one that applies to kernel text. Obtainable offline from a retire trace (a lower
   bound; wrong-path fetch adds more). This is the single number that says whether
   §1.8 is needed at all. Estimate to beat: if crossings at the granularity item 2
   reports are under ~0.5% of cycles, stage 1 ships alone.
   Report the **shape** as well as the rate: how many distinct leaves are live in a
   window of N crossings, and what fraction of crossings return to the leaf left most
   recently. That distribution, not the rate, sizes §1.8 item 0 — a two-entry file
   answers a ping-pong and nothing else.
2. **The iTLB fill-level histogram** (4 KiB / 64 KiB / 2 MiB / 1 GiB), separately
   for kernel and user PCs. This says how big "the page" actually is on GB5 and on
   the boot, and therefore how much of (1) survives tracking the leaf instead of
   4 KiB.
3. **What the removal is worth in slack**: an OOC run of the frontend
   (`ooc_frontend.tcl` exists) with `imem_addr`/`avail_gate` driven from flops,
   against the same frontend as built. This is the cheapest honest answer and it
   does not disturb the full build.

### 1.10 Staging

- **Stage 1 — the FTR alone.** `ftr_v` replaces the translation terms in
  `imem_avail_g`; `imem_addr` becomes a concat; `fb_al`/`fb_want`'s realign arm
  loses the iMMU; the straddle folds into the re-translate state. Buffer tags shrink
  from the 64-bit VA to a **within-leaf chunk index**, since every chunk is inside the
  current leaf by construction — but the width is set by the largest leaf the buffer
  is allowed to span, not by 4 KiB: `pc[20:4]` (17 bits) if it may span a 2 MiB leaf,
  `pc[11:4]` (8 bits) only if it is capped at one frame. Capping at a frame would put
  back the every-4-KiB realign §1.4 removes, so 17 bits is the honest number, with a
  1 GiB leaf treated as 2 MiB for the buffer's purposes (a realign every 2 MiB is a
  non-event). Still far less logic than the pair of 64-bit tags today.
  Gates: `src/run-vl-tests.sh` (failures: 0), `CACHE=1`, `src/lint.sh`,
  `tb_fetch_pagecross`, the Linux cosim with the §1.6 invariant on, `src/sweep.sh`,
  then the board.
- **Stage 2 — recovery**, items a→b→c→d, each gated on the measurement and each on
  the epoch.

### 1.11 What idea 1 hands to the next one

Every CTI except a true `jalr` has a statically known target, so **whether it
leaves the leaf is a property of the instruction and the PC's offset in the leaf,
not of runtime state** — and a same-leaf target is fully described by that offset:
12 bits at 4 KiB, 21 at 2 MiB, against the 38 the BTB stores today. That is a fact
the BTB and the redirect path can both spend.

---

## Idea 2 — the "VLIW" packet cache

**Cache the aligned, expanded, predecoded, partially renamed and pre-scheduled fetch
group, built once at I$ miss time.** Trade space for time in the only direction an
FPGA likes: a wide fast path that is a RAM read, and a narrow slow path that is
allowed to take cycles.

### 2.1 What actually scales badly, and why it is not fixable in place

- **Alignment.** `aligner.v` is an O(W) *dependent* prefix scan — `p[k+1] = p[k] +
  len[k]` — followed by W variable muxes out of an HW-halfword window. The scan is
  serial in W and the muxing is W × HW. It sits in the middle of the F cloud, between
  the I$ data and the queue write, and the depth is fabric-bound: nothing about a
  faster process or a better placer makes a 4-deep carry of variable lengths cheap.
- **Rename.** The quadratic term is the intra-bundle match: slot k's sources against
  slots 0..k−1's destinations, W(W−1)/2 compares per source plus a priority mux, *in
  series with* the map read and the free-list read. Item 10b's own description of
  W=2 — "two LUTRAM copies plus a per-entry newer bit … the intra-pair bypass (B's
  source = A's destination takes A's new register)" — is the W=2 instance of a
  structure that grows as W² and has to be resolved before anything can be written.
- **Dispatch.** The port rule is already a width limiter: "a pair dispatches only to
  two different schedulers … otherwise B waits a cycle", and 10d's answer is a crude
  static balance ("a lone ALU op is always slot A's; measure before alternating").

All three are per-cycle work on bytes that will be fetched again thousands of times.

### 2.2 The packet

A packet is **the bundle the aligner already forms** — up to N instructions ending at
the first CTI or SYSTEM op — stored in decoded form. Per slot, roughly:

```
uop / control word     the expanded 32-bit op, or a decoded control word
unit class        3    ALU / LD / ST / MUL / DIV / FP / BR / SYS  (the scheduler select)
rd, rd_v          7    architectural destination
rs1, rs2, rs3    21    architectural sources
rs*_intra    3 x 3     "produced by slot i of this packet" + valid   <- the W^2 term, precomputed
no_map_write      1    a later slot in this packet overwrites this rd
no_alloc          1    ...and nothing outside the packet needs the value  (see 2.5)
prog_idx          2    true program-order index, if slots are reordered (2.6)
```

plus a packet header: base PC, length in bytes, slot count, `br_term`, the tail's CTI
class, the fall-through PC. Call it ~80 bits per slot and ~100 per packet.

On a hit the fast path is **PC → packet RAM read → dispatch**. No translation, no I$,
no fetch buffer, no shift, no aligner, no RVC expansion, no decode, no intra-bundle
compare, no steering decision.

### 2.3 Where it lives

The tree's current impl_1 run uses **122 of 480 BRAM tiles (25%)**, 69.5 K of 217 K
LUTs and 47.5 K of 434 K flops. So BRAM is *not* the constraint: 512 packets × 4 slots
× 80 bits is ~164 Kbit ≈ 5 RAMB36; 2048 packets is ~20. Either is affordable. What is
scarce is fabric on the fast path and BRAM *read width* (72 bits per RAMB36 in SDP), so
a 4-slot packet is ~5 tiles read in parallel.

That argues for a **separate, VA-tagged packet cache beside the I$**, not a widened I$:

- Storing expanded+predecoded packets *in* the I$ costs 2–3× the bits for the whole
  64 KB whether or not the code is ever executed, and forces packet boundaries to be
  chosen at fill time from bytes, without knowing where control actually enters.
- A separate cache is filled from the *execution* stream, so it only ever holds code
  that ran, and its packets start where control really entered.
- **The existing 2-wide path stays as both the builder and the fallback**, so a packet
  miss is never worse than today. This is the property that makes the whole idea safe
  to try: it is an accelerator, not a replacement.

Packet boundaries depend on the entry PC (a branch into the middle of a block yields a
different packet over the same bytes), so some duplication is inherent. Whether to key
packets by entry PC or force them chunk-aligned is a question for the study in §2.11,
not for the RTL.

### 2.4 Atomicity is the lever, and it buys four things at once

Require that **only the tail slot may redirect**. That is already true of the bundles
the aligner forms — "a control transfer OR a serializing op … *terminates* the bundle:
it is included as the last valid slot, so every such op is the youngest instruction in
its checkpoint" — so the constraint costs nothing new. What it buys is that the packet
has **no architecturally visible state between its first and last slot**, and that
single property is what licenses all four of:

1. intra-packet source tags (§2.2),
2. dead-destination elimination (§2.5),
3. reordering slots to fit the dispatch ports (§2.6),
4. instruction fusion (§2.7).

The escape hatch already exists and is already used: a fault inside a checkpoint rolls
back and re-fetches **solo** (`solo_all` into `fetch`, `dflt_replay`'s two-phase
path, "a checkpoint is atomic → re-executing it is correct"). Every optimisation above
is therefore allowed to be *unreplayable*, because the recovery is not a replay — it is
a re-execution of the original instructions with the packet cache bypassed. Two
consequences to write into the design: **`solo_all` must bypass the packet cache**, and
**interrupts are taken at packet boundaries only** (any instruction boundary is a legal
point for an asynchronous interrupt, and a packet boundary is one).

### 2.5 Split the destination kill in two — they are not the same change

"Rename a destination that is overwritten later in the packet to x0" is two independent
bits with very different costs:

- **`no_map_write`** — allocate the physical register as usual, but suppress the
  *speculative map* write. Removes the write-conflict case entirely: the map's W writes
  are then to W *distinct* architectural registers by construction, so the ordering
  logic (10b's "newer" bit and its priority resolution) disappears. **Nothing becomes
  architecturally invisible**, so the cosim, the ROB and `pold` are untouched — the
  later writer's `pold` is the pre-packet mapping, which is what it reads anyway. This
  is free, and it is most of the win.
- **`no_alloc`** — allocate nothing. Saves a free-list pop, a physical register and a
  PRF write port's worth of pressure. But the intermediate value now never exists, so a
  trap or a retire boundary between the two writers would expose it. That needs
  **atomic retire of the packet** (or the guarantee that nothing observes between them)
  and it needs the cosim taught that a suppressed write is legal exactly when a later
  slot of the same packet writes the same architectural register.

Take `no_map_write` first; `no_alloc` only if the study says the free-list and PRF
pressure is worth the verification.

### 2.6 Reordering for dispatch — the rule that makes it safe

The packet's slots are the machine's **dispatch ports**, so the builder should sort the
block into them. The rule:

> Reorder freely **across** schedulers; preserve relative order **within** a scheduler.

That is sufficient because order is consumed in exactly two places: `u_iq_l` keeps
program order by issuing at "fixed priority, lowest entry index", and memory ordering
rides the seqno/ROB index. Ordered ops (mem, AMO, mul, div, CSR, branches, SYSTEM) all
share `u_iq_l`, so keeping their relative order keeps everything `u_iq_l` relies on;
pure ALU and FP-arith ops go to `u_iq_i`/`u_iq_f`, which reorder by design.

Retirement stays in program order for free if **ROB entries are allocated by
`prog_idx`, not by slot** — a fixed permutation of the tail's N entries, which is
wiring, not logic. So the packet dispatches in port order and retires in program order.

This deletes 10b's "otherwise B waits a cycle" and replaces 10d's crude balance with a
decision made once, at build time, with the whole block in view.

### 2.7 Fusion

The builder is the only place in this machine where a multi-cycle sequential scan over
instructions is affordable, and it is amortised over every subsequent execution of the
block. The candidates that pay in RV64GC:

- `lui`/`auipc` + `addi`/`ld`/`sd` — the PC-relative and large-constant idioms; each
  fusion removes an instruction from the packet, the ROB, the PRF and the free list.
- `slli` + `add` — the indexed-address idiom.
- adjacent RVC pairs that map onto one 32-bit operation.

Fusion changes the retire stream, `minstret`, and the precise-trap point between the
fused pair — i.e. it needs exactly the §2.4 atomicity and the same cosim work as
`no_alloc`. It is the last item, not the first.

### 2.8 What it does to the rest of the frontend

- **The aligner never has to go past `IW=2`.** Width comes from the packet, not from
  the aligner, so the O(W) scan and the O(W·HW) muxing stop growing. This is the point
  of the whole idea: it *separates the frontend's width problem from its latency
  problem*.
- **`lenp` is unnecessary on a hit** — the packet stores its own length and
  fall-through — and `apc` becomes exact, so the ahead-PC's 0.13%-of-retires lost
  predictions go to zero on packet hits.
- **`br_term` and the tail's CTI class are stored bits**, not a predecode of the
  window.
- The predictor is already keyed by the bundle base (§4.2); a packet *is* that bundle,
  with a fixed base, so the key stops being something the fetch timing can perturb.

### 2.9 With idea 1: about two stages out of the front

Today `F` is one clock doing seven things in series — `PC → iTLB → I$ → fetch buffer →
aligner → RVC expand → decode` — then the F/X queue, then `X`'s rename.

- Idea 1 deletes the `iTLB` element (and the translation terms in the consume gate).
- Idea 2 deletes `I$ → fetch buffer → aligner → RVC expand → decode` on a hit, and the
  intra-bundle half of rename, replacing all of it with one BRAM read.

What is left on the fast path is *PC → packet RAM → free-list read and map read →
scheduler write*. That is what makes it plausible to fold decode+rename into the packet
read cycle and to stop needing the F/X queue to absorb aligner variability — **about
two stages, and the thing those two stages cost is the redirect-to-first-dispatch
latency, i.e. the mispredict penalty** (plan item 5). Treat the "two stages" as the
claim to check with `tools/fe-pipe-model.py` before any RTL, not as a measured number.

### 2.10 Hazards

- **A packet miss runs at builder width.** Keep the existing 2-wide path as the
  fallback so a miss is never a regression; then the only question is how often the
  fast path is taken.
- **Invalidation**: `fence.i` (the `fi` FSM already exists) and the context-change set
  of §1.5. A VA-tagged packet cache wants exactly idea 1's epoch — the two ideas share
  one invalidation rule.
- **`solo_all` must bypass the packet cache**, or fault replay is not a replay.
- **The cosim is the main verification cost**, and it is real: `no_alloc`, fusion and
  reordered dispatch all change the retire stream. `no_map_write`, intra-packet source
  tags and the packet cache itself do not — which is another reason to stage them
  first.
- **The always-on invariant writes itself**: in simulation, decode the packet's bytes
  sequentially through the existing `aligner`/`decode_stage` and `$fatal` if the packet
  disagrees, slot for slot. Plus two structural checks that cost nothing: an intra
  source tag must name a *lower* slot, and the slot it names must have that
  architectural register as its destination. A packet builder that goes wrong must not
  be discoverable only as a cosim divergence 40 M cycles later.

### 2.11 The pre-RTL study — one pass over a retire trace answers all of it

Before any RTL, from a trace of retired PCs and instructions:

1. **Block-length distribution** (instructions between CTIs) — this is the hard cap on
   packet size and therefore on achievable width. If the mean is 5, `N=4` is right and
   `N=8` is waste.
2. **Packet-cache hit rate** vs capacity and organisation, and the entry-PC duplication
   factor. Decides whether the fast path is ever taken on the boot and on GB5.
3. **Fraction of sources that are intra-packet** — the W² work actually removed.
4. **Fraction of destinations with `no_map_write` / `no_alloc`** — decides §2.5.
5. **How often a block's op mix fits the port shape** without splitting — decides how
   much §2.6 buys.
6. **Fusion counts** per candidate pair.

None of this needs a build, and (1)+(2) alone can kill or confirm the idea.

### 2.12 Staging

- **2a** — packet cache holding *aligned + RVC-expanded + predecoded* slots only; no
  renaming, no reordering, no fusion. Existing path as builder and fallback. This alone
  removes the aligner and the expander from the hot path.
- **2b** — intra-packet source tags and `no_map_write`. Removes the W² rename term and
  the map write-conflict logic. Still no change to the retire stream.
- **2c** — reordering for dispatch (§2.6), with ROB allocation by `prog_idx`.
- **2d** — `no_alloc`, then fusion. Both need atomic retire and cosim work.

Gates at every step: `src/run-vl-tests.sh` (failures: 0), `CACHE=1`, `src/lint.sh`, the
Linux cosim with the §2.10 invariant on, `src/sweep.sh`, then the board.

---

## Idea 3 — predict basic blocks, not destinations

**Make the BTB entry describe the whole block that starts at its key — extent, tail CTI
class, target — instead of only the CTI that ends the bundle.** One prediction then
covers a basic block rather than a bundle, so the predictor can run ahead of fetch.

### 3.1 What it changes relative to what is here now

Today the predictor is read **once per bundle**, keyed by the bundle base, and `apc`'s
fall-through arm guesses that bundle's byte length from `lenp` — 4 096 entries, 2 bits,
untagged, trained on the aligner's own count. The entry answers "what terminates the
bundle starting at A".

A block entry answers "what terminates the **block** starting at A", and carries the
distance to it. The well-definedness argument is already in the tree, one step short:
`branch-predictor-plan.md` observes that "the aligner is a pure function of the fetched
bytes, so a given block-start PC deterministically produces the same bundle and the same
terminating CTI". The first CTI *at or after* A is a function of the code in exactly the
same way, so a block entry keyed by A is equally well-defined. Consequences:

- **`lenp` disappears.** The fall-through arm of `apc` becomes a tagged, exact field.
- **The predictor is read once per block, not once per cycle.** The entry is held from
  block entry until the CTI is reached; the existing `btb_qpc == base_pc` check still
  proves the entry belongs to this block. That takes read pressure off the BTB and YAGS
  BRAMs, which is worth something — those are the arrays §4.2 moved into BRAM.
- **One prediction covers ~5–8 instructions instead of ~2.**
- The GHR update rate is unchanged: one CTI per block is one CTI per bundle-with-a-CTI,
  so the GHR/rollback repair (`res_rep`) does not move. Worth stating because that
  machinery is delicate.

### 3.2 The value is run-ahead, not accuracy — say so plainly

`lenp` at 4 096 entries already costs **0.002 redirects per thousand** (0.45 at 1 024,
which is why it was grown), and a wrong `apc` costs **0.13% of retires**. So replacing
the length guess with an exact field buys almost nothing in *accuracy*. Every bit of
this idea's payoff is in **throughput of predictions**, and that payoff is zero unless
something consumes predictions faster than fetch does.

### 3.3 Which means: a queue between the predictor and fetch

A prediction per block is useless while the predictor produces one per cycle and fetch
consumes two instructions per cycle — they are matched, and the predictor simply idles
more. The structure that cashes it in is a **fetch target queue**: predictor → FTQ →
the fetch buffer's *request* stream.

The existing request logic wants exactly this. Today `fb_want` is derived from where the
PC **is** — `fb_al`, `fb_pa1..3`, gated by `fb_samepg*` — and item 10e's entire content
was "run two chunks ahead, two requests in flight". With an FTQ the request stream comes
from where the **predictor** has got to, and the run-ahead distance is set by the queue
depth and the block length, not by three chunk slots.

**Cap a block descriptor at a chunk (or line) boundary** so one FTQ entry is exactly one
I$ request; a long block becomes several entries. Then the FTQ *is* a fetch-request
queue, and line-granular instruction prefetch falls out of it rather than being a
separate mechanism.

### 3.4 The correction question

"We don't know the correction until the following branch is seen (or we can update
twice)." Sorting the failure modes says where each correction actually lives:

| error | detected | costs |
|---|---|---|
| wrong direction / target | resolve, as today | a full mispredict, unchanged |
| extent **too long** | decode: the aligner cuts at the first CTI regardless | wasted FTQ entries and I$ requests; `apc` wrong once |
| extent **too short** | decode: `cti_ok` says the bundle does not end on a CTI | `pred_v = apred_v & cti_ok` already drops it — a lost prediction |

So both extent errors degrade to *lost predictions plus wasted fetch*, never to a wrong
architectural result — the same degradation property §4.1 already leans on. And the
correction is **available at decode**: the aligner is the authority on where the block
really ends, and the design already carries the terminating CTI's offset from the base
in the predict details (`BOW`, PDW 16 → 18) so that training can recompute the key.

**The second update is not an optimisation, it is the forward-progress argument.**
Resolve-only training does eventually learn the extent — every block ends in a CTI that
resolves — but it leaves the entry wrong between the decode-time redirect and that
resolve, and the re-fetch reads the same stale entry and takes the same wrong turn. That
shape has already hung this design once: `fetch.v` records the 2026-08-23 attempt that
indexed from `norm_npc` while stamping with `npc`, where "a redirect could read a stale
entry stamped with the redirect target, and the mispredict it caused re-read the same
stale entry — a loop that never resynchronised (it hung `rv64mi-p-illegal`)". So either

- **update the extent at decode** — the BTB is BRAM, so the second port is a true
  dual-port port, and the two writers touch disjoint fields (extent/class vs
  direction/target) so per-field write enables keep them apart; resolve wins a
  same-cycle tie — or
- **carry the corrected extent forward past the redirect**, so the re-fetch does not
  consult the entry at all.

Pick one deliberately and write down which. An always-on assertion that the same block
base takes the same wrong extent twice in a row is the cheap detector for having picked
wrong.

### 3.5 Where it composes — and why it probably comes first

- **With idea 1.** The FTQ hands over the *next block's start address* many cycles
  before its bytes are wanted, so a leaf crossing can be translated early and the §1.7
  bubble disappears — including for the ping-pong residue §1.4 is about. §1.8's whole
  ladder (the FTR file, PPNs in the redirect, the RAS and the BTB) is a set of
  workarounds for **not knowing the next fetch address early enough**. An FTQ knows it.
  If idea 3 lands, most of §1.8 should never be built, and §1.8 item 0 shrinks to
  "enough FTR entries to cover the queue's live leaves".
- **With idea 2.** Idea 2's packet header holds base PC, length, slot count, `br_term`,
  tail CTI class and fall-through PC. Idea 3's entry needs length, tail CTI class,
  target and direction state, keyed by the block start. **These are the same object.**
  One tag array keyed by the block's start PC with two payloads: the predictor's
  (direction, target, counters) and the packet's (slots). The predictor names the next
  packet; the packet cache says what is in it — and a packet-cache hit *is* the exact
  extent, which is §3.4's decode-time update for free, with no second BTB port.

That composition is the argument for doing idea 3 **before** idea 2: it is much smaller,
it de-risks idea 1 by removing most of §1.8, and it builds the key that idea 2's cache
is indexed by.

### 3.6 What it costs

- A length field on the BTB entry: 5–6 bits if capped at a chunk or a line, which is the
  cap that makes an FTQ entry one I$ request. The BTB is 1 024 × 53 bits in 1 RAMB36 +
  1 RAMB18 today; this fits without a second tile.
- The FTQ itself: a shallow queue of {block start VA, length, tail class, predicted next}.
- `lenp` (4 096 × 2 bits of LUTRAM) is deleted.
- A second BTB write port, or the carry-forward alternative (§3.4).

### 3.7 The measurements

Two of them are already in §2.11 — the same pass over the same trace:

1. **Basic-block length distribution** (§2.11 item 1). It sets both the packet size and
   the predictions-per-instruction ratio, i.e. how much run-ahead there is to have.
2. **How far ahead the predictor gets**, simulated: run the FTQ against the trace and
   find the depth at which run-ahead saturates. Beyond that depth the queue is area for
   nothing.
3. **What fraction of I$ misses that run-ahead would convert into prefetches.** This is
   the number that decides whether the FTQ pays, and it is the only one of the three
   that cannot be guessed from the block-length histogram.
4. `lenp`'s current 0.002 redirects/1 000 is the **accuracy baseline to not regress**;
   idea 3 must be measured as neutral there, not better.

### 3.8 Staging

- **3a — the extent field.** Add it to the entry, train it (decode or carry-forward per
  §3.4), delete `lenp`, keep everything else. Pure plumbing; the gate is *no regression*
  against 0.002 redirects/1 000, not an improvement.
- **3b — the FTQ.** Predictor decoupled, fetch requests driven from the queue instead of
  from `fb_want`'s view of the PC. This is where the win is.
- **3c — pre-translate from the FTQ**, folding most of §1.8.
- **3d — index the packet cache from the FTQ** (idea 2).

---

## Status log

- 2026-09-06: idea 1 written from a read of `ooo2/rv_soc_top.v`, `ooo2/ooo2_core.v`,
  `src/fetch.v`, `src/mmu.v` and the impl_1 routed timing report. Nothing measured.
- 2026-09-06: idea 3 (predict basic blocks) written. Its value is run-ahead, not
  accuracy — `lenp` is already at 0.002 redirects/1 000 — so it is worth nothing without
  the FTQ of §3.3. It composes hard: an FTQ removes most of §1.8, and its entry and idea
  2's packet header are the same object, which argues for doing it first.
- 2026-09-06: §1.4's "calls and returns stop crossing" was wrong — text spans leaves,
  and PLT/libc/syscall control flow crosses at any leaf size, twice per call. Replaced
  with what still crosses, and the ping-pong shape of the residue produced §1.8 item 0
  (a 2–4 entry FTR file), which is cheaper than the RAS and BTB items and now precedes
  them. §1.9 asks the study for the crossing *shape*, which is what sizes it.
- 2026-09-06: leaf vs frame separated throughout §1 — validity is per leaf, only the
  address concatenation is per 4 KiB frame; a frame crossing inside a leaf is not a
  re-translate. Fixed the buffer-tag width in §1.10 (17 bits within a 2 MiB leaf, not
  8 within a frame — capping at a frame would put back the realign §1.4 removes), made
  §1.9's crossing-rate measurement report every granularity, and named the two 4 KiB
  constants in the existing RTL (`hw_bound`, `fb_samepg*`) that have to follow.
- 2026-09-06: idea 2 (the "VLIW" packet cache) written. Nothing measured; §2.11 is the
  study that has to come first.
- 2026-09-06: §1.4 reworked around the 2 MiB leaf — the tracked unit is the leaf, and
  on 2 MiB kernel text the crossing cost of §1.7 case (4) goes to zero, which is what
  decides whether §1.8 is ever built. Materialised-PPN rule added to §1.2.
