# Stage 3 implementation design — parametrizable pipeline width (the 3-wide stretch)

Branch `wip/fe-stage3` off `main` (`0a1e2de2`, Stage 2 tip). Parent: `PLAN-2026-09-13-frontend.md`
§"Stage 3". Prereq context: Stages 1–2 landed and board-clean (`docs/PLAN-2026-09-13-frontend-stage2.md`).

## Goal and the honest framing

Make pipeline width a **real parameter** (`OOO2_IW`, default 2) and restructure rename so the
machine can be built 1/2/3-wide from one source. **`OOO2_IW` never existed** — width today is a
hardcoded `localparam FW = 2` in `ooo2_frontend.v`; only the fetch window `HW` flows top-down.

Be honest about payoff:

- The **current 2-wide already closes 166.667 MHz** (Stage-1 board WNS +0.011, VHPR builds
  +0.021/+0.033; the rename/scheduler families sit at ~+0.05–0.14 slack, behind the FPU-FMA and
  store-queue worst paths). So at `IW=2` the restructure buys **parametrizability + cleanliness
  (+ maybe a little headroom), NOT IPC** — same width is same IPC.
- **The IPC bet is entirely `IW=3` closing timing.** With width a real knob, 2-wide and 3-wide
  stop being separate efforts: build the parametric machine, land it **board-clean at `IW=2`
  first** (proving the restructure does not regress the working machine), then **`IW=3` is a
  one-build synthesis experiment** — flip the knob, build, read WNS. `IW=2` is the guaranteed
  deliverable; `IW=3` the cheap stretch on top. **3-wide @ 166 is a stretch goal, not a
  commitment.**

Risk to respect: the restructure *touches the working 2-wide* — the flop speculative livemap's
W-way select and the new cross-pipe sibling-forward are new paths even at `IW=2`, spending that
+0.05–0.14 of slack. So each `IW=2` increment must stay board-clean, and gate-0 spikes come first.

## Gate-0 — two OOC spikes BEFORE any RTL (rule: confirm before building)

1. **3rd-ALU scheduler entry.** OOC `ooo2_iq` at the incumbent `NWB=4` vs the 3-wide `NWB=5`
   (5th wakeup broadcast), for the ALU sched (`NENT=10,NSRC=2,FIXEDL=1`) and the heaviest
   M-class (`NENT=12,NSRC=3,INORDER=1`), 6.000 ns both directives. If `NWB=5` falls off the
   cliff, the 3rd ALU is the gate regardless of how clean the rename is. Answers the "major
   question" in an hour, not a stage.
2. **Restructured rename path @ `IW=2`.** A throwaway spike of the flop speculative-livemap read
   → W-way/sibling select → issue-queue write cone. Does the restructure *keep* 2-wide's
   headroom? If it regresses the currently-comfortable machine for zero 2-wide IPC, we do not
   start — we keep the working 2-wide and take IPC from the back-burnered MLP recovery instead.

Go/no-go: (1) close-ish and (2) not a regression → build. Either fails badly → stop, one hour spent.

**Spike 1 RESULT (2026-09-14, `ooo2_iq` OOC @ 6 ns):** ALU sched `NWB=4` WNS +1.571 / `NWB=5`
+2.133; M-class `NWB=4` +3.209 / `NWB=5` +3.158. The 5th broadcast is intrinsically free (the
delta is OOC noise), so there is **no logic-depth wall** in the scheduler for the 3rd ALU. Caveat
(rule I2): OOC is route-blind and optimistic — the routed scheduler families sit at ~+0.05–0.14
ns, so this refutes a fundamental wall but does NOT prove the routed 3-wide closes; that is
increment 5's full build. **Verdict: proceed.** Spike 2 is folded into the increment-4 board gate
(the real routed regression test for the restructured rename path) rather than a throwaway module.

## The architecture — steer-then-rename with decode-time dependency resolution

The width-hard part of rename is the O(W²) intra-bundle bypass and the multi-write map. Move the
steering **before** rename so rename becomes scalar per pipe:

1. **Aligner** — bounds the group (cuts at CTIs, ≤W), as today. It CANNOT class-chop (steering
   by issue queue needs each op's decoded class), so the chop is the decoder's.
2. **Decoder** — decodes each op, **numbers the uops** (program-order age), does the
   **pre-rename intra-group dependency resolution** (per source → *from map* or *from sibling
   uop k*; per arch-reg → its *youngest* writer in the group, for WAW), and **steers** the group
   into pipe lanes with **≤1 per issue queue** (this is today's `d2_hold` "different scheduler
   than A", generalized to W — it is exactly what keeps every `ooo2_iq` at one write port).
3. **Per-pipe scalar rename** — each pipe renames its own ≤1 uop: read the map for its sources,
   pop one preg from *its* shard's free list, write its dest to its map copy; the decoder's dep
   result picks map-vs-sibling per source (a small mux, not a live comparator).

Execution: **one added ALU pipe** — 3 symmetric ALU pipes + the existing shared M/F issue port.
That is +1 wakeup broadcast (`NWB` 4→5) and a 3-producer ALU bypass. `mul/div/FPU/LSU/CSR` stay
single (shared through the one M/F port). The 3rd-ALU timing (wakeup/bypass) is issue-side and
**orthogonal** to the rename restructure — steer-then-rename does nothing for it; that is what
gate-0 spike 1 measures.

## Per-structure treatment

**Width knob.** Introduce `OOO2_IW` (default 2), thread it top-down like `OOO2_HW`. Derive
`localparam NBANKS = 1 << $clog2(IW)` — the **next power of two** (IW 1/2/3/4 → 1/2/4/4). This is
the banking answer: keep `bank = i[log2(NBANKS)-1:0]`, `index = i >> log2(NBANKS)` as free
bit-slices even at IW=3 (mod-3 never appears), because ≤W ≤ NBANKS consecutive positions are
always distinct mod NBANKS. `IW=3` and a future `IW=4` share the 4-bank layout.

**Fetch + aligner** — already fully `IW`-parametric (`src/fetch.v`, `src/aligner.v`; run at IW=4
in tbs). Set `IW=OOO2_IW`. Trivial.

**Decoupling queue + ROB** — identical banking problem (both are queues: W consecutive writes at
the tail, W consecutive reads at the head). `NBANKS=next_pow2(IW)`, one muxed `{we,addr,data}`
per bank (rule: one write statement per LUTRAM bank, else Vivado demotes to flops), an `NBANKS:1`
read mux per head. The only difference between them is payload width.

**Rename.**
- **Speculative live map: flops**, W write ports (each rename writes its dest, WAW resolved by
  the decoder's youngest-writer pick), combinationally read by every source port. This is the
  rename-path timing suspect and the thing that grows with W (a 2-way select → a W-way pick).
- **Committed map: read-replicated LUTRAM** (like the PRF) — cold, read only for recovery.
- **Intra-group dep resolution in the DECODER** (a stage with slack) → the O(W²) comparison is
  off the rename critical path; its compact per-source result feeds the pipes.
- **Free lists → per-shard BITVECTOR** (not FIFO/parity queues). Return = OR the freed pregs'
  one-hots into the free vector (any number of writers, no ports, no banking). Alloc =
  find-first-set (≤1/shard under steering → a plain priority encoder). **Recovery** = keep a
  committed-free shadow bitvector (updated at commit) and bulk-copy `spec_free ← committed_free`
  on redirect — **precise and single-cycle because redirects fire only from the ROB head**, so
  the branch IS the commit point at redirect (§Only-M-redirects). *Deferred future item:* faster
  mispredict recovery (redirect BEFORE the head, plan item 5 / the `RD_WAIT` drain) would need
  mid-window rollback of the bitvector (per-branch snapshot or reconstruct) — a bitvector's one
  weakness, cleanly coupled to the one optimization we punt.

**Issue / exec (the `IW=3` step).** +1 ALU scheduler (`u_iq_i3`), +1 ALU exec (`u_xc`), +1 PRF
write shard (`SH_IE3`), +1 wakeup broadcast (`NWB`=5). The ≤1-per-queue chop keeps every
`ooo2_iq` single-write, so each is just another instance.

**PRF / bypass / pending.** 5th write shard + its free list; +2 read ports for the 3rd ALU; a
3rd producer arm on every operand mux; a 5th broadcast + a 3rd alloc port on `ooo2_pending`.

**Retire.** ROB commit width = W (`head`, `head+1`, `head+2`), the writer-shard commit map and
free-return following.

## Increments (each: lint → 240/0 → cosim ±0.5% → census; board on every `IW=2` increment)

0. **Gate-0 spikes** (above). Go/no-go.
1. **Introduce `OOO2_IW`** (default 2) + convert the already-generic structures (fetch/aligner
   `IW`; `generate` the `ooo2_iq`/`ooo2_exec`/PRF-shard counts). **Retire-identical at IW=2.**
2. **`NBANKS=next_pow2(IW)` banking** on the decoupling queue + ROB. Retire-identical at IW=2
   (2 banks == today's parity). Board-clean.
3. **Bitvector free lists** (per shard) + committed-shadow recovery, replacing the parity-banked
   FIFO free lists. Retire-identical at IW=2. Board-clean.
4. **Steer-then-rename** — decoder numbers + resolves deps + steers; per-pipe scalar rename; flop
   speculative livemap. The big one. **Retire-identical at IW=2** (same width, restructured) —
   this increment proves the restructure does not regress the working 2-wide. Board-clean.
5. **Flip to `IW=3`** — the 3rd ALU pipe/shard/scheduler/broadcast; NBANKS already 4. The
   stretch. Full census + `make timing` + board; measure WNS and the IPC delta. If it does not
   close, `IW=2` is the shipped deliverable and this stays a documented experiment.

## Not in Stage 3 (deferred)

- **Faster mispredict recovery** (redirect-before-head, plan item 5 / `RD_WAIT`; the bitvector
  mid-window rollback it needs). Tracked separately.
- **The −8.6% adapter-MLP recovery** (Stage 2 follow-up) — orthogonal, still separate.
- `IW=4` — the banking and shards already support it; a later experiment if `IW=3` closes.
