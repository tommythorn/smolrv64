# Handoff — 2026-08-29: store buffer, load queue, and 166.67 MHz at NF=8

`main` is at **`6642e63`**, verified green. Work in progress is parked on
**`wip/pipelined-loads`** (`0681b57`) and **does not work** — see §4.

> **SUPERSEDED IN PART.** §7 item 3 has since been done — see the note on it, which
> corrects this document's framing of what the load queue's two cycles cost. Camera is
> **18.08** cyc/elem, not the 19.08 quoted in §1 and §3, and `ldbench` is 5.00/4.00, not
> 6.00/5.00. `docs/OOO2-Spec.md` sections 8 and "Camera" carry the current numbers;
> everything else here still stands.

---

## 1. Where things stand

| | |
|---|---|
| `main` | `6642e63`, lint clean, **240/240**, **215/215**, Linux cosim 40 M cycles no divergence |
| Fmax | `probe_clk` **+0.015 ns** at `NF`=8, BTB 1024, 118 of 480 BRAM tiles (built at `394ef24`) |
| Camera (`workloads/saxpybench`) | **19.08 cyc/elem**, from 22.08 at the start of the session |
| Bitstreams | `/var/tmp/ooo2_NF8_btb1024_166MHz_394ef24.bit` (current), `ooo2_NF8_apcfix_166MHz.bit`, `ooo2_NF5_166MHz_a33ad1b.bit` |

Nothing has been programmed to the board this session. The banked bitstream at
`394ef24` predates the store buffer and load queue.

---

## 2. What landed

Timing, in order:

- **`0b53c4b`** — `apc` is register-only again. `cti_ok` (the aligner's `br_term`) was
  ANDed in at the top of `ooo2_predictor`'s predict cone, so it reached `pred_v` and,
  through `p_ret`, `pred_tgt` — the two things `apc` selects on, and `apc` is the BTB
  read address. The routed path was 5.521 ns of 6.000, 70% route, ending at a BRAM
  address pin. **Recorded as rule I6**: a late signal may reach a RAM's enable, never
  its address. Cost: 1 retire in 10.4 million.
- **`2e32d52`** — `mhpmcounter` takes a delayed retire count; `minstret` keeps the live
  one. Zihpm counters are permitted arbitrary read latency.
- **`47e1d26`** — `NF` 5 → 8. The minimum-8 scheduler policy was never the scheduler's
  fault; NF=8 failed on the frontend `apc` cone and nothing else.
- **`7926354`/`e2642d5`/`394ef24`** — BTB 256 → **1024**, and `PDW` 44 → **16**.
  `bidx`/`btag`/`ytagf` are pure functions of the branch's own PC, which the pipeline
  already carries, so they are recomputed at resolve instead of riding along. Only
  `yidx` still rides (it folds the predict-time GHR). **4096 was built and reverted**:
  it cost **434 ps**, and not where predicted — no BTB path failed at all; the extra
  five RAMB36 pushed `m_addr -> u_rs_i/i_ps*` and `i_ps2 -> u_prf` over, both already
  at 0.000. That is rule I1 verbatim, and I1's own worked example is `u_prf/mem_ie`.

Memory:

- **`ec6ad3e`** — store buffer (`ooo2_sq`) wired in. A store issues on its **address
  alone**; data arrives by snooping the writeback ports. **Camera 22.08 → 17.08
  cyc/elem, −22.6%**, 49.2% of loads reordering past an uncommitted store.
- **`cb02868`** — load queue (`ooo2_lq`) wired in. Costs **+2.00 cyc/elem exactly**
  (one cycle to register the address, one to select), taking Camera to 19.08 and
  reordering to 0%. Deliberate: it moves the alias test off the translate path and is
  the structure pipelining needs.
- **`6642e63`** — restores a `$finish` that `cb02868` deleted. See §5.

Analysis:

- **`eed2ea2`** — all 13 GB5 traces, plus `tools/trace-bp.py`, which replays a trace
  through the real BTB/RAS and splits misses into cold vs conflict.

---

## 3. The numbers worth trusting

Linux boot, 40 M cycles, `VDEFS="-DOOO2_HW=4"`, no divergence at any point:

| state | retires |
|---|---:|
| before the BTB work | 10 444 328 |
| BTB 1024 | 10 496 381 |
| + store buffer | 10 513 562 |
| + load queue (`cb02868`) | 10 315 103 |

`workloads/saxpybench` (the Camera kernel), RESIDENT:

| state | cyc/elem |
|---|---:|
| session start | 22.08 |
| store buffer | **17.08** |
| load queue | 19.08 |

`workloads/ldbench` at `cb02868`: latency **6.00** cyc/load, throughput **5.00**,
overlap **1.19x**.

Measured and rejected, so nobody repeats them:

- **ROB 16 → 32**: `ST_ROB` 58% → 0%, cyc/elem unchanged at 17.08. The ROB fills
  *because* something downstream is slow.
- **`NENT` 4 → 8** (store buffer): "full" from 426 514 cycles to 62, cyc/elem
  unchanged. Kept at 8 anyway — Linux boot was full 29% of cycles at 4.
- **BTB 4096**: better on every workload metric, costs 434 ps. Revisit only when
  `m_addr -> scheduler` has slack.

---

## 4. `wip/pipelined-loads` — parked and broken

Branch `wip/pipelined-loads` (`0681b57`) holds steps 2+3 of load pipelining:
multi-outstanding LSU load path plus load-queue-index read tags.

**It passes 240/240 and 215/215 and STALLS the Linux cosim**: 1 518 881 retires in
40 M cycles (0.038 IPC), `M-empty` 38.6 M, `d_hold` 38.0 M — and `d_hold` is not
`src_pend` or `rob_full`. At the timeout, `rtag_q` names one load while the response
carries another. At least one defect remains in the tag/serialisation path.

Four bugs were found and fixed on that branch, and **three are the same shape — a
signal that did not mean what its name said**:

1. `rtag_q` — `pt_tag` is the queue's *current* candidate and `mem_ren` is registered,
   so the cache latched `r_tag` a cycle after issue, naming the next entry.
2. `dc_rv_ok` — matched only the requester field, so it never went stale; it gates
   `c_rd_req`, so one delivered response blocked every later request.
3. `ld_fast_ok` — not gated by `start_ok`, so a *refused* access was still marked
   outstanding.
4. `pt_ack` — was `pt_start`, not "accepted". The queue advances its access pointer on
   it (rule D5).

Two of those were jobs `lsu_gen` had been doing. It was deleted while quoting the
comment that explained why it existed.

**Next step for whoever picks this up:** dump `o_v`/`rtag_q`/`acc`/`head` **per cycle
around the first stall**, not at timeout. The timeout snapshot shows the symptom, not
the transition, and that cost several iterations.

---

## 5. Facts that cost time — read before repeating them

**The D$ is the load-throughput floor, and the spec overstates the tag.**
`rv_cache.v` is a **single-request FSM**: `S_IDLE` latches `r_addr`/`r_tag`/`cur_line`
and addresses the banks, `S_CHECK` computes `hit` combinationally from `cur_line`,
returns, and goes back to `S_IDLE`. Two cycles per hit, one request at a time.
`rd_tag`/`rd_resp_tag` distinguish **requesters** (LSU=00, I$/prefetch=01/10, see
`rv_soc_top.v:901`), not requests in flight. The spec's "the load queue IS the tag
space the D$ already reserves" is true about the tag *width* only.

Pipelining therefore needs `S_CHECK` to self-loop: return the current data (the old
`r_*` are safe — nonblocking) while latching the next request and re-addressing the
banks from `a_live`. Feasible precisely because `hit` is combinational from
`cur_line` rather than a registered tag-RAM output. That is **step 1**, not started.

> **CORRECTION, measured.** The self-loop is written (`wip/cache-selfloop`, `baf96df`) and it
> is **not a cache-only change**. Alone it makes the cache do every lookup TWICE -- `ldbench`
> `D$acc` 1.6 M -> 3.2 M for the same loads, Linux -2.9%, `saxpybench` `FE_BUB` 0% -> 43%,
> cosim clean because the defect is waste, not corruption. Both requesters drop their request
> on the cache's REGISTERED `rd_valid`, so during the `S_CHECK` cycle that produces the
> response the request is still asserted and gets re-accepted. It needs an `rd_ack` and a
> requesting/outstanding split at every requester (rule D5) -- and even then buys nothing
> until a requester can stream, because all three are single-outstanding. See
> `docs/OOO2-Spec.md`, P0.

**The cosim does not check a store it thinks made no access.** `probe_cosim.cpp:285`
skips the address compare whenever *either* side reports `mem_kind == 0` — deliberate,
so a model reporting no access never forces a false abort. A buffered store's M pass
only translates, so `cs_mkind` was capturing stale `lsu_cos_*`, often zero, and every
buffered store sailed through unvalidated. Fixed in `ec6ad3e`; the first "clean 40 M
run" before that fix proved nothing.

**`tb_ooo2_riscv` models memory as `dmem_rvalid = 1'b1`** — permanently valid, no real
tag. It cannot catch tag-matching bugs, which is why `wip/pipelined-loads` passes
240/240 and stalls the SoC cosim. Any change to the memory tag scheme must be gated on
the **cosim**, not the 240 suite.

**Loads must carry a store-seqno.** Store-buffer entries are allocated at *dispatch*,
so the buffer also holds stores **younger** than a load sitting in M. Blocking on those
is a circular wait — the load waits for a younger store's address, and that store
cannot execute because `u_rs_l` is in-order and the load is at its head. It hung 115 of
240 tests. The load carries the tail it captured at dispatch; only entries nearer the
head count.

**A redirect fires only at the ROB head** (`redirect` is gated by `head_block`, and
`m_needs_head` includes `m_redirect | m_is_sys | m_is_fencei`). Every older instruction
has therefore already retired, so wholesale flush of `ooo2_sq`/`ooo2_lq` is *correct*,
not merely convenient. Worth re-deriving before changing either flush.

---

## 6. Process failures this session — the expensive ones

**Diff a line-range edit; do not eyeball it.** `sed -i '134,149d'` removing debug
`$display`s took one line too many and deleted `$finish` from `tb_ooo2_riscv.v`. Every
test then ran forever at 100% CPU with no output. I re-read the region and it looked
correct, because a removed line leaves nothing behind. This cost roughly two hours and
put a broken commit on `main`.

**Never commit on a gate you did not watch finish.** `cb02868` carried "240/240,
215 failures: 0" from a run made several edits earlier; the confirming run exceeded a
tool timeout and was killed. That run is exactly what would have caught the `$finish`.

**`pgrep -f` / `pkill -f` match their own command line.** Already in the build memory
for `pkill`; it applies to `pgrep` too. I used it to check for stale jobs, got the
self-match back, and reported the machine clean — while **129 simulator processes** from
a killed suite had been running for over three hours with load average **119**. Use
`pgrep -x <name>` / `pkill -x <name>`.

**`( cmd & )` inside a tool call does not survive the call.** Builds launched that way
are killed mid-write, leaving a missing or partial binary. Several "the simulation
hangs" conclusions were runs against a binary that was not there. Use the harness's
background mechanism, or run in the foreground with a timeout.

**Instrument earlier.** Three separate bugs were mis-diagnosed by reasoning from RTL
(BTB address fanout; the placer directive; a delta loop) and then found in one shot by
dumping state. In each case the giveaway was arithmetic that could not be true —
`sqtag=1`, `head=1`, yet `ld_dist=1`.

---

## 7. Suggested order from here

1. **Fix `wip/pipelined-loads`** (§4). Per-cycle dump around the first stall.
2. **Step 1 — the cache** (§5): `S_CHECK` self-loop for back-to-back hits. This is what
   turns the throughput on; steps 2+3 gain ~0 without it. Gate on the **cosim**.
3. **Bypass the queue when no older store is live.** — **DONE**, but read this before
   believing the framing above. It recovers **1.00**, not 2.00: Camera 19.08 → 18.08,
   ldbench latency 6.00 → 5.00, throughput 5.00 → 4.00, Linux boot +1.81%.
   The first attempt took the framing literally — drop the entry, let M hold the load
   through its access, the pre-queue shape — and it was a **regression**: ldbench
   6.00 → **7.00**, Camera unmoved. The queue's two cycles are not both overhead. The fill
   cycle is M's EARLY RELEASE, and letting the following non-memory instructions execute
   while data is in flight is worth more than the latency it costs. What works is to keep
   fill, release and land exactly as they are and move only the access START into the
   translate pass (`req_early`). Details in `docs/OOO2-Spec.md` section 8.
4. **Speculative parallel TLB / cache / alias.** The delicate one; do it last, against a
   correct implementation to check it.

Camera's remaining wall is `ST_MEM` at 64–76% with the LSU single-outstanding. Neither
ROB depth nor buffer depth moves it (§3).
