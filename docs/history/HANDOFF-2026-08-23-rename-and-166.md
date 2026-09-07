# Handoff, 2026-08-23 — rename landed; 166.67 MHz is 52 ps short on the predictor

## State

Milestone 1 (register renaming + a sharded physical register file) is **complete and
verified**, including on hardware. Issue and commit are still in order; this milestone
changes no architectural behaviour and exists to remove the single-write-port blocker.

| gate | result |
|---|---|
| `src/lint.sh` | clean |
| `ooo2/run-ooo2-vl.sh` | pass=240 fail=0 |
| `src/run-vl-tests.sh` | failures: 0 |
| ubuntu-mini cosim | 3 runs, **26e9 retirements**, zero mismatches, zero assertion fires |
| retire stream vs pre-rename | **bit-identical** at c=1e9/2e9/3e9 |
| hardware GB5 | IPC 0.238 vs 0.238, CPI 4.200 vs 4.199 |
| timing | **+0.031 ns at 111.11 MHz** (shipped), −0.052 at 166.67 |

Bitstreams: `/var/tmp/inorder_rename_111MHz_e50a9277.bit` (booted, ran GB5),
`/var/tmp/inorder_rename_166MHz_e9ce9dfb.bit` (closed at +0.003 — see "Do not trust" below).

---

## THE IMMEDIATE TASK: close 166.67 MHz

Best measurement: **WNS −0.052 ns, TNS −0.98** (`b20a8d93`, `PLACE_DIRECTIVE=Explore`).
TNS is under a nanosecond, so very little is failing.

The limiter is the **predictor cone**, and the caches are no longer involved:

```
u_fetch/strad_reg_replica_4 → u_bp/ycorr_qv     6.462 ns   logic 2.23   route 65%
u_lsu/mem_raddr_reg[13]     → m_result_reg[14]  6.195 ns   logic 1.82   route 71%
```

`ooo2_predictor.v:259-263` reads the arrays a cycle ahead:

```verilog
btb_q    <= t_fwd ? ... : btb[bidx(npc)];
btb_qpc  <= npc;
ycorr_q  <= y_fwd ? ... : ycorr[yidx(npc, ghr)];
```

and `src/fetch.v:174` builds `npc` from a mux containing `redirect_pc`, `irq_inject`,
`strad`, `straddle_det` and `norm_npc`. That mux feeds the array **index**, which is the
failing path.

### What was tried and FAILED — do not repeat without a mechanism

Indexing the arrays from `norm_npc` (= `pred_v ? pred_tgt : ft_npc`) instead of `npc`, to
keep `strad`/`redirect`/`irq_inject` out of the index path. Reasoning was that a stale read
is safe because the hit test requires `btb_qpc == base_pc`.

**It hangs `rv64mi-p-illegal`** — 5e6 cycles, no completion. Reverted. The mechanism was
never established. The read-a-cycle-ahead structure exists *precisely* so the prediction
lands on the cycle fetch needs it; changing the read address so it no longer tracks `npc`
breaks something beyond losing predictions.

### The real change

A predictor **stage** means the prediction for PC_n arrives at cycle n+1, after fetch has
already moved on. So it stops being a next-PC select and becomes a **late redirect** that
squashes the sequential fetch behind it:

* one bubble per predicted-taken branch (~10–15% of instructions → 0.10–0.15 CPI, ~3% of
  the current 4.2)
* fetch needs a second redirect port, distinct from the backend mispredict redirect
* interacts with the F/X queue (`ooo2/ooo2_frontend.v`)

Tommy's position, 2026-08-23: **frequency is critical and latency is an acceptable price**,
and BP accuracy is a moving target — "BP can be improved. What we have now is just the
beginning." So do not size this against today's predictor quality.

### Also on the table

`u_lsu/mem_raddr_reg[13] → m_result_reg[14]` at 6.195 ns is nearly as bad and is a
*different* cone. Closing only the predictor may just expose it.

---

## Do not trust: a single build's WNS

Measured placement spread on identical RTL: **81 ps on the baseline, up to 400 ps with
rename**. Directive ordering is not even stable — `Explore` and `AltSpreadLogic_medium`
have swapped which is better between builds.

The `+0.003` "MET" at `e9ce9dfb` is three picoseconds on one directive out of two, with the
other at −0.333. That is a lottery ticket, not a close. **Always build two directives**;
treat anything under ~100 ps as noise. `Explore` is the default (`dc3fb0ad`) because it
measured best on the baseline, not because it always wins.

---

## What actually moved timing (and what did not)

| change | effect | verdict |
|---|---|---|
| `3c5936a3` PRF arrays sized per shard | **+465 ps** | real — a bug: `mem_ie` was 128 deep holding 64 registers, so unreachable LUTRAM was being synthesised |
| `b20a8d93` caches 128 KB → 64 KB | **+50/+189 ps**, both directives | real — and it moved the limiter off the caches |
| `2792fc08` write-through out of the read path | −83/+53 ps | noise |
| `e50a9277` FP-int to SH_LD, N_FE halved | noise | later reverted for a cleaner writer split |
| `e9ce9dfb` shared write bus | −191/+103 ps | noise; **its commit message claims a fix that is not one** — see `f6501c60` |

Three theories of mine died on the way here — "congestion", "3× write fanout", "PRF area" —
and what settled it was opening the routed checkpoint and asking where the cells are:

```
u_prf/mem_*     X 81..95   Y 128..165     (compact, one corner)
u_dcache/banks  X  6..72   Y  29..193     (most of the die, no floorplan)
```

The D$ paths were never failing *because of* rename. The cache had no floorplan, its worst
internal route was a placement lottery, and rename re-rolled the dice. **Open the checkpoint
before theorising.**

A pblock on `u_dcache`/`u_icache` was proposed and not tried — 64 KB may have made it
unnecessary, but it is still the cheapest untried lever (constraints only, no RTL, no cosim
cost).

---

## The design, in brief

**PRF** (`ooo2/ooo2_prf.v`) — three shards, one writer each, so no write arbitration:

| shard | writer | holds | entries |
|---|---|---|---|
| `SH_IE` | ALU / CSR | integer | 64 |
| `SH_LD` | LSU, mul, div | integer **and** FP | 128 |
| `SH_FE` | FPU | FP **and** integer (`fcvt.w.d`, `fmv.x.w`, `fclass`, compares) | 128 |

Each shard is 3 LUTRAM copies (rs1/rs2/rs3), so 9 arrays. **Each shard takes its own
writer's data** (`wb_ie`/`wb_ld`/`wb_fe`, `20d5dcca`) — routing everything through the
global `m_wb_val` mux sent the D$ read data to all 9. The address is shared; there is one
writeback per cycle.

Sizes are a **correctness floor**, not a preference: a shard sized below what can map into
it deadlocks rename (everything mapped, nothing free → nothing commits → nothing is freed).
Checked at elaboration. They must also be powers of two — the free-list pointers carry one
extra MSB and index with the low bits.

**Map** (`ooo2/ooo2_rename.v`) — `lv[a] ? SMAP[a] : RMAP[a]`; rename writes SMAP and sets
`lv`, commit writes RMAP, rollback is `lv <= 0` plus three pointer restores. One control
signal to 64 flops instead of `docs/Area-Efficient-Scalar-OoO.md` §9.2's 32-wide bulk copy,
which is the fanout that hurts on FPGA.

**Departure from §9.1:** that scheme derives the free-list tail from a fixed occupancy,
which holds only because exactly 32 architectural registers are mapped at all times. Per
shard the mapped count varies 0..64, so each shard carries a real tail pointer. Rollback is
still pointer-only.

**`WRTHRU=0`** — write-through is dead code with in-order issue (the collision is exactly
`byp1/2/3`, where `x_rs` takes `m_byp_val`). `ooo2_core` asserts on an unbypassed read that
collides with the writeback, naming `WRTHRU` as the fix, so enabling it for OoO issue is not
something to remember.

**`rv_regfile` is still instantiated under `ifndef SYNTHESIS`** as an every-cycle
cross-check. It caught five defects (four rename bugs plus a missing `+a1=` boot seed) and
costs nothing in hardware. Keep it until the design has run the cosim as long as the shadow
version did.

---

## After 166 MHz

1. **ROB + `pending[]` bits.** LSU 38.8% + FPU 28.5% = **67% of GB5 cycles**, and 94–98% of
   the LSU share is hit latency, not misses (3.24 cycles per D$ access; miss rate 0.195%).
   `ooo2_core.v:703` — `m_done = ... m_mem_op ? lsu_done ...` — blocks M until the unit
   finishes. Rename removed the reason that had to be true; the ROB removes the blocking.
   The ROB is needed for **precise exceptions** once younger instructions complete ahead of
   a load. Expect 0.5–1.0 CPI.
2. **Headroom first, again.** 111 MHz is only +0.031 with rename (baseline was +0.091/+0.107).
   A ROB is real logic in the issue path and there is nowhere to put it.
3. **Missing counters**, all one `hpm_ev` bit each (the bus is registered, so free):
   * branches executed — BP accuracy is currently **unmeasurable**; `RED_BR` exists but has
     no denominator. This sets the latency budget for a predictor stage.
   * predicted-taken — the bubble cost of that stage.
   * M stalled with the next instruction independent — the directly recoverable fraction,
     which would replace the 0.5–1.0 CPI estimate with a number.
4. **UltraRAM for the I$** — 64 KB is 2 URAM288 in one column versus dozens of scattered
   BRAMs. Its mandatory output register is a stage you would be adding anyway at 333 MHz,
   but it would kill `fb_arr`, the arrival bypass that serves data combinationally the cycle
   it lands.
5. **333.33 MHz is the next rung** (`probe_clk = ui_clk/(DIV8/24)` — nothing between). That
   is 3.000 ns against today's 6.2–6.5 ns worst paths: a re-pipelining, not a tuning
   exercise.

## Loose ends

* `docs/rtl-rules.md` **I1** attributes the 465 ps to "area → congestion → slack". The
  measurement is real; the mechanism is not established, and the placement data suggests it
  was as much about the cache having no floorplan. Read I1 with that caveat.
* `e9ce9dfb`'s commit message asserts a fix that is not one; `f6501c60` corrects the record
  but the original text stands.
* `perf_event_paranoid=-1` did not survive a reboot despite the sysctl drop-in being in the
  NFS root. Reapply with `~/perf-smol-setup.sh` after each boot until someone finds why.
* The board takes a **new DHCP address every boot** (random MAC). Find it with
  `ss -tn state established '( sport = :2049 )'` on coffee.
