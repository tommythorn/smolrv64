# OOO2 dynamic issue — the switchover

Everything below is the remaining work to make issue dynamic. The preconditions are
committed and green; this file exists so the switchover is execution rather than
rediscovery. Written against the RTL as of `6642270f`.

## What is already in place

| piece | commit | state |
|---|---|---|
| `ooo2_iq` — oldest-ready scheduler | `300a69d9` | 16/16 unit TB |
| ROB entry 28→16 bits | `c4155099` | live |
| `ooo2_pending` — per-physreg readiness | `0ba595d0` | live as a **shadow**, soaked 300M cycles |
| PRF: one write address per shard | `0ba595d0` | live, behaviour-neutral |
| ROB: NW completion ports | `6642270f` | live, pinned `NW=1` |
| scheduler + payload wired, drained at issue | `b78efa7a` | live, **not steering**; payload checked against `m_*` every cycle |
| `in_order` gate, `iss_ps1/2/3` | `526ef02d` | live |

All of it gated: lint, 240/240, and a 300M-cycle cosim at `retires=67160189`.

## The one fact that shapes everything

`ooo2_exec` is already a pure combinational block — decoded control plus three operand
values in, `{result, addr, redirect, target, taken, taken_tgt}` out. It does not need to
change. What changes is *where its operands come from* and *when it runs*.

Today: operands are read in **X** (`prf_rs1/2/3` plus the M→X bypass), and everything —
ALU, LSU, mul, div, FPU, CSR — funnels through the single **M** slot.

Target: operands are read at **issue**, from the PRF, addressed by the physical register
numbers the scheduler holds. `ooo2_exec` runs there. Long-latency units are entered from
there and hold their own op.

## Step 1 — payload storage

The scheduler holds scheduling state only. The execute bundle goes in arrays indexed by
**scheduler entry number** (`fsel` to write at dispatch, `sel` to read at select) — not by
`rob_idx`, which would be `ROB_SIZE` deep and would put a read port at issue on a ROB-sized
array, the thing the ROB/scheduler split exists to avoid.

Per-field arrays (`p_pc[8]`, `p_insn[8]`, …), not one packed vector: ~35 fields, and one
obviously-correct line each beats a concatenation nobody can check. Fields are exactly the
X→M register list in `ooo2_core.v`, **minus**:

- `m_rs1_val`, `m_st_data`, `m_rs3_val` — operand VALUES. Read from the PRF at issue.
- `m_result`, `m_addr`, `m_target`, `m_taken`, `m_taken_tgt`, `m_redirect` — computed by
  `ooo2_exec` at issue from those operands.
- `m_pold` — already deleted (`c4155099`).

## Step 2 (Step I) — dispatch stops waiting

`d_hold` loses `src_pend` entirely. Dispatch is blocked only by: ROB full, **scheduler
full**, `rn_stall` (a rename shard low), and `ser_block`. This is the change that makes the
window fill; everything else is machinery to survive it.

`ooo2_pending`'s outputs stop being a shadow and become `d_r1/2/3` into the scheduler.
Delete the shadow assertion at `ooo2_core.v:411` — it asserts in-order consumption.

## Step 3 (Step I) — issue

```
sel        -> payload read (async LUTRAM)
e_ps1/2/3  -> PRF read (3 ports, already exist)
           -> ooo2_exec  -> result / addr / branch resolution
```

Unit routing, one-hot, matching `ooo2_iq`'s `unit_busy`:

| unit | ops | busy while |
|---|---|---|
| ALU | everything single-cycle, incl. branches and jumps | never (1 cycle) |
| MEM | loads, stores, AMO | LSU not `S_IDLE` |
| MD | mul, div | `mul_busy \| div_busy` |
| FP | `fp_arith` | `fb_busy` |
| SYS | CSR, `fence.i`, SYSTEM | until it commits (serializing) |

Per doc 7, a 1-cycle unit never justifies its own structural hazard — so ALU is "always
free" and branches resolve on it.

## Step 4 — the hard parts (Step II only)

**4a. Redirects and traps become deferred.** A branch resolves at issue, out of order, and
must not squash anything. Replace `head_block` with the doc's §12 **redirect register**: a
single register holding `{valid, rob_idx, kind, payload}`, written oldest-wins at execute.
Commit tests `redirect_valid && redirect_rob_idx == head` and flushes then. This is
strictly simpler than what is there now and removes `m_needs_head`.

**4b. In-flight ops at flush.** Today nothing younger than the redirecting op can be in
flight, because `head_block` guarantees it. That guarantee is gone: a load may be in the D$
and an FP op in the CVFPU when the flush lands. Their destinations are dead and may already
be **reallocated**, so their writeback must be suppressed, not just ignored — a per-unit
`zombie` bit set on flush and cleared when the unit's result is discarded. `src/exec_shard.v`
already does exactly this (`fp_zomb`, "drain, don't flush"); copy the shape.

**4c. Load/store ordering.** No disambiguation exists, so a load must not issue past an
older store. Until there is a store queue, the cheap correct rule is: a memory op is
`ready` only when no older memory op is live. That serialises memory but nothing else, and
it is what `load_may_issue` degenerates to.

**4d. Writeback arbitration.** IE takes the ALU. FE takes the FPU. LD takes **both** the
LSU and mul/div — so LD needs a two-way arbiter with the loser holding its result, or
mul/div moves to its own shard. The `$fatal` added in `0ba595d0` fires the moment this is
wrong, which is the point.

**4e. The cosim retire stream.** `cs_val` is captured "at the writeback event"; with several
writeback events per cycle it must be captured per ROB slot from each port.

## Order of work — TWO steps, not one

An earlier version of this note said steps 1–3 had to land together with no gateable
intermediate. That is true only because issue REORDERS — and reordering is exactly what
makes 4a, 4b and 4c hard. `ooo2_iq`'s `in_order` gate (`526ef02d`) splits it.

**Step I — `in_order = 1`.** Dispatch stops waiting on operands; the scheduler fills; issue
takes the oldest LIVE entry; operands are read from the PRF at issue; M is fed from issue
rather than from X. Execution ORDER is unchanged, so:

- nothing younger than a redirecting instruction is ever in flight → **4b does not arise**
- memory ops keep program order → **4c does not arise**
- traps still fire in order → **4a can stay as `head_block`**

That is the mechanical 80%, and it gates against the existing 240 tests and the cosim.
Retires WILL move: dispatch→issue adds a stage, so it needs a writeback→issue operand
forward — the datapath twin of the scheduler's `wb_hit` wakeup — or every dependent pair
pays a cycle. The M→X bypass disappears with it, and `ooo2_pending`'s shadow assertion
(`ooo2_core.v`, keyed on `rn_valid`) goes with the bypass since it asserts in-order
consumption.

**Step II — `in_order = 0`.** One flag, then 4a, 4b, 4c, 4d, 4e, on structures already
known good. This is where the performance is and where the time will go.

Gate every step with `run-ooo2-vl.sh` (fast) and only then the 300M cosim.

## What to measure when it runs

`ST_MEM` and `ST_FPU` already carry the dependent wait (`dep_ld`/`dep_fp`) that dynamic
issue is meant to remove — on GB5 those were 0.940 and 2.567 CPI. Those two buckets
shrinking is the whole point, and if they do not, the scheduler is not doing anything.
