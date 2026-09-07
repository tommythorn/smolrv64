# Handoff — 2026-09-03: multiple outstanding loads

The largest measured IPC opportunity in the design, now quantified rather than argued.
`main` is at **`bcff35e7`**. Nothing of this work has started; two branches are parked.

> **BEFORE ANYTHING ELSE:** `tb_ooo2_lq` and `tb_ooo2_sq` **do not compile**. This work edits
> exactly the code they cover. Fix them first — §6.

---

## 1. The measurement that justifies the work

Ubuntu NFS boot, `OOO2_HW=4`, 300 M cycles per point, lockstep clean at every point:

| `DDR_LAT` | retires | retires/cycle | vs. a 4-cycle memory |
|---|---|---|---|
| 4 | 80,008,912 | 0.2667 | — |
| 20 | 68,713,515 | 0.2290 | −14.1% |
| 80 | 42,307,472 | 0.1410 | **−47.1%** |

**Nearly half the throughput is spent waiting for memory**, and the board sits at the slow end
of this curve — not the 4-cycle end that every cosim before 2026-09-02 ran at.

## 2. Why the cache is already ready, and is not the constraint

The D$ split (`rv_cache` as a LOOKUP pipeline plus a FILL machine with one MSHR) lets a hit
proceed under a miss. It recovers **+0.33%** — and that is the *same* +0.32% it scores at
`DDR_LAT=4`:

    pre-split  (c4134edd)   42,169,204 retires  @ DDR_LAT=80
    post-split (main)       42,307,472 retires  @ DDR_LAT=80    +0.328%

Making memory 20× slower **did not widen its lead**. That is the measurement that settles it:
a win from hiding miss latency would have grown, and it did not. The cache can overlap; the
design produces no second request to overlap with, because the LSU, the I$ fetch buffer and
each PTW walk are all single-outstanding, so `S_CHECK` usually has nothing to run under the
miss.

**Corollary:** further latency-hiding work inside `rv_cache` optimises a resource that is not
the constraint. (`rv_cache` *is* the timing constraint — a separate matter, see the timing
handoff.)

---

## 3. Where to start

Two branches, kept deliberately for this:

    wip/pipelined-loads   0681b571   "steps 2+3 of pipelined loads -- NOT WORKING, parked"
    wip/cache-selfloop    baf96dff   "rv_cache S_CHECK self-loop -- correct shape, WRONG
                                      without a request ack"

They pair, and `docs/OOO2-Spec.md` (~line 1440) already states the conclusion:

> The implicit contract "hold the request until the response matches" only worked because the
> cache stayed busy for exactly the round trip. A pipelined port needs the explicit one: an
> `rd_ack` combinational from the accept, with every requester splitting its single pending
> bit into REQUESTING (cleared on ack) and OUTSTANDING (cleared on the response) — rule D5,
> and the same defect `wip/pipelined-loads` hit as `pt_ack`.
>
> **And even with the ack it buys nothing alone**: the LSU, the I$ fetch buffer and each PTW
> walk are all single-outstanding... Design the ack together with the multi-outstanding LSU
> that consumes it, not ahead of it.

So the self-loop alone is a defect; the ack alone buys nothing. They land together.

---

## 4. The constraint that will bite

**The L2 ack carries no tag.** Today the only thing binding a response to its requester is
that exactly one is in flight, enforced by the issue gate `fill_l2_busy` (spec §8). This is
not a detail — it is the defect that cost 2026-09-01 through 09-02:

> `rv_cache` had two L2 requests outstanding because the prefetch guard tested `!l2_req`, a
> one-cycle **pulse**, so the port read as free for the whole round trip. Both consumers
> latched the one ack; the stream buffer filed the demand line's bytes under an address it
> never fetched. Right tag, wrong data, in the I$ — invisible to parity, to the provenance
> check, and to the existing two-transaction tripwire. Proven on hardware by a negative
> control: one token removed → illegal-instruction panic at 1.22 s (`b85e6e19`, rules A6/B6/D9).

Going multi-outstanding means **tagging the transaction**. Do not extend the issue gate;
rule B6 is explicit that a gate is not a substitute for a tag, and B1 requires a response to
be matched by a tag the requester allocated. `docs/OOO2-Spec.md` P0 notes the load queue *is*
the tag space the D$ already reserves.

Other constraints:

- **Rule I7** — keep payload in RAM, not flops, and do **not** buy timing by registering a RAM
  output. That adds a pipeline stage and costs the IPC this work exists to win. A
  behaviour-neutral change must be **bit-identical** in cosim retires.
- **Rule I6** — a flop, not a RAM output, drives the next RAM's address pin.
- LQ and SQ are **`NENT=4`**. This work will likely deepen them; the OOC numbers
  (`ooo2_lq` 908 MHz, `ooo2_sq` 330 MHz) give headroom to spend, but re-measure after.

---

## 5. Interaction with the timing work

`rv_cache` is the timing bottleneck — **178 MHz alone on an empty die**, against 330–1144 MHz
for every core module (see the timing handoff §4). This work adds logic to exactly that
region, and main currently closes about half the time on identical RTL.

Going in with zero margin means every step is decided by placement luck rather than merit.
Either sequence the cache timing work first, or accept that intermediate commits will not
produce bitstreams and gate on OOC + simulation until a batch is ready.

---

## 6. BLOCKER: the unit tests for this exact code do not compile

    tb_ooo2_lq.v:33   Pin not found: q_pa, q_size, q_block
    tb_ooo2_sq.v:34   Pin not found: ld_addr, ld_size, ld_block

Cause: the alias test became a conflict matrix rather than a compare at issue (`69f1ac6a`),
deleting the per-candidate address ports. `ooo2_lq` now exposes all entries and consumes a
per-entry block vector:

    output [NENT*PAW-1:0] e_pa       all entries' addresses
    output [NENT*2-1:0]   e_size
    input  [NENT-1:0]     e_block    per entry: an older store aliases it
    output                x_block    ...and the candidate is one

So the testbench must emulate the store queue's side of the matrix. That is a rewrite, not a
port rename. Nothing noticed the rot because no gate ran these and the runners sent build
errors to `/dev/null`; both are fixed, and `gate.sh` now globs `run-ooo2-*-tb.sh` and treats a
build failure as a gate failure. **It is red until these are rewritten.**

These are the only unit-level check on the load/store ordering path. Coverage wanted:

- a load held because an older store aliases it, released when that store commits
- the no-alias case issuing immediately (`ld_older` false → `req_early` collapses fill+access)
- partial overlap and size mismatch, both directions
- commit and alias landing in the same cycle
- flush while entries are blocked

State the configuration in the output. A pass at `NENT=4` says nothing about a deeper queue,
and `run-ooo2-cache-tb.sh` sweeps `LAT=4/20/100/200` precisely because a single point misled
once.

---

## 7. Gates

    src/lint.sh                     lint: clean
    ooo2/run-ooo2-vl.sh             pass=240 fail=0
    ooo2/run-ooo2-cache-tb.sh       both shapes, LAT=4/20/100/200
    ooo2/run-ooo2-lq-tb.sh          must exist and pass — §6
    ooo2/run-ooo2-sq-tb.sh          must exist and pass — §6
    ooo2/run-ooo2-cosim-linux.sh    AND a DDR_LAT sweep

**The latency sweep is mandatory for this work.** `DDR_LAT=4` cannot show a latency-hiding
win and hid a real defect for months — the prefetch/L2 bug fires at `DDR_LAT=40 DDR_JIT=63`
and *never* at 4. Measure at 40–80. `BUILD=1` is required whenever `VDEFS` changes, or a
stale binary is silently reused.

Then the board: closes at 166.67 MHz and boots to `login:` with **zero** segfaults. Reaching
`login:` is not on its own a pass; grep for `unhandled signal`, `segfault`, `SIGSEGV`,
`status=11/SEGV`, `core dumped`, `Unable to handle kernel paging`, `Oops [#`, `Kernel panic`.
`screenlog.0` is cumulative — record `wc -c` before booting and search only past that offset,
or a previous boot's prompt reads as a pass.
