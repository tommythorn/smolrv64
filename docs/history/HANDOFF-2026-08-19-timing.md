# Handoff — in-order core timing, 2026-08-19

Written to be enough on its own. Everything is measured unless it says otherwise, and the
places I was wrong are recorded deliberately.

## The result

**+4.95% IPC, banked. Frequency gain real but not yet bankable.**

Shippable bitstream: `/var/tmp/inorder_111MHz_1ba5da1b.bit` — `PROBE_CLK_DIV8=72`
(111.11 MHz), `OOO2_HW=4`, closed at **WNS +0.008** via `make physopt`. Program any banked
bit with:

    make program BIT=/var/tmp/inorder_111MHz_1ba5da1b.bit

IPC, cosim at a fixed 60M cycles (tiny128 Linux boot):

| RTL | retires | vs start |
|---|---|---|
| session start | 11,082,760 | — |
| forked predictor, details combinational | 11,443,315 | +3.25% |
| + F/X queue | 11,623,201 | +4.88% |
| + tag-matched D$ responses | **11,631,167** | **+4.95%** |

Timing, matched 6 ns builds (`PROBE_CLK_DIV8=48`), "Fmax as built" = 6.000 - WNS:

| RTL | WNS | Fmax as built |
|---|---|---|
| session start | -3.405 | 106.33 MHz |
| three device-path fixes | -2.791 | 113.75 |
| RAS snapshot dropped | -2.762 | 114.13 |
| F/X queue | -3.273 | 107.84  **(regression)** |
| tag-matched responses | -2.660 | 115.47 |
| `dmem_wready` -> OR | -2.568 | 116.71 |
| `raw_rvalid` -> OR | **-2.436** | **118.54** |

## THE THING TO UNDERSTAND BEFORE DOING ANYTHING ELSE

**The design is now faster when pushed hard and slower when not.** Same RTL, two asks:

| ask | session-start RTL | end-of-session RTL |
|---|---|---|
| 6.00 ns | 9.405 achieved | **8.436** (0.97 ns better) |
| 9.00 ns | 8.996 (WNS +0.004, closed) | 9.113 (WNS -0.113, **did not close**) |

111.11 MHz closed on its own this morning and now needs a phys-opt rescue. The added area
(the F/X queue's ~560 flops, the tag logic) costs more at a relaxed target than the
shortened logic saves, because Vivado stops the moment it meets the ask.

Consequences, and I got the second one wrong today:

1. **The 6 ns build is the right instrument for deciding WHAT TO FIX.** It ranks structural
   problems honestly because everything is being pushed.
2. **"Fmax as built" is NOT a shippable frequency.** I read 118.54 MHz and built at 117.65
   (`DIV8=68`) expecting it to close. It missed by 2.074 ns, because relaxing the ask from
   6 ns to 8.5 ns made the achieved path *2.1 ns worse*. This is the monotonicity
   `HANDOFF-2026-08-18-fetchbuffer.md` already documented and I mis-applied it anyway.

**The measurement nobody has made:** a build asked at ~8.7-8.8 ns — tight enough that Vivado
works, loose enough to close. That is the only way to find out whether +11.5% is bankable.
Do this before any more RTL.

## What was fixed, and the one class behind most of it

Five commits were **one defect**: the device-vs-cache decision re-derived from a 64-bit
address at every point of use, inside the cycle where it was needed.

| commit | recomputed on the critical path |
|---|---|
| `01e1f265` | device read data muxed into the load return |
| `df96e431` | three write-qualified device address muxes |
| `2a9f4c1`  | response matched by address, not an allocated tag |
| `4747f19d` | write-ready SELECTED by a device compare |
| `f0a1c2d`  | read-valid SELECTED by a device compare |

`docs/rtl-rules.md` already names the rule ("a precondition that applies to N units is
computed once and applied at one site"). **Diagnostic detail: the last three were
cycle-neutral, two BIT-IDENTICAL in cosim.** These were not trade-offs, they were redundant
logic — the select never selected anything. If deleting logic changes nothing at all in
simulation, it wasn't doing anything.

**Generator-level fix if it recurs:** `rv_soc_top` has no single place where "is this access
a device?" is decided and named. Give it one and the class is closed.

Also landed: the predictor forked to `ooo2/ooo2_predictor.v` with checkpoints deleted
entirely (nothing used them; predict details now ride the pipeline as `pd_fetch -> d_pdet ->
m_pdet -> res_pdet`), and the F/X queue that decouples `fetch.ready` from `lsu_done`.

## Where the limiter is now

`m_rs1_val_reg[3] -> fe/u_bp/btb_q` (226 paths, 36 levels):

```
m_rs1_val (CSR write source) -> u_csr mstatus/mtvec next-state (CARRY8 x3)
                             -> csr_writes1 -> ... -> fe/u_bp/btb_q
```

A **CSR write in M reaching the fetch-side BTB read in the same cycle**. The startpoint has
left the LSU for the first time, so the device-decode playbook does not apply directly.

Second family, and a different kind of problem: `u_icache/cur_line -> u_icache/linebuf[*]/CE`
— only **11 logic levels but 7.091 ns of routing (91%)**. That is congestion. No logic
restructuring will touch it; it wants placement or reduced fanout.

The tail everything still shares: `lsu_done -> m_done -> redirect -> u_fetch/va_q -> u_bp`.
Every fix so far shortened the FRONT. The tail is a memory completion and a frontend
redirect sharing a cycle, and it is what keeps promoting whichever backend signal is longest.

## Gates added today (three, all for the same failure shape)

A documented invariant with nothing behind it:

- **`ooo2/cosim-expected.txt` + a check in `run-ooo2-cosim-linux.sh`** — fails when the
  retire count drops >0.5% below a recorded number. Three regressions today were
  architecturally invisible (cosim clean, tests green, Linux booting) and visible ONLY here;
  two cost ~4% each. **Raise the number in the same commit that earns it.**
- **`tools/gen-perf-events.py --check`** in `src/lint.sh` — `docs/smolrv64-perf-events.json`
  is generated from `csr_file.v`. It had drifted to describe a core that no longer exists.
- **`tools/check-dts-timebase.py`** in `src/lint.sh` — the DTB timebase is a pure function of
  `PROBE_CLK_DIV8`, and `SCALE_DIV` is an INTEGER divide so it is 501253 only at 66.67 MHz.
  Both FPGA DTS files declared 501253, so every board run since the 111 MHz milestone (the
  GB5 run included) had every kernel deadline **0.302% fast**. Now 502765.

`src/lint.sh` also now covers the in-order core (it previously linted only `soc_top`), which
found eight real defects on its first run — including `perf_access`/`perf_miss` sitting
inside `` `ifdef PERF_TRACE ``, so the cache HPM counters read **zero in every bitstream ever
built**.

## Open, in the order I would do them

1. **Find the shippable frequency.** Build at ~8.7 ns. Nothing else is worth doing until we
   know whether the structural work converts.
2. **Re-run `hpmstat` on the board.** The cache HPM counters work now for the first time, so
   this gives real D$/I$ miss rates on silicon. Ten-minute job.
3. **The CSR -> BTB coupling** (the current limiter).
4. **The I$ congestion family** — placement/fanout, not logic.
5. **The `ST_MEM` sub-breakdown is still unreproducible.** `.D$ write / .D$ read / .xlate/iss
   / .dTLB walk` in the 2026-08-18 handoff came from uncommitted instrumentation. The "next
   lever: store buffer" recommendation rests entirely on it. Rebuild it as committed,
   default-off `hpm_ev` taps.
6. **~20 DTS files still carry the stale 501253.** I fixed only the two whose FPGA use a boot
   script confirms; the `-simmerv`/`-cosim` ones model a different clock and must keep
   theirs. Wants an audit, not a sed.
7. **`-Wno-WIDTHEXPAND` hides half of every truncate-then-expand round trip** (87 existing
   hits). That is what let the `hpm_ev` bug recur after the identical `retire_cnt` bug was
   fixed once.
8. **The five `run-ino-*.sh` runners each carry their own source list** — adding one module
   meant patching all five. `src/rtl-sources.sh` has `ooo2_sources()` already; they should
   use it.

## Verification recipe

    src/lint.sh                                    # both tops + perf-events + dts timebase
    ooo2/run-ooo2-vl.sh rv64ui-p rv64um-p rv64ua-p rv64uc-p     # pass=85 fail=0
    VDEFS=-DINO_HW=4 BUILD=1 CYC=60000000 ooo2/run-ooo2-cosim-linux.sh
    src/run-vl-tests.sh                            # only if src/ changed; failures: 0

`BUILD=1` is MANDATORY when `VDEFS` changes — the cosim runner reuses a stale binary
otherwise, which is exactly the "compared across non-matched builds" trap.

`run-ooo2-vl.sh` does NOT compile `rv_soc_top.v`. A change there can pass every riscv-test
and be broken; the Linux cosim is the only gate that sees it.
