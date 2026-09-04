# Handoff — 2026-09-03: timing at 166.67 MHz

> **§4 IS REFUTED -- read `HANDOFF-2026-09-04-timing.md`.** The 178 MHz OOC figure was
> measured on `rv_cache`'s parameter defaults, not the shipped shape, and the failing core
> paths were one five-module chain through the LSU's port arbitration, not slack the cache
> had spent. The rest of this file (the WNS record, the refuted floorplan, the instruments)
> still stands.

`main` is at **`bcff35e7`**. It is correct and boots, but **it does not reliably close
timing**, and the last commit that produced a bitstream is `f9540bf1`.

> **READ THIS FIRST.** Three structural changes were made this session, each of which did
> exactly what it was designed to do, and **none of them moved WNS**. One OOC run then
> explained why: every core module has 2× headroom and **`rv_cache` does not**. Do not start
> by redesigning the scheduler. §4 is the whole point of this document.

---

## 1. Where things stand

| | |
|---|---|
| `main` | `bcff35e7` — lint clean, 240/240, cosim bit-identical at 14,657,366 |
| Last bitstream | `f9540bf1`, **WNS 0.000000**, booted to `login:` with 0 faults |
| `github/main` | `c5c8df52` — three commits behind local |
| Utilisation | 66,754 LUTs (**30.8%**), 46,223 regs (10.7%), 12,655 CLB sites (**46.7%**), BRAM 118/480 |
| Board | holds the `f9540bf1` bitstream, working |
| `tools/gate.sh` | **RED** by design — `tb_ooo2_lq`/`tb_ooo2_sq` do not compile, see the multiple-loads handoff |

`fb82d188` (readiness restructure) is committed and **fails timing at −0.0346**. That is
inside the placement spread, so it is neutral rather than a regression, but the tip currently
does not build a bitstream. Reverting it is defensible; it is structurally better and costs
nothing measurable.

---

## 2. The WNS record, on RTL that is functionally identical

    +0.019   pre-session main (c4134edd + gate repairs)
    -0.041   same RTL, clean rebuild
    -0.099   same RTL, PLACE_DIRECTIVE=ExtraTimingOpt
    +0.000902  3b518832  issue-queue tags to LUTRAM
     0.000000  f9540bf1  rename arrays to bitstream init      <-- last bitstream
    -0.206305  pblock confining the core to 6 clock regions   <-- refuted, §5
    -0.034550  fb82d188  readiness queries both maps

**The design closes about half the time on identical source.** Rule I2 exists because of
exactly this: one build's WNS cannot judge a change smaller than the 81–400 ps placement
spread. Judge nothing on a single build.

`AltSpreadLogic_medium` is the placer directive and is part of the shipping configuration,
not a knob — see the reasoning in `build.tcl` around `set place_directive`.

---

## 3. What landed, and what it actually bought

| commit | change | effect |
|---|---|---|
| `3b518832` | issue queue emits an index; source tags from a `psmem` LUTRAM | target path **gone** (`i_ps1_reg` absent from the report); WNS unmoved |
| `f9540bf1` | rename free lists + maps initialised by the bitstream, reset is a flush | −5,620 LUTs, −3,533 regs, −1,169 CLB sites; WNS unmoved |
| `fb82d188` | readiness queries both maps, `lv` selects the 1-bit result | WNS −0.0346 (neutral) |

All three are correct and worth keeping. `f9540bf1` in particular removed 8.5% of CLB sites.
None of them bought slack, because none of them was the constraint.

---

## 4. THE FINDING: it is `rv_cache`, not the scheduler

`make ooc MODULE=<name>` synthesises one module alone on the real part and reports its
intrinsic Fmax. In the flat design every path is 65–83% route and placement swamps anything
under ~200 ps, so the integrated build can only tell you whether today's placement was lucky.
Measured at a 6.000 ns period:

    ooo2_pending  1144 MHz        ooo2_rename   403 MHz
    ooo2_lq        908 MHz        ooo2_iq       373 MHz
                                  ooo2_sq       330 MHz
    rv_cache       178 MHz    <-- +0.385 ns ALONE ON AN EMPTY DIE

Its own worst path, with nothing else on the chip:

    cur_line_reg[18]/C -> valm_reg_r4_0_255_0_0/DP.A/WE
    5.415 ns, 18 logic levels (MUXF7 x3, MUXF8 x2, RAMD64E x2, CARRY8 x1), 70.9% ROUTE

Eighteen levels from a register to the valid array's **write enable**. Rule I6 permits a late
signal on a RAM's enable — this is what spending that permission costs.

**So the failing paths in the integrated build (`ps_out -> u_sq/data`, `u_sq/v -> u_iq/e_r`,
`u_sq/v -> fe/q_dat`) are where the slack RUNS OUT, not where it is SPENT.** They end in core
logic that has no budget left after the cache has consumed it. That is why three correct
changes to core structures moved nothing.

### What to do

1. **Attack `cur_line -> valm/WE` in `rv_cache`.** 18 levels to a write enable is the
   deliverable. `make ooc MODULE=rv_cache` is a ~4 minute loop; the full build is ~45 minutes.
   Iterate OOC, then confirm in the full build — never the other way round.
2. `rv_cache` is ONE module in TWO roles (I$ `WRITABLE=0, PREFETCH=1`; D$ `WRITABLE=1,
   WRTHRU=0`). A pass in one role is not a pass in the other — that mistake cost a full day
   on 2026-09-01. `ooo2/run-ooo2-cache-tb.sh` drives both shapes at `LAT=4/20/100/200`.
3. A replacement at the same interface can be compared directly with `make ooc`, then gated
   with the directed tb, then the cosim, then the board.

---

## 5. Refuted — do not repeat

**Floorplanning.** `make place-report` showed `probe_core/core`'s 53,634 cells over NINE
clock regions, `u_prf` mostly X2Y3 and `u_sq` mostly X3Y2, diagonally apart. A pblock
confining the core to six contiguous regions (X1Y2:X3Y3, ~8,900 cells each, no exclusion so
the caches could still share X1Y2) gave **WNS −0.206 / TNS −117** against 0.000
unconstrained, and the failing paths never left the core:

    u_sq/v_reg[2]        -> u_iq_i/e_r_reg[8][1]     70% route
    u_rename/lv_reg[15]  -> u_iq_l/e_r_reg[9][2]     82% route

Those are single control registers fanning out to every entry's ready bits. `e_r` is
per-entry wakeup state, so its destinations are spread *by construction* — the route being
timed is the fanout tree, not a journey between blocks. Confining the core removed the room
the placer needs to spread replicas, which is what `AltSpreadLogic_medium` exists to provide.
**"65–83% route on a third-full die" does not imply a spread problem.**

**`ExtraTimingOpt`.** −0.099 against the default's +0.019/−0.041. The directive is not the
lever.

---

## 6. Instruments added this session

| command | what it answers |
|---|---|
| `make util` | is the device full? (no: 30.8% LUTs) |
| `make place-report` | where does each block physically sit, by clock region |
| `make ooc MODULE=<m> [PERIOD=n]` | what is this module's intrinsic Fmax, alone |
| `tools/check-ram-inference.py` | did an array silently become flops? (fails the build, names it) |
| `tools/gate.sh` | full gate; archives evidence to `gate-results/<sha>/` |

`make ooc` cannot yet synthesise `ooo2_core` — the CVFPU wrapper (`fp_unit`) is outside the
`ooo2/` + `src/` glob. Worth fixing if core-level OOC is wanted.

---

## 7. Rules written from this work

`docs/rtl-rules.md`, all with the measurements attached:

- **I7** — prefer an indexed RAM read over a mux across N scattered flops. Three reasons:
  route dominates here (65–83%), it **scales** (flops grow the gather with N; a RAM grows by
  adding depth to something already local), and synthesis tells you which you have for free.
  Two constraints: never buy timing by registering a RAM output (that is a pipeline stage and
  an IPC loss), and I6 still holds — a flop, not a RAM output, drives the next RAM's address.
  Also records that a **parallel reset loop** demotes an array just as a broadcast read does.
- **I2** — extended with: measure a structure OOC before redesigning it; the placement
  ranking does not carry forward; the design has no margin anywhere (seven distinct worst
  paths); floorplanning refuted.
- **A6 / B6 / D9** — from the prefetch defect, see the git log for `b85e6e19`.

---

## 8. Gates

    src/lint.sh                     lint: clean
    ooo2/run-ooo2-vl.sh             pass=240 fail=0
    ooo2/run-ooo2-cache-tb.sh       both shapes, LAT=4/20/100/200
    ooo2/run-ooo2-iq-tb.sh          IQ-TB PASS
    ooo2/run-ooo2-cosim-linux.sh    14,657,366 retires at CYC=60000000 — a structural change
                                    that is behaviour-neutral must be BIT-IDENTICAL
    make                            WNS >= 0 at OOO2_CORE=1 OOO2_HW=4 PROBE_CLK_DIV8=48
    make program + ubuntu-boot.sh   login: with ZERO faults

`gate.sh` runs every `run-ooo2-*-tb.sh` by glob and treats a build failure as a gate failure.
It is red until the lq/sq testbenches are rewritten.
