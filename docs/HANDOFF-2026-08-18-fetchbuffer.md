# Handoff — in-order core frontend + clocking, 2026-08-18 (overnight)

Written to be enough on its own. Everything below is measured unless it says otherwise,
and the places where I guessed wrong are recorded deliberately.

## The result

**The fetch buffer is worth 1.34x on the in-order core.** Matched 400M-cycle cosim runs,
tiny128 Linux boot, steady state:

| | retires @400M | x base | CPI | FE_BUB | .other |
|---|---|---|---|---|---|
| baseline | 74,805,581 | 1.00 | 5.348 | 2.2973 | 1.0019 |
| v2 @ HW=2 | 83,782,282 | 1.12 | 4.774 | 1.7301 | 0.9024 |
| **v2 @ HW=4** | **100,179,639** | **1.34** | **3.993** | **0.9782** | **0.3462** |
| v2 @ HW=4 + parked fixes | 97,744,362 | 1.31 | 4.092 | 0.9994 | 0.3742 |

Cosim vs simmerv: **100,000,000 retirements ok**, only the 200 pre-existing `MMIO-DIVERGE`
UART LSR lines (an unmodified baseline emits the identical 200 — controlled for).

## The bug

`inorder/ino_soc_top.v` tagged its single held fetch window with the **exact byte address**
it was fetched at:

```verilog
wire i_match = i_have & (i_pa == imem_addr);
```

So every PC change missed and re-requested bytes the adapter was already holding —
**1.004 I$ lookups per retired instruction**, measured. Not a miss rate: a fixed
per-instruction toll, which is why `FE_BUB` measured ~2.0 CPI on *every* hardware workload
(cold boot 2.237, tight hot loop 1.985, gzip 1.991) and decomposed as ~1.0 of lookup wait
plus ~1.0 of arrival bubble.

It was invisible because at HW=2 the window is exactly one instruction wide, so an
exact-match tag and a range check behave identically. The comparison only becomes *wrong*
once the window is wider than the instruction being fetched — which is why the bug and the
fix are the same change.

## What shipped

Branch **`inorder-fb`** on coffee, on top of `inorder-nospan`:

| commit | what |
|---|---|
| `8e71c72e` | probe_clk from an MMCM — continuous frequency instead of the 5-rung BUFGCE ladder |
| `00a00b7c` | move the probe_clk guard out of the XDC, where Vivado silently ignored it |
| `f38f909c` | constrain the `ddr_line_cdc` RETURN path — unconstrained in **every build ever made** |
| `38916986` | `INO_HW` build knob (refuses 8: RDW=128 trips the sdpram geometry guard) |
| `c2b01cbe` | fetch buffer v2 |
| `e51ccb01`, `fa235782` | your two parked timing commits, now cosim-validated |

`inorder-fbv2` is the buffer **without** the parked commits — **do not ship it**, see below.

## Do not ship inorder-fbv2

The parked commits are **required** at HW=4. Measured Fmax, all with the buffer:

| | Fmax |
|---|---|
| baseline HW=2, no parked commits | 111.9 MHz |
| buffer v2 HW=4, **no** parked commits | **86.6 MHz** |
| buffer v2 HW=4, **with** them | **108.0 MHz** |

They are worth ~21 MHz here, not the ~0.09 ns I first estimated (that comparison confounded
two builds). Mechanism: HW=4 widens the fetch window, lengthening the M-stage redirect ->
iMMU -> predictor cone that `b0f5f312` registers away. Buffer and redirect-registration are
complementary. Their 2.4% IPC cost is cheap.

They were parked in August because on the BUFGCE ladder a 0.1 ns gain rounded to zero. That
is the same reason the MMCM mattered: **it is what makes any of this bankable.**

## Timing is NOT closed — and why more builds won't fix it

The achieved path grows monotonically with the constraint. Same RTL, three builds:

| constraint | achieved path | misses by |
|---|---|---|
| 9.00 ns (111.11 MHz) | 9.262 ns | 0.26 |
| 9.50 ns (105.26 MHz) | 10.90 ns | 1.40 |
| 9.75 ns (102.56 MHz)* | 11.55 ns | 1.80 |

*also lacked the parked commits

Vivado optimises exactly to the target and stops, so **loosening the clock to make it fit is
self-defeating** — the design gets slower as fast as you relax it. The tightest constraint
gives the best silicon. Best available: the banked 111.11 MHz build missing by **0.26 ns
(2.9%)**. That may well run fine on the bench (sign-off is the slow corner), but that is a
call for you to make about your board, not for an unattended agent at 5am.

**The real limiter is the ycorr cone, not the buffer.** `u_bp/ycorr_qv_reg` is a sink
reachable from the whole back of the machine. Three different sources measured into it in one
night: `u_lsu/pa_q` (6 ns build), `u_csr/stimecmp_reg` (this build: 9.585 ns, 39 levels, 72%
routing), with the CSR mux and redirect already registered out by the parked commits. Each
feeder fix buys ~0.1 ns — the whack-a-mole `docs/pipelining-findings.md` warns about. The fix
is to make the predictor unreachable from the back of the pipe, i.e. the decoupled frontend.

## A THIRD silently-broken constraint — read the report by PATH GROUP, not the headline WNS

The design summary WNS is **not** the core. Broken down on the 105.26 MHz build:

| group | WNS | endpoints failing |
|---|---|---|
| design (headline) | -1.401 | 529 |
| `probe_clk_unbuf` intra-clock (the core) | **-0.780** | 343 |
| `probe_clk_unbuf` -> `mmcm_clkout0` (CDC) | **-1.401** | **10** |

The worst path in the whole design is a clock crossing, not logic:

```
Source:      mmio_clock_bridge/mmio_cmd_fifo/xpm_fifo_async_inst/.../count_value_i_reg[1]_replica/C
Destination: .../gen_pntr_pf_rc.wpr_rc_reg/reg_out_i_reg[1]/D
Data Path Delay: 0.248 ns   (1 logic level, one LUT2)
Requirement:     0.500 ns   (mmcm_clkout0 rise@48.000 - probe_clk_unbuf rise@47.500)
```

0.248 ns of delay failing a 0.5 ns requirement, inside an **XPM async FIFO's gray-code
pointer logic** — a structure that ships with its own CDC constraints.

Note the source name: `count_value_i_reg[1]_replica`. **`phys_opt_design` replicated a
register inside the XPM FIFO and the replica stopped matching XPM's constraint patterns**, so
the crossing reverted to being timed synchronously.

It only became visible because of the MMCM, and this is the genuine cost of a continuous
clock: under `BUFGCE_DIV`, `probe_clk` was an integer divide of `ui_clk` with aligned edges,
so the worst edge relationship was a comfortable 3 ns. At 105.26 MHz the ratio is
333.33/105.26 = 3.167 and edges land 0.5 ns apart. **Non-integer ratios expose every crossing
whose CDC constraints have been broken.** Expect more of these as the frequency moves.

Fixing it moves design WNS -1.401 -> -0.780 (91.7 -> 97.3 MHz on that build). Worth having,
but it is NOT what blocks the frequency — the core is.

**Do not reach for `set_clock_groups -asynchronous` without thought.** `cvfpu_timing.tcl`
records that it outranks `set_max_delay`, and a `set_max_delay -datapath_only` is what
protects the 512-bit `ddr_line_cdc` payload. Grouping probe_clk/ui_clk asynchronous would
leave that bus untimed — the same mistake that once produced stale FP data on core<->fpu.
A surgical waiver on the replicated XPM pointer cells, or preventing their replication, is
the safer shape.

This is the **third** silently-broken constraint found in one night, after the `if` inside
the XDC that Vivado ignored with only a CRITICAL WARNING, and the `ddr_line_cdc` return path
that had no `-from` in any build ever made. None of them failed loudly; a dropped constraint
does not error, it just stops applying. **Read `report_timing_summary` by path group.**

## What is now the biggest stall (measured, post-buffer, CPI 3.993)

```
ST_MEM  1.8596  (47%)    FE_BUB  0.9782      ST_FPU  0.0000
  .D$ write   0.7408       .FE_IC  0.5445
  .D$ read    0.5017       .other  0.3462
  .xlate/iss  0.3852
  .dTLB walk  0.2319
```

Burst histogram: loads-that-hit 18.99M x 3 cyc, stores-that-hit 17.18M x 4 cyc, misses only
**6.1%** of 38.6M memory ops. So ST_MEM is **~1.26 CPI of fixed hit occupancy vs ~0.63 CPI of
miss latency**.

**Next lever: a store buffer.** ~0.69 CPI (~1.23x). Stores that *hit* stall the pipe 4 cycles
because a blocking LSU has nowhere to put them. It is also the store queue
`Area-Efficient-Scalar-OoO.md` already specifies for the new core — so building it here
validates it against a booting Linux, with the cosim as oracle.

Then load pipelining (~0.57 CPI). Non-blocking/MLP is only ~0.63 CPI and is the *hardest* —
note that hit occupancy, which MLP does not touch at all, is twice as large.

## Things I got wrong (recorded so they are not re-derived)

1. **"Memory-level parallelism is the lever."** My opening recommendation. It is the
   *smallest* of the three LSU wins. The frontend was twice as large and free to fix.
2. **"+5.7% from the fetch buffer."** That was a 40M-cycle run — entirely cold boot, where
   I$ misses dominate. Steady state is 1.34x. Never compare across run lengths.
3. **"The workload goes idle at ~55M cycles."** It does not; that is a transient slow phase
   (CPI 9-14) around c=44-64M. Steady state resumes at c~84M at CPI 3.7-4.0.
4. **"The MMCM cost setup margin."** It cost *hold*, which routing fixed. Final: WNS +0.061
   vs the BUFGCE milestone's +0.019 — the MMCM is free.
5. **"v1's 64-bit subtractor cost 0.79 ns."** v2 removed the subtractor and WNS did not move.
6. **"The parked commits buy ~0.09 ns."** They buy ~21 MHz.

The through-line: every wrong call came from reasoning across non-matched measurements. The
ones that held up came from matched controls and from reading report *files* rather than log
lines — Vivado replays prior runs' messages from `.pb`, which showed a killed build's numbers
as current twice tonight.

## Operational notes

- Board: `ssh` to it from coffee; **its DHCP address changes every boot** — find it with
  `ss -tn state established '( sport = :2049 )'` on coffee (it always NFS-mounts root there).
- `hpmstat_dyn` and `/var/tmp/blob.bin` live on the NFS root and survive reboots.
  Hardware CPI baselines already taken: md5sum 3.796 CPI / FE_BUB 1.985, gzip 5.261 / 1.991.
- Link board binaries **dynamically**; coffee's static glibc executes vector instructions and
  SIGILLs (`cause: 2`, opcode 0x7057). `readelf -A` does NOT predict this.
- **After killing a Vivado build, `reset_run synth_1`/`impl_1`** or the next build dies ~30
  min in with "needs to be reset before launching". Cost an hour tonight, twice.
- Serial getty was dead because a previous session masked `systemd-udev-trigger.service`;
  `serial-getty@ttyS0` `BindsTo=dev-ttyS0.device` which then never appears. Fixed.

## Suggested next steps

1. Decide whether to run the banked 111.11 MHz bitstream (`/var/tmp/fbv2_fmax_111.bit`,
   misses by 0.26 ns) to validate 1.34x on silicon. `hpmstat` is staged and the baselines
   are taken, so it is a ten-minute job once you say yes.
2. Store buffer — the largest measured stall, and it is task 4's store queue.
3. Decoupled frontend — kills the ycorr cone and is the precondition for the OoO core.
