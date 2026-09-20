# IW=3 timing: the −0.528 plateau was late control roots, not PRF ports

Written 2026-09-17 (session `012LAhVdXKrjj2AKMZXy7CBx`), on `wip/iw3-timing` in
`/home/tommy/smolrv64-wt-stage1`, base `dd371e6d` (dispatch swizzle + dispatch stage,
WNS −0.528 ns at IW=3/HW=8). Supersedes `HANDOFF-iw3-prf.md`'s conclusion that only a PRF
read-port reduction could move the design: no PRF port was removed for the first result.

## Result so far

| build | RTL | WNS | TNS | failing endpoints |
|---|---|---|---|---|
| dd371e6d (handoff) | dispatch stage | −0.528 | −1275 | 3940 |
| hm1 | round 1 (five cuts below) | **−0.117** | **−5.5** | **90** |
| hm2 | rounds 1+2+3 | −0.130 | −6.7 | 143 |
| hm3 | rounds 1+2+3+4 (+ the matrix column clear after the row write) | **+0.007** | **0** | **0** |

Every round is verified the same way: `src/lint.sh` clean (it was red on the base for the
very loop cut in round 1), riscv-tests 240/0, the IW=3/HW=8 60M-cycle Linux lockstep cosim
clean (retires 11,682,439 vs 11,684,956 on the base: −0.02%, the one-cycle head hold), and
the IW=2 60M cosim clean (13,491,763 vs the expected 13,424,357: +0.5%).

## What the plateau actually was

Reading the −0.528 census with the per-hop delays: every family was 20–28 logic levels at
0.15–0.25 ns of route per hop. That is not congestion (congested routes show as a few
long nets, not a uniform per-hop cost); it is depth, and the depth came from LATE ROOTS in
front of otherwise reasonable logic:

- **fetch loop** (pc_q → BTB write address, pc_q → pc_q, ~1000 paths, 24 levels): a 64-bit
  adder (`pc_ca + CHB`) and a 64-bit compare sat at the HEAD of the loop, deciding whether
  the next chunk is resident before the served window could be selected. ~1.9 ns from
  pc_q to the compare result.
- **t_ld fan-out** (rename tail → SQ, dispatch stage, pending, ~360 paths): the rename
  stall (five tail−head subtractions, low-water compares, an OR) was INSIDE the free-list
  LUTRAM read address, so the whole stall decision preceded the read, whose data preceded
  every dispatch write.
- **mideleg / kcc → e_r** (the schedulers' ready bits, ~1100 paths): two late roots in
  front of the wakeup broadcast. (1) `pt_ld_kill = inflight & (ld_sq | flush)` with
  `flush = redirect` put the whole redirect cone (CSR delegation, ROB head, SQ commit) in
  front of a landing load's `we_ld`; worse, `redirect` depends on `m_done_red` which yields
  to `ld_land`, so this was a **combinational loop** — nine TIMING-23 loops in the synth
  log, all through `ld_land`, an arc disabled inside the critical path, and Vivado's
  synthesis retiming silently OFF for the whole design ("The design has combinational loop
  or hierarchical design, no retiming"). Verilator reported the same loop as the UNOPTFLAT
  that had made `lint.sh` red. (2) `cf_link_wb = pend & ~fp_wb & ~m_wb_fe`: because
  in-core FP ops (FSGNJ/FMIN/FCMP/FMV/FCLASS) executed in M were routed to SH_FE, M was a
  second writer of the F/CTF shard, and the CTF link's broadcast waited on M's completion
  cone — the same mistake the code's own comment records for `m_wb_ie`.
- **m_addr → minstret** (28 levels): the live retire count through the ROB's same-cycle
  completion bypass.

## Round 1 (hm1): five cuts, no PRF change

1. `rv_soc_top.v` F1: each chunk slot (and the in-flight request) stores `va − CHB`;
   "is this the next chunk" is `c_vm == pc_ca`, a compare of two registers. The adder
   stays only on the demand-read address, in parallel. Always-on invariant: `c_vm ==
   c_va − CHB`.
2. `ooo2_lsu.v` L1: `pt_ld_kill = inflight & ld_sq` — the live flush reaches the kill only
   through the registered latch. A load landing in the redirect cycle lands harmlessly
   (every consumer orders its flush arm last). Zero timing loops in the synth log after.
3. `ooo2_core.v` W1: SH_FE is the F stage's shard alone (`d_cls_f`), in-core FP results
   take SH_LD like every other M result; `cf_link_wb = pend & ~fp_wb`; `wb_fe = fp ? :
   cf_link`; `wb_ld` takes `csr_rdata` BEFORE the latched result (this also fixed a latent
   stale-CSR-read hazard whenever a landing load held a CSR op at head). `m_wb_fe` is
   asserted zero, like `m_wb_ie`.
4. `ooo2_rename.v` R1: the free-list read addresses use the stall-free allocation view
   (`ar_*`, `br_*`); the stall gates only the head advance and the alloc outputs.
5. `ooo2_core.v` + `src/csr_file.v` M1: minstret takes the delayed retire count
   (`hpm_ret_q`); a head-gated op completes in its SECOND cycle at head (`m_head_q`) so the
   lag is invisible to a CSR read; `csr_file` drops the writer's own retirement after a
   `csrw minstret` (`minstret_wr_q`). One cycle per CSR op, trap, fence.i, system op.

## Round 2 (hm2): fpnew +1 stage, UART decode, registered ALU-scheduler ready, M rs3 off

- `fp_unit` PIPE_REGS 4 → 5 (fpnew's own 25-level path was −0.51; FP gives way).
- The UART write decode subtracted the 64-bit base per byte lane; `dmem_waddr[2:0]` is the
  offset (16-aligned base, window bounded by `is_uart_w`), as the read side already did.
- `ooo2_iq` REGRDY (u_iq_i, u_iq_i2 only): `d_ready` is a register, "≥2 free last cycle"
  ⇒ ≥1 free now (one dispatch per cycle; the held entry only ever moves). Asserted.
- M's third PRF read port (`ra3`) tied off: only an FMA routed to M under FS=Off could
  reach it, and it traps. The live read-port set is 9: M 2, ALUa 2, ALUb 2, F/CTF 3.

Rejected: registering the SH_LD/SH_FE array writes with the broadcast live. The schedulers
select in the SAME cycle as the broadcast (`srdy` takes the live hit), so a reader can read
in the write's register cycle; the always-on collision assertion caught the F pipe's FMA
doing exactly that. It would need a read-path bypass — see the memory
`wake-and-select-are-same-cycle`.

## Round 3 (hm2, −0.130 = hm1 within noise): the three shapes left at −0.117

- **fetch loop** (186 + 159 paths): round 4, below.
- **decode → free list → pending → dispatch stage** (34 paths, 26 levels): R2 — bank g of
  each free list reads at `h_hi + (g < h_lo)`, registers only; which slot allocates from
  which shard (the class decode) now selects among the banks' OUTPUTS. P2 — the pending
  lookup indexes the MAP's candidate (`r_sprs*_b/_c`); the intra-bundle bypass is applied
  at `r_prs*_b/_c` alone (its pending result is masked by `r_byp*` anyway).
- **SQ dv → scheduler e_r** (20 paths, 18 levels): the tail of every path into `e_r` was
  `sel → e_prd[isel] (LUTRAM, fan-out 248) → self_pr → 10-bit compare per source → e_r`.
  X1 replaces the wake-at-select with a **dependency matrix** (Henry Wong's form, for the
  intra-queue wake only): `dep[k][q][j]` is written at dispatch from the new entry's
  sources against every live destination (register to register), and read at select as
  one column. Results from other units still arrive by tag. `e_prd` becomes flops (removed
  from `tools/ram-manifest.txt`). Always-on assertion: the matrix agrees with the tag
  compare on every select.
- The dead third ALU's shard: `we_ie3/wa_ie3/wd_ie3/ra8/ra9` tied off, `alu3_q_v` asserted
  never.

## Round 4 (hm3, +0.007): the fetch served-slot hint

Implemented as described below and exact (the 60 M cosim's retire count is unchanged, the
wrong-hint assertion never fires). hm2's matrix row write also had to be unmasked: masking it
by the issuing column put the select (and the D$ response behind it) on 359 `dep` endpoints;
the column clear is ordered after the row write instead. hm3's census (1823 endpoints under
+0.35): fpnew's own pipeline +0.007 (66), ROB head → scheduler ready +0.020, ROB head →
`pl_q` CE +0.038 (257), `m_addr` → rename commit +0.08, PRF read → ALU +0.085. The fetch
family is gone from the list. Round 5a (the stall taxonomy / hpm_ev from a 41-root
snapshot, `scratchpad/edit5.py`, `scratchpad/hm5/ooo2_core.v`) is verified (lint, 240/0,
both cosims) but NOT built: hpm_ev_q sits at +0.072 in hm3 and another build+board was not
worth the margin.

## Next, for margin

- **Fetch loop** (done as round 4; kept here as the design note) (`pc_q → h0 compare → chunk select → shifter → aligner → next PC`, 22
  levels): the remaining head is the 64-bit slot-tag compare plus the fan-out-120 chunk
  select. The cut is a REGISTERED SLOT HINT: which slot holds the next PC's chunk is decided
  from the next-PC arms (same chunk / next chunk / predicted target / redirect) against
  the tags one cycle early; the served data selects on the hint, and the live compare only
  gates `imem_ok` (a wrong hint costs one bubble and self-corrects). Needs the arms out of
  `src/fetch.v` (or the tags into it); the riscv tb drives `imem_*` directly, so the hint
  must live in `rv_soc_top`'s adapter with `fetch` exporting `pred_tgt`/`redirect_pc` and
  an advance kind.
- **SQ `c_v` → LSU port** (`dv`/`k_take` live into `pt_start`, then M's done): the SQ's
  commit candidate is live from the ROB's `k_take` ("release costs no cycle on a store the
  LSU is waiting for"). Registering it costs one cycle per store drain; measure first.
- The L2 arbiter grant register (`scratchpad/edit3.py arb`, drafted, untested): a 1-LUT
  path with 5.8 ns of route in the base census; not in the top of hm1's.
- Vivado still prints "no retiming" — now for hierarchy, not loops. `-flatten_hierarchy
  full` is an untested flow knob (the RAM manifest checker keys on hierarchical names).


## Board (2026-09-17): the branch base is sick, and it is CTF-on-FP

Every verdict below is read from the console (`workloads/ubuntu/screenlog.0`) from the boot's
OWN `riscv: base ISA extensions` marker. `tools/board-gate.sh` reads past a byte offset taken
before programming and so picks up the PREVIOUS kernel's tail (it declared FAIL on boots that
went on to reach `login:`, and PASS-by-login on boots whose NIC had died) -- do not trust its
line right after a reprogram. A clean boot reaches `ubuntu login:` at 440-600 s with ZERO
`nfs: server ... not responding` lines; a sick one shows NFS stalls from 180-330 s and/or
`virtio_net eth0: NETDEV WATCHDOG: transmit queue 0 timed out` anywhere from 10 s to 783 s,
kernel alive. Timing is not a factor: everything below is at 111.11 MHz (`PROBE_CLK_DIV8=72`)
unless marked, where every variant closes with margin.

| bitstream | RTL | 1st NETDEV | 1st NFS stall | login | verdict |
|---|---|---|---|---|---|
| ALIGN_ctag 4baaa3e1 (166) | last board-clean main | - | - | 517 s | CLEAN (01:53) |
| same, re-run 08:08 | | - | - | 440 s | CLEAN (environment fine) |
| iw3_hm3 (166) | dd371e6d + rounds 1-4 | - | 183 s | - | sick |
| iw2_hm (166) | same at IW=2 | - | 247 s | - | sick |
| noM1 (166) | stack minus M1 | 76 s | 251 s | - | sick |
| stack111 | full stack | - | 237 s | - | sick |
| baseL1_111 | dd371e6d + L1 only | - | 918 s | 918 s | sick (login after the stalls) |
| r1b111 / A2 / A2a / A2b / W1abc / W1a / noW1 / w1p3_111 | base+L1 plus subsets | 11-147 s or - | 187-328 s or - | - | all sick |
| bis760 | 760fe336 pure (pre CTF-on-FP, no L1 needed) | - | - | 600 s | **CLEAN** |
| bis5ef | 5ef15e3d (CTF-on-FP) + L1 | 22 s | - | - | sick |
| bis5efb | 5ef15e3d + L1b (live flush kept) | panic 7 s | | | sick (Oops: epc=4 ra=5 in interrupt) |

So: the environment is clean (the old bitstream boots clean twice, the same morning, between
sick boots); every bitstream of this branch is sick, including the base with nothing but the
loop cut; the first sick commit is **5ef15e3d "control flow on the FP-shared pipe + block-RAM
read-ahead predictor"**, whichever loop cut makes its bitstream possible (L1: the live flush out
of `pt_ld_kill`; L1b: the flush kept, `m_done/m_done_red/m_done_wb` yielding to the raw
`lsu_pt_ld_done` instead of `ld_land` -- both lint-clean, both 240/0). 760fe336, the commit
right before it (Stage 3 C1-C5 + the minstret fix), builds without any cut (no DRC loop: the
loop is CTF-on-FP's) and boots clean. The commit is monolithic (core +458/-.. lines, PDW
18->21 so the predictor is not separable, resolution moved to the CTF pipe, the LQ's
device-load head gate, the LSU's speculative-load squash); the two commits after it
(4ee73d23, dd371e6d) were not tested separately because they cannot be clean on top of a sick
parent.

Earlier "W1 kills the NIC" readings in this document's history were wrong: W1a's boot (no
NETDEV, NFS stalls from 328 s) was indistinguishable from base+L1's, and noW1 died too.

**Why no simulation sees it.** `ooo2/tb_ooo2_linux.v` ties virtio off (`virtio_irq`,
`virtio_net_irq` = 0, rdata 0), the tiny128 DTB has no virtio node, and the UART never gets
an RX interrupt (ier=0, seip=0 through 700 M cycles of the GB5 boot). No cosim has ever taken
a PLIC interrupt, read a read-to-clear device register, or run a DMA ring. The board's
virtio-net under NFS root is the first thing that does, against CTF-on-FP's out-of-order
branch resolution, deferred squash (`fr_*`), early frontend resteer (`fe_red`) and the
irqop-injection interplay it had to add (`dcr_arm`, the "fe_red would flush the irqop out of
the frontend" comment). The one panic seen (epc=4, ra=5, "Fatal exception in interrupt")
is a `ret` through a corrupted link register inside an interrupt handler.

**Simulation stimulus (in progress, sim-only, `-DOOO2_IRQ_STIM`):** `rv_soc_top.v` pulses the
UART's PLIC source for 256 of every 32768 cycles and `src/plic.v` treats source 10 as enabled
at priority >= 1 whatever the kernel wrote, so the 8250's fasteoi flow (no action: mask + eoi)
runs thousands of times -- claim, complete, irqop injection, sret -- under the lockstep, which
follows the DUT's interrupts (`simmerv_set_forced_interrupt`). The storm can only start once
the kernel has mapped hwirq 10 (the 8250 probe, guest ~1.0 s = ~575 M cosim cycles; the tiny128
guest clock runs ~575 M cycles per guest second). Run: `VDEFS=-DOOO2_IRQ_STIM CYC=800000000
ooo2/run-ooo2-cosim-linux.sh`. The plain `run-ooo2-linux.sh` tb is stale (asserts on a
non-DRAM straddle at PA 0x22 before the kernel) and was not used.

## Timing: closed, with W1' instead of W1

W1 (route M's in-core FP results to SH_LD so the CTF link write never waits on M's completion
cone) is replaced by **W1'**: the base routing stays (in-core FP on SH_FE), `cf_link_wb =
cf_link_pend & ~fp_wb` as before, and M YIELDS the cycle (`m_fe_yield = cf_link_wb &
(m_shard == SH_FE)` in `m_done`, `m_done_red`, `m_done_wb`, registers only). Same timing
intent, no op changes shard. Lint clean, 240/0, IW=2 60 M cosim 13,494,359 (+0.02% on the hm6
row), IW=3 60 M cosim 11,684,956 (= the base count). **IW=3/HW=8 at 166.67 MHz: post-route WNS
+0.034** (`scratchpad/build-w1p3.log`), against +0.007 for the hm3 stack. The shipping RTL
candidate is `scratchpad/hm6/*` with `scratchpad/w1p/ooo2_core.v`; the IW=2 build at 166 is
`build-w1p2.sh`: **IW=2 at 166.67 MHz, post-route WNS +0.035** (`scratchpad/build-w1p2.log`; the
board is pointless until CTF-on-FP is fixed).

## Root cause and fix (2026-09-17, 11:30): the LSU classified a translate by the PORT's address

The storm stimulus caught it in 447 M cycles as `ooo2_lsu: non-DRAM access straddles a word:
pa=0fff0f0d nb=8 boff=5` -- an 8-byte misaligned load into the PLIC's region, started by the
load queue (`pt_start=1`) two cycles after a deferred CTF squash. The 40-cycle waterfall with
the backend signals (`scratchpad/stim-cosim5.out`) shows ROB entry 5, a wrong-path load
fetched after the mispredicted branch 4 (its squash deferred to head, `fr_set` at 300,
`cf_red_fire` at 311), translating at 303 with a garbage base register while the LSU port was
starting an older load, and offered by the queue with `x_v=1` from 308 although it was not
the head -- so its "DRAM, speculatable" bit was set for a device address.

`ooo2_lsu.v`: `eff_pa = (pt_start | take_next) ? pt_pa : t_paddr` and `xo_mem = pa_mem`
(from `eff_pa`). In the one cycle where the port starts or chains an access while M's
translate completes, the translating load is classified by the PORT's (DRAM) address. The
queue records `mem=1`, the head gate `mem | rob==rob_head` passes, and a device load issues
speculatively -- on the board a wrong-path read of a read-to-clear virtio ISR or a PLIC claim,
which is a lost interrupt, which is the dead NIC. An aligned wrong-path device read is silent;
only the misaligned garbage address tripped an assertion. Latent since CTF-on-FP: before it,
M resolved branches in order, so a load in M was never on the wrong path.

Fix (F1, `scratchpad/f1/`): `xo_mem` (and `xl_early`'s gate) from the translate's own PA
(`t_mem`); `ooo2_lq` exports `x_head`; the LSU asserts at every start that a non-DRAM access is
non-speculative (`pt_nonspec = pt_store | lq_x_head`, checked against the LSU's own region
decode, so a wrong `mem` bit fires it). Rules D12 and G7 in `docs/rtl-rules.md`; spec §8.
Verified: lint clean; riscv-tests 240/0; every `run-ooo2-*-tb.sh` PASS; the storm cosim to
800 M cycles lockstep-clean (236,499,567 retires, ~20 k spurious interrupts); **IW=3/HW=8 at
166.67 MHz: post-route WNS +0.017, and the board boots Ubuntu over NFS root to `ubuntu login:`
with zero faults, zero NFS stalls, no watchdog (`board-f1iw3`, 12:20)** -- the first clean
boot of anything on this branch. `src/run-tb.sh`'s `tb_fetch_pagecross.v` FAILs, but it fails
identically at 5ef15e3d (pre-existing on the branch, not touched here).

## Catching this class without the board (2026-09-17, afternoon)

Three layers, in the order they pay off:

1. **Cross-check at the consumer (done).** `ooo2_lq` recomputes the "DRAM, idempotent" bit
   from the PA it stores and `$fatal`s at the fill if the LSU's classification disagrees
   (rule D12; `DRAM_BASE`/`LRAM_*` parameters, a unit bench declares its synthetic world all
   memory with `DRAM_BASE = 0`). This fires on the first device load of any boot, in every
   cosim, with no stimulus -- the D12 defect would have died at cycle ~2 M of the plain
   tiny128 boot. The pattern generalises: any predicate a queue captures about its own
   subject gets recomputed from the subject at capture.
2. **The interrupt storm as a standing gate (done).** `ooo2/run-ooo2-cosim-storm.sh` (G7):
   `IW=`, `CYC=`, `DDR_LAT=` knobs; the DDR latency shape is what changes the interleavings.
   Run after any change to redirect, squash, the LQ/LSU start gates, the irq FSM or the PLIC.
3. **A device with DMA in the Linux testbench (assessed, not done).** The pieces: the
   virtio-mmio/blk/net RTL is in `src/` (the board's own devices); `rv_soc_top` still exposes
   the virtio passthrough (`virtio_*`, region 0x10002000, PLIC sources 11/12); the harness
   overrides Simmerv's MMIO loads with the DUT's (`armed_load_value`) and has an
   `cosim_inert_devstore`; `src/probe_cosim.cpp` still defines `cosim_dma_write` but the
   Rust side of simmerv-cosim no longer references it, so device DMA into memory would have to
   be mirrored into Simmerv again (or the run done without lockstep, and the plain
   `run-ooo2-linux.sh` tb is stale: it asserts on a non-DRAM straddle at PA 0x22 before the
   kernel). The retired `src/tb_cosim_linux.v` (453d9989^) shows the wiring: `virtio_mmio` +
   `virtio_blk` behind the passthrough, an AXI-to-memory bridge for the DMA, an SD backing
   store over DPI (that C model is gone too). Plus a tiny128 DTS variant with a virtio-mmio
   node at 0x10002000, interrupt 11. Estimate: a day, and it is the only layer that sees DMA
   ordering. virtio-blk first (the probe alone DMA-reads the partition table under interrupts),
   virtio-net with a testbench packet source after.

**The board verdict is trustworthy again:** `tools/board-gate.sh` anchors on the boot's own
`riscv: base ISA extensions` marker (the previous kernel keeps printing past the pre-program
offset while the board is reprogrammed), counts `nfs: server ... not responding` as a fault,
and has `REPLAY=<byte>` to judge an old boot from the console log without a board. Replayed
on today's log it matches every hand verdict: f1iw2 and bis760 PASS; W1a, A2b and baseL1
FAIL -- the last two reached `login:` with a dead NIC, which the old gate called PASS.

## What would have made this faster (the retrospective rule)

Tommy's standing rule: after every solved problem, ask which changes or tools would have made
us more efficient and effective at it, and act on the cheap ones in the same commit.

- **Reading the board (hours lost).** The gate judged bytes past the pre-program offset, so
  the previous kernel's lines produced false FAILs and a false PASS, and "boots to login"
  hid a NIC dead since 783 s. Done: marker-anchored gate with `REPLAY=`, NFS stalls as
  faults, `tools/console-boots.sh` (one line per boot). Rule: a boot's verdict comes from its
  own marker, never from "past an offset".
- **One bitstream variant per 45 minutes, sequentially.** Two Vivado builds cannot overlap on
  coffee (the IP OOC phase adds ~13 GB each). A shared IP output-product cache across
  worktrees would allow two at once; a build lock (`flock` around `make bit`) would have
  prevented the reap. Not done; estimate an hour each.
- **The variant generator's restore was wrong** (it put the m1b core back), which cost a
  snapshot and a false assertion. Rule: a swap/restore script restores exactly what it swapped,
  from the same list, and the verification runs read the snapshot, not the worktree.
- **25 minutes per rerun to the failing cycle** (447 M cycles at ~300 k cycles/s) for each
  added trace signal. The retired testbench had checkpoint/restore (`ckpt_wait_cmd`);
  Verilator's `--savable` would make a rerun from 440 M cycles a 30-second affair. Not done;
  the single most valuable harness tool for the next cosim-visible bug. Estimate: a day.
- **Assertions that name the fault but not its inputs.** The straddle assertion printed the
  PA; it took a rerun to learn the start path and whether M was at head. Done for that one.
  Rule: an assertion prints every input of its predicate.
- **The waterfall lacked backend state** (redirect kind, M's op, the LQ candidate, the ROB
  head). Done: `+pipe` now prints them. The retire trace still needs a compile-time tracer.
- **No simulation had ever taken a device interrupt.** Done: the storm, G7, the LQ
  cross-check. Left: the virtio model (layer 3 above).
- **Two shell self-inflictions**: `pkill -f` matching the calling shell, a `grep DONE` matching
  Vivado's log. Rule already in G6; it bit again.

## Gates left before this can merge

1. DONE: IW=2 at 166.67 MHz with the fix, **post-route WNS +0.039**, boots the board to `login:`
   with zero faults, zero NFS stalls, no watchdog (`board-f1iw2`, 13:00).
2. DONE: IW=2 60 M 13,494,359 (`cosim-expected`), IW=3 60 M 11,684,956, IW=2 300 M 72,713,900,
   all lockstep-clean on the final RTL.
3. Commit outside Mon-Fri 09:00-17:00 PDT, explicit paths, `.xpr` excluded (scheduled 17:05 PDT).
4. Open for the branch: `tb_fetch_pagecross.v` (pre-existing FAIL at 5ef15e3d too); the −8.6%
   adapter-MLP recovery from Stage 2; a real virtio-net model in the cosim tb.

## The memory backend program, C0: measurement (2026-09-17 evening)

Plan: `~/.claude/plans/cosmic-splashing-jellyfish.md` (approved). C0 built the instruments
before any backend RTL changes:

- **B1** `retire3` is a port of the core and the SoC; the testbench sums all three commit
  ports. Every IW=3 count before this summed two: the true tiny128 60 M count at IW=3 is
  **13,367,267** against 13,494,359 at IW=2 (−0.9%), not the −11.7%/−13.4% on record.
  `cosim-expected.txt` rows carry the width in a 5th column and the runner grades IW=3.
- **B2** the memory buckets in RTL: `MEM_HITSER`, `MEM_LDINFL`, `MEM_STDOOR`, `MEM_ALIAS_UNK`,
  `MEM_ALIAS_OVL`, `MEM_REORD`, `MEM_WPKILL`, `MEM_DEVWAIT` (r0319-r0320) and the queue
  occupancies `MEM_LQOCC`/`MEM_SQOCC` (r0321/r0322, values); `hpm_ev` is 39 bits; the SQ
  exports `l_block_unk_q`, the LQ `x_devwait`, the LSU `ld_busy`; `tools/perf-smol.sh memcpi`.
- **B3** the testbench prints a `perf stat`-shaped block (`perf-stat-sim`) from `hpm_ev_q`, and
  `tools/perf-cpi-stack.py` prints the composition of ST_MEM underneath it.
- **B8** OOC baselines and the census of the last passing IW=2 build:
  `docs/measurements/ooc-baselines-2026-09-17.md` (LQ 604, SQ 240/246 at 8/16, LSU 236,
  D$ 203, I$ 202 MHz at 6 ns). The worst placed family is `m_addr_reg -> hpm_ev_q_reg`.
- **B9** `tools/trace-limit.py` gained the backend knobs (load-use, load/store port
  occupancy, miss rate/latency/MLP, a mispredict drain); the sweep is
  `scratchpad/limit-sweep.txt`.

First reading (tiny128 boot, 60 M, IW=2, `scratchpad/c0-iw2-60m.perf`): dispatch held by a
FULL STORE QUEUE 37.5% of cycles; a store unaccepted at the D$ door 44.0% (this bucket
includes the one-cycle registered accept every store pays); a load access in flight 43.0%
(estimated miss wait 31.1%); a ready load the door did not take 12.0%; ST_MEM 11.5%; frontend
bubble 33.9%; mean occupancy LQ 0.62 of 4, SQ 3.76 of 8; reorders 5.6 and wrong-path kills 4.4
per 1k instructions; alias blocks negligible. This boot is store-drain-bound at the door
first (P8's wall), miss-bound second. The counters are the argument for C4a's one-access-per-
cycle door and C5's MSHRs in that order.

B9's sweep (in the measurements doc): the pipelined door alone is worth 40-70% at the ceiling
on integer code at IW=3; the store drain another 5-20%; W32 10-20%; the FP subtests are bound
elsewhere. The model cannot see MLP (uniform misses); C5 is judged on the cosim sweep.

**The counters are not free.** The first IW=3 build of C0 met timing at WNS 0.000 against
+0.017 before it: widening the event bus to 41 sources made `csr_file`'s per-counter,
per-cycle 16-bit event-code case a 644-endpoint family at +0.006 ns (`m_imm_reg ->
mhpmcounter_reg`, the census). The fix decodes the event code into a 6-bit registered index
when `mhpmeventN` is written (`hpm_esel`), so the per-cycle increment is a mux on a register;
the code form is kept as a shadow the cosim asserts against every cycle (it caught a bare
`if` covering one of two statements within a minute). Rule for the rest of the program: an
instrument's per-cycle logic reads registers only; decode at the write, never at the use.
With the registered select the C0 IW=3 build closes at **post-route WNS +0.025** (the counter
family is gone from the census); IW=2 60 M 13,494,359, IW=3 60 M 13,367,267 and IW=2 300 M
72,713,900 are exact with the instruments in; riscv-tests 240/0. Committed as C0.

## C1: mul/div off the ordered pipe (2026-09-17 evening)

A mul/div is dispatch class M (`d_cls_m`), shares the F/CTF queue (NF 5 → 8) and drains from
its select register into an MD stage (`iss_md`, the port's third drain): the units start from
the port's forwarded reads, the result is latched on the unit's done pulse and written to SH_FE
by the stage's own tag when the FPU and the CTF link are not writing (M yields the shard as it
does to the link); the ROB completes through a ninth port. M, `wb_ld`, `m_unit_ok` and the
ordered queue no longer know a multiplier exists; ST_MUL/ST_DIV now mean "the MD stage holds
one". One defect, found by a five-signal trace on `rv64um-p-mul`: an issue and a writeback in
the same cycle, with the writeback's clear written after the issue's set, lost the op while
mul3 computed it; the next issue started into a busy multiplier, which ignores starts, and the
stage waited forever. The clear now precedes the set, and "a start into a busy unit" is an
always-on assertion. The two LOAD/STORE completion events were re-sourced from registered
landings (they were the last counter family on the worst path).

Verified: lint clean; riscv-tests 240/0 (the rv64um set included); every unit bench; IW=2
60 M 13,490,465 (−0.03% on the boot: few multiplies there, and the port now shares with every
branch), IW=3 60 M 13,371,386 (+0.03%); the 500 M storm clean. Builds and the board gate (with
the new stress) follow.

## B10: the board gate's PASS now includes a userspace stress (2026-09-17 21:00)

Tommy's red alert: the committed IW=2 stack passed the gate at `login:` and crashed Geekbench
6.9 h later (an instruction page fault at kernel text `ffffffff804a09f6`, cause 0xc, in the
geekbench process; not debugged, per Tommy). `tools/board-gate.sh` now finds the board as this
host's NFS peer on 192.168.1.x, waits for sshd, and runs Geekbench 5 over ssh for `STRESS_S`
seconds (default 900); PASS needs the run to use its whole budget (exit 124) with zero fault
lines in dmesg and on the console during it. `STRESS_ONLY=1` runs the stress on the board as it
is (the plumbing was proved on the live board: 180 s, two subtests, PASS). The full Geekbench
run remains the release gate; a 15-minute slice does not cover a 7-hour crash.

## B5: the lockstep is byte-exact on stores (2026-09-17 22:30)

`probe_retire` carries the store's raw value and log2 size (the SQ's committing entry through
its landing bypass; the LSU's `cos_data/cos_size` for M-path stores; 4'hF = not checkable);
simmerv's record gained `mem_size` beside its existing `mem_rdback` (the aligned word after the
store, which nobody had used). `store_data_ok()` requires equal sizes and equal bytes on every
lane the DUT wrote within the word; unchecked classes are counted on the progress line.
Verified: 60 M IW=2 clean with 3,051,638 stores checked and 4,469 unchecked (the AMO/SC/cbo/MMIO
classes), retire count unchanged (observation only); `+st_corrupt=35` aborts at retire 35 with
`STORE DATA MISMATCH ... value=...80044080 ... word-after=...80044000`. The first run reported
value 0 for the boot's first `sd`: the commit-time read of `data[kc]` missed the landing bypass
that `c_data` has (rule G9's corollary). Simmerv change: `cosim_mem_size` in `mmu.rs`/`cpu.rs`
and the header (uncommitted in ~/simmerv, on top of the MMIO-diagnostic edit already there).

## C1 on the board and the timing (2026-09-17 22:45)

IW=3 build: post-route WNS **0.000** (met with no margin; C0 had +0.025). Worst path
`fe/d2_rs3_reg[2] -> stg_r_l_reg[2]`, 15 levels (4 MUXF7 + 2 MUXF8: a PRF read mux), 79% route:
the third-source read the dispatch stage now feeds for the mul/div class as well. Not a
rejection (the rule is WNS >= 0) but the margin is gone; the census is banked (`ps_out_reg -> m_result_reg`
63 paths at 0.000, `md_rd_v_reg -> u_iq_i/e_r_reg` at +0.022 is the one new family). IW=2 build:
core WNS **+0.070** (the top-level +0.010 is `virtio_blk_inst -> virtio_blk_backend`, outside the
core). Board, IW=3: `login: 1 faults: 0`, then 900 s of Geekbench (3 subtests) with zero faults --
**BOARD: PASS** (23:14). IW=2 gate chained behind it.

## B4 as built: memrand, a generated program under the lockstep (2026-09-17 22:45)

The plan's B4 was a unit bench of LQ+SQ+LSU+D$+MMU with its own golden model. That model would
have to replay the core's own protocols (M's translate pass, the ROB's irrevocable pointer, the
dispatch stamps), and every one of them changes in C3-C8. The lockstep already judges every load
(rd) and, since B5, every store byte, so the bench is a PROGRAM instead: `workloads/memrand/gen.py`
emits a straight-line stream of 200 k random ops (loads/stores of every size, aligned and not,
line and page straddles; AMOs; LR/SC pairs; FP loads/stores; cbo.zero/clean/flush; fence,
fence.i, sfence.vma; pointer chases whose next address arrives with a load's data; a rare remap
of a megapage under live accesses) over a 128 KiB region reachable through three VA aliases
(identity, W0, W1) in S-mode under Sv39, plus an NC window over a DMA region. The testbench's
DMA agent (`+dma_rand=<seed>`) bursts random bytes into that region on its own schedule, mirrors
each beat into simmerv and raises PLIC source 11; the program's handler sums the window through
the NC mapping and acks -- race-free by protocol, where a polled flag would not be (a beat
landing between a poll's execution and its retire is applied to the reference before that poll
is compared). Each seed is ~1.5 M cycles: `make -C workloads/memrand sweep SEEDS="1 2 3 4"`.
Invariants stay where they belong (always-on assertions in the RTL); the bench supplies the
orderings. The unit-level random benches (`tb_ooo2_lqsq_rand`, `tb_ooo2_cache`) remain for the
structures they cover.

## Retrospective: C1 and C2 (Tommy's rule: what would have made this faster)

- **The store check found two things in its first hour that no gate had seen in a year**: a
  commit-time read that missed a landing bypass, and cbo.zero's size. Neither was a core defect,
  both were holes in the instrument. The lesson is the one already in G9: a compare that skips a
  class silently is not a compare. The unused `mem_rdback` had been exported by simmerv since
  2026-08-20 and never read -- an exported observable nobody consumes should be an assertion in
  the runner ("the reference exports N fields, the DUT supplies N").
- **Editing a running shell script corrupts its own run.** The 300 M runner printed a syntax
  error at its end because its file was rewritten under it (bash reads incrementally). Scratch
  copies for long runs, or edit the file only between runs; the same rule as "never rebuild an
  obj_dir a run is using".
- **One obj_dir serialises every cosim.** Tonight's queue (300 M x2, blk, storm, sweep, memrand)
  is two hours of wall time on a 32-core box because the runner owns one build directory. A
  per-config obj_dir (`obj_dir_ooo2_clinux.<hash>`) would let the IW=2 and IW=3 binaries and the
  storm coexist and cut the verification wall time by 3x. Worth doing before C3.
- **A unit bench that replays the core's protocols is a second core.** B4 as planned (LQ+SQ+LSU+
  D$+MMU with a golden model) would have re-implemented M's translate pass, the ROB pointer and
  the dispatch stamps -- everything C3-C8 change. The lockstep with byte-exact stores is the
  golden model; the bench is a program. Ask "what already judges this?" before writing a model.
- **Timing margin is a budget, not a verdict.** C1 spent the whole +0.025 (WNS 0.000 at IW=3)
  on the third PRF read the dispatch stage now feeds for mul/div. The census names it
  (`d2_rs3_reg -> stg_r_l_reg`); the next increment that touches dispatch reads must find slack
  first (rule I2: OOC before redesign).

## C3 (SYSQ) design notes, from the RTL as it is (2026-09-17)

What M still does besides memory, with the signals to move (`ooo2_core.v`):
- **Traps**: `xtrap_v = m_valid & (m_fault | m_ill_eff | (m_mem_op & m_lsu_flt))`; cause/tval from
  the fetch fault, the illegal latch or the LSU (`m_lsu_fc`, `lsu_fault_tval`); `cot_fire =
  u_csr.trap_v`. System ops: `m_is_sys` (opcode 11100), `m_is_irqop` (the irqop encoding),
  `csr_redir_v/_trap/_tgt` from `csr_file`.
- **Ordering at head**: `head_block = m_valid & ((m_needs_head & ~m_at_head) | (m_instret_rd &
  ~m_head_q))` -- only a CSR read of instret takes the second cycle at head (M1b).
- **Completion**: `m_unit_ok` (fault/illegal immediate; memory = `lsu_done`; else single cycle),
  sticky in `m_unit_done_q` with the result and fault latched (a pulse in a held cycle was lost).
- **Redirect**: `m_done_red = (m_unit_ok_nomem | m_unit_done_q) & ~head_block & ~ld_land & ~fp_land
  & ~m_fe_yield`; `m_red_fire = m_valid & m_done_red & (csr_red | m_is_fencei)`, `csr_red = xtrap_v |
  (m_is_sys & csr_redir_v)`; `redirect = m_red_fire | cf_red_fire` (the branch squash at head,
  mutually exclusive by the head's uniqueness). The invariant assertions compare `m_red_fire`
  against `m_red_ref` (the memory-inclusive done) every cycle.
- **CSR**: `m_wb & m_is_csr` writes `csr_rdata` at the second cycle; `m_fe_yield` and the LD shard
  write (`m_shard`) carry the CSR result.
- **Drains**: `m_cbo_wait = m_is_cbo & sq_av_any`; fence.i FSM on `dmem_idle`.
SYSQ shape: keyed by ROB index, written (a) at dispatch for decode faults/irqop/illegal (already
known at decode), (b) by the F/CTF port at issue for CSR/system ops (three PRF reads, `xf_rs1`),
(c) by the LQ/SQ translate fault; executed when its entry is the ROB head from a registered
condition; `xtrap_v`, `upd_valid`, `m_red_fire` sourced from it; brought up as a shadow beside M
with an every-cycle equality assertion, then switched (retire-identical at both widths).

## memrand's first night (2026-09-18 00:15): three program fixes, one reference bug, no core defect

Seed 1 at IW=2 passes (50 k ops, 1.95 M retires, 95 DMA bursts handled through the interrupt
protocol). On the way:
- **mstatus read-modify-write**: MPP's reset value is not architected; the DUT has 0, simmerv M.
  No real software reads it before writing it, so the lockstep never saw it. The program writes
  mstatus whole.
- **A 64 KiB-aligned megapage**: the remap wrote a level-1 leaf with PPN[0] != 0. The DUT raised
  the page fault the spec requires; **simmerv accepted it** -- its misaligned-superpage mask was
  `(1 << j) - 1`, one bit per level, instead of `(1 << 9j) - 1`. Fixed in `~/simmerv/src/mmu.rs`
  (Tommy: "we should fix it"); `gen.py --misaligned-megapage` keeps the case as a test where both
  models must fault at the same retire -- verified after the fix: `MEMRAND-TRAP cause=d` with no
  divergence (`make cosim SEED=1 TAG=-mm GENFLAGS=--misaligned-megapage`). ~/simmerv is behind
  upstream (Tommy: update at a convenient point, not now); the fix and the `mem_size` field must
  survive that update.
- **Page-straddling misaligned accesses**: the LSU raises address-misaligned for a misaligned
  access whose span leaves the page (documented in `ooo2_lsu.v`, Linux emulates it). The pointer
  chases now stay inside the pointer's page; simmerv agreed with the DUT's trap.
- A zero-initialised region let zero loads wipe the value pool within a few hundred ops; the
  region is seeded with random data now (`region_init`, 128 KiB in .rodata).
The stream is memory-bound by design: ~16 cycles per instruction (straddles, AMOs, fence.i every
~250 ops, sfence.vma, divides), so 50 k ops is ~4 M cycles per seed.
Seeds 1-4 at IW=2 and 1-3 at IW=3 pass (00:13); the same seed's final region checksum is identical
at both widths (the DMA sums differ only by burst timing). Runs of different configs now proceed in
parallel (rule G10), so the six seeds took under two minutes.

## The DDR-latency sweep (B7, 2026-09-17, `docs/measurements/2026-09-17-mem-sweep.txt`)

| IW | lat 4 | measured | lat 80 |
|---|---|---|---|
| 2 | 19,609,228 | 13,490,465 (−31.2%) | 10,134,592 (−48.3%) |
| 3 | 18,946,720 | 13,371,386 (−29.4%) | 10,159,647 (−46.4%) |

Retires at 60 M cycles on the tiny128 boot. Two facts for the program: the measured DDR shape
costs the boot 31% of its retires against a 4-cycle memory (the MLP ceiling C5 is judged against),
and **IW=3 retires 3.4% FEWER than IW=2 with a 4-cycle memory** -- with memory out of the way the
3-wide pipe is slower on this boot, which points at the dispatch stage / the shared F-CTF-MD port
rather than at width; worth its own census before C7 widens the memory path.

## B6 verified (2026-09-18 01:40): the disk-backed lockstep

`make -C workloads/tiny128 cosim-blk` at 1.5 G cycles: the kernel probes virtio-blk (`[vda] 8192
512-byte logical blocks`), the initrd's S99blkcheck mounts the ext4 image at 7.6 s of guest time,
`data.bin: OK`, the write-back copy re-read past the page cache matches, `BLKCHECK-OK`. Lockstep
clean throughout: 488,157,252 retires, 66,857,823 stores byte-checked, 13,700 DMA read beats and
28,956 write beats mirrored into the reference. The first grading missed the verdict because the
testbench's `[c=...]` progress line landed inside the word (`BLKCHEC[c=...]\nK-OK`): the runner
and the memrand Makefile now read the console with those lines removed and the newlines joined.
A 300 M run cannot reach the check (the kernel is at 0.24 s then), so this is a per-batch gate
like the long guest, not a per-edit one.

## C3 step plan (2026-09-18 01:50), from the interfaces as they are

`csr_file` already has the two ports the SYSQ needs to drive: `upd_{valid,is_csr,func,addr,src,pc}`
(today `upd_valid = m_is_sys & m_done_red`, `upd_src = m_csr_func[2] ? zimm : m_rs1_val`,
`upd_addr = m_imm[11:0]`) and `xtrap_{v,cause,epc,tval}` (today `xtrap_v & m_done_red`, epc = `m_pc`);
its redirect outputs (`csr_redir_v/_tgt/_trap`, `csr_illegal`) are combinational from those. So a
SYSQ entry is exactly that payload keyed by ROB index: `{v, kind: SYS | XTRAP (| REPLAY in C4b),
is_csr, func, addr[11:0], src, pc, cause[3:0], tval}`; `raddr` (the CSR read) is `addr` of the
firing entry.

1. **Shadow** (`ooo2_sysq.v`, no behaviour change): written (a) at dispatch for `d_illegal`,
   `d_fault` (cause/tval from decode), `d_is_irqop` (the three slots' ROB indices), (b) at M's entry
   capture (`m_*  <= q_*/x_*`, ~line 3640) for `q_is_sys`: `{x_rs1, q_imm, q_csr_func, q_pc}`, (c) at
   the LSU's latched fault (`m_flt_pulse`: `m_lsu_fc`, `lsu_fault_tval`). Cleared on `redirect`
   (every entry is younger than the head or already retired). Every cycle: when M drives
   `upd_valid`, the entry at `m_rob_idx` is valid, kind SYS and equal in every field; when
   `xtrap_v & m_done_red`, the entry is kind XTRAP with the same cause/tval/pc. `$fatal` otherwise.
   Gates: lint, 240/0, 60 M both widths (parallel now), the storm.
2. **Switch the drives**: `upd_*` and `xtrap_*` come from the SYSQ entry at `rob_head_idx`, fired by
   a registered `sysq_fire` = head entry valid & M's own done-gates for that op (`~head_block`,
   `~ld_land`, `~fp_land`, `~m_fe_yield`, i.e. the same cycle M would have fired); M's `csr_red`/
   `m_red_fire` become the SYSQ's. Retire-identical at both widths by construction (the shadow
   proved the fields; the fire cycle is M's). Gates as 1, plus the board.
3. **Issue through the F/CTF port**: `d_cls_m`-style class for CSR/system ops (the MD drain's
   pattern: a fourth drain `iss_sys` writes the SYSQ entry from the port's forwarded rs1 instead
   of executing); `d_cls_l` loses them; M loses `m_is_sys/m_is_csr/m_is_serialize` arms and the
   `head_block` instret hold moves to the SYSQ's fire condition. Retire count changes (record it).
4. **Delete** the dead M arms; the trap-target cone now starts at a LUTRAM read at `rob_head_idx`
   (a flop) -- the timing lever the plan named.

## C3 step 1 done, step 2 tried and rejected for timing (2026-09-18 02:30 / 08:00)

Step 1 (the shadow, `sysq_*` arrays in `ooo2_core.v` keyed by ROB index, written at dispatch for
decode faults/illegal, at M's take for a SYSTEM-opcode op's operands and the FP-off illegal, at
the LSU's fault pulse) held under every gate without a single assertion: 240/0, 60 M and 300 M
at both widths bit-identical, the 500 M storm, memrand. Step 2 makes `csr_file`'s `upd_*` and
`xtrap_*` payloads the entry's (read at `m_rob_idx`, a flop) with `upd_valid`/`xtrap_v` derived
from the entry's kind; M's own classification and payload stay as every-cycle equalities. The
CSR READ address (`raddr`) stays M's `m_imm` flop on purpose: `m_imm -> csr read mux ->
csr_rdata -> m_wb_val -> x_rs1` is a recorded FMAX path and a LUTRAM read in front of it is
step 3's problem to solve (read the head entry's address into a flop a cycle ahead).

**Step 2's verdict (08:00):** retire-identical under every gate (240/0, 60 M and 300 M at both
widths, the storm, memrand), IW=2 core WNS +0.030 with a board PASS (900 s stress) -- and IW=3
**−0.030** (from 0.000), no bitstream. The IW=2 census shows the new worst family:
`m_rob_idx_reg_rep -> u_iq_i2/e_r_reg` (19 levels): the LUTRAM read of the entry sits in front
of `csr_file`'s combinational `csr_illegal`/`redir_valid`, which reach the schedulers' kills. So
step 2 is reverted (the shadow stays; the arrays have no synthesis reader) and its lesson is
the design constraint for step 3: **the payload csr_file consumes must be flops**, registered
from the head entry a cycle ahead of the fire (the head is known a cycle ahead: `rob_head_idx`
is a flop and the entry is written long before). The reverted edit is the inverse of
`$S/c3-step2.py` (payload ports `upd_{is_csr,func,addr,src,pc}`, `xtrap_{cause,epc,tval}` from
`sysq_*[m_rob_idx]`, `upd_valid`/`xtrap_v` from the entry's kind) -- twelve lines, easy to redo
once the payload is a flop.

## Step 3 facts, for the next session

- Serialization is already at dispatch: `ser_block = ser_inflight | (d_valid & d_is_serialize &
  ~drained)` (~line 3625) holds younger dispatch until a serializing op has drained; M's
  `m_is_serialize` assertion only checks it. Moving the op off M does not change this.
- A CSR read's value reaches the register file through M's writeback (`m_wb_val = m_is_csr ?
  csr_rdata : m_byp_val`, `m_wb_{ie,ld,fe}` by `m_shard`); a trap retires through
  `c_kill(m_valid & m_done & m_trap)`, `m_trap = xtrap_v | (m_is_sys & csr_redir_trap)`. In step 3
  the SYSQ fire must supply both: a completion port into the ROB (a tenth `w_v`, the MD stage's
  pattern) with the CSR read's value onto SH_LD through the yield that M uses today, and the
  kill.
- `m_needs_head = m_is_sys | m_redirect | m_is_fencei | m_fault | m_ill_eff | (m_mem_op &
  m_lsu_flt)` with the instret second-cycle hold: the SYSQ fire condition inherits exactly this
  for its own ops; fence.i, AMO and cbo stay M's (memory-side) until C4b.
- The F port's fourth drain (`iss_sys`, the `iss_md` pattern at ~line 1302) writes the entry from
  `xf_rs1`, `j_*`'s imm/func/pc; the op then owns nothing but its ROB slot until the head.

## Step 1 on the board (2026-09-18 09:10) and the step-3 shape that follows from step 2's lesson

The committed tree (390d5028) at IW=3: core WNS **+0.007**, board PASS (login, 0 faults, 900 s
of Geekbench, 3 subtests). Every increment through C3 step 1 is board-clean at both widths.

Step 2 showed that a payload read from a LUTRAM at `m_rob_idx` is one level too many in front
of `csr_file`. With the op still in M, M's own flops ARE the registered payload, so there is
nothing to gain from the arrays until the op leaves M. That fixes step 3's shape:

- **The SYSQ is an in-order FIFO, not a ROB-indexed table.** System-class ops (CSR, ecall/ebreak/
  xret/wfi/sfence.vma, the irqop, a dispatch-time illegal or fetch fault) take a FIFO slot at
  dispatch in program order (the three slots allocate in order; depth 4 is plenty, dispatch
  holds when full). The payload is filled at dispatch (illegal/fault: cause, tval, pc) or by the
  F/CTF port's fourth drain (`iss_sys`: rs1 or zimm from `xf_rs1`, addr, func, pc). The FIFO's
  HEAD payload lives in flops, so the fire is a flop compare: `head.v & head.filled &
  (head.rob == rob_head_idx) & ~ld_land & ~fp_land & ~m_fe_yield` (+ the instret second cycle),
  and `csr_file` is driven from flops as today. Data faults stay where the faulting op is: in
  M until C4b, then the LQ/SQ head entry (flops), never in the FIFO.
- **On fire:** `upd_*`/`xtrap_*` from the head flops; a tenth ROB completion port (`sysq_wb`) or
  `c_kill` for a trap; a CSR read's value onto SH_LD through M's yield (`m_fe_yield`'s pattern,
  `we_ld` gains the arm); pop. **On redirect:** clear the FIFO (everything in it is younger than
  the head, and the firing op has already left).
- **M loses** `m_is_sys/m_is_csr/m_is_serialize` and `m_needs_head`'s sys arms; `d_cls_l` loses the
  system class (`d_cls_fc` gains it, like `d_cls_m` in C1). The dispatch-side serialisation
  (`ser_block`) is untouched. The ROB-indexed shadow of step 1 becomes the FIFO's own
  always-on check against M until M's arms go (step 4), then a plain invariant set.
- Retire count changes (CSR ops no longer occupy the ordered queue): record it; storm essential
  (the storm's interrupt entries are `csr_file`'s own, not the FIFO's).

## C3 step 3 in simulation (2026-09-18 10:12): system ops through the F port, the SYSQ fires at head

Built as the plan above: class S at dispatch (SYSTEM opcode, irqop included, illegal/fault
excluded), the fourth drain `iss_sys`, one register (serialisation at dispatch makes depth 1
exact, asserted), `csr_file` driven from its flops, M's ports reused for the SH_LD write and the
ROB completion (M empty, asserted), `c_kill` for the trap, the one yield gate `port_yield`.
Two defects on the way, both of the same shape -- **a site that enumerated its sources**:
- `wb_ld`'s data mux consulted `m_is_mem`/`m_is_amo` before the SYSQ arm; with M empty those
  bits are the last op's, so a CSR read's value reached SH_LD as load data while the retire
  record (from `csr_rdata` directly) looked right. Every 240 test failed at retire ~100.
- `fe_red_pulse = m_red_fire | fr_set | dec_red` did not include the SYSQ's redirect, so the
  backend flushed and fetch kept its predicted path (OpenSBI's mtopi probe: the illegal-CSR
  trap killed the op, the frontend never restarted, the next retire was pc+8); and the cosim's
  trap record was M-based (`m_valid & m_done & cot_fire`), so the SYSQ's traps produced no record.
  `cot_take = cot_fire & ((m_valid & m_done) | sy_fire)`, `sy_insn` for the record, `red_trap`
  (the HPM event) includes `sy_trap`.
Result: 240/0, 60 M at both widths **bit-identical counts** (13,490,465 / 13,371,386: the fire
cycle is exactly M's first cycle at head); 300 M IW=2 +0.05%, IW=3 −0.14%; the storm clean with
19,778 interrupts delivered (17,924 before: the irqop no longer waits for M's take); memrand;
every bench. **Builds: IW=3 core WNS +0.061 (from +0.007), IW=2 +0.058 (from +0.030)** -- taking
the system class out of M's head-block/needs-head cone is the timing lever the plan named --
and **BOARD: PASS at both widths** with the 900 s stress (12:17). Rule candidate (I12): a redirect/writeback/record site lists
its sources in ONE place -- `redirect`, `fe_red_pulse`, `fe_red_tgt/seq`, `redirect_is_trap`,
`c_kill`, `rob_w_valid/idx`, `we_ld/wa_ld/wb_ld`, `cot_take` all had to learn the SYSQ, and
two of them were missed; a table of "who fires this" would have made the omission visible.

## C4a design notes (2026-09-18 11:00): the pipelined tagged load path, from the interfaces as they are

Facts (ooo2_lsu.v / ooo2_lq.v / rv_cache.v / rv_soc_top.v at 390d5028, and `git show da28895b`):
- **The D$ read door is already tagged.** `rd_tag[RTW-1:0]` (RTW=4, "room for a load-queue index")
  is opaque and echoed as `rd_resp_tag`; `rd_ack = accept & rd_req` is COMBINATIONAL (`accept =
  acc_slot & ~inv_go & ~inv_busy & ~fin_hazard & ~f_replay & ~f_solo & ...`, `acc_slot = (st==S_IDLE
  | fin_wr) & ~fill_banks`), `rd_valid/rd_data/rd_resp_tag` are registers; the header says a
  requester must advance on `rd_ack`, never hold for the response. A hit is accept(N) ->
  S_CHECK(N+1, banks addressed from `a_live` in the accept cycle) -> `rd_valid` in N+1's edge; the
  door is closed during S_CHECK, hence one read per two cycles -- the P0 door item. A miss copies
  the request into the ONE MSHR (`f_*`), the FSM returns to S_IDLE, and F_ANS answers the read from
  `linebuf` by tag; a second miss holds S_CHECK (and the port) until the fill lands.
- **The LSU's single-access limit is one latch triple** (`own_pt/own_pt_st/src_pt`) naming the one
  access in flight, `ld_inflight = own_pt & ~own_pt_st`, plus the FSM parking in S_LD; `pt_start`
  and `xl_early` both require `st == S_IDLE`. **The core's limit is one register**, `ld_inflight_idx`
  (the LQ index of the one load), feeding `ld_land -> lq_l_*`, the ROB port, `ld_wb`, the record.
- **The LQ is already written for out-of-order landings**: `l_v/l_idx` land by index, `d_ready`
  requires the tail slot free (`~v[tail]`), the candidate is `acc` (oldest unsent), and the SQ's
  alias matrix (`e_block`) is per entry.
- **da28895b did exactly this on the old tree** (+14.1% at DDR latency 4, +0.63% at 80: one MSHR,
  so misses never overlap; ldbench 8-stream 4.00 -> 2.25 cycles/load): `LDTW`, per-tag format state
  `o_v/o_nb/o_sgn/o_fp/o_boff[NLD]`, `ld_fast_ok = start_ok & ((pt_start & ~pt_store) | xl_early) &
  ~xword & ~eff_unc & pa_dram` starting a tagged read without taking the FSM, `mem_rfast/mem_rtag`
  beside `mem_ren`, the response `ld_fast_ret = mem_rvalid_c & o_v[mem_rtag_resp]` landing by tag,
  `port_free = ~mem_rbusy & ~mem_ren` folded into the starts, `~o_v[tag]` against tag reuse; the
  slow path unchanged with `slow_rv = mem_rvalid & ~ld_fast_ret` (one landing port). In the SoC:
  `TAG_SLOW = 4'b1100`, `lsu_tag_req = rfast ? {2'b00, tag} : TAG_SLOW`, `lsu_gen` deleted,
  `c_rd_want` gained `~is_dev_r` (a device read is never acked by the cache: a stuck want froze
  every load at the UART LSR). Core: `ld_land = fast | slow`, `ld_land_idx = fast ? rtag : ld_inflight_idx`.

C4a on HEAD, in order (each a gate-clean step):
1. **The tagged fast path** (da28895b re-done; LQ 4 -> 8, RTW stays 4: `{00, idx[2:0]}` + slow
   `1100` + walkers): the LSU gains `LDTW=3`, the per-tag format array, `ld_fast_ok`, the tag-echo
   landing; the SoC the tag space and `c_rd_want & ~is_dev_r`; the core the two landing arms and
   `ld_land_idx`. The B-rules: the tag is the LQ index the LQ allocated, the LQ pins the slot until
   the landing, `~o_v[tag]` at the start. Expect ldbench 8-stream ~2.25 and the boot +1% at the
   measured DDR shape (da28895b's numbers; the sweep table is the baseline).
2. **The door self-loop** (P0): `acc_slot` also true in S_CHECK when the check is a plain cached
   read hit-or-miss that does not need the banks next cycle (the requester presents the next
   request during S_CHECK; `fin_hazard`'s row rule extends to "the row S_CHECK will write on a
   miss"); `wip/cache-selfloop` (baf96dff) has the shape but no ack -- the `rd_ack` contract above
   is the missing piece. Expect ldbench 8-stream -> ~1.2 and the boot's `MEM_HITSER` bucket to fall.
3. Everything a straddle, an uncached load, an AMO or LR/SC still parks (the slow path) -- as
   da28895b left it; C5 (4 MSHRs) is what makes misses overlap; C4b puts the AGU on the ALU ports.
Assertions to carry: rv_cache.v:1335 (a bank row read and written in the same cycle -- "for the day
the LSU goes multiple-outstanding"), the LQ's ten (out-of-order landing hazards), the SoC's two
(two responses at once; a waiting read whose tag/address moved), plus new: a fast response whose
tag has no `o_v`, a slow response with a fast one in the same cycle (one port), and the alias
matrix's registered answer never less conservative than the live one (ooo2_core.v:1824).

## C4a step 1 built (2026-09-18 12:30, worktree smolrv64-wt-c4a, branch wip/c4a)

The tagged fast load path, ported from da28895b onto the step-3 tree: the LSU's `LDTW`, the
per-tag format arrays, `port_free`/`~o_v[tag]` in the starts, `rfast_q/rtag_q` written at the
`mem_raddr` sites, `slow_rv` for the FSM's own response, `pt_fast_done/pt_rtag`; the SoC's tag
space (`TAG_SLOW`, `dc_rv_fast`, `c_rd_want & ~is_dev_r`, `dmem_ren_slow`, the two assertions);
the core's two landing arms and `ld_land_idx`; the riscv bench answers a fast read in the cycle
it is presented. HEAD already had the LQ's `~v[tail]` allocation rule. One defect: **a fast load
starting in the redirect cycle was not killed** (the LQ's candidate in that cycle is younger than
the head, the slow path's `ld_sq` kills that case) -- 114 riscv-tests failed and the pending
scoreboard saw a writeback to a register that was not pending when the dead load's response
landed in a reallocated slot; `o_kill` now covers `ld_fast_ok` in the flush cycle. Result: 240/0,
60 M clean at both widths, **+3.6% (IW=2) / +3.5% (IW=3)** on the boot at the measured DDR shape
(da28895b had +14.1% at latency 4 and +0.63% at 80 on the old tree). ldbench: latency 5.00
cycles per load unchanged, throughput 4.00 -> 2.00 (da28895b reached 2.25), overlap 1.24x ->
2.49x -- the door's one read per two cycles, the P0 item, is now the visible floor. The long
gates (two storms, one at DDR_LAT=40; 300 M x2; memrand 6 seeds; every bench) and the builds run
in parallel.

## C4a step 1, the long gates (2026-09-18 14:10): one pre-existing ordering hole, one timing miss

- **memrand seed 2 (both widths) and the 300 M IW=3 boot diverged on a load returning zero.**
  The plusarg-aimed LSU/door trace (`-DLSUDBG +dbg_line=<PA>`, kept) showed the sequence: a
  `cbo.zero` two instructions YOUNGER than the load zeroed the line (a solo write, accepted
  after 30 cycles of waiting for the fill machine), then the older load, still queued, read it.
  `m_cbo_wait` covered older STORES (`sq_av_any`, rule C5) but not older loads: a load that has
  passed M sits in the load queue with its address known until it lands, and the cbo in M never
  waited for it. The hole predates the fast path (a queued load blocked by an older store had
  the same exposure); the fast path's two-cycle response widened it enough for memrand to hit
  it in its second seed. Fix: the load queue exports `av_any` (an entry with a known address is
  older than M's op, since a younger load has not had its translate pass) and
  `m_cbo_wait = m_is_cbo & (sq_av_any | lq_av_any)`; no deadlock, because a blocked older load
  waits only on older stores or on being the head, both of which precede the cbo. memrand seed
  2 passes; the storm, 300 M IW=3, seven memrand seeds and the benches re-run.
- **IW=3: −0.070, no bitstream** (IW=2 +0.091, board PASS with the stress) on the FIRST build.
  The IW=2 census named the new worst core family `u_dcache/rd_resp_tag_reg -> u_iq_i2/e_r_reg`
  (20 levels): the D$'s registered response tag -> `o_v/o_kill[tag]` -> `ld_land` -> the
  `ld_land_idx` mux -> the load queue's `rdv/prd` arrays -> `we_ld/wa_ld` -> the schedulers'
  wake compare. Before, the landing index was a flop (`ld_inflight_idx`). Fix: `ld_land_idx`
  selects on the RAW response (`dmem_rvalid_c ? dmem_rtag_resp : ld_inflight_idx`) instead of
  on `ld_land_fast` (which goes through the per-tag `o_v`/`o_kill` lookups first), so the array
  read starts at the clock edge; exactness is asserted (a slow landing never coincides with a
  fast response, and a fast landing's tag always equals the response's). **Result on the FINAL
  tree (2026-09-18 16:55): IW=3 core WNS +0.039, IW=2 +0.048 -- both boards PASS with the
  900 s stress.** C4a step 1 is closed.

## C4a step 2 (2026-09-18/19): the D$ door self-loop -- correct on the first try, generically
## slower on the first try too, both for the SAME reason: a shared resource with no fairness rule

Built as the design notes said: `chk_rd` is true in exactly the cycle S_CHECK's own case arm
takes the "fast read delivery: skip S_FIN" branch (a plain cached read hit, not held by
`pipe_hold`), and it joins `fin_wr` in `acc_slot` -- the same admission-widening shape the
door already used for a store's last cycle (item 4b). The register-capture block (`r_*`,
`cur_line`, `phase<=0`) and the bank-address drive (`bk_rd_drv = accept | do_replay`) already
keyed off `accept`/state membership rather than a fixed S_IDLE check, so chk_rd only needed
three touch points: the admission wire itself, the capture block's condition, and the S_CHECK
arm's own `st <= S_IDLE` becoming `st <= accept ? S_CHECK : S_IDLE` (mirroring `fin_wr`'s
identical line in S_FIN). `rd_ack` is already `accept & rd_req`, so this alone flows through
for free. No RAM address, no invalidate gate, no fill-machine interaction needed touching --
`~fill_banks` and the shared `accept` qualifiers (`~inv_go`, `~f_replay`, `~f_solo`) already
apply to every acc_slot arm uniformly.

**First full-length result (2026-09-18 22:52): both widths clean at every correctness gate
(lint, 240/0, 8 benches, both storms, 7 memrand seeds) but a 300 M tiny128 boot retired
10.4% FEWER instructions than the recorded C4a-step-1 baseline at BOTH widths** (IW=2:
68,115,004 vs 76,007,711 expected; IW=3: 67,052,468 vs 74,614,701 expected) -- the tooling's
own verdict: "A correct-but-slower change." The 60 M checkpoint had looked fine (+2.58%
IW=2), so the regression is cumulative, not immediate -- ruled out flakiness by reproducing
it at both widths and by direct A/B: a throwaway worktree at 493dc343 (pre-self-loop) run
with the IDENTICAL 300 M config reproduced the recorded 76,007,711 exactly (+0.00%).

**Root cause: the self-loop has no fairness term, and the door already had a documented
priority rule that only worked because the door was never more than 50% available.**
`r_is_wr <= wr_req && !rd_req` (the register-capture mux) has ALWAYS let a live read starve
a live write when both are asserted the same cycle -- this predates C4a entirely and is
inherited from the original I$/D$ share. Before the self-loop, a read stream -- even a
continuous one, since C4a step 1's tagged path already lets the LSU re-present a pending
request every cycle via `c_rd_want` -- could still only WIN the door every other cycle (the
S_CHECK cycle was a mandatory, uncontested gap). A waiting store queue drain or PTW walk got
real, if infrequent, openings in the gaps where the read stream's OWN presentation happened
to lapse. The self-loop removes that gap: a dense read burst can now claim every single
cycle, shrinking the fraction of cycles genuinely up for grabs and pushing an
already-marginal balance measurably further from the write/PTW side -- not a hard freeze (the
lockstep never diverged, `for(;;)`-style livelock never triggered, SQ-full/door-unaccepted
counters were only modestly higher, not order-of-magnitude), just a broader tax across a long
boot that a 60 M window is too short to show.

**Fix: `chk_rd` also requires `~wr_req`.** A self-loop continuation yields the door the
instant a competing store wants it, falling back to `S_IDLE` where the existing (unmodified,
decades-tested) tie-break decides -- so a self-loop cycle can never leave a waiting write
worse off than the pre-self-loop door already could. This is deliberately NOT a new
round-robin/age-based arbiter: it is the narrowest change that provably cannot regress
fairness relative to the code that already shipped and passed every prior gate, while still
capturing the win whenever nothing else wants the door. (A PTW-vs-LSU-read priority rule
exists too, entirely at the SoC level in `rv_soc_top.v`'s `pw_ack`/`c_rd_req` arbitration,
outside `rv_cache.v` and outside this fix's reach; the fixed numbers below don't show a
residual PTW-starvation signature, so it was not chased further, but it is the next place to
look if a future window-growth increment (C5/C7) reopens this class of problem.)

**Result with the fix (2026-09-19, full re-verification against a clean rebuild): lint
clean, 240/0, all 8 benches, both 500 M storms clean, all 7 memrand seeds (both widths), the
1.5 G disk-backed cosim BLKCHECK-OK. 300 M IW=2: retires=76,913,505 vs 76,007,711 expected
(+1.19%, a genuine net win, not just recovered). 300 M IW=3: retires=74,597,923 vs 74,614,701
expected (-0.02%, neutral, within the 0.5% floor).** OOC: `rv_cache` (D$ shape: SIZE_KB=64,
WRITABLE=1, WRTHRU=0, PREFETCH=0) WNS +1.252 (worst family `cur_line_reg -> r_addr_reg/CE`,
13 levels, route-dominated); `ooo2_lsu` (LDTW=2) WNS +1.859 (worst path inside `u_mmu`,
unrelated to the self-loop). Both healthy margins at the 6.000 ns OOC period.

## C4a step 2's timing DOES NOT CLOSE at IW=3 -- NOT COMMITTED, open item

**Full builds (2026-09-19 03:05): IW=2 WNS +0.037 (met, thin); IW=3 WNS -0.270 (VIOLATED).**
Both board gates happened to PASS anyway (a timing violation does not reliably fail a boot in
one run -- it is a probability, not a certainty, and is not a basis for shipping). Per the
project's own hard rule (WNS>=0 both widths before commit), **this does not ship. wip/c4a's
tip stays 493dc343 (C4a step 1); the self-loop sits UNCOMMITTED in the worktree**, correct and
a genuine simulated win, but blocked on IW=3 timing.

**Root cause, and why OOC missed it.** `rv_cache`'s OOC (+1.252) measures the MODULE's own
internal critical path with `rd_ack` as an ordinary, lightly-loaded primary output. In the
full build `rd_ack` fans out through the SoC into `c_rd_want`/`lsu_rd_ack` and from there into
the LSU's own FSM and every scheduler that reads its results -- a small addition to `rd_ack`'s
OWN logic depth (chk_rd's `hit`/`pipe_hold` term) costs every one of those downstream
consumers the same amount, which OOC's isolated port cannot see. Census confirms the shape:
818 -> 6028 near-critical (<+0.35 ns) endpoints at IW=3; by STARTPOINT MODULE, `u_lsu` alone
went 3 -> 1280, `u_dcache/rd_resp_tag_reg` 359 -> 782, `u_rob` 0 -> 864 -- a broad congestion
increase downstream of the door, not one bad path.

**And this was foreseeable: the plan's own I8 reference says "the accept stays
register-decoded."** `chk_rd` is the FIRST time this cache's `accept`/`rd_ack` has ever
depended on `hit` -- every existing admission path (`S_IDLE`, `fin_wr`) is built entirely from
STATE and already-registered fields, decoded fast; `hit` (the tag compare, ~13-18 levels) was
until now used only INSIDE the S_CHECK case arm, gating internal register writes that are
allowed to be wrong and silently discarded (I8's original point). Wiring it into the ACK
itself breaks that property, because a requester cannot silently discard an ack: once told
"accepted," it advances and never revisits that cycle. `rd_ack` is exactly the "the compare
DECIDES, it does not ENABLE" boundary I8 warns about, and the self-loop's admission decision
is a hit-dependent enable by its very nature -- there may be no way to keep it purely
register-decoded without more structural change than "just gate it right."

**Tried and reverted (2026-09-19): making the CAPTURE (not the accept) hit-independent.**
The theory: since a capture that turns out unneeded is normally silently discarded (matches
S_IDLE's own documented property), maybe only the capture block's wide, ~150-bit `chk_rd`
gate -- not `accept`/`rd_ack` itself -- was the actual congestion source, and it could be
replaced by a cheap `chk_cap` built from state + already-registered fields alone (dropping
`hit`/`pipe_hold`). WRONG: `pipe_hold`'s hold cases (the banks weren't actually addressed
last cycle, F_ANS is busy, the one MSHR is occupied, a write-hit-victim collision) all mean
the FSM is RETRYING THE SAME REQUEST next cycle and r_addr/r_tag/cur_line/etc. must survive
unchanged -- a hit-independent capture clobbers them regardless of which case it is, and
`ooo2_lsu.v:555`'s own B-rule assertion ("fast response with tag N that nothing is waiting
on") caught it on the very first cosim run. Reverted in full; `chk_rd` is the sole admission
wire again, exactly as the working version above. OOC for `chk_cap` alone (+1.206, barely
different from the un-split version) suggests the capture's own width was NOT actually the
dominant congestion source anyway -- consistent with the `rd_ack`-fanout theory above.

**What the boot numbers say, and what they do NOT say.** The self-loop measures:

| 300 M boot | IW=2 | IW=3 |
|---|---:|---:|
| C4a step 1 (baseline) | 76,007,711 | 74,614,701 |
| + the self-loop | 76,913,505 (**+1.19%**) | 74,597,923 (**−0.02%**) |

with the 60 M checkpoints agreeing (+2.6% / +0.11%). **This is NOT a verdict on the
self-loop.** It is a statement about the machine that exists TODAY: with ONE AGU port only
one address is generated per cycle, so the LSU physically cannot present enough demand to
saturate a 1-per-cycle door, and C4a step 1 already collected what the current port count can
reach. In the END STATE this program is building (§ the plan: 3 ALU/AGU ports, C4b then C7),
loads issue from each of three ALUs, so cycles with several loads waiting to execute are
routine and a door that accepts once every two cycles is a hard ceiling on all three. The
self-loop is a PREREQUISITE for that machine, not an optimization of this one -- do not
retire it because today's single AGU cannot feel it. (Tommy, 2026-09-19, correcting exactly
this mistake in an earlier draft of this section.)

So: the increment continues. What actually blocks it is the timing structure above --
`rd_ack` must stop depending on `hit` -- and that has a known fix.

**THE FIX: a one-deep SKID BUFFER at the door.** (Recorded first, because the obvious "just
add a delivery state" idea does NOT work: accept -> S_CHECK -> a new one-cycle S_RDDELIV with
the door open there would still be one accept every two cycles -- the same cadence as today,
for an extra cycle of latency. Worthless; do not build it.) The skid-buffer shape instead:
- The door accepts during S_CHECK on the CHEAP, register-decoded condition alone (`chk_cap`
  above: state + already-registered request fields + `wr_req`, no `hit`), so `rd_ack` never
  sees the tag compare and I8 holds.
- The accepted request lands in a SECOND request register set `n_*` (a one-deep skid slot),
  NOT in `r_*` -- which is exactly what makes it safe where the reverted attempt was not:
  `r_*` is never clobbered, so `pipe_hold`'s retry cases keep their live request.
- `~n_v` joins the accept condition (a register, cheap): the slot holds at most one.
- When the pipeline frees (a self-looped hit, or a return to S_IDLE), it takes from `n_*`
  before the door -- older wins, the same priority shape `f_replay` already uses.
- The bank read address mux gains `n_addr` as a source, selected by `n_v` -- a REGISTER, so
  rule I6 (nothing late on a BRAM address) still holds.
Cost: ~220 bits of duplicate request registers plus that mux. Cheap on this part, and it is
the shape that makes the accept honest.

**BUILT (2026-09-19).** `n_v`/`n_*` in `rv_cache.v`, and the door's one question split into
two:
- `door_take` -- "I took your request" -- is `door_ok & ~n_v & ~fill_banks & ~inv_go &
  ~inv_busy & ~fin_hazard & ~f_replay & ~f_solo & (rd_req | req_wr) & ~(req_solo & f_v)`,
  where `door_ok = (st == S_IDLE) | fin_wr | (slot_ok & ~req_solo)`. Every term is a state
  bit, an already-registered field or a module input: **`hit` and `pipe_hold` appear
  nowhere**, so `rd_ack` is register-decoded end to end, which was the whole point.
- `do_pull` -- "I am starting your lookup" -- is `pipe_free & ~f_replay & (n_v | door_take)`
  with `pipe_free = ((st == S_IDLE) | fin_wr | pull_hit) & ...`, and `pull_hit` is the
  self-loop (`slot_ok & ~pipe_hold & hit`). This one may be late: it drives only `r_*`,
  `cur_line`, `st`, `n_v`, `b_live` and the bank address -- all inside the module.
`do_pull` replaced `accept` at every one of its old sites (`bk_rd_drv`, `b_live`, the capture
enable, the three `st <=` arms); `door_take` replaced it at the ones that are really about
what came IN (`wr_acc`, the chunk-alignment/solo/PAW_SIG assertions, the perf stamp).

The BYPASS is what keeps a streaming hit at today's latency: in the cycle S_CHECK resolves a
hit, the banks are already being addressed from the live door address (`a_live`'s new
`n_v ? n_addr : door` select), `door_take` and `do_pull` both fire, `r_*` loads straight from
the door and `n_v` stays 0 -- the slot is skipped entirely. The slot only fills when a
request is taken in a cycle the pipeline does NOT free up (a miss, a hold), which is exactly
the case where the old door would have made the requester wait at its own port instead.

Four new always-on assertions state the slot's hazards: a second request taken onto a full
slot (the first would vanish, and the requester cannot see that -- it already advanced on the
ack), a pull with nothing at the door and nothing in the slot, a SOLO request found in the
slot (they are taken only at S_IDLE/fin_wr, which is what lets the pull re-check nothing
about them), and a pull overtaking an owed fill replay.

**THE GATE SET CHANGED HERE: IW=3 is POR, IW=2 is dropped completely** (Tommy,
2026-09-19). The both-widths rule existed because IW=2 was the SHIPPING width while IW=3
could not close; that inverted on 2026-09-17 when IW=3 closed board-clean. Gate each
increment on IW=3 alone -- one `make bit`, one board gate. IW=2 cosims are optional and never
block. The IW=2 build+gate was the dominant wall-clock cost of every increment (~40 min +
~20 min) and bought only timing coverage for a configuration we do not ship; the
width-parameterisation bugs it was credited with are functional and come from cosims.

**And the bitstream that gate reads must be fresh -- see rule G11.** This increment is where
`build.tcl`'s "Bitstream already up to date, skipping" was caught lying: the routed
checkpoint was 10:32 (the skid buffer) and the `.bit` beside it was 02:32, from a DIFFERENT
build that had MISSED timing at -0.270. The timing report and the bitstream disagreed, and a
board gate cannot tell you that. Guard fixed to compare mtimes against the routed checkpoint.

**T21 in `tb_ooo2_dcache.v` is the regression test this increment was missing.** It warms 32
lines, then streams 32 read hits with a requester that presents the next address in the same
edge the door took the last one, and measures cycles per accepted read: an S_IDLE-only door
is pinned at 2.00 whatever the requester does, a self-looping one approaches 1.00. It fails
under 32 acks or over 48 cycles. This matters because a BOOT retire count cannot see the
door at one AGU port -- both designs sit at ~10% door utilisation there (5.8-5.9 M accesses
in 60 M cycles), which is precisely why the end-state argument above, not a boot delta, is
what justifies the increment.

**Aside, ruled out and not chased:** `workloads/ldbench` hangs identically (pc~0x20,
retires=61,904,704 at the 200 M cutoff, no console text at all) on BOTH this tree and the
untouched 493dc343 baseline via `FW=ldbench.bin ../../ooo2/run-ooo2-linux.sh` -- a
pre-existing harness/invocation issue (mtvec is never programmed in `ldbench`'s crt0.s; a
guess is some other trap reaches an unmapped address 0x0-ish and free-loops), not a self-loop
defect. ldbench's own numbers in the C4a design notes above must have come from a different
invocation than the one tried here; re-deriving the right one is a follow-up, not a blocker
(every OTHER measurement -- the 300 M retire counts, the OOC margins -- already answers "did
the self-loop help" without it).

**Rule candidate (I13): a shared door/arbiter that grants one class priority over another
needs that priority re-examined every time the winning class's access rate changes** -- not
just when the LOSING class's request rate changes, which is the usual thing a code review
checks for. `r_is_wr <= wr_req && !rd_req` was reviewed and accepted when reads could only
ever claim 50% of cycles; doubling that ceiling (the self-loop) needed the SAME rule
revisited even though nothing about writes changed at all.

## Future consideration (Tommy, 2026-09-19, deferred until everything else here is done)

**CTF as ALU + occasional side effect, on the ALU ports.** (Tommy's end-state strawman,
2026-09-19: `LSA (Load,Store,Atomic) + ALU/CTF + ALU/CTF + FP/MULDIV/CSR/fence`. Note this
SUPERSEDES the plan's C4b/C7 "AGU on the ALU ports": loads stay in a dedicated pipe, because
an ALU scheduler entry that produces no result of its own -- `e_prd = 0`, excluded from the
FIXEDL wakeup matrix, dependents woken from the SH_LD landing instead -- is a special case in
the most timing-critical structure there is, paid for only to buy address bandwidth. Stores
stay there too: they are not ALU-shaped (no register result) and their one ALU-ish part, the
address add, is already bought off by the memory-dependence speculation, which has loads
issue past unknown-address stores and replay on violation. One LSA pipe also means at most
one address per cycle, which is exactly what the 1-per-cycle door above serves.) Today's control-transfer
resolution (`u_xf`, mirroring M one-for-one, fed from `j_*`/`iss_c`) is ONE pipe: `u_iq_c` is
in-order, so exactly one branch/jump resolves per cycle and the "oldest wins" interlock on
`fr_v` is free (there is only ever one). Tommy's idea: treat a CTF op as an ALU op with an
occasional side effect (the link write, the redirect) and let it execute on either
ALU port instead of the single shared CTF pipe.

**THE LINK VALUE IS ALREADY THERE, AND IT IS NOT PC+4.** `src/exec_alu.v:42` computes
`next_pc = pc + (is_rvc ? 2 : 4)` and `result = res_link ? next_pc : alu_r`, and the ALU
ports ALREADY wire it: `u_xa` takes `.res_link(qa_res_link) .is_rvc(qa_rvc) .pc(qa_pc)` (the
payload carries the PC for AUIPC regardless). `ooo2_exec` is the same module the CTF pipe
instantiates as `u_xf`. So there is no payload widening, no new adder, and no `cf_link_wb`
arm on SH_FE -- the link just uses the ALU's own write port. What has to move is only the
comparator/target side (`xf_redirect`/`xf_taken`/`xf_taken_tgt`) and the resolution
bookkeeping. Carry the `is_rvc` term explicitly: C.JALR links x1 = PC+2 (and C.JAL likewise
on RV32; on RV64 that encoding is C.ADDIW), so a re-derived "PC+4" would be a silent wrong
link value that only a lockstep divergence would eventually expose. (Tommy caught exactly
this in an earlier draft of this note.)

**The costs, and how big each really is:**
- **Multi-redirect is smaller than it looks.** The SQUASH already fires at ROB head, so two
  squashes cannot collide -- oldest-wins is implicit there. What needs a real age compare is
  the EARLY frontend restart (`fr_set`/`fr_v`), which today gets oldest-wins for free from
  `u_iq_c` being in-order. One compare, on a non-head path -- and C6 (mid-window rollback)
  builds exactly that machinery, so doing this AFTER C6 is much cheaper than before it.
- **Predictor/BTB TRAINING doubles to 2/cycle** (mispredict redirects stay <=1/cycle at head;
  per-branch counter training does not). **Just buffer it** (Tommy): two same-cycle
  resolutions are rare, so a 1-deep skid absorbs them and drains at 1/cycle. A buffer is the
  RIGHT answer and not merely the cheap one because a training update is DROPPABLE -- it
  feeds a hint structure, so an overflow may discard rather than stall, and a cycle or two of
  staleness costs essentially nothing in accuracy.
- **Slot contention** is modest: ~0.3 branches/cycle at IW=3 IPC against two ALU ports.

**The real prize is not bandwidth.** One CTF pipe already has ~3x headroom on the average
branch rate. The win is that `u_iq_c` is IN-ORDER today, so a branch whose operands are ready
waits behind an older branch whose are not, and every cycle of delayed resolution is
wrong-path work; resolving out-of-order on the ALU ports cuts mispredict LATENCY. A bonus:
taking CTF off the shared F queue drops it from four drains (`iss_f`/`iss_c`/`iss_md`/
`iss_sys`) to three, and that queue is on the critical path -- the CTF-on-FP-port change was
part of the -0.528 plateau -- so this may RECOVER timing rather than cost it. Not started;
sequence it after C6.

**WHICH SIGNALS MAY BE LATE, WRONG OR LOST -- the taxonomy C4a step 2 paid to learn.**
`rd_ack` may be neither late nor lossy: a requester acts on it irrevocably (it advances and
never revisits), which is why `hit` in its cone cost IW=3 its closure. An `r_*` capture may
be WRONG provided it is overwritten before anything reads it (I8's original point, and what
S_IDLE's own capture has always relied on) -- but NOT while a `pipe_hold` retry still needs
it. Predictor training may be late AND dropped outright. Classify a new signal into one of
those three before deciding what is allowed to gate it.

