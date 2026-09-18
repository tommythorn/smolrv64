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
