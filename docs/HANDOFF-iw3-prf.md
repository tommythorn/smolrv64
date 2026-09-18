# IW=3 timing handoff — the PRF read-port lever (next session)

Written 2026-09-16 (session `012LAhVdXKrjj2AKMZXy7CBx`). Companion to the memory
`ctf-on-fp-port.md` (the full running record). Read this to pick up the IW=3 timing work.

## Where things stand

- **Branch** `wip/iw3-timing`, worktree `/home/tommy/smolrv64-wt-stage1`.
  - HEAD **`dd371e6d`** — *dispatch swizzle + dispatch stage* (this session).
  - Base `4ee73d23` — flop-release + BTB/YAGS-2048.
- **IW=3/HW=8 WNS = −0.528 ns** at probe_clk 6.000 (166.667 MHz). VERIFIED: riscv-tests
  240/0, 60M Linux lockstep cosim clean (13M retirements, zero divergence), Vivado build.
- **IPC**: the dispatch stage costs **−2.1%** on the tiny128 boot (a cycle boundary; the
  +1 dispatch→issue latency is exposed only when a scheduler drains, which the memory-bound
  boot does a lot; hidden in steady-state high-IPC code).

## What has been RULED OUT (don't repeat these)

The −0.528 wall is a **broad route-congestion plateau** — ~6 families all at ≈−0.52, every
one **70–81 % route** (fetch `pc_q` loop, fetch BTB read, rename `t_ld` fanout, SQ `kcc` →
scheduler `e_r`, CSR → `e_r`, `m_addr` → minstret). No single point-fix moves WNS.

- **2-stage BTB / micro-BTB — MEASURED NON-LEVER.** A throwaway probe (read the BTB at
  `bidx(base_pc)`, a register slice, vs the 24-level `rd_pc` cone) removed the `pc_q→BTB`
  path entirely — but WNS did not improve, because the fetch `pc_q→pc_q` next-PC loop sits
  at −0.523, one thousandth behind. The days-long 2-stage-fetch rearchitecture would buy
  ~0.005 ns. Do not build it. (The micro-BTB shadow was written + verified bit-identical,
  then reverted — not committed.)
- **Placement — MEASURED EXHAUSTED.** Every constraint is worse than the tool's default:
  - `AltSpreadLogic_medium` (unconstrained): **−0.528** ← optimum
  - pblock `core → CLOCKREGION_X1Y1:X3Y3`: −0.888
  - `PLACE_DIRECTIVE=WLDrivenBlockPlacement` (aggressive pack): −1.174

  The core spreads over all 16 clock regions (`fe` over 13) *to relieve local congestion*;
  bounding/packing re-congests and lengthens routes. Placement can only rearrange the
  congestion, not reduce it. Do not pursue placement.

**Therefore:** closing −0.528 requires **reducing the near-critical logic amount**, which is
the PRF read-port pressure (`ps_out→ALU` + scheduler-`e_r` route cluster). This is
[[scheduler-is-the-critical-path]] applied at IW=3.

## The lever: PRF read-port pressure

The unified PRF is **~10R/4W** at IW=3 (the swizzle dropped the 3rd ALU + SH_IE3 shard):
reads = M rs1/2/3, ALUa rs1/2, ALUb rs1/2, F rs1/2/3; writes = ie, ld, fe, ie2. Every read
muxes all shards → the route-bound cluster. Two directions (Tommy chooses; the rule
**"never trade integer for FP"** applies):

1. **ALU read-register** (targeted, measurable — the shape that worked for the dispatch
   stage). Register the shard read *before* the ALU, splitting `ps_out→ALU` into two stages.
   Cost: +1 ALU latency; add a bypass for back-to-back dependent ALU ops to limit the IPC
   hit. Dodges the branch penalty (branches issue on F/CTF, not the ALUs). Prototype it and
   measure WNS gain vs IPC cost — one build tells you if it's the lever.

2. **INT/FP PRF split** (fundamental, multi-day). FP/CTF gets its own small PRF; the main
   PRF loses the F rs1/2/3 reads. Open design questions: FCVT/FMV cross-PRF ports; the
   FP-load write path from M into the FP PRF; FP-store read of the FP PRF. This **reverts
   CTF-on-FP's +2.75 % boot IPC**, so weigh it against direction 1.

## Caveat worth settling FIRST

**IW=3's IPC win is unproven.** On the tiny128 boot it is **−11 % vs IW=2** — but that is
backend/memory-bound, not a width failure: the store-buffer taxonomy shows the store queue
full ~38 % of cycles and the D$ door unaccepted 26M times (store-drain + load-latency bound;
boot is store-heavy memset/memcpy). IW=3 has **never been measured on a compute-bound
workload** where 3-wide should actually pay. Before trading *more* IPC for timing, consider
measuring IW=3 vs IW=2 on Geekbench / an ALU-dense micro-bench. If 3-wide doesn't win IPC
even there, the IW=3 timing push is moot and the real boot-IPC lever is the **D$/store path**
(the "plan item 4c" axis), independent of width.

## How to work here

- Gates: `ooo2/run-ooo2-vl.sh` (pass=240 fail=0); `CYC=60000000 VDEFS='-DOOO2_IW=3
  -DOOO2_HW=8' ooo2/run-ooo2-cosim-linux.sh` (retire-lockstep). `src/lint.sh` is red on a
  **pre-existing** UNOPTFLAT (`m_done_red`) fatal under Verilator 5.041 — verify via
  sim/Vivado, not lint.
- Vivado: from `platforms/rk-xcku5p-f-v1.2`, `OOO2_IW=3 OOO2_HW=8 make reset && make bit`;
  then `make timing` / `make census` / `make place-report` read the existing checkpoint (no
  rebuild). **No Verilator while Vivado builds.** Long jobs in tmux (the harness culls
  background waiters under memory pressure; tmux jobs survive).
- Always pass `VDEFS='-DOOO2_IW=3 -DOOO2_HW=8'` for IW=3 cosims (default is IW=2).
- Git in the worktree: `third_party/cvfpu` is a **symlink** — `mv` it aside for any git op,
  `mv` it back for sims. Stage EXPLICIT paths only, never `git add -A/-u`. Commit only
  outside Mon–Fri 09:00–17:00 PDT and only when Tommy asks; exclude `rk_xcku5p.xpr`.
- Cheap-probe lesson: before a large rearchitecture, run a correctness-broken **timing
  probe** (like the BTB read-address probe, or kill-ALU) to prove the lever moves WNS first.

## Key files

- `ooo2/ooo2_prf.v` — the PRF and its read ports.
- `ooo2/ooo2_core.v` — `ps_out_a/ps_out_a2/ps_out_f` (PRF read outputs) → ALU/F execute; the
  schedulers `u_iq_i/i2/l/f`; the `wkv`/`wkp` writeback broadcast; the new dispatch stage
  (`stg_*`, `mv_*`) and the swizzle (`c_to_*`, `l_slot0`, `f_slot0`).
- `platforms/rk-xcku5p-f-v1.2/report_place.tcl` (`make place-report`) — clock-region spread.
