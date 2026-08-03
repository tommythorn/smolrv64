# Pipelining — Verified Findings (start here)

Last updated 2026-06-17. Authoritative orientation for anyone (human or AI)
picking up smolrv64 pipelining. Read this **before** the longer plans. It exists
to stop re-deriving what is already known and to stop retrying known dead ends.

## TL;DR

- The core is a single ~9.5k-line FSM module (`src/smolrv64.v`, module
  `smolrv64` ~L341): ~40 live states, **196 `state <=` sites**, plus sub-FSMs
  (cache, mem-engine, frontend). The control *is* the architecture.
- **Do not rewrite from scratch.** The datapath is already lifted into
  stage-shaped pieces: `execute_req_*` payload registers, a parameterized
  `EXOP_*` ALU split across `S_EXECUTE/S_EXECUTE2`, a unified load/store/AMO
  address adder, a separate `smolrv64_frontend` module, and an 8-entry
  `rf_decode` predecode FIFO. A clean sheet discards debugged corner cases and
  the cosim oracle's value, and maximizes unverified surface exactly when
  verification is the binding constraint.

## Observed failure signal (be precise about this)

The only reliable signal we have ever had from a broken pipelining change is
**"userspace apps crash that shouldn't."** The underlying mechanism is
**unconfirmed** — do NOT cite "dropped/skipped instruction" as the failure mode.
That was a *hypothesis*, and a provable property of one specific reverted guard
(see content-dedup below), not a demonstrated general cause. State the symptom
(apps crash), not a guessed mechanism.

## CPI root cause (PINNED — do not re-derive)

This is the *performance* root cause (CPI), separate from the correctness signal
above. Full blow-by-blow is in the memory note `project-cpi-breakdown`.

- **The frontend already runs ahead.** `rf_decode` reaches full depth (4) during
  RF/EXECUTE. "Make the frontend run ahead" is NOT the task — it already does.
  (Confirmed by TT 2026-06-17.)
- **The backend discards that lead on every single retire** via a perpetual
  redirect. `retire_prepared_fetch` takes the redirect path because
  `rf_decode_matches_retire` fails: the queue HEAD is a stale duplicate of the
  redirect-target instruction, which the frontend enqueued TWICE (racing enqueue
  sites: hit-path `F_FETCH_BUF_USE`, backend `accept_instruction_fetch`,
  speculative prefetch, pending-slot drain). One redirect seeds a self-sustaining
  redirect→squash→refetch→duplicate loop. That loop **is** the ~9.7 sim CPI (and
  most of 14–17 FPGA CPI; `dcache_miss≈0`, so DRAM is secondary).

## Dead ends — DO NOT RETRY

- **Content-dedup** (drop an enqueue whose `(pc,epoch)` == previous): collapsed
  sim CPI 9.70→3.67 (~2.6× IPC) and passed riscv-tests 240, but is **unsound by
  construction** — dedup-by-content on a 2-bit epoch that wraps every 4 redirects
  can discard a *legitimate* enqueue when the epoch aliases. Observed as
  tinymembench and FPGA Ubuntu crashing. ALSO failed FPGA timing (WNS −0.665: the
  8×64-bit array mux + 64-bit compare on the enqueue path). Two variants tried;
  both reverted. Dead.
- **Consumed-hit latch** (`f_consumed_hit_q`, bounce the backend re-enqueue):
  partial sim win (9.70→5.67), riscv-tests 240, but crashed FPGA linux boot (a
  re-latch race). Reverted.
- These passed riscv-tests + boot-to-initramfs and still broke real workloads —
  see the validation problem below.

## Correct fix direction (not yet done)

Two viable paths, both real multi-step work:

1. **Source-suppress the redirect-induced double-enqueue** at the seam — never by
   content. The dup is the redirect target enqueued by two racing sites on
   consecutive cycles; existing `f_consumed_hit` only covers the same-cycle case.
   Needs the exact site-pair pinned (tagged enqueue trace) plus a 1-bit
   cross-cycle guard.
2. **Ownership cutover** (the old "Plan C"): after a redirect/retire-fetch, the
   backend must NOT run its own `FETCH_REQ→BUF_CHECK→BUF_USE→accept` fetch path
   that races the frontend; it lands in a wait state and only consumes
   `rf_decode`, acting on the frontend's explicit PUNT (page-boundary) or MISS.
   Removes the race by construction. Bigger restructuring; where prior attempts
   got bitten.

## THE real blocker: no root-causable reproduction

This, not the RTL strategy, is why a month produced nothing.

- **The failures only show up under heavy real code, and the only gate that
  reliably trips them is a full Ubuntu boot.** riscv-tests (240),
  boot-to-initramfs, and `workloads/linux` all **under-exercise** and pass while
  the bug is present (TT: `workloads/linux` doesn't exercise enough code).
- **Ubuntu boot is currently FPGA-only.** `workloads/ubuntu/ubuntu-boot.sh`
  serial-uploads `fw_payload.bin` + `ubuntu.dtb` + initrd into the board monitor;
  the rootfs lives on the SD card and is reached via virtio-blk. FPGA gives a
  pass/fail but **zero visibility** — you cannot root-cause a crash on the board,
  and each attempt costs an hours-long bitstream (timing is always tight, below).
- **The fix is not a faster proxy workload — it is making the real signal
  root-causable.** Direction (TT 2026-06-17): **run the Ubuntu boot in
  simulation.** Until something better exists, Ubuntu boot is the best signal we
  have, so bring it into a simulator where it can be observed.

### What "Ubuntu in sim" requires (grounded 2026-06-17)

- **The sim testbench has NO virtio today.** `grep virtio_blk|virtio_net` over
  `sim_main.cpp` and `smolrv64.v` is empty — the devices exist only in the FPGA
  top (`platforms/rk-xcku5p-f-v1.2/...rk_xcku5p.v`). The Verilator TB currently
  models only core + UART/CLINT/PLIC + memory.
- **The device pieces already exist and are independently sim-tested:**
  `virtio_blk.v`, `virtio_mmio.v`, `sd_spi_host.v`, and crucially
  `sd_spi_card_model.h` (a simulation model of the SD card), each with a
  standalone `*_tb.cpp`. The FPGA top shows the exact wiring (selects, IRQ,
  arbiter). So step 1 is integration of known-good blocks, not new RTL.
- **Three sub-problems, in order of leverage:**
  1. *Wire it up:* instantiate virtio-blk + the SD card sim model in the core TB,
     backed by a disk-image file (the SD contents); load fw_payload/dtb/initrd at
     the same addresses `ubuntu-boot.sh` uses; size memory to match the DTB.
  2. *Make it fast enough:* full Ubuntu boot in Verilator is slow (tiny128 sim
     boot is already >30 min). Levers: Verilator `--threads`/`-O3`; and
     **snapshot/replay** — boot once to just before the failure, save state, then
     iterate fixes over the last window instead of re-booting from zero.
  3. *Pinpoint, don't just observe:* ideally run lockstep against simmerv
     (cosim), so divergence flags the bug at its first wrong retirement instead of
     a downstream app crash. Hard part: keeping simmerv in sync through virtio
     DMA — extend the existing DUT→simmerv MMIO-load override to the
     device-touched memory rather than modeling virtio in simmerv.
- **Cheapest first confirmation:** wire the existing device blocks into the TB,
  start an Ubuntu boot in plain sim, and measure cycles/sec. That single number
  says whether this is a days job or a weeks job and is a prerequisite for every
  route above. No new RTL.

## Working rules for the next attempt

- **Name the invariant first.** A change is a hypothesis (e.g. "enabling overlap
  X preserves: every committed PC reaches execute exactly once across
  redirect/epoch-wrap; `execute_req` payload stable while `valid && !accepted`").
  Then a failure points at a violated invariant instead of forcing a bare revert.
- **No bare reverts.** A revert without a root cause is a deferred recurrence —
  proven here over 8 days. The `project-cpi-breakdown` diagnosis is the quality
  bar; the three bare `git revert`s from that window are the anti-pattern.
- **Size increments to carry an invariant.** One-line "launch earlier" changes
  can't host one; "backend stops racing the frontend fetch path" can.
- **Don't trust green from weak gates.** riscv-tests 240 / initramfs /
  `workloads/linux` are all necessary-not-sufficient; they passed while the dead
  ends above were broken. The real gate is Ubuntu boot — get it into sim.
- **Timing is always tight** (typically WNS ~0.01 ns — TT). Treat any added logic
  as timing-risky and budget a P&R/timing pass; don't blame functional commits
  for P&R-noise timing failures in unrelated paths.

## Tooling

- `tools/perf-cpi.sh` — CPI decomposition on live FPGA Ubuntu (`BUS_WAIT_CYCLE`
  HPM event = true memory-stall meter; structural CPI = (cycles−bus_wait)/instret).
- `+state_summary` / `+state_summary_interval=N` plusargs — per-state cycle
  histogram (sim-faithful for the cache-HIT / structural portion).
- Cosim oracle: `make -C src smolrv64-linux-cosim` / `smolrv64-tiny128-cosim`
  (lockstep vs simmerv, aborts on divergence). No Ubuntu/virtio variant yet.
- Diagnostic method that worked: temporary `$display` INSIDE the decision tasks
  (block B), gated by `+state_summary`. Do NOT trust npc/fetch_epoch/rf_count
  reads from the separate SIMULATE always block (~L9511) — they lag one cycle.

## Doc status (2026-06-17)

- This file is the orientation. The longer narrative is `docs/PIPELINING.md` (still
  directionally valid: independent frontend + retiring backend).
- DELETED as superseded: `PIPELINING_PLAN_C.md` (its stage-contract +
  no-bare-overlap methodology is folded into "Working rules" above).
- `docs/frontend-plan.md` is a refactor sketch (frontend-boundary extraction),
  not the blocker — secondary.
- The `MEMORY.md` one-liners for CPI and timing were corrected today; older
  versions said "frontend never runs ahead," which is WRONG — see
  `project-cpi-breakdown` for the truth.
