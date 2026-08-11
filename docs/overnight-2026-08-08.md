# Overnight report 2026-08-08 (autonomous session)

TT asked for IPC numbers on real workloads and general perf findings. That
surfaced two distinct core-hang bugs first; one is fixed and validated, the
other is now reproduced in simulation with a captured wedge state.

## Fixed and validated

**virtio-net IRQ livelock (262d46c)** — the backend ignored
VRING_AVAIL_F_NO_INTERRUPT, so every RX delivery re-raised the level IRQ
*during* NAPI's poll; under NFS-root's sustained RX the core re-entered the
handler forever. This killed two boots (mid-service-storm) and TT's session.
Fix: capture avail-flags bit 0 on every avail read, gate the interrupt.
Validated on HW: full boot through the previously-fatal window, then 3x cold
70MB NFS reads (15 min sustained load) with irq/deliver counters sane and
RX still 0 bad frames.

## Reproduced, not yet fixed

**PMU-enable hard wedge** — `sudo perf stat` (TT) and my `ipcstat` (same
perf_event_open path) hard-hang the whole SoC at counter enable. 2x on HW.
Sim reproduction hunt (7 attempts, tooling now reusable): bare-metal CSR
replay passes; self-attach and child-attach counters on TT's exact kernel
pass idle; timer-IRQ storm passes; **counters + virtio-blk IRQ/DMA + timer
storm dies in one iteration** at c~587M (VBLK ceases, console dead, fetch
wandering, zero commits, one shard's store buffer full of never-committed
stores, S-mode with SIE=0 and SEIP+STIP pending). Everything needed to dig
(recipe, DTBs, initramfs auto-run poker, onset cycle) is in
memory/project_pmu_enable_wedge.md; next step is the +ckpt/wave dig at onset.

Until this is fixed, any perf-counter use wedges the board — IPC measurement
via perf is blocked (that's why there are no IPC numbers yet).

## Measurements in hand

- NFS-root cold-read throughput: **317 kB/s** (boot-churn) / **~350 kB/s**
  (quiesced, server-cache warm). Limiter identified: eth_rx_engine is
  single-buffered — a 1MB NFSv4 READ reply arrives as ~700 back-to-back
  wire frames, most drop, TCP lives in retransmit. Not a hang, pure perf.
- DDR HPM re-confirmed under disk-heavy load: rd 13.40 cyc mean / wr 8.69,
  95.9% of reads in 8-15 cycles (matches the f609672 sim model within 1%).

## Prioritized perf backlog (once the PMU wedge is fixed)

1. **RX multi-frame buffering** (BRAM ring in eth_rx_engine) — the NFS
   throughput limiter; stopgap: rsize=32768 in nfsroot opts.
2. **CPI redirect fix** (docs/pipelining-findings.md, root cause pinned) and
   **BP phase 1 (YAGS)** — the known IPC levers; I$ supply is the top stall.
3. ipcstat/perf on real workloads for the actual IPC numbers TT wants.

## Housekeeping

- Fresh SD card survived everything; the CRC-checked read path (e73b176)
  showed zero wire transients across all boots (dbg_crc_err clear).
- NFS export got: LOGIN_TIMEOUT 300, motd scripts disabled,
  DefaultTimeoutStartSec=300s (udevd now has runway; getty appears unaided).
- snapd masked (useless at 67 MHz, was blocking multi-user).
- Fixed-MAC for virtio-net still TODO (DHCP address changes every boot).
- Board is currently wedged from the last ipcstat trigger — reprogram+boot
  gives a healthy NFS system; just don't run perf on it yet.
