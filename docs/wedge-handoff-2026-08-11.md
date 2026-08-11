# Wedge handoff — 2026-08-11

Board is yours. Everything below is current as of handoff; I've stopped my
watchers. The coffee-resident monitor keeps writing its verdict to a file you
can check anytime (see "Soak").

## Board state right now

- Bit: **4db98a0** (all three wedge fixes) + `ILA_TIMER=1` + `PROBE_CLK_DIV=6`
  (55.6 MHz — chosen for ILA timing closure; a /5 production bit needs a plain
  `make bit` rebuild once you trust the soak).
- Booted NFS-root at **192.168.1.151**, up since ~06:50, alive at handoff —
  already past the 25–60 min window that killed the two previous bits.
- Soak verdict file (written by a detached loop on coffee, survives ssh):
  `cat /tmp/wedgewatch.txt`  → `BOOTED 192.168.1.151` now; flips to
  `WEDGED <ip> <time>` if it dies. Loop pings every 60 s.
- **If it wedges**: the ILA_TIMER probes are in this bit. Immediately:
  `cd ~/smolrv64/platforms/rk-xcku5p-f-v1.2 && make ila-capture` — probe7
  (`probe_wedge`) carries the frontend/dispatch/interrupt gate state
  (bit meanings at its pack site in backend_top.v ~line 1655), probe2 = pc.
  That capture names the stuck gate directly.

## The three fixes (all on main, coffee synced at 4db98a0)

1. **262d46c — virtio-net ignored VRING_AVAIL_F_NO_INTERRUPT.** Per-frame IRQ
   re-assertion during NAPI poll = full-system livelock under sustained RX
   (NFS). Validated on HW: boot through the formerly fatal service storm +
   15 min sustained cold reads, RX clean (0 bad frames).
2. **8585044 — mip/sip csrrs/csrrc latched the hw-OR'd read.** An M-mode RMW
   of mip (OpenSBI SBI-PMU path) while a device line was high latched
   transient SEIP into the register → spurious-interrupt storm (claim reads
   0 forever). This was the `perf stat`/ipcstat kill. Deterministic sim repro
   died at c=587,022,357 twice; fixed RTL passes that cycle and completes
   10/10 poker iterations under blk+timer load.
3. **4db98a0 — orphaned OP_IRQ left inject_inflight latched.** An injected
   interrupt pseudo-op can commit without its trap (the CPR commit-count
   orphan, cosim-proven at retire 84.17M). No rollback comes, csr_irq_v
   can't fall, all future injection vetoed → machine freezes at next wfi.
   HW-proven by the ILA_IRQ capture (seip=1, best_irq=12, zero PLIC claims).
   Fix = 64k-cycle watchdog re-arm. Validated: 215/215 + Ubuntu cosim to
   **216M retirements / 500M cycles, zero divergence** (2.6× past the orphan).
   Proper fix later: commit-side orphan detection (see the code comment).

## Open items, in order

1. **Soak verdict** — just leave it running and check the file. Survival
   through today ≈ wedge closed.
2. **Login/boot stall (blocks ssh + getty, NOT a core bug).** Multi-user
   never completes: some "no limit" systemd job stalls; `/run/nologin` then
   blocks all logins (ssh key is accepted then denied by pam_nologin —
   that's the "Permission denied" you see despite the key being right).
   udev-trigger is already masked (helped once, insufficient). The
   instrumentation that names the culprit without any console racing: a
   self-reporting unit in the export — as root on coffee:

   ```
   sudo tee /srv/ubuntu-root/etc/systemd/system/bootstate.service > /dev/null <<'EOF'
   [Unit]
   Description=boot-state reporter
   DefaultDependencies=no
   After=local-fs.target
   [Service]
   ExecStart=/bin/sh -c 'while true; do { date; systemctl is-system-running; echo ---; systemctl list-jobs --no-pager; } > /home/ubuntu/bootstate.txt.tmp 2>&1; mv /home/ubuntu/bootstate.txt.tmp /home/ubuntu/bootstate.txt; sleep 60; done'
   [Install]
   WantedBy=sysinit.target
   EOF
   sudo mkdir -p /srv/ubuntu-root/etc/systemd/system/sysinit.target.wants
   sudo ln -sf ../bootstate.service /srv/ubuntu-root/etc/systemd/system/sysinit.target.wants/bootstate.service
   ```

   Next boot: `cat /srv/ubuntu-root/home/ubuntu/bootstate.txt` from coffee
   shows the stuck job list; mask the named unit; logins work forever after.
   (Alternative without sudo: the board's own rescue shell is root over the
   no_root_squash export and can write the same files —
   `DTB=ubuntu-nfs-rescue.dtb ./ubuntu-boot.sh <screen-session>`.)
3. **Measurements (everything staged in /srv/ubuntu-root/home/ubuntu/):**
   - `./pmupoke` — freestanding perf trigger; stage markers S1..S12; used to
     be a guaranteed kill, now safe (fix #2/#3).
   - `sudo perf stat sha256sum /boot/vmlinuz` — the original crime scene.
   - `./ipcstat <cmd>` — cycles/instret/IPC per command (needs the fixes; it
     was rebuilt dynamic, ungrouped, no exclude_hv — all three were needed).
   - `python3 mmio.py 0x10003f00 26 4` — virtio-net debug overlay
     (word 6=irq_count, 14=tx_frames, 18={bad,good}, 20=rx_deliver, 21=nobuf).
   - `python3 mmio.py 0x18000000 20 8` — DDR HPM histogram.
   - NFS throughput: `echo 3 > /proc/sys/vm/drop_caches; time sha256sum
     /boot/initrd.img` (70 MB; ~317 kB/s baseline — limiter is the
     single-buffered eth_rx_engine, a known perf item, not a bug).
4. **Fixed MAC for virtio-net** (DHCP address changes every boot; ssh
   workflows want it pinned). Provide a config-space MAC in virtio_mmio or a
   netplan `match`, either works.

## Perf backlog (from the earlier report, unchanged)

RX multi-frame buffering (throughput ×10 candidate) → redirect-CPI fix →
BP phase 1 → then the CPR-vs-ROB and IW=1-no-forwarding decisions per the
criteria in memory (both want the same synth-attribution data).

## Tooling index

- Sim wedge harness: scratchpad DTBs (`tiny128-pmu*.dtb`) + concatenated-cpio
  recipe + `workloads/ipcstat/pmupoke.c` — recipe details in
  memory/project_pmu_enable_wedge.md. The `VIRTIO_MMIO_TRACE` windowed trace
  (`VDEFS="-DVIRTIO_MMIO_TRACE -DVIRTIO_MMIO_T0=<cycle>"`) is what cracked
  fix #2 — claim-loops are visible instantly.
- Cosim repro: `UBUNTU=1 DISK=../workloads/ubuntu/<img> CYC=500000000
  ./run-cosim-linux.sh` (the DISK= is required on this Mac; without it the
  kernel panics at 0.058 s and the oracle happily locksteps the panic loop —
  retire counts lie unless vda attached).
- Console: screen `ubcon` on coffee, log `workloads/ubuntu/screenlog.0`,
  paced sender `/tmp/sendcmd.sh` (update `S=` on session change).
