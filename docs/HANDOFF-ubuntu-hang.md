# HANDOFF — Ubuntu hangs on FPGA hardware (2026-07-25)

Written by the outgoing model after a 24h session that made **zero progress on the actual
question** (why Ubuntu hangs on the FPGA) and wasted a 12h sim on a bad cap. Read this so you
don't repeat that. The deep prior history is in memory `project_ubuntu_hostname_wedge` — **read
that whole file first**; it has weeks of real investigation.

## THE PROBLEM (unchanged, still open)
Ubuntu boots on the probe OoO core on the XCKU5P FPGA, reaches systemd, the generators run and
succeed, then it **hangs** ("the usual spot") ~post-generators. This is what TT cares about.

## HARD FACTS confirmed this session
- **The FPGA bit that hangs is `d2674b9`** (current main: CKMAX=1 + owner-guard `fd592d2` +
  count-guard `e06e2aa`), verified via the build-id MMIO (`rtl=d2674b9f`). So **every fix this
  session is already on the failing bit and does NOT cure the hang.**
- **Storage-independent.** TT gets the identical hang with the ubuntu-mini *initrd* boot on FPGA
  (no virtio-blk, no disk) as with the disk boot. Kills any virtio-blk/DMA hypothesis.
- **It's a real-timing effect.** cosim (which forces/mirrors interrupts, zero-latency) does NOT
  reproduce this freeze. Only tb_virtio / FPGA (real CLINT/PLIC/interrupt latency) reproduce it.
- **~12h of sim to reach the wedge** (TT's number). It is far into boot.

## DO NOT CONFLATE TWO DIFFERENT BUGS (I did, repeatedly — big time sink)
1. **The HW freeze** (above): post-generators, real-timing, cosim-invisible. **THIS is the target.**
2. **The cosim store-corruption** at retire ~8.368B (`sd a0,80(sp)` in `ret_from_exception` commits
   0x0 instead of 0xb across a page-fault). We root-caused this to a leaked wrong-path writeback and
   "fixed" it with the owner-guard. **It is a DIFFERENT bug, earlier-invisible to the HW symptom, and
   its fix is UNVALIDATED** — the 12h re-run I did was capped at CYC=60B and stopped at retire 8.340B,
   ~28M short of the divergence seqno (and the fixed boot stalled in a poll-loop). "0 divergence to
   8.34B" is NOT evidence: the *broken* build was also clean to 8.368B. See
   `feedback_no_cycle_cap_on_event_runs`. Don't chase #2 thinking it's #1.

## READY ASSET (the one useful thing from this session)
A **validated tb_virtio real-timing repro of ubuntu-mini** (real interrupt timing — the ingredient
cosim lacks). From `probe/`:
```
DTB=../workloads/ubuntu-mini/ubuntu-ram-dbg.dtb \
INITRD=../workloads/ubuntu-mini/ubuntu-mini.cpio \
INITRD_OFF=10000000 DISK=none PROBE_IW=2 CYC=0 BUILD=1 \
./run-virtio.sh > /tmp/tbv.log 2>&1 &
```
- Boots Linux 7.1.0, "SmooolRV64 w/2048 MiB RAM-root", DDR 2GB, no disk. Validated to kernel.
- `run-virtio.sh` was edited to honor `DISK=none` (uncommitted; keep or drop).
- **CYC=0 = NO CAP.** Do NOT put a cycle cap on it. The freeze / a wedge-watchdog / login is the stop.
- ~12h to the freeze (initramfs unpack alone is ~6-8B cycles).

## OPEN LEADS (from the deep memory — verify against current code, all pre-date this session)
- **Lost timer-wakeup**: a sleeping task never woken — `mtime >= stimecmp` (Sstc) but STIP not
  delivered. Instrument tb_virtio's periodic `[c= pc=]` print (tb_virtio.v:244) with priv / mip /
  mie / mtime (`dut.u_clint.mtime/mtimecmp`) / stimecmp / satp to test this at the freeze.
- **FP->INT tag race** at seq ~3.077B (a poisoned int from FP->INT writeback = a GLib size seed).
- **VA-alias stale-value** LSU gap (memfd double-mapping in the systemd sandbox).
- **sandbox-stress** (workloads/stress): a tiny payload mimicking the generator sandbox — IF it
  trips the freeze under *real timing* in tb_virtio, it's a minutes-not-hours loop. Worth trying FIRST.

## SUGGESTED FIRST MOVES for the next model
1. Read `project_ubuntu_hostname_wedge` fully.
2. Instrument tb_virtio BEFORE the long run (one uncapped shot): a UART-silence watchdog that, when
   the console goes quiet post-generators, dumps PC + full interrupt/timer CSR state (lost-wakeup test).
3. Prefer sandbox-stress if it reproduces under real timing — otherwise the 12h tb_virtio full boot.
4. Never cap an event-targeting run by a cycle number.

## GIT / ENV STATE
- main = perf/bp = `d2674b9` (CKMAX=1 default; owner-guard + count-guard + YAGS + checkers; wide-I$
  reverted/parked on branch `parked/wide-icache`). Working tree: only the `run-virtio.sh` DISK=none edit.
- **Vivado is NOT on this Mac** — it's on a Linux host; TT runs all FPGA builds. You do sim + RTL only.
- No sims running as of handoff (cosim finished, tb_virtio killed).
