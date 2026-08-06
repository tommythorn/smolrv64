# Handoff — NC-store line clobber → Ubuntu disk-root boot wedge

**Status 2026-08-06 (evening): FIXED — root cause was one level deeper than below.**
The full-line push was the *blast radius*, not the trigger. The trigger, caught by a
push-integrity checker (`-DWTCHK` in cache.v) plus an FSM history ring during the
boot repro: a **spanning NC store whose line1 fill evicts a DIRTY victim**. The
victim capture (`S_WB`) reuses `wb_way/wb_idx` — the same registers that held the
line0 push slot — so `S_WTR` then streamed the *victim's* slot (which by then held
the freshly installed line1) and `S_WTI` pushed that whole wrong line under line0's
address. The virtqueue desc3.flags store (2 bytes at line offset 60) takes the span
path via the width-based `r_span` test, which is why the descriptor table was the
recurring victim: desc3 → zeros = desc4-7's (zero) line content.

Fixes, all in `cache.v` (+`tb_cache.v` regression that reproduces the exact scenario):
1. `S_SPANW` re-derives the push slot from `w0_*` (stable since phase 0) — never `wb_*`.
2. Span stores serialize against the MSHR at phase 0 (an install preempt after the
   phase-0 merge could reallocate line0's slot the same way).
3. The redundant `S_FIN` low-chunk rewrite for spans is gone (it landed in whatever
   the slot holds after the line1 fill).
Plus, defense in depth: the whole L2/DDR write path now carries per-byte strobes
(`l2_wmask` → `l2_arbiter` → `ddr_wmask` → CDC → AXI WSTRB → all TB DRAM models), so
an NC/WT store push writes ONLY its own bytes (full mask only when the line was
dirty = combined writeback). And `virtio_blk.v` completes an unparseable chain with
an empty used entry instead of silently dropping it.

Previous HEAD = `9ce7d11`. Related memory: `project_ubuntu_boot_wedge_2026_08`.
The original (superseded) analysis follows for the record.

---

## The bug

**`cache.v`'s uncached-store path pushes a WHOLE 64-byte line.**

A byte-granular Svpbmt-NC store takes `S_WTR → S_WTW → S_WTI → S_WTA`: it reads the
line out of the banks, merges its own bytes, and writes all 64 bytes to memory. Any
staleness in that snapshot silently destroys bytes that nothing ever stored to.

The virtqueue **descriptor table packs 4 descriptors per 64-byte line**, and the
driver writes descriptors with separate NC stores — so every store rewrites its
three neighbours from whatever snapshot it holds.

Second, independent defect — `virtio_blk.v:412`:

```verilog
if (dma_rsp_error || (dma_rsp_rdata[47:32] & VRING_DESC_F_NEXT) == 16'd0)
   state <= S_IDLE;      // silent drop: no used entry, no error status, no IRQ
```

A head descriptor the FSM cannot parse is dropped with no trace. This is what turns
a corrupted descriptor into a permanent silent hang, and it hid the cause behind
five clean subsystems for the whole investigation.

## Failure chain (each link measured)

1. Driver NC-stores a descriptor → lands correctly in DDR
2. **An NC full-line push clobbers that line** → descriptor 3 went from
   `len=0x10 flags=NEXT next=4` to all zeros in 3 cycles, with no store targeting it.
   6168/6168 writes to that line came from `dcache.st=16` (`S_WTA`)
3. Driver publishes `ring[7]=3` and `avail.idx=617` — both verified current in DDR
4. `QueueNotify` reaches the device (3×)
5. Device reads `avail.idx=617` and `ring[7]=3` correctly (reads are faithful)
6. Device fetches descriptor 3 → **zeros** → no NEXT → silent drop
7. No completion, no interrupt → page-fault waiter never resumed → kernel idles in
   `do_idle` / `__get_next_timer_interrupt` / `tick_nohz_idle_stop_tick` forever

Userspace dies at **c=798M**; only **16,526 U-mode retires** for the whole boot
(vs 427M kernel retires — demand paging). Last device activity c≈812M.

## Fix direction (the decision to make)

NC stores must be **byte-granular** to memory — never a line RMW. Options:

- **Add byte strobes to the L2/DDR write path** for NC stores. Cleanest
  semantically; touches `cache.v`, `l2_arbiter.v`, the DDR/AXI bridge, and both
  testbench memory models.
- **Make the RMW safe**: re-read the line fresh from DDR and merge+push with no
  window in which another agent can write it. Cheaper, but only narrows the race —
  DMA can still land inside it. Not recommended as the final answer.

Also fix `virtio_blk.v:412` regardless: complete the request with an error status
so the driver learns instead of hanging.

Same family as `project_nc_ptw_flusharound` (`6edac85`). **The NC path deserves a
systematic review, not a third point fix.**

## Reproducing

```
cd src
cp ../workloads/ubuntu/ubuntu-master-ro.img ../workloads/ubuntu/ubuntu-rtl.img
chmod u+w ../workloads/ubuntu/ubuntu-rtl.img
VDEFS="-DNOTIFYCHECK" BUILD=1 CYC=900000000 \
  DISK=../workloads/ubuntu/ubuntu-rtl.img ./run-virtio.sh
```
Fires once, at c≈812.5M:
`[NOTIFY DROPPED ... driver avail.idx=617, device last_avail_idx=615]`
~28 min. That single line is the regression gate for the fix.

Board repro: same wedge, `Hostname set` then `Freezing execution` ~90 s later
(that 90 s is systemd's start timeout — the fault is at its START, t≈12 s).

## Uncommitted instrumentation in the working tree

All `ifdef`-gated, none active by default. Keep or drop as you prefer:

| file | defines |
|---|---|
| `src/plic.v` | `PLIC_TRACE` — claim/complete pairing + stuck-in-service detector |
| `src/tb_virtio.v` | `NOTIFYCHECK` (the repro gate), `DMACHECK`, `RINGWATCH`, `DEVTRACE`, `AVAILTRACE`, `FSMTRACE`, `UMODEWATCH`, `DDRWRITEWATCH` |
| `src/sd_dpi.cpp` | `sd_image_word()` DPI — reads the image file as ground truth |
| `src/tb_cosim_linux.v`, `src/probe_cosim.cpp` | earlier corruption-hunt taps |

`NOTIFYCHECK` and `PLIC_TRACE` are worth committing; the rest are one-off scopes.

## Eliminated by measurement — do NOT re-litigate

- **PLIC delivery**: 617/617 claim/complete balanced, nothing stuck in_service
- **Used-ring staleness**: 17,807/17,807 ring reads `uncached=1`, CPU matches DDR
- **DMA payload**: 614 reads, 0 with wrong data (compared against the image file)
- **Device MMIO reads**: 633/634 paired, correct address and data every time
- **Architectural divergence**: cosim `UBUNTU=1`, 1B cycles, ZERO divergence — the
  oracle cannot see this because it FOLLOWS the DUT for MMIO/IRQ/DMA
- **Stuck AMO**: `amo_pend` toggles normally
- **Descriptor address/slot selection**: correct (see trap #3 below)

## Traps that cost runs here

1. Checkers carrying **my own bookkeeping** produce false positives — a used-ring
   "STALE" check compared mismatched byte offsets and manufactured 3,614 false hits.
   Prefer passive logs compared against ground truth (the image file, DDR, the
   design's own registers).
2. `sd_dpi` opens the image **O_RDWR** → a read-only master gives
   `[sd_dpi] FATAL: cannot open` → silent fallback capacity → no partition table →
   root panic. `+disk_ro` is snapshot mode; it still needs write PERMISSION.
   (Cost a 34-min cosim run.)
3. `wb_way/wb_idx/r_addr` sampled at **DDR-write** time are cycles late — the push
   issued earlier. An "off-by-one line" concluded from that was a sampling error;
   addresses and slots are correct. **Sample at `l2_req`.**
4. `systemd.log_level=debug` makes this board wedge EARLIER with ZERO output (2/2,
   both `console` and `kmsg` targets). Use baseline bootargs.
5. `dmem_wen` is held across cycles — raw store counts in traces are inflated;
   collapse duplicates before drawing conclusions.
6. Verilator 5.050 MISCOMPILES `for (i) if (mask[i]) mem[base+i] <= x;` (guarded NBA
   to a memory in a loop) in some blocks — it corrupted an UNRELATED read path in
   tb_vl, deterministically, with an all-ones mask. Write TB masked memory updates
   as `mem[i] <= mask[i] ? new : mem[i]` or a full-word and/or merge instead.

## Also open (unrelated)

- **Confidence-gated checkpoint inclusion** — TT's design, planned in
  `docs/confidence-checkpoint-plan.md`, untouched. 5 steps, first 4 inert plumbing.
- `r_span` in `cache.v` tests the full store WIDTH, ignoring the byte mask, so a
  4-byte store near a line end takes the span path unnecessarily. Correct after
  `9ce7d11` (the case is folded in), but it costs a needless second lookup.
- `ring_mask` in `virtio_blk.v` derives from the `QUEUE_SIZE` parameter, not the
  negotiated `queue_num`. Benign at 8/8; latent if negotiation ever changes.
