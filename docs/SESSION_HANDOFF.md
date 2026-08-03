# Session Handoff — RVA22 Cache Extensions + VirtIO Restart

Branch: `dev`. All work below is committed; working tree has no uncommitted
*source* changes (only Vivado build artifacts and unrelated workload blobs).

## TL;DR

All four RVA22 cache extensions are **done, validated, and committed**. The last
one (Svpbmt NC bypass) provides an **uncached memory window**, which is exactly
the missing piece that blocked VirtIO. The next task is **restarting VirtIO on
the non-coherent (NC-vring) path** — basically DT changes + bitstream rebuild +
on-board retest, no new RTL to start.

## Commits this session (newest first)

- `3cfc6ae` Svpbmt: honor NC/IO via L1 bypass (flush-around); advertise svpbmt
- `037fedb` Svpbmt foundation: PBMT faults + uncacheable bit plumbing
- `37bb04b` Add Svpbmt plan
- `9dc2838` Implement Zicboz cbo.zero as a whole-line cache zero  (also folded in
  the tiny128.dts zicbom/zicbop/zicboz advertisement)

Earlier baseline was `97ad9ad`.

## Extension status

| Ext | State | How it's done |
|-----|-------|---------------|
| **Zicbom** `cbo.clean/flush/inval` | shipping (pre-existing) + advertised | already in DUT |
| **Zicboz** `cbo.zero` | done | reuses store path; one cache write zeros all 8 banks via the per-bank write-enable mask; `cache_req_zero` flag + 2 combinational hooks. Ungated on CBZE. |
| **Zicbop** `prefetch.r/w/i` | done | already no-ops (ORI-x0 hint space); no DUT change, just advertised |
| **Svpbmt** | done | perm widened to 6 bits `{uncacheable,physical,U,X,W,R}` (uncacheable=`perm[5]`); PTW derives it from PBMT; cache flush-around bypass |

### Svpbmt NC bypass — how it works (key design)

- Perm field is 6 bits now; the NC/IO "uncacheable" bit rides the existing perm
  path (`mem_perm` → `cache_req_perm` → TLB → cache meta). `perm[5]`=uncacheable,
  `perm[4]`=physical marker.
- PTW (`S_PTW_PROCESS`): `ptw_pte_uncacheable = pte[62] | pte[61]` folded into the
  translated perm. **PBMT is honored leniently** — no `menvcfg.PBMTE` gate, no
  reserved-encoding fault. (A strict fault was tried in the foundation but
  **diverged from simmerv at 46M** once svpbmt was advertised, because OpenSBI 0.9
  doesn't set PBMTE yet the kernel uses PBMT for ioremap. Lenient matches simmerv
  and tolerates old firmware. Do not re-add the strict fault.)
- Bypass = **flush-around in the cache FSM** (`FILL_LINE_INSTALL` beat-7 +
  `cache_finish_writeback_line`, flag `cache_wb_after_ncstore`): NC lines are
  never installed, so an NC access always misses. NC **load** invalidates the slot
  instead of installing (data already returned). NC **store** writes the
  filled+merged line back to memory via the existing writeback engine, then
  invalidates. No new datapath.
- **Known limitation (deferred, OPTIMIZATIONS.md):** NC stores read-for-ownership
  (fill whole line, merge, writeback). A concurrent DMA write to *other bytes of
  the same 64-byte line* could be clobbered. Safe for virtio (avail/used rings are
  separate lines). Strict fix = Option B (write only the store's bytes, no RFO).

## Validation (all green)

- **riscv-tests 240/240** after every change: `cd tests && JOBS=8 ./run-riscv-tests.sh passes 2>/dev/null | grep -c 'Test Passed'` → 240
- **Directed tests** (in `src/`, run on `smolrv64-tester`):
  - `svpbmt_nc_test.s` (committed) — HPM proof the NC bypass is real: NC reads fill
    every access (~16/16) vs a PMA alias (~1). This is the ONLY way to validate the
    bypass; cosim can't see it (simmerv has no cache).
  - `cbozero_test.s`, `prefetch_test.s` (untracked) — directed checks for those.
  - Build+run: `make -C src smolrv64-tester <name>.even <name>.odd && cd src && ./smolrv64-tester +even=<name>.even +odd=<name>.odd`
- **Cosim** (no-regression, svpbmt advertised): `make -C workloads/tiny128 cosim`
  ran clean **past 54M** retirements under Linux/Sv39. (Run in background; it boots
  Linux in lockstep vs simmerv and aborts loudly on divergence.)
- **FPGA timing** (`make -C platforms/rk-xcku5p-f-v1.2 bit`, grep
  `Estimated Timing Summary` in the log): Zicboz +0.129ns, Svpbmt foundation
  +0.194ns, **NC bypass +0.046ns** (positive but tightest — the writeback branch
  sits on the cache path).

## simmerv (cosim oracle) state

simmerv lives at `~/simmerv`. Committed there this session:
- `aad4d6d` Mask PTE PPN to 44 bits (stop PBMT/N bits leaking into the phys addr) +
  document that `menvcfg.{CBZE,CBIE,CBCFE,PBMTE}` gating is NOT implemented (safe
  because OpenSBI sets menvcfg before S-mode; the DUT is now also lenient, so they
  match).
- `d674f41` README: mark Svnapot + Svpbmt supported.
- cbo.zero, cbo.*, prefetch.* were already correctly implemented in simmerv.

## NEXT TASK: restart VirtIO (non-coherent / NC-vring path)

**Why now:** the last virtio attempt (`366ce99`) died because the device read
`avail.idx==0` forever — vrings were CPU-cached while device DMA read stale DRAM.
Svpbmt NC is the uncached window that fixes this: `dma_alloc_coherent` maps the
vrings PBMT=NC → our bypass sends them to DRAM → device sees them.

**What already exists (in the RK top + DT, just disabled):**
- virtio-net + virtio-blk MMIO devices in
  `platforms/rk-xcku5p-f-v1.2/rk_xcku5p.srcs/rk_xcku5p.v` (selects, TX path, IRQ
  wiring, and a **debug overlay** with avail/notify/complete counters).
- `workloads/ubuntu/ubuntu.dts`: both virtio nodes already carry `dma-noncoherent`
  but are `status = "disabled"`. ISA string advertises zicbom but NOT svpbmt.

**Concrete first steps:**
1. `ubuntu.dts`: set the virtio nodes to `status = "okay"`, and add `svpbmt`
   (+ zicboz, zicbop) to `riscv,isa` and `riscv,isa-extensions` so Linux maps the
   `dma-noncoherent` vrings as NC. (Note: ubuntu.dts also has unrelated pending
   edits / a separate cbom-block already; only touch the virtio status + ISA.)
2. Rebuild bitstream with the new Svpbmt RTL + virtio active:
   `make -C platforms/rk-xcku5p-f-v1.2 bit` (or `make load WORKLOAD=...`).
3. Program the board (`make program`) and boot Ubuntu (`make connect`, 3 Mbaud).
4. Retest virtio-net TX; read the debug overlay at its MMIO address. Success =
   `read_ring_count`/`complete_count` advance past `notify_count` (before,
   `avail.idx` was stuck at 0, read/complete = 0).

**Validation is FPGA-only** — there is no virtio device in cosim, so the loop is
program-board → boot-Ubuntu → debug-overlay, not cosim.

**Strategic note:** this is the non-coherent path (uncached vrings) — fast to a
*working* device, reusing what exists, but vring access stays uncached/slow. The
coherent-DMA snoop (cached rings + DMA snoops the L1) is the performance endgame
and a much larger RTL effort. Recommended order: get virtio working on NC first,
then build the snoop. See `VIRTIO_PLAN.md` (its "Resume Note" predates Svpbmt and
is pessimistic about non-coherent for the cbo reason — Svpbmt NC is a different,
viable mechanism).

## Doc pointers

- `SVPBMT_PLAN.md` — full Svpbmt design, the flush-around bypass, Option B (no-RFO).
- `OPTIMIZATIONS.md` — deferred perf work: cbo.zero cold-miss RFO, single-cycle
  unaligned stores, NC-store RFO.
- `VIRTIO_PLAN.md` — virtio device plan + the pre-Svpbmt resume note.
- `COSIM_HANDOFF.md` — cross-machine cosim setup.
- CLAUDE.md — project conventions (Vivado via the platform Makefile; verify
  riscv-tests = 240 before committing; commit style = terse imperative, no
  Co-Authored-By trailer).
