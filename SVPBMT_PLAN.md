# Svpbmt Implementation Plan

Status: **recognition trivial / honored partially; NC cache-bypass deferred.**

Svpbmt (Supervisor-mode Page-Based Memory Types, mandatory from RVA22S64) adds a
2-bit memory-type field to each leaf PTE, bits **[62:61]**:

| PBMT | Meaning |
|------|---------|
| 00 PMA | default (normal cached main memory) |
| 01 NC  | non-cacheable, idempotent, weakly-ordered main memory |
| 10 IO  | non-cacheable, non-idempotent, strongly-ordered |
| 11     | reserved (must page-fault) |

## What already works (no change needed)

- **Addressing is already correct.** The PTW computes the physical address from
  `aligned[53:10]` (PPN), which *excludes* the PBMT bits [62:61] and the N bit
  [63]. So a PBMT-annotated PTE translates to the right address today — the DUT
  does not have simmerv's old PPN-leak bug.
- **`menvcfg.PBMTE` (bit 62) is already a stored WARL bit** (`csr_menvcfg`), so the
  enable bit needs no new storage.
- **IO-type pages that map to MMIO are already uncacheable** via `phys_region`
  (UART/PLIC/CLINT/MMIO regions never enter the L1). So `ioremap` of devices with
  PBMT=IO behaves correctly already.

## The gap: NC (and IO) pages that map to DRAM are still cached

`clear`/load/store routing decides cacheable purely by physical region:
`phys_region(mem_addr) == REGION_BRAM || REGION_DRAM` (smolrv64.v ~5020, 7133,
7234, 7406, 7450, and the PTW PTE read at ~8463). There is **no uncacheable-DRAM
path** — a DRAM physical address always goes through the L1. So a page marked NC
whose PPN lands in DRAM is currently cached. For a single coherent hart with no
DMA this is *functionally* invisible, which is why it is safe to defer; but it is
exactly the property [[project_virtio_coherent_dma]] needs (uncached vrings), and
strictly it is non-compliant for NC.

## Work to honor NC/IO bypass

1. **PTW (`S_PTW_PROCESS`, ~8316):** extract `pbmt = aligned[62:61]`. Fault
   (`ptw_fault_cause`) if `pbmt == 2'b11` (reserved) or if `pbmt != 0 &&
   !csr_menvcfg[62]` (Svpbmt disabled). Derive an `uncacheable = (pbmt == NC ||
   pbmt == IO)` bit.
2. **Carry the bit to the access.** For 4 KiB pages it must ride the TLB entry
   (widen `TLB_*_DATA_BITS` by 1, store `uncacheable`, return it on hit). For the
   superpage / NAPOT paths (`route_translated_addr`) it is available inline.
3. **Cache routing.** Every cacheable test (the ~5 sites above) becomes
   `cacheable_region && !page_uncacheable`. NC/IO DRAM accesses then need a real
   uncacheable DRAM datapath: reuse the existing `ptw_direct` read machinery
   (BRAM read / `l2_direct_read`, which already bypasses the VHPR L1) for loads,
   and add a matching direct DRAM write for stores. IO additionally implies
   strong ordering (no speculation/reorder).
4. **Advertise** `svpbmt` in the DT once NC bypass is real.

## Why this is deferred (not rush-committed)

- **Hot-path + zero-margin timing.** The cacheable decision sits on the common
  load/store path; adding an NC term there risks unrelated timing failures on the
  +0.044 ns-class design ([[project_fpga_timing_margin]]). Must be measured with
  `make timing`, not assumed.
- **The key behavior is not cosim-observable.** simmerv has no cache, so NC vs PMA
  are architecturally identical there — cosim cannot catch a bug where the L1
  fails to actually bypass. Validation needs an **HPM-counter directed test**: a
  bare-metal program with hand-built Sv39 page tables mapping one page NC and one
  PMA, looping loads/stores over each, asserting `VHPR_FILLS`/hit counters stay
  zero for the NC page and advance for the PMA page. (Then the real virtio
  coherent-DMA test once that exists.)
- These together make it a change that wants the author's review and a dedicated
  validation pass, not an unattended commit.

## simmerv (cosim oracle) parity

simmerv already accepts PBMT and masks the PPN (committed: `aad4d6d`). It does not
gate on `menvcfg.PBMTE` (documented gap, csr.rs). If the DUT adds PBMTE/reserved
faults, simmerv should match for exact parity — but OpenSBI sets `menvcfg` before
S-mode and Linux never sets reserved PBMT, so normal boots do not exercise the
divergent paths.
