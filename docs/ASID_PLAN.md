# SmolRV64 ASID Plan

This note records the intended path for correct RISC-V ASID support in
SmolRV64. ASIDs are useful only once translations can be cached across address
space switches. Until there is an ASID-tagged TLB, the architecturally honest
behavior is to report no implemented ASID bits.

## Current Position

SmolRV64 currently performs page table walks directly and does not have a TLB.
With no TLB, every translation observes the current `satp` and current page
tables, so ASIDs provide no performance benefit.

For now, `satp` should therefore behave as though `ASIDLEN == 0`:

- Bare writes read back as zero.
- Sv39 writes preserve `MODE=8` and `PPN`.
- Sv39 writes clear all ASID bits in `satp[59:44]`.
- Unsupported `MODE` writes have no effect.

This makes Linux disable its ASID allocator instead of relying on ASID-tagged
translation state that the hardware does not yet implement.

## Future Goal

Add real ASID support as part of the unified TLB work. The TLB should support
keeping translations from multiple address spaces resident without flushing on
every context switch.

The final ASID width does not need to be 16 bits. A smaller ASID field is
probably enough for this core. Candidate widths:

- 0 bits: current no-TLB behavior.
- 4 bits: 16 ASIDs, very small and easy to test, probably enough to reduce
  obvious context switch churn on single-core workloads.
- 6 bits: 64 ASIDs, still cheap and likely more than enough for a small FPGA
  core.
- 8 bits: 256 ASIDs, a conservative upper bound for this design unless Linux
  workloads show real pressure.

The implemented ASID width must be exposed through `satp` WARL behavior: only
implemented low ASID bits may stick, and unimplemented ASID bits must read back
as zero.

## Required RTL Work

- Add a unified TLB for instruction fetch, load/store, and page-table-walk
  consumers as appropriate.
- Tag every non-global TLB entry with:
  - ASID.
  - VPN match bits.
  - Page size or leaf level.
  - PPN.
  - Permission bits and fault-relevant PTE state.
- Support Sv39 leaf sizes:
  - 4 KiB.
  - 2 MiB.
  - 1 GiB.
- Match superpage entries using only the VPN bits relevant to their level.
- Treat `PTE.G` global mappings specially: global entries match regardless of
  ASID.
- Make TLB fill use the ASID from the active `satp` at the time of the walk.
- Make TLB lookup use the current `satp` ASID for non-global entries.

## `SFENCE.VMA` Requirements

Correct ASID support requires real `SFENCE.VMA` invalidation. NOP behavior is
not sufficient once translations are cached.

Required invalidation behavior:

- `sfence.vma x0, x0`: invalidate all non-global TLB entries.
- `sfence.vma va, x0`: invalidate non-global entries matching `va`, across all
  ASIDs.
- `sfence.vma x0, asid`: invalidate all non-global entries for `asid`.
- `sfence.vma va, asid`: invalidate non-global entries matching both `va` and
  `asid`.

Global entries are not invalidated by ASID-specific fences. A full global flush
policy can be considered later if needed for debugging, but it should not be
the architectural steady-state behavior.

## `satp` Rules

The `satp` CSR should be WARL:

- Bare mode is supported.
- Sv39 is supported.
- Unsupported modes leave `satp` unchanged.
- Implemented ASID bits stick.
- Unimplemented ASID bits read as zero.
- PPN is preserved subject to the supported address width.

Changing `satp` must not require a hidden full TLB flush for correctness. RISC-V
software is responsible for issuing `SFENCE.VMA` when reusing an ASID for a
different address space or after changing relevant page tables. The hardware
must make this contract true by tagging translations correctly and honoring
`SFENCE.VMA`.

## Cache Coherence

The unified physical data cache is coherent with page-table memory from the
core's point of view. Page-table writes update normal memory, and later page
table walks observe that memory through the cache.

The stale state introduced by a TLB is separate from the data cache. TLB
entries must be invalidated by `SFENCE.VMA`; the data cache does not need to be
flushed for ordinary page-table updates.

## Testing Plan

- Boot Linux with `ASIDLEN == 0` and confirm Linux reports ASID allocator
  disabled.
- Add a small ASID width, such as 4 or 6 bits, with a TLB.
- Confirm Linux reports the implemented width.
- Run context-switch-heavy workloads.
- Stress ASID reuse by forcing a small ASID width.
- Test each `SFENCE.VMA` form directly in simulation.
- Test global mappings and confirm ASID-specific fences do not evict them.
- Test 4 KiB, 2 MiB, and 1 GiB TLB entries.
- Add randomized page table mutation tests that perform page-table writes,
  fences, and accesses across multiple ASIDs.

