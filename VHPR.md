# VHPR L1 Cache

VHPR means "virtually hit, physically reconciled".  It is an L1 cache design
that removes TLB lookup from the ordinary L1 hit path while preserving physical
correctness on misses, writeback, and coherence operations.

The L1 is backed by a physically indexed, physically tagged L2.

## Goals

- Keep the L1 hit path short enough for FPGA timing closure.
- Avoid TLB access on ordinary L1 hits.
- Keep a single resident L1 copy for each physical cache line.
- Support write-back L1 behavior without duplicate dirty aliases.
- Move synonym handling and translation cost off the hit path.
- Accept extra physical-tag metadata as the price of the virtual-hit path.
- Keep the first RTL implementation small enough to reason about and test.

## Configuration

Initial fixed target:

```text
capacity       = 64 KiB instruction + 64 KiB data
line size      = 64 bytes
associativity  = 2-way skew associative
indices per way = 512
index bits     = 9
index example  = VA[14:6]
page size      = 4 KiB
```

For 4 KiB pages and 64-byte lines:

```text
line-within-page offset = VA[11:6]
virtual color           = VA[14:12]
```

Aliases of the same physical line share `VA[11:6] == PA[11:6]`, but may have
different virtual colors.

Each cache is independently VHPR-indexed and VHPR-tagged.  The data cache is
write-back.  The instruction cache never holds dirty lines, but otherwise uses
the same virtual-hit, physical-reconcile rules.

The synonym search domain per cache is therefore:

```text
2 ways * 8 virtual colors = 16 physical tags
```

Comparing four physical tags per cycle searches the full domain in four cycles.
This can overlap naturally with a four-beat 64-byte line fill.

## L1 Line Metadata

Each L1 line stores:

```text
valid
dirty
ASID
virtual tag
physical tag
valid epoch
cached permission/state bits
replacement/coherence state as needed
```

The virtual tag is used for L1 hit determination.

The physical tag is used for:

```text
synonym reconciliation
writeback address generation
physical coherence probes
DMA-related cache maintenance
single-copy invariant checks
```

The physical tags are the main metadata overhead of VHPR.  They are not on the
ordinary virtual-hit compare path, but they must be stored per line and read by
miss, coherence, DMA, and writeback machinery.

The valid epoch is a compact generation tag for conservative invalidation.  A
flush can advance the current epoch instead of clearing every clean line
immediately.  Dirty lines still require physical writeback before their old
contents may be discarded.

## Core Invariant

Each VHPR L1 must maintain:

```text
At most one valid line may exist for any physical cache line within that L1.
```

This is the central correctness rule for a VHPR cache.  In the data cache it
prevents duplicate dirty ownership and keeps physical visibility unambiguous.
In the instruction cache it prevents stale virtual aliases from silently
surviving synonym movement.

The invariant must hold after every:

```text
allocation
eviction
invalidation
synonym migration
writeback
SFENCE.VMA-driven flush
coherence or DMA probe response
```

## Hit Path

The ordinary hit path is virtual and does not access the TLB.

Inputs:

```text
VA
ASID
access type
current privilege/control state needed for permission evaluation
```

Pipeline:

```text
1. Compute the candidate index for each way from the request VA.
2. Read both candidate lines' tags, metadata, and data.
3. Compare valid, ASID, and virtual tag.
4. Re-evaluate cached permission/state bits for the current access.
5. Return data or accept store data on hit.
```

Hit condition:

```text
valid &&
line.epoch == current_epoch &&
line.ASID == request.ASID &&
line.virtual_tag == request.virtual_tag &&
permissions_allow_access
```

The physical tag is not part of the ordinary hit condition.  Comparing the
stored physical tag against a freshly translated physical address on every hit
is useful as temporary debug instrumentation, but it puts translation back into
the hit path and defeats the point of VHPR.

Global mappings may later relax the ASID comparison with a per-line global bit.
The first implementation can keep all lines ASID-qualified.

The SATP root PPN is also not part of the hit tag.  Translation visibility is
controlled by `SFENCE.VMA`; a SATP CSR write by itself does not invalidate VHPR
or TLB state.

## Indexing And Skewing

Each way may use a different index function:

```text
way0_index = hash0(VA, ASID)
way1_index = hash1(VA, ASID)
```

The hash functions choose the two hit-path candidate locations for the request
VA.  The miss path must also be able to enumerate every location in each way
whose line-within-page component is `PA[11:6]`.

The implementation should treat the index as:

```text
ASID-mixed virtual color + line-within-page offset
```

possibly with skewing or permutation inside each way, as long as the same-offset
candidate set remains directly enumerable.

Hashing ASID into the virtual color improves mixing between address spaces and
reduces avoidable conflicts.  It does not change the physical synonym search
size: the miss path still enumerates all eight color positions for the
line-within-page offset in each way.

## Miss Path

On a virtual miss:

```text
1. Translate VA to PA and page permission/state information.
2. Select the target allocation location from the request VA.
3. Probe all 16 same-offset physical-tag candidates.
4. Reconcile any resident physical synonym.
5. Allocate, fill, or migrate the requested line.
```

The physical-tag probe checks all lines that could contain the requested PA:

```text
for each way:
  for each virtual color:
    read line at {way, virtual_color, PA[11:6]}
    compare valid && physical_tag == request physical_tag
```

There can be at most one match if the invariant has been maintained.

## Synonym Cases

### No Synonym

If no physical tag matches:

```text
evict target victim if needed
fill target line from L2
install virtual tag, physical tag, ASID, permissions
```

### Clean Synonym

If a matching physical line is resident and clean:

```text
invalidate old synonym
fill target line from L2
```

An optimized implementation may migrate the clean data directly instead of
using L2 fill data.

### Dirty Synonym

If a matching physical line is resident and dirty, the L1 line is the newest
copy.  L2 data for the same PA must be treated as stale until the dirty line is
written back or migrated.

Initial policy:

```text
write back dirty synonym to L2
invalidate old synonym
fill target line from L2
```

This is slower than migration, but it keeps the first implementation simple and
keeps all line movement through the existing physical writeback/fill path.

The implementation should count dirty alias evictions so the cost of this
policy is measured under real workloads.

Future optimization:

```text
evict target victim if needed
copy dirty synonym data into target location
install new virtual tag, ASID, permissions, and same physical tag
preserve dirty state
invalidate old synonym
```

This synonym migration avoids writeback followed by refill, but adds more data
movement and corner cases.

## Overlapped Miss Pipeline

The 64 KiB configuration permits a compact overlapped miss pipeline.

Safe sequence:

```text
1. Virtual miss translates to PA.

2. Reserve:
   - target allocation location
   - all 16 possible synonym locations for PA[11:6]
   - lower-level transaction state for the PA line

3. If the target location contains a dirty victim:
   copy it into a victim buffer, two 64-bit words per cycle.

4. In parallel:
   - probe four physical tags per cycle
   - start a provisional L2 lookup/fill

5. If no synonym is found:
   commit the L2 fill into the target location.

6. If a clean synonym is found:
   invalidate the old synonym, then commit the L2 fill or migrate clean data.

7. If a dirty synonym is found:
   discard or ignore provisional L2 fill data.
   write back the dirty synonym to L2.
   invalidate the old synonym.
   refill the target location from L2.

8. Release reservations after the single-copy invariant is restored.
```

The L2 fill must not become architecturally visible until the synonym probe is
resolved.  If the dirty-synonym case is detected after a provisional fill has
started, the fill data must be discarded or replayed after the dirty writeback
updates L2.

## L2 Role

The L2 is physically indexed and physically tagged.  It provides:

```text
capacity behind the 64 KiB L1
physical writeback destination
physical coherence point
DMA integration point
```

The L1 is optimized for virtual hit latency.  The L2 is the physical authority.

All L1 writebacks use physical address information from the L1 physical tag and
index/offset state.

Page-table walker reads should use the physical L2 path, not the virtual-hit L1
path.  The PTW is a producer of translation information; letting it consume the
VHPR hit path makes the translation mechanism depend on translation-derived
state and obscures ordering bugs.  If the L1 holds dirty page-table data, the
physical PTW path must first reconcile that L1 line before reading the backing
store.  A simple implementation can perform a physical probe for the PTE line,
write back and invalidate any matching dirty line, and only then issue the L2 or
backing-memory read.  This is ordinary physical coherence for a PTW read that
intentionally bypasses the virtual-hit L1 lookup.

## Physical Residency Lookup

A separate physical directory is not required for the first implementation.

The L1 physical tags already form a distributed residency structure:

```text
cache location -> physical tag
```

The miss path searches the bounded same-offset domain and compares physical
tags.  With the 64 KiB configuration, that search is 16 candidates.

A later implementation can add a reverse directory:

```text
physical tag -> cache location
```

but that adds exactness and consistency requirements.  The bounded physical-tag
search is the simpler first design.

## Permissions

Lines should cache PTE-derived permission and page-state information, not just
a pre-resolved yes/no decision.

Recommended cached state:

```text
R/W/X/U
page size
global bit, when ASIDs are implemented
A/D state if needed by the implementation
PMA/PMP-derived access and cacheability state, if used
```

On hit, the cache re-evaluates the cached state against the current request and
current privilege/control state.  This avoids a TLB lookup while remaining
robust across changes to `SUM`, `MXR`, `MPRV`, and privilege mode.

## SFENCE.VMA And Mapping Changes

The initial implementation should use conservative invalidation:

```text
SFENCE.VMA:
  invalidate or flush the full VHPR L1
  invalidate translation cache state according to the TLB policy
```

The full L1 flush avoids stale virtual tags and stale cached permissions after
page table changes.

Later optimizations may add ASID-selective or VA-selective L1 invalidation.

The hardware requirement is to honor the same `SFENCE.VMA` invalidation contract
for both TLB entries and VHPR virtual-hit lines.

## Epoch Roll-Over

Epoch invalidation only works while an old line's epoch cannot be confused with
the current epoch.  With a small epoch, roll-over must be handled explicitly.

Required behavior:

```text
advance epoch:
  if next_epoch != 0:
    publish next_epoch
  else:
    flush/write back the full L1
    publish epoch 0 only after the flush completes
```

The current RTL uses a 2-bit VHPR epoch.  That keeps the hit metadata small, but
means roll-over is normal under long Linux runs.  Roll-over is correct only if
the cache performs a full walk of both ways, writes back dirty lines, invalidates
old clean lines, and delays publishing the wrapped epoch until that walk is
complete.

The monitor may expose the current functional epoch, but production builds
should not keep hot-path debug counters or physical-tag mismatch checks enabled.

## Coherence And DMA

External physical probes do not have a useful VA.  They should use the same
bounded physical-tag search as miss reconciliation:

```text
physical probe PA
  -> enumerate 16 same-offset L1 candidates
  -> compare physical tags
  -> respond with hit/miss, dirty state, and data or invalidation as required
```

The L2 should be the main physical coherence and DMA integration point.  The
data L1 must still respond correctly when it holds the newest dirty copy.

The split instruction cache is coherent through the architectural instruction
synchronization path.  `FENCE.I` flushes frontend speculation and invalidates
instruction-cache state after any required data-cache writeback, so later
fetches refill from the physical backing hierarchy.  Ordinary instruction
fills do not probe the data-cache virtual-hit path.

## Simulator Prototype

An ISA simulator prototype should model the VHPR L1 explicitly rather than
treating it as an ordinary VIPT cache.

Required assertions:

```text
no two valid L1 lines have the same physical line address
dirty synonym data is never overwritten by stale L2 fill data
L2 fill data is not committed until synonym probing resolves
SFENCE.VMA invalidates stale virtual-hit state
PTW reads use the physical L2 path rather than the VHPR hit path
epoch roll-over cannot make an old line valid in the new epoch
permission hits are rechecked against current privilege/control state
```

Useful stress cases:

```text
clean aliases
dirty aliases
alternating accesses through aliases
target victim dirty while requested PA also has a dirty synonym
ASID changes
global mappings
SUM/MXR/MPRV transitions
permission downgrades followed by SFENCE.VMA
long runs with repeated epoch roll-over
coherence or DMA probes while aliases are present
```

Useful counters:

```text
L1 virtual hits
L1 virtual misses
synonym probes
synonym hits
clean synonym invalidations
alias evictions
dirty alias evictions
dirty alias writebacks
target dirty victim evictions
provisional L2 fills discarded due to dirty synonym
full L1 flushes
```
