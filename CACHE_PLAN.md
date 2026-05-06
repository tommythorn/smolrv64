# SmolRV64 Physical Cache Plan

This is the implementation plan for adding a physical cache to SmolRV64.
The goal is to remove most of the DDR/MIG latency from Linux workloads while
also simplifying the existing memory paths. The first cache should be simple,
blocking, physically indexed, physically tagged, direct mapped, and unified for
instruction fetch, data access, and page-table walks.

## Goals

- Improve Linux and monitor performance substantially on FPGA.
- Keep correctness higher priority than resource use or peak performance.
- Replace the current collection of separate SRAM, DRAM, MMIO, fetch, load,
  store, and PTW paths with a single memory front-end.
- Use FPGA UltraRAM if timing and placement allow it.
- Keep instruction fetch simple by requiring all executable memory to be
  cacheable.
- Cache normal memory, including local SRAM and external DRAM.
- Never architecturally cache MMIO.
- Make the first implementation easy to debug in simulation and on hardware.

## Non-Goals For The First Version

- No TLB yet.
- No out-of-order memory behavior.
- No nonblocking cache misses.
- No hit-under-miss.
- No write-back cache initially.
- No cache coherence with external DMA masters.
- No speculative MMIO.
- No instruction fetch from MMIO.
- No separate I-cache/D-cache.

## Proposed First Design

- Unified L1 cache.
- Physically indexed, physically tagged.
- Direct mapped.
- 64-byte cache lines.
- 64-bit CPU access granularity.
- 8 x 64-bit banks per cache line.
- Blocking miss handling.
- Write-through.
- Write-allocate for cached stores.
- No dirty bits initially.
- MMIO uses an uncached bypass through the same front-end.
- Instruction fetch is only allowed from cacheable physical memory.
- Page table walker reads go through the cache.

The cache should be parameterized, but the first real FPGA target should be a
large UltraRAM-backed cache:

```text
1 MiB cache:
  line size       = 64 bytes
  line count      = 16384
  offset bits     = 6
  index bits      = 14
  covered window  = 20 PA bits per index+offset

2 MiB cache:
  line size       = 64 bytes
  line count      = 32768
  offset bits     = 6
  index bits      = 15
  covered window  = 21 PA bits per index+offset
```

The implementation should support smaller sizes for simulation and synthesis
experiments, but it should not be designed around a tiny LUTRAM-only cache.

## Current Status

The first hardware cache milestone has been reached: SmolRV64 boots Ubuntu on
the FPGA with a unified physical DRAM read cache, and the large cache arrays no
longer synthesize as distributed RAM. The implementation is still a prototype,
not the final memory architecture described by this plan.

Known current properties:

- DRAM cache data arrays and tag metadata are mapped to FPGA memory resources
  instead of massive LUTRAM.
- The cache is unified for the implemented DRAM read path and has been tested
  far enough to boot Ubuntu on hardware.
- Stores to DRAM now use write-allocate/write-back behavior: cache hits update
  the selected bytes and mark the line dirty; dirty direct-mapped victims are
  written back before refill.
- The local `mem0`/`mem1` SRAM arrays still appear as separate memories in the
  load/store/fetch path. They should instead become cacheable backing memory
  behind the same memory front-end as DRAM, or be replaced by that backing path.
- The current RTL contains a correctness workaround for Sv39 data accesses that
  cross a 4 KiB page boundary: those accesses trap as misaligned instead of
  being split across two translated pages. That avoids the corruption class that
  blocked Ubuntu boot, but it is not the desired long-term behavior.

## Outstanding Work

The remaining cache work is substantial. This list is not ordered by priority.

- Move `mem0`/`mem1` behind the cache/front-end. Local SRAM should be cacheable
  backing memory, like DRAM, not a separate load/store/fetch fast path. This is
  both a cleanup goal and a synthesis goal, because those arrays still show up
  as independent memories.
- Continue validating the write-back policy under hardware workloads. The first
  write-back implementation is still blocking and direct-mapped: it has dirty
  metadata, store allocation, and eviction writeback, but no explicit cache
  maintenance path, write buffer, or external DMA coherence.
- Consider a two-way skew-associative cache. A direct-mapped cache is simple,
  but Linux can create conflict-heavy access patterns. Two ways with different
  index hashes should reduce misses more than simply shrinking/expanding a
  direct-mapped cache, but it needs a replacement policy and a timing study.
- Review all misaligned memory operation paths: within one 64-byte line,
  crossing 64-bit banks inside one line, crossing two cache lines, and crossing
  two pages. Ideally these should have no added penalty on cache hits.
  Cross-page accesses are the hardest case because both pages must be
  translated, checked, and then combined/split without partial corruption.
- Eventually split instruction and data caches, but keep them coherent. A split
  design needs `fence.i` semantics, D-cache to I-cache invalidation or snooping,
  and PTW coherence with stores that update page tables.
- Timing: quantify how small the cache would need to be to recover the cycle or
  pipeline stage added to meet timing. Compare cache size, associativity, BRAM
  versus UltraRAM mapping, WNS, resource use, and hit latency.
- Add cache-specific simulation and hardware stress tests for corruption cases:
  repeated SD reads, page-table-heavy workloads, line-crossing stores, random
  byte-mask stores, replacement under write pressure, and user/kernel workloads
  that exercise high virtual addresses near Sv39 boundaries.

## Current Memory Map Context

Current relevant regions:

```text
0x0200_0000 .. 0x0200_ffff  CLINT
0x0c00_0000 .. 0x0fff_ffff  PLIC
0x1000_0000 .. 0x1000_00ff  UART
0x1000_1000 .. 0x1000_10ff  SPI
0x1000_1100 .. 0x1000_11ff  SD chip-select GPIO
0x1000_1200 .. 0x1000_12ff  SD card-detect GPIO
0x7000_0000 .. SRAM_TOP     local SRAM / monitor memory
0x8000_0000 .. RAM_TOP      external DRAM
```

The exact SRAM size is controlled by `MEM_BASEADDR` and `MEM_SIZE_LG2`. The
FPGA Linux memory size is described by the DTB. The cache should use explicit
hardware region checks, not infer cacheability from the DTB.

Initial cacheability rules:

```text
local SRAM range             cacheable
external DRAM range          cacheable
everything else              uncached data/MMIO
uncached instruction fetch   access fault
```

This keeps the current monitor SRAM cached as well as DDR. Caching SRAM avoids
special instruction-fetch and load/store paths for monitor code.

## Architectural Rule: Cacheability Is By Physical Address

The cache is PIPT, so lookup happens after translation.

Instruction fetch:

```text
virtual PC
  -> existing translation/PTW if paging is enabled
  -> physical address
  -> cacheable-region check
  -> cache lookup/fill
```

Load/store:

```text
virtual data address
  -> existing translation/PTW if paging is enabled
  -> physical address
  -> cacheable-region check
      cached: cache lookup/fill/store update/write-back
      uncached: single MMIO transaction through bypass
```

Page table walker:

```text
PTE physical address
  -> cacheable-region check
  -> cache lookup/fill
```

PTW addresses should normally be DRAM addresses and therefore cacheable. A PTW
access to non-cacheable memory should be treated as an access fault or PTW
fault path, not as MMIO.

## Front-End Shape

Create a single internal request/response front-end. All core-side memory users
submit requests to this front-end:

```text
instruction fetch
load/store unit
page table walker
possibly monitor/debug helper paths later
```

The front-end performs:

- Arbitration.
- Region classification.
- Cache lookup.
- Miss fill.
- Write-through issue.
- Uncached MMIO issue.
- Registered response back to the core.

The core should see one response path:

```text
cached hit/fill data  \
                       -> response_data register -> core/PTW/fetch
uncached MMIO data    /
```

The MMIO mux should be inside the front-end before the registered response. Do
not write MMIO data into the cache array just to avoid a mux. That would create
architectural complexity and potential side effects for no real benefit.

## Arbitration

Use a single logical cache port initially. Arbitration priority should be
conservative:

```text
1. load/store unit
2. page table walker
3. instruction fetch
```

Rationale:

- Data-side accesses often unblock retirement.
- PTW accesses unblock both fetch and data translation.
- Fetch can wait without changing architectural state.

If PTW is invoked by fetch, ensure the fetch request is parked while PTW owns
the memory front-end. If PTW is invoked by a data access, the data access
remains the owning transaction until translation and the final memory access
complete.

## Cache Array Organization

Use eight 64-bit banks per line:

```text
bank0: line word offset 0, PA[5:3] == 0
bank1: line word offset 1, PA[5:3] == 1
bank2: line word offset 2, PA[5:3] == 2
bank3: line word offset 3, PA[5:3] == 3
bank4: line word offset 4, PA[5:3] == 4
bank5: line word offset 5, PA[5:3] == 5
bank6: line word offset 6, PA[5:3] == 6
bank7: line word offset 7, PA[5:3] == 7
```

Benefits:

- A 64-bit load/fetch hit reads one selected bank.
- A 64-bit store updates one selected bank with byte enables.
- A line fill writes one bank per returned 64-bit beat.
- The hit path avoids a 512-bit data mux.
- The layout matches the current even/odd memory instinct, but generalizes it
  to all eight words in a 64-byte line.

Metadata:

```text
valid[line_index]
dirty[line_index]
tag[line_index]
```

The current prototype uses dirty metadata for the DRAM cache so stores can be
write-back.

For a 1 MiB cache with 34-bit physical addresses, approximate metadata is:

```text
line count = 16384
tag bits   = PA_BITS - index_bits - offset_bits
           = 34 - 14 - 6
           = 14 bits
valid      = 16384 bits
dirty      = 16384 bits
tag RAM    = 16384 * 14 bits
```

If physical addresses are treated as wider internally, the tag grows
accordingly. The metadata is still small relative to the data array.

## UltraRAM Considerations

UltraRAM is synchronous. Assume at least one registered read cycle. The design
should not depend on combinational cache data.

Likely hit sequence:

```text
cycle N:
  accept request
  compute index/tag/word select
  issue data-bank and tag read

cycle N+1:
  compare tag
  if hit, select bank output and register response
  if miss, start fill

cycle N+2:
  core consumes hit response
```

If UltraRAM inference or explicit primitives require another stage, add it. The
core already tolerates memory latency; the big win is avoiding DDR/MIG latency
on hits.

Synthesis should be checked early for:

- Whether data banks infer UltraRAM.
- Whether tag/valid infer BRAM/LUTRAM as expected.
- Placement pressure with 1 MiB.
- Timing of index/tag compare and bank select.
- Timing of the front-end arbiter.

Start with 1 MiB for the real target. Try 2 MiB after 1 MiB meets timing.

## Cached Load Behavior

For cached loads:

```text
1. Translate VA to PA if required.
2. Classify PA.
3. Read tag and selected data bank.
4. If valid/tag match, return selected bytes.
5. If miss, fill the 64-byte line from memory.
6. After fill, return the requested bytes.
```

Sub-word extraction and sign/zero extension can remain near the load/store
unit, or move into the front-end. The first implementation should minimize
behavioral churn: keep architectural load formatting where it is today if
practical.

Misaligned accesses need a dedicated follow-up review. The ideal behavior is no
extra penalty when all required bytes are already resident in cache, including
accesses that cross a 64-bit bank boundary or a 64-byte cache-line boundary.
Cross-page accesses require two translations and two permission checks before
any architectural state is updated.

The current cross-page Sv39 workaround traps such accesses as misaligned. That
is acceptable as a temporary correctness fix, but the long-term cache/front-end
should translate both pages, split the physical accesses as needed, and combine
or merge the result without partial stores or stale bytes.

## Cached Store Behavior

For cached stores:

```text
1. Translate VA to PA if required.
2. Classify PA.
3. If cache hit:
     update selected bank bytes using byte enables
     mark the line dirty
4. If cache miss and write-allocate:
     write back the victim first if it is valid and dirty
     fill line
     update selected bank bytes
     mark the line dirty
5. Complete store when required ordering is satisfied.
```

For now stores block until the cache hit/miss action is complete. Dirty victim
writeback writes the whole 64-byte line before the new line is filled. A write
buffer can be added later, but it will need explicit ordering around uncached
MMIO and FENCE.

## Write Buffer As A Second Step

After the blocking cache is working, add a small write buffer:

```text
entry fields:
  valid
  physical address
  data
  byte enables
  target cached DRAM/SRAM
```

Rules:

- Cached stores can retire after enqueue if the buffer is not full.
- Loads must check for conflicts with pending writes.
- MMIO must drain the write buffer first.
- FENCE must drain the write buffer.
- Before entering low-power/wait states, drain if architecturally needed.

Do not add the write buffer in the first cache commit. It will complicate
debugging because load/store ordering bugs look like memory corruption.

## Uncached/MMIO Behavior

Uncached accesses use the same front-end but bypass the cache arrays:

```text
uncached load:
  issue one read of original requested width/alignment
  register returned data into normal response register
  do not allocate
  do not update tags
  do not update data banks

uncached store:
  issue one write with original byte enables
  do not allocate
  do not update tags
  do not update data banks
```

Ordering:

- Do not reorder uncached accesses relative to each other.
- Initially, do not reorder uncached accesses relative to cached stores.
- If a write buffer exists, drain it before uncached MMIO.
- FENCE drains all pending cached writes and all MMIO.

Instruction fetch:

- If translated/final PA is uncached, raise an instruction access fault.
- Do not provide an uncached instruction-fetch path.

This avoids accidental execution from UART/PLIC/CLINT/SPI/GPIO.

## Fill Behavior

On a cache miss:

```text
1. Compute line base = PA with PA[5:0] cleared.
2. Request 8 x 64-bit beats from backing memory.
3. Write each beat into bank0..bank7.
4. Write tag.
5. Set valid.
6. Return the originally requested word/bytes.
```

For direct-mapped write-back, a valid dirty victim must be written back before
the new line is filled. A valid clean line can be overwritten directly.

Important detail: update valid last, after all data banks and tag are written.
If reset or exception machinery can observe partially filled lines, the line
must not appear valid until fill is complete.

## Memory Backing Interface

The cache front-end needs two lower-level target paths:

```text
cached backing path:
  SRAM and DRAM line fills
  dirty cache-line writebacks

uncached bypass path:
  MMIO reads/writes
```

The current implementation has separate cases for:

- local SRAM fetch/load/store
- DRAM fetch/load/store
- PTW SRAM/DRAM reads
- UART
- CLINT
- PLIC
- SPI/GPIO

The cache plan should consolidate these behind:

```text
memfe_req_valid
memfe_req_ready
memfe_req_kind       fetch/load/store/ptw
memfe_req_pa
memfe_req_size       byte/half/word/dword
memfe_req_wdata
memfe_req_wstrb

memfe_resp_valid
memfe_resp_data
memfe_resp_fault
```

The exact signal names can follow existing local style. The important point is
that fetch, LSU, and PTW stop reaching directly into SRAM/DRAM/MMIO cases.

`mem0` and `mem1` are specifically part of this cleanup. They should not remain
special CPU-visible memories that bypass the cache. Either keep them as the
local SRAM backing store behind the cached-memory path, or replace them with the
same lower-level backing interface used by DRAM. In both cases, the core-side
load/store/fetch/PTW logic should see only the memory front-end.

## Page Table Walker Integration

The PTW should become a normal cache client.

Benefits:

- Linux page-table accesses are hot and should hit frequently.
- Stores to PTEs by the kernel and later PTW reads naturally see the same
  unified cache.
- This reduces the immediate pressure to add a TLB.

Correctness rules:

- PTW only performs physical reads.
- PTW reads must not go to MMIO.
- PTW faults must preserve current exception behavior.
- If a PTE spans a cache line due to misalignment, that is already invalid for
  RISC-V page tables and should not require special support.

## Instruction Fetch Integration

Instruction fetch should always use the cache for cacheable physical memory.

Rules:

- Fetch VA is translated when paging is enabled.
- Fetch PA must be cacheable.
- Fetch miss fills the corresponding line.
- Fetch hit returns the selected 16-bit/32-bit instruction bytes as today.
- Fetch from uncached/MMIO PA raises instruction access fault.

Because the cache is unified, stores that modify code update the same cache
state used by fetch. `fence.i` should still flush any prefetched/decompressed
instruction state in the pipeline, but it does not need to invalidate a separate
I-cache in the first unified-cache design.

## FENCE And FENCE.I

Initial behavior:

```text
FENCE:
  wait until the cache front-end is idle
  wait until any active writeback transaction has completed
  if write buffer exists later, drain it

FENCE.I:
  flush/kill prefetched instruction state
  no cache invalidation needed for unified cache
```

If a split I/D cache is ever added later, `fence.i` becomes more complex. That
is one reason to keep the first cache unified.

Future split instruction/data caches must remain coherent from the architecture
point of view. Stores that modify executable memory must become visible to
later instruction fetches after `fence.i`; stores that update page tables must
be visible to PTW reads; and write-back dirty data must not leave the I-cache or
PTW observing stale memory. The split-cache design therefore needs either
targeted invalidation, snooping, or explicit cache-maintenance machinery.

## Exceptions And Faults

The cache front-end should report fault conditions without hiding their source.

Possible faults:

- Instruction fetch from uncached/MMIO region.
- Data access to unmapped physical region.
- PTW access to unmapped/non-memory physical region.
- Lower-level bus error from DRAM path.
- Unsupported access size/alignment if not already trapped earlier.

The existing exception machinery should continue to own architectural cause and
`tval` selection where practical. The front-end can return a generic fault plus
enough classification for the existing pipeline state to select the right trap.

## Reset And Initialization

At FPGA configuration / simulation start:

- Initialize cache metadata RAMs to zero, so every valid bit starts clear.
- Data banks do not need to be initialized if valid bits are clear.
- Backing SRAM/DRAM image loading should initialize backing memory, not cache
  contents.

On soft reset:

- Treat reset like an interrupt: latch the request, then take it only when the
  CPU/cache/AXI state machine is in its home state.
- Do not reset the DRAM controller.
- Do not flush cache tags or data.
- Do not reinitialize backing memory.
- Reset only architectural CPU/control state after the memory path is idle.

This is valid because the cache is unified, physical, and coherent with the
core's own stores. A cached line remains a valid copy of physical memory across
soft reset as long as backing memory is preserved and there are no external
incoherent writers.

Any write-back cache added later should preserve this rule: soft reset should
wait until it can be taken cleanly, not discard dirty cache state.

For simulation:

- Existing SRAM and DRAM image loading should continue to initialize backing
  memory.
- The cache starts cold.
- The first accesses fill from initialized backing memory.

For FPGA monitor boot:

- SRAM monitor contents are in backing SRAM.
- First monitor fetches fill cache lines from SRAM.
- XMODEM uploads write through the cache to backing memory.
- Executing uploaded code should work because the cache is unified.

## Coherency Assumptions

The first cache assumes:

- No external DMA writes into cached DRAM behind the cache.
- The FPGA-side monitor/core is the only coherent CPU-side writer.
- MMIO is uncached.

If future SD/MMC or other devices gain DMA, either:

- DMA buffers must be in uncached memory, or
- explicit cache maintenance must be added, or
- the cache must snoop/invalidate, which is out of scope.

The current GPIO/SPI SD path is programmed I/O, so it does not create DMA
coherency problems.

## Direct-Mapped Conflict Risk

A direct-mapped cache is intentionally simple, but conflict misses can happen.
A large 1 MiB or 2 MiB cache should reduce this significantly for Linux.

Known possible conflict sources:

- Kernel text and hot data with matching index bits.
- Stack and page tables with matching index bits.
- Initramfs buffers and block I/O buffers.
- Monitor SRAM and DRAM if index/tag classification is wrong.

Correct tagging across SRAM and DRAM is essential. Do not rely on index alone.

If conflict misses are severe, the next design step should be two-way
skew-associativity rather than plain two-way associativity. Use a different
index hash per way so addresses that collide in one way are less likely to
collide in the other. This needs replacement-state metadata, hit selection, and
eviction policy work, and it must be measured against the timing cost of reading
and comparing two candidate lines.

## Timing Strategy

The cache should improve timing by centralizing memory decode and by registering
the response path.

Important timing choices:

- Synchronous cache arrays.
- Registered request fields.
- Registered response data.
- No combinational 512-bit line mux on the hit path.
- Select one 64-bit bank using PA[5:3].
- Keep MMIO response mux before the response register.
- Keep lower-level memory bus control out of the core pipeline.
- Avoid direct fanout from every pipeline state into every device decoder.

Expected hit latency can be more than one cycle. That is acceptable.

The current hardware path paid an additional cycle/stage to meet timing. A
specific follow-up experiment should determine what cache sizes and
organizations could recover that cycle:

```text
vary line count / index bits
compare direct-mapped versus two-way skew-associative
compare BRAM and UltraRAM mapping
record WNS, resource use, and cache hit latency
boot or run a representative workload when timing closes
```

The result should answer whether a smaller cache is actually faster overall, or
whether the extra hit stage is still the better tradeoff because it avoids DDR
miss latency more often.

## Suggested FSM

One possible blocking FSM:

```text
IDLE
  wait for arbitrated request
  capture request fields
  classify PA
  if uncached data access -> UNCACHED_REQ
  if uncached fetch/PTW   -> FAULT_RESP
  else                   -> TAG_READ

TAG_READ
  issue tag/data-bank read

TAG_CHECK
  compare valid/tag
  if hit load/fetch/PTW -> HIT_RESP
  if hit store          -> STORE_UPDATE
  if miss               -> MISS_REQ

HIT_RESP
  register response data/fault=0
  return to IDLE

STORE_UPDATE
  update selected bank bytes
  mark line dirty
  return STORE_RESP

WRITEBACK_WAIT
  wait for backing write completion
  -> STORE_RESP

STORE_RESP
  acknowledge store
  -> IDLE

MISS_REQ
  request line fill at line base
  beat_count = 0
  -> MISS_FILL

MISS_FILL
  receive/write bank[beat_count]
  increment beat_count
  after beat 7 -> MISS_COMMIT

MISS_COMMIT
  write tag
  set valid
  if original request was store -> STORE_UPDATE
  else                          -> HIT_RESP or RESP_AFTER_FILL

UNCACHED_REQ
  issue single original-width MMIO transaction
  -> UNCACHED_WAIT

UNCACHED_WAIT
  wait for data/ack
  -> UNCACHED_RESP

UNCACHED_RESP
  register response
  -> IDLE

FAULT_RESP
  register fault
  -> IDLE
```

The exact state split should match the current pipeline style, but these phases
should remain visible in the design.

## Bring-Up Stages

### Stage 0: Refactor Without Caching

Introduce the memory front-end and route requests through it, but make all
cacheable accesses behave like current backing-memory accesses.

Expected result:

- No performance improvement yet.
- Same tests should pass.
- This isolates interface and arbitration bugs before adding cache state.

### Stage 1: Uncached Bypass Only

Make MMIO use the front-end uncached path.

Expected result:

- UART, CLINT, PLIC, SPI, GPIO still work.
- Instruction fetch from MMIO faults.
- Linux still boots as before.

### Stage 2: Small Sim Cache

Enable a small direct-mapped cache in simulation only.

Suggested size:

```text
4 KiB or 8 KiB
64-byte lines
write-back
write-allocate
```

Expected result:

- Unit tests pass.
- ISA/cosim traces match.
- Linux simulation gets through known checkpoints.

### Stage 3: SRAM Cached

Enable caching for local SRAM.

Expected result:

- Monitor boots from cached SRAM.
- XMODEM upload works.
- Executing uploaded code works.
- No special fetch path remains for SRAM.

### Stage 4: DRAM Cached In Simulation

Enable caching for DRAM in simulation.

Expected result:

- Linux boots.
- PTW reads hit after warm-up.
- Existing SD monitor commands still work.
- Existing memory corruption tests do not regress.

### Stage 5: FPGA 1 MiB UltraRAM Cache

Synthesize and implement with a 1 MiB cache.

Check:

- UltraRAM inference/usage.
- Timing WNS.
- Monitor boot.
- XMODEM upload.
- Linux boot to login.
- SD read/write.
- Large SD read hash stability.
- Ubuntu boot.

### Stage 6: FPGA 2 MiB Cache Experiment

Only after 1 MiB works and meets timing:

- Increase line count to 32768.
- Re-run synthesis/implementation.
- Compare timing, utilization, boot speed, SD hash tests.

### Stage 7: Complete Unified Memory Front-End

After the DRAM cache is stable on hardware, remove the remaining separate
SRAM/DRAM/fetch/load/store/PTW memory paths. `mem0`/`mem1` should become
cacheable backing memory or disappear behind the same lower-level interface as
DRAM.

Expected result:

- No direct CPU-side access to `mem0`/`mem1`.
- SRAM and DRAM both use the same cacheability and miss/fill machinery.
- PTW, fetch, and data accesses share one response path.
- Distributed RAM inference is limited to genuinely small structures such as
  FIFOs, not large architectural memories.

### Stage 8: Store Policy Cleanup

Replace the debug-first write-through/invalidate behavior with write-back
store allocation.

Expected result:

- Store hits update cached bytes and mark the line dirty.
- Store misses write-allocate.
- Dirty victims are written back before replacement.
- Read-after-write to the same line hits correctly.
- Replacement and write pressure tests do not corrupt backing memory.

### Stage 9: Associativity And Timing Experiments

Compare direct-mapped, smaller direct-mapped, and two-way skew-associative
variants.

Expected result:

- A measured cache size/timing table.
- A measured miss-rate or workload-runtime table.
- A decision on whether the extra timing stage should stay.

## Test Plan

Before each commit that changes behavior:

```text
cd src
make testall
```

Additional focused tests:

- Monitor prompt from SRAM.
- XMODEM upload of a small file.
- Execute uploaded code from DRAM.
- Linux boot to login.
- SD probe from monitor.
- SD sector read from monitor.
- SD multi-sector load from monitor.
- Linux mount/touch/sync on SD.
- Repeated hash of the same SD region.
- PTW-heavy Linux boot with trace sampling.

Cache-specific tests to add:

- Load hit after fill.
- Store hit updates cached data.
- Store hit does not invalidate the line.
- Store miss write-allocates.
- Dirty victim writeback preserves all eight 64-bit beats.
- Byte/half/word/dword stores update only selected bytes.
- Fill crossing multiple banks.
- Direct-mapped replacement with different tags.
- SRAM and DRAM addresses with same index do not alias.
- `mem0`/`mem1` accesses go through the same cache/front-end as DRAM.
- MMIO read does not allocate.
- MMIO write does not allocate.
- Instruction fetch from MMIO faults.
- PTW read through cache.
- FENCE waits for pending cache activity.
- FENCE.I flushes prefetched instruction state.
- Misaligned load/store within one cache line.
- Misaligned load/store crossing two cache lines, with both lines already hot.
- Misaligned load/store crossing two pages, with both translations valid.
- Cross-page store fault cases do not partially corrupt either page.
- High Sv39 user addresses near the canonical boundary.

## Debug Instrumentation

Add optional counters:

```text
icache/fetch requests
data load requests
data store requests
ptw requests
cache hits
cache misses
dirty writebacks
uncached reads
uncached writes
fills
replacements
faults
```

Expose them either through simulation logging or a monitor command. The monitor
already has a precedent for useful hardware diagnostics.

Optional trace messages:

```text
CACHE hit/miss pa=... index=... tag=... client=...
CACHE fill pa=... beat=...
CACHE uncached pa=... size=... write=...
CACHE fault pa=... client=...
```

Keep these behind synthesis/simulation guards so they do not affect FPGA timing.

## Open Questions

- What exact physical address width should tags store on FPGA?
- Should SRAM and DRAM cacheability be controlled by parameters or fixed
  constants?
- Should local SRAM remain a separate backing RAM, or eventually be folded into
  the same lower-level backing path as DRAM?
- How much hit latency is acceptable before fetch becomes the next bottleneck?
- Does UltraRAM inference work cleanly for 8 independent 64-bit banks at 1 MiB?
- Does the direct-mapped 1 MiB cache meet timing with the current CVFPU
  integration?
- Should cache counters be memory-mapped, CSR-like, or monitor-only?
- Does the blocking write-back policy hold up under Linux write pressure?
- How much additional write-back machinery, such as a write buffer or explicit
  maintenance operation, is worth adding before split I/D caches?
- What index hashes work best for a two-way skew-associative cache without
  hurting timing?
- Can cross-line misaligned hits be serviced without a bubble, and what extra
  bank/tag read ports or prefetch state would that require?
- How should cross-page misaligned stores preserve precise behavior when the
  first page succeeds and the second page faults?

## Recommended Next Commit Boundaries

Future commits should keep correctness fixes separate from architectural cache
changes whenever possible.

Suggested boundaries:

- Cross-page Sv39 correctness fix and tests.
- `mem0`/`mem1` front-end unification with behavior-preserving tests.
- Write-back stress tests and any required correctness fixes.
- Misaligned hot-hit behavior improvements.
- Two-way skew-associative experiment.
- Split coherent I/D cache experiment.

Each commit should have a simulation check and, for cache/timing changes, a
hardware-oriented test note describing whether Vivado implementation was run or
intentionally deferred.
