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
      cached: cache lookup/fill/store update/write-through
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
tag[line_index]
```

No dirty bit in the first implementation because stores are write-through.

For a 1 MiB cache with 34-bit physical addresses, approximate metadata is:

```text
line count = 16384
tag bits   = PA_BITS - index_bits - offset_bits
           = 34 - 14 - 6
           = 14 bits
valid      = 16384 bits
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

Misaligned loads should preserve current behavior. If the current core traps or
handles them specially, the cache should not silently change that behavior.

## Cached Store Behavior

For cached stores:

```text
1. Translate VA to PA if required.
2. Classify PA.
3. If cache hit:
     update selected bank bytes using byte enables
     issue write-through to backing memory
4. If cache miss and write-allocate:
     fill line
     update selected bank bytes
     issue write-through to backing memory
5. Complete store when required ordering is satisfied.
```

For the first version, stores may block until the write-through completes. This
is simple and correct. A write buffer can be added later.

The write-through path should write only the requested bytes/word to backing
memory, not the whole line.

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

For direct-mapped write-through, no eviction writeback is required. If a valid
line is replaced, simply overwrite it.

Important detail: update valid last, after all data banks and tag are written.
If reset or exception machinery can observe partially filled lines, the line
must not appear valid until fill is complete.

## Memory Backing Interface

The cache front-end needs two lower-level target paths:

```text
cached backing path:
  SRAM and DRAM line fills
  cached write-through stores

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
  wait until any write-through transaction has completed
  if write buffer exists later, drain it

FENCE.I:
  flush/kill prefetched instruction state
  no cache invalidation needed for unified cache
```

If a split I/D cache is ever added later, `fence.i` becomes more complex. That
is one reason to keep the first cache unified.

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

On reset:

- Clear all valid bits.
- Tags and data banks do not need to be cleared if valid bits are cleared.
- Any lower-level transaction state must reset to idle.
- Any write buffer added later must reset empty.

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

If conflict misses are severe, the next design step would be 2-way
associativity, but that should not be part of the first implementation.

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
  issue write-through
  if blocking stores -> WRITE_THROUGH_WAIT
  else               -> STORE_RESP

WRITE_THROUGH_WAIT
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
write-through
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

### Stage 6: FPGA 2 MiB Cache Experiment

Only after 1 MiB works and meets timing:

- Increase line count to 32768.
- Re-run synthesis/implementation.
- Compare timing, utilization, boot speed, SD hash tests.

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
- Store hit writes through to backing memory.
- Store miss write-allocates.
- Byte/half/word/dword stores update only selected bytes.
- Fill crossing multiple banks.
- Direct-mapped replacement with different tags.
- SRAM and DRAM addresses with same index do not alias.
- MMIO read does not allocate.
- MMIO write does not allocate.
- Instruction fetch from MMIO faults.
- PTW read through cache.
- FENCE waits for pending write-through.
- FENCE.I flushes prefetched instruction state.

## Debug Instrumentation

Add optional counters:

```text
icache/fetch requests
data load requests
data store requests
ptw requests
cache hits
cache misses
write-through writes
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

## Recommended First Commit Boundary

The safest first commit should only add the memory front-end skeleton and
region classifier, with behavior equivalent to today. It should not add cache
state yet.

That commit should make later cache work mechanical:

- Existing fetch path submits a front-end request.
- Existing LSU submits a front-end request.
- Existing PTW submits a front-end request.
- Front-end routes to current SRAM/DRAM/MMIO logic.
- Tests pass with no expected behavioral change.

After that, adding the cache array becomes a localized replacement of the
cacheable-memory branch inside the front-end.
