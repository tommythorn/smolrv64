# smolrv64 OOO2 / L1 Cache Architecture Context

> Handoff from an extended architecture discussion. Purpose: give Codex enough context to continue the reasoning without reconstructing it from scratch.

## 1. Repository / source files

Primary current cache implementation:
- `ooo2/rv_cache.v`
- https://github.com/tommythorn/smolrv64/blob/main/ooo2/rv_cache.v

Related architectural idea:
- `docs/VHPR.md`
- https://github.com/tommythorn/smolrv64/blob/main/docs/VHPR.md

VHPR has already been implemented and validated, but is not currently used in OOO2. This is the author's second OoO design.

## 2. Central problem

The current L1 cache is the critical path in the entire core.

This is surprising given the complexity elsewhere in the OoO core, but the cache hit path currently limits timing closure.

The goal is not merely to simplify RTL. The important constraints are:
- improve L1 timing substantially
- do not casually add load-use latency
- preserve the benefits of the current skewed two-way organization

An extra load-use cycle can matter on real applications and benchmarks, even with OoO execution.

## 3. Critical cache-geometry correction

`LINEB` in `rv_cache.v` is measured in **bits**, not bytes.

The code derives:
- `WORDB = LINEB / 8`

Default parameters:
- `PAW = 34`
- `SIZE_KB = 128`
- `WAYS = 2`
- `LINEB = 512` bits = **64 bytes**
- `RDW = 64`
- `WDW = 64`
- `OFFB = 6`
- `WRITABLE = 1`
- `WRTHRU = 0`
- `PREFETCH = 0`

Therefore:
- capacity = 128 KiB
- line size = 64 B
- total lines = 2048
- sets = 1024
- index bits = 10
- line-offset bits = 6
- default tag bits = `34 - 10 - 6 = 18`

Do not interpret `LINEB=512` as a 512-byte line.

## 4. Current cache architecture

The cache is a:
> Unified skewed-2-way PIPT L1 cache.

It can act as:
- I$ (`WRITABLE=0`, fill-only)
- D$ (`WRITABLE=1`)
- write-back or write-through depending on configuration

It is physically indexed / physically tagged in the current OOO2 implementation.

### Skewed 2-way indexing

Way 0 uses the ordinary base index.

Way 1 uses:
`way1_index = base_index(address) XOR low tag bits`

This is important and should not be discarded casually.

The skew allows useful conflict behavior with only two ways. Replacing it with a conventional higher-way cache would increase associativity/muxing/comparison complexity and loses an important advantage.

For the default configuration:
- 1024 sets
- 10-bit base index
- way 1 XORs with 10 low tag bits

The skew is address-derived, so there is no fundamental circular dependency.

## 5. Data organization

The data array is split into even and odd banks.

Default read width:
- 64 bits = 8 bytes

A 64-byte line therefore has:
- 8 x 64-bit chunks
- 4 even chunks
- 4 odd chunks

There are effectively `2 * WAYS` BRAMs:
- way 0 even
- way 0 odd
- way 1 even
- way 1 odd

The RAMs are synchronous.

This organization supports arbitrary 64-bit loads efficiently because an access spans at most two adjacent 64-bit chunks, and adjacent chunks live in opposite banks.

## 6. Clever unaligned-load mechanism

A major source of current complexity is deliberate:

> The design was engineered to make most unaligned loads effectively penalty-free.

Conceptually:
```
chunk[n]     chunk[n+1]
     \         /
      128-bit window
            |
       variable shift
            |
        requested 64b
```

The RTL has concepts such as:
- `wlo`
- `whi`
- `win = {whi,wlo}`
- `win_sh`
- `fast_sh`

`fast_sh` is especially relevant to timing because it provides fast read-hit delivery directly from live BRAM outputs.

This machinery is elegant, but may now be optimizing the wrong common case.

## 7. Workload realization

The original motivation for penalty-free unaligned loads was sound. Some applications care about them.

However, after months of use, the observed workload distribution is:

> Quasi-totality of programs use aligned accesses.

This suggests a better tradeoff:
- make aligned loads extremely fast
- allow unaligned loads to take a more expensive path
- preserve skewed two-way associativity
- avoid adding a cycle to the common aligned load-use case

The goal is not to make unaligned accesses arbitrarily bad. It is to stop uncommon unaligned steering from determining the timing of the overwhelmingly common aligned path.

## 8. Load-use latency is a hard constraint

Do not assume OoO makes an extra cycle harmless.

A dependent load-use chain can expose that cycle, and some benchmarks are sensitive to it.

Therefore, the preferred optimization is structural rather than simply:
> "pipeline the cache and accept another cycle."

A desired conceptual split is:

### Aligned
```
address
  |
tag/index
  |
selected BRAM word
  |
load result
```

### Unaligned
```
address
  |
tag/index
  |
two-word extraction
  |
byte steering / shifting
  |
load result
```

The unaligned machinery must ideally be physically off the aligned critical path.

A naive RTL mux such as:
```
fast = selected_word
slow = barrel_shift(window)
result = aligned ? fast : slow
```
may still let the slow network hurt timing depending on synthesis/routing. The goal is physical separation, not merely textual separation.

## 9. Three access classes

Distinguish:
1. naturally aligned, entirely within a line
2. unaligned, entirely within a line
3. cross-line access

The first should be the privileged fast path.

The second can plausibly be slower.

The third is inherently more complex.

Do not conflate unaligned with cross-line: many unaligned accesses remain inside a single 64-byte line.

## 10. Current lookup / miss architecture

The cache has two machines, explicitly described in the source as:
> "TWO machines now, not one"

There is:
- lookup pipeline `st`
- fill machine `fst`

This allows plain cached reads to overlap an outstanding fill: hit-under-miss behavior.

Only one miss/fill is tracked at a time using state such as:
- `f_v`
- `f_*`
- `f_line`
- `f_addr`
- `f_tag`
- `f_replay`

Conceptual miss flow:
1. detect miss
2. select victim
3. if dirty, write victim back to lower level
4. fetch missing 64-byte line
5. receive into `linebuf`
6. install
7. replay original request through normal lookup

Replay costs one normal pipeline pass and avoids a separate complicated completion mux.

## 11. Hit-path details

Request state includes:
- `r_is_wr`
- `r_uncached`
- CBO flags
- `r_addr`
- `r_tag`
- `r_wdata`
- `r_wmask`
- `r_off`
- `r_span`

Line addresses are conceptually:
```
line0 = {r_addr[PAW-1:OFFB], 0}
line1 = line0 + (1 << OFFB)
```

Per-way indices:
```
ci0 = way_idx(0, cur_line)
ci1 = way_idx(1, cur_line)
```

Hit is based on valid/tag comparison:
```
hit = hit0 | hit1
```

Victim metadata is per set. Because way 1 is skewed, its victim base index must be recovered using the inverse skew.

## 12. Useful alignment constants

For 64-byte lines and 64-bit chunks:
- line = 64 B
- chunk = 8 B
- 8 chunks/line
- line offset = 6 bits
- chunk index = upper 3 bits of the 6-bit line offset
- byte-within-chunk offset = lower 3 bits

A naturally aligned 64-bit load selects exactly one 64-bit chunk.

This is a strong reason to give aligned loads a direct BRAM-word path.

## 13. VHPR: major architectural opportunity

`docs/VHPR.md` describes:
> VHPR = virtually hit, physically reconciled.

The purpose is to remove translation from the ordinary L1 hit path while preserving physical correctness for:
- misses
- writeback
- coherence
- synonyms
- DMA
- maintenance

The design is explicitly motivated by FPGA timing closure.

Core philosophy:
- ordinary hits use virtual identity
- physical translation/reconciliation happens on miss/exception paths
- physical metadata remains available for correctness
- synonyms are reconciled before committing a new physical line

## 14. VHPR target geometry

The fixed initial VHPR target in the document is:
- 64 KiB I$
- 64 KiB D$
- 64-byte lines
- 2-way skew associative
- 512 indices per way
- 9 index bits
- 4 KiB pages

Example:
- virtual index = `VA[14:6]`
- line-within-page offset = `VA[11:6]`
- virtual color = `VA[14:12]`

This is different from the current default OOO2 cache geometry. Do not assume the VHPR target parameters directly match `rv_cache.v`.

## 15. VHPR hit semantics

Each line has metadata including roughly:
- valid
- dirty
- ASID
- virtual tag
- physical tag
- valid epoch
- cached permission/state information
- replacement/coherence state

Ordinary hit determination uses:
- valid
- epoch
- ASID
- virtual tag
- permissions

The physical tag is NOT needed on the ordinary hit path.

Miss/reconciliation uses:
- VA -> PA translation
- physical tag
- synonym search
- physical coherence state
- L2

## 16. VHPR synonym handling

A physical line can have multiple virtual aliases, so VHPR needs bounded reconciliation.

Documented search domain:
- 2 ways * 8 virtual colors = 16 candidates

Four physical tags can be compared per cycle, so the complete search takes four cycles.

This can naturally overlap a four-beat 64-byte fill.

Cases:
- no synonym -> fill/install
- clean synonym -> invalidate old synonym, then fill/migrate
- dirty synonym -> write dirty synonym to L2, invalidate, refill
- future optimization -> migrate dirty data directly

The important principle is:
> synonym complexity belongs on the miss path, not the ordinary hit path.

## 17. VHPR + skew

VHPR does not require abandoning skewing.

The document explicitly permits per-way virtual hashing/skewing provided the miss path can enumerate all same-offset candidates.

Conceptually:
```
hash0(VA, ASID)
hash1(VA, ASID)
```

The key is bounded enumeration for physical reconciliation.

Therefore:
> VHPR + skewed 2-way is a plausible combination.

Do not propose abandoning skew as the default solution.

## 18. VHPR + OOO

The combination is potentially powerful:

### VHPR
Removes TLB / physical reconciliation from ordinary L1 hit determination.

### Skew
Preserves useful conflict behavior with only two ways.

### OOO
Hides latency of less common long-latency cases when independent work exists.

This supports the strategy:
> Spend complexity on miss/reconciliation machinery; make the common hit path brutally simple.

## 19. VHPR is not automatically the solution

First identify what actually dominates the current timing path.

VHPR helps enormously if the critical path contains:
- TLB lookup
- physical translation
- physical tag dependency

But it may not solve a path dominated by:
- variable byte steering
- barrel shifting
- large muxes
- way-selection muxing
- BRAM routing
- fanout
- `fast_sh`
- `win_sh`

Therefore:
> Get the actual timing report before deciding which architectural change matters.

## 20. Promising combined architecture

The current hypothesis is:

### Ordinary aligned load
```
VA
 |
VHPR virtual index/tag
 |
skewed way selection
 |
selected 64-bit BRAM word
 |
result
```

No variable byte steering.
No physical-tag compare.
No TLB on ordinary hit.

### Unaligned intra-line
```
VA
 |
normal lookup
 |
two adjacent BRAM words
 |
128-bit window
 |
byte shift
 |
result
```

Potentially slower.

### Cross-line
Special/slow handling.

### Miss
Translation + physical reconciliation + synonym handling + fill.

The design becomes a hierarchy of paths whose complexity matches frequency.

## 21. Muxing warning

Do not assume separate RTL expressions guarantee timing separation.

For example:
```
fast = selected_word
slow = barrel_shift(window)
result = is_aligned ? fast : slow
```
may still synthesize/routе through the expensive network.

Potential techniques to investigate:
- structurally separate datapaths
- registered boundary for slow path
- separate BRAM output usage
- specialized aligned datapath
- early selection before expensive steering
- make aligned result the natural BRAM output
- synthesis-friendly structural/generate organization where appropriate

But do not add a cycle just for cleanliness.

## 22. Things not to do casually

### Do not abandon skew
The skew is a key reason two ways work well. Giving it up guarantees more conflicts in the cases it was designed to help.

### Do not automatically add a load-use cycle
OoO hides some latency but not all.

### Do not optimize arbitrary unaligned accesses at the expense of aligned accesses
Observed workload behavior argues for the opposite priority.

### Do not assume VHPR fixes everything
Measure the current critical path first.

### Do not turn the hit path into a general-purpose data-manipulation pipeline
The common path should be boring.

## 23. Existing features to preserve

The current cache contains:
- hit-under-miss
- fill replay
- write-back
- write-through option
- uncached accesses
- CBO/cache maintenance
- optional prefetch stream buffer
- request/response tags for OOO matching
- line-spanning access support
- skewed indexing
- even/odd BRAM banking
- unaligned extraction
- invalidation machinery

A redesign should preserve semantics unless there is a deliberate architectural reason not to.

## 24. OOO request tags

`rd_tag` is opaque and echoed as `rd_resp_tag`, allowing the OoO LSU to match returned cache data with outstanding requests.

`rd_ack` / `wr_ack` indicate request acceptance.

`rd_valid` indicates returned read data.

When discussing latency, explicitly distinguish:
- request accepted
- BRAM read issued
- hit known
- `rd_valid`
- architectural result availability
- dependent instruction execution

## 25. CBO / uncached behavior

The cache supports CBO operations and uncached accesses.

Relevant state includes:
- `cbo_req`
- `cbo_zero`
- `cbo_keep`
- `r_uncached`

Uncached accesses use flush-around semantics.

Any VHPR integration must preserve maintenance and uncached correctness, especially because physical identity and dirty state matter.

## 26. VHPR correctness model

L2 is the physical authority / coherence point.

Physical correctness comes from:
- physical tags
- synonym reconciliation
- physical writeback addresses
- physical probes
- single-copy enforcement
- physical L2 backing

Ordinary L1 hits should not pay for all of this.

Mental model:
```
FAST PATH:
    virtual identity / permission / residency

SLOW PATH:
    physical identity / translation / synonym / coherence
```

## 27. PTW, epochs, coherence

PTW accesses should use the physical L2 path rather than the virtual-hit L1 shortcut. Dirty L1 page-table lines must be reconciled first.

VHPR uses an epoch mechanism so ordinary hits need not perform full translation checks. The documented design uses a 2-bit epoch. Epoch rollover requires a full walk/writeback/invalidation before publishing the wrapped epoch.

Coherence/DMA probes use physical synonym search. I$ coherence uses FENCE.I semantics.

## 28. Investigation priority

Before editing RTL:

1. Find exact worst timing path.
2. Determine whether TLB/translation is involved.
3. Determine whether `fast_sh` / variable shifting is involved.
4. Determine whether way selection/skew is involved.
5. Determine whether BRAM routing/output is involved.
6. Prototype a direct aligned BRAM-word result.
7. Measure Fmax.
8. Separately measure VHPR timing benefit.
9. Combine VHPR + skew + direct aligned datapath.
10. Measure exact aligned load-use latency.

Do not guess at FPGA/tool-specific effects without checking the project.

## 29. Suggested experiments

### Experiment A — timing decomposition
Record:
- startpoint
- endpoint
- slack
- logic depth
- major LUT/mux structures
- BRAMs involved
- TLB involvement
- `fast_sh` / shift involvement
- way selection involvement

### Experiment B — aligned-only datapath
Temporarily model the aligned case so it gets exactly one BRAM word with no variable shift.

Keep associativity and skew unchanged.

Measure Fmax.

### Experiment C — VHPR
Use the already-validated VHPR implementation to remove translation from the ordinary hit path.

Measure independently.

### Experiment D — combined
VHPR + skewed 2-way + direct aligned BRAM word.

This is the most interesting architectural point to investigate.

## 30. Questions Codex should answer first

Inspect:
- exact current `ooo2/rv_cache.v`
- OOO2 LSU/cache interface
- VHPR implementation and validation code
- timing reports, if present
- target FPGA and synthesis tool
- actual I$/D$ configurations
- whether the worst path is I$ or D$
- exact measured load-use latency

Do not rewrite first. Diagnose first.

## 31. Measurement table for future changes

For every proposed architecture, compare:

| Metric | Current | Candidate |
|---|---:|---:|
| Fmax / worst slack | measure | measure |
| Aligned load latency | measure | measure |
| Unaligned intra-line latency | measure | measure |
| Cross-line latency | measure | measure |
| Miss behavior | measure | measure |
| LUT/FF/BRAM use | measure | measure |
| Correctness | pass/fail | pass/fail |
| Conflict behavior | baseline | compare |

The success criterion is not "the RTL is simpler."

It is:
> Common aligned L1 hits become faster without losing the properties that motivated the current cache.

## 32. Final architectural hypothesis

The cache may have become too clever in optimizing every access equally.

The emerging lesson is:
> **Do less on the common path.**

Keep:
- skew
- two ways
- hit-under-miss
- fill replay
- correctness machinery

Use VHPR to move physical complexity off ordinary hits.

Make aligned loads naturally map to one BRAM word.

Move byte steering out of the aligned critical path.

Allow rare unaligned accesses to be slower.

Do not add a load-use cycle unless measurement proves it necessary.

### Phrase to keep in mind

> **Fast path should be boring.**
