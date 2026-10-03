# The data cache end state: an alias-free VIPT L1 and a 1.5 MiB L2

Status: direction decided (Tommy, 2026-10-03). Nothing below is built. This plan supersedes the
VHPR end state of the D$ plan (`PLAN-2026-09-25-dcache-vhpr.md`: increment 5c, phase 3 / increment
6b, decisions 1 and 5) and step 4 of the lanes plan. The memory path, the MSHRs, the write-back
buffer, the store port, NC, the CBOs and the coherent I$ that the D$ plan built all stay.

## Decision (Tommy, 2026-10-03)

- **The D$ is a VIPT cache with page-sized ways: 128 KiB, 32 ways of 4 KiB, 64 sets.** The set
  index is VA[11:6], inside the page offset, so it equals PA[11:6]: no synonyms, no epochs, no
  virtual stamps, no refusals for a stale stamp. The dTLB is read in parallel with the banks and a
  hit is a physical-tag compare.
- **The L2 is 1.5 MiB of UltraRAM** in the memory controller's 333 MHz clock domain.
- **Either may be scaled down** (fewer L1 ways, fewer L2 rows) only if timing or congestion forces
  it after real work, never for a measured IPC delta.
- The I$ is not part of this decision; this plan changes nothing above its memory port.

## Why

**VHPR as built lost to its PIPT base.** The 4b lockstep against the PIPT-skew D$ at the same tip
(retires, 60 M / 300 M):

| variant | 60 M | 300 M |
|---|---|---|
| VHPR, synonyms dropped and refetched | -2.20% | -10.55% |
| VHPR, synonyms moved inside the L1 | -1.46% | -5.68% |
| the dTLB read beside the virtual hit | -1.34% | -8.76% |
| the same, way 1 physical | -0.31% | -2.81% |

The losses are structural: refusals, synonym traffic between colours, and epoch scans.

**An alias-free VIPT loses nothing to VHPR in the L1, and the L2 absorbs the difference to a skew.**
Simmerv's whole-GB5 sampled model (172 windows, `wip/wset`), L1 misses per 1000 instructions, and
DRAM reads behind a physically indexed L2 that sees the L1's fetches:

| L1 | L1 misses: suite / without ML / ML | DRAM behind 512 KiB / 1 MiB / 2 MiB |
|---|---|---|
| 2-way, 64 KiB ways, straight index | 8.36 / 5.51 / 70.9 | 5.86 / 2.75 / 2.26 |
| PIPT, way 1 skewed (ships today) | 5.43 / 5.27 / 8.92 | 3.17 / 2.59 / 2.26 |
| VIPT 128 KiB, 32 ways | 8.17 / 5.30 / 70.9 | 5.83 / 2.75 / 2.26 |
| VIPT 64 KiB, 16 ways | 9.12 / 6.20 / 73.2 | 5.83 / 2.75 / 2.26 |
| VIPT 32 KiB, 8 ways | 11.55 / 8.29 / 82.9 | 5.84 / 2.75 / 2.26 |

- Outside Machine Learning, VIPT at 32 ways matches the skew (5.30 against 5.27).
- ML's SGEMM column walk sits in one or two sets at any index taken from bits below the page,
  however many ways: 70.9. Only a skew fixes it in the L1. Behind a 1 MiB L2 it costs 4.8 DRAM reads
  per 1000 instructions against 1.1 for the skew, and the rest are L2 hits.
- Behind any L2 of 1 MiB or more, DRAM traffic is the same for every L1 above.

**The hit path closes, and a small first-level dTLB takes it past 300 MHz.** Out of context at 6 ns
(`wip/l2-spike`, `smolrv64_vipt_spike.v`: a direct-mapped LUTRAM dTLB, the per-way tag LUTRAMs, the
compare into a registered one-hot hit, the select a cycle later):

| ways | dTLB entries | set index | Fmax | worst path |
|---|---|---|---|---|
| 8 | 2048 | one register | 280 MHz | dTLB read (7 levels) into the compare |
| 16 | 2048 | one register | 282 MHz | the same |
| 32 | 2048 | one register | 246 MHz | the set index into the 32 tag arrays: 928 loads, 3.05 ns of route |
| 32 | 2048 | 8 copies | 240 MHz | dTLB read into the compare, 4.14 ns |
| 16 | 64 | one register | 368 MHz | the data select |
| 32 | 64 | one register | 267 MHz | the set index's fanout |
| 32 | 64 | 8 copies | 330 MHz | the PPN into the 32 compares (32 loads) |

So the 32-way geometry costs nothing once the set index is replicated; what limits the clock is
the 2048-entry dTLB's read.

**The URAM fits.** 48 URAM288 out of context: +2.8 ns at 166.67 MHz, +0.1 ns at 333 MHz. Placed in
the full chip with the core, the default directive went from -0.026 to -0.120 ns and Explore from
+0.020 to -0.030 ns, and no URAM path is in the timing census.

## The L1

### Arrays

| array | shape | holds |
|---|---|---|
| data | 32 RAMB36 at 512 x 72, simple dual port | one 64-bit chunk per (set, chunk) per array, laid out diagonally (below) |
| tag | LUTRAM, 32 arrays of 64 x (PA tag + valid + dirty), one per way, read at VA[11:6] from a replicated set-index register | the line's PA[35:12] |
| store tag copy | the same, a second copy | the committed store's lookup in the same cycle as a load's |
| replacement | per set | chosen by the model (step 0) |

The block-RAM count is today's: 128 KiB of data is 32 RAMB36 either way.

**The data layout is diagonal.** Chunk `c` (0-7) of way `w` lives in array `(w + c) mod 32`, at
address `{set, c}`. Then:

- a load reads `{set, c}` from all 32 arrays at one shared address, and gets chunk `c` of all 32
  ways, rotated by `c`;
- a line's eight chunks sit in eight different arrays, so a 128-bit fill beat writes two arrays in
  one cycle, and a whole line could write in one;
- a store writes one array.

The rotation is applied to the one-hot hit vector (32 bits), not to the 2048 bits of data.

### The hit pipeline

- **T:** the door accepts a load (registered accept, as today). Its VA reads, in parallel:
  - the 32 data arrays at `{VA[11:6], chunk}`;
  - the 32 tag arrays at VA[11:6];
  - the dTLB, by VPN.

  The dTLB's PPN is compared with the 32 tags; the one-hot hit, the permission check and the
  fault are registered.
- **T+1:** the arrays' outputs are selected by the rotated one-hot hit (an AND-OR of 32 x 64 bits)
  and the response is registered. The lanes' write-slot reservation happens here: the tag compare
  is known one cycle before the data.
- **T+2:** the response is visible, tagged.

A dTLB miss is not a cache miss: the load is refused, the walker (in the queues since 5b) fills the
dTLB, and the load asks again. A tag miss with a dTLB hit is a miss with its PA in hand, into the
MSHRs as today.

**Stores** carry their PA from translation into the store queue. At commit a store reads the store
tag copy at PA[11:6], compares its PA, and writes its chunk into the one array that holds it. No
dTLB read.

### Everything addressed by PA

The physical index is the virtual index, so every request that has only a PA is a normal lookup:

- the page-table walker's reads;
- the CBOs (`clean`, `flush`, `inval`, `zero`);
- the MSHRs' merge and the write-back buffer's match;
- a future DMA snoop.

`sfence.vma` and `satp` writes touch only the dTLB. Nothing in the L1 depends on the address space.

### What goes from `rv_dcache`

The virtual stamps, the epochs and their wrap scan, the per-(way, colour) physical tags, the
skewed way 1, the VA-alone asks and their refusals, and synonym handling of every kind.

### What stays from `rv_dcache`

The MSHRs and their waiter lists and store-merge buffers, the write-back buffer (demand reads
first), critical-chunk-first fills with early restart, NC without allocation, the CBO completion
on write done, the store port's one store per cycle, the integrity log and the Zihpm events.

## The L2

- **Geometry.** 24,576 lines of 64 bytes: 48 URAM288, eight wide (512 data bits, the spare 64 for
  ECC or tag bits later) by six deep in one cascade. A line is read or written whole in one
  access. Associativity and index (12 ways x 2048 sets, 24 x 1024, a hashed index or not) come
  from the model (step 0).
- **Tags.** Block RAM, all of a set's ways read at once.
- **Place and clock.** Behind the core's memory port, in the memory controller's clock domain
  (333 MHz), between the asynchronous FIFOs and the AXI master. The L1s see it only as a faster
  memory: the port is already tagged, and its responses may already return out of order.
- **Policy.** Physically indexed and tagged, write-back, filled by both L1s' misses and L1
  write-backs. Not inclusive: nothing above it needs a reverse directory.
- **Non-blocking.** It takes the D$'s and the I$'s outstanding misses and sends its own misses
  through the AXI master's `NOUT`. An L2 hit answers behind an older L2 miss, matched by its id.
- **Coherence with DMA.** DMA reaches DRAM below the L2, so software coherence must reach through
  it: a CBO `clean`/`flush` cleans or flushes the L2's copy too, and `wr_cpl` still means the data
  is in DRAM. NC accesses bypass both levels. The virtio rings are NC today, so the protocol does
  not change; what changes is that a CBO has a second level to push through.

## Order

Every step is measured or modelled first, then gated by riscv-tests, the unit benches, lint, the
60 M / 300 M Linux lockstep, the glibc and sysd lockstep, a build, the board and GB5.

0. **Park VHPR.** Commit 4a, 4b and the in-L1 synonym move on their branches as the record.
   **Model** in Simmerv (`wip/wset`):
   - the L2 at 1.5 MiB, at 12 and 24 ways, with a straight and a hashed index;
   - the L2 seeing the I$'s fills and both L1s' write-backs, not only the D$'s fetches;
   - the 32-way L1's replacement: the model's true LRU against tree pseudo-LRU and
     not-recently-used.
1. **The L2, below today's caches.** The D$ and the I$ do not change. Its own bench first, then the
   SoC. The lockstep needs the L2 under the testbench's memory model, so the bench gains the second
   clock and the crossing it sits behind on the board.
2. **The VIPT geometry, with translation where it is.** `rv_dcache` at 32 ways of 4 KiB, diagonal
   data, the 32 tag compares and the chosen replacement; the skewed way 1 goes. The core still
   translates before the D$ and the D$ is handed a PA, which is exact because the index is inside
   the page offset. This step measures the hit rate and the 32-way compare and select in the full
   chip at 166.67 MHz, apart from the cycle.
3. **The dTLB in the access cycle.** The D$ takes the VA and owns the dTLB's lookup; a dTLB miss
   refuses, the walker fills, the load asks again. The LSU/LQ half of 4b (`wip/dc-5c-4b`: VA-first
   asks, refusal, the walk only after a refusal, the re-ask) is the starting point. M's translate
   pass for loads goes, and with it the cycle. It lands with or before the lanes' memory unit
   (lanes step 5.2), whichever is first.

## Open

- **Where a store translates** once M is gone: a second dTLB read port (LUTRAM read ports are
  copies), or the load port's idle cycles.
- **What store-to-load ordering compares**, VA or PA, once loads carry only a VA until the D$
  answers.
- **A small first-level dTLB** in the hit path, the 2048 entries behind it, if the clock goes past
  about 240 MHz: at 64 entries the 32-way hit path runs at 330 MHz out of context. It joins the
  future TLB plan.
- **Fills straight into the L1 from the 333 MHz domain** (a block RAM's two ports may sit in two
  clock domains). The data arrays' write port also takes the stores, which are in the core's
  clock, so this needs either the stores on the read port's cycles or a different split of the
  ports.
- **The L2's hit latency** in core cycles, through both crossings, from step 1's bench.
