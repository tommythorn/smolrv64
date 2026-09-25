# The data cache rewrite: VHPR, non-blocking, end-state shape

Status: DESIGN, for review (2026-09-25). Nothing here is built.

## Why now

The D$ is `rv_cache` at `VIRT=0`: 64 KiB, 2-way skewed, physically indexed and tagged, write-back,
one MSHR. MEM-SIM (b5661cf6, 300 M Linux lockstep at IW=3, tip 0790196c) measured its cost:

| | |
|---|---|
| a second miss parked in the lookup pipeline behind the one fill (every request behind it waits too) | 20.2% of all cycles |
| the D$ busy with a fill | 28.4% of all cycles |
| mean fill occupancy | 66.7 cycles against a 28.75-cycle measured DDR read |
| store queue full (it drains through the same blocked D$) | 12.0% of all cycles |
| the in-order `u_iq_l` head not ready while a younger load is (upper bound for the queue-side work) | 7.8% |

From the RTL, a clean-victim fill occupies the D$ for DDR latency + 11 cycles:
- 1 cycle F_WB;
- 2 + 1 arbiter hops;
- `linebuf` capture;
- a 4-cycle install;
- then F_ANS answers from `linebuf`.

A dirty victim is written back first, serially, which adds about 30 cycles.

Every stage below the D$ is single-outstanding: `rv_l2_arbiter`, `ddr_line_cdc`'s 4-phase line
handshake, `ddr_line_axi` (one 8-beat burst; a write waits for B), and the platform's
`axi_two_master_arbiter`, which holds read ownership from AR to `rlast`. A cache with MSHRs above this
path would only queue at the arbiter.

Tommy (2026-09-25): rewrite the D$ now, ahead of the queue-side translate (C4b step 3), as its own
module like `rv_icache`, and make it right from the start: as much parallelism as we can. Increments may
stage verification, never the shape.

## Constraints

- **The Xilinx MIG returns read data in order.** Several reads may be outstanding; their data
  comes back in request order. Every transaction carries a tag regardless, so a reordering
  controller can replace the MIG without touching the cache, but nothing may depend on reordering.
- **Clock:** 166.67 MHz now; the 250 MHz review is queued. The hit path is a register-to-register
  pipeline with a parameterized depth, and no D$ signal enters the schedulers' ready logic
  (memory: the scheduler is the critical path).
- **RTL rules that bind this design:**
  - A1, always-on invariants;
  - B1, a response is matched by the requester's tag;
  - D5, drop a request on its ack;
  - D12, a predicate describes its own address;
  - D17, nothing that changes memory is speculative;
  - F4, no function RAM reads;
  - I6, nothing late on a BRAM address;
  - I8, the compare decides, never enables;
  - I10, one write statement per LUTRAM bank.
- **The SoC-facing contract stays** for the first swap:
  - `dmem_*`: tagged fast reads, one slow read, the registered write accept, `wr_cpl` for CBO/NC,
    `dmem_idle`;
  - the walker reads;
  - `inv_req`/`inv_busy` for fence.i;
  - the integrity log (bits 0-15);
  - the Zihpm events.

  The new module drops in behind that contract; the core changes later.

## Architecture

### Geometry and arrays

**128 KiB, 2 ways, 64-byte lines, 1024 sets indexed by VA[15:6]**: 16 colours (VA[15:12]), unskewed
so a physical line's candidates stay findable. All widths are parameters.

**Why 128 KiB (measured 2026-09-25):** the 300 M Linux lockstep at IW=3 (0790196c, today's PIPT
`rv_cache` with only `SIZE_KB` changed):

| configuration | retires | change | D$ misses |
|---|---|---|---|
| 64 KiB (today) | 85,894,028 | | 1.278 M |
| 128 KiB D$ | 89,349,163 | +4.02% | 1.036 M (-19%) |
| 128 KiB I$ | 85,550,054 | -0.40% | |
| both at 128 KiB | 88,198,468 | +2.68% | 1.027 M |

The I$ stays at 64 KiB: the boot's I$ stalls are not capacity misses.

**Placement is the known risk.** The last 128 KB D$ (`rv_cache`) spread over most of the die, and its
worst internal route was 5.7 ns, 82% of it wire; that is why `SIZE_KB` went to 64. So the new
module is floorplanned from the first build: the data banks in one block-RAM column pair, the tags
beside them, the hit path registered at the bank outputs.

| array | shape | holds |
|---|---|---|
| data | 4 BRAM banks (way × chunk parity), 1R1W, 4096 × 64 | one read port for lookups and the victim read-out, one write port for store writes and fill beats |
| virtual tag | LUTRAM, per way, 1024 deep | `vtag` = VA[38:16], `ep` (2 bits), `vvalid`, perms `{R, W, X, U, D}` (X for MXR loads) |
| physical tag | LUTRAM, **32 arrays of 64 × PTB**, one per (way, colour), all read at VA[11:6] | `ptag` = PA[35:12], `pvalid`, `dirty` |
| replacement | one bit per set | round-robin |

**The per-(way, colour) physical tag is the design's key structure.** The 32 lines a physical line
can live in (2 ways × 16 colours) share VA[11:6] = PA[11:6]. So one read at that row returns all 32
candidates, with no replicas and no walk. The 32 compares take one cycle, or two of 16 if timing
wants it (Tommy: the probe may take a few cycles; it is on the miss path only). Each
array has its own write enable (one write statement each, I10). A line is written by selecting its
(way, colour) array.

**Virtual stamp and physical line are separate state.** A line is physically valid (`pvalid`,
`dirty`, `ptag`, data) independently of whether a virtual stamp is live (`vvalid`, `vtag`, `ep`,
perms). An epoch wrap clears `vvalid` only. It needs no write-back, because the physical line is
still correct. This removes VHPR.md's "walk and write back every dirty line before epoch 0".

### Invariants (asserted, and in the integrity log)

1. At most one `pvalid` line per physical line (the probe enforces it at every allocation).
2. A live virtual stamp names a `pvalid` line: `vvalid → pvalid`.
3. A hit never needs translation: `vtag & ep & vvalid & perms-ok` is the whole hit.
4. Every response carries the tag its request brought (B1); every MSHR, write-back buffer entry and
   memory transaction has an id; no generation bits, no address matching.

### The hit pipeline

- **T:** the door accepts a request (a registered accept, register-decoded door, as today). The
  data banks and the virtual-tag arrays are read at VA[15:6].
- **T+1:**
  - compare `vtag`/`ep`/`vvalid` per way;
  - check the perms against priv/SUM/MXR, captured with the request;
  - select the way's chunk;
  - register the response.
- **T+2:** the response is visible, tagged, the same as today's 2-cycle path. At 250 MHz a stage
  is added between the bank read and the compare.

Two request streams share the door, **one load and one store per cycle**:

- **Loads** read the banks' read port.
- **Stores** (committed, from the senior SQ) look up the virtual tag in a second read of the
  LUTRAM arrays in the same cycle. A hit with W & D writes its masked chunk through the bank's
  write port at T+1.
  - A load to the same row as a store written in the same or previous cycle takes the write
    data through a one-entry bypass: no hold, no `fin_hazard` door closure.
  - The store drain goes from one per two cycles to one per cycle.
- **Fill beats** also use the write port.
  - They have priority over a store write; a store that loses retries the next cycle from its
    one-entry store stage.
  - The victim read-out uses the read port and has priority over a new load lookup, for the 4
    cycles it takes.

A virtual miss, or a failed permission check, goes to the miss path. It never holds the pipeline:
the request leaves into an MSHR or the retry queue, and the next request proceeds (hit-under-miss).

### The miss path: MSHRs

`NMSHR` (default 8, = `LQ_N`) entries, each holding:
- `{valid, state, line PA, VA set, reserved way, colour}`;
- a **waiter list**: the load tags and their chunk offsets;
- a **store-merge buffer**: 64 bytes + a 64-bit byte mask.

A miss:

1. **Translate.** The PA comes with the request in phase 1 (the core still translates, as for the
   I$). In phase 2 it comes from the D$'s translate port (the dTLB), when the load path stops
   translating.
2. **Probe, one or two cycles.** The 32 candidates at PA[11:6], compared with the PA's `ptag`:
   - **Match in the request's own set** (the common case after an epoch bump or a new mapping of
     the same page): re-stamp `vtag`/`ep`/perms (one cycle, the I$'s reconcile) and replay.
     This is not a memory access.
   - **Match in another colour (a synonym):** if clean, invalidate it; if dirty, move it to the
     write-back buffer and invalidate it. Then fill as a miss.
     Migrating the data in place, with no memory round trip, is an optimisation left out of the
     first version.
   - **No match:** allocate an MSHR.
3. **Secondary misses merge.** A request whose line PA matches a live MSHR joins it: a load is
   added to the waiter list; a store merges into the buffer. A request to a set whose reserved
   way is taken by another MSHR, or when all MSHRs are busy, goes to a small retry queue. It never
   parks in the pipeline.
4. **Victim: the demand read goes first; the write-back waits for it.**
   - The fill read is issued in the allocation cycle.
   - The reserved way's line, if `pvalid` and dirty, is read out in the next 4 cycles into the
     **write-back buffer** (`NWB`, default 2 lines), long before the first fill beat can land
     in those rows.
   - The buffered write-back is sent to memory only **after the demand fill has returned**.
     Reads have priority over write-backs all the way down the memory path. A write-back is
     sent when no demand read is waiting, or when the buffer is full and a new victim needs
     the slot.
   - With the MIG serving in order, a write-back sent first would put its whole transfer ahead
     of the read the load is waiting for. Today's D$ does exactly that: serially, about 30
     cycles per dirty miss.
   - A later miss to a line sitting in the write-back buffer is served from the buffer.
     A CBO or fence.i that must reach DRAM forces the buffer out first.
5. **Fill.** The read goes to memory as a critical-chunk-first WRAP burst starting at the waiting
   load's chunk.
   - Each 64-bit beat is written into its bank row as it arrives.
   - A waiter whose chunk arrives is answered from the beat itself (early restart): the first
     load sees data about DDR latency + 3 cycles after the miss instead of + 11.
   - Store-merge bytes override the beat data at install.
   - A store-only MSHR whose mask covers the whole line (`cbo.zero`, a streamed memset) installs
     without a memory read.
6. **Install.** The last beat writes the tags: the `ptag` array of (reserved way, colour), and the
   virtual stamp with the requester's epoch. Stamping with the requester's epoch means a fill
   issued before a mapping change can never make its line current (the I$'s rule).

The waiters are answered through the response port. When a hit and a fill answer collide, the hit
waits one cycle in a two-entry response queue. The LSU lands at most one load per cycle, so one
response port suffices.

### Paths that bypass the arrays

- **NC/IO (Svpbmt), no allocation.** An uncached read or write goes straight to the memory path
  as a single-beat transaction; a write carries its byte mask as AXI WSTRB. This replaces today's
  allocate / flush-around / read-modify-write-the-line. NC accesses stay in order with each other
  through one NC queue. Devices below 0x8000_0000 still never reach the D$.
- **CBOs, by physical address.** They issue only at the ROB head (D17), so they are never
  speculative. The physical probe finds the line from the PA alone.
  - `clean`/`flush`: a dirty line goes to the write-back buffer; `wr_cpl` is raised when the
    memory write's B response returns, so the data is in DRAM before a later doorbell.
  - `inval`: implemented as flush (as today).
  - `zero`: a full-mask store-merge MSHR; it installs without a memory read.
- **Page-table walker reads, by physical address.** Probe the 32 candidates and read the matching
  line's chunk, so a dirty PTE is always seen. On a miss, fill into the PA's own colour, then
  answer. Walker requests keep their fixed tags.

### Epochs, fence.i, DMA

- **Epochs.** An `sfence.vma` or a satp change bumps the D$ epoch (today the D$'s `ep_bump` is
  tied 0 because the D$ is PIPT). A wrap clears `vvalid` over a 512-cycle scan with the door shut.
  No write-back is needed; physical lines survive.
- **fence.i becomes a pipeline-only operation: the I$ is made coherent with stores** (Tommy,
  2026-09-25). The I$ gets the D$'s per-(way, colour) physical-tag layout (16 arrays at 64 KiB). A store probes it by PA (one
  cycle), filtered by a small set of physical pages the I$ has fetched from, and a match
  invalidates that I$ line.
  - The filter is one bit per hashed physical page (1024 bits, LUTRAM). An I$ fill sets its
    page's bit; a store probes only when its page's bit is set. A stale or aliased bit only costs
    an extra probe, never a missed one, so bits are cleared only when the whole I$ is invalidated.
    There is no compare on the store path and no overflow case.
  - Stores to code pages are rare, so a probe is rare.
  - There is no inclusion and no D$ capacity spent on code.
  - Then fence.i only flushes the fetch ring and predictions.

  Until that increment lands, fence.i keeps today's model: a clean of the D$ (write back every
  dirty line, now through the write-back buffer), then an invalidation of the I$.
- **DMA stays software-coherent**: the virtio rings are NC, and Zicbom maintains the rest, as
  today. Hardware snooping would use the same physical probe, but is out of scope unless the DMA
  cosim (B6) shows it is needed.

## The memory path below the D$ (built first)

Every piece is single-outstanding today; all of them change:

1. **`rv_l2_arbiter` becomes `rv_mem_arbiter`.** It is not an L2. Reads have priority over
   write-backs. It gets:
   - tagged requests, several outstanding;
   - reads, writes, and masked single-beat NC writes;
   - beat-streaming read responses `{tag, beat index, last, data}`;
   - write responses `{tag}`;
   - requesters: D$ (fills, write-backs, NC), I$ (fills, prefetch).
2. **`ddr_line_cdc` → asynchronous FIFOs**: request `{tag, we, addr, burst, mask}`, write beats,
   read beats `{tag, last, data}`, write responses `{tag}`. This removes the 4-phase handshake's
   per-line round trips and streams beats at one per cycle.
3. **`ddr_line_axi` → a pipelined AXI master:**
   - AR and AW issue as their FIFOs fill;
   - up to `NOUT` (default 8) outstanding on one AXI ID, with an in-order tag FIFO naming each
     returning burst;
   - WRAP bursts for fills, INCR for write-backs, single beats with WSTRB for NC;
   - no wait for B before the next write.
4. **`axi_two_master_arbiter` (the platform's core vs DMA):** it holds ownership per transaction,
   not until `rlast`/B. An owner FIFO per channel routes the in-order responses back.
5. **The testbench memory model:**
   - a queue of `NOUT` outstanding requests, each with its own draw from the measured latency
     shape;
   - served in order (the MIG's rule);
   - beats streamed;
   - a knob to reorder across ids, so the design is shown not to depend on order where it must
     not.

## Increments

Each ends with the full gate set: lint, riscv-tests, the 60 M and 300 M lockstep, memrand (the
shadow op included), unit benches, a build at IW=3, the board gate, then GB5.

1. **Memory path, transparent above.** The new arbiter, CDC and bridge, with the old `rv_cache`
   and `rv_icache` as requesters. They issue one at a time, so behaviour is the same. The
   lockstep retire count moves only by the latency change (expected: shorter).
   - New benches: `tb_mem_path` (random tagged traffic against the in-order and reordering
     models).
   - Board: the DDR stress run and GB5.
2. **`rv_dcache`, complete, behind today's contract.** The whole module (MSHRs, write-back buffer,
   probe, store port, NC path, CBO, walker path, epochs) swapped in for `rv_cache` at the SoC. The
   core is unchanged: it presents VA and PA.
   - New bench: `tb_ooo2_dcache2`. A golden byte image plus a reference translation, with random
     loads, stores, CBOs, NC, DMA writes, remaps with epoch bumps, synonyms (two VAs → one PA, dirty
     in one colour, accessed from another), and walker reads. Run at several latencies, in-order
     and reordering memory, every answer checked, every invariant live.
   - memrand gains a synonym-alias op, which it already half has: three aliases of one region.
3. **The store port.** The SQ drains one store per cycle: the LSU's S_ST/`take_next` chain
   becomes a stream.
4. **The coherent I$:** a filtered physical probe of the I$ on every store; fence.i becomes pipeline-only.
5. **Phase 2, with the queue-side translate:** the load path stops translating; the D$ translates
   on a miss through its translate port. This is where the dTLB leaves the load's hit path; it
   folds into C4b step 3.

## Verification specific to this design

- The single-copy invariant, checked on every install and every re-stamp (`pvalid` count per
  physical line ≤ 1 over the 32 candidates).
- The integrity log keeps bits 0-15 for the D$, renumbered to the new invariants:
  - MSHR tag reuse;
  - response with no waiter;
  - two `pvalid` copies;
  - `vvalid` without `pvalid`;
  - fill beat to a non-reserved way;
  - write-back of a clean line;
  - NC access that allocated;
  - store merge into a dead MSHR.
- MEM-SIM grows the new structure's occupancy counters: MSHRs live, merges, retries, write-back
  buffer full, response-queue waits, probe outcomes. So the first lockstep says whether `NMSHR`,
  `NWB` or `NOUT` binds.

## Decisions (Tommy, 2026-09-25)

1. **VHPR, not VIPT.** Two large ways and no dTLB on the hit path are the point. Cache size is
   decided by measurement; a 128 KiB D$ and I$ is under test, with the probe allowed to take a
   few cycles.
2. **fence.i:** the I$ becomes coherent with stores (a filtered physical probe), so fence.i is a
   pipeline-only operation. This is its own increment after the D$.
3. **Sizes:** `NMSHR` 8, `NWB` 2, `NOUT` 8, if they fit and time.
4. **The open-source memory controller** is out of scope; the tagged path keeps the door open.
