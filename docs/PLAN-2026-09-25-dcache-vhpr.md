# The data cache rewrite: VHPR, non-blocking, end-state shape

Status: increments 1-3, 6a and 4a are built and on main; 4b step 1 is built and board-gated
(`wip/ic-coh-4b`); increments 5a and 5b are built. **The D$'s VHPR end state (5c, phase 3 and 6b,
decisions 1 and 5) is superseded by `PLAN-2026-10-03-dcache-vipt-l2.md`** (Tommy, 2026-10-03): an
alias-free VIPT L1 of 32 ways of 4 KiB and a 1.5 MiB L2. 5c is parked on `wip/dc-5c` and
`wip/dc-5c-4b` as the record. Everything else built under this plan stays.

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

## The memory port (increment 1's contract)

One port in the core clock domain, between `rv_mem_arbiter` and whatever serves memory: the
testbench model, the on-chip SRAM, or the platform's crossing into the DDR4 controller. Three
valid/ready channels:

| channel | fields | notes |
|---|---|---|
| request | `id[4:0]`, `we`, `addr[57:0]` (line address), `wmask[63:0]`, `wdata[511:0]` | a write carries its whole line; `wmask` is per byte (an NC store later sends only its own bytes) |
| read data | `id[4:0]`, `beat[1:0]`, `last`, `data[127:0]` | four 128-bit beats per line; the beat index says which quarter, so a WRAP burst can return the critical quarter first |
| write done | `id[4:0]` | raised when the write's AXI B response arrives, so a CBO or a fence completes only when its data is in DRAM |

- **Width.** 128 bits per core cycle on the read side matches the MIG's 64 bits at 333 MHz
  (2.67 GB/s). A 64-bit core-side beat would cap the path at a line per 8 cycles, less than 8
  outstanding misses need.
- **Order.** Responses carry the id their request brought and are matched by it (B1). The
  MIG returns in order; the on-chip SRAM answers faster than DDR, so across targets the
  responses do reorder, and the arbiter routes by id either way.
- **Ids.** `{requester, slot}`. The arbiter owns the id space; a requester never sees
  another's ids.
- **Read priority.** Among waiting requests the arbiter prefers reads; a write goes when no
  read waits, or when its requester says its buffer is full.
- **Below the port on the board:**
  - three asynchronous FIFOs into ui_clk (request, read data, write done);
  - an AXI master that pipelines AR and AW/W, one AXI ID, an in-order tag FIFO per channel,
    two 64-bit beats packed per 128-bit beat;
  - the platform's core-vs-DMA arbiter, which grants per transaction and routes R and B by
    an ID bit it stamps on the way in, so it never holds a channel from AR to `rlast`.

**In increment 1 the old caches sit on top unchanged.** An adapter per cache turns its single
outstanding `l2_req`/`l2_ack` line handshake into port transactions: it gathers four beats into
the line, and for a write it acks on write done. So the D$ and the I$ can each have one fill in
flight at the same time, and the crossing's 4-phase round trips are gone.

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
   - New bench: `tb_smolrv64_dcache2`. A golden byte image plus a reference translation, with random
     loads, stores, CBOs, NC, DMA writes, remaps with epoch bumps, synonyms (two VAs → one PA, dirty
     in one colour, accessed from another), and walker reads. Run at several latencies, in-order
     and reordering memory, every answer checked, every invariant live.
   - memrand gains a synonym-alias op, which it already half has: three aliases of one region.

   **As built (2026-09-27, `wip/dcache`).** `core/rv_dcache.v`, bench `tb_rv_dcache.v` (120 runs:
   VIRT=1 and VIRT=0 × 6 seeds × in-order/reordering memory × 5 shapes). Where it departs from
   the text above, and why:
   - **The core presents PAs only (`VIRT=0`), and the VA plumbing moves to increment 5.** The LSU
     has translated before it asks, so a virtual hit shortens nothing in phase 1; at `VIRT=0`
     every request is looked up by PA in its own colour and the D$ is a 128 KiB 2-way PIPT
     cache. The swap is then a drop-in with no LSU, LQ or SQ change and no epoch plumbing.
   - **A match in the request's own set answers at once** (both ways' data was read with the
     lookup) and re-stamps; replaying it let two aliases trade the line forever.
   - **The retry queue is the waiter table**: one entry per requester tag and one for the store,
     waiting on an MSHR or on any MSHR, write-back entry, NC slot or store freeing. Waiters on
     any freeing replay round robin; a fixed order starved the store under slow memory.
   - **A load to the line of the store in flight waits for it**: the store is accepted before it
     resolves.
   - **Stores go through the one lookup** (one per two cycles, as today, until increment 3). A
     hit writes the whole chunk its lookup read; a one-entry bypass covers the row written at
     the edge a later lookup read it.
   - **A miss to a line in the write-back buffer waits** for the write rather than being served
     from it; **cbo.zero reads the line** it then overwrites; there is **no early restart** and
     **no response queue** (an NC answer waits for a cycle the lookup does not answer). Each is
     a later optimisation.
   - **NC drops any cached copy first**, so NC stays coherent with a cacheable alias.
   - **fence.i's clean waits** for the store in flight and every dirty merge buffer.
   - **The allocation state is written speculatively**: a free MSHR's fields, its merge buffer
     and the lookup's own waiter entry are dead, so they are written whenever the lookup is
     busy, and only the bits that make them live wait for the probe (OOC WNS -0.381 -> +0.182
     ns at 6 ns).
   First measurement, IW=3 Linux lockstep (tiny128 boot): 19,122,783 -> 29,278,185 retires at
   60 M cycles (+53.1%), 91,331,479 -> 123,843,694 at 300 M (+35.6%).
   - **fence.i's clean walks rows, not slots.** The boot runs 3,867 fence.i in 60 M cycles; the
     slot walk spent 8.09 M cycles there, the row walk (all of a row's slots read at once)
     0.60 M: 29,278,185 -> 32,343,542 retires (+10.5%).
   - **128 KiB, and only timing may shrink it** (Tommy). The first 128 KiB build missed by 0.244
     ns across the whole core -- a route-bound plateau the D$'s area pushed on: 17.4 k LUTs and
     6.3 k flops, 4.6 k of them the merge buffers as flops with multi-way write muxes. As two
     LUTRAMs (even/odd chunks) with one write a cycle, a chunk-valid bit and a zero bit per
     MSHR, the module is 5.6 k logic LUTs and 1.8 k flops (out of context, WNS +0.457 ns). A
     cbo.zero meeting a line being filled waits for it rather than merging. Lockstep at 128 KiB
     against 64 KiB: +0.36% at 60 M, +4.62% at 300 M (125,118,782 -> 130,900,475).
   - **The read-out race is by arithmetic, and the invariant says exactly that.** The board's
     integrity log caught a fill beat landing in a slot still being read out (the boot SRAM
     answers in a few cycles; a drop's freed slot can be reserved by the next lookup). Beat c is
     written after row c is read, so the check is for a beat on a row not yet read; the bench
     gains a 1-4 cycle memory.
3. **The store port.** The SQ drains one store per cycle: the LSU's S_ST/`take_next` chain
   becomes a stream.

   **As built (2026-09-29, `wip/dc-inc3`).**
   - **A 3-entry store queue in the D$ replaces the one store slot.** It takes a store in any
     cycle it has room; `wr_room`, a register, says so. The LSU finishes a store to memory on it,
     so no accept travels back, and chains the next store the same cycle (`take_next`).
   - **Only the head resolves.** The queue issues its head, and the entry behind it in the cycle
     the head is in the compare. If the head parks in the waiter table, the entry behind it goes
     back to the queue untouched. The store's waiter entry is only ever the head's, and invariant
     11 (`sq_order`) asserts that the send-back never meets the head leaving.
   - **Loads wait on any queued store to their line,** not just the one in flight.
   - **fence.i's clean lets the queue drain first:** stores keep issuing while loads are held.
   - **A plain store to a device** (bare M-mode, no NC bit) ends at the device's accept. The LSU
     tells the two classes apart by a registered `mem_q`, never by an address decode.
   - **The bench streams stores at `wr_room`** and presents loads to the lines of queued stores.
     Three mutations are caught: loads blocked only on the head's line, the send-back clobbering
     the head's waiter entry, and a store behind a parked head resolving.
   - **Measured, IW=3 Linux lockstep (tiny128 boot):**
     - 33,867,866 -> 35,672,123 retires at 60 M cycles (+5.33%);
     - 135,292,357 -> 143,322,223 at 300 M (+5.94%);
     - the store queue full now 2.36% of the 300 M cycles.
   - `run-vl.sh` fails its build when the riscv-tests bench leaves a core input unconnected: the
     new `dmem_wroom` read as 0 there and hung every test that stores.
   - **Build (548c43ea):**
     - At IW=3, AltSpreadLogic_medium missed by 0.135 ns on the broad core plateau: 5,567
       endpoints under +0.35. Its worst families are the ALU operand -> `alu_q_val`, the D$
       response tag -> `pl_q`, and `cf_link`, none in the store port.
     - The Explore directive met, WNS +0.004.
   - **Board gate: PASS** (login, 0 faults, 900 s GB5 stress, errlog clean). Top-Down on the
     board, 30 MB of random data, against 06a88177:
     - `sha256sum`: IPC 1.637 -> 1.718; back-end memory 7.30% -> 3.61% of cycles.
     - `xz -6 -T1`: IPC 0.531 -> 0.542.
4. **The coherent I$:** a physical probe of the I$ on every store; fence.i becomes pipeline-only.
   The phase-3 text in "Epochs, fence.i, DMA" leaves two holes, which fix the split below:
   - **Code stored before its page is ever fetched from** sits dirty in the D$. The page filter
     never saw the store, and an I$ fill reads memory, so it would install stale bytes.
   - **Code written by DMA** (a block device filling page frames) never probes the I$. Linux
     relies on fence.i after mapping such a page executable, and the VHPR I$ keeps physically
     resident lines across a mapping change, so a stale line would survive both.

   **4a, coherent fills.** Before any I$ line read goes to memory, the D$ cleans that line by PA
   (a cbo.clean from the I$, `ic_req`/`ic_ack`, queued in the D$'s store queue behind every
   store taken before it). An I$ fill therefore never reads a line the D$ holds dirty, and
   fence.i drops the D$ clean walk: drain the core, invalidate the I$ (64 cycles).

   **4a as built (2026-09-30).**
   - The clean runs beside the fill, not before it. `rv_soc_top` sends the I$'s line read to
     memory at once and uses the data only if the D$ has answered by then that the line was clean
     on its first look (`ic_dty` low). Otherwise, including data faster than the answer (the boot
     SRAM), it drops the data and reads again after the answer.
   - The clean in front of the read measured -2.61% at 60 M and -4.7% at 300 M: the I$ is a
     quarter of the boot's cycles, and every miss paid a D$ round trip.
   - Two assertions guard the gate: no new I$ read while one is owed, and the line client's own
     (no request while one is owed). The second caught a re-read in the answer's cycle.
   - Lockstep: 37,362,613 -> 37,767,298 at 60 M (+1.08%), 159,905,670 -> 160,629,194 at 300 M
     (+0.45%). The storm is clean to 500 M.
   - The D$ bench asks for I$ cleans about 1,400 times a run (a third find the line dirty) and
     checks memory at each ack. Two mutations are caught: acking at take, and the clean
     completing the store port.
   - Build (036440f2): the default directive missed by 0.081 ns; the Explore directive met, WNS
     +0.044. Board gate PASS (login, 0 faults, 900 s GB5 stress, errlog clean).

   **4b, invalidation by probe.**
   - **Layout:** the I$'s valid bits and physical tags move to the D$'s layout, one array per
     (way, colour) of 64 rows. A probe by PA reads all 32 candidates at PA[11:6] in one cycle,
     and clears the valid bit of every match (several at once: the I$ keeps synonyms).
   - **What probes:**
     - every store the D$ takes, including CBO zeros and AMO writes;
     - every line of every DMA write burst, tapped at the platform's DDR arbiter (the device
       side, `ui_clk`) and carried to `probe_clk` through an async FIFO.
     The virtio-net receive traffic is constant on the NFS-root board, so a "DMA happened: wipe
     the I$" flag would wipe it at nearly every fence.i. The exact probe does not.
   - **Races:** a probe also kills a fill in flight or a prefetched line for the same PA, so a
     fill read before the store cannot install stale bytes.
   - **No filter:** the probe has its own read port and is cheap at one per cycle.
   - **fence.i:** waits for the probe queue to drain, then flushes only the fetch ring, the
     decoupling queue and the predictions.
   - **Why it pays:** the boot runs a fence.i every ~15 k cycles, and each wipe refetches the
     working set.
   **4b as built (2026-09-30), step 1: stores probe; a device write keeps the old fence.i.**
   - The I$'s physical tags and validity are one 64-row array per (way, colour); the probe reads
     all 32 at the written line's row and invalidates every match. A fill in flight or a prefetched
     line for it lands dead.
   - **Validity is two single-writer bits that must agree:** `ig` (installs, the scan) and `kg`
     (probe kills). A kill and an install in one array in one cycle then both land. With one
     valid bit, one of two same-cycle writes would be lost, and a dead fill could stay valid.
   - **Every write the core presents probes** (`dmem_wen` registered): stores, AMOs, CBOs,
     straddle beats. Repeats are harmless, so there is no filter and no queue: one per cycle.
   - **fence.i drains** every older store to the D$ (the store queue now included, in the
     commit before) and the last probe, then redirects. It clears the I$ only if a device wrote
     memory since the last one.
   - **The device write event:** in the platform, each device write burst's address handshake,
     held four `ui_clk` cycles and synchronised to `probe_clk`. In simulation, the virtio-blk
     slave's and the DMA agent's writes.
   - **Step 2:** exact DMA probes, per line through an async FIFO, so device writes stop clearing
     the I$ at fence.i. Measured below as not needed yet.
   - **Bench:** the I$ bench rewrites and probes a line every ~150 cycles with requests in
     flight. A third of the probes are aimed at the line being filled or prefetched: about 550-800
     fills and 20-120 prefetches killed a run. Three mutations are caught: no kill, fills never
     dead, the prefetched line surviving.
   - **Lockstep** (tiny128: no device writes, so fence.i never clears the I$):
     37,767,298 -> 38,695,476 at 60 M (+2.46%), 160,629,194 -> 162,576,179 at 300 M (+1.21%).
     fe:icache 23.4% -> 21.5% at 60 M. The storm is clean to 500 M.
   - **Build (76e0e0cd):** the default directive missed by 0.058 ns; the Explore directive met, WNS
     +0.003. Board gate PASS (login, 0 faults, 900 s GB5 stress, errlog clean).
   - **Board A/B against 4a,** each bitstream booted clean, I$ misses per 1000 instructions and IPC:

     | workload | 4a: I$ misses | 4a: IPC | 4b: I$ misses | 4b: IPC |
     |---|---|---|---|---|
     | 300 execs of `/bin/true` | 48.8 | 0.303 | 49.0 | 0.303 |
     | 6 x `gcc -O2 -c` | 11.5 | 0.580 | 11.5 | 0.581 |
     | `xz -6`, 10 MB of /usr/bin | 1.9 | 0.911 | 1.4 | 0.912 |

     Steady-state userspace barely runs fence.i: Linux flushes the I$ when a page-cache page is
     first mapped executable, not on every exec of it. So neither 4b nor exact DMA probes move
     these workloads; 4b's gain is the boot's fence.i (the lockstep above). Step 2 is not built
     unless a workload shows fence.i clearing the I$.

5. **Phase 2: the queues translate, and a load hits by its VA** (design, 2026-09-30; C4b step 3).

   **What it buys.** A load today translates in M in the same cycle it asks the D$ (`req_early`),
   so a virtual hit alone shortens nothing. What changes is where translation sits:
   - **Walks leave M.** A load whose dTLB misses holds M, and every memory op behind it, for the
     walk. After this increment a walk serves only a D$ miss, while other loads proceed.
   - **The dTLB leaves the hit path,** so it can become the miss-path TLB of the future-TLB design
     (a registered BRAM read, 4 KiB and 2 MiB tables) with no timing cost on a hit.
   - **A load never waits for an older store's translation** to learn whether it aliases.
   - **M never waits on translation,** a precondition for deleting it (C4b steps 4 and 5, the LSA
     pipe).
   - **6b's VA-hashed way 1** needs the VA at the D$.

   The tiny128 lockstep walks rarely (the board's `xz` walked 1.6 times per 1000 instructions with
   the 2048-entry dTLB), so the lockstep gain will be small. The increment is judged against the
   end state.

   **The translation stays in the LSU, fed by the queues, not in the D$.** Faults and the NC and
   device classes belong to the queue entries, because the trap fires from the entry at the ROB
   head. The D$ then stays what it is: every miss is looked up by PA. (This replaces "the D$
   translates on a miss through its translate port".)

   **5a, the shadow.**
   - The LQ and SQ entries gain the VA (39 bits) beside the PA.
   - A second read port on the dTLB (LUTRAM: duplication buys read ports) translates each entry
     from the queue. It asserts that the PA, the fault and the NC/`mem` classes equal M's.
   - Bit-identical in every run.

   **5a as built (2026-09-30, `wip/dc-5a`).**
   - The entry M filled at T is read out of its queue at T+1 and looked up at T+2 in `mmu`'s second
     port (`s_*`: the TLB alone, never a walk). A lookup that resolves must name the entry's PA and
     NC bit: a `$fatal`, and integrity bit 40 (`qx_xlate`).
   - In the first 30 M cycles of the boot every one of the 4.4 M lookups resolved; the boot turns
     paging on at about 49 M cycles.
   - Mutations caught: bit 12 of a load's or a store's stored VA flipped, and bit 12 of the second
     port's Sv39 PA flipped (at 49 M cycles). The LQ/SQ benches read each filled entry back the
     next cycle, writing the VA as the PA's complement; a read port stuck on entry 0 is caught.
   - Lockstep identical: 38,695,476 at 60 M, 162,576,179 at 300 M. The storm is clean to 500 M.
   - Build (the same RTL, uncommitted): the default directive missed by 0.009 ns; the Explore
     directive met, WNS +0.003. 116,317 -> 121,974 LUTs, the second read port of the 2048-entry
     LUTRAM. Board gate PASS (login, 0 faults, 900 s GB5 stress, errlog clean).

   **5b, walks leave M; the D$ still by PA.**
   - **The dTLB keeps one read port, M's lookup; the walker serves the queues.** An entry reaches
     the walker only after M's lookup missed, so the walker walks at once and needs no lookup of
     its own. 5a's second read port goes: in LUTRAM a read port of the 2048-entry table costs
     about 5.7 k LUTs (5a's build: 116,317 -> 121,974).
   - **A TLB hit in M is today's pass, unchanged:** the entry is filled with the PA, M lets go,
     and a load may start its access in the same pass (`req_early`).
   - **A TLB miss, or a hit whose perms fail, does not hold M.** M writes the VA alone (the entry's
     `tv`, translated, stays clear) and lets go. The walker then translates the oldest
     untranslated entry, stores first (a store cannot commit until it is translated), and writes
     its PA, NC and memory classes, or its fault.
   - **M's head-only ops (AMO, LR/SC, CBO) keep M's own translation,** walking through the walker
     when it is free: they are at the ROB head, so nothing older waits on it.
   - **What waits for `tv`:** a load's access (the queue's candidate needs it) and a store's
     commit (`kc_v` needs it). A store's ROB slot already completes at commit (`sq_k_take`), not
     in M, so the ROB needs no new completion port.
   - **Page faults ride in the entries:** `{fault, cause}`, with `tval` the entry's VA and `epc` the
     op's PC, which each entry keeps from dispatch (the ROB holds no PC in synthesis): 16 x 39 bits
     of flops. The misaligned and non-canonical faults need no TLB and stay M's.
     - A faulting entry never completes (a load never lands, a store never commits), so its op
       reaches the ROB head and waits there. The queue's fault record for the head (flops: the
       store queue's first uncommitted entry, the load queue's candidate) joins the SYSQ's fire at
       the head, the one trap gate. Its payload is registered a cycle ahead (C3 step 2 lost IW=3
       closure reading a LUTRAM payload into `csr_file`).
     - A wrong-path fault dies with the flush.
   - **The alias matrix compares VA[11:0] byte ranges** (= PA[11:0]).
     - It is conservative for synonyms, which share their page offset.
     - It is known when the VA is, at M, so a load never waits for an older store's walk.
     - A false alias needs the same 8-byte chunk offset in a 4 KiB page: 1 in 512 for a random
       pair. The lockstep counts false blocks.
   - **The gain now:** a walk no longer stalls M and every memory op behind it; TLB hits keep
     today's latency.

   **5b as built (2026-10-01, `wip/dc-5b`).**
   - As designed, with M's lookup in 5a's port and the walker's port serving the queues; 5a's
     check goes. M's early-start loads take their PA from the lookup port (`eff_pa`): taken from
     the walker's port, every AMO test died on the speculative-device assertion.
   - Each entry keeps its op's PC (VA[38:0]) and seq for the trap. The SYSQ takes a queue fault's
     record when the op is the ROB head and fires it; two SYSQ assertions (M busy, FP work in
     flight) exempt it, since younger work may be in flight and dies.
   - The queue benches assumed PA-wide alias compares: a store in another page at the load's offset
     now blocks (a new case says so), and a page-crossing load cannot be queued (the store queue
     asserts it).
   - Coverage: `rv64si-p-dirty` traps three store page faults from the store queue, and the
     `rv64*-v-*` tests fault their pages in through both queues; a wrong cause on a queue trap
     fails them. In the first 60 M cycles of the boot M's lookup hits 14.65 M times and hands
     166,108 entries to the walker, which answers 134,111 (980 faults, all on wrong paths: no
     queue trap fires).
   - Lockstep: 39,179,272 at 60 M (-0.03%), 170,226,822 at 300 M (+1.46%). The storm is clean to
     500 M.
   - **The walker's request is a register.** Built with the entry chosen combinationally, the
     design missed by 0.42 ns (default) and 0.45 ns (Explore), with 12,917 endpoints under +0.35 ns:
     the queues' candidate pointers ran through the entry's VA into the 2048-entry TLB's read
     address (`u_lq/acc -> u_lsu/u_mmu`, 1,560 paths) and congested the whole core. The walker now
     latches the entry's VA as it takes it, a cycle per walk: 39,529,260 at 60 M (+0.89%),
     168,133,862 at 300 M (-1.23%; +0.21% over the fill hold alone).
   - **Two more cones, found by `make census` on the next builds.** The SYSQ took a queue fault's
     record on `qf_in & ~redirect` and the walker latched its VA in the else of the redirect arm
     (rule I11); the record now takes a register stage and the payloads load with no redirect. Then
     the census still put `u_rob/head` on the queues' PA registers and the dTLB: the walking port's
     address mux selected on M's live `xl_f`, and M's completion was written `xo_ok | xo_nopa`,
     whose TLB terms synthesis kept. The port's requester is now a register (`pm_q`: an AMO, LR/SC
     or CBO takes the port a cycle after it asks) and M's completion is the address-only test.
   - Build (the tree of 286af18f + those two): the default directive met, WNS +0.005. Board gate PASS
     (login, 0 faults, 900 s GB5 stress, errlog clean): the first bitstream with the I$ fill hold too.
     Lockstep 39,218,710 at 60 M, 170,075,891 at 300 M (+1.37% over the fill hold alone); the storm
     is clean to 500 M.

   **5c, `VIRT=1`: a load asks by its VA first.**
   - **A virtual hit answers with no PA.** A virtual miss answers a nack with no side effect: no
     MSHR and no walk. The entry then asks again by PA once it is translated, exactly as in 5b.
   - **Two separate choices about when a load uses the TLB:**
     - **The lookup runs beside the D$ ask**, not after the nack: M's lookup runs in the pass that
       sends the load's VA-only ask. The PA is then in the entry when the nack
       arrives, and the re-ask by PA follows at once: about one
       cycle behind a D$ that translated internally, against 2-3 for a lookup started by the nack.
     - **A walk starts only after the nack:** the walker takes an untranslated load only once
       the D$ has refused it. A TLB miss on a load that then hits the D$ needs no
       walk (its permissions are on the stamp), so walks serve D$ misses only: the stream Simmerv
       measured for design B. For today's 2048-entry direct-mapped table: 0.21 walks per 1000
       instructions on that stream, against 0.74 when every access walks on a TLB miss.
   - **The last cycle, later:** the TLB's answer can reach the D$ in the lookup's compare cycle, so a
     miss allocates its MSHR without a re-ask. That puts the TLB in front of the allocation, and a
     fault or an NC page must hold the allocation off; it is left until the L2 makes a miss's
     cycles count more.
   - **Permissions on the stamp.**
     - The leaf's `{U, R, X}` are written with the stamp, from the translation that installed or
       re-stamped the line.
     - A hit checks them against the request's effective privilege, SUM and MXR, captured with
       the request:
       - the page must be readable: `R | (X & MXR)`;
       - a U page is readable from U-mode, or from S-mode with SUM;
       - an S page is readable from S-mode only.
     - A stamp is written only from a translation that passed, so A is set.
     - Stores never use the virtual hit: they carry the PA they translated before committing. So
       W and D are not on the stamp.
   - **Untranslated accesses go by PA and never stamp.** These are M-mode without MPRV, and a bare
     `satp`. A VA equal to a PA must not hit a stamp made under a mapping.
   - **`ep_bump` on `sfence.vma` and `satp` writes.** Both already serialise on an empty ROB and
     store queue, and `dmem_idle` includes the D$'s store queue since b1b6bb64.
   - **The D$'s load-behind-queued-store block** compares line bits [11:6] for a VA-only request
     (synonym-safe) and the PA line for a request by PA.
   - **NC and device pages never stamp,** so their first ask always misses. The translation
     classifies them, and they take today's path.
   - **Way 1 is unskewed at `VIRT=1` until 6b.** 5c alone gives back 6a (lockstep -0.4%, SGEMM
     fills x3), so 5c and 6b make one board point.
   - **New counters:**
     - virtual hits and nacks;
     - re-stamps (a physical match in the request's own set);
     - synonym drops;
     - the translate port busy;
     - walks by requester.

   **Cost:**
   - 16 entries x (39 VA bits + fault fields) of flops;
   - 3 perm bits per stamp (LUTRAM, 2 x 1024);
   - 5a's second dTLB read port (about 5.7 k LUTs), for 5a only.

   **Decisions (Tommy, 2026-09-30):** the LSU translates (a miss's re-ask costs cycles that will
   matter for L2 misses, taken for simplicity); the TLB is looked up beside the D$ ask and walked
   only after a nack; the alias matrix compares page offsets; 5c and 6b make one board point.

   **Risks:**
   - **Synonym drops between the kernel's linear map and user VAs** (`copy_to_user`): the colours
     differ, so every crossing drops the line. The Simmerv models ran user and kernel VAs but did
     not count drops. 5c counts them first.
   - **Wrong-path walks.** A nacked wrong-path load can walk. That is harmless: walks only read,
     because the hardware sets no A/D bits.
   - **The trap payload's timing,** above.

6. **The skewed way.**
   - **6a, at `VIRT=0`:** way 1 is indexed by the xor-fold of the request's PA line. This is
     PIPT-skew as phase 1 already pays for it, with no D and no synonyms. Placement is one NRU
     bit per line, both set -> way 1.

     **As built (2026-09-29, `wip/dc-skew6a`).**
     - `SKEW = (VIRT == 0)`. Every way-1 site takes its set from `s1_set1`: the fold of the PA
       line at `VIRT=0`, else the VA set. So `VIRT=1` is unchanged.
     - The physical tag is the whole line (PA[35:6]), because way 1's row no longer names PA[11:6].
       A victim's or a clean's write-back address is then the tag itself.
     - Each way's physical arrays are read at the row of that way's set.
     - The round-robin bit gives way to `nru0`/`nru1`, one write statement each.
     - The bench covers way-1 fills away from way 0's set (about 1,400 a run at `VIRT=0`) and
       ageings. Three mutations are caught: way 1's probe at the PA row, a way-1 fill into way 0's
       set, and a way-1 store hit writing way 0's row.
     - IW=3 Linux lockstep (tiny128, no strided arrays): 35,672,123 -> 35,987,163 at 60 M (+0.88%),
       143,322,223 -> 143,935,610 at 300 M (+0.43%). The storm is clean to 500 M.
     - Build: the Explore directive missed by 0.087 on the core plateau (the LSU's `o_v` into the
       schedulers' wakeups, the ALU results, the predictor). The default directive met, WNS +0.006.
       The skew's own family (`s1_set1` into the store queue's shift enables) sits at -0.058.
     - Board gate PASS. A/B on the board, the two bitstreams back to back, IPC and D$ line fills
       per 1000 instructions:

       | workload | increment 3: IPC | increment 3: fills | 6a: IPC | 6a: fills |
       |---|---|---|---|---|
       | naive SGEMM, N=512 (2048 B pitch) | 0.38 | 139.6 | 0.41 | 42.6 |
       | `xz -6`, 10 MB of /usr/bin | 0.68 | 5.1 | 0.68 | 4.3 |
       | `xz -6`, 30 MB random (two runs each) | 0.54, 0.54 | 10.0-10.5 | 0.53, 0.56 | 9.4 |
       | `sha256sum` | 1.68 | 0.45 | 1.69 | 0.31 |

     - SGEMM loses 70% of its misses but gains only 8%. It is dTLB-bound: 84 walks per 1000
       instructions at 18 cycles each, the same on both, about 1.5 of its 2.4 CPI. The miss-path
       TLB (increment 5, the future-TLB design) is what turns the skew into ML speed.
     - A board run taken just after a gate's boot and stress is not an A/B point: one such
       `xz` run read 0.45.
   - **6b, with phase 2:** the same hash of the VA line, plus D and its invariants. D is built as
     the tag array a later L2 will carry data for.

## Phase 3: the skewed way and the reverse directory (design, 2026-09-29)

### Why: one power-of-two stride defeats any index built from bits [15:6]

Geekbench 5's Machine Learning is 85% one naive i-j-k SGEMM loop. Its B-column load walks a
2048- or 4096-byte pitch. That makes 98.9% of all D$ misses (Simmerv, from the 673 G checkpoint
for 15 G instructions; `wip/wset` in the Simmerv tree).

- **The row bits PA[11:6]** take 2 values at a 2048 B pitch and 1 at 4096 B.
- **The colour bits [15:12]** take 16.

So the column lives in 32 of the 1024 sets. Any index made only of bits [15:6] caps there: straight,
permuted or XORed among themselves. More ways at 128 KiB do not help (8- and 16-way measured
unchanged). The index has to take in bits at 16 or above.

Whole GB5 single-core in Simmerv: 172 windows of 500 M instructions after a 50 M warm-up, one every
4 G from boot, weighted by subtest length. DRAM misses per 1000 instructions:

| design | all of GB5 | GB5 without ML | ML | cost |
|---|---|---|---|---|
| today: 2-way, straight index | 8.31 | 5.45 | 70.9 | -- |
| **B: way 1 skewed by a VA hash, reverse directory 2048x4** | **5.20** | **5.21** | **5.0** | forced evictions 0.19, moves 0.034 |
| B with a 1024x4 directory | 5.52 | 5.46 | 6.8 | forced evictions 1.54 |
| PIPT-skew: both ways physical, way 1 PA-hashed | 5.14 | 5.15 | 4.97 | a physical index: a cycle on every access |
| A-P1: way 1 physical behind a translation, older-of-two | 5.13 | 5.14 | 4.97 | 140 slow hits (each translated first) |
| A-P4: as A, promote on the second way-1 hit | 5.58 | 5.60 | 5.15 | 12.6 line moves |

The skew pays beyond ML: SQLite 1.93 -> 0.46, Ray Tracing 4.84 -> 2.30, Clang 3.57 -> 2.89,
Rigid Body 4.23 -> 3.27 (A-P1; B is within 0.1). With a PC-indexed stride prefetcher (64 entries,
degree 2) on B: 1.45, 70% coverage.

### Rejected, and why

- **A physically indexed way 1 (design A).** Its index needs the PA, so way 1 is read only after a
  translation, and a way-1 hit is slow. Every placement policy then pays in one of two ways:
  - it leaves hits slow: older-of-two leaves 45% of hits in way 1; move-free reuse-bit rules leave 35%;
  - or it moves lines: 12.6-17 moves per 1000 instructions.
  Tommy: moving lines is very expensive and must be rare and overwhelmingly advantageous.
- **PIPT-skew.** Its miss rates equal B's. But a physical index costs a cycle of latency on every
  access (Tommy), and the plan's point is no translation on the hit path.
- **A reuse-bit placement** ("fill way 0 unless its line was reused"): ML collapses to 65.8. The
  strided fills alternate between the ways and evict half the column each pass.
- **A multiplicative hash:** 13.5-19.5 on ML against 5.3 for the xor-fold.

### The design: B

- **Way 0** is indexed by VA[15:6] (colour and row), as built.
- **Way 1** is indexed by `(l ^ l>>10 ^ l>>20)[9:0]`, where `l` is the VA line number. Both ways are
  read at T and both answer at the same latency.
  - The xor-fold is one LUT level. It is computed from the request's address before the door and
    registered with the request, so the BRAM address stays early (I6).
- **The index takes the address the request carries.** At `VIRT=0` that is the PA, and way 1 is a
  PA-hashed physical way with no synonyms. So the skew does not wait for phase 2 (increment 6
  below); phase 2 changes only what the address is.
- **Placement: one NRU bit per line.** way0[VA[15:6]] and way1[hash] are in different sets, so
  recency is per line. The bit is set on a fill and on every hit. A new line takes an empty
  candidate, else one whose bit is clear (way 0 if both are), else it clears both bits and fills
  way 1.
  - Whole GB5 in Simmerv, DRAM misses per 1000 instructions (all / without ML / ML):

    | design | NRU, both set -> way 1 | true LRU |
    |---|---|---|
    | B | 5.41 / 5.26 / 8.7 | 5.20 / 5.21 / 5.0 |
    | PIPT-skew | 5.39 / 5.23 / 8.9 | 5.14 / 5.15 / 5.0 |

  - Filling way 0 when both bits are set is worse (ML 9.5-9.7). A fill-order bit per pair gains
    about 0.1 on ML and loses 0.16-0.19 without ML, so it does not exist.
  - B's forced evictions do not move (about 0.19).
- **Tags.** B keeps the virtual stamp in both ways (vtag, epoch, vvalid, perms), so a hit needs no
  TLB. Way 1's physical tag is the full PA line (~30 bits), because its index says nothing about the
  PA.

**The reverse directory D.** It answers "given a PA, which way-1 set holds it?"
- **Shape:** 2048 entries, 4-way, indexed by the xor-fold of the PA line. An entry is {PA line tag,
  way-1 set}, about 38 bits: 4 BRAM36, no data.
  - It must be hashed for the same reason as way 1. A D indexed by PA[11:6] inherits ML's
    32-set pathology.
- **When it is consulted:**
  - a miss, in parallel with the memory request, which is dropped if D finds the line (about 5 per
    1000 instructions);
  - page-walk PTE reads;
  - CBOs and NC accesses.
  It is never consulted on a hit.
- **Updates:**
  - A way-1 fill writes an entry.
  - A full D set evicts its oldest entry's way-1 line: a *forced eviction*, written back if dirty
    (0.19 per 1000 instructions at 2048x4).
- **Stale entries are harmless.** An entry is a hint, verified against way 1's PA tag at the set it
  names. So deletion on eviction can be lazy.
- **The one exact invariant:** no `pvalid` way-1 line without an entry. A missing entry lets a miss
  re-fetch a PA already cached under another VA: two copies, and silent corruption on the first
  store.
- **Way 0** keeps its 16-colour probe at PA[11:6]. An option folds way 0 into D (4096 entries), which
  removes the 32 probe arrays: one PA-to-location mechanism for both ways.
- **The L2.** An inclusive physical L2 whose tags carry each line's L1 location *is* this directory
  (the R10000's arrangement). D is the L2's tag array built before its data. The UltraRAM (about
  2 MiB) holds an inclusive L2 of a 128 KiB L1 easily.

**Variants measured, same miss behaviour:**
- **B'.** Way 1 is tagged by PA only. Its data is returned speculatively and confirmed a stage later
  by the TLB.
  - 134.5 speculative returns per 1000 instructions, 96% confirmed.
  - The mis-speculations are the genuine misses (5.0) plus 0.10.
- **C.** Both ways are tagged by PA, and a direct-mapped TLB is read in parallel on every access.
  - The virtual stamps, epochs and per-line permissions go; `satp`/`sfence.vma` touch only the TLB.
  - The price is TLB misses on every access: 0.74 per 1000 instructions at 2048 entries, 0.68 at
    4096. Gaussian Blur is 2.10 (page conflicts), against 0.089 walks for B's miss-path 2048x4
    table.

**The TLB** is the miss-path table of the future-TLB design: 4 KiB pages 2048x4, and 2 MiB pages in
a 32-entry direct-mapped table (0 misses: the kernel touches about 14 regions). On B's way-0-miss
stream it walks 0.089 times per 1000 instructions, against 22.3 for today's 16-entry dTLB.

**New invariants:**
- D is inclusive of way 1 (asserted at every way-1 fill and every D eviction).
- The single copy spans way 0's colours and way 1 (the probe plus D, at every allocation).

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

1. **VHPR, not VIPT** (superseded 2026-10-03: VIPT with page-sized ways). Two large ways and no dTLB on the hit path are the point. Cache size is
   decided by measurement; a 128 KiB D$ and I$ is under test, with the probe allowed to take a
   few cycles.
2. **fence.i:** the I$ becomes coherent with stores (a filtered physical probe), so fence.i is a
   pipeline-only operation. This is its own increment after the D$.
3. **Sizes:** `NMSHR` 8, `NWB` 2, `NOUT` 8, if they fit and time.
4. **The open-source memory controller** is out of scope; the tagged path keeps the door open.
5. **Two ways stay** (Tommy, 2026-09-28; superseded 2026-10-03: 32 ways of 4 KiB): the only
   sacred structure, for the fast lookup.
   - Way 1 is skewed by a hash that takes bits at 16 and above. Both ways answer at the same latency.
   - No line moves on a hit.
   - Measured and chosen 2026-09-29: design B (phase 3 above) over the physically indexed way
     (design A) and PIPT-skew.
