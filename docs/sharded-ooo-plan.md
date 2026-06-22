# Sharded (clustered) OoO superscalar — design plan

Status: design in progress (2026-06-20). The renamer is the only part with RTL +
timing data so far (`probe/`). This doc is the running spec; numbers and decisions
otherwise live only in chat history.

## Goal & philosophy

A 4-wide out-of-order RISC-V core built as **S parallel shards**. The organizing
principle is to **minimize, not eliminate, inter-shard communication**: shard the
execution, register file, scheduler, renamer and decoder, and pay for the few
edges that must cross. Bias hard toward *cheaper* over *maximum ILP* — accept
imbalance and lost speculation when it buys a smaller/faster machine. The ALU
(~400 MHz on the XCKU5P, see `docs/timing-probe.md`) is the accepted Fmax bar;
anything comfortably above it is fine, so we do **not** sweep for limits.

## Geometry & parameters (all parametrizable)

| Param | Default | Meaning |
|---|---|---|
| `S` / `W` | 4 | shards = issue width; **1 instruction per shard per cycle** |
| `AREGS` / `ABITS` | 64 / 6 | architectural regs — **unified int+FP PRF**: FP = arch 32–63 |
| `NPHYS` / `PBITS` | 128 / 7 | physical registers |
| pool/shard | 32 | `NPHYS/S` — each shard owns a disjoint physreg pool |
| `NCHK` | 4 (TBD) | checkpoints |

Unified PRF consequence: `x0` is still absent/zero, but `f0` (arch 32) is a normal
register — so "reg index 0 == no operand" is dead. The decoder emits **explicit
per-operand valid bits**, and one physreg is reserved as a constant-zero that `x0`
maps to permanently (never allocated/freed). This folds the x0 case into the
general valid mechanism.

## Sharding scheme

- **Register file — replicated, bank-disjoint writes.** Each shard holds a full
  copy of all banks. A shard writes only its own bank locally and *broadcasts*
  that write to the sibling copies → each copy sees S write ports but to disjoint
  banks (no multiport arbitration). Cost = S² RAMs + broadcast routing. (Alpha
  21264 clustered trick.)
- **Freelist — partitioned.** Each shard allocates only from its own pool →
  single allocation per shard per cycle. See *Freelist* below.
- **MAP — replicated, NOT bank-disjoint.** Arch regs can't be partitioned, so
  every copy needs true W write ports (the bundle's destination broadcasts). Made
  of flops. Reads are local (this shard's sources).
- **Decoder pre-resolution — the key move.** The decoder rewrites each source as
  `{ARCH(a) | SLOT(j)}` (j = an earlier in-bundle producer) and flags only the
  *last* writer per arch reg as MAP-visible. This pulls the O(W²) source-vs-dest
  comparator network **and** the WAW write-priority mux **out of the rename loop**:
  rename source becomes a W:1 mux, and the W MAP write ports always hit
  guaranteed-distinct addresses (no priority logic). The O(W²) comparison moves to
  the decoder, where there is pipeline slack.

## Cross-shard communication model

Per stage, what must cross between shards:

| Stage | Crosses to neighbors | Notes |
|---|---|---|
| Decoder | destination arch regs | to compute `SLOT(j)` + last-writer matrix |
| Renamer | `(rd, pr)` + `is_map_writer`; **all valid dest pr** | pr alone (any valid dest, incl. overwritten) for SLOT resolution; `(rd,pr)` only when MAP-visible for the MAP write |
| Scheduler | issued instruction's dest pr + ready-time | scoreboard update (see *Scheduler*) |
| Execution | `(pr, value)` | **both** RF writeback **and** the bypass network |

**Key timing insight (corrects an earlier "+1 cross-shard tax" framing):** every
one of these is a *registered, next-cycle state update*. They therefore are **not
logic timing paths** (a full cycle to propagate) and add **no pipeline bubble**
(the consumer reads the updated state next cycle regardless). The cost is purely
**wire delay** — the broadcasts must close timing across the physical shard span
in one cycle. The single tightest wire is the **1-cycle-ALU → cross-shard
consumer bypass** (result valid end of N, consumer executes N+1 → one cycle of
wire). Longer-latency ops have slack. If that bypass wire ever fails to close, the
fallback is to pipeline it and eat a bubble *only* on cross-shard 1-cycle deps.
Expect results to be wire-dominated.

## Frontend: basic-block prediction → fetch → aligner

**Basic-block (fetch-target) prediction + FTQ.** The BP emits
`(block_start_PC, length_in_halfwords, next_PC)` tuples into a **fetch-target
queue**; fetch consumes the FTQ. "Next control transfer" = next predicted-*taken*
transfer, so a block may *contain* not-taken conditional branches (each a real
instruction and a misprediction/checkpoint point). Wins from decoupling: BP runs
ahead of fetch (FTQ depth = run-ahead window), enabling early I$-miss/prefetch off
future fetch addresses and absorbing a 2-stage BP. BP internals (TAGE/BTB/…) are
isolated behind the tuple interface — deferred.

**Fetch buffer + even/odd I$.** A 16-byte buffer is presented to the aligner;
byte 0 is guaranteed an instruction start. Unconsumed tail is shifted down and
refilled (within a block); at a taken boundary the tail is **flushed** and refill
starts at `next_PC` (FTQ says which). To deliver a full aligned 16 B at any
alignment, the I$ is **even/odd banked** and fetches `pc` and `pc+16` (32 B), from
which a big mux extracts the 16 B window (expensive; may be >1 stage). 16 B is
provably minimal-sufficient for 4-wide: 4 contiguous insns are each ≤4 B → ≤16 B.

**Aligner.** Breaks the 16 B into ≤4 instructions in program order and steers them
to shards. **No predecode boundary bits** — a 32-bit insn can straddle a line so
boundaries are entry-alignment-dependent; the only alignment-independent fact
(2-vs-4-byte length = `&parcel[1:0]`) is a pure function of the parcel, free to
recompute, and storing it wouldn't shorten the dependent chain. The boundary scan
is a 4-step advance-by-2-or-4 prefix chain (find ≤4 starts) feeding four
variable-offset 16 B→4 B extraction muxes; the same scan yields the consumed-byte
count driving the shift + PC advance. Mux/route heavy, pipelineable.

**Early termination wrinkle:** if the decoder finds a serializing instruction at
slot k, slots k+1.. are squashed and we refetch past it — handled by the normal
redirect path, not a special aligner case.

## Steering — fixed, by the aligner

Instruction→shard assignment is **static and sequence-based**: **leftmost shard =
oldest instruction, always** (slot i = shard i = i-th oldest in the bundle). This
keeps inter-shard routing static (no dynamic crossbar), makes the broadcasts above
cheap, and is what the decoder's positional program order relies on. Possible
load/freelist imbalance is an accepted cost.

## Recovery — Checkpoint Processing & Recovery (CPR), no ROB

- No reorder buffer. All instructions carry a **checkpoint number**. Recovery is
  to a checkpoint (coarse, not precise-per-instruction).
- **Checkpoint policy is a knob, kept out of the mechanism:** the bundle/FTQ
  carries a "create checkpoint here" flag the renamer acts on; policy lives
  upstream. Bootstrap = every K fetches. Adaptive (AIMD-style: grow when stable,
  shrink/halve on restart) is attractive since restart cost ∝ checkpoint size, but
  not baked in. Constraints regardless of policy: `NCHK` bounds run-ahead — no free
  checkpoint bank ⇒ **rename back-pressures** (size × `NCHK` = instruction window);
  serializing instructions force a checkpoint boundary + drain.
- Checkpoints (and thus instructions) **retire strictly in order** — only the
  oldest checkpoint can commit, when all its instructions have completed.
- A destination's displaced previous mapping (`pold`) is **held pending in the
  checkpoint and bulk-released** when the checkpoint commits.

## Redirect / recovery flow

Triggered by a branch misprediction detected in execution, or an exception/fault.
It is a **race-rich** path; the mechanisms below are what keep it correct.

**Parallel actions on a branch mispredict:**
1. Stop fetch; **bump the prediction epoch** (flushes the frontend, fetch→rename —
   see below).
2. **Roll back to the checkpoint C containing the branch** (direct jump, see
   below), restoring map + freelist + BP history + store-buffer tail.
3. Install the correct target as a **BP override keyed by `seqno`**.
4. Restart fetch from **C's PC**.

**Why `seqno` keying works (by design):** `seqno` is a program-order counter that
is part of the rolled-back state, so re-fetching deterministically from C
reassigns the branch the *same* `seqno`. The override therefore lands on exactly
the re-fetched instance. Combined with restored BP history (deterministic
re-fetch), the branch is re-reached and corrected → forward progress, no
re-mispredict livelock. The BP *training* update is deferred until the next CFT
(basic-block prediction needs the block length).

**Prediction epoch (frontend flush):** a small counter incremented on every
flush; every fetch→rename item carries the epoch it was born under; `live = epoch
== current`. The compare must **gate admission into the renamer** atomically with
the flush (the frontend/backend seam) so no stale bundle enters the speculative
window; stale items already in frontend buffers simply drain and are ignored.
Size the epoch wider than the max in-flight flush generations so a wrap can't
alias a stale item as valid.

**BP history:** restored via the BP's **own lockstep checkpoint**, driven only by
the shared `create(ckpt#)`/`rollback(ckpt#)` events — BP internals (GHR/TAGE/…)
never cross the unit boundary.

**Direct-jump rollback (snapshots, not deltas):** restore C's MAP snapshot
(parallel flop load from the LUTRAM row, separate from the W rename write ports,
~1–2 cycles), OR the younger `A[]` into `free` and drop younger `A`/`P`, clear the
younger `ckpt_alive` bits. **Recovery latency is independent of squash depth.** An
older mispredict discovered mid-recovery just re-points the jump target at the
older C (no incremental unwind to abort).

### `ckpt_alive` — the unifying invalidation

Keep a small `ckpt_alive[NCHK]` bit-vector (set on create, cleared on
rollback-kill). Every speculative structure tags entries with `ckpt#`, and
`entry_live = entry.valid AND ckpt_alive[entry.ckpt]` (a 1-bit indexed read, no
wrap/range compares). Rollback to C clears the younger alive bits → machine-wide
invalidation in one signal.

**Evacuation list on rollback to C** (all keyed off `ckpt#`/`ckpt_alive`):
- **Scheduler** — issue-select gated on `ckpt_alive` (dead entries can never
  issue); slot reclaim = one-shot `valid &= ckpt_alive[ckpt]` across the small
  per-shard window. **Survivors need no scoreboard fixup**: program order means a
  survivor (older than C) can't depend on a squashed producer (younger than C).
- **In-flight execution ops + LSU loads** — their `(pr,value)` writebacks and
  wakeups carry `ckpt#`; receivers drop them when `!ckpt_alive`. **Caveat:** these
  must be suppressed/squashed *before* their physregs are reused, or a stale write
  corrupts restored state.
- **Store buffer** — rewind tail + the global store-seq counter to C. No cache
  un-write needed: commit-gated drain means speculative stores never landed.
- **Freelist** — drop younger `A`/`P` (folded into the direct jump above).
- **MAP** — restore C's snapshot.
- **RF** — nothing; rename guarantees write-before-read, so stale physreg
  contents are harmless once reallocated.

**Property:** recovery never touches memory — only the store-buffer tail +
store-seq counter rewind.

### Exceptions (two-pass precise trap on coarse CPR)

On a fault/"missing" (page/access fault) at instruction X in checkpoint C:
same flow, but the BP is **not** updated, and on replay we **force a checkpoint
boundary right after X** (X identified by its rolled-back `seqno`). That seals the
segment before X as a precisely-committable checkpoint, so X traps with exact
pre-trap state. Optimization: **if X is already the last instruction in its
checkpoint, no rollback is needed** — the boundary is already aligned; commit
through and trap.

## Freelist — per-shard bitmap + `A`/`P` reclamation

Bulk release wants set-operations, so the freelist is a **bitmap over the shard's
pool** (32 bits), not a FIFO/table. (Supersedes an earlier head-pointer-snapshot
idea, which was a FIFO artifact.) Per shard, per checkpoint, keep two bitmaps:

- `A[C]` — registers **allocated** during checkpoint C's span.
- `P[C]` — `pold`s **displaced** during C's span (the bulk-free set).

Operations (all 32-bit bitmap ops; only `ffs` is non-trivial and it's a single
small priority encoder):

- **Allocate**: `r = ffs(free)`; `free[r]=0`; `A[cur][r]=1`. One per shard/cycle.
- **Commit oldest C**: `free |= P[C]`; drop `A[C],P[C]`.
- **Rollback to C**: `free |= (A[C+1] | … | A[cur])`; drop younger `A`/`P`;
  restore the maps. (Off the critical path; ≤ NCHK−1 parallel ORs.)

`free` is a single live structure that already carries every older commit, so
there is **no stale snapshot to refresh** — this is why `A` (allocated-since)
beats snapshotting `free`: a register freed by an older checkpoint after a younger
one snapshotted, then reallocated, then squashed, would leak under a `free`
snapshot; tracking allocations avoids it entirely.

**Cross-shard:** `pold` belongs to whatever shard last wrote `rd` (`owner(pold)`
encoded in the pr). Shard A's displacement sets a bit in `owner(pold)`'s
`P[current_ckpt]` — a demux + bit-set, ≤ S bit-sets received per shard per cycle.

## Checkpoints — MAP storage

The MAP checkpoint is intrinsically a full copy (arbitrary entries mutate — no
pointer trick). Since **restore is off the critical path and may be multi-cycle**,
store the NCHK MAP snapshots as **wide LUTRAM rows** (create = one wide row-write;
restore = read row, fan out over a few cycles) rather than flop banks — denser,
frees the FF fabric. Freelist checkpoint state is the `A`/`P` bitmaps above.

## Scheduler — non-speculative scoreboard

- A **scoreboard**, not a single-cycle wakeup-select CAM loop. Readiness/timing
  updates are **registered for next cycle** (hence not a logic timing path).
- **Fixed latencies.** Wakeup is driven from the pipe one cycle ahead of
  writeback, off the known latency — *not* speculatively from the scheduler.
- **Loads have fixed latency 3** (AGU + tag/cache). `ready` is asserted only on a
  **confirmed hit**; a **miss defers `ready`** (dependents simply wait — no
  speculative wakeup, no replay). Cost: lost ILP on hits we could have bet on;
  gain: no replay machinery. (Open: exactly how a miss re-drives `ready` on fill.)

## Loads & the register file — scheduled writeback lane (no speculation)

The load result needs a write port into its owning shard's RF. Fixed latency
makes this a **scheduled resource**, not an asynchronous one:

- The load issues *from its owning shard* and writes back through **that shard's
  existing single WB lane** — no extra RF ports. The cache data array is shared;
  only the WB port is the shard's.
- At issue, the scheduler reserves WB slot `N+3` in a small per-shard WB
  reservation shift register (depth = max fixed latency) and won't issue an op
  whose WB slot is taken. Cost = an occasional ALU issue stall (pure structural
  hazard). No `LOAD_COMPLETE` second issue, no preempt, no recovery.
- **Misses** are the only unscheduled writeback: when a fill returns it
  **preempts one issue cycle** to drain — cheap because misses are already slow
  and rare.

Conclusion: **do not speculate loads.** Speculation only buys back the small
structural-hazard throughput on the WB lane, at the cost of reintroducing the
replay/recovery machinery the fixed-latency stance exists to avoid.

## LSU / store buffer — **UNIFIED** (decided 2026-06-21, TT)

Execution/RF/rename/scheduler shard; **memory does not.** Disambiguation is
inherently global (a load must check *every* older store regardless of shard), so
a single store buffer + load queue + L1D port set is the right structure — the
Alpha 21264 answer (cluster the integer datapath, keep one load/store queue).
This *resolves the earlier contradiction in this doc* (the cross-shard-comms
section's "LSU does not shard" wins; the sharded-store-buffer sketch is dropped)
and **dissolves both former open questions**: store-to-load forwarding is a single
CAM over one buffer (no cross-shard data-locality fetch), and L1D read-port
contention becomes a structural issue-stall gated by the *same* WB-reservation
the scheduler already runs. Throughput cost is small (~1–1.2 mem ops/cycle avg);
simultaneous mem-issue stalls rather than demanding 4× ports / 4-way CAMs.

- **Global store sequence number** gives total store order; a load records the
  seq# of its youngest-older store. The buffer tracks outstanding stores in order
  with **address and data readiness tracked separately** (store splits into
  addr-gen rs1+imm — already computed in `exec_alu` — and data rs2).
- **Register-readiness vs. memory-readiness are decoupled.** The scoreboard
  scheduler issues a load when its *address operand* (rs1) is ready; the LSU owns
  memory ordering downstream (the load takes a load-queue slot and the LSU
  resolves ordering + forwarding before driving WB). Memory ordering is *not*
  forced onto a register scoreboard bit.
- **Non-speculative ordering (first cut):** a load executes only once all older
  stores have *resolved addresses* — no memory-order violations, no replay path
  (we already have branch recovery; we don't add a second one). Speculative
  disambiguation is a later perf lever, not a correctness need.
- **The store buffer is a CPR structure** (parallel to the freelist): stores never
  hit memory until commit; **commit advances a "committed-through store#"
  pointer**, a drain engine writes ≤1 store/cycle (commit width ⊥ cache write
  ports), and **rollback rewinds the tail** to the branch's store#. Driven by the
  existing `commit`/`rollback`/`ckpt#` signals from `commit_ctl` — no new recovery.

### Forwarding is byte-granular — and under CPR it must be *complete*
The `resolved_through` gate (load waits until all older stores have resolved
addresses) is cheap: one shared find-first-unresolved pointer + one comparator per
waiting load. The forwarding itself is **per byte**, with no word/line framing
(we support **arbitrary alignment — misaligned memops are not trapped**, matching
the current core; only page-crossing traps, and that lives with the dTLB):

```
for each byte address X in [A, A+N):
    value[X-A] = data of the YOUNGEST older store (store# ≤ S_max)
                 whose [saddr, saddr+ssize) contains X ;   else  mem[X]
```

So a load is always memory bytes overlaid with per-byte youngest-older-store
forwards — two older stores overlapping each other and the load, with some bytes
falling through to memory, just works; there is no special case unless you impose a
word model. The only legitimate stall is on store **data** (`data_rdy`), whose
producer is older than the load → cannot cycle.

**CPR consequence (a real ROB-vs-CPR drawback, but a narrow one):** the common
path is identical to a ROB machine — both forward from the store queue and neither
waits for a store to *commit/retire* to satisfy a normal load. What CPR removes is
the **fallback**: a ROB machine that can't forward can stall the load until the
blocking store *retires to cache*, and per-instruction retirement makes that
deadlock-free. Under coarse checkpoints **that fallback deadlocks** — a same-
checkpoint store can't drain (drain is commit-gated) until the checkpoint commits,
which waits on the load. *(The deadlock is armed precisely once loads count
completion correctly — see below — rather than at issue.)* Therefore **store-to-load
forwarding must be complete** (the full per-byte merge); a load may wait only on
store *data*, **never on a drain**. The escape is only ever needed for op classes
forwarding fundamentally can't serve — **atomics (LR/SC, AMO)** and **MMIO/
uncacheable loads** — and there the **checkpointing policy** is the tool: forcing a
checkpoint boundary before such an op puts preceding stores in strictly-older
checkpoints that commit/drain independently, reinstating ROB-like granularity
*selectively* (cost: a checkpoint slot + serialization). It's a dispatch-time
decision, so it's clean for opcode-known classes (AMO/LR/SC); a plain load that
turns out to be MMIO isn't known until address-resolve and needs a resolve-time
"serialize when oldest" path instead.

### Completion accounting is per op-class (loads/stores complete at the LSU)
`commit_ctl` counts an instruction done when it *completes*; the **source of the
decrement varies by class**. ALU/branch complete **at issue** (fixed latency, no
fault in the subset). **Loads complete at LSU writeback**, **stores complete when
their buffer entry is ready to drain** (addr_rdy && data_rdy) — *not at issue*. So
memory ops carry a "completes-later" bit: the issue-time decrement skips them, and
the LSU supplies a second decrement source. (This is the "revisit when loads can
page-fault" caveat made concrete; faulting ops will likewise complete at
fault-resolution.)

### Addressing is physical — disambiguation on virtual addresses is unsound
Synonyms (two VAs → one PA) mean a load comparing **virtual** addresses could miss
a forward from an aliasing older store and read stale memory. So the store buffer
and load queue hold **physical** addresses, and disambiguation/forwarding/the
`resolved_through` gate are all physical. What changes between cache schemes is only
*where the PA comes from*, not the LSU — the LSU is alias-safe by construction and
the alias question lives entirely inside the D$.

**VHPR is the leading D$ scheme** (revised — it was prematurely deferred). A
virtually-*indexed* D$ that stores the **physical tag (PPN)** per line supplies the
PA as a byproduct of a hit: read out the stored PPN, no dTLB on the common path. On
a miss you translate via the miss-path TLB/PTW — the structure you'd have had anyway
— and you're filling a line regardless, so the miss cost is unchanged (the dedicated
dTLB merely did its lookup up front; the fill dominates either way). Net: the hit
path is *strictly cheaper* (no TLB), and resolution coverage is the union of (D$ hit
⇒ PA) and (TLB hit on miss ⇒ PA), not limited to the cache's line reach. Bonus: the
translator now serves only **D$ misses**, not every AGU access, so a 4-wide machine
no longer needs a multi-ported dTLB at AGU — it drops to miss-rate bandwidth and can
fold into the PTW.

The **synonym/alias problem is definitional, not open**: VHPR maintains
**single-resident-alias** — a probe-by-physical-tag on miss guarantees a physical
line is resident under exactly one VA, so an aliasing access simply *misses and
migrates* the line; the stale-second-copy case cannot arise. The only cost is alias
ping-pong on a program actively sharing a physical line under two VAs — a
performance footnote, not correctness. (A store could go PIPT-style by translating
first, but the VA-indexed access is cheaper and the miss path translates anyway.)

The **I$ is asymmetric**: read-only ⇒ no store queue, no disambiguation, no
aliasing-wrong-value (stale lines only matter for `fence.i`, handled by flush), so a
virtual I$ is fine and easy to reuse. Milestone 1 sidesteps all of this by running
the **dTLB as identity (bare mode)** against a **flat byte-addressable** stub:
VA==PA, arbitrary alignment is free (no lines), disambiguation is trivially physical.
Cache-line mechanics (misaligned **line-crossing** → two reads + merge), miss
latency, and real translation (with the inherited **page-crossing trap**, single
dTLB) all arrive together at the real-D$ milestone.

### LSU ↔ cache contract (designed now, cache stubbed during validation)
Like `fetch`'s `imem`, the LSU talks to memory through an abstract port; a
behavioral model stands in while the LSU core is built, and the real dual-bank
D$ + dTLB drop on later (separable per the cache-extraction plan). The contract is
harder than I$↔fetch and so must be fixed up front:
- **Variable latency:** assume **hit at fixed latency N+3**, reserve the WB slot;
  on **miss or ordering-stall the load defers** (doesn't assert ready, re-arbitrates
  later) — deferral, *not* replay. A returning fill preempts one issue cycle.
- **Write side gated by CPR:** the drain engine is the only writer, post-commit.
- **Physical addresses first:** run the LSU post-translation with the dTLB
  stubbed/bare-mode; integrate dTLB+D$ together as a later milestone.

### Build order (mirrors the CPR integration)
1. **DONE** — `lsu.v` (unified store buffer + load queue, pool/seqno-ordered) vs. a
   **flat byte-addressable** dmem (dTLB = identity): order gate (no older unfilled
   store), **complete byte-granular forwarding** (per-byte youngest-older-store ∪
   memory), commit-gated drain, rollback squash-by-seqno. Wired into `backend_top`:
   LSU slots allocated at dispatch (mem_idx threaded through the scheduler), AGU +
   store-data from the shards, load WB muxed onto the owner lane (busy-gated, no
   collision), loads counted at LSU completion. `tb_lsu` + end-to-end `tb_ldst`
   (store → forwarded load → dependent) green. M1 simplifications still standing:
   stores issue-on-both; combinational (fixed-1-cycle) load; serialize fences/
   atomics/MMIO via a forced checkpoint is a **TODO** (unused by the M1 tests).
2. Scheduler WB-slot reservation + fixed N+3 hit latency (needed once load latency
   goes variable; M1's combinational load needs no reservation).
3. Miss deferral (variable latency on the stub).
4. Real D$ (**VHPR — virtually-indexed, physical-tag/PPN per line; single-resident
   alias**) + miss-path translator: cache lines, line-crossing two-read+merge, miss
   latency, translation (PA from stored PPN on hit) + page-crossing trap.

Forwarding implementation is a separate axis from correctness: the **sequential
byte-merge** (init a byte buffer from memory over the load's range, replay older
overlapping stores oldest→youngest, youngest wins per byte) is correct for
arbitrary alignment and cheap in gates; the **parallel per-byte age-priority
network** replaces it later for latency, same semantics.

## Aligner → decoder interface

Per slot: `{ valid, seq, inst[31:0] }`.
- `inst[31:16]` is ignored when `inst[1:0] != 2'b11` (compressed).
- The aligner may invalidate a slot for many reasons (`valid`).
- `seq` (width parametrized) is for debugging now; may gain a microarchitectural
  use later — carried opaque.

## Decoder (next to build) — output IR & scope

The decoder's output is the renamer-facing contract. Per slot:

```
valid                       // slot is a real instruction
seq, ckpt#                  // carried opaque from the aligner
src1: { valid, kind: ARCH(a[5:0]) | SLOT(j) }   // unified 0..63 arch space
src2: { valid, kind: ARCH(a[5:0]) | SLOT(j) }
dst:  { valid, arch[5:0], is_map_writer }       // is_map_writer = last writer of this arch in the bundle
uses_imm, imm
ctl                         // decoded execution control (op/unit/latency-class) — opaque to rename
```

Scope:
1. Decode-in-place: RVC `inst[15:0]` + full `inst[31:0]` (port the existing mask
   decode in `src/smolrv64.v`; aligner sends a 32-bit word, top 16 ignored when
   `inst[1:0]!=11`). Emit explicit per-operand valid bits (kills the x0 magic-zero;
   `f0`=arch 32 is a normal reg).
2. Cross-slot matrix (the O(W²) moved here from rename): each slot compares its 2
   sources against earlier slots' dests → `SLOT(j)`; compute `is_map_writer`
   (last-writer-per-arch in the bundle).
3. Carry-through `valid/seq/ckpt#`; ready/valid boundary.
4. `decode_shard_probe` for timing (combinational first; pipeline the matrix if
   it's a path — same method as the renamer).

**Assumption:** intra-bundle program order is **positional** — slot 0 oldest …
slot 3 youngest — so `SLOT(j)` requires `j < i` and last-writer = highest index.
(Depends on the steering mapping preserving program order; see frontend.)

## Validated results (timing probe, `probe/`, xcku5p-ffvb676-2-i, OOC)

| Design | Fmax | Area | File | Func |
|---|---|---|---|---|
| Monolithic 4-wide renamer (full intra-bundle bypass) | ~440 MHz | 4351 LUTs | `renamer.v` | — |
| **Sharded renamer slice** (decoder-resolved `{ARCH\|SLOT}`) | **~790 MHz** | 1469 LUTs/shard | `rename_shard.v` | — |
| **Cross-slot dependency matrix** | **~1.1 GHz** | 59 LUTs | `decode_xslot.v` | 9/9 ✓ |
| 4-wide renamer bundle (matrix + 4 shards + broadcasts) | ~397 MHz* | 11.5k LUTs | `renamer_bundle.v` | ✓ |
| RV64C expander (16b→32b) | not probed | — | `rvc_expand.v` | 65536/65536 ✓ |
| RV64I operand decode | not probed | — | `decode_operands.v` | 15/15 (directed) |
| Full decode stage (lanes + matrix) | not probed | — | `decode_stage.v` | composition ✓ |
| Decode→rename (registered boundary) | not probed | — | `decode_rename.v` | end-to-end ✓ |
| Fetch-window aligner (RVC/32b carve) | not probed | — | `aligner.v` | directed ✓ |
| Fetch (PC seq + carry-free window) | not probed | — | `fetch.v` | directed ✓ |
| **Full frontend** (PC→align→decode→rename) | not probed | — | `frontend.v` | end-to-end ✓ |
| Scheduler shard (scoreboard issue queue) | not probed | — | `sched_shard.v` | directed ✓ |
| Execution ctl decode | not probed | — | `decode_exec.v` | directed ✓ |
| Execute datapath (reuses `src/alu.v`) | not probed | — | `exec_alu.v` | directed ✓ |
| PRF shard (S²-banked, LUTRAM) | (in exec_shard) | — | `rf_shard.v` | directed ✓ |
| **Execute shard** (RF+ALU+wb) | **272 MHz\*** | 2288 LUT / 39 CARRY8 / ~0 FF | `exec_shard.v` | directed ✓ |
| Execute shard @ NPHYS=256 | 271–282 MHz\* | 2288 LUT (==128!) | `exec_shard_probe.v` | — |
| **ALU alone** (`exec_alu`) | **364 MHz\*** | 2125 LUT / 39 CARRY8 | `exec_alu_probe.v` | — |
| Execute shard, full bypass (experiment) | 210 MHz\* | 2852 LUT | `exec_shard_bp.v` | — |
| Execute bundle (RF+ALU+wb net) | not probed | — | `exec_bundle.v` | cross-shard ✓ |
| Scheduler bundle | not probed | — | `sched_bundle.v` | (in backend) |
| **Full core, ALU subset** (FE→sched→exec) | not probed | — | `backend_top.v` | **end-to-end ✓** |
| Branch unit (resolve taken/target) | not probed | — | `branch_unit.v` | directed ✓ |
| **Branches + redirect + rollback** (CPR) | not probed | — | (`backend_top`) | **end-to-end ✓** |
| Bitmap freelist + A[C]/P[C] reclamation | not probed | — | `freelist.v` | directed ✓ |
| Commit control (per-bundle ckpt, by-issue) | not probed | — | `commit_ctl.v` | directed ✓ |
| **Commit/CPR integrated** (reclaim + back-pressure) | not probed | — | (`backend_top`) | **end-to-end ✓** (`tb_reclaim`) |
| Pipeline trace (observability) | — | — | `tb_trace.v` | — |

The sharded slice's critical path is the W-port MAP write-enable decode — only 3
LUT6 levels, 77% routing. The flagged "true N write ports of flops" is cheap at
W=4 / 32-entry. The cross-slot matrix (the O(W²) moved out of rename) is
functionally verified and ~free (`tb_decode_xslot.v`, 9/9).

\* The bundle's in-context number is **congestion-limited and unrepresentative**,
for two reasons: (1) the probe harness forces all 4 shards into one clock region
(`CLOCKREGION_X0Y0`) — the real design spreads shards across regions as distinct
clusters; the worst path is 75% routing / 6 logic levels = congestion, not logic.
(2) the bundle probe is one combinational input→MAP block; the real design
**registers the cross-shard broadcasts** (next-cycle updates, not single-cycle
logic paths). A faithful in-context number needs a multi-region floorplan +
pipelined broadcasts. The component numbers (slice 790, matrix 1.1 GHz) + the
registered-broadcast architecture are the real timing story. The bundle is
functionally validated end-to-end (`tb_renamer_bundle.v`): RAW-through-bundle,
ARCH identity read, cross-cycle MAP update, WAW youngest (intra-bundle +
committed).

Note: the old slice/monolithic Fmax were at the 32-entry MAP / 64-phys geometry;
`renamer_bundle` is the unified `AREGS=64` / `NPHYS=128`. All far enough above the
400 MHz bar (per-component) that we don't sweep further.

## Build order / status

1. Renamer slice — **done** (probe + RTL, ~790 MHz).
2. Cross-slot dependency matrix — **done** (`decode_xslot.v`, 9/9, ~1.1 GHz).
3. 4-wide renamer bundle — **assembled + functionally verified**
   (`renamer_bundle.v` + `tb_renamer_bundle.v`). In-context timing deferred until
   a multi-region floorplan + registered broadcasts exist (single-region probe is
   pessimistic — see results note).
4. RV64C expander — **done** (`rvc_expand.v`, combinational 16b→32b). Verified
   **exhaustively, 65536/65536** against the oracle `tools/rvc.rs` (→
   `rvc_cases.hex`) by `tb_rvc_expand.v`. Expand-then-decode: this feeds the
   32-bit operand decode, so the decoder only handles full-width forms.
5. RV64I operand decode — **done** (`decode_operands.v`): 32-bit word →
   unified-arch `rs1/rs2/rd` + explicit per-operand valids (x0 dest dropped;
   shift-imm/CSR-imm read no rs2/rs1) + sign-extended `imm`/`has_imm` + `legal`.
   Directed TB 15/15 (`tb_decode_operands.v`). **Verification gap:** directed only
   — needs cosim vs `src/smolrv64.v` for full coverage. FP regs (arch 32..63),
   AMO, SFENCE operands, and the execution `ctl` blob are TODO.
6. Full decode stage — **done** (`decode_slot.v` per lane, `decode_stage.v` =
   IW lanes + `decode_xslot`). Produces the renamer's input contract; composition
   TB (`tb_decode_stage.v`) passes on a mixed RVC/32b dependency-rich bundle.
7. Decode → rename — **done** (`decode_rename.v` = `decode_stage` → registered
   boundary → `renamer_bundle`). `renamer_bundle` no longer embeds `decode_xslot`;
   it takes the cross-slot result (`s*_is_slot/s*_slot/map_writer`) as inputs, so
   the O(W²) matrix is computed once in decode and the register cuts it out of the
   rename critical path. End-to-end TB (`tb_decode_rename.v`): intra-bundle SLOT
   RAW resolves to producer `pdst`s, dest arch/seq carry through the boundary, and
   a second bundle reads the first's MAP writes one rename cycle later. The bundle
   tb/probe now drive the new inputs via a **stimulus-side** `decode_xslot`.
   *Note:* the boundary register advances every cycle — back-pressure (freeze on
   stall / no free checkpoint / <2 free regs) is the next pending item.
8. Aligner + fetch + frontend — **done**. `aligner.v`: O(W) length prefix-scan
   carving up to IW RVC/32b instructions from an HW-halfword window; uniform
   32-bit data window (length depends on `[1:0]`, data does not); validity is a
   prefix, a window-straddling 32b op is excluded from `consumed`. `fetch.v`: PC
   sequencer with **carry-free windowing** — the window always starts at PC and
   PC advances by `2*consumed`, so a straddler's first halfword simply reappears
   as the next window's slot 0 (no leftover/shift buffer). Fall-through +
   `redirect`; ready/valid downstream. `frontend.v` = `fetch` + `decode_rename`;
   `tb_frontend` streams a real RV64IC program and gets the same rename as feeding
   `decode_rename` directly. (Fix along the way: `decode_rename` gained a reset
   that squashes the boundary — without it fetch presented bundle 0 during reset
   and rename allocated it twice.)
   - **Instruction memory is a behavioral TB stub.** `fetch` reads via an external
     **combinational** interface (`imem_addr` → `imem_data`[HW halfwords] /
     `imem_avail`); the TB supplies a `reg` array. **No cache integrated.** The
     real SmolRV64 dual-bank even/odd I$ is the integration target for this
     interface and needs: a request/response handshake (not single-cycle-present),
     fetch-side stall on cache-not-ready, and miss handling. `imem_avail` already
     models a short (sub-window) read. Note the memory's "I$ already in frontend"
     refers to the *existing* smolrv64 frontend, a different module.
9. Scheduler shard — **done** (`sched_shard.v`): non-speculative scoreboard issue
   queue, eager allocation (keys off physical-register readiness — no slot naming,
   no PR↔slot CAM). Shared `ready[]` indexed by each entry's source PRs (wide mux
   per source: linear in NPHYS/IQ depth, not the quadratic of a matrix, not a
   per-entry wakeup CAM). 1 dispatch + 1 issue/cycle, oldest-first by seq;
   cross-shard scoreboard updates ride SHARDS-wide `clr` (dispatched dests) /
   `wake` (issued dests) broadcasts (self looped). v1 wakeup = 1-cycle (ALU);
   fixed multi-cycle load latency needs a small per-source delay line on the wake
   path (goes in with the LSU — `disp_lat`/`iss_lat` already carried so the
   interface won't change); WB-slot reservation deferred. `tb_sched_shard`:
   dep-chain wakeup, oldest-first contention via a sibling producer, IQ-full
   backpressure.

   **Allocation/scheduler decision (settled with TT, see plan below):** name by the
   commit-lifetime physical register the bitmap freelist already mints (single MAP
   write); the scheduler keys off PR readiness. *Matrix scheduler rejected* — N×N
   in IQ entries, only practical at ~8-12; indexing columns by PR makes columns =
   NPHYS (128×rows), past the quadratic wall. *Late/virtual allocation deferred* —
   it's a storage optimization (smaller, S²-banked PRF), addable later as a
   pure RF-path change (VR→PR table at writeback) without touching rename or the
   scheduler, because the MAP already names a commit-lifetime token. Slot-naming
   would force either a double MAP write or a PR→slot translation CAM (= P6
   RAT-at-retire); avoided.
10. Execution datapath — **done** (`decode_exec.v` + `exec_alu.v`), reusing
    `src/alu.v` (RVA22 + Zicond). `decode_exec` is the 32-bit-only ctl decode (RVC
    expanded upstream), op map ported from `smolrv64.v`'s pre-decode; folds
    `EXOP_OPB` away (LUI=`ADD` 0, AUIPC=`ADD` pc, link=`next_pc`) and emits
    operand-select + class flags. `exec_alu` = operand select → `alu.v` →
    result/AGU; `sum`=address, `eq`/`lt`/`ltu` exported for the branch unit.
    **Scope:** full RVA22 integer ALU + LUI/AUIPC/JAL(R)-link + load/store address
    gen + branch compares. **Store split:** addr-gen (rs1) and data (rs2) are
    separate deps so a store occupies its LSU slot without waiting on data.
    **CSR reads** ride an ALU lane; **CSR-write / system / fence** → `is_serialize`
    so steering pins them to shard 0. Deferred (flagged, routed to future units):
    actual memory (LSU/D$), branch redirect, CSR file, M/A/F. `tb_exec` passes on
    ALU/upper-imm/`*W`/Zbb/load-store-addr/branch/JALR/CSR/MUL. NB: probe TBs now
    need `-I ../src` (for `alu.v` / `alu_ops.vh`).
11. PRF + writeback — **done** (`rf_shard.v` + `exec_shard.v`). Per-shard RF copy
    holds all NPHYS regs as SHARDS **owner-banks** (owner = `pr[SBITS-1:0]`, idx =
    `pr[PBITS-1:SBITS]`), each bank single-write (its owner's wb lane) → no
    multiport arbitration, cost = S² single-write banks + broadcast. 2 comb read
    ports, pr0 reads 0. `exec_shard` = `rf_shard` + `exec_alu` + wb. Forwarding is
    write-before-read (no bypass net). `tb_exec_shard`: self/cross-shard fwd,
    persistence, x0=0. **Fit/timing probe (`exec_shard_probe.v`, OOC single
    region):** Fmax **272 MHz\***, **2288 LUT / 39 CARRY8 / ~0 extra FF** — the
    128×64 RF infers as **LUTRAM** (the FF count is just the harness), so the
    S²-RF is cheap (~9.2k LUT for the 4-shard backend, <5% of xcku5p → fits
    easily). \* single-region congestion-limited: 72% routing, logic only
    1.03 ns / 10 levels — pessimistic (like the `renamer_bundle` probe); a
    multi-region floorplan lifts it. Structural limiter = RF-read + ALU in one
    cycle; lever if needed = split operand-read and ALU into two pipe stages (ALU
    latency 2). **Decision (TT-aligned): keep single-cycle for now**, optimize with
    the floorplan later. 1-cycle-ALU forwarding is just **write-before-read**
    (producer issues T, result registered into every shard's RF copy at edge
    T→T+1, dependent woken at the same edge issues T+1 and reads it) — no separate
    bypass net for the 1-cycle case.
**Execute timing experiments (single-region probes, routing-dominated → readings
are pessimistic but the *comparison* is informative):**
- *Bypass vs write-before-read:* a full-bypass 2-stage variant (`exec_shard_bp.v`,
  RR read | EX bypass+ALU, 2·SHARDS sources) probed **210 MHz / 2852 LUT — slower**
  than the 272 MHz write-before-read `exec_shard`. The 272 limit is **routing/
  congestion** (72% route; the RF-read logic was only 0.137 ns), not RF-read-in-
  series, so splitting it out doesn't help — and bypass is bigger so it congests
  worse, with the full ALU adder still in EX. **The real timing lever is the
  multi-region floorplan, not bypass.** Bypass theory (EX≈mux+ALU≈2.2 ns) only
  pays once routing is fixed; kept as an experiment to revisit then.
- *256 physical registers:* `exec_shard` at NPHYS=256 probed **271–282 MHz / 2288
  LUT — identical LUTs to the 128-PRF build.** A LUT6 is natively a 64-deep RAM, so
  the 128-PRF's 32-deep banks under-filled it; 256 fills it at ~zero extra
  area/timing. **DONE: NPHYS=256 adopted globally** (PBITS 8, POOL 64, HPTR/IDXB 6
  defaults across renamer_bundle/decode_rename/frontend/rf_shard/exec_shard/
  sched_shard; all TBs pass). Late allocation is now **moot, not deferred**: on a
  resource-rich FPGA 256 regs cost ~0 LUTs, so there's no PRF storage to reclaim.
- *333 MHz target (match the memory-interface clock, single clock domain):* the
  ALU alone (`exec_alu`) probes **364 MHz — clears 333**; the single-cycle
  RF-read+ALU `exec_shard` is **272 MHz**. So the RF read in series is what misses
  333, not the ALU. A **2×2-region pblock did not change `exec_shard` (271 MHz)** →
  the flopwrap number is placement-unrepresentative (the RF read-address fans to
  all 64 bit-columns and the harness scatters it), not single-region congestion.
  **Path to 333: split execute into RR (RF read) | EX (ALU) with single-level
  result forwarding** (EX→WB reg into the EX operand mux) to keep dependent ALU
  ops at 1/cycle. Cost = ALU latency 2 (small CPI hit) in exchange for one clock
  domain at 333. NB: this is *single-level light* bypass — not the heavy 2-level
  priority bypass that probed 210 MHz. Build the 2-stage execute at integration;
  keep single-cycle `exec_shard` as the functional reference.
12. Backend integration — **done** (`backend_top.v`): `frontend → sched_bundle →
    exec_bundle → wake`, ALU subset. The exec payload (ctl+imm+pc) rides through
    rename (registered) into the scheduler as an opaque IQ field (`exec_pay.vh`),
    emitted at issue. `exec_bundle` = 4 `exec_shard` + the wb broadcast (= RF-write
    feed *and* scheduler wake); `sched_bundle` = 4 `sched_shard` + clr(dispatch)/
    wake(writeback) net. `tb_backend` runs a real RV64I program (4 independent +
    4 cross-bundle-RAW `addi`) and checks the `(pr,val)` writeback stream — RAW
    resolves via scoreboard + write-before-read forwarding. **End-to-end verified.**
    - *Latent bug fixed:* `rename_shard` freelist started at phys 0, but phys
      0..AREGS-1 are the initial arch mappings (`map[r]=r`) — allocating them
      corrupts live state / hands out x0. Now `head` starts past the
      `ARSH=AREGS/SHARDS` arch-mapped regs per shard (phys 0..63 reserved, incl
      x0); `rf_shard` inits banks to 0. First alloc is now `64+shard`.
    - *Now done (item 14):* renamer back-pressure + commit/CPR + freelist→bitmap
      reclamation are wired and verified.
13. Branches + redirect + rollback — **done & verified** (`branch_unit.v` +
    `backend_top` wiring). Conditional branches resolve in execute; `exec_bundle`
    picks the **oldest** mispredicting branch → one redirect (predict-not-taken, so
    a taken branch / any jump is the mispredict). On redirect: fetch → target with
    seqno rolled back to `branch_seq+1`; decode→rename boundary flushed; rename
    MAP+freelist restored from the speculative checkpoint taken at the branch
    (`chk_create`/`chk_restore`, single slot); scheduler invalidates IQ entries
    with `seqno > branch_seq`. ALU writebacks are self-contained (touch only their
    own soon-to-be-freed physreg), so no execute-side wb suppression yet. `tb_branch`
    proves it: a taken `beq` over wrong-path `x20` writes, a later consumer reads
    the **target's** `x20` (88), never the squashed wrong-path values (131/132);
    redirect target verified. (Also fixed: `sched` `PAYW` was a stale 142 silently
    truncating 20 payload bits — now tracks `exec_pay.vh`.)
    - *Mid-bundle branches — **done** (truncation in the aligner):* the aligner
      predecodes control transfers (opcode bits only, incl. RVC C.J/C.BEQZ/C.BNEZ/
      C.JR/C.JALR) and **terminates the fetch bundle at the first CTI**, so a branch
      is always the youngest instruction in its checkpoint. Mid-bundle recovery then
      folds into the already-exact "branch last in bundle" case — **no sub-bundle MAP
      snapshot, freelist, or commit-count machinery**. The straggling tail reappears
      as slot 0 of the next window (`consumed` stops at the branch, like the straddle
      case); it is also the start of basic-block fetch. Without this, a younger
      same-bundle slot is seqno-squashed in the scheduler but **still counted** in the
      branch's checkpoint, so the count never drains → in-order commit deadlocks.
      `tb_branch_mid` (taken `beq` at slot 2; older slots 0/1 must commit) is the
      discriminator — it checks `commit` fires (no deadlock) and the wrong path never
      reaches arch state. Note the wrong-path instr *may* execute speculatively before
      the branch resolves; that is correct, the rollback abandons its physreg.
    - *Multiple in-flight branches:* now that each branch is its own checkpoint's
      youngest, up to **NCHK** branches can be in flight (each a distinct checkpoint;
      `exec_bundle` redirects on the oldest mispredict, rollback to `rckpt+1` discards
      all younger checkpoints). Deeper nesting beyond NCHK needs `ckpt_alive` evac.
      `tb_branch_multi` verifies it: three branches in three distinct checkpoints live
      at once (B0/B1 not-taken → commit, B2 taken → redirects to its own target 0x40),
      older committed path undisturbed, 10 commits drain in order. (Same-cycle
      oldest-wins among *multiple taken* branches — the double-redirect race — is not
      yet directly forced; it's timing-fragile and the oldest-mispredict select covers
      it by construction.)
    - **Truncation is a stopgap, NOT the end state** (Tommy): a checkpoint per branch
      makes checkpoint == basic block, so the window is capped at NCHK basic blocks
      (~NCHK×5–6 instr) and a checkpoint is burned on *every* branch even when the BP
      is accurate — wasteful, since the whole CPR win is spending the scarce checkpoint
      budget only where rollback actually happens. **General direction = selective,
      confidence-gated checkpoint placement, decoupled from bundle/branch boundaries:**
      open a checkpoint only at a *low-confidence* branch, a *forced-serializing* op
      (the atomics/MMIO/fence checkpoint — already an instance of on-demand placement),
      or a *distance cap* (every N instr, for forward progress + bounded replay); let
      many confidently-predicted branches ride inside one checkpoint.
      - *Checkpointed* branch miss → today's cheap precise recovery (restore `rckpt+1`,
        redirect to target).
      - *Non-checkpointed* branch miss → **coarse rollback to the nearest enclosing
        (older) checkpoint `C`, then re-execute forward** from `C`'s start PC, replaying
        the correctly-predicted C→branch work. Rare under confidence gating, so net win.
      - *Why replay is mandatory:* the MAP is snapshotted only at `C`, so restoring
        `chk_map[C]` discards the C→branch renames — they must be re-executed to rebuild
        the MAP. LSU rollback-by-seqno + freelist A[C]/P[C] already compose with a
        coarser `rollback_seq = C_start_seq` (no new teardown). Need to store each
        checkpoint's **start PC/seqno**.
      - *The subtle bit:* the offending branch must be **forced to its resolved outcome
        on replay** (else it re-mispredicts → livelock). Record `(PC → dir/target)` in a
        small **fix-up override** the front-end applies at that branch on replay, then
        clears; the distance cap guarantees forward progress.
      - *Build order:* BP **confidence** (ties into the postponed basic-block/FTQ BP) →
        **decouple `create` from dispatch** (checkpoint opens on policy; counts/freelist
        accumulate across bundles) → **coarse rollback + replay + fix-up override**.
    - *JAL/JALR link-write restore — **done & verified** (no RTL change; truncation
      already covered it):* a jump is the youngest instr in its checkpoint, so its link
      write (`rd ← next_pc`, `exec_alu` `result = res_link ? pc+(rvc?2:4) : alu`) lives
      in the checkpoint that is **kept** on redirect (rollback to `rckpt+1`), and the
      preg holds the link value — so the target path reads `ra` correctly. `tb_jump`
      proves both forms end-to-end: JAL (direct, link 0x4 read on the target) and JALR
      (indirect, target = `rs1+imm`, link 0x18), each across its own redirect. (RVC
      `c.jalr` link = `pc+2` rides the same `is_rvc` payload bit, validated by
      `tb_rvc_expand` + RV64IC streaming.)
    - *Multi-cycle load WB suppression on squash — **done & verified**:* load
      completion is combinational off `lq_v`, but `lq_v` only clears at the next edge,
      so in the squash cycle itself a wrong-path load (seq newer than `rollback_seq`)
      could still be selected → assert `ld_wb_v` (RF write / wake) and `ld_done` (a
      spurious `commit_ctl` decrement on a checkpoint being squashed). Fix: the load
      selector excludes `rollback && older(rollback_seq, lq_seq[i])` this cycle (a
      correct-path *older* load is not gated and still completes). `tb_lsu` T5 drives a
      ready, gate-open load and asserts `rollback` same-cycle: `ld_wb_v`/`ld_done`
      suppressed, entry cleared (fails 3 ways without the gate). The edge-clear handles
      every later cycle; this closes the squash-cycle gap that opens once loads are
      truly multi-cycle (miss-deferral / WB reservation).
    - **Seqno wrap invariant (all program-order compares):** seqno has limited range
      and wraps, so every `a>b`/`a<b` must be the signed-difference form
      `$signed(a-b)>0` / `<0` (== `(unsigned)(a-b) <= MAX_POSITIVE`), valid **only
      while any two compared seqnos are ≤ `2^(SEQW-1)` apart**. Applied to all three
      seqno compares (scheduler issue-select & squash, execute oldest-mispredict).
      Constraint: `SEQW` must exceed `clog2(2 × max in-flight instructions)` (today
      SEQW=8 → window 127 ≫ ~36 in-flight). Wrap itself is not yet exercised by a TB.
14. Commit / CPR — **DONE & integrated end-to-end** (sustained execution past the
    freelist depth). Pieces:
    - **`freelist.v`** — bitmap free + A[C]/P[C]; a committed checkpoint bulk-frees
      its dead polds in one cycle (the array/ring couldn't). Inclusive rollback
      (recover before span C). Now lives **inside** `rename_shard` (the ring +
      `chk_fl` snapshots are gone; only the MAP + `chk_map` snapshot remain).
    - **`commit_ctl.v`** — one checkpoint per dispatched bundle; per-checkpoint
      outstanding count **incremented at dispatch (all valid instrs) and decremented
      at ISSUE — not writeback**, because nops/stores/branches never write back but
      all issue (issue is the universal completion event; "issued ⟹ done" holds for
      non-faulting fixed-latency ops — revisit when loads can page-fault). Oldest
      checkpoint commits in order when its count drains and it's closed; `full`
      back-pressure when the NCHK ring fills (1 slot reserved); rollback clears
      squashed checkpoints' counts.
    - **pold (intra-bundle WAW), the clean general case:** `decode_xslot` now also
      emits `d_is_slot/d_slot` (youngest earlier in-bundle writer of the destination),
      mirroring the source SLOT resolution. So `pold = d_is_slot ? al_phys[d_slot] :
      map[d_arch]` — every allocating instruction displaces *exactly one* register
      (the winner frees the intermediate pdst, each loser frees the pre-bundle MAP
      entry), so `#freed == #allocated` per span and the freelist balances. Polds are
      broadcast across the bundle (`pold_valid/pold_bus`, mirror of `al_phys`) so each
      shard's freelist records the polds it owns into `P[cur]`.
    - **MAP snapshot for rollback:** at each `create` (closing span `cur`) the
      *post-bundle* MAP (`nmap`) is snapshotted into the **next** span's slot
      `chk_map[cur+1]`; a branch in span C reopens span C+1 and restores
      `chk_map[C+1]` (= the map just after the branch's bundle) — robust even if no
      successor bundle has renamed. `chk_map[0]` starts at the identity map.
    - **back-pressure:** `accept = !any_valid || (!full && &disp_ready && !any_stall
      && !redirect)` freezes both `fetch.ready` and the decode/rename boundary; the
      bitmap alloc + MAP write only commit on `create` (= the bundle actually fires),
      so a held bundle re-presents without double-allocating.
    - **redirect granularity:** scheduler squashes by seqno (precise per-instruction);
      the freelist/MAP roll back by checkpoint (per-bundle) to `redirect_ckpt+1`, so a
      redirecting branch must be the youngest in its bundle (as today). The branch's
      ckpt# rides dispatch→IQ (`iqck`)→issue and out of `exec_bundle.redirect_ckpt`.
    - **Verified:** `tb_branch`/`tb_backend`/`tb_trace` (redirect+rollback through the
      new path), and `tb_reclaim` — 64 sequential `addi x1,x1,1` (all shard 1) run
      through a 48-deep pool to completion (x1=64, 112 commits), impossible without
      reclamation + back-pressure.
15. **Then:** LSU (loads/stores, store addr/data split, commit-gated drain) — the
    rest of "real programs"; generalize branch recovery (mid-bundle truncation /
    basic-block fetch, NCHK nested checkpoints via `ckpt_alive`, JAL/JALR precise) —
    **all branch-recovery sub-items now DONE+verified**; CSR/FPU units. **M extension
    DONE+verified** (`muldiv.v`: combinational RV64 datapath, all 13 ops, op =
    `{alu_w,br_func}`; `is_mul` threaded to payload bit 151; `exec_shard` muxes
    `wb_val`). **Multiply is combinational** (`mul.v`, DSP-friendly); **divide is
    iterative** (`divider.v`, restoring, ~64 cyc, start/busy/done/abort) and integrated
    with **deferred completion like a load**: issue starts the per-shard divider,
    `exec_busy` stalls the shard (freeing its WB lane), completion drives the owner lane
    + wakes (the scoreboard is writeback-driven, so no scheduler-latency change) and is
    counted at `div_done` in `commit_ctl` (excluded from the issue decrement via
    `iss_is_div`); a squash aborts an in-flight wrong-path divide. Subtle fix: a divide
    is *not started* if a branch squashes it the same cycle it issues (before `dv_busy`
    sets), else the divider wedges. Tests: `tb_mul`, `tb_divider` (+abort),
    `tb_muldiv_e2e`, `tb_divsquash`. (Pipelined multiply is a later area/timing opt; a
    shared divider would save 3 dividers.) Note (TT): integrating Mul/Div, FPU, CSR lengthens
    the issue→result path → 333 MHz gets harder; the 2-stage execute (RR|EX) is the
    lever. Deferred: branch prediction/FTQ; multi-region floorplan; real I$ on
    `fetch`'s imem; cosim for the operand-decode gap.

Open discussion threads (flagged by TT, not yet detailed): back-pressure across
stages; LSU store-to-load forwarding data locality + shared L1D read ports; the
multi-region floorplan for a real in-context bundle number.

Every stage gets a ready/valid (elastic) boundary. New RTL currently lives in
`probe/` (validate-first); migrates to `src/` at integration.
