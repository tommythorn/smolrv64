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

## LSU / store buffer

- Sharded execution, but stores occupy a **fixed slot in a circular store
  buffer** in store order (global **store sequence number** gives the total
  order). A load records the seq# of its youngest-older store. The LSU tracks
  outstanding stores in store order, with **address and data readiness tracked
  separately**.
- The store buffer **drains as checkpoints commit**. Committing N stores in one
  cycle would need N cache write ports — avoided by **decoupling commit from
  drain**: commit just advances a "committed-through store#" pointer; a separate
  drain engine writes ≤1 store/cycle into the cache. Commit width and cache write
  ports become independent.

Open questions (LSU deep-dive, details to follow):
- **Store-to-load forwarding data locality** — the matching store's *data* may sit
  in another shard's store buffer; forwarding is then a cross-shard fetch, or the
  store data must live somewhere centrally addressable by seq#.
- **Shared L1D read-port contention** when multiple shards issue loads the same
  cycle (separate from, and not solved by, the RF-WB answer above).

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
12. **Next: integrate the backend end-to-end.** Thread the exec payload (ctl +
    imm + pc) through the scheduler (opaque pass-through field in the IQ entry, or
    a payload RAM indexed by the IQ slot); build `exec_bundle` (4 `exec_shard` +
    the wb broadcast net, wb → scheduler `wake` + every RF copy); wire
    `frontend → sched_bundle → exec_bundle → wake` and demonstrate end-to-end ALU
    execution of a real program. Then commit / CPR + convert the rename freelist
    to the bitmap + A[C]/P[C] reclamation. Deferred: branch prediction / FTQ (TT
    has a BP to make basic-block — once the pipeline runs); load wake delay-line +
    WB reservation (with the LSU); back-pressure to freeze the decode→rename
    boundary; multi-region floorplan (lifts the 272 MHz); wire the real I$ to
    `fetch`'s imem interface; cosim to close the operand-decode gap.

Open discussion threads (flagged by TT, not yet detailed): back-pressure across
stages; LSU store-to-load forwarding data locality + shared L1D read ports; the
multi-region floorplan for a real in-context bundle number.

Every stage gets a ready/valid (elastic) boundary. New RTL currently lives in
`probe/` (validate-first); migrates to `src/` at integration.
