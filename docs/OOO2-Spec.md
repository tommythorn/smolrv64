# OOO2 — microarchitecture specification

The core in `ooo2/`. **This file is normative and must be updated in the same commit as any
change it describes** (docs/rtl-rules.md H4). Every number here is read off the RTL; where a
figure is *measured* rather than structural it says so, with the workload.

Naming: `rv_*` modules are generic RISC-V blocks, `ooo2_*` are specific to this core.
Shared blocks (`fetch`, `aligner`, `rvc_expand`, `decode_*`, `alu`, `mul3`, `divider`,
`mmu`, `csr_file`, `fp_unit`) live in `src/` and are used by both this core and the older
sharded-OoO core rooted at `src/soc_top.v`.

---

## 1. What it is

| | |
|---|---|
| ISA | RV64IMAFDC (`misa` = A, C, D, F, I, M, S, U; MXL=2) |
| Privilege | M / S / U |
| Translation | Sv39 (`satp.MODE`=8) or Bare; Ssvnapot level-0 NAPOT leaves |
| Also implemented | Zicsr, Zifencei, Zicntr, Zihpm (13 counters), Sstc, Smstateen, Ssvnapot |
| Decoded but not in `misa` | Zba, Zbb, Zbs, Zicond (`src/decode_exec.v`) |
| Issue | **in-order, 1 instruction/cycle** |
| Completion | **out of order** (non-blocking loads and FP) |
| Commit | in order, from the ROB head, 1/cycle |
| Speculation | branch/jump prediction only; no memory speculation, no value speculation |
| Target | AMD XCKU5P, `probe_clk` **166.67 MHz** (6.000 ns) at `PROBE_CLK_DIV8=48` |

It is called OOO2 because it is on the path to full out-of-order and because `src/` already
holds a sharded-OoO core that keeps the plain `ooo` name. Issue is still in-order; the
"out-of-order" is in completion and writeback.

---

## 2. Pipeline

Three architected stages plus a commit point. `F` is itself pipelined internally.

```
  F  ── PC → iTLB → I$ → fetch buffer → aligner → RVC expand → decode ──┐
                                                                        │  F/X queue (8)
  X  ── rename → PRF read / M→X bypass → ALU, branch resolve ───────────┘
  M  ── LSU / mul / div / FPU / CSR ── trap, redirect, BTB train
  C  ── ROB head: architectural commit, minstret, free-list release
```

| stage | holds | can stall | can stall others | can restart the pipe |
|---|---|---|---|---|
| F | PC, predictor read, fetch buffer, aligner | yes | no | no |
| F/X queue | up to 8 decoded instructions | — | back-pressures F when full | no |
| X | one decoded+renamed instruction | yes (`d_hold`) | holds F via `accept` | no |
| M | one instruction | yes (`m_done` low) | holds X via `m_advance` | **yes** (`redirect`) |
| C | ROB head | — | `head_block` holds M | no (the redirect fires from M) |

Only **M** redirects. Anything that redirects or traps must additionally be the **ROB head**
(`head_block`), because a younger instruction must not squash an older load or FP op still
in flight.

### 2.1 Issue is dynamic — for ALU ops

**X is no longer a single slot that everything funnels through.** Dispatch renames,
allocates a ROB slot and a scheduler entry, and moves on; it does **not** wait for operands.
The scheduler holds the instruction until its sources are ready and then issues the
**oldest ready** entry.

What may reorder is deliberately narrow. Every entry carries an `ord` bit, set for anything
that can trap, redirect, touch memory, or hold a unit for more than a cycle — memory, AMO,
mul, div, FP, CSR, `fence.i`, cbo, branches, jumps, and anything already known to fault.
Those keep program order. **Pure ALU ops are free to reorder**, and they complete at issue
without ever entering M.

That split is what makes the rest unnecessary rather than merely deferred:

| usually needed for OoO | why not here |
|---|---|
| deferred traps / redirect register | anything that can trap issues only as the oldest entry, so traps still fire in order |
| per-unit zombie bits at flush | nothing younger is ever in flight when a trap fires, so there is no writeback to suppress |
| memory disambiguation, store queue | memory keeps program order |
| deadlock avoidance | an op head-blocked in M cannot starve an older one: the only thing that can be older and un-issued is an ALU op, and those never enter M |

**Operands are read at ISSUE**, addressed by the selected entry's physical registers — doc 1's
"values live in one place". The old M→X bypass is gone; a writeback→issue forward (`fwd`)
replaces it, matching the same ports the scheduler wakes on, so readiness and data agree by
construction.

Wakeup is split in two. **Fast** wakeups are writebacks whose value can be forwarded in the
same cycle, and none of them may depend on which entry was selected or readiness becomes a
function of itself. **Slow** is the ALU op completing at issue: its result is written at the
end of the cycle, so a dependent cannot read it before the next one anyway, and waking it
late costs nothing.

Measured, 300 M-cycle Linux cosim, retires: **67,160,189 → 68,981,116** (dispatch decoupled)
**→ 69,754,834** (dynamic issue) — **+3.9%**. Small on this workload because a Linux boot is
frontend-bound (§14) and only ALU ops reorder; the buckets it targets are `ST_MEM` and
`ST_FPU`, which dominate GB5 rather than a boot.

### 2.2 Why the F/X queue exists

`accept` (X←F) is registered off M-stage state. The queue decouples fetch from the backend so
a fetch bubble and a backend stall do not serialise. Depth 8 (`QDEPTH`), which bought ~3.9% on
the Linux cosim over depth 4.

---

## 3. Stall taxonomy

Every non-retiring cycle is attributed to exactly one bucket by the hardware counters in
§11, and the buckets are additive because `st_m` (M stalled) and `m_advance` are exact
complements.

### 3.1 F stalls — the frontend has nothing to hand over

| cause | counter | note |
|---|---|---|
| I$ miss | `FE_IC` r0312 | line fill through the L2 arbiter |
| iTLB miss / page-table walk | `FE_MMU` r0311 | PTW runs as a line requester |
| had bytes, no complete instruction | `FE_ALN` r0313 | window is `OOO2_HW` halfwords |
| had an instruction, F/X queue empty | `FE_QUE` r0314 | backend drained the queue |
| X idle for any other reason | `FE_BUB` r0310 | catch-all frontend bubble |

### 3.2 X stalls — `d_hold`

`d_hold = d_valid & (src_pend | ~rob_ready | rn_stall)`

| cause | meaning |
|---|---|
| *(operands)* | **no longer a dispatch stall.** Waiting for operands happens in the scheduler now (§2.1); dispatch is blocked by structural resources only. |
| `~rob_ready` | ROB full (16 entries) |
| `~rs_ready` | the scheduler this op belongs to is full — integer 10, in-order 12 (§6.1) |
| `rn_stall` | any rename shard below `LOWAT`=4 free registers |
| `ser_block` | a serializing op is **alone in flight**: it does not dispatch until the ROB has drained, and nothing dispatches behind it until it commits |

### 3.3 M stalls — `m_done` low

`m_done = m_done_raw & ~head_block & ~ld_land & ~fp_land`

| cause | meaning |
|---|---|
| unit not complete | LSU / mul / div / FPU still working |
| `head_block` | the op is a trap, redirect, `fence.i`, CSR/SYSTEM, fault, or faulting memory op, and is **not yet the ROB head** |
| `ld_land` | a non-blocking load is landing this cycle and takes the single PRF write port + the single ROB completion port; M yields |
| `fp_land` | same, for an FP result |

Completion is **sticky** (`m_unit_done_q`): every unit's `done` is a one-cycle pulse, and
`ld_land`/`fp_land` can hold `m_done` low afterwards, so the pulse and its result/fault are
latched or they are lost.

### 3.4 Restarts

| source | condition | cost |
|---|---|---|
| branch/jump mispredict | `m_redirect`, must be ROB head | full frontend refill |
| trap / interrupt | `xtrap_v`, must be ROB head | full refill |
| CSR-induced redirect | `sret`/`mret`/`sfence`, serializing | full refill |
| `fence.i` | `ifence`, must be ROB head | full refill + I$ invalidate |
| interrupt injection | `irq_inject`, a solo SYSTEM pseudo-op that traps in M | full refill |

Measured redirect rate: **3.4 per 1000 instructions** (Linux cosim).

---

## 4. Frontend

### 4.1 Fetch

- Window: `OOO2_HW` halfwords. FPGA builds use `OOO2_HW=4` → **8-byte window**, so the I$
  read width is `HW*16` = 64 bits.
- **Ahead prediction**: the predictor arrays are addressed from `apc`, a register-only ahead
  PC, never from a combinational `npc`. This is what bought the predictor a full stage of
  slack at 166 MHz. A wrong guess degrades to a *lost* prediction, never a wrong one — the
  existing `btb_qpc == base_pc` tag check catches it. Cost: **0.13% of retires**.
- **Length predictor**: 1024-entry, 1 bit/entry, distributed RAM, indexed `pc_q[10:1]`.
  Predicts RVC-vs-32-bit so `apc` can advance without decoding.

### 4.2 Branch prediction

| structure | size | organisation | storage |
|---|---|---|---|
| BTB | 256 entries (`BTBB`=8) | 12-bit tag + 3-bit type + 38-bit target | BRAM, sync read |
| YAGS correector | 1024 entries (`YBITS`=10) | 8-bit tag + 2-bit counter | BRAM, sync read |
| GHR | 12 bits (`GHL`) | global history | flops |
| RAS | 8 entries (`RASB`=3) | call/return stack | flops |

**Neither array has a valid bit.** Validity *is* the tag match; a mismatched tag is a
miss. Removing them was bit-identical in simulation and moved both arrays from LUT/flop
structures into BRAM — a valid bit kept outside its array is a mux the size of the array
(rule I4). There are **no checkpoints**: this core forked `src/predictor.v` precisely to drop
them. Full OoO will need them back.

---

## 5. Rename and the physical register file

Architectural registers are a **unified 64-entry space**: 0–31 integer, 32–63 FP, matching
`decode_operands`' `{fp_bit, field}` encoding.

`ooo2_rename` carries a full speculative/committed split — SMAP/RMAP/lv for the map, and a
per-shard free list with a speculative head and a committed head. Rollback is `h := hc` in
**one cycle with no walk**, because rename never writes the free-list array.

### 5.1 Sharding

The PRF has **one write address and three shard-selected write enables**. Duplication buys
read ports only; sharding by *writer* is what buys write ports.

| shard | entries | written by | why the size |
|---|---|---|---|
| IE | 64 | **the ALU, alone** | > 32 (integer arch regs) |
| LD | 128 | everything M completes: LSU, mul, div, CSR, jump link | > 64: can hold integer *and* FP mappings |
| FE | 128 | FPU | > 64: the FPU writes integer regs too (`fcvt.w.d`, `fmv.x.d`, `fle.d`) |

**A destination's shard is chosen by the UNIT that writes it, never by the data type.** A
CSR read and a jump's link register are integer results, but M produces them, so they take
LD. Leaving those two in IE gave that shard a second writer, and the only way to keep one
write port was to hold the ALU off whenever M was writing IE — which put the entire LSU
completion cone inside the integer scheduler's ready bits. Post-route that was the critical
path: `m_addr -> lsu -> m_done -> m_wb_ie -> u_rs_i/e_r[9][1]`, 24 logic levels, WNS
-0.383 ns at 166.67 MHz. Applying the rule was worth **+0.397 ns** and is what closed
166.67 MHz with dynamic issue. With it, `m_wb_ie` is identically zero (asserted in
`ooo2_core`, not assumed) and the integer scheduler has no `unit_busy` term at all.

A physical register's shard is encoded in its number and never changes, so a commit's
`c_pold` is returned to **its own** shard's free list, not to `c_shard`.

`LOWAT`=4: fetch stalls when any shard drops below 4 free.

---

## 6. Reorder buffer

- **16 entries**, status only — no result values, no PC, no operands.

**Entry format — 16 bits.** `ent[]` is `{noret, rd, prd}`, plus `v` and `done` as separate
bulk-clearable bit vectors.

| field | bits | meaning |
|---|---|---|
| `noret` | 1 | commits but must not be counted (see below) |
| `rd` | 6 | architectural destination, unified numbering (0–31 int, 32–63 FP) |
| `prd` | `PBITS`=9 | physical register allocated; **0 when none** |
| | **16** | × `DEPTH`=16 = **256 bits** |
| `v[16]`, `done[16]` | 32 | separate flops — both are bulk-cleared on flush |

**Three fields are deliberately absent**, each recoverable from state the design already
keeps. The ROB is sized by the *window*; the scheduler that needs execute detail is sized by
*dependency depth*, so anything derivable does not belong here.

| not stored | recovered as | why it is sound |
|---|---|---|
| `rd_v` | `\|prd` | physical register 0 is architectural x0's permanent mapping and is never freed, so it is never allocated |
| `shard` | `prd[PBITS-1:IDXB]` | a physical register's shard is the top bits of its number and never changes |
| `pold` | `rmap[c_rd]`, read at commit | `rmap` holds committed state, so in the cycle an entry commits its architectural register still maps to what that entry displaced; the commit write is what replaces it |
- Completion is by **slot index**, allocated at rename and carried with the op.
- Write-forward on the head's `done` bit, so an op completing in the cycle its entry reaches
  the head commits that same cycle.
- Squash is **pointer-only**; there is nothing to walk.
- `noret` exists because an injected `OP_IRQ` can commit with its trap not firing, and
  `minstret` must not count an instruction that architecturally does not exist.

Simulation-only side arrays (`cs_pc`, `cs_insn`, `cs_val`, `cs_mkind`, `cs_mpa`) hold the
cosim payload per slot so the ROB stays status-only in hardware.

### 6.1 Scheduler (`ooo2_rs`) — LIVE

Stated here because its absence is the single largest fact about the machine (§2.1).
It selects what executes. See §2.1 for what may reorder and why the usual OoO machinery
is not needed alongside it.

**There are three schedulers, one per unit, and none stores age.**

| | `u_rs_i` | `u_rs_l` | `u_rs_f` |
|---|---|---|---|
| entries (`NENT`) | 10 | 12 | 4 |
| sources (`NSRC`) | 2 | 3 | 3 |
| holds | pure ALU and non-trapping ops | memory, AMO, mul/div, CSR, branches, jumps | **FP arithmetic** |
| ordering | **reorders freely** | **in order**, circular `qhead`/`qtail` | **reorders freely** |
| unit | completes at issue, writes IE | M | **stage F** |
| `unit_busy` | **none** — IE has one writer | `~m_advance \| (i_v & i_needs_m)` | `~f_advance \| (i_v & i_needs_f)` |

**One scheduler per unit is what makes three safe.** Three schedulers feeding ONE execute
stage deadlock (`Area-Efficient-Scalar-OoO.md` 12.2): an op reaches the shared stage, finds
it must be ROB head to retire, and an older op in a different scheduler cannot issue to
free it. Stage F removes the condition rather than arbitrating it -- **an FP arith op never
enters M**. It cannot head-block because it cannot trap: a bad encoding is `~d_fp_valid`,
and `mstatus.FS=Off` routes FP to M as before (a write to FS redirects and refetches, so
the value read at dispatch is what every in-flight FP op retires under).

`u_rs_f` reorders, and that is the point rather than a detail. `workloads/blurbench`, taken
from a hardware trace of GB5 Gaussian Blur, is a 4-deep serial `fadds` chain whose taps are
independent; in-order issue held it at exactly its critical path (39.50 cycles/pixel against
5 dependence levels x 8 cycles) because the next iteration's multiplies could not start
until this one's adds had issued. Reordering: **29.44 cycles/pixel, -25%**.

**Entry format — `2 + NSRC×PBITS` bits.** Scheduling state only; no age, nothing quadratic.

| field | bits | meaning |
|---|---|---|
| `v` | 1 | entry live |
| `e_rob` | `ROBB`=4 | ROB slot, carried to completion |
| `e_ps[NSRC]` | `NSRC` × `PBITS`=9 | source **physical** registers |
| `e_r[NSRC]` | `NSRC` | per-source ready bits |

`NSRC` is a parameter precisely so the integer scheduler does not pay for the third operand
only `fmadd` has: 20 bits/entry against 29.

- **Wakeup**: every PRF write broadcasts its destination; `NWB`=3 ports, one per shard.
  A source not yet ready is also compared against the live ports **in its dispatch cycle**,
  because a producer broadcasts exactly once and would otherwise be missed forever.
- **Select**: **fixed priority**, lowest entry index first. Age is not stored, not
  compared, and not needed — an age matrix is `O(N²)` and buys nothing a free list does not
  already give. Starvation is impossible because an entry is only freed by issuing.
- **Wake-at-select**, not wake-at-writeback (`FIXEDL`): a fixed-latency producer broadcasts
  its destination in the cycle it is *selected*, so a dependent issues the very next cycle.
  Back-to-back dependent issue is non-negotiable and is check 7 of the module TB.
- **`hold_v`/`hold_ent`**: the payload LUTRAM is read the cycle *after* selection, so an
  entry is not released at select — it is held until the issue register accepts it.
  Releasing at select let a dispatch overwrite `plmem[i_ent]` under a stalled issue stage;
  that survived two full 240-test runs before surfacing as `auipc`/`jalr` failures.
- **In-order mode** (`INORDER`) is a circular head pointer, not a comparator tree: entry
  `qhead` is the only one eligible. A younger ready entry does not pass an unready head.
- **No execute payload and no operand values.** The payload is a separate LUTRAM indexed
  by the scheduler's own entry number (`d_ent` to write at dispatch, `iss_ent` to read at
  issue) — **396 bits × 8 = 3 168 bits**, 34 fields. Not indexed by `rob_idx`: that would be
  `ROB_SIZE` deep where `NENT` suffices, and would put a read port at issue on a ROB-sized
  array, the specific thing the ROB/scheduler split exists to avoid. Operand values are
  never stored; the PRF is read **at issue**.
- The payload holds the X→M bundle **minus** operand values and minus everything
  `ooo2_exec` recomputes (`result`, `addr`, `target`, `taken`, `redirect`). `prd` is stored
  **gated by `rd_v`**, matching the ROB's "0 means writes nothing" convention — which is
  where it differs from the ungated `m_prd`.
- Packed and unpacked with the **same concatenation**, so a width or ordering error is a
  lint failure. And checked at runtime: dispatch happens in the cycle an instruction enters
  M and the scheduler cannot offer it before the next cycle, so at issue the payload holds
  exactly what M holds — `pc`, `insn`, `imm`, `rd`, `prd` are compared every cycle.

Gates: `ooo2/run-ooo2-rs-tb.sh` (seconds, 13 checks) plus the payload check above, which
runs inside every 240-test and cosim run.

---

## 7. Execution units

| unit | latency | outstanding | blocks M? | writes |
|---|---|---|---|---|
| ALU / branch | 1 cycle (at issue) | — | no | IE shard |
| jump link (`jal`/`jalr`) | 1 cycle | 1 | yes | LD shard (M writes it) |
| CSR | 1 cycle, **serializing** | 1 | yes | LD shard (M writes it) |
| mul (`mul3`) | 3 cycles, pipelined | 1 | yes | LD shard |
| div (`divider`) | ~64 cycles, FSM | 1 | yes | LD shard |
| FPU (CVFPU) | 6 cycles (§7.1) | **4** | **never enters M** (stage F) | FE shard |
| LSU load | see §8 | 1 | **no** | LD shard |
| LSU store / AMO | see §8 | 1 | yes | — |

Only loads and FP are non-blocking. mul, div, stores and AMOs still hold M — for mul/div
because they are rare enough not to have paid for the work yet, for stores because a store
has no destination register so a scoreboard slot buys it nothing.

### 7.1 FPU

`fp_unit` drives `fpnew_top` **directly**. It does *not* go through `smolrv64_cvfpu`, which
is a two-clock-domain wrapper — both instantiations of `fp_unit` tie `fpu_clock` to `clk`, so
its toggle handshake and two ASYNC_REG synchronisers cost ~9 cycles for a crossing that does
not exist. (`smolrv64_cvfpu` is still used by `rk_xcku5p.v` on a real `fpu_clk`.)

- `PIPE_REGS`=4, `DISTRIBUTED`, ADDMUL/DIVSQRT/CONV `MERGED`, NONCOMP `PARALLEL`,
  `DivSqrtSel = THMULTI`.
- **NONCOMP (min/max, sign-inject, compare, classify) and parts of CONV return
  combinationally** — `out_valid` in the same cycle as `in_valid`, `PipeRegs` notwithstanding.
- One op in flight. `iss_ready` is **registers-only**: `fpnew`'s `in_ready_o` is combinational
  in `in_valid_i`, and `exec_shard.v` feeds `iss_ready` back into its issue decision, so
  exposing it closes a loop through the scheduler.
- Occupancy in M: **6 cycles**. Measured on hardware (`chain` kernel, a pure dependent FMA
  chain with nothing to overlap, so it isolates unit latency): **14.018 → 7.004 → 6.004**
  stall cycles per FP op, for the CDC removal and then the request-register bypass. The
  request register is a fallback for the cycle `fpnew` declines, not a stage every op pays.

### 7.2 FP scoreboard

M completes an FP op when the unit **accepts** it. The result is captured into a holding
register (`fb_val`) and written to the PRF when the single write port is free — a landing
load wins, FP waits. Safe without a ROB walk because an FP op cannot trap (it reports via
`fflags`) and anything older that could trap is head-gated.

`fflags` from an FPU completion and from an in-core compare are **OR**ed, not priority-muxed:
once FP stopped blocking M the two can coincide, and a mux silently dropped the compare's NV.

---

## 8. Load/store unit and MMU

- **Blocking store/AMO, non-blocking load.** One memory op in flight.
- A load's fault is decided **before the access starts**: `mis_flt` and `xl_flt` are both
  qualified by `xl_req = req_valid & (st == S_IDLE)`. Once the LSU leaves `S_IDLE` the access
  cannot fault. This is what makes precise exceptions possible **with no ROB walk** — M holds
  only until translation resolves (same cycle on a TLB hit), and after that the load is
  architecturally guaranteed to complete.
- **No line-spanning access**: the LSU splits a line-crossing access, and the cache resolves
  the two halves internally via a two-phase lookup.
- Load-format controls (`nb`, signed, fp) are **latched at dispatch**, not read at
  completion — the stage they came from has moved on by then.

### 8.1 Address translation

Two independent `mmu` instances — **iTLB** in `ooo2_core`, **dTLB** in `ooo2_lsu`.

| | |
|---|---|
| TLB | 16 entries, **direct-mapped**, per instance |
| Scheme | Sv39, 3-level hardware page-table walk |
| PTW | runs as a line requester through the L2 arbiter |
| Superpages | 2 MiB / 1 GiB; misaligned superpage → fault |
| Ssvnapot | level-0 NAPOT leaves recognised |
| PA width | 56 bits produced (`AW`), 34 significant to the caches |

The MMU also range-checks the resolved PA: anything outside {RAM, CLINT, PLIC, UART, LSRAM,
virtio} faults rather than being silently dropped.

---

## 9. Memory system

### 9.1 L1 caches

Both are the **same module** (`rv_cache`), specialised by parameter.

| | I$ | D$ |
|---|---|---|
| Size | 64 KB | 64 KB |
| Associativity | **2-way skew-associative** | 2-way skew-associative |
| Sets | 512 | 512 |
| Line | 64 B (512 bit) | 64 B |
| Indexing | **PIPT** | **PIPT** |
| Read width | `OOO2_HW*16` = 64 bit | 64 bit |
| Write policy | fill-only (`WRITABLE=0`) | **write-back** (`WRTHRU=0`) |
| Prefetch | next-line, single-line stream buffer | none |
| Storage | BRAM (`smolrv64_sdpram`, 1R1W, `READ_LATENCY=1`) | same |

**Not UltraRAM.** Data is even/odd **banks** of `BANKW` bits per way — `2*WAYS` sync-read
BRAMs. Because `BANKW` equals the read width, any read at any byte offset spans at most two
consecutive chunks, which have opposite parity and therefore live in different banks: one
read serves it, with a byte shift instead of a full-line mux. A byte-masked store is a
read-modify-write of one chunk, no cross-bank RMW.

Skew: way 1 XORs low tag bits into the index; a victim's base index is recovered as
`skewed_index ^ victim_tag`.

Measured D$: **3.24 cycles per access** at a **0.195% miss rate** — i.e. the LSU cost is hit
latency, not misses.

### 9.2 Below L1

`rv_l2_arbiter`: fixed-priority merge of **4** line requesters (I$ fill, D$ fill, D$
writeback/write-through, PTW-as-line) onto one 512-bit line port to DDR4.

### 9.3 Physical memory map

| region | base | size |
|---|---|---|
| CLINT | `0x0200_0000` | 64 KiB |
| PLIC | `0x0c00_0000` | 64 MiB |
| UART (NS16550) | `0x1000_0000` | 8 byte registers |
| virtio-mmio (blk + net) | `0x1000_2000` | 8 KiB |
| on-chip SRAM (boot/monitor) | `0x7000_0000` | 256 KiB |
| DDR4 | `0x8000_0000` | platform |

UART is **3 Mbps, hardwired in RTL** — not derived from the DTS or the kernel printout.

---

## 10. Array inventory

Every array in the core, with the shape the RTL actually declares. Sizes are for the
shipping configuration (`SIZE_KB`=64, `OOO2_HW`=4, `PAW`=64 into the caches).

### 10.1 Core

| array | module | shape | width | bits | storage | ports |
|---|---|---|---|---|---|---|
| `mem_ie` | `ooo2_prf` | 64 | 64 | 4 096 | LUTRAM | 3R shared, 1W |
| `mem_ld` | `ooo2_prf` | 128 | 64 | 8 192 | LUTRAM | 3R shared, 1W |
| `mem_fe` | `ooo2_prf` | 128 | 64 | 8 192 | LUTRAM | 3R shared, 1W |
| `smap` | `ooo2_rename` | 64 | 9 | 576 | LUTRAM | 3R, 1W + bulk |
| `rmap` | `ooo2_rename` | 64 | 9 | 576 | LUTRAM | 4R, 1W |
| `lv` | `ooo2_rename` | 64 | 1 | 64 | flops | bulk-cleared on flush |
| `fl_ie` | `ooo2_rename` | 64 | 7 | 448 | LUTRAM | free list |
| `fl_ld` | `ooo2_rename` | 128 | 7 | 896 | LUTRAM | free list |
| `fl_fe` | `ooo2_rename` | 128 | 7 | 896 | LUTRAM | free list |
| `ent` | `ooo2_rob` | 16 | 16 | 256 | LUTRAM | 1W dispatch, 1R commit |
| `v`, `done` | `ooo2_rob` | 16 | 1 each | 32 | flops | bulk-clearable |
| `u_rs_i` entry | `ooo2_rs` | 10 | 2+2×9 = 20 | 200 | flops | integer, `NSRC`=2 (§6.1) |
| `u_rs_l` entry | `ooo2_rs` | 12 | 2+3×9 = 29 | 348 | flops | in-order, `NSRC`=3 (§6.1) |
| `u_rs_f` entry | `ooo2_rs` | 4 | 2+3×9 = 29 | 116 | flops | FP, reorders, `NSRC`=3 (§6.1) |
| `plmem` (payload) | `ooo2_core` | 26 | 413 | 10 738 | LUTRAM | 1W dispatch, 1R issue |
| `pend` | `ooo2_pending` | 512 | 1 | 512 | flops | 3R, 1 set + 3 clear, bulk-clear |
| `q_dat` | `ooo2_frontend` | 8 | 281 | 2 248 | LUTRAM | F/X queue |

Each PRF shard now has **its own write address** (`wa_ie`/`wa_ld`/`wa_fe`), which is §7 of
`Area-Efficient-Scalar-OoO.md`'s "give each file its own port and a single writer and the
arbiter disappears". Half of that holds: IE takes only the ALU/CSR result and FE only the
FPU, but **LD has two writers** — a landing load and M's mul/div — so LD still needs an
arbiter, or mul/div needs its own shard. It costs nothing today because `m_done` is forced
low on `ld_land`/`fp_land`, and an assertion fires the moment that stops being true.

The remaining single point is the **ROB completion port**: widened to `NW` ports but pinned
at `NW=1`, and it is now the only reason a cycle has to be yielded at all.

### 10.2 Front end

| array | module | shape | width | bits | storage |
|---|---|---|---|---|---|
| `btb` | `ooo2_predictor` | 256 | 53 | 13 568 | **BRAM**, sync read |
| `ycorr` | `ooo2_predictor` | 1024 | 10 | 10 240 | **BRAM**, sync read |
| `ras` | `ooo2_predictor` | 8 | 64 | 512 | flops |
| `lenp` | `src/fetch` | 1024 | 1 | 1 024 | LUTRAM |

Neither `btb` nor `ycorr` has a valid bit — validity is the tag match (§4.2).

### 10.3 Memory system

Per cache instance; there are two (I$, D$), identical geometry.

| array | shape | width | bits | storage |
|---|---|---|---|---|
| data banks | 4 × 2048 | 64 | 524 288 | **BRAM** (`smolrv64_sdpram`, 1R1W, `READ_LATENCY=1`) |
| `tagm` | 1024 | 49 | 50 176 | LUTRAM |
| `valm` | 1024 | 1 | 1 024 | flops |
| `dirm` | 1024 | 1 | 1 024 | flops (D$ only in effect) |
| `vicm` | 512 | 1 | 512 | flops — victim/replacement bit per set |

Data is 4 banks (`2*WAYS`) of `2048 × 64`, which is the 64 KB: even/odd chunk banking per
way, so any read at any byte offset is served by one access (§9.1).

**`PTAGB` = 49 bits and does not need to be.** It is `PAW - IDXB - OFFB` with `PAW`=64 as
instantiated, but the MMU produces 56-bit physical addresses, so eight tag bits per way are
structurally zero — ~8 Kib per cache, ~16 Kib total. Untested observation, not a measurement;
narrowing `PAW` to 56 is a one-parameter change that needs a build to confirm it is free.

Address translation, two instances (iTLB in `ooo2_core`, dTLB in `ooo2_lsu`):

| array | shape | width | bits |
|---|---|---|---|
| `tlb_v`/`tag`/`ppn`/`lvl`/`perm`/`nc`/`n` | 16 | 1+27+44+2+8+1+1 = 84 | 1 344 |

Also in `rv_soc_top`: `lmem` — the 256 KiB on-chip boot/monitor SRAM, `NLLINE × 512`.

### 10.4 Verification-only

Not present in synthesis (`ifndef SYNTHESIS`), listed so nobody counts them as area:

| array | shape | width | purpose |
|---|---|---|---|
| `cs_pc` | 16 | 64 | retire PC for the cosim trace |
| `cs_insn` | 16 | 32 | retire instruction word |
| `cs_val` | 16 | 64 | retire value — captured at the writeback event |
| `cs_mkind` | 16 | 2 | memory-effect kind |
| `cs_mpa` | 16 | 56 | memory-effect physical address |
| `r` (`rv_regfile`) | 64 | 64 | architectural shadow, written at commit |

---

## 11. Counters and observability

`Zihpm` with **13** programmable counters (`mhpmcounter3..15`), driven by a 22-bit `hpm_ev`
bus. This is exactly the width of the event list, so a 13-event `perf stat` is full — adding
an event to a run means dropping one.

| event | id | meaning |
|---|---|---|
| `ST_MEM` | r0300 | M stalled on the LSU **+** X held for a pending load result |
| `ST_DIV` / `ST_MUL` | r0301 / r0302 | M stalled on divider / multiplier |
| `ST_FPU` | r0303 | M stalled on the FPU **+** X held for a pending FP result |
| `ST_SER` | r0304 | frontend held by a serializing op (excludes the dependent waits above) |
| `FE_BUB` | r0310 | frontend bubble |
| `FE_MMU` / `FE_IC` | r0311 / r0312 | iMMU walking / I$ no window |
| `FE_ALN` / `FE_QUE` | r0313 / r0314 | no whole instruction / queue empty |
| D$ / I$ access, miss | r0100/r0102, r0110/r0112 | |
| redirects | r0005 | |

`ST_MEM` and `ST_FPU` deliberately include the *dependent* wait, charged to the unit that
owns the register being waited on: when a unit stopped blocking M, the wait did not go away,
it moved to X, and charging it to `ST_SER` made a data dependency read as a serializing op.
A consumer waiting on both a load and an FP result is charged to `ST_MEM`.

---

## 12. Verification

| gate | command | pass |
|---|---|---|
| lint | `src/lint.sh` | `lint: clean` |
| riscv-tests, this core | `ooo2/run-ooo2-vl.sh` | `pass=240 fail=0` |
| riscv-tests, `src/` core | `src/run-vl-tests.sh` | `failures: 0` (shares `fp_unit`) |
| Linux lockstep vs simmerv | `BUILD=1 CYC=300000000 ooo2/run-ooo2-cosim-linux.sh` | no assertion, no divergence |

**`CYC=300000000` is the required cosim length.** At 40e6 the run reports `inj=0` — it never
reaches the first interrupt — and three defects that wedged hardware were invisible at that
budget. `BUILD=1` is required whenever RTL changed; the runner does not rebuild otherwise.

Invariant assertions are **always on** (`$fatal`, never `` `ifdef ``). Only flood-volume
tracers and stats are gated.

---

## 13. FPGA implementation (XCKU5P, `platforms/rk-xcku5p-f-v1.2`)

| | |
|---|---|
| `probe_clk` | 166.67 MHz (6.000 ns), `PROBE_CLK_DIV8=48`, from an MMCM |
| Timing | `probe_clk` WNS **+0.050 ns**, TNS 0.000, 0 failing endpoints |
| Margin | 50 ps against a placement spread of 81-400 ps (rule I2): closed, **not robustly**. History, because each step was paid for: +0.029 at 10/12 dynamic issue; +0.124 at 8/8 (reverted -- it cost 4.1% geomean on GB5); +0.069 with FP four-in-flight, which *gained* 40 ps by deleting `fpu_inflight`/`fb_busy` from M; +0.028 with the FP scheduler and stage F, which cost 41 ps; +0.050 at `NF`=4, which gave 22 ps back for no measured performance. |
| Measured clock | 164.2 MHz by on-chip counter |
| Core voltage | 0.853 V measured against a 0.85 V design point |
| 333 MHz | measured **−2.492 ns**, 39,389 failing endpoints. Operating-condition levers are worth exactly zero. |
| Build | `OOO2_CORE=1 OOO2_HW=4 PROBE_CLK_DIV8=48 make bit` |

Read `probe_clk` from the **Intra Clock Table**, not global WNS — global WNS is usually
pinned by the MIG's `ui_clk`.

---

## 14. Known limits

- **Single-outstanding everything.** One load, one FP op, one mul/div. Independent FP cannot
  overlap itself; that needs a second PRF write port.
- **Only ALU ops reorder** (§2.1). A memory, mul, div, FP or CSR op still issues in program
  order, so a long-latency op still blocks *other long-latency ops* behind it. Freeing those
  needs what §2.1's table says this design currently avoids: deferred traps, zombie units,
  and memory disambiguation.
- **M is still a single execute slot** for everything except ALU ops, so only one
  long-latency op is in flight at a time.
- **`IW=1`.** The aligner emits at most one instruction per cycle, exactly what the backend
  consumes, so the F/X queue can never build a backlog and every frontend hiccup is
  unrecoverable. On integer workloads this is the dominant cost (frontend 41.8% of cycles on
  `sha256sum`); on GB5 it is 2.2%.
- **Stores and AMOs still block M.**
- **No memory disambiguation** — a younger memory op simply waits for `S_IDLE`.
- Workload sensitivity is large and measured: GB5 is FPU- and serialisation-bound,
  `sha256sum` is frontend-bound. Do not generalise a CPI stack from one workload.

## 15. Prioritised work list

Ordered by measured impact over effort. Every claim names its measurement and the
CONFIGURATION it was measured in; anything unmeasured says so.

### Measurement discipline (read before adding a number here)

**Always measure at `VDEFS="-DOOO2_HW=4"`.** The simulation default is `OOO2_HW=2`
(32-bit fetch); the FPGA runs 4 (64-bit). They do not merely differ in degree, they
disagree about which unit is the bottleneck:

| workloads/aesbench | HW=2 (sim default) | **HW=4 (the FPGA)** |
|---|---:|---:|
| cycles/byte | 222.90 | **122.70** |
| IPC | 0.277 | **0.506** |
| `FE_ALN` (RVC aligner) | **37.0%** | 8.0% |
| `FE_QUE` (F/X empty) | 24.0% | 24.0% |
| `ST_MEM` | 7.9% | **23.7%** |

At HW=2 the RVC aligner looks like the largest stall in the machine and the scheduler
sweep saturates; at HW=4 the aligner is a minor term and the sweep has a real optimum at
8. An earlier version of this list ranked the aligner P0 on the HW=2 numbers. It was
wrong, and it was believable because `run-ooo2-cosim-linux.sh` reported the width by
grepping for `-DINO_HW=`, a name that died in the rename -- so it printed the default no
matter what was set (fixed, `a0d651e6`+).

Two workloads, two different answers, both valid:

| | GB5 full suite (hardware) | aesbench (AES kernel, HW=4) |
|---|---:|---:|
| `ST_MEM` | **54.7%** | 23.7% |
| `ST_FPU` | **28.9%** | ~0 |
| `FE_BUB` | 5.3% | **32.4%** |
| `ST_SER` | 3.9% | ~0 |

The frontend is an integer/crypto-code problem, not a machine-wide one. Memory is
machine-wide. Rank by the full suite unless the goal is a specific workload.

### Answered: why is the ROB only 16 entries, and would more help?

**No, and it is measured twice** -- once with FP in the in-order scheduler and again after
FP got its own reordering scheduler, in case the first answer was an artifact of that:

| ROB | blur cyc/px | AES cyc/byte |
|---|---:|---:|
| 16 | 29.44 | 122.7 |
| 32 | 29.32 | 121.6 |
| 64 | 29.32 | 121.6 |

It saturates at 32 and the whole range spans 0.4% on blur, 0.9% on AES. Before the FP
scheduler the blur column was 39.50 at **all three** depths, bit-identical.

The reason is that the ROB has never been the binding constraint -- the UNITS are, which
the scheduler sweeps say from the other side too (4 to 20 entries spans 0.8%). Blur now
sits at 29.44 cycles/pixel with `ST_FPU` at 65%, i.e. 19.2 stall cycles for 8 FP ops =
2.4 cycles per op, against the 2.25 cyc/op `fpbench` measures as FP throughput. It is at
the FPU's throughput bound, and no amount of window changes that.

What a deeper ROB WOULD cost is real: `N_IE` must grow with it (32 of its 64 physical
registers are the architectural integer set, so only 32 are free, and the free list runs
dry via `LOWAT` before a 32-entry ROB fills), and both are `$clog2`-wide in the FP tag,
the payload index and every scheduler entry. Revisit only when a measurement shows the
window binding.

### P0 -- multiple outstanding loads

`ST_MEM` is **54.7% of full-suite GB5 cycles** at a 2.04% miss rate costing 4.995 cycles
per access, and 23.7% even on the cache-resident AES kernel. The scheduler sweep proves the
window is not the constraint (4 entries to 20 spans 0.8%); `workloads/ldbench` proves what
is, directly:

| `ldbench`, entirely L1-resident (0 misses) | cyc/load | `ST_MEM`/load |
|---|---:|---:|
| latency (pointer chase) | 5.00 | 4.99 |
| throughput (8 INDEPENDENT streams) | **4.00** | 1.75 |
| **overlap (lat/thru)** | **1.24x** | |

Eight independent streams overlap 1.24x. Loads are very nearly serialised, `ST_MEM` is 43%
of cycles with zero misses, and the ceiling is ~4 cycles per load however much ILP is
offered.

**This is NOT the same fix as P3a, and the difference is the whole cost estimate.** The FPU
turned out to be pipelined already (`PipeRegs=4`) with its tag ports wired and unused -- the
wrapper was the only thing serialising it, so the fix was a counter and a tag. Here both
stages are genuinely FSMs:

    rv_cache: S_IDLE (accept) -> S_CHECK -> deliver -> S_IDLE
    ooo2_lsu: S_IDLE -> S_LD  -> (S_LD2 if xword) -> S_IDLE

Neither accepts a new request until it returns to idle, and ~2 cycles each is exactly the
measured 4. Pipelining the hit path -- address/BRAM-read in one stage, compare/select in
the next, with the miss path left on the FSM -- is a real rewrite of the most
corruption-sensitive module in the design, not a wrapper change. The tag plumbing
(`rd_tag`/`rd_resp_tag`, and 2 free bits in the LSU tag space) already exists for it.

Do not start this in the same batch as anything else: it wants its own bisect.

### P1 -- triage the regression against 95aff227 (8/22)

Reported: many workloads regressed between 95aff227 (in-order issue) and 72d14cde
(dynamic issue), while AES-XTS improved 814.5 -> 950.6 kB/s. Mechanism already measured:
dynamic issue grew the window from ~2 instructions to 16 and both penalties scale with
depth --

    FE_BUB per redirect   9.6 -> 54.4 cycles
    ST_SER               0.16% -> 8.67% of cycles

A win on load-bound code and a loss on branch-heavy or serialising code is exactly what
that predicts, and it is the same trade the scheduler sweep shows in miniature (8/8 beats
10/12). Needs the 8/22 page in `bench/gb5-results.tsv` to confirm which. Noise floor:
95aff227 measured 814.5 and 754.4 kB/s on identical RTL, 7.4% apart.

### P2 -- delete `head_block` (redirect register + issue-time head gate)

Completion must not depend on retirement (`Area-Efficient-Scalar-OoO.md` 12.2). Blocks P3
outright and cuts `ST_SER`. Frontend half **done** (`575781db`, +0.82% on the boot). Five
of six `m_needs_head` terms are decode-static and can gate at issue on
`entry_rob == rob_head`; only a mispredict and a faulting memory access need the register.

### P3a -- FP multiple in flight -- **DONE**

`fp_unit` held exactly one op while `fpnew` underneath is pipelined (`PipeRegs=4`) and
already had tag ports, instantiated `TAGW(1)` with `iss_tag(1'b0)` and `res_tag()`
unconnected. The destination now rides in a 21-bit tag (`dst32, rd_v, rd, rob, prd`), so
results self-describe and may return out of issue order -- which they do, because fpnew's
op groups (ADDMUL, DIVSQRT, NONCOMP, CONV) have different latencies. `NFLIGHT=4`.

Measured on `workloads/fpbench`, eight independent chains:

| | before | after |
|---|---:|---:|
| throughput | 4.00 cyc/op | **2.25 cyc/op** |
| `ST_FPU` | 43% of cycles | **0%** |
| overlap (lat/thru) | 1.99x | **3.55x** |
| latency (serial chain) | 8.00 cyc/op | 8.00 cyc/op |

Latency is unchanged and should be: a dependent chain waits on the FPU no matter how many
slots are free. `FE_BUB` is now 44% on that kernel -- the frontend is what is left.

`NFLIGHT` defaults to **1** so `src/exec_shard.v`, which shares this file, is bit-identical
(215/215 confirms). FP also took its own ROB completion port (`NW` 2 -> 3); it used to share
one with landing loads, which is why a result had to be held when a load landed in the same
cycle. With several in flight that collision stops being rare.

**This was the same defect as the LSU**: a pipelined unit throttled to one outstanding
operation by its wrapper. P0 is the same shape of fix.

### P3b -- FP gets its own scheduler AND its own unit -- **DONE**

`u_rs_f` (8 entries, `NSRC`=3, reordering) feeding stage F, a one-entry execute stage
parallel to M. Measured: `blurbench` 39.50 -> **29.44** cycles/pixel (-25%). `fpbench` is
unchanged at 2.25 cyc/op throughput and 8.00 latency, correctly -- it was already FPU-bound
rather than issue-bound, so the gain lands exactly where the trace predicted and nowhere
else.

One hazard this exposed, recorded because it will return: an FP op still in fpnew's
pipeline when a squash happens writes back after rename has rolled its physreg away.
`ooo2_pending` caught it as *"writeback to p352, which was not pending"*. `fp_unit` already
forwarded `flush` to `fpnew_top`; the core tied it to 0. Flushing EVERY in-flight op on
redirect is correct only while `head_block` holds -- a redirect fires only at ROB head, so
anything in flight is younger by construction. **Removing `head_block` (P2) requires
replacing this with an epoch tag first.**

### P3b-old -- superseded, kept for the reasoning

`ST_FPU` is **28.9% of full-suite cycles** at 1.148 CPI, a regression from the 0.973 the
FPU rework reached, caused by FP arithmetic sitting in the in-order scheduler behind every
load, mul/div, CSR and branch. Blocked on P2: three schedulers into one execute stage
deadlock.

### P4 -- Machine Learning scores 0

GB5 aggregates a category as a **geometric mean**, so this single 0 (0.01 images/sec)
zeroes the entire Floating Point score. Cause uninvestigated -- the cheapest possible score
win if it is one pathology.

### P5 -- frontend run-ahead (`IW>=2`)

`FE_QUE` is **24% of cycles on the AES kernel** and unchanged between HW=2 and HW=4, so it
is not an alignment artifact: the frontend makes at most one instruction per cycle and the
backend consumes one per cycle, so no hiccup is ever recovered. Only 5.3% on the full
suite, hence below the memory and FP items despite being large on integer code.
`sha256sum` at 41.8% frontend is the same effect.

### P6 -- shrink the schedulers to 8/8 -- **REVERTED, it was a regression**

`aesbench` at `OOO2_HW=4` showed an optimum at 8/8 (122.18 cycles/byte against 10/12's
122.70) and it bought **+95 ps** of `probe_clk` margin. The full GB5 suite then measured
8/8 at **-4.1% geomean** against 10/12, including on the workload `aesbench` exists to
model:

| | 10/12 | 8/8 | |
|---|---:|---:|---:|
| AES-XTS | 950.6 | 851.1 KB/sec | **-10.5%**, and Crypto score 1 -> **0** |
| Ray Tracing | 3.53 | 2.94 | -16.7% |
| PDF Rendering | 237.1 | 201.9 | -14.8% |
| SQLite | 1.19 | 1.05 | -11.8% |

**A microbenchmark can validate a change the real workload rejects.** `aesbench` is
L1-resident and single-phase; the real AES-XTS runs under virtual memory with real misses
and a mixed instruction stream, where a smaller window costs more than it saves. Treat a
microbenchmark win as PROVISIONAL until a suite run confirms it, and size the schedulers on
the suite. The 95 ps must be found elsewhere.

### Measured: dynamic issue vs in-order issue, full GB5 single-core

From saved result pages, per workload, rate not score.

| | geomean |
|---|---:|
| dynamic issue vs in-order (`95aff227` -> `72d14cde`) | **+24.5%** |
| 8/8 vs 10/12 (`72d14cde` -> `2ba7716e`) | -4.1% |
| net, in-order -> today | +19.1% |

12 workloads improved, 2 regressed, 7 within the 8% noise floor. Biggest gains are FP and
media -- Gaussian Blur +70%, Image Inpainting +61%, Rigid Body +52%, Structure from Motion
+52%, Ray Tracing +42%. **The only two real losses are branch-heavy integer code**:
Navigation -12.2% and HTML5 -8.5%, which is the window-depth mispredict penalty
(`FE_BUB` 9.6 -> 54.4 cycles per redirect) showing up exactly where predicted. Dynamic
issue is not a broad regression; it is a large win with a narrow, understood cost.

### P7 -- rename walk-back, to remove the mispredict DRAIN

The other half of the mispredict cost. Needs a FIFO of ~ROB-size (index, value) pairs
replayed on squash, and introduces a race between the replay and the ROB head advancing.
Deferred deliberately: the first item here that makes an otherwise simple design
complicated, and it must not cost frequency.

### P8 -- stores and AMOs still block M

Unmeasured in isolation. Listed so it is not forgotten, not because it is next.
