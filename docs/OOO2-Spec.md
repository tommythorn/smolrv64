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

### 2.1 There is no issue queue

Worth stating plainly, because a ROB plus a physical register file plus a scoreboard usually
implies one. **X is a single instruction slot.** There are no reservation stations, no
wakeup/select, no scheduler, and nothing may issue around anything else. The scoreboard
(§7.2, §8) is an *interlock*, not a dispatcher: it decides whether the instruction in X may
advance, and its only two answers are "go" and "stall".

So an `add` waiting on a pending load **blocks every younger instruction**, including ones
that do not touch the load's destination at all:

```verilog
d_hold = d_valid & (src_pend | ~rob_ready | rn_stall);   // ooo2_core.v
accept = m_advance & ~d_hold & ~ser_block;
q_pop  = accept & ~q_empty;                              // ooo2_frontend.v
```

`accept` low means the F/X queue does not pop. Younger instructions accumulate behind the
stalled one in that 8-entry FIFO — strictly in order, no bypass — and once it fills, fetch
stalls too. This is **head-of-line blocking** and it is the single largest structural gap
between this core and an out-of-order one.

The direct consequence: a non-blocking unit buys only the instructions **between the
producer and its first consumer**. Nothing after the consumer benefits. Measured, on the
three FP kernels of §7.1 — `dot` gains 2.19x because the next iteration's two loads and its
address arithmetic fit under the FMA, while `saxpy`, whose FP work is independent across
iterations but whose consumer follows immediately, gains only 1.24x.

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
| `src_pend` | a source's **physical** register is the destination of an in-flight load (`ld_pend`) or FP op (`fp_pend`). Compared on renamed physregs, not architectural ones. **Blocks all younger instructions too** — see §2.1. |
| `~rob_ready` | ROB full (16 entries) |
| `rn_stall` | any rename shard below `LOWAT`=4 free registers |
| `ser_block` | a serializing op (CSR, per `decode_exec.v`) is in X or M — separate term, ANDed into `accept` |

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
| IE | 64 | ALU / CSR | > 32 (integer arch regs) |
| LD | 128 | LSU, mul, div | > 64: can hold integer *and* FP mappings |
| FE | 128 | FPU | > 64: the FPU writes integer regs too (`fcvt.w.d`, `fmv.x.d`, `fle.d`) |

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

### 6.1 Scheduler (`ooo2_rs`) — BUILT AND UNIT-TESTED, NOT YET WIRED

Stated here because its absence is the single largest fact about the machine (§2.1) and
because the module now exists. **It is not instantiated by `ooo2_core` yet**, so nothing in
§2, §3 or §7 describes its behaviour; this subsection describes what will replace them.

**Entry format — 38 bits.** Scheduling state only.

| field | bits | meaning |
|---|---|---|
| `v` | 1 | entry live |
| `e_rob` | `ROBB`=4 | ROB slot, for age and for completion |
| `e_ps1/2/3` | 3 × `PBITS`=27 | source **physical** registers |
| `e_r1/2/3` | 3 | per-source ready bits |
| `e_unit` | `NUNIT`=4 | one-hot unit requirement |
| | **38** | × `NENT`=8 = **304 bits** |

- **Wakeup**: every PRF write broadcasts its destination; `NWB`=3 ports, one per shard.
  A source not yet ready is also compared against the live ports **in its dispatch cycle**,
  because a producer broadcasts exactly once and would otherwise be missed forever.
- **Select**: oldest ready, `age = (rob - head)`, minimum-reduction comparator tree.
  Deterministic and starvation-free.
- **The unit check is inside `ready`**, so a busy MEM cannot block an ALU entry.
- **No execute payload and no operand values.** Payload goes in a LUTRAM indexed by the
  scheduler's own entry number — written with the free-slot index at dispatch, read with the
  select index at issue. Not indexed by `rob_idx`: that would be `ROB_SIZE` deep where
  `NENT` suffices, and would put a read port at issue on a ROB-sized array, which is the
  specific thing the ROB/scheduler split exists to avoid. Operand values are never stored;
  the PRF is read **at issue**.

Gate: `ooo2/run-ooo2-rs-tb.sh` — seconds, no core build, 13 checks.

---

## 7. Execution units

| unit | latency | outstanding | blocks M? | writes |
|---|---|---|---|---|
| ALU / branch | 1 cycle (in X) | — | no | IE shard |
| CSR | 1 cycle, **serializing** | 1 | yes | IE shard |
| mul (`mul3`) | 3 cycles, pipelined | 1 | yes | LD shard |
| div (`divider`) | ~64 cycles, FSM | 1 | yes | LD shard |
| FPU (CVFPU) | 6 cycles (§7.1) | 1 | **no** | FE shard |
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
| scheduler entry | `ooo2_rs` | 8 | 38 | 304 | flops | **not yet wired** (§6.1) |
| `q_dat` | `ooo2_frontend` | 8 | 281 | 2 248 | LUTRAM | F/X queue |

The PRF's three shards share **one write address** with three shard enables, so it is one
write per cycle in total, not one per shard. §7 of `Area-Efficient-Scalar-OoO.md` offers
exactly this split as the way to delete writeback arbitration — but only if each file has
its own write port and a single writer. Neither holds yet: the address is shared, and the LD
shard has three writers (LSU, mul, div). This is the binding constraint on dynamic issue.

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
| Timing | WNS +0.027 ns, TNS 0.000, 0 failing endpoints |
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
- **No issue queue, so a stalled consumer stalls the whole machine** (§2.1). An instruction
  waiting on an in-flight load or FP result holds X, and everything younger queues behind it
  in order. This is what caps the return on every non-blocking unit, and it is the next
  structural thing to fix if the goal is out-of-order.
- **`IW=1`.** The aligner emits at most one instruction per cycle, exactly what the backend
  consumes, so the F/X queue can never build a backlog and every frontend hiccup is
  unrecoverable. On integer workloads this is the dominant cost (frontend 41.8% of cycles on
  `sha256sum`); on GB5 it is 2.2%.
- **Stores and AMOs still block M.**
- **No memory disambiguation** — a younger memory op simply waits for `S_IDLE`.
- Workload sensitivity is large and measured: GB5 is FPU- and serialisation-bound,
  `sha256sum` is frontend-bound. Do not generalise a CPI stack from one workload.
