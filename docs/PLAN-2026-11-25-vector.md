# RVA23U64 on SmolRV64: the vector extension and the small ones -- design space

Status: PLAN, 2026-11-25. Tommy's decisions are in §6; increment 0 (the small ones) is in.

## 1. Why, and what "done" is

Ubuntu 25.10 and later are built for RVA23U64; the boards run 25.04, the last release that
does not need it. The gate is the user profile: every instruction the distribution's compiler
may emit. RVA23S64's H (hypervisor) is not on that path -- the kernel boots without it -- and
is out of scope here.

RVA23U64 (ratified 2024-10, `riscv-profiles/src/rva23-profile.adoc`) against the RTL before increment 0:

| extension | state | cost |
|---|---|---|
| M A F D C, B (Zba Zbb Zbs), Zicond, Zicsr, Zicntr, Zihpm, Zicbom/op/oz, Zic64b, Za64rs, Ziccif/rse/amoa | have | -- |
| Zihintpause, Zihintntl | have: their encodings are FENCE / `ADD x0` hints, already no-ops | -- |
| Zicclsm (misaligned loads/stores supported) | have for scalar (OpenSBI emulates what traps); vector misaligned: §5.6 | -- |
| Zkt (data-independent timing of the listed scalar ops) | to confirm: add/logic/shift/mul are fixed-latency; the divider is data-dependent but Zkt exempts division | audit |
| **Zcb** (c.lbu/lhu/lh/sb/sh, c.zext/sext, c.not, c.mul) | missing | RVC expander only |
| **Zimop, Zcmop** (mop.r.N / mop.rr.N write 0; c.mop.N) | missing | decoder only |
| **Zawrs** (wrs.nto, wrs.sto) | missing | decoder: the spec lets the wait end at once |
| **Zfa** (fli, fminm/fmaxm, fround/froundnx, fcvtmod.w.d, fleq/fltq) | missing | fli a ROM; fminm/fmaxm and the quiet compares beside FMIN/FMAX in the core; fround/fcvtmod are new fp_fma modes (F2I's alignment, then back) |
| **Zfhmin** (flh/fsh, fmv.x.h/fmv.h.x, fcvt between H and S/D) | missing | a third format in fp_fma's F2F; NaN-boxing of a 16-bit value |
| **Supm** (user pointer masking, PMLEN 7) | missing | an AGU mask in the lanes (opt-in per process: software that does not ask never sees it). In increment 0 |
| **V** (Zve64d + Zvl128b: VLEN >= 128, ELEN 64, FP32 and FP64 elements) | missing | the rest of this document |
| **Zvfhmin** (vfwcvt/vfncvt between FP16 and FP32) | missing | the FP16 format of Zfhmin, per element |
| **Zvbb** (vandn, vbrev, vbrev8, vrev8, vclz, vctz, vcpop.v, **vrol, vror**, vwsll) | missing | the vector integer unit; contains all of Zvkb, so vrol comes with it. No other Zvk*. |
| **Zvkt** | -- | no data-dependent latency in the vector integer, multiply, slide or gather paths (gather's index values and the slide amount are exempt) |

Simmerv already models all of RVA23U64 except Supm (`--rva23`, `--vlen 128|256`), checked
against QEMU and the Tenstorrent suite. It writes **agnostic elements undisturbed**, which is
also the cheapest policy for the RTL, so the lockstep checks vector state bit for bit from the
first increment.

## 2. The constraints that decide it

- **Area.** Chip 120.3K LUTs, CLB 80.7% (7a94f999's routed build); core 87.4K. The core built
  and closed at 159.5K before the diet (ae31c628), so the ceiling is known, but every 10K costs
  timing effort. BRAM 197/480 used, DSP 76/1824: both nearly free; URAM full.
- **Workload shape.** Ubuntu's code is auto-vectorized general code, not HPC kernels: short
  vectors, a `vsetvli` per strip-mine iteration, unit-stride loads and stores, compares into
  masks, and vector-to-scalar results feeding branches (glibc's RVV `strlen`/`memchr`:
  `vle8ff` -> `vmseq` -> `vfirst.m` -> branch). Latency between scalar and vector code matters
  more than DLEN.
- **Nothing may get slower.** Once V is advertised, glibc's ifuncs and the kernel pick vector
  routines; a vector unit slower than the scalar code it replaces is a regression across the
  whole system. Unit-stride memory and the mask/scalar round trip must be at least scalar speed.
- **The core's rules** (`docs/rtl-rules.md`, §5.1): one writer per register-file shard; every
  handshake into the scheduler and LSU registered; the integer scheduler is the critical path;
  integer performance is never traded for FP; a precondition computed once.

## 3. What others did (survey, sources in the 2026-11-25 notes)

| design | VRF | coupling | memory | note |
|---|---|---|---|---|
| Saturn (Berkeley) | flops, own, not renamed | **post-commit**, decoupled sequencers | own port beside L1; vector dispatch waits for the scalar store buffer to drain | vector-to-scalar results stall the host; Saturn 0.1 (ahead of commit) was abandoned for its interlocks with an in-order host |
| Ara2, Spatz (ETH) | per-lane banks / latch SCM | after the host's speculation | own AXI port, coarse scalar/vector ordering | lane-interleaved layout needs reshuffles on EEW change |
| **C910** (T-Head) | **renamed, FP and vector in one physical pool** (64 tags) | **in the OoO issue path**, 2 of 8 pipes FP/vector | through the LSU | 64-bit slices; vector added ~33% core area |
| **XiangShan Kunminghu** | renamed, separate 128 x 128-bit VRF, own v0 and vl files | in the OoO; **one vector instruction a cycle cracked into 128-bit uops by LMUL** | **reuses the scalar load/store units, LQ and SQ** (split + merge buffers) | vtype speculated at decode, restored from the ROB |
| SpacemiT X60 | VLEN 256 | in-order | -- | agnostic = undisturbed; vrgather.vv 4 -> 256 cycles m1 -> m8 |
| UBC RVV-lite (UltraScale+) | BRAM | -- | -- | the one per-feature FPGA cost table: ~7.2K LUTs for a 64-bit integer-only subset without segments, gather, compress or FP |

The two OoO designs closest to ours both integrate the vector unit into rename and issue,
both crack LMUL into register-wide uops at decode, and both keep vectors with or next to the
FP file -- not in the integer file.

## 4. The design space

### Axis 1 -- VLEN

**128** (the minimum) or 256; Simmerv does both. At 128 a vector register is one 128-bit
physical entry, a whole register is one operand, and a full crossbar over it (gather,
compress, slides, widening) is a 16x16 byte network of a few hundred LUTs. 256 doubles the
file and quadruples the crossbars for throughput the D$ cannot feed (§5.6). **128.**

### Axis 2 -- where the vector registers live

**(a) Striped across the lane shards** (the tempting one): chunk k of a vector register in lane
k's integer shard, a vector op cracked into one uop per lane, the lanes' ALUs made SWAR. It
gives DLEN 256 integer arithmetic for the price of deeper shards, and the shards' read
crossbar (every lane reads every shard) is already an inter-lane network. What it runs into:

- **Writer sharding.** A compare's mask result for all four chunks lands bit-packed in one
  register's chunk 0 -- lane 0's shard -- so four lanes write one shard. The same for
  reductions, `vcpop`/`vfirst`, narrowing and every mask op.
- **The schedulers.** A masked, tail-undisturbed op reads vs1, vs2, the old vd and v0: four
  sources in the integer scheduler, which is the critical path, and a uop pinned to lane k
  breaks the lane rotation.
- **FP has no home.** The FP port never reads an integer shard (asserted, §5.1); four lane
  FMAs are ~15K LUTs.
- **The memory side does not keep up.** The D$ answers one 64-bit load a cycle, so DLEN 256
  only pays on register-resident integer SIMD, which auto-vectorized code rarely is.
- **VLEN is tied to IW.** At VLEN 128 it occupies two lanes.

**(b) On the FP side, renamed, beside the FP file** (C910/Kunminghu): a 128-bit vector
physical file, renamed like the FP shards, executed by a vector unit at the F stage on the
FP port. The FP side already has everything this needs:
- out-of-order issue among its own ops;
- tagged results returned out of order;
- a hand-over for an integer source (the lane reads rs1, §5.1);
- a landing for an integer result through the lane's landing buffer;
- flags that are safe out of order.

The vector file has **one writer**, the vector unit (§5.6), so writer sharding holds by
construction. The integer lanes, their schedulers and their shards are untouched.

**(c) Decoupled post-commit** (Saturn): own unrenamed VRF in BRAM, sequencers behind commit, own
memory path. Cheapest storage, no rename changes. But every vector-to-scalar result (`vfirst`,
`vcpop`, `vmv.x.s`) waits for commit plus the unit's queue, which is the strlen/memchr loop's
critical path; every vector memory op waits for the scalar store queue to drain; and it needs
its own TLB check and a coherent port beside a VIPT D$ that already has two clients.

**(b).**

### Axis 3 -- the vector file's storage

The file holds 32 architectural registers, the rename margin and a few temporaries.
Proposed: 64 or 96 entries of 128 bits, with five read ports (vs1, vs2, old vd, v0, and a
store/extract read) and one write port.

- **LUTRAM:** about 0.9K LUTs at 64 entries, about 1.7K at 96.
- **BRAM:** about 10 RAMB36 and roughly zero LUTs, but the read becomes synchronous, so the
  address is presented at the pick and the op takes one more cycle.

Settle it with an OOC build of the F stage with each version.

### Axis 4 -- LMUL and the uops

Crack at decode, one uop per 128-bit register (Kunminghu), from a decode-time `vtype`. The
cracking rules by class:

- **Widening** (EMUL = 2·LMUL): two uops per source register, each taking one half.
- **Narrowing:** the reverse, two source halves per destination register.
- **Reductions:** a chain over the group, then a tree inside one register.
- **Gather at LMUL > 1:** LMUL² uops, quadratic, as on X60.

A uop with four vector sources takes a whole dispatch row's rename-map reads, so a vector
group dispatches at most 2 uops a row; at most 4 rows for LMUL = 8.

### Axis 5 -- vl and vtype

- **vl is a renamed register**, a hidden architectural integer register.
- **`vsetvli`/`vsetivli`** carry `vtype` in the immediate, so decode knows `vtype` for every
  younger op. Each cracks into two lane ALU uops, `vl := min(AVL, VLMAX)` and `rd := vl`. A
  vector op reads vl as its integer source through the existing hand-over.
- **Decode's `vtype` is per-row ROB state** (8 bits × 64 rows) and is restored on a walk or
  squash.
- **`vsetvl`** (vtype from a register) is rare. It fires as a system op at the ROB head and
  refetches.
- **`vstart`** changes only at a trap or a CSR write, both of which refetch, so decode sees its
  architectural value. A nonzero `vstart` cracks with the prestart elements undisturbed, and
  every vector instruction's commit clears it.

### Axis 6 -- vector memory (the hard part)

**Step 1: crack into the scalar LSU; change nothing there.**
- **Unit-stride.** A 128-bit register becomes two 64-bit loads into hidden FP temporaries plus
  one *pack* uop on the vector unit. Stores are the mirror: an *extract* uop, then two `fsd`s.
- **Bandwidth.** 8 bytes a cycle, the same as scalar `memcpy`, so nothing gets slower.
- **Precise traps.** Precision is per uop: the uop carries its element number, and a trap
  writes `vstart` from it.
- **Fault-only-first** (`vle8ff`). Loads after the first report a fault as a status instead of
  trapping, and a final uop computes the new vl.
- **Strided, indexed and segment.** These go one element per uop (extract the index, add,
  load, insert); segment loads become unit-stride loads plus deinterleave uops.
- **Masked.** An inactive element must not fault; it is a page-local split plus the mask.
- **Misaligned.** As scalar.

**Step 2, if the measurement asks: a 128-bit unit-stride path.**
- **Loads.** The D$ reads two 64-bit chunks of a line (the line is spread across eight BRAMs)
  and lands 128 bits.
- **Stores.** SQ entries get 128-bit data and a 16-byte mask.
- **The cost is in the SQ.** The forwarding cross-product (§8) is what grows, so this step is
  justified only by a measured unit-stride bottleneck.

### Axis 7 -- the vector execution units (beside `fp_unit`, F stage)

| unit | scope | estimate |
|---|---|---|
| integer SIMD, 128 bits, SEW 8/16/32/64 | add/sub/adc/sbc, logic, shifts, compares to mask, min/max, merge/move, Zvbb (vrol/vror, clz/ctz/cpop per element, brev/brev8/rev8, andn, wsll) | 5-7K |
| permute, 16x16 byte crossbar | slides, gather, compress, widening/narrowing halves, pack/extract/deinterleave for memory | 1-2K |
| mask | vcpop.m, vfirst.m, vmsbf/sif/of, viota, vid, mask logic | ~1K |
| multiply, SIMD over DSPs | vmul/vmulh*, vmacc/vmadd, widening multiplies, fixed-point (vsmul, vssra, vnclip, vaadd, vsadd with vxrm/vxsat) | 2-3K + ~32 DSP |
| divide | vdiv/vrem: element-serial, iterative | ~1.5K |
| FP | through `fp_fma` and `fp_divsqrt` one element a cycle, plus FP16 for Zvfhmin; compare/min/max/sgnj/class in the F stage's in-core FP ops, widened | ~1-2K |
| decode, cracking, vtype | ~400 encodings, the uop sequencer | 3-6K |
| rename, ROB, CSRs | +32 vregs and temporaries in the arch space, the vector free list, vtype per row, vstart/vxrm/vxsat/vcsr/vl/vtype/vlenb, mstatus.VS | 2-3K |
| vector file | §4 axis 3 | 0-2K |

**Total about 17-27K LUTs**, which puts the chip at 137-147K, under the 159.5K the core
already closed at. FP vector throughput starts at one element a cycle; a second `fp_fma`
(+3.6K, 9 DSP) doubles it if a workload asks.

### Axis 8 -- tail and mask policy

**Agnostic = undisturbed**, as in Simmerv, Ara2 and X60. Every tail- or mask-active uop then
reads the old vd, which the five read ports already cover.

## 5. Order of work

Every increment is locksteped against Simmerv `--rva23 --vlen 128` and gated on the boards.
V is advertised (misa.V and the device tree) only when all of it is in.

0. **The small ones.** Zcb, Zimop/Zcmop and Zawrs come first (decoder only), then Zfa and
   Zfhmin (fp_fma modes and the FP16 format). Each is advertised as it lands, and Supm with them.
1. **Vector state.** The CSRs, mstatus.VS, vset*, decode vtype, renaming the vector registers,
   the vector file, whole-register moves, loads and stores (`vl<n>r`, `vs<n>r`), vmv.
2. **Integer vector.** The integer SIMD unit and unit-stride memory (step 1 of axis 6).
3. **Masks and permutes.** Masks, permutes, reductions, widening/narrowing, fixed point, Zvbb.
4. **FP vector and Zvfhmin.**
5. **The rest of memory.** Strided, indexed, segment and fault-only-first.
6. **Advertise V.** Then boot an RVA23 Ubuntu userland on the boards and measure. A wider
   unit-stride path, a second FMA or VLEN 256 each need a measured case.

Test sources: chipsalliance riscv-vector-tests, the Tenstorrent suite Simmerv already passes,
rvv-bench, and our benchmarks rebuilt with `-march=rva23u64` (the full-march rule).

## 6. Decisions (Tommy, 2026-11-25)

1. VLEN 128.
2. The vector registers live on the FP side, renamed, beside the FP file.
3. Memory is cracked into the existing LSU first, widened only on a measured need.
4. Supm is in increment 0.
5. Zvbb (all of it, for vrol) is implemented; Zvkb is not advertised.
6. One vector unit: a uop is one 128-bit register a cycle; a second unit, a second FMA or VLEN 256
   only on a measured case.

## 7. Increment 1: vector state

What the core has after it: mstatus.VS and the vector CSRs, `vset{i}vl{i}` and `vsetvl`, the 32
vector registers renamed into their own file, the vector unit's moves (`vmv<nr>r.v`, `vmv.v.v/x/i`,
`vmv.x.s`, `vmv.s.x`) and the whole-register loads and stores (`vl<nf>re<eew>.v`, `vs<nf>r.v`).
V is not advertised (misa.V and the device trees stay as they are), so Linux never runs it; the
directed tests and the lockstep do.

### 7.1 Cracking: in the bundle register

An instruction that needs more than one uop is cracked where the bundle register (`pb_in`) feeds
the queue. Each slot's uop count is predecoded when the register loads (from the instruction and
the decode state, below) and held with the slot. A bundle then pushes in as many cycles as its uops
need, slots and uops in program order; `pb_ready` waits for its last part. A push is the run of
one-uop slots from the cursor, or up to FW uops of the cracked slot at it: each queue bank's slot
then comes from flops (`pb_cs`, `pb_cr`, `q_wp`), as without cracking. Packing uops across slots
put the cursor's running sums in front of the slot decoders (fad8af8a: WNS -1.32 ns, 23 levels).
Each uop is a queue entry of its own with the instruction's PC and bits, its uop index `ux` and a
`last` bit, so the queue, the head's rebuild, dispatch, the ROB and the LQ/SQ see ordinary entries.

**Seqnos are given at the push**, one per uop, by a counter the redirect sets (`redirect_seq`):
the LQ/SQ order loads against stores by seqno (`older`), so a cracked load and store each need
their own. Fetch's own count is then read by nothing (`cur_seq` has no consumer today).

**Retirement.** Every uop but the last commits as `d_noret` (the ROB's existing not-counted
commit, the interrupt pseudo-op's); the last retires the instruction, so minstret, the cosim's
retire and a trap's epc are the instruction's. A uop that traps traps the instruction.

### 7.2 Decode state

The frontend holds, in program order at the predecode: `vtype` (vlmul, vsew, vta, vma, vill), the
**vl version** (below) and **vz**. `vset{i}vl{i}` is decoded completely there (its vtype is an
immediate), so every younger instruction is predecoded with the state it executes under. Each
queue entry carries the state before its instruction, and the record the state after it.

- **vstart** changes only by a CSR write (which refetches), a trap (which goes to a handler, whose
  `sret` refetches) and a vector instruction's retirement (which clears it), so its CSR value is
  exact at a refetch. vz says a vector instruction has been predecoded since: vstart is 0. A uop
  carries vz and the F stage reads vstart as `vz ? 0 : the CSR`.
- **vsetvl** (vtype from a register) is a SYSQ op: it fires at the ROB head on a drained machine,
  writes the decode state, vl and rd itself, and refetches the next instruction.
- **Restore.** A per-entry table beside the ROB (column x row, the state after each entry) is
  read at a lane's redirect, as the branch's own snapshot (rule D14); a decode redirect takes it
  from its record; a squash at the head takes the committed copy, which commit keeps from the
  same table (every commit port, rule C6), with vz 0.

### 7.3 vl: versioned at decode

`vl` is not an architectural register in the map. Each `vset{i}vl{i}` that sets vl allocates the
next **version**, a register of a small file on the F side (the VL shard: 16 entries of 8 bits;
VLMAX is at most 128), in ring order by the predecode, not from a free list. Its vl uop writes
the version; a vector uop names the version it reads in its record, a fourth source the F-side
scheduler waits on like any other. A version is free when the next one's `vset` commits; the
predecode stalls when all 16 are live. vl therefore costs no rename-map read.

- `vsetvli rd, rs1` (rs1 != x0): uop 0 hands rs1 to the F stage, which writes `min(rs1, VLMAX)`
  into the version; uop 1, the last (it retires the instruction), is a lane ALU op
  `rd := minu(rs1, VLMAX)` (VLMAX an immediate).
- `vsetivli`, and `vsetvli` with rs1 = x0 and rd != x0: vl is a constant; the same two uops with
  an immediate.
- `vsetvli x0, x0`: vtype changes, vl is kept: no version is allocated (Simmerv's behaviour; the
  spec reserves a change of VLMAX here).
- An unsupported vtype sets vill and vl = 0, as `legalize_vtype` in Simmerv: reserved bits, vlmul
  100, SEW above 64, or a VLMAX of 0.
- `csrr vl` reads the committed version, `csrr vtype` the committed vtype, `vlenb` is 16; vl, vtype
  and vlenb are read-only.

### 7.4 Registers

- **Architectural ids are 7 bits**: 0-31 x, 32-63 f, 64-95 v, 96-127 hidden. The hidden ones are
  FP-kind temporaries (one in this increment, VH0: a move's scalar, and a register's chunks on
  their way to or from memory), mapped at reset to SH_FF's first register. The rename maps grow to 128 entries; every literal 6-bit id (decode_operands, the
  records, rename, the ROB, the LQ, the cosim's capture) becomes the one width.
- **A vector register is `{SH_V, index}`**, shard 6, 64 entries of 128 bits, written only by the
  vector unit (rule B2), read by it for vs1, vs2, vs3/v0 and the old vd. **The VL shard** is 7: a
  VL uop's destination tag is `{SH_VL, version}` (rename allocates none), so its landing wakes and
  clears pending on the FE port as any result does.
  `shard[2]` then means "the F side" (FP or vector), which is what every lane-or-not test of it
  asks; what tells SH_FL from SH_FF (the pending table's index, the PRF's port mux) names them,
  and the vector and VL registers get pending bits and ports of their own.
- **Old vd** is the rename's own `r_pold`, the mapping the destination replaces: a vector uop reads
  it as a source (tail and prestart elements are undisturbed, the agnostic policy).

### 7.5 Scheduling and the vector unit

Vector uops are FP-class: they go to the FP queue and scheduler (`u_iq_f`), whose entries grow
from three sources to five (vs1, vs2, vs3/v0, old vd, the vl version), and its wakes take the
vector file's and the VL shard's writes, which land on the FE port. The vector unit is the F
stage's: one uop a cycle, single-cycle like the in-core FP ops. Every result takes the in-core
ops' result register (`icr`, with a high half for a vector result), and lands from it: a vector
one through the vector file's own write port, an f-register one (an extract) and an x-register
one (`vmv.x.s`) as FMV's do. A vector uop keeps its instruction's word (the last uop retires it,
and the lockstep compares the word); its place, `{vz, SEW, the register of the group}`, rides in
the record's immediate and the F side's payload. The vector file has three read ports in this
increment (rs1, rs2, old vd); v0's comes with masking.

Increment 1's unit: copy (`vmv<nr>r.v`), the move by element (`vmv.v.*`, `vmv.s.x`: vl, vstart
and SEW from the record and the version), element 0 to a scalar (`vmv.x.s`), **pack** (VH0 into a
chunk of a register, prestart elements and the other chunk from the old vd) and **extract** (a
register's chunk into VH0). A move from an x register (`vmv.v.x`, `vmv.s.x`) first
moves the scalar into VH0 (an FMV.D.X uop through a lane, as any int-to-FP op).

### 7.6 Whole-register memory

Each register is two 8-byte chunks through VH0, a chunk two uops, so a fault at a chunk leaves the
chunks before it written, as the element order requires:

- `vl<nf>re<eew>.v`: per register k and chunk h, an FLD of `[rs1 + 16k + 8h]` into VH0 and a pack
  of VH0 into chunk h of `vd + k` (the other chunk and the prestart bytes from the old vd).
- `vs<nf>r.v`: per register k and chunk h, an extract of chunk h of `vs3 + k` into VH0 and an FSD
  of VH0 to `[rs1 + 16k + 8h]`.
- Every uop keeps the instruction's word (the last retires it); the record's fields are an FLD's or
  an FSD's for the memory uops, and the pack's and extract's place rides in the immediate. A uop's
  position is `{register, chunk, second}`.
- **A trap** in a load or store uop writes vstart with the first element of its chunk (a table
  beside the ROB keeps it, by entry), tval with its address; the handler's `sret` refetches and the
  predecode cracks from vstart's chunk, every chunk before it done. A load's pack keeps prestart
  bytes from the old vd. Past the last element the instruction is one uop that changes nothing.
- The LSU and the store queue are unchanged. Known limits: a misaligned chunk that crosses a page
  traps as a scalar one does, at the chunk's address rather than the faulting byte's; a store whose
  vstart (only a CSR write makes one) falls inside a chunk stores the whole chunk.
- The lockstep checks each chunk store against Simmerv's memory after the instruction
  (`probe_vstore`, `simmerv_read_memory`): Simmerv stores a vector access byte by byte.

### 7.7 CSRs and VS

`vstart` (7 bits), `vxsat`, `vxrm`, `vcsr` live in `csr_file`; `vl`, `vtype` come from the
committed state. `mstatus.VS` (and sstatus's view, and SD) is writable and resets Off. VS Off
makes every vector instruction illegal at the predecode (as FS Off does) and every vector CSR
access illegal; a write that changes VS or vstart refetches. Commit of any vector instruction's
uop sets VS Dirty (every commit port) and the last one clears vstart.

### 7.8 Lockstep

Simmerv runs with V (`simmerv_set_isa(ctx, true, true)`), VS Off at reset as the core's, and the
harness takes the core's misa. A uop that writes a vector register reports it as it commits
(`probe_vwrite`: the register and its 128 bits); the harness holds them until the instruction
retires and compares each against Simmerv's register file after its step (`simmerv_read_vreg`;
`+vw_corrupt=<n>` corrupts the n-th, the check's self-test). vl and vtype are compared where a
program reads them (`csrr`, `vset*`'s rd). Simmerv fixes found against the spec: `vmv<nr>r.v` writes whatever vl is
(it skipped at vl = 0), and a whole-register access counts vstart in elements of EEW (it counted
bytes).

### 7.9 Steps, each locksteped and with a directed test

- **a.** Simmerv: the vector-state API, the VS reset, the two fixes; the harness's misa.
- **b.** VS and the vector CSRs; vector instructions illegal while VS is Off.
- **c.** 7-bit architectural ids: no behaviour change, the Linux locksteps cycle-identical.
- **d.** The cracker and the decode state, with `vset*` and the VL shard as its first users.
- **e.** The vector file, its renaming and the vector unit's moves.
- **f.** Whole-register loads and stores, and their traps.

Then the board gates on the whole increment.

## 8. Increment 2: integer vector and unit-stride memory

Tommy, 2026-11-26, chose (A) of three: aligned doublewords with vl at the lane, over predicting vl
at decode (glibc's short `memcpy`/`strlen` would refetch on nearly every call) and one uop per
element (4-16x slower than scalar).

### 8.1 The integer operations

`vadd`, `vsub`, `vrsub`, `vand`, `vor`, `vxor`, `vsll`, `vsrl`, `vsra`, `vminu`, `vmin`, `vmaxu`,
`vmax`, in their `.vv`, `.vx` and `.vi` forms (`vrsub` has no `.vv`, `vsub` no `.vi`), unmasked
(vm = 1): one uop a register of the group, as `vmv.v.*`. The unit's new value is an ALU result per
SEW over vs2 and the second operand (vs1, the scalar in VH0 after a move-in, or the immediate:
simm5, uimm5 for the shifts); the per-byte merge (vstart, vl, the old vd) is the moves'. Masked
forms, the compares, `vmadc`/`vmsbc`, `vmerge`, multiply, divide and fixed point come with
increment 3, and are illegal until then.

### 8.2 Why the LSU changes

Decode knows vtype but not vl (an AVL from a register), and not the base's alignment. A unit-stride
access must touch no byte past vl or before vstart, and fault on none, and an element-aligned access
that crosses a page must not raise address-misaligned (today the LSU raises it for any misaligned
access across a page, `smolrv64_lsu.v`). So each memory uop decides at its lane what it touches.

### 8.3 The uops

For `vle<eew>.v` / `vse<eew>.v` (vm = 1) with EMUL = EEW/SEW * LMUL registers (one for a fractional
EMUL; a reserved EMUL is illegal), base b = rs1, o = b mod 8, the instruction's bytes
[S, B) = [vstart * EEW, vl * EEW) from b:

- **The end** (an F-stage in-core op with an integer source, as the VL uop): XH0 := o + B, from rs1
  and the VL file. XH0 is an **integer-kind hidden register** (ids 104-111 are integer hidden; it maps
  at reset to SH_IE's register 32, so SH_IE's free list starts at 33).
- **The base's move-in**: VH1 := rs1 (FMV.D.X through a lane), for the packs' and extracts' o.
- **Doubleword k**, k from S/8 to 2*EMUL, is [8k, 8k + 8) from b - o, aligned, so it never straddles
  and never crosses a page. Its active bytes are [lo, hi) = [max(o + S, 8k), min(XH0, 8k + 8)), the
  memory uop's lane computes them from rs1 (o), rs2 = XH0 and the immediate (k and S):
  - a **load** reads the whole doubleword into VH0 when lo < hi (its other bytes are on the same
    page), else is a null load: no translation, no fault, no access;
  - a **store** writes bytes [lo, hi) of it (a **byte count**, 0 to 8, in the SQ: a count of 0 drains
    as nothing), its data from rs3 (VH0), rs2 being XH0.
- **A load's pack** of doubleword k writes its active bytes into the group, bytes [8k - o, 8k - o + 8)
  of it: one register for odd k, two (the register before and after) for even k, a pack a register
  (sources VH0, VH1, the old vd, the VL file); it follows the doubleword's load.
- **A store's extract** for doubleword k gathers group bytes [8k - o + lo, ...) into VH0, byte 0 the
  first active one (sources the two registers it may span and VH1).
- An element-misaligned base (o not a multiple of EEW) raises address-misaligned (cause 4 / 6), as
  the spec allows.

A group of EMUL registers is then 3 + 5 * EMUL uops for a load, 4 + 4 * EMUL for a store.

### 8.4 Traps

With an element-aligned base, elements start on doubleword boundaries, and every doubleword before k
is loaded and packed (or stored) before k's access, so a fault at doubleword k leaves exactly the
elements before its first active byte done. The lane computes that element, (8k + lo - o) >> log2
EEW; the LQ/SQ entry keeps it (7 bits) with the address, and a vector memory fault writes vstart
with it (tval is b - o + 8k + lo, the faulting element's address). The retry cracks from vstart's
doubleword.

### 8.5 What changes in the LSU and the queues (the census, 2026-11-26)

- The D$ takes a byte mask already: no change.
- The SQ's `sz[]` (2 bits, log2) becomes a byte count; every range test is `+ (1 << size)` and turns
  into `+ count` (`ovl_ab`, `cov_ab`, `dw[]`, the page-cross assertions); the LSU's commit mask
  (`sp_wm`) becomes `((1 << n) - 1) << off`. Scalar stores are unchanged: their count is 1 << size.
- The LQ gets a null bit and a whole-doubleword bit (the raw doubleword, unformatted).
- The lane's rs2 port (integer, idle for loads and stores) carries XH0; the scheduler waits on it
  for a vector memory uop (a store's rs2 is waited on today only as data, by the SQ's snoop).
- The cosim's store-size convention becomes a byte count.

### 8.6 Steps, each locksteped and with a directed test

- **a.** This design.
- **b.** The integer operations.
- **c.** Byte counts in the SQ and the LSU: no behaviour change, the Linux locksteps cycle-identical.
- **d.** The integer hidden register XH0.
- **e.** Unit-stride loads and their traps.
- **f.** Unit-stride stores and their traps.

Then the board gates on the whole increment.
