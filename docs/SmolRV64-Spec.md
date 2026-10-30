# SmolRV64 — microarchitecture specification

The core in `core/`. **This file is normative and must be updated in the same commit as any
change it describes** (docs/rtl-rules.md H4). Every number here is read off the RTL; where a
figure is *measured* rather than structural it says so, with the workload.

Naming: `rv_*` modules are generic RISC-V blocks, `smolrv64_*` are specific to this core.
The blocks the core inherited from its predecessors (`fetch`, `aligner`, `rvc_expand`,
`decode_*`, `alu`, `mul3`, `divider`, `mmu`, `csr_file`, `fp_unit`) and the SoC devices live
in `src/`. The sequential core and the sharded-OoO core that came before it were removed in
the 2026-09 release; their history is in git, and the dated records in `docs/history/`.

---

## 1. What it is

| | |
|---|---|
| ISA | RV64IMAFDC (`misa` = A, C, D, F, I, M, S, U; MXL=2) |
| Privilege | M / S / U |
| Translation | Sv39 (`satp.MODE`=8) or Bare; Ssvnapot level-0 NAPOT leaves |
| Also implemented | Zicsr, Zifencei, Zicntr, Zihpm (13 counters), Sscofpmf, Sstc, Smstateen, Ssvnapot |
| Decoded but not in `misa` | Zba, Zbb, Zbs, Zicond (`src/decode_exec.v`) |
| Fetch / dispatch / retire | **four-wide** (`SMOLRV64_IW=4`, set once in `core/smolrv64_width.vh` for the RTL, the build and the benches; `SMOLRV64_IW=3` and `2` build the three- and two-wide machines); fetch is one 16-byte pair per cycle into the fetch ring (§4) |
| Issue | **dynamic**: ALU ops, multiplies, divides, branches and jumps issue in their slot's lane (a scheduler, an ALU and a multiplier per lane, four of each; a divide's operands go to the one MD stage) and reorder freely; a system op issues in lane A and fires from the SYSQ at the ROB head; FP ops reorder on the FP port (§7); loads, stores, AMOs, LR/SC and CBOs generate their address in their slot's lane and reach M through their LQ/SQ entry or the head-op register (§8) |
| Completion | **out of order** (non-blocking loads, tagged FP results, ALU at issue) |
| Commit | in order, from the ROB head, up to `IW`/cycle |
| Speculation | branch/jump prediction; loads ahead of older stores of unknown address (§8); an address generator issues on a load's D$ lookup before it answers (§8); no value speculation |
| Target | AMD XCKU5P, `probe_clk` **166.67 MHz** (6.000 ns) at `PROBE_CLK_DIV8=48` |

Sections that describe the core as it was when they were written say so.

---

## 2. Pipeline

Three architected stages plus a commit point. `F` is itself pipelined internally.

```
  F  ── PC → I$ (VHPR: virtual hit) → fetch ring → aligner → RVC expand → decode ──────┐
       (the iMMU runs alongside: its PA is the I$ miss-path physical tag, not on the hit) │  decoupling queue (16)
  X  ── rename → PRF read / M→X bypass → ALU, branch resolve ──────────────────────────────┘
  M  ── LSU / mul / div / FPU / CSR ── trap, redirect, BTB train
  C  ── ROB head: architectural commit, minstret (delayed one cycle; a CSR op waits a second cycle at head), free-list release
```

| stage | holds | can stall | can stall others | can restart the pipe |
|---|---|---|---|---|
| F | the stream and its predictor, fetch ring window, aligner | yes | no | no |
| decoupling queue | up to 16 decoded instructions (`SMOLRV64_QD`), each with its dispatch class (`smolrv64_gclass`, evaluated once at decode: scheduler, plain, decode-redirect; FS Off is sampled there, exact since a write to FS refetches) | — | back-pressures F when full | no |
| X | one decoded+renamed instruction | yes (`d_hold`) | holds F via `accept` | no |
| M | one instruction | yes (`m_done` low) | holds X via `m_advance` | **yes** (`redirect`) |
| C | ROB head | — | `head_block` holds M | no (the redirect fires from M) |

Only **M** redirects. Anything that redirects or traps must additionally be the **ROB head**
(`head_block`), because a younger instruction must not squash an older load or FP op still
in flight.

### 2.1 Issue is dynamic — for ALU and FP ops

**X is no longer a single slot that everything funnels through.** Dispatch renames,
allocates a ROB slot and a scheduler entry, and moves on; it does **not** wait for operands.
The scheduler holds the instruction until its sources are ready and then issues its **oldest
live entry when that one is ready, else by fixed priority, lowest entry index** — no tag is ever
compared for age (§6.1).

What may reorder is deliberately narrow, but it is no longer only the ALU. **Pure ALU ops
and FP arithmetic both reorder freely**, in the lanes' schedulers (`ln[k].u_iq`) and `u_iq_f`
respectively. **Slot k's ALU op goes to lane k** (lanes plan step 5.2a): an integer scheduler
per lane, four, each with its own ALU and shard, and no steering. **The lanes rotate** (§3.2):
consecutive instructions go to consecutive lanes, so every lane takes every fourth instruction. An ALU op completes at issue; an FP op goes to stage F. Neither ever enters M.

Everything else carries an `ord` bit; of those, FP ops go to the FP queue, and branches, jumps,
multiplies, divides, system ops (lane A), loads, stores and the head ops (AMO, LR/SC, CBO) run in
the lanes (§7, §8). Memory ordering lives in the LQ and SQ and in M's in-order fill. Memory
ordering lives in the LQ and SQ, not in issue order.

That split is what makes the rest unnecessary rather than merely deferred:

| usually needed for OoO | why not here |
|---|---|
| deferred traps / redirect register | anything that can trap issues only as the oldest entry, so traps still fire in order |
| per-unit zombie bits at flush | nothing younger is ever in flight when a trap fires, so there is no writeback to suppress |
| memory disambiguation, store queue | memory keeps program order |
| deadlock avoidance | an op head-blocked in M cannot starve an older one: the only things that can be older and un-issued are an ALU op or an FP op, and **neither ever enters M** |

**Operands are read at ISSUE**, addressed by the selected entry's physical registers — doc 1's
"values live in one place". The old M→X bypass is gone; a writeback→issue forward
replaces it, and its select is the scheduler's own wake match: an op picked in the cycle a
source's register is written back reads it from that cycle's write register, which is the
source's shard's lane's (a lane writes only its own shard; a stream lands a register in the lane
of its shard). The scheduler registers every entry's matches (`iss_hq`), so the execute cycle
selects by two flops and compares no tag, and readiness and data agree by construction.

Wakeup is split in two. **Fast** wakeups are writebacks whose value can be forwarded in the
same cycle, and none of them may depend on which entry was selected or readiness becomes a
function of itself. **Slow** is the ALU op completing at issue: its result is registered at the
end of the issue cycle and written to the PRF a cycle later (2026-09-05; before that the write
landed at the end of the issue cycle itself), so a dependent cannot read it before the next
cycle anyway, and waking it late costs nothing. The one cycle in which a dependent selected
right behind its producer reads the file before the write lands is covered by a forward from
that writeback register (`alu_q`, a tag compare and a 2:1 mux on each operand). Everything
issue-timed -- the wake, the pending clear, the store queue's data snoop, the ROB's done, the
cosim capture -- stays at issue, so no consumer waits longer. Registering the write took the
write leg off what was the design's critical path: issue register → PRF read → ALU → PRF
write data in one cycle, 13 levels and 82% route, +0.001 ns on Q, −0.034 on S.

Measured, 300 M-cycle Linux cosim, retires: **67,160,189 → 68,981,116** (dispatch decoupled)
**→ 69,754,834** (dynamic issue) — **+3.9%**. Small on this workload because a Linux boot is
frontend-bound (§14) and only ALU ops reorder; the buckets it targets are `ST_MEM` and
`ST_FPU`, which dominate GB5 rather than a boot.

### 2.2 Why the decoupling queue exists

`accept` (X←F) is registered off M-stage state. The queue decouples fetch from the backend so
a fetch bubble and a backend stall do not serialise. Fetch pushes a bundle of up to FW
instructions a cycle (`SMOLRV64_FW`, 6, up to 8; at IW=4 a width of 8 took 0.09% more cycles
over GB5's 77 SimPoints) and dispatch pops a group of up to IW, so the queue refills faster than dispatch drains it. Depth `SMOLRV64_QD` (16),
at least 2 x FW: a bundle is pushed only with room for a whole one.

---

## 3. Stall taxonomy

Every cycle is charged to exactly one top-down cause (`td_k`, §11 and §11.2): dispatching, bad
speculation, front-end or back-end, and a cause beneath each. The stall counters below (§11's
`ST_*`, `FE_*`) say what each kind of wait looks like; they overlap, and only `td_k` ranks them.

### 3.1 F stalls — the frontend has nothing to hand over

| cause | counter | note |
|---|---|---|
| I$ miss | `FE_IC` r0312 | line fill through the memory arbiter |
| iTLB miss / page-table walk | `FE_MMU` r0311 | PTW runs as a line requester |
| had bytes, no complete instruction | `FE_ALN` r0313 | window is `SMOLRV64_HW` halfwords |
| had an instruction, not yet in the IR | `FE_QUE` r0314 | queue latency, or a dispatch freeze |
| the IR empty, nothing behind it waiting on a credit | `FE_BUB` r0310 | `FE_MMU + FE_IC + FE_ALN + FE_QUE`; not the IR's refill after a backend stall |

### 3.2 X stalls — the queue head without a credit

Nothing after the decoupling queue holds. The queue head forms a dispatch group of up to `IW` (four)
instructions that may go together: every member plain (a serialising op, `fence.i`, a CBO, an
AMO, a CSR op, a trap at dispatch or the interrupt pseudo-op goes alone), at most one divide
(the MD stage tracks two in flight), any number of loads, stores and FP ops (the LQ, the SQ and
the F stage take a group's together, the credits below counting each slot position), and nothing
behind a CTI that redirects fetch at decode (`smolrv64_gclass`, read from the queue record). A
load takes the store-seqno past its group's stores ahead of it.

**The lanes rotate.** Instruction j of a group goes to lane `(lc + j) % IW`, `lc` the lane the
next instruction takes, so consecutive instructions go to consecutive lanes whatever the groups'
sizes and every lane takes every IW-th instruction: the lanes' schedulers fill evenly with no
steering logic. The IR holds the group by lane (slot k IS lane k), a cyclic run from its first
lane `d_fc`; rename, the ROB and the group's load-behind-store test order its members by program
position `(k - d_fc) % IW`. A head op (a non-plain op, alone) goes to lane A, whose paths are
the SYSQ's and the head-op register's: it waits for `lc` = 0, setting it in a cycle that pops
nothing (`realign`, `ROT-SIM` counts them), and lane B takes the next instruction. The queue
reads its heads twice: in program order for the group rules (their classes alone), and at
`q_rp + (k - lc) % IW` for lane k's record, so the rotation selects among the queue banks'
outputs by flops and the record goes straight into the IR. A member's lane credit is its lane's
(`ia`, `crd`'s lane bits rotated by `lc`). On the 60 M boot the lanes dispatch 20.45 / 20.14 / 19.87 M
instructions, a head op waits for lane A in 238,526 cycles, the cycles the head's lane is full
while another lane has room fall from 3,427,455 (slot k as lane k) to 1,551,254 (`IQFULL-SIM`),
and the boot retires 3.82% more (52,794,871 against 50,850,470). The GB5 checkpoint slices (50 M
retires each) take 11.89% fewer cycles in Navigation, 6.79% in HTML5, 6.18% in SQLite, 1.45% in
PDF Rendering and 0.03% in Camera. A member pops only with a **credit** for everything it allocates, from
flops, counted against what is already between the head and there: the ROB against the IR
group, each lane's scheduler against its dispatch-stage register and the IR's op, and per slot
position (room for the op at position k with every slot before it of its kind too) the SQ against
the IR's stores, the LQ's banks with room against the IR's loads, each of which may take a bank's
last entry (§8), and the
F stage against its FIFO (below) and the IR's FP ops, a full stage still taking one a cycle while
the scheduler has room for two (`crd`, `CR_*`; this
cycle's frees are not credited). The IR loads every cycle and
dispatches the next, and a lane's dispatch-stage register never waits for its scheduler; both are
always-on assertions. **The F stage is a FIFO of 8** (`FQ`) whose head is the register its
scheduler fills from: a burst of FP ops waits there while dispatch goes on, and the head waits for
the scheduler's room. A group's FP ops enter together, in program order: the first into the head
when it and the FIFO are empty, the rest at the FIFO's tail, whose distributed RAM is banked four
ways by entry so that each of a row's consecutive entries has a bank of its own. The FIFO's entries watch a registered copy of the wake buses, a cycle late:
an entry ORs the copy's match of its sources into the readiness it had at dispatch, the head adds
the copy's match to its own live snoop, and what the scheduler is handed is the readiness the
pending table would give, in the same cycle, with no port on the table and no load on the live
buses (a check compares them at every move into the head). The head also pops nothing while dispatch is frozen
(`redirect_q`, `fr_v`, `dec_red_q`: what the IR holds then is the wrong path and is dropped),
while a rename shard is below its low-water mark (a lane's shard 4, two groups of one
register each; an FP shard 2 x `IW`, two groups of a full row) -- the flag a register computed from
registers only: the list's free count after the cycle's registered frees and after the allocation
of the group the IR holds, counted from the IR's own registers (`sel_r`) whether or not it
dispatches, so neither the dispatch take nor the commit-and-flush cone stands in front of it -- or rename's map is being swept after a total squash (§5), or while a
serialising op is in the IR or in flight. A CBO, which waits
in M for the ROB head, pops only once every older load and store has its address (`CR_CBO`:
no LQ or SQ entry unfilled, none in the IR), and no load or store pops while a CBO is in the
IR or in flight (`cbo_any`): nothing can need M behind it, reach memory ahead of it, or fill
the store queue it waits on.

A dispatch stall is `hd_wait`, the head holding an instruction it has no credit for. In the
60 M boot (`DISP-SIM`, 5.3c) it waits in 14.8 M cycles; the rules cut a group before slot B in
18.9 M cycles and before slot C in 4.8 M more. The credits cost 0.68% against 5.3b, whose IR
held a group until its room appeared and dispatched it in that cycle.

| cause | meaning |
|---|---|
| *(operands)* | **no longer a dispatch stall.** Waiting for operands happens in the scheduler now (§2.1); dispatch is blocked by structural resources only. |
| `~cr_rob` | no free ROB row within the 57 the ROB fills at IW=4 (`ROB_CAP`, §6) beyond the IR group's (`st_rob`) |
| `~cr_i/l/f` | the member's scheduler (its lane's, rotated by `lc`) has no entry beyond its dispatch-stage register and the IR's op — integer 10 each; the head-op register holds one (`cr_l`: none in flight, none in the IR); for FP (`cr_f`), the F FIFO (8) and the FP scheduler (8, §6.1) have no room beyond the FIFO's ops and the IR's; `~cr_ld`/`~cr_st`, no LQ or SQ entry beyond the IR's (`st_iq`, `st_lq`, `st_sq`) |
| `rn_stall` | any rename shard below its low-water mark (4 free registers for a lane's, 2 x `IW` for an FP shard), or the map's sweep after a squash (`st_rn`) |
| `cr_ser`, `cr_csr` | a serializing op -- a fence, `fence.i`, an AMO, an xret, `ecall`/`ebreak`/`wfi`/`sfence.vma`, a trap from dispatch -- is **alone in flight**: it pops only when the ROB AND the store queue have drained (`drained`, rule C5) and the IR is empty, and nothing pops behind it until it commits. A CSR op does not drain: it waits only for the CSR op before it (`csr_infl`, or one in the IR) (`st_srz`) |

### 3.3 M stalls — `m_done` low

`m_done = m_done_raw & ~head_block & ~((port_yield | sy_at_head) & ~m_nb) & ~m_flt_pulse`

| cause | meaning |
|---|---|
| unit not complete | LSU / mul / div / FPU still working |
| `head_block` | the op is a trap, redirect, `fence.i`, CSR/SYSTEM, fault, or faulting memory op, and is **not yet the ROB head** (`m_at_head`, a flop computed a cycle ahead from M's and the ROB's next state: set in the cycle the head reaches M's op) |
| `port_yield` | a load or an FP result is landing this cycle (`ld_land`, `fp_land`) and takes the write port + M's ROB completion port; M yields |
| `sy_at_head` | the SYSQ's op is the ROB head; M's op, younger, waits for it |
| `m_flt_pulse` | a memory op's fault, in the cycle the LSU reports it: the trap takes the latched fault a cycle later |

A plain load or store (`m_nb`) waits for neither `port_yield` nor `sy_at_head`: it only fills its
queue entry and completes on neither port, and it leaves M in its fill cycle (asserted). From then
on the entry is the queue's -- a load lands and leaves, and its slot is allocated again -- so M's
index into it would name another op's entry.

Completion is **sticky** (`m_unit_done_q`): every unit's `done` is a one-cycle pulse, and the terms
above can hold `m_done` low afterwards, so the pulse and its result/fault are latched or they are
lost.

### 3.4 Restarts

| source | condition | cost |
|---|---|---|
| branch/jump mispredict | fetch restarts at the resolve; rename resumes at the release (§6: the walk back and the kill) or at the squash when the branch reaches the ROB head first | the frontend refill, overlapping the walk |
| trap / interrupt | `xtrap_v`, must be ROB head | full refill |
| CSR-induced redirect | `sret`/`mret`/`sfence`, serializing | full refill |
| `fence.i` | `ifence`, must be ROB head | full refill; the I$ is invalidated only after a device write |
| interrupt injection | `irq_inject`, a solo SYSTEM pseudo-op that traps in M | full refill |

Measured redirect rate: **3.4 per 1000 instructions** (Linux cosim).

**Measured mispredict cost (`workloads/brbench`, the lanes core at IW=3 with restart at
resolve, 5.4e-3, 2026-10-05, L1-resident).** A loop with one random-direction branch per
iteration (xorshift64, so no history predicts it) against the same loop with the branch always
not taken; `drain` puts an older D$-missing load in flight ahead of the branch:

| loop | cyc/iter | redirects/iter | FE_BUB/iter | RD_WAIT/iter |
|---|---:|---:|---:|---:|
| `pred`, never taken | 6.73 | 0 | 1.82 | 0 |
| `near`, target a few bytes away | 15.15 | 0.50 | 4.05 | 0 |
| `far`, target 1 KiB away (I$ hit, the jump back the same) | 15.46 | 0.50 | 5.66 | 0 |
| `drain`, an older D$-missing load in flight | **17.05** | 0.50 | 5.85 | 4.47 |

A mispredict costs about 17 cycles (`near`). Without the release (5.4e-2, the squash waiting
for the ROB head) `drain` is 32.26 cycles per iteration with `RD_WAIT` 19.38 and `FE_BUB`
20.41: the resolved branch waited the whole miss for the head while the right path sat fetched
in the decoupling queue. With it the walk and the kill end the restart a few cycles after the
resolve, and the miss overlaps the right path.

A restart drops the store queue's uncommitted tail only: stores the ROB has committed at
the irrevocable pointer (§6) are architecturally done and drain after the flush.

---

## 4. Frontend

### 4.1 Fetch

- Window: `SMOLRV64_HW` halfwords. The shipping build is `SMOLRV64_HW=8` → **16-byte window** (since
  2026-09-05; 4 before), so the I$ read width is `HW*16` = 128 bits, read as the 64-bit
  chunk pair (§9.1). On sha256sum the frontend bubble went from 43.4% of cycles (I, HW=4) to
  10.1% (M, HW=8 with the senior store queue), IPC 0.435 → 0.714. With the full two-wide
  stack (two-wide fetch, dispatch and retire, the ALU's own port and a second ALU, the
  run-ahead buffer with its page-crossing fix, the T1 cuts; board build 9b3050f9, 2026-09-07)
  `perf stat -e cycles,instructions sha256sum` on a 30 MB tmpfs file reads IPC 1.32 / 1.38 /
  1.39 on the board (three back-to-back runs after boot), against 0.90 with two-wide fetch
  alone (T1F3).
- **VHPR I-cache: the run-ahead fetch buffer is gone; the I$ is virtually hit** (Stage 2,
  2026-09-13). The L1 I$ is virtually indexed AND virtually tagged (`rv_cache` `VIRT=1`,
  §9.1): a hit is `valid & (vtag == VA) & (epoch == cur_epoch)`, so address translation is
  **off** the fetch hit path. The iMMU still translates every fetch, but its PA is used only
  as the miss-path **physical tag** (for reconcile) and for the L2 fill — never on the hit
  compare. VAs are Sv39, sign-extended to 64 bits and masked to the canonical 39
  (`PAW_SIG=39`); the virtual tag is that 39-bit VA. **A PC is 40 bits** (`PCW`=40,
  `core/smolrv64_pc.vh`) everywhere from fetch to the ROB: the two's-complement number whose sign
  extension is the address, which holds every canonical Sv39 address and every PA exactly, so
  `pc + 2`, `pc + imm` and the predictor's arithmetic are done at 40 bits. A 64-bit value that
  becomes a PC (a `jalr` target, a trap vector, an xRET's epc) is packed as `va_codec.vh` packs
  `epc` and `tval`: a non-canonical address stays non-canonical and faults. A PC leaves the core
  sign-extended (the link value, `auipc`, the iMMU's VA, the lockstep's record); `mtvec` and
  `stvec` are stored packed beside `mepc`/`sepc`, so the trap target is a 40-bit sum. A `satp`/`sfence.vma` mapping change
  (`imem_ctx_chg`, narrowed to satp/sfence — a bare U↔S privilege change needs no I$ action;
  permissions stay the iMMU's) advances an **8-bit epoch**, staling every virtual tag in one
  cycle without clearing arrays. Retained lines are then re-validated by **physical-tag
  reconcile** on the next miss, so code physically resident survives a mapping change exactly
  as the old PIPT I$ retained it across `satp` — the coherence requirement that a virtual-hit
  I$ must replicate (`docs/VHPR.md`, "Instruction-Cache Coherence, Outstanding Fills, And
  Mapping Changes"). `fence.i` invalidates once the core's memory operations drain; an I$ fill
  never reads a line the D$ holds dirty (§9.1, the coherent fill); an outstanding fill taken under the old mapping is **poisoned** at the install point
  (`v_wd = 0` when `f_epoch != cur_epoch`). Prefetch (next-line stream) is kept and re-keyed
  VIRT-safe (probe by `r_pa`/`f_pa`, arm by `f_pa`). The single-copy invariant (≤1 valid line
  per physical line) and the epoch roll-over walk are in `docs/VHPR.md`.
- **The fetch ring** (Stage 4 increment 1a, `smolrv64_fring`, in the frontend; the core's instruction
  port is the I$ request and its answer). A ring of 32 halfword slots holds the
  instruction stream from the PC on, filled by an address stream that reads one **window** per
  accepted `rv_icache` request -- up to 16 bytes from the stream's address, any halfword, never
  across its 4 KiB page (the stream address is the predictor's, §4.2) -- and runs ahead of the PC
  while the ring has room for a whole window and the predictor's queue has room. Every request is
  the next 16 bytes of the stream, so after a taken branch the windows are in step with the bundles
  of 4-byte code and the ring's room suffices at 16 bytes a cycle: ChaCha20 (`crypt.c`, its loop 12
  bytes into a pair) runs 25 cycles an iteration, as aligned code does (on the 4 GiB board, its loop
  6 bytes into a pair: 26.09 cycles, as `-falign-loops=16` code does, against 34.09), against 34 when the ring
  asked for 16-byte-aligned pairs and every bundle straddled two of them (the ring then skipped
  every fourth request; docs/PLAN-2026-10-27-fetch-window.md). **Its head is the PC by construction:** the head
  advances by the halfwords fetch consumed (`adv_hw`: a bundle's `consumed`, or one halfword per
  step of a page straddle), and a predicted-taken branch does not empty it -- the predictor
  (§4.2) already steered the stream to the target, whose halfwords follow the branch's; a restart
  (a redirect, or, a cycle later, a mark fetch rejects) empties it and restarts the stream at the
  redirect's target or the PC.
  So the window fetch aligns is a rotation of the ring from its registered head, with no address
  compare on the fetch loop, and each halfword carries a mark bit: it ends a CTI the predictor
  knows. Answers come back in order, and each request in flight is an entry of a small FIFO with
  its halfwords, its marks and a kill bit: a flush kills every one in flight (a generation in the
  tag cross-checks it), and a correction of the predictor's (§4.2) the youngest one or two, whose
  reservation and marks are given back. That needs an answer two cycles after its request, as
  `rv_icache` gives it; the ring asserts it. A `fence.i` or a
  mapping change (`freeze`) empties the ring. **The stream stays inside the PC's enclosing page**
  (4 KiB, or 2 MiB for a >= 2 MiB leaf) and takes its PA from the iMMU's translation of the PC;
  increment 1d translates on an I$ miss instead. The ring replaced Stage 2's alignment adapter
  (two chunk slots, one demand read in flight): the 60 M boot +1.25% (17,277,202),
  `workloads/rvbench` Dhrystone 616 -> 513 us per run, `local/sillyfp` IPC 1.31 -> 2.27 (the
  chunk-tail wait is gone), `local/sillyloop` 1.79 -> 1.89 (one bubble per taken branch left:
  prediction still happens at the aligner, increment 1b).
- **The fetch address is the PC register, bare** (plan item T1 (F) steps 4-6, 2026-09-07; the
  buffer's translation-carry is gone with the buffer). `imem_addr = pc_q` in every state: a
  page straddle steps pc_q to the high halfword's address and `ipc_q` keeps the instruction's
  PC (the bundle's PC, a fault's EPC); a pending interrupt waits out a straddle (`irq_go`)
  instead of abandoning it. The alignment window drops on a context change
  (`freeze = fi_stall | ic_inv_busy | imem_ctx_chg`, gating `imem_ok`); a miss still takes the
  iMMU's fault. `imem_ok` is "the ring holds bytes and nothing is flushing"; a miss still
  takes the iMMU's fault. The cap on the aligner's window is `pc_q`'s index against the **enclosing page's**
  last `HW` halfwords (`in_last`, `hw_left`), not `(pgsz - off) >> 1` compared with HW — the sum
  form is kept as the oracle. **Enclosing page** (Stage 2 inc 3): the cap and the straddle
  boundary are the 4 KiB boundary for a 4K leaf and the **2 MiB** boundary for a ≥2 MiB leaf (a
  1 GiB leaf caps conservatively as 2 MiB). The page size is the **served chunk's** — carried by
  the fetch ring off the fetch loop (captured from the iMMU's leaf level at each I$ request, a
  register; the ring holds only the PC's enclosing page), never re-translated on a hit. Inside a
  2 MiB page the window is no longer chopped every 4 KiB, and the `strad`/`ipc_q` FSM Stage 1 kept
  now fires **only at the true enclosing-page boundary** (a 32-bit op whose high half is in the
  next page, a different translation), not at every 4 KiB sub-boundary within a superpage — where
  the adapter's 2-chunk carry (the next chunk is same-page, PA-contiguous) assembles it instead.
- **Two-wide fetch** (2026-09-05, plan item 10a): the aligner emits up to two instructions per
  cycle (`IW=2`) and both enter the decoupling queue in one cycle; the queue is two LUTRAM banks on
  entry parity, so each bank takes one write per cycle and the head is a 2:1 mux. Decode still
  pops one. A bundle ends at a jump, a SYSTEM op or the page boundary; when a later
  slot's bytes are not in the window yet but are coming (the shortfall is fetch's, not
  the page's: `bytes_late`) the bundle WAITS rather than cutting (item 10e; until then slot 1
  could not cross a 16-byte chunk boundary, which made 338 of the sha256 kernel's 934 bundles
  singles), so a bundle's shape is a function of the code alone, never of chunk-arrival
  timing. (One hole in
  that, closed 2026-09-10: the aligner's "a SYSTEM/AMO/FENCE op starts a fresh bundle" test
  ran on slot 1's halfwords before checking they were in the window, so after a restart into
  a chunk whose successor had not landed, the pair register's stale bytes could end the
  bundle after slot 0 -- a shape the code did not determine, and a third bundle base for the
  loop back edge in `workloads/brbench`, 15 lost predictions in 300 iterations; 2 after.) A
  bundle also ends at a mark predicted taken and carries on through one predicted not taken, so
  a bundle holds up to `IW` predictions, one for each slot that ends on a mark. The fetch ring
  predecodes a START bit per halfword as each window arrives (a halfword begins an instruction unless
  the one before began a 32-bit op; the chain runs on from the last halfword appended and restarts
  at the PC), so the aligner places slot k at the window's k-th start, decodes every halfword in
  parallel and selects per slot: no slot's position waits for the lengths before it. `PDW` = 32:
  {RAS-pointer snapshot 3, history snapshot 11, yhit, yctr 2, yidx 11, the BTB way 1, hit, ctr 2}.
### 4.2 Branch prediction

**At the fetch stream, keyed by the last halfword** (Stage 4 increment 1b). The predictor is the
fetch stream: it holds the stream's address (a registered **window**: its first 16-byte pair and
its first halfword in it), looks the window up as the fetch ring asks the I$ for it, and steers the
stream to the next window. **Two BTBs:** a small L1 in the predictor's loop, and an 8K-entry L2
behind it that checks every window a cycle later and corrects L1 (below). A BTB entry belongs to
its CTI's last halfword; a window covers eight
consecutive halfwords, one in each halfword position, so the BTB and the corrector are each eight
banks by position. Each BTB bank reads the row of the pair its halfword is in -- the window's first
pair at or after its offset, else the pair after it -- and compares the window's tag (the bits above
its 2 KiB block, which the two pairs share except across a block, where the second pair misses: a
lost hint); the corrector's tag for the pair after takes the pair's own bits plus one, wrapping
inside them. The corrector's banks all read the row of the window the stream is leaving (the restart's on a restart) -- read
ahead, so the BTB's read and pick are not in front of its read address. No two CTIs share an entry
(rule B10). Every position evaluates its own hit, corrector hit and direction from registers, and
the hits in window order mark the known CTIs. **A window ends at its first known CTI predicted
taken, or at its second known CTI** (`M`=2): the stream goes on at the target (the RAS top for a
return), at the halfword after the last mark, or 16 bytes on; a predicted call pushes its last
halfword + 2. A window stops at its first pair's end only when that pair is its page's last. A
window therefore holds at most two predictions, both read with the history from before the window,
and the history shifts at most once per window. The corrector is read with the history before the
current window's own shift (a window of lag), and the index read is the one carried for training. Training recomputes the key
from the resolving CTI's PC and length.

**Both BTBs are skewed-associative and taken-only.** Each halfword bank is `W` = 2 ways (`BTBW`=1);
way `w` reads row `lo ^ rotl(hi, w)`, `lo` the row bits above the pair (L2 9, L1 6) and `hi` as many
above those, so halfwords that share a row in one way rarely share it in another. A window reads
both ways of all eight banks; a halfword's entry is its first way whose tag matches. Every jump,
call and return is allocated; a conditional is allocated only when it resolves taken, so a
conditional the BTB misses is one never taken (or evicted) and falls through, with no decode
redirect (`DCR_BACK`=0). Training never reads the entries: a CTI the BTB hit carries the way it hit
in its details, and training rewrites that way; a miss allocates the first way whose
recently-trained bit is clear (set when trained; when that would set both, the other clears), else a
rotating way. Each L2 way's entries are therefore a block RAM with two ports, the check's read and
training's write. An instance predicted before an earlier instance's allocation landed allocates
again, but a first miss mispredicts and drops everything younger, so few do: 11 to 184 duplicate
allocations in about 9 M trainings on each GB5 slice (the lockstep's `BTB-SIM` line, which also
counts entries rewritten after they changed: 0 to 18). Modelled in Simmerv over GB5 single-core (690
G instructions), the BTB-coverage cost (decode redirects and mispredicts a BTB miss causes) is 44.8
cycles/K instructions for the 2048-entry direct-mapped, allocate-every-CTI BTB with backward-taken
decode redirects, 1.9 for 2048 entries 2-way skewed taken-only, and 0.2 for 8192 entries 4-way. Each
position decides from the hits before it in the window alone whether the window ends there: the
window's order is its first pair's banks ascending, then the second pair's, so a bank's hits before
it are its own pair's below it (a constant mask) and, for a second-pair bank, the first pair's as a
whole (any, exactly one, any taken); the offset register only splits the hits in two. **The BTB in
the loop is a small L1, and the 8K-entry L2 behind it checks it.** L1 is eight halfword banks of 2
skewed ways of 64 rows (`L1RB`=6) in distributed RAM, read at the START of the cycle at rows of the
stream's registers: bank b at its pair's -- the first pair's when b is at or after the offset, else
the pair after's, which is the first pair's plus one in L1's row bits alone, with the first pair's
skew and tag, so no carry chain stands in front of the read (it is the pair's own slot except where
those bits wrap, and the fill writes a pair after's entries where the read finds them). So the loop
is the stream's registers, the rows, the read, the hits and the end pick, the next address -- none
of the next window's rows. An L1 entry mirrors an L2 entry in the SAME way: its tag is the L2 index
bits above L1's and L2's tag, its type carries the bimodal counter, and the way the details carry is
both levels'. L2 (`BTBB`=13: 512 rows a bank and way, in the 16 RAMB36 that held 128) reads the same
rows in the same cycle, each bank its own pair's (`sa_q + 16`, exact), so its entries arrive a cycle
later, when a second end pick (`u_chk`) gives L2's view of the window: its last position, where the
stream goes on, its marks. When L2's differs from L1's in where the window ends or where the stream
goes on, the cycle after corrects: L1 is filled from L2 at that window (L2's entry into L2's way,
and every other way that matched given the window's L1 tag inverted: L1's rows are not L2's, so an
L2 eviction can leave L1's copy, which would stay L1's first match for ever, and an entry is never
empty -- zeros are tag 0, which some windows' tags are, so a way cleared to zeros matched such a
window again and the correction repeated every three cycles while fetch waited on that window's
bytes -- a livelock the boot lockstep reaches in 14 M cycles with 1-bit tags), the stream goes back to
the window's start with its history, RAS pointer and queue tail from before it, and the ring drops that window's
request and the next's (§4.1) -- the window is looked up again three cycles after it first was, and
that check agrees with L2 unless training wrote either level meanwhile (asserted, `e_conv`). A
window that differs only in its marks fills L1 and is left alone: the not-taken conditional L1
lacked goes unpredicted once, as a BTB miss does. Training writes both levels at the CTI's own
slots. A window's two queue entries are picked one-hot by the end pick's masks, a mark's own fields
only, and L2's bimodal counter is read from a register of its row during the block RAM's
clock-to-out. Out of context at 6.000 ns (FW 6, PCW 40) the predictor meets with +0.109 ns at a
queue write, its loop with more (not among the 40 worst paths, the 40th at +0.237); 6,799 LUTs
(2,832 of them distributed RAM). The one-level predictor (2K entries in block RAM, in the loop) met
with +0.204 ns out of context and missed all nine of its builds (-0.065 to -0.83 ns): its loop's
route to 16 block RAMs in two columns was 60% of 5.88 ns. On GB5's single-core SimPoints the two
levels take -0.03% cycles against that one level (L1 at 32 rows +0.21%, SQLite +9.9%; one 8K level
in the loop, which would not close timing, -0.49%, Clang -17%): L1's corrections cost what the
bigger L2 gains. The 300 M boot corrects 2,978,277 of 122,700,264 windows.

The ring marks each predicted CTI's last halfword, and each prediction enters an 8-entry queue (two
banks by position parity, one write each) in order (a window pushes up to two; the stream moves
only with two entries free): taken, the target,
the corrector details training needs, and the RAS pointer and history as they stand before the
CTI. The aligner reads the queue's first `IW` entries; an instruction ending on a marked halfword
takes the next one, and a taken one ends the bundle. A bundle pops one entry per mark it holds
and, when its last instruction is a real branch or jump predicted taken, takes it. A mark that does not fit the code is rejected and
costs a restart a cycle later, as a one-cycle freeze: taken on anything but a branch or jump (a
page straddler included), the bundle falls through and the stream restarts at its fall-through;
taken inside a 32-bit instruction, the stream restarts at the PC predicting nothing in its first
window; not taken inside a 32-bit instruction, the ring clears the mark and the head is popped.
Between marks the stream's state does not change, so the queue head's pre-state -- or the
stream's own registers, when the queue is empty -- is the state as of fetch: every instruction's
snapshot, and what every restart of the stream (a rejected mark, a freeze) goes back to.

**The history** is a path history (`smolrv64_phist.vh`): a taken control transfer shifts it one
place and folds in bits [11:1] of its last halfword's address; a not-taken branch leaves it alone.
Every instruction carries the history from before it; a redirect restores the redirecting
instruction's snapshot, stepped with its own address when it was taken, and a redirect at the head (a trap, an xret, a serializing op) restarts it at zero. Nothing is
rebuilt from resolves, which arrive out of order and include wrong-path branches (rule D14).

The 60 M boot (IW=3): 17,784,535 retires (+3.04% over the ring alone), 59,797 redirects
(63,515); the stream waits on a full ring in 39.5 M of its cycles. Out of context at 4.5 ns the
frontend reaches 214.6 MHz (196.5 with the ring alone).

| structure | size | organisation | storage |
|---|---|---|---|
| L1 BTB | 1024 entries (`L1RB`=6), 8 halfword banks x 2 skewed ways (`BTBW`=1) of 64 | 15-bit tag + 3-bit type (a conditional's counter) + 38-bit target | distributed RAM, read at the stream's registers (in the predictor's loop); one write, training's, else a correction's fill |
| L2 BTB | **8192 entries** (`BTBB`=13), 8 halfword banks x 2 skewed ways of 512 | 12-bit tag + 3-bit type + 38-bit target | block RAM, one RAMB36 per bank and way: the check's read (a cycle behind the stream) and training's write |
| YAGS corrector | 2048 entries (`YBITS`=11), 8 halfword banks | 8-bit tag + 2-bit counter | distributed RAM, read as the stream moves |
| GHR | 11 bits (`GHL`) | path history | flops |
| RAS | 8 entries (`RASB`=3) | call/return stack | flops |
| prediction queue | 8 entries (`PQB`=3) | one per mark, in order | LUTRAM or flops |

**RAS recovery.** Each instruction's predict details carry the RAS pointer from before it, before
its own push or pop. A redirect restores the pointer (`rb_rsp`, formed in `smolrv64_core`
beside the redirect target and registered with it) from the redirecting instruction itself:
a lane's restart from the mispredicting CTI's snapshot plus its own push or pop; a decode
resteer from the redirected slot's snapshot plus its push when it is a call; a redirect at
the ROB head from `rsp_r`, the pointer the retired calls and returns leave (two class bits
per ROB entry, written at dispatch and read at commit). The array itself is not restored. A
count kept at resolve cannot serve as the committed pointer: control flow resolves out of
order on its own pipe, and a wrong-path call or return resolves before its squash (rule D14).

**Training.** Every CTI trains the predictor once, mispredicted or not, and only once it is
NON-SPECULATIVE: when the ROB's irrevocable point (§6) has reached it, so everything older is done
and nothing can squash it, or it has retired. Up to `IW` (four) resolve per cycle; each enters a
32-entry queue in age order from its lane's resolve register (`lr_*`), carrying a copy of all
training reads (rule D16; the copy is a banked distributed RAM read only at the head, the
fields every entry is compared on are flops), and the queue's head trains one per cycle once
non-speculative. A CTI
arriving at a full queue is dropped (training is a hint). The queue is 32 entries: at 16, with the
store register and loads first (§8, §9), Dhrystone (`workloads/rvbench`, lockstep) dropped 8.5%
of its CTIs, the same two taken branches on every run, which never reached the BTB and
mispredicted every run (253.2 cycles a run); at 32 it drops none and runs in 222.2. A CTI younger than a pending restart
queues nothing; an entry the kill marks dead, or one past the pending restart, is discarded at
the head. A redirect discards every entry that has not retired, except the restart's own CTI at
its squash (the ROB head). Training is a read-modify-write of the tables' current counters (L2's
bimodal banks `bc`, the corrector; L1 takes the same write), never of the snapshot carried from fetch, which every
in-flight instance of a loop's branch shares. A mispredicting
conditional branch's early restart restores its own history snapshot, stepped by its own address
when it was taken. The key is the CTI's last halfword (its PC and length). Dhrystone
(`workloads/rvbench`, 5,000 runs, IW=3): 1.06 redirects per iteration, the strcmp loop exit;
1 470 958 cycles. The Linux lockstep (tiny128, IW=3) retires 44 372 101 instructions in 60 M
cycles and 229 214 494 in 300 M.

**The corrector's tag carries the PC bits its index consumes** (2026-09-10): the index is
PC[10:1] xor the history, so those PC bits are not recoverable from the slot; the tag used to
be built from PC[26:11] alone, which is 0 for every branch within the same 2 KiB, so every
slot "hit" for every such branch and a branch whose history is polluted by an unpredictable
neighbour read cold and foreign counters as its own. `workloads/brbench`: the loop back edge
next to a random branch mispredicted 66 times in 300 iterations, 50 of them this way; with
PC[10:1] folded into the tag the loop's redirects are the random branch's alone (0.73 ->
0.51 per iteration, 20.4 -> 18.3 cycles) and the drain loop's fall 0.96 -> 0.55.

**Neither array has a valid bit.** Validity *is* the tag match; a mismatched tag is a
miss. Removing them was bit-identical in simulation and moved both arrays from LUT/flop
structures into BRAM — a valid bit kept outside its array is a mux the size of the array
(rule I4). There are **no checkpoints**: this core forked `src/predictor.v` precisely to drop
them. Full OoO will need them back.

---

## 5. Rename and the physical register file

Architectural registers are a **unified 64-entry space**: 0–31 integer, 32–63 FP, matching
`decode_operands`' `{fp_bit, field}` encoding. The physical registers are not: f0–f31 rename
only into the **FP file**, two shards SH_FL and SH_FF, and x1–x31 only into the integer shards.
A physical register is `{shard[2:0], index}` (`core/smolrv64_shards.vh`): lane k's integer
shard is k, SH_FL (4) holds the FP loads' destinations and SH_FF (5) the FP ops', so an
f-register is exactly one with `shard[2]` set. A width-IW core uses lanes 0..IW-1 and the two
FP shards; rename builds free lists for those alone, and a result routed to any other shard
is asserted.

`smolrv64_rename` carries a full speculative/committed split — SMAP/RMAP for the map, and a
per-shard free list with a speculative head and a committed head. Every source reads SMAP alone
(no committed candidate beside it), so each slot looks up three readiness bits, not six. A
mispredict's walk restores SMAP entry by entry (§6). A total squash (a redirect at the ROB head,
or reset) restores it by **the sweep**: `dirty` (64 flops) names the registers whose SMAP mapping
may be uncommitted -- set by a rename and by the walk, cleared a cycle after a commit whose
register is still the newest mapping, and as swept -- and the sweep rewrites them from RMAP, the
IW lowest dirty registers each cycle from the squash's own cycle (picked, then written a cycle
later), while `stall` holds the queue's pop. A clean register maps to its committed mapping
(asserted every cycle). At a squash on the 60 M boot 6 registers are dirty (4.4 exactly; the rest
the walk restored to committed mappings), 2.4 cycles of picks, mostly under the redirect's own
freeze (`SWEEP-SIM`). Against restoring in one cycle: -0.09% retires on the 60 M boot,
Navigation +0.07% and SQLite +0.00% cycles; walking back to the head instead would cost
+1.4-2.3%. The free lists roll back by
`h := hc` in **one cycle with no walk**, because rename never writes the free-list array.
**The free-list read addresses are registers only** (2026-09-17): bank g of each list
reads at `h_hi + (g < h_lo)`, its next free entry; which slot allocates from which shard
(the instruction-class decode, and never the stall) selects among the banks' OUTPUTS. The
intra-group bypass (a source that is an earlier slot's destination) is applied at `r_prs*`
alone; the core's pending lookup indexes the MAP's candidate and masks its result with `r_byp*`.
The group is a cyclic run of slots from `r_fc` (§3.2): slot k looks at the slots k-1, k-2, ...
cyclically, nearest first, as far back as its program position, and a register two slots write
takes the younger's copy (`newer`). A group allocates one register from a lane's shard -- lane
k's integer destination is lane k's shard -- and up to `IW` from an FP shard: the k-th of a list's
allocations in program order takes its head's k-th entry, which the list's four banks already read
(from the head pointer alone, so no class decode reaches a read address). Program order, not slot
order: the committed head advances as a row retires and the walk backs a row out youngest first.

### 5.1 Sharding

The PRF has **a write address and a write enable per bank, one writer each**. Duplication buys
read ports only; sharding by *writer* is what buys write ports. The integer shards are the
lanes' (shard k is lane k's, k < IW: SH_IE, SH_IE2, SH_IE3, ...), one write register each: the lane's ALU
result and link at issue, a multiply in its reserved slot (§7), and every other integer result
-- a load's, AMO's or CSR read's (§8), an FP op's integer destination, a divide's -- through the
lane's landing buffer (§8). The F stage
writes SH_FF; its writes include the in-core FP ops (FSGNJ, FEQ/FLT/FLE, FMV both ways, FCLASS; C4b step 1(c)):
one cycle off the stage's operand registers into a one-entry result register (`icr_*`) that
lands through the FPU's own landing in any cycle the FPU is not landing; a waiting result
holds the stage. An FP op with mstatus.FS off is illegal at dispatch and traps from the SYSQ.
**A link is its lane's ALU result**, written into the lane's
shard at issue, so a jal/jalr never waits for a port and its squash waits for nothing (rule D13
is satisfied by construction). **The read ports**: each integer shard has 2 x NL + 1, nine at
four lanes -- the lanes' rs1/rs2 and the store queue's data read -- and each FP shard has four,
the store queue's data read and the FP port's three operands. A lane never names an f-register,
and the FP port never an integer register (both asserted). **An FP op with an integer source**
(`fcvt.d.l`, `fmv.d.x`: rs1 is its only source) issues in its slot's lane, which reads rs1 and
hands the op whole to the F stage: into the stage in its execute cycle, or, when the stage takes
another, into a one-op register per lane (`hx_*`). The F stage takes a waiting register first,
then a lane's op executing that cycle (`hl_vv`), then its scheduler's pick (`j_*`), each the
lowest lane first; an int-to-FP op reaches the F stage two cycles after its pick, as an FP op
does. The lane's scheduler holds every such op while its lane's register is taken or about to
be (`hx_busy`, from flops: the register, or such an op in the select register), so a lane never
executes one into a full register and nothing waits at dispatch; the ops it holds wait in the
scheduler, as any op does for an operand. On the GB5 slices (vs the FP port's integer read):
Camera -0.19%, Gaussian Blur -0.02%, Structure from Motion +0.05%, SQLite +0.05%, HTML5
-0.27%, PDF Rendering -0.01%, Navigation identical (no int-to-FP op).

| shard | entries | written by | why the size |
|---|---|---|---|
| IE, IE2, ... (one per lane) | 64 each | **its lane's write register, alone**: its ALU results and links, its multiplies, its landings (loads, AMOs, CSR reads, FP ops' integer results, divides) | > 32 (integer arch regs) each |
| FL | 64 | f0–f31 only: **the load landing, alone** (FP loads) | > 32 (FP arch regs). The shard rename runs short of on the FP slices (Camera: every `be:rename` cycle, RENAME-SIM); at 128 those stalls are gone and the cycles move 0.04% at most (Camera, Gaussian Blur, Structure from Motion, PDF Rendering): the window waits on something else instead |
| FF | 64 | f0–f31 only: **the F stage, alone** (FP results) | > 32 (FP arch regs) |

**The FP file is sharded by writer.** An FP load's f-register destination allocates from SH_FL,
any other's from SH_FF (a trapping FP op too: it writes nothing). A group holds at most one load,
so SH_FL hands out one register a cycle, and any number of FP ops, so SH_FF hands out up to `IW`. At reset
f0–f31 map to SH_FL. Each shard is written by its writer alone, the load landing (`we_ld`) or
the F stage (`we_fe`), through the address, data and wakeup broadcast it already had, so the
FP file adds no wakeup port, no write ever waits, and a read never chooses between banks. It is read by the store queue's
data read (`fsw`/`fsd`) and the FP port's three operands.

**A destination's shard is chosen by the UNIT that writes it, never by the data type.** A
CSR read and a jump's link register are integer results, but M produces them, so they take
LD. Leaving those two in IE gave that shard a second writer, and the only way to keep one
write port was to hold the ALU off whenever M was writing IE — which put the entire LSU
completion cone inside the integer scheduler's ready bits. Post-route that was the critical
path: `m_addr -> lsu -> m_done -> m_wb_ie -> u_iq_i/e_r[9][1]`, 24 logic levels, WNS
-0.383 ns at 166.67 MHz. Applying the rule was worth **+0.397 ns** and is what closed
166.67 MHz with dynamic issue. With it, `m_wb_ie` is identically zero (asserted in
`smolrv64_core`, not assumed) and the integer scheduler has no `unit_busy` term at all.

A physical register's shard is encoded in its number and never changes, so a commit's
`c_pold` is returned to **its own** shard's free list, not to `c_shard`.

Fetch stalls when a lane's shard drops below `LOWAT`=4 free, or an FP shard below 2 x `IW`, counting
the allocation of the group in the IR (smolrv64_rename `low_n`).

---

## 6. Reorder buffer

- **64 rows of a group** (256 entries at IW=4, 228 of them live at once: `ROB_CAP`, below), status only — no result values, no PC, no operands. A dispatch
  group takes a row, its instruction in lane k in column k. The lanes rotate (§3.2), so a
  row's instructions start at its **first column** `fc[row]` and run on cyclically: position p
  is column `(fc + p) % IW`. An op's **tag** is `{row, wrap, column}`, the wrap bit set when its
  column is below `fc` (it is younger than its column suggests), so tags order as numbers in
  program order and every age compare outside the ROB -- `kdead`, a distance from the head -- is
  the plain one. The storage index is `{row, column}`, the tag without its wrap bit (`rsi`), and
  indexes every per-entry array. A row has 2^⌈log2 `IW`⌉ columns (at IW=3 the fourth is never
  valid); the dispatch credit is a free row.
- **Sized by one knob**, `SMOLRV64_ROB_ROWS` (a power of two; 64 in the shipping build), **and
  capped by the seqno.** The fetch sequence number that orders the restarts (`older()`, the sign
  of a difference) is `SEQW` = 9 bits (`SMOLRV64_SEQW`), so it orders a window of 2^(SEQW−1) = 256
  ops, and `SEQ_LIVE` = FW + QD + IW + `ROB_CAP` × IW, every op from the bundle register to the ROB
  head, must fit it. The ROB holds `ROB_CAP` = min(`ROB_ROWS`, ⌊(256 − FW − QD − IW) / IW⌋) rows
  at once -- all 64 at IW=3 (`SEQ_LIVE` 217), 57 at IW=4 (254) -- and its `ready` and dispatch's
  ROB credit both take the cap. Measured at IW=4 (2026-10-24): the 28 entries the cap leaves out
  cost GB5's 77 SimPoints 0.04% of their cycles (SQLite +0.61% the most) and the 60 M boot
  0.004%, where a 10-bit seqno costs every age compare a bit. An always-on check fatals when the
  seqno order of the mispredicts and the pending restart disagrees with their order in the ROB.
  The size is measured below (*Answered: how big a ROB*).
- Column k is one array, written only by lane k. Inside, the pointers are positions `{ring
  wrap, row, p}`. Commit port p is the head row's position p: the ports read every column of
  the head's row and rotate the records into program order by `hfc`, the head row's first column
  (a flop that follows the head), and the walk's ports likewise by its row's (a flop loaded with
the walk's row: rows are written only at allocation, and nothing allocates while the walk runs). Commit runs along
  the head's row from its first uncommitted position; a row may take several cycles, and the
  head moves to the next row when the row's last valid position commits.

**Entry format — 25 bits.** `ent[]` is `{pold, noret, rd, prd}`, plus `v` and `done` as separate
bit vectors.

| field | bits | meaning |
|---|---|---|
| `pold` | `PBITS`=9 | the destination's previous mapping: the walk restores it, commit frees it |
| `noret` | 1 | commits but must not be counted (see below) |
| `rd` | 6 | architectural destination, unified numbering (0–31 int, 32–63 FP) |
| `prd` | `PBITS`=9 | physical register allocated; **0 when none** |
| | **25** | × 64 rows × `IW` columns = **6,400 bits** at IW=4 |
| `v`, `done` | 2 × 64 × 2^⌈log2 `IW`⌉ | separate flops — `v` is bulk-cleared by the flush and the kill; `done` is read only under `v`, so its allocation alone clears it |

**Three fields are deliberately absent**, each recoverable from state the design already
keeps. The ROB is sized by the *window*; the scheduler that needs execute detail is sized by
*dependency depth*, so anything derivable does not belong here.

| not stored | recovered as | why it is sound |
|---|---|---|
| `rd_v` | `\|prd` | physical register 0 is architectural x0's permanent mapping and is never freed, so it is never allocated |
| `shard` | `prd[PBITS-1:IDXB]` | a physical register's shard is the top bits of its number and never changes |
| `pold` | `rmap[c_rd]`, read at commit | `rmap` holds committed state, so in the cycle an entry commits its architectural register still maps to what that entry displaced; the commit write is what replaces it |
- Completion is by **tag**, allocated at rename and carried with the op. Lane k completes
  only column k: every op a lane runs or lands is its slot's (an integer register is renamed
  in its slot's lane), so the lane's port carries its column as a constant (asserted) and the
  other columns never compare it. The shared ports -- M's (with the landing loads), the FP
  landing, the store queue's commit, the MD stage and a mispredict's squash -- complete any
  column.
- Commit reads `done`, a register: an op completes in one cycle and commits in a later one.
  The one exception is the head completing in a flush cycle (`h_fin`: a mispredict at its
  squash, M's or the SYSQ's redirecting op), which commits in that cycle, before the flush.
  No completion cone (the lanes' branch compare, `m_addr -> dTLB -> lsu_done`, `csr_file`)
  reaches retire, the free lists or the rename stall. With a same-cycle forward from every
  port the lanes build stood at -0.768 ns over 10,991 endpoints, -0.155 with the deepest
  ports cut. The cost is ROB and SQ occupancy: -1.48% in the 60 M boot (ROB 64 recovers
  0.24 points and moves the stalls into the SQ, sized in §14).
- The irrevocable pointer reads the store queue's completion port (`IRR_FWD`) in its own
  cycle as well as `done`: without it a store leaves the SQ a cycle later, and the boot loses
  another 2.8% to `be:sq-full`.
- A flush is **pointer-only**: it leaves the head, the tail and `irr` at the next row boundary
  (the head's own row if nothing of it has committed). A mispredict's restart is not a flush
  (below).
- `noret` exists because an injected `OP_IRQ` can commit with its trap not firing, and
  `minstret` must not count an instruction that architecturally does not exist.

**The irrevocable pointer (`irr`, 2026-09-04).** A second pointer walks forward from the
head, a ROB row per cycle, over entries that are done (or completing on the SQ's port) and
stops at the first that is not. In this core every op that can restart the machine -- a
mispredicted branch, a trap, a system op, `fence.i`, the interrupt pseudo-op -- waits in M
for the ROB head and is done only once it has completed there, so an entry that is done can
no longer restart, and everything older than the pointer is settled. A store is COMMITTED
when the pointer reaches its slot: `smolrv64_core` fires `sq_k_take` for the store queue's first
uncommitted entry when its ROB index equals `irr_idx` and the entry has address and data;
that commit is the write that sets the store's `done`, the head retires it like any other
op, and the queue drains it to the cache behind retirement (§8). The pointer also stops at a
load flagged for replay (§8), done or not: nothing younger -- a store -- may become irrevocable
ahead of the replay that squashes it. The head therefore no
longer sits on every store for the ~6 cycles the cache takes. The pointer is conservative
on purpose: it stops at ANY not-done entry, a load in flight or an FP op included, although
neither can restart -- letting a store commit past an older load that has not read yet
needs a write-after-read check in the load queue (the store's drain must not pass the
load's read), and that is the next step, measured separately. It steps like the head, along a row and then to the next.
On a flush it returns with the head (above); `(irr - head) > (tail - head)` is fatal. Cost:
one `{wrap, row, p}` pointer, one `done` lookup, one tag compare in `smolrv64_core`.

Simulation-only side arrays (`cs_pc`, `cs_insn`, `cs_val`, `cs_mkind`, `cs_mpa`) hold the
cosim payload per slot so the ROB stays status-only in hardware.

**Restart at resolve (lanes 5.4e).** A mispredict restarts fetch in the cycle after it resolves
(`fr_set`, the oldest restart winning) and freezes dispatch (`fr_v`); the backend recovers
without waiting for the branch to reach the head:

- **The walk.** Each entry keeps `pold`, the mapping its rename displaced. From the cycle after
  the resolve a cursor walks from the tail back to the branch, a row a cycle, on a second read
  port of each column: every dead entry with a destination writes `SMAP[rd] := pold` through
  its column's map copy (the oldest column winning in a row, `lv` set) and steps its shard's
  speculative free-list head back over its register. An older mispredict resolving meanwhile
  moves the target back and the cursor goes on.
- **The kill.** The cycle after the resolve the ROB turns the branch into a position, from
  flops: `kh`, the head row's first tag when the walk began, and `kb`, the branch's distance from
  it. An op is dead while the kill holds when its tag is farther from `kh` than `kb` -- the rows
  after the branch's and its row's later tags -- which each structure tests for its own entries
  with one subtract and compare (`core/smolrv64_kill.vh`), not by indexing a 256-entry vector;
  the vector (`kd`, asserted equal) serves only the per-entry clears at the release. Every
  structure that holds an op clears its dead ones each cycle the kill holds: the schedulers,
  dispatch stages, multiplies, landing buffers (a dead entry drains as a bubble), F, MD (the
  divider aborts), the SYSQ, M and the head-op register, the arrival flags, the walker lock,
  the resolve registers and the training queue; an entry filled in a cycle the kill holds is
  tested by the op it takes, not the one it held (the FP queue fills from the F stage then, rule
  D19). The load and store queues drop their dead
  entries, their youngest, and step their tails back; the LSU drops a dead load's response by
  tag. fpnew has no per-op kill: a table of its ops in flight (by ROB index) holds the release
  until no dead one is left.
- **The release** (`fr_rel`): once the walk has reached the branch, the kill has held for
  `KREL` = 6 cycles (an op in transit dies at its next register) and the FPU holds no dead op,
  the dead entries leave the ROB, the tail returns to the row after the branch's, the branch
  completes (before it, it is not done, so `irr` cannot reach a dead store) and dispatch
  resumes, with the right path already in the decoupling queue. A branch that reaches the ROB
  head first takes the squash there instead (`cf_red_fire`, the full flush and rename's sweep),
  whichever comes first; in the 60 M boot about one restart in five ends at its release.

Checks: at a squash at the head after the walk has ended, every register's speculative mapping
equals its committed one and every shard's speculative head its committed head; whenever no
sweep is in flight, every clean register's speculative mapping equals its committed one; nothing
renames or walks while the sweep runs, and it ends within a pick per group; at one after
the kill has held four cycles, no live op is left in the backend; at a release, no dead op is
(all asserted, `WALK-SIM` counts them).

### 6.1 Scheduler (`smolrv64_iq`) — LIVE

It selects what executes. See §2.1 for what may reorder and why the usual OoO machinery
is not needed alongside it.

**There is a scheduler per lane, four, and the FP scheduler, and none stores age.**

**SCHEDULER-SIZE TIMING NUMBERS IN THIS DOCUMENT ARE SINGLE SAMPLES AND DO NOT
DISTINGUISH THE CONFIGURATIONS.** Rule I2: four placer directives over IDENTICAL RTL span
**81 ps straddling zero**; treat one build as a sample, not a result. Three NF builds:

| | `probe_clk` | worst family |
|---|---:|---|
| `NF`=8 | -0.012 | frontend PC increment, 24 levels, 6x CARRY8 |
| `NF`=7 | **-0.082** | `u_csr/mhpmcounter[12]` carry, 32 levels, 10x CARRY8 |
| `NF`=6 | **-0.210** | `fpnew` `i_fpnew_cast_multi` internal pipeline |
| `NF`=5 | **+0.038** | PASSES — the shipped size until 2026-08-28; 8 since (47e1d26a), closing at +0.001–0.002 ns on every gated build |
| `NF`=4 | +0.050 | -- |

`NF`=7 is 70 ps WORSE than `NF`=8, from REMOVING an entry. That is not a logic effect. All
three sit inside a 132 ps range, i.e. inside the I2 envelope, and each build fails on a
DIFFERENT family. **Dialling the scheduler down one entry at a time cannot work: the step is
far below the noise.** Any NF judgement needs at least two placer directives per config.

Two marginal families are now visible and both are worth attacking on their own merits,
independent of NF:

- ~~**Frontend PC increment** -> BTB address: 24 levels, 6x CARRY8.~~ **DIAGNOSED AND
  FIXED — it was never the increment.** See "The frontend path" below.
- **`mhpmcounter` carry**: 32 levels, 10x CARRY8. **DIAGNOSED AND FIXED.** The earlier
  `hpm_ev_q` fix registered the event bus and closed one route out of `lsu_done`; it left
  the other alive by explicitly exempting `retire_cnt` as "architectural". That is true of
  `minstret` and false of `mhpmcounterN`. `retire` is `rob_c_valid`, and `smolrv64_rob`'s
  `head_done` write-forwarded across every writeback port, so:

      m_addr -> lsu_done -> rob w_hits -> retire -> retire_cnt
             -> hpm_inc's INSTRET arm -> 13 event muxes -> 13x 64-bit carry chain

  Zihpm counters are permitted **arbitrary** read latency — a counter may reflect state N
  cycles ago — so `csr_file` now takes a second `hpm_retire_cnt` port and `smolrv64_core`
  feeds it a registered copy (`hpm_ret_q`). **`minstret` takes the delayed copy too since
  2026-09-17**: a head-gated op (CSR, trap, fence.i, system) completes only in its SECOND
  cycle at the ROB head (`m_head_q`), nothing retires while it waits, so the one-cycle lag
  is invisible to any CSR read; a `csrw minstret` drops the writer's own retirement a cycle
  later (`minstret_wr_q`). The live count was `m_addr -> dTLB -> lsu_done -> w_hits ->
  retire -> minstret`, 28 levels, the deepest path in the IW=3 build. Every input
  to `hpm_inc` is now a flop, so the mux and the adder start at the top of the cycle.
  (The retired sharded core passed its live count to both ports and was bit-identical.)

  If the mux and the 64-bit adder still bind *together*, the same licence permits
  splitting them across cycles (register `hpm_inc` per counter, 13x6 flops). Not done:
  unmeasured, and it is 78 flops for a path that may already fit.

**Standing rule (2026-09-05): integer performance is never traded for FP.** When a build
misses timing or an integer change needs slack, `NF` is the first thing to give -- back to 5,
which closed with +0.038 ns -- never the integer scheduler, the ALU path or the frontend.

**NF=8 DID NOT CLOSE TIMING at 166.67 MHz on 2026-08-27, and the reason was not the scheduler.**

| | `probe_clk` |
|---|---:|
| `NF`=4 | **+0.050 ns** (closes) |
| `NF`=8 | **-0.012 ns** (FAILS) |

62 ps, landing 12 ps under. But the failing paths are in the FRONTEND:

| slack | path | levels |
|---|---|---|
| -0.012 | `fe/u_fetch/pc_q_reg[17]` -> `pc_q_reg[12]` | 24, **6x CARRY8** |
| -0.008 | `fe/u_fetch/pc_q_reg[12]` -> `u_bp/btb_reg/ADDRARDADDR[9]` | 23, 6x CARRY8 |
| -0.006 | `m_addr_reg[14]` -> `i_ps3_reg[1]/CE` | scheduler-related |

The PC increment and its path into the BTB address were already marginal; NF=8's extra area
tipped them. Only the third path is the scheduler's (`i_ps3` is the `NSRC`=3 third source,
whose clock enable sits in M's completion cone). **A faster scheduler would not fix this.**
Three ways out, in order of what they cost:

1. **Keep `NF`=5** -- the largest that closes (+0.038 ns), and no measured workload can see
   the difference against 8.
2. **Attack the frontend PC path** to buy headroom, then `NF`=8 fits. This is the only
   option that makes the policy affordable rather than abandoning it. **Done** -- see below.
3. ~~Scale the frequency back.~~ **HARD RULE: frequency is never scaled back except for a
   diagnostic run.** 166.67 MHz is a floor, not a variable. When a change does not fit, the
   change gives way or the path it broke gets fixed -- the clock does not.

Standing procedure when a scheduler size does not fit: **dial it down one entry at a time
until it passes**, rather than jumping to a known-good size. 8, 7 and 6 all failed, each on
a different family; `NF`=5 closed at +0.038.

#### The frontend path (2026-08-28)

The name was wrong and so was the diagnosis. Nothing in it is a PC increment. On the routed
`NF`=5 checkpoint the two frontend paths are:

| slack | source -> destination | levels |
|---|---|---|
| +0.062 | `fe/u_fetch/strad_reg` -> `u_bp/btb_reg/ADDRARDADDR[12]` | 22, 5x CARRY8 |
| +0.075 | `fe/u_fetch/strad_reg` -> `u_bp/ycorr_reg_bram_0/ADDRBWRADDR[9]` | -- |

and the first expands to

    strad -> imem_addr -> u_immu/req_match -> u_icache/fb_w0 -> I$ data
          -> u_bp/ras_ptr -> p_ret -> bp_tgt -> btb ADDRARDADDR[12]

**5.521 ns of 6.000, and 3.848 ns of that (69.7%) is route.** The CARRY8s are the iMMU's
TLB compare and the I$'s fetch-buffer address compare, not an adder. `apc` is supposed to
be register-only precisely so this cannot happen; it was not, because `cti_ok` (the
aligner's `br_term`) was ANDed into `hit` at the top of `smolrv64_predictor`'s predict cone and
so reached both `pred_v` and, through `p_ret`, `pred_tgt` — the two things `apc` selects
on. Rule I6.

Fixed by splitting the cone: `tag_hit`/`apred_v`/`pred_tgt` from registers only, `pred_v =
apred_v & cti_ok` for the real PC. `pred_tgt` is bit-identical where it is read, since
`pred_v` implies `cti_ok`. Verified: `lint: clean`, 240/240, 215/215, Linux cosim clean over
40 M cycles at **10,444,328 retires against a 10,444,329 baseline — 1 in 10.4 million.**
The `probe_clk` effect is **not yet built and therefore not yet known**; rule I2 applies
when it is.

**Minimum 8 entries for any scheduler is policy.** Measurement does not currently justify it
for `u_iq_f` -- blurbench is 29.44 cycles/pixel and saxpybench 22.08 at both 4 and 8, and
`NF`=8 costs 22 ps of `probe_clk` -- but both of those workloads are limited elsewhere (FP
latency, and in-order memory issue), so neither can see a deeper FP queue. Recorded as a
standing rule rather than a measured optimum, and worth re-measuring once the store buffer
moves the wall.

| | `u_iq_i`, `u_iq_i2`, `u_iq_i3` | `u_iq_f` |
|---|---|---|
| entries (`NENT`) | 10 each | 8 |
| sources (`NSRC`) | 2 | 3 |
| holds | slot A's, B's, C's ALU ops, multiplies, divides, branches, jumps, loads, stores and head ops; slot A's system ops | FP arithmetic |
| ordering | **reorders freely** | **reorders freely** |
| unit | completes at issue -- a CTI the next cycle, from registers, on a port of its own (a mispredict at its squash) -- writes IE, IE2, IE3; a load, store or head op hands its VA to M (§8), a divide its operands to the MD stage, a system op its record and rs1 to the SYSQ | stage F |
| `unit_busy` | the lane's multiply slot; lane A's, a system op at the ROB head | `j_v & ~j_adv`: the FP select register is full and not draining |
| `ag_only` | the lane's landing buffer holds an entry: only address generators issue (§8) | 0 |

**One scheduler per unit is what makes four safe.** Three schedulers feeding ONE execute
stage deadlock (`Area-Efficient-Scalar-OoO.md` 12.2): an op reaches the shared stage, finds
it must be ROB head to retire, and an older op in a different scheduler cannot issue to
free it. Stage F removes the condition rather than arbitrating it -- **an FP arith op never
enters M**. It cannot head-block because it cannot trap: an encoding `decode_fp` does not
know (an unlisted funct5, the H or Q format, an FP load or store other than W or D) and any FP
op under `mstatus.FS=Off` is illegal at decode, so a SYS-class op that traps alone in slot A
(`core/directed/fpill.S`). A write to FS redirects and refetches, so the value read at decode
is what every in-flight FP op retires under.

**Only `u_iq_l` is in-order, and it should not be at all.** In-order ISSUE is not what
memory ordering requires (`Area-Efficient-Scalar-OoO.md` 11.1): an address calculation is
ordinary ALU work, so loads and stores should issue as soon as their address operands are
ready, in any order, with the ordering living in a **load queue** (program order in, out of
order out) and a **store buffer** (indexed by store-seqno, committing when data is ready).

`u_iq_l` is in-order because neither structure exists yet, and the cost falls on far more
than memory: **mul/div, CSR, branches and jumps are all serialized for a constraint only
loads and stores ever had.** They cannot be split into a fourth scheduler while they share
M -- two schedulers feeding one execute stage is the 12.2 deadlock -- so the order of work
is P2 (delete `head_block`), then the load queue and store buffer, then `u_iq_l` reorders
and no scheduler is in-order.

### Out-of-order FP: why the flags are safe and the rounding mode is not

Reordering FP is safe for the exception FLAGS because they **wait for commit**: a result's
flags land by its op's ROB slot (`rob_ffr`, a distributed RAM the landing result writes, with
`rob_ffv`, a bit per slot cleared when the slot is allocated and set when the result lands), and
the flags of the ops retiring in a cycle are ORed into `fcsr[4:0]`. The order results land in cannot
change the answer (the OR accumulates), and a squashed op's flags never reach `fcsr` (its
slot never commits). Completion and commit may fall in one cycle, so the landing result
bypasses the slot.

The **rounding mode is not like that**. A dynamic-rm op (`rnd == 3'b111`) reads `csr_frm`
when stage F hands it to the unit, so **an frm change must be an FP barrier** -- it must
neither overtake nor be overtaken by an FP op in flight.

A CSR op fires from the SYSQ at the ROB head, so every older FP op has retired and none sees a
new frm. Younger work is not held back (a CSR op does not drain, §3), so a younger FP op may
already have read the old frm: a write to `frm` or `fcsr` therefore refetches everything after
it (`csr_file`'s `do_ctxw`, the fall-through redirect FS and the translation bits already use),
and the refetched ops read the new mode. A write to `fflags` refetches nothing: a younger op's
flags are ORed in when it retires, after the write.

`u_iq_f` reorders, and that is the point rather than a detail. `workloads/blurbench`, taken
from a hardware trace of GB5 Gaussian Blur, is a 4-deep serial `fadds` chain whose taps are
independent; in-order issue held it at exactly its critical path (39.50 cycles/pixel against
5 dependence levels x 8 cycles) because the next iteration's multiplies could not start
until this one's adds had issued. Reordering: **29.44 cycles/pixel, -25%**.

**Entry format — `2 + NSRC×PBITS` bits.** Scheduling state only; no age, nothing quadratic.

| field | bits | meaning |
|---|---|---|
| `v` | 1 | entry live |
| `e_rob` | `ROBB`=7 | ROB entry `{row, column}`, carried to completion |
| `e_ps[NSRC]` | `NSRC` × `PBITS`=9 | source **physical** registers |
| `e_r[NSRC]` | `NSRC` | per-source ready bits |

`NSRC` is a parameter precisely so the integer scheduler does not pay for the third operand
only `fmadd` has: 20 bits/entry against 29.

- **Wakeup**: every PRF write broadcasts its destination on one of `NWB` = IW + 2 ports: each
  lane's port (its ALU and multiply results, and what its landing buffer drains: a waiting
  landing or the SYSQ's result) and the two streams' (the LD stream -- a landing load or M's
  result -- and the FE stream -- the F stage's landing or a divide -- each an FP shard's
  register or an integer one straight through a lane's write register). The FP queue, the
  pending table and the store queue watch all of them; an integer lane's queue watches the
  lane ports and, of the two streams, only the writes bypassed into a lane's write register
  (`by0`, `by2`) -- the streams' FP-shard writes can never name one of its sources, and they
  carry M's and the FPU's live completion cones (rule I12). A lane's port selects only on
  registers and the SYSQ's fire, so no completion cone selects a lane's broadcast address.
  A source not yet ready is also compared against the live ports **in its dispatch cycle**,
  because a producer broadcasts exactly once and would otherwise be missed forever.
- **The speculative wake** (`SPEC`, the lanes' schedulers): one more port, from flops, that only
  an address generator's address source matches (`e_ag`, a load or store, written at dispatch). An
  entry picked on it alone keeps its slot, issued and no candidate (`e_sp`), until the next
  cycle's verdict frees it or makes it a candidate again (§8).
- **Select**: the scheduler's **oldest live entry** when it is ready, else **fixed priority**,
  lowest entry index first (`OLDEST`, the lanes' and the FP scheduler). Which entry is oldest
  comes from an order matrix of flops written at dispatch (`od`: a new op is younger than every
  live one; no compare) and is registered a cycle ahead, so the pick reads an index and one ready
  bit beside its priority encoder; out of context the lane scheduler times at +1.609 ns against
  +1.500 without it. A lowest-index pick alone hands a freed low slot's young op priority over
  every older one, and an old op on the critical chain can sit ready for many cycles: the oldest
  first takes GB5's 77 SimPoints -0.56% (every subtest), the 60 M boot +1.25% retires, pi -4.0%,
  CoreMark -1.6% (4645a122); the ideal, oldest READY first, -0.94%. Starvation is impossible
  because an entry is only freed by issuing.
- **Wake-at-select**, not wake-at-writeback (`FIXEDL`): a fixed-latency producer broadcasts
  its destination in the cycle it is *selected*, so a dependent issues the very next cycle.
  Back-to-back dependent issue is non-negotiable and is check 7 of the module TB.
  **As a dependency matrix since 2026-09-17** (Henry Wong's form, for the intra-queue wake
  only): `dep[k][q][j]` -- entry k's source q is produced by entry j -- is written at
  dispatch by comparing the new entry's sources against every live destination (register
  against register) and read as one column the cycle AFTER an issue, from the registered issue
  (`sq_v`, `sq_oh`): a dependent is ready in that cycle either way, so the column is one more
  source of readiness beside the writeback compares (`swk`), and it is cleared then. The pick's
  index reaches neither `e_r` nor `dep`: a landing's wake -> ready -> pick -> the column -> `e_r` was
  one cycle's chain, 145 failing endpoints of a build (2026-10-29); cycle-identical (the 300 M
  boot retires the same). The form before the matrix read `e_prd` at the selected index (a
  LUTRAM read the select decides, fan-out 248) and compared that tag against every source,
  the tail of every path into `e_r`. `e_prd` is flops now. Results from other units still
  arrive by tag on `wb_v`/`wb_preg`. An always-on check compares the matrix with the tag
  compare on every select. The two ALU schedulers also take `REGRDY`=1: `d_ready` is a
  register ("at least two entries were free last cycle"), so the frontend's dispatch
  decision does not start at `a_ent -> held -> freem`.
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
  `smolrv64_exec` recomputes (`result`, `addr`, `target`, `taken`, `redirect`). `prd` is stored
  **gated by `rd_v`**, matching the ROB's "0 means writes nothing" convention — which is
  where it differs from the ungated `m_prd`.
- Packed and unpacked with the **same concatenation**, so a width or ordering error is a
  lint failure. And checked at runtime: dispatch happens in the cycle an instruction enters
  M and the scheduler cannot offer it before the next cycle, so at issue the payload holds
  exactly what M holds — `pc`, `insn`, `imm`, `rd`, `prd` are compared every cycle.

Gates: `core/run-iq-tb.sh` (seconds, 13 checks) plus the payload check above, which
runs inside every 240-test and cosim run.

---

## 7. Execution units

| unit | latency | outstanding | blocks M? | writes |
|---|---|---|---|---|
| each lane's ALU (slot k's ALU ops) | 1 cycle (at issue) | — | no | the lane's shard (k) |
| each lane's multiplier (`mul3`, slot k's multiplies) | 3 cycles: wake at T+2, written at T+3 | — | no (it reserves its lane's T+2 slot) | the lane's shard |
| branch, `jal`/`jalr` (its slot's lane) | 1 cycle (at issue) | — | no | the lane's shard (the link) |
| CSR | 1 cycle at the ROB head; younger work runs meanwhile | 1 | yes | lane A's shard, through its landing slot |
| div (`divider`) | 3 + ceil(q/2) cycles, q the quotient's significant bits (3-35): radix 4, iterations limited to q | 2 (the MD stage and its queue register) | **never enters M** (MD stage, §7.x) | its destination's lane's shard, through the lane's landing buffer |
| FPU (CVFPU) | 6 cycles (§7.1) | **4** | **never enters M** (stage F) | SH_FF; an integer result through its lane's landing buffer |
| LSU load | see §8 | 16 fast (by load-queue tag) + 1 slow | **no** | its destination's lane's shard (straight through or by the landing buffer); SH_FL for an FP load |
| LSU store / AMO | see §8 | 1 | yes | — |

Only loads, FP, multiplies and divides are non-blocking. Stores and AMOs still hold M: a
store has no destination register so a scoreboard slot buys it nothing.

### 7.1 FPU

`fp_unit` drives `fpnew_top` directly (`u_fpu`: `NFLIGHT=4`, `PIPE_REGS=5`).

- `PIPE_REGS`=5, `DISTRIBUTED`, ADDMUL/DIVSQRT/CONV `MERGED`, NONCOMP `PARALLEL`,
  `DivSqrtSel = THMULTI`.
- **Up to four operations in flight**, one accepted per cycle. Results return by tag, not
  necessarily in issue order (fpnew's op groups have different latencies).
- **NONCOMP (min/max, sign-inject, compare, classify) and parts of CONV return
  combinationally** — `out_valid` in the same cycle as `in_valid`, `PipeRegs` notwithstanding.
- `iss_ready` is **registers-only**: `~req_v_q & (in flight != NFLIGHT) & ~out_blk`. fpnew's
  `in_ready_o` is combinational in `in_valid_i`, and the issue decision reads `iss_ready`, so
  exposing it would close a loop through the scheduler. The request register holds an op only in
  the cycle fpnew declines it.
- **Dependent latency is the pipe depth plus the hand-over:** each op of a dependent chain waits
  `PIPE_REGS` stages plus the issue and result hand-over. That bounds a reduction such as GB5
  Machine Learning's SGEMM inner loop, whose `fadd.s` into one accumulator carries the loop.

### 7.2 FP scoreboard

M completes an FP op when the unit **accepts** it. The result is captured into a holding
register (`fb_val`) and written to the PRF when the single write port is free — a landing
load wins, FP waits. Safe without a ROB walk because an FP op cannot trap (it reports via
`fflags`) and anything older that could trap is head-gated.

An FPU result and an in-core compare never land in the same cycle (the compare waits,
`icr_land`), so each landing result writes its own op's `fflags` into that op's ROB slot.
Every integer result of the F port -- an in-core compare, FCLASS or FMV.X, and the FPU's own
FCVT to an integer register -- leaves through that one result register (`icr`), and it lands
only while its lane's landing buffer has room (`lb_fe_room`, §8): the FE stream yields to
the LD stream there. The FPU cannot hold a result, so the register is kept free for it: an
FPU op with an integer destination starts only while no other is in flight and the register
is empty, and no in-core op takes the register while one is in flight.

---

### 7.y The SYSQ: system ops fire at the ROB head from flops (C3 step 3, 2026-09-18)

A CSR or system op (SYSTEM opcode: `csr*`, `ecall`/`ebreak`/`xret`/`wfi`/`sfence.vma`, the
irqop pseudo-op) is dispatch class S and goes alone, so in slot A: it issues in lane A, which
hands the SYSQ its record and rs1 in its execute cycle (`sy_ld`); in the lane it is a late op
that does nothing else (no multiply, divide, address or branch resolution, whatever its bits
look like: a fetch fault's instruction is 0, which reads as a load), and its FP sources (an FP op
trapping at FS off) are no sources to the lane. At most one is in flight: every one but a CSR op
is serialising at dispatch (`ser_block` drains the ROB and the store queue before it and lets
nothing dispatch behind it), and a CSR op dispatches without draining but only once the CSR op
before it has fired (`csr_infl`), so the SYSQ is free when lane A hands one over -- or holds a
load's or store's fault or replay, which the SYSQ takes at the ROB head with no credit: the system
op is younger, dies at that record's redirect, and is not loaded over it. The SYSQ is one
register `{rob, prd, rd_v, is_csr, func,
addr, src (rs1 from lane A, or zimm), pc, seq, insn}` and its fire is from
flops: `sy_fire = sy_at_head & sy_lane_ok & (~sy_instret | sy_head_q)`, where `sy_at_head` is
computed a cycle ahead from the SYSQ's and the ROB's next state, `sy_at_head <= sy_v_n &
(sy_rob_n == rob_head_n_idx)`, so it is set in the cycle the op is the head, as a live compare
would be (asserted equal), `sy_lane_ok` lane A's slot free (`lb_free[0] & ~lb_any[0]`: no
landing waits for it), and the instret read's second cycle at head (M1b) kept. Every "at the
ROB head" decision in the core is a flop (rule I12): M's `m_at_head` and the squash's
`fr_hd_q` (behind `cf_red_fire`) are computed the same way, cycle-exact; the load queue's
per-entry `hd`, which only a device load waits on, compares the head as it stands, a cycle
late. The head register's fan-out ends at those flops instead of running through the queue's
candidate pick, the LSU's start and M's completion into every scheduler's wake and hold. `csr_file`'s `raddr` and `upd_*` are the
register's flops (step 2's lesson: a LUTRAM read in front of `csr_file`'s combinational redirect
cost IW=3 its closure). The read's value goes to lane A's write slot (§8) and the completion to
the ROB through lane A's port (M's op, younger then, waits while the SYSQ's op is the ROB head,
a plain load or store excepted: §3.3); a trap through `c_kill`; a redirect through
the one redirect gate (`redirect`, `fe_red_pulse`, `fe_red_tgt/seq`, `redirect_is_trap`).

The SYSQ also takes FENCE and FENCE.I and every instruction that traps at dispatch (a fetch fault,
an illegal instruction) -- `is_sysq()`, one definition for dispatch and lane A (C4b step 1). All
serialise. A trap from dispatch drives `csr_file`'s `xtrap_*` inputs from the register's
`{xt, xcause, xtval}` flops and is its own redirect (`csr_file` leaves `xtrap` out of
`redir_valid`; an always-on check asserts that every trap presented to it redirects). FENCE
completes at its fire; FENCE.I pulses `ifence` and redirects to its PC + 4. M holds no system
op, fence or dispatch-time trap (all asserted); it keeps memory ops, AMOs, CBOs and data faults. The `xtq_*` shadow checks both M's trap payload and the SYSQ's.

### 7.x Multiplies and divides in the lanes (lanes 5.2c, 2026-10-04; divides 5.4f, 2026-10-14)

**A multiply runs in its slot's lane** (`smolrv64_gclass`: class I). `mul3` starts from the
lane's forwarded operands in the execute cycle T and the lane reserves its slot at T+2 for it:
the lane's scheduler selects nothing at T+1 (`m*_1` is its `unit_busy`, a register), so nothing
else executes at T+2. That slot carries the multiply's wake (its consumers execute at T+3 off
the lane's write register), its ROB completion, its value on the lane's snoop bus (`wb_ie*`,
the store queue's data), and its result (`mul3`'s stage B, `pre_result`) into the lane's write
register at the edge, which the PRF writes at T+3. A multiply is never in a scheduler's
wake-at-select matrix (`e_long`): its dependents wake at its broadcast. The lane takes one
multiply a cycle into `mul3` (it is fully pipelined), but each multiply takes two of the lane's
execute slots, its own and the reserved one, so a lane sustains one every two cycles. Invariants:
nothing executes in a lane's reserved slot, and the multiply is at `mul3`'s stage B in it.

**A divide** is class M at dispatch (`d_cls_m`) and issues in its slot's lane like a multiply:
the lane reads both operands in its execute cycle and hands them, with the op's ROB index and
destination, to the one MD stage (`l_dgo`, `iss_md`). The divider (3 to 35 cycles) starts the
cycle after, from those registered operands. A second divide waits in the stage's queue register
(`mq_*`) with its operands, so a divide and a remainder of the same operands do not hold dispatch
for each other: a dispatch credit (`cr_div`, `dt_*`) keeps at most two divides in flight, each
released at its writeback or its death, and a group holds at most one divide. The result is latched on the divider's done pulse
(`md_pend`, `md_res_q`) and leaves by the stage's own tag when the FPU is not writing (`md_wr`),
into its lane's write slot like an FP op's integer result (§8). `abort(redirect)` and the flush
arm last keep it sound while redirects fire at the ROB head. Invariants: no third divide in
flight, none handed to a full queue, at most two lanes hand one over in a cycle, an issue never
starts a busy divider; the writeback's clear precedes the issue's set in the stage register.

## 8. Load/store unit and MMU

- **Address generation in the lanes; M is the fill stage (lanes 5.2d-a).** A plain load or
  store (integer or FP) is class I: it issues in its slot's lane when its address operand is
  ready (a store names no rs2 there), and the lane's adder generates the VA. The lane writes no
  register, wakes nothing and completes nothing for it (`lane_late`, which also keeps it out of
  the wake-at-select matrix). The VA lands in arrival registers indexed by the op's own LQ or SQ
  entry (`la_*`, `sa_*`), or goes straight into M from the lane's adder when M takes it that
  cycle; the rest of the op is a record written at dispatch (`spl_ld`, `spl_st`: the payload and
  the ROB slot). **M fills in program order**: it takes the oldest load or store whose address
  has not arrived (`uf_idx`/`uf_seq` from each queue, a sequence compare between the two) once
  its VA is in, or the head op when nothing is unfilled, which the credits make always. M fills
  in the order addresses arrive, oldest first among those waiting (each queue's `pk_*`: the first
  arrived-and-unfilled entry in program order, read from the tail or head), so a load whose address
  is ready is not held up by an older op whose address is not. An address in a lane's adder goes
  straight into M when none is waiting. M takes a load or store every cycle. Everything M did for a load or store it still does: the dTLB lookup, the
  LQ/SQ fill, and a load's start in that same cycle (below).
- **A store's data comes from the store queue (5.2d-b step 2).** A value already in a register
  file at allocation (`d_rdy`) is fetched by the SQ's data read: it registers its oldest entry
  with an address still waiting for one, and reads that preg on M's port `ra2` (which reads the
  integer and FP files) the next cycle. A value produced later comes from the snoop, armed at
  allocation; the two never meet (asserted). A group's stores take the entries from the tail on,
  in program order, so their records bank by entry (`spl_st`, a `smolrv64_bankq`) and each
  arms its own snoop. `ra2` is the SQ's alone. **One entry's bytes land a cycle**, so the data array is distributed RAM with one write (§10.1): the data read's when one
  lands, else the oldest entry a writeback names; an entry the landing leaves out keeps `dv` clear
  and is fetched by the data read, its value being in a register file by then. 60 M lockstep boot:
  9,181,990 landings and 667,265 entries left out, 46,498,426 -> 46,473,281 retires (-0.05%); 300 M:
  248,383,137 -> 247,626,564 (-0.30%); GB5 slices (§11.2) within 0.11% either way. Two placed builds
  of 2026-10-13 that differ by this and the forward queue's ring: 190,566 -> 182,820 LUTs, 74,591 ->
  72,241 flops.
- **A result lands in its lane's write slot (5.2d-c).** An integer load's, AMO's or CSR read's
  destination is in its slot's lane shard; SH_LD has no writer and no bank. The value takes the
  lane's write register in a cycle the lane leaves it free (nothing executing that completes at
  issue, no multiply in its slot; the lane's "completes later" bit is read with its tags at the
  pick, so this is a function of flops). A landing load goes straight through when the lane is
  free and nothing waits there (and, for lane A, no system op is the ROB head: the SYSQ's fire
  stays out of the streams' wake), so its latency is unchanged: it broadcasts (wake, pending clear, the store queue's snoop) and
  completes on its stream's own port, and only its value enters the lane's write register.
  Otherwise -- and always for M's own result (an AMO, LR or SC: the LSU's live completion) -- it
  waits in the lane's 4-entry landing buffer, which holds the lane's select to its address
  generators (`ag_only`, a flop term in each entry's valid, like `hx_busy`) until it drains through
  the lane's port, where it broadcasts and completes: an address generator executes without the
  port, so the drain still comes within two cycles and the buffer's bound stands. Holding every op
  instead cost CoreMark 2.84% and GB5's 77 SimPoints 1.03% of the single-core cycles (integer
  0.8-3.6%): a critical load's address generator sat ready in an idle lane behind another load's
  landing, which loads dispatched a row a group make common (CoreMark's list walk: 0.5 cycles an
  iteration of its next-pointer chain). A waiting entry is
  never a retired op's and a redirect empties the buffers. The SYSQ never
  waits: a system op goes alone (lane A), lane A holds its select while one is the ROB head, and
  the SYSQ fires only into a free lane A, so a redirecting op writes and completes in its fire
  cycle. FP loads land on the LD port in SH_FL. An address generator waiting
  on a load may issue on its D$ lookup instead (below); every other consumer wakes at the landing.
- **So does an FP op's or divide's integer result (5.2e).** The FE stream (the F stage's
  landing or the MD stage's divide, `md_wr`) writes an f-register into SH_FF; an integer
  destination (`few_int`, by its shard) takes its lane's landing buffer the same way, behind
  the buffer's own entries, the SYSQ and the load landing, and completes when it drains.
  SH_FE has no writer and no bank. A cycle can enqueue a load's and an FE result in the same
  lane (two enqueue slots), but the FE stream yields to the buffer's room: an integer result
  enters a lane's buffer only while it holds at most one entry (`lb_fe_room`, a flop from the
  next occupancy), which bounds the buffer at three of its four entries -- the LD stream
  cannot wait, and without the gate the two streams fill the buffer by one in every cycle both
  land (Geekbench 5's N-Body epilogue, a run of column-0 loads under four compare results,
  wrapped it and lost a landing). The divide's result waits the same way (`md_room`). The
  Top-Down `dep-fp` class finds the FE stream's destinations by the F stage's, the result
  register's and the MD stage's registers and the buffers' FE entries (`lb_fe`).
- **The head ops run from the head-op register (5.2d-b step 3).** An AMO, LR/SC or CBO goes
  alone, so in slot A and lane A, whose adder generates its address and which reads its rs2 (an
  AMO's or SC's data). Both wait in the one-entry head-op register (`ho_*`), the op's record
  written at dispatch, until M takes it -- when no load or store is unfilled, which is always --
  and M runs it at the ROB head as before. One head op is in flight at a time (`ho_inf`, the
  credit `CR_L`), a CBO dispatches only once every older load and store has its address
  (`CR_CBO`), and no load or store dispatches while a head op is in flight. There is no `u_iq_l`,
  issue register or M-side execute unit, and M holds only a load, a store or a head op
  (asserted): its result is the LSU's, and it redirects only to trap.
- **A load's or store's address-only fault rides in its entry** (5.2d-b step 1). A page
  crossing, a non-canonical VA or an address beyond the top under Bare fills the entry as
  faulted (`xo_flt`, `xo_fc` into `a_flt`, `a_fc`), exactly as the walker's page fault does, and
  traps from the SYSQ when it is the ROB head; `tval` is the full VA M wrote at the fill
  (`fva_l`, `fva_s`, since the entries hold VA[38:0]). M holds only its own accesses' faults
  (AMO, LR/SC, CBO) -- asserted -- and a faulted entry never reaches memory, so the page-offset
  alias test need not be exact for it.
- **cbo.clean, cbo.flush and cbo.inval translate as loads** (the CMO specification permits them
  wherever a load or a store is, and W without R is a reserved PTE) and fault as stores
  (`cbo_fc`: 13 to 15, 5 to 7); cbo.zero translates and faults as a store
  (`core/directed/cboperm.S`).
- **The LSU decides a port start from the port's fields only.** M's request fields describe the
  access only when it is M's (`eff_amo`, `eff_cbo`): M can be empty with its fields left from an
  older op while the LQ or SQ starts one. Asserted: the port's access never runs an AMO's states,
  and a fast load's read is word-aligned.
- **The D$ is non-blocking** (`rv_dcache.v`; its header is the full description). Two lookups a
  cycle. The main lookup -- a released waiter, else a load, else a store-port op the store lookup
  does not take, that op first when it is urgent (its queue full, a CBO, an NC store or the I$'s
  clean at its head, or the head waiting `ST_WAIT` = 3 cycles) -- reads the arrays at T and
  resolves at T+1. A load comes through the read door or straight from M (the fast source,
  `f_*`): M's load is presented only in a cycle `f_ok` says it is taken, the store lookup's turn
  reads a registered hint (`f_hint`, M holds a load) rather than the late request, so the arrays'
  address is a function of registers and M's address bits (rule I6), and `f_cancel` voids the
  lookup at T+1; a load's answer is `rd_valid` at T+2, by its tag, and a hit behind a miss
  answers first. The store lookup takes the queue's plain stores beside it (below, §9.1). A miss never holds the lookup: it parks in the **waiter table**
  (one entry per requester tag, one for the store) on the MSHR filling its line, or on the next
  MSHR, write-back entry, NC slot or store to free. 8 MSHRs, each with a 64-byte **merge
  buffer**, so a store miss completes into the buffer at once; 2 **write-back** lines, a dirty
  victim read out whole the cycle after its miss and written only when no read waits for the
  port; one **NC slot**. Every response is matched by the tag its request carried, on the D$'s
  side (`rd_tag`) and on the memory port's (slot ids, §9.2) -- rule B1 by construction.
- **Ordering inside the D$.** A store is taken into the D$'s store queue before it resolves,
  so a load to the line of any queued store waits for it (`blk_st`); a store's completion is a
  freeing that releases it. A load whose row was read at the edge the store lookup wrote it
  replays (`blk_w1`): the block RAM's read beside a write is not data. The lookup yields in the
  cycle a line's last beat is written, so no lookup ever meets an install, and an array has
  one write statement. Waiters released by their MSHR replay before those waiting on any
  freeing, and those replay round robin: a fixed order starved the store under slow memory.
- **Multiple outstanding loads: the fast path (C4a step 1, 2026-09-18).** A queued load that
  is cached, inside one word and to memory (DRAM or the local SRAM) needs nothing from the
  LSU's FSM after its read is issued: the address is translated, the fault decided, and the
  only per-request state is how to format the word when it returns. So it starts from S_IDLE
  and the FSM STAYS THERE; the next access starts behind it; its D$ read carries the load-queue
  index as `rd_tag` (`{1'b0, idx}`; the walkers `TAG_PTW` and `TAG_PTW + 1`, past the loads'
  tags, and the FSM's slow reads one fixed `TAG_SLOW` = `TAG_PTW + 2`), and the response is claimed by that tag -- never by "only one in
  flight" (rule B1). The formatting state lives per tag in the LSU (`o_nb/o_sgn/o_fp/o_boff`,
  `o_v` = a response is still coming), the landing is `ld_land_fast` by `pt_rtag` into the
  load queue's entry, the ROB, the lane's landing (above) and the retire record. Each per-tag
  lookup a landing makes (the LSU's formatting and kill state, the load queue's payload) is read
  at both entries it can be -- the D$ response's tag and the other candidate, the waiting
  forward's or the slow access's, registers all -- and the response's valid picks the answer, so
  the late pick reaches no array index (rule I6). A start needs no earlier read
  waiting for the D$'s accept (`port_free = ~mem_rbusy`, a register: the SoC's read door holds
  up to two waiting reads, the one presented and the one behind it, so a read starts every
  cycle the D$ takes one) and the tag not outstanding (`~o_v[tag]`); a store needs neither. A redirect kills every fast load in flight, INCLUDING one starting in that
  cycle (`o_kill`; the slow path's `ld_sq` does the same) -- the response still comes and is
  dropped. The landing's kill is a flop: the D$ names, from registers, the tags its next answer can
  carry -- its lookup's and its NC slot's (`rd_pre_a`, `rd_pre_b`) -- and with the answer which it
  was (`rd_src_b`); the LSU registers each candidate's kill as it will stand, and the forward ring's
  next oldest's, so no 16-way lookup of `o_kill` stands in front of the wake (asserted equal to it),
  and a landing is the D$'s valid or a waiting forward, registers alone. The landing's destination
  likewise: the LQ registers `{rdv, prd}` a cycle ahead at the D$'s two pre-tags, the forward ring's
  next oldest (`fw_lt_n`) and the slow path's start, and a landing picks by flags (`LDPRE`, asserted
  equal to the direct read), so no 32-way read of the queue stands in front of the wake, the store
  queue's snoop or the lanes' straight-through choice. Straddles, uncached and device loads, AMOs and LR/SC keep the FSM path and its one
  slow landing (`ld_inflight_idx`), which yields its cycle to a fast landing (one PRF and one
  ROB port). Re-done from the retired da28895b. tiny128 at 60 M, measured DDR shape: IW=2
  13,490,465 -> 13,975,979 (+3.6%), IW=3 13,371,386 -> 13,835,891 (+3.5%). ldbench (L1-resident):
  pointer-chase latency 5.00 cycles per load unchanged, 8-stream throughput 4.00 -> **2.00**
  cycles per load (overlap 1.24x -> 2.49x) -- exactly `rv_cache`'s door, one read per two
  cycles; `rv_dcache` takes a read every cycle and has 8 MSHRs.
- **A plain store and a plain load leave M without touching memory.** Their M pass only
  TRANSLATES; the PA is filled into `smolrv64_sq` (stores) or `smolrv64_lq` (loads) and M is released
  on `xo_v`. Memory is reached later. A load goes through the one pre-translated port `pt_*`; a
  plain store to memory (cached, inside one doubleword: the entry's `c_fast`, a flop set when its
  address arrives) goes to the LSU's **store register**, which writes it to the D$ beside the
  port's loads; any other store (uncached, a device's, one straddling two doublewords) takes the
  port and the LSU's state machine, the store winning, once the store register is empty, so stores
  reach memory in program order. A store drains only once the ROB has COMMITTED it (§6, the
  irrevocable pointer), which means every older op is done. The store queue is thus SENIOR to
  retirement (2026-09-04): a committed store's ROB slot completes and retires while its bytes are
  still in the queue, and the queue keeps the entry until memory has it. It HANDS a store OFF
  (`c_take`, at its handoff pointer) and FREES it (`c_done`, at its head): at the store register's
  write, so a younger load is ordered against a live entry until the D$ holds the store (the D$
  then holds a load to the line of any store it has queued); at the state machine's last write
  for a store through the port (uncached, a device's, or one straddling two doublewords), since a
  load M starts in its own cycle reaches the D$ beside the port and would pass it otherwise. At
  most one store waits between the two (asserted).
  Measured on the GB5 checkpoint slices (§11.2; Navigation, HTML5, SQLite, PDF Rendering, Camera,
  50 M retires after 20 M of warm-up) against the port shared by both: **-5.48%** of their cycles
  (PDF -16.77%, HTML5 -4.53%, SQLite -2.63%, Navigation -1.42%, Camera -1.43%), pi -0.69%, the
  60 M Linux lockstep +1.86% retires; a load ready for the port had waited behind a store in 16%
  (SQLite) to 52% (PDF) of the cycles. Two consequences, both asserted: a redirect flushes only the UNCOMMITTED
  tail of the queue (`tailc <= kcc`; a committed store is architecturally done and survives),
  and "every older store is visible" is no longer implied by the ROB head or by `rob_empty`
  -- the serialization drain (`drained`) and the CBO start in M (`m_cbo_wait`) each name the
  queue (rule C5). The store queue keeps a REGISTERED per-load copy of "an older store is live"
  (`l_older`, the OR of the distance vector `l_block` already computes, one cycle old), which
  the reordering counter (`mem_reord`) reads at the candidate's index; for one load the set of
  older live stores only shrinks (its tag is fixed at dispatch, later stores are younger, head
  only advances), so the copy can only be the more conservative. The live query port stays as
  the oracle the core asserts against. The alias block is a register per load the same way
  (`l_block_q`, T1 (L) step 2): once a load's address is in, its block only clears (oldm
  shrinks; a store's address arriving turns "unknown, blocks" into the overlap), and the
  fill cycle -- the one moment the copy would lag the wrong way -- computes it from the row
  being written (`fill_row`, against `av_next`). The load queue's candidate select reads the
  copy; a load never starts while the live block holds it (asserted). And the landing is
  the LSU's load terms alone (`pt_ld_done`: S_LD/S_LD2, the D$'s registered rvalid,
  xword_q, own_pt; step 3): `pt_done & ~pt_is_store` is the same value, but its cone is the
  whole `acc_done`, whose store terms select on this cycle's new start -- gate W6's fan-out
  report put `pt_v` on 142 of the 200 worst paths and `ld_land` on 140. The CBO's predicate is an entry
  WITH AN ADDRESS (`sq_av_any`), never the occupancy: entries are allocated at dispatch,
  so the queue holds stores younger than the op in M, which cannot translate until M
  frees -- a CBO waiting for an empty queue deadlocked build L at SLUB init (2026-09-04;
  `workloads/fphammer/cbozero.c` is that shape, 129 retires then silence on the unfixed
  RTL). Only an older store can have an address, because M translates in program order.
  A CBO also waits for the ROB head: it writes memory and M speculates past unresolved
  branches, so M's direct accesses (AMO, LR/SC, CBO) start only at the head, asserted at the
  start (integrity bit 39, rule D17). At the head no older load is live, so the load queue
  needs no term (asserted).
  For a younger load nothing changes: an entry in the queue, committed or not, is a store
  whose bytes are not in the cache yet, and the alias matrix holds the load behind it.
  **The drain ends when the D$ takes the store**: the cache has captured address, data and
  mask and completes the write on its own. It takes a store in any cycle its store queue has
  room, which `wr_room` (a register) announces, so the LSU's completion never waits on an
  accept travelling back (a combinational accept into M's `done` and the scheduler was
  0.65 ns over at 166 MHz). The LSU knows a store to memory from one to a device by its
  registered `mem_q`: a plain store to a device (bare M-mode has no PBMT, so no NC bit) ends at
  the device's own accept instead. In the cycle a store is taken the LSU loads the next queued
  store straight into `S_ST` (`take_next`), so stores stream at one a cycle.
  The store register takes its store the same way (`wr_room`), one a cycle, and writes only
  when the state machine does not; its writes are only ever memory, so the SoC's device address,
  decode and write strobes follow the state machine's write (`dmem_wfsm`) and its own base. Two
  classes wait for COMPLETION instead, `wr_cpl`, which
  the D$ raises only for them: a CBO, whose flush must be in memory before the doorbell store
  behind it, and an uncached write, whose bytes must be in DDR before a later device write
  starts the DMA that reads them. The plain write's `wr_ack` is a counter pulse nobody waits
  on, and it must not reach `dmem_wready`: it used to land while the LSU waited on a device
  store started since (the `two write acks at once` assertion caught it). A device store
  carries no NC bit without page tables, so the device acks are in the accept term as well
  (`workloads/fphammer/cbozero.c`'s UART printed `c` forever when they were not). What the
  LSU waits on is decided by the request's class (`st_fin`), never by which ack shows up.
- **The translate-only pass does not arbitrate, and does not wait for the LSU's FSM.** It
  needs the MMU and nothing else, so `smolrv64_lsu` presents it (`xl_x = req_valid & req_xlate`)
  whatever the FSM is doing and whoever the pre-translated port granted this cycle; a walk it
  starts runs on while a queued access uses the FSM. Only an access that STARTS the FSM
  (AMO, LR/SC, CBO, and the `req_early` start of a load) yields to the port. Until 2026-09-03
  the translate pass was gated with those on `~pt_start` and `st == S_IDLE`, which put the
  port's grant -- `smolrv64_sq`'s live bits, `smolrv64_lq`'s candidate, the alias matrix -- in series
  with M's completion for every plain load and store, and M's completion is the wakeup
  broadcast, the redirect and the hpm events: 1708 of the 3401 endpoints under +0.35 ns in
  that day's routed checkpoint started at `u_sq/v_reg` for this reason alone. Rule I9.
- **A queued load carries everything its access needs**: PA, size, sign, fp-ness, the
  Svpbmt uncached bit and the "DRAM/LRAM, idempotent" bit that licenses a speculative
  access (`mem`; a device load waits for the ROB head: `hd`, the head compare registered per
  entry, so it goes the cycle after the head reaches it), all written
  into `smolrv64_lq` at the translate pass FROM THE TRANSLATE'S OWN PA (`t_paddr`, rule D12: the
  LSU's `eff_pa` is the port's address in a cycle the port starts, and classifying the
  translating load by it sent device loads off the wrong path -- the board's dead NIC,
  2026-09-17; the LSU asserts every non-DRAM start is non-speculative, and the queue
  recomputes the bit from the PA it stores and dies at the fill on a disagreement) -- and, from
  dispatch, the store-seqno that bounds which store-queue entries are older than it: the
  queue's tail COUNTER, one bit wider than its index (`SQ_TB`), because a full queue's tail
  equals its head and an index-width seqno then counts zero older stores where there are
  NENT (rule B8; found by the Geekbench boot under the cosim on 2026-09-04, after Ubuntu
  userspace had segfaulted on the board on the pointers it corrupted). The uncached bit
  was missing until 2026-09-04 and the pre-translated port hardwired it to 0 for loads, so
  an NC load that went through the queue (rather than the early start) was cached: the
  board's virtio rings read stale (`id 0 is not a head!`), and no simulation could see it.
  Rule B7.
- **Walks leave M (D$ plan increment 5b).** M's translate-only pass looks its op up in the
  dTLB's lookup port (`mmu`'s `s_*`: the TLB alone, never a walk). A hit is the pass above,
  unchanged: the entry gets its PA and a load may start in the same pass. A miss, or a hit whose
  permissions fail (which only a fresh walk may turn into a fault), does not hold M: the entry is
  filled with its VA alone (`tv` clear) and M lets go: M's completion is the address-only test
  (`xo_any`), so the TLB is never in M's done or the entry's fill enable. M decides only the faults the address
  alone does: a page-crossing misalignment, a non-canonical VA under Sv39, an address beyond the
  top under Bare (`s_flt`); so the entry's VA[38:0] is all the walker needs.
  - **The walker** (`mmu`'s walking port) serves M's FSM-starting op first (an AMO, LR/SC or CBO,
    at the ROB head; the port's requester is a register, `pm_q`, so it takes the port the cycle
    after it asks and M's control never reaches the TLB's read address), else the store queue's
    first uncommitted entry (stores commit in order, so no other store is waited on first), else
    the load queue's oldest untranslated entry. It keeps the entry it was asked for until it
    answers (`wk_lk`, with the load's index `wk_lk_li`) or a redirect ends the walk with the entry.
    Its answer writes the entry's PA, classes and `tv`, or its fault and cause.
  - **What waits for `tv`:** a load's access (the candidate needs it) and a store's commit
    (`kc_v`). A queued access may start while the queue's walk runs; only M's own walk keeps the
    LSU's FSM idle (`wk_m_q`).
  - **A faulted entry traps at the ROB head.** It never completes (the load is never sent, the
    store never commits), so its op reaches the head and stays. Its record -- cause, the VA as
    `tval`, the op's PC and seq, which each entry keeps from dispatch -- then loads the SYSQ as a
    trap from dispatch does, and fires through `sy_fire`, the one trap gate.
  - **The alias matrix compares page offsets,** VA[11:0] = PA[11:0]. No queued access crosses a
    page, so two in one page compare exactly; synonyms share the offset, so they never miss each
    other; and the test runs the cycle the VA arrives, whatever the walker does.
- **A load takes any free entry of its bank; program order is an age matrix.** The LQ is four
  banks (an entry's low bits); a group's loads take the banks that have a free entry in turn from
  a bank pointer, in program order, each the lowest free entry of its bank, so a row of loads
  allocates in one cycle and every array the allocation writes takes one write a bank, and the
  queue refuses a load only when fewer banks than the row's loads have room (`spl_ld` and `pcm` are `smolrv64_bankq`s;
  the fields read at every entry are flops). An allocation sets its row to the live entries and
  the row's loads ahead of it (all older) and clears its column; every "oldest of these" (the
  access candidate, the walker's and the fault's picks, M's fill pick) is the member no other
  member is older than. Loads land out of order, so a ring's next slot could hold a load
  still in flight while others were free: on the GB5 checkpoint slices 76% (Navigation) to 88%
  (SQLite) of the cycles the queue head had no load-queue credit had room by the count. Free-slot
  allocation against the ring: those slices -0.62% of their cycles (Navigation -0.96%, HTML5
  -0.98%, SQLite -0.38%, PDF Rendering -0.19%, Camera -0.59%), pi -1.24%, the 60 M Linux lockstep
  +0.34% retires.
- **A load's access starts in the cycle M fills it; an older store that blocks it cancels it a
  cycle later (D$ plan step 2, 2026-10-19).** M presents a translated load straight to the D$ in
  its own cycle (`smolrv64_lsu` `f_go`, the SoC's fast source): the arrays read at M's address bits
  [11:3] -- the index is inside the page offset -- and the dTLB's page number, read in that cycle
  by the lookup M already makes, is registered for the compare at T+1. The answer is at E+3 and
  the dependent's address at E+4. The start is gated by the dTLB alone (a hit whose permissions
  pass, the page in DRAM by a bit of the entry `mmu` sets at the fill, cached, inside one
  doubleword), its tag free, and the D$'s word that it takes a load this cycle (`f_ok`, of
  registers: no waiter replaying, no urgent store, the read door, the LSU's port register and
  the walkers quiet), so a started load never waits at the door. Never by the store queue: the
  alias test -- M's address against every store's, the filling load's block bit -- takes the
  whole cycle (5.62 ns from `m_addr` on 039314ad's routed checkpoint, 0.117 ns of slack), so it
  stays registered and its block CANCELS the start a cycle later (`smolrv64_sq` `l_cancel`: the
  started-last-cycle bit AND the block): the D$ voids the lookup at T+1 (no answer, no fill, no
  wait), the LSU drops the tag, `smolrv64_lq` unsends the entry, and the load forwards or waits as
  any blocked entry does; the ordering check does not count it as having read memory. Any other
  load goes the queue's way: `smolrv64_lq` registers the address, then selects the oldest *ready*
  entry, in any order the loads become ready: filled and translated, not faulting, no older store
  aliasing it (an older store with no address counts, unless its wait bit lets it go), and, for a
  device load, its op at the ROB head; the port register presents it a cycle later. Loads reach
  memory out of program order: a single hart cannot tell, a store never commits ahead of an older
  load that has not landed (the irrevocable pointer), and a device load waits for the head, which
  orders it against everything older. Fill, M's early release and the landing path are all
  unchanged; only the start moves. Measured against 039314ad: GB5's 77 SimPoints -1.70% of the
  single-core cycles, every subtest (SQLite -3.13%, Text Rendering -3.05%, Image Compression
  -2.84%, HTML5 -2.82%, Navigation -2.28%, AES-XTS -0.14%); the 60 M Linux lockstep +1.66%
  retires, Dhrystone -0.86% cycles, CoreMark -8.37%. On the SimPoints 59% (N-Body) to 96%
  (Horizon Detection) of the loads start in M's cycle and stand, 0.6% to 19% are cancelled, 2% to
  12% find the D$ taking no load, and 0.4% to 12% are not in the dTLB (`LDPATH-SIM`).
  **Do not extend it by also skipping the fill.** A version that dropped the queue entry and
  let M hold the load through its access was a REGRESSION -- ldbench 6.00 -> 7.00 cyc/load,
  Camera unmoved -- because releasing M lets the following non-memory instructions execute
  while the data is in flight, and that is worth more than the latency it costs. The queue's
  two cycles are not overhead; one of them is.
- **An address generator issues on a load's D$ lookup, before it answers (D$ plan step 3a,
  2026-10-20).** An 8-byte integer load with a destination that M starts in its own cycle
  registers that destination as the speculative wake (`smolrv64_core` `sw_v`/`sw_p`, flops: rule
  I12), broadcast at T+1, the lookup's compare, to the lanes' schedulers (§6.1). An entry picked on
  it at T+1 executes at T+2 with the D$'s answer as its base (`sp1`: `rd_data`, an aligned
  doubleword, into the address adder), and the verdict is a flop from the D$: its answer this cycle
  is that load's (`rv_dcache` `rd_f`). Right, the address goes to M and the arrival RAMs as any
  lane's does and the entry frees; wrong -- a miss, a wait or the store queue's cancel -- the
  lane's delivery is suppressed (`l_lm`, an enable) and the entry is a candidate again, its address
  source waiting for the landing's wake. An address generator writes no register, wakes nothing
  and completes nothing in its lane, so a wrong issue has nothing to undo and costs its lane one
  execute slot. A load whose result is a load's or a store's address has load-use 3: its address
  at E, the D$ at E+1, the consumer picked at E+2, its address at E+3; every other consumer keeps
  4. Narrower loads (their answer is shifted and extended), FP loads, and loads the queue's port
  starts do not wake speculatively. Checks: every speculative issue is decided in the next cycle
  and not issued again before; a base that stands is that cycle's D$ response, by its register;
  only an address generator executes on one. Measured against da5d52b3: a bare-metal pointer
  chase (`workloads/rvbench/local/chase.c`, 16 KiB, the lockstep) 3.00 cycles a load, and the
  board's `chase2` 3.0 at 16 KiB (f644696c, was 4.0); CoreMark
  -3.48% cycles, Dhrystone -2.59%, the 60 M Linux lockstep +0.19% retires; GB5's 77 SimPoints
  -0.13% of the single-core cycles (Text Compression -0.38%, AES-XTS -0.33%, SQLite -0.31%, Clang
  -0.30%; the FP kernels within 0.03% either way). 83% (the boot) to 89% (Dhrystone) of the
  speculative issues stand (`SPEC-SIM`).
- **Loads go ahead of older stores whose addresses are unknown, and replay when one overlaps
  (memory-dependence speculation).** A load whose wait bit is clear is not blocked by older stores
  with no address; only a known overlap blocks it. The store queue keeps its per-load overlap
  matrix `conf`, and a load that has REACHED memory (sent, or landed and watched) while an older
  live store overlaps it is flagged for replay (`l_viol`): evaluated every cycle from the matrix,
  and in the very cycle a store's address arrives against it (`arr_ov`, the same compare that
  writes the matrix), so the flag is a flop by the time that store can commit. The registered
  block copy rises in that edge too, so a load never starts on a stale block.
  - *Watching.* A load that lands while an older store's address is unknown stays in the load
    queue (`lnd`, address valid) until the last such address is in and showed no overlap; a
    flagged one stays until it replays. Everything else leaves at its landing.
  - *Replay.* The flagged load is reported through the queue-fault path (`f_rp`) as a record the
    SYSQ takes when the load is the ROB head: `sy_rp`, no trap and no CSR (`csr_file` never sees
    it), a redirect to the load's own PC with its own seq, and the retire of its row held from the
    cycle it is flagged (`rc_kill`'s per-column term, then `rp_hold` through the record and the
    SYSQ). The load re-executes behind
    the stores it conflicted with. The irrevocable pointer stops at it meanwhile. **The hold reads
    registers:** the LQ's pick of its oldest flagged load and that load's ROB index are registered
    (`rp_f_q`, `rp_rob_q`), so the commit ports compare against a load flagged two cycles back; for
    the one cycle after a NEW flag (the LQ's `f_new`, rare), nothing that merely completed commits
    (the ROB's `c_hold`; a head finishing with its own redirect still does) and the irrevocable
    pointer holds (`rp_all`). Asserted: no commit port ever retires the load the live pick names.
    (The live pick into every commit slot was a 24-level cone, -0.48 ns in the oldest-first pick's
    builds; the registered one cost the 60 M boot 0.24% of its retires.)
  - *The wait table.* `mdw`: a 2-bit saturating counter per load-PC hash (PC[8:1], 256 entries),
    a distributed RAM with one write a cycle; it forgets one entry every 16 K cycles, so each
    entry every 4 M (a step that loses its cycle to a replay or a training event waits for the
    entry's next turn). Measured against forgetting the whole table at once: +0.05% cycles over
    five GB5 slices, the 60 M boot +0.004% retires; never forgetting costs Navigation +0.73%. A load waits for older unknown addresses when its counter is 2 or
    3. A replay sets it to 2. The store queue reports, for every load, the cycle the last unknown
    older address comes in and whether the load overlapped it (`l_res_a`, `l_res_c`): overlap
    counts up, no overlap counts down, one event a cycle, trained the cycle after it resolves. A load that aliases rarely goes back to
    speculating after one clear resolution; one that aliases often keeps waiting.
  - *Measured* (lockstep, IW=3, rvbench): replaying costs ~28 cycles, a wait about as much, so the
    table waits only after a replay. Dhrystone never replays; rsort 85-119 times in 94-103 K cycles.
    `SMOLRV64_MDSPEC` selects never (0), the table (1), always (2).
- **A load held by the one older store that holds all its bytes takes them from it (store-to-load
  forwarding).** Without it such a load waits for the store's commit and drain and then reads the
  D$. `smolrv64_sq` keeps a second matrix beside `conf`, written at the same sites: `cov[i][g]`,
  store g holds every byte of load i -- the same page as well as the same offset, as
  `{bare, VA[38:12]}` (`conf` compares offsets only, so another page at the same offset overlaps
  there; `bare`, the access was not translated, keeps a user store's VA apart from an M-mode load's
  PA of the same value, since a committed store outlives a trap's flush), its bytes covering the
  load's inside one doubleword. A load is forwardable (`l_fwd`, a register per load) when exactly
  one older live store overlaps it, `cov` holds for that store, its data is in, it is translated,
  cached and not faulting, and no older store's address is unknown unless the load may go ahead of
  unknown addresses (the dependence table's wait bit clear, as for a load that reads memory). A
  forwarded load counts as having reached memory (`e_acc`): a store older than it whose address then
  arrives and overlaps it flags it for replay; its own forwarding store's overlap is set aside while
  the load stands forwarded (latched as the forward starts, dropped with a replay). The flag clears in the edge its store drains, so the store it names never outlives it.
  `smolrv64_lq` offers a forwardable held load like a clear one (`x_fwd`); it takes the port like
  any queued load and starts as a fast load in the LSU, which asks the D$ for nothing: the store's
  bytes, read in the start cycle through the same landing bypass as the drain's and kept as the
  store holds them (its first byte at bit 0; the load's landing shift is its offset past the
  store's, recorded at the start), wait in the forward queue, two deep, and land by the load's tag, through the
  fast path's formatting, in the first cycle the D$ returns no response, the older first. A forward
  starts while another waits; `fw_busy`, a register loaded with next cycle's count, says both slots
  are full. The queue is a ring: the free
  slot `fw_wp` names takes each cycle's candidate whether it starts or not, a start moves `fw_wp`
  on and a landing `fw_rp`, so neither the start nor the D$'s response reaches a slot's write. The
  landing's tag is a register of its own (`fw_tag0`, the older forward's as the ring will stand next
  cycle), so the lookups a landing makes by it start at a flop.
- A load's fault is decided **before the access starts** (by M for the address-only faults, by the
  walker into the entry for a page fault, above): for an FSM-starting access
  `mis_flt` and `xl_flt` are qualified by `xl_f = req_valid & ~req_xlate & (st == S_IDLE)`,
  and for the translate-only pass by `xl_x = req_valid & req_xlate`, which never enters the
  FSM at all. The pre-translated port's grant (`pt_start`) appears only in the START
  decision (`xl_ok_f`, `xl_early`), never in a request to the MMU or in a fault -- and M's
  own fault tests (`xpage`, `amo_mis`) use M's own width (`req_nb`), not the width of
  whichever access the port selected this cycle. Once the LSU leaves `S_IDLE` an access
  cannot fault. This is
  what makes precise exceptions possible **with no ROB walk** — M holds only until
  translation resolves (same cycle on a TLB hit), and after that the load is architecturally
  guaranteed to complete.
- **A data-side fault completes M one cycle after the LSU reports it.** The cycle that
  reports it only latches it (`m_unit_flt_q`); the trap, the redirect and the ROB-head gate
  read the copy. That keeps `m_addr -> dTLB -> lsu_fault` out of `xtrap_v -> redirect -> the
  fetch adder -> the decoupling queue`, and costs one cycle per data fault -- the rarest thing M
  does. For the same reason the redirect uses `m_done` with the LSU arm removed, and the
  writeback valids `we_ld`/`we_fe` (the wakeup broadcast) use `lsu_done_acc`, the completion
  of an access this stage started: a translate pass or a fault never writes a register.
  Both are asserted equal to the full expressions every cycle.
- **Stores stream at one a cycle, through a 3-entry store queue and a lookup of their own.** A
  plain store goes to the store lookup: a hit writes its bytes through the byte enables into the
  one array that holds its chunk, and a store to a line an MSHR is filling merges its bytes into
  that MSHR's merge buffer, whose bytes override the fill's beats. Only a store to a line neither
  cached nor filling takes the main lookup, and allocates. The queue issues its head, and the
  entry behind it in the cycle the head is in the compare; only the head resolves, so an entry
  that reaches the compare behind a waiting head goes back to the queue (the store's waiter entry
  is only ever the head's). A CBO or an NC store is taken once and held by the LSU until its
  `wr_cpl`, with nothing presented behind it. A store-port op takes the main lookup in a cycle no
  load wants it, or when it is urgent (above); measured on the PIPT D$, where every store took
  that one lookup, against stores first: -1.31% of the GB5 checkpoint slices' cycles (Navigation -3.07%, HTML5 -2.67%, SQLite -0.74%, PDF Rendering
  -0.84%, Camera +1.15%), pi -0.99%, the 60 M Linux lockstep +2.16% retires. `ST_WAIT` = 1
  measured +0.03%, the integer slices 0.4 to 1.0% slower.
- **NC and CBOs.** An NC access drops any cached copy (a dirty one written back first) and then goes to
  memory alone through the NC slot, a store with only its bytes -- coherent with a cacheable
  alias, as `rv_cache`'s flush-around was, and nothing allocates. cbo.clean writes a dirty
  copy back and keeps it clean; cbo.flush/inval drop it, written back if dirty; cbo.zero drops
  it unwritten and fills an MSHR whose merge buffer is a line of zeros. A CBO or NC store
  completes at `wr_cpl`, once memory has its write.
- **Outside requests: the I$'s coherent fill and coherent DMA.** One port asks the D$ to clean
  or flush a line by PA (`ox_req`, `ox_keep`: a CBO queued in the store queue behind every store
  taken before it, answered by `ox_ack`); `rv_soc_top` gives it to the I$ and the DMA masters in
  turn, each owning it from its first cycle to its answer. It takes a store-queue slot in any
  cycle no store is taken; refused four times with room in the queue, it holds the store port's
  room (`wr_room`) off until it is in, so a store stream delays it at most four cycles.
  - The I$: every I$ line read goes to memory at once, and beside it the I$ asks for that line
    cleaned (`ox_dty` with the answer unless the clean found the line clean on its first look).
    `rv_soc_top` uses the memory's data only if the D$ has answered clean by then; otherwise it
    drops the data and reads the line again after the answer, when memory is current. So
    `fence.i` never cleans the D$: it drains the core and invalidates the I$. (The whole-D$
    clean `inv_req` remains for a requester that wants one.)
  - A DMA master (`ddr4_port_axi`, through `ddr_port_sync`'s snoop channel) has its line cleaned
    before it reads it and flushed before it writes it; the answer means memory has the D$'s
    write-back, which the controller, taking requests in order, serves ahead of the master's
    line. An outside flush's completion BLOCKS its line (`xb_*`): no fill of it starts -- a
    load, store, zero or NC access waits as on a line being written back -- until `ox_rel` says
    the master's write is in the controller; a fill already started is the flush's to wait for.
    The device trees mark every DMA master `dma-coherent`, so Linux keeps no CBOs around its
    buffers. The I$ is not snooped: a device write still makes the next fence.i clear it.
- **Every request carries its PA**: the core translates before it asks. The set is inside the
  page offset, so the VA names it as well; the dTLB in the access cycle, the D$ taking the VA, is
  step 2 of `docs/PLAN-2026-10-03-dcache-vipt-l2.md`'s revised order.
- **No access spans an 8-byte chunk**: the LSU splits them (the no-span alignment), and a read
  names its chunk -- the address aligned to 8 bytes, asserted at the D$'s door -- which the D$
  answers as stored; the LSU takes the access's bytes from it (an atomic its half).
- Load-format controls (`nb`, signed, fp) are **latched at dispatch**, not read at
  completion — the stage they came from has moved on by then.

### 8.1 Address translation

Two independent `mmu` instances — **iTLB** in `smolrv64_core`, **dTLB** in `smolrv64_lsu`.

| | |
|---|---|
| TLB | **direct-mapped** on the VPN's low bits, read in the translate cycle: **64 entries** each, the dTLB and the iTLB (LUTRAM, one depth: no read mux) |
| Second level | the dTLB's **2048 entries** (8 MiB of 4 KiB pages, the reach GB5 Machine Learning's SGEMM column walk needs) and the iTLB's **512** (`L2N`), direct-mapped, on the miss path: one block RAM of whole entries, read in `IDLE` at the request's VA and checked in the cycle after a first-level miss (`L2LK`) against the walker's registered VA, so each first level keeps its one LUTRAM depth. A hit (perms checked as on the first level) answers as a one-cycle walk and refills the first level; a miss walks, and the walk installs its leaf in both. It looks up only where a walk could start (`walk_ok`; the dTLB always) |
| Scheme | Sv39, 3-level hardware page-table walk |
| PTW | reads through the D$ read port (§8); a miss fills through the D$ |
| Superpages | 2 MiB / 1 GiB; misaligned superpage → fault |
| Ssvnapot | level-0 NAPOT leaves recognised |
| PA width | `PABITS` (`AW`): the bits of the DRAM's top, 33 at 4 GiB (`smolrv64_width.vh`). A TLB stores `PABITS - 12` bits of a page number (an installed page lies below the top); the walker decides the cap on the PTE's whole 44 |
| Flush | `sfence.vma` and `satp` writes advance a 16-bit generation (`GB`); an entry hits only when it was installed in the current one (`tlb_g` = {installed, generation}, LUTRAM beside the entry). No entry is written on a flush; the arrays are cleared over `max(TLBN, L2N)` cycles at reset and when the generation wraps, and nothing hits or installs while they are |

**No iTLB walk past a weak branch.** The core counts the conditionals dispatched on a weak
direction (the effective counter, the corrector's when it hit, is 01 or 10) until each resolves in
its lane; the backend squash clears the count, which it can do exactly because it fires at the ROB
head. While the count is nonzero and no frontend restart is pending, the iMMU starts no walk
(`walk_ok`): a TLB hit still translates, a miss waits. Every counted branch is older than the
fetch, so it resolves whatever fetch does. The I$ holds a miss's line read on the same terms
(`fill_hold`); a restart of the fetch stream arriving first (`fill_cancel`, in the cycle the
hold drops on a mispredict) answers the waiting miss and the request behind it with no data,
and the ring drops those answers as it drops every stale one. A larger iTLB lets more wrong-path
fetch reach the I$ (tiny128 boot, 300 M cycles: 64 entries alone -1.54%, the walks halved but the
I$ misses up 4.6%, one line in five never retired from); with the walk throttle +1.03% over 16
entries, with the fill hold too +3.20% (fe:icache 24.91% -> 22.67% of cycles). `ITLB-SIM` and
`ICMISS-SIM` in the Linux bench count walks and I$ misses whose page or line nothing retired
from; a held miss that is cancelled counts as a miss.

The MMU also range-checks the resolved PA: anything outside {RAM, CLINT, PLIC, UART, LSRAM,
virtio} faults rather than being silently dropped.

Each `mmu` also drives **`t_lvl`** (the resolved leaf level, from the walk or the hit TLB
entry; Stage 2 increment 1). The iMMU's `t_lvl` (`imem_xlvl` out of the core) is stamped onto
the fetch ring at each I$ request (`rg_lvl`, Stage 4 increment 1a) and served as the
held bytes' page size (`imem_lvl` into the core → fetch), so the fetch's **enclosing-page
cap** (§4.1) uses the actual 4K/≥2M page size without putting the iMMU on the hit cone. Storing
it per adapter chunk (not per cache line) suffices because the window drops on `imem_ctx_chg`,
so a slot's page size is always the current mapping's.

---

## 9. Memory system

### 9.1 L1 caches

The D$ is `rv_dcache`, the alias-free VIPT L1 of `docs/PLAN-2026-10-03-dcache-vipt-l2.md` (step 1 of
its revised order); the I$ is `rv_icache`, a read-only module of its own (Stage 4 increment 0,
`docs/PLAN-2026-09-24-frontend-stage4.md`).

| | I$ (`rv_icache`) | D$ (`rv_dcache`) |
|---|---|---|
| Size | **128 KB** (`SIZE_KB`) | **128 KB** (`DC_KB`) |
| Associativity | 2-way, not skewed | **32-way**, a way being a 4 KiB page; NRU placement |
| Sets | 1024 | 64 |
| Line | 64 B (512 bit) | 64 B |
| Indexing | **VHPR** (virtual index and tag; physical reconcile on a miss) | **VIPT, alias-free**: the set, PA[11:6], is inside the page offset; every request carries its PA |
| Read | a window of up to 16 bytes from any halfword, inside a 4 KiB page, a new one taken every cycle, answered the next | 64 bit, one load lookup a cycle, answered at T+2 by tag |
| Stores | -- | **a store lookup beside the load lookup**: a committed plain store a cycle, written into the one array that holds its chunk or merged into its line's fill |
| Misses | one demand read + prefetch | **non-blocking**: 8 MSHRs with merge buffers, a waiter per tag |
| Write policy | fill-only | **write-back**, 2 write-back lines |
| Prefetch | next line, **on**, within the 4 KiB page | none |
| Storage | BRAM (`smolrv64_sdpram`, 1R1W, `READ_LATENCY=1`) | 32 RAMB36 of 512 x 64 with a write enable per byte (`smolrv64_sdpram_be`), diagonal in groups of eight |

**Not UltraRAM.** The I$'s data is an even and an odd 16-byte bank per way -- a line's pairs 0
and 2, and 1 and 3 -- each two 64-bit block RAMs with a row of its own ({set, pair[1]}). A read is
a **window**: up to 16 bytes from any halfword (`rd_addr` through `rd_last`), the window's first
pair and the pair after it, which has the other parity; the even bank reads the next line's pair 0
when the window starts in pair 3. Each half takes its own line's way, and the answer is **by bank**,
as the BTB's banks are (§4.2): halfword b is the window's halfword at address bits [3:1] == b, the
first pair's when b is at or after the window's first halfword, else the pair after's -- a pick of
the even and the odd pair's halfword b, no rotation. The ring appends bank b at its tail plus b's
position in the window. The fetch ring asks for windows from the stream's address (§4.1),
so a stream that is not 16-byte aligned runs as an aligned one does (ChaCha20: 25 cycles an
iteration; 34 when the ring asked for 16-byte-aligned pairs).

The I$ does not skew: a VHPR cache's physical probe finds the same-offset synonym candidates by
a straight index, and a hashed index would scatter them across sets.

**The D$ (`rv_dcache`).** A way is a 4 KiB page and the set is PA[11:6], inside the page offset, so
the VA names the set too and no line can have a synonym; the core still translates first and hands
every request its PA, and a hit is the PA's tag (PA[32:12], 21 bits: `PABITS - 12`) matching one valid way of the
set. **The data is diagonal in groups of eight:** chunk c of way w sits at row {set, c} of array
{w[4:3], w[2:0] + c}. A load reads row {set, c} of all 32 arrays and takes the hit way's by a
rotation within each group, never the set's 4096 bits; a fill beat writes two arrays, one of each
parity, through one address and data bus per parity; a dirty victim is read out whole in one cycle
from its group's eight arrays; a store writes its bytes into one array through the byte enables,
never reading the word. Tags, valid, dirty and the not-recently-used bit are distributed RAM per
way, read at the set, the tags twice: the main lookup's copy and the store lookup's. **Two lookups
a cycle.** The main lookup takes a load (or a replayed waiter, a CBO, an NC access, a store that
must allocate) and answers a hit at T+2. The **store lookup** takes the store queue's head when it
is a plain store: its tag copy names the way, and the store writes the line and marks it dirty, or
merges its bytes into the MSHR filling the line, in the same cycle as the main lookup's load. It
waits for anything of the main lookup's in its set but a load that hits, for a fill beat and for an
install; nothing the main lookup does depends on it. A store to a line neither cached nor filling
allocates through the main lookup, the only store that lookup sees (asserted: while it is queued,
no load to its line gets past the main lookup). Placement is NRU: an invalid way, else a way whose
bit is clear, else the first, the set's bits then cleared. A miss's fill is a four-beat burst into
the reserved way, a cycle after each beat; the line installs the cycle after its last beat is
written. Unit bench: `core/run-dcache-tb.sh` (60 runs: six seeds, in-order and reordering memory,
five shapes; memory equal to a golden image after every clean, and the integrity log clear).

Measured against the 2-way PIPT it replaces (main dbc8ec75, its slices run on 8fbe5bd5, which
retires the same at 60 M): the 60 M Linux lockstep retires +2.58%; the GB5 checkpoint slices take
-2.70% cycles (PDF Rendering -7.03%, HTML5 -2.92%, Camera -2.06%, SQLite -1.05%, Navigation -0.95%),
the pi slices -0.77%. Over GB5's single-core section as its 77 SimPoints (§11.2), which one slice a
subtest overstates, the cycles fall 0.50%: Ray Tracing -6.06%, Rigid Body Physics -3.95%, the
integer subtests -1.3% to -2.3%, Structure from Motion +0.85% and Machine Learning +9.48% -- its
SGEMM column walk at a 4 KiB stride puts every element of a column in one set, which holds 32 of
them, until the L2 holds the column. PDF Rendering's `be:sq-full` falls from 12.0% of its cycles to
3.4%: the store lookup merges 73% of its stores into fills and writes 22% into the arrays. Dhrystone
takes 4.9% more cycles though it mispredicts less: strcmp's byte loads land sooner, so its loop-exit
branch resolves before reaching the ROB head and its recovery walks back about 15 rows instead of
squashing at the head (§6).

**VHPR I$ (`rv_icache`).** A request is a window of up to 16 bytes from any halfword, inside one
page (asserted). It reaches the next line only when it starts in a line's last pair, and then it
hits when both lines do: the next line's tags come from the tag RAMs' second port (the fill's, idle
in every cycle the banks read) and its valid bits from a second read of the valid arrays. A miss
fills the first of the window's lines that misses (never a line it does not reach); the next
line's reconcile probe is a cycle later (`S_RCN`), through the lookup's read of the physical tags.
Per line: valid, the **virtual tag**, the
**physical tag** (PA above the 4 KiB offset) and an **8-bit epoch**. Every request carries the
epoch it was taken in, and **a hit is `valid & (vtag == VA) & (epoch == the request's)` -- no
translation and no physical tag on the hit path**, so an I$ hit never waits on the iTLB. A
`satp`/`sfence.vma` (`ep_bump = imem_ctx_chg`) advances the epoch, staling every line in one
cycle. A wrap (every 256 bumps) invalidates every line over a 64-cycle scan; the tiny128 boot makes
about 9,000 bumps in 60 M cycles, so at 2 bits the I$ was emptied 2,261 times and at 8 bits 38
(lockstep +11.02% at 60 M, `fe:icache` 21.3% -> 13.0% of cycles; +23.83% at 300 M). **Reconcile is the miss path:** a virtual miss compares the same set's physical tags against
the request's PA, and a match is re-stamped with the new virtual tag and epoch and replays (one
cycle), so physically resident code survives a mapping change; otherwise the line is filled
(prefetch buffer or L2) and installs stamped with the request's epoch, so a request taken before
a mapping change never makes its line current. A miss parks at most one younger request and both
replay in order. **The I$ is coherent with the core's stores.** A set is {colour, row}, its row
VA[11:6] = PA[11:6]; each (way, colour) keeps its lines' physical tags and validity in 64-row
distributed RAMs, so every place a physical line can live is at one row. Every write the core
presents probes its line (`pb_v`, a cycle later): all 32 candidates are compared at that row and
every match is invalidated, and a fill in flight or a prefetched line for it lands dead. Validity is
two single-writer bits that must agree -- `ig` (installs, the scan) and `kg` (probe kills) -- so a
kill and an install in one array in one cycle both land. So **fence.i only drains** (every older
store handed to the D$, its probe applied) and redirects; it clears the I$ only when a device has
written memory since the last one (`dma_wr`: device writes do not probe). Epoch roll-over clears
every line with a scan that takes a row of every (way, colour) a cycle: 64 cycles at any size. (Scanning one set a cycle, a 128 KiB I$ lost 2.65% on
the boot's 3,867 fence.i; banked, it gains +0.83% at 60 M and +1.94% at 300 M over 64 KiB.) Today every request still carries the iMMU's PA (`rd_pa`) and the iMMU
still checks every fetch; Stage 4 increment 1 caches each line's execute and user bits and
translates only on a miss. Unit bench: `core/run-icache-tb.sh`. Design: `docs/VHPR.md`.

Measured D$ (`rv_cache`, 64 KiB, before 2026-09-27): **3.24 cycles per access** at a **0.195% miss
rate** — i.e. the LSU cost is hit latency, not misses.

### 9.2 Below L1

**`rv_mem_arbiter`** puts the caches onto one tagged **memory port**
(docs/PLAN-2026-09-25-dcache-vhpr.md, "The memory port"), with three valid/ready channels:
- **request:** `{id, we, line address, 64-bit byte mask, 512-bit line}`;
- **read data:** four 128-bit beats `{id, beat, last, data}`;
- **write done:** `{id}`, raised when the line is in the L2 (below it, on the DRAM's write response).

Any number of transactions may be in flight, and a response is matched by the id its request
carried, never by order. Ids are `{client, slot}`. Among waiting requests a read wins over a
write; the chosen request is registered before it leaves, and responses are routed to their
client in the cycle they arrive. The D$ (client 0) names its own transactions: slots 0-7 its
MSHRs' fills, 8-9 its write-backs, 10 the NC slot, so up to 11 are in flight. The I$ (client 1)
keeps its single-outstanding line handshake behind `rv_mem_line_client`. Page-table walks go
through the D$ (§8), so their misses are D$ fills.

The on-chip SRAM at `LBASE` answers inside `rv_soc_top`. Everything else goes to the L2.

**The L2** (`rv_l2`, 2 MiB; docs/PLAN-2026-10-21-memory-system.md, "The L2") sits on the port with the
same port below it, in the core's clock. 4096 sets of 8 ways of 64-byte lines, keyed by the line's
index in the DRAM window (`dram_axi`, so a wrong-path fetch of PA 0 finds the window's wrap as the
port below does). The data is all 64 UltraRAMs (`smolrv64_uram_be`: 32 K x 512 at {way, set}, a
write enable per byte; port A takes fill beats, port B hit reads, victim reads and write hits). They
are the XCKU5P's one URAM column, the die's full height through the middle of the core, so both
ports' inputs are registered first: read latency 5, the register and the cascade's 4. The tags
(14 bits), valid, dirty and NRU bits are two block RAMs of half a set each.
- **One lookup at a time:** the set is read (LK0), compared (LK1), acted on and written back (ACT).
  A read hit reads its line four times on port B, a beat each: its first beat leaves the L2 eight
  cycles after the request reached it, its last eleven.
- **Misses are non-blocking:** a read miss takes one of 8 MSHRs (its id below) and a victim way --
  invalid, else not recently used, never one a fill holds -- and the request is taken; the fill's
  beats go into the way and up as they arrive (behind a hit's beats in a 16-deep queue when they
  meet). A dirty victim is read into one of 4 write-back buffers. Below, reads go before write-backs.
- **Relaxed inclusive, write-back:** a fill goes into both levels; a victim leaves alone. A write
  hits, or allocates and looks up again to hit -- a whole line (a D$ write-back) with no read below,
  a partial one (an NC store) after fetching its line -- so only a hit or a fill writes the array;
  the write's done goes up when the line is in it. A request for a line a fill or a write-back holds
  repeats its lookup until that is done.
- **Coherent DMA:** a DMA master's line goes to the D$'s outside request, then to the L2 as a
  lookup ahead of any waiting request. Dirty there, it is written back and left clean (a read's) or
  invalid (a write's), and answered when the write's done is back; else answered at once, a flush
  invalidating it. A flushed line stays blocked in the L2, as in the D$, until its DMA write is in
  the controller (`dma_sn_rel`).

Measured in the lockstep (the drawn DDR model, 2026-10-22): the 60 M boot retires +7.61%, the 300 M
+6.98% (over a count from before the DDR model's recalibration); over GB5's single-core section as its 77 SimPoints the cycles fall 2.75% -- Clang 9.5%,
N-Body 5.0%, Machine Learning 4.6%, Gaussian Blur 4.0%, Navigation 3.7%, the rest of the integer
subtests 0.4% to 3.1% -- while AES-XTS, Horizon Detection and HDR, which stream past it, stay within
0.4%: a miss pays the L2's lookup and a register each way, about 5 cycles. The data array's input
register costs 0.2% of that (2.95% at read latency 4).

The L2's misses and write-backs go out on the `ddr_*` port: the testbench's model, or on the board:
- `ddr_port_sync`: into ui_clk as registers, probe_clk being ui_clk / 2 from a BUFGCE_DIV (its edges
  ~2.7 ns late): ui_clk finds the phase from probe_clk's own toggle sampled on its falling edge (CAP
  a register, as it alternates), and captures a request at the ui_clk edge 3.3 ns after the port's
  (a multicycle path of 2 in the XDC), a second ui_clk register on to the controller; everything back
  to the port is launched at the other ui_clk edges, the request's ready being a promise of room made
  an edge ahead, and a read beat or a write done lands in a probe_clk register by the core -- the long
  route is register to register. Its snoop channel runs the other way, a DMA line to clean or flush
  and the release after a flush as toggles launched at the other edges into probe_clk registers, the
  answer's toggle captured at CAP;
- `ddr4_mem`: our DDR4 controller (`ddr4_ctl`) on the DDR4 IP's PHY-only port (AMD PG150), whose
  PHY and calibration train the DRAM before `calDone`. A requester owns its line buffers (this
  port eight for reads and four for writes, `ddr4_port_line`); the PHY's tag is {buffer, half}, so
  a read's two bursts land in its buffer and a write's are taken from it the cycle after
  `wrDataEn`, and a landed line streams back as four 128-bit beats. One queue of eight: column
  commands in queue order (so data comes back in request order), rows opened and closed ahead by
  each bank's oldest request, banks left open. A line lives in one bank (its two bursts at
  columns c and c+8, tCCD_L apart); consecutive lines alternate bank groups; then the bank and
  the row (`line[0]` group, `[6:1]` column, `[25:9]` row, bank `[8:7] ^ row[1:0] ^ row[3:2]`: a stream's
  D$ victim, 128 KiB back, never shares its fill's bank). Refresh every tREFI,
  ZQCS, ODT on writes and the PHY's VT tracking are its own. The DMA masters' AXI port
  (`ddr4_port_axi`: transactions within one line; up to three reads in flight in slots of their
  own -- the controller's buffers 12, 13 and 15 -- snooped, asked and answered in AR order, so a
  line's snoop overlaps the DRAM reads before it, and one write at a time (buffer 14); the
  request to the controller leaves from a register, a read's before the write's; register
  slices on AW/W and R before it; each line snooped in the D$ before it goes, a read's cleaned, a
  write's flushed and released as the controller takes it) goes first. `tb_ddr4_mem` streams line
  reads with a fixed snoop wait: 48.0 `ui_clk` cycles a line at 30 (134 one at a time), 41.7 at
  15, 68.3 at 60; past ~40 the snoop channel, one snoop at a time, is the bound.

The testbench model keeps 8 transactions in flight. Each draws the measured latency at
acceptance, and they are answered in request order; `+ddr_reorder` answers in any order.
`tb_ddr_port` proves the board path (`ddr_port_sync` and `ddr4_mem` across the two clocks, probe_clk
2.7 ns late as on the board, a DMA master beside the core's traffic) on `ddr4_phy_model`, which checks every DRAM rule in DRAM clocks;
`tb_ddr4_mem` drives `ddr4_mem` alone through streams, write-only and idle stretches and row
conflicts. Built with `VDEFS=-DTB_DDR4`, the Linux lockstep puts the board's path below the port
instead of the drawn latencies: `ddr_port_sync` into a ui_clk of twice the core's, `ddr4_mem` and
`ddr4_phy_model`, whose DRAM is `lram` at the controller's map. The 60 M boot runs clean on it,
retiring 60,019,681 (+7.58% over the drawn model's 55,791,913), port reads at 22.3 cycles mean.

**MEASURED ON THE BOARD** (166.67 MHz; `tools/board/membench`, whose `chase2` counts its own
user-mode cycles with the core's counters: a dependent chase over one random cycle of 64-byte
lines, as one chain or as k independent ones; the gate runs the first three on every bitstream):

| | cycles |
|---|---|
| a load to its dependent load, a D$ hit | 3.0 (f644696c, the 4 GiB board; 4.0 on da5d52b3, 5.7 on 2ed18026) |
| the same, a D$ miss whose page the dTLB's first level holds (256 KiB) | 34.0 (the synchronous crossing on our controller, 2026-10-22, the 2 GiB board; 43.0 through asynchronous FIFOs on the same controller, 45.2 on the IP's controller) |
| a first-level dTLB miss that the second level holds (4 MiB) | 6.5 more (the same) |
| a walk on every load (64 MiB) | 73.5 (the same; 87.2 through the FIFOs) |

The asynchronous FIFOs crossed through `CDC_SYNC_STAGES` of synchronizers each way (their clocks
treated as unrelated), 9 of the 43.0 cycles on our controller; the lockstep's two-flop model of
them never showed it. Sequential 8-byte streams (`bw`, 64 MiB) read 3.05 bytes a cycle and write
5.08 on the same build (the IP's controller: 2.48 and 4.82). Independent chains keep 7 to 8 misses in flight -- the 8 MSHRs -- while
the dTLB translates them (256 KiB and 4 MiB, 8 to 12 chains: 5.1 to 7.0 cycles a load); a
footprint that walks on every load keeps 2.4 (32 MiB, 4 to 16 chains: 33.8 cycles a load), the
dTLB's one walker. A sequential stream of 8-byte loads is held by the load queue (§14): scalar
`MemoryBandwidth` reads 7.8 bytes a cycle from the D$ and 2.4 from DRAM (64 MiB), writes 4.0.

### 9.3 Physical memory map

| region | base | size |
|---|---|---|
| CLINT | `0x0200_0000` | 64 KiB |
| PLIC | `0x0c00_0000` | 64 MiB |
| UART (NS16550) | `0x1000_0000` | 8 byte registers |
| ui_clk device pages, one 4 KiB page each: virtio-blk `0x1000_2000`, virtio-net `0x1000_3000`, virtio keyboard and mouse `0x1000_4000` (fed by `tools/hid-bridge` on header J1 pin 4, IO1_P), VGA scanout `0x1000_5000` (not in the DTB; RGB565 at `0xFF00_0000`, the base of the DTB's 16 MiB framebuffer region; its pixel clock comes up at 166.67 MHz, the clock it is timed at, and the monitor sets each mode's over DRP -- 640x480@60 at reset, another from an X11 modeline with its `V` command -- turns it on, and writes the mode into the framebuffer node of the device tree it starts a payload with), the board's keys `0x1000_6000` (`[3:0]` held now, `[11:8]` pressed since the CPU's reset, write 1 to clear; key[0] pressed at the monitor's prompt boots over TFTP (`go`), key[1] steers UART RX); page `0x1000_7000` reads 0 | `0x1000_2000` | 24 KiB |

**virtio-net offers `VIRTIO_NET_F_MAC`**: its config space holds the board's MAC, the FPGA's
96-bit device DNA (`DNA_PORTE2`, read once after the memory controller's reset) folded to 32
bits under the prefix `02:53` (locally administered, unicast). Linux and the monitor both use
it, so a board keeps one address, and one DHCP lease, across boots.

**virtio-net's DMA moves 8-byte words** (2026-09-05, plan item 7): RX gathers a frame one
byte per clock from the engine's slot ring (a registered BRAM read: the backend presents
the byte it consumes NEXT cycle) into a 64-bit word and issues ONE AXI
write per word, strobed at the buffer's head and tail, with the previous word's write in
flight; TX fetches the next frame word while the current one is byte-written into the
engine. Before this every RX byte was its own AXI transaction (1,518 round trips per
1500-byte frame) and every TX word was fetched and then copied in series. `tb_virtio_net`
at the DDR latencies measured on the board (reads 28, writes 15): RX 31,919 -> 3,953
cycles per 1500-byte frame, TX 7,839 -> 6,534 (RX 4,018 with the flag re-read below).
**Each virtio device's DMA port is `dma_master`** (2026-10-24): one command at a time. A
payload's words (blk's sectors, net's frames) are STREAM accesses through its 64-byte line
buffer: a read fills it with one 8-beat burst, writes merge into it under their strobes and
are answered at once, and its bytes go out as one masked 8-beat burst before any command that
is not a stream write to the same line. Everything else (rings, descriptors, status) is one
beat, after the buffer has gone out and with its read line dropped, so a payload is in memory
before its used-ring write is answered. `tb_virtio_net` at the same latencies: TX 6,534 ->
2,631 cycles per 1500-byte frame, RX 4,018 -> 2,469. **The RX engine holds eight 2 KiB slots** (2026-09-05): a
frame is captured only if its first byte finds its slot free, the decision is never
revisited, and every declined frame is counted (busy, FCS-bad, over-long). The single-buffered
engine let the backend's ack land mid-frame and then committed the frame's TAIL as a good
frame: on the board one kernel `rx_dropped` per retransmitted NFS segment (23,347 vs 23,528
over a 117 MB read at 0.52 MB/s), FCS-bad 0. **The interrupt decision re-reads `avail.flags` after
`used.idx` has landed** (both queues): the driver clears NO_INTERRUPT when it re-arms and
then re-checks the used ring, so a flag sampled when the frame started can be stale by
the time the device decides, and a suppressed interrupt then is a lost wakeup the driver
only recovers from by a timeout -- NFS "server not responding" every few minutes on the
board with bitstream O. `tb_virtio_net` toggles the flag mid-delivery both ways.
| on-chip SRAM (boot/monitor) | `0x7000_0000` | 256 KiB |
| DDR4 | `0x8000_0000` | 2 or 4 GiB (§13, DDR4) |

UART is **3 Mbps, hardwired in RTL** — not derived from the DTS or the kernel printout.
Receive goes through a **4 KiB FIFO** behind RBR, 13.7 ms of input: the line has no flow
control, so a byte that finds the FIFO full is dropped and sets LSR.OE. The THR is one byte
(`fifo-size = <1>` in the DTS).

---

## 10. Array inventory

Every array in the core, with the shape the RTL actually declares. Sizes are for the
shipping configuration (`SIZE_KB`=128, `SMOLRV64_HW`=8, `PAW`=64 into the caches).

### 10.1 Core

| array | module | shape | width | bits | storage | ports |
|---|---|---|---|---|---|---|
| `ish[0..IW-1].mem` | `smolrv64_prf` | 64 each | 64 | 16 384 | LUTRAM | lane k's integer shard: 1W (lane k's write register), 9R at IW=4 (2 per lane, the store queue's) |
| `lb_prd`, `lb_rob`, `lb_fe`, `lb_dk` | `smolrv64_core` | 3×4 | 10/8/1/1 | 240 | flops | the lanes' landing buffers (§8): what the kill and the wakes read at every entry |
| `g_lp[*].u_lbd` `bank[*].mem` | `smolrv64_bankq` (in `smolrv64_core`) | 4 × 2 × 2 | 64 | 1 024 | LUTRAM | the landing buffers' data: the LD stream's at the tail, the FE stream's after it, read at the head |
| `fp[0].mem`, `fp[1].mem` | `smolrv64_prf` | 64 each | 64 | 8 192 | LUTRAM | 4R (the store queue's, the FP port's 3), 1W each: SH_FL (the load landing) and SH_FF (the F stage) |
| `cp[0..IW-1].smap` | `smolrv64_rename` | 64 each, twice at IW=4 (`smap_d`) | 9 | 4 608 | LUTRAM | one copy per rename slot (1W each: the slot's rename, the walk's column, the sweep's lane; `newer` picks), 5*IW reads (the sources and destinations, the commits' dirty tests) |
| `cp[0..IW-1].rmap` | `smolrv64_rename` | 64 each | 9 | 2 304 | LUTRAM | one copy per commit port (1W each, `rnewer` picks), 2*IW reads (the commits' displaced mappings, the sweep) |
| `dirty` | `smolrv64_rename` | 64 | 1 | 64 | flops | the registers the sweep rewrites after a squash |
| `fl[*].u.bank[*].mem` | `smolrv64_rename` | 64 per shard | 6 | 384 per shard | LUTRAM | one free list per shard in use (lanes 0..IW-1, SH_FL, SH_FF), `next_pow2(IW)` banks each |
| `col[0..IW-1].mem` | `smolrv64_rob` | 64 each | 25 | 6 400 | LUTRAM | column k, lane k's entries `{pold, noret, rd, prd}`: 1W (lane k's dispatch), 2R (the head's row, the walk's row; the ports rotate them into program order) |
| `fc` | `smolrv64_rob` | 64 | 2 | 128 | LUTRAM | each row's first column: 1W (dispatch), read at the next head row, `irr`'s, the walk's and the walked-to branch's rows (`hfc`, the head row's, is a flop) |
| *(two-wide dispatch and retire)* | | | | | | items 10b/10c: on the board since 2026-09-06 (gate W2F, 319cd248), after four silent bitstreams whose renamer Vivado had folded (docs/rtl-rules.md F4/F5) |
| `v`, `done` | `smolrv64_rob` | 256 | 1 each | 512 | flops | by storage index; `v` bulk-cleared, `done` cleared at allocation |
| `head`, `tail`, `irr` | `smolrv64_rob` | 1 each | 9 | 27 | flops | positions `{ring wrap, row, p}`; `irr` the irrevocable pointer (§6) |
| `u_iq_i` entry | `smolrv64_iq` | 10 | 2+2×9 = 20 | 200 | flops | integer, slot A's, `NSRC`=2 (§6.1) |
| `u_iq_i2`, `u_iq_i3` entry | `smolrv64_iq` | 10 each | 2+2×10 = 22 | 220 each | flops | integer, slot B's and slot C's, `NSRC`=2 (§6.1) |
| `ho_va`, `ho_dat`, `ho_rec`, `ho_v` | `smolrv64_core` | 1 | 64/64/282/1 | 411 | flops | the head-op register (§8) |
| `u_tq` `bank[*].mem` | `smolrv64_bankq` (in `smolrv64_core`) | 4 x 8 | 117 | 3 744 | LUTRAM | the training queue's payload (a CTI's PC, RVC bit, taken-target, predict details, kind and outcome), written a cycle late from registers (`WLATE`: the head's read bypasses them): one write a bank (a lane's CTI at the tail plus its rank), read at the head into `res_*_q` (§4.2) |
| `tq_seq`, `tq_rob`, `tq_dk`, `tq_rt` | `smolrv64_core` | 32 | 9/9/1/1 | 640 | flops | the training queue's entries as every one is compared: the redirect's seq, the retire ports' and the kill's ROB index |
| `rob_ffr` | `smolrv64_core` | 256 | 5 | 1 280 | LUTRAM | an FP op's exception flags by ROB slot: 1W (the landing result), 1R per retire port |
| `rob_ffv` | `smolrv64_core` | 256 | 1 | 256 | flops | the slot's FP result landed: cleared by its allocation, set by the landing |
| `mdw` | `smolrv64_core` | 256 | 2 | 512 | LUTRAM | the memory-dependence wait table: read at dispatch by the load's PC and by training, 1W (a replay, else one training event, else the forget's step; §8) |
| `spl_ld`, `spl_st` | `smolrv64_core` | 16, 32 | 8+274 = 282 | 13 536 | LUTRAM | a lane load's or store's record by LQ/SQ entry: four banks by entry, 1W each (a group's row), 1R at M's pick (§8) |
| `u_la`, `u_sa` `wb[0..IW-1].mem` | `smolrv64_mwram` (in `smolrv64_core`) | 16, 32 a bank | 64 | 9 216 | LUTRAM | the lanes' arrived VAs, a bank per lane (1W, its lane), 1R at M's pick; `lvt` (2 bits an entry, flops) names the bank that wrote the entry last (§8) |
| `la_v`, `sa_v` | `smolrv64_core` | 16, 32 | 1 | 48 | flops | an entry's address has arrived and M has not taken it |
| `u_iq_f` entry | `smolrv64_iq` | 8 | 2+3×9 = 29 | 232 | flops | FP, reorders, `NSRC`=3 (§6.1) |
| `plmem` (payload) | `smolrv64_core` | 38 (10+10+10+8) | 274 | 10 412 | LUTRAM | one array per scheduler: 1W dispatch, 1R issue each; the immediate is stored as its low 32 bits, which it sign-extends (the queue's write asserts it), and a fetch fault's address as one bit, set for a straddler's pc+2 (`imm` and `fault_tval` are rebuilt from them) |
| `ipend`, `fpend` | `smolrv64_pending` | 256, 128 | 1 | 384 | flops | readiness, a table per file indexed by {shard, index}: 23R, 16 that read both and let the number's file pick (the slots' rs1/rs2, 8; the lanes' operands, 8) and 7 of the FP table alone (the slots' rs3, 4; the F FIFO head's sources, 3), and each writeback its own register (the integrity check); 4 sets, 6 clears into the integer table (the lanes', the streams' integer landings) and 2 into the FP (the streams'), bulk-clear |
| `u_pa` `wb[0..1].mem` | `smolrv64_mwram` (in `smolrv64_lq`) | 16 a bank | 33 (`PABITS`) | 1 056 | LUTRAM | the load queue's PA, a bank per writer (M's fill, the walker's answer), 3R (the start's entry, the landing's two candidates); `lvt` (a bit an entry, flops) names the writer |
| `pcm` | `smolrv64_lq` | 16 | 39 | 624 | LUTRAM | a load's PC: four banks by entry, 1W each at dispatch a cycle late from registers (`WLATE`, the read bypasses them), 1R the fault pick; `pch` (16 × 8 flops) keeps its wait-table bits, which every entry reports |
| `om` | `smolrv64_lq` | 16 | 16 | 256 | flops | the load queue's program order: an entry's row names the entries older than it, written at its allocation (the row the live entries, its column cleared); every "oldest of these" reads it |
| `data` | `smolrv64_sq` | 32 | 64 | 2 048 | LUTRAM | the store queue's bytes: 1W (the landing, one entry a cycle), 2R (the handoff entry, a forward's store; the lockstep's record of the committing entry is a third, in simulation only) |
| `addr_m`, `addr_w` | `smolrv64_sq` | 32 each | 33 (`PABITS`) | 2 112 | LUTRAM | the store queue's PA: 1W each (M's fill, the walker's answer), 1R (the handoff entry; the lockstep's record a second, in simulation only); `addr_lv`, a bit per entry in flops, picks the copy |
| `u_q` `bank[*].mem` | `smolrv64_bankq` (in `smolrv64_frontend`) | 8 x 2 | 179 | 2 864 | LUTRAM | a decoder a bank: the bundle register's slot landing in the bank is rotated to it raw and decoded there, so each bank writes its own decoder's record (`BYBANK`); the heads read it twice, in program order for the group rules and in lane order for the IR. The decoupling queue holds what decode cannot cheaply recompute (`QR_REC`, `QR_RECW = 2*PCW+PDW+SEQW+58`): the PC and the predicted next PC, the predict detail, the seq, the instruction expanded and its RVC bit, the fault, the illegal and static-mispredict verdicts, and the dispatch class `gc`; each lane's head decoder rebuilds the IR record from it, expanding nothing |

Each PRF shard now has **its own write address** (`wa_ie`/`wa_ld`/`wa_fe`), which is §7 of
`Area-Efficient-Scalar-OoO.md`'s "give each file its own port and a single writer and the
arbiter disappears". Half of that holds: IE takes only the ALU/CSR result and FE only the
FPU, but **LD has two writers** — a landing load and M's mul/div — so LD still needs an
arbiter, or mul/div needs its own shard. It costs nothing today because `m_done` is forced
low on `ld_land`/`fp_land`, and an assertion fires the moment that stops being true.
(Resolved by C1, 2026-09-17: mul/div write FE from the MD stage; LD's writers are the
landing load and M's CSR result, and FE's are the FPU, the CTF link and the MD stage, muxed
before the shard with M yielding.)

The **ROB completion ports** are `NW` = 2 x `NL` + 5 (13 at IW=4): M's, the FPU's landing, the store
queue's commit, the MD stage's, the restart's, each lane's own (its column only), and each lane's CTI
port, a cycle after the CTI executes (a CTI's compare and target decide whether it completes now or
at its squash, so they stay out of every entry's done). The only yield left is M's, to a landing
load or FP result.

### 10.2 Front end

| array | module | shape | width | bits | storage |
|---|---|---|---|---|---|
| `bank[b].way[w].btb` | `smolrv64_predictor` | 8 x 2 x 128 | 53 | 108 544 | block RAM, 16 RAMB36 (SDP, 128 x 53 of 512 x 72): 1R the stream's (its read is the predictor's loop), 1W training's |
| `bank[b].yc` | `smolrv64_predictor` | 8 x 256 | 10 | 20 480 | distributed RAM, read as the stream moves |
| `ras` | `smolrv64_predictor` | 8 | 40 | 320 | flops |
| `bank[b].way[w].bc` | `smolrv64_predictor` | 8 x 2 x 128 | 2 | 4 096 | distributed RAM, read as the stream moves and by training |
| `bank[b].way[w].nu` | `smolrv64_predictor` | 8 x 2 x 128 | 1 | 2 048 | distributed RAM, the recently-trained bit, read and written by training |
| `pq0`, `pq1` | `smolrv64_predictor` | 2 x 4 | 72 | 576 | the prediction queue: even and odd positions, one write each |
| `u_rg` `bank[*].mem` | `smolrv64_bankq` (in `smolrv64_fring`) | 8 x 4 | 17 | 544 | LUTRAM (the halfword, its start bit): one write a bank (an answer's halfword landing there), the window's eight read |
| `rmk` | `smolrv64_fring` | 32 | 1 | 32 | flops (the halfword ends a pair at a known CTI; fetch clears the head's) |

Neither BTB nor corrector bank has a valid bit — validity is the tag match (§4.2).

### 10.3 Memory system

The D$ (`rv_dcache`, 128 KiB; out of context 9.1 k logic LUTs, 2.6 k LUTs of distributed RAM, 2.9 k
flops, 32 RAMB36, WNS +0.050 ns at 6 ns; `make ooc MODULE=rv_dcache GENERICS=RTW=6`):

| array | shape | width | bits | storage |
|---|---|---|---|---|
| data | 32 × 512 | 64 | 1 048 576 | **BRAM** (`smolrv64_sdpram_be`, 1R1W, a write enable per byte), diagonal in groups of eight |
| `ta` / `tb` | 32 × 64 | 21 (`PABITS - 12`) | 86 016 | LUTRAM: each way's tags, read at the set -- the main lookup's copy and the store lookup's |
| `va` / `da` / `ua` | 32 × 64 | 1 | 6 144 | LUTRAM: valid, dirty, not recently used, each written by one statement |
| merge buffers | 2 × 32 | 64 + 8 | 4 608 | LUTRAM, even and odd chunks, one write a cycle; a chunk-valid bit and a zero bit per MSHR in flops |
| MSHRs | 8 | -- | ~800 | flops |
| waiters | 20 | ~70 | ~1 400 | flops and the PA in LUTRAM: one per requester tag in use (`NRQ` = 19: the load queue's 16, the two walkers', the slow path's), one for the store |
| store queue | 3 | PA, data, mask | ~400 | LUTRAM, its flags in flops |
| write-back lines | 2 | 512 | 1 024 | flops |

The L2 (`rv_l2`, 2 MiB; out of context 1.5 k logic LUTs, 0.38 k LUTs of distributed RAM, 1.7 k flops,
16 RAMB36, 64 URAM288, WNS +1.087 ns at 6 ns; `make ooc MODULE=rv_l2`):

| array | shape | width | bits | storage |
|---|---|---|---|---|
| data | 32 K × 512 (4096 sets × 8 ways) | 512 | 16 777 216 | **UltraRAM**, all 64 (`smolrv64_uram_be`: port A the fills' beats, port B reads and write hits; a write enable per byte; inputs registered, read latency 5) |
| tags | 2 × 4096 × 68 | four ways' {valid, dirty, 14-bit tag} and their NRU bits | 557 056 | BRAM (`smolrv64_sdpram`), a set's two halves read together |
| write-back lines | 4 × 512 | 512 | 2 048 | LUTRAM, written by the victim's read |
| fill-beat queue | 16 × 136 | {id, beat, last, data} | 2 176 | LUTRAM |
| MSHRs | 8 | line 26, way 3, id 5, 3 flags | 296 | flops |

The I$ (`rv_icache`, 128 KiB): data 8 RAMs of `2048 × 64` (1 048 576 bits, 32 RAMB36): per way an
even and an odd 16-byte bank, each two RAMs on one row address (§9.1). Each way's
`{virtual tag, epoch}` (`tg0`, `tg1`: `1024 × 31`) is one block RAM read with the banks, at the
same set, so a lookup's tags arrive in s1 beside its data; its second port is the fill's (the
write, and the other way's read for the duplicate-stamp check), and in every cycle the banks read
it reads the next line's set. The physical tags and validity
(`pst[*]`: 32 × 64 rows of 27 + 2 bits) are LUTRAM, read at the lookup's, the fill's and the
probe's rows in one cycle.

**The bank is never wider than 64 bits, whatever the port width.** The I$ at `HW=8` reads
128 bits: the even/odd chunk pair at a 16-byte-aligned address, which is the 2*BANKW window
the cache already assembles, delivered without a shift (an unaligned 128-bit read is an
always-on `$fatal`). The old 4-wide I$ widened the BANK to 128 and fetched garbage on real
BRAM (the width-cascade geometry no simulation models); `smolrv64_sdpram`'s guard still
refuses that, and this never reaches it. `febench` (straight-line 32-bit code): 0.66 ->
0.98 aligned, 0.40 -> 0.79 at a 2-byte offset, at `HW=4` -> `HW=8` (2026-09-05); the 2-byte
offset then reached 0.98 with the pre-VHPR run-ahead buffer's slide-cycle request. On the VHPR
I$ + alignment adapter (43fbcfce, 2026-09-14, re-baselined for Stage 2 inc 4) `febench` reads
aligned 1.28, 2-byte-offset 0.98, compressed 1.95, mixed 1.40 -- the adapter MATCHES the buffer
on straight-line throughput (0.98 at the 2-byte offset); its reduced run-ahead shows on
taken-branch refetch (the mispredict table, §3), not straight-line fetch.

**The D$'s physical tag is `PABITS - 12` = 21 bits, not the port width's 52.** The ports are 64
wide because a PA rides in a 64-bit bus, but a PA has `PABITS` = 33 significant bits (the bits
of the DRAM's top, `smolrv64_width.vh`), and a write-back's address is rebuilt from the
tag and the row.

**The physical-address cap.** DRAM starts at `0x8000_0000` and is `2^DRAM_LG2` bytes, a
configuration of the instance: 32 on the board (4 GiB, the DMA masters' window,
`src/dram_window.vh`; a board DTS describes 2 or 4 GiB of it, §13, and `src/lint.sh` checks its
memory nodes against the cap with `tools/check-dts-memory.py`), the modeled DDR's size under
cosim. A PA
at or above `DRAM_TOP` = `0x8000_0000 + 2^DRAM_LG2` does not exist and takes an access fault
(cause 1/5/7), so no access to it reaches a cache or the bus; every device lies below DRAM, and
the SoC's decode owns that hole. `PABITS` is the bits of `DRAM_TOP`. Both MMUs enforce it
where a PA is created, never on the TLB hit path: Bare mode compares the address beside the
non-canonical test; the walker faults a table pointer or a leaf access at or above the top, and
installs a leaf only when its whole page lies below it, so no TLB entry names a PA that does
not exist (a page reaching past the top answers its own access and is not cached).

Address translation, two instances (iTLB in `smolrv64_core`, dTLB in `smolrv64_lsu`):

| array | shape | width | bits |
|---|---|---|---|
| iTLB `tlb_g` ({installed, 16-bit generation})/`tag`/`ppn`/`lvl`/`perm`/`nc`/`n`/`dram` | 64 | 17+21+21+2+8+1+1+1 = 72 | 4 608 |
| dTLB, the same fields | 64, read by two ports (M's lookup, the walker's), so the LUTRAM is held twice | 72 | 4 608 per copy |
| second level `tlb2`, block RAM: the fields but `dram` | iTLB 512, dTLB 2048 | 17+18+21+2+8+1+1 = 68, 17+16+21+2+8+1+1 = 66 | 34 816 + 135 168 |

A tag is the VPN above the entry's index (`VPN[26:TLBI]`, `VPN[26:L2I]`): the index bits are the
entry's own.

Also in `rv_soc_top`: `lmem` — the 256 KiB on-chip boot/monitor SRAM, `NLLINE × 512`; and
`u_uart_rxq` — the UART's receive FIFO, 4096 × 8, one block RAM.

### 10.4 Verification-only

Not present in synthesis (`ifndef SYNTHESIS`), listed so nobody counts them as area:

| array | shape | width | purpose |
|---|---|---|---|
| `cs_pc` | 256 (`ROB_DEPTH`) | 40 | retire PC for the cosim trace |
| `cs_insn` | 256 | 32 | retire instruction word |
| `cs_val` | 256 | 64 | retire value — captured at the writeback event |
| `cs_mkind` | 256 | 2 | memory-effect kind |
| `cs_mpa` | 256 | 33 (`PABITS`) | memory-effect physical address |

---

## 11. Counters and observability

`Zihpm` with **13** programmable counters (`mhpmcounter3..15`), driven by a registered 44-bit
`hpm_ev` bus, four per-cycle counts (the queue occupancies, the fast loads in flight and
`DPATCH`) and the registered top-down cause `td_k`. A 13-event `perf stat` fills the counters;
more is multiplexed, and its counts are estimates.

**Sscofpmf (sampling).** Each `mhpmeventN` carries OF (bit 63) and MINH/SINH/UINH (62:60);
VSINH/VUINH (59:58) are stored and never consulted. A counter stops in a privilege whose INH bit
is set, and also under `mcountinhibit`. A counter that wraps sets OF, and OF's 0 -> 1 edge sets
`mip.LCOFIP` (bit 13): while OF stays set the counter keeps counting and raises nothing more,
until software clears OF. LCOFI is delegable (`mideleg[13]`), `sie`/`sip` show bit 13, and it
ranks last, after STI. `scountovf` (0xDA0) reads the OF bits; below M a bit reads 0 unless
`mcounteren` grants that counter. Linux's SBI PMU driver samples on it, so `perf record -g`
works (the kernel keeps frame pointers). The privilege filter compares the current privilege
with the event bus, which runs two cycles late, so an event in the cycle of a privilege change
may count on either side of it. A write of `mhpmeventN` selects its event from the stored
register a cycle later, and the counter holds for that cycle; a write of a counter (`mcycle`,
`minstret`, `mhpmcounterN`) loads a cycle late, over that cycle's increment, and a read (`csrr`)
of one writes nothing. A read of `mhpmeventN` or
`mhpmcounterN` (`csrr`) writes nothing, so it cannot swallow that cycle's increment or overflow. `core/directed/sscofpmf.S`
checks overflow, OF's silence, the INH bits in M and S, LCOFI's priority and delegation, and
`scountovf`'s masking and U-mode trap.

Unimplemented CSRs read 0 and ignore writes instead of trapping, so OpenSBI's probe also
reports Smcntrpmf (`mcyclecfg`/`minstretcfg`) and Sdtrig (one trigger): neither exists, and
`mcycle`/`minstret` are never filtered. With Sscofpmf present OpenSBI places cycles and
instructions on programmable counters, which do filter.

**TOP-DOWN (§11.2, on the board).** `smolrv64_core`'s one classifier (`td_k`) charges every cycle to
exactly one of bad speculation, front-end, back-end or dispatching; its events count it, so
`TD_BS + TD_FE + TD_BE` plus the dispatching cycles is the cycle count by construction, and the
simulator's `TOPDOWN-SIM` prints the same classifier. A cycle whose IR group dispatches is a
dispatching cycle whatever waits behind it (the queue head's credit is the next group's), so the
dispatching cycles' instructions are exactly `DPATCH`. `perf-smol td CMD` prints the report: level 1, the depth under each parent, and the slots; the
`tdbe` and `tdfe` sets add level 3, `td_k` itself.

| event | id | meaning |
|---|---|---|
| `TD_BS` | r0401 | bad speculation: a redirect, or a resolved restart waiting |
| `TD_FE` | r0407 | front-end: the IR is empty, the queue head waits for no credit, M is free, not bad speculation |
| `TD_BE` | r0402 | back-end, unless the IR's group dispatches: M holds a head op, the queue head waits for a credit (`hd_wait`, §3.2), the IR holds its group, or the refill cycle after one |
| `TD_BE_MEM` / `TD_BE_ROB` / `TD_BE_IQ` | r0403 / r0404 / r0405 | within `TD_BE`: memory (td_k 10, 13: M holds a head op, or issue waits on an integer load, AMO or CSR read) / no ROB room (12) / no scheduler, store-queue or load-queue credit (15, 17, 18) |
| `TD_FE_LAT` | r0406 | within `TD_FE`: the iMMU, or no fetch bytes (td_k 5, 6) |
| `DPATCH` | r0408 | instructions dispatched, 0..IW a cycle: the slots against IW × cycles |
| `TDK_*` | r05nn | the cycles with td_k = nn, one event per cause below |

| td_k | `TDK_` | TOPDOWN-SIM | the cycle |
|---|---|---|---|
| 0 / 1 / 2 / 22 | `OK1` / `OK2` / `OK3` / `OK4` | ok:1 .. ok:4 | the IR's group dispatches 1 / 2 / 3 / 4 instructions (4: IW=4) |
| 3 / 4 | `REDIRECT` / `DRAIN` | bs:redirect / bs:drain | a redirect / a resolved mispredict's restart (`RD_WAIT`) |
| 5 / 6 / 7 / 8 | `FE_IMMU` / `FE_ICACHE` / `FE_ALIGN` / `FE_QUEUE` | fe:immu / fe:icache / fe:align / fe:queue | the iMMU cannot translate / no fetch bytes / bytes but no whole instruction / an instruction not yet in the IR |
| 10 | `BE_MMEM` | be:M-mem | M holds a head op (AMO, LR/SC, CBO) |
| 12 | `BE_ROB` | be:rob-full | no ROB row for the head's group |
| 13 / 14 | `BE_DEPLD` / `BE_DEPFP` | be:dep-load / be:dep-fp | no lane issued while a scheduler's reported entry waits on an integer load, AMO or CSR read / on an FP op's, FP load's, FP-port or divide result |
| 15 | `BE_IQ` | be:iq-full | the head's scheduler credit: its lane's (`hd_lane`), the FP queue's, the head-op register's or the divide's |
| 16 / 17 / 18 | `BE_RN` / `BE_SQ` / `BE_LQ` | be:rename / be:sq-full / be:lq-full | a rename shard below `LOWAT` or the map's sweep / no store-queue credit / no load-queue credit |
| 19 | `BE_SER` | be:serialize | a serialising op drains or is in flight, a CSR op waits for the one before it, a CBO waits for older addresses |
| 20 | `BE_HELD` | be:held | the IR holds its group otherwise: the cycle after a redirect or a decode resteer |

td_k 9 (fe:other), 11 (M on a non-memory op) and 21 (a dispatching cycle with nothing
dispatched) cannot happen, and the core asserts it: FE_MMU..FE_QUE cover the empty IR, M takes
only memory ops, and a dispatching cycle has the IR's group taken.

The wait-cycle events below overlap (a cycle waits on several things at once) and are depth,
never a partition. `docs/smolrv64-perf-events.json`, generated from `src/csr_file.v`'s event map,
has every event's exact description; this is the map by family:

| event | id | meaning |
|---|---|---|
| `ST_MEM` | r0300 | M holds an unfinished head op **+** no lane issued while a scheduler's reported entry waits on an integer load, AMO or CSR read (`dep_ld`) |
| `ST_FPU` | r0303 | stage F holds an op it cannot advance **+** no lane issued while a reported entry waits on an FP, FP-load, FP-port or divide result (`dep_fp`) |
| `ST_DSP` | r0304 | the queue head waits for a credit other than the ROB's, while M can advance and no dependency holds issue: `ST_IQ + ST_RN + ST_SQ + ST_LQ + ST_SRZ` |
| `ST_ROB` | r0305 | the queue head waits for ROB room |
| `ST_IQ` / `ST_RN` / `ST_SQ` / `ST_LQ` / `ST_SRZ` | r0306 / r0307 / r0308 / r0309 / r030a | `ST_DSP` by cause, disjoint, in this order: the head's lane's scheduler, the FP queue, the head-op register or the divide credit / a rename shard below `LOWAT` or the map's sweep / a store with no store-queue credit / a load with no load-queue credit (neither has one while a head op is in flight or in the IR) / serialising, a CSR op or a CBO waiting |
| `FE_BUB` | r0310 | the IR is empty, the head waits for no credit, M is free, no redirect is live and no back-end refill is due: `FE_MMU + FE_IC + FE_ALN + FE_QUE` |
| `FE_MMU` / `FE_IC` / `FE_ALN` / `FE_QUE` | r0311 / r0312 / r0313 / r0314 | within `FE_BUB`: the iMMU cannot translate (a second-level lookup, a walk, or a walk held behind a weak branch) / no fetch bytes / bytes but no whole instruction / an instruction not yet in the IR |
| `RD_WAIT` | r0317 | a mispredict's restart: from its resolve to its release (the walk and the kill, §6) or its squash at the ROB head |
| `ST_MUL` / `ST_DIV` | r0302 / r0301 | occupancy, not stalls: a multiply in flight in any lane / the MD stage holds a divide |
| D$ requests / fills | r0100 / r0102 | requests taken (a read at the door, loads and page-table-walk reads, or a write into the store queue) / line fills started (an MSHR allocated; a join of a fill in flight is not one) |
| I$ requests / misses | r0110 / r0112 | requests taken, one 16-byte pair each / requests whose lookup misses the virtual tag |
| `DT_WALK` / `DTLB_MISS` | r0318 / r0104 | the dMMU walker busy (a second-level lookup or a walk, for a queued load or store or M's head op) / walker starts (a request the first level does not hold). The dTLB is 64 entries direct-mapped on VPN[5:0] and its second level 2048 on VPN[10:0]; two hot pages 8 MiB apart share a second-level index and cost a walk per load and no D$ miss |
| redirects | r0005 / r0006 / r0007 / r0008 | restarts (a squash at the head, a mispredict's release) / conditional branches / jalr / exceptions, interrupts, faults and memory-ordering replays (xRET, sfence.vma, CSR-write redirects and fence.i are restarts of neither kind) |
| `LOAD` / `STORE` | r0003 / r0004 | load landings (fast, slow or forwarded; wrong-path and replayed landings included; AMO and LR not) / stores the store queue hands the LSU after commit |
| `MEM_HITSER` | r0319 | the LQ has a ready candidate the LSU port does not take |
| `MEM_LDINFL` / `MEM_LDSENT` | r031a / r0323 | fast-path loads in flight (tags whose D$ response or store-queue forward is due), summed per cycle / cycles with a fast-path load start (the port's or M's; two in one cycle count once, and a cancelled start counts): their ratio is the mean fast-load latency in cycles, a hit's and a miss's together |
| `MEM_STDOOR` | r031b | a write at the D$ door that has no room and no device acknowledge |
| `MEM_ALIAS_UNK` / `MEM_ALIAS_OVL` | r031c / r031d | an LQ candidate is alias-blocked, while a blocked load with its wait bit set waits on an older store's unknown address / and none does |
| `MEM_REORD` | r031e | loads the port takes with an older store, committed or not, live in the store queue (count) |
| `MEM_WPKILL` | r031f | load landings dropped by a kill that came while the access ran, fast or slow path (count) |
| `MEM_DEVWAIT` | r0320 | no candidate is ready and an unblocked device load waits to be the ROB head |
| `MEM_LQOCC` / `MEM_SQOCC` | r0321 / r0322 | load / store queue occupancy summed per cycle (mean = count / cycles) |

`ST_MEM` and `ST_FPU` include the *dependent* wait, charged to the unit that owns the register
being waited on; the events OR their parts, so a consumer waiting on both a load and an FP
result counts in both, and only `td_k` ranks them (dep-load before dep-fp). The schedulers'
"blocked on" reports are registered before they are classified, so the dependent wait is the one
the schedulers reported the cycle before; each scheduler reports its lowest-slot waiting entry
(it compares no age).

**Reading the report.** On coffee, `tools/perf-smol-install.sh` installs `perf-smol`
(`tools/perf-smol.sh`), `perf-cpi-stack.py` and the event table together into the boards' shared
home (`~/bin`, on PATH at login); `perf-cpi-stack.py` refuses a table older than itself. On the
board (once, `tools/perf-smol-setup.sh` lowers `perf_event_paranoid` so no `sudo` is needed),
each run prints its report on stderr when the command ends and keeps the counts in
`perf-smol.<set>.txt`:

    perf-smol td     CMD > /dev/null    # where the cycles go
    perf-smol tdbe   CMD > /dev/null    # the back-end's causes
    perf-smol tdfe   CMD > /dev/null    # bad speculation, front-end, width
    perf-smol cpi    CMD > /dev/null    # the wait events, in depth
    perf-smol memcpi CMD > /dev/null    # the memory events, fast-load latency

Each set is one run of 13 counters; never ask for more (`all`), which multiplexes. `perf stat`
counts the whole command, so the answer is about the whole command: run the region of interest
in a loop long enough to dominate start-up, or read it with a different window and say so. For
Geekbench, `IV=1000 tools/board/gb5-run.sh <rtl>` counts each second as well (+0.12% cycles), and
`tools/gb5-subtests.py <perf.txt> <ts.log>` sums the seconds into each subtest.

The `td` report, top to bottom:

- **level 1** partitions the cycles: *dispatching* (the IR's group dispatched), *bad
  speculation*, *front-end*, *back-end*. It sums to 100% by construction; if it does not, the
  event map is wrong and the script stops.
- **bad speculation** splits into the redirect itself and *a resolved restart waiting*
  (`RD_WAIT`). The per-1k redirect counts beneath say which kind: conditional branches and jalr
  point at the predictor, exceptions and replays at the program and the memory speculation.
- **front-end**: the IR was empty with nothing waiting behind it. *latency* is the iMMU or no
  fetch bytes (the I$; see `I$ misses` per 1k); the rest is alignment and the queue refilling
  after a redirect (`tdfe`).
- **back-end**: *memory* is M holding a head op or a scheduler waiting on an integer load
  (`memcpi` next); *the ROB is full* means something long-latency at the head with nothing behind
  it to retire; *a scheduler or load/store queue is full* means one class saturated; `tdbe` gives
  every cause, the rest (rename, serialisation, FP waits, the IR held after a redirect) included.
- **slots** are IW per cycle: *retired* is the IPC as a share, and *dispatched, squashed* is
  the wrong-path work.

The `cpi` set's wait-cycle rows overlap (a cycle waiting on a load and on ROB room counts in
both), so they are depth under the `td` buckets, not a second partition; its *unattributed*
line is what no event names. The simulator's `TOPDOWN-SIM` prints the same classifier for a
whole simulated run, and `workloads/rvbench` (`make run B=<name>`) runs the bare-metal programs
there; before reasoning from a surprising board number, look for the same shape in simulation.
The board's reference run is `sha256sum` on `/usr/bin/emacs-nox`, five times: at 1c4ff06f (the
lanes' multipliers, 5.2c) it dispatches 66.3% of cycles, IPC 2.07, with the back-end at 21.3%
(no ROB room 14.8%, memory 4.4%), the front-end at 11.9% and bad speculation at 0.5%.

### 11.1 The integrity log (`rv_errlog`, 2026-09-20)

The memory backend's invariants, latched in hardware. Rule A1 makes every check always-on in
simulation; on hardware `$fatal` is a no-op and synthesis deletes the condition, so until this
existed the bitstream enforced none of them — and the longest simulation this project runs
(1.5 G cycles) is two and a half orders of magnitude short of a Geekbench run (the GB5 crash of
C4a step 2 came after ~290 G cycles, the GB6 crash of 390d5028+ after 831 G). Each unit names
its conditions once (`e_*` wires read by both the `$fatal` and the log, so the message and the
detector cannot drift), registers them (`err_q`, one flop per bit: nothing crosses a hierarchy
boundary combinationally, nothing lands on a lookup path), and `rv_soc_top`'s single
`rv_errlog` keeps a sticky bit per invariant plus the index and 48-bit cycle stamp of the first
one to fire. The I$'s parity array stays opt-in (`-DCACHE_PARITY`); the D$ has none.

| bits | unit | source of the numbering |
|---|---|---|
| `[15:0]` | D$ | `rv_dcache.v`, invariants block (16 conditions: an orphan fill beat, a response nothing awaits, a write completion nothing sent, beats out of order, two valid ways holding one line in either lookup, a tag reused while outstanding, a waiter on a dead MSHR, a beat into a line being read out, a store in both lookups or a plain store that hits or joins in the main lookup, a victim an MSHR reserves or the store lookup writes, a store sent back or the head issued as the head leaves, a store taken while a clean is asked for or under way, M's fast load misused, an outside flush finishing on a blocked line or a release with none, an MSHR's read issued while the MSHR is not live or a second time) |
| `[31:16]` | I$ | `rv_icache.v`, invariants block (11 conditions: a line hitting in both ways, an unrequested L2 answer, a window not halfword aligned, a full skid, VA/PA page-offset mismatch, a demand read and a prefetch both outstanding, a duplicate stamp, one physical line in both ways, a tag write in a bank read's cycle, a window across a page, a lookup beside the next line's reconcile probe) |
| `[47:32]` | LSU | `smolrv64_lsu.v` (8 conditions: the three B-rule tag checks, the two non-DRAM checks, `pt_ld_done` equivalence, `req_early`, an AMO or CBO off the ROB head), and at `[47:45]` the scheduler's pending table (2) and the FPU (1) |
| `[63:48]` | frontend | `smolrv64_frontend.v` (14 conditions: the fetch ring's, the predictor's -- among them a BTB correction that does not converge -- and fetch's) |

Read-only MMIO, 64-bit words, in the window the deleted fetch-buffer diagnostic owned so no
comparator was added to the `dmem_raddr → is_dev_r` cone (a load-path critical cone): `0x1000_E000`
`{version, "ERRL"}`, `+0x08` the sticky vector, `+0x10` `{idx[55:48], cycle[47:0]}` of the first
fault. Not in the DTB. Readers: the ROM monitor prints `err=` in its banner and `E` decodes it;
`tools/errlog-read.sh <ip>` reads it from Linux through `/dev/mem` and names the bits; the board
gate reads it after the userspace stress and FAILs on a nonzero vector whatever the console
says. The two things a nonzero word buys: *which* rule broke and *when* — and a zero word after a
crash exonerates the whole logged set in one read. Simulation keeps the log honest: every bench
`$fatal`s if a bit ever rises, which in a passing run can only mean a condition is wired wrong
(its own `$fatal` would have fired a cycle earlier).

### 11.2 Simulation cycle accounting and the pipe view

**An experiment is one command.** `tools/exp.sh <tag> [<tree>]` copies a tree as it stands to the
simulation hosts and runs the standard set: the 60 M Linux lockstep, Dhrystone and CoreMark in
lockstep (`workloads/rvbench`), the four pi checkpoint slices and the five GB5 checkpoint slices
(Navigation, HTML5, SQLite, PDF Rendering, Camera: 20 M retires of warm-up, 50 M measured); the
table (`tools/exp-table.py`) compares each with a baseline tag, by default main's. `SET=sp` runs
GB5's single-core section as SimPoints instead: `workloads/gb5/simpoints.txt` holds 77 points of
50 M instructions (Simmerv's `--bbv` basic-block vectors, `tools/simpoint.py`'s k-means per
subtest), each restored from its own checkpoint, and the table totals each subtest's cycles as its
instructions times its points' weighted CPI. Against the board's per-subtest times at d7fd88f3
(the same RTL) the integer subtests come within 5% but for AES-XTS (+7%), PDF Rendering (+16%)
and Clang (+18%), while the floating-point ones run 7 to 44% faster in simulation (Machine Learning
-44%): the testbench's memory answers a D$ fill in about 32 cycles, the board in 46.6 (§9.2).

`tb_smolrv64_linux` puts every cycle into exactly one cause: the Top-Down Variant A partition,
split down to the core's own stall taps. The classifier is `smolrv64_core`'s `td_k`, the one the
`TD_*` counters count (§11), and the run's `perf-stat-sim` block (the event bus counted per
cycle, printed as `perf stat` would) run through `tools/perf-cpi-stack.py` reproduces the
`TOPDOWN-SIM` totals exactly (the `TDK_*` set; csr_file's event map is the bench's, read through
`hpm_sel_of`/`hpm_val`, so the two cannot drift). Bad
speculation (`bs:redirect`, `bs:drain` = `rd_wait`) wins over frontend (`fe:immu`, `fe:icache`,
`fe:align`, `fe:queue`), which wins over backend (`be:M-mem`, `be:rob-full`, `be:dep-load`, `be:dep-fp`, `be:iq-full`,
`be:rename`, `be:sq-full`, `be:lq-full`, `be:serialize`, `be:held`, in `d_hold`'s order). The IR loads the group the queue head takes, so the cycle
after the head waits on a backend credit finds the IR empty with the queue still full: that
refill cycle (`td_rf`) takes the previous cycle's backend cause, and no `FE_*` event counts it.
Any other cycle is `ok:N`, where N is the number of instructions dispatched that cycle. The run
prints the nonzero totals as `TOPDOWN-SIM` lines and closes them: the causes sum to the run's
cycles, and Σ N·ok:N equals the instructions the IR took outside redirect cycles (a group the IR
takes in a redirect's cycle is wrong-path, and the cycle is `bs:redirect`; the board's `DPATCH`
counts those too). Either failing is a `$fatal`.

Beside it, one line per backend cause splits that cause by the state the core holds that cycle,
and each `$fatal`s on a cycle it cannot account for: `RENAME-SIM` (the shard below `LOWAT`, or the sweep; `SWEEP-SIM` counts the sweep's registers and cycles),
`IQFULL-SIM` (the head's missing credit: its lane's, the head-op register's, the FP queue's, the
divide's; and its lane's alone while another lane has room), `ROT-SIM` (the instructions each lane
dispatched, and the cycles a head op waited for lane A), `DEPLD-SIM` (the load the reported
entry waits on: not issued, at M, a dTLB miss, faulting, ready, held behind an unknown or an
overlapping store, a device load at the head, forwarding, a D$ waiter, in the D$, the slow path,
in a landing buffer; a CSR read in the SYSQ; an AMO in M), `SER-SIM` (the serialising op, the CSR
op or the CBO the head waits behind), `MEM-SIM` (the D$'s misses), `DISP-SIM` (why slot k does not
dispatch behind slot k-1: a group rule, or the credit it lacks), `FWD-SIM` (the loads the store
queue holds, by whether they can forward; an older store at the same offset in another page,
which the offset compare holds it behind, apart), `SQ-DRAIN` (the store queue's head: the door
unaccepted, drainable with the LSU idle, not drainable), `LQ-SIM` (the load queue's occupancy, its no-credit
cycles by why: a head op in flight or in the IR, full, or room by the count but none in a bank its
row would take; and
the entries then by state), `STLK-SIM` (the D$'s store queue: the store lookup's writes and merges,
what its head waits for, its misses, and a plain store's outcome in the main lookup), `PORT-SIM`
(a ready queued load the port
does not take, by what holds it), `PORTS-SIM` (cycles with one or more and two or more loads ready
for memory, a ready load beside a committed store, two or more loads waiting for M), `DTLB-SIM`
(the dMMU walker's busy cycles, its second-level lookups and walks), `TRAP-SIM` (the traps by
cause and mode, the cycles and retires in each mode, the PCs of the M-mode traps), `XLATE-SIM`
(dTLB hits, address faults,
the walker), `FRING-SIM` (the fetch stream, its taken and not-taken marks) and `FPORT-SIM`/
`TRAIN-SIM` (the register-file readers, the predictor's training).

`+kanata=<file>` writes the **pipe trace** (`core/tb_pipetrace.vh`) as a Konata pipe view
(github.com/shioyadan/Konata), limited to the `+trace_from`/`+trace_to` window: one row per
fetched instruction and ONE CODE PER CYCLE, the stage it moved to or what holds it, read from
the hardware's own state that cycle. A row opens the cycle its instruction is in the bundle
register; it is keyed by the fetch sequence number until dispatch binds a ROB index.

| codes | where | from |
|---|---|---|
| `f` / `Q` | the bundle register / held there, the queue full | mirrors of the bundle register and the queue (`q_push`, `q_pop`; a mismatch with `q_cnt` is printed) |
| `q` | the decoupling queue, behind the head group or popping | |
| `g r O W I o n S L z` | at the queue head, not popping: a group rule; a redirect freeze; no ROB row; nothing issues, a load or FP result awaited; its scheduler full (a divide: the MD stage; the FP queue); a head op in flight; rename low or sweeping; SQ full; LQ full; a serialising, CSR or CBO gate | `take`, `rule`; slot A, the head, takes td_k's conditions in td_k's order (`st_rob`, `dep_ld`/`dep_fp`, `hd_sched`, `rn_stall`, `hd_sq`, `hd_lq`), so it names what the stall row names; slot k > 0 the room credit it lacks (`crd`) |
| `i` | the IR, dispatching | `s_v`, `d_take` |
| `t` | a scheduler's dispatch stage (for F, the head of its FIFO) | `stg_v` |
| `w` / `s` / `p` | in a scheduler: an operand pending / ready, not picked / picked | the scheduler's `v`, `srdy`, `iss_ent` |
| `a b c d` | executes in lane A..D | the lane's select register |
| `u v j F Y` | a multiply in flight; a divide in the MD stage or its queue register; the FP port's select register; stage F or the FPU; a system op waiting in the SYSQ | `m1`/`m2`, `md_v`/`mq_v`, `j_v`, `f_valid`/`fq_v`, `sy_v` |
| `m M k l y e` | an address waiting for M's fill; in M; a load filled, waiting to access; a load accessing; a store waiting for its data; a result in its landing buffer | `la_v`/`sa_v`, `m_valid`, the LQ's `av`/`sent`, the SQ's `dv`, `lb_*` |
| `!` / `X` | a mispredicted CTI waiting for its restart / killed, waiting for the release | `lr_mis`, `fr_v`; `rob_kd` |
| `=` | complete, waiting to retire | the ROB's `done` |

A FETCH row (`~~ fetch: <cause>`) covers each run of cycles in which the bundle register could
take a bundle and fetch has none: `redirect`, `freeze` (fence.i, an I$ invalidation, a mapping
change, a rejected mark), `itlb` (no translation of the PC, or a fault for it), `page` (the
stream left the PC's page), `pq-full`, `in-flight` (the ring empty, its room reserved by the pairs
in flight),
`icache-busy` (the I$ refuses the request), `icache-wait` (requests out, no bytes back yet),
`straddle`, `reject`, `bytes-late`, `align`. A STALL row (`** <cause>`) covers each run of
non-producing cycles with the core's top-down cause above, so each stall appears in program
order where it happened.

`tools/pipeview` (Rust, `cargo build --release`) is the terminal viewer:
`pipeview run.kanata [prog.elf]` is interactive (scroll, search by PC, mnemonic, symbol or cause,
with IPC and the stall causes recomputed over the visible rows), `--text N` prints the first N
rows, and `?` (or `--help`) shows the legend. Each cell is the row's code, coloured by class
(executing, moving, waiting on data, held by a structure, complete); `R`/`x` mark the cycle it
retired or was flushed in, and an op that executes and retires in one cycle shows its lane's
letter in upper case. With the ELF, labels are objdump's disassembly and
symbols; without it, `tools/rvdisasm` decodes the instruction word. `tools/kanata-disasm.py`
does the same for Konata. For example, in `workloads/rvbench`, where `local/<name>.c` builds a
program of your own:
`make run B=sillyloop PLUSARGS="+kanata=$PWD/s.kanata +trace_from=2000000 +trace_to=2001000"`,
then `../../tools/pipeview/target/release/pipeview s.kanata sillyloop.elf`.

**A slice of a long workload.** `simmerv-cli --insn-time 100 --ckpt-at N[,N...]:<dir> <images>`
writes architectural checkpoints -- registers, CSRs, device registers and memory, no
microarchitectural state (`docs/PLAN-2026-09-25-arch-checkpoints.md`) -- that any RTL version
restores. `CKPT=<dir>/N.ckpt core/run-linux.sh +measure_from=<warm-up retires> +max_retires=<end>`
runs a restore stub from the boot SRAM, then the workload, and the testbench's statistics
(TOPDOWN-SIM, the perf-stat block, the -SIM lines; not the core's own WALK-SIM, SQ-SIM and MEMDEP counts)
cover only the measured window. `core/run-cosim-linux.sh` takes the same `CKPT` and locksteps
from the checkpoint's first instruction.

---

## 12. Verification

| gate | command | pass |
|---|---|---|
| lint | `src/lint.sh` | `lint: clean` |
| riscv-tests | `core/run-vl.sh` | `pass=240 fail=0` |
| directed tests, for what riscv-tests never does (a misaligned AMO; a counter overflow, Sscofpmf) | `core/run-directed.sh` (after `run-vl.sh`) | `PASS` for every test in `core/directed/` |
| unit benches of the shared blocks and devices, under Verilator (the two C++-driven benches, the SD-SPI host and virtio_blk, included) | `src/run-tb.sh` | `tb pass=23 / 23` |
| Linux lockstep vs simmerv | `CYC=300000000 core/run-cosim-linux.sh` | no assertion, no divergence; the retire count against `cosim-expected.txt`; every plain RAM store byte-exact (below) |
| the D$ | `core/run-dcache-tb.sh` | `DCACHE-TB: 48 of 48 runs pass` (6 seeds × in-order/reordering memory × default, slow, fast-with-stores, remap-heavy; every outcome occurs; memory equals gold after every clean) |
| `rv_cache`, both shapes (no longer instantiated) | `core/run-cache-tb.sh` | PASS at LAT=4/20/100/200 |
| load/store queues | `core/run-lqsq-tb.sh` | `LQSQ-TB PASS` (85 directed checks) |
| load/store queues, random | `core/run-lqsq-rand-tb.sh` | `LQSQ-RAND PASS` |
| virtio-net DMA, both directions | `src/run-tb.sh` (`tb_virtio_net.v`) | `VNET-TB PASS` (8 TX + 8 RX frames at every alignment, cycles per frame printed) |
| Ethernet RX engine (MACs + the slot ring, two clocks) | `core/run-ethrx-tb.sh` | `eth_rx_engine: PASS` (a burst, a full ring, an ack landing mid-frame, FCS-bad, over-long) |
| CBO behind and ahead of stores | `make -C workloads/fphammer cbozero.bin && FW=$PWD/workloads/fphammer/cbozero.bin CYC=4000000 core/run-linux.sh` | `cbozero: ok` (the tiny128 boot issues no cbo.zero; the Geekbench image does, at SLUB init) |
| long guest (per batch) | `core/run-cosim-gb5.sh` | no divergence through the kernel boot (>400 M cycles) |
| disk-backed lockstep: virtio-blk, coherent DMA (`dma-coherent`: the testbench's slave snoops each line through `rv_soc_top`'s DMA port first; 1.5 G cycles, ~80 min: userspace starts after ~1.3 G) | `make -C workloads/tiny128 cosim-blk` | no divergence, `BLKCHECK-OK` on the console (the initrd's S99blkcheck mounts the 4 MiB ext4 image, verifies every byte, writes a copy back and re-reads it past the page cache); the runner fails on `BLKCHECK-FAIL` or its absence |
| memrand: random memory ordering under the lockstep (B4, 2026-09-18) | `make -C workloads/memrand sweep SEEDS="1 2 3 4"` (and `sweep-iw3`) | `MEMRAND seed=N: PASS` for every seed: a 50 k-op random stream of loads/stores/AMOs/LR-SC/FP/cbo/wrong-path cbo.zero and stores behind a late always-taken branch/fences/sfence.vma/pointer chases/megapage remaps over three VA aliases in S-mode, with the testbench's DMA agent (its lines flushed from the D$ first) interrupting through PLIC source 11, whose handler reads the window cached (a stale line diverges: with the agent's flush turned into a clean, seed 1 does); every load and every store byte judged by the lockstep |
| DDR-latency sweep (B7, 2026-09-17) | `tools/mem-sweep.sh docs/measurements/<date>-mem-sweep.txt` | the table of retires at 60 M for latency 4 / measured / 80 at IW=2 and IW=3; an MLP increment is judged by how much of the gap to latency 4 it closes |
| glibc userspace (per batch) | `workloads/glibc/run-cosim.sh` | `GLIBC-TEST PASS` and `STOP-AFTER` (the runner's `STOP_AFTER` ends the lockstep 1 M cycles after that console marker, ~0.6 G cycles at 6f2bbf1f, instead of running out its 1.8 G budget) |
| systemd userspace (per batch) | `workloads/sysd/run-cosim.sh` | `SYSD-TEST PASS` and `STOP-AFTER` (~0.45 G cycles of its 1.6 G at 6f2bbf1f) |
| the board | `tools/board-gate.sh <dir>` | `BOARD: PASS`: `login:` with zero faults, the userspace stress (B10), the integrity log read back clean from Linux (§11.1, 2026-09-20), rtl= recorded — a `+` on it means a dirty tree and no commit's verdict (G12) |
| a Geekbench 6 bisection step (2026-09-20) | `tools/gb6-bisect.sh <bit> <tag> [PASS_S]` | one line, `GB6-BISECT: PASS <tag> ... survived 16200 s` or `FAIL <tag> ... after <s> s at subtest <n>`; GB6's PDF Renderer kills 390d5028+ at 87 min where GB5 completes, so a pass is 3× that |

**`CYC=300000000` is the required tiny128 cosim length.** At 40e6 the run reports `inj=0` — it
never reaches the first interrupt — and three defects that wedged hardware were invisible at
that budget. The runner rebuilds whenever any source changed (its stamp hashes them) and
prints the model's source hash on the verdict line; a verdict whose log lacks
`building obj_dir_smolrv64_clinux` after an RTL edit is not a verdict (rule G5; a non-default
MEM_LG2/VDEFS builds `obj_dir_smolrv64_clinux.<hash>` and runs in parallel with the others, rule G10). tiny128 is a
small guest: it never lined up the store-queue age defect of 2026-09-04 in 60 M cycles,
the Geekbench boot did at 447 M, so the long-guest run is a standing per-batch gate. The
DDR model is the board's measured shape by default (`+ddr_lat=N` for a flat sweep, `+ddr_cut=N` for a
faster memory path): `rv_soc_top`'s `ddr_hpm` read with `workloads/ddrhpm` on 414eda07, 2026-10-21,
reads 35.2 and writes 21.3 cycles mean from acceptance at the line port. Bare-metal kernels in the
lockstep against the same code on the board (user mode, 512 KiB arrays): a dependent chase 45.89 vs
45.87 cycles a load, a read stream 3.27 vs ~3.2 cycles an element, a daxpy 7.32 vs 7.4, a fill 2.04
vs 2.03; a copy 3.81 vs 4.15, which is bare metal against Linux, not the model: the same binary
under Linux in the RTL, from a Simmerv checkpoint, copies at 4.53 cycles an element against the
board's 4.64, with the same lines (16.5 K reads, 8.2 K write-backs a pass of 512 KiB) and read
latencies 38.4 against 36.8. Under translation the RTL runs that copy 20% slower than bare metal;
the model errs slow on a read stream (3.48 against 3.32), having no open-row hits. At its end the
lockstep prints `ddr_hpm`'s totals (`[ddr-hpm: ...]`). Integer subtests' Top-Down matches
the board's within a few points; the FP subtests' back end is about twice the sim's on the board
(Machine Learning 56.7% vs 21.5%), which the latencies do not explain. On GB5's 77 SimPoints a port
10 cycles faster is worth 4.31% of the single-core cycles, 20 cycles 7.74% (no L2).

Invariant assertions are **always on** (`$fatal`, never `` `ifdef ``). Only flood-volume
tracers and stats are gated.

**The lockstep compares stores byte-exact (B5, 2026-09-17).** Until then the memory effect of
a retiring instruction was its kind and PA only: a store that landed the right address with
wrong bytes or wrong byte enables corrupted both memories in silence and surfaced, if ever, as
an unrelated fault much later. Now the reference reads back the aligned 64-bit word after each
RAM store (`mem_rdback`, `mem_size`), the DUT reports the raw value and log2 size of every
plain store (the store queue's committing entry through its landing bypass, or the LSU's
M-path store), and `probe_cosim.cpp` requires the sizes to agree and every byte lane the DUT
wrote within that word to equal the reference word's byte. An AMO's read-modify-write, an SC,
a cbo and an MMIO store are not checkable this way and are counted, not skipped silently: the
progress line prints `stores checked=N unchecked=M` (tiny128 at 60 M: 3,051,638 checked, 4,469
unchecked). `+st_corrupt=<retire#>` flips a byte of that retirement's reported value and must
abort at that retire (the check's self-test; it does on retire 35, the boot's first store).

---

## 13. FPGA implementation (XCKU5P, `platforms/rk-xcku5p-f-v1.2`)

| | |
|---|---|
| `probe_clk` | 166.67 MHz (6.000 ns), `PROBE_CLK_DIV8=48`: a BUFGCE_DIV of the 333.33 MHz DDR4 `ui_clk` (an MMCM was tried and reverted, 2026-08-20; only integer multiples of 3.000 ns time) |
| DDR4 | Two x16 parts, 32 bits at 2666 MT/s (`ui_clk` 333.33 MHz). One controller configuration serves 8 Gb parts (MT40A512M16LY-075, 2 GiB) and 16 Gb parts (MT40A1G16RC-062E, 4 GiB): the custom part `ddr4_part.csv` has 17 row bits, CL19/CWL14 at 750 ps, and each timing the worse of the two (tRFC 550 ns, tXPR 560 ns, tRCD/tRP 14.25 ns). The AXI address is 32 bits, PA − `0x8000_0000` (`src/dram_window.vh`); an 8 Gb part ignores row bit 16 (A16), so on a 2 GiB board the upper 2 GiB repeats the lower and its device tree describes 2 GiB (`ubuntu-nfs.dts`). A 16 Gb board boots either; `ubuntu-nfs-4g.dts` and `ubuntu-sd-4g.dts` describe 4 GiB. |
| Timing | `probe_clk` WNS **+0.023 ns**, TNS 0.000, 0 failing endpoints on the default directive (the 2026-09-07 release build 384fd599: the two-wide stack with the six T1 cuts; +0.019 on W7, +0.056 on the counter split) |
| Margin | 50 ps against a placement spread of 81-400 ps (rule I2): closed, **not robustly**. History, because each step was paid for: +0.029 at 10/12 dynamic issue; +0.124 at 8/8 (reverted -- it cost 4.1% geomean on GB5); +0.069 with FP four-in-flight, which *gained* 40 ps by deleting `fpu_inflight`/`fb_busy` from M; +0.028 with the FP scheduler and stage F, which cost 41 ps; +0.038 at `NF`=5; **+0.015 at `NF`=8** once the frontend's `apc` cone was cut (rule I6) -- the minimum-8 policy is affordable and was never the scheduler's fault. Area of the release build 384fd599: 83,093 LUTs (38.3% of the XCKU5P), 49,498 flip-flops, 122 of 480 block RAM tiles, 28 DSPs, no UltraRAM. |
| Measured clock | 164.2 MHz by on-chip counter |
| Core voltage | 0.853 V measured against a 0.85 V design point |
| 333 MHz | measured **−2.492 ns**, 39,389 failing endpoints. Operating-condition levers are worth exactly zero. |
| Build | `make bit` (`SMOLRV64_CORE=1 SMOLRV64_HW=8 PROBE_CLK_DIV8=48` are the defaults; HW=4 until 2026-09-05) |

Read `probe_clk` from the **Intra Clock Table**, not global WNS — global WNS is usually
pinned by `ui_clk`'s paths.

---

### 13.x IW=3 closure at 166.67 MHz (2026-09-17)

`SMOLRV64_IW=3 SMOLRV64_HW=8 make bit` closes: **WNS +0.007 ns** (from −0.528 on the dispatch-stage
base dd371e6d), no PRF read-port change. The plateau was late control roots in front of
ordinary logic, not congestion: a 64-bit adder+compare at the head of the fetch loop, the
rename stall inside the free-list read address, the redirect in front of a landing load
(nine combinational loops through `ld_land` -- `redirect` needs `m_done_red`, which yields to
`ld_land`, which was `~flush` -- and a DRC LUTLP-1 bitstream blocker; synthesis retiming
stays off regardless, every build log says "no retiming"), M as
a second writer of SH_FE in front of the CTF link's broadcast, and the live retire count into
`minstret`. Per build (each verified by lint, riscv-tests 240/0, the IW=3 and IW=2 60 M
lockstep cosims):

(From C4a step 2 on, **IW=3 is POR and IW=2 is no longer built or board-gated** -- Tommy,
2026-09-19. Earlier rows carry both widths because IW=2 was the shipping width until IW=3
closed on 2026-09-17.)

| build | cuts | WNS | TNS | failing |
|---|---|---|---|---|
| dd371e6d | dispatch stage | −0.528 | −1275 | 3940 |
| hm1 | fetch tags, `pt_ld_kill` latch-only, SH_FE = F only, stall-free free-list address, delayed minstret | −0.117 | −5.5 | 90 |
| hm2 | + fpnew 5, UART decode, REGRDY, M rs3 off, free-list bank index, map-indexed pending lookup, matrix wake | −0.130 | −6.7 | 143 |
| hm3 | + the served-slot hint, matrix column clear after the row write | **+0.007** | 0 | 0 |
| w1p3 | hm3 with W1' (in-core FP back on SH_FE; the CTF link has priority, M yields) and the narrow instret hold (M1b) | **+0.034** | 0 | 0 |
| w1p2 | the same RTL at `SMOLRV64_IW=2` (the shipping width) | **+0.035** | 0 | 0 |
| f1iw3 | + the D12 fix (device loads classified by their own translate PA; boots the board clean) | **+0.017** | 0 | 0 |
| f1iw2 | the same at `SMOLRV64_IW=2` | **+0.039** | 0 | 0 |
| c1iw3 | C1 (mul/div on the F/CTF/MD port, NF 8) + B5 (store-data capture, sim-only) | **0.000** | 0 | 0 |
| c1iw2 | the same at `SMOLRV64_IW=2` (core; the top-level +0.010 is the virtio_blk backend) | **+0.070** | 0 | 0 |
| c3s1iw3 | + C3 step 1 (the SYSQ shadow; no synthesis reader) | **+0.007** | 0 | 0 |
| c3aiw2 | C3 step 2 (csr_file's payload from the LUTRAM at m_rob_idx) at IW=2 -- rejected: the same at IW=3 was −0.030 | +0.030 | 0 | 0 |
| c3s3iw3 | C3 step 3: system ops through the F port, the SYSQ fires at head from flops; M's system arms gone | **+0.061** | 0 | 0 |
| c3s3iw2 | the same at `SMOLRV64_IW=2` | **+0.058** | 0 | 0 |
| c4a1iw3 | C4a step 1: the tagged fast load path (da28895b re-done) -- FIRST build | −0.070 | — | no bitstream |
| c4a3iw3 | + the landing-index fix (selects on the raw response, not the per-tag o_v/o_kill) | **+0.039** | 0 | 0 |
| c4a3iw2 | the same at `SMOLRV64_IW=2` | **+0.048** | 0 | 0 |
| c4a2iw3 | C4a step 2 attempt 1: the door self-loop with `hit` in `rd_ack` | −0.270 | — | 6028 near-critical |
| c4a2skid | + the door's skid buffer (the accept is register-decoded again) | **+0.061** | 0 | 0 |

Worst families at hm3 (census, slack < +0.35: 1823 endpoints): fpnew's own pipeline +0.007,
ROB head → scheduler ready +0.020, ROB head → `pl_q` CE +0.038 (257), `m_addr` → rename
commit +0.08, the PRF read into the ALUs +0.085. Cosim: IW=3 retires 11,682,439 (−0.02%
vs the base: the second cycle at head), IW=2 13,491,763 (+0.5%). `docs/HANDOFF-iw3-hail-mary.md`
has the per-round record and the rejected cuts. **The IW=3 counts above were LOW**: the
testbench summed two of the three commit ports until 2026-09-17 (`retire3` is a port of
the core and the SoC now); the true 60 M count at IW=3 is 13,367,267 against 13,494,359 at
IW=2 (−0.9%, not the −11.7%/−13.4% on record), and `cosim-expected.txt` rows carry the width
in a 5th column. **Board (2026-09-17): every bitstream of
this branch, the base dd371e6d included, kills the NIC (NFS stalls, virtio-net TX watchdog)
while the last board-clean main bitstream boots clean the same morning; bisected on the board
at 111 MHz to 5ef15e3d (CTF-on-FP) -- 760fe336 boots clean, 5ef15e3d with either loop cut does
not. No cosim can see it: `tb_smolrv64_linux` ties virtio and every PLIC interrupt off. The
closure above is real; the branch cannot ship until that defect is found (handoff, 'Board').**

### 13.y IW=4 closure at 166.67 MHz (2026-10-24)

`make` (the shipping build, `SMOLRV64_IW=4`, `SMOLRV64_HW=8`) closes at **WNS +0.001 ns** on 325e5341,
from −0.591 on the first IW=4 build. The plateau was route: 92–95% of the CLBs in use and every
failing family within 0.03 ns of the worst, so area went first, then the deepest family. Per build
(each verified by lint, riscv-tests 240/0 and the 60 M Linux lockstep at IW=4):

| build | cuts | WNS | TNS | LUTs |
|---|---|---|---|---|
| 79c1a9ac | IW=4, `SEQW` 9 (the ROB at 57 rows), FW 8 | −0.591 | | 160 825 |
| 3b1b81d3 | + the geometry header, 9-bit physical registers, the state review (lane write tags, the ROB's done clears, the FP wake ports, the waiter table, the translate flag, ...) | −0.412 | −516 | 151 696 |
| 9ec51393 | + FW 6, `PABITS` 33 for every PA the core holds, TLB tags without their index bits, the queues' banked allocation writes, the divider's one rotation, the PRF's shard select | −0.167 | −81 | 147 239 |
| 325e5341 | + the lanes' operand bypass selected by the scheduler's registered wake match | **+0.001** | 0 | 146 097 |
| 71796044 | + nine cycle-identical cuts (the walk's first column, the D$'s doubleword unshifted, a forward's one shift; a landing from registers and its kill looked up a cycle ahead; the translation regime and the lanes' ages as registers; three queues writing a cycle late) and the corrector read ahead | **0.000** | 0 | 146 537 |
| 7a3b6c95 | + the forward ring's busy flag a register (the scanout's 16-bit line counts and the monitor's device-tree rewrite beside it) | **0.000** | 0 | 146 975 |
| 0446b9de | + the DMA port's three reads in flight and its request from a register | **0.000** | 0 | 147 932 |
| e51bbfee | + the schedulers' oldest-first pick (alone it missed by 0.497 and, with `ExtraTimingOpt`, 0.480 -- the LQ's commit cone, the lanes' completion, the landing's wake, ~3,000 endpoints), and three cycle-identical-or-nearly cuts: rename's low-water flag from registers, a CTI's completion a cycle late on a port of its own, the landing's destination read a cycle ahead | **+0.001** | 0 | 148 748 |
| 7f8f5a94 | + rename's low-water flag counting the IR group's allocation from the IR's registers (e51bbfee's mark plus a cycle's worst-case allocation had doubled the rename stalls) | **0.000** | 0 | 150 870 |

The board gate passes on 325e5341 (the 2g board, 2026-10-24): login, 900 s of Geekbench, zero faults,
and on 71796044 (2g, 2026-10-25), 7a3b6c95 (4g), 0446b9de (4g), e51bbfee and 7f8f5a94 (4g, 2026-10-27; both boards' flash since). The margin is a draw's, not the design's: the same netlist family
missed by 0.15 to 0.62 ns in other draws (6ca128ca -0.615, 23ae98a9 -0.335, 5f3fdaca -0.208,
2f791a83 -0.249, b06455d0 -0.152), each cut taking its family off the failing list (5,593 endpoints
under 0 at -0.615, 1,367 at -0.208); what remains is a plateau of the core's loops -- the lane's
execute and completion, the predictor's and the fetch ring's next address, the commit count.

## 14. Known limits

- **Up to `LQ_N`=16 fast loads outstanding (by tag), one slow one, and two divides in the MD
  stage** (the stage and its queue register; the divide credit holds a third at dispatch). FP is no longer
  among them: `NFLIGHT`=4 and results return by tag, out of issue order (§7). `LQ_N = 1 << LQ_IB`;
  `LQ_IB` = `SMOLRV64_LQB` = 4 (`smolrv64_tags.vh`, which the core, `rv_soc_top` and the benches
  read), and the D$ read tag is `{1'b0, lq_idx}` for a load, the walkers' and the slow path's past them, `DRTW = LQ_IB + 1`. 8 -> 16 entries:
  Dhrystone 1,660,876 -> 1,585,889 cycles (lockstep, IW=3). 16 -> 32 at the lane rotation
  (6f2bbf1f, the GB5 checkpoint slices): -0.37% of their cycles, PDF Rendering +1.21%. PDF's full
  queue was 13.1 loads of 16 ready and waiting for the port in its no-credit cycles, behind the
  stores that shared it; the store register and loads first at the D$ (§8, §9) took PDF's
  `be:lq-full` from 22.2% to 2.3% of its cycles at 16 entries, so it stays at 16. A
  sequential stream of 8-byte loads holds a line in eight entries, so sixteen keep under two of
  its lines in flight: scalar `MemoryBandwidth` over 64 MiB (`tools/board/membench`, 2ed18026)
  reads 2.4 bytes a cycle from DRAM, 26.7 cycles a line, with under two of the 8 MSHRs busy;
  counted over the whole run, its 64 MiB fill included, the queue holds 13.3 entries on average
  and its head waits for a credit in 60% of the cycles.
- **A dispatch group holds one divide, and a cut slot cuts every slot behind it**
  (`smolrv64_frontend` `rule`). Loads, stores and FP ops fill a group (2026-10-20,
  `docs/PLAN-2026-10-20-dispatch-groups.md`). A run of loads dispatches a row a cycle and M
  fills one a cycle: the bare-metal `workloads/rvbench/local/bwread.c` (64 passes over 32 KiB) in
  lockstep takes 272,474 cycles, about 7.7 bytes a cycle against the D$ port's 8, where one load a
  group took 338,454 (slot B cut by the rule in 197,046 of them); the LQ's credit now holds slot B
  in 97,643, its 16 entries filling while M drains one a cycle. Tommy, 2026-10-20: a group is cut
  only where it must be, beside a system op.
- **One load lookup a cycle.** Stores have a lookup of their own (§9.1), so a ready load waits for
  the D$ in 0.9% (SQLite) to 1.7% (PDF Rendering) of the GB5 checkpoint slices' cycles, against 3.6%
  to 35.9% with one lookup for both. Two or more loads are ready to go to memory in 10% of
  Navigation's cycles and 1% of Camera's (the testbench's `PORTS-SIM`): a second load port is a
  second copy of the data arrays (`docs/PLAN-2026-10-03-dcache-vipt-l2.md`, step 4 of its revised
  order).
- **One page walker per MMU.** Walks run one at a time (hits keep the lookup port, §8): a
  footprint that walks on every load (`chase2` over 32 MiB, `tools/board/membench`, 2ed18026) keeps 2.4 loads in flight
  at any number of independent chains, the walker busy 30 of a load's 33.8 cycles, against 7 to
  8 while the dTLB's two levels translate them.
- **Up to `SQ_N`=32 stores between dispatch and their drain.** `SQ_N = 1 << SQ_IB`, `SQ_IB` =
  `SMOLRV64_SQB` = 5 (`smolrv64_tags.vh`). A store holds its entry from dispatch until the LSU
  drains it after commit, and a load whose bytes overlap an older queued store waits for that
  drain unless it can forward from it (§8). 8 -> 32 entries (2026-10-13, IW=3): the 300 M
  lockstep boot 238,877,773 -> 246,963,412 retires (+3.38%; 16 entries +3.35%); GB5 slices of
  50 M retires after 20 M of warm-up, restored from Simmerv checkpoints (§11.2), in cycles:
  Navigation -0.74%, HTML5 -2.49%, SQLite -1.40%, PDF Rendering -2.22%. `be:sq-full` falls
  below 1% of cycles in all four and the stall moves on: Navigation's ROB fills (0 -> 10.7%)
  with loads waiting on older stores (a load blocked in 19.5% -> 35.2% of cycles), PDF
  Rendering's ROB and load queue (24.5% and 21.9%).
- **The FP scheduler holds `NF`=8 ops, behind a FIFO of `FQ`=8** (§2), which takes a group's FP
  ops together: one a group, against a row, cost GB5's 77 SimPoints 2.48% of the single-core cycles
  (2026-10-20, d3dd7b52: Structure from Motion -5.42%, Gaussian Blur -5.00%, Camera -2.74%, Rigid
  Body -1.67%; the integer subtests within 0.06%). The FIFO, not a bigger
  scheduler: GB5 slices restored from Simmerv checkpoints (§11.2), in cycles, FIFO 1 -> 8: Camera
  39,230,305 -> 32,826,256 (-16.3%), Gaussian Blur 39,653,597 -> 33,357,181 (-15.9%), Structure from
  Motion 36,336,791 -> 29,204,556 (-19.6%); Navigation unchanged. `be:iq-full` falls from ~21% to
  2-4% and `be:dep-fp` from 23-28% to 5-9% (dispatch keeps feeding the loads behind a burst); the
  ROB fills instead (`be:rob-full` 24-27%). A 12- or 16-entry scheduler saves the same cycles (16:
  -16.9 / -16.8 / -19.8%) and does not close 166.67 MHz (2026-10-13: routing congestion, WNS -0.3 to
  -1.0 ns). `NFLIGHT` 4 -> 8 changes no cycle: the FPU never holds four ops with a fifth ready.
- **ALU, FP, CTF and mul/div reorder** (§2.1, §7); memory, AMO, CSR and fences still issue
  in program order from `u_iq_l`. A long-latency op in M still blocks *other M-class ops*
  behind it. Freeing those needs the load queue and store buffer, and `head_block` gone.
- **M is a single execute slot** for everything except ALU ops (which complete at issue) and
  FP arithmetic (stage F), so one M-class long-latency op is in flight at a time.
- **Two-wide, not wider.** The aligner emits up to two instructions per cycle and dispatch
  and retire match it (items 10a-10c); the frontend is 4% of Geekbench's cycles and 10% of
  `sha256sum`'s now (the two-wide stack, 2026-09-07). Going past two is a frontend redesign
  (docs/PLAN-2026-09-06-frontend.md on `wip/fe-ideas`).
- **Stores and AMOs still block M.**
- **No memory disambiguation** — a younger memory op simply waits for `S_IDLE`.
- Workload sensitivity is large and measured: Geekbench 5 is LSU-bound (47% of cycles on the
  two-wide stack, FPU 19%), `sha256sum` was frontend-bound until the two-wide fetch. Do not
  generalise a CPI stack from one workload.
- **No vector extension.** Three Geekbench 5 subtests (Gaussian Blur, Structure from Motion,
  Machine Learning) run at a normal IPC and score 0-2 because the reference machine's SIMD
  does in one instruction what this core does in many (2026-09-07, per-subtest trace).

## 15. Prioritised work list

Ordered by measured impact over effort. Every claim names its measurement and the
CONFIGURATION it was measured in; anything unmeasured says so.

### Measurement discipline (read before adding a number here)

**THE SHIPPING `SMOLRV64_HW` AND `SMOLRV64_IW` ARE THE DEFAULTS EVERYWHERE** (`SMOLRV64_HW` 8 since 2026-09-05, 4 before;
`SMOLRV64_IW` 3 since 2026-09-25, 2 before) -- the RTL, `build.tcl` and the sim runners -- because a shipping configuration that has to be remembered is one that will be
forgotten, and this one was: `SMOLRV64_HW` and `PROBE_CLK_DIV8` live only on the command line
(`build.tcl` rebuilds the define list from scratch every run, so the `.xpr` records the
last build and carries nothing forward), and the command anybody actually types is
`make SMOLRV64_CORE=1`. Every bitstream built from that line ran at HW=2 and 66.67 MHz. The
table below is why that is not a small thing -- the two widths do not merely differ in
degree, they disagree about which unit is the bottleneck:

| workloads/aesbench | HW=2 (never build this) | **HW=4 (the shipping build until 2026-09-05)** |
|---|---:|---:|
| cycles/byte | 222.90 | **122.70** |
| IPC | 0.277 | **0.506** |
| `FE_ALN` (RVC aligner) | **37.0%** | 8.0% |
| `FE_QUE` (decoupling-queue empty) | 24.0% | 24.0% |
| `ST_MEM` | 7.9% | **23.7%** |

At HW=2 the RVC aligner looks like the largest stall in the machine and the scheduler
sweep saturates; at HW=4 the aligner is a minor term and the sweep has a real optimum at
8. An earlier version of this list ranked the aligner P0 on the HW=2 numbers. It was
wrong, and it was believable because `run-cosim-linux.sh` reported the width by
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

### Answered: how big a ROB?

**64 rows of a group (192 entries at IW=3).** Measured on the GB5 checkpoint slices (20 M
instructions warm, 50 M measured, `core/run-linux.sh` with `CKPT`), cycles against 32 rows on
0f633992's RTL (2026-10-13/14):

| slice | 16 rows | 64 rows |
|---|---:|---:|
| Camera | +29.4% | -5.8% |
| Gaussian Blur | +33.2% | -5.8% |
| Structure from Motion | +37.3% | -2.9% |
| HTML5 | +2.6% | -0.8% |
| PDF Rendering | +1.8% | -0.6% |
| SQLite | +0.8% | +0.1% |
| Navigation | +4.4% | +1.5% |

The FP-heavy slices gain most (Camera is in GB5's integer section). Navigation's stalls move from `be:rob-full` (15.4% of
its cycles at 32 rows) to `be:iq-full` (8.2% -> 19.0%), `be:dep-load` and `be:lq-full`, and it
runs deeper wrong paths: its window is bound by the integer schedulers next. None of a replay
that sets the memory-dependence counter to 3, a 16x longer forget period, or a 32-entry branch
training queue (16 drops 4.8% of its trainings at 64 rows) moves it by more than 0.15%. The
window costs about 10 ps on the ROB's done fan-in (the 64-row probe on wip/bid: WNS -0.064 ns
against -0.057 at 32 rows, the same path families).

### Open question: mul/div were conflated with loads. How much does it matter?

Measured, full-suite GB5 (`2ba7716e`). `ST_MUL`/`ST_DIV` count cycles M is *stalled* on
those units, so they already ARE the cost of occupying the shared stage -- everything
M-class waits during them -- rather than a separate cost on top:

| | % of cycles |
|---|---:|
| `ST_MUL` | 0.901 |
| `ST_DIV` | 0.354 |
| **both** | **1.255** |
| `ST_MEM` | 59.620 |
| `ST_FPU` | 23.249 |

It is two conflations and they resolve differently.

**Scheduler — dissolves for free.** mul/div sit in `u_iq_l` and inherit memory's in-order
issue. Once ordering moves to the load queue and store buffer (P0), `u_iq_l` reorders and
this cost disappears without anyone touching mul/div.

**Shard — not actually a conflation.** Under shard-by-writer, `SH_LD` means "the shard M
writes", and M writes mul/div results. It becomes wrong only if mul/div get their own unit,
which would need a fourth shard: one writer per shard is the property the PRF rests on.

**What is genuinely left is div, not mul.** They are different units sharing a name --
`mul3` is 3 cycles and pipelined, `divider` is 3 to 35 cycles and iterative. Only a long divide
occupying the shared stage is a real problem, and the fix follows two precedents already in
the design: release the stage at start and land the result by tag, as non-blocking loads
and the FPU both do. No new shard and no new scheduler.

**Recommendation: move them, as part of P0 -- and the reason is not their stall.**

An earlier version of this section said "leave it", weighing only the 1.255%. That missed
the dominant cost. A scheduler's `NSRC` is set by its **worst-case occupant** and multiplies
through every entry and every wakeup comparator:

| memory scheduler | entry bits (`NENT`=12) | wakeup comparators (`NWB`=3) |
|---|---:|---:|
| today, `NSRC`=3 (FP legacy) | 348 | 108 |
| FP moved out, `NSRC`=2 | 240 | 72 |
| **unary, `NSRC`=1** | **132** | **36** |

**A memory op needs exactly one register operand: the base address.** The immediate is in
the payload, and the store's DATA stops being the scheduler's problem the moment the store
buffer accepts it separately (§11). So after P0, **mul/div is the only thing left forcing
`NSRC` > 1** -- not one cost among several, the sole remaining one. Moving them makes the
memory scheduler unary: 45% fewer entry bits and 50% fewer comparators than `NSRC`=2, on
the scheduler that is also the largest.

**Where they go: into the FP pipeline.** Three facts already in the design make this the
cheapest option, and they were not obvious:

- **The PRF is unified.** "FP sources are just rs1/rs2/rs3 with the fp bit set" -- there is
  no separate FP register file, so stage F and the integer pipe read the same array through
  the same ports.
- **`SH_FE` already holds INTEGER destinations** (`N_FE = 128, // > 64: the FPU writes
  INTEGER regs too` -- `fcvt.w.d`, `fmv.x.d`, `fle.d`). A mul's integer destination renamed
  into FE is an existing case, not a new one.
- **`u_iq_f` is already `FIXEDL`=0**, i.e. wake-at-writeback, which is exactly what a
  variable-latency op needs.

So it needs: `d_shard` routing mul/div to `SH_FE`, `mul3`/`divider` hung off stage F, and
the FE write port arbitrated between FP and MD results. **No new scheduler, no fourth
shard, no extra read ports, no IE arbitration**, and it inherits the tag mechanism that
already lets FP complete out of order. `u_iq_l` is then memory-only and goes unary.

**mul and div are treated identically at issue. There is no per-entry bit.** An earlier
draft of this section reached for a readiness gate (`~e_var[g] | ~md_busy`) so that a
64-cycle divide would not block FP issue. That is the wrong place to solve it.

**NEVER STALL ISSUE.** Issue is the most important critical path in the machine, and making
it conditional on a functional unit's state is precisely what puts unit state on that path.
Back-pressure belongs at the FRONTEND, where it is off the critical path and where there is
already a stall mechanism.

So: a divide issues exactly like a multiply and enters a **small request FIFO**, which the
single divider drains at one per 3 to 35 cycles. The FIFO is allowed to fill. Only when it
comes within **M entries of full** does the frontend stall (or restart) -- and `M` is the
number of instructions that may already be past the frontend and still turn out to be
divides. That slack is the whole design: it guarantees the back-pressure lands upstream
before issue could ever be asked to wait.

Measured, this mechanism is cold. On full-suite GB5, `ST_DIV` is 0.354% of cycles at ~64
cycles per divide (the radix-2 divider of that measurement), i.e. **one divide per ~4,560 instructions, one per ~18,000 cycles**. The
FIFO is essentially always empty and the frontend stall essentially never fires. It exists
to handle a divide-dense burst correctly, rather than paying for that burst on the issue
path in every cycle that is not one.

The same reasoning applies to the FPU's `iss_ready`, which today does feed `unit_busy` and
therefore does gate issue. It is registers-only, so it is not the worst version of the
mistake, but it is the same shape and should become a FIFO with frontend back-pressure when
this is built.

#### Alternative considered: with the ALU ops, in `u_iq_i` A fourth scheduler and stage
would need a **fourth PRF shard** -- one writer per shard is the property the register file
rests on, and mul/div can write neither `SH_IE` (that reintroduces the second writer whose
removal was worth 397 ps) nor `SH_LD` (the LSU owns it after P0). It would *not* need extra
read ports, contrary to a first reading: there is one set of `ra1/ra2/ra3` and a single
issue slot (`iq_iss_v = pick_l | pick_f | pick_i`), so a fourth scheduler shares them. Read
ports only become the cost under multi-issue.

The cheaper route keeps mul/div in `u_iq_i`, which is already `NSRC`=2, and teaches the
scheduler that some of its entries are not fixed-latency:

| | |
|---|---|
| routing | mul/div -> `u_iq_i`, no new scheduler and no new shard |
| new state | one per-entry `var` bit, set at dispatch |
| ready | `& (~e_var[g] \| ~md_busy)` -- gates mul/div entries only; ALU entries unaffected |
| wakeup | `self_v = ~e_var[sel] & do_iss` -- wake-at-select for ALU, wake-at-writeback for mul/div |
| execute | a small MD stage off the shared issue register, like stage F; result lands by tag |
| writeback | `we_ie` arbitrates: **the ALU always wins, MD holds its result until a gap** |

**The asymmetry is the whole point.** `unit_busy = m_wb_ie` used to hold *the ALU* off
whenever M wrote IE, and that was the 397 ps critical path. Inverted -- the ALU never
waits, and the rare case (1.2% of ops) absorbs the stall -- the shared write port costs
nothing on the common path.

`FIXEDL` is a module parameter today, so making it per-entry is the one real change inside
`smolrv64_iq`: one bit of storage and a mux on the selected entry. **The scheduler change can
be avoided entirely** by dispatching mul/div as if their destination were `x0` -- `prd`=0
broadcasts to nobody at select, so no dependent wakes early -- while the real physical
destination travels in the payload and is used at writeback. `x0` is already a handled
case. That leaves only the readiness gate.

Two things to verify rather than assume when this is built:

- **Starvation.** If the ALU issued every cycle MD could never write. It is self-limiting --
  a held result leaves its ROB entry incomplete, the ROB fills, dispatch stalls, ALU issue
  stops, a gap appears -- but that is an argument, and it wants an asserted bound on how
  long a result may be held.
- **Latency.** Dependents of a mul wake at writeback instead of select, losing the bypass
  that makes ALU chains free. Bounded (`mul3` is 3 cycles) and `ST_MUL` is 0.901%, but it
  is a real regression on mul-dependent chains and should be measured, not assumed small.

Do it while P0 is rebuilding the memory path anyway, not before and not separately.

### Deliberately postponed: store-to-load forwarding

The load-queue/store-buffer scheme (§11) admits a degree of forwarding as an extension --
a load whose address matches a pending store's, whose data has arrived, could take the
value from the buffer instead of waiting. It is **postponed on purpose**.

The general form -- complete, arbitrary bypass from any store to any load -- is
complicated, slow and large: it is an associative match of every load address against
every buffer entry, with byte-granular merge for partial overlap, on the critical path of
every load. The cheap correct behaviour is to WAIT, and §11.1's aliasing spectrum already
provides the useful middle ground without any forwarding at all: the cheapest test
("assume any older pending store aliases") is correct, and `|addr_load - addr_store| > 8`
against a few entries recovers most independent traffic.

Revisit only after P0 is measured. Forwarding buys nothing until loads can issue and
access out of order in the first place.

### CPI-stack counters: what they can and cannot be trusted for

Fixed in `f637fc8`, and the history matters because these numbers appear throughout:

- `iq_blk_pr` was a **priority mux** (LD, then FP, then integer), so an FP dependency was
  invisible in any cycle the LD scheduler was also blocked and got charged to `ST_MEM`.
  Now each scheduler is classified on its own shard and OR-ed.
- `ST_FPU` was `(st_m & fp_arith) | dep_fp`, and `fp_arith` is identically 0 since FP left
  M — the FPU's own occupancy vanished from the stack when it got stage F. Now
  `(f_valid & ~fp_disp) | dep_fp`.
- `dep_*` was gated on `m_advance`, from when "M could accept" and "issue could proceed"
  were the same statement. It masked completely on memory-heavy code. Now `~iq_iss_v`.

**Every CPI stack recorded above predates these fixes** and overstates `ST_MEM` at the
expense of `ST_FPU`. On `mlbench` the correction moved `ST_MEM` 41% → 33%.

**`ST_ROB` (0x0305) now exists**: dispatch has an instruction and the ROB has no room,
`d_valid & ~rob_ready`. The Zihpm bus was full at `[21:0]` and is now `[22:0]`, which
reached `src/csr_file.v`, `src/exec_bundle.v` and `src/backend_top.v` (the other core keeps
the new bit at zero; both files are gone since the 2026-09 release). `ST_SER` now excludes
it, so the two are disjoint rather than double-counting.

**It immediately corrected the claim that motivated it.** This document said `mlbench` was
ROB-full limited, on the strength of the testbench's `rob_full=68670`. That is a WHOLE-RUN
counter and mlbench's array initialisation is store-heavy; measured over the dot-product
kernel alone, `ST_ROB` is **1,001 cycles of 1,705,045 — 0.06%**. The ROB is not the
kernel's constraint. `FE_BUB` at **53.7%** is, and the ROB-full stalls live in init.

The recurring lesson: **a counter accumulated over a different window than the claim
answers a different question.** This is the second time in this session that produced a
confidently wrong conclusion.

**The events overlap and do not sum to a budget.** The stack reaches 94% of cycles with
rows that double-count a cycle stalled on two things. Read them as shares.

### UNRESOLVED: efe6dd6 is a 5.4% geomean regression that has no mechanism

**Do not run GB5 again until this is understood.**

`efe6dd6` differs from `72d14cde` by exactly one RTL change -- FP four-in-flight -- on
identical 10/12 schedulers. Aggregate counters said +2.4% IPC. The per-workload geomean,
which is what the score follows, says **-5.4%**:

| gained | | regressed (>8% noise floor) | |
|---|---:|---|---:|
| Rigid Body Physics | +6.3% | **Navigation** | **-25.4%** |
| Clang | +4.6% | **PDF Rendering** | **-12.1%** |
| Gaussian Blur | +3.9% | **AES-XTS** | **-9.1%** |
| Ray Tracing | +3.4% | Image Compression | -8.5% |
| Horizon Detection | +3.0% | | |

(Camera's -50% is 0.02 -> 0.01, single-digit quantisation, not real.)

The FP workloads gaining is consistent with the change. **The integer workloads regressing
is not**: nothing in FP four-in-flight can slow Navigation by 25%. AES-XTS fell 950.6 ->
863.8 and the Crypto score went 1 -> 0 again.

Candidate explanations, none confirmed:

- **Run-to-run variation larger than the AES-XTS-derived 7.4% floor.** That floor came from
  one workload; others may be noisier. Nothing establishes it as a global bound.
- **The ROB completion port widening** (`NW` 2 -> 3) that shipped with the FP change. It
  touches every completion, not just FP.
- **Machine conditions.** The run started at `load=1.82`, but no comparable figure was
  recorded for `72d14cde`.

**Method lesson, and it is the second time in this session.** Aggregate IPC and per-workload
geomean disagreed in SIGN here, and the aggregate is the one that misleads: it weights by
instruction count, so a workload that runs long dominates it while contributing one of 21
equal terms to the score. Judge changes on the geomean of rates. The same mistake in the
other direction is what made 8/8 look good on `aesbench`.

### Camera: what actually limits it, and the work in priority order

Camera is classified **Integer** by GB5 and scores 1, so it drags the Integer geomean.
`workloads/saxpybench` reproduces its loop from the trace: `y[i] = a*x[i] + y[i]`, 7
instructions per element, **no accumulator carried between iterations** -- every element
is independent.

Measured at `SMOLRV64_HW=4`, arrays L1-resident. **Always measure at `SMOLRV64_HW=4`** -- at the
simulation default of 2 this loop is frontend-bound (`FE_BUB` 76%) and reads 30.01 cyc/elem
whatever the memory system does.

| state | RESIDENT cyc/elem | STREAM |
|---|---:|---:|
| before the store buffer | 22.08 | 25.43 |
| store buffer (`ec6ad3e`) | **17.08** | -- |
| + load queue (`cb02868`) | 19.08 | 22.43 |
| + early start | **18.08** | **21.43** |

At 18.08: `ST_FPU` 0%, **`ST_MEM` 72%**, `ST_ROB` 21%, `FE_BUB` 0%.

The early start does not recover the queue's whole +2.00, and the reason is structural: it
fires only when NO older store is live, and about half of Camera's loads have one at M time
(49.2% reordered past an uncommitted store at `ec6ad3e`). Deciding that the older store does
not ALIAS needs the address, which is the flop the queue exists for. A partial-address test
on `VA[11:3]` would be legal and off the translate path -- page offsets survive translation
-- but saxpy's three arrays are page-aligned at equal offsets, so their low bits collide
every iteration. That door is closed; do not spend a session on it.

**22.08 was one element's critical path**: load 5 + `fmuls` 8 + `fadds` 8 ~= 21. So there was
**zero overlap between iterations that depend on nothing at all.**

What was swept at 22.08 and did NOT move it -- each of these is a hypothesis killed:

| swept | range | cycles/elem |
|---|---|---|
| ROB depth | 16 -> 32 -> 64 -> 128 | 22.08 at every one |
| FP scheduler `NF` | 4 -> 8 -> 16 | 22.08 |
| FPU `NFLIGHT` | 4 -> 8 | 22.08 |
| D$ footprint | resident -> 2x the cache | 22.08 -> 25.43 (misses are minor) |

Re-confirmed after the store buffer: **ROB 16 -> 32** takes `ST_ROB` 58% -> 0% with cyc/elem
unchanged at 17.08, and store-buffer `NENT` 4 -> 8 takes "full" from 426 514 cycles to 62,
also unchanged. The ROB fills *because* something downstream is slow.

On Dhrystone (`workloads/rvbench`, 5,000 runs, IW=3), ROB 16 -> 32 takes `be:rob-full` from
13.1% to 2.3% of cycles and the run time moves +0.5% (3 102 919 -> 3 118 298 cycles). The
freed cycles become frontend bubbles (`fe:*` +63 cycles per iteration, against -67 of
`rob-full`) and load-queue stalls (`be:lq-full` +18 per iteration). The frontend cannot fill
the deeper window, so it is the limit here; the LQ has to grow with the ROB.

`ST_ROB` goes 68% -> 0% at ROB=32 **with no change in speed**, which settles it: the ROB
filled *because* something downstream was slow, not the reverse. And `FE_BUB` is 0%, so the
frontend is innocent here.

**The remaining candidate is `u_iq_l` being in-order.** A store waits for `rs2` -- the
`fadds` result, ~21 cycles away -- and while it waits it holds the head of an in-order
queue, so the *next* element's loads cannot issue behind it. Independent work, perfectly
serialized. (An attempt to confirm by setting `INORDER(0)` on `u_iq_l` produced no output at
all: the head pointer is load-bearing for M's single-slot coherence, so the upside cannot be
measured that cheaply.)

#### Priority order for Camera

1. **A store issues on its ADDRESS alone; its data arrives at the store buffer separately**
   (`Area-Efficient-Scalar-OoO.md` 11). This is the whole diagnosis in one change: it stops
   a store waiting 21 cycles at the head of the queue.
2. **Load queue -- loads issue and access out of order.** Lets element *i+1* begin while
   *i*'s chain runs.
3. **Multiple outstanding loads.** `ldbench` measures 4.00 cyc/load throughput against 5.00
   latency, an overlap of only 1.24x; Camera does 2 loads per element. The D$ is no longer
   the floor: the port can hold two requests in flight. The gain from that alone is +0.32%
   (14,610,812 -> 14,657,366 retires at 60 M cycles) because the D$'s clients are the LSU and
   two page-table walkers, and when the LSU misses it is blocked on that miss anyway -- only
   an independent walk overlaps. **The next gain needs a CLIENT that can hold two requests
   in flight**, which is a multiple-outstanding LSU, not more cache work.
4. **FP latency.** 16 of the 21-cycle chain is `fmuls` + `fadds` at 8 cycles each. Real, but
   it is an fpnew `PIPE_REGS` / Fmax trade, not free.
5. **Superscalar.** 7 instructions per element is a 7-cycle floor at `IW=1`, so this cannot
   pay until 1-3 have moved the wall below it.

**Estimated ceiling, not measured:** with memory decoupled the floor becomes the largest of
7 cycles (issue width), ~4.5 (FP throughput at 2.25 cyc/op x2), and the memory throughput
term -- so roughly **10-12 cycles/element against 18.08 today**. Items 1-3 are one
piece of work (P0), which makes Camera the clearest single argument for it.

### Corroborated: the ST_MEM attribution fix (aesbench as control)

`aesbench` contains no FP, so a correct FP-attribution fix must barely move it. `blurbench`
is FP-heavy, so it should move a lot. That is what happened:

| | pre-fix | post-fix |
|---|---:|---:|
| **aesbench** `ST_MEM` (control) | 23.7% | **22.8%** |
| aesbench cyc/byte, `FE_BUB`, `FE_ALN`, `FE_QUE` | | **bit-identical** |
| **blurbench** `ST_MEM` | 31% | **44%** |
| **blurbench** `ST_FPU` | 65% | **49%** |

**The direction on blurbench was the opposite of predicted, and the reason matters.** The
expectation was that mis-charged FP dependencies would move `ST_MEM` -> `ST_FPU`. The
dominant effect is the reverse: in blur, `u_iq_f` blocks waiting on LOAD results (the taps
are `flw`), i.e. on `SH_LD` registers. The old priority mux reported only `u_iq_l`'s own
block, whose sources are address registers in `SH_IE` and so counted as *neither* event --
so the FP scheduler's waits on memory were invisible entirely. They correctly land in
`ST_MEM` now. `ST_FPU` falls because the stricter `~iq_iss_v` gate removes over-counting.

### Two score-1 workloads share shapes we already model

| workload | per-iteration shape | already covered by |
|---|---|---|
| Gaussian Blur | 5 `flw`, 4 `fmuls`, 4 `fadds`, 1 `fsw` -- 5-tap FIR, serial accumulator | `workloads/blurbench` |
| **Structure from Motion** | **identical** -- 5 `flw`, 4 `fmuls`, 4 `fadds`, 1 `fsw` | `workloads/blurbench` |
| **Camera** | 2 `flw`, 1 `fmuls`, 1 `fadds`, 1 `fsw` -- SAXPY, no cross-iteration accumulator | (none yet) |

Structure from Motion and Gaussian Blur are the SAME KERNEL SHAPE, which is why the limit
study gives them identical numbers in all nine cells despite being different traces (23 vs
24 distinct PCs). **The FP scheduler win on blur (39.50 -> 29.44 cyc/px) should carry to
Structure from Motion**, and both score 1.

Camera has no carried accumulator, so its elements are independent -- yet `IW2/W16` = 1.09
against `IW2/W64` = 2.00. It is **window-limited**: 9 instructions per iteration against an
8-cycle FP latency needs ~2 iterations in flight. It also stores every iteration, which is
where the store buffer (P0) would tell.

### Workload shapes, from hardware traces

Thirteen GB5 workloads have been traced on the FPGA (`~/gb5-traces/`). Two tools read them:
`tools/trace-limit.py` (how much ILP is there) and `tools/trace-bp.py` (does the frontend
find it). Both are ceilings from a ~19k-instruction window, not predictions.

#### Limit study — relative throughput, `IW1/W64` = 1.00

| workload | IW1/W16 | IW2/W16 | IW2/W64 | IW4/W64 | limited by |
|---|---:|---:|---:|---:|---|
| pdf-rendering | 1.00 | **1.86** | 1.99 | 3.32 | **width**, shallow window |
| text-rendering | 0.99 | **1.84** | 1.98 | 3.81 | **width**, shallow window |
| clang | 0.98 | **1.79** | 1.97 | 3.65 | **width**, shallow window |
| text-compression | 1.00 | **1.79** | 2.00 | 3.76 | **width**, shallow window |
| html5 | 0.98 | **1.72** | 2.00 | 3.69 | **width**, shallow window |
| sqlite | 0.93 | 1.62 | 1.81 | 3.03 | width |
| image-compression | 0.96 | 1.48 | 2.00 | 3.51 | width, some window |
| aes-xts | 1.00 | 1.90 | 2.00 | 3.84 | width |
| **camera** | 0.93 | **1.09** | **2.00** | 3.12 | **window** — needs W64 before width pays |
| n-body-physics | **0.69** | 0.83 | 1.49 | 1.59 | **window + FP latency** |
| gaussian-blur | 0.69 | 0.70 | 1.61 | 1.75 | FP latency |
| structure-from-motion | 0.69 | 0.70 | 1.61 | 1.75 | FP latency (same kernel as blur) |
| machine-learning | 0.86 | 0.86 | 1.14 | 1.14 | **serial accumulator** — nothing helps |

Five workloads reach 1.7-1.9x at **`IW2` with today's 16-entry window**. Superscalar pays
them immediately, with no ROB growth. Camera is the opposite: 1.09 at W16 against 2.00 at
W64, so width buys it nothing until the window is deep enough to hold two iterations.

#### Frontend study — the BTB is the second finding

`tools/trace-bp.py`, replaying through the real 256-entry BTB and 8-entry RAS. A miss is
COLD (PC never seen) or CONFLICT (the index belongs to another PC); only conflict is bought
back by capacity.

| workload | CTI/insn | BTB256 hit | cold | confl | BTB1024 hit | taken-miss/insn |
|---|---:|---:|---:|---:|---:|---:|
| **html5** | 15.6% | 60% | 9% | **31%** | **81%** | **3.6% -> 1.8%** |
| text-rendering | 19.1% | 53% | 16% | **32%** | **72%** | 5.6% -> 3.1% |
| sqlite | 13.6% | 53% | 33% | 15% | 63% | 4.0% -> 3.1% |
| clang | 19.4% | 39% | **41%** | 20% | 48% | 7.0% -> 6.0% |
| pdf-rendering | 12.2% | 83% | 14% | 3% | 85% | 1.0% -> 0.9% |
| text-compression | 12.1% | 86% | 5% | 9% | 94% | 0.9% -> 0.5% |
| n-body / image-compression / **camera** | 4-11% | 96-100% | ~0 | ~0 | -- | ~0 |

**The RAS is not a problem: 91-100% on every workload that returns at all.** An earlier
reading of the raw traces suggested deep call nesting overflowing the 8-entry stack; that
was an artifact of an unbalanced trace window (more calls than returns visible) and the
replay does not support it. `RASB`=3 stays.

**DONE: the BTB is now 1024 entries** (`BTBB`=10). 4096 was built and REVERTED -- it is
better on every workload metric and it costs 434 ps of `probe_clk`; see "What 4096 cost"
below. taken-miss/insn
across the depths, and note that the traces CANNOT judge 4096 — a 19k-instruction window
holds 286-884 distinct branch sites, so a 4096-entry table is 5-14x the observed working
set and what it measures there is residual tag aliasing, not capacity. The real working set
over billions of instructions is larger, so this understates the depth:

| | 256 | 1024 | 2048 | **4096** |
|---|---:|---:|---:|---:|
| html5 | 3.6% | 1.8% | 1.2% | **1.1%** |
| text-rendering | 5.6% | 3.1% | 2.5% | **2.1%** |
| clang | 7.0% | 6.0% | 5.5% | **5.2%** |
| sqlite | 4.0% | 3.1% | 3.0% | **2.9%** |

**1024 is chosen because 4096 does not fit, not because it performs comparably.** That
distinction matters, because the performance evidence for the choice is weak and the timing
evidence is not.

ONE workload, 40 M cycles, tiny128 Linux boot -- which is not branch-heavy, so it is close
to the worst case for showing a BTB difference and cannot stand in for GB5:

| `BTBB` | entries | retires | vs 256 | BRAM |
|---|---:|---:|---:|---|
| 8 | 256 | 10 444 328 | -- | 1x RAMB36 |
| **10** | **1024** | **10 496 381** | **+0.50%** | 1x RAMB36 + 1x RAMB18 |

Later states of the same 40 M-cycle boot, for continuity (BTB 1024 throughout):

| state | retires | vs previous |
|---|---:|---:|
| + store buffer (`ec6ad3e`) | 10 513 562 | +0.16% |
| + load queue (`cb02868`) | 10 315 103 | -1.9% |
| + early start | **10 501 952** | **+1.81%** |
| 12 | 4096 | 10 510 154 | +0.63% | **6x RAMB36** |

Do NOT read "1024 captures most of 4096's value" out of that. The 1024-vs-4096 gap is 13 k
retires on a single non-branch-heavy boot, and the trace study that agrees with it is a set
of 19 k-instruction windows that by its own limits cannot judge a 4096-entry table at all.
Two weak measurements pointing the same way are still weak. What is actually established is
that 1024 beats 256 on this workload, and that 4096 costs 434 ps.

#### What 4096 cost — rule I1, and it was NOT the address fanout

Built at `7926354` and reverted. `probe_clk` **-0.434 ns, 880 failing endpoints**, against
**+0.000** for the same RTL minus this change. Reproduced under a second placer directive
(`AltSpreadLogic_medium`: -0.408), so it is not placement noise.

**No BTB, `apc` or predictor path appears anywhere in the failing set.** The prediction that
6 BRAMs would put ~72 pins on `apc[12:1]` and cost route on that path was simply wrong. What
failed is `m_addr -> u_iq_i/i_ps*` and `i_ps2 -> u_prf/mem_ie` — the scheduler wakeup and
the PRF write, 74% route, and both were already sitting at exactly 0.000.

That is rule I1 verbatim: *area anywhere buys congestion everywhere, and congestion is paid
in slack by whatever is already marginal, not by the block that grew.* I1's own worked
example is `u_prf/mem_ie`, which is one of the two endpoints that failed here, and its
recorded magnitude — 465 ps for deleting unreachable memory — is the same order as the 434
ps this cost for adding five reachable tiles.

**The BTB is not depth-limited by its own timing; it is limited by the slack the rest of the
design has to lend it.** Revisit 2048/4096 once the `m_addr -> scheduler` family has margin.

**UltraRAM was considered and rejected**, though the part has 64 unused blocks and one
URAM288 is exactly 4096x72. Two disqualifiers. The BTB's output register is the START of
the prediction loop (`btb_raw` -> tag compare -> `apred_v`/`pred_tgt` -> `apc` -> address
pin), which must close in one cycle; URAM's clock-to-out is worse than BRAM's and its
remedy is the optional output pipeline register, which adds a cycle and breaks ahead
prediction outright — `apc` is a ONE-cycle-ahead guess. And UltraRAM has no initialisation:
contents are undefined after configuration, where this BTB deliberately has no valid bit and
relies on configuration INIT to start every entry untagged. Hardware would boot with random
tags where simulation boots with zeros. Harmless in itself (a bogus entry is a mispredict
the exec-side compare catches) but a sim/hardware divergence in exactly the structure whose
safety argument is that the two agree. BRAM costs 6 tiles of 480; URAM saves five of them.

**Clang is the exception and the reason to check the split.** It is 41% COLD: its branch
working set is genuinely larger than any affordable BTB, so 256 -> 1024 buys it 9 points
where it buys html5 21. Clang needs width and cheaper redirects, not a bigger BTB.

#### The three traced-and-benched workloads

They want different things, and averaging them hides that.

| | Gaussian Blur | Machine Learning | Clang |
|---|---|---|---|
| bench | `workloads/blurbench` | `workloads/mlbench` | (trace only) |
| shape | 5-tap FIR, 4-deep serial `fadds`, independent taps | dot product, **one** accumulator across all iterations | no kernel; branchy pointer code |
| basic block | long | 9 instructions | **5.2 instructions** |
| mem ops | 31% `ST_MEM` | 41% | **41.2%**, of which `c.sdsp`/`c.ldsp` = **22.3%** |
| control transfers | rare | 1 per 9 | **19.4%**, 13.9% branches |
| what binds it | FP issue order (fixed: 39.50 -> 29.44) | **the frontend**, `FE_BUB` 53.7% (NOT the ROB: `ST_ROB` is 0.06%) | mispredicts + stack traffic |
| what would help | done | ROB depth; FP add latency | superscalar, store forwarding, cheaper redirect |

**Clang and Text Compression are the workloads that matter to Tommy personally**, and they
are the ones a scalar core helps least. At a 5.2-instruction basic block a scalar machine is
capped at 1 IPC and we are far below it. Two-wide issue is the ceiling-raiser, and its cost
is a second issue slot -- i.e. a second set of register read ports, the same bill that makes
a fourth functional unit look expensive today. Once it is paid for superscalar, a fourth
unit is nearly free.

**Clang is also the workload that wants store-to-load forwarding**, which is postponed
(above) on the strength of the FP workloads. 22.3% of its instructions are stack spill
immediately followed by reload of the same slot; with no forwarding every reload waits for
its store to commit. Revisit the postponement once P0 exists, because the store buffer is
where forwarding would live.

**And it is the workload where the redirect cost bites hardest**: 13.9% branches against a
measured 54.4 cycles of `FE_BUB` per redirect is one mispredict per ~7 instructions at a
price that swamps everything else. That puts P2 and the deferred rename walk-back back in
contention, specifically for Clang and Text Compression rather than for the score.

### P0 -- load queue + store buffer (which is also multiple outstanding loads)

**Store buffer and load queue are BUILT** (`ec6ad3e`, `cb02868`, plus the early start in
section 8); multiple outstanding loads is not, and is blocked on the D$ read below. What
follows is the original motivation, kept because the measurement is still the argument.

This is one piece of work, not two. `Area-Efficient-Scalar-OoO.md` 11.1 has the design:
address calculation issues freely; stores take a store-buffer slot indexed by store-seqno
and commit when their data is ready; loads insert into a program-order load queue and
access memory out of order once no older pending store can alias, starting with the
cheapest correct test ("assume any older pending store aliases").

It subsumes three separate entries that were on this list:

- **multiple outstanding loads** -- the load queue IS the tag space the D$ already reserves
  (`rd_tag`/`rd_resp_tag`, plus 2 free bits in the LSU tag).
- **stores and AMOs block M** -- a store that commits from a buffer when its data arrives
  does not hold an execute stage waiting for `rs2`.
- **`u_iq_l` is in-order** -- with ordering moved to the access, the scheduler no longer
  carries it.

Measured motivation, `workloads/ldbench`, entirely L1-resident:

| | cyc/load | with the load queue (`cb02868`) | + early start |
|---|---:|---:|---:|
| latency (pointer chase) | 5.00 | 6.00 | **5.00** |
| throughput (8 INDEPENDENT streams) | 4.00 | 5.00 | **4.00** |
| **overlap** | **1.24x** | 1.19x | **1.24x** |

The queue cost a cycle on both and the early start gives it back, so the argument below is
unchanged: eight independent streams overlap 1.24x. And `ST_MEM` is 54.7% of full-suite GB5
cycles.

Unlike P3a, the units here are genuinely FSMs (`rv_cache`: `S_IDLE -> S_CHECK -> deliver`;
`smolrv64_lsu`: `S_IDLE -> S_LD -> S_LD2`), neither accepting a request until idle again --
~2 cycles each, which is the measured 4. So this needs a pipelined hit path as well as the
queue. Batch it alone: it wants its own bisect.

**The pipelined hit path is NOT a cache-only change, and it is not step 1.** Self-looping
`S_CHECK` is the right shape and is written -- `wip/cache-selfloop` (`baf96df`) -- but on its
own it makes the cache do **every lookup twice**: `ldbench` `D$acc` 1 600 000 -> 3 200 000 for
the same 1 600 000 loads, Linux boot -2.9%, `saxpybench` `FE_BUB` 0% -> 43%. The cause is the
requester handshake, not the FSM:

    c_rd_req  = (dmem_ren | c_rd_pend) & ~dc_rv_ok & ~is_dev_r      (dc_rv_ok <- dc_rd_valid)
    ic_rd_req = (fb_wantv | fb_pend) & ~ic_rd_valid

Both drop their request on `rd_valid`, which is a **registered** output, so in the very
`S_CHECK` cycle the response is being produced the request is still asserted and the
self-loop accepts it again. The implicit contract "hold the request until the response
matches" only worked because the cache stayed busy for exactly the round trip. A pipelined
port needs the explicit one: an `rd_ack` combinational from the accept, with every requester
splitting its single pending bit into REQUESTING (cleared on ack) and OUTSTANDING (cleared on
the response) -- rule D5, and the same defect `wip/pipelined-loads` hit as `pt_ack`.

**And even with the ack it buys nothing alone**: the LSU, the I$ alignment adapter (Stage 2;
the run-ahead fetch buffer before it) and each PTW walk are all single-outstanding, so each
drops its request on the ack with no next address ready and `S_CHECK` never actually
self-loops. Design the ack together with the
multi-outstanding LSU that consumes it, not ahead of it.

**ATTEMPTED as C4a step 2 (2026-09-19), NOT YET CLOSED -- uncommitted.** Once C4a step 1 gave
the LSU multiple outstanding loads to feed it, `rd_ack` already existed as the documented
combinational accept (§8) and the requester side had already become ack-based
(`c_rd_want`, cleared on `lsu_rd_ack` not on `dc_rv_ok`) as part of the tagged fast path, so
neither of this section's two original blockers needed separate work -- only the door's own
`S_CHECK` admission (`chk_rd`) was missing, and it built cleanly and correctly (the lockstep
never diverged at any point in this section). Two things this section did NOT anticipate,
both now documented in the handoff's C4a step 2 section:

1. The self-loop exposed a pre-existing, previously-harmless priority rule (reads always win
   a tie for the door) as a real fairness bug once reads could claim every cycle instead of
   every other one -- fixed with `chk_rd & ~wr_req` (rule I13).
2. **`chk_rd` needs `hit` to decide `rd_ack`, and this is the FIRST time this cache's accept
   has ever depended on the tag compare** -- every existing admission path (`S_IDLE`,
   `fin_wr`) was pure state/registered-field decode. `hit`'s ~13-18 levels fan out through
   `rd_ack` into the whole SoC (the LSU and everything downstream of its decisions), invisible
   to an isolated OOC check of `rd_ack` as a lightly-loaded port. Full IW=3 build: WNS -0.270
   (VIOLATED; IW=2 closes at +0.037). This is a second instance of exactly the risk P0 warned
   about in general ("the accept stays register-decoded") landing on the one signal that
   cannot tolerate it. Not yet resolved -- see the handoff for a candidate redesign (a
   dedicated one-cycle delivery state, `fin_wr`-shaped, that needs no `hit`) and for a tried
   and reverted attempt (splitting only the register capture, not the accept, off of `hit` --
   wrong, because `pipe_hold`'s hold cases need the captured fields to survive a retry
   unchanged). wip/c4a's committed tip stays C4a step 1 (493dc343) until this closes.

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

`u_iq_f` (8 entries, `NSRC`=3, reordering) feeding stage F, a one-entry execute stage
parallel to M. Measured: `blurbench` 39.50 -> **29.44** cycles/pixel (-25%). `fpbench` is
unchanged at 2.25 cyc/op throughput and 8.00 latency, correctly -- it was already FPU-bound
rather than issue-bound, so the gain lands exactly where the trace predicted and nowhere
else.

One hazard this exposed, recorded because it will return: an FP op still in fpnew's
pipeline when a squash happens writes back after rename has rolled its physreg away.
`smolrv64_pending` caught it as *"writeback to p352, which was not pending"*. `fp_unit` already
forwarded `flush` to `fpnew_top`; the core tied it to 0. Flushing EVERY in-flight op on
redirect is correct only while `head_block` holds -- a redirect fires only at ROB head, so
anything in flight is younger by construction. **Removing `head_block` (P2) requires
replacing this with an epoch tag first.**

### P4 -- Machine Learning scores 0 -- ANSWERED 2026-09-07: the ISA, not the core

GB5 aggregates a category as a **geometric mean**, so this single 0 (0.01 images/sec)
zeroes the entire Floating Point score. The per-subtest counter trace of the 2026-09-07 run
(`tools/gb5-trace-stack.py`) shows IPC 0.36, 0.04 traps per thousand instructions, LSU 61%,
FPU 21%, D$ miss 4.4%: no pathology. The rate is the scalar instruction count, ~6 G per
image against the reference machine's SIMD. Gaussian Blur and Structure from Motion read the
same way. No IPC item reaches these three; the vector extension would.

### P5 -- frontend run-ahead (`IW>=2`)

`FE_QUE` is **24% of cycles on the AES kernel** and unchanged between HW=2 and HW=4, so it
is not an alignment artifact: the frontend makes at most one instruction per cycle and the
backend consumes one per cycle, so no hiccup is ever recovered. Only 5.3% on the full
suite, hence below the memory and FP items despite being large on integer code.
`sha256sum` at 41.8% frontend is the same effect.

### P6 -- shrink the schedulers to 8/8 -- **REVERTED, it was a regression**

`aesbench` at `SMOLRV64_HW=4` showed an optimum at 8/8 (122.18 cycles/byte against 10/12's
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

### P8 -- stores and AMOs still block M -- and the STORE QUEUE is the measured wall

Stores no longer block M (they translate and leave, §8); what they block is DISPATCH, by
filling the queue: >= 16 M of 60 M boot cycles held on `sq_d_ready` at `DDR_LAT`=4 and at
80, mean occupancy 3.89 of 4 (P0's `SB-WHERE2` table, 2026-09-04). This is now the largest
single stall in the design and the next IPC item.

**Measured, `workloads/stbench` (L1-resident, `ipc/multi-loads`, 2026-09-04):**

| loop | cyc/op | insn/op | note |
|---|---:|---:|---|
| 8 independent stores per iteration, different lines | **5.88** | 1.88 | the drain rate of `smolrv64_sq`; loads do 2.25 (ldbench) |
| 4 stores + 4 loads, different lines (the boot's 1:1) | 4.50 | 2.38 | 9.0 cycles per store+load pair |
| store then load of the SAME word, 4 pairs/iteration | 11.75 | 4.75 | `ST_MEM` 7.00 per pair: the load waits for the store to commit |

So a store that HITS costs the port ~6 cycles: it leaves the queue only at the ROB head,
the LSU parks in `S_ST` until `mem_wready`, and the cache takes the write as a solo request
(`S_IDLE -> S_CHECK -> S_FIN -> ack`) with nothing accepted meanwhile. A queue of four at
six cycles each is what SB-WHERE2 sees full. The shape of the fix is the one the loads
just took: the store leaves the queue when the cache ACCEPTS the write, not when it
completes, and the cache takes a write under a fill (a write is solo today because it
shares the fill machine's window registers, §8) and pipelines it behind the next lookup.
The hazard that comes with it -- a load to a line whose write is still in the cache's
pipeline -- is a one-entry compare, and the alias matrix in `smolrv64_lq` already holds a load
behind an older store in the QUEUE; what moves is where "committed" is decided.

**2026-09-04, the senior store queue (item 3 of `docs/PLAN-2026-09-05-ipc.md`), the same
bench on main, before and after:**

| loop | before | after |
|---|---:|---:|
| 8 independent stores per iteration | 5.88 | **5.00** |
| 4 stores + 4 loads, different lines | 4.88 | **4.50** |
| store then load of the SAME word | 11.75 | 11.75 |

The 0.88 cycles per store was the ROB head: a store's slot completed only when the cache
took its write, so every store held the head for the cache's write and the window filled
behind it. Now the ROB's irrevocable pointer (§6) commits the store as soon as everything
older is done, the head retires it, and the queue drains it behind retirement. The 5.00
that remains is the cache's own write cost -- accept, `S_CHECK`, `S_FIN`, the registered
ack, the LSU's return to idle -- which is the next item. FWD is unchanged because the
dependent load waits for the drain either way; a first version that released a store one
cycle before it could drain measured 12.75 there, which is why a release at the head
drains in its own cycle. tiny128 boot at 60 M cycles: +0.22% retires (11,604,336): the
boot's store stall is the cache's write path (SOLO writes, none under a fill), not the head.

**2026-09-05, plan item 4a+4b: the store leaves the LSU at the D$'s ACCEPT, and the D$'s door
is open in a plain write's last cycle.** The same bench:

| loop | senior queue | + 4a | + 4b, HW=4 | + 4b, HW=8 |
|---|---:|---:|---:|---:|
| 8 independent stores per iteration | 5.00 | 3.00 | 3.00 | **2.31** |
| 4 stores + 4 loads, different lines | 4.50 | 3.88 | 3.88 | 3.25 |
| store then load of the SAME word | 11.75 | 9.75 | 9.75 | 9.75 |

4a is the LSU (§8): a plain store is done when the D$ takes it, not when it acks. 4b is the
cache's door (§8, `fin_wr`). At HW=4 the 3.00 that remains is the FRONTEND, not the store
path: stbench's loop is 15 instructions of mixed 16- and 32-bit code per 8 stores, and at
8 bytes per fetch window the 32-bit stores straddle (febench's 2-byte-offset case, 0.40
IPC). In that loop M is empty 63% of the cycles, dispatch is held 0.05%, and the
`SQ-DRAIN` line of `tb_smolrv64_linux` shows the queue holding stores that cannot drain yet --
allocated at dispatch, waiting for their translate pass through a starved M -- while the LSU
sits idle and the door is free. The 16-byte window (item 2, build K) takes it to 2.31; the
floor is 2.00. tiny128 boot at 60 M cycles: 11,604,336 -> 13,291,779 (4a, **+14.5%**) ->
13,466,847 (4b, +1.3%): the boot's store stall was the LSU holding every store to its ack
while the queue filled behind it, more than the cache's write cost itself.

**2026-09-05, plan item 4c: the write miss is completed by the fill machine (§8), so a plain
write is no longer solo.** stbench is unchanged (L1-resident, no misses); tiny128 boot at
60 M cycles 13,466,847 -> 13,783,921 (+2.35%): the boot's remaining store cost is the
misses themselves (a page clear or copy is a miss per line), and a write miss no longer
closes the cache to every load behind it for the length of the fill.
