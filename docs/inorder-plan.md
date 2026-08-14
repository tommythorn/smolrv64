# In-order pipelined RVA22S64 core — design plan

Status: in progress (branch `inorder`, 2026-08-13). Goal: a correct in-order
pipelined core reusing the sharded-OoO frontend and execution leaf modules,
replacing rename/schedule/CPR with a 3-stage in-order pipe and a simple blocking
LSU. **Correctness first; Fmax is a later exercise** (TT, 2026-08-13).

## Why

The OoO core (`probe/`) is a large machine whose recovery/ordering machinery
(rename, scoreboard, CPR checkpoints, unified store buffer + load queue,
replay-to-solo fault delivery) dominates both the area and the debug cost. An
in-order pipe keeps the parts that were expensive to *build* and cheap to *run*
— the frontend (fetch/align/RVC/decode/BTB), the execute leaf units, the caches,
MMU, CSR file, and SoC — and throws away the parts that exist only to recover
from speculation past a single instruction.

## Decisions (settled with TT, 2026-08-13)

| Question | Decision |
|---|---|
| Width | **Scalar, IW=1.** No cross-slot hazard logic; `decode_xslot` unused. |
| Layout | New `inorder/` dir. **Shared modules compile out of `../probe` unmodified** — `inorder/` never edits a `probe/` file; anything that must change is forked in. |
| Multi-cycle ops | **Stall the whole pipe.** D$ miss, divide, FPU, page-table walk all freeze F/X/M. No scoreboard, no late writeback. |
| Milestone | riscv-tests → cosim vs Simmerv → Linux boot in sim → FPGA/Ubuntu. Integer + CSR/trap/MMU first, FP wired in after. |
| Branch prediction | **Keep the frontend's** (`predictor.v` + BTB/bimodal/RAS) as-is. |
| Misalignment | Hardware, via `cache.v`'s internal two-phase (line-crossing) lookup. No trap-and-emulate. |

## Pipeline

Three stages. The **M stage is the single commit point**: every architectural
side effect (memory write, CSR write, regfile write, trap delivery, branch
redirect) happens there and nowhere else. That one rule is what makes precise
traps free — there is no younger instruction that can have already changed state,
and no older one that can still trap.

```
  F : pc_q -> iMMU -> I$ window -> aligner -> RVC expand -> decode  --> [IR reg]
  X : regfile read (+ M-stage bypass) -> exec_alu (ALU/AGU/compare)
      -> branch_unit (computes redirect+target, does NOT apply it)   --> [EX reg]
  M : dMMU -> D$ access | CSR update | trap | branch redirect | RF write
```

- **F** is `probe/fetch.v` + `predictor.v` + `decode_slot.v` as one combinational
  cloud terminating at the IR register — the same shape `frontend.v` has today,
  with `decode_rename` replaced by a plain register.
- **X** is one combinational cloud: RF read → `exec_alu` → `branch_unit`.
- **M** owns the LSU, `csr_file`, the trap mux, and the RF write port.

### Why branches resolve in M, not X

`predictor.v` snapshots its speculative `{ghr, ras, ras_ptr}` at `create` (one
cycle after the fetch handshake) and reads the per-checkpoint predict details
`pdet[res_ckpt]` at resolve. Resolving in X would put `create` and the resolve in
the *same* cycle for the same instruction: `pdet[cur]` would not yet be written,
and a mispredict would suppress the very snapshot the rollback restores.
Resolving in M restores exactly the OoO timing relationship (fetch T → create T+1
→ resolve T+2), so **`predictor.v` is reused verbatim**.

Cost: a mispredict costs 3 cycles instead of 2. Correctly-predicted branches cost
nothing, which is the case that matters.

### Checkpoints, in-order

The predictor's checkpoint interface is kept, degenerate: **one checkpoint per
instruction**, `cur` a rotating counter incremented on each X-entry, `create` =
an instruction entering X, `rollback_idx` = the redirecting branch's index + 1.
NCHK=4 covers the ≤2 instructions ever in flight past a branch.

### Bypass

Exactly one level: **M → X**. An instruction two ahead has already written the
regfile by the time X reads it, so no second level exists. (`probe` needed 1- and
2-ahead forwarding because its RR|EX split put two stages between issue and
writeback.)

### Hazards / stalls

- **Load-use**: the D$ is synchronous, so a load occupies M for ≥1 extra cycle;
  the pipe stalls and the M→X bypass delivers the value when it lands. No
  separate interlock needed — the stall *is* the interlock.
- **Serializing ops** (`decode_exec.is_serialize`: CSR writes, system ops,
  fences): nothing enters X behind one until it has left M. Removes every
  CSR-value / privilege / satp hazard by construction.
- Everything else is single-cycle in X and needs no interlock.

## Traps

All delivered in M, as a poison bit carried down the pipe:

| Source | Detected | Carried as |
|---|---|---|
| Instruction page/access fault | F (iMMU `t_fault`) | poison + cause/tval in the IR reg |
| Illegal instruction | F (`decode_slot.illegal`) | poison |
| Load/store page fault | M (dMMU) | direct |
| ecall / ebreak / mret / sret | M (`csr_file`) | direct |
| Interrupt | F, as the `irq_inject` pseudo-op (reused from `fetch.v`) | a solo SYSTEM op that traps in M |

This replaces ~600 lines of `backend_top` (pend_iflt, dflt two-phase
replay-to-solo, ill_v latch, devld solo window, amo_gap, rollback priority mux)
with a poison bit and a mux in one stage.

## Module inventory

**Reused unmodified from `probe/`:** `fetch.v` `aligner.v` `rvc_expand.v`
`decode_slot.v` `decode_operands.v` `decode_exec.v` `decode_fp.v` `predictor.v`
`btb.v` `exec_alu.v` `branch_unit.v` `mul3.v` `divider.v` `fp_unit.sv`
`csr_file.v` `mmu.v` `cache.v` `l2_arbiter.v` `clint.v` `plic.v` `ddr_hpm.v`
`flopwrap.v`, plus `src/alu.v` and the SoC IP (virtio, SD, ethernet, DDR).

**Dropped:** `renamer*.v` `rename_shard.v` `freelist.v` `sched_*.v`
`commit_ctl.v` `decode_xslot.v` `decode_rename.v` `decode_stage.v`
`exec_shard*.v` `exec_bundle.v` `rf_shard.v` `lsu.v` `backend_top.v`.

**New in `inorder/`:**
- `ino_frontend.v` — `fetch` + `predictor` + `decode_slot` → IR register.
- `ino_regfile.v` — 64×64 unified arch RF (int 0–31, FP 32–63, x0 hardwired 0),
  3R1W, LUTRAM.
- `ino_exec.v` — X stage: RF read, bypass mux, `exec_alu`, `branch_unit`, M-units.
- `ino_lsu.v` — M stage memory: one `mmu`, D$ request/response, byte
  extract/sign-extend/NaN-box, AMO read-modify-write, LR/SC reservation, fence.i.
- `ino_core.v` — the three stages, stall control, trap mux, CSR file, RF write.
- `ino_soc_top.v` — fork of `soc_top.v` driving `ino_core` (2 PTW ports, not 3).

## Build order

1. ~~`ino_regfile` + `ino_exec`~~ — **done**.
2. ~~`ino_frontend` + IR register~~ — **done**.
3. ~~`ino_core` integer subset~~ — **done**: rv64ui-p 52/52.
4. ~~`ino_lsu`~~ — **done**: rv64um-p 13/13, rv64ua-p 19/19, rv64uc-p 1/1.
5. ~~CSR/trap/MMU~~ — **done**: rv64mi-p 15/16, rv64si-p 7/7, ssvnapot-p 1/1, and
   the whole Sv39 `-v` set (rv64ui/um/ua/uc-v) 85/85. **193/194 overall.**
6. ~~FP (`fp_unit.sv`)~~ — **done**: rv64uf/ud-p+v pass, and with it the last
   integer holdout (`rv64mi-p-csr` test 12). **240/240 overall.**
7. ~~Cosim vs Simmerv~~ — **harness done**, 130/131 `-p` tests lockstep clean.
8. ~~Linux boot in sim~~ — **done, to a login prompt.** `ino_soc_top` +
   `tb_ino_linux.v` boot OpenSBI v1.8.1 and the Ubuntu 6.x kernel all the way to
   `smolrv64 login:` on the tiny128 initrd workload, matching
   `workloads/tiny128/golden-output.txt` line for line (syslogd / klogd / sysctl /
   random seed / network, all OK). ~5.3 CPI; login at ~1.1B cycles. Linux cosim
   (`run-ino-cosim-linux.sh`) separately lockstepped 171M retirements against
   simmerv with zero divergence.
9. **Next:** FPGA bring-up (RK platform wrapper), then cycle time.

Harnesses (`inorder/tb_ino_riscv.v` serves the first three):
- `run-ino-tests.sh [class ...]` — iverilog, `fp_unit_stub.sv`. Fast **integer**
  regression.
- `run-ino-vl.sh [class ...]` — verilator, the **real CVFPU**. This is the flow
  that covers F/D: fpnew uses SystemVerilog concurrent assertions iverilog cannot
  parse.
- `run-ino-cosim.sh <test> | -a [class ...]` — verilator + simmerv lockstep.
- `run-ino-linux.sh` — verilator, `ino_soc_top` + `tb_ino_linux.v`: boots OpenSBI +
  Linux on the tiny128 initrd workload. `CYC=0` runs unbounded (wrap in `timeout`).
  Pass `INITRD=<uncompressed cpio>` to skip ~1B cycles of in-kernel zstd decode.
- `run-ino-cosim-linux.sh` — the same boot, lockstepped against simmerv.

## `ino_soc_top` — a fork, deliberately

`ino_soc_top.v` is a copy of `probe/soc_top.v` retargeted to `ino_core`. Everything
outside the core instance — MMIO routing, CLINT/PLIC/UART, the virtio bridge, the
I$/D$ adapters, the PTW-through-D$ adapters, `l2_arbiter`, local SRAM, the DDR line
port — is carried over verbatim, so **keep the two in sync when touching those**.
The deltas are small and structural:

- `backend_top` → `ino_core` (scalar: `HW=2`, no `POOL`/`PBITS`, no per-shard
  writeback observation bus).
- **Two page-table walkers, not three.** The OoO LSU runs separate load and store
  walkers because loads and stores translate in parallel; the in-order LSU has one
  memory op in flight, so one data walker serves loads, stores and atomics.
- `commit` → `retire`.

This is the one place the "never fork, just reference `probe/`" rule had to give:
`soc_top` names its core instance directly, so retargeting it means editing it.

## Cosim

`ino_core.v`'s `INO_COSIM` block emits the same `probe_retire()` DPI stream the
OoO core does, so **`probe/probe_cosim.cpp` is reused unmodified**. The RTL side
is what collapsed: the OoO harness needs ~250 lines — a 40-deep FIFO to rebuild
program order from out-of-order commit, per-entry value-ready tracking (an ALU op
commits on the issue count, *before* its writeback), squash-by-seqno tail
truncation, and a by-PC search to convert a committed entry into its own trap.
In-order needs none of it: M retires one instruction per cycle in program order
and its rd value is live on the writeback bus that same cycle.

The one thing that carries over is the OoO harness's **1-cycle emission lag**. A
retiring instruction's CSR writes land on the edge that ends its M cycle, and
`probe_cosim` wants `mepc` *after* the retire (post-`mret`, or post-trap-capture),
so the record is registered and emitted one cycle later with `mepc` read live.
Anything a trap changes on that same edge — privilege above all — must therefore
be **registered at retire time**, not read live.

Two environment notes: `libsimmerv_cosim.a` carries a vmnet shim, so the link
needs `-framework vmnet` on macOS; and `tb_ino_riscv.v` ends its loop on an
explicit `done` flag rather than relying on `$finish`, which under Verilator
completes the current time slot (the loop ran on and printed a bogus TIMEOUT
*after* PASS).

## FP integration notes

FP lives in M with the LSU and mul/div, and in-order deletes most of what the OoO
core needs around it:

- **No zombie/abort/drain latch.** M is the commit point, so an in-flight CVFPU op
  can never be squashed. `probe/exec_shard.v` needs `fp_zomb` (a squashed op must
  *drain*, never flush — a mid-op flush corrupts fpnew's internal state) plus an
  abort-by-seqno; neither exists here.
- **No latched per-op FP state.** The M-stage instruction cannot change while the
  FPU runs, so every `decode_fp` output — including `fp_dst32` for NaN-boxing an
  FP32 result — is stable and read combinationally off `m_insn`. The OoO core has
  to snapshot them at issue.
- **`mstatus.FS` -> Dirty is just `retire & m_rd_v & m_rd[5]`**: a retiring write to
  arch 32..63. That covers FP arith, the in-core FP writers and FP loads, and
  excludes FSW/FSD (which write memory, not FP state). Commit-gated by
  construction, where the OoO core routes an issue-time flag through `commit_ctl`.
- Operands need no special path: the regfile is the unified 64-entry file the
  decoders already address, so `rs1/rs2/rs3` *are* the FP sources.
- ~~`probe/mmu.v` does not implement Ssvnapot~~ — **fixed in `probe/mmu.v`**, so
  both cores get it. Found via this core: `mmu.v`'s `leaf_pa()` used the PTE's PPN
  verbatim for a 4 KiB leaf with no `pte[63]` (N) handling, so a 64 KiB NAPOT page
  resolved to the NAPOT-encoded PPN instead of substituting the VA's `VPN[3:0]`.
  The OoO core "passed" `rv64ssvnapot-p-napot` only because `probe/tb_riscv.v`'s
  2 MiB memory puts the relevant PA out of range — the compare read X and iverilog
  did not take the fail-branch. `inorder/tb_ino_riscv.v`'s 4 MiB makes the address
  real and exposed it.

## RESOLVED: the "stall before `/init`" was not a bug

It was the kernel **decompressing the initramfs in software**, working exactly as
intended. `workloads/tiny128/tiny128.cpio` is a **Zstandard** stream (magic
`28 b5 2f fd`), 3.3 MB compressed → 10,305,536 bytes out. Modern kernels unpack the
initramfs asynchronously and call `wait_for_initramfs()` immediately before
`Run /init`, which is precisely where console output stopped. The two hot PC
regions confirm it: `ffffffff80515748` is `memcpy` (byte-align prologue then a
128-byte `ld`/`sd` block loop) and `ffffffff802e53xx` is the zstd decoder
(byte loads, shifts, 3-byte sums, table indexing). A 5.3-CPI core simply needs a
lot of cycles for ~10 MB of software decompression.

Feeding the **decompressed** cpio instead boots all the way through:

```
[    8.001883] Freeing initrd memory: 10064K
[    8.092432] Run /init as init process
Starting syslogd: OK
Starting klogd: OK
Running sysctl: OK
Saving random seed: OK
Starting network: OK

Welcome to Simmerv/SmolRV64's tiny 10 MiB image
smolrv64 login:
```

And the DTB agrees this is the intended configuration: `tiny128-cosim.dts`
declares `linux,initrd-start/end = 0x9f52c000 .. 0x9ff00000` — exactly 10,305,536
bytes, the *decompressed* size, flush-packed against the DTB base. Use
`INITRD=<uncompressed cpio>` for fast iteration; the compressed file also boots,
it just spends ~1B extra cycles in zstd first.

This also cleared the interrupt-path question the cosim could not: once userspace
runs, `uart_ier` = 5/7, `uart_irq` = 125740 and `plic_seip` = 18177 — the
**interrupt-driven UART TX path works**, which is the thing `soc_top.v` flags as
load-bearing. Per-process `satp` values (ASIDs changing per switch) confirm real
U-mode multitasking.

## Superseded: what the investigation looked like before that

`run-ino-linux.sh` gets all the way through kernel init — VFS/rootfs mount,
clocksource switch, TCP hash tables, initramfs unpack, io schedulers, PLIC
registration, the 8250 driver taking the console from the SBI bootconsole, and the
kernel's misaligned-access probe (*"scalar unaligned word access speed is 5.86x
byte access speed (fast)"* — hardware misalignment support working) — and stops
after `clk: Disabling unused clocks`. The golden log for this workload
(`workloads/tiny128/golden-output.txt`) continues immediately from there through
`Run /init as init process`.

**It is not wedged, and it is not the UART.** Instrumented over ~1B further cycles:

| Signal | Observation |
|---|---|
| retires | still ~200k per 1M cycles — the core is executing |
| `irq_inject` | climbing steadily (6093 → 7390) — **timer interrupts are being delivered** |
| `uart_ier` | **0** — the 8250 driver has not enabled UART interrupts, so this is *not* a missed-THRE console wedge |
| `uart_irq` / `plic_seip` | 0, consistent with the above |
| priv / satp | stays S-mode, Sv39 on (`satp=80000000000810c3`) — never reaches U-mode |
| fetch VA | spins around `ffffffff805158xx` and `ffffffff802e53xx–55xx` |

So the kernel is alive, taking interrupts, and looping in S-mode without making
forward progress toward `/init`.

### Linux cosim result: the retire stream is CORRECT

`run-ino-cosim-linux.sh` locksteps the whole SoC against simmerv.
**171,000,000 retirements, zero divergence** — clean through the stall region and
well past it (c=879M, spinning at `ffffffff80515838`). So the core is *not*
architecturally wrong: every instruction it retires, and every trap it takes,
matches the golden model. That rules out wrong ALU/branch results, wrong memory
values or ordering out of RAM, wrong traps, and (mostly) wrong CSR behavior.

**Which leaves the cosim's two deliberate blind spots**, both consequences of
making the reference *follow* the DUT:

- **Interrupts.** `probe_cosim.cpp` calls `simmerv_set_forced_interrupt(...)` with
  whatever interrupt the DUT took, so the reference takes exactly that one and no
  other. An interrupt the DUT should have delivered but *never did* is therefore
  invisible to the compare — both sides simply don't take it.
- **MMIO / device load values.** `simmerv_arm_load_value(g_ctx, dut.rd_val)` makes
  the reference adopt the DUT's load result, so a wrong value read back from a
  device register cannot diverge either.

The kernel spinning in S-mode while timer interrupts *do* arrive, with the retire
stream provably correct, points at one of those two. Next diagnostic: log the
cause of each injected interrupt and the duty cycle of `csr_irq_v` vs
`irq_inject`/`inject_inflight` in `ino_core`, to see whether an interrupt is
pending-but-never-injected (an injection-gating bug) rather than never-pending
(a CLINT/PLIC or `mip`/`mie` bug).

(One sampling caveat for whoever picks this up: the `pa=` column in the progress
line is `imem_addr`, the iMMU output, which is only meaningful when the translation
is ready. Samples taken mid-walk show a stale PA — e.g. `va=ffffffff80515748
pa=0000000080515748` alongside `va=ffffffff8051582c pa=000000008071582c` for the
same 4 KiB page. That is a sampling artifact, not a translation inconsistency.)

## Open: unimplemented CSRs do not trap (shared `probe/csr_file.v`)

The first cosim sweep found one divergence in 131 `-p` tests:
`rv64mi-p-breakpoint` retire #81, `csrrs x0, tdata2, a1` (CSR **0x7A5**, a
debug-trigger register). Simmerv raises illegal-instruction (cause 2); the DUT
retires the instruction.

**Pre-existing and shared, not introduced here.** `csr_file.v`'s `csr_unimpl` is
a narrow *blacklist* — `MTOPI` plus the `stateen` group — and the read path ends
in `default: rdata = ... : 64'd0`, so any unlisted CSR address silently reads 0
and drops its write. The OoO core uses the same file and behaves identically;
riscv-tests does not catch it because `rv64mi-p-breakpoint` passes either way.

Matching simmerv means **default-illegal**: simmerv's CSR enum stops at
`Tinfo = 0x7a4`, and anything not in the enum traps. That is the architecturally
correct behavior, but it inverts the policy for the whole address space, so any
CSR that OpenSBI/Linux currently touches and that `csr_file` silently tolerates
would begin trapping. Not a change to make blind — it wants a Linux-boot cosim
run behind it, which is the next milestone anyway.

## Verified-by-test notes

- **`ino_lsu` atomics must use the ALIGNED PA for the write.** The D$ port is
  byte-address-relative in both directions, and an AMO's `a_wdata`/`a_wmask` are
  built relative to the containing 8-byte word (`.W` picks its half with
  `addr[2]`). Writing at the raw PA lands a `.W` half four bytes past its target;
  every AMO test passes anyway (their addresses have `addr[2]==0`) and only
  `rv64ua-p-lrsc`'s barrier, whose counter sits at `+4`, catches it.
