# Cosim oracle — cross-machine handoff (2026-06-03)

Self-contained resume notes for continuing the tiny128 lockstep-cosim
divergence-hunt on another machine (e.g. MacBook Air M5). This file lives
in the repo; the Claude memory dir does NOT transfer between machines, so
everything needed is duplicated here.

## Goal

Use the tiny128 cosim as a fast lockstep oracle against simmerv to find
and fix DUT/model divergences, pushing the oracle as deep into Linux
boot/execution as possible. End goal: **run indefinitely without
divergences.** simmerv is NOT golden — when a divergence is genuinely a
model bug, fix simmerv; when it is an unspecified/HW-timing-dependent
quantity, make the cosim glue take the DUT's value.

## Two repos involved

- `smolrv64` (this repo) — the DUT (RTL `src/smolrv64.v`) + Verilator
  cosim glue (`src/sim_main.cpp`). Branch `dev`, remote `origin`
  (codeberg PocketDogDensity/smolrv64).
- `simmerv` (`~/simmerv`, sibling checkout) — the Rust ISA reference
  model. Branch `main`, remote `codeberg` (PocketDogDensity/simmerv).
  The cosim links `libsimmerv_cosim.a` built from it.

Both must be checked out side-by-side (`../simmerv` relative to this
repo's `src/`). The smolrv64 `src/Makefile` builds the simmerv lib via
`cargo build --release -p simmerv-cosim` (it has a proper .rs dep), but
**Verilator `--build` does not relink when only the .a changes** — after
editing simmerv, you MUST `touch src/sim_main.cpp` before `make` or the
binary keeps the stale model. Verify with binary mtime > lib mtime.

## Build + run (exact commands)

```sh
# 1. (after any simmerv edit) rebuild the model lib:
cd ~/simmerv && cargo build --release -p simmerv-cosim

# 2. relink the cosim binary (touch forces the relink):
cd ~/smolrv64/src && touch sim_main.cpp && make smolrv64-tiny128-cosim

# 3. run against the tiny128 workload images:
W=~/smolrv64/workloads/tiny128
./smolrv64-tiny128-cosim +even=$W/mem.even +odd=$W/mem.odd \
    +sram_even=$W/mem.even +sram_odd=$W/mem.odd +rf=$W/rf.hex \
    > /tmp/cosim.log 2>&1
```

The DUT `$readmemh`s `+even/+odd` into AXI memory and `+sram_even/
+sram_odd` into the boot SRAM (both point at the same 26 MB image);
`sim_main.cpp` loads the same image into simmerv. On a divergence the run
aborts and prints a `*** cosim MISMATCH at retire #N ***` block with a
DUT-vs-REF history ring. It prints `cosim: <N> retirements ok` every 1 M.

The 26 MB `workloads/tiny128/mem.even` / `mem.odd` / `rf.hex` are
gitignored build products — regenerate on the new machine if absent (see
`workloads/tiny128/run-tiny128.sh` / the tiny128 build flow) or copy them
over. They are required to run.

Quality gates (must stay green): riscv-tests `(make)|&grep 'Test
Passed'|wc -l` == 240; FPGA timing WNS (baseline +0.044 ns); cosim no
divergence.

## Progress this session: ~3.98 M → ~168.4 M retirements

Starting point was a divergence at the satp boot-trampoline (~3.98 M,
already fixed/committed as `bcef83f`). This session fixed five more; with
the first four the run reached ~168 M (steady-state kernel idle/sched
loop — clean from ~4 M through 168 M), then hit the SEIP timing
divergence (#6). In order:

1. **satp trampoline (~3.98 M)** — DUT-side, committed `bcef83f`
   "Serialize fetch on a value-changing SATP write". A value-changing
   `csrw satp` now flushes the frontend buffer + redirect-fetches so the
   refetched PC re-translates (matches eager simmerv). riscv-tests stays
   240 (riscv-tests writes satp in M-mode where the flush is a no-op).

2. **Supervisor-timer interrupt (STIP) delivery timing (~27.8 M)** —
   cosim-glue side. Two mirror-image MRET→S-mode cases that NO fixed
   defer/eager rule satisfies: when STIP was pending long before the
   MRET the DUT takes it AT the xret target; when STIP crosses
   `stimecmp` right at the MRET the DUT's 1-cycle-stale registered
   `pre_intr_pending` misses it at the target and takes it one insn
   later. Both legal (imprecise interrupts). Fix = make simmerv FOLLOW
   the DUT on STIP *taking*, exactly like the pre-existing MTIP gate.
   New `simmerv_set_stip_armed(bool)` (simmerv `cosim/src/lib.rs` +
   `cosim/simmerv_cosim.h`) sets `cpu.cosim_stip_armed`;
   `handle_interrupt` masks `MIP_STIP` out of the take-set when false.
   `sim_main.cpp` sets it = (DUT trapped this retire with cause
   0x8000000000000005). Gate only the TAKING — `mip.STIP`/`sip.STIP`
   stay computed from `mtime>=stimecmp` so guest reads still match
   (can't clobber `stimecmp` like MTIP's `mtimecmp`: it's a guest CSR).
   Dead ends: a simmerv `defer_interrupt` one-retire xret suppress gets
   case 1 wrong; removing it gets case 2 wrong. Gate subsumes/replaces
   defer — defer fully removed.

3. **sstatus.UXL WARL (~29.34 M)** — simmerv fix. `csrrw sstatus`/`csrr`
   readback diverged: DUT `0x2_…` (UXL=2, RV64), REF `0x1_…`. `sstatus.
   UXL[33:32]` is read-only WARL = 2 on this RV64 hart. simmerv's
   `Csr::Sstatus` write mask `0x8000_0003_000d_e162` wrongly included the
   UXL bits. Fix: `mask & !MSTATUS_UXL_MASK` (mirrors the Mstatus write,
   which already excluded it).

4. **satp.ASID WARL width (~29.338 M)** — simmerv fix. Linux probes ASID
   width by writing all-1s and reading back. DUT returns ASID=`0x3ff`
   (10 bits — `smolrv64.v TLB_ASID_BITS=10`); simmerv stored all 16.
   Fix: mask written satp ASID to 10 bits in `Csr::Satp` write.

5. **Page-crossing data access trap (~32.77 M)** — simmerv fix, but see
   MISFEATURE note below. smolrv64 does ONE dTLB translation per data
   access, so a load/store whose byte range crosses a 4 KiB page boundary
   traps as Load/StoreAddressMisaligned (cause 4/6) and OpenSBI (M-mode,
   vector 0x80000520) emulates it. Exact DUT condition (smolrv64.v
   ~5989): `csr_satp[63:60]==8 (Sv39) && (mprv?mpp:prv)!=M &&
   (mem_addr[11:0]+bytes > 4096)`. (NOT natural alignment — within-page
   misaligned is handled; six prior misaligned loads in the same page did
   not trap.) Fix: new `Mmu::page_cross_access_traps()`; guarded the
   page-cross branches in `cpu.rs memop_read`/`memop_write` (hot LD/SD
   path) and `mmu.rs load_virt_bytes`/`store_virt_bytes` (AMO/other) to
   return the misaligned trap when it holds. M-mode/Bare still byte-split.

6. **Supervisor external interrupt (SEIP) delivery timing (~168.4 M)** —
   cosim-glue side, same family as #2. At retire 168,401,574 a `csrrw
   sstatus` that sets SIE was followed by REF taking SEIP (cause
   0x8000000000000009) immediately while the DUT deferred one instruction.
   The DUT suppresses interrupts for one instruction after a write to an
   interrupt-control CSR (sstatus/sie/mstatus/mie/mip/mideleg — its
   `just_xret` flag, smolrv64.v ~7936). Fix = same DUT-follow gate as
   STIP: new `simmerv_set_seip_armed(bool)` masks `MIP_SEIP` out of the
   take-set unless the DUT vectors SEIP this retire (`sim_main.cpp` sets
   it = DUT trapped with cause …9). `mip.SEIP` (mirrored from the DUT PLIC
   via `simmerv_set_seip`) stays set for sip reads. **VERIFIED**: cosim20
   cleared 168,401,574 and ran to 172 M+ — and reached **userspace**
   (saw a `pc=0x3f…` user address ~171 M), so the guest is now booting
   into userland. (If a supervisor *software* interrupt (SSIP, cause …1)
   ever shows the same pattern, add the identical gate for it.)

> **MISFEATURE (Tommy):** the DUT trapping page-crossing memops is itself
> a misfeature — ideally the hardware should handle the cross (two dTLB
> translations) rather than trap to firmware. Fix #5 is correct ONLY
> while the DUT keeps trapping. When the DUT is fixed to handle
> page-crossing in HW, **revert simmerv fix #5** (simmerv's native
> byte-split is already the right behavior). Revisit together; do not
> treat the DUT trap as ground truth.

## gb5 / OpenSBI-v1.7 CSR-probe cascade fixes (#7–#11, 2026-06-03)

Switching to the gb5 workload (OpenSBI v1.7) surfaced a boot-time
CSR-probe cascade: the newer firmware reads a batch of optional CSRs the
older tiny128 firmware did not. Each one the DUT implements (mostly
read-0) but simmerv lacked → `IllegalInstruction` divergence. Cleared in
order (DUT pc ~0x8001117x probe routine, retires ~685 k–5.37 M):

7. **Sstc / stimecmp (~685 k)** — DUT-side feature add (`smolrv64.v`).
   `csrr stimecmp` (0x14d) trapped illegal on the DUT (no Sstc). Added
   real Sstc: `csr_stimecmp` reg (reset all-1s, matches simmerv's
   `u64::MAX`); STIP split into `stip_sw` (software-written via MIP) and a
   registered comparator `stip_stc <= clint_mtime >= csr_stimecmp` (regd
   for the same timing reason as `mtip`); `wire stip = csr_menvcfg[63]
   (STCE) ? stip_stc : stip_sw` so STIP is HW-driven/read-only under Sstc,
   software-driven otherwise; CSR read/write of 0x14d; S-mode access traps
   illegal when `menvcfg.STCE=0` (M-mode always allowed) on both read+write
   privilege checks. `gb5.dts` advertises `"sstc"`. The existing
   `simmerv_set_stip_armed` STIP timing-gate (#2) still papers over the
   1-cycle registered-comparator delivery skew. riscv-tests stays 240.

8. **Smcntrpmf mcyclecfg/minstretcfg (~685.5 k)** — simmerv fix. DUT
   implements 0x321/0x322 as plain M-mode RW regs (reset 0, used for the
   UINH/SINH/MINH cycle/instret privilege-inhibit bits 60–62); simmerv
   trapped. Added `Csr::Mcyclecfg`/`Minstretcfg` (enum + `legal()` +
   `CsrFile` fields init 0 + read/write_csr_raw store-raw). Not wired into
   simmerv's counter filtering (only stored) — revisit if a cycle/instret
   value ever diverges.

9. **Debug-trigger CSRs tselect/tdata1-3/tinfo (~685.5 k)** — simmerv fix.
   DUT reads 0x7a0–0x7a4 as 0, writes to 0x7a0–0x7a3 no-op (no triggers).
   Added `Tdata1/2/3/Tinfo` to the enum (`Tselect` already present) and
   all five to `legal()`; reads fall through to `_ => 0`, writes to the
   no-op `_ =>` default. `Tcontrol` (0x7a5) deliberately NOT added — the
   DUT traps it too.

10. **mideleg/sie LCOFIP bit 13 (~5.37 M)** — simmerv fix. Sscofpmf setup
    writes `mideleg=0x2222`; DUT stores mideleg unmasked and reads back
    0x2222, simmerv masked to 0x222 (dropped bit 13 / LCOFIP). Widened
    simmerv's `Csr::Mideleg` write mask 0x222→0x2222 and the `Csr::Sie`
    mask 0x222→0x2222 (DUT's SIE read mask is `csr_mie & 0x2222`). Bit 13
    = supervisor local-counter-overflow interrupt.

(11. reserved — next divergence in kernel-space, TBD.)

After #7–#10 the cascade is clear and the guest jumps into the Linux
kernel (kernel-virtual PCs ~6 M). simmerv changes #8–#10 are uncommitted
+ unbuilt-for-commit (clippy/fmt hook not yet run); DUT #7 verified
riscv-tests 240 but uncommitted. **Commit pending** — see below.

## Current state / next step

Two tracks are verified:
- **tiny128** (fixes #1–#6): clean ~4 M → past **172 M**, reached
  **userspace**. This is the original oracle and remains valid.
- **gb5 / OpenSBI v1.7** (fixes #7–#11 on top): clears the CSR-probe
  cascade and runs into **Linux kernel-space** (kernel-virtual PCs from
  ~6 M; clean past ~12 M and counting as of this writing).

**First step on resume:** re-run the gb5 cosim (`cd ~/smolrv64/workloads/
gb5 && make cosim`), let it run, read the next MISMATCH block. The kernel
memory-init phase is a long loop (clearing 2 GiB) around pc
`0xffffffff801a6exx` — expect many millions of clean retires there before
new code regions. Each new divergence is either (a) a genuine simmerv
model bug → fix simmerv, or (b) an unspecified / HW-timing quantity →
make the cosim glue follow the DUT (see the STIP/SEIP/MTIP/HPM/mtime
precedents in `sim_main.cpp`).

**Build note:** after editing simmerv, `cd ~/simmerv && cargo build
--release`, then `cd ~/smolrv64/src && touch sim_main.cpp` BEFORE
`make cosim` (the stale-link guard). `cargo` cwd resets to the parent
repo after each shell call — always `cd` explicitly.

## gb5 (Geekbench5) workload — 2 GiB build DONE, now the active workload

`workloads/gb5` (`make cosim` there) is the active oracle workload — a
large run that needs **~2 GiB guest RAM**. The 2 GiB conversion is
complete (`MEM_SIZE_LG2=31`): `src/Makefile` `TINY128_SIM_MEM_SIZE_LG2=31`
(feeds `MEM_SIZE_LG2`/`COSIM_MEM_SIZE_LG2`) with `AXI_MEM_SIZE_LG2=29`
kept in `TINY128_COSIM_VDEFS`; `src/smolrv64.v` `MEM_SIZE`/`AXI_MEM_SIZE`
use `64'd1 <<` (the 32-bit-literal overflow fix, committed); `gb5.dts`
memory node `<0 0x80000000 0 0x7ff00000>`. `make cosim` from
`workloads/gb5` rebuilds the dtb + 188 MB `mem.even`/`mem.odd` (gitignored
— carry them or rebuild) and the cosim binary, then runs:

```sh
cd ~/smolrv64/workloads/gb5 && make cosim > /tmp/cosim_gb5.log 2>&1
```

`a1`/x11 (initrd FDT pointer) is set from `workloads/gb5/rf.hex` line 12
(= 0x9ff00000) — it is derived from the gb5 parameters, NOT firmware.

This run uses **OpenSBI v1.7** (`FW=fw_payload-7.1.0-rc6.bin`), newer than
tiny128's firmware, which probes a different/larger set of optional CSRs
at boot — fixes #7–#11 below clear that probe cascade. As of 2026-06-03
the gb5 cosim clears the cascade and reaches **Linux kernel-space
execution** (kernel-virtual PCs `0xffffffff8xxxxxxx` from ~6 M retires).

## How to read a MISMATCH

The block prints a DUT and REF line per recent retire; the diverging one
is repeated under "Diverging retire". Fields: `pc npc insn prv trap rd
cause tval mt mepc`. Decode `insn` to see what executed; compare `rd`
value / `trap`+`cause` / `npc`. `mt` is the (DUT-driven) mtime. A `trap=1
cause=…5` is supervisor-timer, `…7` machine-timer, `4/6` load/store
misaligned, `c/d/f` instruction/load/store page-fault.

## Cosim glue precedents (sim_main.cpp `cosim_retire`)

The DUT exports per-retire state via DPI; simmerv is driven to match
HW-dependent inputs before each `simmerv_step_retire`:
- `simmerv_set_mtime` — mtime synced from DUT every retire.
- `simmerv_set_mtimecmp(~0 unless DUT taking MTIP)` — gate machine-timer.
- `simmerv_set_stip_armed(DUT taking STIP)` — gate supervisor-timer (new).
- `simmerv_set_seip` / `simmerv_set_plic_ip(10,…)` — mirror DUT PLIC/UART.
- `simmerv_arm_csr_read` — for cycle/time/instret/mip-ish CSR reads, the
  next read returns the DUT's value (`csr_read_to_override`).
- mvendorid/marchid/mimpid (0xF11/12/13) also take the DUT value;
  `MIMPID` is stamped with the HEAD short commit id (`bcef83f` build).

## Process hygiene (Tommy's rules)

- One `make` at a time; trust the completion notification.
- Kill only the exact process: `pkill -9 -f
  /home/.../smolrv64-tiny128-cosim` (the 15-char name limit breaks plain
  `pkill smolrv64-tiny128-cosim`).
- Commit style: terse imperative subject, **no `Co-Authored-By` trailer**.
- Don't commit Vivado build artifacts under `platforms/.../*.runs/` or
  `*.gen/` (lots of churn shows in `git status`; only commit real source).
