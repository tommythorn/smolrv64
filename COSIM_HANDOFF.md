# Cosim oracle — cross-machine handoff (2026-06-02)

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
   via `simmerv_set_seip`) stays set for sip reads. **Verification of this
   one was still running (cosim20) at handoff — confirm it clears
   168,401,574; if a supervisor *software* interrupt (SSIP, cause …1) ever
   shows the same pattern, add the identical gate for it.**

> **MISFEATURE (Tommy):** the DUT trapping page-crossing memops is itself
> a misfeature — ideally the hardware should handle the cross (two dTLB
> translations) rather than trap to firmware. Fix #5 is correct ONLY
> while the DUT keeps trapping. When the DUT is fixed to handle
> page-crossing in HW, **revert simmerv fix #5** (simmerv's native
> byte-split is already the right behavior). Revisit together; do not
> treat the DUT trap as ground truth.

## Current state / next step

Fixes #1–#5 are verified: the run was clean from ~4 M to **168.4 M**.
Fix #6 (SEIP gate) is committed but its verification run (`cosim20`) had
not yet reached 168.4 M at handoff — **first step on resume: confirm
`cosim20` (or a fresh run) clears retire 168,401,574, then find the next
divergence.** Re-run the build+run commands above and read the MISMATCH
block. With #1–#6 the workload appears to reach kernel steady state
(repeating idle/scheduler PCs); divergences are now rare (~100 M apart).
Each is either (a) a genuine simmerv model bug → fix simmerv, or (b) an
unspecified / HW-timing quantity → make the cosim glue follow the DUT
(see the STIP/SEIP/MTIP/HPM/mtime precedents in `sim_main.cpp`).

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
