# RTL rules

Rules derived from the defect record, not from taste. 1177 commits, 218 with
defect-class subjects. Nearly all of them fall into six generators; each rule
below exists to make one of those generators structurally impossible rather
than to catch its next instance by review.

Every rule cites the commits that paid for it. If a rule looks like overhead,
read the commits.

**Enforced today:** `src/lint.sh` (load-bearing lint rules, file-scoped waivers
in `src/verilator.vlt`); always-on invariants in the RTL; `CHECKS` in
`run-vl-tests.sh` (the O(N) checkers, default on). **Not yet enforced:** the
config sweep (G3) is manual, and there is no formal flow (H3). Rules marked
against a defect with no assertion behind them are the backlog.

---

## The six generators

1. **A slot name is a raw pointer.** `{way, idx}`, a freelist index, a queue
   index — captured in one cycle, dereferenced several cycles later, after
   another agent can have reallocated what it names. Use-after-free.
   `9ce7d11`, `fc861f8`, `cap_ok` in `cache.v`, `f592200`.

2. **Address used as transaction identity.** Works until two transactions share
   an address, or an address is reused after a squash.
   `23ede45`, `2c5c639`, `d03aad5`, `7e64df8`.

3. **A cross-cutting precondition replicated by hand at N sites.** Every new
   functional unit is a new chance to forget it. `dest-valid` was fixed four
   times (`495ba74` AMO, `3d0bf73` mul/div, `71bd221` FP); the EX-squash gate
   three times (`49f723d`, `74eba24`, and the ALU/CSR originals).

4. **Stale context surviving a control event, solved once per engine.**
   `ctx_poison` (`57138a6`), fresh-checkpoint traps (`73dd13b`), sticky-inv
   (`804f3ea`), `pend_iflt` (`38c64dc`), stale-wake masking (`80dfe59`),
   `47e1b41`. Five mechanisms for one problem.

5. **Fail-silent by default.** An unmatched response is ignored, a dropped
   pulse is a wedge, a bad SD read is accepted, an ignored `-D` means you
   tested something else. `e73b176`, `08d3bfd`, `20acd8c`, `7cf760e`,
   `c0cb6ee`, `e826b6d`. The bug is cheap; the 90M-cycle detection latency is
   what costs days.

6. **The verified artifact is not the shipped artifact.** Behavioral `sdpram`
   vs XPM BRAM, `fp_unit_stub` vs CVFPU, CDC constraints living only in `.tcl`,
   width lints disabled. `3ef357e`, `e552aab`, `c629047`, `d3d09157`.

---

## A. Invariants

**A1. Checks are always on. Tracers are optional.**
A `$display` tracer or a stats counter belongs behind an `` `ifdef ``. An
invariant check does not. The 215-test gate defines no `VDEFS`, so every check
written behind `LSU_ASSERT`, `WTCHK`, `FL_ASSERT`, `FL_DBLALLOC`, `PRFVAL_CHK`,
`SEQROB`, `SERSQ`, `CSRDBG` used to be compiled out of every regression run.
`21e9fe5` is titled *"keep the two invariants that found the boot wedge"* — and
CI did not compile them.

Two tiers, because cost is real and a dogmatic rule gets quietly disabled:

- **O(1) per cycle → unconditional in the RTL.** No define, no opt-out. The
  freelist double-alloc detector, the LSU order-safety check, the two-L2-
  transactions tripwire, illegal-FSM-state defaults.
- **O(N) sweeps and shadow models → a define the GATE always builds in.**
  `FL_ASSERT` (arch map vs free set, `AREGS×SHARDS` per cycle) and `SEQROB`
  (shadow ROB) are in `run-vl-tests.sh`'s `CHECKS`, default on, measured under
  2% on the suite. `CHECKS=` turns them off for a multi-hour soak.

The model to copy is already in the tree: `cache.v:462` `$fatal`s if two L2
transactions are ever outstanding, unconditionally, because the ack is
untagged. That is how every assumed-impossible condition should be written.

**A1a. A debugging aid is not an invariant.**
Promoting one is how you get a false failure and lose trust in the whole layer.
A check earns always-on status only if it is true for *every* legal execution,
not just the scenario it was written to debug. `LSU_ASSERT`'s FWD-MISS check
looked like an invariant and is not: it assumes the older covering store is
also the youngest one covering the word, and it compares against `c_val`, which
is the post-transform result (sign-extend / zero-extend / NaN-box) rather than
the raw merged word. A legal `fld` over a word written by two different stores
trips it — `rv64ud-p-ldst` under `CACHE=1` does exactly that. It stays opt-in.

Same trap on the other side: the exclusivity invariant "no line is valid in both
the array and the prefetch buffer" reads like B4 but is false here — `PF_EN`
implies `WRITABLE==0`, both copies are clean and identical, and the prefetcher
arms `cur_line+1` without probing the tags, so duplicates are routine. The
property that *is* true, and worth asserting, is fate-sharing: no pre-flush line
may survive into the buffer.

**A2. Every FSM `case` has a `default` that asserts.**
`cache.v:775`, `mmu.v:220`, `lsu.v:903` have none. An FSM that lands on an
unlisted encoding holds state forever: a wedge indistinguishable from a
hundred other causes, hours of bisect. The `default` costs nothing and turns it
into one line of output.

**A3. "This cannot happen" is an assertion or it is deleted.**
A comment asserting an invariant, with the checker behind an off-by-default
`` `ifdef ``, is the exact shape that lets a bug survive. See `SERSQ` in
`exec_shard.v:572` — it documents that `csr_req_v` has no squash gate, that the
safety argument is an assumed scheduling property, and that hardware showed
behaviour consistent with the assumption breaking. That check must be on.

**A4. Anything the design currently drops must assert instead.**
Unmatched response, request accepted while busy, index out of range, an
operation whose completion has no live owner. `lsu.v:161` — *"An abandoned
(squashed) read's response matches no live entry and is ignored"* — is the
policy that turned `23ede45` into a boot wedge instead of a one-line abort.

**A5. A fix ships with the invariant it violated, always on.**
Not the debug tracer used to find it. The invariant.

---

## B. Identity and ownership

**B1. Responses are matched by a tag the requester allocated.**
Not by address, not by "there is only one in flight", not by FSM phase. The
`lsu.v` load path matches responses by PA (`lsu.v:383`); the device path had no
identity at all until `2c5c639`. Where a mutual-exclusion argument is used
instead of a tag — as on the shared L2 port — it must carry an always-on
assertion (`cache.v:462` does; copy it).

Note: this does not replace ordering requirements. MMIO must still be solo
because it has side effects and cannot be replayed or executed speculatively —
that is a semantic constraint, not a mechanism gap, and a tag does not remove
it.

**B2. One writer per shared buffer.**
If two FSM phases write the same register, they are two registers, or the
second phase re-derives its value. `fc861f8`: `S_WB` reused `wb_*` for the
victim capture while a span-NC push still needed it, and the push wrote the
wrong slot's line.

**B3. A slot name captured now and used later carries its tag, or its slot is
pinned.**
Either validate at every dereference (`cache.v:476` does this for one case —
generalise it into a function all dereferences go through), or make the victim
chooser unable to select a slot that an outstanding op has named. Never
dereference a bare `{way, idx}` across a state in which an install can run.

**B5. The response is selected by the responder that FIRED, not by re-decoding
the address.**
The corollary of B1 on the return path, and this design has now paid for it
three times in the same module. `ino_soc_top` routed the load return by
recomputing `is_dev_r`/`is_virtio_r` from the live `dmem_raddr` — a set of
masked 64-bit compares — at three sites in turn: `dmem_wready`, then
`raw_rvalid`, then `raw_rdata`. Each fix collapsed one site onto the valid
signals (`vio_rack` / `dev_rvalid_q` / `dc_rv_ok`, mutually exclusive and
asserted so) and left the others, so the address decode simply became the head
of the next critical path. Measured 2026-08-24 on the third one: 57 failing
endpoints at 166 MHz, `mem_raddr[13] -> CARRY8 x3 -> is_virtio_r -> is_dev_r`
costing **1.27 ns before the 64-bit mux began**, on a path that then ran through
load alignment and the X-stage ALU into `m_result`.

It is also the more robust structure, which is the real reason to prefer it:
the select no longer depends on `mem_raddr` still holding the address the
response belongs to. When a mutual-exclusion argument replaces a tag, assert it
once (this one does) and then let *every* consumer select on it. If one site
still decodes the address, the rule is not applied — it is postponed.

**B4. A second copy of a line must share the original's invalidation fate, and
that fate must be asserted.**
The prefetch buffer (`pf_val`/`pf_addr`/`pf_line`) holds a line the tag array
does not track, so the duplicate-line tripwire (`24f7dd3`) cannot see it.
Duplication itself is fine here — `PF_EN` implies `WRITABLE==0`, so both copies
are clean and identical — but the copy must die when the array does, or a
`fence.i` leaves a stale line reachable through `pf_hit` while the array reads
clean. `pf_drop` and the `S_IDLE` clear deliver that across three separate
sites and nothing checked the property they collectively provide; it is now
asserted at flush completion. Where a design keeps a shadow copy, name the
fate-sharing property and assert it — do not assert exclusivity you do not have.

---

## C. Cross-cutting predicates

**C1. Computed once, applied at one site.**
`exec_shard.v` writes `ex_v & ex_pdv & ~ex_squash` longhand at five sites
(401, 423, 427, 501, 534). Line 528 records the cost: *"ALU/CSR/in-core-FP have
it; the AMO drive was MISSED"*. Compute `wb_ok` once; every unit's enable is
`wb_ok & <unit-specific>`.

**C2. A new functional unit routes through the existing gate.**
Do not add a parallel path to the writeback port, the redirect port, or the
LSU. If the gate is in the way, move the gate — do not go around it.

**C3. Speculative state never becomes visible.**
Visible state (`mstatus.FS`, `fcsr`, BP tables, CSRs) updates at retire. See
`8fc57f1`; `fcsr` flags are the remaining latent case.

**C4. A derived read must never feed its own read-modify-write.**
Where a register's read value is an overlay of stored state and live hardware,
name the raw register as the RMW base explicitly. `mip` reads as
`(mip | hw_ip)` because the spec requires the device lines to OR into the read;
`8585044` computed `csrrs`/`csrrc`'s new value from that *read*, so any M-mode
RMW of `mip` while a device line happened to be high latched the transient into
the software register. A stale SEIP then storms spurious external interrupts
forever — trap, PLIC CLAIM returns 0, return, SEIP still reads 1, re-trap: a
~1300-cycle loop that starves commit and presents as a full-system hang. It
needed counters plus external-IRQ load to reproduce, which is why 15 minutes of
plain NFS traffic never found it. Audit every CSR whose read differs from its
stored value.

---

## D. Staleness

**D1. One epoch mechanism, not one per engine.**
A global epoch bumps on redirect / sfence / checkpoint close. Every
long-latency engine latches the epoch it started in. A result delivers only on
epoch match; a mismatch asserts and drops. `ctx_poison`, fresh-checkpoint
traps, sticky-inv, `pend_iflt` and stale-wake masking are five instances of
this one rule.

**D2. A one-cycle pulse to a receiver that can be busy is made sticky at the
receiver.**
`804f3ea` (`inv_req` during a refill), `4239971` (line-port request pulse).
The receiver latches; the sender does not retry.

**D3. An in-flight flag needs a completion guarantee, and an assertion on its
age.**
A flag that says "something is outstanding" and is cleared only by that
something completing will wedge the whole machine if the completion can be
lost. `4db98a0`: an injected `OP_IRQ` can commit without its trap firing, which
leaves `inject_inflight` latched; no rollback ever comes, `csr_irq_v` cannot
fall because the handler that would clear the level source never runs, every
future injection is vetoed, and once the kernel reaches `wfi` the machine
freezes solid — console, ping, everything. It took an ILA capture on hardware
and a cosim run to 84.17M retires to see it. The fix in place is a 64k-cycle
watchdog, which is a mitigation: it is stated in its own commit message as
something commit-side orphan detection should replace. Either way the age bound
is known, so assert it — a wedge that announces itself costs minutes.

**D4. An entry read at a GUESSED index is labelled with the index actually
used, never with the one you wish you had used.**
Speculatively indexing an array early is safe exactly when a later check can
tell that the guess was wrong — and that check can only work if the entry
carries the address it was really read at. On 2026-08-23 the predictor was
changed to index the BTB from `norm_npc` instead of `npc` while still recording
`btb_qpc <= npc`. The stated reasoning was that "a stale read is safe because
the hit test requires `btb_qpc == base_pc`", which is true of the test and
false of that code: the entry came from one address and was stamped with
another, so on a redirect cycle a stale entry could pass a hit test as the
entry for the redirect target. The mispredict it caused redirected to the same
PC, which re-read at the same wrong index — a mispredict loop that never
resynchronised. It hung `rv64mi-p-illegal` and was reverted with the mechanism
unexplained.

The same shortcut, with `btb_qpc <= apc` (the address actually indexed), is what
`fetch.v`'s ahead PC now does, and every wrong guess degrades to a LOST
prediction: 240/240 including `rv64mi-p-illegal`, and 0.13% of retires over 40M
cycles of Linux cosim. The guard was always there. It was pointed at the wrong
address.

**D5. A request that has been ACCEPTED is withdrawn, not re-presented.**
A unit's "go" signal built from `op is in M` plus `unit is free` will fire a
SECOND time if the stage is held after the hand-off — and the stage can be held,
because `m_done` is forced low whenever another writer takes the single PRF/ROB
port. The FPU re-issued an op whose ROB slot had already completed, committed
and been freed; the ROB's own "completion for a slot with no live entry"
assertion is what caught it. The guard is the already-completed latch
(`~m_unit_done_q`), the same one `ino_lsu`'s `req_valid` carries. If two units
need it, it is one predicate applied at both sites, not two spellings of it.

**D6. An op that writes LATE is excluded from the BYPASS, not merely from the
writeback.**
A non-blocking load and an FP op both leave M with no result in hand and write
the PRF later from their scoreboard slot. Excluding them from `m_wb` is the
obvious half; the half that bites is the M→X bypass, which compares
architectural `rd` and happily forwards `m_unit_res_q` — a register latched at
DISPATCH, i.e. the result bus as it stood before the unit had produced anything.
The window is real because `m_done` is held low on another writer's landing
cycle, so the op sits in M with `rd` asserted AFTER its interlock has cleared. A
correct `fmin` retired with the right value while the very next instruction read
zero. The interlock covers these consumers; the bypass must decline them.

**D7. A variable-latency unit can answer in the ISSUE cycle.**
"Latency >= 1" is an assumption, not a property. CVFPU's NONCOMP ops
(min/max, sign-inject, compare, classify) assert `out_valid` combinationally
with `in_valid`, `PipeRegs` notwithstanding, and parts of CONV do too. A
completion path that sets an in-flight flag on issue and clears it on result
must handle both arriving together — `else if` on the issue arm, not a separate
state. Dropping the branch the previous FSM had failed exactly the 16
fcvt/fmin/recoding tests and nothing else.

**D8. An output a caller feeds back into its ISSUE decision is a register.**
Exposing a downstream unit's combinational `ready` closes a loop through the
scheduler: `fpnew_top.in_ready_o` is combinational in `in_valid_i`, and
`exec_shard.v` computes `munit_busy = ... | ~fp_iss_ready`, which produces
`fp_start`, which is that valid. The reason this is a RULE and not a lint
finding is that `run-vl-tests.sh` builds with `-Wno-UNOPTFLAT`: the loop does
not error, it is settled wrong, and the damage lands on whatever else that shard
was issuing — 11 atomics tests and `rv64uc-v-rvc`, none of which touch FP. Hold
the request in a register and hand the caller a registered `ready`. A gate that
waives UNOPTFLAT cannot be the thing that finds this, so the interface rule has
to.

---

## E. Widths and lint

**E1. The lint gate is `-Werror` on the load-bearing rules.**
`src/lint.sh` is the gate: `WIDTHTRUNC`, `CASEINCOMPLETE`, `LATCH`,
`UNOPTFLAT`, `UNDRIVEN`, `MODDUP`, `IMPLICIT`, `PINNOTFOUND`, `BLKANDNBLK`,
`MULTIDRIVEN` are errors. Waivers are file-scoped in `src/verilator.vlt` and
must name a file; if a waiver would have to name one of our own modules, fix
the RTL instead. Never a global `-Wno-`: the build carried global suppressions
to quiet imported CVFPU, which switched the same rules off for our code, and
that is why `e552aab`, `c629047`, `4ac3d92`, `f157d1d` and `857dfc8` reached a
bitstream or a wrong measurement. `-Wno-fatal` meant none of it stopped a build
anyway.

Style rules (`TIMESCALEMOD`, `UNUSEDSIGNAL`, `UNUSEDPARAM`, `DECLFILENAME`,
`ASCRANGE`, `UNSIGNED`, `BLKSEQ`, `GENUNNAMED`, `PROCASSINIT`,
`PINCONNECTEMPTY`) stay off, by name, so the list stays auditable.
`WIDTHEXPAND` and `PINMISSING` are advisory until their pre-existing hits in
the experimental files are cleared — shrink that list, do not grow the error
list back down.

**E2. An array index is the array's index width.**
No wide expression truncated at the bracket. `soc_top.v:693` indexes a
4096-entry array with a 58-bit value; if the subtraction that produces it ever
underflows, the wrap aliases silently into valid boot SRAM instead of faulting.

**E3. A width mismatch across a hierarchy boundary is an error.**
`backend_top.v:830` widens `cc_commit_count` to 6 bits and passes it into
`exec_bundle`'s 3-bit `retire_cnt` port (`exec_bundle.v:113`), which
zero-extends it back to 6 for `csr_file` (`csr_file.v:64`). `CNTW =
$clog2(CKMAX+IW+1)` is 3 at the default `CKMAX=2, IW=2`, so nothing is lost
today — but the port is a hard 3 bits, and the moment `CKMAX+IW > 7` (e.g.
`CKMAX=6` at `IW=2`, or `CKMAX=4` at `IW=4`) a checkpoint retiring ≥ 8
instructions adds `count mod 8` to `minstret`: it drops 8, it does not
saturate. `csr_file.v:172` truncates the same value a second time into the
3-bit `hpm_inc`. Coarse-CPR window growth is exactly the work that raises
`CKMAX`, and cosim takes DUT values for HPM counters, so nothing in the current
gate can catch it.

**E4. An output must be driven in every configuration that instantiates it.**
`cache.v:1244` assigns `perf_access`/`perf_miss` inside `` `ifdef PERF_TRACE ``,
directly under a comment describing them as *"Zihpm hardware cache events
(always-on, unlike the PERF_TRACE DPI trace below)"*. Intent and code disagree:
in every build without `-DPERF_TRACE` — which is every regression run and every
bitstream — the two outputs are undriven, and the four Zihpm cache events
(`HPMEV_DCACC`/`DCMISS`/`ICACC`/`ICMISS`) read zero. `UNDRIVEN` catches this
class; it is currently suppressed.

---

## F. Sim/synth divergence

**F1. One source per memory.**
The file the simulator elaborates and the file synthesis elaborates are the
same file. `d3d09157` killed a bitstream that was clean across 238 tests and a
full boot cosim.

**F2. No module name defined twice in the source list.**
`rs232.v` defined `rs232rx`/`rs232tx` a second time, with a different `rs232rx`
port list and a different default baud. Verilator's glob took `rs232.v` first,
so every simulation build bound a *different UART than the board*:
`rk_xcku5p.v` instantiates the standalone pair with `.ready()`/`.overflow()`,
which `rs232.v`'s five-port `rs232rx` does not have. It was harmless only
because no sim TB instantiates the UART — the testbench drives
`uart_rx_we`/`uart_tx_ready` directly. `rs232.v` is deleted; `MODDUP` is now a
lint error.

**F3. A constraint that affects function has a sim-side equivalent or a
checked-in check.**
CDC false paths are function, not timing (`make cdc`; the CVFPU
`clock_groups` bug).

---

## G. Harness

**G1. Every runner echoes the config it actually consumed, and refuses unknown
arguments.**
Passed is not consumed. `20acd8c` (every run was silently `IW=2`), `7cf760e`,
`0c1a90b`.

**G2. A stale binary is a build error, never a silent run.** `c0cb6ee`.

**G3. The config space is swept, not just the default point.**
`src/sweep.sh` runs `IW ∈ {1,2,3,4}` × `CKMAX ∈ {1,2}` × `CACHE ∈ {0,1}` and
compares against `src/sweep-expected.txt`; a cell worse than recorded fails.
Known breakage is written down *with a reason* rather than silently tolerated —
an unexplained non-zero entry is a bug someone owes an explanation for, not a
passing test. `--record` carries the reasons across a re-record, because the
first re-record after a fix would otherwise erase why every other cell is
broken.

`023a5df`, `61f7d0a`, `41061b8`, `9444f81` all only existed off-default and all
were found by a person tripping over them. The first run of this sweep found a
fifth: `ARSH = AREGS/SHARDS` truncated instead of taking the ceiling, so at
`SHARDS=3` the freelist left arch r63's home physreg marked free and the first
allocation handed out a live architectural mapping — every `IW=3` test dead at
25ns. Two more defects are recorded and unfixed (`IW=1 × CKMAX=2`, and `IW=3` +
cache starving the frontend), the second of which was unreachable while `IW=3`
failed outright.

Sweeping only power-of-2 points is not sweeping. The bugs live where the
arithmetic stops dividing evenly.

---

## H. Process

**H1. Cheapest confirmation first.**
lint → unit TB → formal → 215-test → boot. A defect found by a Linux boot cost
a hundred times what the same defect costs at lint.

**H2. Fix the generator, not the instance.**
If a bug is the Nth of a class, the deliverable is the rule that makes N+1
impossible — not the Nth patch. Four `dest-valid` commits is three too many.

**H3. Unit-level formal on the memory system.**
The cache slot allocator, the MSHR, the write-back buffer and the LSU response
matcher are small, control-dominated, and hold most of the worst bugs. Every
one of those bugs is a one-line invariant: *a line is in at most one of {array,
MSHR, write-back buffer, prefetch buffer}*; *no two MSHRs cover the same line*;
*a slot named by an outstanding op is not reallocated*; *every response matches
exactly one live entry*; *a dirty line is never overwritten before capture*.
Bounded model checking finds these in seconds.

## I. Area and timing

**I1. Unreachable memory costs slack somewhere else.**
This design fails timing on *routing*, not logic: the worst paths at 166 MHz
are `u_dcache/vw0_rep -> bank WEA` and `u_icache/cur_line -> linebuf CE`, both
under 1 ns of logic and 83–85% route on high-fanout nets. Area anywhere
therefore buys congestion everywhere, and congestion is paid in slack by
whatever is already marginal — not by the block that grew.

Measured 2026-08-23: `ino_prf` declared all three shard arrays `[0:NMAX-1]`
where `NMAX` was the *largest* shard, so `mem_ie` was 128 deep with `N_IE=64`.
Half of it was unreachable and synthesis built it anyway — the Distributed RAM
report showed all three as an identical `128 x 64, RAM64M8 x 60`. Sizing each
array to its own shard, with no functional change at all, moved WNS from
**−0.524 ns to −0.059 ns: 465 ps for deleting memory nothing could address**
(`3c5936a3`).

So: size every array to what it can actually hold, and read the Distributed RAM
and BRAM mapping reports after adding a structure. A parameter that is
"obviously big enough" is a timing bug on a congested die.

**I2. A single build's WNS cannot judge a change smaller than the placement
spread.** Four placer directives over identical RTL at DIV8=48 gave `Explore`
+0.054, `AltSpreadLogic_medium` +0.047, `ExtraTimingOpt` +0.025,
`ExtraPostPlacementOpt` −0.027 — an **81 ps spread that straddles zero**.
Every "166 MHz closed / did not close" judgement made before that measurement
sits inside the envelope. Compare against two directives; treat one number as a
sample, not a result. `Explore` is the default because it measured best
(`dc3fb0ad`).

**I3. Bring a replacement up as a shadow, checked every cycle.**
`ino_rename` + `ino_prf` ran against the real instruction stream with
`ino_regfile` still the operand source and an always-on comparison between them
(`7a2605f6`). That caught five defects at the mistake rather than downstream:
non-power-of-two free lists handing out physical register 0; a capacity
assertion whose bound truncated to 0 and fired on every write; a commit+flush
race restoring a stale head pointer; frees routed to the destination's shard
instead of the register's own shard; and a boot seed (`+a1=`) the new structure
never received. Only the last was reachable by the 240-test suite, and none by
inspection. The switch-over then moved one variable
(`914292fe`), and the retire stream stayed **bit-identical over 3e9 cycles**.

**I4. A valid bit kept outside its array is a mux the size of the array,
bolted to the read address.**
`ino_predictor` held the BTB and YAGS valid bits in flop vectors "so the data
array stays a clean BRAM/LUTRAM inference" — and thereby produced the opposite
of that. `ycorr_v[yidx(npc,ghr)]` is a 1024:1 LUT/MUXF tree, and it sat at the
END of the fetch loop (iMMU -> I$ -> aligner -> npc -> yidx). Measured
2026-08-24 on the design's worst path, 6.462 ns of a 6.245 ns budget:

* the valid mux alone, `yidx` to the capture flop: **1.051 ns**
* `yidx` route: **0.546 ns**, fanout 161 — a 1024-deep distributed RAM is 16
  primitives deep *per bit*, so the index drives 161 address pins

1.6 ns of a 6.0 ns period to read an 11-bit entry, and none of it logic.
Folding valid into the entry and forcing `ram_style = "block"` gave one RAMB18
(1 K x 11) and one RAMB36 (256 x 54), ended the path at a BRAM address pin, and
took the family from 33 failing endpoints to none.

The second half of the rule is what had blocked BRAM inference in the first
place: the write-forward mux sat between the array and its capture flop, and a
BRAM's read register is *inside* the primitive, so any output-side mux forces
the array back into LUTs. Register the forward DECISION and apply it in the
consumer's cycle instead. That also defines the read-during-write result, which
a simple dual-port BRAM leaves indeterminate in hardware — the forward is not
merely preserved by the move, it is what makes the BRAM legal.

**I5. A CDC wrapper instantiated on a SINGLE clock still costs its full
synchroniser latency.**
`smolrv64_cvfpu` crosses two clock domains with a toggle request/response and an
`ASYNC_REG` two-flop synchroniser each way. `fp_unit.sv` had tied `fpu_clock` to
`clk` since it was written — with a comment saying so, treating the crossing as
absent — but the toggle protocol and both synchronisers are structural and run
regardless:

    1 req toggle + 2 sync + 1 IDLE->ISSUE + 5 accept/PipeRegs
                 + 1 resp toggle + 2 sync + 1 latch   ~= 13

against 4 cycles of actual arithmetic. Hardware measured **14.02 stall cycles
per FP op**, dead on, and the FPU was 45% of all GB5 cycles — about two thirds
of it this handshake. Nothing in simulation or timing flags it: the design is
correct, closes timing, and is simply ~3x slower than the arithmetic it wraps.

Two things follow. A wrapper whose cost depends on a clocking choice states that
cost where the choice is made, so a same-clock instantiation is either
parameterised or a direct connection. And the "how many cycles does this
actually take" question belongs in the CPI stack before any RTL is written for
the unit — one hardware counter run sized this correctly and would have
redirected a day of work had it been run first.
