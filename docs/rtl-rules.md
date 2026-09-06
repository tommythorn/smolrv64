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

**A6. An invariant states the HAZARD, not the route to it.**
A check phrased in the same terms as the gate it is checking has the gate's
blind spot, and reports success from inside it. A1 above holds up `cache.v`'s
two-L2-transactions tripwire as the model to copy; in the `rv_cache.v` fork that
model is exactly what a real bug walked past. The tripwire (`5ce1666a`) is
`l2_req && l2_out && !l2_ack` -- *is a new request being raised while one is
outstanding*. The prefetch engine's own guard was `!l2_req`, so the prefetcher
can never raise `l2_req` in a cycle where the tripwire could observe it. Check
and bug were both written in terms of the request pulse. 300 M cosim cycles ran
green with two transactions genuinely outstanding, and `5ce1666a` concluded in
writing that the race does not occur.

The hazard is not "two requests raised in one cycle". It is **two consumers
waiting on one untagged ack** -- state, not an event. Written that way,
`pf_infl && fst inside {F_FILLW, F_WBA, F_FLUSHA}`, it holds no matter which
gate the issue path uses, and it fired on the first run at realistic memory
latency after being silent for the life of the bug.

The test to apply before believing a check: **if someone changed the gate, would
this still catch the bug?** If the check restates the gate's own condition, the
answer is no, and it is decoration that costs trust. Assert the state that must
never hold, never the transition you believe leads there.

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
three times in the same module. `rv_soc_top` routed the load return by
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

**B6. A shared port's BUSY covers the whole round trip, not the request pulse.**
`l2_req` is a one-cycle pulse, so a second user gated on `!l2_req` is gated on
nothing: the cycle after the first request the pulse is already low and the
response is still in flight. The port *looks* free for the entire memory
latency. That is how `rv_cache.v` came to have two L2 requests outstanding
against one untagged ack -- both consumers latched it, and the stream buffer
filed the demand line's bytes under the address it had asked for. Right tag,
wrong data, which no parity or provenance check can see (parity is computed on
the write, so wrong data gets consistent parity, and the row read is the row
asked for). `PF_EN` is I$-only, so it corrupted INSTRUCTIONS: the board died in
arbitrary places with both integrity checks silent.

The gate is *issued OR outstanding* -- `fill_l2_busy`, naming every fill,
writeback and flush state that awaits an ack. This is B1 seen from the requester
side: when an ack carries no tag, the only thing keeping it matched to its
requester is that there is exactly one, and that is a property of the ISSUE
GATE, not of the pulse. Either tag the transaction or make the gate cover the
latency; `!<req_pulse>` is neither.

PROVEN ON HARDWARE, not inferred, because a clean boot after a fix is not evidence that the
fix is why it booted -- the build also placed differently. The control was `main` with
exactly ONE TOKEN removed, `&& !fill_l2_busy`, and nothing else (the `.xpr` was restored so
Vivado saw a single-variable tree). It was validated in simulation FIRST, so a board run was
not spent on a control that might not be broken: at `DDR_LAT=40 DDR_JIT=63` the invariant
fires with `fst=7` = F_FILLW, the OUTSTANDING state, which is exactly what `!l2_req` cannot
see. Then both bitstreams, both MEETING timing:

| | WNS | result |
|---|---|---|
| guard in | +0.019106 ns | `ubuntu login:`, 0 faults, 801 KB of clean boot |
| guard out (1 token) | +0.010035 ns | Kernel panic at 1.22 s |

    epc: rtnl_fill_ifinfo.isra.0+0x23a/0x1070
    cause: 2 = ILLEGAL INSTRUCTION       badaddr: 00000000f6e40423

`cause=2` is the signature this mechanism predicts and nothing else does: `PF_EN` is I$-only,
so the corruption lands in INSTRUCTIONS and the core died executing a word that is not code.
Timing is excluded -- both met, and the one that failed had the LARGER logic removed. When a
fix and a rebuild land together, the negative control is what separates them, and it costs
one build.

---

**B7. Every attribute an access needs travels with its queue entry; none is re-derived
or hardwired downstream.** `ooo2_lq` carried a load's PA, size, sign and fp-ness but not
its Svpbmt uncached bit, and `ooo2_core` fed the LSU's pre-translated port
`pt_unc(pt_store ? sq_c_unc : 1'b0)`: a queued load was cacheable by construction. A load
that took the early start read the MMU's bit directly and was right; the same load a few
cycles later, queued behind a live store, was cached. Linux's virtio rings are NC memory
that a device writes behind the cache, so the guest read a stale used ring -- `virtio_net:
id 0 is not a head!` -- on the board and nowhere else: the cosim's guest maps nothing NC, the
riscv-tests never do, and the queue bench cannot know what the bit should have been. The
defect was six days old (`cb028682`) and surfaced when the day's LSU changes shifted which
loads got queued. The store queue had carried its own bit all along; the two entry formats
should have been one list. When a request passes through a queue, diff the queue's entry
against the request's port: every field on the port that the access consumes is on the
entry, or the queue is a defect. There is no assertion for a value the design never had;
the check is the diff, at review time.

**B8. A sequence number that orders entries of a ring is one bit wider than the ring's
index.** `ooo2_sq` handed a dispatching load its tail INDEX as the store-seqno, and "older"
was `(slot - head) < (tag - head)` in index width. With the queue full, tail == head, the
distance read zero, and a load with eight older stores live saw none of them: it took the
early start and read memory ahead of the store to its own address. Eight back-to-back
stores followed by a load of the second one's target is ordinary compiled code -- Linux
itself, at retire 123,081,278 of the Geekbench boot under the ooo2 cosim, and Ubuntu's
shell on the board, which segfaulted on the corrupted pointers it read back. Six days old
(`cb028682`); invisible to the 60 M-cycle tiny128 cosim, visible in the first long run of
a bigger guest. The counters that produce the seqno carry a wrap bit (`headc`/`tailc`),
the distance is exact up to NENT, and the queue asserts that no load claims more older
stores than are live. The same arithmetic error is possible in any ring whose occupancy
can reach its size; when a pointer is captured as an age, check what a FULL ring hands
out.

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

**C5. "Everything older is visible" is the ROB empty AND the store queue empty.**
Since the senior store queue (2026-09-04) a store RETIRES when the ROB's
irrevocable pointer releases it and the LSU drains it to the cache later, so an
empty ROB no longer means its stores have reached memory, and "at the ROB head"
no longer means "every older store has drained". Every precondition that used
either as a proxy for visibility names the queue explicitly, at one site each:
the serialization drain (`drained = rob_empty & sq_occ == 0`, which every fence,
fence.i, sfence.vma, AMO/LR/SC, CSR and trap op waits on -- sound there because a
serializing op dispatches only once nothing older is in flight, so the queue holds
nothing younger either), the CBO start in M (`m_cbo_wait = sq_av_any`: an entry
WITH AN ADDRESS, because M translates in program order so every older store has
one and no younger store can get one while M is held), and a load's early start
(`ld_older`, by store-seqno). The occupancy is NOT "older stores": entries are
allocated at dispatch, so the queue holds stores younger than M's op, which cannot
translate until M frees -- a CBO waiting on `sq_occ != 0` deadlocked build L at
SLUB init on 2026-09-04 (Ubuntu's clear_page is cbo.zero; the tiny128 kernel never
issues one, so the tiny128 cosim was blind to it -- the Geekbench image is the gate
for CBOs). Before the change a CBO could already pass an older store that was not
yet at the head; the senior queue widened that to retired stores, which is when it
was found. A new M-executed access that reads or maintains memory routes through
one of these three gates or adds its own by name; it never assumes the head or
`rob_empty` ordered it, and it never reads the occupancy as "older".

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
(`~m_unit_done_q`), the same one `ooo2_lsu`'s `req_valid` carries. If two units
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

**D9. Splitting a machine invalidates every predicate that names its states.**
A guard reading "X is only ever raised in states outside this set" is a theorem
about one state encoding, not a fact about the design, and it does not survive
the encoding changing under it. `rv_cache.v`'s prefetch guard named `st` states
and was true when a single FSM owned both the hit path and the fills. `2e3791c`
moved L2 requests to the new `fst` machine and left the guard on `st` -- and
since the split `st` sits in `S_IDLE` for the WHOLE fill, the window the guard
permits grew to cover precisely the cycles the port is busy. The guard did not
merely stop working; it inverted. The comment above it still asserted the old
invariant in the old machine's vocabulary, which is why it read as reviewed and
correct for four commits.

When a machine is split, `grep` every reference to the states of the machine
that changed and re-derive each predicate against the new one, comment included.
A stale comment stating a no-longer-true theorem is worse than no comment: it is
the thing that stops the next reader from checking.

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

**G4. A port-list change is a WHOLE-DESIGN change, so the gate is the full lint.**
Adding a port to a module is not verified by that module's own unit TB — the
instantiation is half the change and lives in another file. `ooo2_iq` grew
`in_order` and `iss_ps1/2/3`, its 16-check TB passed, and the commit went in
with `ooo2_core`'s instantiation still missing all four pins: the design did
not elaborate at all. `src/lint.sh` is `-Werror` on PINMISSING and catches it
in seconds, so the rule is simply that a changed port list means the full lint
before the commit, never the module TB alone. The commit had to be rewritten
out of history, which is the cheap version of this mistake — the expensive
version is a bisect landing on a revision that does not build.

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

**G5. A verdict comes from a model built from the sources under test, and the runner
proves it.** `run-ooo2-cosim-linux.sh` reused the built model whenever `BUILD` was unset
and its MEM_LG2/VDEFS stamp matched. On 2026-09-04 a day of core commits was declared
"bit-identical, 14,657,366 retires" one after another -- the lockstep that passed was the
previous week's core, and the true number at the end of the day was 16,025,548. Every one
of those logs lacked the line `building obj_dir_ooo2_clinux/tb_ooo2_clinux`, and nobody
looked. The stamp now hashes every RTL source, so an edit forces the rebuild; the rule
is the general one: a runner that can skip a build must say in its output whether it
did, and a verdict without that line is not read. The same hole exists in any harness
with a cached binary; check the run log before the number.

**G6. A trace is aimed by plusarg; a wait is keyed on the process.** Three 17 M-cycle
rebuilds on 2026-09-04 went to a `$time` literal in a debug print, in the wrong unit. The
testbench reads `+trace_from=<cycle> +trace_to=<cycle>` and exposes `trace_on`; an
`` `ifdef `` trace in RTL reads the same plusargs itself (`$value$plusargs`), never a
literal. And a waiter for Vivado keys on `pgrep -x vivado` (`tools/wait-vivado.sh`): the
flow runs synthesis and implementation as separate processes, so a log line such as
"Exiting Vivado" fires mid-build, and `pgrep -f` matches the shell running the wait and
hangs it -- both happened the same day, the second for the third time in this project.

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

**H4. `docs/OOO2-Spec.md` is updated in the SAME commit as the change it
describes.**
The spec is normative, not a summary written afterwards: geometry, sizes,
associativity, indexing, storage primitive, latencies, the pipeline stages, and
crucially WHAT STALLS AND WHAT RESTARTS. A spec that lags the RTL is worse than
no spec, because the next person budgets against it — this project has already
paid for that twice. The FPU handshake cost 14.02 cycles per op against 4 cycles
of arithmetic and nobody knew, because "CVFPU, PIPE_REGS=4" was the only thing
written down; and a whole day went into a CPI bucket worth 0.243 while a 45%
bucket sat unexamined. If a change moves a number in that file, moving the
number is part of the change.

Two corollaries. A figure that is MEASURED says so, with the workload, because
a measured number on one workload is not a property of the core — GB5 is
FPU-bound and `sha256sum` is frontend-bound on the same silicon. And a
"known limits" section is mandatory: what is single-outstanding, what still
blocks, what is not implemented. Those are the questions people actually ask,
and leaving them out is how an intermediate step gets mistaken for the
destination.

## I. Area and timing

**I1. Unreachable memory costs slack somewhere else.**
This design fails timing on *routing*, not logic: the worst paths at 166 MHz
are `u_dcache/vw0_rep -> bank WEA` and `u_icache/cur_line -> linebuf CE`, both
under 1 ns of logic and 83–85% route on high-fanout nets. Area anywhere
therefore buys congestion everywhere, and congestion is paid in slack by
whatever is already marginal — not by the block that grew.

Measured 2026-08-23: `ooo2_prf` declared all three shard arrays `[0:NMAX-1]`
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
sample, not a result.

**AND THE RANKING DOES NOT CARRY FORWARD.** Re-measured 2026-09-01 on a later
tree, the order INVERTED: `AltSpreadLogic_medium` +0.024 MET against `Explore`
-0.180 FAIL (-0.113 after `make physopt`). 204 ps apart on identical source --
inside the spread this rule warns about, and larger than the entire margin. So
the directive is part of the SHIPPING CONFIGURATION, not a sweep knob: a build
that meets timing only when somebody remembers to pass `PLACE_DIRECTIVE` is the
same trap `OOO2_HW` and `PROBE_CLK_DIV8` were. `AltSpreadLogic_medium` is the
default in `build.tcl` (`c4134edd`); this rule said `Explore` for a day after
that stopped being true, which is D9 applied to a document.

**MEASURE A STRUCTURE OUT OF CONTEXT BEFORE REDESIGNING IT -- AT THE INSTANCE'S
PARAMETERS.** `make ooc MODULE=<m> GENERICS="P=V ..."` synthesises one module alone on the
part and reports its intrinsic Fmax. In the flat design every path is 65-83% route and
placement swamps anything under ~200 ps (I2), so the full build tells you whether today's
placement was lucky, never whether a STRUCTURE is good. Measured 2026-09-03 at 6.000 ns:

    ooo2_pending  1144 MHz      ooo2_rename   403 MHz
    ooo2_lq        908 MHz      ooo2_iq       373 MHz
                                ooo2_sq       330 MHz
    rv_cache D$ as shipped (PAW=64 SIZE_KB=64)   181 MHz   +0.478 ns
    rv_cache D$ after 1271c96d                   206 MHz   +1.134 ns

The first cache figure recorded here (178 MHz, `cur_line_reg[18] -> valm/WE`) was taken
WITHOUT generics, i.e. on the module DEFAULTS -- PAW=34, 128 KB, no prefetch, a 2048-set
array with 18-bit tags that nothing instantiates. ooc.tcl now refuses to be silent about
that. Measure BOTH roles of a two-role module; the I$ shape adds `WRITABLE=0 PREFETCH=1`.

AND AN OOC NUMBER CANNOT SEE A PATH THAT CROSSES MODULES. The conclusion this paragraph
used to draw -- "every core module clears 166.67 MHz by 2x, the cache does not, so the
failing core paths merely END in logic the cache has starved" -- was wrong, and the data
that refuted it was already on disk: the routed checkpoint BEFORE post-route phys_opt was
at -0.169 with 709 failing endpoints, all of them core; phys_opt then equalised the worst
paths at -0.03 and only THEN did a cache family sit level with them. A census of that
checkpoint (`make census`: every endpoint under +0.35 ns, keyed by startpoint):

    u_sq/v_reg           1708 of 3401   one chain: pt port grant -> M's done -> wakeup, redirect
    u_dcache/cur_line     815           `hit` on enables and a dependent array read (I8)
    ps_out_reg            463           issued tags -> PRF -> ALU -> store-queue snoop
    m_addr_reg            145           dTLB compare -> M's done -> issue select
    fe/u_fetch            129           the fetch loop's fall-through adder into the F/X queue

The 22-25-level core chains span five modules (ooo2_sq -> ooo2_lq -> ooo2_lsu/mmu ->
ooo2_core -> ooo2_iq -> psmem); no per-module OOC can measure them, and `make ooc` cannot
build ooo2_core at all. So: OOC to compare a replacement against the incumbent AT THE SAME
INTERFACE, a census of the routed checkpoint to know WHICH families exist, and the full
build on two directives to confirm -- never one of the three alone.

**FLOORPLANNING IS NOT THE LEVER HERE -- TESTED AND REFUTED, 2026-09-03.** The obvious
reading of "65-83% route on a die that is only a third full" is that the design is
spread and wants compacting. `make place-report` showed `probe_core/core`'s 53,634
cells over NINE clock regions with `u_prf` mostly in X2Y3 and `u_sq` mostly in X3Y2,
diagonally apart. A pblock confining the core to six contiguous regions
(X1Y2:X3Y3, ~8,900 cells per region, no exclusion so the caches could still share
X1Y2) made it WORSE: WNS -0.206 / TNS -117 against 0.000 unconstrained.

The failing paths say why, and they did not move outside the core:

    u_sq/v_reg[2]        -> u_iq_i/e_r_reg[8][1]     70% route
    u_rename/lv_reg[15]  -> u_iq_l/e_r_reg[9][2]     82% route
    u_sq/v_reg[2]        -> ps_out_reg[10]/CE        72% route

These are SINGLE CONTROL REGISTERS fanning out to every issue-queue entry's ready
bits. `e_r` is per-entry wakeup state, so its destinations are spread BY CONSTRUCTION;
the route being timed is the fanout tree, not a journey between two blocks. Confining
the core gave the placer less room to spread the replicas it needs, which is exactly
what `AltSpreadLogic_medium` was chosen to do. Distance was never the mechanism.

So the lever is FANOUT, not placement: either fewer broadcast consumers, or readiness
held as state per physical register instead of recomputed from control that must reach
every entry. `ooo2_pending` (ooo2_core.v, brought up as a shadow under I3, nothing
consuming it yet) is exactly that structure and is the thing to finish.

**THE DESIGN HAS NO MARGIN ANYWHERE, and that is the real finding.** Across the
D$ work, six builds failed on SIX UNRELATED PATHS -- the D$ accept cone,
`u_lq/acc -> u_iq_i/e_r`, `u_sq/head -> u_iq_i/e_r`, virtio DMA ->
`probe_bridge`, and two more -- and the tree that ships closes at +0.019 to
+0.024. Whichever near-critical path placement treats worst that day is the one
that fails. The consequence for method: landing IPC one commit at a time against
a hard floor is right, but each candidate is currently decided by placement luck
rather than by its merit. Structural headroom has to come before the next IPC
change, and it must be aimed at whatever the tool reports as worst on TODAY's
main -- not at a cone suspected in advance.

The corollaries, priced on 2026-09-04: (a) the second-directive build is worth its hour
only when the shipping directive FAILS by less than the spread -- F closed at +0.084 on
AltSpread and the Explore build before it told nothing F did not; (b) never bisect on the
board with 166.67 MHz builds, because the lottery makes a bisect point unusable (B+unc: B's
own RTL plus one flop failed by 0.5 ns) -- bisect at `PROBE_CLK_DIV8=72`, where placement
cannot fail, and let the device tree's timebase be wrong; (c) when a cosim repro exists,
do not bisect on the board at all.

**I3. Bring a replacement up as a shadow, checked every cycle.**
`ooo2_rename` + `ooo2_prf` ran against the real instruction stream with
`rv_regfile` still the operand source and an always-on comparison between them
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
`ooo2_predictor` held the BTB and YAGS valid bits in flop vectors "so the data
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

**I7. On this FPGA, an indexed RAM read beats a mux across N scattered flops --
and it is the option that SCALES.**
Three separate reasons, and the third is the one that matters over time.

*Routing, not levels.* Measured 2026-09-03 at DIV8=48, every critical path in
this design is 65-83% ROUTE and only 17-35% logic. A mux over N entries is
therefore paying mostly to GATHER N physically scattered flop groups into a mux
tree; a LUTRAM is one compact primitive with local routing. The ASIC intuition
that flop-to-flop through random logic beats RAM-to-RAM is inverted here, because
in an ASIC the logic cloud is rarely made that big and route is not the term that
dominates.

*It SCALES.* Flops grow the gather with N -- more entries, more sources to route
from, worse placement pressure, and the cost lands on whatever else wanted those
sites. A RAM grows by adding depth to a primitive that is already local. Making
the queue or the ROB bigger later is an ordinary change if the payload is in RAM
and a re-floorplan if it is in flops. This is the reason to prefer RAM even where
today's timing does not demand it.

*Synthesis will tell you which you have, for free.* Any array declared
`reg [W-1:0] a [0:N-1]` that does NOT appear in the synth log's `The RAM "..."`
list is a mux of flops. The split showed up INSIDE `ooo2_iq.v`: `e_prd` and
`e_rob`, read only at `[sel]`, became `RAM32M`; `e_ps` and `e_r`, read by every
wakeup comparator, stayed flops -- distributed RAM has one read port per instance,
so a broadcast read forces a CAM. `e_r` must be a CAM; it IS the wakeup state.
`e_ps` need not be, and it was the tail of the critical path until its tags were
kept REDUNDANTLY in a `psmem` LUTRAM read at the selected index (`3b518832`).

TWO CONSTRAINTS, both load-bearing:

- **Do not fix these by registering RAM outputs.** That buys timing by adding a
  pipeline stage and costs IPC, which is the wrong trade in a design whose whole
  problem is memory-level parallelism. The valid move keeps depth IDENTICAL:
  replace the logic FEEDING an existing flop with a RAM read feeding that same
  flop. Verify it: a correct change is BIT-IDENTICAL in cosim retires
  (`psmem` measured 14,657,366 against 14,657,366), not merely close.
- **I6 still applies.** The flop, not the RAM output, drives the next RAM's
  address pin. Reading the payload RAM's registered port would have put a LUTRAM
  output on the PRF address pins and traded one path for a worse one.

**A PARALLEL RESET LOOP IS THE OTHER THING THAT FORCES FLOPS**, and it is easy to
miss because the steady-state access pattern looks perfect. `e_ps` was forced by a
broadcast READ; `ooo2_rename`'s free lists and rename maps are forced by a broadcast
WRITE -- `for (i=0;i<N;i=i+1) fl_ie[i] <= OFF32 + i;` in the reset branch. No RAM can
be written at every address in one cycle, so the whole array becomes flops however
clean the running behaviour is. Audited 2026-09-03: `fl_ie`/`fl_ld`/`fl_fe` (~2,240
flops) and `rmap`/`smap` (~1,152) are strictly one-write/few-read once out of reset --
the free lists are circular FIFOs read at the head and written at the tail, and the
rename maps need no wholesale copy on a flush because `lv <= 0` clears the live vector
instead. All ~3,400 flops are held there by the init loop alone. The fix is a
SEQUENTIAL init: walk a counter across the array while reset is asserted. Read ports
beyond three come from replicating the LUTRAM -- duplication buys READ ports, never
write ports.

AND KNOW WHAT IT DOES NOT BUY. `3b518832` removed its target completely --
`i_ps1_reg` appears ZERO times in the postroute report afterwards -- and whole-design
WNS still landed at +0.000902. The next path took its place at 74.8% route. Seven
distinct paths have now been worst across this work. One path fix does not make
margin here; see I2.

**I6. A late signal may reach a RAM's ENABLE. It may never reach its ADDRESS.**
An array's address pin has to be stable early: it fans out to every primitive in
the depth, it usually cannot be placed near the logic that computes it, and on a
BRAM it carries a real setup requirement. A one-bit enable does none of that. So
when the fetch cloud has to touch a predictor read, it touches the enable.

`ooo2_predictor` already had that split — `apc_en = fire | rollback | reset`
deliberately spends the aligner's `fire` on a RAM enable — while its own header
claimed "nothing from the I$-data -> aligner cloud feeds the PC mux". The claim
was false. `cti_ok`, the aligner's "this bundle ends on a CTI", was ANDed in at
the TOP of the predict cone, so `hit` carried it, so `p_ret` — the RAS-vs-BTB
target select — carried it, and `fetch.v`'s `apc` selected on *both* `pred_v`
and `pred_tgt`. The register-only ahead PC was register-only in intent and
aligner-dependent in fact. Measured on the routed NF=5 checkpoint:

    strad -> imem_addr -> u_immu/req_match -> u_icache/fb_w0 -> I$ data
          -> p_ret -> bp_tgt -> u_bp/btb_reg/ADDRARDADDR[12]

22 levels, 5 CARRY8, **5.521 ns of a 6.000 ns budget, 69.7% of it route**, plus
a twin ending at the corrector's address pin. Two of the three worst
non-FPU families in the design, from one term.

The fix is to split the cone, not to shorten it: a `tag_hit` half computed from
registers only (the arrays' read registers, `btb_qpc`, `base_pc`, the RAS) and a
`cti_ok`-qualified half. The *address* consumes the register-only half; the real
PC and every architectural update consume the qualified one. `pred_tgt` needed
`t_ret` rather than `p_ret` for the same reason, and is bit-identical where it
is used, because `pred_v` implies `cti_ok`. Cost: **1 retire in 10.4 million**
over the 40 M-cycle Linux cosim.

This is the third instance of the class. `irq_inject` was a live mux select into
`u_fetch/npc -> u_bp/btb_q` and was fixed by registering it
(`ooo2_core.v:1722`); D4 is the same pin reached from a guessed index; I4 is the
same pin reached through a valid mux. The general form: **for every array in the
design, write down its address expression and name the flop each term comes
from.** A term you cannot name that way is the bug, whatever the comment above
it says.

**I8. The compare DECIDES; it does not ENABLE.**
A set-associative lookup produces its answer -- the tag compare -- as the last thing in
the cycle. Everything that is captured *because* of that answer and consumed only under it
(the MSHR copy of the request, the window registers, the response fields, a victim's line
buffer) can be captured on the STATE alone and left as garbage when the other outcome
happens; only the bit that IS the outcome has to wait for the compare. Measured 2026-09-03
on `rv_cache` (`1271c96d`): `hit` sat on ~900 clock-enables -- the cbo.zero arm's
`linebuf <= 0` alone was 325 endpoints at -0.033 in the integrated build, 14 levels and 80%
route -- and on the ADDRESS of a second array (the Zicbom dirty test
`dirm[flat(hway,cih)]`, the module's own worst path at 19 levels). Moving the enables to
state-decoded conditions, masking the install data instead of zeroing the buffer, shifting
both ways' windows before the way select, and taking the dirty test one cycle later on the
registered slot took the D$ alone from 181 to 206 MHz with the cosim bit-identical. The
general form: for every register whose enable or address contains a late signal, ask
whether the value would be READ if the signal were false. If not, the enable is the state.
The one exception is a register another machine shares (the fill machine's `linebuf`), and
that needs the sharing predicate (`~f_v`), which is still a register.

**I9. An arbitration that preempts a request must not sit in the requester's COMPLETION.**
`ooo2_lsu` granted its FSM to the pre-translated port with priority over M's request, and
implemented that by gating M's request to the MMU on `~pt_start`. Correct for an access
that starts the FSM; for the translate-only pass -- which needs the MMU and nothing else
-- it put the port's grant, i.e. `ooo2_sq`'s live bits, `ooo2_lq`'s candidate and the
alias matrix, in series with M's completion for every plain load and store. M's completion
is the writeback valid (the wakeup broadcast, fanout 194), the redirect (the fetch adder,
the F/X queue) and the hpm events, and 1708 of the 3401 endpoints under +0.35 ns in the
2026-09-03 routed checkpoint began at `u_sq/v_reg` for that reason alone, 22-27 levels each,
the worst at -0.035. The general form: a grant decides what the loser may START. Whether
the loser is DONE is decided by the loser's own unit, and a broadcast completion (a wakeup
valid, a redirect) is built only from the terms that can actually produce it -- an access
that completed, a latched fault, a fixed-latency unit -- never from the whole `done` mux.
Keep the full expression alongside and assert the two equal every cycle, so the narrowing
is checked rather than argued.

**I10. A LUTRAM bank has ONE write port, and it must be written by ONE statement.**
Two `if`s that write the same bank under conditions that can never both be true are still
two write ports to synthesis, which cannot prove the exclusion and demotes the whole bank to
flops -- silently, with a warning nobody reads, and with every read of that bank becoming a
mux across N flops (I7). Gate V2 (2026-09-05) lost all eight parity banks of the rename
free lists and the ROB's entry array this way: the head's free and the second's land in
different banks by parity (`t2 = t + fre`), and each was written as its own `if`. The shape
that stays a RAM is one `{we, addr, data}` per bank, muxed from the writers BEFORE the
array: `if (fw0) fl0[fa0] <= fd0;`. Reads are free to multiply (duplication buys read
ports); writes never are. tools/ram-manifest.txt names every array that must stay a RAM and
the build fails the moment one does not -- that is the check that caught this, six minutes
into synthesis, and the reason the manifest lists the banks by name.

