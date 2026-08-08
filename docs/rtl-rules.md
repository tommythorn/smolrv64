# RTL rules

Rules derived from the defect record, not from taste. 1177 commits, 218 with
defect-class subjects. Nearly all of them fall into six generators; each rule
below exists to make one of those generators structurally impossible rather
than to catch its next instance by review.

Every rule cites the commits that paid for it. If a rule looks like overhead,
read the commits.

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
invariant check does not. The default 215-test gate defines no `VDEFS`, so
every check written behind `LSU_ASSERT`, `WTCHK`, `FL_ASSERT`, `FL_DBLALLOC`,
`PRFVAL_CHK`, `SEQROB`, `SERSQ`, `CSRDBG`, `WBGUARD_DBG` is compiled out of
every regression run. `21e9fe5` is titled *"keep the two invariants that found
the boot wedge"* — and CI does not compile them. Move checks to always-on
`$fatal`/`$display`; keep only the flood-volume tracers gated.

The model to copy is already in the tree: `cache.v:462` `$fatal`s if two L2
transactions are ever outstanding, unconditionally, because the ack is
untagged. That is how every assumed-impossible condition should be written.

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

**B4. A second copy of a line is a coherence problem.**
The prefetch buffer (`pf_val`/`pf_addr`/`pf_line`) holds a line that is not in
the tag array and is not covered by the duplicate-line tripwire (`24f7dd3`).
There must be an always-on assertion that no line is valid in both the array
and the prefetch buffer.

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

---

## E. Widths and lint

**E1. The lint gate is `-Werror` on the load-bearing rules.**
`WIDTHTRUNC`, `WIDTHEXPAND`, `WIDTH`, `CASEINCOMPLETE`, `LATCH`, `UNOPTFLAT`,
`UNDRIVEN`, `MODDUP`, `PINMISSING`. Waivers are file-scoped in a
`verilator.vlt` config, never global `-Wno-` flags — the current build
suppresses all of these globally in order to quiet imported CVFPU, and that is
why `e552aab`, `c629047`, `4ac3d92`, `f157d1d` and `857dfc8` reached a
bitstream or a wrong measurement. `-Wno-fatal` means none of it stops a build
anyway.

Style rules (`TIMESCALEMOD`, `UNUSEDSIGNAL`, `UNUSEDPARAM`, `DECLFILENAME`,
`ASCRANGE`, `UNSIGNED`) stay off. Six of the twelve suppressions are
load-bearing; the rest are noise.

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
`rs232.v` defines `rs232rx` and `rs232tx`; so do `rs232rx.v` and `rs232tx.v`,
with *different ports and different default baud*. Which definition wins is
tool-dependent. This is the console.

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
`IW ∈ {1,2,3,4}` × `CACHE ∈ {0,1}` × `CKMAX ∈ {1,2}`. `023a5df`, `61f7d0a`,
`41061b8`, `9444f81` all only exist off-default. The Verilator gate is ~8s;
the matrix is minutes.

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
