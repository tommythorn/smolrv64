# PLAN 2026-11-06: the machine past the aligner on a doubled clock, at half the width

Tommy, 2026-11-06: *"Branch and make a version with the same performance and much less area.
How? Double clock [most] everything post the aligner with half the resources (everything used
twice per cycle). Of course we can't meet timing without also allowing all latencies to double
(thus same wall-clock latency). [...] go for the minimal proof of concept first, and then grow
it incrementally until we can't save any more area."* And: *"I would have thought only the
aligner needed to be in the slow domain."*

This differs from `PLAN-2026-10-27-333mhz.md`, which doubled the frontend too: there the BTB loop
(-0.293 at 3 ns) and the fetch ring's next head (25 levels) were the hardest loops. Here fetch,
the predictor, the ring and the aligner stay at 166.67 MHz exactly as they are, and only what
follows the aligner moves to the 2x clock.

## 1. The clocks

`probe_clk` is `BUFGCE_DIV(ui_clk) / 2`. The 2x clock is `ui_clk` through a second BUFGCE_DIV at
divide 1 (`core_clk2x_buf`, `smolrv64_shell.v`), with the same clock root: both clocks see the same
buffer delay, every `probe_clk` edge is a `clk2x` edge, and Vivado derives both waveforms. Raw
`ui_clk` would not do: it leads `probe_clk` by the buffer (~2.7 ns, `smolrv64_shell.xdc`), which
leaves a `probe_clk -> ui_clk` path 0.3 ns.

In the 2x domain, which half of a `clk` cycle it is comes from two flops: `tg` flips at every
`clk` edge, `tg2` is `tg` as `clk2x` last saw it; they differ in the first half and agree in the
second (`smolrv64_frontend`, `h2`).

## 2. The rule that keeps an increment cycle-identical

A `clk` stage that serves N slots a cycle becomes a `clk2x` stage that serves N/2 slots a half:
the first half's slots in the first 3 ns, the rest in the second. If the half-width logic fits in
3 ns, the second half's results arrive at the `clk` edge exactly when the whole stage's did, and
the first half's 3 ns early. Nothing downstream sees a difference: **the machine is
cycle-identical to main**, so performance is unchanged by construction and the 300 M-cycle Linux
lockstep must retire exactly main's count (379,230,245 at f5662859's RTL).

Where the half-width logic does not fit in 3 ns, the stage takes two `clk2x` cycles: the first
half's results still arrive by the `clk` edge, the second half's 3 ns after it. At a crossing back
into `clk` that is a cycle, so such an increment is not cycle-identical and is priced like any
latency change (the GB5 SimPoints). Inside a contiguous 2x domain it is only the serialisation of
a narrower machine: op 3 starts 3 ns after op 1.

Each increment is therefore judged on area and timing alone when it is cycle-identical, and on
area, timing and the SimPoints when it is not.

## 3. Increments

**Increment 0 (the proof of concept): decode.** The bundle register (`pb_in`, up to FW = 6
slots) stays `clk`'s. Four `smolrv64_dslot` decoders on `clk2x` (main has eight, one per bank)
take the bundle's first IW = 4 slots in `clk`'s first half and the other two in the second. Each
decoder is two `clk2x` stages (`REG=1`: a register after `decode_slot`; the branch compares, the
dispatch class and the record are the second stage), so the first IW slots are written at `clk`'s
edge, exactly when a whole bundle was, and the other two at the mid-cycle edge 3 ns later.
Nothing is late: the IR reads at most IW positions past its head and a bundle lands at the head
or beyond, so in the cycle after a push the IR can need only the bundle's first IW slots. The
same fact makes the mid-cycle write invisible that cycle (asserted always on), which is what lets
the queue RAMs' write-clock -> read-data arcs into `clk` be multicycle (`smolrv64_shell.xdc`,
`ooc.tcl`). A half's consecutive positions are distinct mod 4, so decoder d feeds banks d and
d+4 and the bank enable picks: no data mux.

The first try split the bundle 3 + 3 with one-stage decoders: -1.776 ns at 3 ns (17 levels, the
pc + imm adder and its compare inside decode). Splitting IW + rest made two stages free.

Measured out of context (`smolrv64_frontend`, 6.000 ns with `clk2x` at 3.000 ns):

| | main f5662859 | increment 0 |
|---|---|---|
| WNS | +0.108 | +0.101 (`clk2x`: stage A 9 levels +0.101, stage B into the RAM +0.159) |
| CLB LUTs | 17,262 | 15,939 (-1,323, -7.7%) |
| registers | 2,472 | 3,158 (+686: the decoders' stage registers) |

Cycle-identical: the 300 M Linux lockstep retires 379,230,245, main's count, with all 294
per-million checkpoints (cycle, retires, pc) equal to main's.

**The IR's read side does not follow cheaply.** Measured on increment 0's routed frontend: the
read pointer reaches the IR's records in 3 levels but 4.4 ns of route (`q_rp` fans out to every
bank's read address, 1,076 endpoints), so a first half could not deliver its records by 3 ns.
The IR, rename and the dispatch decision are one combinational chain per `clk` cycle; they move
together, with the lanes, or not at all.

**The lanes: calibrated (2026-11-06, `~/claude-scratch-hdmi/2x/cal/stag2x.v`, out of context,
3.000 ns).** One physical lane on `clk2x` with the staggered ALU and a 7-way bypass of registered
halves (the PRF, its own last two results, the other lane's two, the load stream):

| stage | slack | levels | logic / route |
|---|---|---|---|
| S1: bypass -> the low 32 bits (add/sub, Zba, logic, Zbs, sll, ext, the W forms whole, link), the low compare | +0.142 | 11 | 1.130 / 1.710 |
| S2: bypass -> the high 32 bits from S1's carries, the compare's verdict | +0.105 | 14 | 0.989 / 1.887 |
| S2: the late class whole (srl/sra/ror/rol/bext, slt, min/max, czero) from S1's low and S2's high | +0.148 | 12 | 1.063 / 1.771 |

1,942 LUTs and 879 flops for the lane (main: `u_x` is 1,989 LUTs per lane, four of them, plus each
lane's bypass in the core). Counts and rev8 (clz/ctz/cpop/rev8/orc.b) go to a slow pipe.

**The lane step, cycle-identical where it can be.** Physical lane 0 executes logical lanes A
and C, physical lane 1 B and D: the A/B op in `clk`'s first half, the C/D op in its second. The
four schedulers, their select registers and the wake stay `clk`'s. With the staggered ALU a
dependent issued at the next `clk` edge reads its producer's low half in its S1 and its high
half in its S2, so every ALU -> ALU chain keeps today's timing whichever phase either op is in.
The second phase's results reach full width 3 ns after today's, which only full-width consumers
see: branch resolution (+1 `clk` cycle on a C/D-lane mispredict), the multiplier's and the other
units' operands when the producer was a C/D op the cycle before, a late-class C/D result read by an
A/B dependent the next cycle, and the PRF write (written on `clk2x`, before any read needs it).
Those are the step's price, to measure on the SimPoints before it lands.

**But the PRF read needs a stage of its own.** Today a lane reads the PRF inside its execute
cycle (the select register's tags -> the LUTRAM shards -> the operand mux -> the ALU), using the
6 ns. The calibration fed registered operands; with the LUTRAM read ahead of S1 that stage needs
about 3.9 ns. So a `clk2x` lane is S0 (PRF read), S1, S2, and every lane's results move 3 ns later
than in the calibration: ALU -> ALU chains still keep their timing (the dependent's own S0 covers
the delay), but every branch's verdict lands 3 ns later, a `clk` cycle on every mispredict's
restart, in all four lanes. The lane step is therefore not cycle-identical. Its main price is
measured as a knob: `exp/red1` (a lane's mispredict restarts the frontend one cycle late; the
freeze still sets at once) against main 390bed9c, on the 77 GB5 SimPoints: **+0.27% cycles**
(the sum), Clang +1.65%, Text Compression +1.14%, Text Rendering +1.01%, Image Compression
+0.72%, the FP subtests +0.02% to +0.16%.

**And a C/D-lane address is a cycle late.** With no address waiting, the oldest lane delivering a
load or store bypasses its adder straight into M's register at the `clk` edge (`by_va`, "a cycle
sooner"). A C/D op's address is whole 3 ns after that edge, so it falls back to the arrival RAM
(`u_la`/`u_sa`) and M a cycle later: half the loads lose a cycle of load -> use, a larger price
than the mispredicts'. Keeping an AGU (and the branch compare) per logical lane at full rate,
fed by its own operand mux, protects both, and leaves only the ALU proper double-pumped: about
3K LUTs.

**Where the increments stop.** Mixing a `clk2x` piece into the `clk` machine is cycle-identical only
where the second half's result has slack before its consumer: a queue (decode, increment 0). Every
other boundary pays the half-cycle misalignment where it crosses back into `clk`. The large saving
-- rename's map reads and compares, the schedulers' wake fan-in, the bypass network, the PRF's read
copies, the lanes -- needs the backend's loops on `clk2x` at once, two-wide, every latency doubled
in cycles: the design of `PLAN-2026-10-27-333mhz.md` behind an unchanged frontend, whose latencies
equal today's in nanoseconds by construction (one crossing, at the decoupling queue). That is a
redesign of the core's issue-to-writeback machinery, to be decided on its own: the
calibrations above say its hardest stages close at 3 ns out of context.

**The ceiling, measured (2026-11-06, `make ooc MODULE=smolrv64_core HIER=1`, synthesis only).**
Main f5662859 at IW=4 against the same RTL at the end state's shape: IW=2 with the totals kept
(two lanes of 20 entries, 128-register shards, 128 ROB rows of two; the FP shards went to 128 too,
as the pending table needs one shard size, which slightly overstates IW=2):

| | IW=4 | IW=2 | |
|---|---|---|---|
| **core** | **112,398** | **91,386** | **-21,012 (-18.7%)** |
| rename | 8,181 | 2,504 | -5,677 |
| the core's own logic (bypass, dispatch, staging) | 16,402 | 10,855 | -5,547 |
| the lanes' ALUs (`u_x`) | 7,956 | 3,978 | -3,978 |
| SQ, LQ | 18,317 | 15,453 | -2,864 |
| frontend | 18,297 | 17,169 | -1,128 (stays IW=4 here; increment 0 takes -1,323 instead) |
| the lane schedulers | 6,311 | 7,631 | +1,320 |
| ROB, PRF, tq, the rest | | | -2,600 together |

So the whole programme is worth about 21 K LUTs, a fifth of the core, before the extra pipeline
registers the doubled latencies need. The FPU, the caches, the CSRs and the MMUs do not move.

**Rename alone closes at 3 ns at IW=2** (`smolrv64_rename` out of context: IW=4 at 6 ns +1.268;
IW=2, 128-register shards, at 3 ns +0.219, 15 levels). It is the largest single saving, and it
can be an increment: the IR read on `clk2x` with a register after it, rename on `clk2x`, and a
queue after rename (written mid-cycle like increment 0's), from which dispatch takes its group at
`clk`. The price is one cycle between the decode queue and dispatch, exposed after every
redirect and whenever the queue runs dry; the knob `exp/rn1` (the IR sees a bundle one cycle
after it lands) measures it on the SimPoints (`rn1-sp`). Two costs the knob leaves out: rename recovers by walking the ROB back to
the mispredicted branch, and an op renamed into the queue is not in the ROB yet, so the walk
covers the queue first (at most two groups: a row or two more of walk under the redirect's
freeze); and a half renames two lanes' slots, so each half reaches two of the four shards' free
lists, a mux the IW=2 calibration (two shards) does not have.

**Measured, `exp/rn1` against main 390bed9c on the 77 SimPoints: +0.41% cycles** (the sum);
Clang +1.91%, Text Rendering +1.73%, SQLite +1.34%, HTML5 +1.30%, Text Compression +1.28%, Image
Compression +0.99%, Navigation +0.83%; the FP subtests +0.01% to +0.20%. About 1% of integer
performance for 5.7 K LUTs, larger than the lane restart's price (+0.27%) because every redirect
and every drained queue pays it, not only a lane's mispredicts. Against "the same performance"
it does not land on its own. The same nanoseconds come back in the end state only if the IR read,
rename and the dispatch decision, one 6 ns cycle today, fit two 3 ns stages together; the IR's
read side alone (route-bound, above) says they take three.

**The scheduler's one-cycle loop does not carry over (2026-11-06, `smolrv64_iq` out of context,
NSRC=2, FIXEDL=1, OLDEST=1).** Main's lane scheduler (NENT=10, six wake ports) is +1.988 alone at
6 ns, and in context the same loop sits on the critical families. At 3 ns, alone:

| entries | wake ports | SPEC | WNS | |
|---|---|---|---|---|
| 10 | 4 | 1 | +0.218 | |
| 12 | 4 | 1 | +0.191 | |
| 16 | 4 | 1 | +0.045 | |
| 20 | 4 | 0 | +0.102 | |
| 20 | 4 | 1 | -0.138 | 11 levels: the wake compare, the pick, `sp_only[isel]` into `e_sp` |
| 20 | 6 | 1 | -0.210 | |

A margin that thin alone is gone in context, where the same structure loses about 2 ns today. So
the 2x scheduler is the two-cycle loop of `PLAN-2026-10-27-333mhz.md` section 5 (pick in S0, the
picked entry's wake broadcast in S1, +0.119 at 20 entries): a dependent picks two `clk2x` cycles
after its producer, 6 ns, which is today's ALU -> ALU.

**Then, in order, each a contiguous step of the 2x domain toward the back:**

1. The queue's read side and the IR: two heads a half instead of four a cycle (the read ports
   2*IW = 8 -> 4, the group rules over two slots).
2. Rename: the map read 8 -> 4 times, the intra-group compares 4^2 -> 2^2, the free lists.
3. Dispatch and the schedulers: two lanes of 2*NI entries (the totals stay), the wake broadcast
   lanes x entries x ports halves.
4. The lanes: PRF read copies, the bypass network (lanes^2), the ALUs; the ALU split low half /
   high half (`PLAN-2026-10-27-333mhz.md` section 6) so ALU -> ALU stays 6 ns.
5. The ROB's completion ports and commit, the LSU's ports, the FPU's issue.

At every step the 2x domain is contiguous and its crossing back into `clk` is one place. The
step stops paying when the logic it moves is no longer width-proportional (arrays, the caches).

## 3b. The whole stack at once (Tommy, 2026-11-06 morning)

*"I think we should try the whole stack despite the IPC cost: decode through execution. It may
be that we can recover some of the loss with some amount of redesign, but first let's see the
savings on a full build."*

Beyond decode-through-execution, only the LSU's ports (LQ, SQ: -2.9 K at IW=2) and the ROB's
completion and commit ports (-0.6 K) are width-proportional, and both are inside the core. The
FPU, the caches, the CSRs and the MMUs are one unit or one array each: `clk2x` buys them nothing.

**The cut.** `clk`: fetch, the predictor, the fetch ring, the aligner and the bundle register
(the fetch side of `smolrv64_frontend`), the iMMU, the I$, the memory arbiter, the L2, the
devices. `clk2x`: decode, the queue, the IR and everything after it in `smolrv64_core`, and the
D$ (`PLAN-2026-10-27-333mhz.md` pipelines it on the fast clock; its memory side is valid/ready
channels, the cheapest place to cross).

**The crossing rule.** A `clk2x` flop that `clk` logic reads changes only at the `clk2x` edges
that are `clk` edges (the end of a second half, `h2`); a `clk` pulse that `clk2x` logic counts
is counted in one half. Then:

- fetch -> queue: the bundle register is `clk`'s; `q_room` is computed at the `clk` edge only, so
  a push never changes its mind mid-cycle; the decoders already take the bundle in two halves.
- redirect -> fetch: held until the `clk` edge, the latest redirect's payload winning; until the
  fetch side has seen it, the queue drops the bundles fetch still pushes from the old path.
- training: through the training queue, written on `clk2x`, read on `clk`.
- the D$'s memory channels: a 2:1 valid/ready bridge.

**M1: the area, functionally.** Today's RTL at IW=2 with the totals kept (two lanes of 20,
128-register shards, 128 ROB rows of two), on `clk2x`, built at half speed (`clk` 83.33 MHz,
`clk2x` 166.67 MHz) so the backend's single-cycle loops keep today's 6 ns: a full build that
meets timing, shows the area, boots on the board and runs the lockstep. Its IPC is not the
design's (the backend's latencies are single `clk2x` cycles).

**M2: the same RTL at 166.67 / 333.33 MHz.** The timing census names the loops to redesign (the
two-cycle scheduler, the staggered ALU, the PRF's read stage, the D$'s pipeline); then the
redesign, and the SimPoints.

Step A (the shape): IW=2, NI=20, 128-register shards (the pending table wants one shard size, so
the FP shards are 128 too), 128 ROB rows. Two IW=4 assumptions fixed on the way (a lane index
taken as two shard bits; the debug bus's ROB row field). riscv-tests 240/0, lint clean.

**M2's census (routed, 2026-11-07): -2.722 ns, 72,438 failing endpoints of `clk2x`'s 178,133.** By
clock pair: 62,096 `clk2x` -> `clk2x` (6 ns logic in a 3 ns cycle: the redesign below), 15,501
`clk` -> `clk2x` and 8,188 `clk2x` -> `clk` (crossings timed at one `clk2x` cycle), 743 `clk` ->
`clk`. By unit the most: the D$ 13.8 K, the FPU 5.8 K, the core's own logic 5.7 K, the SQ 5.7 K,
the CSRs 6.4 K (3.6 K of them from `clk`), the PRF 5.4 K. The largest family starts at the phase
flop: `h2` came combinationally from `tg`, a `clk` flop, into the D$'s answer, the LSU's completion
and the schedulers' wake and select (18 levels into `e_sp`). Correctness first, IPC after: every
stage that does not close gets a register, and the GB5 run on the board prices it.

**Step 1, the queue's read side and the IR.** At M2 the take loop ran from the lanes' and the LQ's
valid bits through the credits' counts into `q_rp` (-2.68 ns, 16 levels), and the head's record
rebuild from the queue's read into `d_rec` (-2.36 ns, 8 levels). Now:

- THE PICK STAGE (`smolrv64_frontend` P): the take registers the lane-ordered queue records it read
  (`p_raw`), and the IR rebuilds them the next cycle. Two groups are in flight between the take and
  dispatch; every credit and gate counts P's beside the IR's (`fe_pv`, `fe_pgc`, by lane), and P
  passes its group to the IR only while dispatch is not frozen (`drop`: every freeze is raised by
  an op already dispatched, so what P holds then is younger). A group's loads take one entry a
  bank, so the LQ's bank credit counts, net of the IR's and P's loads, the banks sure to keep two
  free entries and those sure to keep one (a bank drops a level where both groups hit it -- at most
  the smaller group's count of banks -- or either does); rename's low-water mark covers three
  groups (the FP shard's 3*IW).
- THE COUNTING CREDITS ARE REGISTERED: a scheduler's free entries, the LQ's two bank counts, the
  SQ's and the F stage's room, each a cycle old net of everything in flight then; the take compares
  them with P's group, the one it has not seen (whose loads can empty only banks sure of one). Frees in between, and an op leaving the F stage,
  are not credited.
- `h2` is a `clk2x` flop (`smolrv64_x2ph`: `h2r <= tg2 != tg`, asserted equal to the clocks'),
  and the core's reset a replicated `clk2x` flop.

The benches: the Linux bench's LQ credit check is four free entries a bank a cycle ago (four
groups can hit a bank: that cycle's IR and P, P's now, the take), and its
serialise accounting charges a cycle a serialising op in P held to that op (in the IR when the cycle
is classified).

Step 1's build (3a4e0b98): -3.315 ns overall; in the step's region the phase flop's 8,269 paths are
gone and the take loop is -2.04 ns (9 levels, from P's class bits through the credits' compares
into `q_rp`), the queue read into P -1.23 ns at 4 levels (wire: 3 ns is not enough for a read of
eight banks plus a pick), and the bundle register into the decoders -1.64 ns (a `clk` -> `clk2x`
crossing has one `clk2x` cycle). The second pass:

- THE HEADS ARE REGISTERED (`hr`): the queue reads its 2*IW oldest entries every cycle into flops
  and the next cycle's take works on them (the heads are `hr[n..]` after a take of n), so the take's
  loop starts and ends at flops of the frontend: the heads' classes, the rules, the pop, `q_rp`.
  A head is valid (`hv`) when the read saw it: the count less the pop, chosen by the pop among
  compares of `q_cnt`.
- THE COUNTING CREDITS ARE REGISTERED BITS: each is the compare done a cycle early against the most
  P's group can hold (one entry a scheduler, IW FP ops, IW stores, a ROB row; for the LQ only the
  banks sure of two free entries count). Only P's class gates stay live (a head op, a CBO, a
  divide, a serialising or CSR op in P), each an OR of flops.
- DECODE IS THREE `clk2x` STAGES: a register takes each decoder's raw slot (`px`), so the crossing
  from the bundle register is the slot's pick alone. A bundle is placed (`q_wp`) at the `clk` edge
  that ends its push and counted at the mid-cycle edge after it, when its first H0 slots are
  written; the rest are written at the next edge. `q_room` counts the bundle placed and not yet
  counted, and no pop. READ BEFORE WRITTEN (asserted): no slot of a bundle is taken in the cycle
  after its count, and none past the first H0 in the cycle after that.

**The FPU: twice the pipeline registers.** fpnew's datapath closed 6 ns with five (`PIPE_REGS`, DISTRIBUTED);
at 3 ns it takes ten, the same latency in ns. Still four ops in flight (`NFLIGHT`).

**Step 3, first part: READY IS A REGISTER.** At M2 and step 1 the largest family is a load's landing
through the wake compares, the pick and the payload read (`u_lsu/fw_n`, `k_f_q` -> `ln[].pl`,
`e_sp`: 12-17 levels, -3.37 ns). A scheduler's pick now reads only flops: a wake (a writeback's
compare, the dependency matrix's column) sets `e_r` at the edge and the entry is a candidate the
cycle after. A writeback's consumer issues a cycle later than before; a one-cycle producer's
consumer issues two cycles after it (the same 6 ns as main's back-to-back). The speculative
load wake (D$ plan step 3a) is gone: its one-cycle verdict was built on a same-cycle pick.

**Step 5, first part: THE ROB'S COMPLETIONS ARE REGISTERED.** The completion ports' cones (a
landing, the walker's answer through the SQ's take, a lane's resolution) ended at `done` (`wk_lk_va`
-> `u_rob/done`, 16-17 levels). They end at a register now, and `done` is set the cycle after, if the
entry still holds that op (a kill or a flush in between frees the slot, which may be allocated
again). The irrevocable pointer's forward (the store's take) reads the registered completion, in
the cycle its `done` is written. `done`'s consumers -- commit, the irrevocable pointer, the replay
flag -- all only move later (rule D21); `h_fin` (the head committing in its flush cycle) is live.

**Step 4, first part: THE LANE IS THREE STAGES** -- the pick, R, E. `PLAN-2026-10-27-333mhz.md`
section 5 measured the operand read (-0.024) and a one-stage ALU (-0.638) each against 3 ns alone;
M2 had both in one stage after the pick (`ln[].ent` -> `ln[].qval`, 15-24 levels). R holds the
picked op, reads the PRF and the write registers' bypass (the scheduler's registered matches at its
entry, as before) and registers the operands; E executes from flops. Everything that left the
execute stage leaves E. The multiply's slot two cycles after its execute is reserved by holding
the pick while it executes (`e_mul`); the int-to-FP handoff is busy while one is in R or E. A
one-cycle producer's consumer still executes two cycles after it.

**Step 4, second part: EXECUTE IS TWO STAGES** (`smolrv64_exec` PIPE=1). One stage of ALU from
registered operands misses 3 ns alone (-0.638, the 333 MHz calibration). E1 runs the ALU's units
(adder, shifter, compares, logic, count and permute) and registers them with the group select; E2
selects the result, sign-extends a W form and runs the branch unit from registers. The address is
E1's, so address generation, the multiply's start, a divide's and an int-to-FP op's hand-off stay
in E1; the write register, the ROB completion and a CTI's resolution are E2's. The multiply's slot
is kept by holding the pick while it is in R. A one-cycle producer's consumer issues three cycles
after it (the dependency matrix's wake registered, `swk_q`) and executes three after it: 9 ns, a
cycle more than main's 6 -- the staggered ALU (section 6 of the 333 MHz plan) is what wins it back.

**Step 7, first part: the clk -> clk2x levels and pulses take a clk2x register at the boundary.**
Step 1's build had 11,969 clk -> clk2x failing endpoints: the PLIC's arbitration (`has_irq`, 5 K into
the CSRs and through them the whole core), the shell's reset into the D$ (1.7 K), the devices'
answers into the lanes. The interrupt levels and `mtime` reach the core through a clk2x register;
the SoC's clk2x side (the D$, the arbiter, the read door, the walkers' adapters, the line client,
the crossings) takes its own clk2x reset flop, as the core does; a device's read answer and write
ack are registered in the first half and taken by the LSU in the second.

**Step 5, second part: THE LD STREAM IS REGISTERED.** A load's answer through the LSU's alignment
was the stream's whole cycle into the lanes' landing buffers, the FP shard and the snoop
(`u_dcache/rd_data` -> `g_lp[].u_lbd`, 7-11 levels, -3.16). What the stream writes, wakes, snoops and
pushes into a lane is now a register a cycle after the landing, dropped by a redirect in the
landing's cycle. Its ROB completion is not delayed: an FP register's on M's port at the landing, an
integer register's on its lane's port, straight through or at the drain. The landing buffers are
eight deep and the FE stream's gate is LBN-6: with the lane four stages deep the drain starts
within four cycles of the first entry, not one.

**Step 7, second part: THE RULES AS CONSTRAINTS.** A clk2x register that changes only at clk edges
carries `(* x2_cedge = "yes" *)`, one whose captures from clk logic take effect only at clk edges
`(* x2_ccap = "yes" *)` (Vivado keeps both as cell properties: checked out of context), and
`smolrv64_shell.xdc` gives each direction probe_clk's period. Marked: `smolrv64_x2f2s`'s held pulse
and payload, the queue's room, the frontend's CSR copies (`priv_fe`, `satp_fe`, ...), the interrupt
presentation and its interlock, the I$'s line register (`l2_rdata` is the register now: its one
reader takes it a clk cycle after the ack), the fetch hold (`imem_hold`, now a register updated at
clk edges), and the devices' write port: `dw_*`, the LSU's write as it stood at the clk edge,
dropped at the edge after its (registered) ack, so a device never sees one write twice.

**m2b's build (b906ad3a): -2.187 ns, 77,174 failing endpoints** (step 1's second pass: -2.530,
88,564). The lane's R stage held the picked entry against reallocation (`hold_v`), which put R's
valid in front of the scheduler's slot choice and the payload write (5.4 K endpoints): nothing needs
the hold now -- the payload is read at the pick and R reads the entry's registered matches the cycle
after, before a new occupant's dispatch can change them -- so none is held. A multiply's and an
int-to-FP op's busies decoded R's and E1's instruction words in front of the pick: they are bits
predecoded at dispatch into the tag array (`psmem`) and carried in flops.

**The ordered redirects are registered.** M's trap and the SYSQ's redirect were decided through
the CSR file's next state (mcause, the trap target, the write decode) and were the flush itself, in
front of every flush arm (the ROB's irrevocable pointer, the FPU, the divider: m2b's
`lb_n -> u_fpu`, `sy_addr -> u_csr`, 18 levels). The decision is registered and the flush, the
frontend's pulse and the redirecting op's commit (h_fin) are the next cycle's. Nothing younger
commits or drains in between (in-order commit; the irrevocable point is a register that has not
passed the op; a trap's commit is killed in both cycles), and the frontend's pulse, delayed with
the flush, wins over a younger restart or resteer in its cycle.

**THE FPU RUNS ON clk** (`smolrv64_x2fpu`), as section 3b's principle says it may: one unit, which
the 2x clock buys nothing. With ten pipeline registers fpnew still had 20-level stages at 3 ns (its
registers reset asynchronously, so synthesis cannot retime them: m2b's `u_fpu -> u_fpu`, 4.7 K
endpoints); on clk with five it closed main's 6 ns, and it takes an op a clk cycle, as at 166.67 MHz.
The crossing: the core writes an op into a clk2x buffer (`ib`, its ready a flop), which moves at a
clk edge into the register the FPU reads (`ob`, changing only at clk edges and holding until the FPU
takes it there); the result -- and `res_fp`/`res_int`, which are its valid by destination kind -- is
registered in the first half and is the core's for one clk2x cycle; the flush goes through
`smolrv64_x2f2s`, and until the FPU has seen it every result is of a killed op and is dropped.

**THE DIVIDER RUNS ON clk** (`smolrv64_x2div`) for the same reason (m2b: `u_div/dvsr -> u_div/rem`,
11-12 levels). The clk side is never aborted -- an abort and the start after it could reach it in
one clk cycle, where its own abort wins -- so the clk2x side drops a start not yet handed over and
discards the result of the divide the divider is running, and a new start waits until it is idle.
Its busy toward the core is a waiting start or a divide still wanted.

**m2c's build (968f669b): -1.976 ns, 66,760 failing endpoints, TNS halved.** The schedulers' own
loops led the core (`u_iq -> u_iq` 4.1 K, `-> plmem` 3.2 K): the pick's result cleared the issued
entry's valid, and the lowest free entry, chosen from the valids, named every entry's writes. THE
ISSUED ENTRY CLEARS A CYCLE LATER (`iss_last` masks it from the pick meanwhile) and THE FREE SLOT
IS RESERVED A CYCLE AHEAD (`fs_v`, `fs_ix`: only a dispatch allocates, so a slot free now stays free
unless dispatched into now), so the pick and the dispatch both read flops.

**THE FE STREAM IS REGISTERED** as the LD stream is (m2c: `icr_room -> u_prf`, `-> u_lbd`, 2.3 K):
the F port's and the MD stage's results write, wake, snoop and push a cycle after the landing; an
integer one completes on its lane's port, straight through or at the drain; the FE gate is LBN-7,
its push being a cycle behind the gate's decision.

**m2d's build (93103479): -1.918 ns, 70,153 failing endpoints; the FPU's families are gone.** The D$
leads (17.3 K of its own), 10 K of it the install (`fl_m -> way[].ta/tb`, 3 levels): the installing
MSHR's set, way, tag and merge-dirty bit muxed by `fl_m` into every way's tag and state writes. They
are registers now, taken from the MSHR as its last beat lands (stable from the first beat: a store
to the line waits for the install), the way one-hot, replicated.

**THE D$ LOOKUP TAKES TWO CYCLES** (m2g, b27b2fa8: -2.108 ns, 51.3 K endpoints, the D$'s compare
leading: `s1_pa -> way[].va/da/ua`, `-> rd_data`, `-> arr[].u_a`, `ms_way -> way[].va`). C1 reads
the set's state and compares (the tags, the MSHRs', the write-back buffer's, the victim); C2 decides
from those registers and writes. A request issues only while C1 is empty, so C1 and C2 never both
hold one: one lookup per 6 ns, the rate the 6 ns cache had. The request's fields stand through
both, so C2 reads the set's tags (the victim's write-back) and the arrays' answer arrives in C2 from
the block RAMs' output registers (read latency 2: a load answers a cycle later). Between C1 and C2:

- the store lookup may write a dirty bit in C1's set; C2's copy of the dirty bits takes it, and the
  store lookup waits only for C2 (a load that hits excepted, as before);
- a write-back or the queue's head may free what C1 saw busy; C2's wait counts C1's freeing;
- an outside flush may block a line; C2 compares the block itself;
- a line installs only with C1 and C2 empty: the install comes a cycle after the last beat's write
  (`fl_p`) and the lookup yields in both.

The clean walk reads the state through the lookup's port, so it starts only with the lookup empty
and taking nothing.

**THE STORE LOOKUP TAKES TWO CYCLES TOO** (`sb_pa -> way[].da`, `-> arr[].u_a`: its compare drove the
arrays' byte enables and the dirty bit's write). SB1 reads the valid bits and its tag copy and
compares (and takes the MSHR filling its line); SB2 resolves from those registers and writes. The
entry behind the head issues while the head is in either stage, so stores still go one per 6 ns.
The main lookup's decision in SB1's set (but a load that hits) or an install there changes what SB1
saw: SB2 then sends the store back to the queue. The D$ bench counts both new hazards (a store's
view changed after SB1, a store's write folded into C1's dirty bits) and demands each occur; with
either fix removed it loses stores.

**TWO D$ WRITES AND READS TAKEN A CYCLE AHEAD** (m2g: `wt_rdy -> arr[].u_a`, `-> s1_pa`, 655
endpoints; `fb_way -> arr[].u_a`, 486). The replay pick is registered with its entry's PA and NC
bit: the lookup never issues two cycles running, so the entry picked a cycle ago is still waiting
and ready when it issues (asserted), and a stale pick meets the hold. The fill beat's two arrays
(one-hot) and each array parity's write row (the beat's, else SB1's, which is SB2's a cycle later)
are registers taken as the beat arrives.

**m2h's build (4ad940a8, the two-cycle main lookup): -2.003 ns, 53.5 K failing endpoints** (the D$'s
own 6.5 K -> 4.6 K; shell, frontend and SQ-internal +1 K each, the draw's). Its D$ leaders after
the store lookup's split: `fb_way -> arr[].u_a` (937, one level: the arrays' write data, the beat's
chunks or the head's, muxed in front of 16 block RAMs a parity) and `ms_v -> ms_pl`, `-> mbd_e/o`
(543: the free MSHR's priority encoder in front of the allocation's writes). The write data is now
a register taken a cycle ahead like the write row (the beat's chunks as it arrives, else the head's
next data), and the free MSHR is C1's, registered: nothing allocates or frees between C1 and C2
(asserted).

**THE HPM COUNTERS ADD A CYCLE LATE** (m2g: `hpm_ev_q`/`hpm_esel -> mhpmcounter`, 810 endpoints,
17 levels: the 50-way event select and the 64-bit add in one cycle). Each counter's increment is
registered (`hpm_iq`) and added the next cycle; the cycle after an mhpmeventN write gives 0, so a
new event never counts its predecessor's last cycle.

**THE LSU'S PORT, FROM ITS QUEUES** (m2h traces: `u_sq/rob -> ... -> u_lsu/o_boff`, 19 levels, -2.05;
`u_lq/v -> ... -> u_lsu/o_nb`, 14 levels, -2.07). Both queues pick and the LSU starts in one cycle.
The SQ's half: a store drains from the cycle after its commit (`cmt`, a register), not in the
commit's own cycle -- the commit is a compare against the ROB's irrevocable pointer, and it led
the 19 levels. A store the LSU waits for costs a cycle. The LQ's half: the access candidate is
picked a cycle ahead (the 32-entry age pick into a register, `xi`) and offered while that entry is
still ready, so the start reads one entry's ready bit and fields at a registered index. A pick
taken last cycle is sent, so not ready: back to back, a load starts every other cycle -- the D$'s
rate now. The directed bench's take waits for the offer, and its release checks step once.

**EVERY CACHE READ WAITS A CYCLE AT THE DOOR** (m2h: `u_lsu/mem_raddr -> u_dcache/arr[].u_a`, 14
levels, -2.01: the SoC's device decode of the LSU's live read address, three CARRY8 compares, then
the D$'s issue and its arrays' address). The door's registered copy is the cache's only source:
the D$ is asked from registers, a cycle later. The LSU already starts a read only while none
waits, and the D$ takes a lookup every other cycle. It is the dearest cut so far: the 300 M
lockstep retires 186.8 M at 4e25f2a4 against 201.0 M at 1d81b2d7 (-7.1%) -- the first to revisit
once the core closes (the decode alone could be registered with the LSU's read address).

**AN AMO'S WRITE DATA IS A REGISTER** (m2h: `u_lsu/mem_ren -> u_dcache/sf_wd`, 11 levels, -2.03: the
SoC's held read data into the AMO's add or compare and on into the D$'s store queue). The value
an AMO writes is computed at its read (S_ARD, where its old value is already taken) and written
from that register in S_AWR.

**M'S TRAP REACHES ONLY WHAT IT WRITES** (m2h: `u_lsu/fw_n -> redirect_target_q`, 16 levels, -1.94,
and `-> u_csr/mhpmevent` CE: M's trap decision, which waits on a landing's port yield, ran through
the CSR file's trap cause, delegation and target adder, and gated every CSR's write enable). An
injected trap's target is computed from its own cause (an exception to stvec when delegated from
below M, else mtvec; asserted equal to the general target), so the decision only selects it. A
CSR write yields only to its own op's trap: an injected trap never meets a system op's update
(asserted), so the CSRs a trap does not write lose it from their enables.

**A LANE'S PAYLOAD IS READ IN R** (m2h: `lb_n -> ln[].r_pl`, 392 endpoints, -1.93: the pick -- the
landing buffer's AGU-only gate, the select tree -- on the payload LUTRAM's address). R reads it at
the registered entry (`r_ent`); it feeds only E1's register. The entry frees a cycle after its
pick and dispatch writes a slot reserved a cycle ahead, so it is not rewritten before R (asserted).

**M'S PICK IS MADE A CYCLE AHEAD** (m2h: `u_lq/v -> m_addr`, 15 levels, -2.04: each queue's age
pick of a waiting address, the two picks' sequence compare, the address read, into M). Both
queues register the pick (`pk_q`) and offer it while that entry's address still waits; the
compare and the reads take registered indices. Back to back, M takes a waiting address every
other cycle; an address arriving with none waiting still goes straight in from its lane.

**THE RETIRE-SIDE CALL/RETURN CLASS IS WRITTEN A CYCLE AFTER DISPATCH** (m2h: `u_rob/tail -> rcls`,
-1.88: four writers' tail-relative decode over every ROB entry's flops). The writes are registered
and land a cycle later; no op commits the cycle after its dispatch (asserted).

**m2i's build (5366ac84): -1.841 ns, 49.0 K failing endpoints** (m2h -2.003, 53.5 K). Its new D$
family was the registered write data's own source: the head's next data (`st_wd_n`) carries the
store port's take and the LSU's live store data (`u_lsu/rfast_q -> u_dcache/wd_e`, 13-15 levels)
and the head's completion (`bhitv -> wd_e`). The data SB2 writes is its own slot's, so the
register now reads `sf_wd` at SB1's slot (asserted equal to the head's data whenever it writes).

**A SYSTEM OP'S SOURCE CARRIES ITS ZERO TEST** (m2i: `sy_src -> u_rob/v`, `-> u_rename/rnewer`,
`-> u_rob/head`, 16-19 levels: `csr_writes` tested the 64-bit source for zero, and that ran
through the read-only-CSR illegal check, the op's own trap and the ROB's commit kill). The SYSQ
takes `sy_src_nz` with `sy_src`; csr_file reads it as `upd_src_nz` (asserted equal to the value's).

**EACH CSR WRITE SETS OR CLEARS ITS OWN REGISTER** (m2i: `sy_addr -> u_csr/mhpmevent`, 7-10 levels:
the CSR read mux, indexed by the op's address, in front of every CSR's write data). A write is
`rmw` of the CSR's own stored value (or its unpacked view); every write mask lies inside the bits
where the read view is the stored value, and MIP/SIP's base was already the raw register, so the
values are the same. The counters' and events' load value (`cw_d`) sets or clears its own counter
or event.

**A CSR OP'S RESULT IS ITS CSR READ A CYCLE EARLIER** (m2i: `sy_addr -> ln[].qval`, 9-12 levels: the
read mux into lane A's write register). The read is registered (`csr_rdq`) and a CSR op fires in
its second cycle at the head, before its own write; an instret read in its third, minstret
having caught up in the second. The cosim's captures take the same register.


**m2j's build (60eb3235): -1.613 ns, 49.2 K failing endpoints** (m2i -1.841, 49.0 K; TNS -28.0 K vs
-29.0 K). Fan-out families in it: the flush's flops (`red_m_q -> ln[].hx_val`, `-> u_mul` DSP
enables, 1-2 levels, -1.6) and the scheduler's free slot onto the payload LUTRAMs' write address
(`fs_ix -> plmem`, 0 levels, -1.33). The flush's flops (red_m_q, red_s_q, fr_v, fr_hd_q) and fs_ix
are replicated (max_fanout).

**THE SYSTEM OP'S REDIRECT IS csr_file's AS IF IT FIRES** (m2j: `lb_n -> u_rob/v`, `-> u_rename/rnewer`,
`-> redirect_target_q`, 15-18 levels: the fire -- lane A's slot, the landing buffer -- ran into the
file's trap and redirect decode as upd_valid, then on into the ROB's commit kill). The file drives
redir_valid, redir_is_trap and the target from the op's registered fields alone (asserted equal to
the gated terms when it fires); the core ANDs its fire last, with upd_valid's own qualifiers (not a
dispatch trap, a fence or a replay: a fence.i's immediate decodes as ecall).

**THE D$'s QUEUED CBO/NC STORE IS A COUNT** (m2j: `u_dcache/sq_hp -> sf_pa` write enables, 224
endpoints, -1.33: the store port's take scanned the queue's slots for a CBO or NC store held for
its completion). `cn_n` counts them as they are taken and leave (asserted equal to the scan).

**AN AMO COMPUTES ITS WRITE A CYCLE AFTER ITS READ** (m2j: `u_lsu/rfast_q -> u_lsu/amo_wd_q`, 11
levels, -1.63: the SoC's held-data select in front of the AMO's adder). S_ARD registers the word
as read (`amo_mem_q`); a new state S_ACP computes the write value from it; S_AWR writes.

**M'S TRANSLATE-ONLY PASS TAKES TWO CYCLES** (m2j: `m_addr -> u_sq/fc`, `-> m_addr` CE, 10-11
levels: the dTLB's LUTRAM read and tag compare in front of the queues' fill fields, the fast
load's start and M's completion). A plain load or store spends two cycles in M: the first reads
and compares the dTLB through its side lookup (`s_*`, which walks nothing), registered; the
second fills the queue entry, starts the fast load and completes M from that register (`xp`).
Between the two the dTLB can only gain an entry or replace one: a stale hit is still the page's
translation, a stale miss hands the entry over untranslated as any miss does. M's own FSM ops
(AMO, LR/SC, CBO) keep the walking lookup.

**m2k's build (ea34957f): -1.516 ns, 42.5 K failing endpoints, TNS -20.6 K** (m2j -1.613, 49.2 K,
-28.0 K). Lockstep: 459ee964's two-cycle translate costs nothing measurable (186.5 M vs 186.2 M):
loads and stores already go every other cycle at the queues' picks and the D$.

**THE SYSTEM OP'S FIRE IS A REGISTER** (m2k: `sy_at_head -> u_csr/mhpmevent`, `-> scause`, `-> mepc`
..., 900 endpoints, 7-10 levels, and `-> redirect_target_q`, 13-18: the fire -- lane A's slot, the
landing buffer -- as upd_valid into every CSR's enable, the trap and the commit kill). `sy_fire_q`
is taken a cycle ahead from the head flag's next state, the head-cycle flags, lane 0's E1 (its
E2 next cycle) and the landing buffer's next count. Every system op fires in its second cycle at
the head at the earliest (an instret read in its third). A multiply in E1 counts busy even if it
is about to be killed, so the register may fire a cycle late, never early: both are asserted
(it fires only where the live condition holds, and lags it by at most a cycle).

**THE D$'s STORE TAKE READS A REGISTERED DEVICE DECODE** (m2k: `u_lsu/pa_q -> u_dcache/sf_pa`,
`-> sf_wd`, 451 endpoints, 8-9 levels: the SoC's device decode of the state machine's write base in
front of the store port's take). The SoC registers the decode (`is_dev_w_q`); the LSU's state
machine never writes in the cycle its base is loaded (`pa_new`), so the registered decode is the
base's (asserted). A state-machine store (device, NC, straddling, CBO) costs a cycle. And the
lane's R entry (`r_ent`, now the payload LUTRAMs' read address: `-> ln[].pl`, one level) is
replicated.

**A RELEASE CLEARS THE WEAK-BRANCH BITS A CYCLE LATE** (m2k: `u_rob/kh -> wk_e`, 220 endpoints, 11
levels: the ROB's per-entry kill vector into the walk hold's bit per entry). The vector and the
release are registered and the clear comes first in the block, so an entry dispatched again
meanwhile keeps its bit; the walk hold lingers a cycle.

**M TAKES ONE LIVE MEMORY COMPLETION** (m2k: `wk_lk_va -> m_addr`, `-> m_st_data`, `-> m_pc`, 12
levels: the walker's VA through the dTLB's walking lookup and the LSU's done into M's advance and
its next op's load). Only a plain load's or store's completion is live -- its translate pass, a
register's (asserted equal to the LSU's done whenever M holds one). An access M starts (AMO, LR/SC,
CBO) completes from the latch a cycle after the LSU's done, its request already withdrawn, and
writes its register from the latched value, as a fault's pulse already worked.

**m2l's build (459ee964): -1.672 ns, 42.2 K failing endpoints, TNS -19.1 K** (m2k -1.516, 42.5 K,
-20.6 K: the WNS is the draw's -- instrumentation now leads it, the top-down leaves `tdr_*` and the
event bus, 35 endpoints). Lockstep: 0b47eae3 retires 185.3 M (-0.6%: the state machine's stores
wait out their base's cycle).

**M's OWN ACCESS STARTS FROM THE REGISTERED WALKING LOOKUP** (m2l: `m_addr -> u_lsu/o_nb`,
`-> o_boff`, `-> o_kill`, 17 levels: the dTLB's walking lookup in front of the state machine's
start for an AMO, LR/SC or CBO). The lookup's answer is registered; `xl_fp` (M held the port the
cycle before too) makes it M's own, at the same address (asserted: two such accesses always have a
cycle without a request between them, the latched completion withdrawing it). A cycle each.

**THE LQ's FAULT REPORT STAYS A LIVE PICK** (m2l: `u_lq/av -> qfr_pc`, 10 levels). Registering the
age pick (c4c507bf) broke the 0c1bd7c3 lockstep at 19.5 M retires ("a store committed in a
redirect cycle"): the ROB's replay hold names the oldest flagged load from this report, registered,
and covers only the one cycle a fresh flag is not yet named (rp_new_q); the queue-fault trap reads
the same report at the head. Reverted; the family needs the hold and the trap reworked with it.

**A CSR WRITE YIELDS ONLY TO ITS OWN OP'S EXCEPTION** (m2l: `u_csr/mip -> mie`, `-> scounteren`,
`-> mnstatus`, 9-10 levels: the pending interrupts, through the interrupt op's take, in every CSR's
write enable). The interrupt pseudo-op is neither a CSR op nor an xret (asserted), so the write
gates take `sysop_exc` alone.

**A FAST LANDING'S FORMAT IS LOOKED UP A CYCLE AHEAD** (m2l: `u_dcache/rd_resp_tag -> wb_ld_q`, 9
levels: the response's tag reading the per-tag size, offset, sign and FP bits in front of the
alignment). As its kill already was: at each tag the next landing can carry (the D$'s two
pre-answer tags, the forward ring's next; a forward starting now takes the values it writes),
registered, the source picking one (asserted equal to the live read). Lockstep: 147ee68a retires
184.7 M (-0.4%: an access M starts completes a cycle later).

**THE SQ's SNOOP MARKS, THE DATA READ LANDS** (m2l: `lb_n`/`u_rename/newer -> u_sq/rd`, `-> dv`, 15-16
levels: the writeback bus, compared against every entry's data register, then a ring-oldest pick of
the one match whose bytes land from the bus). Every match now sets `rd` -- the value is in a
register file -- and the data read lands it a cycle or two on; no pick stands between the bus and
`rd`. (Delaying the bus instead was rejected: a register freed and reallocated around a flush
could match a stale writeback, and delaying the SQ's allocation would hide a just-dispatched store
from a younger load's alias test.) The directed bench's register-file model now takes the
writebacks, as the core's does; its data-path checks wait for the read.

**m2m's build (cbff8942): -1.413 ns, 42.6 K failing endpoints, TNS -21.4 K** -- the best yet. Lockstep:
cbff8942 retires 184.3 M. Its leader, `ln[].stg_ps -> ln[].u_iq/dep` (970 endpoints, 3 levels), is the
staged op's tags against every entry's for the dependency rows: stg_ps is replicated. The walk
hold's weak-branch bits are set a cycle after dispatch from registers too (`fe/d_valid -> wk_e`,
`redirect_q -> wk_e`, 4 levels over 256 bits): no weak conditional resolves the cycle after it
dispatches (the resolution's check).

**THE PAGE-CROSSING AND AMO-ALIGNMENT CHECKS ARE REGISTERED** (m2m: `m_addr -> u_sq/fc`, `-> unc`,
`-> flt`, `-> addr_lv`, 12 levels: the 13-bit add and compare on M's address in front of the fill's
fault fields and M's own access's start). Every user is a second cycle at the same address (the
translate pass's `xp`, M's own access's `xl_fp`), so both take the cycle before's value; a
misaligned AMO's fault comes in that second cycle, and the start excludes it there. Lockstep:
1e7af192 (the fault pick reverted) retires 184.3 M, clean past the 19.5 M failure.

**THE SQ OFFERS ITS HAND-OFF A CYCLE AFTER IT IS READY** (m2m: `u_sq/dv -> u_lsu/o_nb`, `-> o_boff`,
`-> fd_s`, 13 levels: the hand-off entry's bits, read at the pointer, into the LSU's start). `c_v`
is a register: an entry ready and not taken stays ready (a committed entry's bits clear only once
it is freed past its hand-off, and none is outstanding until the next take); asserted against the
live condition. The directed bench's drain waits for the offer.

**m2n's build (1e7af192): -1.675 ns, 45.0 K, TNS -23.0 K** (the draw's: m2m -1.413). The tail past
-1 ns keeps shrinking (m2g 12.1 K, m2m 3.8 K, m2n 5.3 K) while about 33 K endpoints sit between 0 and
-0.75 ns, a sixth of them in the shell (which closed at this clock before the core moved onto it).

**THE D$ ARRAYS' READ ADDRESS IS A REGISTER** (m2n: `m_is_mem -> u_dcache/arr[].u_a`, `u_dcache/sq_hp
-> arr`, 6-8 levels, -1.6: the issue's arbitration on the block RAMs' address pins). The issue's row
is registered (`a_row_q`) and read at C1's edge; the arrays drop their output register (latency 1),
so the chunks still arrive in C2. The read-out, which shares the read port, is decided in C2 and
pends a cycle (`ro_p`, the lookup yielding to it too), so the lookup issued with the decision reads
first. A store written at C1's edge to the load's line is queued in C1, so the load waits behind it
(`sq_ln`): the row-written-as-read check (w1) is unreachable and now an assertion. Lockstep:
6ea83abc 184.3 M, 30483569 184.1 M.

**THE FIRE STAYS OUT OF csr_file's DECODE** (m2n: `sy_fire_q -> u_csr/stimecmp`, `-> u_rob/v`, 8-10
levels, the fire entering at the first LUT). `csr_illegal` was `upd_valid & ...`, and the exception,
the as-if-fired redirect and the write gates were all built from it. `csr_ill_f` is the illegal
access as if the op fires; every as-if-fired term and every write gate takes it (or `exc_active`,
now as-if-fired too), so upd_valid is the last AND.

**THE SECOND LEVEL'S REFILL IGNORES THE LIVE REQUEST** (m2n: `wk_lk_va -> u_lsu/u_mmu/tlb_perm`, 216
endpoints, 12 levels: the request's VA compared with the walk's, `req_match`, in front of every
first-level TLB write). A second-level hit is the page's translation for the walk's own VA, its
context unchanged (`ctx_poison`), so it is installed whatever the request is now; `req_match`
still decides whether the hit answers the request.

**THE LQ RESERVES EACH BANK'S FREE ROW A CYCLE AHEAD** (m2n: `u_lq/v -> u_lq/om`, `-> u_spl_ld` write
address, 6-8 levels: each bank's priority scan of the valid bits, the banks' ranking and the writers'
entries in one cycle). `bfr`/`bhas`/`bfc` are registers: the lowest row free now other than the one
this cycle allocates in the bank, and the count with it, so the dispatch credit (which reads the
count) never promises a row the reservation lacks; a row freed now is counted from the next cycle.
The count is the old one less the previous cycle's frees, so the credit stays conservative. (The
1c003570 lockstep stopped on the bench's liveness check, which assumed the credit a cycle behind
the valid bits; it is two now, and out of reset the count was zero for a cycle: both fixed.)

**m2o's build (30483569): -1.543 ns, 45.3 K, TNS -20.4 K.** The tail past -1 ns is 2.6 K (m2n 5.3 K) while
the middle grows: 37 K endpoints between 0 and -0.75 ns. 9.4 K of those have 0-2 logic levels -- fan-
out, not depth: the DDR IP's calDone across the memory controller (1.8 K), core_reset (0.9 K), the
PRF write addresses and valids (1.2 K), the phase flag into the I$ memory, redirect_q and fr_hd_q, the
scheduler's free-slot valid, the arbiter's address into the line memory. The shell takes calibration's
done into a register of its own (cal_q, replicated; static once set); the rest replicate harder.

**m2p's build (e2977334): -1.719 ns (the draw's), 37.7 K failing endpoints, TNS -15.8 K** (m2o 45.3 K,
-20.4 K): the middle shrinks for the first time (32.5 K between 0 and -0.75 ns, from 37 K). Lockstep:
e95b83f5 retires 184.0 M; every commit through it is lockstep-clean.

**THE SQ TAKES A LOAD'S ROW A CYCLE AHEAD** (m2p: `m_addr -> u_sq/cov`, 336 endpoints, 5-6 levels: the
filling load's address against every store's, into the coverage and overlap matrices). A load fills
in the second cycle of M's translate pass at the first cycle's address, and no store's address
arrives in that first cycle (M holds the load), so the row computed then is the fill's (asserted:
the inputs unchanged, no store arrival). The directed and random benches present a load's address a
cycle before its fill, as M does.

**m2q (e95b83f5): -1.669 ns, 39.0 K, TNS -18.1 K; m2r (7a01e51b): -1.606 ns, 38.3 K, TNS -15.7 K**, the tail
past -1 ns at 1.4 K. Lockstep: 7a01e51b retires 184.0 M.

**M FILLS WHATEVER THE KILL SAYS NOW** (m2r: `m_rob_idx -> u_sq/va`, `-> u_sq/fc`, 241 endpoints: the
kill's compare on M's op in front of every queue field the fill writes). An op killed in an earlier
cycle has left M already; the queues apply this cycle's kill to their own entries after every other
arm, the fill included (asserted: a dead op's fill meets its queue's kill).

**M's PREFERENCE BETWEEN THE QUEUES' PICKS IS A REGISTER** (m2r: `u_sq/v -> m_addr`, `-> m_st_data`,
`-> m_unit_res_q`, 9 levels: the two picks' sequence compare in front of M's take). Taking the older
of the two is a preference -- either fill is correct, the queues order themselves -- so it is the
previous cycle's compare.

**m2s's build (19ccf540): -1.493 ns, 36.8 K failing endpoints, TNS -13.6 K** -- the best on all three; the
tail past -1 ns is 965 endpoints (m2g: 12.1 K). Lockstep: 19ccf540 retires 183.3 M; every commit on the
branch is lockstep-clean. Its leaders are spread: the frontend's fetch FSM into the ring, the MMU's
own walker, the lanes' E1 ALU (`ln[].rs1`, `ln[].pl -> u_x/u_alu`, 7-15 levels), M's trap behind the
landing port's yield, the training queue's enqueue, the ROB walk into rename.

**THE INTERRUPT OP DELIVERS WHAT WAS PENDING A CYCLE AGO** (m2s: `u_csr/mie -> u_rob/v`, 12 levels: the
pending decision -- mie, mip, mstatus, priv -- through the interrupt op's as-if-fired trap into the
commit kill). irq_v, its cause and its delegation are registered for the op's trap: a system op fires
in its second head cycle at the earliest, so the last CSR write is a cycle older than the interrupt
op's fire and the registered decision already holds it.

**THE WALKER'S SECOND-LEVEL HIT TAKES TWO STATES** (m2s: `u_lsu/u_mmu -> u_lsu/u_mmu`, 162 endpoints
at -1.35 ns, 10 levels: the second level's block-RAM word, its tag compare, the FSM's next state and the
first level's permission write). L2LK registers the hit; L2H answers and refills the first level from
it. The block-RAM word is loaded only in IDLE, so it holds through both states (asserted against the
request's VA in both).

**THE WALK'S PORTS ARE REGISTERED** (m2s: `u_rob/wl_row -> u_rename/newer`, 64 endpoints at -1.42 ns,
10 levels: the walk row's LUTRAM read, its validity against the branch's position, and rename's
per-register write decode of `newer`, `smap`, `dirty`). The ROB names a walked row's entries a cycle
after reading it; `wl_done` waits for the ports to empty, so the release still follows the last
undo. A flush or release clears the ports, as it stops the walk.

**LANE A'S PORT SELECTS THE SYSTEM OP ON A FREE-SLOT FLAG, NOT ON ITS FIRE** (m2s: `sy_fire_q ->
ln[].u_iq/e_r`, 54 endpoints at -1.36 ns, 8-10 levels: the fire through the landing slot's select
into the wake tag, the schedulers' compares and `e_r`). `sy_slot_q` registers sy_fire_q's lane terms
(E1 not reaching E2 to write, no multiply in E1, the buffer empty), so in its cycle nothing else
writes or completes on lane A's port. The tag, data and ROB-index muxes select the SYSQ's on it,
and the fire gates only the enables. The streams' straight-through landings (by0/by2) no longer
pass through `lb_drain`'s SYSQ term.

**THE STORE QUEUE'S ADDRESS PAYLOAD HAS NO RESET** (m2s: `core_reset -> u_sq/fc`, 127 endpoints at
-0.99 ns, 1 level: the reset term in every payload field's enable). va, size, the translation bits
and the fault cause are read only under v and av, so their writes (the address, the walker's
answer) sit in a block of their own, enabled by their write alone.

**THE TRAINING QUEUE TAKES A WRONG-PATH CTI AND DISCARDS IT** (m2s: `lr_seq -> tq_rob`, 109 endpoints at
-1.13 ns, 9 levels: the seqno compare against the pending restart in front of the enqueue's ranks,
the tail adder and the write decode). The compare was redundant: the head discards an entry past
the pending restart, the kill marks the rest while it holds, a redirect's cycle enqueues dead, and
nothing dead outlives the release (asserted at the release for every queued entry). A wrong-path CTI
now holds a queue slot until the head discards it.

**M'S TRAP DOES NOT WAIT FOR THE LANDING PORT** (m2s: `u_lsu/fw_n -> u_csr/mcause`, `mtval`, `scause`,
`stval`, ~230 endpoints at -1.2 ns, 9 levels: the forward injection's port yield in M's done, into
the CSR file's trap write). M's only redirect is a data fault's trap. It writes no register and
completes on no port (it never commits), so `m_trap_go = m_valid & xtrap_v & ~head_block` is three
flops, and it feeds the CSR file, the commit kill and the redirect. `m_done` exempts a trapping op
from the yield to match, and `m_done_red` is gone.

**THE INTEGRITY LOG REGISTERS ITS INPUTS ON ARRIVAL** (m2s: `fe/fe_err_q -> u_errlog/first_cyc`, 48
endpoints at -1.11 ns, 4 levels: a net across the die into the 64-input reduce and the record's
enables). `err_q` takes the bits as they arrive, `any_q`/`low_q` the reduce and the priority encode,
and the record stamps `cyc - 2`.

**THE COUNTER CSRS DECODE FROM THEIR BLOCK AND OFFSET** (m2s: `sy_addr -> csr_rdq`, 40 endpoints at
-1.15 ns, 6-8 levels: three 12-bit range compares and three 12-bit subtractions selected into the
counter index, in front of the 16:1 counter mux and the read case). The three ranges are each
N = 3..31 at offset N of a 32-aligned block, so the block is `raddr[11:5]` and the index is
`raddr[4:0] - 3` for all three (an elaboration check pins the bases).

**THE LOAD QUEUE'S AGE MATRIX TAKES AN ALLOCATION A CYCLE LATE** (m2s: `fe/d_valid -> u_lq/om`, 44
endpoints at -0.99 ns; m2n's `u_lq/v -> u_lq/om`: dispatch's decision and the live vector into the
matrix's enables and rows). The writers, their entries and rows (`v | wahead`) are registered
(`oa_q`) and land a cycle later. A new entry is in no pick in its first cycle, because every pick
needs its address and the fill comes a cycle after the allocation at the earliest (asserted: a
pending entry has no address, replay or send). A later allocation's column clear lands after the
row, in arrival order. The one-oldest check leaves the pending entries out.

**THE LOCAL SRAM'S REGION IS AN EQUALITY** (m2t: `u_arb/mq_addr -> u_arb/mq_wdata`, 369 endpoints at
-0.97 ns, 6 levels: two 64-bit magnitude compares, three carry chains, into `m_q_ready`, the arbiter's
take and its 512-bit request register's enable). The SRAM is aligned to its size (an elaboration
check pins it), so the region is the line address's bits above its index.

**THE DECODE QUEUE'S READ POINTER IS REPLICATED** (m2t: `fe/q_rp -> fe/hr`, 201 endpoints at -1.01 ns,
4 levels: the pointer's net to every bank's LUTRAM read address). `q_rp` has `max_fanout = 16`.

**A LOAD'S OLDER STORES ARE A MATRIX ROW TAKEN AT ITS FILL** (m2t: `u_sq/headc -> u_lq/cnt`, 14 levels:
the per-load tag subtraction and per-store compare (`oldm`, 16 x 32 live every cycle) into the
violation, the load queue's free and its popcount; the same compare heads the block, forward and
unknown-address cones). The stores older than a load when its address arrives stay older until
they leave, and a store allocated later is younger, so `lold[load]` takes one row of compares at
the fill (`oldm_f`). A store's allocation clears its column, and `oldm = v & lold`. The compare
form stays as the oracle, asserted equal while the load's address is in. `l_older` (a performance
event and its own check) is exact from the fill and `l_dist != 0` before it.

**THE STORE QUEUE'S KILL MARKS ARE REGISTERED** (m2t: `u_rob/kh -> u_sq/v`, 42 endpoints at -1.13 ns,
10 levels: each entry's position compare against the kill, two carry chains, into the clears, the
data read's cancel and the tail's step back). The kill holds for cycles, and nothing allocates while
it does, so `kds_q` takes the compares and the live `kd_v` ends them with the kill. The queue applies
the kill from its second cycle, and the check that a dead op's fill meets the kill looks a cycle
later. The directed bench holds its kill for two cycles, as the ROB's does.

**THE WHOLE-LINE FLAG IS REGISTERED WITH THE REQUEST** (m2u: `u_arb/mq_wmask -> u_arb/mq_wdata`, 518
endpoints at -1.29 ns, 8 levels: the L2's `&u_q_wmask` in its ready, into the arbiter's take of the
512-bit request). The arbiter registers `mq_full = &g_wmask` with the mask, and the L2 takes it as
`u_q_full` (asserted equal to its mask's AND). The L2 bench drives it from its own mask.

**THE DIVIDER'S OPERANDS LOAD WITHOUT THE ABORT** (m2u: `u_rob/kh -> u_div/da`, `db`, 128 endpoints
at -1.33 ns, 9 levels: the kill's position compare on the divide in their enables). The hand-over
loads them whatever the abort says: the start (`dst`) keeps `~abort`, and the abort arm drops the
waiting start.

**THE CSR FILE'S DECISION IS OUT OF THE COMMIT KILL** (m2u: `sy_addr -> u_rob/v`, `u_rename/rnewer`,
`cp[].rmap`, `u_rob/head`, about 290 endpoints at -1.27 ns, 12 levels: the CSR decode's trap decision
through `sy_trap` into `rc_tkill`, the commit ports, the ROB's valid bits and rename's retire maps).
A CSR or system op is completed by its fire alone (`sy_done`), so in its fire cycle the head is not
done and nothing commits (asserted). The kill keeps only the SYSQ's flops for an entry that may be
done (a queue fault `sy_xt`, a replay `sy_rp`), and `red_trap_q` holds the cycle after.
## 4. Verification, every increment

- `src/lint.sh` clean, `core/run-vl.sh` 240/0, the unit benches.
- The 300 M Linux lockstep: retire count equal to main's when the increment is cycle-identical.
- Out of context, `make ooc MODULE=<the changed module>` at 6.000 ns (`clk2x` at 3.000 ns) on main
  and on the branch: WNS and LUTs side by side.
- Then the full build and the board gate when an increment is worth landing.
