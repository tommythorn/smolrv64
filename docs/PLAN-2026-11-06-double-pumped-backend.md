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

## 4. Verification, every increment

- `src/lint.sh` clean, `core/run-vl.sh` 240/0, the unit benches.
- The 300 M Linux lockstep: retire count equal to main's when the increment is cycle-identical.
- Out of context, `make ooc MODULE=<the changed module>` at 6.000 ns (`clk2x` at 3.000 ns) on main
  and on the branch: WNS and LUTs side by side.
- Then the full build and the board gate when an increment is worth landing.
