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
being measured as a knob: `exp/red1` (a lane's mispredict restarts the frontend one cycle late;
the freeze still sets at once) against main, on the 77 GB5 SimPoints (`m390b-sp`, `red1-sp`).

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

## 4. Verification, every increment

- `src/lint.sh` clean, `core/run-vl.sh` 240/0, the unit benches.
- The 300 M Linux lockstep: retire count equal to main's when the increment is cycle-identical.
- Out of context, `make ooc MODULE=<the changed module>` at 6.000 ns (`clk2x` at 3.000 ns) on main
  and on the branch: WNS and LUTs side by side.
- Then the full build and the board gate when an increment is worth landing.
