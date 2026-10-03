# Four uniform lanes and one elastic front end

Status: direction decided (Tommy, 2026-10-01). Every step below is measured or modelled before it
is built. This plan supersedes parts of the memory-backend program and the D$ plan (listed below);
the rest of them stands.

## Why

The core sits in an IPC trough whose causes the traces and the counters show directly:

- **The front end is blocked by structural limits at dispatch.** One load/store and one
  FP/control-flow op per cycle, the slot-pairing rules, and serialising ops waiting at dispatch for
  the machine to drain. Each such limit stops every younger instruction, whatever its class.
- **Ready ALU work waits for two ALUs.** Address generation, branches, multiplies and FP all queue
  behind narrow special ports while ALUs are cheap; what costs is PRF write ports.
- **Supply.** A 16-byte pair ends at its first known control transfer, taken or not: in the 300 M
  tiny128 lockstep 12.3 M of 77.6 M pairs (16%) end at a not-taken branch. `fe:icache` is 22-25% of
  cycles.
- **Back-pressure in the backend** (300 M, tiny128): ROB full 5.9%, LQ full 4.9%, serialise 3.8%,
  SQ full 2.5%, a scheduler full 0.4% of cycles.

## The end state

1. **Front end.** Predict -> fetch -> align -> decode -> **one elastic queue of decoded
   instructions** -> rename -> dispatch. Fetch is limited only by bytes from the I$ and by
   predicted-taken control transfers. Decoding before the queue (RVC expanded there too) uses the
   bytes the first time they arrive: an unpredicted direct `jal` is seen at decode, cuts its fetch
   parcel and redirects fetch to its target at once; an unpredicted `jalr` cuts the parcel and
   pauses fetch until it resolves, even while the back end is stalled and the queue is full.
   ("Unpredicted" is neither the BTB nor the RAS naming it.) A conditional the BTB does not name
   gets no guess: never-taken branches are kept out of the BTB by design, so a miss cannot tell
   an unknown branch from a known never-taken one. Fetch, align and decode stall together when
   the queue has no room; **nothing after the queue can stall:** rename and dispatch advance
   every cycle through registers that never hold, with no buffer until the execution lanes
   (today's bundle register, decoupling queue, IR survivors and dispatch-stage register go).
   "Stalling the front end" means the queue feeds rename bubbles. It does so on registered credits: the ROB, the LQ, the SQ, each
   lane's scheduler and every free list, each counted against everything already between the
   queue and dispatch, so any op past the queue is guaranteed its room.
2. **Predictor.** Path history: a shifted hash of taken-branch addresses, once per block. A block ends
   at its first predicted-taken control transfer; not-taken branches inside it do not cut it. Every
   branch in a block is predicted with the history from before the block; training uses the carried
   index.
3. **Four uniform lanes.** Bundle slot k is lane k: there is no steering. Each lane has its own
   scheduler, two PRF read ports, one write port and its own integer PRF shard, and does ALU work,
   multiplies, branch resolution and address generation.
4. **Memory.** A lane generates a load's or store's address and wakes nothing (the lane op has no
   result of its own). One memory unit (LQ, SQ, the D$) serves all four lanes. A load's result is
   written through the port of the lane that generated its address. Stores are numbered and each load
   carries the latest store's number, as today.
5. **One write slot per lane per cycle, reserved.** Each lane keeps a short write-slot reservation
   register. Fixed-latency producers reserve at issue: an ALU op the next cycle, a multiply three
   cycles out. A load hit reserves at the D$ tag compare, one cycle before its data, and wakes its
   dependents then, so they issue back to back with the data. A miss, a divide and an FP result bound
   for an integer register announce themselves a cycle ahead and take the next free slot. Select never
   issues into a reserved slot, so a schedule once made never breaks.
6. **FP.** Its own register file, renamed in four static slices (one free list per slot, a partition
   of the FP registers); rename picks the free list by the destination's class. One FP unit with its
   own scheduler serves all four slots. Its register file gives the FP unit three read ports, so a
   three-operand op issues whole. Crossings go through a lane: an FP result bound for an integer
   register returns through a lane's write slot; an integer operand for the FP side is read by a
   lane and handed over.
7. **Divide** is one shared iterative unit; its result returns like a miss.
8. **Branches** resolve in any lane; the oldest mispredict wins by age; predictor training is
   buffered (droppable: it is a hint).
9. **Wakeup** is same-cycle across lanes where timing allows; a delayed path costs IPC, never
   correctness, so it is decided by timing.
10. **Serialising ops** dispatch and wait in the SYSQ for the ROB head; context changes (`satp`,
    `sfence.vma`, `fence.i`) refetch through their redirect. Nothing blocks at dispatch.
11. **The ROB is sharded like the rest: 32 rows of four.** Each dispatch group takes one row,
    slot k in column k, and a short group leaves holes. Dispatch writes, completion (a lane writes
    only its own column) and retire (one row address) lose their crossbars; the ROB's credit is
    one row per group; `{row, column}` is the age. A row retires whole, one a cycle: dispatch
    allocates at most one row a cycle, so that keeps pace. 32 rows (128 slots, holes included)
    hold more than today's 32 dense entries by far, which pays only if a mispredict restarts at
    resolve (C6) rather than at the head; the two land together.

## What stays

- **D$:** the walker that 5b moved to the queues. The D$'s end state is
  `PLAN-2026-10-03-dcache-vipt-l2.md`: an alias-free VIPT L1 (32 ways of 4 KiB) whose dTLB is read
  in the access cycle, and a 1.5 MiB L2.
- **Memory-backend program:** loads past unknown-address stores with replay and a wait predictor
  (C4b step 6), the MSHR window (C5), mid-window recovery (C6), the wrong-path throttle (C9).
- **The wrong-path holds:** no iTLB walk and no I$ line read past a weakly predicted conditional.
- **Future TLB:** the big miss-path TLB, the 2 MiB table, filling a PTE line's eight entries per walk,
  BRAM against LUTRAM.
- **Sscofpmf**, and the **pipe trace** (one code per instruction per cycle, the pair log).
- **The clock review** (250 MHz).

## What it supersedes

- The LSA-pipe end state (2026-09-24) and the memory-backend program's C4b steps 4 and 5 (an
  out-of-order memory scheduler, M deleted): address generation moves into the lanes, and M goes with
  it.
- The per-class dispatch FIFOs discussed on 2026-10-01: uniform lanes have no class mix to absorb.
- Today's dispatch rules and ports: one load/store and one FP/control-flow op per cycle, the slot
  pairing, the swizzle, the shared F/CTF/MD/SYS port, the dead third-ALU scheduler.

## Order

0. **Close out:** commit the last 5b timing fix, push on Tommy's go, GB5 on the new tip.
1. **Sscofpmf** (`perf record -g` on the board).
2. **Measurements and models, no RTL:**
   - the share of dispatch stalls caused by structural rules, and the cycles a ready ALU op waits
     for an ALU;
   - a trace-driven predictor model: path history against today's direction history, the pairs
     a not-taken branch cuts short, and the transfers the BTB has never seen, by kind (what a
     resteer at decode recovers);
   - the core's limit study with four lanes;
   - the PRF's technology, out of context (`make ooc`): block RAM with a register-read stage and
     its bypass against LUTRAM. Today's LUTRAM PRF is 13,761 LUTs (3,996 LUTRAM, 9,765 of
     select and bypass) for 10 read ports; in block RAM a shard with one write and eight reads
     is eight RAMB36 (512 x 72 each), 36 for four integer shards and the FP file, its LUTs
     only the shard select, and 512 registers a shard for nothing. The cost is a stage after
     issue (a cycle more on a mispredict, which restart at resolve absorbs) and two stages of
     results in each lane's bypass; wakeup and back-to-back issue do not change;
   - the survey's three findings: the memory scheduler selects at most every other cycle, the LSU
     starts D$ reads at most every other cycle, the schedulers pick the lowest-numbered ready entry.
   Measured so far (the 60 M tiny128 lockstep's `DISP-SIM` lines, tools/trace-limit.py and
   tools/bp-model.py, `make util`):
   - the class rule holds a ready slot B behind a dispatching slot A in 8.20 M cycles (13.7%) and
     slot C in 5.03 M more (8.4%); room (scheduler, ROB, queues) holds slot B in only 2.08 M;
   - `u_iq_l` never issues in back-to-back cycles (0 of 15.1 M issues), nor does the LSU start
     two accesses in a row (0 of 2.76 M), and its head is ready and not issued in 13.9 M cycles
     (23%);
   - ready ALU work left waiting: 3.42 M op-cycles, 1.59 M of them while the other ALU had
     nothing ready (a steering imbalance fixed lanes do not cure either); issue passed an older
     ready entry 1.17 M times;
   - the limit model (GB5 board traces, perfect caches, 32 entries): today's rules 1.31-1.92 on
     the integer workloads against 1.89-2.05 for four lanes, camera 0.41 against 1.38;
   - path history: 2.55 direction MPKI against 2.71 (16 bits, shift 2); the BTB-unknown taken
     transfers, 3.06 MPKI on the boot, are 1.25 conditional, 0.82 direct jump or call, 0.67
     return, 0.32 indirect; ending a pair only at a taken transfer cuts fetch pairs 17%;
   - kernel system ops per 1000 instructions: 0.31 CSR writes (mostly `sstatus.SIE`), 0.36 AMOs,
     0.05 `sfence.vma`, 0.03 `fence.i`.
   - the PRF at the lanes geometry (four 64-entry shards, eight 64-bit read ports), alone with
     `make ooc`: block RAM with a registered read and its bypass is 2,152 LUTs and 32 RAMB36
     (6.7% of the part), its read stage 336.6 MHz; LUTRAM is 5,032 LUTs (2,368 of them LUTRAM).
     Block RAM is the PRF; the same 32 RAMB36 hold 512 registers a shard.
   - the memory pipe's cadence, root-caused: `u_iq_l`'s busy counted its issue register's
     occupancy (fixed: fc351fa0, +6.37% at 300 M); the LSU starts at most every other cycle
     because its one-deep read request buffer frees only the cycle after the request, and
     knowing the accept sooner would mean a combinational D$ accept. In the 60 M lockstep the D$
     sees 7.66 M reads, none in consecutive cycles, and in 4.73 M cycles a load is ready to
     start and held only by the previous cycle's request. A two-deep buffer needs either a
     registered-select mux on the request address (whose worst path into the D$'s block RAM has
     +0.308 ns today) or a credit the D$ gives a cycle ahead with a skid of its own; both belong
     to the lanes' memory unit (step 5.2), built to take a load every cycle, not to today's LSU.
3. **Serialisation out of dispatch** (small, independent).
4. **The D$ end state:** the L2, then the VIPT L1 (`PLAN-2026-10-03-dcache-vipt-l2.md`), deferred
   until after step 5. Step 5.2 deletes M, where loads translate today; it keeps that translation
   in the memory unit until the VIPT L1 takes the dTLB into its access cycle.
5. **The lanes, in lockstep-gated steps:**
   1. the FP register file split, with sliced FP rename;
   2. uniform integer lanes at IW=3: slot = lane, an ALU and a multiplier per lane, write-slot
      reservation, address generation in the lanes feeding the memory unit, M deleted, branches in any
      lane;
   3. the rigid front end with credits: the fetch queue the only elastic stage;
   4. IW=4: four lanes, the sharded ROB at 32 rows, and branches restarting at resolve (C6).
6. **The pipe trace**, built for the pipeline the lanes leave (its stages, its credits, its block
   reasons). Until then the steps use today's trace and counters.
7. **The predictor:** path history, blocks cut at the first predicted-taken transfer.
8. Then loads past unknown-address stores (C4b step 6), C5, the future TLB, the clock review.

## Step 5.1: the FP register file (design, 2026-10-03)

**Today.** f0-f31 rename into the shards of their WRITERS: an FP load's result lives in SH_LD, an
FP op's in SH_FE, beside integer results of loads, CSR reads, mul/div, links and the FP ops with
integer destinations. So SH_LD and SH_FE each carry the 64-register floor, and every one of the
twelve read ports decodes both.

**The step.** f0-f31 get their own physical registers and nothing else maps there.

- **Number space.** The FP registers are shards SH_F0, SH_F1, SH_F2 (shard codes 5-7, free today),
  64 registers each at IW=3, inside today's 10-bit physical number. Every tag compare (the
  schedulers' wakeup, the pending table, the store queue's snoop, the ALU forwards) and
  "physical 0 is x0" keep working unchanged, because no FP register shares a number with an
  integer one.
- **Sliced rename.** An FP destination renamed in slot k allocates from slice k: one free list per
  slot, so no FP allocation counts the slots before it in the bundle. Each slice holds more than
  the 32 architectural FP registers (the deadlock floor: every f-register may map into one
  slice). At reset f0-f31 map to SH_F0, which starts with 32 free; SH_F1 and SH_F2 start empty.
  The slice is chosen by the destination's register class first, so an FP op that traps (FS off)
  allocates from the FP slices as well, never from an integer shard.
- **Two writers, two banks, one live-value bit.** The FP file has the same two writers the FP
  values have today: the load landing (FP loads) and the F stage (FP results). Each writes its
  own bank, through the write address, data and WAKE PORT it already uses: FP loads keep
  waking on the load port, FP results on the F stage's port. A one-bit-per-register live-value
  table, set by the write, says which bank a read takes. A physical register is written once
  per allocation, so the table is exact. No scheduler gains a wake port, no write waits, and
  no latency changes.
- **Reads.** The FP file is read by the F/CTF port's three operands and by M's store-data
  operand (`fsw`/`fsd`). The integer-only ports (M's base, the two ALUs) no longer decode it, and
  no longer decode FP values in SH_LD/SH_FE either.
- **The integer shards shrink.** SH_LD (integer loads, AMOs, CSR reads) and SH_FE (FP ops with
  integer destinations, mul/div, links) hold integers only: 64 each, floor 32.
- **Counters.** The FP dependency stall is a wait on SH_F0-F2; a wait on SH_FE is a mul/div, a
  link or an FP compare/convert result.
- **Gate.** Architectural behaviour is unchanged, so the cosims must pass as before; retire
  counts move only through the free lists' new sizes.

**Toward 5.2.** The lanes dissolve SH_LD and SH_FE into the lanes' shards; the FP file and its two
banks stay, the load bank written by the memory unit and the other by the FP unit.

## Step 5.2: uniform integer lanes at IW=3 (design, 2026-10-03)

Four sub-steps, each gated by the lockstep and the userspace cosims; IPC is measured before
timing is fought.

1. **5.2a, slot = lane for ALU ops.** Slot k's ALU op goes to lane k: scheduler `u_iq_i`,
   `u_iq_i2`, `u_iq_i3`, shard IE, IE2, IE3, ALU `u_xa`, `u_xb`, `u_xc`. The swizzle goes, and
   slot C's ALU op is accepted whatever A and B are. Lane C's issue register, ALU, forwards,
   write-back register and ROB port already exist; it gets its dispatch stage, source-tag and
   payload arrays, its PRF ports (`ra8/ra9`) and its shard's write. The fifth wakeup port
   (IE3) becomes live in every scheduler.
2. **5.2b, branches in any lane.** A branch, `jal` or `jalr` issues in its slot's lane, whose
   ALU resolves it and writes the link into the lane's shard. Up to three resolve in a cycle:
   the oldest mispredict wins by age into the existing early restart (`fr_*`), and predictor
   training goes through a short buffer that drops on overflow (it is a hint). The CTF stage
   leaves `u_iq_f`.
   **5.2b in detail (design for review).** Today one CTF stage (`cf_*`, fed from `u_iq_f`)
   resolves every branch, writes jal/jalr links into SH_FE when the FPU is not landing, tracks
   the oldest pending restart (`fr_v`, `fr_seq`, `fr_rob`), trains the predictor once per CTI
   (`res_*`), and marks a mispredict done only at its squash so that the ROB head stops on it.
   In the lanes:
   - **Resolve.** Each lane's `smolrv64_exec` gets its branch inputs (today tied 0). A CTI's
     link is the lane's ALU result, written at issue into the lane's shard like any ALU result:
     the link no longer waits for a port, and `cf_link_pend` and its squash interlock go.
   - **A resolve register per lane** (`lr_*`: valid, seq, ROB index, mispredict, target, taken,
     PC, predictor snapshot, call/return bits), loaded at issue. Everything downstream reads
     registers, so the exec unit's compare never reaches the restart logic in its own cycle.
   - **Oldest wins.** From the three resolve registers and the tracked restart, the oldest
     mispredict (wrap-safe seq compares, three pairwise among the lanes plus each against
     `fr_seq`) sets `fr_*` and drives `fe_red_tgt`, the RAS and history restore.
   - **ROB completion.** A correctly predicted CTI completes at issue on its lane's port, like
     an ALU op. A mispredict must not retire past its squash: it completes on the squash
     (`cf_red_fire` with `fr_rob`, as today) through one shared port, and the lane's port does
     not mark it at issue. That needs the mispredict known at issue on the lane's ROB port;
     if that is too late for timing, the alternative is a per-entry "stop" bit the ROB's
     multi-retire honours (retire stops before a stop entry, which then retires alone as the
     head, in the squash cycle).
   - **Training.** Up to three CTIs resolve per cycle; the predictor trains one per cycle
     through a short queue that drops on overflow (training is a hint), except the tracked
     restart's CTI, which takes the queue's head so rule D16 (it trains before its squash)
     holds by construction. The weak-branch count (`wk_n`) subtracts up to three per cycle.
   - **Dispatch.** Branches stop being class FC, so the class rule (which holds slot B behind
     a slot A of its class: 13.7% of cycles in the 60 M boot, `DISP-SIM`, loads and FC
     together) no longer separates two branches, or a branch and an FP op.

3. **5.2c, a multiplier per lane, and write-slot reservation.** `mul3` in every lane. Each
   lane keeps a short reservation register of its write port's future cycles: an ALU op takes
   the next cycle, a multiply the third; select never issues into a reserved slot. The divide
   stays shared and returns like a miss: it announces itself a cycle ahead and takes its
   lane's next free slot.
4. **5.2d, address generation in the lanes, M deleted.** A load or store issues in its slot's
   lane, whose ALU adds the address, and enters the memory unit (the LQ/SQ entry takes the
   VA). The memory unit translates in its own stage (the dTLB lookup M does today), takes the
   faults into the entry (as since 5b), and starts the access. A load's result returns through
   its lane's write port: it announces itself a cycle ahead and takes the next free slot. AMOs,
   LR/SC and CBOs, which run only at the ROB head, execute in the memory unit there. SH_LD
   dissolves into the lanes' shards. Until the VIPT L1 takes the dTLB into its access cycle
   (`PLAN-2026-10-03-dcache-vipt-l2.md`), the memory unit translates before the access.

What stays in `u_iq_f` after 5.2: FP arithmetic, divide, and the SYSQ's system ops. Their
integer results (FP compares and converts, CSR reads, the divide) cross into a lane's write
slot, which dissolves SH_FE.

## Decisions (Tommy, 2026-10-01)

- Four uniform execution lanes, two read ports and one write port each; ALUs are cheap, PRF write
  ports are the cost.
- Address generation in the lanes; the scheduler knows a load's or store's lane op wakes nothing.
- No steering: slot k is lane k. Each lane has its own PRF shard.
- A load's result is scheduled just in time, once the D$ knows when the data arrives.
- Each lane gets a multiplier; divide is shared.
- FP has its own register file, renamed in four static slices, served by one FP unit.
- No back-pressure in the renamer: any free list too low to guarantee registers stalls the front end.
- Same-cycle cross-lane wakeup unless timing forbids it; delaying one is an IPC cost only.
- The only elastic front-end stage is the fetch queue; fetch is limited only by byte supply and
  predicted-taken control transfers.
- Branch history is a shifted hash of fetch addresses.
- The queue holds decoded instructions and sits after decode, so an unpredicted `jal`/`jalr` is
  caught at decode while the back end is stalled.
- Rename stays after the queue. Before it, every queued op would hold a physical register for its
  time in the queue (each shard larger by the queue's depth), and rename would have to stall:
  a hold there gates the map table's, the free lists' and the checkpoints' write enables, the
  widest enables in the front end, where after the queue it never holds.
- The ROB is 32 rows of four, sharded by column like the lanes, retired a row a cycle; it grows
  together with restart at resolve (C6), which keeps a deeper ROB from lengthening the
  mispredict drain.

## The sharded ROB, measured

Holes cost density: `tools/trace-limit.py --machine lanes --cut taken --rob-rows R` (perfect caches)
gives clang 2.19 at 16 rows against 1.95 for 32 dense entries and 2.24 for 64, and camera
2.00 against 1.38 and 2.66; 24 rows beat 64 dense entries on both. With 3% of loads missing
for 36 cycles, four in flight, 16 rows give 1.71 / 1.52 against 1.48 / 1.14 dense at 32 and
1.78 / 1.76 at 64, and 24 rows 1.91 / 1.85. Retiring two rows a cycle changes nothing, with or
without misses: dispatch allocates at most one row a cycle, so retiring one keeps pace, and a
completed backlog drains a row a cycle while dispatch refills one.
With the misses, 32 rows give clang 2.08, camera 2.23 and text-compression 2.08, against
1.91, 1.85 and 1.98 at 24.

## Open

- The return path for variable-latency results (depth, bypass from it).
- Which addresses the path history hashes (taken-branch addresses or every block), from the model.
