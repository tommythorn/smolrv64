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
   5. load-hit speculation: a load's dependents woken from its D$ lookup, replayed when it
      misses (Tommy: needed for good performance; postponed from 5.2d-c).
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
   **5.2b in detail (settled as prototyped on `wip/lanes`, 2026-10-04).** Before it, one CTF stage (`cf_*`, fed from `u_iq_f`)
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
     (`cf_red_fire` with `fr_rob`) through one shared port, and the lane's port does not mark
     it at issue (`lane_mis` masks the port). If the mispredict is too late at issue for
     timing, the fallback is a per-entry "stop" bit the ROB's multi-retire honours (retire
     stops before a stop entry, which then retires alone as the head, in the squash cycle).
   - **Training.** Up to three CTIs resolve per cycle; the predictor trains one per cycle
     through an 8-entry queue in age order that drops on overflow (training is a hint; 29
     drops in the 60 M boot). The restart's own CTI bypasses the queue and trains in its
     restart cycle, so rule D16 (it trains before its squash) holds by construction; queued
     entries younger than a pending restart are discarded. The weak-branch count (`wk_n`) subtracts up to three per cycle.
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

   **5.2d in detail (design, 2026-10-04).** M today: `u_iq_l` (in order) -> `i_*` -> `u_x`
   computes `m_addr` -> the dTLB's lookup port (`s_`) translates it and fills the op's LQ/SQ
   entry (VA, PA, `tv`, `unc`, `mem`; a load may start its access in the same pass) -> for an
   AMO, LR/SC or CBO, the LSU's own FSM at the ROB head. Everything else in M is already dead or
   asserted out. The queue side already walks (`wk_*`, the `t_` port), records faults in entries,
   starts loads (`acc`), and commits and drains stores. What M alone still provides, and so what
   5.2d has to replace:
   - **the first dTLB lookup of every load and store** (the queue walker serves only M's misses);
   - **address-only faults** (a page-crossing misaligned access, a non-canonical VA), trapping
     from M with the full 64-bit `tval` (the entries hold VA[38:0]);
   - **store data already in the PRF at issue** (`a_data`), including the FP file's for
     `fsw`/`fsd`: M's `ra2` is the only integer-side port that reads the FP file;
   - **program-order address fills**, which two things assume: a CBO waiting on `sq_av_any`
     (meant as "an older store's address is unknown") and the LQ's `l_older` timing argument;
   - **the head-only ops** (AMO, LR/SC, CBO) and M's trap and redirect.

   The increments, each lockstep-gated:
   1. **5.2d-a: plain loads and integer stores generate their address in their slot's lane.**
      They become class I (`smolrv64_gclass`), keep their LQ/SQ credit, and the group rule
      becomes "at most one load and one store" (the queues' single allocation ports) instead of
      "one ordered op". The lane treats one like a multiply: no write, wake or ROB completion at
      issue, no wake at select (`e_long`). Built (2026-10-04) with M as the fill stage rather
      than three fills a cycle: the lane's VA (and a store's rs2) lands in arrival registers by
      LQ/SQ entry, the rest of the op in a record written at dispatch, and M fills in program
      order -- the oldest unfilled load or store once its VA is in, or `i_*`'s op -- through the
      translate, fill, early start and address-only faults it already had. In-order fills are
      forced while a fault still waits in M for the ROB head: a younger wrong-path fault taken
      ahead of an older arrival deadlocks the head on that older op (the first 60 M run, in
      OpenSBI's memmove). Recording address-only faults in the entry (5.2d-b) lifts it. The CBO gets a
      dispatch credit instead of an age compare: it dispatches once every older load and store
      has its address, and no load or store dispatches while one is in flight, so `sq_av_any`
      keeps meaning "an older store" and no younger load reaches memory first. M keeps FP stores
      and the head-only ops. A lane's VA goes straight into M when its op is the oldest unfilled
      one (the arrival registers hold it otherwise), so a load reaches M as soon as it did from
      `u_iq_l`.

      **Measured (2026-10-04): -3.7% at 60 M (41,939,504 retires), Dhrystone -5.9%.** It does what
      it is for -- Dhrystone's M-on-memory stall falls from 19.4% to 4.1% of cycles and 3-wide
      dispatch rises from 10.2% to 16.0% -- but memory issue now shares the three lanes with the
      ALU ops, so the machine has three issue ports where it had four: ALU ops ready but waiting
      rise 3.8 M -> 12.1 M cycles in the boot, and the ROB and both queues fill. The fourth lane
      returns the port. **No tuning before the end state** (Tommy): the select policy is tuned
      once the four lanes exist. The ideal there is branch, store, load, divide, multiply, the
      rest, with age as the secondary key, costed against the scheduler's select path; a
      long-latency-first select alone measured +0.7% on Dhrystone.
   2. **5.2d-b: FP stores and the head-only ops leave M.** Built (2026-10-04) in three gated
      steps. (1) A load's or store's address-only fault rides in its entry, filled as faulted, and
      traps from the SYSQ like a page fault (tval from a full-VA side array): M holds only its
      own accesses' faults (+0.16% at 60 M). (2) The SQ fetches store data itself: a value in a
      register file at allocation is read on `ra2` by the SQ's oldest entry still waiting for
      one, a later one by the snoop; lanes deliver only a VA, and FP stores are lane stores
      (-0.27%). (3) An AMO, LR/SC or CBO goes alone, so in lane A, which generates its address
      and reads its rs2 into a one-entry head-op register; M takes it when no load or store is
      unfilled and runs it at the ROB head. One head op is in flight at a time. `u_iq_l`, the
      issue register `i_*` and M's execute unit `u_x` are gone (-0.02%). M remains as the fill
      and head-op stage.
   3. **5.2d-c: SH_LD dissolves.** A landing load (and the head-op result, and the SYSQ's CSR
      read) announces itself a cycle ahead and takes its lane's next free write slot, the
      reservation 5.2c built.

      **5.2d-c in detail (design, 2026-10-04, for review).** A load's, AMO's or CSR read's
      destination moves from SH_LD to its slot's lane shard (`shard_of`), so the landing needs
      the lane's one write port in a cycle the lane leaves empty. 5.2c's reservation works
      because the multiply is known at execute T and writes at T+2: the registered `m*_1` is
      the lane's `unit_busy` at T+1, nothing executes at T+2, and the slot carries the wake,
      the ROB completion and the result into the lane's write register. A landing has to be
      known two cycles ahead the same way; "a cycle ahead" is too late for a registered
      `unit_busy`. The fork:
      - **A: the D$ lookup is the announce.** A load's data is `rd_valid` two
        cycles after its lookup on a hit, and a miss lands through a released waiter's replay,
        which is again a lookup two cycles ahead. So every lookup of a lane's load reserves
        that lane's slot two cycles on, exactly like a multiply; the landing then is the
        multiply's T+2 slot (wake, ROB completion, the lane's write register). A lookup that
        misses wastes its reserved slot. Needs: rv_dcache exports the lookup and its tag, the
        LQ keeps each entry's lane, and the uncached/device and straddle paths (the LSU's FSM)
        announce the same way or land in a reserved slot of their own.
      - **B: a landing buffer per lane.** The landing waits in a one-entry buffer and takes the
        lane's next empty write slot, the buffer's occupancy (registered) holding the lane's
        select for a cycle. No D$ protocol change, but load-to-use grows by a cycle or more in a
        busy lane, and the buffer is a second write path into the lane's register.
      The head-op result and the SYSQ's CSR read are rare and serialised: they take the same
      buffer or reservation as the slot they dispatched in (always slot A for a head op).

      **Built (2026-10-05):** B with a 4-entry landing buffer per lane whose occupancy is the
      lane's `unit_busy`; a landing completes in the ROB when it drains. The SYSQ's result (a CSR
      read) takes lane A's slot too, but never waits: it fires only into a free lane A, which holds
      its select while a system op is the ROB head, so a redirecting system op writes in its fire
      cycle -- a write kept past its own flush lands in a register rename has rolled back. With
      that, `sy_fire` no longer waits for the LD port (`port_yield`), and SH_LD's bank is deleted.
      60 M: 40,754,856 (-2.82% on 5.2d-b, the lanes now also carrying the landings).

      **Decided (Tommy, 2026-10-04): B now, load-hit speculation later.** A is load-hit
      speculation: dependents woken from the lookup must be rolled back when it misses, which is
      the complicated part. It is postponed to its own step after the lanes, and it is
      absolutely needed for good performance; 5.2d-c lands non-speculatively (B), the data known
      before the lane's slot is taken.

   The defaults chosen where the design forks: fills from up to three lanes rather than one
   memory op per cycle (an issue-side limit would put cross-lane arbitration into select);
   one `s_` lookup a cycle with the walker as overflow, measured before a second lookup port
   is added; FP stores in M for one increment rather than a split store-data uop.

5. **5.2e, SH_FE dissolves.** Built (2026-10-05): an FP op's or divide's integer destination is
   its slot's lane shard; the FE stream (the F stage's landing, else the MD stage's divide)
   takes the lane's landing buffer as a load does, behind the buffer, the SYSQ and the load
   landing, and completes when it drains. SH_FE's bank is deleted, and the shard numbers 1 and
   2 (SH_LD, SH_FE) are free for 5.4. 60 M: 40,883,141 (+0.31% on 5.2d-c).

What stays in `u_iq_f` after 5.2: FP arithmetic, divide, and the SYSQ's system ops. Their
integer results (FP compares and converts, CSR reads, the divide) cross into a lane's write
slot.

## Step 5.4: IW=4 (design, 2026-10-05)

IW=4 is the fourth lane (D), the fourth slot through rename, dispatch and retire, and the FP
file's fourth slice; then the sharded ROB and restart at resolve, which land together (the
decision of 2026-10-01). With SH_LD and SH_FE gone, the eight shard numbers are exactly the
four lanes and the four FP slices.

**What a fourth slot needs, by structure:**

- **Shards.** Renumbered: lanes 0-3, FP slices 4-7, so "an f-register" is one shard bit.
  SH_LD's and SH_FE's free lists go (nothing renames into them since 5.2d-c and 5.2e).
- **Front end.** The decoupling queue forms groups of up to `FW` = 4 at its head and decode
  writes four records a cycle. The group rules are unchanged: at most one load, one store and
  one FP/system op; a head op alone.
- **Rename.** A fourth port: D's sources compare against A's, B's and C's destinations (the
  intra-group bypass chain grows by one), a fourth SMAP and RMAP copy, a fourth commit port.
  The free lists already have next_pow2(IW) = 4 banks.
- **Pending.** A fourth set port and six more queries (D's two map candidates, three sources).
- **ROB.** A fourth allocation and commit port (dense, 32 entries, through 5.4c; rows in 5.4d).
- **Lane D.** Scheduler, dispatch stage, payload and tag arrays, select register, exec,
  multiplier, address generation into the LQ/SQ arrival registers, landing buffer, write
  register and resolve register; the oldest-mispredict pick and the training queue's ranking
  go four-way. Its own PRF shard.
- **PRF.** Each integer shard gains lane D's two read ports: 12 per shard (two per lane, the F
  port's three, the store queue's one). The FP file gains slice F3: two banks and the
  live-value bit, like F0-F2.
- **Wakeup.** Six broadcasts at IW=4: the four lanes' write registers and the two FP streams
  (LD and FE, which since 5.2e write only the FP slices). **A lane's scheduler needs only the
  lanes' four**: a lane op never names an f-register (an FP store's rs2 is the store queue's
  and is zeroed in the lane; the PRF asserts the lanes' ports never read an FP shard), so an
  FP stream can never wake it. The FP scheduler and the store queue's snoop watch all six.

**Increments**, each lockstep-gated:

1. **5.4a: the lanes wake on the lanes (IW=3, bit-identical).** The three lane schedulers and
   their dispatch-stage folds drop the LD and FE comparators: two of five per source per
   entry, out of the scheduler, which is the critical path. The shards are renumbered in the
   same step (retire-identical: the numbers are names).
   **Built (2026-10-05):** retire-identical at 60 M (40,883,141). The shard numbers live in
   `core/smolrv64_shards.vh`; rename builds free lists for the shards the width uses (six at
   IW=3: SH_LD's and SH_FE's eight LUTRAM banks are gone).
2. **5.4b: the lanes and slots as arrays (IW=3, cycle-identical).** The per-lane and per-slot
   copies (`qa/qb/qc`, `a/a2/a3`, `mA/mB/mC`, `dA/dB/dC`, `d/d2/d3`, the rename and ROB ports
   `_b/_c`) become generate loops over `IW`. Lane D is then a parameter rather than a fourth
   copy of some 250 sites in the core. Gate: the same 60 M retire count and the same timing
   at IW=3.
   **Built (2026-10-05):** cycle-identical at 60 M (40,756,540). The PRF, the pending table,
   the ROB and rename take per-lane and per-slot vectors; the core's slots are one generate
   (`sl[k]`), its lanes another (`ln[k]`), and its commit side is per port (`rc_*`; the core's
   and the SoC's `retire` is a bit per commit port). The architectural shadow register file
   (`rv_regfile`), which nothing read, is gone. The credits' layout follows IW
   (`smolrv64_credits.vh`), lane k > 0 completes on ROB port 3 + k, and nothing in the core
   names a lane or a slot past the arrays.
3. **5.4c: IW=4 with the dense ROB.** `SMOLRV64_IW=4`: lane D, slot D, slice F3. Lockstep IPC
   at 60 M and 300 M against IW=3 before timing is fought, as at Stage 3.
   **Built (2026-10-05):** with 5.4b the width is a parameter, so IW=4 needed only the ROB's
   fourth lane completion port and the credit layout (both in 5.4b-6); `build.tcl` takes 2..4.
   The 240 riscv-tests pass, and the boot runs in lockstep: 60 M retires 41,109,903 (+0.87% on
   IW=3's 40,756,540), 300 M 197,517,200 (-0.98% on IW=3's 199,478,367), on the dense 32-entry
   ROB. The fourth slot buys nothing yet; 5.4d's rows come next.
4. **5.4d: the sharded ROB, 32 rows of four.** A group takes a row, slot k in column k; the
   ROB's credit is a row; `{row, col}` is the age. Each lane completes its own column. The
   ports that complete any column -- the FP landing, the store queue's irrevocable take, the
   MD stage and M -- take the column from the slot carried in their tag. The landings already
   go through their op's lane.
   **Built (2026-10-05):** `smolrv64_rob` keeps a row per group: column k is one LUTRAM written
   by slot k and read by commit port k, the head commits along its row from its first
   uncommitted column, and the ROB's credit is a free row. A lane's completion port carries its
   column as a constant (asserted): every integer register is renamed in its slot's lane, and a
   load without a destination never lands (it completes on M's port), so the expected x0
   exception does not arise and the LQ carries nothing new. 60 M: 41,106,564 at IW=3 (+0.78% on
   the dense ROB's 40,786,470), 41,464,991 at IW=4 (+0.86% on its dense 41,109,903); 300 M at
   IW=4 +3.35% on the dense ROB.
5. **5.4e: restart at resolve (C6).** A mispredict recovers when it resolves, not when it
   reaches the head. Fetch already restarts at resolve (`fr_set`); what moves is the squash.
   Everything younger than the branch dies by age (`{row, col}` against the branch's) in the
   schedulers, the dispatch stages, the LQ/SQ, the landing buffers, the FP and MD stages and
   the ROB, and the rename map returns to its state just after the branch.

   **The map's recovery is the open decision.** Rollback today is `lv := 0` onto the committed
   map, correct only at the head. Two ways to the branch's state:

   - **Walk back (the default).** Each ROB entry keeps the mapping its rename displaced
     (`pold`, 10 bits). From the tail back to the branch, a row a cycle, rename rewrites
     `SMAP[rd] := pold` through its own four write ports, which are idle because rename is
     frozen while the restart's path fills the queue, and moves the free lists' speculative
     heads back over the walked registers. No checkpoints and no wide restore. The walk
     overlaps fetch's refill, which takes longer than a typical walk of a few rows.
   - **Checkpoints.** A copy of SMAP and the free-list heads per unresolved branch (a credit),
     restored in one cycle. The restore is the 640-flop parallel load the rename module's
     header rejects for the head rollback on fanout, times the number of checkpoints.

   The age kill is a compare per entry. It is registered (the kill vector is computed from
   flops the cycle after resolve), so a doomed entry can still issue in between; its write is
   harmless because it lands before the walk frees its register.

**Defaults where the design forks:** the dense ROB stays through 5.4c, so IW=4's IPC is
measured against one change; slice F3 rather than three FP slices shared by four slots (a
partition per slot keeps each FP free list at one allocation a cycle); walk back for C6.

## Step 5.4e in detail: restart at resolve (2026-10-05)

**What it buys.** In the 60 M boot `bs:drain` is 3.84% of cycles: a mispredict has resolved,
fetch has restarted on the right path, and rename sits frozen until the branch reaches the ROB
head (`fr_v` gates `cr_pop` and `d_take`). That is the ceiling; the walk below costs a cycle
per row of wrong path.

**What holds a younger op today, and how the flush clears it.** Every structure clears
wholesale on `redirect`, which fires only at the head:

| holds | names its op by | in program order? | partial kill |
|---|---|---|---|
| ROB `v`/`done`, head/tail/irr | its own `{row, col}` | yes | tail back to the row after the branch's; the branch row's younger columns' `v` cleared |
| rename `lv`, SMAP, free-list `h` | -- | allocation order | **the walk** (below) |
| pending table | physical register | -- | none: a walked register's bit stays set until it is renamed again |
| lane schedulers, `u_iq_f` | `e_rob` | no | per entry, by age |
| dispatch stages `stg_v`, select `j_v`/lane `v`, `m1`/`m2` | their `rob` | -- | by age |
| landing buffers `lb_*` | `lb_rob` | arrival order | a dead entry drains as a bubble (no write, no wake, no completion) |
| M, `ho_v`, `la_v`/`sa_v`, SYSQ `sy_v`, `qfr_v`, `lr_v`, MD `md_v` (divider abort), F `f_valid`, `icr_v` | `rob` | -- | by age |
| LQ | `rob` | allocated in order, frees out of order | dead entries are the youngest: `v` cleared, tail back to the oldest dead; the LSU's `o_kill` takes the dead entries' tags (it already drops a killed tag's response and holds the tag until it returns) |
| SQ | `rob`, `sqn` | yes | the flush already keeps the committed part; it keeps everything older than the branch instead |
| training queue | `tq_rob`, `tq_seq` | resolve order | by age (the restarted path reuses the dead ops' sequence numbers) |
| FPU (fpnew, 4 in flight) | its tag, which holds `rob` | no | **no per-op kill in fpnew**: the release waits for its dead ops (below) |
| `csr_infl`, `ho_inf` | -- | -- | cleared when the op that set them dies |
| `wk_n` (weak branches in flight) | -- | -- | zeroed: it only holds walks back, and a low count costs nothing but a walk |

`ser_inflight` needs nothing: a serialising op dispatches only into an empty ROB, so no
unresolved branch is older than it. `inject_inflight` already clears on `fr_v`.

**The age kill, computed once (rule: one precondition, one site).** At resolve (`fr_set`) the
branch's `{row, col}` is registered; the cycle after, a central block turns it, the head row
and the tail into `k_v`, a one-hot **dead-row vector** (rows strictly after the branch's, up to
the tail) and `{k_row, k_col}`. An entry is dead when `dead_row[e_row] | (e_row == k_row &
e_col > k_col)`: a 32:1 mux and a 2-bit compare per entry, from flops, nothing from the
resolve cone. Everything above applies the same function.

**The FPU holds the release.** fpnew has one global flush and no per-op kill, so a dead op in
it must land before its register and ROB index are handed out again. A table of the FPU's ops
in flight (by ROB index, added as the unit accepts one, removed as its result lands) says
whether a dead one is left; the release waits for none. (A generation bit per ROB entry,
filtering the landing, was the alternative; the table is exact and needs no lookup at the
landing.) The MD stage holds one op and aborts it by age.

**The walk restores the rename map.** Each ROB entry keeps `pold`, the mapping its rename
displaced (`lv[rd] ? SMAP[rd] : RMAP[rd]`, or an earlier slot's new register when the group
writes `rd` twice): 10 bits per column, and rename reads each slot's `rd` as it reads its
sources. From the cycle after resolve a cursor walks the ROB from the tail back to the
branch, a row a cycle, on a second read port of each column array. For each dead entry with a
destination it writes `SMAP[rd] := pold` through that column's own copy (`newer[rd] := k`,
the oldest column winning within a row) and sets `lv[rd]`; and its shard's speculative
free-list head steps back one. A column maps to two shards (lane k, FP slice k), so no shard
steps back twice in a row. Restoring `lv[rd] := 1` over a `pold` that came from RMAP is sound:
`lv[rd] = 0` meant no op in flight wrote `rd`, so RMAP keeps that mapping until a younger
writer commits, and the younger writers are the dead ones. Rename's write ports are idle:
rename is frozen for the walk. Commits go on at the head meanwhile (RMAP and the free-list
tails are theirs, the walk touches neither).

An older mispredict resolving during the walk moves the walk's target back; the cursor goes
on. A younger one is dead. A trap or system redirect at the head is the full flush as today
and ends any walk.

**The increments**, each in lockstep (60 M, 300 M, glibc, sysd, storm):

1. **5.4e-1: the walk, checked at the head.** `pold` in the ROB, the cursor, the walk's map
   and free-list writes. Rename stays frozen until the branch reaches the head and the full
   flush still runs there, so the machine is cycle-identical. Checked in that cycle: every
   register's speculative mapping equals its committed one after the branch's commit, and
   every shard's speculative head equals its committed head. A walk that is wrong anywhere
   fails at the first mispredict.
   **Built (2026-10-05):** cycle-identical at 60 M (41,106,564 at IW=3, 41,464,991 at IW=4).
   Of the 482,239 squashes at the head in the IW=3 boot, 123,380 come after the walk has ended
   and are checked; the rest reach the head first (from resolve to head is about 5 cycles on
   average, `bs:drain` 2.3 M cycles over 482 K squashes).
2. **5.4e-2: the age kill, checked at the head.** The central dead-row block, the kill in
   every row of the table, `o_kill` by tag. Rename is
   still frozen to the head, so killing early only frees the wrong path's resources sooner.
   Checked when the squash fires at the head: no live op anywhere in the backend (schedulers,
   stages, landing buffers, LQ, uncommitted SQ, M, SYSQ, MD, F), so the kill missed nothing.
   **Built (2026-10-05):** the dead-entry vector (`kd`) is the ROB's, from flops, and every
   structure clears its dead ops from it each cycle the kill holds; the kill itself is the
   check's subject, so holding the release for the FPU's dead ops is 5.4e-3's. 60 M:
   41,190,896 at IW=3 (+0.21%: the wrong path leaves the queues sooner), 41,498,079 at IW=4;
   the check holds at 136,512 squashes. Its first runs found four real faults, each now an
   assertion or fixed by construction: a dead store delivering its address after its entry
   died (the queues' kill masks name every dead slot while the kill holds), the store queue's
   conflict rows written whole over the same cycle's cell updates (per cell now), the load
   queue's candidate pointer left past the rolled-back tail (it stays only on a waiting live
   load; every waiting load lies in acc..tail, asserted), and the walker locking an entry that
   dies in that cycle (it never takes a dead one).
3. **5.4e-3: release at the walk's end.** `fr_v` drops when the cursor reaches the branch and
   the kill has applied; the branch then commits like any op. The restarted path dispatches
   with sequence numbers the dead ops held, which is why every seq-compared structure (LQ, SQ,
   training queue, the resolve registers) is in the kill. IPC is measured here, at IW=3 and
   IW=4, against 5.4d.
   **Built (2026-10-05):** the release (`fr_rel`) needs the walk at its branch, the kill held
   for `KREL` = 6 cycles (an op in transit dies at its next register) and no dead op in the
   FPU; it drops the dead entries, returns the tail to the row after the branch's and completes
   the branch. The squash at the head stays as the fast path for a branch that reaches the head
   first, so the 5.4e-1 and 5.4e-2 checks keep their meaning; in the 60 M boot 104,841
   restarts end at their release and 396,756 at the head. The weak-branch count (the I$ fill
   hold) is a bit per ROB entry, cleared for the dead at the release. 60 M: 41,509,274 at IW=3
   (+0.98% on 5.4d), 41,764,026 at IW=4 (+0.72%); `workloads/brbench`'s `drain` (a mispredict
   behind a D$ miss) 32.26 -> 17.05 cycles per iteration. A release in the cycle an older
   restart resolves would lose that restart (asserted, and the release waits a cycle).

**Cost.** `pold` is 10 bits per ROB entry (the 16-bit entry becomes 26: +960 bits of LUTRAM at
IW=3) and a second read port per column; one read per slot on each SMAP and RMAP copy; the
dead-row compare per scheduler, LQ and landing-buffer entry; the FPU's in-flight table (8
entries of a ROB index).

## Step 5.4f: two read ports per lane, three in the FP pipe (design, 2026-10-06)

**Decided (Tommy, 2026-10-06): IW=4 as soon as it times, before 5.5, and with 8 integer read
ports, not 12.** Integer registers are read only by the four lanes, two ports each; FP
registers only in the FP pipe, three ports (a 3R1W FP file).

**Today** every integer shard has `2*NL + 4` read ports (10 at IW=3, 12 at IW=4): the lanes'
two each, the store queue's data read, and the F/CTF port's three, which read the integer
operands of divides, of the FP ops with an integer source (`fcvt.*.w/l`, `fmv.*.x`) and of
system ops (a CSR's `rs1`, `sfence.vma`). Each FP slice has four reads (the F port's three, the
store queue's) and two write banks behind a live-value table (FP loads, the F stage).

**The moves**, each a lockstep increment, measured at IW=3 and IW=4:

1. **Store data from the lane.** The store's lane already reads `rs2` at its address
   generation; it hands the value to the store queue entry with the address when `rs2` is ready
   (the queue's snoop, armed at allocation, covers a later write). The store queue's read port
   goes. An FP store's data becomes a store-data op in the FP pipe, one of its three reads.
2. **Divides issue in their slot's lane**, which reads both operands and hands them to the one
   MD stage (one divide in flight stays a dispatch credit); the result lands through the lane's
   landing buffer as today.
3. **The FP ops with an integer source issue in their slot's lane**, which reads `rs1` and hands
   it to the F stage (the F stage takes its scheduler's pick or a lane's handoff).
4. **System ops read their operands in lane A**, which already carries them (a head op goes
   alone), and hand them to the SYSQ.
5. **The FP write side:** an FP load's result takes its slice's one write port through a landing
   buffer, as an integer load takes its lane's, and the second bank and the live-value table go.

Then the F port reads FP registers only, the integer shards are 2R per lane, and the FP slices
3R1W. **After 5.4f:** the IW=4 timing build, the default flipped to IW=4 (board gate, GB5),
then 5.5.

## Step 5.5 in detail: load-hit speculation (design, 2026-10-05)

**Today, a hit.** X is the load's execute cycle in its lane (address generation); M fills and
starts it at X+1; the D$ accepts it at T = X+2, compares at T+1 and answers at T+2 (`rd_valid`,
registered); the landing broadcasts the wake in T+2, a dependent is selected then and executes
at T+3 = X+5, reading the value from the landing lane's write register. A miss answers at B+5
(B the fill's last beat) through the D$'s own waiter replay, which is again a lookup two cycles
ahead of its data. The core sees no tagged lookup and no hit or miss before the data.

**What it would buy.** In the 60 M boot at 5.4e-3, `be:dep-load` is 10.62% of all cycles
(6.37 M), the largest back-end bucket after `be:sq-full` (11.66%), now that restart at resolve
has emptied `be:rob-full` (10.19% -> 0.05%). Speculation moves the dependent's execute to T+2,
the data's own cycle: one cycle of every load-to-use, about a fifth of it.

**The hard part is the rollback, and three facts shape it.**

1. *A speculative wake cannot latch.* A dependent woken at T but not selected at T+1 would
   execute later with nothing in the register if the load missed, or before a buffered landing
   writes it. So the speculative wake is a separate, non-latching wake port (`srdy` sees it in
   T+1 only); an entry not selected then waits for the real wake as today.
2. *The landing must be in a known cycle.* A consumer executing at T+2 reads the data from the
   D$ response; later consumers read the landing lane's write register and the PRF, so a hit
   must land at T+2 exactly, never in the landing buffer. That is the 5.2d-c fork's option A:
   every lookup of a lane's load reserves the landing lane's write slot two cycles on, as a
   multiply reserves its slot (`unit_busy` at T+1); a miss wastes the slot, and the replay's
   lookup reserves again.
3. *A cancelled op must not have woken anyone.* A dependent issued at T+1 on the speculative
   wake learns the hit at T+2 (`rd_valid`, a register). If it woke its own dependents at its
   select (the lanes' dependency matrix) or at its execute (the broadcast), a miss would have to
   reach them too, and those wakes latch: the rollback would cascade. So an op issued on the
   speculative wake wakes nobody until its load is confirmed: its broadcast comes from its
   write register a cycle late (T+3, where `rd_valid` can gate it from flops) and its matrix
   wake is suppressed. Its own dependents then run exactly as they do today; the gain is the
   first consumer's cycle, which is every load whose consumer is an address (pointer chasing),
   a store's data, a branch or the end of a chain.

**The rollback.** At T+2 a lane executing an op whose source is the speculated register (one
registered tag a cycle, compared in the execute stage, off the scheduler) and whose load did not
answer cancels it: no write, no wake, no ROB completion, no resolve, no multiply start, no
address delivery; and its scheduler entry, still held by the select register, comes back
(`v` set, that source's ready bit cleared). Nothing else saw the op.

**The cost on the scheduler, the critical path:** one more wake comparator per source per entry
(the 5.4a step removed two), the suppression of the matrix wake for an entry issued on the
speculative port (a select-path term), and the restore write. The suppression is the term to
measure first; the default if it does not close is to give speculatively issued ops no matrix
wake at all only in the lane that took them.

**The increments**, each in lockstep:

1. **5.5a: the lookup announces and reserves.** `rv_dcache` exports each lookup of a
   fast-path load (a new accept or a waiter's replay) with its tag at T; the LSU names the LQ
   entry, hence the landing lane, and reserves that lane's write slot at T+2 (`unit_busy` at
   T+1). A hit then lands in its reserved slot, never in the buffer. No speculation yet: the
   wake is the landing's, as today. Measured alone (it can lose: a reserved slot that misses is
   a lost lane cycle).
2. **5.5b: the speculative wake and the D$ bypass.** The non-latching wake port at T, the
   D$ response bypassed into every lane's operands at T+2, the late broadcast and suppressed
   matrix wake for speculatively issued ops, the cancel-and-restore on a miss. Integer loads
   only; FP loads keep the landing's wake.
3. **5.5c: chains,** only if 5.5b's numbers say the first consumer is not enough: a confirmed
   speculative op's dependents woken at T+2 instead of T+3.

## The PRF in block RAM: built, measured, dropped (2026-10-03)

Built on `wip/prf-r` (parked): a read stage on the lanes, a two-deep bypass, the lanes waking
each other from the read stage, then the lanes' six read ports as block RAM (30 RAMB36). It
is cycle-identical between the two increments and correct in the lockstep, and it is out
(Tommy, 2026-10-03):

- **It saved little area.** `u_prf` went from 18,856 to 17,120 LUTs, all of it LUTRAM. The
  instance's 13.5K logic LUTs are not the register file's: counted by cell name they are its
  consumers (the lanes' ALUs, M's and the CTF stage's exec units, the FPU's operand muxes, the
  divider's operands), which Vivado pulls across the boundary. The out-of-context comparison
  (2,152 against 5,032 LUTs) did not carry over to the chip. Per-instance utilization after
  flattening is not evidence of what an RTL block costs.
- **It cost IPC.** -1.15% at 60 M against 5.2a with the early wake. At ROB 64 the ROB-full
  stalls vanish and the gap stays (-1.22%): it is latency, ALU results reaching M, F and the
  serialise drain a cycle later, spread thinly over many stall causes.
- **It added a path.** Block-RAM clock-to-out now led the execute cycle (block RAM -> shard
  mux -> bypass -> ALU -> result register, 16-17 levels), and WNS moved only from -0.334 to
  -0.310 ns.

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
