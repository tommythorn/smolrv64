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

- **D$ plan:** 5c (a load asks the D$ by its VA first) and 6b (way 1 hashed by the VA, the reverse
  directory), one board point. The walker that 5b moved to the queues stays.
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
3. **Serialisation out of dispatch** (small, independent).
4. **D$ 5c + 6b.**
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
