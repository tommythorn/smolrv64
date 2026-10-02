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

1. **Front end.** Fetch -> one elastic halfword fetch queue (the fetch ring) -> align -> decode ->
   rename -> dispatch. Fetch is limited only by bytes from the I$ and by predicted-taken control
   transfers. **Nothing from the aligner on can stall:** every stage from the aligner to the
   schedulers advances every cycle, through pipeline registers that never hold, with no buffer
   until the execution lanes (today's bundle register, decoupling queue, IR survivors and
   dispatch-stage register go). "Stalling the front end" means the fetch queue feeds the aligner
   bubbles. It does so on registered credits: the ROB, the LQ, the SQ, each lane's scheduler and
   every free list, each counted against everything already between the fetch queue and dispatch
   (the pipe's depth times four), so any op past the queue is guaranteed its room.
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
   own scheduler serves all four slots. Three-operand FP ops are cracked into two µops (TBD).
   Crossings go through a lane: an FP result bound for an integer register returns through a lane's
   write slot; an integer operand for the FP side is read by a lane and handed over.
7. **Divide** is one shared iterative unit; its result returns like a miss.
8. **Branches** resolve in any lane; the oldest mispredict wins by age; predictor training is
   buffered (droppable: it is a hint).
9. **Wakeup** is same-cycle across lanes where timing allows; a delayed path costs IPC, never
   correctness, so it is decided by timing.
10. **Serialising ops** dispatch and wait in the SYSQ for the ROB head; context changes (`satp`,
    `sfence.vma`, `fence.i`) refetch through their redirect. Nothing blocks at dispatch.
11. **Retire** up to four per cycle.

## What stays

- **D$ plan:** 5c (a load asks the D$ by its VA first) and 6b (way 1 hashed by the VA, the reverse
  directory), one board point. The walker that 5b moved to the queues stays.
- **Memory-backend program:** loads past unknown-address stores with replay and a wait predictor
  (C4b step 6), the MSHR window (C5), mid-window recovery (C6), the wrong-path throttle (C9).
- **The wrong-path holds:** no iTLB walk and no I$ line read past a weakly predicted conditional.
- **Future TLB:** the big miss-path TLB, the 2 MiB table, filling a PTE line's eight entries per walk,
  BRAM against LUTRAM.
- **Sscofpmf**, and the **pipe trace** (one code per instruction per cycle, the pair log).
- **The clock review** (250 MHz): a register-read stage, decode before the queue.

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
   - a trace-driven predictor model: path history against today's direction history, and the pairs
     a not-taken branch cuts short;
   - the core's limit study with four lanes;
   - PRF area: four integer shards x eight read ports, plus the FP file;
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
   4. IW=4: four lanes, retire four.
6. **The pipe trace**, built for the pipeline the lanes leave (its stages, its credits, its block
   reasons). Until then the steps use today's trace and counters.
7. **The predictor:** path history, blocks cut at the first predicted-taken transfer.
8. Then loads past unknown-address stores (C4b step 6), C5/C6, the future TLB, the clock review.

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
- Retire up to four per cycle.

## Open

- Cracking three-operand FP ops into two µops.
- The return path for variable-latency results (depth, bypass from it).
- Which addresses the path history hashes (taken-branch addresses or every block), from the model.
