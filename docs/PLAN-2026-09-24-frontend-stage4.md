# Frontend Stage 4: a fetch stream decoupled from decode

Tommy's design (2026-09-23), agreed the same night; the highest priority after the e96a6340
validation. The I$ stream runs on its own address register and appends to a byte ring; decode
drains the ring. Fetch stops only for a full ring or a misprediction.

## Why: fetch cannot run ahead of decode today

The I$ is addressed by fetch's PC register, which advances by what the aligner consumed
(`src/fetch.v`: `imem_addr = pc_q`, `pc_q <= pc_plus[...]`). Every I$ read waits for the previous
bundle to be carved, so any aligner hiccup becomes I$ latency and there is no run-ahead to hide
it. Measured with `+fe_trace` and `tools/pipeview` on `workloads/rvbench/local` (IW=3, e96a6340's
tree):

| workload | IPC | what the frontend trace shows |
|---|---:|---|
| `sillyloop` (32 `c.nop`, `addiw`, `bnez`) | 1.79 | a correctly predicted taken branch: the target's bytes are `imem_ok` 4 cycles after the branch's bundle fires; the 8-entry queue covers 4 cycles of 2-wide dispatch, so 2 dispatch bubbles per iteration (19 cycles, ceiling 17 at two ALUs) |
| `sillyfp` (22 `c.nop`, 10 `fcvt.d.w`) | 1.31 | a chunk tail holding less than a full bundle does not fire; fetch waits 2 cycles for the next chunk, every 16 bytes: `fe:align` 23% + `fe:queue` 27% of cycles |
| Dhrystone (5,000 runs) | 0.64 | `fe:icache` 22-27% of cycles, one bubble run per taken CTI |

## The design

- **I$ reads at 8-byte alignment.** The two I$ banks are addressed by their own rows (`pair_e`,
  `pair_o`), so a read returns chunks *n* and *n+1* whatever *n*'s parity. The frontend asks for
  16-byte-aligned pairs today; Stage 4 asks for `fa & ~7`, giving 10-16 useful bytes per read.
- **The fetch address register `fa`** (VA, halfword granular) advances every cycle the ring has
  room for 16 bytes: to the predicted target if a predicted-taken branch ends in the fetched
  bytes, else to `(fa & ~7) + 16`. Nothing on this loop depends on decode.
- **The ring.** 32-48 bytes of halfword slots with head and tail pointers. The I$ writes the fetched
  bytes at the tail (up to and including a predicted-taken branch's last halfword; the rest is
  dropped). Decode reads a window from the registered head and consumption only advances the
  head: no data moves on the critical path. The ring replaces the bundle register, the 8-entry
  decoupling queue and the 16-byte alignment latch.
- **Prediction keyed by the aligned 8-byte granule holding the branch's last halfword**, never by
  where the fetch began: the fetch start depends on how the stream arrived (after a redirect it is
  the target's granule), which is exactly the train/lookup mismatch of D15 and of Proc_8's `ret`.
  A read covers two granules, so the BTB splits odd/even like the I$ and does two lookups per
  cycle; the entry holds the tag, type, target and the last halfword's offset in the granule; the
  first predicted-taken branch at or after `fa` in fetch order wins. The RAS pushes and pops on
  the fetch-time prediction as now. YAGS overrides a cycle later: a disagreement truncates the ring
  after the branch and redirects `fa`.
- **Page end.** A read whose odd chunk lies on the next page appends only the even chunk (the
  VHPR enclosing-page bit, 4 K or 2 M, as now); the straddling instruction completes from the next
  page's read.
- **Training** carries each CTI's PC and length, so resolve computes the same granule and offset
  the lookup used.

## Stages: shorter by two on every refill

Tommy (2026-09-24): "it seems longer than I expect". Fetch into an empty queue to execute is 6
cycles plus the I$ read today:

| today | cycle | Stage 4 |
|---|---|---|
| I$ read at `pc_q` | -1 | I$ read at `fa` |
| `f`: aligner carves a bundle, fetch fires | 0 | ring append |
| `f`: bundle register | 1 | ring head window -> aligner -> IR |
| `q`: decoupling queue -> IR | 2 | rename/dispatch |
| `d`: rename/dispatch | 3 | dispatch-stage register |
| `d`: dispatch-stage register (the IW=3 timing cut) | 4 | wakeup/select |
| `d`: wakeup/select | 5 | execute |
| execute | 6 | |

The bundle register and the queue-to-IR hop go; aligner and decode stay separate stages (the
2026-09-13 FA/FD spike: merged at IW=3 closes OOC at only +1.2 ns, 13 levels). Whether one of the
three dispatch-to-execute cycles can go is a separate question for after Stage 4.

## Timing: two cones, one already measured

1. **The fetch-address loop** (`fa` -> two LUTRAM BTB lookups -> first-taken select -> target /
   RAS / +16 -> `fa`). The single-lookup form of this cone closed OOC with 2.6-3.5 ns to spare at
   every BTB depth (2026-09-13 spike) and ships today reading from `pc_q`. The second lookup and the
   select are a few hundred ps; spike it only if the integrated build says otherwise.
2. **The ring's consumption loop** at IW=3: the registered head selects the aligner's window
   (the rotation), the aligner carves the bundle, and what it consumed advances the head, its PC
   and sequence number. **Spike result (2026-09-24, `src/ooo2_ring_spike.v`, OOC at 6 ns, default
   placement), against today's loop measured the same way:**

   | loop | WNS | Fmax | levels | worst path |
   |---|---:|---:|---:|---|
   | today, `fetch` IW=3 HW=8 (`pc_q` -> window -> aligner -> `pc_q`) | +0.889 | 196 MHz | 16 | `pc_q` -> `ipc_q` |
   | ring, 32 bytes (`RS=16`) | +1.409 | 218 MHz | 20 | `head` -> the 64-bit head-PC add |
   | ring, 64 bytes (`RS=32`) | +1.139 | 206 MHz | 21 | `head` -> the 64-bit head-PC add |

   The ring's loop is faster than today's at either size; its worst path is the carry chain of
   the full-width head-PC increment (8 CARRY8), not the rotation or the aligner, and the real
   design splits it (the page offset added, the page number incremented on a carry).

## Increments, each gated (lint, riscv-tests, benches, memrand/fpmix, rvbench, 60 M and 300 M lockstep, board gate)

1. The ring and `fa`, sequential only (predictor forced not-taken on the fetch side, resolve
   redirects everything): sillyfp's chunk-tail stall must go.
2. Granule-keyed BTB with the training key; the RAS on the fetch-time prediction.
3. The YAGS override against the ring.
4. Delete the bundle register, the decoupling queue, the alignment latch and `pc_q`'s
   consumption-driven advance; re-measure the stage count.

**Done when:** sillyloop runs at 17 cycles per iteration (IPC 2.0), sillyfp near 3, Dhrystone's
`fe:icache` under 5% of cycles, and GB5 on the board validates the tip.
