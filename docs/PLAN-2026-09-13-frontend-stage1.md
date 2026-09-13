# Stage 1 implementation design (frontend rewrite)

Branch `wip/fe-stage1` off main. Parent plan: `PLAN-2026-09-13-frontend.md`. Goal of Stage 1:
the conventional **FP/FA/FD** front end, `IW=1` bit-identical, no `apc`/`pnpc_kind`/`lenp`.
`OOO2_IW` and the VHPR I$ are later stages; Stage 1 keeps today's PIPT I$ + fetch buffer and
today's IW=1 backend untouched.

## The key realisation (from reading `src/fetch.v`)

`apc` is an **early copy** of values the pipeline already computes at `base_pc`:
- the sequential next-PC is `pc_q + 2*al_consumed` (`norm_npc`) — the aligner's own consumed
  count, combinational at `base_pc`; `lenp` exists ONLY to reproduce that a cycle early for the
  ahead read.
- the predicted next-PC is `pred_npc` from the predictor resolved at `base_pc`.

So the ahead read (`apc` → `btb_raw`/`ycorr_raw`, resolved next cycle) is not producing a
*different* prediction — it is producing the *same* prediction one cycle early to hide the BRAM
read latency. The BTB spike (2026-09-13: +2.6–3.5 ns OOC, YAGS excluded) proved a
combinational-read BTB resolves at `base_pc` in one cycle. Therefore Stage 1 can read the
predictor **combinationally at `base_pc`** and feed the **fetch-target register** with
`{redirect_pc, pred_npc, norm_npc}` — deleting `apc`, `apred_v`, `pnpc_kind` and `lenp` while
producing the **same prediction outcomes**. That makes Stage 1 bit-identical (same retire
stream AND retire count within noise), not merely correct.

YAGS stays **combinational in the resolve** for Stage 1 (as today), single-cycle. The
two-cycle YAGS **override** is a Stage 3 refinement, introduced only when the widened cone
needs it (the census will say). Stage 1 does not add the override.

## Increments (each: lint → 240/0 → cosim retire ±0.5% → census/timing; board on the last)

**Refinement after reading the code (2026-09-13):** `apc` is *only* the predictor's read
address — `pc_q` already advances on `norm_npc` (which folds in `pred_npc` when `pred_v`), and
`lenp` feeds `apc`'s advance alone. So increments 1 and 2 **collapse into one change** that
touches only `ooo2_predictor.v` (read at `base_pc`), `src/fetch.v` (delete the `apc` output,
`lenp`, `len_adv`, `lidx`, `al_cons_m1` training) and `ooo2_frontend.v` (drop the `.apc`/
`.apred_v` wires). **`pnpc_kind`, the queue, and the decode rebuild stay untouched in Stage 1**
— they are the queue-width optimisation, not the `apc` hack; revisit later if wanted. Not
literally bit-identical: the exact `base_pc` read drops the `apc`-mismatch lost predictions
(~0.13% of retires), so the retire count rises slightly — strictly better, within the 0.5%
band. Exact anchors in `ooo2_predictor.v`: the registered read block (`apc_en`, `btb_raw`/
`btb_qpc`/`ycorr_raw`) and the write-forward (`t_fwd_q`/`y_fwd_q`/`t_dat_q`/`y_dat_q`) delete
together — a combinational distributed-RAM read at `base_pc` sees a same-cycle train write as
the OLD value, and the redirect refetch a cycle later reads the updated array, so no forward is
needed; `tag_hit` drops the `btb_qpc == base_pc` term; the BTB/YAGS `ram_style` goes
block→distributed; the BP_TRACE block loses its `apc`/`btb_qpc` fields.

1. **Predictor read at `base_pc`, combinational** (`ooo2_predictor.v`). Convert `btb`/`ycorr`
   from BRAM synchronous-read addressed by `apc` to distributed-RAM combinational-read addressed
   by `base_pc`; the tag/target/direction resolve stays where it is. Delete the `apc` input, the
   `btb_qpc`/`apred_v` machinery and the write-forward (a combinational read sees the write
   the same cycle — keep the 1W train port). Keep `pred_v`/`pred_tgt`/`pred_npc`/`pd_fetch`
   identical in meaning. **Risk:** the BTB moving BRAM→LUTRAM is the storage the spike measured
   as fine at 1024; confirm with the census. **Bit-identical check:** the retire count must match
   14,301,801 ± 0.5% and the cosim stay clean.

2. **Fetch-target register** (`src/fetch.v`). Delete `apc`, `lenp`, `len_adv`, `lidx`. The PC
   register's next value is `redirect ? redirect_pc : irq_go ? ipc_q : strad ? pc_q+2 :
   pred_v ? pred_npc : norm_npc` — every arm already exists; this only removes the `apc` arm and
   its table. `pnpc_kind` output deleted; the F/X queue carries `pred_npc` directly if a consumer
   needs it (check `ooo2_frontend.v`/decode — today decode rebuilds it from the decoded length,
   which still works, so `pnpc_kind` may just delete with no queue change).

3. **Rename the F/X queue → the decoupling queue** (`ooo2_frontend.v`): `dq_*`, relabel
   `FE_QUE` keeping its counter id/bit. Cosmetic; cosim retire count identical.

4. **Straddle: deferred to Stage 2.** The `strad`/`ipc_q` FSM handles a 32-bit op crossing the
   page/window end; the alignment latch + aligner carry that replace it belong with the VHPR I$
   (Stage 2). Stage 1 keeps `strad` unchanged. (The plan's "straddle FSM → aligner's carry" lands
   in Stage 2, not here.)

## Not in Stage 1

The BTB→base_pc read is the whole prediction restructure at IW=1. The override, the VHPR I$,
the alignment latch, `OOO2_IW>1`, the livemap-banked rename map — all later stages. Keeping
Stage 1 to "same outcomes, conventional structure" is what makes it the zero-IPC-risk step the
parent plan promises.

## Spec updates (per commit)

§2/§2.2 (the F/X queue renamed; the ahead-PC gone from the fetch description), §4.1 (`apc`,
`pnpc_kind`, `lenp` deleted; the fetch-target register), §4.2 (the predictor reads at `base_pc`
combinationally; the override deferred), §10.2 (`lenp` row removed), §11 (`FE_QUE` relabel).
