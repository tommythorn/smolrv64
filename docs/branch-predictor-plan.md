# Branch prediction for the sharded-OoO core — Phase 0 plan

Status: design, not yet implemented. Target core: `probe/` (frontend.v → fetch.v /
aligner.v → decode_rename → backend). Today fetch is **fall-through + backend
redirect only** (`fetch.v:5`), so every taken branch and every jump costs a
backend-resolved redirect bubble. This plan adds a frontend predictor so the
common control flow is steered at fetch.

The algorithm is lifted from YAGS+RAS in `~/projects/yarvi/rtl/yarvi.v` (BTB with an
embedded bimodal weight, a tagged YAGS corrector, a RAS, and a global history
register). The *arrays and predict/update logic* port cleanly; the *integration*
(bundle-granular prediction + speculative-state recovery under CPR) is core-specific
and is what this document specifies.

## Scope

- **Phase 0 (this doc):** BTB (block-indexed, with embedded 2-bit bimodal direction)
  + RAS, plus the full per-checkpoint recovery machinery for the speculative state.
  No YAGS / no global history yet — but the GHR checkpoint slots are built now so
  Phase 1 is a drop-in.
- **Phase 1 (later):** add the YAGS corrector + global history register indexing.
  Mechanically this only turns on the (already-checkpointed) `ghr` and adds a second
  table read/update; no new recovery work.

## The invariant that makes this small

`aligner.v:111` ends the bundle at its **first** control-transfer instruction
(`is_cti` → `run = 0`), including it as the youngest valid slot and stopping
`consumed` after it. Consequences we lean on throughout:

1. A fetch bundle contains **at most one CTI, and it is the last valid slot.** No
   truncation logic to add — the "one branch per bundle, last in bundle" property
   that both the predict-redirect and the GHR repair need already holds. It is the
   same invariant `decode_rename`'s last-in-bundle recovery already depends on.
2. `create = disp_fire` (`commit_ctl.v:73`) opens **one checkpoint per dispatched
   bundle** (`cur`, NCHK=4 ring). So "per-bundle" == "per-checkpoint" == "per-CTI".
3. The predictor's *only* speculative state is `{ghr, ras}`. The BTB/YAGS tables are
   written **only at branch resolution** — a cache, never rolled back. And this
   state is a **hint, never architectural**: a mis-restored bit can only cause an
   extra mispredict (which `branch_unit` catches), never a wrong result. Checkpoint
   precision is therefore a perf/area knob, not a correctness obligation.

## Architecture

A new `predictor` module instantiated **inside `frontend.v`**, alongside fetch. It
owns the BTB, RAS, GHR, and the per-checkpoint snapshots of `{ghr, ras}`. It taps
only signals `frontend` already routes — `create`, `rollback`, `rollback_idx`
(inputs) and `cur` (computed internally by `decode_rename`). The single genuinely
new top-level wire is a resolve/training port threaded up from `branch_unit`.

```
            pc_q ──► BTB[block] ─┐
 fetch.v ───┤                    ├─► predict {taken, npc, class} ─► pc_q override
            └─► aligner ─► cti_term, last-slot bytes ┘                 │
                                                                       ▼
            predictor: ghr, ras, chk_{ghr,ras,rptr}[NCHK]   carry pred_npc + btb_idx
                  ▲ create/cur/rollback/rollback_idx              in the branch payload
                  │                                                    │
   branch_unit ───┴──────── resolve {taken,target,class,btb_idx} ◄─────┘  (train + repair)
```

---

## Part A — predict-redirect (the sibling piece)

### A1. BTB lookup (block-indexed)

Index the BTB by the **bundle base PC** = `pc_q`. Because the aligner is a pure
function of the fetched bytes, a given block-start PC deterministically produces the
same bundle and the same terminating CTI, so a block-indexed entry is well-defined.
One BTB read per fetch, in parallel with the aligner. Entry (yarvi-shaped):

```
btb_tag      partial tag of pc_q  (alias filter)
btb_type     3-bit: BR_S_N/W_N/W_T/S_T (cond branch + 2-bit bimodal) | JUMP | CALL | RETURN
btb_target   partial target (for direct branch / JAL / indirect-JALR last-target)
```

**Timing structure (yarvi-faithful):** read the BTB combinationally at the
*computed next PC* and register the result, so the BTB RAM read feeds a register,
not the redirect→fetch path (mirrors `yarvi.v:336-338`). `pc_q`'s next value is
already computed combinationally in fetch (the `+= 2*consumed` advance); read
`BTB[npc]`, register into `btb_q`, predict at T from `btb_q` read at T-1. The exact
pipelining is tuned against `make timing` (FPGA margin is ~zero), but the rule is:
**BTB read terminates at a register, never at the same-cycle PC mux.**

### A2. Class, direction, target, RAS (combinational in the predictor)

The terminating CTI's bytes are in the last valid slot — decode its class directly
(better than yarvi, which waits for the BTB to learn the type):

- **class** ∈ {cond-branch, JAL, JALR-call, JALR-return, JALR-other} from opcode +
  (for call/return) `rd`/`rs1` ∈ {x1,x5}, the standard RAS rule.
- **predicted taken** = unconditional ? 1 : `btb_type[2:1]` says taken (bimodal).
- **predicted target** =
  - return → **RAS top** (`ras[ras_ptr]`);
  - direct branch / JAL / indirect JALR → **`btb_target`** (learned at resolve);
  - (direct targets are PC+imm and *could* be computed from the bytes, but that adds
    an immediate-decode+adder to the fetch path — store-in-BTB instead, like yarvi.)
- **RAS update** (speculative, see Part B): call → push `cti_pc + cti_len`;
  return → pop. Gated by `fetch.fire`.
- **GHR update** (Phase 1, dormant in Phase 0): conditional → shift in predicted dir.

Gate the whole prediction on `cti_term` (the aligner ended the bundle on a CTI, not
a window/page edge or a solo cut). One new aligner output bit; everything else the
predictor re-derives from the existing last-slot `inst`/`pc`.

### A3. Next-PC override in fetch.v

`fetch.v:147-150` advances `pc_q += 2*consumed` on `fire`. Add the override:

```verilog
end else if (fire) begin
   pc_q  <= predict_taken ? predict_target
                          : pc_q + {…, al_consumed, 1'b0};   // fall-through
   seq_q <= seq_q + nvalid;
end
```

`redirect` (backend) keeps priority over both, unchanged. Note this now makes the
**frontend follow unconditional jumps** (JAL/JALR/call/return are "predicted taken"),
removing their backend-redirect bubble — a large part of the IPC win.

### A4. Mispredict detection — carry one field, fold all CTI kinds into one compare

Today `branch_unit.v` assumes predict-not-taken and redirects on
`(is_branch & taken) | is_jump`. With a frontend predictor it must redirect only on a
**genuine misprediction**. The clean encoding: the frontend records the next PC it
actually fetched (`pred_npc`) on the branch's payload; `branch_unit` redirects iff
reality disagrees.

```verilog
// branch_unit, replacing redirect/target:
wire [63:0] actual_npc = taken ? target : seq_npc;   // jumps: taken=1 always
assign redirect        = actual_npc != pred_npc;     // one comparator, all CTI kinds
assign redirect_target = actual_npc;
```

where:
- `pred_npc` = the frontend's chosen next PC for the bundle = `predict_taken ?
  predict_target : (base_pc + 2*consumed)` — a new payload field on the terminating
  CTI (`PAY_PRED_NPC` in `exec_pay.vh`).
- `seq_npc` = sequential next PC = `pc + ilen` — derivable from `pc` + the already-
  carried compressed flag; pass it (or `is_c`) into `branch_unit`.
- `taken` stays as today's per-`br_func` compare; also **expose `taken` and a 2-bit
  call/return class as outputs** for the Part B repair / Part C training.

This folds conditional branch, JAL, JALR, and return into a single
`actual_npc != pred_npc` test, replacing the special-cased `(is_branch&taken)|is_jump`.

---

## Part B — GHR/RAS checkpoint integration (clone of `chk_map`)

The recovery of `{ghr, ras}` is a structural clone of `rename_shard.v`'s `chk_map`
(`rename_shard.v:159-172`): per-checkpoint snapshot, snapshot-at-`create`,
restore-at-`rollback`, using the **same** `create / cur / rollback / rollback_idx`
the renamer already consumes. No new top-level checkpoint plumbing.

```verilog
reg [GHL-1:0] ghr;       reg [GHL-1:0] chk_ghr [0:NCHK-1];
reg [PCW-1:0] ras  [0:RASN-1];   reg [PCW-1:0] chk_ras [0:NCHK-1][0:RASN-1];
reg [RPW-1:0] ras_ptr;   reg [RPW-1:0] chk_rptr[0:NCHK-1];

wire [CBITS-1:0] nxt = cur + 1'b1;
always @(posedge clk)
   if (rollback) begin                          // restore the squashed-bundle's pre-state
      ghr     <= ghr_restored;                  // chk_ghr[rollback_idx], LSB-repaired (B2)
      ras_ptr <= chk_rptr[rollback_idx];
      for (k…) ras[k] <= chk_ras[rollback_idx][k];
   end else begin
      if (fetch_fire) {ghr,ras,ras_ptr} <= {ghr_nxt,ras_nxt,rptr_nxt};   // speculate (B1)
      if (create) begin                         // snapshot post-bundle state into next span
         chk_ghr[nxt]  <= ghr;                  // registered post-fetch-bundle value
         chk_rptr[nxt] <= ras_ptr;
         for (k…) chk_ras[nxt][k] <= ras[k];
      end
   end
```

Semantics are identical to `chk_map`: `chk_spec[S]` = state to restore when span `S`
is reopened = **post-bundle state of `S-1`** = **pre-bundle state of `S`**.
`chk_spec[0]` resets to `{0, empty RAS}` (cf. `chk_map[0]` = identity). Because
`rollback_idx = redirect_ckpt` (`commit_ctl.v:75`) is the *same wire*
`rename_shard` consumes, predictor and rename map roll back to the same point by
construction — no separate index math, no drift.

### B1. Time-domain alignment (the one subtlety)

Speculation happens at **fetch** (cycle T); the snapshot happens at **`create`**
(cycle T+1, rename domain), because `create = disp_fire` lags `fetch.fire` by the one
fetch→rename pipe stage. This lines up for free: the bundle dispatching at `create`
is the one fetched last cycle, so the *registered* `ghr/ras` already hold its
post-state. In a flowing pipe, at edge T+1 the `create` snapshot and the next
`fetch.fire` both read the same old `ghr` value → the snapshot captures exactly the
dispatching bundle's post-state (same argument as `nmap` for `chk_map`).

**Invariant to hold:** `ghr/ras` advance **iff `fetch.fire`** (`= ready & valid`,
the condition that lets the bundle reach the rename boundary). Then back-pressure
freezes the predictor in lockstep with fetch and the delayed `create` still reads the
right value. Gate the speculative update with `fetch.fire`; never free-run the GHR.

### B2. Rollback repair — the only new input

Restoring `chk_spec[rollback_idx]` is almost enough. On a **conditional-branch
direction mispredict**, `rollback_idx = eb_rckpt+1` (`backend_top` `rb_idx`) and the
snapshot's **bit 0** is the branch's *predicted* direction; overwrite it with the
resolved one:

```verilog
assign ghr_restored = resolve_is_cbr ? {chk_ghr[rollback_idx][GHL-1:1], resolve_taken}
                                     :  chk_ghr[rollback_idx];
```

- **cond-branch mispredict:** branch contributed exactly bit 0 (one CTI per bundle,
  per the §invariant) → LSB-override is **exact**.
- **JALR/return target mispredict:** jumps don't shift GHR; RAS pointer already moved
  correctly (only the predicted *target* was wrong) → restore verbatim.
- **trap / fetch-fault / data-fault replay:** no branch resolved → restore verbatim;
  the squashed bundle refetches and re-applies its own correct speculation.

So the entire new interface beyond the (already-routed) checkpoint signals is a small
resolve port, valid the cycle of `rollback`, threaded `branch_unit → exec_bundle →
backend_top → frontend → predictor`:

```
resolve_v        // a CTI resolved & redirected this cycle (eb_redirect path)
resolve_is_cbr   // conditional branch (vs jump/JALR)
resolve_taken    // resolved direction      (expose branch_unit `taken`)
resolve_is_call  // JAL/JALR rd∈{x1,x5}     ┐ RAS repair (B) + training (C)
resolve_is_ret   // JALR rd=x0 rs1∈{x1,x5}  ┘
resolve_btb_idx  // carried predict index   (training, C)
resolve_pc, resolve_target   // training (C)
```

### B3. RAS snapshot depth

The sketch snapshots **full RAS contents** per checkpoint — the faithful `chk_map`
mirror, exact, and cheap (`NCHK × RASN × target-bits` LUTRAM, same shape as
`chk_map`'s `NCHK × AREGS × PBITS`). Start here to remove a variable. The area
fallback is **pointer-only** (snapshot `chk_rptr` only, share one `ras` array) —
near-free but lets a wrong-path push/pop corrupt a live entry → an occasional extra
return mispredict. Since RAS is a pure hint, drop to pointer-only only if full
snapshot shows up in area/timing.

---

## Part C — training (at resolve)

Mirror yarvi: the predict-side computes the BTB index and carries it
(`resolve_btb_idx`), the resolve-side writes the same entry (no history
reconstruction needed). On `resolve_v`:

- `btb_type`: from the resolved class — CALL / RETURN / JUMP, or for a conditional
  branch nudge the 2-bit bimodal weight toward `resolve_taken`
  (`S_N↔W_N↔W_T↔S_T`, saturating), exactly `yarvi.v:970-976`.
- `btb_target`: resolved `target` (direct = PC+imm; indirect JALR = last actual
  target — the classic monotonic-indirect capture).
- `btb_tag`: tag of the block base PC.

BTB/bimodal writes are **not** checkpointed (resolve-time only). Phase 1 adds the
analogous YAGS write keyed by the carried `yags_idx`.

---

## Change surface

**New, self-contained in `predictor` (inside `frontend`):**
- BTB + RAS + GHR arrays; `chk_ghr/chk_ras/chk_rptr[NCHK]` snapshot/restore — a clone
  of the `chk_map` block, taps existing `create/cur/rollback/rollback_idx`.
- predict logic (A1–A2), training logic (C).

**`fetch.v`:** next-PC override (A3); export the terminating-CTI bytes/PC to the
predictor (already in the slot outputs).

**`aligner.v`:** one new output bit `cti_term` (bundle ended on a CTI).

**`branch_unit.v`:** replace `redirect/target` with the `actual_npc != pred_npc`
compare (A4); add inputs `pred_npc`, `seq_npc`; export `taken` + 2-bit call/return
class.

**Payload (`exec_pay.vh`) + threading:** add `PAY_PRED_NPC` and `PAY_BTB_IDX`
(+ predicted type/hit for the bimodal update) on the CTI's payload; thread the
`resolve_*` port `branch_unit → exec_bundle → backend_top → frontend`.

**One rule, already satisfied:** ≤1 CTI per bundle, last slot — guaranteed by
`aligner.v:111`. No new truncation.

## Verification plan

- `tb_branch*` (existing) still pass — predictor must be transparent to correctness
  (a never-hitting BTB ≡ today's fall-through).
- New microtests: a hot loop (BTB direction learns, mispredict count drops), a
  call/return chain (RAS depth, nested), and a mispredict-under-speculation that
  forces a `rollback` and checks the restored `ghr/ras` (cosim retire stream stays
  divergence-free — prediction must not change *architectural* results, only timing).
- gb5/gb6 under `make pcosim`: retire-vs-cycle ratio improves; **zero** cosim
  divergence (the hint-not-architectural property is the regression guard).
- `make timing` after integration — BTB read must terminate at a register (A1);
  watch the npc→BTB and the new `pred_npc` compare paths (FPGA margin ~zero).
