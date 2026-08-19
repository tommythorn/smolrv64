# Decoupling the frontend from memory completion

Measured on the `inorder-fb` branch, 2026-08-19, from a build **actually constrained at
6 ns** (`PROBE_CLK_DIV8=48`). Reading path counts off a 9 ns build is meaningless — Vivado
optimises to the constraint and stops, so slack-rich paths were never pushed.

## The measurement

WNS **-3.405 ns**, longest path 9.405 ns, Fmax as built **106.33 MHz**. Collapsed to
families by `worklist.tcl`:

| worst | paths | levels | logic | route | family |
|---|---|---|---|---|---|
| -3.405 | **132** | 39 | 2.533 | 5.767 | `u_lsu/FSM_onehot_st[3]` → `fe/u_bp` |
| -3.400 | ~35 (1/bit) | 26 | 2.011 | 7.147 | `u_lsu/pa2_q` → `m_result[*]` |
| -3.374 | 1 | 41 | 2.889 | 6.464 | `u_lsu/FSM_onehot_st[3]` → `fe/d_mis_taken` |

The `pa2_q → m_result` family was the device-read path, fixed by `01e1f265`, `df96e431`
and `4567b019`. The **132-path family is the subject of this document**.

## Measured progress (matched builds, same constraint and knobs)

| RTL | WNS | longest path | Fmax as built |
|---|---|---|---|
| before the device-path fixes | -3.405 ns | 9.405 ns | 106.33 MHz |
| after `01e1f265`+`df96e431`+`4567b019` | **-2.791 ns** | **8.791 ns** | **113.75 MHz** |

+0.614 ns, +7.0%, at zero IPC cost (cosim retire count identical at identical cycles).
Both LSU-startpoint families are gone from the work list. The limiter moved, it did not
shrink:

```
-2.791  1176 paths  37 levels   m_csr_func_reg[2] -> fe/u_bp
-2.770     1        23 levels   m_imm_reg[0]      -> m_result_reg[*]
```

Same shape, new feeder: backend state reaching the frontend predictor in one cycle, now via
the CSR/system-op path instead of the memory path. `fe/u_bp` is the sink of every remaining
family. Chasing feeders from here is whack-a-mole -- each new one is whatever backend
structure happens to be longest. Decouple the frontend instead.

## The cone

```
u_lsu/FSM_onehot_st[3] -> st2_go -> device write decode (is_uart_w, CARRY8 x3)
                       -> dmem_wready -> lsu_done -> redirect
                       -> u_fetch/va_q -> iMMU tag compare (CARRY8 x3)
                       -> fe/u_bp/ycorr_qv
```

`4567b019` removed the `st2_go` entry. The rest is two lines of `ino_frontend.v`:

```verilog
fetch ... u_fetch (..., .ready(accept), ...);   // accept = m_advance = f(lsu_done)
wire fire = accept & f_valid;
predictor ... u_bp (.npc(f_npc), .fire(fire), .create(create), .cur(cur), ...);
```

The frontend cannot advance without knowing whether **M completed this cycle**. The
predictor is a fetch-side structure being clocked by a backend completion signal.

Note the sink precisely: `ycorr_qv`'s D is `ycorr_v[yidx(npc, ghr)]` — an array read
indexed by fetch's `npc`. The cone therefore enters through **`fetch.ready`**, not through
`fire`. Moving `fire` alone does not cut it; the queue is required.

## The design

A **2-deep queue between `fetch` and the decode/IR stage.**

- `fetch.ready = ~q_full` — depends on a counter, not on `lsu_done`.
- IR loads from the queue head on `accept`; queue pops there.
- `redirect` flushes the queue (`redirect_q` already gates the IR).

Depth 2, not 1: with depth 1 a clean `ready` (`~q_valid`, no `accept` term) drops
throughput to one instruction per **two** cycles. Depth 2 lets the count oscillate 1↔2 so
fetch pushes every cycle while `ready` stays a registered function of the count.

### NCHK = 4 is not a constraint — it is a structure to delete

An earlier draft of this plan designed *around* the checkpoint ring: `CBITS=2`, `NCHK=4`,
so allocating at queue-push time with a 2-deep queue puts four checkpoints in flight,
exactly at capacity, and a fifth allocation would reuse a slot still needed for rollback.

That framing was wrong (2026-08-19). **The in-order core does not use checkpoints at all**,
and the planned OoO core will not either. The ring survives only inside `predictor.v`,
which restores per-checkpoint branch state on a rollback:

```verilog
end else if (rollback) begin
   ghr     <= res_rep ? {chk_ghr[rollback_idx][GHL-1:1], res_taken}
                      :  chk_ghr[rollback_idx];
   ras_ptr <= chk_rptr[rollback_idx];
   for (k = 0; k < RASN; k = k + 1) ras[k] <= chk_ras[rollback_idx][k];
```

`chk_ras` is a **full RAS copy per checkpoint**, indexed at rollback, living inside `u_bp` —
the exact module whose `ycorr_qv` is the sink of the 132-path family.

It is unnecessary here. **M is the only commit point**, so at most one instruction can be
mispredicting at any time and everything younger is squashed wholesale. That needs two
copies, not `NCHK`:

- a **committed** GHR/RAS/ptr, advanced at retire with the resolved outcome, and
- a **speculative** copy used at fetch, reloaded from committed on redirect.

`res_rep`'s existing "this resolve caused this cycle's rollback" term is already computing
the committed value; it just writes it through the ring instead of into a committed copy.

Consequences, all favourable:

1. The queue depth question disappears — no ring, nothing to overflow, so allocation can
   happen wherever is convenient and depth 2 is unconditionally safe.
2. `chk_ghr[4]`, `chk_rptr[4]` and `chk_ras[4][RASN]` collapse 4:1, removing a large
   indexed array from the critical sink's neighbourhood.
3. `create`/`cur`/`rb_idx`/`d_ckpt`/`res_ckpt`/`redirect_ckpt` and the `CBITS`/`NCHK`
   parameters lose their only consumer and can follow.

Do this **before** the queue: it shrinks the sink, deletes the coupling the queue would
otherwise have to thread, and is independently verifiable (cosim is sensitive to predictor
state through the retire count at fixed cycles, and to any rollback error architecturally).

## Cost

One extra cycle of frontend latency after a redirect. Under the "assume perfect branch
prediction" framing that is close to free; measure the real IPC cost with a matched cosim
retire count at fixed cycles, as every other step here was measured.

## Validation

Cosim is the oracle: it aborts at the first architecturally wrong instruction. A change
this shape breaks in two ways worth watching — a checkpoint-ring aliasing bug (wrong
rollback target after a mispredict) and a lost/duplicated slot at a redirect boundary.
Neither is visible in a retire *count*; both are visible to lockstep.

Baseline to match: `VDEFS=-DINO_HW=4 BUILD=1 CYC=60000000 ./run-ino-cosim-linux.sh` gives
**retires=11,082,760** with 200 pre-existing UART-LSR `MMIO-DIVERGE` lines and no others.
