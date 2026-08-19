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

### The constraint that shapes it: NCHK = 4

`CBITS=2`, `NCHK=4` — four checkpoint slots, and `cur` wraps mod 4. Today at most two are
outstanding (IR + M). Allocating at queue-push time with a 2-deep queue makes it **four**,
exactly at capacity, so a new allocation can reuse a slot still needed for rollback. That
is the "slot captured now, dereferenced later" failure `docs/rtl-rules.md` is built around,
and it would corrupt silently.

**So checkpoint allocation stays at IR load** (`create`/`cur` on `accept`), leaving the
outstanding count unchanged.

### What that forces

`fire` captures fetch-time prediction state, which `create` later writes into `pdet[cur]`
(`predictor.v:160`). Once fetch runs ahead of the IR, the state captured at `fire` belongs
to a *different* instruction than the one loading into the IR. Therefore:

- `fire` moves to the queue-push handshake (fetch-side, which is where it belongs), and
- **the predictor's fetch-time capture must ride the queue** and be written at `create`.

That is the actual content of "decoupled frontend": the fetch-time prediction metadata
becomes part of the queued bundle. Queue payload:

```
inst[31:0], pc[PCW-1:0], seq[SEQW-1:0], pnpc[PCW-1:0],
fault, cause[3:0], tval[PCW-1:0],          // the fault_op pseudo-slot rides the queue too
pdet payload                                // predictor fetch-time capture
```

Decode (`decode_slot` and friends) is combinational off the queue head, so no decoded field
needs queueing — only `inst`/`seq`.

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
