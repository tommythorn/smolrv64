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
| after `01e1f265`+`df96e431`+`4567b019` | -2.791 ns | 8.791 ns | 113.75 MHz |
| after `953c45a3` (RAS snapshot dropped) | -2.762 ns | 8.762 ns | 114.13 MHz |
| after the F/X queue | -3.273 ns | 9.273 ns | 107.84 MHz |
| after tag-matched D$ responses | -2.660 ns | 8.660 ns | 115.47 MHz |
| after `dmem_wready` collapsed to an OR | -2.568 ns | 8.568 ns | 116.71 MHz |
| **after `raw_rvalid` collapsed to an OR** | **-2.436 ns** | **8.436 ns** | **118.54 MHz** |

The queue REGRESSED timing by 0.511 ns on its own, then tag matching recovered 0.613 ns.
Both are kept: the queue is IPC-positive and structurally right, and the regression was not
caused by the queue's premise but by what it exposed underneath -- a 64-bit address
comparator on the response path that the queue's added area pushed over the edge.

IPC across the same steps, cosim at a fixed 60M cycles:

| RTL | retires | vs baseline |
|---|---|---|
| checkpoint-ring baseline | 11,082,760 | - |
| forked predictor, details combinational | 11,443,315 | +3.25% |
| + F/X queue | 11,623,201 | +4.88% |
| + tag-matched responses | 11,631,167 | **+4.95%** |

Session total on the in-order core: **106.33 -> 118.54 MHz (+11.5%)** with **+4.95% IPC**,
i.e. about **+17% real performance**, and every step DELETED structure rather than adding it.

## One defect, five appearances

The device-vs-cache decision was re-derived from a 64-bit address at every point of use,
inside the cycle where it was needed:

| commit | what was recomputed on the critical path |
|---|---|
| `01e1f265` | device read data muxed into the load return |
| `df96e431` | three write-qualified device address muxes |
| `2a9f4c1`  | response matched by address instead of an allocated tag |
| `4747f19d` | write-ready SELECTED by a device compare |
| `f0a1c2d`  | read-valid SELECTED by a device compare |

Every one is docs/rtl-rules.md's "a precondition that applies to N units is computed once and
applied at one site", broken under a different name. Diagnostic detail: the last three were
cycle-neutral and two were BIT-IDENTICAL in cosim -- these were not design trade-offs, they
were redundant logic. The select never selected anything.

The generator-level fix, if this recurs: rv_soc_top has no single place where "is this
access a device?" is decided and named. Give it one, and the class cannot come back.

## Where the limiter is now

`m_rs1_val_reg[3] -> fe/u_bp/btb_q` (226 paths, 36 levels). The startpoint has left the LSU
for the first time:

```
m_rs1_val (CSR write source) -> u_csr mstatus/mtvec next-state (CARRY8 x3)
                             -> csr_writes1 -> ... -> fe/u_bp/btb_q
```

A CSR write in M reaching the fetch-side BTB read in the same cycle. Different coupling from
the memory one, same shape: backend state with a combinational path into the frontend.

Also newly visible: `u_icache/cur_line -> u_icache/linebuf[*]/CE` at only 11 levels but
**7.091 ns of routing** (91%). That is congestion, not logic, and no amount of restructuring
the I$'s logic will touch it -- it wants placement or a narrower fanout.

+0.614 ns, +7.0%, at zero IPC cost (cosim retire count identical at identical cycles).
Both LSU-startpoint families are gone from the work list. The limiter moved, it did not
shrink:

```
-2.791  1176 paths  37 levels   m_csr_func_reg[2] -> fe/u_bp
-2.770     1        23 levels   m_imm_reg[0]      -> m_result_reg[*]
```

Same shape, new feeder: backend state reaching the frontend predictor in one cycle, now via
the CSR/system-op path instead of the memory path.

The limiter has now moved four times without shrinking -- LSU FSM, CSR mux, LSU mem_raddr,
now `u_lsu/pa_q_reg[29] -> fe/u_bp` (226 paths, 37 levels). Every one of them ends at
`fe/u_bp`, and every one reaches it the same way: an LSU signal -> `lsu_done` -> `m_done` ->
redirect -> `u_fetch/va_q` -> the predictor. Each fix so far has shortened the FRONT of that
chain. The chain itself is what remains.

`953c45a3` (dropping the RAS snapshot) added only **+0.029 ns** on top — a rounding error
next to the device fixes' +0.614. Its real effect was on the near-critical TAIL: the
dominant family fell from **1176 paths to 276**, a 4.3x reduction, and a D$ family surfaced
underneath (`u_dcache/cur_line -> valm_reg`, 9 paths, 17 levels, 6.598 ns of it routing --
nearly pure congestion). So the congestion relief is real but it is not what sets WNS; the
critical path is the `m_csr_func -> fe/u_bp` cone, which that change does not touch. Keep it
for the IPC-free tail reduction and because it makes NCHK=8 affordable, not for Fmax. `fe/u_bp` is the sink of every remaining
family. Chasing feeders from here is whack-a-mole -- each new one is whatever backend
structure happens to be longest. Decouple the frontend instead.

## The cone

```
u_lsu/FSM_onehot_st[3] -> st2_go -> device write decode (is_uart_w, CARRY8 x3)
                       -> dmem_wready -> lsu_done -> redirect
                       -> u_fetch/va_q -> iMMU tag compare (CARRY8 x3)
                       -> fe/u_bp/ycorr_qv
```

`4567b019` removed the `st2_go` entry. The rest is two lines of `ooo2_frontend.v`:

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

### Checkpoints are gone — this section used to be the hard part

Two earlier drafts of this document were about sizing the checkpoint ring for the queue:
first "NCHK=4 is exactly at capacity", then "allocate at push, so NCHK=8, and here is why
that is affordable now". **Both are obsolete.** `ca2e9360` deleted checkpoints from the
in-order core entirely — there was never anything but the predictor consuming them.

What replaced them, and why the queue is now simple:

- `ooo2/ooo2_predictor.v` keeps two committed scalars (`ghr_c`, `rptr_c`), advanced at
  resolve and restored on redirect. No ring, no `rollback_idx`, no `NCHK`, no `CBITS`.
- Predict details ride **with the instruction** — `pd_fetch` -> `d_pdet` -> `m_pdet` ->
  `res_pdet`. The `pdet_f` overwrite hazard that forced allocation-at-push does not exist,
  because nothing is stored in a side structure keyed by a tag.

So the queue is a **plain FIFO of instruction payloads**. Depth 2 (depth 1 with a clean
`ready` halves throughput). Payload:

```
inst[31:0], pc[PCW-1:0], seq[SEQW-1:0], pnpc[PCW-1:0], pdet[PDW-1:0],
fault, cause[3:0], tval[PCW-1:0]      // the fault_op pseudo-slot rides the queue too
```

`fetch.ready` becomes `~q_full` — a registered function of the count, with no `lsu_done` in
it, which is the entire point. `fire` becomes the push handshake. `redirect` flushes.

Measured cost of getting here: -4,061 retires in 11,082,760 (-0.037%), from restoring
GHR/RAS-pointer from committed scalars rather than an exact per-instruction snapshot.
Recoverable if it matters: carry fetch-time `{ghr, ras_ptr}` with the instruction the way
`pdet` is carried (15 bits per stage) and restore `{m_ghr[GHL-2:0], res_taken}`, which is
exact.

## Cost

One extra cycle of frontend latency after a redirect. Under the "assume perfect branch
prediction" framing that is close to free; measure the real IPC cost with a matched cosim
retire count at fixed cycles, as every other step here was measured.

## Validation

Cosim is the oracle: it aborts at the first architecturally wrong instruction. A change
this shape breaks in two ways worth watching — a checkpoint-ring aliasing bug (wrong
rollback target after a mispredict) and a lost/duplicated slot at a redirect boundary.
Neither is visible in a retire *count*; both are visible to lockstep.

Baseline to match: `VDEFS=-DINO_HW=4 BUILD=1 CYC=60000000 ./run-ooo2-cosim-linux.sh` gives
**retires=11,082,760** with 200 pre-existing UART-LSR `MMIO-DIVERGE` lines and no others.
