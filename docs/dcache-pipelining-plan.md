# D$ pipelining plan

Decision (TT, 2026-07-31): pipeline the D$ unconditionally. The present design is
architecturally indefensible for an OoO core and is upstream of every other load-path
improvement.

## Why this is level 0

`cache.v:265` accepts a request only at `S_IDLE`:

```verilog
if (st==S_IDLE && (rd_req || (wr_req && WRITABLE!=0))) begin
```

and the FSM must walk back to `S_IDLE` before taking another. Requests are therefore
**strictly serialized -- no two overlap, hit or miss**. Consequences:

* multiple outstanding loads in the LSU is meaningless: the cache accepts one request at
  a time, so tagged returns have nothing to tag;
* hit-under-miss is a *subset* of pipelining -- a pipelined cache with miss handling on a
  side path gets it for free;
* PTW reads go through the D$ (`soc_top.v:518` `dcr_*`), and although the LSU has top mux
  priority, a walk already accepted by the FSM blocks every later load until it retires.

## Measured baseline (200M-cycle Ubuntu boot, IW=1)

| metric | value | source |
| --- | --- | --- |
| D$ read return, mode | 2c, 82.1% of reads | `LSU-RD` (0a6e991) |
| D$ read return, tail | 17+c, 12.1%, ~40c avg | `LSU-RD` |
| D$ read return, mean | 6.72c | `LSU-RD` |
| MERGE waiting on memory | 57.65M cyc = 28.8% of all cycles | `LSU-LQ` (74c9e9a) |
| loads blocked by MERGE busy | 7.23M cyc = 71% of selection stalls | `LSU-LQ` |
| D$ read hit rate | 99.16% | perftool |

The 2c hit latency is already the structural floor (registered, address-matched response
off a synchronous array) and is **not** the target. The target is **throughput** and the
12.1% tail: with a 0.84% read miss rate, ~11 of those 12 points are loads blocked behind
*someone else's* fill or walk.

## Target shape

Accept one request per cycle:

```
stage A   data + tag array read (way0/way1), request identity captured
stage B   tag compare -> way mux -> align -> rd_valid + rd_resp_addr
miss      divert to the EXISTING fill machinery as a SIDE path; later hits continue
```

Per-request identity must survive the pipe: `soc_top.v:354` discards a non-matching
response (`dc_rv_ok = dc_rd_valid & (dc_rd_resp_addr == dmem_raddr)`) so a squashed load's
line is never delivered to the next load. Pipelining must keep that property.

## Hard cases

1. **Hit to a line currently being filled.** Either stall that request only, or forward
   from the fill buffer. Start with stall-that-one (correct, simple); measure before
   adding forwarding.
2. **Write ordering.** `WRITABLE` (D$) takes `wr_req`; a write and a later read to the
   same line must not reorder. The store buffer already drains in seqno order, so the
   cache only has to preserve its own accept order.
3. **Write-back path.** A dirty eviction (`S_WB..S_WBA`) must not block subsequent hits --
   it is the same side-path problem as fill. A write-back buffer may be needed.
4. **Prefetch interlocks.** The prefetch engine borrows the L2 port in parallel and has
   documented interlocks with `S_FILL` (`cache.v:193-208`). Preserve them; the I$ depends
   on this for its 2.4% fill rate.
5. **CBO / flush / NC / zero ops.** These are rare, serializing, and correctness-critical.
   Keep them on the slow path: drain the pipe, run the FSM as today, resume.
6. **Shared module.** `cache.v` is instantiated as both I$ and D$ with different
   `WRITABLE`/`WRTHRU`/`PREFETCH`/`RDW`. The pipeline must be parameter-clean for both, or
   be gated so the I$ keeps today's behaviour until separately validated.

## Baseline: what actually blocks a pending read

`CACHE-BLK`, same 200M-cycle boot. Cycles with a read request pending while the FSM is
busy, split by what is holding it:

| | total | fill | write-back | lookup | other |
| --- | --- | --- | --- | --- | --- |
| **I$** (id=0) | 36.42M | 1.41M (3.9%) | 0.10M (0.3%) | **28.87M (79.3%)** | 6.04M (16.6%) |
| **D$** (id=1) | 50.15M | 15.71M (31.3%) | **20.58M (41.0%)** | 13.85M (27.6%) | 0.01M |

Two results that reorder the work:

* **D$: write-back blocks more than fill.** Dirty evictions hold the FSM for 20.58M cycles
  against fill's 15.71M. A write-back buffer is not a step-4 afterthought -- it is joint
  first with hit-under-miss.
* **I$: lookup serialization is 79.3%, 28.87M cycles = 14.4% of the whole run.** The I$
  cannot accept back-to-back fetches. This is a prime suspect for the `fetch-bubble`
  (17.3% of all cycles) previously attributed to fetch throughput / no BP -- see
  `project_cpi_breakdown`. Pipelining the shared read path may fix the frontend and the
  load path together, which makes step 2 the highest-value step for BOTH caches.

## Staging

1. Baseline captured (above).
2. Pipeline the **read hit path only**; every miss/write/maintenance op drains the pipe and
   uses today's FSM unchanged. Removes back-to-back hit serialization: 79.3% of the I$'s
   blocking and 27.6% of the D$'s. Do this first -- it is the shared win and the smallest
   change.
3. Write-back buffer (D$ 41.0%) and fill-as-side-path / hit-under-miss (D$ 31.3%). Joint
   second; write-back is marginally the larger. The 12.1% `LSU-RD` tail is waiting on both.
4. Re-measure `CACHE-BLK` + `LSU-RD`; only then revisit LSU multiple-outstanding (#2a),
   which needs step 2-3 to mean anything.
5. Separately re-measure the frontend after step 2 -- if `fetch-bubble` drops, the I$ FSM
   was the cause and the branch-predictor work is deprioritized accordingly.

## Step 2 results (f371fa1 + a20a719, 2026-07-31)

The read port is now a ready/valid request channel (`rd_req`/`rd_rdy`) + address-tagged
response; all clients (soc I$ adapter, LSU `c_infl`, PTW `pw_issued` arbiter, tb_vl,
tb_cache) handshake and stop presenting after the grant. Hits stream 1/cycle via
`pipe_take` (same-address b2b legal). Same 200M-cycle boot, before -> after:

| `LSU-RD` | IW=1 before | IW=1 after | IW=2 before | IW=2 after |
| --- | --- | --- | --- | --- |
| 2c mode | 82.1% | 83.0% | 71.8% | 73.4% |
| 3-4c | 5.1% | 4.2% | 21.4% | 19.8% |
| 17+c tail | 12.07% | 12.07% | 5.87% | 5.86% |
| mean | 6.72c | 6.71c | 4.20c | 4.19c |

The 3-4c shoulder (lookup serialization) moved into the 2c mode; b2b-accepts = 136k
(IW=1) / 210k (IW=2), mostly PTE-under-load + squash reissue; I$ b2b = 0 (frontend
still a single-outstanding client). The 17+c tail did not move a single point -- it is
fill/write-back occupancy, step 3's target, and owns ~72% of the IW=1 mean (~40c/tail
load). NOTE: `CACHE-BLK` semantics changed at a20a719 -- clients no longer hold rd_req
during their own service, so the counter now measures only genuine cross-blocking
(D$ IW=2: 5.2M cyc; fill 0.55M, wb 0.48M, lookup 4.14M). Not comparable to the
baseline table above.

## Step 3a results: write-back buffer (2026-07-31)

Dirty victims capture into a single-entry buffer (`wbb_*`); the demand fill no longer
waits for the L2 write, which drains in the background on idle port cycles. A miss on
the buffered line bounces it back in as dirty (no L2 trip); NC fills and CBO ops
targeting it drain it first; flush holds `inv_busy` until the buffer is empty. Same
200M-cycle boot, IW=1, vs the step-2 numbers: **reads completed 8.62M -> 9.47M (+9.9%
progress in the same window), `LSU-RD` mean 6.71 -> 5.59c**, tail ~40c -> ~33.5c per
tail load, `CACHE-BLK` D$ writeback 9.78M -> 4.41M cyc. Remaining for step 3b
(fill-as-side-path / hit-under-miss): fill is now the largest D$ blocker (7.51M cyc),
and the 8-cycle victim capture is still on the miss path (could overlap the fill's
L2 read with an ack catcher).

## Step 3b results: fill side-path / hit-under-miss (2026-07-31)

Single MSHR (D$ WB config): a plain cacheable non-span miss moves its context to
`msh_*` and FREES the FSM -- hits and stores are served under the outstanding miss
(the read-hit pipe keeps streaming) while parallel launcher/catcher engines run the
L2 read. The install preempts at the next idle or parked-S_CHECK boundary: victim
capture (deferred to install, so under-miss stores to the victim are included) then
`S_MSHI`, which delivers straight from the caught line (window cut combinationally,
store bytes merged at the catch -- no re-lookup) and streams it into the banks.
Serializing misses (CBO/NC/span/wbb-bounce) and second misses park at S_CHECK;
a parked op resumes via S_LOOK and usually hits the just-installed line.

Same 200M-cycle boot, IW=1 (baseline -> step 2 -> 3a -> 3b):

| | base | step 2 | 3a | **3b** |
| --- | --- | --- | --- | --- |
| `LSU-RD` mean | 6.72c | 6.71c | 5.59c | **3.60c** |
| 2c mode | 82.1% | 83.0% | 83.0% | **87.9%** |
| 17+c tail | 12.07% | 12.07% | 11.05% | **5.71%** (~27.6c/load) |
| reads completed | 8.62M | 8.62M | 9.47M | **9.70M** |
| D$ blocked cyc (fill/wb) | -- | 7.46M/9.78M | 7.51M/4.41M | **0.21M/0.09M** |

Cosim: 200M-cycle Linux boot lockstep vs simmerv, 0 divergences. Remaining ideas:
overlap the 8-cycle victim capture with the fill's L2 read (ack catcher already
exists -- capture could run during msh_infl), multi-entry MSHR with LSU
multi-outstanding (step 4), and the I$ (HUM is D$-only; the frontend is still a
single-outstanding client).

## Step 4 results: LSU multi-outstanding, depth 2 (2026-07-31)

Head (`p_*`) + shadow (`s_*`) in-flight slots: responses are matched per entry by PA
(`mem_resp_addr`) and captured raw; writeback stays in issue order (shadow promotes on
head retire); response ARRIVAL order is free (a hit returns under a miss in the MSHR).
New `mem_rdy`/`mem_resp_addr` ports; the soc/tb adapters shrink to a 1-deep issue skid +
raw address-tagged response forwarding (`dc_rv_ok`/pend/sticky machinery deleted -- a
squashed load's response matches no live entry). The drain interlock, atomics and device
loads gate on both slots. Two bugs found on the way: (1) `mem_rdy` must drop during a
live-but-ungranted request cycle or the second issue is silently lost; (2) the AMO RMW
result (`a_resv`) consumed LIVE `mem_rdata` at A_WR -- legal when adapters held rdata
stable, but under raw forwarding a PTW PTE response landing between A_RD and A_WR became
the RMW operand (cosim caught an `amoor` writing `ppn|flags` into a kernel stack slot at
8.75M retires). Fixed by capturing at `aresp` (`a_memq`). The `LSU-RD` histogram is now
per-entry (the old single tracker mismeasured under overlap; scale moved +1: it now
counts from the issue edge).

200M-cycle boot, IW=1, vs 3b: loads completed 9.70M -> 9.75M, mean ~3.6c (unchanged --
one ready load at a time at IW=1; the win is capacity), pipe-full stalls
(`merge_busy`) 8.08M -> 1.33M cyc, ready-but-unselected 12.0M -> 5.44M, in-MERGE
mem_wait 34.8M -> 29.9M, D$ b2b-accepts 188k -> 1.81M (loads now stream through the
pipelined cache back-to-back). Cosim: 200M cycles lockstep vs simmerv, 0 divergences.
Re-measure at IW=2/4 where multiple ready loads exist; frontend streaming (I$) remains
the open client-side item.

## Validation

* `run-vl-tests.sh` (215/215) after every step -- covers the LSU/D$ through `backend_top`.
* Full `make` riscv-tests (240 "Test Passed") before any commit that changes cache
  behaviour.
* 200M-cycle Ubuntu boot with `-DCACHE_BLOCK_STATS -DLSU_LQ_STATS`; compare `LSU-RD` mean
  and the 17+c bucket against the baseline above.
* NOTE `project_sdpram_behavioral_vs_xpm_blindspot`: the behavioral sdpram hides memory
  width/latency bugs that only appear in synthesis. A cache change that is clean in
  Verilator can still break the bitstream -- keep bank widths untouched, and treat an FPGA
  build as part of the acceptance for this work, not an afterthought.
