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
