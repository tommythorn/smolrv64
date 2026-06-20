# Memory-subsystem extraction plan (`smolrv64_dcache`)

Plan for pulling the data cache + local memory + PTW memory-service out of
`src/smolrv64.v` into one module. Line numbers are from the state of
`smolrv64.v` when this was written and will drift; treat them as anchors.

> **Boundary corrected 2026-06-20** after reading the full cache block. The
> first draft assumed a "cache only" cut with `mem0/mem1` and the PTW left in the
> core. That is wrong: the PTW coherency probe *is* a cache operation (it drives
> the cache's own probe sub-FSM), and `mem0/mem1` has no accessor outside this
> block. The correct, lower-risk cut is the whole **memory subsystem**.

## Verdict

Worth doing. The cache FSM is already its own clocked process and owns its
registers cleanly (no multi-driver across the split). The right module is
everything downstream of address translation: cache FSM + D$ arrays + `mem0/mem1`
+ the PTW memory-service. Estimated ~1 day, dominated by the wide interface
(~40 in / ~40 out) and post-cut timing verification (zero margin today), not by
logic surgery.

## Why memory-subsystem, not "cache only"

Reading the full block (`~7935–8609`) showed two couplings that kill the
finer-grained cut:

1. **The PTW probe is a cache operation.** On a PTW direct read, the block drives
   `cache_state` into the cache's own `CACHE_PROBE_READ/WAIT/CHECK` sub-FSM to
   find-and-evict (clean) the line before reading the PTE from memory
   (`8034`, `8056`, `8266`). It cannot be "left behind" — it walks the cache.
2. **`mem0/mem1` has exactly one accessor: this block.** All 6 read/write sites
   are in the cache block (BRAM fill `8486`, BRAM writeback `8422`, PTW PTE read
   `7957`), plus the `$readmemh` init. The testbench uses a separate `axi_mem0/1`
   model; nothing reaches the core's `mem0/mem1` hierarchically. Leaving it in the
   core would force a memory arbiter and cross-module handshakes on a zero-margin
   timing design — strictly worse.

So `mem0/mem1` and the PTW memory-service belong *in* the module, with the memory
they touch. Only the PTW **walk** FSM (`S_PTW_*`, already in the main block), the
TLB, the translate/issue stage, the pipeline, and the frontend stay in the core.

## What stays clean

- The cache FSM is its own clocked block (`7935–8609`): all `cache_state <=`
  there, all main `state <= S_*` in `4123–7934`, zero overlap.
- Every register the module will own is written only from cache-side code (the
  block + its tasks). After attributing each task write to its single call site,
  no register is driven from both sides — no multi-driver violations:
  - **Module-owned → outputs:** `vhpr_epoch`, `vhpr_epoch_bump_ack`,
    `cache_flush_ack`, `dmem_rsp_*`, `dmem_write_done`, `ptw_direct_rsp_*`.
  - **Core-owned → inputs:** `vhpr_epoch_bump_req`, `cache_flush_req`,
    `cache_issue_*`, `ifetch_read`/`dmem_*`, `ptw_direct_read`/`ptw_direct_addr`.

## Module interface (`smolrv64_dcache`)

Geometry params passed from `smolrv64_defs.vh` (`CACHE_INDEX_BITS`,
`CACHE_META_BITS`, `CACHE_PERM_BITS`, `TLB_ASID_BITS`, `TLB_CTX_BITS`,
`VHPR_EPOCH_BITS`, `MEM_SIZE_LG2`, `MEM_BASEADDR`).

| Group | Dir | Signals |
|---|---|---|
| Clock/reset | in | `clock`, `reset` (`core_reset_now`) |
| Request | in | `ifetch_read`, `dmem_read`, `dmem_write`, `dmem_write_zero`, `dmem_write_data`, `dmem_write_strb`, `cache_issue_{dw_addr,va,asid,perm,ctx}` |
| Load/store response | out | `dmem_rsp_{valid,data,next_data,next_valid}`, `dmem_write_done`, `ifetch_refill_retry_valid`, `ifetch_rsp_valid`, `cache_idle` |
| PTW service | in/out | in `ptw_direct_read`, `ptw_direct_addr`; out `ptw_direct_rsp_{valid,data}` |
| CBO | in | `cache_cbo_flush`, `cache_cbo_line_addr`, `cache_cbo_ptag`; out `cache_cbo_done` |
| Flush/epoch | in/out | in `cache_flush_req`, `vhpr_epoch_bump_req`, `vhpr_epoch_bump_pending`; out `cache_flush_ack`, `vhpr_epoch_bump_ack`, `vhpr_epoch` |
| Frontend I$ fill (out) | out | `icache_fill_begin{,_idx,_way,_asid,_perm,_vtag,_ptag,_epoch}`, `icache_fill_{valid,beat,data}`, `icache_invalidate_*`, and the `icache_req_*`/`cache_req_*` the frontend consumes |
| Frontend probe results (in) | in | `icache_rsp_*`, `icache_target_*` |
| L2 line bus | in/out | `l2_fill_req_*`, `l2_fill_rsp_*`, `l2_wb_req_*`, `l2_wb_rsp_*` |
| L2 direct (PTE) bus | in/out | `l2_direct_read_req_*`, `l2_direct_read_rsp_*` |
| Quiescent (for reset home) | out | aggregate `memsys_quiescent` (folds the internal `l2_*_valid` + `ptw_direct_*` that `core_reset_home` checks today) |
| HPM | out | `hpm_vhpr_pulse` bundle |

### Moves into the module
- The `7935–8609` clocked block (cache FSM + PTW memory-service + reset).
- Tasks: `cache_flush_next_line`, `cache_start_fill_request`,
  `cache_finish_writeback_line`, `emit_dmem_load_rsp`.
- The 6 `smolrv64_sdpram` tag/bank instances + the `dcache_bank_wr` `always @*`
  + the `dcache_lookup/target/tag_hit` combinational logic.
- `mem0`/`mem1` + their init block.
- `merge_store_bytes` (used by both the block and `dcache_bank_wr`); plus the
  `smolrv64_cache_meta.vh` include (functions already shared there).
- All module-owned register declarations (`cache_*`, `dcache_*`, `vhpr_epoch*`,
  `l2_*`, `ptw_direct_*`, response latches).

### Stays in the core
- Main pipeline FSM, translate/issue stage, **PTW walk FSM** (`S_PTW_*`), TLB.
- `vhpr_request_full_flush`, `issue_ifetch_cache_read`, `issue_dmem_cache_read`
  (they drive the module's request inputs).
- `smolrv64_frontend` (wired to the module's I$-fill back-channel + `cache_req_*`).

## Procedure

1. Create `src/smolrv64_dcache.v`: params, full port list, includes, owned-reg
   declarations.
2. Move the block + tasks + sdprams + combinational logic + `mem0/mem1` + init.
3. Delete the moved code from `smolrv64.v`; instantiate the module, wiring the
   port groups; replace `core_reset_home`'s internal-signal checks with
   `memsys_quiescent`.
4. Build housekeeping: add to `src/Makefile` SRCS and `build.tcl`.
5. **Verilator first** (fast) to catch port/width/direction errors, then the gate.

## Verification

- **Functional gate (mandatory):** `(make)|& grep 'Test Passed' | wc -l` must
  still return **240**. The 109 `-v-` tests prove the PTW↔cache probe + BRAM-fill
  path survived; machine-mode `-p-` tests prove the ordinary load/store path.
- **Cosim** (tiny128 oracle) if any ordering looks subtly changed.
- **Timing:** rebuild bitstream, `make timing`. The 6 cache BRAMs + `mem0/mem1`
  crossing a module boundary add no logic, but placement can shift WNS on a
  zero-margin design (`npc→rf_decode`). Do not assume neutral.

## Non-goals (decided)

- **Do not** fold the TLB into the module — joined only by the resolved-`ptag`
  wire + `vhpr_epoch`; the PTW walk FSM that fills the TLB stays in the core.
- **Do not** drop the BRAM-PTW path to "simplify" — required by 109 of the 240
  passes (`rv64*-v-` virtual-memory tests ≈ 45% of the gate).
- **Do not** split the cache from `mem0/mem1` / the PTW memory-service in this
  pass — that finer cut needs an arbiter and adds timing risk; revisit later.
