# Cache extraction plan (`smolrv64_dcache`)

Plan for pulling the data cache out of `src/smolrv64.v` into its own module.
Line numbers are from the state of `smolrv64.v` when this was written and will
drift; treat them as anchors, not addresses.

## Verdict

Worth doing. The hard part of FSM extraction — untangling interleaved state and
fixing multi-driver registers — is already absent. The cache is its own clocked
process and owns its registers cleanly. The job is mostly cut/paste/wire plus
resolving one code-location coupling (the PTW direct-read bus). Estimated ~1 day,
dominated by care, not by surgery. Main residual risk is FPGA timing (zero margin
today), verified after the fact.

## Architecture as found

The "unified cache" is really **three independent units sharing memory ports**:

1. **Main pipeline FSM** (`state`) — clocked block at `~4123–7934`. Owns the
   translate/issue stage (produces the resolved request) and the page-table
   walker (`S_PTW_*`).
2. **TLB + PTW** — TLB arrays (`tlb_2m_ram`/`tlb_4k_ram`), looked up in the issue
   stage, filled by the PTW. The PTW reads PTEs **straight from backing memory**,
   never through the cache arrays; coherency is kept by a probe-then-writeback of
   the cache before each direct read (`ptw_direct_probe_pending` →
   `ptw_direct_wait_probe` → `cache_cbo_done_r`). Stays in the core.
3. **Data cache** — clocked block at `~7935–8609`. Its own FSM, tag/bank BRAMs,
   lookup/target combinational logic. **This is what we extract.**

Two things people assume are part of the cache but are not:
- **The I$ is already extracted** into `smolrv64_frontend`. The cache block only
  *drives the fill* of the frontend's I$ via a back-channel
  (`icache_fill_*`, `icache_invalidate_*`); it reads back `icache_rsp_*`/
  `icache_target_*`. There is no separate I$ to pull.
- **`mem0`/`mem1` is the on-chip local RAM**, not cache storage. The cache caches
  it (`cache_fill_from_bram`/`cache_wb_to_bram`) and the PTW can walk page tables
  in it (`ptw_direct_from_bram`, required by the 109 `rv64*-v-` riscv-tests — do
  **not** drop this). It stays in the core.

### Why the split is clean

- All 36 inline `cache_state <=` live in the `7935–8609` block; **zero** in the
  main block. All 154 inline `state <= S_*` live in the main block; **zero** in
  the cache block. The two FSMs do not share a clocked process.
- Every cache-owned register is written only from cache-side code (the cache
  block + 4 cache tasks): `dmem_rsp_*_r`, `dmem_write_done_r`, `vhpr_epoch`,
  `cache_replace_way`, the bank/tag write regs. No cross-block writers → no
  multi-driver violations when the block becomes a module.

## Module: `smolrv64_dcache`

Parameters mirror the existing macro-driven geometry (`CACHE_INDEX_BITS`,
`CACHE_META_BITS`, `CACHE_PERM_BITS`, `TLB_ASID_BITS`, `TLB_CTX_BITS`,
`VHPR_EPOCH_BITS`), passed as params so `smolrv64_defs.vh` stays the source of
truth.

| Group | Dir | Signals |
|---|---|---|
| Clock/reset | in | `clock`, `reset` (`core_reset_now`) |
| Request (from issue stage) | in | `ifetch_read`, `dmem_read`, `dmem_write`, `cache_issue_{va,asid,perm,ctx}`, resolved `ptag`, store data/strb, CBO/`zero` flags |
| Response (to main FSM) | out | `dmem_rsp_{valid,data,next_data,next_valid}`, `dmem_write_done`, `cache_idle`, `cache_cbo_done`, `ifetch_rsp_valid` |
| I$ fill back-channel (to frontend) | out | `icache_fill_begin{,_idx,_way,_asid,_perm,_vtag,_ptag,_epoch}`, `icache_fill_{valid,beat,data}`, `icache_invalidate_*` |
| I$ probe results (from frontend) | in | `icache_rsp_*`, `icache_target_*` |
| L2 line bus (to mem layer) | both | `l2_fill_req_*`, `l2_fill_rsp_*`, `l2_wb_req_*`, `l2_wb_rsp_*` |
| BRAM fast-path (to `mem0/mem1`) | both | bank read addr/data + write addr/en/data port pair |
| Invalidation / epoch | out | `vhpr_epoch` (TLB consumes), flush req/ack |
| Cache probe service (for PTW) | both | `cache_probe_*` request + `cache_cbo_done` ack |
| HPM | out | `icache_*`/`dcache_*` pulse bundle |

### Moves into the module
- The `7935–8609` clocked block (the cache FSM).
- The 4 cache tasks: `cache_flush_next_line`, `cache_start_fill_request`,
  `cache_finish_writeback_line`, `vhpr_request_full_flush`.
- The 6 `smolrv64_sdpram` tag/bank instances + the `dcache_bank_wr` `always @*`.
- The `dcache_lookup/target/tag_hit` combinational logic + `smolrv64_cache_meta.vh`.
- The cache-side HPM pulse generation.

### Stays in the core
- Main pipeline FSM, translate/issue stage, TLB + PTW.
- `mem0`/`mem1` and the AXI/L2 arbitration (the memory-access layer).
- `smolrv64_frontend` (already its own module; just gets wired to the new module).

## The one knot: the PTW direct-read bus

The PTW's direct-read code (`l2_direct_read_*` driving + `ptw_direct_wait_bram/axi`
handling, ~7 writes) currently sits **physically inside the cache block** — a
code-location coupling, not a datapath one (PTEs never flow through the cache).

Resolution: keep `mem0/mem1` + AXI arbitration in the core as the **memory-access
layer**, with two independent clients:
- `smolrv64_dcache` → issues `l2_fill_*` / `l2_wb_*` line transfers.
- the PTW (in the core) → issues `ptw_direct_*` reads.

So when lifting the cache block, **leave the `ptw_direct_*` lines behind** with the
PTW; only the cache's own line-bus and BRAM-fill logic moves. The cache's probe
service (used by the PTW for coherency) is exposed as a port pair
(`cache_probe_*` in, `cache_cbo_done` out) rather than an internal coupling.

Rejected alternative: making the cache a "memory hub" the PTW routes through —
over-engineered; the datapaths are already independent.

## Procedure

1. **Carve the interface on paper first** — list every signal the `7935–8609`
   block reads (→ inputs) and every reg/wire it writes that something else reads
   (→ outputs). The buckets above are the starting point; reconcile against the
   actual block.
2. **Split the PTW lines out** of the cache block back to the PTW/core side, so
   the block to be lifted is cache-only.
3. **Create `src/smolrv64_dcache.v`**: params, ports, move the block + tasks +
   sdpram instances + combinational logic + `cache_meta` include.
4. **Instantiate in `smolrv64.v`**, wiring the port groups. The frontend's
   `icache_*` ports now connect to the new module instead of inline signals.
5. **Build housekeeping**: add `smolrv64_dcache.v` to `src/Makefile` SRCS and to
   `platforms/rk-xcku5p-f-v1.2/build.tcl`.

## Verification

- **Functional gate (mandatory):** `(make)|& grep 'Test Passed' | wc -l` must
  still return **240**. The 109 `-v-` tests are the proof the PTW↔cache probe and
  the BRAM-fill path survived the cut intact.
- **Cosim** if any subtle ordering changed (tiny128 oracle).
- **Timing:** rebuild bitstream and check WNS. The 6 cache BRAMs crossing a module
  boundary add no logic, but placement can shift on a design with ~zero margin
  (`npc→rf_decode`). Run `make timing` after; do not assume neutral.

## Non-goals (decided, do not revisit here)

- **Do not** fold the TLB into the cache — connected only by the resolved-`ptag`
  wire + `vhpr_epoch`; the PTW that fills the TLB lives in the main FSM.
- **Do not** drop the BRAM-PTW path to "simplify" — it costs 109 of the 240 passes
  for a ~20-line trim off a non-critical path.
