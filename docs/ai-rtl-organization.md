# AI-Efficient RTL Organization

`src/smolrv64.v` is workable for AI-assisted review because it has useful
comments, but it is not ideal. The 11k-line monolith forces an assistant to
repeatedly rediscover module boundaries, keep unrelated state in context, and
reason across frontend, core, cache, MMIO, TLB, AXI, and testbench logic at the
same time.

The recommendations below are intended to improve AI and human navigation while
preserving behavior.

## Highest-Value Changes

1. Split by hardware boundary.

   Move already distinct blocks into separate files:

   - `smolrv64_core.v`: architectural FSM, decode, execute, traps
   - `smolrv64_frontend.v`: fetch buffer, prediction, I-cache lookup
   - `smolrv64_l1.v`: D-cache/VHPR state machine
   - `smolrv64_tlb_ptw.v`: TLB arrays and Sv39 page table walk
   - `smolrv64_platform.v`: UART, CLINT, PLIC, and address map
   - `smolrv64_mem_engine.v`: L2 boundary and AXI engine
   - `smolrv64_regfile.v`: integer and FP register files
   - `smolrv64_ram.v`: SRAM/RAM wrappers
   - `smolrv64_tb.v`: simulation harness

2. Add a top-level RTL map.

   Add `docs/rtl-map.md` with:

   - file/module purpose
   - key state machines
   - ownership of each bus and signal group
   - instruction lifecycle from fetch to retire
   - load/store lifecycle from virtual address to local device, BRAM, or AXI

3. Narrow shared state ownership.

   Many tasks mutate broad shared globals such as `state`, `mem_addr`, `cause`,
   `tval`, and frontend flags. Prefer narrower request/result bundles where
   practical:

   - fetch request/result
   - decode/execute request/result
   - translation request/result
   - cache request/result
   - trap request

   Plain Verilog can still do this with named packed vectors and helper
   functions; SystemVerilog packed structs would be clearer if acceptable.

4. Separate decode tables from control flow.

   Move dense instruction pattern matching into named predicates or decode
   helpers such as `is_addi`, `is_c_lw`, `is_fmadd`, `decode_alu_op`, and
   `decode_mem_op`. This makes "where is instruction X implemented?" a targeted
   search instead of a scan through a giant `if`/`else` chain.

5. Document state-machine transitions.

   The comments near the state definitions are useful. Add compact transition
   summaries for major states, for example:

   ```text
   S_EXECUTE:
     MEMOP_LOAD  -> S_LOAD_ALIGN
     MEMOP_STORE -> S_STORE
     CSR         -> S_HANDLE_CSR
     ALU         -> S_EXECUTE2
     illegal     -> S_EXCEPTION
   ```

6. Keep generated output out of search paths.

   Add or maintain `.ignore`/`.rgignore` entries for generated Verilator and
   build output such as `src/obj_dir_*`. This directly improves `rg` results and
   reduces irrelevant context for AI tools.

7. Add explicit invariants.

   The existing simulation checks are valuable. Additional assertions or notes
   should cover:

   - one-cycle ownership of frontend command mutation
   - `execute_req_valid` payload stability
   - TLB insert preconditions
   - cache state ownership of L2 requests
   - trap/redirect squashing of younger frontend and decode state

## Pragmatic First Step

Start by splitting out self-contained helper modules and the testbench, then add
`docs/rtl-map.md`. That reduces context load immediately without touching the
timing-sensitive core FSM.

