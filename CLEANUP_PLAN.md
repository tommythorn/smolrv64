## Simplify `smolrv64.v` for bug-surface reduction

### Summary
Keep behavior and major pipeline cuts intact for now. The best simplification targets are the places where the same control decision is spelled out in multiple states. That is where the recent regressions came from, and it is also where Linux-boot bugs will be hardest to isolate later.

### Key Changes
- Factor PTW launch into one shared helper block.
  - Today the same “start Sv39 walk” sequence appears in fetch, cross-page fetch, store, and load paths.
  - Replace that duplication with one shared “PTW request” path that takes `{va, access, prv, return_state}` and emits either BRAM PTE fetch setup or DRAM PTE fetch setup.
  - Keep `S_PTW_READ` and `S_PTW_PROCESS` separate; they are useful timing cuts.

- Factor physical memory dispatch into one shared classifier.
  - Store dispatch and load dispatch repeat the same address classification for UART, CLINT, PLIC, BRAM, generic MMIO, DRAM, and illegal space.
  - Introduce a small internal region code such as `REGION_UART/CLINT/PLIC/BRAM/MMIO/DRAM/ILLEGAL`, computed once from `mem_addr`, then switch on that code in load/store handling.
  - This reduces the chance that load/store behavior drifts apart again.

- Unify load extraction/sign-extension logic.
  - The same `load_size_lg2` decode is repeated in BRAM load, DRAM first-beat load, DRAM split load, and MMIO alignment.
  - Replace that with one helper function that takes a 64-bit aligned word plus `load_size_lg2`, and a second helper for the 128-bit cross-boundary case if needed.
  - Do not change the current `load_size_lg2` encoding yet.

- Factor trap entry into one shared trap pack/commit path.
  - `S_EXCEPTION` still recomputes delegation and trap-vector selection inline.
  - Move delegation choice, `{m,s}{cause,epc,tval}` updates, and vectored `npc` formation behind one shared helper-style block.
  - Goal is not fewer features; goal is one place to audit when Linux boot hits privilege/trap edge cases.

- Simplify DRAM store control around “issue” vs “wait”.
  - The current write path is correctable but still awkward: `S_STORE`, `S_DRAM_STORE_WAIT`, `S_DRAM_STORE2`, `S_DRAM_STORE_RESP_ARM`, and `S_DRAM_STORE_RESP_WAIT` split one conceptual transaction across many ad hoc branches.
  - Refactor around an explicit two-beat store descriptor: current beat, optional second beat, and a single “write in flight drained” condition.
  - Preserve the AXI master interface shape; do not redesign the master yet.

- Leave instruction decode mostly alone for this pass.
  - `S_RF3`/`S_EXECUTE` decode is large, but it is already partially structured and is carrying timing intent.
  - The one safe follow-up there is extracting repeated “illegal instruction” trap sequences and maybe grouping the CSR/xRET/system-insn block, but not rewriting decode style before Linux boots on FPGA.

### Test Plan
- Before any refactor, keep the current baseline and require `209` riscv-tests passing.
- After each simplification slice, run only the cheapest focused regressions for the touched subsystem first:
  - PTW refactor: `rv64ui-v-ma_data`, `rv64si-p-icache-alias`, one passing physical test.
  - Region classifier/load helper refactor: one UART test path, one CLINT/PLIC path, one DRAM load/store misaligned test.
  - Trap-entry refactor: one `ecall`, one page-fault test, one interrupt path if already reproducible.
- Only after targeted checks pass, rerun the full `209` riscv-tests gate.

### Assumptions
- No performance work yet.
- No functional changes are intended; this is a control-logic consolidation pass.
- Existing timing cuts such as `S_FETCH1B`, `S_LOAD_LATCH`, `S_PTW_READ`, and `S_EXECUTE2` stay unless a later review proves one can be removed safely.
- The first refactor to do, once you are ready for edits, should be the shared PTW launch plus shared address-region classifier. Those two give the biggest reduction in duplicated bug-prone logic for the least architectural risk.
