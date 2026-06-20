`ifndef SMOLRV64_DEFS_VH
`define SMOLRV64_DEFS_VH

// Shared definitions for smolrv64 and its extracted submodules.
// Included (textually) by both smolrv64.v and the standalone module files so
// the encodings have a single source of truth.

// execute_req_alu_op: ALU operation code pre-decoded in S_RF, consumed in S_EXECUTE.
// Breaking the 50-case priority if-else exe_add path into two pipeline stages
// reduces the critical path from ~15 LUT levels to ~7 LUT levels per stage.
// The core's pre-decoded ALU op (execute_req_alu_op) now carries the 6-bit
// ALU_* codes from alu.v directly; the base RV64GC ops keep their EXOP_* names
// as aliases. OPB (pass b) / ONE (=1) are smolrv64 result-routing pseudo-ops
// outside the ALU_* range, handled in the smolrv64_alu wrapper.
`include "alu_ops.vh"
`define EXOP_ADD  `ALU_ADD
`define EXOP_SUB  `ALU_SUB
`define EXOP_SHL  `ALU_SLL
`define EXOP_SHR  `ALU_SRL
`define EXOP_SAR  `ALU_SRA
`define EXOP_XOR  `ALU_XOR
`define EXOP_OR   `ALU_OR
`define EXOP_AND  `ALU_AND
`define EXOP_LTS  `ALU_SLT
`define EXOP_LTU  `ALU_SLTU
`define EXOP_OPB  6'd48
`define EXOP_ONE  6'd49

// Cache / TLB geometry. The macro bodies reference parameters (TLB_ASID_BITS,
// CACHE_PERM_BITS, VHPR_EPOCH_BITS) that are resolved at each expansion site,
// so any module using these must declare those parameters in scope.
`ifndef CACHE_INDEX_BITS
`define CACHE_INDEX_BITS 9 // VHPR L1: 64 KiB, 2 ways, 512 64-byte lines/way
`endif
`define CACHE_WAYS      2
`define CACHE_LINES     (1 << `CACHE_INDEX_BITS)
`ifndef TLB_2M_INDEX_BITS
`define TLB_2M_INDEX_BITS 8
`endif
`ifndef TLB_4K_INDEX_BITS
`define TLB_4K_INDEX_BITS 10
`endif
`define TLB_2M_ENTRIES (1 << `TLB_2M_INDEX_BITS)
`define TLB_4K_ENTRIES (1 << `TLB_4K_INDEX_BITS)
`define TLB_ENTRIES (`TLB_2M_ENTRIES + `TLB_4K_ENTRIES)
`define CACHE_PHYS_BITS 31 // Cached physical addresses are {33'd0, cache_issue_dw_addr[27:0], 3'b000}.
`define CACHE_LINE_OFFSET_BITS 6
`define CACHE_LINE_WORDS 8
`define CACHE_PAGE_OFFSET_BITS 12
`define CACHE_COLOR_BITS 3
`define CACHE_PAGE_LINE_BITS (`CACHE_PAGE_OFFSET_BITS - `CACHE_LINE_OFFSET_BITS)
`define CACHE_PHYS_TAG_BITS (`CACHE_PHYS_BITS - `CACHE_PAGE_OFFSET_BITS)
`define CACHE_VTAG_BITS (64 - `CACHE_INDEX_BITS - `CACHE_LINE_OFFSET_BITS)
`define CACHE_PTAG_LSB 0
`define CACHE_VTAG_LSB (`CACHE_PTAG_LSB + `CACHE_PHYS_TAG_BITS)
`define CACHE_ASID_LSB (`CACHE_VTAG_LSB + `CACHE_VTAG_BITS)
`define CACHE_PERM_LSB (`CACHE_ASID_LSB + TLB_ASID_BITS)
`define CACHE_EPOCH_LSB (`CACHE_PERM_LSB + CACHE_PERM_BITS)
`define CACHE_VALID_BIT (`CACHE_EPOCH_LSB + VHPR_EPOCH_BITS)
`define CACHE_DIRTY_BIT (`CACHE_VALID_BIT + 1)
`define CACHE_META_BITS (`CACHE_DIRTY_BIT + 1)

// execute_req_mem_op: memory access class pre-decoded in S_RF (sibling of EXOP).
`define MEMOP_NONE  3'd0
`define MEMOP_LOAD  3'd1  // L{B,H,W,D}{,U}, FLW/FLD, compressed integer/FP loads
`define MEMOP_STORE 3'd2  // S{B,H,W,D}, FSW/FSD, compressed integer/FP stores
`define MEMOP_LR    3'd3  // LR.W / LR.D
`define MEMOP_SC    3'd4  // SC.W / SC.D
`define MEMOP_AMO   3'd5  // AMO*.W / AMO*.D

// Physical address region classification (phys_region()).
`define REGION_UART    3'd0
`define REGION_CLINT   3'd1
`define REGION_PLIC    3'd2
`define REGION_BRAM    3'd3
`define REGION_MMIO    3'd4
`define REGION_DRAM    3'd5
`define REGION_ILLEGAL 3'd6

`endif // SMOLRV64_DEFS_VH
