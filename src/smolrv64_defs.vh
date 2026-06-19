`ifndef SMOLRV64_DEFS_VH
`define SMOLRV64_DEFS_VH

// Shared definitions for smolrv64 and its extracted submodules.
// Included (textually) by both smolrv64.v and the standalone module files so
// the encodings have a single source of truth.

// execute_req_alu_op: ALU operation code pre-decoded in S_RF, consumed in S_EXECUTE.
// Breaking the 50-case priority if-else exe_add path into two pipeline stages
// reduces the critical path from ~15 LUT levels to ~7 LUT levels per stage.
`define EXOP_ADD  4'd0   // exe_add = execute_req_rs1_value + execute_req_alu_b  (execute_req_rs1_value[31:0]+b[31:0] if sxt)
`define EXOP_SUB  4'd1   // exe_add = execute_req_rs1_value - execute_req_alu_b
`define EXOP_SHL  4'd2   // exe_add = execute_req_rs1_value << b[5:0]    (execute_req_rs1_value[31:0]<<b[4:0] if sxt)
`define EXOP_SHR  4'd3   // exe_add = execute_req_rs1_value >> b[5:0]
`define EXOP_SAR  4'd4   // exe_add = $signed(execute_req_rs1_value) >>> b[5:0]
`define EXOP_XOR  4'd5   // exe_add = execute_req_rs1_value ^ b
`define EXOP_OR   4'd6   // exe_add = execute_req_rs1_value | b
`define EXOP_AND  4'd7   // exe_add = execute_req_rs1_value & b
`define EXOP_LTS  4'd8   // exe_add = ($signed(execute_req_rs1_value) < $signed(b)) ? 1 : 0
`define EXOP_LTU  4'd9   // exe_add = (execute_req_rs1_value < b) ? 1 : 0
`define EXOP_OPB  4'd10  // exe_add = b               (LUI, AUIPC, JAL link, MV, LI)
`define EXOP_ONE  4'd11  // exe_add = 1               (SC.W/D fail)

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

`endif // SMOLRV64_DEFS_VH
