`ifndef EXEC_PAY_VH
`define EXEC_PAY_VH
// Execute payload packing, shared by the scheduler (carries it opaquely through
// the IQ and emits it at issue) and the execute bundle (unpacks it). PAYW=157.
//   [5:0] alu_op  [6] alu_w  [7] alu_uw  [9:8] op1_sel  [10] op2_imm
//   [11] res_link [12] is_rvc [13] is_mem  [77:14] imm[64]  [141:78] pc[64]
//   [142] is_branch  [143] is_jump  [146:144] br_func
//   [147] is_store  [149:148] mem_size  [150] mem_signed   (LSU)
//   [151] is_mul    (M ext; the op is {alu_w, br_func} = {is_w, funct3})
//   [152] is_csr  [155:153] csr_func  [156] is_serialize    (SYSTEM/FENCE)
//   [157] is_amo  [162:158] amo_func (funct5)               (A extension)
//   [163] illegal  (raises an illegal-instruction trap, cause 2)
// For SYSTEM ops imm carries {.., zimm=imm[16:12], csr_addr=imm[11:0]} (the
// instruction's rs1 field + insn[31:20]); decode_operands packs it there.
`define PAYW        164
`define PAY_ALUOP   5:0
`define PAY_W       6
`define PAY_UW      7
`define PAY_O1S     9:8
`define PAY_O2I     10
`define PAY_LINK    11
`define PAY_RVC     12
`define PAY_MEM     13
`define PAY_IMM     77:14
`define PAY_PC      141:78
`define PAY_BR      142
`define PAY_JMP     143
`define PAY_BRFUNC  146:144
`define PAY_STORE   147
`define PAY_MSIZE   149:148
`define PAY_MSGN    150
`define PAY_MUL     151
`define PAY_CSR     152
`define PAY_CSRF    155:153
`define PAY_SER     156
`define PAY_AMO     157
`define PAY_AMOF    162:158
`define PAY_ILL     163
`endif
