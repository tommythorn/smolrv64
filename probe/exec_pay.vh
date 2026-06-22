`ifndef EXEC_PAY_VH
`define EXEC_PAY_VH
// Execute payload packing, shared by the scheduler (carries it opaquely through
// the IQ and emits it at issue) and the execute bundle (unpacks it). PAYW=151.
//   [5:0] alu_op  [6] alu_w  [7] alu_uw  [9:8] op1_sel  [10] op2_imm
//   [11] res_link [12] is_rvc [13] is_mem  [77:14] imm[64]  [141:78] pc[64]
//   [142] is_branch  [143] is_jump  [146:144] br_func
//   [147] is_store  [149:148] mem_size  [150] mem_signed   (LSU)
`define PAYW        151
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
`endif
