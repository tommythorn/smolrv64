`ifndef EXEC_PAY_VH
`define EXEC_PAY_VH
// Execute payload packing, shared by the scheduler (carries it opaquely through
// the IQ and emits it at issue) and the execute bundle (unpacks it). PAYW=142.
//   [5:0] alu_op  [6] alu_w  [7] alu_uw  [9:8] op1_sel  [10] op2_imm
//   [11] res_link [12] is_rvc [13] is_mem  [77:14] imm[64]  [141:78] pc[64]
`define PAYW        142
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
`endif
