`ifndef ALU_OPS_VH
`define ALU_OPS_VH

// RVA22+Zicond ALU operation selector (see alu.v). Grouped by functional unit;
// `w` (word) and `uw` (zero-extend op1 from 32 bits) are orthogonal modifiers.
// Shared by alu.v and the smolrv64_alu wrapper.
`define ALU_ADD    6'd0
`define ALU_SUB    6'd1
`define ALU_SH1ADD 6'd2
`define ALU_SH2ADD 6'd3
`define ALU_SH3ADD 6'd4

`define ALU_SLT    6'd8
`define ALU_SLTU   6'd9
`define ALU_MIN    6'd10
`define ALU_MINU   6'd11
`define ALU_MAX    6'd12
`define ALU_MAXU   6'd13

`define ALU_SLL    6'd16
`define ALU_SRL    6'd17
`define ALU_SRA    6'd18
`define ALU_ROL    6'd19
`define ALU_ROR    6'd20
`define ALU_BEXT   6'd21

`define ALU_AND    6'd24
`define ALU_OR     6'd25
`define ALU_XOR    6'd26
`define ALU_ANDN   6'd27
`define ALU_ORN    6'd28
`define ALU_XNOR   6'd29
`define ALU_BCLR   6'd30
`define ALU_BSET   6'd31
`define ALU_BINV   6'd32

`define ALU_CZEQZ  6'd33   // Zicond czero.eqz (RVA23): rd = (rs2==0) ? 0 : rs1
`define ALU_CZNEZ  6'd34   // Zicond czero.nez (RVA23): rd = (rs2!=0) ? 0 : rs1

`define ALU_CLZ    6'd40
`define ALU_CTZ    6'd41
`define ALU_CPOP   6'd42
`define ALU_REV8   6'd43
`define ALU_ORCB   6'd44
`define ALU_SEXTB  6'd45
`define ALU_SEXTH  6'd46
`define ALU_ZEXTH  6'd47

`endif // ALU_OPS_VH
