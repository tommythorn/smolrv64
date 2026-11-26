// An architectural register id, AREG_W bits: {kind[1:0], field[4:0]}. kind 00 is x0..x31, 01
// f0..f31, 10 v0..v31, 11 the hidden registers that cracked instructions pass values through:
// 96..103 FP-kind, 104..111 integer-kind (AR_XH), 112..127 the VL file's entries (named, never
// renamed). AR_FPK says a register lives in an FP shard (f and the FP-kind hidden ones); AR_F names
// the architectural f-registers (mstatus.FS, the lockstep's report), which the hidden ones are not.
`ifndef SMOLRV64_AREG_VH
`define SMOLRV64_AREG_VH
`define AREG_W 7
`define AR_XH(a)  (a[6:3] == 4'b1101)
`define AR_FPK(a) (a[5] & ~`AR_XH(a))
`define AR_F(a)   (a[6:5] == 2'b01)
`define AR_X(a)   (a[6:5] == 2'b00)
`endif
