// An architectural register id, AREG_W bits: {kind[1:0], field[4:0]}. kind 00 is x0..x31, 01
// f0..f31, 10 v0..v31, 11 the hidden registers (FP-kind temporaries that cracked instructions pass
// values through). Bit 5 alone says a register lives in an FP shard (f and hidden); AR_F names the
// architectural f-registers (mstatus.FS, the lockstep's report), which the hidden ones are not.
`ifndef SMOLRV64_AREG_VH
`define SMOLRV64_AREG_VH
`define AREG_W 7
`define AR_FPK(a) (a[5])
`define AR_F(a)   (a[6:5] == 2'b01)
`define AR_X(a)   (a[6:5] == 2'b00)
`endif
