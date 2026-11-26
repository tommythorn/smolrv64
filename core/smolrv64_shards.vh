// The physical register file's shards. A physical register is {shard[2:0], index}: the lanes'
// integer shards are 0..3 (lane k's is k), the FP file's two 4 and 5, one per writer (SH_FL the
// load landing's, SH_FF the F stage's), the vector file 6 (SH_V, the vector unit's) and the VL file
// 7 (SH_VL: vl by version, written by the F stage's VL uops, never renamed). shard[2] is the F
// side: a register a lane never reads. A width-IW core uses lanes 0..IW-1.
localparam [2:0] SH_IE = 3'd0, SH_IE2 = 3'd1, SH_IE3 = 3'd2,
                 SH_FL = 3'd4, SH_FF = 3'd5, SH_V = 3'd6, SH_VL = 3'd7;
