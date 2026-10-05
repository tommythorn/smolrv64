// The physical register file's shards. A physical register is {shard[2:0], index}: the lanes'
// integer shards are 0..3 (lane k's is k), the FP file's slices 4..7 (slot k's is 4 + k), so a
// register is an f-register exactly when shard[2] is set. A width-IW core uses lanes 0..IW-1 and
// slices 4..4+IW-1.
localparam [2:0] SH_IE = 3'd0, SH_IE2 = 3'd1, SH_IE3 = 3'd2,
                 SH_F0 = 3'd4, SH_F1 = 3'd5, SH_F2 = 3'd6;
