// The core's geometry that the Linux bench reads too, defined ONCE: included INSIDE smolrv64_core
// and the bench's module (a localparam cannot name another module's, so the bench kept copies
// that drifted). smolrv64_core.v says why each is what it is.
   localparam integer RN_NINT  = 128, RN_NFL = 128, RN_NFF = 128;   // a shard's registers: a lane's, SH_FL's, SH_FF's
   localparam integer RN_IDXB  = $clog2(RN_NINT);                // a physical register: {shard, index}
   localparam integer RN_PBITS = RN_IDXB + 3;
   localparam integer NI = 20, IBI = 5;                          // a lane scheduler's entries
   localparam integer NF = 8,  IBF = 3;                          // the FP scheduler's
