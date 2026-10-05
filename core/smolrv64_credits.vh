// The dispatch credits (`crd`): smolrv64_core computes them from flops, smolrv64_frontend's group
// rules read them. Lane k's scheduler has room (IW bits), the head-op register, the FP scheduler,
// the LQ, the SQ, the ROB has room for slots 0..k (IW bits), then the pop, serialise, CSR and CBO
// gates. Needs IW.
localparam integer CR_IA = 0, CR_L = IW, CR_F = IW + 1, CR_LD = IW + 2, CR_ST = IW + 3,
                   CR_ROB1 = IW + 4, CR_POP = 2*IW + 4, CR_SER = 2*IW + 5, CR_CSR = 2*IW + 6,
                   CR_CBO = 2*IW + 7, CRW = 2*IW + 8;
