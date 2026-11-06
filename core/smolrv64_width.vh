// The pipeline's widths the design ships at, in ONE place: the core, the SoC top and the benches
// include it; the build (build.tcl), the runners and the bitstream cache read IW here.
// A -DSMOLRV64_IW=<n> (or _FW, _QD) overrides one for a run or a build.
`ifndef SMOLRV64_IW
 `define SMOLRV64_IW 2
`endif
`ifndef SMOLRV64_FW
 `define SMOLRV64_FW 6                 // fetch width: instructions into the decoupling queue per cycle (IW..8)
`endif
`ifndef SMOLRV64_DRAM_LG2                // DRAM: 2^this bytes from 0x8000_0000 (the instance's; PABITS follows)
 `ifdef COSIM_MEM_SIZE_LG2
  `define SMOLRV64_DRAM_LG2 `COSIM_MEM_SIZE_LG2   // the cosim's modeled DDR
 `else
  `define SMOLRV64_DRAM_LG2 32                    // the board's 4 GiB window (src/dram_window.vh)
 `endif
`endif
// The physical-address width: the bits of the DRAM's top (33 for the board's 4 GiB). Every PA the
// core holds is this wide, as the MMUs fault any PA at or above the top (smolrv64_core).
`define SMOLRV64_PABITS ($clog2(64'h8000_0000 + (64'd1 << `SMOLRV64_DRAM_LG2)))
`ifndef SMOLRV64_QD
 `define SMOLRV64_QD 16                // the decoupling queue's depth (a power of two, at least 2*FW)
`endif
