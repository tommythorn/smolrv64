// The widths of the tags that cross the core's boundary, the one definition every module and
// bench carrying them uses.
//   SMOLRV64_ICTAG_W  the I$ request tag (smolrv64_fring builds it: the pair's marks, its end, its
//                     skip and the stream generation, and asserts this matches)
//   SMOLRV64_LQB      the load queue's index width (1 << it entries), which a fast load's D$ read
//                     tag carries
//   SMOLRV64_ROB_ROWS the ROB's rows of IW entries (a power of two), which a ROB tag's row field
//                     indexes and the Linux bench's recovery counter walks
`ifndef SMOLRV64_TAGS_VH
`define SMOLRV64_TAGS_VH
`define SMOLRV64_ICTAG_W 17
`ifndef SMOLRV64_LQB
 `define SMOLRV64_LQB 4
`endif
`ifndef SMOLRV64_SQB
 `define SMOLRV64_SQB 5                 // the store queue's index width (1 << it entries)
`endif
`ifndef SMOLRV64_ROB_ROWS
 `define SMOLRV64_ROB_ROWS 64          // the ROB's rows of IW entries (a power of two)
`endif
`endif
