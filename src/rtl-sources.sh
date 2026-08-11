# The RTL source list, in ONE place (docs/rtl-rules.md C1).
#
# Nine runner scripts each carried their own copy of this glob. Two of them additionally
# named alu.v and smolrv64_sdpram.v a second time on the verilator command line, which is
# a MODDUP the gate now rejects. A list replicated at N sites is the same generator as a
# predicate replicated at N sites: it only has to be updated wrong once.
#
# Excluded:
#   tb_*            testbenches (each runner names the one it wants)
#   *probe*         Fmax timing-probe wrappers
#   lsu_fmax.v      \
#   exec_shard_bp.v / probe-only variants: defined here, instantiated only by a *_probe.v
#                     that is itself excluded, so they are dead weight in a sim/lint build
#                     and their unconnected ports are what kept PINMISSING advisory.
#   flopwrap/rf_alu probe scaffolding
#
# Usage:  . ./rtl-sources.sh ; srcs=$(rtl_sources)
rtl_sources() {
   ls *.v | grep -vE '^tb_|probe|^flopwrap\.v$|^rf_alu\.v$|^lsu_fmax\.v$|^exec_shard_bp\.v$'
}
