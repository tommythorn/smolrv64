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

# The IN-ORDER core's source list: ooo2/*.v (minus testbenches) plus the src/ modules
# it shares with the OoO core. It lives here, next to rtl_sources(), for the same reason
# that one does -- ooo2/'s five runner scripts each carry their own copy of this list
# today, and a list replicated at N sites only has to be updated wrong once.
#
# Not simply "rtl_sources + ooo2": the OoO-only modules are not instantiated under
# rv_soc_top, and linting an uninstantiated module standalone is what kept PINMISSING
# advisory for the probe variants.
ooo2_sources() {
   ls ../ooo2/*.v | grep -vE '/tb_'
   for f in fetch.v aligner.v rvc_expand.v decode_slot.v decode_operands.v decode_exec.v \
            decode_fp.v predictor.v exec_alu.v branch_unit.v mul3.v divider.v csr_file.v \
            mmu.v alu.v clint.v plic.v ddr_hpm.v smolrv64_sdpram.v smolrv64_plic_arbiter.v
   do echo "$f"; done
}
