# The RTL source lists, in ONE place (docs/rtl-rules.md C1).
#
# A list replicated at N sites is the same generator as a predicate replicated at N sites:
# it only has to be updated wrong once. Two lists:
#
#   rtl_sources   every synthesizable module under src/: the blocks the core shares
#                 (fetch, aligner, decode, ALU, MMU, CSRs, FPU wrapper) and the SoC devices
#                 (CLINT, PLIC, UART, virtio, Ethernet, SD, the DDR line bridge). The unit
#                 benches (run-tb.sh) compile against this.
#   ooo2_sources  the core as it ships: ooo2/*.v (minus benches) plus the src/ modules
#                 rv_soc_top reaches. Lint's top.
#
# Usage:  . ./rtl-sources.sh ; srcs=$(ooo2_sources)
rtl_sources() {
   ls *.v | grep -vE '^tb_'
}

ooo2_sources() {
   ls ../ooo2/*.v | grep -vE '/tb_'
   for f in fetch.v aligner.v rvc_expand.v decode_slot.v decode_operands.v decode_exec.v \
            decode_fp.v exec_alu.v branch_unit.v mul3.v divider.v csr_file.v \
            mmu.v alu.v clint.v plic.v ddr_hpm.v smolrv64_sdpram.v smolrv64_plic_arbiter.v
   do echo "$f"; done
}
