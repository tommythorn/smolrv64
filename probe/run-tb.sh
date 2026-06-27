#!/bin/bash
# Compile+run every probe unit testbench (tb_*.v), count PASS. tb_trace is a
# trace-only harness (no PASS string) -> excluded.
set -u
cd "$(dirname "$0")"
ulimit -v $((25 * 1024 * 1024)) 2>/dev/null || true   # cap @25 GiB: runaway aborts, not OOM
srcs=$(ls *.v | grep -vE '^tb_|probe|^flopwrap.v$|^rf_alu.v')
pass=0; total=0; fails=""
for tb in tb_*.v; do
   [ "$tb" = tb_trace.v ] && continue
   [ "$tb" = tb_riscv.v ] && continue
   [ "$tb" = tb_vl.v ]    && continue   # +hex-driven riscv-test harnesses (run via
   [ "$tb" = tb_irq.v ]   && continue   # run-vl-tests.sh / a custom interrupt program)
   [ "$tb" = tb_soc.v ]   && continue   # +hex-driven SoC harness (run-soc-test.sh)
   [ "$tb" = tb_soctop.v ] && continue  # +hex-driven soc_top harness (verilated)
   [ "$tb" = tb_mon.v ]    && continue  # +monhex monitor-boot harness (verilated)
   [ "$tb" = tb_linux.v ]  && continue  # +bin-driven Linux-boot harness (verilated)
   total=$((total+1))
   if ! timeout 90 iverilog -g2012 -I. -I../src -s tb -o /tmp/tb.vvp $srcs "$tb" ../src/alu.v ../src/smolrv64_sdpram.v fp_unit_stub.sv ../src/smolrv64_plic_arbiter.v >/tmp/tb_cc.log 2>&1; then
      printf "%-22s COMPILE-FAIL\n" "$tb"; fails="$fails $tb"; continue
   fi
   out=$(timeout 60 vvp /tmp/tb.vvp 2>&1)
   if echo "$out" | grep -qiE 'ALL TESTS PASSED|ALL 65536 MATCH'; then
      pass=$((pass+1))
   else
      printf "%-22s FAIL\n" "$tb"; fails="$fails $tb"
   fi
done
echo "----"
echo "tb pass=$pass / $total"
[ -n "$fails" ] && echo "fails:$fails"
