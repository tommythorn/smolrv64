#!/bin/bash
# Compile+run every unit testbench under src/ (tb_*.v), count PASS. These cover the blocks
# the core shares (aligner, fetch, decode, ALU, MMU, mul/div, FPU wrapper) and the SoC
# devices (CLINT, PLIC, DDR line bridge, virtio-net). The core's own benches live in
# ooo2/run-ooo2-*-tb.sh. Rule G4: run this after any port change to a shared module.
set -u
cd "$(dirname "$0")"
ulimit -v $((25 * 1024 * 1024)) 2>/dev/null || true   # cap @25 GiB: runaway aborts, not OOM
srcs=$(. ./rtl-sources.sh; rtl_sources)
pass=0; total=0; fails=""
for tb in tb_*.v; do
   # tb_virtio_net declares ring_mask after its first use, which iverilog rejects; it runs
   # under Verilator in ooo2/run-ooo2-vnet-tb.sh.
   [ "$tb" = tb_virtio_net.v ] && continue
   total=$((total+1))
   top=$(grep -m1 -oE '^module +[A-Za-z_0-9]+' "$tb" | awk '{print $2}')
   if ! timeout 90 iverilog -g2012 -I. -s "$top" -o /tmp/tb.vvp $srcs "$tb" fp_unit_stub.sv >/tmp/tb_cc.log 2>&1; then
      printf "%-22s COMPILE-FAIL\n" "$tb"; fails="$fails $tb"; continue
   fi
   out=$(timeout 60 vvp /tmp/tb.vvp 2>&1)
   # Each bench prints its own verdict line; a PASS with no FAIL anywhere in the output is a pass.
   if echo "$out" | grep -qiE 'ALL TESTS PASSED|ALL 65536 MATCH|(^|[: >])PASS\b' && ! echo "$out" | grep -qE '\bFAIL'; then
      pass=$((pass+1))
   else
      printf "%-22s FAIL\n" "$tb"; fails="$fails $tb"
   fi
done
echo "----"
echo "tb pass=$pass / $total"
[ -n "$fails" ] && echo "fails:$fails"
