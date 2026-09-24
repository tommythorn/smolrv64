#!/bin/bash
# Build and run every unit testbench under src/ (tb_*.v) with Verilator, in parallel, and count
# PASS. These cover the blocks the core shares (aligner, fetch, decode, ALU, MMU, mul/div, FPU
# wrapper) and the SoC devices (CLINT, PLIC, DDR line bridge, virtio-net). The core's own benches
# live in ooo2/run-ooo2-*-tb.sh. Rule G4: run this after any port change to a shared module.
#
# Each bench gets every src/ module and its own obj_dir_src_<bench>; --top-module elaborates
# what the bench reaches. Build logs: obj_dir_src_<bench>/build.log, run output: .../run.log.
set -u
cd "$(dirname "$0")"
ulimit -v $((25 * 1024 * 1024)) 2>/dev/null || true   # cap @25 GiB: runaway aborts, not OOM
export SRCS=$(. ./rtl-sources.sh; rtl_sources)

one() {
   local tb=$1 name=${1%.v} d=obj_dir_src_${1%.v} top
   top=$(grep -m1 -oE '^module +[A-Za-z_0-9]+' "$tb" | awk '{print $2}')
   mkdir -p "$d"
   if ! timeout 600 verilator --binary --timing -j 4 -sv -Wno-fatal -Wno-lint -Wno-style -I. \
          --top-module "$top" --Mdir "$d" -o "$name" $SRCS "$tb" fp_unit_stub.sv > "$d/build.log" 2>&1; then
      printf "%-22s BUILD-FAIL (%s/build.log)\n" "$tb" "$d"; return
   fi
   timeout 300 "./$d/$name" > "$d/run.log" 2>&1
   # Each bench prints its own verdict line; a PASS with no FAIL anywhere in the output is a pass.
   if grep -qiE 'ALL TESTS PASSED|ALL 65536 MATCH|(^|[: >])PASS\b' "$d/run.log" && ! grep -qE '\bFAIL' "$d/run.log"; then
      printf "%-22s PASS\n" "$tb"
   else
      printf "%-22s FAIL (%s/run.log)\n" "$tb" "$d"
   fi
}
export -f one

res=$(ls tb_*.v | xargs -P 8 -I{} bash -c 'one {}' | sort)
echo "$res" | grep -v ' PASS$'
echo "----"
echo "tb pass=$(echo "$res" | grep -c ' PASS$') / $(echo "$res" | grep -c .)"
