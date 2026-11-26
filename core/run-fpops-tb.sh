#!/usr/bin/env bash
# Unit TB for the core's own Zfa FP ops (smolrv64_fp_ops.vh): tools/fpref.py's exact vectors for fround,
# froundnx, fminm, fmaxm, fleq, fltq and fli, both formats, every rounding mode; FPOPS_N of them
# (default 200000; FPOPS_SEED=<s> for another draw). Seconds, no core build.
set -euo pipefail
cd "$(dirname "$0")"
N=${FPOPS_N:-200000}
SEED=${FPOPS_SEED:-1}
D=obj_dir_fpops
mkdir -p "$D"
python3 ../tools/fpref.py vectors zfa "$N" "$SEED" > "$D/zfa.vec"
verilator --binary --timing -sv -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -I../src \
   --top-module tb --Mdir "$D" -o tb_fpops tb_fp_ops.v > "$D/build.log" 2>&1 \
   || { echo "BUILD FAILED"; grep -E '%Error|%Warning' "$D/build.log" | head; exit 1; }
"./$D/tb_fpops" +vec="$D/zfa.vec" 2>&1 | grep -E 'FPOPS-TB|FAIL'
