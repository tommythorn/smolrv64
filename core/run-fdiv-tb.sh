#!/usr/bin/env bash
# Unit TB for fp_divsqrt: tools/fpref.py's exact vectors (both formats, every rounding mode, the
# special and subnormal cases), N of each op (default 20000; FDIV_N=<n> for a longer campaign,
# FDIV_SEED=<s> for another draw). Seconds, no core build.
set -euo pipefail
cd "$(dirname "$0")"
N=${FDIV_N:-20000}
SEED=${FDIV_SEED:-1}
D=obj_dir_fdiv
mkdir -p "$D"
python3 ../tools/fpref.py vectors div  "$N" "$SEED" > "$D/div.vec"
python3 ../tools/fpref.py vectors sqrt "$N" "$SEED" > "$D/sqrt.vec"
verilator --binary --timing -sv -Wall \
   -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-PROCASSINIT \
   --top-module tb --Mdir "$D" -o tb_fdiv ../src/fp_divsqrt.v tb_fp_divsqrt.v > "$D/build.log" 2>&1 \
   || { echo "BUILD FAILED"; grep -E '%Error' "$D/build.log" | head; exit 1; }
for op in div sqrt; do
   sq=0; [ "$op" = sqrt ] && sq=1
   "./$D/tb_fdiv" +vec="$D/$op.vec" +sqrt=$sq 2>&1 | grep -E 'FDIV-TB|FAIL'
done
