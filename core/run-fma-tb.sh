#!/usr/bin/env bash
# Unit TB for fp_fma: tools/fpref.py's exact FMA-group vectors (both formats, every rounding mode,
# all seven op variants, a quarter of them cancelling), FMA_N of them (default 40000; FMA_SEED=<s>
# for another draw), issued back to back. Seconds, no core build.
set -euo pipefail
cd "$(dirname "$0")"
N=${FMA_N:-40000}
SEED=${FMA_SEED:-1}
D=obj_dir_fma
mkdir -p "$D"
python3 ../tools/fpref.py vectors fma "$N" "$SEED" > "$D/fma.vec"
verilator --binary --timing -sv -Wall \
   -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-PROCASSINIT -Wno-BLKSEQ \
   --top-module tb --Mdir "$D" -o tb_fma ../src/fp_fma.v tb_fp_fma.v > "$D/build.log" 2>&1 \
   || { echo "BUILD FAILED"; grep -E '%Error' "$D/build.log" | head; exit 1; }
"./$D/tb_fma" +vec="$D/fma.vec" 2>&1 | grep -E 'FMA-TB|FAIL'
