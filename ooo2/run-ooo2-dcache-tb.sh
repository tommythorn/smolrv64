#!/bin/bash
# rv_dcache's bench (tb_rv_dcache.v): random loads and committed stores against a golden memory
# and a reference translation with synonyms and remaps, over in-order and reordering memory,
# several seeds.
#   ./run-ooo2-dcache-tb.sh [seeds...]
set -u
cd "$(dirname "$0")"
verilator --binary --timing -j 0 -sv -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL \
   -Wno-DECLFILENAME -Wno-TIMESCALEMOD -Wno-BLKSEQ -Wno-PROCASSINIT \
   -I../src --top-module tb --Mdir obj_dir_dcache -o tb_dcache \
   rv_dcache.v ../src/smolrv64_sdpram.v tb_rv_dcache.v > obj_dir_dcache.build.log 2>&1 \
   || { echo "BUILD FAILED"; grep -m10 '%Error' obj_dir_dcache.build.log; exit 1; }
# Each seed runs over both memory orders and four shapes: the default, a slow memory (every
# MSHR busy: the waiters' fairness), a fast one with twice the stores (fills racing the read-out
# and the merge buffer), and a remap every 500 cycles (epoch wraps and their scans).
CFGS=("" "+latmin=100 +latmax=300" "+latmin=8 +latmax=12 +stores=70" "+remap=500")
rc=0; n=0; bad=0
for s in ${@:-1 2 3 4 5 6}; do
   for mode in "" "+reorder"; do
      for cfg in "${CFGS[@]}"; do
         out=$(./obj_dir_dcache/tb_dcache +seed=$s $mode $cfg 2>&1)
         n=$((n + 1))
         if ! grep -q 'DCACHE-TB PASS' <<< "$out"; then
            bad=$((bad + 1)); rc=1
            echo "---- seed=$s $mode $cfg"; grep -iE 'FAIL|fatal|Error' <<< "$out" | head -5
         elif [ -z "$mode$cfg" ]; then
            grep 'DCACHE-TB PASS' <<< "$out"
         fi
      done
   done
done
echo "DCACHE-TB: $((n - bad)) of $n runs pass"
exit $rc
