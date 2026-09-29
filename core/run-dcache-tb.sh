#!/bin/bash
# rv_dcache's bench (tb_rv_dcache.v): random loads, stores, walks, NC accesses and CBOs against a
# golden memory and a reference translation with synonyms and remaps, over in-order and
# reordering memory, several seeds, at VIRT=1 and VIRT=0.
#   ./run-dcache-tb.sh [seeds...]
set -u
cd "$(dirname "$0")"
# Two builds: VIRT=1 (virtual hits, synonyms across colours) and VIRT=0, every request by PA, the
# configuration rv_soc_top ships in phase 1.
for v in 1 0; do
   verilator --binary --timing -j 0 -sv -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL \
      -Wno-DECLFILENAME -Wno-TIMESCALEMOD -Wno-BLKSEQ -Wno-PROCASSINIT -DDC_VIRT=$v \
      -I../src --top-module tb --Mdir obj_dir_dcache_v$v -o tb_dcache \
      rv_dcache.v ../src/smolrv64_sdpram.v tb_rv_dcache.v > obj_dir_dcache_v$v.build.log 2>&1 \
      || { echo "BUILD FAILED (VIRT=$v)"; grep -m10 '%Error' obj_dir_dcache_v$v.build.log; exit 1; }
done
# Each seed runs over both memory orders and five shapes: the default, a slow memory (every
# MSHR busy: the waiters' fairness), a fast one with twice the stores (fills racing the read-out
# and the merge buffer), a remap every 500 cycles (epoch wraps and their scans), and the on-chip
# SRAM's 1-4 cycles (a fill landing while its slot is still being read out).
CFGS=("" "+latmin=100 +latmax=300" "+latmin=8 +latmax=12 +stores=70" "+remap=500" "+latmin=1 +latmax=4")
rc=0; n=0; bad=0
for v in 1 0; do
 for s in ${@:-1 2 3 4 5 6}; do
   for mode in "" "+reorder"; do
      for cfg in "${CFGS[@]}"; do
         out=$(./obj_dir_dcache_v$v/tb_dcache +seed=$s $mode $cfg 2>&1)
         n=$((n + 1))
         if ! grep -q 'DCACHE-TB PASS' <<< "$out"; then
            bad=$((bad + 1)); rc=1
            echo "---- VIRT=$v seed=$s $mode $cfg"; grep -iE 'FAIL|fatal|Error' <<< "$out" | head -5
         elif [ -z "$mode$cfg" ]; then
            echo "VIRT=$v $(grep 'DCACHE-TB PASS' <<< "$out")"
         fi
      done
   done
 done
done
echo "DCACHE-TB: $((n - bad)) of $n runs pass"
exit $rc
