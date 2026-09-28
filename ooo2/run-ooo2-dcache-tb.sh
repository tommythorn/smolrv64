#!/bin/bash
# rv_dcache's bench (tb_rv_dcache.v): random loads against a golden memory and a reference
# translation with synonyms and remaps, over in-order and reordering memory, several seeds.
#   ./run-ooo2-dcache-tb.sh [seeds...]
set -u
cd "$(dirname "$0")"
verilator --binary --timing -j 0 -sv -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL \
   -Wno-DECLFILENAME -Wno-TIMESCALEMOD -Wno-BLKSEQ -Wno-PROCASSINIT \
   -I../src --top-module tb --Mdir obj_dir_dcache -o tb_dcache \
   rv_dcache.v ../src/smolrv64_sdpram.v tb_rv_dcache.v > obj_dir_dcache.build.log 2>&1 \
   || { echo "BUILD FAILED"; grep -m10 '%Error' obj_dir_dcache.build.log; exit 1; }
rc=0
for s in ${@:-1 2 3}; do
   for mode in "" "+reorder"; do
      ./obj_dir_dcache/tb_dcache +seed=$s $mode 2>&1 | grep -E 'PASS|FAIL|fatal|Error' | head -5
      ./obj_dir_dcache/tb_dcache +seed=$s $mode 2>&1 | grep -q 'DCACHE-TB PASS' || rc=1
   done
done
exit $rc
