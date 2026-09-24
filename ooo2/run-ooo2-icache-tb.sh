#!/bin/bash
# rv_icache, the read-only VHPR I$: the random stress bench (tb_ooo2_icache.v) at two L2 latencies.
set -u
cd "$(dirname "$0")"
for lat in 4 40; do
   verilator --binary --timing -j 0 -sv -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDSIGNAL \
      -Wno-DECLFILENAME -Wno-TIMESCALEMOD -Wno-BLKSEQ -Wno-PROCASSINIT -DLAT=$lat \
      -I../src --top-module tb --Mdir obj_dir_icache_$lat -o tb_icache \
      rv_icache.v ../src/smolrv64_sdpram.v tb_ooo2_icache.v > obj_dir_icache_$lat.build.log 2>&1 \
      || { echo "BUILD FAILED (LAT=$lat)"; grep -m10 '%Error' obj_dir_icache_$lat.build.log; exit 1; }
   ./obj_dir_icache_$lat/tb_icache 2>&1 | grep -E 'PASS|FAIL|fatal|Error'
done
