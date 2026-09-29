#!/bin/bash
# Directed rv_cache regression: the lookup pipeline vs. a fill in flight.
# See tb_smolrv64_cache.v for what each case is and which defect it was written for.
#   ./run-cache-tb.sh            # gate: prints PASS or FAIL
#   ./run-cache-tb.sh -trace     # per-cycle response trace
set -u
cd "$(dirname "$0")"
extra=""; [ "${1:-}" = "-trace" ] && extra="+trace"
verilator --binary --timing -j 0 -sv -Wno-fatal \
   -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-CASEINCOMPLETE -Wno-LATCH -Wno-UNUSEDSIGNAL \
   -Wno-UNUSEDPARAM -Wno-DECLFILENAME -Wno-ASCRANGE -Wno-UNSIGNED -Wno-WIDTH -Wno-UNOPTFLAT \
   -Wno-TIMESCALEMOD -Wno-BLKSEQ -Wno-PROCASSINIT -Wno-VARHIDDEN \
   -I../src --top-module tb --Mdir obj_dir_cache -o tb_cache \
   rv_cache.v ../src/smolrv64_sdpram.v tb_smolrv64_cache.v > /tmp/core-cache-tb-build.log 2>&1 \
   || { echo "BUILD FAILED (I\$ shape)"; grep -m10 '%Error' /tmp/core-cache-tb-build.log; exit 1; }
./obj_dir_cache/tb_cache $extra 2>&1 | grep -vE '^- '

# ...and the D$ shape, WRITABLE=1, swept over the L2 latency. Both roles, every time: a pass
# at WRITABLE=0 says nothing about the instance that carries stores, cbo.zero and evictions.
for lat in 4 20 100 200; do
   verilator --binary --timing -j 0 -sv -Wno-fatal \
      -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-CASEINCOMPLETE -Wno-LATCH -Wno-UNUSEDSIGNAL \
      -Wno-UNUSEDPARAM -Wno-DECLFILENAME -Wno-ASCRANGE -Wno-UNSIGNED -Wno-WIDTH -Wno-UNOPTFLAT \
      -Wno-TIMESCALEMOD -Wno-BLKSEQ -Wno-PROCASSINIT -Wno-VARHIDDEN -DLAT=$lat \
      -I../src --top-module tb --Mdir obj_dir_dcache_$lat -o tb_dcache \
      rv_cache.v ../src/smolrv64_sdpram.v tb_smolrv64_dcache.v > /tmp/core-dcache-tb-build.log 2>&1 \
      || { echo "BUILD FAILED (D\$ shape, LAT=$lat)"; grep -m10 '%Error' /tmp/core-dcache-tb-build.log; exit 1; }
   printf 'LAT=%-4s ' "$lat"
   ./obj_dir_dcache_$lat/tb_dcache $extra 2>&1 | grep -vE '^- '
done
