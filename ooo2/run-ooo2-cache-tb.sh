#!/bin/bash
# Directed rv_cache regression: the lookup pipeline vs. a fill in flight.
# See tb_ooo2_cache.v for what each case is and which defect it was written for.
#   ./run-ooo2-cache-tb.sh            # gate: prints PASS or FAIL
#   ./run-ooo2-cache-tb.sh -trace     # per-cycle response trace
set -u
cd "$(dirname "$0")"
extra=""; [ "${1:-}" = "-trace" ] && extra="+trace"
verilator --binary --timing -j 0 -sv -Wno-fatal \
   -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-CASEINCOMPLETE -Wno-LATCH -Wno-UNUSEDSIGNAL \
   -Wno-UNUSEDPARAM -Wno-DECLFILENAME -Wno-ASCRANGE -Wno-UNSIGNED -Wno-WIDTH -Wno-UNOPTFLAT \
   -Wno-TIMESCALEMOD -Wno-BLKSEQ -Wno-PROCASSINIT -Wno-VARHIDDEN \
   -I../src --top-module tb --Mdir obj_dir_cache -o tb_cache \
   rv_cache.v ../src/smolrv64_sdpram.v tb_ooo2_cache.v > /tmp/ooo2-cache-tb-build.log 2>&1 \
   || { echo "BUILD FAILED"; grep -m10 '%Error' /tmp/ooo2-cache-tb-build.log; exit 1; }
./obj_dir_cache/tb_cache $extra 2>&1 | grep -vE '^- '
