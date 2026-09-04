#!/usr/bin/env bash
# Unit TB for the load queue and the store queue TOGETHER (tb_ooo2_lqsq.v): the alias test
# is a conflict matrix that crosses the two, so one bench owns both. Seconds, no core build.
set -euo pipefail
cd "$(dirname "$0")"
verilator --binary --timing -j 0 -sv -Wall \
   -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
   -Wno-TIMESCALEMOD -Wno-PROCASSINIT -Wno-UNUSEDPARAM -Wno-BLKSEQ \
   --top-module tb --Mdir obj_dir_lqsq -o tb_lqsq ooo2_lq.v ooo2_sq.v tb_ooo2_lqsq.v > /tmp/lqsqtb_build.log 2>&1 \
   || { echo "BUILD FAILED"; grep -E '%Error' /tmp/lqsqtb_build.log | head; exit 1; }
./obj_dir_lqsq/tb_lqsq 2>&1 | grep -E 'FAIL|PASS|tb_ooo2_lqsq'
