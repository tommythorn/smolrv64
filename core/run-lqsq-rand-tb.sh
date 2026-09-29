#!/usr/bin/env bash
# Constrained-random ORDERING test of the load queue and the store queue together
# (tb_smolrv64_lqsq_rand.v): a random program-order stream of loads and stores over one line,
# checked on every offered candidate against a program-order model. Seconds, no core build.
#
#   ./run-lqsq-rand-tb.sh                 # three seeds, 20000 ops each
#   SEEDS="7 8 9" NOPS=100000 ./run-lqsq-rand-tb.sh
#
# The directed bench (run-lqsq-tb.sh) pins the cases we know; this one is for the ones
# we do not. It would have found the store-seqno wrap of 2026-09-04 on its first seed.
set -euo pipefail
cd "$(dirname "$0")"
verilator --binary --timing -j 0 -sv -Wall \
   -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
   -Wno-TIMESCALEMOD -Wno-PROCASSINIT -Wno-UNUSEDPARAM -Wno-BLKSEQ -Wno-UNUSEDLOOP \
   --top-module tb --Mdir obj_dir_lqsq_rand -o tb_lqsq_rand smolrv64_lq.v smolrv64_sq.v tb_smolrv64_lqsq_rand.v > /tmp/lqsqrandtb_build.log 2>&1 \
   || { echo "BUILD FAILED"; grep -E '%Error|%Warning' /tmp/lqsqrandtb_build.log | head; exit 1; }
rc=0
for s in ${SEEDS:-1 2 3}; do
   ./obj_dir_lqsq_rand/tb_lqsq_rand +seed=$s +nops=${NOPS:-20000} 2>&1 | grep -E 'FAIL|PASS|tb_smolrv64_lqsq_rand' || rc=1
done
exit $rc
