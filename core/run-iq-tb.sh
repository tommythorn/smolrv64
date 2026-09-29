#!/usr/bin/env bash
# Unit TB for the scheduler (smolrv64_iq). Seconds, no core build -- run it before any
# integration attempt. docs/rtl-rules.md H1: cheapest confirmation first.
set -euo pipefail
cd "$(dirname "$0")"
verilator --binary --timing -sv -Wall \
   -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
   -Wno-TIMESCALEMOD -Wno-PROCASSINIT \
   --top-module tb --Mdir obj_dir_iq -o tb_iq smolrv64_iq.v tb_smolrv64_iq.v > /tmp/iqtb_build.log 2>&1 \
   || { echo "BUILD FAILED"; grep -E '%Error' /tmp/iqtb_build.log | head; exit 1; }
./obj_dir_iq/tb_iq 2>&1 | grep -E '  ok|FAIL|IQ-TB'
