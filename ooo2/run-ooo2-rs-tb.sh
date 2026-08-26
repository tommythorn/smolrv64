#!/usr/bin/env bash
# Unit TB for the scheduler (ooo2_rs). Seconds, no core build -- run it before any
# integration attempt. docs/rtl-rules.md H1: cheapest confirmation first.
set -euo pipefail
cd "$(dirname "$0")"
verilator --binary --timing -sv -Wall \
   -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
   -Wno-TIMESCALEMOD -Wno-PROCASSINIT \
   --top-module tb --Mdir obj_dir_rs -o tb_rs ooo2_rs.v tb_ooo2_rs.v > /tmp/rstb_build.log 2>&1 \
   || { echo "BUILD FAILED"; grep -E '%Error' /tmp/rstb_build.log | head; exit 1; }
./obj_dir_rs/tb_rs 2>&1 | grep -E '  ok|FAIL|RS-TB'
