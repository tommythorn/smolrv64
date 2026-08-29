#!/bin/bash
# Standalone check for ooo2_lq, the load queue. Seconds; run before the full suite.
set -u
cd "$(dirname "$0")"
verilator --binary -Wno-DECLFILENAME -Wno-WIDTHEXPAND \
   --timing -j 0 --Mdir obj_dir_lq \
   --top-module tb_ooo2_lq tb_ooo2_lq.v ooo2_lq.v -o tb_lq >/dev/null 2>&1
[ $? -ne 0 ] && { echo "BUILD FAILED"; exit 1; }
exec ./obj_dir_lq/tb_lq
