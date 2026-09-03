#!/bin/sh
# Standalone check for ooo2_sq, the store buffer. Seconds; run before the full suite.
set -e
cd "$(dirname "$0")"
verilator --binary -Wno-DECLFILENAME -Wno-WIDTHEXPAND \
   --top-module tb_ooo2_sq tb_ooo2_sq.v ooo2_sq.v -o tb_sq > /tmp/sqtb_build.log 2>&1 \
   || { echo "BUILD FAILED:"; grep -E "%Error" /tmp/sqtb_build.log | head -5; exit 1; }
exec ./obj_dir/tb_sq
