#!/usr/bin/env bash
# Wait for the running Vivado build to END, then print its timing and where the bitstream is.
#
#   tools/wait-vivado.sh [platform-dir]
#
# Keyed on the PROCESS (`pgrep -x vivado`), never on a log line: build.tcl runs synthesis and
# implementation as separate Vivado processes, so "Exiting Vivado" appears mid-build (a waiter
# keyed on it fired 15 minutes early on 2026-09-04). `-x` matches the process name exactly and
# cannot match the shell running this script the way `pgrep -f` does.
set -u
P=${1:-$(dirname "$0")/../platforms/rk-xcku5p-f-v1.2}
while pgrep -x vivado >/dev/null; do sleep 30; done
R=$P/rk_xcku5p.runs/impl_1
for r in routed postroute_physopted; do
   f=$R/rk_xcku5p_timing_summary_$r.rpt
   [ -f "$f" ] || { echo "$r: no report"; continue; }
   grep -m1 -A6 'Design Timing Summary' "$f" | tail -1 | awk -v r=$r '{print r": WNS="$1" TNS="$2" failing="$3}'
done
if [ -f "$R/rk_xcku5p.bit" ]; then echo "bitstream: $R/rk_xcku5p.bit ($(date -r "$R/rk_xcku5p.bit" +%H:%M))"
else echo "NO BITSTREAM (timing failed, or the build died: see the build log)"; fi
