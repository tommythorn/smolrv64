#!/usr/bin/env bash
# THE GATE, as one command. Build at the shipping configuration, program, boot, and decide.
#
#   tools/gate.sh              # gate the working tree
#   tools/gate.sh <commit>     # check out <commit> first (detached), then gate it
#
# Two hard gates, in this order (docs/OOO2-Spec.md, and Tommy 2026-09-01):
#   1. probe_clk closes at 166.67 MHz -- `make` alone, no last-mile pass to remember
#   2. the board boots to a login: prompt WITH ZERO PROCESSES SEGFAULTING
# Neither is tradeable. An IPC regression on the way to more IPC is fine; a WNS miss is not,
# and a bitstream that closes but does not boot is worth nothing.
#
# Exists because gating a BATCH of commits and only testing the tip cannot attribute a
# breakage: when the D$ split Oopsed the board on 2026-09-01, the cache change and a
# zero-slack bitstream had moved together and the board could not say which was at fault.
# Per-commit costs the same hour as bisecting later, and answers immediately.
set -u
cd "$(dirname "$0")/.."
REPO=$(pwd)
PLAT=$REPO/platforms/rk-xcku5p-f-v1.2
UB=$REPO/workloads/ubuntu
BOOT_WAIT=${BOOT_WAIT:-1800}      # seconds to wait for login:

if [ $# -ge 1 ]; then
   git checkout -q --detach "$1" || { echo "GATE: cannot check out $1"; exit 2; }
fi
WHAT=$(git log -1 --format='%h %s' | cut -c1-70)
echo "=== GATE: $WHAT ==="

# ---- 1. fast gates first: they cost seconds and can veto an hour ----------------------
(cd "$REPO/src" && ./lint.sh 2>&1 | tail -1) || { echo "GATE: FAIL (lint)"; exit 1; }
if [ -x "$REPO/ooo2/run-ooo2-cache-tb.sh" ]; then
   # Run ONCE and judge the saved output: running it twice to grep it twice doubles the
   # cost and, worse, lets the decision be made on a different execution than the one shown.
   (cd "$REPO/ooo2" && ./run-ooo2-cache-tb.sh) > /tmp/gate-cachetb.log 2>&1
   grep -E 'PASS|FAIL' /tmp/gate-cachetb.log
   grep -q FAIL /tmp/gate-cachetb.log && { echo "GATE: FAIL (directed cache)"; exit 1; }
fi

# ---- 2. the floor: `make` alone, from a clean tree ------------------------------------
echo "--- building (no arguments; the shipping config is the default) ---"
( cd "$PLAT" && git clean -fxdq . && timeout 10800 make ) > /tmp/gate-build.log 2>&1
WNS=$(grep -oE "Timing met: WNS=[-0-9.]+|TIMING VIOLATION: WNS=[-0-9.]+" /tmp/gate-build.log | grep -v '^#' | tail -1)
if [ ! -f "$PLAT/rk_xcku5p.runs/impl_1/rk_xcku5p.bit" ]; then
   echo "GATE: FAIL (timing) $WNS"
   R=$(ls -t "$PLAT"/rk_xcku5p.runs/impl_1/*timing_summary*.rpt 2>/dev/null | head -1)
   [ -n "$R" ] && grep -A6 "Slack (VIOLATED)" "$R" | grep -E "Slack|Source:|Destination:" | head -3
   exit 1
fi
echo "timing: $WNS"

# ---- 3. the board -------------------------------------------------------------------
( cd "$PLAT" && timeout 900 make program ) > /tmp/gate-prog.log 2>&1
grep -qi "programmed successfully" /tmp/gate-prog.log || { echo "GATE: FAIL (program)"; exit 1; }

# screenlog.0 is CUMULATIVE across boots: record the offset or a previous boot's login:
# reads as a pass. That produced a false pass on 2026-09-01.
PRE=$(wc -c < "$UB/screenlog.0")
( cd "$UB" && touch ubuntu-nfs.dts.in && make dtbs >/dev/null 2>&1; timeout 3000 ./ubuntu-boot.sh ) > /tmp/gate-boot.log 2>&1 \
   || { echo "GATE: FAIL (upload)"; tail -3 /tmp/gate-boot.log; exit 1; }

echo "--- booting, watching past byte $PRE ---"
END=$(( $(date +%s) + BOOT_WAIT ))
while [ "$(date +%s)" -lt "$END" ]; do
   NEW=$(tail -c +$((PRE+1)) "$UB/screenlog.0" | tr -d '\r')
   printf '%s' "$NEW" | grep -aq "login:" && break
   printf '%s' "$NEW" | grep -aqE "Kernel panic" && break
   sleep 15
done
NEW=$(tail -c +$((PRE+1)) "$UB/screenlog.0" | tr -d '\r')
FAULTS=$(printf '%s' "$NEW" | grep -acE "unhandled signal|segfault|SIGSEGV|status=11/SEGV|core dumped|Unable to handle kernel paging|Oops \[#|Kernel panic")
LOGIN=$(printf '%s' "$NEW" | grep -ac "login:")
echo "login: $LOGIN   faults: $FAULTS   last: $(printf '%s' "$NEW" | grep -aoE '^\[ *[0-9]+\.[0-9]+\]' | tail -1)"
if [ "$LOGIN" -ge 1 ] && [ "$FAULTS" -eq 0 ]; then
   echo "GATE: PASS  ($WHAT)  $WNS"; exit 0
fi
printf '%s' "$NEW" | grep -aE "unhandled signal|segfault|Oops \[#|epc :" | head -4
echo "GATE: FAIL (board)  login=$LOGIN faults=$FAULTS"; exit 1
