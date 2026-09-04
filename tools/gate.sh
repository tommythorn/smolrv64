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

# Evidence outlives the build tree. `make clean` deletes *.log inside the platform dir, so a
# log written there is gone the moment the NEXT gate runs -- which is how the 2026-09-02 pass
# lost its timing report and with it the answer to "where is the margin now". Results go
# under the repo root, keyed by commit, and nothing in the platform Makefile can reach them.
SHA=$(git rev-parse --short HEAD)
RES="$REPO/gate-results/$SHA"
mkdir -p "$RES"
git log -1 --format='%H%n%s%n%ci' > "$RES/commit.txt"

# ---- 1. fast gates first: they cost seconds and can veto an hour ----------------------
(cd "$REPO/src" && ./lint.sh 2>&1 | tail -1) || { echo "GATE: FAIL (lint)"; exit 1; }
# EVERY unit tb, found by GLOB rather than named one at a time. Only the cache tb was ever
# run here; on 2026-09-03 three of the other four turned out not even to COMPILE -- tb_ooo2_iq
# lost `iss_ps` when the tags moved to a LUTRAM, and tb_ooo2_lq/tb_ooo2_sq still drive the
# per-candidate address ports that the conflict-matrix rewrite deleted. Nothing noticed,
# because no gate ran them and the runners sent their build errors to /dev/null. A test that
# does not compile is a test that cannot fail, which is the same defect as a test that passes
# vacuously. A build failure here is a GATE failure, and a new run-ooo2-*-tb.sh is picked up
# without editing this file.
for tb in "$REPO"/ooo2/run-ooo2-*-tb.sh; do
   [ -x "$tb" ] || continue
   n=$(basename "$tb" .sh)
   (cd "$REPO/ooo2" && "$tb") > "$RES/$n.log" 2>&1
   rc=$?
   grep -E 'PASS|FAIL|BUILD FAILED' "$RES/$n.log" | sed "s/^/  [$n] /"
   if [ $rc -ne 0 ] || grep -q 'FAIL' "$RES/$n.log"; then
      echo "GATE: FAIL ($n)"; exit 1
   fi
done

# ---- 2. the floor: `make` alone, from a clean tree ------------------------------------
echo "--- building (no arguments; the shipping config is the default) ---"
( cd "$PLAT" && git clean -fxdq . && timeout 10800 make ) > "$RES/build.log" 2>&1
WNS=$(grep -oE "Timing met: WNS=[-0-9.]+|TIMING VIOLATION: WNS=[-0-9.]+" "$RES/build.log" | grep -v '^#' | tail -1)
echo "$WNS" > "$RES/wns.txt"
# The worst path is worth keeping WHETHER OR NOT it met: a pass at +0.019 ns names the path
# the next change has to avoid, and that is the whole input to headroom work.
R=$(ls -t "$PLAT"/rk_xcku5p.runs/impl_1/*timing_summary*.rpt 2>/dev/null | head -1)
[ -n "$R" ] && cp "$R" "$RES/timing_summary.rpt"
WORST=$(grep -A9 -E "Slack \((VIOLATED|MET)\)" "$RES/timing_summary.rpt" 2>/dev/null \
        | grep -E "Slack|Source:|Destination:" | head -3)
[ -n "$WORST" ] && printf '%s\n' "$WORST" > "$RES/worst-path.txt"
if [ ! -f "$PLAT/rk_xcku5p.runs/impl_1/rk_xcku5p.bit" ]; then
   echo "GATE: FAIL (timing) $WNS   [evidence: gate-results/$SHA/]"
   printf '%s\n' "$WORST"
   exit 1
fi
echo "timing: $WNS"
printf '%s\n' "$WORST" | sed 's/^/  /'

# ---- 3. the board: tools/board-gate.sh, runnable alone on a banked bitstream ------------
tools/board-gate.sh "$RES" | tee "$RES/board.log"
if [ "${PIPESTATUS[0]}" -eq 0 ]; then
   echo "GATE: PASS  ($WHAT)  $WNS   [evidence: gate-results/$SHA/]"; exit 0
fi
echo "GATE: FAIL (board)   [evidence: gate-results/$SHA/]"; exit 1
