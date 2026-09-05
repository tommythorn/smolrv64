#!/usr/bin/env bash
# A store, a cbo.zero, a store, under the full core (workloads/fphammer/cbozero.c): the
# M-executed maintenance op that must follow every OLDER store while a YOUNGER one is already
# allocated in the store queue behind it. This shape hung build L at SLUB init (2026-09-04):
# the CBO waited for an EMPTY queue, and the younger store could not get its address until M
# freed. The tiny128 boot never issues a cbo.zero, so the cosim was blind to it; this is the
# gate for the class (rule C5). Seconds; a hang is the failure mode, so the run is bounded.
#   ./run-ooo2-cbozero-tb.sh        # CBOZERO-TB PASS or FAIL
set -u
cd "$(dirname "$0")"
make -s -C ../workloads/fphammer cbozero.bin || { echo "CBOZERO-TB FAIL (build)"; exit 1; }
OUT=$(BUILD=${BUILD:-0} FW=$(pwd)/../workloads/fphammer/cbozero.bin CYC=4000000 timeout 1200 ./run-ooo2-linux.sh 2>&1)
if grep -q 'cbozero: ok' <<<"$OUT"; then echo "CBOZERO-TB PASS"; exit 0; fi
grep -a 'cbozero\|TIMEOUT\|Fatal\|BUILD' <<<"$OUT" | head -5
echo "CBOZERO-TB FAIL"; exit 1
