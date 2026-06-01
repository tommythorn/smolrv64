#!/usr/bin/env bash
set -euo pipefail

# Decompose smolrv64 CPI into structural vs memory-stall on the live FPGA.
#
# Run under the Ubuntu/Linux that is booted on the board, against a
# representative workload, e.g.:
#
#   ./perf-cpi.sh sha256sum bigfile
#   ./perf-cpi.sh dd if=/dev/zero of=/dev/null bs=1M count=256
#
# BUS_WAIT_CYCLE (0x0202) counts every cycle the backend is parked in a
# memory-wait state, so it is a faithful DRAM-stall meter on real silicon
# (simulation has no true DRAM model and cannot stand in for it).
#
#   memory-stall CPI = BUS_WAIT_CYCLE / instructions
#   structural  CPI = (cycles - BUS_WAIT_CYCLE) / instructions
#
# A large structural CPI means the FSM walk dominates (pipeline overlap is
# the win); a large memory CPI means misses dominate (attack the cache/fill
# path) -- and that half must be validated on the board, not in sim.

if [[ $# -eq 0 ]]; then
    echo "usage: $0 <command> [args...]" >&2
    exit 2
fi

# Raw HPM event codes (see docs/smolrv64-perf-events.json):
#   r0202 BUS_WAIT_CYCLE   r0102 ICACHE_MISS   r0113 DCACHE_MISS
EVENTS="instructions,cycles,r0202,r0102,r0113"

TMP=$(mktemp)
trap 'rm -f "$TMP"' EXIT

# -x, gives machine-readable CSV: value,unit,event,...
perf stat -x, -e "$EVENTS" -- "$@" 2>"$TMP" || {
    echo "perf failed:" >&2
    cat "$TMP" >&2
    exit 1
}

get() { awk -F, -v e="$1" '$3==e {gsub(/[^0-9]/,"",$1); print $1}' "$TMP"; }

INSN=$(get instructions)
CYC=$(get cycles)
BUS=$(get r0202)
IMISS=$(get r0102)
DMISS=$(get r0113)

: "${INSN:=0}" "${CYC:=0}" "${BUS:=0}" "${IMISS:=0}" "${DMISS:=0}"

if [[ "$INSN" -eq 0 || "$CYC" -eq 0 ]]; then
    echo "no instruction/cycle counts captured -- is the HPM PMU exposed to perf?" >&2
    cat "$TMP" >&2
    exit 1
fi

awk -v insn="$INSN" -v cyc="$CYC" -v bus="$BUS" -v im="$IMISS" -v dm="$DMISS" 'BEGIN {
    printf "instructions     %15d\n", insn
    printf "cycles           %15d\n", cyc
    printf "bus_wait_cycle   %15d\n", bus
    printf "icache_miss      %15d\n", im
    printf "dcache_miss      %15d\n", dm
    printf "----\n"
    printf "total CPI        %15.3f\n", cyc/insn
    printf "memory  CPI      %15.3f  (%.1f%% of cycles)\n", bus/insn, 100.0*bus/cyc
    printf "structural CPI   %15.3f  (%.1f%% of cycles)\n", (cyc-bus)/insn, 100.0*(cyc-bus)/cyc
}'
