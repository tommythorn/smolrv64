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
# ST_MEM (0x0300) counts every cycle M is stalled on the LSU, so it is a faithful
# memory-stall meter on real silicon (simulation has no true DRAM model and cannot
# stand in for it).
#
#   memory-stall CPI = ST_MEM / instructions
#   structural  CPI = (cycles - ST_MEM) / instructions
#
# This script previously asked for r0202 BUS_WAIT_CYCLE, r0102 "ICACHE_MISS" and r0113
# "DCACHE_MISS". None of those were right: the RTL implements no 0x0202 at all, 0x0102
# is the D$ miss (not the I$), and 0x0113 does not exist -- so all three read zero. The
# names came from docs/smolrv64-perf-events.json back when it was hand-maintained and had
# drifted to describe a retired core; it is now generated from src/csr_file.v.
#
# For the full per-cause CPI stack (ST_MEM/ST_DIV/ST_MUL/ST_FPU/ST_SER/FE_BUB and the
# FE_MMU/FE_IC breakdown, with a self-check against measured CPI) use workloads/ipcstat.
#
# A large structural CPI means the FSM walk dominates (pipeline overlap is
# the win); a large memory CPI means misses dominate (attack the cache/fill
# path) -- and that half must be validated on the board, not in sim.

if [[ $# -eq 0 ]]; then
    echo "usage: $0 <command> [args...]" >&2
    exit 2
fi

# Raw HPM event codes -- docs/smolrv64-perf-events.json is generated from csr_file.v:
#   r0300 ST_MEM   r0102 DCMISS   r0112 ICMISS
EVENTS="instructions,cycles,r0300,r0102,r0112"

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
STMEM=$(get r0300)
DMISS=$(get r0102)
IMISS=$(get r0112)

: "${INSN:=0}" "${CYC:=0}" "${STMEM:=0}" "${IMISS:=0}" "${DMISS:=0}"

if [[ "$INSN" -eq 0 || "$CYC" -eq 0 ]]; then
    echo "no instruction/cycle counts captured -- is the HPM PMU exposed to perf?" >&2
    cat "$TMP" >&2
    exit 1
fi

awk -v insn="$INSN" -v cyc="$CYC" -v bus="$STMEM" -v im="$IMISS" -v dm="$DMISS" 'BEGIN {
    printf "instructions     %15d\n", insn
    printf "cycles           %15d\n", cyc
    printf "st_mem (cycles)  %15d\n", bus
    printf "icache_miss      %15d\n", im
    printf "dcache_miss      %15d\n", dm
    printf "----\n"
    printf "total CPI        %15.3f\n", cyc/insn
    printf "memory  CPI      %15.3f  (%.1f%% of cycles)\n", bus/insn, 100.0*bus/cyc
    printf "structural CPI   %15.3f  (%.1f%% of cycles)\n", (cyc-bus)/insn, 100.0*(cyc-bus)/cyc
}'
