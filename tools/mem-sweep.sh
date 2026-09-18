#!/usr/bin/env bash
# B7 (2026-09-17): the DDR-latency sweep. Retires at a fixed cycle budget for each latency at each
# width -- the memory backend's sensitivity to latency is the number every MLP increment is judged
# against (C5's exit: the gap between the measured shape and latency 4 shrinks by a third or more).
#
#   tools/mem-sweep.sh [table.txt]      env: CYC=60000000  LATS="4 0 80" (0 = the measured shape)
#                                            IWS="2 3"      MEM_LG2 as the runner's
# Each point is one lockstep cosim (the runner rebuilds when the width changes); a divergence or
# assertion leaves the cell empty, and the runner's retire-count grading does not apply here.
set -u
cd "$(dirname "$0")/../ooo2" || exit 1
CYC=${CYC:-60000000}; LATS=${LATS:-"4 0 80"}; IWS=${IWS:-"2 3"}; OUT=${1:-/dev/stdout}
{ echo "# mem-sweep $(date +%F) CYC=$CYC (lat 0 = the measured DDR shape; reads mean 27.5, writes 13.5)"
  printf '%-3s %-5s %-10s %-8s\n' IW lat retires vs_lat4; } > "$OUT"
for iw in $IWS; do
   base=""
   for lat in $LATS; do
      log=$(mktemp); PLUSARGS="+ddr_lat=$lat" VDEFS="-DOOO2_IW=$iw" CYC=$CYC ./run-ooo2-cosim-linux.sh > "$log" 2>&1
      r=$(sed -n 's/.*TIMEOUT after [0-9]* cycles (retires=\([0-9]*\).*/\1/p' "$log" | tail -1)
      [ -z "$r" ] && { echo "IW=$iw lat=$lat: NO RESULT (see $log)"; printf '%-3s %-5s %-10s %-8s\n' $iw $lat - - >> "$OUT"; continue; }
      [ "$lat" = 4 ] && base=$r
      pct=$( [ -n "$base" ] && awk -v g=$r -v b=$base 'BEGIN{printf "%+.1f%%", 100*(g-b)/b}' || echo - )
      printf '%-3s %-5s %-10s %-8s\n' $iw $lat $r "$pct" >> "$OUT"; rm -f "$log"
   done
done
[ "$OUT" != /dev/stdout ] && cat "$OUT"
