#!/bin/bash
# Config-space sweep (docs/rtl-rules.md G3). The default point is not the only point:
# 023a5df (POOL deadlock at IW=1), 61f7d0a (non-power-of-2 arch map), 41061b8, 9444f81
# all only existed off-default, and every one of them was found by a person tripping
# over it rather than by a gate.
#
#   ./sweep.sh                 # full matrix, compare against sweep-expected.txt
#   ./sweep.sh --record        # rewrite sweep-expected.txt from this run
#   IW="1 2" CKMAX="2" ./sweep.sh
#
# IW and CKMAX are compile-time, CACHE is a run-time plusarg, so CACHE varies innermost
# and the matrix costs |IW| x |CKMAX| builds rather than a build per cell.
#
# A cell FAILS the sweep only if it is WORSE than what sweep-expected.txt records. Known
# breakage is written down, never silently tolerated -- an unexplained entry in that file
# is a bug someone owes an explanation for, not a passing test.
set -u
cd "$(dirname "$0")"

IW=${IW:-"1 2 3 4"}
CKMAX=${CKMAX:-"1 2"}
CACHE_SET=${CACHE_SET:-"0 1"}
EXPECT=sweep-expected.txt
record=0; [ "${1:-}" = --record ] && record=1

out=$(mktemp); trap 'rm -f "$out"' EXIT
printf "%-6s %-6s %-6s %s\n" IW CKMAX CACHE failures

for iw in $IW; do
   for ck in $CKMAX; do
      # one build per (IW, CKMAX)
      if ! VDEFS="-DPROBE_IW=$iw -DPROBE_CKMAX=$ck" ./run-vl-tests.sh rv64ui-p >/dev/null 2>&1; then
         printf "%-6s %-6s %-6s %s\n" "$iw" "$ck" "-" "BUILD-FAILED" | tee -a "$out"
         continue
      fi
      for ca in $CACHE_SET; do
         n=$(CACHE=$ca VDEFS="-DPROBE_IW=$iw -DPROBE_CKMAX=$ck" ./run-vl-tests.sh 2>&1 \
             | awk '/^failures:/{print $2}')
         printf "%-6s %-6s %-6s %s\n" "$iw" "$ck" "$ca" "${n:-ERR}" | tee -a "$out"
      done
   done
done

if [ $record = 1 ]; then
   # Carry each cell's trailing "# reason" across a re-record. Without this the first
   # --record after a fix silently erases why every known-broken cell is broken, which
   # is the same silent-loss failure this file exists to prevent.
   { echo "# IW CKMAX CACHE failures  # why, for every non-zero entry"
     echo "# Regenerate with ./sweep.sh --record; reasons are carried over."
     grep -v '^IW' "$out" | while read -r iw ck ca n; do
        why=$(awk -v a="$iw" -v b="$ck" -v c="$ca" \
              '$1==a&&$2==b&&$3==c{i=index($0,"#"); if(i)print substr($0,i)}' $EXPECT 2>/dev/null)
        if [ -n "$why" ]; then printf "%-6s %-6s %-6s %-6s %s\n" "$iw" "$ck" "$ca" "$n" "$why"
        else                   printf "%-6s %-6s %-6s %s\n"      "$iw" "$ck" "$ca" "$n"; fi
     done; } > $EXPECT.new && mv $EXPECT.new $EXPECT
   echo "recorded $EXPECT"; exit 0
fi

[ -f $EXPECT ] || { echo "no $EXPECT -- run ./sweep.sh --record first"; exit 1; }
rc=0
while read -r iw ck ca n; do
   case "$iw" in '#'*|'') continue;; esac
   exp=$(awk -v a="$iw" -v b="$ck" -v c="$ca" '$1==a&&$2==b&&$3==c{print $4}' $EXPECT)
   [ -z "$exp" ] && { echo "REGRESSION: cell IW=$iw CKMAX=$ck CACHE=$ca is new (not in $EXPECT)"; rc=1; continue; }
   case "$n" in ''|*[!0-9]*) echo "REGRESSION: IW=$iw CKMAX=$ck CACHE=$ca -> $n"; rc=1; continue;; esac
   if [ "$n" -gt "$exp" ]; then
      echo "REGRESSION: IW=$iw CKMAX=$ck CACHE=$ca -> $n failures, expected $exp"; rc=1
   elif [ "$n" -lt "$exp" ]; then
      echo "IMPROVED: IW=$iw CKMAX=$ck CACHE=$ca -> $n failures, expected $exp (update $EXPECT)"
   fi
done < "$out"
[ $rc = 0 ] && echo "sweep: no regressions"
exit $rc
