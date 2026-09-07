#!/usr/bin/env bash
# Read the fetch buffer's silicon readout (FBDIAG_BASE = 0x1000_e000, rv_soc_top.v) through the
# ROM monitor on the serial console: after a self-reset (fb_stale_now / fb_stuck_now) the
# monitor is back and the sticky registers say which chunk was served from a stale mapping.
#   tools/fbdiag-read.sh [screen-session]      # needs the monitor prompt, not Linux
set -u
REPO=$(cd "$(dirname "$0")/.." && pwd)
UB=$REPO/workloads/ubuntu
[ -f "$UB/screenlog.0" ] || UB=$(git -C "$REPO" worktree list | head -1 | awk '{print $1}')/workloads/ubuntu
SESSION=${1:-$(screen -ls | awk '/\t[0-9]+\./ {print $1; exit}')}
[ -n "$SESSION" ] || { echo "no screen session"; exit 1; }
NAMES=(magic "stuck|stale" va panow pacached satp pc flags cyc freecyc ctxhits)
PRE=$(wc -c < "$UB/screenlog.0")
for i in $(seq 0 10); do
   printf -v A '%x' $((0x1000e000 + 8*i))
   screen -S "$SESSION" -X stuff "R$A"$'\r'; sleep 0.4
done
sleep 1
tail -c +$((PRE+1)) "$UB/screenlog.0" | tr -d '\r' | grep -E "^0*1000e0[0-9a-f]{2}: " | while read -r line; do
   addr=${line%%:*}; val=${line#*: }; i=$(( (0x$addr - 0x1000e000) / 8 ))
   printf '%-10s %s' "${NAMES[$i]}" "$val"
   if [ $i -eq 7 ]; then
      v=$((16#$val))
      printf '   ok=%d in1=%d v0=%d v1=%d rq=%d ctx=%d priv=%d tagv=%d arr=%d arr_stale=%d' $((v&1)) $(((v>>1)&1)) $(((v>>2)&1)) $(((v>>3)&1)) $(((v>>4)&1)) $(((v>>5)&1)) $(((v>>6)&3)) $(((v>>8)&1)) $(((v>>9)&1)) $(((v>>10)&1))
   fi
   echo
done
