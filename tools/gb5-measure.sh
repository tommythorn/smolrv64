#!/usr/bin/env bash
# A measured Geekbench 5 run: tools/gb5-measure.sh <2g|4g> <bit> <tag> [<gate dir>]
# Boots the bitstream fresh (tools/board-gate.sh, no stress), starts tools/board/gb5-run.sh <tag> with
# ONE ssh, then watches only its files over NFS (/srv/ubuntu-root-<board>/var/tmp): no ssh, no logger,
# nothing else on the board while it runs -- a login a minute cost 3.4% of every subtest's time
# (2026-10-30). Prints the result and claim URLs and the perf counts; exits 2 if Geekbench prints
# nothing for 25 minutes (its longest subtest is about 13 silent). IP=<address> overrides the board's.
set -u
B=${1:?usage: tools/gb5-measure.sh <2g|4g> <bit> <tag> [<gate dir>]}; BIT=${2:?bit}; TAG=${3:?tag}
here=$(cd "$(dirname "$0")/.." && pwd)
case $B in 2g|4g) ;; *) echo "board: 2g or 4g"; exit 2;; esac
MAIN=$(git -C "$here" worktree list | head -1 | awk '{print $1}'); BOARD=$B . "$here/tools/boards.sh"
V=/srv/ubuntu-root-$B/var/tmp; G=$(realpath -m "${4:-/var/tmp/gb5-measure-$B-$TAG}"); BIT=$(realpath "$BIT")
O="-o ControlMaster=no -o BatchMode=yes -o ConnectTimeout=10"
mkdir -p "$G"
(cd "$here" && BOARD=$B STRESS_S=0 LAT=0 BIT=$BIT tools/board-gate.sh "$G" > "$G/gate.log" 2>&1)
grep -a -E 'BOARD:|login:' "$G/gate.log" | tail -2
grep -q 'BOARD: PASS' "$G/gate.log" || { echo "gb5-measure: the board gate failed ($G/gate.log)"; exit 1; }
# the address its boot just took (DHCP), off its own console, as the board gate reads it
ip=${IP:-$(tr -d '\r' < "$CON" | grep -aoE 'ipaddr=192\.168\.1\.[0-9]+' | tail -1 | cut -d= -f2)}
[ -n "$ip" ] || { echo "gb5-measure: no address on $B's console ($CON)"; exit 1; }
sleep 60
timeout 30 ssh $O tommy@$ip "nohup setsid sh /var/tmp/gb5-run.sh $TAG > /var/tmp/gb5-$TAG.nohup 2>&1 < /dev/null & sleep 2; true" \
   < /dev/null 2>&1 | grep -v bashrc
start=$(date +%s); echo "gb5-measure: $TAG launched on $B $(date +%H:%M)"
while true; do
   if grep -q "claim?key=" "$V/gb5-$TAG-raw.log" 2>/dev/null && grep -q 'gb5 full end' "$V/gb5-$TAG-raw.log"; then
      echo "gb5-measure: $TAG done after $(( ($(date +%s) - start) / 60 )) min"
      grep -a "browser.geekbench.com" "$V/gb5-$TAG-raw.log"
      grep -a -E 'cycles|instructions|seconds' "$V/gb5-$TAG-perf.txt"; exit 0
   fi
   f=$V/gb5-$TAG-ts.log; [ -f "$f" ] || f=$V/gb5-$TAG.nohup
   age=$(( $(date +%s) - $(stat -c %Y "$f" 2>/dev/null || echo "$start") ))
   if [ $age -gt 1500 ] && [ $(( $(date +%s) - start )) -gt 1800 ]; then
      echo "gb5-measure: $TAG printed nothing for $(( age / 60 )) min ($(date +%H:%M)); last: $(tail -1 "$f")"; exit 2
   fi
   sleep 60
done
