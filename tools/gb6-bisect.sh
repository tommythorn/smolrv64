#!/usr/bin/env bash
# One Geekbench 6 bisection step on the board, end to end, with ONE verdict line at the end:
#
#   tools/gb6-bisect.sh <bitstream> <tag> [PASS_S]
#
#   GB6-BISECT: FAIL <tag> rtl=<banner> after <s> s at subtest <n> (<name>) -- <reason>
#   GB6-BISECT: PASS <tag> rtl=<banner> survived <PASS_S> s (<n> subtests)
#
# Programs the bitstream (board-gate.sh, no GB5 stress), boots Ubuntu to login:, launches GB6
# single-core under perf on the board (/var/tmp/gb6-bisect.sh <tag>), then watches dmesg, the
# raw log and the serial console once a minute. FAIL on the first fault line, on geekbench
# exiting early, or on the board going unreachable for 5 minutes; PASS when the run survives
# PASS_S seconds, at which point geekbench is stopped (the child, so perf keeps its counters).
# PASS_S defaults to 16200 s = 4.5 h, three times the 87-minute failure this bisection chases
# (2026-09-20: 390d5028+ dies in PDF Renderer; GB5 completes on the same bitstream).
# The board should be shut down first; the script programs it regardless.
set -u
cd "$(dirname "$0")/.."
REPO=$(pwd); UB=$REPO/workloads/ubuntu
[ -f "$UB/screenlog.0" ] || UB=$(git -C "$REPO" worktree list | head -1 | awk '{print $1}')/workloads/ubuntu
BIT=${1:?bitstream}; TAG=${2:?tag}; PASS_S=${3:-28800}; PASS_N=${PASS_N:-7}
# PASS is by SUBTEST COUNT, not time: PASS_N=7 means Text Processing has started, i.e. six
# subtests completed, two past the PDF Renderer that kills 390d5028+. Subtest times vary 3x
# between trees (c3s3iw3 spends 26% of its cycles in FE_IC and takes >55 min on File
# Compression alone; the C4a1 tree averaged ~20 min), so a time budget would have passed a
# run that never reached the failing point. PASS_S is only the hard cap (8 h): reaching it
# with fewer than PASS_N subtests is INCONCLUSIVE, not a pass.
# WATCH=1 START=HH:MM:SS: neither program nor launch -- attach the watcher to a GB6 already
# running (a watcher that had to be replaced); START is the launch time on this host.
RES=$REPO/gate-results/gb6-bisect-$TAG; mkdir -p "$RES"
FAULT='Oops|BUG:|Unable to handle|unhandled signal|segfault|cause:|is not a head|NETDEV WATCHDOG|Kernel panic'
BAD='Kernel panic|Oops \[#|Unable to handle kernel paging|unhandled signal|segfault|SIGSEGV|status=11/SEGV|core dumped|is not a head|NETDEV WATCHDOG|nfs: server .* not responding'
say() { echo "[$(date +%H:%M:%S)] $*"; }

# ATTACH=1: the bitstream is already programmed and booting (a gate run that was lost);
# skip program+boot, take the banner from the existing gate.log, and wait for ssh instead.
if [ -n "${WATCH:-}" ]; then
   BANNER=$(grep -ao 'banner: rtl=[0-9a-f]*+\?' "$RES/gate.log" 2>/dev/null | head -1 | cut -d= -f2)
   say "watching the GB6 already running as $TAG (rtl=${BANNER:-?}, started ${START:?START=HH:MM:SS})"; SSH_TRIES=6
elif [ -n "${ATTACH:-}" ]; then
   BANNER=$(grep -ao 'banner: rtl=[0-9a-f]*+\?' "$RES/gate.log" 2>/dev/null | head -1 | cut -d= -f2)
   say "attaching to the running boot of $TAG (rtl=${BANNER:-?})"; SSH_TRIES=150
else
   say "programming $BIT as $TAG"
   BIT=$BIT STRESS_S=0 "$REPO/tools/board-gate.sh" "$RES" > "$RES/gate.log" 2>&1
   BANNER=$(grep -ao 'banner: rtl=[0-9a-f]*+\?' "$RES/gate.log" | head -1 | cut -d= -f2)
   grep -q "^BOARD: PASS" "$RES/gate.log" || { cat "$RES/gate.log" | tail -6; echo "GB6-BISECT: FAIL $TAG rtl=${BANNER:-?} after 0 s at subtest 0 (boot) -- $(grep -a '^BOARD:' "$RES/gate.log" | tail -1)"; exit 1; }
   say "booted rtl=$BANNER ($(grep -a '^login:' "$RES/gate.log"))"; SSH_TRIES=40
fi

ip=""
for i in $(seq 1 $SSH_TRIES); do
   ip=$(ss -tan 2>/dev/null | awk '$1=="ESTAB" && $4 ~ /:2049$/ {print $5}' | grep -o '192\.168\.1\.[0-9]*' | sort -u | head -1)
   [ -n "$ip" ] && timeout 15 ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=no tommy@$ip true 2>/dev/null && break
   ip=""; sleep 10
done
[ -n "$ip" ] || { echo "GB6-BISECT: FAIL $TAG rtl=$BANNER after 0 s at subtest 0 (ssh) -- no ssh within $((SSH_TRIES*10)) s"; exit 1; }
SSH="ssh -o BatchMode=yes -o ConnectTimeout=10 -o StrictHostKeyChecking=no tommy@$ip"
PRE_D=$(timeout 20 $SSH 'dmesg | wc -l'); PRE_C=$(wc -c < "$UB/screenlog.0")
if [ -n "${WATCH:-}" ]; then
   T0=$(date -d "today $START" +%s); PRE_D=207
else
   timeout 30 $SSH "sudo sh -c 'nohup sh /var/tmp/gb6-bisect.sh $TAG >/dev/null 2>&1 &'" || { echo "GB6-BISECT: FAIL $TAG rtl=$BANNER after 0 s at subtest 0 (launch) -- could not start gb6"; exit 1; }
   T0=$(date +%s)
fi
say "gb6 on $ip (dmesg lines before: $PRE_D), PASS at $PASS_N subtests, cap ${PASS_S}s"
RAW=/var/tmp/gb6-$TAG-raw.log
dead=0; last_say=0; n=0; name="-"
while :; do
   sleep 60
   el=$(( $(date +%s) - T0 ))
   st=$(timeout 40 $SSH "dmesg | tail -n +$((PRE_D+1)) | grep -cE '$FAULT'; grep -c '^  Running' $RAW 2>/dev/null; grep -E '^  Running' $RAW 2>/dev/null | tail -1; grep -aE 'Segmentation|gb6 end|Illegal|Bus error|Aborted' $RAW 2>/dev/null | head -1" 2>/dev/null)
   if [ -z "$st" ]; then
      dead=$((dead+1))
      con=$(tail -c +$((PRE_C+1)) "$UB/screenlog.0" | tr -d '\r' | grep -aE "$BAD" | head -2)
      if [ -n "$con" ] || [ "$dead" -ge 5 ]; then
         echo "$con"
         echo "GB6-BISECT: FAIL $TAG rtl=$BANNER after $el s at subtest $n ($name) -- board unreachable ${dead} min${con:+, console: $(echo "$con" | head -1)}"
         exit 1
      fi
      continue
   fi
   dead=0
   f=$(echo "$st" | sed -n 1p); n=$(echo "$st" | sed -n 2p); name=$(echo "$st" | sed -n 3p | sed 's/^  Running //'); end=$(echo "$st" | sed -n 4p)
   con=$(tail -c +$((PRE_C+1)) "$UB/screenlog.0" | tr -d '\r' | grep -aE "$BAD" | head -1)
   # THE RUN'S NATURAL END IS A VERDICT TOO (2026-09-21, the D13 fix's proof run): `gb6 end rc=0`
   # with no fault and no console line is a completed test, single- and multi-core -- PASS, with
   # the raw log (the result URL and whatever scores the CLI prints) kept in $RES. PASS_N=999
   # turns the subtest-count pass off so the run goes the distance.
   if [ -n "$end" ] && echo "$end" | grep -q 'gb6 end.*rc=0' && [ "${f:-0}" -eq 0 ] && [ -z "$con" ]; then
      cp "/srv/ubuntu-root$RAW" "$RES/" 2>/dev/null; cp "/srv/ubuntu-root/var/tmp/gb6-$TAG-perf.txt" "$RES/" 2>/dev/null
      "$REPO/tools/errlog-read.sh" "$ip" | tee "$RES/errlog-final.log"
      grep -aE 'Score|score|https://browser.geekbench.com|Upload' "/srv/ubuntu-root$RAW" | tee "$RES/score.log"
      echo "GB6-BISECT: PASS $TAG rtl=$BANNER COMPLETED in $el s ($n subtests, last: $name) -- $(grep -aoE 'https://browser.geekbench.com/[^ ]*' "/srv/ubuntu-root$RAW" | tail -1)"
      exit 0
   fi
   if [ "${f:-0}" -gt 0 ] || [ -n "$end" ] || [ -n "$con" ]; then
      timeout 40 $SSH "dmesg | tail -n +$((PRE_D+1)) | grep -aE '$FAULT|epc :|badaddr' | head -6; cp $RAW /var/tmp/gb6-$TAG-perf.txt /tmp/ 2>/dev/null" | tee "$RES/fault.log"
      cp "/srv/ubuntu-root$RAW" "$RES/" 2>/dev/null
      "$REPO/tools/errlog-read.sh" "$ip" | tee -a "$RES/fault.log"     # the integrity log, if this bitstream has one
      echo "GB6-BISECT: FAIL $TAG rtl=$BANNER after $el s at subtest $n ($name) -- ${end:-dmesg: $f fault lines}${con:+; console: $con}"
      exit 1
   fi
   if [ "${n:-0}" -ge "$PASS_N" ] || [ "$el" -ge "$PASS_S" ]; then
      timeout 40 $SSH "pkill -TERM -x geekbench_riscv; sleep 5; pgrep -x geekbench_riscv >/dev/null && pkill -KILL -x geekbench_riscv; true"
      sleep 3; cp "/srv/ubuntu-root$RAW" "/srv/ubuntu-root/var/tmp/gb6-$TAG-perf.txt" "$RES/" 2>/dev/null
      "$REPO/tools/errlog-read.sh" "$ip" | tee "$RES/errlog.log"
      if [ "${n:-0}" -ge "$PASS_N" ]; then echo "GB6-BISECT: PASS $TAG rtl=$BANNER survived $el s ($n subtests, last: $name)"; exit 0; fi
      echo "GB6-BISECT: INCONCLUSIVE $TAG rtl=$BANNER hit the ${PASS_S}s cap at $n subtests ($name)"; exit 2
   fi
   if [ $((el - last_say)) -ge 600 ]; then say "running: ${el}s, $n subtests, in $name"; last_say=$el; fi
done
