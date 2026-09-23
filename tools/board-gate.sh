#!/usr/bin/env bash
# The board half of tools/gate.sh, runnable alone: program a bitstream, NFS-boot Ubuntu, judge.
#
#   tools/board-gate.sh <resultdir>                    # impl_1's bitstream
#   BIT=/var/tmp/x.bit tools/board-gate.sh <resultdir> # a banked one
#   BOOT_WAIT=1500 ...                                 # seconds to allow for login: (default 1800)
#   REPLAY=<byte> tools/board-gate.sh <resultdir>       # judge an old boot from that console offset, no board
#   STRESS_S=900 ...                                  # seconds of post-login userspace stress (0 = skip)
#   STRESS_ONLY=1 tools/board-gate.sh <resultdir>        # no program/boot: run the stress on the board as it is
#
# PASS = `login:` reached with ZERO faults, AND (since 2026-09-17) a bounded USERSPACE STRESS
# over ssh (Geekbench 5 for STRESS_S seconds) that produces no fault in dmesg or on the console:
# the committed IW=2 stack passed at login and crashed GB5 6.9 h later (an instruction page
# fault at kernel text), so a login-only PASS is not a verdict on the core. FAIL fast on the first line that already decides
# it: a kernel panic or Oops, a userspace fault, or virtio dying ("id N is not a head!",
# NETDEV WATCHDOG, `nfs: server not responding`) -- the first two sat unrecognised for ten
# minutes per boot on 2026-09-04 while the gate waited for a login that could not come, and
# the NFS stall is how a dying NIC looks before the watchdog. The console is cumulative across
# boots AND the previous kernel keeps printing while the board is reprogrammed, so the verdict
# is read from THIS boot's own kernel marker (`riscv: base ISA extensions`) onward: judging
# everything past the pre-program offset produced false FAILs and one false PASS on 2026-09-17
# (the old kernel's watchdog lines, the old kernel's login prompt). The monitor's `rtl=` line
# is captured so the log proves WHICH RTL booted.
set -u
cd "$(dirname "$0")/.."
REPO=$(pwd); PLAT=$REPO/platforms/rk-xcku5p-f-v1.2; UB=$REPO/workloads/ubuntu
# The serial console (`screen -L`, screenlog.0) lives in ONE checkout's workloads/ubuntu --
# the main one. A gate run from a worktree programs the board and then cannot watch it boot
# (gate V4, 2026-09-05: "BOARD: FAIL (upload)" with the bitstream already on the board), so
# fall back to the main worktree's copy, which `git worktree list` prints first.
[ -f "$UB/screenlog.0" ] || UB=$(git -C "$REPO" worktree list | head -1 | awk '{print $1}')/workloads/ubuntu
RES=${1:?result dir}; BOOT_WAIT=${BOOT_WAIT:-1800}; mkdir -p "$RES"
BAD='Kernel panic|Oops \[#|Unable to handle kernel paging|unhandled signal|segfault|SIGSEGV|status=11/SEGV|core dumped|is not a head|NETDEV WATCHDOG|nfs: server .* not responding'
MARK='riscv: base ISA extensions'

# ---- the post-login userspace stress (B10) --------------------------------------------------
# The board is the NFS peer of this host on 192.168.1.x (see the memory note: its address is DHCP);
# sshd comes up a few minutes after login:. Geekbench runs its subtests in order under a timeout
# (124 = the budget ended, the expected exit); the verdict is the exit status plus every fault
# line dmesg and the console gained during the run.
stress() {
   [ "${STRESS_S:-900}" -gt 0 ] || { echo "stress: skipped (STRESS_S=0)"; return 0; }
   local i rc pre_d pre_c faults; ip=""            # ip stays visible: the integrity log is read after
   for i in $(seq 1 40); do
      ip=$(ss -tan 2>/dev/null | awk '$1=="ESTAB" && $4 ~ /:2049$/ {print $5}' | grep -o '192\.168\.1\.[0-9]*' | sort -u | head -1)
      [ -n "$ip" ] && timeout 15 ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=no tommy@$ip true 2>/dev/null && break
      sleep 10
   done
   [ -n "$ip" ] || { echo "BOARD: FAIL (stress: no ssh to the board within 400 s)"; return 1; }
   pre_d=$(timeout 20 ssh -o BatchMode=yes tommy@$ip 'dmesg | wc -l' 2>/dev/null); pre_c=$(wc -c < "$UB/screenlog.0")
   echo "stress: $ip, ${STRESS_S:-900}s of Geekbench (dmesg lines before: $pre_d)"
   timeout $(( ${STRESS_S:-900} + 120 )) ssh -o BatchMode=yes -o ServerAliveInterval=30 tommy@$ip \
      "cd ~/Geekbench-5.4.1-LinuxRISCVPreview && timeout ${STRESS_S:-900} ./geekbench_riscv64 > /var/tmp/gate-stress.log 2>&1; echo RC=\$?; dmesg | tail -n +$((pre_d+1)) | grep -E 'Oops|BUG:|Unable to handle|unhandled signal|segfault|cause:|is not a head|NETDEV WATCHDOG'" > "$RES/stress.log" 2>&1
   rc=$(grep -o 'RC=[0-9]*' "$RES/stress.log" | tail -1 | cut -d= -f2)
   faults=$(( $(grep -cE 'Oops|BUG:|Unable to handle|unhandled signal|segfault|cause:|is not a head|NETDEV WATCHDOG' "$RES/stress.log") ))
   faults=$(( faults + $(tail -c +$((pre_c+1)) "$UB/screenlog.0" | tr -d '\r' | grep -acE "$BAD") ))
   echo "stress: rc=${rc:-?} (124 = ran the whole budget) faults=$faults  subtests: $(timeout 20 ssh -o BatchMode=yes tommy@$ip 'grep -c "^  Running" /var/tmp/gate-stress.log' 2>/dev/null)"
   if [ "$faults" -eq 0 ] && { [ "${rc:-1}" = 124 ] || [ "${rc:-1}" = 0 ]; }; then errlog || return 1; return 0; fi
   grep -E 'Oops|BUG:|Unable to handle|unhandled signal|segfault|cause:|NETDEV' "$RES/stress.log" | head -4
   echo "BOARD: FAIL (stress: rc=${rc:-?} faults=$faults)"; return 1
}
# ---- the integrity log (rv_errlog; docs/OOO2-Spec.md) --------------------------------------
# The design's own invariants, latched in hardware and read back from Linux once the stress is
# over. A nonzero vector is a FAIL whatever the console says: a run can look clean and still
# have broken a rule the core relies on -- and this is the only gate that sees a violation
# whose consequence would have surfaced hours later. A bitstream without the log is reported
# as such, not as clean.
errlog() {
   local out; out=$("$REPO/tools/errlog-read.sh" "$ip" 2>&1); echo "$out"
   case "$out" in "errlog: FAULT"*) echo "BOARD: FAIL (integrity log)"; return 1;; esac
   return 0
}
if [ -n "${STRESS_ONLY:-}" ]; then RES=${1:?result dir}; mkdir -p "$RES"; stress && { echo "BOARD: PASS (stress only)"; exit 0; }; exit 1; fi

if [ -n "${REPLAY:-}" ]; then PRE=$REPLAY; BOOT_WAIT=0; echo "replay from byte $PRE"; else
PRE=$(wc -c < "$UB/screenlog.0")
( cd "$PLAT" && timeout 900 make program ${BIT:+BIT=$BIT} ) > "$RES/program.log" 2>&1
grep -qi "programmed successfully" "$RES/program.log" || { echo "BOARD: FAIL (program)"; tail -5 "$RES/program.log"; exit 1; }
echo "programmed ${BIT:-impl_1}"
# The monitor's banner names the bitstream just programmed; wait for it past the offset (the
# console log flushes late) and hand it to ubuntu-boot.sh, whose model-string stamp otherwise
# reads the PREVIOUS build's banner (W7's board turn, 2026-09-07: the DTB said 9b3050f9 on
# a 6c1eeff1 bitstream).
for i in $(seq 1 30); do
   RTL_BANNER=$(tail -c +$((PRE+1)) "$UB/screenlog.0" | tr -d '\r' | grep -aoE 'rtl=[0-9a-f]{7,12}\+?' | tail -1 | cut -d= -f2)
   [ -n "$RTL_BANNER" ] && break; sleep 2
done
echo "banner: rtl=${RTL_BANNER:-?}"
# A TRAILING '+' MEANS THE BITSTREAM IS NOT THAT COMMIT. The monitor appends it from the
# build-id's source-dirty word; this gate used to grep only [0-9a-f] and silently drop it,
# so a dirty build was reported -- and remembered -- as the commit it was built on top of.
# 2026-09-19: a GB5 crash was attributed to 493dc343 for exactly this reason; the bitstream
# was 493dc343 + uncommitted C4a step 2. Never record a dirty build as a commit's verdict.
case "${RTL_BANNER:-}" in
   *+) echo "WARNING: the loaded bitstream is a DIRTY tree (${RTL_BANNER}) -- its verdict belongs to no commit" ;;
esac
export RTL_BANNER
( cd "$UB" && touch ubuntu-nfs.dts.in && make dtbs >/dev/null 2>&1; timeout 3000 ./ubuntu-boot.sh ) > "$RES/upload.log" 2>&1 \
   || { echo "BOARD: FAIL (upload)"; tail -3 "$RES/upload.log"; exit 1; }
fi
echo "booting, watching past byte $PRE"
# The boot's own marker: its byte offset anchors everything below.
END=$(( $(date +%s) + BOOT_WAIT )); POS=""
while :; do
   OFF=$(tail -c +$((PRE+1)) "$UB/screenlog.0" | grep -abm1 "$MARK" | cut -d: -f1)
   [ -n "$OFF" ] && { POS=$((PRE+OFF)); break; }
   [ "$(date +%s)" -ge "$END" ] && break; sleep 10
done
[ -n "$POS" ] || { echo "BOARD: FAIL (no kernel marker past byte $PRE within ${BOOT_WAIT}s)"; exit 1; }
echo "kernel marker at byte $POS"
while [ "$(date +%s)" -lt "$END" ]; do
   NEW=$(tail -c +$((POS+1)) "$UB/screenlog.0" | tr -d '\r')
   printf '%s' "$NEW" | grep -aq "login:" && break
   printf '%s' "$NEW" | grep -aqE "$BAD" && break
   sleep 10
done
NEW=$(tail -c +$((POS+1)) "$UB/screenlog.0" | tr -d '\r')
# A replayed boot ends where the next programming's monitor banner begins.
[ -n "${REPLAY:-}" ] && NEW=$(printf '%s' "$NEW" | awk '/smolrv64 monitor/{exit} {print}')
printf '%s' "$NEW" > "$RES/boot.log"
RTL=$(tail -c +$((PRE+1)) "$UB/screenlog.0" | tr -d '\r' | grep -aoE 'rtl=[0-9a-f]+\+?' | head -1)   # the banner precedes the marker
FAULTS=$(printf '%s' "$NEW" | grep -acE "$BAD")
LOGIN=$(printf '%s' "$NEW" | grep -ac "login:")
echo "login: $LOGIN   faults: $FAULTS   ${RTL:-rtl=?}   last: $(printf '%s' "$NEW" | grep -aoE '^\[ *[0-9]+\.[0-9]+\]' | tail -1)"
if [ "$LOGIN" -ge 1 ] && [ "$FAULTS" -eq 0 ]; then
   if [ -n "${REPLAY:-}" ]; then echo "BOARD: PASS"; exit 0; fi
   stress || exit 1
   echo "BOARD: PASS"; exit 0
fi
printf '%s' "$NEW" | grep -aE "$BAD|epc :" | head -4
echo "BOARD: FAIL login=$LOGIN faults=$FAULTS"; exit 1
